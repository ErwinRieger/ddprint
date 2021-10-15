#!/usr/bin/python
# -*- coding: utf-8 -*-
#/*
# This file is part of ddprint - a 3D printer firmware.
# 
# Copyright 2015 erwin.rieger@ibrieger.de
# 
# ddprint is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# ddprint is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with ddprint.  If not, see <http://www.gnu.org/licenses/>.
#*/

import logging, pprint, sys, os
import argparse, time, cProfile

logging.basicConfig(level=logging.DEBUG)

import ddprintutil as util, gcodeparser, packedvalue, ddhome
import ddtest

from ddplanner import Planner, initParser, initMatProfile
from ddprinter import Printer, RxTimeout
from ddprofile import NozzleProfile, MatProfile

#
# USB packet format:
# --------------------
#
# * Startbyte (SOH, 0x81)
# * Command counter 8 bit 0
# * Commandbyte         buffered (print) commands (G1...), direct commands (Temp, start, stop)
#    + commands < 128: buffered commands
#    + commands >= 128: direct commands
# * Length of command packet 32 bits, this includes the length of the variable command payload.
#   Or length of sub block (1-512).
# * if length > 0: command specific parameters/data
# * checksum 16 bits
#
# Speicherung eines buffered commands:
#   - entferne startbye, commandcounter am anfang
#   - entferne checksum am ende
#   - speichere das kommando
#
# Move segment data, new with bresenham in firmware:
#   * Header data:
#       + flags 
#           Bit 7: flag bits 0-4 contain stepper direction bits for X, Y, Z, A, B
#           Bit 6: Accel step or raw timer values are compressd (one word + differences as bytes)
#           Bit 5: Decel step values are compressd (one word + differences as bytes)
#       + index lead axis, 8 bits
#       + array of absolute steps, 5 * 32 bits
#       + number of accel steps, naccel, 16 bits
#       + constant linear timer value, 16 bits
#       + number of deccel steps, ndeccel, 16 bits
#
#   * accel steps: naccel*(timer value(16bits))
#
#   * deccel steps: ndeccel*(timer value(16bits))
#
# Note: 16 bit is enough for 819mm/327mm/232mm (XY,Z,E) acceleration/linear/decceleration distances. 
#

#
# Firmware commands: 
#
from ddprintcommands import *

#
# Printer states:
#
from ddprintstates import *

#
# For float commandline arg range check
# todo: help (-h) funktioniert nicht mehr
#
class ArgRange(object):
    def __init__(self, start, end):
        self.start = start
        self.end = end
    def __eq__(self, other):
        return self.start <= other <= self.end
    def __repr__(self):
        return "%f:%f" % (self.start, self.end)

def main():

    argParser = argparse.ArgumentParser(description='%s, USB Print.' % sys.argv[0])

    argParser.add_argument("-b", dest="baud", action="store", type=int, help="Baudrate, default 1Mbaud.", default=1000000)
    defaultSerialDev = os.getenv("DDPRINTDEV") or os.getenv("dev") or "/dev/ttyACM0"
    argParser.add_argument("-d", dest="device", action="store", type=str, help="Device to use, default: %s." % defaultSerialDev, default=defaultSerialDev)

    argParser.add_argument("-t0", dest="t0", action="store", type=int, help="Temp 0 (heated bed), default comes from mat. profile.")
    argParser.add_argument("-t1", dest="t1", action="store", type=int, help="Temp 1 (hotend 1), default comes from mat. profile.")

    # todo: describe limit k-advance
    argParser.add_argument("-kadvance", dest="kadvance", action="store", type=float, choices=[ArgRange(0.0, 1.0)], help="K-Advance factor, default comes from mat. profile.")
    argParser.add_argument("-autotemp", dest="autotemp", action="store", type=int, help="Use autotemp algorithm, default is True.", default=1)

    argParser.add_argument("-startadvance", dest="startAdvance", action="store", type=float, help="Gradual advance: advance startvalue.")
    argParser.add_argument("-advincrease", dest="advIncrease", action="store", type=float, help="Gradual advance: increase kAdvance by advIncrease after each part.")
    argParser.add_argument("-advstepheight", dest="advStepHeight", action="store", type=int, help="Gradual advance: height of each step (number of layers).")

    argParser.add_argument("-smat", dest="smat", action="store", help="Name of specific material profile to use.")

    # fake endstops as long we have no real ones
    argParser.add_argument("-dt", dest="dummyTempTable", action="store", type=bool, help="Debug: download dummy temperature table, don't limit speeed.", default=False)
    argParser.add_argument("-F", dest="fakeendstop", action="store", type=bool, help="Debug: fake endstops", default=False)
    argParser.add_argument("-nc", dest="noCoolDown", action="store", type=bool, help="Debug: don't wait for heater cool down after print.", default=False)

    argParser.add_argument("-pidset", dest="pidset", action="store", type=str, help="Debug: Specify PID parameter sets to use (ZNCH).", default="ZNCH")

    argParser.add_argument("-fr", dest="feedrate", action="store", type=float, help="Feedrate for move commands.", default=0)

    argParser.add_argument("-inctemp", dest="inctemp", action="store", type=int, help="Increase extruder temperature niveau (layer bonding).", default=0)

    # XXX Should we call this parameter strength, figurine-mode or parts-mode?
    argParser.add_argument("-wp", dest="workingPoint", action="store", type=float, choices=[ArgRange(0.0, 1.0)], help="AutoTemp: Working Point in range [0.0:1.0] 0: strong parts (higher temp range), 1: figurine mode (lower temps).", default=0.5)

    subparsers = argParser.add_subparsers(dest="mode", help='Mode: mon(itor)|print|store|reset|pre(process).')

    sp = subparsers.add_parser("autotune", help=u"Measure open loop step resopnse of hotend. Used to autotune hotend PID values (open-loop).")
    sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")

    sp = subparsers.add_parser("bootbootloader", help=u"ARM/stm32: boot into bootloader mode for firmware download.")

    sp = subparsers.add_parser("mon", help=u"Monitor printer state.")

    sp = subparsers.add_parser("changenozzle", help=u"Heat hotend and change nozzle.")

    sp = subparsers.add_parser("print", help=u"Download and print file at once.")
    sp.add_argument("nozzle", help="Name of nozzle profile to use [nozzle40, nozzle80...].")
    sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")
    sp.add_argument("gfile", help="Input GCode file.")

    # sp = subparsers.add_parser("reset", help=u"Try to stop/reset printer.")

    sp = subparsers.add_parser("pre", help=u"Preprocess gcode, for debugging purpose.")
    sp.add_argument("printer", help="Name of printer profile to use.")
    sp.add_argument("nozzle", help="Name of nozzle profile to use [nozzle40, nozzle80...].")
    sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")
    sp.add_argument("gfile", help="Input GCode file.")

    sp = subparsers.add_parser("test", help=u"Debug: tests for debugging purpose.")

    sp = subparsers.add_parser("disablesteppers", help=u"Disable stepper current (this dis-homes the printer).")

    sp = subparsers.add_parser("home", help=u"Home the printer.")

    sp = subparsers.add_parser("measureTempFlowrateCurve", help=u"Determine temperature/flowrate properties of filament.")
    sp.add_argument("nozzle", help="Name of nozzle profile to use [nozzle40, nozzle80...].")
    sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")
    sp.add_argument("flowrate", action="store", help="Start-flowrate in mm³/s.", type=float)
    sp.add_argument("mingrip", action="store", help="Minimum feeder grip when to stop measurement [0.9...0.95].", type=float)

    sp = subparsers.add_parser("measureTempFlowrateCurve2", help=u"Determine temperature/flowrate properties of filament.")
    sp.add_argument("nozzle", help="Name of nozzle profile to use [nozzle40, nozzle80...].")
    sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")
    sp.add_argument("gfile", help="Measurement GCode file.")
    sp.add_argument("mingrip", action="store", help="Minimum feeder grip when to stop measurement [0.9...0.95].", type=float)

    sp = subparsers.add_parser("moverel", help=u"Debug: Move axis manually, relative coords.")
    sp.add_argument("axis", help="Axis (XYZAB).", type=str)
    sp.add_argument("distance", action="store", help="Move-distance (+/-) in mm.", type=float)

    sp = subparsers.add_parser("moveabs", help=u"Debug: Move axis manually, absolute coords.")
    sp.add_argument("axis", help="Axis (XYZAB).", type=str)
    sp.add_argument("distance", action="store", help="Move-distance (+/-) in mm.", type=float)

    sp = subparsers.add_parser("exec", help=u"Debug: exec gcode.")
    sp.add_argument("nozzle", help="Name of nozzle profile to use [nozzle40, nozzle80...].")
    sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")
    sp.add_argument("gcode", action="store", help="G-Code string.", type=str)

    sp = subparsers.add_parser("insertfilament", help=u"Insert filament (heatup, forward filament).")
    sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")

    sp = subparsers.add_parser("fanspeed", help=u"Debug: set hotend fanspeed.")
    sp.add_argument("fanspeed", type=int, help="Fanspeed to set [0..255].")

    sp = subparsers.add_parser("removefilament", help=u"Remove filament (heatup, retract filament).")
    sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")

    sp = subparsers.add_parser("readGpio", help=u"Debug: Set gpio pin to INPUT and read value.")
    sp.add_argument("pin", type=int, help="Pin number to read.")

    sp = subparsers.add_parser("readAnalogGpio", help=u"Debug: Set gpio pin to INPUT_ANALOG and read value.")
    sp.add_argument("pin", type=int, help="Pin number to read.")

    sp = subparsers.add_parser("reset", help=u"Emergency hard reset printer.")

    sp = subparsers.add_parser("bedleveling", help=u"Do bed leveling sequence.")
    sp.add_argument("-rl", dest="relevel", action="store", type=bool, help="Releveling, don't adjust bedlevel offset, just turn screws.", default=False)
    sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")

    sp = subparsers.add_parser("heathotend", help=u"Heat up hotend.")
    sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")

    sp = subparsers.add_parser("heatbed", help=u"Heat up bed.")
    sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")

    sp = subparsers.add_parser("getendstops", help=u"Get current endstop state.")

    sp = subparsers.add_parser("getfilsensor", help=u"Get current filament position.")

    sp = subparsers.add_parser("getfreemem", help=u"Get printer free memory.")

    sp = subparsers.add_parser("getpos", help=u"Get current printer and virtual position.")

    sp = subparsers.add_parser("getprintername", help=u"Get printer name from eeprom.")

    sp = subparsers.add_parser("getTemps", help=u"Get current temperatures (Bed, Extruder1, [Extruder2]).")

    sp = subparsers.add_parser("version", help=u"Get git version from firmware.")

    sp = subparsers.add_parser("getstatus", help=u"Get current printer status.")
    sp = subparsers.add_parser("stat", help=u"Same as getstatus.")

    sp = subparsers.add_parser("top", help=u"Get 'process stats' from printer.")

    sp = subparsers.add_parser("zRepeatability", help=u"Debug: Move Z to 10 random positions to test repeatability.")

    sp = subparsers.add_parser("setGpio", help=u"Set GPIO pin - dangerous.")
    sp.add_argument("pin", type=int, help="Pin number to set.")
    sp.add_argument("value", type=int, help="Value to set.")

    sp = subparsers.add_parser("setprintername", help=u"Store printer name in eeprom.")
    sp.add_argument("name", help="Printer name.")

    sp = subparsers.add_parser("stepResponse", help=u"Measure and plot stepResponse of hotend PID (closed-loop).")

    sp = subparsers.add_parser("stop", help=u"Stop print, cooldown, home, disable steppers.")

    sp = subparsers.add_parser("testFeederUniformity", help=u"Debug: check smoothness/roundness of feeder measurements.")

    sp = subparsers.add_parser("testFilSensor", help=u"Debug: move filament manually, output filament sensor measurement.")
    sp.add_argument("printer", help="Name of printer profile to use.")
    sp.add_argument("distance", action="store", help="Move-distance (+/-) in mm.", type=float)

    sp = subparsers.add_parser("calibrateesteps", help=u"Debug: helper to determine the e-steps value.")
    # sp.add_argument("printer", help="Name of printer profile to use.")

    sp = subparsers.add_parser("calibratefilsensor", help=u"Debug: helper to determine the ratio of stepper to flowrate sensor.")
    # sp.add_argument("printer", help="Name of printer profile to use.")

    sp = subparsers.add_parser("workingpos", help=u"Move to working pos for manual tasks.")

    args = argParser.parse_args()

    
    if args.mode == 'autotune':

        printer = Printer()
        printer.commandInit(args)
        matProfile = initMatProfile(args, printer, None)
        util.measureHotendStepResponse(args, printer, matProfile)

    elif args.mode == 'bootbootloader':

        printer = Printer()
        printer.initSerial(args.device, args.baud)
        printer.sendCommand(CmdBootBootloader)

    elif args.mode == 'changenozzle':

        # xxx todo create printer/profile singletons ...
        print "command %s currently disabled, exiting" % args.mode
        return
        util.changeNozzle(args, parser)

    elif args.mode == "mon":

        printer = Printer()
        printer.initSerial(args.device, args.baud)

        while True:

            print "=== mon ==="
            status = printer.getStatus()
            printer.printStatus(status)

            printer.top()

            pos = printer.getPos()
            print "*** Printer POS: %s ***" % str(pos)

            # counts = printer.getFilSensor()
            # print "*** Filament pos: %s ***" % str(counts)

            freeMem = printer.getFreeMem()
            print "*** Free memory: %d bytes ***" % freeMem

            try:
                (cmd, payload) = printer.readResponse()        
            except RxTimeout:
                pass
            else:
                print "Response cmd    :", cmd
                print "Response payload:", payload.encode("hex")
                printer.checkErrorResponse(cmd, payload, False)


            """
            # We monitor end-of print here,
            # 
            if not printer.stateMoving(status):

                print "print ended because of printer.stateMoving()"

                printer.coolDown(HeaterEx1)
                printer.coolDown(HeaterBed)

                ddhome.home(args, printer, parser, planner)

                printer.sendCommand(CmdDisableSteppers)

                printer.coolDown(HeaterEx1, wait=100, log=doLog)

                # Stop hotend fan
                printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(0)])

            else:
            """

            # printer.checkStall(status)
            # printer.stallwarn.lastSwap = status.Swap
            # printer.stallwarn.lastSteps = status.StepBuffer
            # printer.stallwarn.lastSDReader = status.SDReader

            time.sleep(1)

    elif args.mode == "print":

        # check for reconnect
        printer = Printer()
        printer.initSerial(args.device, args.baud)
        status = printer.getStatus()
        print "Status: "
        pprint.pprint(status)

        printer.initPrinterProfile(args)

        if "nozzle" in args:
            nozzle = NozzleProfile(args.nozzle)
        else:
            nozzle = None

        if status.state >= StateInit:

            print ""
            print "Reconnecting..."
            print ""
            # (printer, parser, planner) = initParser(args, mode=args.mode)

            # if "nozzle" in args:
                # nozzle = NozzleProfile(args.nozzle)
            # else:
                # nozzle = None

            # Create material profile singleton instance
            if "mat" in args:
                mat = initMatProfile(args, printer, nozzle)
            else:
                mat = None

            print "nozzle, mat:", nozzle, mat

            planner = Planner(args, nozzleProfile=nozzle, materialProfile=mat, printer=printer)
            planner.reconnect(status)

            parser = gcodeparser.UM2GcodeParser(planner)

            t0 = planner.matProfile.getBedTemp()
            t0Wait = min(t0, printer.printerProfile.getWeakPowerBedTemp())
            t1 = planner.matProfile.getHotendGoodTemp() + planner.l0TempIncrease

            util.printFile(args, printer, parser, planner, printer.gui,
                    args.gfile, t0, t0Wait, t1, doLog=True, reconnect=True)

            return
                
        # (printer, parser, planner) = initParser(args, mode=args.mode)
        printer.commandInit(args) # reconn

        # Create material profile singleton instance
        if "mat" in args:
            mat = initMatProfile(args, printer, nozzle)
        else:
            mat = None

        print "nozzle, mat:", nozzle, mat

        planner = Planner(args, nozzleProfile=nozzle, materialProfile=mat, printer=printer)
        parser = gcodeparser.UM2GcodeParser(planner)

        t0 = planner.matProfile.getBedTemp()
        t0Wait = min(t0, printer.printerProfile.getWeakPowerBedTemp())
        t1 = planner.matProfile.getHotendGoodTemp() + planner.l0TempIncrease

        util.printFile(args, printer, parser, planner, printer.gui,
                args.gfile, t0, t0Wait, t1, doLog=True)

    elif args.mode == "pre":

        (printer, parser, planner) = initParser(args, mode=args.mode)

        # Virtuelle position des druckkopfes falls 'gehomed'
        homePosMM = planner.getHomePos()[0]
        parser.setPos(homePosMM)

        (f, preloadLines) = parser.preParse(args.gfile, args.baud)
        lineNr = 0
        for line in f:
            parser.execute_line(line)
            lineNr += 1

        print "Parsed %d gcode lines." % lineNr

        planner.finishMoves()

    elif args.mode == 'disablesteppers':

        printer = Printer()
        printer.initSerial(args.device, args.baud)
        printer.sendCommand(CmdDisableSteppers)

    elif args.mode == 'measureTempFlowrateCurve':

        args.pidset="TSTS"
        # args.pidset="CHCH"
        # args.pidset="ZNCH"
        (printer, parser, planner) = initParser(args, mode=args.mode)
        util.measureTempFlowrateCurve(args, printer, parser, planner)

    elif args.mode == 'measureTempFlowrateCurve2':

        # Disable linear advance
        args.kadvance = 0.0
        # Disable autotemp
        args.autotemp = False

        (printer, parser, planner) = initParser(args, mode=args.mode)
        util.measureTempFlowrateCurve2(args, printer, parser, planner)

    elif args.mode == 'moverel':

        assert(args.axis.upper() in "XYZAB")

        (printer, parser, planner) = initParser(args, mode=args.mode, travelMovesOnly=True)
        util.manualMove(args, printer, parser, planner, util.dimIndex[args.axis.upper()], args.distance, args.feedrate)

    elif args.mode == 'moveabs':

        assert(args.axis.upper() in "XYZAB")

        (printer, parser, planner) = initParser(args, mode=args.mode, travelMovesOnly=True)
        util.manualMove(args, printer, parser, planner, util.dimIndex[args.axis.upper()], args.distance, args.feedrate, True)

    elif args.mode == 'insertfilament':

        (printer, parser, planner) = initParser(args, mode=args.mode, travelMovesOnly=True)
        util.insertFilament(args, printer, parser, planner, args.feedrate)

    elif args.mode == 'fanspeed':

        # (printer, parser, planner) = initParser(args, mode=args.mode, travelMovesOnly=True)
        printer = Printer()
        printer.commandInit(args)
        printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(args.fanspeed)])

    elif args.mode == 'removefilament':

        (printer, parser, planner) = initParser(args, mode=args.mode, travelMovesOnly=True)
        util.removeFilament(args, printer, parser, planner, args.feedrate)

    elif args.mode == 'readGpio':
        printer = Printer()
        printer.initSerial(args.device, args.baud)
        val = printer.readGpio(args.pin)
        print val

    elif args.mode == 'readAnalogGpio':
        printer = Printer()
        printer.initSerial(args.device, args.baud)
        val = printer.readAnalogGpio(args.pin)
        print val

    elif args.mode == 'reset':

        printer = Printer()
        printer.initSerial(args.device, args.baud)
        printer.systemReset()

    elif args.mode == 'bedleveling':

        (printer, parser, planner) = initParser(args, mode=args.mode, travelMovesOnly=True)
        util.bedLeveling(args, printer, parser, planner)

    elif args.mode == 'heathotend':

        printer = Printer()
        printer.commandInit(args)
        matProfile = initMatProfile(args, printer, None)
        util.heatHotend(args, matProfile, printer)

    elif args.mode == 'heatbed':

        printer = Printer()
        printer.commandInit(args)
        matProfile = initMatProfile(args, printer, None)
        util.heatBed(args, matProfile, printer)

    elif args.mode == 'getendstops':

        printer = Printer()
        printer.initSerial(args.device, args.baud)
        res = printer.getEndstops()
        print "Endstop state: ", res
    
    elif args.mode == 'getfilsensor':

        printer = Printer()
        printer.initSerial(args.device, args.baud)
        counts = printer.getFilSensor()
        print "Filament pos:", counts

    elif args.mode == 'getfreemem':

        printer = Printer()
        printer.initSerial(args.device, args.baud)
        freeMem = printer.getFreeMem()
        print "Free memory: %d bytes" % freeMem

    elif args.mode == "getpos":

        printer = Printer()
        printer.initPrinterProfile(args)
        planner = Planner(args, printer, travelMovesOnly=True)

        res = printer.getPos()

        steps_per_mm = printer.printerProfile.getStepsPerMMVector()

        curPosMM = util.MyPoint(
            X = res[0] / float(steps_per_mm[0]),
            Y = res[1] / float(steps_per_mm[1]),
            Z = res[2] / float(steps_per_mm[2]),
            A = res[3] / float(steps_per_mm[3]),
            # B = res[4] / float(steps_per_mm[4]),
            )

        (homePosMM, homePosStepped) = planner.getHomePos()

        print "Printer pos [steps]:", res
        print "Printer pos [mm]:", curPosMM
        print "Virtual home pos [mm]: ", homePosMM

    elif args.mode == "getprintername":

        printer = Printer()
        printer.initSerial(args.device, args.baud)
        print "Printer name: '%s'" % printer.getPrinterName(args)

    elif args.mode == "getTemps":

        printer = Printer()
        printer.initSerial(args.device, args.baud)
        temps = printer.getTemps()
        print "temperatures: ", temps

    elif args.mode == "version":

        printer = Printer()
        printer.initSerial(args.device, args.baud)
        print "Firmware version: '%s'" % printer.getPrinterVersion(args)

    elif args.mode == "getstatus" or args.mode == "stat":

        printer = Printer()
        printer.initSerial(args.device, args.baud)
        status = printer.getStatus()
        printer.printStatus(status)

    elif args.mode == "top":

        printer = Printer()
        printer.initSerial(args.device, args.baud)
        printer.top()

    elif args.mode == 'home':

        (printer, parser, planner) = initParser(args, mode=args.mode, travelMovesOnly=True)
        ddhome.home(args, printer, parser, planner)

    elif args.mode == 'todo zRepeatability':

        util.zRepeatability(parser)

    elif args.mode == 'stepResponse':

        printer = Printer()
        printer.commandInit(args)
        util.stepResponse(args, printer)

    elif args.mode == 'todo stop':

        printer.commandInit(args)
        util.stopMove(args, parser)

    elif args.mode == "setprintername":

        printer = Printer()
        printer.setPrinterName(args)

    elif args.mode == 'setGpio':
        printer = Printer()
        printer.initSerial(args.device, args.baud)
        val = printer.setGpio(args.pin, args.value)

    elif args.mode == 'todo testFeederUniformity':

        ddtest.testFeederUniformity(args, parser)

    elif args.mode == 'testFilSensor':

        printer = Printer()
        initPrinterProfile(args)
        planner = Planner(args, printer, travelMovesOnly=True)
        parser = gcodeparser.UM2GcodeParser(planner, logger=printer.gui, travelMovesOnly=True)

        ddtest.testFilSensor(args, printer, parser)

    elif args.mode == 'calibrateesteps':

        printer = Printer()
        printer.initPrinterProfile(args)
        planner = Planner(args, printer, travelMovesOnly=True)
        ddtest.calibrateESteps(args, printer, planner)

    elif args.mode == 'calibratefilsensor':

        printer = Printer()
        printer.initPrinterProfile(args)
        planner = Planner(args, printer, travelMovesOnly=True)
        ddtest.calibrateFilSensor(args, printer, planner)

    elif args.mode == "exec":

        ddtest.execGcode(args)

    elif args.mode == "workingpos":

        (printer, parser, planner) = initParser(args, mode=args.mode, travelMovesOnly=True)
        util.workingPos(args, printer, parser, planner)

    elif args.mode == "test":

        printer = Printer()
        printer.commandInit(args)

        # while True:
            # print "FSReadings:"
            # pprint.pprint(printer.getFSReadings())
            # time.sleep(1)

        # cal = printer.printerProfile.getFilSensorCalibration()
        # for (stepper, sensor) in printer.getFSReadings():
            # print "dSteps: %d, dSensor: %d, slip: %.2f" % (stepper, sensor, stepper*cal/sensor)

        #printer.erase(2048 * 250)
        printer.erase(0)

    else:
        print "Unknown/not implemented command: ", args.mode
        assert(0)

if __name__ == "__main__":

    main()
    # cProfile.run("main()", "ddprintstats.prof")






