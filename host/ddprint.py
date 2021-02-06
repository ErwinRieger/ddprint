#!/usr/bin/python
# -*- coding: utf-8 -*-
#/*
# This file is part of ddprint - a direct drive 3D printer firmware.
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

import logging, pprint, sys
import argparse, time, cProfile

logging.basicConfig(level=logging.DEBUG)

import ddprintutil as util, gcodeparser, packedvalue, ddhome
import ddtest

from ddplanner import Planner, initParser, initMatProfile
from ddprinter import Printer, RxTimeout

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

def main():

    argParser = argparse.ArgumentParser(description='%s, Direct Drive USB Print.' % sys.argv[0])

    argParser.add_argument("-b", dest="baud", action="store", type=int, help="Baudrate, default 500000.", default=500000)
    argParser.add_argument("-d", dest="device", action="store", type=str, help="Device to use, default: /dev/ttyACM0.", default="/dev/ttyACM0")

    argParser.add_argument("-t0", dest="t0", action="store", type=int, help="Temp 0 (heated bed), default comes from mat. profile.")
    argParser.add_argument("-t1", dest="t1", action="store", type=int, help="Temp 1 (hotend 1), default comes from mat. profile.")

    argParser.add_argument("-kadvance", dest="kadvance", action="store", type=float, help="K-Advance factor, default comes from mat. profile.")
    argParser.add_argument("-autotemp", dest="autotemp", action="store", type=int, help="Use autotemp algorithm, default is True.", default=1)

    argParser.add_argument("-startadvance", dest="startAdvance", action="store", type=float, help="Gradual advance: advance startvalue.")
    argParser.add_argument("-advincrease", dest="advIncrease", action="store", type=float, help="Gradual advance: increase kAdvance by advIncrease after each part.")

    argParser.add_argument("-smat", dest="smat", action="store", help="Name of specific material profile to use.")

    # fake endstops as long we have no real ones
    argParser.add_argument("-dt", dest="dummyTempTable", action="store", type=bool, help="Debug: download dummy temperature table, don't limit speeed.", default=False)
    argParser.add_argument("-F", dest="fakeendstop", action="store", type=bool, help="Debug: fake endstops", default=False)
    argParser.add_argument("-nc", dest="noCoolDown", action="store", type=bool, help="Debug: don't wait for heater cool down after print.", default=False)

    # testbatch: assume dummyTempTable, fakeendstop, noCoolDown, dont wait for print-done
    argParser.add_argument("-testbatch", action="store", type=bool, help="Debug: testbatch.", default=False)


    argParser.add_argument("-pidset", dest="pidset", action="store", type=str, help="Debug: Specify PID parameter sets to use (ZNCH).", default="ZNCH")

    argParser.add_argument("-fr", dest="feedrate", action="store", type=float, help="Feedrate for move commands.", default=0)

    argParser.add_argument("-inctemp", dest="inctemp", action="store", type=int, help="Increase extruder temperature niveau (layer bonding).", default=0)

    # XXX Should we call this parameter strength, figurine-mode or parts-mode?
    argParser.add_argument("-wp", dest="workingPoint", action="store", type=float, help="xxx temp niveau between best/worst case SLE.", default=0.5)

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

    sp = subparsers.add_parser("measureTempFlowrateCurve2", help=u"Determine temperature/flowrate properties of filament.")
    sp.add_argument("nozzle", help="Name of nozzle profile to use [nozzle40, nozzle80...].")
    sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")
    sp.add_argument("gfile", help="Measurement GCode file.")

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

    sp = subparsers.add_parser("heathotend", help=u"Heat up hotend (to clean it, etc).")
    # sp.add_argument("printer", help="Name of printer profile to use.")
    sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")

    sp = subparsers.add_parser("getendstops", help=u"Get current endstop state.")

    sp = subparsers.add_parser("getfilsensor", help=u"Get current filament position.")

    sp = subparsers.add_parser("getfreemem", help=u"Get printer free memory.")

    sp = subparsers.add_parser("getpos", help=u"Get current printer and virtual position.")

    sp = subparsers.add_parser("getprintername", help=u"Get printer name from eeprom.")

    sp = subparsers.add_parser("getTemps", help=u"Get current temperatures (Bed, Extruder1, [Extruder2]).")

    sp = subparsers.add_parser("getTempTable", help=u"Get temperature-speed table from printer, print it to stdout and to /tmp/temptable_printer.txt.")

    sp = subparsers.add_parser("getstatus", help=u"Get current printer status.")

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

    sp = subparsers.add_parser("calibrateESteps", help=u"Debug: helper to determine the e-steps value.")
    # sp.add_argument("printer", help="Name of printer profile to use.")

    sp = subparsers.add_parser("calibratefilsensor", help=u"Debug: helper to determine the ratio of stepper to flowrate sensor.")
    # sp.add_argument("printer", help="Name of printer profile to use.")

    args = argParser.parse_args()

    
    if args.mode == 'autotune':

        printer = Printer()
        printer.commandInit(args)
        matProfile = initMatProfile(args, printer.getPrinterName())
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

            print "\n#"
            print "# MONITOR:", time.time()
            print "#"
            status = printer.getStatus()
            print "Status: "
            pprint.pprint(status)


            pos = printer.getPos()
            print "POS: "
            pprint.pprint(pos)

            counts = printer.getFilSensor()
            print "Filament pos:", counts

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

                printer.coolDown(HeaterEx1, wait=150, log=doLog)

                # Stop hotend fan
                printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(0)])

            else:
            """

            printer.checkStall(status)

            printer.stallwarn.lastSwap = status["Swap"]
            printer.stallwarn.lastSteps = status["StepBuffer"]
            printer.stallwarn.lastSDReader = status["SDReader"]

    elif args.mode == "print":

        if args.testbatch:
            args.fakeendstop = True
            args.noCoolDown = True
            args.autotemp = False

        (printer, parser, planner) = initParser(args, mode=args.mode)

        t0 = planner.matProfile.getBedTemp()
        t0Wait = min(t0, printer.printerProfile.getWeakPowerBedTemp())
        t1 = planner.matProfile.getHotendGoodTemp() + planner.l0TempIncrease

        if args.testbatch:
            t0 = 25
            t0Wait = 20
            t1 = 30

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
        matProfile = initMatProfile(args, printer.getPrinterName())
        util.heatHotend(args, matProfile, printer)

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

    elif args.mode == 'getprintername':

        printer = Printer()
        printer.initSerial(args.device, args.baud)
        print "Printer name: '%s'" % printer.getPrinterName()

    elif args.mode == 'getTemps':

        printer = Printer()
        printer.initSerial(args.device, args.baud)
        temps = printer.getTemps()
        print "temperatures: ", temps

    elif args.mode == 'getTempTable':

        printer = Printer()
        printer.commandInit(args)
        (baseTemp, tempTable) = printer.getTempTable()
        util.printTempTable(baseTemp, tempTable)

    elif args.mode == "getstatus":

        printer = Printer()
        printer.initSerial(args.device, args.baud)
        status = printer.getStatus()
        print "Status: "
        pprint.pprint(status)
        printer.ppStatus(status)

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

    elif args.mode == 'calibrateESteps':

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

    elif args.mode == 'test':

        printer = Printer()
        printer.commandInit(args)

        while True:
            pprint.pprint(printer.getFSReadings())

    else:
        print "Unknown/not implemented command: ", args.mode
        assert(0)

if __name__ == "__main__":

    main()
    # cProfile.run("main()", "ddprintstats.prof")






