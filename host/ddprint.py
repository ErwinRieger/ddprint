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
import ddtest, ddargs

from ddplanner import Planner, initParser, initMatProfile
from ddprinter import Printer, RxTimeout
from ddprofile import NozzleProfile
from ddprintconstants import dimNames

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

    argParser = argparse.ArgumentParser(description='%s - ddPrint CLI application.' % os.path.basename(sys.argv[0]))

    ddargs.addCommonArguments(argParser)

    argParser.add_argument("-fr", dest="feedrate", action="store", type=float, help="Feedrate for move commands.", default=0)
    argParser.add_argument("-nc", dest="noCoolDown", action="store", type=bool, help="Debug: don't wait for heater cool down after print.", default=False)

    subparsers = argParser.add_subparsers(dest="mode", help='Mode: mon(itor)|print|store|reset|pre(process).')

    sp = subparsers.add_parser("autotune", help="Measure open loop step resopnse of hotend. Used to autotune hotend PID values (open-loop).")
    # sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")
    ddargs.addMatArgument(sp)

    sp = subparsers.add_parser("bedleveling", help="Do bed leveling sequence.")
    sp.add_argument("-rl", dest="relevel", action="store", type=bool, help="Releveling, don't adjust bedlevel offset, just turn screws.", default=False)
    # sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")
    ddargs.addMatArgument(sp)

    sp = subparsers.add_parser("bootbootloader", help="ARM/stm32: boot into bootloader mode for firmware download.")

    sp = subparsers.add_parser("calibrateesteps", help="Debug: helper to determine the e-steps value.")

    sp = subparsers.add_parser("calibratefilsensor", help="Debug: helper to determine the ratio of stepper to flowrate sensor.")

    sp = subparsers.add_parser("changenozzle", help="Heat hotend and change nozzle.")

    sp = subparsers.add_parser("continuosmove", help="Debug: Run stepper, no endstop check, no acceleration, dangerous.")
    sp.add_argument("axis", help="Axis (XYZAB).", type=str, choices=["X", "Y", "Z", "A", "B"])

    sp = subparsers.add_parser("disablesteppers", help="Disable stepper current (this dis-homes the printer).")

    sp = subparsers.add_parser("exec", help="Debug: exec gcode.")
    # sp.add_argument("nozzle", help="Name of nozzle profile to use [nozzle40, nozzle80...].")
    ddargs.addNozzleArgument(sp)
    # sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")
    ddargs.addMatArgument(sp)
    sp.add_argument("gcode", action="store", help="G-Code string.", type=str)

    sp = subparsers.add_parser("fanspeed", help="Debug: set hotend fanspeed.")
    sp.add_argument("fanspeed", type=int, help="Fanspeed to set [0..255].")

    sp = subparsers.add_parser("getTemps", help="Get current temperatures (Bed, Extruder1, [Extruder2]).")

    sp = subparsers.add_parser("getendstops", help="Get current endstop state.")

    sp = subparsers.add_parser("getfilsensor", help="Get current filament position.")

    sp = subparsers.add_parser("getfreemem", help="Get printer free memory.")

    sp = subparsers.add_parser("getpos", help="Get current printer and virtual position.")

    sp = subparsers.add_parser("getprintername", help="Get printer name from eeprom.")

    sp = subparsers.add_parser("getstatus", help="Get current printer status.")

    sp = subparsers.add_parser("heatbed", help="Heat up bed.")
    # sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")
    ddargs.addMatArgument(sp)

    sp = subparsers.add_parser("heathotend", help="Heat up hotend.")
    # sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")
    ddargs.addMatArgument(sp)

    sp = subparsers.add_parser("home", help="Home the printer.")

    sp = subparsers.add_parser("insertfilament", help="Insert filament (heatup, forward filament).")
    # sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")
    ddargs.addMatArgument(sp)

    sp = subparsers.add_parser("measureTempFlowrateCurve", help="Determine temperature/flowrate properties of filament.")
    # sp.add_argument("nozzle", help="Name of nozzle profile to use [nozzle40, nozzle80...].")
    ddargs.addNozzleArgument(sp)
    # sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")
    ddargs.addMatArgument(sp)
    sp.add_argument("flowrate", action="store", help="Start-flowrate in mm³/s.", type=float)
    sp.add_argument("mingrip", action="store", help="Minimum feeder grip when to stop measurement [0.9...0.95].", type=float)

    sp = subparsers.add_parser("measureTempFlowrateCurve2", help="Determine temperature/flowrate properties of filament.")
    ddargs.addPrintArguments(sp)
    # sp.add_argument("nozzle", help="Name of nozzle profile to use [nozzle40, nozzle80...].")
    # sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")
    # sp.add_argument("gfile", help="Measurement GCode file.")
    sp.add_argument("mingrip", action="store", help="Minimum feeder grip when to stop measurement [0.9...0.95].", type=float)

    sp = subparsers.add_parser("mon", help="Monitor printer state.")

    sp = subparsers.add_parser("moveabs", help="Debug: Move axis manually, absolute coords.")
    sp.add_argument("axis", help="Axis (XYZAB).", type=str, choices=["X", "Y", "Z", "A", "B"])
    sp.add_argument("distance", action="store", help="Move-distance (+/-) in mm.", type=float)

    sp = subparsers.add_parser("moverel", help="Debug: Move axis manually, relative coords.")
    sp.add_argument("axis", help="Axis (XYZAB).", type=str, choices=["X", "Y", "Z", "A", "B"])
    sp.add_argument("distance", action="store", help="Move-distance (+/-) in mm.", type=float)

    sp = subparsers.add_parser("pre", help="Preprocess gcode, for debugging purpose.")
    # sp.add_argument("printer", help="Name of printer profile to use.")
    ddargs.addPrinterArgument(sp)
    ddargs.addPrintArguments(sp)
    # sp.add_argument("nozzle", help="Name of nozzle profile to use [nozzle40, nozzle80...].")
    # sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")
    # sp.add_argument("gfile", help="Input GCode file.")

    sp = subparsers.add_parser("print", help="Parse gcode file, download and print it.")
    ddargs.addPrintArguments(sp)
    # sp.add_argument("nozzle", help="Name of nozzle profile to use [nozzle40, nozzle80...].")
    # sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")
    # sp.add_argument("gfile", help="Input GCode file.")

    sp = subparsers.add_parser("readAnalogGpio", help="Debug: Set gpio pin to INPUT_ANALOG and read value.")
    sp.add_argument("pin", type=int, help="Pin number to read.")

    sp = subparsers.add_parser("readGpio", help="Debug: Set gpio pin to INPUT and read value.")
    sp.add_argument("pin", type=int, help="Pin number to read.")

    sp = subparsers.add_parser("removefilament", help="Remove filament (heatup, retract filament).")
    # sp.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")
    ddargs.addMatArgument(sp)

    sp = subparsers.add_parser("reset", help="Emergency hard reset printer.")

    sp = subparsers.add_parser("setGpio", help="Debug: Set GPIO pin - dangerous.")
    sp.add_argument("pin", type=int, help="Pin number to set.")
    sp.add_argument("value", type=int, help="Value to set.")

    sp = subparsers.add_parser("setprintername", help="Store printer name in eeprom.")
    sp.add_argument("name", help="Printer name.")

    sp = subparsers.add_parser("stat", help="Same as getstatus.")

    sp = subparsers.add_parser("stepResponse", help="Measure and plot stepResponse of hotend PID (closed-loop).")

    sp = subparsers.add_parser("stop", help="Stop print, cooldown, home, disable steppers.")

    sp = subparsers.add_parser("test", help="Debug: tests for debugging purpose.")

    sp = subparsers.add_parser("testFeederUniformity", help="Debug: check smoothness/roundness of feeder measurements.")

    sp = subparsers.add_parser("testFilSensor", help="Debug: move filament manually, output filament sensor measurement.")
    # sp.add_argument("printer", help="Name of printer profile to use.")
    ddargs.addPrinterArgument(sp)
    sp.add_argument("distance", action="store", help="Move-distance (+/-) in mm.", type=float)

    sp = subparsers.add_parser("top", help="Get 'process stats' from printer.")

    sp = subparsers.add_parser("version", help="Get git version from firmware.")

    sp = subparsers.add_parser("workingpos", help="Move to working pos for manual tasks.")

    sp = subparsers.add_parser("zRepeatability", help="Debug: Move Z to 10 random positions to test repeatability.")

    args = argParser.parse_args()

    # print "args:", args
    
    if args.mode == 'autotune':

        printer = Printer(args)
        printer.commandInit()
        matProfile = initMatProfile(args, printer, None)
        util.measureHotendStepResponse(args, printer, matProfile)

    elif args.mode == 'bootbootloader':

        printer = Printer(args)
        printer.initSerial(args.device)
        printer.sendCommand(CmdBootBootloader)

    elif args.mode == 'changenozzle':

        # xxx todo create printer/profile singletons ...
        print("command %s currently disabled, exiting" % args.mode)
        return
        util.changeNozzle(args, parser)

    elif args.mode == 'continuosmove':

        printer = Printer(args)
        printer.commandInit()
        util.continuosmove(args, printer)

    elif args.mode == "mon":

        printer = Printer(args)
        printer.initSerial(args.device)

        while True:

            print("=== mon ===")
            status = printer.getStatus()
            printer.printStatus(status)

            printer.top()

            pos = printer.getPos()
            print("*** Printer POS: %s ***" % str(pos))

            # counts = printer.getFilSensor()
            # print "*** Filament pos: %s ***" % str(counts)

            freeMem = printer.getFreeMem()
            print("*** Free memory: %d bytes ***" % freeMem)

            try:
                (cmd, payload) = printer.readResponse()        
            except RxTimeout:
                pass
            else:
                print("Response cmd    :", cmd)
                print("Response payload:", payload.encode("hex"))
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
        printer = Printer(args)
        printer.initSerial(args.device)
        status = printer.getStatus()
        print("Status: ")
        pprint.pprint(status)

        printer.initPrinterProfile()

        if "nozzle" in args:
            nozzle = NozzleProfile(args.nozzle)
        else:
            nozzle = None

        if status.state >= StateInit:

            print("")
            print("Reconnecting...")
            print("")
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

            # print("nozzle, mat:", nozzle, mat)

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
        printer.commandInit() # reconn

        # Create material profile singleton instance
        if "mat" in args:
            mat = initMatProfile(args, printer, nozzle)
        else:
            mat = None

        # print("nozzle, mat:", nozzle, mat)

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

        planner.setPrintMode(PrintModePrinting)

        (f, preloadLines) = parser.preParse(args.gfile)
        lineNr = 0
        for line in f:
            parser.execute_line(line)
            lineNr += 1

        print("Parsed %d gcode lines." % lineNr)

        planner.finishMoves()

    elif args.mode == 'disablesteppers':

        printer = Printer(args)
        printer.initSerial(args.device)
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

        (printer, parser, planner) = initParser(args, mode=args.mode, travelMovesOnly=True)
        util.manualMove(args, printer, parser, planner, util.dimIndex[args.axis.upper()], args.distance, args.feedrate)

    elif args.mode == 'moveabs':

        (printer, parser, planner) = initParser(args, mode=args.mode, travelMovesOnly=True)
        util.manualMove(args, printer, parser, planner, util.dimIndex[args.axis.upper()], args.distance, args.feedrate, True)

    elif args.mode == 'insertfilament':

        (printer, parser, planner) = initParser(args, mode=args.mode, travelMovesOnly=True)
        util.insertFilament(args, printer, parser, planner, args.feedrate)

    elif args.mode == 'fanspeed':

        printer = Printer(args)
        printer.initSerial(args.device)
        printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(args.fanspeed)])

    elif args.mode == 'removefilament':

        (printer, parser, planner) = initParser(args, mode=args.mode, travelMovesOnly=True)
        util.removeFilament(args, printer, parser, planner, args.feedrate)

    elif args.mode == 'readGpio':
        printer = Printer(args)
        printer.initSerial(args.device)
        val = printer.readGpio(args.pin)
        print(val)

    elif args.mode == 'readAnalogGpio':
        printer = Printer(args)
        printer.initSerial(args.device)
        val = printer.readAnalogGpio(args.pin)
        print(val)

    elif args.mode == 'reset':

        printer = Printer(args)
        printer.initSerial(args.device)
        printer.systemReset()

    elif args.mode == 'bedleveling':

        (printer, parser, planner) = initParser(args, mode=args.mode, travelMovesOnly=True)
        util.bedLeveling(args, printer, parser, planner)

    elif args.mode == 'heathotend':

        printer = Printer(args)
        printer.commandInit()
        matProfile = initMatProfile(args, printer, None)
        util.heatHotend(args, matProfile, printer)

    elif args.mode == 'heatbed':

        printer = Printer(args)
        printer.commandInit()
        matProfile = initMatProfile(args, printer, None)
        util.heatBed(args, matProfile, printer)

    elif args.mode == 'getendstops':

        printer = Printer(args)
        printer.initSerial(args.device)
        res = printer.getEndstops()

        print("Endstop state: ")
        for i in range(len(res)//2):
            state = res[i*2]
            pos = res[i*2+1]
            if i < 3:
                print(f"{i} {dimNames[i]}: state: {state}, pos: {pos}")
            else:
                print(f"{i}  : state: {state}, pos: {pos}")
    
    elif args.mode == 'getfilsensor':

        printer = Printer(args)
        printer.initSerial(args.device)
        counts = printer.getFilSensor()
        print("Filament pos:", counts)

    elif args.mode == 'getfreemem':

        printer = Printer(args)
        printer.initSerial(args.device)
        freeMem = printer.getFreeMem()
        print("Free memory: %d bytes" % freeMem)

    elif args.mode == "getpos":

        printer = Printer(args)
        printer.initPrinterProfile()
        planner = Planner(args, printer, travelMovesOnly=True)

        res = printer.getPos()

        steps_per_mm = printer.printerProfile.getStepsPerMMVectorI()

        curPosMM = util.MyPoint(
            X = res[0] / float(steps_per_mm[0]),
            Y = res[1] / float(steps_per_mm[1]),
            Z = res[2] / float(steps_per_mm[2]),
            A = res[3] / float(steps_per_mm[3]),
            # B = res[4] / float(steps_per_mm[4]),
            )

        (homePosMM, homePosStepped) = planner.getHomePos()

        print("Printer pos [steps]:", res)
        print("Printer pos [mm]:", curPosMM)
        print("Virtual home pos [mm]: ", homePosMM)

    elif args.mode == "getprintername":

        printer = Printer(args)
        printer.initSerial(args.device)
        print("Printer name: '%s'" % printer.getPrinterName())

    elif args.mode == "getTemps":

        printer = Printer(args)
        printer.initSerial(args.device)
        temps = printer.getTemps()
        print("temperatures: ", temps)

    elif args.mode == "version":

        printer = Printer(args)
        printer.initSerial(args.device)
        print("Firmware version: '%s'" % printer.getPrinterVersion())

    elif args.mode == "getstatus" or args.mode == "stat":

        printer = Printer(args)
        printer.initSerial(args.device)
        status = printer.getStatus()
        printer.printStatus(status)

    elif args.mode == "top":

        printer = Printer(args)
        printer.initSerial(args.device)
        printer.top()

    elif args.mode == 'home':

        (printer, parser, planner) = initParser(args, mode=args.mode, travelMovesOnly=True)
        ddhome.home(args, printer, parser, planner)

    elif args.mode == 'todo zRepeatability':

        util.zRepeatability(parser)

    elif args.mode == 'stepResponse':

        printer = Printer(args)
        printer.commandInit()
        util.stepResponse(args, printer)

    elif args.mode == 'todo stop':

        printer.commandInit()
        util.stopMove(args, parser)

    elif args.mode == "setprintername":

        printer = Printer(args)
        printer.initSerial(args.device)
        printer.setPrinterName()

    elif args.mode == 'setGpio':
        printer = Printer(args)
        printer.initSerial(args.device)
        val = printer.setGpio(args.pin, args.value)

    elif args.mode == 'todo testFeederUniformity':

        ddtest.testFeederUniformity(args, parser)

    elif args.mode == 'testFilSensor':

        printer = Printer(args)
        initPrinterProfile()
        planner = Planner(args, printer, travelMovesOnly=True)
        parser = gcodeparser.UM2GcodeParser(planner, logger=printer.gui, travelMovesOnly=True)

        ddtest.testFilSensor(args, printer, parser)

    elif args.mode == 'calibrateesteps':

        printer = Printer(args)
        printer.initPrinterProfile()
        planner = Planner(args, printer, travelMovesOnly=True)
        ddtest.calibrateESteps(args, printer, planner)

    elif args.mode == 'calibratefilsensor':

        printer = Printer(args)
        printer.initPrinterProfile()
        planner = Planner(args, printer, travelMovesOnly=True)
        ddtest.calibrateFilSensor(args, printer, planner)

    elif args.mode == "exec":

        ddtest.execGcode(args)

    elif args.mode == "workingpos":

        (printer, parser, planner) = initParser(args, mode=args.mode, travelMovesOnly=True)
        util.workingPos(args, printer, parser, planner)

    elif args.mode == "test":

        printer = Printer(args)
        printer.commandInit()

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
        print("Unknown/not implemented command: ", args.mode)
        assert(0)

if __name__ == "__main__":

    main()
    # cProfile.run("main()", "ddprintstats.prof")






