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
import argparse, time

logging.basicConfig(level=logging.DEBUG)

import ddprintutil as util, gcodeparser, packedvalue, ddhome
import ddtest

from ddprofile import PrinterProfile, MatProfile, NozzleProfile
from ddplanner import Planner
from ddprinter import Printer, RxTimeout
from ddprintconstants import A_AXIS

#
# Firmware Retraction:
# --------------------
#
# G10, G11, end-of-print-retraction
#
# UM2 Microstepping:
# ------------------
#
# micorsteps = [16, 16, 8, 16]
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


def plotArrow(f, v, startv, color="blue"):

    f.write("set arrow from %f,%f,%f to %f,%f,%f linetype 3 linewidth 5 linecolor rgb \"%s\"\n" % (startv[0], startv[1], startv[2], startv[0]+v[0], startv[1]+v[1], startv[2]+v[2], color))

def getMaxKoord(v, maxkoord):
    for i in range(2):
        if v[i] > maxkoord[0]:
            maxkoord[0] = v[i]
    if v[2] > maxkoord[1]:
        maxkoord[1] = v[2]

def plotSpeedChange(v1, v2, jerk, diff, title="Velocity X/Y/Z", fn="/tmp/2v.gnuplot"):

    maxkoord = [0, 0]

    f = open(fn, "w")
    f.write("set xyplane 0\n")
    f.write("set grid\n")

    if jerk[0]:
        v = [jerk[0], 0, 0]
        getMaxKoord(v, maxkoord)
        plotArrow(f, v, v1, "green")

    if jerk[1]:
        v = [0, jerk[1], 0]
        getMaxKoord(v, maxkoord)
        plotArrow(f, v, v1, "green")

    if jerk[2]:
        v = [0, 0, jerk[2]]
        getMaxKoord(v, maxkoord)
        plotArrow(f, v, v1, "green")

    getMaxKoord(jerk, maxkoord)
    plotArrow(f, jerk, v1)

    getMaxKoord(diff, maxkoord)
    plotArrow(f, diff, v1, "purple")

    getMaxKoord(v1, maxkoord)
    getMaxKoord(v2, maxkoord)

    f.write("set xrange [0:%f]\n" % (maxkoord[0]*1.2))
    f.write("set yrange [0:%f]\n" % (maxkoord[0]*1.2))
    f.write("set zrange [0:%f]\n" % (maxkoord[1]*1.2))

    f.write("splot \"-\" using 1:2:3:4:5:6 with vectors filled head title \"%s\"\n" % title)

    f.write("0 0 0 ")
    for v in v1[:3]:
        f.write("%f " % v)
    f.write("\n")

    f.write("0 0 0 ")
    for v in v2[:3]:
        f.write("%f " % v)
    f.write("\n")

    f.write("e\n")
    f.close()

def plot2v(v1, v2, jerk, diff, title="Velocity X/Y/Z", fn="/tmp/2v.gnuplot"):

    maxkoord = [0, 0]

    f = open(fn, "w")
    f.write("set xyplane 0\n")
    f.write("set grid\n")

    if jerk[0]:
        v = [jerk[0], 0, 0]
        getMaxKoord(v, maxkoord)
        plotArrow(f, v, v1, "green")

    if jerk[1]:
        v = [0, jerk[1], 0]
        getMaxKoord(v, maxkoord)
        plotArrow(f, v, v1, "green")

    if jerk[2]:
        v = [0, 0, jerk[2]]
        getMaxKoord(v, maxkoord)
        plotArrow(f, v, v1, "green")

    getMaxKoord(jerk, maxkoord)
    plotArrow(f, jerk, v1)

    if diff:
        getMaxKoord(diff, maxkoord)
        plotArrow(f, diff, v1, "purple")

    getMaxKoord(v1, maxkoord)
    getMaxKoord(v2, maxkoord)

    f.write("set xrange [0:%f]\n" % (maxkoord[0]*1.2))
    f.write("set yrange [0:%f]\n" % (maxkoord[0]*1.2))
    f.write("set zrange [0:%f]\n" % (maxkoord[1]*1.2))

    f.write("splot \"-\" using 1:2:3:4:5:6 with vectors filled head title \"%s\"\n" % title)

    f.write("0 0 0 ")
    for v in v1[:3]:
        f.write("%f " % v)
    f.write("\n")

    f.write("0 0 0 ")
    for v in v2[:3]:
        f.write("%f " % v)
    f.write("\n")

    f.write("e\n")
    f.close()

# plot x/y/E
def plot2vE(v1, v2, jerk, diff):
    plot2v(v1[:2] + [v1[3]], v2[:2] + [v2[3]],jerk[:2] + [jerk[3]],diff[:2] + [diff[3]], title="Velocity X/Y/E")

def plot4v(nom1, nom2, v1, v2, jerk, diff, fn = "/tmp/4v.gnuplot"):

    maxx = (max(nom1[0], nom2[0]) + diff[0]) * 2
    maxy = (max(nom1[1], nom2[1]) + diff[1]) * 2

    f = open(fn, "w")
    f.write("set xyplane 0\n")
    f.write("set grid\n")
    f.write("set xrange [0:%f]\n" % max(maxx, maxy))
    f.write("set yrange [0:%f]\n" % max(maxx, maxy))
    f.write("set zrange [0:100]\n")

    if jerk[0]:
        plotArrow(f, [jerk[0], 0, 0], v1, "green")

    if jerk[1]:
        plotArrow(f, [0, jerk[1], 0], v1, "green")

    if jerk[2]:
        plotArrow(f, [0, 0, jerk[2]], v1, "green")

    plotArrow(f, jerk, v1)

    plotArrow(f, diff, v1, "purple")

    f.write("splot \"-\" using 1:2:3:4:5:6 with vectors filled head\n")


    f.write("0 0 0 ")
    for v in nom1[:3]:
        f.write("%f " % v)
    f.write("\n")


    for v in nom1[:3]:
        f.write("%f " % v)
    for i in range(3):
        v = v1[i]
        f.write("-%f " % (nom1[i] - v))
    f.write("\n")


    f.write("0 0 0 ")
    for v in nom2[:3]:
        f.write("%f " % v)
    f.write("\n")


    for v in nom2[:3]:
        f.write("%f " % v)
    for i in range(3):
        v = v2[i]
        f.write("-%f " % (nom2[i] - v))
    f.write("\n")

    f.write("e\n")
    f.close()

def initParser(args, mode=None, gui=None):

    # Create printer profile singleton instance
    # printerProfile = PrinterProfile(args.pp)
    # Create material profile singleton instance
    mat = MatProfile(args.mat, args.smat)

    # Overwrite settings from material profile with command line arguments:
    if args.t0:
        mat.override("bedTemp", args.t0)
    if args.t1:
        mat.override("hotendStartTemp", args.t1)

    nozzle = NozzleProfile(args.nozzle)

    # Create the Printer singleton instance
    printer = Printer(gui=gui)

    if args.mode == "pre":
        printerProfile = PrinterProfile("UM2")
    else:
        printer.initSerial(args.device, args.baud)
        printerProfile = PrinterProfile(printer.getPrinterName())

    # Overwrite settings from printer profile with command line arguments:
    if args.retractLength:
        printerProfile.override("RetractLength", args.retractLength)

    # Create the planner singleton instance
    planner = Planner(args, gui)

    # Create parser singleton instance
    parser = gcodeparser.UM2GcodeParser(gui)

    return (parser, planner, printer)

def main():

    argParser = argparse.ArgumentParser(description='%s, Direct Drive USB Print.' % sys.argv[0])

    argParser.add_argument("-b", dest="baud", action="store", type=int, help="Baudrate, default 500000.", default=500000)
    argParser.add_argument("-d", dest="device", action="store", type=str, help="Device to use, default: /dev/ttyACM0.", default="/dev/ttyACM0")

    # argParser.add_argument("-h")

    argParser.add_argument("-t0", dest="t0", action="store", type=int, help="Temp 0 (heated bed), default comes from mat. profile.")
    argParser.add_argument("-t1", dest="t1", action="store", type=int, help="Temp 1 (hotend 1), default comes from mat. profile.")

    argParser.add_argument("-kAdvance", dest="kAdvance", action="store", type=float, help="K-Advance factor, default comes from mat. profile.")

    argParser.add_argument("-startAdvance", dest="startAdvance", action="store", type=float, help="Gradual advance: advance startvalue.")
    argParser.add_argument("-advIncrease", dest="advIncrease", action="store", type=float, help="Gradual advance: increase kAdvance by advIncrease after each step.")
    argParser.add_argument("-advStepHeight", dest="advStepHeight", action="store", type=int, help="Gradual advance: height of each step (number of layers).")

    argParser.add_argument("-mat", dest="mat", action="store", help="Name of generic material profile to use [pla, abs...], default is pla.", default="pla_1.75mm")
    argParser.add_argument("-smat", dest="smat", action="store", help="Name of specific material profile to use.")
    argParser.add_argument("-noz", dest="nozzle", action="store", help="Name of nozzle profile to use [nozzle40, nozzle80...], default is nozzle40.", default="nozzle40")

    argParser.add_argument("-np", dest="noPrime", action="store_const", const=True, help="Debug: don't prime nozzle, to test extrusion-less moves.")

    # fake endstops as long we have no real ones
    argParser.add_argument("-F", dest="fakeendstop", action="store", type=bool, help="Debug: fake endstops", default=False)
    argParser.add_argument("-nc", dest="noCoolDown", action="store", type=bool, help="Debug: don't wait for heater cool down after print.", default=False)

    argParser.add_argument("-fr", dest="feedrate", action="store", type=float, help="Feedrate for move commands.", default=0)
    argParser.add_argument("-rl", dest="retractLength", action="store", type=float, help="Retraction length, default comes from printer profile.", default=0)

    argParser.add_argument("-inctemp", dest="inctemp", action="store", type=int, help="Increase extruder temperature niveau (layer bonding).", default=0)

    subparsers = argParser.add_subparsers(dest="mode", help='Mode: mon(itor)|print|store|reset|pre(process).')

    sp = subparsers.add_parser("autoTune", help=u"Autotune hotend PID values.")

    sp = subparsers.add_parser("binmon", help=u"Monitor serial printer interface (binary responses).")

    sp = subparsers.add_parser("changenozzle", help=u"Heat hotend and change nozzle.")

    sp = subparsers.add_parser("mon", help=u"Monitor serial printer interface (asci).")

    sp = subparsers.add_parser("print", help=u"Download and print file at once.")
    sp.add_argument("gfile", help="Input GCode file.")

    sp = subparsers.add_parser("setPrinterName", help=u"Store printer name into printer eeprom.")
    sp.add_argument("name", help="Printer name.")

    # sp = subparsers.add_parser("reset", help=u"Try to stop/reset printer.")

    sp = subparsers.add_parser("pre", help=u"Preprocess gcode, for debugging purpose.")
    sp.add_argument("gfile", help="Input GCode file.")

    sp = subparsers.add_parser("test", help=u"Debug: tests for debugging purpose.")

    sp = subparsers.add_parser("disableSteppers", help=u"Disable stepper current (this dis-homes the printer).")

    sp = subparsers.add_parser("home", help=u"Home the printer.")

    sp = subparsers.add_parser("measureTempFlowrateCurve", help=u"Determine temperature/flowrate properties of filament.")
    sp.add_argument("tstart", action="store", type=int, help="Start temperature.")
    sp.add_argument("tend", action="store", type=int, help="End temperature.")
    sp.add_argument("-tstep", action="store", type=int, help="Temperature step width.", default=2)

    sp = subparsers.add_parser("moverel", help=u"Debug: Move axis manually, relative coords.")
    sp.add_argument("axis", help="Axis (XYZAB).", type=str)
    sp.add_argument("distance", action="store", help="Move-distance (+/-) in mm.", type=float)

    sp = subparsers.add_parser("moveabs", help=u"Debug: Move axis manually, absolute coords.")
    sp.add_argument("axis", help="Axis (XYZAB).", type=str)
    sp.add_argument("distance", action="store", help="Move-distance (+/-) in mm.", type=float)

    sp = subparsers.add_parser("insertFilament", help=u"Insert filament (heatup, forward filament).")

    sp = subparsers.add_parser("removeFilament", help=u"Remove filament (heatup, retract filament).")

    sp = subparsers.add_parser("bedLeveling", help=u"Do bed leveling sequence.")

    sp = subparsers.add_parser("heatHotend", help=u"Heat up hotend (to clean it, etc).")

    sp = subparsers.add_parser("genTempTable", help=u"Generate extrusion rate limit table.")

    sp = subparsers.add_parser("getEndstops", help=u"Get current endstop state.")

    sp = subparsers.add_parser("getFilSensor", help=u"Get current filament position.")

    sp = subparsers.add_parser("getFreeMem", help=u"Get printer free memory.")

    sp = subparsers.add_parser("getpos", help=u"Get current printer and virtual position.")

    sp = subparsers.add_parser("getPrinterName", help=u"Get printer name from eeprom.")

    sp = subparsers.add_parser("getTemps", help=u"Get current temperatures (Bed, Extruder1, [Extruder2]).")

    sp = subparsers.add_parser("getTempTable", help=u"Get temperature-speed table from printer, print it to stdout and to /tmp/temptable_printer.txt.")

    sp = subparsers.add_parser("getStatus", help=u"Get current printer status.")

    sp = subparsers.add_parser("zRepeatability", help=u"Debug: Move Z to 10 random positions to test repeatability.")

    sp = subparsers.add_parser("stop", help=u"Stop print, cooldown, home, disable steppers.")

    sp = subparsers.add_parser("stepResponse", help=u"Measure and plot stepResponse of hotend PID.")

    sp = subparsers.add_parser("retract", help=u"Debug: Do the end-of-print retract manually after heating up.")

    sp = subparsers.add_parser("fanspeed", help=u"Set fan speed manually.")
    sp.add_argument("speed", help="Fanspeed 0 - 255.", type=int)

    sp = subparsers.add_parser("testFeederUniformity", help=u"Debug: check smoothness/roundness of feeder measurements.")

    sp = subparsers.add_parser("testFilSensor", help=u"Debug: move filament manually, output filament sensor measurement.")
    sp.add_argument("distance", action="store", help="Move-distance (+/-) in mm.", type=float)

    sp = subparsers.add_parser("calibrateFilSensor", help=u"Debug: helper to determine the ratio of stepper to flowrate sensor.")

    args = argParser.parse_args()

    print "args: ", args

    if args.mode == "setPrinterName":
        printer = Printer()
        printer.setPrinterName(args)
        return

    (parser, planner, printer) = initParser(args, mode=args.mode)

    steps_per_mm = PrinterProfile.getStepsPerMMVector()

    if args.mode == 'autoTune':

        util.measureHotendStepResponse(args, parser)

    elif args.mode == 'changenozzle':

        util.changeNozzle(args, parser)

    elif args.mode == "binmon":
        while True:
            try:
                (cmd, payload) = printer.readResponse()        
            except RxTimeout:
                pass
            else:
                print "Response cmd    :", cmd
                print "Response payload:", payload.encode("hex")
                printer.checkErrorResponse(cmd, payload, False)

    elif args.mode == 'print':

        util.commonInit(args, parser)

        t0 = MatProfile.getBedTemp()
        t1 = MatProfile.getHotendStartTemp() + planner.l0TempIncrease

        # Send heat up  command
        print "\nHeating bed (t0: %d)...\n" % t0
        printer.heatUp(HeaterBed, t0)
        # print "\nPre-Heating extruder...\n"
        # printer.heatUp(HeaterEx1, t1/2)

        f = parser.preParse(args.gfile)

        if not args.noPrime:
            util.prime(parser)

        lineNr = 0
        printStarted = False

        startTime = time.time()

        for line in f:
            parser.execute_line(line)

            #
            # Send more than one 512 byte block for dlprint
            #
            if lineNr > 1000 and (lineNr % 250) == 0:
                # check temp and start print

                if  not printStarted:

                    sleepTime = max(0, (startTime+30) - time.time())
                    print "waiting to fire hotend...", sleepTime
                    time.sleep( sleepTime )

                    print "\nPre-Heating extruder (t1: %d)...\n" % t1
                    printer.heatUp(HeaterEx1, t1/2)

                    print "\nHeating bed (t0: %d)...\n" % t0
                    printer.heatUp(HeaterBed, t0, t0)
                    print "\nHeating extruder (t1: %d)...\n" % t1
                    printer.heatUp(HeaterEx1, t1, t1-1)

                    # Send print command
                    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
                    printStarted = True

                else:

                    # Stop sending moves on error
                    status = printer.getStatus()
                    pprint.pprint(status)
                    if not printer.stateMoving(status):
                        break

            lineNr += 1

        print "Parsed %d gcode lines." % lineNr

        # 
        # Add a move to lift the nozzle end of print
        # 
        util.endOfPrintLift(parser)

        planner.finishMoves()
        printer.sendCommand(CmdEOT)

        # Start print if less than 1000 lines or temp not yet reached:
        if not printStarted:

            print "\nHeating bed (t0: %d)...\n" % t0
            printer.heatUp(HeaterBed, t0, t0)
            print "\nHeating extruder (t1: %d)...\n" % t1
            printer.heatUp(HeaterEx1, t1, t1-1)

            # Send print command
            printer.sendCommandParamV(CmdMove, [MoveTypeNormal])

        printer.waitForState(StateIdle)

        printer.coolDown(HeaterEx1)
        printer.coolDown(HeaterBed)

        ddhome.home(parser, args.fakeendstop)

        printer.sendCommand(CmdDisableSteppers)

        if not args.noCoolDown:
            printer.coolDown(HeaterEx1, wait=150)
            printer.coolDown(HeaterBed, wait=55)

        printer.readMore()

        # Exit simulator for profiling
        # printer.sendCommand(CmdExit)

    elif args.mode == "pre":

        # Virtuelle position des druckkopfes falls 'gehomed'
        homePosMM = planner.getHomePos()[0]
        parser.setPos(homePosMM)

        f = parser.preParse(args.gfile)
        lineNr = 0
        for line in f:
            parser.execute_line(line)
            lineNr += 1

        print "Parsed %d gcode lines." % lineNr

        planner.finishMoves()

    elif args.mode == "mon":
        while True:
            printer.readMore()

    elif args.mode == 'disableSteppers':

        printer.commandInit(args, PrinterProfile.getSettings())
        printer.sendCommand(CmdDisableSteppers)

    elif args.mode == 'measureTempFlowrateCurve':

        util.measureTempFlowrateCurve(args, parser)

    elif args.mode == 'moverel':

        assert(args.axis.upper() in "XYZAB")

        printer.commandInit(args, PrinterProfile.getSettings())
        axis = util.dimIndex[args.axis.upper()]
        util.manualMove(parser, axis, args.distance, args.feedrate)

    elif args.mode == 'moveabs':

        assert(args.axis.upper() in "XYZAB")

        printer.commandInit(args, PrinterProfile.getSettings())
        axis = util.dimIndex[args.axis.upper()]
        util.manualMove(parser, axis, args.distance, args.feedrate, True)

    elif args.mode == 'insertFilament':

        util.insertFilament(args, parser, args.feedrate)

    elif args.mode == 'removeFilament':

        util.removeFilament(args, parser, args.feedrate)

    elif args.mode == 'bedLeveling':

        util.bedLeveling(args, parser)

    elif args.mode == 'heatHotend':

        util.heatHotend(args, parser)

    elif args.mode == 'genTempTable':

        util.genTempTable(planner)

    elif args.mode == 'getEndstops':

        printer.commandInit(args, PrinterProfile.getSettings())
        res = printer.getEndstops()
        print "Endstop state: ", res
    
    elif args.mode == 'getFilSensor':

        printer.commandInit(args, PrinterProfile.getSettings())
        # TODO: hardcoded CPI
        print "hack: Using fixed CPI value of 12000"
        counts = printer.getFilSensor()
        res = 12000.0 / 25.4
        print "filament sensor resolution is %.2f / mm" % res
        mm = counts / res
        print "Filament pos:", printer.getFilSensor(), " in mm: %.2f" % mm

    elif args.mode == 'getFreeMem':

        printer.commandInit(args, PrinterProfile.getSettings())
        freeMem = printer.getFreeMem()
        print "Free memory: %d bytes" % freeMem

    elif args.mode == 'getpos':

        printer.commandInit(args, PrinterProfile.getSettings())

        res = printer.getPos()

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

    elif args.mode == 'getPrinterName':

        print "Printer name: '%s'" % printer.getPrinterName()

    elif args.mode == 'getTemps':

        printer.commandInit(args, PrinterProfile.getSettings())
        printer.getTemps()

    elif args.mode == 'getTempTable':

        printer.commandInit(args, PrinterProfile.getSettings())
        (baseTemp, tempTable) = printer.getTempTable()
        print "tempTable: ", pprint.pprint(tempTable)
        util.printTempTable(baseTemp, tempTable)

    elif args.mode == 'getStatus':

        printer.commandInit(args, PrinterProfile.getSettings())
        status = printer.getStatus()
        print "Status: "
        pprint.pprint(status)

    elif args.mode == 'home':

        printer.commandInit(args, PrinterProfile.getSettings())
        ddhome.home(parser, args.fakeendstop)

    elif args.mode == 'zRepeatability':

        util.zRepeatability(parser)

    elif args.mode == 'stepResponse':

        util.stepResponse(args, parser)

    elif args.mode == 'retract':

        util.retract(args, parser)

    elif args.mode == 'stop':

        printer.commandInit(args, PrinterProfile.getSettings())
        util.stopMove(args, parser)

    elif args.mode == 'fanspeed':

        printer.commandInit(args, PrinterProfile.getSettings())
        printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(args.speed)])

    elif args.mode == 'testFeederUniformity':
        ddtest.testFeederUniformity(args, parser)

    elif args.mode == 'testFilSensor':
        ddtest.testFilSensor(args, parser)

    elif args.mode == 'calibrateFilSensor':
        ddtest.calibrateFilSensor(args, parser)

    elif args.mode == 'test':

        printer.commandInit(args, PrinterProfile.getSettings())
        readings = printer.getFSReadings(10)
        print "Readings: "
        pprint.pprint(readings)

    else:
        print "Unknown command: ", args.mode
        assert(0)

if __name__ == "__main__":

    res = 0

    try:
        main()
    except:
        import traceback
        print "Exception: ", traceback.format_exc()
        res = 1

    sys.exit(res)










