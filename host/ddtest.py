#!/usr/bin/python
# -*- coding: utf-8 -*-
#
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

#
# Some seldom used functions, needed only if the hardware of the printer is modified.
#

import ddhome, ddprintutil as util, time
from ddprofile import PrinterProfile
from ddprinter import Printer
from ddprintcommands import *
from ddprintstates import *
from ddprintconstants import A_AXIS

def testFilSensor(args, parser):

    planner = parser.planner
    printer = planner.printer

    feedrate = args.feedrate or 1

    printer.commandInit(args, PrinterProfile.getSettings())
    startPos = printer.getFilSensor()
    util.manualMove(parser, util.dimIndex['A'], args.distance, feedrate=feedrate)
    endPos = printer.getFilSensor()
    diff = endPos - startPos

    # diff ist in sensor-counts
    pcal = PrinterProfile.get().getFilSensorCalibration()
    steps_per_mm = PrinterProfile.getStepsPerMM(A_AXIS)

    print "steps_per_mm  E:", steps_per_mm

    cal = diff / (steps_per_mm * args.distance) 

    print "Commanded steps: ", (steps_per_mm * args.distance)
    print "Filament pos in counts old:", startPos, ", new:", endPos, "difference: %d counts" % diff
    print "Calibration value from profile: %f, measured ratio: %f" % (pcal, cal)

#
# Calibrate filament sensor, determine ratio between extruder steps and the value 
# measured by the flowrate sensor.
#
def calibrateFilSensor(args, parser):

    def writeDataSet(f, data):

        for tup in data:
            f.write("%f %f\n" % tup)
        f.write("E\n")

    planner = parser.planner
    printer = planner.printer

    maxFeedrate = args.feedrate or 15

    eAccel = PrinterProfile.getMaxAxisAcceleration()[A_AXIS]

    printer.commandInit(args, PrinterProfile.getSettings())

    ddhome.home(parser, args.fakeendstop)

    # Disable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])

    # Disable temp-flowrate limit
    util.downloadDummyTempTable(printer)

    # Set filament sensor calibration to 1
    printer.sendCommandParamV(CmdSetFilSensorCal, [packedvalue.float_t(1.0)])

    feedrate = 0.25
    startPos = parser.getPos()[A_AXIS]

    calFile = open("calibrateFilSensor.json", "w")
    calFile.write('{\n    "filSensorCalibration": [\n')

    calValues = []
    valueSum = 0

    RAVGWINDOW = 10 # xxx defined in filsensor.h

    while feedrate <= maxFeedrate:

        # tStartup = util.getStartupTime(feedrate)

        # xxx 0.25 hardcoded in fw!
        sstartup = 5 * (RAVGWINDOW * 0.25) 
        tStartup = (((sstartup*2.0)/3.0) / feedrate)

        print "sstartup: ", sstartup

        apos = parser.getPos()[A_AXIS]

        printer.sendPrinterInit()

        parser.execute_line("G0 F%d %s%f" % (feedrate*60, util.dimNames[A_AXIS], apos+sstartup))

        planner.finishMoves()
        printer.sendCommand(CmdEOT)
        printer.sendCommandParamV(CmdMove, [MoveTypeNormal])

        status = printer.getStatus()

        while status["slippage"] == 1.0:
            print "wait for startup..."
            status = printer.getStatus()
            time.sleep(0.1)

        data = []
        t = time.time()
        while status["state"] != StateIdle:

            tMeasure = time.time()-t

            if tMeasure >= tStartup:
                print "measure..."
                data.append(status["slippage"])

            status = printer.getStatus()
            time.sleep(0.01)

        # Cut the last tAccel:
        # cut = int(tAccel/0.01+1)
        cut = int(len(data)*0.9)
        del data[-cut:]

        # Average of ratio for this speed
        grip = sum(data) / len(data)

        # Calibration value, targetSpeed/measuredSensorSpeed, this ist the value
        # to multiply the measured sensor speed to get the real speed:
        print "speed:", feedrate, "ratio:", grip

        calValues.append((feedrate, grip))
        valueSum += grip

        feedrate += 0.25

    f = open("calibrateFilSensor.gnuplot", "w")
    f.write("""
set grid
set yrange [0:%f]
plot '-' using 1:2 with linespoints title 'ratio'
""" % max(1.5, feedrate*2))

    writeDataSet(f, calValues)
    f.write("pause mouse close\n")

    calFile.write(",\n".join(map(lambda tup: "        [%f, %f]" % tup, calValues)))
    calFile.write("\n    ]\n")
    calFile.write("}\n")
    calFile.close()

    avg = valueSum / len(calValues)
    print "Done, average gip: ", avg, ", calibration value:", 1/avg

    printer.sendPrinterInit()

    parser.execute_line("G0 F%d %s%f" % (maxFeedrate*60, util.dimNames[A_AXIS], startPos))

    planner.finishMoves()
    printer.sendCommand(CmdEOT)
    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])

    printer.waitForState(StateIdle)

    # Enable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(1)])








