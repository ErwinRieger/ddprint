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

    printer.commandInit(args)
    startPos = printer.getFilSensor()
    util.manualMove(parser, util.dimIndex['A'], args.distance)
    endPos = printer.getFilSensor()
    diff = endPos - startPos
    print "Filament pos:", startPos, endPos, "counts, difference: %d,  %.3f mm" % (diff, diff*25.4/1000)


#
# Calibrate filament sensor, determine ratio between extruder stepper feedrate and the speed
# measured by the flowrate sensor.
#
def old_calibrateFilSensor(args, parser):

    def writeDataSet(f, data):

        for tup in data:
            f.write("%f %f %f\n" % tup)
        f.write("E\n")

    planner = parser.planner
    printer = planner.printer

    maxFeedrate = args.feedrate or 20

    printer.commandInit(args)

    ddhome.home(parser, args.fakeendstop)

    # Set slow E-acceleration, no jerk
    # eAccel = 1.0
    # eAccel = 0.5
    # eAccel = 0.1
    # PrinterProfile.overrideEAccel(eAccel)
    # PrinterProfile.overrideEJerk(0.001)
    # print "Jerk: ", planner.getJerk()

    # Disable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])
    # Disable temp-flowrate limit
    util.downloadDummyTempTable(printer)

    # Distance needed to accelerate to maxFlowrate
    # accelTime = feedrate / eAccel
    # accelDistance = util.accelDist(0, eAccel, accelTime)

    feedrate = 0.5
    startPos = parser.getPos()[A_AXIS]

    calFile = open("calibrateFilSensor.json", "w")
    calFile.write('{\n    "filSensorCalibration": [\n')

    calValues = []

    while feedrate <= maxFeedrate:

        tStartup = getStartupTime(feedrate)

        accelDistance = 5*feedrate
        print "accelDistance: ", accelDistance

        if False:
            fractEwma = util.EWMA(0.1)
            frtargetEwma = util.EWMA(0.1)

        current_position = parser.getPos()
        apos = current_position[A_AXIS]

        printer.sendPrinterInit()

        parser.execute_line("G0 F%d %s%f" % (feedrate*60, util.dimNames[A_AXIS], apos+accelDistance))

        planner.finishMoves()
        printer.sendCommand(CmdEOT)
        printer.sendCommandParamV(CmdMove, [MoveTypeNormal])

        status = printer.getStatus()

        while status["targetExtrusionSpeed"] < 0.1 and status["actualExtrusionSpeed"] < 0.1:
            print "wait for startup..."
            status = printer.getStatus()
            time.sleep(0.1)

        data = []
        t = time.time()
        while status["state"] != StateIdle:

            st = status["targetExtrusionSpeed"]
            sa = status["actualExtrusionSpeed"]

            tMeasure = time.time()-t

            if tMeasure >= tStartup:

                if False:
                    frtargetEwma.add(st)
                    fractEwma.add(sa)
                    data.append((tMeasure, frtargetEwma.value(), fractEwma.value()))
                else:
                    data.append((tMeasure, st, sa))

            status = printer.getStatus()
            time.sleep(0.01)

        # Cut the last tAccel:
        cut = int(tAccel/0.01+1)
        del data[-cut:]

        # Average of target speed
        tAvg = sum(map(lambda tup: tup[1], data)) / len(data)
        # Average of sensor speed
        aAvg = sum(map(lambda tup: tup[2], data)) / len(data)

        # Calibration value, targetSpeed/measuredSensorSpeed, this ist the value
        # to multiply the measured sensor speed to get the real speed:
        calValue = tAvg / aAvg
        print "tAvg:", tAvg, "aAvg:", aAvg, "calValue:", calValue

        calValues.append((feedrate, calValue))

        f = open("calibrateFilSensor_%.2f.gnuplot" % feedrate, "w")
        f.write("""
set grid
set yrange [0:%f]
plot "-" using 1:2 with linespoints title "TargetSpeed", \\
     "-" using 1:3 with linespoints title "SensorSpeed", \\
     "-" using 1:($2/$3) with linespoints title "ratio target/sensor", \\
     %f with lines title "Target Avg", \\
     %f with lines title "Sensor Avg", \\
     "-" using 1:($3*%f) with linespoints title "Calibrated Speed (f: %.3f)"
""" % (max(1.5, feedrate*2), tAvg, aAvg, calValue, calValue))

        writeDataSet(f, data)
        writeDataSet(f, data)
        writeDataSet(f, data)
        writeDataSet(f, data)
        f.write("pause mouse close\n")

        feedrate += 0.5

    calFile.write(",\n".join(map(lambda tup: "        [%f, %f]" % tup, calValues)))
    calFile.write("\n    ]\n")
    calFile.write("}\n")
    calFile.close()

    printer.sendPrinterInit()

    parser.execute_line("G0 F%d %s%f" % (maxFeedrate*60, util.dimNames[A_AXIS], startPos))

    planner.finishMoves()
    printer.sendCommand(CmdEOT)
    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])

    printer.waitForState(StateIdle)

    # Enable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(1)])


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

    printer.commandInit(args)

    ddhome.home(parser, args.fakeendstop)

    # Disable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])

    # Disable temp-flowrate limit
    util.downloadDummyTempTable(printer)

    feedrate = 0.25
    startPos = parser.getPos()[A_AXIS]

    calFile = open("calibrateFilSensor.json", "w")
    calFile.write('{\n    "filSensorCalibration": [\n')

    calValues = []

    while feedrate <= maxFeedrate:

        tStartup = util.getStartupTime(feedrate)
        tAccel = feedrate / eAccel

        accelDistance = 5*feedrate
        print "accelDistance: ", accelDistance

        # if False:
            # fractEwma = util.EWMA(0.1)
            # frtargetEwma = util.EWMA(0.1)

        current_position = parser.getPos()
        apos = current_position[A_AXIS]

        printer.sendPrinterInit()

        parser.execute_line("G0 F%d %s%f" % (feedrate*60, util.dimNames[A_AXIS], apos+accelDistance))

        planner.finishMoves()
        printer.sendCommand(CmdEOT)
        printer.sendCommandParamV(CmdMove, [MoveTypeNormal])

        status = printer.getStatus()

        while status["actualGrip"] < 0.1:
            print "wait for startup..."
            status = printer.getStatus()
            time.sleep(0.1)

        data = []
        t = time.time()
        while status["state"] != StateIdle:

            # st = status["targetExtrusionSpeed"]
            # sa = status["actualExtrusionSpeed"]
            tMeasure = time.time()-t

            if tMeasure >= tStartup:

                data.append(status["actualGrip"])

            status = printer.getStatus()
            time.sleep(0.01)

        # Cut the last tAccel:
        cut = int(tAccel/0.01+1)
        del data[-cut:]

        # Average of ratio for this speed
        grip = sum(data) / len(data)

        # Calibration value, targetSpeed/measuredSensorSpeed, this ist the value
        # to multiply the measured sensor speed to get the real speed:
        print "speed:", feedrate, "ratio:", grip

        calValues.append((feedrate, grip))

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

    printer.sendPrinterInit()

    parser.execute_line("G0 F%d %s%f" % (maxFeedrate*60, util.dimNames[A_AXIS], startPos))

    planner.finishMoves()
    printer.sendCommand(CmdEOT)
    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])

    printer.waitForState(StateIdle)

    # Enable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(1)])








