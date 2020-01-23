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

import time, math, pprint

import numpy as np

import ddhome, ddprintutil as util
from ddprofile import PrinterProfile, MatProfile
from ddprinter import Printer
from ddprintcommands import *
from ddprintstates import *
from ddprintconstants import A_AXIS, maxTimerValue16




# write measured sensor data to python file. xxx use json here...
#
def writePython(x, y, filename):

    print "writing", filename
    with open(filename, "w") as f:
        f.write("ts=[\n")
        for ts in x:
            f.write("  %d,\n" % ts)
        f.write("]\n")
        f.write("y=[\n")
        for value in y:
            f.write("  %d,\n" % value)
        f.write("]\n")
    f.close()

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
def old_calibrateFilSensor(args, parser):

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

    # RAVGWINDOW = 10 # xxx defined in filsensor.h

    twoRevs = math.pi * 15 # [mm]

    while feedrate <= maxFeedrate:

        # tStartup = util.getStartupTime(feedrate)

        # xxx 0.25 hardcoded in fw!
        # sstartup = 5 * (RAVGWINDOW * 0.25) 
        # tStartup = (((sstartup*2.0)/3.0) / feedrate)

        # print "sstartup: ", sstartup

        apos = parser.getPos()[A_AXIS]

        printer.sendPrinterInit()

        parser.execute_line("G0 F%d %s%f" % (feedrate*60, util.dimNames[A_AXIS], apos+twoRevs))

        planner.finishMoves()
        printer.sendCommand(CmdEOT)
        printer.sendCommandParamV(CmdMove, [MoveTypeNormal])

        status = printer.getStatus()

        # while status["slippage"] == 1.0:
            # print "wait for startup..."
            # status = printer.getStatus()
            # time.sleep(0.1)

        data = []
        while status["state"] != StateIdle:

            data.append(status["slippage"])

            status = printer.getStatus()
            time.sleep(0.1)

        # Average of ratio for this speed
        grip = sum(data) / len(data)

        # Calibration value, targetSpeed/measuredSensorSpeed, this ist the value
        # to multiply the measured sensor speed to get the real speed:
        print "Feedrate %.2f mm/s done, ratio: %.3f" % (feedrate, grip)

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
    print "Done, average ratio: ", avg, ", calibration value:", 1/avg

    printer.sendPrinterInit()

    # Rewind
    print "Rewinding..."
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

    def compactReadings(readings):

        x = []
        y = []

        timeStamps = readings.keys()
        timeStamps.sort()
        lastTs = timeStamps[0] - 100
        for ts in timeStamps:

            interval = ts - lastTs

            if interval != 0:
                print "XXX normalize ", interval

            assert(abs(ts - lastTs) < 110)

            x.append(ts)
            y.append(readings[ts])

            lastTs = ts

        return (np.array(x), np.array(y))

    planner = parser.planner
    printer = planner.printer

    aFilament = MatProfile.getMatArea()

    # 0.8 nozzle
    max80 = 25 # mm3/s
    maxFeedrate = args.feedrate or max80/aFilament

    # 0.4 nozzle
    # max40 = 10 # mm3/s

    printer.commandInit(args, PrinterProfile.getSettings())

    # ddhome.home(parser, args.fakeendstop)

    # Disable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])

    # Disable temp-flowrate limit
    util.downloadDummyTempTable(printer)

    # Set filament sensor calibration to 1
    printer.sendCommandParamV(CmdSetFilSensorCal, [packedvalue.float_t(1.0)])

    # Feedrate step
    eJerk = PrinterProfile.getValues()['axes']["A"]['jerk']
    feedrateStep = 0.25
    feedrateStep = 0.5 # debug
    assert(feedrateStep <= eJerk)

    # Start feedrate
    feedrate = feedrateStep
    feedrate = 2 # debug
    # feedrate = 3 # debug

    calFile = open("calibrateFilSensor.json", "w")
    calFile.write('{\n    "filSensorCalibration": [\n')

    calValues = []
    valueSum = 0

    # xxx feeder wheel diameter hardcoded, add printer profile entry
    oneRev = math.pi * 10.2 # [mm]

    steps_per_mm = PrinterProfile.getStepsPerMM(A_AXIS)
    # stepsOneRev = oneRev * steps_per_mm

    # Start feeder 
    # printer.sendPrinterInit()
    printer.sendCommandParamV(CmdContinuousE, [packedvalue.uint16_t(maxTimerValue16)])

    stepperValues = []
    while feedrate <= maxFeedrate:

        # set new feedrate:
        stepperVal = util.eTimerValue(planner, feedrate)
        printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(stepperVal)])
        stepperValues.append(stepperVal)

        time.sleep(1)

        # Time for one rev
        tRun = oneRev / feedrate
        endTime = time.time() + 10 * tRun
        endTime = time.time() + 5 * tRun # debug
        # endTime = time.time() + 2 * tRun # debug
        data = []

        stepsPerSecond = feedrate * steps_per_mm 

        print "measure feedrate %.2f for %.2f seconds" % (feedrate, tRun)

        readings = {}
        # measurements = []

        while time.time() < endTime:

            # status = printer.getStatus()
            # data.append(status["slippage"])
            fsreadings = printer.getFSReadings()

            # print "fsreadings:"
            # pprint.pprint(fsreadings)

            allZero = True
            for (ts, dy) in fsreadings:

                readings[ts] = dy

                if dy:
                    allZero = False

            if allZero:
                break

            time.sleep(0.75)

        (x, y) = compactReadings(readings)

        writePython(x, y, "calibrationData.%.2f.py" % feedrate)


        #######################################################################

        median = np.median(y)

        mask = [True] * np.size(y)

        # Trim data, front 
        i = 0
        while y[i] < median:
            mask[i] = False
            i += 1

        # Trim data, back 
        i = -1
        while y[i] < median:
            mask[i] = False
            i -= 1

        x = np.extract( mask, x)
        y = np.extract( mask, y)

        startTime = x[0] + (tRun*0.25*1000)
        nRev = (x[-1] - startTime) / (tRun*1000)

        print "sampling %d revolutions" % nRev
        ts1 = np.extract( x >= startTime, x)
        y1 = np.extract( x >= startTime, y)

        ts2 = np.extract( ts1 <= (startTime+nRev*(tRun*1000)), ts1)
        y2 = np.extract( ts1 <= (startTime+nRev*(tRun*1000)), y1)

        # methode1 über durchschnitt
        meanSensor = np.mean(y2)

        # print "sampling data, mean sensor counts:", y2, meanSensor
        print "mean sensor counts:", meanSensor

        times = np.diff(ts2)
        tMean = np.mean(times)
        print "mean time between samples: %.2f ms" % tMean

        # xxx ggf besser über die step-zeit gehen?
        # meanSteps = stepsOneRev
        meanSteps = (stepsPerSecond * tMean) / 1000.0

        ratio1 = meanSensor / meanSteps
        print "Methode1: mean steps, ratio:", meanSteps, ratio1

        # methode2 über summe:
        sumSensorCounts = np.sum(y2)
        sumSteps = (stepsPerSecond * (ts2[-1] - ts2[0])) / 1000.0

        ratio2 = sumSensorCounts/sumSteps

        print "Methode2, counts:", sumSensorCounts, "steps:", sumSteps, "ratio: ", ratio2

        #######################################################################

        # Average of ratio for this speed
        # grip = sum(data) / len(data)

        # Calibration value, targetSpeed/measuredSensorSpeed, this ist the value
        # to multiply the measured sensor speed to get the real speed:
        print "Feedrate %.2f mm/s done, ratio1: %.4f, ratio2: %.4f" % (feedrate, ratio1, ratio2)

        calValues.append((feedrate, ratio2))
        valueSum += ratio2

        feedrate += feedrateStep

    # Decelerate feeder
    stepperValues.reverse()
    for stepperVal in stepperValues:
        printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(stepperVal)])
        time.sleep(0.1)

    printer.sendCommandParamV(CmdContinuousE, [packedvalue.uint16_t(0)])

    avg = valueSum / len(calValues)

    yrange = int(avg*10) / 10.0

    f = open("calibrateFilSensor.gnuplot", "w")
    f.write("""
set grid
set yrange [%f:%f]
plot '-' using 1:2 with linespoints title 'ratio', \\
     %.4f title 'avg'
""" % (yrange, yrange+0.1, avg))

    writeDataSet(f, calValues)
    f.write("pause mouse close\n")

    calFile.write(",\n".join(map(lambda tup: "        [%f, %f]" % tup, calValues)))
    calFile.write("\n    ]\n")
    calFile.write("}\n")
    calFile.close()

    print "Done, average ratio/calibration value: ", avg

    # XXX does not work, continuous mode does not update planner pos...
    # # Rewind
    # print "Rewinding..."
    # printer.sendPrinterInit()
    # parser.execute_line("G0 F%d %s%f" % (maxFeedrate*60, util.dimNames[A_AXIS], startPos))
    # planner.finishMoves()
    # printer.sendCommand(CmdEOT)
    # printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    # printer.waitForState(StateIdle)

    # Enable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(1)])







#
# Check smoothness/roundness of feeder measurements. Move filament for
# 10 flowratesensor turns and plot the sensor values.
#
# XXX Assumes 100ms filament sensor measurement interval
#
def testFeederUniformity(args, parser):

    def writeData(f, measurements):

        for (t, s) in measurements:
            f.write("%f %f\n" % (t, s))
        f.write("E\n")


    planner = parser.planner
    printer = planner.printer

    # durchmesser rolle feeder: ca. 11mm
    droll = 11

    # umfang
    circ = droll * math.pi

    nRound = 2
    nRound = 12

    d = circ * nRound

    # speed: 1U/10sec
    tRound = 10
    feedrate = args.feedrate or (circ / tRound)

    print "circum:", circ, "feedrate:", feedrate

    printer.commandInit(args, PrinterProfile.getSettings())

    # ddhome.home(parser, args.fakeendstop)

    # Disable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])

    # Disable temp-flowrate limit
    util.downloadDummyTempTable(printer)

    startPos = parser.getPos()[A_AXIS]

    readings = {}
    measurements = []

    print "running feeder with %.2f mm/s and %.2f mm distance" % (feedrate, d)

    # Start feeder motor
    printer.sendPrinterInit()
    parser.execute_line("G0 F%d %s%f" % (feedrate*60, util.dimNames[A_AXIS], startPos + d))
    planner.finishMoves()
    printer.sendCommand(CmdEOT)
    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])

    hack = 1 # 5
    tWait = 0.75 * hack
    tMove = (tRound * nRound) * 1.1

    t = time.time()
    tEnd = t + tMove
    while t < tEnd:

        time.sleep(tWait)

        fsreadings = printer.getFSReadings()

        print "fsreadings:"
        pprint.pprint(fsreadings)

        allZero = True
        for (ts, dy) in fsreadings:

            readings[ts] = dy

            if dy:
                allZero = False

        if allZero:
            break

        t = time.time()

    timeStamps = readings.keys()
    timeStamps.sort()
    lastTs = timeStamps[0] - 100 * hack
    for ts in timeStamps:

        assert(abs(ts - lastTs) < 150 * hack)

        measurements.append((ts, readings[ts]))

        lastTs = ts

    """
    while measurements[0][1] == 0:
        del measurements[0]

    while measurements[-1][1] == 0:
        del measurements[-1]
    """

    print "writing testFeederUniformity_dataset.py:"
    with open("testFeederUniformity_dataset.py", "w") as f:
        f.write("dataset=[\n")
        for tup in measurements:
            f.write("  (%d, %d),\n" % tup)
        f.write("]\n")
    f.close()

    f = open("testFeederUniformity.gnuplot", "w")
    f.write("""
set grid
stats "-" using 2 name "stats"
""")

    writeData(f, measurements)

    f.write("""
plot '-' using 1:2 with linespoints title 'sensor counts', \\
     "-" using 1:2 with linespoints smooth bezier lt rgb "red" lw 3 title "sensor counts smooth", \\
     stats_median, stats_lo_quartile lt rgb "brown", stats_up_quartile lt rgb "brown", \\
     (stats_up_quartile - stats_lo_quartile)*100/stats_median title "error %"
""")

    writeData(f, measurements)
    writeData(f, measurements)

    f.write("pause mouse close\n")
    f.close()

    printer.waitForState(StateIdle)

    # Rewind
    # print "Rewinding..."
    # printer.sendPrinterInit()
    # parser.execute_line("G0 F%d %s%f" % (PrinterProfile.getMaxFeedrate(A_AXIS)*60, util.dimNames[A_AXIS], startPos))
    # planner.finishMoves()
    # printer.sendCommand(CmdEOT)
    # printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    # printer.waitForState(StateIdle)

    # Re-enable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(1)])













