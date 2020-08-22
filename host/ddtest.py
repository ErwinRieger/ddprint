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
# Some functions to test and calibrate the printer.
#

import time, math, pprint, sys

import numpy as np

import ddhome, movingavg, ddprintutil as util
from ddprofile import MatProfile
from ddprinter import Printer
from ddprintcommands import *
from ddprintstates import *
from ddprintconstants import A_AXIS, maxTimerValue16



#
# Write measured sensor data to python file. xxx use json here...
#
def writePython(dFeederWheel, feedrate, x, y, fn):

    filename = fn + ".%.2f.py" % feedrate
    print "writing", filename
    with open(filename, "w") as f:
        f.write("dFeederWheel = %f\n" % dFeederWheel)
        f.write("feedrate = %f\n" % feedrate)
        f.write("ts=[\n")
        for ts in x:
            f.write("  %d,\n" % ts)
        f.write("]\n")
        f.write("y=[\n")
        for value in y:
            f.write("  %d,\n" % value)
        f.write("]\n")
    f.close()

def compactReadings(readings):

        x = []
        y = []

        timeStamps = readings.keys()
        timeStamps.sort()
        lastTs = timeStamps[0] - 100
        for ts in timeStamps:

            interval = ts - lastTs

            if ts - lastTs >= 110:
                print "sequence error:", ts, lastTs, ts - lastTs
                assert(0)

            x.append(ts)
            y.append(readings[ts])

            lastTs = ts

        return (x, y)

def writeDataSet(dFeederWheel, feedrate, data, fn):

    (x, y) = compactReadings(data)
    writePython(dFeederWheel, feedrate, x, y, fn)

# Write data for gnuplot
def writeData(f, measurements):

    for (t, s) in measurements:
        f.write("%f %f\n" % (t, s))
    f.write("E\n")


def testFilSensor(args, printer, parser):

    feedrate = args.feedrate or 1.0

    printer.commandInit(args)
    startPos = printer.getFilSensor()
    util.manualMove(parser, util.dimIndex['A'], args.distance, feedrate=feedrate)
    endPos = printer.getFilSensor()
    diff = endPos - startPos

    # diff ist in sensor-counts
    pcal = printer.printerProfile.getFilSensorCalibration()
    steps_per_mm = printer.printerProfile.getStepsPerMMI(A_AXIS)

    print "steps_per_mm  E:", steps_per_mm

    cal = diff / (steps_per_mm * args.distance) 

    print "Commanded steps: ", (steps_per_mm * args.distance)
    print "Filament pos in counts old:", startPos, ", new:", endPos, "difference: %d counts" % diff
    print "Calibration value from profile: %f, measured ratio: %f" % (pcal, cal)

#
# Measure/calibrate feeder e-steps value
# * run with disconnected bowden from head
# * dial in constant speed (feedrate)
# * wait for average-lock
# * print out ratio of dialed in speed and measured speed
#
def calibrateESteps(args, printer, planner):

    assert(0) # todo: transition to printer.printerProfile...

    feedrate = args.feedrate or 5.0
    feedrate = min(feedrate, PrinterProfile.getValues()['axes']["A"]['jerk'])

    printer.commandInit(args)

    # Disable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])

    # Disable temp-flowrate limit
    util.downloadDummyTempTable(printer)

    # Set filament sensor calibration to 1
    printer.sendCommandParamV(CmdSetFilSensorCal, [packedvalue.float_t(1.0)])

    dt = PrinterProfile.getFilSensorInterval()

    # Time for one revolution
    tRound = PrinterProfile.getFeederWheelCircum() / feedrate
    tStartup = util.getStartupTime(feedrate)
    print "tRound:", tRound, "tStartup:", tStartup

    nAvgLong = PrinterProfile.getNLongInterval(feedrate)

    print "running %.2f seconds with %.2f mm/s, # of samples for long average: %d" % (tRound, feedrate, nAvgLong)

    printer.sendCommandParamV(CmdContinuousE, [packedvalue.uint16_t(maxTimerValue16)])

    stepperVal = util.eTimerValue(planner, feedrate)
    printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(stepperVal)])

    time.sleep(tStartup)

    t = time.time()
    tEnd = t + 5 * tRound # max running time, bail out when average is locked
    tWait = 0.75          # 2 readings overlap

    crossAvg = movingavg.CrossingAverage(nAvgLong)

    while t < tEnd:

        fsreadings = printer.getFSReadings()
        for (ts, dy) in fsreadings:
            crossAvg.addValue(ts, dy)

        print "\r# %d, avg short term: %.2f, long term: %.2f" % (crossAvg.nValues, crossAvg.shortAvg(), crossAvg.longAvg()),
        sys.stdout.flush()

        if crossAvg.locked:
            break

        time.sleep(tWait)
        t = time.time()

    print ""

    dFeederWheel = PrinterProfile.getFeederWheelDiam()
    writeDataSet(dFeederWheel, feedrate, crossAvg.data, "calibrateESteps_dataset")

    pp = PrinterProfile.get()

    if crossAvg.locked:

        meanShort = crossAvg.locked[2]
        meanLong = crossAvg.locked[3]
        avg = crossAvg.longAvg()
        print "avg lock at t:", crossAvg.locked[1], meanLong, avg, "%.1f%%" % (((meanLong/avg)-1.0)*100)
   
        # should be speed:
        s = (feedrate * pp.getFilSensorCountsPerMM()) * dt

        print "speed should be:", s

        ratio = s/meanLong
        esteps = PrinterProfile.getStepsPerMM(A_AXIS)
        print "ratio commanded speed / measured speed: ", ratio
        print "adjusted e-steps: %d (%d)" % (esteps * ratio, esteps)

    else:
        print "Error avg did not lock..."

    printer.sendCommandParamV(CmdContinuousE, [packedvalue.uint16_t(maxTimerValue16)])
    printer.sendCommandParamV(CmdContinuousE, [packedvalue.uint16_t(0)])

    # Re-enable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(1)])


#
# Calibrate filament sensor, determine ratio between extruder steps and the value 
# measured by the flowrate sensor.
#
# * run the feeder with different feedrates
# * build average stepper-to-flowsensor ratio
#
def calibrateFilSensor(args, printer, planner):

    assert(0) # todo: transition to printer.printerProfile...

    maxFeedrate = args.feedrate or 10.0 # mm/s

    printer.commandInit(args)

    # Disable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])

    # Disable temp-flowrate limit
    util.downloadDummyTempTable(printer)

    # Set filament sensor calibration to 1
    printer.sendCommandParamV(CmdSetFilSensorCal, [packedvalue.float_t(1.0)])

    # xxx check jerk
    print "todo: jerk"
    eJerk = PrinterProfile.getValues()['axes']["A"]['jerk']

    calValues = []
    valueSum = 0

    steps_per_mm = PrinterProfile.getStepsPerMM(A_AXIS)

    # Start feeder 
    printer.sendCommandParamV(CmdContinuousE, [packedvalue.uint16_t(maxTimerValue16)])

    stepperValues = []

    steps = [0.25, 0.5, 0.75, 1.0]
    stepindex = 0
    stepfactor = 1.0

    # Start feedrate
    feedrate = steps[0] * stepfactor

    dt = PrinterProfile.getFilSensorInterval()
    dFeederWheel = PrinterProfile.getFeederWheelDiam()

    while feedrate <= maxFeedrate:

        # Time for one revolution
        tRound = PrinterProfile.getFeederWheelCircum() / feedrate
        tStartup = util.getStartupTime(feedrate)
        print "tRound:", tRound, "tStartup:", tStartup

        nAvgLong = PrinterProfile.getNLongInterval(feedrate)

        print "running %.2f seconds with %.2f mm/s, # of samples for long average: %d" % (tRound, feedrate, nAvgLong)

        # set new feedrate:
        stepperVal = util.eTimerValue(planner, feedrate)
        printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(stepperVal)])
        stepperValues.append(stepperVal)

        time.sleep(tStartup)

        t = time.time()
        tEnd = t + 5 * tRound # max running time, bail out when average is locked
        tWait = 0.75          # 2 readings overlap

        crossAvg = movingavg.CrossingAverage(nAvgLong)

        # print "measure feedrate %.2f for %.2f seconds" % (feedrate, 5*tRun)

        # readings = {}
        # measurements = []

        while t < tEnd:

            fsreadings = printer.getFSReadings()

            for (ts, dy) in fsreadings:

                crossAvg.addValue(ts, dy)

            print "\r# %d, avg short term: %.2f, long term: %.2f" % (crossAvg.nValues, crossAvg.shortAvg(), crossAvg.longAvg()),
            sys.stdout.flush()

            if crossAvg.locked:
                break

            time.sleep(tWait)
            t = time.time()

        print ""

        writeDataSet(dFeederWheel, feedrate, crossAvg.data, "calibrationData")

        if not crossAvg.locked:
            print "Error avg did not lock..."
            break

        meanShort = crossAvg.locked[2]
        meanLong = crossAvg.locked[3]
        avg = crossAvg.longAvg()
        print "avg lock at t:", crossAvg.locked[1], meanLong, avg, "%.1f%%" % (((meanLong/avg)-1.0)*100)
   
        # should be speed:
        stepsPerInterval = feedrate * steps_per_mm * dt

        ratio = meanLong / stepsPerInterval
        print "speed should be:", stepsPerInterval, "ratio:", ratio

        calValues.append((feedrate, ratio))
        valueSum += ratio

        stepindex += 1
        if stepindex == 4:
            stepindex = 0
            stepfactor *= 10.0

        feedrate = steps[stepindex] * stepfactor

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

    writeData(f, calValues)

    f.write("pause mouse close\n")

    calFile = open("calibrateFilSensor.json", "w")
    calFile.write('{\n    "filSensorCalibration": [\n')
    calFile.write(",\n".join(map(lambda tup: "        [%f, %f]" % tup, calValues)))
    calFile.write("\n    ]\n")
    calFile.write("}\n")
    calFile.close()

    print "Done, average ratio/calibration value: ", avg

    # Re-enable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(1)])




#
# Check smoothness/roundness of feeder measurements. Move filament for
# 10 flowratesensor turns and plot the sensor values.
#
# XXX Assumes 100ms filament sensor measurement interval
#
def testFeederUniformity(args, parser):

    planner = parser.planner
    printer = planner.printer

    assert(0) # todo: transition to printer.printerProfile...

    # umfang
    dFeederWheel = PrinterProfile.getFeederWheelDiam()
    circ = dFeederWheel * math.pi

    nRound = 2
    nRound = 12

    d = circ * nRound

    # speed: 1U/10sec
    tRound = 10
    feedrate = args.feedrate or (circ / tRound)

    print "circum:", circ, "feedrate:", feedrate

    printer.commandInit(args)

    # Disable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])

    # Disable temp-flowrate limit
    util.downloadDummyTempTable(printer)

    startPos = parser.getPos()[A_AXIS]

    readings = {}
    measurements = []

    print "running feeder with %.2f mm/s and %.2f mm distance" % (feedrate, d)

    # Start feeder motor
    parser.execute_line("G0 F%d %s%f" % (feedrate*60, util.dimNames[A_AXIS], startPos + d))
    planner.finishMoves()
    printer.sendCommand(CmdEOT)
    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])

    tWait = 0.75
    tMove = (tRound * nRound) * 1.1

    t = time.time()
    tEnd = t + tMove

    time.sleep(10 * tWait) # fill fsreadings buffer

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
    lastTs = timeStamps[0] - 100
    for ts in timeStamps:

        dt = ts - lastTs
        v = readings[ts]

        # print "dt:", ts, lastTs, dt, v

        assert(abs(dt) < 150)

        measurements.append((ts, v))

        lastTs = ts

    """
    while measurements[0][1] == 0:
        del measurements[0]

    while measurements[-1][1] == 0:
        del measurements[-1]
    """

    print "writing testFeederUniformity_dataset.py:"
    with open("testFeederUniformity_dataset.py", "w") as f:
        f.write("dFeederWheel = %f\n" % dFeederWheel)
        f.write("feedrate = %f\n" % feedrate)
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
     (stats_up_quartile - stats_lo_quartile)*50/stats_median title "error %"
""")

    writeData(f, measurements)
    writeData(f, measurements)

    f.write("pause mouse close\n")
    f.close()

    printer.waitForState(StateInit)

    # Rewind
    # print "Rewinding..."
    # parser.execute_line("G0 F%d %s%f" % (PrinterProfile.getMaxFeedrate(A_AXIS)*60, util.dimNames[A_AXIS], startPos))
    # planner.finishMoves()
    # printer.sendCommand(CmdEOT)
    # printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    # printer.waitForState(StateInit)

    # Re-enable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(1)])













