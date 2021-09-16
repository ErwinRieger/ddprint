# -*- coding: utf-8 -*-
#
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

#
# Some functions to test and calibrate the printer.
#

import time, math, pprint, sys

import numpy as np

import ddhome, movingavg, gcodeparser, ddprintutil as util
import intmath
from ddprofile import MatProfile
from ddprinter import Printer
from ddplanner import Planner, initParser
from ddprintcommands import *
from ddprintstates import *
from ddprintconstants import A_AXIS, maxTimerValue16

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
# * wait for average-converged
# * print out ratio of dialed in speed and measured speed
#
def calibrateESteps(args, printer, planner):

    feedrate = args.feedrate or 5.0
    feedrate = min(feedrate, printer.printerProfile.getJerk("A"))

    printer.commandInit(args)

    # Disable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])

    # Disable temp-flowrate limit
    util.downloadDummyTempTable(printer)

    # Set filament sensor calibration to 1
    p = intmath.fsCalibration(1.0)
    printer.sendCommandParamV(CmdSetFilSensorCal, (p, ))

    print "\n*"
    print "* Machine Calibration: Measeure e-steps"
    print "*"

    # Time for one revolution
    tRound = printer.printerProfile.getFeederWheelCircumI() / feedrate
    tStartup = util.getStartupTime(printer, feedrate)
    print "tRound:", tRound, "tStartup:", tStartup

    print "running %.2f seconds with %.2f mm/s" % (tRound, feedrate)

    printer.sendCommandParamV(CmdContinuousE, [packedvalue.uint16_t(maxTimerValue16)])

    stepperVal = util.eTimerValue(printer, feedrate)
    printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(stepperVal)])

    time.sleep(tStartup)

    tStart = time.time()

    crossAvg = movingavg.CrossingAverage()

    while not crossAvg.converged:

        if crossAvg.converged:
            break

        fsreadings = printer.getFSReadings()
        crossAvg.addReadings(fsreadings, 1.0)

        t = time.time()
        if t > (tStart + tRound):

            if not crossAvg.started:

                treading = (t-tStart) / crossAvg.getNValues()
                crossAvg.started = True

        if crossAvg.getNValues():

            print "\r# %s, %d(%d), avg short term: %.3f, long term: %.3f" % ((crossAvg.started and "Measure" or "Starting"), crossAvg.getNValues(), crossAvg.nLongPeriod, crossAvg.shortAvg(), crossAvg.longAvg()),
            sys.stdout.flush()

        time.sleep(0.25)

    print ""

    dFeederWheel = printer.printerProfile.getFeederWheelDiamI()

    if crossAvg.converged:

        avg = crossAvg.shortAvg()
        meanLong = crossAvg.longAvg()
        print "\nAvg converged after %d seconds:" % int(time.time()-tStart), meanLong, avg, "%.5f%%" % ((((meanLong/avg)-1.0))*100)
   
        measuredEsteps = printer.printerProfile.getFilSensorCountsPerMM() / avg

        esteps = printer.printerProfile.getStepsPerMMI(A_AXIS)
        print "Measured e-steps: %.3f (%d)" % (measuredEsteps, esteps)

    else:
        print "Error avg did not converge."

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

    maxFeedrate = args.feedrate or 10.0 # mm/s

    printer.commandInit(args)

    # Disable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])

    # Disable temp-flowrate limit
    util.downloadDummyTempTable(printer)

    # Set filament sensor calibration to 1
    p = intmath.fsCalibration(1.0)
    printer.sendCommandParamV(CmdSetFilSensorCal, (p, ))

    dt = printer.printerProfile.getFilSensorIntervalI()

    # calValues = []
    # valueSum = 0

    # Start feeder 
    printer.sendCommandParamV(CmdContinuousE, [packedvalue.uint16_t(maxTimerValue16)])

    stepperValues = []

    # steps = [0.25, 0.5, 0.75, 1.0]
    steps = [0.5, 0.75, 1.0]
    stepindex = 0
    stepfactor = 1.0

    # Start feedrate
    feedrate = steps[0] * stepfactor

    dFeederWheel = printer.printerProfile.getFeederWheelDiamI()

    print "\n*"
    print "* Machine Calibration: Measeure flowrate sensor calibration value"
    print "*"
    while feedrate <= maxFeedrate:

        # Time for one revolution
        tRound = printer.printerProfile.getFeederWheelCircumI() / feedrate
        tStartup = util.getStartupTime(printer, feedrate)
        print "tRound:", tRound, "tStartup:", tStartup

        print "running %.2f seconds with %.2f mm/s" % (tRound, feedrate)

        # set new feedrate:
        stepperVal = util.eTimerValue(printer, feedrate)
        printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(stepperVal)])
        stepperValues.append(stepperVal)

        time.sleep(tStartup)

        tStart = time.time()

        crossAvg = movingavg.CrossingAverage()

        # print "measure feedrate %.2f for %.2f seconds" % (feedrate, 5*tRun)

        while not crossAvg.converged:

            if crossAvg.converged:
                break

            fsreadings = printer.getFSReadings()
            crossAvg.addReadings(fsreadings, 1.0)

            t = time.time()
            if t > (tStart + tRound):

                if not crossAvg.started and crossAvg.getNValues():

                    treading = (t-tStart) / crossAvg.getNValues()
                    crossAvg.started = True

            print "\r# %s, %d(%d), avg short term: %.3f, long term: %.3f" % ((crossAvg.started and "Measure" or "Starting"), crossAvg.nLongPeriod, crossAvg.getNValues(), crossAvg.shortAvg(), crossAvg.longAvg()),
            sys.stdout.flush()

            time.sleep(0.25)

        print ""

        if crossAvg.converged:

            avg = crossAvg.shortAvg()
            meanLong = crossAvg.longAvg()
            print "\nAvg converged after %d seconds:" % int(time.time()-tStart), meanLong, avg, "%.5f%%" % (((meanLong/avg)-1.0)*100)
   
        else:
            print "Error avg did not converge."
            break

        stepindex += 1
        if stepindex == len(steps):
            stepindex = 0
            stepfactor *= 10.0

        feedrate = steps[stepindex] * stepfactor

    # Decelerate feeder
    stepperValues.reverse()
    for stepperVal in stepperValues:
        printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(stepperVal)])
        time.sleep(0.1)

    printer.sendCommandParamV(CmdContinuousE, [packedvalue.uint16_t(0)])

    fscal = "---"
    if printer.printerProfile.hasValue("filSensorCalibration"):
        fscal = "%.4f" % printer.printerProfile.getFilSensorCalibration()

    print "Done, average ratio/calibration value: %.4f (%s)" % (crossAvg.longAvg(), fscal)

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
    dFeederWheel = printer.printerProfile.getFeederWheelDiamI()
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

    # Re-enable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(1)])



def execGcode(args):

    (printer, parser, planner) = initParser(args, mode=args.mode)

    ddhome.home(args, printer, parser, planner)

    for gcode in args.gcode.split("\n"):
        for g in gcode.split(";"):
            print "Exec gcode: '%s'" % g
            parser.execute_line(g)

    planner.finishMoves()

    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])

    printer.waitForState(StateInit)










