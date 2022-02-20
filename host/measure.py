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

import ddhome, movingavg, ddprintutil as util
import intmath
from ddprinter import Printer
from ddplanner import Planner, initParser
from ddprintcommands import *
from ddprintstates import *
from ddprintconstants import A_AXIS, maxTimerValue16, dimBitsIndex, fTimer

# Write data for gnuplot
def writeData(f, measurements):

    for (t, s) in measurements:
        f.write("%f %f\n" % (t, s))
    f.write("E\n")

def testFilSensor(args, printer, parser):

    feedrate = args.feedrate or 1.0

    printer.commandInit()
    startPos = printer.getFilSensor()
    util.manualMove(parser, util.dimIndex['A'], args.distance, feedrate=feedrate)
    endPos = printer.getFilSensor()
    diff = endPos - startPos

    steps_per_mm = printer.printerProfile.getStepsPerMMI(A_AXIS)

    print("steps_per_mm  E:", steps_per_mm)

    cal = diff / (steps_per_mm * args.distance) 

    print("Commanded steps: ", (steps_per_mm * args.distance))
    print("Filament pos in counts old:", startPos, ", new:", endPos, "difference: %d counts" % diff)
    print("Calibration value from profile: %f, measured ratio: %f" % (pcal, cal))

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

    printer.commandInit()

    # Disable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])

    # Disable temp-flowrate limit
    util.downloadDummyTempTable(printer)

    # Set filament sensor calibration to 1
    p = intmath.fsCalibration(1.0)
    printer.sendCommandParamV(CmdSetFilSensorCal, (p, ))

    print("\n*")
    print("* Machine Calibration: Measeure e-steps")
    print("*")

    # Time for n revolutions
    nRound = 5
    tShortAvg = (printer.printerProfile.getFeederWheelCircumI() / feedrate) * nRound
    tStartup = util.getStartupTime(printer, feedrate)
    print("tShortAvg:", tShortAvg, "tStartup:", tStartup)

    print("running %.2f seconds with %.2f mm/s" % (tShortAvg, feedrate))

    printer.sendCommandParamV(CmdContinuous,
            [ packedvalue.uint8_t(dimBitsIndex["A"]),
              packedvalue.uint16_t(maxTimerValue16)])

    stepperVal = util.eTimerValue(printer, feedrate)
    printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(stepperVal)])

    time.sleep(tStartup)

    tStart = time.time()

    crossAvg = movingavg.CrossingAverage()

    while not crossAvg.converged:

        # if crossAvg.converged:
            # break

        fsreadings = printer.getFSReadings()
        crossAvg.addReadings(fsreadings, 1.0)

        t = time.time()
        if t > (tStart + tShortAvg):

            if not crossAvg.started:

                # treading = (t-tStart) / crossAvg.getNValues()
                crossAvg.started = True

        if crossAvg.getNValues():

            print("\r# %s, %d(%d), avg short term: %.3f, long term: %.3f" % ((crossAvg.started and "Measure" or "Starting"), crossAvg.getNValues(), crossAvg.nLongPeriod, crossAvg.shortAvg(), crossAvg.longAvg()), end='')
            sys.stdout.flush()

        time.sleep(0.25)

    print("")

    if crossAvg.converged:

        avg = crossAvg.shortAvg()
        meanLong = crossAvg.longAvg()
        print("\nAvg converged after %d seconds:" % int(time.time()-tStart), meanLong, avg, "%.5f%%" % ((((meanLong/avg)-1.0))*100))
   
        measuredEsteps = printer.printerProfile.getFilSensorCountsPerMM() / avg

        esteps = printer.printerProfile.getStepsPerMMI(A_AXIS)
        print("Measured e-steps: %.3f (%d)" % (measuredEsteps, esteps))

    else:
        print("Error avg did not converge.")

    printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(0)])

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

    printer.commandInit()

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
    printer.sendCommandParamV(CmdContinuous, 
            [ packedvalue.uint8_t(dimBitsIndex["A"]),
              packedvalue.uint16_t(maxTimerValue16)])

    stepperValues = []

    # steps = [0.25, 0.5, 0.75, 1.0]
    steps = [0.5, 0.75, 1.0]
    stepindex = 0
    stepfactor = 1.0

    # Start feedrate
    feedrate = steps[0] * stepfactor

    print("\n*")
    print("* Machine Calibration: Measeure flowrate sensor calibration value")
    print("*")
    while feedrate <= maxFeedrate:

        # Time for one revolution
        tRound = printer.printerProfile.getFeederWheelCircumI() / feedrate
        tStartup = util.getStartupTime(printer, feedrate)
        print("tRound:", tRound, "tStartup:", tStartup)

        print("running %.2f seconds with %.2f mm/s" % (tRound, feedrate))

        # set new feedrate:
        stepperVal = util.eTimerValue(printer, feedrate)
        printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(stepperVal)])
        stepperValues.append(stepperVal)

        time.sleep(tStartup)

        tStart = time.time()

        crossAvg = movingavg.CrossingAverage()

        # print "measure feedrate %.2f for %.2f seconds" % (feedrate, 5*tRun)

        while not crossAvg.converged:

            # if crossAvg.converged:
                # break

            fsreadings = printer.getFSReadings()
            crossAvg.addReadings(fsreadings, 1.0)

            t = time.time()
            if t > (tStart + tRound):

                if not crossAvg.started and crossAvg.getNValues():

                    # treading = (t-tStart) / crossAvg.getNValues()
                    crossAvg.started = True

            print("\r# %s, %d(%d), avg short term: %.3f, long term: %.3f" % ((crossAvg.started and "Measure" or "Starting"), crossAvg.nLongPeriod, crossAvg.getNValues(), crossAvg.shortAvg(), crossAvg.longAvg()), end='')
            sys.stdout.flush()

            time.sleep(0.25)

        print("")

        if crossAvg.converged:

            avg = crossAvg.shortAvg()
            meanLong = crossAvg.longAvg()
            print("\nAvg converged after %d seconds:" % int(time.time()-tStart), meanLong, avg, "%.5f%%" % (((meanLong/avg)-1.0)*100))
   
        else:
            print("Error avg did not converge.")
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

    # Stop continuos e-mode
    printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(0)])

    fscal = "---"
    if printer.printerProfile.hasValue("filSensorCalibration"):
        fscal = "%.4f" % printer.printerProfile.getFilSensorCalibration()

    print("Done, average ratio/calibration value: %.4f (%s)" % (crossAvg.longAvg(), fscal))

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

    print("circum:", circ, "feedrate:", feedrate)

    printer.commandInit()

    # Disable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])

    # Disable temp-flowrate limit
    util.downloadDummyTempTable(printer)

    startPos = parser.getPos()[A_AXIS]

    readings = {}
    measurements = []

    print("running feeder with %.2f mm/s and %.2f mm distance" % (feedrate, d))

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

        print("fsreadings:")
        pprint.pprint(fsreadings)

        allZero = True
        for (ts, dy) in fsreadings:

            readings[ts] = dy

            if dy:
                allZero = False

        if allZero:
            break

        t = time.time()

    timeStamps = list(readings.keys())
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

    print("writing testFeederUniformity_dataset.py:")
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
            print("Exec gcode: '%s'" % g)
            parser.execute_line(g)

    planner.finishMoves()

    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])

    printer.waitForState(StateInit)

####################################################################################################

def feedrateRampUp(printer, startSpeed, endSpeed):

    while startSpeed < endSpeed:

        print(f"feedrateRampUp(): set speed: {startSpeed}")

        printer.sendCommandParamV(CmdContinuous,
            [ packedvalue.uint8_t(dimBitsIndex["A"]),
              packedvalue.uint16_t(util.eTimerValue(printer, startSpeed))])

        startSpeed += 0.25
        time.sleep(0.1)

####################################################################################################

def feedrateRampDown(printer, startSpeed, endSpeed):

    while startSpeed > endSpeed:

        print(f"feedrateRampDown(): set speed: {startSpeed}")

        printer.sendCommandParamV(CmdContinuous,
            [ packedvalue.uint8_t(dimBitsIndex["A"]),
              packedvalue.uint16_t(util.eTimerValue(printer, startSpeed))])

        startSpeed -= 0.25
        time.sleep(0.1)

####################################################################################################

def heatupAndAveragePWM(printer, t1):

    pwmAvg = movingavg.MovingAvg(100, 128)
    printer.heatUp(HeaterEx1, round(t1*1.025+0.5))

    while True:

        status = printer.getStatus()
        pwmAvg.add(status.pwmOutput)

        print("\rTemp: %.2f (%.2f), PWM: %d" % (status.t1, t1, status.pwmOutput), end='')

        if status.t1 >= t1:
            print("\n")
            return round(pwmAvg.mean() + 0.5)

        time.sleep(0.1)

####################################################################################################

def measureTempFlowrateCurve(args, printer, parser, planner):

    util.workingPos(args, printer, parser, planner) #dft

    aFilament = planner.matProfile.getMatArea()

    # Disable temp-flowrate limit
    util.downloadDummyTempTable(printer)

    printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(100)])

    # start with 1mm³/s
    # XXX speed too high for small nozzles!?
    startFeedrate = (args.flowrate or 1.0) / aFilament

    dt = printer.printerProfile.getFilSensorIntervalI()
    pcal = printer.printerProfile.getFilSensorCalibration()

    ####################################################################################################
    # * set initial temp wait for min temp
    # * start measurement loop
    ####################################################################################################

    # Running average of hotend temperature
    tempAvg = movingavg.MovingAvg(10)
    # Running average of feeder grip
    e_steps_per_mm = printer.printerProfile.getStepsPerMMI(A_AXIS)
    circum = printer.printerProfile.getFeederWheelCircumI()
    eStepsPerRound = circum * e_steps_per_mm;

    printer.sendCommandParamV(
            CmdSetFilSensorConfig, (
                intmath.fsCalibration(pcal),
                packedvalue.uint16_t(int(eStepsPerRound/10))
                )
            )

    print("eStepsPerRound:", e_steps_per_mm, circum, eStepsPerRound)

    flowAvg = movingavg.MovingAvgReadings(10)

    data = []

    ####################################################################################################

    # Minimal extruder speed
    minStepRate = (fTimer / maxTimerValue16) + 1
    minFeedrate = minStepRate / e_steps_per_mm
    print(f"Minimal extruder speed: {minFeedrate}")

    feedrate = startFeedrate
    currentFeedrate = minFeedrate
    lastGoodFlowrate = 0

    # for t1 in [planner.matProfile.getHotendBaseTemp(), planner.matProfile.getHotendMaxTemp()]:
    for t1 in [planner.matProfile.getHotendGoodTemp(), planner.matProfile.getHotendMaxTemp()]:

      print("\nHeating up to target temp:", t1)
      # printer.heatUp(HeaterEx1, t1, wait=t1, log=True) #dft
      pwm = heatupAndAveragePWM(printer, t1)

      # Set extruder motor speed
      print(f"\nRamp up extruder motor from {currentFeedrate} mm/s to {feedrate} mm/s")
      # printer.sendCommandParamV(CmdContinuous,
            # [ packedvalue.uint8_t(dimBitsIndex["A"]),
              # packedvalue.uint16_t(util.eTimerValue(printer, feedrate))])
      feedrateRampUp(printer, currentFeedrate, feedrate)

      tStart = time.time()
      startup = 5.0       # initial time for averages to settle

      # Fix pwm value, enter *pwmMode*
      # status = printer.getStatus()
      # pwm = status.pwmOutput

      print("Fixed PWM:", pwm)

      printer.setTempPWM(HeaterEx1, pwm)

      tempAvg.preload(t1)
      flowAvg.preload(1.0);

      while True:

        status = printer.getStatus()
        tempAvg.add(status.t1)

        fsreadings = printer.getFSReadings()
        flowAvg.addReadings(fsreadings, pcal)

        t1Avg = tempAvg.mean()
        r = flowAvg.mean()

        targetFlowRate = feedrate * aFilament

        currentFlowrate = targetFlowRate * r

        print("\rTemp: %.2f, fixed PWM: %d, target flowrate: %.2f mm³/s, actual flowrate: %.2f mm³/s, grip: %.2f" % \
                (t1Avg, pwm, targetFlowRate, currentFlowrate, r), end='')
        sys.stdout.flush()

        # debug
        if r > 2:

            print("mean, pcal", pcal)

            print("fsreadings:", fsreadings)

            print("index:", flowAvg.index)
            print("navg:", flowAvg.navg)
            print("nValues:", flowAvg.nValues)
            print("array:", flowAvg.array)
            assert(0)

        if startup > 0:
            time.sleep(dt)
            startup -= dt
            continue

        if r >= args.mingrip:

            # Increase feedrate by max 1% and min 0.01% mm per second
            frincrease = feedrate * max(0.01 * (max(r, args.mingrip) - args.mingrip), 0.0001)
            feedrate += frincrease
            # print "\nincreased feedrate by %.3f to %.3f" % (frincrease, feedrate)
        
            # Set new feedrate:
            printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(util.eTimerValue(printer, feedrate))])

            lastGoodFlowrate = currentFlowrate

        else:

            #
            # Flowrate got to high for this PWM/temperature values, do next step or exit measurement.
            #
            print("\nEnd step, average flowrate: %.2f mm³/s at pwm: %d, temp: %.1f °C, ratio: %.2f, break" % \
                    (lastGoodFlowrate, pwm, t1Avg, r))
            data.append( (lastGoodFlowrate, pwm, t1Avg) )

            # Slow down extruder while heating to upper temp to save filament
            print(f"\nSlow down extruder motor from {feedrate} mm/s to {startFeedrate} mm/s")
            feedrateRampDown(printer, feedrate, startFeedrate/2)
            currentFeedrate = startFeedrate/2

            # Enable PID mode, leave *pwmMode*
            printer.setTempPWM(HeaterEx1, 0)
            break

        time.sleep(dt)

    ####################################################################################################

    printer.coolDown(HeaterEx1) #dft

    # Slow down E move
    # while feedrate > 1:
        # feedrate -= 0.1
        # printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(util.eTimerValue(printer, feedrate))])
        # time.sleep(0.1)
    print(f"\nSlow down extruder motor from {currentFeedrate} mm/s to {minFeedrate} mm/s")
    feedrateRampDown(printer, currentFeedrate, minFeedrate)

    # Stop continuos e-mode
    printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(0)])

    ####################################################################################################
    dfr = data[1][0] - data[0][0]
    dpwm = data[1][1] - data[0][1]
    dtemp = data[1][2] - data[0][2]

    #
    # Write measured material profile (json)
    #
    s =  """{\n"""
    s += """  "properties_%02d" : {\n""" % int(planner.nozzleProfile.getSizeI()*100)
    s += """    "version": %d,\n""" % printer.printerProfile.getHwVersionI()
    s += """    "mingrip": %.2f,\n\n""" % args.mingrip
    s += """    "# Material properties:",\n"""
    s += """    "# measure 1 (into-air):",\n"""
    s += """    "# a1 for pwm",\n"""
    s += """    "Kpwm": %.4f,\n""" % (dfr / dpwm) #dft
    s += """    "# a1 for temp",\n"""
    s += """    "Ktemp": %.4f,\n""" % (dfr / dtemp)

    s += """    "# a0 for pwm",\n"""
    s += """    "P0pwm": %.4f,\n""" % data[0][1]
    s += """    "# a0 for temp",\n"""
    s += """    "P0temp": %.4f,\n""" % data[0][2]

    s += """    "# feedrate at a0",\n"""
    s += """    "FR0pwm": %.4f\n""" % data[0][0]
    s += """  }\n"""
    s += """}\n"""

    print("\nMaterial properties:\n\n", s)

    fn = "./mat-profile1.json"
    f = open(fn, "w")
    f.write(s)
    f.close()
    print("Data written to: ", fn)

    printer.sendCommand(CmdDisableSteppers)
    printer.coolDown(HeaterEx1, wait=100, log=True)
    printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(0)])

####################################################################################################

# Helper for measureTempFlowrateCurve2(), similar to startPrint()
def xstartPrint(args, printer, parser, planner, t1):

        ddhome.home(args, printer, parser, planner)

        t0 = planner.matProfile.getBedTemp()
        t0Wait = min(t0, printer.printerProfile.getWeakPowerBedTemp())

        # Send heat up  command
        print("\nHeating bed (t0: %d)...\n" % t0)
        printer.heatUp(HeaterBed, t0, log=True)

        printer.erase(0)

        planner.setPrintMode(PrintModePrinting)

        # Disable temp-flowrate limit
        util.downloadDummyTempTable(printer)

        (f, _) = parser.preParse(args.gfile)

        checkTime = time.time() + 2

        StateWeakPreheat = 0
        StateHeatBed = 1
        StateStarting = 3

        state = StateWeakPreheat

        while True:

            if f:
                line = f.readline()
                if line:
                    parser.execute_line(line)
                else:
                    # Reading done
                    print("Parsed all gcode lines.")

                    # Add commands to switch off heaters
                    util.endOfPrintHeaterOff(planner, 0)

                    planner.finishMoves()
                    f = None
            else:
                time.sleep(0.5)

            if time.time() > checkTime:

                status = printer.getStatus()
                printer.ppStatus(status)
                printer.top()

                #
                # Check temp and start print
                #
                if state == StateWeakPreheat:
                    
                    if status.t0 >= t0Wait:
                        # Start pre-heating of hotend, bed is still heating
                        print("\nPre-Heating extruder %.2f (t1: %d)...\n" % (t1/2.0, t1))
                        printer.heatUp(HeaterEx1, t1//2)
                        state = StateHeatBed

                elif state == StateHeatBed:

                    # Wait until entire stepdata is sent and bed is 
                    # heated.
                    if (not f) and (status.t0 >= t0):
                        # Heat hotend if bed is at temp
                        print("\nHeating extruder (t1: %d)...\n" % t1)
                        tempRamp = printer.heatUpRamp(HeaterEx1, t1)
                        state = StateStarting

                elif state == StateStarting:

                    try:
                        next(tempRamp)
                    except StopIteration:
                        print("\nStarting print...\n")

                        # Send print start command
                        printer.startPrint()
                        break

                checkTime = time.time() + 1.5

####################################################################################################
def measureTempFlowrateCurve2(args, printer, parser, planner):

    print("")
    print("measureTempFlowrateCurve2():")
    print("")

    # * start print slowed down
    # * increase flowrate on each layer
    # * monitor feeder grip and break loop
    #   if grip below minGrip value

    # Read data from first measurement step
    fn = "./mat-profile1.json"
    s = ""
    try:
        f = open(fn)
    except IOError as ex:
        print(f"Error, cant read data from first measurement step ({fn}), exception:", ex)
        return

    nozzleProp = "properties_%02d" % int(planner.nozzleProfile.getSizeI()*100)
    m1Data = util.jsonLoad(f)[ nozzleProp ]
    print("Data read from measure1: ", fn, m1Data)
   
    fixedPwm = int(m1Data['P0pwm'])

    t1 = planner.matProfile.getHotendGoodTemp()

    ####################################################################################################

    aFilament = planner.matProfile.getMatArea()

    e_steps_per_mm = printer.printerProfile.getStepsPerMMI(A_AXIS)

    nAvg = 10

    # Running average of hotend temperature and pwm
    tempAvg = movingavg.MovingAvg(nAvg, t1)
    flowAvg = movingavg.MovingAvg(nAvg)
    gripAvg = movingavg.MovingAvgReadings(nAvg, 1.0)

    # Running average of feeder grip
    e_steps_per_mm = printer.printerProfile.getStepsPerMMI(A_AXIS)
    circum = printer.printerProfile.getFeederWheelCircumI()
    eStepsPerRound = circum * e_steps_per_mm;
    pcal = printer.printerProfile.getFilSensorCalibration()

    print(f"pcal: {pcal}")

    printer.sendCommandParamV(
            CmdSetFilSensorConfig, (
                intmath.fsCalibration(pcal),
                packedvalue.uint16_t(int(eStepsPerRound/10))
                )
            )

    print("eStepsPerRound:", e_steps_per_mm, circum, eStepsPerRound)

    ####################################################################################################

    # slow down firmware
    A = 2000000
    y0 = 2.5 * 1024.0
    y1 = 1024.0

    x0 = A / y0
    x1 = A / y1

    x_range = x1 - x0
    x_step = x_range / 20

    x = x0
    y = A / x
    print("x, y:", x, y)
    printer.sendCommandParamV(CmdSetSlowDown, [packedvalue.uint32_t(int(y))])

    ####################################################################################################
    #
    # Start print:
    #
    xstartPrint(args, printer, parser, planner, t1) #dft

    lastEPos = 0.0
    lastTime = 0.0

    # Fix pwm value, enter *pwmMode*
    print("Setting fixed pwm:", fixedPwm)
    printer.setTempPWM(HeaterEx1, fixedPwm)

    #
    # Wait until nozzle lowered/bed lifted.
    #
    # Assuming first layer starts when Z-pos is below 1mm, this will
    # not work if first layer is thicker than 1mm.
    #
    # XXX add timeout here, deadlock 
    print("\nPrint started, waiting for start of first layer...")
    curPosMM = util.getVirtualPos(printer, parser)
    # """
    while curPosMM.Z > 1.0: #dft
        print("waiting for first layer, Z pos:", curPosMM.Z)
        status = printer.getStatus()
        printer.ppStatus(status)

        time.sleep(1)
        curPosMM = util.getVirtualPos(printer, parser)
    # """

    #
    # Wait till print reaches some (hardcoded) height to reduce heating effect of bed.
    #
    print("\nWaiting for layerheight 2mm ...")
    # XXX add timeout here, deadlock 
    # """
    while curPosMM.Z < 2: #dft
        print("waiting for layerheight 2mm , Z pos:", curPosMM.Z)
        status = printer.getStatus()
        printer.ppStatus(status)

        lastEPos = status.ePos
        lastTime = time.time()

        time.sleep(1)
        curPosMM = util.getVirtualPos(printer, parser)
    # """

    # Disable flowrate limit
    print("Disable flowrate limiter...")
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])

    testOk = False

    #
    # Increase speed while monitoring feeder slippage
    #
    while True:

        status = printer.getStatus()
        tempAvg.add(status.t1)

        fsreadings = printer.getFSReadings()
        gripAvg.addReadings(fsreadings, pcal)

        t1Avg = tempAvg.mean()
        gAvg = gripAvg.mean()

        t1Avg = tempAvg.mean()

        # Current target flowrate
        ePos = status.ePos
        tim = time.time()

        deltaE = ePos - lastEPos
        deltaTime = tim - lastTime

        deltaEmm = float(deltaE)/e_steps_per_mm
        flowrate = (deltaEmm*aFilament)/deltaTime

        flowAvg.add(flowrate)
        flowrateAvg = flowAvg.mean()

        # Current measured flowrate
        frAvg = flowrateAvg * gAvg

        lastEPos = ePos
        lastTime = tim

        print("\rTemp: %.2f, fixed PWM: %.2f, slowdown: %5d, target flowrate: %.2f, flowrate: %.2f mm³/s, grip: %.2f" % \
                (t1Avg, fixedPwm, y, flowrateAvg, frAvg, gAvg), end='')

        if tempAvg.valid():

            if not printer.stateMoving(status):
                print("\nError: testprint ended before measurement was done!")
                break

            if gAvg <= args.mingrip:
                print("\nBreak loop, avg grip: %.2f, mingrip %.2f" % (gAvg, args.mingrip))
                testOk = True
                break

            pos = util.getVirtualPos(printer, parser)
            if pos.Z > curPosMM.Z:

                # layer change, speed up print
                x = x+x_step
                y = int(A / x)

                if y < (1024/2): # cap at two times speedup 
                    # Test did not converge even with two times speedup, flowrate in gcode to low
                    print("\nError: test did not converge, even with %.2f times spedup!" % (1024/y))
                    print("Increase flowrate in sliced model to fix this.")
                    break

                printer.sendCommandParamV(CmdSetSlowDown, [packedvalue.uint32_t(y)])
                curPosMM = pos

        time.sleep(1)

    ####################################################################################################

    # Stop print
    print("\nSoft-stop...")
    if printer.stateMoving(status):
        printer.sendCommand(CmdSoftStop)

    # Wait till print is cancelled.
    while printer.stateMoving(status):
        time.sleep(2)
        status = printer.getStatus()
        printer.ppStatus(status)

    # Stop pwm mode
    printer.setTempPWM(HeaterEx1, 0)

    ddhome.home(args, printer, parser, planner) #dft

    ####################################################################################################

    print("# Avg: P0temp: %.2f, P0pwm: %.2f, grip: %.2f, FR0pwm: %.2f mm³/s" % (t1Avg, fixedPwm, gAvg, frAvg))

    if testOk:

        #
        # Write measured material profile (json)
        #
        s =  """{\n"""
        s += """  "%s" : {\n""" % nozzleProp
        s += """    "version": %d,\n""" % printer.printerProfile.getHwVersionI()
        s += """    "mingrip": %.2f,\n\n""" % args.mingrip
        s += """    "# Material properties:",\n"""
        s += """    "# measure 1 (into-air):",\n"""
        s += """    "# a1 for pwm",\n"""
        s += """    "Kpwm": %.4f,\n""" % m1Data['Kpwm']
        s += """    "# a1 for temp",\n"""
        s += """    "Ktemp": %.4f,\n""" % m1Data['Ktemp']

        s += """    "# a0 for pwm",\n"""
        s += """    "P0pwm": %d,\n""" % fixedPwm
        s += """    "# a0 for temp",\n"""
        s += """    "P0temp": %.4f,\n""" % m1Data['P0temp']

        s += """    "# feedrate at a0",\n"""
        s += """    "FR0pwm": %.4f,\n""" % m1Data['FR0pwm']

        s += """    "# measure 2 (printing):",\n"""
        s += """    "# a0 for pwm:",\n"""
        s += """    "P0pwmPrint": %d,\n""" % fixedPwm
        s += """    "# a0 for temp",\n"""
        s += """    "P0tempPrint": %.4f,\n""" % t1Avg
        s += """    "# feedrate at a0",\n"""
        s += """    "FR0pwmPrint": %.4f\n""" % frAvg
        s += """  }\n"""
        s += """}\n"""

        print("\nMaterial properties (printing):\n\n", s)

        fn = "./mat-profile2.json"
        f = open(fn, "w")
        f.write(s)
        f.close()
        print("Data written to: ", fn)

    else:

        print("Test was not successful!")

    # Re-enable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(1)])

    printer.sendCommand(CmdDisableSteppers)
    printer.coolDown(HeaterEx1, wait=100, log=True)
    printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(0)])

    printer.sendCommandParamV(CmdSetSlowDown, [packedvalue.uint32_t(1024)])

####################################################################################################








