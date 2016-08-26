# -*- coding: utf-8 -*-
#
#/*
# This file is part of ddprint - a direct drive 3D printer firmware.
# 
# Copyright 2016 erwin.rieger@ibrieger.de
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


from argparse import Namespace

from ddprintconstants import *
from ddconfig import *
from ddprofile import PrinterProfile # , MatProfile, NozzleProfile

import ddprintutil as util
import math


if debugPlot:
    import pickle, matplotlib.pyplot as plt 
    from matplotlib import collections as mc

#####################################################################

class DebugPlotSegment (object):

    def __init__(self, y1, y2=None, color="grey"):
        self.y1 = y1
        self.y2 = y2
        self.color = color

class DebugPlot (object):

    def __init__(self, nr):

        self.plotfile = "/tmp/ddplot_%04d.pkl" % nr

        self.plot1 = self.makePlot()
        self.plot2 = self.makePlot()

    def makePlot(self):

        pl = Namespace()
        pl.time = 0.01
        pl.Lines = []
        pl.Colors = []
        pl.Styles = []
        pl.Ticks = []
        return pl

    # def newMove(self):
        # self.colors += self.segcol

    def plot1Tick(self, y, nr):

        self.plot1.Ticks.append((self.plot1.time, y, nr))

    def plot1Segments(self, dt, segspec):

        self.plotSegments(self.plot1, dt, segspec)

    def plot2Segments(self, dt, segspec):

        self.plotSegments(self.plot2, dt, segspec)

    def plotSegments(self, plot, dt, segspec):

        for seg in segspec:

            style = "solid"
            y2 = seg.y2
            t = dt
            if y2 == None:
                y2 = seg.y1
                if (t > 0.05):
                    style = "dashed"
                    t = 0.05

            plot.Lines.append(((plot.time, seg.y1), (plot.time+t, y2)))
            plot.Colors.append(seg.color)
            plot.Styles.append(style)

        plot.time += t

    def close(self):
        pickle.dump((self.plot1, self.plot2), open(self.plotfile, "wb"))


#####################################################################

class Advance (object):

    __single = None 

    def __init__(self, planner): # , args, gui=None):

        self.planner = planner
        self.printer = planner.printer

        #
        # Extruder advance, max allowed E-acceleration for the given E-Jerk
        #
        # dvin = dvout + ka * acceleration [ m/s + s * m/s² = m/s ]
        #
        self.kAdv = 0.035 # [s] # xxx move to material profile
        self.kAdv = 0.010 # [s]
        # self.advAccel = ((sel.jerk[A_AXIS] / 1000.0) / self.kAdv) * 1000
        advJerk = planner.jerk[A_AXIS]
        # self.advAccel = self.advJerk / self.kAdv
        maxEAccel = advJerk / self.kAdv

        #
        # Limit E-acceleration by DEFAULT_ACCELERATION/DEFAULT_MAX_ACCELERATION
        #
        print "ADV: max E-acceleration:", maxEAccel, " [mm/s²]"
        self.maxEAccel = min(maxEAccel, MAX_AXIS_ACCELERATION_NOADV[A_AXIS])
        print "ADV: max E-acceleration, limited:", self.maxEAccel, " [mm/s²]"

        self.maxAxisAcceleration = MAX_AXIS_ACCELERATION_NOADV[:3] + [self.maxEAccel, self.maxEAccel]

        # self.plotfile = None

        # Sum of skipped small accelration ramps
        self.advSum = 0
        # Likewise for decel
        # self.endSum = 0

    def eJerk(self, accel):
        return accel * self.kAdv

    def planPath(self, path):

        if debugMoves:
            print "***** Start planPath() *****"

        # self.startSum = self.endSum = 0
        self.advSum = 0

        if debugPlot: #  and not self.plotfile:
            self.plottime = 1
            # self.plotfile=open("/tmp/ddpath_%d.ascii" % path[0].moveNumber, "w")
            # self.plotfile.write("#time trig vXY vExt\n")
            # self.plotfile.write("0 0 0 0\n");
            self.plotfile = DebugPlot(path[0].moveNumber)

        # First move of this path, startspeed is jerkspeed/2
        # path[0].setNominalJerkStartSpeed(self.jerk.scale(0.5))
        # Last move of this path, ensdpeed is jerkspeed/2
        # path[-1].setNominalJerkEndSpeed(self.jerk.scale(0.5))

        # First move
        path[0].setNominalStartFr(0.0)
        # Last move
        path[-1].setNominalEndFr(0.0)

        if UseExtrusionAutoTemp:
            self.pathData.time = 0 # Reset path time
            self.pathData.extrusionAmount = 0
            self.pathData.lastTemp = MatProfile.getHotendBaseTemp()

        lastMove = path[0]

        # Step 1: join moves forward
        for move in path[1:]:
            util.joinSpeed(lastMove, move, self.planner.jerk, self.maxAxisAcceleration)
            lastMove = move

        # Sanity check
        for move in path:
            move.pprint("sanicheck")
            move.sanityCheck(self.planner.jerk)

        # Step 2: join moves backwards
        self.joinMovesBwd(path)

        # Sanity check
        for move in path:

            move.pprint("sanicheck")
            move.sanityCheck(self.planner.jerk)

        # Step 3: handle extruder advance

        # for move in path[1:]:
            # util.xxx(lastMove, move, self.jerk, self.min_speeds)
            # lastMove = move

        # assert(0); # self.streamMoves(path)
        ################################################# StreamMoves #####################################################
        # def streamMoves(self, moves, finish = False):

        if debugMoves:
            print "Streaming %d moves..." % len(path)

        # if debugPlot and not self.plotfile:
            # self.plottime = 0
            # self.plotfile=open("/tmp/accel_%d.plt" % moves[0].moveNumber, "w")
            # self.plotfile.write("set grid\n")
            # self.plotfile.write("set label 2 at  graph 0.01, graph 0.95 'Note: Trapez top not included, e-only moves green.'\n")
            # self.plotfile.write('plot "-" using 1:2:3 with linespoints lc variable\n')

        for move in path:

            move.pprint("sanicheck")
            move.sanityCheck(self.planner.jerk)

            self.planAcceleration(move)

            self.planAdvance(move)

            self.planSteps(move)


            # xxx todo move to own function
            self.plotfile.plot1Tick(move.getReachedFr(), move.moveNumber)

            at = move.accelTime()
            lt = move.linearTime()
            dt = move.decelTime()

            if at:

                self.plotfile.plot1Segments(at, (
                    DebugPlotSegment(move.getStartFr(), move.getReachedFr(), "green"),
                    DebugPlotSegment(move.getStartFeedrateV()[A_AXIS], move.getReachedFeedrateV()[A_AXIS], "green"),
                    ))

            if lt:

                self.plotfile.plot1Segments(lt, (
                    DebugPlotSegment(move.getReachedFr(), color="blue"),
                    DebugPlotSegment(move.getReachedFeedrateV()[A_AXIS], color="blue"),
                    ))

            if dt:

                self.plotfile.plot1Segments(dt, (
                    DebugPlotSegment(move.getReachedFr(), move.getEndFr(), "red"),
                    DebugPlotSegment(move.getReachedFeedrateV()[A_AXIS], move.getEndFeedrateV()[A_AXIS], "red"),
                    ))

            if at:

                self.plotfile.plot2Segments(0, (
                    DebugPlotSegment(move.getStartFeedrateV()[A_AXIS], move.advanceData.startEFeedrate(), "green"),
                    ))
                self.plotfile.plot2Segments(at, (
                    DebugPlotSegment(move.getStartFeedrateV()[A_AXIS], move.getReachedFeedrateV()[A_AXIS]),
                    DebugPlotSegment(move.advanceData.startEFeedrate(), move.advanceData.startEReachedFeedrate(), "green"),
                    ))
                self.plotfile.plot2Segments(0, (
                    DebugPlotSegment(move.advanceData.startEReachedFeedrate(), move.getReachedFeedrateV()[A_AXIS], "green"),
                    ))

            if lt:
                self.plotfile.plot2Segments(lt, (
                    DebugPlotSegment(move.getReachedFeedrateV()[A_AXIS]),
                    ))

            if dt:

                self.plotfile.plot2Segments(0, (
                    DebugPlotSegment(move.getReachedFeedrateV()[A_AXIS], move.advanceData.endEReachedFeedrate(), "red"),
                    ))
                self.plotfile.plot2Segments(dt, (
                    DebugPlotSegment(move.getReachedFeedrateV()[A_AXIS], move.getEndFeedrateV()[A_AXIS]),
                    DebugPlotSegment(move.advanceData.endEReachedFeedrate(), move.advanceData.endEFeedrate(), "red"),
                    ))
                self.plotfile.plot2Segments(0, (
                    DebugPlotSegment(move.advanceData.endEFeedrate(), move.getEndFeedrateV()[A_AXIS], "red"),
                    ))

            continue # xxx work
            assert(0) # work

            #
            # Collect some statistics
            #
            self.pathData.maxExtrusionRate.stat(move)

            #
            # Collect moves if AutoTemp
            #
            if UseExtrusionAutoTemp:
                if move.xsExtrudingMove(A_AXIS):
                    # Collect moves and sum up path time
                    self.pathData.time += move.getTime()

                    # Sum extrusion volume
                    self.pathData.extrusionAmount += move.getAdjustedExtrusionVolume(A_AXIS, NozzleProfile, MatProfile)

                self.pathData.atMoves.append(move)

            else:

                self.streamMove(move)

            move.streamed = True

            # Help garbage collection
            move.lastMove = errorMove
            move.nextMove = errorMove
       


        if debugPlot:
            # self.plotfile.write(1)
            # self.plotfile.write()
            self.plotfile.close()

        return # xxx work
        assert(0)

        if UseExtrusionAutoTemp:

            if self.pathData.time >= ATInterval or finish:

                if self.pathData.time > 0:

                    # Compute temperature for this segment and add tempcommand into the stream
                    # Average speed:
                    avgSpeed = self.pathData.extrusionAmount / self.pathData.time

                    # UseAutoTemp: Adjust temp between Tbase and HotendMaxTemp, if speed is greater than 20 mm/s
                    # UseExtrusionAutoTemp: Adjust temp between Tbase and HotendMaxTemp, if speed is greater than 5 mm³/s
                    newTemp = MatProfile.getHotendBaseTemp() # Extruder 1 temp
                    # extrusionLow = self.ExtrusionAmountLow * pow(NozzleProfile.getSize(), 2)
                    if avgSpeed > self.ExtrusionAmountLow:
                        f = MatProfile.getAutoTempFactor()
                        # newTemp += min((avgSpeed - self.ExtrusionAmountLow * pow(NozzleProfile.getSize(), 2)) * f, ATMaxTempIncrease)
                        newTemp += (avgSpeed - self.ExtrusionAmountLow) * f
                        # newTemp *= 1.15 # xxx sync withtemp-speed-adjust
                        newTemp = min(newTemp, MatProfile.getHotendMaxTemp())

                    if debugAutoTemp:
                        print "AutoTemp: collected %d moves with %.2f s duration." % (len(self.pathData.atMoves), self.pathData.time)
                        print "AutoTemp: amount: %.2f, avg extrusion rate: %.2f mm³/s." % (self.pathData.extrusionAmount, avgSpeed)
                        print "AutoTemp: new temp: %.2f." % (newTemp)
                        self.pathData.maxExtrusionRate.avgStat(avgSpeed)

                    if newTemp != self.pathData.lastTemp and self.args.mode != "pre":

                        # Schedule target temp command
                        self.printer.sendCommandParamV(
                            CmdSyncTargetTemp,
                            [packedvalue.uint8_t(HeaterEx1), packedvalue.uint16_t(newTemp)])

                        self.pathData.lastTemp = newTemp

                for move in self.pathData.atMoves:
                    self.streamMove(move)
                   
                self.pathData.atMoves = []
                self.pathData.time = self.pathData.extrusionAmount = 0 # Reset path time
                # assert(0)

        # if debugPlot:
            # self.plotfile.write("e\n")
            # self.plotfile.close()
        ################################################# end StreamMoves #####################################################

        if debugMoves:
            print "***** End planPath() *****"

    #
    # xxx
    # Letzter move in revMoves ist entweder:
    #   * letzer move des pfades, in dem fall ist endspeed bereits berechnet
    #     und gleich jerkspeed.
    #   * der move vor dem "isolation move", in diesem falle ist endspeed noch nicht 
    #     gesetzt, aber die min/max grenzen
    #
    def joinMovesBwd(self, moves):

        if debugMoves:
            print "***** Start joinMovesBwd() *****"

        index = len(moves) - 1
        while index >= 0:

            move = moves[index]
            index -= 1

            if debugMoves: 
                move.pprint("joinMovesBwd")

            # Check, if breaking between startspeed and endspeed of
            # move is possible with the given acceleration and within the 
            # given distance:

            endSpeedS = move.getEndFr()

            allowedAccel = move.getMaxAllowedAccel(self.maxAxisAcceleration)
            maxAllowedStartSpeed = util.vAccelPerDist(endSpeedS, allowedAccel, move.distance)

            # print "joinMovesBwd, startspeed, max startspeed: ", move.getStartFr(), maxAllowedStartSpeed

            if maxAllowedStartSpeed >= move.getStartFr():
                # good, move is ok
                continue

            if debugMoves: 
                print "Startspeed of %.5f is to high to reach the desired endspeed of %.5f." % (move.getStartFr(), endSpeedS)
                print "Max. allowed startspeed: %.5f." % maxAllowedStartSpeed

            if move.lastMove:

                #
                # Adjust endspeed of the previous move, also.
                #

                factor = maxAllowedStartSpeed / move.getStartFr()
                # print "factor: ", factor
                # Adjust endspeed of last move:
          
                assert(factor < 1)

                # XXX einfacher algo, kann man das besser machen (z.b. mit jerk-berechnung,
                # vector subtraktion oder so?)
                move.lastMove.setTrueEndFr(move.lastMove.getEndFr() * factor)

            # Adjust startspeed of this move:
            move.setTrueStartFr(maxAllowedStartSpeed)

        if debugMoves:
            print "***** End joinMovesBwd() *****"

    def planAcceleration(self, move):

        if debugMoves: 
            move.pprint("planAcceleration")
            print "***** Start planAcceleration() *****"

        move.state = 2

        allowedAccel = allowedDeccel = move.getMaxAllowedAccel(self.maxAxisAcceleration)

        #
        # Check if the speed difference between startspeed and endspeed can be done with
        # this acceleration in this distance
        #
        startSpeedS = move.getStartFr()
        endSpeedS = move.getEndFr()

        deltaSpeedS = endSpeedS - startSpeedS

        if abs(deltaSpeedS) > 0.001:
       
            ta = abs(deltaSpeedS) / allowedAccel

            if deltaSpeedS > 0:
                # acceleration
                sa = util.accelDist(startSpeedS, allowedAccel, ta)
            else:
                # decceleration
                sa = util.accelDist(startSpeedS, -allowedAccel, ta)
      
            if (sa - move.distance) > 0.001:
                print " 0.5 * %f * pow(%f, 2) + %f * %f" % (allowedAccel, ta, startSpeedS, ta)
                print "VStart %f mm/s kann nicht innerhalb von %f mm auf Endgeschwindigkeit %f mm/s gebracht werden!" % (startSpeedS, move.distance, endSpeedS)
                print "Dafür werden %f mm benötigt" % sa
                assert(0)

        #
        # Compute distance to accel from start speed to nominal speed:
        #

        ta = 0.0
        sa = 0.0
        # if move.getStartFr().fr != None:
        deltaStartSpeedS = move.feedrateS - startSpeedS

        if deltaStartSpeedS:

            ta = deltaStartSpeedS / allowedAccel
            # print "accel time (for %f mm/s): %f [s]" % (deltaStartSpeedS, ta)

            # debug Check axxis acceleration
            deltaSpeedV = move.getFeedrateV(deltaStartSpeedS)
            for dim in range(5):
                if deltaSpeedV[dim] != 0:
                    dimAccel = deltaSpeedV[dim] / ta
                    if (dimAccel / self.maxAxisAcceleration[dim]) > 1.001:
                        print "dim %d verletzt max accel: " % dim, dimAccel, " > ", self.maxAxisAcceleration[dim]
                        assert(0)
            #end debug

            sa = util.accelDist(startSpeedS, allowedAccel, ta)

        #
        # Compute distance to deccel from nominal speed to endspeed:
        #
        tb = 0.0
        sb = 0.0
        # if move.getEndFr().fr != None:
        deltaEndSpeedS = move.feedrateS - endSpeedS                          # [mm/s]

        if deltaEndSpeedS:

            tb = deltaEndSpeedS / allowedAccel                          # [s]
            # print "deccel time (for %f mm/s): %f [s]" % (deltaEndSpeedS, tb)

            # debug Check axxis acceleration
            deltaSpeedV = move.getFeedrateV(deltaEndSpeedS)
            for dim in range(5):
                if deltaSpeedV[dim] != 0:
                    dimDeccel = deltaSpeedV[dim] / tb  
                    if (dimDeccel / self.maxAxisAcceleration[dim]) > 1.001:
                        print "dim %d verletzt max accel: " % dim, dimDeccel, " [mm/s] > ", self.maxAxisAcceleration[dim], " [mm/s]"
                        assert(0)
            # end debug

            sb = util.accelDist(endSpeedS, allowedAccel, tb)

        # print "e_distance: %f, sbeschl, sbrems: %f, %f" % (move.e_distance, sa, sb)

        if move.distance < (sa+sb):

            #
            # Strecke zu kurz, Trapez nicht möglich, geschwindigkeit muss abgesenkt werden.
            #
            if debugMoves:
                print "Trapez nicht möglich: s: %f, sbeschl (%f) + sbrems (%f) = %f" % (move.distance, sa, sb, sa+sb)

            # ??? 
            assert(sa>0 and sb>0)

            nominalSpeedS = move.feedrateS

            # sa = (2 * allowedAccel * move.e_distance - pow(startSpeedS, 2) + pow(endSpeedS, 2)) /(4 * allowedAccel)
            sa = (2 * allowedAccel * move.distance - pow(startSpeedS, 2) + pow(endSpeedS, 2)) / (4 * allowedAccel)
            sb = move.distance - sa

            if debugMoves:
                print "sbeschl, sbrems neu: %f, %f" % (sa, sb)

            # 
            # Geschwindigkeit, die auf strecke sa mit erreicht werden kann
            # 
            v = math.sqrt ( 2 * allowedAccel * sa + pow(startSpeedS, 2) )

            # debug, test
            v2 = math.sqrt ( 2 * allowedAccel * sb + pow(endSpeedS, 2) )
            # print "move.feedrate neu: %f (test: %f, diff: %f)" % (v, v2, abs(v - v2))

            assert( abs(v - v2) < 0.001)

            nominalSpeedV = move.getFeedrateV().scale(v / nominalSpeedS)

            move.accelData.reachedovNominalVVector = nominalSpeedV

            # reachedNominalSpeedS = nominalSpeedV.feedrate3() # [mm/s]
            reachedNominalSpeedS = nominalSpeedV.len5() # [mm/s]

            deltaSpeedS = reachedNominalSpeedS - startSpeedS                          # [mm/s]
            ta = deltaSpeedS / allowedAccel
            # print "ta: ", ta, deltaSpeedS

            deltaSpeedS = reachedNominalSpeedS - endSpeedS                          # [mm/s]
            tb = deltaSpeedS / allowedAccel
            # print "tb: ", tb, deltaSpeedS

            move.setDuration(ta, 0, tb)

            if debugMoves:
                move.pprint("planAcceleration")
                print 
                print "***** End planAcceleration() *****"

            return

        # 
        # Strecke reicht aus, um auf nominal speed zu beschleunigen
        # 

        # print "ta: ", ta, deltaStartSpeedS, sa
        # print "tb: ", tb, deltaEndSpeedS, sb

        nominalSpeed = move.getFeedrateV().len5() # [mm/s]
        slin = move.distance - (sa+sb)
        tlin = slin / nominalSpeed
        # print "tlin: ", tlin, slin
        move.setDuration(ta, tlin, tb)

        if debugMoves:
            move.pprint("planAcceleration")
            print 
            print "***** End planAcceleration() *****"

    def planAdvance(self, move):

        if debugMoves:
            print "***** Start planAdvance() *****"
            move.pprint("planAdvance:")

        reachedFeedrateV = move.getReachedFeedrateV()

        startFeedrateV = move.getStartFeedrateV()
        endFeedrateV = move.getEndFeedrateV()

        startEVelDiff = reachedFeedrateV[A_AXIS] - startFeedrateV[A_AXIS]
        startEAccelSign = util.sign(startEVelDiff)

        endEVelDiff = endFeedrateV[A_AXIS] - reachedFeedrateV[A_AXIS]
        endEAccelSign = util.sign(endEVelDiff)

        print "startEVelDiff, endEVelDiff: ", startEVelDiff, endEVelDiff

        allowedAccelV = move.getMaxAllowedAccelVectorNoAdv()

        #
        # Advance of start-ramp
        #
        at = move.accelTime()
        if at > AdvanceMinRamp:
            move.advanceData.startFeedrateIncrease = self.eJerk(allowedAccelV[A_AXIS]) * startEAccelSign
        elif at > 0:
            # Dont advance very small acceleration ramps, but sum up the missing advance.
            # Note: this simple method works only if every ramp has the same acceleration.
            self.advSum += at * startEAccelSign

        #
        # End of start-ramp
        #
        dt = move.decelTime()
        if dt > AdvanceMinRamp:
            move.advanceData.endFeedrateIncrease = self.eJerk(allowedAccelV[A_AXIS]) * endEAccelSign
        elif dt > 0:
            # Dont advance, sum up like the acceleration ramp.
            self.advSum += dt * endEAccelSign

        print "advSum: ", self.advSum
        assert(self.advSum < AdvanceMinRamp)

        """
        if abs(startEVelDiff) < AdvanceMinERate and abs(endEVelDiff) < AdvanceMinERate:
            # Ignore small ramps
            print "Warning, handle small e-difference!"
            x""
            print "Warning, sum up small extrusion velocity difference!"
            move.startEFeedrate = startFeedrateV[A_AXIS]
            move.startEReachedFeedrate = move.endEReachedFeedrate = reachedFeedrateV[A_AXIS]
            move.endEFeedrate = endFeedrateV[A_AXIS]
            return
            x""

        if abs(startEVelDiff) < AdvanceMinERate or abs(endEVelDiff) < AdvanceMinERate:
            # 
            print "Warning, handle small e-difference!"
            x""
            print "Warning, sum up small extrusion velocity difference!"
            move.startEFeedrate = startFeedrateV[A_AXIS]
            move.startEReachedFeedrate = move.endEReachedFeedrate = reachedFeedrateV[A_AXIS]
            move.endEFeedrate = endFeedrateV[A_AXIS]
            return
            x""

        # move.startEFeedrate = startFeedrateV[A_AXIS] + self.advJerk * startEAccelSign
        move.advanceData.startFeedrateIncrease = self.advJerk * startEAccelSign
        # move.startEReachedFeedrate = reachedFeedrateV[A_AXIS] + self.advJerk * startEAccelSign

        # move.endEReachedFeedrate = reachedFeedrateV[A_AXIS] + self.advJerk * endEAccelSign
        # move.endEFeedrate = endFeedrateV[A_AXIS] + self.advJerk * endEAccelSign
        move.accelData.endFeedrateIncrease = self.advJerk * endEAccelSign

        """

        print "Adv: adjusted E startspeed: %f -> %f" % (startFeedrateV[A_AXIS], move.advanceData.startEFeedrate())
        print "Adv: adjusted E   endspeed: %f -> %f" % (endFeedrateV[A_AXIS], move.advanceData.endEFeedrate())

        if debugMoves:
            print 
            print "***** End planAdvance() *****"

    def planSteps(self, move):

        if debugMoves:
            print "***** Start planSteps() *****"
            move.pprint("PlanSTeps:")

        move.state = 3

        dirBits = 0
        abs_displacement_vector_steps = []

        leadAxis = 0
        leadAxis_value = 0

        print "Warning, disabled extrusion adjust!"

        """
        for i in range(5):
            dirBits += (move.displacement_vector_steps_raw()[i] >= 0) << i
            adjustedDisplacement = move.displacement_vector_steps_adjusted(NozzleProfile, MatProfile, PrinterProfile)

            s = abs(adjustedDisplacement[i])
            if s > leadAxis_value:
                leadAxis = i
                leadAxis_value = s

            abs_displacement_vector_steps.append(s)
        """

        for i in range(5):
            disp = move.displacement_vector_steps_raw()
            dirBits += (disp[i] >= 0) << i # xxx use sign here

            s = abs(disp[i])
            abs_displacement_vector_steps.append(s)
            if s > leadAxis_value:
                leadAxis = i
                leadAxis_value = s

        if dirBits != self.printer.curDirBits:
            move.stepData.setDirBits = True
            move.stepData.dirBits = dirBits
            self.printer.curDirBits = dirBits

        steps_per_mm = PrinterProfile.getStepsPerMM(leadAxis)

        #
        # Bresenham's variables
        #
        deltaLead = abs_displacement_vector_steps[leadAxis]
        move.stepData.setBresenhamParameters(leadAxis, abs_displacement_vector_steps)

        #
        # Create a list of stepper pulses
        #

        # debnegtimer
        nominalSpeed = abs( move.getReachedFeedrateV()[leadAxis] ) # [mm/s]

        reachedSpeedV = move.getReachedFeedrateV()
        reachedSpeedFr = reachedSpeedV.len5()

        # advance
        allowedAccel = abs(move.getMaxAllowedAccel(self.maxAxisAcceleration))

        print "allowedAccel: ", move.getMaxAllowedAccel(self.maxAxisAcceleration), self.maxAxisAcceleration
        accel_steps_per_square_second = allowedAccel * steps_per_mm

        # todo work
        return

        # startSpeed = move.getStartFr()

        # debnegtimer
        v0 = abs(move.getFeedrateV(move.getStartFr())[leadAxis])                # [mm/s]

        # endSpeed = move.getEndFr()

        # debnegtimer
        v1 = abs(move.getFeedrateV(move.getEndFr())[leadAxis])                # [mm/s]

        # debnegtimer
            
        steps_per_second_0 = steps_per_second_accel = v0 * steps_per_mm
        steps_per_second_1 = steps_per_second_deccel = v1 * steps_per_mm
        steps_per_second_nominal = nominalSpeed * steps_per_mm

        #
        # Acceleration variables
        #
        # tAccel = 0  # [s], sum of all acceeration steptimes
        # tAccel mit der initialen zeitspanne vorbelegen, da wir bereits im
        # ersten schleifendurchlauf (d.h. ab t=0) eine beschleunigung haben wollen.
        tAccel = 1.0 / steps_per_second_0  # [s], sum of all acceeration steptimes
        tDeccel = 1.0 / steps_per_second_1

        stepNr = 0

        if debugPlot:
            accelPlotData = []
            deccelPlotData = []

        if not circaf(move.getStartFr(), reachedSpeedFr, 0.1) and not circaf(move.getEndFr(), reachedSpeedFr, 0.1):

            #
            # Acceleration ramp on both sides.
            #
            # Ramp up both sides in parallel to not exeed available steps
            #

            return # work
            assert(0)

            done = False
            while not done and stepNr < deltaLead:

                done = True

                # 
                # Compute acceleration timer values
                # 
                if steps_per_second_accel < steps_per_second_nominal:

                    #
                    # Compute timer value
                    #
                    steps_per_second_accel = min(steps_per_second_0 + tAccel * accel_steps_per_square_second, steps_per_second_nominal)

                    dt = 1.0 / steps_per_second_accel
                    timerValue = int(fTimer / steps_per_second_accel)

                    # print "dt: ", dt*1000000, "[uS]", steps_per_second_accel, "[steps/s], timerValue: ", timerValue

                    if debugPlot:
                        if move.eOnly:
                            accelPlotData.append((steps_per_second_accel/steps_per_mm, 2, dt))
                        else:
                            accelPlotData.append((steps_per_second_accel/steps_per_mm, 1, dt))

                    tAccel += dt

                    if timerValue > maxTimerValue24:
                        move.pprint("PlanSTeps:")
                        print "v0: ", dt*1000000, "[uS]", steps_per_second_accel, "[steps/s], timerValue: ", timerValue
                        print "dt: ", dt*1000000, "[uS]", steps_per_second_accel, "[steps/s], timerValue: ", timerValue
                        assert(0)

                    move.stepData.addAccelPulse(timerValue)
                    stepNr += 1
                    done = False

                if stepNr == deltaLead:
                    break

                #
                # Compute ramp down (in reverse), decceleration
                #
                if steps_per_second_deccel < steps_per_second_nominal:

                    #
                    # Compute timer value
                    #
                    steps_per_second_deccel = min(steps_per_second_1 + tDeccel * accel_steps_per_square_second, steps_per_second_nominal)

                    dt = 1.0 / steps_per_second_deccel
                    timerValue = int(fTimer / steps_per_second_deccel)

                    # print "dt: ", dt*1000000, "[uS]", steps_per_second_deccel, "[steps/s], timerValue: ", timerValue, ", v: ", steps_per_second_deccel/steps_per_mm

                    if debugPlot:
                        if move.eOnly:
                            deccelPlotData.append((steps_per_second_deccel/steps_per_mm, 2, dt))
                        else:
                            deccelPlotData.append((steps_per_second_deccel/steps_per_mm, 1, dt))

                    tDeccel += dt

                    if timerValue > maxTimerValue24:
                        move.pprint("PlanSTeps:")
                        print "v0: ", dt*1000000, "[uS]", steps_per_second_deccel, "[steps/s], timerValue: ", timerValue
                        print "dt: ", dt*1000000, "[uS]", steps_per_second_deccel, "[steps/s], timerValue: ", timerValue
                        assert(0)

                    move.stepData.addDeccelPulse(timerValue)
                    stepNr += 1
                    done = False

        elif not circaf(move.getStartFr(), reachedSpeedFr, 0.1): 

            #
            # Acceleration only
            #

            # 
            # Compute acceleration timer values
            # 
            while steps_per_second_accel < steps_per_second_nominal and stepNr < deltaLead:

                #
                # Compute timer value
                #
                steps_per_second_accel = min(steps_per_second_0 + tAccel * accel_steps_per_square_second, steps_per_second_nominal)

                dt = 1.0 / steps_per_second_accel
                timerValue = int(fTimer / steps_per_second_accel)

                # print "dt: ", dt*1000000, "[uS]", steps_per_second_accel, "[steps/s], timerValue: ", timerValue

                if debugPlot:
                    if move.eOnly:
                        accelPlotData.append((steps_per_second_accel/steps_per_mm, 2, dt))
                    else:
                        accelPlotData.append((steps_per_second_accel/steps_per_mm, 1, dt))

                tAccel += dt

                if timerValue > maxTimerValue24:
                    move.pprint("PlanSTeps:")
                    print "v0: ", dt*1000000, "[uS]", steps_per_second_accel, "[steps/s], timerValue: ", timerValue
                    print "dt: ", dt*1000000, "[uS]", steps_per_second_accel, "[steps/s], timerValue: ", timerValue
                    assert(0)

                move.stepData.addAccelPulse(timerValue)
                stepNr += 1

        else:

            #
            # Decceleration only
            #

            #
            # Compute ramp down (in reverse), decceleration
            #
            while steps_per_second_deccel < steps_per_second_nominal and stepNr < deltaLead:

                return # work
                assert(0)

                #
                # Compute timer value
                #
                steps_per_second_deccel = min(steps_per_second_1 + tDeccel * accel_steps_per_square_second, steps_per_second_nominal)

                dt = 1.0 / steps_per_second_deccel
                timerValue = int(fTimer / steps_per_second_deccel)

                # print "dt: ", dt*1000000, "[uS]", steps_per_second_deccel, "[steps/s], timerValue: ", timerValue, ", v: ", steps_per_second_deccel/steps_per_mm

                if debugPlot:
                    if move.eOnly:
                        deccelPlotData.append((steps_per_second_deccel/steps_per_mm, 2, dt))
                    else:
                        deccelPlotData.append((steps_per_second_deccel/steps_per_mm, 1, dt))

                tDeccel += dt

                if timerValue > maxTimerValue24:
                    move.pprint("PlanSTeps:")
                    print "v0: ", dt*1000000, "[uS]", steps_per_second_deccel, "[steps/s], timerValue: ", timerValue
                    print "dt: ", dt*1000000, "[uS]", steps_per_second_deccel, "[steps/s], timerValue: ", timerValue
                    assert(0)

                move.stepData.addDeccelPulse(timerValue)
                stepNr += 1

        #
        # Linear phase
        #
        timerValue = fTimer / steps_per_second_nominal
        move.stepData.setLinTimer(timerValue)

        if debugPlot:
            self.plotfile.write("# Acceleration:\n")
            for (speed, color, dt) in accelPlotData:
                self.plotfile.write("%f %f %d\n" % (self.plottime, speed, color))
                self.plottime += dt

            self.plotfile.write("# Linear top:\n")
            self.plotfile.write("%f %f 0\n" % (self.plottime, steps_per_second_nominal/steps_per_mm))
            self.plottime += timerValue / fTimer

            self.plotfile.write("# Decceleration:\n")
            deccelPlotData.reverse()
            for (speed, color, dt) in deccelPlotData:
                self.plotfile.write("%f %f %d\n" % (self.plottime, speed, color))
                self.plottime += dt

        if debugMoves:
            print "# of steps for move: ", deltaLead
            move.pprint("move:")
            print 

        move.stepData.checkLen(deltaLead)

        if debugMoves:
            print "***** End planSteps() *****"


