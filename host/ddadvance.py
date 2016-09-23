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
from ddprofile import PrinterProfile, MatProfile # , NozzleProfile

import ddprintutil as util
import math


if debugPlot:
    import pickle, matplotlib.pyplot as plt 
    from matplotlib import collections as mc

emptyVector3 = [0] * 3
emptyVector5 = [0] * 5

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
        # self.kAdv = 0.0 # [s] # xxx move to material profile
        # self.kAdv = 0.035 # [s] # xxx move to material profile
        # self.kAdv = 0.010 # [s]
        # self.kAdv = 0.100 # [s] # xxx move to material profile
        # self.kAdv = 0.250 # [s] # xxx move to material profile

        self.kAdv = MatProfile.getKAdv()

        if self.kAdv:

            # self.advAccel = ((sel.jerk[A_AXIS] / 1000.0) / self.kAdv) * 1000
            advJerk = planner.jerk[A_AXIS]
            # self.advAccel = self.advJerk / self.kAdv
            maxEAccel = advJerk / self.kAdv

            #
            # Limit E-acceleration by DEFAULT_ACCELERATION/DEFAULT_MAX_ACCELERATION
            #
            print "ADV: max E-acceleration:", maxEAccel, ", unlimited accel vector: ", MAX_AXIS_ACCELERATION_NOADV, " [mm/s²]"
            maxEAccel = min(maxEAccel, MAX_AXIS_ACCELERATION_NOADV[A_AXIS])

            self.maxAxisAcceleration = MAX_AXIS_ACCELERATION_NOADV[:3] + [maxEAccel, maxEAccel]

            print "ADV: max E-acceleration, limited acceleration vector:", self.maxAxisAcceleration, " [mm/s²]"

        else:

            self.maxAxisAcceleration = MAX_AXIS_ACCELERATION_NOADV

        # self.plotfile = None

        # Running sum of e-distances through advance, for debugging of planAdvance()
        self.advSum = 0
        # Sum of skipped small accelration ramps
        self.skippedStartAdvSum = 0
        self.skippedLinSteps = 0
        self.skippedAccelSteps = 0
        self.skippedDecelSteps = 0
        self.skippedSimpleSteps = 0
        # self.skippedEndAdvSum = 0
        # Likewise for decel
        # self.endSum = 0
        # Running balance of e-distances through advance, for debugging of planSteps()
        self.advStepBalance = 0
        # Running sum of e-distances through advance, for debugging of planSteps()
        self.advStepSum = 0

        # Running sum of move esteps, for debugging of planSteps()
        self.moveEsteps = 0.0

        self.e_steps_per_mm = PrinterProfile.getStepsPerMM(A_AXIS)

    def eJerk(self, accel):
        return abs(accel) * self.kAdv

    def planPath(self, path):

        if debugMoves:
            print "***** Start planPath() *****"

        # self.startSum = self.endSum = 0
        self.advSum = 0
        self.skippedStartAdvSum = 0
        self.skippedLinSteps = 0
        self.skippedAccelSteps = 0
        self.skippedDecelSteps = 0
        self.skippedSimpleSteps = 0
        # self.skippedEndAdvSum = 0
        self.advStepBalance = 0
        self.advStepSum = 0
        self.moveEsteps = 0.0

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
        path[0].startSpeed.plannedSpeed(path[0].topSpeed.nominalSpeed().scale(0.0))

        # Last move
        path[-1].endSpeed.plannedSpeed(path[-1].topSpeed.nominalSpeed().scale(0.0))

        if UseExtrusionAutoTemp:
            self.pathData.time = 0 # Reset path time
            self.pathData.extrusionAmount = 0
            self.pathData.lastTemp = MatProfile.getHotendBaseTemp()

        prevMove = path[0]

        # Step 1: join moves forward
        for move in path[1:]:
            util.joinMoves(prevMove, move, self.planner.jerk, self.maxAxisAcceleration)
            prevMove = move


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
            # util.xxx(prevMove, move, self.jerk, self.min_speeds)
            # prevMove = move

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

            move.sanityCheck(self.planner.jerk)
            self.planAcceleration(move)

        if self.kAdv:
            for move in path:
                move.sanityCheck(self.planner.jerk)

                # sum up esteps
                self.moveEsteps += move.displacement_vector_steps_raw[A_AXIS]
                print "moveEsteps+: %7.3f %7.3f" % ( move.displacement_vector_steps_raw[A_AXIS], self.moveEsteps)

                self.planAdvance(move)

        if debugPlot and debugPlotLevel == "plotLevelPlanned":

            # xxx todo move to own function
            for move in path:

                self.plotfile.plot1Tick(move.topSpeed.trueSpeed().XY(), move.moveNumber)

                at = move.accelTime()
                lt = move.linearTime()
                dt = move.decelTime()

                if at:

                    self.plotfile.plot1Segments(at, (
                        DebugPlotSegment(move.startSpeed.trueSpeed().XY(), move.topSpeed.trueSpeed().XY(), "green"),
                        DebugPlotSegment(move.startSpeed.trueSpeed()[A_AXIS], move.topSpeed.trueSpeed()[A_AXIS], "green"),
                        ))

                if lt:

                    self.plotfile.plot1Segments(lt, (
                        DebugPlotSegment(move.topSpeed.trueSpeed().XY(), color="blue"),
                        DebugPlotSegment(move.topSpeed.trueSpeed()[A_AXIS], color="blue"),
                        ))

                if dt:

                    self.plotfile.plot1Segments(dt, (
                        DebugPlotSegment(move.topSpeed.trueSpeed().XY(), move.endSpeed.trueSpeed().XY(), "red"),
                        DebugPlotSegment(move.topSpeed.trueSpeed()[A_AXIS], move.endSpeed.trueSpeed()[A_AXIS], "red"),
                        ))

                if at:

                    self.plotfile.plot2Segments(0, (
                        DebugPlotSegment(move.startSpeed.trueSpeed()[A_AXIS], move.advanceData.startEFeedrate(), "green"),
                        ))
                    self.plotfile.plot2Segments(at, (
                        DebugPlotSegment(move.startSpeed.trueSpeed()[A_AXIS], move.topSpeed.trueSpeed()[A_AXIS]),
                        DebugPlotSegment(move.advanceData.startEFeedrate(), move.advanceData.startEReachedFeedrate(), "green"),
                        ))
                    self.plotfile.plot2Segments(0, (
                        DebugPlotSegment(move.advanceData.startEReachedFeedrate(), move.topSpeed.trueSpeed()[A_AXIS], "green"),
                        ))

                if lt:
                    self.plotfile.plot2Segments(lt, (
                        DebugPlotSegment(move.topSpeed.trueSpeed()[A_AXIS]),
                        ))

                if dt:

                    self.plotfile.plot2Segments(0, (
                        DebugPlotSegment(move.topSpeed.trueSpeed()[A_AXIS], move.advanceData.endEReachedFeedrate(), "red"),
                        ))
                    self.plotfile.plot2Segments(dt, (
                        DebugPlotSegment(move.topSpeed.trueSpeed()[A_AXIS], move.endSpeed.trueSpeed()[A_AXIS]),
                        DebugPlotSegment(move.advanceData.endEReachedFeedrate(), move.advanceData.endEFeedrate(), "red"),
                        ))
                    self.plotfile.plot2Segments(0, (
                        DebugPlotSegment(move.advanceData.endEFeedrate(), move.endSpeed.trueSpeed()[A_AXIS], "red"),
                        ))

        # self.plotfile.close()
        # return # xxx work

        newPath = []
        for move in path:

            newMoves = self.planSteps(move)

            newPath += newMoves

        print "Path advSum: ", self.advSum
        print "Path skippedStartAdvSum: ", self.skippedStartAdvSum
        # print "Path skippedEndAdvSum: ", self.skippedEndAdvSum
        print "Path skippedLinSteps: ", self.skippedLinSteps
        print "Path skippedAccelSteps: ", self.skippedAccelSteps
        print "Path skippedDecelSteps: ", self.skippedDecelSteps
        print "Path skippedSimpleSteps: ", self.skippedSimpleSteps
        print "Path advStepBalance, advStepSum: ", self.advStepBalance, self.advStepSum
        print "Path moveEsteps: %7.3f" % ( self.moveEsteps)
        # print "ediff: ", self.ediff

        assert(util.circaf(self.advSum, 0, 0.02))
        assert(util.circaf(self.skippedStartAdvSum, 0, 2.0/self.e_steps_per_mm))
        # assert(util.circaf(self.skippedEndAdvSum, 0, 1.0/self.e_steps_per_mm))
        assert(util.circaf(self.skippedLinSteps, 0, 1.0/self.e_steps_per_mm))
        assert(util.circaf(self.skippedAccelSteps, 0, 1.0/self.e_steps_per_mm))
        assert(util.circaf(self.skippedDecelSteps, 0, 1.0/self.e_steps_per_mm))
        assert(util.circaf(self.skippedSimpleSteps, 0, 1))
        assert(util.circaf(self.advStepBalance, self.advStepSum, 1))
        assert(util.circaf(self.moveEsteps, 0, 3))

        # xxx debug, check chain
        n = 1
        m = newPath[0]
        while m.nextMove:
            m = m.nextMove
            n += 1
        print "checkchain:", n, len(newPath)
        assert(n == len(newPath))
        n = 1
        m = newPath[-1]
        while m.prevMove:
            m = m.prevMove
            n += 1
        print "checkchain:", n, len(newPath)
        assert(n == len(newPath))

        if debugPlot and debugPlotLevel == "plotLevelSplitted":

            for move in newPath:

                # xxx todo move to own function
                self.plotfile.plot1Tick(move.topSpeed.trueSpeed().XY(), move.moveNumber)

                at = move.accelTime()
                lt = move.linearTime()
                dt = move.decelTime()

                print "ald:", at, lt, dt

                if at:

                    self.plotfile.plot1Segments(at, (
                        DebugPlotSegment(move.startSpeed.trueSpeed().XY(), move.topSpeed.trueSpeed().XY(), "green"),
                        DebugPlotSegment(move.startSpeed.trueSpeed()[A_AXIS], move.topSpeed.trueSpeed()[A_AXIS], "green"),
                        ))

                if lt:

                    self.plotfile.plot1Segments(lt, (
                        DebugPlotSegment(move.topSpeed.trueSpeed().XY(), color="blue"),
                        DebugPlotSegment(move.topSpeed.trueSpeed()[A_AXIS], color="blue"),
                        ))

                if dt:

                    self.plotfile.plot1Segments(dt, (
                        DebugPlotSegment(move.topSpeed.trueSpeed().XY(), move.endSpeed.trueSpeed().XY(), "red"),
                        DebugPlotSegment(move.topSpeed.trueSpeed()[A_AXIS], move.endSpeed.trueSpeed()[A_AXIS], "red"),
                        ))

                """
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

                """

        print "xxx no statistic"
        for move in newPath:

            #
            # Collect some statistics
            #
            # self.pathData.maxExtrusionRate.stat(move)

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

                # if debugMoves:
                    # print "Streaming print-move:", move.moveNumber
                self.planner.streamMove(move)

            move.streamed = True

            # Help garbage collection
            move.prevMove = util.StreamedMove()
            move.nextMove = util.StreamedMove()
       


        if debugPlot:
            # self.plotfile.write(1)
            # self.plotfile.write()
            self.plotfile.close()

        """
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
        """

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

            endSpeedS = move.endSpeed.plannedSpeed().feedrate

            allowedAccel = move.getMaxAllowedAccel(self.maxAxisAcceleration)
            maxAllowedStartSpeed = util.vAccelPerDist(endSpeedS, allowedAccel, move.distance)

            # print "joinMovesBwd, startspeed, max startspeed: ", move.getStartFr(), maxAllowedStartSpeed

            startSpeed1 = move.startSpeed.plannedSpeed()
            startSpeedS = startSpeed1.feedrate

            if maxAllowedStartSpeed >= startSpeedS:
                # good, move is ok
                continue

            if debugMoves: 
                print "Startspeed of %.5f is to high to reach the desired endspeed of %.5f." % (startSpeedS, endSpeedS)
                print "Max. allowed startspeed: %.5f." % maxAllowedStartSpeed

            if move.prevMove:

                #
                # Adjust endspeed of the previous move, also.
                #

                factor = maxAllowedStartSpeed / startSpeedS
                # print "factor: ", factor
                # Adjust endspeed of last move:
          
                assert(factor < 1)

                # XXX einfacher algo, kann man das besser machen (z.b. mit jerk-berechnung,
                # vector subtraktion oder so?)
                # move.prevMove.setTrueEndFr(move.prevMove.getEndFr() * factor)
                endSpeedS1 = move.prevMove.endSpeed.plannedSpeed()
                endSpeedS1.feedrate *= factor
                move.prevMove.endSpeed.trueSpeed(endSpeedS1)

            # Adjust startspeed of this move:
            # move.setTrueStartFr(maxAllowedStartSpeed)
            startSpeed1.feedrate = maxAllowedStartSpeed
            print "len: ", len(move.startSpeed.speeds)
            move.startSpeed.trueSpeed(startSpeed1)

        if debugMoves:
            print "***** End joinMovesBwd() *****"

    def planAcceleration(self, move):

        if debugMoves: 
            move.pprint("Start planAcceleration")
            print "***** Start planAcceleration() *****"

        move.state = 2

        allowedAccel = allowedDeccel = move.getMaxAllowedAccel(self.maxAxisAcceleration)
        print "allowed accel: ", allowedAccel

        #
        # Check if the speed difference between startspeed and endspeed can be done with
        # this acceleration in this distance
        #
        startSpeedS = move.startSpeed.plannedSpeed().feedrate
        endSpeedS = move.endSpeed.plannedSpeed().feedrate

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

        deltaStartSpeedS = move.topSpeed.plannedSpeed().feedrate - startSpeedS

        if deltaStartSpeedS:

            ta = deltaStartSpeedS / allowedAccel
            print "accel time (for %f mm/s): %f [s]" % (deltaStartSpeedS, ta)

            # debug Check axxis acceleration
            deltaSpeedV = move.direction.scale(deltaStartSpeedS)
            for dim in range(5):
                dimAccel = abs(deltaSpeedV[dim]) / ta
                print "dimaccel ", dim, dimAccel
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

        deltaEndSpeedS = move.topSpeed.plannedSpeed().feedrate - endSpeedS                          # [mm/s]

        if deltaEndSpeedS:

            tb = deltaEndSpeedS / allowedAccel                          # [s]
            # print "deccel time (for %f mm/s): %f [s]" % (deltaEndSpeedS, tb)

            # debug Check axxis acceleration
            deltaSpeedV = move.direction.scale(deltaEndSpeedS)
            for dim in range(5):
                dimDeccel = abs(deltaSpeedV[dim]) / tb  
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

            topSpeed = move.topSpeed.speed()

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

            topSpeed.feedrate = v

            move.topSpeed.setSpeed(topSpeed)

            deltaSpeedS = v - startSpeedS                          # [mm/s]
            ta = deltaSpeedS / allowedAccel
            # print "ta: ", ta, deltaSpeedS

            deltaSpeedS = v - endSpeedS                          # [mm/s]
            tb = deltaSpeedS / allowedAccel
            # print "tb: ", tb, deltaSpeedS

            move.setDuration(ta, 0, tb)

            if debugMoves:
                move.pprint("End planAcceleration")
                print 
                print "***** End planAcceleration() *****"
                # if move.moveNumber == 40: assert(0)
                
            return

        # 
        # Strecke reicht aus, um auf nominal speed zu beschleunigen
        # 

        # print "ta: ", ta, deltaStartSpeedS, sa
        # print "tb: ", tb, deltaEndSpeedS, sb

        nominalSpeed = move.topSpeed.plannedSpeed().feedrate # [mm/s]
        slin = move.distance - (sa+sb)
        tlin = slin / nominalSpeed
        # print "tlin: ", tlin, slin
        move.setDuration(ta, tlin, tb)

        if debugMoves:
            move.pprint("End planAcceleration")
            print 
            print "***** End planAcceleration() *****"
            # if move.moveNumber == 40: assert(0)

    ################################################################################

    def planAdvance(self, move):

        if debugMoves:
            print "***** Start planAdvance() *****"
            move.pprint("planAdvance:")

        ta = move.accelTime()
        tl = move.linearTime()
        td = move.decelTime()

        # xxxxxxxxxxxxxxxxxxxx# xxx funktion trotzdem durchlaufen wegen rundung von e-steps
        # No advance if there are no (accel- or decel-) ramps.
        if not (ta or td):

            """
            ######################
            # round e
            disp = move.displacement_vector_steps_raw

            e = disp[A_AXIS] + self.skippedSimpleSteps

            assert(e>=0)

            rest = e % 1
            disp[A_AXIS] = int(e)

            self.skippedSimpleSteps += rest
            ######################
            """

            if debugMoves:
                print "***** End planAdvance() *****"
            return

        startSpeed = move.startSpeed.trueSpeed()
        startFeedrateE = startSpeed[A_AXIS]

        topSpeed = move.topSpeed.trueSpeed()
        reachedFeedrateE = topSpeed[A_AXIS]

        endSpeed = move.endSpeed.trueSpeed()
        endFeedrateE = endSpeed[A_AXIS]

        startEVelDiff = reachedFeedrateE - startFeedrateE
        # startEAccelSign = util.sign(startEVelDiff)

        endEVelDiff = endFeedrateE - reachedFeedrateE
        # endEAccelSign = util.sign(endEVelDiff)

        print "startEVelDiff, endEVelDiff: ", startEVelDiff, endEVelDiff
        assert(startEVelDiff >= 0)
        assert(endEVelDiff <= 0)

        allowedAccelV = move.getMaxAllowedAccelVector(self.maxAxisAcceleration)

        if ta:
            #
            # Advance of start-ramp
            #
            startFeedrateIncrease = self.eJerk(allowedAccelV[A_AXIS])
            (nu_sa, esteps, nu_ediff) = move.startAdvSteps(startFeedrateIncrease=startFeedrateIncrease)

            if esteps > AdvanceMinRamp:
                move.advanceData.startFeedrateIncrease = startFeedrateIncrease

                (sa, esteps, ediff) = move.startERampSteps(roundError=self.skippedStartAdvSum)

                print "(sd, esteps, ediff):", (sa, esteps, ediff) 
                # if esteps >= AdvanceMinRamp:
                if True:

                    move.advanceData.startESteps = esteps
                    self.advStepSum += esteps
                    self.skippedStartAdvSum = ediff
                else:
                    assert(0)

                    # Dont advance very small acceleration ramps, but sum up the missing advance.
                    self.skippedStartAdvSum += nu_sa

                    # E-steps of non-advanced accel ramp
                    sa = move.startRampDistance(startSpeed[A_AXIS], topSpeed[A_AXIS], ta) + self.skippedAccelSteps

                    esteps = int(sa * self.e_steps_per_mm)
                    self.skippedAccelSteps = sa - (esteps / float(self.e_steps_per_mm))
                    print "dim E moves %.3f mm in accel phase -> %d steps" % (sa, esteps)

                    move.advanceData.startESteps = esteps
                    self.advStepSum += esteps
            else:

                print "(sa, esteps, ediff):", (nu_sa, esteps, nu_ediff) 

                # Dont advance very small acceleration ramps, but sum up the missing advance.
                self.skippedStartAdvSum += nu_sa

                # E-steps of non-advanced accel ramp
                sa = move.startRampDistance(startSpeed[A_AXIS], topSpeed[A_AXIS], ta) + self.skippedAccelSteps

                esteps = int(sa * self.e_steps_per_mm)
                self.skippedAccelSteps = sa - (esteps / float(self.e_steps_per_mm))
                print "dim E moves %.3f mm in accel phase -> %d steps" % (sa, esteps)

                move.advanceData.startESteps = esteps
                self.advStepSum += esteps

            self.advSum += nu_sa

        if tl:
            # E-steps in linear phase
            # E-distance ist nicht einfach der rest der e-steps, linearer e-anteil muss über
            # die dauer des linearen anteils (tLinear) berechnet werden.
            sl = tl * topSpeed[A_AXIS] + self.skippedLinSteps
            esteps = int(sl * self.e_steps_per_mm)
            self.skippedLinSteps = sl - (esteps / float(self.e_steps_per_mm))
            print "dim E moves %.3f mm in linear phase -> %d steps" % (sl, esteps)

            move.advanceData.linESteps = esteps
            self.advStepSum += esteps

        if td:
            #
            # Advance of end-ramp
            #
            endFeedrateIncrease = - self.eJerk(allowedAccelV[A_AXIS])
            (nu_sd, esteps, nu_ediff) = move.endAdvSteps(endFeedrateIncrease=endFeedrateIncrease)

            if esteps < -AdvanceMinRamp:

                move.advanceData.endFeedrateIncrease = endFeedrateIncrease

                if move.advanceData.endSignChange(): 

                    # assert(0)
                    ###############################################################
                    # Compute additional data for planSteps()

                    # Time till the e-velocity crosses zero
                    allowedAccelV = move.getMaxAllowedAccelVector(self.maxAxisAcceleration)

                    tdc = abs(move.advanceData.endEReachedFeedrate() / allowedAccelV[A_AXIS])
                    print "Time to reach zero-crossing (tdc):", tdc, ", tdd: ", td - tdc

                    # Nominal speed at zero crossing
                    allowedAccel = move.getMaxAllowedAccel(self.maxAxisAcceleration)

                    if topSpeed.feedrate > endSpeed.feedrate:
                        # decel
                        zeroCrossingS = util.vAccelPerTime(topSpeed.feedrate, -allowedAccel, tdc)
                    else:
                        assert(0) # does this happen?

                    crossingSpeed = move.topSpeed.speed()
                    crossingSpeed.feedrate = zeroCrossingS

                    print "top, zerocrossspeed:", topSpeed.feedrate, zeroCrossingS, crossingSpeed

                    move.advanceData.tdc = tdc
                    move.advanceData.tdd = td - tdc
                    move.advanceData.crossingSpeed = crossingSpeed

                    # PART C, E
                    (sdc, estepsc, ediffc) = move.endERampSteps(tdc, v1=crossingSpeed[A_AXIS], roundError=self.skippedStartAdvSum)
                    print "(sdc, estepsc, ediffc):", (sdc, estepsc, ediffc) 

                    move.advanceData.endEStepsC = estepsc
                    self.advStepSum += estepsc

                    (sdd, estepsd, ediffd) = move.endERampSteps(move.advanceData.tdd, v0=crossingSpeed[A_AXIS], roundError=ediffc)
                    print "(sdd, estepsd, ediffd):", (sdd, estepsd, ediffd) 

                    move.advanceData.endEStepsD = estepsd
                    self.advStepSum += estepsd

                    self.skippedStartAdvSum = ediffd

                    ###############################################################
                else:

                    (sd, esteps, ediff) = move.endERampSteps(roundError=self.skippedStartAdvSum)

                    print "(sd, esteps, ediff):", (sd, esteps, ediff) 
                    
                    # if esteps <= -AdvanceMinRamp:
                    if True:

                        move.advanceData.endESteps = esteps
                        self.advStepSum += esteps

                        # self.advSum += move.endAdvDistance(td)
                        self.skippedStartAdvSum = ediff

                    else:
                        # Dont advance, sum up like the decel ramp.
                        self.skippedStartAdvSum = sd

                        # E-steps of non-advanced decel ramp
                        sd = move.endRampDistance(topSpeed[A_AXIS], endSpeed[A_AXIS], td) + self.skippedDecelSteps

                        esteps = int(sd * self.e_steps_per_mm)
                        self.skippedDecelSteps = sd - (esteps / float(self.e_steps_per_mm))
                        print "dim E moves %.3f mm in decel phase -> %d steps" % (sd, esteps)

                        move.advanceData.endESteps = esteps
                        self.advStepSum += esteps

                        assert(0)

            else:

                print "(sd, esteps, ediff):", (nu_sd, esteps, nu_ediff) 

                # Dont advance, sum up like the decel ramp.
                self.skippedStartAdvSum += nu_sd

                # E-steps of non-advanced decel ramp
                sd = move.endRampDistance(topSpeed[A_AXIS], endSpeed[A_AXIS], td) + self.skippedDecelSteps

                esteps = int(sd * self.e_steps_per_mm)
                self.skippedDecelSteps = sd - (esteps / float(self.e_steps_per_mm))
                print "dim E moves %.3f mm in decel phase -> %d steps" % (sd, esteps)

                move.advanceData.endESteps = esteps
                self.advStepSum += esteps

            self.advSum += nu_sd

        # assert(self.skippedAdvSum < AdvanceMinRamp)

        print "advSum: ", self.advSum
        print "skippedLinSteps: ", self.skippedLinSteps
        print "skippedAccelSteps: ", self.skippedAccelSteps
        print "skippedDecelSteps: ", self.skippedDecelSteps
        print "skippedStartAdvSum: ", self.skippedStartAdvSum
        # print "skippedEndAdvSum: ", self.skippedEndAdvSum

        move.advanceData.advStepSum = self.advStepSum

        # Split move into up to 5 sub moves
        nsub = 1
        if move.advanceData.hasStartAdvance():

            nsub +=1
            move.advanceData.startSplits += 1

            if move.advanceData.startSignChange():
                nsub +=1
                move.advanceData.startSplits += 1
                assert(0)

        if move.advanceData.hasEndAdvance():

            nsub +=1
            move.advanceData.endSplits += 1

            if move.advanceData.endSignChange():
                nsub +=1
                move.advanceData.endSplits += 1

        if nsub > 1:
            print "# movesplits:", nsub

        if move.advanceData.hasStartAdvance():
            print "Adv: adjusted E startspeed: %f -> %f" % (startFeedrateE, move.advanceData.startEFeedrate())

        if move.advanceData.hasEndAdvance():
            print "Adv: adjusted E   endspeed: %f -> %f" % (endFeedrateE, move.advanceData.endEFeedrate())

        if debugMoves:
            print 
            print "***** End planAdvance() *****"

    def planSteps(self, move):

        if debugMoves:
            print "***** Start planSteps() *****"
            move.pprint("PlanSTeps:")

        if (move.advanceData.startSplits + move.advanceData.endSplits) == 0:

            ta = move.accelTime()
            td = move.decelTime()

            esteps = move.advanceData.estepSum()

            if ta or td:
                # xxx hack
                move.displacement_vector_steps_raw[A_AXIS] = esteps

            # Single move with simple ramps
            # xxx use plantravelsteps here
            self.planStepsSimple(move)

            # Sum up esteps for debugging
            if esteps:
                # assert(esteps == move.displacement_vector_steps_raw[A_AXIS])
                self.advStepBalance += esteps

                # Check sum of esteps used
                print "estep balance, move.sum: ", self.advStepBalance, move.advanceData.advStepSum
                assert(self.advStepBalance == move.advanceData.advStepSum)

            if debugMoves:
                print "***** End planSteps() *****"

            return [move]

        assert(move.displacement_vector_steps_raw[Z_AXIS] == 0)

        mask = 0
        if move.advanceData.startSplits == 1:
            mask |= 8
        if move.advanceData.startSplits == 2:
            mask |= 12
        if move.advanceData.endSplits == 2:
            mask |= 3
        if move.advanceData.endSplits == 1:
            mask |= 1

        if mask == 0x8:
            # Simple advanceed ramp at start
            # Create addtional 'sub-move' at beginning

            # if move.linearTime():
            newMoves = self.planSA(move) # simple accel
            # else:
                # newMoves = self.planA(move) # simple accel, only
        elif mask == 0x1:
            # Simple advanceed ramp at end
            # Create addtional 'sub-move' at end

            # assert (0)
            # if move.linearTime():
            newMoves = self.planSD(move) # simple decel
            # else:
                # newMoves = self.planD(move) # simple decel, only
        elif mask == 0x3:
            # Advanced ramp at end with sign-change
            newMoves = self.planLDD(move)
        elif mask == 0x9:
            # Simple advanceed ramp at start
            # Simple advanceed ramp at end

            # if move.linearTime():
            newMoves = self.planSALSD(move) # simple accel, linear part, simple decel
            # else:
                # newMoves = self.planSASD(move) # simple accel, simple decel
        elif mask == 0xb:
            # * advanceed ramp at start
            # * advanceed ramp at end with sign-change
            # * Create three addtional 'sub-moves', on at the start
            #   of the move at two at the end
            # if move.linearTime():
            newMoves = self.planSALDD(move) # simple accel, linear part, dual deccel
            # else:
                # newMoves = self.planSADD(move) # simple accel, dual deccel
        else:
            # Create addtional 'sub-move' at end

            print "unhandled mask: 0x%x" % mask
            assert(0)

        # Check sum of esteps used
        print "estep balance, move.sum: ", self.advStepBalance, move.advanceData.advStepSum
        assert(self.advStepBalance == move.advanceData.advStepSum)

        print "new moves: ", newMoves

        if move.prevMove:
            move.prevMove.nextMove = newMoves[0]
            newMoves[0].prevMove = move.prevMove

        if move.nextMove:
            move.nextMove.prevMove = newMoves[-1]
            newMoves[-1].nextMove = move.nextMove

        for move in newMoves:
            move.sanityCheck()
            self.planStepsSimple(move)

        if debugMoves:
            print "***** End planSteps() *****"

        return newMoves
    
    # xxx use planner.planTravelMove here
    def planStepsSimple(self, move):

        if debugMoves:
            print "***** Start planStepsSimple() *****"
            move.pprint("PlanSTepsSimple:")

        move.state = 3

        dirBits = 0
        abs_displacement_vector_steps = []

        # Determine the 'lead axis' - the axis with the most steps
        leadAxis = 0
        leadAxis_steps = 0

        print "Warning, disabled extrusion adjust!"

        """
        for i in range(5):
            dirBits += (move.displacement_vector_steps_raw()[i] >= 0) << i
            adjustedDisplacement = move.displacement_vector_steps_adjusted(NozzleProfile, MatProfile, PrinterProfile)

            s = abs(adjustedDisplacement[i])
            if s > leadAxis_steps:
                leadAxis = i
                leadAxis_steps = s

            abs_displacement_vector_steps.append(s)
        """

        disp = move.displacement_vector_steps_raw

        # rest = disp[A_AXIS] % 1
        # assert(rest == 0)

        # if rest:
            # esteps = move.advanceData.estepSum()
            # print "Warning, e-steps not integer:", disp[A_AXIS], rest, esteps
            # assert(util.circaf(disp[A_AXIS], esteps, 2))
            # disp[A_AXIS] = esteps

        ######################
        # round e
        # disp = move.displacement_vector_steps_raw

        e = disp[A_AXIS] + self.skippedSimpleSteps
        esteps = int(e)

        # assert(e>=0)

        rest = e - esteps
        disp[A_AXIS] = esteps

        self.skippedSimpleSteps = rest
        ######################

        self.moveEsteps -= disp[A_AXIS]
        print "moveEsteps-: %7.3f %7.3f" % ( disp[A_AXIS], self.moveEsteps)

        for i in range(5):
            dirBits += (disp[i] >= 0) << i # xxx use sign here

            s = abs(disp[i])
            abs_displacement_vector_steps.append(s)
            if s > leadAxis_steps:
                leadAxis = i
                leadAxis_steps = s

        if dirBits != self.printer.curDirBits:
            move.stepData.setDirBits = True
            move.stepData.dirBits = dirBits
            self.printer.curDirBits = dirBits

        steps_per_mm = PrinterProfile.getStepsPerMM(leadAxis)

        #
        # Bresenham's variables
        #
        move.stepData.setBresenhamParameters(leadAxis, abs_displacement_vector_steps)

        #
        # Create a list of stepper pulses
        #
        nominalSpeed = abs( move.topSpeed.trueSpeed()[leadAxis] ) # [mm/s]

        # advance
        allowedAccel = move.getMaxAllowedAccel(self.maxAxisAcceleration)

        print "allowedAccel: ", move.getMaxAllowedAccel(self.maxAxisAcceleration), self.maxAxisAcceleration
        accel_steps_per_square_second = allowedAccel * steps_per_mm

        # v0 = abs(move.getFeedrateV(move.getStartFr())[leadAxis])                # [mm/s]
        v0 = abs(move.startSpeed.trueSpeed()[leadAxis])                # [mm/s]
        if v0 < 0.1:
          print "planstepsSimple: min v0 hack", v0
          v0 = 0.1

        # v1 = abs(move.getFeedrateV(move.getEndFr())[leadAxis])                # [mm/s]
        v1 = abs(move.endSpeed.trueSpeed()[leadAxis])                # [mm/s]
        if v1 < 0.1:
          print "planstepsSimple: min v1 hack", v1
          v1 = 0.1

        print "lead, v0, v1: ", leadAxis, v0, v1

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

        if move.accelTime() and move.decelTime():

            #
            # Acceleration ramp on both sides.
            #
            # Ramp up both sides in parallel to not exeed available steps
            #

            done = False
            while not done and stepNr < leadAxis_steps:

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

                    """
                    if debugPlot:
                        if move.eOnly:
                            accelPlotData.append((steps_per_second_accel/steps_per_mm, 2, dt))
                        else:
                            accelPlotData.append((steps_per_second_accel/steps_per_mm, 1, dt))
                    """

                    tAccel += dt

                    if timerValue > maxTimerValue24:
                        move.pprint("PlanSTepsSimple:")
                        print "v0: ", dt*1000000, "[uS]", steps_per_second_accel, "[steps/s], timerValue: ", timerValue
                        print "dt: ", dt*1000000, "[uS]", steps_per_second_accel, "[steps/s], timerValue: ", timerValue
                        assert(0)

                    move.stepData.addAccelPulse(timerValue)
                    stepNr += 1
                    done = False

                if stepNr == leadAxis_steps:
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

                    """
                    if debugPlot:
                        if move.eOnly:
                            deccelPlotData.append((steps_per_second_deccel/steps_per_mm, 2, dt))
                        else:
                            deccelPlotData.append((steps_per_second_deccel/steps_per_mm, 1, dt))
                    """

                    tDeccel += dt

                    if timerValue > maxTimerValue24:
                        move.pprint("PlanSTepsSimple:")
                        print "v0: ", dt*1000000, "[uS]", steps_per_second_deccel, "[steps/s], timerValue: ", timerValue
                        print "dt: ", dt*1000000, "[uS]", steps_per_second_deccel, "[steps/s], timerValue: ", timerValue
                        assert(0)

                    move.stepData.addDeccelPulse(timerValue)
                    stepNr += 1
                    done = False

        elif move.accelTime():

            #
            # Acceleration only
            #

            # 
            # Compute acceleration timer values
            # 
            while steps_per_second_accel < steps_per_second_nominal and stepNr < leadAxis_steps:

                #
                # Compute timer value
                #
                steps_per_second_accel = min(steps_per_second_0 + tAccel * accel_steps_per_square_second, steps_per_second_nominal)

                dt = 1.0 / steps_per_second_accel
                timerValue = int(fTimer / steps_per_second_accel)

                # print "dt: ", dt*1000000, "[uS]", steps_per_second_accel, "[steps/s], timerValue: ", timerValue

                """
                if debugPlot:
                    if move.eOnly:
                        accelPlotData.append((steps_per_second_accel/steps_per_mm, 2, dt))
                    else:
                        accelPlotData.append((steps_per_second_accel/steps_per_mm, 1, dt))
                """

                tAccel += dt

                if timerValue > maxTimerValue24:
                    move.pprint("PlanSTepsSimple:")
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
            while steps_per_second_deccel < steps_per_second_nominal and stepNr < leadAxis_steps:

                #
                # Compute timer value
                #
                steps_per_second_deccel = min(steps_per_second_1 + tDeccel * accel_steps_per_square_second, steps_per_second_nominal)

                dt = 1.0 / steps_per_second_deccel
                timerValue = int(fTimer / steps_per_second_deccel)

                # print "dt: ", dt*1000000, "[uS]", steps_per_second_deccel, "[steps/s], timerValue: ", timerValue, ", v: ", steps_per_second_deccel/steps_per_mm

                """
                if debugPlot:
                    if move.eOnly:
                        deccelPlotData.append((steps_per_second_deccel/steps_per_mm, 2, dt))
                    else:
                        deccelPlotData.append((steps_per_second_deccel/steps_per_mm, 1, dt))
                """

                tDeccel += dt

                if timerValue > maxTimerValue24:
                    move.pprint("PlanSTepsSimple:")
                    print "v0: ", dt*1000000, "[uS]", steps_per_second_deccel, "[steps/s], timerValue: ", timerValue
                    print "dt: ", dt*1000000, "[uS]", steps_per_second_deccel, "[steps/s], timerValue: ", timerValue
                    assert(0)

                move.stepData.addDeccelPulse(timerValue)
                stepNr += 1

        #
        # Linear phase
        #
        if move.linearTime():
        # if True:
            timerValue = fTimer / steps_per_second_nominal
            move.stepData.setLinTimer(timerValue)
        else:
            move.stepData.setLinTimer(0xffff)

        """
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
        """

        if debugMoves:
            print "# of steps for move: ", leadAxis_steps
            move.pprint("move:")
            print 

        move.stepData.checkLen(leadAxis_steps)

        if debugMoves:
            print "***** End planStepsSimple() *****"


    #
    # Single advanceed ramp at start, no linear part
    #
    def planA(self, parentMove):

        if debugMoves:
            print "***** Start planA() *****"
            parentMove.pprint("planA:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw 
        displacement_vector_steps_A = parentMove.displacement_vector_steps_raw[:]

        ta = parentMove.accelTime()

        startSpeed = parentMove.startSpeed.speed()
        topSpeed =   parentMove.topSpeed.speed()

        ####################################################################################
        # PART A, E
        (sa, esteps, ediff) = parentMove.startERampSteps()
        self.ediff += ediff
        print "ediff: ", self.ediff
        print "dim E moves %.3f mm while accelerating -> %d steps" % (sa, esteps)
        assert(0)

        displacement_vector_steps_A[A_AXIS] = esteps
        ####################################################################################

        print "new steps: ", displacement_vector_steps_A

        from move import SubMove

        moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)

        moveA.setDuration(ta, 0, 0)
     
        sv = parentMove.startSpeed.trueSpeed().vv()
        sv[A_AXIS] = parentMove.advanceData.startEFeedrate()

        tv = topSpeed.vv()
        tv[A_AXIS] = parentMove.advanceData.startEReachedFeedrate()

        moveA.setSpeeds(sv, tv, tv)

        # Sum up additional e-distance of this move for debugging
        self.advStepBalance += displacement_vector_steps_A[A_AXIS] - displacement_vector_steps_raw[A_AXIS]
        assert(0)
        print "advStepBalance: ", self.advStepBalance

        if debugMoves:
            print "***** End planA() *****"

        assert(0)
        return [moveA]


    #
    # Single advanceed ramp at start
    # Create addtional 'sub-move' at beginning
    # Das wichtigste ist, die anzahl der steps genau zu treffen, geringe
    # abweichungen was die beschleunigung oder die geschwindigkeiten betrifft,
    # sind ok.
    #
    # Aufteilung nach taccel/tlinear
    #
    def planSA(self, parentMove):

        if debugMoves:
            print "***** Start planSA() *****"
            parentMove.pprint("planSA:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw

        displacement_vector_steps_A = [0] * 5
        displacement_vector_steps_B = [0] * 5

        ta = parentMove.accelTime()
        tl = parentMove.linearTime()
        td = parentMove.decelTime()

        startSpeed = parentMove.startSpeed.speed()
        topSpeed =   parentMove.topSpeed.speed()
        endSpeed =   parentMove.endSpeed.speed()

        ####################################################################################
        # PART A, XY
        for dim in [X_AXIS, Y_AXIS]:

            sa = parentMove.startRampDistance(
                    startSpeed[dim],
                    topSpeed[dim], ta)

            steps = int(round(sa * PrinterProfile.getStepsPerMM(dim)))
            print "dim %d moves %.3f mm while accelerating -> %d steps" % (dim, sa, steps)

            displacement_vector_steps_A[dim] = steps

        # PART A, E
        # (sa, esteps, ediff) = parentMove.startERampSteps()
        # self.ediff += ediff
        # print "ediff: ", self.ediff
        # print "dim E moves %.3f mm while accelerating -> %d steps" % (sa, esteps)

        print "dim E moves %d steps while accelerating" % parentMove.advanceData.startESteps
        displacement_vector_steps_A[A_AXIS] = parentMove.advanceData.startESteps
        # assert(util.circaf(esteps, parentMove.advanceData.startESteps, 1.001))
        ####################################################################################

        ####################################################################################
        # PART B, XY

        if tl or td:

            for dim in range(3):
                displacement_vector_steps_B[dim] = displacement_vector_steps_raw[dim] - displacement_vector_steps_A[dim]
       
        # PART B, E
        # E-distance ist nicht einfach der rest der e-steps, linearer e-anteil muss über
        # die dauer des linearen anteils (tLinear) berechnet werden.
        # s = tl * topSpeed[A_AXIS]

        if tl:

            print "dim E moves %d steps in linear phase" % parentMove.advanceData.linESteps
            displacement_vector_steps_B[A_AXIS] += parentMove.advanceData.linESteps

        if td:
            # assert(0)
            # s += parentMove.endRampDistance(
                # topSpeed[A_AXIS],
                # endSpeed[A_AXIS],
                # td)

            print "dim E moves %d steps while decelerating" % parentMove.advanceData.endESteps
            displacement_vector_steps_B[A_AXIS] += parentMove.advanceData.endESteps
        ####################################################################################

        print "new A steps: ", displacement_vector_steps_A
        print "new B steps: ", displacement_vector_steps_B

        from move import SubMove

        moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)

        moveA.setDuration(ta, 0, 0)

        sv = startSpeed.vv()
        sv[A_AXIS] = parentMove.advanceData.startEFeedrate()

        tv = topSpeed.vv()
        tv[A_AXIS] = parentMove.advanceData.startEReachedFeedrate()

        moveA.setSpeeds(sv, tv, tv)

        newMoves = [moveA]

        if tl or td:

            if displacement_vector_steps_B != emptyVector5:

                moveB = SubMove(parentMove, parentMove.moveNumber + 2, displacement_vector_steps_B)
                moveB.setDuration(0, tl, td)
     
                moveB.startSpeed.setSpeed(topSpeed)
                moveB.topSpeed.setSpeed(topSpeed)
                moveB.endSpeed.setSpeed(endSpeed)

                moveA.nextMove = moveB
                moveB.prevMove = moveA

                newMoves.append(moveB)

            else:

                print "planSA: skipping empty b-move" 

        # Sum up additional e-distance of this move for debugging
        esteps = displacement_vector_steps_A[A_AXIS]+displacement_vector_steps_B[A_AXIS]
        # self.advStepBalance += esteps - displacement_vector_steps_raw[A_AXIS]
        self.advStepBalance += esteps
        print "advStepBalance: ", self.advStepBalance

        if debugMoves:
            print "***** End planSA() *****"

        assert(util.vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_B[:3]) == displacement_vector_steps_raw[:3])

        return newMoves


    #
    # Single simple advanceed ramp at end without linear part
    #
    def planD(self, parentMove):

        if debugMoves:
            print "***** Start planD() *****"
            parentMove.pprint("planD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw

        displacement_vector_steps_A = parentMove.displacement_vector_steps_raw[:]

        td = parentMove.decelTime()

        ####################################################################################
        # PART A, E
        assert(0)
        (sd, esteps, ediff) = parentMove.endERampSteps()
        self.ediff += ediff
        print "ediff: ", self.ediff
        print "dim E moves %.3f mm while accelerating -> %d steps" % (sd, esteps)

        displacement_vector_steps_A[A_AXIS] = esteps
        ####################################################################################

        print "new A steps: ", displacement_vector_steps_A

        from move import SubMove

        moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)
        moveA.setDuration(0, 0, parentMove.decelTime())

        sv = parentMove.topSpeed.speed().vv()
        sv[A_AXIS] = parentMove.advanceData.endEReachedFeedrate()

        ev = parentMove.endSpeed.speed().vv()
        ev[A_AXIS] = parentMove.advanceData.endEFeedrate()

        moveA.setSpeeds(sv, sv, ev)

        # Sum up additional e-distance of this move for debugging
        self.advStepBalance += displacement_vector_steps_A[A_AXIS] - displacement_vector_steps_raw[A_AXIS]
        assert(0)
        print "advStepBalance: ", self.advStepBalance

        if debugMoves:
            print "***** End planD() *****"

        assert(0)
        return [moveA]


    #
    # Single advanceed ramp at end
    # Create addtional 'sub-move' at end
    #
    def planSD(self, parentMove):

        if debugMoves:
            print "***** Start planSD() *****"
            parentMove.pprint("planSD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw

        displacement_vector_steps_A = [0] * 5
        displacement_vector_steps_B = [0] * 5

        ta = parentMove.accelTime()
        tl = parentMove.linearTime()
        td = parentMove.decelTime()

        startSpeed = parentMove.startSpeed.speed()
        topSpeed =   parentMove.topSpeed.speed()
        endSpeed =   parentMove.endSpeed.speed()

        ####################################################################################
        # PART B, XY
        for dim in [X_AXIS, Y_AXIS]:

            sd = parentMove.endRampDistance(
                    topSpeed[dim],
                    endSpeed[dim],
                    td)

            steps = int(round(sd * PrinterProfile.getStepsPerMM(dim)))
            print "dim %d moves %.3f mm while decelerating -> %d steps" % (dim, sd, steps)

            displacement_vector_steps_B[dim] = steps
       
        # # PART B, E 
        # (sd, esteps, ediff) = parentMove.endERampSteps()
        # self.ediff += ediff
        # print "ediff: ", self.ediff
        # print "dim E moves %.3f mm while accelerating -> %d steps" % (sd, esteps)

        print "dim E moves %d steps while decelerating" % parentMove.advanceData.endESteps
        displacement_vector_steps_B[A_AXIS] = parentMove.advanceData.endESteps
        # assert(util.circaf(esteps, parentMove.advanceData.endESteps, 3.001))
        ####################################################################################

        ####################################################################################
        if ta or tl:

            # PART A, XY
            for dim in range(3):
                displacement_vector_steps_A[dim] = displacement_vector_steps_raw[dim] - displacement_vector_steps_B[dim]
     
        # PART A, E
        if ta:

            print "dim E moves %d steps while accelerating" % parentMove.advanceData.startESteps
            displacement_vector_steps_A[A_AXIS] += parentMove.advanceData.startESteps

        if tl:

            print "dim E moves %d steps in linear phase" % parentMove.advanceData.linESteps
            displacement_vector_steps_A[A_AXIS] += parentMove.advanceData.linESteps
        ####################################################################################

        print "new A steps: ", displacement_vector_steps_A
        print "new B steps: ", displacement_vector_steps_B

        from move import SubMove

        moveB = SubMove(parentMove, parentMove.moveNumber + 2, displacement_vector_steps_B)
        
        newMoves = [moveB]

        moveB.setDuration(0, 0, td)

        sv = topSpeed.vv()
        sv[A_AXIS] = parentMove.advanceData.endEReachedFeedrate()

        ev = endSpeed.vv()
        ev[A_AXIS] = parentMove.advanceData.endEFeedrate()

        moveB.setSpeeds(sv, sv, ev)

        if (ta or tl) and displacement_vector_steps_A != emptyVector5:

            moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)
            moveA.setDuration(ta, tl, 0)

            moveA.startSpeed.nominalSpeed(startSpeed)
            moveA.topSpeed.nominalSpeed(topSpeed)
            moveA.endSpeed.nominalSpeed(topSpeed)

            moveA.nextMove = moveB
            moveB.prevMove = moveA

            newMoves.insert(0, moveA)

        # Sum up additional e-distance of this move for debugging
        esteps = displacement_vector_steps_A[A_AXIS]+displacement_vector_steps_B[A_AXIS]
        # self.advStepBalance += esteps - displacement_vector_steps_raw[A_AXIS]
        self.advStepBalance += esteps
        print "advStepBalance: ", self.advStepBalance

        if debugMoves:
            print "***** End planSD() *****"

        assert(util.vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_B[:3]) == displacement_vector_steps_raw[:3])

        return newMoves


    #
    # Simple advanceed ramp at start
    # Simple advanceed ramp at end
    # Submoves: A C
    #
    def planSASD(self, parentMove):
        assert(0)

        if debugMoves:
            print "***** Start planSASD() *****"
            parentMove.pprint("planSASD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw

        # PART A, XY
        # Neuen displacement_vector_steps aufbauen:
        displacement_vector_steps_A = [0] * 5
        displacement_vector_steps_B = [0] * 5

        ta = parentMove.accelTime()
        td = parentMove.decelTime()

        allowedAccelV = parentMove.absGetMaxAllowedAccelVector(self.maxAxisAcceleration)

        startSpeed = parentMove.startSpeed.speed()
        topSpeed = parentMove.topSpeed.speed()
        endSpeed = parentMove.endSpeed.speed()

        for dim in [X_AXIS, Y_AXIS]:

            sa = parentMove.startRampDistance(
                    startSpeed[dim],
                    topSpeed[dim], ta)

            steps_per_mm = PrinterProfile.getStepsPerMM(dim)
            steps = int(round(sa * steps_per_mm))
            print "dim %d moves %.3f mm while accelerating -> %d steps" % (dim, sa, steps)
            displacement_vector_steps_A[dim] = steps

        # PART A, E
        (sa, esteps, ediff) = parentMove.startERampSteps()
        assert(0) # ediff
        print "dim E moves %.3f mm while accelerating -> %d steps" % (sa, steps)
        displacement_vector_steps_A[A_AXIS] = steps

        # PART B, XY
        for dim in [X_AXIS, Y_AXIS]:

            sd = parentMove.endRampDistance(
                    topSpeed[dim],
                    endSpeed[dim],
                    td)

            steps_per_mm = PrinterProfile.getStepsPerMM(dim)
            steps = int(round(sd * steps_per_mm))
            print "dim %d moves %.3f mm while decelerating -> %d steps" % (dim, sd, steps)
            displacement_vector_steps_B[dim] = steps

        # PART C, E
        (sd, esteps, ediff) = parentMove.endERampSteps()
        assert(0) # ediff
        print "dim E moves %.3f mm while decelerating -> %d steps" % (sd, steps)
        displacement_vector_steps_B[A_AXIS] = steps

        print "new A steps: ", displacement_vector_steps_A
        print "new B steps: ", displacement_vector_steps_B

        from move import SubMove

        moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)
        moveB = SubMove(parentMove, parentMove.moveNumber + 2, displacement_vector_steps_B)
       
        moveA.setDuration(ta, 0, 0)
        moveB.setDuration(0, 0, td)

        # startSpeed = parentMove.startSpeed.speed()
        # topSpeed = parentMove.topSpeed.speed()
        # endSpeed = parentMove.endSpeed.speed()

        sv = startSpeed.vv()
        sv[A_AXIS] = parentMove.advanceData.startEFeedrate()
        tv = topSpeed.vv()
        tv[A_AXIS] = parentMove.advanceData.startEReachedFeedrate()
        moveA.setSpeeds(sv, tv, tv)

        sv = topSpeed.vv()
        sv[A_AXIS] = parentMove.advanceData.endEReachedFeedrate()
        ev = endSpeed.vv()
        ev[A_AXIS] = parentMove.advanceData.endEFeedrate()
        moveB.setSpeeds(sv, sv, ev)

        moveA.nextMove = moveB
        moveB.prevMove = moveA

        if debugMoves:
            print "***** End planSASD() *****"

        assert(util.vectorAdd(displacement_vector_steps_A, displacement_vector_steps_B) == displacement_vector_steps_raw)

        assert(0)
        return [moveA, moveB]


    #
    # Simple advanceed ramp at start and end
    # optional linear middle part
    # Generates 2 or 3 moves
    #
    def planSALSD(self, parentMove):

        if debugMoves:
            print "***** Start planSALSD() *****"
            parentMove.pprint("planSALSD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw

        displacement_vector_steps_A = [0] * 5
        displacement_vector_steps_B = [0] * 5
        displacement_vector_steps_C = [0] * 5

        ta = parentMove.accelTime()
        tl = parentMove.linearTime()
        td = parentMove.decelTime()

        startSpeed = parentMove.startSpeed.speed()
        topSpeed =   parentMove.topSpeed.speed()
        endSpeed =   parentMove.endSpeed.speed()

        ####################################################################################
        # PART A, XY
        for dim in [X_AXIS, Y_AXIS]:

            sa = parentMove.startRampDistance(
                    startSpeed[dim],
                    topSpeed[dim], ta)

            steps = int(round(sa * PrinterProfile.getStepsPerMM(dim)))
            print "dim %d moves %.3f mm while accelerating -> %d steps" % (dim, sa, steps)

            displacement_vector_steps_A[dim] = steps

        # PART A, E
        print "dim E moves %d steps while accelerating" % parentMove.advanceData.startESteps
        displacement_vector_steps_A[A_AXIS] = parentMove.advanceData.startESteps
        ####################################################################################

        ####################################################################################
        # PART C, XY
        for dim in [X_AXIS, Y_AXIS]:

            sd = parentMove.endRampDistance(
                    topSpeed[dim],
                    endSpeed[dim],
                    td)

            steps = int(round(sd * PrinterProfile.getStepsPerMM(dim)))
            print "dim %d moves %.3f mm while decelerating -> %d steps" % (dim, sd, steps)

            displacement_vector_steps_C[dim] = steps
       
        # PART C, E
        print "dim E moves %d steps while decelerating" % parentMove.advanceData.endESteps
        displacement_vector_steps_C[A_AXIS] = parentMove.advanceData.endESteps
        ####################################################################################

        # Distribute missing X/Y steps from rounding errors (only if no linear part that uses them)
        stepsUsed = util.vectorAdd(util.vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_B[:3]), displacement_vector_steps_C[:3])
        stepsMissing = util.vectorSub(displacement_vector_steps_raw[:3], stepsUsed)

        if not tl and stepsMissing != emptyVector3:

            print "stepsMissing:", stepsMissing

            maxSteps = displacement_vector_steps_A
            for dim in [X_AXIS, Y_AXIS]:
                for dvs in [displacement_vector_steps_B, displacement_vector_steps_C]:
                    if abs(dvs[dim]) > abs(maxSteps[dim]):
                        maxSteps = dvs

                maxSteps[dim] += stepsMissing[dim]
                print "adjusted steps: ", dim, maxSteps

        ####################################################################################
        if tl:

            # Optional PART B, XY
            for dim in range(3):
                displacement_vector_steps_B[dim] = displacement_vector_steps_raw[dim] - (displacement_vector_steps_A[dim] + displacement_vector_steps_C[dim])
      
            # PART B, E
            print "dim E moves %d steps in linear phase" % parentMove.advanceData.linESteps
            displacement_vector_steps_B[A_AXIS] = parentMove.advanceData.linESteps
        ####################################################################################

        print "new A steps: ", displacement_vector_steps_A
        print "new B steps: ", displacement_vector_steps_B
        print "new C steps: ", displacement_vector_steps_C

        from move import SubMove

        moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)
        moveC = SubMove(parentMove, parentMove.moveNumber + 3, displacement_vector_steps_C)

        newMoves = [moveA, moveC]
       
        moveA.setDuration(ta, 0, 0)
        moveC.setDuration(0, 0, td)

        sv = startSpeed.vv()
        sv[A_AXIS] = parentMove.advanceData.startEFeedrate()
        tv = topSpeed.vv()
        tv[A_AXIS] = parentMove.advanceData.startEReachedFeedrate()
        moveA.setSpeeds(sv, tv, tv)

        sv = topSpeed.vv()
        sv[A_AXIS] = parentMove.advanceData.endEReachedFeedrate()
        ev = endSpeed.vv()
        ev[A_AXIS] = parentMove.advanceData.endEFeedrate()
        moveC.setSpeeds(sv, sv, ev)

        if tl:
            moveB = SubMove(parentMove, parentMove.moveNumber + 2, displacement_vector_steps_B)
            moveB.setDuration(0, tl, 0)

            moveB.startSpeed.setSpeed(topSpeed)
            moveB.topSpeed.setSpeed(topSpeed)
            moveB.endSpeed.setSpeed(topSpeed)

            moveA.nextMove = moveB
            moveB.prevMove = moveA
            moveB.nextMove = moveC
            moveC.prevMove = moveB

            newMoves.insert(1, moveB)

        else :

            moveA.nextMove = moveC
            moveC.prevMove = moveA

        # Sum up additional e-distance of this move for debugging
        esteps = displacement_vector_steps_A[A_AXIS]+displacement_vector_steps_B[A_AXIS]+displacement_vector_steps_C[A_AXIS]
        # self.advStepBalance += esteps - displacement_vector_steps_raw[A_AXIS]
        self.advStepBalance += esteps
        print "advStepBalance: ", self.advStepBalance

        if debugMoves:
            print "***** End planSALSD() *****"

        assert(util.vectorAdd(util.vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_B[:3]), displacement_vector_steps_C[:3]) == displacement_vector_steps_raw[:3])

        return newMoves


    #
    # Advanced ramp with sign-change at the end
    # Optional accel/linear part
    # Generates 2 or 3 moves
    def planLDD(self, parentMove):

        if debugMoves:
            print "***** Start planLDD() *****"
            parentMove.pprint("planLDD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw

        displacement_vector_steps_A = [0] * 5
        displacement_vector_steps_B = [0] * 5
        displacement_vector_steps_C = [0] * 5

        ta = parentMove.accelTime()
        tl = parentMove.linearTime()
        td = parentMove.decelTime()

        startSpeed = parentMove.startSpeed.speed()
        topSpeed =   parentMove.topSpeed.speed()
        endSpeed =   parentMove.endSpeed.speed()

        ####################################################################################
        # PART B, XY
        for dim in [X_AXIS, Y_AXIS]:

            sd = parentMove.endRampDistance(
                    topSpeed[dim],
                    parentMove.advanceData.crossingSpeed[dim],
                    parentMove.advanceData.tdc)

            steps = int(round(sd * PrinterProfile.getStepsPerMM(dim)))
            print "dim %d moves %.3f mm while decelerating -> %d steps" % (dim, sd, steps)

            displacement_vector_steps_B[dim] = steps
       
        # PART B, E
        displacement_vector_steps_B[A_AXIS] = parentMove.advanceData.endEStepsC
        print "tdc esteps: ", parentMove.advanceData.endEStepsC
        ####################################################################################

        ####################################################################################
        # PART C, XY
        for dim in [X_AXIS, Y_AXIS]:

            sd = parentMove.endRampDistance(
                    parentMove.advanceData.crossingSpeed[dim],
                    endSpeed[dim],
                    parentMove.advanceData.tdd)

            steps = int(round(sd * PrinterProfile.getStepsPerMM(dim)))
            print "dim %d moves %.3f mm while decelerating -> %d steps" % (dim, sd, steps)

            displacement_vector_steps_C[dim] = steps
       
        # PART C, E
        displacement_vector_steps_C[A_AXIS] = parentMove.advanceData.endEStepsD
        print "tdd esteps: ", parentMove.advanceData.endEStepsD
        ####################################################################################

        # PART A
        if ta or tl:

            for dim in range(3):
                displacement_vector_steps_A[dim] = displacement_vector_steps_raw[dim] - (displacement_vector_steps_B[dim] + displacement_vector_steps_C[dim])
       
        if ta:

            print "dim E moves %d steps while accelerating" % parentMove.advanceData.startESteps
            displacement_vector_steps_A[A_AXIS] += parentMove.advanceData.startESteps

        if tl:

            print "dim E moves %d steps in linear phase" % parentMove.advanceData.linESteps
            displacement_vector_steps_A[A_AXIS] += parentMove.advanceData.linESteps

        ####################################################################################

        print "new A steps: ", displacement_vector_steps_A
        print "new B steps: ", displacement_vector_steps_B
        print "new C steps: ", displacement_vector_steps_C

        from move import SubMove

        moveB = SubMove(parentMove, parentMove.moveNumber + 2, displacement_vector_steps_B)
        moveC = SubMove(parentMove, parentMove.moveNumber + 3, displacement_vector_steps_C)
       
        moveB.setDuration(0, 0, parentMove.advanceData.tdc)
        moveC.setDuration(0, 0, parentMove.advanceData.tdd)

        newMoves = [moveB, moveC]

        sv = topSpeed.vv()
        sv[A_AXIS] = parentMove.advanceData.endEReachedFeedrate()
        ev = [
            parentMove.advanceData.crossingSpeed[X_AXIS],
            parentMove.advanceData.crossingSpeed[Y_AXIS],
            parentMove.advanceData.crossingSpeed[Z_AXIS],
            0.0,
            0.0]
        moveB.setSpeeds(sv, sv, ev)

        sv = [
            parentMove.advanceData.crossingSpeed[X_AXIS],
            parentMove.advanceData.crossingSpeed[Y_AXIS],
            parentMove.advanceData.crossingSpeed[Z_AXIS],
            0.0,
            0.0]
        ev = parentMove.endSpeed.speed().vv()
        ev[A_AXIS] = parentMove.advanceData.endEFeedrate()
        moveC.setSpeeds(sv, sv, ev)


        if ta or tl:

            moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)

            moveA.setDuration(ta, tl, 0)

            moveA.startSpeed.setSpeed(startSpeed)
            moveA.topSpeed.setSpeed(topSpeed)
            moveA.endSpeed.setSpeed(topSpeed)

            moveA.nextMove = moveB
            moveB.prevMove = moveA
            moveB.nextMove = moveC
            moveC.prevMove = moveB

            newMoves.insert(0, moveA)

        else:

            moveB.nextMove = moveC
            moveC.prevMove = moveB

        # Sum up additional e-distance of this move for debugging
        esteps = displacement_vector_steps_A[A_AXIS]+displacement_vector_steps_B[A_AXIS]+displacement_vector_steps_C[A_AXIS]
        # self.advStepBalance += esteps - displacement_vector_steps_raw[A_AXIS]
        self.advStepBalance += esteps
        print "advStepBalance: ", self.advStepBalance

        if debugMoves:
            print "***** End planLDD() *****"

        assert(util.vectorAdd(util.vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_B[:3]), displacement_vector_steps_C[:3]) == displacement_vector_steps_raw[:3])

        return newMoves

    #
    # Simple advanceed ramp at start and advanced ramp with sign-change at the end
    # Optional linear part
    # Generates 3 or 4 moves
    def planSALDD(self, parentMove):

        if debugMoves:
            print "***** Start planSALDD() *****"
            parentMove.pprint("planSALDD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw

        displacement_vector_steps_A = [0] * 5
        displacement_vector_steps_B = [0] * 5
        displacement_vector_steps_C = [0] * 5
        displacement_vector_steps_D = [0] * 5

        ta = parentMove.accelTime()
        tl = parentMove.linearTime()
        td = parentMove.decelTime()

        startSpeed = parentMove.startSpeed.speed()
        topSpeed =   parentMove.topSpeed.speed()
        endSpeed =   parentMove.endSpeed.speed()

        ####################################################################################
        # PART A, XY
        for dim in [X_AXIS, Y_AXIS]:

            sa = parentMove.startRampDistance(
                    startSpeed[dim],
                    topSpeed[dim], ta)

            steps = int(round(sa * PrinterProfile.getStepsPerMM(dim)))
            print "dim %d moves %.3f mm while accelerating -> %d steps" % (dim, sa, steps)

            displacement_vector_steps_A[dim] = steps

        print "dim E moves %d steps while accelerating" % parentMove.advanceData.startESteps
        displacement_vector_steps_A[A_AXIS] = parentMove.advanceData.startESteps
        ####################################################################################

        ####################################################################################
        # PART C, XY
        for dim in [X_AXIS, Y_AXIS]:

            sd = parentMove.endRampDistance(
                    topSpeed[dim],
                    parentMove.advanceData.crossingSpeed[dim],
                    parentMove.advanceData.tdc)

            steps = int(round(sd * PrinterProfile.getStepsPerMM(dim)))
            print "dim %d moves %.3f mm while decelerating -> %d steps" % (dim, sd, steps)

            displacement_vector_steps_C[dim] = steps
      
        displacement_vector_steps_C[A_AXIS] = parentMove.advanceData.endEStepsC
        print "tdc esteps: ", parentMove.advanceData.endEStepsC

        # PART D, XY
        for dim in [X_AXIS, Y_AXIS]:

            sd = parentMove.endRampDistance(
                    parentMove.advanceData.crossingSpeed[dim],
                    endSpeed[dim],
                    parentMove.advanceData.tdd)

            steps = int(round(sd * PrinterProfile.getStepsPerMM(dim)))
            print "dim %d moves %.3f mm while decelerating -> %d steps" % (dim, sd, steps)

            displacement_vector_steps_D[dim] = steps
       
        # PART D, E
        displacement_vector_steps_D[A_AXIS] = parentMove.advanceData.endEStepsD
        print "tdd esteps: ", parentMove.advanceData.endEStepsD
        ####################################################################################

        # Distribute missing X/Y steps from rounding errors (only if no linear part that uses them)
        stepsUsed = util.vectorAdd(util.vectorAdd(util.vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_B[:3]), displacement_vector_steps_C[:3]), displacement_vector_steps_D[:3])
        stepsMissing = util.vectorSub(displacement_vector_steps_raw[:3], stepsUsed)

        if not tl and stepsMissing != emptyVector3:

            print "stepsMissing:", stepsMissing

            maxSteps = displacement_vector_steps_A
            for dim in [X_AXIS, Y_AXIS]:
                for dvs in [displacement_vector_steps_B, displacement_vector_steps_C, displacement_vector_steps_D]:
                    if abs(dvs[dim]) > abs(maxSteps[dim]):
                        maxSteps = dvs

                maxSteps[dim] += stepsMissing[dim]
                print "adjusted steps: ", dim, maxSteps

        ####################################################################################
        # PART B
        if tl:

            for dim in range(3):
                displacement_vector_steps_B[dim] = displacement_vector_steps_raw[dim] - (displacement_vector_steps_A[dim] + displacement_vector_steps_C[dim] + displacement_vector_steps_D[dim])
       
            print "dim E moves %d steps in linear phase" % parentMove.advanceData.linESteps
            displacement_vector_steps_B[A_AXIS] += parentMove.advanceData.linESteps

        ####################################################################################

        print "new A steps: ", displacement_vector_steps_A
        print "new B steps: ", displacement_vector_steps_B
        print "new C steps: ", displacement_vector_steps_C
        print "new D steps: ", displacement_vector_steps_D

        from move import SubMove

        moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)
        moveC = SubMove(parentMove, parentMove.moveNumber + 3, displacement_vector_steps_C)
        moveD = SubMove(parentMove, parentMove.moveNumber + 4, displacement_vector_steps_D)
       
        moveA.setDuration(ta, 0, 0)
        moveC.setDuration(0, 0, parentMove.advanceData.tdc)
        moveD.setDuration(0, 0, parentMove.advanceData.tdd)

        newMoves = [moveA, moveC, moveD]

        sv = startSpeed.vv()
        sv[A_AXIS] = parentMove.advanceData.startEFeedrate()
        tv = topSpeed.vv()
        tv[A_AXIS] = parentMove.advanceData.startEReachedFeedrate()
        moveA.setSpeeds(sv, tv, tv)



        sv = topSpeed.vv()
        sv[A_AXIS] = parentMove.advanceData.endEReachedFeedrate()
        ev = [
            parentMove.advanceData.crossingSpeed[X_AXIS],
            parentMove.advanceData.crossingSpeed[Y_AXIS],
            parentMove.advanceData.crossingSpeed[Z_AXIS],
            0.0,
            0.0]
        moveC.setSpeeds(sv, sv, ev)

        sv = [
            parentMove.advanceData.crossingSpeed[X_AXIS],
            parentMove.advanceData.crossingSpeed[Y_AXIS],
            parentMove.advanceData.crossingSpeed[Z_AXIS],
            0.0,
            0.0]
        ev = parentMove.endSpeed.speed().vv()
        ev[A_AXIS] = parentMove.advanceData.endEFeedrate()
        moveD.setSpeeds(sv, sv, ev)

        if tl:

            moveB = SubMove(parentMove, parentMove.moveNumber + 2, displacement_vector_steps_B)

            moveB.setDuration(0, tl, 0)

            moveB.startSpeed.setSpeed(topSpeed)
            moveB.topSpeed.setSpeed(topSpeed)
            moveB.endSpeed.setSpeed(topSpeed)

            moveA.nextMove = moveB
            moveB.prevMove = moveA
            moveB.nextMove = moveC
            moveC.prevMove = moveB
            moveC.nextMove = moveD
            moveD.prevMove = moveC

            newMoves.insert(1, moveB)

        else:

            moveA.nextMove = moveC
            moveC.prevMove = moveA
            moveC.nextMove = moveD
            moveD.prevMove = moveC


        # Sum up additional e-distance of this move for debugging
        esteps = displacement_vector_steps_A[A_AXIS]+displacement_vector_steps_B[A_AXIS]+displacement_vector_steps_C[A_AXIS]+displacement_vector_steps_D[A_AXIS]
        # self.advStepBalance += esteps - displacement_vector_steps_raw[A_AXIS]
        self.advStepBalance += esteps
        print "advStepBalance: ", self.advStepBalance

        if debugMoves:
            print "***** End planSALDD() *****"

        assert(util.vectorAdd(util.vectorAdd(util.vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_B[:3]), displacement_vector_steps_C[:3]), displacement_vector_steps_D[:3]) == displacement_vector_steps_raw[:3])

        return newMoves

    #
    # Simple advanceed ramp at start
    # End ramp with sign-change
    # Submoves: A B CD
    def planSADD(self, parentMove):
        assert(0)

        if debugMoves:
            print "***** Start planSADD() *****"
            parentMove.pprint("planSADD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw

        # PART A, XY
        # Neuen displacement_vector_steps aufbauen:
        displacement_vector_steps_A = [0] * 5
        displacement_vector_steps_B = [0] * 5
        displacement_vector_steps_C = [0] * 5

        ta = parentMove.accelTime()
        td = parentMove.decelTime()

        allowedAccelV = parentMove.absGetMaxAllowedAccelVector(self.maxAxisAcceleration)

        startSpeedV = parentMove.startSpeed.trueSpeed().vv()

        for dim in [X_AXIS, Y_AXIS]:

            sa = util.accelDist(abs(startSpeedV[dim]), allowedAccelV[dim], ta)

            steps_per_mm = PrinterProfile.getStepsPerMM(dim)
            steps = int(round(sa * steps_per_mm * util.sign(parentMove.direction[dim])))
            print "dim %d moves %.3f mm while accelerating -> %d steps" % (dim, sa, steps)
            displacement_vector_steps_A[dim] = steps

        # PART A, E
        startSpeedS = parentMove.advanceData.startEFeedrate()
        sa = util.accelDist(abs(startSpeedS), allowedAccelV[A_AXIS], ta)

        assert(parentMove.direction[A_AXIS] > 0)

        assert(0)
        steps = int(round(sa * ee_steps_per_mm))
        print "dim E moves %.3f mm while accelerating -> %d steps" % (sa, steps)
        displacement_vector_steps_A[A_AXIS] = steps





        # Part B, C
        # Time till the e-velocity crosses zero
        endEReachedFeedrate = parentMove.advanceData.endEReachedFeedrate()
        # endEFeedrate = parentMove.advanceData.endEFeedrate()

        tdc = abs(endEReachedFeedrate) / allowedAccelV[A_AXIS]
        sdc = util.accelDist(abs(endEReachedFeedrate), allowedAccelV[A_AXIS], tdc)
        bsteps = int(round(sdc * ee_steps_per_mm))
        print "dim E reaches zero in %.3f s, %.3f mm, %d steps" % (tdc, sdc, bsteps)

        displacement_vector_steps_B[A_AXIS] = bsteps

        # Part B, XY
        reachedSpeedV = parentMove.topSpeed.trueSpeed().vv()

        for dim in [X_AXIS, Y_AXIS]:

            sd = util.accelDist(abs(reachedSpeedV[dim]), allowedAccelV[dim], tdc)

            steps_per_mm = PrinterProfile.getStepsPerMM(dim)
            steps = int(round(sd * steps_per_mm * util.sign(parentMove.direction[dim])))
            print "dim %d moves %.3f mm in phase C -> %d steps" % (dim, sd, steps)
            displacement_vector_steps_B[dim] = steps
       

        # Part C
        # Time of rest of decel ramp
        tdd = td - tdc

        # PART C
        for dim in range(3):
            displacement_vector_steps_C[dim] = displacement_vector_steps_raw[dim] - (displacement_vector_steps_A[dim] + displacement_vector_steps_B[dim])
       
        print "new A steps: ", displacement_vector_steps_A
        print "new B steps: ", displacement_vector_steps_B
        print "new C steps: ", displacement_vector_steps_C

        from move import SubMove

        moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)
        moveB = SubMove(parentMove, parentMove.moveNumber + 2, displacement_vector_steps_B)
        moveC = SubMove(parentMove, parentMove.moveNumber + 3, displacement_vector_steps_C)
       
        moveA.setDuration(ta, 0, 0)
        moveB.setDuration(0, 0, tdc)
        moveC.setDuration(0, 0, tdd)

        topSpeed = parentMove.topSpeed.trueSpeed()
        endSpeed = parentMove.endSpeed.trueSpeed()

        sv = parentMove.startSpeed.trueSpeed().vv()
        sv[A_AXIS] = parentMove.advanceData.startEFeedrate()
        tv = topSpeed.vv()
        tv[A_AXIS] = parentMove.advanceData.startEReachedFeedrate()
        moveA.setSpeeds(sv, tv, tv)

        # Nominal speed at zero crossing
        allowedAccel = parentMove.getMaxAllowedAccel(self.maxAxisAcceleration)
        dv = allowedAccel*tdc
        if topSpeed.feedrate > endSpeed.feedrate:
          # decel
          zeroCrossingS = topSpeed.feedrate - dv
        else:
          assert(0) # does this happen?

        print "top, zerocrossspeed:", topSpeed.feedrate, zeroCrossingS

        sv = topSpeed.vv()
        sv[A_AXIS] = parentMove.advanceData.endEReachedFeedrate()
        ev = [
            parentMove.direction[X_AXIS] * zeroCrossingS,
            parentMove.direction[Y_AXIS] * zeroCrossingS,
            parentMove.direction[Z_AXIS] * zeroCrossingS,
            0.0,
            0.0]
        moveB.setSpeeds(sv, sv, ev)

        sv = [
            parentMove.direction[X_AXIS] * zeroCrossingS,
            parentMove.direction[Y_AXIS] * zeroCrossingS,
            parentMove.direction[Z_AXIS] * zeroCrossingS,
            0.0,
            0.0]
        ev = parentMove.endSpeed.trueSpeed().vv()
        ev[A_AXIS] = parentMove.advanceData.endEFeedrate()
        moveC.setSpeeds(sv, sv, ev)

        moveA.nextMove = moveB
        moveB.prevMove = moveA
        moveB.nextMove = moveC
        moveC.prevMove = moveB

        if debugMoves:
            print "***** End planSADD() *****"

        assert(util.vectorAdd(util.vectorAdd(displacement_vector_steps_A, displacement_vector_steps_B), displacement_vector_steps_C) == displacement_vector_steps_raw)

        return [moveA, moveB, moveC]


    #
    # Simple advanceed ramp at start
    # Linear middle part
    # End ramp with sign-change
    # Submoves: A B CD
    def planSALD(self, parentMove):
        assert(0)

        if debugMoves:
            print "***** Start planSALD() *****"
            parentMove.pprint("planSALD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw

        # PART A, XY
        # Neuen displacement_vector_steps aufbauen:
        displacement_vector_steps_A = [0] * 5
        displacement_vector_steps_B = [0] * 5
        displacement_vector_steps_C = [0] * 5
        displacement_vector_steps_D = [0] * 5

        ta = parentMove.accelTime()
        td = parentMove.decelTime()

        allowedAccelV = parentMove.absGetMaxAllowedAccelVector(self.maxAxisAcceleration)

        startSpeedV = parentMove.startSpeed.trueSpeed().vv()

        for dim in [X_AXIS, Y_AXIS]:

            sa = util.accelDist(abs(startSpeedV[dim]), allowedAccelV[dim], ta)

            steps_per_mm = PrinterProfile.getStepsPerMM(dim)
            steps = int(round(sa * steps_per_mm * util.sign(parentMove.direction[dim])))
            print "dim %d moves %.3f mm while accelerating -> %d steps" % (dim, sa, steps)
            displacement_vector_steps_A[dim] = steps

        # PART A, E
        startSpeedS = parentMove.advanceData.startEFeedrate()
        sa = util.accelDist(abs(startSpeedS), allowedAccelV[A_AXIS], ta)

        assert(parentMove.direction[A_AXIS] > 0)

        assert(0)
        steps = int(round(sa * ee_steps_per_mm))
        print "dim E moves %.3f mm while accelerating -> %d steps" % (sa, steps)
        displacement_vector_steps_A[A_AXIS] = steps





        # Part C, E
        # Time till the e-velocity crosses zero
        endEReachedFeedrate = parentMove.advanceData.endEReachedFeedrate()
        # endEFeedrate = parentMove.advanceData.endEFeedrate()

        tdc = abs(endEReachedFeedrate) / allowedAccelV[A_AXIS]
        sdc = util.accelDist(abs(endEReachedFeedrate), allowedAccelV[A_AXIS], tdc)
        csteps = int(round(sdc * ee_steps_per_mm))
        print "dim E reaches zero in %.3f s, %.3f mm, %d steps" % (tdc, sdc, csteps)

        displacement_vector_steps_C[A_AXIS] = csteps

        # Part C, XY
        reachedSpeedV = parentMove.topSpeed.trueSpeed().vv()

        for dim in [X_AXIS, Y_AXIS]:

            sd = util.accelDist(abs(reachedSpeedV[dim]), allowedAccelV[dim], tdc)

            steps_per_mm = PrinterProfile.getStepsPerMM(dim)
            steps = int(round(sd * steps_per_mm * util.sign(parentMove.direction[dim])))
            print "dim %d moves %.3f mm in phase C -> %d steps" % (dim, sd, steps)
            displacement_vector_steps_C[dim] = steps
       

        # Part D, E
        # Time of rest of decel ramp
        tdd = td - tdc
        sdd = util.accelDist(abs(endEFeedrate), allowedAccelV[A_AXIS], tdd)
        dsteps = int(round(sdd * ee_steps_per_mm))
        print "dim E remaining %.3f s, %.3f mm, %d steps" % (tdd, sdd, dsteps)

        displacement_vector_steps_D[A_AXIS] = dsteps

        # Part C, XY
        for dim in [X_AXIS, Y_AXIS]:

            sd = util.accelDist(0, allowedAccelV[dim], tdd)

            steps_per_mm = PrinterProfile.getStepsPerMM(dim)
            steps = int(round(sd * steps_per_mm * util.sign(parentMove.direction[dim])))
            print "dim %d moves %.3f mm in phase D -> %d steps" % (dim, sd, steps)
            displacement_vector_steps_D[dim] = steps
       
        # PART B, XY
        for dim in range(3):
            displacement_vector_steps_B[dim] = displacement_vector_steps_raw[dim] - (displacement_vector_steps_A[dim] + displacement_vector_steps_C[dim] + displacement_vector_steps_D[dim])
       
        print "new A steps: ", displacement_vector_steps_A
        print "new B steps: ", displacement_vector_steps_B
        print "new C steps: ", displacement_vector_steps_C
        print "new D steps: ", displacement_vector_steps_D

        from move import SubMove

        moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)
        moveB = SubMove(parentMove, parentMove.moveNumber + 2, displacement_vector_steps_B)
        moveC = SubMove(parentMove, parentMove.moveNumber + 3, displacement_vector_steps_C)
        moveD = SubMove(parentMove, parentMove.moveNumber + 4, displacement_vector_steps_D)
       
        moveA.setDuration(ta, 0, 0)
        moveB.setDuration(0, parentMove.linearTime(), 0)
        moveC.setDuration(0, 0, tdc)
        moveD.setDuration(0, 0, tdd)

        topSpeed = parentMove.topSpeed.trueSpeed()
        endSpeed = parentMove.endSpeed.trueSpeed()

        sv = parentMove.startSpeed.trueSpeed().vv()
        sv[A_AXIS] = parentMove.advanceData.startEFeedrate()
        tv = topSpeed.vv()
        tv[A_AXIS] = parentMove.advanceData.startEReachedFeedrate()
        moveA.setSpeeds(sv, tv, tv)

        moveB.startSpeed.nominalSpeed(topSpeed)
        moveB.topSpeed.nominalSpeed(topSpeed)
        moveB.endSpeed.nominalSpeed(topSpeed)

        # Nominal speed at zero crossing
        allowedAccel = parentMove.getMaxAllowedAccel(self.maxAxisAcceleration)
        dv = allowedAccel*tdc
        if topSpeed.feedrate > endSpeed.feedrate:
          # decel
          zeroCrossingS = topSpeed.feedrate - dv
        else:
          assert(0) # does this happen?

        print "top, zerocrossspeed:", topSpeed.feedrate, zeroCrossingS

        sv = topSpeed.vv()
        sv[A_AXIS] = parentMove.advanceData.endEReachedFeedrate()
        ev = [
            parentMove.direction[X_AXIS] * zeroCrossingS,
            parentMove.direction[Y_AXIS] * zeroCrossingS,
            parentMove.direction[Z_AXIS] * zeroCrossingS,
            0.0,
            0.0]
        moveC.setSpeeds(sv, sv, ev)

        sv = [
            parentMove.direction[X_AXIS] * zeroCrossingS,
            parentMove.direction[Y_AXIS] * zeroCrossingS,
            parentMove.direction[Z_AXIS] * zeroCrossingS,
            0.0,
            0.0]
        ev = parentMove.endSpeed.trueSpeed().vv()
        ev[A_AXIS] = parentMove.advanceData.endEFeedrate()
        moveD.setSpeeds(sv, sv, ev)

        moveA.nextMove = moveB
        moveB.prevMove = moveA
        moveB.nextMove = moveC
        moveC.prevMove = moveB
        moveC.nextMove = moveD
        moveD.prevMove = moveC

        if debugMoves:
            print "***** End planSALD() *****"

        assert(util.vectorAdd(util.vectorAdd(util.vectorAdd(displacement_vector_steps_A, displacement_vector_steps_B), displacement_vector_steps_C), displacement_vector_steps_D) == displacement_vector_steps_raw)

        return [moveA, moveB, moveC, moveD]



