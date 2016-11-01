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
import math, collections
import pprint

#####################################################################
if debugPlot:
    import pickle

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

    def __init__(self, planner, args): # , gui=None):

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

        # K-Advance is defined in material profile and overridable by the
        # commandline
        if args.kAdvance != None:
            self.kAdv = args.kAdvance
        else:
            self.kAdv = MatProfile.getKAdv()

        if self.kAdv:

            print "Advance: usinge K-Advance %.3f" % self.kAdv

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

        self.e_steps_per_mm = PrinterProfile.getStepsPerMM(A_AXIS)

        # Compute minimal e speed
        maxStepTime = maxTimerValue16 / fTimer
        v = (1.0/self.e_steps_per_mm) / maxStepTime
        self.minESpeed = v * 1.1 # savety margin for rounding errors
        print "min e-speed", self.minESpeed

    def eJerk(self, accel):
        return abs(accel) * self.kAdv

    def resetDebugStatistics(self):

        # Running sum of e-distances through advance, for debugging of planAdvance()
        self.advSum = 0.0
        # Sum of skipped small accelration ramps
        self.skippedAdvance = 0.0
        self.skippedSimpleSteps = 0.0

        # Running sum of move esteps, for debugging of planSteps()
        self.moveEsteps = 0.0

        self.longestRampMove = None
        self.longestRampAdvance = 0

    def planPath(self, path):

        if debugMoves:
            print "***** Start planPath() *****"

        self.resetDebugStatistics()

        if debugPlot:
            self.plottime = 1
            self.plotfile = DebugPlot(path[0].moveNumber)

        # First move
        v0 = path[0].startSpeed.speed()
        # xxx use same start speed as PrintMove::sanityCheck() here!
        # v0.setSpeed(0.1)
        v0.setSpeed(0.0)
        path[0].startSpeed.setSpeed(v0)

        # Last move
        # xxx use same start speed as PrintMove::sanityCheck() here!
        v0 = path[-1].endSpeed.speed()
        # v0.setSpeed(0.1)
        v0.setSpeed(0.0)
        path[-1].endSpeed.setSpeed(v0)

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
            # move.pprint("sanicheck")
            move.sanityCheck(self.planner.jerk)

        # Step 2: join moves backwards
        self.joinMovesBwd(path)

        # Sanity check
        for move in path:

            # move.pprint("sanicheck")
            move.sanityCheck(self.planner.jerk)

        # Step 3: handle extruder advance

        if debugMoves:
            print "Streaming %d moves..." % len(path)

        for move in path:

            move.sanityCheck(self.planner.jerk)
            self.planAcceleration(move)

        if self.kAdv:

            self.groupAdvance(path)

            for move in path:
                move.sanityCheck(self.planner.jerk)

                # Sum up esteps
                self.moveEsteps += move.eSteps
                print "moveEsteps+: %7.3f %7.3f" % ( move.eSteps, self.moveEsteps)

            self.planAdvanceGroup(path)

            print "Path skippedAdvance: ", self.skippedAdvance, "-->", self.skippedAdvance*self.e_steps_per_mm, "e-steps"

            if self.skippedAdvance > 1.0/self.e_steps_per_mm:
                print "spread skippedAdvance accel on biggest ramp: ", self.longestRampMove, self.longestRampAdvance

                self.longestRampMove.pprint("longestRampMove")

                ta = self.longestRampMove.accelTime()

                startIncrease = self.skippedAdvance / ta

                print "Increasing startramp by: ", startIncrease, "[mm/s]"

                # XXX this possibly increases e-jerk above max e-jerk, a better way is to decrease
                # the e-jerk of a decel advance ramp...

                assert(startIncrease < 1) # xxx constrain it

                self.longestRampMove.advanceData.startFeedrateIncrease += startIncrease

                estepIncrease = self.skippedAdvance * self.e_steps_per_mm 

                print "Increasing startramp by: ", estepIncrease, "[steps]"

                self.longestRampMove.advanceData.startESteps += estepIncrease
                self.longestRampMove.advanceData.advStepSum += estepIncrease

                self.skippedAdvance = 0

                self.longestRampMove.pprint("spread longestRampMove")

                # assert(0)

            elif self.skippedAdvance < -1.0/self.e_steps_per_mm:

                # xxxx using longest start ramp instead of longest end ramp here! maybe better change this
                # xxx wenn der positive start advance benutzt wird, besteht keine gefahr, einen 'crossing decel'
                # zu verändern.

                print "spread skippedAdvance accel on biggest ramp: ", self.longestRampMove, self.longestRampAdvance

                self.longestRampMove.pprint("longestRampMove")

                ta = self.longestRampMove.accelTime()

                startIncrease = self.skippedAdvance / ta

                print "Increasing startramp by: ", startIncrease, "[mm/s]"

                # XXX this possibly increases e-jerk above max e-jerk, a better way is to decrease
                # the e-jerk of a decel advance ramp...

                assert(startIncrease > -1) # xxx constrain it

                self.longestRampMove.advanceData.startFeedrateIncrease += startIncrease
                assert(self.longestRampMove.advanceData.startFeedrateIncrease >= 0)

                estepIncrease = self.skippedAdvance * self.e_steps_per_mm 

                print "Increasing startramp by: ", estepIncrease, "[steps]"

                self.longestRampMove.advanceData.startESteps += estepIncrease
                self.longestRampMove.advanceData.advStepSum += estepIncrease

                self.skippedAdvance = 0

                self.longestRampMove.pprint("spread longestRampMove")

                # assert(0)

        if debugPlot and debugPlotLevel == "plotLevelPlanned":

            # xxx todo move to own function
            for move in path:

                self.plotfile.plot1Tick(move.topSpeed.trueSpeed().feedrate3(), move.moveNumber)

                at = move.accelTime()
                lt = move.linearTime()
                dt = move.decelTime()

                if at:

                    self.plotfile.plot1Segments(at, (
                        DebugPlotSegment(move.startSpeed.trueSpeed().feedrate3(), move.topSpeed.trueSpeed().feedrate3(), "green"),
                        DebugPlotSegment(move.startSpeed.trueSpeed().eSpeed, move.topSpeed.trueSpeed().eSpeed, "green"),
                        ))

                if lt:

                    self.plotfile.plot1Segments(lt, (
                        DebugPlotSegment(move.topSpeed.trueSpeed().feedrate3(), color="blue"),
                        DebugPlotSegment(move.topSpeed.trueSpeed().eSpeed, color="blue"),
                        ))

                if dt:

                    self.plotfile.plot1Segments(dt, (
                        DebugPlotSegment(move.topSpeed.trueSpeed().feedrate3(), move.endSpeed.trueSpeed().feedrate3(), "red"),
                        DebugPlotSegment(move.topSpeed.trueSpeed().eSpeed, move.endSpeed.trueSpeed().eSpeed, "red"),
                        ))

                if at:

                    self.plotfile.plot2Segments(0, (
                        DebugPlotSegment(move.startSpeed.trueSpeed().eSpeed, move.advanceData.startEFeedrate(), "green"),
                        ))
                    self.plotfile.plot2Segments(at, (
                        DebugPlotSegment(move.startSpeed.trueSpeed().eSpeed, move.topSpeed.trueSpeed().eSpeed),
                        DebugPlotSegment(move.advanceData.startEFeedrate(), move.advanceData.startEReachedFeedrate(), "green"),
                        ))
                    self.plotfile.plot2Segments(0, (
                        DebugPlotSegment(move.advanceData.startEReachedFeedrate(), move.topSpeed.trueSpeed().eSpeed, "green"),
                        ))

                if lt:
                    self.plotfile.plot2Segments(lt, (
                        DebugPlotSegment(move.topSpeed.trueSpeed().eSpeed),
                        ))

                if dt:

                    self.plotfile.plot2Segments(0, (
                        DebugPlotSegment(move.topSpeed.trueSpeed().eSpeed, move.advanceData.endEReachedFeedrate(), "red"),
                        ))
                    self.plotfile.plot2Segments(dt, (
                        DebugPlotSegment(move.topSpeed.trueSpeed().eSpeed, move.endSpeed.trueSpeed().eSpeed),
                        DebugPlotSegment(move.advanceData.endEReachedFeedrate(), move.advanceData.endEFeedrate(), "red"),
                        ))
                    self.plotfile.plot2Segments(0, (
                        DebugPlotSegment(move.advanceData.endEFeedrate(), move.endSpeed.trueSpeed().eSpeed, "red"),
                        ))

            self.plotfile.close()

        newPath = []
        for move in path:

            newMoves = self.planSteps(move)
            newPath += newMoves

        if debugPlot and debugPlotLevel == "plotLevelSplitted":

            for move in newPath:

                # xxx todo move to own function
                self.plotfile.plot1Tick(move.topSpeed.trueSpeed().feedrate3(), move.moveNumber)

                at = move.accelTime()
                lt = move.linearTime()
                dt = move.decelTime()

                if at:

                    self.plotfile.plot1Segments(at, (
                        DebugPlotSegment(move.startSpeed.trueSpeed().feedrate3(), move.topSpeed.trueSpeed().feedrate3(), "green"),
                        ))
                    self.plotfile.plot2Segments(at, (
                        DebugPlotSegment(move.startSpeed.trueSpeed().eSpeed, move.topSpeed.trueSpeed().eSpeed, "green"),
                        ))

                if lt:

                    self.plotfile.plot1Segments(lt, (
                        DebugPlotSegment(move.topSpeed.trueSpeed().feedrate3(), color="blue"),
                        ))
                    self.plotfile.plot2Segments(lt, (
                        DebugPlotSegment(move.topSpeed.trueSpeed().eSpeed, color="blue"),
                        ))

                if dt:

                    self.plotfile.plot1Segments(dt, (
                        DebugPlotSegment(move.topSpeed.trueSpeed().feedrate3(), move.endSpeed.trueSpeed().feedrate3(), "red"),
                        ))
                    self.plotfile.plot2Segments(dt, (
                        DebugPlotSegment(move.topSpeed.trueSpeed().eSpeed, move.endSpeed.trueSpeed().eSpeed, "red"),
                        ))

                self.plotfile.close()

        print "\nPath advSum: ", self.advSum
        print "Path skippedAdvance: ", self.skippedAdvance, self.skippedAdvance*self.e_steps_per_mm
        print "Path skippedSimpleSteps: ", self.skippedSimpleSteps

        if self.kAdv:
            print "Path moveEsteps:", self.moveEsteps, len(newPath)

        # Summe aller advance-rampen muss nicht unbedingt null sein, je nach verteilung
        # von e-jerk jumps. Somit ist folgender test völlig willkürlich.
        assert(util.circaf(self.advSum, 0, 1))

        # assert(util.circaf(self.skippedAdvance, 0, 2.0/self.e_steps_per_mm))
        assert(util.circaf(self.skippedAdvance, 0, 1.0/self.e_steps_per_mm))
        assert(util.circaf(self.skippedSimpleSteps, 0, 1.001))

        if self.kAdv:
            assert(util.circaf(self.moveEsteps, 0, 1.001))

        # Debug, check chain
        n = 2
        m = newPath[0]
        while m.nextMove:
            m = m.nextMove
            n += 1
        m = newPath[-1]
        while m.prevMove:
            m = m.prevMove
            n += 1
        assert(n == 2*len(newPath))
        # Debug, end check chain

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

            startSpeed1 = move.startSpeed.speed()
            startSpeedS = startSpeed1.feedrate3()

            endSpeed1 = move.endSpeed.speed()
            endSpeedS = endSpeed1.feedrate3()

            av = move.getMaxAllowedAccelVector5(self.maxAxisAcceleration)
            allowedAccel3 = util.vectorLength(av[:3])
            # allowedAccel = move.getMaxAllowedAccel5(self.maxAxisAcceleration)
            maxAllowedStartSpeed = util.vAccelPerDist(endSpeedS, allowedAccel3, move.distance3)

            # print "joinMovesBwd, startspeed, max startspeed: ", move.getStartFr(), maxAllowedStartSpeed

            # print "WARNING: joinMovesBwd() no check of e-axis"

            maxAllowedEStartSpeed = util.vAccelPerDist(endSpeed1.eSpeed, av[A_AXIS], move.eDistance)

            if maxAllowedStartSpeed >= startSpeedS:
                # good, move is ok

                assert(maxAllowedEStartSpeed >= startSpeed1.eSpeed)
                continue

            if debugMoves: 
                print "Startspeed of %.5f is to high to reach the desired endspeed of %.5f." % (startSpeedS, endSpeedS)
                print "Max. allowed startspeed: %.5f." % maxAllowedStartSpeed

            # Adjust startspeed of this move:
            startSpeed1.setSpeed(maxAllowedStartSpeed)

            if startSpeed1.eSpeed > maxAllowedEStartSpeed:
                print "startSpeed1.eSpeed higher than maxAllowedEStartSpeed:", startSpeed1.eSpeed, maxAllowedEStartSpeed

                factor = maxAllowedEStartSpeed / startSpeed1.eSpeed

                print "factor: ", factor
                assert(factor < 1)

                startSpeed1 = startSpeed1.scale(factor)

            move.startSpeed.setSpeed(startSpeed1)

            if move.prevMove:

                #
                # Adjust endspeed of the previous move, also.
                #

                factor = startSpeed1.feedrate3() / startSpeedS

                print "factor: ", factor

                # Adjust endspeed of last move:
                assert(factor < 1)

                # XXX einfacher algo, kann man das besser machen (z.b. mit jerk-berechnung,
                # vector subtraktion oder so?)
                # move.prevMove.setTrueEndFr(move.prevMove.getEndFr() * factor)
                endSpeed0 = move.prevMove.endSpeed.speed().scale(factor)
                move.prevMove.endSpeed.setSpeed(endSpeed0)

        if debugMoves:
            print "***** End joinMovesBwd() *****"

    def planAcceleration(self, move):

        if debugMoves: 
            move.pprint("Start planAcceleration")
            print "***** Start planAcceleration() *****"

        move.state = 2

        allowedAccel3 = util.vectorLength(move.getMaxAllowedAccelVector5(self.maxAxisAcceleration)[:3])
        av = move.getMaxAllowedAccelVector5(self.maxAxisAcceleration)
        print "allowed accel: ", av, allowedAccel3, util.vectorLength(av)

        #
        # Check if the speed difference between startspeed and endspeed can be done with
        # this acceleration in this distance
        #
        startSpeed = move.startSpeed.speed()
        startSpeedS = startSpeed.feedrate3()

        topSpeed = move.topSpeed.speed()

        endSpeed = move.endSpeed.speed()
        endSpeedS = endSpeed.feedrate3()

        deltaSpeedS = endSpeedS - startSpeedS

        if abs(deltaSpeedS) > 0.001:
       
            ta = abs(deltaSpeedS) / allowedAccel3

            sa = util.accelDist(startSpeedS, util.sign(deltaSpeedS) * allowedAccel3, ta)
      
            if (sa - move.distance3) > 0.001:
                print " 0.5 * %f * pow(%f, 2) + %f * %f" % (allowedAccel3, ta, startSpeedS, ta)
                print "VStart %f mm/s kann nicht innerhalb von %f mm auf Endgeschwindigkeit %f mm/s gebracht werden!" % (startSpeedS, move.distance3, endSpeedS)
                print "Dafür werden %f mm benötigt" % sa
                assert(0)

        deltaESpeed = endSpeed.eSpeed - startSpeed.eSpeed

        if abs(deltaESpeed) > 0.001:
       
            ta = abs(deltaESpeed) / av[A_AXIS]

            sa = util.accelDist(startSpeed.eSpeed, util.sign(deltaESpeed) * av[A_AXIS], ta)
      
            if (sa - move.eDistance) > 0.001:
                # print " 0.5 * %f * pow(%f, 2) + %f * %f" % (allowedAccel, ta, startSpeedS, ta)
                # print "VStart %f mm/s kann nicht innerhalb von %f mm auf Endgeschwindigkeit %f mm/s gebracht werden!" % (startSpeedS, move.distance3, endSpeedS)
                # print "Dafür werden %f mm benötigt" % sa
                assert(0)


        #
        # Compute distance to accel from start speed to nominal speed:
        #
        ta = 0.0
        sa = 0.0

        deltaStartSpeedS = topSpeed.feedrate3() - startSpeedS

        if deltaStartSpeedS > AccelThreshold:

            ta = deltaStartSpeedS / allowedAccel3
            print "accel time (for %f mm/s): %f [s]" % (deltaStartSpeedS, ta)

            # debug Check axxis acceleration
            deltaSpeedV = move.direction3.scale(deltaStartSpeedS)
            for dim in range(3):
                dimAccel = abs(deltaSpeedV[dim]) / ta
                print "dimaccel ", dim, dimAccel
                if (dimAccel / self.maxAxisAcceleration[dim]) > 1.001:
                    print "dim %d verletzt max accel: " % dim, dimAccel, " > ", self.maxAxisAcceleration[dim]
                    assert(0)

            # print "WARNING: planAcceleration() no check of e-axis"
            eAccel = (topSpeed.eSpeed - startSpeed.eSpeed) / ta
            print "eaccel: ", eAccel, av
            assert((eAccel / av[A_AXIS]) < 1.001)
            #end debug

            sa = util.accelDist(startSpeedS, allowedAccel3, ta)

        #
        # Compute distance to deccel from nominal speed to endspeed:
        #
        tb = 0.0
        sb = 0.0

        deltaEndSpeedS = topSpeed.feedrate3() - endSpeedS                          # [mm/s]

        if deltaEndSpeedS > AccelThreshold:

            tb = deltaEndSpeedS / allowedAccel3                          # [s]
            print "XY: deccel time (for %f mm/s): %f [s]" % (deltaEndSpeedS, tb)

            # debug Check axxis acceleration
            deltaSpeedV = move.direction3.scale(deltaEndSpeedS)
            for dim in range(3):
                dimDeccel = abs(deltaSpeedV[dim]) / tb  
                if (dimDeccel / self.maxAxisAcceleration[dim]) > 1.001:
                    print "dim %d verletzt max accel: " % dim, dimDeccel, " [mm/s] > ", self.maxAxisAcceleration[dim], " [mm/s]"
                    assert(0)

            # print "WARNING: planAcceleration() no check of e-axis"
            eAccel = (topSpeed.eSpeed - endSpeed.eSpeed) / tb
            print "edecel: ", eAccel, av
            assert((eAccel / av[A_AXIS]) < 1.001)

            # end debug

            sb = util.accelDist(endSpeedS, allowedAccel3, tb)

        print "e_distance: %f, sbeschl, sbrems: %f, %f" % (move.distance3, sa, sb)

        if (sa+sb) and (move.distance3 < (sa+sb)):

            #
            # Strecke zu kurz, Trapez nicht möglich, geschwindigkeit muss abgesenkt werden.
            #
            if debugMoves:
                print "Trapez nicht möglich: s: %f, sbeschl (%f) + sbrems (%f) = %f" % (move.distance3, sa, sb, sa+sb)

            sa = min(
                    (2 * allowedAccel3 * move.distance3 - pow(startSpeedS, 2) + pow(endSpeedS, 2)) / (4 * allowedAccel3),
                    move.distance3
                    )

            sb = move.distance3 - sa

            # 
            # Geschwindigkeit, die auf strecke sa mit erreicht werden kann
            # 
            v = math.sqrt ( 2 * allowedAccel3 * sa + pow(startSpeedS, 2) )

            v = max(v, startSpeedS)
            v = max(v, endSpeedS)

            # debug, test
            if sb:

                v2 = math.sqrt ( 2 * allowedAccel3 * sb + pow(endSpeedS, 2) )
                print "move.feedrate neu: %f (test: %f, diff: %f)" % (v, v2, abs(v - v2))

                assert( abs(v - v2) < 0.001)
            # end debug

            # topSpeed.setSpeed(v / util.RoundSafe)
            topSpeed.setSpeed(v / 0.999999)

            if debugMoves:
                print "sbeschl, sbrems neu: %f, %f" % (sa, sb), ", reachable topspeed: ", topSpeed

            move.topSpeed.setSpeed(topSpeed)

            deltaSpeedS = v - startSpeedS                          # [mm/s]
            ta = deltaSpeedS / allowedAccel3
            print "ta: ", ta, deltaSpeedS

            deltaSpeedS = v - endSpeedS                          # [mm/s]
            tb = deltaSpeedS / allowedAccel3
            print "tb: ", tb, deltaSpeedS

            move.setDuration(ta, 0, tb)

            if debugMoves:
                move.pprint("End planAcceleration")
                print 
                print "***** End planAcceleration() *****"
                
            return

        # 
        # Strecke reicht aus, um auf nominal speed zu beschleunigen
        # 

        # print "ta: ", ta, deltaStartSpeedS, sa
        # print "tb: ", tb, deltaEndSpeedS, sb

        nominalSpeed = move.topSpeed.plannedSpeed().feedrate3() # [mm/s]
        slin = move.distance3 - (sa+sb)
        tlin = slin / nominalSpeed
        # print "tlin: ", tlin, slin
        move.setDuration(ta, tlin, tb)

        if debugMoves:
            move.pprint("End planAcceleration")
            print 
            print "***** End planAcceleration() *****"
            # if move.moveNumber == 40: assert(0)

    ################################################################################

    # Group consecutive acceleration and deceleration advanced ramps.
    def groupAdvance(self, path):

        if debugMoves:
            print "***** Start groupAdvance() *****"
            # move.pprint("groupAdvance:")

        accelGroup = None
        for move in path:

            allowedAccelV = move.getMaxAllowedAccelVector5(self.maxAxisAcceleration)
            startFeedrateIncrease = self.eJerk(allowedAccelV[A_AXIS])
            (sadv, _) = move.startAdvSteps(startFeedrateIncrease=startFeedrateIncrease)

            if sadv:

                if not accelGroup:
                    print "start accelgroup with move", move.moveNumber
                    accelGroup = move

                move.advanceData.sAccel = sadv

                print "add accelgroup:", move.moveNumber
                accelGroup.advanceData.accelGroup.append(move)
                accelGroup.advanceData.sAccelSum += sadv

            if (not sadv) or move.linearTime() or move.decelTime():
                if accelGroup:
                    print "end accelgroup with move", move.moveNumber, sadv, move.linearTime(), move.decelTime()
                accelGroup = None

        decelGroup = None
        for index in range(len(path)): 

            move = path[len(path) - 1 - index]

            allowedAccelV = move.getMaxAllowedAccelVector5(self.maxAxisAcceleration)
            endFeedrateIncrease = - self.eJerk(allowedAccelV[A_AXIS])
            (sdec, _) = move.endAdvSteps(endFeedrateIncrease=endFeedrateIncrease)

            if sdec:

                if not decelGroup:
                    print "start decelgroup with move", move.moveNumber
                    decelGroup = move

                move.advanceData.sDecel = sdec

                print "add decelgroup:", move.moveNumber, "sdec:", sdec
                decelGroup.advanceData.decelGroup.append(move)
                decelGroup.advanceData.sDecelSum += sdec

            if (not sdec) or move.linearTime() or move.accelTime():

                if decelGroup:
                    print "end decelgroup with move", move.moveNumber, move.accelTime(), move.linearTime()
                decelGroup = None

        for move in path:
            if move.advanceData.accelGroup or move.advanceData.decelGroup:
                print "group: ", move.moveNumber, move.advanceData


    def planAdvanceGroup(self, path):

        if debugMoves:
            print "***** Start planAdvanceGroup() *****"
            # move.pprint("planAdvanceGroup:")

        for baseMove in path:

            # No advance if there are no (accel- or decel-) ramps.
            if not (baseMove.advanceData.accelGroup or baseMove.advanceData.decelGroup):

                print "group: ", baseMove.moveNumber, baseMove.advanceData

                if debugMoves:
                    print "***** End planAdvanceGroup() *****"

                continue

            if baseMove.advanceData.accelGroup:

                print "accel group: ", baseMove.moveNumber, baseMove.advanceData

                if ((baseMove.advanceData.sAccelSum + self.skippedAdvance) * self.e_steps_per_mm) > AdvanceMinRamp:

                    for advMove in baseMove.advanceData.accelGroup:

                        allowedAccelV = advMove.getMaxAllowedAccelVector5(self.maxAxisAcceleration)
                        startFeedrateIncrease = self.eJerk(allowedAccelV[A_AXIS])
                        advMove.advanceData.startFeedrateIncrease = startFeedrateIncrease

                        advMove.pprint("planAdvanceGroup accel adv:")

                        (sa, esteps, usedRoundError) = advMove.startERampSteps(self.planner.jerk[A_AXIS], roundError=self.skippedAdvance)
                        self.skippedAdvance -= usedRoundError

                        if sa > self.longestRampAdvance:
                            self.longestRampMove = advMove
                            self.longestRampAdvance = sa

                        advMove.advanceData.startESteps = esteps
                        advMove.advanceData.advStepSum += esteps

                        advMove.advanceData.startSplits = 1
                        if advMove.advanceData.startSignChange():
                            assert(0) # should not happen

                        startIncrease = usedRoundError / advMove.accelTime()

                        print "Adjust startFeedrateIncrease %f by %f" %(startFeedrateIncrease, startIncrease)
                        assert(startIncrease <= 1.001) # xxx constrain it
                        advMove.advanceData.startFeedrateIncrease += startIncrease

                        tl = advMove.linearTime()
                        if tl and not advMove.advanceData.linESteps:

                            # E-steps in linear phase
                            advMove.pprint("planAdvanceGroup linear phase:")

                            # E-distance ist nicht einfach der rest der e-steps, linearer e-anteil muss über
                            # die dauer des linearen anteils (tLinear) berechnet werden.
                            sl = tl * advMove.topSpeed.speed().eSpeed
                            esteps = sl * self.e_steps_per_mm
                            print "dim E moves %.3f mm in linear phase -> %d steps" % (sl, esteps)
    
                            advMove.advanceData.linESteps = esteps
                            advMove.advanceData.advStepSum += esteps

                else:
                    
                    assert(len(baseMove.advanceData.accelGroup) == 1)

                    # Dont advance very small acceleration ramps, but sum up the missing advance.
                    baseMove.pprint("planAdvanceGroup skipped adv:")

                    # E-steps of non-advanced accel ramp
                    sa = baseMove.startRampDistance(
                            baseMove.startSpeed.speed().eSpeed, baseMove.topSpeed.speed().eSpeed, baseMove.accelTime())

                    esteps = sa * self.e_steps_per_mm
                    print "dim E moves %.3f mm in accel phase -> %d steps" % (sa, esteps)

                    baseMove.advanceData.startESteps = esteps
                    baseMove.advanceData.advStepSum += esteps
                    self.skippedAdvance += baseMove.advanceData.sAccelSum

                    # xxx longramp

                self.advSum += baseMove.advanceData.sAccelSum

            if baseMove.advanceData.decelGroup:

                print "decel group: ", baseMove.moveNumber, baseMove.advanceData

                if ((baseMove.advanceData.sDecelSum + self.skippedAdvance) * self.e_steps_per_mm + self.skippedAdvance) < -AdvanceMinRamp:

                    for advMove in baseMove.advanceData.decelGroup:

                        allowedAccelV = advMove.getMaxAllowedAccelVector5(self.maxAxisAcceleration)
                        endFeedrateIncrease = - self.eJerk(allowedAccelV[A_AXIS])
                        advMove.advanceData.endFeedrateIncrease = endFeedrateIncrease

                        advMove.pprint("planAdvanceGroup decel adv:")

                        advMove.advanceData.endSplits = 1

                        if advMove.advanceData.endSignChange(): 

                            ###############################################################
                            # Compute additional data for planSteps()
                            # Time till the e-velocity crosses zero

                            td = advMove.decelTime()
                            topSpeed = advMove.topSpeed.speed()
                            endSpeed = advMove.endSpeed.speed()

                            tdc = 0
                            crossingSpeed = topSpeed

                            v0 = advMove.advanceData.endEReachedFeedrate()

                            # Skip first part of crossing decel ramp if its startspeed
                            # is to low.
                            # if abs(v0) > self.minESpeed:
                            if True:

                                print "v0, accel: ", v0, allowedAccelV[A_AXIS]
                                tdc = abs(v0 / allowedAccelV[A_AXIS])
                                print "Time to reach zero-crossing (tdc):", tdc, ", tdd: ", td - tdc

                                # Nominal speed at zero crossing
                                allowedAccelXYZ = util.vectorLength(allowedAccelV[:3])

                                if topSpeed.feedrate3() > endSpeed.feedrate3():
                                    # decel
                                    zeroCrossingS = util.vAccelPerTime(topSpeed.feedrate3(), -allowedAccelXYZ, tdc)
                                else:
                                    assert(0) # does this happen?

                                crossingSpeed = advMove.topSpeed.speed()
                                crossingSpeed.setSpeed(zeroCrossingS)

                                # PART C, E
                                (sdc, estepsc, usedRoundError) = advMove.endERampSteps(
                                        self.planner.jerk[A_AXIS], td=tdc, v1=crossingSpeed.eSpeed) # , roundError=self.skippedAdvance)

                                endIncrease = usedRoundError / advMove.decelTime()

                                print "Adjust endFeedrateIncrease %f by %f" % (endFeedrateIncrease, endIncrease)
                                assert(endIncrease >= -1.001) # xxx constrain it
                                advMove.advanceData.endFeedrateIncrease += endIncrease

                                if usedRoundError:
                                    #xxx two different endFeedrateIncrease members needed
                                    #xxx crossing pont depends on endFeedrateIncrease!
                                    assert(0)

                                print "(sdc, estepsc):", (sdc, estepsc) 

                                advMove.advanceData.endEStepsC = estepsc
                                advMove.advanceData.advStepSum += estepsc

                            else:
                                assert(0)
                                advMove.advanceData.endEStepsC = 0

                            print "top, zerocrossspeed:", topSpeed.feedrate3(), crossingSpeed

                            advMove.advanceData.tdc = tdc
                            advMove.advanceData.tdd = td - tdc
                            advMove.advanceData.crossingSpeed = crossingSpeed

                            (sdd, estepsd, usedRoundError) = advMove.endERampSteps(
                                    self.planner.jerk[A_AXIS], td=advMove.advanceData.tdd, v0=crossingSpeed.eSpeed) # , roundError=ediffc)

                            endIncrease = usedRoundError / advMove.decelTime()

                            print "Adjust endFeedrateIncrease %f by %f" % (endFeedrateIncrease, endIncrease)
                            assert(endIncrease >= -1.001) # xxx constrain it
                            advMove.advanceData.endFeedrateIncrease += endIncrease

                            if usedRoundError:
                                #xxx two different endFeedrateIncrease members needed
                                #xxx crossing pont depends on endFeedrateIncrease!
                                assert(0)

                            print "(sdd, estepsd):", (sdd, estepsd) 

                            advMove.advanceData.endEStepsD = estepsd
                            advMove.advanceData.advStepSum += estepsd

                            advMove.advanceData.endSplits += 1 # Mark as crossing decel ramp

                            ###############################################################

                        else:

                            (sd, esteps, usedRoundError) = advMove.endERampSteps(
                                    self.planner.jerk[A_AXIS], roundError=self.skippedAdvance)

                            print "sd: %f, skippedAdvance: %f, used: %f" % (sd, self.skippedAdvance, usedRoundError)
                            self.skippedAdvance -= usedRoundError
                            
                            
                            # xxx longramp

                            advMove.advanceData.endESteps = esteps
                            advMove.advanceData.advStepSum += esteps

                            endIncrease = usedRoundError / advMove.decelTime()

                            print "Adjust endFeedrateIncrease %f by %f" % (endFeedrateIncrease, endIncrease)
                            assert(endIncrease >= -1.001) # xxx constrain it
                            advMove.advanceData.endFeedrateIncrease += endIncrease

                            # xxx check if move is a crossing decel move after adjust:
                            assert(not advMove.advanceData.endSignChange())

                            # if usedRoundError:
                                # assert(0)

                        tl = advMove.linearTime()
                        if tl and not advMove.advanceData.linESteps:

                            # E-steps in linear phase
                            advMove.pprint("planAdvanceGroup linear phase:")

                            # E-distance ist nicht einfach der rest der e-steps, linearer e-anteil muss über
                            # die dauer des linearen anteils (tLinear) berechnet werden.
                            sl = tl * advMove.topSpeed.speed().eSpeed
                            esteps = sl * self.e_steps_per_mm
                            print "dim E moves %.3f mm in linear phase -> %d steps" % (sl, esteps)
    
                            advMove.advanceData.linESteps = esteps
                            advMove.advanceData.advStepSum += esteps

                else:

                    # Dont advance very small acceleration ramps, but sum up the missing advance.
                    for advMove in baseMove.advanceData.decelGroup:

                        advMove.pprint("planAdvanceGroup skipped adv:")

                        # E-steps of non-advanced decel ramp
                        sd = advMove.endRampDistance(
                            advMove.topSpeed.speed().eSpeed, advMove.endSpeed.speed().eSpeed, advMove.decelTime())

                        esteps = sd * self.e_steps_per_mm
                        print "dim E moves %.3f mm in decel phase -> %d steps" % (sd, esteps)

                        advMove.advanceData.endESteps = esteps
                        advMove.advanceData.advStepSum += esteps
                        self.skippedAdvance += advMove.advanceData.sDecel

                self.advSum += baseMove.advanceData.sDecelSum

            print "move %d advSum: " % baseMove.moveNumber, self.advSum
            print "skippedAdvance: ", self.skippedAdvance, self.skippedAdvance*self.e_steps_per_mm

            if debugMoves:
                print 
                print "***** End planAdvanceGroup() *****"

    def planSteps(self, move):

        if debugMoves:
            print "***** Start planSteps() *****"
            move.pprint("PlanSTeps:")

        if (move.advanceData.startSplits + move.advanceData.endSplits) == 0:

            esteps = move.advanceData.estepSum()
            if esteps:
                print "move.esteps: ", move.eSteps, ", esteps:", esteps, move.eSteps-esteps

            # Single move with simple ramps
            # xxx can we use plantravelsteps here
            self.planStepsSimple(move)

            if debugMoves:
                print "***** End planSteps() *****"

            return [move]

        assert(move.displacement_vector_steps_raw3[Z_AXIS] == 0)

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
            newMoves = self.planStepsAdvSA(move) # simple accel

        elif mask == 0x1:

            # Simple advanceed ramp at end
            # Create addtional 'sub-move' at end
            newMoves = self.planStepsAdvSD(move) # simple decel

        elif mask == 0x3:

            # Advanced ramp at end with sign-change
            newMoves = self.planStepsAdvLDD(move)

        elif mask == 0x9:

            # Simple advanceed ramp at start
            # Simple advanceed ramp at end
            newMoves = self.planStepsAdvSALSD(move) # simple accel, linear part, simple decel

        elif mask == 0xb:

            # * advanceed ramp at start
            # * advanceed ramp at end with sign-change
            # * Create three addtional 'sub-moves', on at the start
            #   of the move at two at the end
            newMoves = self.planStepsAdvSALDD(move) # simple accel, linear part, dual deccel

        else:
            print "unhandled mask: 0x%x" % mask
            assert(0)

        # Debug, prüfung ob alle in planAdvance() berechneten e-steps in planSteps() 
        # verwendet werden. Summe ist im idealfall 0, kann aber aufgrund von rundungsfehlern
        # auch ungleich null sein.
        print "estep move.sum: ", move.advanceData.advStepSum
        assert(abs(move.advanceData.advStepSum) < 0.001)

        print "new moves: ", newMoves

        if move.prevMove:
            move.prevMove.nextMove = newMoves[0]
            newMoves[0].prevMove = move.prevMove

        if move.nextMove:
            move.nextMove.prevMove = newMoves[-1]
            newMoves[-1].nextMove = move.nextMove

        for move in newMoves:
            move.sanityCheck()

            if move.crossedDecelStep():
                self.planCrossedDecelSteps(move)
            else:
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

        move.initStepData(StepDataTypeBresenham)

        # abs_displacement_vector_steps = move.absStepsVector()

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

        ######################
        # round e
        e = move.eSteps + self.skippedSimpleSteps
        # esteps = int(e)
        esteps = int(round(e))

        print "eround:", move.eSteps, e, e - esteps

        disp = move.displacement_vector_steps_raw3 + [esteps, 0]

        if disp == emptyVector5:
            print "Empty move..."
            print "***** End PlanSTepsSimple() *****"
            # assert(0)
            self.skippedSimpleSteps += move.eSteps
            return

        self.skippedSimpleSteps = e - esteps
        ######################

        abs_displacement_vector_steps = util.vectorAbs(disp)

        self.moveEsteps -= esteps
        print "planStepsSimple(): moveEsteps-: %7.3f %7.3f" % ( esteps, self.moveEsteps)

        # Determine the 'lead axis' - the axis with the most steps
        leadAxis = move.leadAxis(disp=disp)
        leadAxis_steps = abs_displacement_vector_steps[leadAxis]

        #
        # Init Bresenham's variables
        #
        move.stepData.setBresenhamParameters(leadAxis, abs_displacement_vector_steps)

        dirBits = util.directionBits(disp, self.printer.curDirBits)

        if dirBits != self.printer.curDirBits:
            move.stepData.setDirBits = True
            move.stepData.dirBits = dirBits
            self.printer.curDirBits = dirBits

        steps_per_mm = PrinterProfile.getStepsPerMM(leadAxis)

        #
        # Create a list of stepper pulses
        #

        if leadAxis < A_AXIS:
            nominalSpeed = abs( move.topSpeed.trueSpeed()[leadAxis] ) # [mm/s]
        else:
            nominalSpeed = abs( move.topSpeed.speed().eSpeed )

        if leadAxis < A_AXIS:
            v0 = abs(move.startSpeed.trueSpeed()[leadAxis])                # [mm/s]
        else:
            v0 = abs(move.startSpeed.speed().eSpeed)

        if leadAxis < A_AXIS:
            v1 = abs(move.endSpeed.trueSpeed()[leadAxis])                # [mm/s]
        else:
            v1 = abs(move.endSpeed.speed().eSpeed)

        allowedAccel = abs(move.getMaxAllowedAccelVector5(self.maxAxisAcceleration)[leadAxis])
        print "lead, v0, v1: ", leadAxis, v0, v1

        nAccel = 0
        if move.accelTime():

            accelClocks = util.accelRamp(
                leadAxis,
                v0,
                nominalSpeed,
                allowedAccel,
                leadAxis_steps) # maximum number of steps

            move.stepData.addAccelPulsees(accelClocks)

            nAccel = len(accelClocks)

        nDecel = 0
        if move.decelTime():

            decelClocks = util.decelRamp(
                leadAxis,
                nominalSpeed,
                v1,
                allowedAccel,
                leadAxis_steps) # maximum number of steps

            move.stepData.addDecelPulsees(decelClocks)

            nDecel = len(decelClocks)

        #
        # Linear phase
        #
        nLin = leadAxis_steps - (nAccel + nDecel)
        print "# linear steps:", nLin

        if nLin > 0:

            steps_per_second_nominal = nominalSpeed * steps_per_mm
            timerValue = fTimer / steps_per_second_nominal
            move.stepData.setLinTimer(min(timerValue, 0xffff))

        else:

            if nLin < 0:

                if nAccel and nDecel:

                    # Überschüssige steps im verhältnis von nAccel zu nDecel abschneiden
                    cd = int(-nLin / ((float(nAccel) / nDecel) + 1))
                    ca = -nLin - cd

                    print "ca: ", ca, "cd:", cd

                    if ca:
                        del move.stepData.accelPulses[:ca]

                    if cd:
                        del move.stepData.accelPulses[-cd:]

                    assert(len(move.stepData.accelPulses)+len(move.stepData.deccelPulses) == leadAxis_steps)

                elif nAccel:
                    del move.stepData.accelPulses[:-nLin]
                    assert(len(move.stepData.accelPulses) == leadAxis_steps)
                else:
                    del move.stepData.deccelPulses[nLin:]
                    assert(len(move.stepData.deccelPulses) == leadAxis_steps)

            move.stepData.setLinTimer(0xffff)

        if debugMoves:
            print "# of steps for move: ", leadAxis_steps
            move.pprint("move:")
            print 

        move.stepData.checkLen(leadAxis_steps)

        if debugMoves:
            print "***** End planStepsSimple() *****"

    def planCrossedDecelSteps(self, move):

        if debugMoves:
            print "***** Start planCrossedDecelSteps() *****"
            move.pprint("PlanCrossedDecelSteps:")

        move.state = 3

        move.initStepData(StepDataTypeRaw)

        # Zwei gegenläufige bewegungen bezüglich der beschleunigung. Während
        # XYZ abbremst wird die E achse (negativ) beschleunigt.

        ta = move.accelTime()
        tl = move.linearTime()
        td = move.decelTime()

        topSpeed =  move.topSpeed.speed()
        topSpeedS = topSpeed.feedrate3()
        topSpeedE = topSpeed.eSpeed

        endSpeed =  move.endSpeed.speed()
        endSpeedS = endSpeed.feedrate3()
        endSpeedE = endSpeed.eSpeed

        # xxxxxxxxxxxxxxxxxxxx# xxx funktion trotzdem durchlaufen wegen rundung von e-steps
        # Some tests
        assert(ta == 0)
        assert(tl == 0)
        assert(td > 0)
        assert(topSpeedS > endSpeedS) # XYZ should be decelerating
        assert(abs(topSpeedE) < abs(endSpeedE)) # E should be accelerating

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

        ######################
        # round e
        e = move.eSteps + self.skippedSimpleSteps
        # esteps = int(e)
        esteps = int(round(e))

        print "eround:", move.eSteps, e, e - esteps

        disp = move.displacement_vector_steps_raw3 + [esteps, 0]

        if disp == emptyVector5:
            print "Empty move...", 
            print "***** End planCrossedDecelSteps() *****"
            # assert(0)
            self.skippedSimpleSteps += move.eSteps
            return

        self.skippedSimpleSteps = e - esteps
        ######################

        abs_displacement_vector_steps = util.vectorAbs(disp)

        self.moveEsteps -= esteps
        print "planCrossedDecelSteps(): moveEsteps-: %7.3f %7.3f" % ( esteps, self.moveEsteps)

        dirBits = util.directionBits(disp, self.printer.curDirBits)

        if dirBits != self.printer.curDirBits:
            move.stepData.setDirBits = True
            move.stepData.dirBits = dirBits
            self.printer.curDirBits = dirBits

        leadAxisxy = move.leadAxis(nAxes = 2, disp=disp)
        leadAxis_stepsxy = abs_displacement_vector_steps[leadAxisxy]
        print "lead axisxy is:", leadAxisxy, "lead steps:", leadAxis_stepsxy

        ############################################################################################
        #
        # Create a list of XY-stepper pulses (DDA)
        #
        xyClocks = util.decelRampXY(
            leadAxisxy,
            abs( topSpeed[leadAxisxy] ),
            abs(endSpeed[leadAxisxy]),
            abs(move.getMaxAllowedAccelVector5(self.maxAxisAcceleration)[leadAxisxy]),
            abs_displacement_vector_steps)

        if debugMoves:
            print "Generated %d/%d XY steps" % (len(xyClocks), leadAxis_stepsxy)

        ############################################################################################
        #
        # Steps for E acceleration
        #
        eStepsToMove = abs_displacement_vector_steps[A_AXIS]
        eClocks = util.accelRamp(
                A_AXIS,
                abs( topSpeed.eSpeed ),
                abs( endSpeed.eSpeed ),
                abs(move.getMaxAllowedAccelVector5(self.maxAxisAcceleration)[A_AXIS]),
                eStepsToMove,
                forceFill=False)

        if debugMoves:
            print "Generated %d/%d E steps" % (len(eClocks), eStepsToMove)

        ############################################################################################

        tv75khz = int(fTimer / 75000.0)
        print "Timer value of a 75khz stepper: %d" % tv75khz

        tvsum = 0
        tvIndex = []
        tMap = {}
        for (t, dt, tv, stepBits) in xyClocks:
            tvIndex.append(tvsum)
            tMap[tvsum] = Namespace(t=t, ttv=tvsum, dt=dt, tv=tv, steps=stepBits + [0, 0, 0])
            tvsum += tv

        # print "lead tMap:"
        # pprint.pprint(tMap)

        nMerges = 0
        tvsum = 0
        for (_, _, tv) in eClocks:

            if tvsum in tvIndex:

                # Exact match
                tMap[tvsum].steps[A_AXIS] = 1
                print "merged :", tMap[tvsum]
                nMerges += 1
            else:

                # XXX user faster inserting algo...
                minDist = maxTimerValue16
                bestIndex = -1
                for i in range(len(tvIndex)):
                    tvSearch = tvIndex[i]
                    if abs(tvSearch - tvsum) < minDist:
                        minDist = abs(tvSearch - tvsum)
                        bestIndex = i

                tvBestIndex = tvIndex[bestIndex]

                # print "\nfound index %d for tvsum %d, ttv of found index: %d, distance: %d" % (bestIndex, tvsum, tvBestIndex, minDist)

                if minDist <= tv75khz:

                    tMap[tvBestIndex].steps[A_AXIS] = 1
                    print "merged :", tMap[tvBestIndex]
                    nMerges += 1

                else:

                    stepBits = [0, 0, 0, 1, 0]

                    if tvsum > tvBestIndex:
                        # insert above best match
                        
                        prevStepDesc = tMap[tvBestIndex]
                        prevTv = prevStepDesc.tv

                        nextttv = prevStepDesc.ttv + prevStepDesc.tv
                        # print "prev situation: %d --(%5d)-->                     %6d" % (prevStepDesc.ttv, prevStepDesc.tv, nextttv)

                        if prevTv < minDist:

                            # print "append at end..."

                            # append at end
                            assert(bestIndex == len(tvIndex) -1)

                            if prevTv > minDist:
                                newTv2 = max(prevTv - minDist, tv75khz)
                                newTv1 = prevTv - newTv2
                                tvsum = prevStepDesc.ttv + newTv1
                                # print "anew situation: %d --(%5d)--> %6d --(%5d)--> %d" % (prevStepDesc.ttv, newTv1, tvsum, newTv2, nextttv)
                            else:
                                newTv2 = tv
                                newTv1 = minDist
                                # print "bnew situation: %d --(%5d)--> %6d --(%5d)--> %d" % (prevStepDesc.ttv, newTv1, tvsum, newTv2, nextttv)

                            newStepDesc = Namespace(t=None, ttv=tvsum, dt=None, tv=newTv2, steps=stepBits)
                            tvIndex.append(tvsum)

                        else:

                            # print "insert above index: %d/%d" % (bestIndex, len(tvIndex))
                            assert(prevTv >= 2*tv75khz) # is gap long enough?

                            newTv2 = max(prevTv - minDist, tv75khz)
                            newTv1 = prevTv - newTv2
                            tvsum = prevStepDesc.ttv + newTv1

                            # print "modify prev tv:", prevTv, " --> ", newTv2
                            # print "insert new  tv:", newTv2
                            # print "cnew situation: %d --(%5d)--> %6d --(%5d)--> %d" % (prevStepDesc.ttv, newTv1, tvsum, newTv2, nextttv)

                            newStepDesc = Namespace(t=None, ttv=tvsum, dt=None, tv=newTv2, steps=stepBits)
                            tvIndex.insert(bestIndex+1, tvsum)

                        prevStepDesc.tv = newTv1
                        prevStepDesc.dt = None

                        tMap[tvsum] = newStepDesc

                    else:
                        # insert below best match
                        # print "insert below:", bestIndex

                        prevTvSum = tvIndex[bestIndex-1]
                        prevStepDesc = tMap[prevTvSum]
                        prevTv = prevStepDesc.tv

                        assert(prevTv >= 2*tv75khz) # is gap long enough?

                        nextttv = prevStepDesc.ttv + prevStepDesc.tv

                        newTv1 = prevTv - minDist

                        # print "prev situation: %d --(%5d)-->                     %6d" % (prevStepDesc.ttv, prevStepDesc.tv, nextttv)
                        # print "modify prev tv:", prevTv, " --> ", newTv1
                        # print "insert new  tv:", minDist
                        # print " new situation: %d --(%5d)--> %6d --(%5d)--> %d" % (prevStepDesc.ttv, newTv1, tvsum, minDist, nextttv)

                        prevStepDesc.dt = None
                        prevStepDesc.tv = newTv1

                        newStepDesc = Namespace(t=None, ttv=tvsum, dt=None, tv=minDist, steps=stepBits)
                        tvIndex.insert(bestIndex, tvsum)
                        tMap[tvsum] = newStepDesc

            tvsum += tv

            # Sanity check
            ts = 0
            for t in tvIndex:
                stepDesc = tMap[t]

                util.pdbAssert(stepDesc.ttv == ts)
                util.pdbAssert(stepDesc.tv >= tv75khz)
                ts += stepDesc.tv
            # End sanity check

        # Sanity check
        # print t+stepDesc.tv, tvsum
        # assert(t+stepDesc.tv == tvsum)
        # End sanity check

            # print "other tMap:", otherAxis
            # pprint.pprint(tvIndex)
            # pprint.pprint(tMap)

        # if move.moveNumber == 9713:
            # assert(1)

        assert((leadAxis_stepsxy + eStepsToMove) == (len(tvIndex) + nMerges))

        for ttv in tvIndex:

            stepDesc = tMap[ttv]
            
            move.stepData.addPulse(stepDesc.tv, stepDesc.steps)

        assert(len(tvIndex) == len(move.stepData.pulses))

        if debugMoves:
            move.pprint("planCrossedDecelSteps:")
            print "***** End planCrossedDecelSteps() *****"

        return 

        ############################################################################################
        #
        # Create a list of X-stepper pulses
        #
        xStepsToMove = abs_displacement_vector_steps[X_AXIS]
        xClocks = util.decelRamp(
                X_AXIS,
                abs( topSpeed[X_AXIS] ),
                abs(endSpeed[X_AXIS]),
                abs(move.getMaxAllowedAccelVector5(self.maxAxisAcceleration)[X_AXIS]),
                xStepsToMove,
                forceFill=True)

        if debugMoves:
            print "Generated %d/%d X steps" % (len(xClocks), xStepsToMove)

        ############################################################################################

        ############################################################################################
        #
        # Create a list of Y-stepper pulses
        #
        yStepsToMove = abs_displacement_vector_steps[Y_AXIS]
        yClocks = util.decelRamp(
                Y_AXIS,
                abs( topSpeed[Y_AXIS] ),
                abs(endSpeed[Y_AXIS]),
                abs(move.getMaxAllowedAccelVector5(self.maxAxisAcceleration)[Y_AXIS]),
                yStepsToMove,
                forceFill=True)

        if debugMoves:
            print "Generated %d/%d Y steps" % (len(yClocks), yStepsToMove)

        ############################################################################################

        ############################################################################################
        #
        # Steps for E acceleration
        #
        eStepsToMove = abs_displacement_vector_steps[A_AXIS]
        eClocks = util.accelRamp(
                A_AXIS,
                abs( topSpeed.eSpeed ),
                abs( endSpeed.eSpeed ),
                abs(move.getMaxAllowedAccelVector5(self.maxAxisAcceleration)[A_AXIS]),
                eStepsToMove,
                forceFill=True)

        if debugMoves:
            print "generated %d/%d E steps" % (len(eClocks), eStepsToMove)

        ###########################################################################
        tv75khz = int(fTimer / 75000.0)

        print "Timer value of a 75khz stepper: %d" % tv75khz

        leadAxis = move.leadAxis(disp=disp)
        leadAxis_steps = abs_displacement_vector_steps[leadAxis]

        print "lead axis is:", leadAxis, "lead steps:", leadAxis_steps

        axisClocks = [xClocks, yClocks, None, eClocks]

        if leadAxis == X_AXIS:
            leadClocks = xClocks
            otherAxes = [Y_AXIS, A_AXIS]
        elif leadAxis == Y_AXIS:
            leadClocks = yClocks
            otherAxes = [X_AXIS, A_AXIS]
        elif leadAxis == A_AXIS:
            leadClocks = eClocks
            otherAxes = [X_AXIS, Y_AXIS]
        else:
            assert(0)

        tvsum = 0
        tvIndex = []
        tMap = {}
        for (t, dt, tv) in leadClocks:
            tvIndex.append(tvsum)
            leadSteps = [0, 0, 0, 0, 0]
            leadSteps[leadAxis] = 1
            tMap[tvsum] = Namespace(t=t, dt=dt, tv=tv, steps=leadSteps)
            tvsum += tv

        tMoveLead = leadClocks[-1][0] + leadClocks[-1][1]
        print "move time:", tMoveLead

        dt0Lead = leadClocks[0][1]

        # print "lead tMap:"
        # pprint.pprint(tMap)

        # XXXXXXXXX todo: mit aktueller methode machen aller stepper einen schritt bei t=0 , das sollte evtl
        # besser verteilt werden...
        nMerges = 0
        for otherAxis in otherAxes:

            dimAxis_steps = abs_displacement_vector_steps[otherAxis]

            if not dimAxis_steps:
                continue

            print "merging other axis:", otherAxis
            otherClocks = axisClocks[otherAxis]

            f = float(leadAxis_steps) / dimAxis_steps
            tDelay = (f - 1) * dt0Lead

            assert(tDelay >= 0)

            # tvsum = 0
            tvsum = int(tDelay * fTimer)

            print "dt0Lead: %f, f: %f, tDelay: %f, tvsum %d" % (dt0Lead, f, tDelay, tvsum)

            for (_, _, tv) in otherClocks:

                if tvsum in tvIndex:
                    tMap[tvsum].steps[otherAxis] = 1
                    # print "merged :", tMap[tvsum]
                    nMerges += 1
                else:

                    # XXX user faster inserting algo...
                    minDist = maxTimerValue16
                    bestIndex = -1
                    for i in range(len(tvIndex)):
                        tvSearch = tvIndex[i]
                        if abs(tvSearch - tvsum) < minDist:
                            minDist = abs(tvSearch - tvsum)
                            bestIndex = i

                    tvBestIndex = tvIndex[bestIndex]

                    # print "best index", tvsum, bestIndex, tvBestIndex, minDist

                    if minDist < tv75khz:

                        tMap[tvBestIndex].steps[otherAxis] = 1
                        # print "merged :", tMap[tvBestIndex]
                        nMerges += 1

                    else:

                        stepBits = [0, 0, 0, 0, 0]
                        stepBits[otherAxis] = 1

                        if tvsum > tvBestIndex:
                            # insert above best match
                            # print "insert above:", tvBestIndex, tvsum

                            prevStepDesc = tMap[tvBestIndex]

                            prevTv = prevStepDesc.tv

                            if prevTv < minDist:
                                # append at end
                                assert(bestIndex == len(tvIndex) -1)

                                newTv2 = minDist - prevTv

                                assert(prevTv+newTv2 == minDist)

                                newStepDesc = Namespace(t=None, dt=None, tv=newTv2, steps=stepBits)
                                tvIndex.append(tvsum)

                            else:
                                newTv2 = prevTv - minDist

                                # print "modify prev tv:", prevTv, " --> ", minDist
                                # print "insert new  tv:", newTv2

                                assert(minDist > 0)
                                assert(newTv2 > 0)
                                assert(minDist+newTv2 == prevTv)

                                newStepDesc = Namespace(t=None, dt=None, tv=newTv2, steps=stepBits)
                                tvIndex.insert(bestIndex+1, tvsum)

                            prevStepDesc.tv = minDist
                            prevStepDesc.dt = None

                            tMap[tvsum] = newStepDesc

                        else:
                            # insert below best match
                            # print "insert below:", tvsum, tvBestIndex

                            prevTvSum = tvIndex[bestIndex-1]
                            prevStepDesc = tMap[prevTvSum]
                            prevTv = prevStepDesc.tv
                            newTv1 = prevTv - minDist

                            # print "modify prev tv:", prevTv, " --> ", newTv1
                            # print "insert new  tv:", minDist

                            assert(newTv1 > 0)
                            assert(minDist > 0)
                            assert(newTv1+minDist == prevTv)

                            prevStepDesc.dt = None
                            prevStepDesc.tv = newTv1

                            newStepDesc = Namespace(t=None, dt=None, tv=minDist, steps=stepBits)
                            tvIndex.insert(bestIndex, tvsum)
                            tMap[tvsum] = newStepDesc

                            # print newStepDesc
                            # pprint.pprint(tMap)

                tvsum += tv

            # print "other tMap:", otherAxis
            # pprint.pprint(tvIndex)
            # pprint.pprint(tMap)

        # if move.moveNumber == 9713:
            # assert(1)

        assert((xStepsToMove + yStepsToMove + eStepsToMove) == (len(tvIndex) + nMerges))

        for ttv in tvIndex:

            stepDesc = tMap[ttv]
            
            move.stepData.addPulse(stepDesc.tv, stepDesc.steps)

        assert(len(tvIndex) == len(move.stepData.pulses))

        if debugMoves:
            move.pprint("planCrossedDecelSteps:")
            print "***** End planCrossedDecelSteps() *****"

        return 

    #
    # Single advanceed ramp at start
    # Create addtional 'sub-move' at beginning
    # Das wichtigste ist, die anzahl der steps genau zu treffen, geringe
    # abweichungen was die beschleunigung oder die geschwindigkeiten betrifft,
    # sind ok.
    #
    # Aufteilung nach taccel/tlinear
    #
    def planStepsAdvSA(self, parentMove):

        if debugMoves:
            print "***** Start planStepsAdvSA() *****"
            parentMove.pprint("planStepsAdvSA:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw3

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
        print "dim E moves %f steps while accelerating" % parentMove.advanceData.startESteps
        displacement_vector_steps_A[A_AXIS] = parentMove.advanceData.startESteps
        ####################################################################################

        ####################################################################################
        # PART B, XY

        if tl or td:

            for dim in range(3):
                displacement_vector_steps_B[dim] = displacement_vector_steps_raw[dim] - displacement_vector_steps_A[dim]
       
        if tl:

            print "dim E moves %f steps in linear phase" % parentMove.advanceData.linESteps
            displacement_vector_steps_B[A_AXIS] += parentMove.advanceData.linESteps

        if td:

            print "dim E moves %f steps while decelerating" % parentMove.advanceData.endESteps
            displacement_vector_steps_B[A_AXIS] += parentMove.advanceData.endESteps
        ####################################################################################

        print "new A steps: ", displacement_vector_steps_A
        print "new B steps: ", displacement_vector_steps_B

        from move import SubMove

        moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)

        moveA.setDuration(ta, 0, 0)

        # sv = startSpeed.vv()
        # sv[A_AXIS] = parentMove.advanceData.startEFeedrate()
        sv = parentMove.startSpeed.speed()
        sv.setESpeed(parentMove.advanceData.startEFeedrate())

        # tv = topSpeed.vv()
        # tv[A_AXIS] = parentMove.advanceData.startEReachedFeedrate()
        tv = parentMove.topSpeed.speed()
        tv.setESpeed(parentMove.advanceData.startEReachedFeedrate())

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

                print "planStepsAdvSA: skipping empty b-move" 

        # Sum up additional e-distance of this move for debugging
        parentMove.advanceData.advStepSum -= displacement_vector_steps_A[A_AXIS]+displacement_vector_steps_B[A_AXIS]

        if debugMoves:
            print "***** End planStepsAdvSA() *****"

        assert(util.vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_B[:3]) == displacement_vector_steps_raw[:3])

        return newMoves

    #
    # Single advanceed ramp at end
    # Create addtional 'sub-move' at end
    #
    def planStepsAdvSD(self, parentMove):

        if debugMoves:
            print "***** Start planStepsAdvSD() *****"
            parentMove.pprint("planStepsAdvSD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw3

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
        print "dim E moves %f steps while decelerating" % parentMove.advanceData.endESteps
        displacement_vector_steps_B[A_AXIS] = parentMove.advanceData.endESteps
        ####################################################################################

        ####################################################################################
        if ta or tl:

            # PART A, XY
            for dim in range(3):
                displacement_vector_steps_A[dim] = displacement_vector_steps_raw[dim] - displacement_vector_steps_B[dim]
     
        # PART A, E
        if ta:

            print "dim E moves %f steps while accelerating" % parentMove.advanceData.startESteps
            displacement_vector_steps_A[A_AXIS] += parentMove.advanceData.startESteps

        if tl:

            print "dim E moves %f steps in linear phase" % parentMove.advanceData.linESteps
            displacement_vector_steps_A[A_AXIS] += parentMove.advanceData.linESteps
        ####################################################################################

        print "new A steps: ", displacement_vector_steps_A
        print "new B steps: ", displacement_vector_steps_B

        from move import SubMove

        moveB = SubMove(parentMove, parentMove.moveNumber + 2, displacement_vector_steps_B)
        
        newMoves = [moveB]

        moveB.setDuration(0, 0, td)

        # sv = topSpeed.vv()
        # sv[A_AXIS] = parentMove.advanceData.endEReachedFeedrate()
        sv = parentMove.topSpeed.speed()
        sv.setESpeed(parentMove.advanceData.endEReachedFeedrate())

        # ev = endSpeed.vv()
        # ev[A_AXIS] = parentMove.advanceData.endEFeedrate()
        ev = parentMove.endSpeed.speed()
        ev.setESpeed(parentMove.advanceData.endEFeedrate())

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
        parentMove.advanceData.advStepSum -= esteps

        if debugMoves:
            print "***** End planStepsAdvSD() *****"

        assert(util.vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_B[:3]) == displacement_vector_steps_raw[:3])

        return newMoves

    #
    # Simple advanceed ramp at start and end
    # optional linear middle part
    # Generates 2 or 3 moves
    #
    def planStepsAdvSALSD(self, parentMove):

        if debugMoves:
            print "***** Start planStepsAdvSALSD() *****"
            parentMove.pprint("planStepsAdvSALSD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw3

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
        print "dim E moves %f steps while accelerating" % parentMove.advanceData.startESteps
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
        print "dim E moves %f steps while decelerating" % parentMove.advanceData.endESteps
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
            print "dim E moves %f steps in linear phase" % parentMove.advanceData.linESteps
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

        sv = parentMove.startSpeed.speed()
        sv.setESpeed(parentMove.advanceData.startEFeedrate())
        tv = parentMove.topSpeed.speed()
        tv.setESpeed(parentMove.advanceData.startEReachedFeedrate())
        moveA.setSpeeds(sv, tv, tv)

        sv = parentMove.topSpeed.speed()
        sv.setESpeed(parentMove.advanceData.endEReachedFeedrate())
        ev = parentMove.endSpeed.speed()
        ev.setESpeed(parentMove.advanceData.endEFeedrate())
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
        parentMove.advanceData.advStepSum -= esteps

        if debugMoves:
            print "***** End planStepsAdvSALSD() *****"

        assert(util.vectorAdd(util.vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_B[:3]), displacement_vector_steps_C[:3]) == displacement_vector_steps_raw[:3])

        return newMoves


    #
    # Advanced ramp with sign-change at the end
    # Optional accel/linear part
    # Generates 2 or 3 moves
    def planStepsAdvLDD(self, parentMove):

        if debugMoves:
            print "***** Start planStepsAdvLDD() *****"
            parentMove.pprint("planStepsAdvLDD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw3

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

        stepsUsed = util.vectorAdd(displacement_vector_steps_B[:3], displacement_vector_steps_C[:3])
        stepsMissing = util.vectorSub(displacement_vector_steps_raw[:3], stepsUsed)

        # PART A
        if ta or tl:

            for dim in range(3):
                displacement_vector_steps_A[dim] = displacement_vector_steps_raw[dim] - (displacement_vector_steps_B[dim] + displacement_vector_steps_C[dim])

        else:

            assert(stepsMissing == emptyVector3)
       
        if ta:

            print "dim E moves %f steps while accelerating" % parentMove.advanceData.startESteps
            displacement_vector_steps_A[A_AXIS] += parentMove.advanceData.startESteps

        if tl:

            print "dim E moves %f steps in linear phase" % parentMove.advanceData.linESteps
            displacement_vector_steps_A[A_AXIS] += parentMove.advanceData.linESteps

        ####################################################################################

        print "new A steps: ", displacement_vector_steps_A
        print "new B steps: ", displacement_vector_steps_B
        print "new C steps: ", displacement_vector_steps_C

        from move import SubMove

        moveC = SubMove(parentMove, parentMove.moveNumber + 3, displacement_vector_steps_C)
       
        moveC.setDuration(0, 0, parentMove.advanceData.tdd)

        newMoves = [moveC]

        sv = parentMove.advanceData.crossingSpeed.copy()
        sv.eSpeed = 0
        ev = endSpeed
        ev.setESpeed(parentMove.advanceData.endEFeedrate())
        moveC.setSpeeds(sv, sv, ev)

        # if ta or tl:
        if displacement_vector_steps_A != emptyVector5:

            moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)
            moveA.setDuration(ta, tl, 0)

            moveA.startSpeed.setSpeed(startSpeed)
            moveA.topSpeed.setSpeed(topSpeed)
            moveA.endSpeed.setSpeed(topSpeed)

            # moveA.nextMove = moveB
            # moveB.prevMove = moveA
            # moveB.nextMove = moveC
            # moveC.prevMove = moveB

            newMoves.insert(0, moveA)

        # else:

            # moveB.nextMove = moveC
            # moveC.prevMove = moveB

        if displacement_vector_steps_B != emptyVector5:

            moveB = SubMove(parentMove, parentMove.moveNumber + 2, displacement_vector_steps_B)
            moveB.setDuration(0, 0, parentMove.advanceData.tdc)

            sv = topSpeed
            sv.setESpeed(parentMove.advanceData.endEReachedFeedrate())
            ev = parentMove.advanceData.crossingSpeed.copy()
            ev.eSpeed = 0
            moveB.setSpeeds(sv, sv, ev)

            newMoves.insert(-1, moveB)

        prevMove = None
        nextMove = None
        for move in newMoves:

            if prevMove: 
                prevMove.nextMove = move
            move.prevMove = prevMove
            prevMove = move

        # Sum up additional e-distance of this move for debugging
        esteps = displacement_vector_steps_A[A_AXIS]+displacement_vector_steps_B[A_AXIS]+displacement_vector_steps_C[A_AXIS]
        parentMove.advanceData.advStepSum -= esteps

        if debugMoves:
            print "***** End planStepsAdvLDD() *****"

        assert(util.vectorAdd(util.vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_B[:3]), displacement_vector_steps_C[:3]) == displacement_vector_steps_raw[:3])

        return newMoves

    #
    # Simple advanceed ramp at start and advanced ramp with sign-change at the end
    # Optional linear part
    # Generates 3 or 4 moves
    def planStepsAdvSALDD(self, parentMove):

        if debugMoves:
            print "***** Start planStepsAdvSALDD() *****"
            parentMove.pprint("planStepsAdvSALDD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw3

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

        print "dim E moves %f steps while accelerating" % parentMove.advanceData.startESteps
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
       
            print "dim E moves %f steps in linear phase" % parentMove.advanceData.linESteps
            displacement_vector_steps_B[A_AXIS] += parentMove.advanceData.linESteps

        ####################################################################################

        print "new A steps: ", displacement_vector_steps_A
        print "new B steps: ", displacement_vector_steps_B
        print "new C steps: ", displacement_vector_steps_C
        print "new D steps: ", displacement_vector_steps_D

        from move import SubMove

        moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)
        moveD = SubMove(parentMove, parentMove.moveNumber + 4, displacement_vector_steps_D)
       
        moveA.setDuration(ta, 0, 0)
        moveD.setDuration(0, 0, parentMove.advanceData.tdd)

        # newMoves = [moveA, moveC, moveD]
        newMoves = [moveA]

        sv = parentMove.startSpeed.speed()
        sv.setESpeed(parentMove.advanceData.startEFeedrate())
        tv = parentMove.topSpeed.speed()
        tv.setESpeed(parentMove.advanceData.startEReachedFeedrate())
        moveA.setSpeeds(sv, tv, tv)

        if displacement_vector_steps_C != emptyVector5:

            moveC = SubMove(parentMove, parentMove.moveNumber + 3, displacement_vector_steps_C)
            moveC.setDuration(0, 0, parentMove.advanceData.tdc)

            newMoves.append(moveC)

            sv = parentMove.topSpeed.speed()
            sv.setESpeed(parentMove.advanceData.endEReachedFeedrate())
            ev = parentMove.advanceData.crossingSpeed.copy()
            ev.eSpeed = 0
            moveC.setSpeeds(sv, sv, ev)

        sv = parentMove.advanceData.crossingSpeed.copy()
        sv.setESpeed(0)
        ev = parentMove.endSpeed.speed()
        ev.setESpeed(parentMove.advanceData.endEFeedrate())
        moveD.setSpeeds(sv, sv, ev)

        newMoves.append(moveD)

        if tl:

            moveB = SubMove(parentMove, parentMove.moveNumber + 2, displacement_vector_steps_B)

            moveB.setDuration(0, tl, 0)

            moveB.startSpeed.setSpeed(topSpeed)
            moveB.topSpeed.setSpeed(topSpeed)
            moveB.endSpeed.setSpeed(topSpeed)

            newMoves.insert(1, moveB)

        prevMove = None
        nextMove = None
        for move in newMoves:

            if prevMove: 
                prevMove.nextMove = move
            move.prevMove = prevMove
            prevMove = move

        # Sum up additional e-distance of this move for debugging
        esteps = displacement_vector_steps_A[A_AXIS]+displacement_vector_steps_B[A_AXIS]+displacement_vector_steps_C[A_AXIS]+displacement_vector_steps_D[A_AXIS]
        parentMove.advanceData.advStepSum -= esteps

        if debugMoves:
            print "***** End planStepsAdvSALDD() *****"

        assert(util.vectorAdd(util.vectorAdd(util.vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_B[:3]), displacement_vector_steps_C[:3]), displacement_vector_steps_D[:3]) == displacement_vector_steps_raw[:3])

        return newMoves

    #
    # Simple advanceed ramp at start
    # End ramp with sign-change
    # Submoves: A B CD
    def planStepsAdvSADD(self, parentMove):
        assert(0)

        if debugMoves:
            print "***** Start planStepsAdvSADD() *****"
            parentMove.pprint("planStepsAdvSADD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw

        # PART A, XY
        # Neuen displacement_vector_steps aufbauen:
        displacement_vector_steps_A = [0] * 5
        displacement_vector_steps_B = [0] * 5
        displacement_vector_steps_C = [0] * 5

        ta = parentMove.accelTime()
        td = parentMove.decelTime()

        xllowedAccelV = parentMove.absGetMaxAllowedAccelVector(self.maxAxisAcceleration)

        startSpeedV = parentMove.startSpeed.trueSpeed().vv()

        for dim in [X_AXIS, Y_AXIS]:

            sa = util.accelDist(abs(startSpeedV[dim]), xllowedAccelV[dim], ta)

            steps_per_mm = PrinterProfile.getStepsPerMM(dim)
            steps = int(round(sa * steps_per_mm * util.sign(parentMove.direction[dim])))
            print "dim %d moves %.3f mm while accelerating -> %d steps" % (dim, sa, steps)
            displacement_vector_steps_A[dim] = steps

        # PART A, E
        startSpeedS = parentMove.advanceData.startEFeedrate()
        sa = util.accelDist(abs(startSpeedS), xllowedAccelV[A_AXIS], ta)

        assert(parentMove.direction[A_AXIS] > 0)

        assert(0)
        steps = int(round(sa * ee_steps_per_mm))
        print "dim E moves %.3f mm while accelerating -> %d steps" % (sa, steps)
        displacement_vector_steps_A[A_AXIS] = steps





        # Part B, C
        # Time till the e-velocity crosses zero
        endEReachedFeedrate = parentMove.advanceData.endEReachedFeedrate()
        # endEFeedrate = parentMove.advanceData.endEFeedrate()

        tdc = abs(endEReachedFeedrate) / xllowedAccelV[A_AXIS]
        sdc = util.accelDist(abs(endEReachedFeedrate), xllowedAccelV[A_AXIS], tdc)
        bsteps = int(round(sdc * ee_steps_per_mm))
        print "dim E reaches zero in %.3f s, %.3f mm, %d steps" % (tdc, sdc, bsteps)

        displacement_vector_steps_B[A_AXIS] = bsteps

        # Part B, XY
        reachedSpeedV = parentMove.topSpeed.trueSpeed().vv()

        for dim in [X_AXIS, Y_AXIS]:

            sd = util.accelDist(abs(reachedSpeedV[dim]), xllowedAccelV[dim], tdc)

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
            0,
            0]
        moveB.setSpeeds(sv, sv, ev)

        assert(0)
        assert(0) # nullspeed)
        sv = [
            parentMove.direction[X_AXIS] * zeroCrossingS,
            parentMove.direction[Y_AXIS] * zeroCrossingS,
            parentMove.direction[Z_AXIS] * zeroCrossingS,
            0,
            0]
        ev = parentMove.endSpeed.trueSpeed().vv()
        ev[A_AXIS] = parentMove.advanceData.endEFeedrate()
        moveC.setSpeeds(sv, sv, ev)

        moveA.nextMove = moveB
        moveB.prevMove = moveA
        moveB.nextMove = moveC
        moveC.prevMove = moveB

        if debugMoves:
            print "***** End planStepsAdvSADD() *****"

        assert(util.vectorAdd(util.vectorAdd(displacement_vector_steps_A, displacement_vector_steps_B), displacement_vector_steps_C) == displacement_vector_steps_raw)

        return [moveA, moveB, moveC]


    #
    # Simple advanceed ramp at start
    # Linear middle part
    # End ramp with sign-change
    # Submoves: A B CD
    def planStepsAdvSALD(self, parentMove):
        assert(0)

        if debugMoves:
            print "***** Start planStepsAdvSALD() *****"
            parentMove.pprint("planStepsAdvSALD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw

        # PART A, XY
        # Neuen displacement_vector_steps aufbauen:
        displacement_vector_steps_A = [0] * 5
        displacement_vector_steps_B = [0] * 5
        displacement_vector_steps_C = [0] * 5
        displacement_vector_steps_D = [0] * 5

        ta = parentMove.accelTime()
        td = parentMove.decelTime()

        xllowedAccelV = parentMove.absGetMaxAllowedAccelVector(self.maxAxisAcceleration)

        startSpeedV = parentMove.startSpeed.trueSpeed().vv()

        for dim in [X_AXIS, Y_AXIS]:

            sa = util.accelDist(abs(startSpeedV[dim]), xllowedAccelV[dim], ta)

            steps_per_mm = PrinterProfile.getStepsPerMM(dim)
            steps = int(round(sa * steps_per_mm * util.sign(parentMove.direction[dim])))
            print "dim %d moves %.3f mm while accelerating -> %d steps" % (dim, sa, steps)
            displacement_vector_steps_A[dim] = steps

        # PART A, E
        startSpeedS = parentMove.advanceData.startEFeedrate()
        sa = util.accelDist(abs(startSpeedS), xllowedAccelV[A_AXIS], ta)

        assert(parentMove.direction[A_AXIS] > 0)

        assert(0)
        steps = int(round(sa * ee_steps_per_mm))
        print "dim E moves %.3f mm while accelerating -> %d steps" % (sa, steps)
        displacement_vector_steps_A[A_AXIS] = steps





        # Part C, E
        # Time till the e-velocity crosses zero
        endEReachedFeedrate = parentMove.advanceData.endEReachedFeedrate()
        # endEFeedrate = parentMove.advanceData.endEFeedrate()

        tdc = abs(endEReachedFeedrate) / xllowedAccelV[A_AXIS]
        sdc = util.accelDist(abs(endEReachedFeedrate), xllowedAccelV[A_AXIS], tdc)
        csteps = int(round(sdc * ee_steps_per_mm))
        print "dim E reaches zero in %.3f s, %.3f mm, %d steps" % (tdc, sdc, csteps)

        displacement_vector_steps_C[A_AXIS] = csteps

        # Part C, XY
        reachedSpeedV = parentMove.topSpeed.trueSpeed().vv()

        for dim in [X_AXIS, Y_AXIS]:

            sd = util.accelDist(abs(reachedSpeedV[dim]), xllowedAccelV[dim], tdc)

            steps_per_mm = PrinterProfile.getStepsPerMM(dim)
            steps = int(round(sd * steps_per_mm * util.sign(parentMove.direction[dim])))
            print "dim %d moves %.3f mm in phase C -> %d steps" % (dim, sd, steps)
            displacement_vector_steps_C[dim] = steps
       

        # Part D, E
        # Time of rest of decel ramp
        tdd = td - tdc
        sdd = util.accelDist(abs(endEFeedrate), xllowedAccelV[A_AXIS], tdd)
        dsteps = int(round(sdd * ee_steps_per_mm))
        print "dim E remaining %.3f s, %.3f mm, %d steps" % (tdd, sdd, dsteps)

        displacement_vector_steps_D[A_AXIS] = dsteps

        # Part C, XY
        for dim in [X_AXIS, Y_AXIS]:

            sd = util.accelDist(0, xllowedAccelV[dim], tdd)

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
            0,
            0]
        moveC.setSpeeds(sv, sv, ev)

        assert(0)
        assert(0) # nullspeed)
        sv = [
            parentMove.direction[X_AXIS] * zeroCrossingS,
            parentMove.direction[Y_AXIS] * zeroCrossingS,
            parentMove.direction[Z_AXIS] * zeroCrossingS,
            0,
            0]
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
            print "***** End planStepsAdvSALD() *****"

        assert(util.vectorAdd(util.vectorAdd(util.vectorAdd(displacement_vector_steps_A, displacement_vector_steps_B), displacement_vector_steps_C), displacement_vector_steps_D) == displacement_vector_steps_raw)

        return [moveA, moveB, moveC, moveD]



