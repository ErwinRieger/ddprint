# -*- coding: utf-8 -*-
#
#/*
# This file is part of ddprint - a 3D printer firmware.
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
from ddvector import vectorAdd, vectorSub, vectorLength, vectorAbs

import ddprintutil as util
import math
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

    def __init__(self, nr, prefix=""):

        self.plotfile = "/tmp/%sddplot_%04d.pkl" % (prefix, nr)

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

def eJerk(kAdv, accel):
    return abs(accel) * kAdv

#####################################################################

class Advance (object):

    def __init__(self, planner, args): # , gui=None):

        self.planner = planner
        self.printer = planner.printer

        #
        # Extruder advance, max allowed E-acceleration for the given E-Jerk
        #
        # dvin = dvout + ka * acceleration [ m/s + s * m/s² = m/s ]
        #

        self.__kAdv = args.kadvance
        if self.__kAdv == None:
            self.__kAdv = planner.matProfile.getKAdvI()

        self.startAdvance = None

        # K-Advance is defined in material profile and overridable by the
        # commandline.
        # Gradual advance mode: 
        #   * startadvance (example: 0.1)
        #   * advance increase (example: 0.1)
        #   * stepheight in layers (example: 10)
        if "startAdvance" in args and args.startAdvance != None:

            self.startAdvance = args.startAdvance
            self.advIncrease = args.advIncrease
            self.advStepHeight = args.advStepHeight

            self.setkAdvance(self.startAdvance)

        self.useAutoTemp = args.autotemp

        print("Using kAdvance: %.2f, autotemp is %s" % (self.getKAdv(), self.useAutoTemp))

        self.e_steps_per_mm = planner.printer.printerProfile.getStepsPerMMI(A_AXIS)

        self.maxFeedrateVector = self.printer.printerProfile.getMaxFeedrateVectorI()

    def getKAdv(self):
        return self.__kAdv

    def maxAxisAcceleration(self, useAdvance):

        if self.getKAdv() and useAdvance:

            advJerk = self.planner.getJerk()[A_AXIS]
            maxEAccel = advJerk / self.getKAdv()

            #
            # Limit E-acceleration by DEFAULT_ACCELERATION/DEFAULT_MAX_ACCELERATION
            #
            max_axis_acceleration_noadv = self.printer.printerProfile.getMaxAxisAccelerationI()
            # print "ADV: max E-acceleration:", maxEAccel, ", unlimited accel vector: ", max_axis_acceleration_noadv, " [mm/s²]"
            maxEAccel = min(maxEAccel, max_axis_acceleration_noadv[A_AXIS])

            maxAccel = max_axis_acceleration_noadv[:3] + [maxEAccel, maxEAccel]

            # print "ADV: max E-acceleration, limited acceleration to:", maxEAccel, " [mm/s²]"
            return maxAccel

        else:

            return self.printer.printerProfile.getMaxAxisAccelerationI()

    # Implement gradual advance on layer change (used to determine lin-advance k-value).
    def layerChange(self, layer):

        # self.planner.gui.log("Advance: layer changed: ", layer)

        if self.startAdvance != None:
            self.setkAdvance(self.startAdvance + (layer / self.advStepHeight) * self.advIncrease)
            self.planner.gui.log("Advance: layer changed, new k-adv: ", self.getKAdv())

    def setkAdvance(self, kAdv):

        print("Advance: setting K-Advance to %.3f" % kAdv)
        self.__kAdv = kAdv

    # Called from gcode parser
    def m900(self, values):

        self.setkAdvance(values["K"])

    def reset(self):

        # Reset debug counters
        # Running sum of move esteps, for debugging of planSteps()
        self.moveEsteps = 0.0
        # End debug counters

    def planPath(self, path):

        if debugAdvance:
            print("***** Start planPath() *****")

        self.reset()

        if debugPlot:
            self.pre_plotfile = DebugPlot(path[0].moveNumber, "pre")
            self.plotfile = DebugPlot(path[0].moveNumber)

        if SmoothExtrusionRate:

            # Smooth extrusion rate, compute average extrusion rate of this path and
            # scale the speed of all moves in this path to get an more even flowrate.
            # This decreases the amount of advance ramps.
            rateSum = 0
            dSum = 0
            for move in path:

                d = move.eDistance;
                rateSum += move.topSpeed.speed().eSpeed * d
                dSum += d

            avgRate = rateSum / dSum

            if debugAdvance:
                print("avgRate:", avgRate)

            for move in path:

                v = move.topSpeed.speed()
                f = min(avgRate / v.eSpeed, 10)

                move.startSpeed.setSpeed(v.scale(f), "planPath - Smooth FlowRate")
                move.topSpeed.setSpeed(v.scale(f), "planPath - Smooth FlowRate")
                move.endSpeed.setSpeed(v.scale(f), "planPath - Smooth FlowRate")

        # First move
        v0 = path[0].startSpeed.speed()
        v0.setSpeed(0.0)
        path[0].startSpeed.setSpeed(v0, "planPath - startSpeed")

        # Last move
        lastMove = path[-1]

        v0 = lastMove.endSpeed.speed()
        v0.setSpeed(0.0)
        lastMove.endSpeed.setSpeed(v0, "planPath - endSpeed")

        # Mark last move as end-move, for softstop
        lastMove.isEndMove = True

        prevMove = path[0]

        # Step 1: join moves forward
        for move in path[1:]:
            util.joinMoves(prevMove, move, self)
            prevMove = move


        """
        # Sanity check
        for move in path:
            # move.pprint("sanicheck")
            move.sanityCheck()
        """

        # Step 2: join moves backwards
        self.joinMovesBwd(path)

        """
        # Sanity check
        for move in path:
            move.sanityCheck()
        """

        # Step 3: plan acceleration
        for move in path:
            self.planAcceleration(move)

        #
        # Compute auto hotend temperature
        #
        if self.useAutoTemp:
            self.planner.pathData.doAutoTemp(path)

        # """
        # Sanity check
        for move in path:
            move.sanityCheck()
        # """
        # assert(0)

        # Step 4: handle extruder advance
        if self.getKAdv():

            def processAdvancedMoves(path):

                #debug:
                if debugPlot and debugPlotLevel == "plotLevelPlanned":
                    self.plotPlannedPath(path, self.pre_plotfile)

                self.planAdvance(path)

                # #debug:
                if debugPlot and debugPlotLevel == "plotLevelPlanned":
                    self.plotPlannedPath(path)

            # Aufteilung in segmente, die advance benutzen und die ohne
            currentHasAdvance = path[0].hasAdvance()
            currentPath = []

            for move in path:
                # print "move type: ", move.layerPart

                # Sum up esteps for debugging
                self.moveEsteps += move.eSteps
                # print "moveEsteps+: %7.3f %7.3f" % ( move.eSteps, self.moveEsteps)

                if move.hasAdvance() != currentHasAdvance:
                    assert(0)

                currentPath.append(move)

            if currentPath and currentHasAdvance:
                processAdvancedMoves(currentPath)

            """
            xxx path
            # heavy debug

            plannedEsteps = 0
            roundErrorSum = 0
            plannedEstepsNASum = 0
            for move in path:

                assert(move.advanceData.endESteps==None or (move.advanceData.endEStepsC==None and move.advanceData.endEStepsD==None))

                ta = move.accelTime()
                tl = move.linearTime()
                td = move.decelTime()

                startSpeed =  move.startSpeed.speed()
                topSpeed =  move.topSpeed.speed()
                endSpeed =  move.endSpeed.speed()

                # Step sum of the three phases without advance
                plannedEstepsNA = 0

                if move.advanceData.hasStartAdvance() or move.advanceData.hasEndAdvance():

                    # Sum to check result of advanceData.estepSum()
                    estepSum = move.advanceData.estepSum()
                    plannedEsteps += estepSum

                    if move.advanceData.startESteps:
                        sbase = ((ta * (topSpeed.eSpeed-startSpeed.eSpeed)) / 2 + ta*startSpeed.eSpeed) * self.e_steps_per_mm
                        sadv = (ta * move.advanceData.startFeedrateIncrease) * self.e_steps_per_mm
                        print "sbase, sadv:", sbase, sadv
                        roundError = move.advanceData.startESteps - sbase - sadv
                        print "starte:", move.advanceData.startESteps, sbase+sadv, "err:", roundError
                        assert(util.circaf(roundError, 0, 0.001))

                        roundErrorSum += roundError
                        plannedEstepsNA += sbase
                        estepSum -= sbase + sadv

                    if move.advanceData.linESteps:

                        sbase = tl * topSpeed.eSpeed * self.e_steps_per_mm
                        roundError = move.advanceData.linESteps - sbase
                        print "lin-e:", move.advanceData.linESteps, sbase, "err:", roundError
                        assert(util.circaf(roundError, 0, 0.001))

                        roundErrorSum += roundError
                        plannedEstepsNA += sbase
                        estepSum -= sbase

                    if move.advanceData.endESteps:
                        sbase = ((td * (topSpeed.eSpeed-endSpeed.eSpeed)) / 2 + td*endSpeed.eSpeed) * self.e_steps_per_mm
                        sadv = (td * move.advanceData.endFeedrateIncrease) * self.e_steps_per_mm
                        print "sbase, sadv:", sbase, sadv
                        roundError = move.advanceData.endESteps - sbase - sadv
                        print "ende:", move.advanceData.endESteps, sbase+sadv, "err:", roundError
                        assert(util.circaf(roundError, 0, 0.001))

                        roundErrorSum += roundError
                        plannedEstepsNA += sbase
                        estepSum -= sbase + sadv

                    if move.advanceData.endEStepsC or move.advanceData.endEStepsD:

                        v0 = move.advanceData.endEReachedFeedrate()
                        v1 = move.advanceData.endEFeedrate()
                        slope = (v0 - v1) / td
                        tdc = v0 / slope
                        tdd = td - tdc

                        print "tdc: ", tdc
                        print "tdd: ", tdd

                        sc = (v0 * tdc) / 2
                        endStepsC = sc * self.e_steps_per_mm
                        print "endStepsC: ", move.advanceData.endEStepsC, endStepsC, "err:", move.advanceData.endEStepsC-endStepsC

                        sd = (v1 * tdd) / 2
                        endStepsD = sd * self.e_steps_per_mm
                        print "endStepsD: ", move.advanceData.endEStepsD, endStepsD, "err:", move.advanceData.endEStepsD-endStepsD

                        roundError = move.advanceData.endEStepsC + move.advanceData.endEStepsD - endStepsC - endStepsD
                        assert(util.circaf(roundError, 0, 0.001))

                        sbase = ((td * (topSpeed.eSpeed-endSpeed.eSpeed)) / 2 + td*endSpeed.eSpeed) * self.e_steps_per_mm

                        roundErrorSum += roundError
                        plannedEstepsNA += sbase
                        estepSum -= endStepsC + endStepsD # sbase + endStepsC + endStepsD

                    print "recomputed estepSum:", estepSum
                    assert(util.circaf(estepSum, 0, 0.001)) 

                else: # no advance

                    # if move.advanceData.estepSum() != 0:
                        # move.pprint("no advance but estepSum() != 0")
                        # assert(0)

                    plannedEstepsNA += move.eSteps
                    plannedEsteps += move.eSteps

                print "\nmove steps, nasteps:", move.eSteps, plannedEstepsNA, move.eSteps - plannedEstepsNA
                if not util.circaf(move.eSteps - plannedEstepsNA, 0, 0.001):
                    move.pprint("error move")

                assert(util.circaf(move.eSteps - plannedEstepsNA, 0, 0.001))

                plannedEstepsNASum += plannedEstepsNA

            print "\nesteps %f, plannedsteps %f, diff %f" % (self.moveEsteps, plannedEsteps, self.moveEsteps - plannedEsteps)
            assert(util.circaf( self.moveEsteps - plannedEsteps, 0, 0.001))

            print "NA esteps, plannedsteps, diff:", self.moveEsteps, plannedEstepsNASum, self.moveEsteps-plannedEstepsNASum
            assert(util.circaf( self.moveEsteps-plannedEstepsNASum, 0, 0.001))

            print "roundErrorSum:", roundErrorSum
            assert(roundErrorSum < 0.001)
            # end heavy debug
            """

        #debug if debugPlot and debugPlotLevel == "plotLevelPlanned":
            #debug self.plotPlannedPath(path)

        for move in path:
            if move.eDistance > 1.0 and move.linearTime() > 0.15:
                # print "FRS: e-dist, linear time:", move.eDistance, move.linearTime()
                move.isMeasureMove = True

        newPath = []
        for move in path:

            newMoves = self.planSteps(move)
            newPath += newMoves

        if debugPlot and debugPlotLevel == "plotLevelSplitted":

            for move in newPath:

                # xxx todo move to own function
                self.plotfile.plot1Tick(move.topSpeed.speed().feedrate3(), move.moveNumber)

                at = move.accelTime()
                lt = move.linearTime()
                dt = move.decelTime()

                if at:

                    self.plotfile.plot1Segments(at, (
                        DebugPlotSegment(move.startSpeed.speed().feedrate3(), move.topSpeed.speed().feedrate3(), "green"),
                        ))
                    self.plotfile.plot2Segments(at, (
                        DebugPlotSegment(move.startSpeed.speed().eSpeed, move.topSpeed.speed().eSpeed, "green"),
                        ))

                if lt:

                    self.plotfile.plot1Segments(lt, (
                        DebugPlotSegment(move.topSpeed.speed().feedrate3(), color="blue"),
                        ))
                    self.plotfile.plot2Segments(lt, (
                        DebugPlotSegment(move.topSpeed.speed().eSpeed, color="blue"),
                        ))

                if dt:

                    self.plotfile.plot1Segments(dt, (
                        DebugPlotSegment(move.topSpeed.speed().feedrate3(), move.endSpeed.speed().feedrate3(), "red"),
                        ))
                    self.plotfile.plot2Segments(dt, (
                        DebugPlotSegment(move.topSpeed.speed().eSpeed, move.endSpeed.speed().eSpeed, "red"),
                        ))

                self.plotfile.close()

        # print "rounding remainders:"
        # self.planner.stepRounders.pprint()

        # if self.getKAdv():
            # print "Path moveEsteps:", self.moveEsteps, len(newPath)

        self.planner.stepRounders.check()

        if self.getKAdv():
            assert(abs(self.moveEsteps) < 0.1)

        # Stream moves to printer
        if debugAdvance:
            print("Streaming %d moves..." % len(newPath))

        # Move timeline housekeeping
        # for move in newPath:
            # self.pathData.updateTimeline(move)

        for move in newPath:

            # self.planner.streamMove(move)
            self.planner.pathData.updateHistory(move)

        if debugAdvance:
            print("***** End planPath() *****")

    #
    #
    #
    def joinMovesBwd(self, moves):

        if debugAdvance:
            print("***** Start joinMovesBwd() *****")

        index = len(moves) - 1
        while index >= 0:

            move = moves[index]
            index -= 1

            if debugAdvance: 
                move.pprint("joinMovesBwd")

            # Check, if deceleration between startspeed and endspeed of
            # move is possible with the given acceleration and within the 
            # given distance:

            startSpeed1 = move.startSpeed.speed()
            startSpeed1S = startSpeed1.feedrate3()

            endSpeed1 = move.endSpeed.speed()
            endSpeed1S = endSpeed1.feedrate3()

            allowedAccel3 = move.endAccel.xyAccel()

            maxAllowedStartSpeed = util.vAccelPerDist(endSpeed1S, allowedAccel3, move.distance3)

            # print "joinMovesBwd, startspeed, max startspeed: ", startSpeed1S, maxAllowedStartSpeed

            maxAllowedEStartSpeed = util.vAccelPerDist(endSpeed1.eSpeed, move.endAccel.eAccel(), move.eDistance)

            # print "joinMovesBwd, E-startspeed, max E-startspeed: ", startSpeed1.eSpeed, maxAllowedEStartSpeed

            if maxAllowedStartSpeed >= startSpeed1S and maxAllowedEStartSpeed >= startSpeed1.eSpeed:

                # Join speeds ok
                continue

            if debugAdvance: 
                print("Startspeed of %.5f is to high to reach the desired endspeed of %.5f." % (startSpeed1S, endSpeed1S))
                print("Max. allowed startspeed: %.5f." % maxAllowedStartSpeed)

            # Adjust startspeed of this move:
            startSpeed1.setSpeed(maxAllowedStartSpeed)

            if startSpeed1.eSpeed > maxAllowedEStartSpeed:

                # print "startSpeed1.eSpeed higher than maxAllowedEStartSpeed:", startSpeed1.eSpeed, maxAllowedEStartSpeed

                factor = maxAllowedEStartSpeed / startSpeed1.eSpeed
                # print "factor: ", factor
                assert(factor < 1)

                startSpeed1 = startSpeed1.scale(factor)

                # Check/adjust accleeration
                # a = util.accelPerDist(endSpeedS, startSpeed1.feedrate3(), move.distance3)
                # print "accel old/new:", allowedAccel3, a
                # assert(0)

            move.startSpeed.setSpeed(startSpeed1, "joinMovesBwd - breaking")

            if index >= 0:

                #
                # Adjust endspeed of the previous move, also.
                #
                prevMove = moves[index]

                factor = startSpeed1.feedrate3() / startSpeed1S
                # print "factor: ", factor
                assert(factor < 1)

                # Adjust endspeed of last move:

                # XXX einfacher algo, kann man das besser machen (z.b. mit jerk-berechnung,
                # vector subtraktion oder so?)
                endSpeed0 = prevMove.endSpeed.speed().scale(factor)
                prevMove.endSpeed.setSpeed(endSpeed0, "joinMovesBwd - prevMove breaking")

        if debugAdvance:
            print("***** End joinMovesBwd() *****")

    def planAcceleration(self, move):

        if debugAdvance: 
            print("***** Start planAcceleration() *****")
            move.pprint("Start planAcceleration")

        move.state = 2

        accel3 = move.startAccel.xyAccel()
        # print "allowed XY start accel: ", accel3

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

        if deltaSpeedS > 0.001:
       
            ta = deltaSpeedS / accel3

            sa = util.accelDist(startSpeedS, accel3, ta)
      
            if (sa - move.distance3) > 0.001:
                print(" 0.5 * %f * pow(%f, 2) + %f * %f" % (accel3, ta, startSpeedS, ta))
                print("VStart %f mm/s kann nicht innerhalb von %f mm auf Endgeschwindigkeit %f mm/s gebracht werden!" % (startSpeedS, move.distance3, endSpeedS))
                print("Dafür werden %f mm benötigt" % sa)
                assert(0)

        elif deltaSpeedS < -0.001:
       
            td = abs(deltaSpeedS) / accel3

            sd = util.accelDist(startSpeedS, -accel3, td)
      
            if (sd - move.distance3) > 0.001:
                print(" 0.5 * %f * pow(%f, 2) + %f * %f" % (accel3, td, startSpeedS, td))
                print("VStart %f mm/s kann nicht innerhalb von %f mm auf Endgeschwindigkeit %f mm/s gebracht werden!" % (startSpeedS, move.distance3, endSpeedS))
                print("Dafür werden %f mm benötigt" % sd)
                assert(0)

        deltaESpeed = endSpeed.eSpeed - startSpeed.eSpeed

        eAccel = move.startAccel.eAccel()

        if deltaESpeed > 0.001:
       
            ta = deltaESpeed / eAccel

            sa = util.accelDist(startSpeed.eSpeed, eAccel, ta)
      
            if (sa - move.eDistance) > 0.001:
                # print " 0.5 * %f * pow(%f, 2) + %f * %f" % (allowedAccel, ta, startSpeedS, ta)
                # print "VStart %f mm/s kann nicht innerhalb von %f mm auf Endgeschwindigkeit %f mm/s gebracht werden!" % (startSpeedS, move.distance3, endSpeedS)
                # print "Dafür werden %f mm benötigt" % sa
                assert(0)

        elif deltaESpeed < -0.001:
       
            td = abs(deltaESpeed) / eAccel

            sd = util.accelDist(startSpeed.eSpeed, -eAccel, td)
      
            if (sd - move.eDistance) > 0.001:
                print("VStartE %f mm/s kann nicht innerhalb von %f mm auf Endgeschwindigkeit %f mm/s gebracht werden!" % (startSpeed.eSpeed, move.eDistance, endSpeed.eSpeed))
                print("Dafür werden %f mm benötigt" % sd)
                assert(0)


        #
        # Compute distance to accel from start speed to nominal speed:
        ta = 0.0
        sa = 0.0

        deltaStartSpeedS = topSpeed.feedrate3() - startSpeedS

        maxAccel = self.maxAxisAcceleration(move.hasAdvance())

        if deltaStartSpeedS > AccelThreshold:

            ta = deltaStartSpeedS / accel3
            # print "XY accel time (for %f mm/s): %f [s]" % (deltaStartSpeedS, ta)

            # debug Check axxis acceleration
            deltaSpeedV = move.direction3.scale(deltaStartSpeedS)
            for dim in range(3):
                dimAccel = abs(deltaSpeedV[dim]) / ta
                # print "dimaccel ", dim, dimAccel
                if (dimAccel / maxAccel[dim]) > 1.001:
                    print("dim %d verletzt max accel: " % dim, dimAccel, " > ", maxAccel[dim])
                    assert(0)

            # Check acceleration of e-axis
            eAccel = (topSpeed.eSpeed - startSpeed.eSpeed) / ta
            # print "eaccel: ", eAccel, move.startAccel.eAccel()
            assert((eAccel / move.startAccel.eAccel()) < 1.001)
            #end debug

            sa = util.accelDist(startSpeedS, accel3, ta)

        #
        # Compute distance to decel from nominal speed to endspeed:
        #
        tb = 0.0
        sb = 0.0

        deltaEndSpeedS = topSpeed.feedrate3() - endSpeedS                          # [mm/s]

        if deltaEndSpeedS > AccelThreshold:

            tb = deltaEndSpeedS / accel3                          # [s]
            # print "XY decel time (for %f mm/s): %f [s]" % (deltaEndSpeedS, tb)

            # debug Check axxis acceleration
            deltaSpeedV = move.direction3.scale(deltaEndSpeedS)
            for dim in range(3):
                dimDecel = abs(deltaSpeedV[dim]) / tb  
                if (dimDecel / maxAccel[dim]) > 1.001:
                    print("dim %d verletzt max accel: " % dim, dimDecel, " [mm/s] > ", maxAccel[dim], " [mm/s]")
                    assert(0)

            # Check acceleration of e-axis
            eAccel = (topSpeed.eSpeed - endSpeed.eSpeed) / tb
            # print "edecel: ", eAccel, move.accel.eAccel()
            assert((eAccel / move.endAccel.eAccel()) < 1.001)
            # end debug

            sb = util.accelDist(endSpeedS, accel3, tb)

        # print "e_distance: %f, sbeschl, sbrems: %f, %f" % (move.distance3, sa, sb)

        if (sa+sb) and (move.distance3 < (sa+sb)):

            #
            # Strecke zu kurz, Trapez nicht möglich, geschwindigkeit muss abgesenkt werden.
            #
            if debugAdvance:
                print("Trapez nicht möglich: s: %f, sbeschl (%f) + sbrems (%f) = %f" % (move.distance3, sa, sb, sa+sb))

            # float vsquared = (acceleration * totalDistance) + 0.5 * (fsquare(startSpeed) + fsquare(endSpeed));
            vReachedSquared = (accel3 * move.distance3) + (pow(startSpeedS, 2) + pow(endSpeedS, 2))/2;
            # vReachedSquared = (2*startAccel3*endAccel3*move.distance3 + startAccel3*pow(startSpeedS, 2) + endAccel3*pow(endSpeedS, 2) ) / (startAccel3 + endAccel3)

            assert(vReachedSquared >= 0)
            vReached = math.sqrt(vReachedSquared)

            vReached = math.sqrt(vReachedSquared)

            # print "Vreached:", vReachedSquared, vReached

            # assert(util.circaf(vReached, startSpeedS, 0.001))
            # assert(util.circaf(vReached, endSpeedS, 0.001))

            sa = max((vReachedSquared - pow(startSpeedS, 2))/(2.0 * accel3), 0.0)

            # print "sa:", sa

            # Handle very small acceleration distances
            if sa < 0.000001:

                # Zero acceleration, only deceleration, topspeed is startspeed

                # Time to do deceleration
                tb = util.timePerDist(endSpeedS, startSpeedS, move.distance3)

                # Check used acceleration
                usedAccel3 = -deltaSpeedS / tb
                # print "allowedAccel3, usedAccel3:", accel3, usedAccel3, accel3-usedAccel3
                
                usedEAccel = -deltaESpeed / tb
                # print "eAccel, usedEAccel:", eAccel, usedEAccel, eAccel-usedEAccel

                if usedAccel3 != accel3 and usedEAccel != eAccel:
                    move.startAccel.setAccel(usedAccel3, usedEAccel)

                topSpeed.setSpeed(startSpeedS)
                move.topSpeed.setSpeed(topSpeed, "planAcceleration - max reachable speed")

                if debugAdvance:
                    print("reachable topspeed: ", topSpeed)

                move.setDuration(0, 0, tb)

                if debugAdvance:
                    move.pprint("End planAcceleration")
                    print() 
                    print("***** End planAcceleration() *****")
                
                return

            sb = move.distance3 - sa

            # Handle very small deceleration distances
            if sb < 0.000001:

                # Zero deceleration, only acceleration, topspeed is endspeed

                # Time to do acceleration
                ta = util.timePerDist(startSpeedS, endSpeedS, move.distance3)

                # Check used acceleration
                usedAccel3 = deltaSpeedS / ta
                # print "accel3, usedAccel3:", accel3, usedAccel3, accel3-usedAccel3

                usedEAccel = deltaESpeed / ta
                # print "eAccel, usedEAccel:", eAccel, usedEAccel, eAccel-usedEAccel

                if usedAccel3 != accel3 and usedEAccel != eAccel:
                    move.endAccel.setAccel(usedAccel3, usedEAccel)

                topSpeed.setSpeed(endSpeedS)
                move.topSpeed.setSpeed(topSpeed, "planAcceleration - max reachable speed")

                if debugAdvance:
                    print("reachable topspeed: ", topSpeed)

                move.setDuration(ta, 0, 0)

                if debugAdvance:
                    move.pprint("End planAcceleration")
                    print() 
                    print("***** End planAcceleration() *****")
               
                return

            # 
            # Geschwindigkeit, die auf strecke sa erreicht werden kann
            # 
            v = math.sqrt ( 2 * accel3 * sa + pow(startSpeedS, 2) )

            assert(v > startSpeedS) # v = max(v, startSpeedS)
            assert(v > endSpeedS) # v = max(v, endSpeedS)

            # debug, test
            if sb:

                v2 = math.sqrt ( 2 * accel3 * sb + pow(endSpeedS, 2) )
                # print "move.feedrate neu: %f (test: %f, diff: %f)" % (v, v2, abs(v - v2))

                assert( abs(v - v2) < 0.001)
            # end debug

            topSpeed.setSpeed(v)

            if debugAdvance:
                print("sbeschl, sbrems neu: %f, %f" % (sa, sb), ", reachable topspeed: ", topSpeed)

            move.topSpeed.setSpeed(topSpeed, "planAcceleration - max reachable speed")

            deltaSpeedS = v - startSpeedS                          # [mm/s]
            ta = deltaSpeedS / accel3
            # print "ta: ", ta, deltaSpeedS

            deltaSpeedS = v - endSpeedS                          # [mm/s]
            tb = deltaSpeedS / accel3
            # print "tb: ", tb, deltaSpeedS

            assert(ta > 0)
            assert(tb > 0)

            move.setDuration(ta, 0, tb)

            if debugAdvance:
                move.pprint("End planAcceleration")
                print() 
                print("***** End planAcceleration() *****")
                
            return

        # 
        # Strecke reicht aus, um auf nominal speed zu beschleunigen
        # 

        # print "ta: ", ta, deltaStartSpeedS, sa
        # print "tb: ", tb, deltaEndSpeedS, sb

        nominalSpeed = move.topSpeed.speed().feedrate3() # [mm/s]
        slin = move.distance3 - (sa+sb)
        tlin = slin / nominalSpeed
        # print "tlin: ", tlin, slin
        move.setDuration(ta, tlin, tb)

        if debugAdvance:
            move.pprint("End planAcceleration")
            print() 
            print("***** End planAcceleration() *****")
            # if move.moveNumber == 40: assert(0)

    ################################################################################

    def planAdvance(self, path):

        if debugAdvance:
            print("***** Start planAdvance() *****")
            print("len path:", len(path))
            # move.pprint("planAdvance:")

        values = []
        revValues = []

        ewma = util.EWMA(0.25)
        fwd = []

        t = 0
        # f = open("adv_%d.dat" % path[0].moveNumber, "w")

        diff = 0.0 # To compute rounding errors
        posAdv = 0.0 # To compute relative rounding error
        
        for move in path:

            # Compute area (= amount of e-advance) of the accel- and decel-ramps:
            startFeedrateIncrease = eJerk(self.getKAdv(), move.startAccel.eAccel())
            sadv = move.startAdvSteps(startFeedrateIncrease=startFeedrateIncrease)

            if sadv != 0.0:

                diff += sadv
                posAdv += sadv
                if util.isclose(sadv, 0, 1e-16):
                    print("sadv:", sadv)
                    assert(0)

            move.advanceData.sAccel = sadv

            # if sadv:
                # print "s: startFeedrateIncrease, sadv", startFeedrateIncrease, sadv

            endFeedrateIncrease = - eJerk(self.getKAdv(), move.endAccel.eAccel())
            sdec = move.endAdvSteps(endFeedrateIncrease=endFeedrateIncrease)

            if sdec != 0.0:
               
                diff += sdec
                posAdv -= sdec
                if util.isclose(sdec, 0, 2e-9):
                    print("sadv:", sdec)
                    assert(0)

            move.advanceData.sDecel = sdec

            # if sadv:
                # print "s: endFeedrateIncrease, sdec", endFeedrateIncrease, sdec

            tmove = move.accelTime() + move.linearTime() + move.decelTime()
            tmid = t + tmove / 2

            v = sadv - sdec
            values.append((move, tmid, v))

            t += tmove

            ewma.add(v)
            fwd.append(ewma.value())

            revValues.insert(0, v)

            # move.pprint("planAdvance step 1:")

        # print "diff:", abs(diff), abs(s/1000)
        assert(abs(diff) < (abs(posAdv)/1000))

        ewma = util.EWMA(0.25)
        bwd = []
        for v in revValues:
            ewma.add(v)
            bwd.append(ewma.value())

        bwd.reverse()

        avg = []

        for i in range(len(values)):

            a = (fwd[i]+bwd[i])/2
            avg.append(a)

            # f.write("%f %f %f %f %f\n" % (values[i][1], values[i][2], fwd[i], bwd[i], a))

        #----------------------------------------
        splitRanges = []
        splitRange = None
        nonSkipRange = None

        for i in range(len(path)):

            if avg[i] < AdvanceMinRamp:

                if nonSkipRange:
                    splitRanges.append(nonSkipRange)
                nonSkipRange = None

                if splitRange:
                    splitRange["index"].append(i)
                else:
                    splitRange = { "skip": True, "index": [i], "sum": 0.0 }

                move = path[i]
                if move.advanceData.sAccel:
                    splitRange['sum'] += move.advanceData.sAccel
                if move.advanceData.sDecel:
                    splitRange['sum'] += move.advanceData.sDecel

            else:

                if splitRange:
                    splitRanges.append(splitRange)
                splitRange = None

                if nonSkipRange:
                    nonSkipRange["index"].append(i)
                else:
                    nonSkipRange = { "skip": False, "index": [i], "sum": 0.0 }

                move = path[i]
                if move.advanceData.sAccel:
                    nonSkipRange['sum'] += move.advanceData.sAccel
                if move.advanceData.sDecel:
                    nonSkipRange['sum'] += move.advanceData.sDecel

        if splitRange:
            splitRanges.append(splitRange)
        if nonSkipRange:
            splitRanges.append(nonSkipRange)

        if debugAdvance:
            print("split Ranges:", len(splitRanges)) # , "# unskipped:", nonSkip, "# skipped:", len(path)-nonSkip
            for splitRange in splitRanges:
                print("Range:", splitRange)

        if debugAdvance:

            l = 0
            for splitRange in splitRanges:
                l += len(splitRange["index"])
            assert(len(path) == l)

        #----------------------------------------

        advSum = 0
        advance = 0

        for splitRange in splitRanges:

            # Amount of advance ([mm]?) to apply for this range
            # Note: we apply rounding differences from above (diff) into the first run.

            if splitRange["skip"]:

                toAdvance = splitRange["sum"] + diff
                diff = 0

                # Benutze moves aus dem skipped range um das fehlende
                # advance abzuarbeiten.

                if debugAdvance:
                    print("Skip range: [%d:%d]" % (splitRange["index"][0], splitRange["index"][-1]))
                    print("advance at start of splitRange:", advance, ", advance of this range:", toAdvance)

                # print "corr adv:", toAdvance
                advance += toAdvance

                if debugAdvance:
                    print("advance at end of splitRange:", advance)

                if toAdvance > 0:

                    index = 0
                    while toAdvance > 0 and index < len(splitRange["index"]):

                        skippedMove = path[splitRange["index"][index]]

                        adv = min(skippedMove.advanceData.sAccel, toAdvance)

                        if adv:

                            if debugAdvance:
                                print("Move %d, corr. adv:" % skippedMove.moveNumber, adv)

                            self.computeAccelAdvance(skippedMove, adv)
                            advSum += adv

                            self.computeConstSteps(skippedMove)

                            #---------------------------------------------------------------------------------------------
                            # E-steps of non-advanced decel ramp
                            sd = skippedMove.endRampDistance(
                                skippedMove.topSpeed.speed().eSpeed, skippedMove.endSpeed.speed().eSpeed, skippedMove.decelTime())

                            esteps = sd * self.e_steps_per_mm
                            # print "dim E moves %.3f mm in decel phase -> %d steps" % (sd, esteps)

                            skippedMove.advanceData.endESteps = esteps
                            skippedMove.advanceData.advStepSum += esteps
                            #---------------------------------------------------------------------------------------------

                            toAdvance -= adv

                        index += 1

                        # print "Advance left:", toAdvance

                elif toAdvance < 0:

                    index = 0
                    while toAdvance < 0 and index < len(splitRange["index"]):

                        skippedMove = path[splitRange["index"][index]]

                        adv = max(skippedMove.advanceData.sDecel, toAdvance)

                        if adv:

                            if debugAdvance:
                                print("Move %d, corr. adv:" % skippedMove.moveNumber, adv)

                            self.computeDecelAdvance(skippedMove, adv)
                            advSum += adv

                            self.computeConstSteps(skippedMove)

                            #---------------------------------------------------------------------------------------------
                            # E-steps of non-advanced accel ramp
                            sa = skippedMove.startRampDistance(
                                skippedMove.startSpeed.speed().eSpeed, skippedMove.topSpeed.speed().eSpeed, skippedMove.accelTime())

                            esteps = sa * self.e_steps_per_mm
                            # print "dim E moves %.3f mm in acel phase -> %d steps" % (sa, esteps)

                            skippedMove.advanceData.startESteps = esteps
                            skippedMove.advanceData.advStepSum += esteps
                            #---------------------------------------------------------------------------------------------

                            toAdvance -= adv

                        index += 1

            else: # if splitRange["skip"]:

                toAdvance = splitRange["sum"]

                if debugAdvance:
                    print("Non-Skip range: [%d:%d]" % (splitRange["index"][0], splitRange["index"][-1]))
                    print("advance at start of non-skip range:", advance, ", advance of this range:", toAdvance)

                advance += toAdvance

                if debugAdvance:
                    print("advance at end of non-splitRange:", advance)

                # plan advance steps
                for i in splitRange["index"]:

                    advMove = path[i]

                    accelAdvance = advMove.advanceData.sAccel + diff
                    diff = 0
                    assert(accelAdvance >= 0)
                    decelAdvance = advMove.advanceData.sDecel

                    if accelAdvance:
                        self.computeAccelAdvance(advMove, accelAdvance)
                        advSum += accelAdvance
                        posAdv += accelAdvance

                    if decelAdvance:
                        self.computeDecelAdvance(advMove, decelAdvance)
                        advSum += decelAdvance

                    if accelAdvance or decelAdvance:
                        self.computeConstSteps(advMove)


        if advSum:
            if not util.isclose(advSum, 0):
                print("s: advSum (should be 0): %f, f: %f, %f ppm" % ( advSum, advSum/posAdv, (advSum/(posAdv/1000000.0))))
            assert(advSum < posAdv/1000.0)

        if advance:
            if not util.isclose(advance, 0):
                print("s: advance (should be 0): %f, f: %f, %f ppm" %( advance, advSum/posAdv, (advance/(posAdv/1000000.0))))
            assert(advance < posAdv/1000.0)

        if debugAdvance:
            print() 
            print("***** End planAdvance() *****")

    def computeAccelAdvance(self, advMove, accelAdvance):

        # Aparallel = g * h; g = fri; h = ta
        # Sadv = fri * ta -> fri = Sadv / ta 
        ta = advMove.accelTime()

        if not ta:
            return None

        startFeedrateIncrease = accelAdvance / ta

        # print "startFeedrateIncrease: ", startFeedrateIncrease

        advMove.advanceData.startFeedrateIncrease = startFeedrateIncrease

        # advMove.pprint("computeAccelAdvance accel adv:")

        esteps = advMove.startERampSteps()

        advMove.advanceData.startESteps = esteps
        advMove.advanceData.advStepSum += esteps

        advMove.advanceData.startSplits = 1

        assert(not advMove.advanceData.startSignChange()) # debug

    def computeDecelAdvance(self, advMove, decelAdvance):

        # Aparallel = g * h; g = fri; h = td
        # Sadv = fri * td -> fri = Sadv / td 
        td = advMove.decelTime()

        if not td:
            return None

        endFeedrateIncrease = decelAdvance / td

        # print "endFeedrateIncrease: ", endFeedrateIncrease

        advMove.advanceData.endFeedrateIncrease = endFeedrateIncrease

        # advMove.pprint("computeDecelAdvance decel adv:")

        advMove.advanceData.endSplits = 1

        if advMove.advanceData.endSignChange(): 

            # advMove.pprint("zero cross move")

            ###############################################################
            # Compute additional data for planSteps()
            # Time till the e-velocity crosses zero

            topSpeed = advMove.topSpeed.speed()
            endSpeed = advMove.endSpeed.speed()

            v0 = advMove.advanceData.endEReachedFeedrate()

            # print "v0, accel, endFeedrateIncrease: ", v0, advMove.endAccel.eAccel(), advMove.advanceData.endFeedrateIncrease
            tdc = min(v0 / advMove.endAccel.eAccel(), td)

            # print "Time to reach zero-crossing tdc:", tdc, ", tdd: ", td - tdc
            # print "tdc: %f, td: %f" % (tdc, td)
            assert(tdc >= 0 and ((tdc < td) or util.isclose(tdc,td)))

            # Nominal speed at zero crossing
            if topSpeed.feedrate3() > endSpeed.feedrate3():
                # decel
                zeroCrossingS = util.vAccelPerTime(topSpeed.feedrate3(), - advMove.endAccel.xyAccel(), tdc)
            else:
                assert(0) # does this happen?

            # XYZ speed at E zerocrossing
            crossingSpeed = advMove.topSpeed.speed()
            crossingSpeed.setSpeed(zeroCrossingS)

            # print "crossingspeed: ", crossingSpeed

            # PART C, D
            estepsc = advMove.endRampTriangle(v0 = v0, v1 = 0, dt = tdc) * self.e_steps_per_mm

            # print "(sdc, estepsc):", (sdc, estepsc) 

            advMove.advanceData.endEStepsC = estepsc
            advMove.advanceData.advStepSum += estepsc

            # print "top, zerocrossspeed:", topSpeed.feedrate3(), crossingSpeed

            advMove.advanceData.tdc = tdc
            advMove.advanceData.tdd = td - tdc
            advMove.advanceData.crossingSpeed = crossingSpeed

            v1 = advMove.advanceData.endEFeedrate()

            # print "vo, v1:", v0, v1
            assert(not util.isclose(0, v1))
            
            estepsd = advMove.startRampTriangle(
                    v0 = 0,
                    v1 = v1,
                    dt = advMove.advanceData.tdd)  * self.e_steps_per_mm

            # print "(sdd, estepsd):", (sdd, estepsd) 

            advMove.advanceData.endEStepsD = estepsd
            advMove.advanceData.advStepSum += estepsd

            advMove.advanceData.endSplits += 1 # Mark as crossing decel ramp

            ###############################################################

        else:

            esteps = advMove.endERampSteps()

            advMove.advanceData.endESteps = esteps
            advMove.advanceData.advStepSum += esteps

    # Compute E-steps of linear phase
    def computeConstSteps(self, advMove):

        tl = advMove.linearTime()

        # advMove.pprint("computeConstSteps linear phase:")

        # E-distance ist nicht einfach der rest der e-steps, linearer e-anteil muss über
        # die dauer des linearen anteils (tLinear) berechnet werden.
        sl = tl * advMove.topSpeed.speed().eSpeed
        esteps = sl * self.e_steps_per_mm
        # print "dim E moves %.3f mm in linear phase -> %d steps" % (sl, esteps)

        advMove.advanceData.linESteps = esteps
        advMove.advanceData.advStepSum += esteps

    def planSteps(self, move):

        if debugAdvance:
            print("***** Start planSteps() *****")
            move.pprint("PlanSTeps:")

        if (move.advanceData.startSplits + move.advanceData.endSplits) == 0:

            move.sanityCheck()

            esteps = move.advanceData.estepSum()
            # if esteps:
                # print "move.esteps: ", move.eSteps, ", esteps:", esteps, move.eSteps-esteps

            # Single move with simple ramps
            # xxx can we use plantravelsteps here
            if self.planStepsSimple(move):

                move.isStartMove = True

                if debugAdvance:
                    print("***** End planSteps() *****")

                return [move]

            if debugAdvance:
                print("***** End planSteps() *****")

            return []

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
            newMoves = self.planStepsAdvSALDD(move) # simple accel, linear part, dual decel

        else:
            print("unhandled mask: 0x%x" % mask)
            assert(0)

        # Debug, prüfung ob alle in planAdvance() berechneten e-steps in planSteps() 
        # verwendet werden. Summe ist im idealfall 0, kann aber aufgrund von rundungsfehlern
        # auch ungleich null sein.
        # print "estep move.sum: ", move.advanceData.advStepSum
        assert(abs(move.advanceData.advStepSum) < 0.001)

        # print "new moves: ", newMoves

        startMove = True
        subMoves = []
        for newMove in newMoves:
            newMove.sanityCheck()

            v_1 = newMove.topSpeed.speed().feedrate3()
            v_2 = newMove.endSpeed.speed().feedrate3()
            xyzSign = util.sign(abs(v_2) - abs(v_1))

            # print "\nisCrossedDecelStep(): v_1: %f, v_2: %f\n" % (v_1, v_2), xyzSign

            ve_1 = newMove.topSpeed.speed().eSpeed
            ve_2 = newMove.endSpeed.speed().eSpeed
            eSign = util.sign(abs(ve_2) - abs(ve_1))

            # print "isCrossedDecelStep(): ve_1: %f, ve_2: %f\n" % (ve_1, ve_2), eSign

            if xyzSign == eSign: # no crossed move
                if self.planStepsSimple(newMove):
                    newMove.isStartMove = startMove
                    newMove.isMeasureMove = move.isMeasureMove
                    startMove = False
                    subMoves.append(newMove)
            else: # crossed deceleration step
                if self.planCrossedDecelSteps(newMove):
                    newMove.isStartMove = startMove
                    newMove.isMeasureMove = move.isMeasureMove
                    startMove = False
                    subMoves.append(newMove)

        if debugAdvance:
            print("***** End planSteps() *****")

        return subMoves
    
    # xxx use planner.planTravelMove here
    def planStepsSimple(self, move):

        if debugAdvance:
            print("***** Start planStepsSimple() *****")
            move.pprint("PlanSTepsSimple:")

        move.state = 3

        move.initStepData(StepDataTypeBresenham)

        # Round step values
        dispF = move.displacement_vector_steps_raw3 + [move.eSteps, 0.0]
        dispS = self.planner.stepRounders.round(dispF)

        self.moveEsteps -= move.eSteps
        # print "planStepsSimple(): moveEsteps-: %7.3f %7.3f" % (move.eSteps, dispS[A_AXIS])

        if dispS == emptyVector5:

            if debugAdvance:
                print("Empty move...")
                print("***** End PlanSTepsSimple() *****")

            self.planner.stepRounders.rollback()
            return False

        self.planner.stepRounders.commit()

        abs_displacement_vector_steps = vectorAbs(dispS)

        # Determine the 'lead axis' - the axis with the most steps
        leadAxis = move.leadAxis(disp=dispS)
        leadAxis_steps = abs_displacement_vector_steps[leadAxis]

        #
        # Init Bresenham's variables
        #
        move.stepData.setBresenhamParameters(leadAxis, abs_displacement_vector_steps)

        dirBits = util.directionBits(dispS)

        if dirBits != self.planner.curDirBits:
            move.stepData.setDirBits = True
            move.stepData.dirBits = dirBits
            self.planner.curDirBits = dirBits

        steps_per_mm = self.printer.printerProfile.getStepsPerMMI(leadAxis)

        #
        # Create a list of stepper pulses
        #
        if leadAxis < A_AXIS:
            nominalSpeed = abs( move.topSpeed.speed()[leadAxis] ) # [mm/s]
        else:
            nominalSpeed = abs( move.topSpeed.speed().eSpeed )

        if leadAxis < A_AXIS:
            v0 = abs(move.startSpeed.speed()[leadAxis])                # [mm/s]
        else:
            v0 = abs(move.startSpeed.speed().eSpeed)

        if leadAxis < A_AXIS:
            v1 = abs(move.endSpeed.speed()[leadAxis])                # [mm/s]
        else:
            v1 = abs(move.endSpeed.speed().eSpeed)

        steps_per_second_nominal = nominalSpeed * steps_per_mm
        linearTimerValue = self.planner.timerLimit(int(fTimer / steps_per_second_nominal))

        nAccel = 0
        if move.accelTime():

            accelClocks = self.planner.accelRamp(
                steps_per_mm,
                v0,
                nominalSpeed,
                abs(move.startAccel.accel(leadAxis)),
                leadAxis_steps,
                linearTimerValue)

            move.stepData.setAccelPulses(accelClocks)
            nAccel = len(accelClocks)

        nDecel = 0
        if move.decelTime():

            decelClocks = self.planner.decelRamp(
                steps_per_mm,
                nominalSpeed,
                v1,
                abs(move.endAccel.accel(leadAxis)),
                leadAxis_steps,
                linearTimerValue)

            move.stepData.setDecelPulses(decelClocks)
            nDecel = len(decelClocks)

        #
        # Linear phase
        #
        nLin = leadAxis_steps - (nAccel + nDecel)
        # print "# linear steps:", nLin

        if nLin > 0:

            move.stepData.setLinTimer(linearTimerValue)

        else:

            if nLin < 0:

                if nAccel and nDecel:

                    # Überschüssige steps im verhältnis von nAccel zu nDecel abschneiden
                    cd = int(-nLin / ((float(nAccel) / nDecel) + 1))
                    ca = -nLin - cd

                    if ca:
                        del move.stepData.accelPulses[:ca]

                    if cd:
                        del move.stepData.accelPulses[-cd:]

                    assert(len(move.stepData.accelPulses)+len(move.stepData.decelPulses) == leadAxis_steps)

                elif nAccel:
                    del move.stepData.accelPulses[:-nLin]
                    assert(len(move.stepData.accelPulses) == leadAxis_steps)
                else:
                    del move.stepData.decelPulses[nLin:]
                    assert(len(move.stepData.decelPulses) == leadAxis_steps)

            move.stepData.setLinTimer(0xffff)

        if debugAdvance:
            print("# of steps for move: ", leadAxis_steps)
            move.pprint("move:")
            print() 

        move.stepData.checkLen(leadAxis_steps)

        if debugAdvance:
            print("***** End planStepsSimple() *****")

        return True

    def planCrossedDecelSteps(self, move):

        if debugAdvance:
            print("***** Start planCrossedDecelSteps() *****")
            move.pprint("PlanCrossedDecelSteps:")

        move.state = 3

        move.initStepData(StepDataTypeRaw)

        # Zwei gegenläufige bewegungen bezüglich der beschleunigung. Während
        # XYZ abbremst wird die E achse (negativ) beschleunigt.

        # Round step values
        dispF = move.displacement_vector_steps_raw3 + [move.eSteps, 0.0]
        dispS = self.planner.stepRounders.round(dispF)

        self.moveEsteps -= move.eSteps
        # print "planCrossedDecelSteps(): moveEsteps-: %7.3f %7.3f" % (move.eSteps, dispS[A_AXIS])

        if dispS == emptyVector5:
       
            if debugAdvance:
                print("Empty move...")
                print("***** End planCrossedDecelSteps() *****")

            self.planner.stepRounders.rollback()
            return False

        self.planner.stepRounders.commit()

        topSpeed =  move.topSpeed.speed()
        topSpeedS = topSpeed.feedrate3()
        topSpeedE = abs( topSpeed.eSpeed )

        endSpeed =  move.endSpeed.speed()
        endSpeedS = endSpeed.feedrate3()
        endSpeedE = abs( endSpeed.eSpeed )

        # Some sanity tests
        # print "topSpeedS > endSpeedS:", topSpeedS> endSpeedS
        assert(topSpeedS > endSpeedS)           # XYZ should be decelerating
        assert(topSpeedE < endSpeedE) # E should be accelerating

        abs_displacement_vector_steps = vectorAbs(dispS)

        dirBits = util.directionBits(dispS)

        if dirBits != self.planner.curDirBits:
            move.stepData.setDirBits = True
            move.stepData.dirBits = dirBits
            self.planner.curDirBits = dirBits

        leadAxisxy = move.leadAxis(nAxes = 2, disp=dispS)

        ############################################################################################
        #
        # Steps for E acceleration
        #
        eStepsToMove = abs_displacement_vector_steps[A_AXIS]
        eClocks = self.planner.accelRamp(
                self.e_steps_per_mm,
                topSpeedE,
                endSpeedE,
                abs(move.endAccel.eAccel()),
                eStepsToMove,
                0)

        if debugAdvance:
            print("Generated %d/%d E steps" % (len(eClocks), eStepsToMove))

        if not abs_displacement_vector_steps[leadAxisxy]:

            #
            # Move consists of e-steps only
            #
            for tv in eClocks:
                move.stepData.addPulse(tv, [0, 0, 0, 1, 0])

            if debugAdvance:
                move.pprint("planCrossedDecelSteps:")
                print("***** End planCrossedDecelSteps() *****")

            return True

        ############################################################################################
        #
        # Create a list of XY-stepper pulses (DDA)
        #
        xyClocks = self.decelRampXY(
            leadAxisxy,
            self.printer.printerProfile.getStepsPerMMI(leadAxisxy),
            abs(topSpeed[leadAxisxy]),
            abs(endSpeed[leadAxisxy]),
            abs(move.endAccel.accel(leadAxisxy)),
            abs_displacement_vector_steps)

        ############################################################################################

        # Merge steps into list of rawsteps
        # xxx can this be done in one step together with acceleration/deceleration
        # computation. "rasterisierung"
        tv75khz = int(fTimer / 75000)

        tvMap = {}
        currentClock = 0
        nMerges = 0
        for tv in eClocks:

            tvR = round(tv / tv75khz)
            currentClock += tvR

            try:
                tvMap[currentClock]
                assert(0)
            except KeyError:
                tvMap[currentClock] = [0, 0, 0, 1, 0] # e-step at this time

        # print "tvMap:", 
        # pprint.pprint(tvMap)

        currentClock = 0
        for (t, dt, tv, stepBits) in xyClocks:

            tvR = round(tv / tv75khz)
            currentClock += tvR

            try:
                steps = tvMap[currentClock]
                assert(not (steps[0] or steps[1])) # sanity
                steps[0] = stepBits[0]
                steps[1] = stepBits[1]
                nMerges += 1
            except KeyError:
                tvMap[currentClock] = stepBits + [0, 0, 0] # xy-step at this time

        tvIndex = list(tvMap.keys())
        tvIndex.sort()

        # print "tvIndex:", 
        # pprint.pprint(tvIndex)
        # print "tvMap:", 
        # pprint.pprint(tvMap)

        currentClock = 0
        for ts in tvIndex:

            move.stepData.addPulse((ts-currentClock)*tv75khz, tvMap[ts])
            currentClock = ts

        # print "pulses:", 
        # pprint.pprint(move.stepData.pulses)

        if debugAdvance:
        
            leadAxis_stepsxy = abs_displacement_vector_steps[leadAxisxy]
            print("Generated %d/%d XY steps" % (len(xyClocks), leadAxis_stepsxy))
            print("Merged %d steps" % nMerges)
            assert((leadAxis_stepsxy + eStepsToMove) == (len(tvIndex) + nMerges))

            move.pprint("planCrossedDecelSteps:")
            print("***** End planCrossedDecelSteps() *****")

        return True

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

        if debugAdvance:
            print("***** Start planStepsAdvSA() *****")
            parentMove.pprint("planStepsAdvSA:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw3

        displacement_vector_steps_A = [0.0] * 5
        displacement_vector_steps_B = [0.0] * 5

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

            steps = sa * self.printer.printerProfile.getStepsPerMMI(dim)
            # print "dim %d moves %.3f mm while accelerating -> %d steps" % (dim, sa, steps)

            displacement_vector_steps_A[dim] = steps

        # PART A, E
        # print "dim E moves %f steps while accelerating" % parentMove.advanceData.startESteps
        displacement_vector_steps_A[A_AXIS] = parentMove.advanceData.startESteps
        ####################################################################################

        ####################################################################################
        # PART B, XY

        if tl or td:

            for dim in range(3):
                displacement_vector_steps_B[dim] = displacement_vector_steps_raw[dim] - displacement_vector_steps_A[dim]
       
        if tl:

            # print "dim E moves %f steps in linear phase" % parentMove.advanceData.linESteps
            displacement_vector_steps_B[A_AXIS] += parentMove.advanceData.linESteps

        if td:

            # print "dim E moves %f steps while decelerating" % parentMove.advanceData.endESteps
            displacement_vector_steps_B[A_AXIS] += parentMove.advanceData.endESteps
        ####################################################################################

        # print "new A steps: ", displacement_vector_steps_A
        # print "new B steps: ", displacement_vector_steps_B

        from move import SubMove

        moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)

        moveA.setDuration(ta, 0, 0)

        sv = parentMove.startSpeed.speed().copy()
        sv.setESpeed(parentMove.advanceData.startEFeedrate())

        tv = parentMove.topSpeed.speed().copy()
        tv.setESpeed(parentMove.advanceData.startEReachedFeedrate())

        moveA.setSpeeds(sv, tv, tv)

        newMoves = [moveA]

        if tl or td:

            if displacement_vector_steps_B != emptyVector5:

                moveB = SubMove(parentMove, parentMove.moveNumber + 2, displacement_vector_steps_B)
                moveB.setDuration(0, tl, td)
     
                moveB.startSpeed.setSpeed(topSpeed, "planStepsAdvSA()")
                moveB.topSpeed.setSpeed(topSpeed, "planStepsAdvSA()")
                moveB.endSpeed.setSpeed(endSpeed, "planStepsAdvSA()")

                newMoves.append(moveB)

            else:

                print("planStepsAdvSA: skipping empty b-move") 

        # Sum up additional e-distance of this move for debugging
        parentMove.advanceData.advStepSum -= displacement_vector_steps_A[A_AXIS]+displacement_vector_steps_B[A_AXIS]

        if debugAdvance:
            print("***** End planStepsAdvSA() *****")

        xyzStepSum = vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_B[:3])
        stepsMissing = vectorSub(displacement_vector_steps_raw[:3], xyzStepSum)
        # print "stepsMissing:", stepsMissing
        assert(vectorLength(stepsMissing) < 0.000001)

        return newMoves

    #
    # Single advanceed ramp at end
    # Create addtional 'sub-move' at end
    #
    def planStepsAdvSD(self, parentMove):

        if debugAdvance:
            print("***** Start planStepsAdvSD() *****")
            parentMove.pprint("planStepsAdvSD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw3

        displacement_vector_steps_A = [0.0] * 5
        displacement_vector_steps_B = [0.0] * 5

        ta = parentMove.accelTime()
        tl = parentMove.linearTime()
        td = parentMove.decelTime()

        startSpeed = parentMove.startSpeed.speed()
        topSpeed =   parentMove.topSpeed.speed()
        endSpeed =   parentMove.endSpeed.speed()

        ####################################################################################
        # PART B, XY
        if ta or tl:
            for dim in [X_AXIS, Y_AXIS]:

                sd = parentMove.endRampDistance(
                    topSpeed[dim],
                    endSpeed[dim],
                    td)

                steps = sd * self.printer.printerProfile.getStepsPerMMI(dim)
                # print "dim %d moves %.3f mm while decelerating -> %f steps" % (dim, sd, steps)

                displacement_vector_steps_B[dim] = steps
        else:

                displacement_vector_steps_B = displacement_vector_steps_raw + [0.0, 0.0]
       
        # # PART B, E 
        # print "dim E moves %f steps while decelerating" % parentMove.advanceData.endESteps
        displacement_vector_steps_B[A_AXIS] = parentMove.advanceData.endESteps
        ####################################################################################

        ####################################################################################
        if ta or tl:

            # PART A, XY
            for dim in range(3):
                displacement_vector_steps_A[dim] = displacement_vector_steps_raw[dim] - displacement_vector_steps_B[dim]
     
        # PART A, E
        if ta:

            # print "dim E moves %f steps while accelerating" % parentMove.advanceData.startESteps
            displacement_vector_steps_A[A_AXIS] += parentMove.advanceData.startESteps

        if tl:

            # print "dim E moves %f steps in linear phase" % parentMove.advanceData.linESteps
            displacement_vector_steps_A[A_AXIS] += parentMove.advanceData.linESteps
        ####################################################################################

        # print "new A steps: ", displacement_vector_steps_A
        # print "new B steps: ", displacement_vector_steps_B

        from move import SubMove

        moveB = SubMove(parentMove, parentMove.moveNumber + 2, displacement_vector_steps_B)
        
        newMoves = [moveB]

        moveB.setDuration(0, 0, td)

        sv = parentMove.topSpeed.speed().copy()
        sv.setESpeed(parentMove.advanceData.endEReachedFeedrate())

        ev = parentMove.endSpeed.speed().copy()
        ev.setESpeed(parentMove.advanceData.endEFeedrate())

        moveB.setSpeeds(sv, sv, ev)

        if (ta or tl) and displacement_vector_steps_A != emptyVector5:

            moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)
            moveA.setDuration(ta, tl, 0)

            moveA.startSpeed.setSpeed(startSpeed, "planStepsAdvSD()")
            moveA.topSpeed.setSpeed(topSpeed, "planStepsAdvSD()")
            moveA.endSpeed.setSpeed(topSpeed, "planStepsAdvSD()")

            newMoves.insert(0, moveA)

        # Sum up additional e-distance of this move for debugging
        esteps = displacement_vector_steps_A[A_AXIS]+displacement_vector_steps_B[A_AXIS]
        parentMove.advanceData.advStepSum -= esteps

        if debugAdvance:
            print("***** End planStepsAdvSD() *****")

        xyzStepSum = vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_B[:3])
        stepsMissing = vectorSub(displacement_vector_steps_raw[:3], xyzStepSum)
        # print "stepsMissing:", stepsMissing
        assert(vectorLength(stepsMissing) < 0.000001)

        return newMoves

    #
    # Simple advanceed ramp at start and end
    # optional linear middle part
    # Generates 2 or 3 moves
    #
    def planStepsAdvSALSD(self, parentMove):

        if debugAdvance:
            print("***** Start planStepsAdvSALSD() *****")
            parentMove.pprint("planStepsAdvSALSD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw3

        displacement_vector_steps_A = [0.0] * 5
        displacement_vector_steps_B = [0.0] * 5
        displacement_vector_steps_C = [0.0] * 5

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

            steps = sa * self.printer.printerProfile.getStepsPerMMI(dim)
            # print "dim %d moves %.3f mm while accelerating -> %f steps" % (dim, sa, steps)

            displacement_vector_steps_A[dim] = steps

        # PART A, E
        # print "dim E moves %f steps while accelerating" % parentMove.advanceData.startESteps
        displacement_vector_steps_A[A_AXIS] = parentMove.advanceData.startESteps
        ####################################################################################

        ####################################################################################
        # PART C, XY
        for dim in [X_AXIS, Y_AXIS]:

            sd = parentMove.endRampDistance(
                    topSpeed[dim],
                    endSpeed[dim],
                    td)

            steps = sd * self.printer.printerProfile.getStepsPerMMI(dim)
            # print "dim %d moves %.3f mm while decelerating -> %f steps" % (dim, sd, steps)

            displacement_vector_steps_C[dim] = steps
       
        # PART C, E
        # print "dim E moves %f steps while decelerating" % parentMove.advanceData.endESteps
        displacement_vector_steps_C[A_AXIS] = parentMove.advanceData.endESteps
        ####################################################################################

        # Distribute missing X/Y steps from rounding errors (only if no linear part that uses them)
        stepsUsed = vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_C[:3])
        stepsMissing = vectorSub(displacement_vector_steps_raw[:3], stepsUsed)

        ####################################################################################
        if tl:

            # Optional PART B, XY
            for dim in range(3):
                displacement_vector_steps_B[dim] = displacement_vector_steps_raw[dim] - (displacement_vector_steps_A[dim] + displacement_vector_steps_C[dim])
      
            # PART B, E
            # print "dim E moves %f steps in linear phase" % parentMove.advanceData.linESteps
            displacement_vector_steps_B[A_AXIS] = parentMove.advanceData.linESteps

        elif stepsMissing != emptyVector3:

            # print "stepsMissing:", stepsMissing
            assert(vectorLength(stepsMissing) < 1.0)

            maxSteps = displacement_vector_steps_A
            maxLen = vectorLength(maxSteps[:3])
            if vectorLength(displacement_vector_steps_C[:3]) > maxLen:
                maxSteps = displacement_vector_steps_C

            for dim in [X_AXIS, Y_AXIS]:
                maxSteps[dim] += stepsMissing[dim]
            # print "adjusted steps: ", maxSteps

        ####################################################################################

        # print "new A steps: ", displacement_vector_steps_A
        # print "new B steps: ", displacement_vector_steps_B
        # print "new C steps: ", displacement_vector_steps_C

        from move import SubMove

        moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)
        moveC = SubMove(parentMove, parentMove.moveNumber + 3, displacement_vector_steps_C)

        newMoves = [moveA, moveC]
       
        moveA.setDuration(ta, 0, 0)
        moveC.setDuration(0, 0, td)

        sv = parentMove.startSpeed.speed().copy()
        sv.setESpeed(parentMove.advanceData.startEFeedrate())

        tv = parentMove.topSpeed.speed().copy()
        tv.setESpeed(parentMove.advanceData.startEReachedFeedrate())

        moveA.setSpeeds(sv, tv, tv)

        sv = parentMove.topSpeed.speed().copy()
        sv.setESpeed(parentMove.advanceData.endEReachedFeedrate())

        ev = parentMove.endSpeed.speed().copy()
        ev.setESpeed(parentMove.advanceData.endEFeedrate())

        moveC.setSpeeds(sv, sv, ev)

        if tl:
            moveB = SubMove(parentMove, parentMove.moveNumber + 2, displacement_vector_steps_B)
            moveB.setDuration(0, tl, 0)

            moveB.startSpeed.setSpeed(topSpeed, "planStepsAdvSALSD(")
            moveB.topSpeed.setSpeed(topSpeed, "planStepsAdvSALSD(")
            moveB.endSpeed.setSpeed(topSpeed, "planStepsAdvSALSD(")

            newMoves.insert(1, moveB)

        # Sum up additional e-distance of this move for debugging
        esteps = displacement_vector_steps_A[A_AXIS]+displacement_vector_steps_B[A_AXIS]+displacement_vector_steps_C[A_AXIS]
        parentMove.advanceData.advStepSum -= esteps

        if debugAdvance:
            print("***** End planStepsAdvSALSD() *****")

        xyzStepSum = vectorAdd(vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_B[:3]), displacement_vector_steps_C[:3])
        stepsMissing = vectorSub(displacement_vector_steps_raw[:3], xyzStepSum)
        # print "stepsMissing:", stepsMissing
        assert(vectorLength(stepsMissing) < 0.000001)

        return newMoves


    #
    # Advanced ramp with sign-change at the end
    # Optional accel/linear part
    # Generates 2 or 3 moves
    def planStepsAdvLDD(self, parentMove):

        if debugAdvance:
            print("***** Start planStepsAdvLDD() *****")
            parentMove.pprint("planStepsAdvLDD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw3

        displacement_vector_steps_A = [0.0] * 5
        displacement_vector_steps_B = [0.0] * 5
        displacement_vector_steps_C = [0.0] * 5

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

            steps = sd * self.printer.printerProfile.getStepsPerMMI(dim)
            # print "dim %d moves %.3f mm while decelerating -> %f steps" % (dim, sd, steps)

            displacement_vector_steps_B[dim] = steps
       
        # PART B, E
        displacement_vector_steps_B[A_AXIS] = parentMove.advanceData.endEStepsC
        # print "tdc esteps: ", parentMove.advanceData.endEStepsC
        ####################################################################################

        ####################################################################################
        # PART C, XY
        for dim in [X_AXIS, Y_AXIS]:

            sd = parentMove.endRampDistance(
                    parentMove.advanceData.crossingSpeed[dim],
                    endSpeed[dim],
                    parentMove.advanceData.tdd)

            steps = sd * self.printer.printerProfile.getStepsPerMMI(dim)
            # print "dim %d moves %.3f mm while decelerating -> %f steps" % (dim, sd, steps)

            displacement_vector_steps_C[dim] = steps
       
        # PART C, E
        displacement_vector_steps_C[A_AXIS] = parentMove.advanceData.endEStepsD
        # print "tdd esteps: ", parentMove.advanceData.endEStepsD
        ####################################################################################

        # Distribute missing X/Y steps from rounding errors (only if no other part that uses them)
        stepsUsed = vectorAdd(displacement_vector_steps_B[:3], displacement_vector_steps_C[:3])
        stepsMissing = vectorSub(displacement_vector_steps_raw[:3], stepsUsed)

        # PART A
        if ta or tl:

            for dim in range(3):
                displacement_vector_steps_A[dim] = displacement_vector_steps_raw[dim] - stepsUsed[dim]

        elif stepsMissing != emptyVector3:
        
            # print "stepsMissing:", stepsMissing
            assert(vectorLength(stepsMissing) < 1.0)

            maxSteps = displacement_vector_steps_B
            maxLen = vectorLength(maxSteps[:3])
            if vectorLength(displacement_vector_steps_C[:3]) > maxLen:
                maxSteps = displacement_vector_steps_C

            for dim in [X_AXIS, Y_AXIS]:
                maxSteps[dim] += stepsMissing[dim]
            # print "adjusted steps: ", maxSteps

        if ta:

            # print "dim E moves %f steps while accelerating" % parentMove.advanceData.startESteps
            displacement_vector_steps_A[A_AXIS] += parentMove.advanceData.startESteps

        if tl:

            # print "dim E moves %f steps in linear phase" % parentMove.advanceData.linESteps
            displacement_vector_steps_A[A_AXIS] += parentMove.advanceData.linESteps

        ####################################################################################

        # print "new A steps: ", displacement_vector_steps_A
        # print "new B steps: ", displacement_vector_steps_B
        # print "new C steps 2: ", displacement_vector_steps_C

        from move import SubMove


        moveC = SubMove(parentMove, parentMove.moveNumber + 3, displacement_vector_steps_C)

        newMoves = [moveC]
      
        moveC.setDuration(0, 0, parentMove.advanceData.tdd)

        sv = parentMove.advanceData.crossingSpeed.copy()
        sv.setESpeed(0)
        ev = endSpeed.copy()
        ev.setESpeed(parentMove.advanceData.endEFeedrate())
        moveC.setSpeeds(sv, sv, ev)

        # if ta or tl:
        if displacement_vector_steps_A != emptyVector5:

            moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)
            moveA.setDuration(ta, tl, 0)

            moveA.startSpeed.setSpeed(startSpeed, "planStepsAdvLDD()")
            moveA.topSpeed.setSpeed(topSpeed, "planStepsAdvLDD()")
            moveA.endSpeed.setSpeed(topSpeed, "planStepsAdvLDD()")

            newMoves.insert(0, moveA)

        if displacement_vector_steps_B != emptyVector5:

            moveB = SubMove(parentMove, parentMove.moveNumber + 2, displacement_vector_steps_B)
            moveB.setDuration(0, 0, parentMove.advanceData.tdc)

            sv = topSpeed.copy()
            sv.setESpeed(parentMove.advanceData.endEReachedFeedrate())
            ev = parentMove.advanceData.crossingSpeed.copy()
            ev.setESpeed(0)
            moveB.setSpeeds(sv, sv, ev)

            newMoves.insert(-1, moveB)

        # Sum up additional e-distance of this move for debugging
        esteps = displacement_vector_steps_A[A_AXIS]+displacement_vector_steps_B[A_AXIS]+displacement_vector_steps_C[A_AXIS]
        parentMove.advanceData.advStepSum -= esteps

        if debugAdvance:
            print("***** End planStepsAdvLDD() *****")

        xyzStepSum = vectorAdd(vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_B[:3]), displacement_vector_steps_C[:3])
        stepsMissing = vectorSub(displacement_vector_steps_raw[:3], xyzStepSum)
        # print "stepsMissing:", stepsMissing
        assert(vectorLength(stepsMissing) < 0.000001)

        return newMoves

    #
    # Simple advanceed ramp at start and advanced ramp with sign-change at the end
    # Optional linear part
    # Generates 3 or 4 moves
    def planStepsAdvSALDD(self, parentMove):

        if debugAdvance:
            print("***** Start planStepsAdvSALDD() *****")
            parentMove.pprint("planStepsAdvSALDD:")

        displacement_vector_steps_raw = parentMove.displacement_vector_steps_raw3

        displacement_vector_steps_A = [0.0] * 5
        displacement_vector_steps_B = [0.0] * 5
        displacement_vector_steps_C = [0.0] * 5
        displacement_vector_steps_D = [0.0] * 5

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

            steps = sa * self.printer.printerProfile.getStepsPerMMI(dim)
            # print "dim %d moves %.3f mm while accelerating -> %f steps" % (dim, sa, steps)

            displacement_vector_steps_A[dim] = steps

        # print "dim E moves %f steps while accelerating" % parentMove.advanceData.startESteps
        displacement_vector_steps_A[A_AXIS] = parentMove.advanceData.startESteps
        ####################################################################################

        ####################################################################################
        # PART C, XY
        for dim in [X_AXIS, Y_AXIS]:

            sd = parentMove.endRampDistance(
                    topSpeed[dim],
                    parentMove.advanceData.crossingSpeed[dim],
                    parentMove.advanceData.tdc)

            steps = sd * self.printer.printerProfile.getStepsPerMMI(dim)
            # print "dim %d moves %.3f mm while decelerating -> %f steps" % (dim, sd, steps)

            displacement_vector_steps_C[dim] = steps
      
        displacement_vector_steps_C[A_AXIS] = parentMove.advanceData.endEStepsC
        # print "tdc esteps: ", parentMove.advanceData.endEStepsC

        # PART D, XY
        for dim in [X_AXIS, Y_AXIS]:

            sd = parentMove.endRampDistance(
                    parentMove.advanceData.crossingSpeed[dim],
                    endSpeed[dim],
                    parentMove.advanceData.tdd)

            steps = sd * self.printer.printerProfile.getStepsPerMMI(dim)
            # print "dim %d moves %.3f mm while decelerating -> %f steps" % (dim, sd, steps)

            displacement_vector_steps_D[dim] = steps
       
        # PART D, E
        displacement_vector_steps_D[A_AXIS] = parentMove.advanceData.endEStepsD
        # print "tdd esteps: ", parentMove.advanceData.endEStepsD
        ####################################################################################

        # Distribute missing X/Y steps from rounding errors (only if no linear part that uses them)
        stepsUsed = vectorAdd(vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_C[:3]), displacement_vector_steps_D[:3])
        stepsMissing = vectorSub(displacement_vector_steps_raw[:3], stepsUsed)

        ####################################################################################
        # PART B
        if tl:

            for dim in range(3):
                displacement_vector_steps_B[dim] = displacement_vector_steps_raw[dim] - (displacement_vector_steps_A[dim] + displacement_vector_steps_C[dim] + displacement_vector_steps_D[dim])
       
            # print "dim E moves %f steps in linear phase" % parentMove.advanceData.linESteps
            displacement_vector_steps_B[A_AXIS] += parentMove.advanceData.linESteps

        elif stepsMissing != emptyVector3:

            # print "stepsMissing:", stepsMissing

            maxSteps = displacement_vector_steps_A
            maxLen = vectorLength(maxSteps[:3])
            for dvs in [displacement_vector_steps_C, displacement_vector_steps_D]:
                if vectorLength(dvs[:3]) > maxLen:
                    maxSteps = dvs
                    maxLen = vectorLength(dvs[:3])

            assert((vectorLength(stepsMissing)/maxLen) < 0.001)

            for dim in [X_AXIS, Y_AXIS]:
                maxSteps[dim] += stepsMissing[dim]
            # print "adjusted steps: ", maxSteps

        ####################################################################################

        # print "new A steps: ", displacement_vector_steps_A
        # print "new B steps: ", displacement_vector_steps_B
        # print "new C steps 3: ", displacement_vector_steps_C
        # print "new D steps: ", displacement_vector_steps_D

        from move import SubMove

        moveA = SubMove(parentMove, parentMove.moveNumber + 1, displacement_vector_steps_A)
        moveD = SubMove(parentMove, parentMove.moveNumber + 4, displacement_vector_steps_D)
       
        moveA.setDuration(ta, 0, 0)
        moveD.setDuration(0, 0, parentMove.advanceData.tdd)

        # newMoves = [moveA, moveC, moveD]
        newMoves = [moveA]

        sv = parentMove.startSpeed.speed().copy()
        sv.setESpeed(parentMove.advanceData.startEFeedrate())
        tv = parentMove.topSpeed.speed().copy()
        tv.setESpeed(parentMove.advanceData.startEReachedFeedrate())
        moveA.setSpeeds(sv, tv, tv)

        if displacement_vector_steps_C != emptyVector5:

            moveC = SubMove(parentMove, parentMove.moveNumber + 3, displacement_vector_steps_C)
            moveC.setDuration(0, 0, parentMove.advanceData.tdc)

            newMoves.append(moveC)

            sv = parentMove.topSpeed.speed().copy()
            sv.setESpeed(parentMove.advanceData.endEReachedFeedrate())
            ev = parentMove.advanceData.crossingSpeed.copy()
            ev.setESpeed(0)
            moveC.setSpeeds(sv, sv, ev)

        sv = parentMove.advanceData.crossingSpeed.copy()
        sv.setESpeed(0)
        ev = parentMove.endSpeed.speed().copy()
        ev.setESpeed(parentMove.advanceData.endEFeedrate())
        moveD.setSpeeds(sv, sv, ev)

        newMoves.append(moveD)

        if tl:

            moveB = SubMove(parentMove, parentMove.moveNumber + 2, displacement_vector_steps_B)

            moveB.setDuration(0, tl, 0)

            moveB.startSpeed.setSpeed(topSpeed, "planStepsAdvSALDD()")
            moveB.topSpeed.setSpeed(topSpeed, "planStepsAdvSALDD()")
            moveB.endSpeed.setSpeed(topSpeed, "planStepsAdvSALDD()")

            newMoves.insert(1, moveB)

        # Sum up additional e-distance of this move for debugging
        esteps = displacement_vector_steps_A[A_AXIS]+displacement_vector_steps_B[A_AXIS]+displacement_vector_steps_C[A_AXIS]+displacement_vector_steps_D[A_AXIS]
        parentMove.advanceData.advStepSum -= esteps

        if debugAdvance:
            print("***** End planStepsAdvSALDD() *****")

        xyzStepSum = vectorAdd(vectorAdd(vectorAdd(displacement_vector_steps_A[:3], displacement_vector_steps_B[:3]), displacement_vector_steps_C[:3]), displacement_vector_steps_D[:3])
        stepsMissing = vectorSub(displacement_vector_steps_raw[:3], xyzStepSum)
        # print "stepsMissing:", stepsMissing
        assert(vectorLength(stepsMissing) < 0.000001)

        return newMoves

    ####################################################################################################
    #
    # Create a list of stepper pulses for a deceleration ramp.
    #
    def decelRampXY(self, leadAxis, leadStepsPerMM, vstart, vend, a, absSteps):

        assert(vstart >= vend)

        pulses = [] # (tstep, dt, timerValue)

        leadSteps = absSteps[leadAxis]

        otherAxis = Y_AXIS if leadAxis == X_AXIS else X_AXIS
        otherSteps = absSteps[otherAxis]

        bFactor = float(otherSteps) / leadSteps
        # print "bfactor:", bFactor
        otherCount = 0

        sPerStep = 1.0/leadStepsPerMM

        tstep = 0

        # Lower speed
        maxStepTime = maxTimerValue16 / fTimer
        vmin = sPerStep / maxStepTime

        stepsToDo = 0
        if vstart > vmin:

            vend = max(vend, vmin)

            # Number of lead axis steps needed
            dv = vstart - vend
            dt = dv / a
            s = util.accelDist(vstart, -a, dt)

            # print "vstart, vend, a", vstart, vend, a
            # print "dv, dt, s:", dv, dt, s

            stepsToDo = int(s * leadStepsPerMM)

        else:

            vstart = vmin

        # print "doing ", stepsToDo, "of", leadSteps

        # # debug
        # prepended = False

        if leadSteps > stepsToDo:

            # Prepend fast steps
            for i in range(leadSteps - stepsToDo):

                # Time we need for this speed change/this step:
                dt = sPerStep / vstart

                # Timervalue for this time
                timerValue = self.planner.timerLimit(int(dt * fTimer))

                stepBits = [0, 0]
                stepBits[leadAxis] = 1

                otherCount += bFactor

                if otherCount >= 0.5:
                    stepBits[otherAxis] = 1
                    otherSteps -= 1
                    otherCount -= 1.0

                # print "steps, otherCount:", stepBits, otherCount
                pulses.append((tstep, dt, timerValue, stepBits))

                tstep += dt
                leadSteps -= 1

            # prepended = True

        v = vstart
        s = sPerStep

        # while v > vend and leadSteps > 0:
        while leadSteps > 0:

            # Speed after this step
            # print "vstar, -a, s:", vstart, -a, s
            vn1 = util.vAccelPerDist(vstart, -a, s)

            # Time we need for this speed change/this step:
            dv = v - vn1
            dt = dv / a

            # Timervalue for this time
            timerValue = self.planner.timerLimit(int(dt * fTimer))

            # if timerValue > maxTimerValue16:
                # # print "break on timeroverflow, v after this step:", vn1, s, dt, timerValue
                # break

            # print "v after this step:", vn1, s, dt, timerValue

            stepBits = [0, 0]
            stepBits[leadAxis] = 1

            otherCount += bFactor

            # if otherCount >= 1:
            if otherCount >= 0.5:
                stepBits[otherAxis] = 1
                otherSteps -= 1
                otherCount -= 1.0

            # print "steps, otherCount:", stepBits, otherCount
            pulses.append((tstep, dt, timerValue, stepBits))

            s += sPerStep
            v = vn1
            tstep += dt
            leadSteps -= 1

        # print "Missing steps: ", leadSteps, otherSteps

        # if prepended:
            # print "prepended steps:"
            # pprint.pprint(pulses)

        assert(leadSteps == 0 and otherSteps == 0)
        return pulses

    def plotPlannedPath(self, path, plotfile=None):

        if not plotfile:
            plotfile = self.plotfile

        for move in path:

            plotfile.plot1Tick(move.topSpeed.speed().feedrate3(), move.moveNumber)

            at = move.accelTime()
            lt = move.linearTime()
            dt = move.decelTime()

            if at:

                plotfile.plot1Segments(at, (
                    DebugPlotSegment(move.startSpeed.speed().feedrate3(), move.topSpeed.speed().feedrate3(), "green"),
                    DebugPlotSegment(move.startSpeed.speed().eSpeed, move.topSpeed.speed().eSpeed, "green"),
                    ))

            if lt:

                plotfile.plot1Segments(lt, (
                    DebugPlotSegment(move.topSpeed.speed().feedrate3(), color="blue"),
                    DebugPlotSegment(move.topSpeed.speed().eSpeed, color="blue"),
                    ))

            if dt:

                plotfile.plot1Segments(dt, (
                    DebugPlotSegment(move.topSpeed.speed().feedrate3(), move.endSpeed.speed().feedrate3(), "red"),
                    DebugPlotSegment(move.topSpeed.speed().eSpeed, move.endSpeed.speed().eSpeed, "red"),
                    ))

            if at:

                plotfile.plot2Segments(0, (
                    DebugPlotSegment(move.startSpeed.speed().eSpeed, move.advanceData.startEFeedrate(), "green"),
                    ))
                plotfile.plot2Segments(at, (
                    DebugPlotSegment(move.startSpeed.speed().eSpeed, move.topSpeed.speed().eSpeed),
                    DebugPlotSegment(move.advanceData.startEFeedrate(), move.advanceData.startEReachedFeedrate(), "green"),
                    ))
                plotfile.plot2Segments(0, (
                    DebugPlotSegment(move.advanceData.startEReachedFeedrate(), move.topSpeed.speed().eSpeed, "green"),
                    ))

            if lt:
                plotfile.plot2Segments(lt, (
                    DebugPlotSegment(move.topSpeed.speed().eSpeed),
                    ))

            if dt:

                plotfile.plot2Segments(0, (
                    DebugPlotSegment(move.topSpeed.speed().eSpeed, move.advanceData.endEReachedFeedrate(), "red"),
                    ))
                plotfile.plot2Segments(dt, (
                    DebugPlotSegment(move.topSpeed.speed().eSpeed, move.endSpeed.speed().eSpeed),
                    DebugPlotSegment(move.advanceData.endEReachedFeedrate(), move.advanceData.endEFeedrate(), "red"),
                    ))
                plotfile.plot2Segments(0, (
                    DebugPlotSegment(move.advanceData.endEFeedrate(), move.endSpeed.speed().eSpeed, "red"),
                    ))

        plotfile.close()


