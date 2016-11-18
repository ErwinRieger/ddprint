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

import math, collections, types
from argparse import Namespace

import ddprintutil as util, dddumbui, packedvalue
from ddprofile import PrinterProfile, MatProfile, NozzleProfile
from ddvector import Vector, vectorMul, vectorAbs
from ddprintconstants import *
from ddconfig import *
from ddprintutil import Z_AXIS, circaf
from ddprinter import Printer
from ddprintcommands import CmdSyncTargetTemp
from ddprintstates import HeaterEx1, HeaterBed
from ddadvance import Advance

#####################################################################

if debugPlot:
    import pickle

#####################################################################
#
# "AutoTemp" constants
#
# Don't adjust hotend temperature on every move, collect moves for
# ATInterval time and compute/set the temp for that interval.
#
ATInterval = 3 # [s]
# ATMaxTempIncrease = 50

emptyVector5 = [0] * 5

#####################################################################
#
# Computes some statistics about the used maximal extrusion rates.
#
class MaxExtrusionRate:
    def __init__(self):
        self.maxRate = 0
        self.move = None
        self.max10 = []

        self.maxAvgRate = 0

    def stat(self, move):

        # Do not count very small moves, the computation of the extrusion rate is inaccurate because of the
        # discretization in the gcodeparser (map float values to discrete stepper values).
        if move.distance3 < 0.1:
            return 

        # Get maximum extrusion rate, take plateau speed into account only
        # length of max constant speed:
        reachedEExtrusion = move.topSpeed.speed().eSpeed * MatProfile.getMatArea()

        if reachedEExtrusion > self.maxRate:

            # print "New max reachedEExtrusion: ", reachedEExtrusion, "mm³/s"

            self.maxRate = reachedEExtrusion
            self.move = move

            self.max10.append(reachedEExtrusion)
            if len(self.max10) > 10:
                self.max10 = self.max10[1:]

    def avgStat(self, maxAvgRate):

        if maxAvgRate > self.maxAvgRate:

            # print "New max avg reachedEExtrusion: ", maxAvgRate, "mm³/s"

            self.maxAvgRate = maxAvgRate

    def printStat(self):

        # Extrusion adjust
        adjustedExtrusion = self.maxRate + pow(self.maxRate, 2) * NozzleProfile.getExtrusionAdjustFactor()

        if self.maxRate:
            print "Maximal net Extrusion Rate (Extruder A): %.1f mm³/s, adjusted/gross: %.1f mm³/s, move:" % (self.maxRate, adjustedExtrusion)
            self.move.pprint("Max. extrusion Move")
            print "Net Max10: ", self.max10
            print "Maximal Extrusion Rate (Extruder A) 5 second average: %.1f" % self.maxAvgRate, "mm³/s\n"

#####################################################################

class StepRounder(object):

    def __init__(self, axis):

        self.axis = axis
        self.g92()

    def g92(self):

        self.roundError = 0
        self.f = 0

    def round(self, f):

        assert(type(f) == types.FloatType)

        self.f = f

        s = int(round(f + self.roundError))
       
        # print "round axis %d: %f + %f" % (self.axis, f, self.roundError)
        return s

    def commit(self):

        f = self.f + self.roundError
        s = int(round(f))
        self.roundError = f - s
        self.f = 0

        # print "commit %d, error: %f" % (self.axis, self.roundError)

    def rollback(self):

        self.roundError += self.f
        self.f = 0

        # print "rollback %d, error: %f" % (self.axis, self.roundError)

    def pprint(self):

        assert(self.f == 0)
        print "Reminder axis %s: %f" % (dimNames[self.axis], self.roundError)

        
    def check(self):

        assert(self.f == 0)
        assert(abs(self.roundError) < 1.0)

class StepRounders(object):

    def __init__(self):

        self.stepRounders = [
                StepRounder(0),
                StepRounder(1),
                StepRounder(2),
                StepRounder(3),
                StepRounder(4)
                ]

    def g92(self, values):

        for dim in values.keys():
            self.stepRounders[dimNames.index(dim)].g92()

    def round(self, dispF):

        dispS = []
        for dim in range(5):
            dispS.append(self.stepRounders[dim].round(dispF[dim]))

        return dispS

    def commit(self):

        for i in range(5):
            self.stepRounders[i].commit()

    def rollback(self):

        for i in range(5):
            self.stepRounders[i].rollback()

    def pprint(self):

        for i in range(5):
            self.stepRounders[i].pprint()

    def check(self):

        for i in range(5):
            self.stepRounders[i].check()

class PathData (object):

    def __init__(self, planner):

        self.planner = planner

        self.path = []
        self.count = -10

        # AutoTemp
        if UseExtrusionAutoTemp:

            # Time needed to complete the moves, extruding moves only
            self.time = 0
            # Head move distance sum or extrusion volume, extruding moves only
            self.extrusionAmount = 0
            self.lastTemp = MatProfile.getHotendBaseTemp()

            # Extrusionspeed where autotemp increase starts
            area04 = pow(0.4, 2)*math.pi/4
            self.ExtrusionAmountLow = MatProfile.getBaseExtrusionRate() * (NozzleProfile.getArea() / area04)

        # Some statistics
        self.maxExtrusionRate = MaxExtrusionRate()

    # Number of moves
    def incCount(self):
        self.count += 10 # leave space for advance-submoves
        return self.count

    def doAutoTemp(self, move):

        # Collect moves and sum up path time
        self.time += move.accelData.getTime()

        # Sum extrusion volume
        # self.extrusionAmount += move.getAdjustedExtrusionVolume(A_AXIS, NozzleProfile, MatProfile)
        self.extrusionAmount += move.getExtrusionVolume(MatProfile)

        if self.time >= ATInterval:

            # Compute temperature for this segment and add tempcommand into the stream
            # Average speed:
            avgSpeed = self.extrusionAmount / self.time

            # UseExtrusionAutoTemp: Adjust temp between Tbase and HotendMaxTemp, if speed is greater than 5 mm³/s
            newTemp = MatProfile.getHotendBaseTemp() # Extruder 1 temp

            if avgSpeed > self.ExtrusionAmountLow:
                f = MatProfile.getAutoTempFactor()
                newTemp += (avgSpeed - self.ExtrusionAmountLow) * f
                newTemp = int(min(newTemp, MatProfile.getHotendMaxTemp()))

            if newTemp != self.lastTemp: #  and self.mode != "pre":

                # Schedule target temp command
                self.planner.addSynchronizedCommand(
                    CmdSyncTargetTemp, 
                    p1 = packedvalue.uint8_t(HeaterEx1),
                    p2 = packedvalue.uint16_t(newTemp), 
                    moveNumber = move.moveNumber)

                self.lastTemp = newTemp

            if debugAutoTemp:
                print "AutoTemp: collected moves with %.2f s duration." % self.time
                print "AutoTemp: amount: %.2f mm³, avg extrusion rate: %.2f mm³/s." % (self.extrusionAmount, avgSpeed)
                print "AutoTemp: new temp: %d." % newTemp
                self.maxExtrusionRate.avgStat(avgSpeed)

            self.time = self.extrusionAmount = 0 # Reset path time

#####################################################################

class DebugPlot (object):

    def __init__(self, nr):

        self.plotfile = "/tmp/ddsteps_%04d.pkl" % nr

        self.plot = Namespace()
        # pl.time = 0.01
        self.plot.moves = []

    def plotSteps(self, move):
  
        if not move.empty():
            d = move.stepData.debugPlot()
            d["number"] = move.moveNumber
            self.plot.moves.append(d)

    def close(self):
        pickle.dump(self.plot, open(self.plotfile, "wb"))

#####################################################################

class Planner (object):

    __single = None 

    def __init__(self, args, gui=None):

        if Planner.__single:
            raise RuntimeError('A Planner already exists')

        Planner.__single = self

        if gui:
            self.gui = gui
        else:
            self.gui = dddumbui.DumbGui()

        self.args = args

        self.printer = Printer.get()
        # self.parser = UM2GcodeParser.get()

        jerk = []
        for dim in dimNames:
            jerk.append(PrinterProfile.getValues()['axes'][dim]['jerk'])

        self.jerk = Vector(jerk)
        self.gui.log("Jerk vector: ", self.jerk)

        self.zeroPos = util.MyPoint()

        #
        # Constants, xxx todo: query from printer and/or profile
        #
        self.HOMING_FEEDRATE = [100, 100, 40]  # set the homing speeds (mm/s) 
        self.HOME_RETRACT_MM = 7               # [mm]

        # ENDSTOP SETTINGS:
        # Sets direction of endstops when homing; 1=MAX, -1=MIN
        self.X_HOME_DIR = -1
        self.Y_HOME_DIR = 1
        self.Z_HOME_DIR = 1
        self.HOME_DIR = (self.X_HOME_DIR, self.Y_HOME_DIR, self.Z_HOME_DIR)

        # XXX defined in profile !!!
        # Travel limits after homing
        self.X_MIN_POS = 0
        self.X_MAX_POS = 225.0 # 230.0
        # X_MIN_POS = 0
        self.Y_MAX_POS = 225.0 # 230.0
        # Y_MIN_POS = 0
        # self.Z_MAX_POS = 229.0 # 230.0 // Dauerdruckplatte hat 5mm im vergleich zur glassplatte 4mm
        self.Z_MAX_POS = 212.25 # solex nozzle
        # Z_MIN_POS = 0
        self.MAX_POS = (self.X_MAX_POS, self.Y_MAX_POS, self.Z_MAX_POS)

        # Bed leveling constants
        self.LEVELING_OFFSET = 0.1                   # Assumed thickness of feeler gauge/paper used in leveling (mm)
        # self.HEAD_HEIGHT = 35.0                      # Let enough room for the head, XXX UM2 specific !!!
        self.HEAD_HEIGHT = 15.0                      # Let enough room for the head, XXX UM2 specific !!!

        # Homing
        self.X_HOME_POS = self.X_MIN_POS
        self.Y_HOME_POS = self.Y_MAX_POS
        self.Z_HOME_POS = self.Z_MAX_POS                  # XXX + add_homeing_z
        #
        # End Constants
        #

        self.plotfile = None

        self.advance = Advance(self, args)

        self.reset()

    def reset(self):

        self.pathData = PathData(self)

        self.syncCommands = collections.defaultdict(list)
        self.partNumber = 1

        self.stepRounders = StepRounders()

    @classmethod
    def get(cls):
        return cls.__single

    def getHomePos(self):

        # Get additional z-offset from eeprom
        add_homeing_z = self.printer.getAddHomeing_z()

        assert((add_homeing_z <= 0) and (add_homeing_z >= -35))

        print "add_homeing_z from eeprom: ", add_homeing_z

        # Virtuelle position des druckkopfes falls 'gehomed'
        homePosMM = util.MyPoint(
            X = self.X_HOME_POS,
            Y = self.Y_HOME_POS,
            Z = self.Z_HOME_POS + add_homeing_z,
            )

        # Diese stepper position wird gesetzt falls der drucker 'gehomed' ist
        homePosStepped = vectorMul(homePosMM.vector(), PrinterProfile.getStepsPerMMVector())

        return (homePosMM, homePosStepped)

    def addSynchronizedCommand(self, command, p1=None, p2=None, moveNumber=None):

        if moveNumber == None:
            moveNumber = max(self.pathData.count, 0)

        self.syncCommands[moveNumber].append((command, p1, p2))

    # Called from gcode parser
    def newPart(self, partNumber):
        self.partNumber = partNumber

    # Called from gcode parser
    def layerChange(self, layer):

        if layer == 2:

            self.partNumber -= 1
            if self.partNumber:
                return

            # Reduce bedtemp
            bedTemp = MatProfile.getBedTempReduced()
            self.gui.log("Layer2, reducing bedtemp to: ", bedTemp)
            self.addSynchronizedCommand(
                CmdSyncTargetTemp, 
                p1 = packedvalue.uint8_t(HeaterBed),
                p2 = packedvalue.uint16_t(bedTemp))

    def g92(self, values):

        self.stepRounders.g92(values)

    # Called from gcode parser
    def addMove(self, move):

        move.moveNumber = self.pathData.incCount()

        # print "addmove ...", move.comment
        if debugMoves:
            print "***** Start addMove() *****"
            move.pprint("AddMove")

        pathEnding = False
        if self.pathData.path:
            prevMove = self.pathData.path[-1]
            if prevMove.isPrintMove() == move.isPrintMove():
                prevMove.nextMove = move
                move.prevMove = prevMove
            else:
                pathEnding = True

        if pathEnding:

            if move.isPrintMove():

                #
                # Finisch preceding travelmoves.
                #
                #
                # Do a simple path planning for traveling moves:
                # * start/stop at jerk/2
                # * don't do advance
                #
                # print "addMove(): ending travel path with %d moves" % len(self.pathData.path)
                self.planTravelPath(self.pathData.path)

            else:

                #
                # Finisch preceding printmoves.
                #
                # print "addMove(): ending path print with %d moves" % len(self.pathData.path)
                self.advance.planPath(self.pathData.path)

                if debugPlot:
                    self.plotfile.close()
                    self.plotfile = None

            self.pathData.path = []

        self.pathData.path.append(move)

        if debugMoves:
            print "***** End addMove() *****"
        
    def planTravelPath(self, path):

        # old ################################################################
        """
        """
        # end old ################################################################

        if debugMoves:
            print "***** Start planTravelPath() *****"

        # Set startspeed of first move
        path[0].setPlannedJerkStartSpeed(self.jerk, "planTravelPath() first move")

        prevMove = path[0]

        # Step 1: join moves forward
        for move in path[1:]:
        
            util.joinTravelMoves(prevMove, move, self.jerk)
            prevMove = move

        for move in path:
            move.state = 1

        # Check max endspeed of last move
        # Set endspeed of last move
        lastMove = path[-1]

        # Max reachable speed of last move
        allowedAccel5 = lastMove.getMaxAllowedAccelNoAdv5()
        maxEndSpeed = util.vAccelPerDist(lastMove.startSpeed.speed().feedrate5(), allowedAccel5, lastMove.distance5)

        v = lastMove.getJerkSpeed(self.jerk) or lastMove.topSpeed.speed()

        endSpeed = lastMove.endSpeed.speed()
        endSpeed.setSpeed(min(v.feedrate5(), maxEndSpeed))
        lastMove.endSpeed.setSpeed(endSpeed, "planTravelPath() - last move")

        """
        # Sanity check
        for move in path:
            move.sanityCheck(self.jerk)
        """

        # Step 2: join moves backwards
        self.joinTravelMovesBwd(path)

        """
        # Sanity check
        for move in path:
            move.sanityCheck(self.jerk)
        """

        # Step 3: plan acceleration
        for move in path:
            self.planTravelAcceleration(move)

        # Sanity check
        for move in path:
            move.sanityCheck(self.jerk)

        # Step 4: plan steps and stream moves to printer
        if debugMoves:
            print "Streaming %d travel moves..." % len(path)

        for move in path:

            self.planTravelSteps(move)

            if debugMoves:
                print "Streaming travel-move:", move.moveNumber

            self.streamMove(move)

            # Help garbage collection
            move.prevMove = util.StreamedMove()
            move.nextMove = util.StreamedMove()

        if debugMoves:
            print "***** End planTravelPath() *****"

    # xxx should be called finishPath()
    def finishMoves(self):

        # debug
        self.stepRounders.check()

        if self.pathData.path:
            
            move = self.pathData.path[0]
            if move.isPrintMove():

                # print "finishMoves(): ending path print with %d moves" % len(self.pathData.path)
                self.advance.planPath(self.pathData.path)

                if debugPlot:
                    self.plotfile.close()
                    self.plotfile = None

                print "\nStatistics:"
                print "-----------"
                self.pathData.maxExtrusionRate.printStat()

            else:

                # print "finishMoves(): ending travel path with %d moves" % len(self.pathData.path)
                self.planTravelPath(self.pathData.path)

        assert(not self.syncCommands)
        self.reset()

    def streamMove(self, move):

        def sendSyncCommands(moveNumber):

            for (cmd, p1, p2) in self.syncCommands[moveNumber]:
                if self.args.mode != "pre":
                    self.printer.sendCommandParamV(cmd, [p1, p2])

            del self.syncCommands[moveNumber]

        if debugPlot:

            if not self.plotfile:
                self.plotfile = DebugPlot(move.moveNumber)

            self.plotfile.plotSteps(move)

        if move.moveNumber in self.syncCommands:
            sendSyncCommands(move.moveNumber)

        if move.isSubMove() and move.parentMove.moveNumber in self.syncCommands:
            sendSyncCommands(move.parentMove.moveNumber)

        for (cmd, cobsBlock) in move.commands():
            if self.args.mode != "pre":
                self.printer.sendCommandC(cmd, cobsBlock)

    #
    #
    #
    def joinTravelMovesBwd(self, moves):

        if debugMoves:
            print "***** Start joinTravelMovesBwd() *****"

        index = len(moves) - 1
        while index >= 0:

            move = moves[index]
            index -= 1

            if debugMoves: 
                move.pprint("joinTravelMovesBwd")

            # Check, if deceleration between startspeed and endspeed of
            # move is possible with the given acceleration and within the 
            # given distance:

            startSpeed1 = move.startSpeed.speed()
            startSpeed1S = startSpeed1.feedrate5()

            endSpeed1 = move.endSpeed.speed()
            endSpeed1S = endSpeed1.feedrate5()

            allowedAccel5 = move.getMaxAllowedAccelNoAdv5()

            maxAllowedStartSpeed = util.vAccelPerDist(endSpeed1S, allowedAccel5, move.distance5)

            # print "joinMovesBwd, startspeed, max startspeed: ", startSpeedS, maxAllowedStartSpeed



            if maxAllowedStartSpeed >= startSpeed1S:

                # Join speeds ok
                continue

            if debugMoves: 
                print "Startspeed of %.5f is to high to reach the desired endspeed." % startSpeed1S
                print "Max. allowed startspeed: %.5f." % maxAllowedStartSpeed

            # Adjust startspeed of this move:
            startSpeed1.setSpeed(maxAllowedStartSpeed)
            move.startSpeed.setSpeed(startSpeed1, "joinTravelMovesBwd - breaking")

            if move.prevMove:

                #
                # Adjust endspeed of the previous move, also.
                #

                factor = maxAllowedStartSpeed / startSpeed1S
                # print "factor: ", factor

                # Adjust endspeed of last move:
                assert(factor < 1)

                # XXX einfacher algo, kann man das besser machen (z.b. mit jerk-berechnung,
                # vector subtraktion oder so?)
                endSpeed0 = move.prevMove.endSpeed.speed().scale(factor)
                move.prevMove.endSpeed.setSpeed(endSpeed0, "joinMovesBwd - prevMove breaking")

        if debugMoves:
            print "***** End joinTravelMovesBwd() *****"

    def planTravelAcceleration(self, move):

        if debugMoves: 
            print "***** Start planTravelAcceleration() *****"
            move.pprint("planTravelAcceleration")

        move.state = 2

        allowedAccel = allowedDecel = move.getMaxAllowedAccelNoAdv5()

        #
        # Check if the speed difference between startspeed and endspeed can be done with
        # this acceleration in this distance
        #
        startSpeedS = move.startSpeed.speed().feedrate
        endSpeedS = move.endSpeed.speed().feedrate

        deltaSpeedS = endSpeedS - startSpeedS

        if abs(deltaSpeedS) > 0.001:
       
            ta = abs(deltaSpeedS) / allowedAccel

            if deltaSpeedS >= 0:
                # acceleration
                sa = util.accelDist(startSpeedS, allowedAccel, ta)
            else:
                # deceleration
                sa = util.accelDist(startSpeedS, -allowedAccel, ta)
      
            if (sa - move.distance5) > 0.001:
                print "VStart %f mm/s kann nicht innerhalb von %f mm auf Endgeschwindigkeit %f mm/s gebracht werden!" % (startSpeedS, move.distance5, endSpeedS)
                print "Dafür werden %f mm benötigt" % sa
                assert(0)

        #
        # Compute distance to accel from start speed to nominal speed:
        #

        ta = 0.0
        sa = 0.0

        deltaStartSpeedS = move.topSpeed.speed().feedrate - startSpeedS

        if deltaStartSpeedS:

            ta = deltaStartSpeedS / allowedAccel
            # print "accel time (for %f mm/s): %f [s]" % (deltaStartSpeedS, ta)

            # debug Check axxis acceleration
            deltaSpeedV = move.direction5.scale(deltaStartSpeedS)
            for dim in range(5):
                dimAccel = abs(deltaSpeedV[dim]) / ta
                if (dimAccel / MAX_AXIS_ACCELERATION_NOADV[dim]) > 1.001:
                    print "dim %d verletzt max accel: " % dim, dimAccel, " > ", MAX_AXIS_ACCELERATION_NOADV[dim]
                    assert(0)
            #end debug

            sa = util.accelDist(startSpeedS, allowedAccel, ta)

        #
        # Compute distance to decel from nominal speed to endspeed:
        #
        tb = 0.0
        sb = 0.0

        deltaEndSpeedS = move.topSpeed.speed().feedrate - endSpeedS                          # [mm/s]

        if deltaEndSpeedS:

            tb = deltaEndSpeedS / allowedAccel                          # [s]
            # print "decel time (for %f mm/s): %f [s]" % (deltaEndSpeedS, tb)

            # debug Check axxis acceleration
            deltaSpeedV = move.direction5.scale(deltaEndSpeedS)
            for dim in range(5):
                dimDecel = abs(deltaSpeedV[dim]) / tb  
                if (dimDecel / MAX_AXIS_ACCELERATION_NOADV[dim]) > 1.001:
                    print "dim %d verletzt max accel: " % dim, dimDecel, " [mm/s] > ", MAX_AXIS_ACCELERATION_NOADV[dim], " [mm/s]"
                    assert(0)
            # end debug

            sb = util.accelDist(endSpeedS, allowedAccel, tb)

        # print "e_distance: %f, sbeschl, sbrems: %f, %f" % (move.e_distance, sa, sb)

        if move.distance5 < (sa+sb):

            #
            # Strecke zu kurz, Trapez nicht möglich, geschwindigkeit muss abgesenkt werden.
            #
            if debugMoves:
                print "Trapez nicht möglich: s: %f, sbeschl (%f) + sbrems (%f) = %f" % (move.distance5, sa, sb, sa+sb)

            # ??? 
            assert(sa>0 and sb>0)

            topSpeed = move.topSpeed.speed()

            sa = (2 * allowedAccel * move.distance5 - pow(startSpeedS, 2) + pow(endSpeedS, 2)) /(4 * allowedAccel)
            sb = move.distance5 - sa

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

            deltaSpeedS = v - startSpeedS                          # [mm/s]

            # Handle rounding errors
            if deltaSpeedS < 0:
                assert(deltaSpeedS > -0.000001)
                deltaSpeedS = 0
                v = startSpeedS

            ta = deltaSpeedS / allowedAccel
            print "ta: ", ta, deltaSpeedS

            deltaSpeedS = v - endSpeedS                          # [mm/s]

            # Handle rounding errors
            if deltaSpeedS < 0:
                assert(deltaSpeedS > -0.000001)
                deltaSpeedS = 0
                v = endSpeedS

            tb = deltaSpeedS / allowedAccel
            print "tb: ", tb, deltaSpeedS

            topSpeed.feedrate = v
            move.topSpeed.setSpeed(topSpeed, "planTravelAcceleration - max reachable topspeed")

            move.setDuration(ta, 0, tb)

            if debugMoves:
                move.pprint("planTravelAcceleration")
                print 
                print "***** End planTravelAcceleration() *****"

            return

        # 
        # Strecke reicht aus, um auf nominal speed zu beschleunigen
        # 

        # print "ta: ", ta, deltaStartSpeedS, sa
        # print "tb: ", tb, deltaEndSpeedS, sb

        nominalSpeed = move.topSpeed.speed().feedrate # [mm/s]
        slin = move.distance5 - (sa+sb)
        tlin = slin / nominalSpeed
        # print "tlin: ", tlin, slin
        move.setDuration(ta, tlin, tb)

        if debugMoves:
            move.pprint("planTravelAcceleration")
            print 
            print "***** End planTravelAcceleration() *****"

    def packAxesBits(self, bitList):

        bits = 0
        for i in range(5):
            bits += bitList[i] << i
        return bits

    def planTravelSteps(self, move):

        if debugMoves:
            print "***** Start planTravelSteps() *****"
            move.pprint("planTravelSTeps:")

        move.state = 3

        move.initStepData(StepDataTypeBresenham)

        print "Warning, disabled extrusion adjust in planTravelSteps!"

        """
        for i in range(5):
            dirBits += (move.direction[i] >= 0) << i
            adjustedDisplacement = move.displacement_vector_steps_adjusted(NozzleProfile, MatProfile, PrinterProfile)

            s = abs(adjustedDisplacement[i])
            if s > leadAxis_steps:
                leadAxis = i
                leadAxis_steps = s

            abs_displacement_vector_steps.append(s)
        """

        # Round step values
        dispF = move.displacement_vector_steps_raw5
        dispS = self.stepRounders.round(dispF)

        if dispS == emptyVector5:

            print "Empty move..."
            print "***** End planTravelSteps() *****"
            self.stepRounders.rollback()
            return

        self.stepRounders.commit()

        abs_displacement_vector_steps = vectorAbs(dispS)

        # Determine the 'lead axis' - the axis with the most steps
        leadAxis = move.leadAxis(disp=dispS)
        leadAxis_steps = abs_displacement_vector_steps[leadAxis]

        dirBits = util.directionBits(dispS, self.printer.curDirBits)

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
        nominalSpeed = abs( move.topSpeed.speed().vv()[leadAxis] ) # [mm/s]

        allowedAccel = move.getMaxAllowedAccelVectorNoAdv5()[leadAxis]

        v0 = abs(move.startSpeed.speed()[leadAxis])                # [mm/s]

        nominalSpeed = abs( move.topSpeed.speed()[leadAxis] ) # [mm/s]

        v1 = abs(move.endSpeed.speed()[leadAxis])                # [mm/s]

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
        # print "# linear steps:", nLin

        if nLin > 0:

            steps_per_second_nominal = nominalSpeed * steps_per_mm
            timerValue = fTimer / steps_per_second_nominal
            move.stepData.setLinTimer(timerValue)

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

                    assert(len(move.stepData.accelPulses)+len(move.stepData.decelPulses) == leadAxis_steps)

                elif nAccel:
                    del move.stepData.accelPulses[:-nLin]
                    assert(len(move.stepData.accelPulses) == leadAxis_steps)
                else:
                    del move.stepData.decelPulses[nLin:]
                    assert(len(move.stepData.decelPulses) == leadAxis_steps)

            move.stepData.setLinTimer(0xffff)

        # timerValue = fTimer / steps_per_second_nominal
        # move.stepData.setLinTimer(timerValue)

        if debugMoves:
            print "# of steps for move: ", leadAxis_steps
            move.pprint("move:")
            print 

        move.stepData.checkLen(leadAxis_steps)

        if debugMoves:
            print "***** End planTravelSteps() *****"


