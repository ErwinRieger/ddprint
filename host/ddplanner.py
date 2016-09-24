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

import math, collections

import ddprintutil as util, dddumbui, packedvalue
from ddprofile import PrinterProfile, MatProfile, NozzleProfile
from move import Vector # , Move
from ddprintconstants import *
from ddconfig import *
from ddprintutil import Z_AXIS, vectorMul, circaf
from ddprinter import Printer
from ddprintcommands import CmdSyncTargetTemp
from ddprintstates import HeaterEx1, HeaterBed
from ddadvance import Advance

#####################################################################
#
# "AutoTemp" constants
#
# Don't adjust hotend temperature on every move, collect moves for
# ATInterval time and compute/set the temp for that interval.
#
ATInterval = 5 # [s]
# ATMaxTempIncrease = 50

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
        reachedESpeed = abs( move.getReachedFeedrateV()[A_AXIS] ) # [mm/s]
        reachedEExtrusion = reachedESpeed * MatProfile.getMatArea()

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

class PathData (object):

    def __init__(self):

        self.path = []
        self.count = -10

        # AutoTemp
        if UseExtrusionAutoTemp:

            # Time needed to complete the moves, extruding moves only
            self.time = 0
            # Head move distance sum or extrusion volume, extruding moves only
            self.extrusionAmount = 0
            self.lastTemp = 0

        # Moves collected in the ATInterval
        self.atMoves = []

        # Some statistics
        self.maxExtrusionRate = MaxExtrusionRate()

    # Number of moves
    def incCount(self):
        self.count += 10 # leave space for advance-submoves
        return self.count

#####################################################################
"""
class LayerMoves (object):

    def __init__(self):

        self.height = 0.0
        self.path = []

        xxx
        self.count = -1

        # AutoTemp
        if UseExtrusionAutoTemp:

            # Time needed to complete the moves, extruding moves only
            self.time = 0
            # Head move distance sum or extrusion volume, extruding moves only
            self.extrusionAmount = 0
            self.lastTemp = 0

        # Moves collected in the ATInterval
        self.atMoves = []

        # Some statistics
        self.maxExtrusionRate = MaxExtrusionRate()
        xxx

    # # Number of moves
    # def incCount(self):
        # self.count += 1
        # return self.count
"""
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

        # Lowest allowed speed in mm/s for every dimension
        # xxx used? self.min_speeds = 5 * [0]
        # xxx used? for dim in range(5):
            # xxx used? # vmin = (fTimer / maxTimerValue) / steps_per_mm
            # xxx used? if PrinterProfile.getStepsPerMM(dim):
                # xxx used? self.min_speeds[dim] = float(fTimer) / (maxTimerValue24 * PrinterProfile.getStepsPerMM(dim))

        # xxx used? self.gui.log( "min speeds: ", self.min_speeds)

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

        # self.plotfile = None

        # Headspeed/extrusionspeed where autotemp increase starts
        self.ExtrusionAmountLow = 30 # [mm/s] for a 1mm nozzle
        if UseExtrusionAutoTemp:
            # self.ExtrusionAmountLow = 7.5 # [mm³/s] for a 1mm nozzle
            area04 = pow(0.4, 2)*math.pi/4
            self.ExtrusionAmountLow = MatProfile.getBaseExtrusionRate() * (NozzleProfile.getArea() / area04)

        self.advance = Advance(self)

        self.reset()

    def reset(self):

        self.pathData = PathData()
        # self.layerMoves = LayerMoves()

        self.syncCommands = collections.defaultdict(list)
        self.partNumber = 1

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

    def addSynchronizedCommand(self, command, p1=None, p2=None):
        self.syncCommands[self.pathData.count+1].append((command, p1, p2))

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

    # Called from gcode parser
    def addMove(self, move):

        move.moveNumber = self.pathData.incCount()

        # print "addmove ...", move.comment
        if debugMoves:
            print "***** Start addMove() *****"
            move.pprint("AddMove")

        if not move.isPrintMove():

            if self.pathData.path:
                #
                # Finisch preceding printmoves.
                #
                self.advance.planPath(self.pathData.path)
                self.pathData.path = []

            #
            # Do a simple path planning for traveling moves:
            # * start/stop at jerk/2
            # * don't join moves 
            # * don't do advance
            #
            self.planTravelMove(move)

            if debugMoves:
                print "***** End addMove() *****"
            return

        if self.pathData.path:
            prevMove = self.pathData.path[-1]
            prevMove.nextMove = move
            move.prevMove = prevMove

        self.pathData.path.append(move)

        if debugMoves:
            print "***** End addMove() *****"
        

    def planTravelMove(self, move):

        # skip moves with very small e-steps:
        if move.displacement_vector_steps_raw[:3] == [0.0, 0.0, 0.0] and abs(move.displacement_vector_steps_raw[A_AXIS]) < 1:
            print "planTravelMove: skipping small move...", move.displacement_vector_steps_raw
            return

        move.state = 1

        move.setPlannedJerkStartSpeed(self.jerk.scale(0.5))
        move.setPlannedJerkEndSpeed(self.jerk.scale(0.5))

        move.sanityCheck(self.jerk)

        self.planTravelAcceleration(move)

        move.sanityCheck(self.jerk)

        self.planTravelSteps(move)

        if debugMoves:
            print "Streaming travel-move:", move.moveNumber

        self.streamMove(move)

        move.streamed = True # xxx remove

    def isIsolationMove(self, prevMove):

        # 
        # Alle vorgänger-moves können weiterverarbeitet werden, falls prevMove (nicht move, der ist noch nicht
        # fertig geplant) die folgenden bedingung erfüllt:
        # 
        #  Es kann innerhalb des moves von der startgeschwindigkeit auf Jerk/2 (xxx 0) gebremst werden. In diesem fall
        #  wird dann eine verringerung der endgeschwindigkeit von Miso KEINE auswirkung mehr auf die
        #  startgeschwindigkeit von Miso (und dadurch evtl auf seine vorgänger) haben.
        # 
        # 

        allowedAccel = prevMove.getAllowedAccel()
        startSpeedS = prevMove.getStartFr()

        #
        # Abbremsen auf 0 ist zwar hartes kriterium, aber das beeinflusst ja nicht
        # die erreichten geschwindigkeiten, sonder nur das segmentieren und streamen
        # der moves.
        #
        endSpeedS = 0 # min(self.jerk.mul(0.5).len3(), prevMove.getEndFr())

        deltaSpeedS = endSpeedS - startSpeedS

        # print "Start, End, delta:", startSpeedS, endSpeedS, deltaSpeedS

        if abs(deltaSpeedS) > 0.001:

            if deltaSpeedS > 0:
                # beschleunigung, gültiger iso-move
                return True

            # bremsen
            minEndSpeed = util.vAccelPerDist(startSpeedS, -allowedAccel, prevMove.distance)

            # print "minEndSpeed:", minEndSpeed

            if minEndSpeed > endSpeedS:
                # es kann nicht gebremst werden
                return False

            # es kann auf jeden fall gebremst werden
            return True

        # ebenfalls iso-move, da start geschwindigkeit schon so niedrig ist.
        return True

    def finishMoves(self):

        if self.pathData.path:

            self.advance.planPath(self.pathData.path)
            self.pathData.path = []

            # if debugPlot:
                # self.plotfile.write("e\n")
                # self.plotfile.close()

            print "\nStatistics:"
            print "-----------"
            self.pathData.maxExtrusionRate.printStat()
            return

        print "finishMoves: nothing to do..."

    def streamMoves(self, moves, finish = False):

        if debugMoves:
            print "Streaming %d moves..." % len(moves)

        if debugPlot and not self.plotfile:
            self.plottime = 0
            self.plotfile=open("/tmp/accel_%d.plt" % moves[0].moveNumber, "w")
            self.plotfile.write("set grid\n")
            self.plotfile.write("set label 2 at  graph 0.01, graph 0.95 'Note: Trapez top not included, e-only moves green.'\n")
            self.plotfile.write('plot "-" using 1:2:3 with linespoints lc variable\n')

        #################################################################################
        # Backwards move planning
        # self.joinMovesBwd(moves)
        #################################################################################

        for move in moves:

            # move.pprint("sanicheck")
            move.sanityCheck(self.jerk)

            self.xlanAcceleration(move)
            self.xlanSteps(move)

            #
            # Collect some statistics
            #
            if move.isPrintMove():
                self.pathData.maxExtrusionRate.stat(move)

            #
            # Collect moves if AutoTemp
            #
            if UseExtrusionAutoTemp:
                if move.isExtrudingMove(A_AXIS):
                    # Collect moves and sum up path time
                    self.pathData.time += move.getTime()

                    # Sum extrusion volume
                    self.pathData.extrusionAmount += move.getAdjustedExtrusionVolume(A_AXIS, NozzleProfile, MatProfile)

                self.pathData.atMoves.append(move)

            else:

                self.streamMove(move)

            move.streamed = True

            # Help garbage collection
            move.prevMove = errorMove
            move.nextMove = errorMove
        
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

    def streamMove(self, move):

        if self.args.mode != "pre":

            if move.moveNumber in self.syncCommands:
                for (cmd, p1, p2) in self.syncCommands[move.moveNumber]:
                    self.printer.sendCommandParamV(cmd, [p1, p2])

            for (cmd, cobsBlock) in move.commands():
                self.printer.sendCommandC(cmd, cobsBlock)

    #
    # Letzter move in revMoves ist entweder:
    #   * letzer move des pfades, in dem fall ist endspeed bereits berechnet
    #     und gleich jerkspeed.
    #   * der move vor dem "isolation move", in diesem falle ist endspeed noch nicht 
    #     gesetzt, aber die min/max grenzen
    #
    def old_joinMovesBwd(self, moves):

        if debugMoves:
            print "***** Start old_joinMovesBwd() *****"

        index = len(moves) - 1
        # index = len(moves) - 2
        # for move in revMoves[:-1]:
        while index >= 0:

            move = moves[index]
            index -= 1

            if debugMoves: 
                move.pprint("old_joinMovesBwd")

            # Check, if breaking between startspeed and endspeed of
            # move is possible with the given acceleration and within the 
            # given distance:

            endSpeedS = move.getEndFr()
            allowedAccel = move.getAllowedAccel()

            maxAllowedStartSpeed = util.vAccelPerDist(endSpeedS, allowedAccel, move.distance)

            if maxAllowedStartSpeed >= move.getStartFr():
                # good, move is ok
                continue

            if debugMoves: 
                print "Startspeed of %.5f is to high to reach the desired endspeed." % move.getStartFr()
                print "Max. allowed startspeed: %.5f." % maxAllowedStartSpeed

            if move.prevMove:

                #
                # Adjust endspeed of the previous move, also.
                #

                factor = maxAllowedStartSpeed / move.getStartFr()
                # print "factor: ", factor
                # Adjust endspeed of last move:
          
                assert(factor < 1)

                if move.prevMove.streamed:
                    move.prevMove.pprint("error-streamed")

                assert( not move.prevMove.streamed )

                # XXX einfacher algo, kann man das besser machen (z.b. mit jerk-berechnung,
                # vector subtraktion oder so?)
                move.prevMove.setTrueEndFr(move.prevMove.getEndFr() * factor)

            # Adjust startspeed of this move:
            move.setTrueStartFr(maxAllowedStartSpeed)

        if debugMoves:
            print "***** End old_joinMovesBwd() *****"

    def planTravelAcceleration(self, move):

        if debugMoves: 
            print "***** Start planTravelAcceleration() *****"
            move.pprint("planTravelAcceleration")

        move.state = 2

        allowedAccel = allowedDeccel = move.getMaxAllowedAccelNoAdv()

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
                if (dimAccel / MAX_AXIS_ACCELERATION_NOADV[dim]) > 1.001:
                    print "dim %d verletzt max accel: " % dim, dimAccel, " > ", MAX_AXIS_ACCELERATION_NOADV[dim]
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
            print "deccel time (for %f mm/s): %f [s]" % (deltaEndSpeedS, tb)

            # debug Check axxis acceleration
            deltaSpeedV = move.direction.scale(deltaEndSpeedS)
            for dim in range(5):
                dimDeccel = abs(deltaSpeedV[dim]) / tb  
                if (dimDeccel / MAX_AXIS_ACCELERATION_NOADV[dim]) > 1.001:
                    print "dim %d verletzt max accel: " % dim, dimDeccel, " [mm/s] > ", MAX_AXIS_ACCELERATION_NOADV[dim], " [mm/s]"
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

            topSpeed = move.topSpeed.plannedSpeed()

            # sa = (2 * allowedAccel * move.e_distance - pow(startSpeedS, 2) + pow(endSpeedS, 2)) /(4 * allowedAccel)
            sa = (2 * allowedAccel * move.distance - pow(startSpeedS, 2) + pow(endSpeedS, 2)) /(4 * allowedAccel)
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

            move.topSpeed.plannedSpeed(topSpeed)

            deltaSpeedS = v - startSpeedS                          # [mm/s]
            ta = deltaSpeedS / allowedAccel
            # print "ta: ", ta, deltaSpeedS

            deltaSpeedS = v - endSpeedS                          # [mm/s]
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

        nominalSpeed = move.topSpeed.plannedSpeed().feedrate # [mm/s]
        slin = move.distance - (sa+sb)
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

        dirBits = 0
        abs_displacement_vector_steps = []

        # Determine the 'lead axis' - the axis with the most steps
        leadAxis = 0
        leadAxis_steps = 0

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

        disp = move.displacement_vector_steps_raw
        disp[A_AXIS] = int(disp[A_AXIS]) # xxx round float e-steps

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
        nominalSpeed = abs( move.topSpeed.trueSpeed().vv()[leadAxis] ) # [mm/s]
        reachedSpeedFr = move.topSpeed.trueSpeed().feedrate

        accel = move.getMaxAllowedAccelVectorNoAdv()[leadAxis]
        accel_steps_per_square_second = accel * steps_per_mm

        v0 = abs(move.startSpeed.trueSpeed().vv()[leadAxis])                # [mm/s]

        v1 = abs(move.endSpeed.trueSpeed().vv()[leadAxis])                # [mm/s]

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

        # if debugPlot:
            # accelPlotData = []
            # deccelPlotData = []

        # if not circaf(move.getStartFr(), reachedSpeedFr, 0.1) and not circaf(move.getEndFr(), reachedSpeedFr, 0.1):
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

                    # if debugPlot:
                        # if move.eOnly:
                            # accelPlotData.append((steps_per_second_accel/steps_per_mm, 2, dt))
                        # else:
                            # accelPlotData.append((steps_per_second_accel/steps_per_mm, 1, dt))

                    tAccel += dt

                    if timerValue > maxTimerValue24:
                        move.pprint("planTravelSTeps:")
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

                    # if debugPlot:
                        # if move.eOnly:
                            # deccelPlotData.append((steps_per_second_deccel/steps_per_mm, 2, dt))
                        # else:
                            # deccelPlotData.append((steps_per_second_deccel/steps_per_mm, 1, dt))

                    tDeccel += dt

                    if timerValue > maxTimerValue24:
                        move.pprint("planTravelSTeps:")
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

                if debugPlot:
                    if move.eOnly:
                        accelPlotData.append((steps_per_second_accel/steps_per_mm, 2, dt))
                    else:
                        accelPlotData.append((steps_per_second_accel/steps_per_mm, 1, dt))

                tAccel += dt

                if timerValue > maxTimerValue24:
                    move.pprint("planTravelSTeps:")
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

                if debugPlot:
                    if move.eOnly:
                        deccelPlotData.append((steps_per_second_deccel/steps_per_mm, 2, dt))
                    else:
                        deccelPlotData.append((steps_per_second_deccel/steps_per_mm, 1, dt))

                tDeccel += dt

                if timerValue > maxTimerValue24:
                    move.pprint("planTravelSTeps:")
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

        # if debugPlot:
            # self.plotfile.write("# Acceleration:\n")
            # for (speed, color, dt) in accelPlotData:
                # self.plotfile.write("%f %f %d\n" % (self.plottime, speed, color))
                # self.plottime += dt

            # self.plotfile.write("# Linear top:\n")
            # self.plotfile.write("%f %f 0\n" % (self.plottime, steps_per_second_nominal/steps_per_mm))
            # self.plottime += timerValue / fTimer

            # self.plotfile.write("# Decceleration:\n")
            # deccelPlotData.reverse()
            # for (speed, color, dt) in deccelPlotData:
                # self.plotfile.write("%f %f %d\n" % (self.plottime, speed, color))
                # self.plottime += dt

        if debugMoves:
            print "# of steps for move: ", leadAxis_steps
            move.pprint("move:")
            print 

        move.stepData.checkLen(leadAxis_steps)

        if debugMoves:
            print "***** End planTravelSteps() *****"


