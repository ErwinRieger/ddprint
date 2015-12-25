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
from move import VVector, Move

from ddprintconstants import fTimer, dimNames, maxTimerValue24, DEFAULT_MAX_ACCELERATION, DropSegments
from ddprintutil import Z_AXIS, debugMoves, vectorMul, circaf
from ddprinter import Printer
from ddprintcommands import CmdGetEepromSettings, CmdSyncTargetTemp
from ddprintstates import HeaterEx1, HeaterBed

debugPlot = True
debugPlot = False

debugAutoTemp = False
debugAutoTemp = True

# Debug object
class StreamedMove:
        pass
errorMove = StreamedMove()

#####################################################################
#
# "AutoTemp" constants
#
# Don't adjust hotend temperature on every move, collect moves for
# ATInterval time and compute/set the temp for that interval.
#
ATInterval = 5 # [s]
# ATMaxTempIncrease = 50

# UseAutoTemp = True
UseAutoTemp = False
UseExtrusionAutoTemp = True

# Headspeed/extrusionspeed where autotemp increase starts
ExtrusionAmountLow = 30 # [mm/s] for a 1mm nozzle
if UseExtrusionAutoTemp:
    ExtrusionAmountLow = 7.5 # [mm³/s] for a 1mm nozzle

#####################################################################
#
# Auto extrusion adjust
#
UseExtrusionAdjust = True

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

        # Get maximum extrusion rate, take plateau speed into account only
        # length of max constant speed:
        reachedESpeed = abs( move.getReachedSpeedV()[util.A_AXIS] ) # [mm/s]
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

        if self.maxRate:
            print "Maximal Extrusion Rate (Extruder A): %.1f" % self.maxRate, "mm³/s, move: ",
            self.move.pprint("Max. extrusion Move")
            print "Max10: ", self.max10
            print "Maximal Extrusion Rate (Extruder A) 5 second average: %.1f" % self.maxAvgRate, "mm³/s\n"

#####################################################################

class PathData (object):

    def __init__(self):

        self.path = []
        self.count = -1

        # AutoTemp
        if UseAutoTemp or UseExtrusionAutoTemp:

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
        self.count += 1
        return self.count

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

        self.jerk = VVector(jerk)
        self.gui.log("Jerk vector: ", self.jerk)

        self.zeroPos = util.MyPoint()

        # Lowest allowed speed in mm/s for every dimension
        self.min_speeds = 5 * [0]
        for dim in range(5):
            # vmin = (fTimer / maxTimerValue) / steps_per_mm
            if PrinterProfile.getStepsPerMM(dim):
                self.min_speeds[dim] = float(fTimer) / (maxTimerValue24 * PrinterProfile.getStepsPerMM(dim))

        self.gui.log( "min speeds: ", self.min_speeds)

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

        self.reset()

    def reset(self):

        self.pathData = PathData()

        self.syncCommands = collections.defaultdict(list)
        self.partNumber = 1


    @classmethod
    def get(cls):
        return cls.__single

    def getHomePos(self):

        # Get additional z-offset from eeprom
        add_homeing_z = self.printer.query(CmdGetEepromSettings)['add_homeing'][Z_AXIS]

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

    def addSynchronizedCommand(self, command, p1=None, p2=None, wantReply=None):
        self.syncCommands[self.pathData.count+1].append((command, p1, p2, wantReply))

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
        """
        Queue a position: this function chooses the correct movement command based on the print_to_file_type
        Moves to a certain position over a given duration with either relative or absolute positioning. 
        Relative vs. Absolute positioning is done on an axis to axis basis.
        @param list position: A 5 dimentional position in steps specifying where each axis should move to
        @param float e_distance: distance in millimeters moved in (x,y,z) space OR if distance(x,y,z) == 0, then max(distance(A),distance(B))
        @param float feedrate_mm_sec: the actual feedrate in units of millimeters/second
        @param list relative_axes: Array of axes whose coordinates should be considered relative
        """

        # print "addmove ...", move.comment
        if debugMoves:
            print "***** Start addMove() *****"

        move.moveNumber = self.pathData.incCount()

        if not self.pathData.path:

            # First move of this path, startspeed is jerkspeed
            move.typ = Move.StartMove

            # jerkSpeed = move.vVector().constrain(self.jerk) or move.vVector()
            # move.setNominalStartFr(jerkSpeed)

            move.setNominalJerkStartSpeed(self.jerk)

            # self.prepareMoveEnd(move)
            self.pathData.path.append(move)

            if UseAutoTemp or UseExtrusionAutoTemp:
                self.pathData.time = 0 # Reset path time
                self.pathData.extrusionAmount = 0
                self.pathData.lastTemp = MatProfile.getHotendBaseTemp()

            #
            # Zum end-speed können wir nichts sagen, da der nächste move noch nicht
            # bekannt ist.
            #
            if debugMoves:
                print "***** End addMove() *****"
            return

        lastMove = self.pathData.path[-1]
        lastMove.nextMove = move
        move.lastMove = lastMove

        move.typ = Move.NormalMove

        util.joinSpeed(lastMove, move, self.jerk, self.min_speeds)

        self.pathData.path.append(move)

        if len(self.pathData.path) < 3:
            # Wir brauchen mind. 3 moves um einen 'PathBlock' zu bilden
            if debugMoves:
                print "***** End addMove() *****"
            return

        if self.isIsolationMove(lastMove):

            l = len(self.pathData.path) 
            pathBlock = self.pathData.path[:l-2]

            if debugMoves:
                print "Move #:", lastMove.moveNumber, " is a isolation move."
                print "Streaming block of moves, len: %d/%d" % (len(pathBlock), l), ", blocks: ", pathBlock[0].moveNumber, " - ", pathBlock[-1].moveNumber

            self.streamMoves(pathBlock)
            del self.pathData.path[:l-2]
            assert(len(self.pathData.path) == 2)

        if debugMoves:
            print "***** End addMove() *****"

    def isIsolationMove(self, lastMove):

        # 
        # Alle vorgänger-moves können weiterverarbeitet werden, falls lastMove (nicht move, der ist noch nicht
        # fertig geplant) die folgenden bedingung erfüllt:
        # 
        #  Es kann innerhalb des moves von der startgeschwindigkeit auf Jerk/2 (xxx 0) gebremst werden. In diesem fall
        #  wird dann eine verringerung der endgeschwindigkeit von Miso KEINE auswirkung mehr auf die
        #  startgeschwindigkeit von Miso (und dadurch evtl auf seine vorgänger) haben.
        # 
        # 

        allowedAccel = lastMove.getAllowedAccel()
        startSpeedS = lastMove.getStartFr()

        #
        # Abbremsen auf 0 ist zwar hartes kriterium, aber das beeinflusst ja nicht
        # die erreichten geschwindigkeiten, sonder nur das segmentieren und streamen
        # der moves.
        #
        endSpeedS = 0 # min(self.jerk.mul(0.5).len3(), lastMove.getEndFr())

        deltaSpeedS = endSpeedS - startSpeedS

        # print "Start, End, delta:", startSpeedS, endSpeedS, deltaSpeedS

        if abs(deltaSpeedS) > 0.001:

            if deltaSpeedS > 0:
                # beschleunigung, gültiger iso-move
                return True

            # bremsen
            # minEndSpeed = util.vAccelPerDist(startSpeedS, -allowedAccel, lastMove.e_distance)
            # minEndSpeed = util.vAccelPerDist(startSpeedS, -allowedAccel, lastMove.move_distance)
            minEndSpeed = util.vAccelPerDist(startSpeedS, -allowedAccel, lastMove.distance) # distchange

            # print "minEndSpeed:", minEndSpeed

            if minEndSpeed > endSpeedS:
                # es kann nicht gebremst werden
                return False

            # es kann auf jeden fall gebremst werden
            return True

        # ebenfalls iso-move, da start geschwindigkeit schon so niedrig ist.
        return True

    def finishMoves(self):

        # xxx set endspeed
        # xxx check accel

        if self.pathData.path:

            # End path
            move = self.pathData.path[-1]
            move.state = 1
            move.typ |= Move.EndMove

            # Finish end move
            # jerkSpeed = move.vVector().constrain(self.jerk) or move.vVector()
            # move.setNominalEndFr(jerkSpeed)
            move.setNominalJerkEndSpeed(self.jerk)

            if debugMoves:
                print "Streaming last block of moves, len: ", len(self.pathData.path), ", blocks: ", self.pathData.path[0].moveNumber, " - ", move.moveNumber

            self.streamMoves(self.pathData.path, True)
            self.pathData.path = []

            if debugPlot:
                self.plotfile.write("e\n")
                self.plotfile.close()

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
        self.joinMovesBwd(moves)
        #################################################################################

        for move in moves:

            # move.pprint("sanicheck")
            move.sanityCheck(self.jerk)

            self.planAcceleration(move)

            #
            # Auto adjust extrusion rate
            #
            if move.isExtrudingMove(util.A_AXIS):
                move.adjustExtrusion()

            self.planSteps(move)

            #
            # Collect some statistics
            #
            if move.isHeadMove():
                self.pathData.maxExtrusionRate.stat(move)

            #
            # Collect moves if AutoTemp
            #
            if UseAutoTemp or UseExtrusionAutoTemp:
                if move.isExtrudingMove(util.A_AXIS):
                    # Collect moves and sum up path time
                    self.pathData.time += move.getTime()

                    if UseAutoTemp:
                        # Sum up distance
                        self.pathData.extrusionAmount += move.move_distance
                    else:
                        # Sum extrusion volume
                        self.pathData.extrusionAmount += move.getExtrusionVolume(util.A_AXIS, MatProfile.getMatArea())

                self.pathData.atMoves.append(move)

            else:

                self.streamMove(move)

            move.streamed = True

            # Help garbage collection
            move.lastMove = errorMove
            move.nextMove = errorMove
        
        if UseAutoTemp or UseExtrusionAutoTemp:

            if self.pathData.time >= ATInterval or finish:

                if self.pathData.time > 0:

                    # Compute temperature for this segment and add tempcommand into the stream
                    # Average speed:
                    avgSpeed = self.pathData.extrusionAmount / self.pathData.time

                    # UseAutoTemp: Adjust temp between Tbase and HotendMaxTemp, if speed is greater than 20 mm/s
                    # UseExtrusionAutoTemp: Adjust temp between Tbase and HotendMaxTemp, if speed is greater than 5 mm³/s
                    newTemp = MatProfile.getHotendBaseTemp() # Extruder 1 temp
                    extrusionLow = ExtrusionAmountLow * pow(NozzleProfile.getSize(), 2)
                    if avgSpeed > extrusionLow:
                        f = NozzleProfile.getAutoTempFactor(UseExtrusionAutoTemp)
                        # newTemp += min((avgSpeed - ExtrusionAmountLow * pow(NozzleProfile.getSize(), 2)) * f, ATMaxTempIncrease)
                        newTemp += (avgSpeed - extrusionLow) * f
                        newTemp = min(newTemp, MatProfile.getHotendMaxTemp())

                    if debugAutoTemp:
                        print "AutoTemp: collected %d moves with %.2f s duration." % (len(self.pathData.atMoves), self.pathData.time)
                        print "AutoTemp: amount: %.2f, avg extrusion rate: %.2f mm³/s." % (self.pathData.extrusionAmount, avgSpeed)
                        print "AutoTemp: new temp: %.2f." % (newTemp)
                        self.pathData.maxExtrusionRate.avgStat(avgSpeed)

                    if newTemp != self.pathData.lastTemp and self.args.mode != "pre":

                        # Schedule target temp command
                        self.printer.sendCommandParam(
                            CmdSyncTargetTemp,
                            p1=packedvalue.uint8_t(HeaterEx1),
                            p2=packedvalue.uint16_t(newTemp),
                            wantReply=None)

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
                for (cmd, p1, p2, reply) in self.syncCommands[move.moveNumber]:
                    self.printer.sendCommandParam(cmd, p1=p1, p2=p2, wantReply=reply)

            for (cmd, payload) in move.commands():
                self.printer.sendBinaryCommand(cmd, binPayload=payload)

    #
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
        # index = len(moves) - 2
        # for move in revMoves[:-1]:
        while index >= 0:

            move = moves[index]
            index -= 1

            if debugMoves: 
                move.pprint("joinMovesBwd")

            # Check, if breaking between startspeed and endspeed of
            # move is possible with the given acceleration and within the 
            # given distance:

            endSpeedS = move.getEndFr()
            allowedAccel = move.getAllowedAccel()

            # maxAllowedStartSpeed = util.vAccelPerDist(endSpeedS, allowedAccel, move.e_distance)
            # maxAllowedStartSpeed = util.vAccelPerDist(endSpeedS, allowedAccel, move.move_distance)
            maxAllowedStartSpeed = util.vAccelPerDist(endSpeedS, allowedAccel, move.distance) # distchange

            if maxAllowedStartSpeed >= move.getStartFr():
                # good, move is ok
                continue

            if debugMoves: 
                print "Startspeed of %.5f is to high to reach the desired endspeed." % move.getStartFr()
                print "Max. allowed startspeed: %.5f." % maxAllowedStartSpeed

            if move.lastMove:

                #
                # Adjust endspeed of the previous move, also.
                #

                factor = maxAllowedStartSpeed / move.getStartFr()
                # print "factor: ", factor
                # Adjust endspeed of last move:
          
                assert(factor < 1)

                if move.lastMove.streamed:
                    move.lastMove.pprint("error-streamed")

                assert( not move.lastMove.streamed )

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

        allowedAccel = allowedDeccel = move.getAllowedAccel()

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
      
            # if (sa - move.move_distance) > 0.001:
            if (sa - move.distance) > 0.001: # distchange
                print " 0.5 * %f * pow(%f, 2) + %f * %f" % (allowedAccel, ta, startSpeedS, ta)
                # print "VStart %f mm/s kann nicht innerhalb von %f mm auf Endgeschwindigkeit %f mm/s gebracht werden!" % (startSpeedS, move.move_distance, endSpeedS)
                print "VStart %f mm/s kann nicht innerhalb von %f mm auf Endgeschwindigkeit %f mm/s gebracht werden!" % (startSpeedS, move.distance, endSpeedS) # distchange
                print "Dafür werden %f mm benötigt" % sa
                assert(0)

        #
        # Compute distance to accel from start speed to nominal speed:
        #

        ta = 0.0
        sa = 0.0
        # if move.getStartFr().fr != None:
        startSpeedS = move.getStartFr()
        deltaStartSpeedS = move.feedrateS - startSpeedS

        if deltaStartSpeedS:

            ta = deltaStartSpeedS / allowedAccel
            # print "accel time (for %f mm/s): %f [s]" % (deltaStartSpeedS, ta)

            # debug Check axxis acceleration
            deltaSpeedV = move.getFeedrateV(deltaStartSpeedS)
            for dim in range(5):
                if deltaSpeedV[dim] != 0:
                    dimAccel = deltaSpeedV[dim] / ta
                    if (dimAccel / DEFAULT_MAX_ACCELERATION[dim]) > 1.001:
                        print "dim %d verletzt max accel: " % dim, dimAccel, " > ", DEFAULT_MAX_ACCELERATION[dim]
                        assert(0)
            #end debug

            sa = util.accelDist(startSpeedS, allowedAccel, ta)

        #
        # Compute distance to deccel from nominal speed to endspeed:
        #
        tb = 0.0
        sb = 0.0
        # if move.getEndFr().fr != None:
        endSpeedS = move.getEndFr() # [mm/s]
        deltaEndSpeedS = move.feedrateS - endSpeedS                          # [mm/s]

        if deltaEndSpeedS:

            tb = deltaEndSpeedS / allowedAccel                          # [s]
            # print "deccel time (for %f mm/s): %f [s]" % (deltaEndSpeedS, tb)

            # debug Check axxis acceleration
            deltaSpeedV = move.getFeedrateV(deltaEndSpeedS)
            for dim in range(5):
                if deltaSpeedV[dim] != 0:
                    dimDeccel = deltaSpeedV[dim] / tb  
                    if (dimDeccel / DEFAULT_MAX_ACCELERATION[dim]) > 1.001:
                        print "dim %d verletzt max accel: " % dim, dimDeccel, " [mm/s] > ", DEFAULT_MAX_ACCELERATION[dim], " [mm/s]"
                        assert(0)
            # end debug

            sb = util.accelDist(endSpeedS, allowedAccel, tb)

        # print "e_distance: %f, sbeschl, sbrems: %f, %f" % (move.e_distance, sa, sb)

        # if move.move_distance < (sa+sb):
        if move.distance < (sa+sb): # distchange

            #
            # Strecke zu kurz, Trapez nicht möglich, geschwindigkeit muss abgesenkt werden.
            #
            if debugMoves:
                print "Trapez nicht möglich: s: %f, sbeschl (%f) + sbrems (%f) = %f" % (move.distance, sa, sb, sa+sb) # distchange

            # ??? 
            assert(sa>0 and sb>0)
            assert(sa>0)

            startSpeedS = move.getStartFr()
            nominalSpeedS = move.feedrateS

            # sa = (2 * allowedAccel * move.e_distance - pow(startSpeedS, 2) + pow(endSpeedS, 2)) /(4 * allowedAccel)
            sa = (2 * allowedAccel * move.distance - pow(startSpeedS, 2) + pow(endSpeedS, 2)) /(4 * allowedAccel) # distchange
            # sb = move.e_distance - sa
            # sb = move.move_distance - sa
            sb = move.distance - sa # distchange

            if debugMoves:
                print "sbeschl, sbrems neu: %f, %f" % (sa, sb)

            # 
            # Geschwindigkeit, die auf strecke sa mit erreicht werden kann
            # 
            v = math.sqrt ( 2 * allowedAccel * sa + pow(startSpeedS, 2) )

            # debug, test
            endSpeedS = move.getEndFr() # [mm/s]
            v2 = math.sqrt ( 2 * allowedAccel * sb + pow(endSpeedS, 2) )
            # print "move.feedrate neu: %f (test: %f, diff: %f)" % (v, v2, abs(v - v2))

            assert( abs(v - v2) < 0.001)

            nominalSpeedV = move.getFeedrateV().scale(v / nominalSpeedS)

            move.accelData.reachedovNominalVVector = nominalSpeedV

            if move.isExtrudingMove(util.A_AXIS):
                # reachedNominalSpeedS = nominalSpeedV.feedrate3() # [mm/s]
                reachedNominalSpeedS = nominalSpeedV.len5() # [mm/s]

                deltaSpeedS = reachedNominalSpeedS - startSpeedS                          # [mm/s]
                ta = deltaSpeedS / allowedAccel
                print "ta: ", ta, deltaSpeedS

                deltaSpeedS = reachedNominalSpeedS - endSpeedS                          # [mm/s]
                tb = deltaSpeedS / allowedAccel
                print "tb: ", tb, deltaSpeedS

                move.setDuration(ta, 0, tb)

        else:

            # 
            # Strecke reicht aus, um auf nominal speed zu beschleunigen
            # 

            if move.isExtrudingMove(util.A_AXIS):
                print "ta: ", ta, deltaStartSpeedS, sa
                print "tb: ", tb, deltaEndSpeedS, sb

                # nominalSpeed = move.getFeedrateV().feedrate3() # [mm/s]
                # nominalSpeed = move.getFeedrateV().len5() # [mm/s]
                nominalSpeed = move.getReachedSpeedV().len5() # [mm/s]
                # slin = move.move_distance - (sa+sb)
                slin = move.distance - (sa+sb)
                tlin = slin / nominalSpeed
                print "tlin: ", tlin, slin
                move.setDuration(ta, tlin, tb)

        if debugMoves:
            move.pprint("planAcceleration")
            print 
            print "***** End planAcceleration() *****"

    def packAxesBits(self, bitList):

        bits = 0
        for i in range(5):
            bits += bitList[i] << i
        return bits

    def planSteps(self, move):

        if debugMoves:
            print "***** Start planSteps() *****"
            move.pprint("PlanSTeps:")

        move.state = 3

        dirBits = 0
        abs_displacement_vector_steps = []

        for i in range(5):
            dirBits += (move.displacement_vector_steps[i] >= 0) << i
            abs_displacement_vector_steps.append(abs(move.displacement_vector_steps[i]))

        if dirBits != self.printer.curDirBits:
            move.stepData.setDirBits = True
            move.stepData.dirBits = dirBits
            self.printer.curDirBits = dirBits

        leadAxis = move.longest_axis
        steps_per_mm = PrinterProfile.getStepsPerMM(leadAxis)

        #
        # Bresenham's variables
        #
        # deltaLead = int(abs(move.displacement_vector_steps[leadAxis]))
        # deltaLead = abs(move.displacement_vector_steps[leadAxis])
        deltaLead = abs_displacement_vector_steps[leadAxis]
        move.stepData.setBresenhamParameters(leadAxis, abs_displacement_vector_steps)

        #
        # Create a list of stepper pulses
        #

        # debnegtimer
        # nominalSpeed = ( (move.accelData.reachedovNominalVVector or move.vVector())[leadAxis]) # [mm/s]
        # nominalSpeed = abs( (move.accelData.reachedovNominalVVector or move.getFeedrateV())[leadAxis]) # [mm/s]
        nominalSpeed = abs( move.getReachedSpeedV()[leadAxis] ) # [mm/s]

        reachedSpeedV = move.getReachedSpeedV()
        reachedSpeedFr = reachedSpeedV.len5()

        # debnegtimer
        accel = abs(move.getAllowedAccelVector()[leadAxis])
        accel_steps_per_square_second = accel * steps_per_mm

        # startSpeed = move.getStartFr()

        # debnegtimer
        # v0 = (startSpeed[leadAxis])                # [mm/s]
        v0 = abs(move.getFeedrateV(move.getStartFr())[leadAxis])                # [mm/s]

        # endSpeed = move.getEndFr()

        # debnegtimer
        # v1 = (endSpeed[leadAxis])                # [mm/s]
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
            self.plotfile.write("# Acceleration:\n")

        if not circaf(move.getStartFr(), reachedSpeedFr, 0.1) and not circaf(move.getEndFr(), reachedSpeedFr, 0.1):

            #
            # Acceleration ramp on both sides.
            #
            # Ramp up both sides in parallel to not exeed available steps
            #

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
                            self.plotfile.write("%f %f 2\n" % (self.plottime, steps_per_second_accel/steps_per_mm))
                        else:
                            self.plotfile.write("%f %f 1\n" % (self.plottime, steps_per_second_accel/steps_per_mm))
                        self.plottime += dt

                    tAccel += dt

                    if timerValue > maxTimerValue24:
                        move.pprint("PlanSTeps:")
                        print "v0: ", dt*1000000, "[uS]", steps_per_second_accel, "[steps/s], timerValue: ", timerValue
                        print "dt: ", dt*1000000, "[uS]", steps_per_second_accel, "[steps/s], timerValue: ", timerValue
                        assert(0)

                    move.stepData.addAccelPulse(timerValue)
                    stepNr += 1
                    done = False

                #
                # Compute ramp down (in reverse), decceleration
                #
                while steps_per_second_deccel < steps_per_second_nominal:

                    #
                    # Compute timer value
                    #
                    steps_per_second_deccel = min(steps_per_second_1 + tDeccel * accel_steps_per_square_second, steps_per_second_nominal)

                    dt = 1.0 / steps_per_second_deccel
                    timerValue = int(fTimer / steps_per_second_deccel)

                    # print "dt: ", dt*1000000, "[uS]", steps_per_second_deccel, "[steps/s], timerValue: ", timerValue, ", v: ", steps_per_second_deccel/steps_per_mm

                    if debugPlot:
                        if move.eOnly:
                            self.plotfile.write("%f %f 2\n" % (self.plottime, steps_per_second_deccel/steps_per_mm))
                        else:
                            self.plotfile.write("%f %f 1\n" % (self.plottime, steps_per_second_deccel/steps_per_mm))
                        self.plottime += dt

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
                        self.plotfile.write("%f %f 2\n" % (self.plottime, steps_per_second_accel/steps_per_mm))
                    else:
                        self.plotfile.write("%f %f 1\n" % (self.plottime, steps_per_second_accel/steps_per_mm))
                    self.plottime += dt

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

                #
                # Compute timer value
                #
                steps_per_second_deccel = min(steps_per_second_1 + tDeccel * accel_steps_per_square_second, steps_per_second_nominal)

                dt = 1.0 / steps_per_second_deccel
                timerValue = int(fTimer / steps_per_second_deccel)

                # print "dt: ", dt*1000000, "[uS]", steps_per_second_deccel, "[steps/s], timerValue: ", timerValue, ", v: ", steps_per_second_deccel/steps_per_mm

                if debugPlot:
                    if move.eOnly:
                        self.plotfile.write("%f %f 2\n" % (self.plottime, steps_per_second_deccel/steps_per_mm))
                    else:
                        self.plotfile.write("%f %f 1\n" % (self.plottime, steps_per_second_deccel/steps_per_mm))
                    self.plottime += dt

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
        print "left linear steps: ", deltaLead-stepNr

        timerValue = fTimer / steps_per_second_nominal
        move.stepData.setLinTimer(timerValue)

        if debugPlot:
            self.plotfile.write("# Linear top:\n")
            self.plotfile.write("%f %f 0\n" % (self.plottime, steps_per_second/steps_per_mm))
            self.plottime += timerValue / fTimer

            self.plotfile.write("# Decceleration:\n")

        return 

        # XXXXXXXXXXXXXX old, unused ...........

        # 
        # Compute acceleration timer values
        # 
        # while v0 and steps_per_second < steps_per_second_nominal:
        # lastTimer = None
        while steps_per_second < steps_per_second_nominal: #  and stepNr < deltaLead:

            assert(stepNr < deltaLead)

            #
            # Compute timer value
            #
            steps_per_second = min(steps_per_second_0 + tAccel * accel_steps_per_square_second, steps_per_second_nominal)

            dt = 1.0 / steps_per_second
            timerValue = int(fTimer / steps_per_second)

            # print "dt: ", dt*1000000, "[uS]", steps_per_second, "[steps/s], timerValue: ", timerValue

            if debugPlot:
                if move.eOnly:
                    self.plotfile.write("%f %f 2\n" % (self.plottime, steps_per_second/steps_per_mm))
                else:
                    self.plotfile.write("%f %f 1\n" % (self.plottime, steps_per_second/steps_per_mm))
                self.plottime += dt

            tAccel += dt

            # debnegtimer
            # if timerValue <= 0:
                # print "dt: ", dt*1000000, "[uS]", steps_per_second, "[steps/s], timerValue: ", timerValue
                # assert(0)
            # debtimeroverflow
            if timerValue > maxTimerValue24:
                move.pprint("PlanSTeps:")
                print "v0: ", dt*1000000, "[uS]", steps_per_second, "[steps/s], timerValue: ", timerValue
                print "dt: ", dt*1000000, "[uS]", steps_per_second, "[steps/s], timerValue: ", timerValue
                assert(0)

            move.stepData.addAccelPulse(timerValue)
            stepNr += 1

            # if lastTimer:
                # assert((lastTimer - timerValue) < maxTimerValue16)
            # lastTimer = timerValue

        # Benutze als timer wert für die lineare phase den letzen timerwert der
        # beschleunigungsphase falls es diese gab. Sonst:
        # berechne timervalue ausgehend von linear feedrate:
        # if not timerValue:
            # timerValue = fTimer / steps_per_second
            # dt = 1.0 / steps_per_second
            # print "dt: ", dt*1000000, "[uS]", steps_per_second, "[steps/s], timerValue: ", timerValue

        timerValue = fTimer / steps_per_second_nominal
        move.stepData.setLinTimer(timerValue)

        if debugPlot:
            self.plotfile.write("# Linear top:\n")
            self.plotfile.write("%f %f 0\n" % (self.plottime, steps_per_second/steps_per_mm))
            self.plottime += timerValue / fTimer

            self.plotfile.write("# Decceleration:\n")

        #
        # Compute ramp down (in reverse), decceleration
        #
        tDeccel = 1.0 / steps_per_second_nominal
        steps_per_second = steps_per_second_nominal
        steps_per_second_1 = v1 * steps_per_mm

        # lastTimer = None
        while steps_per_second > steps_per_second_1: #  and stepNr < deltaLead:

            if stepNr >= deltaLead:
                print "stepNr, deltaLead:", stepNr, deltaLead

            assert(stepNr < deltaLead)

            #
            # Compute timer value
            #
            steps_per_second = max(steps_per_second_nominal - tDeccel * accel_steps_per_square_second, steps_per_second_1)

            dt = 1.0 / steps_per_second
            timerValue = int(fTimer / steps_per_second)

            # print "dt: ", dt*1000000, "[uS]", steps_per_second, "[steps/s], timerValue: ", timerValue, ", v: ", steps_per_second/steps_per_mm

            if debugPlot:
                if move.eOnly:
                    self.plotfile.write("%f %f 2\n" % (self.plottime, steps_per_second/steps_per_mm))
                else:
                    self.plotfile.write("%f %f 1\n" % (self.plottime, steps_per_second/steps_per_mm))
                self.plottime += dt

            tDeccel += dt

            # debnegtimer
            # assert(timerValue > 0)
            if timerValue > maxTimerValue24:
                move.pprint("PlanSTeps:")
                print "v0: ", dt*1000000, "[uS]", steps_per_second, "[steps/s], timerValue: ", timerValue
                print "dt: ", dt*1000000, "[uS]", steps_per_second, "[steps/s], timerValue: ", timerValue
                assert(0)

            move.stepData.addDeccelPulse(timerValue)
            stepNr += 1

            # if lastTimer:
                # assert((timerValue - lastTimer) < maxTimerValue16)
            # lastTimer = timerValue

        if debugMoves:
            print "# of steps for move: ", deltaLead
            move.pprint("move:")
            print 

        move.stepData.checkLen(deltaLead)

        if debugMoves:
            print "***** End planSteps() *****"

