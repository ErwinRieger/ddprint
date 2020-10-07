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

import math, collections, types, pprint
from argparse import Namespace

import ddprintutil as util, dddumbui, packedvalue
# from ddprofile import PrinterProfile, NozzleProfile
from ddprofile import NozzleProfile
from ddvector import Vector, vectorMul, vectorAbs
from ddprintconstants import *
from ddconfig import *
from ddprinter import Printer
from ddprintcommands import CmdSyncTargetTemp, CmdDwellMS, CmdSyncFanSpeed, CmdSuggestPwm
from ddprintstates import HeaterEx1, HeaterBed
from ddadvance import Advance

#####################################################################

if debugPlot:
    import pickle

emptyVector5 = [0] * 5

#####################################################################

class SyncedCommand(object):

    def __init__(self, cmd, duration, params=[]):

        self.duration = duration
        self.cmd = cmd
        self.params = params

    def getTime(self):
        return self.duration

    def isMove(self):
        return False

    def isTemperatureCmd(self):
        return self.cmd == CmdSuggestPwm

    def streamCommand(self, printer):
        printer.sendCommandParamV(self.cmd, self.params)

    def __repr__(self):
        return "SyncedCommand, cmd: %d, time: %f, %s" % (self.cmd, self.duration, str(self.params))

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

#
# Store some move history, used to implement synchronized commands
# and feed forward temperature control.
#
class MoveHistory (object):

    def __init__(self, planner, duration):
        
        self.planner = planner
        self.duration = duration
        self.history = []

        # timespan in history
        self.span = 0.0

    def append(self, move):

        self.history.append(move)
        self.span += move.getTime()

        # Check, if we can stream move(s) at the beginning:
        i = 0

        while i < len(self.history):

            move = self.history[i]

            t = move.getTime()

            if (self.span - t) < self.duration:
                break

            # this move can be streamed to the printer
            self.span -= t
            i += 1

        if i:

            # print "append(): streaming %d moves" % i

            for j in range(i):
                self.streamMove(self.history[j])

            del self.history[:i]

    # Stream leftover moves in history
    def finishMoves(self):

        # print "finishMoves(): streaming history:", len(self.history)
        for move in self.history:
            self.streamMove(move)

    def streamMove(self, move):

            if move.isMove():
                self.planner.streamMove(move)
            else:
                if self.planner.args.mode != "pre":
                    move.streamCommand(self.planner.printer)


class PathData (object):

    def __init__(self, planner):

        self.planner = planner

        self.Tu = planner.printer.printerProfile.getTuI()

        self.path = []

        # History of last fftime moves
        self.history = MoveHistory(planner, self.Tu * 2.0)

        self.count = -10

        if not planner.travelMovesOnly:

            mp = self.planner.matProfile

            hwVersion = planner.printer.printerProfile.getHwVersionI()
            nozzleDiam = NozzleProfile.getSize()

            self.ks = mp.getKpwm(hwVersion, nozzleDiam)
            self.ktemp = mp.getKtemp(hwVersion, nozzleDiam)

            p0TempAir = mp.getP0temp(hwVersion, nozzleDiam)
            self.p0Air = mp.getP0pwm(hwVersion, nozzleDiam) # xxx hardcoded in firmware!
            self.fr0Air = mp.getFR0pwm(hwVersion, nozzleDiam)

            p0TempPrint = mp.getP0tempPrint(hwVersion, nozzleDiam)
            self.p0Print = mp.getP0pwmPrint(hwVersion, nozzleDiam) # xxx hardcoded in firmware!
            self.fr0Print = mp.getFR0pwmPrint(hwVersion, nozzleDiam)

            self.goodtemp = mp.getHotendGoodTemp()
            self.lastTemp = self.goodtemp

            # xxx same as in genTempTable

            # Interpolate best case flowrate (into air)
            (sleTempBest, slePwmBest) = mp.getFrSLE(hwVersion, nozzleDiam)
            print "best case flowrate:", sleTempBest, slePwmBest

            # Interpolate worst case flowrate (100% fill with small nozzle)
            (sleTempPrint, slePwmPrint) = mp.getFrSLEPrint(hwVersion, nozzleDiam)
            print "worst case flowrate:", sleTempPrint, slePwmPrint

            # XXX simple way, use average of best and worst flowrate:

            assert(self.planner.args.workingPoint >= 0 and self.planner.args.workingPoint <= 1.0)

            delta = sleTempBest.c-sleTempPrint.c
            assert(delta > 0.0)

            self.tempSLE = util.SLE(x1=0, y1=sleTempPrint.c + delta*self.planner.args.workingPoint, m=sleTempBest.m)

            delta = slePwmBest.c-slePwmPrint.c
            assert(delta > 0.0)

            self.pwmSLE = util.SLE(x1=0, y1=slePwmPrint.c + delta*self.planner.args.workingPoint, m=slePwmBest.m)

            print "Temp sle:", self.tempSLE, self.tempSLE.y(self.goodtemp)
            print "Pwm sle:", self.pwmSLE

    def updateHistory(self, move):
        self.history.append(move)

    def nextMoveNumber(self):
        return self.count + 10 # leave space for advance-submoves

    # Number of moves
    def incCount(self):
        self.count = self.nextMoveNumber()
        return self.count

    def doAutoTemp(self, moves):

        # Sum up path time and extrusion volume of moves
        tsum = 0
        vsum = 0
        for move in moves:
            tsum += move.accelData.getTime()
            vsum += move.getExtrusionVolume(self.planner.matProfile)

        avgERate = vsum / tsum

        # Increase temp somewhat above the values in the meterial
        # profile to keep away from temperature limiter.
        adj = 1.0 + 0.1

        # Compute new temp and suggested heater-PWM for this segment

        # Amount of flowrate above floor (that is the flowrate at goodtemp)
        rateDiff = (avgERate*adj) - self.tempSLE.y(self.goodtemp)

        tempIncrease = 0.0

        if rateDiff > 0.0:
            # Needed temp increase for this flowrate delta
            tempIncrease = round((rateDiff / self.ktemp) + 0.5)

        # Floor temp
        baseTemp = self.goodtemp + self.planner.l0TempIncrease
        newTemp = min(baseTemp+tempIncrease, self.planner.matProfile.getHotendMaxTemp())
      
        rateDiff = (newTemp - baseTemp) * self.ktemp

        newTemp = int(newTemp)

        if newTemp == self.lastTemp:
            # Nothing to change...
            return 

        suggestPwm = int(round(self.pwmSLE.x( self.tempSLE.y(self.goodtemp) + rateDiff ) + 0.5))
        suggestPwm = min(suggestPwm, 255)

        if debugAutoTemp:
            self.planner.gui.log( "AutoTemp: %d moves of %.2f s duration, extrusion rate: %.2f mm³/s, temp: %d, PWM: %d" % (len(moves), tsum, avgERate, newTemp, suggestPwm))

        cmd = SyncedCommand(
            CmdSuggestPwm,
            0.0, [
             packedvalue.uint8_t(HeaterEx1),
             packedvalue.uint16_t(newTemp), 
             packedvalue.uint8_t(suggestPwm) ])
        self.updateHistory(cmd)

#####################################################################

class DebugPlot (object):

    def __init__(self, nr):

        self.plotfile = "/tmp/ddsteps_%04d.pkl" % nr

        self.plot = Namespace()
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

    def __init__(self, args, printer, materialProfile=None, travelMovesOnly=False):

        if Planner.__single:
            raise RuntimeError('A Planner already exists')

        Planner.__single = self

        # if gui:
            # self.gui = gui
        # else:
            # self.gui = dddumbui.DumbGui()

        self.gui = printer.gui

        self.args = args
        self.matProfile = materialProfile

        self.printer = printer

        self.zeroPos = util.MyPoint()

        #
        # Constants, xxx todo: query from printer and/or profile
        #
        self.HOMING_FEEDRATE = [100, 100, 40]  # set the homing speeds (mm/s) 
        # self.HOMING_FEEDRATE = [50, 50, 10]  # set the homing speeds (mm/s) 
        self.HOME_RETRACT_MM = 7               # [mm]

        # ENDSTOP SETTINGS:
        # Sets direction of endstops when homing; 1=MAX, -1=MIN
        self.X_HOME_DIR = -1
        self.Y_HOME_DIR = 1
        self.Z_HOME_DIR = 1
        self.HOME_DIR = (self.X_HOME_DIR, self.Y_HOME_DIR, self.Z_HOME_DIR)

        # Travel limits after homing
        self.X_MIN_POS = 0
        self.X_MAX_POS = printer.printerProfile.getPlatformLengthI(X_AXIS)
        # X_MIN_POS = 0
        self.Y_MAX_POS = printer.printerProfile.getPlatformLengthI(Y_AXIS)
        # Y_MIN_POS = 0
        self._Z_MAX_POS = printer.printerProfile.getPlatformLengthI(Z_AXIS)
        # Z_MIN_POS = 0
        self.MAX_POS = (self.X_MAX_POS, self.Y_MAX_POS, self._Z_MAX_POS)

        # Bed leveling constants
        self.LEVELING_OFFSET = 0.1                   # Assumed thickness of feeler gauge/paper used in leveling (mm)
        self.HEAD_HEIGHT = 35.0                      # Let enough room for the head.

        # Homing
        self.X_HOME_POS = self.X_MIN_POS
        self.Y_HOME_POS = self.Y_MAX_POS
        self._Z_HOME_POS = self._Z_MAX_POS

        # 40 für avr, 50khz
        # 20 für arm, 100khz
        self.minTimerValue = int(fTimer / printer.printerProfile.getMaxStepperFreq())
        self.maxTimerValue = maxTimerValue16

        #
        # End Constants
        #

        # Temp. increase for layer 0
        self.l0TempIncrease = Layer0TempIncrease

        self.plotfile = None
        self.travelMovesOnly = travelMovesOnly

        if not travelMovesOnly:
            self.advance = Advance(self, args)

        self.reset()

    def reset(self):

        self.pathData = PathData(self)

        self.partNumber = 1

        self.stepRounders = StepRounders()

        self.curDirBits = None

    # @classmethod
    def get(cls):
        return cls.__single

    def getJerk(self):

        jerk = []
        for dim in dimNames:
            jerk.append(self.printer.printerProfile.getJerk(dim))

        return jerk

    def getHomePos(self):

        # Get additional z-offset from printer profile
        add_homeing_z = self.printer.printerProfile.getBedlevelOffsetI()

        assert((add_homeing_z <= 0) and (add_homeing_z >= -35))

        # print "add_homeing_z from printer profile: ", add_homeing_z

        # Virtuelle position des druckkopfes falls 'gehomed'
        homePosMM = util.MyPoint(
            X = self.X_HOME_POS,
            Y = self.Y_HOME_POS,
            #    
            # add_homeing_z is the not-usable space of the z dimension of the
            # build volume.
            #    
            Z = self._Z_HOME_POS + add_homeing_z
            )

        # Diese stepper position wird gesetzt falls der drucker 'gehomed' ist
        homePosStepped = vectorMul(homePosMM.vector(), self.printer.printerProfile.getStepsPerMMVectorI())

        return (homePosMM, homePosStepped)

    # Called from gcode parser
    def newPart(self, partNumber):

        self.partNumber = partNumber

    # Called from gcode parser
    def layerChange(self, layer):

        self.advance.layerChange(layer)

        if layer == 0:

            self.gui.log("Layer 0, increasing hotend temp by ", Layer0TempIncrease)
            self.l0TempIncrease = Layer0TempIncrease

        else:

            self.l0TempIncrease = 0

        if layer == 2:

            self.partNumber -= 1
            if self.partNumber:
                return

            # Reduce bedtemp
            bedTemp = self.matProfile.getBedTempReduced()
            self.gui.log("Layer2, reducing bedtemp to: ", bedTemp)

            cmd = SyncedCommand(
                CmdSyncTargetTemp, 
                0.0, [
                packedvalue.uint8_t(HeaterBed),
                packedvalue.uint16_t(bedTemp) ])

            self.pathData.updateHistory(cmd)

    # Called from gcode parser
    def g92(self, values):

        self.stepRounders.g92(values)

    # Called from gcode parser
    def g900(self, values):

        self.advance.g900(values)

    # Called from gcode parser, pause the print for the specified amount
    # of time.
    def dwellMS(self, ms):

        # A timervalue of 50,000 gives us a 25 mS pause:
        nNop = int(round(ms / 25.0))

        assert(nNop <= Uint16Max)

        # Ignore pause's smaller than 25 mS
        if nNop > 0:

            # This ends the current path and stops the head
            self.endPath()

            # Add NOP moves
            if self.args.mode != "pre":
                # self.printer.sendCommandParamV(CmdDwellMS, [packedvalue.uint16_t(nNop)])
                cmd = SyncedCommand(CmdDwellMS, ms / 1000.0, [ packedvalue.uint16_t(nNop) ])
                self.pathData.updateHistory(cmd)

    # Called from gcode parser, start fan
    def fanOn(self, fanSpeed, blipTime):

        cmd = SyncedCommand(CmdSyncFanSpeed, blipTime / 1000.0, [ packedvalue.uint8_t(fanSpeed), packedvalue.uint8_t(blipTime) ])
        self.pathData.updateHistory(cmd)

    # Start planning of this path
    def endPath(self):

        if self.pathData.path:

            lastMove = self.pathData.path[-1]

            if lastMove.isPrintMove():

                #
                # Finish path of printmoves.
                #
                # print "endPath(): ending path print with %d moves" % len(self.pathData.path)
                self.advance.planPath(self.pathData.path)

                if debugPlot:
                    self.plotfile.close()
                    self.plotfile = None

            else:

                #
                # Finish path of travelmoves.
                #
                #
                # Do a simple path planning for traveling moves:
                # * start/stop at jerk/2
                # * don't do advance
                #
                # print "endPath(): ending travel path with %d moves" % len(self.pathData.path)
                self.planTravelPath(self.pathData.path)

            self.pathData.path = []

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
                # Append segment to current path
                prevMove.nextMove = move
                move.prevMove = prevMove
            else:
                # Trigger processing of current path and start a new path
                # with the new segment.
                pathEnding = True

        if pathEnding:
            self.endPath()

        self.pathData.path.append(move)

        if debugMoves:
            print "***** End addMove() *****"
        
    def planTravelPath(self, path):

        if debugMoves:
            print "***** Start planTravelPath() *****"

        jerk = self.getJerk()

        # Set startspeed of first move
        path[0].setPlannedJerkStartSpeed(jerk, "planTravelPath() first move")

        prevMove = path[0]

        # Step 1: join moves forward
        for move in path[1:]:
        
            util.joinTravelMoves(prevMove, move, jerk)
            prevMove = move

        for move in path:
            move.state = 1

        # Check max endspeed of last move
        # Set endspeed of last move
        lastMove = path[-1]

        # Max reachable speed of last move
        allowedAccel5 = lastMove.getMaxAllowedAccelNoAdv5()
        maxEndSpeed = util.vAccelPerDist(lastMove.startSpeed.speed().feedrate5(), allowedAccel5, lastMove.distance5)

        v = lastMove.getJerkSpeed(jerk) or lastMove.topSpeed.speed()

        endSpeed = lastMove.endSpeed.speed()
        endSpeed.setSpeed(min(v.feedrate5(), maxEndSpeed))
        lastMove.endSpeed.setSpeed(endSpeed, "planTravelPath() - last move")

        # Mark last move as end-move, for softstop
        lastMove.isEndMove = True

        """
        # Sanity check
        for move in path:
            move.sanityCheck(jerk)
        """

        # Step 2: join moves backwards
        self.joinTravelMovesBwd(path)

        """
        # Sanity check
        for move in path:
            move.sanityCheck(jerk)
        """

        # Step 3: plan acceleration
        for move in path:
            self.planTravelAcceleration(move)

        # Sanity check
        for move in path:
            move.sanityCheck(jerk)

        # Step 4: plan steps and stream moves to printer
        if debugMoves:
            print "Streaming %d travel moves..." % len(path)

        rateList = []
        for move in path:
            # rateList.append(move.topSpeed.speed().eSpeed)
            rateList.append(move.topSpeed.speed()[3])

        avgRate = sum(rateList) / len(rateList)

        if avgRate > 0:
            path[0].isMeasureMove = True
            path[0].measureSpeed = avgRate

        # Move timeline housekeeping
        # for move in path:
            # self.pathData.updateTimeline(move)

        for move in path:

            self.planTravelSteps(move)

            # self.streamMove(move)
            self.pathData.updateHistory(move)

            # Help garbage collection
            move.prevMove = util.StreamedMove()
            move.nextMove = util.StreamedMove()

        if debugMoves:
            print "***** End planTravelPath() *****"

    # Todo: should be called finishPath()
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

            else:

                # print "finishMoves(): ending travel path with %d moves" % len(self.pathData.path)
                self.planTravelPath(self.pathData.path)

        self.pathData.history.finishMoves()
        self.reset()


    def streamMove(self, move):

        if debugMoves:
            if move.isSubMove():
                print "Streaming sub-move:", move.moveNumber, move.parentMove.moveNumber
            else:
                print "Streaming move:", move.moveNumber

        if debugPlot:

            if not self.plotfile:
                self.plotfile = DebugPlot(move.moveNumber)

            self.plotfile.plotSteps(move)

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

        maxAccel = self.printer.printerProfile.getMaxAxisAccelerationI()

        if deltaStartSpeedS:

            ta = deltaStartSpeedS / allowedAccel
            # print "accel time (for %f mm/s): %f [s]" % (deltaStartSpeedS, ta)

            # debug Check axxis acceleration
            deltaSpeedV = move.direction5.scale(deltaStartSpeedS)
            for dim in range(5):
                dimAccel = abs(deltaSpeedV[dim]) / ta
                if (dimAccel / maxAccel[dim]) > 1.001:
                    print "dim %d verletzt max accel: " % dim, dimAccel, " > ", maxAccel[dim]
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
                if (dimDecel / maxAccel[dim]) > 1.001:
                    print "dim %d verletzt max accel: " % dim, dimDecel, " [mm/s] > ", maxAccel[dim], " [mm/s]"
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
            # print "ta: ", ta, deltaSpeedS

            deltaSpeedS = v - endSpeedS                          # [mm/s]

            # Handle rounding errors
            if deltaSpeedS < 0:
                assert(deltaSpeedS > -0.000001)
                deltaSpeedS = 0
                v = endSpeedS

            tb = deltaSpeedS / allowedAccel
            # print "tb: ", tb, deltaSpeedS

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

        # Round step values
        dispF = move.displacement_vector_steps_raw5
        dispS = self.stepRounders.round(dispF)

        if dispS == emptyVector5:

            if debugMoves:
                print "Empty move..."
                print "***** End planTravelSteps() *****"

            self.stepRounders.rollback()
            return

        move.isStartMove = True

        self.stepRounders.commit()

        abs_displacement_vector_steps = vectorAbs(dispS)

        # Determine the 'lead axis' - the axis with the most steps
        leadAxis = move.leadAxis(disp=dispS)
        leadAxis_steps = abs_displacement_vector_steps[leadAxis]

        dirBits = util.directionBits(dispS)

        if dirBits != self.curDirBits:
            move.stepData.setDirBits = True
            move.stepData.dirBits = dirBits
            self.curDirBits = dirBits

        steps_per_mm = self.printer.printerProfile.getStepsPerMMI(leadAxis)

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

            accelClocks = self.accelRamp(
                steps_per_mm,
                v0,
                nominalSpeed,
                allowedAccel,
                leadAxis_steps) # maximum number of steps

            move.stepData.setAccelPulses(accelClocks)

            nAccel = len(accelClocks)

        nDecel = 0
        if move.decelTime():

            decelClocks = self.decelRamp(
                steps_per_mm,
                nominalSpeed,
                v1,
                allowedAccel,
                leadAxis_steps) # maximum number of steps

            move.stepData.setDecelPulses(decelClocks)

            nDecel = len(decelClocks)


        #
        # Linear phase
        #
        nLin = leadAxis_steps - (nAccel + nDecel)
        # print "# linear steps:", nLin

        if nLin > 0:

            steps_per_second_nominal = nominalSpeed * steps_per_mm
            timerValue = fTimer / steps_per_second_nominal
            move.stepData.setLinTimer(self.timerLimit(timerValue))

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

        if debugMoves:
            print "# of steps for move: ", leadAxis_steps
            move.pprint("move:")
            print 

        move.stepData.checkLen(leadAxis_steps)

        if debugMoves:
            print "***** End planTravelSteps() *****"
    
    # Check if stepper frequency gets to high or to low
    def timerLimit(self, timer):

        if timer < self.minTimerValue:
            print "Warning, timervalue %d to low (%d)!" % (timer, self.minTimerValue)
            timer = self.minTimerValue
        elif timer > self.maxTimerValue:
            print "Warning, timervalue %d to high (%d)!" % (timer, self.maxTimerValue)
            timer = self.maxTimerValue

        return timer

    ####################################################################################################
    #
    # Create a list of stepper pulses for a acceleration ramp.
    #
    def accelRamp(self, steps_per_mm, vstart, vend, a, nSteps):

        assert(vstart <= vend)

        pulses = [] # (tstep, dt, timerValue)

        # steps_per_mm = PrinterProfile.getStepsPerMM(axis)
        sPerStep = 1.0/steps_per_mm

        v = vstart
        tstep = 0
        s = sPerStep

        stepToDo = nSteps

        while v < vend and stepToDo > 0:

            # Speed after this step
            vn1 = util.vAccelPerDist(vstart, a, s)

            # Time we need for this speed change/this step:
            dv = vn1 - v
            dt = dv / a

            # Timervalue for this time
            timerValue = self.timerLimit(int(dt * fTimer))

            # timerValue = min(timerValue, ddprintconstants.maxTimerValue16)

            # print "v after this step:", vn1, s, dt, timerValue

            pulses.append(timerValue)

            s += sPerStep
            v = vn1
            tstep += dt
            stepToDo -= 1

        # print "acel pulses:",
        # pprint.pprint(pulses)

        return pulses

    ####################################################################################################
    #
    # Create a list of stepper pulses for a deceleration ramp.
    #
    def decelRamp(self, steps_per_mm, vstart, vend, a, nSteps):

        assert(vstart >= vend)

        pulses = []

        # steps_per_mm = PrinterProfile.getStepsPerMM(axis)
        sPerStep = 1.0/steps_per_mm

        v = vstart
        tstep = 0
        s = sPerStep

        while v > vend and nSteps > 0:

            # Speed after this step
            vn1 = util.vAccelPerDist(vstart, -a, s)

            # Time we need for this speed change/this step:
            dv = v - vn1
            dt = dv / a

            # Timervalue for this time
            timerValue = self.timerLimit(int(dt * fTimer))

            # if timerValue > ddprintconstants.maxTimerValue16:
                # # print "break on timeroverflow, v after this step:", vn1, s, dt, timerValue
                # break

            # print "v after this step:", vn1, s, dt, timerValue

            pulses.append(timerValue)

            s += sPerStep
            v = vn1
            tstep += dt
            nSteps -= 1

        # print "decel pulses:",
        # pprint.pprint(pulses)

        return pulses


