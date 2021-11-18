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

import math, collections, types, pprint, bisect
from argparse import Namespace

import ddprintutil as util, dddumbui, packedvalue
import gcodeparser, cobs, intmath

from ddvector import Vector, vectorMul, vectorAbs
from ddprintconstants import *
from ddconfig import *
from ddprinter import Printer
from ddprintcommands import CmdSyncTargetTemp, CmdDwellMS, CmdSyncFanSpeed, CmdSuggestPwm
from ddprintcommands import CmdNop
from ddprintstates import HeaterEx1, HeaterBed, PrintModeManual, PrintModePrinting
from ddadvance import Advance
from ddprofile import NozzleProfile, MatProfile

#####################################################################

if debugPlot:
    import pickle

emptyVector5 = [0] * 5

FillByte = chr(CmdNop)

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

    def streamCommand(self, planner):

        payload = chr(self.cmd)
        for p in self.params:
            if p:
                payload += p.pack()

        planner.streamStepData(payload)

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
                # move.streamCommand(self.planner.printer)
                move.streamCommand(self.planner)


class PathData (object):

    def __init__(self, planner):

        self.planner = planner

        self.Tu = planner.printer.printerProfile.getTuI()

        self.path = []

        # History of last fftime moves
        self.history = MoveHistory(planner, self.Tu * 2.0)

        self.count = -10

        self.wpscale = (1 - self.planner.args.workingPoint)

        if not planner.travelMovesOnly:

            mp = self.planner.matProfile

            self.ks = mp.getKpwm()
            self.ktemp = mp.getKtemp()

            p0TempAir = mp.getP0temp()
            self.p0Air = mp.getP0pwm() # xxx hardcoded in firmware!
            self.fr0Air = mp.getFR0pwm()

            p0TempPrint = mp.getP0tempPrint()
            self.p0Print = mp.getP0pwmPrint() # xxx hardcoded in firmware!
            self.fr0Print = mp.getFR0pwmPrint()

            self._goodtemp = mp.getHotendGoodTemp()
            self.lastTemp = self._goodtemp

            # xxx same as in genTempTable

            # Interpolate best case flowrate (into air)
            (sleTempBest, slePwmBest) = mp.getFrSLE()
            # print "best case flowrate:", sleTempBest, slePwmBest

            # Interpolate worst case flowrate (100% fill with small layerheight)
            (sleTempPrint, slePwmPrint) = mp.getFrSLEPrint()
            # print "worst case flowrate:", sleTempPrint, slePwmPrint

            # XXX simple way, use average of best and worst flowrate:

            # WorkingPoint = 0: Parts print, higer temperatures for good layer bonding
            # WorkingPoint = 1: Figurine print, lower temperature range
            assert(self.planner.args.workingPoint >= 0 and self.planner.args.workingPoint <= 1.0)

            # Note: into-air extrusion not always higher rates

            temp_delta = sleTempBest.c-sleTempPrint.c
            # print "Delta temp :", temp_delta

            # if the temperatue needed to print is lower than the 
            # into-air temperature, the we assume some
            # cooling-, environmental-, bed-temperature- or some material effects that
            # makes it flow better when really printing.
            # But we can not count on this effect, so ignore this
            # here:

            # but check if error is small (<=10%)
            if temp_delta < 0:
                assert((abs(temp_delta) / 230) <= 0.1)

            temp_delta = max(temp_delta, 0.0)
            self.tempSLE = util.SLE(x1=0, y1=sleTempPrint.c+temp_delta*self.wpscale, m=sleTempBest.m)

            # Note: into-air extrusion not always higher rates

            pwm_delta = slePwmBest.c-slePwmPrint.c
            # print "Delta pwm :", pwm_delta

            # but check if error is small (<=10%)
            if pwm_delta < 0:
                assert((abs(pwm_delta) / 255) <= 0.1)

            pwm_delta = max(pwm_delta, 0.0)

            assert( temp_delta >= 0.0)
            assert( pwm_delta >= 0.0)

            self.pwmSLE = util.SLE(x1=0, y1=slePwmPrint.c+pwm_delta*self.wpscale, m=slePwmBest.m)

            # print "Temp sle:", self.tempSLE, self.tempSLE.y(self._goodtemp)
            # print "Pwm sle:", self.pwmSLE

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

        # Floor temp is between basetemp and goodtemp, controlled by workingPoint parameter
        bt = self.planner.matProfile.getHotendBaseTemp()
        goodtemp = bt + (self._goodtemp - bt) * self.wpscale + self.planner.l0TempIncrease

        # Amount of flowrate above floor (that is the flowrate at goodtemp)
        rateDiff = (avgERate*adj) - self.tempSLE.y(goodtemp)

        tempIncrease = 0.0

        if rateDiff > 0.0:
            # Needed temp increase for this flowrate delta
            tempIncrease = round((rateDiff / self.ktemp) + 0.5)

        newTemp = min(goodtemp+tempIncrease, self.planner.matProfile.getHotendMaxTemp())
      
        rateDiff = (newTemp - goodtemp) * self.ktemp

        newTemp = int(newTemp)

        suggestPwm = int(round(self.pwmSLE.x( self.tempSLE.y(goodtemp) + rateDiff ) + 0.5))
        suggestPwm = min(suggestPwm, 255)

        if newTemp == self.lastTemp:
            # self.planner.gui.log( "AutoTemp: temp did not change, skipping temperature command.")
            return 

        if debugAutoTemp:
            self.planner.gui.log( "AutoTemp: segment of %.2f s duration, avg. extrusion rate: %.2f mm³/s, new temp: %d, forward-PWM: %d" % (tsum, avgERate, newTemp, suggestPwm))

        cmd = SyncedCommand(
            CmdSuggestPwm,
            0.0, [
             packedvalue.uint8_t(HeaterEx1),
             packedvalue.uint16_t(intmath.toFWTemp(newTemp)), 
             packedvalue.uint8_t(suggestPwm) ])
        self.updateHistory(cmd)

        self.lastTemp = newTemp

#####################################################################

# Helper to detect layer changes
class Layers (object):

    def __init__(self, planner):

        self.planner = planner
        self.layerList = []
        self.curLayer = None

        self.zStep = planner.printer.printerProfile.getStepsPerMMI(Z_AXIS)

    def addMove(self, move):

        zpos = move.getPos()[Z_AXIS]
        zstep = int(zpos * self.zStep)

        try:
            layer = self.layerList.index(zstep)
        except ValueError:
            # Z value not in list
            bisect.insort(self.layerList, zstep)
            layer = self.layerList.index(zstep)

        if layer != self.curLayer:
            if layer > 0:
                self.planner.layerChange(layer-1)
            self.curLayer = layer

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

    def __init__(self, args, printer, nozzleProfile=None, materialProfile=None, travelMovesOnly=False):

        if Planner.__single:
            raise RuntimeError('A Planner already exists')

        Planner.__single = self

        # if gui:
            # self.gui = gui
        # else:
            # self.gui = dddumbui.DumbGui()

        self.gui = printer.gui

        self.args = args
        self.nozzleProfile = nozzleProfile
        self.matProfile = materialProfile

        self.printer = printer

        self.zeroPos = util.MyPoint()

        #
        # Constants, xxx todo: query from printer and/or profile
        #
        self.HOME_RETRACT_MM = 7               # [mm]

        # Travel limits after homing
        self.X_MAX_POS = printer.printerProfile.getPlatformLengthI(X_AXIS)
        self.Y_MAX_POS = printer.printerProfile.getPlatformLengthI(Y_AXIS)
        self.Z_MAX_POS = printer.printerProfile.getPlatformLengthI(Z_AXIS)
        self.MAX_POS = (self.X_MAX_POS, self.Y_MAX_POS, self.Z_MAX_POS)

        # Bed leveling constants
        self.LEVELING_OFFSET = 0.1                   # Assumed thickness of feeler gauge/paper used in leveling (mm)
        self.HEAD_HEIGHT = 35.0                      # Let enough room for the head.

        # Homing pos
        if printer.printerProfile.getHomeDir(X_AXIS) > 0:
            self.X_HOME_POS = self.X_MAX_POS
        else:
            self.X_HOME_POS = 0

        if printer.printerProfile.getHomeDir(Y_AXIS) > 0:
            self.Y_HOME_POS = self.Y_MAX_POS
        else:
            self.Y_HOME_POS = 0

        # 40 für avr, 50khz
        # 20 für arm, 100khz
        self.minTimerValue = int(fTimer / printer.printerProfile.getMaxStepperFreq())
        self.maxTimerValue = maxTimerValue16

        #
        # End Constants
        #

        # Temp. increase for layer 0
        self.l0TempIncrease = Layer0TempIncrease

        # Bed temperatures
        if materialProfile:
            self.bedTemp = materialProfile.getBedTemp()
            self.bedTempReduced = materialProfile.getBedTempReduced()
            self.curBedTemp = self.bedTemp

        self.plotfile = None
        self.travelMovesOnly = travelMovesOnly

        if not travelMovesOnly:
            self.advance = Advance(self, args)

        self.replay = 0

        self.reset()

    def reset(self):

        self.pathData = PathData(self)

        self.stepRounders = StepRounders()

        self.curDirBits = None

        # Binary data to send to printer
        self.stepData = ""

        self.printMode = PrintModeManual

        self.layers = Layers(self)

    def reconnect(self, status):

        self.replay = status.swapsize / 512 
        print "Reconnect: replaying blocks:", self.replay

    # @classmethod
    def get(cls):
        return cls.__single

    def setPrintMode(self, mode):
        self.printMode = mode

    def getJerk(self):

        jerk = []
        for dim in dimNames:
            jerk.append(self.printer.printerProfile.getJerk(dim))

        return jerk


    # Z position after hw endstop was hit and we did some back-off.
    def getZHomePos(self):

        if not self.printer.printerProfile.homingToZero():
            #
            # Printer z-homes in positive direction
            #
            #    
            # add_homeing_z is the not-usable space of the z dimension of the
            # build volume.
            #    
            add_homeing_z = self.printer.printerProfile.getBedlevelOffset()
            return self.Z_MAX_POS - add_homeing_z
        
        # Bed is levelled at home pos, homing sets
        # bed-level-distance.
        return self.LEVELING_OFFSET

    def getHomePos(self, liftHead = False):

        # Z-height after homing
        if liftHead and self.printer.printerProfile.homingToZero():
            z = HOMEZLIFT
        else:
            z = self.getZHomePos()

        homePosMM = util.MyPoint(
            X = self.X_HOME_POS,
            Y = self.Y_HOME_POS,
            Z = z
            )

        # Diese stepper position wird gesetzt falls der drucker 'gehomed' ist
        homePosStepped = vectorMul(homePosMM.vector(), self.printer.printerProfile.getStepsPerMMVectorI())

        return (homePosMM, homePosStepped)

    # Called from gcode parser
    def layerChange(self, layer):

        if not self.printMode == PrintModePrinting:
            return

        self.advance.layerChange(layer)

        if layer == 0:

            # Print first layer hotter (hotend)
            self.gui.log("Layer 0, increasing hotend temp by ", Layer0TempIncrease)
            self.l0TempIncrease = Layer0TempIncrease

            if self.curBedTemp != self.bedTemp:

                # Set bed temp to normal bedtemp
                self.gui.log("Layer %d, setting bedtemp to: " % layer, self.bedTemp)

                cmd = SyncedCommand(
                    CmdSyncTargetTemp, 
                    0.0, [
                    packedvalue.uint8_t(HeaterBed),
                    packedvalue.uint16_t(intmath.toFWTemp(self.bedTemp)) ])

                self.pathData.updateHistory(cmd)

                self.curBedTemp = self.bedTemp

        else:

            if self.l0TempIncrease:
                self.gui.log("Layer %d, lowering hotend temp by " % layer, Layer0TempIncrease)
                self.l0TempIncrease = 0

            if self.curBedTemp != self.bedTempReduced:

                # Set bed temp to reduced bedtemp
                self.gui.log("Layer %d, reducing bedtemp to: " % layer, self.bedTempReduced)

                cmd = SyncedCommand(
                    CmdSyncTargetTemp, 
                    0.0, [
                    packedvalue.uint8_t(HeaterBed),
                    packedvalue.uint16_t(intmath.toFWTemp(self.bedTempReduced)) ])

                self.pathData.updateHistory(cmd)

                self.curBedTemp = self.bedTempReduced

    # Called from gcode parser
    def g92(self, values):

        self.stepRounders.g92(values)

    # Called from gcode parser
    def m900(self, values):

        self.advance.m900(values)

    # Called from gcode parser,
    # set strength aka workingpoint
    def m901(self, values):
        
        print "XXX m901 set workingpoint or set whatever ddprint option not implemented", values

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

    # xxxxxxxxxxxxx# Called from gcode parser, set new position
    # xxxxxxxxxxxxxdef setPos(self, oldPos, newPos):
        # xxxxxxxxxxxxxself.layers.setPos(oldPos, newPos)

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

                if self.plotfile:
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

        self.layers.addMove(move)

        if self.pathData.path:

            prevMove = self.pathData.path[-1]
            if prevMove.isPrintMove() != move.isPrintMove():
                # Trigger processing of current path and start a new path
                # with the new segment.
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
            move.sanityCheck()
        """

        # Step 2: join moves backwards
        self.joinTravelMovesBwd(path)

        """
        # Sanity check
        for move in path:
            move.sanityCheck()
        """

        # Step 3: plan acceleration
        for move in path:
            self.planTravelAcceleration(move)

        # Sanity check
        for move in path:
            move.sanityCheck()

        # Step 4: plan steps and stream moves to printer
        if debugMoves:
            print "Streaming %d travel moves..." % len(path)

        for move in path:
            # xxxx check for minimal frs steps ....
            if move.eDistance() > 1.0 and move.linearTime() > 0.15:
                print "FRS: e-dist, linear time:", move.eDistance(), move.linearTime()
                move.isMeasureMove = True

        # Move timeline housekeeping
        # for move in path:
            # self.pathData.updateTimeline(move)

        for move in path:

            if self.planTravelSteps(move):

                self.pathData.updateHistory(move)

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

                if self.plotfile:
                    self.plotfile.close()
                    self.plotfile = None

            else:

                # print "finishMoves(): ending travel path with %d moves" % len(self.pathData.path)
                self.planTravelPath(self.pathData.path)

        self.pathData.history.finishMoves()

        # Send last partial 512 bytes block
        remaining = len(self.stepData)

        if self.args.mode != "pre":
        
            # print "Sending last stepdata block of size %d" % remaining

            # Fill sector with dummy data
            block = self.stepData + (cobs.SectorSize-remaining)*FillByte
            if self.replay:
                self.replay -= 1
            else:
                cobsPayload = cobs.encodeCobs512(block)
                self.printer.sendCommand512(cobsPayload)

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

        self.streamStepData(move.command())

    def streamStepData(self, stepData):

        self.stepData += stepData

        sent = 0
        left = len(self.stepData)
        while left >= cobs.SectorSize:

            block = self.stepData[sent:sent+cobs.SectorSize]

            if self.replay:
                self.replay -= 1
            else:
                cobsPayload = cobs.encodeCobs512(block)
    
                assert((len(cobsPayload) >= 512) and (len(cobsPayload) <= 515))

                # print "cobs encoded sector block:", cobsPayload.encode("hex")

                if self.args.mode != "pre":
                    self.printer.sendCommand512(cobsPayload)

            sent += cobs.SectorSize
            left -= cobs.SectorSize

        self.stepData = self.stepData[sent:]
        # print "streamStepData: %d bytes stepdata left..." % left

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

            if index >= 0:

                #
                # Adjust endspeed of the previous move, also.
                #
                prevMove = moves[index]

                factor = maxAllowedStartSpeed / startSpeed1S
                # print "factor: ", factor

                # Adjust endspeed of last move:
                assert(factor < 1)

                # XXX einfacher algo, kann man das besser machen (z.b. mit jerk-berechnung,
                # vector subtraktion oder so?)
                endSpeed0 = prevMove.endSpeed.speed().scale(factor)
                prevMove.endSpeed.setSpeed(endSpeed0, "joinMovesBwd - prevMove breaking")

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
            return False

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
        allowedAccel = move.getMaxAllowedAccelVectorNoAdv5()[leadAxis]

        v0 = abs(move.startSpeed.speed()[leadAxis])                # [mm/s]

        nominalSpeed = abs( move.topSpeed.speed()[leadAxis] ) # [mm/s]

        v1 = abs(move.endSpeed.speed()[leadAxis])                # [mm/s]

        steps_per_second_nominal = nominalSpeed * steps_per_mm
        linearTimerValue = self.timerLimit(int(fTimer / steps_per_second_nominal))

        nAccel = 0
        if move.accelTime():

            accelClocks = self.accelRamp(
                steps_per_mm,
                v0,
                nominalSpeed,
                allowedAccel,
                leadAxis_steps,
                linearTimerValue)

            move.stepData.setAccelPulses(accelClocks)
            nAccel = len(accelClocks)

        nDecel = 0
        if move.decelTime():

            decelClocks = self.decelRamp(
                steps_per_mm,
                nominalSpeed,
                v1,
                allowedAccel,
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

        if debugMoves:
            print "# of steps for move: ", leadAxis_steps
            move.pprint("move:")
            print 

        move.stepData.checkLen(leadAxis_steps)

        if debugMoves:
            print "***** End planTravelSteps() *****"

        return True
    
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
    def accelRamp(self, steps_per_mm, vstart, vend, a, nSteps, linearTimerValue):

        # no-acceleration-aceleration, makes no sense?
        # assert(vstart <= vend)
        assert(vstart < vend)

        pulses = [] # (tstep, dt, timerValue)

        sPerStep = 1.0/steps_per_mm

        v = vstart
        tstep = 0
        s = sPerStep

        while v < vend and nSteps > 0:

            # Speed after this step
            vn1 = util.vAccelPerDist(vstart, a, s)

            # Time we need for this speed change/this step:
            dv = vn1 - v
            dt = dv / a

            # Timervalue for this time
            timerValue = self.timerLimit(int(dt * fTimer))

            # timerValue = min(timerValue, ddprintconstants.maxTimerValue16)

            # print "v after this step:", vn1, s, dt, timerValue

            if timerValue <= linearTimerValue:
                break

            pulses.append(timerValue)

            s += sPerStep
            v = vn1
            tstep += dt
            nSteps -= 1

        # print "acel pulses:",
        # pprint.pprint(pulses)

        return pulses

    ####################################################################################################
    #
    # Create a list of stepper pulses for a deceleration ramp.
    #
    def decelRamp(self, steps_per_mm, vstart, vend, a, nSteps, linearTimerValue):

        # no-deceleration-deceleration, makes no sense?
        # assert(vstart >= vend)
        assert(vstart > vend)

        pulses = []

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

            if timerValue > linearTimerValue:
                pulses.append(timerValue)

            s += sPerStep
            v = vn1
            tstep += dt
            nSteps -= 1

        # print "decel pulses:",
        # pprint.pprint(pulses)

        return pulses

####################################################################################################
# Create material profile singleton instance
def initMatProfile(args, printer, nozzleProfile):
    
    nozzle = None
    if nozzleProfile:
        nozzle = nozzleProfile.getSizeI()

    mat = MatProfile(
            args.mat, args.smat,
            printer.getPrinterName(args),
            printer.printerProfile.getHwVersionI(),
            nozzle
            )

    # Overwrite settings from material profile with command line arguments:
    if args.t0:
        mat.override("bedTemp", args.t0)
    if args.t0_reduced:
        mat.override("bedTempReduced", args.t0_reduced)
    elif args.t0:
        mat.override("bedTempReduced", args.t0)
    if args.t1:
        mat.override("hotendGoodTemp", args.t1)
        mat.override("hotendStartTemp", args.t1)

    return mat

def initParser(args, mode=None, gui=None, travelMovesOnly=False):

    # Create the Printer singleton instance
    printer = Printer(gui=gui)

    # Create printer profile
    printer.commandInit(args) # reconn

    if "nozzle" in args:
        nozzle = NozzleProfile(args.nozzle)
    else:
        nozzle = None

    # Create material profile singleton instance
    if "mat" in args:
        mat = initMatProfile(args, printer, nozzle)
    else:
        mat = None

    print "nozzle, mat:", nozzle, mat

    # Create planner singleton instance
    planner = Planner(args, nozzleProfile=nozzle, materialProfile=mat, printer=printer, travelMovesOnly=travelMovesOnly)

    parser = gcodeparser.UM2GcodeParser(planner, logger=gui, travelMovesOnly=travelMovesOnly)

    return (printer, parser, planner)

####################################################################################################

