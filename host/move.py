# -*- coding: utf-8 -*-
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

import math, struct, zlib

import ddprintcommands, cobs, io, intmath, packedvalue

import ddprintutil as util

from ddprintconstants import maxTimerValue16, fTimer, _MAX_ACCELERATION, X_AXIS, Y_AXIS, Z_AXIS, A_AXIS
from ddprintconstants import AdvanceEThreshold, StepDataTypeBresenham, StepDataTypeRaw
from ddvector import Vector, VelocityVector32, VelocityVector5, vectorLength, vectorSub, vectorAbs
from ddprintcommands import CommandNames, DirBitsBit, DirBitsBitRaw, MoveStartBit
from ddprintcommands import MeasureStartBit, MeasureStartBitRaw, EndMoveBit, EndMoveBitRaw
from ddprintcommands import TimerByteFlagBit, MoveStartBitRaw
from ddprofile import PrinterProfile

from ddconfig import *

##################################################
# Move types.
# Printing move, X/Y move in combination with extrusion A/B
MoveTypePrint = 0
# Traveling move,
# * X/Y move without extrusion A/B
# * Z move
# * Extrusion A/B only move
MoveTypeTravel = 1

##################################################

class AccelData:

    def __init__(self):

        pass

        # Time for the three phases of this move
        # self.accelTime = None
        # self.linearTime = None
        # self.decelTime = None

    def setDuration(self, accelTime, linearTime, decelTime):

        assert(accelTime >= 0)
        assert(linearTime >= 0)
        assert(decelTime >= 0)

        self.accelTime = accelTime
        self.linearTime = linearTime
        self.decelTime = decelTime

    def getTime(self):

        return self.accelTime + self.linearTime + self.decelTime

    def sanityCheck(self):

        if hasattr(self, "accelTime"):
            assert(self.accelTime >= 0)
        if hasattr(self, "linearTime"):
            assert(self.linearTime >= 0)
        if hasattr(self, "decelTime"):
            assert(self.decelTime >= 0)

    def __repr__(self):
        return "\n  AccelTime: %f, LinearTime: %f, DecelTime: %f" % (self.accelTime, self.linearTime, self.decelTime)

class StepData:

    def __init__(self):
        self.accelPulses = []
        self.linearTimer = None
        self.decelPulses = []
        self.setDirBits = False
        self.dirBits = 0
        self.leadAxis = 0
        self.abs_vector_steps = None

    """
    def addAccelPulse(self, timer):

        self.checkTimerValue(timer, maxTimerValue16)
        self.accelPulses.append(timer)

    def addAccelPulsees(self, accelPulses):

        for (_, _, timer) in accelPulses:
            self.addAccelPulse(timer)
    """

    def setAccelPulses(self, accelPulses):
        self.accelPulses = accelPulses

    def setLinTimer(self, timer):
        self.linearTimer = timer
  
    def setDecelPulses(self, decelPulses):
        self.decelPulses = decelPulses

    """
    def addDecelPulse(self, timer):

        self.checkTimerValue(timer, maxTimerValue16)
        self.decelPulses.append(timer)

    def addDecelPulsees(self, decelPulses):

        for (_, _, timer) in decelPulses:
            self.addDecelPulse(timer)
    """

    def setBresenhamParameters(self, leadAxis, abs_vector_steps):
        self.leadAxis = leadAxis
        self.abs_vector_steps = abs_vector_steps

    def empty(self):
        # return not self.abs_vector_steps[self.leadAxis]
        return not self.abs_vector_steps

    def __repr__(self):
        return "StepData:" + \
           "\n  SetDirbits: %s, Direction bits: 0x%x" % (self.setDirBits, self.dirBits) + \
           "\n  # leadAxis: %d" % self.leadAxis + \
           "\n  # abs_vector_steps: %s" % repr(self.abs_vector_steps) + \
           "\n  # accelPulses: %d" % len(self.accelPulses) + \
           "\n  # linearPulses: %d" % (self.abs_vector_steps[self.leadAxis] - (len(self.accelPulses) + len(self.decelPulses))) + \
           "\n  # decelPulses: %d" % len(self.decelPulses)

    def checkLen(self, deltaLead):
        assert(self.abs_vector_steps[self.leadAxis] - (len(self.accelPulses) + len(self.decelPulses)) >= 0)

    # Return binary encoded stepdata for
    # this PrintMove, ready to be send over serial...
    def command(self, move):

        flags = 0
        if self.setDirBits:
            flags = self.dirBits | DirBitsBit

        # Check how much acceleration timer values can be sent as a bytes
        nAccel16 = 0
        for tv in self.accelPulses:

                if tv > 255:
                    nAccel16 += 1

        assert(nAccel16 < 0xffff)

        # Check how many deceleration timer values can be sent as a byte
        nDecel8 = 0
        for tv in self.decelPulses:

                if tv < 256:
                    nDecel8 += 1
      
        assert(nDecel8 < 0xffff)

        startMoveFlag = 0
        if move.isStartMove:
            startMoveFlag = MoveStartBit

        measureStartBit = 0
        if move.isMeasureMove:
            measureStartBit = MeasureStartBit

        endMoveBit = 0
        if move.isEndMove:
            endMoveBit = EndMoveBit

        # print "ald:", len(self.accelPulses), self.abs_vector_steps[self.leadAxis]-(len(self.accelPulses)+len(self.decelPulses)), len(self.decelPulses)

        payLoad = struct.pack("<BHBiiiiiHH",
                ddprintcommands.CmdG1,
                flags | startMoveFlag | measureStartBit | endMoveBit,
                self.leadAxis,
                self.abs_vector_steps[0],
                self.abs_vector_steps[1],
                self.abs_vector_steps[2],
                self.abs_vector_steps[3],
                self.abs_vector_steps[4],
                nAccel16,
                len(self.accelPulses) - nAccel16)

        if move.isStartMove:

            # Store timervalue of max e-speed
            baseMove = move.getBaseMove()
            if baseMove.isPrintMove():

                # xxx move to own method  
                nominalSpeed = abs( baseMove.topSpeed.speed().eSpeed )
                nominalSpeed = baseMove.topSpeed.speed().eSpeed

                assert(nominalSpeed > 0)

                e_steps_per_second_nominal = nominalSpeed * baseMove.e_steps_per_mm
                eTimer = fTimer / e_steps_per_second_nominal

                # print "StartMove, E-Timer:", nominalSpeed, eTimer
                payLoad += intmath.eTimer(min(eTimer, 0xffff)).pack()

            else:
                # payLoad += struct.pack("<H", 0)
                payLoad += packedvalue.scaledint_t(0, 0).pack()

        payLoad += struct.pack("<HHH",
                self.linearTimer,
                nDecel8,
                len(self.decelPulses) - nDecel8)

        for timer in self.accelPulses[:nAccel16]:
            payLoad += struct.pack("<H", timer)
        for timer in self.accelPulses[nAccel16:]:
            payLoad += struct.pack("<B", timer)

        for timer in self.decelPulses[:nDecel8]:
            payLoad += struct.pack("<B", timer)
        for timer in self.decelPulses[nDecel8:]:
            payLoad += struct.pack("<H", timer)

        # print "size of payload:", len(payLoad)

        """
        rawPayloadSize = len(payLoad)

        # Test if compression gives smaller packet
        # XXX reuse compressor object?
        compressor = zlib.compressobj(9, zlib.DEFLATED, -15)
        compressedPayload = compressor.compress(payLoad)
        compressedPayload += compressor.flush()
        print "encodeCobs_cmd_packed compressed %d blocksize into %d bytes..." % (rawPayloadSize, len(compressedPayload))

        if len(compressedPayload) > rawPayloadSize*0.95:
            print "fixme Sending uncompressed..."
            # return chr(ddprintcommands.CmdG1Packed) + compressedPayload
        """

        return payLoad

        # stream = cStringIO.StringIO(payLoad)

        cmds = [ cobs.encodeCobs_cmd_packed(ddprintcommands.CmdG1, ddprintcommands.CmdG1Packed, stream) ]

        while True:

            (cmd, cobsBlock) = cobs.encodeCobs_cmd_packed(ddprintcommands.CmdBlock, ddprintcommands.CmdBlockPacked, stream)

            if not cobsBlock:
                break

            # cmds.append(( ddprintcommands.CmdBlock, cobsBlock ))
            cmds.append( (cmd, cobsBlock) )

        return cmds

    def debugPlot(self):

        d = {
                "stepType": "bresenham",
                "leadAxis": self.leadAxis,
                "accelPulses": self.accelPulses,
                "linearTimer": self.linearTimer,
                "linearSteps": self.abs_vector_steps[self.leadAxis] - (len(self.accelPulses)+len(self.decelPulses)),
                "decelPulses": self.decelPulses,
                }

        if self.setDirBits:
            d["dirbits"] = self.dirBits

        return d

class RawStepData:

    def __init__(self):
        self.pulses = []
        self.setDirBits = False
        self.dirBits = 0

    def addPulse(self, timerValue, pulse):

        assert(timerValue <= maxTimerValue16)

        if timerValue < 25:
            print("timervalue:", timerValue, pulse)

        assert(timerValue >= 25) # xxx hardcoded 100khz avr
        self.pulses.append((timerValue, pulse))

    def empty(self):
        return not self.pulses

    def __repr__(self):
        return "RawStepData:" + \
           "\n  Direction bits: 0x%x" % self.dirBits + \
           "\n  # pulses: %d" % len(self.pulses)

    # For step bitmask see move.h::st_get_move_bit_mask()
    def stepBits(self, stepBits):

        bits = 0
        for i in range(5):
            if stepBits[i]:
                bits |= (0x1 << i)

        return bits

    # Return a list of binary encoded commands
    # for this RawMove, ready to be send over serial...
    def command(self, move):

        flags = 0
        if self.setDirBits:
            flags = self.dirBits | DirBitsBitRaw

        # Check if timer values can be sent as difference bytes
        timerByteFlag = TimerByteFlagBit
        if self.pulses:
            lastTimer = self.pulses[0][0]
            for (tv, _) in self.pulses[1:]:

                #print "rtv:%d" % tv

                dtv = lastTimer - tv

                if dtv > 127 or dtv < -128:
                    timerByteFlag = 0
                    break

                lastTimer = tv
       
        startMoveFlag = 0
        if move.isStartMove:
            startMoveFlag = MoveStartBitRaw

        measureStartBit = 0
        if move.isMeasureMove:
            measureStartBit = MeasureStartBitRaw

        endMoveBit = 0
        if move.isEndMove:
            endMoveBit = EndMoveBitRaw

        payLoad = struct.pack("<BHH",
                ddprintcommands.CmdG1Raw,
                flags | timerByteFlag | startMoveFlag | measureStartBit | endMoveBit,
                len(self.pulses))

        if move.isStartMove:

            # Store timervalue of max e-speed
            baseMove = move.getBaseMove()

            # xxx move to own method  
            nominalSpeed = abs( baseMove.topSpeed.speed().eSpeed )
            e_steps_per_second_nominal = nominalSpeed * baseMove.e_steps_per_mm
            eTimer = fTimer / e_steps_per_second_nominal

            # print "StartMove, E-Timer:", nominalSpeed, eTimer
            payLoad += intmath.eTimer(min(eTimer, 0xffff)).pack()

        if timerByteFlag:

            (lastTimer, stepBits) = self.pulses[0]
            payLoad += struct.pack("<HB", lastTimer, self.stepBits(stepBits))

            for (tv, stepBits) in self.pulses[1:]:
                dtv = lastTimer - tv
                payLoad += struct.pack("<bB", dtv, self.stepBits(stepBits))
                lastTimer = tv

        else:

            for (etimer, stepBits) in self.pulses:
                assert(etimer >= 25)
                payLoad += struct.pack("<HB", etimer, self.stepBits(stepBits))

        # print "size of payload:", len(payLoad)

        """
        rawPayloadSize = len(payLoad)

        # Test if compression gives smaller packet
        # XXX reuse compressor object?
        compressor = zlib.compressobj(9, zlib.DEFLATED, -15)
        compressor.compress(payLoad)
        compressedPayload = compressor.flush()
        print "encodeCobs_cmd_packed compressed %d blocksize into %d bytes..." % (rawPayloadSize, len(compressedPayload))

        if len(compressedPayload) > rawPayloadSize*0.95:
            print "fixme Sending uncompressed..."
            # return chr(ddprintcommands.CmdG1RawPacked) + compressedPayload
        """

        return payLoad

        # stream = cStringIO.StringIO(payLoad)
        # cobsBlock = cobs.encodeCobs_cmd_packed(stream)
        # cmds = [( ddprintcommands.CmdG1Raw, cobsBlock )]

        cmds = [ cobs.encodeCobs_cmd_packed(ddprintcommands.CmdG1Raw, ddprintcommands.CmdG1RawPacked, stream) ]

        while True:

            # cobsBlock = cobs.encodeCobs_cmd_packed(stream)
            (cmd, cobsBlock) = cobs.encodeCobs_cmd_packed(ddprintcommands.CmdBlock, ddprintcommands.CmdBlockPacked, stream)

            if not cobsBlock:
                break

            # cmds.append(( ddprintcommands.CmdBlock, cobsBlock ))
            cmds.append( (cmd, cobsBlock) )

        return cmds

    def debugPlot(self):

        d = {
                "stepType": "raw",
                "pulses": self.pulses,
                }

        if self.setDirBits:
            d["dirbits"] = self.dirBits

        return d

class AdvanceData:

    def __init__(self, move):

        self.move = move

        # Additional start E-Feedrate if advance applied or 0
        self.startFeedrateIncrease = 0
        # Additional end E-Feedrate if advance applied or 0
        self.endFeedrateIncrease = 0

        self.startSplits = 0
        self.endSplits = 0

        self.startESteps = None
        self.linESteps = None
        self.endESteps = None
        self.endEStepsC = None
        self.endEStepsD = None

        # self.accelGroup = []
        self.sAccel = 0.0
        # self.sAccelSum = 0.0

        # self.decelGroup = []
        self.sDecel = 0.0
        # self.sDecelSum = 0.0

        # Debug, prüfung ob alle in planAdvance() berechneten e-steps in planSteps() 
        # verwendet werden. Summe ist im idealfall 0, kann aber aufgrund von rundungsfehlern
        # auch ungleich null sein.
        self.advStepSum = 0

        # xxx
        # self.hasAccelAdvance = False
        # self.hasDecelAdvance = False

    def _hasStartAdvance(self):
        assert(self.move.accelTime())
        return self.startFeedrateIncrease != 0

    def startEFeedrate(self):
        assert(self.move.accelTime())
        return self.move.startSpeed.speed().eSpeed + self.startFeedrateIncrease

    # xxx rename to startETopFeedrate
    def startEReachedFeedrate(self):
        assert(self.move.accelTime())
        return self.move.topSpeed.speed().eSpeed + self.startFeedrateIncrease

    def _hasEndAdvance(self):
        assert(self.move.decelTime())
        return self.endFeedrateIncrease != 0

    # xxx rename to endETopFeedrate
    def endEReachedFeedrate(self):
        assert(self.move.decelTime())
        return self.move.topSpeed.speed().eSpeed + self.endFeedrateIncrease

    def endEFeedrate(self):

        assert(self.move.decelTime())
        return self.move.endSpeed.speed().eSpeed + self.endFeedrateIncrease

    # Check if sign changes at accel/decel
    def startSignChange(self):
        return util.sign(self.startEFeedrate()) != util.sign(self.startEReachedFeedrate())

    def endSignChange(self):

        v0 = self.endEReachedFeedrate()
        v1 = self.endEFeedrate()

        if util.isclose(v0, v1):
            assert(0)

        if util.isclose(v0, 0) or util.isclose(v1, 0):
            return False

        if v0 >= 0 and v1 >= 0:
            return False

        if v0 < 0 and v1 < 0:
            return False

        return True

    def estepSum(self):

        esteps = 0
        if self.startESteps:
            esteps += self.startESteps
        if self.linESteps:
            esteps += self.linESteps
        if self.endESteps:
            esteps += self.endESteps
        if self.endEStepsC:
            esteps += self.endEStepsC
        if self.endEStepsD:
            esteps += self.endEStepsD

        return esteps

    def __repr__(self):

        s = ""
        if self.move.state > 1:
            if self.move.accelTime() and self._hasStartAdvance():
                s += "\n  EStartAdvance: %.3f, Start %.3f, Top: %.3f" % (self.startFeedrateIncrease, self.startEFeedrate(), self.startEReachedFeedrate())
            if self.move.decelTime() and self._hasEndAdvance():
                s += "\n    EEndAdvance: %.3f, Top %.3f, End: %.3f" % (self.endFeedrateIncrease, self.endEReachedFeedrate(), self.endEFeedrate())

        if self.startESteps:
            s += "\n startESteps: %.3f" % self.startESteps
        if self.linESteps:
            s += "\n linESteps: %.3f" % self.linESteps
        if self.endESteps:
            s += "\n endESteps: %.3f" % self.endESteps
        if self.endEStepsC:
            s += "\n endEStepsC: %.3f" % self.endEStepsC
        if self.endEStepsD:
            s += "\n endEStepsD: %.3f" % self.endEStepsD

        esteps = self.estepSum()
        if esteps:
            s += "\n estep sum: %.3f" % esteps

        # s += "\n Group data:"
        # s += "\n Accel group:" + str(map(lambda m: m.moveNumber, self.accelGroup))
        s += "\n sAccel: %.3f" % self.sAccel
        # s += "\n sAccelSum: %.3f" % self.sAccelSum
        # s += "\n Decel group:" + str(map(lambda m: m.moveNumber, self.decelGroup))
        s += "\n sDecel: %.3f" % self.sDecel
        # s += "\n sDecelSum: %.3f" % self.sDecelSum
        return s

    def sanityCheck(self):

        if self.move.state > 1:

            # XXX assuming no retraction moves with advance
            if self.move.accelTime() and self._hasStartAdvance():
                assert(self.move.startSpeed.speed()[A_AXIS] >= 0)
                assert(self.move.topSpeed.speed()[A_AXIS] > 0)
                assert(self.startFeedrateIncrease >= 0)

            if self.move.decelTime() and self._hasEndAdvance():
                assert(self.move.topSpeed.speed()[A_AXIS] > 0)
                assert(self.move.endSpeed.speed()[A_AXIS] >= 0)
                assert(self.endFeedrateIncrease >= 0)

##################################################
#
#
#
class VelocityOverride(object):

    def __init__(self, nominalSpeed):
        self.speeds = [(nominalSpeed, "initial")]

    def speed(self):
        return self.speeds[-1][0].copy()

    def setSpeed(self, speed, comment):

        # debug
        if speed == self.speeds[-1]:
            print("duplicate speed: ", speed, comment)
            assert(0)

        if speed != self.speeds[-1]:
            self.speeds.append((speed, comment))

    def __repr__(self):

        s = ""

        for (speed, comment) in self.speeds:
            s += "\n\tSpeed: " + str(speed) + " " + comment

        return s

##################################################

class AccelOverride(object):

    def __init__(self, accel, xyzDirection):
        self.accels = [accel]
        self.xyzDirection = xyzDirection

    def xyAccel(self):
        return vectorLength(self.accels[-1][0])

    def eAccel(self):
        return self.accels[-1][1]

    def setAccel(self, xyAccel, eAccel):

        """
        # debug
        if xyAccel == self.xyAccel() and eAccel == self.eAccel():
            print "duplicate accel: ", xyAccel, eAccel
            assert(0)
        """

        self.accels.append([self.xyzDirection.scale(xyAccel), eAccel])

    def accel(self, dim):

        if dim == A_AXIS:
            return self.eAccel()

        return self.accels[-1][0][dim]

    def __repr__(self):

        s = ""

        for (accel3, eAccel) in self.accels:
            s += "\n\tXYZAccel: " + str(accel3) + (" EAccel: %f" % eAccel)

        return s

##################################################

class MoveBase(object):

    def __init__(self):

        self.accelData = AccelData()

        # debug
        self.state = 0 # 1: joined, 2: accel planned, 3: steps planned

        self.moveNumber = None

        self.stepData = None

        self.isStartMove = False

        self.isMeasureMove = False

        # Last move of a path, used to implement
        # soft stop.
        self.isEndMove = False

    def isMove(self):
        return True

    def isSubMove(self):
        return False

    # Returns base move, eg our self
    def getBaseMove(self):
        return self

    def setDuration(self, accelTime, linearTime, decelTime):
        self.accelData.setDuration(accelTime, linearTime, decelTime)

    def accelTime(self):
        return self.accelData.accelTime

    def linearTime(self):

        return self.accelData.linearTime

    def decelTime(self):

        return self.accelData.decelTime

    def getTime(self):

        return self.accelData.getTime()

    # Get vector of absolute steps.
    def absStepsVector(self, disp=None):

        if disp != None:
            return vectorAbs(disp)

        asv = []
        for dim in range(3):
            asv.append(abs(self.displacement_vector_steps_raw3[dim]))

        return asv + [abs(self.eSteps), 0]

    def leadAxis(self, nAxes=5, disp=None):

        asv = self.absStepsVector(disp)
        maxstep = 0
        maxdim = 0
        for dim in range(nAxes):
            if asv[dim] > maxstep:
                maxdim = dim
                maxstep = asv[dim]

        # Use top speed to determine leadAxis if x- and y-axis have the same step amount
        if maxdim == X_AXIS and asv[X_AXIS] == asv[Y_AXIS] and abs(self.topSpeed.speed()[Y_AXIS]) > abs(self.topSpeed.speed()[X_AXIS]):
            return Y_AXIS

        return maxdim

    def initStepData(self, stepDataType):

        if stepDataType == StepDataTypeBresenham:
            self.stepData = StepData()
        elif stepDataType == StepDataTypeRaw:
            self.stepData = RawStepData()
        else:
            assert(0)

    def empty(self):
        return self.stepData.empty()

    # Return binary encoded stepdata, ready to be send over serial...
    def command(self):
        # if self.empty():
        #    self.pprint("empty move")
        #    assert(0)
        return self.stepData.command(self)

    def sanityCheck(self, checkDirection=True):

        ss = self.startSpeed.speed()
        ts = self.topSpeed.speed()
        es = self.endSpeed.speed()

        # All velocities should have reasonable feedrates
        assert(ss.feedrateGEZ())
        assert(ts.feedrateGZ())
        assert(es.feedrateGEZ())

        if checkDirection:

            # All velocities should point into the same direction
            assert(vectorLength(vectorSub(ss.direction, ts.direction)) < 0.001)
            assert(vectorLength(vectorSub(es.direction, ts.direction)) < 0.001)

        self.accelData.sanityCheck()

    def pprint(self, title):

        print("\n------ Move %s, #: %d, '%s' ------" % (title, self.moveNumber, self.comment))

        if self.isPrintMove():
            print("Print-move, distance: %s" % self.distanceStr())
        else:
            print("Travel-move, distance: %s" % self.distanceStr())

        print("displacement_vector:", self.rawDisplacementStr(), "_steps:", self.rawDisplacementStepsStr())

        print("Startspeed: ", end=' ')
        print(self.startSpeed)
        print("Top  speed: ", end=' ')
        print(self.topSpeed)
        print("End  speed: ", end=' ')
        print(self.endSpeed)

        if self.state > 1:
            print("")
            print(self.accelData)

        if self.state > 2:
            print("")
            print(self.stepData)

        print("---------------------")

# Base class for TravelMove and PrintMove
class RealMove(MoveBase):

    def __init__(self, comment, layerPart, pos, printerProfile):

        MoveBase.__init__(self)

        self.comment = comment

        self.accelData = AccelData()

        self.layerPart = layerPart

        self.pos = pos

        self.printerProfile = printerProfile

        # assert(self.layerPart != "infill")

        # debug
        self.state = 0 # 1: joined, 2: accel planned, 3: steps planned

        self.e_steps_per_mm = printerProfile.getStepsPerMMI(A_AXIS)

    def getJerkSpeed(self, jerk):

        return self.topSpeed.speed().constrain(jerk)

    def setPlannedJerkStartSpeed(self, jerk, comment):

        v = self.getJerkSpeed(jerk)
        if v != None:
            self.startSpeed.setSpeed(v, "setPlannedJerkStartSpeed " + comment)

    def setPlannedJerkEndSpeed(self, jerk, comment):

        v = self.getJerkSpeed(jerk)
        if v != None:
            self.endSpeed.setSpeed(v, "setPlannedJerkEndSpeed " + comment)

    def sanityCheck(self):

        MoveBase.sanityCheck(self)

    def isInfill(self):
        return self.layerPart == "infill"

    def hasAdvance(self):
        return not self.isInfill()

    def getPos(self):
        return self.pos

class TravelMove(RealMove):

    def __init__(self,
                 comment,
                 displacement_vector,
                 displacement_vector_steps,
                 feedrate, # mm/s
                 layerPart,
                 pos,
                 printerProfile,
                 ):

        # assert(layerPart != "infill")
        RealMove.__init__(self, comment, layerPart, pos, printerProfile)

        # self.displacement_vector_raw = displacement_vector

        # self.displacement_vector3=displacement_vector[:3]
        # self.displacement_vector_steps3=displacement_vector_steps[:3]
        # self.extrusion_displacement_raw = displacement_vector[3:]
        # self.extrusion_displacement_steps_raw = displacement_vector_steps[3:]

        #
        # Move distance in XYZAB plane
        #
        self.distance5 = displacement_vector.length()

        self.displacement_vector_raw5 = displacement_vector
        self.displacement_vector_steps_raw5 = displacement_vector_steps
        self.direction5 = displacement_vector.normalized()

        v = VelocityVector5(feedrate = feedrate, direction = self.direction5)

        self.startSpeed = VelocityOverride(v)
        self.topSpeed = VelocityOverride(v)
        self.endSpeed = VelocityOverride(v)

    def isPrintMove(self):
        return False

    # def isExtrudingMove(self):
        # return self.eDistance() > 0

    def eDistance(self):
        return self.displacement_vector_raw5[A_AXIS]

    def distanceStr(self):
        d = self.displacement_vector_raw5.length()
        return "%.2f mm (XYZAB)" % d

    def rawDisplacementStr(self):
        return str(self.displacement_vector_raw5)

    def rawDisplacementStepsStr(self):
       return str(self.displacement_vector_steps_raw5)

    # Note: returns positive values 
    def getMaxAllowedAccelVectorNoAdv5(self):

        accelVector = self.direction5.scale(_MAX_ACCELERATION)
        return abs(accelVector.constrain(self.printerProfile.getMaxAxisAccelerationI()) or accelVector)

    # Note: always positive
    def getMaxAllowedAccelNoAdv5(self):

        accelVector = self.getMaxAllowedAccelVectorNoAdv5()
        return accelVector.length() # always positive

    def sanityCheck(self):

        RealMove.sanityCheck(self)

        # Check start ramp
        assert(self.startSpeed.speed().feedrate5() <= self.topSpeed.speed().feedrate5());

        # Check end ramp
        assert(self.topSpeed.speed().feedrate5() >= self.endSpeed.speed().feedrate5());

##################################################

class PrintMove(RealMove):

    def __init__(self,
                 comment,
                 displacement_vector,
                 displacement_vector_steps,
                 feedrate, # mm/s
                 layerPart,
                 maxAccelV,
                 pos,
                 printerProfile
                 ):

        # assert(layerPart != "infill")
        RealMove.__init__(self, comment, layerPart, pos, printerProfile)

        #
        # Move distance in XYZ plane
        #
        self.distance3 = displacement_vector.length(3)

        self.advanceData = AdvanceData(self)

        self.displacement_vector_raw3 = Vector(displacement_vector[:3])
        self.displacement_vector_steps_raw3 = displacement_vector_steps[:3]

        self.direction3 = self.displacement_vector_raw3.normalized()

        ### Apply extrusion adjust
        ### if UseExtrusionAdjust:

        direction5 = displacement_vector.normalized()

        self.eDistance = displacement_vector[A_AXIS]

        # xxx todo: add override
        self.eSteps = displacement_vector_steps[A_AXIS]

        self.eDirection = self.eDistance / self.distance3

        # Compute nominal eSpeed

        v = VelocityVector32(feedrate*self.eDirection, feedrate = feedrate, direction = self.direction3)

        self.startSpeed = VelocityOverride(v)
        self.topSpeed = VelocityOverride(v)
        self.endSpeed = VelocityOverride(v)

        # self.__direction5 = displacement_vector.normalized()
        # av = self.__getMaxAllowedAccelVector5(maxAccelV)

        accelVector = direction5.scale(_MAX_ACCELERATION)
        av = accelVector.constrain(maxAccelV) or accelVector

        # xxx rework accel, store default xyz and eaccel, make start- and eaccel overridable
        self.startAccel = AccelOverride([av[:3], av[A_AXIS]], self.direction3)
        self.endAccel = AccelOverride([av[:3], av[A_AXIS]], self.direction3)

    def isPrintMove(self):
        return True

    # def isExtrudingMove(self):
        # return True

    def distanceStr(self):
        d = self.displacement_vector_raw3.length()
        return "%.2f mm (XYZ)" % d

    def rawDisplacementStr(self):
        return str(self.displacement_vector_raw3) + (" [%.3f]" % self.eDistance)

    def rawDisplacementStepsStr(self):
        return str(self.displacement_vector_steps_raw3) + (" [%.3f]" % self.eSteps)

    # Alle achsen werden in der gleichen zeit beschleunigt.
    # Dadurch teilen sich die zulässigen einzelbeschleunigungen
    # im entsprechenden verhältnis auf.
    # def __getMaxAllowedAccelVector5(self, maxAccelV):
        # accelVector = self.__direction5.scale(_MAX_ACCELERATION)
        # return accelVector.constrain(maxAccelV) or accelVector

    ####### always positive
    ######def __getMaxAllowedAccel5(self, maxAccelV):
        ######accelVector = self.getMaxAllowedAccelVector5(maxAccelV)
        ######return accelVector.length()

    ################################################################################
    # Area (e-distance) of advance start ramp
    # Berechnet die trapezfläche und damit die durch advance zusätzlich
    # extruderstrecke.
    # Annahme: startFeedrateIncrease immer positiv (keine retraction), dadurch
    # return value auch immer positiv.
    def startAdvDistance(self, ta, startFeedrateIncrease=None):

        if startFeedrateIncrease == None:
            startFeedrateIncrease = self.advanceData.startFeedrateIncrease

        # Trapezberechnung
        sadv = startFeedrateIncrease * ta
        assert(sadv > 0)
        return sadv

    # Area (e-distance) of advance end ramp
    # Berechnet die trapezfläche und damit die durch advance verringerte
    # extruderstrecke.
    # Annahme: endFeedrateIncrease immer negativ (keine retraction), dadurch
    # return value auch immer negativ.
    def endAdvDistance(self, td, endFeedrateIncrease=None):

        if endFeedrateIncrease == None:
            endFeedrateIncrease = self.advanceData.endFeedrateIncrease

        # Trapezberechnung, resultat negativ, da endFeedrateIncrease negativ ist
        sadv = endFeedrateIncrease * td
        assert(sadv < 0)
        return sadv
    ################################################################################

    ################################################################################
    def startAdvSteps(self, startFeedrateIncrease=None):

        ta = self.accelTime()

        if not ta:
            return 0.0

        sa = self.startAdvDistance(ta, startFeedrateIncrease)
        return sa

    def endAdvSteps(self, endFeedrateIncrease=None):

        td = self.decelTime()

        if not td:
            return 0.0

        sd = self.endAdvDistance(td, endFeedrateIncrease)
        return sd

    ################################################################################

    ################################################################################
    # Berechnet die fläche des dreieckigen anteils (und damit die strecke) der start-rampe.
    # Vorzeichen:
    #   v0, v1 positiv: resultat positiv
    #   v0, v1 negativ: resultat negativ
    def startRampTriangle(self, v0, v1, dt):

        # print "v0:", v0, "v1:", v1, "dt:", dt

        if not dt:
            return 0.0

        if v1 > 0: 
            assert(v0 >= 0)
            assert(v1 >= v0)
        elif v1 < 0:
            assert(v0 <= 0)
            assert(v1 <= v0)

        return ((v1 - v0) * dt) / 2.0

    def endRampTriangle(self, v0, v1, dt):

        # print "v0:", v0, "v1:", v1, "dt:", dt

        if not dt:
            return 0.0

        if v0 > 0:
            assert(v1 >= 0)
            assert(v0 >= v1)
        if v0 < 0:
            assert(v1 <= 0)
            assert(v0 <= v1)

        return ((v0 - v1) * dt) / 2.0
    ################################################################################

    ################################################################################
    # Berechnet die fläche (und damit die strecke) der start-rampe.
    # Diese besteht aus zwei teilen:
    # * Der rechteckige teil der aus v0*dt besteht
    # * Und dem dreieckigen teil der aus dv*dt besteht.
    # Vorzeichen:
    #   v0, v1 positiv: resultat positiv
    #   v0, v1 negativ: resultat negativ
    def startRampDistance(self, v0, v1, dt):

        return self.startRampTriangle(v0, v1, dt) + v0 * dt

    def endRampDistance(self, v0, v1, dt):

        return self.endRampTriangle(v0, v1, dt) + v1 * dt
    ################################################################################

    ################################################################################
    def startERampDistance(self, ta=None, startFeedrateIncrease=None):

        if ta == None:
            ta = self.accelTime()

        s = self.startRampDistance(
                self.startSpeed.speed().eSpeed,
                self.topSpeed.speed().eSpeed,
                ta)
        return s + self.startAdvDistance(ta, startFeedrateIncrease)

    def endERampDistance(self, td=None, endFeedrateIncrease=None, v0=None, v1=None):

        if td == None:
            td = self.decelTime()

        if v0 == None:
            v0 = self.topSpeed.speed().eSpeed

        if v1 == None:
            v1 = self.endSpeed.speed().eSpeed

        s = self.endRampDistance(v0, v1, td)
        return s + self.endAdvDistance(td, endFeedrateIncrease)
    ################################################################################

    ################################################################################
    def startERampSteps(self, startFeedrateIncrease=None):

        ta = self.accelTime()

        sTri = self.startRampDistance(
                self.startSpeed.speed().eSpeed,
                self.topSpeed.speed().eSpeed,
                ta)

        sPara = self.startAdvDistance(ta, startFeedrateIncrease)

        assert(sPara >= 0)

        sa = sTri + sPara
        esteps = sa * self.e_steps_per_mm

        return esteps

    def endERampSteps(self, td=None, endFeedrateIncrease=None, v0=None, v1=None, roundError=0):

        if td == None:
            td = self.decelTime()

        if endFeedrateIncrease == None:
            endFeedrateIncrease = self.advanceData.endFeedrateIncrease

        if v0 == None:
            v0 = self.topSpeed.speed().eSpeed

        if v1 == None:
            v1 = self.endSpeed.speed().eSpeed

        sTri = self.endRampDistance(v0, v1, td)

        sPara = self.endAdvDistance(td, endFeedrateIncrease)

        # print "sPara: %f" % sPara

        assert(sPara <= 0)

        sd = sTri + sPara
        esteps = sd * self.e_steps_per_mm

        return esteps
    ################################################################################

    # notused ?
    # def isCrossedDecelStep(self):
        # return False

    def getExtrusionVolume(self, matProfile):
        return self.eDistance * matProfile.getMatArea()

    def sanityCheck(self):

        RealMove.sanityCheck(self)

        # Check start ramp
        assert(self.startSpeed.speed().feedrate3() <= self.topSpeed.speed().feedrate3());

        # Check end ramp
        assert(self.topSpeed.speed().feedrate3() >= self.endSpeed.speed().feedrate3());

        self.advanceData.sanityCheck()

    # not used
    def checkAdvance(self):

        # Check direction of start advance increase

        # Check direction of end advance decrease
        pass

    def pprint(self, title):

        RealMove.pprint(self, title)

        print("Start ESpeed: " + self.startSpeed.speed().eSpeedStr())
        print("  End ESpeed: " + self.endSpeed.speed().eSpeedStr())

        print("Allowed Start Acceleration: ", end=' ')
        print(self.startAccel)
        print("Allowed End Acceleration: ", end=' ')
        print(self.endAccel)

        if self.state > 1:
            print(self.advanceData)

        print("---------------------")

##################################################

class SubMove(MoveBase):

    def __init__(self,
                 parentMove,
                 moveNumber,
                 displacement_vector_steps):

        assert(displacement_vector_steps != ([0] * 5))

        # MoveBase.__init__(self, Vector(displacement_vector_steps))
        MoveBase.__init__(self)

        self.parentMove = parentMove

        self.moveNumber = moveNumber

        self.startSpeed = VelocityOverride(None)
        self.topSpeed = VelocityOverride(None)
        self.endSpeed = VelocityOverride(None)

        # self.topSpeed = parentMove.topSpeed

        self.displacement_vector_steps_raw3 = displacement_vector_steps[:3]
        self.eSteps = displacement_vector_steps[A_AXIS]

        self.startAccel = parentMove.startAccel
        self.endAccel = parentMove.endAccel

        self.state = 2

    def isSubMove(self):
        return True

    # Returns base move, eg our parent move
    def getBaseMove(self):
        return self.parentMove

    def setSpeeds(self, sv, tv, ev):

        self.startSpeed.setSpeed(sv, "SubMove.setSpeeds")

        self.topSpeed.setSpeed(tv, "SubMove.setSpeeds")

        self.endSpeed.setSpeed(ev, "SubMove.setSpeeds")

    # xxx method is inlined into  planSteps()
    """
    def isCrossedDecelStep(self):

        v_1 = self.topSpeed.speed().feedrate3()
        v_2 = self.endSpeed.speed().feedrate3()
        xyzSign = util.sign(abs(v_2) - abs(v_1))

        # print "\nisCrossedDecelStep(): v_1: %f, v_2: %f\n" % (v_1, v_2), xyzSign

        ve_1 = self.topSpeed.speed().eSpeed
        ve_2 = self.endSpeed.speed().eSpeed

        eSign = util.sign(abs(ve_2) - abs(ve_1))

        # print "isCrossedDecelStep(): ve_1: %f, ve_2: %f\n" % (ve_1, ve_2), eSign

        if (xyzSign != eSign):
            # print "isCrossedDecelStep, different sign", xyzSign, eSign
            return True

        # print "isCrossedDecelStep false"
        return False
    """

    def pprint(self, title):

        print("\n------ SubMove # %d: %s, Parent #: %d ------" % (self.moveNumber, title, self.parentMove.moveNumber))

        # print "Print-move, distance: %.2f" % self.distance

        print("displacement_vector_steps:", self.displacement_vector_steps_raw3, self.eSteps)

        print("Startspeed: ", end=' ')
        print(self.startSpeed)
        print("Top  speed: ", end=' ')
        print(self.topSpeed)
        print("End  speed: ", end=' ')
        print(self.endSpeed)

        if self.state > 1:
            print("")
            print(self.accelData)

        if self.state > 2:
            print("")
            print(self.stepData)

        print("---------------------")

    def sanityCheck(self):

        # MoveBase.sanityCheck(self, checkDirection=False) # directionCheck not true for advanced moves

        if self.displacement_vector_steps_raw3 == [0, 0, 0] and self.eSteps == 0:
            print("ERROR: null move:")
            self.pprint("Nullmove")
            assert(0)


















