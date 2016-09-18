# -*- coding: utf-8 -*-
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

import math, struct

import ddprintcommands, cobs, cStringIO

from ddprintconstants import maxTimerValue16, maxTimerValue24, fTimer, _MAX_ACCELERATION, MAX_AXIS_ACCELERATION_NOADV
from ddprintconstants import AdvanceEThreshold
from ddprintutil import X_AXIS, Y_AXIS, Z_AXIS, A_AXIS, B_AXIS,vectorLength, vectorMul, vectorSub, circaf, sign
from ddprintcommands import CommandNames
from ddprofile import NozzleProfile, MatProfile, PrinterProfile
from types import ListType


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

# class VVector(object):
class Vector(object):

    def __init__(self, v):
        # if v:
        self.vv = v
        # else:
            # self.vv = 5 * [0.0]

    def __setitem__(self, dim, v):
        self.vv[dim] = v

    def __getitem__(self, dim):
        return self.vv[dim]

    def __getattr__(self, dim):

        dim = dim.upper()
        if dim not in "XYZAB":
            print "unknown dimension requested: ", dim
            assert(0)

        return self.vv["XYZAB".index(dim)]

    def __len__(self):
        return len(self.vv)

    def __repr__(self):
        return self.printSpeedVector(self.vv)

    def printSpeedVector(self, v):
        return "[%8.3f, %8.3f, %8.3f, %8.3f, %8.3f]" % tuple(v)

    def __abs__(self):
        return Vector((abs(self.x), abs(self.y), abs(self.z), abs(self.a), abs(self.b)))

    def __eq__(self, other):

        if other == None:
            return False

        for dim in range(len(self.vv)):
            if not circaf(self[dim], other[dim], 0.000001):
                return False
        return True

    def __ne__(self, other):
        return not self == other

    def checkJerk(self, other, jerk, selfName="", otherName=""):
        # xxx add __sub__

        for dim in range(len(self.vv)):
            j = abs(other[dim] - self[dim])
            if (j / jerk[dim]) > 1.1:
                print "Join '%s' <-> '%s': dimension %d %f verletzt max jerk %f" % (selfName, otherName, dim, j, jerk[dim])
                print "V1: ", self
                print "V2: ", other
                assert(0)

    def length(self, nelem=None):
        return vectorLength(self.vv[:nelem])

    # def len3(self):
        # return self.length(3)

    # def len5(self):
        # return self.length(5)

    def feedrate3(self):
        assert(0)
        f = self.len3()
        if circaf(f, 0, 0.000001):
            print "emove"
            assert(0)
        return f

    def _setLength(self, length):
        return self.normalized().mul(length)

    def cosAngle(self, other):
        #
        #            V1 * V2
        # cos a = ----------------
        #           |V1| + |V2|
        #
        scalarProduct = sum(multiply_vector(self.vv, other.vv))
        lenProduct = calculate_vector_magnitude(self.vv) * calculate_vector_magnitude(other.vv)
        return scalarProduct / lenProduct

    def angleBetween(self, other):
        cos = self.cosAngle(other)

        if cos >= 1:
            return 0

        return math.degrees(math.acos(cos))

    def addVVector(self, other):
        self.vv = vectorAdd(self.vv, other.vv)

    def subVVector(self, other):
        return Vector(vectorSub(self.vv, other.vv))

    def scale(self, scale):
        return Vector(vectorMul(self.vv, len(self.vv)*[scale]))

    mul = scale

    def div(self, other):
        return VVector(vectorDiv(self.vv, other.vv))

    def normalized(self):

        length = self.length()

        if length == 0:
            return Vector([0, 0, 0, 0, 0])

        return self.scale(1.0 / length)
 
    def constrain(self, jerkVector):

        speedScale = 1.0
        for dim in range(len(self.vv)):
            if abs(self.vv[dim]) > jerkVector[dim]:
                speedScale = min(speedScale, jerkVector[dim] / abs(self.vv[dim]))

        # print "speedScale: ", speedScale

        if abs(1-speedScale) < 0.001:
            return None

        assert(speedScale < 1)

        return self.scale(speedScale)
  
    def constrain3(self, jerkVector):

        assert(0)

        speedScale = 1.0
        for dim in range(3):
            if abs(self.vv[dim]) > jerkVector[dim]:
                speedScale = min(speedScale, jerkVector[dim] / abs(self.vv[dim]))

        # print "speedScale: ", speedScale

        assert(speedScale <= 1.0)
        return self.scale(speedScale)
  
    def isDisjointV(self, other, delta=0.000001):

        for dim in range(len(self.vv)):
            if abs(self.vv[dim]) > delta and abs(other.vv[dim]) > delta:
                return False
        return True

##################################################
#
# Handles feedrate and direction of a speedvector
#
class VelocityVector(object):

    def __init__(self, feedrate=None, direction=None, v=None):

        if v:
            self.feedrate = v.length() # always positive
            self.direction = v.normalized()
        else:
            self.feedrate = feedrate
            self.direction = direction

    def __repr__(self):

        return ("%.3f [mm/s] " % self.feedrate) + str(self.direction.scale(self.feedrate)) + " [mm/s]"

    def vv(self):
        return self.direction.scale(self.feedrate)

    # Feedrate in XY direction
    def XY(self):
        return Vector([self[X_AXIS], self[Y_AXIS]]).length()

    def __getitem__(self, dim):

        return self.direction[dim] * self.feedrate

    def constrain(self, jerkVector):

        speedScale = 1.0
        vv = self.vv()

        for dim in range(5):
            if abs(vv[dim]) > jerkVector[dim]:
                speedScale = min(speedScale, jerkVector[dim] / abs(vv[dim]))

        if abs(1-speedScale) < 0.001:
            return None

        assert(speedScale < 1)

        return VelocityVector(feedrate = self.feedrate*speedScale, direction = self.direction)
 
    def scale(self, s):
        return VelocityVector(feedrate = self.feedrate * s, direction = self.direction)

    def copy(self):
        return VelocityVector(feedrate = self.feedrate, direction = self.direction)

    def checkJerk(self, other, jerk, selfName="", otherName=""):
        # xxx add __sub__

        thisv = self.vv()
        otherv = other.vv()

        for dim in range(len(thisv)):
            j = abs(otherv[dim] - thisv[dim])
            if (j / jerk[dim]) > 1.001:
                print "Join '%s' <-> '%s': dimension %d %f verletzt max jerk %f" % (selfName, otherName, dim, j, jerk[dim])
                print "V1: ", self
                print "V2: ", other
                assert(0)

##################################################

class AccelData:

    def __init__(self):

        pass

        # Erreichbare geschwindigkeit falls kein plateu [mm/s]
        # self.reachedovNominalVVector = None

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

        s = ""
        # if self.reachedovNominalVVector:
            # s = "AccelData:"
            # s += "\n  Nom. speed not reached, Vreach: " + str(self.reachedovNominalVVector)

        s += "\n  AccelTime: %f, LinearTime: %f, DecelTime: %f" % (self.accelTime, self.linearTime, self.decelTime)
        return s

class StepData:

    def __init__(self):
        self.accelPulses = []
        self.linearTimer = None
        self.deccelPulses = []
        self.setDirBits = False
        self.dirBits = 0
        self.leadAxis = 0
        self.abs_vector_steps = None

    def checkTimerValue(self, timer, limit=maxTimerValue24):

        if timer < 50:
            print "timervalue %d to low, > 40kHz!" % timer
            assert(0)

        if timer > limit:
            print "timervalue %d to high, counter overflow!" % timer
            assert(0)

    def addAccelPulse(self, timer):
        self.checkTimerValue(timer)
        self.accelPulses.append(timer)

    def setLinTimer(self, timer):
        self.checkTimerValue(timer, maxTimerValue16)
        self.linearTimer = timer
    
    def addDeccelPulse(self, timer):
        self.checkTimerValue(timer)
        self.deccelPulses.insert(0, timer)

    def setBresenhamParameters(self, leadAxis, abs_vector_steps):
        self.leadAxis = leadAxis
        self.abs_vector_steps = abs_vector_steps

    def __repr__(self):
        return "StepData:" + \
           "\n  Direction bits: 0x%x" % self.dirBits + \
           "\n  # leadAxis: %d" % self.leadAxis + \
           "\n  # abs_vector_steps: %s" % `self.abs_vector_steps` + \
           "\n  # accelPulses: %d" % len(self.accelPulses) + \
           "\n  # linearPulses: %d" % (self.abs_vector_steps[self.leadAxis] - (len(self.accelPulses) + len(self.deccelPulses))) + \
           "\n  # deccelPulses: %d" % len(self.deccelPulses)

    def checkLen(self, deltaLead):
        assert(self.abs_vector_steps[self.leadAxis] - (len(self.accelPulses) + len(self.deccelPulses)) >= 0)

class AdvanceData:

    def __init__(self, move):

        self.move = move

        # Additional start E-Feedrate if advance applied or 0
        self.startFeedrateIncrease = 0
        # Additional end E-Feedrate if advance applied or 0
        self.endFeedrateIncrease = 0

        self.startSplits = 0
        self.endSplits = 0

    def hasStartAdvance(self):
        return self.startFeedrateIncrease != 0

    def startEFeedrate(self):
        return self.move.startSpeed.trueSpeed()[A_AXIS] + self.startFeedrateIncrease

    # xxx rename to startETopFeedrate
    def startEReachedFeedrate(self):
        return self.move.topSpeed.trueSpeed()[A_AXIS] + self.startFeedrateIncrease

    def hasEndAdvance(self):
        return self.endFeedrateIncrease != 0

    # xxx rename to endETopFeedrate
    def endEReachedFeedrate(self):
        return self.move.topSpeed.trueSpeed()[A_AXIS] + self.endFeedrateIncrease

    def endEFeedrate(self):
        return self.move.endSpeed.trueSpeed()[A_AXIS] + self.endFeedrateIncrease

    # Check if sign changes at accel/decel
    def startSignChange(self):
        return sign(self.startEFeedrate()) != sign(self.startEReachedFeedrate())

    def endSignChange(self):
        return sign(self.endEFeedrate()) != sign(self.endEReachedFeedrate())

    def __repr__(self):

        s = ""
        if self.hasStartAdvance():
            s += "\n  EStartAdvance: %.3f, Start %.3f, Top: %.3f" % (self.startFeedrateIncrease, self.startEFeedrate(), self.startEReachedFeedrate())
        if self.hasEndAdvance():
            s += "\n    EEndAdvance: %.3f, Top %.3f, End: %.3f" % (self.endFeedrateIncrease, self.endEReachedFeedrate(), self.endEFeedrate())
        return s

    def sanityCheck(self):

        # XXX assuming no retraction moves with advance
        if self.hasStartAdvance():
            assert(self.move.startSpeed.speed()[A_AXIS] >= 0)
            assert(self.move.topSpeed.speed()[A_AXIS] > 0)
            assert(self.startFeedrateIncrease >= 0)

        if self.hasEndAdvance():
            assert(self.move.topSpeed.speed()[A_AXIS] > 0)
            assert(self.move.endSpeed.speed()[A_AXIS] >= 0)
            assert(self.endFeedrateIncrease >= 0)

##################################################
#
#
#
class VelocityOverride(object):

    def __init__(self, nominalSpeed):
        self.speeds = [nominalSpeed]

    def speed(self):
        return self.speeds[-1].copy()

    def setSpeed(self, speed):

        if speed != self.speeds[-1]:
            self.speeds.append(speed)

    def nominalSpeed(self, speed=None):

        if speed:
            self.setSpeed(speed)
            return

        return self.speed()

    def plannedSpeed(self, speed=None):
        return self.nominalSpeed(speed)

    def trueSpeed(self, speed=None):
        return self.nominalSpeed(speed)

    def __repr__(self):

        s = ""

        for speed in self.speeds:
            s += "\n\tSpeed: " + str(speed)

        return s

        s = "\n\tNominal: " + str(self.nominalSpeed())
        if len(self.speeds) > 1:
            s += "\n\tPlanned: " + str(self.plannedSpeed())
        if len(self.speeds) > 2:
            s += "\n\tTrue: " + str(self.trueSpeed())
        # if len(self.speeds) > 3:
            # s += "\n\tADV: " + str(self.advancedSpeed())

        return s

##################################################

class MoveBase(object):

    def __init__(self, displacement_vector_steps):

        # self.displacement_vector_raw = displacement_vector
        self.displacement_vector_steps_raw = displacement_vector_steps

        self.accelData = AccelData()

        self.stepData = StepData()

        # debug
        self.state = 0 # 1: joined, 2: accel planned, 3: steps planned

        self.moveNumber = None

    def setDuration(self, accelTime, linearTime, decelTime):

        self.accelData.setDuration(accelTime, linearTime, decelTime)

    def accelTime(self):

        return self.accelData.accelTime

    def linearTime(self):

        return self.accelData.linearTime

    def decelTime(self):

        return self.accelData.decelTime

    # return a list of binary encoded commands, ready to be send over serial...
    def commands(self):

        cmds = []

        payLoad = ""
        cmdOfs = 0

        if self.stepData.setDirBits:
            payLoad += struct.pack("<B", self.stepData.dirBits | 0x80)
            cmdOfs = 1

        use24Bits = False
        if (self.stepData.accelPulses and self.stepData.accelPulses[0] > maxTimerValue16) or \
           (self.stepData.deccelPulses and self.stepData.deccelPulses[-1] > maxTimerValue16):
            use24Bits = True

        payLoad += struct.pack("<BiiiiiH",
                self.stepData.leadAxis,
                self.stepData.abs_vector_steps[0],
                self.stepData.abs_vector_steps[1],
                self.stepData.abs_vector_steps[2],
                self.stepData.abs_vector_steps[3],
                self.stepData.abs_vector_steps[4],
                len(self.stepData.accelPulses))

        if not use24Bits:
            print "commands(): ignoring isExtrudingMove!"
            if False: # self.isExtrudingMove(A_AXIS):
                leadFactor = int((self.stepData.abs_vector_steps[self.stepData.leadAxis]*1000) / self.stepData.abs_vector_steps[A_AXIS])
                payLoad += struct.pack("<H", min(leadFactor, 0xffff))
            else:
                payLoad += struct.pack("<H", 0)

        payLoad += struct.pack("<HH", 
                self.stepData.linearTimer,
                len(self.stepData.deccelPulses))

        if use24Bits:

            cmdHex = ddprintcommands.CmdG1_24 + cmdOfs

            for timer in self.stepData.accelPulses:
                timerCount = timer / 0xffff
                payLoad += struct.pack("<B", timerCount)

                rest = timer - (timerCount * 0xffff)
                payLoad += struct.pack("<H", max(50, rest))

            for timer in self.stepData.deccelPulses:
                timerCount = timer / 0xffff
                payLoad += struct.pack("<B", timerCount)

                rest = timer - (timerCount * 0xffff)
                payLoad += struct.pack("<H", max(50, rest))
        else:

            cmdHex = ddprintcommands.CmdG1 + cmdOfs

            for timer in self.stepData.accelPulses:
                payLoad += struct.pack("<H", timer)
            for timer in self.stepData.deccelPulses:
                payLoad += struct.pack("<H", timer)


        stream = cStringIO.StringIO(payLoad)

        cobsBlock = cobs.encodeCobs(stream)
        cmds.append(( cmdHex, cobsBlock ))

        while True:

            cobsBlock = cobs.encodeCobs(stream)

            if not cobsBlock:
                break

            cmds.append(( ddprintcommands.CmdBlock, cobsBlock ))

        return cmds

    def sanityCheck(self, checkDirection=True):

        ss = self.startSpeed.trueSpeed()
        ts = self.topSpeed.trueSpeed()
        es = self.endSpeed.trueSpeed()

        # All velocities should have reasonable feedrates
        assert(ss.feedrate >= 0)
        assert(ts.feedrate >  0)
        assert(es.feedrate >= 0)

        if checkDirection:

            # All velocities should point into the same direction
            assert(vectorLength(vectorSub(ss.direction, ts.direction)) < 0.001)
            assert(vectorLength(vectorSub(es.direction, ts.direction)) < 0.001)

        self.accelData.sanityCheck()

    def pprint(self, title):

        print "\n------ Move %s, #: %d, '%s' ------" % (title, self.moveNumber, self.comment)

        if self.isPrintMove():
            print "Print-move, distance: %.2f" % self.distance
        else:
            print "Travel-move, distance: %.2f" % self.distance

        print "displacement_vector:", self.rawDisplacementV(), "_steps:", self.rawDisplacementStepsL()

        print "Startspeed: ",
        print self.startSpeed
        print "Top  speed: ",
        print self.topSpeed
        print "End  speed: ",
        print self.endSpeed

        if self.state > 1:
            print ""
            print self.accelData

        if self.state > 2:
            print ""
            print self.stepData

        print "---------------------"

# Base class for TravelMove and PrintMove
class RealMove(MoveBase):

    def __init__(self, comment, displacement_vector, displacement_vector_steps, feedrate):

        MoveBase.__init__(self, displacement_vector_steps)

        self.comment = comment
        self.displacement_vector_raw = displacement_vector

        self.direction = displacement_vector.normalized()
        v = VelocityVector(feedrate = feedrate, direction = self.direction)

        self.startSpeed = VelocityOverride(v)
        self.topSpeed = VelocityOverride(v)
        self.endSpeed = VelocityOverride(v)

        self.accelData = AccelData()

        self.stepData = StepData()

        # debug
        self.state = 0 # 1: joined, 2: accel planned, 3: steps planned

    def rawDisplacementV(self):
        return self.displacement_vector_raw

    def rawDisplacementStepsL(self):
       return self.displacement_vector_steps_raw

    def getJerkSpeed(self, jerk):

        return self.topSpeed.nominalSpeed().constrain(jerk)

    def setPlannedJerkStartSpeed(self, jerk):

        v = self.getJerkSpeed(jerk)
        self.startSpeed.plannedSpeed(v)

    def setPlannedJerkEndSpeed(self, jerk):

        v = self.getJerkSpeed(jerk)
        self.endSpeed.plannedSpeed(v)

    # Note: returns positive values 
    def getMaxAllowedAccelVectorNoAdv(self):

        accelVector = self.direction.scale(_MAX_ACCELERATION)
        return abs(accelVector.constrain(MAX_AXIS_ACCELERATION_NOADV) or accelVector)

    # Note: always positive
    def getMaxAllowedAccelNoAdv(self):

        accelVector = self.getMaxAllowedAccelVectorNoAdv()
        return accelVector.length() # always positive

    def sanityCheck(self):

        MoveBase.sanityCheck(self)

        # Check start ramp
        assert(self.startSpeed.speed().feedrate <= self.topSpeed.speed().feedrate);
        # Check end ramp
        assert(self.topSpeed.speed().feedrate >= self.endSpeed.speed().feedrate);

    def xpprint(self, title):

        print "\n------ Move %s, #: %d, '%s' ------" % (title, self.moveNumber, self.comment)

        if self.isPrintMove():
            print "Print-move, distance: %.2f" % self.distance
        else:
            print "Travel-move, distance: %.2f" % self.distance

        print "displacement_vector:", self.rawDisplacementV(), "_steps:", self.rawDisplacementStepsL()

        print "Startspeed: ",
        print self.startSpeed
        print "Top  speed: ",
        print self.topSpeed
        print "End  speed: ",
        print self.endSpeed

        if self.state > 1:
            print ""
            print self.accelData

        if self.state > 2:
            print ""
            print self.stepData

        print "---------------------"

class TravelMove(RealMove):

    def __init__(self,
                 comment,
                 displacement_vector,
                 displacement_vector_steps,
                 feedrate, # mm/s
                 ):

        RealMove.__init__(self, comment, displacement_vector, displacement_vector_steps, feedrate)

        # self.displacement_vector3=displacement_vector[:3]
        # self.displacement_vector_steps3=displacement_vector_steps[:3]
        # self.extrusion_displacement_raw = displacement_vector[3:]
        # self.extrusion_displacement_steps_raw = displacement_vector_steps[3:]

        #
        # Move distance in XYZAB plane
        #
        self.distance = displacement_vector.length()

    def isPrintMove(self):
        return False

    def sanityCheck(self, jerk):

        RealMove.sanityCheck(self)

        nullV = VelocityVector(v=Vector([0, 0, 0, 0, 0]))

        self.startSpeed.trueSpeed().checkJerk(nullV, jerk, "start 0", "#: %d" % self.moveNumber)
        self.endSpeed.trueSpeed().checkJerk(nullV, jerk)

##################################################

class PrintMove(RealMove):

    def __init__(self,
                 comment,
                 displacement_vector,
                 displacement_vector_steps,
                 feedrate, # mm/s
                 ):

        RealMove.__init__(self, comment, displacement_vector, displacement_vector_steps, feedrate)

        # self.displacement_vector3=displacement_vector[:3]
        # self.displacement_vector_steps3=displacement_vector_steps[:3]

        #
        # Move distance in XYZ plane
        #
        # self.distance = displacement_vector.length(3)
        self.distance = displacement_vector.length()

        self.prevMove = None
        self.nextMove = None

        self.advanceData = AdvanceData(self)

        self.e_steps_per_mm = PrinterProfile.getStepsPerMM(A_AXIS)

    def isPrintMove(self):
        return True

    def getMaxAllowedAccelVector(self, maxAccelV):

        accelVector = self.direction.scale(_MAX_ACCELERATION)
        # return abs(accelVector.constrain(maxAccelV) or accelVector)
        return accelVector.constrain(maxAccelV) or accelVector

    # always positive
    def getMaxAllowedAccel(self, maxAccelV):

        accelVector = self.getMaxAllowedAccelVector(maxAccelV)
        return accelVector.length()

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
    def startAdvSteps(self, startFeedrateIncrease, roundError=0):

        ta = self.accelTime()

        sa = self.startAdvDistance(ta, startFeedrateIncrease) + roundError
        esteps = int(sa * self.e_steps_per_mm)
        # esteps = int(round(sa * self.e_steps_per_mm))
        ediff = sa - (esteps / float(self.e_steps_per_mm))

        return (sa, esteps, ediff)

    def endAdvSteps(self, endFeedrateIncrease, roundError=0):

        td = self.decelTime()

        sd = self.endAdvDistance(td, endFeedrateIncrease) + roundError
        esteps = int(sd * self.e_steps_per_mm)
        # esteps = int(round(sd * self.e_steps_per_mm))
        ediff = sd - (esteps / float(self.e_steps_per_mm))

        return (sd, esteps, ediff)
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

        print "v0:", v0, "v1:", v1

        if v1 > 0: 
            assert(v0 >= 0)
            assert(v1 > v0)
        elif v1 < 0:
            assert(v0 <= 0)
            assert(v1 < v0)

        return ((v1 - v0) * dt) / 2.0 + v0 * dt

    def endRampDistance(self, v0, v1, dt):

        print "v0:", v0, "v1:", v1

        if v0 > 0:
            assert(v1 >= 0)
            assert(v0 > v1)
        if v0 < 0:
            assert(v1 <= 0)
            assert(v0 < v1)

        return ((v0 - v1) * dt) / 2.0 + v1 * dt
    ################################################################################

    ################################################################################
    def startERampDistance(self, ta=None, startFeedrateIncrease=None):

        if ta == None:
            ta = self.accelTime()

        s = self.startRampDistance(
                self.startSpeed.speed()[A_AXIS], 
                self.topSpeed.speed()[A_AXIS],
                ta)
        return s + self.startAdvDistance(ta, startFeedrateIncrease)

    def endERampDistance(self, td=None, endFeedrateIncrease=None, v0=None, v1=None):

        if td == None:
            td = self.decelTime()

        if v0 == None:
            v0 = self.topSpeed.speed()[A_AXIS]

        if v1 == None:
            v1 = self.endSpeed.speed()[A_AXIS]

        s = self.endRampDistance(v0, v1, td)
        return s + self.endAdvDistance(td, endFeedrateIncrease)
    ################################################################################

    ################################################################################
    def startERampSteps(self, startFeedrateIncrease=None, roundError=0):

        sa = self.startERampDistance(startFeedrateIncrease=startFeedrateIncrease) + roundError
        # esteps = int(round(sa * self.e_steps_per_mm))
        esteps = int(sa * self.e_steps_per_mm)
        ediff = sa - (esteps / float(self.e_steps_per_mm))

        return (sa, esteps, ediff)

    def endERampSteps(self, td=None, endFeedrateIncrease=None, v0=None, v1=None, roundError=0):

        sd = self.endERampDistance(td, endFeedrateIncrease, v0=v0, v1=v1) + roundError
        # esteps = int(round(sd * self.e_steps_per_mm))
        esteps = int(sd * self.e_steps_per_mm)
        ediff = sd - (esteps / float(self.e_steps_per_mm))

        return (sd, esteps, ediff)
    ################################################################################


    def sanityCheck(self, jerk):

        RealMove.sanityCheck(self)

        nullV = VelocityVector(v=Vector([0, 0, 0, 0, 0]))

        nextMove = self.nextMove

        if nextMove:

            endSpeed1 = self.endSpeed.trueSpeed()
            startSpeed2 = nextMove.startSpeed.trueSpeed()
            endSpeed1.checkJerk(startSpeed2, jerk, "#: %d" % self.moveNumber, "#: %d" % nextMove.moveNumber)

            eJerk = endSpeed1[A_AXIS] - startSpeed2[A_AXIS]
            if not abs(eJerk) <= AdvanceEThreshold:
           
                print "Error, E-AXIS jerk: ", eJerk
                nextMove.pprint("sanityCheck() - nextMove")
                assert(0)

        else:

            # Last move
            self.endSpeed.trueSpeed().checkJerk(nullV, jerk)

        if not self.prevMove:

            # First move
            self.startSpeed.trueSpeed().checkJerk(nullV, jerk, "start 0", "#: %d" % self.moveNumber)

        self.advanceData.sanityCheck()

    def checkAdvance(self):

        # Check direction of start advance increase

        # Check direction of end advance decrease
        pass

    def pprint(self, title):

        RealMove.pprint(self, title)

        print "Start ESpeed: %.3f" % self.startSpeed.trueSpeed()[A_AXIS]
        print "  End ESpeed: %.3f" % self.endSpeed.trueSpeed()[A_AXIS]

        print self.advanceData
        print "---------------------"

##################################################

class SubMove(MoveBase):

    def __init__(self,
                 parentMove,
                 moveNumber,
                 displacement_vector_steps):

        MoveBase.__init__(self, Vector(displacement_vector_steps))

        self.parentMove = parentMove

        self.moveNumber = moveNumber

        self.startSpeed = VelocityOverride(None)
        self.topSpeed = VelocityOverride(None)
        self.endSpeed = VelocityOverride(None)

        self.prevMove = None
        self.nextMove = None

        # self.topSpeed = parentMove.topSpeed

        self.state = 2

    def setSpeeds(self, sv, tv, ev):

        if type(sv) == ListType:
            sv = Vector(sv)
        self.startSpeed.nominalSpeed(VelocityVector(v=sv))

        if type(tv) == ListType:
            tv = Vector(tv)
        self.topSpeed.nominalSpeed(VelocityVector(v=tv))

        if type(ev) == ListType:
            ev = Vector(ev)
        self.endSpeed.nominalSpeed(VelocityVector(v=ev))

    # XXX not exact, only for planSteps
    def getMaxAllowedAccel(self, maxAccelV):

        return self.parentMove.getMaxAllowedAccel(maxAccelV)

    def pprint(self, title):

        print "\n------ SubMove # %d: %s, Parent #: %d ------" % (self.moveNumber, title, self.parentMove.moveNumber)

        # print "Print-move, distance: %.2f" % self.distance

        print "displacement_vector_steps:", self.displacement_vector_steps_raw

        print "Startspeed: ",
        print self.startSpeed
        print "Top  speed: ",
        print self.topSpeed
        print "End  speed: ",
        print self.endSpeed

        if self.state > 1:
            print ""
            print self.accelData

        if self.state > 2:
            print ""
            print self.stepData

        print "---------------------"

    def sanityCheck(self):

        # MoveBase.sanityCheck(self, checkDirection=False) # directionCheck not true for advanced moves

        if self.displacement_vector_steps_raw.length() == 0:
            self.pprint("sanityCheck")
            assert(0)


















