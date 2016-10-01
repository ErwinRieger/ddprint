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
        l =  []
        for el in v:
            l.append("%8.3f" % el)
        return "[" + ", ".join(l)+"]"

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
        assert(scale >= 0)
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

    def nElem(self):
        return len(self.vv)

##################################################
#
# Handles feedrate and direction of a speedvector
#
class VelocityVector5(object):

    def __init__(self, feedrate=None, direction=None, v=None):

        if v:
            assert(v.nElem() == 5)
            self.feedrate = v.length() # always positive
            self.direction = v.normalized()
        else:
            assert(len(direction) == 5)
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

        return VelocityVector5(feedrate = self.feedrate*speedScale, direction = self.direction)
 
    def scale(self, s):
        return VelocityVector(feedrate = self.feedrate * s, direction = self.direction)

    def feedrateGZ(self):
        return self.feedrate > 0

    def feedrateGEZ(self):
        return self.feedrate >= 0

    def copy(self):
        return VelocityVector5(feedrate = self.feedrate, direction = self.direction)

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
#
# Handles feedrate and direction of a speedvector
#
class VelocityVector32(object):

    def __init__(self, eSpeed, feedrate=None, direction=None, v=None):

        # Nominal speed without advance
        self.eSpeed = eSpeed

        if v:
            assert(v.nElem() == 3)
            self._feedrate = v.length() # always positive
            self.direction = v.normalized()
        else:
            assert(len(direction) == 3)
            self._feedrate = feedrate
            self.direction = direction

    def __repr__(self):
        return ("%.3f [mm/s] " % self._feedrate) + str(self.direction.scale(self._feedrate)) + " " + self.eSpeedStr()

    def eSpeedStr(self):
        if self.eSpeed == None:
            return "[-] [mm/s]"
        return "[%.3f] [mm/s] " % self.eSpeed

    def vv3(self):
        return self.direction.scale(self._feedrate)

    def setSpeed(self, feedrate):

        self.eSpeed *= feedrate/self._feedrate
        self._feedrate = feedrate

    # debug catch assignment to self.feedrate
    def __setattr__(self, attr, val):
        assert(attr != "feedrate")
        object.__setattr__(self, attr, val)

    def setESpeed(self, eSpeed):

        self.eSpeed = eSpeed

    def copy(self):
        return VelocityVector32(self.eSpeed, feedrate = self._feedrate, direction = self.direction)

    def scale(self, s):

        return VelocityVector32(self.eSpeed * s, feedrate = self._feedrate * s, direction = self.direction)

    def feedrate3(self):
        return self._feedrate

    def feedrateGZ(self):
        return self.feedrate3() > 0

    def feedrateGEZ(self):
        return self.feedrate3() >= 0

    def __getitem__(self, dim):

        assert(dim < 3)
        return self.direction[dim] * self._feedrate


    """
    # Feedrate in XY direction
    def XY(self):
        return Vector([self[X_AXIS], self[Y_AXIS]]).length()

    def constrain(self, jerkVector):

        speedScale = 1.0
        vv = self.vv()

        for dim in range(5):
            if abs(vv[dim]) > jerkVector[dim]:
                speedScale = min(speedScale, jerkVector[dim] / abs(vv[dim]))

        if abs(1-speedScale) < 0.001:
            return None

        assert(speedScale < 1)

        return VelocityVector(feedrate = self._feedrate*speedScale, direction = self.direction)
    """

    def checkJerk(self, other, jerk, selfName="", otherName=""):
        # xxx add __sub__

        thisv = self.vv3()
        otherv = other.vv3()

        for dim in range(len(thisv)):
            j = abs(otherv[dim] - thisv[dim])
            if (j / jerk[dim]) > 1.001:
                print "Join '%s' <-> '%s': dimension %d %f verletzt max jerk %f" % (selfName, otherName, dim, j, jerk[dim])
                print "V1: ", self
                print "V2: ", other
                assert(0)

        eJerk = self.eSpeed - other.eSpeed
        if not abs(eJerk) <= AdvanceEThreshold:
       
            print "Error, E-AXIS jerk: ", eJerk
            nextMove.pprint("sanityCheck() - nextMove")
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

        self.startESteps = None
        self.linESteps = None
        self.endESteps = None
        self.endEStepsC = None
        self.endEStepsD = None

    def hasStartAdvance(self):
        return self.startFeedrateIncrease != 0

    def startEFeedrate(self):
        return self.move.startSpeed.trueSpeed().eSpeed + self.startFeedrateIncrease

    # xxx rename to startETopFeedrate
    def startEReachedFeedrate(self):
        return self.move.topSpeed.trueSpeed().eSpeed + self.startFeedrateIncrease

    def hasEndAdvance(self):
        return self.endFeedrateIncrease != 0

    # xxx rename to endETopFeedrate
    def endEReachedFeedrate(self):
        return self.move.topSpeed.trueSpeed().eSpeed + self.endFeedrateIncrease

    def endEFeedrate(self):
        return self.move.endSpeed.trueSpeed().eSpeed + self.endFeedrateIncrease

    # Check if sign changes at accel/decel
    def startSignChange(self):
        return sign(self.startEFeedrate()) != sign(self.startEReachedFeedrate())

    def endSignChange(self):
        # return sign(self.endEFeedrate()) != sign(self.endEReachedFeedrate())
        v0 = self.endEReachedFeedrate()
        v1 = self.endEFeedrate()

        if v0 == 0 or v1 == 0:
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
        if self.hasStartAdvance():
            s += "\n  EStartAdvance: %.3f, Start %.3f, Top: %.3f" % (self.startFeedrateIncrease, self.startEFeedrate(), self.startEReachedFeedrate())
        if self.hasEndAdvance():
            s += "\n    EEndAdvance: %.3f, Top %.3f, End: %.3f" % (self.endFeedrateIncrease, self.endEReachedFeedrate(), self.endEFeedrate())

        if self.startESteps:
            s += "\n startESteps: %d" % self.startESteps
        if self.linESteps:
            s += "\n linESteps: %d" % self.linESteps
        if self.endESteps:
            s += "\n endESteps: %d" % self.endESteps
        if self.endEStepsC:
            s += "\n endEStepsC: %d" % self.endEStepsC
        if self.endEStepsD:
            s += "\n endEStepsD: %d" % self.endEStepsD

        esteps = self.estepSum()
        if esteps:
            s += "\n estep sum: %d" % esteps

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

        if speed == self.speeds[-1]:
            print "duplicate speed: ", speed
            assert(0)

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

    def __init__(self):

        self.accelData = AccelData()

        self.stepData = StepData()

        # debug
        self.state = 0 # 1: joined, 2: accel planned, 3: steps planned

        self.moveNumber = None

        self.crossedDecelStep = False

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

        print "\n------ Move %s, #: %d, '%s' ------" % (title, self.moveNumber, self.comment)

        if self.isPrintMove():
            print "Print-move, distance: %s" % self.distanceStr()
        else:
            print "Travel-move, distance: %s" % self.distanceStr()

        print "displacement_vector:", self.rawDisplacementStr(), "_steps:", self.rawDisplacementStepsStr()

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

    def __init__(self, comment):

        MoveBase.__init__(self)

        self.comment = comment

        self.accelData = AccelData()

        self.stepData = StepData()

        # debug
        self.state = 0 # 1: joined, 2: accel planned, 3: steps planned

    def getJerkSpeed(self, jerk):

        return self.topSpeed.nominalSpeed().constrain(jerk)

    def setPlannedJerkStartSpeed(self, jerk):

        v = self.getJerkSpeed(jerk)
        self.startSpeed.plannedSpeed(v)

    def setPlannedJerkEndSpeed(self, jerk):

        v = self.getJerkSpeed(jerk)
        self.endSpeed.plannedSpeed(v)

    def sanityCheck(self):

        MoveBase.sanityCheck(self)

    def xpprint(self, title):

        print "\n------ Move %s, #: %d, '%s' ------" % (title, self.moveNumber, self.comment)

        if self.isPrintMove():
            print "Print-move, distance: %.2f" % self.distance
        else:
            print "Travel-move, distance: %.2f" % self.distance

        print "displacement_vector:", self.rawDisplacementV(), ", steps:", self.rawDisplacementStepsL()

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

        RealMove.__init__(self, comment)

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
        return abs(accelVector.constrain(MAX_AXIS_ACCELERATION_NOADV) or accelVector)

    # Note: always positive
    def getMaxAllowedAccelNoAdv5(self):

        accelVector = self.getMaxAllowedAccelVectorNoAdv5()
        return accelVector.length() # always positive

    def sanityCheck(self, jerk):

        RealMove.sanityCheck(self)

        nullV = VelocityVector5(v=Vector([0, 0, 0, 0, 0]))

        self.startSpeed.trueSpeed().checkJerk(nullV, jerk, "start 0", "#: %d" % self.moveNumber)
        self.endSpeed.trueSpeed().checkJerk(nullV, jerk)

        # Check start ramp
        assert(self.startSpeed.speed().feedrate <= self.topSpeed.speed().feedrate);

        # Check end ramp
        assert(self.topSpeed.speed().feedrate >= self.endSpeed.speed().feedrate);

##################################################

class PrintMove(RealMove):

    def __init__(self,
                 comment,
                 displacement_vector,
                 displacement_vector_steps,
                 feedrate, # mm/s
                 ):

        RealMove.__init__(self, comment)

        # self.displacement_vector_raw = displacement_vector
        # self.displacement_vector_steps_raw = displacement_vector_steps

        # self.displacement_vector3=displacement_vector[:3]
        # self.displacement_vector_steps3=displacement_vector_steps[:3]

        #
        # Move distance in XYZ plane
        #
        self.distance3 = displacement_vector.length(3)

        self.prevMove = None
        self.nextMove = None

        self.advanceData = AdvanceData(self)

        self.e_steps_per_mm = PrinterProfile.getStepsPerMM(A_AXIS)

        self.displacement_vector_raw3 = Vector(displacement_vector[:3])
        self.displacement_vector_steps_raw3 = displacement_vector_steps[:3]
        self.direction5 = displacement_vector.normalized()
        self.direction3 = self.displacement_vector_raw3.normalized()

        self.eDistance = displacement_vector[A_AXIS]
        self.eSteps = displacement_vector_steps[A_AXIS]

        self.eDirection = self.eDistance / self.distance3

        # Compute nominal eSpeed

        v = VelocityVector32(feedrate*self.eDirection, feedrate = feedrate, direction = self.direction3)

        self.startSpeed = VelocityOverride(v)
        self.topSpeed = VelocityOverride(v)
        self.endSpeed = VelocityOverride(v)

    def isPrintMove(self):
        return True

    def distanceStr(self):
        d = self.displacement_vector_raw3.length()
        return "%.2f mm (XYZ)" % d

    def rawDisplacementStr(self):
        return str(self.displacement_vector_raw3) + (" [%.3f]" % self.eDistance)

    def rawDisplacementStepsStr(self):
        return str(self.displacement_vector_steps_raw3) + (" [%.3f]" % self.eSteps)

    def nu_getMaxAllowedAccelVector3(self, maxAccelV):

        accelVector = self.direction3.scale(_MAX_ACCELERATION)
        # return abs(accelVector.constrain(maxAccelV) or accelVector)
        return accelVector.constrain(maxAccelV) or accelVector

    # always positive
    def nu_getMaxAllowedAccel3(self, maxAccelV):

        accelVector = self.getMaxAllowedAccelVector3(maxAccelV)
        return accelVector.length()

    def getMaxAllowedAccelVector5(self, maxAccelV):
        accelVector = self.direction5.scale(_MAX_ACCELERATION)
        return accelVector.constrain(maxAccelV) or accelVector

    # always positive
    def getMaxAllowedAccel5(self, maxAccelV):
        accelVector = self.getMaxAllowedAccelVector5(maxAccelV)
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
    # Berechnet die fläche des dreieckigen anteils (und damit die strecke) der start-rampe.
    # Vorzeichen:
    #   v0, v1 positiv: resultat positiv
    #   v0, v1 negativ: resultat negativ
    def startRampTriangle(self, v0, v1, dt):

        print "v0:", v0, "v1:", v1

        if v1 > 0: 
            assert(v0 >= 0)
            assert(v1 >= v0)
        elif v1 < 0:
            assert(v0 <= 0)
            assert(v1 <= v0)

        return ((v1 - v0) * dt) / 2.0

    def endRampTriangle(self, v0, v1, dt):

        print "v0:", v0, "v1:", v1

        if v0 > 0:
            assert(v1 >= 0)
            assert(v0 >= v1)
        if v0 < 0:
            assert(v1 <= 0)
            assert(v0 <= v1)

        return ((v0 - v1) * dt) / 2.0
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

        # Check start ramp
        assert(self.startSpeed.speed().feedrate3() <= self.topSpeed.speed().feedrate3());

        # Check end ramp
        assert(self.topSpeed.speed().feedrate3() >= self.endSpeed.speed().feedrate3());

        nullV = self.topSpeed.speed()
        nullV.setSpeed(0.1) # xxx use same start speed as addMove() here...

        nextMove = self.nextMove

        if nextMove:

            endSpeed1 = self.endSpeed.speed()
            startSpeed2 = nextMove.startSpeed.speed()
            endSpeed1.checkJerk(startSpeed2, jerk, "#: %d" % self.moveNumber, "#: %d" % nextMove.moveNumber)

        else:

            # Last move
            self.endSpeed.speed().checkJerk(nullV, jerk)

        if not self.prevMove:

            # First move
            self.startSpeed.speed().checkJerk(nullV, jerk, "start 0", "#: %d" % self.moveNumber)

        self.advanceData.sanityCheck()

    def checkAdvance(self):

        # Check direction of start advance increase

        # Check direction of end advance decrease
        pass

    def pprint(self, title):

        RealMove.pprint(self, title)

        print "Start ESpeed: " + self.startSpeed.speed().eSpeedStr()
        print "  End ESpeed: " + self.endSpeed.speed().eSpeedStr()

        print self.advanceData
        print "---------------------"

##################################################

class SubMove(MoveBase):

    def __init__(self,
                 parentMove,
                 moveNumber,
                 displacement_vector_steps):

        # MoveBase.__init__(self, Vector(displacement_vector_steps))
        MoveBase.__init__(self)

        self.parentMove = parentMove

        self.moveNumber = moveNumber

        self.startSpeed = VelocityOverride(None)
        self.topSpeed = VelocityOverride(None)
        self.endSpeed = VelocityOverride(None)

        self.prevMove = None
        self.nextMove = None

        # self.topSpeed = parentMove.topSpeed

        self.displacement_vector_steps_raw3 = displacement_vector_steps[:3]
        self.eSteps = displacement_vector_steps[A_AXIS]

        self.state = 2

    def setSpeeds(self, sv, tv, ev):

        if type(sv) == ListType:
            assert(0)
            sv = Vector(sv)
        # self.startSpeed.nominalSpeed(VelocityVector(v=sv))
        self.startSpeed.nominalSpeed(sv)

        if type(tv) == ListType:
            assert(0)
            tv = Vector(tv)
        # self.topSpeed.nominalSpeed(VelocityVector(v=tv))
        self.topSpeed.nominalSpeed(tv)

        if type(ev) == ListType:
            assert(0)
            ev = Vector(ev)
        # self.endSpeed.nominalSpeed(VelocityVector(v=ev))
        self.endSpeed.nominalSpeed(ev)

    def getMaxAllowedAccelVector5(self, maxAccelV):
        return self.parentMove.getMaxAllowedAccelVector5(maxAccelV)

    # def endSignChange(self):
    #    return self.parentMove.advanceData.endSignChange()

    def pprint(self, title):

        print "\n------ SubMove # %d: %s, Parent #: %d ------" % (self.moveNumber, title, self.parentMove.moveNumber)

        # print "Print-move, distance: %.2f" % self.distance

        print "displacement_vector_steps:", self.displacement_vector_steps_raw3, self.eSteps

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

        if self.displacement_vector_steps_raw3 == [0, 0, 0] and self.eSteps == 0:
            print "ERROR: null move:"
            self.pprint("Nullmove")
            assert(0)


















