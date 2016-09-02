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
from ddprofile import NozzleProfile, MatProfile


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

        # Erreichbare geschwindigkeit falls kein plateu [mm/s]
        # self.reachedovNominalVVector = None

        # Time for the three phases of this move
        self.accelTime = 0
        self.linearTime = 0
        self.deccelTime = 0

    def setDuration(self, accelTime, linearTime, deccelTime):

        assert(accelTime >= 0)
        assert(linearTime >= 0)
        assert(deccelTime >= 0)

        self.accelTime = accelTime
        self.linearTime = linearTime
        self.deccelTime = deccelTime

    def getTime(self):

        return self.accelTime + self.linearTime + self.deccelTime

    def sanityCheck(self):

        assert(self.accelTime >= 0)
        assert(self.linearTime >= 0)
        assert(self.deccelTime >= 0)

    def __repr__(self):

        s = ""
        # if self.reachedovNominalVVector:
            # s = "AccelData:"
            # s += "\n  Nom. speed not reached, Vreach: " + str(self.reachedovNominalVVector)

        s += "\n  AccelTime: %f, LinearTime: %f, DecelTime: %f" % (self.accelTime, self.linearTime, self.deccelTime)
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

    def startEReachedFeedrate(self):
        return self.move.topSpeed.trueSpeed()[A_AXIS] + self.startFeedrateIncrease

    def hasEndAdvance(self):
        return self.endFeedrateIncrease != 0

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
            s += "\n  StartAdvance: Start %.3f Top: %.3f" % (self.startEFeedrate(), self.startEReachedFeedrate())
        if self.hasEndAdvance():
            s += "\n    EndAdvance: Top %.3f End: %.3f" % (self.endEReachedFeedrate(), self.endEFeedrate())
        return s

##################################################
#
#
#
class VelocityOverride(object):

    def __init__(self, nominalSpeed):
        self.speeds = [nominalSpeed]

    def nominalSpeed(self, speed=None):

        if speed:
            self.speeds[0] = speed
            return

        return self.speeds[0].copy()

    def plannedSpeed(self, speed=None):

        if speed:
            if len(self.speeds) == 1:
                self.speeds.append(speed)
            else:
                self.speeds[1] = speed
            return

        if len(self.speeds) < 2:
            return self.nominalSpeed()

        return self.speeds[1].copy()

    def trueSpeed(self, speed=None):

        if speed:
            assert(len(self.speeds) == 2)
            self.speeds.append(speed)
            return

        if len(self.speeds) < 3:
            return self.plannedSpeed()

        return self.speeds[2].copy()

    # def advancedSpeed(self, speed=None):
        # if speed:
            # assert(len(self.speeds) == 3)
            # self.speeds.append(speed)
            # return
        # return self.speeds[3].copy()

    def __repr__(self):

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

    def __init__(self, comment, displacement_vector, displacement_vector_steps, feedrate):

        self.comment = comment
        self.displacement_vector_raw = displacement_vector
        self.displacement_vector_steps_raw = displacement_vector_steps

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

    def setDuration(self, accelTime, linearTime, deccelTime):

        self.accelData.setDuration(accelTime, linearTime, deccelTime)

    def accelTime(self):

        return self.accelData.accelTime

    def linearTime(self):

        return self.accelData.linearTime

    def decelTime(self):

        return self.accelData.deccelTime

    def sanityCheck(self):

        ss = self.startSpeed.trueSpeed()
        ts = self.topSpeed.trueSpeed()
        es = self.endSpeed.trueSpeed()

        # All velocities should have reasonable feedrates
        assert(ss.feedrate >= 0)
        assert(ts.feedrate >  0)
        assert(es.feedrate >= 0)

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

class TravelMove(MoveBase):

    def __init__(self,
                 comment,
                 displacement_vector,
                 displacement_vector_steps,
                 feedrate, # mm/s
                 ):

        MoveBase.__init__(self, comment, displacement_vector, displacement_vector_steps, feedrate)

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

        MoveBase.sanityCheck(self)

        nullV = VelocityVector(v=Vector([0, 0, 0, 0, 0]))

        self.startSpeed.trueSpeed().checkJerk(nullV, jerk, "start 0", "#: %d" % self.moveNumber)
        self.endSpeed.trueSpeed().checkJerk(nullV, jerk)

##################################################

class PrintMove(MoveBase):

    def __init__(self,
                 comment,
                 displacement_vector,
                 displacement_vector_steps,
                 feedrate, # mm/s
                 ):

        MoveBase.__init__(self, comment, displacement_vector, displacement_vector_steps, feedrate)

        # self.displacement_vector3=displacement_vector[:3]
        # self.displacement_vector_steps3=displacement_vector_steps[:3]

        #
        # Move distance in XYZ plane
        #
        self.distance = displacement_vector.length(3)

        self.prevMove = None
        self.nextMove = None

        self.advanceData = AdvanceData(self)

    def isPrintMove(self):
        return True

    # Note: returns positive values 
    def getMaxAllowedAccelVector(self, maxAccelV):

        accelVector = self.direction.scale(_MAX_ACCELERATION)
        return abs(accelVector.constrain(maxAccelV) or accelVector)

    # always positive
    def getMaxAllowedAccel(self, maxAccelV):

        accelVector = self.getMaxAllowedAccelVector(maxAccelV)
        return accelVector.length()

    def sanityCheck(self, jerk):

        MoveBase.sanityCheck(self)

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

    def pprint(self, title):

        MoveBase.pprint(self, title)

        print "Start ESpeed: %.3f" % self.startSpeed.trueSpeed()[A_AXIS]
        print "  End ESpeed: %.3f" % self.endSpeed.trueSpeed()[A_AXIS]

        print self.advanceData
        print "---------------------"

##################################################

class SubMove(object):

    def __init__(self,
                 parentMove,
                 displacement_vector_steps):

        self.parentMove = parentMove
        # self.displacement_vector_steps3=displacement_vector_steps[:3]
        self.extrusion_displacement_steps_raw = displacement_vector_steps[3:]


        self.prevMove = None
        self.nextMove = None

        self.moveNumber = None

    def pprint(self, title):

        print "\n------ SubMove %s, Parent #: %d ------" % (title, self.parentMove.moveNumber)

        # print "Print-move, distance: %.2f" % self.distance

        print "displacement_vector:", displacement_vector_raw, "_steps:", self.displacement_vector_steps_raw

        print "xxx todo SubMove pprint"
        return

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


















