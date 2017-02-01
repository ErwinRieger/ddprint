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

import math

from ddprintconstants import AdvanceEThreshold

####################################################################################################
def vectorAdd(v1, v2):

    sum = []
    for a, b in zip(v1, v2):
        sum.append(a + b)

    return sum

####################################################################################################
def vectorSub(v1, v2):

    diff = []
    for a, b in zip(v1, v2):
        diff.append(a - b)

    return diff

####################################################################################################
def vectorLength(vv):

    sum = 0
    for v in vv:
        sum += pow(v, 2)
    return math.sqrt(sum)

####################################################################################################
def vectorMul(v1, v2):

    product = []
    for i in range(len(v2)):
        product.append(v1[i] * v2[i])

    return product

####################################################################################################
def vectorDiv(v1, v2):

    product = []
    for i in range(len(v2)):
        product.append(v1[i] / v2[i])

    return product

####################################################################################################
def vectorDistance(a, b):
    return vectorLength(vectorSub(a, b))

####################################################################################################
def vectorAbs(v):
    absv = []
    for val in v:
        absv.append(abs(val))
    return absv
####################################################################################################

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
        # return Vector((abs(self.x), abs(self.y), abs(self.z), abs(self.a), abs(self.b)))
        return Vector(vectorAbs(self.vv))

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

        if abs(1-speedScale) < 0.000001:
            return None

        assert(speedScale < 1)

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

    def setSpeed(self, feedrate):
        self.feedrate = feedrate

    # debug catch assignment to self.feedrate
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

        if abs(1-speedScale) < 0.000001:
            return None

        assert(speedScale < 1)

        return VelocityVector5(feedrate = self.feedrate*speedScale, direction = self.direction)
 
    def scale(self, s):
        return VelocityVector5(feedrate = self.feedrate * s, direction = self.direction)

    def feedrate5(self):
        return self.feedrate

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

    def constrain(self, jerkVector):

        speedScale = 1.0
        vv = self.vv3()

        for dim in range(3):
            if abs(vv[dim]) > jerkVector[dim]:
                speedScale = min(speedScale, jerkVector[dim] / abs(vv[dim]))

        if abs(1-speedScale) < 0.000001:
            return None

        assert(speedScale < 1)

        return VelocityVector32(self.eSpeed, feedrate = self._feedrate*speedScale, direction = self.direction)
 
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
       
            print "Error, E-AXIS jerk: ", eJerk, self, other
            assert(0)

##################################################














