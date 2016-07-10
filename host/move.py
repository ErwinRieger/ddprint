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

from ddprintconstants import maxTimerValue16, maxTimerValue24, DEFAULT_ACCELERATION, DEFAULT_MAX_ACCELERATION, fTimer
from ddprintutil import X_AXIS, Y_AXIS, Z_AXIS, A_AXIS, B_AXIS,vectorLength, vectorMul, vectorSub, circaf
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

class VVector(object):

    def __init__(self, v=None):
        if v:
            self.vv = v
        else:
            self.vv = 5 * [0.0]

    def __setitem__(self, dim, v):
        self.vv[dim] = v

    def __getitem__(self, dim):
        return self.vv[dim]

    def __getattr__(self, dim):
        return self.vv["XYZAB".index(dim.upper())]

    def __len__(self):
        return len(self.vv)

    def __repr__(self):
        return self.printSpeedVector(self.vv)

    def printSpeedVector(self, v):
        return "[%8.3f, %8.3f, %8.3f, %8.3f, %8.3f]" % tuple(v)

    def __abs__(self):
        return VVector((abs(self.x), abs(self.y), abs(self.z), abs(self.a), abs(self.b)))

    def __eq__(self, other):

        if other == None:
            return False

        for dim in range(5):
            if not circaf(self[dim], other[dim], 0.000001):
                return False
        return True

    def __ne__(self, other):
        return not self == other

    def checkJerk(self, other, jerk, selfName="", otherName=""):
        # xxx add __sub__

        for dim in range(5):
            j = abs(other[dim] - self[dim])
            if (j / jerk[dim]) > 1.1:
                print "Join '%s' <-> '%s': dimension %d %f verletzt max jerk %f" % (selfName, otherName, dim, j, jerk[dim])
                print "V1: ", self
                print "V2: ", other
                assert(0)

    def len3(self):
        return vectorLength(self.vv[:3])

    def len5(self):
        return vectorLength(self.vv)

    def feedrate3(self):
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
        return VVector(vectorSub(self.vv, other.vv))

    def scale(self, scale):
        return VVector(vectorMul(self.vv, 5*[scale]))

    mul = scale

    def normalized(self):
        length = self.len5()
        assert(length)
        if length != 0:
            return self.scale(1.0 / length)

        # xxx not reached
        return Vec3d(self)
 
    def constrain(self, jerkVector):

        speedScale = 1.0
        for dim in range(5):
            if abs(self.vv[dim]) > jerkVector[dim]:
                speedScale = min(speedScale, jerkVector[dim] / abs(self.vv[dim]))

        # print "speedScale: ", speedScale

        if abs(1-speedScale) < 0.001:
            return None

        assert(speedScale < 1)

        return self.scale(speedScale)
  
    def constrain3(self, jerkVector):

        speedScale = 1.0
        for dim in range(3):
            if abs(self.vv[dim]) > jerkVector[dim]:
                speedScale = min(speedScale, jerkVector[dim] / abs(self.vv[dim]))

        # print "speedScale: ", speedScale

        assert(speedScale <= 1.0)
        return self.scale(speedScale)
  
    def isDisjointV(self, other, delta=0.000001):

        for dim in range(5):
            if abs(self.vv[dim]) > delta and abs(other.vv[dim]) > delta:
                return False
        return True
  
class AccelData:
    def __init__(self):
        # Erreichbare geschwindigkeit falls kein plateu [mm/s]
        self.reachedovNominalVVector = None

    def __repr__(self):
        if self.reachedovNominalVVector:
            s = "AccelData:"
            s += "\n  Nom. speed not reached, Vreach: " + str(self.reachedovNominalVVector)
            return s

        return ""

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

##################################################

class Move(object):

    def __init__(self,
                 comment,
                 # stepped_point,
                 displacement_vector,
                 displacement_vector_steps,
                 feedrate, # mm/s
                 ):

        self.comment=comment
        self.displacement_vector3=displacement_vector[:3]
        self.displacement_vector_steps3=displacement_vector_steps[:3]
        self.extrusion_displacement_raw = displacement_vector[3:]
        self.extrusion_displacement_steps_raw = displacement_vector_steps[3:]

        #
        # Move distance in XYZAB plane
        #
        self.distance = displacement_vector.len5()
        #
        # Move distance in XYZ plane
        #
        self.distance3 = displacement_vector.len3()

        # self.eOnly = self.displacement_vector3 == 3*[0]
        self.eOnly = self.distance3 == 0

        #
        # Limit feedrate by maximum extrusion rate
        #
        self.feedrateS = feedrate # mm/s

        # Do not count very small moves, the computation of the extrusion rate is inaccurate because of the
        # discretization in the gcodeparser (map float values to discrete stepper values).
        if self.isPrintMove() and self.distance3 >= 0.1:

            matArea = MatProfile.getMatArea()

            t = self.distance3 / feedrate

            extrusionVolume = displacement_vector[A_AXIS] * matArea
            extrusionRate = extrusionVolume / t
            # print "extrusionRate:", displacement_vector[A_AXIS], t, extrusionRate

            if extrusionRate > NozzleProfile.getNetMaxExtrusionRate():

                print "Warning, extrusion rate to high: %.1f mm³/s, Move: '%s', len: %.3f., extrusionVolume: %.5f" % (extrusionRate, comment, self.distance3, extrusionVolume)
                # xxx feedrate adjust disabled
                ### print "Warning, extrusion rate to high: %.1f mm³/s, reducing to %.1f mm³/s, Move: '%s', len: %.3f., extrusionVolume: %.5f" % (extrusionRate, NozzleProfile.getNetMaxExtrusionRate(), comment, self.distance3, extrusionVolume)
                ### self.feedrateS = feedrate * (NozzleProfile.getNetMaxExtrusionRate() / extrusionRate)
                # print "Adjusted feedrate: ", feedrate, self.feedrateS

        self.trueStartSpeed = self.nominalStartSpeed = None
        self.trueEndSpeed = self.nominalEndSpeed = None

        self.accelData = AccelData()

        self.stepData = StepData()

        self.lastMove = None
        self.nextMove = None

        self.moveNumber = 0

        # debug
        self.state = 0 # 1: joined, 2: accel planned, 3: steps planned
        self.streamed = False

        # Time for the three phases of this move
        self.accelTime = 0
        self.linearTime = 0
        self.deccelTime = 0

    def displacement_vector_raw(self):
        return VVector(self.displacement_vector3 + self.extrusion_displacement_raw)

    def displacement_vector_steps_raw(self):
       return self.displacement_vector_steps3 + self.extrusion_displacement_steps_raw 

    # [mm/s]
    def getFeedrateV(self, feedrateS = None):

        if feedrateS == None:
            feedrateS = self.feedrateS
       
        assert(feedrateS)

        if self.eOnly:

            move_seconds = vectorLength(self.extrusion_displacement_raw) / feedrateS
            return VVector([0.0, 0.0, 0.0] + vectorMul(self.extrusion_displacement_raw, 2 * [1.0 / move_seconds]))

        else:

            # move_seconds = self.distance / feedrateS
            move_seconds = vectorLength(self.displacement_vector3) / feedrateS
            return self.displacement_vector_raw().scale(1.0 / move_seconds)

    # Get the reached speedvector, this is the nominal plateau speed if the move is long enough, else it is 
    # the reached peek speed. Returns a speed vector.
    def getReachedSpeedV(self):
        return self.accelData.reachedovNominalVVector or self.getFeedrateV()

    # [mm/s]
    def _vDim(self, axis):
        return self.vVector()[axis]

    def getAllowedAccelVector(self):

        # accelVector = self.vVector().setLength(DEFAULT_ACCELERATION)
        accelVector = self.displacement_vector_raw()._setLength(DEFAULT_ACCELERATION)
        return accelVector.constrain(DEFAULT_MAX_ACCELERATION) or accelVector

    def getAllowedAccel(self):

        accelVector = self.getAllowedAccelVector()
        allowedAccel = accelVector.len5() # always positive

        return allowedAccel

    def sanityCheck(self, jerk):

        nextMove = self.nextMove
        dirVE = self.getFeedrateV(self.getEndFr())
        nullV = VVector((0, 0, 0, 0, 0))

        if nextMove:
            nextDirVS = nextMove.getFeedrateV(nextMove.getStartFr())
            dirVE.checkJerk(nextDirVS, jerk, "#: %d" % self.moveNumber, "#: %d" % nextMove.moveNumber)

        else:

            # Last move
            dirVE.checkJerk(nullV, jerk)

        if not self.lastMove:

            # First move
            dirVS = self.getFeedrateV(self.getStartFr())
            nullV.checkJerk(dirVS, jerk, "start 0", "#: %d" % self.moveNumber)

    def pprint(self, title):

        print "\n------ Move %s, #: %d, '%s' ------" % (title, self.moveNumber, self.comment)

        if self.isPrintMove():
            print "Print-move, distance: %.2f" % self.distance
        else:
            print "Travel-move, distance: %.2f" % self.distance

        print "displacement_vector:", self.displacement_vector_raw(), "_steps:", self.displacement_vector_steps_raw()
        print "feedrate:", self.feedrateS, "[mm/s], nominalVVector:", self.getFeedrateV(), "[mm/s]"
        print ""

        if self.trueStartSpeed or self.nominalStartSpeed:
            print "\n  Startspeed: ",
            if self.trueStartSpeed != None:
                print "True: %.3f <= " % self.trueStartSpeed,
            else:
                print "Nominal: %.3f" % self.nominalStartSpeed

        if self.trueEndSpeed or self.nominalEndSpeed:
            print "\n  Endspeed: ",
            if self.trueEndSpeed != None:
                print "True: %.3f <= " % self.trueEndSpeed,
            else:
                print "Nominal: %.3f" % self.nominalEndSpeed,


        if self.state > 1:
            print ""
            print self.accelData

        if self.state > 2:
            print ""
            print self.stepData
        print "---------------------"

    def getStartFr(self):

        if self.trueStartSpeed:
            return self.trueStartSpeed

        assert(self.nominalStartSpeed != None)
        return self.nominalStartSpeed

    def getEndFr(self):

        if self.trueEndSpeed:
            return self.trueEndSpeed

        assert(self.nominalEndSpeed != None)
        return self.nominalEndSpeed

    def setNominalStartFr(self, fr):

        self.nominalStartSpeed = fr

    def setNominalEndFr(self, fr):

        self.nominalEndSpeed = fr

    def setTrueStartFr(self, fr):

        self.trueStartSpeed = fr

    def setTrueEndFr(self, fr):
        
        self.trueEndSpeed = fr

    def getJerkSpeed(self, jerk):

        if self.eOnly:
            return min(self.feedrateS, jerk[A_AXIS])

        fv = self.getFeedrateV()
        v = fv.constrain(jerk)
        if v:
            return v.feedrate3()

        return fv.feedrate3()

    def setNominalJerkStartSpeed(self, jerk):

        f = self.getJerkSpeed(jerk)
        self.setNominalStartFr(f)

    def setNominalJerkEndSpeed(self, jerk):

        f = self.getJerkSpeed(jerk)
        self.setNominalEndFr(f)

    #####################################################################################################################################################
    # Move-type, Matrix aller möglichen kombinationen:
    # X Y Z E Movetype
    # - - - - --------
    # 1 0 0 0 travel
    # 0 1 0 0 travel
    # 1 1 0 0 travel
    # 0 0 1 0 travel
    # 1 0 1 0 travel
    # 0 1 1 0 travel
    # 1 1 1 0 travel
    # 0 0 0 1 travel
    # 1 0 0 1 print
    # 0 1 0 1 print
    # 1 1 0 1 print
    # 0 0 1 1 invalid
    # 1 0 1 1 invalid
    # 0 1 1 1 invalid
    # 1 1 1 1 invalid
    #
    def _isHeadMove(self):
        return self.displacement_vector3[X_AXIS] or self.displacement_vector3[Y_AXIS] or self.displacement_vector3[Z_AXIS]

    def _isExtrudingMove(self, extruder):
        return self._isHeadMove() and self.extrusion_displacement_raw[extruder-3]

    def _isZMove(self):
        return self.displacement_vector3[Z_AXIS]

    def isPrintMove(self):

        printMove = self._isHeadMove() and (self._isExtrudingMove(A_AXIS) or self._isExtrudingMove(B_AXIS))

        if printMove:
            assert(not self._isZMove())

        return printMove

    #####################################################################################################################################################

    def setDuration(self, accelTime, linearTime, deccelTime):
        self.accelTime = accelTime
        self.linearTime = linearTime
        self.deccelTime = deccelTime

    def getTime(self):

        return self.accelTime + self.linearTime + self.deccelTime

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
            if self.isExtrudingMove(A_AXIS):
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

    def isDisjointSteps(self, other, delta=2):

        for dim in range(5):
            if abs(self.displacement_vector_steps_raw()[dim]) > delta and abs(other.displacement_vector_steps_raw()[dim]) > delta:
                return False
        return True

    def getAdjustedExtrusionVolume(self, extruder, nozzleProfile, matProfile):
        return self.displacement_vector_adjusted(nozzleProfile, matProfile)[extruder] * matProfile.getMatArea()

    def displacement_vector_adjusted(self, nozzleProfile, matProfile):

        raw = self.displacement_vector_raw()

        if not self.isExtrudingMove(A_AXIS) and not self.isExtrudingMove(B_AXIS):
            return raw

        matArea = matProfile.getMatArea()
        t = self.getTime()

        extrusionRate = (raw[A_AXIS] * matArea) / t

        #
        # Do not increase extrusion rate if it's already above the limit.
        #
        if extrusionRate > nozzleProfile.getNetMaxExtrusionRate():
            return raw

        extrusionAdjustFactor = nozzleProfile.getExtrusionAdjustFactor()

        # print "extrusionRate:", extrusionRate
        adjustedRate = extrusionRate + pow(extrusionRate, 2) * extrusionAdjustFactor
        # print "adjustedRate:", adjustedRate
        raw[A_AXIS] = (adjustedRate * t) / matArea

        extrusionRate = (raw[B_AXIS] * matArea) / t
        adjustedRate = extrusionRate + pow(extrusionRate, 2) * extrusionAdjustFactor
        raw[B_AXIS] = (adjustedRate * t) / matArea

        return raw

    def displacement_vector_steps_adjusted(self, nozzleProfile, matProfile, printerProfile):

        raw = self.displacement_vector_steps_raw()

        if not self.isPrintMove():
            return raw

        adjusted_displacement = self.displacement_vector_adjusted(nozzleProfile, matProfile)

        raw = raw[:3]
        raw.append(int(adjusted_displacement[3] * printerProfile.getStepsPerMM(3)))
        raw.append(int(adjusted_displacement[4] * printerProfile.getStepsPerMM(4)))
        return raw






















