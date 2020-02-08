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

import struct, time, math, tty, termios, sys, types, json
import ddprintconstants, ddhome, ddadvance, pprint, movingavg

from ddprintcommands import *
from ddprintstates import *
from ddprintconstants import *
from ddconfig import *
from ddprofile import PrinterProfile, MatProfile, NozzleProfile
from ddvector import vectorMul

####################################################################################################
# XXX todo: read from printer profile
# UM2:
FILAMENT_REVERSAL_LENGTH = 750

####################################################################################################
def sign(x):
    if x == 0:
        return 1.0
    return math.copysign(1, x)

####################################################################################################
def circaf(a, b, delta):
    return abs(a-b) < delta

####################################################################################################

# Speed after accelerating for a certain time
def vAccelPerTime(v0, a, t):
    # v = a * t + v0
    return a * t + v0

####################################################################################################

# Distance traveled after accelerating for a certain time
def accelDist(v0, a, t):
    # s = 0,5 * a * t² + vo * t
    s = 0.5 * a * pow(t, 2) + v0 * t

    # if s < 0:
        # print "Error in accelDist(), negative distance computed: v0=%.3f, a=%.3f, t=%.3f, s=%.3f" % (v0, a, t, s)
        # assert(0)
    return s

####################################################################################################

# Speed after acceleration along a certain distance
def vAccelPerDist(v0, a, s):

    assert(v0 >= 0)

    t1 = pow(v0, 2)
    t2 = 2.0 * a * s

    # print "t1: ", t1
    # print "t2: ", t2

    if (t1 + t2) >= 0:
        return math.sqrt(t1+t2)

    return - math.sqrt(abs(t1 + t2))

####################################################################################################

# Time needed to do deltaV in given distance s
def timePerDist(v1, v2, s):

    assert(v2 >= v1)
    return s / (v1 + ((v2 - v1)/2.0))

####################################################################################################

# Compute acceleration by distance and delta-v
def accelPerDist(v1, v2, s):

    # a = ( ve² - v0² ) / ( 2 * s )
    assert(v2 >= v1)
    return ( pow(v2, 2) - pow(v1, 2) ) / ( 2 * s )

####################################################################################################

def joinMoves(move1, move2, advInstance): # jerk, maxAccelV):

        if debugMoves:
            print "***** Start joinMoves() *****"
            move1.pprint("JoinSpeed - Move1")
            move2.pprint("JoinSpeed - Move2")

        startSpeed1 = move1.startSpeed.speed()
        startSpeedS1 = startSpeed1.feedrate3()

        endSpeed1 = move1.endSpeed.speed()
        endSpeedS1 = endSpeed1.feedrate3()

        startSpeed2 = move2.startSpeed.speed()
        startSpeedS2 = startSpeed2.feedrate3()

        allowedAccel3 = move1.startAccel.xyAccel()

        maxEndSpeed1 = vAccelPerDist(startSpeedS1, allowedAccel3, move1.distance3)

        if maxEndSpeed1 < endSpeedS1:

            # Endspeed of move 1 not reachable, lowering it to the max. reachable speed
            if debugMoves:
                print "Max. reachable endspeed: %.3f < feedrate: %.3f" % (maxEndSpeed1, endSpeedS1)

            endSpeed1.setSpeed(maxEndSpeed1)

            # print "Move1, endspeed lowered: ", endSpeed1
            move1.endSpeed.setSpeed(endSpeed1, "joinMoves - max. reachable endspeed")

        # Check max reachable e endspeed
        maxAllowedEEndSpeed = vAccelPerDist(startSpeed1.eSpeed, move1.startAccel.eAccel(), move1.eDistance)
        if maxAllowedEEndSpeed < endSpeed1.eSpeed:
            assert(circaf(maxAllowedEEndSpeed, endSpeed1.eSpeed, 0.000000001))

        joinMoves2(move1, move2, advInstance)

def joinMoves2(move1, move2, advInstance): # jerk):

        endSpeed1 = move1.endSpeed.speed()
        eEndSpeed1 = endSpeed1.eSpeed

        startSpeed2 = move2.startSpeed.speed()
        eStartSpeed2 = startSpeed2.eSpeed

        # print "joinMoves2(): move 1   end e speed: ", eEndSpeed1
        # print "joinMoves2(): move 2 start e speed: ", eStartSpeed2

        #
        # Compare E-speed of moves
        #
        # Note: normally we would compare eEndSpeed1 and eStartSpeed2 here.
        # But this is a problem with feederCompensation and move.sanityCheck() with the
        # increased E feedrate values (the difference of compensated endspeed/startspeed could be
        # more than AdvanceEThreshold).
        # Therefore we compare the compensated values here:
        #
        # if circaf(eEndSpeed1, eStartSpeed2, AdvanceEThreshold):
        #
        adjEEndSpeed1 = advInstance.eComp(eEndSpeed1)
        adjEStartSpeed2 = advInstance.eComp(eStartSpeed2)
        if circaf(adjEEndSpeed1, adjEStartSpeed2, AdvanceEThreshold):

            # E-speed difference is small enough, check X/Y jerk
            endSpeedV1 = endSpeed1.vv3()
            startSpeedV2 = startSpeed2.vv3()
            differenceVector = endSpeedV1.subVVector(startSpeedV2)
            # print "Case1, differenceVector, jerk:", differenceVector, jerk

##################
            # old joinMoves

            #
            # Join in bezug auf den maximalen jerk aller achsen betrachten:
            #
            jerk = advInstance.planner.getJerk()
            speedDiff = {}
            toMuch = False
            for dim in range(3):

                vdim = startSpeedV2[dim]

                # Speed difference from endspeed to theoretical max speed for axis
                vdiff = vdim - endSpeedV1[dim]

                vdiffAbs = abs(vdiff)

                dimJerk = jerk[dim]

                if vdiffAbs > dimJerk: 
                    toMuch = True

                if vdiffAbs > 1:
                    speedDiff[dim] = (vdiff, vdiffAbs)

            if toMuch:

                if debugMoves:
                    print "E-speed ok, XY-speedDiff:", speedDiff

                # 
                # Der geschwindigkeitsunterschied mindestens einer achse ist grösser als der 
                # erlaubte jerk fuer diese achse.
                # 
                # Man könnte auch sagen: der "geschwindigkeits knick" ist zu gross. 
                # 
                # Letzter step muss also am ende gebremst werden und dieser step muss mit
                # entsprechend kleiner geschwindigkeit gestartet werden.
                #
                # Abbremsen auf 0 geht auf jeden fall, aber ist auch eine höhere
                # jerk geschwindigkeit möglich? Bzw. wie kann diese berechnet werden?
                # 
                # 

                #
                # Skalierungsfaktor berechnen um den die beiden geschwindigkeiten
                # reduziert werden müssen, damit sie nur noch einen unterschied von
                # 'jerk' haben.
                #

                speedScale = 1.0
                for dim in speedDiff.keys():
                    # print "diff: ", differenceVector[dim], jerk[dim]
                    if abs(differenceVector[dim]) > jerk[dim]:
                        # print "mindiff: ", dimNames[dim], differenceVector[dim], jerk[dim]
                        speedScale = min(speedScale, jerk[dim] / abs(differenceVector[dim]))

                if debugMoves:
                    move1.pprint("JoinMoves - Move1")
                    move2.pprint("JoinMoves - Move2")
                    print "speedScale: ", speedScale

                assert(speedScale <= 1.0)

                endSpeed1 = endSpeed1.scale(speedScale)

                if debugMoves:
                    print "set nominal endspeed of move1:", endSpeed1

                move1.endSpeed.setSpeed(endSpeed1, "joinMoves2 - adjust jerk")

                startSpeed2 = startSpeed2.scale(speedScale)

                if debugMoves:
                    print "set nominal startspeed of move2:", speedScale

                move2.startSpeed.setSpeed(startSpeed2, "joinMoves2 - adjust jerk")

            else:

                if debugMoves:
                    print "Doing a full speed join between move %d and %d" % (move1.moveNumber, move2.moveNumber)

                # move1.setNominalEndFr(endSpeedS)
                # move2.setNominalStartFr(move2.feedrateS)

            move1.sanityCheck(jerk)
            return

##################

        joinMoves3(move1, move2, advInstance)
     
def joinMoves3(move1, move2, advInstance): # jerk):

        endSpeed1 = move1.endSpeed.speed()
        startSpeed2 = move2.startSpeed.speed()

        eEndSpeed1 = endSpeed1.eSpeed
        eStartSpeed2 = startSpeed2.eSpeed

        # print "joinMoves3(): e-feedrate 1: ", eEndSpeed1
        # print "joinMoves3(): e-feedrate 2: ", eStartSpeed2

        if eEndSpeed1 > eStartSpeed2:
            # Slow down move1
            f = eStartSpeed2 / eEndSpeed1
            # print "f1: ", f

            # endSpeedS1 *= f
            # endSpeed1.feedrate = endSpeedS1
            endSpeed1 = endSpeed1.scale(f)
            move1.endSpeed.setSpeed(endSpeed1, "joinMoves3 - adjust ejerk")

            # print "slowed down move1 endspeed:", eEndSpeed1

        else:
            # Slow down move2
            f = eEndSpeed1 / eStartSpeed2
            # print "f2: ", f

            # startSpeedS2 *= f
            # startSpeed2.feedrate = startSpeedS2
            startSpeed2 = startSpeed2.scale(f)
            move2.startSpeed.setSpeed(startSpeed2, "joinMoves3 - adjust ejerk")

            # print "slowed down move2 startspeed:", startSpeed2

        #
        # Join in bezug auf den maximalen jerk aller achsen betrachten:
        #
        endSpeedV1 = endSpeed1.vv3()
        startSpeedV2 = startSpeed2.vv3()

        jerk = advInstance.planner.getJerk()
        speedDiff = {}
        toMuch = False
        for dim in range(3):

            vdim = startSpeedV2[dim]

            # Speed difference from endspeed to theoretical max speed for axis
            vdiff = vdim - endSpeedV1[dim]

            vdiffAbs = abs(vdiff)

            dimJerk = jerk[dim]

            if vdiffAbs > dimJerk: 
                toMuch = True

            if vdiffAbs > 1:
                speedDiff[dim] = (vdiff, vdiffAbs)

        if toMuch:

            differenceVector = endSpeedV1.subVVector(startSpeedV2)
            # print "differenceVector, jerk:", differenceVector, jerk

            if debugMoves:
                print "E-speed angepasst, XY-speedDiff:", speedDiff

            # 
            # Der geschwindigkeitsunterschied mindestens einer achse ist grösser als der 
            # erlaubte jerk fuer diese achse.
            # 
            # Man könnte auch sagen: der "geschwindigkeits knick" ist zu gross. 
            # 
            # Letzter step muss also am ende gebremst werden und dieser step muss mit
            # entsprechend kleiner geschwindigkeit gestartet werden.
            #
            # Abbremsen auf 0 geht auf jeden fall, aber ist auch eine höhere
            # jerk geschwindigkeit möglich? Bzw. wie kann diese berechnet werden?
            # 
            # 

            #
            # Skalierungsfaktor berechnen um den die beiden geschwindigkeiten
            # reduziert werden müssen, damit sie nur noch einen unterschied von
            # 'jerk' haben.
            #

            speedScale = 1.0
            for dim in speedDiff.keys():
                # print "diff: ", differenceVector[dim], jerk[dim]
                if abs(differenceVector[dim]) > jerk[dim]:
                    # print "mindiff: ", dimNames[dim], differenceVector[dim], jerk[dim]
                    speedScale = min(speedScale, jerk[dim] / abs(differenceVector[dim]))

            if debugMoves:
                move1.pprint("JoinMoves - Move1")
                move2.pprint("JoinMoves - Move2")
                print "speedScale: ", speedScale

            assert(speedScale <= 1.0)

            endSpeed1 = endSpeed1.scale(speedScale)

            if debugMoves:
                print "set nominal endspeed of move1:", endSpeed1

            move1.endSpeed.setSpeed(endSpeed1, "joinMoves3 - adjust jerk")

            startSpeed2 = startSpeed2.scale(speedScale)

            if debugMoves:
                print "set nominal startspeed of move2:", speedScale

            move2.startSpeed.setSpeed(startSpeed2, "joinMoves3 - adjust jerk")

        else:

            if debugMoves:
                print "Doing a full speed join between move %d and %d" % (move1.moveNumber, move2.moveNumber)

            # move1.setNominalEndFr(endSpeedS)
            # move2.setNominalStartFr(move2.feedrateS)

        move1.sanityCheck(jerk)
##################
        if debugMoves:
            move1.pprint("Move1, e-adjusted")
            move2.pprint("Move2, e-adjusted")
            print "***** End joinMoves() *****"


####################################################################################################

#
# Join travel moves, this is easier than joining printing moves since E-axis jerk has not to be
# considered.
#
def joinTravelMoves(move1, move2, jerk):

        if debugMoves:
            print "***** Start joinTravelMoves() *****"

        allowedAccel = move1.getMaxAllowedAccelNoAdv5()

        startSpeedMove1 = move1.startSpeed.speed()
        startSpeedMove1S = startSpeedMove1.feedrate5()

        endSpeedMove1 = move1.endSpeed.speed()
        endSpeedVMove1 = endSpeedMove1.vv()

        startSpeedMove2 = move2.startSpeed.speed()

        # Compute max reachable endspeed of move1
        maxEndSpeed = vAccelPerDist(startSpeedMove1S, allowedAccel, move1.distance5)

        """
        if endSpeedVMove1.isDisjointV(startSpeedMove2.vv()):

            # Set endspeed to minimum of reachable endspeed and jerkspeed
            jerkSpeed = move1.getJerkSpeed(jerk) or endSpeedMove1

            print "maxEndSpeed of first disjoint move:", maxEndSpeed, jerkSpeed.feedrate5()
            endSpeedMove1.setSpeed(min(jerkSpeed.feedrate5(), maxEndSpeed))
            move1.endSpeed.setSpeed(endSpeedMove1, "joinTravelMoves - disjoint, jerk ormaxendspeed")

            move2.setPlannedJerkStartSpeed(jerk, "joinTravelMoves - disjoint, jerk")

            if debugMoves:
                print "***** End joinTravelMoves() *****"
            return

        """

        endSpeedMove1S = endSpeedMove1.feedrate5()

        if maxEndSpeed < endSpeedMove1S:

            if debugMoves:
                print "Max. reachable endspeed: %.3f < feedrate: %.3f" % (maxEndSpeed, endSpeedMove1S)

            endSpeedMove1S = maxEndSpeed
            endSpeedMove1.setSpeed(endSpeedMove1S)
            move1.endSpeed.setSpeed(endSpeedMove1, "joinTravelMoves - max. reachable endspeed")
            endSpeedVMove1 = endSpeedMove1.vv()

        startSpeedVMove2 = startSpeedMove2.vv()
        differenceVector = startSpeedVMove2.subVVector(endSpeedVMove1)

        #
        # Join in bezug auf den maximalen jerk aller achsen betrachten:
        #
        speedDiff = {}
        toMuch = False
        for dim in range(5):

            vdim = startSpeedVMove2[dim]

            # Speed difference from start speed to theoretical max speed for axis
            vdiff = vdim - endSpeedVMove1[dim]

            vdiffAbs = abs(vdiff)

            dimJerk = jerk[dim]

            if vdiffAbs > dimJerk: 
                toMuch = True

            if vdiffAbs > 1:
                speedDiff[dim] = (vdiff, vdiffAbs)

        if debugMoves:
            print "speedDiff:", speedDiff

        if toMuch:

            # 
            # Der geschwindigkeitsunterschied mindestens einer achse ist grösser als der 
            # erlaubte jerk fuer diese achse.
            # 
            # Man könnte auch sagen: der "geschwindigkeits knick" ist zu gross. 
            # 
            # Letzter step muss also am ende gebremst werden und dieser step muss mit
            # entsprechend kleiner geschwindigkeit gestartet werden.
            #
            # Abbremsen auf 0 geht auf jeden fall, aber ist auch eine höhere
            # jerk geschwindigkeit möglich? Bzw. wie kann diese berechnet werden?
            # 
            # 

            #
            # Skalierungsfaktor berechnen um den die beiden geschwindigkeiten
            # reduziert werden müssen, damit sie nur noch einen unterschied von
            # 'jerk' haben.
            #

            speedScale = 1.0
            for dim in speedDiff.keys():
                # print "diff: ", differenceVector[dim], jerk[dim]
                if abs(differenceVector[dim]) > jerk[dim]:
                    # print "mindiff: ", dimNames[dim], differenceVector[dim], jerk[dim]
                    speedScale = min(speedScale, jerk[dim] / abs(differenceVector[dim]))

            if debugMoves:
                move1.pprint("joinTravelMoves - Move1")
                move2.pprint("joinTravelMoves - Move2")
                print "speedScale: ", speedScale

            assert(speedScale <= 1.0)

            if debugMoves:
                print "set nominal endspeed of move1:", endSpeedMove1S * speedScale
            move1.endSpeed.setSpeed(endSpeedMove1.scale(speedScale), "joinTravelMoves - adjust jerk")

            if debugMoves:
                print "set nominal startspeed of move2:", startSpeedMove2.feedrate5() * speedScale
            move2.startSpeed.setSpeed(startSpeedMove2.scale(speedScale), "joinTravelMoves - adjust jerk")

        else:

            if debugMoves:
                print "Doing a full speed join between move %d and %d" % (move1.moveNumber, move2.moveNumber)

            # move1.setNominalEndFr(endSpeedMove1S)
            # move2.setNominalStartFr(move2.feedrateS)
      
        if debugMoves:
            print "***** End joinTravelMoves() *****"

####################################################################################################

# Move object without references to help garbage collection.
class StreamedMove:
        pass

####################################################################################################


class MyPoint:

    def __init__(self, X=0, Y=0, Z=0, A=0, B=0):

        self.X = X
        self.Y = Y
        self.Z = Z
        self.A = A
        self.B = B

    def equal(self, other, dimsToCompare="XYZAB"):

        for dim in dimsToCompare:
            if getattr(self, dim) != getattr(other, dim):
                return False
        return True
    
    # def __iter__(self):
        # return iter("XYZAB")

    def __getitem__(self, dim):

        if type(dim) == types.StringType:
            return getattr(self, dim)

        if type(dim) == types.SliceType:
            if not dim.start and dim.stop == 3 and dim.step == None:
                return [self.X, self.Y, self.Z]
            print "slice: ", dim
            assert(0)

        if dim == 0:
            return self.X
        elif dim == 1:
            return self.Y
        elif dim == 2:
            return self.Z
        elif dim == 3:
            return self.A
        elif dim == 4:
            return self.B

        print "unknown index:", dim
        assert(0)

    def __setitem__(self, dim, val):

        if type(dim) == types.StringType:
            return setattr(self, dim, val)

        if dim == 0:
            self.X = val
        elif dim == 1:
            self.Y = val
        elif dim == 2:
            self.Z = val
        elif dim == 3:
            self.A = val
        elif dim == 4:
            self.B = val
        else:
            assert(0)

    def __repr__(self):
        return str(self.vector())

    def vector(self):
        return map(lambda dim: self[dim], range(5))

    def copy(self):

        new = MyPoint()
        new.__dict__ = self.__dict__.copy()

        return new

####################################################################################################

class GetChar:

    def __init__(self, msg):
        self.msg = msg
        self.fd = sys.stdin.fileno()
        try:
            self.old = termios.tcgetattr(self.fd)
        except termios.error:
            self.old = None

    def getc(self):

        print self.msg
        try:
            tty.setcbreak(self.fd)
        except termios.error:
            self.old = None
        ch = sys.stdin.read(1)
        if self.old:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        return ch

####################################################################################################

class Average:

    def __init__(self):
        self._sum = 0.0
        self._n = 0

    def add(self, value):
        self._sum += value
        self._n += 1

    def value(self):
        return self._sum / self._n

    # def setValue(self, v):
        # self._value = v

####################################################################################################

class EWMA:

    def __init__(self, weight):
        self.weight = weight
        self._value = 0.0

    def add(self, value):
        """Adds a value to the series and updates the moving average."""
        self._value = (value*self.weight) + (self._value*(1 - self.weight))

    def value(self):
        return self._value

    def setValue(self, v):
        self._value = v

####################################################################################################

# Compute new stepper direction bits
def directionBits(disp, curDirBits):

    for i in range(5):

        mask = 1 << i

        if disp[i] > 0 and not (curDirBits & mask):
            curDirBits += mask
        elif disp[i] < 0 and (curDirBits & mask):
            curDirBits -= mask

    return curDirBits

####################################################################################################

def commonInit(args, parser):

    planner = parser.planner
    printer = planner.printer

    printer.commandInit(args, PrinterProfile.getSettings())

    ddhome.home(parser, args.fakeendstop)
    # downloadTempTable(planner)
    # downloadDummyTempTable(printer)
    print "xxx newdownloadTempTable"
    newdownloadTempTable(planner)
    printer.sendPrinterInit()

####################################################################################################

def getVirtualPos(parser):

    printer = parser.planner.printer

    # Get currend stepped pos
    res = printer.getPos()
    print "Printer home pos [steps]:", res

    curPosMM = MyPoint(
        X = res[0] / float(parser.steps_per_mm[0]),
        Y = res[1] / float(parser.steps_per_mm[1]),
        Z = res[2] / float(parser.steps_per_mm[2]),
        A = res[3] / float(parser.steps_per_mm[3]),
        # B = res[4] / float(parser.steps_per_mm[4]),
        )

    print "Printer is at [mm]: ", curPosMM
    parser.setPos(curPosMM)

    return curPosMM

####################################################################################################

def prime(parser):

    planner = parser.planner

    parser.execute_line("G0 F%f Y0 Z%f" % (planner.HOMING_FEEDRATE[X_AXIS]*60, ddprintconstants.PRIMING_HEIGHT))

    pos = parser.getPos()

    aFilament = MatProfile.getMatArea()

    parser.execute_line("G0 F%f A%f" % (
        ((ddprintconstants.PRIMING_MM3_PER_SEC / aFilament) * 60),
        pos[A_AXIS] + (ddprintconstants.PRIMING_MM3 / aFilament)))

    # Set E to 0
    current_position = parser.getPos()
    current_position[A_AXIS] = 0.0
    parser.setPos(current_position)

    #
    # Retract
    #
    # parser.execute_line("G10")

    # Wipe priming material if not ultigcode flavor 
    if not parser.gcodeType == GCODEULTI:
        parser.execute_line("G0 F9000 X20 Z0.1")

####################################################################################################

def zRepeatability(parser):

    import random

    printer.commandInit(args, PrinterProfile.getSettings())

    feedrate = PrinterProfile.getMaxFeedrate(Z_AXIS)

    ddhome.home(parser, args.fakeendstop)

    printer.sendPrinterInit()

    for i in range(10):

        parser.execute_line("G0 F%d X115 Y210 Z10" % (feedrate*60))
        parser.execute_line("G0 F%d Z%f" % (feedrate*60, random.randint(20, 150)))

    planner.finishMoves()
    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    printer.sendCommand(CmdEOT)

    printer.waitForState(StateIdle)

####################################################################################################

def manualMove(parser, axis, distance, feedrate=0, absolute=False):

    planner = parser.planner
    printer = planner.printer

    printer.sendPrinterInit()

    assert(printer.isHomed())

    # Get current pos from printer and set our virtual pos
    getVirtualPos(parser)

    if not feedrate:
        feedrate = PrinterProfile.getMaxFeedrate(axis)

    current_position = parser.getPos()
    if absolute:
        d = distance - current_position[axis] 
        assert(abs(d) <= 1000)
        parser.execute_line("G0 F%d %s%f" % (feedrate*60, dimNames[axis], distance))
    else:
        assert(abs(distance) <= 1000)
        parser.execute_line("G0 F%d %s%f" % (feedrate*60, dimNames[axis], current_position[axis] + distance))

    planner.finishMoves()

    printer.sendCommand(CmdEOT)

    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])

    printer.waitForState(StateIdle)

####################################################################################################

def insertFilament(args, parser, feedrate):

    planner = parser.planner
    printer = planner.printer

    def manualMoveE():

        current_position = parser.getPos()
        aofs = current_position[A_AXIS]
        print "cura: ", aofs

        kbd = GetChar("Enter (f)orward (b)ackwards (F)orward 10mm (B)ackwards 10mm (q)uit")

        ch = " "
        while ch not in "q\n":
            ch = kbd.getc()

            print "ch: ", ch
            if ch == "f":       # filament forward, 'small' step
                aofs += 1
            elif ch == "F":     # filament forward, 'big' step
                aofs += 10
            elif ch == "b":     # filament backwards, 'small' step
                aofs -= 1
            elif ch == "B":     # filament backwards, 'big' step
                aofs -= 10

            printer.sendPrinterInit()

            # XXX hardcoded feedrate
            parser.execute_line("G0 F%d A%f" % (5*60, aofs))

            planner.finishMoves()
            printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
            printer.sendCommand(CmdEOT)
            printer.waitForState(StateIdle, wait=0.1)

    commonInit(args, parser)

    # Move to mid-position
    maxFeedrate = PrinterProfile.getMaxFeedrate(X_AXIS)
    parser.execute_line("G0 F%d X%f Y%f" % (maxFeedrate*60, planner.MAX_POS[X_AXIS]/2, planner.MAX_POS[Y_AXIS]/2))

    planner.finishMoves()

    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    printer.sendCommand(CmdEOT)

    printer.waitForState(StateIdle)

    t1 = MatProfile.getHotendStartTemp()
    printer.heatUp(HeaterEx1, t1, wait=t1 - 5, log=True)

    print "\nInsert filament.\n"
    manualMoveE()

    print "\nForwarding filament.\n"
    manualMove(parser, A_AXIS, FILAMENT_REVERSAL_LENGTH * 0.85, feedrate)

    print "\nExtrude filament.\n"
    manualMoveE()

    #
    # Retract
    #
    printer.sendPrinterInit()
    parser.execute_line("G10")
    planner.finishMoves()

    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    printer.sendCommand(CmdEOT)

    printer.waitForState(StateIdle)

    if not args.noCoolDown:
        printer.coolDown(HeaterEx1, wait=150)

####################################################################################################

def removeFilament(args, parser, feedrate):

    planner = parser.planner
    printer = planner.printer

    printer.commandInit(args, PrinterProfile.getSettings())

    ddhome.home(parser, args.fakeendstop)

    printer.sendPrinterInit()

    # Move to mid-position
    maxFeedrate = PrinterProfile.getMaxFeedrate(X_AXIS)
    parser.execute_line("G0 F%d X%f Y%f" % (maxFeedrate*60, planner.MAX_POS[X_AXIS]/2, planner.MAX_POS[Y_AXIS]/2))

    planner.finishMoves()

    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    printer.sendCommand(CmdEOT)

    printer.waitForState(StateIdle)

    t1 = MatProfile.getHotendStartTemp()
    printer.heatUp(HeaterEx1, t1, wait=t1, log=True)

    # Etwas warten und filament vorwärts feeden um den retract-pfropfen einzuschmelzen
    manualMove(parser, A_AXIS, PrinterProfile.getRetractLength() + 50, 5)

    manualMove(parser, A_AXIS, -1.3*FILAMENT_REVERSAL_LENGTH, feedrate)

    if not args.noCoolDown:
        printer.coolDown(HeaterEx1,wait=150)

####################################################################################################

def retract(args, parser, doCooldown = True):

    planner = parser.planner
    printer = planner.printer

    commonInit(args, parser)

    t1 = MatProfile.getHotendStartTemp()
    printer.heatUp(HeaterEx1, t1, wait=t1 - 5, log=True)

    parser.execute_line("G10")

    planner.finishMoves()

    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    printer.sendCommand(CmdEOT)

    printer.waitForState(StateIdle)

    if doCooldown:
        printer.coolDown(HeaterEx1,wait=150)

####################################################################################################

def bedLeveling(args, parser):

    planner = parser.planner
    printer = planner.printer

    printer.commandInit(args, PrinterProfile.getSettings())

    # Reset bedlevel offset in printer profile
    PrinterProfile.get().override("add_homeing_z", 0)

    ddhome.home(parser, args.fakeendstop, True)

    zFeedrate = PrinterProfile.getMaxFeedrate(Z_AXIS)
    kbd = GetChar("Enter (u)p (d)own (U)p 1mm (D)own 1mm (2-5) Up Xmm (q)uit")

    def manualMoveZ():

        current_position = parser.getPos()
        zofs = current_position[Z_AXIS]
        print "curz: ", zofs

        ch = " "
        while ch not in "q\n":
            ch = kbd.getc()

            print "ch: ", ch
            if ch == "u":       # head down, 'small' step
                zofs -= 0.05
            elif ch == "U":     # head down, 'big' step
                zofs -= 1
            elif ch == "d":     # head up, 'small' step
                zofs += 0.05
            elif ch == "D":     # head up, 'big' step
                zofs += 1
            else:
                try:
                    o = int(ch) # up x mmm
                except ValueError:
                    continue
                if o < 2 or o > 5:
                    continue
                zofs -= o

            printer.sendPrinterInit()

            parser.execute_line("G0 F%d Z%f" % (zFeedrate*60, zofs))

            planner.finishMoves()
            printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
            printer.sendCommand(CmdEOT)

            printer.waitForState(StateIdle, wait=0.1)


    feedrate = PrinterProfile.getMaxFeedrate(X_AXIS)

    #######################################################################################################
    print "Level point 1/3"

    printer.sendPrinterInit()
    parser.execute_line("G0 F%d X%f Y%f Z%f" % (feedrate*60, planner.X_MAX_POS/2, planner.Y_MAX_POS - 10, planner.HEAD_HEIGHT))

    planner.finishMoves()
    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    printer.sendCommand(CmdEOT)

    printer.waitForState(StateIdle, wait=0.1)

    manualMoveZ()

    current_position = parser.getPos()
    print "curz: ", current_position[Z_AXIS]

    add_homeing_z = (current_position[Z_AXIS] * -1) + planner.LEVELING_OFFSET

    # Store into printer profile:
    PrinterProfile.get().override("add_homeing_z", add_homeing_z)

    # Finally we know the zero z position
    current_position[Z_AXIS] = planner.LEVELING_OFFSET;

    # Adjust the virtual position
    parser.setPos(current_position)

    # Adjust the printer position
    posStepped = vectorMul(current_position, parser.steps_per_mm)
    payload = struct.pack("<iiiii", *posStepped)
    printer.sendCommand(CmdSetHomePos, binPayload=payload)

    #######################################################################################################
    print "Level point 2/3", current_position

    printer.sendPrinterInit()
    parser.execute_line("G0 F%d Z5" % (zFeedrate*60))
    parser.execute_line("G0 F%d X35 Y20" % (feedrate*60))
    parser.execute_line("G0 F%d Z%f" % (zFeedrate*60, planner.LEVELING_OFFSET))

    planner.finishMoves()
    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    printer.sendCommand(CmdEOT)

    printer.waitForState(StateIdle, wait=0.1)

    raw_input("\nAdjust left front buildplate screw and press <Return>\n")

    #######################################################################################################
    print "Level point 3/3", current_position

    printer.sendPrinterInit()
    parser.execute_line("G0 F%d Z5" % (zFeedrate*60))
    parser.execute_line("G0 F%d X%f" % (feedrate*60, planner.X_MAX_POS-10))
    parser.execute_line("G0 F%d Z%f" % (zFeedrate*60, planner.LEVELING_OFFSET))

    planner.finishMoves()
    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    printer.sendCommand(CmdEOT)

    printer.waitForState(StateIdle, wait=0.1)

    raw_input("\nAdjust right fron buildplate screw and press <Return>\n")

    ddhome.home(parser, args.fakeendstop)

    raw_input("\n! Please update your Z-Offset (add_homeing_z) in printer profile: %.3f\n" % add_homeing_z)

####################################################################################################

def stringFromArgs(*args):
    r = ""
    for a in args:
        if type(a) == types.StringType:
            r += a
        else:
            r += str(a)
    return r

####################################################################################################

def endOfPrintLift(parser):

    planner = parser.planner

    pos = parser.getPos()
    (homePosMM, homePosStepped) = planner.getHomePos()

    zlift = min(pos.Z + 25, homePosMM.Z)

    if zlift > pos.Z:
        parser.execute_line("G0 F%f Z%f" % (planner.HOMING_FEEDRATE[Z_AXIS]*60, zlift))


####################################################################################################

def stopMove(args, parser):

    planner = parser.planner
    printer = planner.printer

    if printer.isHomed():
        parser.reset()
        planner.reset()
        ddhome.home(parser, args.fakeendstop)

    printer.sendCommand(CmdStopMove)
    printer.sendCommand(CmdDisableSteppers)

####################################################################################################

def heatHotend(args, parser):

    planner = parser.planner
    printer = planner.printer

    printer.commandInit(args, PrinterProfile.getSettings())

    t1 = MatProfile.getHotendStartTemp()

    printer.heatUp(HeaterEx1, t1, wait=t1-5, log=True)

    raw_input("Press return to stop heating...")

    if not args.noCoolDown:
        printer.coolDown(HeaterEx1, wait=150)

####################################################################################################

#
# Execute a single gcode command on the printer and wait for completion
#
def execSingleGcode(parser, gcode):

    planner = parser.planner
    printer = planner.printer

    printer.sendPrinterInit()

    parser.execute_line(gcode)
    planner.finishMoves()

    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    printer.sendCommand(CmdEOT)

    printer.waitForState(StateIdle)

####################################################################################################

# Heat up nozzle
# Retract
# Wait for user to change nozzle
# Un-Retract
# Cooldown
def changeNozzle(args, parser):

    planner = parser.planner
    printer = planner.printer

    retract(args, parser, doCooldown = False)

    feedrate = PrinterProfile.getMaxFeedrate(X_AXIS)
    execSingleGcode(parser, "G0 F%d X%f Y%f" % (feedrate*60, planner.MAX_POS[X_AXIS]/2, planner.MAX_POS[Y_AXIS]*0.1))

    raw_input("Now change nozzle, Press return to stop heating...")

    execSingleGcode(parser, "G11")

    if not args.noCoolDown:
        printer.coolDown(HeaterEx1, wait=100)

####################################################################################################
#
# Measure closed loop step-response of the hotend and plot it with gnuplot.
# Do three setpoint changes: the first is from current temperature to args.t1 or 200 °C, then
# after reaching this first set temperature a step upwards to 120%, then a downward step to 90% of
# the first temperature is done.
#
def stepResponse(args, parser):

    planner = parser.planner
    printer = planner.printer

    def stopHeater():
        payload = struct.pack("<BH", HeaterEx1, 0) # Parameters: heater, temp
        printer.sendCommand(CmdSetTargetTemp, binPayload=payload)
        printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(0)])

    # Destination temperature
    tDest1 = args.t1 or 200
    tDest2 = tDest1 * 1.2
    tDest3 = tDest1 * 0.9
    assert(tDest1 <= 275)

    # Bail out if tmax reached
    tmax = tDest1 * 1.5

    printer.commandInit(args, PrinterProfile.getSettings())
    es = printer.getEepromSettings()

    # Open output gnuplot file
    f = open("stepresponse_closed.gnuplot", "w")

    f.write("# Kp: %f\n" % es["Kp"])
    f.write("# Ki: %f\n" % es["Ki"])
    f.write("# Kd: %f\n" % es["Kd"])
    f.write("set yrange [0:%d]\n" % tmax)
    f.write("set grid\n")
    f.write("plot \"-\" using 1:2 with linespoints title \"StepResponse\",")
    f.write("%d title \"tDest1\", %d title \"tDest2\", %d title \"tDest3\"\n" % (tDest1, tDest2, tDest3))

    # Do the first step
    print "Starting input step to %d °" % tDest1
    payload = struct.pack("<BH", HeaterEx1, tDest1) # Parameters: heater, temp
    printer.sendCommand(CmdSetTargetTemp, binPayload=payload)
    printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(100)])

    timeStart = time.time()
    status = printer.getStatus()
    tempStart = status["t1"]

    print "Temp:", tempStart
    f.write("0 %f\n" % tempStart)

    lastTime = timeStart 
    lastTemp = tempStart

    # Build 60 second average of the temperature change derivation
    nAvg = 60
    aAvg = nAvg * 1.0

    # Stop if temp curve gets flat enough
    wait = 60
    flatReached = False
    while wait:
        
        if lastTemp > tmax:
            print "Error, max temp (%d) reached: " % tmax, lastTemp
            stopHeater()
            return

        time.sleep(1)

        tim = time.time()
        status = printer.getStatus()
        temp = status["t1"]

        relTime = tim-timeStart

        f.write("%f %f\n" % (relTime, temp))

        dy = temp - lastTemp
        dx = tim - lastTime

        a = dy / dx
        print "Temp:", temp, "Wait:", wait

        aAvg = aAvg - (aAvg/nAvg) + a
        print "Steigung: %7.4f %7.4f" % (a, aAvg/nAvg)

        if abs((aAvg/nAvg)) < 0.2 and not flatReached:
            flatReached = True

        if flatReached:
            wait -= 1

        lastTime = tim
        lastTemp = temp
        
    # Do the second step
    print "Starting input step to %d °" % tDest2
    payload = struct.pack("<BH", HeaterEx1, tDest2) # Parameters: heater, temp
    printer.sendCommand(CmdSetTargetTemp, binPayload=payload)

    aAvg = nAvg * 1.0

    # Stop if temp curve gets flat enough
    wait = 20
    flatReached = False
    while wait:
        
        if lastTemp > tmax:
            print "Error, max temp (%d) reached: " % tmax, lastTemp
            stopHeater()
            return

        time.sleep(1)

        tim = time.time()
        status = printer.getStatus()
        temp = status["t1"]

        relTime = tim-timeStart

        f.write("%f %f\n" % (relTime, temp))

        dy = temp - lastTemp
        dx = tim - lastTime

        a = dy / dx
        print "Temp:", temp, "Wait:", wait

        aAvg = aAvg - (aAvg/nAvg) + a
        print "Steigung: %7.4f %7.4f" % (a, aAvg/nAvg)

        if abs((aAvg/nAvg)) < 0.25 and not flatReached:
            flatReached = True

        if flatReached:
            wait -= 1

        lastTime = tim
        lastTemp = temp
        
    # Do the third step
    print "Starting input step to %d °" % tDest3
    payload = struct.pack("<BH", HeaterEx1, tDest3) # Parameters: heater, temp
    printer.sendCommand(CmdSetTargetTemp, binPayload=payload)

    aAvg = nAvg * 1.0

    # Stop if temp curve gets flat enough
    wait = 20
    flatReached = False
    while wait:
        
        if lastTemp > tmax:
            print "Error, max temp (%d) reached: " % tmax, lastTemp
            stopHeater()
            return

        time.sleep(1)

        tim = time.time()
        status = printer.getStatus()
        temp = status["t1"]

        relTime = tim-timeStart

        f.write("%f %f\n" % (relTime, temp))

        dy = temp - lastTemp
        dx = tim - lastTime

        a = dy / dx
        print "Temp:", temp, "Wait:", wait

        aAvg = aAvg - (aAvg/nAvg) + a
        print "Steigung: %7.4f %7.4f" % (a, aAvg/nAvg)

        if abs((aAvg/nAvg)) < 0.25 and not flatReached:
            flatReached = True

        if flatReached:
            wait -= 1

        lastTime = tim
        lastTemp = temp
        
    f.write("e\npause mouse close\n")
    stopHeater()

# Open loop step response to determine control parameters.
def measureHotendStepResponse(args, parser):

    planner = parser.planner
    printer = planner.printer

    def stopHeater():
        payload = struct.pack("<BB", HeaterEx1, 0) # heater, pwmvalue
        printer.sendCommand(CmdSetHeaterY, binPayload=payload)

    printer.commandInit(args, PrinterProfile.getSettings())

    tmax = 260

    ## Eingangssprung, nicht die volle leistung, da sonst die temperatur am ende
    ## der sprungantwort zu stark überschwingt und zu hoch wird.
    Xo = 100.0
    interval = 0.01

    temp = tempStart = printer.getTemp(doLog = False)[1]

    # Output file for raw data
    fraw = open("autotune.raw.json", "w")
    fraw.write("""{
    "Xo": %d,
    "dt": %f,
    "startTemp": %f,
    "columns":  "time temperature",
    """ % (Xo, interval, tempStart))

    print "Starting input step, tmax is:", tmax

    # Fan is running while printing (and filament has to be molten), so run fan 
    # at max speed to simulate this energy-loss.
    printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(255)])
    payload = struct.pack("<BB", HeaterEx1, Xo) # heater, pwmvalue
    printer.sendCommand(CmdSetHeaterY, binPayload=payload)

    timeStart = time.time()

    print "Starting temp:", tempStart
    data = []

    # Abbruch, falls 10 sekunden lang keine steigerung der temperatur mehr auftritt
    maxTemp = 0
    tMaxTemp = timeStart

    while (time.time() - tMaxTemp) < 30:

        if temp > tmax:
            print "Error, max temp (%d) reached (you should decrease Xo): " % tmax, temp
            stopHeater()
            return

        tim = time.time()
        temp = printer.getTemp(doLog = False)[1]

        relTime = tim-timeStart
        Mu = temp-tempStart

        data.append("       [%f, %f]" % (relTime, Mu))

        if Mu > maxTemp:
            maxTemp = Mu
            tMaxTemp = tim

        print "Time, Temp, tconst:", relTime, temp, tim-tMaxTemp

        lastTime = tim
        lastTemp = temp
        time.sleep(interval)

    stopHeater()

    fraw.write("""
    "tEnd": %f,
    "data": [
    """ % maxTemp)
    fraw.write("    " + ",\n".join(data))
    fraw.write("    ]\n}\n")
    fraw.close()

    print "Step response done, result in autotune.raw."

    printer.coolDown(HeaterEx1, wait=100)
    printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(0)])

####################################################################################################
#
# Allow comments in json files.
#
def jsonLoad(f):

    s = ""
    for l in f.readlines():
        l = l.strip()
        if l.startswith("#"):
            # print "skip comment:", l
            continue
        s += l

    return json.loads(s)

####################################################################################################

def handleGenericResponse(resp):
    (cmd, payload) = resp

    code = ord(payload[0])
    if code != RespOK:
        print "Command '%s' returned code '%s'." % (CommandNames[cmd], RespCodeNames[code])
        return False

    return True

####################################################################################################

def getResponseString(s, offset):

    length = ord(s[offset])
    return s[offset+1:offset+1+length]

####################################################################################################

def old_genTempTable(planner):

    hwVersion = PrinterProfile.getHwVersion()
    nozzleDiam = NozzleProfile.getSize()
    baseTemp = MatProfile.getHotendBaseTemp(hwVersion, nozzleDiam)
    baseFlowrate = MatProfile.getFlowrateForTemp(baseTemp, hwVersion, nozzleDiam) * (1.0-AutotempSafetyMargin)

    # Temps lower than 170 not handled yet
    assert(baseTemp >= 170)

    aFilament = MatProfile.getMatArea()
    spm = PrinterProfile.getStepsPerMM(A_AXIS)

    of = open("/tmp/temptable0.txt", "w")
    of.write("# xxx mat, nozzle, settings...\n")
    of.write("# basetemp: %f\n" % baseTemp)
    of.write("# temp flowrate steprate timer\n")

    print "TempTable (basetemp: %f):" % baseTemp
    table = []
    for i in range(NExtrusionLimit):

        # t = baseTemp + i*2
        # t = baseTemp + i
        t = 170 + i

        if t < baseTemp:
            # Note: simple interpolation for values between 170° and start of profile basetemp
            f = (baseFlowrate-0.1) / (baseTemp-170)
            flowrate = 0.1 + i*f
        else:
            flowrate = MatProfile.getFlowrateForTemp(t, hwVersion, nozzleDiam) * (1.0-AutotempSafetyMargin)

        espeed = flowrate / aFilament
        print "flowrate for temp %f: %f -> espeed %f" % (t, flowrate, espeed)

        # espeed = planner.advance.eComp(espeed)
        # print "corrected espeed:", espeed

        steprate = espeed * spm
        tvs = 1.0/steprate
        timerValue = min(int(fTimer / steprate), 0xffff)

        print "    Temp: %f, max flowrate: %.2f mm³/s, max espeed: %.2f mm/s, steps/s: %d, steprate: %d us, timervalue: %d" % (t, flowrate, espeed, int(steprate), int(tvs*1000000), timerValue)
        table.append(timerValue)

        of.write("%f %4.1f %d %d\n" % (t, flowrate, int(steprate), timerValue))

    of.close()

    return (170, table)

def genTempTable(planner):

    hwVersion = PrinterProfile.getHwVersion()
    spm = PrinterProfile.getStepsPerMM(A_AXIS)
    nozzleDiam = NozzleProfile.getSize()
    aFilament = MatProfile.getMatArea()

    baseTemp = MatProfile.getHotendBaseTemp(hwVersion, nozzleDiam)
    baseFlowrate = MatProfile.getFlowrateForTemp(baseTemp, hwVersion, nozzleDiam) * (1.0-AutotempSafetyMargin)

    # Temps lower than 170 not handled yet
    assert(baseTemp >= 170)

    table = []
    for i in range(NExtrusionLimit):

        # t = baseTemp + i*2
        # t = baseTemp + i
        t = 170 + i

        if t < baseTemp:
            # Note: simple interpolation for values between 170° and start of profile basetemp
            f = (baseFlowrate-0.1) / (baseTemp-170)
            flowrate = 0.1 + i*f
        else:
            flowrate = MatProfile.getFlowrateForTemp(t, hwVersion, nozzleDiam) * (1.0-AutotempSafetyMargin)

        espeed = flowrate / aFilament
        print "flowrate for temp %f: %f -> espeed %f" % (t, flowrate, espeed)

        steprate = espeed * spm
        tvs = 1.0/steprate
        timerValue = min(int(fTimer / steprate), 0xffff)

        print "    Temp: %f, max flowrate: %.2f mm³/s, max espeed: %.2f mm/s, steps/s: %d, steprate: %d us, timervalue: %d" % (t, flowrate, espeed, int(steprate), int(tvs*1000000), timerValue)
        table.append(timerValue)

    return (170, table)


    of = open("/tmp/temptable0.txt", "w")
    of.write("# xxx mat, nozzle, settings...\n")
    of.write("# basetemp: %f\n" % baseTemp)
    of.write("# temp flowrate steprate timer\n")

    print "TempTable (basetemp: %f):" % baseTemp
    table = []
    for i in range(NExtrusionLimit):

        # t = baseTemp + i*2
        # t = baseTemp + i
        t = 170 + i

        if t < baseTemp:
            # Note: simple interpolation for values between 170° and start of profile basetemp
            f = (baseFlowrate-0.1) / (baseTemp-170)
            flowrate = 0.1 + i*f
        else:
            flowrate = MatProfile.getFlowrateForTemp(t, hwVersion, nozzleDiam) * (1.0-AutotempSafetyMargin)

        espeed = flowrate / aFilament
        print "flowrate for temp %f: %f -> espeed %f" % (t, flowrate, espeed)

        # espeed = planner.advance.eComp(espeed)
        # print "corrected espeed:", espeed

        steprate = espeed * spm
        tvs = 1.0/steprate
        timerValue = min(int(fTimer / steprate), 0xffff)

        print "    Temp: %f, max flowrate: %.2f mm³/s, max espeed: %.2f mm/s, steps/s: %d, steprate: %d us, timervalue: %d" % (t, flowrate, espeed, int(steprate), int(tvs*1000000), timerValue)
        table.append(timerValue)

        of.write("%f %4.1f %d %d\n" % (t, flowrate, int(steprate), timerValue))

    of.close()

    return (170, table)

####################################################################################################

def eTimerValue(planner, eSpeed):

    spm = PrinterProfile.getStepsPerMM(A_AXIS)
    steprate = eSpeed * spm
    timerValue = int(fTimer / steprate)
    return timerValue


####################################################################################################

def printTempTable(temp, tempTable):

    of = open("/tmp/temptable_printer.txt", "w")
    of.write("# XXX output mat, nozzle, settings...\n")
    of.write("# Basetemp: %d\n" % temp)
    of.write("# Columns: temp rate steprate timer\n")

    print "TempTable (basetemp: %d):" % temp

    mmpermm3 = 1 / MatProfile.getMatArea()
    spm = PrinterProfile.getStepsPerMM(A_AXIS)

    for timerValue in tempTable:

        steprate = fTimer / timerValue

        speed = (steprate / spm) / mmpermm3

        print "    Temp: %d, max extrusion: %.1f mm³/s, steps/s: %d, timervalue: %d" % (temp, speed, int(steprate), timerValue)

        of.write("%d %4.1f %d %d\n" % (temp, speed, int(steprate), timerValue))

        # temp += 2
        temp += 1

    of.close()

####################################################################################################

def downloadTempTable(planner):

    assert(0)

    (baseTemp, table) = genTempTable(planner)

    payload = struct.pack("<HB", baseTemp, NExtrusionLimit)

    print "Downloading TempTable..."

    for timerValue in table:
        payload += struct.pack("<H", timerValue)
    resp = planner.printer.query(CmdSetTempTable, binPayload=payload)
    assert(handleGenericResponse(resp))

def newdownloadTempTable(planner):

    (baseTemp, table) = genTempTable(planner)

    payload = struct.pack("<HB", baseTemp, NExtrusionLimit)

    print "Downloading TempTable..."

    for timerValue in table:
        payload += struct.pack("<H", timerValue)
    resp = planner.printer.query(CmdSetTempTable, binPayload=payload)
    assert(handleGenericResponse(resp))

####################################################################################################

def downloadDummyTempTable(printer):

    print "Downloading Dummy TempTable..."
    payload = struct.pack("<HB", 100, NExtrusionLimit)

    for i in range(NExtrusionLimit):
        payload += struct.pack("<H", 50)

    resp = printer.query(CmdSetTempTable, binPayload=payload)
    assert(handleGenericResponse(resp))

####################################################################################################

def getStartupTime(feedrate):

    eAccel = PrinterProfile.getMaxAxisAcceleration()[A_AXIS]

    # Zeit bis sich der messwert der target geschwindigkeit
    # stabilisiert hat.
    # 1. timer accel ramp
    tAccel = feedrate / eAccel

    return tAccel

def old_getStartupTime(feedrate):

    eAccel = PrinterProfile.getMaxAxisAcceleration()[A_AXIS]
    e_steps_per_mm = PrinterProfile.getStepsPerMM(A_AXIS)

    # Zeit bis sich der messwert der target geschwindigkeit
    # stabilisiert hat.
    # 1. timer accel ramp
    tAccel = feedrate / eAccel
    # 2. time for 50 steps (50 steps: siehe FilamentSensorADNS9800::run())
    stepsPerSecond = feedrate * e_steps_per_mm
    t50 = 50 / stepsPerSecond
    tTargetStartup = tAccel + 2*t50

    # Zeit bis sich der messwert der filsensor geschwindigkeit
    # stabilisiert hat.
    # 1. timer accel ramp
    # 2. time for 50 steps (50 steps: siehe FilamentSensorADNS9800::run())
    stepsPerSecond = feedrate * 250
    t50 = 50 / stepsPerSecond
    tActualStartup = tAccel + 2*t50
    print "tTargetStartup:", tTargetStartup, ", tActualStartup", tActualStartup

    return max(tTargetStartup, tActualStartup)

####################################################################################################
# 
# Ks is: 2.09997869
# Tp: 109.869478607
# Td: 7.72118860714
#
#
# * Lege start-pwm wert von z.b. 100 an
# * messe temperatur und starte messvorgang falls t >= 190
# * messvorgang: mit kurzer avg funktion messen, das ist zwar etwas ungenauer, aber wir können
#   ja richtig gegen den anschlag fahren und z.b. 75% grip als grenze vorgeben.
#
#

def measureFlowrateStepResponse(args, parser):

    def writeDataSet(f, dataSet):

        temps = dataSet.keys()
        temps.sort()

        for temp in temps:
            tup = dataSet[temp]
            f.write("%d %f %f %f %f\n" % (temp, tup[0], tup[1], tup[2], tup[3]))

        f.write("E\n")

    def writeGnuplot(t1, dataSet):

        gnuplotFile = open("temp-flowrate-curve.gnuplot", "w")
        gnuplotFile.write("""

set grid
set yrange [0:35]

# BaseTemp=%d

# Startwert steigung
a=0.5

# Startwert y-achse
b=5
f(x)=b+a*(x-%d)

fit f(x) "-" using 1:3 noerror via a,b\n""" % (t1, t1))

        writeDataSet(gnuplotFile, dataSet)

        gnuplotFile.write("""
plot "-" using 1:2 with linespoints title "Target Flowrate", \\
     "-" using 1:3 with linespoints title "Actual Flowrate", \\
     "-" using 1:3 with linespoints smooth bezier title "Actual Flowrate smooth", \\
     f(x) title sprintf("y=B+A*x, A=%.2f, B=%.1f, TempFactor 1/A: %.2f", a, b, 1/a)\n""")

        writeDataSet(gnuplotFile, dataSet)
        writeDataSet(gnuplotFile, dataSet)
        writeDataSet(gnuplotFile, dataSet)

        gnuplotFile.write("pause mouse close")
        gnuplotFile.close()

    nozzleSize = NozzleProfile.getSize()

    def writeJson(dataSet, slippage):

        jsonFile = open("temp-flowrate-curve.json", "w")
        jsonFile.write("""
    "tempFlowrateCurve_%d": {
        "version": %d,
        "slippage": %f,
        "data": [\n""" % (nozzleSize*100, PrinterProfile.getHwVersion(), slippage))
    
        temps = dataSet.keys()
        temps.sort()

        strings = []
        for temp in temps:
            tup = dataSet[temp]
            strings.append("            [%d, %f, %f]" % (temp, tup[1], tup[3]))

        jsonFile.write(",\n".join(strings))
        jsonFile.write("\n        ]\n    }\n")
        jsonFile.close()

    def tempGood(actT1, t1):
        return actT1 >= (t1-2) and actT1 <= (t1+2)

    planner = parser.planner
    printer = planner.printer

    aFilament = MatProfile.getMatArea()

    printerSettings = PrinterProfile.getSettings()

    # Override integral value for temperature PID to avoid big temperature-overshots
    printerProfile = PrinterProfile.get()
    printerProfile.override("Ki", printerSettings["Ki"] * 0.75)

    printer.commandInit(args, PrinterProfile.getSettings())

    print "overwritten Ki: ", printerSettings["Ki"], PrinterProfile.getSettings()["Ki"]

    # ddhome.home(parser, args.fakeendstop)

    # Disable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])

    # Disable temp-flowrate limit
    downloadDummyTempTable(printer)

    # Move to mid-position
    # printer.sendPrinterInit()
    # feedrate = PrinterProfile.getMaxFeedrate(X_AXIS)
    # parser.execute_line("G0 F%d X%f Y%f" % (feedrate*60, planner.MAX_POS[X_AXIS]/2, planner.MAX_POS[Y_AXIS]/2))

    # planner.finishMoves()
    # printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    # printer.sendCommand(CmdEOT)
    # printer.waitForState(StateIdle)

    dataSet = {}

    printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(100)])

    # start with 1mm³/s
    # XXX speed too high for small nozzles!?
    feedrate = (args.flowrate or 1.0) / aFilament

    tWait = 0.1

    eMotorRunning = False
    primed = False

    # Running average of hotend temperature
    # tempAvg = EWMA(0.5)
    tempAvg = movingavg.MovingAvg(10)
    flowAvg = movingavg.MovingAvgReadings(10)
    # Running average of *grip*
    # ratioAvg = EWMA(0.5)
    # ratioAvg = MovingAvg(10)
    # pwmAvg = EWMA(0.5)

    minGrip = 0.90

    t1 = args.t1 or MatProfile.getHotendStartTemp()

    dt = PrinterProfile.getFilSensorInterval()

    steps_per_mm = PrinterProfile.getStepsPerMM(A_AXIS)

    pcal = PrinterProfile.get().getFilSensorCalibration()

    pwmOutput = 0
    pwmMax = 255

    # ratioAvg.setValue(minGrip + (1.0-minGrip)/10.0)
    # ratio = minGrip + (1.0-minGrip)/10.0

    ####################################################################################################

    # * set initial pwm value and wait for min temp
    # * start measurement loop
    pwm0 = 80 # 100
    startTemp = 160 # 190
    endTemp = 300
    Ks = 2.09997869
    # Tp: 109.869478607
    # Td: 7.72118860714
    T66 = 125
    sprung = min(255-pwm0, (endTemp-startTemp) / Ks)

    ####################################################################################################
    # Output file for raw data
    fraw = open("flowrateMeasurement.raw.json", "w")
    fraw.write("""{
    "PrinterName": "%s",
    "p0": %d,
    "step": %f,
    "dt": %f,
    "startTemp": %f,
    "columns":  "time targetFlowrate ratio flowrate temp",
    "data": [
    """ % (printer.getPrinterName(), pwm0, sprung, dt, startTemp))
    ####################################################################################################

    print "setting initial pwm:", pwm0
    printer.setTempPWM(HeaterEx1, pwm0)
    while True:

        status = printer.getStatus()
        actT1 = status["t1"]
        tempAvg.add(actT1)

        t1Avg = tempAvg.mean()

        print "\rCurrent temp: %.2f/%.2f" % (actT1, startTemp),
        sys.stdout.flush()

        if t1Avg >= startTemp:
            print "\nreached temp:", t1Avg
            break

        time.sleep(tWait)

    # start motor
    print "\nstarting motor with %.2f mm/s" % feedrate
    printer.sendCommandParamV(CmdContinuousE, [packedvalue.uint16_t(eTimerValue(planner, feedrate))])

    mode = "starting"

    tStart = time.time()
    tStep = 30        # debug
    tEnd = tStep + 30 # debug

    tStep = 2*T66
    tEnd = tStep + 2*T66

    while True:

        status = printer.getStatus()
        actT1 = status["t1"]
        tempAvg.add(actT1)

        fsreadings = printer.getFSReadings()
        flowAvg.addReadings(fsreadings)

        t1Avg = tempAvg.mean()

        meanShort = flowAvg.mean()
        targetFlowRate = feedrate * aFilament

        # should be speed:
        stepsPerInterval = feedrate * steps_per_mm * dt

        r = meanShort / (stepsPerInterval * pcal)

        print "t: %.2f, TempAvg: %.1f, target flowrate: %.3f mm³/s, actual flowrate: %.2f mm³/s, current ratio: %.2f" % (time.time()-tStart, t1Avg, targetFlowRate, targetFlowRate*r, r)

        # addcomma = False
        if r > minGrip:
            # increase speed

            frincrease = feedrate * max(0.05 * (max(r, minGrip) - minGrip), 0.0001) # increase feedrate by max 5% and min 0.1% mm per second
            feedrate += frincrease
            print "\nincreased feedrate by %.3f to %.3f" % (frincrease, feedrate)
            
            # set new feedrate:
            printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(eTimerValue(planner, feedrate))])

        fraw.write("    [%f, %f, %f, %f, %f]" % (time.time()-tStart, targetFlowRate, r, targetFlowRate*r, t1Avg))
        # addcomma = True

        if time.time() >= tStart+tStep and mode == "starting":
            print "T66 reached, do step %d --> %d" % (pwm0, pwm0+sprung)
            printer.setTempPWM(HeaterEx1, pwm0+sprung)
            mode = "measure"

        if time.time() >= tStart+tEnd:
            print "2*T66 reached, break"
            break

        # if addcomma:
        fraw.write(",\n")

        time.sleep(tWait)

    ####################################################################################################


    # Done
    fraw.write("""  ],\n""")
    fraw.write("""  "tStep": %f\n""" % tStep)
    fraw.write("}\n")
    fraw.close()

    printer.setTempPWM(HeaterEx1, 0) # re-enable temperature PID
    printer.coolDown(HeaterEx1)

    # Slow down E move
    while feedrate > 1:
        feedrate -= 0.1
        printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(eTimerValue(planner, feedrate))])
        time.sleep(0.5)

    # Stop continuos e-mode
    printer.sendCommandParamV(CmdContinuousE, [packedvalue.uint16_t(0)])

    # Re-enable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(1)])


####################################################################################################
# 
# Ks is: 2.09997869
# Tp: 109.869478607
# Td: 7.72118860714
#
#
# * Lege start-pwm wert von z.b. 100 an
# * messe temperatur und starte messvorgang falls t >= 190
# * messvorgang: mit kurzer avg funktion messen, das ist zwar etwas ungenauer, aber wir können
#   ja richtig gegen den anschlag fahren und z.b. 75% grip als grenze vorgeben.
#
#

def measureTempFlowrateCurve(args, parser):

    def writeDataSet(f, dataSet):

        temps = dataSet.keys()
        temps.sort()

        for temp in temps:
            tup = dataSet[temp]
            f.write("%d %f %f %f %f\n" % (temp, tup[0], tup[1], tup[2], tup[3]))

        f.write("E\n")

    def writeGnuplot(t1, dataSet):

        gnuplotFile = open("temp-flowrate-curve.gnuplot", "w")
        gnuplotFile.write("""

set grid
set yrange [0:35]

# BaseTemp=%d

# Startwert steigung
a=0.5

# Startwert y-achse
b=5
f(x)=b+a*(x-%d)

fit f(x) "-" using 1:3 noerror via a,b\n""" % (t1, t1))

        writeDataSet(gnuplotFile, dataSet)

        gnuplotFile.write("""
plot "-" using 1:2 with linespoints title "Target Flowrate", \\
     "-" using 1:3 with linespoints title "Actual Flowrate", \\
     "-" using 1:3 with linespoints smooth bezier title "Actual Flowrate smooth", \\
     f(x) title sprintf("y=B+A*x, A=%.2f, B=%.1f, TempFactor 1/A: %.2f", a, b, 1/a)\n""")

        writeDataSet(gnuplotFile, dataSet)
        writeDataSet(gnuplotFile, dataSet)
        writeDataSet(gnuplotFile, dataSet)

        gnuplotFile.write("pause mouse close")
        gnuplotFile.close()

    nozzleSize = NozzleProfile.getSize()

    def writeJson(dataSet, slippage):

        jsonFile = open("temp-flowrate-curve.json", "w")
        jsonFile.write("""
    "tempFlowrateCurve_%d": {
        "version": %d,
        "slippage": %f,
        "data": [\n""" % (nozzleSize*100, PrinterProfile.getHwVersion(), slippage))
    
        temps = dataSet.keys()
        temps.sort()

        strings = []
        for temp in temps:
            tup = dataSet[temp]
            strings.append("            [%d, %f, %f]" % (temp, tup[1], tup[3]))

        jsonFile.write(",\n".join(strings))
        jsonFile.write("\n        ]\n    }\n")
        jsonFile.close()

    def tempGood(actT1, t1):
        return actT1 >= (t1-2) and actT1 <= (t1+2)

    planner = parser.planner
    printer = planner.printer

    aFilament = MatProfile.getMatArea()

    printerSettings = PrinterProfile.getSettings()

    # Override integral value for temperature PID to avoid big temperature-overshots
    printerProfile = PrinterProfile.get()
    printerProfile.override("Ki", printerSettings["Ki"] * 0.75)

    printer.commandInit(args, PrinterProfile.getSettings())

    print "overwritten Ki: ", printerSettings["Ki"], PrinterProfile.getSettings()["Ki"]

    # ddhome.home(parser, args.fakeendstop)

    # Disable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])

    # Disable temp-flowrate limit
    downloadDummyTempTable(printer)

    # Move to mid-position
    # printer.sendPrinterInit()
    # feedrate = PrinterProfile.getMaxFeedrate(X_AXIS)
    # parser.execute_line("G0 F%d X%f Y%f" % (feedrate*60, planner.MAX_POS[X_AXIS]/2, planner.MAX_POS[Y_AXIS]/2))

    # planner.finishMoves()
    # printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    # printer.sendCommand(CmdEOT)
    # printer.waitForState(StateIdle)

    dataSet = {}

    printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(100)])

    # start with 1mm³/s
    # XXX speed too high for small nozzles!?
    feedrate = (args.flowrate or 1.0) / aFilament

    tWait = 0.1

    t1 = args.t1 or MatProfile.getHotendStartTemp()
    dt = PrinterProfile.getFilSensorInterval()
    steps_per_mm = PrinterProfile.getStepsPerMM(A_AXIS)
    pcal = PrinterProfile.get().getFilSensorCalibration()

    ####################################################################################################

    # * set initial pwm value and wait for min temp
    # * start measurement loop
    # pwm0 = 90 # 100
    startTemp = 170     # xxx use profile.mintemp
    endTemp = 260       # xxx use profile.maxtemp
    # Ks = 2.09997869
    # Tp: 109.869478607
    # Td: 7.72118860714
    T66 = 125

    # Averaging window, 1/4 of timeconstant
    nAvg = int(round(T66 / (dt * 4))) # debug
    nAvg = int(round(T66 / (dt * 10))) # debug
    print "navg:", nAvg

    # sprung = min(255-pwm0, (endTemp-startTemp) / Ks)
    ####################################################################################################

    # ######################eMotorRunning = False
    # ######################primed = False

    # Running average of hotend temperature
    # tempAvg = EWMA(0.5)
    tempAvg = movingavg.MovingAvg(10)
    flowAvg = movingavg.MovingAvgReadings(10)
    # Running average of *grip*
    # ratioAvg = EWMA(0.5)
    # ratioAvg = MovingAvg(10)
    # pwmAvg = EWMA(0.5)

    pwmAvg = movingavg.MovingAvg(nAvg)

    minGrip = 0.90
    pwmOutput = 0
    pwmMax = 255

    # ratioAvg.setValue(minGrip + (1.0-minGrip)/10.0)
    # ratio = minGrip + (1.0-minGrip)/10.0

    ####################################################################################################
    # Output file for raw data
    fraw = open("flowrateMeasurement.raw.json", "w")
    fraw.write("""{
    "PrinterName": "%s",
    "p0": %d,
    "step": %f,
    "dt": %f,
    "startTemp": %f,
    "columns":  "time targetFlowrate ratio flowrate temp",
    "data": [
    """ % (printer.getPrinterName(), 0, 0, dt, startTemp))
    ####################################################################################################

    print "setting initial temp:", t1
    printer.heatUp(HeaterEx1, t1, wait=t1-1.5, log=True)

    # start motor
    print "\nstarting motor with %.2f mm/s" % feedrate
    printer.sendCommandParamV(CmdContinuousE, [packedvalue.uint16_t(eTimerValue(planner, feedrate))])

    tStart = time.time()
    # tStep = 30        # debug
    # tEnd = tStep + 30 # debug

    # tStep = 2*T66
    # tEnd = tStep + 2*T66

    while True:

        status = printer.getStatus()
        actT1 = status["t1"]
        tempAvg.add(actT1)

        fsreadings = printer.getFSReadings()
        flowAvg.addReadings(fsreadings)

        pwmAvg.add(status["pwmOutput"])

        t1Avg = tempAvg.mean()

        meanShort = flowAvg.mean()
        targetFlowRate = feedrate * aFilament

        # should be speed:
        stepsPerInterval = feedrate * steps_per_mm * dt

        # pcal: 0.7463
        r = meanShort / (stepsPerInterval * pcal)

        print "t: %.2f, TempAvg: %.1f, target flowrate: %.3f mm³/s, actual flowrate: %.2f mm³/s, current ratio: %.2f" % \
                (time.time()-tStart, t1Avg, targetFlowRate, targetFlowRate*r, r)

        # addcomma = False
        if r > minGrip:
            # increase speed

            frincrease = feedrate * max(0.05 * (max(r, minGrip) - minGrip), 0.0001) # increase feedrate by max 5% and min 0.1% mm per second
            feedrate += frincrease
            print "\nincreased feedrate by %.3f to %.3f" % (frincrease, feedrate)
            
            # set new feedrate:
            printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(eTimerValue(planner, feedrate))])

        fraw.write("    [%f, %f, %f, %f, %f]" % (time.time()-tStart, targetFlowRate, r, targetFlowRate*r, t1Avg))
        # addcomma = True

        if pwmAvg.valid():
            print "pwm avg valid:", pwmAvg.mean()
            if pwmAvg.near(0.05):
                print "pwm in 5% tolerance band, break"
                break
        else:
            print "pwm avg not valid, yet:", pwmAvg.mean()

        # if addcomma:
        fraw.write(",\n")

        time.sleep(tWait)

    ####################################################################################################


    # Done
    fraw.write("""  ],\n""")
    fraw.write("""  "tStep": %f\n""" % 0.0)
    fraw.write("}\n")
    fraw.close()

    # printer.setTempPWM(HeaterEx1, 0) # re-enable temperature PID
    printer.coolDown(HeaterEx1)

    # Slow down E move
    while feedrate > 1:
        feedrate -= 0.1
        printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(eTimerValue(planner, feedrate))])
        time.sleep(0.5)

    # Stop continuos e-mode
    printer.sendCommandParamV(CmdContinuousE, [packedvalue.uint16_t(0)])

    # Re-enable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(1)])


####################################################################################################
#
# Create a list of stepper pulses for a acceleration ramp.
#
def accelRamp(axis, vstart, vend, a, nSteps):

    assert(vstart <= vend)

    pulses = [] # (tstep, dt, timerValue)

    steps_per_mm = PrinterProfile.getStepsPerMM(axis)
    sPerStep = 1.0/steps_per_mm

    v = vstart
    tstep = 0
    s = sPerStep

    stepToDo = nSteps

    while v < vend and stepToDo > 0:

        # Speed after this step
        vn1 = vAccelPerDist(vstart, a, s)

        # Time we need for this speed change/this step:
        dv = vn1 - v
        dt = dv / a

        # Timervalue for this time
        timerValue = int(dt * fTimer)

        timerValue = min(timerValue, ddprintconstants.maxTimerValue16)

        # print "v after this step:", vn1, s, dt, timerValue

        pulses.append((tstep, dt, timerValue))

        s += sPerStep
        v = vn1
        tstep += dt
        stepToDo -= 1

    return pulses

####################################################################################################
#
# Create a list of stepper pulses for a deceleration ramp.
#
def decelRamp(axis, vstart, vend, a, nSteps):

    assert(vstart >= vend)
    # assert(nSteps)

    pulses = [] # (tstep, dt, timerValue)

    steps_per_mm = PrinterProfile.getStepsPerMM(axis)
    sPerStep = 1.0/steps_per_mm

    v = vstart
    tstep = 0
    s = sPerStep

    while v > vend and nSteps > 0:

        # Speed after this step
        vn1 = vAccelPerDist(vstart, -a, s)

        # Time we need for this speed change/this step:
        dv = v - vn1
        dt = dv / a

        # Timervalue for this time
        timerValue = int(dt * fTimer)

        if timerValue > ddprintconstants.maxTimerValue16:
            # print "break on timeroverflow, v after this step:", vn1, s, dt, timerValue
            break

        # print "v after this step:", vn1, s, dt, timerValue

        pulses.append((tstep, dt, timerValue))

        s += sPerStep
        v = vn1
        tstep += dt
        nSteps -= 1

    return pulses

####################################################################################################
#
# Create a list of stepper pulses for a deceleration ramp.
#
def decelRampXY(leadAxis, vstart, vend, a, absSteps):

    assert(vstart >= vend)

    pulses = [] # (tstep, dt, timerValue)

    leadSteps = absSteps[leadAxis]

    otherAxis = Y_AXIS if leadAxis == X_AXIS else X_AXIS
    otherSteps = absSteps[otherAxis]

    bFactor = float(otherSteps) / leadSteps
    # print "bfactor:", bFactor
    otherCount = 0

    steps_per_mm = PrinterProfile.getStepsPerMM(leadAxis)
    sPerStep = 1.0/steps_per_mm

    tstep = 0

    # Lower speed
    maxStepTime = ddprintconstants.maxTimerValue16 / fTimer
    vmin = sPerStep / maxStepTime

    stepsToDo = 0
    if vstart > vmin:

        vend = max(vend, vmin)

        # Number of lead axis steps needed
        dv = vstart - vend
        dt = dv / a
        s = accelDist(vstart, -a, dt)

        # print "vstart, vend, a", vstart, vend, a
        # print "dv, dt, s:", dv, dt, s

        stepsToDo = int(s * steps_per_mm)

    else:

        vstart = vmin

    # print "doing ", stepsToDo, "of", leadSteps

    # # debug
    # prepended = False

    if leadSteps > stepsToDo:

        # Prepend fast steps
        for i in range(leadSteps - stepsToDo):

            # Time we need for this speed change/this step:
            dt = sPerStep / vstart

            # Timervalue for this time
            timerValue = int(dt * fTimer)

            stepBits = [0, 0]
            stepBits[leadAxis] = 1

            otherCount += bFactor

            if otherCount >= 0.5:
                stepBits[otherAxis] = 1
                otherSteps -= 1
                otherCount -= 1.0

            # print "steps, otherCount:", stepBits, otherCount
            pulses.append((tstep, dt, timerValue, stepBits))

            tstep += dt
            leadSteps -= 1

        # prepended = True

    v = vstart
    s = sPerStep

    # while v > vend and leadSteps > 0:
    while leadSteps > 0:

        # Speed after this step
        # print "vstar, -a, s:", vstart, -a, s
        vn1 = vAccelPerDist(vstart, -a, s)

        # Time we need for this speed change/this step:
        dv = v - vn1
        dt = dv / a

        # Timervalue for this time
        timerValue = int(dt * fTimer)

        if timerValue > ddprintconstants.maxTimerValue16:
            # print "break on timeroverflow, v after this step:", vn1, s, dt, timerValue
            break

        # print "v after this step:", vn1, s, dt, timerValue

        stepBits = [0, 0]
        stepBits[leadAxis] = 1

        otherCount += bFactor

        # if otherCount >= 1:
        if otherCount >= 0.5:
            stepBits[otherAxis] = 1
            otherSteps -= 1
            otherCount -= 1.0

        # print "steps, otherCount:", stepBits, otherCount
        pulses.append((tstep, dt, timerValue, stepBits))

        s += sPerStep
        v = vn1
        tstep += dt
        leadSteps -= 1

    # print "Missing steps: ", leadSteps, otherSteps

    # if prepended:
        # print "prepended steps:"
        # pprint.pprint(pulses)

    assert(leadSteps == 0 and otherSteps == 0)
    return pulses

####################################################################################################
def pdbAssert(expr):

    if not expr:
        import pdb
        pdb.set_trace()
####################################################################################################
# Print size in human readable form

def sizeof_fmt(num):
    for unit in ['B','kB','mB','gB','tB','pB','eB','zB']:
        if abs(num) < 1024.0:
            if unit == "B":
                return "%3d %s" % (num, unit)
            return "%3.1f %s" % (num, unit)
        num /= 1024.0
    return "%.1f %s" % (num, 'yB')
####################################################################################################









