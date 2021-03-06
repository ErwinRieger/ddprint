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

import struct, time, math, tty, termios, sys, types, json, fcntl, os
import ddhome, ddadvance, pprint, movingavg, gcodeparser

from ddprintcommands import *
from ddprintstates import *
from ddprinter import Printer
from ddprintconstants import *
from ddconfig import *
from ddprofile import NozzleProfile, MatProfile
from ddvector import vectorMul

####################################################################################################
def sign(x):

    #
    # special handling of 0 and (pseudo-) -0.0
    #
    if (x == 0): #  or ((x<0) and isclose(x, 0)):
        return 1.0
    return math.copysign(1, x)

####################################################################################################
def circaf(a, b, delta):
    return isclose(a, b, delta)

# def isclose(a, b, rel_tol=1e-9, abs_tol=0.0):
def isclose(a, b, abs_tol=1e-9):
    return abs(a-b) < abs_tol

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
            # assert(isclose(maxAllowedEEndSpeed, endSpeed1.eSpeed))

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
        # if isclose(eEndSpeed1, eStartSpeed2, AdvanceEThreshold):
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
            for dim in range(3):

                vdim = startSpeedV2[dim]

                # Speed difference from endspeed to theoretical max speed for axis
                vdiff = vdim - endSpeedV1[dim]

                vdiffAbs = abs(vdiff)

                dimJerk = jerk[dim]

                if vdiffAbs > dimJerk: 
                    speedDiff[dim] = (vdiff, vdiffAbs)

            if speedDiff:

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
        for dim in range(3):

            vdim = startSpeedV2[dim]

            # Speed difference from endspeed to theoretical max speed for axis
            vdiff = vdim - endSpeedV1[dim]

            vdiffAbs = abs(vdiff)

            dimJerk = jerk[dim]

            if vdiffAbs > dimJerk: 
                speedDiff[dim] = (vdiff, vdiffAbs)

        if speedDiff:

            if debugMoves:
                print "E-speed angepasst, XY-speedDiff:", speedDiff

            differenceVector = endSpeedV1.subVVector(startSpeedV2)
            # print "differenceVector, jerk:", differenceVector, jerk

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
        for dim in range(5):

            vdim = startSpeedVMove2[dim]

            # Speed difference from start speed to theoretical max speed for axis
            vdiff = vdim - endSpeedVMove1[dim]

            vdiffAbs = abs(vdiff)

            dimJerk = jerk[dim]

            if vdiffAbs > dimJerk: 
                speedDiff[dim] = (vdiff, vdiffAbs)

        if speedDiff:

            if debugMoves:
                print "speedDiff:", speedDiff

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

    def __init__(self, msg=""):
        self.msg = msg
        self.fd = sys.stdin.fileno()
        try:
            self.prevTcattr = termios.tcgetattr(self.fd)
        except termios.error:
            self.prevTcattr = None
        self.prevFcntl = fcntl.fcntl(sys.stdin, fcntl.F_GETFL) 

    def getc(self):

        print self.msg
        try:
            tty.setcbreak(self.fd)
        except termios.error:
            self.prevTcattr = None
        ch = sys.stdin.read(1)
        if self.prevTcattr:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.prevTcattr)
        return ch

    def getcNB(self):

        # print self.msg
        fcntl.fcntl(sys.stdin, fcntl.F_SETFL, self.prevFcntl | os.O_NONBLOCK)

        ch = None
        try:
            ch = sys.stdin.read(1)
        except IOError:
            pass

        fcntl.fcntl(sys.stdin, fcntl.F_SETFL, self.prevFcntl)
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

#
# Helper for *Straight Line Equations* y = c + m*x
#
class SLE:

    def __init__(self, x1, y1, m):

        self.m = m
        if x1 == 0:
            self.c = y1
        else:
            self.c = y1 - x1*m

    def y(self, x):

        return self.c + x*self.m

    def x(self, y):

        return (y - self.c) / self.m

    def __repr__(self):
        return "SLE: y = %.3f + %.3f * x" % (self.c, self.m)

####################################################################################################

# Compute new stepper direction bits
def directionBits(disp):

    dirbits = 0

    for i in range(5):

        if disp[i] >= 0:
            mask = 1 << i
            dirbits += mask

    return dirbits

####################################################################################################

####################################################################################################

def getVirtualPos(printer, parser):

    # Get currend stepped pos
    res = printer.getPos()

    print "Printer home pos [steps]:", res

    curPosMM = MyPoint(
        X = res[0] / float(parser.steps_per_mm[0]),
        Y = res[1] / float(parser.steps_per_mm[1]),
        Z = res[2] / float(parser.steps_per_mm[2]),
        A = res[3] / float(parser.steps_per_mm[3]),
        B = 0.0 # res[4] / float(parser.steps_per_mm[4]),
        )

    print "Printer is at [mm]: ", curPosMM
    parser.setPos(curPosMM)

    return curPosMM

####################################################################################################

def zRepeatability(parser):

    import random

    assert(0) # todo: transition to printer.printerProfile...

    printer.commandInit(args, PrinterProfile.getSettings())

    feedrate = PrinterProfile.getMaxFeedrate(Z_AXIS)

    assert(0) # commandInit
    ddhome.home(args, printer, parser, planner)

    for i in range(10):

        parser.execute_line("G0 F%d X115 Y210 Z10" % (feedrate*60))
        parser.execute_line("G0 F%d Z%f" % (feedrate*60, random.randint(20, 150)))

    planner.finishMoves()
    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    # printer.sendCommand(CmdEOT)

    printer.waitForState(StateInit)

####################################################################################################

def manualMove(args, printer, parser, planner, axis, distance, feedrate=0, absolute=False):

    ddhome.assureIsHomed(args, printer, parser, planner)

    # Get current pos from printer and set our virtual pos
    getVirtualPos(printer, parser)

    if not feedrate:
        feedrate = printer.printerProfile.getMaxFeedrateI(axis)

    current_position = parser.getPos()
    if absolute:
        d = distance - current_position[axis] 
        assert(abs(d) <= 1000)
        parser.execute_line("G0 F%d %s%f" % (feedrate*60, dimNames[axis], distance))
    else:
        assert(abs(distance) <= 1000)
        parser.execute_line("G0 F%d %s%f" % (feedrate*60, dimNames[axis], current_position[axis] + distance))

    planner.finishMoves()
    # printer.sendCommand(CmdEOT)

    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])

    printer.waitForState(StateInit, log=True)

####################################################################################################


def printFile(args, printer, parser, planner, logObj, gfile, t0, t0_wait, t1, doLog=False):

    ddhome.home(args, printer, parser, planner)

    if args.dummyTempTable:
        downloadDummyTempTable(printer)
    else:
        downloadTempTable(printer, planner.nozzleProfile, planner.matProfile)

    # Send heat up  command
    logObj.log( "Pre-Heating bed (t0: %d)...\n" % t0)
    printer.heatUp(HeaterBed, t0, log=doLog)

    (f, preloadLines) = parser.preParse(gfile, args.baud)

    logObj.log( "Nuber of lines to preload: %d" % preloadLines)

    lineNr = 0

    checkTime = time.time() + 2

    StateWeakPreheat = 0
    StatePreload = 1
    StateStarting = 3
    StatePrinting = 4
    state = StateWeakPreheat

    # for line in f:
    while True:

        if f:
            line = f.readline()
            if line:
                parser.execute_line(line)
                lineNr += 1
            else:
                # Reading done

                logObj.log( "Parsed %d gcode lines." % lineNr)

                # 
                # Add a move to lift the nozzle at end of print
                # 
                endOfPrintLift(parser)

                planner.finishMoves()
                # printer.sendCommand(CmdEOT)

                f = None
        else:
            time.sleep(0.5)

        if time.time() > checkTime:

            if args.testbatch:
                print "\n#"
                print "# MONITOR:", time.time()
                print "#"

            status = printer.getStatus()
            # pprint.pprint(status)
            if doLog:
              printer.ppStatus(status)
              printer.top()

            if args.testbatch:

                pos = printer.getPos()
                print "POS: "
                pprint.pprint(pos)

                counts = printer.getFilSensor()
                print "Filament pos:", counts

                printer.checkStall(status)

            #
            # Check temp and start print
            #
            if state == StateWeakPreheat:
                
                if status["t0"] >= t0_wait:
                    # Start pre-heating of hotend, bed is still heating
                    logObj.log( "Pre-Heating extruder %.2f (t1: %d)...\n" % (t1/2.0, t1))
                    printer.heatUp(HeaterEx1, t1/2)
                    state = StatePreload

            elif state == StatePreload:

                # Wait until entire stepdata or preload amout of it is sent and bed is 
                # heated.
                if ((not f) or (lineNr >= preloadLines)) and status["t0"] >= t0:
                    # Heat hotend further if bed is at temp
                    logObj.log( "Heating extruder (t1: %d)...\n" % t1 )
                    tempRamp = printer.heatUpRamp(HeaterEx1, t1)
                    state = StateStarting

            elif state == StateStarting:

                try:
                    tempRamp.next()
                except StopIteration:
                    logObj.log("Starting print...\n" )

                    # Send print start command
                    printer.startPrint()
                    state = StatePrinting

            elif state == StatePrinting:

                # Stop sending moves on error of if print is done
                if not printer.stateMoving(status) or ((not f) and args.testbatch):
                    break

            checkTime = time.time() + 1.5

    logObj.log( "Print finished, duration: %s" % printer.getPrintDuration() )

    if args.testbatch:

        print "#"
        print "# NOTE: testbatch mode, dont monitoring rest of print !!!"
        print "# NOTE: heaters are RUNNING !!!"
        print "# NOTE: note printer is not momitored - no cooldown, no homing, no disabling anything!"
        print "#"
        return

    printer.coolDown(HeaterEx1)
    printer.coolDown(HeaterBed)

    ddhome.home(args, printer, parser, planner)

    printer.sendCommand(CmdDisableSteppers)

    if not args.noCoolDown:
        printer.coolDown(HeaterEx1, wait=150, log=doLog)

    # Stop hotend fan
    printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(0)])



####################################################################################################

def insertFilament(args, printer, parser, planner, feedrate):

    ddhome.assureIsHomed(args, printer, parser, planner)

    pp = printer.printerProfile

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

            # XXX hardcoded feedrate
            parser.execute_line("G0 F%d A%f" % (5*60, aofs))

            planner.finishMoves()
            printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
            # printer.sendCommand(CmdEOT)
            printer.waitForState(StateInit, wait=0.1)

    # Move to mid-position
    maxFeedrate = pp.getMaxFeedrateI(X_AXIS)
    parser.execute_line("G0 F%d X%f Y%f" % (maxFeedrate*60, planner.MAX_POS[X_AXIS]/2, planner.MAX_POS[Y_AXIS]/2))

    planner.finishMoves()

    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    # printer.sendCommand(CmdEOT)

    printer.waitForState(StateInit)

    t1 = planner.matProfile.getHotendGoodTemp()
    printer.heatUp(HeaterEx1, t1, wait=t1 * 0.95)

    print "\nInsert filament.\n"
    manualMoveE()

    print "\nForwarding filament.\n"
    manualMove(args, printer, parser, planner, A_AXIS, pp.getBowdenLength(), feedrate)

    print "\nExtrude filament.\n"
    manualMoveE()

    #
    # Retract
    #
    parser.execute_line("G10")
    planner.finishMoves()

    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    # printer.sendCommand(CmdEOT)

    printer.waitForState(StateInit)

    printer.sendCommand(CmdDisableSteppers) # power off motors

    if not args.noCoolDown:
        printer.coolDown(HeaterEx1, wait=150, log=True)

####################################################################################################

def removeFilament(args, printer, parser, planner, feedrate):

    ddhome.assureIsHomed(args, printer, parser, planner)

    pp = printer.printerProfile

    # Move to mid-position
    maxFeedrate = pp.getMaxFeedrateI(X_AXIS)
    parser.execute_line("G0 F%d X%f Y%f" % (maxFeedrate*60, planner.MAX_POS[X_AXIS]/2, planner.MAX_POS[Y_AXIS]/2))

    planner.finishMoves()

    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    # printer.sendCommand(CmdEOT)

    printer.waitForState(StateInit)

    t1 = (planner.matProfile.getHotendGoodTemp() + planner.matProfile.getHotendMaxTemp()) / 2
    printer.heatUp(HeaterEx1, t1, wait=t1)

    # Filament vorwärts feeden um den 'retract-pfropfen' einzuschmelzen
    manualMove(args, printer, parser, planner, A_AXIS, pp.getRetractLength() + 50, 5)

    # Retract filament
    manualMove(args, printer, parser, planner, A_AXIS, -1.25 * pp.getBowdenLength(), feedrate)

    printer.sendCommand(CmdDisableSteppers) # power off motors

    if not args.noCoolDown:
        printer.coolDown(HeaterEx1, wait=150, log=True)

####################################################################################################

def bedLeveling(args, printer, parser, planner):

    t1 = args.t1 or planner.matProfile.getHotendBaseTemp()
    printer.heatUp(HeaterEx1, t1)

    # Reset bedlevel offset in printer profile
    if args.relevel:
        head_height = planner.LEVELING_OFFSET
    else:
        printer.printerProfile.override("add_homeing_z", 0)
        head_height = planner.HEAD_HEIGHT

    # printer.commandInit(args)
    ddhome.home(args, printer, parser, planner)

    zFeedrate = printer.printerProfile.getMaxFeedrateI(Z_AXIS)
    kbd = GetChar("Enter (u)p (d)own (U)p 1mm (D)own 1mm (2-5) Up Xmm (q)uit")

    # "bedLevelMode": "Triangle3Point"
    levelMode = printer.printerProfile.getBedLevelMode()

    if levelMode == "Triangle3Point":
        # Ultimaker UM2 style, 3 srews triangle shaped
        levelPoints = [
                (planner.X_MAX_POS/2, planner.Y_MAX_POS-15, head_height, "back mid"),
                (15, 15, planner.LEVELING_OFFSET, "front left"),
                (planner.X_MAX_POS-15, 15, planner.LEVELING_OFFSET, "front right"),
                ]
    elif levelMode == "U5Point":
        # JennyPrinter style, 5 srews in an U-shape
        levelPoints = [
                (15, 15, head_height, "front left"),
                (15, planner.Y_MAX_POS-15, planner.LEVELING_OFFSET, "back left"),
                (planner.X_MAX_POS-15, planner.Y_MAX_POS-15, planner.LEVELING_OFFSET, "back right"),
                (planner.X_MAX_POS-15, 15, planner.LEVELING_OFFSET, "front right"),
                (planner.X_MAX_POS/2, planner.Y_MAX_POS-15, planner.LEVELING_OFFSET, "back mid"),
                ]
    else:
        assert(0)

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

            parser.execute_line("G0 F%d Z%f" % (zFeedrate*60, zofs))

            planner.finishMoves()
            printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
            # printer.sendCommand(CmdEOT)

            printer.waitForState(StateInit, wait=0.1)

    printer.heatUp(HeaterEx1, t1, wait=t1-5, log=True)

    feedrate = printer.printerProfile.getMaxFeedrateI(X_AXIS)

    pointNumber = 0
    for (x, y, z, pointName) in levelPoints:

        print "Leveling point %d/%d" % (pointNumber+1, len(levelPoints))

        if pointNumber == 0:
            parser.execute_line("G0 F%d X%f Y%f Z%f" % (feedrate*60, x, y, z))
        else:
            parser.execute_line("G0 F%d Z5" % (zFeedrate*60))
            parser.execute_line("G0 F%d X%f Y%f" % (feedrate*60, x, y))
            parser.execute_line("G0 F%d Z%f" % (zFeedrate*60, z))

        planner.finishMoves()
        printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
        # printer.sendCommand(CmdEOT)
        printer.waitForState(StateInit, wait=0.1)

        if (pointNumber == 0) and (not args.relevel):

            manualMoveZ()

            current_position = parser.getPos()
            print "curz: ", current_position[Z_AXIS]

            add_homeing_z = (current_position[Z_AXIS] * -1) + planner.LEVELING_OFFSET

            # Store into printer profile (not persistent, just for the rest of the levelling procedure):
            printer.printerProfile.override("add_homeing_z", add_homeing_z)

            # Finally we know the zero z position
            current_position[Z_AXIS] = planner.LEVELING_OFFSET

            # Adjust the virtual position
            parser.setPos(current_position)

            # Adjust the printer position in firmware part
            posStepped = vectorMul(current_position, parser.steps_per_mm)
            payload = struct.pack("<iiiii", *posStepped)
            printer.sendCommand(CmdSetHomePos, binPayload=payload)

        else:
        
            raw_input("\nAdjust %s buildplate screw and press <Return>\n" % pointName)

        pointNumber += 1

    ddhome.home(args, printer, parser, planner)
    printer.sendCommand(CmdDisableSteppers) # Force homing/reset

    if not args.relevel:
        raw_input("\n! Please update your Z-Offset (add_homeing_z) in printer profile: %.3f\n" % add_homeing_z)

    if not args.noCoolDown:
        printer.coolDown(HeaterEx1, wait=100, log=True)

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
        ddhome.home(args, printer, parser, planner)

    printer.sendCommand(CmdStopMove)
    printer.sendCommand(CmdDisableSteppers)

####################################################################################################

def heatHotend(args, matProfile, printer):

    t1 = args.t1 or matProfile.getHotendBaseTemp()

    startTime = time.time()
    printer.heatUp(HeaterEx1, t1, wait=t1, log=True)

    kbd = GetChar()
    print("\nTemp %d reached in %.2f sec. Press return to stop heating...\n" % (t1, time.time() - startTime))
    while not kbd.getcNB():
        status = printer.getStatus()
        printer.ppStatus(status, msg="T %.2f" % (time.time() - startTime))
        time.sleep(1)

    if not args.noCoolDown:
        printer.coolDown(HeaterEx1, wait=150, log=True)

####################################################################################################

#
# Execute a single gcode command on the printer and wait for completion
#
def execSingleGcode(parser, gcode):

    planner = parser.planner
    printer = planner.printer

    # xxxx use commandInit() here or do homing/move to mid first
    printer.xsendPrinterInit()

    parser.execute_line(gcode)
    planner.finishMoves()

    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    # printer.sendCommand(CmdEOT)

    printer.waitForState(StateInit)

####################################################################################################

# Heat up nozzle
# Retract
# Wait for user to change nozzle
# Un-Retract
# Cooldown
def changeNozzle(args, parser):

    assert(0) # todo: transition to printer.printerProfile...

    planner = parser.planner
    printer = planner.printer

    retract(args, parser, doCooldown = False)

    feedrate = PrinterProfile.getMaxFeedrate(X_AXIS)
    execSingleGcode(parser, "G0 F%d X%f Y%f" % (feedrate*60, planner.MAX_POS[X_AXIS]/2, planner.MAX_POS[Y_AXIS]*0.1))

    raw_input("Now change nozzle, Press return to stop heating...")

    execSingleGcode(parser, "G11")

    if not args.noCoolDown:
        printer.coolDown(HeaterEx1, wait=100, log=True)

####################################################################################################
#
# Measure closed loop step-response of the hotend and plot it with gnuplot.
# Do three setpoint changes: the first is from current temperature to args.t1 or 200 °C, then
# after reaching this first set temperature a step upwards to 120%, then a downward step to 50% of
# the first temperature (to test a bigger downward temp change, *asymmetric gain*) is done.
#
def stepResponse(args, printer):

    def stopHeater():
        printer.coolDown(HeaterEx1)
        printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(0)])

    # Destination temperature
    tDest1 = args.t1 or 200
    tDest2 = int(round(tDest1 * 1.2))
    tDest3 = tDest1
    assert(tDest1 <= 275)

    # Bail out if tmax reached
    tmax = tDest1 * 1.5

    timeConstant = int(printer.printerProfile.getTuI()) * 4

    # Open output gnuplot file
    f = open("stepresponse_closed.gnuplot", "w")

    f.write("set yrange [0:%d]\n" % tmax)
    f.write("set grid\n")
    f.write("plot \"-\" using 1:2 with linespoints title \"StepResponse\",")
    f.write("%d title \"tDest1\", %d title \"tDest2\", %d title \"tDest3\"\n" % (tDest1, tDest2, tDest3))

    # Do the first step
    print "Starting input step to %d °" % tDest1
    # printer.heatUp(HeaterEx1, tDest1)
    printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(100)])

    timeStart = time.time()
    status = printer.getStatus()
    tempStart = status["t1"]

    print "Temp:", tempStart
    f.write("0 %f\n" % tempStart)

    temp = tempStart

    for tDest in (tDest1, tDest2, tDest3):

        tempAvg = movingavg.MovingAvg( timeConstant )

        # Stop if temp curve gets flat enough
        flatReached = False

        # Do the n-th temperature step
        print "Setting input step to %d °" % tDest
        printer.heatUp(HeaterEx1, tDest)
        # tempRamp = printer.heatUpRamp(HeaterEx1, tDest)

        while True:
        
            if temp > tmax:
                print "Error, max temp (%d) reached: " % tmax, temp
                # stopHeater()
                # return

            time.sleep(1)

            # try:
                # print "temp ramp..."
                # tempRamp.next()
            # except StopIteration:
                # print "\ntemp reached...\n"

            tim = time.time()
            relTime = tim-timeStart

            status = printer.getStatus()

            print "\ntemp: %.2f/%.2f, avg: %.2f, pwm: %d\n" % (temp, tDest, tempAvg.mean(), status["pwmOutput"])

            temp = status["t1"]

            tempAvg.add(temp)

            f.write("%f %f\n" % (relTime, temp))

            if tempAvg.valid() and tempAvg.near(0.05, tDest):
                print "Temp is in 5% range, this temp done..."
                break

        # Do the second step...

    f.write("e\npause mouse close\n")
    stopHeater()

####################################################################################################

def measureHotendStepResponse(args, printer, matProfile):

    print "*************************************************************"
    print "* Record open loop step response of hotend to determine     *"
    print "* the pid control parameters for temperature control.       *"
    print "*************************************************************"

    def stopHeater():
        printer.setTempPWM(HeaterEx1, 0)

    tmax = matProfile.getHotendMaxTemp()

    ## Eingangssprung, nicht die volle leistung, da sonst die temperatur am ende
    ## der sprungantwort zu stark überschwingt und zu hoch wird.
    Xo = printer.printerProfile.getXo()
    interval = 0.1

    navg = int(round(30/interval))
    tempAvg = movingavg.MovingAvg( navg )

    print "Starting input step with pwm value: %d, tmax is: %.2f, nAvg: %d" % (Xo, tmax, navg)

    # Fan is running while printing (and filament has to be molten), so run fan 
    # at max speed to simulate this energy-loss.
    printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(255)])

    maxStartTemp = 35 # Note: arbitrary value, autoTune shold be run with a cold hotend

    temp = printer.getTemp(doLog = False)[HeaterEx1] # current temp
    if temp > maxStartTemp:
        print "Current hotend temp %.2f, this is higher than the max. starting temp of %.2f." % (temp, maxStartTemp)
        printer.coolDown(HeaterEx1, 0, wait=maxStartTemp, log=True)

    temp = tempStart = printer.getTemp(doLog = False)[HeaterEx1] # current temp

    # Output file for raw data
    fraw = open("autotune.raw.json", "w")
    fraw.write("""{
    "PrinterName": "%s",
    "Xo": %d,
    "dt": %f,
    "startTemp": %f,
    "columns":  "time temperature",
    """ % (printer.getPrinterName(args), Xo, interval, tempStart))

    print "Current hotend temp: %.f" % tempStart

    # Apply input step:
    printer.setTempPWM(HeaterEx1, Xo)
    timeStart = time.time()

    data = []

    maxTemp = 0

    while True:

        if temp > tmax:
            print "Error, max temp (%d) reached (you should decrease Xo): " % tmax, temp
            stopHeater()
            return

        tim = time.time()
        temp = printer.getTemp(doLog = False)[1]

        relTime = tim-timeStart
        Mu = temp-tempStart

        if Mu > maxTemp:
            maxTemp = Mu

        data.append("       [%f, %f]" % (relTime, Mu))

        tempAvg.add(Mu)

        print "\rTime %.2f, temp: %.2f, Mu: %.2f, navg: %d, moving temp avg: %.2f" % (relTime, temp, Mu, navg, tempAvg.mean()),
        sys.stdout.flush()

        # Window for running average to detect steady state is 5%
        if tempAvg.valid() and tempAvg.near(0.05):
            print "\nTemp reached 5% steady state...", relTime, Mu, tempAvg.mean()
            break

        navg = int(round(max( len(data)/3, 30/interval )))
        tempAvg.expand(navg)

        time.sleep(interval)

    stopHeater()

    fraw.write("""
    "tEnd": %f,
    "data": [
    """ % maxTemp)
    fraw.write("    " + ",\n".join(data))
    fraw.write("    ]\n}\n")
    fraw.close()

    print "Step response done, result in file ./autotune.raw.json, evaluate it with pidAutoTune.py."

    printer.coolDown(HeaterEx1, wait=100, log=True)
    printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(0)])

####################################################################################################
#
# Allow comments in json files.
#
def jsonLoad(f):

    s = ""
    for l in f.readlines():
        l = l.strip()
        if l.startswith('"#') or l.startswith("#"):
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

def genTempTable(printerProfile, nozzleProfile, matProfile):

    spm = printerProfile.getStepsPerMMI(A_AXIS)
    aFilament = matProfile.getMatArea()

    startTemp = matProfile.getHotendBaseTemp()

    # Temps lower than 170 not handled
    # todo: use maxtemp - NExtrusionLimit as startvalue
    assert(startTemp >= 170)

    # xxx same as in PathData
    # Interpolate best case flowrate (into air)
    (sleTempBest, slePwmBest) = matProfile.getFrSLE()
    print "best case flowrate:", sleTempBest, slePwmBest

    # Interpolate worst case flowrate (100% fill with small nozzle)
    (sleTempPrint, slePwmPrint) = matProfile.getFrSLEPrint()
    print "worst case flowrate:", sleTempPrint, slePwmPrint

    # XXX simple way, use average of best and worst flowrate:
    tempSLE = SLE(x1=0, y1=(sleTempBest.c+sleTempPrint.c)/2, m=sleTempBest.m)
    pwmSLE = SLE(x1=0, y1=(slePwmBest.c+slePwmPrint.c)/2, m=slePwmBest.m)

    print "Temp sle:", tempSLE
    print "Pwm sle:", pwmSLE

    table = []
    for i in range(NExtrusionLimit):

        t = 170 + i

        flowrate = max(tempSLE.y(t), 0.1)

        espeed = flowrate / aFilament
        # print "flowrate for temp %f: %f -> espeed %f" % (t, flowrate, espeed)

        steprate = espeed * spm
        tvs = 1.0/steprate
        timerValue = min(int(fTimer / steprate), 0xffff) 

        print "    Temp: %f, max flowrate: %.2f mm³/s, max espeed: %.2f mm/s, steps/s: %d, steprate: %d us, timervalue: %d" % (t, flowrate, espeed, int(steprate), int(tvs*1000000), timerValue)
        table.append(timerValue)

    return (170, table)

####################################################################################################

def eTimerValue(printer, eSpeed):

    spm = printer.printerProfile.getStepsPerMMI(A_AXIS)
    steprate = eSpeed * spm
    timerValue = int(fTimer / steprate)
    return timerValue

####################################################################################################

def printTempTable(temp, tempTable):

    assert(0) # todo: transition to printer.printerProfile...

    print "TempTable for 1.75mm filament:"
    print "Basetemp: %d:" % temp

    filamentArea = math.pi*pow(1.75, 2) / 4.0
    spm = PrinterProfile.getStepsPerMM(A_AXIS)

    for timerValue in tempTable:

        steprate = fTimer / timerValue

        speed = (steprate / spm) * filamentArea

        print "    Temp: %d, max extrusion: %.1f mm³/s, steps/s: %d, timervalue: %d" % (temp, speed, int(steprate), timerValue)

        # temp += 2
        temp += 1

####################################################################################################

def downloadTempTable(printer, nozzleProfile, matProfile):

    (startTemp, tempTable) = genTempTable(printer.printerProfile, nozzleProfile, matProfile)

    print "Downloading TempTable..."

    printer.setTempTable(startTemp, NExtrusionLimit, tempTable)

####################################################################################################

def downloadDummyTempTable(printer):

    timerValue = int(fTimer / printer.printerProfile.getMaxStepperFreq())

    tempTable = []
    for i in range(NExtrusionLimit):
        tempTable.append( timerValue )

    print "Downloading Dummy TempTable..."

    # todo: use maxtemp - NExtrusionLimit as startvalue
    printer.setTempTable(170, NExtrusionLimit, tempTable)

####################################################################################################

def getStartupTime(printer, feedrate):

    eAccel = printer.printerProfile.getMaxAxisAccelerationI()[A_AXIS]

    # Zeit bis sich der messwert der target geschwindigkeit
    # stabilisiert hat.
    # 1. timer accel ramp
    tAccel = feedrate / eAccel

    return tAccel

####################################################################################################
# 
def measureFlowrateStepResponse(args, parser):

    assert(0) # todo: transition to printer.printerProfile...

    planner = parser.planner
    printer = planner.printer

    nozzleSize = printer.nozzleProfile.getSizeI()

    aFilament = planner.matProfile.getMatArea()

    printerProfile = PrinterProfile.get()

    printer.commandInit(args, PrinterProfile.getSettings())

    # Disable flowrate limit
# xxx not needed if using ContinuousE mode
    # printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])

    # Disable temp-flowrate limit
    downloadDummyTempTable(printer)

    # Move to mid-position, xxx move code to own function
    feedrate = PrinterProfile.getMaxFeedrate(X_AXIS)
    parser.execute_line("G0 F%d X%f Y%f" % (feedrate*60, planner.MAX_POS[X_AXIS]/2, planner.MAX_POS[Y_AXIS]/2))

    planner.finishMoves()
    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    # printer.sendCommand(CmdEOT)
    printer.waitForState(StateInit)

    printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(100)])

    # start with 1mm³/s
    # XXX speed too high for small nozzles!?
    feedrate = (args.flowrate or 1.0) / aFilament

    tWait = 0.1

    eMotorRunning = False

    # Running average of hotend temperature
    # tempAvg = EWMA(0.5)
    tempAvg = movingavg.MovingAvg(10)
    flowAvg = movingavg.MovingAvgReadings(10)
    # Running average of *grip*
    # ratioAvg = EWMA(0.5)
    # ratioAvg = MovingAvg(10)
    # pwmAvg = EWMA(0.5)

    minGrip = 0.90

    t1 = args.t1 or planner.matProfile.getHotendBaseTemp()

    dt = PrinterProfile.getFilSensorInterval()

    steps_per_mm = PrinterProfile.getStepsPerMM(A_AXIS)

    pcal = PrinterProfile.get().getFilSensorCalibration()

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
    T66 = 125 # xxx use timeConstant here...
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
    """ % (printer.getPrinterName(args), pwm0, sprung, dt, startTemp))
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
    printer.sendCommandParamV(CmdContinuousE, [packedvalue.uint16_t(eTimerValue(printer, feedrate))])

    mode = "starting"

    tStart = time.time()
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

        if r > minGrip:
            # increase speed

            frincrease = feedrate * max(0.05 * (max(r, minGrip) - minGrip), 0.0001) # increase feedrate by max 5% and min 0.1% mm per second
            feedrate += frincrease
            print "\nincreased feedrate by %.3f to %.3f" % (frincrease, feedrate)
            
            # set new feedrate:
            printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(eTimerValue(printer, feedrate))])

        fraw.write("    [%f, %f, %f, %f, %f]" % (time.time()-tStart, targetFlowRate, r, targetFlowRate*r, t1Avg))

        if time.time() >= tStart+tStep and mode == "starting":
            print "T66 reached, do step %d --> %d" % (pwm0, pwm0+sprung)
            printer.setTempPWM(HeaterEx1, pwm0+sprung)
            mode = "measure"

        if time.time() >= tStart+tEnd:
            print "2*T66 reached, break"
            break

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
        printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(eTimerValue(printer, feedrate))])
        time.sleep(0.5)

    # Stop continuos e-mode
    printer.sendCommandParamV(CmdContinuousE, [packedvalue.uint16_t(0)])

# xxx not needed if using ContinuousE mode
    # Re-enable flowrate limit
    # printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(1)])


####################################################################################################

def measureTempFlowrateCurve(args, printer, parser, planner):

    aFilament = planner.matProfile.getMatArea()

    ddhome.assureIsHomed(args, printer, parser, planner)

# xxx not needed if using ContinuousE mode
    # Disable flowrate limit
    # printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])

    # Disable temp-flowrate limit
    downloadDummyTempTable(printer)

    # Move to mid-position, xxx factor out into own function
    feedrate = printer.printerProfile.getMaxFeedrateI(X_AXIS)
    parser.execute_line("G0 F%d X%f Y%f" % (feedrate*60, planner.MAX_POS[X_AXIS]/2, planner.MAX_POS[Y_AXIS]/2))

    planner.finishMoves()
    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
    # printer.sendCommand(CmdEOT)
    printer.waitForState(StateInit)

    printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(100)])

    # start with 1mm³/s
    # XXX speed too high for small nozzles!?
    feedrate = (args.flowrate or 1.0) / aFilament

    dt = printer.printerProfile.getFilSensorIntervalI()
    pcal = printer.printerProfile.getFilSensorCalibration()

    ####################################################################################################
    # * set initial temp wait for min temp
    # * start measurement loop
    ####################################################################################################

    # Running average of hotend temperature
    tempAvg = movingavg.MovingAvg(int(round(1.0/dt)))
    flowAvg = movingavg.MovingAvgReadings(int(round(1.0/dt)), startavg = 1.0)

    minGrip = 0.90
    data = []

    ####################################################################################################

    lastGoodFlowrate = 0

    for t1 in [planner.matProfile.getHotendBaseTemp(), planner.matProfile.getHotendMaxTemp()]:

      print "\nHeating up to target temp:", t1
      # printer.heatUp(HeaterEx1, t1, wait=round(t1 - t1*.01), log=True) # Wait until 99% of temp reached
      printer.heatUp(HeaterEx1, int(round(t1*1.025)), wait=t1, log=True)

      # Set extruder motor speed
      print "\nRunning extruder motor with %.2f mm/s" % feedrate
      printer.sendCommandParamV(CmdContinuousE, [packedvalue.uint16_t(eTimerValue(printer, feedrate))])

      tStart = time.time()
      startup = 5.0       # initial time for averages to settle

      # Fix pwm value, enter *pwmMode*
      status = printer.getStatus()
      pwm = status["pwmOutput"]
      print "Fixed PWM:", pwm

      printer.setTempPWM(HeaterEx1, pwm)

      while True:

        status = printer.getStatus()
        actT1 = status["t1"]
        tempAvg.add(actT1)

        fsreadings = printer.getFSReadings()
        flowAvg.addReadings(fsreadings, pcal)

        t1Avg = tempAvg.mean()
        r = flowAvg.mean()

        targetFlowRate = feedrate * aFilament

        currentFlowrate = targetFlowRate * r

        print "\rt: %.2f, TempAvg: %.1f, fixed pwm: %d, target flowrate: %.3f mm³/s, actual flowrate: %.2f mm³/s, current ratio: %.2f" % \
                (time.time()-tStart, t1Avg, pwm, targetFlowRate, currentFlowrate, r),
        sys.stdout.flush()

        if r > 2:

            print "mean, pcal", pcal

            print "fsreadings:", fsreadings

            print "index:", flowAvg.index
            print "navg:", flowAvg.navg
            print "nValues:", flowAvg.nValues
            print "array:", flowAvg.array
            assert(0)

        if startup > 0:
            time.sleep(dt)
            startup -= dt
            continue

        if r >= minGrip:

            # Increase feedrate by max 1% and min 0.01% mm per second
            frincrease = feedrate * max(0.01 * (max(r, minGrip) - minGrip), 0.0001)
            feedrate += frincrease
            # print "\nincreased feedrate by %.3f to %.3f" % (frincrease, feedrate)
        
            # Set new feedrate:
            printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(eTimerValue(printer, feedrate))])

            lastGoodFlowrate = currentFlowrate

        else:

            #
            # Flowrate got to high for this PWM/temperature values, do next step or exit measurement.
            #
            print "\nEnd step, average flowrate: %.2f mm³/s at pwm: %d, temp: %.1f °C, ratio: %.2f, break" % \
                    (lastGoodFlowrate, pwm, t1Avg, r)
            data.append( (lastGoodFlowrate, pwm, t1Avg) )

            # Enable PID mode, leave *pwmMode*
            printer.setTempPWM(HeaterEx1, 0)
            break

        time.sleep(dt)

    ####################################################################################################

    printer.coolDown(HeaterEx1)

    # Slow down E move
    while feedrate > 1:
        feedrate -= 0.1
        printer.sendCommandParamV(CmdSetContTimer, [packedvalue.uint16_t(eTimerValue(printer, feedrate))])
        time.sleep(0.1)

    # Stop continuos e-mode
    printer.sendCommandParamV(CmdContinuousE, [packedvalue.uint16_t(0)])

# xxx not needed if using ContinuousE mode
    # Re-enable flowrate limit
    # printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(1)])

    ####################################################################################################
    dfr = data[1][0] - data[0][0]
    dpwm = data[1][1] - data[0][1]
    dtemp = data[1][2] - data[0][2]

    # todo: create template material profile with name from commandline
    print ""
    print "# Material properties:"
    print "# a1 for pwm"
    print '"Kpwm": %.4f,' % (dfr / dpwm)
    print "# a1 for temp"
    print '"Ktemp": %.4f,' % (dfr / dtemp)

    print "# a0 for pwm"
    print '"P0pwm": %.4f,' % data[0][1]
    print "# a0 for temp"
    print '"P0temp": %.4f,' % data[0][2]

    print "# feedrate at a0"
    print '"FR0pwm": %.4f,\n' % data[0][0]

    printer.sendCommand(CmdDisableSteppers)
    printer.coolDown(HeaterEx1, wait=100, log=True)
    printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(0)])

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




####################################################################################################

# 
# Ablauf messung:
# 
# * starttemp: midrange (maxtemp+goodtemp) / 2
# * flowrate: FR0pwm (zunächst objkt so slicen)
#   -> für esun black petg: speed = FR0pwm / (nozzle*lh) = 11.6253 / (0.8*0.15) = 95 mm/s
#   -> für dasfilament white pla: speed = FR0pwm / (nozzle*lh) = 9.3865 / (0.8*0.15) = 95 mm/s
# * druck von erstem layer abwarten -> getpos()
# * ab dem zweitem layer temperatur schrittweise absenken und grip überwachen
# * abbruch falls avg(grip) < 95%, das ist dann neues P0temp (P0pwm wird errechnet, da wir zur veeinfachung der messung
#   die stabilisierung der tempregelung nicht abwarten).


def xstartPrint(args, printer, parser, planner, t1):

        ddhome.home(args, printer, parser, planner)

        t0 = planner.matProfile.getBedTemp()
        t0Wait = min(t0, printer.printerProfile.getWeakPowerBedTemp())

        # Send heat up  command
        print "\nHeating bed (t0: %d)...\n" % t0
        printer.heatUp(HeaterBed, t0, log=True)

        # Disable flowrate limit
        printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(0)])

        # Disable temp-flowrate limit
        downloadDummyTempTable(printer)

        (f, _) = parser.preParse(args.gfile, args.baud)

        checkTime = time.time() + 2

        StateWeakPreheat = 0
        StateHeatBed = 1
        StateStarting = 3

        state = StateWeakPreheat

        while True:

            if f:
                line = f.readline()
                if line:
                    parser.execute_line(line)
                else:
                    # Reading done
                    print "Parsed all gcode lines."
                    planner.finishMoves()
                    # printer.sendCommand(CmdEOT)
                    f = None
            else:
                time.sleep(0.5)

            if time.time() > checkTime:

                status = printer.getStatus()
                printer.ppStatus(status)
                printer.top()

                #
                # Check temp and start print
                #
                if state == StateWeakPreheat:
                    
                    if status["t0"] >= t0Wait:
                        # Start pre-heating of hotend, bed is still heating
                        print "\nPre-Heating extruder %.2f (t1: %d)...\n" % (t1/2.0, t1)
                        printer.heatUp(HeaterEx1, t1/2)
                        state = StateHeatBed

                elif state == StateHeatBed:

                    # Wait until entire stepdata is sent and bed is 
                    # heated.
                    if (not f) and (status["t0"] >= t0):
                        # Heat hotend if bed is at temp
                        print "\nHeating extruder (t1: %d)...\n" % t1
                        tempRamp = printer.heatUpRamp(HeaterEx1, t1)
                        state = StateStarting

                elif state == StateStarting:

                    try:
                        tempRamp.next()
                    except StopIteration:
                        print "\nStarting print...\n"

                        # Send print start command
                        printer.startPrint()
                        break

                checkTime = time.time() + 1.5

def measureTempFlowrateCurve2(args, printer, parser, planner):

    # Temperature-band
    def tempGood(t, t1, percent):
        return abs(t1 - t) <= (t1*percent)

    print ""
    print "measureTempFlowrateCurve2():"
    print ""

    goodtemp = planner.matProfile.getHotendGoodTemp()
    maxtemp = planner.matProfile.getHotendMaxTemp()

    tempSpan = maxtemp - goodtemp

    t1 = goodtemp + tempSpan / 2

    ####################################################################################################

    p0pwm = planner.matProfile.getP0pwm() # xxx hardcoded in firmware!

    aFilament = planner.matProfile.getMatArea()

    e_steps_per_mm = printer.printerProfile.getStepsPerMM(A_AXIS)

    nAvg = 10
    print "navg:", nAvg

    # Running average of hotend temperature and grip
    tempAvg = movingavg.MovingAvg(nAvg, t1)
    pwmAvg = movingavg.MovingAvg(nAvg, p0pwm)
    gripAvg = movingavg.MovingAvg(nAvg, 1.0)
    flowAvg = movingavg.MovingAvg(nAvg)

    minGrip = 0.90

    ####################################################################################################

    #
    # Start print:
    #
    xstartPrint(args, printer, parser, planner, t1)

    # Assure we have read layerheight from gcode comments
    assert(parser.layerHeight)

    lastEPos = 0.0
    lastTime = 0.0

    #
    # Wait for second layer
    #
    # XXX add timeout here, deadlock 
    #
    print "\nPrint started, waiting for start of first layer..."
    curPosMM = getVirtualPos(printer, parser)
    while curPosMM.Z >= parser.layerHeight:
        print "waiting for first layer, Z pos:", curPosMM.Z
        status = printer.getStatus()
        printer.ppStatus(status)
        time.sleep(1)
        curPosMM = getVirtualPos(printer, parser)

    # XXX add timeout here, deadlock 
    print "\nFirst layer started, waiting for second layer..."
    while curPosMM.Z < parser.layerHeight:
        print "waiting for second layer, Z pos:", curPosMM.Z
        status = printer.getStatus()
        printer.ppStatus(status)

        lastEPos = status["extruder_pos"]
        lastTime = time.time()

        time.sleep(1)
        curPosMM = getVirtualPos(printer, parser)

    timeConstant = printer.printerProfile.getTgI() 
    tempdec = float(t1 - goodtemp) / timeConstant
    print "Temperature decreasing step:", tempdec

    #
    # Decrease temp while monitoring feeder slippage
    #
    t = t1
    while True:

        status = printer.getStatus()
        printer.ppStatus(status)

        actT1 = status["t1"]
        tempAvg.add(actT1)

        pwmAvg.add(status["pwmOutput"])

        grip = 1.0
        if status["slippage"]:
          grip = 1.0 / status["slippage"]

        # temp in 5% band
        # if tempGood(t1Avg, t1, 0.025):
        gripAvg.add(grip)

        t1Avg = tempAvg.mean()
        pAvg = pwmAvg.mean()
        gAvg = gripAvg.mean()

        # current target flowrate
        ePos = status["extruder_pos"]
        tim = time.time()

        deltaE = ePos - lastEPos
        deltaTime = tim - lastTime

        deltaEmm = float(deltaE)/e_steps_per_mm
        flowrate = (deltaEmm*aFilament)/deltaTime

        flowAvg.add(flowrate)

        # Current measured flowrate
        frAvg = flowAvg.mean() * gAvg

        print deltaE, deltaTime, deltaEmm, flowrate

        print "Avg: temp: %.2f, pwm: %.2f, grip: %.2f, flowrate: %.2f mm³/s" % (t1Avg, pAvg, gAvg, frAvg)

        if tempAvg.valid():

            if not printer.stateMoving(status):
                print "XXX testprint ended before measurement done!"
                break

            # temp in 5% band
            if tempGood(t1Avg, t1, 0.025):

                t -= tempdec

                if int(t) != status["targetT1"]:
                    t1 = int(t)
                    print "setting new temp:", t1
                    printer.heatUp(HeaterEx1, t1)

            if gAvg <= minGrip:
                print "break on mingrip...", gAvg, minGrip
                break

            if t1Avg < goodtemp:
                print "Test did not converge! break on temp (goodtemp reached) ...", t1Avg, goodtemp
                break

        lastEPos = ePos
        lastTime = tim

        time.sleep(1)

    ####################################################################################################

    # Stop print
    print "soft-stop..."
    if printer.stateMoving(status):
        printer.sendCommand(CmdSoftStop)

    # Increase temp of hotend to release stress from hotend
    # for the rest of the print.
    printer.heatUp(HeaterEx1, min(maxtemp, t1+10))

    # Wait till print is cancelled.
    while printer.stateMoving(status):
        time.sleep(2)
        status = printer.getStatus()
        printer.ppStatus(status)

    printer.coolDown(HeaterBed)
    printer.coolDown(HeaterEx1)

    ddhome.home(args, printer, parser, planner)

    ####################################################################################################

    # todo: create template material profile with name from commandline
    print "# Avg: P0temp: %.2f, P0pwm: %.2f, grip: %.2f, FR0pwm: %.2f mm³/s" % (t1Avg, pAvg, gAvg, frAvg)
    print ""
    print "# Material properties, printing:"
    print "# a0 for pwm"
    print '"P0pwmPrint": %.4f,' % pAvg
    print "# a0 for temp"
    print '"P0tempPrint": %.4f,' % t1Avg
    print "# feedrate at a0"
    print '"FR0pwmPrint": %.4f,\n' % frAvg

    # Re-enable flowrate limit
    printer.sendCommandParamV(CmdEnableFRLimit, [packedvalue.uint8_t(1)])
    printer.sendCommand(CmdDisableSteppers)
    printer.coolDown(HeaterEx1, wait=100, log=True)
    printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(0)])

####################################################################################################




