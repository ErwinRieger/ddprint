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

import struct, time, math, tty, termios, sys, types
import ddprintconstants

from ddprintcommands import *
from ddprintstates import *
from ddprintconstants import dimNames
from ddprofile import PrinterProfile, MatProfile

debugMoves = True
debugMoves = False

####################################################################################################
#
# Constants, some are printer specific.
#
X_AXIS = 0
Y_AXIS = 1
Z_AXIS = 2
A_AXIS = 3
B_AXIS = 4


# UM2:
FILAMENT_REVERSAL_LENGTH = 750

# To prevent false assertions because of rounding errors
RoundSafe = 0.999

####################################################################################################
#
# Generic Constants
#
dimIndex = { "X": 0, "Y": 1, "Z": 2, "A": 3, "B": 4 }


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
def vectorDistance(a, b):
    return vectorLength(vectorSub(a, b))

####################################################################################################
def circaf(a, b, delta):
    return abs(a-b) < delta

# Speed after accelerating for a certain time
def vAccelPerTime(v0, a, t):
    # v = a * t + v0
    return a * t + v0

####################################################################################################

# Distance traveled after accelerating for a certain time
def accelDist(v0, a, t):
    # s = 0,5 * a * t² + vo * t
    return 0.5 * a * pow(t, 2) + v0 * t

####################################################################################################

# Speed after acceleration along a certain distance
def vAccelPerDist(v0, a, s):

    assert(v0 >= 0)

    term = 2 * a * s + pow(v0, 2)
    if term >= 0:
        return math.sqrt( term )
    else:
        return - math.sqrt( abs(term) )

####################################################################################################

def joinSpeed(move1, move2, jerk, min_speeds):

        if debugMoves:
            print "***** Start joinSpeed() *****"

        allowedAccel = move1.getAllowedAccel()

        # if move1.vVector().isDisjointV(move2.vVector(), 0.01):
        if move1.getFeedrateV().isDisjointV(move2.getFeedrateV()):
        # if move1.isDisjointSteps(move2, 5):
            if debugMoves:
                move1.pprint("JoinSpeed - disjoint Move1")
                move2.pprint("JoinSpeed - disjoint Move2")

            # jerkSpeed = move1.vVector().constrain(jerk) or move1.feedrate
            # move1.setNominalEndFr(jerkSpeed)
            move1.setNominalJerkEndSpeed(jerk)

            # Set endspeed to minimum of reachable endspeed and jerkspeed
            f = move1.getJerkSpeed(jerk)
            maxEndSpeed = vAccelPerDist(move1.getStartFr(), allowedAccel, move1.distance) * RoundSafe
            # print "maxEndSpeed of first disjoint move:", maxEndSpeed, f
            move1.setNominalEndFr(min(f, maxEndSpeed))

            # jerkSpeed = move2.vVector().constrain(jerk) or move2.feedrate
            # move2.setNominalStartFr(jerkSpeed)
            move2.setNominalJerkStartSpeed(jerk)

            if debugMoves:
                print "***** End joinSpeed() *****"
            return

        endSpeedS = move1.feedrateS
        endSpeedV = move1.getFeedrateV()

        # Compute max reachable endspeed of move1
        # maxEndSpeed = vAccelPerDist(move1.getStartFr(), allowedAccel, move1.e_distance)
        maxEndSpeed = vAccelPerDist(move1.getStartFr(), allowedAccel, move1.distance) * RoundSafe

        if maxEndSpeed < endSpeedS:

            if debugMoves:
                print "Max. reachable endspeed: %.3f < feedrate: %.3f" % (maxEndSpeed, endSpeedS)

            endSpeedS = maxEndSpeed

            endSpeedV = move1.getFeedrateV(maxEndSpeed)

        nomFeedrateVMove2 = move2.getFeedrateV()
        differenceVector = nomFeedrateVMove2.subVVector(endSpeedV)

        #
        # Join in bezug auf den maximalen jerk aller achsen betrachten:
        #
        speedDiff = {}
        toMuch = False
        for dim in range(5):

            vdim = nomFeedrateVMove2[dim]

            # Speed difference from start speed to theoretical max speed for axis
            vdiff = vdim - endSpeedV[dim]

            vdiffAbs = abs(vdiff)

            dimJerk = jerk[dim]

            if vdiffAbs > dimJerk: 
                toMuch = True

            if vdiffAbs > 1:
                speedDiff[dim] = (vdiff, vdiffAbs)

        if debugMoves:
            print "speedDiff:", speedDiff

        if toMuch:

            # angle = endSpeedV.angleBetween(move2.vVector())
            # assert(angle >= 0)
            # if angle < 0.01:
                # # Vektoren sind parallel:
                # assert(0)

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

            # weight = 1.0; # move1.feedrate / move2.feedrate

            speedScale = 1.0
            for dim in speedDiff.keys():
                # print "diff: ", differenceVector[dim], jerk[dim]
                if abs(differenceVector[dim]) > jerk[dim]:
                    # print "mindiff: ", dimNames[dim], differenceVector[dim], jerk[dim]
                    speedScale = min(speedScale, jerk[dim] / abs(differenceVector[dim]))

            if debugMoves:
                move1.pprint("JoinSpeed - Move1")
                move2.pprint("JoinSpeed - Move2")
                print "speedScale: ", speedScale # , weight

            assert(speedScale <= 1.0)

            """
            endV = move1.vVector().setLength(endSpeedS * speedScale)
            startV = move2.vVector().scale(speedScale)

            if abs(endV[move1.longest_axis]) < min_speeds[move1.longest_axis]:

                # Move 1 am schluss zu langsam, move 1 am ende schneller machen, move 2 entsprechend
                # langsamer starten lassen
                if debugMoves:
                    print "end speed to low: ", endV, ", min:", min_speeds
                    print "startv          : ", startV

                upScale = min_speeds[move1.longest_axis] / abs(endV[move1.longest_axis])
                downScale = abs(startV[move2.longest_axis]) / min_speeds[move2.longest_axis]

                print "upscale, downscale:", upScale, downScale

                assert (downScale >= upScale)

                move1.setNominalEndFr(endSpeedS * speedScale * upScale)
                move2.setNominalStartFr(move2.feedrate * speedScale / upScale)

                if debugMoves:
                    print "end speed to low: ", endV, ", min:", min_speeds
                    print "startv          : ", startV

            else:
                print "set endspeed:", endSpeedS * speedScale
                move1.setNominalEndFr(endSpeedS * speedScale)
            """

            if debugMoves:
                print "set nominal endspeed:", endSpeedS * speedScale
            move1.setNominalEndFr(endSpeedS * speedScale)

            """
            if abs(startV[move2.longest_axis]) < min_speeds[move2.longest_axis]:

                # Move 2 am anfang zu langsam
                if debugMoves:
                    print "start speed to low: ", startV, ", min:", min_speeds
                    print "endv              : ", endV

                upScale = min_speeds[move2.longest_axis] / abs(startV[move2.longest_axis])
                downScale = abs(endV[move1.longest_axis]) / min_speeds[move1.longest_axis]

                print "upscale, downscale:", upScale, downScale

                assert (downScale >= upScale)

                move1.setNominalEndFr(endSpeedS * speedScale / upScale)
                move2.setNominalStartFr(move2.feedrate * speedScale * upScale)

                if debugMoves:
                    print "start speed to low: ", move2.getStartV(), ", min:", min_speeds
                    print "endv              : ", move1.getEndV()

            else:
                print "set start:", move2.feedrate * speedScale
                move2.setNominalStartFr(move2.feedrate * speedScale)
            """
            
            if debugMoves:
                print "set nominal startspeed:", move2.feedrateS * speedScale
            move2.setNominalStartFr(move2.feedrateS * speedScale)

        else:

            if debugMoves:
                print "Doing a full speed join between move %d and %d" % (move1.moveNumber, move2.moveNumber)

            move1.setNominalEndFr(endSpeedS)
            move2.setNominalStartFr(move2.feedrateS)
      
        if debugMoves:
            print "***** End joinSpeed() *****"

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
        self.old = termios.tcgetattr(self.fd)

    def getc(self):

        print self.msg
        tty.setcbreak(self.fd)
        ch = sys.stdin.read(1)
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        return ch

####################################################################################################

def commonInit(args, parser):

    driver = parser.planner
    printer = driver.printer

    printer.commandInit(args)

    home(parser, args.fakeendstop)

    printer.sendPrinterInit()

####################################################################################################

def isHomed(printer):
    res = printer.query(CmdGetHomed)
    return res == (1, 1, 1)

####################################################################################################

def getVirtualPos(parser):

    planner = parser.planner
    printer = planner.printer

    # Get currend stepped pos
    res = printer.query(CmdGetPos)
    print "endstop state:", res

    curPosMM = MyPoint(
        X = res[0] / float(parser.steps_per_mm[0]),
        Y = res[1] / float(parser.steps_per_mm[1]),
        Z = res[2] / float(parser.steps_per_mm[2]),
        A = res[3] / float(parser.steps_per_mm[3]),
        # B = res[4] / float(parser.steps_per_mm[4]),
        )

    print "Printer is at: ", curPosMM
    parser.set_position(curPosMM)

    return curPosMM

####################################################################################################

def home(parser, fakeHomingEndstops=False, force=False):

    driver = parser.planner
    printer = driver.printer

    print "*"
    print "* Start homing..."
    print "*"
    if isHomed(printer) and not force:
        print "Printer is homed already..."

        # Send init command
        printer.sendPrinterInit()

        # Get current pos from printer and set our virtual pos
        curPosMM = getVirtualPos(parser)

        (homePosMM, homePosStepped) = driver.getHomePos()

        # Move to home
        if not curPosMM.equal(homePosMM, "XYZ"):

            #
            # Z Achse isoliert und als erstes bewegen, um zusammenstoss mit den klammern
            # der buildplatte oder mit objekten auf der druckplatte zu vermeiden.
            #
            if not curPosMM.equal(homePosMM, "Z"):

                parser.execute_line("G0 F%d Z%f" % (
                    PrinterProfile.getMaxFeedrate(Z_AXIS)*60, homePosMM.Z))

            if not curPosMM.equal(homePosMM, "XY"):

                parser.execute_line("G0 F%d X%f Y%f" % (
                    PrinterProfile.getMaxFeedrate(X_AXIS)*60, 
                    homePosMM.X, homePosMM.Y))

            parser.finishMoves()
            printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
            printer.sendCommand(CmdEOT, wantReply="ok")

            printer.waitForState(StateInit, wait=0.1)

            # res = printer.query(CmdGetEndstops)
            # print "endstop state:", res

        #
        # Set Virtual E-pos 0:
        #
        parser.set_position(homePosMM)

        print "*"
        print "* Done homing..."
        print "*"
        return

    #
    # Z Achse isoliert und als erstes bewegen, um zusammenstoss mit den klammern
    # der buildplatte zu vermeiden.
    #
    for dim in [Z_AXIS, X_AXIS, Y_AXIS]:

        printer.sendPrinterInit()

        # Send homing moves
        # xxx create Moves directly
        parser.set_position(driver.zeroPos)
        parser.execute_line("G0 F%d %s%f" % (driver.HOMING_FEEDRATE[dim]*60, dimNames[dim], driver.MAX_POS[dim] * driver.HOME_DIR[dim] * 1.25)) # Move towards endstop
        parser.finishMoves()

        # Send homing command
        print "---------------- send homing cmd 1 on axis", dimNames[dim]
        printer.sendCommandParam(CmdMove, p1=MoveTypeHoming, wantReply="ok")
        printer.sendCommand(CmdEOT, wantReply="ok")

        if fakeHomingEndstops:
            time.sleep(0.1)
            printer.sendCommand(CmdDisableStepperIsr, wantReply="ok")

        printer.waitForState(StateInit, wait=0.1)

        # Check, if enstop was pressed
        res = printer.query(CmdGetEndstops)
        print "endstop state:", res

        if res[dim][0] or fakeHomingEndstops:
            print "Endstop %d hit at position: %d - good" % (dim, res[dim][1])
        else:
            print "Error, Endstop %d NOT hit!" % dim
            assert(0)

        printer.sendPrinterInit()

        parser.set_position(driver.zeroPos)
        parser.execute_line("G0 F%d %s%f" % (driver.HOMING_FEEDRATE[dim]*60, dimNames[dim], driver.HOME_RETRACT_MM * -1 * driver.HOME_DIR[dim])) # Back off
        parser.finishMoves()

        # Send homing command
        print "---------------- send homing cmd 2"
        printer.sendCommandParam(CmdMove, p1=MoveTypeHoming, wantReply="ok")
        printer.sendCommand(CmdEOT, wantReply="ok")

        printer.waitForState(StateInit, wait=0.1)

        # Check, if enstop was opened
        res = printer.query(CmdGetEndstops)
        print "endstop state:", res

        if not res[dim][0]:
            print "Endstop %d opened at position: %d - good" % (dim, res[dim][1])
        else:
            print "Error, Endstop %d NOT oken!" % dim
            assert(0)

        printer.sendPrinterInit()

        parser.set_position(driver.zeroPos)
        parser.execute_line("G0 F%d %s%f" % (driver.HOMING_FEEDRATE[dim]*60/3, dimNames[dim], driver.HOME_RETRACT_MM*1.5*driver.HOME_DIR[dim])) # Move towards slowly
        parser.finishMoves()

        # Send homing command
        print "---------------- send homing cmd 3"
        printer.sendCommandParam(CmdMove, p1=MoveTypeHoming, wantReply="ok")
        printer.sendCommand(CmdEOT, wantReply="ok")

        if fakeHomingEndstops:
            # time.sleep(0.1)
            printer.sendCommand(CmdDisableStepperIsr, wantReply="ok")

        printer.waitForState(StateInit, wait=0.1)

        # Check, if enstop was pressed
        res = printer.query(CmdGetEndstops)
        print "endstop state:", res

        if res[dim][0] or fakeHomingEndstops:
            print "Endstop %d hit at position: %d - good" % (dim, res[dim][1])
        else:
            print "Error, Endstop %d NOT hit!" % dim
            assert(0)

        printer.sendPrinterInit()

        parser.set_position(driver.zeroPos)
        parser.execute_line("G0 F%d %s%f" % (driver.HOMING_FEEDRATE[dim]*60/3, dimNames[dim], driver.HOME_RETRACT_MM * -1 * driver.HOME_DIR[dim])) # Back off
        parser.finishMoves()

        # Send homing command
        print "---------------- send homing cmd 4"
        printer.sendCommandParam(CmdMove, p1=MoveTypeHoming, wantReply="ok")
        printer.sendCommand(CmdEOT, wantReply="ok")

        printer.waitForState(StateInit, wait=0.1)

        # Check, if enstop was opened
        res = printer.query(CmdGetEndstops)
        print "endstop state:", res

        if not res[dim][0]:
            print "Endstop %d opened at position: %d - good" % (dim, res[dim][1])
        else:
            print "Error, Endstop %d NOT oken!" % dim
            assert(0)


    #
    # Set position in steps on the printer side and set our 'virtual position':
    #
    # xxx set end-of-print retraction e-pos also here?
    #
    # parser.set_position(driver.homePosMM)
    # payload = struct.pack("<iiiii", *driver.homePosStepped)
    (homePosMM, homePosStepped) = driver.getHomePos()
    parser.set_position(homePosMM)
    payload = struct.pack("<iiiii", *homePosStepped)
    printer.sendCommand(CmdSetHomePos, binPayload=payload, wantReply="ok")

    print "*"
    print "* Done homing..."
    # res = printer.query(CmdGetEndstops)
    # print "endstop state:", res
    print "*"

####################################################################################################

def prime(parser):

    driver = parser.planner
    # printer = driver.printer

    parser.execute_line("G0 F%f Y0 Z%f" % (driver.HOMING_FEEDRATE[X_AXIS]*60, ddprintconstants.PRIMING_HEIGHT))

    pos = parser.getRealPos()

    aFilament = (math.pi * pow(MatProfile.getMatDiameter(), 2)) / 4.0

    parser.execute_line("G0 F%f A%f" % (
        ((ddprintconstants.PRIMING_MM3_PER_SEC / aFilament) * 60),
        pos[A_AXIS] + (ddprintconstants.PRIMING_MM3 / aFilament)))

    # Set E to 0
    current_position = parser.getRealPos()
    current_position[A_AXIS] = 0.0
    parser.set_position(current_position)

    #
    # Retract
    #
    # parser.execute_line("G10")

    # Wipe priming material if not ultigcode flavor 
    if not parser.ultiGcodeFlavor:
        # parser.execute_line("G10")
        # parser.execute_line("G0 F%f X50 Z0.5" % (driver.HOMING_FEEDRATE[X_AXIS]*60))
        parser.execute_line("G0 F9000 X20 Z0.1")
        # parser.execute_line("G0 F1200 X%.2f" % (driver.X_MAX_POS * 0.8))
        # parser.execute_line("G0 F%d Z10" % (driver.HOMING_FEEDRATE[Z_AXIS]*60))
        # parser.execute_line("G1 X190 Z0.1 F9000") #  ; pull away filament
        # parser.execute_line("G1 X210 F9000") #  ; wipe
        # parser.execute_line("G1 Y20 F9000") #  ; wipe
        # parser.execute_line("G11")

####################################################################################################

def zRepeatability(parser):

    import random

    printer.commandInit(args)

    feedrate = PrinterProfile.getMaxFeedrate(Z_AXIS)

    home(parser, args.fakeendstop)

    printer.sendPrinterInit()

    for i in range(10):

        parser.execute_line("G0 F%d X115 Y210 Z10" % (feedrate*60))
        parser.execute_line("G0 F%d Z%f" % (feedrate*60, random.randint(20, 150)))

    parser.finishMoves()
    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.sendCommand(CmdEOT, wantReply="ok")

    printer.waitForState(StateInit)

####################################################################################################

def manualMove(parser, axis, distance, absolute=False):

    driver = parser.planner
    printer = driver.printer

    printer.sendPrinterInit()

    assert(isHomed(printer))

    assert(abs(distance) <= 1000)

    # Get current pos from printer and set our virtual pos
    getVirtualPos(parser)

    feedrate = PrinterProfile.getMaxFeedrate(axis)

    # parser.execute_line("G0 F%d E%f" % (feedrate*60, distance / parser.e_to_filament_length))
    if absolute:
        parser.execute_line("G0 F%d %s%f" % (feedrate*60, dimNames[axis], distance))
    else:
        current_position = parser.getRealPos()
        parser.execute_line("G0 F%d %s%f" % (feedrate*60, dimNames[axis], current_position[axis] + distance))

    parser.finishMoves()

    printer.sendCommand(CmdEOT, wantReply="ok")
    # time.sleep(1)

    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")

    printer.waitForState(StateInit)

    printer.readMore(10)

####################################################################################################

def insertFilament(args, parser):

    driver = parser.planner
    printer = driver.printer

    def manualMoveE():

        current_position = parser.getRealPos()
        aofs = current_position[A_AXIS]
        print "cura: ", aofs

        kbd = GetChar("Enter (f)orward (b)ackwards (F)orward 10mm (B)ackwards 10mm (q)uit")

        ch = " "
        while ch not in "q\n":
            ch = kbd.getc()

            print "ch: ", ch
            if ch == "f":       # filament backwards, 'small' step
                aofs += 1
            elif ch == "F":     # filament backwards, 'big' step
                aofs += 10
            elif ch == "b":     # filament backwards, 'small' step
                aofs -= 1
            elif ch == "B":     # filament backwards, 'big' step
                aofs -= 10

            printer.sendPrinterInit()

            # XXX hardcoded feedrate
            parser.execute_line("G0 F%d A%f" % (5*60, aofs))

            parser.finishMoves()
            printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
            printer.sendCommand(CmdEOT, wantReply="ok")
            printer.waitForState(StateInit, wait=0.1)

    commonInit(args, parser)

    # Move to mid-front
    feedrate = PrinterProfile.getMaxFeedrate(X_AXIS)
    parser.execute_line("G0 F%d X%f Y%f" % (feedrate*60, driver.MAX_POS[X_AXIS]/2, driver.MAX_POS[Y_AXIS]/2))

    parser.finishMoves()

    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.sendCommand(CmdEOT, wantReply="ok")

    printer.waitForState(StateInit)

    t1 = MatProfile.getHotendBaseTemp()
    printer.heatUp(HeaterEx1, t1, wait=t1 - 5)

    print "\nInsert filament.\n"
    manualMoveE()

    print "\nForwarding filament.\n"
    manualMove(parser, A_AXIS, FILAMENT_REVERSAL_LENGTH * 0.8)

    print "\nExtrude filament.\n"
    manualMoveE()

    #
    # Retract
    #
    parser.execute_line("G10")

    parser.finishMoves()

    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.sendCommand(CmdEOT, wantReply="ok")

    printer.waitForState(StateInit)

    printer.coolDown(HeaterEx1, wait=150)

####################################################################################################

def removeFilament(args, parser):

    driver = parser.planner
    printer = driver.printer

    printer.commandInit(args)

    home(parser, args.fakeendstop)

    printer.sendPrinterInit()

    # Move to mid-front
    # MAX_POS = (X_MAX_POS, Y_MAX_POS, Z_MAX_POS)
    # feedrate = PrinterProfile.getMaxFeedrate(Z_AXIS)
    # parser.execute_line("G0 F%d Z%f" % (feedrate*60, MAX_POS[Z_AXIS]))

    feedrate = PrinterProfile.getMaxFeedrate(X_AXIS)
    parser.execute_line("G0 F%d X%f Y%f" % (feedrate*60, driver.MAX_POS[X_AXIS]/2, driver.MAX_POS[Y_AXIS]/2))

    parser.finishMoves()

    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.sendCommand(CmdEOT, wantReply="ok")

    printer.waitForState(StateInit)

    t1 = MatProfile.getHotendBaseTemp()
    printer.heatUp(HeaterEx1, t1, wait=t1 - 5)

    manualMove(parser, A_AXIS, -1.3*FILAMENT_REVERSAL_LENGTH)

    printer.coolDown(HeaterEx1,wait=150)

####################################################################################################

def retract(args, parser):

    driver = parser.planner
    printer = driver.printer

    commonInit(args, parser)

    t1 = MatProfile.getHotendBaseTemp()
    printer.heatUp(HeaterEx1, t1, wait=t1 - 5)

    # hack
    # parser.retracted = False
    parser.execute_line("G10")

    parser.finishMoves()

    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.sendCommand(CmdEOT, wantReply="ok")

    printer.waitForState(StateInit)

    printer.coolDown(HeaterEx1,wait=150)

####################################################################################################

def bedLeveling(args, parser):

    driver = parser.planner
    printer = driver.printer

    printer.commandInit(args)

    # Reset bedlevel offset in printer eeprom
    payload = struct.pack("<pf", "add_homeing_z", 0)
    printer.sendCommand(CmdWriteEepromFloat, binPayload=payload, wantReply="ok")

    home(parser, args.fakeendstop, True)

    zFeedrate = PrinterProfile.getMaxFeedrate(Z_AXIS)
    kbd = GetChar("Enter (u)p (d)own (U)p 1mm (D)own 1mm (2-5) Up Xmm (q)uit")

    def manualMoveZ():

        current_position = parser.getRealPos()
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

            parser.finishMoves()
            printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
            printer.sendCommand(CmdEOT, wantReply="ok")

            printer.waitForState(StateInit, wait=0.1)


    feedrate = PrinterProfile.getMaxFeedrate(X_AXIS)

    #######################################################################################################
    print "Level point 1/3"

    printer.sendPrinterInit()
    parser.execute_line("G0 F%d X%f Y%f Z%f" % (feedrate*60, driver.X_MAX_POS/2, driver.Y_MAX_POS - 10, driver.HEAD_HEIGHT))

    parser.finishMoves()
    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.sendCommand(CmdEOT, wantReply="ok")

    printer.waitForState(StateInit, wait=0.1)

    manualMoveZ()

    current_position = parser.getRealPos()
    print "curz: ", current_position[Z_AXIS]

    add_homeing_z = -1 * current_position[Z_AXIS];
    print "\nZ-Offset: ", add_homeing_z, "\n"

    # Store into printer eeprom:
    payload = struct.pack("<pf", "add_homeing_z", add_homeing_z)
    printer.sendCommand(CmdWriteEepromFloat, binPayload=payload, wantReply="ok")

    # Finally we know the zero z position
    # current_position[Z_AXIS] = 0
    current_position[Z_AXIS] = driver.LEVELING_OFFSET;

    # Adjust the virtual position
    parser.set_position(current_position)

    # Adjust the printer position
    posStepped = vectorMul(current_position, parser.steps_per_mm)
    payload = struct.pack("<iiiii", *posStepped)
    printer.sendCommand(CmdSetHomePos, binPayload=payload, wantReply="ok")

    #######################################################################################################
    print "Level point 2/3", current_position

    printer.sendPrinterInit()
    parser.execute_line("G0 F%d Z5" % (zFeedrate*60))
    parser.execute_line("G0 F%d X35 Y20" % (feedrate*60))
    parser.execute_line("G0 F%d Z0.1" % (zFeedrate*60))

    parser.finishMoves()
    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.sendCommand(CmdEOT, wantReply="ok")

    printer.waitForState(StateInit, wait=0.1)

    raw_input("\nAdjust left front buildplate screw and press <Return>\n")

    #######################################################################################################
    print "Level point 3/3", current_position

    printer.sendPrinterInit()
    parser.execute_line("G0 F%d Z5" % (zFeedrate*60))
    parser.execute_line("G0 F%d X%f" % (feedrate*60, driver.X_MAX_POS-10))
    parser.execute_line("G0 F%d Z0.1" % (zFeedrate*60))

    parser.finishMoves()
    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.sendCommand(CmdEOT, wantReply="ok")

    printer.waitForState(StateInit, wait=0.1)

    raw_input("\nAdjust right fron buildplate screw and press <Return>\n")

    home(parser, args.fakeendstop)

####################################################################################################

def bedLevelAdjust(args, parser):

    planner = parser.planner
    printer = planner.printer

    printer.commandInit(args)

    distance = float(args.distance)

    add_homeing_z = printer.query(CmdGetEepromSettings)['add_homeing'][Z_AXIS] + distance

    # Store new bedlevel offset in printer eeprom
    payload = struct.pack("<pf", "add_homeing_z", add_homeing_z)
    printer.sendCommand(CmdWriteEepromFloat, binPayload=payload, wantReply="ok")

####################################################################################################

def storeSD(parser):

    driver = parser.planner
    printer = driver.printer

    buf = (512 - 5) * chr(0x55)
    printer.sendBinaryCommand(chr(CmdRaw), binPayload=buf)

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










