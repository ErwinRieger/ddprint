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
import ddprintconstants, ddhome

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
RoundSafe = 0.995

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

        if move1.getFeedrateV().isDisjointV(move2.getFeedrateV()):

            if debugMoves:
                move1.pprint("JoinSpeed - disjoint Move1")
                move2.pprint("JoinSpeed - disjoint Move2")

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
                print "math.sqrt( 2 * %f * %f + pow(%f, 2) ) * %f" % (move1.distance, allowedAccel, move1.getStartFr(), RoundSafe)
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
                move1.pprint("JoinSpeed - Move1")
                move2.pprint("JoinSpeed - Move2")
                print "speedScale: ", speedScale # , weight

            assert(speedScale <= 1.0)

            if debugMoves:
                print "set nominal endspeed of move1:", endSpeedS * speedScale
            move1.setNominalEndFr(endSpeedS * speedScale)

            if debugMoves:
                print "set nominal startspeed of move2:", move2.feedrateS * speedScale
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

    planner = parser.planner
    printer = planner.printer

    printer.commandInit(args)

    ddhome.home(parser, args.fakeendstop)

    printer.sendPrinterInit()

####################################################################################################

def getVirtualPos(parser):

    planner = parser.planner
    printer = planner.printer

    # Get currend stepped pos
    res = printer.query(CmdGetPos)
    print "Printer home pos:", res

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

def prime(parser):

    planner = parser.planner
    # printer = planner.printer

    parser.execute_line("G0 F%f Y0 Z%f" % (planner.HOMING_FEEDRATE[X_AXIS]*60, ddprintconstants.PRIMING_HEIGHT))

    pos = parser.getRealPos()

    aFilament = MatProfile.getMatArea()

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
        # parser.execute_line("G0 F%f X50 Z0.5" % (planner.HOMING_FEEDRATE[X_AXIS]*60))
        parser.execute_line("G0 F9000 X20 Z0.1")
        # parser.execute_line("G0 F1200 X%.2f" % (planner.X_MAX_POS * 0.8))
        # parser.execute_line("G0 F%d Z10" % (planner.HOMING_FEEDRATE[Z_AXIS]*60))
        # parser.execute_line("G1 X190 Z0.1 F9000") #  ; pull away filament
        # parser.execute_line("G1 X210 F9000") #  ; wipe
        # parser.execute_line("G1 Y20 F9000") #  ; wipe
        # parser.execute_line("G11")

####################################################################################################

def zRepeatability(parser):

    import random

    printer.commandInit(args)

    feedrate = PrinterProfile.getMaxFeedrate(Z_AXIS)

    ddhome.home(parser, args.fakeendstop)

    printer.sendPrinterInit()

    for i in range(10):

        parser.execute_line("G0 F%d X115 Y210 Z10" % (feedrate*60))
        parser.execute_line("G0 F%d Z%f" % (feedrate*60, random.randint(20, 150)))

    planner.finishMoves()
    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.sendCommand(CmdEOT, wantReply="ok")

    printer.waitForState(StateIdle)

####################################################################################################

def manualMove(parser, axis, distance, feedrate, absolute=False):

    planner = parser.planner
    printer = planner.printer

    printer.sendPrinterInit()

    assert(printer.isHomed())

    assert(abs(distance) <= 1000)

    # Get current pos from printer and set our virtual pos
    getVirtualPos(parser)

    if not feedrate:
        feedrate = PrinterProfile.getMaxFeedrate(axis)

    if absolute:
        parser.execute_line("G0 F%d %s%f" % (feedrate*60, dimNames[axis], distance))
    else:
        current_position = parser.getRealPos()
        parser.execute_line("G0 F%d %s%f" % (feedrate*60, dimNames[axis], current_position[axis] + distance))

    planner.finishMoves()

    printer.sendCommand(CmdEOT, wantReply="ok")
    # time.sleep(1)

    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")

    printer.waitForState(StateIdle)

    printer.readMore(10)

####################################################################################################

def insertFilament(args, parser):

    planner = parser.planner
    printer = planner.printer

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

            planner.finishMoves()
            printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
            printer.sendCommand(CmdEOT, wantReply="ok")
            printer.waitForState(StateIdle, wait=0.1)

    commonInit(args, parser)

    # Move to mid-front
    feedrate = PrinterProfile.getMaxFeedrate(X_AXIS)
    parser.execute_line("G0 F%d X%f Y%f" % (feedrate*60, planner.MAX_POS[X_AXIS]/2, planner.MAX_POS[Y_AXIS]/2))

    planner.finishMoves()

    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.sendCommand(CmdEOT, wantReply="ok")

    printer.waitForState(StateIdle)

    t1 = MatProfile.getHotendBaseTemp()
    printer.heatUp(HeaterEx1, t1, wait=t1 - 5)

    print "\nInsert filament.\n"
    manualMoveE()

    print "\nForwarding filament.\n"
    manualMove(parser, A_AXIS, FILAMENT_REVERSAL_LENGTH * 0.85)

    print "\nExtrude filament.\n"
    manualMoveE()

    #
    # Retract
    #
    printer.sendPrinterInit()
    parser.execute_line("G10")
    planner.finishMoves()

    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.sendCommand(CmdEOT, wantReply="ok")

    printer.waitForState(StateIdle)

    printer.coolDown(HeaterEx1, wait=150)

####################################################################################################

def removeFilament(args, parser):

    planner = parser.planner
    printer = planner.printer

    printer.commandInit(args)

    ddhome.home(parser, args.fakeendstop)

    printer.sendPrinterInit()

    # Move to mid-front
    # MAX_POS = (X_MAX_POS, Y_MAX_POS, Z_MAX_POS)
    # feedrate = PrinterProfile.getMaxFeedrate(Z_AXIS)
    # parser.execute_line("G0 F%d Z%f" % (feedrate*60, MAX_POS[Z_AXIS]))

    feedrate = PrinterProfile.getMaxFeedrate(X_AXIS)
    parser.execute_line("G0 F%d X%f Y%f" % (feedrate*60, planner.MAX_POS[X_AXIS]/2, planner.MAX_POS[Y_AXIS]/2))

    planner.finishMoves()

    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.sendCommand(CmdEOT, wantReply="ok")

    printer.waitForState(StateIdle)

    t1 = MatProfile.getHotendBaseTemp()
    printer.heatUp(HeaterEx1, t1, wait=t1 - 5)

    manualMove(parser, A_AXIS, -1.3*FILAMENT_REVERSAL_LENGTH)

    printer.coolDown(HeaterEx1,wait=150)

####################################################################################################

def retract(args, parser, doCooldown = True):

    planner = parser.planner
    printer = planner.printer

    commonInit(args, parser)

    t1 = MatProfile.getHotendBaseTemp()
    printer.heatUp(HeaterEx1, t1, wait=t1 - 5)

    # hack
    # parser.retracted = False
    parser.execute_line("G10")

    planner.finishMoves()

    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.sendCommand(CmdEOT, wantReply="ok")

    printer.waitForState(StateIdle)

    if doCooldown:
        printer.coolDown(HeaterEx1,wait=150)

####################################################################################################

def bedLeveling(args, parser):

    planner = parser.planner
    printer = planner.printer

    printer.commandInit(args)

    # Reset bedlevel offset in printer eeprom
    payload = struct.pack("<%dpf" % (len("add_homeing_z")+1), "add_homeing_z", 0)
    printer.sendCommand(CmdWriteEepromFloat, binPayload=payload, wantReply="ok")

    ddhome.home(parser, args.fakeendstop, True)

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

            planner.finishMoves()
            printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
            printer.sendCommand(CmdEOT, wantReply="ok")

            printer.waitForState(StateIdle, wait=0.1)


    feedrate = PrinterProfile.getMaxFeedrate(X_AXIS)

    #######################################################################################################
    print "Level point 1/3"

    printer.sendPrinterInit()
    parser.execute_line("G0 F%d X%f Y%f Z%f" % (feedrate*60, planner.X_MAX_POS/2, planner.Y_MAX_POS - 10, planner.HEAD_HEIGHT))

    planner.finishMoves()
    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.sendCommand(CmdEOT, wantReply="ok")

    printer.waitForState(StateIdle, wait=0.1)

    manualMoveZ()

    current_position = parser.getRealPos()
    print "curz: ", current_position[Z_AXIS]

    add_homeing_z = -1 * current_position[Z_AXIS];
    print "\nZ-Offset: ", add_homeing_z, "\n"

    # Store into printer eeprom:
    payload = struct.pack("<%dpf" % (len("add_homeing_z")+1), "add_homeing_z", add_homeing_z)
    printer.sendCommand(CmdWriteEepromFloat, binPayload=payload, wantReply="ok")

    # Finally we know the zero z position
    # current_position[Z_AXIS] = 0
    current_position[Z_AXIS] = planner.LEVELING_OFFSET;

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

    planner.finishMoves()
    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.sendCommand(CmdEOT, wantReply="ok")

    printer.waitForState(StateIdle, wait=0.1)

    raw_input("\nAdjust left front buildplate screw and press <Return>\n")

    #######################################################################################################
    print "Level point 3/3", current_position

    printer.sendPrinterInit()
    parser.execute_line("G0 F%d Z5" % (zFeedrate*60))
    parser.execute_line("G0 F%d X%f" % (feedrate*60, planner.X_MAX_POS-10))
    parser.execute_line("G0 F%d Z0.1" % (zFeedrate*60))

    planner.finishMoves()
    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.sendCommand(CmdEOT, wantReply="ok")

    printer.waitForState(StateIdle, wait=0.1)

    raw_input("\nAdjust right fron buildplate screw and press <Return>\n")

    ddhome.home(parser, args.fakeendstop)

####################################################################################################

def bedLevelAdjust(args, parser):

    planner = parser.planner
    printer = planner.printer

    printer.commandInit(args)


    add_homeing_z = printer.query(CmdGetEepromSettings)['add_homeing'][Z_AXIS] + args.distance

    # Store new bedlevel offset in printer eeprom
    payload = struct.pack("<%dpf" % (len("add_homeing_z")+1), "add_homeing_z", add_homeing_z)
    printer.sendCommand(CmdWriteEepromFloat, binPayload=payload, wantReply="ok")

####################################################################################################

def writeEEpromFloat(args, parser):

    planner = parser.planner
    printer = planner.printer

    printer.commandInit(args)

    payload = struct.pack("<%dpf" % (len(args.name)+1), args.name, args.value)
    printer.sendCommand(CmdWriteEepromFloat, binPayload=payload, wantReply="ok")

####################################################################################################

def storeSD(parser):

    planner = parser.planner
    printer = planner.printer

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

def endOfPrintLift(parser):

    planner = parser.planner

    pos = parser.getRealPos()
    zlift = min(pos.Z + 25, planner.Z_MAX_POS)

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

    printer.sendCommand(CmdStopMove, wantReply="ok")
    printer.sendCommand(CmdDisableSteppers, wantReply="ok")

####################################################################################################

def heatHotend(args, parser):

    planner = parser.planner
    printer = planner.printer

    printer.commandInit(args)

    t1 = MatProfile.getHotendBaseTemp()

    printer.heatUp(HeaterEx1, t1, wait=t1-5)

    raw_input("Press return to stop heating...")

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

    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.sendCommand(CmdEOT, wantReply="ok")

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
    printer.coolDown(HeaterEx1, wait=100)

####################################################################################################
#
# Measure closed loop step-response of the hotend and plot it with gnuplot
#
def stepResponse(args, parser):

    planner = parser.planner
    printer = planner.printer

    def stopHeater():
        payload = struct.pack("<BH", HeaterEx1, 0) # Parameters: heater, temp
        printer.sendCommand(CmdSetTargetTemp, binPayload=payload, wantReply="ok")
        printer.sendCommandParam(CmdFanSpeed, p1=packedvalue.uint8_t(0), wantReply="ok")

    # Destination temperature
    tDest = 200
    # Bail out if tmax reached
    tmax = 240

    # Open output gnuplot file
    f = open("stepresponse_closed.gnuplot", "w")

    f.write("set grid\n")
    f.write("set xrange [0:140]\n")
    f.write("set yrange [0:250]\n")
    f.write("set grid\n")
    f.write("plot \"-\" using 1:2 with linespoints title \"StepResponse\",")
    f.write("%d title \"tDest\"\n" % tDest)

    printer.commandInit(args)

    # Do the step
    print "Starting input step to %d °" % tDest
    payload = struct.pack("<BH", HeaterEx1, tDest) # Parameters: heater, temp
    printer.sendCommand(CmdSetTargetTemp, binPayload=payload, wantReply="ok")
    printer.sendCommandParam(CmdFanSpeed, p1=packedvalue.uint8_t(100), wantReply="ok")

    timeStart = time.time()
    status = printer.getStatus()
    tempStart = status["t1"]

    print "Temp:", tempStart
    f.write("0 %f\n" % tempStart)

    lastTime = timeStart 
    lastTemp = tempStart

    # Build 10 second average of the temperature change derivation
    nAvg = 10
    aAvg = nAvg * 1.0

    # Stop if temp curve gets flat enough
    while abs((aAvg/nAvg)) > 0.02:

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
        print "Temp:", temp

        aAvg = aAvg - (aAvg/nAvg) + a
        print "Steigung: %7.4f %7.4f" % (a, aAvg/nAvg)

        lastTime = tim
        lastTemp = temp
        
    f.write("e\n")
    stopHeater()


####################################################################################################
#
# amax: 2.32109877421 , Tinflection: 12.5696818829 , tinflection: 15.64
# Tdead: 5.83149384288
# Mu, T: 205.63 94.4231566728
# Ko:  10.0791100704
# Kc: 12.0949, Ki:  1.0370, Kd: 35.2658
#
# https://controls.engin.umich.edu/wiki/index.php/PIDTuningClassical
# http://rn-wissen.de/wiki/index.php/Regelungstechnik
#
def zieglerNichols(args, parser):

    planner = parser.planner
    printer = planner.printer

    def stopHeater():
        payload = struct.pack("<BB", HeaterEx1, 0) # heater, pwmvalue
        printer.sendCommand(CmdSetHeaterY, binPayload=payload, wantReply="ok")
        printer.sendCommandParam(CmdFanSpeed, p1=packedvalue.uint8_t(0), wantReply="ok")

    f = open("stepresponse.gnuplot", "w")
    # f.write("set xyplane 0\n")
    f.write("set grid\n")
    f.write("plot \"-\" using 1:2 with linespoints title \"StepResponse\",")
    f.write("\"-\" using 1:($2*10) with linespoints title \"a\",")
    f.write("\"-\" using 1:($2*10) with linespoints title \"a-avg\",")
    f.write("\"-\" using 1:2 with linespoints title \"tangente\"\n")

    printer.commandInit(args)

    tmax = 240

    Xo = 128

    print "Starting input step"
    printer.sendCommandParam(CmdFanSpeed, p1=packedvalue.uint8_t(100), wantReply="ok")
    payload = struct.pack("<BB", HeaterEx1, Xo) # heater, pwmvalue
    printer.sendCommand(CmdSetHeaterY, binPayload=payload, wantReply="ok")

    timeStart = time.time()
    status = printer.getStatus()
    tempStart = status["t1"]

    print "Temp:", tempStart
    f.write("0 0\n")

    lastTime = timeStart 
    lastTemp = tempStart

    temps = {}
    aPlot = []

    nAvg = 26
    aAvg = nAvg * 1.0

    while abs((aAvg/nAvg)) > 0.02:

        if lastTemp > tmax:
            print "Error, max temp (%d) reached: " % tmax, lastTemp
            stopHeater()
            return

        time.sleep(0.1)

        tim = time.time()
        status = printer.getStatus()
        temp = status["t1"]

        relTime = tim-timeStart
        Mu = temp-tempStart

        temps[relTime] = Mu

        f.write("%f %f\n" % (relTime, Mu))

        dy = temp - lastTemp
        dx = tim - lastTime

        a = dy / dx
        print "Temp:", temp

        aPlot.append((relTime, a))

        aAvg = aAvg - (aAvg/nAvg) + a
        print "Steigung: %7.4f %7.4f" % (a, aAvg/nAvg)

        lastTime = tim
        lastTemp = temp
        
    stopHeater()

    f.write("e\n")

    for (x, y) in aPlot:
        f.write("%f %f\n" % (x, y))
    f.write("e\n")

    nAvg = 20
    aMax = 0
    Tinflection = 0

    aavg = []
    for i in range(len(aPlot)-nAvg):

        sum = 0.0
        for j in range(nAvg):
            sum += aPlot[i - (nAvg/2) + j][1]

        avg = sum/nAvg
        aavg.append((aPlot[i][0], avg))

        f.write("%f %f\n" % (aPlot[i][0], avg))

        if avg > aMax:
            aMax = avg
            Tinflection = aPlot[i][0]

    f.write("e\n")

    tinflection = temps[Tinflection]
    print "amax:", aMax, ", Tinflection:", Tinflection, ", tinflection:", tinflection

    # Tdead or Tu
    Tdead = Tinflection - (tinflection / aMax)

    print "Tdead:", Tdead

    # T or Tg
    T = Tinflection + ((Mu - tinflection) / aMax)

    print "Mu, T:", Mu, T

    f.write("%f 0\n" % Tdead)
    f.write("%f %f\n" % (T, Mu))
    f.write("e\n")

    Ko = (Xo * T) / (Mu * Tdead)
    print "Ko: ", Ko

    # Kc or Kp
    Kc = 1.2 * Ko
    # Ti or Tn (Nachstellzeit)
    Ti = 2 * Tdead
    # Td or Tv (Vorhaltezeit)
    Td = 0.5 * Tdead

    print "Kc: %7.4f, Ki: %7.4f, Kd: %7.4f" % (Kc, Kc/Ti, Kc*Td)

    # Ki = Kp/Tn
    # Kd = Kp·Tv
    # y = Kp * e + Ki * Ta * esum + Kd * (e – ealt)/Ta

####################################################################################################


#
# Allow comments in json files.
#
def jsonLoad(f):

    s = ""
    for l in f.readlines():
        l = l.strip()
        if l.startswith("#"):
            print "skip comment:", l
            continue
        s += l

    return json.loads(s)

####################################################################################################















