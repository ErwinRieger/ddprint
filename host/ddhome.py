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

import struct

import gcodeparser

from ddprintcommands import *
from ddprintstates import *
from ddprintconstants import dimNames

import ddprintutil as util

####################################################################################################

# Return true if move was successful. That means the endstop
# is pressed if we are moving towards the endstop and the
# endstop is released if we are moving away from the endstop.
def homeMove(parser, dim, direction, dist, fakeHomingEndstops, feedRateFactor=1.0):

    planner = parser.planner
    printer = planner.printer

    parser.setPos(planner.zeroPos)

    feedRate = printer.printerProfile.getHomeFeedrate(dim) * feedRateFactor

    print("--- homeMove(): send %s - homing move, dist: %.2f, feedrate: %.2f mm/s" % (dimNames[dim], dist*direction, feedRate))

    cmd = "G0 F%f %s%f" % (feedRate*60, dimNames[dim], dist * direction)

    # print "homeMove: ", cmd

    parser.execute_line(cmd);

    planner.finishMoves()

    # Send homing command
    printer.sendCommandParamV(CmdMove, [MoveTypeHoming])

    printer.waitForState(StateInit, wait=0.25)

    endstop = printer.endStopTriggered(dim, fakeHomingEndstops)

    if direction > 0 and printer.printerProfile.getHomeDir(dim) > 0:
        return endstop

    if direction < 0 and printer.printerProfile.getHomeDir(dim) <= 0:
        return endstop

    return not endstop

def homeBounce(parser, dim, direction, fakeHomingEndstops):

    planner = parser.planner

    # Release endstop
    if not homeMove(parser, dim, direction*-1, planner.HOME_RETRACT_MM, fakeHomingEndstops): # Back off
        return False

    # Press endstop
    if not homeMove(parser, dim, direction, planner.HOME_RETRACT_MM * 1.5, fakeHomingEndstops, 0.33): # Move towards endstop slowly
        return False

    # Release endstop
    return homeMove(parser, dim, direction*-1, planner.HOME_RETRACT_MM * 1.5, fakeHomingEndstops, 0.33) # Back off


####################################################################################################

def home(args, printer, parser, planner, force=False):

    print("*")
    print("* Start homing...")
    print("*")

    (homePosMMLifted, homePosSteppedLifted) = planner.getHomePos(True)

    if printer.isHomed() and not force:
        print("Printer is homed already...")

        # Get current pos from printer and set our virtual pos
        curPosMM = util.getVirtualPos(printer, parser)
        parser.setPos(curPosMM)

        # Move to home
        if not curPosMM.equal(homePosMMLifted, "XYZ"):
    
            #
            # Z Achse isoliert und als erstes bewegen, um zusammenstoss mit den klammern
            # der buildplatte oder mit objekten auf der druckplatte zu vermeiden.
            #
            if not curPosMM.equal(homePosMMLifted, "Z"):

                feedRate = printer.printerProfile.getMaxFeedrateI(util.Z_AXIS)
                parser.execute_line("G0 F%d Z%f" % (feedRate*60, homePosMMLifted.Z))

            if not curPosMM.equal(homePosMMLifted, "XY"):

                feedRate = printer.printerProfile.getMaxFeedrateI(util.X_AXIS)
                parser.execute_line("G0 F%d X%f Y%f" % (
                    feedRate*60, 
                    homePosMMLifted.X, homePosMMLifted.Y))

            planner.finishMoves()
            printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
            printer.waitForState(StateInit, wait=0.25)

            #
            # Set Virtual pos
            #
            parser.setPos(homePosMMLifted)

        print("*")
        print("* Done homing...")
        print("*")
        return

    # Position directly after homing
    (homePosMM, homePosStepped) = planner.getHomePos()

    def liftHead():

        # print "Debug: Tell parser its position [mm]:", homePosMM
        parser.setPos(homePosMM)

        # print "Debug: Tell printer its position [steps]:", homePosStepped
        payload = struct.pack("<iiiii", *homePosStepped)
        printer.sendCommand(CmdSetPos, binPayload=payload)

        # Move bed away from nozzle
        if homePosMM[dim] != homePosMMLifted[dim]:

            feedRate = printer.printerProfile.getHomeFeedrate(dim);
            parser.execute_line("G0 F%d %s%f" % (feedRate*60, dimNames[dim], homePosMMLifted[dim]))
            planner.finishMoves()
            printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
            printer.waitForState(StateInit, log=True)

            homePosMM[dim] = homePosMMLifted[dim]
            homePosStepped[dim] = homePosSteppedLifted[dim]

        # print "Debug: printer pos after lift:", homePosMM
        # print "Debug: parser pos after lift:", parser.getPos()


    printer.erase(2048 * 10)

    #
    # Z Achse isoliert und als erstes bewegen, um zusammenstoss mit den klammern
    # der buildplatte oder mit einem modell zu vermeiden.
    #
    for dim in [util.Z_AXIS, util.X_AXIS, util.Y_AXIS]:

        if printer.printerProfile.getHomeDir(dim) > 0:
            direction = 1.0
        else:
            direction = -1.0

        print("\nHoming axis %s, direction: %d" % (dimNames[dim], direction))

        # Try fast home if endstop is pressed
        if printer.endStopTriggered(dim, args.fakeendstop):

            print("Homing: %s - endstop is triggered, trying fast home." % dimNames[dim])
            if homeBounce(parser, dim, direction, args.fakeendstop):
                liftHead()
                continue

        if not homeMove(parser, dim, direction, planner.MAX_POS[dim] * 1.25, args.fakeendstop): # Move towards endstop fast
            print("Error, %s - endstop NOT hit!" % dimNames[dim])
            assert(0)

        if not homeBounce(parser, dim, direction, args.fakeendstop):
            print("Error, Bounce %s - endstop!" % dimNames[dim])
            assert(0)

        liftHead()

    print("*")
    print("* Done homing...")
    print("*")

####################################################################################################

def assureIsHomed(args, printer, parser, planner):

    if not printer.isHomed():

        input("Press return to start homing...")
        home(args, printer, parser, planner)

    else:

        # Printer is homed and knows it's position,
        # tell parser where it is
        curPosMM = util.getVirtualPos(printer, parser)
        parser.setPos(curPosMM)

####################################################################################################



