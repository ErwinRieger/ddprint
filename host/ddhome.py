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

def homeMove(parser, dim, direction, dist, fakeHomingEndstops, feedRateFactor=1.0):

    planner = parser.planner
    printer = planner.printer

    parser.setPos(planner.zeroPos)

    feedRate = planner.HOMING_FEEDRATE[dim]*feedRateFactor

    print "--- homeMove(): send %s - homing move, dist: %.2f, feedrate: %.2f mm/s" % (dimNames[dim], dist*direction * planner.HOME_DIR[dim], feedRate)

    cmd = "G0 F%f %s%f" % (
        feedRate*60, dimNames[dim], dist * direction * planner.HOME_DIR[dim])

    # print "homeMove: ", cmd

    parser.execute_line(cmd);

    planner.finishMoves()

    # Send homing command
    printer.sendCommandParamV(CmdMove, [MoveTypeHoming])
    # printer.sendCommand(CmdEOT)

    printer.waitForState(StateInit, wait=0.25, log=True)

    if direction > 0:
        # Move towards endstop
        # Check, if enstop was triggered
        if not printer.endStopTriggered(dim, fakeHomingEndstops):
            return False

    else:
        # Move away from endstop
        # Check, if enstop was released
        if printer.endStopTriggered(dim):
            return False

    return True

def homeBounce(parser, dim, fakeHomingEndstops):

    planner = parser.planner
    printer = planner.printer

    if not homeMove(parser, dim, -1, planner.HOME_RETRACT_MM, fakeHomingEndstops): # Back off
        return False

    if not homeMove(parser, dim, 1, planner.HOME_RETRACT_MM * 1.5, fakeHomingEndstops, 0.33): # Move towards endstop slowly
        return False

    return homeMove(parser, dim, -1, planner.HOME_RETRACT_MM * 1.5, fakeHomingEndstops, 0.33) # Back off


####################################################################################################

def home(args, printer, parser, planner, force=False):

    print "*"
    print "* Start homing..."
    print "*"
    if printer.isHomed() and not force:
        print "Printer is homed already..."

        # Get current pos from printer and set our virtual pos
        curPosMM = util.getVirtualPos(printer, parser)

        (homePosMM, homePosStepped) = planner.getHomePos()

        # print "Printer should be at [mm]: ", homePosMM, ", [steps]: ", homePosStepped

        # Move to home
        if not curPosMM.equal(homePosMM, "XYZ"):

    
            #
            # Z Achse isoliert und als erstes bewegen, um zusammenstoss mit den klammern
            # der buildplatte oder mit objekten auf der druckplatte zu vermeiden.
            #
            if not curPosMM.equal(homePosMM, "Z"):

                feedRate = printer.printerProfile.getMaxFeedrateI(util.Z_AXIS)
                print "--- home(): homing z-move feedrate: %.2f mm/s" % (feedRate)

                parser.execute_line("G0 F%d Z%f" % (feedRate*60, homePosMM.Z))

            if not curPosMM.equal(homePosMM, "XY"):

                feedRate = printer.printerProfile.getMaxFeedrateI(util.X_AXIS)
                print "--- home(): homing xy-move feedrate: %.2f mm/s" % (feedRate)

                parser.execute_line("G0 F%d X%f Y%f" % (
                    feedRate*60, 
                    homePosMM.X, homePosMM.Y))

            planner.finishMoves()
            printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
            # printer.sendCommand(CmdEOT)

            printer.waitForState(StateInit, wait=0.25, log=True)

        #
        # Set Virtual E-pos 0:
        #
        parser.setPos(homePosMM)

        print "*"
        print "* Done homing..."
        print "*"
        return

    #
    # Z Achse isoliert und als erstes bewegen, um zusammenstoss mit den klammern
    # der buildplatte oder mit einem modell zu vermeiden.
    #
    for dim in [util.Z_AXIS, util.X_AXIS, util.Y_AXIS]:

        print "\nHoming axis %s" % dimNames[dim]

        # Try fast home if endstop is pressed
        if printer.endStopTriggered(dim, args.fakeendstop):

            print "Homing: %s - endstop is triggered, trying fast home." % dimNames[dim]
            if homeBounce(parser, dim, args.fakeendstop):
                continue

        """
        # Try fast home if head/bed is near the endstop
        print "Homing: doing short/fast home."
        if homeMove(parser, dim, 1, planner.MAX_POS[dim] / 10.0, args.fakeendstop): # Move towards endstop fast

            print "Homing: %s - endstop is triggered, trying fast home." % dimNames[dim]
            if homeBounce(parser, dim, args.fakeendstop):
                continue
        """

        # Do the full slow homing move
        print "Homing: doing full/slow home."
        if not homeMove(parser, dim, 1, planner.MAX_POS[dim] * 1.25, args.fakeendstop): # Move towards endstop fast
            print "Error, %s - endstop NOT hit!" % dimNames[dim]
            assert(0)

        if not homeBounce(parser, dim, args.fakeendstop):
            print "Error, Bounce %s - endstop!" % dimNames[dim]
            assert(0)

    #
    # Set position in steps on the printer side and set our 'virtual position':
    #
    (homePosMM, homePosStepped) = planner.getHomePos()
    parser.setPos(homePosMM)

    # print "Tell printer its position [steps]:", homePosStepped
    payload = struct.pack("<iiiii", *homePosStepped)
    printer.sendCommand(CmdSetHomePos, binPayload=payload)

    print "*"
    print "* Done homing..."
    print "*"

####################################################################################################

def assureIsHomed(args, printer, parser, planner):

    if not printer.isHomed():
        raw_input("Press return to start homing...")
        home(args, printer, parser, planner)

####################################################################################################



