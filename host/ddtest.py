#!/usr/bin/python
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

#
# Some seldom used functions, needed only if the hardware of the printer is modified.
#

import ddhome, ddprintutil as util
from ddprinter import Printer
from ddprintcommands import *
from ddprintstates import *

def testFilSensor(args, parser):

    planner = parser.planner
    printer = planner.printer

    printer.commandInit(args)
    startPos = printer.query(CmdGetFilSensor)
    util.manualMove(parser, util.dimIndex['A'], args.distance)
    endPos = printer.query(CmdGetFilSensor)
    diff = endPos - startPos
    print "Filament pos:", startPos, endPos, "counts, difference: %d,  %.3f mm" % (diff, diff*25.4/1000)


#
# Calibrate filament sensor, determine ratio between extruder stepper feedrate and the speed
# measured by the flowrate sensor.
#

def calibrateFilSensor(args, parser):

    planner = parser.planner
    printer = planner.printer

    printer.commandInit(args)

    # ddhome.home(parser, args.fakeendstop)

    printer.sendPrinterInit()

    current_position = parser.getRealPos()
    apos = current_position[util.A_AXIS]

    # ramp up speed from 1mm/s to 13mm/s, that is about 2.5 to 31 mm³/s flowrate
    for feedrate in range(13):

        feedrate += 1

        # choose length of move to get about 25 points per feedrate step
        distance = 2.5 * feedrate

        apos += distance
        parser.execute_line("G0 F%d %s%f" % (feedrate*60, util.dimNames[util.A_AXIS], apos))

    planner.finishMoves()
    printer.sendCommand(CmdEOT, wantReply="ok")
    printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
    printer.waitForState(StateIdle)




