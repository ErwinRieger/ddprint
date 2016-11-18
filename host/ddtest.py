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

import ddhome, ddprintutil as util, time
from ddprofile import PrinterProfile
from ddprinter import Printer
from ddprintcommands import *
from ddprintstates import *

def testFilSensor(args, parser):

    planner = parser.planner
    printer = planner.printer

    printer.commandInit(args)
    startPos = printer.getFilSensor()
    util.manualMove(parser, util.dimIndex['A'], args.distance)
    endPos = printer.getFilSensor()
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

    steps_per_mm = PrinterProfile.getStepsPerMMVector()[util.A_AXIS]

    # ddhome.home(parser, args.fakeendstop)

    current_position = parser.getPos()
    apos = current_position[util.A_AXIS]

    fsstepsum = 0
    distsum = 0

    # ramp up speed from 3mm/s to 13mm/s, that is about 7.2 to 31 mm³/s flowrate (1.75mm filament)
    for feedrate in range(11):

        feedrate += 3

        # choose length of move to get about 25 points per feedrate step
        distance = 2.5 * feedrate

        apos += distance

        printer.sendPrinterInit()

        parser.execute_line("G0 F%d %s%f" % (feedrate*60, util.dimNames[util.A_AXIS], apos))

        planner.finishMoves()
        printer.sendCommand(CmdEOT)
        printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
        printer.waitForState(StateIdle)

        time.sleep(0.25)

        fssteps = printer.getFilSensor()

        print "Speed: %d, dist: %.2f, FSsteps: %d, fssteps/mm: %.4f" % (feedrate, distance, fssteps, fssteps/distance)

        distsum += distance
        fsstepsum += fssteps


    print "Average fssteps/mm: %.4f" % (fsstepsum/distsum)


