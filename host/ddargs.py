#!/usr/bin/python
# -*- coding: utf-8 -*-
#
#/*
# This file is part of ddprint - a 3D printer firmware.
# 
# Copyright 2021 erwin.rieger@ibrieger.de
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
# along with ddprint. If not, see <http://www.gnu.org/licenses/>.
#*/

#
# Common command line argument parsing stuff
#

import sys, os
import argparse

#
# For float commandline arg range check
#
class ArgRange(object):
    def __init__(self, start, end):
        self.start = start
        self.end = end
    def __eq__(self, other):
        return self.start <= other <= self.end
    def __repr__(self):
        return "%.2f:%.2f" % (self.start, self.end)

def addCommonArguments(argParser):

    # Serial device
    defaultSerialDev = os.getenv("DDPRINTDEV") or os.getenv("dev") or "/dev/ttyACM0"
    argParser.add_argument("-d", dest="device", action="store", type=str, help="Device to use, default: %s." % defaultSerialDev, default=defaultSerialDev)
    argParser.add_argument("-b", dest="baud", action="store", type=int, help="Baudrate, default 1Mbaud.", default=1000000)

    # Temperatures, auto-temp
    argParser.add_argument("-t0", dest="t0", action="store", type=int, help="Temp 0 (heated bed), default comes from mat. profile.")
    argParser.add_argument("-t0-reduced", dest="t0_reduced", action="store", type=int, help="Reduced temp 0 (heated bed), default comes from mat. profile.")
    argParser.add_argument("-t1", dest="t1", action="store", type=int, help="Temp 1 (hotend 1), default comes from mat. profile.")
    argParser.add_argument("-autotemp", dest="autotemp", action="store", type=int, help="Use autotemp algorithm, default is True.", default=1)
    argParser.add_argument("-pidset", dest="pidset", action="store", type=str, help="Debug: Specify PID parameter sets to use (ZNCH).", default="ZNCH")
    argParser.add_argument("-inctemp", dest="inctemp", action="store", type=int, help="Increase extruder temperature niveau (layer bonding).", default=0)
    argParser.add_argument("-dt", dest="dummyTempTable", action="store", type=bool, help="Debug: download dummy temperature table, don't limit speeed.", default=False)
    # XXX Should we call this parameter strength, figurine-mode or parts-mode?
    argParser.add_argument("-wp", dest="workingPoint", action="store", type=float, choices=[ArgRange(0.0, 1.0)], help="AutoTemp: Working Point in range [0.0:1.0] 0: strong parts (higher temp range), 1: figurine mode (lower temps). Default: 0.5.", default=0.5)

    # Linear advance
    argParser.add_argument("-kadvance", dest="kadvance", action="store", type=float, choices=[ArgRange(0.0, 1.0)], help="Linear Advance K-factor in range [0.0:1.0], default comes from mat. profile.")
    argParser.add_argument("-startadvance", dest="startAdvance", action="store", type=float, help="Gradual advance: advance startvalue.")
    argParser.add_argument("-advincrease", dest="advIncrease", action="store", type=float, help="Gradual advance: increase kAdvance by advIncrease after each part.")
    argParser.add_argument("-advstepheight", dest="advStepHeight", action="store", type=int, help="Gradual advance: height of each step (number of layers).")

    # Material profile
    argParser.add_argument("-smat", dest="smat", action="store", help="Name of specific material profile to use.")

    # Debug/testing
    # fake endstops as long we have no real ones
    argParser.add_argument("-F", dest="fakeendstop", action="store", type=bool, help="Debug: fake endstops", default=False)

def addPrinterArgument(parser):
    parser.add_argument("printer", help="Name of printer to select profiles.")
    
def addNozzleArgument(parser):
    parser.add_argument("nozzle", help="Name of nozzle profile to use [nozzle40, nozzle80...].")

def addMatArgument(parser):
    parser.add_argument("mat", help="Name of generic material profile to use [pla, petg...].")

def addPrintArguments(parser):

    addNozzleArgument(parser)
    addMatArgument(parser)
    parser.add_argument("gfile", help="Input GCode file.")

