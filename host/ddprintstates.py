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

import packedvalue

############################################################################
#
# Printer states:
# ----------------------------
StateIdle    = 0
StateInit    = 1
StateErasing = 2
StateStart   = 3

StateNames = {
        StateIdle: "Idle",
        StateInit: "Init",
        StateErasing: "Erasing",
        StateStart: "Printing",
}

############################################################################
#
# Serial send result:
# ----------------------------
ResendWasOK = "ResendWasOK"

############################################################################
#
#
# MoveTypes:
# ----------------------------
# MoveTypeNone = 0
MoveTypeHoming = packedvalue.uint8_t(1) # Homing moves, endstops are used to detect end of move
MoveTypeNormal = packedvalue.uint8_t(2) # Travelling move or printing move, printer must be homed, endstops are checked while moving
MoveTypeForced = packedvalue.uint8_t(3) # Special moves (for e.g. homing), homed flages are ignored.

############################################################################
#
# Heater Id's
#
HeaterBed = 0 # Heated bed
HeaterEx1 = 1 # Extruder 1
HeaterEx2 = 2 # Extruder 2

############################################################################
#
# Response error codes
#
RespOK =                1 # Command successful
RespInvalidArgument =   2 
RespHardwareEndstop =   3 
RespSoftwareEndstop =   4 
RespUnknownBCommand =   5
RespAssertion       =   6
RespFilSensorInit   =   7
RespSDInit          =   8
RespSDReadError     =   9
RespMinTemp         =  10
RespMaxTemp         =  11
RespSDWriteError    =  12
RespUnderrun        =  13

RespCodeNames = {
        RespOK: "RespOK",
        RespInvalidArgument: "InvalidArgument",
        RespHardwareEndstop: "HardwareEndstop",
        RespSoftwareEndstop: "SoftwareEndstop",
        RespUnknownBCommand: "RespUnknownBCommand",
        RespAssertion: "RespAssertion",
        RespFilSensorInit: "RespFilSensorInit",
        RespSDInit: "RespSDInit",
        RespSDReadError: "RespSDReadError",
        RespSDWriteError: "RespSDWriteError",
        RespMinTemp: "RespMinTemp",
        RespMaxTemp: "RespMaxTemp",
        RespUnderrun: "RespUnderrun",
}

############################################################################
#
# Unsolicited message types
#
ExtrusionLimitDbg = 0x0
PidDebug          = 0x1
FilSensorDebug    = 0x2
GenericMessage    = 0x3
BufDebug          = 0x4
PidSwitch         = 0x5
GenericInt32      = 0x6

############################################################################
#
# Printing modes
#
PrintModeManual   = 0 # Manual moves, no control of bed/hotend temp
PrintModePrinting = 1 # Normal printing, control bed/hotend on layer change




