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

import packedvalue

############################################################################
#
# Printer states:
# ----------------------------
StateIdle = 0
StateInit = 1
StateStart = 2
StateDwell = 3


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
RespADNS9800Init    =   7
RespSDInit          =   8
RespSDError         =   9
RespMinTemp         =  10
RespMaxTemp         =  11

RespCodeNames = {
        RespOK: "RespOK",
        RespInvalidArgument: "InvalidArgument",
        RespHardwareEndstop: "HardwareEndstop",
        RespSoftwareEndstop: "SoftwareEndstop",
        RespUnknownBCommand: "RespUnknownBCommand",
        RespAssertion: "RespAssertion",
        RespADNS9800Init: "RespADNS9800Init",
        RespSDInit: "RespSDInit",
        RespSDError: "RespSDError",
        RespMinTemp: "RespMinTemp",
        RespMaxTemp: "RespMaxTemp",
}






