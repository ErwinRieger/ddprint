# -*- coding: utf-8 -*-
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
############################################################################

# Buffered commands:
# ----------------------
#
CmdNull          = 0x0
CmdDirBits       = 0x2
CmdSyncFanSpeed  = 0x3 # Parameters: pwm value 0 - 255
CmdRaw           = 0x4 # Parameters: blob data
CmdBlock         = 0x6 # A 512byte block of a lager command

CmdG1            = 0x7
CmdDirG1         = 0x8 # CmdDirBits and CmdG1 combined

CmdSyncTargetTemp= 0xb # Parameters: heater, temp 
CmdDwellMS       = 0xc # Parameters: dwell time in mS
CmdG1Raw         = 0xd # Raw print move steps, bresenham algo already done.
CmdDirG1Raw      = 0xe # CmdDirBits and CmdG1Raw combined

CmdUnknown       = 0x7f # Unknown command for debugging

#
# Direct commands:
# ----------------------
#
CmdPrinterInit = 128
# * get homed flag
# * get endstop state
# * get temp
# 
# * set temp
# 
# 
# 
# 
# * dump file buffer
# * start move, print
CmdMove = 129               # Parameters: MoveType
# * End of text, all moves have been sent
CmdEOT = 130
CmdResetLineNr = 131
# CmdHome = 132
CmdSetHomePos = 133
CmdSetTargetTemp = 134 # Parameters: heater, temp
CmdWriteEepromFloat = 135 # Parameters: valuename (len max 63 chars!), value
CmdEepromFactory = 136 # Factory reset of eeprom values, bed leveling needed
CmdFanSpeed = 137
CmdStopMove = 138
# Set heater PWM value (stellgrösse) directly, used by PID autoTune. Parameters: heater, pwmvalue
CmdSetHeaterY = 139

# Getters, they return a string of the form "ret: <expr>"
# currently not used: CmdGetState = 150
CmdGetHomed = 151
CmdGetEndstops = 152   # Get endstop state and pos
CmdGetEepromVersion = 153
CmdGetEepromSettings = 154
# currently not used: CmdDisableStepperIsr = 155
CmdDisableSteppers = 156
CmdGetCurrentTemps = 157
CmdGetTargetTemps = 158
CmdGetPos = 159
CmdExit = 160
CmdGetStatus = 161
CmdGetFilSensor = 162 # Get raw value of filament sensor pos
CmdGetTempTable = 163 # ExtrusionLimit: get tempTable 
CmdSetTempTable = 164 # ExtrusionLimit: set tempTable
CmdEnableFRLimit = 165 # Enable/disable flowrate limit

CommandNames = {
    CmdNull: "CmdNull",
    CmdG1: "CmdG1",
    CmdDirBits: "CmdDirBits",
    CmdBlock: "CmdBlock",
    CmdDirG1: "CmdDirG1",
    CmdG1Raw: "CmdG1Raw",
    CmdDirG1Raw: "CmdDirG1Raw",
    CmdSyncTargetTemp: "CmdSyncTargetTemp",
    CmdDwellMS: "CmdDwellMS",
    CmdUnknown: "CmdUnknown",
    #
    # Direct commands:
    # ----------------------
    #
    CmdPrinterInit: "CmdPrinterInit",
    # * get homed flag
    # * get endstop state
    # * get temp
    # 
    # * set temp
    # 
    # 
    # 
    # 
    # * dump file buffer
    # * start move, print
    CmdMove: "CmdMove",
    # * End of text, all moves have been sent
    CmdEOT: "CmdEOT",
    CmdResetLineNr: "CmdResetLineNr",
    # CmdHome: "CmdHome",
    CmdSetHomePos: "CmdSetHomePos",
    CmdSetTargetTemp: "CmdSetTargetTemp",
    CmdWriteEepromFloat: "CmdWriteEepromFloat",
    CmdEepromFactory: "CmdEepromFactory",
    CmdSyncFanSpeed: "CmdSyncFanSpeed",
    CmdFanSpeed: "CmdFanSpeed",
    CmdStopMove: "CmdStopMove",
    CmdSetHeaterY: "CmdSetHeaterY",
    CmdRaw: "CmdRaw",
    
    # Getters, they return a string of the form "ret: <expr>"
    # currently not used: CmdGetState: "CmdGetState",
    CmdGetHomed: "CmdGetHomed",
    CmdGetEndstops: "CmdGetEndstops",
    CmdGetEepromVersion: "CmdGetEepromVersion",
    CmdGetEepromSettings: "CmdGetEepromSettings",
    # currently not used: CmdDisableStepperIsr: "CmdDisableStepperIsr",
    CmdDisableSteppers: "CmdDisableSteppers",
    CmdGetCurrentTemps: "CmdGetCurrentTemps",
    CmdGetTargetTemps: "CmdGetTargetTemps",
    CmdGetPos: "CmdGetPos",
    CmdExit: "CmdExit",
    CmdGetStatus: "CmdGetStatus",
    CmdGetFilSensor: "CmdGetFilSensor",
    CmdGetTempTable: "CmdGetTempTable",
    CmdSetTempTable: "CmdSetTempTable",
    CmdEnableFRLimit: "CmdEnableFRLimit",
}

#
# Response codes
#
RespUnknownCommand =            1
# RespGenericString =             2
RespKilled =                    3
RespRXError =                   4 # Payload: serialNumber (last line), errorflags
RespRXCRCError =                5 # Payload: serialNumber (last line)
RespACK =                       6 #
RespSerNumberError =            7 # Payload: serialNumber (last line)
RespRXTimeoutError =            8 # Payload: serialNumber (last line)
RespUnsolicitedMsg =            9 # Payload: message type, params

ResponseNames = {
        RespUnknownCommand: "RespUnknownCommand",
        # RespGenericString: "RespGenericString",
        RespKilled: "RespKilled",
        RespRXError: "RespRXError",
        RespRXCRCError: "RespRXCRCError",
        RespACK: "RespACK",
        RespSerNumberError: "RespSerNumberError",
        RespRXTimeoutError: "RespRXTimeoutError",
        RespUnsolicitedMsg: "RespUnsolicitedMsg",
}















############################################################################
