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
# CmdDirBits       = 0x2
CmdSyncFanSpeed  = 0x3 # Parameters: pwm value 0 - 255
CmdRaw           = 0x4 # Parameters: blob data
CmdBlock         = 0x6 # A 512byte block of a lager command

CmdG1            = 0x7
# CmdDirG1         = 0x8 # CmdDirBits and CmdG1 combined

CmdSyncTargetTemp= 0xb # Parameters: heater, temp 
CmdDwellMS       = 0xc # Parameters: dwell time in mS
CmdG1Raw         = 0xd # Raw print move steps, bresenham algo already done.
# CmdDirG1Raw      = 0xe # CmdDirBits and CmdG1Raw combined
CmdSyncHotendPWM = 0xf # Set hotend pmw value, Parameters: heater, pmw value

CmdUnknown       = 0x7f # Unknown command for debugging

#
# Direct commands:
# ----------------------
#
CmdPrinterInit = 128
# * start move, print
CmdMove = 129               # Parameters: MoveType
# * End of text, all moves have been sent
CmdEOT = 130
CmdResetLineNr = 131
# free: 132
CmdSetHomePos = 133
# Set heater target temp
CmdSetTargetTemp = 134 # Parameters: heater, temp
# CmdWriteEepromFloat = 135 # Parameters: valuename (len max 63 chars!), value
# CmdEepromFactory = 136 # Factory reset of eeprom values, bed leveling needed
CmdFanSpeed = 137
CmdStopMove = 138
# Set heater PWM value (stellgrösse) directly, used by PID autoTune. Parameters: heater, pwmvalue
CmdSetHeaterY = 139

CmdGetDirBits = 150
# Get homed flag
CmdGetHomed = 151
# Get endstop switch state
CmdGetEndstops = 152   # Get endstop state and pos
# CmdGetEepromVersion = 153
# CmdGetEepromSettings = 154
# currently not used: CmdDisableStepperIsr = 155
CmdDisableSteppers = 156
# Get temperatures
CmdGetCurrentTemps = 157
CmdGetTargetTemps = 158
CmdGetPos = 159
CmdExit = 160
CmdGetStatus = 161
CmdGetFilSensor = 162 # Get raw value of filament sensor pos
CmdGetTempTable = 163 # ExtrusionLimit: get tempTable 
CmdSetTempTable = 164 # ExtrusionLimit: set tempTable
CmdEnableFRLimit = 165 # Enable/disable flowrate limit

CmdSetContTimer =    166 # Timer value for CmdContinuousE -> E-Speed
CmdContinuousE =     167 # Start/Stop continuous e-move for filament measurement
CmdSetFilSensorCal = 168 # Flowrate sensor: set calibration value.
CmdSetStepsPerMME =  169 # Set steps per mm value
CmdSetPrinterName =  170 # Set printer (-profile) name from printer eeprom, payload is a 'pascal string'
CmdGetPrinterName =  171 # Read printer (-profile) name from printer eeprom, payload is a 'pascal string'

CmdSetPIDValues =    172
# CmdSetBedlevelOffset = 173
CmdSetIncTemp   =    174 # Adjust temperature niveau 
CmdGetFreeMem   =    175 # Get number of free memory bytes
CmdGetFSReadings =   176 # Get last n filsensor readings
CmdSetTempPWM =      177 # Set PWM value of hotend heater (for filament profile measurement)

CommandNames = {
}

def insertCommandName(cmd, cmdName):

    assert(cmdName not in CommandNames)

    CommandNames[cmd] = cmdName

for (cmd, cmdName) in [
    (CmdNull, "CmdNull",),
    (CmdG1, "CmdG1",),
    # CmdDirBits, "CmdDirBits",
    (CmdBlock, "CmdBlock",),
    # CmdDirG1, "CmdDirG1",
    (CmdG1Raw, "CmdG1Raw",),
    # CmdDirG1Raw, "CmdDirG1Raw",
    (CmdSyncTargetTemp, "CmdSyncTargetTemp",),
    (CmdDwellMS, "CmdDwellMS",),
    (CmdSyncHotendPWM, "CmdSyncHotendPWM",),
    (CmdUnknown, "CmdUnknown",),
    #
    # Direct commands:
    # ----------------------
    #
    (CmdPrinterInit, "CmdPrinterInit",),
    # * start move, print
    (CmdMove, "CmdMove",),
    # * End of text, all moves have been sent
    (CmdEOT, "CmdEOT",),
    (CmdResetLineNr, "CmdResetLineNr",),
    (CmdSetHomePos, "CmdSetHomePos",),
    (CmdSetTargetTemp, "CmdSetTargetTemp",),
    # (CmdWriteEepromFloat, "CmdWriteEepromFloat",),
    # (CmdEepromFactory, "CmdEepromFactory",),
    (CmdSyncFanSpeed, "CmdSyncFanSpeed",),
    (CmdFanSpeed, "CmdFanSpeed",),
    (CmdStopMove, "CmdStopMove",),
    (CmdSetHeaterY, "CmdSetHeaterY",),
    (CmdRaw, "CmdRaw",),
    
    # Getters
    (CmdGetDirBits, "CmdGetDirBits",),
    (CmdGetHomed, "CmdGetHomed",),
    (CmdGetEndstops, "CmdGetEndstops",),
    # (CmdGetEepromVersion, "CmdGetEepromVersion",),
    # (CmdGetEepromSettings, "CmdGetEepromSettings",),
    # currently not used: CmdDisableStepperIsr: "CmdDisableStepperIsr",
    (CmdDisableSteppers, "CmdDisableSteppers",),
    (CmdGetCurrentTemps, "CmdGetCurrentTemps",),
    (CmdGetTargetTemps, "CmdGetTargetTemps",),
    (CmdGetPos, "CmdGetPos",),
    (CmdExit, "CmdExit",),
    (CmdGetStatus, "CmdGetStatus",),
    (CmdGetFilSensor, "CmdGetFilSensor",),
    (CmdGetTempTable, "CmdGetTempTable",),
    (CmdSetTempTable, "CmdSetTempTable",),
    (CmdEnableFRLimit, "CmdEnableFRLimit",),
    (CmdSetContTimer, "CmdSetContTimer",),
    (CmdContinuousE, "CmdContinuousE",),
    (CmdSetFilSensorCal, "CmdSetFilSensorCal",),
    (CmdSetStepsPerMME, "CmdSetStepsPerMME",),
    (CmdSetPrinterName, "CmdSetPrinterName",),
    (CmdGetPrinterName, "CmdGetPrinterName",),
    (CmdSetPIDValues, "CmdSetPIDValues",),
    # (CmdSetBedlevelOffset, "CmdSetBedlevelOffset",),
    (CmdSetIncTemp, "CmdSetIncTemp",),
    (CmdGetFreeMem, "CmdGetFreeMem",),
    (CmdGetFSReadings, "CmdGetFSReadings",),
    (CmdSetTempPWM, "CmdSetTempPWM",),
    ]:

        insertCommandName(cmd, cmdName)

#
# Flag bits for CmdG1x commands
#
# Bits 0, 1, 2, 3, 4 reserved for direction bits
DecelByteFlagBit     = (1 << 5) # 0x20
AccelByteFlagBit     = (1 << 6) # 0x40
DirBitsBit           = (1 << 7) # 0x80
MoveStartBit         = (1 << 8) # 0x100
MeasureStartBit      = (1 << 9) # 0x200
# Raw moves
MoveStartBitRaw      = (1 << 5) # 0x20
TimerByteFlagBit     = (1 << 6) # 0x40
DirBitsBitRaw        = (1 << 7) # 0x80
MeasureStartBitRaw   = (1 << 8) # 0100

#
# Response codes
#
RespUnknownCommand =            1
RespKilled =                    3
RespRXError =                   4 # Payload: serialNumber (last line), errorflags
RespRXCRCError =                5 # Payload: serialNumber (last line)
RespACK =                       6 #
RespSerNumberError =            7 # Payload: serialNumber (last line)
RespRXTimeoutError =            8 # Payload: serialNumber (last line)
RespUnsolicitedMsg =            9 # Payload: message type, params

ResponseNames = {
        RespUnknownCommand: "RespUnknownCommand",
        RespKilled: "RespKilled",
        RespRXError: "RespRXError",
        RespRXCRCError: "RespRXCRCError",
        RespACK: "RespACK",
        RespSerNumberError: "RespSerNumberError",
        RespRXTimeoutError: "RespRXTimeoutError",
        RespUnsolicitedMsg: "RespUnsolicitedMsg",
}















############################################################################
