# -*- coding: utf-8 -*-
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
############################################################################

# Buffered commands:
# ----------------------
#
CmdNull          = 0x0
# CmdDirBits       = 0x2
CmdSyncFanSpeed  = 0x3 # Parameters: pwm value 0 - 255, blipTime 0 - 255 mS
CmdRaw           = 0x4 # Parameters: blob data
Cmd5             = 0x5
# CmdBlock         = 0x6 # A 512byte block of a lager command
CmdG1            = 0x7
# CmdDirG1         = 0x8 # CmdDirBits and CmdG1 combined

CmdSyncTargetTemp= 0xb # Parameters: heater, temp 
CmdDwellMS       = 0xc # Parameters: number of 25 mS dwell NOP moves
CmdG1Raw         = 0xd # Raw print move steps, bresenham algo already done.
# CMDDirG1Raw      = 0xe # CmdDirBits and CmdG1Raw combined

CmdSuggestPwm    = 0x11 # Parameters: heater, target temp, pwm value

CmdNop           = 0x77 # NOP command for padding of partial sectors
CmdUnknown       = 0x7f # Unknown command for debugging

#
# Direct commands:
# ----------------------
#
CmdPrinterInit = 128
# * start move, print
CmdMove = 129               # Parameters: MoveType
CmdResetLineNr = 131
# free: 132
CmdSetPos = 133
# Set heater target temp
CmdSetTargetTemp = 134 # Parameters: heater, temp
CmdGetCardSize     = 135      # Get size of mass storage, number of 512b blocks
CmdErase           = 136      # Erase mass storage, parameter: number of 512b blocks
CmdFanSpeed = 137
CmdStopMove = 138

# CmdGetDirBits = 150
# Get homed flag
CmdGetHomed = 151
# Get endstop switch state
CmdGetEndstops = 152   # Get endstop state and pos
# currently not used: CmdDisableStepperIsr = 155
CmdDisableSteppers = 156
# Get temperatures
CmdGetCurrentTemps = 157
CmdGetTargetTemps = 158
CmdGetPos = 159
CmdExit = 160
CmdGetStatus = 161
CmdGetFilSensor = 162 # Get raw value of filament sensor pos
# CmdGetTempTable = 163 # ExtrusionLimit: get tempTable 
CmdSetTempTable = 164 # ExtrusionLimit: set tempTable
CmdEnableFRLimit = 165 # Enable/disable flowrate limit

CmdSetContTimer =    166 # Timer value for CmdContinuousE -> E-Speed
CmdContinuousE =     167 # Start/Stop continuous e-move for filament measurement
CmdSetFilSensorConfig = 168 # Flowrate sensor: Set steps per mm value and calibration value.
CmdSetFilSensorCal = 169 # Flowrate sensor: set calibration value.
CmdSetPrinterName =  170 # Set printer (-profile) name from printer eeprom, payload is a 'pascal string'
CmdGetPrinterName =  171 # Read printer (-profile) name from printer eeprom, payload is a 'pascal string'

CmdSetPIDValues =    172
# CmdSetBedlevelOffset = 173
CmdSetIncTemp   =    174 # Adjust temperature niveau 
CmdGetFreeMem   =    175 # Get number of free memory bytes
CmdGetFSReadings =   176 # Get last n filsensor readings
CmdSetTempPWM =      177 # Set PWM value of hotend heater (for filament profile measurement)
CmdSoftStop    =     180 # Stop printer softly after current path is finished.
CmdBootBootloader =  181 # Reboot into bootloader (stm32)
CmdReadGpio =        182 # Read a gpio port, used to determine pinmap.
CmdSetGpio =         183 # Set a gpio port, dangerous, used to determine pinmap.
CmdReadAnalogGpio =  184 # Read a analog value from gpio port, used to determine pinmap.
CmdSetStepsPerMM  =  185 # Initial printer settings: steps per mm for X/Y/Z
CmdSetHostSettings = 186 # Initial printer settings
CmdSystemReset     = 187 # Emergency hard reset system 
CmdGetTaskStatus   = 188 # 
CmdGetIOStats      = 189 # 
CmdDumpMassStorage = 190 # Dump a 512 bytes sector from sdcard/usb.
CmdSetBaudRate     = 191 # Autobaudrate
CmdSetSlowDown     = 192 # Slowdown print, for filament measurement
CmdGetVersion      = 193 # Get git version.

CommandNames = {
}

def insertCommandName(cmd, cmdName):

    assert(cmdName not in CommandNames)

    CommandNames[cmd] = cmdName

for (cmd, cmdName) in [
    (CmdNull, "CmdNull",),
    (CmdG1, "CmdG1",),
    # CmdDirBits, "CmdDirBits",
    # (CmdBlock, "CmdBlock",),
    (Cmd5, "Cmd5",),
    # CmdDirG1, "CmdDirG1",
    (CmdG1Raw, "CmdG1Raw",),
    # CmdDirG1Raw, "CmdDirG1Raw",
    (CmdSyncTargetTemp, "CmdSyncTargetTemp",),
    (CmdDwellMS, "CmdDwellMS",),
    (CmdSuggestPwm, "CmdSuggestPwm",),
    (CmdUnknown, "CmdUnknown",),
    #
    # Direct commands:
    # ----------------------
    #
    (CmdPrinterInit, "CmdPrinterInit",),
    # * start move, print
    (CmdMove, "CmdMove",),
    (CmdResetLineNr, "CmdResetLineNr",),
    (CmdSetPos, "CmdSetPos",),
    (CmdSetTargetTemp, "CmdSetTargetTemp",),
    (CmdGetCardSize, "CmdGetCardSize",),
    (CmdErase, "CmdErase",),
    (CmdSyncFanSpeed, "CmdSyncFanSpeed",),
    (CmdFanSpeed, "CmdFanSpeed",),
    (CmdStopMove, "CmdStopMove",),
    (CmdRaw, "CmdRaw",),
    
    # Getters
    # (CmdGetDirBits, "CmdGetDirBits",),
    (CmdGetHomed, "CmdGetHomed",),
    (CmdGetEndstops, "CmdGetEndstops",),
    # currently not used: CmdDisableStepperIsr: "CmdDisableStepperIsr",
    (CmdDisableSteppers, "CmdDisableSteppers",),
    (CmdGetCurrentTemps, "CmdGetCurrentTemps",),
    (CmdGetTargetTemps, "CmdGetTargetTemps",),
    (CmdGetPos, "CmdGetPos",),
    (CmdExit, "CmdExit",),
    (CmdGetStatus, "CmdGetStatus",),
    (CmdGetFilSensor, "CmdGetFilSensor",),
    # (CmdGetTempTable, "CmdGetTempTable",),
    (CmdSetTempTable, "CmdSetTempTable",),
    (CmdEnableFRLimit, "CmdEnableFRLimit",),
    (CmdSetContTimer, "CmdSetContTimer",),
    (CmdContinuousE, "CmdContinuousE",),
    (CmdSetFilSensorConfig, "CmdSetFilSensorConfig",),
    # (CmdSetStepsPerMME, "CmdSetStepsPerMME",),
    (CmdSetPrinterName, "CmdSetPrinterName",),
    (CmdGetPrinterName, "CmdGetPrinterName",),
    (CmdSetPIDValues, "CmdSetPIDValues",),
    # (CmdSetBedlevelOffset, "CmdSetBedlevelOffset",),
    (CmdSetIncTemp, "CmdSetIncTemp",),
    (CmdGetFreeMem, "CmdGetFreeMem",),
    (CmdGetFSReadings, "CmdGetFSReadings",),
    (CmdSetTempPWM, "CmdSetTempPWM",),
    (CmdSoftStop, "CmdSoftStop",),
    (CmdBootBootloader, "CmdBootBootloader",),
    (CmdReadGpio, "CmdReadGpio",),
    (CmdSetGpio, "CmdSetGpio",),
    (CmdReadAnalogGpio, "CmdReadAnalogGpio",),
    (CmdSetStepsPerMM, "CmdSetStepsPerMM",),
    (CmdSetHostSettings, "CmdSetHostSettings",),
    (CmdSystemReset, "CmdSystemReset",),
    (CmdGetTaskStatus, "CmdGetTaskStatus",),
    (CmdGetIOStats, "CmdGetIOStats",),
    (CmdSetBaudRate, "CmdSetBaudRate",),
    (CmdSetSlowDown, "CmdSetSlowDown",),
    (CmdGetVersion, "CmdGetVersion",),
    ]:

        insertCommandName(cmd, cmdName)

#
# Flag bits for CmdG1x commands
#
# Bits 0, 1, 2, 3, 4 reserved for direction bits
# DecelByteFlagBit     = (1 << 5)  # 0x20
# AccelByteFlagBit     = (1 << 6)  # 0x40
DirBitsBit           = (1 << 7)  # 0x80
MoveStartBit         = (1 << 8)  # 0x100
MeasureStartBit      = (1 << 9)  # 0x200
EndMoveBit           = (1 << 10) # 0x400
# CompressedBit        = (1 << 11) # 0x800
# Raw moves
MoveStartBitRaw      = (1 << 5) # 0x20
TimerByteFlagBit     = (1 << 6) # 0x40
DirBitsBitRaw        = (1 << 7) # 0x80
MeasureStartBitRaw   = (1 << 8) # 0100
EndMoveBitRaw        = (1 << 9) # 0200
# CompressedBitRaw     = (1 << 10) # 0400

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
# RespRXFullError =              10 # Payload: received len
RespUnderrun      =            13 # Payload: empty flag, size, message

ResponseNames = {
        RespUnknownCommand: "RespUnknownCommand",
        RespKilled: "RespKilled",
        RespRXError: "RespRXError",
        RespRXCRCError: "RespRXCRCError",
        RespACK: "RespACK",
        RespSerNumberError: "RespSerNumberError",
        RespRXTimeoutError: "RespRXTimeoutError",
        RespUnsolicitedMsg: "RespUnsolicitedMsg",
        # RespRXFullError: "RespRXFullError",
        RespUnderrun: "RespUnderrun",
}















############################################################################
