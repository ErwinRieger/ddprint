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

CmdG1_24         = 0x9 # Same as CmdG1, but with 32 bit accel- and deccel-timervalues (vor very slow moves)
CmdDirG1_24      = 0xa # CmdDirBits and CmdG1_24 combined
CmdSyncTargetTemp= 0xb # Parameters: heater, temp 

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
CmdWriteEepromFloat = 135 # Parameters: valuename, value
CmdEepromFactory = 136 # Factory reset of eeprom values, bed leveling needed
CmdFanSpeed = 137

# Getters, they return a string of the form "ret: <expr>"
CmdGetState = 150
CmdGetHomed = 151
CmdGetEndstops = 152   # Get endstop state and pos
CmdGetEepromVersion = 153
CmdGetEepromSettings = 154
CmdDisableStepperIsr = 155
CmdDisableSteppers = 156
CmdGetTemps = 157
CmdGetTargetTemps = 158
CmdGetPos = 159
CmdExit = 160
CmdGetStatus = 161

#
#
#
#
#
# Drucker koordinatensystem:
# ----------------------------
#
# X: positive richtung nach rechts, endstop in MIN richtung
# Y: positive richtung nach hinten, endstop in MAX richtung
# Z: positive richtung nach unten, endstop in MAX richtung
#
#
#
#
#
#
#
#
#
#
#
#

CommandNames = {
    CmdNull: "CmdNull",
    CmdG1: "CmdG1",
    CmdG1_24: "CmdG1_24",
    CmdDirBits: "CmdDirBits",
    CmdBlock: "CmdBlock",
    CmdDirG1: "CmdDirG1",
    CmdDirG1_24: "CmdDirG1_24",
    CmdSyncTargetTemp: "CmdSyncTargetTemp",
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
    CmdRaw: "CmdRaw",
    
    # Getters, they return a string of the form "ret: <expr>"
    CmdGetState: "CmdGetState",
    CmdGetHomed: "CmdGetHomed",
    CmdGetEndstops: "CmdGetEndstops",
    CmdGetEepromVersion: "CmdGetEepromVersion",
    CmdGetEepromSettings: "CmdGetEepromSettings",
    CmdDisableStepperIsr: "CmdDisableStepperIsr",
    CmdDisableSteppers: "CmdDisableSteppers",
    CmdGetTemps: "CmdGetTemps",
    CmdGetTargetTemps: "CmdGetTargetTemps",
    CmdGetPos: "CmdGetPos",
    CmdExit: "CmdExit",
    CmdGetStatus: "CmdGetStatus",
}


############################################################################
