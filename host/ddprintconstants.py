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

Uint8Max = pow(2, 8) - 1
Uint16Max = pow(2, 16) - 1

############################################################################

# Maximum combined acceleration of the X, Y, Z and E axes 
_MAX_ACCELERATION =         3000    # [ mm/s^s ]

############################################################################
#
# Stepdata types
#
StepDataTypeBresenham = 0
StepDataTypeRaw       = 1

############################################################################
#
# Board config, move to printer-profile ?
#
fCPU =  16000000 # F_CPU, Hz
fTimer = 2000000.0
maxTimerValue16 = Uint16Max

############################################################################
#
# Axes
#
X_AXIS = 0
Y_AXIS = 1
Z_AXIS = 2
A_AXIS = 3
B_AXIS = 4

dimNames = ["X", "Y", "Z", "A", "B"]
dimIndex = { "X": X_AXIS, "Y": Y_AXIS, "Z": Z_AXIS, "A": A_AXIS, "B": B_AXIS }

dimBits = [0x1, 0x2, 0x4, 0x8, 0x10]
dimBitsIndex = { "X": 0x1, "Y": 0x2, "Z": 0x4, "A": 0x8, "B": 0x10 }

############################################################################
#
# Cobs encoding
#
SOH = 0x0 # 'Start of header', startbit for COBS encoded data block.

############################################################################
#
# Extrusion limit, tempTable. This must match the value in the
# firmware (ddprint.h).
#
NExtrusionLimit = 100

############################################################################
#
# Acceleration planning
#
# Threshold value, if the difference of the velocity at the start or end
# of a move is below this value, we do not generate a accel-/deceleration ramp .
AccelThreshold=0.001 # [mm/s]

############################################################################
#
# Extruder advance
#
# Threshold value, if the difference of the extrusion rate of two moves
# is below this value, we assume they have the same extrusion rate.
AdvanceEThreshold=0.000001 # [mm/s]

#
# Minimum len in mm of acceleration- or deceleration ramp to apply advance.
#
AdvanceMinRamp = 0.1

############################################################################
#
# Layer 0 hotend temperature increase
#
Layer0TempIncrease = 10

############################################################################
#
# Scalingfactor for temperatures, firmware uses fractions of °C
#
TempScale = 16

############################################################################
#
# To compute PID scaling factors, integer-math
#
PidFrequency = 10.0 # [Hz] PID is called every 0.1 seconds, must match firmware
PidPrecision = 10   # Number of bits

# PidKpScale = (float(pow(2, 14)) / TempScale)

############################################################################
#
# Homing
#
# When homing to zero (nozzle at printbed), this is the amount of z-lift
# after Z is homed.
HOMEZLIFT=50

# Todo: remove, unused
# MinExtrusionForMeasurement = 1.0



