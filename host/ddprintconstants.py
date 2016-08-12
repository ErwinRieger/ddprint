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
# Maximum acceleration of each axis X, Y, Z, (A/B/E)
#define DEFAULT_MAX_ACCELERATION      {9000,9000,100,10000}
MAX_AXIS_ACCELERATION = [9000, 9000, 100, 10000, 10000] # [ mm/s^s ]

# Maximum combined acceleration of the X, Y, Z and E axes 
#define DEFAULT_ACCELERATION          3000
MAX_ACCELERATION =         3000    # [ mm/s^s ]

#define DEFAULT_RETRACT_ACCELERATION  3000   // E max acceleration in mm/s^2 for retracts

# const unsigned int dropsegments=5; //everything with less than this number of steps will be ignored as move and joined with the next movement
DropSegments = 5          # steps


############################################################################
#
# Priming
#
#// number of mm^3 of plastic to extrude when priming
#// (Ultimaker 2 hot end capacity is approx 80 mm^3)
# PRIMING_MM3	= 50
PRIMING_MM3	= 60 # increased because of end-of-print retraction

#// Rate at which to prime head (in mm^3/s)
#// (Ultimaker 2 upper limit is 8-10)
PRIMING_MM3_PER_SEC = 5

#// Height at which to perform the priming extrusions
PRIMING_HEIGHT = 20


############################################################################
#
# Board config, move to printer-profile ?
#
fTimer = 2000000.0 # Hz
maxTimerValue16 = pow(2, 16)
maxTimerValue24 = pow(2, 24)

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

dimIndex = { "X": 0, "Y": 1, "Z": 2, "A": 3, "B": 4 }

############################################################################
#
# Cobs encoding
#
SOH = 0x0 # COBS encoding

############################################################################
#
# Extrusion limit, tempTable
#
NExtrusionLimit = 40
# ExtrusionLimitBaseTemp = 190




