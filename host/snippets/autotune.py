#!/usr/bin/env python 
# encoding: utf-8 
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
# Auswertung sprungantwort nach der "T-Summen-Regel".
#

import matplotlib.pyplot as plt
from libautotune import *

import sys
import ddprintutil as util

f=open(sys.argv[1])
raw = util.jsonLoad(f)
# data = raw["data"]
d = raw["data"]

Xo = raw["Xo"]
data = []
for tim, temp in d:
    data.append((tim, temp/Xo))

# print "data:", data

plt.title('Raw temperature step response')
plt.grid(True)
plt.plot(map(lambda t: t[0], data), map(lambda t: t[1], data), '-')
plt.show()

# Zeit bis konstante endtemperatur erreicht wird:
endTime = getTEnd(data, raw["tEnd"]/Xo)
print "endTime: ", endTime

# Fläche für summen-regel integrieren:

aSum = sumArea(data, raw["tEnd"]/Xo, endTime)

# tSum2 = sumArea2(data, raw["tEnd"]/Xo, endTime)

#####################################################################
#
# Ks = Mu / Xo, Verstärkung der strecke
#
Ks = raw["tEnd"] / Xo
print "Ks: ", Ks

tSum = aSum / Ks
print "tSum:", tSum # , "tSum2:", tSum2

## # Kc or Kp
## Kc = (1.2 / Ks) * (T / Tdead)
## # Ti or Tn (Nachstellzeit)
## Ti = 2 * Tdead
## # Td or Tv (Vorhaltezeit)
## Td = 0.5 * Tdead

# Normal control
Kc = Kp = 1.0/Ks
Ti = Tn = 0.66 * tSum
Td = Tv = 0.167 * tSum

print "\nNormal: Kc;Kp %7.4f, Ti;Tn: %7.4f, Td;Tv: %7.4f" % (Kc, Ti, Td)
print "Normal: Kp %7.4f, Ki: %7.4f, Kd: %7.4f" % (Kc, Kc/Ti, Kc*Td)

# Fast control
Kc = Kp = 2.0/Ks
Ti = Tn = 0.8 * tSum
Td = Tv = 0.194 * tSum

print "\nFast: Kc;Kp %7.4f, Ti;Tn: %7.4f, Td;Tv: %7.4f" % (Kc, Ti, Td)
print "Fast: Kp %7.4f, Ki: %7.4f, Kd: %7.4f" % (Kc, Kc/Ti, Kc*Td)

