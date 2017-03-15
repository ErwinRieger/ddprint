#!/usr/bin/env python 
# encoding: utf-8 
#
#/*
# This file is part of ddprint - a direct drive 3D printer firmware.
# 
# Copyright 2017 erwin.rieger@ibrieger.de
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

import matplotlib.pyplot as plt
from libautotune import *

import sys
import ddprintutil as util

fig = plt.figure()
ax = fig.add_subplot(111)

f=open(sys.argv[1])
raw = util.jsonLoad(f)
# data = raw["data"]
d = raw["data"]

Xo = raw["Xo"]
data = []
for tim, temp in d:
    data.append((tim, temp/Xo))

Ks = raw["tEnd"]/Xo

endTemp = raw["tEnd"]/Xo
t033 = getPercentT(data, endTemp/3.0)
t066 = getPercentT(data, endTemp*2/3.0)

print "##################################################"
print "## PID Parameter aus Sprungantwort              ##"
print "##################################################"

print "\nKs is:", Ks

print "\nTp/Td nach Gregory Reeves, https://www.youtube.com/watch?v=4o4cqsu8JnE:"
# "Time Constant"
Tp1 = (t066-t033) / 0.7
print "Tp:", Tp1
# "Dead Time"
Td1 = t033 - 0.4*Tp1
print "Td:", Td1

print "\nTp/Td nach http://controlguru.com/:"
# "Time Constant"
Tp2 = getPercentT(data, endTemp*0.632)
print "Tp:", Tp2
# "Dead Time"
Td2 = getFoptdTd(data, Ks)
print "Td:", Td2

# Mittelwert Tp, Td:
print "\nAverage Tp/Td:"
avgTp = (Tp1+Tp2)/2.0
avgTd = (Td1+Td2)/2.0
print "Tp:", avgTp
print "Td:", avgTd


print "\nmoderate PID values:     Tc is the larger of    1·Tp  or     8·Өp"
Tc = max(avgTp, 8*avgTd)
pidParameters(Ks, Tc, avgTp, avgTd)

agressive = 0.5
print "\n%f aggressive PID values:     Tc is the larger of    0.55·Tp  or     4.4·Өp" % agressive
Tc = max(agressive*avgTp, agressive*8*avgTd)
pidParameters(Ks, Tc, avgTp, avgTd)

agressive = 0.25
print "\n%f aggressive PID values:     Tc is the larger of    0.55·Tp  or     4.4·Өp" % agressive
Tc = max(agressive*avgTp, agressive*8*avgTd)
pidParameters(Ks, Tc, avgTp, avgTd)

agressive = 0.2
print "\n%f aggressive PID values:     Tc is the larger of    0.55·Tp  or     4.4·Өp" % agressive
Tc = max(agressive*avgTp, agressive*8*avgTd)
pidParameters(Ks, Tc, avgTp, avgTd)

agressive = 0.15
print "\n0.15 aggressive PID values:     Tc is the larger of    0.1·Tp  or     0.8·Өp"
Tc = max(agressive*avgTp, agressive*8*avgTd)
pidParameters(Ks, Tc, avgTp, avgTd)

print "\n0.1 aggressive PID values:     Tc is the larger of    0.1·Tp  or     0.8·Өp"
Tc = max(0.1*avgTp, 0.8*avgTd)
pidParameters(Ks, Tc, avgTp, avgTd)

plt.title('Raw temperature step response')
plt.grid(True)
plt.plot(map(lambda t: t[0], data), map(lambda t: t[1], data), '-')
plt.plot([0, data[-1][0]], [Ks, Ks])
plt.plot([t033, t033], [0, 0.4*Ks])
plt.plot([t066, t066], [0, 0.7*Ks])
plt.plot([avgTp, avgTp], [0, 1.1*Ks])
plt.plot([avgTd, avgTd], [0, 1.1*Ks])

ax.annotate('1/3', xy=(t033*1.1, 0.3*Ks))
ax.annotate('2/3', xy=(t066*1.05, 0.6*Ks))
plt.show()






