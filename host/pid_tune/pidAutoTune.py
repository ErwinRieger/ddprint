#!/usr/bin/env python 
# encoding: utf-8 
#
#/*
# This file is part of ddprint - a 3D printer firmware.
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
import numpy as np
from libautotune import *

import sys
import ddprintutil as util, movingavg

f=open(sys.argv[1])
raw = util.jsonLoad(f)
d = raw["data"]

# Get timestamps, sampling frequency
rawXValues = np.array([x[0] for x in d], dtype=float) # [:,0]
rawYValues = np.array([x[1] for x in d], dtype=float)

times = np.diff(rawXValues)

# Normalize by input step
Xo = raw["Xo"]
normData = []
for tim, temp in d:
    normData.append((tim, temp/Xo))

rawTMax = np.max(rawYValues)

# Ks = raw["tEnd"]/Xo # Ks is also normalized endtemp
Ks = rawTMax / Xo # Ks is also normalized endtemp

print("# raw endtemp, endtemp:", raw["tEnd"], rawTMax, Ks)

t033 = getPercentT(normData, Ks/3.0)
t066 = getPercentT(normData, Ks*2/3.0)

print("##################################################")
print("## PID Parameter aus Sprungantwort              ##")
print("##################################################")
print("# Max time between samples: %.4f s" % np.max(times))
fs = 1.0 / np.mean(times)
print("# Mean sampling freq %.4f s, %.4f Hz" % (np.mean(times), fs))
print("# Ks,Kp is:", Ks)
print("# Tg/Tu nach Gregory Reeves, https://www.youtube.com/watch?v=4o4cqsu8JnE:")
# "Time Constant"
Tg1 = (t066-t033) / 0.7
# print '"Tg": %.4f'% Tg1
# "Dead Time"
Tu1 = t033 - 0.4*Tg1
# print '"Tu": %.4f' % Tu1

# print "\nTg/Tu nach http://controlguru.com/:"
# "Time Constant"
Tg2 = getPercentT(normData, Ks*0.632)
# print "Tg:", Tg2
# "Dead Time"
Tu2 = getFoptdTd(normData, Ks)
# print "Tu:", Tu2

# tangente über steigung

# print "\nTg/Tu über steigung"
nAvgShortterm = int(round(((Tu1+Tu2)/20.0) * fs))
# print "nAvgShortterm:", nAvgShortterm
tangentAvg = movingavg.MovingAvg(nAvgShortterm)

last = normData[0]
tangente = []
i = 0
while i < len(normData)-1:
    
    (t, y) = normData[i+1]

    dt = t - last[0]
    dy = y - last[1]

    s = dy / dt

    # print "s:", abs(s)
    tangentAvg.add(s)

    if tangentAvg.valid():
        tforavg = normData[i+1-nAvgShortterm//2][0]
        tangente.append((tforavg, s, tangentAvg.mean()))
    else:
        if (i < nAvgShortterm//2):
            tangente.append((t, s, 0))

    i += 1
    last = (t, y)

# max steigung tangente:
tts = np.array([x[0] for x in tangente], dtype=float)
ty = np.array([x[2] for x in tangente], dtype=float)

smax = np.max(ty)
idx = np.argmax(ty)
tempidx = normData[idx][1]
ttu = tts[idx] - (tempidx / smax)
ttg = tts[idx] + ((Ks - tempidx) / smax)

# print "max steigung:", smax, idx, tempidx
# print "Tu from tangent:", ttu
# print "Tg from tangent:", ttg

#end # tangente über steigung

# Mittelwert Tg, Tu:
print("# Average Tg/Tu:")
Tg = (Tg1+Tg2+ttg)/3.0
Tu = (Tu1+Tu2+ttu)/3.0
# print "Tu:", Tu
# print "Tg:", Tg
print('"Tu": %.4f,' % Tu)
print('"Tg": %.4f, "ZN": {'% Tg)

# print "\nZiegler PI:"
# Kr = (0.9 / Ks) * (Tg / Tu)
# Tn = 3.33 * Tu 
# print '"Kp": %.4f,' % Kr
# print '"Ki": %.4f,' % (Kr / Tn)

print("#Ziegler PID 1.2:")
Kr = (1.2 / Ks) * (Tg / Tu)
Tn = 2.0 * Tu 
Tv = 0.5 * Tu
print('"Kp": %.4f,' % Kr)
print('"Ki": %.4f,' % (Kr / Tn))
print('"Kd": %.4f} , "CH": {' % (Kr / Tv))

print("#Chien aperiodisch PID, gute führung:")
Kr = (0.6 / Ks) * (Tg / Tu) 
Tn = Tg 
Tv = 0.5 * Tu
print('"Kp": %.4f,' % Kr)
print('"Ki": %.4f,' % (Kr / Tn))
print('"Kd": %.4f},' % (Kr / Tv))

##################################################

#
# T-sum method, search the time Tsum where area below the curve
# before Tsum is the same as the area above behind Tsum.
#
areaA = 0.0
areaB = 0.0

inversY = np.array([rawTMax - x[1] for x in d], dtype=float)

for i in range(len(rawXValues)-1):
    areaA = np.trapz(y=rawYValues[:i], x=rawXValues[:i])
    # print "areaA:", areaA
    areaB = np.trapz(y=inversY[i:], x=rawXValues[i:])
    # print "areaB:", areaB

    if areaA >= areaB:
        tSum = rawXValues[i]
        break

print("\nT-Sum method PID parameters (fast):")
print("Tsum: ", tSum)
Kr = 2.0 / Ks
Tn = 0.8 * tSum
Tv = 0.194 * tSum
print('"Kp": %.4f,' % Kr)
print('"Ki": %.4f,' % (Kr / Tn))
print('"Kd": %.4f,' % (Kr / Tv))

# print "\nT-Sum method PI parameters (fast):"
# Kr = 1.0 / Ks
# Tn = 0.7 * tSum
# print '"Kp": %.4f,' % Kr
# print '"Ki": %.4f,' % (Kr / Tn)


##################################################

plt.subplot(3,1,1)
plt.title('Raw temperature step response')
plt.grid(True)
# plt.plot(map(lambda t: t[0], d), map(lambda t: t[1], d), '-')
plt.plot(rawXValues, [t[1] for t in d], '-')

plt.subplot(3,1,2)
plt.title('Tangente')
plt.grid(True)
plt.plot([t[0] for t in tangente], [t[2] for t in tangente], '-')
plt.plot((tts[idx], tts[idx]), (0, smax*1.1))

plt.subplot(3,1,3)
plt.title('Step response')
plt.grid(True)
plt.plot([t[0] for t in normData], [t[1] for t in normData], '-')
plt.plot([0, normData[-1][0]], [Ks, Ks])

plt.plot([t033, t033], [0, 0.4*Ks])
plt.annotate('1/3', xy=(t033*1.1, 0.3*Ks))

plt.plot([t066, t066], [0, 0.7*Ks])
plt.annotate('2/3', xy=(t066*1.05, 0.6*Ks))

plt.plot([Tg, Tg], [0, 1.1*Ks])
plt.annotate('Tg %.1f' % Tg, xy=(Tg*1.05, 1.1*Ks))

plt.plot([Tu, Tu], [0, 1.1*Ks])
plt.annotate('Tu %.1f' % Tu, xy=(Tu*1.05, 1.1*Ks))

plt.plot([Tu, Tg], [0, Ks])
plt.plot([Tu1, Tg1], [0, Ks])
plt.plot([Tu1, Tg2], [0, Ks])
plt.plot([ttu, ttg], [0, Ks])
# plt.annotate('Tg %.1f' % Tu, xy=(Tu*1.05, 1.1*Ks))

plt.show()





