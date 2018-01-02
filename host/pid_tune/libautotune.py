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

def getTEnd(data, temp):

    nTemp = 0
    for (tim, Y) in data:

        if Y >= temp:
            nTemp += 1

        if nTemp == 10:
            return tim

    # constant phase not reached?
    assert(0)

def getFoptdTd(data, temp):

    start = temp * 0.01
    return getPercentT(data, start)

def sumArea(data, endTemp, endTime):

    aSum = 0
    lastTime = data[0][0]
    lastY = data[0][1]

    for (tim, Y) in data[1:]:

        if tim > endTime:
            return aSum

        aSum += (tim - lastTime) * (endTemp - lastY)

        lastTime = tim
        lastY = Y

    # constant phase not reached?
    assert(0)

def sumArea2(data, endTemp, endTime):

    print "Computing sumArea 2..."

    def sumA(endIndex):

        aSum = 0
        lastTime = data[0][0]
        lastY = data[0][1]
        i = 1
        while True:

            (tim, Y) = data[i]

            if i == endIndex:
                return aSum

            aSum += (tim - lastTime) * lastY

            lastTime = tim
            lastY = Y
            i += 1

        assert(0)

    def sumB(startIndex):

        aSum = 0
        lastTime = data[startIndex][0]
        lastY = data[startIndex][1]

        i = startIndex + 1
        while True:

            (tim, Y) = data[i]

            if tim > endTime:
                return aSum

            aSum += (tim - lastTime) * (endTemp - lastY)

            lastTime = tim
            lastY = Y
            i += 1

        assert(0)

    index = 1
    while True:

        sa = sumA(index)
        sb = sumB(index)

        ds = sa - sb
        # print "sa - sb:", sa - sb

        if ds >= 0:
            return data[index][0]

        index += 1

def getPercentT(data, temp):

    tim1 = 0
    tim2 = 0
    for (tim, Y) in data:

        if Y >= temp:
            tim1 = tim
            break

    i = len(data)-1
    while i >= 0:

        (tim, Y) = data[i]

        if Y <= temp:
            tim2 = tim
            break

        i -= 1

    # print "t1, t2", tim1, tim2
    return (tim1+tim2) / 2.0


# PID parameters as of http://controlguru.com/pid-control-of-the-heat-exchanger/
def pidParameters(Ks, Tc, Tp, Td):

    print "Ks,Kp:", Ks, "Tc:", Tc

    Kc = 1/Ks * (Tp + 0.5*Td) / (Tc + 0.5*Td)
    print "Kc: %f (Kc,independent == Kc,ideal" % Kc

    Ti = Tp + 0.5*Td
    print "Ti,ideal:", Ti
    print "Ki,independent:", Kc/Ti

    Td = (Tp * Td) / (2*Tp + Td)
    print "Td,ideal:", Td
    print "Kd,independent:", Kc*Td

    print "\n./ddprint.py writeEepromFloat Kp %f; ./ddprint.py writeEepromFloat Ki %f; ./ddprint.py writeEepromFloat Kd %f; ./reset.sh" % (Kc, Kc/Ti, Kc*Td)






