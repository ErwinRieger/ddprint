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

def getFoptdTd(data, temp):

    start = temp * 0.01
    return getPercentT(data, start)

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



