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
# Merge several calibration data sets by averaging.
#
import sys
import ddprintutil as util

inputs = []
l = None
for jsonfn in sys.argv[1:]:

    f=open(jsonfn)
    data = util.jsonLoad(f)["filSensorCalibration"]

    if l:
        assert(len(data) == l)
    else:
        l = len(data)

    inputs.append(data)

sums = []
index = map(lambda tup: tup[0], inputs[0])
for i in range(l):
    s = 0
    for data in inputs:
        (fr, cv) = data[i]
        assert(fr == index[i])
        s += cv
    sums.append(s)

l3 = []
for i in range(l):

    l3.append( (index[i], sums[i]/len(inputs)) )

print '{\n    "filSensorCalibration": ['
print ",\n".join(map(lambda tup: "        [%f, %f]" % tup, l3))
print "    ]"
print "}"

