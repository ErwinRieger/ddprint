#!/usr/bin/python
# -*- coding: utf-8 -*-
#
#/*
# This file is part of ddprint - a 3D printer firmware.
# 
# Copyright 2020 erwin.rieger@ibrieger.de
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


import sys, math
import numpy as np
from argparse import Namespace


EPSILON = 0.000001

#########################################################################################

class MovingAvg:

    def __init__(self, navg, startavg=0.0):
        self.index = 0
        self.navg = navg
        self.array = np.array([startavg] * navg)
        self.nValues = 0

    def add(self, v):

        self.array[self.index] = v
        self.index += 1
        if self.index == self.navg:
            self.index = 0

        self.nValues += 1

    def mean(self):
        return np.mean(self.array)

    def preload(self, steps):
        self.array = [float(steps)] * self.navg

    def trim(self, navg):

        assert(navg <= self.navg)

        # self.array = self.array[self.navg - navg:]

        array = np.array([0.0] * navg)
        for i in range(navg):
            if (self.index - (i+1)) >= 0:
                array[i] = self.array[self.index - (i+1)]
            else:
                array[i] = self.array[self.index - (i+1) + self.navg]

        self.array = array
        self.index = 0
        self.navg = navg

    def expand(self, navg):

        assert(navg >= self.navg)

        lastIndex = self.index - 1
        if lastIndex < 0:
            lastIndex += self.navg;

        lastValue = self.array[lastIndex]

        self.array = np.append(self.array, np.array([lastValue]))
        self.navg = navg

    def valid(self):
        return self.nValues >= self.navg

    def near(self, rel, avg=None):

        if avg == None:
            avg = self.mean()

        for v in self.array:
            if abs(avg - v) > (avg * rel * 0.5):
                return False

        return True

class MovingAvgReadings(MovingAvg):

    def __init__(self, navg, startavg=0.0):
        MovingAvg.__init__(self, navg, startavg)

        # dict of all used timestamps to filter duplicates
        # from printer.getFSReadings()
        # self.data = {}

    def addReadings(self, readings, fscal):

        for (ds, dy) in readings:

            # if ts in self.data:
                # continue

            # self.data[ts] = dy

            ratio = dy / (ds * fscal)
            print "ds, dy, ratio: ", ds, dy, ratio
            self.add(ratio)


#########################################################################################

# https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect

# Given three colinear points p, q, r, the function checks if
# point q lies on line segment 'pr'
def onSegment(p, q, r):

    if q.x <= max(p.x, r.x) and q.x >= min(p.x, r.x) and q.y <= max(p.y, r.y) and q.y >= min(p.y, r.y):
       return True

    return False

# To find orientation of ordered triplet (p, q, r).
# The function returns following values
# 0 --> p, q and r are colinear
# 1 --> Clockwise
# 2 --> Counterclockwise
def orientation(p, q, r):

    # See 10th slides from following link for derivation of the formula
    # http://www.dcs.gla.ac.uk/~pat/52233/slides/Geometry1x1.pdf
    val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)

    # if (val == 0) return 0  // colinear
    if abs(val) < EPSILON:
        return 0  # colinear

    # return (val > 0)? 1: 2 // clock or counterclock wise
    if val > 0:
        return 1
    return 2

# The main function that returns true if line segment 'p1q1'
# and 'p2q2' intersect.
def doIntersect(p1, q1, p2, q2):

    # Find the four orientations needed for general and
    # special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if o1 != o2 and o3 != o4:
        return True

    # Special Cases
    # p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if o1 == 0 and onSegment(p1, p2, q1): return True

    # p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if o2 == 0 and onSegment(p1, q2, q1): return True

    # p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if o3 == 0 and onSegment(p2, p1, q2): return True

    # p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if o4 == 0 and onSegment(p2, q1, q2): return True

    return False # Doesn't fall in any of the above cases

#########################################################################################
# https://dsp.stackexchange.com/questions/4886/zero-crossing-of-a-noisy-sine-wave

class CrossingAverage:

    def __init__(self, nLongPeriod):

        self.started = False
        self.locked = None

        self.nLongPeriod = nLongPeriod
        nShortPeriod = max(int(round(nLongPeriod / 8.0)), 2)

        print "# of samples per round (long period):", nLongPeriod
        print "# of samples short period:", nShortPeriod

        self.avgLong = MovingAvg(nLongPeriod)
        self.avgShort = MovingAvg(nShortPeriod)

        self.nValues = 0

        self.lastMeanLong = 0
        self.lastMeanShort = 0
        self.lastTime = 0

        # dict of all used timestamps to filter duplicates
        # from printer.getFSReadings()
        self.data = {}

    def longAvg(self):
        return self.avgLong.mean()

    def shortAvg(self):
        return self.avgShort.mean()

    def addValue(self, t, v):

        if t in self.data:
            return

        self.data[t] = v

        self.avgLong.add(v)
        self.avgShort.add(v)

        meanShort = self.avgShort.mean()
        meanLong = self.avgLong.mean()

        if not self.started and abs(meanShort) < 1.0:
            # don't count zero startup values
            # print "skip startup value:", t, v
            return

        self.started = True

        if (self.nValues+1) >= self.nLongPeriod and not self.locked:

            # if abs(meanLong - meanShort) < EPSILON:
                # self.locked = (self.nValues, t, meanShort)
                # return

            p1 = Namespace(x=self.lastTime, y=self.lastMeanShort)
            q1 = Namespace(x=t, y=meanShort)
            p2 = Namespace(x=self.lastTime, y=self.lastMeanLong)
            q2 = Namespace(x=t, y=meanLong)
            if doIntersect(p1, q1, p2, q2):
                self.locked = (self.nValues+1, t, meanShort, meanLong)

        self.nValues += 1
        self.lastMeanLong = meanLong
        self.lastMeanShort = meanShort
        self.lastTime = t

    def setNLong(self, nLongPeriod):

        self.locked = None
        self.nLongPeriod = nLongPeriod
        nShortPeriod = max(int(round(nLongPeriod / 8.0)), 2)

        print "# of samples per round (long period):", nLongPeriod
        print "# of samples short period:", nShortPeriod

        self.avgLong.trim(nLongPeriod)
        self.avgShort.trim(nShortPeriod)
        self.avgShort.preload(self.longAvg() * 1.05)

#########################################################################################


if __name__ == "__main__":

    f = open(sys.argv[1])
    exec(f.read())

    if "dataset" in locals():

        tround = (dFeederWheel*math.pi) / feedrate
        print "time for one rev:", tround

        # Average sample interval
        dtMean = np.mean(np.diff(np.array(map(lambda x: x[0], dataset)))) / 1000.0
        print "dt mean:", dtMean

        nLongPeriod = int(round(tround / dtMean))

        # x = np.array(map(lambda x: x[1], dataset), dtype=int) # [:,0]
        # median = np.median(x)
        # while dataset[0][1] < median:
            # del dataset[0]

        dataset = dataset[:2*nLongPeriod]

        t = np.array(map(lambda x: x[0], dataset), dtype=int) # [:,0]
        x = np.array(map(lambda x: x[1], dataset), dtype=int) # [:,0]

    else:
        # use *ts* and *y* data

        tround = (dFeederWheel*math.pi) / feedrate
        print "time for one rev:", tround

        # Average sample interval
        dtMean = np.mean(np.diff(np.array(ts))) / 1000.0
        print "dt mean:", dtMean

        nLongPeriod = int(round(tround / dtMean))

        # median = np.median(y)
        # while y[0] < median:
            # del ts[0]
            # del y[0]

        ts = ts[:2*nLongPeriod]
        y = y[:2*nLongPeriod]

        t = np.array(ts, dtype=int)
        x = np.array(y, dtype=int) 

    crossingAvg = CrossingAverage(nLongPeriod)

    ba = np.empty(np.size(x), dtype=float)
    la = np.empty(np.size(x), dtype=float)

    for i in range(np.size(x)):

        crossingAvg.addValue(t[i], x[i])

        ba[i] = crossingAvg.shortAvg()
        la[i] = crossingAvg.longAvg()

    assert(crossingAvg.locked)

    avg = crossingAvg.longAvg() 
    meanShort = crossingAvg.locked[2]
    meanLong = crossingAvg.locked[3]
    print "avg lock short at t:", crossingAvg.locked[1], meanShort, avg, "%.1f%%" % (((meanShort/avg)-1.0)*100)
    print "avg lock long  at t:", crossingAvg.locked[1], meanLong, avg, "%.1f%%" % (((meanLong/avg)-1.0)*100)

    import matplotlib.pyplot as plt

    plot1 = plt.subplot(3,1,1)
    plt.title(sys.argv[1])
    plt.xlabel('t [ms]')
    plt.ylabel('raw data vs long avg [counts]')
    plt.grid()
    plt.axvline(x=crossingAvg.locked[1], color='r', linestyle='-')
    plt.plot(t, la)
    plt.plot(t, ba)
    plt.plot(t, x)
    plt.ylim(0)


    plt.subplot(2,1,2, sharex=plot1)
    plt.xlabel('t [ms]')
    plt.ylabel('short avg vs long avg [counts]')
    plt.grid()
    plt.axhline(y=avg, color='r', linestyle='-')
    plt.axvline(x=crossingAvg.locked[1], color='r', linestyle='-')
    plt.plot(t, la)
    plt.plot(t, ba)

    plt.show()

