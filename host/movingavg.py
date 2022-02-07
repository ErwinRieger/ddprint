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

#########################################################################################

class MovingAvg:

    def __init__(self, navg, startavg=0.0):
        self.index = 0
        self.navg = navg
        self.array = np.array([startavg] * navg)
        self.nValues = 0
        self.startavg = startavg

    def add(self, v):

        self.array[self.index] = v
        self.index += 1
        if self.index == self.navg:
            self.index = 0

        if self.nValues < self.navg:
            self.nValues += 1

    def mean(self):
        return float(np.mean(self.array)) # avoid NP-float behaviour when rounding

    def preload(self, steps):
        self.array = [float(steps)] * self.navg

    def trim(self, navg):

        if navg < self.navg:

            array = np.array([0.0] * navg)

            for i in range(navg):
                if (self.index - (i+1)) >= 0:
                    array[i] = self.array[self.index - (i+1)]
                else:
                    array[i] = self.array[self.index - (i+1) + self.navg]

            self.array = array
            self.index = 0
            self.navg = navg

        if navg > self.navg:

            array = np.array([0.0] * navg)

            if self.nValues:

                for i in range(self.nValues):
                    array[i] = self.array[i];

                avg = self.mean()
                for i in range(navg - self.nValues):
                    array[self.nValues+i] = avg

            else:

                for i in range(navg):
                    array[self.navg+i] = self.startavg

            self.array = array
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

    def addReadings(self, readings, fscal):

        for (ds, dy) in readings:

            ratio = dy / (ds * fscal)
            self.add(ratio)


#########################################################################################

class CrossingAverage:

    def __init__(self):

        self.started = False
        self.converged = None

        self.nLongPeriod = 0
        self.avgLong = MovingAvg(0)
        self.avgShort = MovingAvg(0)

    def getNValues(self):
        return self.avgLong.nValues

    def addReadings(self, readings, fscal):

        cap = self.getNValues()+len(readings)
        if not self.started and self.nLongPeriod < cap:
            self.setNLong(cap)

        for (ds, dy) in readings:

            ratio = dy / (ds * fscal)
            # print "ds, dy, ratio: ", ds, dy, ratio
            self.avgShort.add(ratio)
            self.avgLong.add(ratio)

        if self.started and self.getNValues() >= self.nLongPeriod and not self.converged:

            meanShort = self.avgShort.mean()
            meanLong = self.avgLong.mean()
            if abs((meanLong / meanShort) - 1.0) < 0.001:
                # print "avg converged: ", meanShort, meanLong, (meanLong / meanShort)-1.0
                self.converged = True

    def longAvg(self):

        if self.getNValues():
            return self.avgLong.mean()
        return 0.0

    def shortAvg(self):

        if self.getNValues():
            return self.avgShort.mean()
        return 0.0

    def setNLong(self, nLongPeriod):

        if self.nLongPeriod != nLongPeriod:

            self.nLongPeriod = nLongPeriod
            nShortPeriod = max(int(round(nLongPeriod / 8.0)), 2)

            # print("# of samples per round (long period):", nLongPeriod)
            # print("# of samples short period:", nShortPeriod)

            self.avgLong.trim(nLongPeriod)
            self.avgShort.trim(nShortPeriod)
            self.avgLong.preload(self.shortAvg())
            self.avgShort.preload(self.longAvg())

#########################################################################################


