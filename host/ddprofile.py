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

import math, os, types
import ddprintutil as util

from ddprintconstants import dimNames, A_AXIS, B_AXIS

####################################################################################################
#
# Json profile base class
#
####################################################################################################
class ProfileBase(object):

    def __init__(self, cls, name, specificName=""):

        if cls._single:
            raise RuntimeError('A ' + str(cls) + ' instance already exists')

        self.name = name

        f = self.openJson(name)

        self.values = util.jsonLoad(f)

        # Let specific profile overwrite values from a generic profile
        if specificName:
            f = self.openJson(specificName)
            specificValues = util.jsonLoad(f)
            for k in specificValues.keys():
                self.values[k] = specificValues[k]

        cls._single = self

    def getValue(self, valueName):
        try:
            return self.values[valueName]
        except KeyError:
            print "\nERROR: Profile ", self.name, " does not conain key:", valueName, "\n"
            raise

    def openJson(self, name):

        try:
            f = open(name + ".json")
            return f
        except IOError:
            pass

        try:
            f = open(name)
            return f
        except IOError:
            pass

        try:
            f = open(os.path.join("mat-profiles", name) + ".json")
            return f
        except IOError:
            pass

        try:
            f = open(os.path.join("mat-profiles", name))
            return f
        except IOError:
            pass

        print "\nERROR: Profile ", name, " not found"
        assert(0)

####################################################################################################
#
# Printer profile, singleton
#
####################################################################################################
class PrinterProfile(ProfileBase):

    _single = None 

    def __init__(self, name):

        super(PrinterProfile, self).__init__(PrinterProfile, name)

    @classmethod
    def get(cls):
        return cls._single

    @classmethod
    def overrideEJerk(cls, jerk):
        cls.get().values["axes"]["A"]["jerk"] = jerk
        cls.get().values["axes"]["B"]["jerk"] = jerk

    @classmethod
    def overrideEAccel(cls, accel):
        cls.get().values["MaxAxisAcceleration"][A_AXIS] = accel
        cls.get().values["MaxAxisAcceleration"][B_AXIS] = accel

    @classmethod
    def getValues(cls):
        return cls.get().values

    @classmethod
    def getStepsPerMM(cls, axisNr):
        return cls.getValues()["axes"][dimNames[axisNr]]["steps_per_mm"]

    @classmethod
    def getStepsPerMMVector(cls):
        return map(lambda d: cls.getStepsPerMM(d), range(5))

    @classmethod
    def getMaxFeedrate(cls, axisNr):
        return cls.getValues()["axes"][dimNames[axisNr]]["max_feedrate"]

    @classmethod
    def getMaxFeedrateVector(cls):
        return map(lambda d: cls.getMaxFeedrate(d), range(5))

    @classmethod
    def getRetractFeedrate(cls):
        return cls.getValues()["RetractFeedrate"]

    @classmethod
    def getRetractLength(cls):
        return float(cls.getValues()["RetractLength"])

    @classmethod
    def getMaxAxisAcceleration(cls):
        accel = cls.getValues()["MaxAxisAcceleration"]
        return accel

####################################################################################################
#
# To access tempearture curve data
#
####################################################################################################
class TempCurve:

    def __init__(self, curveList):

        self.curveList = curveList
        
        (minTemp, minFlowrate) = curveList[0]
        self.minTemp = int(minTemp)
        self.minFlowrate = int(minFlowrate*10)

        (maxTemp, maxFlowrate) = curveList[-1]
        self.maxTemp = int(maxTemp)
        self.maxFlowrate = int(maxFlowrate*10)

        # Init dict for faster lookup
        self.tempCurveDict = {}
        for temp, frate in curveList:

            key = int(frate*10)
            if key not in self.tempCurveDict:
                self.tempCurveDict[key] = int(temp)

        print"tempDict:", self.tempCurveDict

    def lookupFlowrate(self, flowrate):

        key = int(flowrate*10)
        if key in self.tempCurveDict:
            return self.tempCurveDict[key]

        if key <= self.minFlowrate:
            return self.minTemp

        if key >= self.maxFlowrate:
            return self.maxTemp

        print "flowrate not found:", key

        # Linear interpolation
        for i in range(len(self.curveList)-1):
            temp1, fr1 = self.curveList[i]
            temp2, fr2 = self.curveList[i+1]

            k1 = int(fr1*10)
            k2 = int(fr2*10)
            if key > k1 and key < k2:
                print "found range:", k1, key, k2

                t = int( temp1 + ((temp2 - temp1) / (k2 - k1)) * (key - k1) )

                print "interplated temp: ", t
                self.tempCurveDict[key] = t
                return t


        assert(0)

    # Used for temp-table download, so this is not time-critical
    # and therefore not optimized.
    def lookupTemp(self, temp):

        assert(type(temp) == types.IntType)

        if temp <= self.minTemp:
            print "match mintemp:", temp, self.minTemp
            return self.minFlowrate / 10

        if temp >= self.maxTemp:
            print "match maxtemp:", temp, self.maxTemp
            return self.maxFlowrate / 10

        print "temp not found:", temp

        # Linear interpolation
        for i in range(len(self.curveList)-1):
            temp1, fr1 = self.curveList[i]
            temp2, fr2 = self.curveList[i+1]

            t1 = int(temp1)
            t2 = int(temp2)

            if temp >= t1 and temp <= t2:

                fr = int( fr1 + ((fr2 - fr1) / (t2 - t1)) * (temp - t1) )

                print "interplated feedrate: ", fr
                return fr

        assert(0)

####################################################################################################
#
# Material profile, singleton
#
####################################################################################################
class MatProfile(ProfileBase):

    _single = None 

    def __init__(self, name, smatName):

        super(MatProfile, self).__init__(MatProfile, name, smatName)

        self.matArea = (math.pi * pow(float(self.values["material_diameter"]), 2)) / 4.0

        self.tempCurves = {}

    def override(self, key, value):
        assert(key != "material_diameter")
        self.values[key] = value

    @classmethod
    def get(cls):
        return cls._single

    @classmethod
    def getValues(cls):
        return cls.get().values

    @classmethod
    def getMatDiameter(cls):
        return float(cls.getValues()["material_diameter"])

    # XXX use first entry of temptable here...
    @classmethod
    def getHotendBaseTemp(cls):
        return int(cls.getValues()["hotendBaseTemp"])

    @classmethod
    def getHotendMaxTemp(cls):
        return int(cls.getValues()["hotendMaxTemp"])

    @classmethod
    def getBedTemp(cls):
        return int(cls.getValues()["bedTemp"])

    @classmethod
    def getBedTempReduced(cls):
        return int(cls.getValues()["bedTempReduced"])

    @classmethod
    def getFanPercent(cls):
        return int(cls.getValues()["fanPercent"])

    @classmethod
    def getFlow(cls):
        flow = float(cls.getValues()["flow"])
        assert((flow >= 50) and (flow <= 150))
        return flow

    @classmethod
    def getMatArea(cls):
        return cls.get().matArea

    # @classmethod
    # def getBaseExtrusionRate(cls, nozzleDiam):
        # return float(cls.getValues()["baseExtrusionRate_%d" % int(nozzleDiam * 100)])

    # @classmethod
    # def getAutoTempFactor(cls):
        # return float(cls.getValues()["extrusionAutoTempFactor"])

    @classmethod
    def getKAdv(cls):
        return float(cls.getValues()["kAdvance"])

    @classmethod
    def getKFeederCompensation(cls):
        return float(cls.getValues()["kFeederCompensation"])

    @classmethod
    def getTempForFlowrate(cls, flowrate, nozzleDiam):
        return cls.get()._getTempForFlowrate(flowrate, nozzleDiam)

    def _getTempForFlowrate(self, flowrate, nozzleDiam):

        if nozzleDiam not in self.tempCurves:

            tempCurve = TempCurve(self.getValues()["tempFlowrateCurve_%d" % (nozzleDiam*100)])
            self.tempCurves[nozzleDiam] = tempCurve

        tempCurve = self.tempCurves[nozzleDiam]
        return tempCurve.lookupFlowrate(flowrate)

    @classmethod
    def getFlowrateForTemp(cls, temp, nozzleDiam):
        return cls.get()._getFlowrateForTemp(temp, nozzleDiam)

    def _getFlowrateForTemp(self, temp, nozzleDiam):

        if nozzleDiam not in self.tempCurves:

            tempCurve = TempCurve(self.getValues()["tempFlowrateCurve_%d" % (nozzleDiam*100)])
            self.tempCurves[nozzleDiam] = tempCurve

        tempCurve = self.tempCurves[nozzleDiam]
        return tempCurve.lookupTemp(temp)

####################################################################################################
#
# Nozzle profile, singleton
#
####################################################################################################
class NozzleProfile(ProfileBase):

    _single = None 

    def __init__(self, name):

        super(NozzleProfile, self).__init__(NozzleProfile, name)

        # Compute max extrusion rate without extrusion adjust
        extrusionRateAdjust = float(self.getValue("extrusionRateAdjust"))
        self.netMaxExtrusionRate = NozzleProfile.getMaxExtrusionRate() / (1 + extrusionRateAdjust/100)
        print "Max net extrusion rate:", self.netMaxExtrusionRate
        # Extrusion adjust factor
        self.kExtrusionAdjust = (extrusionRateAdjust/100) / self.netMaxExtrusionRate
        print "kExtrusionAdjust: ", self.kExtrusionAdjust

    
    @classmethod
    def get(cls):
        return cls._single

    @classmethod
    def getValues(cls):
        return cls.get().values

    @classmethod
    def getSize(cls):
        return float(cls.getValues()["size"])

    @classmethod
    def getMaxExtrusionRate(cls):
        return float(cls.getValues()["maxExtrusionRate"])

    @classmethod
    def getExtrusionAdjustFactor(cls):
        return cls.get().kExtrusionAdjust

    @classmethod
    def getNetMaxExtrusionRate(cls):
        return cls.get().netMaxExtrusionRate

    @classmethod
    def getArea(cls):
        return (math.pi/4) * pow(cls.getSize(), 2)























