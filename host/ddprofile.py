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

    def getBaseName(self):
        return os.path.basename(self.name)

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

    def override(self, key, value):
        self.values[key] = value

####################################################################################################
#
# Printer profile, singleton
#
####################################################################################################
class PrinterProfile(ProfileBase):

    _single = None 

    def __init__(self, name):

        super(PrinterProfile, self).__init__(PrinterProfile, name)

        self.calTable = self.getValues()["filSensorCalibration"]
        self.minFlowRate = self.calTable[0][0]
        self.maxFlowRate = self.calTable[-1][0]

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

    def getFlowrateFromSensorRate(self, fr):

        if fr < self.minFlowRate:
            # print "minflowrate:", fr, self.calTable[0][1], fr * self.calTable[0][1]
            return fr * self.calTable[0][1]

        if fr >= self.maxFlowRate:
            # print "maxflowrate:", fr, self.calTable[-1][1], fr * self.calTable[-1][1]
            return fr * self.calTable[-1][1]

        index = int(fr/0.5) - 1

        (fr1, f1) = self.calTable[index]
        (fr2, f2) = self.calTable[index+1]

        print "fr1, fr, fr2", fr1, fr, fr2

        assert(fr >= fr1 and fr < fr2)

        dx = fr2 - fr1
        assert(dx == 0.5)

        dy = f2 - f1
        a = dy / dx

        f = f1 + a * (fr-fr1)

        print "a, f1, f, f2", a, f1, f, f2
        print "interpol:", fr, f, fr*f
        return fr*f

    @classmethod
    def getHwVersion(cls):
        return cls.getValues()["hwVersion"]

####################################################################################################
#
# To access tempearture curve data
#
####################################################################################################
class TempCurve:

    def __init__(self, curveList):

        self.curveList = curveList
        
        (minTemp, minFlowrate) = curveList[0]
        self.minTemp = minTemp
        # self.minFlowrate = int(minFlowrate*10)
        self.minFlowrate = minFlowrate

        (maxTemp, maxFlowrate) = curveList[-1]
        self.maxTemp = maxTemp
        # self.maxFlowrate = int(maxFlowrate*10)
        self.maxFlowrate = maxFlowrate

        # Init dict for faster lookup
        self.tempCurveDict = {}
        for temp, frate in curveList:

            key = int(frate*10)
            if key not in self.tempCurveDict:
                self.tempCurveDict[key] = temp

    def lookupFlowrate(self, flowrate):

        key = int(flowrate*10)
        if key in self.tempCurveDict:
            return self.tempCurveDict[key]

        if flowrate <= self.minFlowrate:
            return self.minTemp

        if flowrate >= self.maxFlowrate:
            return self.maxTemp

        print "flowrate not found:", flowrate, key

        # Linear interpolation
        for i in range(len(self.curveList)-1):

            temp1, fr1 = self.curveList[i]
            temp2, fr2 = self.curveList[i+1]

            if flowrate >= fr1 and flowrate < fr2 :
                # print "found range:", k1, key, k2
                print "found range:", fr1, flowrate, fr2

                dx = fr2 - fr1
                dy = temp2 - temp1

                t = temp1 + (dy / dx) * (flowrate - fr1)

                print "interplated temp: ", t
                self.tempCurveDict[key] = t
                return t


        assert(0)

    # Lookup allowed flowrate for a given temp.
    # Used for temp-table download, so this is not time-critical
    # and therefore not optimized.
    def lookupTemp(self, temp):

        if temp <= self.minTemp:
            print "match mintemp:", temp, self.minTemp
            return self.minFlowrate

        if temp >= self.maxTemp:
            print "match maxtemp:", temp, self.maxTemp
            return self.maxFlowrate

        print "temp not found:", temp

        # Linear interpolation
        for i in range(len(self.curveList)-1):

            temp1, fr1 = self.curveList[i]
            temp2, fr2 = self.curveList[i+1]

            if temp >= temp1 and temp < temp2:

                fr = fr1 + ((fr2 - fr1) / (temp2 - temp1)) * (temp - temp1)

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
        ProfileBase.override(self, key, value)

    @classmethod
    def get(cls):
        return cls._single

    @classmethod
    def getValues(cls):
        return cls.get().values

    @classmethod
    def getMatDiameter(cls):
        return float(cls.getValues()["material_diameter"])

    @classmethod
    def getHotendBaseTemp(cls, hwVersion, nozzleDiam):
        return  cls.get().getTempCurve(hwVersion, nozzleDiam).minTemp

    @classmethod
    def getHotendStartTemp(cls):
        return float(cls.getValues()["hotendStartTemp"])

    @classmethod
    def getHotendMaxTemp(cls):
        return float(cls.getValues()["hotendMaxTemp"])

    @classmethod
    def getBedTemp(cls):
        return float(cls.getValues()["bedTemp"])

    @classmethod
    def getBedTempReduced(cls):
        return float(cls.getValues()["bedTempReduced"])

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

    @classmethod
    def getKAdv(cls):
        return float(cls.getValues()["kAdvance"])

    # @classmethod
    # def getKFeederCompensation(cls):
        # return float(cls.getValues()["kFeederCompensation"])

    @classmethod
    def getTempForFlowrate(cls, flowrate, hwVersion, nozzleDiam):
        return cls.get()._getTempForFlowrate(flowrate, hwVersion, nozzleDiam)

    def _getTempForFlowrate(self, flowrate, hwVersion, nozzleDiam):

        tempCurve = self.getTempCurve(hwVersion, nozzleDiam)
        return tempCurve.lookupFlowrate(flowrate)

    @classmethod
    def getFlowrateForTemp(cls, temp, hwVersion, nozzleDiam):
        return cls.get()._getFlowrateForTemp(temp, hwVersion, nozzleDiam)

    def _getFlowrateForTemp(self, temp, hwVersion, nozzleDiam):

        tempCurve = self.getTempCurve(hwVersion, nozzleDiam)
        return tempCurve.lookupTemp(temp)

    def getTempCurve(self, hwVersion, nozzleDiam):

        if nozzleDiam not in self.tempCurves:

            flowrateData = self.getValues()["tempFlowrateCurve_%d" % (nozzleDiam*100)]

            # if type(data) == types.DictType:
                # data = data["data"]
            # assert(type(data) == types.ListType)

            # Check hardware version
            assert(flowrateData["version"] == hwVersion)

            tempCurve = TempCurve(flowrateData["data"])
            self.tempCurves[nozzleDiam] = tempCurve

        return self.tempCurves[nozzleDiam]

    @classmethod
    def getSlippage(cls, hwVersion, nozzleDiam):

        flowrateData = cls.get().getValues()["tempFlowrateCurve_%d" % (nozzleDiam*100)]
        assert(flowrateData["version"] == hwVersion)
        return flowrateData["slippage"]

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



if __name__ == "__main__":

    printerProfile = PrinterProfile("UM2.json")

    import pprint
    print "table:", pprint.pprint(printerProfile.calTable)

    for sa in [0.25, 1.25, 10.25, 20.25]:

        print "flowrate for %f: %f" % (sa, printerProfile.getFlowrateFromSensorRate(sa))
        print




















