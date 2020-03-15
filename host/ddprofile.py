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

        try:
            f = open(os.path.join(os.environ["HOME"], ".ddprint", name) + ".json")
            return f
        except IOError:
            pass

        try:
            f = open(os.path.join(os.environ["HOME"], ".ddprint", name))
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

        if PrinterProfile._single:
            raise RuntimeError('Error: a PrinterProfile instance already exists')

        super(PrinterProfile, self).__init__(PrinterProfile, name)

    @classmethod
    def get(cls):

        if not cls._single:
            raise RuntimeError('PrinterProfile instance not created, yet')

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

    @classmethod
    def getHwVersion(cls):
        return cls.getValues()["hwVersion"]

    def getFilSensorCalibration(self):
        cal = self.getValues()["filSensorCalibration"]
        return cal

    @classmethod
    def getBedlevelOffset(cls):
        return cls.getValues()["add_homeing_z"]

    @classmethod
    def getFeederWheelDiam(cls):
        return cls.getValues()["feederWheelDiam"]

    @classmethod
    def getFeederWheelCircum(cls):
        return cls.getFeederWheelDiam() * math.pi

    @classmethod
    def getFilSensorCountsPerMM(cls):
        return cls.getValues()["filSensorCountsPerMM"]

    @classmethod
    def getFilSensorInterval(cls):
        return cls.getValues()["filSensorInterval"]

    @classmethod
    def getSettings(cls):
        return {
            "filSensorCalibration": cls.getValues()["filSensorCalibration"],
            "Kp": cls.getValues()["Kp"],
            "Ki": cls.getValues()["Ki"],
            "Kd": cls.getValues()["Kd"],
            "Tu": cls.getValues()["Tu"],
            }

    @classmethod
    def getTu(cls):
        return cls.get()._getTu()

    def _getTu(self):
        return self.values["Tu"]

    @classmethod
    def getTg(cls):
        return cls.get()._getTg()

    def _getTg(self):
        return self.values["Tg"]

    @classmethod
    def getNLongInterval(cls, feedrate):

        dt = cls.getFilSensorInterval()
        # Time for one revolution
        tRound = cls.getFeederWheelCircum() / feedrate
        nAvg = int(round(tRound / dt))

        nAvg = max(nAvg, 2)
        return nAvg

    @classmethod
    def getNShortInterval(cls, feedrate):

        dt = cls.getFilSensorInterval()
        # Time for one revolution
        tRound = cls.getFeederWheelCircum() / feedrate
        nAvg = int(round(tRound / (dt*8)))

        nAvg = max(nAvg, 2)
        return nAvg

####################################################################################################
#
# To access tempearture curve data
#
####################################################################################################
class old_TempCurve:

    def __init__(self, curveList):

        self.curveList = curveList
        
        (minTemp, minFlowrate) = curveList[0]
        self.minTemp = minTemp
        self.minFlowrate = minFlowrate

        (maxTemp, maxFlowrate) = curveList[-1]
        self.maxTemp = maxTemp
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

        # print "flowrate not found:", flowrate, key

        # Linear interpolation
        for i in range(len(self.curveList)-1):

            temp1, fr1 = self.curveList[i]
            temp2, fr2 = self.curveList[i+1]

            if flowrate >= fr1 and flowrate < fr2 :
                # print "found range:", fr1, flowrate, fr2

                dx = fr2 - fr1
                dy = temp2 - temp1

                t = temp1 + (dy / dx) * (flowrate - fr1)

                # print "interpolated temp: ", t
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

                print "interpolated feedrate: ", fr
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

        if MatProfile._single:
            raise RuntimeError('Error: a MatProfile instance already exists')

        super(MatProfile, self).__init__(MatProfile, name, smatName)

        self.matArea = (math.pi * pow(float(self.values["material_diameter"]), 2)) / 4.0

        self.tempCurves = {}

    def override(self, key, value):
        assert(key != "material_diameter")
        ProfileBase.override(self, key, value)

    @classmethod
    def get(cls):

        if not cls._single:
            raise RuntimeError('MatProfile instance not created, yet')

        return cls._single

    @classmethod
    def getValues(cls):
        return cls.get().values

    @classmethod
    def getMatDiameter(cls):
        return float(cls.getValues()["material_diameter"])

    @classmethod
    def getHotendBaseTemp(cls):
        return int(cls.getValues()["hotendBaseTemp"])

    @classmethod
    def getHotendGoodTemp(cls):
        return int(cls.getValues()["hotendGoodTemp"])

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
        return cls.get()._getMatArea()

    def _getMatArea(self):
        return self.matArea

    @classmethod
    def getKAdv(cls):
        return float(cls.getValues()["kAdvance"])

    @classmethod
    def old_getTempForFlowrate(cls, flowrate, hwVersion, nozzleDiam):
        return cls.get()._getTempForFlowrate(flowrate, hwVersion, nozzleDiam)

    def _getTempForFlowrate(self, flowrate, hwVersion, nozzleDiam):

        old_tempCurve = self.getTempCurve(hwVersion, nozzleDiam)
        return old_tempCurve.lookupFlowrate(flowrate)

    @classmethod
    def getFlowrateForTemp(cls, temp, hwVersion, nozzleDiam):
        return cls.get()._getFlowrateForTemp(temp, hwVersion, nozzleDiam)

    def getKpwm(self, hwVersion, nozzleDiam):

        flowrateData = self.getValues()["properties_%d" % (nozzleDiam*100)]
        # Check hardware version
        assert(flowrateData["version"] == hwVersion)
        return flowrateData["Kpwm"]

    def getKtemp(self, hwVersion, nozzleDiam):

        flowrateData = self.getValues()["properties_%d" % (nozzleDiam*100)]
        # Check hardware version
        assert(flowrateData["version"] == hwVersion)
        return flowrateData["Ktemp"]

    # @classmethod
    def getP0pwm(cls, hwVersion, nozzleDiam):
        return cls.get()._getP0pwm(hwVersion, nozzleDiam);

    def _getP0pwm(self, hwVersion, nozzleDiam):

        flowrateData = self.getValues()["properties_%d" % (nozzleDiam*100)]
        # Check hardware version
        assert(flowrateData["version"] == hwVersion)
        return flowrateData["P0pwm"]

    def getFR0pwm(self, hwVersion, nozzleDiam):

        flowrateData = self.getValues()["properties_%d" % (nozzleDiam*100)]
        # Check hardware version
        assert(flowrateData["version"] == hwVersion)
        return flowrateData["FR0pwm"]

    def getP0temp(self, hwVersion, nozzleDiam):

        flowrateData = self.getValues()["properties_%d" % (nozzleDiam*100)]
        # Check hardware version
        assert(flowrateData["version"] == hwVersion)
        return flowrateData["P0temp"]

    def old_getFlowrateForTemp(self, temp, hwVersion, nozzleDiam):

        old_tempCurve = self.getTempCurve(hwVersion, nozzleDiam)
        return old_tempCurve.lookupTemp(temp)

    def _getFlowrateForTemp(self, temp, hwVersion, nozzleDiam):

        FR0pwm= self.getFR0pwm(hwVersion, nozzleDiam)   # a0 feedrate
        Ktemp = self.getKtemp (hwVersion, nozzleDiam)   # a1
        p0Temp = self.getP0temp(hwVersion, nozzleDiam)  # a0 temperature

        fr = FR0pwm + (temp - p0Temp) * Ktemp

        print "flowrate for temp:", temp, fr
        return fr

    def getTempCurve(self, hwVersion, nozzleDiam):

        if nozzleDiam not in self.tempCurves:

            flowrateData = self.getValues()["properties_%d" % (nozzleDiam*100)]

            # if type(data) == types.DictType:
                # data = data["data"]
            # assert(type(data) == types.ListType)

            # Check hardware version
            assert(flowrateData["version"] == hwVersion)

            old_tempCurve = TempCurve(flowrateData["data"])
            old_self.tempCurves[nozzleDiam] = tempCurve

        return self.tempCurves[nozzleDiam]

    @classmethod
    def getSlippage(cls, hwVersion, nozzleDiam):
        return cls.get()._getSlippage(temp, hwVersion, nozzleDiam)

    def _getSlippage(cls, hwVersion, nozzleDiam):
        flowrateData = cls.get().getValues()["properties_%d" % (nozzleDiam*100)]
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

        if NozzleProfile._single:
            raise RuntimeError('Error: a NozzleProfile instance already exists')

        super(NozzleProfile, self).__init__(NozzleProfile, name)

    @classmethod
    def get(cls):

        if not cls._single:
            raise RuntimeError('NozzleProfile instance not created, yet')

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
    def getArea(cls):
        return (math.pi/4) * pow(cls.getSize(), 2)



if __name__ == "__main__":

    printerProfile = PrinterProfile("UM2.json")




















