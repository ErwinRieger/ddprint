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
# Material profile, singleton
#
####################################################################################################
class MatProfile(ProfileBase):

    _single = None 

    def __init__(self, name, smatName, printerName):

        if MatProfile._single:
            raise RuntimeError('Error: a MatProfile instance already exists')

        if smatName:
            smatName = os.path.join(printerName, smatName)

        super(MatProfile, self).__init__(MatProfile, name, smatName)

        self.matArea = (math.pi * pow(float(self.values["material_diameter"]), 2)) / 4.0

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
    def getMatArea(cls):
        return cls.get()._getMatArea()

    def _getMatArea(self):
        return self.matArea

    @classmethod
    def getKAdv(cls):
        return float(cls.getValues()["kAdvance"])

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

    def _getFlowrateForTemp(self, temp, hwVersion, nozzleDiam):

        FR0pwm= self.getFR0pwm(hwVersion, nozzleDiam)   # a0 feedrate
        Ktemp = self.getKtemp (hwVersion, nozzleDiam)   # a1
        p0Temp = self.getP0temp(hwVersion, nozzleDiam)  # a0 temperature

        fr = FR0pwm + (temp - p0Temp) * Ktemp

        print "flowrate for temp:", temp, fr
        return fr

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




















