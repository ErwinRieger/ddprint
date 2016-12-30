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

import math, os
import ddprintutil as util

from ddprintconstants import dimNames

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

    @classmethod
    def getBaseExtrusionRate(cls):
        return float(cls.getValues()["baseExtrusionRate"])

    @classmethod
    def getAutoTempFactor(cls):
        return float(cls.getValues()["extrusionAutoTempFactor"])

    @classmethod
    def getKAdv(cls):
        return float(cls.getValues()["kAdvance"])

    @classmethod
    def getKFeederCompensation(cls):
        return float(cls.getValues()["kFeederCompensation"])

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























