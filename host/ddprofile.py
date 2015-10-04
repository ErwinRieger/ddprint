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

import json, math

from ddprintconstants import dimNames

####################################################################################################
#
# Printer profile, singleton
#
####################################################################################################
class PrinterProfile():

    __single = None 

    def __init__(self, name):

        if PrinterProfile.__single:
            raise RuntimeError('A PrinterProfile already exists')

        self.values = json.load(open(name + ".json"))
        PrinterProfile.__single = self

    @classmethod
    def get(cls):
        return cls.__single

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

####################################################################################################
#
# Material profile, singleton
#
####################################################################################################
class MatProfile():

    __single = None 

    def __init__(self, name):

        if MatProfile.__single:
            raise RuntimeError('A MatProfile already exists')

        self.values = json.load(open(name + ".json"))
        MatProfile.__single = self

    def override(self, key, value):
        self.values[key] = value

    @classmethod
    def get(cls):
        return cls.__single

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
        aFilament = (math.pi * pow(cls.getMatDiameter(), 2)) / 4.0
        return aFilament

####################################################################################################
#
# Nozzle profile, singleton
#
####################################################################################################
class NozzleProfile():

    __single = None 

    def __init__(self, name):

        if NozzleProfile.__single:
            raise RuntimeError('A NozzleProfile already exists')

        self.values = json.load(open(name + ".json"))
        NozzleProfile.__single = self

    @classmethod
    def get(cls):
        return cls.__single

    @classmethod
    def getValues(cls):
        return cls.get().values

    @classmethod
    def getAutoTempFactor(cls, useExtrusionAutoTemp):

        if useExtrusionAutoTemp:
            return float(cls.getValues()["extrusionAutoTempFactor"])

        return float(cls.getValues()["autoTempFactor"])

    @classmethod
    def getSize(cls):
        return float(cls.getValues()["size"])







