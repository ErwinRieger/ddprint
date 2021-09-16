#/*
# This file is part of ddprint - a 3D printer firmware.
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

from ddprintconstants import dimNames, X_AXIS, Y_AXIS, Z_AXIS, A_AXIS, B_AXIS

#
# Some methods are duplicated, one class instance (singleton) version and one 
# instance version. This has the goal to slowly remove the singletons.
#

class ProfileException(Exception):

    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return "Profile error: " + self.msg

####################################################################################################
#
# Json profile base class
#
####################################################################################################
class ProfileBase(object):

    def __init__(self, name, specificName=""):

        self.name = name
        self.specificName = specificName

        f = self.openJson(name)

        self.values = util.jsonLoad(f)

        # Let specific profile overwrite values from a generic profile
        if specificName:
            f = self.openJson(specificName)
            specificValues = util.jsonLoad(f)
            for k in specificValues.keys():
                self.values[k] = specificValues[k]

    def getBaseName(self):
        return os.path.basename(self.name)

    def hasValue(self, valueName):
        return valueName in self.values

    def getValue(self, valueName):
        try:
            return self.values[valueName]
        except KeyError:
            raise ProfileException("Profile '%s' does has no key: '%s'!" % (self.name, valueName))

    def openJson(self, name):

        #
        # Directory search order:
        # * working directory
        # * directories listed in $DDPRINTPROFILES path
        #
        searchpath = []

        try:
            dirlist = ["."] + os.environ["DDPRINTPROFILES"].split(":")
        except KeyError:
            dirlist = ["."]

        for d in dirlist:
            for p in ["mat-profiles", "nozzle-profiles", "machine-profiles"]:
                searchpath.append(os.path.join(d, p))

        for searchdir in searchpath:

            for extension in ["", ".json"]:

                try:
                    f = open(os.path.join(searchdir, name+extension))
                    return f
                except IOError:
                    pass

        raise Exception("Profile %s not found." % name)

    def override(self, key, value):
        self.values[key] = value

    def logValues(self, heading, logger):

        logger.logPrintLog("\n%s: %s\n" % (heading, self.name))
        if self.specificName:
            logger.logPrintLog("  Specific profile: %s\n" % self.specificName)
        for key in self.values.keys():
            logger.logPrintLog("  %s: %s\n" % (key, str(self.values[key])))


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

        PrinterProfile._single = self

        super(PrinterProfile, self).__init__(name)

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

    def getStepsPerMMI(self, axisNr):
        return int(self.getValue("axes")[dimNames[axisNr]]["steps_per_mm"])

    def getHomeDir(self, axisNr):
        return int(self.getValue("axes")[dimNames[axisNr]]["home_dir"])

    def getHomeFeedrate(self, axisNr):
        return int(self.getValue("axes")[dimNames[axisNr]]["home_feedrate"])

    @classmethod
    def getStepsPerMMVector(cls):
        return map(lambda d: cls.getStepsPerMM(d), range(5))

    def getStepsPerMMVectorI(self):
        return map(lambda d: self.getStepsPerMM(d), range(5))

    @classmethod
    def getMaxFeedrate(cls, axisNr):
        return cls.getValues()["axes"][dimNames[axisNr]]["max_feedrate"]

    def getMaxFeedrateI(self, axisNr):
        return self.getValue("axes")[dimNames[axisNr]]["max_feedrate"]

    @classmethod
    def getMaxFeedrateVector(cls):
        return map(lambda d: cls.getMaxFeedrate(d), range(5))

    def getMaxFeedrateVectorI(self):
        return map(lambda d: self.getMaxFeedrateI(d), range(5))

    def getRetractFeedrate(self):
        return self.getValue("RetractFeedrate")

    def getRetractLength(self):
        return float(self.getValue("RetractLength"))

    @classmethod
    def getMaxAxisAcceleration(cls):
        return cls.getValues()["MaxAxisAcceleration"]

    def getMaxAxisAccelerationI(self):
        return self.getValue("MaxAxisAcceleration")

    @classmethod
    def getHwVersion(cls):
        return cls.getValues()["hwVersion"]

    def getHwVersionI(self):
        return self.getValue("hwVersion")

    def getFilSensorCalibration(self):
        return self.getValues()["filSensorCalibration"]

    @classmethod
    def getBedlevelOffset(cls):
        return cls.getValues()["add_homeing_z"]

    def getBedlevelOffsetI(self):
        return self.getValue("add_homeing_z")

    @classmethod
    def getFeederWheelDiam(cls):
        return cls.getValues()["feederWheelDiam"]

    def getFeederWheelDiamI(self):
        return float(self.getValue("feederWheelDiam"))

    @classmethod
    def getFeederWheelCircum(cls):
        return cls.getFeederWheelDiam() * math.pi

    def getFeederWheelCircumI(self):
        return self.getFeederWheelDiamI() * math.pi

    def getFilSensorCountsPerMM(self):
        return self.getValue("filSensorCountsPerMM")

    @classmethod
    def getFilSensorInterval(cls):
        return cls.getValues()["filSensorInterval"]

    def getFilSensorIntervalI(self):
        return float(self.getValue("filSensorInterval"))

    def getSettings(self, pidSet):

        pidSetHeating = pidSet[:2]
        pidSetCooling = pidSet[2:]

        print "getSettings(): pidset to use: %s, heating: %s, cooling: %s" % (pidSet, pidSetHeating, pidSetCooling)

        return {
            "filSensorCalibration": self.getFilSensorCalibration(),
            "Kp": self.getPidValue(pidSetHeating, "Kp"),
            "Ki": self.getPidValue(pidSetHeating, "Ki"),
            "Kd": self.getPidValue(pidSetHeating, "Kd"),
            "KpC": self.getPidValue(pidSetCooling, "Kp"),
            "KiC": self.getPidValue(pidSetCooling, "Ki"),
            "KdC": self.getPidValue(pidSetCooling, "Kd"),
            "Tu": self.getTuI(),
            "stepsPerMMX": self.getStepsPerMMI(X_AXIS),
            "stepsPerMMY": self.getStepsPerMMI(Y_AXIS),
            "stepsPerMMZ": self.getStepsPerMMI(Z_AXIS),
            "stepsPerMMA": self.getStepsPerMMI(A_AXIS),
            # "stepsPerMMB": self.getStepsPerMMI(B_AXIS),
            "buildVolX": int(self.getPlatformLengthI(X_AXIS) * self.getStepsPerMMI(X_AXIS)),
            "buildVolY": int(self.getPlatformLengthI(Y_AXIS) * self.getStepsPerMMI(Y_AXIS)),
            "buildVolZ": int(self.getPlatformLengthI(Z_AXIS) * self.getStepsPerMMI(Z_AXIS)),
            "xHomeDir": int(self.getHomeDir(X_AXIS)),
            "yHomeDir": int(self.getHomeDir(Y_AXIS)),
            "zHomeDir": int(self.getHomeDir(Z_AXIS)),
            }

    def getTuI(self):
        return float(self.getValue("Tu"))

    def getTgI(self):
        return float(self.getValue("Tg"))

    def getPidValue(self, pidSet, key):
        return float(self.getValue(pidSet)[key])

    def getNLongIntervalI(self, feedrate, howlong):

        # dt = self.getFilSensorIntervalI()
        # Time for one revolution
        tRound = self.getFeederWheelCircumI() / feedrate
        # nAvg = int(round(tRound / dt))

        # nAvg = max(nAvg, 2)
        return howlong / tRound

    def getWeakPowerBedTemp(self):
        return int(self.getValues()["weakPowerBedTemp"])

    @classmethod
    def getPlatformLength(cls, axisNr):
        return cls.get().getValue("axes")[dimNames[axisNr]]["platform_length"]

    def getPlatformLengthI(self, axisNr):
        return self.getValue("axes")[dimNames[axisNr]]["platform_length"]

    def getBedLevelMode(self):
        return self.getValue("bedLevelMode")

    def getJerk(self, dim):
        return self.getValue("axes")[dim]["jerk"]

    def getMaxStepperFreq(self):
        return float(self.getValue("maxStepperFreq"))

    def getBowdenLength(self):
        return self.getValue("bowdenLength")

    def getXo(self):
        return int(self.getValue("Xo"))

####################################################################################################
#
# Material profile, singleton
#
####################################################################################################
class MatProfile(ProfileBase):

    def __init__(self, name, smatName, printerName, hwVersion, nozzleDiam):

        if smatName:
            smatName = os.path.join(printerName, smatName)

        super(MatProfile, self).__init__(name, smatName)

        # Check hardware version
        assert(self.getValue("version") == hwVersion)

        self.matArea = (math.pi * pow(float(self.values["material_diameter"]), 2)) / 4.0

        self.nozzleDiam = nozzleDiam

    def override(self, key, value):
        assert(key != "material_diameter")
        ProfileBase.override(self, key, value)

    def getValuesI(self):
        return self.values

    def getHotendBaseTemp(self):
        return int(self.getValue("hotendBaseTemp"))

    def getHotendGoodTemp(self):
        return int(self.getValue("hotendGoodTemp"))

    def getHotendMaxTemp(self):
        return int(self.getValuesI()["hotendMaxTemp"])

    def getBedTemp(self):
        return int(self.getValue("bedTemp"))

    def getBedTempReduced(self):
        return int(self.getValue("bedTempReduced"))

    def getMatArea(self):
        return self.matArea

    def getKAdvI(self):
        return float(self.getValue("kAdvance"))
        
    def getFlowrateData(self):
        return  self.getValue("properties_%d" % (self.nozzleDiam*100))

    def getKpwm(self):

        flowrateData = self.getFlowrateData()
        return flowrateData["Kpwm"]

    def getKtemp(self):

        flowrateData = self.getFlowrateData()
        return flowrateData["Ktemp"]

    def getP0pwm(self):

        flowrateData = self.getFlowrateData()
        return flowrateData["P0pwm"]

    def getP0pwmPrint(self):

        flowrateData = self.getFlowrateData()
        return flowrateData["P0pwmPrint"]

    def getFR0pwm(self):

        flowrateData = self.getFlowrateData()
        return flowrateData["FR0pwm"]

    def getFR0pwmPrint(self):

        flowrateData = self.getFlowrateData()
        return flowrateData["FR0pwmPrint"]

    def getP0temp(self):

        flowrateData = self.getFlowrateData()
        return flowrateData["P0temp"]

    def getP0tempPrint(self):

        flowrateData = self.getFlowrateData()
        return flowrateData["P0tempPrint"]

    def getSlippageI(self):
        flowrateData = self.getFlowrateData()
        return flowrateData["slippage"]

    def getFrSLE(self):

        return (
                util.SLE(x1=self.getP0temp(), y1=self.getFR0pwm(), m=self.getKtemp()),
                util.SLE(x1=self.getP0pwm(), y1=self.getFR0pwm(), m=self.getKpwm())
                )

    def getFrSLEPrint(self):

        return (
                util.SLE(x1=self.getP0tempPrint(), y1=self.getFR0pwmPrint(), m=self.getKtemp()),
                util.SLE(x1=self.getP0pwmPrint(), y1=self.getFR0pwmPrint(), m=self.getKpwm())
                )



####################################################################################################
#
# Nozzle profile, singleton
#
####################################################################################################
class NozzleProfile(ProfileBase):

    # _single = None 

    def __init__(self, name):

        # if NozzleProfile._single:
            # raise RuntimeError('Error: a NozzleProfile instance already exists')
        # NozzleProfile._single = self

        super(NozzleProfile, self).__init__(name)

    # @classmethod
    # def get(cls):
        # if not cls._single:
            # raise RuntimeError('NozzleProfile instance not created, yet')
        # return cls._single

    @classmethod
    def getValues(cls):
        return cls.get().values

    @classmethod
    def getSize(cls):
        return float(cls.getValues()["size"])

    def getSizeI(self):
        return float(self.getValues("size"))

    @classmethod
    def getMaxExtrusionRate(cls):
        return float(cls.getValues()["maxExtrusionRate"])

    @classmethod
    def getArea(cls):
        return (math.pi/4) * pow(cls.getSize(), 2)

    def getSizeI(self):
        return float(self.getValue("size"))

    def getMaxExtrusionRateI(self):
        return float(self.getValue("maxExtrusionRate"))

    def getAreaI(self):
        return (math.pi/4) * pow(self.getSize(), 2)



if __name__ == "__main__":

    printerProfile = PrinterProfile("UM2.json")




















