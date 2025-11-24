# -*- coding: utf-8 -*-
#
#/*
# This file is part of ddprint - a 3D printer firmware.
# 
# Copyright 2021 erwin.rieger@ibrieger.de
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

import math
import packedvalue
from ddprintconstants import TempScale, PidPrecision, PidFrequency

####################################################################################################

# Convert temperature (int) to firmware temperature (fractions of °C)
def toFWTemp(t):
    return t * TempScale

####################################################################################################

# Convert firmware temperature (fractions of °C) to temperature (float)
def fromFWTemp(t):
    return float(t) / TempScale

####################################################################################################

# Compute scaling factor and shiftvalue for PID Kp
def pidScaleKp(kP):

    #
    # pTerm = Kp * e
    #
    kPMult = kP / TempScale

    shiftBits = int( PidPrecision - math.floor(math.log(kPMult, 2)))

    kPScale = pow(2, shiftBits)

    kPScaled = int(kPMult * kPScale + 0.5)

    # print "kP: %.4f, kPMult: %f, shiftBits: %d, kPScaled: %d" % (kP, kPMult, shiftBits, kPScaled)

    return packedvalue.scaledint_t(kPScaled, shiftBits)

####################################################################################################
# Compute scaling factor and shiftvalue for PID Ki
def pidScaleKi(kI):

    kiPrecision = 8

    #
    # iTerm = Ki * pid_dt * eSum
    #
    kIMult = kI / (TempScale * PidFrequency)

    maxEsum16 = int(255 / kIMult) # Limit of eSum 

    maxScale = (pow(2, 31)-1) / (kIMult * maxEsum16) # Max scaling factor to avoid overflow

    maxBits = math.log(maxScale, 2)
    shiftBits = int( kiPrecision - math.floor(math.log(kIMult, 2)))

    kIScale = pow(2, shiftBits)

    kIScaled = int(kIMult * kIScale + 0.5)

    # print "kI: %.4f, kIMult: %f, shiftBits: %d(of %d), kIScaled: %d, maxEsum16: %d" % (kI, kIMult, shiftBits, maxBits, kIScaled, maxEsum16)

    assert(shiftBits <= maxBits)

    return (packedvalue.scaledint_t(kIScaled, shiftBits), maxEsum16)

####################################################################################################

# Compute scaling factor and shiftvalue for PID Kd
def pidScaleKd(kD):

    #
    # dTerm = Kd * (e - eAlt) / pid_dt; 
    #
    kDMult = (kD * PidFrequency) / TempScale

    maxDeltaE = 500.0 * TempScale # Switch from 0 to 500°C

    maxScale = (pow(2, 31)-1) / (kDMult * maxDeltaE) # Max scaling factor to avoid overflow

    maxBits = math.log(maxScale, 2)
    shiftBits = int( PidPrecision - math.floor(math.log(kDMult, 2)))

    assert(shiftBits <= maxBits)

    kDScale = pow(2, shiftBits)

    kDScaled = int(kDMult * kDScale + 0.5)

    # print "kD: %.4f, kDMult: %f, shiftBits: %d(of %d), kDScaled: %d" % (kD, kDMult, shiftBits, maxBits, kDScaled)

    return packedvalue.scaledint_t(kDScaled, shiftBits)

####################################################################################################


def pidSwitch(kiOld, kiNew):

    swMult = kiOld / kiNew 

    shiftBits = int(PidPrecision - math.floor(math.log(swMult, 2)))

    swScale = pow(2, shiftBits)

    swScaled = int(swMult * swScale + 0.5)

    # print "swMult: %.4f, shiftBits: %d, swScaled: %d" % (swMult, shiftBits, swScaled)

    return packedvalue.scaledint_t(swScaled, shiftBits)

####################################################################################################

def fsCalibration(fsc):

    fsScale = 128 # 32 # Hardcoded in firmware
    FSPrecision = 8

    fsc128 = fsc * fsScale

    shiftBits = int(FSPrecision - math.floor(math.log(fsc128, 2)))

    Scale = pow(2, shiftBits)

    Scaled = int(fsc128 * Scale + 0.5)

    print(f"value: {fsc}, fsc128: %.4f, shiftBits: %d, Scaled: %d" % (fsc128, shiftBits, Scaled))

    return packedvalue.scaledint16_t(Scaled, shiftBits)

####################################################################################################

def eTimer(eTimer):

    fsScale = 1024.0 # Hardcoded in firmware
    FSPrecision = 10

    et = fsScale / eTimer

    shiftBits = int(FSPrecision - math.floor(math.log(et, 2)))

    Scale = pow(2, shiftBits)

    Scaled = int(et * Scale + 0.5)

    # print "eTimer: %d, fscaled: %f, shiftBits: %d, Scaled: %d" % (eTimer, et, shiftBits, Scaled)

    return packedvalue.scaledint_t(Scaled, shiftBits)

####################################################################################################



if __name__ == "__main__":

    fsc = fsCalibration(0.25)
    print(fsc)
    fsc = fsCalibration(0.35)
    print(fsc)
    fsc = fsCalibration(1.35)
    print(fsc)
    fsc = fsCalibration(0.01)
    print(fsc)





