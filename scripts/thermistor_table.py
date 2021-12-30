#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#/*
# This file is part of ddprint - a direct drive 3D printer firmware.
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


#
# Print a C-code table to convert between ADC value (0-1023) and the
# correspoding temperature.
#

import sys, os

from math import sqrt, log, exp

#
# Fixed values for the epcos 100k with a 4.7k pullup (ender5),
# todo: specify values on commandline.
#
circuitName = "EPCOS 100k with 4.7k pullup"
AdcRes = 1024

#
# GND -*- NTC -*- 4.7k -*- Ub 5V
#
# Pullup resistor value
Rp = 4700.0
#
Ub = 5.0

#######################################################
#
# Steinhart-hart, from wikipedia
#
def kelvin(t): return t + 273.15

# The three points from the datasheet
T1 = kelvin(25)
R1 = 100000.0
T2 = kelvin(150)
R2 = 1641.9
T3 = kelvin(250)
R3 = 226.1

L1 = log(R1)
L2 = log(R2)
L3 = log(R3)

Y1 = 1.0 / T1
Y2 = 1.0 / T2
Y3 = 1.0 / T3

G2 = (Y2 - Y1) / (L2 - L1)
G3 = (Y3 - Y1) / (L3 - L1)

# Steinhart-hart coefficients
C = (G3 - G2) / ((L3 - L2) * (L1 + L2 + L3))
B = G2 - C * (L1**2 + L1*L2 + L2**2)
A = Y1 - (B + L1**2 * C) * L1

# print "Steinhart-hart: a: %.4f, b: %.4f, c: %.8f" % (A, B, C)

#######################################################

vstep = Ub / AdcRes

#################
def rtFromTemp(temp):
    x = (1.0/C) * (A - (1.0 / kelvin(temp)))
    y = sqrt((B / (3.0*C))**3 + (x**2 / 4.0))
    rt = exp((y-(x/2.0))**(1.0/3) - (y+(x/2.0))**(1.0/3))
    return rt

#
# vout = (Rt * Ub) / (Rt + Rp)
#
def calcVout(rt):
    return (rt * Ub) / (rt + Rp)

#
# Adc value at specific temp
#
def calcAdc(temp):
    rt = rtFromTemp(temp)
    vout = calcVout(rt)
    adc = int(round(vout / vstep))
    # print "temp: %.2f, rt: %.2f, vout: %.2f, adc: %d" % (temp, rt, vout, adc)
    return adc

def rtFromU(u):
    rt = (u * Rp) / (Ub - u)
    return rt

def tFromrt(rt):
    t = 1.0 / (A + B * log(rt) + C * (log(rt)**3))
    return t - 273.15

#################

# Lower end of adc values at 0°
adc0 = calcAdc(0.0)

# Upper end of adc values at 500°
adc500 = calcAdc(500.0)

lowADC = min(adc0, adc500)
highADC = max(adc0, adc500)

print("//")
print("// Table to convert ADC values into temperatures.")
print("// This is for the '%s' circuit." % circuitName)
print("// The table begins with ADC value %d." % lowADC)
print("// The last table entry is for ADC value %d." % highADC)
print("// Temperature values are stored as 1/16-th °C int values.")
print("// Generated by script %s." % os.path.basename(sys.argv[0]))
print("//")
print("#define ThermistorTableLowADC %d" % lowADC)
print("#define ThermistorTableHighADC %d" % highADC)
print("#define MaxThermistorTableIndex (ThermistorTableHighADC - ThermistorTableLowADC)")
print("const int16_t thermistorTable[] PROGMEM = {")

i = 0
for adcIn in range(lowADC, highADC+1):

    uIn = vstep * adcIn

    rt = rtFromU(uIn)

    # print "rt:", rt

    t = tFromrt(rt)

    # print "t:", t

    print("    %d, // Index %d, ADC value %d, T %.2f°C" % (round(t*16), i, adcIn, t))

    i += 1

print("};")

