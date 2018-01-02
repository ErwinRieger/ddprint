# -*- coding: utf-8 -*-
#
# Compute ADC values for the temperatre range from 200 to 300 °C
# This is for the ultimaker2 board with pt100 and INA826 amplifier.
#

import sys, math

#
# GND -*- 2.2K -*- pt100 -*- 2.2k -*- Ub
#
Rg = 5490.0
#
G = 1 + 49400 / Rg 
#
R2 = 2200.0
#
Ub = 5.0
#
Rp0 = 100

A = 3.9083e-3
B = -5.775e-7

for t in range(200, 301):

    rP = Rp0 * ( 1 + A*t + B*t**2 ) 

    i = Ub / (2*R2 + rP)

    uRp = rP * i

    uIn = uRp * G

    adc = round(uIn / (5.0/1023))

    print "t: %d, rP: %f, ADC: %d" % (t, rP, adc)


