#!/usr/bin/python
# -*- coding: utf-8 -*-
#
#/*
# This file is part of ddprint - a 3D printer firmware.
# 
# Copyright 2020 erwin.rieger@ibrieger.de
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


import sys, os
import numpy as np
from argparse import Namespace

import matplotlib.pyplot as plt

sys.path.append(os.path.join(os.path.dirname(sys.argv[0]), "../host"))

import ddprintutil
from ddprofile import MatProfile, PrinterProfile, NozzleProfile

#########################################################################################

#########################################################################################

if __name__ == "__main__":

    printerName = sys.argv[1]
    nozzleProfile = sys.argv[2]
    baseProfile = sys.argv[3]

    pp = PrinterProfile(printerName)
    nozzle = NozzleProfile(nozzleProfile)
    nozzleDiam = nozzle.getSizeI()

    tempGraphs = []
    pwmGraphs = []

    bp = MatProfile(name=baseProfile, smatName=None, printerName=printerName)
    goodTemp = bp.getHotendGoodTemp()

    for specificProfile in sys.argv[4:]:

        mp = MatProfile(name=baseProfile, smatName=specificProfile, printerName=printerName)

        kpwm = mp.getKpwm(pp.getHwVersion(), nozzleDiam)
        ktemp = mp.getKtemp(pp.getHwVersion(), nozzleDiam)

        p0 = mp.getP0pwm(pp.getHwVersion(), nozzleDiam)
        p0Temp = mp.getP0temp(pp.getHwVersion(), nozzleDiam)
        fr0 = mp.getFR0pwm(pp.getHwVersion(), nozzleDiam)

        flowrateData = mp.getFlowrateData(pp.getHwVersion(), nozzleDiam)
        hasPrintingValues = "P0tempPrint" in flowrateData

        if hasPrintingValues:
            p0Print = mp.getP0pwmPrint(pp.getHwVersion(), nozzleDiam)
            p0TempPrint = mp.getP0tempPrint(pp.getHwVersion(), nozzleDiam)
            fr0Print = mp.getFR0pwmPrint(pp.getHwVersion(), nozzleDiam)

        maxTemp = mp.getHotendMaxTemp()

        xser = []
        tser = []
        pser = []
        xserPrint = []
        tserPrint = []
        pserPrint = []

        fr = fr0 
        if hasPrintingValues:
            fr = fr0Print - (p0TempPrint-p0Temp) * ktemp

        while True:

            t = p0Temp + (fr - fr0)/ktemp
            p = p0 + (fr - fr0)/kpwm

            if (fr >= fr0):
                xser.append(fr)
                tser.append(t)
                pser.append(p)

            if hasPrintingValues:
                tPrint = p0TempPrint + (fr - fr0Print)/ktemp
                pPrint = p0Print + (fr - fr0Print)/kpwm

                if (tPrint <= maxTemp):
                    xserPrint.append(fr)
                    tserPrint.append(tPrint)
                    pserPrint.append(pPrint)

            if t > maxTemp:
                break

            fr += 0.1

        tempGraphs.append((xser, tser, specificProfile))
        pwmGraphs.append((xser, pser, specificProfile))

        if hasPrintingValues:
            tempGraphs.append((xserPrint, tserPrint, specificProfile+" print"))
            pwmGraphs.append((xserPrint, pserPrint, specificProfile+" print"))

    plot1 = plt.subplot(2,1,1)
    plt.title("Flowrate vs Temp")
    plt.xlabel(u'temp [°C]')
    plt.ylabel(u'flowrate [mm³/s]')
    plt.grid()
    plt.axvline(x=goodTemp, color='r', linestyle='-')

    for (xser, tser, specificProfile) in tempGraphs:
        plt.plot(tser, xser, label=specificProfile)

    plt.legend(loc="upper left")

    # plt.plot(t, ba)
    # plt.plot(t, x)
    plt.ylim(0)

    plt.subplot(2,1,2, sharex=plot1)
    plt.title("Flowrate vs PWM")
    plt.xlabel(u'PWM')
    plt.ylabel(u'flowrate [mm³/s]')
    plt.grid()
    # plt.axhline(y=avg, color='r', linestyle='-')
    # plt.axvline(x=crossingAvg.locked[1], color='r', linestyle='-')
    # plt.plot(t, la)
    # plt.plot(t, ba)
    
    for (xser, pser, specificProfile) in pwmGraphs:
        plt.plot(pser, xser, label=specificProfile)

    plt.legend(loc="upper left")

    plt.show()











