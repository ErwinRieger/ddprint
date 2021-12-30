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

#
# Program to plot ddPrint material profiles.
#

import sys, os
import argparse, ddargs

import matplotlib.pyplot as plt

sys.path.append(os.path.join(os.path.dirname(sys.argv[0]), "../host"))

import ddprintutil
from ddprofile import MatProfile, PrinterProfile, NozzleProfile

#########################################################################################

# Straight-line equation, y = x*m + y0 
def sleY(x, m, y0):
    return m*x + y0

# Straight-line equation, x = y/m + x0
def sleX(y, m, x0):
    return y/m + x0

if __name__ == "__main__":

    argParser = argparse.ArgumentParser(description='%s - plot ddPrint material profiles.' % os.path.basename(sys.argv[0]))

    ddargs.addPrinterArgument(argParser)
    ddargs.addNozzleArgument(argParser)
    ddargs.addMatArgument(argParser)

    argParser.add_argument("smat", help="Material profile(s) to plot.", nargs='+')

    args = argParser.parse_args()

    pp = PrinterProfile(args.printer)
    nozzle = NozzleProfile(args.nozzle)
    nozzleDiam = nozzle.getSizeI()

    tempGraphs = []
    pwmGraphs = []

    bp = MatProfile(
            name=args.mat, smatName=None,
            printerName=args.printer,
            hwVersion=pp.getHwVersionI(),
            nozzleDiam=nozzleDiam)

    baseTemp = bp.getHotendBaseTemp()
    goodTemp = bp.getHotendGoodTemp()
    maxTemp = bp.getHotendMaxTemp()

    windowTitle = "Mat. Profile"

    for specificProfile in args.smat:

        windowTitle += " " + specificProfile

        mp = MatProfile(
                name=args.mat, smatName=specificProfile,
                printerName=args.printer,
                hwVersion=pp.getHwVersionI(),
                nozzleDiam=nozzleDiam)

        flowrateData = mp.getFlowrateData()
        hasPrintingValues = "P0tempPrint" in flowrateData

        #
        # Two into-air graphs, flowrate vs temperature and flowrate vs pwm
        #
        # Into-air graph, flowrate at p0Temp
        fr0 = mp.getFR0pwm()

        # Into-air graph, temperature
        ktemp = mp.getKtemp()
        p0Temp = mp.getP0temp()

        # Into-air graph, pwm
        kpwm = mp.getKpwm()
        p0Pwm = mp.getP0pwm()

        xTemp = [baseTemp, maxTemp]

        frBaseTemp = sleY(baseTemp-p0Temp, ktemp, fr0)
        frMaxTemp = sleY(maxTemp-p0Temp, ktemp, fr0)
        yAirTemp = [frBaseTemp, frMaxTemp]

        tempGraphs.append((xTemp, yAirTemp, specificProfile+" air", (p0Temp, fr0), None))

        pwmMin1 = pwmMin2 = sleX(frBaseTemp-fr0, kpwm, p0Pwm)
        pwmMax1 = pwmMax2 = sleX(frMaxTemp-fr0, kpwm, p0Pwm)

        xAirPwm = [pwmMin1, pwmMax1]
        yAirPwm = [frBaseTemp, frMaxTemp]

        pwmGraphs.append((xAirPwm, yAirPwm, specificProfile+" air", (p0Pwm, fr0)))

        if hasPrintingValues:

            #
            # Two print graphs, flowrate vs temperature and flowrate vs pwm
            #
            p0PwmPrint = mp.getP0pwmPrint()
            p0TempPrint = mp.getP0tempPrint()
            fr0Print = mp.getFR0pwmPrint()

            frBaseTemp = sleY(baseTemp-p0TempPrint, ktemp, fr0Print)
            frMaxTemp = sleY(maxTemp-p0TempPrint, ktemp, fr0Print)
            yPrintTemp = [frBaseTemp, frMaxTemp]

            fill = None
            if len(args.smat) == 1:
                fill = (xTemp, (yAirTemp), (yPrintTemp))

            tempGraphs.append((xTemp, yPrintTemp, specificProfile+" print", (p0TempPrint, fr0Print), fill))

            pwmMin2 = sleX(frBaseTemp-fr0Print, kpwm, p0PwmPrint)
            pwmMax2 = sleX(frMaxTemp-fr0Print, kpwm, p0PwmPrint)

            xPrintPwm = [pwmMin2, pwmMax2]
            yPrintPwm = [frBaseTemp, frMaxTemp]

            fill = (xTemp, (), ())

            pwmGraphs.append((xPrintPwm, yPrintPwm, specificProfile+" print", (p0PwmPrint, fr0Print)))

    plt.figure(windowTitle, figsize=(10, 8))
    plt.subplots_adjust(hspace=0.5)

    ax = plt.subplot(2,1,1)
    plt.title("Printer: %s, nozzle: %.2f, generic mat: %s\nFlowrate vs Temperature" %
        (args.printer, nozzleDiam, args.mat))
    plt.xlabel(u'temp [°C]')
    plt.ylabel(u'Flowrate [mm³/s]')
    plt.grid()
    plt.axvline(x=baseTemp, color='grey', linestyle=':')
    plt.axvline(x=goodTemp, color='r', linestyle=':')
    plt.axvline(x=maxTemp, color='grey', linestyle=':')

    for (p1, p2, specificProfile, marker, fill) in tempGraphs:
        plt.plot(p1, p2, label=specificProfile)
        plt.plot(marker[0], marker[1], 'x', markeredgecolor="grey")
        if fill:
            plt.fill_between(fill[0], fill[1], fill[2], color='grey', alpha='0.2')

    plt.legend(loc="upper left")

    border = (baseTemp + maxTemp)/40
    ax.set_xlim(baseTemp-border, maxTemp+border)

    plt.ylim(0)

    ax = plt.subplot(2,1,2) #  sharex=plot1)
    plt.title("Flowrate vs PWM")
    plt.xlabel(u'PWM')
    plt.ylabel(u'Flowrate [mm³/s]')
    plt.grid()
    
    for (p1, p2, specificProfile, marker) in pwmGraphs:
        plt.plot(p1, p2, label=specificProfile)
        plt.plot(marker[0], marker[1], 'x', markeredgecolor="grey")

    plt.legend(loc="upper left")

    pwmMin = min(pwmMin1, pwmMin2)
    pwmMax = max(pwmMax1, pwmMax2)
    border = (pwmMin + pwmMax)/40
    ax.set_xlim(pwmMin-border, pwmMax+border)

    plt.show()











