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

import ddprintutil as util
from ddprofile import MatProfile, PrinterProfile, NozzleProfile

#########################################################################################

# Straight-line equation, y = x*m + y0 
def nu_sleY(x, m, y0):
    return m*x + y0

# Straight-line equation, x = y/m + x0
def nu_sleX(y, m, x0):
    return y/m + x0

# Plot additional data for documentation purposes
plotForDoc = 0
# workingPoint = 0.5
# workingPoint = 0.3
workingPoint = 0.8
def docFrLookup(fr, sle, label, minTemp=0, maxTemp=0):

    t = sle.x(fr)
    print(sle)
    print("t:", t)

    plt.plot([0, t], [fr, fr], 'm', linewidth=3)
    plt.plot(t, fr, 'x', markeredgecolor="m")
    plt.plot([t, t], [0, fr], 'm:', linewidth=3)

    if label == "T1" or plotForDoc == 2:
      ax.annotate(f"flowrate={fr}mm³/s",
            xy=(185, fr), xycoords='data',
            xytext=(0.1, 0.7), textcoords='axes fraction',
            arrowprops=dict(shrink=0.05, width=3), horizontalalignment='center')

    lx = 0.35
    if label == "T0":
        lx = 0.65
    ax.annotate(label+f"={round(t)}°C",
            xy=(t, 0), xycoords='data',
            xytext=(lx, -0.2), textcoords='axes fraction',
            arrowprops=dict(shrink=0.05, width=3), horizontalalignment='center')

    # Plot workingPoint graph
    plt.plot([minTemp, maxTemp], [sle.y(minTemp), sle.y(maxTemp)], label=f"Workingpoint = {workingPoint}")

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
        (sleAir, pwmSle) = mp.getFrSLE()

        # Into-air graph, flowrate at p0Temp
        fr0 = sleAir.y1 # mp.getFR0pwm()

        # Into-air graph, temperature
        ktemp = sleAir.m # mp.getKtemp()
        p0Temp = sleAir.x1 # mp.getP0temp()

        # Into-air graph, pwm
        kpwm = pwmSle.m # mp.getKpwm()
        p0Pwm = pwmSle.x1 # mp.getP0pwm()

        xTemp = [baseTemp, maxTemp]

        frBaseTemp = sleAir.y(baseTemp) # sleY(baseTemp-p0Temp, ktemp, fr0)
        frMaxTemp = sleAir.y(maxTemp) # sleY(maxTemp-p0Temp, ktemp, fr0)
        yAirTemp = [frBaseTemp, frMaxTemp]

        tempGraphs.append((xTemp, yAirTemp, specificProfile+" air", (p0Temp, fr0), None))

        pwmMin1 = pwmMin2 = pwmSle.x(frBaseTemp) # sleX(frBaseTemp-fr0, kpwm, p0Pwm)
        pwmMax1 = pwmMax2 = pwmSle.x(frMaxTemp) # sleX(frMaxTemp-fr0, kpwm, p0Pwm)

        xAirPwm = [pwmMin1, pwmMax1]
        yAirPwm = [frBaseTemp, frMaxTemp]

        pwmGraphs.append((xAirPwm, yAirPwm, specificProfile+" air", (p0Pwm, fr0)))

        if hasPrintingValues:

            #
            # Two print graphs, flowrate vs temperature and flowrate vs pwm
            #
            (slePrint, pwmSlePrint) = mp.getFrSLEPrint()

            p0TempPrint = slePrint.x1 # mp.getP0tempPrint()
            p0PwmPrint = pwmSlePrint.x1 # mp.getP0pwmPrint()
            fr0Print = slePrint.y1 # mp.getFR0pwmPrint()

            frBaseTemp = slePrint.y(baseTemp) # sleY(baseTemp-p0TempPrint, ktemp, fr0Print)
            frMaxTemp = slePrint.y(maxTemp) # sleY(maxTemp-p0TempPrint, ktemp, fr0Print)
            yPrintTemp = [frBaseTemp, frMaxTemp]

            fill = None
            if len(args.smat) == 1:
                fill = (xTemp, (yAirTemp), (yPrintTemp))

            tempGraphs.append((xTemp, yPrintTemp, specificProfile+" print", (p0TempPrint, fr0Print), fill))

            pwmMin2 = pwmSlePrint.x(frBaseTemp) # sleX(frBaseTemp-fr0Print, kpwm, p0PwmPrint)
            pwmMax2 = pwmSlePrint.x(frMaxTemp) # sleX(frMaxTemp-fr0Print, kpwm, p0PwmPrint)

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

    ###############################
    # To create graphs for documentation:
    if plotForDoc == 1:
        docFrLookup(9, sleAir, "T1")
        docFrLookup(9, slePrint, "T0")

    elif plotForDoc == 2:
        sleWp = util.SLE(x1=0, y1=slePrint.c + (sleAir.c-slePrint.c)*workingPoint, m=sleAir.m)
        docFrLookup(9, sleWp, "Twp", baseTemp, maxTemp)
    ###############################

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











