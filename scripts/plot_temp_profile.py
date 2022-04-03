# -*- coding: utf-8 -*-
#
#/*
# This file is part of ddprint - a 3D printer firmware.
# 
# Copyright 2022 erwin.rieger@ibrieger.de
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
# Program to plot ddprint autotemp temperature profile (created with "ddprint -logat=<logfile>...").
#

import sys, os, json
import argparse

import matplotlib.pyplot as plt

import ddprintutil
from ddprofile import MatProfile, PrinterProfile, NozzleProfile

#########################################################################################

argParser = argparse.ArgumentParser(description='%s - plot ddPrint temperature profile.' % os.path.basename(sys.argv[0]))
argParser.add_argument("atlog", help="Temperature profile to plot (created with 'ddprint -logat=<logfile>...').")
args = argParser.parse_args()

with open(args.atlog) as f:

    atdata = json.load(f)
    atArgs = atdata["args"]

    pp = PrinterProfile(atArgs["printer"])

    nozzle = NozzleProfile(atArgs["nozzle"])
    nozzleDiam = nozzle.getSizeI()

    bp = MatProfile(
        name=atArgs["mat"], smatName=None,
        printerName=atArgs["printer"],
        hwVersion=pp.getHwVersionI(),
        nozzleDiam=nozzleDiam)

    baseTemp = bp.getHotendBaseTemp()
    goodTemp = bp.getHotendGoodTemp()

    x = []
    y = []

    lastTemp = 0
    for (t, temp) in atdata["autotemp"]:

        if temp != lastTemp:
            x.append(t/60)
            y.append(lastTemp)

        x.append(t/60)
        y.append(temp)

        lastTemp = temp

    x.append(x[-1])
    y.append(0)

    windowTitle = "AutoTemp temperature profile"
    plt.figure(windowTitle)

    ax = plt.subplot(1,1,1)
    plt.title("Temp. profile, Model: %s, WorkingPoint: %.2f" % (os.path.basename(atArgs["gfile"]), atArgs["workingPoint"]))
    plt.xlabel(u'Time [min]')
    plt.ylabel(u'Temperature [°C]')
    plt.grid()
    plt.grid(which='minor', linestyle='-', alpha=0.5)
    plt.minorticks_on()
    ax.yaxis.set_minor_locator(plt.MultipleLocator(2.5))

    plt.plot(x, y, label="autotemp")
    plt.axhline(y=goodTemp, color='r', linestyle='dotted', label="goodTemp")
    plt.axhline(y=baseTemp, color='r', linestyle='dashed', label="baseTemp")
    plt.legend(loc="lower center")

    ax.set_ylim(baseTemp-10)

    plt.show()






