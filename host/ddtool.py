#!/usr/bin/python
# -*- coding: utf-8 -*-
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

import pprint, sys, os
import argparse, time

# import ddprintutil as util, gcodeparser, packedvalue, ddhome
# import ddtest

# from ddplanner import Planner
# from ddprinter import Printer, RxTimeout

def filamentTool(fn):
    assert(0)

#
# Print some info's about gcode
#
def gcodeTool(fn):
   
    i = 0
    settings = argparse.Namespace(generator="")
    for line in open(fn).readlines():
        l = line.strip()

        if l.startswith(";"):
            tokens = l.split()
            ti = 1
            while ti < len(tokens):
                token = tokens[ti].lower()
                valtokens = token.split(",")
                if len(valtokens) >= 2:
                    v = valtokens[1:]

                    while ti+1 < len(tokens):
                        v.append(tokens[ti])
                        ti += 1
                    settings.__setattr__(valtokens[0].lower(), v)
                    break
                if token == "generated":
                    settings.generator = l
                    break

                ti += 1

        i += 1

    # print "gcode settings:",
    # pprint.pprint(settings)
    # print "lh:", settings.layerheight
    # print "starting:", settings.startinggcode

    lh = float(settings.layerheight[0])
    ew = float(settings.extruderwidth[0])
    spd = float(settings.defaultspeed[0]) / 60

    xyspd = float(settings.rapidxyspeed[0]) / 60
    zspd = float(settings.rapidzspeed[0]) / 60

    print("\nGCode info's:")
    print("-------------")
    print("Model (STL) : %s" % settings.applytomodels)
    print("Generator   : %s" % settings.generator)
    print("LayerHeight : %.2f mm" % lh)
    print("ExtrudeWidth: %.2f mm" % ew)
    print("Speed       : %.2f mm/s" % spd)
    print("Travelspeed : XY: %.2f Z: %.2f mm/s" % (xyspd, zspd))
    print("starting cod: \n  %s" % "".join(settings.startinggcode).replace(";", "\n  ;"))
    print("\nMax flowrate: %.2f mm³/s" % (lh*ew*spd))


if len(sys.argv) == 2:

    fn = sys.argv[1]

    ext = os.path.splitext(fn)[-1]

    if ext == ".json":
        # filamenttool
        filamentTool(fn)
    elif ext == ".gcode":
        # gcodetool
        gcodeTool(fn)
    else:
        print("unknown file/extension", ext)


