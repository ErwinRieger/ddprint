# -*- coding: utf-8 -*-
#
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
#

import math
import packedvalue
import shutil, os

from ddprintcommands import CmdUnknown
from ddprintconstants import dimNames
from ddprintconstants import X_AXIS, Y_AXIS, Z_AXIS, A_AXIS, B_AXIS, Uint8Max
from ddconfig import *
from move import TravelMove, PrintMove
from ddvector import Vector, vectorDistance, vectorLength

import ddprintutil as util

import dddumbui

#degug
import movingavg

############################################################################
############################################################################

class GcodeException(Exception):
    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return "Gcode parser error: " + self.msg

#####################################################################################################################################################
# Move-type, Matrix aller möglichen kombinationen:
# X Y Z E Movetype
# - - - - --------
# 1 0 0 0 travel
# 0 1 0 0 travel
# 1 1 0 0 travel
# 0 0 1 0 travel
# 1 0 1 0 travel
# 0 1 1 0 travel
# 1 1 1 0 travel
# 0 0 0 1 travel
# 1 0 0 1 print
# 0 1 0 1 print
# 1 1 0 1 print
# 0 0 1 1 invalid
# 1 0 1 1 invalid
# 0 1 1 1 invalid
# 1 1 1 1 invalid
#
def _isHeadMove(displacement_vector):
    return displacement_vector[X_AXIS] or displacement_vector[Y_AXIS] or displacement_vector[Z_AXIS]

def _isExtrudingMove(displacement_vector, extruder):
    return displacement_vector[extruder]

def _isZMove(displacement_vector):
    return displacement_vector[Z_AXIS]

def isExtrudingMove(displacement_vector):
    return _isExtrudingMove(displacement_vector, A_AXIS) or _isExtrudingMove(displacement_vector, B_AXIS)

def isPrintMove(displacement_vector):

    printMove = _isHeadMove(displacement_vector) and isExtrudingMove(displacement_vector)

    # Printmoves with reverting E are invalid
    # Note: some Cura versions generate gcode with a initial G11 and without the matching G10 for it,
    # this triggers this test.
    if printMove and ((displacement_vector[A_AXIS] < 0) or (displacement_vector[B_AXIS] < 0)):
        print "ERROR: printmove with reversing E:", displacement_vector.vv
        assert(0)

    # Printmoves in X, Y AND Z are not tested
    if printMove and _isZMove(displacement_vector):
        print "ERROR: printmove with z-part:", displacement_vector.vv
        assert(0)

    return printMove

#####################################################################################################################################################

class UM2GcodeParser: 

    def __init__(self, planner, logger=None, travelMovesOnly=False):

        if logger:
            self.logger = logger
        else:
            self.logger = dddumbui.DumbGui()

        self.reset()

        self.commands = {
                # G0 and G1 are the same
                "G0": self.g0,
                "G1": self.g0,
                "G4": self.g4_dwell,
                "G10": self.g10_retract,
                "G11": self.g11_retract_recover,
                "G21": self.g21_metric_values,
                "G28": self.g28_home,
                "G29": self.g29_autolevel,
                "G80": self.g80_bed_leveling,
                "G90": self.g90_abs_values,
                "G91": self.g91_rel_values,
                "G92": self.g92_set_pos,
                "M25": self.m25_stop_reading,
                "M82": self.m82_extruder_abs_values,
                "M83": self.m83_extruder_relative_values,
                "M84": self.m84_disable_motors,
                "M104": self.m104_extruder_temp,
                "M106": self.m106_fan_on,
                "M107": self.m107_fan_off,
                "M109": self.m109_extruder_temp_wait,
                "M117": self.m117_message,
                "M140": self.m140_bed_temp,
                "M190": self.m190_bed_temp_wait,
                "M201": self.m201_set_accel,
                "M203": self.m203_set_speed,
                "M204": self.m204_set_accel,
                "M205": self.m205_set_jerk,
                "M501": self.m501_reset_params,
                "M502": self.m502_reset_params,
                "M900": self.m900_set_kAdvance,
                "M901": self.m901_set_wp,
                "M907": self.m907_motor_current,
                "T0": self.t0,
                "U": self.unknown, # unknown command for testing purposes
                }

        self.steps_per_mm = planner.printer.printerProfile.getStepsPerMMVectorI()
        self.mm_per_step = map(lambda dim: 1.0 / self.steps_per_mm[dim], range(5))
        self.maxFeedrateVector = planner.printer.printerProfile.getMaxFeedrateVectorI()

        self.planner = planner

    def reset(self):

        self.position = util.MyPoint()
        self.feedrate = None

        self.absolute = {
                X_AXIS: True,
                Y_AXIS: True,
                Z_AXIS: True,
                A_AXIS: True,
                B_AXIS: True,
                }

        self.layerPart = "unknown"

    # Set current virtual printer position
    def setPos(self, point):
        self.position = point.copy()

    def getPos(self):
        return self.position.copy()

    # Preprocess gcode to
    # * determine number of parts
    # * compute amount of preload data
    # 
    # xxx todo: cache preprocesse data for later real parsing?
    #
    # preload: ratio between time used to move head and time needed to download data
    #
    # to estimate move time: worst case for movetime: a move without
    # acceleration/deceleration -> simply use the commanded
    # feedrate.
    #
    # to estimate download time: 
    # size of data packages depend on amount of acceleration/decelleration, but those
    # moves are also slower...
    # we use a simple estimation here: 2 times the size of a small packet
    #
    # todo: implement relative mode
    # 
    # Return: number of gcode lines to preload.
    # 

    """
    Packet around data payload:

        *  1 byte, SOH
        *  3 bytes, header (linenumber, cmd, payloadsize)
        *  3 bytes, flags + checksum

        -> 7 bytes

    Move payload:

        * 2 + 1 + 5*4 + 2  bytes header, flags, axes
        *               2  bytes E-timer
        *               1  byte avg window size
        *               4  bytes linear timer, decel pulses
        *               2  bytes accel timer
        *              [n] bytes accel timer delta's
        *               2  bytes decel timer
        *              [n] bytes decel timer delta's

        -> 36 bytes 

    Entire packet about 43 bytes -> round up to 50 bytes
    """
    def preParse(self, fn, nu_baudrate):

        # lenavg = movingavg.MovingAvg(100)

        # Copy file to prevet problems with overwritten files
        # while reading.
        tmpfname = ("/tmp/%d_" % id(fn)) + os.path.basename(fn)
        shutil.copyfile(fn, tmpfname)

        f = open(tmpfname)

        self.logger.log("Unlinking temp. copy of gcode input: ", tmpfname)
        os.unlink(tmpfname)

        # preload
        # moveTime = 0.0
        # downloadTime = 0.0
        # feedrate = 0.0
        pos = [0.0, 0.0, 0.0] # use vector 3 here
        lineNumber = 0
        preloadLines = 0
        preloadMinLen = 0.15

        # Time to send one move at given baudrate:
        # * assume we send 10 bits for a byte
        # * multiply by 5 to account for raw moves
        # bitsPerMove = 5.0 * 50.0 * 10.0
        # timePerMove = bitsPerMove / baudrate

        self.logger.log("Pre-parsing ", tmpfname)

        for line in f:

            lineNumber += 1
            line = line.strip()

            if line and not line.startswith(";"): # Gcode comments

                # Look for movement codes
                tokens = line.split()
                cmd = tokens[0]

                if cmd == "G1" or cmd == "G0":
                
                    values = self.getValues(tokens[1:])

                    # if 'F' in values:
                        # feedrate = values['F']

                    if not ("X" in values or "Y" in values or "Z" in values):
                        # Nothing to move, F-Only gcode or invalid
                        continue

                    displacement_vector = [0.0, 0.0, 0.0]
                    # newPos = [0.0, 0.0, 0.0]

                    for dim in range(3):

                        dimC = dimNames[dim]
                        if dimC not in values:
                            continue

                        # if self.absolute[dim]:
                        rDiff = values[dimC] - pos[dim]
                        # newPos[dim] = values[dimC]
                        pos[dim] = values[dimC]
                        # else:
                            # rDiff = values[dimC]
                            # newPos[dim] += values[dimC]

                        displacement_vector[dim] = rDiff

                    # Check if zero length:
                    if displacement_vector == [0.0, 0.0, 0.0]:
                        # Skip this empty move.
                        continue

                    # Todo: Constrain feedrate to max values

                    l = vectorLength(displacement_vector)

                    # lenavg.add(l)

                    print "move distance:", displacement_vector, l # , feedrate
                    # print "move distance:", l, lenavg.mean()

                    if l < preloadMinLen:
                        preloadLines += 1
                   
                    """
                    t = l / feedrate

                    moveTime += t

                    # Weight small moves
                    d = 1.0 - l
                    w = 1.0
                    if d > 0:
                        # distance smaller than 1.0
                        # shorter moves have a higher d value, max 1.0
                        w = 1.0 + (d*5.0)

                    downloadTime += timePerMove * w

                    # print "t, tmove, tdownload:", t, moveTime, downloadTime

                    if downloadTime > moveTime:
                        preloadLines = lineNumber
                    """

                    # pos = newPos

                elif cmd == "G2" or cmd == "G3":

                    print "line:", line
                    assert(0) # not implemented

                elif cmd == "G92":

                    values = self.getValues(tokens[1:])

                    if not ("X" in values or "Y" in values or "Z" in values):
                        # No head move, E-Only 
                        continue

                    for dim in range(3):

                        dimC = dimNames[dim]
                        if dimC not in values:
                            continue

                        pos[dim] = values[dimC]

        f.seek(0) # rewind

        # if downloadTime < moveTime:
            # print "downloadTime %.2f smaller than printing time %.2f, can use preload (#lines: %d)." % (downloadTime, moveTime, preloadLines)
        # else:
            # print "downloadTime %.2f greater than printing time %.2f, no preload possible (#lines: %d)." % (downloadTime, moveTime, preloadLines)

        pl = max(preloadLines, 1000)
        print "Using preload (preloadLines: %d): %d (%.1f %%)" % (preloadLines, pl, preloadLines*100.0/lineNumber)
        # XXXXX hack weil download von stopfen-4mal.gcode ewig zum download dauert...
        # return (f, max(min(preloadLines, 10000), 1000))
        return (f, pl)

    def execute_line(self, rawLine):

        # print " execute_line:", rawLine

        line = rawLine.strip()
        if line:
            tokens = line.split()

            cmd = tokens[0]
            if cmd.startswith(";"):

                upperLine = line.upper()

                if upperLine.endswith("INFILL"):
                    # print "gcodeparser: Starting infill..."
                    self.layerPart = "infill"
                elif upperLine.endswith("PERIMETER"):
                    # print "gcodeparser: Starting perimeter..."
                    self.layerPart = "perimeter"
                elif "SUPPORT" in upperLine:
                    # print "gcodeparser: Starting support..."
                    self.layerPart = "support"
                elif upperLine.endswith("BRIDGE"):
                    # print "gcodeparser: Starting bridge..."
                    self.layerPart = "bridge"
                elif upperLine.endswith("GAP FILL"):
                    # print "gcodeparser: Starting gapfill..."
                    self.layerPart = "gapfill"
                elif upperLine.endswith("TOOL"):
                    # print "gcodeparser: Starting tool..."
                    self.layerPart = "tool"
                elif upperLine.endswith("SINGLE EXTRUSION"):
                    # print "gcodeparser: Starting single extrusion..."
                    self.layerPart = "single extrusion"
                elif upperLine.endswith("SKIRT"):
                    self.layerPart = "skirt"
                elif upperLine.endswith("THIN WALL"):
                    self.layerPart = "thinwall"
                elif upperLine.endswith("CUSTOM"):
                    self.layerPart = "custom"
                elif upperLine.endswith("SUMMARY"):
                    self.layerPart = "unknown"
                elif upperLine.startswith(";Z:"):
                    self.layerPart = "unknown"
                elif len(tokens) > 1 and tokens[1].upper() in [ "PURGING:", "PROCESS", "FEATURE" ]:
                    pass
                else:
                    if not (("LAYER" in upperLine) or ("TOOL" in upperLine) or ("WIDTH" in upperLine) or ("HEIGHT:" in upperLine)):

                        self.logger.log("gcodeparser comment:", line)

                        # Log this info into printlog, too. Dont log values that end 
                        # with ",0" and ",-1" to save space.
                        if not (line.endswith(",0") or line.endswith(",0")):
                            self.logger.logPrintLog(rawLine)

                        if self.layerPart == "infill":
                            print "unhandled comment type while infill active, gcodeparser comment:", line
                            assert(0)

                # print "skipping comment: ", line
                return

            # print "line:", tokens
            try:
                meth = self.commands[cmd]
            except KeyError:
                self.logger.log("GCode '%s' ('%s') unknown!" % (cmd, line))
                raise

            if cmd not in ["M117"]:
                values = self.getValues(tokens[1:])
                meth(line, values)
            else:
                meth(line)

    def getValues(self, tokens):

        values = {}
        for param in tokens:

            valueChar = param[0].upper()

            if valueChar == ";":
                # Skip rest of line/comment
                break

            rest = param[1:]

            factor = 1.0

            if valueChar == "E":
                # print "replacing ", param, " --> A", float(param[1:]) * factor
                valueChar = "A"

            if valueChar == "F":
                # print "replace feedrate in mm/min with mm/sec..."
                factor = 1.0 / 60

            if not rest:
                # Param without value
                continue

            try:
                values[valueChar] = float(rest) * factor
            except ValueError:
                raise GcodeException("Error converting '%s' to float!" % rest)

        return values

    def m25_stop_reading(self, line, values):
        print "ignoring m25 (stop reading)"

    def m82_extruder_abs_values(self, line, values):

        self.absolute[A_AXIS] = True
        self.absolute[B_AXIS] = True

    def m83_extruder_relative_values(self, line, values):
        
        self.absolute[A_AXIS] = False
        self.absolute[B_AXIS] = False

    def m84_disable_motors(self, line, values):
        self.logger.log("ignoring m84 (disable steppers)")

    def m104_extruder_temp(self, line, values):
        self.logger.log("ignoring m104 (extruder temp)")

    def m106_fan_on(self, line, values):

        # print "m106_fan_on", values
        fanSpeed = int(values["S"])

        blipTime = 0

        if "B" in values:
            blipTime = int(values["B"] * 1000)

        # self.planner.addSynchronizedCommand(CmdSyncFanSpeed, p1=packedvalue.uint8_t(fanSpeed))
        self.planner.fanOn(fanSpeed, min(blipTime, Uint8Max))

    def m107_fan_off(self, line, values):
        # print "m107_fan_off", values
        self.planner.fanOn(0, 0)

    def m109_extruder_temp_wait(self, line, values):
        self.logger.log("ignoring m109 (wait for extruder temp)")

    def m117_message(self, line):
        self.logger.log("m117_message: ", line)

    def m140_bed_temp(self, line, values):
        self.logger.log("ignoring m140 (bed temp)")

    def m190_bed_temp_wait(self, line, values):
        self.logger.log("ignoring m190 (wait for bed temp)")

    def m201_set_accel(self, line, values):
        self.logger.log("ignoring m201 (set acceleration)")

    def m203_set_speed(self, line, values):
        self.logger.log("ignoring m203 (set max speed)")

    def m204_set_accel(self, line, values):
        self.logger.log("ignoring m204 (set acceleration)")

    def m205_set_jerk(self, line, values):
        self.logger.log("ignoring m205 (set acceleration)")

    def m501_reset_params(self, line, values):
        self.logger.log("ignoring m501 (reset params)")

    def m502_reset_params(self, line, values):
        self.logger.log("ignoring m502 (reset params)")

    def m900_set_kAdvance(self, line, values):
        self.planner.m900(values)

    # Experimental gcode, set 'workingpoint' [0..1]
    def m901_set_wp(self, line, values):
        self.planner.m901(values)

    def m907_motor_current(self, line, values):
        self.logger.log("ignoring m907 (motor current)")

    def t0(self, line, values):
        self.logger.log("ignoring t0 (tool select)")

    def g4_dwell(self, line, values):
        if "P" in values:
            self.logger.log("dwell, ", values["P"], " ms")
            self.planner.dwellMS(values["P"])
        elif "S" in values:
            self.logger.log("dwell, ", values["S"], " s")
            self.planner.dwellMS(values["S"] * 1000)
        else:
            # unknown parameter
            assert(0)

    # Firmware Retraction
    def g10_retract(self, line, values):
        # print "g10_retract", values

        current_position = self.getPos()

        rl = - self.planner.printer.printerProfile.getRetractLength()

        current_position[A_AXIS] += rl

        self.planner.addMove(TravelMove(
            line,
            displacement_vector=Vector([0.0, 0.0, 0.0, rl, 0.0]),
            displacement_vector_steps=[0.0, 0.0, 0.0, rl * self.steps_per_mm[A_AXIS], 0.0],
            feedrate=min(self.planner.printer.printerProfile.getRetractFeedrate(), self.planner.printer.printerProfile.getMaxFeedrateI(3)),
            layerPart=self.layerPart,
            pos = self.getPos(),
            printerProfile = self.planner.printer.printerProfile,
            ))

        self.setPos(current_position)
            
    # Firmware Retraction
    def g11_retract_recover(self, line, values):
        # print "g11_retract_recover", values

        current_position = self.getPos()

        rl = self.planner.printer.printerProfile.getRetractLength()

        current_position[A_AXIS] += rl

        self.planner.addMove(TravelMove(
            line,
            displacement_vector=Vector([0.0, 0.0, 0.0, rl, 0.0]),
            displacement_vector_steps=[0.0, 0.0, 0.0, rl * self.steps_per_mm[A_AXIS], 0.0],
            feedrate=min(self.planner.printer.printerProfile.getRetractFeedrate(), self.planner.printer.printerProfile.getMaxFeedrateI(3)),
            layerPart=self.layerPart,
            pos = self.getPos(),
            printerProfile = self.planner.printer.printerProfile,
            ))

        self.setPos(current_position)

    def g21_metric_values(self, line, values):
        # We're always using metric values...
        print "ignoring g21 (metric values)"

    # Homing is done by the host part of ddprint. Because it would be
    # difficult get things in the right order, g28 is ignored and done
    # implicitly by ddprint.
    def g28_home(self, line, values):
        print "ignoring g28 (homing)"

    def g29_autolevel(self, line, values):
        # Autoleveling not implemented
        print "ignoring g29 (autolevel)"

    def g80_bed_leveling(self, line, values):
        print "ignoring g80 (bedlevel)"

    def g90_abs_values(self, line, values):

        for d in range(5):
            self.absolute[d] = True

    def g91_rel_values(self, line, values):

        for d in range(5):
            self.absolute[d] = False

    def g92_set_pos(self, line, values):

        realPos = self.getPos()
        for key in values:
            realPos[key] = values[key]

        self.setPos(realPos)

        self.planner.g92(values)

    def g0(self, line, values):

        # print "g0", values

        if 'F' in values:

            self.feedrate = values['F']

        feedrate = self.feedrate

        if not ("X" in values or "Y" in values or "Z" in values or "A" in values or "B" in values):
            # Nothing to move, F-Only gcode or invalid
            return

        curRealPos = self.getPos()
        newRealPos = curRealPos.copy()

        # print "curRealPos: ", newRealPos

        displacement_vector = Vector([0.0, 0.0, 0.0, 0.0, 0.0])
        displacement_vector_steps = [0.0, 0.0, 0.0, 0.0, 0.0]

        for dim in range(5):

            dimC = dimNames[dim]
            if dimC not in values:
                continue

            if self.absolute[dim]:
                rDiff = values[dimC] - curRealPos[dim]
                newRealPos[dim] = values[dimC]
            else:
                rDiff = values[dimC]
                newRealPos[dim] += values[dimC]

            displacement_vector[dim] = rDiff

            nSteps = rDiff * self.steps_per_mm[dim]
            displacement_vector_steps[dim] = nSteps


        # Check if zero length:
        if displacement_vector_steps == [0.0, 0.0, 0.0, 0.0, 0.0]:
            # Skip this empty move.
            return

        # Constrain feedrate to max values
        feedrateVector = displacement_vector._setLength(feedrate).constrain(self.maxFeedrateVector)
        if feedrateVector:
            feedrate = feedrateVector.length()

        if isPrintMove(displacement_vector):  #  and self.layerPart != "infill":
            self.planner.addMove(PrintMove(
                line,
                displacement_vector=displacement_vector,
                displacement_vector_steps=displacement_vector_steps,
                feedrate=feedrate, # mm/s
                layerPart=self.layerPart,
                maxAccelV = self.planner.advance.maxAxisAcceleration(self.layerPart != "infill"),
                pos = curRealPos,
                printerProfile = self.planner.printer.printerProfile,
                ))
        else:
            self.planner.addMove(TravelMove(
                line,
                displacement_vector=displacement_vector,
                displacement_vector_steps=displacement_vector_steps,
                feedrate=feedrate, # mm/s
                layerPart=self.layerPart,
                pos = curRealPos,
                printerProfile = self.planner.printer.printerProfile,
                ))
            
        # print "newRealPos: ", newRealPos
        self.setPos(newRealPos)

    def unknown(self, line, values):
        self.planner.addSynchronizedCommand(CmdUnknown, p1=packedvalue.uint8_t(values["X"]))









