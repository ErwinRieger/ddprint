# -*- coding: utf-8 -*-
#
# This file is part of ddprint - a direct drive 3D printer firmware.
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
import shutil, os, re

from ddprofile import MatProfile, PrinterProfile
from ddplanner import Planner
from ddprintcommands import CmdSyncFanSpeed, CmdUnknown, CmdDwellMS
from ddprintconstants import dimNames, GCODEUNKNOWN, GCODEULTI, GCODES3D
from ddconfig import *
from move import TravelMove, PrintMove
from ddvector import Vector, vectorDistance
from ddprintutil import X_AXIS, Y_AXIS, Z_AXIS, A_AXIS, B_AXIS, circaf

import ddprintutil as util

import dddumbui

############################################################################
############################################################################

CuraLayerRE = re.compile("; \s* LAYER \s* : \s* (\d+) $", re.VERBOSE)

#
# Get layer number from cura gcode comment
# Example:
# ;LAYER:0
#
def getCuraLayer(line):

    m = re.match(CuraLayerRE, line)

    if m:
        # Note: Cura layer numbering starts with 0
        layer = int(m.groups()[0])
        return layer

    return None


S3DLayerRE = re.compile("; \s layer \s (\d+), \s Z \s = \s (\d+ [\.,] \d+) $", re.VERBOSE)

#
# Get layer number from simplify3d gcode comment
# Example: 
# ; layer 1, Z = 0.225
#
def getSimplifyLayer(line):

    m = re.match(S3DLayerRE, line)

    if m:
        # Note: S3D layer numbering starts with 1
        layer = int(m.groups()[0])
        assert(layer > 0)
        return layer - 1

    return None

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

def isPrintMove(displacement_vector):

    printMove = _isHeadMove(displacement_vector) and (_isExtrudingMove(displacement_vector, A_AXIS) or _isExtrudingMove(displacement_vector, B_AXIS))

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

    __single = None 

    def __init__(self, logger=None):

        if UM2GcodeParser.__single:
            raise RuntimeError('A UM2GcodeParser already exists')

        UM2GcodeParser.__single = self

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
                "M204": self.m204_set_accel,
                "M501": self.m501_reset_params,
                "M502": self.m502_reset_params,
                "M900": self.m900_set_kAdvance,
                "M907": self.m907_motor_current,
                "T0": self.t0,
                "U": self.unknown, # unknown command for testing purposes
                }

        # Apply material flow parameter from material profile
        self.e_to_filament_length = MatProfile.getFlow() / 100.0

        self.steps_per_mm = PrinterProfile.getStepsPerMMVector()
        self.mm_per_step = map(lambda dim: 1.0 / self.steps_per_mm[dim], range(5))
        self.maxFeedrateVector = PrinterProfile.getMaxFeedrateVector()

        self.planner = Planner.get()

    @classmethod
    def get(cls):
        return cls.__single

    def reset(self):

        self.position = util.MyPoint()
        self.feedrate = None
        self.numParts = 1

        self.absolute = {
                X_AXIS: True,
                Y_AXIS: True,
                Z_AXIS: True,
                A_AXIS: True,
                B_AXIS: True,
                }

        # self.ultiGcodeFlavor = False
        # types : s3d, ulti, unknown
        self.gcodeType = GCODEUNKNOWN
        self.layerPart = "unknown"

    # Set current virtual printer position
    def setPos(self, point):
        self.position = point.copy()

    def getPos(self):
        return self.position.copy()

    def preParse(self, fn):

        # Copy file to prevet problems with overwritten files
        # while reading.
        tmpfname = ("/tmp/%d_" % id(fn)) + os.path.basename(fn)
        shutil.copyfile(fn, tmpfname)

        f = open(tmpfname)

        self.logger.log("Unlinking temp. copy of gcode input: ", tmpfname)
        os.unlink(tmpfname)

        self.numParts = 0

        self.logger.log("Pre-parsing ", fn, tmpfname)
        for line in f:
            line = line.strip()
            if line.startswith(";"):

                upperLine = line.upper()

                # Cura: "LAYER:"
                layerNum = getCuraLayer(line)
                if layerNum == 1:
                    self.numParts += 1
                    self.planner.newPart(self.numParts)

                # layerNum = getSimplifyLayer(line)
                # if layerNum == 1:
                    # self.numParts += 1
                    # self.planner.newPart(self.numParts)

                # Simplify3D: "; skirt "
                if upperLine.startswith("; SKIRT"):
                    self.numParts += 1
                    self.planner.newPart(self.numParts)

                # ;FLAVOR:UltiGCode
                elif "FLAVOR:ULTIGCODE" in upperLine:
                    self.gcodeType = GCODEULTI
                    # To compute extrude length from volume (see getValues()):
                    # V = A * h, h = V / A, A = pi/4 * diameter²
                    aFilament = MatProfile.getMatArea()
                    self.e_to_filament_length = self.e_to_filament_length / aFilament
                elif "SIMPLIFY3D" in upperLine:
                    self.gcodeType = GCODES3D


        self.logger.log("pre-parsing # parts:", self.numParts)
        f.seek(0) # rewind
        return f

    def execute_line(self, line):

        # print " execute_line:", line

        line = line.strip()
        if line:
            tokens = line.split()

            cmd = tokens[0]
            if cmd.startswith(";"):

                upperLine = line.upper()

                layerNum = getCuraLayer(line)
                if layerNum != None:
                    self.planner.layerChange(layerNum)
                    return

                layerNum = getSimplifyLayer(line)
                if layerNum != None:
                    self.planner.layerChange(layerNum)
                    return

                if upperLine.endswith("INFILL"):
                    # print "gcodeparser: Starting infill..."
                    self.layerPart = "infill"
                elif upperLine.endswith("PERIMETER"):
                    # print "gcodeparser: Starting perimeter..."
                    self.layerPart = "perimeter"
                elif upperLine.endswith("SUPPORT"):
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
                else:
                    if not (("LAYER" in upperLine) or ("TOOL" in upperLine)):
                        self.logger.log("gcodeparser: Unhandled comment:", line)
                        if self.layerPart == "infill":
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

            factor = 1

            # XXX assuming ultiGcode here !!! --> parse ;FLAVOR:UltiGCode
            if valueChar == "E":
                factor = self.e_to_filament_length
                # print "replacing ", param, " --> A", float(param[1:]) * factor
                valueChar = "A"

            if valueChar == "F":
                # print "replace feedrate in mm/min with mm/sec..."
                factor = 1.0 / 60

            if not rest:
                # Param without value
                continue

            # try:
                # values[valueChar] = int(rest) * factor
            # except ValueError:
            values[valueChar] = float(rest) * factor

        return values

    def m25_stop_reading(self, line, values):
        print "XXX todo implement M25", values

    def m82_extruder_abs_values(self, line, values):

        self.absolute[A_AXIS] = True
        self.absolute[B_AXIS] = True

    def m83_extruder_relative_values(self, line, values):
        
        self.absolute[A_AXIS] = False
        self.absolute[B_AXIS] = False

    def m84_disable_motors(self, line, values):
        self.logger.log("ignoring m84...")

    def m104_extruder_temp(self, line, values):
        self.logger.log("ignoring m104 (set extruder temp)...")

    def m106_fan_on(self, line, values):
        # print "m106_fan_on", values
        fanSpeed = (values["S"] * MatProfile.getFanPercent()) / 100

        # "Blip fan" for Cura (S3D supports blip fan)
        if fanSpeed < 50 and self.gcodeType==GCODEULTI:
            # Start fan with full power
            self.planner.addSynchronizedCommand(CmdSyncFanSpeed, p1=packedvalue.uint8_t(255))
            # Dwell 0.25s
            self.planner.addSynchronizedCommand(CmdDwellMS, p1=packedvalue.uint16_t(250))

        self.planner.addSynchronizedCommand(CmdSyncFanSpeed, p1=packedvalue.uint8_t(fanSpeed))

    def m107_fan_off(self, line, values):
        # print "m107_fan_off", values
        self.planner.addSynchronizedCommand(CmdSyncFanSpeed, p1=packedvalue.uint8_t(0))

    def m109_extruder_temp_wait(self, line, values):
        self.logger.log("ignoring m109 (wait for extruder temp)...")

    def m117_message(self, line):
        self.logger.log("m117_message: ", line)

    def m140_bed_temp(self, line, values):
        self.logger.log("ignoring m140...")

    def m190_bed_temp_wait(self, line, values):
        self.logger.log("ignoring m190 (wait for bed temp)...")

    def m204_set_accel(self, line, values):
        self.logger.log("ignoring m204 (set acceleration)...")

    def m501_reset_params(self, line, values):
        self.logger.log("ignoring m501 (reset params)...")

    def m502_reset_params(self, line, values):
        self.logger.log("ignoring m502 (reset params)...")

    def m900_set_kAdvance(self, line, values):

        self.planner.g900(values)

    def m907_motor_current(self, line, values):
        self.logger.log("ignoring m907...")

    def t0(self, line, values):
        self.logger.log("ignoring t0...")

    def g4_dwell(self, line, values):
        if "P" in values:
            self.logger.log("dwell, ", values["P"], "ms")
            self.planner.addSynchronizedCommand(CmdDwellMS, p1=packedvalue.uint16_t(values["P"]))
        elif "S" in values:
            self.planner.addSynchronizedCommand(CmdDwellMS, p1=packedvalue.uint16_t(values["S"] * 1000))
        else:
            # unknown parameter
            assert(0)

    def g10_retract(self, line, values):
        # print "g10_retract", values

        current_position = self.getPos()

        rl = - PrinterProfile.getRetractLength()

        current_position[A_AXIS] += rl

        self.planner.addMove(TravelMove(
            line,
            displacement_vector=Vector([0.0, 0.0, 0.0, rl, 0.0]),
            displacement_vector_steps=[0.0, 0.0, 0.0, rl * self.steps_per_mm[A_AXIS], 0.0],
            feedrate=min(PrinterProfile.getRetractFeedrate(), PrinterProfile.getMaxFeedrate(3)),
            layerPart=self.layerPart,
            ))

        self.setPos(current_position)
            
    def g11_retract_recover(self, line, values):
        # print "g11_retract_recover", values

        current_position = self.getPos()

        rl = PrinterProfile.getRetractLength()

        current_position[A_AXIS] += rl

        self.planner.addMove(TravelMove(
            line,
            displacement_vector=Vector([0.0, 0.0, 0.0, rl, 0.0]),
            displacement_vector_steps=[0.0, 0.0, 0.0, rl * self.steps_per_mm[A_AXIS], 0.0],
            feedrate=min(PrinterProfile.getRetractFeedrate(), PrinterProfile.getMaxFeedrate(3)),
            layerPart=self.layerPart,
            ))

        self.setPos(current_position)

    def g21_metric_values(self, line, values):
        # We're always using metric values...
        pass

    def g28_home(self, line, values):
        # Homing is done implicitly...
        pass

    def g29_autolevel(self, line, values):
        # Autoleveling not implemented
        pass

    def g80_bed_leveling(self, line, values):
        print "ignoring g80..."

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
                ))
        else:
            self.planner.addMove(TravelMove(
                line,
                displacement_vector=displacement_vector,
                displacement_vector_steps=displacement_vector_steps,
                feedrate=feedrate, # mm/s
                layerPart=self.layerPart,
                ))
            
        # print "newRealPos: ", newRealPos
        self.setPos(newRealPos)

    def unknown(self, line, values):
        self.planner.addSynchronizedCommand(CmdUnknown, p1=packedvalue.uint8_t(values["X"]))









