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
import shutil, os

from ddprofile import MatProfile, PrinterProfile
from ddplanner import Planner
from ddprintcommands import CmdSyncFanSpeed, CmdUnknown, CmdDwellMS
from ddprintconstants import dimNames
from ddconfig import *
from move import TravelMove, PrintMove
from ddvector import Vector, vectorDistance
from ddprintutil import X_AXIS, Y_AXIS, Z_AXIS, A_AXIS, B_AXIS, circaf

import ddprintutil as util

############################################################################

# Get layer number from cura gcode comment
def getCuraLayer(line):

    return int(line.split(":")[1])

# Get layer number from simplify3d gcode comment
def getSimplifyLayer(line):

    lstr = line.split(" ")[2].split(",")[0]
    if lstr != "end":
        return int(lstr)

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

    if printMove and _isZMove(displacement_vector):
        print "ERROR: printmove with z-part:", displacement_vector.vv
        assert(0)

    return printMove

#####################################################################################################################################################

class UM2GcodeParser: 

    __single = None 

    def __init__(self):

        if UM2GcodeParser.__single:
            raise RuntimeError('A UM2GcodeParser already exists')

        UM2GcodeParser.__single = self

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
                "G90": self.g90_abs_values,
                "G92": self.g92_set_pos,
                "M25": self.m25_stop_reading,
                "M82": self.m82_extruder_abs_values,
                "M84": self.m84_disable_motors,
                "M104": self.m104_extruder_temp,
                "M106": self.m106_fan_on,
                "M107": self.m107_fan_off,
                "M109": self.m109_extruder_temp_wait,
                "M117": self.m117_message,
                "M140": self.m140_bed_temp,
                "M190": self.m190_bed_temp_wait,
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

        # For G10/G11 handling: xxx session handling
        # self.retracted = False

        self.ultiGcodeFlavor = False

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

        print "Unlinking temp. copy of gcode input: ", tmpfname
        os.unlink(tmpfname)

        self.numParts = 0

        print "pre-parsing ", fn, tmpfname
        for line in f:
            line = line.strip()
            if line.startswith(";"):
                upperLine = line.upper()
                # Cura: "LAYER:"
                if "LAYER:" in upperLine:
                    layerNum = getCuraLayer(line)
                    if layerNum == 1:
                        self.numParts += 1
                        self.planner.newPart(self.numParts)

                # # Simplify3D: "; LAYER "
                # elif "; LAYER " in upperLine:
                    # layerNum = getSimplifyLayer(line)
                    # if layerNum == 1:
                        # self.numParts += 1
                        # self.planner.newPart(self.numParts)

                # Simplify3D: "; skirt "
                elif upperLine.startswith("; SKIRT"):
                    self.numParts += 1
                    self.planner.newPart(self.numParts)

                # ;FLAVOR:UltiGCode
                elif "FLAVOR:ULTIGCODE" in upperLine:
                    self.ultiGcodeFlavor = True
                    # To compute extrude length from volume (see getValues()):
                    # V = A * h, h = V / A, A = pi/4 * diameter²
                    aFilament = MatProfile.getMatArea()
                    self.e_to_filament_length = self.e_to_filament_length / aFilament

        print "pre-parsing # parts:", self.numParts
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

                if "LAYER:" in upperLine:
                    layerNum = getCuraLayer(line)
                    self.planner.layerChange(layerNum)
                    return

                elif "; LAYER " in upperLine:
                    layerNum = getSimplifyLayer(line)
                    self.planner.layerChange(layerNum)
                    return


                # print "skipping comment: ", line
                return

            # print "line:", tokens
            meth = self.commands[cmd]

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

            # try:
                # values[valueChar] = int(rest) * factor
            # except ValueError:
            values[valueChar] = float(rest) * factor

        return values

    def m25_stop_reading(self, line, values):
        print "XXX todo implement M25", values

    def m82_extruder_abs_values(self, line, values):
        # We're always using absolute coords...
        pass

    def m84_disable_motors(self, line, values):
        print "ignoring m84..."

    def m104_extruder_temp(self, line, values):
        print "ignoring m104 (set extruder temp)..."

    def m106_fan_on(self, line, values):
        # print "m106_fan_on", values
        fanSpeed = (values["S"] * MatProfile.getFanPercent()) / 100

        # "Blip fan" for Cura (S3D supports blip fan)
        if fanSpeed < 50 and self.ultiGcodeFlavor:
            # Start fan with full power
            self.planner.addSynchronizedCommand(CmdSyncFanSpeed, p1=packedvalue.uint8_t(255))
            # Dwell 0.25s
            self.planner.addSynchronizedCommand(CmdDwellMS, p1=packedvalue.uint16_t(250))

        self.planner.addSynchronizedCommand(CmdSyncFanSpeed, p1=packedvalue.uint8_t(fanSpeed))

    def m107_fan_off(self, line, values):
        # print "m107_fan_off", values
        self.planner.addSynchronizedCommand(CmdSyncFanSpeed, p1=packedvalue.uint8_t(0))

    def m109_extruder_temp_wait(self, line, values):
        print "ignoring m109..."

    def m117_message(self, line):
        print "m117_message: ", line

    def m140_bed_temp(self, line, values):
        print "ignoring m140..."

    def m190_bed_temp_wait(self, line, values):
        print "ignoring m190..."

    def m907_motor_current(self, line, values):
        print "ignoring m907..."

    def t0(self, line, values):
        print "ignoring t0..."

    def g4_dwell(self, line, values):
        if "P" in values:
            print "dwell, ", values["P"], "ms"
            self.planner.addSynchronizedCommand(CmdDwellMS, p1=packedvalue.uint16_t(values["P"]))
        elif "S" in values:
            self.planner.addSynchronizedCommand(CmdDwellMS, p1=packedvalue.uint16_t(values["S"] * 1000))
        else:
            # unknown parameter
            assert(0)

    def g10_retract(self, line, values):
        # print "g10_retract", values

        # if self.retracted:
            # return

        current_position = self.getPos()

        """
        values = {
                "F": PrinterProfile.getRetractFeedrate(),
                "A": current_position[A_AXIS] - PrinterProfile.getRetractLength(),
                }
        self.g0("G10", values)
        """

        rl = - PrinterProfile.getRetractLength()

        current_position[A_AXIS] += rl

        self.planner.addMove(TravelMove(
            line,
            displacement_vector=Vector([0.0, 0.0, 0.0, rl, 0.0]),
            displacement_vector_steps=[0.0, 0.0, 0.0, rl * self.steps_per_mm[A_AXIS], 0.0],
            feedrate=min(PrinterProfile.getRetractFeedrate(), PrinterProfile.getMaxFeedrate(3))
            ))

        # self.retracted = True
        self.setPos(current_position)
            
    def g11_retract_recover(self, line, values):
        # print "g11_retract_recover", values

        # if not self.retracted:
            # return

        current_position = self.getPos()

        """
        values = {
                "F": PrinterProfile.getRetractFeedrate(),
                "A": current_position[A_AXIS] + PrinterProfile.getRetractLength(),
                }
        self.g0("G11", values)
        self.retracted = False
        """

        rl = PrinterProfile.getRetractLength()

        current_position[A_AXIS] += rl

        self.planner.addMove(TravelMove(
            line,
            displacement_vector=Vector([0.0, 0.0, 0.0, rl, 0.0]),
            displacement_vector_steps=[0.0, 0.0, 0.0, rl * self.steps_per_mm[A_AXIS], 0.0],
            feedrate=min(PrinterProfile.getRetractFeedrate(), PrinterProfile.getMaxFeedrate(3))
            ))

        # self.retracted = False
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

    def g90_abs_values(self, line, values):
        # We're always using absolute coords...
        pass

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

            rDiff = values[dimC] - curRealPos[dim]
            displacement_vector[dim] = rDiff

            nSteps = rDiff * self.steps_per_mm[dim]
            displacement_vector_steps[dim] = nSteps

            newRealPos[dim] = values[dimC]

        # Check if zero length:
        if displacement_vector_steps == [0.0, 0.0, 0.0, 0.0, 0.0]:
            # Skip this empty move.
            return

        # Constrain feedrate to max values
        feedrateVector = displacement_vector._setLength(feedrate).constrain(self.maxFeedrateVector)
        if feedrateVector:
            feedrate = feedrateVector.length()

        if isPrintMove(displacement_vector):
            self.planner.addMove(PrintMove(
                line,
                displacement_vector=displacement_vector,
                displacement_vector_steps=displacement_vector_steps,
                feedrate=feedrate, # mm/s
                maxAccelV = self.planner.advance.maxAxisAcceleration(),
                ))
        else:
            self.planner.addMove(TravelMove(
                line,
                displacement_vector=displacement_vector,
                displacement_vector_steps=displacement_vector_steps,
                feedrate=feedrate, # mm/s
                ))
            
        # print "newRealPos: ", newRealPos
        self.setPos(newRealPos)

    def unknown(self, line, values):
        self.planner.addSynchronizedCommand(CmdUnknown, p1=packedvalue.uint8_t(values["X"]))









