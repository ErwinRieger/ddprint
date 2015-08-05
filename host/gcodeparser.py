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

from ddprofile import MatProfile, PrinterProfile
from ddplanner import Planner
from ddprintcommands import CmdSyncFanSpeed
from ddprintconstants import dimNames
from move import VVector, Move
from ddprintutil import A_AXIS, B_AXIS, debugMoves, vectorDistance, circaf

import ddprintutil as util

############################################################################
# Constants, xxx todo: query from printer and/or profile
RetractFeedrate = 25      # mm/s
RetractLength = 4.5       # mm
# xxx erhöht wegen bowdenzug-spiel am feeder/kopf
# RetractLength = 6.5       # mm

# MATERIAL_DIAMETER = 2.9 # [mm]
############################################################################

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
                "G10": self.g10_retract,
                "G11": self.g11_retract_recover,
                "G21": self.g21_metric_values,
                "G28": self.g28_home,
                "G29": self.g29_autolevel,
                "G90": self.g90_abs_values,
                "G92": self.g92_set_pos,
                "M25": self.m25_stop_reading,
                "M106": self.m106_fan_on,
                "M107": self.m107_fan_off,
                "M117": self.m117_message,
                }

        # To compute extrude length from volume (see getValues()):
        # V = A * h, h = V / A, A = pi/4 * diameter²
        self.volume_to_filament_length = 4 / (math.pi * pow(MatProfile.getValues()["material_diameter"], 2))
        # Apply material flow parameter from material profile
        self.volume_to_filament_length *= MatProfile.getFlow() / 100.0

        self.steps_per_mm = PrinterProfile.getStepsPerMMVector()
        self.mm_per_step = map(lambda dim: 1.0 / self.steps_per_mm[dim], range(5))
        self.maxFeedrateVector = PrinterProfile.getMaxFeedrateVector()

        self.planner = Planner.get()

    @classmethod
    def get(cls):
        return cls.__single

    def reset(self):
        self._gcodePos = util.MyPoint()
        self._realPos = util.MyPoint()
        self.feedrate = None
        self.numParts = 1

        # For G10/G11 handling: xxx session handling
        self.retracted = False

    # Set current virtual printer position
    def set_position(self, point):
        self._gcodePos = point.copy()
        self._realPos = point.copy()

    def getRealPos(self):
        return self._realPos.copy()

    def getGcodePos(self):
        return self._gcodePos.copy()

    def setRealPos(self, pos):
        self._realPos = pos.copy()

    def setGcodePos(self, pos):
        self._gcodePos = pos.copy()

    def preParse(self, fn):
        f = open(fn)

        self.numParts = 0

        print "pre-parsing ", fn
        for line in f:
            line = line.strip()
            if line.startswith(";"):
                if "LAYER:" in line.upper():
                    layerNum = int(line.split(":")[1])
                    if layerNum == 1:
                        self.numParts += 1
                        self.planner.newPart(self.numParts)

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

                if "LAYER:" in line.upper():
                    layerNum = int(line.split(":")[1])
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
                factor = self.volume_to_filament_length
                # print "replacing ", param, " --> A", float(param[1:]) * factor
                valueChar = "A"

            if valueChar == "F":
                # print "replace feedrate in mm/min with mm/sec..."
                factor = 1.0 / 60

            try:
                values[valueChar] = int(rest) * factor
            except ValueError:
                values[valueChar] = float(rest) * factor

        return values

    def finishMoves(self):
        self.planner.finishMoves()

    def m25_stop_reading(self, line, values):
        print "XXX todo implement M25", values

    def m106_fan_on(self, line, values):
        # print "m106_fan_on", values
        fanSpeed = (values["S"] * MatProfile.getFanPercent()) / 100
        self.planner.addSynchronizedCommand(CmdSyncFanSpeed, p1=packedvalue.uint8_t(fanSpeed))

    def m107_fan_off(self, line, values):
        # print "m107_fan_off", values
        self.planner.addSynchronizedCommand(CmdSyncFanSpeed, p1=packedvalue.uint8_t(0))

    def m117_message(self, line):
        print "m117_message: ", line

    def g10_retract(self, line, values):
        # print "g10_retract", values

        # Compute retract 
        """
        self.addMove(Move(
                    typ="G10",
                    stepped_point=None, # stepped_point,
                    displacement_vector=displacement_vector,
                    displacement_vector_steps=displacement_vector_steps,
                    e_distance=e_distance,
                    feedrate = RetractFeedrate,
                    longest_axis=longest_axis,
                    ))
        """

        if not self.retracted:
            current_position = self.getRealPos()
            values = {
                    "F": RetractFeedrate,
                    "A": current_position[A_AXIS] - RetractLength,
                    }
            self.g0("G10", values)
            self.retracted = True

    def g11_retract_recover(self, line, values):
        # print "g11_retract_recover", values

        if self.retracted:
            current_position = self.getRealPos()
            values = {
                    "F": RetractFeedrate,
                    "A": current_position[A_AXIS] + RetractLength,
                    }
            self.g0("G11", values)
            self.retracted = False

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

        pos = self.getRealPos()
        for key in values:
            pos[key] = values[key]

        self.set_position(pos)

    def g0(self, line, values):
        # print "g0", values

        feedrate = self.feedrate
        if 'F' in values:
            feedrate = values['F']

        if not ("X" in values or "Y" in values or "Z" in values or "A" in values or "B" in values):
            # Nothing to move, F-Only gcode or invalid
            assert("F" in values)
            return

        curGcodePos = self.getGcodePos()
        curRealPos = self.getRealPos()

        newGcodePos = curGcodePos.copy()
        newRealPos = curRealPos.copy()

        # print "curGcodePos: ", newGcodePos
        # print "curRealPos: ", newRealPos

        displacement_vector = VVector()
        displacement_vector_steps = [0, 0, 0, 0, 0]

        eOnlyByGcode = True

        longest_axis = 0
        longestAxisStep = 0
        for dim in range(5):

            dimC = dimNames[dim]
            if dimC not in values:
                continue

            gDiff = values[dimC] - curGcodePos[dim]
            if gDiff and dimC in ['X', 'Y', 'Z']:
                eOnlyByGcode = False

            rDiff = values[dimC] - curRealPos[dim]

            nSteps = (int)((rDiff * self.steps_per_mm[dim]) + 0.5)

            displacement_vector_steps[dim] = nSteps

            # debug
            # delta = nSteps * self.mm_per_step[dim] - rDiff
            # print "values: ", dimC, rDiff, nSteps, delta
            # end debug

            rDiff = nSteps * self.mm_per_step[dim]
                    
            displacement_vector[dim] = rDiff

            newGcodePos[dimC] = values[dimC]
            newRealPos[dim] = curRealPos[dim] + rDiff

            if abs(nSteps) > longestAxisStep:
                longest_axis = dim
                longestAxisStep = abs(nSteps)

        #
        # Check if zero or small length:
        #
        if displacement_vector_steps == [0, 0, 0, 0, 0]:
            # Skip this very small move, the delta of this move is not lost,
            # since it is included in the next absolute gcode command.
            # Current position has not been updated, yet (see self.state.set_position(values))
            # at the end of this method.
            return

        # Get head move distance:
        move_distance = vectorDistance(curRealPos[:3], newRealPos[:3])

        #If that distance is 0, get move_distance for A/B axis
        if move_distance == 0:

            """
            if not eOnlyByGcode:
                print "XXX e-only move through rounding correction:", comment
                print "displacement_vector", displacement_vector
                print "displacement_vector_steps", displacement_vector_steps
                assert(0)
            """

            move_distance = max(
                abs(curRealPos[A_AXIS] - newRealPos[A_AXIS]),
                abs(curRealPos[B_AXIS] - newRealPos[B_AXIS])
            )

        else:

            assert(not eOnlyByGcode)

        if debugMoves:
            for dim in range(5):
                assert(circaf(newRealPos[dim] - curRealPos[dim], displacement_vector[dim], 0.000001))

        # feedrate = makerbot_driver.Gcode.get_safe_feedrate(
        #    displacement_vector,
        #    self.state.get_axes_values('max_feedrate'),
        #    feedrate,
        #)
        feedrateVector = displacement_vector._setLength(feedrate).constrain(self.maxFeedrateVector)
        if feedrateVector:
            feedrate = feedrateVector.len5()

        # print "pos:", stepped_point, "[steps]"
        # print "displacement_vector:", displacement_vector, "[mm]"
        # print "dda_speed:", dda_speed, "[uS/step]"
        # print "move_distance:", move_distance, "[mm]"
        # print "move_minutes:", move_minutes, "[min]"
        # print "feedrate:", feedrate, "[mm/s]"

        self.planner.addMove(Move(
            line,
            # stepped_point=stepped_point,
            displacement_vector=displacement_vector,
            displacement_vector_steps=displacement_vector_steps,
            move_distance=move_distance,
            feedrate=feedrate, # mm/s
            longest_axis=longest_axis
            ))
            
        if not eOnlyByGcode:
            self.feedrate = feedrate
        # else:
            # print "NOT storing new feedrate of E-Only move '%s'!" % comment

        # self.state.setGcodePosition(newGcodePos)
        # self.state.setRealPosition(newRealPos)

        # print "newGcodePos: ", newGcodePos
        # print "newRealPos: ", newRealPos
        self.setGcodePos(newGcodePos)
        self.setRealPos(newRealPos)













