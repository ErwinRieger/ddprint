#/*
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
#*/


############################################################################
#
# Enable Debugging
#
debugMoves = True
# debugMoves = False
debugAutoTemp = False
debugAutoTemp = True

#
# Enable mathplotlib plotting of planned paths
#
debugPlot = True
# Define which stage of path planning to plot
# plotLevelPlanned: plot path after advance-planning
# plotLevelSplitted: plot advance-splitted path
debugPlotLevel = "plotLevelPlanned"
# debugPlotLevel = "plotLevelSplitted"

############################################################################
#
# AutoTemp algorithm
#
UseExtrusionAutoTemp = True
UseExtrusionAutoTemp = False

#####################################################################
#
# Auto extrusion adjust, always enabled
#
############### xxx not used??? UseExtrusionAdjust = True

#####################################################################
#
# Limit extrusion by hotend temperature in the firmware
#
# xxx not used??? UseExtrusionLimit = True










