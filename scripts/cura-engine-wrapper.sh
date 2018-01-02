#!/usr/bin/env bash
#
# Usage:
#
#   cura-engine-settings.sh <cura-dir> <machine-profile> <mat-profile> <input-stl> <output-gcode> [<additionalSettings>]
#
# See also: http://ibrieger.de/cura-engine-wrapperpy-helper-to-call-curaengine-from-command-line.html
#

curaDir="$1"
machProfile="$2"
matProfile="$3"
inputFile="$4"
outputFile="$5"
additionalSettings="$6"

dn="$(dirname $0)"

export PYTHONPATH="${curaDir}/lib/python3/dist-packages"
export LD_LIBRARY_PATH="${curaDir}/lib"

$dn/cura-engine-wrapper.py "${curaDir}" "Ultimaker 2 0.80" "advance_08" "${inputFile}" "${outputFile}" "${additionalSettings}"

