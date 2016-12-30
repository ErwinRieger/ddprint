#!/usr/bin/env bash
#
# Usage:
#
#   cura-engine-settings.sh <cura-dir> <machine-profile> <mat-profile> <input-stl> <output-gcode> | xargs -0 /opt/cura/bin/CuraEngine
#

# XXX adjust to your cura installation dir:
CURADIR="$1"
machProfile="$2"
matProfile="$3"
inputFile="$4"
outputFile="$5"

export PYTHONPATH="${CURADIR}/lib/python3/dist-packages"
export LD_LIBRARY_PATH="${CURADIR}/lib"

dn=$(dirname $0)

# echo -ne "slice\0-v\0-j\0${CURADIR}/share/cura/resources/definitions/fdmprinter.def.json\0"
echo -ne "slice\0-j\0${CURADIR}/share/cura/resources/definitions/fdmprinter.def.json\0"
$dn/cura-engine-settings.py ${CURADIR} "Ultimaker 2 0.80" "advance_08" 2>/dev/null
echo -ne "-s\0center_object=true\0-o\0${outputFile}\0-l\0${inputFile}\0"

