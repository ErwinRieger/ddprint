#!/bin/bash

#
# Hard-reset arduino style printers (toggle DTR)
#

dev="$DDPRINTDEV"
if [ -n "$1" ]; then
    dev="$1"
fi

# keep_serial_open.sh prevents reset, so stop it before:
keep_serial_open.sh "$dev" stop

stty -F $dev hupcl
stty -F $dev -hupcl

