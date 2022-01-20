#!/usr/bin/env bash

if [ "$#" != 2 ]; then
    echo "Usage: dd-fw-upload.sh [atmega2560|atmega1284p|swd] <fw-file>"
    exit 1
fi

envscript="$(realpath -m ${0}/../ddprint.env)"
. $envscript

hwtype="$1"
fwfile="$2"

case $hwtype in
    atmega2560)
        avrdude -q -V -p atmega2560 -D -c wiring -b 115200 -P $DDPRINTDEV -U flash:w:$fwfile:i
        ;;
    atmega1284p)
        avrdude -q -V -p atmega1284p -D -c arduino -b 115200 -P $DDPRINTDEV -U flash:w:$fwfile:i
        ;;
    swd)
        echo "swd upload not implemented yet"
        ;;
    *)
        echo "unknown hardware type: $hwtype"
        ;;
esac
        

