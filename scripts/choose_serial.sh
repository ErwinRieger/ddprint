#!/bin/bash

#
# Choose serial device used to set DDPRINTDEV env. var.
#

menuItems() {
    for dev in $*; do echo "$dev $(basename $dev)"; done
}

devices="$(find /dev/serial/by-id/ -type l 2>/dev/null)"
ndev="$(echo "$devices" | wc -l)"

if [ "$ndev" == "1" ]; then
    # choose this single device
    echo "$devices"
elif [ "$ndev" -gt "1" ]; then
    let h="$((7 + ndev))"
    choosen="$(dialog --stdout --no-tags --menu "Choose printer serial device" "$menuh" 75 "$ndev" $(menuItems $devices))"
    echo "$choosen"
fi


