#!/bin/bash

#
# Work around for problems with the cp210x linux driver.
# Seems that the DTR line is toggled on every 'open' call on the usb-serial
# device leading to a reset of arduino style printers (even if using '-hupcl') :-( 
#
#
# This script tries to keeps the device opened to avoid this.
#

dev="$1"
cmd="$2"

if [ ! -e "$dev" ]; then
    echo "device '$dev' not found, exiting."
    exit 1
fi

pidfile="/tmp/ddprint_keepopen_$(basename $(realpath $dev))"

isRunning() {

    if [ -e "$pidfile" ]; then
        echo "pidfile exists: $pidfile"
        pid="$(cat $pidfile)"
        procfile="/proc/$pid/cmdline"
        if [ -e "$procfile" ] && grep -q "keep_serial_open" $procfile; then
            echo "keep_serial_open running on $dev"
            return 0
        else
            echo "removing stale pid file $pidfile"
            rm $pidfile
        fi
    fi

    return 1
}

if [ "$cmd" == "running" ]; then
    isRunning && exit 0
    exit 1
elif [ "$cmd" == "stop" ]; then
    if isRunning; then
        pid="$(cat $pidfile)"
        kill $pid
        rm "$pidfile"
        sleep 1
    fi
    exit 0
fi

if isRunning; then
    exit 0
fi

exec 3>> $dev

pid="$$"
echo "keepOpen thread pid: $pid."
echo "$pid" > $pidfile

echo "start watching $dev..."
while [ -e "$dev" ] && [ -e "$pidfile" ]; do
    sleep 1
done

rm "$pidfile"

# echo "device $dev disappeared, stopping keepOpen thread (pid: $pid)."
# kill "$pid"
echo "device $dev disappeared, stopping."

