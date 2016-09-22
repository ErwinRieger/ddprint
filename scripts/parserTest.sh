#! /bin/bash

nTests=0
for inp in $* test_files/*.gcode; do 

    echo "###################################"
    echo "$0: running python ddprint.py pre $inp"
    echo "###################################"
    python ddprint.py pre $inp

    if [ "$?" != "0" ]; then
        echo "###################################"
        echo "$0: error running Test $nTests $inp"
        echo "###################################"
        exit 1
    fi

    let "nTests = nTests + 1"

    echo "###################################"
    echo "$0: Test $nTests $inp done"
    echo "###################################"
done

