#! /bin/bash

for inp in $* test_files/*.gcode; do 

    echo "###################################"
    echo "$0: running python ddprint.py pre $inp"
    echo "###################################"
    python ddprint.py pre $inp

    if [ "$?" != "0" ]; then
        echo "###################################"
        echo "$0: error running $inp"
        echo "###################################"
        exit 1
    fi
    echo "###################################"
    echo "$0: $inp done"
    echo "###################################"
done

