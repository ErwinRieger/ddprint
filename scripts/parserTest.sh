#! /bin/bash

nTests=0
for inp in $(ls -rS $* test_files/*.gcode); do 

    echo "###################################"
    echo "$0: running python ddprint.py pre $inp" >&2
    echo "###################################"

    for kadv in 0 0.1 0.4 0.5 1; do

        echo "python ddprint.py -kadvance $kadv pre UM2_profile_template nozzle40 pla_1.75mm $inp"
        python ddprint.py -kadvance $kadv pre UM2_profile_template nozzle40 pla_1.75mm $inp

        if [ "$?" != "0" ]; then
            echo "########################################################"
            echo "$0: error running Test $nTests $inp with kadvance $kadv"
            echo "########################################################"
            echo "$kadv $inp" > /tmp/parserTest_error_file
            exit 1
        fi

        let "nTests = nTests + 1"

    done

    echo "###################################"
    echo "$0: Test $nTests $inp done"
    echo "###################################"
done

