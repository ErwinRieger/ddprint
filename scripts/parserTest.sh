#! /bin/bash

nTests=0
count="$(ls $*|wc -l)"

for inp in $*; do 

    echo "###################################"
    echo "$0: running python ddprint.py pre $inp" >&2
    echo "###################################"

    for kadv in 0.4 0.5 1 0 0.1; do

        echo "python -u ddprint.py -kadvance $kadv pre UM2_profile_template nozzle40 pla $inp"
        python -u ddprint.py -kadvance $kadv pre UM2_profile_template nozzle40 pla $inp

        if [ "$?" != "0" ]; then
            echo "########################################################"
            echo "$0: error running Test $nTests $inp with kadvance $kadv"
            echo "########################################################"
            echo "$kadv $inp" > /tmp/parserTest_error_file
            exit 1
        fi
    done

    let "nTests = nTests + 1"
    echo "###################################"
    echo "$0: Test $inp (#$nTests of $count) done"
    echo "###################################"
done

