#!env bash

nTests=0
id=0
count="$(ls $*|wc -l)"
maxProc=8

function nError() {
    echo $(find /tmp -name "parserTest_error_*" 2>/dev/null|wc -l)
}

function nProc() {
    echo $(find /tmp -name "parserTest_*.run" 2>/dev/null|wc -l)
}

function runTest() {

    local kadv=$1
    local inp=$2
    local id=$3

    echo "nice python -u ddprint.py -kadvance $kadv pre UM2_profile_template nozzle40 pla $inp"
    nice python ddprint.py -kadvance $kadv pre UM2_profile_template nozzle40 pla $inp

    if [ "$?" != "0" ]; then
        fn=/tmp/parserTest_error_${id}
        echo "########################################################" >> $fn
        echo "$0: error running Test $((nTests+1)) $inp with kadvance $kadv" >> $fn
        echo "########################################################" >> $fn
    fi

    echo "###################################"
    echo "$0: Test $inp (#$((nTests+1)) of $count) done"
    echo "###################################"

    fn=/tmp/parserTest_$(basename $inp)_${id}.run
    rm ${fn}
}

rm  /tmp/parserTest_error_*
rm  /tmp/parserTest_*.run

for inp in $*; do 

    for kadv in 0.4 0.5 1 0 0.1; do

        if [ $(nError) -gt 0 ]; then
            break
        fi

        while [ $(nProc) -eq $maxProc ]; do
            sleep 1
        done

        echo "###################################"
        echo "$0: running test $((nTests+1)): python ddprint.py pre $inp" >&2
        echo "###################################"

        fn=/tmp/parserTest_$(basename $inp)_${id}.run
        echo "$inp" > ${fn}
        runTest $kadv $inp $id &

        id=$((id + 1))
    done

    nTests=$((nTests + 1))

    if [ $(nError) -gt 0 ]; then
        break
    fi
done

while [ $(nProc) -gt 0 ]; do

    echo "waiting for finish 2... $(nProc)"
    sleep 1
done

echo "$nTests of $count tests done, $(nError) errors"

