
if [ "$1" == "/tmp/cobs.out" ]; then
    exit 0
fi

set -x

echo "$1"
python cobs.py $1 || exit 1
cmp $1 /tmp/cobs.out

