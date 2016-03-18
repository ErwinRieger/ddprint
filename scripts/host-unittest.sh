
#
# Simple test of the cli.
#

dev="/dev/ttyACM0"
if [ -n "$1" ]; then
    dev="$1"
    shift
fi

echo "Don't run this on your good printer! - press RETURN to continue."
read in

set -x

python ddprint.py -d $dev  factoryReset || exit 1
python ddprint.py -d $dev  writeEepromFloat add_homeing_z -25 || exit 1
python ddprint.py -d $dev  dumpeeprom || exit 1
python ddprint.py -d $dev  disableSteppers  || exit 1
python ddprint.py -d $dev $* home  || exit 1
python ddprint.py -d $dev $* home  || exit 1
python ddprint.py -d $dev  getTemps || exit 1
python ddprint.py -d $dev  bedLevelAdjust 0.1 || exit 1
python ddprint.py -d $dev  getEndstops || exit 1
python ddprint.py -d $dev  getpos || exit 1
python ddprint.py -d $dev  getStatus || exit 1

python ddprint.py -d $dev  moverel X 50.1 || exit 1
python ddprint.py -d $dev  moverel X -50 || exit 1
python ddprint.py -d $dev  moveabs X 0 || exit 1
python ddprint.py -d $dev  fanspeed 100 || exit 1

for axis in Y Z A; do
    python ddprint.py -d $dev  moverel $axis -10.1 || exit 1
    python ddprint.py -d $dev  moverel $axis 10 || exit 1
    python ddprint.py -d $dev  moveabs $axis 0 || exit 1
done

./ddprint.py -d /tmp/ttyUSB10 -t1 16 retract || exit 1
echo "" | ./ddprint.py -d /tmp/ttyUSB10 -t1 16 heatHotend || exit 1
yes | ./ddprint.py -d /tmp/ttyUSB10 -t1 16 changenozzle || exit 1
yes | ./ddprint.py -d /tmp/ttyUSB10 -t1 16 insertFilament || exit 1
yes | ./ddprint.py -d /tmp/ttyUSB10 -t1 16 bedLeveling || exit 1
yes | ./ddprint.py -d /tmp/ttyUSB10 -t1 16 removeFilament || exit 1
./ddprint.py -d /tmp/ttyUSB10 pre test_files/buchse.gcode || exit 1
./ddprint.py -d /tmp/ttyUSB10 -t0 16 print test_files/buchse.gcode || exit 1
python ddprint.py -d $dev  stop || exit 1



#    mon                 Monitor serial printer interface.
#    getTempTable        Output temperature-speed table.
#    getFilSensor        Get current filament position.
#    zRepeatability      Debug: Move Z to 10 random positions to test
#    stepResponse        Measure and plot stepResponse of hotend PID.
#    testFilSensor       Debug: move filament manually, output filament sensor
#    calibrateFilSensor  Debug: helper to determine the ratio of stepper to


