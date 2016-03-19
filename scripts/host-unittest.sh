
#
# Simple test of the cli.
#

t1=""
if echo $*|grep ttyUSB10; then
    t1="-t1 130"
fi

echo "Don't run this on your good printer! - press RETURN to continue."
read in

set -x

python ddprint.py $t1 $*  factoryReset || exit 1
python ddprint.py $t1 $*  writeEepromFloat add_homeing_z -25 || exit 1
python ddprint.py $t1 $*  dumpeeprom || exit 1
python ddprint.py $t1 $*  disableSteppers  || exit 1
python ddprint.py $t1 $* home  || exit 1
python ddprint.py $t1 $* home  || exit 1
python ddprint.py $t1 $*  getTemps || exit 1
python ddprint.py $t1 $*  bedLevelAdjust 0.1 || exit 1
python ddprint.py $t1 $*  getEndstops || exit 1
python ddprint.py $t1 $*  getpos || exit 1
python ddprint.py $t1 $*  getStatus || exit 1

python ddprint.py $t1 $*  moverel X 50.1 || exit 1
python ddprint.py $t1 $*  moverel X -50 || exit 1
python ddprint.py $t1 $*  moveabs X 0 || exit 1
python ddprint.py $t1 $*  fanspeed 100 || exit 1

for axis in Y Z A; do
    python ddprint.py $t1 $*  moverel $axis -10.1 || exit 1
    python ddprint.py $t1 $*  moverel $axis 10 || exit 1
    python ddprint.py $t1 $*  moveabs $axis 0 || exit 1
done

./ddprint.py $* -t1 16 retract || exit 1
echo "" | ./ddprint.py $* -t1 16 heatHotend || exit 1
yes | ./ddprint.py $* -t1 16 changenozzle || exit 1
yes | ./ddprint.py $* -t1 16 insertFilament || exit 1
yes | ./ddprint.py $* -t1 16 bedLeveling || exit 1
yes | ./ddprint.py $* -t1 16 removeFilament || exit 1
./ddprint.py $t1 $* pre test_files/buchse.gcode || exit 1
./ddprint.py $t1 $* -t1 16 print test_files/buchse.gcode || exit 1
python ddprint.py $t1 $*  stop || exit 1



#    mon                 Monitor serial printer interface.
#    getTempTable        Output temperature-speed table.
#    getFilSensor        Get current filament position.
#    zRepeatability      Debug: Move Z to 10 random positions to test
#    stepResponse        Measure and plot stepResponse of hotend PID.
#    testFilSensor       Debug: move filament manually, output filament sensor
#    calibrateFilSensor  Debug: helper to determine the ratio of stepper to


