

gp="$1"


baseTemp="$(grep BaseTemp $gp|cut -d"=" -f2)"

# echo "baseTemp: $baseTemp"

gnuplot -e "set terminal unknown" -p 

f=$(gnuplot -e "set terminal unknown" -p $gp 2>&1|awk -e "/^a.*=/ {printf 1.0/\$3 \"\n\";}")
b=$(gnuplot -e "set terminal unknown" -p $gp 2>&1|awk -e "/^b.*=/ {printf \$3 \"\n\";}")

# echo "b: $b"
# echo "f: $f"

flow200="$(echo "$b + (200 - ${baseTemp})/${f}" | bc)"

# echo "flow200: $flow200"

echo "\"baseExtrusionRate_xx\" : $flow200,"
echo "\"extrusionAutoTempFactor\" : $f"


