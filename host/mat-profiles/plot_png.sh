
f="$1"

bn="$(basename $f .gnuplot | tr '_' '-')"

gnuplot -e "set title '$bn'; set term png;" -p "$f" > "$f.png"

