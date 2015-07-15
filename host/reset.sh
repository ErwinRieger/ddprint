
dev="ttyACM0"
if [ -n "$1" ]; then
    dev="$1"
fi

stty -F /dev/$dev hupcl
stty -F /dev/$dev -hupcl

