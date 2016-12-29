
export PYTHONPATH="/opt/cura/lib/python3/dist-packages"
export LD_LIBRARY_PATH=/opt/cura/lib

dn=$(dirname $0)

echo -ne "slice\0-v\0-j\0/opt/cura/share/cura/resources/definitions/fdmprinter.def.json\0"
$dn/ce-settings.py /opt/cura "Ultimaker 2 0.80" "advance_08" 2>/dev/null
echo -ne "-s\0center_obj=true\0-o\0$2\0-l\0$1\0"

