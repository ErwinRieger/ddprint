#!/usr/bin/env bash

ask() {

	echo -e "\nPress <enter> to run '$*', use <control-c> to exit."
	read in
	$*
}

install_python_deps() {

	grep -q "ModuleNotFoundError.*numpy" /tmp/ddprint.out && ask sudo apt-get install python3-numpy
	grep -q "ModuleNotFoundError.*serial" /tmp/ddprint.out && ask sudo apt-get install python3-serial
}

type git >/dev/null || ask sudo apt-get install git

type dialog >/dev/null || ask sudo apt-get install dialog

ask git clone https://github.com/ErwinRieger/ddprint.git

ask git clone https://github.com/ErwinRieger/ddprint-profiles.git

DDPRINTDEV="none" source ddprint/scripts/ddprint.env

DDPRINTDEV="none" ddprint -h 2>/tmp/ddprint.out > /dev/null || install_python_deps

echo -e "\nNote: For convenience, you shold add $DDPRINTHOME to your PATH variable."

echo -e "\nInstallation done, exiting."

