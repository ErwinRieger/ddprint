#/*
# This file is part of ddprint - a direct drive 3D printer firmware.
# 
# Copyright 2015 erwin.rieger@ibrieger.de
# 
# ddprint is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# ddprint is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with ddprint.  If not, see <http://www.gnu.org/licenses/>.
#*/

all: fw

fw:
	make -f Makefile.fw 

fwupload:
	make -f Makefile.fw do_upload

fwall: fw fwupload
	echo "make fw and upload done"

ramps:
	make -f Makefile.ramps 

jp:
	make -f Makefile.jp 

world: fw ramps jp
	make -f Makefile.sim 

clean: 
	make -f Makefile.ramps clean
	make -f Makefile.fw clean
	make -f Makefile.sim clean

reset: 
	echo "yeah reset"
