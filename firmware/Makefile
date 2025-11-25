#/*
# This file is part of ddprint - a 3D printer firmware.
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

FIRMWARES = um2 ender3 ai3m ramps rumba jp

# .PHONY: $(FIRMWARES)
.PHONY: version.cpp

all: $(FIRMWARES)

$(FIRMWARES): version.cpp
	make -f build_files/Makefile.$@

clean: 
	$(foreach fw, $(FIRMWARES), make -f build_files/Makefile.$(fw) clean;)

version.cpp:
	echo "const char *buildversion = \"$(shell cd ..; ./scripts/getversion.sh)\";" > version.cpp
	echo "const char *builddate = \"$(shell date -I)\";" >> version.cpp


