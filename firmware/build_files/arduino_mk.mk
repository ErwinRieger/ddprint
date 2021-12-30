#/*
# This file is part of ddprint - a 3D printer firmware.
# 
# Copyright 2021 erwin.rieger@ibrieger.de
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


# ARDUINO_DIR  = /usr/share/arduino
ARDUINO_DIR  = ../../arduino-1.6.13

TARGET = firmware.$(FW_NAME)
OBJDIR := build.$(FW_NAME)

ARDUINO_LIBS += SdFat/src/SdCard 
USER_LIB_PATH = libraries

NO_CORE_MAIN_CPP = 1

AVRDUDE_ARD_BAUDRATE = 115200

include ../../Arduino-Makefile/Arduino.mk

OPTIMIZATION_LEVEL = 2
# Debug version
# OPTIMIZATION_LEVEL = s
# DEBUGFLAGS    =  -g3 -ggdb
# LDFLAGS += -g

CPPFLAGS      +=  -I. -Iavr -Isrc
CPPFLAGS      +=  -I./libraries/SdFat/src
CPPFLAGS      +=  -I$(ARDUINO_PLATFORM_LIB_PATH)
CPPFLAGS      +=  $(DEBUGFLAGS)

# Temp. fix depreciated warning in arduino:Print.cpp
CPPFLAGS      +=  -D"PGM_P=const char*"

# Disable colored gcc output (don't confuse vim)
CXXFLAGS      +=  -fno-diagnostics-color

CXXFLAGS      +=  -ffat-lto-objects -save-temps=obj -fverbose-asm

# Dump assembler with line information for debugging
# CXXFLAGS      +=  -g -Wa,-adhln

LDFLAGS 	  += -mrelax

# Create map file
OTHER_LIBS      += -Wl,-Map,build.$(FW_NAME)/build.map

floatcheck:
	@echo "\nGrepping for soft math functions, watch out for divsf and mulsf:\n"
	grep -E "call.*divs|call.*muls" build.$(FW_NAME)/*.s|sort -u

