/*
* This file is part of ddprint - a direct drive 3D printer firmware.
* 
* Copyright 2015 erwin.rieger@ibrieger.de
* 
* ddprint is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* ddprint is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with ddprint.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "config.h"
#include "hal.h"

#if MOTHERBOARD == 1
    //
    // Ultimaker 2
    //
    #include "pins_um2.h"
#elif MOTHERBOARD == 2
    //
    // Ramps 1.4
    //
    #include "pins_ramps.h"
#elif MOTHERBOARD == 3
    //
    // Jennyprinter arm shuttle gear board
    //
    #include "pins_jp.h"
#elif MOTHERBOARD == 4
    //
    // Rumba mega2560
    //
    #include "pins_rumba.h"
#elif MOTHERBOARD == 5
    //
    // Ender 3, ender 5, atmega1284p
    //
    #include "pins_ender3.h"
#elif MOTHERBOARD == 6
    //
    // Anycubic I3 trigorilla
    //
    #include "pins_ai3m.h"
#else
    #error Unknown MOTHERBOARD value in pins.h
#endif


