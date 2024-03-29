/************************************************************************************************
* Note by erwin.rieger@ibrieger.de:
* This file is part of ddprint - a 3d printer firmware.
* The Origin of this code is Ultimaker2Marlin (https://github.com/Ultimaker/Ultimaker2Marlin).
************************************************************************************************/

/*
  temperature.h - temperature controller
  Part of Marlin

  Copyright (c) 2011 Erik van der Zalm

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef temperature_h
#define temperature_h

#include "Configuration.h"

#define STOP_REASON_MAXTEMP              1
#define STOP_REASON_MINTEMP              2
#define STOP_REASON_MAXTEMP_BED          3
// #define STOP_REASON_HEATER_ERROR         4
#define STOP_REASON_MINTEMP_BED          11

#if EXTRUDERS > 3
  # error Unsupported number of extruders
#elif EXTRUDERS > 2
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2, v3 }
#elif EXTRUDERS > 1
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2 }
#else
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1 }
#endif

extern int16_t target_temperature_bed;
extern int16_t current_temperature_bed;

extern int16_t target_temperature[EXTRUDERS];
extern int16_t current_temperature[EXTRUDERS];

void disable_heater();
void tp_init();  //initialise the heating

FWINLINE void setTargetHotend(int16_t celsius, uint8_t extruder) {
  target_temperature[extruder] = celsius;
};

FWINLINE void setTargetBed(int16_t celsius) {
  target_temperature_bed = celsius;
};

#endif

