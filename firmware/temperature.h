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

extern int target_temperature_bed;
extern float current_temperature_bed;

extern int target_temperature[EXTRUDERS];
extern float current_temperature[EXTRUDERS];

void disable_heater();
void tp_init();  //initialise the heating

inline void setTargetHotend(const float &celsius, uint8_t extruder) {
  target_temperature[extruder] = celsius;
};

inline void setTargetBed(const float &celsius) {
  target_temperature_bed = celsius;
};

float analog2temp(int raw, uint8_t e);
float analog2tempBed(int raw);

void min_temp_error(uint8_t e);
void max_temp_error(uint8_t e);

void bed_min_temp_error(void);
void bed_max_temp_error(void);

#endif

