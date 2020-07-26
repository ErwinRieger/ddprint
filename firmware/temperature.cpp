/*
  temperature.c - temperature control
  Part of Marlin

 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)

 It has preliminary support for Matthew Roberts advance algorithm
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html

 */

#include <Arduino.h>

#include "temperature.h"
#include "ddtemp.h"
#include "pins.h"

#include "hal.h"

#include "serialport.h"

// Redundant definitions to avoid include of ddprint.h
// extern bool IsStopped();
// extern void Stop(uint8_t reasonNr);
// extern void kill(const char* msg);

//===========================================================================
//=============================public variables============================
//===========================================================================
// XXX todo: make member of TempControl
uint16_t target_temperature[EXTRUDERS] = { 0 };
// XXX todo: make member of TempControl
uint8_t target_temperature_bed = 0;

float current_temperature_bed = HEATER_0_MINTEMP;
float current_temperature[EXTRUDERS] = ARRAY_BY_EXTRUDERS(HEATER_1_MINTEMP, HEATER_2_MINTEMP, HEATER_3_MINTEMP);

void tp_init()
{

#if defined(HEATER_0_PIN)
    HEATER_0_PIN :: init();
#endif

#if defined(HEATER_1_PIN)
    HEATER_1_PIN :: init();
#endif

// #if defined(HEATER_2_PIN)
    // SET_OUTPUT_PWM(HEATER_2_PIN, false);
// #endif

// Bitbang
// #if defined(HEATER_BED_PIN)
//    SET_OUTPUT_PWM(HEATER_BED_PIN, HB_PIN_ACTIVE_LOW);
//#endif

    // SET_OUTPUT(HEATER_BED_PIN);
    // WRITE(HEATER_BED_PIN, ~HEATER_BED_ACTIVE);
    HEATER_BED_PIN :: initDeActive();

    tempControl.init();
}

void disable_heater()
{

    for(int i=0;i<EXTRUDERS;i++)
        setTargetHotend(0,i);
    setTargetBed(0);

    HEATER_BED_PIN :: saveState();
    HEATER_0_PIN :: saveState();
    HEATER_1_PIN :: saveState();
}

