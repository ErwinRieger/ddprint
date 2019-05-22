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
#include "fastio.h"
#include "serialport.h"

// Redundant definitions to avoid include of ddprint.h
extern bool IsStopped();
extern void Stop(uint8_t reasonNr);
extern void kill(const char* msg);

//===========================================================================
//=============================public variables============================
//===========================================================================
// XXX todo: make member of TempControl
uint16_t target_temperature[EXTRUDERS] = { 0 };
// XXX todo: make member of TempControl
uint8_t target_temperature_bed = 0;

float current_temperature_bed = BED_MINTEMP;
float current_temperature[EXTRUDERS] = ARRAY_BY_EXTRUDERS(HEATER_1_MINTEMP, HEATER_2_MINTEMP, HEATER_3_MINTEMP);

void tp_init()
{

  #if defined(HEATER_0_PIN) && (HEATER_0_PIN > -1)
    SET_OUTPUT(HEATER_0_PIN);
  #endif
  #if defined(HEATER_1_PIN) && (HEATER_1_PIN > -1)
    SET_OUTPUT(HEATER_1_PIN);
  #endif
  #if defined(HEATER_2_PIN) && (HEATER_2_PIN > -1)
    SET_OUTPUT(HEATER_2_PIN);
  #endif
  #if defined(HEATER_BED_PIN) && (HEATER_BED_PIN > -1)
    SET_OUTPUT(HEATER_BED_PIN);
  #endif

  #if defined(FAN_PIN) && (FAN_PIN > -1)
    SET_OUTPUT(FAN_PIN);
    #ifdef FAST_PWM_FAN
    setPwmFrequency(FAN_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
    #ifdef FAN_SOFT_PWM
    soft_pwm_fan = fanSpeedSoftPwm / 2;
    #endif
  #endif

#if (EXTRUDERS > 1) && defined(HEATER_1_MAXTEMP)
  // maxttemp[1] = HEATER_1_MAXTEMP;
  while(analog2temp(maxttemp_raw[1], 1) > HEATER_1_MAXTEMP) {
#if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
    maxttemp_raw[1] -= OVERSAMPLENR;
#else
    maxttemp_raw[1] += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP 1

    tempControl.init();
}

void disable_heater()
{
    for(int i=0;i<EXTRUDERS;i++)
        setTargetHotend(0,i);
    WRITE(HEATER_0_PIN,LOW);

    setTargetBed(0);
    WRITE(HEATER_BED_PIN,LOW);
}

#if 0
void max_temp_error(uint8_t e) {
  disable_heater();
  if(IsStopped() == false) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN((int)e);
    SERIAL_ERRORLNPGM(": Extruder switched off. MAXTEMP triggered !");
    // LCD_ALERTMESSAGEPGM("Err: MAXTEMP");
  }
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop(STOP_REASON_MAXTEMP);
  #endif
}

void min_temp_error(uint8_t e) {
  disable_heater();
  if(IsStopped() == false) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN((int)e);
    SERIAL_ERRORLNPGM(": Extruder switched off. MINTEMP triggered !");
    // LCD_ALERTMESSAGEPGM("Err: MINTEMP");
  }
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop(STOP_REASON_MINTEMP);
  #endif
}

void bed_max_temp_error(void) {
  disable_heater();
  if(IsStopped() == false) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Temperature heated bed switched off. MAXTEMP triggered !!");
    // LCD_ALERTMESSAGEPGM("Err: MAXTEMP BED");
  }
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop(STOP_REASON_MAXTEMP_BED);
  #endif
}

void bed_min_temp_error(void) {
  disable_heater();
  if(IsStopped() == false) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Temperature heated bed switched off. MINTEMP triggered !!");
    // LCD_ALERTMESSAGEPGM("Err: MINTEMP BED");
  }
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop(STOP_REASON_MINTEMP_BED);
  #endif
}
#endif
