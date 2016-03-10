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
#include "MarlinSerial.h"

// Redundant definitions to avoid include of ddprint.h
extern bool IsStopped();
extern void Stop(uint8_t reasonNr);
extern void kill(const char* msg);

//===========================================================================
//=============================public variables============================
//===========================================================================
int target_temperature[EXTRUDERS] = { 0 };
int target_temperature_bed = 0;

float current_temperature_bed = 0.0;
float current_temperature[EXTRUDERS] = { 0.0 };

static const void *heater_ttbl_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS(
        (void *)HEATER_0_TEMPTABLE, (void *)HEATER_1_TEMPTABLE, (void *)HEATER_2_TEMPTABLE );

static const uint8_t heater_ttbllen_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS(
        HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN, HEATER_2_TEMPTABLE_LEN );

void tp_init()
{

  #if 0
  // Finish init of mult extruder arrays
  for(int e = 0; e < EXTRUDERS; e++) {
    // populate with the first value
    maxttemp[e] = maxttemp[0];
  }
  #endif

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

#if 0
#ifdef HEATER_0_MINTEMP
  // minttemp[0] = HEATER_0_MINTEMP;
  while(analog2temp(minttemp_raw[0], 0) < HEATER_0_MINTEMP) {
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
    minttemp_raw[0] += OVERSAMPLENR;
#else
    minttemp_raw[0] -= OVERSAMPLENR;
#endif
  }
#endif //MINTEMP
#ifdef HEATER_0_MAXTEMP
  // maxttemp[0] = HEATER_0_MAXTEMP;
  while(analog2temp(maxttemp_raw[0], 0) > HEATER_0_MAXTEMP) {
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
    maxttemp_raw[0] -= OVERSAMPLENR;
#else
    maxttemp_raw[0] += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP

#if (EXTRUDERS > 1) && defined(HEATER_1_MINTEMP)
  // minttemp[1] = HEATER_1_MINTEMP;
  while(analog2temp(minttemp_raw[1], 1) < HEATER_1_MINTEMP) {
#if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
    minttemp_raw[1] += OVERSAMPLENR;
#else
    minttemp_raw[1] -= OVERSAMPLENR;
#endif
  }
#endif // MINTEMP 1
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

#ifdef BED_MINTEMP
  /* No bed MINTEMP error implemented?!? */ /*
  while(analog2tempBed(bed_minttemp_raw) < BED_MINTEMP) {
#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
    bed_minttemp_raw += OVERSAMPLENR;
#else
    bed_minttemp_raw -= OVERSAMPLENR;
#endif
  }
  */
#endif //BED_MINTEMP

#if 0

#ifdef BED_MAXTEMP
  while(analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
    bed_maxttemp_raw -= OVERSAMPLENR;
#else
    bed_maxttemp_raw += OVERSAMPLENR;
#endif
  }
#endif //BED_MAXTEMP
#endif

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

#define PGM_RD_W(x)   (short)pgm_read_word(&x)
// Derived from RepRap FiveD extruder::getTemperature()
// For hot end temperature measurement.
float analog2temp(int raw, uint8_t e) {
  if(e >= EXTRUDERS)
  {
      SERIAL_ERROR_START;
      SERIAL_ERROR((int)e);
      SERIAL_ERRORLNPGM(" - Invalid extruder number !");
      kill("Invalid extruder number!");
  }

  if(heater_ttbl_map[e] != NULL)
  {
    float celsius = 0;
    uint8_t i;
    short (*tt)[][2] = (short (*)[][2])(heater_ttbl_map[e]);

    for (i=1; i<heater_ttbllen_map[e]; i++)
    {
      if (PGM_RD_W((*tt)[i][0]) > raw)
      {
        celsius = PGM_RD_W((*tt)[i-1][1]) +
          (raw - PGM_RD_W((*tt)[i-1][0])) *
          (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i-1][1])) /
          (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i-1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map[e]) celsius = PGM_RD_W((*tt)[i-1][1]);

    return celsius;
  }
  return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
}

// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
float analog2tempBed(int raw) {
  #ifdef BED_USES_THERMISTOR
    float celsius = 0;
    uint8_t i;

    for (i=1; i<BEDTEMPTABLE_LEN; i++)
    {
      if (PGM_RD_W(BEDTEMPTABLE[i][0]) > raw)
      {
        celsius  = PGM_RD_W(BEDTEMPTABLE[i-1][1]) +
          (raw - PGM_RD_W(BEDTEMPTABLE[i-1][0])) *
          (float)(PGM_RD_W(BEDTEMPTABLE[i][1]) - PGM_RD_W(BEDTEMPTABLE[i-1][1])) /
          (float)(PGM_RD_W(BEDTEMPTABLE[i][0]) - PGM_RD_W(BEDTEMPTABLE[i-1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == BEDTEMPTABLE_LEN) celsius = PGM_RD_W(BEDTEMPTABLE[i-1][1]);

    return celsius;
  #elif defined BED_USES_AD595
    return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
  #else
    return 0;
  #endif
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
