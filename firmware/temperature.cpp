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

#include "ddprint.h"
#include "MarlinSerial.h"
#include "temperature.h"
#include "eepromSettings.h"


//===========================================================================
//=============================public variables============================
//===========================================================================
int target_temperature[EXTRUDERS] = { 0 };
int target_temperature_bed = 0;
float current_temperature[EXTRUDERS] = { 0.0 };
float current_temperature_bed = BED_MINTEMP;

#ifdef FAN_SOFT_PWM
  unsigned char fanSpeedSoftPwm;
#endif


//===========================================================================
//=============================private variables============================
//===========================================================================
// static volatile bool temp_meas_ready = false;

	// static unsigned long  previous_millis_bed_heater;

  static unsigned char soft_pwm[EXTRUDERS];
#ifdef FAN_SOFT_PWM
  static unsigned char soft_pwm_fan;
#endif
#if (defined(EXTRUDER_0_AUTO_FAN_PIN) && EXTRUDER_0_AUTO_FAN_PIN > -1) || \
    (defined(EXTRUDER_1_AUTO_FAN_PIN) && EXTRUDER_1_AUTO_FAN_PIN > -1) || \
    (defined(EXTRUDER_2_AUTO_FAN_PIN) && EXTRUDER_2_AUTO_FAN_PIN > -1)
  static unsigned long extruder_autofan_last_check;
#endif

// Init min and max temp with extreme values to prevent false errors during startup
static int minttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP , HEATER_2_RAW_LO_TEMP );
static int maxttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP , HEATER_2_RAW_HI_TEMP );
static int minttemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS( 0, 0, 0 );
static int maxttemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS( 16383, 16383, 16383 );
//static int bed_minttemp_raw = HEATER_BED_RAW_LO_TEMP; /* No bed mintemp error implemented?!? */

static void min_temp_error(uint8_t e);
static void max_temp_error(uint8_t e);

#ifdef BED_MAXTEMP
static int bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;
#endif

// static unsigned long max_heating_start_millis[EXTRUDERS];
// static float max_heating_start_temperature[EXTRUDERS];

  static void *heater_ttbl_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS( (void *)HEATER_0_TEMPTABLE, (void *)HEATER_1_TEMPTABLE, (void *)HEATER_2_TEMPTABLE );
  static uint8_t heater_ttbllen_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN, HEATER_2_TEMPTABLE_LEN );

static float analog2temp(int raw, uint8_t e);
static float analog2tempBed(int raw);
static void updateTemperaturesFromRawValues();

#ifdef WATCH_TEMP_PERIOD
// int watch_start_temp[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0,0,0);
// unsigned long watchmillis[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0,0,0);
#endif //WATCH_TEMP_PERIOD

#ifndef SOFT_PWM_SCALE
#define SOFT_PWM_SCALE 0
#endif
	
//===========================================================================
//=============================   functions      ============================
//===========================================================================

#if (defined(EXTRUDER_0_AUTO_FAN_PIN) && EXTRUDER_0_AUTO_FAN_PIN > -1) || \
    (defined(EXTRUDER_1_AUTO_FAN_PIN) && EXTRUDER_1_AUTO_FAN_PIN > -1) || \
    (defined(EXTRUDER_2_AUTO_FAN_PIN) && EXTRUDER_2_AUTO_FAN_PIN > -1)

  #if defined(FAN_PIN) && FAN_PIN > -1
    #if EXTRUDER_0_AUTO_FAN_PIN == FAN_PIN
       #error "You cannot set EXTRUDER_0_AUTO_FAN_PIN equal to FAN_PIN"
    #endif
    #if EXTRUDER_1_AUTO_FAN_PIN == FAN_PIN
       #error "You cannot set EXTRUDER_1_AUTO_FAN_PIN equal to FAN_PIN"
    #endif
    #if EXTRUDER_2_AUTO_FAN_PIN == FAN_PIN
       #error "You cannot set EXTRUDER_2_AUTO_FAN_PIN equal to FAN_PIN"
    #endif
  #endif

void setExtruderAutoFanState(int pin, bool state)
{
  unsigned char newFanSpeed = (state != 0) ? EXTRUDER_AUTO_FAN_SPEED : 0;
  // this idiom allows both digital and PWM fan outputs (see M42 handling).
  pinMode(pin, OUTPUT);
  digitalWrite(pin, newFanSpeed);
  analogWrite(pin, newFanSpeed);
}

void checkExtruderAutoFans()
{
  uint8_t fanState = 0;

  // which fan pins need to be turned on?
  #if defined(EXTRUDER_0_AUTO_FAN_PIN) && EXTRUDER_0_AUTO_FAN_PIN > -1
    if (current_temperature[0] > EXTRUDER_AUTO_FAN_TEMPERATURE)
      fanState |= 1;
  #endif
  #if defined(EXTRUDER_1_AUTO_FAN_PIN) && EXTRUDER_1_AUTO_FAN_PIN > -1 && EXTRUDERS > 1
    if (current_temperature[1] > EXTRUDER_AUTO_FAN_TEMPERATURE)
    {
      if (EXTRUDER_1_AUTO_FAN_PIN == EXTRUDER_0_AUTO_FAN_PIN)
        fanState |= 1;
      else
        fanState |= 2;
    }
  #endif
  #if defined(EXTRUDER_2_AUTO_FAN_PIN) && EXTRUDER_2_AUTO_FAN_PIN > -1 && EXTRUDERS > 2
    if (current_temperature[2] > EXTRUDER_AUTO_FAN_TEMPERATURE)
    {
      if (EXTRUDER_2_AUTO_FAN_PIN == EXTRUDER_0_AUTO_FAN_PIN)
        fanState |= 1;
      else if (EXTRUDER_2_AUTO_FAN_PIN == EXTRUDER_1_AUTO_FAN_PIN)
        fanState |= 2;
      else
        fanState |= 4;
    }
  #endif

  // update extruder auto fan states
  #if defined(EXTRUDER_0_AUTO_FAN_PIN) && EXTRUDER_0_AUTO_FAN_PIN > -1
    setExtruderAutoFanState(EXTRUDER_0_AUTO_FAN_PIN, (fanState & 1) != 0);
  #endif
  #if defined(EXTRUDER_1_AUTO_FAN_PIN) && EXTRUDER_1_AUTO_FAN_PIN > -1 && EXTRUDERS > 1
    if (EXTRUDER_1_AUTO_FAN_PIN != EXTRUDER_0_AUTO_FAN_PIN)
      setExtruderAutoFanState(EXTRUDER_1_AUTO_FAN_PIN, (fanState & 2) != 0);
  #endif
  #if defined(EXTRUDER_2_AUTO_FAN_PIN) && EXTRUDER_2_AUTO_FAN_PIN > -1 && EXTRUDERS > 2
    if (EXTRUDER_2_AUTO_FAN_PIN != EXTRUDER_0_AUTO_FAN_PIN
        && EXTRUDER_2_AUTO_FAN_PIN != EXTRUDER_1_AUTO_FAN_PIN)
      setExtruderAutoFanState(EXTRUDER_2_AUTO_FAN_PIN, (fanState & 4) != 0);
  #endif
}

#endif // any extruder auto fan pins set

#if 0
void manage_heater()
{
  // float pid_input;
  // float pid_output;

    for(uint8_t e=0;e<EXTRUDERS;e++)
    {
        current_temperature[e] = analog2temp(current_temperature_raw[e], e);
    }
    current_temperature_bed = analog2tempBed(current_temperature_bed_raw);
    //Reset the watchdog after we know we have a temperature measurement.
    watchdog_reset();

    // CRITICAL_SECTION_START;
    // temp_meas_ready = false;
    // CRITICAL_SECTION_END;

  for(int e = 0; e < EXTRUDERS; e++)
  {

    // Check if temperature is within the correct range
    if((current_temperature[e] > minttemp[e]) && (current_temperature[e] < maxttemp[e]))
    {
      soft_pwm[e] = (int)pid_output >> 1;
    }
    else {
      soft_pwm[e] = 0;
    }

    #ifdef WATCH_TEMP_PERIOD
    if(watchmillis[e] && millis() - watchmillis[e] > WATCH_TEMP_PERIOD)
    {
        if(degHotend(e) < watch_start_temp[e] + WATCH_TEMP_INCREASE)
        {
            setTargetHotend(0, e);
            LCD_MESSAGEPGM("Heating failed");
            SERIAL_ECHO_START;
            SERIAL_ECHOLN("Heating failed");
        }else{
            watchmillis[e] = 0;
        }
    }
    #endif
    if (pid_output == PID_MAX)
    {
        if (current_temperature[e] - max_heating_start_temperature[e] > MAX_HEATING_TEMPERATURE_INCREASE)
        {
            max_heating_start_millis[e] = 0;
        }
        if (max_heating_start_millis[e] == 0)
        {
            max_heating_start_millis[e] = millis();
            max_heating_start_temperature[e] = current_temperature[e];
        }
        if (millis() > max_heating_start_millis[e] + MAX_HEATING_CHECK_MILLIS)
        {
            //Did not heat up MAX_HEATING_TEMPERATURE_INCREASE in MAX_HEATING_CHECK_MILLIS while the PID was at the maximum.
            //Potential problems could be that the heater is not working, or the temperature sensor is not measuring what the heater is heating.
            disable_heater();
            Stop(STOP_REASON_HEATER_ERROR);
        }
    }else{
        max_heating_start_millis[e] = 0;
    }
  } // End extruder for loop

  #if (defined(EXTRUDER_0_AUTO_FAN_PIN) && EXTRUDER_0_AUTO_FAN_PIN > -1) || \
      (defined(EXTRUDER_1_AUTO_FAN_PIN) && EXTRUDER_1_AUTO_FAN_PIN > -1) || \
      (defined(EXTRUDER_2_AUTO_FAN_PIN) && EXTRUDER_2_AUTO_FAN_PIN > -1)
  if(millis() - extruder_autofan_last_check > 2500)  // only need to check fan state very infrequently
  {
    checkExtruderAutoFans();
    extruder_autofan_last_check = millis();
  }
  #endif

  {
    //For the UM2 the head fan is connected to PJ6, which does not have an Arduino PIN definition. So use direct register access.
    DDRJ |= _BV(6);
    if (current_temperature[0] > EXTRUDER_AUTO_FAN_TEMPERATURE
#if EXTRUDERS > 1
        || current_temperature[1] > EXTRUDER_AUTO_FAN_TEMPERATURE
#endif
#if EXTRUDERS > 2
        || current_temperature[2] > EXTRUDER_AUTO_FAN_TEMPERATURE
#endif
        )
    {
        PORTJ |= _BV(6);
    }else{
        PORTJ &=~_BV(6);
    }
  }

  if(millis() - previous_millis_bed_heater < BED_CHECK_INTERVAL)
    return;
  previous_millis_bed_heater = millis();

}
#endif

#define PGM_RD_W(x)   (short)pgm_read_word(&x)
// Derived from RepRap FiveD extruder::getTemperature()
// For hot end temperature measurement.
static float analog2temp(int raw, uint8_t e) {
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
static float analog2tempBed(int raw) {
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
/* Called to get the raw values into the the actual temperatures. The raw values are created in interrupt context,
    and this function is called from normal context as it is too slow to run in interrupts and will block the stepper routine otherwise */
static void updateTemperaturesFromRawValues()
{

    for(uint8_t e=0;e<EXTRUDERS;e++)
    {
        current_temperature[e] = analog2temp(current_temperature_raw[e], e);
    }
    current_temperature_bed = analog2tempBed(current_temperature_bed_raw);
    //Reset the watchdog after we know we have a temperature measurement.
    watchdog_reset();

    CRITICAL_SECTION_START;
    temp_meas_ready = false;
    CRITICAL_SECTION_END;
}
#endif

void tp_init()
{

  // Finish init of mult extruder arrays
  for(int e = 0; e < EXTRUDERS; e++) {
    // populate with the first value
    maxttemp[e] = maxttemp[0];
  }

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

#ifdef HEATER_0_MINTEMP
  minttemp[0] = HEATER_0_MINTEMP;
  while(analog2temp(minttemp_raw[0], 0) < HEATER_0_MINTEMP) {
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
    minttemp_raw[0] += OVERSAMPLENR;
#else
    minttemp_raw[0] -= OVERSAMPLENR;
#endif
  }
#endif //MINTEMP
#ifdef HEATER_0_MAXTEMP
  maxttemp[0] = HEATER_0_MAXTEMP;
  while(analog2temp(maxttemp_raw[0], 0) > HEATER_0_MAXTEMP) {
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
    maxttemp_raw[0] -= OVERSAMPLENR;
#else
    maxttemp_raw[0] += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP

#if (EXTRUDERS > 1) && defined(HEATER_1_MINTEMP)
  minttemp[1] = HEATER_1_MINTEMP;
  while(analog2temp(minttemp_raw[1], 1) < HEATER_1_MINTEMP) {
#if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
    minttemp_raw[1] += OVERSAMPLENR;
#else
    minttemp_raw[1] -= OVERSAMPLENR;
#endif
  }
#endif // MINTEMP 1
#if (EXTRUDERS > 1) && defined(HEATER_1_MAXTEMP)
  maxttemp[1] = HEATER_1_MAXTEMP;
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

#ifdef BED_MAXTEMP
  while(analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
    bed_maxttemp_raw -= OVERSAMPLENR;
#else
    bed_maxttemp_raw += OVERSAMPLENR;
#endif
  }
#endif //BED_MAXTEMP

    tempControl.init();
}

void setWatch()
{
#ifdef WATCH_TEMP_PERIOD
  for (int e = 0; e < EXTRUDERS; e++)
  {
    if(degHotend(e) < degTargetHotend(e) - (WATCH_TEMP_INCREASE * 2))
    {
      watch_start_temp[e] = degHotend(e);
      watchmillis[e] = millis();
    }
  }
#endif
}

void disable_heater()
{
    for(int i=0;i<EXTRUDERS;i++)
        setTargetHotend(0,i);
    WRITE(HEATER_0_PIN,LOW);

    setTargetBed(0);
    WRITE(HEATER_BED_PIN,LOW);
}

void max_temp_error(uint8_t e) {
  disable_heater();
  if(IsStopped() == false) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN((int)e);
    SERIAL_ERRORLNPGM(": Extruder switched off. MAXTEMP triggered !");
    LCD_ALERTMESSAGEPGM("Err: MAXTEMP");
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
    LCD_ALERTMESSAGEPGM("Err: MINTEMP");
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
    LCD_ALERTMESSAGEPGM("Err: MAXTEMP BED");
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
    LCD_ALERTMESSAGEPGM("Err: MINTEMP BED");
  }
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop(STOP_REASON_MINTEMP_BED);
  #endif
}

#if 0
// Timer 0 is shared with millies
ISR(TIMER0_COMPB_vect)
{
  //these variables are only accesible from the ISR, but static, so they don't lose their value
  static unsigned char temp_count = 0;
  static unsigned long raw_temp_0_value = 0;
  static unsigned long raw_temp_1_value = 0;
  static unsigned long raw_temp_2_value = 0;
  static unsigned long raw_temp_bed_value = 0;
  static unsigned char temp_state = 5;
  static unsigned char pwm_count = (1 << SOFT_PWM_SCALE);
  static unsigned char soft_pwm_0;
  #if EXTRUDERS > 1
  static unsigned char soft_pwm_1;
  #endif
  #if EXTRUDERS > 2
  static unsigned char soft_pwm_2;
  #endif
  #if HEATER_BED_PIN > -1
  static unsigned char soft_pwm_b;
  #endif

  if(pwm_count == 0){
    soft_pwm_0 = soft_pwm[0];
    if(soft_pwm_0 > 0) WRITE(HEATER_0_PIN,1);
    #if EXTRUDERS > 1
    soft_pwm_1 = soft_pwm[1];
    if(soft_pwm_1 > 0) WRITE(HEATER_1_PIN,1);
    #endif
    #if EXTRUDERS > 2
    soft_pwm_2 = soft_pwm[2];
    if(soft_pwm_2 > 0) WRITE(HEATER_2_PIN,1);
    #endif
    #if defined(HEATER_BED_PIN) && HEATER_BED_PIN > -1
    soft_pwm_b = soft_pwm_bed;
    if(soft_pwm_b > 0) WRITE(HEATER_BED_PIN,1);
    #endif
    #ifdef FAN_SOFT_PWM
    soft_pwm_fan = fanSpeedSoftPwm / 2;
    if(soft_pwm_fan > 0) WRITE(FAN_PIN,1);
    #endif
  }
  if(soft_pwm_0 <= pwm_count) WRITE(HEATER_0_PIN,0);
  #if EXTRUDERS > 1
  if(soft_pwm_1 <= pwm_count) WRITE(HEATER_1_PIN,0);
  #endif
  #if EXTRUDERS > 2
  if(soft_pwm_2 <= pwm_count) WRITE(HEATER_2_PIN,0);
  #endif
  #if defined(HEATER_BED_PIN) && HEATER_BED_PIN > -1
  if(soft_pwm_b <= pwm_count) WRITE(HEATER_BED_PIN,0);
  #endif
  #ifdef FAN_SOFT_PWM
  if(soft_pwm_fan <= pwm_count) WRITE(FAN_PIN,0);
  #endif

  pwm_count += (1 << SOFT_PWM_SCALE);
  pwm_count &= 0x7f;

  switch(temp_state) {
    case 1: // Measure TEMP_0
      #if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
        raw_temp_0_value += ADC;
      #endif
      // Prepare TEMP_BED
      #if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
        #if TEMP_BED_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_BED_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      lcd_buttons_update();
      temp_state = 2;
      break;
    case 2: // Measure TEMP_BED
      #if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
        raw_temp_bed_value += ADC;
      #endif
      // Prepare TEMP_1
      #if defined(TEMP_1_PIN) && (TEMP_1_PIN > -1) && EXTRUDERS > 1
        #if TEMP_1_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_1_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      lcd_buttons_update();
      temp_state = 3;
      break;
    case 3: // Measure TEMP_1
      #if defined(TEMP_1_PIN) && (TEMP_1_PIN > -1) && EXTRUDERS > 1
        raw_temp_1_value += ADC;
      #endif
      // Prepare TEMP_2
      #if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1) && EXTRUDERS > 2
        #if TEMP_2_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_2_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      lcd_buttons_update();
      temp_state = 4;
      break;
    case 4: // Measure TEMP_2
      #if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1) && EXTRUDERS > 2
        raw_temp_2_value += ADC;
      #endif
      temp_count++;
      //Fall trough to state 0
    case 0: // Prepare TEMP_0
      #if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
        #if TEMP_0_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_0_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      lcd_buttons_update();
      temp_state = 1;
      break;
    case 5: //Startup, delay initial temp reading a tiny bit so the hardware can settle.
      temp_state = 0;
      break;
//    default:
//      SERIAL_ERROR_START;
//      SERIAL_ERRORLNPGM("Temp measurement error!");
//      break;
  }

  if(temp_count >= OVERSAMPLENR) // 8 ms * 16 = 128ms.
  {
    if (!temp_meas_ready) //Only update the raw values if they have been read. Else we could be updating them during reading.
    {
      current_temperature_raw[0] = raw_temp_0_value;
#if EXTRUDERS > 1
      current_temperature_raw[1] = raw_temp_1_value;
#endif
#if EXTRUDERS > 2
      current_temperature_raw[2] = raw_temp_2_value;
#endif
      current_temperature_bed_raw = raw_temp_bed_value;
    }

    temp_meas_ready = true;
    temp_count = 0;
    raw_temp_0_value = 0;
    raw_temp_1_value = 0;
    raw_temp_2_value = 0;
    raw_temp_bed_value = 0;

#if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
    if(current_temperature_raw[0] <= maxttemp_raw[0]) {
#else
    if(current_temperature_raw[0] >= maxttemp_raw[0]) {
#endif
       max_temp_error(0);
    }
#if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
    if(current_temperature_raw[0] >= minttemp_raw[0]) {
#else
    if(current_temperature_raw[0] <= minttemp_raw[0]) {
#endif
       min_temp_error(0);
    }
#if EXTRUDERS > 1
#if HEATER_1_RAW_LO_TEMP > HEATER_1_RAW_HI_TEMP
    if(current_temperature_raw[1] <= maxttemp_raw[1]) {
#else
    if(current_temperature_raw[1] >= maxttemp_raw[1]) {
#endif
        max_temp_error(1);
    }
#if HEATER_1_RAW_LO_TEMP > HEATER_1_RAW_HI_TEMP
    if(current_temperature_raw[1] >= minttemp_raw[1]) {
#else
    if(current_temperature_raw[1] <= minttemp_raw[1]) {
#endif
        min_temp_error(1);
    }
#endif
#if EXTRUDERS > 2
#if HEATER_2_RAW_LO_TEMP > HEATER_2_RAW_HI_TEMP
    if(current_temperature_raw[2] <= maxttemp_raw[2]) {
#else
    if(current_temperature_raw[2] >= maxttemp_raw[2]) {
#endif
        max_temp_error(2);
    }
#if HEATER_2_RAW_LO_TEMP > HEATER_2_RAW_HI_TEMP
    if(current_temperature_raw[2] >= minttemp_raw[2]) {
#else
    if(current_temperature_raw[2] <= minttemp_raw[2]) {
#endif
        min_temp_error(2);
    }
#endif

  /* No bed MINTEMP error? */
#if defined(BED_MAXTEMP) && (TEMP_SENSOR_BED != 0)
# if HEATER_BED_RAW_LO_TEMP > HEATER_BED_RAW_HI_TEMP
    if(current_temperature_bed_raw <= bed_maxttemp_raw) {
#else
    if(current_temperature_bed_raw >= bed_maxttemp_raw) {
#endif
       target_temperature_bed = 0;
       bed_max_temp_error();
    }
#endif
  }
}
#endif


void TempControl::init() {
    //
    // Set analog inputs
    //
    // Disable 'digital input register' on the ADC pins TEMP_0_PIN and TEMP_BED_PIN
    DIDR0 = 0;
    #ifdef DIDR2
        DIDR2 = 0;
    #endif

    #if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
    #if TEMP_0_PIN < 8
       DIDR0 |= 1 << TEMP_0_PIN;
    #else
       DIDR2 |= 1<<(TEMP_0_PIN - 8);
    #endif

    #endif
    #if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
    #if TEMP_BED_PIN < 8
       DIDR0 |= 1<<TEMP_BED_PIN;
    #else
       DIDR2 |= 1<<(TEMP_BED_PIN - 8);
    #endif
    #endif

    // ADEN: Enable ADC
    // ADSC: Do initial conversion
    // ADIF: Clear interrupt flag?
    // 0x07: Set 16MHz/128 = 125kHz the ADC reference clock
    // ADCSRA = 1<<ADEN | 1<<ADSC | 1<<ADIF | 0x07;
    ADCSRA = 1<<ADEN | 1<<ADSC | 0x07;

    // Wait for initial conversion
    while (ADCSRA & (1<<ADSC));

    //
    // Start initial hotend measure
    //
    #if TEMP_0_PIN > 7
      ADCSRB = 1<<MUX5;
    #else
      ADCSRB = 0;
    #endif

    // Set voltage reference to Avcc, set channel to temp 0
    ADMUX = ((1 << REFS0) | (TEMP_0_PIN & 0x07));
    ADCSRA |= 1<<ADSC; // Start conversion

    // Wait for initial conversion and read value
    while (ADCSRA & (1<<ADSC));
    raw_temp_0_value = ADC * OVERSAMPLENR;

    printf("Initial hotend raw temp: %d\n", raw_temp_0_value);

    //
    // Start initial bedtemp measure
    //
    #if TEMP_BED_PIN > 7
      ADCSRB = 1<<MUX5;
    #else
      ADCSRB = 0;
    #endif

    // Set voltage reference to Avcc, set channel to bedtemp
    ADMUX = ((1 << REFS0) | (TEMP_BED_PIN & 0x07));
    ADCSRA |= 1<<ADSC; // Start conversion

    // Wait for initial conversion and read value
    while (ADCSRA & (1<<ADSC));
    raw_temp_bed_value = ADC * OVERSAMPLENR;
    
    printf("Initial hotend raw temp: %d\n", raw_temp_bed_value);

    //
    // Get PID values from eeprom
    //
    EepromSettings es;
    getEepromSettings(es);
    Kp = es.Kp;
    Ki = es.Ki;
    Kd = es.Kd;

    temp_iState = 0;
    temp_dState = 0;

    //
    // Timestamp of last pid computation
    //
    lastPidCompute = millis();

    for (uint8_t e=0; e<EXTRUDERS; e++)
        current_temperature[e] = HEATER_0_MINTEMP;
}

bool TempControl::Run() {

    PT_BEGIN();

    ////////////////////////////////
    // Handle hotend 
    ////////////////////////////////

    //
    // Start hotend measure
    //
    #if TEMP_0_PIN > 7
      ADCSRB = 1<<MUX5;
    #else
      ADCSRB = 0;
    #endif

    // Set voltage reference to Avcc, set channel to temp 0
    ADMUX = ((1 << REFS0) | (TEMP_0_PIN & 0x07));
    ADCSRA |= 1<<ADSC; // Start conversion

    // Wait for conversion and read value
    // printf("TempControl::Run() wait for hotend\n");
    PT_WAIT_WHILE( ADCSRA & (1<<ADSC) );

    raw_temp_0_value = raw_temp_0_value - (raw_temp_0_value / OVERSAMPLENR) + ADC;
    // printf("read hotend raw temp: %d\n", raw_temp_0_value);

    current_temperature[0] = analog2temp(raw_temp_0_value, 0);

    ////////////////////////////////
    // Handle heated bed 
    ////////////////////////////////

    //
    // Start bedtemp measure
    //
    #if TEMP_BED_PIN > 7
      ADCSRB = 1<<MUX5;
    #else
      ADCSRB = 0;
    #endif

    // Set voltage reference to Avcc, set channel to bedtemp
    ADMUX = ((1 << REFS0) | (TEMP_BED_PIN & 0x07));
    ADCSRA |= 1<<ADSC; // Start conversion

    // Wait for conversion and read value
    // printf("TempControl::Run() wait for bed\n");
    PT_WAIT_WHILE( ADCSRA & (1<<ADSC) );

    raw_temp_bed_value = raw_temp_bed_value - (raw_temp_bed_value / OVERSAMPLENR) + ADC;
    // printf("read hotend raw temp: %d\n", raw_temp_bed_value);

    current_temperature_bed = analog2tempBed(raw_temp_bed_value);

    PT_RESTART();
        
    // Not reached
    PT_END();
}

void TempControl::heater() {

    // Check if temperature is within the correct range
    if(current_temperature[0] < HEATER_0_MINTEMP) {
        min_temp_error(0);
    }
    else {
        if(current_temperature[0] > HEATER_0_MAXTEMP) {
            max_temp_error(0);
        }
        else {

#ifdef PIDTEMP
            // >>> OVERSAMPLENR=8
            // >>> F_CPU=16000000
            // >>> PID_dT = ((OVERSAMPLENR * 4.0)/(F_CPU / 64.0 / 256.0))
            // >>> print PID_dT
            // 0.032768 s

            // HEATER_0_PIN = digital pin 2, PE4 ( OC3B/INT4 ), real pin 6? PWM output B timer 3 
            // #define FAN_PIN            7, PE5 ( OC3C/INT5 )  Digital pin 3 (PWM)

            // ---------------------------------------------------

            unsigned long ts = millis();
            float pid_dt = (ts - lastPidCompute) / 1000.0;

            float ki = (Ki * pid_dt) / 10;
            float kd = Kd / pid_dt;

            float pid_error = target_temperature[0] - current_temperature[0];

            float pTerm = Kp * pid_error / 2;

            temp_iState += pid_error;

            // temp_iState_max[e] = PID_INTEGRAL_DRIVE_MAX / Ki;
            temp_iState = constrain(temp_iState, 0.0, PID_INTEGRAL_DRIVE_MAX / ki);

            float iTerm = ki * temp_iState;

            //K1 defined in Configuration.h in the PID settings
            #define K2 (1.0-K1)
            float dTerm = (kd * (current_temperature[0] - temp_dState))*K2 + (K1 * dTerm);

            temp_dState = current_temperature[0];

            // float pid_output = constrain(pTerm + iTerm - dTerm, 0, 255.0);
            float pid_output = pTerm + iTerm - dTerm;
            if (pid_output > 255.0)
                pid_output = 255;
            else if (pid_output < 0)
                pid_output = 0;

            analogWrite(HEATER_0_PIN, (int)pid_output);

// #ifdef PID_DEBUG
            static int dbgcount=0;

            if ((dbgcount++ % 10) == 0) {

                SERIAL_ECHOPGM(" PIDDEBUG ");
                SERIAL_ECHOPGM(": Input ");
                SERIAL_ECHO(current_temperature[0]);
                SERIAL_ECHOPGM(" pid_dt ");
                SERIAL_ECHO(pid_dt);
                SERIAL_ECHOPGM(" pTerm ");
                SERIAL_ECHO(pTerm);
                SERIAL_ECHOPGM(" iTerm ");
                SERIAL_ECHO(iTerm);
                SERIAL_ECHOPGM(" dTerm ");
                SERIAL_ECHO(dTerm);
                SERIAL_ECHOPGM(" Output ");
                SERIAL_ECHOLN(pid_output);
            }

// #endif //PID_DEBUG

            lastPidCompute = ts;

            // ---------------------------------------------------

#else
            if(current_temperature[0] >= target_temperature[0])
            {
                WRITE(HEATER_0_PIN, LOW);
            }
            else
            {
                WRITE(HEATER_0_PIN, HIGH);
            }
#endif
        }
    }

    ////////////////////////////////
    // Handle heated bed 
    ////////////////////////////////

    // Check if temperature is within the correct range
    if(current_temperature_bed < BED_MINTEMP) {
        bed_min_temp_error();
    }
    else {
        if(current_temperature_bed > BED_MAXTEMP) {
            bed_max_temp_error();
        }
        else {
            if(current_temperature_bed >= target_temperature_bed)
            {
                WRITE(HEATER_BED_PIN, LOW);
            }
            else
            {
                WRITE(HEATER_BED_PIN, HIGH);
            }
        }
    }

    //
    // Reset the watchdog after we know we have a temperature measurement.
    //
    watchdog_reset();
}

TempControl tempControl;



