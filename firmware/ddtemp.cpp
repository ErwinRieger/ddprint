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

#include <Arduino.h>

#include "ddtemp.h"
#include "temperature.h"
#include "pins.h"
#include "thermistortables.h"
#include "eepromSettings.h"
#include "Configuration.h"
#include "ddserial.h"
#include "ddcommands.h"
#include "fastio.h"

// Redundant definitions to avoid include of ddprint.h
extern void watchdog_reset();
extern void kill();

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

    // printf("Initial hotend raw temp: %d\n", raw_temp_0_value);

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
    
    // printf("Initial hotend raw temp: %d\n", raw_temp_bed_value);

    //
    // Get PID values from eeprom
    //
    EepromSettings es;
    getEepromSettings(es);
    Kp = es.Kp;
    Ki = es.Ki;
    Kd = es.Kd;

    eSum = 0;
    eAlt = 0;

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
        txBuffer.sendSimpleResponse(RespKilled, RespMinTemp, 1);
        kill();
    }
    else {
        if(current_temperature[0] > HEATER_0_MAXTEMP) {
            txBuffer.sendSimpleResponse(RespKilled, RespMaxTemp, 1);
            kill();
        }
        else {
#if ! defined(PIDAutoTune)
#ifdef PIDTEMP

            // y = Kp*e + Ki*Ta*esum + Kd/Ta*(e – ealt);   //Reglergleichung

            // PID interval [s]
            unsigned long ts = millis();
            float pid_dt = (ts - lastPidCompute) / 1000.0;

            // Regelabweichung
            float e = target_temperature[0] - current_temperature[0];

            float ki = Ki * pid_dt;
            // float kd = Kd / pid_dt;

            float pTerm = Kp * e;

            // eSum += e;
            eSum = constrain(eSum + e, -PID_INTEGRAL_DRIVE_MAX/ki, PID_INTEGRAL_DRIVE_MAX/ki);

            float iTerm = ki*eSum;

            float dTerm = (Kd/pid_dt) * (e - eAlt);

            float pid_output = pTerm + iTerm + dTerm;

            if (pid_output > PID_MAX)
                pid_output = PID_MAX;
            else if (pid_output < 0)
                pid_output = 0;

            analogWrite(HEATER_0_PIN, (int)pid_output);

            eAlt = e;

#ifdef PID_DEBUG
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

#endif //PID_DEBUG

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
#endif  // PIDTEMP
#endif  // PIDAutoTune
        }
    }

    ////////////////////////////////
    // Handle heated bed 
    ////////////////////////////////

    // Check if temperature is within the correct range
    if(current_temperature_bed < BED_MINTEMP) {
        txBuffer.sendSimpleResponse(RespKilled, RespMinTemp, 0);
        kill();
    }
    else {
        if(current_temperature_bed > BED_MAXTEMP) {
            txBuffer.sendSimpleResponse(RespKilled, RespMaxTemp, 0);
            kill();
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

#if defined(PIDAutoTune)
// Set heater PWM value directly for PID AutoTune
void TempControl::setHeaterY(uint8_t heater, uint8_t pwmValue) {

    if (heater == 1) 
        analogWrite(HEATER_0_PIN, pwmValue);
    else
        analogWrite(HEATER_1_PIN, pwmValue);
}
#endif

TempControl tempControl;

