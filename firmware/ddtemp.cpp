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
#include "ddlcd.h"

//
// ADC is running with a clock of 125 khz. One ADC conversion needs 13 ADC clock cycles,
// this gives us about 10khz max. sampling rate (one conversion is ~100 uS).
//

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

    cobias = 0;
    pid_output = 0;

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
    current_temperature[0] = tempFromRawADC(raw_temp_0_value);

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
    current_temperature_bed = tempFromRawADC(raw_temp_bed_value);

    PT_RESTART();
        
    // Not reached
    PT_END();
}

void TempControl::setTemp(uint8_t heater, uint16_t temp) {

    if (heater == 0)
        target_temperature_bed = temp;
    else {

        // Keep integral eSum if changing setpont from a
        // already stable setpoint. Reset integral sum if
        // we switch "from manual mode to automatic".
        if (target_temperature[heater-1] == 0) {

            // Switch to "automatic mode", bumpless

            // Bumpless mode

            // We achieve this desired outcome at switchover by initializing the controller integral sum of error to zero.
            eSum = 0;
            // Also, the set point and controller bias value are initialized by setting:
            //    ▪ SP equal to the current PV
            //    ▪ CObias equal to the current CO
            // sp = pv;
            cobias = pid_output;

            // CO = controller output signal (the wire out)
            // CObias = controller bias; set by bumpless transfer
            //
            // e(t) = current controller error, defined as SP – PV
            // SP = set point
            // PV = measured process variable (the wire in)
            //
            // Thus, when switching to automatic, SP is set equal to the current PV and CObias is set equal to the current CO.
            //
        }

        target_temperature[heater-1] = temp;

        // eAlt = 0;
    }
}

void TempControl::heater() {

    // Check if temperature is within the correct range
    if(current_temperature[0] < HEATER_0_MINTEMP) {
        LCDMSGKILL(RespMinTemp, 1, current_temperature[0]);
        txBuffer.sendSimpleResponse(RespKilled, RespMinTemp, 1);
        kill();
    }
    else {
        if(current_temperature[0] > HEATER_0_MAXTEMP) {
            LCDMSGKILL(RespMaxTemp, 1, current_temperature[0]);
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

            // Regeldifferenz, grösser 0 falls temperatur zu klein
            float e = target_temperature[0] - current_temperature[0];

            eSum = constrain(
                    eSum + e,
                    -PID_DRIVE_MAX,
                    PID_DRIVE_MAX);

            float pTerm = Kp * e;

            float iTerm = Ki * pid_dt * eSum;

            float dTerm = Kd * (e - eAlt) / pid_dt;

            // int32_t pid_output = cobias + pTerm + iTerm + dTerm + 0.5;
            pid_output = cobias + pTerm + iTerm + dTerm + 0.5;

            if (pid_output > PID_MAX) {
                pid_output = PID_MAX;
            }
            else if (pid_output < -PID_MAX) {
                pid_output = -PID_MAX;
            }

            analogWrite(HEATER_0_PIN, max(pid_output, 0));

            eAlt = e;

#ifdef PID_DEBUG

            static int dbgcount=0;
            
            if ((dbgcount++ % 10) == 0) {

                txBuffer.sendResponseStart(RespUnsolicitedMsg);
                txBuffer.sendResponseUint8(PidDebug);
                txBuffer.sendResponseValue(pid_dt);
                txBuffer.sendResponseValue(pTerm);
                txBuffer.sendResponseValue(iTerm);
                txBuffer.sendResponseValue(dTerm);
                /* txBuffer.sendResponseValue(pwmSum); */
                txBuffer.sendResponseInt32(pid_output);
                txBuffer.sendResponseEnd();
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
        LCDMSGKILL(RespMinTemp, 0, current_temperature_bed);
        txBuffer.sendSimpleResponse(RespKilled, RespMinTemp, 0);
        kill();
    }
    else {
        if(current_temperature_bed > BED_MAXTEMP) {
            LCDMSGKILL(RespMaxTemp, 0, current_temperature_bed);
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

