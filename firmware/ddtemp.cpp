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
#include "ddserial.h"
#include "ddcommands.h"

#include "hal.h"
#include "ddlcd.h"

//
// ADC is running with a clock of 125 khz. One ADC conversion needs 13 ADC clock cycles,
// this gives us about 10khz max. sampling rate (one conversion is ~100 uS).
//

// Redundant definitions to avoid include of ddprint.h
extern void kill();

void TempControl::init() {

    HAL_SETUP_TEMP_ADC();

    eSum = 0;
    eAlt = 0;
    pwmMode = false;

    // cobias = 0;
    pid_output = 0;

    //
    // Timestamp of last pid computation
    //
    lastPidCompute = millis();

    for (uint8_t e=0; e<EXTRUDERS; e++)
        current_temperature[e] = HEATER_1_MINTEMP;

    // isum begrenzen auf max. 255 output
    //
    // Ki * pid_dt * eSum = iTerm < 255
    // eSummax = 255 / (Ki * pid_dt)
    //
    eSumLimit = 255.0 / ((Ki * TIMER100MS) / 1000.0);
}

void TempControl::setTemp(uint8_t heater, uint16_t temp) {

    if (heater == 0) {

        if (temp > HEATER_0_MAXTEMP)
            return;

        target_temperature_bed = temp;
    }
    else {

        // Hack, use hotend1 maxtemp for all hotends here.
        massert(temp <= HEATER_1_MAXTEMP);

        // Keep integral eSum if changing setpont from a
        // already stable setpoint. Reset integral sum if
        // we switch "from manual mode to automatic".
        if (target_temperature[heater-1] == 0) {
            eSum = 0;
            // cobias = 0;
        }
        else {

            // Switch to "automatic mode", bumpless

            // Bumpless mode

            // We achieve this desired outcome at switchover by initializing the controller integral sum of error to zero.
            // eSum = 0;
            // Also, the set point and controller bias value are initialized by setting:
            //    ▪ SP equal to the current PV
            //    ▪ CObias equal to the current CO
            // sp = pv;
            // cobias = pid_output;

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

        // if (pwmMode && (temp < target_temperature[heater-1]))
            // eSum = 0;

        target_temperature[heater-1] = temp;

        // xxx hack, reset/clear suggestPwm
        if (temp < 50) {
            suggestPwm = 0;
            suggestUsed = true;
        }
        // eAlt = 0;
    }
}

void TempControl::heater() {

    // Check if temperature is within the correct range
    if(current_temperature[0] < HEATER_1_MINTEMP) {
        LCDMSGKILL(RespMinTemp, 1, current_temperature[0]);
        txBuffer.sendSimpleResponse(RespKilled, RespMinTemp, 1);
        kill();
    }
    else {
        if(current_temperature[0] > HEATER_1_MAXTEMP) {
            LCDMSGKILL(RespMaxTemp, 1, current_temperature[0]);
            txBuffer.sendSimpleResponse(RespKilled, RespMaxTemp, 1);
            kill();
        }
        else {

            if (pwmMode) {

                HEATER_0_PIN :: write(pwmValueOverride);
            }
            else {

                // y = Kp*e + Ki*Ta*esum + Kd/Ta*(e – ealt);   //Reglergleichung

                // PID interval [s]
                unsigned long ts = millis();
                float pid_dt = (ts - lastPidCompute) / 1000.0;

                // Regeldifferenz, grösser 0 falls temperatur zu klein
                float e = target_temperature[0] - current_temperature[0];

                if (! antiWindupMode) {
                    eSum = constrain(
                        eSum + e,
                        -eSumLimit,
                        eSumLimit);
                }

                float pTerm = Kp * e;

                float iTerm = Ki * pid_dt * eSum;

                float dTerm = Kd * (e - eAlt) / pid_dt;

                // int32_t pid_output = cobias + pTerm + iTerm + dTerm + 0.5;
                pid_output = pTerm + iTerm + dTerm + 0.5;

                if (pid_output > PID_MAX) {
                    pid_output = PID_MAX;
                    antiWindupMode = true;
                }
                else if (pid_output < 0) {
                    pid_output = 0;
                    antiWindupMode = true;
                }
                else {
                    antiWindupMode = false;
                }

                // if ((pid_output < suggestPwm) && (e > 0.0)) {
                // if (e > 0.0) {
                if (suggestPwm && (! suggestUsed)) {

                    if (e > 5.0) {           // xxxx avoid to often disturb pid with suggest pid XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
                        // temp to low
                        if (pid_output < suggestPwm)
                            pid_output = suggestPwm;
                    }
                    else {
                        // temp reached
                        // bumpless transfer
                        //
                        // xxxx recalcuated to mucht here 
                        //
                        eSum = (suggestPwm - (pTerm+dTerm)) / (Ki * pid_dt); // xxx term ki*dt
                        iTerm = Ki * pid_dt * eSum; // xxx term ki*dt

                        pid_output = pTerm + iTerm + dTerm + 0.5;

                        // switch to pid mode
                        // suggestPwm = 0;
                        suggestUsed = true;
    
                if (pid_output > PID_MAX) {
                    pid_output = PID_MAX;
                    antiWindupMode = true;
                }
                else if (pid_output < 0) {
                    pid_output = 0;
                    antiWindupMode = true;
                }
                else {
                    antiWindupMode = false;
                }
                    }
                }

                HEATER_0_PIN :: write( pid_output );

#ifdef PID_DEBUG
                static int dbgcount=0;
    
                if ((dbgcount++ % 10) == 0) {
                    txBuffer.sendResponseStart(RespUnsolicitedMsg);
                    txBuffer.sendResponseUint8(PidDebug);
                    txBuffer.sendResponseFloat(pid_dt);
                    txBuffer.sendResponseFloat(pTerm);
                    txBuffer.sendResponseFloat(iTerm);
                    txBuffer.sendResponseFloat(dTerm);
                    txBuffer.sendResponseInt32(pid_output);
                    txBuffer.sendResponseFloat(e);
                    txBuffer.sendResponseEnd();
                }

#endif //PID_DEBUG

                // Kept values of last pidrun
                eAlt = e;
                lastPidCompute = ts;
            }
        }
    }

    ////////////////////////////////
    // Handle heated bed 
    ////////////////////////////////

    // Check if temperature is within the correct range
    if(current_temperature_bed < HEATER_0_MINTEMP) {
        LCDMSGKILL(RespMinTemp, 0, current_temperature_bed);
        txBuffer.sendSimpleResponse(RespKilled, RespMinTemp, 0);
        kill();
    }
    else {
        if(current_temperature_bed > HEATER_0_MAXTEMP) {
            LCDMSGKILL(RespMaxTemp, 0, current_temperature_bed);
            txBuffer.sendSimpleResponse(RespKilled, RespMaxTemp, 0);
            kill();
        }
        else {
            if(current_temperature_bed >= target_temperature_bed)
            {
                HEATER_BED_PIN :: deActivate();
            }
            else
            {
                HEATER_BED_PIN :: activate();
            }
        }
    }

    //
    // Reset the watchdog after we know we have a temperature measurement.
    //
    WDT_RESET();
}

void TempControl::setTempPWM(uint8_t heater, uint8_t pwmValue) {

    if (heater == 1) 
        HEATER_0_PIN :: write(pwmValue);
    else
        HEATER_1_PIN :: write(pwmValue);

    if (pwmValue)
        pwmMode = true;
    else
        pwmMode = false;

    pwmValueOverride = pwmValue;
}

void TempControl::hotendOn(uint8_t heater) {

    if (heater == 1) 
        HEATER_0_PIN :: write(255);
    else
        HEATER_1_PIN :: write(255);
}

TempControl tempControl;

