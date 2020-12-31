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

// T-Sum method PID parameters (fast):
// static struct PidSet pidSetHeating = { 0.6334, 0.0071, 0.0293 };
// T-Sum method PI (fast)
// static struct PidSet pidSetHeating = { 0.3167, 0.0041, 0 };

// Chien aperiodisch PID, gute führung:
// static struct PidSet pidSetHeating = { 3.5432, 0.0349, 1.3030 };

// Ziegler PID 1.2:
// static struct PidSet pidSetHeating = { 6.4368, 0.6008, 2.4034 };
// Ziegler PI
// static struct PidSet pidSetHeating = { 5.3148, 0.2935, 0.0 };




// static struct PidSet pidSetCooling = { 0.6334, 0.0071, 0.0293 };
// static struct PidSet pidSetCooling = { 0.3167, 0.0041, 0 };

// static struct PidSet pidSetCooling = { 3.5432, 0.0349, 1.3030 };

// static struct PidSet pidSetCooling = { 6.4368, 0.6008, 2.4034 };
// static struct PidSet pidSetCooling = { 5.3148, 0.2935, 0.0 };



// Redundant definitions to avoid include of ddprint.h
extern void kill();

TempControl::TempControl():
            avgBedTemp(HEATER_0_MINTEMP),
            avgHotendTemp(HEATER_1_MINTEMP),
            // Kp(1.0),
            // Ki(0.1),
            // Kd(1.0),
            curPidSet(&pidSetHeating),
            pwmValueOverride(0),
            antiWindupMode(false),
            pidSetHeating({1, 0.1, 0.1}),
            pidSetCooling({1, 0.1, 0.1}),
            suggestPwm(0)
            {};

void TempControl::init() {

    HAL_SETUP_TEMP_ADC();

    eSum = 0;
    eAlt = 0;
    dTerm = 0;
    pwmMode = false;

    pid_output = 0;

    //
    // Timestamp of last pid computation
    //
    lastPidCompute = millis();

    for (uint8_t e=0; e<EXTRUDERS; e++)
        current_temperature[e] = HEATER_1_MINTEMP;

    setPidSet(&pidSetHeating, 0.0, 1.0);
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
            dTerm = 0;
        }

        target_temperature[heater-1] = temp;

        // xxx hack, reset/clear suggestPwm
        if (temp < 50) {
            suggestPwm = 0;
        }
    }
}

void TempControl::choosePidSet(float e, float pid_dt) {

    if (e > 0) {
        // temp to low, use fast pid
        if (curPidSet != &pidSetHeating) {
            setPidSet(&pidSetHeating, e, pid_dt);
        }
    }
    else {
        // temp to high, use slow pid
        if (curPidSet != &pidSetCooling) {
            setPidSet(&pidSetCooling, e, pid_dt);
        }
    }
}

void TempControl::setPidSet(struct PidSet *pidSet, float e, float pid_dt) {

    float pTermOld = curPidSet->Kp * e;
    float iTermOld = curPidSet->Ki * pid_dt * eSum;

    float pTermNew = pidSet->Kp * e;
    float iTermNew = pidSet->Ki * pid_dt * eSum;

    // iTerm = curPidSet->Ki * pid_dt * eSum;
    eSum += ((pTermOld-pTermNew) + (iTermOld-iTermNew)) / (pidSet->Ki * pid_dt);
    
    curPidSet = pidSet;

    eSumLimit = 255.0 / ((pidSet->Ki * TIMER100MS) / 1000.0);
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

                choosePidSet(e, pid_dt);

                float pTerm = curPidSet->Kp * e;

                // float dTerm = curPidSet->Kd * (e - eAlt) / pid_dt;
                dTerm += curPidSet->Kd * (e - eAlt) / pid_dt;

                if (! antiWindupMode) {

                    float newEsum;
#if 0
                    if ((e > 2.5) && (pid_output < suggestPwm)) {           // xxxx avoid to often disturb pid with suggest pid XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
                        // temp to low
                        newEsum = (suggestPwm - (pTerm+dTerm)) / (curPidSet->Ki * pid_dt);
                        suggestPwm = 0;
                    }
                    else {
#endif
                        newEsum = eSum + e;
                    // }

                    eSum = constrain(
                        newEsum,
                        -eSumLimit,
                        eSumLimit);
                }

                float iTerm = curPidSet->Ki * pid_dt * eSum;

                float out = pTerm + iTerm;

                if (dTerm > 0) {
                    if (out < PID_MAX) {
                        float d = min(dTerm, PID_MAX-out);
                        out += d;
                        dTerm -= d;
                    }
                }
                else {
                    if (out > 0) {
                        float d = min(abs(dTerm), out);
                        out -= d;
                        dTerm += d;
                    }
                }

                // pid_output = pTerm + iTerm + dTerm + 0.5;
                pid_output = out + dTerm + 0.5;

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

void TempControl::setPIDValues(float kp, float ki, float kd, float kpC, float kiC, float kdC) {

    pidSetHeating.Kp = kp;
    pidSetHeating.Ki = ki;
    pidSetHeating.Kd = kd;

    pidSetCooling.Kp = kpC;
    pidSetCooling.Ki = kiC;
    pidSetCooling.Kd = kdC;
}

TempControl tempControl;

