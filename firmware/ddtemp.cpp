/*
* This file is part of ddprint - a 3D printer firmware.
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
#include "txbuffer.h"
#include "ddcommands.h"

#include "hal.h"
#include "ddlcd.h"

//
// ADC is running with a clock of 125 khz. One ADC conversion needs 13 ADC clock cycles,
// this gives us about 10khz max. sampling rate (one conversion is ~100 uS).
//

// Redundant definitions to avoid include of ddprint.h
extern void kill();

TempControl::TempControl():
            avgBedTemp(HEATER_0_MINTEMP),
            avgHotendTemp(HEATER_1_MINTEMP),
            curPidSet(&pidSetHeating),
            continousPID(0),
            antiWindupMode(false),
            pidSetHeating({1, 1, 1}),
            pidSetCooling({1, 1, 1}),
            pwmOverride(0)
            {};

void TempControl::init() {

    HAL_SETUP_TEMP_ADC();

    eSum = 0;
    eAlt = 0;

    dTerm = 0;

    pid_output = 0;

#ifdef PID_DEBUG
    lastPidCompute = millis();
#endif

    for (uint8_t e=0; e<EXTRUDERS; e++)
        current_temperature[e] = HEATER_1_MINTEMP;

    curPidSet = &pidSetHeating;
}

void TempControl::setTemp(uint8_t heaterNum, int16_t temp, uint8_t pwmOv) {

    if (heaterNum == 0) {

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
        if (target_temperature[heaterNum-1] == 0) {

            eSum = 0;

            // Regeldifferenz, grösser 0 falls temperatur zu klein
            // int32_t e = target_temperature[0] - current_temperature[0];
            eAlt = target_temperature[0] - current_temperature[0];

            dTerm = 0;
        }

        target_temperature[heaterNum-1] = temp;
        pwmOverride = pwmOv;
    }
}

void TempControl::setPidSet(
        struct PidSet &pidSetOld, struct PidSet &pidSetNew,
        ScaledUInt32 &kiSwitch,
        int32_t e) {

    // eSumNew = kiSwitch * eSumOld; kiSwitch = KiOld/KiNew;

#ifdef PID_DEBUG
    // PID interval [ms]
    uint32_t ts = millis();
#endif

    //
    // P-term
    //

    int32_t pTerm = pidSetNew.Kp_scaled.mul31(e);

    eSum = kiSwitch.mul31(eSum);

    if (!antiWindupMode)
        eSum += e;

    //
    // I-term
    //
    //
    // float iTerm = Ki * pid_dt * eSum;
    //
    int32_t iTerm = pidSetNew.Ki_scaled.mul31(eSum);

    int32_t output = pTerm + iTerm;

    //
    // dTerm += (Kd * (e - eAlt)) / pid_dt;
    //
    int32_t delta = e - eAlt;

    dTerm += pidSetNew.Kd_scaled.mul31(delta);

    if (e >= 0 && (output < pwmOverride)) {

        // Temp to low, and pwmOverride (feed forward) is available.
        output = pwmOverride;
        antiWindupMode = true;
    }

    if (output >= PID_MAX) {
        pid_output = PID_MAX;
        antiWindupMode = true;
    }
    else if (output <= 0) {
        pid_output = 0;
        antiWindupMode = true;
    }
    else {
        pid_output = output;
        antiWindupMode = false;
    }

    HEATER_0_PIN :: write( pid_output );

    // Kept values of last pidrun
    eAlt = e;

#ifdef PID_DEBUG
    txBuffer.sendResponseStart(RespUnsolicitedMsg);
    txBuffer.sendResponseUint8(PidSwitch);
    txBuffer.sendResponseUInt32(ts - lastPidCompute);
    txBuffer.sendResponseInt32(pTerm);
    txBuffer.sendResponseInt32(iTerm);

    txBuffer.sendResponseInt32(dTerm);
    txBuffer.sendResponseUint8(pid_output);
    txBuffer.sendResponseInt16(e);
    txBuffer.sendResponseInt32(eSum);
    txBuffer.sendResponseEnd();

    lastPidCompute = ts;
#endif //PID_DEBUG

    curPidSet = &pidSetNew;
}

void TempControl::heater() {

    // Check if temperature is within the correct range
    if(current_temperature[0] < HEATER_1_MINTEMP) {
        LCDMSGKILL(RespMinTemp, 1, fromFWTemp(current_temperature[0]));
        txBuffer.sendSimpleResponse(RespKilled, RespMinTemp, 1);
        kill();
    }
    else {
        if(current_temperature[0] > HEATER_1_MAXTEMP) {
            LCDMSGKILL(RespMaxTemp, 1, fromFWTemp(current_temperature[0]));
            txBuffer.sendSimpleResponse(RespKilled, RespMaxTemp, 1);
            kill();
        }
        else {

            if (continousPID) {

                HEATER_0_PIN :: write(continousPID);
            }
            else {

                //
                // Reglergleichung
                //
                // y = Kp*e + Ki*Ta*esum + Kd/Ta*(e – ealt);

#ifdef PID_DEBUG
                // PID interval [s]
                uint32_t ts = millis();
#endif

                // Regeldifferenz, grösser 0 falls temperatur zu klein
                int32_t e = target_temperature[0] - current_temperature[0];

                if (e >= 0) {

                    if (curPidSet != &pidSetHeating) {
                        // temp to low, use fast pid
                        setPidSet(
                                pidSetCooling,
                                pidSetHeating,
                                kiSwitchToHeating,
                                e);
                        return;
                    }
                }
                else {

                    if ((curPidSet != &pidSetCooling) && (e & 0xfffffffe)) { // hysterese
                        // temp to high or equal set-temp, use slow pid
                        setPidSet(
                            pidSetHeating,
                            pidSetCooling,
                            kiSwitchToCooling,
                            e);
                        return;
                        }
                }

                //
                // pid_output = (uint8_t)(pTerm + iTerm + dTerm + 0.5);
                //
                // float pTerm = curPidSet->Kp * e;
                int32_t pTerm = curPidSet->Kp_scaled.mul31(e);

                //
                // eSum
                //
                if (!antiWindupMode) {
                    eSum += e;
                }

                //
                // float iTerm = curPidSet->Ki * pid_dt * eSum;
                //
                int32_t iTerm = curPidSet->Ki_scaled.mul31(eSum);

                int32_t output = pTerm + iTerm;

                //
                // dTerm += (curPidSet->Kd * (e - eAlt)) / pid_dt;
                //
                int32_t delta = e - eAlt;
                dTerm += curPidSet->Kd_scaled.mul31(delta);

                int32_t d = 0; // differiential part for this interval
                if (dTerm > 0) {
                    if (output < PID_MAX) {
                        d = min(dTerm, PID_MAX-output);
                    }
                }
                else if (dTerm < 0) {
                    if (output > 0) {
                        d = max(dTerm, output*-1);
                    }
                }

                output += d;
                dTerm -= d;

                if ( (e >= 0) && (output < pwmOverride)) {

                    // Temp to low, and pwmOverride (feed forward) is available.
                    output = pwmOverride;
                    antiWindupMode = true;
                }

                if (output >= PID_MAX) {
                    pid_output = PID_MAX;
                    antiWindupMode = true;
                }
                else if (output <= 0) {
                    pid_output = 0;
                    antiWindupMode = true;
                }
                else {
                    pid_output = output;
                    antiWindupMode = false;
                }

                HEATER_0_PIN :: write( pid_output );

#ifdef PID_DEBUG

                static uint8_t dbgcount=0;
                if ((dbgcount++ & 0x1f) == 0) {
                    txBuffer.sendResponseStart(RespUnsolicitedMsg);
                    txBuffer.sendResponseUint8(PidDebug);
                    txBuffer.sendResponseUInt32(ts - lastPidCompute);
                    txBuffer.sendResponseInt32(pTerm);
                    txBuffer.sendResponseInt32(iTerm);
                    txBuffer.sendResponseInt32(dTerm);
                    txBuffer.sendResponseUint8(pid_output);
                    txBuffer.sendResponseInt16(e);
                    txBuffer.sendResponseInt32(eSum);
                    txBuffer.sendResponseEnd();
                }

                lastPidCompute = ts;
#endif //PID_DEBUG

                // Kept values of last pidrun
                eAlt = e;
            }
        }
    }

    ////////////////////////////////
    // Handle heated bed 
    ////////////////////////////////

    // Check if temperature is within the correct range
    if(current_temperature_bed < HEATER_0_MINTEMP) {
        LCDMSGKILL(RespMinTemp, 0, fromFWTemp(current_temperature_bed));
        txBuffer.sendSimpleResponse(RespKilled, RespMinTemp, 0);
        kill();
    }
    else {
        if(current_temperature_bed > HEATER_0_MAXTEMP) {
            LCDMSGKILL(RespMaxTemp, 0, fromFWTemp(current_temperature_bed));
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

    // Watchdog timeout if temperature control is not run
    // regularily.
    WDT_RESET();
}

void TempControl::setTempPWM(uint8_t heaterNum, uint8_t pwmValue) {

    if (heaterNum == 1) 
        HEATER_0_PIN :: write(pwmValue);
#if defined(HEATER_1_PIN)
    else
        HEATER_1_PIN :: write(pwmValue);
#endif

    continousPID = pwmValue;
}

void TempControl::hotendOn(uint8_t heaterNum) {

    if (heaterNum == 1) 
        HEATER_0_PIN :: write(255);
#if defined(HEATER_1_PIN)
    else
        HEATER_1_PIN :: write(255);
#endif
}

void TempControl::setPIDValues(
    ScaledUInt32 &kp,
    ScaledUInt32 &ki, int32_t  maxEsum,
    ScaledUInt32 &kd,
    ScaledUInt32 &kpC,
    ScaledUInt32 &kiC, int32_t maxEsumC,
    ScaledUInt32 &kdC,
    ScaledUInt32 &toHeating,
    ScaledUInt32 &toCooling) {

    pidSetHeating.Kp_scaled = kp;

    pidSetHeating.Ki_scaled = ki;
    pidSetHeating.maxEsum = maxEsum;

    pidSetHeating.Kd_scaled = kd;

    pidSetCooling.Kp_scaled = kpC;

    pidSetCooling.Ki_scaled = kiC;
    pidSetCooling.maxEsum = maxEsumC;

    pidSetCooling.Kd_scaled = kdC;

    kiSwitchToHeating = toHeating;
    kiSwitchToCooling = toCooling;
}

TempControl tempControl;

