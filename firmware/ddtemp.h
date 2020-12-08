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

#pragma once

#include "config.h"
#include "Configuration.h"
#include "Protothread.h"
#include "mdebug.h"

// Window size running average of temperature.
// Poll rate is 10ms (see ddprint.cpp).
#define RAVGWINDOW 100

// To smooth tempsensor readings.
class RunningAvgF {

    // float values; // [RAVGWINDOW];
    // uint8_t i;
    // uint8_t n;
    float avg;

  public:

    RunningAvgF(float startValue) {
        // i = n = 0;
        // avg = 0;

        // for (uint8_t j=0; j<RAVGWINDOW; j++)
            // values[j] = startValue;
            avg = startValue;
    }

    inline void addValue(float v) {

        avg = (avg * (RAVGWINDOW-1) + v) /RAVGWINDOW;

#if 0
        values[i++] = v;
        if (i == RAVGWINDOW)
            i = 0;
        if (n < RAVGWINDOW)
            n++;

        float sum = 0;
        for (uint8_t j=0; j<n; j++)
            sum += values[j];
        avg = sum / n;
#endif

    }

    inline float value() { return avg; }
};

class TempControl: public Protothread
{
    //
    // ADC has 10 bits
    //
    // float raw_temp_0_value; // sum of OVERSAMPLENR ADC values
    // float raw_temp_bed_value; // sum of OVERSAMPLENR ADC values
    RunningAvgF avgBedTemp;
    RunningAvgF avgHotendTemp;

    // PID values from host printerprofile
    float Kp;
    float Ki;
    float Kd;

    // Timestamp of last pid computation
    unsigned long lastPidCompute;

    float eSum; // For I-Part
    float eAlt; // For D-Part

    // Running sum of pwm output values to handle clipping
    float pwmSum;
    // int32_t cobias;
    int32_t pid_output;

    bool pwmMode;

    // PWM value controlled by host firmware part.
    // Set to 0 to re-enable PID temperature control.
    // Maxtemp is still checked even if PID is disabled.
    uint8_t pwmValueOverride;

    float eSumLimit;

    bool antiWindupMode;

    public:

        // xxx todo setter
        uint8_t suggestPwm;
        bool suggestUsed;

        TempControl():
            avgBedTemp(HEATER_0_MINTEMP),
            avgHotendTemp(HEATER_1_MINTEMP),
            Kp(1.0),
            Ki(0.1),
            Kd(1.0),
            pwmValueOverride(0),
            antiWindupMode(false),
            suggestPwm(0),
            suggestUsed(false)
            {};
        void init();
        virtual bool Run();
        void setTemp(uint8_t heater, uint16_t newTarget);
        void heater();
        // Set heater PWM value directly for PID AutoTune
        // and filament measurements
        void setTempPWM(uint8_t heater, uint8_t pwmValue);
        void hotendOn(uint8_t heater);

        void setPIDValues(float kp, float ki, float kd) {
            Kp = kp;
            Ki = ki;
            Kd = kd;

            // isum begrenzen auf max. 255 output
            //
            // Ki * pid_dt * eSum = iTerm < 255
            // eSummax = 255 / (Ki * pid_dt)
            //
            eSumLimit = 255.0 / ((ki * TIMER100MS) / 1000.0);
        }
        uint8_t getPwmOutput() { 
            if (pwmMode)
                return pwmValueOverride;
            else
                return (uint8_t) pid_output;
        }
};

extern TempControl tempControl;

