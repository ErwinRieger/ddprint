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
#define RAVGWINDOW 25

// To smooth tempsensor readings.
class RunningAvgF {

    float values[RAVGWINDOW];
    uint8_t i;
    uint8_t n;
    float avg;

  public:

    RunningAvgF(float startValue) {
        i = 0;
        avg = 0;

        for (uint8_t j=0; j<RAVGWINDOW; j++)
            values[j] = startValue;

        avg = startValue;
    }

    inline void addValue(float v) {

#if 0
        avg = (avg * (RAVGWINDOW-1) + v) /RAVGWINDOW;
#endif

        values[i++] = v;
        if (i == RAVGWINDOW)
            i = 0;

        float sum = 0;
        for (uint8_t j=0; j<RAVGWINDOW; j++)
            sum += values[j];
        avg = sum / RAVGWINDOW;
    }

    inline float value() { return avg; }
};





struct PidSet {
    // PID values from host printerprofile
    float Kp;
    float Ki;
    float Kd;
};

class TempControl: public Protothread
{
    RunningAvgF avgBedTemp;
    RunningAvgF avgHotendTemp;

    // PID values from host printerprofile
    // float Kp;
    // float Ki;
    // float Kd;
    struct PidSet *curPidSet;

    // Timestamp of last pid computation
    unsigned long lastPidCompute;

    float eSum; // For I-Part
    float eAlt; // For D-Part
    float dTerm;

    int32_t pid_output;

    bool pwmMode;

    // PWM value controlled by host firmware part.
    // Set to 0 to re-enable PID temperature control.
    // Maxtemp is still checked even if PID is disabled.
    uint8_t pwmValueOverride;

    float eSumLimit;

    bool antiWindupMode;

        struct PidSet pidSetHeating;
        struct PidSet pidSetCooling;

    public:

        // xxx todo setter
        uint8_t suggestPwm;

        TempControl();
        void init();
        virtual bool Run();
        void setTemp(uint8_t heater, uint16_t newTarget);

        void choosePidSet(float e, float pid_dt);
        void setPidSet(struct PidSet *pidSet, float e, float pid_dt);

        void heater();
        // Set heater PWM value directly for PID AutoTune
        // and filament measurements
        void setTempPWM(uint8_t heater, uint8_t pwmValue);
        void hotendOn(uint8_t heater);

        void setPIDValues(float kp, float ki, float kd, float kpC, float kiC, float kdC);
        uint8_t getPwmOutput() { 
            if (pwmMode)
                return pwmValueOverride;
            else
                return (uint8_t) pid_output;
        }
};

extern TempControl tempControl;

