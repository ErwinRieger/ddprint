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
#include "Protothread.h"

class TempControl: public Protothread
{
    //
    // ADC has 10 bits
    //
    float raw_temp_0_value; // sum of OVERSAMPLENR ADC values
    float raw_temp_bed_value; // sum of OVERSAMPLENR ADC values

    // PID values from eeprom
    float Kp;
    float Ki;
    float Kd;

    // Timestamp of last pid computation
    unsigned long lastPidCompute;

    float eSum; // For I-Part
    float eAlt; // For D-Part

    // Running sum of pwm output values to handle clipping
    float pwmSum;
    int32_t cobias;
    int32_t pid_output;

    // PWM value dictated by host firmware part. Normal PID
    // temperature control is bypassed if this value is set.
    // Set to 0 to re-enable PID temperature control.
    // Maxtemp is still checked even if PID is disabled.
    uint8_t pwmValueOverride;

    public:
        TempControl():
            raw_temp_0_value(0),
            raw_temp_bed_value(0),
            Kp(1.0),
            Ki(0.1),
            Kd(1.0),
            pwmValueOverride(0) {};
        void init();
        virtual bool Run();
        void setTemp(uint8_t heater, uint16_t newTarget);
        void heater();
        // Set heater PWM value directly for PID AutoTune
        // and filament measurements
        void setTempPWM(uint8_t heater, uint8_t pwmValue);

#if defined(PIDAutoTune)
        // Set heater PWM value directly for PID AutoTune
        void setHeaterY(uint8_t heater, uint8_t pwmValue);
#endif
        void setPIDValues(float kp, float ki, float kd) {
            Kp = kp;
            Ki = ki;
            Kd = kd;
        }
        uint8_t getPwmOutput() { return (pwmValueOverride) ? pwmValueOverride : (uint8_t)max(pid_output, 0); }
};

extern TempControl tempControl;

