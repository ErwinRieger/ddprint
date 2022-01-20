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

#pragma once

#include "config.h"
#include "Configuration.h"
#include "intmath.h"
#include "mdebug.h"

#include "protothreads-cpp/Protothread.h"

// Window size running average of temperature.
// Poll rate is 10ms (see ddprint.cpp).
constexpr const uint8_t RAVGWINDOW = 32; // power of 2
constexpr const uint8_t RAVGSHIFT = 5;   // divisor 32

constexpr const uint8_t TEMPSHIFT = 4; // Temperature stored in 1/16th degree

FWINLINE int16_t toFWTemp(int16_t t) { return t << TEMPSHIFT; }

template <class T>
T toFWTemp(T) = delete;

FWINLINE int16_t fromFWTemp(int16_t t) { return t >> TEMPSHIFT; }

template <class T>
T fromFWTemp(T) = delete;

// To smooth tempsensor readings.
class TempAvg {

    int16_t values[RAVGWINDOW];
    uint8_t i;

  public:

    TempAvg(int16_t startvalue) {
        i = 0;

        uint8_t j = RAVGWINDOW;
        do {
            values[j-1] = startvalue;
        } while (--j); 
    }

    int32_t addValue(int16_t v) {

        values[(i++) & 0x1f] = v;

        int32_t sum = 0;
        uint8_t j = RAVGWINDOW;
        do {
            sum += values[j-1];
        } while (--j); 

       // Note: shift for negative values not defined,
       // but temp always >= 0
       return sum >> RAVGSHIFT; // sum/32
    }
};

// Group PID values from host printerprofile
struct PidSet {

    ScaledUInt32 Kp_scaled;

    ScaledUInt32 Ki_scaled;

    int32_t  maxEsum;

    ScaledUInt32 Kd_scaled;
};

class TempControl: public Protothread
{
        TempAvg avgBedTemp;
        TempAvg avgHotendTemp;

        // Currently active PID values from host printerprofile
        struct PidSet *curPidSet;

#ifdef PID_DEBUG
        // Timestamp of last pid computation
        uint32_t lastPidCompute;
#endif

        // int32_t eSum_m16;  // For I-Term
        int32_t eSum;  // For I-Term
        int32_t eAlt;  // For I-Term

        // int32_t pTermOld;  // Previous P-Term setPidSet()

        // int16_t eAlt_m16;  // For D-Term
        // float dTerm; // To distribute D-Term
        int32_t dTerm; // To distribute D-Term

        uint8_t pid_output;

        // PWM value controlled by host firmware part.
        // Set to 0 to re-enable PID temperature control.
        // Maxtemp is still checked even if PID is disabled.
        uint8_t continousPID;

        bool antiWindupMode;

        struct PidSet pidSetHeating;
        struct PidSet pidSetCooling;

        ScaledUInt32 kiSwitchToHeating;
        ScaledUInt32 kiSwitchToCooling;

        // This is a *suggested pwm value* to keep the
        // current hotend temperature. If hotend temperature
        // is to low, we output this value to the temp-pwm,
        // overriding the PID.
        uint8_t pwmOverride;

    public:

        TempControl();
        void init();
        bool Run();
        void setTemp(uint8_t heater, int16_t newTarget, uint8_t pwmOverride);

        void setPidSet(
                struct PidSet &pidSetOld, struct PidSet &pidSetNew,
                ScaledUInt32 &kiSwitch,
                int32_t e);

        void heater();
        // Set heater PWM value directly for PID AutoTune
        // and hotend/filament measurements
        void setTempPWM(uint8_t heater, uint8_t pwmValue);
        void hotendOn(uint8_t heater);

        void setPIDValues(
                ScaledUInt32 &kp,
                ScaledUInt32 &ki,
                int32_t   maxEsum,
                ScaledUInt32 &kd,
                ScaledUInt32 &kpC,
                ScaledUInt32 &kiC,
                int32_t  maxEsumC,
                ScaledUInt32 &kdC,
                ScaledUInt32 &kiSwitchToHeating,
                ScaledUInt32 &kiSwitchToCooling);

        uint8_t getPwmOutput() { 
            if (continousPID)
                return continousPID;
            else
                return (uint8_t) pid_output;
        }
};

extern TempControl tempControl;

