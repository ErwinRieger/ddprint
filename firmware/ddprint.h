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

#include <stdio.h>
#include <math.h>

#include <stdlib.h>

#include "config.h"
#include "Configuration.h"
#include "pins.h"
#include "mdebug.h"
#include "fastio.h"
#include "swapdev.h"
#include "stepper.h"

#if defined(DDSim)
    #include <unistd.h>
    #include <fcntl.h>
    #include <sys/stat.h>
    #include <assert.h>
#endif

// Number of entries in ExtrusionRateLimit table, this must match the value
// in the host (ddprintconstants.py).
#define NExtrusionLimit 100
extern uint16_t tempExtrusionRateTable[];
extern uint16_t extrusionLimitBaseTemp;

extern uint8_t errorFlags;

void kill();
// void kill(const char* msg);
// void killPGM(const char* msg);

// bool IsStopped();
// void Stop(uint8_t reasonNr);
extern void watchdog_reset();
void setup();

#define N_HEATERS (EXTRUDERS + 1)

class Printer {

        bool eotReceived;
        bool homed;
        // We erase entire swapdev (sdcard) to speed up writes.
        bool swapErased;

        // To adjust temperature-niveau at runtime
        uint16_t increaseTemp[N_HEATERS];

        // Idle pmw value if in *pmw temperature mode*
        uint8_t p0pwm;

        // Timeconstant hotend [ms]
        uint16_t Tu;

        uint8_t nGenericMessage;

        bool hotEndFanOn;

    public:

        // State enum
        enum {
            StateIdle,       // The state when printer is turned on.
            StateInit,       // Host part has initialized the printer and has
                             // done printer setup.
            StateStart,      // We are printing.
            } printerState;

        typedef enum {
            MoveTypeNone,
            MoveTypeHoming,
            MoveTypeNormal,
            } MoveType;

        MoveType moveType;

        // long z_max_pos_steps;
        int16_t bufferLow;

        Printer();
        void printerInit();
        void sendGenericMessage(const char *s, uint8_t l);

        uint8_t getP0pwm() { return p0pwm; }
        uint16_t getTu() { return Tu; }
        uint8_t getIncreaseTemp(uint8_t heater) { return increaseTemp[heater]; }
        void runHotEndFan();

        void cmdMove(MoveType);
        void cmdEot();
        void setHomePos( int32_t x, int32_t y, int32_t z);
        void cmdSetTargetTemp(uint8_t heater, uint16_t temp);
        void cmdSetIncTemp(uint8_t heater, int16_t incTemp);
        void cmdGetFreeMem();
        void cmdGetFSReadings(uint8_t nReadings);
        void checkMoveFinished();
        void disableSteppers();
        void cmdDisableSteppers();
        void cmdDisableStepperIsr();
        void cmdGetDirBits();
        void cmdGetHomed();
        void cmdGetEndstops();
        void cmdGetPos();
        void cmdSetP0pwm(uint8_t pwm);
        void cmdSetPIDValues(float kp, float ki, float kd, uint16_t Tu);
        void cmdFanSpeed(uint8_t speed, uint8_t blipTime);
        void cmdContinuousE(uint16_t timerValue);
        void cmdSetFilSensorCal(float cal);
        void cmdSetStepsPerMME(uint16_t steps);
        void cmdStopMove();
        void cmdGetTargetTemps();
        void cmdGetCurrentTemps();
        void cmdGetStatus();
        void cmdGetFilSensor();
        void cmdGetTempTable();
        void cmdSetTempTable();
};

extern Printer printer;


class FillBufferTask : public Protothread {

        uint16_t flags;
        uint8_t timerLoop;
        uint16_t lastTimer;

        uint16_t nAccel;
        uint8_t leadAxis;
        uint16_t tLin;
        uint16_t nDecel;
        int32_t absSteps[5];

        // Bresenham factors
        int32_t d_axis[5];
        int32_t d1_axis[5];
        int32_t d2_axis[5];

        int32_t deltaLead, step;

        // Hotend target temp for CmdSyncTargetTemp
        // uint8_t targetHeater;
        // uint16_t targetTemp;

        // Hotend target pwm for CmdSyncHotendPWM
        // uint8_t heaterPWM;
        // unsigned long pulseEnd;

        // Number of 25 mS nop segments for G4/dwell
        uint16_t nDwell;

        bool cmdSync;

#if defined(USEExtrusionRateTable)
        // Scaling factor for timerValues to implement temperature speed limit 
        float timerScale;
#endif

        stepData sd;

    public:
        FillBufferTask() {
            sd.dirBits = 0;
            cmdSync = false;
            pulseEnd = 0;
        }

        // xxxx getter/setter
        uint8_t targetHeater;
        uint32_t pulseTime;
        uint32_t pulsePause;
        unsigned long pulseEnd;
        uint16_t targetTemp;

        bool Run();

        void sync() {
            cmdSync = false;
        }

        bool synced() {
            return cmdSync;
        }

        // Flush/init swap, swapreader, fillbuffer task and stepbuffer
        void flush();

        //
        // Compute stepper bits, bresenham
        //
        FWINLINE void computeStepBits() {

            sd.stepBits = 1 << leadAxis;

            for (uint8_t i=0; i<5; i++) {

                if (i == leadAxis)
                    continue;

                if (d_axis[i] < 0) {
                    //  d_axis[a] = d + 2 * abs_displacement_vector_steps[a]
                    d_axis[i] += d1_axis[i];
                }
                else {
                    //  d_axis[a] = d + 2 * (abs_displacement_vector_steps[a] - deltaLead)
                    d_axis[i] += d2_axis[i];
                    sd.stepBits |= 1 << i;
                }
            }
        }

};

extern FillBufferTask fillBufferTask;

//
// One-shot timer for
// * fan blip
// * hotend pulsing
//
class Timer {

    // Fan timer
    uint8_t fanSpeed;
    unsigned long fanEndTime;

    // Hotend pulsing timer
    uint8_t targetHeater;
    uint16_t targetTemp;
    uint32_t pulseTime;
    uint32_t pauseTime;
    unsigned long hotendPwmEndTime;
    uint8_t hotendPwmState;

    public:
        Timer() {
            fanEndTime = 0;
            hotendPwmEndTime = 0;
        }

        void run(unsigned long m);

        void startFanTimer(uint8_t speed, uint8_t blipTime) {

            fanSpeed = speed;
            fanEndTime = millis() + blipTime;
        }

        void endFanTimer() {

            fanEndTime = 0;
        }

        void startHotenPulseTimer(uint8_t th, uint32_t pulseT, uint32_t pauseT, uint16_t tt) {

            targetHeater = th;
            pauseTime = pauseT;
            targetTemp = tt;
            hotendPwmState = 1;

            uint32_t thisPulse;
            uint16_t tu = printer.getTu();

            // Carefull for interger underflow
            if (tu <= pulseT) {
                thisPulse = tu;
                pulseTime = pulseT - tu; 
            }
            else {
                thisPulse = pulseT;
                pulseTime = 0;
            }

            // Time when to end this pulse:
            hotendPwmEndTime = millis() + thisPulse;

#if 0
            // Hotend full throttle, pwm value is reset with next heater run (every 100mS)
            tempControl.hotendOn(targetHeater);
            // TODO 260 hardcoded max hotend temp, this is the max hotend temp value
            // from material profile
            tempControl.setTemp(targetHeater, 260 - printer.getIncreaseTemp(targetHeater));
#endif
        }

        void endHotenPulseTimer() {

            hotendPwmEndTime = 0;
        }
};

extern Timer timer;



