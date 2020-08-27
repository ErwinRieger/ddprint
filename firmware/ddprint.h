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

#include "swapdev.h"

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

void setup();

#define N_HEATERS (EXTRUDERS + 1)

class Printer {

        bool eotReceived;
        bool homed;
        // We erase entire swapdev (sdcard) to speed up writes.
        bool swapErased;

        // To adjust temperature-niveau at runtime
        uint16_t increaseTemp[N_HEATERS];

        // Timeconstant hotend [ms]
        uint16_t Tu;

        uint8_t nGenericMessage;

        bool hotEndFanOn;

        // Timestamp power off (power button press)
        unsigned long powerOffTime;

        // Steps per mm
        uint16_t stepsPerMMX, stepsPerMMY, stepsPerMMZ;

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

        uint16_t getTu() { return Tu; }
        uint8_t getIncreaseTemp(uint8_t heater) { return increaseTemp[heater]; }
        void runHotEndFan();

        uint16_t getStepsPerMMX() { return stepsPerMMX; }
        uint16_t getStepsPerMMY() { return stepsPerMMY; }
        uint16_t getStepsPerMMZ() { return stepsPerMMZ; }

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
        void cmdSetPIDValues(float kp, float ki, float kd, uint16_t Tu);
        void cmdSetStepsPerMM(uint16_t spmmX, uint16_t spmmY, uint16_t spmmZ);
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
        void cmdReadGpio(uint8_t pinNumber);
        void cmdReadAnalogGpio(uint8_t pinNumber);
        void cmdSetGpio(uint8_t pinNumber, uint8_t value);
        void cmdSetPrinterName(char *name, uint8_t len);
        void cmdGetPrinterName();
#if defined(POWER_BUTTON)
        void checkPowerOff(unsigned long ms);
#endif
};

extern Printer printer;


//
// One-shot timer for
// * fan blip
//
class Timer {

    // Fan timer
    uint8_t fanSpeed;
    unsigned long fanEndTime;

    bool bootBootloaderRequest;
    bool resetRequest;

    public:
        Timer() {
            fanEndTime = 0;
            bootBootloaderRequest = false;
            resetRequest = false;
        }

        void run(unsigned long m);

        void startFanTimer(uint8_t speed, uint8_t blipTime) {

            fanSpeed = speed;
            fanEndTime = millis() + blipTime;
        }

        void endFanTimer() {

            fanEndTime = 0;
        }

        void startBootloaderTimer() {

            bootBootloaderRequest = true;
        }

        void startResetTimer() {

            resetRequest = true;
        }

};

extern Timer timer;



