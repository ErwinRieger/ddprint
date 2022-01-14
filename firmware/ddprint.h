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
#include "intmath.h"
#include "mdebug.h"

#include "swapdev.h"
#include "rxbuffer.h"

#include "hostsettings.h"

#if defined(DDSim)
    #include <unistd.h>
    #include <fcntl.h>
    #include <sys/stat.h>
    #include <assert.h>
#endif

// Number of entries in ExtrusionRateLimit table, this must match the value
// in the host (ddprintconstants.py).
#define NExtrusionLimit 100

extern const char *gitversion; // From gitversion.cpp

void kill();

void setup();

#define N_HEATERS (EXTRUDERS + 1)

class Printer {

        bool homed;

        // To adjust temperature-niveau at runtime
        int16_t increaseTemp[N_HEATERS];

        // Timeconstant hotend [ms]
        uint16_t Tu;

        uint8_t nGenericMessage;

        bool hotEndFanOn;

        // Timestamp power off (power button press)
        unsigned long powerOffTime;

        // Steps per mm
        uint16_t stepsPerMMX, stepsPerMMY, stepsPerMMZ;

        // Host settings: buildvolume, homing directions
        HostSettings hostSettings;

        bool limiting;
        uint32_t slowdown;

        // Mass storage erase: first block to erase
        uint32_t eraseStartBlock;
        // Mass storage erase: number of blocks to erase
        uint32_t blocksToErase;

    public:

        uint8_t minBuffer;

        uint16_t underTemp;
        uint16_t underGrip;

        // State enum
        enum {
            StateIdle,       // The state when printer is turned on.
            StateInit,       // Host part has initialized the printer and has
                             // done printer setup.
            StateErasing,    // Mass storage erase in progress
            StateStart,      // We are printing.
            } printerState;

        typedef enum {
            MoveTypeNone,
            MoveTypeHoming,
            MoveTypeNormal,
            } MoveType;

        MoveType moveType;

        int16_t bufferLow;

        Printer();
        void printerInit();
        void runErase();

        void sendGenericMessage(const char *s, uint8_t l);
        void sendGenericInt32(int32_t v);

        uint16_t getTu() { return Tu; }
        uint8_t getIncreaseTemp(uint8_t heater) { return increaseTemp[heater]; }
        void runHotEndFan();

        uint16_t getStepsPerMMX() { return stepsPerMMX; }
        uint16_t getStepsPerMMY() { return stepsPerMMY; }
        uint16_t getStepsPerMMZ() { return stepsPerMMZ; }

        HostSettings & getHostSettings() { return hostSettings; }

        uint32_t getSlowDown() { return slowdown; }
        uint32_t isLimiting() { return limiting; }

        void cmdMove(MoveType);
        void underrunError();

        void setPos( int32_t x, int32_t y, int32_t z);
        void cmdSetTargetTemp(uint8_t heater, int16_t temp, uint8_t pwmOverride=0);
        void cmdSetIncTemp(uint8_t heater, int16_t incTemp);
        void cmdGetFreeMem();
        void checkPrintFinished();
        void disableSteppers();
        void cmdDisableSteppers();
        void cmdDisableStepperIsr();
        void cmdGetHomed();
        void cmdGetEndstops();
        void cmdGetPos();
        void cmdSetPIDValues(
                ScaledUInt32 &kp,
                ScaledUInt32 &ki, int32_t maxEsum,
                ScaledUInt32 &kd,
                ScaledUInt32 &kpC,
                ScaledUInt32 &kiC, int32_t maxEsumC,
                ScaledUInt32 &kdC,
                ScaledUInt32 &kiSwitchToHeating,
                ScaledUInt32 &kiSwitchToCooling,
                uint16_t Tu);
        void cmdSetStepsPerMM(uint16_t spmmX, uint16_t spmmY, uint16_t spmmZ);
        void cmdSetHostSettings(HostSettings &hs);
        void cmdGetCardSize();
        void cmdErase(uint32_t nBlocks); // Erase sd-swap to speed up block writes.
        void cmdFanSpeed(uint8_t speed, uint8_t blipTime);
        void cmdSetFilSensorConfig(ScaledUInt16 & cal, uint16_t fsrMinSteps);
        void cmdSetFilSensorCal(ScaledUInt16 & cal);
        void cmdSetStepsPerMME(uint16_t steps);
        void cmdStopMove();
        void cmdGetTargetTemps();
        void cmdGetCurrentTemps();
        void cmdGetStatus();
        void cmdGetTaskStatus();
        void cmdGetIOStats();
        void cmdGetFilSensor();
        void cmdSetTempTable();
        void cmdReadGpio(uint8_t pinNumber);
        void cmdReadAnalogGpio(uint8_t pinNumber);
        void cmdSetGpio(uint8_t pinNumber, uint8_t value);
        void cmdSetPrinterName(char *name, uint8_t len);
        void cmdGetPrinterName();
#if defined(POWER_BUTTON)
        void checkPowerOff(unsigned long ms);
#endif
        void cmdDumpMassStorage(uint32_t block);
        bool stepsAvailable();
        void cmdSetBaudRate(uint32_t br);
        void cmdSetSlowDown(uint32_t scale) { 
            slowdown = scale;
            limiting = slowdown != 1024;
        }

        void cmdGetVersion();
};

extern Printer printer;


//
// One-shot timer for delayed things like
// * fan blip
// * reset request
// * boot into bootloader request
// * baudrate change command (auto-baudrate)
//
class Timer {

    // Fan timer
    uint8_t fanSpeed;
    unsigned long fanEndTime;

    bool bootBootloaderRequest;
    bool resetRequest;

    int32_t baudRate;
    uint32_t baudRateTime;

    public:
        Timer() {
            fanEndTime = 0;
            bootBootloaderRequest = false;
            resetRequest = false;
            baudRate = -1; 
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

        void baudRateTimer(uint32_t br) {

            baudRate = br;
            baudRateTime = millis() + 10;
        }

};

extern Timer timer;



