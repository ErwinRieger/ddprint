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
#include <limits.h>

#include "ddprint.h"
#include "filsensor.h"
#include "stepper.h"
#include "temperature.h"

#if defined(AVR)
#include "fastio.h"
#endif

#include "ddserial.h"
#include "ddcommands.h"
#include "ddlcd.h"

#define FilSensorDebug 1

// Circular buffer of last 256 filsensor measurements
#if defined(HASFILAMENTSENSOR) || defined(STARTFILAMENTSENSOR)

    #if defined(AVR)
        typedef union /*__packed */ { // packed needed?
        // typedef struct __packed {
        union {
            int16_t       dy   : 11;
            int16_t       ds   : 11;
            uint8_t     fill   : 2;
            } __attribute__((packed));
            uint8_t binary[3];
        } FilsensorReading;
        static_assert (sizeof(FilsensorReading)==3, "FilsensorReading not 3 bytes size.");
    #else
        typedef struct {
            // unsigned long timeStamp;
            int16_t       dy;
            int16_t       ds;
        } FilsensorReading;
    #endif

static FilsensorReading filsensorReadings[256];

#if defined(AVR)
    static_assert (sizeof(filsensorReadings)==(3*256), "FilsensorReading not 3 bytes size.");
#endif

static uint8_t filsensorReadingIndex = 0;
static uint8_t nReadings = 0;
static uint8_t nAvg = 10; // Average 10 filament sensor readings if setNAvg() is not used.
#endif

FilamentSensorEMS22 filamentSensor;

#if defined(BournsEMS22AFS)

HAL_FS_SPI_SETTINGS;

FilamentSensorEMS22::FilamentSensorEMS22() {

    feedrateLimiterEnabled = true;
    filSensorCalibration = 1.0;

    sensorCount = 0;

    init();
}

void FilamentSensorEMS22::init() {

    lastASteps = (int32_t)LONG_MAX; // marker for not set
    grip = 1.0;
}

uint16_t FilamentSensorEMS22::readEncoderPos() {

    //
    // Read rotary encoder pos (0...1023)
    //

    FILSENSNCS :: activate();

    delayMicroseconds(1); // Tclkfe

    //
    // Read two bytes from SPI/USART
    //

    uint8_t byte1 = HAL_FILSENSOR_READ();
    uint8_t byte2 = HAL_FILSENSOR_READ();

    FILSENSNCS :: deActivate();

    // Check status bits
    if ((byte2 & 0x3E) != 0x20) {
        massert(0);
    }
    uint16_t pos = (((uint16_t)byte1) << 2) | ((byte2 & 0xC0) >> 6);

    // Check parity
    uint16_t dataWord = (((uint16_t)byte1) << 8) | byte2;
    uint8_t p = __builtin_parity(dataWord);
    massert(p == 0);

    return pos;
}

int16_t FilamentSensorEMS22::getDY() {

    uint16_t pos = readEncoderPos();

    int16_t dy = pos - lastEncoderPos; 

    if (dy < -512) {
        // Überlauf in positiver richtung
        dy = (1024 - lastEncoderPos) + pos;
    }
    else if (dy > 512) {
        // Überlauf in negativer richtung
        dy = (pos - 1024) - lastEncoderPos;
    }

    lastEncoderPos = pos;
    sensorCount += dy;
    return dy;
}

// Return ratio of FRS counts to stepper stepps (taking filSensorCalibration into account).
// Returns 1.0 if average of FSR readings still no available,
// else slip value, range is [0.1...1.0], but values > 1.0 are possible to some extend (calibraition).
float FilamentSensorEMS22::slippage() {

    if (nReadings == nAvg) {

        // sum up e-stepper and e-sensor steps/counts
        int16_t dssum = 0;
        int16_t dysum = 0;

        for (uint8_t i=nAvg; i>0; i--) {
            dssum += filsensorReadings[(filsensorReadingIndex-i) & 0xff].ds;
            dysum += filsensorReadings[(filsensorReadingIndex-i) & 0xff].dy;
        }

#if 0
        //if (dysum != 0)
            //return abs((dssum * filSensorCalibration) / dysum);

        // Filter out jitter/noise of encoder if there is no movement
        // if (dysum != 0)
        if (abs(dysum) >= 50) { // xxx fixed value, should be derived from stepspermm*1mm or so
            float slip = (dssum * filSensorCalibration) / dysum;
            // if (slip > 0)
                return slip;
        }
#endif

        float slip = (dssum * filSensorCalibration) / dysum;
        // Filter out jitter/noise of encoder if there is no movement.
        // Filters out negative values and other
        // temporary glitches of the Feeder/FRS system (fast short retractions), too.
        if (slip >= 0.1) 
            return slip;
    }

    return 1.0;
}

void FilamentSensorEMS22::run() {

    CRITICAL_SECTION_START;
    int32_t astep = current_pos_steps[E_AXIS];
    CRITICAL_SECTION_END;

    HAL_FILSENSOR_BEGINTRANS();

    int16_t dy = getDY(); // read distance delta from filament sensor

    if (lastASteps == LONG_MAX) {

        // not initialized yet
        lastASteps = astep;
        lastSensorCount = sensorCount;
        return;
    }

    int16_t deltaStepperSteps = astep - lastASteps; // Requested extruded length

    // filsensorReadings[filsensorReadingIndex].timeStamp = millis();

    filsensorReadings[filsensorReadingIndex].ds = deltaStepperSteps;
    filsensorReadings[filsensorReadingIndex].dy = dy;
    
    filsensorReadingIndex++;

    if (nReadings < nAvg)
        nReadings ++;

    float s = slippage();

    if (feedrateLimiterEnabled) { //  && (s > 0.0)) {

        float allowedSlippage = s * 0.90; // allow for 10% slip before we slow down
        float g = max(1.0, pow(allowedSlippage - 1.0, 2)*25);
        grip = STD min((float)3.0, g);
    }

    lastASteps = astep;
    lastSensorCount = sensorCount;
}

void FilamentSensorEMS22::setNAvg(uint8_t n) {
    nAvg = n;
    nReadings = 0;
}

void FilamentSensorEMS22::cmdGetFSReadings(uint8_t nr) {

    txBuffer.sendResponseStart(CmdGetFSReadings);

    uint8_t n = min(nReadings, nr);

    for (uint8_t i=n; i>0; i--) {
        // ramps txBuffer.sendResponseUInt32(filsensorReadings[(filsensorReadingIndex-i) & 0xff].timeStamp);
        // txBuffer.sendResponseUint8(filsensorReadingIndex-i);
        txBuffer.sendResponseInt16(filsensorReadings[(filsensorReadingIndex-i) & 0xff].dy);
    }

    txBuffer.sendResponseEnd();
}

#endif // #if defined(BournsEMS22AFS)

