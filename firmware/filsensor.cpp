
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
static FilsensorReading filsensorReadings[256];
static uint8_t filsensorReadingIndex;
static uint8_t nReadings;
static uint8_t nAvg = 10; // Average 10 filament sensor readings if setNAvg() is not used.

#if defined(BournsEMS22AFS)

HAL_FS_SPI_SETTINGS;

FilamentSensorEMS22 filamentSensor;

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

float FilamentSensorEMS22::slippage() {

    if (nReadings == nAvg) {

        // sum up e-stepper and e-sensor steps/counts
        int16_t dssum = 0;
        int16_t dysum = 0;

        for (uint8_t i=filsensorReadingIndex - nAvg; i<filsensorReadingIndex; i++) {
            dssum += filsensorReadings[i].ds;
            dysum += filsensorReadings[i].dy;
        }

        if (dysum != 0)
            return abs((dssum * filSensorCalibration) / dysum);
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

    filsensorReadings[filsensorReadingIndex].timeStamp = millis();
    filsensorReadings[filsensorReadingIndex].ds = deltaStepperSteps;
    filsensorReadings[filsensorReadingIndex].dy = dy;
    
    filsensorReadingIndex++;

    if (nReadings < nAvg)
        nReadings ++;

    float s = slippage();

    if (feedrateLimiterEnabled && (s > 0.0)) {

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

    massert(nr <= nAvg);

    txBuffer.sendResponseStart(CmdGetFSReadings);

    uint8_t start = filsensorReadingIndex - nr;

    while (nr--) {

        txBuffer.sendResponseUInt32(filsensorReadings[start].timeStamp);
        txBuffer.sendResponseInt16(filsensorReadings[start].dy);
        start++;
    }

    txBuffer.sendResponseEnd();
}

#endif // #if defined(BournsEMS22AFS)

