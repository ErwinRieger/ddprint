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

#include "ddcommands.h"
#include "ddlcd.h"

#define FilSensorDebug 1

// Circular buffer of last n filsensor measurements
#if defined(HASFILAMENTSENSOR) || defined(RUNFILAMENTSENSOR)
    typedef struct {
        uint16_t       deltaStepper;
        uint16_t       deltaSensor;
    } FilsensorReading;

    static CircularBuffer<FilsensorReading, uint8_t, 10> filsensorReadings;
#endif

FilamentSensorEMS22 filamentSensor;

#if defined(BournsEMS22AFS)

HAL_FS_SPI_SETTINGS;

FilamentSensorEMS22::FilamentSensorEMS22() {

    feedrateLimiterEnabled = true;

    filSensorCalibration = { 1, 0 };

    init();
}

void FilamentSensorEMS22::init() {

    slip32 = 32;
    slowDown = 32*32;
    limiting = false;
    frsMode = IDLE;
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

    // Check parity
    uint16_t dataWord = (((uint16_t)byte1) << 8) | byte2;
    uint8_t p = __builtin_parity(dataWord);
    massert(p == 0);

    return (((uint16_t)byte1) << 2) | (byte2 >> 6);
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
    sensorCountAbs += dy;
    return dy;
}

// Called every 10mS
void FilamentSensorEMS22::run() {

    if (idle()) {

        // Keine messung aktiv
        if (stepBuffer.linearFlag) {

            // Messung starten
            frsMode = MEASURING;

            CRITICAL_SECTION_START;
            lastASteps = current_pos_steps[E_AXIS];
            CRITICAL_SECTION_END;

            HAL_FILSENSOR_BEGINTRANS();
            getDY(); // read distance delta from filament sensor
    
            sensorCount = 0;
            measureTimer = 10; // 0.1s measuring interval
        }
    }
    else {

        // Messung aktiv
        if (stepBuffer.linearFlag) {

            if (! --measureTimer) {

                CRITICAL_SECTION_START;
                int32_t astep = current_pos_steps[E_AXIS];
                CRITICAL_SECTION_END;

                uint32_t deltaStepperSteps = astep - lastASteps; // Requested extruded length

                HAL_FILSENSOR_BEGINTRANS();
                sensorCount += getDY(); // read distance delta from filament sensor

                // For accuracy, count a minimum of e-stepper and sensor steps, at least.
                if (deltaStepperSteps >= fsrMinSteps) {

                    //
                    // Compute slippage
                    //
                    //        stepperSteps * calibration 
                    // slip = --------------------------
                    //                frsCount

                    if (sensorCount) {
                        slip32 = ((deltaStepperSteps * filSensorCalibration.value) / sensorCount) >> filSensorCalibration.shift;
                    }

                    // Ignore slip below 10%, 0.1*32 = 3.2, rounded: 3
                    uint16_t s = min( 
                            (uint16_t)max((int16_t)(slip32 - 3), (int16_t)32),
                            (uint16_t)(2*32) ); //  (uint16_t)(4*32) ); // max slowdown: pow(2) = 4

                    #if !defined(COLDEXTRUSION)
                    limiting = (s > 32) && feedrateLimiterEnabled;
                    #endif

                    if (limiting) {
                        // pow() is using floating point: slowDown = pow(s, 2);
                        slowDown = s * s;
                    }

                    if (filsensorReadings.full())
                        filsensorReadings.pop();

                    FilsensorReading fsr = { (uint16_t)deltaStepperSteps, sensorCount };
                    filsensorReadings.pushRef(fsr);
                
                    lastASteps = astep;
                    sensorCount = 0;
                }

                measureTimer = 10;
            }
        }
        else {

            // Stop measure
            frsMode = IDLE;
        }
    }
}

void FilamentSensorEMS22::cmdGetFSReadings(uint8_t nr) {

    txBuffer.sendResponseStart(CmdGetFSReadings);

    uint8_t s = filsensorReadings.size();
    uint8_t n = min(s, nr);

    for (uint8_t i=0; i<n; i++) {
        FilsensorReading &fsr = filsensorReadings.pop();
        txBuffer.sendResponseUInt16(fsr.deltaStepper);
        txBuffer.sendResponseUInt16(fsr.deltaSensor);
    }

    txBuffer.sendResponseEnd();
}

#endif // #if defined(BournsEMS22AFS)

