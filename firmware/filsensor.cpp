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
        uint16_t       dt;
        int16_t       deltaStepper;
        int16_t        deltaSensor;
    } FilsensorReading;

    static CircularBuffer<FilsensorReading, uint8_t, 16> filsensorReadings;
#endif

FilamentSensorEMS22 filamentSensor;

#if defined(BournsEMS22AFS)

HAL_FS_SPI_SETTINGS;

FilamentSensorEMS22::FilamentSensorEMS22() {

    fsrMinSteps = 128;
                      
    feedrateLimiterEnabled = true;

    filSensorCalibration = { 1, 0 };

    slip128 = 128;
    slowDown = 128*8;
    limiting = false;

    lastASteps = 0;
    sensorCount = 0;
    startAt = 0;
}

void FilamentSensorEMS22::start() {
    slip128 = 128;
    slowDown = 128*8;
    limiting = false;
// frsMode = IDLE;

            // Messung starten
// frsMode = MEASURING;

            // CRITICAL_SECTION_START;
            lastASteps = current_pos_steps[E_AXIS];
            // CRITICAL_SECTION_END;

            HAL_FILSENSOR_BEGINTRANS();
            getDY(); // read distance delta from filament sensor
    
            sensorCount = 0;
            // measureTimer = 10; // 0.1s measuring interval
            startAt = millis();
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

    uint16_t dataWord = (((uint16_t)byte1) << 8) | byte2;

    // Check parity
    uint8_t p = __builtin_parity(dataWord);
    if (p != 0) {
        // Parity Error, skip this read
        return lastEncoderPos;
    }

    uint16_t pos = (((uint16_t)byte1) << 2) | (byte2 >> 6);

    // Check status bits
    if ((byte2 & 0x3E) != 0x20) {

        massert((byte2 & 0x3E) == 0);

        printer.sendGenericInt32( pos );
    }

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
    sensorCountAbs += dy;
    return dy;
}

// Called every 10mS
void FilamentSensorEMS22::run() {

    // if (idle()) {
#if 0
        // Keine messung aktiv
        if (stepBuffer.measureFlag) {

            // Messung starten
            frsMode = MEASURING;

            CRITICAL_SECTION_START;
            lastASteps = current_pos_steps[E_AXIS];
            CRITICAL_SECTION_END;

            HAL_FILSENSOR_BEGINTRANS();
            getDY(); // read distance delta from filament sensor
    
            sensorCount = 0;
            // measureTimer = 10; // 0.1s measuring interval
            startAt = millis();
        }
#endif
    // }
    // else {

        // Messung aktiv
        // if (stepBuffer.measureFlag) {

            // if (! --measureTimer) {

                HAL_FILSENSOR_BEGINTRANS();
                sensorCount += getDY(); // read distance delta from filament sensor

                // For accuracy, count a minimum of e-stepper and sensor steps, at least.
                // if (abs(deltaStepperSteps) >= fsrMinSteps) 
                if (abs(sensorCount) >= fsrMinSteps) {

                    CRITICAL_SECTION_START;
                    int32_t astep = current_pos_steps[E_AXIS];
                    CRITICAL_SECTION_END;

                    int32_t deltaStepperSteps = astep - lastASteps; // Requested extruded length

                    // ^if (abs(deltaStepperSteps) > (10*fsrMinSteps)) {
                        // ^lastASteps = astep;
                        // ^sensorCount = 0;
                        // ^startAt = millis();
                        // ^return;
                    // ^}

                    //
                    // Compute slippage
                    //
                    //        stepperSteps * calibration 
                    // slip = --------------------------
                    //                frsCount

                    if (sensorCount != 0) {

                        slip128 = ((deltaStepperSteps * filSensorCalibration.value) / sensorCount) >> filSensorCalibration.shift;

                        // deug
                        // if (slip128 < 0) {
                            // printer.sendGenericInt32( slip128 );
                        // }
                    }

                    // Ignore slip below 10%, 0.1*128 = 12.8, rounded: 13
                    uint16_t s = min( 
                            (uint16_t)max((int16_t)(slip128 - 13), (int16_t)128),
                            (uint16_t)(4*128) ); // max slowdown: 4 times

                    #if !defined(COLDEXTRUSION)
                    limiting = (s > 128) && feedrateLimiterEnabled;
                    #endif

                    if (limiting) {

                        // s in range 128 .. 4*128 = 512
                        // slowdown in range 1024 .. 4096
                        slowDown = s * 8;
                    }

                    if (filsensorReadings.full())
                        filsensorReadings.pop();

                    uint32_t t = millis();

                    FilsensorReading fsr = { 
                        (uint16_t)(t - startAt),
                        (int16_t)deltaStepperSteps,
                        sensorCount };
                    filsensorReadings.pushRef(fsr);
                
                    lastASteps = astep;
                    sensorCount = 0;
                    startAt = t;
                }

                // measureTimer = 10;
            // }
        // }
        // else {

            // // Stop measure
            // frsMode = IDLE;
        // }
    // }
}

void FilamentSensorEMS22::cmdGetFSReadings() {

    txBuffer.sendResponseStart(CmdGetFSReadings);

    uint8_t s = filsensorReadings.size();
    for (uint8_t i=0; i<s; i++) {
        FilsensorReading &fsr = filsensorReadings.pop();
        txBuffer.sendResponseUInt16(fsr.dt);
        txBuffer.sendResponseInt16(fsr.deltaStepper);
        txBuffer.sendResponseInt16(fsr.deltaSensor);
    }

    txBuffer.sendResponseEnd();
}

#endif // #if defined(BournsEMS22AFS)

