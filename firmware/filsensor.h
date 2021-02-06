
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


/*
 * Parts of this code come from https://github.com/mrjohnk
 */

#pragma once

typedef struct {
    unsigned long timeStamp;
    int16_t       dy;
    int16_t       ds;
} FilsensorReading;

#if defined(BournsEMS22AFS)

#define VAR_FILSENSOR_GRIP (filamentSensor.getGrip())

/*
 * Inteface to a Bourns EMS22 incremental sensor.
 */
class FilamentSensorEMS22 {

        int32_t lastASteps;
        int32_t lastSensorCount;

        int16_t rest = 0;
        int16_t lastEncoderPos = 0;

        uint8_t readLoc(uint8_t addr);

        void writeLoc(uint8_t addr, uint8_t value);
        uint8_t pullbyte();
        bool feedrateLimiterEnabled;

        int32_t sensorCount;

        // Factor to slow down movement because feeder slippage is greater than 10%.
        float grip;

        // Ratio between measured filament sensor counts and the 
        // extruder stepper motor steps.
        // For example if steps per mm of the extruder stepper is 141 and
        // the resolution of the filament sensor is 102.8 counts/mm, then:
        // filSensorCalibration = 102.8 Counts / 141 Steps = 0.73.
        float filSensorCalibration;

        // uint16_t axis_steps_per_mm_e;

        uint16_t readEncoderPos();

        int16_t getDY();

    public:

        FilamentSensorEMS22();
        void init();
        void reset();

        // The polling method
        void run();
        void selfTest();

        void enableFeedrateLimiter(bool flag) { feedrateLimiterEnabled = flag; }
        void setFilSensorCalibration(float fc) { filSensorCalibration = fc; }
        // void setStepsPerMME(uint16_t steps) {
            // axis_steps_per_mm_e = steps;
        // }

        float slippage();
        int32_t getSensorCount() { return sensorCount; }
        float getGrip() { return grip; }

        void setNAvg(uint8_t n);
        void cmdGetFSReadings(uint8_t nr);
};

#else // #if defined(BournsEMS22AFS)
    #define VAR_FILSENSOR_GRIP (1.0)

class FilamentSensorEMS22 {
    public:
        FilamentSensorEMS22() { };
        void setNAvg(uint8_t n) { };
        void cmdGetFSReadings(uint8_t nr) {
            txBuffer.sendResponseStart(CmdGetFSReadings);
            uint8_t n = min(10, nr);

            for (uint8_t i=n; i>0; i--) {
                txBuffer.sendResponseUInt32(0);
                txBuffer.sendResponseInt16(0);
            }

            txBuffer.sendResponseEnd();
        }
};

#endif

extern FilamentSensorEMS22 filamentSensor;

