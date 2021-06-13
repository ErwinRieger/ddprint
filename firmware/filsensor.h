
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

#if defined(BournsEMS22AFS)

/*
 * Inteface to a Bourns EMS22 incremental sensor.
 */
class FilamentSensorEMS22 {

        int32_t lastASteps;
        uint16_t sensorCount;

        int16_t lastEncoderPos = 0;

        bool feedrateLimiterEnabled;
        bool limiting;

        // Factor to slow down movement because feeder slippage is greater than 10%.
        uint16_t slowDown;

        int32_t sensorCountAbs;
        int16_t slip32;

        // Ratio between measured filament sensor counts and the 
        // extruder stepper motor steps.
        // For example if steps per mm of the extruder stepper is 141 and
        // the resolution of the filament sensor is 102.8 counts/mm, then:
        // filSensorCalibration = 102.8 Counts / 141 Steps = 0.73.
        ScaledUInt16 filSensorCalibration;

        uint16_t readEncoderPos();

        int16_t getDY();

        // Mode we are in. If *IDLE*, a measurement can be started by
        // the stepper-isr. Mode is *MEASURING* when a measurement is
        // currently in progress.
        enum {
            IDLE,
            MEASURING
        } frsMode;

        uint8_t measureTimer;

        uint16_t fsrMinSteps;

    public:

        FilamentSensorEMS22();

        void init();
        void reset();

        // The polling method
        void run();

        bool idle() { return frsMode == IDLE; }

        void enableFeedrateLimiter(bool flag) { feedrateLimiterEnabled = flag; }
        void setFilSensorConfig(ScaledUInt16 & fc, uint16_t minsteps) {
            filSensorCalibration = fc;
            fsrMinSteps = minsteps;
        }

        void setCalibration(ScaledUInt16 & fc) {
            filSensorCalibration = fc;
        }

        int32_t getSensorCount() { return sensorCountAbs; }
        bool isLimiting() { return limiting; }
        uint16_t getSlowDown() { return slowDown; }
        int16_t getSlip32() { return slip32; }

        void cmdGetFSReadings(uint8_t nr);
};

#else // #if defined(BournsEMS22AFS)

// Mockup for testing if no FRS hardware available.
class FilamentSensorEMS22 {
    public:
        FilamentSensorEMS22() { };
        void enableFeedrateLimiter(bool /* flag */) { }
        bool isLimiting() { return false; }
        uint16_t getSlowDown() { return 0; }
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

