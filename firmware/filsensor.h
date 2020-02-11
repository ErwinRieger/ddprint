
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

#if defined(PMWFS) || defined(BournsEMS22AFS)
    #define HASFILAMENTSENSOR
#endif

typedef struct {
    unsigned long timeStamp;
    int16_t       dy;
    int16_t       ds;
} FilsensorReading;
extern FilsensorReading filsensorReadings[];
extern uint8_t filsensorReadingIndex;
extern uint8_t nReadings;
extern uint8_t nAvg;

extern void setNAvg(uint8_t nAvg);

#if defined(PMWFS)

#define VAR_FILSENSOR_GRIP (filamentSensor.grip)

//
// PMW3360 stuff
//
// PMW3360 Registers
// Registers
#define Product_ID  0x00
#define Revision_ID 0x01
#define Motion  0x02
#define Delta_X_L 0x03
#define Delta_X_H 0x04
#define Delta_Y_L 0x05
#define Delta_Y_H 0x06
#define SQUAL 0x07
#define Raw_Data_Sum  0x08
#define Maximum_Raw_data  0x09
#define Minimum_Raw_data  0x0A
#define Shutter_Lower 0x0B
#define Shutter_Upper 0x0C
#define Control 0x0D
#define Config1 0x0F
#define Config2 0x10
#define Angle_Tune  0x11
#define Frame_Capture 0x12
#define SROM_Enable 0x13
#define Run_Downshift 0x14
#define Rest1_Rate_Lower  0x15
#define Rest1_Rate_Upper  0x16
#define Rest1_Downshift 0x17
#define Rest2_Rate_Lower  0x18
#define Rest2_Rate_Upper  0x19
#define Rest2_Downshift 0x1A
#define Rest3_Rate_Lower  0x1B
#define Rest3_Rate_Upper  0x1C
#define Observation 0x24
#define Data_Out_Lower  0x25
#define Data_Out_Upper  0x26
#define Raw_Data_Dump 0x29
#define SROM_ID 0x2A
#define Min_SQ_Run  0x2B
#define Raw_Data_Threshold  0x2C
#define Config5 0x2F
#define Power_Up_Reset  0x3A
#define Shutdown  0x3B
#define Inverse_Product_ID  0x3F
#define LiftCutoff_Tune3  0x41
#define Angle_Snap  0x42
#define LiftCutoff_Tune1  0x4A
#define Motion_Burst  0x50
#define LiftCutoff_Tune_Timeout 0x58
#define LiftCutoff_Tune_Min_Length  0x5A
#define SROM_Load_Burst 0x62
#define Lift_Config 0x63
#define Raw_Data_Burst  0x64
#define LiftCutoff_Tune2  0x65

/*
 * Inteface to a PMW3360 'Mousesensor'
 */
class FilamentSensorPMW3360 {

        int32_t lastASteps;
        int32_t lastSensorCount;

        int16_t rest = 0;
        int16_t lastEncoderPos;

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
        // the resolution of the filament sensor is 5000 cpi, then:
        // filSensorCalibration = 197 Counts / 141 Steps = 1.4
        float filSensorCalibration;

        // uint16_t axis_steps_per_mm_e;

        int16_t getDY();

    public:

        FilamentSensorPMW3360();
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
};

extern FilamentSensorPMW3360 filamentSensor;

#elif defined(BournsEMS22AFS)

#define VAR_FILSENSOR_GRIP (filamentSensor.getGrip())

/*
 * Inteface to a PMW3360 'Mousesensor'
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
};

extern FilamentSensorEMS22 filamentSensor;

#else // #if defined(BournsEMS22AFS)
    #define VAR_FILSENSOR_GRIP (1.0)
#endif

