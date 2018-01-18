
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
 * Parts of this code come from https://github.com/mrjohnk/ADNS-9800
 */

#pragma once

#if defined(ADNSFS) || defined(BournsEMS22AFS) || defined(PMWFS)
    #define HASFILAMENTSENSOR
#endif

// #if 0
// Weight for exponential filter of e-speed [percent]
#define ESpeedWeight 0.33

class SpeedExpoFilter {

    // float weight;
    float current;
    // int16_t current;

  public:

    SpeedExpoFilter(/* float w*/) {
        // weight = w;
        current = 0;
    }

    void /*int16_t*/ addValue(float v) {

        current = (ESpeedWeight * v + (1.0 - ESpeedWeight) * current);
        // return current;
    }

    float value() { return current; }

    void reset() { current = 0; }
};
// #endif

// Window size running average filament speed
#define RAVGWINDOW 3

/*
class RunninAvg {

    int16_t values[RAVGWINDOW];
    uint8_t i;
    uint8_t n;
    int16_t avg;

  public:

    RunninAvg() {
        i = n = 0;
        avg = 0;
    }

    inline void addValue(int16_t v) {

        values[i++] = v;
        if (i == RAVGWINDOW)
            i = 0;
        if (n < RAVGWINDOW)
            n++;

        avg = 0;
        for (uint8_t j=0; j<n; j++)
            avg += values[j];
        avg /= n;
    }

    inline int16_t value() { return avg; }

    void reset(int16_t av = 0) { n = 0; avg = av;}
};
*/

class RunningAvgF {

    float values[RAVGWINDOW];
    uint8_t i;
    uint8_t n;
    float avg;

  public:

    RunningAvgF() {
        i = n = 0;
        avg = 0;
    }

    inline void addValue(float v) {

        values[i++] = v;
        if (i == RAVGWINDOW)
            i = 0;
        if (n < RAVGWINDOW)
            n++;

        avg = 0;
        for (uint8_t j=0; j<n; j++)
            avg += values[j];
        avg /= n;
    }

    inline float value() { return avg; }

    void reset(float av = 0) { n = 0; avg = av;}
};

#if defined(PMWFS)

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

        uint8_t readLoc(uint8_t addr);
        void writeLoc(uint8_t addr, uint8_t value);
        uint8_t pullbyte();
        int16_t getDY();

        bool feedrateLimiterEnabled;

        // Ratio between measured filament sensor couns and the 
        // extruder stepper motor steps.
        // For example if steps per mm of the extruder stepper is 141 and
        // the resolution of the filament sensor is 5000 cpi, then:
        // filSensorCalibration = 197 Counts / 141 Steps = 1.4
        float filSensorCalibration;

    public:

        // Ratio of target e-steps and filament sensor steps, this is a 
        // measure of the *feeder slippage*.
        RunningAvgF slippage;

        // Factor to slow down movement because feeder slippage is greater than 10%.
        float grip;

        FilamentSensorPMW3360();
        void init();
        void reset();

        // The polling method
        void run();
        void selfTest();

        void enableFeedrateLimiter(bool flag) { feedrateLimiterEnabled = flag; }
        void setFilSensorCalibration(float fc) { filSensorCalibration = fc; }
};

extern FilamentSensorPMW3360 filamentSensor;

#endif // #if defined(PMWFS)

#if 0
#if defined(ADNSFS)

    // See also: calibrateFilSensor function of host part.
    // #define FS_STEPS_PER_MM 265
    // 8200.0/25.4 = 322.8
    #define FS_STEPS_PER_MM 323
#endif
#endif

#if defined(BournsEMS22AFS)
    #define FS_STEPS_PER_MM (1024 / (5.5 * M_PI))
#endif

#if defined(ADNSFS)
//
// ADNS9800 stuff
//
// ADNS9800 Registers
#define REG_Product_ID                           0x00
#define REG_Revision_ID                          0x01
#define REG_Motion                               0x02
#define REG_Delta_X_L                            0x03
#define REG_Delta_X_H                            0x04
#define REG_Delta_Y_L                            0x05
#define REG_Delta_Y_H                            0x06
#define REG_SQUAL                                0x07
#define REG_Pixel_Sum                            0x08
#define REG_Maximum_Pixel                        0x09
#define REG_Minimum_Pixel                        0x0a
#define REG_Shutter_Lower                        0x0b
#define REG_Shutter_Upper                        0x0c
#define REG_Frame_Period_Lower                   0x0d
#define REG_Frame_Period_Upper                   0x0e
#define REG_Configuration_I                      0x0f
#define REG_Configuration_II                     0x10
#define REG_Frame_Capture                        0x12
#define REG_SROM_Enable                          0x13
#define REG_Run_Downshift                        0x14
#define REG_Rest1_Rate                           0x15
#define REG_Rest1_Downshift                      0x16
#define REG_Rest2_Rate                           0x17
#define REG_Rest2_Downshift                      0x18
#define REG_Rest3_Rate                           0x19
#define REG_Frame_Period_Max_Bound_Lower         0x1a
#define REG_Frame_Period_Max_Bound_Upper         0x1b
#define REG_Frame_Period_Min_Bound_Lower         0x1c
#define REG_Frame_Period_Min_Bound_Upper         0x1d
#define REG_Shutter_Max_Bound_Lower              0x1e
#define REG_Shutter_Max_Bound_Upper              0x1f
#define REG_LASER_CTRL0                          0x20
#define REG_Observation                          0x24
#define REG_Data_Out_Lower                       0x25
#define REG_Data_Out_Upper                       0x26
#define REG_SROM_ID                              0x2a
#define REG_Lift_Detection_Thr                   0x2e
#define REG_Configuration_V                      0x2f
#define REG_Configuration_IV                     0x39
#define REG_Power_Up_Reset                       0x3a
#define REG_Shutdown                             0x3b
#define REG_Inverse_Product_ID                   0x3f
#define REG_Snap_Angle                           0x42
#define REG_Motion_Burst                         0x50
#define REG_SROM_Load_Burst                      0x62
#define REG_Pixel_Burst                          0x64


/*
 * Inteface to a ADNS9800 'Mousesensor'
 */
class FilamentSensorADNS9800 {

        uint32_t lastTSs;
        int32_t lastASteps;
        // uint32_t lastTSf;

        uint8_t readLoc(uint8_t addr);
        void writeLoc(uint8_t addr, uint8_t value);
        uint8_t pullbyte();
        int16_t getDY();

        // Ratio of Measured filament speed to stepper speed [0.5%]
        // uint8_t grip;
        bool feedrateLimiterEnabled;

    public:

        // int32_t yPos;

        // Measured stepper speed [0.01 mm/s]
        // int16_t targetSpeed;
        // SpeedExpoFilter targetSpeed;

        // Measured filament speed [0.01 mm/s]
        // SpeedExpoFilter actualSpeed;

        // Running average measured filament speed
        // RunninAvg actualSpeed;

        // Ratio of target e-steps and filament sensor steps, this is a 
        // measure of the *feeder slippage*.
        RunningAvgF slippage;

        // Factor to slow down movement because feeder slippage is greater than 10%.
        float grip;

        // int16_t actualSpeed;

        FilamentSensorADNS9800();
        void init();
        void reset();
        // The polling method
        void run();
        void selfTest();

        void enableFeedrateLimiter(bool flag) { feedrateLimiterEnabled = flag; }
};

extern FilamentSensorADNS9800 filamentSensor;

#endif // #if defined(ADNSFS)

#if defined(BournsEMS22AFS)

/*
 * Inteface to a Bourns ems22a Rotary Encoder
 */
class FilamentSensor {

        int16_t getDY();

        int16_t lastEncoderPos;

        int32_t lastASteps;
        uint32_t lastTS;

        void spiInit(uint8_t spiRate);
        uint16_t readEncoderPos();

    public:

        int32_t yPos;
        // bool feedrateLimiterEnabled;

        float slip; // xxx rename to grip

        FilamentSensor();
        void init();
        // The polling method
        void run();
        // void selfTest();
};

extern FilamentSensor filamentSensor;

#endif // #if defined(BournsEMS22AFS)

