
/************************************************************************************************
* Note by erwin.rieger@ibrieger.de:
* This file is part of ddprint - a 3d printer firmware.
* The Origin of this code is Ultimaker2Marlin (https://github.com/Ultimaker/Ultimaker2Marlin).
************************************************************************************************/

#pragma once

// This determines the communication speed of the printer

//
// Initial baudrate, host software may switch to an other
// baudrate.
//
// Baud 1000000, too high for anycubic i3
// #define BAUDRATE halBaudrate(1000000)
// Baud 500000
#define BAUDRATE halBaudrate(500000)
// Baud 250000
// #define BAUDRATE halBaudrate(250000)

// This defines the number of extruders
#define EXTRUDERS 1

//
//--NORMAL IS 4.7kohm PULLUP!-- 1kohm pullup can be used on hotend sensor, using correct resistor and table
//
//// Temperature sensor settings:
// -1 is thermocouple with AD595
// 0 is not used
// 1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
// 2 is 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
// 3 is mendel-parts thermistor (4.7k pullup)
// 4 is 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!
// 5 is 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan) (4.7k pullup)
// 6 is 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
// 7 is 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
// 8 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
// 9 is 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
// 10 is 100k RS thermistor 198-961 (4.7k pullup)
// 20 is PT100 with INA826 amp in Ultiboard v2.0
//
//    1k ohm pullup tables - This is not normal, you would have to have changed out your 4.7k for 1k
//                          (but gives greater accuracy and more stable PID)
// 51 is 100k thermistor - EPCOS (1k pullup)
// 52 is 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
// 55 is 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan) (1k pullup)

// Actual temperature must be close to target for this long before M109 returns success
// #define TEMP_RESIDENCY_TIME 3   // (seconds)
// #define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
// #define TEMP_WINDOW     2       // (degC) Window around target to start the residency timer x degC early.

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP toFWTemp((int16_t)5)
#define HEATER_1_MINTEMP toFWTemp((int16_t)5)
#define HEATER_2_MINTEMP toFWTemp((int16_t)5)

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP toFWTemp((int16_t)125)
#define HEATER_1_MAXTEMP toFWTemp((int16_t)285)
#define HEATER_2_MAXTEMP toFWTemp((int16_t)285)

//Check if the heater heats up MAX_HEATING_TEMPERATURE_INCREASE within MAX_HEATING_CHECK_MILLIS while the PID was at the maximum.
// If not, raise an error because most likely the heater is not heating up the temperature sensor. Indicating an issue in the system.
// #define MAX_HEATING_TEMPERATURE_INCREASE 10
// #define MAX_HEATING_CHECK_MILLIS (20 * 1000)

// PID settings:
#define PID_MAX 255 // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current

// #if defined(DualZStepper)
    // #define ADDZAXIS 1
// #else
    #define ADDZAXIS 0
// #endif
#define NUM_AXIS (4+ADDZAXIS) // The axis order in all axis related arrays is X, Y, Z, E [, Z1]

// Default motor current for XY,Z,E in mA
#define DEFAULT_PWM_MOTOR_CURRENT 1300

#include "thermistortables.h"

