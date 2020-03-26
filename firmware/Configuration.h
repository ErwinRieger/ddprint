
/************************************************************************************************
* Note by erwin.rieger@ibrieger.de:
* This file is part of ddprint - a direct drive 3d printer firmware.
* The Origin of this code is Ultimaker2Marlin (https://github.com/Ultimaker/Ultimaker2Marlin).
************************************************************************************************/

#pragma once

#if MOTHERBOARD == 33
    #include "Configuration_ramps.h"
#else
    #include "Configuration_um2.h"
#endif

// This determines the communication speed of the printer
// #define BAUDRATE 115200
// #define BAUDRATE 230400
// #define BAUDRATE 250000
#define BAUDRATE 500000
// #define BAUDRATE 1000000

// This defines the number of extruders
#define EXTRUDERS 1

//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================
//
//--NORMAL IS 4.7kohm PULLUP!-- 1kohm pullup can be used on hotend sensor, using correct resistor and table
//
//// Temperature sensor settings:
// -2 is thermocouple with MAX6675 (only for sensor 0)
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

#define TEMP_SENSOR_0 20
#define TEMP_SENSOR_1 20
#define TEMP_SENSOR_BED 20

// This makes temp sensor 1 a redundant sensor for sensor 0. If the temperatures difference between these sensors is to high the print will be aborted.
//#define TEMP_SENSOR_1_AS_REDUNDANT
// #define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10

// Actual temperature must be close to target for this long before M109 returns success
// #define TEMP_RESIDENCY_TIME 3   // (seconds)
// #define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
// #define TEMP_WINDOW     2       // (degC) Window around target to start the residency timer x degC early.

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 125
#define HEATER_1_MAXTEMP 285
#define HEATER_2_MAXTEMP 285

//Check if the heater heats up MAX_HEATING_TEMPERATURE_INCREASE within MAX_HEATING_CHECK_MILLIS while the PID was at the maximum.
// If not, raise an error because most likely the heater is not heating up the temperature sensor. Indicating an issue in the system.
#define MAX_HEATING_TEMPERATURE_INCREASE 10
#define MAX_HEATING_CHECK_MILLIS (20 * 1000)

// If your bed has low resistance e.g. .6 ohm and throws the fuse you can duty cycle it to reduce the
// average current. The value should be an integer and the heat bed will be turned on for 1 interval of
// HEATER_BED_DUTY_CYCLE_DIVIDER intervals.
//#define HEATER_BED_DUTY_CYCLE_DIVIDER 4

// PID settings:
#define PID_MAX 255 // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
//#define PID_OPENLOOP 1 // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
// #define PID_FUNCTIONAL_RANGE 1000 // If the temperature difference between the target temperature and the actual temperature
//                                // is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.

// UM2 with olson block and 35W heater
#define  DEFAULT_Kp 1
#define  DEFAULT_Ki 0.01
#define  DEFAULT_Kd 2.5

//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

const bool X_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
const bool Y_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
const bool Z_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.

// Logic levels for Stepper Enable Pins (Active Low = LOW), Active High = HIGH)
#define X_ENABLE_ON LOW
#define Y_ENABLE_ON LOW
#define Z_ENABLE_ON LOW
#define E_ENABLE_ON LOW // For all extruders

//
// Stepper direction pins. Set this to the logical level for a movement into positive direction.
//
#define POSITIVE_X_DIR LOW
#define POSITIVE_Y_DIR HIGH
#define POSITIVE_Z_DIR LOW
#define POSITIVE_E1_DIR LOW
#define POSITIVE_E2_DIR LOW

// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR 1
#define Z_HOME_DIR 1

// Travel limits after homing
#define X_MAX_POS 230
#define X_MIN_POS 0
#define Y_MAX_POS 230
#define Y_MIN_POS 0
#define Z_MAX_POS 230
#define Z_MIN_POS 0

#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)

// #define DEFAULT_AXIS_STEPS_PER_UNIT   {80.0,80.0,200,282}  // default steps per unit for ultimaker2
// XXX todo: steps_per_mm defined here and in printer profile
#define AXIS_STEPS_PER_MM_X 80
#define AXIS_STEPS_PER_MM_Y 80
#define AXIS_STEPS_PER_MM_Z 200

#define X_MIN_POS_STEPS ((long)X_MIN_POS * AXIS_STEPS_PER_MM_X)
#define X_MAX_POS_STEPS ((long)X_MAX_POS * AXIS_STEPS_PER_MM_X)
#define Y_MIN_POS_STEPS ((long)Y_MIN_POS * AXIS_STEPS_PER_MM_Y)
#define Y_MAX_POS_STEPS ((long)Y_MAX_POS * AXIS_STEPS_PER_MM_Y)
#define Z_MIN_POS_STEPS ((long)Z_MIN_POS * AXIS_STEPS_PER_MM_Z)
#define Z_MAX_POS_STEPS ((long)Z_MAX_POS * AXIS_STEPS_PER_MM_Z)

//// MOVEMENT SETTINGS
#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E
// #define HOMING_FEEDRATE {100*60, 100*60, 40*60, 0}  // set the homing speeds (mm/min)
// xxx faktor 60!!!
// #define HOMING_FEEDRATE_STEPS {100*AXIS_STEPS_PER_MM_X, 100*AXIS_STEPS_PER_MM_Y, 40*AXIS_STEPS_PER_MM_Z, 0}  // set the homing speeds (mm/s)

// default settings

#define DEFAULT_MAX_FEEDRATE          {300, 300, 40, 45}    // (mm/sec)
#define DEFAULT_MAX_ACCELERATION      {9000,9000,100,10000}    // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.

#define DEFAULT_ACCELERATION          3000    // X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION  3000   // X, Y, Z and E max acceleration in mm/s^2 for retracts

#define DEFAULT_MINIMUMFEEDRATE       0.0     // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE     0.0

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

// EEPROM
// the microcontroller can store settings in the EEPROM, e.g. max velocity...
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
//define this to enable eeprom support
// #define EEPROM_SETTINGS
//to disable EEPROM Serial responses and decrease program space by ~1700 byte: comment this out:
// please keep turned on if you can.
// #define EEPROM_CHITCHAT

// Preheat Constants
// #define PLA_PREHEAT_HOTEND_TEMP 180
// #define PLA_PREHEAT_HPB_TEMP 70
// #define PLA_PREHEAT_FAN_SPEED 0     // Insert Value between 0 and 255

// #define ABS_PREHEAT_HOTEND_TEMP 240
// #define ABS_PREHEAT_HPB_TEMP 100
// #define ABS_PREHEAT_FAN_SPEED 0     // Insert Value between 0 and 255

//LCD and SD support
#define SDSUPPORT // Enable SD Card Support in Hardware Console

// #define ENABLE_ULTILCD2 //128x64 pixel display in the Ultimaker 2, with new menus. Note: For compiling with Arduino you need to remove the "SIGNAL(TWI_vect)" function from "libraries/Wire/utility/twi.c"

//I2C PANELS

// Incrementing this by 1 will double the software PWM frequency,
// affecting heaters, and the fan if FAN_SOFT_PWM is enabled.
// However, control resolution will be halved for each increment;
// at zero value, there are 128 effective control positions.
// #define SOFT_PWM_SCALE 0

// Configuration of behaviors at the start and end of prints
// #define END_OF_PRINT_RETRACTION 20		// number of mm to retract when printer goes idle
// #define END_OF_PRINT_RECOVERY_SPEED 5 	// speed to recover that assumed retraction at (mm/s)
// #define PRIMING_MM3	50					// number of mm^3 of plastic to extrude when priming
										// (Ultimaker 2 hot end capacity is approx 80 mm^3)
// #define PRIMING_MM3_PER_SEC 5			// Rate at which to prime head (in mm^3/s)
										// (Ultimaker 2 upper limit is 8-10)
// #define PRIMING_HEIGHT 20				// Height at which to perform the priming extrusions

// Bed leveling wizard configuration
// #define LEVELING_OFFSET 0.1				// Assumed thickness of feeler gauge/paper used in leveling (mm)


// If the temperature has not increased at the end of that period, the target temperature is set to zero.
// It can be reset with another M104/M109. This check is also only triggered if the target temperature and the current temperature
//  differ by at least 2x WATCH_TEMP_INCREASE
//#define WATCH_TEMP_PERIOD 40000 //40 seconds
//#define WATCH_TEMP_INCREASE 10  //Heat up at least 10 degree in 20 seconds

//By default pololu step drivers require an active high signal. However, some high power drivers require an active low signal as step.
// #define INVERT_X_STEP_PIN false
// #define INVERT_Y_STEP_PIN false
// #define INVERT_Z_STEP_PIN false
// #define INVERT_E_STEP_PIN false

// Default motor current for XY,Z,E in mA
// #define DEFAULT_PWM_MOTOR_CURRENT {1300, 1300, 1250}
#define DEFAULT_PWM_MOTOR_CURRENT {1300, 1300, 1300}

#include "thermistortables.h"

