
/************************************************************************************************
* Note by erwin.rieger@ibrieger.de:
* This file is part of ddprint - a direct drive 3d printer firmware.
* The Origin of this code is Ultimaker2Marlin (https://github.com/Ultimaker/Ultimaker2Marlin).
************************************************************************************************/

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#if MOTHERBOARD == 33
    #include "Configuration_ramps.h"
#else
    #include "Configuration_um2.h"
#endif

// This configuration file contains the basic settings.
// Advanced settings can be found in Configuration_adv.h
// BASIC SETTINGS: select your board type, temperature sensor type, axis scaling, and endstop configuration

// User-specified version info of this build to display in [Pronterface, etc] terminal window during
// startup. Implementation of an idea by Prof Braino to inform user that any changes made to this
// build by the user have been successfully uploaded into firmware.
#define STRING_VERSION_CONFIG_H __DATE__ " " __TIME__ // build date and time
#ifndef STRING_CONFIG_H_AUTHOR
#define STRING_CONFIG_H_AUTHOR "Version DEV" // Who made the changes.
#endif

// SERIAL_PORT selects which serial port should be used for communication with the host.
// This allows the connection of wireless adapters (for instance) to non-default port pins.
// Serial port 0 is still used by the Arduino bootloader regardless of this setting.
#define SERIAL_PORT 0

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
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_BED 20

// This makes temp sensor 1 a redundant sensor for sensor 0. If the temperatures difference between these sensors is to high the print will be aborted.
//#define TEMP_SENSOR_1_AS_REDUNDANT
#define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10

// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 3   // (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     2       // (degC) Window around target to start the residency timer x degC early.

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define BED_MINTEMP 5

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 285
#define HEATER_1_MAXTEMP 285
#define HEATER_2_MAXTEMP 285
#define BED_MAXTEMP 150

//Check if the heater heats up MAX_HEATING_TEMPERATURE_INCREASE within MAX_HEATING_CHECK_MILLIS while the PID was at the maximum.
// If not, raise an error because most likely the heater is not heating up the temperature sensor. Indicating an issue in the system.
#define MAX_HEATING_TEMPERATURE_INCREASE 10
#define MAX_HEATING_CHECK_MILLIS (20 * 1000)

// If your bed has low resistance e.g. .6 ohm and throws the fuse you can duty cycle it to reduce the
// average current. The value should be an integer and the heat bed will be turned on for 1 interval of
// HEATER_BED_DUTY_CYCLE_DIVIDER intervals.
//#define HEATER_BED_DUTY_CYCLE_DIVIDER 4

// PID settings:
// Comment the following line to disable PID and enable bang-bang.
#define PIDTEMP

#ifdef PIDTEMP

    #define PID_MAX 255 // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
    //#define PID_OPENLOOP 1 // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
    // #define PID_FUNCTIONAL_RANGE 1000 // If the temperature difference between the target temperature and the actual temperature
    //                                // is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.

    // UM2 with olson block and 35W heater
    #define  DEFAULT_Kp 1
    #define  DEFAULT_Ki 0.01
    #define  DEFAULT_Kd 2.5
#endif // PIDTEMP

//this prevents dangerous Extruder moves, i.e. if the temperature is under the limit
//can be software-disabled for whatever purposes by
#define PREVENT_DANGEROUS_EXTRUDE
//if PREVENT_DANGEROUS_EXTRUDE is on, you can still disable (uncomment) very long bits of extrusion separately.
#define PREVENT_LENGTHY_EXTRUDE

#define EXTRUDE_MINTEMP 170
// #define EXTRUDE_MAXLENGTH 1000.0 //prevent extrusion of very large distances.
#define EXTRUDE_MAXSTEPS (100*AXIS_STEPS_PER_MM_E)  //prevent extrusion of very large distances.

//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// Uncomment the following line to enable CoreXY kinematics
// #define COREXY

// coarse Endstop Settings
#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors

#ifndef ENDSTOPPULLUPS
  // fine Enstop settings: Individual Pullups. will be ignored if ENDSTOPPULLUPS is defined
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  //#define ENDSTOPPULLUP_ZMIN
#endif

#ifdef ENDSTOPPULLUPS
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  #define ENDSTOPPULLUP_ZMIN
#endif

// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
const bool X_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
const bool Y_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
const bool Z_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

// Disables axis when it's not being used.
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false // For all extruders

//
// Stepper direction pins. Set this to true, if the output pin has to be switched HIGH
// for a movement into positive direction.
//
#define POSITIVE_X_DIR false
#define POSITIVE_Y_DIR true
#define POSITIVE_Z_DIR false
#define POSITIVE_E1_DIR true
#define POSITIVE_E2_DIR true

// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR 1
#define Z_HOME_DIR 1

#define MIN_SOFTWARE_ENDSTOPS 1 // If true, axis won't move to coordinates less than HOME_POS.
#define MAX_SOFTWARE_ENDSTOPS 1  // If true, axis won't move to coordinates greater than the defined lengths below.

// Travel limits after homing
#define X_MAX_POS 225 // 230
#define X_MIN_POS 0
#define Y_MAX_POS 225 // 230
#define Y_MIN_POS 0
// #define Z_MAX_POS 230
// #define Z_MAX_POS 229 // Dauerdruckplatte hat 5mm im vergleich zur glassplatte 4mm
#define Z_MAX_POS 212.25 // solex nozzle
#define Z_MIN_POS 0

#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)

// The position of the homing switches
//#define MANUAL_HOME_POSITIONS  // If defined, MANUAL_*_HOME_POS below will be used
//#define BED_CENTER_AT_0_0  // If defined, the center of the bed is at (X=0, Y=0)

//Manual homing switch locations:
// For deltabots this means top and center of the cartesian print volume.
#define MANUAL_X_HOME_POS 0
#define MANUAL_Y_HOME_POS 0
#define MANUAL_Z_HOME_POS 0
//#define MANUAL_Z_HOME_POS 402 // For delta: Distance between nozzle and print surface after homing.

// #define DEFAULT_AXIS_STEPS_PER_UNIT   {80.0,80.0,200,282}  // default steps per unit for ultimaker2
#define AXIS_STEPS_PER_MM_X 80
#define AXIS_STEPS_PER_MM_Y 80
#define AXIS_STEPS_PER_MM_Z 200
// #define AXIS_STEPS_PER_MM_E 282  // Original UM2 feeder motor
#define AXIS_STEPS_PER_MM_E 141     // 60mm 0.65Nm feeder motor
// #define AXIS_STEPS_PER_MM_E 495     // Bulldog XL feeder

#define X_MIN_POS_STEPS (long)(X_MIN_POS * AXIS_STEPS_PER_MM_X)
#define X_MAX_POS_STEPS (long)(X_MAX_POS * AXIS_STEPS_PER_MM_X)
#define Y_MIN_POS_STEPS (long)(Y_MIN_POS * AXIS_STEPS_PER_MM_Y)
#define Y_MAX_POS_STEPS (long)(Y_MAX_POS * AXIS_STEPS_PER_MM_Y)
#define Z_MIN_POS_STEPS (long)(Z_MIN_POS * AXIS_STEPS_PER_MM_Z)
#define Z_MAX_POS_STEPS (long)(Z_MAX_POS * AXIS_STEPS_PER_MM_Z)

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

// minimum time in microseconds that a movement needs to take if the buffer is emptied.
#define DEFAULT_MINSEGMENTTIME        20000

// steps/s
// #define MIN_TRAVEL_FEEDRATE_STEPS (10*60)

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
// For the other hotends it is their distance from the extruder 0 hotend.
// #define EXTRUDER_OFFSET_X {0.0, 20.00} // (in mm) for each extruder, offset of the hotend on the X axis
// #define EXTRUDER_OFFSET_Y {0.0, 5.00}  // (in mm) for each extruder, offset of the hotend on the Y axis

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#define DEFAULT_XYJERK                20.0    // (mm/sec)
#define DEFAULT_ZJERK                 0.5     // (mm/sec)
#define DEFAULT_EJERK                 5.0    // (mm/sec)

//Length of the bowden tube. Used for the material load/unload procedure.
#define FILAMANT_BOWDEN_LENGTH        705

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

// EEPROM
// the microcontroller can store settings in the EEPROM, e.g. max velocity...
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
//define this to enable eeprom support
#define EEPROM_SETTINGS
//to disable EEPROM Serial responses and decrease program space by ~1700 byte: comment this out:
// please keep turned on if you can.
#define EEPROM_CHITCHAT

// Preheat Constants
#define PLA_PREHEAT_HOTEND_TEMP 180
#define PLA_PREHEAT_HPB_TEMP 70
#define PLA_PREHEAT_FAN_SPEED 0     // Insert Value between 0 and 255

#define ABS_PREHEAT_HOTEND_TEMP 240
#define ABS_PREHEAT_HPB_TEMP 100
#define ABS_PREHEAT_FAN_SPEED 0     // Insert Value between 0 and 255

//LCD and SD support
#define SDSUPPORT // Enable SD Card Support in Hardware Console

#define ENABLE_ULTILCD2 //128x64 pixel display in the Ultimaker 2, with new menus. Note: For compiling with Arduino you need to remove the "SIGNAL(TWI_vect)" function from "libraries/Wire/utility/twi.c"

//I2C PANELS

// Incrementing this by 1 will double the software PWM frequency,
// affecting heaters, and the fan if FAN_SOFT_PWM is enabled.
// However, control resolution will be halved for each increment;
// at zero value, there are 128 effective control positions.
#define SOFT_PWM_SCALE 0

// Configuration of behaviors at the start and end of prints
#define END_OF_PRINT_RETRACTION 20		// number of mm to retract when printer goes idle
#define END_OF_PRINT_RECOVERY_SPEED 5 	// speed to recover that assumed retraction at (mm/s)
#define PRIMING_MM3	50					// number of mm^3 of plastic to extrude when priming
										// (Ultimaker 2 hot end capacity is approx 80 mm^3)
#define PRIMING_MM3_PER_SEC 5			// Rate at which to prime head (in mm^3/s)
										// (Ultimaker 2 upper limit is 8-10)
#define PRIMING_HEIGHT 20				// Height at which to perform the priming extrusions

// Bed leveling wizard configuration
#define LEVELING_OFFSET 0.1				// Assumed thickness of feeler gauge/paper used in leveling (mm)

#include "Configuration_adv.h"
#include "thermistortables.h"

#endif //__CONFIGURATION_H
