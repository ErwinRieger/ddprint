/************************************************************************************************
* Note by erwin.rieger@ibrieger.de:
* This file is part of ddprint - a direct drive 3d printer firmware.
* The Origin of this code is Ultimaker2Marlin (https://github.com/Ultimaker/Ultimaker2Marlin).
************************************************************************************************/

/*****************************************************************
* Ultiboard v2.0 pin assignment
******************************************************************/

#define X_STEP_PIN              DigitalOutput<25, ACTIVEHIGHPIN>
#define X_DIR_PIN               DigitalOutput<23, ACTIVEHIGHPIN>
#define X_STOP_PIN              DigitalInput<22, ACTIVELOWPIN>
#define X_ENABLE_PIN            DigitalOutput<27, ACTIVELOWPIN>

#define Y_STEP_PIN              DigitalOutput<32, ACTIVEHIGHPIN>
#define Y_DIR_PIN               DigitalOutput<33, ACTIVEHIGHPIN>
#define Y_STOP_PIN              DigitalInput<26, ACTIVELOWPIN>
#define Y_ENABLE_PIN            DigitalOutput<31, ACTIVELOWPIN>

#define Z_STEP_PIN              DigitalOutput<35, ACTIVEHIGHPIN>
#define Z_DIR_PIN               DigitalOutput<36, ACTIVEHIGHPIN>
#define Z_STOP_PIN              DigitalInput<29, ACTIVELOWPIN>
#define Z_ENABLE_PIN            DigitalOutput<34, ACTIVELOWPIN>

#define HEATER_BED_PIN          DigitalOutput<4, ACTIVEHIGHPIN>

#define TEMP_BED_PIN 10

#define HEATER_0_PIN            PWMOutput<2, ACTIVEHIGHPIN>
#define TEMP_0_PIN 8

#define HEATER_1_PIN            PWMOutput<3, ACTIVEHIGHPIN>
#define TEMP_1_PIN 9

// #define HEATER_2_PIN -1
// #define TEMP_2_PIN -1

#define E0_STEP_PIN             DigitalOutput<42, ACTIVEHIGHPIN>
#define E0_DIR_PIN              DigitalOutput<43, ACTIVEHIGHPIN>
#define E0_ENABLE_PIN           DigitalOutput<37, ACTIVELOWPIN>

// Second extruder not used 
//#define E1_STEP_PIN         49
//#define E1_DIR_PIN          47
//#define E1_ENABLE_PIN       48
//#define E1_ENABLE_ACTIVE    LOW

// SPI
// Chip select SDCard
#define SDSS               53
#define SCK_PIN            52
#define MISO_PIN           50
#define MOSI_PIN           51

#define MAX6675_SS         13
#define LED_PIN                 PWMOutput<8, ACTIVEHIGHPIN>

#define FAN_PIN                 PWMOutput<7, ACTIVEHIGHPIN> 

#define HOTEND_FAN_PIN          DigitalOutput<69, ACTIVEHIGHPIN>       // Small fan behind hotend cooler, software switchable on newer UM2 boards.

// #define POWER_SUPPLY_RELAY 24
#define SAFETY_TRIGGERED_PIN     28 //PIN to detect the safety circuit has triggered
#define MAIN_VOLTAGE_MEASURE_PIN 14 //Analogue PIN to measure the main voltage, with a 100k - 4k7 resitor divider.

#define MOTOR_CURRENT_PWM_XY_PIN    PWMOutput<44, ACTIVEHIGHPIN>
#define MOTOR_CURRENT_PWM_Z_PIN     PWMOutput<45, ACTIVEHIGHPIN>
#define MOTOR_CURRENT_PWM_E_PIN     PWMOutput<46, ACTIVEHIGHPIN>
//Motor current PWM conversion, PWM value = MotorCurrentSetting * 255 / range
#define MOTOR_CURRENT_PWM_RANGE 2000

//
// Filament sensor pins (half duplex/3wire spi)
//
// A5020
// #define FILSENSNCS   65
// A7550
#define FILSENSNCS   30
// #define FILSENSSDIO  66
// #define FILSENSSCLK  67


