/************************************************************************************************
* Note by erwin.rieger@ibrieger.de:
* This file is part of ddprint - a direct drive 3d printer firmware.
* The Origin of this code is Ultimaker2Marlin (https://github.com/Ultimaker/Ultimaker2Marlin).
************************************************************************************************/

/*****************************************************************
* Ultiboard v2.0 pin assignment
******************************************************************/

#ifndef __AVR_ATmega2560__
 #error Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu.
#endif

#define X_STEP_PIN 25
#define X_DIR_PIN 23
#define X_STOP_PIN 22
#define X_ENABLE_PIN 27

#define Y_STEP_PIN 32
#define Y_DIR_PIN 33
#define Y_STOP_PIN 26
#define Y_ENABLE_PIN 31

#define Z_STEP_PIN 35
#define Z_DIR_PIN 36
#define Z_STOP_PIN 29
#define Z_ENABLE_PIN 34

#define HEATER_BED_PIN 4
#define TEMP_BED_PIN 10

#define HEATER_0_PIN  2
#define TEMP_0_PIN 8

#define HEATER_1_PIN 3
#define TEMP_1_PIN 9

#define HEATER_2_PIN -1
#define TEMP_2_PIN -1

#define E0_STEP_PIN         42
#define E0_DIR_PIN          43
#define E0_ENABLE_PIN       37

#define E1_STEP_PIN         49
#define E1_DIR_PIN          47
#define E1_ENABLE_PIN       48

#define SDPOWER            -1

// SPI
// Chip select SDCard
#define SDSS               53
#define SCK_PIN            52
#define MISO_PIN           50
#define MOSI_PIN           51

#define MAX6675_SS         13
#define LED_PIN            8
#define FAN_PIN            7
#define HOTEND_FAN_PIN     69       // Small fan behind hotend cooler, software switchable on newer UM2 boards.
#define PS_ON_PIN          24
#define SAFETY_TRIGGERED_PIN     28 //PIN to detect the safety circuit has triggered
#define MAIN_VOLTAGE_MEASURE_PIN 14 //Analogue PIN to measure the main voltage, with a 100k - 4k7 resitor divider.

#undef MOTOR_CURRENT_PWM_XY_PIN
#undef MOTOR_CURRENT_PWM_Z_PIN
#undef MOTOR_CURRENT_PWM_E_PIN
#define MOTOR_CURRENT_PWM_XY_PIN 44
#define MOTOR_CURRENT_PWM_Z_PIN 45
#define MOTOR_CURRENT_PWM_E_PIN 46
//Motor current PWM conversion, PWM value = MotorCurrentSetting * 255 / range
#define MOTOR_CURRENT_PWM_RANGE 2000

//arduino pin witch triggers an piezzo beeper
#define BEEPER 18

#define LCD_PINS_RS 20
#define LCD_PINS_ENABLE 15
#define LCD_PINS_D4 14
#define LCD_PINS_D5 21
#define LCD_PINS_D6 5
#define LCD_PINS_D7 6

//buttons are directly attached
#define BTN_EN1 40
#define BTN_EN2 41
#define BTN_ENC 19  //the click

#define BLEN_C 2
#define BLEN_B 1
#define BLEN_A 0

#define SDCARDDETECT 39

//encoder rotation values
#define encrot0 0
#define encrot1 1
#define encrot2 3
#define encrot3 2

//
// Filament sensor pins (half duplex/3wire spi)
//
// A5020
// #define FILSENSNCS   65
// A7550
#define FILSENSNCS   30
// #define FILSENSSDIO  66
// #define FILSENSSCLK  67


