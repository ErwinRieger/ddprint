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

/*****************************************************************
* Jennyprinter arm shuttle gear
******************************************************************/

//
// See also:
// * maple/libmaple/timer_map.c
//

#define POWER_BUTTON            DigitalInput<15, INPUT_PULLDOWN, ACTIVEHIGHPIN>  // ["GPIOA","D15","PA15"], # Known: Input, Power button

#define POWER_SUPPLY_RELAY      DigitalOutput<28, ACTIVEHIGHPIN>  // ["GPIOB","D28","PB12"], # Known: Output, Power-Relais  

#define X_STEP_PIN              DigitalOutput<18, ACTIVEHIGHPIN>  // ["GPIOB","D18","PB2"],  # Known: Output, X-Step
// Active state is forward direction (positive)
#define X_DIR_PIN               DigitalOutput<0, ACTIVEHIGHPIN>  // ["GPIOA","D00","PA0"],  # Known: Output, X-Dir
#define X_ENABLE_PIN            DigitalOutput<66, ACTIVELOWPIN>  // ["GPIOE","D66","PE2"],  # Known: Output, X-Enable, active low
#define X_STOP_PIN              DigitalInput<11, INPUT_PULLUP, ACTIVELOWPIN>  // ["GPIOA","D11","PA11"], # Known: Input, X-Endstop, active low
//#define X_ENABLE_ACTIVE     LOW

#define Y_STEP_PIN              DigitalOutput<46, ACTIVEHIGHPIN>  // ["GPIOC","D46","PC14"], # Known: Output, Y-Step
// Active state is forward direction (positive)
#define Y_DIR_PIN               DigitalOutput<70, ACTIVELOWPIN>  // ["GPIOE","D70","PE6"],  # Known: Output, Y-Dir
#define Y_ENABLE_PIN            DigitalOutput<47, ACTIVELOWPIN>  // ["GPIOC","D47","PC15"], # Known: Output, Y-Enable, active low
#define Y_STOP_PIN              DigitalInput<60, INPUT_PULLUP, ACTIVELOWPIN>  // ["GPIOD","D60","PD12"], # Known: Input, Y-Endstop, active low
// #define Y_ENABLE_ACTIVE     LOW

#define Z_STEP_PIN              DigitalOutput<69, ACTIVEHIGHPIN>  // ["GPIOE","D69","PE5"],  # Known: Output, Z-Step
// Active state is forward direction (positive)
#define Z_DIR_PIN               DigitalOutput<68, ACTIVEHIGHPIN>  // ["GPIOE","D68","PE4"],  # Known: Output, Z-Dir
#define Z_ENABLE_PIN            DigitalOutput<45, ACTIVELOWPIN>  // ["GPIOC","D45","PC13"], # Known: Output, Z-Enable, active low
#define Z_STOP_PIN              DigitalInput<65, INPUT_PULLUP, ACTIVELOWPIN>  // ["GPIOE","D65","PE1"],  # Known: Input, Z-Endstop, active low
// #define Z_ENABLE_ACTIVE     LOW

#define E0_STEP_PIN             DigitalOutput<1, ACTIVEHIGHPIN>  // ["GPIOA","D01","PA1"],  # Known: Output, E0-Step
// Active state is forward direction (positive)
#define E0_DIR_PIN              DigitalOutput<16, ACTIVEHIGHPIN>  // ["GPIOB","D16","PB0"],  # Known: Output, E0-Dir
#define E0_ENABLE_PIN           DigitalOutput<37, ACTIVELOWPIN>  // ["GPIOC","D37","PC5"],  # Known: Output, E0-Enable, active low
// #define E0_ENABLE_ACTIVE    LOW

// Second extruder not used 
//#define E1_STEP_PIN         36  // ["GPIOC","D36","PC4"],  # Known: Output, E1-Step
// Active state is forward direction (positive)
//#define E1_DIR_PIN          67  // ["GPIOE","D67","PE3"],  # Known: Output, E1-Dir
//#define E1_ENABLE_PIN       17  // ["GPIOB","D17","PB1"],  # Known: Output, E1-Enable, active low
//#define E1_ENABLE_ACTIVE    LOW

// Small fan behind hotend cooler, software switchable on newer UM2 boards.
#define HOTEND_FAN_PIN          DigitalOutput<24, ACTIVELOWPIN> // ["GPIOB","D24","PB8"],  # Known, Output, Hotend Fan (Fan 1), active low
// #define HOTEND_FAN_ACTIVE   LOW

#define LED_PIN                 PWMOutput<3, ACTIVELOWPIN>   // ["GPIOA","D03","PA3"],  # Known: Output, Case LED, active low

// Note: PB13 has no associated timer in stm32duino -> use bitbang temp control
#define HEATER_BED_PIN          DigitalOutput<29, ACTIVELOWPIN>  // ["GPIOB","D29","PB13"], # Known: Output, Heater Bed, active low

#define HEATER_0_PIN            PWMOutput<38, ACTIVELOWPIN>  // ["GPIOC","D38","PC6"],  # Known: Output, Heater Head 0, active low

#define HEATER_1_PIN            PWMOutput<39, ACTIVELOWPIN>  // ["GPIOC","D39","PC7"],  # Known: Output, Heater Head 1, active low

// #define HEATER_2_PIN        -1

#define FAN_PIN                 PWMOutput<22, ACTIVELOWPIN>  // ["GPIOB","D22","PB6"],  # Known: Output, Fan 0, Part cooling fan, active low

#define TEMP_BED_PIN            AnalogInput<33, 11> // ["GPIOC","D33","PC1"],  # Known: Analog Input, Temp. PT100 Sensor BED
#define TEMP_0_PIN              AnalogInput<32, 10> // ["GPIOC","D32","PC0"],  # Known: Analog Input, Temp. PT100 Sensor X
// #define TEMP_1_PIN              AnalogInput<35, 13> // ["GPIOC","D35","PC3"],  # Known: Analog Input, Temp. PT100 Sensor X 
// #define TEMP_2_PIN              AnalogInput<34, 12> // ["GPIOC","D34","PC2"],  # Known: Analog Input, Temp. PT100 Sensor, nothing connected

//
// USB OTG_HS Host, hardcoded in xxxx!
//
// ["GPIOB","D30","PB14"], # Known: USB OTG DM
// ["GPIOB","D31","PB15"], # Known: USB OTG DP

// SPI for flowrate sensor
// Chip select SDCard
#define FILSENSNCS              DigitalOutput<72, ACTIVELOWPIN> // ["GPIOE","D72","PE8"],  # Known: Chip select for flowrate sensor
#define SCK_PIN                 PA4                             // ["GPIOA","D04","PA4"],  # Known: USART2_CK, SPI Clock flowrate sensor
#define MISO_PIN                PD6                             // ["GPIOD","D54","PD6"],  # Known: USART2 RX, MISO flowrate sensor
#define MOSI_PIN                PD5                             // ["GPIOD","D53","PD5"],  # Known: USART2 TX, could be MISO but not used

#if 0
#define TEMP_BED_PIN 10

#define TEMP_0_PIN 8

#define TEMP_1_PIN 9

#define TEMP_2_PIN -1

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
#endif

#if 0

        ["GPIOB","D19","PB3"],  # Known: Output, E2-Step

        ["GPIOA","D02","PA2"],
        ["GPIOA","D05","PA5"],
        ["GPIOA","D06","PA6"],
        ["GPIOA","D07","PA7"],
        ["GPIOA","D08","PA8"],  # ???: Input, changes with power-on relais ???
        ["GPIOA","D09","PA9"],  # Known: Output, USART1 Tx
        ["GPIOA","D10","PA10"], # Known: Input, USART1 Rx
        ["GPIOA","D12","PA12"],
        ["GPIOA","D13","PA13"],
        ["GPIOA","D14","PA14"],
        ["GPIOA","D15","PA15"], # Known: Input, Power button
        ["GPIOB","D20","PB4"],  # Known: Output, E2-Dir
        ["GPIOB","D21","PB5"],  # Known: Output, E2-Enable
        ["GPIOB","D23","PB7"],  # ???: Input, changes with power-on relais ???
        ["GPIOB","D25","PB9"],  # Known, Output, Fan 2, not connected
        ["GPIOB","D26","PB10"], # ???: Input, changes with power-on relais ???
        ["GPIOB","D27","PB11"], # ???: Input, changes with power-on relais ???
        ["GPIOB","D28","PB12"], # Known: Output, Power-Relais
        ["GPIOC","D40","PC8"],
        ["GPIOC","D41","PC9"],
        ["GPIOC","D42","PC10"],
        ["GPIOC","D43","PC11"],
        ["GPIOC","D44","PC12"],
        ["GPIOD","D48","PD0"],
        ["GPIOD","D49","PD1"],  # ???: Input, changes with power-on relais ???
        ["GPIOD","D50","PD2"],  # ???: Input, changes with power-on relais ???
        ["GPIOD","D51","PD3"],  # ???: Input, changes with power-on relais ???
        ["GPIOD","D52","PD4"],
        ["GPIOD","D55","PD7"],  
        ["GPIOD","D56","PD8"],
        ["GPIOD","D57","PD9"],
        ["GPIOD","D58","PD10"],
        ["GPIOD","D59","PD11"],
        ["GPIOD","D61","PD13"],
        ["GPIOD","D62","PD14"],
        ["GPIOD","D63","PD15"], # ???: Input, changes with power-on relais ???
        ["GPIOE","D64","PE0"],
        ["GPIOE","D71","PE7"],
        ["GPIOE","D73","PE9"],  # ???: Input, changes with power-on relais ???
        ["GPIOE","D74","PE10"],
        ["GPIOE","D75","PE11"],
        ["GPIOE","D76","PE12"],
        ["GPIOE","D77","PE13"],
        ["GPIOE","D78","PE14"],
        ["GPIOE","D79","PE15"],
#endif

