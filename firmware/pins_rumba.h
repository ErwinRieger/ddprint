/*
* This file is part of ddprint - a direct drive 3D printer firmware.
* 
* Copyright 2022 erwin.rieger@ibrieger.de
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


//
// Rumba board
//

// #define X_STEP_PIN         17
// #define X_DIR_PIN          16
// #define X_ENABLE_PIN       48
// #define X_MIN_PIN          37
// #define X_MAX_PIN          36 
#define X_STEP_PIN         DigitalOutput<17, ACTIVEHIGHPIN>
#define X_DIR_PIN          DigitalOutput<16, ACTIVELOWPIN>
#define X_STOP_PIN         DigitalInput<37, ACTIVELOWPIN> /* 3 */
#define X_ENABLE_PIN       DigitalOutput<48, ACTIVELOWPIN> /* 38 */

// #define Y_STEP_PIN         54
// #define Y_DIR_PIN          47 
// #define Y_ENABLE_PIN       55
// #define Y_MIN_PIN          35
// #define Y_MAX_PIN          34 
#define Y_STEP_PIN         DigitalOutput<54, ACTIVEHIGHPIN>
#define Y_DIR_PIN          DigitalOutput<47, ACTIVELOWPIN>
#define Y_STOP_PIN         DigitalInput<35, ACTIVELOWPIN>
#define Y_ENABLE_PIN       DigitalOutput<55, ACTIVELOWPIN>

// #define Z_STEP_PIN         57 
// #define Z_DIR_PIN          56
// #define Z_ENABLE_PIN       62 
// #define Z_MIN_PIN          33
// #define Z_MAX_PIN          32
#define Z_STEP_PIN              DigitalOutput<57, ACTIVEHIGHPIN>
#define Z_DIR_PIN               DigitalOutput<56, ACTIVELOWPIN>
#define Z_STOP_PIN              DigitalInput<33, ACTIVELOWPIN>
#define Z_ENABLE_PIN            DigitalOutput<62, ACTIVELOWPIN>

// #define E0_STEP_PIN        23
// #define E0_DIR_PIN         22
// #define E0_ENABLE_PIN      24
#define E0_STEP_PIN             DigitalOutput<23, ACTIVEHIGHPIN>
#define E0_DIR_PIN              DigitalOutput<22, ACTIVELOWPIN>
#define E0_ENABLE_PIN           DigitalOutput<24, ACTIVELOWPIN>


#define HEATER_BED_PIN     DigitalOutput<9, ACTIVEHIGHPIN>    // BED

#define HEATER_0_PIN        PWMOutput<2, ACTIVEHIGHPIN>   // EXTRUDER 1
#define HEATER_1_PIN        PWMOutput<3, ACTIVEHIGHPIN>   // EXTRUDER 2

#define TEMP_0_PIN         15   // ANALOG NUMBERING
#define TEMP_1_PIN         14   // ANALOG NUMBERING

#define TEMP_BED_PIN       11   // ANALOG NUMBERING

// SPI
// Chip select SDCard
#define SDSS               DigitalOutput<53, ACTIVELOWPIN>
#define SCK_PIN        DigitalOutput<52, ACTIVEHIGHPIN>
// #define MISO_PIN           DigitalInput<50...
#define MOSI_PIN           DigitalOutput<51, ACTIVEHIGHPIN>


#define LED_PIN            PWMOutput<13, ACTIVEHIGHPIN>

#define FAN_PIN            PWMOutput<7, ACTIVEHIGHPIN>

//
// Filament sensor pins (half duplex/3wire spi)
//
#define FILSENSNCS   DigitalOutput<59, ACTIVELOWPIN> // Analog A5

#ifdef REPRAP_DISCOUNT_SMART_CONTROLLER

    #define LCD_PINS_RS 19
    #define LCD_PINS_ENABLE 42
    #define LCD_PINS_D4 18
    #define LCD_PINS_D5 38
    #define LCD_PINS_D6 41
    #define LCD_PINS_D7 40

    #define BEEPER 44

    #define BTN_EN1 11
    #define BTN_EN2 12
    #define BTN_ENC 43
#endif

