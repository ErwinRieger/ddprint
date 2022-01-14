/*
* This file is part of ddprint - a direct drive 3D printer firmware.
* 
* Copyright 2021 erwin.rieger@ibrieger.de
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
// Anycubic i3 mega (s), Trigorilla 0.0.2
//

// Note: Pin nubers are the logical/mapped pins, not the physical chip pins.
// Note: Board design is based on RAMPS.
// Note: Different board versions where fan pins (9 and 44) are swapped, different servo
//       pin logic.

//
// Todo: speed up, use FastDigitalOutput like in pins_um2.h
//
// #define X_STEP_PIN         DigitalOutput<54, ACTIVEHIGHPIN>
#define X_STEP_PIN         FastDigitalOutput<PORTADDR(PORTF), 0, ACTIVEHIGHPIN>  /* Arduino pin 54, PF0 */
// #define X_DIR_PIN          DigitalOutput<55, ACTIVELOWPIN>
#define X_DIR_PIN          FastDigitalOutput<PORTADDR(PORTF), 1, ACTIVELOWPIN>  /* Arduino pin 55, PF1 */
#define X_STOP_PIN         DigitalInput<3, ACTIVELOWPIN> /* 3 */
#define X_ENABLE_PIN       DigitalOutput<38, ACTIVELOWPIN> /* 38 */

/*
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_STOP_PIN         15
*/
// #define Y_STEP_PIN         DigitalOutput<60, ACTIVEHIGHPIN>
#define Y_STEP_PIN         FastDigitalOutput<PORTADDR(PORTF), 6, ACTIVEHIGHPIN>  /* Arduino pin 60, PF6 */
// #define Y_DIR_PIN          DigitalOutput<61, ACTIVELOWPIN>
#define Y_DIR_PIN          FastDigitalOutput<PORTADDR(PORTF), 7, ACTIVELOWPIN>  /* Arduino pin 61, PF7 */
#define Y_STOP_PIN         DigitalInput<42, ACTIVELOWPIN>  // PL7
#define Y_ENABLE_PIN       DigitalOutput<56, ACTIVELOWPIN> // PF2

/*
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_STOP_PIN         19
*/
// #define Z_STEP_PIN              DigitalOutput<46, ACTIVEHIGHPIN>
#define Z_STEP_PIN          FastDigitalOutput<PORTADDR(PORTL), 3, ACTIVEHIGHPIN>  /* Arduino pin 46, PL3 */
// #define Z_DIR_PIN               DigitalOutput<48, ACTIVELOWPIN>
#define Z_DIR_PIN           FastDigitalOutput<PORTADDR(PORTL), 1, ACTIVEHIGHPIN>  /* Arduino pin 48, PL1 */
#define Z_STOP_PIN          DigitalInput<18, ACTIVELOWPIN> // PD3
#define Z_ENABLE_PIN        DigitalOutput<62, ACTIVELOWPIN>
                            
/*
    #define Z2_STEP_PIN        36
    #define Z2_DIR_PIN         34
    #define Z2_ENABLE_PIN      30
*/
#define Z1_STEP_PIN         FastDigitalOutput<PORTADDR(PORTC), 1, ACTIVEHIGHPIN>  /* Arduino pin 36, PC1 */
#define Z1_DIR_PIN          FastDigitalOutput<PORTADDR(PORTC), 3, ACTIVEHIGHPIN>  /* Arduino pin 34, PC3 */
#define Z1_STOP_PIN         DigitalInput<43, ACTIVELOWPIN> // PL6
#define Z1_ENABLE_PIN       DigitalOutput<30, ACTIVELOWPIN> // PC7
                            
/*
    #define E0_STEP_PIN        26
    #define E0_DIR_PIN         28
    #define E0_ENABLE_PIN      24
*/
// #define E0_STEP_PIN             DigitalOutput<26, ACTIVEHIGHPIN>
#define E0_STEP_PIN         FastDigitalOutput<PORTADDR(PORTA), 4, ACTIVEHIGHPIN>  /* Arduino pin 26, PA4 */
                               
// #define E0_DIR_PIN          DigitalOutput<28, ACTIVELOWPIN>
#define E0_DIR_PIN          FastDigitalOutput<PORTADDR(PORTA), 6, ACTIVEHIGHPIN>  /* Arduino pin 28, PA6 */
#define E0_ENABLE_PIN       DigitalOutput<24, ACTIVELOWPIN>

// Bed
#define HEATER_BED_PIN      DigitalOutput<8, ACTIVEHIGHPIN>    // BED
#define TEMP_BED_PIN        14   // ANALOG NUMBERING

// Hotend
#define HEATER_0_PIN        PWMOutput<10, ACTIVEHIGHPIN>   // EXTRUDER 1
#define TEMP_0_PIN          13   // ANALOG NUMBERING

// SPI
// Chip select SDCard
#define SDSS                DigitalOutput<53, ACTIVELOWPIN>
#define SCK_PIN             DigitalOutput<52, ACTIVEHIGHPIN>
// #define MISO_PIN         DigitalInput<50...
#define MOSI_PIN            DigitalOutput<51, ACTIVEHIGHPIN>

// Pin 13 is green (debug) led on mainboard, activelow
// #define LED_PIN            PWMOutput<13, ACTIVEHIGHPIN>

// Parts cooling fan
#define FAN_PIN             PWMOutput<9, ACTIVEHIGHPIN>

// Hotend fan
#define HOTEND_FAN_PIN      DigitalOutput<44, ACTIVEHIGHPIN>

// Additional mainboard fan
#define MB_FAN_PIN          DigitalOutput<7, ACTIVEHIGHPIN>

//
// Filament sensor pins (half duplex/3wire spi)
//
#define FILSENSNCS          DigitalOutput<11, ACTIVELOWPIN> // Servo1















