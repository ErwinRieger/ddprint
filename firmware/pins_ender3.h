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
// Rumba board
//

// Note: Pin nubers are the logical/mapped arduino pins, not the physical chip pins.

#define X_STEP_PIN              FastDigitalOutput<PORTADDR(PORTD), 7, ACTIVEHIGHPIN>  /* Arduino pin 15, PD7 */
// Active state is forward direction (positive)
#define X_DIR_PIN               FastDigitalOutput<PORTADDR(PORTC), 5, ACTIVELOWPIN>  /* Arduino pin 21, PC5 */
#define X_STOP_PIN              DigitalInput<18, ACTIVEHIGHPIN>  // todo: speedup
#define X_ENABLE_PIN            DigitalOutput<14, ACTIVELOWPIN>

#define Y_STEP_PIN              FastDigitalOutput<PORTADDR(PORTC), 6, ACTIVEHIGHPIN>  /* Arduino pin 22, PC6 */
// Active state is forward direction (positive)
#define Y_DIR_PIN               FastDigitalOutput<PORTADDR(PORTC), 7, ACTIVELOWPIN>  /* Arduino pin 23, PC7 */
#define Y_STOP_PIN              DigitalInput<19, ACTIVEHIGHPIN>  // todo: speedup
#define Y_ENABLE_PIN            DigitalOutput<14, ACTIVELOWPIN>

#define Z_STEP_PIN              FastDigitalOutput<PORTADDR(PORTB), 3, ACTIVEHIGHPIN>  /* Arduino pin 3, PB3 */
// Active state is forward direction (positive)
#define Z_DIR_PIN               FastDigitalOutput<PORTADDR(PORTB), 2, ACTIVELOWPIN>  /* Arduino pin 2, PB2 */
#define Z_STOP_PIN              DigitalInput<20, ACTIVEHIGHPIN>  // todo: speedup
#define Z_ENABLE_PIN            DigitalOutput<26, ACTIVELOWPIN>

#define E0_STEP_PIN             FastDigitalOutput<PORTADDR(PORTB), 1, ACTIVEHIGHPIN>  /* Arduino pin 1, PB1 */
// Active state is forward direction (positive)
#define E0_DIR_PIN              FastDigitalOutput<PORTADDR(PORTB), 0, ACTIVELOWPIN>  /* Arduino pin 0, PB0 */
#define E0_ENABLE_PIN           DigitalOutput<14, ACTIVELOWPIN>

#define HEATER_BED_PIN          DigitalOutput<12, ACTIVEHIGHPIN>
#define HEATER_0_PIN            PWMOutput<13, ACTIVEHIGHPIN> // D5

#define TEMP_BED_PIN            6 /* digital 30 */
#define TEMP_0_PIN              7 /* digital 31 */

// SPI
// Chip select SDCard
#define SDSS                    DigitalOutput<31, ACTIVELOWPIN> 
#define SCK_PIN                 DigitalOutput<SCK, ACTIVEHIGHPIN> 
#define MOSI_PIN                DigitalOutput<MOSI, ACTIVEHIGHPIN> 

// #define LED_PIN                 PWMOutput<8, ACTIVEHIGHPIN>

#define FAN_PIN                 PWMOutput<4, ACTIVEHIGHPIN> // B4

//
// Filament sensor chip select pin (half duplex/3wire spi)
//
#define FILSENSNCS              DigitalOutput<27, ACTIVELOWPIN> // CPU-pin 33 *ADC4/PCINT4/PA4*


