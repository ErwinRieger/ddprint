/*
* This file is part of ddprint - a direct drive 3D printer firmware.
* 
* Copyright 2020 erwin.rieger@ibrieger.de
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
// ATMega 2560
//

#define SERIAL_TX_DR_EMPTY() ( (UCSR0A) & (1 << UDRE0) )
#define SERIAL_TX_COMPLETE() ( true )
#define SERIAL_TX_DR_PUTC(c) ( UDR0 = c )

// #define SET_OUTPUT_PWM(pin, activeLow)  SET_OUTPUT(pin)

// #define PWM_WRITE(p, v) analogWrite(p, v)

#define HAL_READ_ANALOG(pin) analogRead(pin)

#define CLI()   cli()
#define SEI()   sei()

#define WDT_ENABLE() wdt_enable(WDTO_4S) /* Timeout 4 seconds */
#define WDT_RESET() wdt_reset()






