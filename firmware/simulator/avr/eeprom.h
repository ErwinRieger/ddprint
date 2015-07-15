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

#pragma once

#include <stdint.h>

extern uint8_t eeprom_read_byte (const uint8_t *__p);
extern void eeprom_read_block( void * __dst, const void * __src, size_t __n);

// extern void eeprom_write_float( float * __p, float __value);
extern uint32_t eeprom_read_dword(const uint32_t * __p);
extern void eeprom_write_dword(uint32_t * __p, uint32_t __value);
extern void eeprom_write_block(const void * __src, void * __dst, size_t __n);




