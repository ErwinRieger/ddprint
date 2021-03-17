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

#if ! defined (DDSim)
    #error this should be included in DDSim mode only
#endif

#include <string.h>
#include <stdint.h>

#define PROGMEM /* nix */

#define strcmp_P strcmp
#define strcpy_P strcpy
#define strstr_P strstr
#define strncmp_P strncmp
#define strcat_P strcat
#define strlen_P strlen

#define  pgm_read_byte(p) (*(p))
#define  PSTR(x) x

typedef char prog_char;
typedef const prog_char * PGM_P;


float pgm_read_float_near(const float *p);
char pgm_read_byte_near(const signed char *p);
unsigned short pgm_read_word_near(const uint8_t *p);

