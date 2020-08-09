#if 0
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

#include <string.h>
#include <stddef.h>

#if defined(AVR)
    #include <avr/eeprom.h>
#endif

#include "eepromSettings.h"
#include "serialport.h"
#include "Configuration.h"
#include "ddcommands.h"
#include "ddserial.h"
#include "ddcommands.h"

#define EEPROM_OFFSET ((uint8_t*)100)
// #define EEPROM_VERSION "V11"
#define DEFAULT_PRINTER_NAME "UM2-generic"

void _EEPROM_readData(uint8_t * &pos, uint8_t* value, uint8_t size)
{
    do
    {
        *value = eeprom_read_byte(pos);
        pos++;
        value++;
    }while(--size);
}
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))


#ifndef eeprom_read_float
//Arduino IDE compatibility, lacks the eeprom_read_float function
float inline eeprom_read_float(float* addr)
{
    union { uint32_t i; float f; } n;
    n.i = eeprom_read_dword((uint32_t*)addr);
    return n.f;
}
void inline eeprom_write_float(float* addr, float f)
{
    union { uint32_t i; float f; } n;
    n.f = f;
    eeprom_write_dword((uint32_t*)addr, n.i);
}
#endif

void getEepromSettings(EepromSettings &es) {

    eeprom_read_block(&es, (void*)EEPROM_OFFSET, sizeof(EepromSettings));

    /*
    if (strncmp(EEPROM_VERSION, es.stored_ver, 3) == 0) {

        // Eeprom version number matches
        return;
    }
    else
    {
        defaultEepromSettings(es);
    }
    */
}


#if 0
void dumpEepromSettings(const char* prefix) {

    EepromSettings es;

    getEepromSettings(es);

    txBuffer.sendResponseStart(CmdGetEepromSettings);
    txBuffer.sendResponseValue((uint8_t*)es.add_homeing, sizeof(es.add_homeing));
    txBuffer.sendResponseValue(es.Kp);
    txBuffer.sendResponseValue(es.Ki);
    txBuffer.sendResponseValue(es.Kd);
    txBuffer.sendResponseEnd();

}

uint8_t writeEepromFloat(char *valueName, uint8_t len, float value) {
    
    if (strncmp("add_homeing_z", valueName, len) == 0) {

        eeprom_write_float(
            (float*)(EEPROM_OFFSET + offsetof(struct EepromSettings, add_homeing) + (2 * sizeof(float))),
            value);
        return RespOK;
    }
    else if (strncmp("Kp", valueName, len) == 0) {

        eeprom_write_float(
            (float*)(EEPROM_OFFSET + offsetof(struct EepromSettings, Kp)), value);
        return RespOK;
    }
    else if (strncmp("Ki", valueName, len) == 0) {

        eeprom_write_float(
            (float*)(EEPROM_OFFSET + offsetof(struct EepromSettings, Ki)), value);
        return RespOK;
    }
    else if (strncmp("Kd", valueName, len) == 0) {

        eeprom_write_float(
            (float*)(EEPROM_OFFSET + offsetof(struct EepromSettings, Kd)), value);
        return RespOK;
    }
    else {
        return RespInvalidArgument;
    }
}

#endif
#endif


