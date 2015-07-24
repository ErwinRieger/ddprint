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
#include <avr/eeprom.h>

#include "eepromSettings.h"
#include "MarlinSerial.h"
#include "Configuration.h"

#define EEPROM_OFFSET ((uint8_t*)100)
#define EEPROM_VERSION "V11"

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


void getEepromVersion() {

    uint8_t * i = EEPROM_OFFSET;
    char stored_ver[4];

    EEPROM_READ_VAR(i, stored_ver); //read stored version
    SERIAL_ECHO("Res:'");
    SERIAL_ECHO(stored_ver);
    SERIAL_ECHOLN("'");
}

template <typename ElemType>
void dumpArray(ElemType *array, uint8_t len) {
    SERIAL_ECHO("(");
    for (uint8_t i=0; i<len; i++) {
        SERIAL_ECHO(array[i]);
        SERIAL_ECHO(",");
    }
    SERIAL_ECHO(")");
}

// void getEepromSettings(EepromSettings &es) {
    // eeprom_read_block(&es, EEPROM_OFFSET, sizeof(EepromSettings));
// }

void eeDumpValue(float f, uint8_t) {
    SERIAL_ECHO(f);
}

void eeDumpValue(long l, uint8_t) {
    SERIAL_ECHO(l);
}

void eeDumpValue(int l, uint8_t) {
    SERIAL_ECHO(l);
}

void eeDumpValue(float *array, uint8_t len) {
    SERIAL_ECHO("(");
    for (uint8_t i=0; i<len/sizeof(float); i++) {
        SERIAL_ECHO(array[i]);
        SERIAL_ECHO(",");
    }
    SERIAL_ECHO(")");
}

void eeDumpValue(long *array, uint8_t len) {
    SERIAL_ECHO("(");
    for (uint8_t i=0; i<len/sizeof(long); i++) {
        SERIAL_ECHO(array[i]);
        SERIAL_ECHO(",");
    }
    SERIAL_ECHO(")");
}

#define eeDump(s, member) {  \
  SERIAL_ECHO("'" #member "':"); \
  eeDumpValue(s . member, sizeof(s . member)); } \

void defaultEepromSettings(EepromSettings &es) {

    SERIAL_ECHOLN("defaultEepromSettings(): initializing eeprom.");

    EepromSettings defaultSettings = {
        /* stored_ver */ EEPROM_VERSION,
        /* axis_steps_per_unit */ { AXIS_STEPS_PER_MM_X, AXIS_STEPS_PER_MM_Y, AXIS_STEPS_PER_MM_Z, AXIS_STEPS_PER_MM_E },
        /* max_feedrate */ DEFAULT_MAX_FEEDRATE,
        /* max_acceleration_units_per_sq_second */ {0, 0, 0, 0}, // XXX unused
        /* acceleration */ DEFAULT_ACCELERATION,
        /* retract_acceleration */ DEFAULT_RETRACT_ACCELERATION,
        /* minimumfeedrate */ DEFAULT_MINIMUMFEEDRATE,
        /* mintravelfeedrate */ DEFAULT_MINTRAVELFEEDRATE,
        /* minsegmenttime */ DEFAULT_MINSEGMENTTIME,
        /* max_xy_jerk */ DEFAULT_XYJERK,
        /* max_z_jerk */ DEFAULT_ZJERK,
        /* max_e_jerk */ DEFAULT_EJERK,
        /* add_homeing */ {0, 0, 0, 0}

        /* plaPreheatHotendTemp */, PLA_PREHEAT_HOTEND_TEMP,
        /* plaPreheatHPBTemp */     PLA_PREHEAT_HPB_TEMP,
        /* plaPreheatFanSpeed */    PLA_PREHEAT_FAN_SPEED,
        /* absPreheatHotendTemp */  ABS_PREHEAT_HOTEND_TEMP,
        /* absPreheatHPBTemp */     ABS_PREHEAT_HPB_TEMP,
        /* absPreheatFanSpeed */    ABS_PREHEAT_FAN_SPEED
#if defined(PIDTEMP)
        /* Kp */,   DEFAULT_Kp,
        /* Ki */    DEFAULT_Ki,
        /* Kd */    DEFAULT_Kd
#endif
    };

    eeprom_write_block(&defaultSettings, (void*)EEPROM_OFFSET, sizeof(EepromSettings));
    es = defaultSettings;
}


void getEepromSettings(EepromSettings &es) {

    eeprom_read_block(&es, (void*)EEPROM_OFFSET, sizeof(EepromSettings));

    if (strncmp(EEPROM_VERSION, es.stored_ver, 3) == 0) {

        // Eeprom version number matches
        return;
    }
    else
    {
        defaultEepromSettings(es);
    }
}

void dumpEepromSettings() {

    EepromSettings es;

    getEepromSettings(es);

    // Eeprom version number matches

        SERIAL_ECHO("Res:{");

        eeDump(es, axis_steps_per_unit);

        SERIAL_ECHO(",");

        eeDump(es, max_feedrate);

        SERIAL_ECHO(",");

        eeDump(es, max_acceleration_units_per_sq_second);

        SERIAL_ECHO(",");

        // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
		// reset_acceleration_rates();

        eeDump(es, acceleration);

        SERIAL_ECHO(",");

        eeDump(es, retract_acceleration);

        SERIAL_ECHO(",");

        eeDump(es, minimumfeedrate);

        SERIAL_ECHO(",");

        eeDump(es, mintravelfeedrate);

        SERIAL_ECHO(",");

        eeDump(es, minsegmenttime);

        SERIAL_ECHO(",");

        eeDump(es, max_xy_jerk);

        SERIAL_ECHO(",");

        eeDump(es, max_z_jerk);

        SERIAL_ECHO(",");

        eeDump(es, max_e_jerk);

        SERIAL_ECHO(",");

        eeDump(es, add_homeing);

        SERIAL_ECHO(",");

        eeDump(es, plaPreheatHotendTemp);

        SERIAL_ECHO(",");

        eeDump(es, plaPreheatHPBTemp);

        SERIAL_ECHO(",");

        eeDump(es, plaPreheatFanSpeed);

        SERIAL_ECHO(",");

        eeDump(es, absPreheatHotendTemp);

        SERIAL_ECHO(",");

        eeDump(es, absPreheatHPBTemp);

        SERIAL_ECHO(",");

        eeDump(es, absPreheatFanSpeed);

        SERIAL_ECHO(",");

        eeDump(es, Kp);

        SERIAL_ECHO(",");

        eeDump(es, Ki);

        SERIAL_ECHO(",");

        eeDump(es, Kd);

#if 0
        // do not need to scale PID values as the values in EEPROM are already scaled
        EEPROM_READ_VAR(i,motor_current_setting);
        #ifdef ENABLE_ULTILCD2
        EEPROM_READ_VAR(i,led_brightness_level);
        EEPROM_READ_VAR(i,led_mode);
        #else
        uint8_t dummyByte;
        EEPROM_READ_VAR(i,dummyByte);
        EEPROM_READ_VAR(i,dummyByte);
        #endif
        EEPROM_READ_VAR(i,retract_length);
        EEPROM_READ_VAR(i,retract_feedrate);

		// Call updatePID (similar to when we have processed M301)
		updatePID();
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("Stored settings retrieved");
#endif

        SERIAL_ECHOLN("}");
}

void writeEepromFloat(char *valueName, uint8_t len, float value) {
    
    if (strncmp("add_homeing_z", valueName, len) == 0) {

        eeprom_write_float(
            (float*)(EEPROM_OFFSET + offsetof(struct EepromSettings, add_homeing) + (2 * sizeof(float))),
            value);
        SERIAL_PROTOCOLLNPGM(MSG_OK);
    }
    else {
        valueName[len] = '\0';
        SERIAL_ECHO("Error: unknown value name in writeEepromFloat: ");
        SERIAL_ECHOLN(valueName);
    }
}




