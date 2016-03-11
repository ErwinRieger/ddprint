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
#include "ddcommands.h"
#include "ddserial.h"
#include "ddcommands.h"

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
    // SERIAL_ECHOPGM("Res:'");
    // SERIAL_ECHO(stored_ver);
    // SERIAL_ECHOLN("'");
    txBuffer.sendResponseStart(CmdGetEepromVersion, 5);
    txBuffer.pushCharChecksum(RespOK);
    txBuffer.sendResponseValue(stored_ver, 3);
    txBuffer.sendResponseEnd();
}

template <typename ElemType>
void dumpArray(ElemType *array, uint8_t len) {
    SERIAL_ECHOPGM("(");
    for (uint8_t i=0; i<len; i++) {
        SERIAL_ECHO(array[i]);
        SERIAL_ECHOPGM(",");
    }
    SERIAL_ECHOPGM(")");
}

// void getEepromSettings(EepromSettings &es) {
    // eeprom_read_block(&es, EEPROM_OFFSET, sizeof(EepromSettings));
// }

void eeDumpValue(float f, uint8_t) {
    SERIAL_ECHO(f);
}

void eeDumpValue(int16_t l, uint8_t) {
    SERIAL_ECHO(l);
}

void eeDumpValue(int32_t l, uint8_t) {
    SERIAL_ECHO(l);
}

void eeDumpValue(float *array, uint8_t len) {
    SERIAL_ECHOPGM("(");
    for (uint8_t i=0; i<len/sizeof(float); i++) {
        SERIAL_ECHO(array[i]);
        SERIAL_ECHOPGM(",");
    }
    SERIAL_ECHOPGM(")");
}

void eeDumpValue(int32_t *array, uint8_t len) {
    SERIAL_ECHOPGM("(");
    for (uint8_t i=0; i<len/sizeof(int32_t); i++) {
        SERIAL_ECHO(array[i]);
        SERIAL_ECHOPGM(",");
    }
    SERIAL_ECHOPGM(")");
}

#define eeDump(s, member) {  \
  SERIAL_ECHOPGM("'" #member "':"); \
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

void dumpEepromSettings(const char* prefix) {

    EepromSettings es;

    getEepromSettings(es);

    txBuffer.sendResponseStart(CmdGetEepromSettings, sizeof(es.add_homeing));
    txBuffer.sendResponseValue((uint8_t*)es.add_homeing, sizeof(es.add_homeing));
    txBuffer.sendResponseEnd();

#if 0
    // Eeprom version number matches

        SERIAL_ECHO(prefix);
        SERIAL_ECHOPGM("{");

        eeDump(es, axis_steps_per_unit);

        SERIAL_ECHOPGM(",");

        eeDump(es, max_feedrate);

        SERIAL_ECHOPGM(",");

        eeDump(es, max_acceleration_units_per_sq_second);

        SERIAL_ECHOPGM(",");

        // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
		// reset_acceleration_rates();

        eeDump(es, acceleration);

        SERIAL_ECHOPGM(",");

        eeDump(es, retract_acceleration);

        SERIAL_ECHOPGM(",");

        eeDump(es, minimumfeedrate);

        SERIAL_ECHOPGM(",");

        eeDump(es, mintravelfeedrate);

        SERIAL_ECHOPGM(",");

        eeDump(es, minsegmenttime);

        SERIAL_ECHOPGM(",");

        eeDump(es, max_xy_jerk);

        SERIAL_ECHOPGM(",");

        eeDump(es, max_z_jerk);

        SERIAL_ECHOPGM(",");

        eeDump(es, max_e_jerk);

        SERIAL_ECHOPGM(",");

        eeDump(es, add_homeing);

        SERIAL_ECHOPGM(",");

        eeDump(es, plaPreheatHotendTemp);

        SERIAL_ECHOPGM(",");

        eeDump(es, plaPreheatHPBTemp);

        SERIAL_ECHOPGM(",");

        eeDump(es, plaPreheatFanSpeed);

        SERIAL_ECHOPGM(",");

        eeDump(es, absPreheatHotendTemp);

        SERIAL_ECHOPGM(",");

        eeDump(es, absPreheatHPBTemp);

        SERIAL_ECHOPGM(",");

        eeDump(es, absPreheatFanSpeed);

        SERIAL_ECHOPGM(",");

        eeDump(es, Kp);

        SERIAL_ECHOPGM(",");

        eeDump(es, Ki);

        SERIAL_ECHOPGM(",");

        eeDump(es, Kd);
#endif
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

uint8_t writeEepromFloat(char *valueName, uint8_t len, float value) {
    
    if (strncmp("add_homeing_z", valueName, len) == 0) {

        eeprom_write_float(
            (float*)(EEPROM_OFFSET + offsetof(struct EepromSettings, add_homeing) + (2 * sizeof(float))),
            value);
        // SERIAL_PROTOCOLLNPGM(MSG_OK);
        return RespOK;
    }
    else if (strncmp("Kp", valueName, len) == 0) {

        eeprom_write_float(
            (float*)(EEPROM_OFFSET + offsetof(struct EepromSettings, Kp)), value);
        // SERIAL_PROTOCOLLNPGM(MSG_OK);
        return RespOK;
    }
    else if (strncmp("Ki", valueName, len) == 0) {

        eeprom_write_float(
            (float*)(EEPROM_OFFSET + offsetof(struct EepromSettings, Ki)), value);
        // SERIAL_PROTOCOLLNPGM(MSG_OK);
        return RespOK;
    }
    else if (strncmp("Kd", valueName, len) == 0) {

        eeprom_write_float(
            (float*)(EEPROM_OFFSET + offsetof(struct EepromSettings, Kd)), value);
        // SERIAL_PROTOCOLLNPGM(MSG_OK);
        return RespOK;
    }
    else {
        // valueName[len] = '\0';
        // SERIAL_ECHOPGM("Error: unknown value name in writeEepromFloat: ");
        // SERIAL_ECHOLN(valueName);
        return RespInvalidArgument;
    }
}




