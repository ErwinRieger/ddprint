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

struct __attribute__((packed)) EepromSettings {

    char stored_ver[4];
    float axis_steps_per_unit[4];
    float max_feedrate[4];
    long max_acceleration_units_per_sq_second[4]; // XXX unused

    float acceleration;
    float retract_acceleration;
    float minimumfeedrate;
    float mintravelfeedrate;
    long minsegmenttime;
    float max_xy_jerk;
    float max_z_jerk;
    float max_e_jerk;

    float add_homeing[4];

    int plaPreheatHotendTemp;
    int plaPreheatHPBTemp;
    int plaPreheatFanSpeed;

    int absPreheatHotendTemp;
    int absPreheatHPBTemp;
    int absPreheatFanSpeed;

    float Kp;
    float Ki;
    float Kd;

    // ...
};

void initEeprom();

void getEepromVersion();

void getEepromSettings(EepromSettings &es);

void defaultEepromSettings(EepromSettings &es);

void dumpEepromSettings();

void writeEepromFloat(char *valueName, uint8_t len, float v);

