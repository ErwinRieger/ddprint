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

#include "Protothread.h"

class TempControl: public Protothread
{
    unsigned long raw_temp_0_value;
    unsigned long raw_temp_bed_value;

    // PID values from eeprom
    float Kp;
    float Ki;
    float Kd;

    // Timestamp of last pid computation
    unsigned long lastPidCompute;

    float temp_iState;
    float temp_dState;

    public:
        TempControl(): raw_temp_0_value(0), raw_temp_bed_value(0) {};
        void init();
        virtual bool Run();
        void heater();
};

extern TempControl tempControl;
