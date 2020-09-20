
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

// Compile firmware in the PID autoTune version:
// #define PIDAutoTune 1

// Use PMW3360 as a flowrate sensor
// #define PMWFS 1

#define USEExtrusionRateTable

// #define PID_DEBUG // Sends hotend pid values as RespUnsolicitedMsg, type PidDebug

// Limit for the integral term and for pwmSum if heater is
// switched off (target temp == 0) or if setpoint temperature
// could not be reached.
// #define PID_DRIVE_MAX 100000.0
// #define PID_DRIVE_MAX 25000.0

#define TIMER10MS 10
#define TIMER100MS 100

// #define COLDEXTRUSION 1

#if MOTHERBOARD == 1
    //
    // Ultimaker UM2
    //
    // Use Bourns ems22a Rotary Encoder as a flowrate sensor
    #define BournsEMS22AFS 1
#elif MOTHERBOARD == 2
    //
    // Ramps
    //
#elif MOTHERBOARD == 3
    //
    // Jennyprinter
    //
    // Use Bourns ems22a Rotary Encoder as a flowrate sensor
    #define BournsEMS22AFS 1
    // #define STEPPER_MINPULSE 5 /* µS */
    #define STEPPER_MINPULSE 1 /* µS */

    #define UseProcessStats 1
#else
    #error Unknown MOTHERBOARD in config.h
#endif

#if defined(PMWFS) || defined(BournsEMS22AFS)
    #define HASFILAMENTSENSOR
#endif

