
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

#define USEExtrusionRateTable

#define TIMER10MS 10
#define TIMER100MS 100

// #define COLDEXTRUSION 1
// #define COLDMovement 1

#if MOTHERBOARD == 1
    //
    // Ultimaker UM2
    //
    // Pt100 and INA826 amplifier, ADC resolution 1024
    #define TempCircuit 2
    // Use Bourns ems22a Rotary Encoder as a flowrate sensor
    #define BournsEMS22AFS 1

    // #define COLDEXTRUSION 1
    // #define COLDMovement 1

    // Use filamentsensor even if COLDEXTRUSION is defined, at least for initializing it.
    // #define RUNFILAMENTSENSOR 1
#elif MOTHERBOARD == 2
    //
    // Ramps
    //
    // Thermistor in series with 4.7k pullup.
    #define TempCircuit 1
    #define REPRAP_DISCOUNT_SMART_CONTROLLER 1
#elif MOTHERBOARD == 3
    //
    // Jennyprinter
    //
    // Pt100 and INA826 amplifier, ADC resolution 4096
    #define TempCircuit 3
    // Use Bourns ems22a Rotary Encoder as a flowrate sensor
    #define BournsEMS22AFS 1
    #define STEPPER_MINPULSE 2 /* µS */
    // #define RUNFILAMENTSENSOR 1
#elif MOTHERBOARD == 4
    //
    // Rumba
    //
    // Thermistor in series with 4.7k pullup.
    #define TempCircuit 1
    #define REPRAP_DISCOUNT_SMART_CONTROLLER 1

    // debug
    #define COLDEXTRUSION 1
#elif MOTHERBOARD == 5
    //
    // Ender 3, ender 5, atmega1284p
    //
    // Epcos 100k thermistor in series with 4.7k pullup.
    #define TempCircuit 1
    // Use Bourns ems22a Rotary Encoder as a flowrate sensor
    #define BournsEMS22AFS 1
    // Stepper isr running on timer3 instead of timer1
    #define StepperOnTimer3 1 
#else
    #error Unknown MOTHERBOARD in config.h
#endif

// Dont use flowrate sensor if COLDEXTRUSION is enabled.
#if defined(BournsEMS22AFS)
    #if ! defined(COLDEXTRUSION)
        #define HASFILAMENTSENSOR
    #endif
#endif





