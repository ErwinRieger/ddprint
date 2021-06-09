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

#include <Arduino.h>

void mAssert(uint16_t line, const char* file);
void killMessage(uint8_t errorCode, uint8_t errorParam /* , const char *msg = "" */);
void killMessage(uint8_t errorCode, uint8_t errorParam1, uint8_t errorParam2 /* , const char *msg = "" */);

// #define PID_DEBUG // Sends hotend pid values as RespUnsolicitedMsg, type PidDebug

//
// Add heavy and time consuming debugging
//
// #define HEAVYDEBUG 1

// Debug usb-serail download to mass-storage
#define HEAVYDEBUGRX 1

#if defined(__amd64__)

    #include <assert.h>
    #define massert assert

    // Assertion that is only active in simulation
    #define simassert assert

    #define FWINLINE  /* */

    #define STD std::

#else

    void disable_heater();
    void st_disableSteppers();

    #define  massert(expr) { \
        if (!(expr)) { mAssert(__LINE__, __FILE__); } }

    // Assertion that is only active in simulation
    #define simassert(x) 

    // To be able to compile a debug version without inlining
    #define FWINLINE inline

    #define printf ERROR_PRINTF_USED

#endif

#if defined(AVR)

                #define STD

extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

                //
                // Get free memory, from https://playground.arduino.cc/Code/AvailableMemory/
                //
                inline uint32_t freeRam () {
                int free_memory;

                    if((int)__brkval == 0)
                        free_memory = ((int)&free_memory) - ((int)&__heap_start);
                    else
                        free_memory = ((int)&free_memory) - ((int)__brkval);

                    return free_memory;
                }

                // Process stats
                #define DEBUGPROCSTAT 1
                // Mass storage timing
                #define DEBUGREADWRITE 1
#elif defined(__arm__)

                #define STD std::

                //
                // Get free memory
                //
                inline uint32_t freeRam () {
                    extern char _lm_heap_start;
                    extern char _lm_heap_end;
                    return (&_lm_heap_end) - (&_lm_heap_start);
                }

                // Process stats
                #define DEBUGPROCSTAT 1
                // Mass storage timing
                #define DEBUGREADWRITE 1
#endif

#if defined(DEBUGPROCSTAT) || defined(DEBUGREADWRITE)
    typedef struct TaskTiming {
        uint32_t taskStart;
        uint32_t ncalls;
        uint32_t sumcall;
        uint32_t longest;
    } TaskTiming;
    #define TaskStart(timings, looptask) { timings[looptask].taskStart = millis(); }
    #define GetTaskStart(timings, looptask) (timings[looptask].taskStart) 
    #define GetTaskDuration(timings, looptask) (millis() - timings[looptask].taskStart)
    #define TaskEnd(timings, looptask) { uint32_t taskDuration = millis() - timings[looptask].taskStart; timings[looptask].ncalls += 1; timings[looptask].sumcall += taskDuration; if (taskDuration > timings[looptask].longest) timings[looptask].longest = taskDuration; }

#else

    #define TaskStart(timings, looptask)
    #define TaskEnd(timings, looptask) 
#endif








