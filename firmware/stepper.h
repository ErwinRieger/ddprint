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

/*
  stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <Arduino.h>

#include "pins.h"
#include "move.h"
#include "mdebug.h"

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)

#define ENABLE_STEPPER1_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1B)
#define DISABLE_STEPPER1_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1B)
#define STEPPER1_DRIVER_INTERRUPT_ENABLED() (TIMSK1 & (1<<OCIE1B))

#if MOTOR_CURRENT_PWM_XY_PIN > -1
extern const int motor_current_setting[3];
#endif

#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
// extern bool abort_on_endstop_hit;
#endif

extern volatile long current_pos_steps[NUM_AXIS];

// Initialize and start the stepper motor subsystem
void st_init();

template<typename MOVE>
void st_set_position_steps(long value);

template<>
void st_set_position_steps<XMove>(long value);
template<>
void st_set_position_steps<YMove>(long value);
template<>
void st_set_position_steps<ZMove>(long value);
template<>
void st_set_position_steps<EMove>(long value);

template<typename MOVE>
long st_get_position_steps();

template<>
long st_get_position_steps<XMove>();
template<>
long st_get_position_steps<YMove>();
template<>
long st_get_position_steps<ZMove>();
template<>
long st_get_position_steps<EMove>();

//
// Three different positions:
// * target pos: temp. position where we want to go to
// * planned_current_pos_steps: position where the print head is or where it should move to
// * current_pos_steps: position where the print head really is now
//   If all planned moves are done, then current_pos_steps == (last planned_current_pos_steps).
//   They are also synchronized by homing operations.
//

void digipot_init();
void digipot_current(uint8_t driver, int current);

template<typename MOVE>
void st_inc_current_pos_steps();

template<>
inline void st_inc_current_pos_steps<XMove>() {
    current_pos_steps[X_AXIS] ++;
}
template<>
inline void st_inc_current_pos_steps<YMove>() {
    current_pos_steps[Y_AXIS] ++;
}
template<>
inline void st_inc_current_pos_steps<ZMove>() {
    current_pos_steps[Z_AXIS] ++;
}
template<>
inline void st_inc_current_pos_steps<EMove>() {
    current_pos_steps[E_AXIS] ++;
}

template<typename MOVE>
void st_dec_current_pos_steps();

template<>
inline void st_dec_current_pos_steps<XMove>() {
    current_pos_steps[X_AXIS] --;
}
template<>
inline void st_dec_current_pos_steps<YMove>() {
    current_pos_steps[Y_AXIS] --;
}
template<>
inline void st_dec_current_pos_steps<ZMove>() {
    current_pos_steps[Z_AXIS] --;
}
template<>
inline void st_dec_current_pos_steps<EMove>() {
    current_pos_steps[E_AXIS] --;
}

template<typename MOVE>
inline void st_set_direction(uint8_t dirbits) {

    if (dirbits & st_get_move_bit_mask<MOVE>())
        st_write_dir_pin<MOVE>(! st_get_invert_dir<MOVE>());
    else
        st_write_dir_pin<MOVE>(  st_get_invert_dir<MOVE>());
}

#define X_ENDSTOP_PRESSED (READ(X_STOP_PIN) != X_ENDSTOPS_INVERTING)
#define Y_ENDSTOP_PRESSED (READ(Y_STOP_PIN) != Y_ENDSTOPS_INVERTING)
#define Z_ENDSTOP_PRESSED (READ(Z_STOP_PIN) != Z_ENDSTOPS_INVERTING)

template<typename MOVE>
bool st_endstop_pressed(bool);

template<>
inline bool st_endstop_pressed<XMove>(bool forward) {

    static uint8_t nPresses = 0;

    if (forward) {
        #if X_HOME_DIR > 0
            if (X_ENDSTOP_PRESSED) {
                if (nPresses++ > (AXIS_STEPS_PER_MM_X / 10))
                    return true;
                return false;
            }
        #endif
    }
    else {
        #if X_HOME_DIR < 0
            if (X_ENDSTOP_PRESSED) {
                if (nPresses++ > (AXIS_STEPS_PER_MM_X / 10))
                    return true;
                return false;
            }
        #endif
    }

    nPresses = 0;
    return false;
}

template<>
inline bool st_endstop_pressed<YMove>(bool forward) {

    static uint8_t nPresses = 0;

    if (forward) {
        #if Y_HOME_DIR > 0
            if (Y_ENDSTOP_PRESSED) {
                if (nPresses++ > (AXIS_STEPS_PER_MM_Y / 10))
                    return true;
                return false;
            }
        #endif
    }
    else {
        #if Y_HOME_DIR < 0
            if (Y_ENDSTOP_PRESSED) {
                if (nPresses++ > (AXIS_STEPS_PER_MM_Y / 10))
                    return true;
                return false;
            }
        #endif
    }

    nPresses = 0;
    return false;
}

template<>
inline bool st_endstop_pressed<ZMove>(bool forward) {

    static uint8_t nPresses = 0;

    if (forward) {
        #if Z_HOME_DIR > 0
            if (Z_ENDSTOP_PRESSED) {
                if (nPresses++ > (AXIS_STEPS_PER_MM_Z / 10))
                    return true;
                return false;
            }
        #endif
    }
    else {
        #if Z_HOME_DIR < 0
            if (Z_ENDSTOP_PRESSED) {
                if (nPresses++ > (AXIS_STEPS_PER_MM_Z / 10))
                    return true;
                return false;
            }
        #endif
    }

    nPresses = 0;
    return false;
}

template<typename MOVE>
bool st_endstop_released(bool);

template<>
inline bool st_endstop_released<XMove>(bool forward) {

    static uint8_t nRelease = 0;

    if (forward) {
        #if X_HOME_DIR < 0
            if (! X_ENDSTOP_PRESSED) {
                if (nRelease++ > (AXIS_STEPS_PER_MM_X / 10))
                    return true;
                return false;
            }
        #endif
    }
    else {
        #if X_HOME_DIR > 0
            if (! X_ENDSTOP_PRESSED) {
                if (nRelease++ > (AXIS_STEPS_PER_MM_X / 10))
                    return true;
                return false;
            }
        #endif
    }

    nRelease = 0;
    return false;
}

template<>
inline bool st_endstop_released<YMove>(bool forward) {

    static uint8_t nRelease = 0;

    if (forward) {
        #if Y_HOME_DIR < 0
            if (! Y_ENDSTOP_PRESSED) {
                if (nRelease++ > (AXIS_STEPS_PER_MM_Y / 10))
                    return true;
                return false;
            }
        #endif
    }
    else {
        #if Y_HOME_DIR > 0
            if (! Y_ENDSTOP_PRESSED) {
                if (nRelease++ > (AXIS_STEPS_PER_MM_Y / 10))
                    return true;
                return false;
            }
        #endif
    }

    nRelease = 0;
    return false;
}

template<>
inline bool st_endstop_released<ZMove>(bool forward) {

    static uint8_t nRelease = 0;

    if (forward) {
        #if Z_HOME_DIR < 0
            if (! Z_ENDSTOP_PRESSED) {
                if (nRelease++ > (AXIS_STEPS_PER_MM_Z / 10))
                    return true;
                return false;
            }
        #endif
    }
    else {
        #if Z_HOME_DIR > 0
            if (! Z_ENDSTOP_PRESSED) {
                if (nRelease++ > (AXIS_STEPS_PER_MM_Z / 10))
                    return true;
                return false;
            }
        #endif
    }

    nRelease = 0;
    return false;
}

template<typename MOVE>
inline void st_step_motor(uint8_t stepBits, uint8_t dirbits) {

    if (stepBits & st_get_move_bit_mask<MOVE>()) {

        st_write_step_pin<MOVE>(! st_get_invert_step_pin<MOVE>());

        if (dirbits & st_get_move_bit_mask<MOVE>())
            st_inc_current_pos_steps<MOVE>();
        else
            st_dec_current_pos_steps<MOVE>();

        st_write_step_pin<MOVE>(  st_get_invert_step_pin<MOVE>());
    }
}

//
// Like st_step_motor, but check endstops
//
template<typename MOVE>
inline void st_step_motor_es(uint8_t stepBits, uint8_t dirbits) {

    if (stepBits & st_get_move_bit_mask<MOVE>()) {

        bool forward = dirbits & st_get_move_bit_mask<MOVE>();

        bool endStop = st_endstop_pressed<MOVE>(forward);

        if (endStop) {
            // if (forward)
                // SERIAL_ECHOLN("Endstop hit forward");
            // else
                // SERIAL_ECHOLN("Endstop hit backward");
            DISABLE_STEPPER1_DRIVER_INTERRUPT();
            return;
        }

        endStop = st_endstop_released<MOVE>(forward);

        if (endStop) {
            // if (forward)
                // SERIAL_ECHOLN("Endstop released forward");
            // else
                // SERIAL_ECHOLN("Endstop released backward");
            DISABLE_STEPPER1_DRIVER_INTERRUPT();
            return;
        }

        st_write_step_pin<MOVE>(! st_get_invert_step_pin<MOVE>());

        if (forward)
            st_inc_current_pos_steps<MOVE>();
        else
            st_dec_current_pos_steps<MOVE>();

        st_write_step_pin<MOVE>(  st_get_invert_step_pin<MOVE>());
    }
}



/*
#
# Konkurrierender zugriff auf den stepbuffer:
#
# Mainthread:
#
#   * fügt steps in den buffer ein -> kein problem wenn head pointer erst danach gesetzt wird
#   * operiert auf head pointer
#   * liest tail pointer (um buffer fill festzustellen)
#   * setzt neuen headpointer -> problematisch, genau in diesem moment könnte das mainprogramm
#       von einem irq unterbrochen werden.
#
# IRQ:
#
#   * operiert auf tail pointer
#   * liest head pointer (um buffer fill festzustellen)
#
# xxx use volatile BYTE variables for head and tail.
#
*/

/*
struct StepBlock {

    // Bit 0-4: Direction bits, F
    // Bits 5-6: size of entry, 0,1 or 3, LL
    // Bit 7: set-direction-flag, DDDDD
    // FLLDDDDD
    uint8_t cmdDirBits;

    uint8_t stepBits;   // 5 bits used

    [uint8_t|uint16_t] timer;

    [uint8_t timerLoop;]
};
*/

// Size of step buffer in bytes, must be a power of 2:
#define StepBufferLen  2048
#define StepBufferMask  (StepBufferLen - 1)

#define CMDLEN3 (0 << 5)
#define CMDLEN4 (1 << 5)
#define CMDLEN5 (2 << 5)

#define GETCMDLEN(v) (v & (3 << 5))

class StepBuffer {
    private:
        uint8_t stepBuffer[StepBufferLen];

        uint16_t head, tail;

        uint8_t timerLoop;

    public:

        // int32_t timeSum;

        StepBuffer() {
            head = tail = 0;
            timerLoop = 0;
        };

        FWINLINE uint16_t byteSize() {
            return (StepBufferLen + head - tail) & StepBufferMask;
            // return ((uint16_t)StepBufferLen + head) - tail;
        }

        FWINLINE bool empty() {
            // return size() == 0;
            return head == tail;
        }

        FWINLINE bool full() {
            return byteSize() >= (StepBufferLen-10);
        }

        FWINLINE void push3(uint8_t cmdDir, uint8_t steps, uint8_t timer) {

            simassert(byteSize()+3 < StepBufferLen);

            stepBuffer[head] = cmdDir | CMDLEN3;
            head = (head+1) & StepBufferMask;

            stepBuffer[head] = steps;
            head = (head+1) & StepBufferMask;

            stepBuffer[head] = timer;
            head = (head+1) & StepBufferMask;
        }

        FWINLINE void push4(uint8_t cmdDir, uint8_t steps, uint16_t timer) {

            simassert(byteSize()+4 < StepBufferLen);

            stepBuffer[head] = cmdDir | CMDLEN4;
            head = (head+1) & StepBufferMask;

            stepBuffer[head] = steps;
            head = (head+1) & StepBufferMask;

            stepBuffer[head] = timer;
            head = (head+1) & StepBufferMask;

            stepBuffer[head] = timer >> 8;
            head = (head+1) & StepBufferMask;
        }

        void push5(uint8_t cmdDir, uint8_t steps, uint16_t timer, uint8_t timerLoop) {

            simassert(byteSize()+5 < StepBufferLen);

            stepBuffer[head] = cmdDir | CMDLEN5;
            head = (head+1) & StepBufferMask;

            stepBuffer[head] = steps;
            head = (head+1) & StepBufferMask;

            stepBuffer[head] = timer;
            head = (head+1) & StepBufferMask;

            stepBuffer[head] = timer >> 8;
            head = (head+1) & StepBufferMask;

            stepBuffer[head] = timerLoop;
            head = (head+1) & StepBufferMask;
        }

        // Get cmdDir
        FWINLINE uint8_t * peek80() {
            simassert(byteSize() >= 3);
            return stepBuffer+tail;
        }
        // Get stepBits
        FWINLINE uint8_t * peek81() {
            simassert(byteSize() >= 3);
            return stepBuffer+((tail+1) & StepBufferMask);
        }
        // Get timer, 8 bit
        FWINLINE uint8_t *peek82() {
            simassert(byteSize() >= 3);
            return stepBuffer+((tail+2) & StepBufferMask);
        }
        // Get timer, high 8 bit
        FWINLINE uint8_t *peek83() {
            simassert(byteSize() >= 4);
            return stepBuffer+((tail+3) & StepBufferMask);
        }

        // // Get timer, 16 bit
        // FWINLINE uint16_t peek162() {
            // simassert(byteSize() >= 4);
            // uint16_t t  = stepBuffer[(tail+2) & StepBufferMask];
                     // t |= stepBuffer[(tail+3) & StepBufferMask] << 8;
            // return t;
        // }

        // Get timerLoop, 8 bit
        uint8_t * peek84() {
            simassert(byteSize() >= 5);
            return stepBuffer+((tail+4) & StepBufferMask);
        }

        FWINLINE void pop3() {
            tail = (tail+3) & StepBufferMask;
        }
        FWINLINE void pop4() {
            tail = (tail+4) & StepBufferMask;
        }
        void pop5() {
            tail = (tail+5) & StepBufferMask;
        }

        void flush() {
            head = tail = 0;
        }

        FWINLINE void runMoveSteps() {

            if (timerLoop) {
                timerLoop --;
                OCR1A = 0xffff;
            }
            // XXX make stepbuffer.pop return pointer, return null if emtpy -> save one function call
            else if (byteSize()) {

                uint8_t cmdDir = *peek80();

                // * Set direction 
                // * Step the motors
                // * Update step-coordinates (current_pos_steps)
                // * Set new timer value

                if (cmdDir & 0x80) {

                    // Set direction bits
                    st_set_direction<XMove>(cmdDir); 
                    st_set_direction<YMove>(cmdDir); 
                    st_set_direction<ZMove>(cmdDir); 
                    st_set_direction<EMove>(cmdDir); 
                }

                uint8_t stepBits = *peek81();

                st_step_motor<XMove>(stepBits, cmdDir); 
                st_step_motor<YMove>(stepBits, cmdDir); 
                st_step_motor<ZMove>(stepBits, cmdDir); 
                st_step_motor<EMove>(stepBits, cmdDir); 

                switch (GETCMDLEN(cmdDir)) {
                    case  CMDLEN3:
                        OCR1A = *peek82();
                        pop3();
                        break;
                    case CMDLEN4:
                        OCR1A = *peek82() | (*peek83() << 8);
                        pop4();
                        break;
                    case CMDLEN5:
                        OCR1A = *peek82() | (*peek83() << 8);
                        timerLoop = *peek84();
                        pop5();
                        break;
                    default:
                        massert(0);
                }
            }
            else {
                // Empty buffer, nothing to step
                OCR1A=2000; // 1kHz.
            }
        }

        FWINLINE void runHomingSteps() {

            if (byteSize()) {

                uint8_t cmdDir = *peek80();

                // * Set direction 
                // * Check endstops
                // * Step the motors
                // * Update step-coordinates (current_pos_steps)
                // * Set new timer value

                if (cmdDir & 0x80) {

                    // Set direction bits
                    st_set_direction<XMove>(cmdDir); 
                    st_set_direction<YMove>(cmdDir); 
                    st_set_direction<ZMove>(cmdDir); 
                    // st_set_direction<EMove>(cmdDir); 
                }

                uint8_t stepBits = *peek81();

                st_step_motor_es<XMove>(stepBits, cmdDir); 
                st_step_motor_es<YMove>(stepBits, cmdDir); 
                st_step_motor_es<ZMove>(stepBits, cmdDir); 

                switch (GETCMDLEN(cmdDir)) {
                    case  CMDLEN3:
                        OCR1A = OCR1B = *peek82();
                        timerLoop = 0;
                        pop3();
                        break;
                    case CMDLEN4:
                        OCR1A = OCR1B = *peek82() | (*peek83() << 8);
                        timerLoop = 0;
                        pop4();
                        break;
                    case CMDLEN5:
                        massert(0);
                        // OCR1A = OCR1B = 
                        // timerLoop = 0;
                        // pop5();
                        break;
                    default:
                        massert(0);
                }
            }
            else {
                OCR1A = OCR1B=2000; // 1kHz.
            }
        }
};

extern StepBuffer stepBuffer;

#include "simulator/stepperSim.h"


