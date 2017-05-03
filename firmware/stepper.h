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

#include "ddmacro.h"
#include "pins.h"
#include "move.h"
#include "mdebug.h"

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)

#define ENABLE_STEPPER1_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1B)
#define DISABLE_STEPPER1_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1B)
#define STEPPER1_DRIVER_INTERRUPT_ENABLED() (TIMSK1 & (1<<OCIE1B))

#if defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1
  #define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x() WRITE(X_ENABLE_PIN,!X_ENABLE_ON)
#else
  #define enable_x() ;
  #define disable_x() ;
#endif

#if defined(Y_ENABLE_PIN) && Y_ENABLE_PIN > -1
  #define  enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
  #define disable_y() WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON)
#else
  #define enable_y() ;
  #define disable_y() ;
#endif

#if defined(Z_ENABLE_PIN) && Z_ENABLE_PIN > -1
  #ifdef Z_DUAL_STEPPER_DRIVERS
    #define  enable_z() { WRITE(Z_ENABLE_PIN, Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON); }
    #define disable_z() { WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN,!Z_ENABLE_ON); }
  #else
    #define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
    #define disable_z() WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON)
  #endif
#else
  #define enable_z() ;
  #define disable_z() ;
#endif

#if defined(E0_ENABLE_PIN) && (E0_ENABLE_PIN > -1)
  #define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e0() WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e0()  /* nothing */
  #define disable_e0() /* nothing */
#endif

#if MOTOR_CURRENT_PWM_XY_PIN > -1
extern const int motor_current_setting[3];
#endif

#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
// extern bool abort_on_endstop_hit;
#endif

extern volatile int32_t current_pos_steps[NUM_AXIS];

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
        st_write_dir_pin<MOVE>(  st_get_positive_dir<MOVE>());
    else
        st_write_dir_pin<MOVE>(! st_get_positive_dir<MOVE>());
}

template<typename MOVE>
inline uint8_t st_get_direction() {

    if (st_read_dir_pin<MOVE>()) {

        if (st_get_positive_dir<MOVE>())
            return st_get_move_bit_mask<MOVE>();
    }
    else {

        if (! st_get_positive_dir<MOVE>())
            return st_get_move_bit_mask<MOVE>();
    }

    return 0;
}

#define X_ENDSTOP_PRESSED (READ(X_STOP_PIN) != X_ENDSTOPS_INVERTING)
#define Y_ENDSTOP_PRESSED (READ(Y_STOP_PIN) != Y_ENDSTOPS_INVERTING)
#define Z_ENDSTOP_PRESSED (READ(Z_STOP_PIN) != Z_ENDSTOPS_INVERTING)

#define X_SW_ENDSTOP_PRESSED ((current_pos_steps[X_AXIS] < X_MIN_POS_STEPS) || (current_pos_steps[X_AXIS] > X_MAX_POS_STEPS))
#define Y_SW_ENDSTOP_PRESSED ((current_pos_steps[Y_AXIS] < Y_MIN_POS_STEPS) || (current_pos_steps[Y_AXIS] > Y_MAX_POS_STEPS))
// (current_pos_steps[Z_AXIS] < Z_MIN_POS_STEPS) || (current_pos_steps[Z_AXIS] > printer.z_max_pos_steps)
#define Z_SW_ENDSTOP_PRESSED ((current_pos_steps[Z_AXIS] < Z_MIN_POS_STEPS) || (current_pos_steps[Z_AXIS] > Z_MAX_POS_STEPS))

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
*/

typedef struct {
    // Bit 0-4: Direction bits, F
    // Bit 7: set-direction-flag, DDDDD
    uint8_t dirBits;
    uint8_t stepBits;
    uint16_t timer;
} stepData;

// Size of step buffer, entries are stepData structs.
#define StepBufferLen  256

class StepBuffer {

        private:
            stepData stepBuffer[StepBufferLen];

            uint8_t head, tail;
            uint8_t syncCount;

            // Mode of misc stepper timer:
            enum {
                HOMINGMODE,   // Misc stepper timer is used for homing
                CONTINUOSMODE // Misc stepper timer is used for continuous E move
            } miscStepperMode;

            uint16_t continuosTimer;

        public:

            StepBuffer() {
                head = tail = 0;
                syncCount = 0;
                miscStepperMode = HOMINGMODE;
            };

            void setContinuosTimer(uint16_t timerValue) {
    
                CRITICAL_SECTION_START;
                continuosTimer = timerValue;
                CRITICAL_SECTION_END;
            }

            FWINLINE uint8_t byteSize() {
                return (uint16_t)(StepBufferLen + head) - tail;
            }

            FWINLINE bool empty() {
                return head == tail;
            }

            FWINLINE bool full() {
                return byteSize() >= (StepBufferLen-1);
            }

            FWINLINE void sync() {
                syncCount = byteSize();
            }

            FWINLINE bool synced() {
                return syncCount == 0;
            }

            FWINLINE void push(stepData &sd) {

                simassert(! full());

                stepBuffer[head++] = sd;
            }

            FWINLINE stepData & pop() {
                if (syncCount > 0)
                    syncCount--;
                return stepBuffer[tail++];
            }

            void flush() {
                head = tail = 0;
                syncCount = 0;
            }

            void homingMode() {

                miscStepperMode = HOMINGMODE;

                // Start interrupt
                ENABLE_STEPPER1_DRIVER_INTERRUPT();
            }

            void continuosMode(uint16_t timerValue) {

                if (timerValue > 0) {

                    miscStepperMode = CONTINUOSMODE;
                    continuosTimer = timerValue;

                    enable_e0();

                    // Direction forward
                    st_write_dir_pin<EMove>( st_get_positive_dir<EMove>() );

                    // Start interrupt
                    ENABLE_STEPPER1_DRIVER_INTERRUPT();
                }
                else {

                    disable_e0();
                    DISABLE_STEPPER1_DRIVER_INTERRUPT();
                }
            }

            // * Timer 1A is running in CTC mode.
            // * ISR is called if timer 1A reaches OCR1A value
            // * Timer 1A is reset to 0 and starts counting up while ISR is running
            // * New timervalue is set at end of ISR
            // --> if ISR is running to long (or if it's locked by another interrupt) it is fired again
            //     immediately on return. The OCR1A value set is therefore ignored and the generated 
            //     pulse is not related to it.
            // --> To relax this situation we set the new OCR1A value as fast as possible.
            FWINLINE void runMoveSteps() {

                if (empty()) {

                    // Empty buffer, nothing to step
                    OCR1A = 2000; // 1kHz.
                }
                else {

                    stepData &sd = pop();

                    OCR1A = sd.timer;

                    if (sd.dirBits & 0x80) {

                        // Set direction bits
                        st_set_direction<XMove>(sd.dirBits);
                        st_set_direction<YMove>(sd.dirBits);
                        st_set_direction<ZMove>(sd.dirBits);
                        st_set_direction<EMove>(sd.dirBits);
                    }

                    // Step the motors and update step-coordinates (current_pos_steps): st_step_motor<>()
                    st_step_motor<XMove>(sd.stepBits, sd.dirBits);
                    st_step_motor<YMove>(sd.stepBits, sd.dirBits);
                    st_step_motor<ZMove>(sd.stepBits, sd.dirBits);
                    st_step_motor<EMove>(sd.stepBits, sd.dirBits);
                }
            }

        // FWINLINE void runHomingSteps() {
        FWINLINE void runMiscSteps() {

            if (miscStepperMode == HOMINGMODE)
                runHomingSteps();
            else
                runContinuosSteps();
        }

        FWINLINE void runHomingSteps() {

            if (empty()) {

                // Empty buffer, nothing to step
                OCR1A = OCR1B = 2000; // 1kHz.
            }
            else {

                stepData &sd = pop();

                OCR1A = OCR1B = sd.timer;

                // * Set direction 
                if (sd.dirBits & 0x80) {

                    // Set direction bits
                    st_set_direction<XMove>(sd.dirBits);
                    st_set_direction<YMove>(sd.dirBits);
                    st_set_direction<ZMove>(sd.dirBits);
                }

                // * Step the motors
                // * Check endstops
                // * Update step-coordinates (current_pos_steps)
                st_step_motor_es<XMove>(sd.stepBits, sd.dirBits);
                st_step_motor_es<YMove>(sd.stepBits, sd.dirBits);
                st_step_motor_es<ZMove>(sd.stepBits, sd.dirBits);
            }
        }

        FWINLINE void runContinuosSteps() {

            OCR1A = OCR1B = continuosTimer;
            st_step_motor<EMove>(st_get_move_bit_mask<EMove>(), st_get_move_bit_mask<EMove>());
        }

};

extern StepBuffer stepBuffer;

#include "simulator/stepperSim.h"


