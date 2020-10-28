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

#include "ddprint.h"
#include "move.h"
#include "ringbuffer.h"

#define  enable_x() (X_ENABLE_PIN :: activate())
#define disable_x() (X_ENABLE_PIN :: deActivate())

#define  enable_y() (Y_ENABLE_PIN :: activate())
#define disable_y() (Y_ENABLE_PIN :: deActivate())

#ifdef Z_DUAL_STEPPER_DRIVERS
  #define  enable_z() { Z_ENABLE_PIN :: activate(); Z2_ENABLE_PIN :: activate(); }
  #define disable_z() { Z_ENABLE_PIN :: deActivate(); Z2_ENABLE_PIN :: deActivate(); }
#else
  #define  enable_z() (Z_ENABLE_PIN :: activate())
  #define disable_z() (Z_ENABLE_PIN :: deActivate())
#endif

#define enable_e0() (E0_ENABLE_PIN :: activate())
#define disable_e0() (E0_ENABLE_PIN :: deActivate())

extern volatile int32_t current_pos_steps[NUM_AXIS];

// Initialize and start the stepper motor subsystem
void st_init();
void st_disableSteppers();

template<typename MOVE>
void st_set_position_steps(long value);

template<>
void st_set_position_steps<XAxisSelector>(long value);
template<>
void st_set_position_steps<YAxisSelector>(long value);
template<>
void st_set_position_steps<ZAxisSelector>(long value);
template<>
void st_set_position_steps<EAxisSelector>(long value);

#if 0
template<typename MOVE>
long st_get_position_steps();

template<>
long st_get_position_steps<XAxisSelector>();
template<>
long st_get_position_steps<YAxisSelector>();
template<>
long st_get_position_steps<ZAxisSelector>();
template<>
long st_get_position_steps<EAxisSelector>();
#endif

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
inline void st_inc_current_pos_steps<XAxisSelector>() {
    current_pos_steps[X_AXIS] ++;
}
template<>
inline void st_inc_current_pos_steps<YAxisSelector>() {
    current_pos_steps[Y_AXIS] ++;
}
template<>
inline void st_inc_current_pos_steps<ZAxisSelector>() {
    current_pos_steps[Z_AXIS] ++;
}
template<>
inline void st_inc_current_pos_steps<EAxisSelector>() {
    current_pos_steps[E_AXIS] ++;
}

template<typename MOVE>
void st_dec_current_pos_steps();

template<>
inline void st_dec_current_pos_steps<XAxisSelector>() {
    current_pos_steps[X_AXIS] --;
}
template<>
inline void st_dec_current_pos_steps<YAxisSelector>() {
    current_pos_steps[Y_AXIS] --;
}
template<>
inline void st_dec_current_pos_steps<ZAxisSelector>() {
    current_pos_steps[Z_AXIS] --;
}
template<>
inline void st_dec_current_pos_steps<EAxisSelector>() {
    current_pos_steps[E_AXIS] --;
}

template<typename MOVE>
inline void st_set_direction(uint8_t dirbits) {

    if (dirbits & st_get_move_bit_mask<MOVE>())
        activate_dir_pin<MOVE>();
    else
        deactivate_dir_pin<MOVE>();
}

#if 0
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
#endif

template<typename MOVE>
bool st_endstop_pressed(bool);

template<>
inline bool st_endstop_pressed<XAxisSelector>(bool forward) {

    static uint8_t nPresses = 0;

    if (forward) {
        #if X_HOME_DIR > 0
            if (X_STOP_PIN :: active()) {
                if (nPresses++ > (printer.getStepsPerMMX() / 4))
                    return true;
                return false;
            }
        #endif
    }
    else {
        #if X_HOME_DIR < 0
            if (X_STOP_PIN :: active()) {
                if (nPresses++ > (printer.getStepsPerMMX() / 4))
                    return true;
                return false;
            }
        #endif
    }

    nPresses = 0;
    return false;
}

template<>
inline bool st_endstop_pressed<YAxisSelector>(bool forward) {

    static uint8_t nPresses = 0;

    if (forward) {
        #if Y_HOME_DIR > 0
            if (Y_STOP_PIN :: active()) {
                if (nPresses++ > (printer.getStepsPerMMY() / 4))
                    return true;
                return false;
            }
        #endif
    }
    else {
        #if Y_HOME_DIR < 0
            if (Y_STOP_PIN :: active()) {
                if (nPresses++ > (printer.getStepsPerMMY() / 4))
                    return true;
                return false;
            }
        #endif
    }

    nPresses = 0;
    return false;
}

template<>
inline bool st_endstop_pressed<ZAxisSelector>(bool forward) {

    static uint8_t nPresses = 0;

    if (forward) {
        #if Z_HOME_DIR > 0
            if (Z_STOP_PIN :: active()) {
                if (nPresses++ > (printer.getStepsPerMMZ() / 4))
                    return true;
                return false;
            }
        #endif
    }
    else {
        #if Z_HOME_DIR < 0
            if (Z_STOP_PIN :: active()) {
                if (nPresses++ > (printer.getStepsPerMMZ() / 4))
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
inline bool st_endstop_released<XAxisSelector>(bool forward) {

    static uint8_t nRelease = 0;

    if (forward) {
        #if X_HOME_DIR < 0
            if (X_STOP_PIN :: deActive()) {
                if (nRelease++ > (printer.getStepsPerMMX() / 4))
                    return true;
                return false;
            }
        #endif
    }
    else {
        #if X_HOME_DIR > 0
            if (X_STOP_PIN :: deActive()) {
                if (nRelease++ > (printer.getStepsPerMMX() / 4))
                    return true;
                return false;
            }
        #endif
    }

    nRelease = 0;
    return false;
}

template<>
inline bool st_endstop_released<YAxisSelector>(bool forward) {

    static uint8_t nRelease = 0;

    if (forward) {
        #if Y_HOME_DIR < 0
            if (Y_STOP_PIN :: deActive()) {
                if (nRelease++ > (printer.getStepsPerMMY() / 4))
                    return true;
                return false;
            }
        #endif
    }
    else {
        #if Y_HOME_DIR > 0
            if (Y_STOP_PIN :: deActive()) {
                if (nRelease++ > (printer.getStepsPerMMY() / 4))
                    return true;
                return false;
            }
        #endif
    }

    nRelease = 0;
    return false;
}

template<>
inline bool st_endstop_released<ZAxisSelector>(bool forward) {

    static uint8_t nRelease = 0;

    if (forward) {
        #if Z_HOME_DIR < 0
            if (Z_STOP_PIN :: deActive()) {
                if (nRelease++ > (printer.getStepsPerMMZ() / 4))
                    return true;
                return false;
            }
        #endif
    }
    else {
        #if Z_HOME_DIR > 0
            if (Z_STOP_PIN :: deActive()) {
                if (nRelease++ > (printer.getStepsPerMMZ() / 4))
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

    uint8_t mask = st_get_move_bit_mask<MOVE>();

    if (stepBits & mask) {

        activate_step_pin<MOVE>();

        if (dirbits & mask)
            st_inc_current_pos_steps<MOVE>();
        else
            st_dec_current_pos_steps<MOVE>();
        
        // #if defined(AVR)
        #if defined(STEPPER_MINPULSE)
            delayMicroseconds(STEPPER_MINPULSE);
        #endif
            deactivate_step_pin<MOVE>();
        // #endif
    }
}

template<typename MOVE>
inline void st_deactivate_pin(uint8_t stepBits) {

    uint8_t mask = st_get_move_bit_mask<MOVE>();

    if (stepBits & mask) {
        deactivate_step_pin<MOVE>();
    }
}

//
// Like st_step_motor, but check endstops
//
template<typename MOVE>
inline void st_step_motor_es(uint8_t stepBits, uint8_t dirbits) {

    uint8_t mask = st_get_move_bit_mask<MOVE>();

    if (stepBits & mask) {

        bool forward = dirbits & mask;

        bool endStop = st_endstop_pressed<MOVE>(forward);

        if (endStop) {
            DISABLE_STEPPER1_DRIVER_INTERRUPT();
            return;
        }

        endStop = st_endstop_released<MOVE>(forward);

        if (endStop) {
            DISABLE_STEPPER1_DRIVER_INTERRUPT();
            return;
        }

        activate_step_pin<MOVE>();

        #if defined(STEPPER_MINPULSE)
            delayMicroseconds(STEPPER_MINPULSE);
        #endif

        if (forward)
            st_inc_current_pos_steps<MOVE>();
        else
            st_dec_current_pos_steps<MOVE>();

        deactivate_step_pin<MOVE>();
    }
}

#if 0
template<typename MOVE>
inline void st_toggle_motor_es(uint8_t stepBits, uint8_t dirbits) {

    uint8_t mask = st_get_move_bit_mask<MOVE>();
    static bool activ = false;

    if (stepBits & mask) {

        if (activ) {

            bool forward = dirbits & mask;
            bool endStop = st_endstop_pressed<MOVE>(forward);

            if (endStop) {
                DISABLE_STEPPER1_DRIVER_INTERRUPT();
                return;
            }

            endStop = st_endstop_released<MOVE>(forward);

            if (endStop) {
                DISABLE_STEPPER1_DRIVER_INTERRUPT();
                return;
            }

            // Update steps counter on falling edge
            if (forward)
                st_inc_current_pos_steps<MOVE>();
            else
                st_dec_current_pos_steps<MOVE>();

            deactivate_step_pin<MOVE>();
            activ = false;
        }
        else {
            activate_step_pin<MOVE>();
            activ = true;
        }
    }
}

#endif



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
    // uint8_t cmd;
    // Bit 0-4: Direction bits, F
    // Bit 7: set-direction-flag, DDDDD
    uint8_t dirBits;
    uint8_t stepBits;
    uint16_t timer;
} stepData;

// Size of step buffer, entries are stepData structs.
#if defined(AVR)
#define StepBufferLen  256
#else
// #define StepBufferLen  512
#define StepBufferLen  2048
#endif

typedef CircularBuffer<stepData, uint32_t, StepBufferLen> StepBufferBase;

class StepBuffer: public StepBufferBase {

        private:
            // undo uint8_t syncCount;

            // Mode of misc stepper timer:
            enum {
                HOMINGMODE,   // Misc stepper timer is used for homing
                CONTINUOSMODE // Misc stepper timer is used for continuous E move
            } miscStepperMode;

            uint16_t continuosTimer;

            uint8_t stepbits; // xxx for deactivate

        public:

            StepBuffer() {

                ringBufferInit();

                // undo syncCount = 0;
                miscStepperMode = HOMINGMODE;
                stepbits = 0;
            };

            void setContinuosTimer(uint16_t timerValue) {
    
                CRITICAL_SECTION_START;
                continuosTimer = timerValue;
                CRITICAL_SECTION_END;
            }

            FWINLINE void sync() {
                // undo syncCount = byteSize();
            }

            // undo FWINLINE bool synced() {
                // undo return syncCount == 0;
            // undo }

            void flush() {
                ringBufferInit();
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
                    E0_DIR_PIN :: activate();

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

                static bool wasnotempty = true;

                if (empty()) {

                    // Empty buffer, nothing to step
                    HAL_SET_STEPPER_TIMER(2000); // 1kHz.
                    stepbits = 0;

                    // Check for step buffer underruns
                    if (wasnotempty && printer.stepsAvailable())
                            printer.underrunError();

                    wasnotempty = false;
                }
                else {

                    stepData &sd = pop();

                    uint16_t t = sd.timer;

                    // Set new timer value
#if defined(HEAVYDEBUG)
                    massert(t >= 25);
#endif

                    HAL_SET_STEPPER_TIMER(t - (2*STEPPER_MINPULSE)); // correction: min step width z.b. 2uS -> 4 timer takte

                    // Set dir bits
                    if (sd.dirBits & 0x80) {

                        // Set direction bits
                        st_set_direction<XAxisSelector>(sd.dirBits);
                        st_set_direction<YAxisSelector>(sd.dirBits);
                        st_set_direction<ZAxisSelector>(sd.dirBits);
                        st_set_direction<EAxisSelector>(sd.dirBits);
                    }

                    stepbits = sd.stepBits;

                    st_step_motor<XAxisSelector>(stepbits, sd.dirBits);
                    st_step_motor<YAxisSelector>(stepbits, sd.dirBits);
                    st_step_motor<ZAxisSelector>(stepbits, sd.dirBits);
                    st_step_motor<EAxisSelector>(stepbits, sd.dirBits);

                    wasnotempty = true;
                }
            }

        FWINLINE void deactivateSteppers() {

            if (stepbits) {

                st_deactivate_pin<XAxisSelector>(stepbits);
                st_deactivate_pin<YAxisSelector>(stepbits);
                st_deactivate_pin<ZAxisSelector>(stepbits);
                st_deactivate_pin<EAxisSelector>(stepbits);

                stepbits = 0;
            }
        }

        FWINLINE void runMiscSteps() {

            if (miscStepperMode == HOMINGMODE)
                runHomingSteps();
            else
                runContinuosSteps();
        }

        FWINLINE void runHomingSteps() {

            if (empty()) {

                // Empty buffer, nothing to step
                HAL_SET_HOMING_TIMER(2000); // 1kHz.
            }
            else {

                stepData &sd = pop();

                HAL_SET_HOMING_TIMER(sd.timer);

                // * Set direction 
                if (sd.dirBits & 0x80) {

                    // Set direction bits
                    st_set_direction<XAxisSelector>(sd.dirBits);
                    st_set_direction<YAxisSelector>(sd.dirBits);
                    st_set_direction<ZAxisSelector>(sd.dirBits);
                }

                // * Step the motors
                // * Check endstops
                // * Update step-coordinates (current_pos_steps)
                st_step_motor_es<XAxisSelector>(sd.stepBits, sd.dirBits);
                st_step_motor_es<YAxisSelector>(sd.stepBits, sd.dirBits);
                st_step_motor_es<ZAxisSelector>(sd.stepBits, sd.dirBits);
            }
        }

        FWINLINE void runContinuosSteps() {

            HAL_SET_HOMING_TIMER(continuosTimer);
            massert(0); 
            // st_step_motor<EAxisSelector>(st_get_move_bit_mask<EAxisSelector>(), st_get_move_bit_mask<EAxisSelector>());
        }

};

extern StepBuffer stepBuffer;

#include "simulator/stepperSim.h"


