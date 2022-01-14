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
#include "filsensor.h"

extern int32_t current_pos_steps[NUM_AXIS];

// Initialize and start the stepper motor subsystem
void st_init();
void st_enableSteppers(uint8_t stepperMask);
void st_disableSteppers();

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
FWINLINE void st_inc_current_pos_steps<XAxisSelector>() {
    current_pos_steps[X_AXIS] ++;
}
template<>
FWINLINE void st_inc_current_pos_steps<YAxisSelector>() {
    current_pos_steps[Y_AXIS] ++;
}
template<>
FWINLINE void st_inc_current_pos_steps<ZAxisSelector>() {
    current_pos_steps[Z_AXIS] ++;
}
template<>
FWINLINE void st_inc_current_pos_steps<EAxisSelector>() {
    current_pos_steps[E_AXIS] ++;
}

template<typename MOVE>
void st_dec_current_pos_steps();

template<>
FWINLINE void st_dec_current_pos_steps<XAxisSelector>() {
    current_pos_steps[X_AXIS] --;
}
template<>
FWINLINE void st_dec_current_pos_steps<YAxisSelector>() {
    current_pos_steps[Y_AXIS] --;
}
template<>
FWINLINE void st_dec_current_pos_steps<ZAxisSelector>() {
    current_pos_steps[Z_AXIS] --;
}
template<>
FWINLINE void st_dec_current_pos_steps<EAxisSelector>() {
    current_pos_steps[E_AXIS] --;
}

template<typename MOVE>
uint8_t getHomeDir();

template<>
FWINLINE uint8_t getHomeDir<XAxisSelector>() {
    return printer.getHostSettings().xHomeDir;
}
template<>
FWINLINE uint8_t getHomeDir<YAxisSelector>() {
    return printer.getHostSettings().yHomeDir;
}
template<>
FWINLINE uint8_t getHomeDir<ZAxisSelector>() {
    return printer.getHostSettings().zHomeDir;
}

template<typename MOVE>
FWINLINE void st_set_direction(uint8_t dirbits) {

    if (dirbits & st_get_move_bit_mask<MOVE>())
        activate_dir_pin<MOVE>();
    else
        deactivate_dir_pin<MOVE>();
}

#define EndstopDebounce(spmm) max(spmm/64, 2)

template<typename MOVE>
bool st_endstop_pressed();

template<>
FWINLINE bool st_endstop_pressed<XAxisSelector>() {

    static uint8_t nPresses = 0;

    if (X_STOP_PIN :: active()) {
        if (++nPresses >= EndstopDebounce(printer.getStepsPerMMX())) {
            nPresses = 0;
            return true;
        }
        return false;
    }

    nPresses = 0;
    return false;
}

template<>
FWINLINE bool st_endstop_pressed<YAxisSelector>() {

    static uint8_t nPresses = 0;

    if (Y_STOP_PIN :: active()) {
        if (++nPresses >= EndstopDebounce(printer.getStepsPerMMY())) {
            nPresses = 0;
            return true;
        }
        return false;
    }

    nPresses = 0;
    return false;
}

template<>
FWINLINE bool st_endstop_pressed<ZAxisSelector>() {

    static uint8_t nPresses = 0;

    if (Z_STOP_PIN :: active()) {
        if (++nPresses >= EndstopDebounce(printer.getStepsPerMMZ())) {
            nPresses = 0;
            return true;
        }
        return false;
    }

    nPresses = 0;
    return false;
}

#if defined(DualZStepper)
template<>
FWINLINE bool st_endstop_pressed<Z1AxisSelector>() {

    static uint8_t nPresses = 0;

    if (Z1_STOP_PIN :: active()) {
        if (++nPresses >= EndstopDebounce(printer.getStepsPerMMZ())) {
            nPresses = 0;
            return true;
        }
        return false;
    }

    nPresses = 0;
    return false;
}
#endif

template<typename MOVE>
bool st_endstop_released();

template<>
FWINLINE bool st_endstop_released<XAxisSelector>() {

    static uint8_t nRelease = 0;

    if (X_STOP_PIN :: deActive()) {
        if (++nRelease >= EndstopDebounce(printer.getStepsPerMMX())) {
            nRelease = 0;
            return true;
        }
        return false;
    }

    nRelease = 0;
    return false;
}

template<>
FWINLINE bool st_endstop_released<YAxisSelector>() {

    static uint8_t nRelease = 0;

    if (Y_STOP_PIN :: deActive()) {
        if (++nRelease >= EndstopDebounce(printer.getStepsPerMMY())) {
            nRelease = 0;
            return true;
        }
        return false;
    }

    nRelease = 0;
    return false;
}

template<>
FWINLINE bool st_endstop_released<ZAxisSelector>() {

    static uint8_t nRelease = 0;

    if (Z_STOP_PIN :: deActive()) {
        if (++nRelease >= EndstopDebounce(printer.getStepsPerMMZ())) {
            nRelease = 0;
            return true;
        }
        return false;
    }

    nRelease = 0;
    return false;
}

#if defined(DualZStepper)
template<>
FWINLINE bool st_endstop_released<Z1AxisSelector>() {

    static uint8_t nRelease = 0;

    if (Z1_STOP_PIN :: deActive()) {
        if (++nRelease >= EndstopDebounce(printer.getStepsPerMMZ())) {
            nRelease = 0;
            return true;
        }
        return false;
    }

    nRelease = 0;
    return false;
}
#endif

template<typename MOVE>
FWINLINE void st_step_motor(uint8_t stepBits, uint8_t dirbits) {

    constexpr uint8_t mask = st_get_move_bit_mask<MOVE>();

    if (stepBits & mask) {

        activate_step_pin<MOVE>();

        if (dirbits & mask)
            st_inc_current_pos_steps<MOVE>();
        else
            st_dec_current_pos_steps<MOVE>();
    }
}

template<typename MOVE>
FWINLINE void st_activate_pin(uint8_t stepBits) {

    constexpr uint8_t mask = st_get_move_bit_mask<MOVE>();

    if (stepBits & mask) {
        activate_step_pin<MOVE>();
    }
}

template<typename MOVE>
FWINLINE void st_deactivate_pin(uint8_t stepBits) {

    constexpr uint8_t mask = st_get_move_bit_mask<MOVE>();

    if (stepBits & mask) {
        deactivate_step_pin<MOVE>();
    }
}

//
// Like st_step_motor, but check endstops
//
template<typename MOVE>
FWINLINE void st_step_motor_es(uint8_t stepBits, uint8_t dirbits) {

    constexpr uint8_t mask = st_get_move_bit_mask<MOVE>();

    if (stepBits & mask) {

        bool forward = dirbits & mask;
        uint8_t homeDir = getHomeDir<MOVE>();

        bool towardsEndstop = (forward == homeDir);

        if (towardsEndstop && st_endstop_pressed<MOVE>()) {
            DISABLE_STEPPER1_DRIVER_INTERRUPT();
            return;
        }

        if (!towardsEndstop && st_endstop_released<MOVE>()) {
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

#if defined(DualZStepper)
//
// Like st_step_motor, but check endstops
// Version for dual steppers/endstops:
// When moving towards endstop, then:
// * move both motors
// * stop motor Zx that belongs to endstop Zx and continue to move the other
// * stop move if both endstops are pressed
//

template<>
FWINLINE void st_step_motor_es<ZAxisSelector>(uint8_t stepBits, uint8_t dirbits) {

    // State of z-homing
    static enum {
        MOVEBOTH, // Move both motors
        MOVEZ0,   // Z1 endtop triggered, move Z0 only
        MOVEZ1    // Z0 endtop triggered, move Z1 only
    } zHomingMode = MOVEBOTH;

    constexpr uint8_t mask = st_get_move_bit_mask<ZAxisSelector>();

    if (stepBits & mask) {

        bool forward = dirbits & mask;
        uint8_t homeDir = getHomeDir<ZAxisSelector>();

        bool towardsEndstop = (forward == homeDir);

        if (towardsEndstop && st_endstop_pressed<ZAxisSelector>()) {

            if (zHomingMode == MOVEBOTH) {
              zHomingMode = MOVEZ1;
            }
            else if (zHomingMode == MOVEZ0) {
                DISABLE_STEPPER1_DRIVER_INTERRUPT();
                zHomingMode = MOVEBOTH;
                return;
            }
        }

        if (towardsEndstop && st_endstop_pressed<Z1AxisSelector>()) {

            if (zHomingMode == MOVEBOTH) {
              zHomingMode = MOVEZ0;
            }
            else if (zHomingMode == MOVEZ1) {
                DISABLE_STEPPER1_DRIVER_INTERRUPT();
                zHomingMode = MOVEBOTH;
                return;
            }
        }

        if (!towardsEndstop && st_endstop_released<ZAxisSelector>() && st_endstop_released<Z1AxisSelector>()) {
            DISABLE_STEPPER1_DRIVER_INTERRUPT();
            zHomingMode = MOVEBOTH;
            return;
        }

        switch (zHomingMode) {
          case MOVEBOTH:
            activate_step_pin<ZAxisSelector>();
            break;
          case MOVEZ0:
            Z_STEP_PIN :: activate();
            break;
          case MOVEZ1:
            Z1_STEP_PIN :: activate();
            break;
        }

        #if defined(STEPPER_MINPULSE)
            delayMicroseconds(STEPPER_MINPULSE);
        #endif

        if (zHomingMode == MOVEBOTH) {
            if (forward)
                st_inc_current_pos_steps<ZAxisSelector>();
            else
                st_dec_current_pos_steps<ZAxisSelector>();
        }

        deactivate_step_pin<ZAxisSelector>();
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
    // Bit 7: set-direction-flag
    // Bit 0-4: Direction bits
    uint8_t dirBits;   // S??DDDDD -> 3 bits unused
    // Bit 0-4: Step bits
    uint8_t stepBits;  // ???SSSSS -> 3 bits unused
    uint16_t timer;
} stepData;

// Stepbuffer
#if defined(AVR)
    typedef Buffer256<stepData> StepBufferBase;
#else
    #define StepBufferLen  4096
    typedef CircularBuffer<stepData, uint16_t, StepBufferLen> StepBufferBase;
#endif

class StepBuffer: public StepBufferBase {

        private:
            // undo uint8_t syncCount;

            // Mode of misc stepper timer:
            enum {
                HOMINGMODE,   // Misc stepper timer is used for homing
                CONTINUOSMODE // Misc stepper timer is used for continuous E move
            } miscStepperMode;

            uint16_t continuosTimer;

            // Sum of timer values in buffer
            uint32_t upcount;
            uint32_t downcount;

            // Steppers to run in continuos mode
            uint8_t contStepBits;

        public:

            bool linearFlag;

            StepBuffer() {

                // undo syncCount = 0;
                miscStepperMode = HOMINGMODE;

                flush();
            };

            void setContinuosTimer(uint16_t timerValue) {
    
                if (timerValue > 0) {

                    CRITICAL_SECTION_START;
                    continuosTimer = timerValue;
                    CRITICAL_SECTION_END;
                }
                else {

                  // Stop continuos move
                  linearFlag = false;
                  st_disableSteppers();
                  DISABLE_STEPPER1_DRIVER_INTERRUPT();
                }
            }

            FWINLINE void sync() {
                // undo syncCount = byteSize();
            }

            // undo FWINLINE bool synced() {
                // undo return syncCount == 0;
            // undo }

            void flush() {
                ringBufferInit();
                upcount = downcount = 0;
                linearFlag = false;
            }

            void homingMode() {

                miscStepperMode = HOMINGMODE;

                // Start interrupt
                HAL_SET_HOMING_TIMER(2000); // 1kHz.
                ENABLE_STEPPER1_DRIVER_INTERRUPT();
            }

            void continuosMode(uint8_t stepperMask, uint16_t timerValue) {

                miscStepperMode = CONTINUOSMODE;
                contStepBits = stepperMask;
                continuosTimer = timerValue;

                st_enableSteppers(stepperMask);

                // Direction forward
                // E0_DIR_PIN :: activate();
                // Set direction bits
                st_set_direction<XAxisSelector>(stepperMask);
                st_set_direction<YAxisSelector>(stepperMask);
                st_set_direction<ZAxisSelector>(stepperMask);
                st_set_direction<EAxisSelector>(stepperMask);

                // Start interrupt
                ENABLE_STEPPER1_DRIVER_INTERRUPT();

                linearFlag = true;
            }

            // Compute clocktics available in stepper
            // buffer.
            FWINLINE uint8_t timeInBuffer() {

                CRITICAL_SECTION_START;
                uint32_t d = downcount;
                CRITICAL_SECTION_END;

                // return STD min( (upcount-d)/2000, 255);
                return STD min( (upcount-d)/2000, (uint32_t)255);
            }

            // Reserve 50+ ms buffer depth for long usb
            // transactions, this assumes 2Mhz timer clock tick.
            FWINLINE bool enough() { 
                return full() || (timeInBuffer() >= 60); }

            void pushRef(stepData& val)  {
                upcount += val.timer;
                StepBufferBase::pushRef(val);
            }

            // * Timer 1A is running in CTC mode.
            // * ISR is called if timer 1A reaches OCR1A value
            // * Timer 1A is reset to 0 and starts counting up while ISR is running
            // * New timervalue is set at end of ISR
            // --> if ISR is running to long (or if it's locked by another interrupt) it is fired again
            //     immediately on return. The OCR1A value set is therefore ignored and the generated 
            //     pulse is not related to it.
            // --> To relax this situation we set the new OCR1A value as fast as possible.
            FWINLINE void runPrintSteps() {

                if (empty()) {

                    // Empty buffer, nothing to step
                    if (printer.stepsAvailable()) {

                        HAL_SET_STEPPER_TIMER(25);
                        printer.underrunError();
                    }
                    else {

                        HAL_SET_STEPPER_TIMER(2000); // 1kHz.
                    }
                }
                else {

                    stepData &sd = pop();

                    uint16_t t = sd.timer;

                    // Set new timer value
                    #if defined(HEAVYDEBUG)
                    massert(t >= 25);
                    #endif

                    HAL_SET_STEPPER_TIMER(t);

                    // Set dir bits
                    if (sd.dirBits & 0x80) {

                        // Set direction bits
                        st_set_direction<XAxisSelector>(sd.dirBits);
                        st_set_direction<YAxisSelector>(sd.dirBits);
                        st_set_direction<ZAxisSelector>(sd.dirBits);
                        st_set_direction<EAxisSelector>(sd.dirBits);
                    }

                    uint8_t stepbits = sd.stepBits;

                    #if defined(HASFILAMENTSENSOR)
                    if (sd.dirBits & 0x40) {
                        // Start FRS measurement if not running already
                        if (filamentSensor.idle())
                            linearFlag = true;
                    }
                    else {
                        linearFlag = false;
                    }
                    #endif

                    if (stepbits) {

                        st_step_motor<XAxisSelector>(stepbits, sd.dirBits);
                        st_step_motor<YAxisSelector>(stepbits, sd.dirBits);
                        st_step_motor<ZAxisSelector>(stepbits, sd.dirBits);
                        st_step_motor<EAxisSelector>(stepbits, sd.dirBits);

                        #if defined(STEPPER_MINPULSE)
                            delayMicroseconds(STEPPER_MINPULSE);
                        #endif

                        st_deactivate_pin<XAxisSelector>(stepbits);
                        st_deactivate_pin<YAxisSelector>(stepbits);
                        st_deactivate_pin<ZAxisSelector>(stepbits);
                        st_deactivate_pin<EAxisSelector>(stepbits);
                    }

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

                downcount += sd.timer;

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

            // st_step_motor<EAxisSelector>(st_get_move_bit_mask<EAxisSelector>(), st_get_move_bit_mask<EAxisSelector>());
            // #if defined(STEPPER_MINPULSE)
                // delayMicroseconds(STEPPER_MINPULSE);
            // #endif
            // st_deactivate_pin<EAxisSelector>(st_get_move_bit_mask<EAxisSelector>());

            st_step_motor<XAxisSelector>(contStepBits, contStepBits);
            st_step_motor<YAxisSelector>(contStepBits, contStepBits);
            st_step_motor<ZAxisSelector>(contStepBits, contStepBits);
            st_step_motor<EAxisSelector>(contStepBits, contStepBits);

            #if defined(STEPPER_MINPULSE)
                delayMicroseconds(STEPPER_MINPULSE);
            #endif

            st_deactivate_pin<XAxisSelector>(contStepBits);
            st_deactivate_pin<YAxisSelector>(contStepBits);
            st_deactivate_pin<ZAxisSelector>(contStepBits);
            st_deactivate_pin<EAxisSelector>(contStepBits);
        }
};

extern StepBuffer stepBuffer;

#if defined(DDSim)
    #include "simulator/stepperSim.h"
#endif


