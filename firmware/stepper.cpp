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


#include <Arduino.h>

#include "ddprint.h"
// #include "rxbuffer.h"
#include "stepper.h"
#include "mdebug.h"

// #if MOTOR_CURRENT_PWM_XY_PIN > -1
  // const int motor_current_setting[3] = DEFAULT_PWM_MOTOR_CURRENT;
// #endif

int32_t current_pos_steps[NUM_AXIS] = {
  0, // X
  0, // Y
  0, // Z
  0, // A
// #if defined(DualZStepper)
  // 0  // Z1
// #endif
};

StepBuffer stepBuffer;

void digipot_current(uint8_t driver, int current)
{
  #if defined( MOTOR_CURRENT_PWM_XY_PIN)
    if (driver == 0)
        MOTOR_CURRENT_PWM_XY_PIN :: write((long)current * 255L / (long)MOTOR_CURRENT_PWM_RANGE);
    if (driver == 1)
        MOTOR_CURRENT_PWM_Z_PIN :: write((long)current * 255L / (long)MOTOR_CURRENT_PWM_RANGE);
    if (driver == 2)
       MOTOR_CURRENT_PWM_E_PIN :: write((long)current * 255L / (long)MOTOR_CURRENT_PWM_RANGE);
  #endif
}

void digipot_init() //Initialize Digipot Motor Current
{
  #if defined(MOTOR_CURRENT_PWM_XY_PIN)
    MOTOR_CURRENT_PWM_XY_PIN :: init();
    MOTOR_CURRENT_PWM_Z_PIN :: init();
    MOTOR_CURRENT_PWM_E_PIN :: init();
    digipot_current(0, DEFAULT_PWM_MOTOR_CURRENT);
    digipot_current(1, DEFAULT_PWM_MOTOR_CURRENT);
    digipot_current(2, DEFAULT_PWM_MOTOR_CURRENT);
    //Set timer5 to 31khz so the PWM of the motor power is as constant as possible.
    TCCR5B = (TCCR5B & ~(_BV(CS50) | _BV(CS51) | _BV(CS52))) | _BV(CS50);
  #endif
}

void st_init() {

    digipot_init(); //Initialize Digipot Motor Current

    //Initialize Dir Pins
    X_DIR_PIN :: initDeActive();
    Y_DIR_PIN :: initDeActive();
    Z_DIR_PIN :: initDeActive();
#if defined(DualZStepper)
    Z1_DIR_PIN :: initDeActive();
#endif
    E0_DIR_PIN :: initDeActive();

    // #if defined(E1_DIR_PIN)
        // E1_DIR_PIN :: initDeActive();
    // #endif

    // Initialize Enable Pins - steppers default to disabled.
    X_ENABLE_PIN :: initDeActive();
    Y_ENABLE_PIN :: initDeActive();
    Z_ENABLE_PIN :: initDeActive();
#if defined(DualZStepper)
    Z1_ENABLE_PIN :: initDeActive();
#endif
    E0_ENABLE_PIN :: initDeActive();

    // #if defined(E1_ENABLE_PIN)
        // E1_ENABLE_PIN :: initDeActive();
    // #endif

    // Endstops and pullups
    X_STOP_PIN :: init();
    Y_STOP_PIN :: init();
    Z_STOP_PIN :: init();
#if defined(DualZStepper)
    Z1_STOP_PIN :: init();
#endif

    //Initialize Step Pins
    X_STEP_PIN :: initDeActive();
    Y_STEP_PIN :: initDeActive();
    Z_STEP_PIN :: initDeActive();
#if defined(DualZStepper)
    Z1_STEP_PIN :: initDeActive();
#endif
    E0_STEP_PIN :: initDeActive();
}

void st_enableSteppers(uint8_t stepperMask) {

#if defined(MB_FAN_PIN)
    if (stepperMask)
      MB_FAN_PIN :: activate();
#endif

    if (stepperMask & st_get_move_bit_mask<XAxisSelector>())
        X_ENABLE_PIN :: activate();
    if (stepperMask & st_get_move_bit_mask<YAxisSelector>())
        Y_ENABLE_PIN :: activate();
    if (stepperMask & st_get_move_bit_mask<ZAxisSelector>()) {
        Z_ENABLE_PIN :: activate();
        #if defined(DualZStepper)
        Z1_ENABLE_PIN :: activate();
        #endif
    }
    if (stepperMask & st_get_move_bit_mask<EAxisSelector>())
        E0_ENABLE_PIN :: activate();
}

void st_disableSteppers() {

    X_ENABLE_PIN :: deActivate();
    Y_ENABLE_PIN :: deActivate();
    Z_ENABLE_PIN :: deActivate();
#if defined(DualZStepper)
    Z1_ENABLE_PIN :: deActivate();
#endif
    E0_ENABLE_PIN :: deActivate();

#if defined(MB_FAN_PIN)
    MB_FAN_PIN :: deActivate();
#endif
}

//
// Stepper interrupt routines
//
#if defined(__arm__)

    //
    // STM32
    //
    // Note, to overwrite stm32duino's irq handlers they had to be defined as weak.
    //
    extern "C" {

        // Stepper irq routine
        void __irq_tim2(void)
        {

            timer_gen_reg_map *regs = timer2.regs.gen;

            if (regs->SR & TIMER_SR_UIF) {

	            stepBuffer.runPrintSteps();
                regs->SR = ~TIMER_SR_UIF;
                regs->SR; // Avoid duplicated pulses
            }
        }

        // Stepper irq routine for homing steps
        void __irq_tim3(void)
        {
            timer_gen_reg_map *regs = timer3.regs.gen;

            if (regs->SR & TIMER_SR_UIF) {

	            stepBuffer.runMiscSteps();
                regs->SR = ~TIMER_SR_UIF;
                regs->SR; // Avoid duplicated pulses
            }
        }

    }

#else

    //
    // AVR
    //
    #if defined(StepperOnTimer3)
        // Stepper irq routine
        ISR(TIMER3_COMPA_vect) {
            stepBuffer.runPrintSteps();
        }

        // Stepper irq routine for homing steps
        ISR(TIMER3_COMPB_vect) {
            stepBuffer.runMiscSteps();
        }
    #else
        // Stepper irq routine
        ISR(TIMER1_COMPA_vect) {
            stepBuffer.runPrintSteps();
        }

        // Stepper irq routine for homing steps
        ISR(TIMER1_COMPB_vect) {
            stepBuffer.runMiscSteps();
        }
    #endif

#endif

