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
#include "serialport.h"
#include "stepper.h"
#include "mdebug.h"

// #if MOTOR_CURRENT_PWM_XY_PIN > -1
  // const int motor_current_setting[3] = DEFAULT_PWM_MOTOR_CURRENT;
// #endif

volatile int32_t current_pos_steps[NUM_AXIS] = { 0, 0, 0, 0};

//armdebug
#if 0

StepBuffer stepBuffer;
#endif

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
  #if defined(X_DIR_PIN)
    // SET_OUTPUT(X_DIR_PIN);
    X_DIR_PIN :: initDeActive();
  #endif
  #if defined(Y_DIR_PIN)
    // SET_OUTPUT(Y_DIR_PIN);
    Y_DIR_PIN :: initDeActive();
  #endif
  #if defined(Z_DIR_PIN)
    // SET_OUTPUT(Z_DIR_PIN);
    Z_DIR_PIN :: initDeActive();
  #endif
  #if defined(E0_DIR_PIN)
    // SET_OUTPUT(E0_DIR_PIN);
    E0_DIR_PIN :: initDeActive();
  #endif
  #if defined(E1_DIR_PIN)
    // SET_OUTPUT(E1_DIR_PIN);
    E1_DIR_PIN :: initDeActive();
  #endif

  // Set initial direction to 0
  // st_set_direction<XAxisSelector>(LOW);
  // st_set_direction<YAxisSelector>(LOW);
  // st_set_direction<ZAxisSelector>(LOW);
  // st_set_direction<EAxisSelector>(LOW);

  // Initialize Enable Pins - steppers default to disabled.
  // SET_OUTPUT(X_ENABLE_PIN);
  // WRITE(X_ENABLE_PIN, ~ X_ENABLE_ACTIVE);
  X_ENABLE_PIN :: initDeActive();

  // SET_OUTPUT(Y_ENABLE_PIN);
  // WRITE(Y_ENABLE_PIN, ~ Y_ENABLE_ACTIVE);
  Y_ENABLE_PIN :: initDeActive();

  // SET_OUTPUT(Z_ENABLE_PIN);
  // WRITE(Z_ENABLE_PIN, ~ Z_ENABLE_ACTIVE);
  Z_ENABLE_PIN :: initDeActive();

  // SET_OUTPUT(E0_ENABLE_PIN);
  // WRITE(E0_ENABLE_PIN, ~ E0_ENABLE_ACTIVE);
  E0_ENABLE_PIN :: initDeActive();

  #if defined(E1_ENABLE_PIN)
    // SET_OUTPUT(E1_ENABLE_PIN);
    // WRITE(E1_ENABLE_PIN, ~ E1_ENABLE_ACTIVE);
    E1_ENABLE_PIN :: initDeActive();
  #endif

  //endstops and pullups

  // SET_INPUT(X_STOP_PIN);
  // xxx armtodo pullup WRITE(X_STOP_PIN,HIGH);
  X_STOP_PIN :: init();

  // SET_INPUT(Y_STOP_PIN);
  // xxx armtodo pullup // WRITE(Y_STOP_PIN,HIGH);
  // gpio_set_mode(Y_STOP_PIN, GPIO_INPUT_PU);
  Y_STOP_PIN :: init();

  // SET_INPUT(Z_STOP_PIN);
  // xxx armtodo pullup WRITE(Z_STOP_PIN,HIGH);
  Z_STOP_PIN :: init();

  //Initialize Step Pins
  #if defined(X_STEP_PIN)
    // SET_OUTPUT(X_STEP_PIN);
    // WRITE(X_STEP_PIN, LOW);
    X_STEP_PIN :: initDeActive();
    disable_x();
  #endif
  #if defined(Y_STEP_PIN)
    // SET_OUTPUT(Y_STEP_PIN);
    // WRITE(Y_STEP_PIN, LOW);
    Y_STEP_PIN :: initDeActive();
    disable_y();
  #endif
  #if defined(Z_STEP_PIN)
    // SET_OUTPUT(Z_STEP_PIN);
    // WRITE(Z_STEP_PIN, LOW);
    Z_STEP_PIN :: initDeActive();
    disable_z();
  #endif
  #if defined(E0_STEP_PIN)
    // SET_OUTPUT(E0_STEP_PIN);
    // WRITE(E0_STEP_PIN, LOW);
    E0_STEP_PIN :: initDeActive();
    disable_e0();
  #endif

// armrun
#if 0
    //
    // Setup Timer1 for stepper interrupt and 
    // homingstepper interrupt.
    //

    // 
    // Timer 0 is used by arduino (millis() ...)
    // Timer 2 is 8bit only
    // Timer 3 ist heater pwm
    // Timer 4 ist LED pin und FAN pin
    // Timer 5 ist digipot
    //

    // waveform generation = 0100 = CTC
    TCCR1B &= ~(1<<WGM13);
    TCCR1B |=  (1<<WGM12);
    TCCR1A &= ~(1<<WGM11);
    TCCR1A &= ~(1<<WGM10);
    
    // output mode = 00 (disconnected)
    // Normal port operation, OCnA/OCnB/OCnC disconnected
    TCCR1A &= ~(3<<COM1A0);
    TCCR1A &= ~(3<<COM1B0);

    // Generally we use a divider of 8, resulting in a 2MHz timer
    // frequency on a 16MHz MCU.
    TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

    OCR1A = 0x4000;
    OCR1B = 0x4000;
    TCNT1 = 0;
#endif

  // ENABLE_STEPPER_DRIVER_INTERRUPT();
  SEI();
}

//armdebug
#if 0

ISR(TIMER1_COMPA_vect) {

    // xxx call it runPrintStep
    stepBuffer.runMoveSteps();
}

ISR(TIMER1_COMPB_vect) {

    // stepBuffer.runHomingSteps();
    stepBuffer.runMiscSteps();
}

#endif











