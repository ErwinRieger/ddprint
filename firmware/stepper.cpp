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

//armdebug
#if 0

#if MOTOR_CURRENT_PWM_XY_PIN > -1
  const int motor_current_setting[3] = DEFAULT_PWM_MOTOR_CURRENT;
#endif

volatile int32_t current_pos_steps[NUM_AXIS] = { 0, 0, 0, 0};
StepBuffer stepBuffer;

void digipot_current(uint8_t driver, int current)
{
  #if MOTOR_CURRENT_PWM_XY_PIN > -1
  if (driver == 0) analogWrite(MOTOR_CURRENT_PWM_XY_PIN, (long)current * 255L / (long)MOTOR_CURRENT_PWM_RANGE);
  if (driver == 1) analogWrite(MOTOR_CURRENT_PWM_Z_PIN, (long)current * 255L / (long)MOTOR_CURRENT_PWM_RANGE);
  if (driver == 2) analogWrite(MOTOR_CURRENT_PWM_E_PIN, (long)current * 255L / (long)MOTOR_CURRENT_PWM_RANGE);
  #endif
}

void digipot_init() //Initialize Digipot Motor Current
{
  #if MOTOR_CURRENT_PWM_XY_PIN > -1
    pinMode(MOTOR_CURRENT_PWM_XY_PIN, OUTPUT);
    pinMode(MOTOR_CURRENT_PWM_Z_PIN, OUTPUT);
    pinMode(MOTOR_CURRENT_PWM_E_PIN, OUTPUT);
    digipot_current(0, motor_current_setting[0]);
    digipot_current(1, motor_current_setting[1]);
    digipot_current(2, motor_current_setting[2]);
    //Set timer5 to 31khz so the PWM of the motor power is as constant as possible.
    TCCR5B = (TCCR5B & ~(_BV(CS50) | _BV(CS51) | _BV(CS52))) | _BV(CS50);
  #endif
}

void st_init() {

  digipot_init(); //Initialize Digipot Motor Current

  #if defined(X_MS1_PIN) && X_MS1_PIN > -1
    #error enable microstep
    // microstep_init(); //Initialize Microstepping Pins // look
  #endif

  //Initialize Dir Pins
  #if defined(X_DIR_PIN) && X_DIR_PIN > -1
    SET_OUTPUT(X_DIR_PIN);
  #endif
  #if defined(Y_DIR_PIN) && Y_DIR_PIN > -1
    SET_OUTPUT(Y_DIR_PIN);
  #endif
  #if defined(Z_DIR_PIN) && Z_DIR_PIN > -1
    SET_OUTPUT(Z_DIR_PIN);
  #endif
  #if defined(E0_DIR_PIN) && E0_DIR_PIN > -1
    SET_OUTPUT(E0_DIR_PIN);
  #endif
  #if defined(E1_DIR_PIN) && (E1_DIR_PIN > -1)
    SET_OUTPUT(E1_DIR_PIN);
  #endif
  #if defined(E2_DIR_PIN) && (E2_DIR_PIN > -1)
    SET_OUTPUT(E2_DIR_PIN);
  #endif

  // Set initial direction to 0
  // st_set_direction<XAxisSelector>(LOW);
  // st_set_direction<YAxisSelector>(LOW);
  // st_set_direction<ZAxisSelector>(LOW);
  // st_set_direction<EAxisSelector>(LOW);

  // Initialize Enable Pins - steppers default to disabled.
  SET_OUTPUT(X_ENABLE_PIN);
  WRITE(X_ENABLE_PIN, ! X_ENABLE_ON);

  SET_OUTPUT(Y_ENABLE_PIN);
  WRITE(Y_ENABLE_PIN, ! Y_ENABLE_ON);

  SET_OUTPUT(Z_ENABLE_PIN);
  WRITE(Z_ENABLE_PIN, ! Z_ENABLE_ON);

  SET_OUTPUT(E0_ENABLE_PIN);
  WRITE(E0_ENABLE_PIN, ! E_ENABLE_ON);

  #if defined(E1_ENABLE_PIN)
    SET_OUTPUT(E1_ENABLE_PIN);
    WRITE(E1_ENABLE_PIN, ! E_ENABLE_ON);
  #endif

  //endstops and pullups

  SET_INPUT(X_STOP_PIN);
  WRITE(X_STOP_PIN,HIGH);

  SET_INPUT(Y_STOP_PIN);
  WRITE(Y_STOP_PIN,HIGH);

  SET_INPUT(Z_STOP_PIN);
  WRITE(Z_STOP_PIN,HIGH);

  //Initialize Step Pins
  #if defined(X_STEP_PIN) && (X_STEP_PIN > -1)
    SET_OUTPUT(X_STEP_PIN);
    WRITE(X_STEP_PIN, LOW);
    disable_x();
  #endif
  #if defined(Y_STEP_PIN) && (Y_STEP_PIN > -1)
    SET_OUTPUT(Y_STEP_PIN);
    WRITE(Y_STEP_PIN, LOW);
    disable_y();
  #endif
  #if defined(Z_STEP_PIN) && (Z_STEP_PIN > -1)
    SET_OUTPUT(Z_STEP_PIN);
    WRITE(Z_STEP_PIN, LOW);
    disable_z();
  #endif
  #if defined(E0_STEP_PIN) && (E0_STEP_PIN > -1)
    SET_OUTPUT(E0_STEP_PIN);
    WRITE(E0_STEP_PIN, LOW);
    disable_e0();
  #endif

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

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

  OCR1A = 0x4000;
  OCR1B = 0x4000;
  TCNT1 = 0;

  // ENABLE_STEPPER_DRIVER_INTERRUPT();
  sei();
}


ISR(TIMER1_COMPA_vect) {

    // xxx call it runPrintStep
    stepBuffer.runMoveSteps();
}

ISR(TIMER1_COMPB_vect) {

    // stepBuffer.runHomingSteps();
    stepBuffer.runMiscSteps();
}

#endif











