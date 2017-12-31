/************************************************************************************************
* Note by erwin.rieger@ibrieger.de:
* This file is part of ddprint - a direct drive 3d printer firmware.
* The Origin of this code is Ultimaker2Marlin (https://github.com/Ultimaker/Ultimaker2Marlin).
************************************************************************************************/

#ifndef PINS_H
#define PINS_H

#define X_MS1_PIN -1
#define MOTOR_CURRENT_PWM_XY_PIN -1
#define MOTOR_CURRENT_PWM_Z_PIN -1
#define MOTOR_CURRENT_PWM_E_PIN -1

//
// Ultimaker 2
//
#include "pins_um2.h"
//
// Ramps 1.4
//
#include "pins_ramps.h"

#ifndef KNOWN_BOARD
#error Unknown MOTHERBOARD value in pins.h
#endif

#endif

