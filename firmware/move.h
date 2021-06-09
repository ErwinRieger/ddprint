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

#include "Configuration.h"

enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};

struct XAxisSelector { };
struct YAxisSelector { };
struct ZAxisSelector { };
struct EAxisSelector { };

template<typename MOVE>
FWINLINE constexpr uint8_t st_get_move_bit_mask();

template<>
FWINLINE constexpr uint8_t st_get_move_bit_mask<XAxisSelector>() { return 0x1; }
template<>
FWINLINE constexpr uint8_t st_get_move_bit_mask<YAxisSelector>() { return 0x2; }
template<>
FWINLINE constexpr uint8_t st_get_move_bit_mask<ZAxisSelector>() { return 0x4; }
template<>
FWINLINE constexpr uint8_t st_get_move_bit_mask<EAxisSelector>() { return 0x8; }

template<typename MOVE>
void activate_dir_pin();

template<>
FWINLINE void activate_dir_pin<XAxisSelector>() {
     X_DIR_PIN :: activate();
}
template<>
FWINLINE void activate_dir_pin<YAxisSelector>() {
     Y_DIR_PIN :: activate();
}
template<>
FWINLINE void activate_dir_pin<ZAxisSelector>() {
     Z_DIR_PIN :: activate();
}
template<>
FWINLINE void activate_dir_pin<EAxisSelector>() {
     E0_DIR_PIN :: activate();
}

template<typename MOVE>
void deactivate_dir_pin();

template<>
FWINLINE void deactivate_dir_pin<XAxisSelector>() {
     X_DIR_PIN :: deActivate();
}
template<>
FWINLINE void deactivate_dir_pin<YAxisSelector>() {
     Y_DIR_PIN :: deActivate();
}
template<>
FWINLINE void deactivate_dir_pin<ZAxisSelector>() {
     Z_DIR_PIN :: deActivate();
}
template<>
FWINLINE void deactivate_dir_pin<EAxisSelector>() {
     E0_DIR_PIN :: deActivate();
}

template<typename MOVE>
void activate_step_pin();

template<>
FWINLINE void activate_step_pin<XAxisSelector>() {
    X_STEP_PIN :: activate();
}
template<>
FWINLINE void activate_step_pin<YAxisSelector>() {
    Y_STEP_PIN :: activate();
}
template<>
FWINLINE void activate_step_pin<ZAxisSelector>() {
    Z_STEP_PIN :: activate();
}
template<>
FWINLINE void activate_step_pin<EAxisSelector>() {
    E0_STEP_PIN :: activate();
}

template<typename MOVE>
void deactivate_step_pin();

template<>
FWINLINE void deactivate_step_pin<XAxisSelector>() {
    X_STEP_PIN :: deActivate();
}
template<>
FWINLINE void deactivate_step_pin<YAxisSelector>() {
    Y_STEP_PIN :: deActivate();
}
template<>
FWINLINE void deactivate_step_pin<ZAxisSelector>() {
    Z_STEP_PIN :: deActivate();
}
template<>
FWINLINE void deactivate_step_pin<EAxisSelector>() {
    E0_STEP_PIN :: deActivate();
}

