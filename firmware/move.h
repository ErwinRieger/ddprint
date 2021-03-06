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

struct XAxisSelector {
    // long position;
};
struct YAxisSelector {
    // long position;
};
struct ZAxisSelector {
    // long position;
};
struct EAxisSelector {
    // long position;
};



#if 0
template<typename MOVE>
uint16_t axis_steps_per_mm();

template<>
inline uint16_t axis_steps_per_mm<XAxisSelector>() { return AXIS_STEPS_PER_MM_X; }
template<>
inline uint16_t axis_steps_per_mm<YAxisSelector>() { return AXIS_STEPS_PER_MM_Y; }
template<>
inline uint16_t axis_steps_per_mm<ZAxisSelector>() { return AXIS_STEPS_PER_MM_Z; }
#endif

#if 0
template<typename MOVE>
long max_length_steps();

template<>
inline long max_length_steps<XAxisSelector>() { return X_MAX_LENGTH * AXIS_STEPS_PER_MM_X; }
template<>
inline long max_length_steps<YAxisSelector>() { return Y_MAX_LENGTH * AXIS_STEPS_PER_MM_Y; }
template<>
inline long max_length_steps<ZAxisSelector>() { return Z_MAX_LENGTH * AXIS_STEPS_PER_MM_Z; }
#endif

#if 0
template<typename MOVE>
uint8_t invert_dir();

template<>
inline uint8_t invert_dir<XAxisSelector>() { return -1; }
template<>
inline uint8_t invert_dir<YAxisSelector>() { return 1; }
template<>
inline uint8_t invert_dir<ZAxisSelector>() { return 1; }
#endif

template<typename MOVE>
uint8_t home_dir();

template<>
inline uint8_t home_dir<XAxisSelector>() { return -1; }
template<>
inline uint8_t home_dir<YAxisSelector>() { return 1; }
template<>
inline uint8_t home_dir<ZAxisSelector>() { return 1; }

template<typename MOVE>
uint8_t st_get_move_bit_mask();

template<>
inline uint8_t st_get_move_bit_mask<XAxisSelector>() {
    return 0x1;
}
template<>
inline uint8_t st_get_move_bit_mask<YAxisSelector>() {
    return 0x2;
}
template<>
inline uint8_t st_get_move_bit_mask<ZAxisSelector>() {
    return 0x4;
}
template<>
inline uint8_t st_get_move_bit_mask<EAxisSelector>() {
    return 0x8;
}

#if 0
template<typename MOVE>
uint8_t st_get_dir_pin();

template<>
inline uint8_t st_get_dir_pin<XAxisSelector>() {
    return X_DIR_PIN;
}
template<>
inline uint8_t st_get_dir_pin<YAxisSelector>() {
    return Y_DIR_PIN;
}
template<>
inline uint8_t st_get_dir_pin<ZAxisSelector>() {
    return Z_DIR_PIN;
}
template<>
inline uint8_t st_get_dir_pin<EAxisSelector>() {
    return E0_DIR_PIN;
}
#endif

template<typename MOVE>
void activate_dir_pin();

template<>
inline void activate_dir_pin<XAxisSelector>() {
     X_DIR_PIN :: activate();
}
template<>
inline void activate_dir_pin<YAxisSelector>() {
     Y_DIR_PIN :: activate();
}
template<>
inline void activate_dir_pin<ZAxisSelector>() {
     Z_DIR_PIN :: activate();
}
template<>
inline void activate_dir_pin<EAxisSelector>() {
     E0_DIR_PIN :: activate();
}

template<typename MOVE>
void deactivate_dir_pin();

template<>
inline void deactivate_dir_pin<XAxisSelector>() {
     X_DIR_PIN :: deActivate();
}
template<>
inline void deactivate_dir_pin<YAxisSelector>() {
     Y_DIR_PIN :: deActivate();
}
template<>
inline void deactivate_dir_pin<ZAxisSelector>() {
     Z_DIR_PIN :: deActivate();
}
template<>
inline void deactivate_dir_pin<EAxisSelector>() {
     E0_DIR_PIN :: deActivate();
}

#if 0
template<typename MOVE>
uint8_t st_read_dir_pin();

template<>
inline uint8_t st_read_dir_pin<XAxisSelector>() {
    return READ(X_DIR_PIN);
}
template<>
inline uint8_t st_read_dir_pin<YAxisSelector>() {
    return READ(Y_DIR_PIN);
}
template<>
inline uint8_t st_read_dir_pin<ZAxisSelector>() {
    return READ(Z_DIR_PIN);
}
template<>
inline uint8_t st_read_dir_pin<EAxisSelector>() {
    return READ(E0_DIR_PIN);
}
#endif

#if 0
template<typename MOVE>
uint8_t st_get_step_pin();

template<>
inline uint8_t st_get_step_pin<XAxisSelector>() {
    return X_STEP_PIN;
}
template<>
inline uint8_t st_get_step_pin<YAxisSelector>() {
    return Y_STEP_PIN;
}
template<>
inline uint8_t st_get_step_pin<ZAxisSelector>() {
    return Z_STEP_PIN;
}
template<>
inline uint8_t st_get_step_pin<EAxisSelector>() {
    return E0_STEP_PIN;
}
#endif

template<typename MOVE>
void activate_step_pin();

template<>
inline void activate_step_pin<XAxisSelector>() {
    X_STEP_PIN :: activate();
}
template<>
inline void activate_step_pin<YAxisSelector>() {
    Y_STEP_PIN :: activate();
}
template<>
inline void activate_step_pin<ZAxisSelector>() {
    Z_STEP_PIN :: activate();
}
template<>
inline void activate_step_pin<EAxisSelector>() {
    E0_STEP_PIN :: activate();
}

template<typename MOVE>
void deactivate_step_pin();

template<>
inline void deactivate_step_pin<XAxisSelector>() {
    X_STEP_PIN :: deActivate();
}
template<>
inline void deactivate_step_pin<YAxisSelector>() {
    Y_STEP_PIN :: deActivate();
}
template<>
inline void deactivate_step_pin<ZAxisSelector>() {
    Z_STEP_PIN :: deActivate();
}
template<>
inline void deactivate_step_pin<EAxisSelector>() {
    E0_STEP_PIN :: deActivate();
}

template<typename MOVE>
int st_get_home_dir();

template<>
inline int st_get_home_dir<XAxisSelector>() {
    return X_HOME_DIR;
}
template<>
inline int st_get_home_dir<YAxisSelector>() {
    return Y_HOME_DIR;
}
template<>
inline int st_get_home_dir<ZAxisSelector>() {
    return Z_HOME_DIR;
}

