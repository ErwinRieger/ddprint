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

struct XMove {
    long position;
};
struct YMove {
    long position;
};
struct ZMove {
    long position;
};
struct EMove {
    long position;
};



template<typename MOVE>
uint16_t axis_steps_per_mm();

template<>
inline uint16_t axis_steps_per_mm<XMove>() { return AXIS_STEPS_PER_MM_X; }
template<>
inline uint16_t axis_steps_per_mm<YMove>() { return AXIS_STEPS_PER_MM_Y; }
template<>
inline uint16_t axis_steps_per_mm<ZMove>() { return AXIS_STEPS_PER_MM_Z; }

#if 0
template<typename MOVE>
long max_length_steps();

template<>
inline long max_length_steps<XMove>() { return X_MAX_LENGTH * AXIS_STEPS_PER_MM_X; }
template<>
inline long max_length_steps<YMove>() { return Y_MAX_LENGTH * AXIS_STEPS_PER_MM_Y; }
template<>
inline long max_length_steps<ZMove>() { return Z_MAX_LENGTH * AXIS_STEPS_PER_MM_Z; }
#endif

#if 0
template<typename MOVE>
uint8_t invert_dir();

template<>
inline uint8_t invert_dir<XMove>() { return -1; }
template<>
inline uint8_t invert_dir<YMove>() { return 1; }
template<>
inline uint8_t invert_dir<ZMove>() { return 1; }
#endif

template<typename MOVE>
uint8_t home_dir();

template<>
inline uint8_t home_dir<XMove>() { return -1; }
template<>
inline uint8_t home_dir<YMove>() { return 1; }
template<>
inline uint8_t home_dir<ZMove>() { return 1; }

template<typename MOVE>
uint8_t st_get_move_bit_mask();

template<>
inline uint8_t st_get_move_bit_mask<XMove>() {
    return 0x1;
}
template<>
inline uint8_t st_get_move_bit_mask<YMove>() {
    return 0x2;
}
template<>
inline uint8_t st_get_move_bit_mask<ZMove>() {
    return 0x4;
}
template<>
inline uint8_t st_get_move_bit_mask<EMove>() {
    return 0x8;
}

#if 0
template<typename MOVE>
uint8_t st_get_dir_pin();

template<>
inline uint8_t st_get_dir_pin<XMove>() {
    return X_DIR_PIN;
}
template<>
inline uint8_t st_get_dir_pin<YMove>() {
    return Y_DIR_PIN;
}
template<>
inline uint8_t st_get_dir_pin<ZMove>() {
    return Z_DIR_PIN;
}
template<>
inline uint8_t st_get_dir_pin<EMove>() {
    return E0_DIR_PIN;
}
#endif

template<typename MOVE>
void st_write_dir_pin(bool);

template<>
inline void st_write_dir_pin<XMove>(bool v) {
    WRITE( X_DIR_PIN, v);
}
template<>
inline void st_write_dir_pin<YMove>(bool v) {
    WRITE( Y_DIR_PIN, v);
}
template<>
inline void st_write_dir_pin<ZMove>(bool v) {
    WRITE( Z_DIR_PIN, v);
}
template<>
inline void st_write_dir_pin<EMove>(bool v) {
    WRITE( E0_DIR_PIN, v);
}

template<typename MOVE>
uint8_t st_read_dir_pin();

template<>
inline uint8_t st_read_dir_pin<XMove>() {
    return READ(X_DIR_PIN);
}
template<>
inline uint8_t st_read_dir_pin<YMove>() {
    return READ(Y_DIR_PIN);
}
template<>
inline uint8_t st_read_dir_pin<ZMove>() {
    return READ(Z_DIR_PIN);
}
template<>
inline uint8_t st_read_dir_pin<EMove>() {
    return READ(E0_DIR_PIN);
}

#if 0
template<typename MOVE>
uint8_t st_get_step_pin();

template<>
inline uint8_t st_get_step_pin<XMove>() {
    return X_STEP_PIN;
}
template<>
inline uint8_t st_get_step_pin<YMove>() {
    return Y_STEP_PIN;
}
template<>
inline uint8_t st_get_step_pin<ZMove>() {
    return Z_STEP_PIN;
}
template<>
inline uint8_t st_get_step_pin<EMove>() {
    return E0_STEP_PIN;
}
#endif

template<typename MOVE>
void st_write_step_pin(bool);

template<>
inline void st_write_step_pin<XMove>(bool v) {
    WRITE( X_STEP_PIN, v);
}
template<>
inline void st_write_step_pin<YMove>(bool v) {
    WRITE( Y_STEP_PIN, v);
}
template<>
inline void st_write_step_pin<ZMove>(bool v) {
    WRITE( Z_STEP_PIN, v);
}
template<>
inline void st_write_step_pin<EMove>(bool v) {
    WRITE( E0_STEP_PIN, v);
}

template<typename MOVE>
bool st_get_positive_dir();

template<>
inline bool st_get_positive_dir<XMove>() {
    return POSITIVE_X_DIR;
}
template<>
inline bool st_get_positive_dir<YMove>() {
    return POSITIVE_Y_DIR;
}
template<>
inline bool st_get_positive_dir<ZMove>() {
    return POSITIVE_Z_DIR;
}
template<>
inline bool st_get_positive_dir<EMove>() {
    return POSITIVE_E1_DIR;
}

template<typename MOVE>
bool st_get_invert_step_pin();

template<>
inline bool st_get_invert_step_pin<XMove>() {
    return INVERT_X_STEP_PIN;
}
template<>
inline bool st_get_invert_step_pin<YMove>() {
    return INVERT_Y_STEP_PIN;
}
template<>
inline bool st_get_invert_step_pin<ZMove>() {
    return INVERT_Z_STEP_PIN;
}
template<>
inline bool st_get_invert_step_pin<EMove>() {
    return INVERT_E_STEP_PIN;
}

template<typename MOVE>
int st_get_home_dir();

template<>
inline int st_get_home_dir<XMove>() {
    return X_HOME_DIR;
}
template<>
inline int st_get_home_dir<YMove>() {
    return Y_HOME_DIR;
}
template<>
inline int st_get_home_dir<ZMove>() {
    return Z_HOME_DIR;
}
// template<>
// inline int st_get_home_dir<EMove>() {
    // return INVERT_E0_DIR;
// }


