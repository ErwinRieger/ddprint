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
void st_write_dir_pin(bool);

template<>
inline void st_write_dir_pin<XAxisSelector>(bool v) {
    WRITE( X_DIR_PIN, v);
}
template<>
inline void st_write_dir_pin<YAxisSelector>(bool v) {
    WRITE( Y_DIR_PIN, v);
}
template<>
inline void st_write_dir_pin<ZAxisSelector>(bool v) {
    WRITE( Z_DIR_PIN, v);
}
template<>
inline void st_write_dir_pin<EAxisSelector>(bool v) {
    WRITE( E0_DIR_PIN, v);
}

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
void st_write_step_pin(uint8_t);

template<>
inline void st_write_step_pin<XAxisSelector>(uint8_t v) {
    WRITE( X_STEP_PIN, v);
}
template<>
inline void st_write_step_pin<YAxisSelector>(uint8_t v) {
    WRITE( Y_STEP_PIN, v);
}
template<>
inline void st_write_step_pin<ZAxisSelector>(uint8_t v) {
    WRITE( Z_STEP_PIN, v);
}
template<>
inline void st_write_step_pin<EAxisSelector>(uint8_t v) {
    WRITE( E0_STEP_PIN, v);
}

template<typename MOVE>
bool st_get_positive_dir();

template<>
inline bool st_get_positive_dir<XAxisSelector>() {
    return POSITIVE_X_DIR;
}
template<>
inline bool st_get_positive_dir<YAxisSelector>() {
    return POSITIVE_Y_DIR;
}
template<>
inline bool st_get_positive_dir<ZAxisSelector>() {
    return POSITIVE_Z_DIR;
}
template<>
inline bool st_get_positive_dir<EAxisSelector>() {
    return POSITIVE_E1_DIR;
}

/*
template<typename MOVE>
bool st_get_invert_step_pin();

template<>
inline bool st_get_invert_step_pin<XAxisSelector>() {
    return INVERT_X_STEP_PIN;
}
template<>
inline bool st_get_invert_step_pin<YAxisSelector>() {
    return INVERT_Y_STEP_PIN;
}
template<>
inline bool st_get_invert_step_pin<ZAxisSelector>() {
    return INVERT_Z_STEP_PIN;
}
template<>
inline bool st_get_invert_step_pin<EAxisSelector>() {
    return INVERT_E_STEP_PIN;
}
*/

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


