
/*
* This file is part of ddprint - a 3D printer firmware.
* 
* Copyright 2021 erwin.rieger@ibrieger.de
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

#include "mdebug.h"

// Integer math

// XXX use a ScaledInt32 for signed values to 
// avoid type conversions.
struct ScaledUInt32 {
    uint32_t value;
    uint8_t  shift;

    int32_t mul31(int32_t f) { 

        if (f >= 0) {
            uint32_t v = value * (uint32_t)f;
            return v >> shift;
        }

        uint32_t v = value * (uint32_t)(-f);
        return -(int32_t)(v >> shift);

#if 0
        // uint32_t v = value * f;
        if (v & 0x80000000) {
            uint32_t res = ((v & 0x7fffffff) >> shift);
            return res & 0x80000000;
        }

        /// // Note: assuming compiler is using arithmetic
        /// // shift here (to preserve sign).
        return v >> shift;
#endif
    }
};

struct ScaledUInt16 {
    uint16_t value;
    uint8_t  shift;
};


// #define UINT32_SHIFT_MULT(scaledInt, f) ((scaledInt.value * (f)) >> scaledInt.shift)
// #define INT31_MUL(scaledInt, f) ((scaledInt.value * (f)) >> scaledInt.shift)



