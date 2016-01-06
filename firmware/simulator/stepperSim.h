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

#if defined(DDSim)

#include <stdint.h>
#include <assert.h>

#if 0
template <typename MOVE>
void checkPos(float pos) {};

template <>
void checkPos<XMove>(float pos);
#endif

template <typename MOVE>
class StepperSim {
    public:
        float pos;
        float dir;
        // float plusdir;
        // float minusdir;
        // int axis;
        bool state;
        bool enabled;

        int count;

        StepperSim(float initialPos=0): enabled(0), pos(initialPos), state(false), count(0) {
        }
        void enable(bool v) { enabled = v; }
        void setPin(uint8_t v) {

            if (enabled) {
                if (state && !v) {
                    // 1 -> 0 flanke, step
                    pos += dir;
                    // if ((count++ % 10) == 0) printf("pos: %.1f\n", pos);
                    // checkPos<MOVE>(pos);
                }
            }
            state = v;
        }
        void setDir(uint8_t v) {

            if (v == st_get_invert_dir<MOVE>()) {
                dir = -1.0 / axis_steps_per_mm<MOVE>();
            }
            else {
                dir = 1.0 / axis_steps_per_mm<MOVE>();
            }
        }
};


// Start: kopf links (MIN)
extern StepperSim< XMove > ssx;
// Start: kopf hinten (MAX)
extern StepperSim< YMove > ssy;
// Start: buildplate unten (MAX)
extern StepperSim< ZMove > ssz;

extern StepperSim< EMove > sse;


#endif

