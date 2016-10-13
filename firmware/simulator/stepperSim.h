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

#include "ddprint.h"
#include "move.h"

////////////////////////
// CONFIG
//
// Write stepper pulses per layer.
//
#define WriteStepFile 1
// End CONFIG
////////////////////////

#if 0
template <typename MOVE>
void checkPos(float pos) {};

template <>
void checkPos<XMove>(float pos);
#endif

unsigned long long getTimestamp();

#if defined(WriteStepFile)
extern FILE *layerFile;
extern int layerNum;
extern long long layerStartTime;
#endif

template <typename MOVE>
class StepperSim {

    // static FILE *viewFile;
    // static int layerNum;

    public:
        float pos;
        float dir;
        int axis;
        bool state;
        bool enabled;

        StepperSim(int ax, float initialPos=0): enabled(0), pos(initialPos), state(false) {
            axis = ax;
        }
        void enable(bool v) { enabled = v; }
        void setPin(uint8_t v);
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

template <typename MOVE>
void StepperSim<MOVE>::setPin(uint8_t v) {

            if (enabled) {
                if (state && !v) {
                    // 1 -> 0 flanke, step
                    pos += dir;
                    // checkPos<MOVE>(pos);

                    #if defined(WriteStepFile)

                    switch (axis) {
                        case Z_AXIS:
                            // Close layer step file
                            if (layerFile) {
                                printf("closing layerfile...\n");
                                fclose(layerFile);
                                layerFile = NULL;
                            }
                            break;
                        case E_AXIS:
                            // Open layer step file if not open
                            if (! layerFile) {
                                char fn[64];
                                sprintf(fn, "layer_%03d.steps", layerNum++);
                                printf("opening layerfile...\n");
                                assert((layerFile = fopen(fn, "w")) != NULL);
                                layerStartTime = getTimestamp();
                            }
                            fprintf(layerFile, "%ld 0 %.5f 0 %.5f %.5f %5f\n", getTimestamp()-layerStartTime, ssx.pos, ssy.pos, dir, pos);
                            break;
                        case X_AXIS:
                            if (layerFile)
                                fprintf(layerFile, "%ld %.5f %.5f 0 %.5f 0 %5f\n", getTimestamp()-layerStartTime, dir, pos, ssy.pos, sse.pos);
                            break;
                        case Y_AXIS:
                            if (layerFile)
                                fprintf(layerFile, "%ld 0 %.5f %.5f %.5f 0 %5f\n", getTimestamp()-layerStartTime, ssx.pos, dir, pos, sse.pos);
                            break;
                    }
                    #endif
                }
            }
            state = v;
        }

#endif

