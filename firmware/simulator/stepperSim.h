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
// #define WriteStepFile 1
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

class ExpoFilter {

    float weight;
    float current;

  public:

    ExpoFilter(float w) {
        weight = w;
        current = 0.0;
    }

    float addValue(float v) {

        current = (weight * v + (1.0 - weight) * current);
        return current;
    }

    void reset() { current = 0.0; }
};
#endif

template<typename MOVE>
uint16_t axis_steps_per_mm();

template<>
inline uint16_t axis_steps_per_mm<XAxisSelector>() { return AXIS_STEPS_PER_MM_X; }
template<>
inline uint16_t axis_steps_per_mm<YAxisSelector>() { return AXIS_STEPS_PER_MM_Y; }
template<>
inline uint16_t axis_steps_per_mm<ZAxisSelector>() { return AXIS_STEPS_PER_MM_Z; }
template<>
inline uint16_t axis_steps_per_mm<EAxisSelector>() { return 123; }


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
        double lastStepX, lastStepY, lastStepE;

#if defined(WriteStepFile)
        ExpoFilter avgX, avgY, avgE;
#endif

        StepperSim(int ax, float initialPos=0): enabled(0), pos(initialPos), state(false)
#if defined(WriteStepFile)
            , avgX(ExpoFilter(0.30)), avgY(ExpoFilter(0.30)), avgE(ExpoFilter(0.30))
#endif
            {

            axis = ax;
        }
        void enable(bool v) { enabled = v; }
        void setPin(uint8_t v);
        void setDir(uint8_t v) {
            if (v == st_get_positive_dir<MOVE>()) {
                dir = 1.0 / axis_steps_per_mm<MOVE>();
            }
            else {
                dir = -1.0 / axis_steps_per_mm<MOVE>();
            }
        }
};


// Start: kopf links (MIN)
extern StepperSim< XAxisSelector > ssx;
// Start: kopf hinten (MAX)
extern StepperSim< YAxisSelector > ssy;
// Start: buildplate unten (MAX)
extern StepperSim< ZAxisSelector > ssz;

extern StepperSim< EAxisSelector > sse;

extern double tTimer1A;

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
                            lastStepX = lastStepY = lastStepE = tTimer1A;
                            break;
                        case E_AXIS:
                            // Open layer step file if not open
                            if (! layerFile) {
                                char fn[64];
                                sprintf(fn, "layer_%03d.steps", layerNum++);
                                printf("opening layerfile %s...\n", fn);
                                assert((layerFile = fopen(fn, "w")) != NULL);
                                // layerStartTime = getTimestamp();
                                // lastStepX = lastStepY = lastStepE = tTimer1A;
                            }
                            {
                                fprintf(layerFile, "%15.10f", tTimer1A);
                                fprintf(layerFile, " %8s %16s", "?", "?");
                                fprintf(layerFile, " %8s %16s", "?", "?");
                                fprintf(layerFile, " % 8.5f % 16.10f", dir, pos);
                                fprintf(layerFile, " %9s", "?");
                                fprintf(layerFile, " %9s", "?");
                                fprintf(layerFile, " % 9.5f", avgE.addValue(dir/(tTimer1A-lastStepE)));
                                fprintf(layerFile, "\n");
                            }

                            lastStepE = tTimer1A;
                            break;
                        case X_AXIS:
                            if (layerFile) {

                                fprintf(layerFile, "%15.10f", tTimer1A);
                                fprintf(layerFile, " % 8.5f % 16.10f", dir, pos);
                                fprintf(layerFile, " %8s %16s", "?", "?");
                                fprintf(layerFile, " %8s %16s", "?", "?");
                                fprintf(layerFile, " % 9.5f", avgX.addValue(dir/(tTimer1A-lastStepX)));
                                fprintf(layerFile, " %9s", "?");
                                fprintf(layerFile, " %9s", "?");
                                fprintf(layerFile, "\n");
                            }

                            lastStepX = tTimer1A;
                            break;
                        case Y_AXIS:
                            if (layerFile) {

                                fprintf(layerFile, "%15.10f", tTimer1A);
                                fprintf(layerFile, " %8s %16s", "?", "?");
                                fprintf(layerFile, " % 8.5f % 16.10f", dir, pos);
                                fprintf(layerFile, " %8s %16s", "?", "?");
                                fprintf(layerFile, " %9s", "?");
                                fprintf(layerFile, " % 9.5f", avgY.addValue(dir/(tTimer1A-lastStepY)));
                                fprintf(layerFile, " %9s", "?");
                                fprintf(layerFile, "\n");

                                // printf("Y: %15.10f %5d %15.10f\n", tTimer1A/2000000.0, tTimer1A-lastStep, dir/(tTimer1A-lastStep));

                            }

                            lastStepY = tTimer1A;
                            break;
                    }
                    #endif

                }
            }
            state = v;
        }

#endif

