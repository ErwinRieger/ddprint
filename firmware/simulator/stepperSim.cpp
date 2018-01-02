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

#include "Configuration.h"
#include "stepper.h"

StepperSim< XMove > ssx(X_AXIS);
// Start kopf hinten
StepperSim< YMove > ssy(Y_AXIS, Y_MAX_POS);
// Start buildplate unten
StepperSim< ZMove > ssz(Z_AXIS, Z_MAX_POS);

StepperSim< EMove > sse(E_AXIS, 0);

// StepperSim<>::viewFile = NULL;
// int StepperSim<>::layerNum = 0;
//
#if defined(WriteStepFile)
FILE *layerFile = NULL;
int layerNum = 0;
long long layerStartTime = 0;
#endif

#if 0
template <>
void checkPos<XMove>(float pos) {
    assert(pos < 90.0);
}
#endif

