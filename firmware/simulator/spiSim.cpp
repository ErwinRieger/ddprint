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


#include <stdint.h>
// #include "ddprint.h"
// #include "swapdev.h"

#include "filsensorSim.h"

DDPrintSpi dDPrintSpi;

bool sdChipSelect = true; // high
extern int sdSpiCommand;
static int commandBytes = 0;
extern uint32_t sdWritePos;
// static int spiRes = -1;

RegSPDR& RegSPDR::operator= (const SimRegister &v)
{

    // printf("RegSPDR, cs: %d, spicommand: %d\n", sdChipSelect, sdSpiCommand);

    if ((sdChipSelect == LOW) && (sdSpiCommand == CMD13) && (v.value == 0xFF)) {

        // Implement behaviour for SdSpiCard::cardCommand(CMD13) SdSpiAltDriver::receive()
        value = receiveValue;
        SPSR |= (1 << SPIF);

        // Cmd13 done now
        sdSpiCommand = -1;
    }
    else
        assert(0);

    return *this;
}

#if 0
void spiSend(uint8_t b) {

    // static bool spiWrite=false;
    // static uint8_t spiAddress;
    // static int romDownLoad = 0;

    if (! sdChipSelect) {

        assert(! filSensorSim.isEnabled());

        // Data or command byte
        if ((sdSpiCommand == -1) && (b == CMD24Byte)) {

            // printf("cmd24 on\n");
            // Wait for 5 bytes, len and crc
            sdSpiCommand = b;
            sdWritePos = 0;
            commandBytes = 5;
            return;
        }

        if (sdSpiCommand == CMD24Byte) {
            if (commandBytes > 1) {
                sdWritePos += ((uint32_t)b) << ((commandBytes-2)*8);
            }
            commandBytes--;
            // printf("commbytes: %d\n",  commandBytes);
            return;
        }
    }

    if (filSensorSim.isEnabled()) {
        assert(sdChipSelect == true);
        filSensorSim.spiSend(b);
        return;
    }

    assert(0);
}
#endif

