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

bool sdChipSelect = true; // high
extern int sdSpiCommand;
static int commandBytes = 0;
extern uint32_t sdWritePos;
// static int spiRes = -1;

#define CMD24Byte (CMD24 | 0x40)

/** SPI receive a byte */
uint8_t spiRec() {

    // SPDR = 0XFF;
    // while (!(SPSR & (1 << SPIF))) { /* Intentionally left empty */ }
    // return SPDR;

    if (! sdChipSelect) {

        assert(! filSensorSim.isEnabled());

        if (sdSpiCommand == -1) {

            // if (busyWait--)
                // return 0;

            // busyWait = 5;
            return 0xff;
        }

        if ((sdSpiCommand == CMD24Byte) && (commandBytes == 0)) {
            // printf("cmd24 off\n");
            sdSpiCommand = -1;
            return 0;
        }

        if (sdSpiCommand == CMD13) {
            sdSpiCommand = -1;
            return 0;
        }
    }

    if (filSensorSim.isEnabled()) {
        assert(sdChipSelect == true);
        return filSensorSim.spiRec();
    }

    assert(0);
}

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

//------------------------------------------------------------------------------
void Sd2Card::chipSelectHigh() {
  sdChipSelect = true;
}
//------------------------------------------------------------------------------
void Sd2Card::chipSelectLow() {
  sdChipSelect = false;
}
//------------------------------------------------------------------------------
