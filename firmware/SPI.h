/*
* This file is part of ddprint - a direct drive 3D printer firmware.
* 
* Copyright 2018 erwin.rieger@ibrieger.de
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

/*
 * Problem with the original Arduino SPI library:
 *
 * The Arduino SPI library masks interrupts globally while doing a SPI transaction (at
 * least with version 1.6.x and 1.8.x, see SPIClass::beginTransaction()).
 * This leads to a stuttering movement of the stepper motors since the stepper
 * interrupt is also blocked.
 */

#pragma once

#include "mdebug.h"

//
// Include real arduino SPI stuff and move Arduino SPI
// instance out of the way.
//
#define SPI RealArduinoSPI
#include "SPI/src/SPI.h"
#undef SPI

//
// Code that uses the SPI instance shold use our implementation:
//
#define SPI dDPrintSpi

// Hack to access the private spcr and spsr members of SPISettings.
class AccessSPISettings {
public:
  uint8_t spcr;
  uint8_t spsr;
};

// Some check if layout of original SPISettings and our AccessSPISettings match.
// We can only compare size here, not the order of the members.
static_assert(sizeof(SPISettings) == sizeof(AccessSPISettings), "SPISettings: class size mismatch");

class DDPrintSpi {

    public:

        void begin() { }

        FWINLINE void beginTransaction(SPISettings &spiSettings) {

            // Does not work, private members of SPISettings.
            // SPCR = spiSettings.spcr;
            // SPSR = spiSettings.spsr;

            SPCR = ((AccessSPISettings&)spiSettings).spcr;
            SPSR = ((AccessSPISettings&)spiSettings).spsr;
        }

        FWINLINE void endTransaction(void) {
        }

        FWINLINE uint8_t transfer(uint8_t data) { 
            return RealArduinoSPI.transfer(data);
        }
};

//
// Note: dDPrintSpi and (RealArduino)SPI instance are not defined (in a .cpp file).
// This works because the classes DDPrintSpi and SPIClass don't have data members.
//
extern DDPrintSpi dDPrintSpi;



