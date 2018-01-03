
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
class RobSPISettings {
public:
  uint8_t spcr;
  uint8_t spsr;
};

// Some check if layout of original SPISettings and our RobSPISettings match.
// We can only compare size here, not the order of the members.
static_assert(sizeof(SPISettings) == sizeof(RobSPISettings), "SPISettings: class size mismatch");

class DDPrintSpi {

    public:

        void begin() { }

        inline void beginTransaction(SPISettings &spiSettings) {

            // Does not work, private members of SPISettings.
            // SPCR = spiSettings.spcr;
            // SPSR = spiSettings.spsr;

            SPCR = ((RobSPISettings&)spiSettings).spcr;
            SPSR = ((RobSPISettings&)spiSettings).spsr;
        }

        inline void endTransaction(void) {
        }

        inline uint8_t transfer(uint8_t data) { 
            return RealArduinoSPI.transfer(data);
        }
};

//
// Note: dDPrintSpi and (RealArduino)SPI instance are not defined (in a .cpp file).
// This works because the classes DDPrintSpi and SPIClass don't have data members.
//
extern DDPrintSpi dDPrintSpi;



