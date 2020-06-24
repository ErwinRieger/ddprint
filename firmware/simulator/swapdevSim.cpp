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


#include "ddprint.h"
#include "SdCard/SdSpiCard.h"


static FILE* swapFile = NULL;
uint32_t sdWritePos = 0;

int sdSpiCommand = -1; // no command

uint8_t SdSpiCard::cardCommand(uint8_t cmd, uint32_t arg) {

    assert (sdSpiCommand == -1);

    sdSpiCommand = cmd;

    WRITE(SDSS, LOW); // spiStart()

    if (cmd == CMD13) {

        // set flag for following m_spi.receive() call
        SPDR.prepareReceive(0);
        return 0; // OK
    }
    else if (cmd == CMD24) {

        // printf("SdSpiCard::cardCommand(CMD24), write pos: %d\n", arg);
        sdWritePos = arg;
        return 0; // OK
    }

    assert(0);
}

bool SdSpiCard::begin(SdSpiAltDriver* m_spi, unsigned char chipSelectPin, SPISettings sckRateID) {

    printf("SdSpiCard::begin(), m_spi: %p, select pin: %d, settings: %d\n", m_spi, chipSelectPin, sckRateID);

    assert(swapFile == NULL);

    // done in SdSpiCard(): m_errorCode = 0;
    type(SD_CARD_TYPE_SDHC);

    //
    // XXX not possible to open a file with O_CREAT | O_APPEND | O_RDWR | O_TRUNC with fopen.
    // Best match is mode "a+": O_RDWR|O_CREAT|O_APPEND.
    // So simulate the missing O_TRUNC flag by an additional fopen/fclose call:
    //
    // swapFile = fopen("swapfile", "w");
    // fclose(swapFile);

    // assert((swapFile = fopen("swapfile", "a+")) != NULL);
    assert((swapFile = fopen("simulator/swapfile", "w+")) != NULL);

    m_spi->begin(chipSelectPin);
    return true;
}

bool SdSpiCard::writeData(uint8_t token, const uint8_t* src) {

    // printf("Write pos: 0x%x\n", sdWritePos);

    if (fseek(swapFile, sdWritePos << 9, SEEK_SET) != 0 ) {
        assert(0);
    }

    int written = fwrite(src, 1, 512, swapFile);
    
    fflush(swapFile);

    sdSpiCommand = -1; // CMD24 done

    return written == 512;
}

bool SdSpiCard::readBlock(uint32_t blockNumber, uint8_t* dst) {

    // return SD_CARD_ERROR_CMD17;

    if (fseek(swapFile, blockNumber << 9, SEEK_SET) != 0 ) {
        assert(0);
    }

    int16_t c = fread(dst, 1, 512, swapFile);
    return c >= 0;
}

//------------------------------------------------------------------------------
uint32_t SdSpiCard::cardSize() {
  return 4194304; // 2Gb in 512byte blocks
}
//------------------------------------------------------------------------------
bool SdSpiCard::erase(uint32_t firstBlock, uint32_t lastBlock) {
  printf("SdSpiCard::eraseSingleBlockEnable(): erasing %d blocks, starting with block %d.\n", lastBlock-firstBlock, firstBlock);
  return true;
}
//------------------------------------------------------------------------------
bool SdSpiCard::eraseSingleBlockEnable() {
    printf("SdSpiCard::eraseSingleBlockEnable() called.\n");
    return true;
}
//------------------------------------------------------------------------------
bool SdSpiCard::isBusy() { return false; }

void SdSpiCard::spiStop() {
    digitalWrite(SDSS, HIGH);
}

