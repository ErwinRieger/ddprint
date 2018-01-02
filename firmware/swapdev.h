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

#include <Arduino.h>

#include "SdCard/SdSpiCard.h"

#include "Protothread.h"
#include "Configuration.h"
#include "pins.h"
#include "fastio.h"
#include "mdebug.h"

#if defined(DDSim)
    #define STD std::
#else
    #define STD
#endif

// Redefined here from ddcommands.h
#define RespSDReadError         9  
#define RespSDWriteError       12 

// The buffersizes for reading and writing to th SD card
#define RCMD_BUFFER_SIZE 512
// #define WCMD_BUFFER_SIZE 1024
#define WCMD_BUFFER_SIZE 512

// uint8_t const SD_CARD_ERROR_SETBLOCKSIZE = 0X21;
// uint8_t const CMD16 = 0X10;

class SDSwap: public SdSpiCard, public Protothread {

    uint8_t writeBuffer[WCMD_BUFFER_SIZE];
    // Pointer to current write position in writeBuffer
    uint16_t writePos;

    bool busyWriting;
    uint32_t writeBlockNumber;
    uint32_t readPos;

    // Written size in bytes, multiple of WCMD_BUFFER_SIZE
    uint32_t size;

    // Number of bytes to shift the block address for non-sdhc cards
    uint8_t blockShift;

    // SdSpiAltDriver m_spi;
    SdSpiDriver m_spi;

public:

    SDSwap() {

        busyWriting = false;
        reset();
    }

    bool swapInit() {

        if (! begin(&m_spi, SDSS, SPI_FULL_SPEED)) {

            return false;
        }

        if (type() == SD_CARD_TYPE_SDHC) {
            blockShift = 0;
        } else {
            blockShift = 9;
        }

        // // Set R/W block size.
        // if ( cardCommand(CMD16, 512) ) {
            // error(SD_CARD_ERROR_SETBLOCKSIZE);
            // return false;
        // }

        massert(eraseSingleBlockEnable());

        return true;
    }

    // FWINLINE uint32_t getWriteBlockNumber() { return writeBlockNumber; }
    FWINLINE uint16_t getWritePos() { return writePos; }

    FWINLINE uint32_t getReadPos() { return readPos; }
    FWINLINE uint32_t getSize() { return size; }

    FWINLINE uint32_t available() {

        simassert(readPos <= size);
        return size - readPos;
    }

    void reset() {
        
        while (busyWriting)
            Run();

        readPos = 0;
        writeBlockNumber = 0;
        writePos = 0;
        size = 0;
    }

    FWINLINE bool isBusyWriting() { return busyWriting; }

    FWINLINE void addByte(uint8_t b) {

        simassert(! busyWriting);
        simassert(writePos < WCMD_BUFFER_SIZE);

        writeBuffer[writePos++] = b;

        if (writePos == WCMD_BUFFER_SIZE)
            writeBlock();
    }

#if 0
    void setWritePos(uint32_t wbn, uint16_t writePos_) {

        if (wbn != writeBlockNumber) {
            // read back block

            simassert(! busyWriting);
            simassert(readPos < (wbn<<9));

            if (! SdSpiCard::readBlock(wbn, writeBuffer)) {

                // xxx errorhandling here
                massert(0);
                return;
            }

            writeBlockNumber = wbn;
        }

        simassert((readPos % WCMD_BUFFER_SIZE) <= writePos_);
        writePos = writePos_;

        size = wbn << 9;
    }
#endif

    //------------------------------------------------------------------------------
    FWINLINE void writeBlock() {

#if defined(HEAVYDEBUG)
        massert(! busyWriting);
#endif
        busyWriting = true;
    }

    FWINLINE int16_t readBlock(uint8_t* dst) {

        // Number of bytes in this block:
        massert(size > readPos);
        massert(! busyWriting);
        massert((readPos % RCMD_BUFFER_SIZE) == 0); // read pos should be block-aligned

        if (! SdSpiCard::readBlock(readPos >> 9, dst)) {

            // XXX how shold we handle this? Retry the read?
            // Note: readBlockEC() retried three times already.

            // Return Sd2Card error code and SPI status byte from sd card.
            // In case of SD_CARD_ERROR_CMD17 this is the return status of
            // CMD17 in Sd2Card::cardCommand().
            killMessage(RespSDReadError, errorCode(), errorData());
            // notreached
        }

        uint16_t readBytes = STD min((uint32_t)(size - readPos), (uint32_t)512);

        // printf("size: %d, readpos: %d, read bytes: %d\n", size, readPos, readBytes);

        readPos += readBytes;

        return readBytes;
    }

    // Async block write.
    bool Run() {

        PT_BEGIN();

        PT_WAIT_UNTIL(busyWriting);

        simassert((size % WCMD_BUFFER_SIZE) == 0); // block write after final partial block is invalid

        //////////////////////////////////////////////////////////////////////////////////          
        // if (cardCommand(CMD24, writeBlockNumber)) {
            // error(SD_CARD_ERROR_CMD24);
            // chipSelectHigh();
            // PT_EXIT(); // xxx or use pt_restart here?
        // }

        // select card
        // chipSelectLow();
        spiStart();

        // wait up to 300 ms if busy
        // xxx no timeout yet
        // waitNotBusy(300);
        // PT_WAIT_UNTIL(spiRec() == 0XFF);
        // xxx changed 1
        PT_WAIT_UNTIL(m_spi.receive() == 0XFF);

        // massert(spiRec() == 0XFF);
        // massert(m_spi.receive() == 0XFF);

        // send command
        // spiSend(CMD24 | 0x40);
        m_spi.send(CMD24 | 0x40);

        // send argument, 4 byte sequence
        // for (int8_t s = 24; s >= 0; s -= 8) spiSend((writeBlockNumber << blockShift) >> s);
        for (int8_t s = 24; s >= 0; s -= 8) m_spi.send((writeBlockNumber << blockShift) >> s);

        // send CRC
        // spiSend(0XFF);
        m_spi.send(0XFF);

        // wait for response 
        // for (uint8_t i = 0; ((status_ = spiRec()) & 0X80) && i != 0XFF; i++) { /* Intentionally left empty */ }
        for (uint8_t i = 0; ((m_status = m_spi.receive()) & 0X80) && i != 0XFF; i++) { /* Intentionally left empty */ }

        // return status_;
        // massert(status_ == 0); // xxx set errorCode_

        // xxx changed 2
        // massert(m_status == 0); // xxx set errorCode_
        if (m_status != 0) {
        }

        //////////////////////////////////////////////////////////////////////////////////          

        if (!writeData(DATA_START_BLOCK, writeBuffer)) {
            // writeData raises chipselect on error
            massert(0); // xxx set errorCode_
        }

        //////////////////////////////////////////////////////////////////////////////////          

        // wait for flash programming to complete
        // if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
            // error(SD_CARD_ERROR_WRITE_TIMEOUT);
            // goto fail;
        // }
        // xxx no timeout yet
        // PT_WAIT_UNTIL(notBusy());
        PT_WAIT_WHILE(isBusy());

        // response is r2 so get and check two bytes for nonzero
        // xxx cardCommand() does some unneccessary stuff like waitNotBusy();

        // if (cardCommand(CMD13, 0) || spiRec()) {
        if (cardCommand(CMD13, 0) || m_spi.receive()) {
            // error(SD_CARD_ERROR_WRITE_PROGRAMMING);
            // error(SD_CARD_ERROR_CMD13);
            // goto fail;
            // chipSelectHigh();
            // spiStop();
            // PT_EXIT(); // xxx or use pt_restart here?
            killMessage(RespSDWriteError, SD_CARD_ERROR_CMD13, errorData());
            // notreached
        }

        // chipSelectHigh();
        spiStop();
            
        // return true;
        //////////////////////////////////////////////////////////////////////////////////          

        size += writePos;

#if defined(HEAVYDEBUG)
        massert(size == (writeBlockNumber*512 + writePos));
#endif
        // printf("Size: %d bytes\n", size);

        writeBlockNumber++;
        writePos = 0;
        busyWriting = false;

        PT_RESTART();
        PT_END();
    };
};

extern SDSwap swapDev;

