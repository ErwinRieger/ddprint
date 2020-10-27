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

#include "Protothread.h"
#include "Configuration.h"
#include "pins.h"

#include "ddserial.h"
#include "ddcommands.h"

// Redefined here from ddcommands.h
#define RespSDReadError         9  
// #define RespSDWriteError       12 


// 
// About read/write times
//
// Max time allowed to write and read sector:
//
// UM2:
//  * Fastest axis is X/Y with 250*80 = 20000 steps/s = 50 us pulse width
//  * Step buffer drained in 255*50 = 12.75 ms
//
// JP:
//  * Fastest axis is X/Y with 250*160 = 40000 steps/s = 25 us pulse width
//  * Step buffer drained in 255*25 = 6.4 ms
//
//



enum SwapTasks { TaskRead, TaskWrite };

class SDSwap: public MassStorage, public Protothread {

    uint8_t writeBuffer[SwapSectorSize];

    // Pointer to current write position (byte) in writeBuffer
    uint16_t writePos;

    bool busyWriting;

    // Sector number on storage device
    uint32_t writeBlockNumber;

    // Read position, bytes, multiple of SwapSectorSize,
    // excluding the first eeprom sector.
    uint32_t readPos;

    // Number of consecutive read attempts
    uint8_t  readRetry;

    // Written size in bytes, multiple of SwapSectorSize
    uint32_t size;

public:

    SDSwap() {

        busyWriting = false;
        reset();
    }

    struct TaskTiming ioStats[2];

    // FWINLINE uint32_t getWriteBlockNumber() { return writeBlockNumber; }
    FWINLINE uint16_t getWritePos() { return writePos & SwapSectorMask; }

    // FWINLINE uint32_t getReadPos() { return readPos; }
    FWINLINE uint32_t getSize() { return size; }

    FWINLINE uint32_t available() {

        simassert(readPos <= size);
        return size - readPos;
    }

    void reset() {
        
        while (busyWriting)
            Run();

        readRetry = 0;
        size = 0;
        writePos = 0;

        // writeBlockNumber = 0;
        readPos = 0;

        // First sector reserved for eeprom emulation
        writeBlockNumber = 1;
    }

    FWINLINE bool isBusyWriting() { return busyWriting; }

    FWINLINE void addByte(uint8_t b) {

        uint16_t wp = getWritePos();

        simassert(! busyWriting);
        simassert(wp < SwapSectorSize);

        writeBuffer[wp] = b;
        writePos++;

        // Block is full if write index
        // overflowed to zero.
        if (getWritePos() == 0)
            startWriteBlock();
    }

    //
    // Add a byte to our writebuffer, just like addByte(),
    // but does the *back-reference* handling of the inflate
    // algorithm. This works hand-in-hand with the UnZipper
    // tread.
    //
    // Note: buffer is written asynchronous to storage device. We use data
    // partly new and partly flushed to mass storage.
    // 
    FWINLINE void addBackRefByte(uint8_t offs) {

        uint16_t wp = getWritePos();

        simassert(! busyWriting);
        simassert(wp < SwapSectorSize);

        uint8_t b = writeBuffer[(writePos - offs) & SwapSectorMask]; // Overflow expected
        writeBuffer[wp] = b;
        writePos++;

        // Block is full if write index
        // overflowed to zero.
        if (getWritePos() == 0)
            startWriteBlock();
    }

    //------------------------------------------------------------------------------
    // Start writing *writeBuffer* to storage device
    FWINLINE void startWriteBlock() {

#if defined(HEAVYDEBUG)
        massert(! busyWriting);
#endif
        busyWriting = true;
    }

    FWINLINE int16_t readBlock(uint8_t* dst) {

        // Number of bytes in this block:
        massert(available() > 0);
        massert(! busyWriting);
        massert((readPos % SwapSectorSize) == 0); // read pos should be block-aligned

        TaskStart(ioStats, TaskRead);
        if (! MassStorage::readBlock((readPos >> 9)+1, dst)) {

            // Return Sd2Card error code and SPI status byte from sd card.
            // In case of SD_CARD_ERROR_CMD17 this is the return status of
            // CMD17 in Sd2Card::cardCommand().

            if (readRetry++ < 5) {
                // Log error and retry
                txBuffer.sendSimpleResponse(RespUnsolicitedMsg, RespSDReadError, errorCode(), errorData());
                return 0;
            }

            killMessage(RespSDReadError, errorCode(), errorData());
            // notreached
        }
        TaskEnd(ioStats, TaskRead);

        // xxx hack
        massert(GetTaskDuration(ioStats, TaskRead) <= 5);

        uint16_t readBytes = STD min(available(), (uint32_t)SwapSectorSize);

        // printf("size: %d, readpos: %d, read bytes: %d\n", size, readPos, readBytes);

        readPos += readBytes;

        // Read was successful, reset retry count
        readRetry = 0;

        return readBytes;
    }

    // Async block write.
    bool Run() {

        uint16_t wp;

        PT_BEGIN();

        PT_WAIT_UNTIL(busyWriting);

        simassert((size % SwapSectorSize) == 0); // block write after final partial block is invalid



        // xxx hack
   // uint32_t t = GetTaskDuration(ioStats, TaskWrite);

        //////////////////////////////////////////////////////////////////////////////////          
        TaskStart(ioStats, TaskWrite);
        PT_WAIT_WHILE(writeBlock(writeBlockNumber, writeBuffer));
        TaskEnd(ioStats, TaskWrite);
        //////////////////////////////////////////////////////////////////////////////////          

        // Update size 
        wp = getWritePos();
        if (wp) { // Last block

            size += wp;
            writePos = 0;
            #if defined(HEAVYDEBUG)
                massert(size == ((writeBlockNumber-1)*SwapSectorSize + wp));
            #endif
        }
        else {
            size += SwapSectorSize;
            #if defined(HEAVYDEBUG)
                massert(size == (writeBlockNumber*SwapSectorSize));
            #endif
        }

        // printf("Size: %d bytes\n", size);

        writeBlockNumber++;
        busyWriting = false;

        PT_RESTART();
        PT_END();
    };

    void writeConfig(MSConfigBlock &config) { while (writeBlock(0, config.sector)); }
    void readConfig(MSConfigBlock &config) { MassStorage::readBlock(0, config.sector); }
};

extern SDSwap swapDev;


