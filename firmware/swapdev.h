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

#include "txbuffer.h"
#include "ddcommands.h"
#include "ddlcd.h"

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


enum SwapTasks { TaskRead, TaskReadSum, TaskWrite, TaskWriteSum };

class SDSwap: public MassStorage, public Protothread {

    //
    // Writing 
    //
    uint8_t writeBuffer[SwapSectorSize];
    // Pointer to current write position (byte) in writeBuffer
    uint16_t writePos;
    // Sector number on storage device
    uint32_t writeBlockNumber;
    // Written size in bytes, multiple of SwapSectorSize
    uint32_t size;


    //
    // Reading 
    //
    // uint8_t readAhead[SwapSectorSize];
    uint8_t b1[SwapSectorSize];
    uint8_t b2[SwapSectorSize];

    // Read position, bytes, multiple of SwapSectorSize,
    // excluding the first eeprom sector.
    uint32_t readPos;
    // Number of consecutive read attempts
    uint8_t  readRetry;
    // uint8_t *readBuffer;
    uint16_t readBytes;

    enum BusyStates { idle, reading, writing };
    BusyStates busyState;

public:

    //
    // Reading 
    //
    bool cacheFilled;
    uint8_t *readBuffer;

    SDSwap() {

        busyState = idle;
        readBytes = 0;
        readBuffer = b1;
        reset();
    }

#if defined(DEBUGREADWRITE)
    struct TaskTiming ioStats[4];
#endif

    FWINLINE void discardWriteBlock() { writePos = 0; }

    FWINLINE uint32_t getSize() { return size; }

    FWINLINE uint32_t available() {

        simassert(readPos <= size);
        return size - readPos;
    }

    void reset() {
      
        while (busy())
            Run();

        readRetry = 0;

        size = 0;
        writePos = 0;

        readPos = 0;
        cacheFilled = false;

        // First sector reserved for eeprom emulation/config storage
        writeBlockNumber = 1;
    }

    FWINLINE bool busy() { 
        return busyState != idle;
    }
    
    FWINLINE bool busyWriting() { return busyState == writing; }

    FWINLINE void addByte(uint8_t b) {

        simassert(busyState != writing);
        simassert(wp < SwapSectorSize);

        writeBuffer[writePos++] = b;
    }

    //------------------------------------------------------------------------------
    // Beginning from block 1 (skipping config block)
    uint16_t readBlock() {

        if (readBuffer == b1) 
            readBuffer = b2;
        else
            readBuffer = b1;

        readPos += readBytes;
        cacheFilled = false;

        return readBytes;
    }

    // Start writing *writeBuffer* to storage device
    FWINLINE void startWriteBlock() {

#if defined(HEAVYDEBUG)
        massert(!busy());
#endif
        busyState = writing;

        Restart(); // Start thread
    }

    // Async block read/write.
    FWINLINE bool Run() {

        static int res;

        PT_BEGIN();

        if (busyState == reading) {

            //////////////////////////////////////////////////////////////////////////////////          
            TaskStart(ioStats, TaskReadSum);

            do { _ptLine = __LINE__; case __LINE__: {
                TaskStart(ioStats, TaskRead);

                if (readBuffer == b1)
                    res = readBlockWrapper((readPos >> 9)+1, b2);
                else
                    res = readBlockWrapper((readPos >> 9)+1, b1);

                TaskEnd(ioStats, TaskRead);
                if (res == 1) return true; // continue thread
                if (res == -1) { // error

                    // Return Sd2Card error code and SPI status byte from sd card.
                    // In case of SD_CARD_ERROR_CMD17 this is the return status of
                    // CMD17 in Sd2Card::cardCommand().
                    if (readRetry++ < 5) {
                        // Log error and retry
                        txBuffer.sendSimpleResponse(RespUnsolicitedMsg, RespSDReadError, errorCode(), errorData());
                        return true;
                    }

                    killMessage(RespSDReadError, errorCode(), errorData());
                    // notreached
                }

                // (res == 0), fall-through
            }
            } while (0);

            TaskEnd(ioStats, TaskReadSum);
            //////////////////////////////////////////////////////////////////////////////////          

            readBytes = STD min(available(), (uint32_t)SwapSectorSize);

            // printf("size: %d, readpos: %d, read bytes: %d\n", size, readPos, readBytes);

            // Done in readBlock()
            // readPos += readBytes;

            // Cache read successful, reset retry count
            readRetry = 0;
            busyState = idle;
            cacheFilled = true;
        }
        else if (busyState == writing) {

            simassert((size % SwapSectorSize) == 0); // block write after final partial block is invalid

            //////////////////////////////////////////////////////////////////////////////////          
            TaskStart(ioStats, TaskWriteSum);

            do { _ptLine = __LINE__; case __LINE__: {
                TaskStart(ioStats, TaskWrite);
                res = writeBlock(writeBlockNumber, writeBuffer);
                TaskEnd(ioStats, TaskWrite);
                if (res == 1) return true; // continue thread
            }
            } while (0);

            TaskEnd(ioStats, TaskWriteSum);
            //////////////////////////////////////////////////////////////////////////////////          

            size += SwapSectorSize;

            writePos = 0;

            // printf("Size: %d bytes\n", size);

            writeBlockNumber++;

            busyState = idle;
        }
        else {
            // idle
            if (!cacheFilled && available())
                busyState = reading;
        }

        PT_RESTART();
        PT_END();
    };

    void writeConfig(MSConfigBlock &config) { 

        int res;  

        TaskStart(ioStats, TaskWriteSum);

        while (true) {
           
            TaskStart(ioStats, TaskWrite);
            res = writeBlock(0, config.sector);
            TaskEnd(ioStats, TaskWrite);

            if (res == 0) { // Done
                TaskEnd(ioStats, TaskWriteSum);
                return;
            }
        }
        TaskEnd(ioStats, TaskWriteSum);
    }

    // Read first 512 byte block, the *config block*.
    void readConfig(MSConfigBlock &config) {

        int res;  

        TaskStart(ioStats, TaskReadSum);

        // BLOCKING loop for config read!
        while (true) {
           
            TaskStart(ioStats, TaskRead);
            res = readBlockWrapper(0, config.sector);

            TaskEnd(ioStats, TaskRead);

            if (res == 1) { // Continue calling
                continue;
            }

            if (res == 0) { // Done
                TaskEnd(ioStats, TaskReadSum);
                return;
            }

            // Error, nothandled yet
            massert(0);
        }
        TaskEnd(ioStats, TaskReadSum);
    }
};

extern SDSwap swapDev;
































