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

    uint8_t writeBuffer[SwapSectorSize];

    // Pointer to current write position (byte) in writeBuffer
    uint16_t writePos;

    enum BusyStates { idle, reading, writing };
    BusyStates busyState;

    // Sector number on storage device
    uint32_t writeBlockNumber;

    // Read position, bytes, multiple of SwapSectorSize,
    // excluding the first eeprom sector.
    uint32_t readPos;

    // Number of consecutive read attempts
    uint8_t  readRetry;

    // Written size in bytes, multiple of SwapSectorSize
    uint32_t size;

    uint8_t *readBuffer;
    uint16_t readBytes;

public:

    SDSwap() {

        busyState = idle;
        readBytes = 0;
        reset();
    }

#if defined(DEBUGREADWRITE)
    struct TaskTiming ioStats[4];
#endif

    // FWINLINE uint32_t getWriteBlockNumber() { return writeBlockNumber; }
    FWINLINE uint16_t getWritePos() { return writePos & SwapSectorMask; }

    // FWINLINE uint32_t getReadPos() { return readPos; }
    FWINLINE uint32_t getSize() { return size; }

    FWINLINE uint16_t getReadLen() { return readBytes; }

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

        // writeBlockNumber = 0;
        readPos = 0;

        // First sector reserved for eeprom emulation
        writeBlockNumber = 1;
    }

    FWINLINE bool busy() { return busyState != idle; }
    // FWINLINE bool busyR() { return busyState == reading; }
    // FWINLINE bool busyW() { return busyState != writing; }

    // void setBusyWriting(int b) { busyWriting = b; }

    FWINLINE void addByte(uint8_t b) {

        uint16_t wp = getWritePos();

        simassert(busyState != writing);
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

        simassert(busyState != writing);
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
    // Start reading *readBuffer* from storage device
    FWINLINE void startReadBlock(uint8_t *buf) {

#if defined(HEAVYDEBUG)
        massert(!busy());
#endif

        massert(available() > 0);
        massert((readPos % SwapSectorSize) == 0); // read pos should be block-aligned

        readBuffer = buf;

        readBytes = 0;

        busyState = reading;

        Restart(); // Start thread
    }

    // Start writing *writeBuffer* to storage device
    FWINLINE void startWriteBlock() {

#if defined(HEAVYDEBUG)
        massert(!busy());
#endif
        busyState = writing;

        Restart(); // Start thread
    }

#if 0
    FWINLINE int16_t _readBlock(uint8_t* dst) {

        // Number of bytes in this block:
#if defined(DEBUGPROCSTAT)
        TaskStart(ioStats, TaskRead);
#endif

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

#if defined(DEBUGPROCSTAT)
        TaskEnd(ioStats, TaskRead);
#endif

#if defined(DEBUGREADWRITE)
        // xxx hack
        massert(GetTaskDuration(ioStats, TaskRead) <= 5);
#endif

        uint16_t bytesRead = STD min(available(), (uint32_t)SwapSectorSize);

        // printf("size: %d, readpos: %d, read bytes: %d\n", size, readPos, bytesRead);

        readPos += bytesRead;

        // Read was successful, reset retry count
        readRetry = 0;

        return bytesRead;
    }
#endif

    // Async block write.
    bool Run() {

        uint16_t wp;
        static int res;

        PT_BEGIN();

        if (busyState == reading) {

            //////////////////////////////////////////////////////////////////////////////////          
            TaskStart(ioStats, TaskReadSum);

            do { _ptLine = __LINE__; case __LINE__: {
                TaskStart(ioStats, TaskRead);
                res = readBlockWrapper((readPos >> 9)+1, readBuffer);
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

            readPos += readBytes;

            // Read was successful, reset retry count
            readRetry = 0;
            busyState = idle;
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
            busyState = idle;
        }


        // PT_RESTART();
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

            // Error, xxx nothandled
            massert(0);
        }
        TaskEnd(ioStats, TaskReadSum);
    }
};

extern SDSwap swapDev;
































