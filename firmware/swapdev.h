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


enum SwapTasks { TaskRead, TaskWrite, TaskWriteSum };

class SDSwap: public MassStorage, public Protothread {

    uint8_t writeBuffer[SwapSectorSize];

    // Pointer to current write position (byte) in writeBuffer
    uint16_t writePos;

    // bool busyWriting;
    int busyWriting;

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

        busyWriting = 0;
        reset();
    }

    struct TaskTiming ioStats[3];

    // FWINLINE uint32_t getWriteBlockNumber() { return writeBlockNumber; }
    FWINLINE uint16_t getWritePos() { return writePos & SwapSectorMask; }

    // FWINLINE uint32_t getReadPos() { return readPos; }
    FWINLINE uint32_t getSize() { return size; }

    FWINLINE uint32_t available() {

        simassert(readPos <= size);
        return size - readPos;
    }

    void reset() {
      
        if (busyWriting == 2)
            busyWriting = 1;

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

    FWINLINE bool isBusyWritingForRead() { return busyWriting == 1; }
    FWINLINE bool isBusyWritingForWrite() { return busyWriting >= 1; }

    int getBusyWriting() { return busyWriting; }
    void setBusyWriting(int b) { busyWriting = b; }

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
        busyWriting = 1;
    }

    FWINLINE int16_t readBlock(uint8_t* dst) {

        // Number of bytes in this block:
        massert(available() > 0);
        massert(busyWriting != 1);
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

        if (busyWriting == 2) {
            busyWriting = 1;  // Retry
        }

        return readBytes;
    }

    // Async block write.
    bool Run() {

        uint16_t wp;
        static int res;
        static USBH_Status usbstatus;

        PT_BEGIN();

        PT_WAIT_UNTIL(busyWriting == 1);

        simassert((size % SwapSectorSize) == 0); // block write after final partial block is invalid

        //////////////////////////////////////////////////////////////////////////////////          
        TaskStart(ioStats, TaskWrite);
        TaskStart(ioStats, TaskWriteSum);

        // PT_WAIT_WHILE((res = writeBlock(writeBlockNumber, writeBuffer, GetTaskDuration(ioStats, TaskWriteSum))) == 1);
        while ((res = writeBlock(writeBlockNumber, writeBuffer, GetTaskDuration(ioStats, TaskWriteSum))) == 1) {
            TaskEnd(ioStats, TaskWrite);
        }
        TaskEnd(ioStats, TaskWrite);

        if (res == -1) {

                massert(0);
            // Clean up and retry fresh write command later
            PT_WAIT_WHILE((usbstatus = USBH_MSC_BlockReset(&USB_OTG_Core_Host, &USB_Host)) == USBH_BUSY);
            debugUSBHStatus(usbstatus);

            PT_WAIT_WHILE((usbstatus = USBH_MSC_BOT_Abort(&USB_OTG_Core_Host, &USB_Host, USBH_MSC_DIR_IN)) == USBH_BUSY);
            debugUSBHStatus(usbstatus);

            PT_WAIT_WHILE((usbstatus = USBH_MSC_BOT_Abort(&USB_OTG_Core_Host, &USB_Host, USBH_MSC_DIR_OUT)) == USBH_BUSY);
            debugUSBHStatus(usbstatus);
    
            // Don't update size 
            // Don't change write buffer
            busyWriting = 2;
        }
        else {

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
            busyWriting = 0;
        }

        TaskEnd(ioStats, TaskWriteSum);

        PT_RESTART();
        PT_END();
    };

    void writeConfig(MSConfigBlock &config) { 

        enum WriteState { WriteBlock, blockReset, blockAbortIn, blockAbortOut };

        int res;  
        static USBH_Status usbstatus;

        TaskStart(ioStats, TaskWriteSum);

        WriteState writeState = WriteBlock;

        while (true) {
           
           switch (writeState) {
                case WriteBlock:
                    TaskStart(ioStats, TaskWrite);
                    res = writeBlock(0, config.sector, GetTaskDuration(ioStats, TaskWriteSum));

//
// xxx timout mit 30 ms hier in TaskEnd:
//
//
// usbh_msc.BOTState: USBH_BOTSTATE_DECODE_CSW
//
// here: taskStart = 36493
// readBlock.start_time_1: 36493
// readBlock.start_time_2: 36493
//
// XXX dazwischen return von USBH_MSC_Write10() ->
//  readBlock() -> hierher
//  und dann 30 ms später wieder aufruf von 
//  readBlock()/USBH_MSC_Write10() ?
//
// readBlock.start_time_3: 36523 30 ms später !
//
//
//normal:
//  {tsqlvl = 41, hcintr = 804, portintr = 0}
//  {tsqlvl = 13, hcintr = 602, portintr = 0}
//
//aber auch:
//  {tsqlvl = 53, hcintr = 15860, portintr = 0}
//
                    TaskEndX(ioStats, TaskWrite);

                    if (res == -1) { // Timeout error
                        writeState = blockReset;
                    }
                    else if (res == 0) { // Done, USBH_OK from USBH_MSC_Write10/USBH_OK from dd_USBH_MSC_HandleBOTXfer
                        TaskEnd(ioStats, TaskWriteSum);
                        return;
                    }
                    break;
                case blockReset:
                    TaskStart(ioStats, TaskWrite);
                    usbstatus = USBH_MSC_BlockReset(&USB_OTG_Core_Host, &USB_Host);
                    TaskEndX(ioStats, TaskWrite);

                    if (usbstatus != USBH_BUSY) {
                        debugUSBHStatus(usbstatus);
                        writeState = blockAbortIn;
                    }
                    break;
                case blockAbortIn:
                    TaskStart(ioStats, TaskWrite);
                    usbstatus = USBH_MSC_BOT_Abort(&USB_OTG_Core_Host, &USB_Host, USBH_MSC_DIR_IN);
                    TaskEndX(ioStats, TaskWrite);

                    if (usbstatus != USBH_BUSY) {
                        debugUSBHStatus(usbstatus);
                        writeState = blockAbortOut;
                    }
                    break;
                case blockAbortOut:
                    TaskStart(ioStats, TaskWrite);
                    usbstatus = USBH_MSC_BOT_Abort(&USB_OTG_Core_Host, &USB_Host, USBH_MSC_DIR_OUT);
                    TaskEndX(ioStats, TaskWrite);

                    if (usbstatus != USBH_BUSY) {
                        debugUSBHStatus(usbstatus);
                        TaskEnd(ioStats, TaskWriteSum);
                        return;
                    }
                    break;
            }
        }
        TaskEnd(ioStats, TaskWriteSum);
    }
    void readConfig(MSConfigBlock &config) {
        TaskStart(ioStats, TaskRead);
        MassStorage::readBlock(0, config.sector);
        TaskEnd(ioStats, TaskRead);
    }
};

extern SDSwap swapDev;
































