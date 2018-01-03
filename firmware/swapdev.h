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
#include "fastio.h"
#include "mdebug.h"

#include "ddprintspi.h"
#define SDCARD_SPI dDPrintSpi
#include "SdCard/SdSpiCard.h"

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

// #define newsdfat

#if defined(newsdfat)
// class MySpiLibDriver: public SdSpiBaseDriver
class SdSpiDriver
{

 public:
  /** Activate SPI hardware. */
  void activate() {
  }

  /** Deactivate SPI hardware. */
  void deactivate() {
  }
  /** Initialize the SPI bus.
   *
   * \param[in] csPin SD card chip select pin.
   */
  void begin(uint8_t csPin) {

    WRITE(SDSS, HIGH);
    SET_OUTPUT(SDSS);

    pinMode(SCK_PIN, OUTPUT); 
    pinMode(MOSI_PIN, OUTPUT); 

#if 0
    SPCR = (1 << SPE) | (1 << MSTR) | (3 >> 1); // Mode 0
    SPSR = 3 & 1 || 3 == 6 ? 0 : 1 << SPI2X;
#endif
  }
  /** Receive a byte.
   *
   * \return The byte.
   */
  uint8_t receive() {
    SPDR = 0XFF;
    while (!(SPSR & (1 << SPIF))) { /* Intentionally left empty */ }
    return SPDR;
  }
  /** Receive multiple bytes.
  *
  * \param[out] buf Buffer to receive the data.
  * \param[in] n Number of bytes to receive.
  *
  * \return Zero for no error or nonzero error code.
  */
  uint8_t receive(uint8_t* buf, size_t n) {
    for (size_t i = 0; i < n; i++) {
      buf[i] = SDCARD_SPI.transfer(0XFF);
    }
    return 0;
  }
  /** Send a byte.
   *
   * \param[in] data Byte to send
   */
  void send(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1 << SPIF))) { /* Intentionally left empty */ }
  }
  /** Send multiple bytes.
   *
   * \param[in] buf Buffer for data to be sent.
   * \param[in] n Number of bytes to send.
   */
  void send(const uint8_t* buf, size_t n) {
    for (size_t i = 0; i < n; i++) {
      SDCARD_SPI.transfer(buf[i]);
    }
  }
  /** Set CS low. */
  void select() {
     WRITE(SDSS, LOW);
  }
  /** Save SPISettings.
   *
   * \param[in] spiSettings SPI speed, mode, and byte order.
   */
  void setSpiSettings(SPISettings spiSettings) {
    // m_spiSettings = spiSettings;
    SPCR = spiSettings.spcr;
    SPSR = spiSettings.spsr;
  }
  /** Set CS high. */
  void unselect() {
    WRITE(SDSS, HIGH);
  }

 private:
  // SPISettings m_spiSettings;
  // uint8_t m_csPin;
};
#endif

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

#if defined(newsdfat)
    // SdSpiDriver m_spi;
#else
    // SdSpiAltDriver m_spi;
#endif
    SdSpiLibDriver m_spi;

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

        // Wait while card is busy, no timeout check!
        PT_WAIT_WHILE(isBusy());  // isBusy() is doing spiStart()/spiStop()

        if (cardCommand(CMD24, writeBlockNumber << blockShift)) { // cardCommand() is doing spiStart()

            killMessage(RespSDWriteError, SD_CARD_ERROR_CMD24, errorData());
            // notreached
        }

        if (!writeData(DATA_START_BLOCK, writeBuffer)) { // writeData() does not spiStart() but spiStop() in case of error

            killMessage(RespSDWriteError, errorCode(), errorData());
            // notreached
        }

        spiStop();

        // Wait for flash programming to complete, no timeout check!
        PT_WAIT_WHILE(isBusy()); // isBusy() is doing spiStart()/spiStop()

        if (cardCommand(CMD13, 0) || m_spi.receive()) { // cardCommand() is doing spiStart()

            killMessage(RespSDWriteError, SD_CARD_ERROR_CMD13, errorData());
            // notreached
        }

        spiStop();
            
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

