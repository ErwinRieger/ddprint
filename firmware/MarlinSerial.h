
/************************************************************************************************
* Note by erwin.rieger@ibrieger.de:
* This file is part of ddprint - a direct drive 3d printer firmware.
* The Origin of this code is Ultimaker2Marlin (https://github.com/Ultimaker/Ultimaker2Marlin).
************************************************************************************************/

/*
  HardwareSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 28 September 2010 by Mark Sproul
*/

#pragma once

#include <stdint.h>
#include "mdebug.h"
#include "ddmacro.h"

#if !defined(SERIAL_PORT)
#define SERIAL_PORT 0
#endif

// These are macros to build serial port register names for the selected SERIAL_PORT (C preprocessor
// requires two levels of indirection to expand macro values properly)
#define SERIAL_REGNAME(registerbase,number,suffix) SERIAL_REGNAME_INTERNAL(registerbase,number,suffix)
#if SERIAL_PORT == 0 && (!defined(UBRR0H) || !defined(UDR0)) // use un-numbered registers if necessary
#define SERIAL_REGNAME_INTERNAL(registerbase,number,suffix) registerbase##suffix
#else
#define SERIAL_REGNAME_INTERNAL(registerbase,number,suffix) registerbase##number##suffix
#endif

// Registers used by MarlinSerial class (these are expanded
// depending on selected serial port
#define M_UCSRxA SERIAL_REGNAME(UCSR,SERIAL_PORT,A) // defines M_UCSRxA to be UCSRnA where n is the serial port number
#define M_UCSRxB SERIAL_REGNAME(UCSR,SERIAL_PORT,B)
#define M_RXENx SERIAL_REGNAME(RXEN,SERIAL_PORT,)
#define M_TXENx SERIAL_REGNAME(TXEN,SERIAL_PORT,)
#define M_RXCIEx SERIAL_REGNAME(RXCIE,SERIAL_PORT,)
#define M_UDREx SERIAL_REGNAME(UDRE,SERIAL_PORT,)
#define M_UDRx SERIAL_REGNAME(UDR,SERIAL_PORT,)
#define M_UBRRxH SERIAL_REGNAME(UBRR,SERIAL_PORT,H)
#define M_UBRRxL SERIAL_REGNAME(UBRR,SERIAL_PORT,L)
#define M_RXCx SERIAL_REGNAME(RXC,SERIAL_PORT,)
#define M_USARTx_RX_vect SERIAL_REGNAME(USART,SERIAL_PORT,_RX_vect)
#define M_U2Xx SERIAL_REGNAME(U2X,SERIAL_PORT,)

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which rx_buffer_head is the index of the
// location to which to write the next incoming character and rx_buffer_tail
// is the index of the location from which to read.
// #define RX_BUFFER_SIZE 128
// #define RX_BUFFER_SIZE 1024
#define RX_BUFFER_SIZE 512
#define RX_BUFFER_MASK (RX_BUFFER_SIZE - 1)

struct ring_buffer
{
  unsigned char buffer[RX_BUFFER_SIZE];
  int16_t head;
  int16_t tail;
};

class MarlinSerial //: public Stream
{

  private:

    ring_buffer rxBuffer;

    // Length of cobs block payload
    uint8_t cobsLen;

  public: //XXXXXXXXXXXXX
    // Length of current cobs code block
    int16_t cobsCodeLen;

    void atCobsBlock();
    uint8_t getCobsByte();

  public:
    MarlinSerial();

    void begin(long);
    uint8_t peekN(uint8_t index);
    void  peekChecksum(uint16_t *checksum, uint8_t count);

    void cobsInit(uint16_t payloadLength);
    bool cobsAvailable() { return (cobsLen > 0) || (cobsCodeLen == 1); }
    // bool cobsAvailable() { return cobsLen > 0; }

    uint8_t serReadNoCheck(void) { simassert(0); }

    uint8_t readNoCheckNoCobs(void);
    uint16_t readUInt16NoCheckNoCobs();

    uint8_t readNoCheckCobs(void);
    uint16_t readUInt16NoCheckCobs();
    float readFloatNoCheckCobs();

    uint16_t serReadUInt16();
    int32_t readInt32NoCheckCobs();
    uint32_t serReadUInt32();

    uint16_t tellN(uint8_t n) { CRITICAL_SECTION_START; uint16_t res = (rxBuffer.tail + n) & RX_BUFFER_MASK; CRITICAL_SECTION_END; return res; }
    uint16_t getRXTail() { CRITICAL_SECTION_START; uint16_t res = rxBuffer.tail; CRITICAL_SECTION_END; return res; }

    void flush(void);
    void flush(uint8_t);

    inline int _available(void) {
      CRITICAL_SECTION_START; uint16_t res = (RX_BUFFER_SIZE + rxBuffer.head - rxBuffer.tail) & RX_BUFFER_MASK; CRITICAL_SECTION_END; return res;
    }

    inline void store_char(unsigned char c);

  public:

    // Bitmask of usart error status bits (frame-/overrun- or parity error)
    uint8_t rxerror;

    // Return bitmask of usart error status bits (frame-/overrun- or parity error)
    // and reset error status.
    uint8_t getError() {
        uint8_t e = rxerror;
        rxerror = 0;
        return e;
    }
};

extern MarlinSerial MSerial;

inline void MarlinSerial::store_char(unsigned char c) {
        int i = (rxBuffer.head + 1) & RX_BUFFER_MASK;

        // if we should be storing the received character into the location
        // just before the tail (meaning that the head would advance to the
        // current location of the tail), we're about to overflow the buffer
        // and so we don't write the character or advance the head.
        if (i != rxBuffer.tail) {
            rxBuffer.buffer[rxBuffer.head] = c;
            rxBuffer.head = i;
        }
        else {
            massert(0);
        }
    }


#if defined(DDSim)
    #include "MarlinSerialSim.h"
#endif


