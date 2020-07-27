
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

// Registers used by SerialPort class (these are expanded
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

// Size of tx buffer in bytes
#define RX_BUFFER_SIZE 256

//
// stm32 port:
// Note: rx/tx buffer memory wasted in struct usart_dev.
//
class SerialPort //: public Stream
{

  private:

    unsigned char buffer[RX_BUFFER_SIZE];
    uint8_t head;
    uint8_t tail;

    // Length of cobs block payload
    uint8_t cobsLen;

    // Length of current cobs code block
    int16_t cobsCodeLen;

    void init();

    void atCobsBlock();

  public:
    SerialPort();

    void begin(long);
    uint8_t peekN(uint8_t index);
    void  peekChecksum(uint16_t *checksum, uint8_t count);

    void cobsInit(uint16_t payloadLength);
    bool cobsAvailable() { return (cobsLen > 0) || (cobsCodeLen == 1); }

    /* uint8_t serReadNoCheck(void) { simassert(0); } */

    uint8_t readNoCheckNoCobs(void);
    /* uint16_t readUInt16NoCheckNoCobs(); */

    uint8_t readNoCheckCobs(void);
    int16_t readInt16NoCheckCobs();
    uint16_t readUInt16NoCheckCobs();
    float readFloatNoCheckCobs();

    int32_t readInt32NoCheckCobs();

    inline void flush0(void) {
        head = tail = 0;
    }

    inline uint8_t _available(void) {
      return head - tail;
    }

    inline void store_char(unsigned char c);
};

inline void SerialPort::store_char(unsigned char c) {

        if (c == 0x0) { // SOH
            flush0();
        }
        else {
            if (head == 0) {
                // Lost SOH
                flush0();
                buffer[head++] = 0x0;
            }
#if defined(HEAVYDEBUG)
            else {
                massert(head < 255);
            }
#endif
        }

        buffer[head++] = c;
    }



extern SerialPort serialPort;





