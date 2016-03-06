
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

#include <avr/pgmspace.h>

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


#define MSG_OK "ok"

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0


// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which rx_buffer_head is the index of the
// location to which to write the next incoming character and rx_buffer_tail
// is the index of the location from which to read.
// #define RX_BUFFER_SIZE 128
// #define RX_BUFFER_SIZE 1024
#define RX_BUFFER_SIZE 512

const char errormagic[] PROGMEM ="Error:";
const char echomagic[] PROGMEM ="echo:";

void serialprintPGM(const char *str);

struct ring_buffer
{
  unsigned char buffer[RX_BUFFER_SIZE];
  int head;
  int tail;
};

// extern ring_buffer rx_buffer;

class MarlinSerial //: public Stream
{

  public:
    MarlinSerial();
    void begin(long);
    uint8_t peekN(uint8_t index);
    void  peekChecksum(uint16_t *checksum, uint8_t count);

    uint8_t serReadNoCheck(void);
    float serReadFloat();

    void flush(void);

    inline int available(void) {
      return (unsigned int)(RX_BUFFER_SIZE + rx_buffer.head - rx_buffer.tail) % RX_BUFFER_SIZE;
    }

    inline void store_char(unsigned char c);

    void serWrite(uint8_t c);

  private:
    void printNumber(unsigned long, uint8_t);
    void printFloat(double, uint8_t);

    ring_buffer rx_buffer;


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

    inline void serWrite(const char *str)
    {
      while (*str)
        serWrite(*str++);
    }


    inline void serWrite(const uint8_t *buffer, size_t size)
    {
      while (size--)
        serWrite(*buffer++);
    }

#if 0
    inline void print(const String &s)
    {
      for (int i = 0; i < (int)s.length(); i++) {
        serWrite(s[i]);
      }
    }
#endif

    inline void print(const char *str)
    {
      serWrite(str);
    }
    void print(char, int = BYTE);
    void print(unsigned char, int = BYTE);
    void print(int, int = DEC);
    void print(unsigned int, int = DEC);
    void print(long, int = DEC);
    void print(unsigned long, int = DEC);
    void print(double, int = 2);

    // void println(const String &s);
    void println(const char[]);
    void println(char, int = BYTE);
    void println(unsigned char, int = BYTE);
    void println(int, int = DEC);
    void println(unsigned int, int = DEC);
    void println(long, int = DEC);
    void println(unsigned long, int = DEC);
    void println(double, int = 2);
    void println(void);
};

extern MarlinSerial MSerial;

#define SERIAL_PROTOCOL(x) MSerial.print(x);
#define SERIAL_PROTOCOLPGM(x) serialprintPGM(PSTR(x));
#define SERIAL_PROTOCOLLNPGM(x) do{ serialprintPGM(PSTR(x)); MSerial.serWrite('\n');} while(0)
#define SERIAL_PROTOCOLLN(x) do { MSerial.print(x); MSerial.serWrite('\n');} while(0)

#define SERIAL_ECHO_START serialprintPGM(echomagic);
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERROR_START serialprintPGM(errormagic);
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

inline void MarlinSerial::store_char(unsigned char c) {
        int i = (unsigned int)(rx_buffer.head + 1) % RX_BUFFER_SIZE;

        // if we should be storing the received character into the location
        // just before the tail (meaning that the head would advance to the
        // current location of the tail), we're about to overflow the buffer
        // and so we don't write the character or advance the head.
        if (i != rx_buffer.tail) {
            rx_buffer.buffer[rx_buffer.head] = c;
            rx_buffer.head = i;
        }
        else {
            massert(0);
        }
    }


#if defined(DDSim)
    #include "MarlinSerialSim.h"
#endif


