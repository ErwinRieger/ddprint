
/************************************************************************************************
* Note by erwin.rieger@ibrieger.de:
* This file is part of ddprint - a direct drive 3d printer firmware.
* The Origin of this code is Ultimaker2Marlin (https://github.com/Ultimaker/Ultimaker2Marlin).
************************************************************************************************/

/*
  HardwareSerial.cpp - Hardware serial library for Wiring
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

  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
*/

#include <avr/interrupt.h>

#include "ddprint.h"
#include "MarlinSerial.h"

//#elif defined(SIG_USART_RECV)
#if defined(M_USARTx_RX_vect)
  // fixed by Mark Sproul this is on the 644/644p
  //SIGNAL(SIG_USART_RECV)
  SIGNAL(M_USARTx_RX_vect)
  {

    // Check error status bits for frame-/overrun- and parity error
    MSerial.rxerror |= M_UCSRxA & 0x1C;
    unsigned char c  =  M_UDRx;
    MSerial.store_char(c);
  }
#endif

// Constructors ////////////////////////////////////////////////////////////////

MarlinSerial::MarlinSerial()
{
    rx_buffer.head = rx_buffer.tail = rxerror = 0;
}

// Public Methods //////////////////////////////////////////////////////////////

void MarlinSerial::begin(long baud)
{
  uint16_t baud_setting;
  bool useU2X = true;

#if F_CPU == 16000000UL && SERIAL_PORT == 0
  // hardcoded exception for compatibility with the bootloader shipped
  // with the Duemilanove and previous boards and the firmware on the 8U2
  // on the Uno and Mega 2560.
  if (baud == 57600) {
    useU2X = false;
  }
#endif

  if (useU2X) {
    M_UCSRxA = 1 << M_U2Xx;
    baud_setting = (F_CPU / 4 / baud - 1) / 2;
  } else {
    M_UCSRxA = 0;
    baud_setting = (F_CPU / 8 / baud - 1) / 2;
  }

  // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
  M_UBRRxH = baud_setting >> 8;
  M_UBRRxL = baud_setting;

  sbi(M_UCSRxB, M_RXENx);
  sbi(M_UCSRxB, M_TXENx);
  sbi(M_UCSRxB, M_RXCIEx);
}

extern uint16_t waitCount;

#if 0
void MarlinSerial::serWrite(uint8_t c) {

    while (!((M_UCSRxA) & (1 << M_UDREx))) waitCount++;
    M_UDRx = c;
}

#endif

#include "ddserial.h"

void MarlinSerial::serWrite(uint8_t c) {

    txBuffer.pushChar(c);
}






