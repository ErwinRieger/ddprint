/*
# This file is part of ddprint - a 3D printer firmware.
# 
# Copyright 2015 erwin.rieger@ibrieger.de
# 
# ddprint is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# ddprint is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with ddprint.  If not, see <http://www.gnu.org/licenses/>.
*/

#if defined(AVR)
    #include <avr/interrupt.h>
#endif

#include "serialport.h"
#include "crc16.h"
#include "hal.h"


#if !defined(DDSim)
SerialPort::SerialPort()
{
    init();
}
#endif

void SerialPort::init()
{
    head = tail; //  = rxerror = 0;
}

uint8_t SerialPort::peekN(uint8_t index) {

    return buffer[tail+index];
}

void SerialPort::peekChecksum(uint16_t *checksum, uint8_t count) {

    for (uint8_t i=0; i<count; i++)
        *checksum = _crc_ccitt_update(*checksum, peekN(i));
}

void SerialPort::cobsInit(uint16_t payloadLength) {

    cobsLen = payloadLength;
    atCobsBlock();
}

void SerialPort::atCobsBlock() {
    cobsCodeLen = 0;
}

uint8_t SerialPort::readNoCheckCobs(void)
{

    if (cobsCodeLen == 0) {
        cobsLen--;
        cobsCodeLen = readNoCheckNoCobs();
    }

    if (cobsCodeLen == 0xFF) {
        cobsLen--;

        uint8_t c = readNoCheckNoCobs();
        simassert(c);
        return c;
    }

    if (cobsCodeLen == 1) {
        atCobsBlock();
        return 0;
    }

    cobsLen--;
    cobsCodeLen--;

    uint8_t c = readNoCheckNoCobs();
    simassert(c);
    return c;
}

uint8_t SerialPort::readNoCheckNoCobs(void)
{

    return buffer[tail++];
}

float SerialPort::readFloatNoCheckCobs()
{

    union {
        uint8_t buf[4];
        float f;
    } floatBuf;

    floatBuf.buf[0] = readNoCheckCobs();
    floatBuf.buf[1] = readNoCheckCobs();
    floatBuf.buf[2] = readNoCheckCobs();
    floatBuf.buf[3] = readNoCheckCobs();

    return floatBuf.f;
}

#if 0
uint16_t SerialPort::readUInt16NoCheckNoCobs()
{

    uint16_t i = *(uint16_t*)(buffer + tail);
    tail += 2;
    return i;
}
#endif

int16_t SerialPort::readInt16NoCheckCobs()
{

    uint8_t  b1 = readNoCheckCobs();
    int16_t  b2 = readNoCheckCobs();
    return (b2<<8) + b1;
}

uint16_t SerialPort::readUInt16NoCheckCobs()
{

    uint8_t  b1 = readNoCheckCobs();
    uint16_t b2 = readNoCheckCobs();
    return (b2<<8) + b1;
}

int32_t SerialPort::readInt32NoCheckCobs()
{

    uint8_t  b1 = readNoCheckCobs();
    uint32_t b2 = readNoCheckCobs();
    uint32_t b3 = readNoCheckCobs();
    int32_t  b4 = readNoCheckCobs();
    return (b4<<24) + (b3<<16) + (b2<<8) + b1;
}

#if defined(AVR)

void SerialPort::begin(long baud)
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

// #if defined(M_USARTx_RX_vect)
// Serial interrupt routine
SIGNAL(M_USARTx_RX_vect)
{

    // // Check error status bits for frame-/overrun- and parity error
    // serialPort.rxerror |= M_UCSRxA & 0x1C;
    unsigned char c  =  M_UDRx;
    serialPort.store_char(c);
}
// #endif

#endif

#if defined(__arm__)
// Note: Fixed usage of USART1
void SerialPort::begin(long baud)
{

    gpio_set_af_mode(BOARD_USART1_TX_PIN, GPIO_AFMODE_USART1_3);
    gpio_set_af_mode(BOARD_USART1_RX_PIN, GPIO_AFMODE_USART1_3);

    gpio_set_mode(BOARD_USART1_TX_PIN, (gpio_pin_mode)(GPIO_AF_OUTPUT_PP_PU | 0x700));
    gpio_set_mode(BOARD_USART1_RX_PIN, (gpio_pin_mode)(GPIO_AF_INPUT_PU | 0x700));

    rcc_clk_enable(USART1->clk_id);

    nvic_irq_enable(USART1->irq_num);

    usart_set_baud_rate(USART1, baud);

    usart_enable(USART1);
}

// Serial interrupt routine
//
// Note, to overwrite stm32duino's irq handler i had to define __irq_usart1() as weak
// in stm32duino/hardware/STM32F4/2019.2.28/cores/maple/libmaple/usart.c:
// __attribute__((weak)) void __irq_usart1(void)
//
// See https://stm32duinoforum.com/forum/viewtopic_f_53_t_2759.html, also.
//
extern "C" {
  void __irq_usart1(void) {

	volatile int sr = USART1->regs->SR;
	if(sr & USART_SR_RXNE) {
		// rb_safe_insert(&dev->rbRX, (uint8)dev->regs->DR);
        unsigned char c = USART1->regs->DR;
        serialPort.store_char(c);
    }
  }
}

#endif

SerialPort serialPort;







