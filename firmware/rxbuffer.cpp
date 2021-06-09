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

#include "rxbuffer.h"
#include "crc16.h"
#include "hal.h"


void RxBuffer::peekChecksum(uint16_t *checksum, uint8_t count) {

    for (uint8_t i=0; i<count; i++)
        *checksum = _crc_ccitt_update(*checksum, peekN(i));
}

void RxBuffer::cobsInit(uint16_t payloadLength) {

    cobsLen = payloadLength;
    atCobsBlock();
}

void RxBuffer::atCobsBlock() {
    cobsCodeLen = 0;
}

uint8_t RxBuffer::readNoCheckCobs(void)
{

    if (cobsCodeLen == 0) {
        cobsLen--;
        cobsCodeLen = pop();
    }

    if (cobsCodeLen == 0xFF) {
        cobsLen--;

        uint8_t c = pop();
        simassert(c);
        return c;
    }

    if (cobsCodeLen == 1) {
        atCobsBlock();
        return 0;
    }

    cobsLen--;
    cobsCodeLen--;

    uint8_t c = pop();
    simassert(c);
    return c;
}

float RxBuffer::readFloatNoCheckCobs()
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

int16_t RxBuffer::readInt16NoCheckCobs()
{

    uint8_t  b1 = readNoCheckCobs();
    int16_t  b2 = readNoCheckCobs();
    return (b2<<8) + b1;
}

uint16_t RxBuffer::readUInt16NoCheckCobs()
{

    uint8_t  b1 = readNoCheckCobs();
    uint16_t b2 = readNoCheckCobs();
    return (b2<<8) + b1;
}

int32_t RxBuffer::readInt32NoCheckCobs()
{

    uint8_t  b1 = readNoCheckCobs();
    uint32_t b2 = readNoCheckCobs();
    uint32_t b3 = readNoCheckCobs();
    int32_t  b4 = readNoCheckCobs();
    return (b4<<24) + (b3<<16) + (b2<<8) + b1;
}

uint32_t RxBuffer::readUInt32NoCheckCobs()
{

    uint8_t   b1 = readNoCheckCobs();
    uint32_t  b2 = readNoCheckCobs();
    uint32_t  b3 = readNoCheckCobs();
    uint32_t  b4 = readNoCheckCobs();
    return (b4<<24) + (b3<<16) + (b2<<8) + b1;
}

void RxBuffer::readScaledUInt32NoCheckCobs(ScaledUInt32 &scaledInt) {

    scaledInt.value = readUInt32NoCheckCobs();
    scaledInt.shift = readNoCheckCobs();
}

#if defined(AVR)

void RxBuffer::begin(uint32_t baud)
{

  M_UCSRxA = 0;

  setBaudrate(baud);

  sbi(M_UCSRxB, M_RXENx);
  sbi(M_UCSRxB, M_TXENx);
  sbi(M_UCSRxB, M_RXCIEx);
}

// Parameter baud is just the lowbyte of the 
// UBRR register.
void RxBuffer::setBaudrate(uint32_t baudRate) {

    M_UBRRxH = 0;
    M_UBRRxL = baudRate;
}

// Serial interrupt routine
SIGNAL(M_USARTx_RX_vect)
{
    // // Check error status bits for frame-/overrun- and parity error
    // RxBuffer.rxerror |= M_UCSRxA & 0x1C;

    //
    // Don't check for full() buffer here:
    //
    // Buffer overrun only if mainloop is to slow().
    //
    // RxCRC and RxTimeOut errors if baudrate is to high.
    //
    unsigned char c  =  M_UDRx;
    rxBuffer.pushVal(c);
}

#endif

#if defined(__arm__)
// Note: Fixed usage of USART1
void RxBuffer::begin(uint32_t baud)
{

    gpio_set_af_mode(BOARD_USART1_TX_PIN, GPIO_AFMODE_USART1_3);
    gpio_set_af_mode(BOARD_USART1_RX_PIN, GPIO_AFMODE_USART1_3);

    gpio_set_mode(BOARD_USART1_TX_PIN, (gpio_pin_mode)(GPIO_AF_OUTPUT_PP_PU | 0x700));
    gpio_set_mode(BOARD_USART1_RX_PIN, (gpio_pin_mode)(GPIO_AF_INPUT_PU | 0x700));

    rcc_clk_enable(USART1->clk_id);

    nvic_irq_enable(USART1->irq_num);

    setBaudrate(baud);

    usart_enable(USART1);
}

void RxBuffer::end() {

    nvic_irq_disable(USART1->irq_num);
}

void RxBuffer::setBaudrate(uint32_t baudRate) {

    usart_set_baud_rate(USART1, baudRate);
}

//
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

	int sr = USART1->regs->SR;
	if(sr & USART_SR_RXNE) {

        unsigned char c = USART1->regs->DR;
        rxBuffer.pushVal(c);
    }
    else if(sr & USART_SR_ORE) {
        USART1->regs->DR;
    }
  }
}

#endif

RxBuffer rxBuffer;







