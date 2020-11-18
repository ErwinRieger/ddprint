/*
* This file is part of ddprint - a direct drive 3D printer firmware.
* 
* Copyright 2016 erwin.rieger@ibrieger.de
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

#if defined(AVR)
    #include <avr/io.h>
    #include <avr/pgmspace.h>
#endif

#include "hal.h"
#include "Protothread.h"
#include "mdebug.h"
#include "crc16.h"
#include "ringbuffer.h"

// Size of tx buffer in bytes.
#define TxBufferLen  256

typedef CircularBuffer<uint8_t, uint16_t, TxBufferLen> TxBufferBase;

// Serial communication ACK
#define RESPUSBACK 0x6

//
// Stm32 port:
// Note: rx/tx buffer memory wasted in struct usart_dev.
//
class TxBuffer: public Protothread, public TxBufferBase {

    private:

        // Checksum for response messages
        uint16_t checksum;
        // Number of (cobs encoded) messages to send in buffer
        uint8_t nMessages;

        uint8_t charToSend;
        // Start pos of cobs block, -1 if no cobs block started yet
        uint16_t cobsStart;
        uint16_t lenIndex;

        int16_t cf, csh, csl;

public:
        FWINLINE bool pushByte(uint8_t c) {

// #if defined(HEAVYDEBUG)
            // massert(! full());
// #endif

            if (! full()) {
                push(c);
                return true;
            }
            return false;
        }

        FWINLINE void pushCharCobs(uint8_t c) {

            if (cobsStart == 0xffff) {
                // printf("start cobs at %d\n", head);
                cobsStart = _ringbuffer_head;
                pushByte(0x1);
                // _ringbuffer_array[mask(lenIndex)] += 1;
                inc(lenIndex);
            }

            if (c == 0) {
                // printf("end 0 at %d\n", head);
                cobsStart = 0xffff;
                return;
            }

            // printf("add '0x%x' at %d\n", c, head);
            // _ringbuffer_array[mask(cobsStart)] += 1;
            inc(cobsStart);
            pushByte(c);
            // _ringbuffer_array[mask(lenIndex)] += 1;
            inc(lenIndex);
        }

    public:
        TxBuffer() {
            ringBufferInit();
            nMessages = 0;
            cobsStart = 0xffff;
        };

        FWINLINE void sendACK() {
            sendSimpleResponse(RESPUSBACK);
        }

        bool Run() {
            
            PT_BEGIN();

            while (nMessages) {

                // PT_WAIT_UNTIL((UCSR0A) & (1 << UDRE0));
                PT_WAIT_UNTIL( SERIAL_TX_DR_EMPTY() );
                charToSend = pop();
                // UDR0 = charToSend;
                SERIAL_TX_DR_PUTC( charToSend );

                simassert(charToSend == 0);

                // printf("payload: ");
                checksum = 0xffff;

                charToSend = peek();
                while (charToSend) {

                    // PT_WAIT_UNTIL((UCSR0A) & (1 << UDRE0));
                    PT_WAIT_UNTIL( SERIAL_TX_DR_EMPTY() );
                    // UDR0 = charToSend;
                    SERIAL_TX_DR_PUTC( charToSend );

                    // printf("%02x", charToSend);

                    _ringbuffer_tail++;

                    checksum = _crc_ccitt_update(checksum, charToSend);

                    if (empty())
                        charToSend = 0;
                    else
                        charToSend = peek();
                }

                // printf(" checksum fw: 0x%x\n", checksum);

                csh = checksum >> 8;
                csl = checksum & 0xFF;
                cf = 0x1;

                if (checksum == 0) {
                    cf = 0x4;
                    csh += 0x1;
                    csl += 0x1;
                }
                else if (csh == 0) {
                    cf = 0x2;
                    csh += 0x1;
                }
                else if  (csl == 0) {
                    cf = 0x3;
                    csl += 0x1;
                }

                // printf(" checksum flags: 0x%x\n", cf);

                // PT_WAIT_UNTIL((UCSR0A) & (1 << UDRE0));
                PT_WAIT_UNTIL( SERIAL_TX_DR_EMPTY() );
                // UDR0 = cf;
                SERIAL_TX_DR_PUTC( cf );

                // PT_WAIT_UNTIL((UCSR0A) & (1 << UDRE0));
                PT_WAIT_UNTIL( SERIAL_TX_DR_EMPTY() );
                // UDR0 = csl;
                SERIAL_TX_DR_PUTC( csl );

                // PT_WAIT_UNTIL((UCSR0A) & (1 << UDRE0));
                PT_WAIT_UNTIL( SERIAL_TX_DR_EMPTY() );
                // UDR0 = csh;
                SERIAL_TX_DR_PUTC( csh );

                PT_WAIT_UNTIL( SERIAL_TX_COMPLETE() );

                nMessages--;
                // Init cobs encoder
                cobsStart = 0xffff;
            }

            PT_RESTART();
            PT_END();
        }

        void sendResponseStart(uint8_t respCode) {

            // Init cobs encoder
            cobsStart = 0xffff;

            if (! pushByte(0x0)) return; // SOH
            if (! pushByte(respCode)) return;
            lenIndex = _ringbuffer_head;
            pushByte(1);
        }

        void sendResponseEnd() {

            if (full()) {
                // Rollback
                _ringbuffer_head = lenIndex - 2;
                return;
            }

            if (cobsStart != 0xffff) {
                // Cobs block does not end with 0x0, mark block as a
                // 'maximum length code block'.
                // _ringbuffer_array[mask(cobsStart)] = 0xff;
                setVar(cobsStart, 0xff);
            }

            // _ringbuffer_array[mask(_ringbuffer_head)] = 0x0; // Terminate loop in Run() if last response in buffer
            setVar(_ringbuffer_head, 0x0);
            nMessages++;
        }

        void sendSimpleResponse(uint8_t respCode) {

            sendResponseStart(respCode);
            sendResponseEnd();
        }

        void sendSimpleResponse(uint8_t respCode, uint8_t payload) {

            sendResponseStart(respCode);
            sendResponseUint8(payload);
            sendResponseEnd();
        }

        void sendSimpleResponse(uint8_t respCode, uint8_t param1, uint8_t param2) {

            sendResponseStart(respCode);
            sendResponseUint8(param1);
            sendResponseUint8(param2);
            sendResponseEnd();
        }

        void sendSimpleResponse(uint8_t respCode, uint8_t param1, uint8_t param2, uint8_t param3) {

            sendResponseStart(respCode);
            sendResponseUint8(param1);
            sendResponseUint8(param2);
            sendResponseUint8(param3);
            sendResponseEnd();
        }

        void sendResponseUint8(uint8_t v) {
            pushCharCobs(v);
        }

        void sendResponseInt8(int8_t v) {
            pushCharCobs(v);
        }

        void sendResponseUInt16(uint16_t v) {

            sendResponseBlob((uint8_t*)&v, 2);
        }

        void sendResponseInt16(int16_t v) {

            sendResponseBlob((uint8_t*)&v, 2);
        }

        void sendResponseUInt32(uint32_t v) {

            sendResponseBlob((uint8_t*)&v, 4);
        }

        void sendResponseInt32(int32_t v) {

            sendResponseBlob((uint8_t*)&v, 4);
        }

        void sendResponseFloat(float v) {

            sendResponseBlob((uint8_t*)&v, 4);
        }

        // Send string, one byte length, then the string
        void sendResponseString(const char *s, uint8_t l) {

            sendResponseUint8(l);
            for (uint8_t i=0; i<l; i++)
                sendResponseUint8(s[i]);
        }

        // Send fixed size blob
        void sendResponseBlob(uint8_t *b, uint8_t l) {

            for (uint8_t i=0; i<l; i++)
                sendResponseUint8(b[i]);
        }

        void flush() {
            nMessages = 0;
            ringBufferInit();
        }
};

extern TxBuffer txBuffer;



