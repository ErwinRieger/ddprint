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
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>

#include "Protothread.h"
#include "mdebug.h"

// XXXX use 256 byts for simpler ringbuffer handling...

// Size of tx buffer in bytes, must be a power of 2:
#define TxBufferLen  128
#define TxBufferMask  (TxBufferLen - 1)

// Serial communication ACK
#define RESPUSBACK 0x6


class TxBuffer: public Protothread {

    private:
        uint8_t txBuffer[TxBufferLen];
        uint8_t head, tail;
        // Checksum for response messages
        uint16_t checksum;
        // Number of (cobs encoded) messages in buffer
        uint8_t nMessages;

        uint8_t charToSend;
        // Start pos of cobs block, -1 if no cobs block started yet
        int16_t cobsStart;
        uint8_t lenIndex;

        int16_t cf, csh, csl;

        FWINLINE void _pushCharNoChecksumNoCobs(uint8_t c) {

#if defined(HEAVYDEBUG)
            massert(! full());
#endif

            txBuffer[head] = c;
            head = (head + 1) & TxBufferMask;
        }

        FWINLINE void pushCharNoChecksumCobs(uint8_t c) {

            if (cobsStart == -1) {
                // printf("start cobs at %d\n", head);
                cobsStart = head;
                _pushCharNoChecksumNoCobs(0x1);
                txBuffer[lenIndex] += 1;
            }

            if (c == 0) {
                // printf("end 0 at %d\n", head);
                cobsStart = -1;
                return;
            }

            // printf("add '0x%x' at %d\n", c, head);
            txBuffer[cobsStart] += 1;
            _pushCharNoChecksumNoCobs(c);
            txBuffer[lenIndex] += 1;
        }

        FWINLINE uint8_t pop() {
            uint8_t c = txBuffer[tail];
            tail = (tail + 1) & TxBufferMask;
            return c;
        }

        FWINLINE uint8_t peek() {
            return txBuffer[tail];
        }


    public:
/// xxx same as rxbuffer
        TxBuffer() {
            head = tail = 0;
            nMessages = 0;
            cobsStart = -1;
        };

        FWINLINE uint8_t byteSize() {
            return ((TxBufferLen + head) - tail) & TxBufferMask;
        }

        FWINLINE bool empty() {
            return head == tail;
        }

        FWINLINE bool full() {
            return byteSize() >= (TxBufferLen-1);
        }

        FWINLINE void sendACK() {
            sendSimpleResponse(RESPUSBACK);
        }

        bool Run() {
            
            PT_BEGIN();

            while (nMessages) {

                PT_WAIT_UNTIL((UCSR0A) & (1 << UDRE0));
                charToSend = pop();
                UDR0 = charToSend;

                simassert(charToSend == 0);

                checksum = 0;

                charToSend = peek();
                while (charToSend) {

                    PT_WAIT_UNTIL((UCSR0A) & (1 << UDRE0));
                    UDR0 = charToSend;

                    tail = (tail + 1) & TxBufferMask;

                    checksum = _crc_xmodem_update(checksum, charToSend);

                    charToSend = peek();
                }

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
                else  { // csl == 0
                    cf = 0x3;
                    csl += 0x1;
                }

                PT_WAIT_UNTIL((UCSR0A) & (1 << UDRE0));
                UDR0 = cf;

                PT_WAIT_UNTIL((UCSR0A) & (1 << UDRE0));
                UDR0 = csl;

                PT_WAIT_UNTIL((UCSR0A) & (1 << UDRE0));
                UDR0 = csh;

                nMessages--;
                // Init cobs encoder
                cobsStart = -1;
            }

            PT_RESTART();
            PT_END();
        }

        void sendResponseStart(uint8_t respCode) {

            // Init cobs encoder
            cobsStart = -1;

            _pushCharNoChecksumNoCobs(0x0); // SOH
            _pushCharNoChecksumNoCobs(respCode);
            lenIndex = head;
            _pushCharNoChecksumNoCobs(1);
        }

        void sendResponseEnd() {

            if (cobsStart != -1) {
                // Cobs block does not end with 0x0, mark block as a
                // 'maximum length code block'.
                txBuffer[cobsStart] = 0xff;
            }

            txBuffer[head] = 0x0; // Terminate loop in Run() if last response in buffer
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

        void sendResponseUint8(uint8_t v) {
            pushCharNoChecksumCobs(v);
        }

        void sendResponseValue(uint16_t v) {

            sendResponseValue((uint8_t*)&v, 2);
        }

        void sendResponseValue(uint32_t v) {

            sendResponseValue((uint8_t*)&v, 4);
        }

        void sendResponseValue(int32_t v) {

            sendResponseValue((uint8_t*)&v, 4);
        }

        void sendResponseValue(float v) {

            sendResponseValue((uint8_t*)&v, 4);
        }

        // Send string, one byte length, then the string
        void sendResponseString(char *s, uint8_t l) {

            sendResponseUint8(l);
            for (uint8_t i=0; i<l; i++)
                sendResponseUint8(s[i]);
        }

        // Send fixed size blob
        void sendResponseValue(uint8_t *b, uint8_t l) {

            for (uint8_t i=0; i<l; i++)
                sendResponseUint8(b[i]);
        }

        void flush() {
            nMessages = 0;
            head = tail;
        }
};

extern TxBuffer txBuffer;



