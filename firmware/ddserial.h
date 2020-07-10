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
    #include <util/crc16.h>
#endif

#include "Protothread.h"
#include "mdebug.h"

// Size of tx buffer in bytes
// Note: Using a buffer size of 256 bytes has two big advantages:
//  * The head and tail pointers simply wrap around, no bitmasks or
//     modulo operations are needed.
//  * The head and tail operations are atomic, so no critical sections
//     are needed.
#define TxBufferLen  256

// Serial communication ACK
#define RESPUSBACK 0x6


class TxBuffer: public Protothread {

    private:
        uint8_t txBuffer[TxBufferLen];
        uint8_t head, tail;

        // Checksum for response messages
        uint16_t checksum;
        // Number of (cobs encoded) messages to send in buffer
        uint8_t nMessages;

        uint8_t charToSend;
        // Start pos of cobs block, -1 if no cobs block started yet
        int16_t cobsStart;
        uint8_t lenIndex;

        int16_t cf, csh, csl;

        FWINLINE bool pushByte(uint8_t c) {

// #if defined(HEAVYDEBUG)
            // massert(! full());
// #endif

            if (! full()) {
                txBuffer[head++] = c;
                return true;
            }
            return false;
        }

        FWINLINE void pushCharCobs(uint8_t c) {

            if (cobsStart == -1) {
                // printf("start cobs at %d\n", head);
                cobsStart = head;
                pushByte(0x1);
                txBuffer[lenIndex] += 1;
            }

            if (c == 0) {
                // printf("end 0 at %d\n", head);
                cobsStart = -1;
                return;
            }

            // printf("add '0x%x' at %d\n", c, head);
            txBuffer[cobsStart] += 1;
            pushByte(c);
            txBuffer[lenIndex] += 1;
        }

        FWINLINE uint8_t pop() {
            return txBuffer[tail++];
        }

        FWINLINE uint8_t peek() {
            return txBuffer[tail];
        }


    public:
        TxBuffer() {
            head = tail = 0;
            nMessages = 0;
            cobsStart = -1;
        };

        FWINLINE uint8_t byteSize() {
            return (uint16_t)(TxBufferLen + head) - tail;
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
//armtodo
#if 0
            while (nMessages) {

                PT_WAIT_UNTIL((UCSR0A) & (1 << UDRE0));
                charToSend = pop();
                UDR0 = charToSend;

                simassert(charToSend == 0);

                // printf("payload: ");
                checksum = 0xffff;

                charToSend = peek();
                while (charToSend) {

                    PT_WAIT_UNTIL((UCSR0A) & (1 << UDRE0));
                    UDR0 = charToSend;

                    // printf("%02x", charToSend);

                    tail++;

                    checksum = _crc_ccitt_update(checksum, charToSend);

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
#endif

            PT_RESTART();
            PT_END();
        }

        void sendResponseStart(uint8_t respCode) {

            // Init cobs encoder
            cobsStart = -1;

            if (! pushByte(0x0)) return; // SOH
            if (! pushByte(respCode)) return;
            lenIndex = head;
            pushByte(1);
        }

        void sendResponseEnd() {

            if (full()) {
                // Rollback
                head = lenIndex - 2;
                return;
            }

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

        void sendResponseValue(uint16_t v) {

            sendResponseValue((uint8_t*)&v, 2);
        }

        void sendResponseInt16(int16_t v) {

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
        void sendResponseString(const char *s, uint8_t l) {

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



