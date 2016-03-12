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

    public:

        TxBuffer() {
            head = tail = 0;
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

        FWINLINE uint8_t pop() {
            uint8_t c = txBuffer[tail];
            tail = (tail + 1) & TxBufferMask;
            return c;
        }

        FWINLINE void pushChar(uint8_t c) {

#if defined(HEAVYDEBUG)
            if (full()) {
                // xxx send error message
                return;
            }
#endif

            // Don't buffer character if it can be sent directly
            if (empty() && ((UCSR0A) & (1 << UDRE0))) {
                UDR0 = c;
                return;
            }

            txBuffer[head] = c;
            head = (head + 1) & TxBufferMask;
        }

        FWINLINE void pushStr(uint8_t *s) {
        }

        void sendUnbufferedPGM(const char *str) {

            char ch=pgm_read_byte(str);
            while(ch) {
                sendCharUnbuffered(ch);
                ch = pgm_read_byte(++str);
            }
        }

        void sendUnbuffered(const char *str) {

            while (*str)
                sendCharUnbuffered(*str++);
        }

        void sendCharUnbuffered(uint8_t c) {

            while (!((UCSR0A) & (1 << UDRE0)));
            UDR0 = c;
        }

        void sendACK() {
            pushChar(RESPUSBACK);
            return;

            if (empty()) {
                sendCharUnbuffered(RESPUSBACK);
                return;
            }

            pushChar(RESPUSBACK);
        }

        bool Run() {
            
            PT_BEGIN();

            while (! empty()) {

                PT_WAIT_UNTIL((UCSR0A) & (1 << UDRE0));

                uint8_t c = pop();
                UDR0 = c;
            }

            PT_RESTART();
            PT_END();
        }

        void pushCharChecksum(uint8_t c) {
            checksum = _crc_xmodem_update(checksum, c);
            pushChar(c);
        }

        void sendResponseStart(uint8_t respCode, uint8_t length) {

            checksum = 0;

            pushCharChecksum(respCode);
            pushCharChecksum(length);
        }

        void sendResponseEnd() {

            // printf("chesum: 0x%x\n", checksum);
            pushChar(checksum & 0xFF);
            pushChar(checksum >> 8);
        }

        void sendSimpleResponse(uint8_t respCode) {

            sendResponseStart(respCode, 1);
            sendResponseEnd();
        }

        void sendSimpleResponse(uint8_t respCode, uint8_t payload) {

            sendResponseStart(respCode, 1);
            pushCharChecksum(payload);
            sendResponseEnd();
        }

        void sendSimpleResponse(uint8_t respCode, uint8_t param1, uint8_t param2) {

            sendResponseStart(respCode, 2);
            pushCharChecksum(param1);
            pushCharChecksum(param2);
            sendResponseEnd();
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
        void sendResponseValue(char *s, uint8_t l) {

            pushCharChecksum(l);
            for (uint8_t i=0; i<l; i++)
                pushCharChecksum(s[i]);
        }

        // Send fixed size blob
        void sendResponseValue(uint8_t *b, uint8_t l) {

            for (uint8_t i=0; i<l; i++)
                pushCharChecksum(b[i]);
        }
};

extern TxBuffer txBuffer;



