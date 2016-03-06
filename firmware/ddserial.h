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

#include "Protothread.h"
// #include "pins.h"
// #include "move.h"
#include "mdebug.h"

// Size of tx buffer in bytes, must be a power of 2:
#define TxBufferLen  128
#define TxBufferMask  (TxBufferLen - 1)

class TxBuffer: public Protothread {

    private:
        uint8_t txBuffer[TxBufferLen];

        uint8_t head, tail;

    public:

        TxBuffer() {
            head = tail = 0;
        };

        FWINLINE uint8_t byteSize() {
            return (((int16_t)TxBufferLen + head) - tail) % TxBufferLen;
        }

        FWINLINE bool empty() {
            return head == tail;
        }

        FWINLINE bool full() {
            return byteSize() >= (TxBufferLen-10);
        }

        FWINLINE uint8_t pop() {
            uint8_t c = txBuffer[tail];
            tail = (tail + 1) % TxBufferLen;
            return c;
        }

        FWINLINE void pushChar(uint8_t c) {

#if defined(HEAVYDEBUG)
            if (full())
                return;
            if (byteSize() == (TxBufferLen-11)) {
                    txBuffer[head] = '\n';
                    head = (head + 1) % TxBufferLen;
                    return;
                    }

#endif

            txBuffer[head] = c;
            head = (head + 1) % TxBufferLen;
        }

        FWINLINE void pushStr(uint8_t *s) {
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

};

extern TxBuffer txBuffer;



