
/************************************************************************************************
* Note by erwin.rieger@ibrieger.de:
* This file is part of ddprint - a direct drive 3d printer firmware.
* The Origin of this code is Ultimaker2Marlin (https://github.com/Ultimaker/Ultimaker2Marlin).
************************************************************************************************/

// xxx
#include <stdio.h>

#include <util/crc16.h>
#include "MarlinSerial.h"

uint8_t MarlinSerial::peekN(uint8_t index) {

    CRITICAL_SECTION_START; uint8_t res = rxBuffer.buffer[(rxBuffer.tail+index) & RX_BUFFER_MASK]; CRITICAL_SECTION_END; return res;
}

void MarlinSerial::peekChecksum(uint16_t *checksum, uint8_t count) {
#if 0
    if ((rxBuffer.head - rxBuffer.tail) >= count) {
        for (uint8_t i=0; i<count; i++)
            *checksum = _crc_xmodem_update(*checksum, rxBuffer.buffer[rxBuffer.tail+i]);
        return;
    }
#endif

    for (uint8_t i=0; i<count; i++)
        *checksum = _crc_xmodem_update(*checksum, peekN(i));
}

void MarlinSerial::cobsInit(uint16_t payloadLength) {

    cobsLen = payloadLength;
    atCobsBlock();
}

void MarlinSerial::atCobsBlock() {
    cobsCodeLen = 0;
}

//  bool cobsAvailable() { return (cobsLen > 0) || (cobsCodeLen == 1); }
uint8_t MarlinSerial::readNoCheckCobs(void)
{

    if (cobsCodeLen == 0) {
        cobsLen--;
        massert(_available()); cobsCodeLen = readNoCheckNoCobs();
    }

    if (cobsCodeLen == 0xFF) {
        cobsLen--;

        massert(_available()); uint8_t c = readNoCheckNoCobs();
        simassert(c);
        return c;
    }

    if (cobsCodeLen == 1) {
        atCobsBlock();
        return 0;
    }

    cobsLen--;
    cobsCodeLen--;

    massert(_available()); uint8_t c = readNoCheckNoCobs();
    simassert(c);
    return c;
}

uint8_t MarlinSerial::readNoCheckNoCobs(void)
{
//xxx
massert(_available());
CRITICAL_SECTION_START;
    unsigned char c = rxBuffer.buffer[rxBuffer.tail];
    rxBuffer.tail = (rxBuffer.tail + 1) & RX_BUFFER_MASK;
    CRITICAL_SECTION_END; return c;
}

float MarlinSerial::readFloatNoCheckCobs()
{
#if 0
    if ((rx_buffer.head - rx_buffer.tail) >= 4) {
        float f = *(float*)(rx_buffer.buffer + rx_buffer.tail);
        rx_buffer.tail += 4;
        return f;
    }
#endif

    massert(_available()); uint8_t  b1 = readNoCheckCobs();
    massert(_available()); uint32_t b2 = readNoCheckCobs();
    massert(_available()); uint32_t b3 = readNoCheckCobs();
    massert(_available()); uint32_t b4 = readNoCheckCobs();
    uint32_t i = (b4<<24) + (b3<<16) + (b2<<8) + b1;
    return *(float*)&i;;
}

uint16_t MarlinSerial::readUInt16NoCheckNoCobs()
{
#if 0
    if ((rxBuffer.head - rxBuffer.tail) >= 2) {
        uint16_t i = *(uint16_t*)(rxBuffer.buffer + rxBuffer.tail);
        rxBuffer.tail += 2;
        return i;
    }
#endif

    massert(_available()); uint8_t  b1 = readNoCheckNoCobs();
    massert(_available()); uint16_t b2 = readNoCheckNoCobs();
    return (b2<<8) + b1;
}

uint16_t MarlinSerial::readUInt16NoCheckCobs()
{
/*
    if ((rxBuffer.head - rxBuffer.tail) >= 2) {
        uint16_t i = *(uint16_t*)(rxBuffer.buffer + rxBuffer.tail);
        rxBuffer.tail += 2;
        return i;
    }
*/

    massert(_available()); uint8_t  b1 = readNoCheckCobs();
    massert(_available()); uint16_t b2 = readNoCheckCobs();
    return (b2<<8) + b1;
}

uint16_t MarlinSerial::serReadUInt16()
{
    simassert(0);
#if 0
    if ((rx_buffer.head - rx_buffer.tail) >= 2) {
        uint16_t i = *(int16_t*)(rx_buffer.buffer + rx_buffer.tail);
        rx_buffer.tail += 2;
        return i;
    }
#endif

    uint8_t  b1 = serReadNoCheck();
    uint16_t b2 = serReadNoCheck();
    return (b2<<8) + b1;
}

int32_t MarlinSerial::readInt32NoCheckCobs()
{
#if 0
    if ((rx_buffer.head - rx_buffer.tail) >= 4) {
        int32_t i = *(int32_t*)(rx_buffer.buffer + rx_buffer.tail);
        rx_buffer.tail += 4;
        return i;
    }
#endif

    massert(_available()); int8_t  b1 = readNoCheckCobs();
    massert(_available()); int32_t b2 = readNoCheckCobs();
    massert(_available()); int32_t b3 = readNoCheckCobs();
    massert(_available()); int32_t b4 = readNoCheckCobs();
    return (b4<<24) + (b3<<16) + (b2<<8) + b1;
}

uint32_t MarlinSerial::serReadUInt32()
{
    simassert(0);
#if 0
    if ((rx_buffer.head - rx_buffer.tail) >= 4) {
        uint32_t i = *(uint32_t*)(rx_buffer.buffer + rx_buffer.tail);
        rx_buffer.tail += 4;
        return i;
    }
#endif

    uint8_t  b1 = serReadNoCheck();
    uint32_t b2 = serReadNoCheck();
    uint32_t b3 = serReadNoCheck();
    uint32_t b4 = serReadNoCheck();
    return (b4<<24) + (b3<<16) + (b2<<8) + b1;
}

void MarlinSerial::flush(uint8_t n) {
    CRITICAL_SECTION_START;
    rxBuffer.tail = (rxBuffer.tail + n) & RX_BUFFER_MASK; CRITICAL_SECTION_END;
}

void MarlinSerial::flush()
{
    CRITICAL_SECTION_START;
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of rx_buffer_head but before writing
  // the value to rx_buffer_tail; the previous value of rx_buffer_head
  // may be written to rx_buffer_tail, making it appear as if the buffer
  // were full, not empty.
  rxBuffer.head = rxBuffer.tail; CRITICAL_SECTION_END;
}

// Preinstantiate Objects //////////////////////////////////////////////////////
MarlinSerial MSerial;

