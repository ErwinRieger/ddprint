
/************************************************************************************************
* Note by erwin.rieger@ibrieger.de:
* This file is part of ddprint - a direct drive 3d printer firmware.
* The Origin of this code is Ultimaker2Marlin (https://github.com/Ultimaker/Ultimaker2Marlin).
************************************************************************************************/

#include <util/crc16.h>
#include "MarlinSerial.h"

uint8_t MarlinSerial::peekN(uint8_t index) {

    return rx_buffer.buffer[(rx_buffer.tail+index) & RX_BUFFER_MASK];
}

void MarlinSerial::peekChecksum(uint16_t *checksum, uint8_t count) {

    if ((rx_buffer.head - rx_buffer.tail) >= count) {
        for (uint8_t i=0; i<count; i++)
            *checksum = _crc_xmodem_update(*checksum, rx_buffer.buffer[rx_buffer.tail+i]);
        return;
    }

    for (uint8_t i=0; i<count; i++)
        *checksum = _crc_xmodem_update(*checksum, peekN(i));
}

uint8_t MarlinSerial::serReadNoCheck(void)
{
    unsigned char c = rx_buffer.buffer[rx_buffer.tail];
    rx_buffer.tail = (rx_buffer.tail + 1) & RX_BUFFER_MASK;
    return c;
}

float MarlinSerial::serReadFloat()
{
    if ((rx_buffer.head - rx_buffer.tail) >= 4) {
        float f = *(float*)(rx_buffer.buffer + rx_buffer.tail);
        rx_buffer.tail += 4;
        return f;
    }

    uint8_t  b1 = serReadNoCheck();
    uint32_t b2 = serReadNoCheck();
    uint32_t b3 = serReadNoCheck();
    uint32_t b4 = serReadNoCheck();
    uint32_t i = (b4<<24) + (b3<<16) + (b2<<8) + b1;
    return *(float*)&i;;
}

uint16_t MarlinSerial::serReadUInt16()
{
    if ((rx_buffer.head - rx_buffer.tail) >= 4) {
        int32_t i = *(int32_t*)(rx_buffer.buffer + rx_buffer.tail);
        rx_buffer.tail += 4;
        return i;
    }

    uint8_t  b1 = serReadNoCheck();
    uint16_t b2 = serReadNoCheck();
    return (b2<<8) + b1;
}

int32_t MarlinSerial::serReadInt32()
{
    if ((rx_buffer.head - rx_buffer.tail) >= 4) {
        int32_t i = *(int32_t*)(rx_buffer.buffer + rx_buffer.tail);
        rx_buffer.tail += 4;
        return i;
    }

    int8_t  b1 = serReadNoCheck();
    int32_t b2 = serReadNoCheck();
    int32_t b3 = serReadNoCheck();
    int32_t b4 = serReadNoCheck();
    return (b4<<24) + (b3<<16) + (b2<<8) + b1;
}

uint32_t MarlinSerial::serReadUInt32()
{
    if ((rx_buffer.head - rx_buffer.tail) >= 4) {
        uint32_t i = *(uint32_t*)(rx_buffer.buffer + rx_buffer.tail);
        rx_buffer.tail += 4;
        return i;
    }

    uint8_t  b1 = serReadNoCheck();
    uint32_t b2 = serReadNoCheck();
    uint32_t b3 = serReadNoCheck();
    uint32_t b4 = serReadNoCheck();
    return (b4<<24) + (b3<<16) + (b2<<8) + b1;
}

void MarlinSerial::flush()
{
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of rx_buffer_head but before writing
  // the value to rx_buffer_tail; the previous value of rx_buffer_head
  // may be written to rx_buffer_tail, making it appear as if the buffer
  // were full, not empty.
  rx_buffer.head = rx_buffer.tail;
}

// Preinstantiate Objects //////////////////////////////////////////////////////
MarlinSerial MSerial;

