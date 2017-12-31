
/************************************************************************************************
* Note by erwin.rieger@ibrieger.de:
* This file is part of ddprint - a direct drive 3d printer firmware.
* The Origin of this code is Ultimaker2Marlin (https://github.com/Ultimaker/Ultimaker2Marlin).
************************************************************************************************/

#include <util/crc16.h>
#include "MarlinSerial.h"

uint8_t MarlinSerial::peekN(uint8_t index) {

    return buffer[tail+index];
}

void MarlinSerial::peekChecksum(uint16_t *checksum, uint8_t count) {

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

uint8_t MarlinSerial::readNoCheckCobs(void)
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

uint8_t MarlinSerial::readNoCheckNoCobs(void)
{

    return buffer[tail++];
}

float MarlinSerial::readFloatNoCheckCobs()
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
uint16_t MarlinSerial::readUInt16NoCheckNoCobs()
{

    uint16_t i = *(uint16_t*)(buffer + tail);
    tail += 2;
    return i;
}
#endif

uint16_t MarlinSerial::readUInt16NoCheckCobs()
{

    uint8_t  b1 = readNoCheckCobs();
    uint16_t b2 = readNoCheckCobs();
    return (b2<<8) + b1;
}

int32_t MarlinSerial::readInt32NoCheckCobs()
{

    int8_t  b1 = readNoCheckCobs();
    int32_t b2 = readNoCheckCobs();
    int32_t b3 = readNoCheckCobs();
    int32_t b4 = readNoCheckCobs();
    return (b4<<24) + (b3<<16) + (b2<<8) + b1;
}

// Preinstantiate Objects //////////////////////////////////////////////////////
MarlinSerial MSerial;

