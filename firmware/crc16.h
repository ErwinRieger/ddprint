
#include <stdint.h>

#pragma once

//
// Arduino has optimized version for avr:
//

#if defined(__avr__)
    #include <util/crc16.h>
#else

#if 0
static uint16_t _crc_xmodem_update (uint16_t crc, uint8_t data) {

    int i;
    crc = crc ^ ((uint16_t)data << 8);
    for (i=0; i<8; i++)
    {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc <<= 1;
    }
    return crc;
}
#endif

static uint16_t _crc_ccitt_update (uint16_t crc, uint8_t data) {

    data ^= crc & 0xFF;
    data ^= data << 4;

    return ((((uint16_t)data << 8) | (crc >> 8)) ^ (uint8_t)(data >> 4) 
            ^ ((uint16_t)data << 3));
}

#endif


