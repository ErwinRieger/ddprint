
#include <stdint.h>

#pragma once

//
// Arduino has optimized version for avr:
//

#if defined(AVR)
    #include <util/crc16.h>
#else

    static uint16_t _crc_ccitt_update (uint16_t crc, uint8_t data) {

        data ^= crc & 0xFF;
        data ^= data << 4;

        return ((((uint16_t)data << 8) | (crc >> 8)) ^ (uint8_t)(data >> 4) 
                ^ ((uint16_t)data << 3));
    }

#endif


