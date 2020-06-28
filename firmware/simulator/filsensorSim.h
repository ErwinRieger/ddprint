
#include <Arduino.h>
#include <stdio.h>
#include <assert.h>

#include "stepperSim.h"

#include "filsensor.h"

#define FILDIR -1

class FilSensorSim {

        bool enabled;
        int  address;
        int16_t dataWord;

        /*
        float lastEPos;
        int16_t delta_y;

        bool spiWrite;
        int romDownLoad, spiRes;
        */

    public:
        FilSensorSim() {
            enabled = false;
            // lastEPos = 0.0;
            // spiWrite = false;
            // romDownLoad = 0;
            // spiRes = -1;
            address = 0;
            // delta_y = 0;
            dataWord = 0;
        }
        void enable(uint8_t v) {
            enabled = !v;

            if (enabled) {
                // spiWrite = false;
                // spiRes = -1;
                address = 0;
                dataWord = 0;
            }
        }

        bool isEnabled() { return enabled; }

        void computeDeltaY() {
assert(0);
#if 0
todo: update
            assert(delta_y == 0);

            float ediff = sse.pos - lastEPos; // [mm]
            delta_y = ediff * FS_STEPS_PER_MM;

            if (abs(delta_y) > 1) {

                if (delta_y > 0) {
                    // simulate slip
                    delta_y *= 0.9;
                }
            }
            lastEPos = sse.pos;
#endif
        }

        uint8_t spiSend(uint8_t b) {

            if (address == 0) {
                // read of first byte
                address = 1;

                // Bit 0x20 is "S1 End of offset compensation algorithm."
                // Bit  0x1 is "P1 Even parity for detecting bits 1-15 transmission error"
                uint8_t byte1 = 0; 
                uint8_t byte2 = 0x20; 
                dataWord = (byte1<<8) | byte2;

                if (__builtin_parity(dataWord))
                    dataWord |= 1;

                return byte1;
            }
            else if (address == 1) {
                // read of second byte
                return dataWord & 0xff;
            }
            
            assert(0);

#if 0
todo: update

        // write = b & 0x80;
        // address = b & 0x7f;
        if (spiWrite) {

            if (romDownLoad) {
                romDownLoad--;
                return;
            }

            switch (address & 0x7f) {
                case REG_Power_Up_Reset:
                case REG_Configuration_I:
                case REG_Configuration_II:
                case REG_Configuration_IV:
                case REG_LASER_CTRL0:
                case REG_Snap_Angle:
                    break;
                case REG_SROM_Enable:
                    romDownLoad = sizeof(sromData);
                    romDownLoad ++; // REG_SROM_Load_Burst byte
                    break;
                default:
                    printf("FilSensor: write to 0x%x not implemented!\n", address&0x7f);
                    assert(0);
            }
            spiWrite = false;
        }
        else if (b & 0x80) {
            // write
            address = b;
            spiWrite = true;
        }
        else {
            // read
            switch (b) {
                case REG_Product_ID:
                    spiRes = 0x33;
                    break;
                case REG_Revision_ID:
                    spiRes = 0x3;
                    break;
                case REG_Configuration_I:
                    spiRes = 0x09;
                    break;
                case REG_Inverse_Product_ID:
                    spiRes = ~0x33;
                    break;
                case REG_SROM_ID:
                    spiRes = SROMVER;
                    break;
                case REG_Motion:
                    computeDeltaY();
                    if (delta_y != 0)
                        spiRes = 0x80;
                    else
                        spiRes = 0;
                    break;
                case REG_Delta_Y_L:
                    spiRes = delta_y & 0xff;
                    break;
                case REG_Delta_Y_H:
                    spiRes = (delta_y >> 8) * FILDIR;
                    delta_y = 0;
                    break;
                case REG_Delta_X_L:
                case REG_Delta_X_H:
                case REG_LASER_CTRL0:
                case REG_Snap_Angle:
                case REG_Configuration_II:
                    spiRes = 0;
                    break;
                default:
                    printf("FilSensor: read at 0x%x not implemented!\n", b);
                    assert(0);
            }
        }
#endif
        }

#if 0
        uint8_t spiRec() {

            uint8_t r = spiRes;
            spiRes = -1;
            return r;
        }
#endif
};

extern FilSensorSim filSensorSim;




