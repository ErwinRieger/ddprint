
#include <Arduino.h>
#include <stdio.h>
#include <assert.h>

#include "stepperSim.h"

#include "filsensor.h"
#if defined(ADNSFS)
    #include "adns9800fwa6.h"
#endif

// #define FILSTEP (25.4 / 1000.0)
#define FILDIR -1

class FilSensorSim {

        bool enabled;
        int  address;

        float lastEPos;
        int16_t delta_y;

        bool spiWrite;
        int romDownLoad, spiRes;

    public:
        FilSensorSim() {
            enabled = false;
            lastEPos = 0.0;
            spiWrite = false;
            romDownLoad = 0;
            spiRes = -1;
            address = -1;
            delta_y = 0;
        }
        void enable(uint8_t v) {
            enabled = !v;
            spiWrite = false;
            spiRes = -1;
            address = -1;
        }

        bool isEnabled() { return enabled; }

        void computeDeltaY() {
assert(0);
#if 0
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

        void spiSend(uint8_t b) {
#if 0

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

        uint8_t spiRec() {

            uint8_t r = spiRes;
            spiRes = -1;
            return r;
        }

};

extern FilSensorSim filSensorSim;

