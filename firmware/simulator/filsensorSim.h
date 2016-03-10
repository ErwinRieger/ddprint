
#include <Arduino.h>
#include <stdio.h>
#include <assert.h>

#include "stepperSim.h"

#define FILSTEP (25.4 / 1000.0)
#define FILDIR -1

class FilSensorSim {

        bool enabled;
        uint8_t clockState;
        uint8_t dataState;
        uint8_t abitpos;
        uint8_t dbitpos;
        uint8_t address;
        int8_t data;

        float lastEPos;
        float filPos;
        int8_t delta_y;
        // unsigned long long lastTime;

    public:
        FilSensorSim() {
            enabled = false;
            clockState = HIGH;
            dataState = HIGH;
            abitpos = dbitpos = 8;
            lastEPos = filPos = 0.0;
            // lastTime = getTimestamp();
        }
        void enable(uint8_t v) {
            // if (!enabled && !v) {
                // abitpos = dbitpos = 8;
                // address = data = 0;
            // }
            enabled = !v;
        }

        bool isEnabled() { return enabled; }

        void clock(uint8_t v) {
            assert(enabled);

            if ((abitpos == 0) && (!(address & 0x80)) && (clockState == HIGH) && (v == LOW)) {
                // Read mode
                dbitpos--;
                return;
            }
            if ((clockState == LOW) && (v == HIGH)) {
                if (abitpos) {
                    if (abitpos == 8)
                        address = 0;
                    address |= dataState << (abitpos-1);
                    // printf("read addr: %d, %d, 0x%x\n", abitpos, dataState, address);
                    abitpos--;
                    if (!abitpos) {
                        if (!(address & 0x80)) { // read mode
                            // printf("read addr: 0x%x\n", address);
                            // set data for following read
                            switch (address) {
                                case 0x0: // product
                                    data = 0x55;
                                    break;
                                case 0x1: // rev
                                    data = 0xff;
                                    break;
                                case 0xd: // control
                                    data = 1;
                                    break;
                                case 0x2: // motion
                                    computeDeltaY();
                                    break;
                                case 0x3: // delta_x
                                    data = 0;
                                    break;
                                case 0x4: // delta_y
                                    data = delta_y * FILDIR;
                                    printf("returning delta_y: %d\n", data);
                                    break;
                                default:
                                    assert(0);
                            }
                            // printf("read 0x%x from addr: 0x%x\n", data, address);
                        }
                        else  {
                            printf("write addr: 0x%x\n", address);
                        }
                    }
                }
                else if (dbitpos) {
                    if (dbitpos == 8)
                        data = 0;
                    data |= dataState << (dbitpos-1);
                    dbitpos--;
                    if (!dbitpos) {
                        printf("write 0x%x to  addr: 0x%x\n", data, address);
                        abitpos = dbitpos = 8;
                    }
                }
                else {
                    assert(0);
                }
            }
            clockState = v;
        }

        void writeBit(uint8_t v) {
            assert(enabled);
            dataState = v != 0;
        }

        uint8_t readBit() {
            assert(enabled);
            assert(!(address & 0x80));
            uint8_t v = data & (0x1 << dbitpos);
            if (!dbitpos)
                abitpos = dbitpos = 8;
            if (v)
                return HIGH;
            else
                return LOW;
        }

        void computeDeltaY() {

            float ediff = sse.pos - lastEPos;
            delta_y = ediff/FILSTEP;
            assert((delta_y >= -128) && (delta_y <= 127));

            if (abs(delta_y) > 1) {

                // unsigned long long ts = getTimestamp();
                data = 1;

                lastEPos += delta_y * FILSTEP;

                if (delta_y > 0) {
                    // simulate slip
                    // estimate speed from OCR1A
                    // float tstep = (float)OCR1A / (F_CPU/8);
                    float clockrate = (float)F_CPU/(8*OCR1A);
                    float speed = clockrate / AXIS_STEPS_PER_MM_X;
                    // float speed = (ediff*1000000) / (ts - lastTime);

                    float slip = 1 - (speed/100) * 0.2;
                    printf("speed: %.2f mm/s, slip: %.2f %\n", speed, slip*100, delta_y);

                    delta_y = delta_y * slip;
                }

                data = delta_y;
                // lastTime = ts;
            }
            else {
                data = 0;
            }
        }
};

extern FilSensorSim filSensorSim;

