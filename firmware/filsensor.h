
#pragma once

class FilamentSensor {

        void mouse_reset();
        // unsigned int readLoc(uint8_t addr);
        uint8_t readLoc(uint8_t addr);
        void pushbyte(uint8_t c);
        // unsigned int pullbyte();
        uint8_t pullbyte();

        int32_t lastASteps;
        int32_t lastYPos;
        uint32_t lastTS;

    public:

        int32_t yPos;

        float slip;
        // xxx use a better name
        // uint16_t maxTempSpeed;

        FilamentSensor();
        void init();
        // The polling method
        void run();
};

extern FilamentSensor filamentSensor;

