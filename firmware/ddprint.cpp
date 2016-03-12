/*
* This file is part of ddprint - a direct drive 3D printer firmware.
* 
* Copyright 2015 erwin.rieger@ibrieger.de
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

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <Arduino.h>
#include <util/crc16.h>

#include "Protothread.h"

#include "ddprint.h"
#include "MarlinSerial.h"
#include "temperature.h"
#include "ddtemp.h"
#include "stepper.h"
#include "swapdev.h"
#include "eepromSettings.h"
#include "filsensor.h"
#include "ddserial.h"
#include "ddcommands.h"

//The ASCII buffer for recieving from SD:
#define SD_BUFFER_SIZE 512

#if 0
#if defined(ExtendedStats)
//
// Bit 0: Line Error
// Bit 1: Checksum Error
// Bit 2: Santax Error
// Bit 3: RX Error
// Bit 4: Timeout Error
#define EFLineError 0x1
#define EFCheckError 0x2
#define EFSyntaxError 0x4
#define EFRXError 0x8
#define EFTimeoutError 0x10
uint8_t errorFlags = 0;
#endif
#endif

// Macros to read scalar types from a buffer
#define FromBuf(typ, adr) ( * ((typ *)(adr)))


uint8_t Stopped = 0;


extern "C"{
  extern unsigned int __bss_end;
  extern unsigned int __heap_start;
  extern unsigned int *__brkval;

  int freeMemory() {
    unsigned int free_memory;

    if(__brkval == 0)
      free_memory = (&free_memory) - (&__bss_end);
    else
      free_memory = (&free_memory) - (__brkval);

    return free_memory;
  }
}


#define USEExtrusionRateTable

/////////////////////////////////////////////////////////////////////////////////////
#if defined(USEExtrusionRateTable)

uint16_t tempExtrusionRateTable[31] = {
    /* temp: 210, max extrusion: 4.80 mm³/s, steps/s: 281, steprate: 3553 us, timervalue: */ 7107,
    /* temp: 212, max extrusion: 5.60 mm³/s, steps/s: 328, steprate: 3046 us, timervalue: */ 6092,
    /* temp: 214, max extrusion: 6.40 mm³/s, steps/s: 375, steprate: 2665 us, timervalue: */ 5330,
    /* temp: 216, max extrusion: 7.20 mm³/s, steps/s: 422, steprate: 2369 us, timervalue: */ 4738,
    /* temp: 218, max extrusion: 8.00 mm³/s, steps/s: 468, steprate: 2132 us, timervalue: */ 4264,
    /* temp: 220, max extrusion: 8.80 mm³/s, steps/s: 515, steprate: 1938 us, timervalue: */ 3876,
    /* temp: 222, max extrusion: 9.60 mm³/s, steps/s: 562, steprate: 1776 us, timervalue: */ 3553,
    /* temp: 224, max extrusion: 10.40 mm³/s, steps/s: 609, steprate: 1640 us, timervalue: */ 3280,
    /* temp: 226, max extrusion: 11.20 mm³/s, steps/s: 656, steprate: 1523 us, timervalue: */ 3046,
    /* temp: 228, max extrusion: 12.00 mm³/s, steps/s: 703, steprate: 1421 us, timervalue: */ 2843,
    /* temp: 230, max extrusion: 12.80 mm³/s, steps/s: 750, steprate: 1332 us, timervalue: */ 2665,
    /* temp: 232, max extrusion: 13.60 mm³/s, steps/s: 797, steprate: 1254 us, timervalue: */ 2508,
    /* temp: 234, max extrusion: 14.40 mm³/s, steps/s: 844, steprate: 1184 us, timervalue: */ 2369,
    /* temp: 236, max extrusion: 15.20 mm³/s, steps/s: 891, steprate: 1122 us, timervalue: */ 2244,
    /* temp: 238, max extrusion: 16.00 mm³/s, steps/s: 937, steprate: 1066 us, timervalue: */ 2132,
    /* temp: 240, max extrusion: 16.80 mm³/s, steps/s: 984, steprate: 1015 us, timervalue: */ 2030,
    /* temp: 242, max extrusion: 17.60 mm³/s, steps/s: 1031, steprate: 969 us, timervalue: */ 1938,
    /* temp: 244, max extrusion: 18.40 mm³/s, steps/s: 1078, steprate: 927 us, timervalue: */ 1854,
    /* temp: 246, max extrusion: 19.20 mm³/s, steps/s: 1125, steprate: 888 us, timervalue: */ 1776,
    /* temp: 248, max extrusion: 20.00 mm³/s, steps/s: 1172, steprate: 852 us, timervalue: */ 1705,
    /* temp: 250, max extrusion: 20.80 mm³/s, steps/s: 1219, steprate: 820 us, timervalue: */ 1640,
    /* temp: 252, max extrusion: 21.60 mm³/s, steps/s: 1266, steprate: 789 us, timervalue: */ 1579,
    /* temp: 254, max extrusion: 22.40 mm³/s, steps/s: 1313, steprate: 761 us, timervalue: */ 1523,
    /* temp: 256, max extrusion: 23.20 mm³/s, steps/s: 1360, steprate: 735 us, timervalue: */ 1470,
    /* temp: 258, max extrusion: 24.00 mm³/s, steps/s: 1406, steprate: 710 us, timervalue: */ 1421,
    /* temp: 260, max extrusion: 24.80 mm³/s, steps/s: 1453, steprate: 687 us, timervalue: */ 1375,
    /* temp: 262, max extrusion: 25.60 mm³/s, steps/s: 1500, steprate: 666 us, timervalue: */ 1332,
    /* temp: 264, max extrusion: 26.40 mm³/s, steps/s: 1547, steprate: 646 us, timervalue: */ 1292,
    /* temp: 266, max extrusion: 27.20 mm³/s, steps/s: 1594, steprate: 627 us, timervalue: */ 1254,
    /* temp: 268, max extrusion: 28.00 mm³/s, steps/s: 1641, steprate: 609 us, timervalue: */ 1218,
    /* temp: 270, max extrusion: 28.80 mm³/s, steps/s: 1688, steprate: 592 us, timervalue: */ 1184,
};
#endif

/////////////////////////////////////////////////////////////////////////////////////




void shutdown() {

    cli(); // Stop interrupts
    disable_heater();
    printer.disableSteppers();

#if defined(PS_ON_PIN) && PS_ON_PIN > -1
    pinMode(PS_ON_PIN,INPUT);
#endif
}


// xxx move to printer class?
void kill() {

    shutdown();
    while(1) {
        // Wait for reset
        txBuffer.Run();
    }
}

void mAssert(uint16_t line, char* file) {

    txBuffer.sendResponseStart(RespKilled, 1+strlen(file));
    txBuffer.sendResponseValue(line);
    txBuffer.sendResponseValue(file, strlen(file));
    txBuffer.sendResponseEnd();
    kill();
}

// bool IsStopped() { return Stopped; };
// uint8_t StoppedReason() { return Stopped; };

// xxx move to printer class?
#if 0
void Stop(uint8_t reasonNr)
{

  disable_heater();
  // disable steppers here ? printer.disableSteppers();

  if(Stopped == false) {
    Stopped = reasonNr;
    // Stopped_gcode_LastN = usbCommand->serialNumber; // Save last g_code for restart
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}
#endif

void setup() {

    // Do some minimal SPI init, prevent SPI to go to spi slave mode
    WRITE(SDSS, HIGH);
    SET_OUTPUT(SDSS);

    WRITE(FILSENSNCS, HIGH);
    SET_OUTPUT(FILSENSNCS);

    MSerial.begin(BAUDRATE);
    // SERIAL_PROTOCOLLNPGM("start");

    // SERIAL_ECHO_START;
    // SERIAL_ECHOPGM(MSG_FREE_MEMORY);
    // SERIAL_ECHOLN(freeMemory());
    // SERIAL_ECHOPGM("Min/Max steps: X: ");
    // SERIAL_ECHO(X_MIN_POS_STEPS);
    // SERIAL_ECHO(", ");
    // SERIAL_ECHO(X_MAX_POS_STEPS);
    // SERIAL_ECHO("; Y: ");
    // SERIAL_ECHO(Y_MIN_POS_STEPS);
    // SERIAL_ECHO(", ");
    // SERIAL_ECHO(Y_MAX_POS_STEPS);
    // SERIAL_ECHO("; Z: ");
    // SERIAL_ECHO(Z_MIN_POS_STEPS);
    // SERIAL_ECHO(", ");
    // SERIAL_ECHOLN(Z_MAX_POS_STEPS);

    // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
    // Config_RetrieveSettings();
    // dumpEepromSettings("Eeprom:");

    tp_init();    // Initialize temperature loop

    wdt_enable(WDTO_4S);

    st_init();    // Initialize stepper, this enables interrupts!

    analogWrite(LED_PIN, 255 * 0.5);

    // lcd_init();

    if (! swapDev.swapInit()) {
        txBuffer.sendSimpleResponse(RespKilled, RespSDInit);
        kill();
    }

#if defined(ADNSFS)
    filamentSensor.reset();
#endif

#if defined(ADNSFS) || defined(BournsEMS22AFS)
    filamentSensor.init();
#endif
}

// Block-buffered sd read
class SDReader: public Protothread {

        // Buffer for block-wise reading of swap memory
        uint8_t buffer[SD_BUFFER_SIZE];

        // Number of characters in buffer
        int16_t bufferLength;

        // Readpointer into buffer
        uint16_t bufferPtr;

        // Number of bytes to read on current thread run
        uint16_t bytesToRead;

        // Temporary buffer if we cross a block boundary,
        // the size if 4 bytes - the length of the longest
        // datatype to read (int32).
        uint8_t tempBuffer[4]; 

        uint16_t haveBytes;

    public:

        // Pointer to the result data, points into buffer directly or
        // to tempBuffer.
        uint8_t *readData;

        SDReader() {
            bufferLength = 0;
            bufferPtr = 0;
        }

        FWINLINE void setBytesToRead1() {
            Restart();
            bytesToRead = 1; }
        FWINLINE void setBytesToRead2() {
            Restart();
            bytesToRead = 2; }
        FWINLINE void setBytesToRead3() {
            Restart();
            bytesToRead = 3; }
        FWINLINE void setBytesToRead4() {
            Restart();
            bytesToRead = 4; }

        bool Run() {

            uint8_t i;

            PT_BEGIN();

            simassert((bytesToRead > 0) && (bytesToRead<5));
            simassert(bufferPtr <= bufferLength);

            haveBytes = bufferLength - bufferPtr;

            if (haveBytes < bytesToRead) {

                //
                // Read data from swap memory if not enough data in buffer 
                //
                if (haveBytes) {
                    //
                    // Copy the last bytes to temp buffer
                    //
                    for (i=0; i<haveBytes; i++) {
                        tempBuffer[i] = buffer[bufferPtr+i]; 
                        bytesToRead--;
                    }

                    //
                    // Get new block from swapmem
                    //
                    PT_WAIT_WHILE(swapDev.isBusyWriting());
                    PT_WAIT_UNTIL(swapDev.available());

                    bufferLength = swapDev.readBlock(buffer);

                    for (i=0; i<bytesToRead; i++) {
                        tempBuffer[haveBytes+i] = buffer[i]; 
                    }

                    readData = tempBuffer;
                    bufferPtr = bytesToRead;
                }
                else {

                    //
                    // Buffer empty, get new block from swapmem
                    //
                    PT_WAIT_WHILE(swapDev.isBusyWriting());
                    PT_WAIT_UNTIL(swapDev.available());

                    bufferLength = swapDev.readBlock(buffer);

                    readData = buffer;
                    bufferPtr = bytesToRead;
                }

                // massert(bufferLength > 0);
            }
            else {

                // Enough data in buffer to satisfy request
                readData = buffer + bufferPtr;
                bufferPtr += bytesToRead;
            }

            PT_END();
        }

        FWINLINE uint16_t available() {

            simassert(bufferLength >= bufferPtr);
            return bufferLength - bufferPtr;
        }

        uint16_t getBufferPtr() { return bufferPtr; }

        void flush() {

            bufferLength = bufferPtr = 0;
            // bytesToRead = 0;

            //
            // Restart is done in setBytesToReadX().
            //
            swapDev.reset();
        }

};


uint16_t STD_max(uint16_t a, uint16_t b) {

    // static uint16_t counter = 0;

    if (b > a) {

#if 0
        if ((counter++ % 100) == 0) {
            SERIAL_ECHOPGM("Tempspeed ");
            SERIAL_ECHO(counter);
            SERIAL_ECHOPGM(", ");
            SERIAL_ECHO(current_temperature[0]);
            SERIAL_ECHOPGM(", ");
            SERIAL_ECHO(a);
            SERIAL_ECHOPGM(", ");
            SERIAL_ECHOLN(b);
        }
#endif
    }

    return STD max(a, b);
}

static SDReader sDReader;

#if defined(USEExtrusionRateTable)
    #define MAXTEMPSPEED maxTempSpeed
#else
    #define MAXTEMPSPEED filamentSensor.maxTempSpeed
#endif

class FillBufferTask : public Protothread {

        uint8_t cmd;
        uint8_t cmdDir;
        uint8_t stepBits;
        uint8_t timerLoop;
        uint16_t timer;

        uint16_t nAccel;
        uint8_t leadAxis;
        uint16_t tLin;
        uint16_t nDeccel;
        int32_t absSteps[5];
#if defined(USEExtrusionRateTable)
        uint16_t maxTempSpeed;
#endif
            // uint16_t leadFactor;
            // int16_t curTempIndex;

        // Bresenham factors
        int32_t d_axis[5];
        int32_t d1_axis[5];
        int32_t d2_axis[5];

        int32_t deltaLead, step;

        // Hotend target temp for CmdSyncTargetTemp
        uint8_t targetHeater;
        uint16_t targetTemp;

        unsigned long dwellEnd;

        // StepBlock stepBlock;

    public:
        FillBufferTask() {
            cmdDir = 0;
        }

        bool Run() {

            uint8_t i;

            int32_t d;
            int32_t d1;
            int32_t d2;

#if defined(USEExtrusionRateTable)
            uint16_t leadFactor;
            int16_t curTempIndex;
#endif

            PT_BEGIN();

            // SERIAL_ECHOPGM(" readSwap, sdr avail: ");
            // SERIAL_ECHO(sDReader.available());
            // SERIAL_ECHOPGM(", swapavail: ");
            // SERIAL_ECHOLN(swapDev.available());

            #if 0
            # Move segment data, new with bresenham in firmware:
            #   * Header data:
            #       + index lead axis, 8 bits
            #       + array of absolute steps, 5 * 16 bits
            #       + number of accel steps, naccel, 16 bits
            #       + constant linear timer value, 16 bits
            #       + number of deccel steps, ndeccel, 16 bits
            #
            #   * accel steps: naccel*(timer value(16bits))
            #
            #   * deccel steps: ndeccel*(timer value(16bits))
            #
            #endif

            sDReader.setBytesToRead1();
            PT_WAIT_THREAD(sDReader);
            cmd = *sDReader.readData;

            // Skip packet size
            sDReader.setBytesToRead4();
            PT_WAIT_THREAD(sDReader);

            switch (cmd) {

                case CmdG1:
                    // stepBlock.cmdDirBits &= ~0x80; // clear set-direction bit
                    goto HandleCmdG1;

                case CmdDirG1:
                    goto HandleCmdDirG1;

                case CmdG1_24:
                    // stepBlock.cmdDirBits &= ~0x80; // clear set-direction bit
                    goto HandleCmdG1_24;

                case CmdDirG1_24:
                    goto HandleCmdDirG1_24;

                // case CmdDirBits:
                    // goto HandleCmdDirBits;

                case CmdSyncFanSpeed:
                    goto HandleCmdSyncFanSpeed;

                case CmdSyncTargetTemp:
                    goto HandleCmdSyncTargetTemp;

                case CmdDwellMS:
                    goto HandleCmdDwellMS;

                default:
#if 0
                    SERIAL_ECHOPGM("Error: UNKNOWN command byte: ");
                    SERIAL_ECHO((int)cmd);
                    SERIAL_ECHOPGM(", Swapsize: ");
                    SERIAL_ECHO(swapDev.getSize());
                    SERIAL_ECHOPGM(", SwapReadPos is: ");
                    SERIAL_ECHO(swapDev.getReadPos());
                    SERIAL_ECHOPGM(", SDRReadPos+5 is: ");
                    SERIAL_ECHOLN(sDReader.getBufferPtr());
#endif

                    txBuffer.sendResponseStart(RespKilled, 2);
                    txBuffer.pushCharChecksum(RespUnknownBCommand);
                    txBuffer.pushCharChecksum(cmd);
                    txBuffer.sendResponseEnd();
                    kill();
            }

            // XXX use template to merge CmdG1 and CmdG1_24
            HandleCmdDirG1:

                sDReader.setBytesToRead1();
                PT_WAIT_THREAD(sDReader);

                cmdDir = *sDReader.readData;

            HandleCmdG1:

                // SERIAL_ECHOLNPGM("C1");

                //
                // Read index of lead axis
                //
                sDReader.setBytesToRead1();
                PT_WAIT_THREAD(sDReader);
                leadAxis = *sDReader.readData;

                //
                // Read array of absolute step distances of the 5 axes
                //
                sDReader.setBytesToRead4();
                PT_WAIT_THREAD(sDReader);
                absSteps[0] = FromBuf(int32_t, sDReader.readData);

                sDReader.setBytesToRead4();
                PT_WAIT_THREAD(sDReader);
                absSteps[1] = FromBuf(int32_t, sDReader.readData);

                sDReader.setBytesToRead4();
                PT_WAIT_THREAD(sDReader);
                absSteps[2] = FromBuf(int32_t, sDReader.readData);

                sDReader.setBytesToRead4();
                PT_WAIT_THREAD(sDReader);
                absSteps[3] = FromBuf(int32_t, sDReader.readData);

                sDReader.setBytesToRead4();
                PT_WAIT_THREAD(sDReader);
                absSteps[4] = FromBuf(int32_t, sDReader.readData);

                // nAccel = sDReader.readPayloadUInt16();
                sDReader.setBytesToRead2();
                PT_WAIT_THREAD(sDReader);
                nAccel = FromBuf(uint16_t, sDReader.readData);
                // SERIAL_ECHOLN(nAccel);

                //////////////////////////////////////////////////////
                sDReader.setBytesToRead2();
                PT_WAIT_THREAD(sDReader);

                // xxx unused:
                filamentSensor.enabled = true;

#if defined(USEExtrusionRateTable)
                leadFactor = FromBuf(uint16_t, sDReader.readData);
#endif

#if defined(USEExtrusionRateTable)
                if (leadFactor) {

                    // curTempIndex = (int16_t)(current_temperature[0] - 210);
                    // curTempIndex = (int16_t)(current_temperature[0] - 130);
                    curTempIndex = (int16_t)(current_temperature[0] - 210) / 2;

                    if (curTempIndex < 0) {

                        maxTempSpeed = ((uint32_t)tempExtrusionRateTable[0] * 1000) / leadFactor;
                    }
                    else if (curTempIndex > 30) { // xxx 30 hardcoded

                        maxTempSpeed = ((uint32_t)tempExtrusionRateTable[30] * 1000) / leadFactor;
                    }
                    else {
#if 0
                        SERIAL_ECHOPGM(" Tempspeed tindex ");
                        SERIAL_ECHO(curTempIndex);
                        SERIAL_ECHOPGM(" tabval ");
                        SERIAL_ECHOLN(tempExtrusionRateTable[curTempIndex]);
#endif
                        maxTempSpeed = ((uint32_t)tempExtrusionRateTable[curTempIndex] * 1000) / leadFactor;
                    }
                }
                else {
                    maxTempSpeed = 0;
                }
#endif
    
                // if (leadFactor < 0xffff)
                    // maxTempSpeed = (uint32_t)(1485 / (leadFactor/10000.0));
                // printf("tempindex: %d, tabvalue: %d, leadfactor: %.2f, maxTempSpeed: %d\n", curTempIndex, (uint32_t)tempExtrusionRateTable[curTempIndex], leadFactor/10000.0, maxTempSpeed);

                //////////////////////////////////////////////////////

                // tLin = sDReader.readPayloadUInt16();
                sDReader.setBytesToRead2();
                PT_WAIT_THREAD(sDReader);

                tLin = STD_max( FromBuf(uint16_t, sDReader.readData), MAXTEMPSPEED);

#if 0
                tLin = FromBuf(uint16_t, sDReader.readData);
                    SERIAL_ECHOPGM("tl ");
                    SERIAL_ECHO(tLin);
                    SERIAL_ECHOPGM(" ms  ");
                    SERIAL_ECHOLN(maxTempSpeed);

                if (maxTempSpeed > tLin) {
                    SERIAL_ECHOPGM(" Tempspeed tindex ");
                    SERIAL_ECHO(curTempIndex);
                    SERIAL_ECHOPGM(" tabval ");
                    SERIAL_ECHO(tempExtrusionRateTable[curTempIndex]);
                    SERIAL_ECHOPGM(" lf ");
                    SERIAL_ECHO(leadFactor);
                    SERIAL_ECHOPGM(" tempspd ");
                    SERIAL_ECHO(maxTempSpeed);
                    SERIAL_ECHOPGM(" f1 ");
                    SERIAL_ECHO(((uint32_t)tempExtrusionRateTable[curTempIndex] * 1000));
                    SERIAL_ECHOPGM(" f2 ");
                    SERIAL_ECHOLN(((uint32_t)tempExtrusionRateTable[curTempIndex] * 1000) / leadFactor);
                }
#endif

                // nDeccel = sDReader.readPayloadUInt16();
                sDReader.setBytesToRead2();
                PT_WAIT_THREAD(sDReader);
                nDeccel = FromBuf(uint16_t, sDReader.readData);
                // SERIAL_ECHOLN(nDeccel);

                //
                // Compute bresenham factors
                //

                deltaLead = absSteps[leadAxis];

                //  d = (2 * deltay) - deltax 
                //    = d1 - deltax
                // d1 = (2 * deltay)
                // d2 = 2 * (deltay - deltax)
                //    = 2 * deltay - 2 * deltax
                //    = d - deltax
                for (i=0; i<5; i++) {

                    if (i == leadAxis)
                        continue;

                    d1 = 2 * absSteps[i];
                    d = d1 - deltaLead;
                    d2 = d - deltaLead;

                    d_axis[i] = d;
                    d1_axis[i] = d1;
                    d2_axis[i] = d2;

                    // SERIAL_ECHOPGM("d/d1/d2: ");
                    // SERIAL_ECHO(d);
                    // SERIAL_ECHOPGM(",");
                    // SERIAL_ECHO(d1);
                    // SERIAL_ECHOPGM(",");
                    // SERIAL_ECHOLN(d2);
                }

                for (step=0; step < deltaLead; step++) {

                    //
                    // Compute stepper bits, bresenham
                    //
                    stepBits = 1 << leadAxis;

                    for (i=0; i<5; i++) {
                        if (i == leadAxis)
                            continue;

                        if (d_axis[i] < 0) {
                            //  d_axis[a] = d + 2 * abs_displacement_vector_steps[a]
                            d_axis[i] += d1_axis[i];
                        }
                        else {
                            //  d_axis[a] = d + 2 * (abs_displacement_vector_steps[a] - deltaLead)
                            d_axis[i] += d2_axis[i];
                            stepBits |= 1 << i;
                        }
                    }

                    //
                    // Get timer value
                    //

                    if (step < nAccel) {

                        // Acceleration
                        sDReader.setBytesToRead2();
                        PT_WAIT_THREAD(sDReader);
                        timer = STD_max( FromBuf(uint16_t, sDReader.readData), MAXTEMPSPEED );

                        PT_WAIT_WHILE(stepBuffer.full());
                        if (timer & 0xff00)
                            stepBuffer.push4(cmdDir, stepBits, timer);
                        else
                            stepBuffer.push3(cmdDir, stepBits, timer);
                    }
                    else if ((deltaLead - step) <= nDeccel) { // TODO: speed up by combining nDeccel and deltaLead to remove subtract

                        // Decceleration
                        sDReader.setBytesToRead2();
                        PT_WAIT_THREAD(sDReader);
                        timer = STD_max( FromBuf(uint16_t, sDReader.readData), MAXTEMPSPEED );

                        PT_WAIT_WHILE(stepBuffer.full());
                        if (timer & 0xff00)
                            stepBuffer.push4(cmdDir, stepBits, timer);
                        else
                            stepBuffer.push3(cmdDir, stepBits, timer);
                    }
                    else {
                        // linear part
                        PT_WAIT_WHILE(stepBuffer.full());
                        if (tLin & 0xff00)
                            stepBuffer.push4(cmdDir, stepBits, tLin);
                        else
                            stepBuffer.push3(cmdDir, stepBits, tLin);
                    }

                    cmdDir &= ~0x80; // clear set-direction bit
                }
            
                PT_RESTART();

            HandleCmdDirG1_24:

                sDReader.setBytesToRead1();
                PT_WAIT_THREAD(sDReader);

                cmdDir = *sDReader.readData;

            HandleCmdG1_24:

                // SERIAL_ECHOLNPGM("C1");

                //
                // Read index of lead axis
                //
                sDReader.setBytesToRead1();
                PT_WAIT_THREAD(sDReader);
                leadAxis = *sDReader.readData;

                //
                // Read array of absolute step distances of the 5 axes
                //
                sDReader.setBytesToRead4();
                PT_WAIT_THREAD(sDReader);
                absSteps[0] = FromBuf(int32_t, sDReader.readData);

                sDReader.setBytesToRead4();
                PT_WAIT_THREAD(sDReader);
                absSteps[1] = FromBuf(int32_t, sDReader.readData);

                sDReader.setBytesToRead4();
                PT_WAIT_THREAD(sDReader);
                absSteps[2] = FromBuf(int32_t, sDReader.readData);

                sDReader.setBytesToRead4();
                PT_WAIT_THREAD(sDReader);
                absSteps[3] = FromBuf(int32_t, sDReader.readData);

                sDReader.setBytesToRead4();
                PT_WAIT_THREAD(sDReader);
                absSteps[4] = FromBuf(int32_t, sDReader.readData);

                // nAccel = sDReader.readPayloadUInt16();
                sDReader.setBytesToRead2();
                PT_WAIT_THREAD(sDReader);
                nAccel = FromBuf(uint16_t, sDReader.readData);
                // SERIAL_ECHOLN(nAccel);

                // tLin = sDReader.readPayloadUInt16();
                sDReader.setBytesToRead2();
                PT_WAIT_THREAD(sDReader);
                tLin = FromBuf(uint16_t, sDReader.readData);

                // nDeccel = sDReader.readPayloadUInt16();
                sDReader.setBytesToRead2();
                PT_WAIT_THREAD(sDReader);
                nDeccel = FromBuf(uint16_t, sDReader.readData);
                // SERIAL_ECHOLN(nDeccel);

                //
                // Compute bresenham factors
                //

                deltaLead = absSteps[leadAxis];

                //  d = (2 * deltay) - deltax 
                //    = d1 - deltax
                // d1 = (2 * deltay)
                // d2 = 2 * (deltay - deltax)
                //    = 2 * deltay - 2 * deltax
                //    = d - deltax
                for (i=0; i<5; i++) {

                    if (i == leadAxis)
                        continue;

                    d1 = 2 * absSteps[i];
                    d = d1 - deltaLead;
                    d2 = d - deltaLead;

                    d_axis[i] = d;
                    d1_axis[i] = d1;
                    d2_axis[i] = d2;

                    // SERIAL_ECHOPGM("d/d1/d2: ");
                    // SERIAL_ECHO(d);
                    // SERIAL_ECHOPGM(",");
                    // SERIAL_ECHO(d1);
                    // SERIAL_ECHOPGM(",");
                    // SERIAL_ECHOLN(d2);
                }


                for (step=0; step < deltaLead; step++) {

                    //
                    // Compute stepper bits, bresenham
                    //
                    stepBits = 1 << leadAxis;

                    for (i=0; i<5; i++) {
                        if (i == leadAxis)
                            continue;

                        if (d_axis[i] < 0) {
                            //  d_axis[a] = d + 2 * abs_displacement_vector_steps[a]
                            d_axis[i] += d1_axis[i];
                        }
                        else {
                            //  d_axis[a] = d + 2 * (abs_displacement_vector_steps[a] - deltaLead)
                            d_axis[i] += d2_axis[i];
                            stepBits |= 1 << i;
                        }
                    }

                    //
                    // Get timer value
                    //

                    if (step < nAccel) {

                        // Acceleration
                        sDReader.setBytesToRead3();
                        PT_WAIT_THREAD(sDReader);
                        timerLoop = FromBuf(uint8_t, sDReader.readData);
                        timer = FromBuf(uint16_t, sDReader.readData);

                        PT_WAIT_WHILE(stepBuffer.full());
                        if (timer & 0xff00) {
                            if (timerLoop)
                                stepBuffer.push5(cmdDir, stepBits, timer, timerLoop);
                            else
                                stepBuffer.push4(cmdDir, stepBits, timer);
                        }
                        else
                            stepBuffer.push3(cmdDir, stepBits, timer);
                    }
                    else if ((deltaLead - step) <= nDeccel) {

                        // Decceleration
                        sDReader.setBytesToRead3();
                        PT_WAIT_THREAD(sDReader);
                        timerLoop = FromBuf(uint8_t, sDReader.readData);
                        timer = FromBuf(uint16_t, sDReader.readData);

                        PT_WAIT_WHILE(stepBuffer.full());
                        if (timer & 0xff00) {
                            if (timerLoop)
                                stepBuffer.push5(cmdDir, stepBits, timer, timerLoop);
                            else
                                stepBuffer.push4(cmdDir, stepBits, timer);
                        }
                        else
                            stepBuffer.push3(cmdDir, stepBits, timer);
                    }
                    else {
                        // linear part
                        PT_WAIT_WHILE(stepBuffer.full());
                        if (tLin & 0xff00)
                            stepBuffer.push4(cmdDir, stepBits, tLin);
                        else
                            stepBuffer.push3(cmdDir, stepBits, tLin);
                    }

                    cmdDir &= ~0x80; // clear set-direction bit
                }
                PT_RESTART();

#if 0
            HandleCmdDirBits:

                // SERIAL_ECHOLNPGM("C2");

                // stepBlock.data.dir.dirBits = sDReader.readPayload1();
                sDReader.setBytesToRead1();
                PT_WAIT_THREAD(sDReader);

                stepBlock.data.dir.dirBits = *sDReader.readData;

                PT_WAIT_WHILE(stepBuffer.full());
                stepBuffer.push(stepBlock);

                PT_RESTART();
#endif
            HandleCmdSyncFanSpeed:

                // SERIAL_ECHOLNPGM("C3");

                sDReader.setBytesToRead1();
                PT_WAIT_THREAD(sDReader);

                printer.cmdFanSpeed(*sDReader.readData);

                PT_RESTART();

            HandleCmdSyncTargetTemp:

                sDReader.setBytesToRead3();
                PT_WAIT_THREAD(sDReader);

                targetHeater = *sDReader.readData;
                targetTemp = FromBuf(uint16_t, sDReader.readData+1);

                PT_WAIT_UNTIL( printer.printerState == Printer::StateStart );

                // printf("autotemp: heater %d, temp: %d\n", *sDReader.readData, *(sDReader.readData+1));
                // printer.cmdSetTargetTemp(*sDReader.readData, FromBuf(uint16_t, sDReader.readData+1));
                printer.cmdSetTargetTemp(targetHeater, targetTemp);

                PT_RESTART();

            HandleCmdDwellMS:

                sDReader.setBytesToRead2();
                PT_WAIT_THREAD(sDReader);

                dwellEnd = millis() + FromBuf(uint16_t, sDReader.readData);
                printer.dwellStart();

                PT_WAIT_WHILE(millis() < dwellEnd);
                printer.dwellEnd();

                PT_RESTART();

            PT_END(); // Not reached
        }

        void flush() {
            step = deltaLead;
            Restart();
        }
};

FillBufferTask fillBufferTask;

Printer::Printer() {

    printerState = StateIdle;
    moveType = MoveTypeNone;
    homed[0] = false;
    homed[1] = false;
    homed[2] = false;
    swapErased = false;
};

void Printer::printerInit() {

    // XXX handle already running state

    //
    // Erase sd-swap to speed up block writes.
    //
    if (! swapErased) {

        // unsigned long eStart = millis();

        massert(swapDev.erase(0, swapDev.cardSize() - 1));

        // SERIAL_ECHO("erase time ");
        // SERIAL_ECHO(swapDev.cardSize());
        // SERIAL_ECHO(" blocks (mS):");
        // SERIAL_PROTOCOLLN(millis() - eStart);

        swapErased = true;
    }

//xxxxxxxxxxx
sDReader.flush();
fillBufferTask.flush();
stepBuffer.flush();

    swapDev.reset();

    printerState = StateInit;
    eotReceived = false;

    analogWrite(LED_PIN, 255);
}

void Printer::cmdEot() {

    massert(printerState >= StateInit);

    eotReceived = true;
}

void Printer::cmdMove(MoveType mt) {

    //
    // Busy state des swapmem ist erlaubt hier.
    //
    // massert(! swapDev.isWriteBusy());

    massert(mt != MoveTypeNone);
    massert(printerState == StateInit);

    printerState = StateStart;
    moveType = mt;

    if (mt == MoveTypeNormal) {
        massert(homed[0]);
        massert(homed[1]);
        massert(homed[2]);

        // xxx check if data is available (swapDev.getSize() > 0)?
        
        // EepromSettings es;
        // getEepromSettings(es);
        // Compute max z step pos for software endstops, xxx um2 specific.
        // z_max_pos_steps = (long)((Z_MAX_POS + es.add_homeing[Z_AXIS]) * AXIS_STEPS_PER_MM_Z);
    }

    if (mt == MoveTypeHoming) {
        ENABLE_STEPPER1_DRIVER_INTERRUPT();
    }
    else {
        ENABLE_STEPPER_DRIVER_INTERRUPT();
    }

#if 0
    if (mt == MoveTypeForced) {

        // Prevent trigger of software endstop
        current_pos_steps[X_AXIS] = (X_MAX_POS_STEPS - X_MIN_POS_STEPS) / 2;
        current_pos_steps[Y_AXIS] = (Y_MAX_POS_STEPS - Y_MIN_POS_STEPS) / 2;
        current_pos_steps[Z_AXIS] = (Z_MAX_POS_STEPS - Z_MIN_POS_STEPS) / 2;
    }
#endif

    enable_x();
    enable_y();
    enable_z();
    enable_e0();

    // xxxxxxxxxxxxxx
    // SERIAL_ECHOPGM("start: readSwap, size: ");
    // SERIAL_ECHO(swapDev.getSize());
    // SERIAL_ECHOPGM(", SwapReadPos: ");
    // SERIAL_ECHO(swapDev.getReadPos());
    // SERIAL_ECHOPGM(", SDRReadPos is: ");
    // SERIAL_ECHOLN(sDReader.getBufferPtr());
    // xxxxxxxxxxxxxx

    bufferLow = -1;

#if defined(ADNSFS) || defined(BournsEMS22AFS)
    filamentSensor.init();
#endif
}

void Printer::setHomePos(int32_t x, int32_t y, int32_t z) {
         
    homed[0] = true;
    homed[1] = true;
    homed[2] = true;

    current_pos_steps[X_AXIS] = x;
    current_pos_steps[Y_AXIS] = y;
    current_pos_steps[Z_AXIS] = z;
}

void Printer::cmdSetTargetTemp(uint8_t heater, uint16_t temp) {

    if (heater == 0)
        target_temperature_bed = temp;
    else
        target_temperature[heater-1] = temp;
}

void Printer::cmdFanSpeed(uint8_t speed) {

    analogWrite(FAN_PIN, speed);
}

void Printer::cmdStopMove() {

    cmdSetTargetTemp(0, 20);
    cmdSetTargetTemp(1, 20);

    DISABLE_STEPPER_DRIVER_INTERRUPT();
    DISABLE_STEPPER1_DRIVER_INTERRUPT();

    // printerState = StateInit;
    printerState = StateIdle;
    moveType = MoveTypeNone;

    // Flush remaining steps
    sDReader.flush();
    fillBufferTask.flush();
    stepBuffer.flush();

    cmdFanSpeed(0);
}

void Printer::cmdGetTargetTemps() {

#if 0
    SERIAL_ECHOPGM("Res:(");
    SERIAL_ECHO(target_temperature_bed);
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(target_temperature[0]);
#if EXTRUDERS > 1
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(target_temperature[1]);
#endif
    SERIAL_ECHOLNPGM(")");
#endif

#if EXTRUDERS == 1
    txBuffer.sendResponseStart(CmdGetTargetTemps, 1 + sizeof(uint16_t));
#else
    txBuffer.sendResponseStart(CmdGetTargetTemps, 1 + 2*sizeof(uint16_t));
#endif

    txBuffer.pushCharChecksum(target_temperature_bed);
    txBuffer.sendResponseValue(target_temperature[0]);
#if EXTRUDERS > 1
    txBuffer.sendResponseValue(target_temperature[1]);
#endif
    txBuffer.sendResponseEnd();
}

void Printer::cmdGetCurrentTemps() {

#if 0
    SERIAL_ECHOPGM("Res:(");
    SERIAL_ECHO(current_temperature_bed);
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(current_temperature[0]);
#if EXTRUDERS > 1
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(current_temperature[1]);
#endif
    SERIAL_ECHOLNPGM(")");
#endif

#if EXTRUDERS == 1
    txBuffer.sendResponseStart(CmdGetCurrentTemps, 2*sizeof(float));
#else
    txBuffer.sendResponseStart(CmdGetCurrentTemps, 3*sizeof(float));
#endif

    txBuffer.sendResponseValue(current_temperature_bed);
    txBuffer.sendResponseValue(current_temperature[0]);
#if EXTRUDERS > 1
    txBuffer.sendResponseValue(current_temperature[1]);
#endif
    txBuffer.sendResponseEnd();
}

void Printer::checkMoveFinished() {

    //
    // Move is finished if:
    // * eot was received (sender has sent all data)
    // * read file pos is at the end of file
    // * buffers are empty
    //
    if (printerState == StateStart) {

        if ((moveType == MoveTypeHoming) && (! STEPPER1_DRIVER_INTERRUPT_ENABLED())) {

            // Flush remaining steps
            sDReader.flush();
            fillBufferTask.flush();
            stepBuffer.flush();
        }

        if ( eotReceived &&
             (! swapDev.isBusyWriting()) &&
             (! swapDev.available()) &&
             (! sDReader.available()) &&
             stepBuffer.empty() ) {

            DISABLE_STEPPER_DRIVER_INTERRUPT();
            DISABLE_STEPPER1_DRIVER_INTERRUPT();

            // SERIAL_PROTOCOLLNPGM("Move finished.");

            // printerState = StateInit;
            printerState = StateIdle;
            moveType = MoveTypeNone;
        }
    }
}

void Printer::disableSteppers() {

    disable_x();
    disable_y();
    disable_z();
    disable_e0();

    homed[0] = false;
    homed[1] = false;
    homed[2] = false;

    analogWrite(LED_PIN, 255 * 0.5);
}

void Printer::cmdDisableSteppers() {

    disableSteppers();
}

#if 0
// Currently not used:
void Printer::cmdDisableStepperIsr() {

    DISABLE_STEPPER_DRIVER_INTERRUPT();
    DISABLE_STEPPER1_DRIVER_INTERRUPT();

    SERIAL_PROTOCOLLNPGM(MSG_OK);
}

void Printer::cmdGetState() {

    SERIAL_ECHOPGM("Res:");
    SERIAL_ECHOLN(printerState);
}
#endif

void Printer::cmdGetHomed() {

#if 0
    SERIAL_ECHOPGM("Res:(");
    SERIAL_ECHO(homed[0]);
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(homed[1]);
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(homed[2]);
    SERIAL_ECHOLNPGM(")");
#endif

    txBuffer.sendResponseStart(CmdGetHomed, sizeof(homed));
    txBuffer.sendResponseValue((uint8_t*)homed, sizeof(homed));
    txBuffer.sendResponseEnd();
}

void Printer::cmdGetEndstops() {

#if 0
    SERIAL_ECHOPGM("Res:((");
    SERIAL_ECHO(X_ENDSTOP_PRESSED);
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(current_pos_steps[X_AXIS]);
    SERIAL_ECHOPGM("),(");
    SERIAL_ECHO(Y_ENDSTOP_PRESSED);
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(current_pos_steps[Y_AXIS]);
    SERIAL_ECHOPGM("),(");
    SERIAL_ECHO(Z_ENDSTOP_PRESSED);
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(current_pos_steps[Z_AXIS]);
    SERIAL_ECHOLNPGM("))");
#endif

    txBuffer.sendResponseStart(CmdGetEndstops, 3*(sizeof(int32_t)+1));

    txBuffer.pushCharChecksum(X_ENDSTOP_PRESSED);
    txBuffer.sendResponseValue((int32_t)current_pos_steps[X_AXIS]);

    txBuffer.pushCharChecksum(Y_ENDSTOP_PRESSED);
    txBuffer.sendResponseValue((int32_t)current_pos_steps[Y_AXIS]);

    txBuffer.pushCharChecksum(Z_ENDSTOP_PRESSED);
    txBuffer.sendResponseValue((int32_t)current_pos_steps[Z_AXIS]);

    txBuffer.sendResponseEnd();
}

void Printer::cmdGetPos() {

#if 0
    SERIAL_ECHOPGM("Res:(");
    SERIAL_ECHO(current_pos_steps[X_AXIS]);
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(current_pos_steps[Y_AXIS]);
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(current_pos_steps[Z_AXIS]);
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(current_pos_steps[E_AXIS]);
    // SERIAL_ECHOPGM(",");
    // SERIAL_ECHO(current_pos_steps[B_AXIS]);
    SERIAL_ECHOLNPGM(")");
#endif

    txBuffer.sendResponseStart(CmdGetPos, sizeof(current_pos_steps));
    txBuffer.sendResponseValue((uint8_t*)current_pos_steps, sizeof(current_pos_steps));
    txBuffer.sendResponseEnd();
}

// uint16_t waitCount = 0;
void Printer::cmdGetStatus() {
#if 0
    // ["state", "t0", "t1", "Swap", "SDReader", "StepBuffer", "StepBufUnderRuns", "targetT1"]
    SERIAL_ECHOPGM("Res:(");
    SERIAL_ECHO(printerState);
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(current_temperature_bed);
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(current_temperature[0]);
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(swapDev.available());
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(sDReader.available());
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(stepBuffer.byteSize());
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO((uint16_t)bufferLow);
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(target_temperature[0]);
    SERIAL_ECHOPGM(",");
    SERIAL_ECHO(waitCount);
    SERIAL_ECHOLNPGM(")");
#endif

    txBuffer.sendResponseStart(CmdGetStatus,
            sizeof(uint8_t) +
            sizeof(float) +
            sizeof(float) +
            sizeof(uint32_t) +
            sizeof(uint16_t) +
            sizeof(uint16_t) +
            sizeof(uint16_t) +
            sizeof(uint16_t)
            );

    txBuffer.pushCharChecksum(printerState);
    txBuffer.sendResponseValue(current_temperature_bed);
    txBuffer.sendResponseValue(current_temperature[0]);
    txBuffer.sendResponseValue(swapDev.available());
    txBuffer.sendResponseValue(sDReader.available());
    txBuffer.sendResponseValue(stepBuffer.byteSize());
    txBuffer.sendResponseValue((uint16_t)bufferLow);
    txBuffer.sendResponseValue(target_temperature[0]);

    txBuffer.sendResponseEnd();
}

#if defined(ADNSFS) || defined(BournsEMS22AFS)
void Printer::cmdGetFilSensor() {

#if 0
    SERIAL_ECHOPGM("Res:");
    SERIAL_ECHOLN(filamentSensor.yPos);
#endif
}
#endif

void Printer::cmdGetTempTable() {

    uint16_t temp = 210; // xxx fixed

#if 0
    SERIAL_ECHOPGM("Res:[");

    for (uint8_t i=0; i<31; i++, temp+=2) { // fixed

        SERIAL_ECHOPGM("(");
        SERIAL_ECHO(temp);
        SERIAL_ECHOPGM(",");
        SERIAL_ECHO(tempExtrusionRateTable[i]);
        SERIAL_ECHOPGM("),");
    }
    SERIAL_ECHOLNPGM("]");
#endif
}

void Printer::dwellStart() {

    massert(printerState == StateStart);
    // massert(moveType == MoveTypeNormal);

    printerState = StateDwell;
    // moveType = MoveTypeNone;
}

void Printer::dwellEnd() {

    printerState = StateStart;
    // moveType = MoveTypeNormal;
    bufferLow = -1;
}

Printer printer;

// Send ascii-ACK 0x6 for accepted usbserial command.
// #define USBACK  { MSerial.serWrite(0x6); }

// Structure to store the result of get_command_usb()
// XXX merge get_command_usb* functions into this
class UsbCommand : public Protothread {
    public:

        // Result of waitForSerial()
        typedef enum {
            NothinAvailable,       // 
            CharsAvailable,       // 
            SerTimeout
            } SerAvailableState;

        // Timestamp of start character of a usbserial command, to
        // detect timeout's.
        unsigned long startTS;

        // Number of characters read for current command
        uint16_t serial_count;

        // Command serial number
        // unsigned long serialNumber;
        uint8_t serialNumber;

        // Filepos where we store the current command, for
        // rollback.
        uint32_t writeBlockNumber;
        uint16_t writePos;

        uint8_t commandByte;

        // Computed checksum
        uint16_t checksum;

        // Number of characters we have to read
        uint32_t payloadLength;

        uint8_t lenByte1;
        uint8_t lenByte2;

        UsbCommand() {
            init();
        }

        void init() {
            serial_count = 0;
            serialNumber = 0;
        }

        void reset() {
            serial_count = 0;

            // Drain usbserial buffers for 50 ms
            unsigned long drainEnd = millis() + 50;
            while (millis() < drainEnd) {
                if( MSerial.available() ) {
                    MSerial.serReadNoCheck();
                }
            }
        }

        void rxError(uint8_t e) {

            /*
            SERIAL_ERROR_START;
            SERIAL_PROTOCOLPGM("RX err(");
            SERIAL_ERROR(e);
            SERIAL_PROTOCOLPGM(") Last Line:");
            // SERIAL_ERRORLN(serialNumber);
            MSerial.println(serialNumber, DEC);
            */

            txBuffer.sendSimpleResponse(RespRXError, serialNumber, e);
            reset();
            // #if defined(ExtendedStats)
                // errorFlags |= EFRXError;
            // #endif
        }

        void crcError() {

            /*
            SERIAL_ERROR_START;
            SERIAL_PROTOCOLPGM("CRC err Last Line:");
            // SERIAL_ERRORLN(serialNumber);
            MSerial.println(serialNumber, DEC);
            */

            txBuffer.sendSimpleResponse(RespRXCRCError, serialNumber);
            reset();
            // #if defined(ExtendedStats)
                // errorFlags |= EFRXError;
            // #endif
        }

        void serialNumberError() {

            /*
            SERIAL_ERROR_START;
            SERIAL_PROTOCOLPGM("SerialNumber err Last Line:");
            // SERIAL_ERRORLN(serialNumber);
            MSerial.println(serialNumber, DEC);
            */

            txBuffer.sendSimpleResponse(RespSerNumberError, serialNumber);
            reset();
            // #if defined(ExtendedStats)
                // errorFlags |= EFRXError;
            // #endif
        }

        //
        // Check if serial chars are available.
        //
        FWINLINE SerAvailableState waitForSerial(uint8_t nChars) {

            //
            // If a serial char gets lost, we have a deadlock situation: Firmware waits
            // for more characters to arrive and host application waits for
            // the acknowledge of the last sent command.
            //

            unsigned long ts = millis();

            if (MSerial.available() >= nChars) {
                startTS = ts; // reset timeout 
                return CharsAvailable;
            }

            if ((ts - startTS) > 2500) {
                /*
                SERIAL_ERROR_START;
                SERIAL_PROTOCOLPGM("RX Timeout Last Line:");
                MSerial.println(serialNumber, DEC);
                */

                txBuffer.sendSimpleResponse(RespRXCRCError, serialNumber);
                reset();
                return SerTimeout;
            }

            return NothinAvailable;
        }


        bool Run() {

            PT_BEGIN();

            // Check usart status bits, restart on error
            uint8_t e, c;

            if (e = MSerial.getError()) {

                rxError(e);
                PT_RESTART();   // does a return
            }

            uint16_t i;
            SerAvailableState av;

            // Read startbyte
            PT_WAIT_UNTIL( (i = MSerial.available()) );
            
            while ( i ) {
                c = MSerial.serReadNoCheck();
                if (c == SOH) {
                    break;
                }
                i--;
            }

            if (c != SOH) {
                PT_RESTART();   // does a return
            }

            startTS = millis();

            serial_count = 0;
            checksum = 0;
            checksum = _crc_xmodem_update(checksum, SOH);

            // Read packet number
            PT_WAIT_WHILE( (av = waitForSerial(6)) == NothinAvailable );
            if (av == SerTimeout)
                PT_RESTART();

            // Packet serial number
            c = MSerial.serReadNoCheck();
            checksum = _crc_xmodem_update(checksum, c);

            // SERIAL_ECHO("serail: ");
            // SERIAL_PROTOCOLLN((uint16_t)c);

            // Read command byte
            commandByte = MSerial.serReadNoCheck();
            checksum = _crc_xmodem_update(checksum, commandByte);

            // SERIAL_ECHO("command: ");
            // SERIAL_PROTOCOLLN((uint16_t)commandByte);

            if (commandByte < 128) {
                //
                // Buffered command
                //

                // Get current write pos for possible data rollback
                writeBlockNumber = swapDev.getWriteBlockNumber();
                writePos = swapDev.getWritePos();

                if (c != serialNumber) {

                    //
                    // xxx rollback not needed here, we have not added any
                    // data yet.
                    //
                    // Roll back data store
                    // swapDev.setWritePos(writeBlockNumber, writePos);

                    serialNumberError();
                    PT_RESTART();   // does a return
                }

                // swapDev.addByte(commandByte);
                // PT_WAIT_WHILE( swapDev.isBusyWriting() );

                // Read payload length, 2 or 4 bytes
                lenByte1 = MSerial.serReadNoCheck();
                checksum = _crc_xmodem_update(checksum, lenByte1);
                payloadLength = lenByte1;

                // swapDev.addByte(c);
                // PT_WAIT_WHILE( swapDev.isBusyWriting() );

                lenByte2 = MSerial.serReadNoCheck();
                checksum = _crc_xmodem_update(checksum, lenByte2);
                payloadLength |= (lenByte2 << 8);

                // swapDev.addByte(c);
                // PT_WAIT_WHILE( swapDev.isBusyWriting() );

                if (commandByte != CmdBlock) {

                    swapDev.addByte(commandByte);
                    PT_WAIT_WHILE( swapDev.isBusyWriting() );

                    swapDev.addByte(lenByte1);
                    PT_WAIT_WHILE( swapDev.isBusyWriting() );

                    swapDev.addByte(lenByte2);
                    PT_WAIT_WHILE( swapDev.isBusyWriting() );

                    c = MSerial.serReadNoCheck();
                    checksum = _crc_xmodem_update(checksum, c);
                    payloadLength |= (c << 16);
                    swapDev.addByte(c);
                    PT_WAIT_WHILE( swapDev.isBusyWriting() );

                    c = MSerial.serReadNoCheck();
                    checksum = _crc_xmodem_update(checksum, c);
                    payloadLength |= (c << 24);
                    swapDev.addByte(c);
                    PT_WAIT_WHILE( swapDev.isBusyWriting() );

                    payloadLength = STD min(payloadLength, (uint32_t)256);
                }

                // SERIAL_ECHO("commandlen: ");
                // SERIAL_PROTOCOLLN(commandLength);
                // printf("reading %d (+9/+7) bytes\n", payloadLength);

                while (serial_count < payloadLength) {

                    massert (MSerial.getError() == 0);

                    PT_WAIT_WHILE( (av = waitForSerial(1)) == NothinAvailable );
                    if (av == SerTimeout) {
                        swapDev.setWritePos(writeBlockNumber, writePos);
                        PT_RESTART();
                    }

                    c = MSerial.serReadNoCheck();
                    checksum = _crc_xmodem_update(checksum, c);

                    swapDev.addByte(c);
                    PT_WAIT_WHILE( swapDev.isBusyWriting() );

                    serial_count++;
                }

                // Read checksum
                PT_WAIT_WHILE( (av = waitForSerial(2)) == NothinAvailable );
                if (av == SerTimeout) {
                    swapDev.setWritePos(writeBlockNumber, writePos);
                    PT_RESTART();
                }

                i = MSerial.serReadNoCheck();
                i += MSerial.serReadNoCheck() << 8;

                // printf("Read %d (+9) bytes, checksum: 0x%x, computed: 0x%x\n", serial_count, i,  checksum);

                // Check checksum
                if (i != checksum) {

                    // Roll back data store
                    swapDev.setWritePos(writeBlockNumber, writePos);

                    crcError();
                    PT_RESTART();   // does a return
                }

                serialNumber++;
                // USBACK;
                txBuffer.sendACK();
            }
            else {

                // 
                // Direct command
                // 
                if (commandByte == CmdResetLineNr) {
                    serialNumber = 0;
                }
                else {
                    if (c != serialNumber) {

                        serialNumberError();
                        PT_RESTART();   // does a return
                    }
                }

                // Read payload length, 4 bytes
                MSerial.peekChecksum(&checksum, 4);
                payloadLength = MSerial.serReadUInt32();

                // SERIAL_ECHO("commandlen: ");
                // SERIAL_PROTOCOLLN(payloadLength);

                // Wait for payload and two checksum bytes
                PT_WAIT_WHILE( (av = waitForSerial(payloadLength+2)) == NothinAvailable );
                if (av == SerTimeout)
                    PT_RESTART();

                massert (MSerial.getError() == 0);

                MSerial.peekChecksum(&checksum, payloadLength);

                i = MSerial.peekN(payloadLength++);
                i |= MSerial.peekN(payloadLength) << 8;

                // printf("checksum: 0x%x, computed: 0x%x\n", i,  checksum);

                // Check checksum
                if (i != checksum) {

                    crcError();
                    PT_RESTART();   // does a return
                }

                serialNumber++;

                // USBACK;

                // Handle direct command
                switch (commandByte) {
                    //
                    // Simple ack commands, just one byte, no payload
                    //
                    case CmdResetLineNr:
                        txBuffer.sendACK();
                        break;
                    case CmdEepromFactory: {
                        EepromSettings es;
                        defaultEepromSettings(es);
                        txBuffer.sendACK();
                        }
                        break;
                    case CmdDisableSteppers:
                        printer.cmdDisableSteppers();
                        txBuffer.sendACK();
                        break;
                    case CmdPrinterInit:
                        printer.printerInit();
                        txBuffer.sendACK();
                        break;
                    case CmdMove: // move
                        printer.cmdMove((Printer::MoveType)MSerial.serReadNoCheck());
                        txBuffer.sendACK();
                        break;
                    case CmdEOT: // EOT
                        if (swapDev.getWritePos()) {
                            // Save last partial block
                            // xxx check busy here
                            swapDev.writeBlock();
                        }
                        printer.cmdEot();
                        txBuffer.sendACK();
                        break;
                    case CmdSetHomePos:
                        //
                        // Following call does NOT work - oder of argument evaluation
                        // is not unspecified:
                        //
                        /*
                        printer.setHomePos(
                                MSerial.serReadInt32(),
                                MSerial.serReadInt32(),
                                MSerial.serReadInt32(),
                                MSerial.serReadInt32(),
                                MSerial.serReadInt32());
                        */
                        {
                            int32_t x = MSerial.serReadInt32();
                            int32_t y = MSerial.serReadInt32();
                            int32_t z = MSerial.serReadInt32();
                            printer.setHomePos(x, y, z);
                            txBuffer.sendACK();
                        }
                        break;
                    case CmdSetTargetTemp:
                        {
                            uint8_t heater = MSerial.serReadNoCheck();
                            uint16_t temp = MSerial.serReadUInt16();
                            printer.cmdSetTargetTemp(heater, temp);
                            txBuffer.sendACK();
                        }
                        break;
                    case CmdStopMove:
                        printer.cmdStopMove();
                        txBuffer.sendACK();
                        break;
                    case CmdFanSpeed:
                        printer.cmdFanSpeed(MSerial.serReadNoCheck());
                        txBuffer.sendACK();
                        break;
                    //
                    // Commands with response payload
                    //
                    case CmdGetStatus:
                        printer.cmdGetStatus();
                        break;
                    case CmdWriteEepromFloat: {
                        uint8_t len = MSerial.serReadNoCheck();
                        char name[64];
                        for (c=0; c<64 && c<len; c++)
                            name[c] = MSerial.serReadNoCheck();
                        txBuffer.sendSimpleResponse(
                            commandByte,
                            writeEepromFloat(name, len, MSerial.serReadFloat()));
                        }
                        break;
                    case CmdGetEepromVersion:
                        getEepromVersion();
                        break;
                    case CmdGetEepromSettings:
                        dumpEepromSettings();
                        break;
                    case CmdGetHomed:
                        printer.cmdGetHomed();
                        break;
                    case CmdGetEndstops:
                        printer.cmdGetEndstops();
                        break;
                    case CmdGetPos:
                        printer.cmdGetPos();
                        break;
                    case CmdGetTargetTemps:
                        printer.cmdGetTargetTemps();
                        break;
                    case CmdGetCurrentTemps:
                        printer.cmdGetCurrentTemps();
                        break;

#if 0
// Currently not used:
                    case CmdGetState:
                        printer.cmdGetState();
                        break;
                    case CmdDisableStepperIsr:
                        printer.cmdDisableStepperIsr();
                        break;
#endif

#if 0
#if defined(PIDAutoTune)
                    case CmdSetHeaterY:
// xxx order of evaluation
                        tempControl.setHeaterY(MSerial.serReadNoCheck(), MSerial.serReadNoCheck());
                        SERIAL_PROTOCOLLNPGM(MSG_OK);
                        break;
#endif
#if defined(ADNSFS) || defined(BournsEMS22AFS)
                    case CmdGetFilSensor:
                        printer.cmdGetFilSensor();
                        break;
#endif
                    case CmdGetTempTable:
                        printer.cmdGetTempTable();
                        break;
#endif

#if defined(DDSim)
                    case CmdExit:
                        printf("CmdExit received, exiting...\n");
                        exit(0);
                        break;
#endif
                    default:
                        txBuffer.sendSimpleResponse(RespUnknownCommand, commandByte);
                }

                MSerial.flush();
            }

            PT_RESTART();
            PT_END();
        }
};

static UsbCommand usbCommand;

FWINLINE void loop() {

    unsigned long ts = millis();
    // Timer for slow running tasks (temp, encoder)
    static unsigned long timer10mS = millis();
    // static unsigned long timer50mS = millis();
    static unsigned long timer100mS = millis();

    // if (printer.moveType == Printer::MoveTypeNormal) {
    if (printer.printerState == Printer::StateStart) {

        if (printer.bufferLow == -1) {
            if  (stepBuffer.byteSize() > 40)
                printer.bufferLow = 0;
        }
        else {
            if ((stepBuffer.byteSize() < 40) && 
                (printer.bufferLow < 255) &&
                swapDev.available()) {

                printer.bufferLow++;
                /*
                SERIAL_ECHO("BUFLOW:");
                SERIAL_PROTOCOL(printer.bufferLow);
                SERIAL_PROTOCOL(" SWP:");
                SERIAL_PROTOCOL(swapDev.available());
                SERIAL_PROTOCOL(" SDR:");
                SERIAL_PROTOCOLLN(sDReader.available());
                */
            }
        }
    }

    if ((ts - timer10mS) > 10) { // Every 10 mS

        // Check hardware and software endstops:
        if (printer.moveType == Printer::MoveTypeNormal) {

            if (X_ENDSTOP_PRESSED || Y_ENDSTOP_PRESSED || Z_ENDSTOP_PRESSED) {

                // SERIAL_ECHOPGM("POS: ");
                // SERIAL_ECHO(current_pos_steps[X_AXIS]);
                // SERIAL_ECHO(", ");
                // SERIAL_ECHO(current_pos_steps[Y_AXIS]);
                // SERIAL_ECHO(", ");
                // SERIAL_ECHOLN(current_pos_steps[Z_AXIS]);
                txBuffer.sendResponseStart(RespKilled, 1 + 3*(sizeof(int32_t)+1));
                txBuffer.pushCharChecksum(RespHardwareEndstop);
                txBuffer.sendResponseValue((uint8_t*)current_pos_steps, 3*sizeof(int32_t));
                txBuffer.pushCharChecksum(X_ENDSTOP_PRESSED);
                txBuffer.pushCharChecksum(Y_ENDSTOP_PRESSED);
                txBuffer.pushCharChecksum(Z_ENDSTOP_PRESSED);
                txBuffer.sendResponseEnd();
                kill();
            }
            else if (X_SW_ENDSTOP_PRESSED || Y_SW_ENDSTOP_PRESSED || Z_SW_ENDSTOP_PRESSED) {

                // SERIAL_ECHOPGM("POS: ");
                // SERIAL_ECHO(current_pos_steps[X_AXIS]);
                // SERIAL_ECHO(", ");
                // SERIAL_ECHO(current_pos_steps[Y_AXIS]);
                // SERIAL_ECHO(", ");
                // SERIAL_ECHOLN(current_pos_steps[Z_AXIS]);
                txBuffer.sendResponseStart(RespKilled, 1 + 3*(sizeof(int32_t)+1));
                txBuffer.pushCharChecksum(RespSoftwareEndstop);
                txBuffer.sendResponseValue((uint8_t*)current_pos_steps, 3*sizeof(int32_t));
                txBuffer.pushCharChecksum(X_SW_ENDSTOP_PRESSED);
                txBuffer.pushCharChecksum(Y_SW_ENDSTOP_PRESSED);
                txBuffer.pushCharChecksum(Z_SW_ENDSTOP_PRESSED);
                txBuffer.sendResponseEnd();
                kill();
            }
        }

        //
        // Measure temperatures every 10ms (build mean value by OVERSAMPLENR)
        //
        tempControl.Run();

        if ((ts - timer100mS) > 100) { // Every 100 mS

            // lcd_lib_buttons_update_interrupt();
            // lcd_update();

            //
            // Control heater 
            //
            tempControl.heater();

            printer.checkMoveFinished();

            /*            
            uint16_t sbs = stepBuffer.byteSize();
            if (sbs && sbs<512) {

                SERIAL_ECHO("buflen: ");
                SERIAL_PROTOCOLLN(sbs);
            }
            */

#if defined(ADNSFS) || defined(BournsEMS22AFS)
            // Read filament sensor
            filamentSensor.run();
#endif

            timer100mS = ts;
        }
        timer10mS = ts;
    }

    // Read usb commands
    usbCommand.Run();

    // Write usb/serial output
    txBuffer.Run();

    /*
    if (printer.moveType == Printer::MoveTypeNone) {
        // Handle swapfile writing
        swapDev.Run();
    }
    else {
*/
        // If printing, then read steps from the sd buffer and push it to
        // the print buffer.
        fillBufferTask.Run();
        swapDev.Run();
#if 0
        /*
        // Handle swapfile writing
        if ((swapDev.available() < 10240) || (sDReader.available() > 400))
        */
        if (swapDev.isBusyWriting() || (swapDev.available() < 10240)) {
            swapDev.Run();
        }
        else if (swapDev.isBusyWriting() && (stepBuffer.timeSum >= 500000)) {
            swapDev.Run();
            SERIAL_ECHO("BT:");
            SERIAL_PROTOCOL(stepBuffer.timeSum);
            SERIAL_PROTOCOL(" SWP:");
            SERIAL_PROTOCOL(swapDev.available());
            SERIAL_PROTOCOL(" SDR:");
            SERIAL_PROTOCOL(sDReader.available());
            SERIAL_PROTOCOL(" SB:");
            SERIAL_PROTOCOLLN((uint16_t)stepBuffer.byteSize());
        }
        /*
        else {
            SERIAL_ECHO("NW:");
            SERIAL_PROTOCOL(" SWP:");
            SERIAL_PROTOCOL(swapDev.available());
            SERIAL_PROTOCOL(" SDR:");
            SERIAL_PROTOCOLLN(sDReader.available());
        }
        */
#endif
        /*
    }
*/

    // Check swap dev error
    if (swapDev.errorCode()) {
        txBuffer.sendSimpleResponse(RespKilled, RespSDError);
        kill();
    }
}



#if ! defined(DDSim)
int main(void) {

    init();

    setup();

    for (;;) {
        loop();
    }

    return 0;
}
#endif





