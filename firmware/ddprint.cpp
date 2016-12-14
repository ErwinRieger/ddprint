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
#include "swapdev.h"
#include "eepromSettings.h"
#include "filsensor.h"
#include "ddserial.h"
#include "ddcommands.h"
#include "ddlcd.h"

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

// uint8_t Stopped = 0;

// Timestamp last call of loop()
unsigned long loopTS;

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


/////////////////////////////////////////////////////////////////////////////////////
#if defined(USEExtrusionRateTable)

uint16_t extrusionLimitBaseTemp = 190;

//
// Limit extrusion rate by the hotend temperature. This is the initial table
// with no real limitations. The pertinent values are downloaded by the host
// software before print.
//
uint16_t tempExtrusionRateTable[NExtrusionLimit] = {
    /* temp: 190, max extrusion: 4.80 mm³/s, steps/s: 281, steprate: 3553 us, timervalue: */ 7107,
    /* temp: 192, max extrusion: 5.80 mm³/s, steps/s: 340, steprate: 2941 us, timervalue: */ 5882,
    /* temp: 194, max extrusion: 6.80 mm³/s, steps/s: 398, steprate: 2508 us, timervalue: */ 5017,
    /* temp: 196, max extrusion: 7.80 mm³/s, steps/s: 457, steprate: 2187 us, timervalue: */ 4374,
    /* temp: 198, max extrusion: 8.80 mm³/s, steps/s: 515, steprate: 1938 us, timervalue: */ 3876,
    /* temp: 200, max extrusion: 9.80 mm³/s, steps/s: 574, steprate: 1740 us, timervalue: */ 3481,
    /* temp: 202, max extrusion: 10.80 mm³/s, steps/s: 633, steprate: 1579 us, timervalue: */ 3159,
    /* temp: 204, max extrusion: 11.80 mm³/s, steps/s: 691, steprate: 1445 us, timervalue: */ 2891,
    /* temp: 206, max extrusion: 12.80 mm³/s, steps/s: 750, steprate: 1332 us, timervalue: */ 2665,
    /* temp: 208, max extrusion: 13.80 mm³/s, steps/s: 808, steprate: 1236 us, timervalue: */ 2472,
    /* temp: 210, max extrusion: 14.80 mm³/s, steps/s: 867, steprate: 1152 us, timervalue: */ 2305,
    /* temp: 212, max extrusion: 15.80 mm³/s, steps/s: 926, steprate: 1079 us, timervalue: */ 2159,
    /* temp: 214, max extrusion: 16.80 mm³/s, steps/s: 984, steprate: 1015 us, timervalue: */ 2030,
    /* temp: 216, max extrusion: 17.80 mm³/s, steps/s: 1043, steprate: 958 us, timervalue: */ 1916,
    /* temp: 218, max extrusion: 18.80 mm³/s, steps/s: 1102, steprate: 907 us, timervalue: */ 1814,
    /* temp: 220, max extrusion: 19.80 mm³/s, steps/s: 1160, steprate: 861 us, timervalue: */ 1723,
    /* temp: 222, max extrusion: 20.80 mm³/s, steps/s: 1219, steprate: 820 us, timervalue: */ 1640,
    /* temp: 224, max extrusion: 21.80 mm³/s, steps/s: 1277, steprate: 782 us, timervalue: */ 1565,
    /* temp: 226, max extrusion: 22.80 mm³/s, steps/s: 1336, steprate: 748 us, timervalue: */ 1496,
    /* temp: 228, max extrusion: 23.80 mm³/s, steps/s: 1395, steprate: 716 us, timervalue: */ 1433,
    /* temp: 230, max extrusion: 24.80 mm³/s, steps/s: 1453, steprate: 687 us, timervalue: */ 1375,
    /* temp: 232, max extrusion: 25.80 mm³/s, steps/s: 1512, steprate: 661 us, timervalue: */ 1322,
    /* temp: 234, max extrusion: 26.80 mm³/s, steps/s: 1571, steprate: 636 us, timervalue: */ 1273,
    /* temp: 236, max extrusion: 27.80 mm³/s, steps/s: 1629, steprate: 613 us, timervalue: */ 1227,
    /* temp: 238, max extrusion: 28.80 mm³/s, steps/s: 1688, steprate: 592 us, timervalue: */ 1184,
    /* temp: 240, max extrusion: 29.80 mm³/s, steps/s: 1746, steprate: 572 us, timervalue: */ 1144,
    /* temp: 242, max extrusion: 30.80 mm³/s, steps/s: 1805, steprate: 553 us, timervalue: */ 1107,
    /* temp: 244, max extrusion: 31.80 mm³/s, steps/s: 1864, steprate: 536 us, timervalue: */ 1072,
    /* temp: 246, max extrusion: 32.80 mm³/s, steps/s: 1922, steprate: 520 us, timervalue: */ 1040,
    /* temp: 248, max extrusion: 33.80 mm³/s, steps/s: 1981, steprate: 504 us, timervalue: */ 1009,
    /* temp: 250, max extrusion: 34.80 mm³/s, steps/s: 2040, steprate: 490 us, timervalue: */ 980,
    /* temp: 252, max extrusion: 35.80 mm³/s, steps/s: 2098, steprate: 476 us, timervalue: */ 953,
    /* temp: 254, max extrusion: 36.80 mm³/s, steps/s: 2157, steprate: 463 us, timervalue: */ 927,
    /* temp: 256, max extrusion: 37.80 mm³/s, steps/s: 2215, steprate: 451 us, timervalue: */ 902,
    /* temp: 258, max extrusion: 38.80 mm³/s, steps/s: 2274, steprate: 439 us, timervalue: */ 879,
    /* temp: 260, max extrusion: 39.80 mm³/s, steps/s: 2333, steprate: 428 us, timervalue: */ 857,
    /* temp: 262, max extrusion: 40.80 mm³/s, steps/s: 2391, steprate: 418 us, timervalue: */ 836,
    /* temp: 264, max extrusion: 41.80 mm³/s, steps/s: 2450, steprate: 408 us, timervalue: */ 816,
    /* temp: 266, max extrusion: 42.80 mm³/s, steps/s: 2508, steprate: 398 us, timervalue: */ 797,
    /* temp: 268, max extrusion: 43.80 mm³/s, steps/s: 2567, steprate: 389 us, timervalue: */ 778,
};

#endif

/////////////////////////////////////////////////////////////////////////////////////


void printDebugInfo();

// xxx move to printer class?
void kill() {

    cli(); // Stop interrupts
    disable_heater();
    printer.disableSteppers();

#if defined(PS_ON_PIN) && PS_ON_PIN > -1
    pinMode(PS_ON_PIN,INPUT);
#endif

    // printDebugInfo();

    while(1) {
        // Wait for reset
        txBuffer.Run();
    }
}

void mAssert(uint16_t line, char* file) {

    LCDMSGKILL(RespAssertion, line, file);

    txBuffer.flush();
    txBuffer.sendResponseStart(RespKilled);
    txBuffer.sendResponseUint8(RespAssertion);
    txBuffer.sendResponseValue(line);
    txBuffer.sendResponseString(file, strlen(file));
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

    #if defined(REPRAP_DISCOUNT_SMART_CONTROLLER)
    lcd.begin(20, 4);
    lcd.print("OK");
    #endif
    

    if (! swapDev.swapInit()) {
        LCDMSGKILL(RespSDInit, "swapDev.swapInit()", "");
        txBuffer.sendSimpleResponse(RespKilled, RespSDInit);
        kill();
    }

#if defined(ADNSFS)
    filamentSensor.reset();
#endif

#if defined(HASFILAMENTSENSOR)
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

static SDReader sDReader;

#if defined(USEExtrusionRateTable)
    #define MAXTEMPSPEED maxTempSpeed
#else
    #define MAXTEMPSPEED filamentSensor.maxTempSpeed
#endif

bool FillBufferTask::Run() {

            uint8_t i, cmd;

            int32_t d;
            int32_t d1;
            int32_t d2;

#if defined(USEExtrusionRateTable)
            uint16_t leadFactor;
            // int16_t curTempIndex;
#endif

            PT_BEGIN();

            // SERIAL_ECHOPGM(" readSwap, sdr avail: ");
            // SERIAL_ECHO(sDReader.available());
            // SERIAL_ECHOPGM(", swapavail: ");
            // SERIAL_ECHOLN(swapDev.available());

            #if 0
            # Move segment data, new with bresenham in firmware:
            #   * Header data:
            #       + 8 flag bits
            #       + index lead axis, 8 bits
            #       + array of absolute steps, 5 * 16 bits
            #       + number of accel steps, naccel, 16 bits
            #       + constant linear timer value, 16 bits
            #       + number of decel steps, ndccel, 16 bits
            #
            #   * accel steps: naccel*(timer value(16bits)) or 16bits + (naccel-1)*8bits
            #
            #   * decel steps: ndecel*(timer value(16bits)) or 16bits + (naccel-1)*8bits
            #
            #endif

            sDReader.setBytesToRead1();
            PT_WAIT_THREAD(sDReader);
            cmd = *sDReader.readData;

            switch (cmd) {

                case CmdG1:
                    goto HandleCmdG1;

                case CmdG1Raw:
                    goto HandleCmdG1Raw;

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

                    LCDMSGKILL(RespUnknownBCommand, cmd, "");
                    txBuffer.sendResponseStart(RespKilled);
                    txBuffer.sendResponseUint8(RespUnknownBCommand);
                    txBuffer.sendResponseUint8(cmd);
                    txBuffer.sendResponseEnd();
                    kill();
            }

            HandleCmdG1:

                // Read flag byte and stepper direction bits
                sDReader.setBytesToRead1();
                PT_WAIT_THREAD(sDReader);

                flags = *sDReader.readData;
                if (flags & 0x80)
                    // Change stepper direction(s)
                    sd.dirBits = flags & 0x9F;
// xxx printf("flags: 0x%x, dirbits: 0x%x\n", flags, dirBits);

                // SERIAL_ECHOLNPGM("C1");

                cmdSync = true;

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

#if defined(USEExtrusionRateTable)
                leadFactor = FromBuf(uint16_t, sDReader.readData);

                if (leadFactor) {

                    int16_t curTempIndex = (current_temperature[0] - extrusionLimitBaseTemp) / 2;

                    if (curTempIndex < 0) {

                        maxTempSpeed = ((uint32_t)tempExtrusionRateTable[0] * 1000) / leadFactor;
                    }
                    else if (curTempIndex >= NExtrusionLimit) {

                        maxTempSpeed = ((uint32_t)tempExtrusionRateTable[NExtrusionLimit-1] * 1000) / leadFactor;
                    }
                    else {

                        maxTempSpeed = ((uint32_t)tempExtrusionRateTable[curTempIndex] * 1000) / leadFactor;
                    }
                }
                else {

                    maxTempSpeed = 0;
                }
#endif
    
                //////////////////////////////////////////////////////

                sDReader.setBytesToRead2();
                PT_WAIT_THREAD(sDReader);

                tLin = STD max ( FromBuf(uint16_t, sDReader.readData), MAXTEMPSPEED);

                // nDecel = sDReader.readPayloadUInt16();
                sDReader.setBytesToRead2();
                PT_WAIT_THREAD(sDReader);
                nDecel = FromBuf(uint16_t, sDReader.readData);
                // SERIAL_ECHOLN(nDecel);

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

                if (nAccel) {

                    //
                    // Acceleration, get first timer value
                    //
                    sDReader.setBytesToRead2();
                    PT_WAIT_THREAD(sDReader);
                    lastTimer = FromBuf(uint16_t, sDReader.readData);
                    sd.timer = STD max ( lastTimer, MAXTEMPSPEED );

                    computeStepBits();
                    PT_WAIT_WHILE(stepBuffer.full());
                    stepBuffer.push(sd);

                    sd.dirBits &= ~0x80; // clear set-direction bit

                    if (flags & AccelByteFlag) {

                        // Timer value as 8bit difference
                        for (step = 1; step < nAccel; step++) {

                            sDReader.setBytesToRead1();
                            PT_WAIT_THREAD(sDReader);
                            lastTimer -= FromBuf(uint8_t, sDReader.readData);
                            sd.timer = STD max ( lastTimer,  MAXTEMPSPEED );

                            computeStepBits();
                            PT_WAIT_WHILE(stepBuffer.full());
                            stepBuffer.push(sd);
                        }
                    }
                    else {

                        // Timer value as 16bit absolute value
                        for (step = 1; step < nAccel; step++) {

                            sDReader.setBytesToRead2();
                            PT_WAIT_THREAD(sDReader);
                            sd.timer = STD max ( FromBuf(uint16_t, sDReader.readData), MAXTEMPSPEED );

                            computeStepBits();
                            PT_WAIT_WHILE(stepBuffer.full());
                            stepBuffer.push(sd);
                        }
                    }
                }

                //
                // Constant phase
                //
                step = deltaLead - (nAccel+nDecel);
                if (step) {

                    sd.timer = tLin;

                    computeStepBits();
                    PT_WAIT_WHILE(stepBuffer.full());
                    stepBuffer.push(sd);

                    sd.dirBits &= ~0x80; // clear set-direction bit

                    for (; step > 1; step--) {

                        computeStepBits();
                        PT_WAIT_WHILE(stepBuffer.full());
                        stepBuffer.push(sd);
                    }
                }

                if (nDecel) {

                    //
                    // Deceleration, get first timer value
                    //
                    sDReader.setBytesToRead2();
                    PT_WAIT_THREAD(sDReader);
                    lastTimer = FromBuf(uint16_t, sDReader.readData);
                    sd.timer = STD max ( lastTimer, MAXTEMPSPEED );

                    computeStepBits();
                    PT_WAIT_WHILE(stepBuffer.full());
                    stepBuffer.push(sd);

                    sd.dirBits &= ~0x80; // clear set-direction bit

                    if (flags & DecelByteFlag) {

                        // Timer value as 8bit difference
                        for (step = 1; step < nDecel; step++) {

                            sDReader.setBytesToRead1();
                            PT_WAIT_THREAD(sDReader);
                            lastTimer += FromBuf(uint8_t, sDReader.readData);
                            sd.timer = STD max ( lastTimer,  MAXTEMPSPEED );

                            computeStepBits();
                            PT_WAIT_WHILE(stepBuffer.full());
                            stepBuffer.push(sd);
                        }
                    }
                    else {

                        // Timer value as 16bit absolute value
                        for (step = 1; step < nDecel; step++) {

                            sDReader.setBytesToRead2();
                            PT_WAIT_THREAD(sDReader);
                            sd.timer = STD max ( FromBuf(uint16_t, sDReader.readData), MAXTEMPSPEED );

                            computeStepBits();
                            PT_WAIT_WHILE(stepBuffer.full());
                            stepBuffer.push(sd);
                        }
                    }
                }

                PT_RESTART();

            HandleCmdG1Raw:

                // Read flag byte and stepper direction bits
                sDReader.setBytesToRead1();
                PT_WAIT_THREAD(sDReader);

                flags = *sDReader.readData;
                if (flags & 0x80)
                    // Change stepper direction(s)
                    sd.dirBits = flags & 0x9F;

                // SERIAL_ECHOLNPGM("C1");

                // ???
                // cmdSync = true;

                //
                // Read len of pulses array, 2 bytes short uint
                //
                sDReader.setBytesToRead2();
                PT_WAIT_THREAD(sDReader);
                nAccel = FromBuf(uint16_t, sDReader.readData);
                // SERIAL_ECHOLN(nAccel);

                //////////////////////////////////////////////////////
                sDReader.setBytesToRead2();
                PT_WAIT_THREAD(sDReader);

#if defined(USEExtrusionRateTable)
                leadFactor = FromBuf(uint16_t, sDReader.readData);

                if (leadFactor) {

                    int16_t curTempIndex = (current_temperature[0] - extrusionLimitBaseTemp) / 2;

                    if (curTempIndex < 0) {

                        maxTempSpeed = ((uint32_t)tempExtrusionRateTable[0] * 1000) / leadFactor;
                    }
                    else if (curTempIndex >= NExtrusionLimit) {

                        maxTempSpeed = ((uint32_t)tempExtrusionRateTable[NExtrusionLimit-1] * 1000) / leadFactor;
                    }
                    else {

                        maxTempSpeed = ((uint32_t)tempExtrusionRateTable[curTempIndex] * 1000) / leadFactor;
                    }
                }
                else {

                    maxTempSpeed = 0;
                }
#endif
    
                //////////////////////////////////////////////////////

                sDReader.setBytesToRead2();
                PT_WAIT_THREAD(sDReader);
                lastTimer = FromBuf(uint16_t, sDReader.readData);
                sd.timer = STD max ( lastTimer, MAXTEMPSPEED );

                sDReader.setBytesToRead1();
                PT_WAIT_THREAD(sDReader);
                sd.stepBits = *sDReader.readData;

                PT_WAIT_WHILE(stepBuffer.full());
                stepBuffer.push(sd);

                sd.dirBits &= ~0x80; // clear set-direction bit

                if (flags & RawByteFlag) {
                    //
                    // Read pulse array, first element is a 16bit timer value and 8bit stepper mask,
                    // then elements of 8bit timer delta and 8bit stepper mask
                    //
                    for (step=1; step < nAccel; step++) {

                        sDReader.setBytesToRead1();
                        PT_WAIT_THREAD(sDReader);
                        lastTimer += FromBuf(int8_t, sDReader.readData);
                        sd.timer = STD max ( lastTimer, MAXTEMPSPEED );

                        sDReader.setBytesToRead1();
                        PT_WAIT_THREAD(sDReader);
                        sd.stepBits = *sDReader.readData;

                        PT_WAIT_WHILE(stepBuffer.full());
                        stepBuffer.push(sd);
                    }
                }
                else {
                    //
                    // Read pulse array, elements of 16bit timer and 8bit stepper mask
                    //
                    for (step=1; step < nAccel; step++) {

                        sDReader.setBytesToRead2();
                        PT_WAIT_THREAD(sDReader);
                        sd.timer = STD max ( FromBuf(uint16_t, sDReader.readData), MAXTEMPSPEED );

                        sDReader.setBytesToRead1();
                        PT_WAIT_THREAD(sDReader);
                        sd.stepBits = *sDReader.readData;

                        PT_WAIT_WHILE(stepBuffer.full());
                        stepBuffer.push(sd);
                    }
                }

                PT_RESTART();

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

    massert(printerState <= StateInit);

    swapDev.reset();

//xxxxxxxxxxx
sDReader.flush();

    fillBufferTask.flush();

stepBuffer.flush();

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

#if defined(HASFILAMENTSENSOR)
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

    txBuffer.sendResponseStart(CmdGetTargetTemps);

    txBuffer.sendResponseUint8(target_temperature_bed);
    txBuffer.sendResponseValue(target_temperature[0]);
#if EXTRUDERS > 1
    txBuffer.sendResponseValue(target_temperature[1]);
#endif
    txBuffer.sendResponseEnd();
}

void Printer::cmdGetCurrentTemps() {

#if EXTRUDERS == 1
    txBuffer.sendResponseStart(CmdGetCurrentTemps);
#else
    txBuffer.sendResponseStart(CmdGetCurrentTemps);
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

#endif

void Printer::cmdGetHomed() {

    txBuffer.sendResponseStart(CmdGetHomed);
    txBuffer.sendResponseValue((uint8_t*)homed, sizeof(homed));
    txBuffer.sendResponseEnd();
}

void Printer::cmdGetEndstops() {

    txBuffer.sendResponseStart(CmdGetEndstops);

    txBuffer.sendResponseUint8(X_ENDSTOP_PRESSED);
    txBuffer.sendResponseValue((int32_t)current_pos_steps[X_AXIS]);

    txBuffer.sendResponseUint8(Y_ENDSTOP_PRESSED);
    txBuffer.sendResponseValue((int32_t)current_pos_steps[Y_AXIS]);

    txBuffer.sendResponseUint8(Z_ENDSTOP_PRESSED);
    txBuffer.sendResponseValue((int32_t)current_pos_steps[Z_AXIS]);

    txBuffer.sendResponseEnd();
}

void Printer::cmdGetPos() {

    txBuffer.sendResponseStart(CmdGetPos);
    txBuffer.sendResponseValue((uint8_t*)current_pos_steps, sizeof(current_pos_steps));
    txBuffer.sendResponseEnd();
}

void Printer::cmdGetDirBits() {

    uint8_t dirbits = st_get_direction<XMove>() | st_get_direction<YMove>() | st_get_direction<ZMove>() | st_get_direction<EMove>();

    txBuffer.sendResponseStart(CmdGetDirBits);
    txBuffer.sendResponseUint8(dirbits);
    txBuffer.sendResponseEnd();
}

// uint16_t waitCount = 0;
void Printer::cmdGetStatus() {

    txBuffer.sendResponseStart(CmdGetStatus);

    txBuffer.sendResponseUint8(printerState);
    txBuffer.sendResponseValue(current_temperature_bed);
    txBuffer.sendResponseValue(current_temperature[0]);
    txBuffer.sendResponseValue(swapDev.available());
    txBuffer.sendResponseValue(sDReader.available());
    txBuffer.sendResponseUint8(stepBuffer.byteSize());
    txBuffer.sendResponseValue((uint16_t)bufferLow);
    txBuffer.sendResponseValue(target_temperature[0]);

    // Flowrate sensor
#if defined(HASFILAMENTSENSOR)
    txBuffer.sendResponseInt16(filamentSensor.targetSpeed.value());
    txBuffer.sendResponseInt16(filamentSensor.actualSpeed.value());
#else
    txBuffer.sendResponseInt16(0);
    txBuffer.sendResponseInt16(0);
#endif

    txBuffer.sendResponseEnd();
}

#if defined(HASFILAMENTSENSOR)
void Printer::cmdGetFilSensor() {

    txBuffer.sendResponseStart(CmdGetFilSensor);
    txBuffer.sendResponseValue(filamentSensor.yPos);
    txBuffer.sendResponseEnd();
}
#endif

void Printer::cmdGetTempTable() {

    txBuffer.sendResponseStart(CmdGetTempTable);

    txBuffer.sendResponseValue(extrusionLimitBaseTemp);

    txBuffer.sendResponseUint8(NExtrusionLimit);

    for (uint8_t i=0; i<NExtrusionLimit; i++) {
        txBuffer.sendResponseValue(tempExtrusionRateTable[i]);
    }

    txBuffer.sendResponseEnd();
}

void Printer::cmdSetTempTable() {

    extrusionLimitBaseTemp = MSerial.readUInt16NoCheckCobs();

    uint8_t len = MSerial.readNoCheckCobs();
    if (len != NExtrusionLimit) {

        txBuffer.sendSimpleResponse(CmdSetTempTable, RespInvalidArgument);
        return;
    }

    for (uint8_t i=0; i<NExtrusionLimit; i++) {
        tempExtrusionRateTable[i] = MSerial.readUInt16NoCheckCobs();
    }

    txBuffer.sendSimpleResponse(CmdSetTempTable, RespOK);
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

        // Command serial number [1..255]
        uint8_t serialNumber;

        uint8_t commandByte;

        // Computed checksum
        uint16_t checksum;

        // Number of characters we have to read
        uint8_t payloadLength;

        UsbCommand() {
            serialNumber = 1;
        }

        void reset() {

            // Drain usbserial buffers for 50 ms
            unsigned long drainEnd = millis() + 50;
            while (millis() < drainEnd) {
                MSerial.flush0();
            }
        }

        void crcError() {

            txBuffer.sendSimpleResponse(RespRXCRCError, serialNumber);
            reset();
            // #if defined(ExtendedStats)
                // errorFlags |= EFRXError;
            // #endif
        }

        void serialNumberError() {

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
            // To prevent this we implement a timeout here.
            //

            unsigned long ts = millis();

            if (MSerial._available() >= nChars) {
                startTS = ts; // reset timeout 
                return CharsAvailable;
            }

            if ((ts - startTS) > 2500) {

                txBuffer.sendSimpleResponse(RespRXTimeoutError, serialNumber);
                reset();
                return SerTimeout;
            }

            return NothinAvailable;
        }


        bool Run() {

            PT_BEGIN();

            uint8_t e, c, flags, cs1, cs2;

            SerAvailableState av;

            // Read startbyte
            PT_WAIT_UNTIL( MSerial._available() );
            PT_WAIT_UNTIL( MSerial.readNoCheckNoCobs() == SOH );
            
            startTS = millis();

            checksum = 0;

            // Read packet number, command and payload length
            PT_WAIT_WHILE( (av = waitForSerial(3)) == NothinAvailable );
            if (av == SerTimeout)
                PT_RESTART();

            // Packet serial number
            c = MSerial.readNoCheckNoCobs();
            checksum = _crc_xmodem_update(checksum, c);

            // Read command byte
            commandByte = MSerial.readNoCheckNoCobs();
            checksum = _crc_xmodem_update(checksum, commandByte);

            if (commandByte < 128) {

                //
                // Buffered command
                //
                if (c != serialNumber) {

                    serialNumberError();
                    PT_RESTART();   // does a return
                }

                // Read payload length 1 byte
                payloadLength = MSerial.readNoCheckNoCobs();
                checksum = _crc_xmodem_update(checksum, payloadLength);
                payloadLength--;

                // Wait for payload, checksum flags and two checksum bytes
                PT_WAIT_WHILE( (av = waitForSerial(payloadLength+3)) == NothinAvailable );
                if (av == SerTimeout)
                    PT_RESTART();

                MSerial.peekChecksum(&checksum, payloadLength);

                flags = MSerial.peekN(payloadLength);
                cs1 = MSerial.peekN(payloadLength+1);
                cs2 = MSerial.peekN(payloadLength+2);

                if (! checkCrc(flags, cs1, cs2, checksum)) {
                    PT_RESTART();   // does a return
                }

                if (commandByte != CmdBlock) {

                    swapDev.addByte(commandByte);
                    PT_WAIT_WHILE( swapDev.isBusyWriting() );
                }

                // Tell RxBuffer that it's pointing to the beginning of a COBS block
                MSerial.cobsInit(payloadLength);

                while (MSerial.cobsAvailable()) {

                    c = MSerial.readNoCheckCobs();

                    swapDev.addByte(c);
                    PT_WAIT_WHILE( swapDev.isBusyWriting() );
                }

                // Successfully received command, increment command counter
                serialNumber++;
                if (serialNumber==0)
                    serialNumber = 1;

#if defined(HEAVYDEBUG)
                massert(MSerial._available() == 3);
#endif

                MSerial.flush0(); // clear rx buffer

                // USBACK;
                txBuffer.sendACK();
            }
            else {

                // 
                // Direct command
                // 
#if 0
                if (commandByte == CmdResetLineNr) {
                    serialNumber = 1;
                }
                else {
                    if (c != serialNumber) {

                        serialNumberError();
                        PT_RESTART();   // does a return
                    }
                }
#endif

                if ((c != serialNumber) && (commandByte != CmdResetLineNr)) {

                    serialNumberError();
                    PT_RESTART();   // does a return
                }

                // Read payload length 1 byte
                payloadLength = MSerial.readNoCheckNoCobs();
                checksum = _crc_xmodem_update(checksum, payloadLength);
                payloadLength--;

                // Wait for payload, checksum flags and two checksum bytes
                PT_WAIT_WHILE( (av = waitForSerial(payloadLength+3)) == NothinAvailable );
                if (av == SerTimeout)
                    PT_RESTART();

                MSerial.peekChecksum(&checksum, payloadLength);

                flags = MSerial.peekN(payloadLength);
                cs1 = MSerial.peekN(payloadLength+1);
                cs2 = MSerial.peekN(payloadLength+2);

                if (! checkCrc(flags, cs1, cs2, checksum))
                    PT_RESTART();   // does a return

                // Handle ResetLineNr command, set command counter
                if (commandByte == CmdResetLineNr) {
                    serialNumber = 1;
                }

                // Successfully received command, increment command counter
                serialNumber++;
                if (serialNumber==0)
                    serialNumber = 1;

                // Tell RxBuffer that it's pointing to the beginning of a COBS block
                MSerial.cobsInit(payloadLength);

#if defined(HEAVYDEBUG)
                uint8_t bytesLeft = 3;
#endif

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
                        printer.cmdMove((Printer::MoveType)MSerial.readNoCheckCobs());
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
                        // is unspecified:
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
                            int32_t x = MSerial.readInt32NoCheckCobs();
                            int32_t y = MSerial.readInt32NoCheckCobs();
                            int32_t z = MSerial.readInt32NoCheckCobs();

                            // consume two additional ints
                            MSerial.readInt32NoCheckCobs();
                            MSerial.readInt32NoCheckCobs();

                            printer.setHomePos(x, y, z);
                            txBuffer.sendACK();
                        }
                        break;
                    case CmdSetTargetTemp:
                        {
                            uint8_t heater = MSerial.readNoCheckCobs();
                            uint16_t temp = MSerial.readUInt16NoCheckCobs();
                            printer.cmdSetTargetTemp(heater, temp);
                            txBuffer.sendACK();
                        }
                        break;
                    case CmdStopMove:
                        printer.cmdStopMove();
                        txBuffer.sendACK();
                        break;
                    case CmdFanSpeed:
                        printer.cmdFanSpeed(MSerial.readNoCheckCobs());
                        txBuffer.sendACK();
                        break;
#if defined(PIDAutoTune)
                    case CmdSetHeaterY:
                        {
                            uint8_t heater = MSerial.readNoCheckCobs();
                            uint8_t pwmValue = MSerial.readNoCheckCobs();
                            tempControl.setHeaterY(heater, pwmValue);
                            txBuffer.sendACK();
                        }
                        break;
#endif
#if defined(HASFILAMENTSENSOR)
                    case CmdEnableFRLimit:
                        filamentSensor.enableFeedrateLimiter(MSerial.readNoCheckCobs());
                        txBuffer.sendACK();
                        break;
#endif


                    //
                    // Commands with response payload
                    //
                    case CmdGetDirBits:
                        printer.cmdGetDirBits();
                        break;
                    case CmdGetStatus:
                        printer.cmdGetStatus();
                        break;
                    case CmdWriteEepromFloat: {
                        uint8_t len = MSerial.readNoCheckCobs();
                        char name[64];
                        for (c=0; c<64 && c<len; c++) {
                            name[c] = MSerial.readNoCheckCobs();
                        }
                        float value = MSerial.readFloatNoCheckCobs();
                        txBuffer.sendSimpleResponse(commandByte, writeEepromFloat(name, len, value));
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
                    case CmdGetTempTable:
                        printer.cmdGetTempTable();
                        break;
                    case CmdSetTempTable:
                        printer.cmdSetTempTable();
                        break;
#if defined(HASFILAMENTSENSOR)
                    case CmdGetFilSensor:
                        printer.cmdGetFilSensor();
                        break;
#endif


#if 0
// Currently not used:
                    case CmdDisableStepperIsr:
                        printer.cmdDisableStepperIsr();
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
                        #if defined(HEAVYDEBUG)
                        bytesLeft = MSerial._available();
                        #endif
                }

                #if defined(HEAVYDEBUG)
                massert(MSerial._available() == bytesLeft);
                #endif

                MSerial.flush0(); // clear rx buffer
            }

            PT_RESTART();
            PT_END();
        }

        // Check checksum, do errorhandling if mismatch
        bool checkCrc(uint8_t flags, uint8_t cs1, uint8_t cs2, uint16_t computedCrc) {

            // 
            //  0x1: (x, y) -> (x, y)
            //  0x2: (0, y) -> (1, y)
            //  0x3: (x, 0) -> (x, 1)
            //  0x4: (0, 0) -> (1, 1)
            // 
            switch (flags) {
                    case 0x2:
                        cs2 &= ~0x1;
                        break;
                    case 0x3:
                        cs1 &= ~0x1;
                        break;
                    case 0x4:
                        cs1 &= ~0x1;
                        cs2 &= ~0x1;
                        break;
            }

            if (((cs2<<8)|cs1) != computedCrc) {

                    // printf("Checksum Error: 0x%x, computed: 0x%x\n", (cs2<<8)|cs1,  computedCrc);
                    crcError();
                    return false;
            }

            return true;
        }
};

static UsbCommand usbCommand;

FWINLINE void loop() {

    loopTS = millis();

    // Timer for slow running tasks (temp, encoder)
    static unsigned long timer10mS = millis();
    // static unsigned long timer50mS = millis();
    static unsigned long timer100mS = millis();

    if (printer.printerState == Printer::StateStart) {

        if (printer.bufferLow == -1) {
            if  (stepBuffer.byteSize() > 10)
                printer.bufferLow = 0;
        }
        else {
            if ((stepBuffer.byteSize() < 10) &&
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

    if ((loopTS - timer10mS) > 10) { // Every 10 mS

        // Check hardware and software endstops:
        if (printer.moveType == Printer::MoveTypeNormal) {

            if (X_ENDSTOP_PRESSED || Y_ENDSTOP_PRESSED || Z_ENDSTOP_PRESSED) {

                // SERIAL_ECHOPGM("POS: ");
                // SERIAL_ECHO(current_pos_steps[X_AXIS]);
                // SERIAL_ECHO(", ");
                // SERIAL_ECHO(current_pos_steps[Y_AXIS]);
                // SERIAL_ECHO(", ");
                // SERIAL_ECHOLN(current_pos_steps[Z_AXIS]);
                LCDMSGKILL(RespHardwareEndstop, "", "");
                txBuffer.sendResponseStart(RespKilled);
                txBuffer.sendResponseUint8(RespHardwareEndstop);
                txBuffer.sendResponseValue((uint8_t*)current_pos_steps, 3*sizeof(int32_t));
                txBuffer.sendResponseUint8(X_ENDSTOP_PRESSED);
                txBuffer.sendResponseUint8(Y_ENDSTOP_PRESSED);
                txBuffer.sendResponseUint8(Z_ENDSTOP_PRESSED);
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
                LCDMSGKILL(RespSoftwareEndstop, "", "");
                txBuffer.sendResponseStart(RespKilled);
                txBuffer.sendResponseUint8(RespSoftwareEndstop);
                txBuffer.sendResponseValue((uint8_t*)current_pos_steps, 3*sizeof(int32_t));
                txBuffer.sendResponseUint8(X_SW_ENDSTOP_PRESSED);
                txBuffer.sendResponseUint8(Y_SW_ENDSTOP_PRESSED);
                txBuffer.sendResponseUint8(Z_SW_ENDSTOP_PRESSED);
                txBuffer.sendResponseEnd();
                kill();
            }
        }

        //
        // Measure temperatures every 10ms (build mean value by OVERSAMPLENR)
        //
        tempControl.Run();

        if ((loopTS - timer100mS) > 100) { // Every 100 mS

            // lcd_lib_buttons_update_interrupt();
            // lcd_update();

            //
            // Control heater 
            //
            tempControl.heater();

            printer.checkMoveFinished();

#if defined(HASFILAMENTSENSOR)
            // Read filament sensor
            filamentSensor.run();
#endif

            timer100mS = loopTS;
        }
        timer10mS = loopTS;
    }

    // Read usb commands
    usbCommand.Run();

    // Write usb/serial output
    txBuffer.Run();

    // If printing, then read steps from the sd buffer and push it to
    // the print buffer.
    fillBufferTask.Run();
    swapDev.Run();

    // Check swap dev error
    if (swapDev.errorCode()) {
        LCDMSGKILL(RespSDError, swapDev.errorCode(), "");
        txBuffer.sendSimpleResponse(RespKilled, RespSDError, swapDev.errorCode());
        kill();
    }
}


void printDebugInfo() {
#if defined(REPRAP_DISCOUNT_SMART_CONTROLLER)
    lcd.setCursor(0, 0); lcd.print("ser:"); lcd.print(MSerial._available());
    lcd.print("B:"); lcd.print(swapDev.isBusyWriting());
    lcd.print("WP:"); lcd.print(swapDev.getWritePos());

    lcd.setCursor(0, 1); lcd.print("swd:"); lcd.print(swapDev.available());
    lcd.print("MIL:"); lcd.print(millis()-loopTS);

    lcd.setCursor(0, 2); lcd.print("sdr:"); lcd.print(sDReader.available());

    lcd.setCursor(0, 3); lcd.print("stb:"); lcd.print(stepBuffer.byteSize());

    // Wait 100 s before reboot
    for (int i=0; i<100000; i++) {
        txBuffer.Run();
        watchdog_reset();
        delay(1);
    }
#endif
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





