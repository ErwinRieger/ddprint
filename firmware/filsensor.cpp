
#include <Arduino.h>

#include "ddprint.h"
#include "filsensor.h"
// #include "MarlinSerial.h"
#include "stepper.h"
#include "temperature.h"
#include "fastio.h"

/*
 * Estimate Polling rate:
 *
 * Range X/Y is -128...127
 * Max extrusion: 50mm³/s, with 1.75mm filament: 20.8 mm/s filament speed
 * Resolution: 25.4mm/1000 = 0.0254mm/count
 * count rate at max extrusion: 20.8 mm/s / 0.0254 mm = 819 counts/s
 * Time for 127 counts at max extrusion: 127 / (819 counts/s) = 155 ms
 */

// Factor to compute Extruder steps from filament sensor count
#define ASTEPS_PER_COUNT (25.4*141/1000.0)
#define FS_STEPS_PER_MM (800.0/25.4)
// #define FS_STEPS_PER_MM (1600.0/25.4)

FilamentSensor filamentSensor;

FilamentSensor::FilamentSensor() {

    return;

    lastASteps = lastYPos = yPos = 0;
    slip = 0.0;
    // maxTempSpeed = 0;
    lastTS = millis();

    // enabled = false;
    enabled = true;

    // SET_INPUT(FILSENSMISO);
    // SET_OUTPUT(FILSENSMOSI);
    // SET_OUTPUT(FILSENSSCLK);
    // Pull high chip select of filament sensor to free the
    // SPI bus (for sdcard).
    SET_OUTPUT(FILSENSNCS);
    WRITE(FILSENSNCS, HIGH);
}

void FilamentSensor::init() {

    return;

    mouse_reset(); // does spiInit()
    // snoop_reset();

    SERIAL_ECHOPGM("Filament sensor Prod: ");
    SERIAL_ECHOLN((int)readLoc(0x0)); // Product
    SERIAL_ECHOPGM("Filament sensor Rev: ");
    SERIAL_ECHOLN((int)readLoc(0x1)); // Rev

    uint8_t configReg1 = readLoc(0x12);
    SERIAL_ECHOPGM("Filament sensor config: ");
    SERIAL_ECHOLN((int)configReg1);

    uint8_t configReg2 = readLoc(0x36);
    if (configReg2 & 0x10) {
        SERIAL_ECHOPGM("Sensor config 0x36: ");
        SERIAL_ECHOLN((int)configReg2);

        switch ((configReg2 & 0xf) >> 1) {
            case 0x01:
                SERIAL_ECHOLNPGM("Sensor resolution 400cpi");
                break;
            case 0x02:
                SERIAL_ECHOLNPGM("Sensor resolution 800cpi");
                break;
            case 0x03:
                SERIAL_ECHOLNPGM("Sensor resolution 1200cpi");
                break;
            case 0x04:
                SERIAL_ECHOLNPGM("Sensor resolution 1600cpi");
                break;
            case 0x05:
                SERIAL_ECHOLNPGM("Sensor resolution 2000cpi");
                break;
        }
    }
    else {

        switch ((configReg1 & 0x60) >> 5) {
            case 0x00:
                SERIAL_ECHOLNPGM("Sensor resolution 400cpi");
                break;
            case 0x01:
                SERIAL_ECHOLNPGM("Sensor resolution 800cpi");
                break;
            case 0x02:
                SERIAL_ECHOLNPGM("Sensor resolution 1200cpi");
                break;
            case 0x03:
                SERIAL_ECHOLNPGM("Sensor resolution 1600cpi");
                break;
        }
    }

    SERIAL_ECHOPGM("Register 1A: ");
    SERIAL_ECHOLN((int)readLoc(0x1A));
    SERIAL_ECHOPGM("Register 1F: ");
    SERIAL_ECHOLN((int)readLoc(0x1F));

    SERIAL_ECHOPGM("Register 1C: ");
    SERIAL_ECHOLN((int)readLoc(0x1C));
    SERIAL_ECHOPGM("Register 1D: ");
    SERIAL_ECHOLN((int)readLoc(0x1D));

    uint8_t mot = readLoc(0x2);
    SERIAL_ECHOPGM("Register MOT: ");
    SERIAL_ECHOLN((int)mot);

    if (mot & 0x4) {
        SERIAL_ERROR_START;
        SERIAL_ECHOLNPGM("FilSensor: FAULT!");
    }
    if ((mot & 0x8) == 0) {
        SERIAL_ERROR_START;
        SERIAL_ECHOLNPGM("FilSensor: Laser off!");
    }
}

static void spiInit(uint8_t spiRate) {
  // See avr processor documentation
  // SPCR = (1 << SPE) | (1 << MSTR) | (spiRate >> 1);
  SPCR = (1 << SPE) | (1 << MSTR) | (spiRate >> 1) | (1<<CPHA) | (1<<CPOL);
  SPSR = spiRate & 1 || spiRate == 6 ? 0 : 1 << SPI2X;
}

void writeLoc(uint8_t addr, uint8_t value) {

  // Initiate chip reset
  WRITE(FILSENSNCS, LOW);
  spiSend(addr | 0x80);
  delayMicroseconds(14); // Tsww (30) - 16
  spiSend(value);
  delayMicroseconds(14); // Tsww (30) - 16
  WRITE(FILSENSNCS, HIGH);
}

extern uint16_t tempExtrusionRateTable[];
#define FTIMER (F_CPU/8.0)
#define FTIMER1000 (FTIMER/1000.0)
#include <pins_arduino.h>

void FilamentSensor::getYPos() {

    spiInit(3); // scale = pow(2, 3+1), 1Mhz

    uint8_t mot = readLoc(0x2);

    if (mot & 0x80) { // Motion register

        static union {
            struct {
                uint8_t lo;
                uint8_t hi;
            } bytes;
            int16_t value;
        } split;

#if !defined(burst)
        // XXX x_delta must be read also?!
        readLoc(0x3); // X_L
        // int16_t y = (int8_t)readLoc(0x4);
        uint8_t y = readLoc(0x4); // Y_L
        int8_t xyh = readLoc(0x5); // XY_H
#endif

#if 0 
        // Compute Delta X as a 16-bits signed integer
        split.bytes.lo = x;
        split.bytes.hi = (xyh >> 4) & 0x0F;
        if ( split.bytes.hi & 0x08 ) {
            split.bytes.hi |= 0xF0;             // Sign extension
        }
        xPos += split.value;
#endif

        // Compute Delta Y as a 16-bits signed integer
        split.bytes.lo = y;
        split.bytes.hi = xyh & 0x0F;
        if ( split.bytes.hi & 0x08 ) {
            split.bytes.hi |= 0xF0;             // Sign extension
        }

#if 0
        if (y>0)
            y += abs(x);
        else if (y<0)
            y -= abs(x);
#endif

        if (! split.value) return;

        yPos += split.value;

#if !defined(burst)
        uint8_t squal = readLoc(0x6);
        uint16_t shutter = (readLoc(0x7) << 8) + readLoc(0x8);
#endif

        // SERIAL_ECHO("X: ");
        // SERIAL_ECHO((int)x);
        SERIAL_ECHO("Squal: ");
        SERIAL_ECHO((int)squal);
        SERIAL_ECHO(", Shut: ");
        SERIAL_ECHO(shutter);
        SERIAL_ECHO(", y: ");
        SERIAL_ECHO((int)y);
        SERIAL_ECHO(", Y12: ");
        SERIAL_ECHO(split.value);
        SERIAL_ECHO(", Y: ");
        SERIAL_ECHO(yPos);
        SERIAL_ECHO(", minpix: ");
        SERIAL_ECHO((int) readLoc(0xb));
        SERIAL_ECHO(", avgpix: ");
        SERIAL_ECHO((int)readLoc(0xa) * 1.515);
        SERIAL_ECHO(", maxpix: ");
        SERIAL_ECHOLN((int) readLoc(0x9));
    }
}

// Extruderspeed for 5mm³/s flowrate
// v5 = 5 / (math.pi/4 * pow(1.75, 2))
//
void FilamentSensor::run() {

    return;

    // Berechne soll flowrate, filamentsensor ist sehr ungenau bei kleiner geschwindigkeit.
    int32_t ds = current_pos_steps[E_AXIS] - lastASteps; // Requested extruded length

    if (ds > 72) {

        getYPos();

        uint32_t ts = millis();

        // float speed = (ds / AXIS_STEPS_PER_MM_E) / ((ts - lastTS)/1000.0);
        float speed = (ds * 1000.0) / (AXIS_STEPS_PER_MM_E * (ts - lastTS));

        if (speed > 2) { // ca. 5mm³/s
            // Berechne ist-flowrate, anhand filamentsensor
            int32_t dy = yPos - lastYPos; // Real extruded length

            float realSpeed = (dy * 1000.0) / (FS_STEPS_PER_MM * (ts - lastTS));

            SERIAL_ECHO("Flowrate c/m: ");
            SERIAL_ECHO(speed);
            SERIAL_ECHO(", ");
            SERIAL_ECHOLN(realSpeed);
        }

        lastYPos = yPos;
        lastTS = ts;
        lastASteps = current_pos_steps[E_AXIS];
    }
    spiInit(3); // scale = pow(2, 3+1), 1Mhz

    return;

#if defined(FilSensorDebug)
    massert(readLoc(0x0) == 0x30);
    massert(readLoc(0x1) == 0x3);
    massert(readLoc(0x0)+readLoc(0x3f) == 255);
    massert(readLoc(0x1)+readLoc(0x3e) == 255);
#endif

#if 0
    static uint8_t power = 0;
    writeLoc(0x1C, power);
    writeLoc(0x1d, ~power); // complementary reg of 0x1C

    SERIAL_ECHO("power: ");
    SERIAL_ECHO((int)power);
    if (power < 255)
        power++;
#endif

#if defined(bust)
    // burst
    digitalWrite(NCS, LOW);

    SPI.transfer(0x42);
    delayMicroseconds(4); // Tsrad
    uint8_t mot=SPI.transfer(0);
    delayMicroseconds(4); // Tsrad
    uint8_t x=SPI.transfer(0);
    delayMicroseconds(4); // Tsrad
    uint8_t y=SPI.transfer(0);
    delayMicroseconds(4); // Tsrad
    uint8_t xyh=SPI.transfer(0);
    delayMicroseconds(4); // Tsrad
    uint8_t squal = SPI.transfer(0);
    delayMicroseconds(4); // Tsrad
    uint16_t shutter = (readLoc(0x7) << 8) + readLoc(0x8);
    // end burst

    digitalWrite(NCS, HIGH);
#else
    uint8_t mot = readLoc(0x2);
#endif

    if (mot & 0x10) {
        SERIAL_ERROR_START;
        SERIAL_ECHOLNPGM("FilSensor: X/Y overflow!");
    }

#if defined(FilSensorDebug)
    if (abs(yPos) > 0 and (millis() - lastTS) > 5000) {
        // xxx debug: use x direction to reset y...
        Serial.println("\n timeout, yyreset...\n");
        // xPos = 0;
        yPos = 0;
    }
#endif

    if (mot & 0x80) { // Motion register

        static union {
            struct {
                uint8_t lo;
                uint8_t hi;
            } bytes;
            int16_t value;
        } split;

#if !defined(burst)
        // XXX x_delta must be read also?!
        readLoc(0x3); // X_L
        // int16_t y = (int8_t)readLoc(0x4);
        uint8_t y = readLoc(0x4); // Y_L
        int8_t xyh = readLoc(0x5); // XY_H
#endif

#if 0 
        // Compute Delta X as a 16-bits signed integer
        split.bytes.lo = x;
        split.bytes.hi = (xyh >> 4) & 0x0F;
        if ( split.bytes.hi & 0x08 ) {
            split.bytes.hi |= 0xF0;             // Sign extension
        }
        xPos += split.value;
#endif

        // Compute Delta Y as a 16-bits signed integer
        split.bytes.lo = y;
        split.bytes.hi = xyh & 0x0F;
        if ( split.bytes.hi & 0x08 ) {
            split.bytes.hi |= 0xF0;             // Sign extension
        }

#if 0
        if (y>0)
            y += abs(x);
        else if (y<0)
            y -= abs(x);
#endif

        if (abs(split.value) > 0)
            lastTS = millis();

        if (! split.value) return;

        yPos += split.value;

#if !defined(burst)
        uint8_t squal = readLoc(0x6);
        uint16_t shutter = (readLoc(0x7) << 8) + readLoc(0x8);
#endif

        // SERIAL_ECHO("X: ");
        // SERIAL_ECHO((int)x);
        SERIAL_ECHO("Squal: ");
        SERIAL_ECHO((int)squal);
        SERIAL_ECHO(", Shut: ");
        SERIAL_ECHO(shutter);
        SERIAL_ECHO(", y: ");
        SERIAL_ECHO((int)y);
        SERIAL_ECHO(", Y12: ");
        SERIAL_ECHO(split.value);
        SERIAL_ECHO(", Y: ");
        SERIAL_ECHO(yPos);
        SERIAL_ECHO(", minpix: ");
        SERIAL_ECHO((int) readLoc(0xb));
        SERIAL_ECHO(", avgpix: ");
        SERIAL_ECHO((int)readLoc(0xa) * 1.515);
        SERIAL_ECHO(", maxpix: ");
        SERIAL_ECHOLN((int) readLoc(0x9));

        int32_t ds = current_pos_steps[E_AXIS] - lastASteps; // Requested extruded length

        if (ds > 72) {
           
            uint32_t ts = millis();

            int32_t dy = yPos - lastYPos; // Real extruded length

            slip = (dy * ASTEPS_PER_COUNT) / ds;

            // printf("slip: %d %d %.2f\n", dy, ds, slip);
            printf("slip: %.2f\n", slip);

            int16_t curTempIndex = (int16_t)(current_temperature[0] - 210) / 2;
            if ((curTempIndex >= 0) && (curTempIndex <= 30)) { // xxx 30 hardcoded

                if (enabled && (slip < 0.88)) {

                    // Increase table value, decrease allowed speed for this temperature

                    uint32_t timerValue = tempExtrusionRateTable[curTempIndex];

                    // 10% slip extruder feedrate:
                    // float speed = (ds / AXIS_STEPS_PER_MM_E) / ((ts - lastTS)/1000.0);
                    // float speed = (ds * 1000.0) / (AXIS_STEPS_PER_MM_E * (ts - lastTS));

                    // printf("90%% feedrate: %.2f\n", speed);

                    uint16_t tvnew = (FTIMER * timerValue)/(FTIMER - 0.3*timerValue*AXIS_STEPS_PER_MM_E);
                    printf("tv++/speed-- for temp %.2f: %d -> %d\n", current_temperature[0], timerValue, tvnew);
                    // tempExtrusionRateTable[curTempIndex] = tvnew; 
                    // tempExtrusionRateTable[curTempIndex] = (9*timerValue + tvnew) / 10;

                    SERIAL_ECHOPGM("Slip-: ");
                    SERIAL_ECHO(slip);
                    SERIAL_ECHOPGM(", ");
                    SERIAL_ECHO((uint16_t)current_temperature[0]);
                    SERIAL_ECHOPGM(", ");
                    SERIAL_ECHOLN(tempExtrusionRateTable[curTempIndex]);

                    // enabled = false;
                }
                else if (enabled && (slip > 0.92)) {
                    // Decrease table value, increase allowed speed for this temperature
                    // uint16_t tvnew = (FTIMER * timerValue) / (timerValue*AXIS_STEPS_PER_MM_E + ft);
                    // uint16_t tvnew = (FTIMER * timerValue)/(0.3*timerValue*AXIS_STEPS_PER_MM_E + FTIMER);
                    // float speed = (ds * 1000.0) / (AXIS_STEPS_PER_MM_E * (ts - lastTS));

                    uint32_t timerValue = tempExtrusionRateTable[curTempIndex];

                    uint16_t tvnew = (FTIMER1000 * (ts - lastTS)) / ds;

                    if (tvnew < timerValue) {

                        printf("tv--/speed++ for temp %.2f: %d -> %d\n", current_temperature[0], timerValue, tvnew);
                        // tempExtrusionRateTable[curTempIndex] = (9*timerValue + tvnew) / 10;

                        SERIAL_ECHOPGM("Slip+: ");
                        SERIAL_ECHO(slip);
                        SERIAL_ECHOPGM(", ");
                        SERIAL_ECHO((uint16_t)current_temperature[0]);
                        SERIAL_ECHOPGM(", ");
                        SERIAL_ECHOLN(tempExtrusionRateTable[curTempIndex]);
                    
                        // enabled = false;
                    }
                }
            }
            else {
                SERIAL_ECHOPGM("Slip-: ");
                SERIAL_ECHO(slip);
                SERIAL_ECHOPGM(" T out range: ");
                SERIAL_ECHOLN(tempExtrusionRateTable[curTempIndex]);
            }

            lastASteps = current_pos_steps[E_AXIS];
            lastYPos = yPos;
            lastTS = ts;
        }
    }
}

#if 0
void FilamentSensor::run() {

    spiInit(3); // scale = pow(2, 3+1), 1Mhz

#if defined(FilSensorDebug)
    massert(readLoc(0x0) == 0x30);
    massert(readLoc(0x1) == 0x3);
    massert(readLoc(0x0)+readLoc(0x3f) == 255);
    massert(readLoc(0x1)+readLoc(0x3e) == 255);
#endif

#if 0
    static uint8_t power = 0;
    writeLoc(0x1C, power);
    writeLoc(0x1d, ~power); // complementary reg of 0x1C

    SERIAL_ECHO("power: ");
    SERIAL_ECHO((int)power);
    if (power < 255)
        power++;
#endif

#if defined(bust)
    // burst
    digitalWrite(NCS, LOW);

    SPI.transfer(0x42);
    delayMicroseconds(4); // Tsrad
    uint8_t mot=SPI.transfer(0);
    delayMicroseconds(4); // Tsrad
    uint8_t x=SPI.transfer(0);
    delayMicroseconds(4); // Tsrad
    uint8_t y=SPI.transfer(0);
    delayMicroseconds(4); // Tsrad
    uint8_t xyh=SPI.transfer(0);
    delayMicroseconds(4); // Tsrad
    uint8_t squal = SPI.transfer(0);
    delayMicroseconds(4); // Tsrad
    uint16_t shutter = (readLoc(0x7) << 8) + readLoc(0x8);
    // end burst

    digitalWrite(NCS, HIGH);
#else
    uint8_t mot = readLoc(0x2);
#endif

    if (mot & 0x10) {
        SERIAL_ERROR_START;
        SERIAL_ECHOLNPGM("FilSensor: X/Y overflow!");
    }

#if defined(FilSensorDebug)
    if (abs(yPos) > 0 and (millis() - lastTS) > 5000) {
        // xxx debug: use x direction to reset y...
        Serial.println("\n timeout, yyreset...\n");
        // xPos = 0;
        yPos = 0;
    }
#endif

    if (mot & 0x80) { // Motion register

        static union {
            struct {
                uint8_t lo;
                uint8_t hi;
            } bytes;
            int16_t value;
        } split;

#if !defined(burst)
        // XXX x_delta must be read also?!
        readLoc(0x3); // X_L
        // int16_t y = (int8_t)readLoc(0x4);
        uint8_t y = readLoc(0x4); // Y_L
        int8_t xyh = readLoc(0x5); // XY_H
#endif

#if 0 
        // Compute Delta X as a 16-bits signed integer
        split.bytes.lo = x;
        split.bytes.hi = (xyh >> 4) & 0x0F;
        if ( split.bytes.hi & 0x08 ) {
            split.bytes.hi |= 0xF0;             // Sign extension
        }
        xPos += split.value;
#endif

        // Compute Delta Y as a 16-bits signed integer
        split.bytes.lo = y;
        split.bytes.hi = xyh & 0x0F;
        if ( split.bytes.hi & 0x08 ) {
            split.bytes.hi |= 0xF0;             // Sign extension
        }

#if 0
        if (y>0)
            y += abs(x);
        else if (y<0)
            y -= abs(x);
#endif

        if (abs(split.value) > 0)
            lastTS = millis();

        if (! split.value) return;

        yPos += split.value;

#if !defined(burst)
        uint8_t squal = readLoc(0x6);
        uint16_t shutter = (readLoc(0x7) << 8) + readLoc(0x8);
#endif

        // SERIAL_ECHO("X: ");
        // SERIAL_ECHO((int)x);
        SERIAL_ECHO("Squal: ");
        SERIAL_ECHO((int)squal);
        SERIAL_ECHO(", Shut: ");
        SERIAL_ECHO(shutter);
        SERIAL_ECHO(", y: ");
        SERIAL_ECHO((int)y);
        SERIAL_ECHO(", Y12: ");
        SERIAL_ECHO(split.value);
        SERIAL_ECHO(", Y: ");
        SERIAL_ECHO(yPos);
        SERIAL_ECHO(", minpix: ");
        SERIAL_ECHO((int) readLoc(0xb));
        SERIAL_ECHO(", avgpix: ");
        SERIAL_ECHO((int)readLoc(0xa) * 1.515);
        SERIAL_ECHO(", maxpix: ");
        SERIAL_ECHOLN((int) readLoc(0x9));

        int32_t ds = current_pos_steps[E_AXIS] - lastASteps; // Requested extruded length

        if (ds > 72) {
           
            uint32_t ts = millis();

            int32_t dy = yPos - lastYPos; // Real extruded length

            slip = (dy * ASTEPS_PER_COUNT) / ds;

            // printf("slip: %d %d %.2f\n", dy, ds, slip);
            printf("slip: %.2f\n", slip);

            int16_t curTempIndex = (int16_t)(current_temperature[0] - 210) / 2;
            if ((curTempIndex >= 0) && (curTempIndex <= 30)) { // xxx 30 hardcoded

                if (enabled && (slip < 0.88)) {

                    // Increase table value, decrease allowed speed for this temperature

                    uint32_t timerValue = tempExtrusionRateTable[curTempIndex];

                    // 10% slip extruder feedrate:
                    // float speed = (ds / AXIS_STEPS_PER_MM_E) / ((ts - lastTS)/1000.0);
                    // float speed = (ds * 1000.0) / (AXIS_STEPS_PER_MM_E * (ts - lastTS));

                    // printf("90%% feedrate: %.2f\n", speed);

                    uint16_t tvnew = (FTIMER * timerValue)/(FTIMER - 0.3*timerValue*AXIS_STEPS_PER_MM_E);
                    printf("tv++/speed-- for temp %.2f: %d -> %d\n", current_temperature[0], timerValue, tvnew);
                    // tempExtrusionRateTable[curTempIndex] = tvnew; 
                    // tempExtrusionRateTable[curTempIndex] = (9*timerValue + tvnew) / 10;

                    SERIAL_ECHOPGM("Slip-: ");
                    SERIAL_ECHO(slip);
                    SERIAL_ECHOPGM(", ");
                    SERIAL_ECHO((uint16_t)current_temperature[0]);
                    SERIAL_ECHOPGM(", ");
                    SERIAL_ECHOLN(tempExtrusionRateTable[curTempIndex]);

                    // enabled = false;
                }
                else if (enabled && (slip > 0.92)) {
                    // Decrease table value, increase allowed speed for this temperature
                    // uint16_t tvnew = (FTIMER * timerValue) / (timerValue*AXIS_STEPS_PER_MM_E + ft);
                    // uint16_t tvnew = (FTIMER * timerValue)/(0.3*timerValue*AXIS_STEPS_PER_MM_E + FTIMER);
                    // float speed = (ds * 1000.0) / (AXIS_STEPS_PER_MM_E * (ts - lastTS));

                    uint32_t timerValue = tempExtrusionRateTable[curTempIndex];

                    uint16_t tvnew = (FTIMER1000 * (ts - lastTS)) / ds;

                    if (tvnew < timerValue) {

                        printf("tv--/speed++ for temp %.2f: %d -> %d\n", current_temperature[0], timerValue, tvnew);
                        // tempExtrusionRateTable[curTempIndex] = (9*timerValue + tvnew) / 10;

                        SERIAL_ECHOPGM("Slip+: ");
                        SERIAL_ECHO(slip);
                        SERIAL_ECHOPGM(", ");
                        SERIAL_ECHO((uint16_t)current_temperature[0]);
                        SERIAL_ECHOPGM(", ");
                        SERIAL_ECHOLN(tempExtrusionRateTable[curTempIndex]);
                    
                        // enabled = false;
                    }
                }
            }
            else {
                SERIAL_ECHOPGM("Slip-: ");
                SERIAL_ECHO(slip);
                SERIAL_ECHOPGM(" T out range: ");
                SERIAL_ECHOLN(tempExtrusionRateTable[curTempIndex]);
            }

            lastASteps = current_pos_steps[E_AXIS];
            lastYPos = yPos;
            lastTS = ts;
        }
    }
}
#endif

void FilamentSensor::mouse_reset(){

    spiInit(3); // scale = pow(2, 3+1), 1Mhz

    // Initiate chip reset
    writeLoc(0x3a, 0x5a);
    delay(50); // Wait one frame - how long is a frame?

    // selfTest();

    writeLoc(0x2e, 0); // Clear observation register.
    delay(50); // Wait one frame

    // check observation register, all bits 0-3 must be set.
    uint8_t obs = readLoc(0x2e);
    if ((obs & 0xf) != 0xf) {
        SERIAL_ERROR_START;
        SERIAL_ECHOPGM("FilSensor: observation register wrong: ");
        SERIAL_ECHOLN((int)obs);
    }
 
    // Read from registers 0x02, 0x03, 0x04 and 0x05 
    for (uint8_t r=0x2; r<0x6; r++)
        readLoc(r);

    // viii. Write 0x27 to register 0x3C
    writeLoc(0x3C, 0x27);
    // ix. Write 0x0a to register 0x22
    writeLoc(0x22, 0x0a);
    // x. Write 0x01 to register 0x21
    writeLoc(0x21, 0x01);
    // xi. Write 0x32 to register 0x3C
    writeLoc(0x3C, 0x32);
    // xii. Write 0x20 to register 0x23
    writeLoc(0x23, 0x20);
    // xiii. Write 0x05 to register 0x3C
    writeLoc(0x3C, 0x05);
    // xiv. Write 0xB9 to register 0x37
    writeLoc(0x37, 0xB9);

    // selfTest();

    // turn on laser
    // writeLoc(0x1a, 0x00); // 0.9-3 mA
    // writeLoc(0x1f, 0xC0);
    writeLoc(0x1a, 0x40); // 2-5 mA
    writeLoc(0x1f, 0x80);
    // invalid: writeLoc(0x1a, 0x80);
    // invalid: writeLoc(0x1f, 0x40);
    // writeLoc(0x1a, 0xC0); // 4-10 mA
    // writeLoc(0x1f, 0x00); // complementary reg of 0x1A
   
    // writeLoc(0x1C, 0x0);
    // writeLoc(0x1d, 0xFF); // complementary reg of 0x1C
    // writeLoc(0x1C, 0x3f);
    // writeLoc(0x1d, 0xC0); // complementary reg of 0x1C
    // writeLoc(0x1C, 0x60);
    // writeLoc(0x1d, ~0x60); // complementary reg of 0x1C
    // writeLoc(0x1C, 0x7f);
    // writeLoc(0x1d, ~0x7f); // complementary reg of 0x1C
    // writeLoc(0x1C, 0x7f);
    // writeLoc(0x1d, ~0x7f); // complementary reg of 0x1C

    // writeLoc(0x1C, 0x5A);
    // writeLoc(0x1d, ~0x5A); // complementary reg of 0x1C
    // writeLoc(0x1C, 0x0);
    // writeLoc(0x1d, ~0x0); // complementary reg of 0x1C 

    writeLoc(0x1C, 0xFF);
    writeLoc(0x1d, 0x0); // complementary reg of 0x1C
    //
    // writeLoc(0x1C, 0xdd);
    // writeLoc(0x1d, 0x22); // complementary reg of 0x1C

    // Resolution 400
    // writeLoc(0x12, 0x00);
    // Resolution 1200
    // writeLoc(0x12, 0x40);
    // Resolution 1600
    // writeLoc(0x12, 0x60);
    // Resolution 2000
    // writeLoc(0x36, 0x1a);

    // selfTest();
}

void FilamentSensor::snoop_reset(){

    // Initiate chip reset
    writeLoc(0x3a, 0x5a);

    delay(10); // Wait one frame - how long is a frame?

    writeLoc(0x2e, 0); // Clear observation register.
    delay(10); // Wait one frame

    // check observation register, all bits 0-3 must be set.
    uint8_t obs = readLoc(0x2e);
    if ((obs & 0xf) != 0xf) {
        SERIAL_ERROR_START;
        SERIAL_ECHOPGM("FilSensor: observation register wrong: ");
        SERIAL_ECHOLN((int)obs);
    }

    // // viii. Write 0x27 to register 0x3C
    writeLoc(0x3C, 0x27);

    // ix. Write 0x10 to register 0x22
    writeLoc(0x22, 0x10);

    // xi. Write 0x22 to register 0x3C
    writeLoc(0x3C, 0x22);

    // xii. Write 0x32 to register 0x3D
    writeLoc(0x3D, 0x32);

    // Read from registers 0x02, 0x03, 0x04 and 0x05 
    for (uint8_t r=0x2; r<0x6; r++)
        readLoc(r);

    writeLoc(0x1a, 0x40); // 2-5 mA
    writeLoc(0x1C, 0x80);
    writeLoc(0x1d, 0x7f); // complementary reg of 0x1C
    writeLoc(0x1f, 0x80);

    // xiii. Write 0x23 to register 0x34
    writeLoc(0x34, 0x23);

    // xiv. Write 0x26 to register 0x12
    writeLoc(0x12, 0x26);
}


uint8_t FilamentSensor::readLoc(uint8_t addr){
  uint8_t ret=0;

  WRITE(FILSENSNCS, LOW);
  spiSend(addr);
  delayMicroseconds(4); // Tsrad
  ret=spiRec();
  WRITE(FILSENSNCS, HIGH);
  return(ret);
}

void FilamentSensor::selfTest(){
    SERIAL_ECHOLNPGM("Running sensor self test...");
    writeLoc(0x10, 0x01);

    delay(5000);

    SERIAL_ECHO("CRC0: ");
    SERIAL_ECHOLN((int)readLoc(0xc));
    SERIAL_ECHO("CRC1: ");
    SERIAL_ECHOLN((int)readLoc(0xd));
    SERIAL_ECHO("CRC2: ");
    SERIAL_ECHOLN((int)readLoc(0xe));
    SERIAL_ECHO("CRC3: ");
    SERIAL_ECHOLN((int)readLoc(0xf));

    massert(readLoc(0xc) == 0xf4);
    massert(readLoc(0xd) == 0x54);
    massert(readLoc(0xe) == 0x6d);
    massert(readLoc(0xf) == 0xeb);

    SERIAL_ECHOLNPGM("sensor self test done");
}


