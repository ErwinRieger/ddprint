
#include <Arduino.h>

#include "ddprint.h"
#include "filsensor.h"
#include "stepper.h"
#include "temperature.h"
#include "fastio.h"

// #include "adns9800fwa4.h"
// #include "adns9800fwa5.h"
#include "adns9800fwa6.h"

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
    WRITE(FILSENSNCS, HIGH);
    SET_OUTPUT(FILSENSNCS);
}

static void spiInit(uint8_t spiRate) {
  // See avr processor documentation
  SPCR = (1 << SPE) | (1 << MSTR) | (spiRate >> 1) | (1<<CPHA) | (1<<CPOL);
  SPSR = spiRate & 1 || spiRate == 6 ? 0 : 1 << SPI2X;
}

void FilamentSensor::init() {

    spiInit(3); // scale = pow(2, 3+1), 1Mhz

    SERIAL_ECHOPGM("Filament sensor Prod: ");
    MSerial.println(readLoc(REG_Product_ID), HEX); // Product

    SERIAL_ECHOPGM("Filament sensor Rev: ");
    MSerial.println(readLoc(REG_Revision_ID), HEX); // Rev

    reset();

    /*
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

    if (mot & 0x40) {
        SERIAL_ERROR_START;
        SERIAL_ECHOLNPGM("FilSensor: FAULT!");
    }
    if ((mot & 0x20) == 0) {
        SERIAL_ERROR_START;
        SERIAL_ECHOLNPGM("FilSensor: Laser off!");
    }
    */
}

uint8_t FilamentSensor::readLoc(uint8_t addr){
  uint8_t ret=0;

  WRITE(FILSENSNCS, LOW);
  spiSend(addr);
  // delayMicroseconds(4); // Tsrad
  delayMicroseconds(10); // Tsrad
  ret=spiRec();
  WRITE(FILSENSNCS, HIGH);
  delayMicroseconds(10); // Tsrad
  return(ret);
}

void writeLoc(uint8_t addr, uint8_t value) {

  // Initiate chip reset
  WRITE(FILSENSNCS, LOW);
  spiSend(addr | 0x80);
  // delayMicroseconds(14); // Tsww (30) - 16
  delayMicroseconds(20); // Tsww (30) - 16
  spiSend(value);
  // delayMicroseconds(14); // Tsww (30) - 16
  delayMicroseconds(20); // Tsww (30) - 16
  WRITE(FILSENSNCS, HIGH);
}

extern uint16_t tempExtrusionRateTable[];
#define FTIMER (F_CPU/8.0)
#define FTIMER1000 (FTIMER/1000.0)
#include <pins_arduino.h>

void FilamentSensor::getYPos() {

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

        spiInit(3); // scale = pow(2, 3+1), 1Mhz

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

    return;

#if defined(FilSensorDebug)
    massert(readLoc(REG_Product_ID) == 0x30);
    massert(readLoc(REG_Revision_ID) == 0x3);
    massert(readLoc(REG_Product_ID)+readLoc(0x3f) == 255);
    massert(readLoc(REG_Revision_ID)+readLoc(0x3e) == 255);
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

    spiSend(0x42);
    delayMicroseconds(4); // Tsrad
    uint8_t mot=spiSend(0);
    delayMicroseconds(4); // Tsrad
    uint8_t x=spiSend(0);
    delayMicroseconds(4); // Tsrad
    uint8_t y=spiSend(0);
    delayMicroseconds(4); // Tsrad
    uint8_t xyh=spiSend(0);
    delayMicroseconds(4); // Tsrad
    uint8_t squal = spiSend(0);
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
        MarlinSerial.println("\n timeout, yyreset...\n");
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
    massert(readLoc(REG_Product_ID) == 0x30);
    massert(readLoc(REG_Revision_ID) == 0x3);
    massert(readLoc(REG_Product_ID)+readLoc(0x3f) == 255);
    massert(readLoc(REG_Revision_ID)+readLoc(0x3e) == 255);
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

    spiSend(0x42);
    delayMicroseconds(4); // Tsrad
    uint8_t mot=spiSend(0);
    delayMicroseconds(4); // Tsrad
    uint8_t x=spiSend(0);
    delayMicroseconds(4); // Tsrad
    uint8_t y=spiSend(0);
    delayMicroseconds(4); // Tsrad
    uint8_t xyh=spiSend(0);
    delayMicroseconds(4); // Tsrad
    uint8_t squal = spiSend(0);
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
        MarlinSerial.println("\n timeout, yyreset...\n");
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

void FilamentSensor::reset(){

    // ensure that the serial port is reset
    WRITE(FILSENSNCS, HIGH);
    WRITE(FILSENSNCS, LOW);
    WRITE(FILSENSNCS, HIGH);

    writeLoc(REG_Power_Up_Reset, 0x5a); // force reset
    delay(50); // wait for it to reboot

    // read registers 0x02 to 0x06 (and discard the data)
    readLoc(REG_Motion);
    readLoc(REG_Delta_X_L);
    readLoc(REG_Delta_X_H);
    readLoc(REG_Delta_Y_L);
    readLoc(REG_Delta_Y_H);

    // upload the firmware
    // send the firmware to the chip, cf p.18 of the datasheet
    SERIAL_ECHOPGM("Uploading firmware, # of bytes: ");
    SERIAL_ECHOLN(sizeof(sromData));

    // set the configuration_IV register in 3k firmware mode
    writeLoc(REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved 
  
    // write 0x1d in SROM_enable reg for initializing
    writeLoc(REG_SROM_Enable, 0x1d); 
  
    // wait for more than one frame period
    delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
    // write 0x18 to SROM_enable to start SROM download
    writeLoc(REG_SROM_Enable, 0x18); 
  
    // write the SROM file (=firmware data) 
    WRITE(FILSENSNCS, LOW);
    delay(1);
    spiSend(REG_SROM_Load_Burst | 0x80); // write burst destination adress
    delayMicroseconds(15);

    // send all bytes of the firmware
    unsigned char c;
    for(int i = 0; i < sizeof(sromData); i++) { 
        c = (unsigned char)pgm_read_byte(sromData + i);
        spiSend(c);
        delayMicroseconds(15);
    }
    WRITE(FILSENSNCS, HIGH);

    delay(1);
    
    uint8_t srom_id = readLoc(REG_SROM_ID);
    SERIAL_ECHOPGM("SROM ID: ");
    MSerial.println(srom_id, HEX );
    
    if (! (srom_id == SROMVER)) {
        SERIAL_ECHOLNPGM("ADNS9500::sromDownload : the firmware was not successful downloaded");
        while(1);
    }

    // end upload

    // fixed frame rate
    // byte conf = readLoc(REG_Configuration_II);
    // writeLoc(REG_Configuration_II, conf & 0x08 );

    // writeLoc(REG_Frame_Period_Max_Bound_Lower, 0xa0 );
    // writeLoc(REG_Frame_Period_Max_Bound_Upper, 0x0f );
    // delay(100);

    //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
    // reading the actual value of the register is important because the real
    // default value is different from what is said in the datasheet, and if you
    // change the reserved bytes (like by writing 0x00...) it would not work.
    byte laser_ctrl0 = readLoc(REG_LASER_CTRL0);
    // MarlinSerial.print("laser vorher : 0x");
    // MarlinSerial.println(laser_ctrl0, HEX );
    // writeLoc(REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );
    writeLoc(REG_LASER_CTRL0, laser_ctrl0 & ~0x1 );
  
    // laser_ctrl0 = readLoc(REG_LASER_CTRL0);
    // MarlinSerial.print("laser nachher : 0x");
    // MarlinSerial.println(laser_ctrl0, HEX );

    // uint8_t b = readLoc(REG_Motion);
    // if ((b & 0x60) != 0x20) {
        // MarlinSerial.print("Motion error !: 0x");
        // MarlinSerial.println(b, HEX );
    // while(1);
    // }

    SERIAL_ECHOLNPGM("Optical Chip Initialized");
}

