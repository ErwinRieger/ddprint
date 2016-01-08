
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

FilamentSensor filamentSensor;

FilamentSensor::FilamentSensor() {

    lastASteps = lastYPos = yPos = 0;
    slip = 0.0;
    // maxTempSpeed = 0;
    lastTS = millis();

    // enabled = false;
    enabled = true;

    SET_INPUT(FILSENSMISO);
    SET_OUTPUT(FILSENSMOSI);
    SET_OUTPUT(FILSENSSCLK);
    // Pull high chip select of filament sensor to free the
    // SPI bus (for sdcard).
    SET_OUTPUT(FILSENSNCS);
    WRITE(FILSENSNCS, HIGH);
}

void FilamentSensor::init() {

    mouse_reset(); // does spiInit()

    SERIAL_ECHOPGM("Filament sensor Prod: ");
    SERIAL_ECHOLN((int)readLoc(0x0)); // Product
    SERIAL_ECHOPGM("Filament sensor Rev: ");
    SERIAL_ECHOLN((int)readLoc(0x1)); // Rev
    SERIAL_ECHOPGM("Filament sensor Control: ");
    SERIAL_ECHOLN((int)readLoc(0xd));

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
  SPCR = (1 << SPE) | (1 << MSTR) | (spiRate >> 1);
  SPSR = spiRate & 1 || spiRate == 6 ? 0 : 1 << SPI2X;
}

extern uint16_t tempExtrusionRateTable[];
#define FTIMER (F_CPU/8.0)
#define FTIMER1000 (FTIMER/1000.0)
#include <pins_arduino.h>

void FilamentSensor::run() {

    spiInit(6); // scale = pow(2, 3+1), 1Mhz ddd

    massert(readLoc(0x0) == 0x32);
    massert(readLoc(0x1) == 0x3);
    massert(readLoc(0x0) == ~readLoc(0xfc));
    massert(readLoc(0x1) == ~readLoc(0xcd));

    uint8_t mot = readLoc(0x2);

    if (mot & 0x16) {
        SERIAL_ERROR_START;
        SERIAL_ECHOLNPGM("FilSensor: X/Y overflow!");
    }

    if (mot & 0x80) { // Motion register

        // XXX x_delta must be read also?!
        readLoc(0x3); // X_L
        uint16_t y = (int8_t)readLoc(0x4);

        uint16_t xyh = readLoc(0x5); // XY_L
        y = y + ((int8_t)((xyh & 0xF) << 4))/16; // Y_L

#if 0
        if (y>0)
            y += abs(x);
        else if (y<0)
            y -= abs(x);
#endif

        yPos -= y;

        // SERIAL_ECHO("X: ");
        // SERIAL_ECHO((int)x);
        SERIAL_ECHO("Squal: ");
        SERIAL_ECHO((int)readLoc(0x6));
        SERIAL_ECHO(", y: ");
        SERIAL_ECHO(y);
        SERIAL_ECHO(", Y: ");
        SERIAL_ECHOLN(yPos);

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

void writeLoc(uint8_t addr, uint8_t value) {

  // Initiate chip reset
  WRITE(FILSENSNCS, LOW);
  spiSend(addr | 0x80);
  delayMicroseconds(14); // Tsww (30) - 16
  spiSend(value);
  delayMicroseconds(14); // Tsww (30) - 16
  WRITE(FILSENSNCS, HIGH);
}

void FilamentSensor::mouse_reset(){

    spiInit(6); // scale = pow(2, 3+1), 1Mhz ddd

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

    // turn on laser
    // writeLoc(0x1a, 0x00);
    // writeLoc(0x1f, 0xC0);
    writeLoc(0x1a, 0x40);
    writeLoc(0x1f, 0x80);
    // invalid: writeLoc(0x1a, 0x80);
    // invalid: writeLoc(0x1f, 0x40);
    // writeLoc(0x1a, 0xC0);
    // writeLoc(0x1f, 0x00); // complementary reg of 0x1A
   
    // writeLoc(0x1C, 0x0);
    // writeLoc(0x1d, 0xFF); // complementary reg of 0x1C

    writeLoc(0x1C, 0xFF);
    writeLoc(0x1d, 0x0); // complementary reg of 0x1C

#if defined SENSORA5020
  // Set 1000cpi resolution, xxx does not work?
  WRITE(FILSENSNCS, LOW);
  spiSend(0x0d | 0x80);
  delayMicroseconds(14); // Tsww (30) - 16
  spiSend(0x01);
  WRITE(FILSENSNCS, HIGH);
  delayMicroseconds(12); // Tswr (20) - 8
#endif
}

uint8_t FilamentSensor::readLoc(uint8_t addr){
  uint8_t ret=0;

  WRITE(FILSENSNCS, LOW);
  spiSend(addr);
  delayMicroseconds(4); // Tsrad
  ret=spiRec();
  WRITE(FILSENSNCS, HIGH);
  delayMicroseconds(50); // dddd
  return(ret);
}

