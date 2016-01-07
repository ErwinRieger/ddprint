
#include <Arduino.h>

#include "ddprint.h"
#include "filsensor.h"
// #include "MarlinSerial.h"
#include "stepper.h"
#include "temperature.h"

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

    // pinMode(FILSENSSCLK, OUTPUT);
    SET_OUTPUT(FILSENSSCLK);
    // pinMode(FILSENSSDIO, OUTPUT);
    SET_OUTPUT(FILSENSSDIO);
    // pinMode(FILSENSNCS, OUTPUT);
    SET_OUTPUT(FILSENSNCS);
}

void FilamentSensor::init() {

    mouse_reset();

    SERIAL_ECHOPGM("Filament sensor Prod: ");
    SERIAL_ECHOLN((int)readLoc(0x0)); // Product
    SERIAL_ECHOPGM("Filament sensor Rev: ");
    SERIAL_ECHOLN((int)readLoc(0x1)); // Rev
    SERIAL_ECHOPGM("Filament sensor Control: ");
    SERIAL_ECHOLN((int)readLoc(0xd));
}

extern uint16_t tempExtrusionRateTable[];
#define FTIMER (F_CPU/8.0)
#define FTIMER1000 (FTIMER/1000.0)

void FilamentSensor::run() {

    if (readLoc(0x2)) { // Motion register

        // XXX x_delta must be read also?!
        readLoc(0x3);
        int8_t y = (int8_t)readLoc(0x4);

        // SERIAL_ECHO("X: ");
        // SERIAL_ECHO((int)x);
        // if (y) {
            // SERIAL_ECHO(" Y: ");
            // SERIAL_ECHOLN((int)y);
        // }

#if 0
        if (y>0)
            y += abs(x);
        else if (y<0)
            y -= abs(x);
#endif

        yPos -= y;

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

void FilamentSensor::mouse_reset(){

  // Initiate chip reset
  WRITE(FILSENSNCS, LOW);
  pushbyte(0x3a | 0x80);
  delayMicroseconds(20);
  pushbyte(0x5a);
  delayMicroseconds(20);
  WRITE(FILSENSNCS, HIGH);

  // Set 1000cpi resolution, xxx does not work?
  WRITE(FILSENSNCS, LOW);
  pushbyte(0x0d | 0x80);
  delayMicroseconds(20);
  pushbyte(0x01);
  delayMicroseconds(20);
  WRITE(FILSENSNCS, HIGH);
}

uint8_t FilamentSensor::readLoc(uint8_t addr){
  uint8_t ret=0;
  WRITE(FILSENSNCS, LOW);
  pushbyte(addr);
  ret=pullbyte();
  WRITE(FILSENSNCS, HIGH);
  return(ret);
}

void FilamentSensor::pushbyte(uint8_t c){
  // pinMode(FILSENSSDIO, OUTPUT);
  SET_OUTPUT(FILSENSSDIO);
  for(unsigned int i=0x80;i;i=i>>1){
    WRITE(FILSENSSCLK, LOW);
    WRITE(FILSENSSDIO, c & i);
    // wait 120ns, Tsetup
    delayMicroseconds(1);
    WRITE(FILSENSSCLK, HIGH);
  }
}

// unsigned int FilamentSensor::pullbyte(){
uint8_t FilamentSensor::pullbyte(){
  uint8_t ret=0;
  // pinMode(FILSENSSDIO, INPUT);
  SET_INPUT(FILSENSSDIO);
  for(unsigned int i=0x80; i>0; i>>=1) {
    WRITE(FILSENSSCLK, LOW);
    // wait 120ns, Tdly
    delayMicroseconds(1);
    ret |= i*READ(FILSENSSDIO);
    WRITE(FILSENSSCLK, HIGH);
  }
  // pinMode(FILSENSSDIO, OUTPUT);
  SET_OUTPUT(FILSENSSDIO);
  return(ret);
}

