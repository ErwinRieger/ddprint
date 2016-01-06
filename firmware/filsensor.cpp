
#include <Arduino.h>

#include "ddprint.h"
#include "filsensor.h"
// #include "MarlinSerial.h"
#include "stepper.h"

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
    maxTempSpeed = 0;
    lastTS = millis();

    pinMode(FILSENSSCLK, OUTPUT);
    pinMode(FILSENSSDIO, OUTPUT);
    pinMode(FILSENSNCS, OUTPUT);
}

void FilamentSensor::init() {

    mouse_reset();

    SERIAL_ECHO("Filament sensor Prod: ");
    SERIAL_ECHOLN((int)readLoc(0x0)); // Product
    SERIAL_ECHO("Filament sensor Rev: ");
    SERIAL_ECHOLN((int)readLoc(0x1)); // Rev
    SERIAL_ECHO("Filament sensor Control: ");
    SERIAL_ECHOLN((int)readLoc(0xd));
}

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

            if (slip < 0.90) {

                SERIAL_ECHO("Slip: ");
                SERIAL_ECHOLN(slip);

                // 10% slip extruder feedrate:
                // float speed = (ds / AXIS_STEPS_PER_MM_E) / ((ts - lastTS)/1000.0);
                float speed = (ds * 1000.0) / (AXIS_STEPS_PER_MM_E * (ts - lastTS));

printf("90%% feedrate: %.2f\n", speed);
            }
            else {
                maxTempSpeed = 0;
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
  delayMicroseconds(100);
  pushbyte(0x5a);
  delayMicroseconds(100);
  WRITE(FILSENSNCS, HIGH);

  // Set 1000cpi resolution, xxx does not work?
  WRITE(FILSENSNCS, LOW);
  pushbyte(0x0d | 0x80);
  delayMicroseconds(100);
  pushbyte(0x01);
  delayMicroseconds(100);
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
  pinMode(FILSENSSDIO, OUTPUT);
  for(unsigned int i=0x80;i;i=i>>1){
    WRITE(FILSENSSCLK, LOW);
    WRITE(FILSENSSDIO, c & i);
    WRITE(FILSENSSCLK, HIGH);
  }
}

// unsigned int FilamentSensor::pullbyte(){
uint8_t FilamentSensor::pullbyte(){
  uint8_t ret=0;
  pinMode(FILSENSSDIO, INPUT);
  for(unsigned int i=0x80; i>0; i>>=1) {
    WRITE(FILSENSSCLK, LOW);
    ret |= i*READ(FILSENSSDIO);
    WRITE(FILSENSSCLK, HIGH);
  }
  pinMode(FILSENSSDIO, OUTPUT);
  return(ret);
}

