
#include <Arduino.h>

#include "filsensor.h"
#include "pins.h"
#include "MarlinSerial.h"

/*
 * Estimate Polling rate:
 *
 * Range X/Y is -128...127
 * Max extrusion: 50mm³/s, with 1.75mm filament: 12mm/s filament speed
 * Resolution: 25.4mm/500 = 0.0508mm/count
 * Clock rate at max extrusion: 12mm/s / 0.0508 = 2362 clocks/s
 * Time for 127 clocks at max extrusion: 127 / (2362 clocks/s) = 54ms
 */

FilamentSensor filamentSensor;

FilamentSensor::FilamentSensor() {

    yPos = 0;

    pinMode(FILSENSSCLK, OUTPUT);
    pinMode(FILSENSSDIO, OUTPUT);
    pinMode(FILSENSNCS, OUTPUT);
}

void FilamentSensor::init() {

    mouse_reset();
    delay(10);

    SERIAL_ECHO("Filament sensor Prod: ");
    SERIAL_ECHOLN(readLoc(0x0)); // Product
    SERIAL_ECHO("Filament sensor Rev: ");
    SERIAL_ECHOLN(readLoc(0x1)); // Rev
    SERIAL_ECHO("Filament sensor Control: ");
    SERIAL_ECHOLN(readLoc(0xd));
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
    }
}

#if 0
unsigned char mot;

void xloop() {

  mot = readLoc(0x2);
  if (mot) {
    // SERIAL_ECHO("Mot: ");
    // SERIAL_ECHO(mot);
    SERIAL_ECHO("X: ");
    SERIAL_ECHO((int)(char)readLoc(0x3));
    SERIAL_ECHO(", y: ");
    int y = (char)readLoc(0x4);
    SERIAL_ECHO(y);
    SERIAL_ECHO(", Y: ");
    Y += y;
    SERIAL_ECHO(Y);
    SERIAL_ECHO(", QUAL: ");
    SERIAL_ECHOLN(readLoc(0x5));
    SERIAL_ECHOLN("");
  }
  delay(50);
}
#endif

void FilamentSensor::mouse_reset(){
  // Initiate chip reset
  digitalWrite(FILSENSNCS, LOW);
  pushbyte(0x3a);
  pushbyte(0x5a);
  digitalWrite(FILSENSNCS, HIGH);

  delay(10);

  // Set 1000cpi resolution, xxx does not work?
  digitalWrite(FILSENSNCS, LOW);
  pushbyte(0x0d);
  pushbyte(0x01);
  digitalWrite(FILSENSNCS, HIGH);
}

uint8_t FilamentSensor::readLoc(uint8_t addr){
  uint8_t ret=0;
  digitalWrite(FILSENSNCS, LOW);
  pushbyte(addr);
  ret=pullbyte();
  digitalWrite(FILSENSNCS, HIGH);
  return(ret);
}

void FilamentSensor::pushbyte(uint8_t c){
  pinMode(FILSENSSDIO, OUTPUT);
  for(unsigned int i=0x80;i;i=i>>1){
    digitalWrite(FILSENSSCLK, LOW);
    digitalWrite(FILSENSSDIO, c & i);
    digitalWrite(FILSENSSCLK, HIGH);
  }
}

// unsigned int FilamentSensor::pullbyte(){
uint8_t FilamentSensor::pullbyte(){
  uint8_t ret=0;
  pinMode(FILSENSSDIO, INPUT);
  for(unsigned int i=0x80; i>0; i>>=1) {
    digitalWrite(FILSENSSCLK, LOW);
    ret |= i*digitalRead(FILSENSSDIO);
    digitalWrite(FILSENSSCLK, HIGH);
  }
  pinMode(FILSENSSDIO, OUTPUT);
  return(ret);
}

