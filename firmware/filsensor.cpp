
#include <Arduino.h>

#include "ddprint.h"
#include "filsensor.h"
#include "stepper.h"
#include "temperature.h"
#include "fastio.h"
#include "ddserial.h"
#include "ddcommands.h"
#include "ddlcd.h"

#if defined(ADNSFS)
    // #include "adns9800fwa4.h"
    // #include "adns9800fwa5.h"
    #include "adns9800fwa6.h"
#endif

/*
 * Estimate Polling rate:
 *
 * Range X/Y is -128...127
 * Max extrusion: 50mm³/s, with 1.75mm filament: 20.8 mm/s filament speed
 * Resolution: 25.4mm/1000 = 0.0254mm/count
 * count rate at max extrusion: 20.8 mm/s / 0.0254 mm = 819 counts/s
 * Time for 127 counts at max extrusion: 127 / (819 counts/s) = 155 ms
 *
 * Extruderspeed for 5mm³/s flowrate
 * v5 = 5 / (math.pi/4 * pow(1.75, 2))
 *
 * Frequenz e-stepper bei 4mm/s:
 *  4 * 141 =  564
 */

// Factor to compute Extruder steps from filament sensor count
// #define ASTEPS_PER_COUNT (25.4*141/1000.0)

// mm/s, ca. 7.2 mm³/s
#define FilSensorMinSpeed 3

#define FilSensorDebug 1

#if defined(ADNSFS)

FilamentSensorADNS9800 filamentSensor;

FilamentSensorADNS9800::FilamentSensorADNS9800() {

    // maxTempSpeed = 0;

    // SET_INPUT(FILSENSMISO);
    // SET_OUTPUT(FILSENSMOSI);
    // SET_OUTPUT(FILSENSSCLK);
    // Pull high chip select of filament sensor to free the
    // SPI bus (for sdcard).
    WRITE(FILSENSNCS, HIGH);
    SET_OUTPUT(FILSENSNCS);

    init();
}

void FilamentSensorADNS9800::spiInit(uint8_t spiRate) {
  // See avr processor documentation
  SPCR = (1 << SPE) | (1 << MSTR) | (spiRate >> 1) | (1<<CPHA) | (1<<CPOL); // Mode 3
  SPSR = spiRate & 1 || spiRate == 6 ? 0 : 1 << SPI2X;
}

void FilamentSensorADNS9800::init() {

    yPos = 0;
    lastASteps = current_pos_steps[E_AXIS];
    lastTS = millis();

    iRAvg = 0;
    nRAvg = 0;

    // grip = 200;
    targetSpeed = 0;
    actualSpeed = 0;
}

uint8_t FilamentSensorADNS9800::readLoc(uint8_t addr){

  WRITE(FILSENSNCS, LOW);
  spiSend(addr);
  delayMicroseconds(100); // Tsrad

  uint8_t ret=spiRec();
  WRITE(FILSENSNCS, HIGH);
  delayMicroseconds(20); // Tsrw/Tsrr
  return(ret);
}

void FilamentSensorADNS9800::writeLoc(uint8_t addr, uint8_t value) {

  WRITE(FILSENSNCS, LOW);
  spiSend(addr | 0x80);

  spiSend(value);
  delayMicroseconds(100); // Tsww/Tswr 
  WRITE(FILSENSNCS, HIGH);
}

#define FTIMER (F_CPU/8.0)
#include <pins_arduino.h>

int16_t FilamentSensorADNS9800::getDY() {

    uint8_t mot = readLoc(REG_Motion);

    if (mot & 0x80) { // Motion register

#if !defined(burst)
        // XXX x_delta must be read also?!
        readLoc(REG_Delta_X_L); // X_L
        readLoc(REG_Delta_X_H); // X_H

        uint8_t y = readLoc(REG_Delta_Y_L); // Y_L
        int8_t yh = readLoc(REG_Delta_Y_H);

#endif

        int16_t dy = ((int16_t)yh << 8) | y;

        if (! dy) return 0;

        yPos += dy;

#if !defined(burst)
        // uint8_t squal = readLoc(REG_SQUAL);
        // uint16_t shutter = ((uint16_t)readLoc(REG_Shutter_Upper)<<8) | readLoc(REG_Shutter_Lower);
#endif

        return dy; 
    }

    return 0;
}

void FilamentSensorADNS9800::run() {

    spiInit(3); // scale = pow(2, 3+1), 1Mhz

    // Berechne soll flowrate, filamentsensor ist sehr ungenau bei kleiner geschwindigkeit.

    long astep = current_pos_steps[E_AXIS];
    uint32_t ts = millis();

    int16_t ds = astep - lastASteps; // Requested extruded length
    int16_t dy = getDY(); // Real extruded length

#if 0
    if (ds < 0) {

        iRAvg = 0;
        nRAvg = 0;

        lastASteps = astep;
        lastTS = ts;
        return;
    }
#endif

    uint16_t dt = ts - lastTS;

    rAvgS[iRAvg] = ds;
    rAvg[iRAvg++] = dy;

    if (iRAvg == RAVGWINDOW)
        iRAvg = 0;

    if (nRAvg < RAVGWINDOW)
        nRAvg++;

    int32_t ssum = 0;
    int32_t rsum = 0;
    for (int i=0; i<nRAvg; i++) {
        ssum += rAvgS[i];
        rsum += rAvg[i];
    }

    // i16          = int         / int
    targetSpeed = (ssum*100000) / ((int32_t)nRAvg * AXIS_STEPS_PER_MM_E * dt);

    // i16          = int         / float
    actualSpeed = (rsum*100000) / (nRAvg * FS_STEPS_PER_MM * dt);

    if ((actualSpeed > 300) && fillBufferTask.synced() && stepBuffer.synced()) { // 3mm/s bzw. 7.2mm³/s

        int16_t grip = ((int32_t)actualSpeed*100) / targetSpeed;

        if ((grip > 0) && (grip < 50)) {

            // printf("grip < 75: %d, %4.1f %4.1f\n", grip, actualSpeed*100, targetSpeed*0.4);

            int16_t curTempIndex = (int16_t)(current_temperature[0] - extrusionLimitBaseTemp) / 2;
            if ((curTempIndex > 0) && (curTempIndex < NExtrusionLimit)) {

                // printf("found temptable index: %d\n", curTempIndex);
                txBuffer.sendResponseStart(RespUnsolicitedMsg);
                txBuffer.sendResponseUint8(ExtrusionLimitDbg);
                txBuffer.sendResponseInt16(curTempIndex);
                txBuffer.sendResponseInt16(actualSpeed);
                txBuffer.sendResponseInt16(targetSpeed);
                txBuffer.sendResponseInt16(grip);
                txBuffer.sendResponseEnd();

                // Increase table values, decrease allowed speed for this and the following temperaturew
                // //////////////////////////////////////////
                uint16_t prev_old_timer = tempExtrusionRateTable[curTempIndex-1];
                uint16_t prev_old_rate = FTIMER / prev_old_timer;
                uint16_t prev_new_rate = prev_old_rate;

                while (curTempIndex < NExtrusionLimit) {

                    uint16_t this_old_timer = tempExtrusionRateTable[curTempIndex];
                    uint16_t this_old_rate = FTIMER / this_old_timer;

                    uint16_t new_dy = (this_old_rate - prev_old_rate) * 0.95; // xxx arbitrary

                    uint16_t this_new_rate = prev_new_rate + new_dy;

                    // printf("adjust rate: %d -> %d\n", this_old_rate, this_new_rate);

                    tempExtrusionRateTable[curTempIndex] = FTIMER / this_new_rate;

                    prev_old_rate = this_old_rate;
                    prev_new_rate = this_new_rate;

                    curTempIndex++;
                }

                fillBufferTask.sync();
                stepBuffer.sync();
            }
        }
    }

    lastASteps = astep;
    lastTS = ts;
}

void FilamentSensorADNS9800::reset(){

    spiInit(3); // scale = pow(2, 3+1), 1Mhz

#if 0
    uint8_t productId = readLoc(REG_Product_ID); // Product
    SERIAL_ECHOPGM("Filament sensor Prod: 0x");
    MSerial.println(productId, HEX);

    // Check inverse product id:
    uint8_t invProductId = readLoc(REG_Inverse_Product_ID);
    SERIAL_ECHOPGM("REG_Inverse_Product_ID: 0x");
    MSerial.println(invProductId, HEX);

    SERIAL_ECHOPGM("Filament sensor Rev: 0x");
    MSerial.println(readLoc(REG_Revision_ID), HEX); // Rev

    uint8_t configReg1 = readLoc(REG_Configuration_I);
    SERIAL_ECHOPGM("Filament sensor config1: 0x");
    MSerial.println(configReg1, HEX);
#endif

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
    // SERIAL_ECHOPGM("Uploading firmware, # of bytes: ");
    // SERIAL_ECHOLN(sizeof(sromData));

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
    // SERIAL_ECHOPGM("SROM ID: ");
    // MSerial.println(srom_id, HEX );
    
    if (srom_id != SROMVER) {
        LCDMSGKILL(RespKilled, "srom_id != SROMVER", "");
        txBuffer.sendSimpleResponse(RespKilled, RespADNS9800Init);
        kill();
    }

    // end upload

    // fixed frame rate
    // uint8_t conf = readLoc(REG_Configuration_II);
    // writeLoc(REG_Configuration_II, conf & 0x08 );

    // writeLoc(REG_Frame_Period_Max_Bound_Lower, 0xa0 );
    // writeLoc(REG_Frame_Period_Max_Bound_Upper, 0x0f );
    // delay(100);

    //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
    // reading the actual value of the register is important because the real
    // default value is different from what is said in the datasheet, and if you
    // change the reserved bytes (like by writing 0x00...) it would not work.
    uint8_t laser_ctrl0 = readLoc(REG_LASER_CTRL0);
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
    // kill();
    // }

    uint8_t configReg1 = readLoc(REG_Configuration_I);
    // writeLoc(REG_Configuration_I, (configReg1 & 0xC0) | 18); // resolution: 3600
    // writeLoc(REG_Configuration_I, (configReg1 & 0xC0) | 22); // resolution: 4400
    writeLoc(REG_Configuration_I, (configReg1 & 0xC0) | 41); // resolution: 8200

#if 0
    configReg1 = readLoc(REG_Configuration_I);
    SERIAL_ECHOPGM("Filament sensor config1: 0x");
    MSerial.println(configReg1, HEX);

    uint8_t resbits = configReg1 & 0x3f;
    SERIAL_ECHOPGM("Filament sensor resolution: ");
    MSerial.println((uint16_t)resbits * 200);

    uint8_t configReg2 = readLoc(REG_Configuration_II);
    SERIAL_ECHOPGM("Filament sensor config2: 0x");
    MSerial.println(configReg2, HEX);
#endif

    uint8_t snap = readLoc(REG_Snap_Angle);
    writeLoc(REG_Snap_Angle, snap | 0x80);

    // SERIAL_ECHOPGM("Snap angle register: 0x");
    // MSerial.println(readLoc(REG_Snap_Angle), HEX);

    // fixed frame rate
    // writeLoc(REG_Configuration_II, configReg2 & 0x08 );
    // writeLoc(REG_Frame_Period_Max_Bound_Lower, 0xa0 );
    // writeLoc(REG_Frame_Period_Max_Bound_Upper, 0x0f );
    // delay(10);

    uint8_t productId = readLoc(REG_Product_ID); // Product
    // SERIAL_ECHOPGM("Filament sensor Prod: 0x");
    // MSerial.println(productId, HEX);

    // Check inverse product id:
    uint8_t invProductId = readLoc(REG_Inverse_Product_ID);
    // SERIAL_ECHOPGM("REG_Inverse_Product_ID: 0x");
    // MSerial.println(invProductId, HEX);
    massert((uint8_t)(~productId) == invProductId);

    // SERIAL_ECHOLNPGM("Optical Chip Initialized");
}

#endif // #if defined(ADNSFS)

#if defined(BournsEMS22AFS)

FilamentSensor filamentSensor;

FilamentSensor::FilamentSensor() {

    // grip = 200;
    // maxTempSpeed = 0;

    // enabled = false;
    enabled = true;

    // Pull high chip select of filament sensor to free the
    // SPI bus (for sdcard).
    WRITE(FILSENSNCS, HIGH);
    SET_OUTPUT(FILSENSNCS);

    init();
}

void FilamentSensor::init() {

    yPos = 0;
    lastASteps = current_pos_steps[E_AXIS];
    lastTS = millis();
    lastEncoderPos = readEncoderPos();
}

void FilamentSensor::spiInit(uint8_t spiRate) {
  // See avr processor documentation
  SPCR = (1 << SPE) | (1 << MSTR) | (spiRate >> 1) | (1<<CPHA); // Mode 1
  SPSR = spiRate & 1 || spiRate == 6 ? 0 : 1 << SPI2X;
}

#define FTIMER (F_CPU/8.0)
#include <pins_arduino.h>

uint16_t FilamentSensor::readEncoderPos() {

    //
    // Read rotary encoder pos (0...1023)
    //
    spiInit(3);

    digitalWrite(FILSENSNCS, LOW);
    delayMicroseconds(1); // Tclkfe

    uint8_t byte1 = spiRec();
    uint8_t byte2 = spiRec();

    digitalWrite(FILSENSNCS, HIGH);
  
    // uint16_t pos = (((uint16_t)byte1) << 2) | (byte2 & 0x3);
    uint16_t pos = (((uint16_t)byte1) << 2) | ((byte2 & 0xC0) >> 6);

    /* 
    MSerial.print("Pos: ");
    MSerial.print(pos, HEX);
    MSerial.print(" compens: ");  // should be set
    MSerial.print(byte2 & 0x20);
    MSerial.print(" overr: ");    // should be cleared
    MSerial.print(byte2 & 0x10);
    MSerial.print(" linalarm: "); // should be cleared
    MSerial.print(byte2 & 0x8);
    MSerial.print(" increase: "); // not used
    MSerial.print(byte2 & 0x4);
    MSerial.print(" decrease: "); // not used
    MSerial.print(byte2 & 0x2);
    MSerial.print(" parity: ");
    MSerial.println(byte2 & 0x1);
    MSerial.println("");
    */

    // MSerial.print("Status: ");
    // MSerial.println(byte2 & 0x3E, HEX);
    // massert((byte2 & 0x3E) == 0x20);
    if ((byte2 & 0x3E) != 0x20) {
        MSerial.print("Error Status: ");
        MSerial.println(byte2 & 0x3E, HEX);
    }
    
    // XXX use bitpattern to test all relevant bits at once here..

    uint16_t dataWord = (((uint16_t)byte1) << 8) | byte2;
    uint8_t p = __builtin_parity(dataWord);
    massert(p == 0);

    return pos;
}

int16_t FilamentSensor::getDY() {

    uint16_t pos = readEncoderPos();

    int16_t dy = pos - lastEncoderPos; 

    if (dy < -512) {
        // Überlauf in positiver richtung
        dy = (1024 - lastEncoderPos) + pos;
    }
    else if (dy > 512) {
        // Überlauf in negativer richtung
        dy = (pos - 1024) - lastEncoderPos;
    }

    yPos += dy;

#if 0
    if (dy) {
        SERIAL_ECHO(", dy: ");
        SERIAL_ECHO(dy);
        SERIAL_ECHO(", yPos: ");
        SERIAL_ECHO(yPos);
        SERIAL_ECHO(", estep: ");
        SERIAL_ECHOLN(current_pos_steps[E_AXIS]);
    }
#endif

    lastEncoderPos = pos;
    return dy;
}

void FilamentSensor::run() {

    // Berechne soll flowrate, filamentsensor ist sehr ungenau bei kleiner geschwindigkeit.
CRITICAL_SECTION_START
    uint32_t ts = millis();
    int16_t ds = current_pos_steps[E_AXIS] - lastASteps; // Requested extruded length
    int16_t dy = getDY(); // Real extruded length 
CRITICAL_SECTION_END

    if (ds < 0) {

        // Retraction, reset/sync values
        // yPos = 0;
        // lastEncoderPos = readEncoderPos();
        // lastASteps = current_pos_steps[E_AXIS];
        // lastTS = millis();

        init();
    }
    else {

      // if (ds > 72) {

        // int16_t dy = getDY(); // Real extruded length 

// #if 0
        SERIAL_ECHO("DS, DY: ");
        SERIAL_ECHO(ts);
        SERIAL_ECHO(" ");
        SERIAL_ECHO(ds);
        SERIAL_ECHO(" ");
        SERIAL_ECHOLN(dy);
// #endif

#if 0
        SERIAL_ECHO("DS, DY: ");
        SERIAL_ECHO(ts);
        SERIAL_ECHO(" ");
        SERIAL_ECHO(current_pos_steps[E_AXIS]);
        SERIAL_ECHO(" ");
        SERIAL_ECHOLN(yPos);
#endif

#if 0
        uint16_t dt = ts - lastTS;

        float speed = (ds * 1000.0) / (AXIS_STEPS_PER_MM_E * dt);

        // if (speed > 2) { // ca. 5mm³/s
            // Berechne ist-flowrate, anhand filamentsensor

            float actualSpeed = (dy * 1000.0) / (FS_STEPS_PER_MM * dt);

            SERIAL_ECHO("Flowrate_mm/s: ");
            SERIAL_ECHO(ts);
            SERIAL_ECHO(" ");
            SERIAL_ECHO(speed);
            SERIAL_ECHO(" ");
            SERIAL_ECHOLN(actualSpeed);
        // }
#endif
        lastTS = ts;
        lastASteps = current_pos_steps[E_AXIS];
      // }
    }

    return;

}

#endif // #if defined(BournsEMS22AFS)

