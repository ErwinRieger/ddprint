
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

#include <Arduino.h>

#include "ddprint.h"
#include "filsensor.h"
#include "stepper.h"
#include "temperature.h"
#include "fastio.h"
#include "ddserial.h"
#include "ddcommands.h"
#include "ddlcd.h"
#include "ddmacro.h"

#define FilSensorDebug 1

#if defined(PMWFS)
    SPISettings spiSettingsFS(4000000, MSBFIRST, SPI_MODE3);
#endif

#if defined(PMWFS)

#include "pmw3360fw.h"

FilamentSensorPMW3360 filamentSensor;

FilamentSensorPMW3360::FilamentSensorPMW3360() {

    feedrateLimiterEnabled = true;
    filSensorCalibration = 1.0;
    axis_steps_per_mm_e = 100;

    sensorCount = 0;

    minStepperSteps = 0.25 * axis_steps_per_mm_e;

    init();
}

void FilamentSensorPMW3360::init() {

    slippage.reset(1.0);
    grip = 1.0;
}

uint8_t FilamentSensorPMW3360::readLoc(uint8_t addr){

  WRITE(FILSENSNCS, LOW);
  dDPrintSpi.transfer(addr);
  delayMicroseconds(100); // Tsrad

  uint8_t ret = dDPrintSpi.transfer(0);
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns

  WRITE(FILSENSNCS, HIGH);
  delayMicroseconds(19); // tSRW/tSRR (=20us) minus tSCLK-NCS 
  return ret;
}

void FilamentSensorPMW3360::writeLoc(uint8_t addr, uint8_t value) {

  WRITE(FILSENSNCS, LOW);
  dDPrintSpi.transfer(addr | 0x80);

  dDPrintSpi.transfer(value);

  delayMicroseconds(20); // tSCLK-NCS for write operation
  WRITE(FILSENSNCS, HIGH);
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
}

void FilamentSensorPMW3360::getDY() {

    //write 0x01 to Motion register and read from it to freeze the motion values and make them available
    writeLoc(Motion, 0x01);
    uint8_t mot = readLoc(Motion);

    if (mot & 0x80) { // Motion register

#if !defined(burst)
        // XXX x_delta must be read also?!
        readLoc(Delta_X_L); // X_L
        readLoc(Delta_X_H); // X_H

        uint8_t y = readLoc(Delta_Y_L); // Y_L
        int8_t yh = readLoc(Delta_Y_H);
#endif

        int32_t dy = ((int16_t)yh << 8) | y;

        // lcd.setCursor(0, 1); lcd.print("          ");
        // lcd.setCursor(0, 1); lcd.print(y);

        // sensorCount += dy;
        // negate for v4 feeder
        sensorCount -= dy;
    }
}

void FilamentSensorPMW3360::run() {

    CRITICAL_SECTION_START
    long astep = current_pos_steps[E_AXIS];
    CRITICAL_SECTION_END

    if (astep != lastASteps) {

        float deltaStepperSteps = astep - lastASteps; // Requested extruded length

        if (deltaStepperSteps >= minStepperSteps) {

            dDPrintSpi.beginTransaction(spiSettingsFS);
            // int16_t deltaSensorSteps = getDY(); // read distance delta from filament sensor
            getDY(); // read distance delta from filament sensor
            int32_t deltaSensorSteps = sensorCount - lastSensorCount;

            //
            // Compute the amount of "feeder slippage"
            //
            // Slippage is 1.0 if the feeder does not slip, then the amount of feed filament is
            // exactly the amount that should be extruded.
            //
            // Normally slippage is greater 1.0 since there is always some back-pressure in the
            // filament and therefore the amount of filamnet fed is lower than it should be.
            //
            if (deltaSensorSteps > 0) {

                float slip = (deltaStepperSteps * filSensorCalibration) / deltaSensorSteps;

                // ungenaue sache: 
                //
                // filSensorCalibration ist experimentell ermittelt und
                // der sensor ist ja im endeffekt ein analoges gerät mit rauschen usw. und
                // wird desshalb keine besondere genauigkeit/reproduzierbarkeit bringen.
                //
                // desshalb gibt es hier beim "slippage" auch ausreißer:
                //  * negative werte
                //      wir haben grad die richtung gewechselt (zigzag,retract usw).
                //      das sollten ausreisser sein und daher den durchschnitt nicht so stark beinflussen
                //  * positive werte kleiner 1:
                //      feeder ist besser als 100%, z.b. filSensorCalibration zu klein
                //      sensor liefert halt für das aktuelle abtastfenster zu viele counts
                //  * positive werte viel größer 1:
                //      entweder fehlmessung oder sehr großes slipping, wir beschränken den wert dann auf 10%
                //      das sollten ausreisser sein und daher den durchschnitt nicht so stark beinflussen
                //
                slippage.addValue( min( max(slip, 0.1), 10.0) );
            }
            else {

                // No measurement from filament sensor:
                //   * no filament
                //   * filament grinding
                //   * some hardware failure
                if (deltaStepperSteps > (axis_steps_per_mm_e * 5)) {
                    slippage.addValue(10);
                }
                // break, still counting steps, counts
                return;
            }

            lastASteps = astep;
            lastSensorCount = sensorCount;

            float s = slippage.value();

            if (feedrateLimiterEnabled && (s > 0.0)) {

                float allowedSlippage = s * 0.90; // allow for 10% slip before we slow down
                float g = max(1.0, pow(allowedSlippage - 1.0, 2)*25);
                grip = STD min((float)3.0, g);
            }

            /*
            lcd.setCursor(0, 0); lcd.print("DS:"); lcd.print(deltaStepperSteps); lcd.print("     ");
            lcd.setCursor(0, 1); lcd.print("DY:"); lcd.print(deltaSensorSteps); lcd.print("     ");
            lcd.setCursor(0, 2); lcd.print("RA:"); lcd.print(ratio); lcd.print("     ");
            lcd.setCursor(0, 3); lcd.print("RA:"); lcd.print(s); lcd.print(" "); lcd.print(grip); lcd.print("     ");
            */
        }

    }
}

void FilamentSensorPMW3360::reset() {

    // return;

    dDPrintSpi.beginTransaction(spiSettingsFS);

    WRITE(FILSENSNCS, HIGH); // adns_com_end(); // ensure that the serial port is reset
    WRITE(FILSENSNCS, LOW); // adns_com_begin(); // ensure that the serial port is reset
    WRITE(FILSENSNCS, HIGH); // adns_com_end(); // ensure that the serial port is reset

    writeLoc(Power_Up_Reset, 0x5a); // force reset
    delay(50); // wait for it to reboot

    // read registers 0x02 to 0x06 (and discard the data)
    readLoc(Motion);
    readLoc(Delta_X_L);
    readLoc(Delta_X_H);
    readLoc(Delta_Y_L);
    readLoc(Delta_Y_H);

    /////////////////////////////////////////////////////////////
    // send the firmware to the chip, cf p.18 of the datasheet

    //Write 0 to Rest_En bit of Config2 register to disable Rest mode.
    writeLoc(Config2, 0x00);
  
    // write 0x1d in SROM_enable reg for initializing
    writeLoc(SROM_Enable, 0x1d); 
  
    // wait for more than one frame period
    delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
    // write 0x18 to SROM_enable to start SROM download
    writeLoc(SROM_Enable, 0x18); 
  
    // write the SROM file (=firmware data) 
    WRITE(FILSENSNCS, LOW); // adns_com_begin();
    dDPrintSpi.transfer(SROM_Load_Burst | 0x80); // write burst destination adress
    delayMicroseconds(15);
  
    // send all bytes of the firmware
    unsigned char c;
    for(uint16_t i = 0; i < sizeof(firmware_data); i++){ 
        c = (unsigned char)pgm_read_byte(firmware_data + i);
        dDPrintSpi.transfer(c);
        delayMicroseconds(15);
    }

    WRITE(FILSENSNCS, HIGH); // adns_com_end();

    delay(1);

    //Read the SROM_ID register to verify the ID before any other register reads or writes.
    uint8_t srom_id = readLoc(SROM_ID);

    //
    // Write 0x00 to Config2 register for wired mouse or 0x20 for wireless mouse design.
    // Wireless mode is powersaving mode (REST MODE).
    //
    writeLoc(Config2, 0x00);

    // set initial CPI resolution, 5000 cpi is default cpi
    // 0x77: 12000 cpi
    writeLoc(Config1, 0x78);
  
    // WRITE(FILSENSNCS, HIGH); // adns_com_end();

    uint8_t snap = readLoc(Angle_Snap);
        writeLoc(Angle_Snap, snap | 0x80);

    /////////////////////////////////////////////////////////////
    // delay(10);

    if (srom_id != SROMVER) {
        LCDMSGKILL(RespKilled, "srom_id != SROMVER", srom_id);
        txBuffer.sendSimpleResponse(RespKilled, RespFilsensorInit);
        kill();
    }


// xxxx was init():
//
    lastASteps = current_pos_steps[E_AXIS];
    lastSensorCount = sensorCount;
}

#endif // #if defined(PMWFS)



