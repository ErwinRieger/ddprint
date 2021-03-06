
/*
* This file is part of ddprint - a direct drive 3D printer firmware.
* 
* Copyright 2020 erwin.rieger@ibrieger.de
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

#if defined(AVR)

#include "ddtemp.h"
#include "temperature.h"
#include "pins.h"
#include "thermistortables.h"



// Common for all avr boards (um2, ramps, rumba)
#if defined(AVR)
    DefineIOPinMembers(3);
    DefineIOPinMembers(51);
    DefineIOPinMembers(52);
    DefineIOPinMembers(53);
#endif

#if MOTHERBOARD == 1
    //
    // Ultimaker UM2
    //
    DefineIOPinMembers(2);
    DefineIOPinMembers(4);
    DefineIOPinMembers(7);
    DefineIOPinMembers(8);
    DefineIOPinMembers(22);
    DefineIOPinMembers(23);
    DefineIOPinMembers(25);
    DefineIOPinMembers(26);
    DefineIOPinMembers(27);
    DefineIOPinMembers(29);
    DefineIOPinMembers(30);
    DefineIOPinMembers(31);
    DefineIOPinMembers(32);
    DefineIOPinMembers(33);
    DefineIOPinMembers(34);
    DefineIOPinMembers(35);
    DefineIOPinMembers(36);
    DefineIOPinMembers(37);
    DefineIOPinMembers(42);
    DefineIOPinMembers(43);
    DefineIOPinMembers(44);
    DefineIOPinMembers(45);
    DefineIOPinMembers(46);
    DefineIOPinMembers(69);
#elif MOTHERBOARD == 2
    //
    // Ramps
    //
    DefineIOPinMembers(8);
    DefineIOPinMembers(9);
    DefineIOPinMembers(10);
    DefineIOPinMembers(13);
    DefineIOPinMembers(15);
    DefineIOPinMembers(19);
    DefineIOPinMembers(24);
    DefineIOPinMembers(26);
    DefineIOPinMembers(28);
    DefineIOPinMembers(38);
    DefineIOPinMembers(46);
    DefineIOPinMembers(48);
    DefineIOPinMembers(54);
    DefineIOPinMembers(55);
    DefineIOPinMembers(56);
    DefineIOPinMembers(59);
    DefineIOPinMembers(60);
    DefineIOPinMembers(61);
    DefineIOPinMembers(62);
#elif MOTHERBOARD == 4
    //
    // Rumba
    //
    DefineIOPinMembers(2);
    DefineIOPinMembers(7);
    DefineIOPinMembers(9);
    DefineIOPinMembers(11);
    DefineIOPinMembers(13);
    DefineIOPinMembers(14);
    DefineIOPinMembers(15);
    DefineIOPinMembers(16);
    DefineIOPinMembers(17);
    DefineIOPinMembers(22);
    DefineIOPinMembers(23);
    DefineIOPinMembers(24);
    DefineIOPinMembers(33);
    DefineIOPinMembers(35);
    DefineIOPinMembers(37);
    DefineIOPinMembers(47);
    DefineIOPinMembers(48);
    DefineIOPinMembers(54);
    DefineIOPinMembers(55);
    DefineIOPinMembers(56);
    DefineIOPinMembers(57);
    DefineIOPinMembers(59);
    DefineIOPinMembers(62);
#endif // motherboard

void HAL_SETUP_TEMP_ADC() {

    //
    // Set analog inputs
    //
    // Disable 'digital input register' on the ADC pins TEMP_0_PIN and TEMP_BED_PIN
    DIDR0 = 0;
    DIDR2 = 0;

    #if defined(TEMP_0_PIN)
        DIDR2 |= 1<<(TEMP_0_PIN - 8);
    #endif

    #if defined(TEMP_BED_PIN)
        DIDR2 |= 1<<(TEMP_BED_PIN - 8);
    #endif

    // ADEN: Enable ADC
    // ADSC: Do initial conversion
    // ADIF: Clear interrupt flag?
    // 0x07: Set 16MHz/128 = 125kHz the ADC reference clock
    // ADCSRA = 1<<ADEN | 1<<ADSC | 1<<ADIF | 0x07;
    ADCSRA = 1<<ADEN | 1<<ADSC | 0x07;

    // Wait for initial conversion
    while (ADCSRA & (1<<ADSC));
}

bool TempControl::Run() {

    PT_BEGIN();

    ////////////////////////////////
    // Handle hotend 
    ////////////////////////////////

    //
    // Start hotend measure
    //
    ADCSRB = 1<<MUX5;

    // Set voltage reference to Avcc, set channel to temp 0
    ADMUX = ((1 << REFS0) | (TEMP_0_PIN & 0x07));
    ADCSRA |= 1<<ADSC; // Start conversion

    // Wait for conversion and read value
    // printf("TempControl::Run() wait for hotend\n");
    PT_WAIT_WHILE( ADCSRA & (1<<ADSC) );

    avgHotendTemp.addValue(tempFromRawADC(ADC));
    current_temperature[0] = avgHotendTemp.value();

    ////////////////////////////////
    // Handle heated bed 
    ////////////////////////////////

    //
    // Start bedtemp measure
    //
    ADCSRB = 1<<MUX5;

    // Set voltage reference to Avcc, set channel to bedtemp
    ADMUX = ((1 << REFS0) | (TEMP_BED_PIN & 0x07));
    ADCSRA |= 1<<ADSC; // Start conversion

    // Wait for conversion and read value
    // printf("TempControl::Run() wait for bed\n");
    PT_WAIT_WHILE( ADCSRA & (1<<ADSC) );

    avgBedTemp.addValue(tempFromRawADC(ADC));
    current_temperature_bed = avgBedTemp.value();

    PT_RESTART();
        
    // Not reached
    PT_END();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Initialize the spi bus
//
void spiInit() {

    // Do some minimal SPI init, prevent SPI to go to spi slave mode
    SDSS :: initDeActive();

    FILSENSNCS :: initDeActive();

    SCK_PIN :: initDeActive();

    MOSI_PIN :: initDeActive();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Mass storage, SDCard interface
//
bool MassStorage::swapInit() {

    if (! begin(&m_spi, SDSS :: getPin(), SPI_FULL_SPEED)) {

        return false;
    }

    if (type() == SD_CARD_TYPE_SDHC) {
        blockShift = 0;
    } else {
        blockShift = 9;
    }

    massert(eraseSingleBlockEnable());

    readRetry = 0;
    return true;
}

#endif

