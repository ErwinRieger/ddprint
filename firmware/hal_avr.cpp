
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

// Thermistortable Bed
static ThermistorTable<TempCircuitBed> thermistorTableBed;

// Thermistortable Hotend
static ThermistorTable<TempCircuitHotend> thermistorTableHotend;

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
    DefineIOPinMembers(18);
    DefineIOPinMembers(19);
    DefineIOPinMembers(22);
    DefineIOPinMembers(23);
    DefineIOPinMembers(25);
    DefineIOPinMembers(26);
    DefineIOPinMembers(27);
    DefineIOPinMembers(29);
    DefineIOPinMembers(30);
    DefineIOPinMembers(31);
    DefineIOPinMembers(33);
    DefineIOPinMembers(34);
    DefineIOPinMembers(35);
    DefineIOPinMembers(36);
    DefineIOPinMembers(37);
    DefineIOPinMembers(42);
    DefineIOPinMembers(32);
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
    DefineIOPinMembers(8);
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
#elif MOTHERBOARD == 5
    //
    // Creality melzi
    //
    DefineIOPinMembers(4);
    DefineIOPinMembers(5);
    DefineIOPinMembers(7);
    DefineIOPinMembers(12);
    DefineIOPinMembers(13);
    DefineIOPinMembers(14);
    DefineIOPinMembers(18);
    DefineIOPinMembers(19);
    DefineIOPinMembers(20);
    DefineIOPinMembers(26);
    DefineIOPinMembers(27);
    // DefineIOPinMembers(24);
    DefineIOPinMembers(31);
#elif MOTHERBOARD == 6
    //
    // Anycubic I3 trigorilla
    //
    DefineIOPinMembers(5);
    DefineIOPinMembers(7);
    DefineIOPinMembers(8);
    DefineIOPinMembers(9);
    DefineIOPinMembers(10);
    DefineIOPinMembers(11);
    // DefineIOPinMembers(15);
    DefineIOPinMembers(18);
    DefineIOPinMembers(24);
    DefineIOPinMembers(30);
    DefineIOPinMembers(38);
    DefineIOPinMembers(42);
    DefineIOPinMembers(43);
    DefineIOPinMembers(44);
    DefineIOPinMembers(56);
    DefineIOPinMembers(62);
#else
    #error UnknownBoard
#endif // motherboard

void HAL_SETUP_TEMP_ADC() {

    //
    // Set analog inputs
    //
    // Disable 'digital input register' on the ADC pins TEMP_0_PIN and TEMP_BED_PIN
    DIDR0 = 0;

#if defined(DIDR2)
    // atmega 2560
    DIDR2 = 0;

    #if defined(TEMP_0_PIN)
    if (TEMP_0_PIN < 8)
        DIDR0 |= 1<<TEMP_0_PIN;
    else
        DIDR2 |= 1<<(TEMP_0_PIN - 8);
    #endif

    #if defined(TEMP_BED_PIN)
    if (TEMP_BED_PIN < 8)
        DIDR0 |= 1<<TEMP_BED_PIN;
    else
        DIDR2 |= 1<<(TEMP_BED_PIN - 8);
    #endif
#else
    // atmega 128p4
    #if defined(TEMP_0_PIN)
        DIDR0 |= 1<<TEMP_0_PIN;
    #endif

    #if defined(TEMP_BED_PIN)
        DIDR0 |= 1<<TEMP_BED_PIN;
    #endif
#endif

    // ADEN: Enable ADC
    // ADSC: Do initial conversion
    // 0x07: Set 16MHz/128 = 125kHz the ADC reference clock
    ADCSRA = 1<<ADEN | 1<<ADSC | 0x07;

    // Wait for initial conversion
    while (ADCSRA & (1<<ADSC));
}

#if defined(MUX5)
    #define START_CONVERSION(chan) \
        if (chan < 8) \
            ADCSRB = 0; \
        else \
            ADCSRB = 1<<MUX5; \
        ADMUX = (1 << REFS0) | (chan & 0x07); \
        ADCSRA |= 1<<ADSC;

#else
    #define START_CONVERSION(chan) \
        ADCSRB = 0; \
        ADMUX = (1 << REFS0) | (chan & 0x07); \
        ADCSRA |= 1<<ADSC;
#endif

bool TempControl::Run() {

    PT_BEGIN();

    ////////////////////////////////
    // Handle hotend 
    ////////////////////////////////
    //
    // Start measurement
    //
    START_CONVERSION(TEMP_0_PIN);

/*
    ADCSRB = 1<<MUX5;

    // Set voltage reference to Avcc, set channel to temp 0
    ADMUX = ((1 << REFS0) | (TEMP_0_PIN & 0x07));
    ADCSRA |= 1<<ADSC; // Start conversion
*/

    // Wait for conversion and read value
    // printf("TempControl::Run() wait for hotend\n");
    PT_WAIT_WHILE( ADCSRA & (1<<ADSC) );

    current_temperature[0] = avgHotendTemp.addValue( thermistorTableHotend.tempFromRawADC(ADC) );

    ////////////////////////////////
    // Handle heated bed measurement
    ////////////////////////////////
    //
    // Start measurement
    //
    START_CONVERSION(TEMP_BED_PIN);

/*
    ADCSRB = 1<<MUX5;

    // Set voltage reference to Avcc, set channel to bedtemp
    ADMUX = ((1 << REFS0) | (TEMP_BED_PIN & 0x07));
    ADCSRA |= 1<<ADSC; // Start conversion
*/

    // Wait for conversion and read value
    // printf("TempControl::Run() wait for bed\n");
    PT_WAIT_WHILE( ADCSRA & (1<<ADSC) );

    current_temperature_bed = avgBedTemp.addValue( thermistorTableBed.tempFromRawADC(ADC) );

    PT_RESTART();
        
    // Not reached
    PT_END();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Emergency hard reset/reboot
//
void systemHardReset() {

    //
    // Watchdog reset method does not work, it halts but 
    // does not reset?
    //
    // wdt_enable(WDTO_1S);
    // while (true);
    
    asm volatile ("jmp 0 \n");
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Initialize timers
//
void timerInit() {

    //
    // Setup Timer1 for stepper interrupt and 
    // homingstepper interrupt.
    //

    // 
    // Timer 0 is used by arduino (millis() ...)
    // Timer 1 is stepper, 16 bit
    // Timer 2 is 8bit only
    // Timer 3 ist heater pwm
    // Timer 4 ist LED pin und FAN pin
    // Timer 5 ist digipot
    //

#if 0
//mega128p4

  timer 0 8bit used by arduino (millis() ...) AND fan at B4?
    B4 alternate function: OC0B (Timer/Counter 0 Output Compare Match B Output)

  timer 1 16bit, heater PWM at D5
    D5 OC1A (Timer/Counter1 Output Compare Match A Output)

  timer 2 8bit

  timer 3 16bit, stepper isr

  [digipot not used]
#endif

#if defined(StepperOnTimer3)
    // waveform generation = 0100 = CTC
    TCCR3B &= ~(1<<WGM33);
    TCCR3B |=  (1<<WGM32);
    TCCR3A &= ~(1<<WGM31);
    TCCR3A &= ~(1<<WGM30);
    
    // output mode = 00 (disconnected)
    // Normal port operation, OCnA/OCnB/OCnC disconnected
    TCCR3A &= ~(3<<COM3A0);
    TCCR3A &= ~(3<<COM3B0);

    // Generally we use a divider of 8, resulting in a 2MHz timer
    // frequency on a 16MHz MCU.
    TCCR3B = (TCCR3B & ~(0x07<<CS30)) | (2<<CS30);

    OCR3A = 0x4000;
    OCR3B = 0x4000;
    TCNT3 = 0;
#else
    // waveform generation = 0100 = CTC
    TCCR1B &= ~(1<<WGM13);
    TCCR1B |=  (1<<WGM12);
    TCCR1A &= ~(1<<WGM11);
    TCCR1A &= ~(1<<WGM10);
    
    // output mode = 00 (disconnected)
    // Normal port operation, OCnA/OCnB/OCnC disconnected
    TCCR1A &= ~(3<<COM1A0);
    TCCR1A &= ~(3<<COM1B0);

    // Generally we use a divider of 8, resulting in a 2MHz timer
    // frequency on a 16MHz MCU.
    TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

    OCR1A = 0x4000;
    OCR1B = 0x4000;
    TCNT1 = 0;
#endif
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

    // Enable pullup on miso
    HAL_SET_INPUT_PU(MISO);
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

    readRetry = 5;
    writeRetry = 5;
    return true;
}

#endif

