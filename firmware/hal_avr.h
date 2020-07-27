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

#include <Arduino.h>
#include <avr/wdt.h>

#include "pin_states.h"
#include "fastio.h"

//
// ATMega 2560
//

#define SERIAL_TX_DR_EMPTY() ( (UCSR0A) & (1 << UDRE0) )
#define SERIAL_TX_COMPLETE() ( true )
#define SERIAL_TX_DR_PUTC(c) ( UDR0 = c )

#define HAL_SET_INPUT_PU(pin) pinMode(pin, INPUT_PULLUP)
#define HAL_SET_INPUT_ANALOG(pin) /* */

#define HAL_READ(pin) digitalRead(pin)
#define HAL_READ_ANALOG(pin) analogRead(pin)

#define HAL_SET_OUTPUT(pin)  pinMode(pin, OUTPUT)

// #define SET_OUTPUT_PWM(pin, activeLow)  SET_OUTPUT(pin)

#define HAL_WRITE(pin, v)  digitalWrite(pin, v)
// #define PWM_WRITE(p, v) analogWrite(p, v)

#define HAL_READ_ANALOG(pin) analogRead(pin)

#define CLI()   cli()
#define SEI()   sei()

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif

#define WDT_ENABLE() wdt_enable(WDTO_4S) /* Timeout 4 seconds */
#define WDT_RESET() wdt_reset()

#define TIMER_INIT() /* */


////////////////////////////////////////////////////////////////////////////////////////////////////

void HAL_SETUP_TEMP_ADC();
////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct AvrPinMapInfo {
    volatile uint8_t * pinAddr;
    volatile uint8_t * portAddr;
    uint8_t  bitMask;
    volatile uint8_t * ddrAddr;
    volatile uint8_t *pwmAddr;
} AvrPinMapInfo;

const AvrPinMapInfo avrPinMap[] =
{
{ &PINE, &PORTE, 1, & DDRE }, // 0 DIO0_PIN PINE0  NULL
{ &PINE, &PORTE, 2, & DDRE }, // 1 DIO1_PIN PINE1  NULL
{ &PINE, &PORTE, 16, & DDRE }, // 2 DIO2_PIN PINE4  &OCR3BL
{ &PINE, &PORTE, 32, & DDRE }, // 3 DIO3_PIN PINE5  &OCR3CL
{ &PING, &PORTG, 32, & DDRG }, // 4 DIO4_PIN PING5  &OCR0B
{ &PINE, &PORTE, 8, & DDRE }, // 5 DIO5_PIN PINE3  &OCR3AL
{ &PINH, &PORTH, 8, & DDRH }, // 6 DIO6_PIN PINH3  &OCR4AL
{ &PINH, &PORTH, 16, & DDRH }, // 7 DIO7_PIN PINH4  &OCR4BL
{ &PINH, &PORTH, 32, & DDRH }, // 8 DIO8_PIN PINH5  &OCR4CL
{ &PINH, &PORTH, 64, & DDRH }, // 9 DIO9_PIN PINH6  &OCR2B
{ &PINB, &PORTB, 16, & DDRB }, // 10 DIO10_PIN PINB4  &OCR2A
{ &PINB, &PORTB, 32, & DDRB }, // 11 DIO11_PIN PINB5  NULL
{ &PINB, &PORTB, 64, & DDRB }, // 12 DIO12_PIN PINB6  NULL
{ &PINB, &PORTB, 128, & DDRB }, // 13 DIO13_PIN PINB7  &OCR0A
{ &PINJ, &PORTJ, 2, & DDRJ }, // 14 DIO14_PIN PINJ1  NULL
{ &PINJ, &PORTJ, 1, & DDRJ }, // 15 DIO15_PIN PINJ0  NULL
{ &PINH, &PORTH, 2, & DDRH }, // 16 DIO16_PIN PINH1  NULL
{ &PINH, &PORTH, 1, & DDRH }, // 17 DIO17_PIN PINH0  NULL
{ &PIND, &PORTD, 8, & DDRD }, // 18 DIO18_PIN PIND3  NULL
{ &PIND, &PORTD, 4, & DDRD }, // 19 DIO19_PIN PIND2  NULL
{ &PIND, &PORTD, 2, & DDRD }, // 20 DIO20_PIN PIND1  NULL
{ &PIND, &PORTD, 1, & DDRD }, // 21 DIO21_PIN PIND0  NULL
{ &PINA, &PORTA, 1, & DDRA }, // 22 DIO22_PIN PINA0  NULL
{ &PINA, &PORTA, 2, & DDRA }, // 23 DIO23_PIN PINA1  NULL
{ &PINA, &PORTA, 4, & DDRA }, // 24 DIO24_PIN PINA2  NULL
{ &PINA, &PORTA, 8, & DDRA }, // 25 DIO25_PIN PINA3  NULL
{ &PINA, &PORTA, 16, & DDRA }, // 26 DIO26_PIN PINA4  NULL
{ &PINA, &PORTA, 32, & DDRA }, // 27 DIO27_PIN PINA5  NULL
{ &PINA, &PORTA, 64, & DDRA }, // 28 DIO28_PIN PINA6  NULL
{ &PINA, &PORTA, 128, & DDRA }, // 29 DIO29_PIN PINA7  NULL
{ &PINC, &PORTC, 128, & DDRC }, // 30 DIO30_PIN PINC7  NULL
{ &PINC, &PORTC, 64, & DDRC }, // 31 DIO31_PIN PINC6  NULL
{ &PINC, &PORTC, 32, & DDRC }, // 32 DIO32_PIN PINC5  NULL
{ &PINC, &PORTC, 16, & DDRC }, // 33 DIO33_PIN PINC4  NULL
{ &PINC, &PORTC, 8, & DDRC }, // 34 DIO34_PIN PINC3  NULL
{ &PINC, &PORTC, 4, & DDRC }, // 35 DIO35_PIN PINC2  NULL
{ &PINC, &PORTC, 2, & DDRC }, // 36 DIO36_PIN PINC1  NULL
{ &PINC, &PORTC, 1, & DDRC }, // 37 DIO37_PIN PINC0  NULL
{ &PIND, &PORTD, 128, & DDRD }, // 38 DIO38_PIN PIND7  NULL
{ &PING, &PORTG, 4, & DDRG }, // 39 DIO39_PIN PING2  NULL
{ &PING, &PORTG, 2, & DDRG }, // 40 DIO40_PIN PING1  NULL
{ &PING, &PORTG, 1, & DDRG }, // 41 DIO41_PIN PING0  NULL
{ &PINL, &PORTL, 128, & DDRL }, // 42 DIO42_PIN PINL7  NULL
{ &PINL, &PORTL, 64, & DDRL }, // 43 DIO43_PIN PINL6  NULL
{ &PINL, &PORTL, 32, & DDRL }, // 44 DIO44_PIN PINL5  &OCR5CL
{ &PINL, &PORTL, 16, & DDRL }, // 45 DIO45_PIN PINL4  &OCR5BL
{ &PINL, &PORTL, 8, & DDRL }, // 46 DIO46_PIN PINL3  &OCR5AL
{ &PINL, &PORTL, 4, & DDRL }, // 47 DIO47_PIN PINL2  NULL
{ &PINL, &PORTL, 2, & DDRL }, // 48 DIO48_PIN PINL1  NULL
{ &PINL, &PORTL, 1, & DDRL }, // 49 DIO49_PIN PINL0  NULL
{ &PINB, &PORTB, 8, & DDRB }, // 50 DIO50_PIN PINB3  NULL
{ &PINB, &PORTB, 4, & DDRB }, // 51 DIO51_PIN PINB2  NULL
{ &PINB, &PORTB, 2, & DDRB }, // 52 DIO52_PIN PINB1  NULL
{ &PINB, &PORTB, 1, & DDRB }, // 53 DIO53_PIN PINB0  NULL
{ &PINF, &PORTF, 1, & DDRF }, // 54 DIO54_PIN PINF0  NULL
{ &PINF, &PORTF, 2, & DDRF }, // 55 DIO55_PIN PINF1  NULL
{ &PINF, &PORTF, 4, & DDRF }, // 56 DIO56_PIN PINF2  NULL
{ &PINF, &PORTF, 8, & DDRF }, // 57 DIO57_PIN PINF3  NULL
{ &PINF, &PORTF, 16, & DDRF }, // 58 DIO58_PIN PINF4  NULL
{ &PINF, &PORTF, 32, & DDRF }, // 59 DIO59_PIN PINF5  NULL
{ &PINF, &PORTF, 64, & DDRF }, // 60 DIO60_PIN PINF6  NULL
{ &PINF, &PORTF, 128, & DDRF }, // 61 DIO61_PIN PINF7  NULL
{ &PINK, &PORTK, 1, & DDRK }, // 62 DIO62_PIN PINK0  NULL
{ &PINK, &PORTK, 2, & DDRK }, // 63 DIO63_PIN PINK1  NULL
{ &PINK, &PORTK, 4, & DDRK }, // 64 DIO64_PIN PINK2  NULL
{ &PINK, &PORTK, 8, & DDRK }, // 65 DIO65_PIN PINK3  NULL
{ &PINK, &PORTK, 16, & DDRK }, // 66 DIO66_PIN PINK4  NULL
{ &PINK, &PORTK, 32, & DDRK }, // 67 DIO67_PIN PINK5  NULL
{ &PINK, &PORTK, 64, & DDRK }, // 68 DIO68_PIN PINK6  NULL
{ &PINJ, &PORTJ, 64, & DDRJ }, // 69 DIO69_PIN PINJ6  NULL
};

////////////////////////////////////////////////////////////////////////////////////////////////////

template <uint8_t PIN>
struct AvrPin {
    static void setOutput() {
        volatile uint8_t *ddr = avrPinMap[PIN].ddrAddr;
        uint8_t bitMask = avrPinMap[PIN].bitMask;
        CRITICAL_SECTION_START
        *ddr = *ddr | bitMask;
        CRITICAL_SECTION_END
    };
    static void setInput() {
        volatile uint8_t *ddr = avrPinMap[PIN].ddrAddr;
        uint8_t bitMask = avrPinMap[PIN].bitMask;
        CRITICAL_SECTION_START
        *ddr = *ddr & ~bitMask;
        CRITICAL_SECTION_END
        outputLow(); // deactivate pullup
    };
    static void outputLow() {
        volatile uint8_t *port = avrPinMap[PIN].portAddr;
        uint8_t bitMask = avrPinMap[PIN].bitMask;
        CRITICAL_SECTION_START
        *port = *port & ~bitMask;
        CRITICAL_SECTION_END
    };
    static void outputHigh() {
        volatile uint8_t *port = avrPinMap[PIN].portAddr;
        uint8_t bitMask = avrPinMap[PIN].bitMask;
        CRITICAL_SECTION_START
        *port = *port | bitMask;
        CRITICAL_SECTION_END
    };
};

template <uint8_t PIN>
struct DigitalOutput<PIN, ACTIVEHIGHPIN>: AvrPin<PIN> {
    static void initDeActive() { AvrPin<PIN>::setOutput(); deActivate(); }
    static void activate() { AvrPin<PIN>::outputHigh(); };
    static void deActivate() { AvrPin<PIN>::outputLow(); };
    static void saveState() { AvrPin<PIN>::setInput(); }
    /*
    static void initActive() { _SET_OUTPUT(PIN) ; activate(); }
    // static void write(uint8_t v) { _WRITE_NC(PIN, v); }
    */
};

template <uint8_t PIN>
struct DigitalOutput<PIN, ACTIVELOWPIN> {
    static void initDeActive() { AvrPin<PIN>::setOutput(); deActivate(); }
    static void activate() { AvrPin<PIN>::outputLow(); };
    static void deActivate() { AvrPin<PIN>::outputHigh(); };
    /*
    static void initActive() { _SET_OUTPUT(PIN) ; activate(); }
    static void saveState() { myPinMode(PIN, INPUT_FLOATING); }
    // static void write(uint8_t v) { _WRITE_NC(PIN, v); }
    */
};

// note: pullup für avr durch high-write auf input: WRITE(X_STOP_PIN,HIGH); 
template <uint8_t PIN, typename ACTIVEHIGH>
struct DigitalInput { };

template <uint8_t PIN>
struct DigitalInput<PIN, ACTIVELOWPIN>: AvrPin<PIN> {
    static void init() { 
        // Leave pin at reset-state input and activate pullup
        // by writing high to input.
        AvrPin<PIN>::outputHigh();
    }
    static bool active() { return digitalRead(PIN) == LOW; }
};

#if 0
template <uint8_t PIN, WiringPinMode PM>
struct DigitalInput<PIN, PM, ACTIVEHIGHPIN>: DigitalInputBase<PIN, PM> {
    static bool active() { return digitalRead(PIN); }
};

// #define PWM_WRITE(p, v) pwmWrite(p, map(v, 0, 255, 0, 65535))
#endif

template <uint8_t PIN>
struct PWMOutput<PIN, ACTIVEHIGHPIN>: AvrPin<PIN> {
    static void init() { AvrPin<PIN>::setOutput(); }
    static void write(uint8_t v) { analogWrite(PIN, v); }
    static void saveState() { AvrPin<PIN>::setInput(); }
};

#if 0
template <uint8_t PIN>
struct PWMOutput<PIN, ACTIVELOWPIN>: AvrPin<PIN> {
    static void init() { pwmInit(PIN, 0, true); }
    static void write(uint8_t v) { pwmWrite(PIN, map(v, 0, 255, 0, 65535)); }
    static void saveState() { myPinMode(PIN, INPUT_FLOATING); }
};
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////




