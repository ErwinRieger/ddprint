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

#define WDT_ENABLE() wdt_enable(WDTO_4S) /* Timeout 4 seconds */
#define WDT_RESET() wdt_reset()

#define TIMER_INIT() /* */


////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct AvrPinMapInfo {
    volatile uint8_t * pinAddr;
    volatile uint8_t * portAddr;
    uint8_t  bit;
    volatile uint8_t * ddrAddr;
    volatile uint8_t *pwmAddr;
} AvrPinMapInfo;

const AvrPinMapInfo avrPinMap[] =
{

{ &PINE, &PORTE, 0, & DDRE,  NULL }, // 0 DIO0_PIN PINE0
{ &PINE, &PORTE, 1, & DDRE,  NULL }, // 1 DIO1_PIN PINE1
{ &PINE, &PORTE, 4, & DDRE,  &OCR3BL }, // 2 DIO2_PIN PINE4
{ &PINE, &PORTE, 5, & DDRE,  &OCR3CL }, // 3 DIO3_PIN PINE5
{ &PING, &PORTG, 5, & DDRG,  &OCR0B }, // 4 DIO4_PIN PING5
{ &PINE, &PORTE, 3, & DDRE,  &OCR3AL }, // 5 DIO5_PIN PINE3
{ &PINH, &PORTH, 3, & DDRH,  &OCR4AL }, // 6 DIO6_PIN PINH3
{ &PINH, &PORTH, 4, & DDRH,  &OCR4BL }, // 7 DIO7_PIN PINH4
{ &PINH, &PORTH, 5, & DDRH,  &OCR4CL }, // 8 DIO8_PIN PINH5
{ &PINH, &PORTH, 6, & DDRH,  &OCR2B }, // 9 DIO9_PIN PINH6
{ &PINB, &PORTB, 4, & DDRB,  &OCR2A }, // 10 DIO10_PIN PINB4
{ &PINB, &PORTB, 5, & DDRB,  NULL }, // 11 DIO11_PIN PINB5
{ &PINB, &PORTB, 6, & DDRB,  NULL }, // 12 DIO12_PIN PINB6
{ &PINB, &PORTB, 7, & DDRB,  &OCR0A }, // 13 DIO13_PIN PINB7
{ &PINJ, &PORTJ, 1, & DDRJ,  NULL }, // 14 DIO14_PIN PINJ1
{ &PINJ, &PORTJ, 0, & DDRJ,  NULL }, // 15 DIO15_PIN PINJ0
{ &PINH, &PORTH, 1, & DDRH,  NULL }, // 16 DIO16_PIN PINH1
{ &PINH, &PORTH, 0, & DDRH,  NULL }, // 17 DIO17_PIN PINH0
{ &PIND, &PORTD, 3, & DDRD,  NULL }, // 18 DIO18_PIN PIND3
{ &PIND, &PORTD, 2, & DDRD,  NULL }, // 19 DIO19_PIN PIND2
{ &PIND, &PORTD, 1, & DDRD,  NULL }, // 20 DIO20_PIN PIND1
{ &PIND, &PORTD, 0, & DDRD,  NULL }, // 21 DIO21_PIN PIND0
{ &PINA, &PORTA, 0, & DDRA,  NULL }, // 22 DIO22_PIN PINA0
{ &PINA, &PORTA, 1, & DDRA,  NULL }, // 23 DIO23_PIN PINA1
{ &PINA, &PORTA, 2, & DDRA,  NULL }, // 24 DIO24_PIN PINA2
{ &PINA, &PORTA, 3, & DDRA,  NULL }, // 25 DIO25_PIN PINA3
{ &PINA, &PORTA, 4, & DDRA,  NULL }, // 26 DIO26_PIN PINA4
{ &PINA, &PORTA, 5, & DDRA,  NULL }, // 27 DIO27_PIN PINA5
{ &PINA, &PORTA, 6, & DDRA,  NULL }, // 28 DIO28_PIN PINA6
{ &PINA, &PORTA, 7, & DDRA,  NULL }, // 29 DIO29_PIN PINA7
{ &PINC, &PORTC, 7, & DDRC,  NULL }, // 30 DIO30_PIN PINC7
{ &PINC, &PORTC, 6, & DDRC,  NULL }, // 31 DIO31_PIN PINC6
{ &PINC, &PORTC, 5, & DDRC,  NULL }, // 32 DIO32_PIN PINC5
{ &PINC, &PORTC, 4, & DDRC,  NULL }, // 33 DIO33_PIN PINC4
{ &PINC, &PORTC, 3, & DDRC,  NULL }, // 34 DIO34_PIN PINC3
{ &PINC, &PORTC, 2, & DDRC,  NULL }, // 35 DIO35_PIN PINC2
{ &PINC, &PORTC, 1, & DDRC,  NULL }, // 36 DIO36_PIN PINC1
{ &PINC, &PORTC, 0, & DDRC,  NULL }, // 37 DIO37_PIN PINC0
{ &PIND, &PORTD, 7, & DDRD,  NULL }, // 38 DIO38_PIN PIND7
{ &PING, &PORTG, 2, & DDRG,  NULL }, // 39 DIO39_PIN PING2
{ &PING, &PORTG, 1, & DDRG,  NULL }, // 40 DIO40_PIN PING1
{ &PING, &PORTG, 0, & DDRG,  NULL }, // 41 DIO41_PIN PING0
{ &PINL, &PORTL, 7, & DDRL,  NULL }, // 42 DIO42_PIN PINL7
{ &PINL, &PORTL, 6, & DDRL,  NULL }, // 43 DIO43_PIN PINL6
{ &PINL, &PORTL, 5, & DDRL,  &OCR5CL }, // 44 DIO44_PIN PINL5
{ &PINL, &PORTL, 4, & DDRL,  &OCR5BL }, // 45 DIO45_PIN PINL4
{ &PINL, &PORTL, 3, & DDRL,  &OCR5AL }, // 46 DIO46_PIN PINL3
{ &PINL, &PORTL, 2, & DDRL,  NULL }, // 47 DIO47_PIN PINL2
{ &PINL, &PORTL, 1, & DDRL,  NULL }, // 48 DIO48_PIN PINL1
{ &PINL, &PORTL, 0, & DDRL,  NULL }, // 49 DIO49_PIN PINL0
{ &PINB, &PORTB, 3, & DDRB,  NULL }, // 50 DIO50_PIN PINB3
{ &PINB, &PORTB, 2, & DDRB,  NULL }, // 51 DIO51_PIN PINB2
{ &PINB, &PORTB, 1, & DDRB,  NULL }, // 52 DIO52_PIN PINB1
{ &PINB, &PORTB, 0, & DDRB,  NULL }, // 53 DIO53_PIN PINB0
{ &PINF, &PORTF, 0, & DDRF,  NULL }, // 54 DIO54_PIN PINF0
{ &PINF, &PORTF, 1, & DDRF,  NULL }, // 55 DIO55_PIN PINF1
{ &PINF, &PORTF, 2, & DDRF,  NULL }, // 56 DIO56_PIN PINF2
{ &PINF, &PORTF, 3, & DDRF,  NULL }, // 57 DIO57_PIN PINF3
{ &PINF, &PORTF, 4, & DDRF,  NULL }, // 58 DIO58_PIN PINF4
{ &PINF, &PORTF, 5, & DDRF,  NULL }, // 59 DIO59_PIN PINF5
{ &PINF, &PORTF, 6, & DDRF,  NULL }, // 60 DIO60_PIN PINF6
{ &PINF, &PORTF, 7, & DDRF,  NULL }, // 61 DIO61_PIN PINF7
{ &PINK, &PORTK, 0, & DDRK,  NULL }, // 62 DIO62_PIN PINK0
{ &PINK, &PORTK, 1, & DDRK,  NULL }, // 63 DIO63_PIN PINK1
{ &PINK, &PORTK, 2, & DDRK,  NULL }, // 64 DIO64_PIN PINK2
{ &PINK, &PORTK, 3, & DDRK,  NULL }, // 65 DIO65_PIN PINK3
{ &PINK, &PORTK, 4, & DDRK,  NULL }, // 66 DIO66_PIN PINK4
{ &PINK, &PORTK, 5, & DDRK,  NULL }, // 67 DIO67_PIN PINK5
{ &PINK, &PORTK, 6, & DDRK,  NULL }, // 68 DIO68_PIN PINK6
{ &PINJ, &PORTJ, 6, & DDRJ,  NULL }, // 69 DIO69_PIN PINJ6

};
////////////////////////////////////////////////////////////////////////////////////////////////////

template <uint8_t PIN>
struct AvrPin {
    static void setOutput() {
        volatile uint8_t *ddr = avrPinMap[PIN].ddrAddr;
        uint8_t bit = avrPinMap[PIN].bit;
        *ddr = *ddr | MASK(bit);
    };
    static void setInput() {
        volatile uint8_t *ddr = avrPinMap[PIN].ddrAddr;
        uint8_t bit = avrPinMap[PIN].bit;
        *ddr = *ddr & ~MASK(bit);
        outputLow(); // deactivate pullup
    };
    static void outputLow() {
        volatile uint8_t *port = avrPinMap[PIN].portAddr;
        uint8_t bit = avrPinMap[PIN].bit;
        *port = *port & ~MASK(bit);
    };
    static void outputHigh() {
        // digitalWrite(PIN, HIGH);
        // return;
        volatile uint8_t *port = avrPinMap[PIN].portAddr;
        uint8_t bit = avrPinMap[PIN].bit;
        *port = *port | MASK(bit);
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




