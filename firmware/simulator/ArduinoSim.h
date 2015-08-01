#/*
# This file is part of ddprint - a direct drive 3D printer firmware.
# 
# Copyright 2015 erwin.rieger@ibrieger.de
# 
# ddprint is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# ddprint is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with ddprint.  If not, see <http://www.gnu.org/licenses/>.
#*/

#pragma once

#if defined(DDSim)

    #include <stdint.h>

    #include <iostream>

    #define HIGH 1
    #define LOW 0

    #define ADEN 0
    #define MUX5 3
    #define ADSC 6
    #define REFS0 6
    #define CS50 0
    #define CS51 0
    #define CS52 0
    #define CS10 0
    #define WGM13 0
    #define WGM12 0
    #define WGM11 0
    #define WGM10 0
    #define COM1A0 0
    #define COM1B0 0
    // #define OCIE1A 0

    #define WDTO_4S 4

    // #define LCD_ALERTMESSAGEPGM(s) printf(s"\n")

    #define PROGMEM /**/
    #define  PSTR(x) x
    #define  pgm_read_byte(p) (*(p))
    #define _BV(x) x

    #define ISR(x) void ISR##x()

    #define INPUT 0
    #define OUTPUT 1

    extern int DIDR0;
    extern int DIDR2;
    extern int ADCSRA;
    extern int ADCSRB;
    extern int ADMUX;
    extern int TCCR5B;
    extern int TCCR1B;
    extern int TCCR1A;
    extern int OCR1A;
    extern int TCNT1;
    extern int TIMSK0;
    extern int TIMSK1;

    //extern uint8_t MCUSR;

    // Temp 0
    extern int ADC;

    uint16_t pgm_read_word(const void* ptr);
    typedef std::string String;

    void SET_OUTPUT(int pin);

    void WRITE(int pin, uint8_t v);
    uint8_t READ(int pin);

    void analogWrite(int pin, uint8_t m);
    void wdt_enable(uint8_t /* wdt_time */);


    void cli();
    void sei();
    void SET_INPUT(int pin);

    void pinMode(int pin, uint8_t m);

    unsigned long millis();


    // #include <hardware/arduino/cores/arduino/Arduino.h>
#endif

