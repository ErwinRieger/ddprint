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


#include "Arduino.h"

RegUCSR0A UCSR0A;
RegUDR0 UDR0;

void cli() {};
void sei() {};
void delayMicroseconds(int) { }

uint8_t SREG=0;
uint8_t DDRA = 0;
uint8_t PINA = 0;
uint8_t PORTA = 0;
uint8_t DDRB = 0;
uint8_t PINB = 0;
uint8_t PORTB = 0;
uint8_t DDRC = 0;
uint8_t PINC = 0;
uint8_t PORTC = 0;
uint8_t DDRD = 0;
uint8_t PIND = 0;
uint8_t PORTD = 0;
uint8_t DDRE = 0;
uint8_t PINE = 0;
uint8_t PORTE = 0;
uint8_t DDRF = 0;
uint8_t PINF = 0;
uint8_t PORTF = 0;
uint8_t DDRG = 0;
uint8_t PING = 0;
uint8_t PORTG = 0;
uint8_t DDRH = 0;
uint8_t PINH = 0;
uint8_t PORTH = 0;
uint8_t DDRI = 0;
uint8_t PINI = 0;
uint8_t PORTI = 0;
uint8_t DDRJ = 0;
uint8_t PINJ = 0;
// Port j, Pin 6 is extruder fan
uint8_t PORTJ = 0;
uint8_t DDRK = 0;
uint8_t PINK = 0;
uint8_t PORTK = 0;
uint8_t DDRL = 0;
uint8_t PINL = 0;
uint8_t PORTL = 0;

