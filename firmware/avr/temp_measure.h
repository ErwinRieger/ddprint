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


    //
    // Start bedtemp measure
    //
    ADCSRB = 1<<MUX5;

    // Set voltage reference to Avcc, set channel to bedtemp
    ADMUX = ((1 << REFS0) | (TEMP_BED_PIN & 0x07));
    ADCSRA |= 1<<ADSC; // Start conversion

    // Wait for conversion and read value
    PT_WAIT_WHILE( ADCSRA & (1<<ADSC) );
    avgBedTemp.addValue(tempFromRawADC(ADC));

    //
    // Start hotend measure
    //
    ADCSRB = 1<<MUX5;

    // Set voltage reference to Avcc, set channel to temp 0
    ADMUX = ((1 << REFS0) | (TEMP_0_PIN & 0x07));
    ADCSRA |= 1<<ADSC; // Start conversion

    // Wait for conversion and read value
    PT_WAIT_WHILE( ADCSRA & (1<<ADSC) );
    avgHotendTemp.addValue(tempFromRawADC(ADC));

    current_temperature_bed = avgBedTemp.value();
    current_temperature[0] = avgHotendTemp.value();

