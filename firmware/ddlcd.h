/*
# This file is part of ddprint - a 3D printer firmware.
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
*/

#pragma once

#include "pins.h"

#if defined(REPRAP_DISCOUNT_SMART_CONTROLLER)

    #include <LiquidCrystal.h> // library for character LCD

    #ifdef REPRAP_DISCOUNT_SMART_CONTROLLER

        #define SDCARDDETECT 49
    #endif

    #define LCDMSGKILL(reason, p1, p2) { \
        lcd.setCursor(0, 0); lcd.print("KILLED:"); \
        lcd.setCursor(0, 1); lcd.print("Reason: "); lcd.print(reason); \
        lcd.setCursor(0, 2); lcd.print("Param1: "); lcd.print(p1); \
        lcd.setCursor(0, 3); lcd.print("Param2: "); lcd.print(p2); }

    #define LCDMSG(p) { lcd.print(p); }
    #define LCDMSGXY(x, y, p) { lcd.setCursor(x, y); lcd.print(p); }

    extern LiquidCrystal lcd;
#else
    #if defined(__amd64__)
        // Simulator
        #define LCDMSGKILL(reason, p1, p2) { \
            std::cout<<"LCDMSGKILL: KILLED!"<<std::endl; \
            std::cout<<"Reason:"<<reason<<std::endl; \
            std::cout<<"Param1:"<<p1<<std::endl; \
            std::cout<<"Param2:"<<p2<<std::endl; }
        #define LCDMSG(p) { lcd.print(p); }
        #define LCDMSGXY(x, y, p) { lcd.setCursor(x, y); lcd.print(p); }
    #else
        #define LCDMSGKILL(reason, p1, p2) /* */
        #define LCDMSG(p) /* */
        #define LCDMSGXY(x, y, p) /* */
    #endif
#endif

