
#pragma once

#include "pins_ramps.h"

#if defined(REPRAP_DISCOUNT_SMART_CONTROLLER)
    #define LCDMSGKILL(reason, p1, p2) { \
        lcd.setCursor(0, 0); lcd.print("KILLED:"); \
        lcd.setCursor(0, 1); lcd.print("Reason: "); lcd.print(reason); \
        lcd.setCursor(0, 2); lcd.print("Param1: "); lcd.print(p1); \
        lcd.setCursor(0, 3); lcd.print("Param2: "); lcd.print(p2); }
#else
    #define LCDMSGKILL(reason, p1, p2) /* */
#endif

#if defined(REPRAP_DISCOUNT_SMART_CONTROLLER)

#include <LiquidCrystal.h> // library for character LCD

extern LiquidCrystal lcd;

#endif

