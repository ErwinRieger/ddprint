/************************************************************************************************
* Note by erwin.rieger@ibrieger.de:
* This file is part of ddprint - a direct drive 3d printer firmware.
* The Origin of this code is Ultimaker2Marlin (https://github.com/Ultimaker/Ultimaker2Marlin).
************************************************************************************************/

#pragma once
//
// Ramps 1.4
//

#if MOTHERBOARD == 3 || MOTHERBOARD == 33 || MOTHERBOARD == 34 || MOTHERBOARD == 77
#define KNOWN_BOARD 1

//////////////////FIX THIS//////////////
#ifndef __AVR_ATmega1280__
 #ifndef __AVR_ATmega2560__
 #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
 #endif
#endif


// uncomment one of the following lines for RAMPS v1.3 or v1.0, comment both for v1.2 or 1.1
// #define RAMPS_V_1_3
// #define RAMPS_V_1_0

#if MOTHERBOARD == 33 
    #define REPRAP_DISCOUNT_SMART_CONTROLLER
    #define NEWPANEL
    #define ULTRA_LCD
#endif

#if MOTHERBOARD == 33 || MOTHERBOARD == 34 || MOTHERBOARD == 77

  #define LARGE_FLASH true

    #define X_STEP_PIN         54
    #define X_DIR_PIN          55
    #define X_ENABLE_PIN       38
    // #define X_MIN_PIN           3
    #define X_STOP_PIN          3
    #define X_MAX_PIN           2


    #define Y_STEP_PIN         60
    #define Y_DIR_PIN          61
    #define Y_ENABLE_PIN       56
    #define Y_MIN_PIN          14
    // #define Y_MAX_PIN          15
    #define Y_STOP_PIN         15

    #define Z_STEP_PIN         46
    #define Z_DIR_PIN          48
    #define Z_ENABLE_PIN       62
    #define Z_MIN_PIN          18
    // #define Z_MAX_PIN          19
    #define Z_STOP_PIN         19

    #define Z2_STEP_PIN        36
    #define Z2_DIR_PIN         34
    #define Z2_ENABLE_PIN      30

    #define E0_STEP_PIN        26
    #define E0_DIR_PIN         28
    #define E0_ENABLE_PIN      24

    #define E1_STEP_PIN        36
    #define E1_DIR_PIN         34
    #define E1_ENABLE_PIN      30

    #define SDPOWER            -1
    // Chip select SDCard
    #define SDSS               53
    #define LED_PIN            13

    #define FAN_PIN            9 // (Sprinter config)

  #define PS_ON_PIN          12

  #if defined(REPRAP_DISCOUNT_SMART_CONTROLLER) || defined(G3D_PANEL)
    #define KILL_PIN           41
  #else
    #define KILL_PIN           -1
  #endif

  #define HEATER_0_PIN       10   // EXTRUDER 1
    #define HEATER_1_PIN       -1
  #define HEATER_2_PIN       -1

  #if MOTHERBOARD == 77
    #define HEATER_0_PIN       10
    #define HEATER_1_PIN       12
    #define HEATER_2_PIN       6
  #endif

  #define TEMP_0_PIN         13   // ANALOG NUMBERING
  // xxx messed up wiring, T1 and T2 swapped
  // #define TEMP_1_PIN         15   // ANALOG NUMBERING
  #define TEMP_1_PIN         14   // ANALOG NUMBERING
  #define TEMP_2_PIN         -1   // ANALOG NUMBERING
  #if MOTHERBOARD == 77
    #define HEATER_BED_PIN     9    // BED
  #else
    #define HEATER_BED_PIN     8    // BED
  #endif
  // xxx messed up wiring, T1 and T2 swapped
  // #define TEMP_BED_PIN       14   // ANALOG NUMBERING
  #define TEMP_BED_PIN       15   // ANALOG NUMBERING



  #ifdef NUM_SERVOS
    #define SERVO0_PIN         11

    #if NUM_SERVOS > 1
      #define SERVO1_PIN         6
    #endif

    #if NUM_SERVOS > 2
      #define SERVO2_PIN         5
    #endif

    #if NUM_SERVOS > 3
      #define SERVO3_PIN         4
    #endif
  #endif

  #ifdef ULTRA_LCD

    #ifdef NEWPANEL
      #define LCD_PINS_RS 16
      #define LCD_PINS_ENABLE 17
      #define LCD_PINS_D4 23
      #define LCD_PINS_D5 25
      #define LCD_PINS_D6 27
      #define LCD_PINS_D7 29

      #ifdef REPRAP_DISCOUNT_SMART_CONTROLLER
        #define BEEPER 37

        #define BTN_EN1 31
        #define BTN_EN2 33
        #define BTN_ENC 35

        #define SDCARDDETECT 49
      #else
        //arduino pin which triggers an piezzo beeper
        #define BEEPER 33  // Beeper on AUX-4

        //buttons are directly attached using AUX-2
        #ifdef REPRAPWORLD_KEYPAD
          #define BTN_EN1 64 // encoder
          #define BTN_EN2 59 // encoder
          #define BTN_ENC 63 // enter button
          #define SHIFT_OUT 40 // shift register
          #define SHIFT_CLK 44 // shift register
          #define SHIFT_LD 42 // shift register
        #else
          #define BTN_EN1 37
          #define BTN_EN2 35
          #define BTN_ENC 31  //the click
        #endif

        #ifdef G3D_PANEL
          #define SDCARDDETECT 49
        #else
          #define SDCARDDETECT -1  // Ramps does not use this port
        #endif

      #endif

      #if MOTHERBOARD == 77
        #define BEEPER -1

        #define LCD_PINS_RS 27
        #define LCD_PINS_ENABLE 29
        #define LCD_PINS_D4 37
        #define LCD_PINS_D5 35
        #define LCD_PINS_D6 33
        #define LCD_PINS_D7 31

       //buttons
       #define BTN_EN1 16
       #define BTN_EN2 17
       #define BTN_ENC 23 //the click

    #endif
    #else //old style panel with shift register
      //arduino pin witch triggers an piezzo beeper
      #define BEEPER 33		//No Beeper added

      //buttons are attached to a shift register
	// Not wired this yet
      //#define SHIFT_CLK 38
      //#define SHIFT_LD 42
      //#define SHIFT_OUT 40
      //#define SHIFT_EN 17

      #define LCD_PINS_RS 16
      #define LCD_PINS_ENABLE 17
      #define LCD_PINS_D4 23
      #define LCD_PINS_D5 25
      #define LCD_PINS_D6 27
      #define LCD_PINS_D7 29
    #endif
  #endif //ULTRA_LCD

#endif // MOTHERBOARD == 33 || MOTHERBOARD == 34 || MOTHERBOARD == 77

// SPI for Max6675 Thermocouple

#ifndef SDSUPPORT
// these pins are defined in the SD library if building with SD support
  // #define SCK_PIN          52
  // #define MISO_PIN         50
  // #define MOSI_PIN         51
  #define MAX6675_SS       53
#else
  #define MAX6675_SS       49
#endif

//
// Filament sensor pins (half duplex/3wire spi)
//
#define FILSENSNCS   59 // Analog A5
// #define FILSENSSDIO  63
#define FILSENSMISO  50
#define FILSENSMOSI  51
#define FILSENSSCLK  52

#endif //MOTHERBOARD == 3 || MOTHERBOARD == 33 || MOTHERBOARD == 34 || MOTHERBOARD == 77

