/************************************************************************************************
* Note by erwin.rieger@ibrieger.de:
* This file is part of ddprint - a direct drive 3d printer firmware.
* The Origin of this code is Ultimaker2Marlin (https://github.com/Ultimaker/Ultimaker2Marlin).
************************************************************************************************/

//
// Ramps 1.4
//

#define X_STEP_PIN         DigitalOutput<54, ACTIVEHIGHPIN>
#define X_DIR_PIN          DigitalOutput<55, ACTIVELOWPIN>
#define X_STOP_PIN         DigitalInput<3, ACTIVELOWPIN> /* 3 */
#define X_ENABLE_PIN       DigitalOutput<38, ACTIVELOWPIN> /* 38 */

/*
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_STOP_PIN         15
*/
#define Y_STEP_PIN         DigitalOutput<60, ACTIVEHIGHPIN>
#define Y_DIR_PIN          DigitalOutput<61, ACTIVELOWPIN>
#define Y_STOP_PIN         DigitalInput<15, ACTIVELOWPIN>
#define Y_ENABLE_PIN       DigitalOutput<56, ACTIVELOWPIN>

/*
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_STOP_PIN         19
*/
#define Z_STEP_PIN              DigitalOutput<46, ACTIVEHIGHPIN>
#define Z_DIR_PIN               DigitalOutput<48, ACTIVELOWPIN>
#define Z_STOP_PIN              DigitalInput<19, ACTIVELOWPIN>
#define Z_ENABLE_PIN            DigitalOutput<62, ACTIVELOWPIN>

/*
    #define Z2_STEP_PIN        36
    #define Z2_DIR_PIN         34
    #define Z2_ENABLE_PIN      30
*/

/*
    #define E0_STEP_PIN        26
    #define E0_DIR_PIN         28
    #define E0_ENABLE_PIN      24
*/
#define E0_STEP_PIN             DigitalOutput<26, ACTIVEHIGHPIN>
#define E0_DIR_PIN              DigitalOutput<28, ACTIVELOWPIN>
#define E0_ENABLE_PIN           DigitalOutput<24, ACTIVELOWPIN>


/*
    #define E1_STEP_PIN        36
    #define E1_DIR_PIN         34
    #define E1_ENABLE_PIN      30
*/


#define HEATER_BED_PIN     DigitalOutput<8, ACTIVEHIGHPIN>    // BED

#define HEATER_0_PIN        PWMOutput<10, ACTIVEHIGHPIN>   // EXTRUDER 1
#define HEATER_1_PIN        PWMOutput<10, ACTIVEHIGHPIN>   // EXTRUDER 2

  // #define HEATER_2_PIN       -1

  #define TEMP_0_PIN         13   // ANALOG NUMBERING
  // xxx messed up wiring, T1 and T2 swapped
  // #define TEMP_1_PIN         15   // ANALOG NUMBERING
  #define TEMP_1_PIN         14   // ANALOG NUMBERING
  #define TEMP_2_PIN         -1   // ANALOG NUMBERING

  // xxx messed up wiring, T1 and T2 swapped
  // #define TEMP_BED_PIN       14   // ANALOG NUMBERING
  #define TEMP_BED_PIN       15   // ANALOG NUMBERING





// SPI
// Chip select SDCard
#define SDSS               DigitalOutput<53, ACTIVELOWPIN>
    #define SCK_PIN        DigitalOutput<52, ACTIVEHIGHPIN>
// #define MISO_PIN           DigitalInput<50...
    #define MOSI_PIN           DigitalOutput<51, ACTIVEHIGHPIN>


// #define LED_PIN_ACTIVE_LOW false
#define LED_PIN            PWMOutput<13, ACTIVEHIGHPIN>

// #define FAN_PIN_ACTIVE_LOW false
#define FAN_PIN            PWMOutput<9, ACTIVEHIGHPIN>

  // #define POWER_SUPPLY_RELAY  12
  //
//
// Filament sensor pins (half duplex/3wire spi)
//
#define FILSENSNCS   DigitalOutput<59, ACTIVELOWPIN> // Analog A5

// #define FILSENSSDIO  63
// #define FILSENSMISO  50
// #define FILSENSMOSI  51
// #define FILSENSSCLK  52

#ifdef REPRAP_DISCOUNT_SMART_CONTROLLER

    #define LCD_PINS_RS 16
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 23
    #define LCD_PINS_D5 25
    #define LCD_PINS_D6 27
    #define LCD_PINS_D7 29

    #define BEEPER 37

    #define BTN_EN1 31
    #define BTN_EN2 33
    #define BTN_ENC 35
#endif

// attic
#if 0

//////////////////FIX THIS//////////////
#ifndef __AVR_ATmega1280__
 #ifndef __AVR_ATmega2560__
 #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
 #endif
#endif


// uncomment one of the following lines for RAMPS v1.3 or v1.0, comment both for v1.2 or 1.1
// #define RAMPS_V_1_3
// #define RAMPS_V_1_0

#if MOTHERBOARD == 2

#endif // MOTHERBOARD == 2






#endif













