/************************************************************************************************
* Note by erwin.rieger@ibrieger.de:
* This file is part of ddprint - a direct drive 3d printer firmware.
* The Origin of this code is Ultimaker2Marlin (https://github.com/Ultimaker/Ultimaker2Marlin).
************************************************************************************************/

/*****************************************************************
* Ultiboard v2.0 pin assignment
******************************************************************/

// Note: Pin nubers are the logical/mapped arduino pins, not the physical chip pins.

#define X_STEP_PIN              FastDigitalOutput<PORTADDR(PORTA), 3, ACTIVEHIGHPIN>  /* Arduino pin 25, PA3 */
// Active state is forward direction (positive)
#define X_DIR_PIN               FastDigitalOutput<PORTADDR(PORTA), 1, ACTIVELOWPIN>  /* Arduino pin 23, PA1 */
#define X_STOP_PIN              DigitalInput<22, ACTIVELOWPIN>  // todo: speedup
#define X_ENABLE_PIN            DigitalOutput<27, ACTIVELOWPIN>

#define Y_STEP_PIN              FastDigitalOutput<PORTADDR(PORTC), 5, ACTIVEHIGHPIN>  /* Arduino pin 32, PC5 */
// Active state is forward direction (positive)
#define Y_DIR_PIN               FastDigitalOutput<PORTADDR(PORTC), 4, ACTIVEHIGHPIN>  /* Arduino pin 33, PC4 */
#define Y_STOP_PIN              DigitalInput<26, ACTIVELOWPIN>  // todo: speedup
#define Y_ENABLE_PIN            DigitalOutput<31, ACTIVELOWPIN>

#define Z_STEP_PIN              FastDigitalOutput<PORTADDR(PORTC), 2, ACTIVEHIGHPIN>  /* Arduino pin 35, PC2 */
// Active state is forward direction (positive)
#define Z_DIR_PIN               FastDigitalOutput<PORTADDR(PORTC), 1, ACTIVELOWPIN>  /* Arduino pin 36, PC1 */
#define Z_STOP_PIN              DigitalInput<29, ACTIVELOWPIN>  // todo: speedup
#define Z_ENABLE_PIN            DigitalOutput<34, ACTIVELOWPIN>

#define E0_STEP_PIN             FastDigitalOutput<PORTADDR(PORTL), 7, ACTIVEHIGHPIN>  /* Arduino pin 42, PL7 */
// Active state is forward direction (positive)
#define E0_DIR_PIN               FastDigitalOutput<PORTADDR(PORTL), 6, ACTIVELOWPIN>  /* Arduino pin 43, PL6 */
#define E0_ENABLE_PIN           DigitalOutput<37, ACTIVELOWPIN>

// Second extruder not used 
//#define E1_STEP_PIN         49
// Active state is forward direction (positive)
//#define E1_DIR_PIN          47
//#define E1_ENABLE_PIN       48
//#define E1_ENABLE_ACTIVE    LOW

#define HEATER_BED_PIN          DigitalOutput<4, ACTIVEHIGHPIN>

#define HEATER_0_PIN            PWMOutput<2, ACTIVEHIGHPIN>
#define HEATER_1_PIN            PWMOutput<3, ACTIVEHIGHPIN>
// #define TEMP_1_PIN 9


#define TEMP_BED_PIN            10
#define TEMP_0_PIN              8

// SPI
// Chip select SDCard
#define SDSS                    DigitalOutput<53, ACTIVELOWPIN> 
#define SCK_PIN                 DigitalOutput<52, ACTIVEHIGHPIN> 
// #define MISO_PIN                DigitalInput<50...
#define MOSI_PIN                DigitalOutput<51, ACTIVEHIGHPIN> 

#define LED_PIN                 PWMOutput<8, ACTIVEHIGHPIN>

#define FAN_PIN                 PWMOutput<7, ACTIVEHIGHPIN> 

#define HOTEND_FAN_PIN          DigitalOutput<69, ACTIVEHIGHPIN>       // Small fan behind hotend cooler, software switchable on newer UM2 boards.

// #define POWER_SUPPLY_RELAY 24
#define SAFETY_TRIGGERED_PIN     28 //PIN to detect the safety circuit has triggered
#define MAIN_VOLTAGE_MEASURE_PIN 14 //Analogue PIN to measure the main voltage, with a 100k - 4k7 resitor divider.

#define MOTOR_CURRENT_PWM_XY_PIN    PWMOutput<44, ACTIVEHIGHPIN>
#define MOTOR_CURRENT_PWM_Z_PIN     PWMOutput<45, ACTIVEHIGHPIN>
#define MOTOR_CURRENT_PWM_E_PIN     PWMOutput<46, ACTIVEHIGHPIN>
//Motor current PWM conversion, PWM value = MotorCurrentSetting * 255 / range
#define MOTOR_CURRENT_PWM_RANGE 2000

//
// Filament sensor pins (half duplex/3wire spi)
//
// A5020
// #define FILSENSNCS   65
// A7550
#define FILSENSNCS              DigitalOutput<30, ACTIVELOWPIN> 
// #define FILSENSSDIO  66
// #define FILSENSSCLK  67


