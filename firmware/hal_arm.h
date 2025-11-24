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

/*
 * State machines:
 *  dd_USBH_MSC_HandleBOTXfer():  usbh_msc.BOTState (USBH_BOTSTATE_SENT_CBW)
 *  USBH_MSC_TestUnitReady():     usbh_msc.CmdStateMachine CMD_SEND_STATE
 */

#include <Arduino.h>
#include <libmaple/iwdg.h>

#include "mdebug.h"
#include "pin_states.h"
#include "massstoragebase.h"

#include "usbh_msc/dd_usbh_msc.h"
#include "usbh_msc/usbh_msc_bot.h"

//
// STM32
//

////////////////////////////////////////////////////////////////////////////////////////////////////

extern "C" {
    extern void systemHardReset(void);
}


//
// Serial interface, note: fixed USART1 usage
//
#define SERIAL_TX_DR_EMPTY() ( USART1->regs->SR & USART_SR_TXE ) 
#define SERIAL_TX_COMPLETE() ( USART1->regs->SR & USART_SR_TC )
#define SERIAL_TX_DR_PUTC(c) ( USART1->regs->DR = c )

// #define SET_INPUT(pin) pinMode(pin, INPUT_FLOATING)
// #define SET_INPUT_PD(pin) pinMode(pin, INPUT_PULLDOWN)
#define HAL_SET_INPUT_PU(pin) myPinMode(pin, INPUT_PULLUP)
#define HAL_SET_INPUT_ANALOG(pin) pinMode(pin, INPUT_ANALOG)

#define HAL_SET_OUTPUT(pin)  myPinMode(pin, OUTPUT)
// #define SET_OUTPUT_PWM(pin)  pinMode(pin, PWM)
// #define SET_OUTPUT_PWM(pin, activeLow)  pwmInit(pin, 0, activeLow)

#define HAL_READ(pin) digitalRead(pin)
#define HAL_READ_ANALOG(pin) analogRead(pin)

#define HAL_WRITE(pin, v)  digitalWrite(pin, v)
// #define PWM_WRITE(p, v) pwmWrite(p, map(v, 0, 255, 0, 65535))

#define CLI()   noInterrupts()
#define SEI()   interrupts()

#define CRITICAL_SECTION_START  noInterrupts() /* uint32_t primask = __get_PRIMASK(); __disable_irq() */
#define CRITICAL_SECTION_END    interrupts() /* if (!primask) __enable_irq() */

#define WDT_ENABLE() iwdg_init(IWDG_PRE_16, 10000) /* Timeout: 10000 / (40khz/16) -> 4 seconds */
#define WDT_RESET() iwdg_feed()

#define HAL_SYSTEM_RESET() systemHardReset();

#define TIMER_INIT() timerInit()

// TIMER_CC1_INTERRUPT
// #define ENABLE_STEPPER_DRIVER_INTERRUPT()  { timer_enable_irq(&timer2, TIMER_UPDATE_INTERRUPT); timer_enable_irq(&timer2, TIMER_CC1_INTERRUPT); }
// #define DISABLE_STEPPER_DRIVER_INTERRUPT() { timer_disable_irq(&timer2, TIMER_UPDATE_INTERRUPT); timer_disable_irq(&timer2, TIMER_CC1_INTERRUPT);}
#define ENABLE_STEPPER_DRIVER_INTERRUPT()  { timer_enable_irq(&timer2, TIMER_UPDATE_INTERRUPT); }
#define DISABLE_STEPPER_DRIVER_INTERRUPT() { timer_disable_irq(&timer2, TIMER_UPDATE_INTERRUPT); }

#define ENABLE_STEPPER1_DRIVER_INTERRUPT()  { timer_enable_irq(&timer3, TIMER_UPDATE_INTERRUPT); }
#define DISABLE_STEPPER1_DRIVER_INTERRUPT() { timer_disable_irq(&timer3, TIMER_UPDATE_INTERRUPT); }

// Test timer3 interrupt bit in timer interrupt enable register(s)
#define STEPPER1_DRIVER_INTERRUPT_ENABLED() ( timer3.regs.gen->DIER & TIMER_SR_UIF )

#define HAL_SET_STEPPER_TIMER(timerval) { timer_set_reload(&timer2, timerval); }
#define HAL_SET_HOMING_TIMER(timerval) { timer_set_reload(&timer3, timerval); }

#define HAL_SPI_INIT() spiInit()

#define HAL_IRQ_INIT() irqInit()

#define HAL_FS_SPI_SETTINGS /* */

#define HAL_FILSENSOR_BEGINTRANS() /* */

#define HAL_FILSENSOR_READ() filsensorRead()

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Bit-reverse
//
FWINLINE uint32_t reverseBits(uint32_t v)
{
    uint32_t reversed;

    asm volatile ("rbit %0, %1" : "=r" (reversed) : "r" (v) );
    return(reversed);
}
////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 * Derived from (the disabled) stm32duino:timerDefaultConfig().
 * This possibly only works if there is no timersetup in arduino:init().
 */
void pwmInit(uint8_t pwmpin, uint16_t duty=0, bool activeLow=false);

/*
 *
 * Re-implement pinMode() from stm32duino() since it has a problem with unintended timer-deactivation
 */
void myPinMode(uint8 pin, WiringPinMode mode);

////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Initialize the used timers
//
void timerInit();

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Initialize interrupt prio's
//
void irqInit();

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Read byte from *spi-like* USART2
//
uint8_t filsensorRead();

////////////////////////////////////////////////////////////////////////////////////////////////////

#if 0
void RCC_DeInit()
{

  /* Reset CFGR register, switch to HSI clock */
  RCC_BASE->CFGR = 0x0;

  // Wait till switch to HSI is done
  while (RCC_BASE->CFGR & 0xC) ;

  RCC_BASE->CR = (RCC_BASE->CR & 0xFF00) | 0x83;

  /* Reset PLLCFGR register */
  RCC_BASE->PLLCFGR = 0x24003010;
}
#endif

/**
 * Function to perform jump to system memory boot from user application
 *
 * Call function when you want to jump to system memory
 */
void JumpToBootloader();

////////////////////////////////////////////////////////////////////////////////////////////////////

void HAL_SETUP_TEMP_ADC();

////////////////////////////////////////////////////////////////////////////////////////////////////

template <uint8_t PIN>
struct DigitalOutput<PIN, ACTIVEHIGHPIN> {
    static void initActive() { myPinMode(PIN, OUTPUT) ; activate(); }
    static void initDeActive() {
        // xxx todo avoid short active state,
        // see AVR DigitalOutput.
        myPinMode(PIN, OUTPUT) ; deActivate(); }
    static void activate() { digitalWrite(PIN, HIGH); };
    static void deActivate() { digitalWrite(PIN, LOW); };
    static void saveState() { myPinMode(PIN, INPUT_FLOATING); }
    // static void write(uint8_t v) { digitalWrite(PIN, v); }
};

template <uint8_t PIN>
struct DigitalOutput<PIN, ACTIVELOWPIN> {
    static void initActive() { myPinMode(PIN, OUTPUT) ; activate(); }
    static void initDeActive() { myPinMode(PIN, OUTPUT) ; deActivate(); }
    static void activate() { digitalWrite(PIN, LOW); };
    static void deActivate() { digitalWrite(PIN, HIGH); };
    static void saveState() { myPinMode(PIN, INPUT_FLOATING); }
    // static void write(uint8_t v) { digitalWrite(PIN, v); }
};

// note: pullup für avr durch high-write auf input: WRITE(X_STOP_PIN,HIGH); 
// #define SET_INPUT(pin) myPinMode(pin, INPUT_FLOATING)
// #define SET_INPUT_PD(pin) myPinMode(pin, INPUT_PULLDOWN)
// #define SET_INPUT_PU(pin) myPinMode(pin, INPUT_PULLUP)
template <uint8_t PIN, WiringPinMode PM>
struct DigitalInputBase {
    static void init() { myPinMode(PIN, PM); }
};

template <uint8_t PIN, WiringPinMode PM, typename ACTIVEHIGH>
struct DigitalInput { };

template <uint8_t PIN, WiringPinMode PM>
struct DigitalInput<PIN, PM, ACTIVEHIGHPIN>: DigitalInputBase<PIN, PM> {
    static bool active() { return digitalRead(PIN); }
    static bool deActive() { return ! active(); }
};

template <uint8_t PIN, WiringPinMode PM>
struct DigitalInput<PIN, PM, ACTIVELOWPIN>: DigitalInputBase<PIN, PM> {
    static bool active() { return digitalRead(PIN) == LOW; }
    static bool deActive() { return ! active(); }
};

// #define PWM_WRITE(p, v) pwmWrite(p, map(v, 0, 255, 0, 65535))

template <uint8_t PIN>
struct PWMOutput<PIN, ACTIVEHIGHPIN> {
    static void init() { pwmInit(PIN, 0, false); }
    static void write(uint8_t v) { pwmWrite(PIN, map(v, 0, 255, 0, 65535)); }
    static void saveState() { myPinMode(PIN, INPUT_FLOATING); }
};

template <uint8_t PIN>
struct PWMOutput<PIN, ACTIVELOWPIN> {
    static void init() { pwmInit(PIN, 0, true); }
    static void write(uint8_t v) { pwmWrite(PIN, map(v, 0, 255, 0, 65535)); }
    static void saveState() { myPinMode(PIN, INPUT_FLOATING); }
};

// See stm32duino: uint16 analogReadDev(uint8 pin, const adc_dev * dev)
// See stm32duino:uint16 adc_read(const adc_dev *dev, uint8 channel)
template <uint8_t PIN, uint8_t CHANNEL>
struct AnalogInput {
    static void init() { pinMode(PIN, INPUT_ANALOG); }
    static void startConversion() { adc_start_single_convert(ADC1, CHANNEL); }
    static bool conversionDone() { return adc_is_end_of_convert(ADC1); }
    static uint16_t read() { return adc_get_data(ADC1); }
};

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Initialize the spi bus (flowrate sensor)
//

void spiInit();

////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Mass storage interface, USB flash drive in this case.
//
class MassStorage: public MassStorageBase {

  public:

    bool swapInit() {

        dd_USBH_Init(&USB_OTG_Core_Host, USB_OTG_HS_CORE_ID, &USB_Host);
        while (! usbhMscInitialized()) {
            USBH_Process(&USB_OTG_Core_Host, &USB_Host);
        }
        return true;
    }

    // Size of storage in sectors
    uint32_t cardSize() { return usbhMscSizeInBlocks(); }

    // Erase range of blocks
    // Is there a MSC/scsi command to erase/trim blocks and would it
    // speed up things?
    bool erase(uint32_t firstBlock, uint32_t lastBlock) { return true; }

    int errorCode() {
        // Todo
        return 0;
    }

  protected:

    int errorData() {
        // Todo
        return 0;
    }

    //
    // Wrapper around lowlevel read
    // Returns 0 if read is done and ok
    // Returns <0 if read is done and error (return value is error code)
    // Returns 1 if we want to be called again to continue work
    //
    ReadState readBlockWrapper(uint32_t readBlockNumber, uint8_t *dst) {

        USBH_Status status = USBH_MSC_Read10(
                &USB_OTG_Core_Host, &USB_Host,
                dst, readBlockNumber, 512);

        if (status == USBH_BUSY) {
            return Rcontinue; // continue thread
        }

        // Todo: check errors
        // if (status != USBH_OK) ...
        massert(status == USBH_OK);
        
        return Rstop; // OK, write done
    }

    WriteState writeBlock(uint32_t writeBlockNumber, uint8_t *src) {

        USBH_Status status = USBH_MSC_Write10(
                &USB_OTG_Core_Host, &USB_Host,
                src, writeBlockNumber, 512);

        if (status == USBH_BUSY) {
            return Wcontinue; // continue thread
        }

        // Todo: check errors
        // if (status != USBH_OK) ...
        massert(status == USBH_OK);
        
        return Wstop; // OK, write done
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


