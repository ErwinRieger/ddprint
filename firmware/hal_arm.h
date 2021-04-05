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
// xxx combine both flags into one timer_enable_irq call?
// #define ENABLE_STEPPER_DRIVER_INTERRUPT()  { timer_enable_irq(&timer2, TIMER_UPDATE_INTERRUPT); timer_enable_irq(&timer2, TIMER_CC1_INTERRUPT); }
// #define DISABLE_STEPPER_DRIVER_INTERRUPT() { timer_disable_irq(&timer2, TIMER_UPDATE_INTERRUPT); timer_disable_irq(&timer2, TIMER_CC1_INTERRUPT);}
#define ENABLE_STEPPER_DRIVER_INTERRUPT()  { timer_enable_irq(&timer2, TIMER_UPDATE_INTERRUPT); }
#define DISABLE_STEPPER_DRIVER_INTERRUPT() { timer_disable_irq(&timer2, TIMER_UPDATE_INTERRUPT); }

#define ENABLE_STEPPER1_DRIVER_INTERRUPT()  { timer_enable_irq(&timer3, TIMER_UPDATE_INTERRUPT); }
#define DISABLE_STEPPER1_DRIVER_INTERRUPT() { timer_disable_irq(&timer3, TIMER_UPDATE_INTERRUPT); }

// Test timer3 interrupt bit in timer interrupt enable register(s)
#define STEPPER1_DRIVER_INTERRUPT_ENABLED() ( timer3.regs.gen->DIER & TIMER_SR_UIF )

// xxxx min stepper time: (20 / 2) / 2 = 5              ; minsteptime / 2 wegen tv75khz / 2 wegen deactivate step in stepper irq
#define HAL_SET_STEPPER_TIMER(timerval) { massert(timerval < 0xffff); massert(timerval >= 5); timer_set_reload(&timer2, timerval); }
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
inline uint32_t reverseBits(uint32_t v)
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
 * XXX re-implement pinMode() from stm32duino() since it has a problem with unintended timer-deactivation
 */
void myPinMode(uint8 pin, WiringPinMode mode);

////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Initialize the used timers
//
inline void timerInit() {

    // PWM FAN, todo: set prescaler to work around hardware problem
    timer_init(&timer4);

    // PWM LED
    timer_init(&timer5);

    //
    // PWM, hotend 1 heater, default pwm is 1280Hz (with overflow 0xffff)
    // This is too fast for the electronics of the jennyprinter (optocoupler)
    // The pulses are up to 100uS to long so we have to use a low frequency
    // to keep the error small. Prescaler 25 gives us 51 Hz, the error for
    // 100uS is then 0.5%.
    //
    timer_init(&timer8);
    timer_set_prescaler(&timer8, 25 - 1);

    //
    // Stepper timers
    // * timer2: general, 32 bit
    // * timer3: general, 16 bit
    //
    timer_init(&timer2); // Stepper interrupt
    timer_init(&timer3); // Homing stepper interrupt

    timer_pause(&timer2);
    timer_pause(&timer3);

    // Prescaler to achieve 2 mhz clock like mega2560, timer 2+3 are
    // running with APB2 clock of 84Mhz
    // 84 / 2.0 = 42,
    timer_set_prescaler(&timer2, 42 - 1);
    timer_set_prescaler(&timer3, 42 - 1);

    timer_set_count(&timer2, 0);
    timer_set_count(&timer3, 0);

    timer_set_reload(&timer2, 2000); // Set ARR
    timer_set_reload(&timer3, 2000); // Set ARR

    // bb_peri_set_bit(&(timer2.regs.gen)->CR1, TIMER_CR1_ARPE_BIT, 1);
    // bb_peri_set_bit(&(timer2.regs.gen)->CR1, TIMER_CR1_OPM_BIT, 1);

    timer_set_compare(&timer2, 1, 5);          // xxx pulse width for stepper motors

    // bb_peri_set_bit(&(timer3.regs.gen)->CR1, TIMER_CR1_ARPE_BIT, 1);

    nvic_irq_enable(NVIC_TIMER2);
    nvic_irq_enable(NVIC_TIMER3);

    timer_resume(&timer2);
    timer_resume(&timer3);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Initialize interrupt prio's
//
inline void irqInit() {

    //
    // Init interrupt prio's, stm32duino initialized all irq't to prio 16.
    //
    nvic_irq_set_priority(NVIC_TIMER2, 2); // like systick
    nvic_irq_set_priority(NVIC_TIMER3, 2); // like systick

    // XXX NVIC_SPIx ???
    // nvic_irq_set_priority(NVIC_USART1, 2);

    // Done in dd_USBH_Init():
    // nvic_irq_set_priority(NVIC_USB_HS, 3);
    //
    // nvic_irq_set_priority(NVIC_USART1, 3);

    // nvic_irq_set_priority(NVIC_SYSTICK, 3);
    //

// reuse cpu cycles of busy wait in stepper routine, lower prio for stepper than serial and usb:
    nvic_irq_set_priority(NVIC_SYSTICK, 2);
 // nvic_irq_set_priority(NVIC_USB_HS, 3);

    // nvic_irq_set_priority(NVIC_TIMER2, 4);
    // nvic_irq_set_priority(NVIC_TIMER3, 4);
    //
    nvic_irq_set_priority(NVIC_USART1, 3);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Read byte from *spi-like* USART2
//
inline uint8_t filsensorRead() {

    // Clock output
    USART2->regs->DR = 0;
    // Wait until clocked out
    while (! SERIAL_TX_COMPLETE() );
    // Wait until rx byte available
	while (! (USART2->regs->SR & USART_SR_RXNE) );
    // Read rx byte and swap bitwise
    return reverseBits(USART2->regs->DR) >> 24;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#if 0
inline void RCC_DeInit()
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
inline void JumpToBootloader() {

    // xxx move to serial.end()
    nvic_irq_disable(USART1->irq_num);

    /**
    * Step: Disable systick timer and reset it to default values
    */
    /*
    SYSTICK_BASE->RVR = 0;
    while (SYSTICK_BASE->CNT);
    SYSTICK_BASE->CSR = 0;
    */
    SYSTICK_BASE->LOAD = 0;
    while (SYSTICK_BASE->VAL);
    SYSTICK_BASE->CTRL = 0;

    /**
    * Step: Disable RCC, set it to default (after reset) settings
    *       Internal clock, no PLL, etc.
    */
    // RCC_DeInit();
 
    /**
    * Step: Set system memory address. 
    *       
    *       For STM32F429, system memory is on 0x1FFF 0000
    *       For other families, check AN2606 document table 110 with descriptions of memory addresses 
    */
    volatile uint32_t addr = 0x1FFF0000;
 
    void (*SysMemBootJump)(void) = (void (*)(void)) (*((uint32_t *)(addr + 4)));;
 
    /**
    * Step: Set main stack pointer.
    *       This step must be done last otherwise local variables in this function
    *       don't have proper value since stack pointer is located on different position
    *
    *       Set direct address location which specifies stack pointer in SRAM location
    */
    asm volatile ("MSR msp, %0\n" : : "r" (*(uint32_t*)addr) );
 
    /**
    * Step: Actually call our function to jump to set location
    *       This will start system memory execution
    */
    SysMemBootJump();
 
    /**
    * Step: Connect USB<->UART converter to dedicated USART pins and test
    *       and test with bootloader works with STM32 Flash Loader Demonstrator software
    */
    while(1) {};
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void HAL_SETUP_TEMP_ADC();

////////////////////////////////////////////////////////////////////////////////////////////////////

template <uint8_t PIN>
struct DigitalOutput<PIN, ACTIVEHIGHPIN> {
    static void initActive() { myPinMode(PIN, OUTPUT) ; activate(); }
    static void initDeActive() { myPinMode(PIN, OUTPUT) ; deActivate(); }
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
};

template <uint8_t PIN, WiringPinMode PM>
struct DigitalInput<PIN, PM, ACTIVELOWPIN>: DigitalInputBase<PIN, PM> {
    static bool active() { return digitalRead(PIN) == LOW; }
    static bool deActive() { return digitalRead(PIN); }
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
    // Is there a MSC/scsi command to erase blocks and would it
    // speed up things?
    bool erase(uint32_t firstBlock, uint32_t lastBlock) { return true; }

  protected:

    int errorCode() {
        // Todo
        return 0;
    }

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
    int readBlockWrapper(uint32_t readBlockNumber, uint8_t *dst) {

        USBH_Status status = USBH_MSC_Read10(
                &USB_OTG_Core_Host, &USB_Host,
                dst, readBlockNumber, 512);
        if (status == USBH_BUSY) {
            return 1; // continue thread
        }

        // Todo: check errors
        // if (status != USBH_OK) ...
        massert(status == USBH_OK);
        
        return 0; // OK, write done
    }

    int writeBlock(uint32_t writeBlockNumber, uint8_t *src) {

        USBH_Status status = USBH_MSC_Write10(
                &USB_OTG_Core_Host, &USB_Host,
                src, writeBlockNumber, 512);

        if (status == USBH_BUSY) {
            return 1; // continue thread
        }

        // Todo: check errors
        // if (status != USBH_OK) ...
        massert(status == USBH_OK);
        
        return 0; // OK, write done
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


