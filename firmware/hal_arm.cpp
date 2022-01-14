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

#if defined(__arm__)

#include "ddtemp.h"
#include "temperature.h"
#include "pins.h"
#include "thermistortables.h"
#include "rxbuffer.h"

// extern const uint8 adc_map[];
extern uint8 *adc_map;

// Thermistortable Bed
static ThermistorTable<TempCircuitBed> thermistorTableBed;

// Thermistortable Hotend
static ThermistorTable<TempCircuitHotend> thermistorTableHotend;

/*
 * Derived from (the disabled) stm32duino:timerDefaultConfig().
 * This possibly only works if there is no timersetup in arduino:init().
 */
void pwmInit(uint8_t pwmpin, uint16_t duty, bool activeLow)
{
    timer_info tinfo = timer_map[pwmpin];

	const timer_dev * dev = timer_devices[tinfo.index];
	uint8_t channel = tinfo.channel;

    // Note: we use the general register map for the advanced timers, too.
    // This works because they are the same with respect to the registers we
    // use here.
    timer_gen_reg_map *regs = dev->regs.gen;

    const uint16 full_overflow = 0xFFFF;

    // Basic timer works differently
    massert(dev->type != TIMER_BASIC);

    timer_pause(dev);

    regs->CR1 = TIMER_CR1_ARPE;

    if (dev->type == TIMER_ADVANCED)
        dev->regs.adv->BDTR = TIMER_BDTR_MOE;

    // Note: ARR register used by all channels of this timer.
    timer_set_reload(dev, full_overflow);
    timer_set_compare(dev, channel, duty);
    timer_oc_set_mode(dev, channel, TIMER_OC_MODE_PWM_1, TIMER_OC_PE);

    if (activeLow)
        regs->CCER |= 1 << (1 + 4*(channel-1));

    // This is from pinMode(pin, PWM), set CCxE bit: 
    regs->CCER |= 1 << (4 * (channel - 1));

    regs->EGR = TIMER_EGR_UG;

    gpio_set_mode(pwmpin, GPIO_AF_OUTPUT_PP);
    gpio_set_af_mode(pwmpin, dev->af_mode);

    timer_resume(dev);
}


////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Te-implement pinMode() from stm32duino() since it has a problem with unintended timer-deactivation
//
void myPinMode(uint8 pin, WiringPinMode mode) {
    gpio_pin_mode outputMode;

    if (pin >= BOARD_NR_GPIO_PINS) {
        return;
    }

    switch(mode) {
    case OUTPUT:
        outputMode = GPIO_OUTPUT_PP;
        break;
    case OUTPUT_OPEN_DRAIN:
        outputMode = GPIO_OUTPUT_OD;
        break;
    case INPUT:
    case INPUT_FLOATING:
        outputMode = GPIO_INPUT_FLOATING;
        break;
    case INPUT_ANALOG:
        outputMode = GPIO_INPUT_ANALOG;
        break;
    case INPUT_PULLUP:
        outputMode = GPIO_INPUT_PU;
        break;
    case INPUT_PULLDOWN:
        outputMode = GPIO_INPUT_PD;
        break;
    default:
        massert(0);
        return;
    }

    gpio_set_mode(pin, outputMode);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void timerInit() {

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

// XXX is this used?
    timer_set_compare(&timer2, 1, 5); // Pulse width for stepper motors

    // bb_peri_set_bit(&(timer3.regs.gen)->CR1, TIMER_CR1_ARPE_BIT, 1);

    nvic_irq_enable(NVIC_TIMER2);
    nvic_irq_enable(NVIC_TIMER3);

    timer_resume(&timer2);
    timer_resume(&timer3);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void irqInit() {

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

uint8_t filsensorRead() {

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
void JumpToBootloader() {

    rxBuffer.end();

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

void spiInit() {

#if defined(HASFILAMENTSENSOR)
    //
    // Filamentsensor is connected to USART2 in *spi-like* mode
    //
    /*
    Connection of EMS22 incremental flowrate sensor to jennyprinter *shuttle gear* board:

    J404, Pin1, PD6: USART2_RX (AF7) as SPI MISO
    J404, Pin3, PE8: GPIO as SPI CS
    J404, Pin9     : GND

    J402, Pin1, PA4: SPI Clock

    J102, Pin4     : +5V VCC
    */

    // Do some minimal SPI init, prevent SPI to go to spi slave mode
    FILSENSNCS :: initDeActive();

    gpio_set_af_mode(MISO_PIN, (gpio_af_mode)7);
    gpio_set_af_mode(SCK_PIN, (gpio_af_mode)7); 
    // gpio_set_af_mode(MOSI_PIN, (gpio_af_mode)7);

    gpio_set_mode(MISO_PIN, (gpio_pin_mode)(GPIO_AF_INPUT_PU | 0x700));         // PD6 as USART2_RX
    gpio_set_mode(SCK_PIN, (gpio_pin_mode)(GPIO_AF_OUTPUT_PP_PU | 0x700));      // PA4 as USART2_CK
    // gpio_set_mode(MOSI_PIN, (gpio_pin_mode)(GPIO_AF_OUTPUT_PP_PU | 0x700));  // PD5 as USART2_TX

    rcc_clk_enable(USART2->clk_id);

    usart_set_baud_rate(USART2, 1000000);

    // Enable clock output, set SPI Mode 3 (CPOL = 0, CPHA = 1)
    usart_reg_map *regs = USART2->regs; 
    regs->CR2 = USART_CR2_CLKEN | USART_CR2_CPHA | USART_CR2_LBCL;
    regs->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
#endif

}

////////////////////////////////////////////////////////////////////////////////////////////////////

void HAL_SETUP_TEMP_ADC() {

    TEMP_BED_PIN :: init();
    TEMP_0_PIN :: init();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool TempControl::Run() {

    PT_BEGIN();

    ////////////////////////////////
    // Read bed temp
    ////////////////////////////////
    TEMP_BED_PIN :: startConversion();

    PT_WAIT_UNTIL( TEMP_BED_PIN :: conversionDone() );

    current_temperature_bed = avgBedTemp.addValue( thermistorTableBed.tempFromRawADC( TEMP_BED_PIN :: read() ) );

    ////////////////////////////////
    // Read hotend temp
    ////////////////////////////////
    TEMP_0_PIN :: startConversion();

    PT_WAIT_UNTIL( TEMP_0_PIN :: conversionDone() );

    current_temperature[0] = avgHotendTemp.addValue( thermistorTableHotend.tempFromRawADC( TEMP_0_PIN :: read() ) );

    PT_RESTART();
        
    // Not reached
    PT_END();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////


#endif
