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

// extern const uint8 adc_map[];
extern uint8 *adc_map;

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
// XXX re-implement pinMode() from stm32duino() since it has a problem with unintended timer-deactivation
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

    avgBedTemp.addValue( tempFromRawADC( TEMP_BED_PIN :: read() ) );
    current_temperature_bed = avgBedTemp.value();

    ////////////////////////////////
    // Read hotend temp
    ////////////////////////////////
    TEMP_0_PIN :: startConversion();

    PT_WAIT_UNTIL( TEMP_0_PIN :: conversionDone() );

    avgHotendTemp.addValue( tempFromRawADC( TEMP_0_PIN :: read() ) );
    current_temperature[0] = avgHotendTemp.value();

    PT_RESTART();
        
    // Not reached
    PT_END();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//--------------------------------------------------------------
// Timer 2 irq handler
void __irq_tim2(void)
{
}

#endif
