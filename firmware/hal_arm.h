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

// #include <VCP/core_cm4.h>

//
// STM32
//

//
// Serial interface, note: fixed USART1 usage
//
#define SERIAL_TX_DR_EMPTY() ( USART1->regs->SR & USART_SR_TXE ) 
#define SERIAL_TX_COMPLETE() ( USART1->regs->SR & USART_SR_TC )
#define SERIAL_TX_DR_PUTC(c) ( USART1->regs->DR = c )

#define SET_INPUT(pin) pinMode(pin, INPUT_FLOATING)
#define SET_INPUT_PD(pin) pinMode(pin, INPUT_PULLDOWN)
#define SET_INPUT_ANALOG(pin) pinMode(pin, INPUT_ANALOG)

#define SET_OUTPUT(pin)  pinMode(pin, OUTPUT)
#define SET_OUTPUT_PWM(pin)  pinMode(pin, PWM)

#define READ(pin) digitalRead(pin)
#define READ_ANALOG(pin) analogRead(pin)

#define WRITE(pin, v)  digitalWrite(pin, v)
// #define PWM_WRITE(p, v) pwmWrite(p, map(v, 0, 255, 0, 65535))
#define PWM_WRITE(p, v) pwmWrite(p, v)

#define CLI()   noInterrupts()

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



