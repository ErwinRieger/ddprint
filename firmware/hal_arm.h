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
#define SERIAL_TX_DR_PUTC(c) ( USART1->regs->DR = c )

#if 0
inline void RCC_DeInit(void)
{
  /* Set HSION bit */
  RCC_BASE->CR |= (uint32_t)0x00000001;

  /* Reset CFGR register */
  RCC_BASE->CFGR = 0x00000000;

  /* Reset HSEON, CSSON, PLLON, PLLI2S and PLLSAI(STM32F42/43xxx devices) bits */
  RCC_BASE->CR &= (uint32_t)0xEAF6FFFF;

  /* Reset PLLCFGR register */
  RCC_BASE->PLLCFGR = 0x24003010;

  /* Reset PLLI2SCFGR register */
  RCC_BASE->PLLI2SCFGR = 0x20003000;

#if defined (STM32F427_437xx) || defined (STM32F429_439xx)
  /* Reset PLLSAICFGR register, only available for STM32F42/43xxx devices */
  RCC_BASE->PLLSAICFGR = 0x24003000;
#endif

  /* Reset HSEBYP bit */
  RCC_BASE->CR &= (uint32_t)0xFFFBFFFF;

  /* Disable all interrupts */
  RCC_BASE->CIR = 0x00000000;

#if defined (STM32F427_437xx) || defined (STM32F429_439xx)
  /* Disable Timers clock prescalers selection, only available for STM32F42/43xxx devices */
  RCC_BASE->DCKCFGR = 0x00000000;
#endif
}
#endif

inline void RCC_DeInit(rcc_reg_map *resetRCC)
{

  RCC_BASE->AHB1RSTR = resetRCC->AHB1RSTR;
  RCC_BASE->AHB2RSTR = resetRCC->AHB2RSTR;
  RCC_BASE->AHB3RSTR = resetRCC->AHB3RSTR;
  RCC_BASE->APB1RSTR = resetRCC->APB1RSTR;
  RCC_BASE->APB2RSTR = resetRCC->APB2RSTR;
  RCC_BASE->AHB1ENR = resetRCC->AHB1ENR;
  RCC_BASE->AHB2ENR = resetRCC->AHB2ENR;
  RCC_BASE->AHB3ENR = resetRCC->AHB3ENR;
  RCC_BASE->APB1ENR = resetRCC->APB1ENR;
  RCC_BASE->APB2ENR = resetRCC->APB2ENR;
  RCC_BASE->AHB1LPENR = resetRCC->AHB1LPENR;
  RCC_BASE->AHB2LPENR = resetRCC->AHB2LPENR;
  RCC_BASE->AHB3LPENR = resetRCC->AHB3LPENR;
  RCC_BASE->APB1LPENR = resetRCC->APB1LPENR;
  RCC_BASE->APB2LPENR = resetRCC->APB2LPENR;
  RCC_BASE->BDCR = resetRCC->BDCR;
  RCC_BASE->CSR = resetRCC->CSR;
  RCC_BASE->SSCGR = resetRCC->SSCGR;

  /* Set HSION bit */
  RCC_BASE->CR |= (uint32_t)0x00000001;

  /* Reset CFGR register */
  RCC_BASE->CFGR = 0x00000000;

  /* Reset HSEON, CSSON, PLLON, PLLI2S and PLLSAI(STM32F42/43xxx devices) bits */
  RCC_BASE->CR &= (uint32_t)0xEAF6FFFF;

  /* Reset PLLCFGR register */
  RCC_BASE->PLLCFGR = 0x24003010;

  /* Reset PLLI2SCFGR register */
  RCC_BASE->PLLI2SCFGR = 0x20003000;

#if defined (STM32F427_437xx) || defined (STM32F429_439xx)
  /* Reset PLLSAICFGR register, only available for STM32F42/43xxx devices */
  RCC_BASE->PLLSAICFGR = 0x24003000;
#endif

  /* Reset HSEBYP bit */
  RCC_BASE->CR &= (uint32_t)0xFFFBFFFF;

  /* Disable all interrupts */
  RCC_BASE->CIR = 0x00000000;

#if defined (STM32F427_437xx) || defined (STM32F429_439xx)
  /* Disable Timers clock prescalers selection, only available for STM32F42/43xxx devices */
  RCC_BASE->DCKCFGR = 0x00000000;
#endif

}

/**
 * Function to perform jump to system memory boot from user application
 *
 * Call function when you want to jump to system memory
 */
inline void JumpToBootloader(rcc_reg_map *resetRCC) {

    void (*SysMemBootJump)(void);
 
    /**
    * Step: Set system memory address. 
    *       
    *       For STM32F429, system memory is on 0x1FFF 0000
    *       For other families, check AN2606 document table 110 with descriptions of memory addresses 
    */
    volatile uint32_t addr = 0x1FFF0000;
 
    /**
    * Step: Disable RCC, set it to default (after reset) settings
    *       Internal clock, no PLL, etc.
    */
    RCC_DeInit(resetRCC);
 
    /**
    * Step: Disable systick timer and reset it to default values
    */
    SYSTICK_BASE->CSR = 0;
    SYSTICK_BASE->RVR = 0;
    SYSTICK_BASE->CNT = 0;

    /**
    * Step: Disable all interrupts
    */
    // asm volatile ("cpsid i" : : : "memory");
 
    /**
    * Step: Remap system memory to address 0x0000 0000 in address space
    *       For each family registers may be different. 
    *       Check reference manual for each family.
    *
    *       For STM32F4xx, MEMRMP register in SYSCFG is used (bits[1:0])
    *       For STM32F0xx, CFGR1 register in SYSCFG is used (bits[1:0])
    *       For others, check family reference manual
    */
    //Remap by hand... {
#if defined(STM32F4)
    // SYSCFG->MEMRMP = 0x01;
    SYSCFG_BASE->MEMRM = 0x01;
#endif
#if defined(STM32F0)
    SYSCFG->CFGR1 = 0x01;
#endif
    //} ...or if you use HAL drivers
    //__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH(); //Call HAL macro to do this for you
 
    /**
    * Step: Set jump memory location for system memory
    *       Use address with 4 bytes offset which specifies jump location where program starts
    */
    SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));
 
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
    while(1);
}



