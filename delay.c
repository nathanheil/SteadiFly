    /**
 ******************************************************************************
 * @file     : delay.c
 * @project  : EE329 Lab A3
 * @author   : Nathan Heil
 * @date     : 2025-05-18
 * @brief    : Software microsecond delay using SysTick hardware timer
 * target    : STM32L4A6ZG @ 4 MHz
 ******************************************************************************
 */

#include "delay.h"
#include "stm32l4xx.h"
/* ------------------------------- SysTick_Init -------------------------------
 * Configures the ARM SysTick timer to run off the core clock.
 * Interrupts are disabled for precise microsecond timing.
 * Warning: this disables HAL_Delay functionality.
 * -------------------------------------------------------------------------- */
void SysTick_Init(void) {
   SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk |
                     SysTick_CTRL_CLKSOURCE_Msk);  // enable SysTick
   SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;     // disable SysTick IRQ
}

/* -------------------------------- delay_us ----------------------------------
 * Generates a blocking delay of time_us microseconds using the SysTick timer.
 *
 * Inputs :
 *   time_us : number of microseconds to delay (must be > 0)
 *
 * Returns :
 *   none (blocks until time_us has passed)
 *
 * Notes :
 *   @4 MHz, delay_us(1) results in ~10-15 us due to setup overhead.
 * -------------------------------------------------------------------------- */
void delay_us(const uint32_t time_us) {
   if (time_us == 0) return;

   // Calculate timer reload value from system clock
   SysTick->LOAD = (uint32_t)((time_us * (SystemCoreClock / 1000000)) - 1);
   SysTick->VAL  = 0;                              // clear current count
   SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;  // clear overflow flag

   // Wait for COUNTFLAG to set when timer expires
   while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
}


