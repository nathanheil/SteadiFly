    /**
 ******************************************************************************
 * @file     : delay.h
 * @project  : EE329 Lab A3
 * @author   : Nathan Heil
 * @date     : 2025-05-18
 * @brief    : Header for microsecond delay functions using SysTick
 ******************************************************************************
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include <stdint.h>     // defines uint32_t

/* ---------------- Function Prototypes ---------------- */
void SysTick_Init(void);                    // sets up SysTick timer
void delay_us(const uint32_t time_us);      // delays for time_us µs

#endif /* INC_DELAY_H_ */


