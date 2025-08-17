/* -----------------------------------------------------------------------------
 * EE 329 - PWM DRIVER HEADER
 *
 * @file     : pwm.h
 * @brief    : Header for PWM control functions using TIM2 and TIM3
 * @project  : EE329 Embedded Actuation / Servo Control
 * @author   : Nathan H., Dante B., Benjamin T., Gulianna C.
 * @version  : 1.0
 * @date     : 6/5/25
 * @target   : STM32 NUCLEO-L4A6ZG
 * -------------------------------------------------------------------------- */

#ifndef PWM_H
#define PWM_H

#include <stdint.h>

/* -----------------------------------------------------------------------------
 * Function Prototypes
 * -------------------------------------------------------------------------- */

/**
 * @brief  Initializes TIM2 and TIM3 for 50 Hz PWM on GPIOA pins
 */
void PWM_Init( void );

/**
 * @brief  Sets pulse width for a given PWM channel
 * @param  channel: Servo index (1–5)
 * @param  pulse_us: Desired pulse in microseconds (1000–2000 µs)
 */
void PWM_SetPulse( uint8_t channel,
                   uint16_t pulse_us );

/**
 * @brief  Converts a servo angle (in degrees) to a PWM pulse width
 * @param  angle: Input angle [-60°, +60°]
 * @return PWM pulse in microseconds [1000, 2000]
 */
uint16_t angle_to_pulse( float angle );

#endif /* PWM_H */



