/* -----------------------------------------------------------------------------
 * EE 329 - PWM DRIVER MODULE
 *
 * @file     : pwm.c
 * @brief    : Generates PWM signals for up to 5 servos using TIM2 and TIM3
 * @project  : EE329 Embedded Actuation / Servo Control
 * @author   : Nathan H., Dante B., Benjamin T., Gulianna C.
 * @version  : 1.0
 * @date     : 6/5/25
 * @target   : STM32 NUCLEO-L4A6ZG
 * -------------------------------------------------------------------------- */

#include "pwm.h"
#include "stm32l4xx.h"

/* -----------------------------------------------------------------------------
 * PWM_Init()
 * Configures GPIOA pins and TIM2/TIM3 to generate 50 Hz PWM signals
 * TIM2 channels: PA0–PA3 for Servos 1–4
 * TIM3 channel1: PA6 for Servo 5
 * -------------------------------------------------------------------------- */
void PWM_Init(void) {
   // Enable GPIOA and required timer clocks
   RCC->AHB2ENR   |= RCC_AHB2ENR_GPIOAEN;
   RCC->APB1ENR1  |= RCC_APB1ENR1_TIM2EN;
   RCC->APB1ENR1  |= RCC_APB1ENR1_TIM3EN;

   // Configure PA0–PA3 for TIM2 CH1–CH4 alternate function (AF1)
   for (int i = 0; i < 4; i++) {
      GPIOA->MODER &= ~(0x3 << (i * 2));
      GPIOA->MODER |=  (0x2 << (i * 2));
      GPIOA->AFR[0] &= ~(0xF << (i * 4));
      GPIOA->AFR[0] |=  (0x1 << (i * 4));
   }

   // TIM2 setup: 50 Hz PWM (20 ms period)
   TIM2->PSC     = 79;      // 4 MHz / 80 = 50 kHz
   TIM2->ARR     = 999;     // 50 kHz / 1000 = 50 Hz

   TIM2->CCMR1   = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                   (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
   TIM2->CCMR2   = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                   (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

   TIM2->CCER   |= TIM_CCER_CC1E | TIM_CCER_CC2E |
                   TIM_CCER_CC3E | TIM_CCER_CC4E;

   TIM2->CR1    |= TIM_CR1_ARPE;
   TIM2->EGR    |= TIM_EGR_UG;
   TIM2->CR1    |= TIM_CR1_CEN;

   // Configure PA6 for TIM3 CH1 alternate function (AF2)
   GPIOA->MODER  &= ~(0x3 << (6 * 2));
   GPIOA->MODER  |=  (0x2 << (6 * 2));
   GPIOA->AFR[0] &= ~(0xF << GPIO_AFRL_AFSEL6_Pos);
   GPIOA->AFR[0] |=  (0x2 << GPIO_AFRL_AFSEL6_Pos);

   // TIM3 setup: 50 Hz PWM on CH1 (Servo 5)
   TIM3->PSC     = 79;
   TIM3->ARR     = 999;
   TIM3->CCMR1  |= (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
   TIM3->CCER   |= TIM_CCER_CC1E;
   TIM3->CR1    |= TIM_CR1_ARPE;
   TIM3->EGR    |= TIM_EGR_UG;
   TIM3->CR1    |= TIM_CR1_CEN;
}

/* -----------------------------------------------------------------------------
 * PWM_SetPulse()
 * Sets pulse width in microseconds for a specified servo channel.
 * Acceptable range is 1000–2000 µs. Converts to 50 Hz ticks.
 *
 * channel   - Servo output (1–5)
 * pulse_us  - Pulse width in microseconds
 * -------------------------------------------------------------------------- */
void PWM_SetPulse(uint8_t channel, uint16_t pulse_us) {
   if (pulse_us > 2000) pulse_us = 2000;
   if (pulse_us < 1000) pulse_us = 1000;

   uint16_t ticks = (pulse_us * 50) / 1000; // convert µs to 50 kHz ticks

   switch (channel) {
      case 1: TIM2->CCR1 = ticks; break;
      case 2: TIM2->CCR2 = ticks; break;
      case 3: TIM2->CCR3 = ticks; break;
      case 4: TIM2->CCR4 = ticks; break;
      case 5: TIM3->CCR1 = ticks; break;
      default: break;
   }
}

/* -----------------------------------------------------------------------------
 * angle_to_pulse()
 * Converts an angle in degrees (±60°) to a corresponding pulse width in µs.
 * Maps angle to range [1000, 2000] µs (i.e. 1–2 ms typical servo range).
 *
 * angle - desired angle from -60 to +60 degrees
 * returns - PWM pulse width in microseconds
 * -------------------------------------------------------------------------- */
uint16_t angle_to_pulse(float angle) {
   if (angle > 60.0f) angle = 60.0f;
   if (angle < -60.0f) angle = -60.0f;

   float norm = (angle + 60.0f) / 120.0f;
   return (uint16_t)(1000 + norm * 1000);
}


