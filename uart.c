/* -----------------------------------------------------------------------------
 * EE 329 - UART DRIVER MODULE
 *
 * @file     : uart.c
 * @brief    : Provides LPUART1 initialization and serial print support
 *             Includes UART interrupt handler and live tuning input buffer
 * @project  : EE329 UART-based Gyro Data Monitor
 * @author   : Nathan H., Dante B., Benjamin T., Gulianna C.
 * @version  : 1.0
 * @date     : 6/5/25
 * @target   : STM32 NUCLEO-L4A6ZG
 * -------------------------------------------------------------------------- */

#include "stm32l4xx.h"
#include "uart.h"
#include <stdio.h>

/* -----------------------------------------------------------------------------
 * LPUART_init()
 * Configures LPUART1 and GPIOG pins for asynchronous serial communication
 * PG7 = TX, PG8 = RX, alternate function AF8
 * -------------------------------------------------------------------------- */
void LPUART_init(void) {
   PWR->CR2        |= PWR_CR2_IOSV;
   RCC->AHB2ENR    |= RCC_AHB2ENR_GPIOGEN;
   RCC->APB1ENR2   |= RCC_APB1ENR2_LPUART1EN;

   GPIOG->MODER   &= ~((3 << 14) | (3 << 16));
   GPIOG->MODER   |=  ((2 << 14) | (2 << 16));
   GPIOG->AFR[0]   = (GPIOG->AFR[0] & ~(0xF << 28)) | (8 << 28);
   GPIOG->AFR[1]   = (GPIOG->AFR[1] & ~(0xF << 0))  | (8 << 0);
   GPIOG->OSPEEDR |= (3 << 14) | (3 << 16);
   GPIOG->OTYPER  &= ~(GPIO_OTYPER_OT7 | GPIO_OTYPER_OT8);
   GPIOG->PUPDR   &= ~(GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD8);

   LPUART1->CR1   &= ~USART_CR1_UE;              // Disable UART
   LPUART1->CR1   &= ~(USART_CR1_M1 | USART_CR1_M0);
   LPUART1->CR1   |= (USART_CR1_TE | USART_CR1_RE);  // TX and RX enable
   RCC->CCIPR     &= ~RCC_CCIPR_LPUART1SEL;
   RCC->CCIPR     |= RCC_CCIPR_LPUART1SEL_0;     // PCLK1 clock source
   LPUART1->BRR    = 0x22b9;                     // 9600 baud @ 4 MHz
   LPUART1->CR1   |= USART_CR1_UE;               // Enable UART
   LPUART1->CR1   |= USART_CR1_RXNEIE;           // Enable RX interrupt
}

/* -----------------------------------------------------------------------------
 * LPUART_Print()
 * Sends a null-terminated string over UART
 * @param message: Pointer to string
 * -------------------------------------------------------------------------- */
void LPUART_Print(const char* message) {
   while (*message) {
      while (!(LPUART1->ISR & USART_ISR_TXE));
      LPUART1->TDR = *message++;
   }
}

/* -----------------------------------------------------------------------------
 * UART_PrintGyroData()
 * Formats and prints gyro X, Y, Z values to UART terminal
 * @param gx, gy, gz: Raw gyroscope values
 * -------------------------------------------------------------------------- */
void UART_PrintGyroData(int16_t gx, int16_t gy, int16_t gz) {
   char buf[64];
   sprintf(buf, "\x1B[HGyX=%d\tGyY=%d\tGyZ=%d\r\n", gx, gy, gz);
   LPUART_Print(buf);
}

/* -----------------------------------------------------------------------------
 * UART live tuning input buffer
 * Stores up to 9 digits for Kp, Ki, Kd via UART input
 * -------------------------------------------------------------------------- */
int digit_buf[9]  = {1, 6, 0, 0, 1, 0, 1, 0, 0};  // Default PID values
int digit_idx     = 0;                           // Current index
int digit_ready   = 0;                           // Flag to commit buffer

/* -----------------------------------------------------------------------------
 * LPUART1_IRQHandler()
 * UART interrupt handler for character reception
 * Accepts digit characters and populates buffer
 * Sets digit_ready on newline
 * -------------------------------------------------------------------------- */
void LPUART1_IRQHandler(void) {
   if (LPUART1->ISR & USART_ISR_RXNE) {
      char ch = LPUART1->RDR;

      // Finish input on Enter key
      if (ch == '\r' || ch == '\n') {
         digit_ready = 1;
         digit_idx = 0;   // Reset index for next entry
      }

      // Accept digits only
      else if (ch >= '0' && ch <= '9' && digit_idx < 9) {
         digit_buf[digit_idx++] = ch - '0';
      }
   }
}


