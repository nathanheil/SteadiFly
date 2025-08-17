        /*
 * uart.h — header for MPU-6050 UART display functions
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include <stdint.h>

void LPUART_init(void);
void LPUART_Print(const char *msg);
void UART_PrintGyroData(int16_t gx, int16_t gy, int16_t gz);

#endif /* INC_UART_H_ */




