/* -----------------------------------------------------------------------------
 * EE 329 - I2C DRIVER FOR MPU6050 IMU SENSOR
 *
 * @file     : I2C.c
 * @brief    : Implements low-level I2C functions for MPU6050 access.
 *             Supports byte and burst reads, and gyro/accel interface.
 * @author   : Nathan H., Dante B., Benjamin T., Gulianna C.
 * @version  : 1.0
 * @date     : 6/5/25
 * @target   : STM32 NUCLEO-L4A6ZG
 * @toolchain: STM32CubeIDE v1.12.0
 * -------------------------------------------------------------------------- */

#include "I2C.h"
#include "delay.h"

/* -----------------------------------------------------------------------------
 * I2C_Init()
 * Configures GPIOB and I2C1 peripheral for I2C communication.
 * Uses PB8 (SCL) and PB9 (SDA) with AF4 function.
 * -------------------------------------------------------------------------- */
void I2C_Init(void) {
   RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;

   GPIOB->MODER  &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
   GPIOB->MODER  |=  (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);
   GPIOB->OTYPER |=  (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
   GPIOB->PUPDR  &= ~(GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9);
   GPIOB->OSPEEDR |= ((3 << GPIO_OSPEEDR_OSPEED8_Pos) |
                      (3 << GPIO_OSPEEDR_OSPEED9_Pos));
   GPIOB->AFR[1] &= ~((0xF << GPIO_AFRH_AFSEL8_Pos) |
                      (0xF << GPIO_AFRH_AFSEL9_Pos));
   GPIOB->AFR[1] |=  ((0x4 << GPIO_AFRH_AFSEL8_Pos) |
                      (0x4 << GPIO_AFRH_AFSEL9_Pos));

   RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
   I2C1->CR1     &= ~I2C_CR1_PE;
   I2C1->CR1     &= ~I2C_CR1_ANFOFF;
   I2C1->CR1     &= ~I2C_CR1_DNF;
   I2C1->TIMINGR  = 0x00000508;              // Set bus speed
   I2C1->CR2     &= ~I2C_CR2_ADD10;
   I2C1->CR1     |= I2C_CR1_PE;              // Enable I2C
}

/* -----------------------------------------------------------------------------
 * I2C_WriteByte()
 * Writes a single byte to a register of a slave device.
 * dev_addr  - 7-bit device address
 * reg_addr  - register address to write to
 * data      - byte to write
 * -------------------------------------------------------------------------- */
void I2C_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
   I2C1->CR2 = 0;
   I2C1->CR2 |= (dev_addr << 1);
   I2C1->CR2 |= (2 << I2C_CR2_NBYTES_Pos);
   I2C1->CR2 &= ~I2C_CR2_RD_WRN;
   I2C1->CR2 |= I2C_CR2_START | I2C_CR2_AUTOEND;

   while (!(I2C1->ISR & I2C_ISR_TXIS));
   I2C1->TXDR = reg_addr;
   while (!(I2C1->ISR & I2C_ISR_TXIS));
   I2C1->TXDR = data;
   while (!(I2C1->ISR & I2C_ISR_STOPF));
   I2C1->ICR |= I2C_ICR_STOPCF;
}

/* -----------------------------------------------------------------------------
 * I2C_ReadByte()
 * Reads a single byte from a register of a slave device.
 * dev_addr  - 7-bit device address
 * reg_addr  - register address to read from
 * returns   - the byte read from the register
 * -------------------------------------------------------------------------- */
uint8_t I2C_ReadByte(uint8_t dev_addr, uint8_t reg_addr) {
   uint8_t data;
   I2C_ReadBytes(dev_addr, reg_addr, &data, 1);
   return data;
}

/* -----------------------------------------------------------------------------
 * I2C_ReadBytes()
 * Performs burst read from I2C slave starting at reg_addr.
 * dev_addr - 7-bit device address
 * reg_addr - start register
 * buffer   - pointer to destination buffer
 * len      - number of bytes to read
 * -------------------------------------------------------------------------- */
void I2C_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t* buffer, uint8_t len) {
   I2C1->CR2 = 0;
   I2C1->CR2 |= (dev_addr << 1);
   I2C1->CR2 |= (1 << I2C_CR2_NBYTES_Pos);
   I2C1->CR2 &= ~I2C_CR2_RD_WRN;
   I2C1->CR2 |= I2C_CR2_START;

   while (!(I2C1->ISR & I2C_ISR_TXIS));
   I2C1->TXDR = reg_addr;
   while (!(I2C1->ISR & I2C_ISR_TC));

   I2C1->CR2 = 0;
   I2C1->CR2 |= (dev_addr << 1) | I2C_CR2_RD_WRN;
   I2C1->CR2 |= (len << I2C_CR2_NBYTES_Pos);
   I2C1->CR2 |= I2C_CR2_START | I2C_CR2_AUTOEND;

   for (uint8_t i = 0; i < len; i++) {
      while (!(I2C1->ISR & I2C_ISR_RXNE));
      buffer[i] = I2C1->RXDR;
   }

   while (!(I2C1->ISR & I2C_ISR_STOPF));
   I2C1->ICR |= I2C_ICR_STOPCF;
}

/* -----------------------------------------------------------------------------
 * MPU6050_Init()
 * Initializes MPU6050 IMU sensor by waking it up and setting gyro config.
 * Uses I2C_WriteByte to write to power and config registers.
 * -------------------------------------------------------------------------- */
void MPU6050_Init(void) {
   I2C_WriteByte(0x69, 0x6B, 0x00);  // Wake up device
   delay_us(100000);                // Wait 100ms
   I2C_WriteByte(0x69, 0x1B, 0x00);  // Set gyro range to ±250 dps
}

/* -----------------------------------------------------------------------------
 * MPU6050_ReadGyro()
 * Reads 6 bytes from the gyro output registers and converts to signed values.
 * gx, gy, gz - pointers to destination integers for gyro data
 * -------------------------------------------------------------------------- */
void MPU6050_ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz) {
   uint8_t data[6];
   I2C_ReadBytes(0x69, 0x43, data, 6);

   *gx = (data[0] << 8) | data[1];
   *gy = (data[2] << 8) | data[3];
   *gz = (data[4] << 8) | data[5];
}

/* -----------------------------------------------------------------------------
 * MPU6050_ReadAccel()
 * Reads 6 bytes from the accelerometer output registers and stores values.
 * ax, ay, az - pointers to destination integers for accel data
 * -------------------------------------------------------------------------- */
void MPU6050_ReadAccel(int16_t* ax, int16_t* ay, int16_t* az) {
   uint8_t data[6];
   I2C_ReadBytes(0x69, 0x3B, data, 6);

   *ax = (data[0] << 8) | data[1];
   *ay = (data[2] << 8) | data[3];
   *az = (data[4] << 8) | data[5];
}


