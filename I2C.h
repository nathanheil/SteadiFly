/* -----------------------------------------------------------------------------
 * EE 329 - I2C HEADER FOR MPU6050 INTERFACE
 *
 * @file     : I2C.h
 * @brief    : Header declarations for I2C communication and MPU6050 access
 * @project  : EE329 MPU-6050 Interface
 * @author   : Nathan Heil
 * @version  : 1.0
 * @date     : 2025-05-18
 * @target   : STM32 NUCLEO-L4A6ZG
 * -------------------------------------------------------------------------- */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "stm32l4xx.h"
#include "delay.h"

/* -----------------------------------------------------------------------------
 * I2C Hardware Initialization and Communication
 * -------------------------------------------------------------------------- */
void    I2C_Init( void );                                         // Initialize I2C1 with GPIOB
void    I2C_WriteByte( uint8_t dev_addr,
                       uint8_t reg_addr,
                       uint8_t data );                            // Write single byte to register
uint8_t I2C_ReadByte( uint8_t dev_addr,
                      uint8_t reg_addr );                         // Read single byte from register
void    I2C_ReadBytes( uint8_t dev_addr,
                       uint8_t reg_addr,
                       uint8_t* buffer,
                       uint8_t len );                             // Read multiple bytes

/* -----------------------------------------------------------------------------
 * MPU-6050 Sensor Control Functions
 * -------------------------------------------------------------------------- */
void    MPU6050_Init( void );                                     // Initialize and configure sensor
void    MPU6050_ReadGyro( int16_t* gx,
                          int16_t* gy,
                          int16_t* gz );                          // Read gyroscope values
void    MPU6050_ReadAccel( int16_t* ax,
                           int16_t* ay,
                           int16_t* az );                         // Read accelerometer values

#endif /* INC_I2C_H_ */

