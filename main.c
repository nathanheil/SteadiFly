/* -----------------------------------------------------------------------------
 * EE 329 - FULL PID CONTROL SYSTEM WITH UART TUNING
 *
 * @file     : main.c
 * @brief    : Implements a Kalman-filtered PID control loop with live UART tuning
 * @author   : Nathan H., Dante B., Benjamin T., Gulianna C.
 * @version  : 1.0
 * @date     : 6/5/25
 * @target   : STM32 NUCLEO-L4A6ZG @ 80 MHz (TIM5 @ 1 MHz)
 * @toolchain: STM32CubeIDE v1.12.0
 * @note     : Ensure MPU6050 is connected via I2C (ADDR = 0x69)
 * -------------------------------------------------------------------------- */

#include "main.h"
#include "I2C.h"
#include "UART.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "PWM.h"
#include "PID.h"

#define MPU_ADDR ( 0x69 )

extern int digit_buf[9];       // UART live tuning input buffer
extern int digit_ready;        // Flag indicating when to update PID gains

/* -----------------------------------------------------------------------------
 * Timer5_Init()
 * Initializes TIM5 for microsecond-precision timekeeping (1 MHz frequency).
 * Used to timestamp data for logging or precise control timing.
 * -------------------------------------------------------------------------- */
void Timer5_Init( void ) {
   RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
   TIM5->PSC      = 79;                  // 80 MHz / 80 = 1 MHz -> 1 us/tick
   TIM5->ARR      = 0xFFFFFFFF;          // Max 32-bit count
   TIM5->CR1     |= TIM_CR1_CEN;        // Enable timer
}

/* -----------------------------------------------------------------------------
 * Kalman filter structure and helper functions
 * Used to fuse and smooth noisy accelerometer and gyroscope data.
 * -------------------------------------------------------------------------- */
typedef struct {
   float angle;     // Estimated angle
   float bias;      // Gyro bias
   float rate;      // Unbiased angular rate
   float P[2][2];   // Error covariance matrix
} Kalman_t;

void Kalman_Init( Kalman_t* k ) {
   k->angle   = 0.0f;
   k->bias    = 0.0f;
   k->P[0][0] = 1.0f;
   k->P[0][1] = 0.0f;
   k->P[1][0] = 0.0f;
   k->P[1][1] = 1.0f;
}

/*
 * Kalman_Update()
 * Applies a Kalman filter step to estimate the new angle.
 * newAngle - from accelerometer, newRate - from gyro, dt - time delta
 */
float Kalman_Update( Kalman_t* k, float newAngle, float newRate, float dt ) {
   const float Q_angle   = 0.005f;
   const float Q_bias    = 0.01f;
   const float R_measure = 0.03f;

   k->rate  = newRate - k->bias;
   k->angle += dt * k->rate;

   k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + Q_angle);
   k->P[0][1] -= dt * k->P[1][1];
   k->P[1][0] -= dt * k->P[1][1];
   k->P[1][1] += Q_bias * dt;

   float S   = k->P[0][0] + R_measure;
   float K0  = k->P[0][0] / S;
   float K1  = k->P[1][0] / S;
   float y   = newAngle - k->angle;

   k->angle += K0 * y;
   k->bias  += K1 * y;

   float P00_temp = k->P[0][0];
   float P01_temp = k->P[0][1];

   k->P[0][0] -= K0 * P00_temp;
   k->P[0][1] -= K0 * P01_temp;
   k->P[1][0] -= K1 * P00_temp;
   k->P[1][1] -= K1 * P01_temp;

   return k->angle;
}

/* -----------------------------------------------------------------------------
 * getPitch() and getRoll()
 * Converts raw accelerometer readings to angles in degrees.
 * Uses arctangent relationships between axis readings.
 * -------------------------------------------------------------------------- */
float getPitch( int16_t ax, int16_t ay, int16_t az ) {
   return atan2f(ax, sqrtf(ay * ay + az * az)) * (180.0f / 3.14159f);
}

float getRoll( int16_t ax, int16_t ay, int16_t az ) {
   return atan2f(ay, sqrtf(ax * ax + az * az)) * (180.0f / 3.14159f);
}

/* -----------------------------------------------------------------------------
 * CalibrateGyro()
 * Calculates offset for the gyroscope's Z-axis by averaging 1000 samples.
 * This removes constant drift from Z-axis yaw rate.
 * -------------------------------------------------------------------------- */
void CalibrateGyro( int16_t* offset_z ) {
   int32_t sum_z = 0;
   int16_t gz;
   for (int i = 0; i < 1000; i++) {
      MPU6050_ReadGyro(NULL, NULL, &gz); // Read only gz
      sum_z += gz;
      delay_us(1000);
   }
   *offset_z = sum_z / 1000;
}

/* -----------------------------------------------------------------------------
 * main()
 * Main control loop: initializes hardware, filters IMU data, updates PID control,
 * prints diagnostics, and drives servo motors based on angle corrections.
 * -------------------------------------------------------------------------- */
int main( void ) {
   Timer5_Init();

   // Sensor variables and system state
   int16_t gx, gy, gz, ax, ay, az;
   float yaw = 0.0f;
   int16_t offset_z = 0;
   const float dt = 0.009f;            // Sampling interval
   float sensitivity = 16.40f;         // Gyro sensitivity scale factor

   // Kalman filter state for pitch and roll
   Kalman_t kal_pitch, kal_roll;
   Kalman_Init(&kal_pitch);
   Kalman_Init(&kal_roll);

   // Filtered output for display
   static float smooth_pitch = 0.0f;
   static float smooth_roll  = 0.0f;
   static float smooth_yaw   = 0.0f;

   // Peripheral initialization
   HAL_Init();
   SystemClock_Config();
   SysTick_Init();
   I2C_Init();
   LPUART_init();
   NVIC->ISER[2] |= (1 << (LPUART1_IRQn & 0x1F));
   __enable_irq();
   PWM_Init();

   LPUART_Print("\x1B[2J\x1B[H"); // Clear terminal
   MPU6050_Init();
   I2C_WriteByte(MPU_ADDR, 0x1B, 0x18); // Configure gyro full scale
   CalibrateGyro(&offset_z);

   // Initial PID gains and scratch buffers for UART tuning
   float kp = 1.5f, ki = 0.006f, kd = 0.005f;
   float kps = 1.2f, kis = 0.01f, kds = 0.1f;

   while (1) {
      // ------------------------ Sensor Input ------------------------
      MPU6050_ReadGyro(&gx, &gy, &gz);
      MPU6050_ReadAccel(&ax, &ay, &az);

      float raw_pitch = getPitch(ax, ay, az);
      float raw_roll  = getRoll(ax, ay, az);
      float gx_dps    = gx / sensitivity;
      float gy_dps    = gy / sensitivity;
      float gz_dps    = (gz - offset_z) / sensitivity;

      // ----------------------- Sensor Fusion ------------------------
      float pitch = Kalman_Update(&kal_pitch, raw_pitch, gx_dps, dt);
      float roll  = Kalman_Update(&kal_roll,  raw_roll,  gy_dps, dt);
      yaw += gz_dps * dt;

      // -------------------------- Smoothing --------------------------
      smooth_pitch = 0.9f * smooth_pitch + 0.1f * pitch;
      smooth_roll  = 0.9f * smooth_roll  + 0.1f * roll;
      smooth_yaw   = 0.9f * smooth_yaw   + 0.1f * yaw;

      // -------------------- Live UART PID Tuning --------------------
      kps = digit_buf[0] + (digit_buf[1] / 10.0f) + (digit_buf[2] / 100.0f);
      kis = (digit_buf[3] / 10.0f) + (digit_buf[4] / 100.0f) + (digit_buf[5] / 1000.0f);
      kds = (digit_buf[6] / 10.0f) + (digit_buf[7] / 100.0f) + (digit_buf[8] / 1000.0f);

      if (digit_ready == 1) {
         kp = kps;
         ki = kis;
         kd = kds;
         digit_ready = 0;
      }

      // -------------------- PID Computation --------------------
      PID_t pid_pitch, pid_roll, pid_yaw;
      PID_Init(&pid_pitch, kp, ki, kd, -90.0f, 90.0f);
      PID_Init(&pid_roll,  kp, ki, kd, -90.0f, 90.0f);
      PID_Init(&pid_yaw,   kp, ki, kd, -90.0f, 90.0f);

      float pitch_out = PID_Compute(&pid_pitch, 0.0f, pitch, dt);
      float roll_out  = PID_Compute(&pid_roll,  0.0f, roll,  dt);
      float yaw_out   = PID_Compute(&pid_yaw,   0.0f, yaw,   dt);

      // -------------------- Data Logging --------------------
      uint32_t time_us = TIM5->CNT;
      float t_ms = time_us / 1000.0f;
      int t_ms100 = (int)(t_ms * 100);

      char log_buf[128];
      sprintf(log_buf, "%d,%d,%d,%d,%d,%d,%d\r\n",
              t_ms100, (int)(pitch * 100), (int)(pitch_out * 100),
              (int)(roll * 100), (int)(roll_out * 100),
              (int)(yaw * 100), (int)(yaw_out * 100));
      LPUART_Print(log_buf);

      // -------------------- Output to Servos --------------------
      PWM_SetPulse(1, angle_to_pulse(+pitch_out));
      PWM_SetPulse(2, angle_to_pulse(-pitch_out));
      PWM_SetPulse(3, angle_to_pulse(+roll_out));
      PWM_SetPulse(4, angle_to_pulse(+roll_out));
      PWM_SetPulse(5, angle_to_pulse(-yaw_out));

      // -------------------- Serial Display Output --------------------
      char msg[160];
      sprintf(msg,
              "\x1B[H\x1B[?25lPitch=%d\xB0\r\nRoll=%d\xB0\r\nYaw=%d\xB0\r\n"
              "Kp = %d.%02d\r\nKi = 0.%03d\r\nKd = 0.%03d\r\n",
              (int)(-smooth_pitch - 0.5f),
              (int)(smooth_roll + 0.5f),
              (int)(smooth_yaw + 0.5f),
              (int)(kps * 100) / 100, abs((int)(kps * 100) % 100),
              (int)(kis * 1000), (int)(kds * 1000));
      LPUART_Print(msg);

      delay_us(1000);  // 1 ms delay for next iteration
   }
}

void SystemClock_Config(void)
{
   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

   if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
      Error_Handler();
   }

   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
   RCC_OscInitStruct.MSIState = RCC_MSI_ON;
   RCC_OscInitStruct.MSICalibrationValue = 0;
   RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
      Error_Handler();
   }

   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                 RCC_CLOCKTYPE_SYSCLK |
                                 RCC_CLOCKTYPE_PCLK1 |
                                 RCC_CLOCKTYPE_PCLK2;
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
      Error_Handler();
   }
}

void Error_Handler(void)
{
   __disable_irq();
   while (1) {
      // trap forever
   }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
   // user can print error location here
}
#endif


