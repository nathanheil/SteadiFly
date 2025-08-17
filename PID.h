/* -----------------------------------------------------------------------------
 * EE 329 - PID CONTROL HEADER
 *
 * @file     : pid.h
 * @brief    : PID controller data structure and function declarations
 * @project  : EE329 Closed-Loop Control Systems
 * @author   : Nathan H., Dante B., Benjamin T., Gulianna C.
 * @version  : 1.0
 * @date     : 6/5/25
 * @target   : STM32 NUCLEO-L4A6ZG
 * -------------------------------------------------------------------------- */

#ifndef PID_H
#define PID_H

#include <stdint.h>

/* -----------------------------------------------------------------------------
 * PID_t Structure
 * Stores gain parameters, state variables, and output constraints.
 * -------------------------------------------------------------------------- */
typedef struct {
   float kp;            // Proportional gain
   float ki;            // Integral gain
   float kd;            // Derivative gain
   float prev_error;    // Previous error (for derivative)
   float integral;      // Accumulated integral
   float output_min;    // Minimum output limit
   float output_max;    // Maximum output limit
} PID_t;

/* -----------------------------------------------------------------------------
 * Function Prototypes
 * -------------------------------------------------------------------------- */
void  PID_Init(PID_t* pid,
               float kp,
               float ki,
               float kd,
               float min_out,
               float max_out);   // Initializes PID gains and constraints

float PID_Compute(PID_t* pid,
                  float setpoint,
                  float measurement,
                  float dt);     // Computes PID output based on input and dt

#endif /* PID_H */



