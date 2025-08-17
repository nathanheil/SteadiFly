/* -----------------------------------------------------------------------------
 * EE 329 - PID CONTROL MODULE
 *
 * @file     : pid.c
 * @brief    : Implements a configurable PID controller with saturation limits
 * @project  : EE329 Closed-Loop Control Systems
 * @author   : Nathan H., Dante B., Benjamin T., Gulianna C.
 * @version  : 1.0
 * @date     : 6/5/25
 * @target   : STM32 NUCLEO-L4A6ZG
 * -------------------------------------------------------------------------- */

#include "pid.h"

/* -----------------------------------------------------------------------------
 * PID_Init()
 * Initializes PID gain parameters and output constraints.
 *
 * pid       - pointer to PID_t struct instance
 * kp        - proportional gain
 * ki        - integral gain
 * kd        - derivative gain
 * min_out   - minimum output saturation limit
 * max_out   - maximum output saturation limit
 * -------------------------------------------------------------------------- */
void PID_Init(PID_t* pid, float kp, float ki, float kd, float min_out, float max_out) {
   pid->kp         = kp;
   pid->ki         = ki;
   pid->kd         = kd;
   pid->prev_error = 0.0f;
   pid->integral   = 0.0f;
   pid->output_min = min_out;
   pid->output_max = max_out;
}

/* -----------------------------------------------------------------------------
 * PID_Compute()
 * Calculates the PID control output based on current error.
 * Applies anti-windup clamping to output limits.
 *
 * pid        - pointer to PID_t struct instance
 * setpoint   - target value
 * measurement- current system value
 * dt         - time step in seconds
 * returns    - PID output (clamped within bounds)
 * -------------------------------------------------------------------------- */
float PID_Compute(PID_t* pid, float setpoint, float measurement, float dt) {
   float error      = setpoint - measurement;
   pid->integral   += error * dt;
   float derivative = (error - pid->prev_error) / dt;

   float output = (pid->kp * error) +
                  (pid->ki * pid->integral) +
                  (pid->kd * derivative);

   if (output > pid->output_max) {
      output = pid->output_max;
   } else if (output < pid->output_min) {
      output = pid->output_min;
   }

   pid->prev_error = error;
   return output;
}



