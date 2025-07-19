#ifndef INC_PID_H_
#define INC_PID_H_

#include "stm32f4xx_hal.h"

/**
 * @brief   PID controller data structure
 */
typedef struct {
    /* Proportional gain */
    float kp;

    /* Integral time constant and stored integral term */
    float ti;
    float u_i_km1;      // Integral term at previous step (u_i[k-1])

    /* Derivative time constant, filter coefficient and stored derivative term */
    float td;
    float N;            // Filter pole (tuning parameter)
    float u_d_km1;      // Derivative term at previous step (u_d[k-1])

    /* Stored previous error and sampling period */
    float e_km1;        // Error at previous step (e[k-1])
    float tc;           // Sampling period (T_c)

    /* ---- for anti-windup ---- */
    float u_min;    // minimum output
    float u_max;    // maximum output
    float tw;       // tracking time constant
} Pid_s;

/**
 * Allocate and initialize a new PID instance
 * - tc  	Sampling period [seconds]
 * - kp  	Proportional gain
 * - ti  	Integral time constant [seconds]
 * - td  	Derivative time constant [seconds]
 * - N   	Derivative filter coefficient
 * - u_min 	minimum output
 * - u_max	maximum output
 * - tw		tracking time constant usually 10/ti
 * returns the pointer to initialized Pid_s
 */

/*
 * Specify the "abstract methods" that will be later implemented in the concrete .c class
 */

Pid_s * pid_create(float tc, float kp, float ti, float td, float N, float u_min, float u_max, float Tw);

/**
 * Compute the PID controller output
 * - pid        Pointer to PID instance
 * - measure    Current measurement y[k]
 * - reference  Desired setpoint r[k]
 * returns the control output u[k]
 */
float pid_calculate(Pid_s *pid, float measure, float reference);

#endif /* INC_PID_H_ */
