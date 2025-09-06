#include "pid.h"
#include <stdlib.h>

/**
 *  Allocate and initialize PID structure
 */
Pid_s * pid_create(float tc, float kp, float ti, float td, float N, float u_min, float u_max, float tw)
{
    Pid_s *pid = malloc(sizeof(Pid_s));
    if (!pid) return NULL;

    pid->tc     = tc;
    pid->kp     = kp;
    pid->ti     = ti;
    pid->td     = td;
    pid->N      = N;

    /* Anti-windup */
    pid->u_min   = u_min;
    pid->u_max   = u_max;
    pid->tw      = tw;

    /* Clear previous state */
    pid->e_km1   = 0.0f;
    pid->u_i_km1 = 0.0f;
    pid->u_d_km1 = 0.0f;

    return pid;
}

/**
 * Calculate PID output using discrete form with derivative filtering
 */
float pid_calculate(Pid_s *pid, float measure, float reference)
{
    /* 1) Error:
       e[k] = r[k] – y[k] */
    float e_k = reference - measure;

    /* 2) Filtered derivative:
       a = T_d / (N·T_c)
       u_d[k] = (a·u_d[k–1] + K_p·(T_d/T_c)·(e[k] – e[k–1])) / (a + 1) */
    float a    = pid->td/(pid->N*pid->tc);
    float num  = a * pid->u_d_km1
               + pid->kp*(pid->td/pid->tc)*(e_k - pid->e_km1);
    float den  = a + 1.0f;
    float u_dk = num / den;

    /* 3) Tentative integral (incremental form):
       u_i[k] = u_i[k–1] + K_p·(T_c/T_i)·e[k] */
    float u_ik = pid->u_i_km1
               + pid->kp*(pid->tc/pid->ti)*e_k;

    /* 4) Unsaturated control:
       u_unsat[k] = K_p·e[k] + u_i[k] + u_d[k] */
    float u_unsat = pid->kp*e_k + u_ik + u_dk;

    /* 5) Saturation:
       u_sat[k] = clamp(u_unsat[k], u_min, u_max) */
    float u_sat = u_unsat;
    if (u_sat > pid->u_max)      u_sat = pid->u_max;
    else if (u_sat < pid->u_min) u_sat = pid->u_min;

    /* 6) Anti-windup (back-calculation):
       e_t[k] = u_sat[k] – u_unsat[k]
       u_i[k] += (T_c/T_w)·e_t[k] */
    float e_t = u_sat - u_unsat;
    u_ik += (pid->tc/pid->tw)*e_t;

    /* 7) Update internal state for next step */
    pid->e_km1   = e_k;
    pid->u_i_km1 = u_ik;
    pid->u_d_km1 = u_dk;

    /* 8) Return saturated output:
       u[k] = u_sat[k] */
    return u_sat;
}
