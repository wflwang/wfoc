/*
 * fixedpoint.c - Fixed-Point Mathematics Library for FOC Motor Control
 *
 * Target: ARM Cortex-M0+ (CIU32F003x5) - No FPU, no hardware divide
 *
 * All trig/filter/control functions use integer-only arithmetic for
 * ISR safety. Float is used ONLY in q16_notch_init for coefficient
 * calculation (called once at startup, never from ISR).
 *
 * Sin/cos: 65-entry table (one quadrant, 260 bytes Flash) with
 * linear interpolation. Accuracy: < 0.01 deg error.
 *
 * atan2: polynomial approximation, max error ~0.22 deg.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include "fixedpoint.h"

/* ======================================================================== */
/* Sin Lookup Table - 65 entries for 0 to 90 degrees (64 intervals)         */
/* ======================================================================== */
/* sin_table[i] = sin(i * pi/128) in Q16.16,  i = 0..64                    */
/* Entry 0  = sin(0 deg)    = 0                                              */
/* Entry 64 = sin(90 deg)   = 65536 (1.0 in Q16.16)                        */
/* Flash usage: 65 * 4 = 260 bytes                                           */

static const q16_t sin_table[65] = {
    0, 1608, 3216, 4821, 6424, 8022, 9616, 11204,
    12785, 14359, 15924, 17479, 19024, 20557, 22078, 23586,
    25080, 26558, 28020, 29466, 30893, 32303, 33692, 35062,
    36410, 37736, 39040, 40320, 41576, 42806, 44011, 45190,
    46341, 47464, 48559, 49624, 50660, 51665, 52639, 53581,
    54491, 55368, 56212, 57022, 57798, 58538, 59244, 59914,
    60547, 61145, 61705, 62228, 62714, 63162, 63572, 63944,
    64277, 64571, 64827, 65043, 65220, 65358, 65457, 65516,
    65536
};

/* ======================================================================== */
/* Internal: Sin lookup with linear interpolation (first quadrant only)     */
/* ======================================================================== */
/* quad_angle: 0-16383 representing 0 to just under 90 degrees              */
/* index = quad_angle >> 8 (0-63), frac = quad_angle & 0xFF (0-255)        */

static q16_t sin_lookup(uint16_t quad_angle)
{
    uint8_t index = (uint8_t)(quad_angle >> 8);     /* 0-63                 */
    uint8_t frac  = (uint8_t)(quad_angle & 0xFF);   /* 0-255                */
    q16_t   s0    = sin_table[index];
    q16_t   s1    = sin_table[index + 1];            /* index+1 <= 64, safe  */

    return s0 + (q16_t)(((int32_t)(s1 - s0) * frac) >> 8);
}

/* ======================================================================== */
/* Trigonometric Functions                                                  */
/* ======================================================================== */
/* Angle: uint16_t 0-65535 = 0-360 degrees                                  */
/*   quadrant = angle >> 14     (0-3)                                       */
/*   quad_angle = angle & 0x3FFF (0-16383, within quadrant)                */

q16_t q16_sin(uint16_t angle)
{
    uint8_t  quadrant   = (uint8_t)(angle >> 14);
    uint16_t quad_angle = angle & 0x3FFF;

    switch (quadrant) {
    case 0:  /* 0 - 90 deg:   sin is positive, increasing  */
        return sin_lookup(quad_angle);
    case 1:  /* 90 - 180 deg: sin is positive, decreasing  */
        return sin_lookup(16383 - quad_angle);
    case 2:  /* 180 - 270 deg: sin is negative, magnitude increasing */
        return -sin_lookup(quad_angle);
    default: /* 270 - 360 deg: sin is negative, magnitude decreasing */
        return -sin_lookup(16383 - quad_angle);
    }
}

q16_t q16_cos(uint16_t angle)
{
    /* cos(angle) = sin(angle + 90 deg) = sin(angle + 16384)               */
    return q16_sin((uint16_t)(angle + 16384));
}

void q16_sincos(uint16_t angle, q16_t *sin_val, q16_t *cos_val)
{
    *sin_val = q16_sin(angle);
    *cos_val = q16_sin((uint16_t)(angle + 16384));
}

/* ======================================================================== */
/* Atan2 - Fixed-Point Arctangent of y/x                                    */
/* ======================================================================== */
/* Returns: uint16_t 0-65535 representing 0-360 degrees                     */
/* Uses polynomial approximation:                                           */
/*   atan(r) ~= r * (pi/4 + 0.273 * (1 - r))   for r in [0, 1]            */
/* Max error: ~0.22 degrees (sufficient for FOC observer/PLL)              */
/*                                                                           */
/* Constants:                                                               */
/*   0.273 in Q16.16 = 17892                                                */
/*   radians-to-angle16 factor = 65536 / (2*pi) = 10430                    */

#define ATAN_C       17892    /* 0.273 in Q16.16                             */
#define RAD_TO_ANG16 10430    /* 65536 / (2*pi), radians(Q16) to angle_16   */

uint16_t q16_atan2(q16_t y, q16_t x)
{
    q16_t   ax, ay, r, atan_val;
    uint16_t angle;

    /* Handle special cases to avoid division by zero */
    if (x == 0) {
        if (y == 0) return 0;
        return (y > 0) ? 16384 : 49152;   /* 90 deg or 270 deg */
    }
    if (y == 0) {
        return (x > 0) ? 0 : 32768;       /* 0 deg or 180 deg  */
    }

    ax = q16_abs(x);
    ay = q16_abs(y);

    /* Compute first-quadrant angle [0, 90 deg] */
    if (ay <= ax) {
        /* r = ay / ax in Q16.16, range [0, 1] */
        r = (q16_t)(((int64_t)ay << 16) / ax);
        /* atan(r) in Q16.16 radians */
        atan_val = Q16_MUL(r, Q16_PI_4 + Q16_MUL(ATAN_C, Q16_ONE - r));
        /* Convert Q16.16 radians to 16-bit angle */
        angle = (uint16_t)(((int64_t)atan_val * RAD_TO_ANG16) >> 16);
    } else {
        /* r = ax / ay in Q16.16, range [0, 1] */
        r = (q16_t)(((int64_t)ax << 16) / ay);
        /* atan(r) in Q16.16 radians */
        atan_val = Q16_MUL(r, Q16_PI_4 + Q16_MUL(ATAN_C, Q16_ONE - r));
        /* angle = 90 deg - atan(r) */
        angle = (uint16_t)(16384 - (uint16_t)(((int64_t)atan_val * RAD_TO_ANG16) >> 16));
    }

    /* Apply quadrant correction */
    if (x >= 0) {
        if (y >= 0) {
            return angle;                /* Q1: 0 - 90 deg    */
        } else {
            return (uint16_t)(65536 - angle);  /* Q4: 270 - 360 deg */
        }
    } else {
        if (y >= 0) {
            return (uint16_t)(32768 - angle);  /* Q2: 90 - 180 deg  */
        } else {
            return (uint16_t)(32768 + angle);  /* Q3: 180 - 270 deg */
        }
    }
}

/* ======================================================================== */
/* Integer Square Root (bit-by-bit method, no multiply/divide)              */
/* ======================================================================== */
/* Classic digit-by-digit computation. O(16) iterations for 32-bit input.   */
/* Result is floor(sqrt(x)).                                                */

uint32_t isqrt(uint32_t x)
{
    uint32_t res = 0;
    uint32_t bit = 1U << 30;   /* largest power of 4 <= 2^31               */

    /* Scale bit down to the largest power of 4 <= x */
    while (bit > x) {
        bit >>= 2;
    }

    /* Digit-by-digit square root */
    while (bit) {
        if (x >= res + bit) {
            x   -= res + bit;
            res  = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

/* ======================================================================== */
/* PID Controller                                                            */
/* ======================================================================== */

void q16_pid_init(q16_pid_t *pid, q16_t kp, q16_t ki, q16_t kd,
                  q16_t int_limit, q16_t out_limit)
{
    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->integral       = 0;
    pid->prev_error     = 0;
    pid->integral_limit = int_limit;
    pid->output_limit   = out_limit;
}

q16_t q16_pid_update(q16_pid_t *pid, q16_t error)
{
    q16_t p_term, i_term, d_term;
    q16_t output;

    /* Proportional term */
    p_term = Q16_MUL(pid->kp, error);

    /* Integral term with anti-windup (clamp accumulator) */
    pid->integral += error;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    i_term = Q16_MUL(pid->ki, pid->integral);

    /* Derivative term */
    d_term = Q16_MUL(pid->kd, error - pid->prev_error);
    pid->prev_error = error;

    /* Sum and saturate output */
    output = p_term + i_term + d_term;
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    return output;
}

/* ======================================================================== */
/* PI Controller for Current Loop                                           */
/* ======================================================================== */

void q16_pi_init(q16_pi_t *pi, q16_t kp, q16_t ki,
                 q16_t int_limit, q16_t out_limit)
{
    pi->kp             = kp;
    pi->ki             = ki;
    pi->integral       = 0;
    pi->integral_limit = int_limit;
    pi->output_limit   = out_limit;
}

q16_t q16_pi_update(q16_pi_t *pi, q16_t error)
{
    q16_t p_term, i_term;
    q16_t output;

    /* Proportional term */
    p_term = Q16_MUL(pi->kp, error);

    /* Predict next output to check for saturation BEFORE accumulating the
     * integral.  If the predicted output would saturate, freeze / reduce
     * the integral growth to prevent windup.                        */
    q16_t i_term_pred = Q16_MUL(pi->ki, pi->integral + error);
    q16_t output_pred = p_term + i_term_pred;
    uint8_t saturated_high = (output_pred >= pi->output_limit);
    uint8_t saturated_low  = (output_pred <= -pi->output_limit);

    if (saturated_high && error > 0) {
        /* Saturated positive and still trying to increase: freeze integral */
    } else if (saturated_low && error < 0) {
        /* Saturated negative and still trying to decrease: freeze integral */
    } else {
        /* Not saturated, or saturated but error pulling back: integrate */
        pi->integral += error;
        if (pi->integral > pi->integral_limit) {
            pi->integral = pi->integral_limit;
        } else if (pi->integral < -pi->integral_limit) {
            pi->integral = -pi->integral_limit;
        }
    }
    i_term = Q16_MUL(pi->ki, pi->integral);

    /* Sum and saturate output */
    output = p_term + i_term;
    if (output > pi->output_limit) {
        output = pi->output_limit;
    } else if (output < -pi->output_limit) {
        output = -pi->output_limit;
    }

    return output;
}

/* ======================================================================== */
/* Notch Filter - Second-Order IIR                                          */
/* ======================================================================== */
/* Coefficient calculation uses float (NON-ISR, called once at init).       */
/* Update function uses only integer Q16_MUL operations (ISR-safe).         */
/*                                                                           */
/* Transfer function:                                                       */
/*   H(z) = (1 - 2*cos(w0)*z^-1 + z^-2) / (1 - 2*r*cos(w0)*z^-1 + r^2*z^-2)*/
/* Difference equation (Direct Form I):                                     */
/*   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] + a1*y[n-1] + a2*y[n-2]        */

void q16_notch_init(q16_notch_t *nf, uint16_t freq,
                    uint16_t sample_freq, uint16_t bandwidth)
{
    /* Notch angle in 16-bit format: angle = freq * 65536 / sample_freq    */
    uint16_t notch_angle = (uint16_t)((uint32_t)freq * 65536U / sample_freq);

    /* cos(w0) using our integer sin/cos table (no float needed) */
    q16_t cos_w = q16_cos(notch_angle);

    /* r = 1 - pi * bandwidth / sample_freq                                */
    /* r_q16 = 65536 - (Q16_PI * bandwidth) / sample_freq                  */
    /* Use 64-bit to avoid overflow: Q16_PI * bandwidth can exceed 2^31    */
    q16_t r = (q16_t)(65536 - (q16_t)(((int64_t)Q16_PI * bandwidth) / sample_freq));

    /* Clamp r to [0, Q16_ONE] to ensure stability */
    if (r < 0) r = 0;
    if (r > Q16_ONE) r = Q16_ONE;

    /* Feedforward coefficients (zeros on unit circle at +/- w0) */
    nf->b0 = Q16_ONE;              /* 1.0                                    */
    nf->b1 = -(cos_w << 1);        /* -2 * cos(w0)                           */
    nf->b2 = Q16_ONE;              /* 1.0                                    */

    /* Feedback coefficients (poles at r * angle +/- w0) */
    nf->a1 = (q16_t)(Q16_MUL(r, cos_w) << 1);  /* 2 * r * cos(w0)          */
    nf->a2 = -Q16_MUL(r, r);                    /* -r^2                    */

    /* Clear filter state */
    nf->x1 = 0;
    nf->x2 = 0;
    nf->y1 = 0;
    nf->y2 = 0;
}

q16_t q16_notch_update(q16_notch_t *nf, q16_t x)
{
    q16_t y;

    /* Direct Form I: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]              */
    /*                   + a1*y[n-1] + a2*y[n-2]                          */
    y = Q16_MUL(nf->b0, x)
      + Q16_MUL(nf->b1, nf->x1)
      + Q16_MUL(nf->b2, nf->x2)
      + Q16_MUL(nf->a1, nf->y1)
      + Q16_MUL(nf->a2, nf->y2);

    /* Shift history */
    nf->x2 = nf->x1;
    nf->x1 = x;
    nf->y2 = nf->y1;
    nf->y1 = y;

    return y;
}
