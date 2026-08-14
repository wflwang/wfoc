/*
 * fixedpoint.h - Fixed-Point Mathematics Library for FOC Motor Control
 *
 * Target: ARM Cortex-M0+ (CIU32F003x5) - No FPU, no hardware divide
 * All ISR-safe functions use ONLY integer operations (shift, add, multiply).
 * Float conversions are provided for initialization only (NON-ISR).
 *
 * Q16.16 format: 32-bit signed, 16 integer bits, 16 fractional bits
 *   Range: -32768.0 to +32767.99998
 *   Resolution: ~1.5e-5
 *
 * Q24.8 format: 32-bit signed, 24 integer bits, 8 fractional bits
 *   Range: -8388608.0 to +8388607.99609
 *   Resolution: ~3.9e-3  (used for angles in degrees: 0-360 with 1/256 deg)
 *
 * Angle format (trig functions): uint16_t 0-65535 = 0-360 degrees
 *   16384 = 90 deg, 32768 = 180 deg, 49152 = 270 deg
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef FIXEDPOINT_H
#define FIXEDPOINT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* Fixed-Point Type Definitions                                              */
/* ======================================================================== */

typedef int32_t q16_t;   /* Q16.16: general FOC calculations               */
typedef int32_t q24_t;   /* Q24.8:  angles in degrees (0-360, 1/256 deg)  */

/* ======================================================================== */
/* Q16.16 Constants                                                          */
/* ======================================================================== */

#define Q16_ONE       65536      /* 1.0                                      */
#define Q16_HALF      32768      /* 0.5                                      */
#define Q16_TWO       131072     /* 2.0                                      */
#define Q16_PI        205887     /* pi                                       */
#define Q16_2PI       411775     /* 2 * pi                                   */
#define Q16_PI_2      102944     /* pi / 2                                   */
#define Q16_PI_3      68629      /* pi / 3  (60 deg, SVPWM sectors)          */
#define Q16_PI_4      51472      /* pi / 4  (45 deg)                         */
#define Q16_PI_6      34181      /* pi / 6  (30 deg)                         */
#define Q16_SQRT3     113512     /* sqrt(3)  (Clarke transform)              */
#define Q16_SQRT3_2   56756      /* sqrt(3)/2 (Clarke transform)             */

/* ======================================================================== */
/* Q24.8 Constants and Macros                                                */
/* ======================================================================== */

#define Q24_ONE       256        /* 1.0                                      */
#define Q24_360       92160      /* 360.0 degrees                            */

#define INT_TO_Q24(x)   ((q24_t)((int32_t)(x) << 8))
#define Q24_TO_INT(x)   ((int)((x) >> 8))
#define FLOAT_TO_Q24(x) ((q24_t)((float)(x) * 256.0f))
#define Q24_TO_FLOAT(x) ((float)(x) / 256.0f)

/* ======================================================================== */
/* Q16.16 Arithmetic Macros (ISR-safe: integer multiply + shift only)       */
/* ======================================================================== */
/* CAUTION: These are macros - avoid side-effect arguments (e.g. i++).      */
/* Q16_MUL uses 64-bit intermediate to prevent overflow during multiply.    */
/* Q16_DIV uses 64-bit division (slow on M0+ - avoid in ISR if possible).   */

/* Multiply two Q16.16 values:  result = (a * b) >> 16                      */
#define Q16_MUL(a, b)   ((q16_t)(((int64_t)(a) * (int64_t)(b)) >> 16))

/* Divide two Q16.16 values:   result = (a << 16) / b                       */
#define Q16_DIV(a, b)   ((q16_t)(((int64_t)(a) << 16) / (b)))

/* Convert signed int to Q16.16                                             */
#define INT_TO_Q16(x)   ((q16_t)((int32_t)(x) << 16))

/* Convert Q16.16 to signed int (floor toward negative infinity)            */
#define Q16_TO_INT(x)   ((int)((x) >> 16))

/* Convert float to Q16.16  (NON-ISR ONLY - uses FPU)                       */
#define FLOAT_TO_Q16(x) ((q16_t)((float)(x) * 65536.0f))

/* Convert Q16.16 to float  (NON-ISR ONLY - uses FPU)                       */
#define Q16_TO_FLOAT(x) ((float)(x) / 65536.0f)

/* ======================================================================== */
/* Saturation and Absolute Value (static inline - ISR-safe)                 */
/* ======================================================================== */

/* Saturate val to [min, max] range */
static inline q16_t q16_sat(q16_t val, q16_t min, q16_t max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/* Absolute value (undefined for INT32_MIN - should not occur in FOC) */
static inline q16_t q16_abs(q16_t val)
{
    return (val >= 0) ? val : -val;
}

/* ======================================================================== */
/* Trigonometric Functions (lookup table + linear interpolation)            */
/* ======================================================================== */
/* Angle input:  uint16_t 0-65535 representing 0-360 degrees                */
/* Sin table: 65 entries for one quadrant (0-90 deg), 260 bytes Flash       */

q16_t  q16_sin(uint16_t angle);
q16_t  q16_cos(uint16_t angle);
void   q16_sincos(uint16_t angle, q16_t *sin_val, q16_t *cos_val);

/* atan2: returns angle 0-65535 (0-360 deg)                                 */
/* Max error ~0.22 deg - acceptable for FOC observer/PLL                     */
uint16_t q16_atan2(q16_t y, q16_t x);

/* ======================================================================== */
/* Integer Square Root (ISR-safe: bit-by-bit, no multiply/divide)           */
/* ======================================================================== */

uint32_t isqrt(uint32_t x);

/* ======================================================================== */
/* Low-Pass Filter (ISR-safe: one multiply-accumulate)                      */
/* ======================================================================== */
/* y[n] = y[n-1] + alpha * (x[n] - y[n-1])                                 */
/* alpha_q16: 0 = no update, 65536 = no filtering (pass-through)            */
/* Typical: alpha = dt / (tau + dt), pre-computed in Q16.16                 */

typedef struct {
    q16_t y;           /* filtered output (init to 0 or initial value)      */
    q16_t alpha_q16;   /* filter coefficient in Q16.16 (0 to 65536)         */
} q16_lpf_t;

static inline q16_t q16_lpf_update(q16_lpf_t *lpf, q16_t x)
{
    lpf->y += (q16_t)(((int64_t)(x - lpf->y) * lpf->alpha_q16) >> 16);
    return lpf->y;
}

/* ======================================================================== */
/* PID Controller (ISR-safe: integer multiply-accumulate)                   */
/* ======================================================================== */
/* Position-form PID with integral anti-windup and output saturation.       */
/* ki should include sample time:  ki = Ki * Ts                              */
/* kd should include sample time:  kd = Kd / Ts                              */

typedef struct {
    q16_t kp;              /* proportional gain (Q16.16)                    */
    q16_t ki;              /* integral gain, includes Ts (Q16.16)           */
    q16_t kd;              /* derivative gain, includes 1/Ts (Q16.16)       */
    q16_t integral;        /* integral accumulator (init to 0)              */
    q16_t prev_error;      /* previous error for derivative (init to 0)     */
    q16_t integral_limit;  /* anti-windup: clamp |integral| to this (Q16.16)*/
    q16_t output_limit;    /* output saturation: clamp |output| to this     */
} q16_pid_t;

void  q16_pid_init(q16_pid_t *pid, q16_t kp, q16_t ki, q16_t kd,
                   q16_t int_limit, q16_t out_limit);
q16_t q16_pid_update(q16_pid_t *pid, q16_t error);

/* ======================================================================== */
/* PI Controller for Current Loop (ISR-safe)                                */
/* ======================================================================== */
/* PI controller without derivative term, optimized for current loop.       */
/* ki should include sample time:  ki = Ki * Ts                              */

typedef struct {
    q16_t kp;              /* proportional gain (Q16.16)                    */
    q16_t ki;              /* integral gain, includes Ts (Q16.16)           */
    q16_t integral;        /* integral accumulator (init to 0)              */
    q16_t integral_limit;  /* anti-windup: clamp |integral| to this         */
    q16_t output_limit;    /* output saturation: clamp |output| to this     */
} q16_pi_t;

void  q16_pi_init(q16_pi_t *pi, q16_t kp, q16_t ki,
                  q16_t int_limit, q16_t out_limit);
q16_t q16_pi_update(q16_pi_t *pi, q16_t error);

/* ======================================================================== */
/* Notch Filter - Second-Order IIR (ISR-safe update)                        */
/* ======================================================================== */
/* Difference equation (Direct Form I):                                     */
/*   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] + a1*y[n-1] + a2*y[n-2]        */
/* Transfer function:                                                       */
/*   H(z) = (1 - 2*cos(w0)*z^-1 + z^-2) / (1 - 2*r*cos(w0)*z^-1 + r^2*z^-2)*/
/* where w0 = 2*pi*f0/fs, r = 1 - pi*BW/fs                                  */
/* Coefficients computed in q16_notch_init (may use float internally).      */

typedef struct {
    q16_t a1, a2;       /* feedback coefficients (Q16.16)                   */
    q16_t b0, b1, b2;   /* feedforward coefficients (Q16.16)                */
    q16_t x1, x2;       /* input history (init to 0)                        */
    q16_t y1, y2;       /* output history (init to 0)                       */
} q16_notch_t;

void  q16_notch_init(q16_notch_t *nf, uint16_t freq,
                     uint16_t sample_freq, uint16_t bandwidth);
q16_t q16_notch_update(q16_notch_t *nf, q16_t x);

#ifdef __cplusplus
}
#endif

#endif /* FIXEDPOINT_H */
