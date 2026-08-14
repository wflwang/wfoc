/*
 * pwm.h - PWM Driver for CIU32F003x5 (3-Phase Motor Control)
 *
 * Configures TIM1 in center-aligned mode with three complementary PWM
 * channels (CH1/CH1N, CH2/CH2N, CH3/CH3N), programmable dead-time,
 * and TRGO output for synchronized ADC sampling.
 *
 * Phase mapping:
 *   Phase A - TIM1_CH1 (PB2) / TIM1_CH1N (PB3)
 *   Phase B - TIM1_CH2 (PC1) / TIM1_CH2N (PB7)
 *   Phase C - TIM1_CH3 (PA1) / TIM1_CH3N (PA0)
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include "ciu32f003x.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* Channel Enumeration                                                       */
/* ======================================================================== */
typedef enum {
    PWM_CH_A = 0,   /* Phase A - TIM1_CH1 / CH1N */
    PWM_CH_B = 1,   /* Phase B - TIM1_CH2 / CH2N */
    PWM_CH_C = 2,   /* Phase C - TIM1_CH3 / CH3N */
    PWM_NUM_CHANNELS = 3U
} pwm_channel_t;

/* ======================================================================== */
/* Initialization & Control                                                  */
/* ======================================================================== */

/*! Initialize TIM1 center-aligned PWM with the given frequency.
 *  Configures GPIO, timer, dead-time, and TRGO for ADC sync.
 *  Does NOT start the PWM outputs - call pwm_enable() after setup.
 *  \param freq_hz  Desired PWM frequency (typically 8000-20000 Hz). */
void pwm_init(uint32_t freq_hz);

/*! Enable TIM1 PWM outputs (MOE bit in BDTR). */
void pwm_enable(void);

/*! Disable TIM1 PWM outputs (clears MOE). */
void pwm_disable(void);

/* ======================================================================== */
/* Duty Cycle Control                                                        */
/* ======================================================================== */

/*! Set duty cycle for a channel using Q16.16 fixed-point.
 *  \param channel  pwm_channel_t (PWM_CH_A, PWM_CH_B, PWM_CH_C)
 *  \param duty     Q16.16 fixed-point duty cycle (0.0 = 0%, 1.0 = 100%) */
void pwm_set_duty_q16(pwm_channel_t channel, int32_t duty);

/*! Set duty cycle for a channel using raw compare value.
 *  \param channel  pwm_channel_t
 *  \param duty     Raw CCR value (0 to ARR) */
void pwm_set_duty(pwm_channel_t channel, uint16_t duty);

/* ======================================================================== */
/* Timer Parameter Access                                                    */
/* ======================================================================== */

/*! Return the current auto-reload register (ARR) value. */
uint32_t pwm_get_arr(void);

/*! Set dead-time in nanoseconds.
 *  \param ns  Dead time in nanoseconds (0 to ~1000 ns max). */
void pwm_set_deadtime(uint32_t ns);

/*! Change the PWM frequency at runtime.
 *  \param freq_hz  New PWM frequency in Hz. */
void pwm_set_freq(uint32_t freq_hz);

#ifdef __cplusplus
}
#endif

#endif /* PWM_H */