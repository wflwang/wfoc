/*
 * foc.h - FOC Core Controller (Q16.16 Fixed-Point)
 *
 * Implements the complete FOC inner / outer control loops:
 *   - Park / inverse Park transforms (ISR-safe)
 *   - d/q axis current PI controllers with on-line gain scheduling
 *   - d/q axis cross-coupling decoupling (omega*L*i)
 *   - Back-EMF feedforward (omega*psi)
 *   - Speed PI controller with speed feedforward
 *   - Low-speed / standstill gain boost with hysteresis
 *   - R/L temperature compensation (called at slower rate)
 *
 * Target: ARM Cortex-M0+ (CIU32F003x5) - No FPU, no hardware divide.
 * All ISR-time math uses Q16.16 fixed-point arithmetic only.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under the MIT License
 */

#ifndef FOC_H
#define FOC_H

#include <stdint.h>
#include "util/fixedpoint.h"
#include "conf/datatypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* Initialization                                                           */
/* ======================================================================== */

/* Initialize the FOC state from a motor configuration.
 * Uses float only (non-ISR). Must be called once after conf is loaded,
 * before the first ADC interrupt. */
void foc_init(foc_state_t *foc, mc_configuration_t *conf);

/* Reset runtime integrators and setpoints (start/stop).
 * ISR-safe. */
void foc_reset(foc_state_t *foc);

/* ======================================================================== */
/* ISR-Time: FOC Current Loop (called from ADC1_IRQHandler @ PWM rate)      */
/* ======================================================================== */

/* Run one PWM-cycle of the FOC inner loop.
 *   ialpha, ibeta : measured alpha/beta currents (Q16.16 A)
 *   vbus          : DC bus voltage (Q16.16 V)
 *   angle         : electrical rotor angle (0..65535)
 *
 * After the call foc->vd / foc->vq / foc->valpha / foc->vbeta are updated
 * and ready for the SVPWM stage.  Uses ONLY integer arithmetic.           */
void foc_current_loop_isr(foc_state_t *foc, mc_configuration_t *conf,
                          q16_t ialpha, q16_t ibeta, q16_t vbus,
                          uint16_t angle);

/* ======================================================================== */
/* Slow-Rate Tasks (called from main loop @ ~100 Hz)                         */
/* ======================================================================== */

/* Update temperature-compensated R, L, psi values.
 * Reads motor_state.temp_motor / temp_mosfet and updates
 * foc->r_q16 / l_q16 / psi_q16 accordingly.  Uses float internally; safe
 * to call from non-ISR context.                                           */
void foc_update_temp_comp(foc_state_t *foc, mc_configuration_t *conf,
                          float temp_motor, float temp_mosfet);

/* Run the speed outer loop (1 kHz call rate).
 * Computes Iq_ref from speed error + feedforward.
 * If conf->foc_speed_ff_en, adds speed_ff * speed_ref.
 * Uses Q16.16 arithmetic; ISR-safe.                                       */
void foc_speed_loop(foc_state_t *foc, mc_configuration_t *conf);

/* Run the position outer loop (100 Hz call rate).
 * Computes speed_ref from position error + feedforward.
 * If conf->foc_pos_en, the position loop overrides speed_set.
 * Uses Q16.16 arithmetic; ISR-safe.                                       */
void foc_position_loop(foc_state_t *foc, mc_configuration_t *conf);

/* Run field weakening control (1 kHz call rate).
 * Injects negative d-axis current when voltage margin is low.
 * Extends the constant power speed range.
 * Uses Q16.16 arithmetic; ISR-safe.                                       */
void foc_field_weakening(foc_state_t *foc, mc_configuration_t *conf);

/* Advance the startup current ramp (1 kHz call rate).
 * If the motor is still nearly stopped (|ERPM| < start_speed_thr) and
 * iq_set was stepped by more than foc->start_step_current_q16, linearly
 * ramp iq_set from its previous value toward the new target over
 * foc->start_ramp_ticks_max_q16 1 kHz ticks.  After the ramp finishes
 * (or the motor spins up) iq_set = start_iq_target.
 *
 * This prevents a large, "jerking" start when the observer has not
 * fully converged yet, which is the main cause of uneven rotation at
 * low speed in VESC-style firmware.  ISR-safe.                            */
void foc_start_ramp_step(foc_state_t *foc, mc_configuration_t *conf);

/* ======================================================================== */
/* Low-Speed Gain Scheduling (ISR-safe)                                     */
/* ======================================================================== */

/* Blends current-loop PI gains and PLL gains between normal-speed and
 * low-speed values based on the measured ERPM, with hysteresis.
 *   speed_erpm : mechanical speed in ERPM (Q16.16)
 * ISR-safe. */
void foc_low_speed_update(foc_state_t *foc, mc_configuration_t *conf,
                          q16_t speed_erpm);

/* Extended version that also forwards the blended PLL gains to the
 * observer state so the PLL tracks them immediately.  obs is a
 * pointer to observer_state_t (passed as void* to avoid header
 * dependency on observer.h).                                            */
void foc_low_speed_update_with_observer(foc_state_t *foc,
                                         mc_configuration_t *conf,
                                         q16_t speed_erpm,
                                         void *obs);

/* ======================================================================== */
/* Utility: Electrical speed <-> ERPM conversion (non-ISR)                  */
/* ======================================================================== */

/* Convert observer electrical speed (Q16.16 rad/s) to mechanical ERPM.
 * poles = number of pole pairs.  Float used, call outside ISR.             */
float foc_radps_to_erpm(q16_t speed_rad_q16, uint8_t poles);

#ifdef __cplusplus
}
#endif

#endif /* FOC_H */
