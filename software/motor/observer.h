/*
 * observer.h - Rotor Position/Speed Observers (Q16.16)
 *
 * Sensorless rotor position and speed estimation for FOC. Implements three
 * observer families plus a shared phase-locked loop (PLL):
 *
 *   - SMO      : Sliding Mode Observer. Robust back-EMF estimation via a
 *                switching term; angle from atan2 of the filtered EMF.
 *   - MXLEMMA  : Full-order flux-linkage observer (VESC-style). Integrates
 *                stator flux and corrects with a current-error term.
 *   - HFI      : High-Frequency Injection for low/zero-speed saliency
 *                tracking; demodulates the q-axis current response.
 *   - PLL      : Extracts a smooth angle and speed from the raw observer
 *                angle (used by SMO and MXLEMMA).
 *
 * Target: ARM Cortex-M0+ (CIU32F003x5) - No FPU.
 * All ISR-time math uses Q16.16 fixed-point (no floating-point).
 *
 * Angle format: uint16_t 0..65535 = 0..360 electrical degrees.
 * Speed format: q16_t electrical rad/s (Q16.16).
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under the MIT License
 */

#ifndef OBSERVER_H
#define OBSERVER_H

#include <stdint.h>
#include "util/fixedpoint.h"
#include "conf/datatypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* Observer State                                                            */
/* ======================================================================== */
typedef struct {
    /* --- SMO (Sliding Mode Observer) --- */
    q16_t smo_alpha;            /* Sliding gain (Q16.16)                     */
    q16_t smo_ialpha_est;       /* Estimated alpha current (Q16.16 A)       */
    q16_t smo_ibeta_est;        /* Estimated beta current  (Q16.16 A)       */
    q16_t smo_valpha_est;       /* Estimated alpha voltage (Q16.16 V)        */
    q16_t smo_vbeta_est;        /* Estimated beta voltage  (Q16.16 V)       */
    q16_t smo_zalpha;           /* Switching signal alpha (Q16.16)          */
    q16_t smo_zbeta;            /* Switching signal beta  (Q16.16)           */
    q16_t smo_ealpha;           /* Filtered back-EMF alpha (Q16.16 V)       */
    q16_t smo_ebeta;            /* Filtered back-EMF beta  (Q16.16 V)       */
    q16_lpf_t smo_lpf_alpha;    /* LPF on switching signal alpha             */
    q16_lpf_t smo_lpf_beta;     /* LPF on switching signal beta              */

    /* --- MXLEMMA observer --- */
    q16_t mxl_ialpha;           /* Estimated alpha current (Q16.16 A)       */
    q16_t mxl_ibeta;            /* Estimated beta current  (Q16.16 A)       */
    q16_t mxl_lambda_alpha;     /* Estimated flux alpha (Q16.16 Wb)         */
    q16_t mxl_lambda_beta;      /* Estimated flux beta  (Q16.16 Wb)         */
    q16_t mxl_gamma;            /* Observer gain (Q16.16)                    */
    q16_t mxl_ealpha;           /* Back-EMF alpha (Q16.16 V)                 */
    q16_t mxl_ebeta;            /* Back-EMF beta  (Q16.16 V)                 */

    /* --- PLL for angle/speed extraction --- */
    q16_t pll_kp;               /* Proportional gain (Q16.16)                */
    q16_t pll_ki;               /* Integral gain (Q16.16)                   */
    q16_t pll_integral;        /* Integral accumulator (Q16.16)              */
    uint16_t angle;             /* Estimated angle 0..65535                  */
    q16_t speed;                /* Estimated speed (Q16.16 rad/s)            */

    /* --- Dynamic PLL gains (updated by foc_low_speed_update) ---
     * These are the gains actually used by pll_update.  They start
     * equal to pll_kp_gain / pll_ki_gain and are blended toward low-
     * speed values by the FOC module as the motor decelerates.         */
    q16_t pll_kp_gain_eff;      /* Effective kp used in PLL (Q16.16)         */
    q16_t pll_ki_gain_eff;      /* Effective ki used in PLL (Q16.16)         */

    /* --- HFI (High Frequency Injection) --- */
    uint16_t hfi_angle;         /* HFI-estimated angle (0..65535)           */
    q16_t hfi_amplitude;        /* Injection voltage amplitude (Q16.16 V)   */
    uint16_t hfi_freq;          /* Injection frequency (Hz)                 */
    q16_t hfi_demod_d;          /* Demodulated d-axis signal (Q16.16)       */
    q16_t hfi_demod_q;          /* Demodulated q-axis signal (Q16.16)       */
    uint16_t hfi_phase;         /* Injection carrier phase (0..65535)       */
    uint16_t hfi_phase_inc;     /* Carrier phase increment per ISR (angle_16)*/
    q16_t hfi_prev_id;          /* Previous d-axis current sample (Q16.16) */
    q16_t hfi_prev_iq;          /* Previous q-axis current sample (Q16.16) */
    q16_t hfi_track_gain;       /* HFI angle-tracking loop gain (Q16.16)    */
    q16_lpf_t hfi_lpf_d;        /* LPF on demodulated d signal               */
    q16_lpf_t hfi_lpf_q;        /* LPF on demodulated q signal               */

    /* --- Config --- */
    observer_type_t type;       /* Active observer type                     */
    mc_configuration_t *config; /* Pointer to motor configuration           */

    /* --- Precomputed ISR gains (set in observer_init, float used once) --- */
    q16_t dt_q16;               /* Sample period (Q16.16 s)                 */
    q16_t r_q16;                /* Phase resistance (Q16.16 ohm)            */
    q16_t l_q16;                /* Phase inductance (Q16.16 H)              */
    q16_t inv_l_q16;            /* 1/L (Q16.16)                             */
    q16_t flux_linkage_q16;     /* Flux linkage (Q16.16 Wb)                 */
    q16_t k_ohm_dt_l;          /* R*dt/L precomputed (Q16.16)              */
    q16_t k_dt_l;              /* dt/L precomputed (Q16.16)                 */
    q16_t pll_kp_gain;          /* PLL kp (Q16.16, 1/s)                      */
    q16_t pll_ki_gain;          /* PLL ki * dt (Q16.16, 1/s)                */
    q16_t pll_angle_rad;        /* PLL tracked angle in radians (Q16.16)    */
    q16_t pll_speed_limit;      /* Anti-windup clamp on speed (Q16.16 rad/s)*/
    uint16_t valid_counter;     /* ISR ticks until angle is deemed valid     */
    bool   valid;               /* True once a usable angle is available    */
} observer_state_t;

/* ======================================================================== */
/* Public API                                                                */
/* ======================================================================== */

/* Initialize the observer from a motor configuration. Called once at startup
 * (NON-ISR; uses float to precompute Q16 gains from the config).        */
void observer_init(observer_state_t *obs, mc_configuration_t *conf);

/* Reset the observer runtime state (currents, flux, PLL, HFI). The precomputed
 * gains and config pointer are preserved. Safe to call from ISR context. */
void observer_reset(observer_state_t *obs);

/* Run one observer update from ISR. valpha/vbeta are the applied alpha-beta
 * voltages (Q16.16 V), ialpha/ibeta the measured currents (Q16.16 A), vbus the
 * DC bus voltage (Q16.16 V). All arithmetic is integer/fixed-point. */
void observer_run_isr(observer_state_t *obs, q16_t valpha, q16_t vbeta,
                      q16_t ialpha, q16_t ibeta, q16_t vbus);

/* Return the estimated electrical angle (0..65535). For HFI returns the HFI
 * angle at low speed, otherwise the PLL-tracking angle. */
uint16_t observer_get_angle(observer_state_t *obs);

/* Return the estimated electrical speed (Q16.16 rad/s). */
q16_t observer_get_speed(observer_state_t *obs);

/* Return true once the observer has converged and its angle is usable. */
bool observer_is_valid(observer_state_t *obs);

/* Adjust the PLL gains on-the-fly (ISR-safe).
 * Called by the FOC low-speed scheduler to blend gains toward their
 * low-speed values.  kp_eff / ki_eff are the new Q16.16 gains.         */
void observer_set_pll_gains_isr(observer_state_t *obs,
                                q16_t kp_eff, q16_t ki_eff);

#ifdef __cplusplus
}
#endif

#endif /* OBSERVER_H */
