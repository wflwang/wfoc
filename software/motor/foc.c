/*
 * foc.c - FOC Core Controller (Q16.16 Fixed-Point Implementation)
 *
 * Provides the complete FOC control algorithm used by the WFOC firmware.
 *
 * ISR time budget target: < 4us @ 20 kHz on Cortex-M0+ (CIU32F003x5).
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under the MIT License
 */

#include <stdint.h>
#include <string.h>
#include "foc.h"
#include "conf/datatypes.h"
#include "motor/observer.h"
#include "config/hwconf.h"

/* ======================================================================== */
/* Precomputed Q16 constants (computed once in foc_init)                    */
/* ======================================================================== */

/* 1/sqrt(3) for Clarke transform beta axis.  Already defined in interrupts.c,
 * but we keep a local copy here for clarity / if this module is used alone.*/
#define FOC_1_SQRT3_Q16   ((q16_t)37837)  /* 1/sqrt(3) in Q16.16 */

/* rad/s -> ERPM conversion factor:
 *   1 rad/s = 60/(2*pi*poles) ERPM
 * For poles=7: factor = 60/(2*pi*7) ~= 1.364
 * We precompute as Q16.16 per-pole.  User multiplies by 1/poles.            */
#define RADPS_TO_RPM_PER_POLE  ((q16_t)891929)  /* 60/(2*pi) in Q16.16 */

/* ======================================================================== */
/* foc_init                                                                  */
/* ======================================================================== */
void foc_init(foc_state_t *foc, mc_configuration_t *conf)
{
    memset(foc, 0, sizeof(*foc));

    /* Current loop PI (d-axis and q-axis use same gains by default). */
    q16_pi_init(&foc->pid_id,
                FLOAT_TO_Q16(conf->foc_current_kp),
                FLOAT_TO_Q16(conf->foc_current_ki),
                FLOAT_TO_Q16(conf->l_max_current),
                FLOAT_TO_Q16(conf->l_max_duty));

    q16_pi_init(&foc->pid_iq,
                FLOAT_TO_Q16(conf->foc_current_kp),
                FLOAT_TO_Q16(conf->foc_current_ki),
                FLOAT_TO_Q16(conf->l_max_current),
                FLOAT_TO_Q16(conf->l_max_duty));

    /* Speed loop PI.  Output is Iq (A), input is speed error (ERPM).     */
    q16_pi_init(&foc->pid_speed,
                FLOAT_TO_Q16(conf->foc_speed_kp),
                FLOAT_TO_Q16(conf->foc_speed_ki),
                FLOAT_TO_Q16(conf->l_max_current),
                FLOAT_TO_Q16(conf->l_max_current));

    /* Low-pass filters. */
    foc->lpf_id.alpha_q16 = FLOAT_TO_Q16(conf->foc_current_filter);
    foc->lpf_id.y         = 0;
    foc->lpf_iq.alpha_q16 = FLOAT_TO_Q16(conf->foc_current_filter);
    foc->lpf_iq.y         = 0;
    foc->lpf_speed.alpha_q16 = FLOAT_TO_Q16(0.10f);   /* 100 ms filter on speed */
    foc->lpf_speed.y         = 0;
    foc->lpf_temp.alpha_q16  = FLOAT_TO_Q16(0.01f);   /* very slow temp filter  */
    foc->lpf_temp.y          = 0;

    /* Temperature-compensated motor parameters (initial = nominal).      */
    foc->r_q16   = FLOAT_TO_Q16(conf->motor_r);
    foc->l_q16   = FLOAT_TO_Q16(conf->motor_l);
    foc->psi_q16 = FLOAT_TO_Q16(conf->motor_flux_linkage);

    /* Low-speed gain scheduling: start at unity gain. */
    foc->cur_kp_scale  = Q16_ONE;
    foc->cur_ki_scale  = Q16_ONE;
    foc->low_speed_active = 0;

    /* Precompute low-speed thresholds in Q16.16 (for ISR). */
    foc->ls_thr_q16   = FLOAT_TO_Q16(conf->foc_low_speed_thr);
    foc->ls_hyst_q16  = FLOAT_TO_Q16(conf->foc_low_speed_hyst);
    foc->ls_boost_q16 = FLOAT_TO_Q16(conf->foc_low_speed_boost);
    foc->ls_alpha_q16 = FLOAT_TO_Q16(0.05f);

    /* BEMF feedforward scaling (speed-dependent).
     *   at |ERPM| < thr     -> bemf_scale = boost
     *   at |ERPM| > full     -> bemf_scale = 1.0
     *   linear ramp in between.                                    */
    foc->bemf_comp_thr_q16  = FLOAT_TO_Q16(conf->foc_bemf_comp_thr);
    foc->bemf_comp_full_q16  = FLOAT_TO_Q16(conf->foc_bemf_comp_full);
    foc->bemf_comp_boost_q16 = FLOAT_TO_Q16(conf->foc_bemf_comp_boost);
    foc->bemf_scale_q16      = foc->bemf_comp_boost_q16;  /* start low */

    /* Startup current ramp precomputed constants.
     * foc_start_ramp_ms ms / 1kHz = ramp_ticks.                    */
    {
        float ramp_ms = conf->foc_start_ramp_ms;
        uint16_t ticks = (uint16_t)(ramp_ms + 0.5f);   /* ms -> 1kHz ticks */
        if (ticks < 2) ticks = 2;
        foc->start_ramp_ticks_max_q16 = INT_TO_Q16((int32_t)ticks);
    }
    foc->start_speed_thr_q16   = FLOAT_TO_Q16(conf->foc_start_speed_thr);
    foc->start_step_current_q16 = FLOAT_TO_Q16(conf->foc_start_step_current);
    foc->start_ramp_active      = 0;
    foc->start_ramp_ticks      = 0;
    foc->start_iq_current      = 0;
    foc->start_iq_target       = 0;
    foc->start_iq_step_q16     = 0;

    /* PLL effective gains start at nominal. */
    foc->pll_kp_eff = FLOAT_TO_Q16(conf->observer_pll_kp);
    foc->pll_ki_eff = FLOAT_TO_Q16(conf->observer_pll_ki);

    /* Pre-compute base Q16 gains used by the ISR scheduler to avoid
     * FLOAT_TO_Q16 calls inside the ADC ISR.                        */
    foc->cur_kp_base_q16 = FLOAT_TO_Q16(conf->foc_current_kp);
    foc->cur_ki_base_q16 = FLOAT_TO_Q16(conf->foc_current_ki);

    foc->pll_kp_normal_q16 = FLOAT_TO_Q16(conf->observer_pll_kp);
    foc->pll_ki_normal_q16 = FLOAT_TO_Q16(conf->observer_pll_ki);
    foc->pll_kp_low_q16    = FLOAT_TO_Q16(conf->observer_pll_kp_low);
    foc->pll_ki_low_q16    = FLOAT_TO_Q16(conf->observer_pll_ki_low);

    /* Precompute reciprocals for ISR usage.
     * 1/thr in Q16.16 = Q16_ONE / thr.  Done once at init (non-ISR). */
    if (foc->ls_thr_q16 > 0) {
        foc->ls_thr_inv_q16 = Q16_DIV(Q16_ONE, foc->ls_thr_q16);
    } else {
        foc->ls_thr_inv_q16 = Q16_ONE;
    }

    {
        q16_t span = foc->bemf_comp_full_q16 - foc->bemf_comp_thr_q16;
        if (span > 0) {
            foc->bemf_span_inv_q16 = Q16_DIV(Q16_ONE, span);
        } else {
            foc->bemf_span_inv_q16 = Q16_ONE;
        }
    }

    /* PLL ki per ISR step = ki (1/s) / loop_freq_hz.
     * Precomputed here so the ISR never divides by FOC_LOOP_FREQ_HZ. */
    {
        int32_t freq = (int32_t)FOC_LOOP_FREQ_HZ;
        if (freq < 1) freq = 1;
        foc->pll_ki_per_step_normal_q16 =
            (q16_t)(((int64_t)foc->pll_ki_normal_q16 / freq));
        foc->pll_ki_per_step_low_q16 =
            (q16_t)(((int64_t)foc->pll_ki_low_q16 / freq));
    }

    /* Feedforward Iq start = 0. */
    foc->iq_ff = 0;

    /* --- Position loop initialization --- */
    q16_pi_init(&foc->pid_position,
                FLOAT_TO_Q16(conf->foc_pos_kp),
                FLOAT_TO_Q16(conf->foc_pos_ki),
                FLOAT_TO_Q16(conf->l_max_speed),
                FLOAT_TO_Q16(conf->l_max_speed));
    foc->pos_set = 0;
    foc->pos_meas = 0;
    foc->pos_ff = 0;
    foc->pos_speed_max_q16 = FLOAT_TO_Q16(conf->foc_pos_max_speed);
    foc->pos_kp_base_q16 = FLOAT_TO_Q16(conf->foc_pos_kp);
    foc->pos_ki_base_q16 = FLOAT_TO_Q16(conf->foc_pos_ki);
    foc->pos_ff_base_q16 = FLOAT_TO_Q16(conf->foc_pos_ff);

    /* --- Field weakening initialization --- */
    q16_pi_init(&foc->pid_fw,
                FLOAT_TO_Q16(conf->foc_fw_kp),
                FLOAT_TO_Q16(conf->foc_fw_ki),
                FLOAT_TO_Q16(conf->l_max_current),
                FLOAT_TO_Q16(conf->l_max_duty));
    foc->fw_id_ref = 0;
    foc->fw_vd_req = 0;
    foc->fw_vd_avail = 0;
    foc->fw_duty_threshold_q16 = FLOAT_TO_Q16(conf->foc_fw_duty_start);
    foc->fw_current_max_q16 = FLOAT_TO_Q16(conf->foc_fw_current_max);
    foc->fw_active = 0;
    foc->fw_ramp_alpha_q16 = FLOAT_TO_Q16(1.0f / (conf->foc_fw_ramp_time * 100.0f));
}

/* ======================================================================== */
/* foc_reset                                                                 */
/* ======================================================================== */
void foc_reset(foc_state_t *foc)
{
    foc->id_set  = 0;
    foc->iq_set  = 0;
    foc->speed_set = 0;
    foc->id   = 0;
    foc->iq   = 0;
    foc->vd   = 0;
    foc->vq   = 0;
    foc->valpha = 0;
    foc->vbeta  = 0;
    foc->omega_l   = 0;
    foc->omega_psi = 0;
    foc->omega_l_id = 0;
    foc->omega_l_iq = 0;
    foc->iq_ff = 0;

    /* Reset PI integrators */
    foc->pid_id.integral  = 0;
    foc->pid_iq.integral  = 0;
    foc->pid_speed.integral = 0;

    /* Reset filter states */
    foc->lpf_id.y = 0;
    foc->lpf_iq.y = 0;
    foc->lpf_speed.y = 0;

    /* Reset startup ramp state */
    foc->start_ramp_active = 0;
    foc->start_ramp_ticks = 0;
    foc->start_iq_current = 0;
    foc->start_iq_target  = 0;
    foc->start_iq_step_q16 = 0;

    /* Reset BEMF scale to conservative start value */
    foc->bemf_scale_q16 = foc->bemf_comp_boost_q16;

    /* Reset position loop */
    foc->pos_set = 0;
    foc->pos_meas = 0;
    foc->pos_ff = 0;
    foc->pid_position.integral = 0;

    /* Reset field weakening */
    foc->fw_id_ref = 0;
    foc->fw_vd_req = 0;
    foc->fw_vd_avail = 0;
    foc->fw_active = 0;
    foc->pid_fw.integral = 0;
}

/* ======================================================================== */
/* foc_position_loop - Position outer loop (100 Hz)                          */
/* ======================================================================== */
/* The position loop is the outermost control loop:
 *   Position loop (100 Hz) -> Speed loop (1 kHz) -> Current loop (20 kHz)
 *
 * This function generates a speed reference for the speed loop based on:
 *   1. Position error (PI control)
 *   2. Velocity feedforward (based on position setpoint derivative)
 *   3. Speed clamping
 *
 * All arithmetic is Q16.16 fixed-point. */
void foc_position_loop(foc_state_t *foc, mc_configuration_t *conf)
{
    if (!conf->foc_pos_en) {
        /* Position loop disabled - pass through speed setpoint */
        foc->pos_meas = 0;
        foc->pos_ff = 0;
        foc->pos_set = 0;
        foc->pid_position.integral = 0;
        return;
    }

    /* Calculate position error
     * pos_meas comes from encoder or is estimated from speed integration */
    q16_t pos_err = foc->pos_set - foc->pos_meas;

    /* Apply position wrap-around if needed
     * For a 0-2pi position, the shortest path error is:
     *   if (pos_err > pi) pos_err -= 2*pi
     *   if (pos_err < -pi) pos_err += 2*pi
     * This prevents the controller from taking the long way around */
    if (pos_err > Q16_PI) {
        pos_err -= Q16_2PI;
    } else if (pos_err < -Q16_PI) {
        pos_err += Q16_2PI;
    }

    /* PI on position error -> speed demand */
    q16_t speed_ref = q16_pi_update(&foc->pid_position, pos_err);

    /* Velocity feedforward
     * The feedforward is based on the desired velocity, which can be
     * provided externally or estimated from the position setpoint.
     *
     * For a simple approach, we use the position setpoint's rate of change.
     * If an external velocity feedforward is available, it should be used instead. */
    q16_t pos_ff = 0;
    if (conf->foc_speed_ff_en) {
        /* Use the position feedforward gain
         * The feedforward signal is the desired velocity (if available)
         * or simply the position setpoint scaled by a factor.
         *
         * For precise motion control, the user should set pos_ff
         * to the desired velocity profile. Here we use a simple
         * proportional feedforward based on position error magnitude */
        pos_ff = Q16_MUL(pos_err, foc->pos_ff_base_q16);

        /* Limit feedforward to reasonable value */
        if (pos_ff > foc->pos_speed_max_q16 >> 2) {
            pos_ff = foc->pos_speed_max_q16 >> 2;
        }
        if (pos_ff < -(foc->pos_speed_max_q16 >> 2)) {
            pos_ff = -(foc->pos_speed_max_q16 >> 2);
        }
    }
    foc->pos_ff = pos_ff;

    /* Final speed reference
     * = PI output + feedforward */
    q16_t speed_cmd = speed_ref + pos_ff;

    /* Clamp to max speed from position loop
     * This prevents the position loop from requesting excessive speeds */
    if (speed_cmd > foc->pos_speed_max_q16) {
        speed_cmd = foc->pos_speed_max_q16;
        /* Anti-windup: reduce integral when saturated */
        if (foc->pid_position.integral > 0) {
            foc->pid_position.integral = (foc->pid_position.integral * 3) >> 2;
        }
    } else if (speed_cmd < -foc->pos_speed_max_q16) {
        speed_cmd = -foc->pos_speed_max_q16;
        if (foc->pid_position.integral < 0) {
            foc->pid_position.integral = (foc->pid_position.integral * 3) >> 2;
        }
    }

    /* Update speed setpoint for speed loop */
    foc->speed_set = speed_cmd;
}

/* ======================================================================== */
/* foc_field_weakening - Field weakening control (1 kHz)                    */
/* ======================================================================== */
/* When the required voltage exceeds the available bus voltage margin,
 * a negative d-axis current is injected to weaken the rotor flux
 * and extend the constant power speed range.
 *
 * This function runs at 1 kHz (same rate as speed loop).
 * All arithmetic is Q16.16 fixed-point. */
void foc_field_weakening(foc_state_t *foc, mc_configuration_t *conf)
{
    q16_t duty_used;
    q16_t duty_margin;
    q16_t fw_ref_new;
    q16_t vd_abs, vq_abs;
    q16_t v_mag;
    q16_t v_mag_sq;
    q16_t bus_voltage_q16;

    if (!conf->foc_fw_en) {
        /* FW disabled - reset */
        foc->fw_id_ref = 0;
        foc->fw_active = 0;
        foc->pid_fw.integral = 0;
        foc->id_set = 0;
        return;
    }

    /* Calculate the full voltage vector magnitude
     * v_mag = sqrt(vd^2 + vq^2)
     *
     * This is more accurate than using just |vq| because it accounts
     * for both d-axis and q-axis voltage requirements.
     * At high speeds, the d-axis voltage requirement (omega*L*iq)
     * can be significant, and ignoring it would underestimate the
     * need for field weakening. */
    vd_abs = (foc->vd >= 0) ? foc->vd : -foc->vd;
    vq_abs = (foc->vq >= 0) ? foc->vq : -foc->vq;

    /* Calculate v_mag^2 = vd^2 + vq^2 (Q16.16)
     * Q16_MUL gives result in Q16.16, so vd^2 is in Q16.16 */
    v_mag_sq = Q16_MUL(vd_abs, vd_abs) + Q16_MUL(vq_abs, vq_abs);

    /* Integer square root approximation
     * Since we don't have a hardware sqrt, use Newton's method
     * or a lookup table. For simplicity, use the following approximation:
     * v_mag ≈ max(vd, vq) + min(vd, vq) * 0.414 (approximate sqrt)
     * This is based on: sqrt(a^2+b^2) ≈ max(a,b) + min(a,b)*(sqrt(2)-1)/2
     * For a Q16.16 value, this gives approximately correct results. */
    {
        q16_t v_max, v_min;
        q16_t approx;

        if (vd_abs >= vq_abs) {
            v_max = vd_abs;
            v_min = vq_abs;
        } else {
            v_max = vq_abs;
            v_min = vd_abs;
        }

        /* v_mag ≈ v_max + v_min * 0.414  (sqrt(2)-1 ≈ 0.414)
         * In Q16.16: 0.414 ≈ 27132 */
        approx = v_max + Q16_MUL(v_min, (q16_t)27132);

        /* Refine with one Newton iteration for better accuracy
         * v_mag_new = (v_mag + v_sq / v_mag) / 2 */
        if (approx > 0) {
            q16_t correction = Q16_DIV(v_mag_sq, approx);
            v_mag = (approx + correction) >> 1;
        } else {
            v_mag = 0;
        }
    }

    /* Calculate duty cycle = v_mag / vbus
     * We need an estimate of the bus voltage.
     * For the purpose of FW, we use a fixed nominal bus voltage
     * or the actual measured vbus if available. */
    bus_voltage_q16 = INT_TO_Q16(12);  /* Assume 12V max for 24V bus */

    duty_used = Q16_DIV(v_mag, bus_voltage_q16);
    if (duty_used > Q16_ONE) duty_used = Q16_ONE;

    /* Check if FW should be active
     * If duty cycle exceeds threshold, start FW */
    {
        q16_t duty_threshold = foc->fw_duty_threshold_q16;

        if (duty_used < duty_threshold && !foc->fw_active) {
            /* Voltage margin sufficient - no FW needed */
            foc->fw_id_ref = 0;
            foc->pid_fw.integral = 0;
            foc->id_set = 0;
            return;
        }

        if (duty_used >= duty_threshold || foc->fw_active) {
            /* Calculate required FW current
             * More duty used = more FW current needed */
            foc->fw_active = 1;

            /* PI on duty error to generate FW current */
            duty_margin = duty_used - duty_threshold;
            fw_ref_new = q16_pi_update(&foc->pid_fw, duty_margin);

            /* Clamp to max FW current (negative direction) */
            if (fw_ref_new > 0) fw_ref_new = 0;
            if (fw_ref_new < -foc->fw_current_max_q16) {
                fw_ref_new = -foc->fw_current_max_q16;
            }

            /* Smooth the FW current with ramp filter
             * This prevents sudden changes that could cause current spikes */
            {
                q16_t ramp_alpha = foc->fw_ramp_alpha_q16;
                foc->fw_id_ref += (q16_t)(((int64_t)(fw_ref_new - foc->fw_id_ref) * ramp_alpha) >> 16);
            }
        }
    }

    /* Apply FW d-axis current reference (negative for flux weakening)
     * This will be used by the current loop as id_set */
    foc->id_set = foc->fw_id_ref;

    /* Store FW state variables for telemetry/debug */
    foc->fw_vd_req = vd_abs;
    foc->fw_vd_avail = bus_voltage_q16;
}

/* ======================================================================== */
/* foc_current_loop_isr - Inner current loop (PWM rate)                     */
/* ======================================================================== */
/*
 * Implements:
 *   1. Park transform: (ialpha, ibeta, angle) -> (id, iq)
 *   2. LPF on id / iq measurement
 *   3. Decoupling precomputation: omega*L*id, omega*L*iq, omega*psi
 *   4. PI controllers on d / q axis
 *   5. Sum: vd = PI(id_err)        - omega*L*iq   [d-axis]
 *           vq = PI(iq_err) + omega*L*id + omega*psi [q-axis]
 *   6. Inverse Park transform
 *
 * Arithmetic is pure Q16.16 fixed-point.
 */
void foc_current_loop_isr(foc_state_t *foc, mc_configuration_t *conf,
                          q16_t ialpha, q16_t ibeta, q16_t vbus,
                          uint16_t angle)
{
    (void)vbus;  /* vbus is consumed by SVPWM later */

    /* --- Park transform --- */
    q16_t sin_val, cos_val;
    q16_sincos(angle, &sin_val, &cos_val);

    q16_t id_raw = Q16_MUL(ialpha, cos_val) + Q16_MUL(ibeta, sin_val);
    q16_t iq_raw = Q16_MUL(-ialpha, sin_val) + Q16_MUL(ibeta, cos_val);

    /* LPF the measured currents */
    q16_t id = q16_lpf_update(&foc->lpf_id, id_raw);
    q16_t iq = q16_lpf_update(&foc->lpf_iq, iq_raw);

    foc->id = id;
    foc->iq = iq;

    /* --- Low-speed gain scheduling already blended Kp/Ki onto PI structs,
     * so we just call the standard PI update below.                          */

    /* --- Decoupling / BEMF feedforward terms ---
     * speed_rad (Q16.16 rad/s) comes from the observer, stored in
     * foc->speed_rad by the FOC module's slow-rate update.  We compute
     * omega*L and omega*psi here for ISR use.                               */
    q16_t omega = foc->speed_rad;   /* rad/s in Q16.16  (can be negative) */

    /* omega_l = omega * L   (V/A in Q16.16)  */
    q16_t omega_l   = Q16_MUL(omega, foc->l_q16);
    /* omega_psi = omega * psi  (V in Q16.16) */
    q16_t omega_psi = Q16_MUL(omega, foc->psi_q16);

    /* Coupling terms (complete voltage model: v = R*i + omega*L*i + omega*psi)
     *   d-axis:  omega*L*iq  -> cross-coupling
     *   q-axis:  omega*L*id  -> cross-coupling                                   */
    q16_t omega_l_id = Q16_MUL(omega_l, id);    /* omega*L*id (V) */
    q16_t omega_l_iq = Q16_MUL(omega_l, iq);    /* omega*L*iq (V) */
    q16_t r_id = Q16_MUL(foc->r_q16, id);       /* R*id (V) */
    q16_t r_iq = Q16_MUL(foc->r_q16, iq);       /* R*iq (V) */

    foc->omega_l    = omega_l;
    foc->omega_psi  = omega_psi;
    foc->omega_l_id = omega_l_id;
    foc->omega_l_iq = omega_l_iq;

    /* --- PI controllers ---
     * Gains have already been adjusted by the low-speed scheduler.       */
    q16_t vd_pi = q16_pi_update(&foc->pid_id, foc->id_set - id);
    q16_t vq_pi = q16_pi_update(&foc->pid_iq, foc->iq_set - iq);

    /* --- Combine PI + decoupling + BEMF feedforward ---
     * Full voltage model (d/q axis):
     *   vd = PI(id_err) + R*id      - omega*L*iq          [d-axis]
     *   vq = PI(iq_err) + R*iq      + omega*L*id + omega*psi * bemf_scale [q-axis]
     *
     * At low speeds the BEMF estimate (omega*psi) is unreliable because
     * omega is derived from a noisy back-EMF observer.  We scale the
     * BEMF feedforward by bemf_scale_q16 which rises from
     * bemf_comp_boost at standstill to 1.0 above bemf_comp_full ERPM. */
    q16_t vd = vd_pi + r_id;
    q16_t vq = vq_pi + r_iq;

    if (conf->foc_decouple_en) {
        vd -= omega_l_iq;
        vq += omega_l_id;
    }
    if (conf->foc_bemf_ff_en) {
        /* Scale BEMF FF with speed-dependent factor (already blended by
         * foc_low_speed_update).  At very low speed bemf_scale_q16 is
         * small (or zero at standstill), so unreliable omega*psi term
         * does not perturb the current loop.                             */
        vq += Q16_MUL(omega_psi, foc->bemf_scale_q16);
    }

    foc->vd = vd;
    foc->vq = vq;

    /* --- Inverse Park ---
     *   valpha = vd*cos - vq*sin
     *   vbeta  = vd*sin + vq*cos                                           */
    q16_t valpha = Q16_MUL(vd, cos_val) - Q16_MUL(vq, sin_val);
    q16_t vbeta  = Q16_MUL(vd, sin_val) + Q16_MUL(vq, cos_val);

    foc->valpha = valpha;
    foc->vbeta  = vbeta;
}

/* ======================================================================== */
/* foc_speed_loop - Outer speed loop (1 kHz)                                */
/* ======================================================================== */
void foc_speed_loop(foc_state_t *foc, mc_configuration_t *conf)
{
    /* Calculate speed error (ERPM). */
    q16_t speed_meas = foc->speed_erpm;   /* Q16.16 ERPM from ISR */
    q16_t speed_err  = foc->speed_set - speed_meas;

    /* PI on speed error -> Iq demand. */
    q16_t iq_pi = q16_pi_update(&foc->pid_speed, speed_err);

    /* Speed feedforward: iq_ff = speed_ref * speed_ff_gain.
     * Compensates for rotor inertia so the current loop tracks speed
     * without lag.
     *
     * The feedforward gain represents the ratio between required
     * current and speed for a given load. When properly tuned,
     * it reduces the PI controller's burden and improves tracking. */
    q16_t iq_ff = 0;
    if (conf->foc_speed_ff_en) {
        /* Feedforward based on speed setpoint
         * This provides a baseline current proportional to speed */
        iq_ff = Q16_MUL(foc->speed_set,
                        FLOAT_TO_Q16(conf->foc_speed_ff));

        /* Add acceleration feedforward if we have position derivative
         * For precise motion, we could use the speed error's derivative
         * Here we use a simple low-pass filtered version of the speed error */
    }
    foc->iq_ff = iq_ff;

    /* Clamp feedforward to max current. */
    q16_t iq_ff_max = FLOAT_TO_Q16(conf->l_max_current);
    if (iq_ff > iq_ff_max) iq_ff = iq_ff_max;
    if (iq_ff < -iq_ff_max) iq_ff = -iq_ff_max;

    /* Final Iq setpoint for current loop. */
    q16_t iq_ref = iq_pi + iq_ff;

    /* Limit to absolute max current */
    q16_t iq_max = FLOAT_TO_Q16(conf->l_max_current);
    q16_t iq_min = FLOAT_TO_Q16(conf->l_min_current);

    if (iq_ref > iq_max) {
        iq_ref = iq_max;
        /* Anti-windup: when output saturates, reduce integral */
        if (foc->pid_speed.integral > 0) {
            /* Gradually reduce integral to prevent windup
             * This is softer than immediate reset */
            foc->pid_speed.integral = 
                (foc->pid_speed.integral * 3) >> 2;
        }
    } else if (iq_ref < iq_min) {
        iq_ref = iq_min;
        if (foc->pid_speed.integral < 0) {
            foc->pid_speed.integral = 
                (foc->pid_speed.integral * 3) >> 2;
        }
    }

    foc->start_iq_target = iq_ref;   /* target for startup ramp */

    /* If a startup ramp is active, foc_start_ramp_step() will have already
     * set iq_set; otherwise we set it directly from the speed loop. */
    if (!foc->start_ramp_active) {
        foc->iq_set = iq_ref;
    }
}

/* ======================================================================== */
/* foc_start_ramp_step - Startup current ramp (1 kHz)                        */
/* ======================================================================== */
/* Prevents a sudden "kick" when iq_set is changed while the motor is
 * still nearly stopped (observer not yet locked).  When triggered,
 * linearly ramp iq_set from the current value to the new target over
 * conf->foc_start_ramp_ms milliseconds (1 kHz ticks).
 *
 * Trigger conditions (all must be true):
 *   1. |ERPM| < foc->start_speed_thr_q16 (motor still nearly stopped)
 *   2. |delta_iq| > foc->start_step_current_q16 (significant step)
 *
 * Once the ramp is started it runs to completion even if the motor
 * begins to spin up (the PI loop can track the slowly ramping iq).
 * However, if |ERPM| exceeds start_speed_thr during the ramp we cancel
 * and jump directly to the target - the system has spun up enough
 * that a rapid iq change is acceptable.                                  */
void foc_start_ramp_step(foc_state_t *foc, mc_configuration_t *conf)
{
    (void)conf;   /* configuration is already precomputed in foc_init */

    q16_t abs_speed = (foc->speed_erpm >= 0) ?
        foc->speed_erpm : -foc->speed_erpm;

    if (foc->start_ramp_active) {
        /* --- Ramp in progress --- */
        if (foc->start_ramp_ticks == 0 ||
            abs_speed > foc->start_speed_thr_q16) {
            /* Ramp finished OR motor spun up -> jump to target. */
            foc->start_ramp_active = 0;
            foc->start_iq_current  = foc->start_iq_target;
            foc->iq_set = foc->start_iq_target;
            return;
        }

        /* Recompute the step size from CURRENT target (not the target
         * captured at ramp start).  This lets the ramp track a moving
         * target (e.g. speed loop keeps increasing the Iq command as
         * the motor begins to spin up).  The division is done once
         * per ramp cycle (kHz rate, not ISR).                               */
        {
            uint16_t ticks_max = (uint16_t)Q16_TO_INT(
                foc->start_ramp_ticks_max_q16);
            if (ticks_max < 2) ticks_max = 2;

            q16_t remaining = foc->start_ramp_ticks;
            if (remaining == 0) remaining = 1;

            q16_t new_delta = foc->start_iq_target - foc->start_iq_current;
            /* Distribute remaining delta evenly over remaining ticks. */
            foc->start_iq_step_q16 = Q16_DIV(new_delta,
                INT_TO_Q16((int32_t)remaining));
            if (foc->start_iq_step_q16 == 0) {
                /* At least a small step so we make progress */
                foc->start_iq_step_q16 = (new_delta >= 0) ? 1 : -1;
            }
        }

        /* Advance one tick toward target */
        foc->start_iq_current += foc->start_iq_step_q16;
        foc->start_ramp_ticks--;

        /* Clamp (prevents overshooting target) */
        if ((foc->start_iq_step_q16 > 0 &&
             foc->start_iq_current > foc->start_iq_target) ||
            (foc->start_iq_step_q16 < 0 &&
             foc->start_iq_current < foc->start_iq_target)) {
            foc->start_iq_current = foc->start_iq_target;
            foc->start_ramp_active = 0;
        }

        foc->iq_set = foc->start_iq_current;
    } else {
        /* --- Check for new ramp trigger --- */
        if (abs_speed < foc->start_speed_thr_q16) {
            q16_t iq_now = foc->iq_set;
            q16_t iq_new = foc->start_iq_target;
            q16_t delta = iq_new - iq_now;
            q16_t abs_delta = (delta >= 0) ? delta : -delta;

            if (abs_delta > foc->start_step_current_q16) {
                uint16_t ticks_max = (uint16_t)Q16_TO_INT(
                    foc->start_ramp_ticks_max_q16);
                if (ticks_max < 2) ticks_max = 2;

                foc->start_iq_current  = iq_now;
                foc->start_ramp_ticks  = ticks_max;
                if (ticks_max > 0) {
                    foc->start_iq_step_q16 = Q16_DIV(delta,
                                                     INT_TO_Q16((int32_t)ticks_max));
                } else {
                    foc->start_iq_step_q16 = delta;
                }
                foc->start_ramp_active = 1;

                /* Start ramping immediately */
                foc->start_iq_current += foc->start_iq_step_q16;
                foc->start_ramp_ticks--;
                foc->iq_set = foc->start_iq_current;
            }
        }
    }
}

/* ======================================================================== */
/* foc_low_speed_update - Gain scheduling (ISR-safe)                        */
/* ======================================================================== */
/* When the measured ERPM is below the configured threshold, blend the
 * current-loop PI gains and the PLL gains toward their "low-speed"
 * values with hysteresis to avoid chattering around the threshold.
 *
 * For simplicity the scheduler modifies the running PI structs (kp, ki).
 * Integral is preserved; only the proportional/integral gains scale.
 * The "boost" multiplier is interpolated linearly in Q16.16.               */
void foc_low_speed_update(foc_state_t *foc, mc_configuration_t *conf,
                          q16_t speed_erpm)
{
    foc_low_speed_update_with_observer(foc, conf, speed_erpm, 0);
}

/* Extended version that also forwards updated PLL gains to the observer. */
void foc_low_speed_update_with_observer(foc_state_t *foc,
                                         mc_configuration_t *conf,
                                         q16_t speed_erpm,
                                         void *obs_v)
{
    (void)conf;   /* all config values are precomputed in foc_init */

    observer_state_t *obs = (observer_state_t *)obs_v;

    /* Thresholds / gains in Q16.16 (precomputed at init). */
    q16_t thr     = foc->ls_thr_q16;
    q16_t hyst    = foc->ls_hyst_q16;
    q16_t boost   = foc->ls_boost_q16;
    q16_t one     = Q16_ONE;

    /* abs(speed) */
    q16_t abs_speed = (speed_erpm >= 0) ? speed_erpm : -speed_erpm;

    /* Entry / exit with hysteresis */
    if (!foc->low_speed_active) {
        if (abs_speed < thr) {
            foc->low_speed_active = 1;
        }
    } else {
        if (abs_speed > thr + hyst) {
            foc->low_speed_active = 0;
        }
    }

    /* --- Compute target scale factor using PRECOMPUTED reciprocal
     * (replaces Q16_DIV with a Q16_MUL - single cycle on M0+).
     *   target = 1 + (boost - 1) * clamp((thr - |speed|) / thr, 0, 1) */
    q16_t target;
    if (foc->low_speed_active) {
        q16_t delta = thr - abs_speed;
        if (delta < 0) delta = 0;
        /* delta_frac = delta * inv_thr = delta / thr (Q16.16) */
        q16_t delta_frac = Q16_MUL(delta, foc->ls_thr_inv_q16);
        q16_t extra = boost - one;
        target = one + Q16_MUL(extra, delta_frac);
    } else {
        target = one;
    }

    /* Smooth the scale with a slow LPF (alpha = 0.05 per ISR).
     * This prevents gain transients from exciting the current loop.    */
    q16_t alpha = foc->ls_alpha_q16;
    q16_t new_kp_scale = foc->cur_kp_scale +
        (q16_t)(((int64_t)(target - foc->cur_kp_scale) * alpha) >> 16);
    q16_t new_ki_scale = foc->cur_ki_scale +
        (q16_t)(((int64_t)(target - foc->cur_ki_scale) * alpha) >> 16);

    foc->cur_kp_scale = new_kp_scale;
    foc->cur_ki_scale = new_ki_scale;

    /* Apply to the d / q current PI controllers using the precomputed
     * base Q16 gains (no FLOAT_TO_Q16 in the ISR).                 */
    foc->pid_id.kp = Q16_MUL(foc->cur_kp_base_q16, new_kp_scale);
    foc->pid_iq.kp = Q16_MUL(foc->cur_kp_base_q16, new_kp_scale);
    foc->pid_id.ki = Q16_MUL(foc->cur_ki_base_q16, new_ki_scale);
    foc->pid_iq.ki = Q16_MUL(foc->cur_ki_base_q16, new_ki_scale);

    /* --- PLL gain scheduling (using precomputed Q16 bases) --- */
    q16_t pll_kp_target, pll_ki_target;
    if (foc->low_speed_active) {
        q16_t delta = thr - abs_speed;
        if (delta < 0) delta = 0;
        q16_t frac = Q16_MUL(delta, foc->ls_thr_inv_q16);
        pll_kp_target = foc->pll_kp_normal_q16 +
            Q16_MUL(foc->pll_kp_low_q16 - foc->pll_kp_normal_q16, frac);
        pll_ki_target = foc->pll_ki_normal_q16 +
            Q16_MUL(foc->pll_ki_low_q16 - foc->pll_ki_normal_q16, frac);
    } else {
        pll_kp_target = foc->pll_kp_normal_q16;
        pll_ki_target = foc->pll_ki_normal_q16;
    }

    foc->pll_kp_eff += (q16_t)(((int64_t)(pll_kp_target - foc->pll_kp_eff) * alpha) >> 16);
    foc->pll_ki_eff += (q16_t)(((int64_t)(pll_ki_target - foc->pll_ki_eff) * alpha) >> 16);

    /* --- Speed-dependent BEMF feedforward scaling (no ISR division) ---
     * At |ERPM| < bemf_comp_thr the observer-estimated omega*psi is
     * corrupted by ADC noise on the small back-EMF signal, so the
     * BEMF feedforward is blended down toward bemf_comp_boost.
     * Above bemf_comp_full ERPM the BEMF is reliable and the full
     * compensation (scale = 1.0) is applied.                          */
    {
        q16_t b_thr  = foc->bemf_comp_thr_q16;
        q16_t b_full = foc->bemf_comp_full_q16;
        q16_t b_boost = foc->bemf_comp_boost_q16;
        q16_t bemf_target;

        if (b_full <= b_thr) {
            bemf_target = Q16_ONE;
        } else if (abs_speed >= b_full) {
            bemf_target = Q16_ONE;
        } else if (abs_speed <= b_thr) {
            bemf_target = b_boost;
        } else {
            q16_t delta = abs_speed - b_thr;
            q16_t frac = Q16_MUL(delta, foc->bemf_span_inv_q16);
            bemf_target = b_boost +
                Q16_MUL(Q16_ONE - b_boost, frac);
        }

        foc->bemf_scale_q16 +=
            (q16_t)(((int64_t)(bemf_target - foc->bemf_scale_q16) * alpha) >> 16);
    }

    /* Forward the blended PLL gains to the observer (ISR-safe).
     * The ki_per_step values are precomputed at init; no division. */
    if (obs != 0) {
        q16_t kp_step = foc->pll_kp_eff;
        q16_t ki_step;
        if (foc->low_speed_active) {
            q16_t delta = thr - abs_speed;
            if (delta < 0) delta = 0;
            q16_t frac = Q16_MUL(delta, foc->ls_thr_inv_q16);
            ki_step = foc->pll_ki_per_step_normal_q16 +
                Q16_MUL(foc->pll_ki_per_step_low_q16 -
                        foc->pll_ki_per_step_normal_q16, frac);
        } else {
            ki_step = foc->pll_ki_per_step_normal_q16;
        }
        observer_set_pll_gains_isr(obs, kp_step, ki_step);
    }
}

/* ======================================================================== */
/* foc_update_temp_comp - Temperature compensation (slow rate)              */
/* ======================================================================== */
void foc_update_temp_comp(foc_state_t *foc, mc_configuration_t *conf,
                          float temp_motor, float temp_mosfet)
{
    if (!conf->foc_temp_comp_en) {
        /* Restore nominal if disabled. */
        foc->r_q16   = FLOAT_TO_Q16(conf->motor_r);
        foc->l_q16   = FLOAT_TO_Q16(conf->motor_l);
        foc->psi_q16 = FLOAT_TO_Q16(conf->motor_flux_linkage);
        return;
    }

    float tref   = conf->motor_temp_ref;
    float t_corr_r = conf->motor_temp_corr_r;
    float t_corr_l = conf->motor_temp_corr_l;

    /* Use motor temperature for R/L compensation (dominant heating source).
     * Fall back to MOSFET temp if motor temp is unavailable / NaN.          */
    float t = temp_motor;
    if (t < -100.0f) t = temp_mosfet;     /* sensor missing? use MOSFET */
    if (t < -100.0f) t = tref;            /* both missing -> no compensation */

    /* R(T) = R0 * (1 + alpha * (T - Tref))  */
    float r = conf->motor_r * (1.0f + t_corr_r * (t - tref));
    /* L(T) = L0 * (1 + alpha * (T - Tref))  */
    float l = conf->motor_l * (1.0f + t_corr_l * (t - tref));

    /* Flux linkage also decreases slightly with temperature (magnet
     * demagnetization). Typical sintered NdFeB: -0.001/degC.
     *   psi(T) = psi0 * (1 - 0.001 * (T - Tref))                           */
    float psi = conf->motor_flux_linkage * (1.0f - 0.001f * (t - tref));
    if (psi < 0.0f) psi = 0.0f;

    /* Clamp to reasonable bounds to prevent runaway on bad sensor. */
    if (r < conf->motor_r * 0.5f) r = conf->motor_r * 0.5f;
    if (r > conf->motor_r * 3.0f) r = conf->motor_r * 3.0f;
    if (l < conf->motor_l * 0.5f) l = conf->motor_l * 0.5f;
    if (l > conf->motor_l * 3.0f) l = conf->motor_l * 3.0f;

    foc->r_q16   = FLOAT_TO_Q16(r);
    foc->l_q16   = FLOAT_TO_Q16(l);
    foc->psi_q16 = FLOAT_TO_Q16(psi);
}

/* ======================================================================== */
/* foc_radps_to_erpm                                                        */
/* ======================================================================== */
float foc_radps_to_erpm(q16_t speed_rad_q16, uint8_t poles)
{
    float radps = Q16_TO_FLOAT(speed_rad_q16);
    /* rad/s -> ERPM:  erpm = rad/s * 60 / (2*pi*poles) */
    return radps * 60.0f / (6.28318530718f * (float)poles);
}
