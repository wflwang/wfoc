/*
 * observer.c - Rotor Position/Speed Observers (Q16.16)
 *
 * Implementation of the SMO, MXLEMMA, HFI observers and the shared PLL
 * declared in observer.h. The active observer is selected by config->type.
 *
 * Math summary (stationary alpha-beta frame):
 *
 *   PMSM model:
 *     v_alpha = R*i_alpha + L*di_alpha/dt + e_alpha
 *     v_beta  = R*i_beta  + L*di_beta/dt  + e_beta
 *   with back-EMF e_alpha = -psi*omega*sin(theta),
 *                 e_beta  =  psi*omega*cos(theta),
 *   so theta = atan2(-e_alpha, e_beta).
 *
 *   SMO      : i_est += (dt/L)*(v - R*i_est - K*sign(i - i_est));
 *              back-EMF = LPF(K*sign(...)); theta from atan2.
 *   MXLEMMA  : full-order observer integrating stator flux lambda and
 *              correcting the estimated current with a Luenberger gain.
 *              theta from the PM flux vector (lambda - L*i).
 *   HFI      : inject a HF voltage on the d-axis and demodulate the
 *              q-axis current response to track saliency at low speed.
 *   PLL      : type-2 loop locking the smooth angle/speed onto the raw
 *              observer angle.
 *
 * Target: ARM Cortex-M0+ (CIU32F003x5) - No FPU.
 * All ISR-time math uses Q16.16 fixed-point (no floating-point).
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under the MIT License
 */

#include "observer.h"
#include "config/hwconf.h"

/* ======================================================================== */
/* Convergence: ISR ticks before declaring the angle valid                   */
/* ======================================================================== */
#define OBSERVER_VALID_TICKS    200U   /* ~10 ms at 20 kHz                   */

/* ======================================================================== */
/* Internal Helpers                                                          */
/* ======================================================================== */

/* Wrap a Q16.16 radian value into [0, 2*pi). The value is assumed to be
 * within one 2*pi band of the range (true because we wrap every ISR step and
 * clamp the per-step increment), so a single conditional suffices. */
static inline q16_t wrap_rad_pos(q16_t rad)
{
    if (rad >= Q16_2PI) rad -= Q16_2PI;
    if (rad < 0)        rad += Q16_2PI;
    return rad;
}

/* Convert angle_16 (0..65535 = 0..360 deg) to Q16.16 radians. */
static inline q16_t angle16_to_rad(uint16_t angle)
{
    return (q16_t)(((int64_t)angle * (int64_t)Q16_2PI) >> 16);
}

/* Convert Q16.16 radians to angle_16 (0..65535). */
static inline uint16_t rad_to_angle16(q16_t rad)
{
    return (uint16_t)(((uint64_t)rad << 16) / (uint64_t)Q16_2PI);
}

/* ======================================================================== */
/* Initialization                                                            */
/* ======================================================================== */

void observer_init(observer_state_t *obs, mc_configuration_t *config)
{
    q16_t dt;
    float r, l, kp, ki;

    if (obs == 0) {
        return;
    }

    /* Zero everything first */
    obs->smo_alpha       = 0;
    obs->smo_ialpha_est  = 0;
    obs->smo_ibeta_est   = 0;
    obs->smo_valpha_est  = 0;
    obs->smo_vbeta_est   = 0;
    obs->smo_zalpha      = 0;
    obs->smo_zbeta       = 0;
    obs->smo_ealpha      = 0;
    obs->smo_ebeta       = 0;
    obs->smo_lpf_alpha.y = 0;
    obs->smo_lpf_beta.y  = 0;

    obs->mxl_ialpha       = 0;
    obs->mxl_ibeta        = 0;
    obs->mxl_lambda_alpha = 0;
    obs->mxl_lambda_beta  = 0;
    obs->mxl_gamma        = 0;
    obs->mxl_ealpha       = 0;
    obs->mxl_ebeta        = 0;

    obs->pll_kp          = 0;
    obs->pll_ki          = 0;
    obs->pll_integral    = 0;
    obs->angle           = 0;
    obs->speed           = 0;
    obs->pll_angle_rad   = 0;
    obs->pll_kp_gain_eff = 0;
    obs->pll_ki_gain_eff = 0;

    obs->hfi_angle       = 0;
    obs->hfi_amplitude   = 0;
    obs->hfi_freq        = 0;
    obs->hfi_phase       = 0;
    obs->hfi_phase_inc   = 0;
    obs->hfi_demod_d     = 0;
    obs->hfi_demod_q     = 0;
    obs->hfi_prev_id     = 0;
    obs->hfi_prev_iq     = 0;
    obs->hfi_track_gain  = 0;
    obs->hfi_lpf_d.y     = 0;
    obs->hfi_lpf_q.y     = 0;

    obs->valid           = false;
    obs->valid_counter   = 0;
    obs->config          = config;

    /* Precompute sample period (Q16.16 seconds). FOC loop = PWM frequency. */
    dt = FLOAT_TO_Q16(1.0f / (float)FOC_LOOP_FREQ_HZ);
    obs->dt_q16 = dt;

    if (config != 0) {
        r  = config->motor_r;
        l  = config->motor_l;
        kp = config->observer_pll_kp;
        ki = config->observer_pll_ki;

        obs->type = (observer_type_t)config->observer_type;

        /* Motor parameters in Q16.16 */
        obs->r_q16            = FLOAT_TO_Q16(r);
        obs->l_q16            = FLOAT_TO_Q16(l);
        obs->inv_l_q16        = (l > 1e-9f) ? FLOAT_TO_Q16(1.0f / l) : 0;
        obs->flux_linkage_q16 = FLOAT_TO_Q16(config->motor_flux_linkage);

        /* dt/L and R*dt/L (Q16.16). dt/L = dt * (1/L). */
        obs->k_dt_l     = Q16_MUL(dt, obs->inv_l_q16);
        obs->k_ohm_dt_l = Q16_MUL(obs->r_q16, obs->k_dt_l);

        /* SMO sliding gain (K). Typical ~0.5..1.0 * (L/dt) bounded; here we
         * use the configured observer gain directly. */
        obs->smo_alpha = FLOAT_TO_Q16(config->observer_gain);

        /* MXLEMMA Luenberger correction gain (gamma). Small, e.g. 0.1..0.3. */
        obs->mxl_gamma = FLOAT_TO_Q16(config->observer_gain * 0.25f);
        if (obs->mxl_gamma > Q16_HALF) obs->mxl_gamma = Q16_HALF;

        /* PLL gains (continuous-time kp in 1/s, ki in 1/s^2).
         * The discrete integral update accumulates ki*dt*err each step, so
         * the per-step gain is ki*dt = ki / loop_freq. */
        obs->pll_kp_gain = FLOAT_TO_Q16(kp);
        obs->pll_ki_gain = FLOAT_TO_Q16(ki / (float)FOC_LOOP_FREQ_HZ);
        obs->pll_kp      = obs->pll_kp_gain;
        obs->pll_ki      = obs->pll_ki_gain;
        obs->pll_kp_gain_eff = obs->pll_kp_gain;
        obs->pll_ki_gain_eff = obs->pll_ki_gain;

        /* Speed clamp for anti-windup (electrical). ~2*pi*500 rad/s. */
        obs->pll_speed_limit = FLOAT_TO_Q16(2.0f * 3.14159265f * 500.0f);

        /* HFI parameters */
        obs->hfi_amplitude  = FLOAT_TO_Q16(config->hfi_amplitude);
        obs->hfi_freq       = (uint16_t)config->hfi_freq;
        /* Carrier phase step per ISR (angle_16): freq * 65536 / loop_freq */
        if (config->hfi_freq > 0 && FOC_LOOP_FREQ_HZ > 0) {
            obs->hfi_phase_inc = (uint16_t)((uint32_t)config->hfi_freq *
                                            65536U / (uint32_t)FOC_LOOP_FREQ_HZ);
        }
        obs->hfi_track_gain = FLOAT_TO_Q16(2.0f);  /* angle_16 per unit error */

        /* LPF coefficients */
        obs->smo_lpf_alpha.alpha_q16 = FLOAT_TO_Q16(0.05f);  /* ~tau 1 ms   */
        obs->smo_lpf_beta.alpha_q16  = FLOAT_TO_Q16(0.05f);
        obs->hfi_lpf_d.alpha_q16     = FLOAT_TO_Q16(0.1f);
        obs->hfi_lpf_q.alpha_q16     = FLOAT_TO_Q16(0.1f);
    } else {
        obs->type           = OBSERVER_NONE;
        obs->r_q16          = 0;
        obs->l_q16          = Q16_ONE;
        obs->inv_l_q16      = Q16_ONE;
        obs->flux_linkage_q16 = 0;
        obs->k_dt_l         = 0;
        obs->k_ohm_dt_l     = 0;
        obs->smo_alpha      = 0;
        obs->mxl_gamma      = 0;
        obs->pll_kp_gain    = 0;
        obs->pll_ki_gain    = 0;
        obs->pll_speed_limit = 0;
        obs->hfi_amplitude  = 0;
        obs->hfi_freq       = 0;
        obs->hfi_phase_inc  = 0;
        obs->hfi_track_gain = 0;
    }
}

/* ======================================================================== */
/* Reset                                                                     */
/* ======================================================================== */

void observer_reset(observer_state_t *obs)
{
    if (obs == 0) {
        return;
    }

    obs->smo_ialpha_est  = 0;
    obs->smo_ibeta_est   = 0;
    obs->smo_valpha_est  = 0;
    obs->smo_vbeta_est   = 0;
    obs->smo_zalpha      = 0;
    obs->smo_zbeta       = 0;
    obs->smo_ealpha      = 0;
    obs->smo_ebeta       = 0;
    obs->smo_lpf_alpha.y = 0;
    obs->smo_lpf_beta.y  = 0;

    obs->mxl_ialpha       = 0;
    obs->mxl_ibeta        = 0;
    obs->mxl_lambda_alpha = 0;
    obs->mxl_lambda_beta  = 0;
    obs->mxl_ealpha       = 0;
    obs->mxl_ebeta        = 0;

    obs->pll_integral    = 0;
    obs->pll_angle_rad   = 0;
    obs->angle           = 0;
    obs->speed           = 0;

    obs->hfi_angle       = 0;
    obs->hfi_phase       = 0;
    obs->hfi_demod_d     = 0;
    obs->hfi_demod_q     = 0;
    obs->hfi_prev_id     = 0;
    obs->hfi_prev_iq     = 0;
    obs->hfi_lpf_d.y     = 0;
    obs->hfi_lpf_q.y     = 0;

    obs->valid           = false;
    obs->valid_counter   = 0;
}

/* ======================================================================== */
/* PLL - Phase-Locked Loop                                                   */
/* ======================================================================== */
/* Tracks the raw observer angle (angle_meas, angle_16) and produces a smooth
 * angle (obs->angle) and speed (obs->speed, Q16.16 rad/s). The loop is a
 * type-2 PLL:
 *   error   = wrap_pi(theta_meas - theta_pll)            [rad, Q16]
 *   omega   = kp * error + integral                       [rad/s, Q16]
 *   integ  += (ki*dt) * error
 *   theta  += omega * dt
 */
static void pll_update(observer_state_t *obs, uint16_t angle_meas)
{
    q16_t theta_meas;
    q16_t err;
    q16_t omega;

    theta_meas = angle16_to_rad(angle_meas);

    /* Phase error (wrap to [-pi, pi]) */
    err = theta_meas - obs->pll_angle_rad;
    if (err >  Q16_PI) err -= Q16_2PI;
    if (err < -Q16_PI) err += Q16_2PI;

    /* Integral with anti-windup (uses the dynamically-adjusted gain) */
    obs->pll_integral += Q16_MUL(obs->pll_ki_gain_eff, err);
    obs->pll_integral  = q16_sat(obs->pll_integral,
                                 -obs->pll_speed_limit, obs->pll_speed_limit);

    /* Speed = kp_eff*error + integral */
    omega = Q16_MUL(obs->pll_kp_gain_eff, err) + obs->pll_integral;
    omega = q16_sat(omega, -obs->pll_speed_limit, obs->pll_speed_limit);
    obs->speed = omega;

    /* Integrate angle and wrap to [0, 2pi) */
    obs->pll_angle_rad = wrap_rad_pos(obs->pll_angle_rad +
                                      Q16_MUL(omega, obs->dt_q16));

    /* Output angle_16 */
    obs->angle = rad_to_angle16(obs->pll_angle_rad);
}

/* ======================================================================== */
/* SMO - Sliding Mode Observer                                               */
/* ======================================================================== */
/*   i_alpha_est += (dt/L) * (v_alpha - R*i_alpha_est - z_alpha)
 *   z_alpha      = K * sign(i_alpha - i_alpha_est)
 *   back-EMF     = LPF(z)          -> e_alpha, e_beta
 *   theta        = atan2(-e_alpha, e_beta)
 */
static void smo_run(observer_state_t *obs,
                    q16_t valpha, q16_t vbeta,
                    q16_t ialpha, q16_t ibeta)
{
    q16_t err_a, err_b;
    q16_t drv_a, drv_b;

    /* Current error */
    err_a = ialpha - obs->smo_ialpha_est;
    err_b = ibeta  - obs->smo_ibeta_est;

    /* Switching (sliding) signal */
    obs->smo_zalpha = (err_a >= 0) ? obs->smo_alpha : -obs->smo_alpha;
    obs->smo_zbeta  = (err_b >= 0) ? obs->smo_alpha : -obs->smo_alpha;

    /* State copy for the voltage estimate (passed through) */
    obs->smo_valpha_est = valpha;
    obs->smo_vbeta_est  = vbeta;

    /* Current observer update:
     *   i_est += (dt/L) * (v - R*i_est - z) */
    drv_a = valpha - Q16_MUL(obs->r_q16, obs->smo_ialpha_est) - obs->smo_zalpha;
    drv_b = vbeta  - Q16_MUL(obs->r_q16, obs->smo_ibeta_est)  - obs->smo_zbeta;
    obs->smo_ialpha_est += Q16_MUL(obs->k_dt_l, drv_a);
    obs->smo_ibeta_est  += Q16_MUL(obs->k_dt_l, drv_b);

    /* Back-EMF = LPF(switching signal) */
    obs->smo_ealpha = q16_lpf_update(&obs->smo_lpf_alpha, obs->smo_zalpha);
    obs->smo_ebeta  = q16_lpf_update(&obs->smo_lpf_beta,  obs->smo_zbeta);

    /* Rotor angle: theta = atan2(-e_alpha, e_beta) */
    {
        uint16_t theta = q16_atan2(-obs->smo_ealpha, obs->smo_ebeta);
        pll_update(obs, theta);
    }
}

/* ======================================================================== */
/* MXLEMMA - Full-Order Flux Observer                                        */
/* ======================================================================== */
/* Predicts stator current and integrates stator flux linkage. The PM flux
 * vector is recovered as (lambda - L*i) and its angle gives the rotor
 * position. A Luenberger gain (gamma) pulls the estimated current toward the
 * measured current, bounding the flux integrator drift.
 *
 *   e_alpha = -omega * pm_beta      (cross product of speed and PM flux)
 *   e_beta  =  omega * pm_alpha
 *   i_est  += (dt/L)*(v - R*i_est - e) + gamma*(i_meas - i_est)
 *   lambda += dt * (v - R*i_est)
 *   pm     = lambda - L*i_est
 *   theta  = atan2(pm_beta, pm_alpha)
 */
static void mxlemma_run(observer_state_t *obs,
                        q16_t valpha, q16_t vbeta,
                        q16_t ialpha, q16_t ibeta)
{
    q16_t omega;
    q16_t pm_alpha, pm_beta;
    q16_t drv_a, drv_b;

    omega = obs->speed;   /* from previous PLL step (Q16.16 rad/s) */

    /* PM flux from previous step (for back-EMF cross product) */
    pm_alpha = obs->mxl_lambda_alpha - Q16_MUL(obs->l_q16, obs->mxl_ialpha);
    pm_beta  = obs->mxl_lambda_beta  - Q16_MUL(obs->l_q16, obs->mxl_ibeta);

    /* Back-EMF from speed x PM flux */
    obs->mxl_ealpha = -Q16_MUL(omega, pm_beta);
    obs->mxl_ebeta  =  Q16_MUL(omega, pm_alpha);

    /* Predict currents */
    drv_a = valpha - Q16_MUL(obs->r_q16, obs->mxl_ialpha) - obs->mxl_ealpha;
    drv_b = vbeta  - Q16_MUL(obs->r_q16, obs->mxl_ibeta)  - obs->mxl_ebeta;
    obs->mxl_ialpha += Q16_MUL(obs->k_dt_l, drv_a);
    obs->mxl_ibeta  += Q16_MUL(obs->k_dt_l, drv_b);

    /* Luenberger correction toward measured current */
    obs->mxl_ialpha += Q16_MUL(obs->mxl_gamma, ialpha - obs->mxl_ialpha);
    obs->mxl_ibeta  += Q16_MUL(obs->mxl_gamma, ibeta  - obs->mxl_ibeta);

    /* Integrate stator flux using the corrected current estimate */
    obs->mxl_lambda_alpha += Q16_MUL(obs->dt_q16,
                                     valpha - Q16_MUL(obs->r_q16, obs->mxl_ialpha));
    obs->mxl_lambda_beta  += Q16_MUL(obs->dt_q16,
                                     vbeta  - Q16_MUL(obs->r_q16, obs->mxl_ibeta));

    /* Recover PM flux vector and rotor angle */
    pm_alpha = obs->mxl_lambda_alpha - Q16_MUL(obs->l_q16, obs->mxl_ialpha);
    pm_beta  = obs->mxl_lambda_beta  - Q16_MUL(obs->l_q16, obs->mxl_ibeta);
    {
        uint16_t theta = q16_atan2(pm_beta, pm_alpha);
        pll_update(obs, theta);
    }
}

/* ======================================================================== */
/* HFI - High Frequency Injection                                            */
/* ======================================================================== */
/* Injects a HF voltage on the d-axis at the estimated angle and demodulates
 * the q-axis current response, which is proportional to the saliency angle
 * error. A tracking loop drives the estimated angle to null the demodulated
 * error signal. The injection voltage itself (V*sin(phase)) is applied by the
 * FOC voltage stage using hfi_phase / hfi_amplitude from this state.
 *
 *   carrier   = sin(hfi_phase)
 *   hf_iq     = (iq - iq_prev)           (high-pass to isolate carrier response)
 *   demod_q   = LPF(hf_iq * carrier)
 *   hfi_angle += track_gain * demod_q
 */
static void hfi_run(observer_state_t *obs,
                    q16_t ialpha, q16_t ibeta)
{
    q16_t sin_a, cos_a;
    q16_t id, iq;
    q16_t hf_id, hf_iq;
    q16_t carrier;

    /* Advance carrier phase */
    obs->hfi_phase = (uint16_t)(obs->hfi_phase + obs->hfi_phase_inc);

    /* Park transform measured current using the tracked HFI angle */
    q16_sincos(obs->hfi_angle, &sin_a, &cos_a);
    id = Q16_MUL(ialpha, cos_a) + Q16_MUL(ibeta, sin_a);
    iq = -Q16_MUL(ialpha, sin_a) + Q16_MUL(ibeta, cos_a);

    /* High-pass: isolate the carrier-frequency current response */
    hf_id = id - obs->hfi_prev_id;
    hf_iq = iq - obs->hfi_prev_iq;
    obs->hfi_prev_id = id;
    obs->hfi_prev_iq = iq;

    /* Demodulate with the injected carrier and low-pass filter */
    carrier = q16_sin(obs->hfi_phase);
    obs->hfi_demod_d = q16_lpf_update(&obs->hfi_lpf_d, Q16_MUL(hf_id, carrier));
    obs->hfi_demod_q = q16_lpf_update(&obs->hfi_lpf_q, Q16_MUL(hf_iq, carrier));

    /* Tracking loop: drive the demodulated q signal to zero. The signal is
     * proportional to sin(2*(theta_actual - theta_est)), so for small errors
     * it is linear in the angle error. Integrate via the angle directly. */
    {
        int32_t delta = (int32_t)Q16_MUL(obs->hfi_track_gain, obs->hfi_demod_q);
        int32_t new_angle = (int32_t)obs->hfi_angle + delta;

        /* Clamp step to avoid large jumps */
        if (new_angle < 0)         new_angle += 65536;
        if (new_angle >= 65536)    new_angle -= 65536;
        obs->hfi_angle = (uint16_t)new_angle;
    }

    /* Report HFI angle and a (coarse) speed from the demodulation magnitude */
    obs->angle = obs->hfi_angle;
    obs->speed = 0;
}

/* ======================================================================== */
/* Main ISR Entry                                                            */
/* ======================================================================== */

void observer_run_isr(observer_state_t *obs, q16_t valpha, q16_t vbeta,
                      q16_t ialpha, q16_t ibeta, q16_t vbus)
{
    (void)vbus;   /* vbus is used by the caller for voltage scaling; the
                   * observers operate on already-scaled valpha/vbeta. */

    if (obs == 0) {
        return;
    }

    switch (obs->type) {
    case OBSERVER_SMO:
        smo_run(obs, valpha, vbeta, ialpha, ibeta);
        break;

    case OBSERVER_MXLEMMA:
    case OBSERVER_MXV_LAMBDA:
        mxlemma_run(obs, valpha, vbeta, ialpha, ibeta);
        break;

    case OBSERVER_HFI:
        hfi_run(obs, ialpha, ibeta);
        break;

    case OBSERVER_NONE:
    default:
        /* No observer: leave angle/speed unchanged */
        break;
    }

    /* Convergence counter: declare angle valid after the loop settles */
    if (obs->valid_counter < OBSERVER_VALID_TICKS) {
        obs->valid_counter++;
        if (obs->valid_counter == OBSERVER_VALID_TICKS) {
            obs->valid = true;
        }
    }
}

/* ======================================================================== */
/* Accessors                                                                 */
/* ======================================================================== */

uint16_t observer_get_angle(observer_state_t *obs)
{
    if (obs == 0) {
        return 0;
    }
    return obs->angle;
}

q16_t observer_get_speed(observer_state_t *obs)
{
    if (obs == 0) {
        return 0;
    }
    return obs->speed;
}

bool observer_is_valid(observer_state_t *obs)
{
    if (obs == 0) {
        return false;
    }
    return obs->valid;
}

/* ======================================================================== */
/* Dynamic PLL gain update (ISR-safe)                                        */
/* ======================================================================== */
/* Called by foc_low_speed_update() to blend the PLL gains toward their
 * low-speed values as the motor decelerates.  Only the gain values are
 * touched; integrator state is preserved to avoid transients.              */
void observer_set_pll_gains_isr(observer_state_t *obs,
                                q16_t kp_eff, q16_t ki_eff)
{
    if (obs == 0) {
        return;
    }
    obs->pll_kp_gain_eff = kp_eff;
    obs->pll_ki_gain_eff = ki_eff;
}
