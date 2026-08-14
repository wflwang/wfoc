/*
 * bldc.c - BLDC Six-Step Commutation (VESC-style, Q16.16)
 *
 * Implementation of the API declared in bldc.h. Sensorless brushless DC motor
 * control using back-EMF zero-crossing detection (ZCD) on the floating phase.
 *
 * Features:
 *   - 6-step commutation table (HIGH / LOW / FLOATING per phase)
 *   - Open-loop startup: align rotor, then forced-commutation ramp
 *   - Closed-loop BEMF tracking with 30-deg commutation timing
 *   - Adaptive commutation timing (auto-learns the ZCD->commutation delay)
 *   - Multiple BEMF front-ends: ADC threshold, sign-change ZCD, comparator
 *   - Duty-cycle control in BLDC mode
 *   - Commutation parameter auto-learning (filtered period + adaptive offset)
 *
 * State machine: IDLE -> ALIGN -> RAMP -> CLOSED_LOOP <-> FALLBACK
 *
 * Target: ARM Cortex-M0+ (CIU32F003x5) - No FPU.
 * All ISR-time math uses Q16.16 fixed-point (no floating-point).
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under the MIT License
 */

#include "bldc.h"
#include "config/hwconf.h"

/* ======================================================================== */
/* Tuning Constants                                                          */
/* ======================================================================== */
#define BLDC_ALIGN_MS            300   /* Align phase duration (ms)          */
#define BLDC_RAMP_MS             1500   /* Max open-loop ramp duration (ms)   */
#define BLDC_RAMP_PERIOD_START    30   /* Start comm period ms (~33 Hz)       */
#define BLDC_RAMP_PERIOD_END        3   /* End comm period ms (~333 Hz)        */
#define BLDC_RAMP_DUTY_F        0.12f   /* Ramp start duty (fraction)         */
#define BLDC_RAMP_DUTY_END_F    0.18f   /* Ramp end duty (fraction)           */
#define BLDC_ZCD_TIMEOUT_FACTOR    3   /* ZCD timeout = N * comm period      */
#define BLDC_ADAPT_GAIN          1638   /* 0.025 in Q16 (adaptive timing)     */
#define BLDC_PERIOD_LPF_ALPHA    8192   /* 0.125 in Q16 (period filter)       */
#define BLDC_BEMF_HYST_Q16       3277   /* ~0.05 V hysteresis (Q16)           */
#define BLDC_COMM_TO_CLOSED_HITS    3   /* Consecutive ZCDs before closing    */

/* ======================================================================== */
/* Module State                                                              */
/* ======================================================================== */
static mc_configuration_t *s_conf       = 0;
static q16_t   s_duty                    = 0;   /* Target duty (Q16)         */
static int8_t  s_direction              = +1;  /* +1 fwd, -1 rev            */
static bldc_state_t *s_state            = 0;   /* Active state (set in ISR) */

static q16_t   s_adc_to_vbus_q16        = 0;   /* volts/count (Q16)         */
static q16_t   s_adc_to_vphase_q16      = 0;   /* volts/count (Q16)         */
static q16_t   s_dt_q16                 = 0;   /* seconds per ISR tick      */
static bldc_bemf_mode_t s_bemf_mode     = BEMF_MODE_ADC;
static q16_t   s_bemf_threshold_q16      = 0;   /* BEMF threshold (Q16 V)   */
static uint8_t s_poles                   = 1;

static uint32_t s_comm_period_filt      = 0;   /* Filtered period (ticks)  */
static int32_t  s_adaptive_offset       = 0;   /* Adaptive timing offset   */
static q16_t    s_bemf_peak             = 0;   /* Learned peak BEMF (Q16)   */
static uint8_t  s_closed_loop_hits      = 0;   /* ZCD hits while ramping    */

/* ======================================================================== */
/* 6-Step Commutation Table                                                 */
/* ======================================================================== */
/* { high-side phase, low-side phase, floating phase, bemf_rising }.
 * bemf_rising = 1 means the floating-phase BEMF crosses UP through neutral.
 * Forward 120-degree conduction sequence:
 *   0: A->B (C floats) 1: A->C (B floats) 2: B->C (A floats)
 *   3: B->A (C floats) 4: C->A (B floats) 5: C->B (A floats)
 */
static const uint8_t comm_table[6][4] = {
    { BLDC_PHASE_A, BLDC_PHASE_B, BLDC_PHASE_C, 0 },
    { BLDC_PHASE_A, BLDC_PHASE_C, BLDC_PHASE_B, 1 },
    { BLDC_PHASE_B, BLDC_PHASE_C, BLDC_PHASE_A, 0 },
    { BLDC_PHASE_B, BLDC_PHASE_A, BLDC_PHASE_C, 1 },
    { BLDC_PHASE_C, BLDC_PHASE_A, BLDC_PHASE_B, 0 },
    { BLDC_PHASE_C, BLDC_PHASE_B, BLDC_PHASE_A, 1 },
};

static void  bldc_commutate(bldc_state_t *state);
static void  bldc_apply_phases(bldc_state_t *state);
static q16_t bldc_read_float_voltage(uint16_t *adc_values, uint8_t flt);
static bool  bldc_detect_zcd(bldc_state_t *state, q16_t v_float, q16_t neutral);
static void  bldc_update_speed(bldc_state_t *state);

/* ======================================================================== */
/* Initialization                                                           */
/* ======================================================================== */

void bldc_init(mc_configuration_t *conf)
{
    s_conf      = conf;
    s_duty      = 0;
    s_direction = +1;
    s_state     = 0;

    s_comm_period_filt = 0;
    s_adaptive_offset  = 0;
    s_bemf_peak        = 0;
    s_closed_loop_hits = 0;

    /* ADC scaling: volts-per-count.
     * VBUS counts pass through the divider (ratio = R_top/R_bot + 1).
     * Phase voltages are measured directly (VREF / resolution). */
    s_adc_to_vbus_q16   = FLOAT_TO_Q16((float)ADC_VREF_V *
                                       (float)VBUS_DIV_RATIO /
                                       (float)ADC_RESOLUTION);
    s_adc_to_vphase_q16 = FLOAT_TO_Q16((float)ADC_VREF_V /
                                       (float)ADC_RESOLUTION);

    /* ISR tick period (Q16.16 seconds) */
    s_dt_q16 = FLOAT_TO_Q16(1.0f / (float)FOC_LOOP_FREQ_HZ);

    if (conf != 0) {
        s_bemf_mode          = (bldc_bemf_mode_t)conf->bldc_bemf_mode;
        s_bemf_threshold_q16 = FLOAT_TO_Q16(conf->bldc_bemf_k);
        s_poles              = conf->motor_poles;
        if (s_poles == 0) {
            s_poles = 1;
        }
    } else {
        s_bemf_mode          = BEMF_MODE_ADC;
        s_bemf_threshold_q16 = FLOAT_TO_Q16(0.5f);
        s_poles              = 1;
    }
}

void bldc_reset(void)
{
    if (s_state != 0) {
        s_state->comm_step              = 0;
        s_state->rpm                    = 0;
        s_state->comm_period            = 0;
        s_state->bemf_voltage           = 0;
        s_state->zcd_detected           = false;
        s_state->zcd_timeout            = 0;
        s_state->comp_comm_timer        = 0;
        s_state->ramp_state             = BLDC_STATE_IDLE;
        s_state->comm_tick_counter      = 0;
        s_state->last_comm_period_ticks = 0;
        s_state->zcd_to_comm_delay      = 0;
        s_state->align_timer            = 0;
        s_state->ramp_timer              = 0;
        s_state->ramp_step_period       = 0;
        s_state->ramp_duty              = 0;
        s_state->bemf_rising             = 0;
        s_state->floating_phase         = 0;
        s_state->prev_float_adc         = 0;
        s_state->is_running              = false;
    }
    s_duty              = 0;
    s_comm_period_filt  = 0;
    s_adaptive_offset   = 0;
    s_bemf_peak         = 0;
    s_closed_loop_hits  = 0;
}

/* ======================================================================== */
/* Duty / Direction / Ramp Control                                          */
/* ======================================================================== */

void bldc_set_duty(q16_t duty)
{
    if (duty < 0)        duty = 0;
    if (duty > Q16_ONE)  duty = Q16_ONE;
    s_duty = duty;
}

void bldc_set_direction(int8_t dir)
{
    s_direction = (dir >= 0) ? +1 : -1;
}

void bldc_start_ramp(void)
{
    if (s_state == 0) {
        return;
    }

    s_state->comm_step              = 0;
    s_state->comm_tick_counter      = 0;
    s_state->last_comm_period_ticks = 0;
    s_state->zcd_detected           = false;
    s_state->zcd_timeout            = 0;
    s_state->comp_comm_timer        = 0;
    s_state->prev_float_adc         = 0;
    s_state->bemf_rising             = comm_table[0][3];

    s_state->align_timer      = (uint16_t)((uint32_t)BLDC_ALIGN_MS *
                                           FOC_LOOP_FREQ_HZ / 1000U);
    s_state->ramp_timer       = (uint16_t)((uint32_t)BLDC_RAMP_MS *
                                           FOC_LOOP_FREQ_HZ / 1000U);
    s_state->ramp_step_period = (uint16_t)((uint32_t)BLDC_RAMP_PERIOD_START *
                                           FOC_LOOP_FREQ_HZ / 1000U);
    s_state->ramp_duty        = FLOAT_TO_Q16(BLDC_RAMP_DUTY_F);

    s_state->ramp_state   = BLDC_STATE_ALIGN;
    s_state->is_running    = true;

    s_comm_period_filt    = s_state->ramp_step_period;
    s_adaptive_offset     = 0;
    s_closed_loop_hits    = 0;
}

uint8_t bldc_get_comm_step(void)
{
    return (s_state != 0) ? s_state->comm_step : 0;
}

/* ======================================================================== */
/* Commutation Helpers                                                      */
/* ======================================================================== */

/* Advance the commutation step by the current direction and reset counters. */
static void bldc_commutate(bldc_state_t *state)
{
    uint8_t step;

    state->last_comm_period_ticks = state->comm_tick_counter;

    /* Auto-learning: filter the commutation period */
    if (s_comm_period_filt == 0) {
        s_comm_period_filt = state->last_comm_period_ticks;
    } else {
        q16_t filt = (q16_t)s_comm_period_filt;
        q16_t newv = (q16_t)state->last_comm_period_ticks;
        filt += (q16_t)(((int64_t)(newv - filt) * BLDC_PERIOD_LPF_ALPHA) >> 16);
        s_comm_period_filt = (uint32_t)filt;
    }

    step = (uint8_t)((state->comm_step + 6 + s_direction) % 6);
    state->comm_step          = step;
    state->comm_tick_counter  = 0;

    /* Reverse rotation flips the expected BEMF crossing polarity */
    state->bemf_rising    = comm_table[step][3] ^ (s_direction < 0 ? 1u : 0u);
    state->floating_phase = comm_table[step][2];
    state->prev_float_adc = 0;
    state->zcd_detected   = false;

    /* Nominal 30-deg ZCD->commutation delay = half period + adaptive offset */
    {
        int32_t nominal = (int32_t)(s_comm_period_filt >> 1);
        int32_t delay    = nominal + s_adaptive_offset;
        if (delay < 2) delay = 2;
        state->zcd_to_comm_delay = (uint16_t)delay;
        state->comp_comm_timer   = (uint16_t)delay;
    }

    state->zcd_timeout = (uint16_t)(s_comm_period_filt * BLDC_ZCD_TIMEOUT_FACTOR);

    bldc_update_speed(state);
}

/* Apply the active step's phase assignment to the hardware. */
static void bldc_apply_phases(bldc_state_t *state)
{
    uint8_t step = state->comm_step;
    q16_t   duty;

    if (state->ramp_state == BLDC_STATE_ALIGN ||
        state->ramp_state == BLDC_STATE_RAMP  ||
        state->ramp_state == BLDC_STATE_FALLBACK) {
        duty = state->ramp_duty;
    } else {
        duty = s_duty;
    }

    bldc_platform_set_phases(comm_table[step][0],
                             comm_table[step][1],
                             comm_table[step][2],
                             duty);
}

/* Read the floating-phase voltage (Q16.16 V) for the given phase. */
static q16_t bldc_read_float_voltage(uint16_t *adc_values, uint8_t flt)
{
    uint16_t adc;
    switch (flt) {
    case BLDC_PHASE_A: adc = adc_values[BLDC_ADC_VA]; break;
    case BLDC_PHASE_B: adc = adc_values[BLDC_ADC_VB]; break;
    case BLDC_PHASE_C: adc = adc_values[BLDC_ADC_VC]; break;
    default:           adc = 0; break;
    }
    return (q16_t)((int32_t)adc * s_adc_to_vphase_q16);
}

/* ======================================================================== */
/* BEMF Zero-Crossing Detection                                             */
/* ======================================================================== */
/* Three front-ends via s_bemf_mode:
 *   ADC         - threshold + hysteresis vs neutral (robust)
 *   ZCD         - pure sign-change crossing (sensitive)
 *   COMPARATOR  - hard-threshold trip (emulates external comparator)
 */
static bool bldc_detect_zcd(bldc_state_t *state, q16_t v_float, q16_t neutral)
{
    int32_t diff = v_float - neutral;
    int32_t prev = state->prev_float_adc;
    bool    zcd  = false;

    state->prev_float_adc = diff;

    /* Auto-learning: track peak |BEMF| for adaptive thresholding */
    {
        q16_t absdiff = (diff >= 0) ? diff : -diff;
        if (absdiff > s_bemf_peak) {
            s_bemf_peak = absdiff;
        }
    }

    if (state->bemf_rising) {
        switch (s_bemf_mode) {
        case BEMF_MODE_ADC:
            if (prev <= -BLDC_BEMF_HYST_Q16 && diff >= BLDC_BEMF_HYST_Q16) zcd = true;
            break;
        case BEMF_MODE_COMPARATOR:
            if (prev < 0 && diff >= s_bemf_threshold_q16) zcd = true;
            break;
        case BEMF_MODE_ZCD:
        default:
            if (prev < 0 && diff >= 0) zcd = true;
            break;
        }
    } else {
        switch (s_bemf_mode) {
        case BEMF_MODE_ADC:
            if (prev >= BLDC_BEMF_HYST_Q16 && diff <= -BLDC_BEMF_HYST_Q16) zcd = true;
            break;
        case BEMF_MODE_COMPARATOR:
            if (prev > 0 && diff <= -s_bemf_threshold_q16) zcd = true;
            break;
        case BEMF_MODE_ZCD:
        default:
            if (prev > 0 && diff <= 0) zcd = true;
            break;
        }
    }

    return zcd;
}

/* ======================================================================== */
/* Speed Estimation                                                         */
/* ======================================================================== */
/* mech_rpm = 10 / (comm_period_seconds * poles)  (Q16.16)
 * 6 commutations per electrical revolution. */
static void bldc_update_speed(bldc_state_t *state)
{
    uint32_t ticks = state->last_comm_period_ticks;

    if (ticks == 0) {
        state->rpm         = 0;
        state->comm_period = 0;
        return;
    }

    /* comm_period (Q16.16 s) = ticks * dt_q16 */
    state->comm_period = (q16_t)((int64_t)ticks * (int64_t)s_dt_q16);

    if (state->comm_period > 0) {
        state->rpm = (q16_t)(((int64_t)10 << 32) /
                             ((int64_t)state->comm_period * (int64_t)s_poles));
    } else {
        state->rpm = 0;
    }
}

/* ======================================================================== */
/* State Machine Handlers                                                   */
/* ======================================================================== */

/* ALIGN: hold step 0 with ramp_duty until align_timer expires. */
static void bldc_state_align(bldc_state_t *state)
{
    if (state->align_timer > 0) state->align_timer--;
    if (state->align_timer == 0) {
        state->ramp_state        = BLDC_STATE_RAMP;
        state->comm_step         = 0;
        state->comm_tick_counter = 0;
        s_comm_period_filt       = state->ramp_step_period;
    }
}

/* RAMP: forced commutation with decreasing period (accelerating). */
static void bldc_state_ramp(bldc_state_t *state, q16_t v_float, q16_t neutral)
{
    uint16_t ramp_end_ticks;

    state->comm_tick_counter++;

    /* Try BEMF detection during ramp to switch to closed loop early */
    if (bldc_detect_zcd(state, v_float, neutral)) {
        s_closed_loop_hits++;
        if (s_closed_loop_hits >= BLDC_COMM_TO_CLOSED_HITS &&
            state->ramp_step_period <=
                (uint16_t)((uint32_t)BLDC_RAMP_PERIOD_END *
                           FOC_LOOP_FREQ_HZ / 1000U)) {
            state->ramp_state        = BLDC_STATE_CLOSED_LOOP;
            state->comm_tick_counter = 0;
            s_adaptive_offset        = 0;
            return;
        }
    }

    if (state->comm_tick_counter >= state->ramp_step_period) {
        bldc_commutate(state);
        ramp_end_ticks = (uint16_t)((uint32_t)BLDC_RAMP_PERIOD_END *
                                    FOC_LOOP_FREQ_HZ / 1000U);
        if (state->ramp_step_period > ramp_end_ticks) {
            state->ramp_step_period--;
        }
        {
            q16_t target = FLOAT_TO_Q16(BLDC_RAMP_DUTY_END_F);
            if (state->ramp_duty < target) {
                state->ramp_duty += (target - state->ramp_duty) >> 6;
            }
        }
    }

    if (state->ramp_timer > 0) state->ramp_timer--;
    if (state->ramp_timer == 0) {
        state->ramp_state        = BLDC_STATE_CLOSED_LOOP;
        state->comm_tick_counter = 0;
        s_adaptive_offset        = 0;
        s_closed_loop_hits       = 0;
    }
}

/* CLOSED_LOOP: BEMF ZCD tracking with 30-degree commutation timing. */
static void bldc_state_closed(bldc_state_t *state, q16_t v_float, q16_t neutral)
{
    state->comm_tick_counter++;
    if (state->zcd_timeout > 0) state->zcd_timeout--;

    if (!state->zcd_detected) {
        if (bldc_detect_zcd(state, v_float, neutral)) {
            state->zcd_detected = true;

            /* Adaptive commutation timing (auto-learning):
             * comm_to_zcd should be ~half the commutation period.
             * ZCD early (rotor leading) -> shorten delay; late -> lengthen. */
            {
                int32_t comm_to_zcd = (int32_t)state->comm_tick_counter;
                int32_t ideal       = (int32_t)(s_comm_period_filt >> 1);
                int32_t err         = comm_to_zcd - ideal;
                s_adaptive_offset  += (int32_t)(((int64_t)err *
                                                 BLDC_ADAPT_GAIN) >> 16);
                {
                    int32_t lim = (int32_t)(s_comm_period_filt >> 2);
                    if (s_adaptive_offset >  lim) s_adaptive_offset =  lim;
                    if (s_adaptive_offset < -lim) s_adaptive_offset = -lim;
                }
            }
            state->comp_comm_timer = state->zcd_to_comm_delay;
        }
    }

    if (state->zcd_detected) {
        if (state->comp_comm_timer > 0) state->comp_comm_timer--;
        if (state->comp_comm_timer == 0) {
            bldc_commutate(state);
            return;
        }
    }

    /* Loss of BEMF: timeout without a ZCD -> fall back to ramping */
    if (state->zcd_timeout == 0) {
        state->ramp_state        = BLDC_STATE_FALLBACK;
        state->ramp_timer        = (uint16_t)((uint32_t)BLDC_RAMP_MS *
                                              FOC_LOOP_FREQ_HZ / 1000U);
        state->ramp_step_period  = (uint16_t)((uint32_t)BLDC_RAMP_PERIOD_START *
                                              FOC_LOOP_FREQ_HZ / 1000U);
        state->comm_tick_counter = 0;
        s_closed_loop_hits       = 0;
    }
}

/* FALLBACK: re-acquire sync via forced commutation, then retry closed loop. */
static void bldc_state_fallback(bldc_state_t *state, q16_t v_float, q16_t neutral)
{
    state->comm_tick_counter++;
    if (state->ramp_timer > 0) state->ramp_timer--;

    if (state->comm_tick_counter >= state->ramp_step_period) {
        bldc_commutate(state);
        if (state->ramp_step_period >
            (uint16_t)((uint32_t)BLDC_RAMP_PERIOD_END *
                       FOC_LOOP_FREQ_HZ / 1000U)) {
            state->ramp_step_period--;
        }
    }

    if (bldc_detect_zcd(state, v_float, neutral)) {
        s_closed_loop_hits++;
        if (s_closed_loop_hits >= BLDC_COMM_TO_CLOSED_HITS) {
            state->ramp_state        = BLDC_STATE_CLOSED_LOOP;
            state->comm_tick_counter = 0;
            s_adaptive_offset        = 0;
            s_closed_loop_hits       = 0;
        }
    }

    if (state->ramp_timer == 0) {
        /* Could not recover: stop */
        state->ramp_state  = BLDC_STATE_IDLE;
        state->is_running  = false;
    }
}

/* ======================================================================== */
/* Main ISR Entry                                                           */
/* ======================================================================== */

void bldc_run_isr(bldc_state_t *state, uint16_t *adc_values)
{
    q16_t   vbus_q16, neutral_q16, v_float_q16;
    uint8_t flt;

    if (state == 0 || adc_values == 0) {
        return;
    }

    s_state = state;

    /* Convert bus voltage and compute the neutral point (~Vbus/2) */
    vbus_q16    = (q16_t)((int32_t)adc_values[BLDC_ADC_VBUS] * s_adc_to_vbus_q16);
    neutral_q16  = vbus_q16 >> 1;

    /* Floating-phase voltage for the current step */
    flt          = comm_table[state->comm_step][2];
    v_float_q16  = bldc_read_float_voltage(adc_values, flt);
    state->bemf_voltage   = v_float_q16 - neutral_q16;
    state->floating_phase = flt;

    switch (state->ramp_state) {
    case BLDC_STATE_ALIGN:
        bldc_state_align(state);
        break;

    case BLDC_STATE_RAMP:
        bldc_state_ramp(state, v_float_q16, neutral_q16);
        break;

    case BLDC_STATE_CLOSED_LOOP:
        bldc_state_closed(state, v_float_q16, neutral_q16);
        break;

    case BLDC_STATE_FALLBACK:
        bldc_state_fallback(state, v_float_q16, neutral_q16);
        break;

    case BLDC_STATE_IDLE:
    default:
        state->is_running = false;
        bldc_platform_set_phases(0, 0, 0, 0);
        return;
    }

    /* Drive the active step onto the hardware */
    bldc_apply_phases(state);
}

/* ======================================================================== */
/* Platform Hook (weak default - override in HAL/platform driver)           */
/* ======================================================================== */
#if defined(__GNUC__) || defined(__clang__)
__attribute__((weak))
#endif
void bldc_platform_set_phases(uint8_t phase_hi, uint8_t phase_lo,
                              uint8_t phase_flt, q16_t duty_q16)
{
    (void)phase_hi;
    (void)phase_lo;
    (void)phase_flt;
    (void)duty_q16;
    /* Default: no-op. Provide a strong override in the platform driver to
     * drive the TIM1 PWM channels (high-side / low-side / float). */
}






