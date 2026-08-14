/*
 * mcpwm.c - Motor PWM Backend Bridge
 *
 * Provides the strong overrides for the weak mcpwm_* stubs in app.c.
 * Maps application-level setpoints (duty / current / speed / position)
 * to the FOC current-loop state (foc_state.id_set / iq_set) or the
 * BLDC six-step state (bldc_state), which are consumed by the ADC1 ISR
 * in interrupts.c at the PWM rate.
 *
 * Architecture (VESC-style):
 *   app.c ──► mcpwm_* ──► foc_state / bldc_state ──► ADC1_ISR
 *                                                          │
 *                                                    ┌─────┴─────┐
 *                                                    │  FOC loop │
 *                                                    │ or BLDC   │
 *                                                    └───────────┘
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include <stdint.h>
#include "mcpwm.h"
#include "driver/mcu/mcu_init.h"
#include "motor/bldc.h"
#include "motor/observer.h"
#include "motor/foc.h"
#include "util/fixedpoint.h"

/* ======================================================================== */
/* External Global Instances (defined in main.c)                             */
/* ======================================================================== */
extern motor_state_t        motor_state;
extern foc_state_t          foc_state;
extern observer_state_t     observer_state;
extern bldc_state_t         bldc_state;

extern mc_configuration_t  mc_conf;

/* ======================================================================== */
/* Internal State                                                            */
/* ======================================================================== */
static motor_mode_t  s_mode = MOTOR_MODE_DISABLED;
static float         s_duty = 0.0f;

/* ======================================================================== */
/* mcpwm_init                                                                */
/* ======================================================================== */
void mcpwm_init(mc_configuration_t *conf)
{
    s_mode = MOTOR_MODE_DISABLED;
    s_duty = 0.0f;

    /* Set initial FOC setpoints to zero. */
    foc_state.id_set = 0;
    foc_state.iq_set = 0;
    foc_state.speed_set = 0;

    /* Reset BLDC state. */
    bldc_reset();
}

/* ======================================================================== */
/* mcpwm_stop                                                                */
/* ======================================================================== */
void mcpwm_stop(void)
{
    s_mode = MOTOR_MODE_DISABLED;

    /* Zero FOC setpoints - ISR will drive zero current. */
    foc_state.id_set = 0;
    foc_state.iq_set = 0;
    foc_state.speed_set = 0;

    /* Reset PI integrators for a clean restart later. */
    foc_reset(&foc_state);

    /* Stop BLDC commutation. */
    bldc_reset();
}

/* ======================================================================== */
/* mcpwm_set_duty - Open-loop duty control (BLDC or FOC)                    */
/* ======================================================================== */
void mcpwm_set_duty(float duty)
{
    s_duty = duty;
    s_mode = MOTOR_MODE_DUTY;

    if (mc_conf.comm_mode != (uint8_t)COMM_MODE_BLDC) {
        /* FOC mode: treat duty as Iq target scaled by max current.
         * This is the "open-loop" FOC path used during startup. */
        q16_t iq_cmd = FLOAT_TO_Q16(duty * mc_conf.l_max_current);
        foc_state.iq_set = iq_cmd;
        foc_state.id_set = 0;
        foc_state.speed_set = 0;
    } else {
        /* BLDC mode: set duty directly. */
        bldc_set_duty(FLOAT_TO_Q16(duty));
        if (!bldc_state.is_running) {
            bldc_start_ramp();
        }
    }
}

/* ======================================================================== */
/* mcpwm_set_current - Closed-loop current control (FOC)                     */
/* ======================================================================== */
void mcpwm_set_current(float current)
{
    s_mode = MOTOR_MODE_CURRENT;

    if (mc_conf.comm_mode != (uint8_t)COMM_MODE_BLDC) {
        q16_t iq_cmd = FLOAT_TO_Q16(current);
        foc_state.iq_set   = iq_cmd;
        foc_state.id_set   = 0;
        foc_state.speed_set = 0;
        foc_state.start_iq_target = iq_cmd;   /* sync target for ramp  */
        /* Cancel any active startup ramp since the new command is explicit */
        foc_state.start_ramp_active = 0;
        foc_state.start_ramp_ticks  = 0;
        foc_state.start_iq_current  = iq_cmd;
    } else {
        /* BLDC fallback: approximate current with duty */
        float duty = current / mc_conf.l_max_current;
        bldc_set_duty(FLOAT_TO_Q16(duty));
        if (!bldc_state.is_running) {
            bldc_start_ramp();
        }
    }
}

/* ======================================================================== */
/* mcpwm_set_current_brake - Brake (negative current)                        */
/* ======================================================================== */
void mcpwm_set_current_brake(float current)
{
    s_mode = MOTOR_MODE_CURRENT_BRAKE;

    if (mc_conf.comm_mode != (uint8_t)COMM_MODE_BLDC) {
        foc_state.iq_set = FLOAT_TO_Q16(current);
        foc_state.id_set = 0;
        foc_state.speed_set = 0;
    } else {
        float duty = current / mc_conf.l_max_current;
        bldc_set_duty(FLOAT_TO_Q16(duty));
        bldc_set_direction(-1);
        if (!bldc_state.is_running) {
            bldc_start_ramp();
        }
    }
}

/* ======================================================================== */
/* mcpwm_set_speed - Closed-loop speed control (FOC)                         */
/* ======================================================================== */
void mcpwm_set_speed(float speed)
{
    s_mode = MOTOR_MODE_SPEED;

    if (mc_conf.comm_mode != (uint8_t)COMM_MODE_BLDC) {
        /* Set the speed reference.  The foc_speed_loop, called at 1 kHz
         * from the main loop, will then compute Iq via PI + feedforward. */
        foc_state.speed_set = FLOAT_TO_Q16(speed);
        foc_state.id_set = 0;
    } else {
        /* BLDC: open-loop speed via duty */
        float duty = speed / mc_conf.l_max_speed;
        duty = duty > mc_conf.l_max_duty ? mc_conf.l_max_duty : duty;
        duty = duty < -mc_conf.l_max_duty ? -mc_conf.l_max_duty : duty;
        bldc_set_duty(FLOAT_TO_Q16(duty));
        if (!bldc_state.is_running) {
            bldc_start_ramp();
        }
    }
}

/* ======================================================================== */
/* mcpwm_set_position - Position control (FOC)                              */
/* ======================================================================== */
void mcpwm_set_position(float pos)
{
    (void)pos;
    s_mode = MOTOR_MODE_POSITION;

    if (mc_conf.comm_mode != (uint8_t)COMM_MODE_BLDC) {
        foc_state.iq_set = 0;
        foc_state.id_set = 0;
        foc_state.speed_set = 0;
        /* Position control loop will be added in a future iteration.
         * For now, just hold at the current position (zero Iq).        */
    } else {
        bldc_reset();
    }
}

/* ======================================================================== */
/* mcpwm_get_state - Copy real-time state to the application layer          */
/* ======================================================================== */
void mcpwm_get_state(motor_state_t *st)
{
    if (st == 0) { return; }

    /* Copy from the global motor_state (updated by ADC1_ISR). */
    *st = motor_state;

    /* Add duty from our tracking. */
    st->duty = s_duty;
}

/* ======================================================================== */
/* mcpwm_sound_tone - Inject a high-frequency tone through the inverter      */
/* ======================================================================== */
void mcpwm_sound_tone(float freq_hz, float amplitude)
{
    /* Sound generation: inject a high-frequency voltage via the FOC loop.
     * This will be implemented in a future iteration using HFI-style
     * injection through the SVPWM modulator.
     *
     * For now, a non-zero amplitude sets up a placeholder that the
     * FOC ISR can pick up. */
    if (amplitude > 0.0f) {
        /* Sound playing - handled in app_sound_process() */
    } else {
        /* Silence - no injection */
    }
}
