/*
 * app.c - Main Application Controller
 *
 * Sits between the user interfaces (UART/PPM/sound) and the motor PWM
 * backend (FOC or BLDC). Owns:
 *   - a run-state machine  INIT -> IDLE -> RUNNING -> FAULT
 *   - control-mode selection (duty / current / speed / position / brake)
 *   - setpoint ramp generators for smooth transitions
 *   - safety supervision (over-current / over-voltage / under-voltage /
 *     over-temperature) that trips the controller into FAULT
 *
 * The motor backend (mcpwm_*) is accessed through weak stubs defined here so
 * the application links standalone; the real FOC/BLDC layer overrides them
 * when it is present. This mirrors the VESC architecture where mcpwm.c wraps
 * the foc/bldc cores.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include "app.h"
#include "conf/conf_general.h"
#include "config/hwconf.h"
#include "config/board_config.h"
#include "driver/mcu/mcu_init.h"

/* ======================================================================== */
/* Motor PWM Backend (weak stubs - override in mcpwm.c / foc.c / bldc.c)     */
/* ======================================================================== */
/* The real backend overrides these to drive FOC or BLDC depending on
 * mc_conf. Returning a no-op keeps the application linkable in isolation. */
__attribute__((weak)) void  mcpwm_init(mc_configuration_t *conf)            { (void)conf; }
__attribute__((weak)) void  mcpwm_stop(void)                                {}
__attribute__((weak)) void  mcpwm_set_duty(float duty)                      { (void)duty; }
__attribute__((weak)) void  mcpwm_set_current(float current)               { (void)current; }
__attribute__((weak)) void  mcpwm_set_current_brake(float current)          { (void)current; }
__attribute__((weak)) void  mcpwm_set_speed(float speed)                    { (void)speed; }
__attribute__((weak)) void  mcpwm_set_position(float pos)                   { (void)pos; }
__attribute__((weak)) void  mcpwm_get_state(motor_state_t *st)              { (void)st; }
/* Inject a high-frequency voltage tone through the FOC inverter. */
__attribute__((weak)) void  mcpwm_sound_tone(float freq_hz, float amplitude){ (void)freq_hz; (void)amplitude; }

/* ======================================================================== */
/* Local State                                                               */
/* ======================================================================== */
static mc_configuration_t  *s_mc_conf;
static app_configuration_t *s_app_conf;

static motor_state_t  s_state;          /* live real-time state            */
static app_state_t    s_run_state = APP_STATE_INIT;
static motor_mode_t  s_mode       = MOTOR_MODE_DISABLED;

/* Active / target setpoints (in physical units). */
static float s_duty_now,    s_duty_tgt;
static float s_current_now, s_current_tgt;
static float s_speed_now,   s_speed_tgt;
static float s_pos_tgt;
static float s_brake_now,   s_brake_tgt;

/* Ramp generators: each ramps its "_now" toward its "_tgt" at rate /s. */
typedef struct {
    float now;
    float target;
    float rate;          /* max change per second */
} ramp_t;

#define DUTY_RAMP_RATE    2.0f     /* 0..1 in 0.5 s                        */
#define CURRENT_RAMP_RATE 4000.0f /* A/s, fast current loop setpoint       */
#define SPEED_RAMP_RATE   20000.0f /* ERPM/s                              */
#define BRAKE_RAMP_RATE   4000.0f

/* Timing bookkeeping for dt. */
static uint32_t s_last_ms;

/* Fault-clear timing: minimum dwell in FAULT before allowing IDLE. */
#define FAULT_DWELL_MS   1000U
static uint32_t s_fault_enter_ms;

/* ======================================================================== */
/* Local Helpers                                                             */
/* ======================================================================== */
static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

static float ramp_update(ramp_t *r, float dt)
{
    float diff = r->target - r->now;
    float step = r->rate * dt;
    if (diff >  step) { r->now += step; }
    else if (diff < -step) { r->now -= step; }
    else { r->now = r->target; }
    return r->now;
}

/* ======================================================================== */
/* Safety Supervision                                                        */
/* ======================================================================== */
/* Returns the fault code, or FAULT_NONE if everything is within limits. */
static fault_code_t app_check_safety(void)
{
    const motor_state_t *st = &s_state;

    /* Over-current: phase or q-axis current magnitude. */
    float i_mag = st->iq;
    if (i_mag < 0.0f) { i_mag = -i_mag; }
    if (i_mag > (s_mc_conf->l_max_current * 1.25f)) {
        return FAULT_OVER_CURRENT;
    }

    /* Bus voltage window. */
    if (st->vbus > VBUS_WARN_V) {
        return FAULT_OVER_VOLTAGE;
    }
    if (st->vbus < VBUS_MIN_V) {
        return FAULT_UNDER_VOLTAGE;
    }

    /* Temperature limits. */
    if (st->temp_motor  > s_mc_conf->l_temp_motor_max) {
        return FAULT_OVER_TEMP_MOTOR;
    }
    if (st->temp_mosfet > s_mc_conf->l_temp_mosfet_max) {
        return FAULT_OVER_TEMP_MOSFET;
    }

    return FAULT_NONE;
}

static void app_enter_fault(fault_code_t f)
{
    s_state.fault = (uint8_t)f;
    s_run_state = APP_STATE_FAULT;
    s_fault_enter_ms = mcu_millis();
    mcpwm_stop();
    /* silence any active tone */
    mcpwm_sound_tone(0.0f, 0.0f);
}

/* ======================================================================== */
/* Lifecycle                                                                 */
/* ======================================================================== */
void app_init(mc_configuration_t *mc_conf, app_configuration_t *app_conf)
{
    s_mc_conf  = mc_conf;
    s_app_conf = app_conf;

    /* Zero all state. */
    s_state.fault = (uint8_t)FAULT_NONE;
    s_mode = (motor_mode_t)app_conf->app_mode;

    s_duty_now = s_duty_tgt = 0.0f;
    s_current_now = s_current_tgt = 0.0f;
    s_speed_now = s_speed_tgt = 0.0f;
    s_brake_now = s_brake_tgt = 0.0f;
    s_pos_tgt = 0.0f;

    s_last_ms = mcu_millis();

    /* Bring up the motor backend with the loaded configuration. */
    mcpwm_init(mc_conf);

    /* Bring up the interfaces. */
    app_uart_init();
    app_ppm_init();
    app_sound_init();

    /* Startup melody (non-blocking: queued in app_sound). */
    app_sound_startup();

    s_run_state = APP_STATE_IDLE;
}

void app_stop(void)
{
    /* Ramp targets to zero; the loop will settle the motor and drop to IDLE. */
    s_duty_tgt    = 0.0f;
    s_current_tgt = 0.0f;
    s_speed_tgt   = 0.0f;
    s_brake_tgt   = 0.0f;
    s_mode = MOTOR_MODE_DISABLED;

    mcpwm_stop();

    if (s_run_state == APP_STATE_RUNNING) {
        s_run_state = APP_STATE_IDLE;
    }
}

/* ======================================================================== */
/* Setpoint API                                                              */
/* ======================================================================== */
void app_set_duty(float duty)
{
    if (s_run_state == APP_STATE_FAULT) { return; }
    duty = clampf(duty, s_mc_conf->l_min_duty, s_mc_conf->l_max_duty);
    s_mode = MOTOR_MODE_DUTY;
    s_duty_tgt = duty;
    if (s_run_state == APP_STATE_IDLE) { s_run_state = APP_STATE_RUNNING; }
}

void app_set_current(float current)
{
    if (s_run_state == APP_STATE_FAULT) { return; }
    current = clampf(current, s_mc_conf->l_min_current, s_mc_conf->l_max_current);
    s_mode = MOTOR_MODE_CURRENT;
    s_current_tgt = current;
    if (s_run_state == APP_STATE_IDLE) { s_run_state = APP_STATE_RUNNING; }
}

void app_set_speed(float speed)
{
    if (s_run_state == APP_STATE_FAULT) { return; }
    speed = clampf(speed, s_mc_conf->l_min_speed, s_mc_conf->l_max_speed);
    s_mode = MOTOR_MODE_SPEED;
    s_speed_tgt = speed;
    if (s_run_state == APP_STATE_IDLE) { s_run_state = APP_STATE_RUNNING; }
}

void app_set_position(float pos)
{
    if (s_run_state == APP_STATE_FAULT) { return; }
    s_mode = MOTOR_MODE_POSITION;
    s_pos_tgt = pos;
    if (s_run_state == APP_STATE_IDLE) { s_run_state = APP_STATE_RUNNING; }
}

void app_set_brake(float brake)
{
    if (s_run_state == APP_STATE_FAULT) { return; }
    if (brake < 0.0f) { brake = 0.0f; }
    if (brake > (-s_mc_conf->l_min_current)) { brake = -s_mc_conf->l_min_current; }
    s_mode = MOTOR_MODE_CURRENT_BRAKE;
    s_brake_tgt = brake;
    if (s_run_state == APP_STATE_IDLE) { s_run_state = APP_STATE_RUNNING; }
}

/* ======================================================================== */
/* State Access                                                              */
/* ======================================================================== */
motor_state_t *app_get_state(void)        { return &s_state; }
app_state_t    app_get_run_state(void)    { return s_run_state; }

/* ======================================================================== */
/* Main Loop                                                                 */
/* ======================================================================== */
void app_process(void)
{
    uint32_t now = mcu_millis();
    float dt = (float)(now - s_last_ms) * 0.001f;
    s_last_ms = now;
    if (dt < 0.0f) { dt = 0.0f; }
    if (dt > 0.1f) { dt = 0.1f; }   /* clamp large gaps (e.g. debug pause) */

    /* 1. Refresh real-time state from the motor backend. */
    mcpwm_get_state(&s_state);

    /* 2. Safety supervision (skipped while in INIT). */
    if (s_run_state != APP_STATE_INIT) {
        fault_code_t f = app_check_safety();
        if (f != FAULT_NONE) {
            app_enter_fault(f);
        }
    }

    /* 3. State machine + ramping. */
    switch (s_run_state) {
    case APP_STATE_INIT:
        /* app_init transitions to IDLE; nothing to do here. */
        break;

    case APP_STATE_FAULT:
        /* Hold stopped. Allow clearing after the dwell period if the
         * safety condition has gone away. */
        if ((now - s_fault_enter_ms) >= FAULT_DWELL_MS) {
            fault_code_t f = app_check_safety();
            if (f == FAULT_NONE) {
                s_state.fault = (uint8_t)FAULT_NONE;
                s_run_state = APP_STATE_IDLE;
            }
        }
        break;

    case APP_STATE_IDLE:
        /* Nothing driving the motor; keep setpoints parked at zero. */
        mcpwm_set_current(0.0f);
        break;

    case APP_STATE_RUNNING: {
        ramp_t r;
        switch (s_mode) {
        case MOTOR_MODE_DUTY:
            r.now = s_duty_now; r.target = s_duty_tgt; r.rate = DUTY_RAMP_RATE;
            s_duty_now = ramp_update(&r, dt);
            mcpwm_set_duty(s_duty_now);
            break;
        case MOTOR_MODE_CURRENT:
            r.now = s_current_now; r.target = s_current_tgt; r.rate = CURRENT_RAMP_RATE;
            s_current_now = ramp_update(&r, dt);
            mcpwm_set_current(s_current_now);
            break;
        case MOTOR_MODE_SPEED:
            r.now = s_speed_now; r.target = s_speed_tgt; r.rate = SPEED_RAMP_RATE;
            s_speed_now = ramp_update(&r, dt);
            mcpwm_set_speed(s_speed_now);
            break;
        case MOTOR_MODE_POSITION:
            mcpwm_set_position(s_pos_tgt);
            break;
        case MOTOR_MODE_CURRENT_BRAKE:
            r.now = s_brake_now; r.target = s_brake_tgt; r.rate = BRAKE_RAMP_RATE;
            s_brake_now = ramp_update(&r, dt);
            mcpwm_set_current_brake(s_brake_now);
            break;
        default:
            /* No active mode: drop to idle. */
            s_run_state = APP_STATE_IDLE;
            break;
        }

        /* Auto-idle when a symmetric mode settles at zero setpoint. */
        if ((s_mode == MOTOR_MODE_DUTY    && s_duty_now    == 0.0f && s_duty_tgt    == 0.0f) ||
            (s_mode == MOTOR_MODE_CURRENT && s_current_now == 0.0f && s_current_tgt == 0.0f) ||
            (s_mode == MOTOR_MODE_SPEED   && s_speed_now   == 0.0f && s_speed_tgt   == 0.0f)) {
            /* keep running until explicitly stopped; user may re-arm. */
        }
        break;
    }

    default:
        s_run_state = APP_STATE_IDLE;
        break;
    }

    /* 4. Service the user interfaces. */
    app_uart_process();
    app_ppm_process();
    app_sound_process();
}
