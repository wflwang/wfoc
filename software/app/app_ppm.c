/*
 * app_ppm.c - PPM / RC Control
 *
 * Decodes a standard hobby RC PPM pulse train (1-2 ms, centred on 1.5 ms)
 * captured on the PPM input pin, maps the normalised throttle to the active
 * control mode (duty / speed / current) and applies:
 *   - a centre deadzone so a released stick rests cleanly at zero,
 *   - a ramp so commanded setpoints change smoothly,
 *   - a failsafe that stops the motor when the signal is lost or out of
 *     range.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include "app.h"
#include "conf/conf_general.h"
#include "driver/mcu/mcu_init.h"

/* ======================================================================== */
/* Platform Hook (weak - override in the timer input-capture driver)        */
/* ======================================================================== */
/* Return the latest PPM pulse width in milliseconds, or 0.0 if no valid
 * edge has been seen within the failsafe window. Override in the PPM timer
 * driver to feed real captured widths. */
__attribute__((weak)) float ppm_get_pulse_ms(void)
{
    return 0.0f;
}

/* ======================================================================== */
/* Tuning Constants                                                          */
/* ======================================================================== */
#define PPM_DEADZONE         0.05f    /* +/- 5% centre deadzone            */
#define PPM_FAILSAFE_MS      300U     /* signal-lost trip (ms)            */
#define PPM_OUT_OF_RANGE_MS  0.5f     /* tolerance beyond min/max          */

/* ======================================================================== */
/* Local State                                                               */
/* ======================================================================== */
static float    s_value;            /* smoothed normalised command [-1,1] */
static uint32_t s_last_valid_ms;    /* last time a sane pulse was seen     */
static bool     s_failsafe_active;

/* ======================================================================== */
/* Local Helpers                                                             */
/* ======================================================================== */
static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/* ======================================================================== */
/* Public API                                                                */
/* ======================================================================== */
void app_ppm_init(void)
{
    s_value           = 0.0f;
    s_last_valid_ms   = mcu_millis();
    s_failsafe_active = false;
}

void app_ppm_process(void)
{
    uint32_t now   = mcu_millis();
    float    pulse = ppm_get_pulse_ms();

    /* --- Signal validity ------------------------------------------------- */
    float lo = app_conf.ppm_min - PPM_OUT_OF_RANGE_MS;
    float hi = app_conf.ppm_max + PPM_OUT_OF_RANGE_MS;

    if ((pulse > lo) && (pulse < hi)) {
        s_last_valid_ms   = now;
        s_failsafe_active = false;
    } else {
        /* Lost or invalid signal. */
        if ((now - s_last_valid_ms) > PPM_FAILSAFE_MS) {
            if (!s_failsafe_active) {
                app_stop();
                s_failsafe_active = true;
            }
            s_value = 0.0f;
        }
        return;
    }

    /* --- Normalise to [-1, +1] about the centre -------------------------- */
    float centre = (app_conf.ppm_min + app_conf.ppm_max) * 0.5f;
    float half   = (app_conf.ppm_max - app_conf.ppm_min) * 0.5f;
    if (half < 1e-3f) { half = 0.5f; }   /* guard against bad config */

    float cmd = (pulse - centre) / half;  /* -1 .. +1 */
    cmd = clampf(cmd, -1.0f, 1.0f);

    /* Centre deadzone, then remap to full range. */
    if ((cmd > -PPM_DEADZONE) && (cmd < PPM_DEADZONE)) {
        cmd = 0.0f;
    } else if (cmd >= PPM_DEADZONE) {
        cmd = (cmd - PPM_DEADZONE) / (1.0f - PPM_DEADZONE);
    } else {
        cmd = (cmd + PPM_DEADZONE) / (1.0f - PPM_DEADZONE);
    }

    /* Ramp toward the new command at app_conf.ppm_ramp (1/s). A ~4 ms tick
     * is assumed for the main loop cadence. */
    float step = app_conf.ppm_ramp * 0.001f * 4.0f;
    float diff = cmd - s_value;
    if (diff >  step) { s_value += step; }
    else if (diff < -step) { s_value -= step; }
    else { s_value = cmd; }

    /* --- Map to the active control mode ---------------------------------- */
    switch ((motor_mode_t)app_conf.app_mode) {
    case MOTOR_MODE_DUTY:
        app_set_duty(s_value * mc_conf.l_max_duty);
        break;
    case MOTOR_MODE_SPEED:
        app_set_speed(s_value * mc_conf.l_max_speed);
        break;
    case MOTOR_MODE_CURRENT:
    default:
        /* Negative value commands braking, positive commands motoring. */
        if (s_value >= 0.0f) {
            app_set_current(s_value * mc_conf.l_max_current);
        } else {
            app_set_brake(-s_value * (-mc_conf.l_min_current));
        }
        break;
    }
}
