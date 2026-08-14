/*
 * encoder.c - Hall Sensor / Encoder Interface (Stub)
 *
 * Implementation of the encoder API declared in encoder.h. On the WFOC V1
 * board there are no dedicated Hall-sensor GPIOs and no SPI encoder header,
 * so every hardware access is stubbed. The functions exist so that sensored
 * code paths link cleanly and can be enabled later by providing overrides
 * (or by extending this file) once the hardware is wired.
 *
 * The speed-differentiation and angle-offset logic is implemented here in
 * fixed-point so it is ready to use the moment real angle data is supplied.
 *
 * Target: ARM Cortex-M0+ (CIU32F003x5) - No FPU.
 * All ISR-time math uses Q16.16 fixed-point (no floating-point).
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under the MIT License
 */

#include "encoder.h"
#include "config/hwconf.h"

/* ======================================================================== */
/* Module Configuration (single active encoder instance)                     */
/* ======================================================================== */
static encoder_config_t s_cfg;

/* Hall sector -> electrical angle (mid-sector) lookup.
 * 6 Hall states map to 6 electrical angles spaced 60 deg apart.
 * Index by raw 3-bit Hall pattern; entries for invalid patterns are 0.
 * Angles in angle_16 units (0..65535 = 0..360 deg).
 *
 * The exact mapping depends on Hall sensor placement. This is the canonical
 * 120-deg-spaced Hall table; it can be recalibrated via offset_angle / dir.
 */
static const uint16_t hall_angle_table[8] = {
       0,    /* 000 invalid */
       0,    /* 001 ->   0 deg */
   10923,    /* 010 ->  60 deg */
   21845,    /* 011 -> 120 deg */
   32768,    /* 100 -> 180 deg */
   43690,    /* 101 -> 240 deg */
   54613,    /* 110 -> 300 deg */
   65535,    /* 111 invalid */
};

/* ======================================================================== */
/* ISR Tick Period (seconds, Q16.16)                                          */
/* ======================================================================== */
/* Precomputed at init from PWM frequency: dt = 1 / FOC_LOOP_FREQ_HZ.
 * 20 kHz -> 50 us -> dt_q16 = FLOAT_TO_Q16(50e-6) = 3277. */
static q16_t s_dt_q16;

/* Speed low-pass filter coefficient. A light filter (tau ~ 2 ms) keeps the
 * differentiated speed usable for the speed loop without too much lag. */
static q16_t s_speed_alpha = 8192;   /* ~0.125 of new sample */

/* ======================================================================== */
/* Initialization                                                            */
/* ======================================================================== */

void encoder_init(encoder_state_t *enc, const encoder_config_t *cfg)
{
    if (enc == 0) {
        return;
    }

    /* Store configuration copy */
    if (cfg != 0) {
        s_cfg = *cfg;
    } else {
        s_cfg.type           = ENCODER_TYPE_NONE;
        s_cfg.hall_pole_pairs = 0;
        s_cfg.hall_dir       = +1;
        s_cfg.spi_cpr        = 0;
        s_cfg.inverted        = false;
        s_cfg.offset_angle    = 0;
    }

    /* Precompute ISR sample period (Q16.16 seconds).
     * FOC loop runs at FOC_LOOP_FREQ_HZ (= PWM_FREQ_HZ, default 20 kHz). */
    s_dt_q16 = FLOAT_TO_Q16(1.0f / (float)FOC_LOOP_FREQ_HZ);

    encoder_reset(enc);

    /* Stub: no hardware to detect. Mark as absent so the caller falls back
     * to the sensorless observer. A future platform override of
     * encoder_read_spi_absolute() / encoder_read_hall() should set
     * enc->present = true after a successful first transaction. */
    enc->type    = s_cfg.type;
    enc->present = false;
}

void encoder_reset(encoder_state_t *enc)
{
    if (enc == 0) {
        return;
    }

    enc->angle          = 0;
    enc->speed          = 0;
    enc->last_angle     = 0;
    enc->hall_state     = 0;
    enc->hall_last      = 0;
    enc->hall_ticks     = 0;
    enc->hall_period    = 0;
    enc->hall_speed_lpf = 0;
}

/* ======================================================================== */
/* Hardware Accessors (Stubs)                                                */
/* ======================================================================== */
/* On WFOC V1 there are no Hall GPIOs. Override these in a platform driver
 * when hardware is added. */

uint8_t encoder_read_hall(void)
{
    /* No Hall pins on WFOC V1 - always returns 0. */
    return 0;
}

uint16_t encoder_read_spi_absolute(void)
{
    /* No SPI encoder on WFOC V1 - always returns 0. */
    return 0;
}

/* ======================================================================== */
/* Sampling                                                                  */
/* ======================================================================== */

void encoder_sample_isr(encoder_state_t *enc)
{
    uint16_t raw_angle;
    uint16_t new_angle;
    int32_t  delta;
    q16_t    inst_speed;

    if (enc == 0) {
        return;
    }

    switch (enc->type) {
    case ENCODER_TYPE_HALL:
        /* --- Hall sensor sampling (stubbed, no hardware) --- */
        enc->hall_state  = encoder_read_hall();
        enc->hall_ticks++;

        /* Detect a Hall transition (glitch-filtered) */
        if (enc->hall_state != enc->hall_last && enc->hall_state != 0 &&
            enc->hall_state != 7) {
            enc->hall_period   = enc->hall_ticks;
            enc->hall_ticks    = 0;
            enc->hall_last     = enc->hall_state;

            /* 60 electrical degrees per Hall transition.
             *   speed_elec(rad/s) = (pi/3) / period_seconds
             *   period_seconds = hall_period * dt
             * In Q16: speed_q16 = Q16_PI_3 / (hall_period * s_dt_q16)
             * Computed via int64 to stay overflow-safe for slow motors. */
            if (enc->hall_period > 0) {
                q16_t period_q16 = (q16_t)((int64_t)s_dt_q16 *
                                          (int64_t)enc->hall_period);
                inst_speed = Q16_DIV(Q16_PI_3, period_q16);
                if (s_cfg.hall_dir < 0) {
                    inst_speed = -inst_speed;
                }
                enc->hall_speed_lpf += (q16_t)
                    (((int64_t)(inst_speed - enc->hall_speed_lpf) *
                      s_speed_alpha) >> 16);
            }

            enc->angle = hall_angle_table[enc->hall_state];
        }
        enc->speed = enc->hall_speed_lpf;
        break;

    case ENCODER_TYPE_SPI_ABS:
        /* --- SPI absolute encoder (stubbed, no hardware) --- */
        raw_angle = encoder_read_spi_absolute();
        if (raw_angle != 0) {
            enc->present = true;
        }
        /* Map counts-per-revolution -> angle_16.
         * new_angle = raw * 65536 / cpr */
        if (s_cfg.spi_cpr > 0) {
            new_angle = (uint16_t)(((uint32_t)raw_angle * 65536U) /
                                   (uint32_t)s_cfg.spi_cpr);
        } else {
            new_angle = 0;
        }
        if (s_cfg.inverted) {
            new_angle = (uint16_t)(65536U - new_angle);
        }
        new_angle = (uint16_t)(new_angle + s_cfg.offset_angle);

        /* Differentiate speed with wrap-around handling.
         * delta is in angle_16 counts (1 count = 2*pi/65536 rad).
         *   speed_q16 = delta * 2*pi / dt   (Q16.16 rad/s, see encoder.h) */
        delta = (int32_t)new_angle - (int32_t)enc->last_angle;
        if (delta >  32768)  delta -= 65536;
        if (delta < -32768)  delta += 65536;

        inst_speed = (q16_t)(((int64_t)delta * (int64_t)Q16_2PI) / s_dt_q16);

        enc->hall_speed_lpf += (q16_t)
            (((int64_t)(inst_speed - enc->hall_speed_lpf) * s_speed_alpha) >> 16);
        enc->speed      = enc->hall_speed_lpf;
        enc->last_angle = new_angle;
        enc->angle      = new_angle;
        break;

    case ENCODER_TYPE_SSI:
    case ENCODER_TYPE_ABI:
        /* Reserved for future incremental/SSI encoders. Not implemented. */
        break;

    case ENCODER_TYPE_NONE:
    default:
        /* Sensorless: no hardware. Leave angle/speed at last value. */
        break;
    }
}

/* ======================================================================== */
/* Accessors                                                                 */
/* ======================================================================== */

uint16_t encoder_get_angle(encoder_state_t *enc)
{
    if (enc == 0) {
        return 0;
    }
    return enc->angle;
}

q16_t encoder_get_speed(encoder_state_t *enc)
{
    if (enc == 0) {
        return 0;
    }
    return enc->speed;
}

bool encoder_is_present(encoder_state_t *enc)
{
    if (enc == 0) {
        return false;
    }
    return enc->present;
}
