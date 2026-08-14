/*
 * encoder.h - Hall Sensor / Encoder Interface (Stub)
 *
 * Provides the position/speed feedback API for sensored operation. The WFOC
 * V1 board has no dedicated Hall-sensor pins or SPI encoder header, so this
 * module is intentionally a stub: the functions are declared and linked so the
 * rest of the firmware compiles, but they return safe defaults until real
 * hardware is wired up.
 *
 * The interface mirrors the VESC encoder abstraction:
 *   - Hall sensors  : 3 digital inputs -> 6-step electrical angle + speed
 *   - SPI encoder   : AS5047 / AS5048A / MT6701 absolute magnetic encoder
 *   - SSI / ABI     : reserved for future expansion
 *
 * Angle format: uint16_t 0..65535 = 0..360 electrical degrees (matches the
 * fixed-point trig tables in fixedpoint.h). Speed is returned in Q16.16
 * electrical rad/s.
 *
 * Target: ARM Cortex-M0+ (CIU32F003x5) - No FPU.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under the MIT License
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "util/fixedpoint.h"
#include "conf/datatypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* Encoder Type                                                              */
/* ======================================================================== */
typedef enum {
    ENCODER_TYPE_NONE = 0,      /* No encoder (sensorless operation)        */
    ENCODER_TYPE_HALL,          /* 3-phase Hall sensors (6-step)           */
    ENCODER_TYPE_SPI_ABS,       /* SPI absolute magnetic encoder (AS5047)   */
    ENCODER_TYPE_SSI,           /* SSI absolute encoder (future)            */
    ENCODER_TYPE_ABI,           /* ABI incremental encoder (future)         */
} encoder_type_t;

/* ======================================================================== */
/* Encoder Configuration                                                     */
/* ======================================================================== */
typedef struct {
    encoder_type_t  type;            /* Hardware type                          */
    uint8_t         hall_pole_pairs;  /* Motor pole pairs (for Hall speed calc) */
    int8_t          hall_dir;        /* +1 or -1 sign for direction correction */
    uint16_t        spi_cpr;          /* Counts-per-revolution (SPI encoder)    */
    bool            inverted;        /* Reverse angle sense                    */
    uint16_t        offset_angle;     /* Zero-position offset (angle_16 units) */
} encoder_config_t;

/* ======================================================================== */
/* Encoder Runtime State                                                     */
/* ======================================================================== */
typedef struct {
    encoder_type_t  type;
    uint16_t        angle;          /* Latest electrical angle 0..65535       */
    q16_t           speed;         /* Electrical speed (Q16.16 rad/s)        */
    uint16_t        last_angle;    /* Previous sample for speed differentiation*/
    uint8_t         hall_state;    /* Raw 3-bit Hall pattern (0..7)          */
    uint8_t         hall_last;     /* Previous Hall pattern (glitch filter)  */
    uint32_t        hall_ticks;    /* ISR ticks since last Hall transition   */
    uint32_t        hall_period;   /* ISR ticks for last 60-deg Hall step    */
    q16_t           hall_speed_lpf;/* Filtered Hall speed                    */
    bool            present;       /* True if hardware responded             */
} encoder_state_t;

/* ======================================================================== */
/* Public API                                                                */
/* ======================================================================== */

/* Initialize the encoder subsystem with the given configuration. Called once
 * at startup (NON-ISR; may use float). On WFOC V1 with no hardware attached
 * this leaves the encoder disabled (present == false). */
void encoder_init(encoder_state_t *enc, const encoder_config_t *cfg);

/* Reset the encoder runtime state (angle/speed/history) without re-running
 * hardware detection. Called when entering/exiting sensored mode. */
void encoder_reset(encoder_state_t *enc);

/* Sample the encoder once. Called from the main ISR (e.g. 20 kHz). Reads the
 * hardware (Hall GPIOs or SPI), updates the angle and differentiates speed.
 * All math is integer/fixed-point. */
void encoder_sample_isr(encoder_state_t *enc);

/* Return the latest electrical angle (0..65535 = 0..360 deg). Returns the
 * last good value if the encoder is not present. */
uint16_t encoder_get_angle(encoder_state_t *enc);

/* Return the latest electrical speed in Q16.16 rad/s. Returns 0 when the
 * encoder is not present. */
q16_t encoder_get_speed(encoder_state_t *enc);

/* Return true if the encoder hardware is present and reporting valid data. */
bool encoder_is_present(encoder_state_t *enc);

/* ======================================================================== */
/* Hall Sensor Helpers (stubbed on WFOC V1 - no Hall pins)                    */
/* ======================================================================== */
/* Read the 3 Hall inputs into a 3-bit value (HA<<2 | HB<<1 | HC<<0).
 * On WFOC V1 this always returns 0 because no Hall GPIOs are wired. The
 * function is declared so sensor-mode HALL links cleanly. */
uint8_t encoder_read_hall(void);

/* ======================================================================== */
/* SPI Encoder Helpers (stubbed for future AS5047 / MT6701 support)          */
/* ======================================================================== */
/* Read a 14-bit absolute position from an SPI magnetic encoder. Returns 0 on
 * WFOC V1 (no SPI encoder connected). Implement in platform driver when the
 * hardware is available. */
uint16_t encoder_read_spi_absolute(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
