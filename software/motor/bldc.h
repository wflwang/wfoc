/*
 * bldc.h - BLDC Six-Step Commutation (VESC-style)
 *
 * Sensorless brushless DC motor control using back-EMF zero-crossing
 * detection. Implements a 6-step commutation sequence, adaptive ZCD
 * timing, open-loop startup ramp, and closed-loop BEMF tracking.
 *
 * Target: ARM Cortex-M0+ (CIU32F003x5) - No FPU.
 * All ISR-time math uses Q16.16 fixed-point (no floating-point).
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef BLDC_H
#define BLDC_H

#include <stdint.h>
#include <stdbool.h>
#include "util/fixedpoint.h"
#include "conf/datatypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* ADC Value Array Indices                                                    */
/* ======================================================================== */
/* Expected layout of the adc_values[] array passed to bldc_run_isr().
 * The platform ADC driver must populate the array in this order. */
#define BLDC_ADC_IA         0       /* Phase A current (unused by BLDC ZCD)  */
#define BLDC_ADC_IB         1       /* Phase B current (unused by BLDC ZCD)  */
#define BLDC_ADC_VA         2       /* Phase A (U) voltage                   */
#define BLDC_ADC_VB         3       /* Phase B (V) voltage                   */
#define BLDC_ADC_VC         4       /* Phase C (W) voltage                   */
#define BLDC_ADC_VBUS       5       /* DC bus voltage                        */
#define BLDC_ADC_TEMP       6       /* Temperature (unused by BLDC)          */
#define BLDC_ADC_COUNT      7       /* Total ADC values expected             */

/* ======================================================================== */
/* Phase Index Helpers                                                        */
/* ======================================================================== */
#define BLDC_PHASE_A        0
#define BLDC_PHASE_B        1
#define BLDC_PHASE_C        2

/* ======================================================================== */
/* Startup / Run State Machine                                                */
/* ======================================================================== */
typedef enum {
    BLDC_STATE_IDLE = 0,        /* Not running                                */
    BLDC_STATE_ALIGN,           /* Phase 1: align rotor to known position     */
    BLDC_STATE_RAMP,            /* Phase 2: open-loop forced commutation      */
    BLDC_STATE_CLOSED_LOOP,     /* Phase 3: BEMF-based closed-loop commutation*/
    BLDC_STATE_FALLBACK,        /* BEMF lost - returning to open-loop ramp    */
} bldc_run_state_t;

/* ======================================================================== */
/* BLDC Runtime State                                                         */
/* ======================================================================== */
typedef struct {
    /* --- Public state (specified) --- */
    uint8_t     comm_step;       /* Current commutation step (0-5)            */
    int8_t      direction;       /* Rotation direction: +1 or -1              */
    q16_t       rpm;             /* Current mechanical speed (Q16.16 RPM)     */
    q16_t       comm_period;     /* Time between commutations (Q16.16 seconds)*/
    q16_t       bemf_voltage;    /* Floating-phase BEMF voltage (Q16.16 V)    */
    bool        zcd_detected;    /* Zero-crossing detected this step          */
    uint16_t    zcd_timeout;     /* ZCD timeout counter (ISR ticks)           */
    uint16_t    comp_comm_timer; /* Commutation delay timer (ISR ticks)       */
    bldc_run_state_t ramp_state; /* Startup / run state                       */

    /* --- Internal algorithm state --- */
    uint32_t    comm_tick_counter;      /* ISR ticks since last commutation  */
    uint32_t    last_comm_period_ticks; /* Previous commutation period (ticks)*/
    uint16_t    zcd_to_comm_delay;      /* ZCD->commute delay (ticks, 30 deg)*/
    uint16_t    align_timer;            /* Alignment phase timer (ticks)     */
    uint16_t    ramp_timer;             /* Ramp phase timer (ticks)          */
    uint16_t    ramp_step_period;       /* Forced comm period during ramp    */
    q16_t       ramp_duty;              /* Duty applied during ramp (Q16)    */
    uint16_t    bemf_rising;            /* 1: rising ZCD expected, 0: falling*/
    uint8_t     floating_phase;         /* 0=A,1=B,2=C floating this step    */
    int32_t     prev_float_adc;         /* Previous floating-phase ADC sample*/
    bool        is_running;             /* True when commutation is active   */
} bldc_state_t;

/* ======================================================================== */
/* Public API                                                                 */
/* ======================================================================== */

/* Initialize the BLDC subsystem from a motor configuration.
 * Called once at startup (NON-ISR; may use float to pre-compute gains). */
void bldc_init(mc_configuration_t *conf);

/* Reset the BLDC runtime state to idle (stops commutation). */
void bldc_reset(void);

/* Main ISR entry point. Called once per PWM cycle (e.g. 20 kHz).
 * Reads phase voltages from adc_values, performs ZCD, commutation timing,
 * speed estimation, and applies phase outputs. All operations use
 * fixed-point arithmetic only. */
void bldc_run_isr(bldc_state_t *state, uint16_t *adc_values);

/* Set the PWM duty cycle (Q16.16, 0..Q16_ONE). */
void bldc_set_duty(q16_t duty);

/* Set rotation direction: +1 (forward) or -1 (reverse). */
void bldc_set_direction(int8_t dir);

/* Begin the sensorless startup sequence (align -> ramp -> closed-loop). */
void bldc_start_ramp(void);

/* Get the current commutation step (0-5). */
uint8_t bldc_get_comm_step(void);

/* ======================================================================== */
/* Platform Hook (override in HAL / platform code)                            */
/* ======================================================================== */
/* Called from bldc_run_isr to apply phase outputs to the PWM hardware.
 * phase_hi/lo/flt: BLDC_PHASE_A/B/C indicating high-side, low-side, and
 * floating phase. duty_q16: applied duty cycle (Q16.16, 0..Q16_ONE).
 * Provide a non-weak override in the platform driver to drive the timer. */
void bldc_platform_set_phases(uint8_t phase_hi, uint8_t phase_lo,
                              uint8_t phase_flt, q16_t duty_q16);

#ifdef __cplusplus
}
#endif

#endif /* BLDC_H */
