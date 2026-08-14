/*
 * adc.h - ADC Driver for CIU32F003x5 (Motor Control)
 *
 * 12-bit ADC with DMA transfer synchronized to the TIM1 PWM timer. Samples
 * two phase currents (low-side shunt), three phase voltages, motor NTC
 * temperature and bus voltage once per PWM cycle.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include "ciu32f003x.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* Channel Count & Ordering                                                  */
/* ======================================================================== */
/* Scan order is low-to-high channel number (SCANDIR = 0). Channel 2 (PA2 /
 * SWCLK) is skipped. The DMA buffer index for each signal is fixed below. */
#define ADC_NUM_CHANNELS         7U

enum {
    ADC_IDX_IA   = 0,   /* PB0  - Phase A current  (ADC_CH0) */
    ADC_IDX_IB   = 1,   /* PB1  - Phase B current  (ADC_CH1) */
    ADC_IDX_VOL_U = 2,  /* PA3  - Phase U voltage (ADC_CH3) */
    ADC_IDX_VOL_V = 3,  /* PA4  - Phase V voltage (ADC_CH4) */
    ADC_IDX_VOL_W = 4,  /* PA5  - Phase W voltage (ADC_CH5) */
    ADC_IDX_TEMP = 5,   /* PA6  - Motor temperature (ADC_CH6) */
    ADC_IDX_VBUS = 6,   /* PA7  - Bus voltage       (ADC_CH7) */
};

/* ======================================================================== */
/* Sample Snapshot                                                           */
/* ======================================================================== */
typedef struct {
    uint16_t ia;      /* Phase A current, raw 12-bit                        */
    uint16_t ib;      /* Phase B current, raw 12-bit                        */
    uint16_t vol_u;   /* Phase U voltage, raw 12-bit                        */
    uint16_t vol_v;   /* Phase V voltage, raw 12-bit                        */
    uint16_t vol_w;   /* Phase W voltage, raw 12-bit                        */
    uint16_t temp;    /* Motor temperature (NTC divider), raw 12-bit        */
    uint16_t vbus;    /* Bus voltage (divider), raw 12-bit                  */
} adc_values_t;

/* ======================================================================== */
/* Initialization & Control                                                  */
/* ======================================================================== */
/*! Initialize ADC1 (calibration, channels, DMA, EOS interrupt) and arm it
 *  for external triggering by TIM1. Does not start the PWM timer. */
void adc_init(void);

/*! Arm ADC for synchronized sampling (called after pwm_init/pwm_enable).
 *  Conversions are triggered automatically by TIM1 update events. */
void adc_start_sync(void);

/*! Stop synchronized sampling. */
void adc_stop_sync(void);

/* ======================================================================== */
/* Offset Calibration (current sensing zero-current trim)                    */
/* ======================================================================== */
/*! Measure the zero-current ADC offsets for phases A and B. Must be called
 *  with the motor drive disabled (no phase current). Averages ADC_CAL_SAMPLES
 *  readings. Returns 0 on success, -1 if the ADC is not ready. */
#define ADC_CAL_SAMPLES    64U
int8_t adc_calibrate_offsets(void);

/*! Return the measured zero-current offset for phase A (raw 12-bit). */
uint16_t adc_get_offset_a(void);
/*! Return the measured zero-current offset for phase B (raw 12-bit). */
uint16_t adc_get_offset_b(void);

/* ======================================================================== */
/* Data Accessors                                                            */
/* ======================================================================== */
/*! Snapshot all ADC channels atomically into \p out. */
void adc_get_values(adc_values_t *out);

/*! Phase A current, offset-corrected (raw - offset). Signed. */
int16_t adc_get_current_a(void);
/*! Phase B current, offset-corrected (raw - offset). Signed. */
int16_t adc_get_current_b(void);

/*! Bus voltage raw 12-bit sample. */
uint16_t adc_get_vbus(void);

/*! Phase voltage raw 12-bit sample. phase: 0=U, 1=V, 2=W. */
uint16_t adc_get_phase_voltage(uint8_t phase);

/*! Latest raw DMA sample for a given index (ADC_IDX_*). ISR-safe single read. */
uint16_t adc_get_raw(uint8_t index);

/* ======================================================================== */
/* End-of-Conversion Callback (weak)                                         */
/* ======================================================================== */
/*! Called from ADC1_IRQHandler once a complete 7-channel scan is in the DMA
 *  buffer. The FOC current loop hooks in here. The default weak impl is empty. */
void adc_conv_complete_callback(void) __attribute__((weak));

#ifdef __cplusplus
}
#endif

#endif /* ADC_H */