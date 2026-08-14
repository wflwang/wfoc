/*
 * hwconf.h - Hardware Configuration Header
 *
 * Selects the appropriate board configuration based on compile-time
 * HW target. Similar to VESC's hwconf system.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef HWCONF_H
#define HWCONF_H

#include "fw_version.h"

/* ======================================================================== */
/* Hardware Target Selection                                                */
/* ======================================================================== */

#if defined(HW_WFoc_V1)
  #include "board_config.h"
  #define HW_NAME "WFOC_V1"
#elif defined(HW_WFoc_V2)
  #error "WFOC V2 not yet defined"
#else
  /* Default to V1 */
  #define HW_WFoc_V1
  #include "board_config.h"
  #define HW_NAME "WFOC_V1"
#endif

/* ======================================================================== */
/* Common Hardware Macros                                                   */
/* ======================================================================== */

/* PWM frequency range */
#ifndef PWM_FREQ_HZ
  #define PWM_FREQ_HZ       20000
#endif

/* FOC loop frequency = PWM frequency */
#define FOC_LOOP_FREQ_HZ    PWM_FREQ_HZ
#define FOC_LOOP_PERIOD_US  (1000000 / FOC_LOOP_FREQ_HZ)

/* ADC sample time for FOC loop */
#define ADC_SAMPLES_PER_CYCLE   1

/* ======================================================================== */
/* Safety Limits                                                            */
/* ======================================================================== */
#define HW_MAX_CURRENT         80.0f   /* Hardware max current (A) */
#define HW_MAX_TEMP_MOTOR      100.0f  /* Max motor temp (°C) */
#define HW_MAX_TEMP_MOSFET     90.0f   /* Max MOSFET temp (°C) */

/* ======================================================================== */
/* Motor Phase Mapping                                                      */
/* ======================================================================== */
/* Maps hardware phases to motor phases.
 * Adjust if motor wires are swapped.
 */
#define HW_PHASE_A             0
#define HW_PHASE_B             1
#define HW_PHASE_C             2

#endif /* HWCONF_H */
