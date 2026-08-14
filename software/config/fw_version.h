/*
 * fw_version.h - Firmware version definitions
 *
 * Similar to VESC, different firmware versions can be compiled
 * with different hardware configurations via compile-time defines.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef FW_VERSION_H
#define FW_VERSION_H

/* ======================================================================== */
/* Firmware Version                                                          */
/* ======================================================================== */
#define FW_VERSION_MAJOR    1
#define FW_VERSION_MINOR    0
#define FW_VERSION_PATCH    0

/* ======================================================================== */
/* Hardware Target Selection (compile-time)                                 */
/* ======================================================================== */
/* Select one hardware target at compile time:
 *   make HW=WFoc_V1
 *   make HW=WFoc_V2
 *
 * Each target defines its own board_config.h include path
 */

#ifndef HW
#define HW WFoc_V1
#endif

/* ======================================================================== */
/* Feature Flags (compile-time enable/disable)                              */
/* ======================================================================== */

/* Motor control modes */
#define ENABLE_FOC          1   /* FOC mode */
#define ENABLE_BLDC         1   /* BLDC six-step mode */

/* Observers */
#define ENABLE_SMO          1   /* Sliding Mode Observer */
#define ENABLE_MXLEMMA      1   /* MXLEMMA Observer */
#define ENABLE_HFI          1   /* High-Frequency Injection */
#define ENABLE_MXV_LAMBDA   1   /* MXV Lambda compensation */

/* FOC features */
#define ENABLE_SVPWM        1   /* Space Vector PWM */
#define ENABLE_OVERMOD      1   /* Overmodulation */
#define ENABLE_FW           1   /* Field Weakening */
#define ENABLE_NOTCH        1   /* Notch Filter */
#define ENABLE_DECOUPLE     1   /* D-Q axis decoupling */

/* BLDC features */
#define ENABLE_BEMF_COMP    1   /* Comparator-based BEMF */
#define ENABLE_BEMF_ADC     1   /* ADC-based BEMF */
#define ENABLE_BEMF_ZCD     1   /* Zero-crossing detection */

/* App interfaces */
#define ENABLE_APP_UART     1   /* UART control */
#define ENABLE_APP_PPM      1   /* PPM control */
#define ENABLE_APP_DUTY     1   /* Duty control */
#define ENABLE_APP_SPEED    1   /* Speed control */
#define ENABLE_APP_POS      1   /* Position control */
#define ENABLE_APP_DSHORT   1   /* Dshort control */
#define ENABLE_APP_SOUND    1   /* Motor sound generation */

/* Auto-tuning */
#define ENABLE_AUTOTUNE     1   /* Parameter auto-tuning */
#define ENABLE_DETECT       1   /* Motor parameter detection */

/* Communication */
#define ENABLE_PACKET       1   /* Packet-based communication */
#define ENABLE_TERMINAL     1   /* Terminal commands */

/* ======================================================================== */
/* Derived Version String                                                    */
/* ======================================================================== */
#define FW_VERSION_STRING   XSTR(FW_VERSION_MAJOR) "." \
                            XSTR(FW_VERSION_MINOR) "." \
                            XSTR(FW_VERSION_PATCH)

#define XSTR(s) STR(s)
#define STR(s) #s

#endif /* FW_VERSION_H */
