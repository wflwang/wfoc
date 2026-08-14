/*
 * datatypes.h - Core data type definitions and configuration structures
 *
 * Defines all the enums, structures, and type definitions used throughout
 * the WFOC firmware for motor control, configuration, state reporting,
 * and UART communication. Modeled after the VESC configuration system.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef DATATYPES_H
#define DATATYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "util/fixedpoint.h"

/* ======================================================================== */
/* Motor Control Modes                                                       */
/* ======================================================================== */
typedef enum {
    MOTOR_MODE_DISABLED = 0,
    MOTOR_MODE_DUTY,
    MOTOR_MODE_SPEED,
    MOTOR_MODE_POSITION,
    MOTOR_MODE_CURRENT,
    MOTOR_MODE_CURRENT_BRAKE,
    MOTOR_MODE_HAND_TEST,
} motor_mode_t;

/* ======================================================================== */
/* Commutation Modes                                                         */
/* ======================================================================== */
typedef enum {
    COMM_MODE_FOC = 0,
    COMM_MODE_BLDC,
} comm_mode_t;

/* ======================================================================== */
/* Sensor Modes                                                              */
/* ======================================================================== */
typedef enum {
    SENSOR_MODE_SENSORLESS = 0,
    SENSOR_MODE_HALL,
    SENSOR_MODE_ENCODER,
    SENSOR_MODE_HFI,
} sensor_mode_t;

/* ======================================================================== */
/* Observer Types                                                            */
/* ======================================================================== */
typedef enum {
    OBSERVER_NONE = 0,
    OBSERVER_SMO,        /* Sliding Mode Observer */
    OBSERVER_MXLEMMA,    /* MXLEMMA Observer */
    OBSERVER_MXV_LAMBDA, /* MXV Lambda Compensation */
    OBSERVER_HFI,        /* High Frequency Injection */
} observer_type_t;

/* ======================================================================== */
/* BLDC BEMF Detection Methods                                               */
/* ======================================================================== */
typedef enum {
    BEMF_MODE_ADC = 0,       /* ADC-based BEMF sensing */
    BEMF_MODE_COMPARATOR,    /* Comparator-based BEMF sensing */
    BEMF_MODE_ZCD,           /* Zero-crossing detection */
} bldc_bemf_mode_t;

/* ======================================================================== */
/* Fault Codes                                                               */
/* ======================================================================== */
typedef enum {
    FAULT_NONE = 0,
    FAULT_OVER_VOLTAGE,
    FAULT_UNDER_VOLTAGE,
    FAULT_OVER_CURRENT,
    FAULT_OVER_TEMP_MOTOR,
    FAULT_OVER_TEMP_MOSFET,
    FAULT_PHASE_LOSS,
    FAULT_DRV_FAULT,
    FAULT_BOOTSTRAP_FAULT,
} fault_code_t;

/* ======================================================================== */
/* FOC Configuration (motor controller configuration)                        */
/* ======================================================================== */
typedef struct {
    /* Motor parameters */
    float motor_r;            /* Phase resistance (ohm) */
    float motor_l;            /* Phase inductance (H) */
    float motor_flux_linkage; /* Flux linkage (Wb) */
    uint8_t motor_poles;      /* Number of pole pairs */

    /* Temperature compensation coefficients
     * R(T) = R0 * (1 + temp_corr_r * (T - T_ref))
     * L(T) = L0 * (1 + temp_corr_l * (T - T_ref))
     * Typical copper: temp_corr_r ~= 0.0039 / degC
     * Typical iron:   temp_corr_l ~= 0.0010 / degC                         */
    float motor_temp_corr_r;  /* R temperature coefficient (1/degC)     */
    float motor_temp_corr_l;  /* L temperature coefficient (1/degC)     */
    float motor_temp_ref;     /* Reference temp for R/L (degC)          */

    /* Commutation mode: FOC or BLDC six-step */
    uint8_t comm_mode;        /* comm_mode_t */

    /* ------- Current loop (d/q axis PI) -------
     * vd = PI(id_ref - id) - omega*L*iq        (d-axis decoupling)
     * vq = PI(iq_ref - iq) + omega*L*id + omega*psi  (q-axis decoupling + BEMF FF)
     * Current loop runs at PWM rate (20 kHz). ki already includes Ts. */
    float foc_current_kp;     /* Current loop KP (same for d/q) */
    float foc_current_ki;     /* Current loop KI (same for d/q, *Ts) */
    float foc_current_filter; /* Current measurement LPF alpha */

    /* ------- Speed loop (ERPM) -------
     * iq_ref = PI( speed_ref - speed ) * speed_ff_gain * speed_ff_base */
    float foc_speed_kp;       /* Speed loop KP */
    float foc_speed_ki;       /* Speed loop KI (*Ts, 1kHz rate) */
    float foc_speed_ff;       /* Speed feedforward gain (A per ERPM) */

    /* ------- Duty loop (legacy / BLDC open-loop) ------- */
    float foc_duty_kp;
    float foc_duty_ki;

    /* ------- Decoupling / Feedforward ------- */
    uint8_t foc_decouple_en;  /* Enable omega*L cross-coupling */
    uint8_t foc_bemf_ff_en;   /* Enable omega*psi back-EMF feedforward */
    uint8_t foc_speed_ff_en;  /* Enable speed feedforward */
    uint8_t foc_temp_comp_en; /* Enable R/L temperature compensation */

    /* ------- Low-speed / standstill improvements -------
     * When |speed| < low_speed_thr (ERPM), boost current-loop gain
     * by low_speed_boost to fight low-BEMF noise and improve tracking.
     * Switch to HFI if available (encoderless) below hfi_speed_thr.
     *
     * Speed-dependent BEMF feedforward:
     *   Below bemf_comp_thr (ERPM) the observer speed estimate is noisy,
     *   so the omega*psi feedforward is scaled by bemf_comp_boost (<= 1.0)
     *   to avoid injecting false voltage.  Above bemf_comp_full the full
     *   compensation is applied.  Linear ramp between the two thresholds.
     *
     * Startup current ramp:
     *   When iq_set changes by more than startup_step_current while the
     *   motor is still (|ERPM| < start_speed_thr), linearly ramp iq_set
     *   from its current value to the new target over startup_ramp_ms.
     *   This prevents a large "kick" that excites the mechanical system
     *   before the observer is fully locked.                              */
    float foc_low_speed_thr;     /* ERPM below which low-speed active */
    float foc_low_speed_boost;   /* Current-loop gain multiplier at standstill */
    float foc_low_speed_hyst;    /* Hysteresis on low-speed entry/exit */
    float foc_hfi_speed_thr;     /* ERPM below which HFI dominates    */

    float foc_bemf_comp_thr;     /* ERPM below which BEMF FF is scaled down */
    float foc_bemf_comp_full;    /* ERPM above which full BEMF FF is applied */
    float foc_bemf_comp_boost;   /* BEMF FF scale factor at standstill (<= 1.0) */

    float foc_start_ramp_ms;     /* Startup current ramp time (ms)     */
    float foc_start_step_current;/* iq_set step that triggers ramp (A) */
    float foc_start_speed_thr;   /* ERPM below which startup still active */

    /* Field weakening (extended)
     * When the required voltage exceeds the available bus voltage,
     * a negative d-axis current is injected to weaken the rotor flux
     * and extend the speed range.                                  */
    uint8_t foc_fw_en;          /* Enable field weakening */
    float foc_fw_current_max;   /* Max FW d-axis current (A) */
    float foc_fw_duty_start;    /* Duty cycle threshold to start FW */
    float foc_fw_ramp_time;     /* FW ramp-up time (s) */
    float foc_fw_kp;            /* FW PI proportional gain */
    float foc_fw_ki;            /* FW PI integral gain */
    float foc_fw_voltage_margin; /* Voltage margin for FW (V) */

    /* Position loop (outermost loop, runs at 100 Hz)
     * Position loop -> Speed loop -> Current loop -> Voltage */
    float foc_pos_kp;           /* Position loop KP (ERPM per rad) */
    float foc_pos_ki;           /* Position loop KI */
    float foc_pos_ff;           /* Position feedforward gain */
    float foc_pos_max_speed;    /* Max speed from position loop (ERPM) */
    uint8_t foc_pos_en;         /* Enable position loop */

    /* Overmodulation */
    uint8_t foc_overmod_en;     /* Enable overmodulation */
    float foc_overmod_factor;   /* Overmodulation factor 0-1 */

    /* Notch filter */
    uint8_t foc_notch_en;     /* Enable notch filter */
    float foc_notch_freq;     /* Notch frequency */
    float foc_notch_bw;       /* Notch bandwidth */

    /* Observer */
    uint8_t observer_type;    /* observer_type_t */
    float observer_gain;      /* Observer gain */
    float observer_pll_kp;    /* PLL KP */
    float observer_pll_ki;    /* PLL KI */
    float observer_pll_kp_low;/* PLL KP at low speed (higher for locking) */
    float observer_pll_ki_low;/* PLL KI at low speed */
    float observer_speed_boost; /* PLL gain boost factor at standstill */

    /* HFI */
    float hfi_amplitude;      /* HFI voltage amplitude */
    float hfi_freq;           /* HFI frequency */

    /* Speed limits */
    float l_max_duty;         /* Max duty cycle */
    float l_min_duty;         /* Min duty cycle */
    float l_max_speed;        /* Max speed (ERPM) */
    float l_min_speed;        /* Min speed (ERPM) */
    float l_max_current;      /* Max current (A) */
    float l_min_current;      /* Min current (brake, A) */
    float l_in_current_max;   /* Max input current */
    float l_in_current_min;   /* Min input current */
    float l_watt_max;         /* Max power */
    float l_watt_min;         /* Min power */
    float l_temp_motor_max;   /* Max motor temp */
    float l_temp_mosfet_max;  /* Max MOSFET temp */

    /* PWM */
    uint16_t pwm_freq;        /* PWM frequency (Hz) */
    uint16_t deadtime;        /* Dead time (ns) */

    /* BLDC parameters */
    uint8_t bldc_bemf_mode;   /* bldc_bemf_mode_t */
    float bldc_bemf_k;        /* BEMF threshold */
    float bldc_comm_time;     /* Commutation timing */

    /* Sound */
    uint8_t sound_mode;       /* Sound mode */
    uint16_t sound_freq;      /* Sound frequency */
    uint16_t sound_time;      /* Sound duration */
} mc_configuration_t;

/* ======================================================================== */
/* App Configuration                                                         */
/* ======================================================================== */
typedef struct {
    uint8_t app_mode;         /* APP mode */
    float ppm_max;            /* PPM max pulse */
    float ppm_min;            /* PPM min pulse */
    float ppm_ramp;           /* PPM ramp */
    uint32_t uart_baud;       /* UART baudrate */

    /* ------- SPI Configuration -------
     * SPI interface for future remote control and OTA
     * Half-duplex: MOSI and MISO are connected externally */
    uint8_t spi_enabled;      /* Enable SPI interface */
    uint8_t spi_mode;         /* SPI mode (SPI_MODE_0-3) */
    uint8_t spi_clk_div;      /* SPI clock divider (SPI_CLK_DIV_*) */
    uint8_t spi_csn_polarity; /* CSN active level (0=low, 1=high) */

    /* ------- OTA Configuration -------
     * OTA upgrade settings */
    uint8_t ota_enabled;      /* Enable OTA functionality */
    uint8_t ota_channel;      /* OTA communication channel */
                              /* 0 = UART, 1 = SPI, 2 = both */
    uint16_t ota_timeout_ms;  /* OTA packet timeout */
    uint8_t ota_auto_restart; /* Auto restart after OTA */
} app_configuration_t;

/* ======================================================================== */
/* Motor State (real-time values)                                            */
/* ======================================================================== */
typedef struct {
    float ia;                 /* Phase A current */
    float ib;                 /* Phase B current */
    float ic;                 /* Phase C current */
    float va;                 /* Phase A voltage (V) */
    float vb;                 /* Phase B voltage (V) */
    float vc;                 /* Phase C voltage (V) */
    float id;                 /* D-axis current */
    float iq;                 /* Q-axis current */
    float vd;                 /* D-axis voltage */
    float vq;                 /* Q-axis voltage */
    float duty;               /* Duty cycle */
    float speed;              /* Speed (ERPM) */
    float position;           /* Rotor angle (rad) */
    float vbus;               /* Bus voltage */
    float temp_motor;         /* Motor temp */
    float temp_mosfet;        /* MOSFET temp */
    float power;              /* Power (W) */
    float energy;             /* Energy (Wh) */
    float ah;                 /* Amp-hours */
    uint8_t fault;            /* Fault code */
} motor_state_t;

/* ======================================================================== */
/* VESC-like Comm Packet Commands (for UART tuning)                          */
/* ======================================================================== */
typedef enum {
    COMM_FW_VERSION = 0,
    COMM_JUMP_TO_BOOTLOADER,
    COMM_GET_VALUES,
    COMM_SET_DUTY,
    COMM_SET_CURRENT,
    COMM_SET_CURRENT_BRAKE,
    COMM_SET_RPM,
    COMM_SET_POS,
    COMM_SET_DETECT,
    COMM_REBOOT,
    COMM_ALIVE,
    COMM_GET_MCCONF,
    COMM_SET_MCCONF,
    COMM_GET_APPCONF,
    COMM_SET_APPCONF,
    COMM_DETECT_MOTOR_PARAM,
    COMM_DETECT_ENCODER,
    COMM_DETECT_HALL,
    COMM_TERMINAL_CMD,
    COMM_GET_VALUES_SELECTIVE,  /* High-speed selective telemetry (bitmask) */
    /* ... more */
} comm_packet_t;

/* ======================================================================== */
/* Telemetry Field Bitmask (for COMM_GET_VALUES_SELECTIVE)                   */
/* ======================================================================== */
/* Upper-layer tool sets these bits in a 32-bit mask to request only the
 * fields it needs, minimising packet size for high-rate streaming.        */
#define TELEMETRY_MASK_IA       (1U << 0)
#define TELEMETRY_MASK_IB       (1U << 1)
#define TELEMETRY_MASK_IC       (1U << 2)
#define TELEMETRY_MASK_VA       (1U << 3)
#define TELEMETRY_MASK_VB       (1U << 4)
#define TELEMETRY_MASK_VC       (1U << 5)
#define TELEMETRY_MASK_ID       (1U << 6)
#define TELEMETRY_MASK_IQ       (1U << 7)
#define TELEMETRY_MASK_VD       (1U << 8)
#define TELEMETRY_MASK_VQ       (1U << 9)
#define TELEMETRY_MASK_DUTY     (1U << 10)
#define TELEMETRY_MASK_SPEED    (1U << 11)
#define TELEMETRY_MASK_POSITION (1U << 12)
#define TELEMETRY_MASK_VBUS     (1U << 13)
#define TELEMETRY_MASK_TEMP_MOS (1U << 14)
#define TELEMETRY_MASK_TEMP_MOT (1U << 15)
#define TELEMETRY_MASK_POWER    (1U << 16)

/* ======================================================================== */
/* FOC State (real-time fixed-point internal state)                          */
/* ======================================================================== */
typedef struct {
    /* ------- Measured / transformed quantities ------- */
    q16_t       ialpha;         /* Alpha-axis current (Q16.16 A)              */
    q16_t       ibeta;          /* Beta-axis current  (Q16.16 A)              */
    q16_t       id;             /* D-axis current (Q16.16 A)                  */
    q16_t       iq;             /* Q-axis current (Q16.16 A)                  */

    /* ------- Reference setpoints ------- */
    q16_t       id_set;         /* D-axis current setpoint (Q16.16 A)         */
    q16_t       iq_set;         /* Q-axis current setpoint (Q16.16 A)         */
    q16_t       speed_set;      /* Speed setpoint (Q16.16 ERPM)              */

    /* ------- Final voltage commands (after PI + FF + decoupling) ------- */
    q16_t       vd;             /* D-axis voltage command (Q16.16 V)          */
    q16_t       vq;             /* Q-axis voltage command (Q16.16 V)          */
    q16_t       valpha;         /* Alpha-axis voltage (Q16.16 V)              */
    q16_t       vbeta;          /* Beta-axis voltage  (Q16.16 V)              */

    /* ------- Electrical angle and speed ------- */
    uint16_t    angle;          /* Rotor electrical angle (0-65535 = 0-360)   */
    q16_t       speed_rad;      /* Electrical speed (Q16.16 rad/s) - from obs*/
    q16_t       speed_erpm;     /* Mechanical speed (Q16.16 ERPM)             */

    /* ------- PI / PID controllers ------- */
    q16_pi_t    pid_id;         /* D-axis current PI controller               */
    q16_pi_t    pid_iq;         /* Q-axis current PI controller               */
    q16_pi_t    pid_speed;      /* Speed PI controller (ERPM -> Iq)          */

    /* ------- Low-pass filters ------- */
    q16_lpf_t   lpf_id;         /* D-axis current low-pass filter             */
    q16_lpf_t   lpf_iq;         /* Q-axis current low-pass filter             */
    q16_lpf_t   lpf_speed;      /* Speed low-pass filter                     */
    q16_lpf_t   lpf_temp;       /* Temperature low-pass filter               */

    /* ------- Decoupling / feedforward (Q16.16, ISR-ready) -------
     * These are precomputed per-ISR to keep inner loop cheap.              */
    q16_t       omega_l;        /* omega * L (Q16.16 V/A)                    */
    q16_t       omega_psi;      /* omega * psi (Q16.16 V)  - BEMF magnitude  */
    q16_t       omega_l_id;     /* omega * L * id  (d-axis coupling, V)      */
    q16_t       omega_l_iq;     /* omega * L * iq  (q-axis coupling, V)      */

    /* ------- Temperature-compensated motor parameters (Q16.16) -------
     * Updated at a slower rate (main loop ~100 Hz).                         */
    q16_t       r_q16;          /* Temperature-compensated R (ohm)           */
    q16_t       l_q16;          /* Temperature-compensated L (H)             */
    q16_t       psi_q16;        /* Temperature-compensated flux linkage (Wb)*/

    /* ------- Low-speed gain scheduling -------
     * cur_kp_scale and cur_ki_scale are blended from 1.0 to boost value
     * as speed falls below foc_low_speed_thr, with hysteresis.               */
    q16_t       cur_kp_scale;   /* Current Kp scale (Q16.16, >= 1.0)         */
    q16_t       cur_ki_scale;   /* Current Ki scale (Q16.16, >= 1.0)         */
    uint8_t     low_speed_active; /* 1 = in low-speed regime                 */

    /* Precomputed ISR constants for low-speed scheduler (Q16.16).
     * Set once in foc_init() to avoid FLOAT_TO_Q16 calls in the ISR.      */
    q16_t       ls_thr_q16;     /* low-speed ERPM threshold                 */
    q16_t       ls_hyst_q16;    /* low-speed hysteresis (ERPM)             */
    q16_t       ls_boost_q16;   /* low-speed boost multiplier              */
    q16_t       ls_alpha_q16;   /* LPF alpha for gain blending              */

    /* ------- Speed feedforward tracking ------- */
    q16_t       iq_ff;          /* Feedforward Iq from speed profile (Q16.16)*/

    /* ------- BEMF feedforward scaling (speed-dependent, ISR-safe) -------
     * The observer-estimated speed is unreliable at low |ERPM| (low BEMF),
     * so the omega*psi feedforward is scaled by bemf_scale_q16 which is
     * pre-blended by foc_low_speed_update().  At standstill the scale
     * approaches bemf_comp_boost (typically 0), rising linearly to 1.0
     * above bemf_comp_full ERPM.                                     */
    q16_t       bemf_scale_q16;   /* BEMF FF scale 0..1 (Q16.16)           */
    q16_t       bemf_comp_thr_q16;
    q16_t       bemf_comp_full_q16;
    q16_t       bemf_comp_boost_q16;

    /* ------- Startup current ramp (1 kHz updates, ISR-safe) -------
     * When a large step in iq_set is commanded while the motor is still
     * nearly stopped, we linearly ramp iq_set toward the target over
     * foc_start_ramp_ms milliseconds.  This avoids a heavy "kick" that
     * makes the motor turn unevenly at start-up.                        */
    q16_t       start_iq_target;   /* Desired final Iq (Q16.16 A)          */
    q16_t       start_iq_current;  /* Current ramped Iq (Q16.16 A)         */
    q16_t       start_iq_step_q16; /* Per-1kHz-step increment (Q16.16)     */
    uint16_t    start_ramp_ticks;  /* Remaining 1kHz ticks in ramp         */
    uint8_t     start_ramp_active; /* 1 = ramp in progress                 */

    /* Precomputed startup ramp constants (Q16.16) */
    q16_t       start_ramp_ticks_max_q16; /* max ramp ticks (for div)       */
    q16_t       start_speed_thr_q16;      /* start |ERPM| threshold         */
    q16_t       start_step_current_q16;    /* step-current trigger (A)       */

    /* ------- Observer / PLL low-speed gains (Q16.16) ------- */
    q16_t       pll_kp_eff;     /* Currently used PLL KP                    */
    q16_t       pll_ki_eff;     /* Currently used PLL KI                    */

    /* ------- Precomputed ISR constants (set once in foc_init) -------
     * These remove FLOAT_TO_Q16 and integer divisions from the ISR,
     * saving several 10-20 cycle operations per PWM cycle.             */
    q16_t       cur_kp_base_q16;   /* foc_current_kp  in Q16.16          */
    q16_t       cur_ki_base_q16;   /* foc_current_ki  in Q16.16          */
    q16_t       pll_kp_normal_q16; /* PLL kp at normal speed             */
    q16_t       pll_ki_normal_q16; /* PLL ki at normal speed             */
    q16_t       pll_kp_low_q16;   /* PLL kp at low speed                */
    q16_t       pll_ki_low_q16;   /* PLL ki at low speed                */
    q16_t       pll_ki_per_step_normal_q16;  /* PLL ki per ISR step normal */
    q16_t       pll_ki_per_step_low_q16;     /* PLL ki per ISR step low  */

    /* Precomputed reciprocals for ISR (Q16.16).
     * Used to replace Q16_DIV with Q16_MUL in the low-speed scheduler,
     * eliminating slow 64-bit divides from the ADC ISR on Cortex-M0+. */
    q16_t       ls_thr_inv_q16;       /* 1 / ls_thr */
    q16_t       bemf_span_inv_q16;    /* 1 / (bemf_full - bemf_thr) */

    /* ------- Position loop state (100 Hz) -------
     * Position outer loop generates speed reference for the speed loop. */
    q16_t       pos_set;          /* Position setpoint (Q16.16 rad)         */
    q16_t       pos_meas;         /* Measured position (Q16.16 rad)         */
    q16_pi_t    pid_position;     /* Position PI controller                 */
    q16_t       pos_ff;           /* Position feedforward                   */
    q16_t       pos_speed_max_q16;/* Max speed from position loop (ERPM)    */
    q16_t       pos_kp_base_q16;  /* Position KP base (precomputed)         */
    q16_t       pos_ki_base_q16;  /* Position KI base (precomputed)         */
    q16_t       pos_ff_base_q16;  /* Position FF base (precomputed)         */

    /* ------- Field weakening state -------
     * Inject negative d-axis current when voltage margin is low. */
    q16_t       fw_id_ref;        /* FW d-axis current reference (Q16.16 A) */
    q16_t       fw_vd_req;        /* Required d-axis voltage for FW (V)      */
    q16_t       fw_vd_avail;      /* Available d-axis voltage (V)            */
    q16_pi_t    pid_fw;           /* FW PI controller                        */
    q16_t       fw_duty_threshold_q16; /* Duty threshold for FW start         */
    q16_t       fw_current_max_q16;  /* Max FW current (Q16.16 A)           */
    uint8_t     fw_active;        /* 1 = FW is active                       */
    q16_t       fw_ramp_alpha_q16; /* FW ramp filter coefficient             */
} foc_state_t;

/* observer_state_t is defined in motor/observer.h (SMO / MXLEMMA / HFI).   */

#endif /* DATATYPES_H */
