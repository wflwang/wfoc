/*
 * conf_default.c - Default Configuration Values
 *
 * Safe, conservative defaults for mc_configuration_t and app_configuration_t
 * derived from the WFOC V1 hardware: 10-48 V bus, 0.005 R shunt, 20 A
 * continuous / 60 A peak current, 20 kHz PWM on the CIU32F003x5.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include "conf/conf_general.h"
#include "config/hwconf.h"
#include "config/board_config.h"

/* ======================================================================== */
/* Default Motor-Controller Configuration                                    */
/* ======================================================================== */
void conf_default_mc(mc_configuration_t *c)
{
    /* --- Motor parameters (generic small BLDC, tune via detection) --- */
    c->motor_r            = 0.08f;       /* phase resistance (ohm)        */
    c->motor_l            = 0.00012f;    /* phase inductance (H, ~120 uH)  */
    c->motor_flux_linkage = 0.005f;      /* flux linkage (Wb)              */
    c->motor_poles        = 7U;          /* 14-pole motor                  */

    /* --- Temperature compensation coefficients --- */
    c->motor_temp_corr_r  = 0.0039f;     /* copper R vs T (1/degC)         */
    c->motor_temp_corr_l  = 0.0010f;     /* iron L vs T (1/degC)           */
    c->motor_temp_ref     = 25.0f;       /* reference T (degC)             */
    c->foc_temp_comp_en   = 1U;          /* enable temp compensation       */

    c->comm_mode          = (uint8_t)COMM_MODE_FOC;  /* FOC by default     */

    /* --- FOC current loop (Q-axis) ---
     * ts = 1/20000 = 50 us.  KI value is *per sample*, so effective
     * Ki = foc_current_ki * ts = 2000 * 50e-6 = 0.1 A/V.s           */
    c->foc_current_kp     = 0.02f;       /* V per A, d/q axis              */
    c->foc_current_ki     = 2000.0f;     /* V/A per sample (integral)     */
    c->foc_current_filter = 0.20f;       /* LPF alpha on current feedback */

    /* --- Speed loop ---
     * Speed loop runs at 1 kHz (called from main loop or slower ISR). */
    c->foc_speed_kp       = 0.002f;      /* A per ERPM                    */
    c->foc_speed_ki       = 50.0f;       /* A/ERPM per 1ms sample         */
    c->foc_speed_ff       = 0.001f;      /* A per ERPM feedforward        */
    c->foc_speed_ff_en    = 1U;          /* enable speed feedforward      */

    /* --- Legacy duty loop --- */
    c->foc_duty_kp        = 0.01f;
    c->foc_duty_ki        = 100.0f;

    /* --- Decoupling / Feedforward (all enabled by default) --- */
    c->foc_decouple_en    = 1U;          /* enable omega*L coupling       */
    c->foc_bemf_ff_en     = 1U;          /* enable omega*psi BEMF FF      */

    /* --- Low-speed / standstill ---
     * Below 600 ERPM the observer BEMF is very small; increase the
     * current-loop gains and PLL gains to stay locked.               */
    c->foc_low_speed_thr  = 600.0f;      /* ERPM                          */
    c->foc_low_speed_boost = 3.0f;       /* 3x gain at standstill         */
    c->foc_low_speed_hyst = 150.0f;      /* ERPM hysteresis band          */
    c->foc_hfi_speed_thr  = 200.0f;      /* switch to HFI below this ERPM */

    /* --- Speed-dependent BEMF feedforward scaling ---
     * The observer's omega*psi estimate is unreliable below ~300 ERPM
     * because the back-EMF amplitude is comparable to ADC noise.
     * Below foc_bemf_comp_thr the BEMF FF is blended from
     * bemf_comp_boost (~0.2) up to 1.0 at foc_bemf_comp_full.     */
    c->foc_bemf_comp_thr   = 300.0f;     /* ERPM below which scale-down   */
    c->foc_bemf_comp_full  = 1200.0f;    /* ERPM for full compensation   */
    c->foc_bemf_comp_boost = 0.2f;       /* scale factor at standstill   */

    /* --- Startup current ramp ---
     * At start-up the observer is still converging; a large iq_step
     * causes a violent kick.  Ramp iq_set from the current value to
     * the target over foc_start_ramp_ms milliseconds when:
     *   |ERPM| < foc_start_speed_thr AND |delta_iq| > foc_start_step_current. */
    c->foc_start_ramp_ms      = 80.0f;   /* ms total ramp time           */
    c->foc_start_step_current = 0.5f;    /* A step that triggers ramp   */
    c->foc_start_speed_thr    = 500.0f;  /* ERPM below which ramp active */

    /* --- Field weakening (enabled by default) ---
     * FW injects negative d-axis current when voltage margin is low */
    c->foc_fw_en              = 1U;     /* Enable FW                     */
    c->foc_fw_current_max     = 3.0f;   /* Max FW d-axis current (A)     */
    c->foc_fw_duty_start      = 0.85f;  /* Duty threshold to start FW   */
    c->foc_fw_ramp_time       = 0.20f;  /* FW ramp time (s)             */
    c->foc_fw_kp              = 0.5f;   /* FW PI KP                      */
    c->foc_fw_ki              = 10.0f;  /* FW PI KI                     */
    c->foc_fw_voltage_margin  = 1.0f;   /* Voltage margin (V)           */

    /* --- Position loop (disabled by default) ---
     * Position loop -> Speed loop -> Current loop */
    c->foc_pos_en             = 0U;      /* Enable position loop         */
    c->foc_pos_kp             = 500.0f; /* Position KP (ERPM/rad)       */
    c->foc_pos_ki             = 50.0f;  /* Position KI                  */
    c->foc_pos_ff             = 100.0f; /* Position feedforward         */
    c->foc_pos_max_speed      = 5000.0f;/* Max speed from pos loop (ERPM) */

    /* --- Overmodulation --- */
    c->foc_overmod_en         = 0U;     /* Disabled by default          */
    c->foc_overmod_factor     = 0.0f;   /* Overmodulation factor 0-1    */

    /* --- Notch filter --- */
    c->foc_notch_en       = 0U;
    c->foc_notch_freq     = 1000.0f;
    c->foc_notch_bw       = 100.0f;

    /* --- Observer --- */
    c->observer_type      = (uint8_t)OBSERVER_SMO;
    c->observer_gain      = 0.0004f;
    c->observer_pll_kp    = 2000.0f;     /* PLL KP at normal speed         */
    c->observer_pll_ki    = 20000.0f;    /* PLL KI at normal speed         */
    c->observer_pll_kp_low = 4000.0f;    /* PLL KP at low speed            */
    c->observer_pll_ki_low = 40000.0f;   /* PLL KI at low speed            */
    c->observer_speed_boost = 2.0f;      /* gain multiplier at standstill  */

    /* --- HFI --- */
    c->hfi_amplitude      = 4.0f;        /* volts                          */
    c->hfi_freq           = 1000.0f;     /* Hz                             */

    /* --- Limits (from board_config.h) --- */
    c->l_max_duty        = 0.95f;
    c->l_min_duty        = 0.05f;
    c->l_max_speed       = 50000.0f;     /* ERPM                           */
    c->l_min_speed       = -50000.0f;
    c->l_max_current     = CURRENT_CONT_A;        /* 20 A continuous       */
    c->l_min_current     = -CURRENT_CONT_A;       /* 20 A brake            */
    c->l_in_current_max  = CURRENT_CONT_A * 1.5f; /* 30 A input peak       */
    c->l_in_current_min  = -CURRENT_CONT_A * 1.5f;
    c->l_watt_max        = 1000.0f;      /* W                              */
    c->l_watt_min        = -1000.0f;
    c->l_temp_motor_max  = 80.0f;        /* deg C                          */
    c->l_temp_mosfet_max = 90.0f;        /* deg C                          */

    /* --- PWM --- */
    c->pwm_freq           = PWM_FREQ_HZ;        /* 20000 Hz                */
    c->deadtime           = DEADTIME_NS;        /* 500 ns                  */

    /* --- BLDC --- */
    c->bldc_bemf_mode     = (uint8_t)BEMF_MODE_ADC;
    c->bldc_bemf_k        = 0.5f;
    c->bldc_comm_time    = 0.0002f;      /* 200 us comm advance window    */

    /* --- Sound --- */
    c->sound_mode         = 0U;
    c->sound_freq         = 1000U;
    c->sound_time         = 100U;
}

/* ======================================================================== */
/* Default App Configuration                                                 */
/* ======================================================================== */
void conf_default_app(app_configuration_t *c)
{
    c->app_mode  = (uint8_t)MOTOR_MODE_CURRENT;
    c->ppm_max   = 2.0f;                /* ms                              */
    c->ppm_min   = 1.0f;                /* ms                              */
    c->ppm_ramp  = 0.04f;               /* ramp rate (1/sec)              */
    c->uart_baud = UART_BAUDRATE;        /* 115200                         */

    /* --- SPI Configuration ---
     * SPI is enabled by default for future remote control/OTA expansion */
    c->spi_enabled = 1U;                /* Enable SPI interface           */
    c->spi_mode = 0U;                   /* SPI Mode 0 (CPOL=0, CPHA=0)    */
    c->spi_clk_div = 1U;                /* Clock divider = 4 (6 MHz)      */
    c->spi_csn_polarity = 0U;           /* CSN active low                 */

    /* --- OTA Configuration ---
     * OTA is enabled, using UART by default */
    c->ota_enabled = 1U;                /* Enable OTA functionality        */
    c->ota_channel = 0U;                /* Use UART for OTA (default)      */
    c->ota_timeout_ms = 1000U;          /* 1 second packet timeout        */
    c->ota_auto_restart = 1U;           /* Auto restart after successful OTA */
}
