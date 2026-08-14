/*
 * app.h - Application Layer
 *
 * Top-level controller sitting above the motor PWM backend (FOC/BLDC) and
 * below the user interfaces (UART, PPM, sound). Owns the run-state machine,
 * control-mode selection, setpoint ramping and safety supervision.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef APP_H
#define APP_H

#include "conf/datatypes.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* Run-State Machine                                                         */
/* ======================================================================== */
typedef enum {
    APP_STATE_INIT = 0,     /* one-shot startup / configuration bring-up   */
    APP_STATE_IDLE,         /* motor stopped, ready to take a setpoint     */
    APP_STATE_RUNNING,      /* motor driving in the selected control mode  */
    APP_STATE_FAULT,        /* safety trip: motor disabled until cleared   */
} app_state_t;

/* ======================================================================== */
/* Lifecycle                                                                 */
/* ======================================================================== */
/* Initialise the application layer against the live configuration instances.
 * Call once after conf_general_init(). */
void app_init(mc_configuration_t *mc_conf, app_configuration_t *app_conf);

/* Main-loop worker. Runs safety checks, the state machine, setpoint ramps
 * and services UART/PPM/sound. Call from the main loop as fast as practical. */
void app_process(void);

/* ======================================================================== */
/* Control Setpoints                                                         */
/* ======================================================================== */
void app_set_duty(float duty);          /* MOTOR_MODE_DUTY                 */
void app_set_current(float current);    /* MOTOR_MODE_CURRENT              */
void app_set_speed(float speed);        /* MOTOR_MODE_SPEED (ERPM)         */
void app_set_position(float pos);       /* MOTOR_MODE_POSITION (rad)       */
void app_set_brake(float brake);        /* MOTOR_MODE_CURRENT_BRAKE (A)    */

/* Disable motor drive and return to IDLE. */
void app_stop(void);

/* ======================================================================== */
/* State Access                                                              */
/* ======================================================================== */
/* Live real-time motor state (currents, voltages, speed, temps, fault...). */
motor_state_t *app_get_state(void);

/* Current run-state (INIT / IDLE / RUNNING / FAULT). */
app_state_t app_get_run_state(void);

/* ======================================================================== */
/* Sound                                                                     */
/* ======================================================================== */
/* Queue a single tone played through the motor coils (VESC-style). */
void app_sound_play(uint16_t freq, uint16_t duration_ms);

/* Play the boot melody. Implemented in app_sound.c. */
void app_sound_startup(void);

/* ======================================================================== */
/* Sub-Module Wiring (implemented in app_uart.c / app_ppm.c / app_sound.c)   */
/* ======================================================================== */
void app_uart_init(void);       /* install packet handlers on the UART     */
void app_uart_process(void);    /* periodic UART housekeeping (watchdog)    */
void app_ppm_init(void);       /* configure PPM input capture               */
void app_ppm_process(void);    /* decode PPM, update setpoints, failsafe    */
void app_sound_init(void);     /* reset the tone queue                      */
void app_sound_process(void);   /* advance any active melody/tone            */

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
