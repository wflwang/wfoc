/*
 * mcpwm.h - Motor PWM Backend Bridge
 *
 * Strong overrides for the weak mcpwm_* stubs in app.c.
 * Maps application setpoints to FOC/BLDC state variables consumed
 * by the ADC1 ISR at the PWM rate.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef MCPWM_H
#define MCPWM_H

#include <stdint.h>
#include "conf/datatypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Lifecycle */
void mcpwm_init(mc_configuration_t *conf);
void mcpwm_stop(void);

/* Control setpoints */
void mcpwm_set_duty(float duty);
void mcpwm_set_current(float current);
void mcpwm_set_current_brake(float current);
void mcpwm_set_speed(float speed);
void mcpwm_set_position(float pos);

/* State access */
void mcpwm_get_state(motor_state_t *st);

/* Sound */
void mcpwm_sound_tone(float freq_hz, float amplitude);

#ifdef __cplusplus
}
#endif

#endif /* MCPWM_H */
