/*
 * timer.h - Timer & Watchdog Driver for CIU32F003x5
 *
 * Provides TIM2 CH1 PPM capture (PB4, 1 MHz timer clock = 1 us/tick,
 * both-edge capture), SysTick-based blocking delays, and independent
 * watchdog (IWDG) management for the WFOC motor controller.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include "ciu32f003x.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* PPM Capture (TIM2 CH1 on PB4)                                            */
/* ======================================================================== */
/*! Initialise TIM2 CH1 for PPM pulse-width capture.
 *  Configures the timer for 1 MHz (1 us tick) with both-edge capture
 *  on TI1 (PB4) and enables the CC1 interrupt.
 *  \note PB4 GPIO must be configured as AF (gpio_config_board or gpio_init). */
void timer_init_ppm(void);

/*! Return the most recently captured PPM timer count (1 us ticks).
 *  Reads TIM2->CCR1 which holds the latched capture value from the
 *  last rising or falling edge on PB4. */
uint32_t timer_get_ppm_ticks(void);

/* ======================================================================== */
/* Independent Watchdog (IWDG)                                              */
/* ======================================================================== */
/*! Initialise the IWDG with a ~1 s timeout and start counting.
 *  Uses the 40 kHz LSI clock with /64 prescaler and a 625-count reload. */
void watchdog_init(void);

/*! Feed the IWDG to prevent a system reset. */
void watchdog_feed(void);

/* ======================================================================== */
/* SysTick-Based Blocking Delays                                             */
/* ======================================================================== */
/*! Blocking microsecond delay using the SysTick counter.
 *  Handles counter wrap for delays up to several milliseconds.
 *  \param us  Number of microseconds to wait. */
void timer_delay_us(uint32_t us);

/*! Blocking millisecond delay using the 1 ms SysTick tick.
 *  \param ms  Number of milliseconds to wait. */
void timer_delay_ms(uint32_t ms);

/* ======================================================================== */
/* Tick Counters                                                            */
/* ======================================================================== */
/*! Return the current millisecond counter (from SysTick_Handler). */
uint32_t timer_get_ms(void);

/*! Return the current microsecond timestamp (ms + sub-ms SysTick). */
uint32_t timer_get_us(void);

#ifdef __cplusplus
}
#endif

#endif /* TIMER_H */