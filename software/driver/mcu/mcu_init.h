/*
 * mcu_init.h - MCU System Initialization for CIU32F003x5
 *
 * System clock (24 MHz from internal RC), flash wait states, NVIC priority
 * grouping, global interrupt enable/disable and SysTick configuration.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef MCU_INIT_H
#define MCU_INIT_H

#include <stdint.h>
#include "ciu32f003x.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* System Clock                                                              */
/* ======================================================================== */
/*! Target system core clock (24 MHz HSI, board_config.h: MCU_CLK_HZ). */
extern uint32_t SystemCoreClock;

/*! Initialize system clock, flash latency and NVIC grouping. Call first. */
void mcu_init(void);

/*! Re-apply the 24 MHz HSI clock tree (e.g. after waking from stop). */
void mcu_clock_config(void);

/*! Configure flash wait states and prefetch for the running frequency. */
void mcu_flash_config(void);

/* ======================================================================== */
/* NVIC Priority Grouping                                                    */
/* ======================================================================== */
/*! Cortex-M0+ has a fixed priority grouping (no PRIGROUP field in AIRCR),
 *  so this is a documented no-op kept for API parity with STM32 layers. */
void mcu_nvic_priority_group_config(void);

/* ======================================================================== */
/* Global Interrupt Enable / Disable                                         */
/* ======================================================================== */
/*! Atomically enable interrupts (CPSIE i). */
static inline void mcu_irq_enable(void)
{
    __asm volatile ("cpsie i" : : : "memory");
}

/*! Atomically disable interrupts (CPSID i). Returns previous PRIMASK. */
static inline uint32_t mcu_irq_disable(void)
{
    uint32_t primask;
    __asm volatile ("mrs %0, primask\n"
                    "cpsid i" : "=r" (primask) : : "memory");
    return primask;
}

/*! Restore interrupt state previously returned by mcu_irq_disable(). */
static inline void mcu_irq_restore(uint32_t primask)
{
    __asm volatile ("msr primask, %0" : : "r" (primask) : "memory");
}

/*! Enter a critical section: disables interrupts, returns previous state. */
static inline uint32_t mcu_critical_enter(void) { return mcu_irq_disable(); }

/*! Leave a critical section, restoring previous interrupt state. */
static inline void mcu_critical_exit(uint32_t primask) { mcu_irq_restore(primask); }

/* ======================================================================== */
/* SysTick                                                                   */
/* ======================================================================== */
/*! Configure SysTick for periodic tick at ticks_per_sec (1 ms typical). */
void mcu_systick_config(uint32_t ticks_per_sec);

/*! Blocking delay (ms) using the SysTick counter polling. */
void mcu_delay_ms(uint32_t ms);

/*! SysTick milliseconds counter (incremented by SysTick_Handler). */
volatile uint32_t mcu_millis(void);

#ifdef __cplusplus
}
#endif

#endif /* MCU_INIT_H */