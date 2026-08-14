/*
 * timer.c - Timer & Watchdog Driver for CIU32F003x5
 *
 * Implements TIM2 CH1 PPM capture (1 MHz / 1 us tick, both-edge on PB4),
 * SysTick-based blocking delays and high-resolution timestamps, and the
 * IWDG independent watchdog for the WFOC motor controller.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include "timer.h"
#include "gpio.h"
#include "mcu_init.h"
#include "../../config/board_config.h"

/* ======================================================================== */
/* IWDG Register Map (not defined in ciu32f003x.h - local definition)       */
/* ======================================================================== */
typedef struct {
    __IO uint32_t KR;                       /*!< Offset 0x00: Key register    */
    __IO uint32_t PR;                       /*!< 0x04: Prescaler register     */
    __IO uint32_t RLR;                      /*!< 0x08: Reload register        */
    __IO uint32_t SR;                       /*!< 0x0C: Status register        */
} IWDG_TypeDef;

#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)

/* IWDG key register values */
#define IWDG_KEY_START       0xCCCCU         /*!< Start the watchdog           */
#define IWDG_KEY_FEED        0xAAAAU         /*!< Reload (feed) the watchdog   */
#define IWDG_KEY_UNLOCK      0x5555U         /*!< Unlock PR / RLR registers    */

/* IWDG prescaler register values (LSI = 40 kHz) */
#define IWDG_PR_DIV4         0x00U
#define IWDG_PR_DIV8         0x01U
#define IWDG_PR_DIV16        0x02U
#define IWDG_PR_DIV32        0x03U
#define IWDG_PR_DIV64        0x04U
#define IWDG_PR_DIV128       0x05U
#define IWDG_PR_DIV256       0x06U

/* IWDG SR bit definitions */
#define IWDG_SR_PVU_Pos      0
#define IWDG_SR_PVU_Msk      (1UL << IWDG_SR_PVU_Pos)
#define IWDG_SR_RVU_Pos      1
#define IWDG_SR_RVU_Msk      (1UL << IWDG_SR_RVU_Pos)

/* ======================================================================== */
/* External References                                                      */
/* ======================================================================== */
/* SysTick millisecond counter (defined in mcu_init.c). */
extern volatile uint32_t s_tick_ms;

/* ======================================================================== */
/* PPM Capture (TIM2 CH1 on PB4)                                            */
/* ======================================================================== */
/* TIM2 runs at 1 MHz (24 MHz PCLK / 24 = 1 MHz) giving 1 us per tick.
 * CH1 is configured for input capture on TI1 (PB4) with both-edge
 * triggering so that both rising and falling edges produce a timestamp
 * in CCR1 for pulse-width measurement. */
void timer_init_ppm(void)
{
    /* --- Enable TIM2 clock on APB1 --- */
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN_Msk);

    /* --- Reset TIM2 to a known state --- */
    CLEAR_BIT(TIM2->CR1, TIM_CR1_CEN_Msk);     /* stop counter          */
    TIM2->SR = 0U;                              /* clear all flags       */

    /* --- Prescaler: 24 MHz / (23 + 1) = 1 MHz -> 1 us per tick --- */
    TIM2->PSC = 23U;

    /* --- Auto-reload: maximum 32-bit value for free-running --- */
    TIM2->ARR = 0xFFFFFFFFU;

    /* --- Channel 1: input capture on TI1 (PB4), no filter, no prescaler --- */
    TIM2->CCMR1 = TIM_CCMR1_CC1S_INPUT_TI1;

    /* --- CCER: enable CH1 capture with both-edge polarity --- */
    TIM2->CCER = TIM_CCER_CC1E_Msk | TIM_CCER_CC1P_Msk;

    /* --- Enable capture/compare 1 interrupt --- */
    SET_BIT(TIM2->DIER, TIM_DIER_CC1IE_Msk);

    /* --- Enable update interrupt for overflow handling --- */
    SET_BIT(TIM2->DIER, TIM_DIER_UIE_Msk);

    /* --- Enable TIM2 in the NVIC so the IRQ handler can fire --- */
    NVIC_EnableIRQ(TIM2_IRQn);

    /* --- Clear any pending flags before starting --- */
    WRITE_REG(TIM2->SR, 0U);

    /* --- Start the counter --- */
    SET_BIT(TIM2->CR1, TIM_CR1_CEN_Msk);
}

uint32_t timer_get_ppm_ticks(void)
{
    return TIM2->CCR1;
}

/* ======================================================================== */
/* Independent Watchdog (IWDG)                                              */
/* ======================================================================== */
/* The IWDG runs from the 40 kHz LSI RC oscillator.  With a /64 prescaler
 * and a 625-count reload, the timeout is approximately 1 second.  The
 * watchdog is started once and must be fed periodically to prevent a
 * system reset. */
void watchdog_init(void)
{
    /* --- Unlock PR and RLR registers --- */
    WRITE_REG(IWDG->KR, IWDG_KEY_UNLOCK);

    /* --- Wait until PR is writable --- */
    while (READ_BIT(IWDG->SR, IWDG_SR_PVU_Msk) != 0U) {
        /* wait for prescaler register update */
    }

    /* --- Set prescaler to /64 (40 kHz / 64 = 625 Hz) --- */
    WRITE_REG(IWDG->PR, IWDG_PR_DIV64);

    /* --- Wait until RLR is writable --- */
    while (READ_BIT(IWDG->SR, IWDG_SR_RVU_Msk) != 0U) {
        /* wait for reload register update */
    }

    /* --- Set reload value for ~1 s timeout (625 counts at 625 Hz) --- */
    WRITE_REG(IWDG->RLR, 625U);

    /* --- Start the watchdog --- */
    WRITE_REG(IWDG->KR, IWDG_KEY_START);
}

void watchdog_feed(void)
{
    WRITE_REG(IWDG->KR, IWDG_KEY_FEED);
}

/* ======================================================================== */
/* SysTick-Based Blocking Delays                                             */
/* ======================================================================== */
/* The Cortex-M0+ SysTick counter is a 24-bit down-counter clocked at the
 * core frequency (24 MHz).  At 24 MHz, each tick = 1/24 us ≈ 41.67 ns,
 * so 24 ticks = 1 us.  The counter wraps from LOAD down to 0 and then
 * reloads LOAD + 1, so elapsed-time calculations must handle wrap. */

/* Number of SysTick ticks per microsecond (MCU_CLK_HZ / 1_000_000). */
#define SYSTICK_TICKS_PER_US   (MCU_CLK_HZ / 1000000UL)

void timer_delay_us(uint32_t us)
{
    if (us == 0U) {
        return;
    }

    uint32_t start   = SysTick->VAL;
    uint32_t ticks   = us * SYSTICK_TICKS_PER_US;
    uint32_t load    = SysTick->LOAD;

    while (1) {
        uint32_t current = SysTick->VAL;

        if (current <= start) {
            /* Counter has not wrapped (or just at the same value). */
            if ((start - current) >= ticks) {
                break;
            }
        } else {
            /* Counter wrapped: start -> 0 -> LOAD -> current. */
            uint32_t elapsed = start + load + 1U - current;
            if (elapsed >= ticks) {
                break;
            }
        }
    }
}

void timer_delay_ms(uint32_t ms)
{
    uint32_t start = s_tick_ms;
    while ((s_tick_ms - start) < ms) {
        /* wait for SysTick_Handler to advance s_tick_ms */
    }
}

/* ======================================================================== */
/* Tick Counters                                                            */
/* ======================================================================== */
uint32_t timer_get_ms(void)
{
    return s_tick_ms;
}

uint32_t timer_get_us(void)
{
    uint32_t ms;
    uint32_t val;
    uint32_t primask;

    /* Snapshot the millisecond counter and SysTick VAL atomically so that
     * a SysTick wrap between reading ms and VAL cannot corrupt the result. */
    primask = mcu_critical_enter();
    ms  = s_tick_ms;
    val = SysTick->VAL;
    mcu_critical_exit(primask);

    uint32_t sub_us = (SysTick->LOAD - val) / SYSTICK_TICKS_PER_US;
    return (ms * 1000U) + sub_us;
}