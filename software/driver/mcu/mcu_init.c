/*
 * mcu_init.c - MCU System Initialization for CIU32F003x5
 *
 * Implements 24 MHz HSI clock tree, flash latency, NVIC grouping and
 * SysTick 1 ms tick. Cortex-M0+ core (no FPU, no PRIGROUP).
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include "mcu_init.h"
#include "../../config/board_config.h"

/* SystemCoreClock is referenced by CMSIS-style code and the FOC layer. */
uint32_t SystemCoreClock = MCU_CLK_HZ;

/* SysTick millisecond counter, incremented in SysTick_Handler (interrupts.c). */
volatile uint32_t s_tick_ms = 0U;

/* ======================================================================== */
/* Flash Wait States & Prefetch                                              */
/* ======================================================================== */
/* CIU32F003x5 flash requires 1 wait state for 24 MHz operation. Prefetch
 * (PRFTBE) must be enabled to hide the wait state on sequential access. */
void mcu_flash_config(void)
{
    /* Enable prefetch buffer */
    SET_BIT(FLASH_REG->ACR, FLASH_ACR_PRFTBE_Msk);

    /* Wait until prefetch is effective */
    while (READ_BIT(FLASH_REG->ACR, FLASH_ACR_PRFTBS_Msk) == 0U) {
        /* wait for PRFTBS */
    }

    /* One wait state for 24 MHz (LATENCY = 1) */
    SET_BIT(FLASH_REG->ACR, FLASH_ACR_LATENCY_Msk);
}

/* ======================================================================== */
/* System Clock Configuration (24 MHz from internal HSI RC)                  */
/* ======================================================================== */
/* The CIU32F003x5 high-speed internal RC (HSI) is the 24 MHz source used by
 * this board (board_config.h: MCU_CLK_HZ). HSIDIV is set to /1 so HCLK runs
 * at the full 24 MHz; APB prescalers are /1 so peripherals also run at 24 MHz.
 *
 * NOTE: ciu32f003x.h defines HSI_VALUE as 8 MHz for legacy compatibility, but
 * the chip's active HSI used here is 24 MHz (see SystemCoreClock). Adjust only
 * if a future silicon revision changes the HSI trim. */
void mcu_clock_config(void)
{
    uint32_t timeout;

    /* Enable HSI and keep it undivided (HSIDIV = 0 -> divide by 1). */
    SET_BIT(RCC->CR, RCC_CR_HSION_Msk);
    CLEAR_BIT(RCC->CR, RCC_CR_HSIDIV_Msk);

    /* Wait for HSI ready, with a bounded timeout. */
    for (timeout = 0U; timeout < 0xFFFFU; timeout++) {
        if (READ_BIT(RCC->CR, RCC_CR_HSIRDY_Msk) != 0U) {
            break;
        }
    }

    /* AHB prescaler = /1, APB prescaler = /1 (HCLK = PCLK = 24 MHz). */
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE_Msk,
               RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE_DIV1);

    /* Select HSI as system clock (SW = 00). */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_HSI);

    /* Wait until HSI is switched in as the system clock. */
    for (timeout = 0U; timeout < 0xFFFFU; timeout++) {
        if ((RCC->CFGR & RCC_CFGR_SWS_Msk) == RCC_CFGR_SW_HSI) {
            break;
        }
    }

    SystemCoreClock = MCU_CLK_HZ;
}

/* ======================================================================== */
/* NVIC Priority Grouping                                                    */
/* ======================================================================== */
/* Cortex-M0+ implements no priority grouping (AIRCR.PRIGROUP is fixed).
 * The weak macro NVIC_SetPriorityGrouping() in ciu32f003x.h is already a
 * no-op; this function documents intent and keeps the API symmetric. */
void mcu_nvic_priority_group_config(void)
{
    /* No operation: Cortex-M0+ uses fixed preemption (no sub-priority). */
}

/* ======================================================================== */
/* SysTick                                                                   */
/* ======================================================================== */
void mcu_systick_config(uint32_t ticks_per_sec)
{
    uint32_t ticks = (MCU_CLK_HZ / ticks_per_sec) - 1U;

    /* Reload value must fit in 24 bits. */
    if (ticks > 0x00FFFFFFU) {
        ticks = 0x00FFFFFFU;
    }

    SysTick->LOAD = ticks;
    SysTick->VAL  = 0U;
    /* Core clock source, enable interrupt, enable counter. */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}

volatile uint32_t mcu_millis(void)
{
    return s_tick_ms;
}

/* Blocking delay. Uses the interrupt-driven tick once SysTick is running; for
 * very early boot (interrupts off / SysTick not yet armed) it falls back to an
 * approximate calibrated busy-wait at 24 MHz. */
void mcu_delay_ms(uint32_t ms)
{
    uint32_t ctrl = SysTick->CTRL;
    if ((ctrl & SysTick_CTRL_ENABLE_Msk) &&
        (ctrl & SysTick_CTRL_TICKINT_Msk)) {
        uint32_t start = s_tick_ms;
        while ((s_tick_ms - start) < ms) {
            /* wait on SysTick_Handler */
        }
    } else {
        while (ms-- > 0U) {
            /* ~6000 iterations approximates 1 ms at 24 MHz (-O0). */
            for (volatile uint32_t i = 0U; i < 6000U; i++) {
                /* spin */
            }
        }
    }
}

/* SysTick_Handler is implemented in interrupts.c (increments s_tick_ms). */

/* ======================================================================== */
/* Top-level MCU Initialization                                              */
/* ======================================================================== */
void mcu_init(void)
{
    /* Flash latency MUST be set before switching to 24 MHz. */
    mcu_flash_config();
    mcu_clock_config();
    mcu_nvic_priority_group_config();

    /* Enable GPIO port clocks (A/B/C) so pin configuration can proceed. */
    SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN_Msk);
    SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN_Msk);
    SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOCEN_Msk);

    /* 1 ms SysTick. */
    mcu_systick_config(1000U);
}