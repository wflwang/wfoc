/*
 * pwm.c - PWM Driver for CIU32F003x5 (3-Phase Motor Control)
 *
 * TIM1 center-aligned PWM with complementary outputs, dead-time, and
 * update-event TRGO for synchronized ADC sampling at the PWM rate.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include "pwm.h"
#include "gpio.h"
#include "mcu_init.h"
#include "../../config/board_config.h"

/* ======================================================================== */
/* Local Definitions                                                         */
/* ======================================================================== */

#define TIM1_CLK_HZ             (MCU_CLK_HZ)

#define PWM_DEADTIME_DEFAULT_NS (DEADTIME_NS)

/* ======================================================================== */
/* Helper: compute ARR for a given PWM frequency                              */
/* ======================================================================== */
static uint32_t pwm_compute_arr(uint32_t freq_hz)
{
    uint32_t arr = TIM1_CLK_HZ / (2U * freq_hz);
    if (arr < 2U) {
        arr = 2U;
    }
    if (arr > 65535U) {
        arr = 65535U;
    }
    return arr;
}

/* ======================================================================== */
/* Helper: compute DTG register value from dead-time in ns                   */
/* ======================================================================== */
static uint32_t pwm_compute_dtg(uint32_t deadtime_ns)
{
    uint32_t dtg = (deadtime_ns * TIM1_CLK_HZ) / 1000000000U;
    if (dtg > 0xFFU) {
        dtg = 0xFFU;
    }
    return dtg;
}

/* ======================================================================== */
/* Initialization                                                            */
/* ======================================================================== */
void pwm_init(uint32_t freq_hz)
{
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN_Msk);

    SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_TIM1RST_Msk);
    CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_TIM1RST_Msk);

    /* --- GPIO: PWM outputs (TIM1 alternate function, push-pull, high speed) --- */
    gpio_init(PWM_AH_PORT, PWM_AH_PIN, GPIO_MODE_AF, GPIO_PULL_NONE,
              GPIO_SPEED_HIGH, GPIO_AF_TIM1);
    gpio_init(PWM_AL_PORT, PWM_AL_PIN, GPIO_MODE_AF, GPIO_PULL_NONE,
              GPIO_SPEED_HIGH, GPIO_AF_TIM1);
    gpio_init(PWM_BH_PORT, PWM_BH_PIN, GPIO_MODE_AF, GPIO_PULL_NONE,
              GPIO_SPEED_HIGH, GPIO_AF_TIM1);
    gpio_init(PWM_BL_PORT, PWM_BL_PIN, GPIO_MODE_AF, GPIO_PULL_NONE,
              GPIO_SPEED_HIGH, GPIO_AF_TIM1);
    gpio_init(PWM_CH_PORT, PWM_CH_PIN, GPIO_MODE_AF, GPIO_PULL_NONE,
              GPIO_SPEED_HIGH, GPIO_AF_TIM1);
    gpio_init(PWM_CL_PORT, PWM_CL_PIN, GPIO_MODE_AF, GPIO_PULL_NONE,
              GPIO_SPEED_HIGH, GPIO_AF_TIM1);

    uint32_t arr = pwm_compute_arr(freq_hz);

    /* CR1: center-aligned mode 3, ARR preload */
    WRITE_REG(TIM1->CR1,
              TIM_CR1_CMS_CENTER3 |
              TIM_CR1_ARPE_Msk);

    /* CR2: CCPC preload, MMS = update event (TRGO for ADC sync) */
    WRITE_REG(TIM1->CR2,
              TIM_CR2_CCPC_Msk |
              (0x2UL << TIM_CR2_MMS_Pos));

    WRITE_REG(TIM1->SMCR, 0U);
    WRITE_REG(TIM1->DIER, 0U);

    /* CCMR1: CH1 PWM1, CH2 PWM1, preload enable */
    WRITE_REG(TIM1->CCMR1,
              TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE_Msk |
              TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE_Msk);

    /* CCMR2: CH3 PWM1, preload enable */
    WRITE_REG(TIM1->CCMR2,
              TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE_Msk);

    /* CCER: enable all 6 outputs */
    WRITE_REG(TIM1->CCER,
              TIM_CCER_CC1E_Msk | TIM_CCER_CC1NE_Msk |
              TIM_CCER_CC2E_Msk | TIM_CCER_CC2NE_Msk |
              TIM_CCER_CC3E_Msk | TIM_CCER_CC3NE_Msk);

    WRITE_REG(TIM1->CNT, 0U);
    WRITE_REG(TIM1->PSC, 0U);
    WRITE_REG(TIM1->ARR, arr);

    /* RCR = 1: center-aligned mode generates two update events per full
     * cycle (peak and trough). RCR = 1 halves the rate so the ADC
     * triggers at exactly the PWM frequency. */
    WRITE_REG(TIM1->RCR, 1U);

    /* CCRs: 0% duty (safe default until pwm_enable + duty set) */
    WRITE_REG(TIM1->CCR1, 0U);
    WRITE_REG(TIM1->CCR2, 0U);
    WRITE_REG(TIM1->CCR3, 0U);

    /* BDTR: dead-time, outputs idle until pwm_enable()
     * MOE = 0: main outputs disabled (pwm_enable sets MOE = 1)
     * AOE = 0: no automatic output enable
     * OSSI = 1: idle state off when MOE = 0
     * OSSR = 1: run state off when MOE = 0 and break occurs */
    uint32_t dtg = pwm_compute_dtg(PWM_DEADTIME_DEFAULT_NS);
    WRITE_REG(TIM1->BDTR,
              TIM_BDTR_OSSI_Msk | TIM_BDTR_OSSR_Msk |
              (dtg << TIM_BDTR_DTG_Pos));

    WRITE_REG(TIM1->DCR, 0U);
    WRITE_REG(TIM1->DMAR, 0U);

    TIM1_ENABLE();
}

/* ======================================================================== */
/* PWM Output Control                                                        */
/* ======================================================================== */
void pwm_enable(void)
{
    SET_BIT(TIM1->BDTR, TIM_BDTR_MOE_Msk);
}

void pwm_disable(void)
{
    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE_Msk);
}

/* ======================================================================== */
/* Duty Cycle Control                                                        */
/* ======================================================================== */
void pwm_set_duty_q16(pwm_channel_t channel, int32_t duty)
{
    if (duty < 0) {
        duty = 0;
    }
    if (duty > 0xFFFF) {
        duty = 0xFFFF;
    }

    uint32_t arr = TIM1->ARR;
    uint16_t ccr = (uint16_t)((duty * (int32_t)arr) >> 16);

    switch (channel) {
        case PWM_CH_A: TIM1->CCR1 = ccr; break;
        case PWM_CH_B: TIM1->CCR2 = ccr; break;
        case PWM_CH_C: TIM1->CCR3 = ccr; break;
        default: break;
    }
}

void pwm_set_duty(pwm_channel_t channel, uint16_t duty)
{
    if (duty > TIM1->ARR) {
        duty = (uint16_t)TIM1->ARR;
    }

    switch (channel) {
        case PWM_CH_A: TIM1->CCR1 = duty; break;
        case PWM_CH_B: TIM1->CCR2 = duty; break;
        case PWM_CH_C: TIM1->CCR3 = duty; break;
        default: break;
    }
}

/* ======================================================================== */
/* Timer Parameter Access                                                    */
/* ======================================================================== */
uint32_t pwm_get_arr(void)
{
    return TIM1->ARR;
}

void pwm_set_deadtime(uint32_t ns)
{
    uint32_t dtg = pwm_compute_dtg(ns);
    MODIFY_REG(TIM1->BDTR, TIM_BDTR_DTG_Msk, dtg << TIM_BDTR_DTG_Pos);
}

void pwm_set_freq(uint32_t freq_hz)
{
    uint32_t arr = pwm_compute_arr(freq_hz);
    MODIFY_REG(TIM1->ARR, 0xFFFFFFFFU, arr);
}