/*
 * gpio.c - GPIO Driver for CIU32F003x5
 *
 * Generic pin configuration and board-level muxing for the WFOC V1 board.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include "gpio.h"
#include "../../config/board_config.h"

/* ======================================================================== */
/* Generic Pin Initialization                                                */
/* ======================================================================== */
void gpio_init(GPIO_TypeDef *port, uint8_t pin,
               uint8_t mode, gpio_pull_t pull,
               gpio_speed_t speed, uint8_t af)
{
    uint32_t pos2  = (2UL * pin);
    uint32_t mask2 = (0x3UL << pos2);

    /* --- Mode --- */
    MODIFY_REG(port->MODER, mask2, ((uint32_t)mode << pos2));

    /* --- Output type: push-pull (clear the bit) --- */
    CLEAR_BIT(port->OTYPER, (1UL << pin));

    /* --- Output speed --- */
    MODIFY_REG(port->OSPEEDR, mask2, ((uint32_t)speed << pos2));

    /* --- Pull-up / pull-down --- */
    MODIFY_REG(port->PUPDR, mask2, ((uint32_t)pull << pos2));

    /* --- Alternate function (only meaningful in AF mode) --- */
    if (pin < 8U) {
        uint32_t shift = (4UL * pin);
        MODIFY_REG(port->AFRL, (0xFUL << shift), ((uint32_t)af << shift));
    } else {
        uint32_t shift = (4UL * (pin - 8U));
        MODIFY_REG(port->AFRH, (0xFUL << shift), ((uint32_t)af << shift));
    }
}

/* ======================================================================== */
/* Board Pin Configuration                                                   */
/* ======================================================================== */
/* Configures every pin of the WFOC V1 board according to board_config.h.
 * SWD pins (PA2/SWCLK, PB6/SWDIO) are intentionally left in their reset
 * (alternate function 0) state so in-circuit debugging is never lost. */
void gpio_config_board(void)
{
    /* ---------------------------------------------------------------- */
    /* ADC inputs - analog, no pull                                      */
    /* ---------------------------------------------------------------- */
    gpio_init(ADC_IA_PORT,  ADC_IA_PIN,  GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_SPEED_LOW, 0U);
    gpio_init(GPIOA, 3U,    GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_SPEED_LOW, 0U); /* Vol_U */
    gpio_init(GPIOA, 4U,    GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_SPEED_LOW, 0U); /* Vol_V */
    gpio_init(GPIOA, 5U,    GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_SPEED_LOW, 0U); /* Vol_W */
    gpio_init(GPIOA, 6U,    GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_SPEED_LOW, 0U); /* Temp  */
    gpio_init(GPIOA, 7U,    GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_SPEED_LOW, 0U); /* Vbus  */
    gpio_init(ADC_IB_PORT,  ADC_IB_PIN,  GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_SPEED_LOW, 0U);

    /* ---------------------------------------------------------------- */
    /* PWM outputs - alternate function (TIM1), push-pull, high speed    */
    /* ---------------------------------------------------------------- */
    gpio_init(PWM_AH_PORT, PWM_AH_PIN, GPIO_MODE_AF, GPIO_PULL_NONE, GPIO_SPEED_HIGH, GPIO_AF_TIM1);
    gpio_init(PWM_AL_PORT, PWM_AL_PIN, GPIO_MODE_AF, GPIO_PULL_NONE, GPIO_SPEED_HIGH, GPIO_AF_TIM1);
    gpio_init(PWM_BH_PORT, PWM_BH_PIN, GPIO_MODE_AF, GPIO_PULL_NONE, GPIO_SPEED_HIGH, GPIO_AF_TIM1);
    gpio_init(PWM_BL_PORT, PWM_BL_PIN, GPIO_MODE_AF, GPIO_PULL_NONE, GPIO_SPEED_HIGH, GPIO_AF_TIM1);
    gpio_init(PWM_CH_PORT, PWM_CH_PIN, GPIO_MODE_AF, GPIO_PULL_NONE, GPIO_SPEED_HIGH, GPIO_AF_TIM1);
    gpio_init(PWM_CL_PORT, PWM_CL_PIN, GPIO_MODE_AF, GPIO_PULL_NONE, GPIO_SPEED_HIGH, GPIO_AF_TIM1);

    /* ---------------------------------------------------------------- */
    /* UART - PC0 TX (AF), PB5 RX (AF)                                   */
    /* ---------------------------------------------------------------- */
    gpio_init(UART_TX_PORT, UART_TX_PIN, GPIO_MODE_AF, GPIO_PULL_UP,   GPIO_SPEED_HIGH, GPIO_AF_USART1);
    gpio_init(UART_RX_PORT, UART_RX_PIN, GPIO_MODE_AF, GPIO_PULL_UP,   GPIO_SPEED_HIGH, GPIO_AF_USART1);

    /* ---------------------------------------------------------------- */
    /* PPM input - PB4 alternate function (TIM2_CH1)                     */
    /* ---------------------------------------------------------------- */
    gpio_init(PPM_PORT, PPM_PIN, GPIO_MODE_AF, GPIO_PULL_DOWN, GPIO_SPEED_LOW, GPIO_AF_TIM2);

    /* ---------------------------------------------------------------- */
    /* SWD pins: left in reset state (PA2=SWCLK, PB6=SWDIO, AF0).        */
    /* A weak pull-up on SWDIO improves noise margin without breaking SW. */
    MODIFY_REG(SWDIO_PORT->PUPDR, (0x3UL << (2UL * SWDIO_PIN)),
               (GPIO_PULL_UP << (2UL * SWDIO_PIN)));
}