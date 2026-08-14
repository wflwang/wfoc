/*
 * gpio.h - GPIO Driver for CIU32F003x5
 *
 * Generic GPIO initialization (mode/pull/speed/alternate function) plus
 * fast inline set/reset/read/toggle helpers and board-level pin muxing
 * driven by board_config.h.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include "ciu32f003x.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* Alternate Function Numbers                                                */
/* ======================================================================== */
/* The CIU32F003x5 AF mapping follows the STM32G0-style scheme. The values
 * below are the defaults used by this board; if a silicon revision changes
 * the mapping, override them in board_config.h before including this file. */
#ifndef GPIO_AF_TIM1
#define GPIO_AF_TIM1          2U    /*!< TIM1 CHx/CHxN on PWM pins           */
#endif
#ifndef GPIO_AF_TIM2
#define GPIO_AF_TIM2          2U    /*!< TIM2 CH1 on PB4 (PPM)              */
#endif
#ifndef GPIO_AF_USART1
#define GPIO_AF_USART1        1U    /*!< USART1 TX/RX on PC0/PB5            */
#endif

/* ======================================================================== */
/* Configuration Enums (values match ciu32f003x.h register field values)    */
/* ======================================================================== */
typedef enum {
    GPIO_PULL_NONE   = GPIO_PUPDR_NONE,
    GPIO_PULL_UP     = GPIO_PUPDR_PULLUP,
    GPIO_PULL_DOWN   = GPIO_PUPDR_PULLDN,
} gpio_pull_t;

typedef enum {
    GPIO_SPEED_LOW   = GPIO_OSPEEDR_LOW,
    GPIO_SPEED_MED   = GPIO_OSPEEDR_MED,
    GPIO_SPEED_HIGH  = GPIO_OSPEEDR_HIGH,
} gpio_speed_t;

/* Mode values are the GPIO_MODE_* macros from ciu32f003x.h:
 *   GPIO_MODE_INPUT / GPIO_MODE_OUTPUT / GPIO_MODE_AF / GPIO_MODE_ANALOG */

/* ======================================================================== */
/* Initialization                                                            */
/* ======================================================================== */
/*! Configure a single GPIO pin.
 *  \param mode   GPIO_MODE_INPUT / _OUTPUT / _AF / _ANALOG
 *  \param pull   gpio_pull_t
 *  \param speed  gpio_speed_t (output / AF only)
 *  \param af     alternate function 0..15 (AF mode only) */
void gpio_init(GPIO_TypeDef *port, uint8_t pin,
               uint8_t mode, gpio_pull_t pull,
               gpio_speed_t speed, uint8_t af);

/*! Configure all board pins per board_config.h (ADC, PWM, UART, PPM, SWD). */
void gpio_config_board(void);

/* ======================================================================== */
/* Fast Inline Pin Operations                                                */
/* ======================================================================== */
/*! Drive a pin high (atomic via BSRR). */
static inline void gpio_set(GPIO_TypeDef *port, uint8_t pin)
{
    port->BSRR = (1UL << pin);
}

/*! Drive a pin low (atomic via BSRR). */
static inline void gpio_reset(GPIO_TypeDef *port, uint8_t pin)
{
    port->BRR = (1UL << pin);
}

/*! Toggle a pin via ODR (use within a critical section for atomicity). */
static inline void gpio_toggle(GPIO_TypeDef *port, uint8_t pin)
{
    port->ODR ^= (1UL << pin);
}

/*! Read the logical level of a pin from IDR. Returns 0 or 1. */
static inline uint8_t gpio_read(GPIO_TypeDef *port, uint8_t pin)
{
    return (uint8_t)((port->IDR >> pin) & 1UL);
}

/*! Write a 0/1 value to a pin. */
static inline void gpio_write(GPIO_TypeDef *port, uint8_t pin, uint8_t val)
{
    if (val) {
        port->BSRR = (1UL << pin);
    } else {
        port->BRR  = (1UL << pin);
    }
}

#ifdef __cplusplus
}
#endif

#endif /* GPIO_H */