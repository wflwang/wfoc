/*
 * board_config.h - WFOC V1 Board Pin Mapping
 *
 * Hardware: CIU32F003x5 (QFN-20, ARM Cortex-M0+)
 * Gate Driver: FD6288 (TSSOP-20, 3-phase half-bridge)
 * Current Sense: 0.005R shunt + XOPA2333 op-amp (2 channels)
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* ======================================================================== */
/* MCU Clock Configuration                                                   */
/* ======================================================================== */
#define MCU_CLK_HZ          24000000   /* 24 MHz internal RC */
#define ADC_CLK_HZ          12000000   /* ADC clock */
#define PWM_FREQ_HZ         20000      /* Default PWM frequency */
#define PWM_FREQ_MIN_HZ     8000       /* Min PWM frequency */
#define PWM_FREQ_MAX_HZ     20000      /* Max PWM frequency */

/* ======================================================================== */
/* Pin Definitions - from PCB net mapping                                   */
/* ======================================================================== */

/* --- UART --- */
#define UART_TX_PORT        GPIOC
#define UART_TX_PIN         0           /* PC0 (pin 1, shared with NRST) */
#define UART_RX_PORT        GPIOB
#define UART_RX_PIN         5           /* PB5 (pin 7) */
#define UART_BAUDRATE       115200

/* --- PWM (FD6288 Gate Driver) --- */
/* Phase A */
#define PWM_AH_PORT         GPIOB
#define PWM_AH_PIN          2           /* PB2 (pin 10) - A high */
#define PWM_AL_PORT         GPIOB
#define PWM_AL_PIN          3           /* PB3 (pin 9) - A low */

/* Phase B */
#define PWM_BH_PORT         GPIOC
#define PWM_BH_PIN          1           /* PC1 (pin 2) - B high */
#define PWM_BL_PORT         GPIOB
#define PWM_BL_PIN          7           /* PB7 (pin 3) - B low */

/* Phase C */
#define PWM_CH_PORT         GPIOA
#define PWM_CH_PIN          1           /* PA1 (pin 14) - C high */
#define PWM_CL_PORT         GPIOA
#define PWM_CL_PIN          0           /* PA0 (pin 13) - C low */

/* --- ADC Channels --- */
/* Current sensing (2 channels, phase A & B) */
#define ADC_IA_CHANNEL      ADC_CH0     /* PB0 (pin 12) - Phase A current */
#define ADC_IA_PORT         GPIOB
#define ADC_IA_PIN          0

#define ADC_IB_CHANNEL      ADC_CH1     /* PB1 (pin 11) - Phase B current */
#define ADC_IB_PORT         GPIOB
#define ADC_IB_PIN          1

/* Phase voltage sensing */
#define ADC_VOL_U_CHANNEL   ADC_CH3     /* PA3 (pin 16) - Phase U voltage */
#define ADC_VOL_V_CHANNEL   ADC_CH4     /* PA4 (pin 17) - Phase V voltage */
#define ADC_VOL_W_CHANNEL   ADC_CH5     /* PA5 (pin 18) - Phase W voltage */

/* Bus voltage sensing */
#define ADC_VBUS_CHANNEL    ADC_CH7     /* PA7 (pin 20) - Bus voltage */

/* Temperature sensing */
#define ADC_TEMP_CHANNEL    ADC_CH6     /* PA6 (pin 19) - Motor temperature */

/* --- PPM Input --- */
#define PPM_PORT            GPIOB
#define PPM_PIN             4           /* PB4 (pin 8) */
#define PPM_TIMER           TIM2

/* --- SPI (reserved for remote control / OTA) ---
 * Note: MISO is connected to MOSI externally for half-duplex */
#define SPI_SCK_PORT        GPIOA
#define SPI_SCK_PIN         2           /* PA2 (shared with SWCLK) */
#define SPI_MOSI_PORT       GPIOA
#define SPI_MOSI_PIN        3           /* PA3 */
#define SPI_MISO_PORT       GPIOA
#define SPI_MISO_PIN        4           /* PA4 (not used internally) */
#define SPI_CSN_PORT        GPIOA
#define SPI_CSN_PIN         5           /* PA5 (chip select, active low) */

/* --- SWD Debug --- */
#define SWDIO_PORT          GPIOB
#define SWDIO_PIN           6           /* PB6 (pin 5) */
#define SWCLK_PORT          GPIOA
#define SWCLK_PIN           2           /* PA2 (pin 15) - shared with SPI_SCK */

/* ======================================================================== */
/* Hardware Parameters                                                       */
/* ======================================================================== */

/* Power supply */
#define VBUS_MIN_V          10.0f       /* Min bus voltage */
#define VBUS_MAX_V          48.0f       /* Max bus voltage */
#define VBUS_WARN_V         50.0f       /* Over-voltage warning */

/* Current sensing */
#define SHUNT_RESISTOR      0.005f      /* 5mΩ shunt resistor */
#define OP_AMP_GAIN         20.0f       /* Op-amp gain (adjustable via R5/R8) */
#define CURRENT_MAX_A       60.0f       /* Max measurable current */
#define CURRENT_CONT_A      20.0f       /* Continuous current rating */

/* ADC reference */
#define ADC_VREF_V          3.3f        /* ADC reference voltage */
#define ADC_RESOLUTION      4096        /* 12-bit ADC */
#define ADC_VREF_INT        3300        /* mV for fixed-point calc */

/* Voltage divider for bus voltage measurement */
/* VBUS -> R_top -> ADC -> R_bot -> GND */
#define VBUS_DIV_R_TOP      20000.0f    /* 20KΩ */
#define VBUS_DIV_R_BOT      2000.0f     /* 2KΩ */
#define VBUS_DIV_RATIO      ((VBUS_DIV_R_TOP + VBUS_DIV_R_BOT) / VBUS_DIV_R_BOT)

/* Temperature sensing (NTC) */
#define NTC_BETA            3950.0f     /* NTC B-value */
#define NTC_R25             10000.0f    /* NTC resistance at 25°C */
#define NTC_R_DIVIDER       10000.0f    /* Divider resistor */

/* MOSFET dead time (ns) */
#define DEADTIME_NS         500         /* 500ns default dead time */

/* PWM timer configuration */
#define PWM_TIMER           TIM1        /* Advanced timer for PWM */
#define PWM_PERIOD_DEFAULT  (MCU_CLK_HZ / (2 * PWM_FREQ_HZ))

/* ======================================================================== */
/* LED / Status (if available)                                              */
/* ======================================================================== */
/* No dedicated LED pin on this board - use UART for status */

#endif /* BOARD_CONFIG_H */
