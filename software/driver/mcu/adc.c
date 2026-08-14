/*
 * adc.c - ADC Driver for CIU32F003x5 (Motor Control)
 *
 * 12-bit ADC sampling 7 channels per PWM cycle via DMA, synchronized to the
 * TIM1 update event (low-side shunt current sensing point). End-of-sequence
 * interrupt drives the FOC current loop through adc_conv_complete_callback().
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include "adc.h"
#include "mcu_init.h"
#include "../../config/board_config.h"

/* ======================================================================== */
/* Local Definitions                                                         */
/* ======================================================================== */
/* External trigger: TIM1_TRGO (EXTSEL = 0000), rising edge (EXTEN = 01). */
#define ADC_EXTSEL_TIM1_TRGO        (0x0UL << ADC_CFGR1_EXTSEL_Pos)
#define ADC_EXTEN_RISING            (0x1UL << ADC_CFGR1_EXTEN_Pos)

/* ADC clock: synchronous PCLK/2 = 12 MHz (board_config.h ADC_CLK_HZ). */
#define ADC_CKMODE_PCLK_DIV2        (0x1UL << ADC_CFGR2_CKMODE_Pos)

/* IRQ priorities (Cortex-M0+ typically implements 2 priority bits). */
#define ADC_IRQ_PRIORITY            0U    /* highest - FOC current loop   */

/* ======================================================================== */
/* State                                                                     */
/* ======================================================================== */
/*! DMA destination buffer. Written by DMA1 channel 1, read by the FOC loop. */
static volatile uint16_t s_dma_buf[ADC_NUM_CHANNELS];

/*! Zero-current offsets, measured by adc_calibrate_offsets(). */
static uint16_t s_offset_a = (ADC_RESOLUTION / 2U);   /* mid-scale default */
static uint16_t s_offset_b = (ADC_RESOLUTION / 2U);

/* ======================================================================== */
/* Weak end-of-conversion callback (overridden by the FOC layer).           */
/* ======================================================================== */
void adc_conv_complete_callback(void) { /* default: no-op */ }

/* ======================================================================== */
/* Local helpers                                                             */
/* ======================================================================== */
static void adc_delay_us(volatile uint32_t us)
{
    /* Rough microsecond delay at 24 MHz (8 cycles/iteration). */
    while (us-- > 0U) {
        for (volatile uint32_t i = 0U; i < 3U; i++) { /* spin */ }
    }
}

/* ======================================================================== */
/* DMA1 Channel 1 configuration (ADC1 -> memory, circular, 16-bit)          */
/* ======================================================================== */
static void adc_dma_config(void)
{
    /* Disable channel before configuration. */
    CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_EN_Msk);

    /* Clear any pending DMA flags for channel 1. */
    DMA1->Common.IFCR = (DMA_ISR_GIF1_Msk | DMA_ISR_TCIF1_Msk |
                         DMA_ISR_HTIF1_Msk | DMA_ISR_TEIF1_Msk);

    /* Peripheral = ADC data register, memory = DMA buffer. */
    WRITE_REG(DMA1_Channel1->CPAR,  ADC1_DR_ADDR);
    WRITE_REG(DMA1_Channel1->CMAR,  (uint32_t)s_dma_buf);
    WRITE_REG(DMA1_Channel1->CNDTR, ADC_NUM_CHANNELS);

    /* 16-bit peripheral/memory, memory increment, circular, high priority,
     * peripheral-to-memory direction. */
    WRITE_REG(DMA1_Channel1->CCR,
              DMA_CCR_PSIZE_16BIT  | DMA_CCR_MSIZE_16BIT |
              DMA_CCR_MINC_Msk     | DMA_CCR_CIRC_Msk     |
              DMA_CCR_PL_HIGH);

    /* Enable channel. */
    SET_BIT(DMA1_Channel1->CCR, DMA_CCR_EN_Msk);
}

/* ======================================================================== */
/* ADC Initialization                                                        */
/* ======================================================================== */
void adc_init(void)
{
    /* Enable ADC and DMA clocks. */
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN_Msk);
    SET_BIT(RCC->AHBENR,  RCC_AHBENR_DMA1EN_Msk);

    /* ADC must be disabled before configuration. */
    CLEAR_BIT(ADC1->CR, ADC_CR_ADEN_Msk);

    /* Enable the ADC voltage regulator and let it stabilize. */
    SET_BIT(ADC1->CR, ADC_CR_ADVREGEN_Msk);
    adc_delay_us(20U);

    /* Run ADC calibration (ADCAL self-clears when done). */
    SET_BIT(ADC1->CR, ADC_CR_ADCAL_Msk);
    while (READ_BIT(ADC1->CR, ADC_CR_ADCAL_Msk) != 0U) {
        /* wait for calibration to finish */
    }

    /* ADC clock selection: PCLK/2 = 12 MHz. */
    MODIFY_REG(ADC1->CFGR2, ADC_CFGR2_CKMODE_Msk, ADC_CKMODE_PCLK_DIV2);

    /* Configure CFGR1:
     *  - 12-bit resolution, right aligned
     *  - DMA enabled, circular DMA
     *  - External trigger TIM1_TRGO, rising edge
     *  - Overrun: last data preserved (OVRMOD = 1)
     *  - Single sequence per trigger (CONT = 0) */
    WRITE_REG(ADC1->CFGR1,
              ADC_CFGR1_RES_12BIT   |
              ADC_CFGR1_DMAEN_Msk   | ADC_CFGR1_DMACFG_Msk |
              ADC_EXTSEL_TIM1_TRGO  | ADC_EXTEN_RISING     |
              ADC_CFGR1_OVRMOD_Msk);

    /* Sampling time: 12.5 ADC cycles (~1 us at 12 MHz) - sufficient for the
     * low-impedance op-amp outputs. */
    MODIFY_REG(ADC1->SMPR, ADC_SMPR_SMP_Msk, ADC_SMPR_SMP_12_5);

    /* Channel selection: 0,1,3,4,5,6,7 (channel 2 / SWCLK skipped). */
    WRITE_REG(ADC1->CHSELR,
              ADC_CH0 | ADC_CH1 | ADC_CH3 | ADC_CH4 |
              ADC_CH5 | ADC_CH6 | ADC_CH7);

    /* Set up DMA and the end-of-sequence interrupt. */
    adc_dma_config();
    SET_BIT(ADC1->IER, ADC_IER_EOSIE_Msk | ADC_IER_OVRIE_Msk);
    NVIC_SetPriority(ADC1_IRQn, ADC_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(ADC1_IRQn);
    NVIC_EnableIRQ(ADC1_IRQn);

    /* Enable the ADC and wait until ready. */
    SET_BIT(ADC1->CR, ADC_CR_ADEN_Msk);
    while (READ_BIT(ADC1->ISR, ADC_ISR_ADRDY_Msk) == 0U) {
        /* wait for ADRDY */
    }
    /* Clear ADRDY by writing 1. */
    SET_BIT(ADC1->ISR, ADC_ISR_ADRDY_Msk);
}

/* ======================================================================== */
/* Synchronized Sampling Control                                             */
/* ======================================================================== */
void adc_start_sync(void)
{
    /* Arm the ADC: with EXTEN set, each TIM1_TRGO launches one scan. */
    NVIC_ClearPendingIRQ(ADC1_IRQn);
    SET_BIT(ADC1->CR, ADC_CR_ADSTART_Msk);
}

void adc_stop_sync(void)
{
    if (READ_BIT(ADC1->CR, ADC_CR_ADSTART_Msk) != 0U) {
        SET_BIT(ADC1->CR, ADC_CR_ADSTP_Msk);
        while (READ_BIT(ADC1->CR, ADC_CR_ADSTP_Msk) != 0U) {
            /* wait for stop */
        }
    }
}

/* ======================================================================== */
/* Offset Calibration                                                        */
/* ======================================================================== */
int8_t adc_calibrate_offsets(void)
{
    uint32_t i;
    uint32_t acc;

    if (READ_BIT(ADC1->CR, ADC_CR_ADEN_Msk) == 0U) {
        return -1;  /* ADC not enabled */
    }

    /* Stop any ongoing conversion. */
    if (READ_BIT(ADC1->CR, ADC_CR_ADSTART_Msk) != 0U) {
        SET_BIT(ADC1->CR, ADC_CR_ADSTP_Msk);
        while (READ_BIT(ADC1->CR, ADC_CR_ADSTP_Msk) != 0U) {
            /* wait */
        }
    }

    /* Save and reconfigure for single-channel, continuous, software trigger. */
    uint32_t saved_cfgr1  = ADC1->CFGR1;
    uint32_t saved_chselr = ADC1->CHSELR;
    MODIFY_REG(ADC1->CFGR1,
               ADC_CFGR1_DMAEN_Msk | ADC_CFGR1_DMACFG_Msk |
               ADC_CFGR1_EXTEN_Msk | ADC_CFGR1_EXTSEL_Msk,
               ADC_CFGR1_CONT_Msk);

    /* --- Phase A offset (channel 0) --- */
    WRITE_REG(ADC1->CHSELR, ADC_CH0);
    SET_BIT(ADC1->CR, ADC_CR_ADSTART_Msk);
    acc = 0U;
    for (i = 0U; i < ADC_CAL_SAMPLES; i++) {
        while (READ_BIT(ADC1->ISR, ADC_ISR_EOC_Msk) == 0U) {
            /* wait for conversion */
        }
        SET_BIT(ADC1->ISR, ADC_ISR_EOC_Msk);
        acc += (ADC1->DR & 0x0FFFU);
    }
    SET_BIT(ADC1->CR, ADC_CR_ADSTP_Msk);
    while (READ_BIT(ADC1->CR, ADC_CR_ADSTP_Msk) != 0U) {
        /* wait */
    }
    s_offset_a = (uint16_t)(acc / ADC_CAL_SAMPLES);

    /* --- Phase B offset (channel 1) --- */
    WRITE_REG(ADC1->CHSELR, ADC_CH1);
    SET_BIT(ADC1->CR, ADC_CR_ADSTART_Msk);
    acc = 0U;
    for (i = 0U; i < ADC_CAL_SAMPLES; i++) {
        while (READ_BIT(ADC1->ISR, ADC_ISR_EOC_Msk) == 0U) {
            /* wait for conversion */
        }
        SET_BIT(ADC1->ISR, ADC_ISR_EOC_Msk);
        acc += (ADC1->DR & 0x0FFFU);
    }
    SET_BIT(ADC1->CR, ADC_CR_ADSTP_Msk);
    while (READ_BIT(ADC1->CR, ADC_CR_ADSTP_Msk) != 0U) {
        /* wait */
    }
    s_offset_b = (uint16_t)(acc / ADC_CAL_SAMPLES);

    /* Restore the multi-channel DMA + external-trigger configuration. */
    WRITE_REG(ADC1->CFGR1, saved_cfgr1);
    WRITE_REG(ADC1->CHSELR, saved_chselr);

    return 0;
}

uint16_t adc_get_offset_a(void) { return s_offset_a; }
uint16_t adc_get_offset_b(void) { return s_offset_b; }

/* ======================================================================== */
/* Data Accessors                                                            */
/* ======================================================================== */
uint16_t adc_get_raw(uint8_t index)
{
    if (index >= ADC_NUM_CHANNELS) {
        return 0U;
    }
    return s_dma_buf[index];
}

void adc_get_values(adc_values_t *out)
{
    uint32_t primask = mcu_critical_enter();
    out->ia    = s_dma_buf[ADC_IDX_IA];
    out->ib    = s_dma_buf[ADC_IDX_IB];
    out->vol_u = s_dma_buf[ADC_IDX_VOL_U];
    out->vol_v = s_dma_buf[ADC_IDX_VOL_V];
    out->vol_w = s_dma_buf[ADC_IDX_VOL_W];
    out->temp  = s_dma_buf[ADC_IDX_TEMP];
    out->vbus  = s_dma_buf[ADC_IDX_VBUS];
    mcu_critical_exit(primask);
}

int16_t adc_get_current_a(void)
{
    return (int16_t)((int16_t)s_dma_buf[ADC_IDX_IA] - (int16_t)s_offset_a);
}

int16_t adc_get_current_b(void)
{
    return (int16_t)((int16_t)s_dma_buf[ADC_IDX_IB] - (int16_t)s_offset_b);
}

uint16_t adc_get_vbus(void)
{
    return s_dma_buf[ADC_IDX_VBUS];
}

uint16_t adc_get_phase_voltage(uint8_t phase)
{
    uint8_t idx;
    switch (phase) {
        case 0U: idx = ADC_IDX_VOL_U; break;
        case 1U: idx = ADC_IDX_VOL_V; break;
        case 2U: idx = ADC_IDX_VOL_W; break;
        default: return 0U;
    }
    return s_dma_buf[idx];
}

/* ======================================================================== */
/* ADC End-of-Sequence Interrupt Handler                                     */
/* ======================================================================== */
/* ADC1_IRQHandler is implemented in interrupts.c where it drives the FOC
 * current control loop. The weak adc_conv_complete_callback() above remains
 * available as a secondary hook for non-FOC consumers. */