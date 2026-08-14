/*
 * spi.c - SPI Driver Implementation for WFOC Motor Controller
 *
 * Implements SPI peripheral operations for future remote control
 * and OTA upgrade support. Uses blocking transfers for simplicity.
 *
 * Note: This is a placeholder implementation. The CIU32F003x5 has
 * limited SPI capabilities, and the pin mapping may need to be
 * adjusted based on the final PCB design.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include "driver/mcu/spi.h"
#include "config/hwconf.h"
#include "driver/mcu/ciu32f003x.h"
#include <string.h>

/* ======================================================================== */
/* Register Access Aliases                                                   */
/* ======================================================================== */
/* Map the generic register names used in this driver to the actual
 * CIU32F003x5 peripheral base addresses. */
#define GPIO_REG    GPIOA
#define SPI1_REG    SPI1
#define RCC_REG     RCC

/* ======================================================================== */
/* Internal State                                                            */
/* ======================================================================== */

typedef struct {
    bool initialized;
    uint8_t div;            /* Clock divider */
    uint8_t mode;           /* SPI mode */
    uint8_t sbit;           /* Bit order */
    uint8_t csn_active;     /* CSN active level */
    bool async_active;      /* True if async transfer in progress */
    spi_callback_t async_cb; /* Async completion callback */
    uint16_t async_remaining; /* Remaining bytes in async transfer */
    uint8_t *async_tx_data;
    uint8_t *async_rx_data;
} spi_state_t;

static spi_state_t s_spi = {0};

/* ======================================================================== */
/* GPIO Pin Configuration                                                    */
/* ======================================================================== */

static void spi_config_pins(void)
{
    /* Configure SPI_SCK (PA2) as AF output (high speed) */
    MODIFY_REG(GPIO_REG->MODER, GPIO_MODEER_Msk(2), GPIO_MODE_AF << GPIO_MODEER_Pos(2));
    MODIFY_REG(GPIO_REG->AFRL, 0xFUL << (2 * 4), 1UL << (2 * 4));  /* AF1 for SPI1 */

    /* Configure SPI_MOSI (PA3) as AF output */
    MODIFY_REG(GPIO_REG->MODER, GPIO_MODEER_Msk(3), GPIO_MODE_AF << GPIO_MODEER_Pos(3));
    MODIFY_REG(GPIO_REG->AFRL, 0xFUL << (3 * 4), 1UL << (3 * 4));  /* AF1 for SPI1 */

    /* Configure SPI_MISO (PA4) as AF input (for half-duplex operation)
     * External hardware connects MISO to MOSI, so MISO will receive
     * data on the same physical line. */
    MODIFY_REG(GPIO_REG->MODER, GPIO_MODEER_Msk(4), GPIO_MODE_AF << GPIO_MODEER_Pos(4));
    MODIFY_REG(GPIO_REG->AFRL, 0xFUL << (4 * 4), 1UL << (4 * 4));  /* AF1 for SPI1 */

    /* Configure SPI_CSN (PA5) as GPIO output (manual CSN control) */
    MODIFY_REG(GPIO_REG->MODER, GPIO_MODEER_Msk(5), GPIO_MODE_OUTPUT << GPIO_MODEER_Pos(5));
    CLEAR_BIT(GPIO_REG->OTYPER, GPIO_OTYPER_PP_Msk(5));  /* Push-pull */
    MODIFY_REG(GPIO_REG->OSPEEDR, GPIO_MODEER_Msk(5), GPIO_OSPEEDR_HIGH << GPIO_MODEER_Pos(5));

    /* Set CSN high (inactive) */
    GPIO_REG->BSRR = (1UL << 5);  /* BS5 */
}

static void spi_deconfig_pins(void)
{
    /* Revert pins to default (input) mode */
    MODIFY_REG(GPIO_REG->MODER, GPIO_MODEER_Msk(2), GPIO_MODE_INPUT << GPIO_MODEER_Pos(2));
    MODIFY_REG(GPIO_REG->MODER, GPIO_MODEER_Msk(3), GPIO_MODE_INPUT << GPIO_MODEER_Pos(3));
    MODIFY_REG(GPIO_REG->MODER, GPIO_MODEER_Msk(4), GPIO_MODE_INPUT << GPIO_MODEER_Pos(4));
    MODIFY_REG(GPIO_REG->MODER, GPIO_MODEER_Msk(5), GPIO_MODE_INPUT << GPIO_MODEER_Pos(5));
}

/* ======================================================================== */
/* SPI Peripheral Configuration                                              */
/* ======================================================================== */

void spi_init(uint8_t div, uint8_t mode, uint8_t sbit)
{
    uint32_t cr1_val = 0;

    /* Store configuration */
    s_spi.div = div;
    s_spi.mode = mode;
    s_spi.sbit = sbit;
    s_spi.csn_active = 0;  /* CSN active low by default */

    /* Enable GPIOA and SPI1 clocks */
    RCC_REG->IOPENR |= RCC_IOPENR_GPIOAEN_Msk;
    RCC_REG->APB2ENR |= RCC_APB2ENR_SPI1EN_Msk;
    
    /* Configure GPIO pins */
    spi_config_pins();
    
    /* Configure SPI CR1 register */
    cr1_val |= SPI_CR1_MSTR_Msk;    /* Master mode */
    cr1_val |= SPI_CR1_SSI_Msk;     /* Internal slave select */
    cr1_val |= SPI_CR1_SSM_Msk;     /* Software slave management */
    
    /* Set clock divider */
    cr1_val |= (uint32_t)div << 3;
    
    /* Set CPOL and CPHA based on mode */
    switch (mode) {
    case SPI_MODE_0:
        break;
    case SPI_MODE_1:
        cr1_val |= SPI_CR1_CPHA_Msk;
        break;
    case SPI_MODE_2:
        cr1_val |= SPI_CR1_CPOL_Msk;
        break;
    case SPI_MODE_3:
        cr1_val |= (SPI_CR1_CPOL_Msk | SPI_CR1_CPHA_Msk);
        break;
    }
    
    /* Set bit order (MSB or LSB first) */
    if (sbit) {
        cr1_val |= SPI_CR1_LSBFIRST_Msk;
    }
    
    /* Set data frame format (8-bit) */
    /* Default is 8-bit (DS=0) */
    
    /* Write CR1 configuration (but don't enable yet) */
    SPI1_REG->CR1 = cr1_val;
    
    /* Enable SPI */
    SPI1_REG->CR1 |= SPI_CR1_SPE_Msk;
    
    s_spi.initialized = true;
}

void spi_deinit(void)
{
    if (!s_spi.initialized) {
        return;
    }

    /* Disable SPI */
    SPI1_REG->CR1 &= ~SPI_CR1_SPE_Msk;

    /* Deconfigure pins */
    spi_deconfig_pins();

    /* Disable SPI clock (keep GPIOA clock for other peripherals) */
    RCC_REG->APB2ENR &= ~RCC_APB2ENR_SPI1EN_Msk;

    s_spi.initialized = false;
}

/* ======================================================================== */
/* Blocking Transfer Implementation                                           */
/* ======================================================================== */

bool spi_transfer(const spi_transfer_t *transfer)
{
    uint16_t i;
    uint8_t tx_val, rx_val;
    
    if (!s_spi.initialized || transfer == NULL) {
        return false;
    }
    
    if (transfer->length == 0 || transfer->length > SPI_MAX_TRANSFER_SIZE) {
        return false;
    }
    
    /* Pull CSN low (active) */
    spi_csn_low();
    
    /* Wait for TX buffer to be empty */
    while (!(SPI1_REG->SR & SPI_SR_TXE_Msk)) {
        /* spin wait */
    }
    
    for (i = 0; i < transfer->length; i++) {
        /* Get transmit byte */
        if (transfer->tx_data != NULL) {
            tx_val = transfer->tx_data[i];
        } else {
            tx_val = 0xFF;  /* Default value for receive-only */
        }
        
        /* Send byte */
        SPI1_REG->DR = tx_val;
        
        /* Wait for receive data */
        while (!(SPI1_REG->SR & SPI_SR_RXNE_Msk)) {
            /* spin wait */
        }
        
        /* Read received byte */
        rx_val = (uint8_t)SPI1_REG->DR;
        
        /* Store received data */
        if (transfer->rx_data != NULL) {
            transfer->rx_data[i] = rx_val;
        }
    }
    
    /* Wait for last transfer to complete */
    while (SPI1_REG->SR & SPI_SR_BSY_Msk) {
        /* spin wait */
    }
    
    /* Release CSN */
    spi_csn_high();
    
    return true;
}

bool spi_transmit(const uint8_t *data, uint16_t length)
{
    spi_transfer_t transfer;
    
    transfer.tx_data = (uint8_t *)data;
    transfer.rx_data = NULL;
    transfer.length = length;
    transfer.csn_polarity = s_spi.csn_active;
    
    return spi_transfer(&transfer);
}

bool spi_receive(uint8_t *data, uint16_t length, uint8_t tx_val)
{
    spi_transfer_t transfer;
    uint8_t tx_buf[SPI_MAX_TRANSFER_SIZE];
    uint16_t i;
    
    /* Prepare dummy transmit buffer */
    for (i = 0; i < length; i++) {
        tx_buf[i] = tx_val;
    }
    
    transfer.tx_data = tx_buf;
    transfer.rx_data = data;
    transfer.length = length;
    transfer.csn_polarity = s_spi.csn_active;
    
    return spi_transfer(&transfer);
}

/* ======================================================================== */
/* CSN Manual Control                                                        */
/* ======================================================================== */

void spi_csn_low(void)
{
    if (s_spi.csn_active == 0) {
        /* Active low - reset bit 5 */
        GPIO_REG->BRR = (1UL << 5);
    } else {
        /* Active high - set bit 5 */
        GPIO_REG->BSRR = (1UL << 5);
    }
}

void spi_csn_high(void)
{
    if (s_spi.csn_active == 0) {
        /* Active low - set bit 5 */
        GPIO_REG->BSRR = (1UL << 5);
    } else {
        /* Active high - reset bit 5 */
        GPIO_REG->BRR = (1UL << 5);
    }
}

/* ======================================================================== */
/* SPI Half-Duplex Implementation                                            */
/* ======================================================================== */
/*
 * The WFOC board uses a half-duplex SPI configuration:
 *   - MOSI (PA3) and MISO (PA4) are connected together externally
 *   - This allows bidirectional communication on a single data line
 *   - The SPI peripheral can be configured in half-duplex mode
 *
 * For half-duplex operation:
 *   1. Transmit phase: MOSI is configured as AF output, MISO is high-Z
 *   2. Receive phase: MOSI is high-Z, MISO is configured as AF input
 *   3. CSN controls the transaction
 *
 * Since the CIU32F003x5 may not have hardware half-duplex support,
 * we implement it in software by dynamically reconfiguring pin modes.
 */

/* Configure MOSI for transmit mode (AF output) */
static void spi_config_mosi_tx(void)
{
    /* Configure SPI_MOSI (PA3) as AF output */
    MODIFY_REG(GPIO_REG->MODER, GPIO_MODEER_Msk(3), GPIO_MODE_AF << GPIO_MODEER_Pos(3));
    MODIFY_REG(GPIO_REG->AFRL, 0xFUL << (3 * 4), 1UL << (3 * 4));  /* AF1 */

    /* Configure SPI_MISO (PA4) as input (high-Z) */
    MODIFY_REG(GPIO_REG->MODER, GPIO_MODEER_Msk(4), GPIO_MODE_INPUT << GPIO_MODEER_Pos(4));
}

/* Configure MOSI for receive mode (MISO becomes input) */
static void spi_config_mosi_rx(void)
{
    /* Configure SPI_MOSI (PA3) as input (high-Z) */
    MODIFY_REG(GPIO_REG->MODER, GPIO_MODEER_Msk(3), GPIO_MODE_INPUT << GPIO_MODEER_Pos(3));

    /* Configure SPI_MISO (PA4) as AF input */
    MODIFY_REG(GPIO_REG->MODER, GPIO_MODEER_Msk(4), GPIO_MODE_AF << GPIO_MODEER_Pos(4));
    MODIFY_REG(GPIO_REG->AFRL, 0xFUL << (4 * 4), 1UL << (4 * 4));  /* AF1 */
}

/* Transmit data in half-duplex mode */
bool spi_half_duplex_transmit(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (!s_spi.initialized || data == NULL) {
        return false;
    }

    if (length == 0 || length > SPI_MAX_TRANSFER_SIZE) {
        return false;
    }

    /* Configure for transmit */
    spi_config_mosi_tx();

    /* Pull CSN low */
    spi_csn_low();

    /* Wait for TX buffer to be empty */
    while (!(SPI1_REG->SR & SPI_SR_TXE_Msk));

    /* Transmit data - send 0xFF dummy to clock out data on MOSI */
    for (i = 0; i < length; i++) {
        /* Send byte on MOSI */
        SPI1_REG->DR = data[i];

        /* Wait for transfer to complete */
        while (!(SPI1_REG->SR & SPI_SR_RXNE_Msk));
        (void)SPI1_REG->DR;  /* Read dummy receive data */
    }

    /* Wait for last transfer to complete */
    while (SPI1_REG->SR & SPI_SR_BSY_Msk);

    /* Release CSN */
    spi_csn_high();

    return true;
}

/* Receive data in half-duplex mode
 * The external device drives the data line during reception */
bool spi_half_duplex_receive(uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (!s_spi.initialized || data == NULL) {
        return false;
    }

    if (length == 0 || length > SPI_MAX_TRANSFER_SIZE) {
        return false;
    }

    /* Configure for receive */
    spi_config_mosi_rx();

    /* Pull CSN low */
    spi_csn_low();

    /* Wait for TX buffer to be empty */
    while (!(SPI1_REG->SR & SPI_SR_TXE_Msk));

    /* Receive data - send 0xFF to generate clock */
    for (i = 0; i < length; i++) {
        /* Send dummy clock */
        SPI1_REG->DR = 0xFF;

        /* Wait for received data */
        while (!(SPI1_REG->SR & SPI_SR_RXNE_Msk));

        /* Read received byte */
        data[i] = (uint8_t)SPI1_REG->DR;
    }

    /* Wait for last transfer to complete */
    while (SPI1_REG->SR & SPI_SR_BSY_Msk);

    /* Release CSN */
    spi_csn_high();

    /* Restore transmit mode for next transaction */
    spi_config_mosi_tx();

    return true;
}

/* Full half-duplex transaction: transmit then receive */
bool spi_half_duplex_transfer(const uint8_t *tx_data, uint16_t tx_len,
                               uint8_t *rx_data, uint16_t rx_len)
{
    /* First transmit phase */
    if (tx_data && tx_len > 0) {
        if (!spi_half_duplex_transmit(tx_data, tx_len)) {
            return false;
        }
    }

    /* Small delay before switching to receive
     * External device needs time to prepare response */
    {
        volatile uint32_t delay;
        for (delay = 0; delay < 100; delay++) {
            __asm__("nop");
        }
    }

    /* Then receive phase */
    if (rx_data && rx_len > 0) {
        if (!spi_half_duplex_receive(rx_data, rx_len)) {
            return false;
        }
    }

    return true;
}

/* ======================================================================== */
/* Asynchronous Transfer (Blocking fallback implementation)                  */
/* ======================================================================== */
/* TODO: Replace with true interrupt/DMA-driven async when needed.
 * For now these are implemented as blocking calls with callback. */

bool spi_transfer_async(const spi_transfer_t *transfer, spi_callback_t callback)
{
    if (!s_spi.initialized || transfer == NULL || callback == NULL) {
        return false;
    }

    if (s_spi.async_active) {
        return false;  /* Transfer already in progress */
    }

    if (transfer->length == 0 || transfer->length > SPI_MAX_TRANSFER_SIZE) {
        return false;
    }

    /* Store transfer parameters */
    s_spi.async_tx_data = transfer->tx_data;
    s_spi.async_rx_data = transfer->rx_data;
    s_spi.async_remaining = transfer->length;
    s_spi.async_cb = callback;
    s_spi.async_active = true;

    /* Pull CSN low */
    spi_csn_low();

    /* Execute synchronous transfer (placeholder) */
    {
        uint16_t i;
        for (i = 0; i < transfer->length; i++) {
            uint8_t tx_val = (s_spi.async_tx_data != NULL) ?
                              s_spi.async_tx_data[i] : 0xFF;

            while (!(SPI1_REG->SR & SPI_SR_TXE_Msk));
            SPI1_REG->DR = tx_val;

            while (!(SPI1_REG->SR & SPI_SR_RXNE_Msk));
            uint8_t rx_val = (uint8_t)SPI1_REG->DR;

            if (s_spi.async_rx_data != NULL) {
                s_spi.async_rx_data[i] = rx_val;
            }
        }
    }

    while (SPI1_REG->SR & SPI_SR_BSY_Msk);

    /* Release CSN */
    spi_csn_high();

    /* Complete transfer */
    s_spi.async_remaining = 0;
    s_spi.async_active = false;

    /* Call completion callback */
    callback();

    return true;
}

bool spi_is_complete(void)
{
    return !s_spi.async_active;
}

void spi_abort(void)
{
    if (s_spi.async_active) {
        spi_csn_high();
        s_spi.async_remaining = 0;
        s_spi.async_active = false;
    }
}

/* ======================================================================== */
/* SPI Interrupt Handler (for future interrupt-driven transfers)              */
/* ======================================================================== */

void SPI1_IRQHandler(void)
{
    if (SPI1_REG->SR & SPI_SR_RXNE_Msk) {
        (void)SPI1_REG->DR;
    }

    if (SPI1_REG->SR & SPI_SR_TXE_Msk) {
        /* TX buffer empty - future: send next byte */
    }
}
