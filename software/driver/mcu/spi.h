/*
 * spi.h - SPI Driver for WFOC Motor Controller
 *
 * Implements SPI peripheral driver for:
 *   - Remote control receiver (future)
 *   - OTA upgrade interface (future)
 *
 * Pin Mapping (WFOC V1 Board):
 *   - SPI_SCK:  PA2 (SWCLK pin) - can be remapped
 *   - SPI_MOSI: PA3 (shared with ADC_VOL_U)
 *   - SPI_MISO: PA4 (shared with ADC_VOL_V) - currently not used
 *   - SPI_CSN:  PA5 (shared with ADC_VOL_W) - chip select
 *
 * Note: MISO is currently not used as the external hardware
 * connects it to MOSI for half-duplex communication.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* SPI Configuration Constants                                               */
/* ======================================================================== */

/* SPI peripheral base address */
#define SPI_BASE_ADDR           0x40013000UL   /* SPI1 base */

/* SPI pin definitions (initial mapping, may change) */
#define SPI_SCK_PORT            GPIOA
#define SPI_SCK_PIN             2             /* PA2 */
#define SPI_MOSI_PORT           GPIOA
#define SPI_MOSI_PIN            3             /* PA3 */
#define SPI_MISO_PORT           GPIOA
#define SPI_MISO_PIN            4             /* PA4 - not used currently */
#define SPI_CSN_PORT            GPIOA
#define SPI_CSN_PIN             5             /* PA5 - chip select */

/* SPI clock divider options */
#define SPI_CLK_DIV_2           0
#define SPI_CLK_DIV_4           1
#define SPI_CLK_DIV_8           2
#define SPI_CLK_DIV_16          3
#define SPI_CLK_DIV_32          4
#define SPI_CLK_DIV_64          5
#define SPI_CLK_DIV_128         6
#define SPI_CLK_DIV_256         7

/* SPI mode (CPOL/CPHA) */
#define SPI_MODE_0              0   /* CPOL=0, CPHA=0 */
#define SPI_MODE_1              1   /* CPOL=0, CPHA=1 */
#define SPI_MODE_2              2   /* CPOL=1, CPHA=0 */
#define SPI_MODE_3              3   /* CPOL=1, CPHA=1 */

/* Maximum SPI transfer size */
#define SPI_MAX_TRANSFER_SIZE   256

/* ======================================================================== */
/* SPI Transfer Structure                                                   */
/* ======================================================================== */
typedef struct {
    uint8_t  *tx_data;         /* Transmit data buffer (NULL for RX only) */
    uint8_t  *rx_data;         /* Receive data buffer (NULL for TX only) */
    uint16_t  length;          /* Number of bytes to transfer */
    uint8_t   csn_polarity;    /* CSN active level (0=low, 1=high) */
} spi_transfer_t;

/* ======================================================================== */
/* Public API                                                               */
/* ======================================================================== */

/* Initialize SPI peripheral with specified settings.
 * div: clock divider (SPI_CLK_DIV_*)
 * mode: SPI mode (SPI_MODE_*)
 * sbit: bit order (0=MSB first, 1=LSB first)
 * cpol: clock polarity
 * cpha: clock phase */
void spi_init(uint8_t div, uint8_t mode, uint8_t sbit);

/* Deinitialize SPI peripheral and release pins. */
void spi_deinit(void);

/* Start an SPI transfer (blocking).
 * Returns true on success. */
bool spi_transfer(const spi_transfer_t *transfer);

/* Transmit data only (blocking). */
bool spi_transmit(const uint8_t *data, uint16_t length);

/* Receive data only (blocking).
 * tx_val: value to send during receive (usually 0xFF for reading) */
bool spi_receive(uint8_t *data, uint16_t length, uint8_t tx_val);

/* CSN control (manual CSN mode) */
void spi_csn_low(void);
void spi_csn_high(void);

/* ======================================================================== */
/* Half-Duplex SPI Interface                                                  */
/* ======================================================================== */
/*
 * The WFOC board uses half-duplex SPI:
 *   - MOSI (PA3) and MISO (PA4) are connected together externally
 *   - This allows bidirectional communication on a single data line
 *   - Pin modes are dynamically switched between TX and RX
 */

/* Half-duplex transmit: send data on MOSI line only.
 * The MISO line is configured as high-Z during transmit.
 * Returns true if successful. */
bool spi_half_duplex_transmit(const uint8_t *data, uint16_t length);

/* Half-duplex receive: receive data from MISO line only.
 * The MOSI line is configured as high-Z during receive.
 * The external device drives the data line.
 * Returns true if successful. */
bool spi_half_duplex_receive(uint8_t *data, uint16_t length);

/* Full half-duplex transaction: transmit then receive.
 * This is the standard transaction format for half-duplex SPI.
 * The external device listens during transmit phase and
 * responds during receive phase after a brief turnaround delay.
 * Returns true if successful. */
bool spi_half_duplex_transfer(const uint8_t *tx_data, uint16_t tx_len,
                               uint8_t *rx_data, uint16_t rx_len);

/* ======================================================================== */
/* Future Expansion - Interrupt-based SPI (TODO)                             */
/* ======================================================================== */
/* These functions are placeholders for future interrupt/DMA-based SPI
 * implementation to support high-speed OTA and remote control.         */

/* Start interrupt-based SPI transfer.
 * callback: function called when transfer completes
 * Returns true if transfer was started successfully. */
typedef void (*spi_callback_t)(void);
bool spi_transfer_async(const spi_transfer_t *transfer, spi_callback_t callback);

/* Check if async transfer is complete. */
bool spi_is_complete(void);

/* Abort current async transfer. */
void spi_abort(void);

#ifdef __cplusplus
}
#endif

#endif /* SPI_H */
