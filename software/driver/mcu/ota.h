/*
 * ota.h - OTA (Over-The-Air) Upgrade Module for WFOC
 *
 * Implements firmware upgrade functionality via:
 *   - UART (serial bootloader protocol)
 *   - SPI (reserved for future wireless module)
 *
 * Features:
 *   - Flash-based download (no large RAM buffer needed)
 *   - Firmware integrity verification (CRC-32)
 *   - OTA metadata for bootloader validation
 *   - Progress tracking
 *   - Rollback capability
 *
 * OTA Protocol (UART):
 *   - Packet format: [CMD][LEN][DATA...][CRC]
 *   - Commands: START, DATA, END, VERIFY, ABORT
 *   - Start packet contains: size, CRC, version info
 *   - Data packets are up to 256 bytes each
 *   - End packet triggers verification and reboot
 *
 * Flash Layout for OTA:
 *   - 0x0000 - 0x0FFF:   Bootloader (4KB)
 *   - 0x1000 - 0x6FFF:   Application Image (24KB)
 *   - 0x7000 - 0x7BFF:   EEPROM emulation (3KB, config storage)
 *   - 0x7C00 - 0x7FFF:   OTA Metadata (1KB, status & CRC)
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef OTA_H
#define OTA_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* OTA Protocol Definitions                                                  */
/* ======================================================================== */

/* OTA command codes */
typedef enum {
    OTA_CMD_START     = 0x01,  /* Start OTA session */
    OTA_CMD_DATA      = 0x02,  /* Data packet */
    OTA_CMD_END       = 0x03,  /* End OTA session */
    OTA_CMD_VERIFY    = 0x04,  /* Verify downloaded firmware */
    OTA_CMD_ABORT     = 0x05,  /* Abort OTA session */
    OTA_CMD_GET_INFO  = 0x06,  /* Get OTA status information */
    OTA_CMD_ACK       = 0x80,  /* Acknowledgment */
    OTA_CMD_NAK       = 0x81,  /* Negative acknowledgment */
    OTA_CMD_PROGRESS  = 0x82,  /* Progress report */
} ota_cmd_t;

/* OTA start packet data structure */
typedef struct {
    uint32_t image_size;       /* Total firmware image size in bytes */
    uint32_t image_crc;        /* CRC-32 of the entire image */
    uint32_t version_major;    /* Major version number */
    uint32_t version_minor;    /* Minor version number */
    uint32_t version_patch;    /* Patch version number */
    uint32_t hardware_version; /* Target hardware version */
} ota_start_data_t;

/* OTA data packet structure */
typedef struct {
    uint16_t sequence;         /* Packet sequence number */
    uint16_t length;           /* Data length (max 256) */
    uint8_t  data[256];        /* Firmware data */
} ota_data_packet_t;

/* OTA status */
typedef enum {
    OTA_STATE_IDLE = 0,       /* No OTA in progress */
    OTA_STATE_STARTING,       /* OTA session starting */
    OTA_STATE_DOWNLOADING,    /* Downloading firmware data */
    OTA_STATE_VERIFYING,      /* Verifying downloaded firmware */
    OTA_STATE_FLASHING,       /* Writing to application area */
    OTA_STATE_COMPLETE,       /* OTA complete, ready to reboot */
    OTA_STATE_ABORTED,        /* OTA aborted */
    OTA_STATE_ERROR,          /* OTA failed */
} ota_state_t;

/* OTA error codes */
typedef enum {
    OTA_ERROR_NONE = 0,
    OTA_ERROR_INVALID_CMD,
    OTA_ERROR_INVALID_SIZE,
    OTA_ERROR_CRC_MISMATCH,
    OTA_ERROR_WRITE_FAILED,
    OTA_ERROR_VERIFY_FAILED,
    OTA_ERROR_TIMEOUT,
    OTA_ERROR_BUSY,
    OTA_ERROR_INVALID_STATE,
} ota_error_t;

/* ======================================================================== */
/* OTA Configuration                                                         */
/* ======================================================================== */

/* Maximum firmware size (must fit in application area) */
#define OTA_MAX_FIRMWARE_SIZE    0x6000UL  /* 24KB */

/* Maximum data packet size */
#define OTA_MAX_PACKET_SIZE      256

/* OTA progress report interval (bytes) */
#define OTA_PROGRESS_INTERVAL    1024

/* OTA timeout (ms) */
#define OTA_TIMEOUT_MS           30000

/* OTA metadata magic number */
#define OTA_METADATA_MAGIC       0x4F54414FUL  /* "OTAF" */

/* OTA metadata address (last page of flash) */
#define OTA_METADATA_ADDR        0x08007C00UL

/* ======================================================================== */
/* OTA Public API                                                            */
/* ======================================================================== */

/* Initialize OTA module. Must be called at startup. */
void ota_init(void);

/* Get current OTA state. */
ota_state_t ota_get_state(void);

/* Get last OTA error code. */
ota_error_t ota_get_error(void);

/* Get OTA progress (0-100 percent). */
uint8_t ota_get_progress(void);

/* Get downloaded firmware size in bytes. */
uint32_t ota_get_downloaded_size(void);

/* Get total firmware size being downloaded. */
uint32_t ota_get_total_size(void);

/* Process an incoming OTA command via UART.
 * data: pointer to command data
 * length: length of data
 * response: buffer for response (can be NULL)
 * response_len: pointer to response length (can be NULL)
 * Returns true if command was processed successfully. */
bool ota_process_command(uint8_t cmd, const uint8_t *data, uint16_t length,
                         uint8_t *response, uint16_t *response_len);

/* Process a raw OTA packet (header + data + CRC).
 * Returns true if packet was processed. */
bool ota_process_packet(const uint8_t *packet, uint16_t length);

/* Abort current OTA session. */
void ota_abort(void);

/* Reboot the system after OTA completion. */
void ota_reboot(void);

/* Check if a valid OTA image exists (for bootloader use).
 * Reads OTA metadata and verifies CRC.
 * Returns true if a valid, verified firmware image exists. */
bool ota_check_valid_image(void);

/* Clear OTA metadata (used by bootloader after successful boot). */
void ota_clear_metadata(void);

/* ======================================================================== */
/* OTA SPI Interface (Future Expansion)                                      */
/* ======================================================================== */

/* Process an SPI OTA transaction (new SPI protocol).
 * This is the main entry point for SPI-based OTA and remote control.
 *
 * SPI Protocol Format:
 *   [CMD][LEN][DATA...][CRC8]
 *   - CMD: OTA_CMD_* command code
 *   - LEN: Number of data bytes
 *   - DATA: Command-specific data
 *   - CRC8: CRC-8 checksum of CMD+LEN+DATA
 *
 * Parameters:
 *   spi_rx_data: Received data from SPI (including CRC)
 *   spi_rx_len:  Length of received data
 *   spi_tx_data: Buffer for response data
 *   spi_tx_len:  Pointer to store response length
 *
 * Returns: true if command was processed successfully */
bool ota_spi_process(const uint8_t *spi_rx_data, uint8_t spi_rx_len,
                      uint8_t *spi_tx_data, uint8_t *spi_tx_len);

/* SPI OTA start - begins OTA session via SPI interface.
 * Returns true if successful. */
bool ota_spi_start(const ota_start_data_t *start_data);

/* SPI OTA data - receives firmware data via SPI.
 * Returns true if successful. */
bool ota_spi_data(const uint8_t *data, uint16_t length);

/* SPI OTA end - finishes OTA session via SPI.
 * Returns true if verification passes. */
bool ota_spi_end(void);

#ifdef __cplusplus
}
#endif

#endif /* OTA_H */
