/*
 * ota.c - OTA Upgrade Module Implementation for WFOC
 *
 * Implements firmware upgrade functionality via UART and SPI.
 * Downloads firmware directly into application flash area using
 * a small RAM staging buffer for page-by-page programming.
 *
 * OTA Flow:
 *   1. HOST sends OTA_CMD_START with firmware metadata
 *   2. HOST sends OTA_CMD_DATA packets (up to 256 bytes each)
 *   3. Each data packet is written to flash incrementally
 *   4. HOST sends OTA_CMD_END to finish download
 *   5. Device verifies CRC and validates firmware
 *   6. Device reboots to run new firmware
 *
 * Memory Layout:
 *   - Bootloader:    0x0000 - 0x0FFF (4KB)
 *   - Application:   0x1000 - 0x6FFF (24KB)
 *   - EEPROM:        0x7000 - 0x7BFF (3KB, config storage)
 *   - OTA metadata:   0x7C00 - 0x7FFF (1KB, status & CRC)
 *
 * OTA Metadata Area (last page of flash):
 *   - Magic number (0x4F54414F) identifies valid OTA image
 *   - Image CRC-32 for integrity verification
 *   - Image size and version info
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include "driver/mcu/ota.h"
#include "driver/mcu/flash.h"
#include "config/hwconf.h"
#include <string.h>

/* ======================================================================== */
/* OTA Metadata Area Definition                                               */
/* ======================================================================== */

/* OTA metadata stored at the last page of flash */
#define OTA_METADATA_ADDR      0x08007C00UL  /* Last 1KB page */
#define OTA_METADATA_MAGIC     0x4F54414FUL  /* "OTAF" in ASCII */

/* OTA metadata structure (stored at OTA_METADATA_ADDR) */
typedef struct {
    uint32_t magic;          /* Magic number to identify valid OTA image */
    uint32_t image_size;     /* Total firmware image size in bytes */
    uint32_t image_crc;      /* CRC-32 of the entire image */
    uint32_t version_major;
    uint32_t version_minor;
    uint32_t version_patch;
    uint32_t hardware_version;
    uint32_t download_size;  /* Number of bytes successfully downloaded */
    uint32_t state;          /* OTA state when metadata was saved */
    uint32_t reserved[4];    /* Reserved for future use */
} ota_metadata_t;

/* Staging buffer for flash programming (256 bytes = one max packet) */
#define OTA_STAGING_SIZE       256

/* ======================================================================== */
/* OTA Internal State                                                         */
/* ======================================================================== */

typedef struct {
    ota_state_t state;
    ota_error_t last_error;
    uint32_t total_size;
    uint32_t downloaded_size;
    uint32_t flash_size;
    uint32_t expected_crc;
    uint32_t calculated_crc;
    uint16_t last_sequence;
    uint8_t  progress;
    uint32_t last_progress_report;

    /* Firmware version information */
    uint32_t version_major;
    uint32_t version_minor;
    uint32_t version_patch;
    uint32_t hw_version;

    /* Flash write state */
    uint32_t flash_addr;     /* Current write address in application area */
    uint32_t page_addr;       /* Current page being written */
    uint16_t page_offset;     /* Offset within current page */
    bool     page_erased;     /* Whether current page has been erased */

    /* Flags */
    bool initialized;
    bool crc_valid;
} ota_internal_t;

static ota_internal_t s_ota;

/* Staging buffer in RAM for flash programming */
static uint8_t s_staging[OTA_STAGING_SIZE];

/* ======================================================================== */
/* Helper Functions                                                           */
/* ======================================================================== */

static void ota_reset_state(void)
{
    s_ota.state = OTA_STATE_IDLE;
    s_ota.last_error = OTA_ERROR_NONE;
    s_ota.total_size = 0;
    s_ota.downloaded_size = 0;
    s_ota.flash_size = 0;
    s_ota.expected_crc = 0;
    s_ota.calculated_crc = 0;
    s_ota.last_sequence = 0;
    s_ota.progress = 0;
    s_ota.last_progress_report = 0;
    s_ota.version_major = 0;
    s_ota.version_minor = 0;
    s_ota.version_patch = 0;
    s_ota.hw_version = 0;
    s_ota.crc_valid = false;
    s_ota.flash_addr = APP_START_ADDR;
    s_ota.page_addr = APP_START_ADDR;
    s_ota.page_offset = 0;
    s_ota.page_erased = false;
    
    /* Clear staging buffer */
    memset(s_staging, 0, sizeof(s_staging));
}

static void ota_set_error(ota_error_t error)
{
    s_ota.last_error = error;
    s_ota.state = OTA_STATE_ERROR;
}

/* Erase the next page in the application area if needed */
static bool ota_erase_next_page(void)
{
    uint32_t page_addr;
    
    /* Calculate page boundary */
    page_addr = APP_START_ADDR + 
        ((s_ota.downloaded_size / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE);
    
    if (page_addr != s_ota.page_addr) {
        /* We've moved to a new page */
        s_ota.page_addr = page_addr;
        s_ota.page_erased = false;
        s_ota.page_offset = 0;
    }
    
    if (!s_ota.page_erased) {
        if (!flash_erase_page(s_ota.page_addr)) {
            return false;
        }
        s_ota.page_erased = true;
    }
    
    return true;
}

/* Write a chunk of data to flash at the current position */
static bool ota_write_chunk(const uint8_t *data, uint16_t length)
{
    uint32_t addr;
    uint16_t i;
    
    addr = APP_START_ADDR + s_ota.downloaded_size;
    
    /* Write word-by-word */
    for (i = 0; i < length; i += 4) {
        uint32_t word = 0;
        uint8_t *p = (uint8_t *)&word;
        
        for (uint8_t j = 0; j < 4 && (i + j) < length; j++) {
            p[j] = data[i + j];
        }
        
        if (!flash_program_word(addr + i, word)) {
            return false;
        }
    }
    
    return true;
}

/* Save OTA metadata to the metadata page */
static bool ota_save_metadata(void)
{
    ota_metadata_t meta;
    uint32_t *src;
    int i;
    
    /* Fill metadata structure */
    meta.magic = OTA_METADATA_MAGIC;
    meta.image_size = s_ota.total_size;
    meta.image_crc = s_ota.expected_crc;
    meta.version_major = s_ota.version_major;
    meta.version_minor = s_ota.version_minor;
    meta.version_patch = s_ota.version_patch;
    meta.hardware_version = s_ota.hw_version;
    meta.download_size = s_ota.downloaded_size;
    meta.state = (uint32_t)s_ota.state;
    
    /* Erase metadata page */
    if (!flash_erase_page(OTA_METADATA_ADDR)) {
        return false;
    }
    
    /* Write metadata */
    src = (uint32_t *)&meta;
    for (i = 0; i < (int)sizeof(ota_metadata_t) / 4; i++) {
        if (!flash_program_word(OTA_METADATA_ADDR + (i * 4), src[i])) {
            return false;
        }
    }
    
    return true;
}

/* Verify the downloaded firmware by reading back and computing CRC */
static bool ota_verify_firmware(void)
{
    uint32_t crc = 0xFFFFFFFF;
    uint32_t i;
    
    /* Read firmware back from flash and compute CRC */
    for (i = 0; i < s_ota.downloaded_size; i++) {
        uint8_t byte = (uint8_t)flash_read_word(APP_START_ADDR + (i & ~3)) >> 
            ((i & 3) * 8);
        crc = (crc >> 8) ^ s_crc32_table[(crc ^ byte) & 0xFF];
    }
    
    crc ^= 0xFFFFFFFF;
    
    return (crc == s_ota.expected_crc);
}

/* ======================================================================== */
/* OTA Protocol Implementation                                                */
/* ======================================================================== */

void ota_init(void)
{
    ota_reset_state();
    s_ota.initialized = true;
}

ota_state_t ota_get_state(void)
{
    return s_ota.state;
}

ota_error_t ota_get_error(void)
{
    return s_ota.last_error;
}

uint8_t ota_get_progress(void)
{
    return s_ota.progress;
}

uint32_t ota_get_downloaded_size(void)
{
    return s_ota.downloaded_size;
}

uint32_t ota_get_total_size(void)
{
    return s_ota.total_size;
}

/* Handle OTA_CMD_START */
static bool ota_handle_start(const uint8_t *data, uint16_t length,
                              uint8_t *response, uint16_t *response_len)
{
    ota_start_data_t start_data;
    
    if (length < sizeof(ota_start_data_t)) {
        ota_set_error(OTA_ERROR_INVALID_SIZE);
        return false;
    }
    
    /* Parse start data */
    memcpy(&start_data, data, sizeof(ota_start_data_t));
    
    /* Validate */
    if (start_data.image_size == 0 || 
        start_data.image_size > OTA_MAX_FIRMWARE_SIZE) {
        ota_set_error(OTA_ERROR_INVALID_SIZE);
        return false;
    }
    
    /* Reset for new download */
    ota_reset_state();
    
    /* Store metadata */
    s_ota.total_size = start_data.image_size;
    s_ota.expected_crc = start_data.image_crc;
    s_ota.version_major = start_data.version_major;
    s_ota.version_minor = start_data.version_minor;
    s_ota.version_patch = start_data.version_patch;
    s_ota.hw_version = start_data.hardware_version;
    s_ota.state = OTA_STATE_DOWNLOADING;
    s_ota.calculated_crc = 0xFFFFFFFF;
    
    /* Send ACK with max packet size */
    if (response && response_len) {
        response[0] = OTA_CMD_ACK;
        response[1] = OTA_MAX_PACKET_SIZE & 0xFF;
        response[2] = (OTA_MAX_PACKET_SIZE >> 8) & 0xFF;
        *response_len = 3;
    }
    
    return true;
}

/* Handle OTA_CMD_DATA */
static bool ota_handle_data(const uint8_t *data, uint16_t length,
                             uint8_t *response, uint16_t *response_len)
{
    uint16_t seq, pkt_len;
    uint8_t *pkt_data;
    uint32_t remaining;
    
    if (s_ota.state != OTA_STATE_DOWNLOADING) {
        ota_set_error(OTA_ERROR_INVALID_STATE);
        return false;
    }
    
    if (length < 4) {
        ota_set_error(OTA_ERROR_INVALID_SIZE);
        return false;
    }
    
    /* Parse packet header */
    seq = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    pkt_len = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    pkt_data = (uint8_t *)&data[4];
    
    if (length < 4 + pkt_len) {
        ota_set_error(OTA_ERROR_INVALID_SIZE);
        return false;
    }
    
    /* Check sequence number */
    if (seq != s_ota.last_sequence) {
        /* Allow some out-of-order, but not duplicate */
        if (seq < s_ota.last_sequence) {
            /* Duplicate or old packet - send ACK but don't process */
            if (response && response_len) {
                response[0] = OTA_CMD_ACK;
                response[1] = seq & 0xFF;
                response[2] = (seq >> 8) & 0xFF;
                *response_len = 3;
            }
            return true;
        }
    }
    
    /* Validate packet length */
    if (pkt_len > OTA_MAX_PACKET_SIZE) {
        ota_set_error(OTA_ERROR_INVALID_SIZE);
        return false;
    }
    
    /* Check buffer overflow */
    remaining = s_ota.total_size - s_ota.downloaded_size;
    if (pkt_len > remaining) {
        ota_set_error(OTA_ERROR_INVALID_SIZE);
        return false;
    }
    
    /* Erase page if needed */
    if (!ota_erase_next_page()) {
        ota_set_error(OTA_ERROR_WRITE_FAILED);
        return false;
    }
    
    /* Copy data to staging buffer */
    memcpy(s_staging, pkt_data, pkt_len);
    
    /* Write to flash */
    if (!ota_write_chunk(s_staging, pkt_len)) {
        ota_set_error(OTA_ERROR_WRITE_FAILED);
        return false;
    }
    
    /* Update state */
    s_ota.downloaded_size += pkt_len;
    s_ota.last_sequence = seq + 1;
    
    /* Update CRC incrementally (on-the-fly from flash) */
    /* For simplicity, CRC will be verified at end */
    
    /* Update progress */
    if (s_ota.total_size > 0) {
        s_ota.progress = (uint8_t)((s_ota.downloaded_size * 100) / s_ota.total_size);
    }
    
    /* Send ACK */
    if (response && response_len) {
        response[0] = OTA_CMD_ACK;
        response[1] = seq & 0xFF;
        response[2] = (seq >> 8) & 0xFF;
        *response_len = 3;
    }
    
    return true;
}

/* Handle OTA_CMD_END */
static bool ota_handle_end(const uint8_t *data, uint16_t length,
                            uint8_t *response, uint16_t *response_len)
{
    (void)data;
    (void)length;
    
    if (s_ota.state != OTA_STATE_DOWNLOADING) {
        ota_set_error(OTA_ERROR_INVALID_STATE);
        return false;
    }
    
    s_ota.state = OTA_STATE_VERIFYING;
    
    /* Verify CRC by reading back firmware */
    if (!ota_verify_firmware()) {
        ota_set_error(OTA_ERROR_CRC_MISMATCH);
        return false;
    }
    
    s_ota.crc_valid = true;
    
    /* Save OTA metadata (marks firmware as valid) */
    if (!ota_save_metadata()) {
        ota_set_error(OTA_ERROR_WRITE_FAILED);
        return false;
    }
    
    s_ota.state = OTA_STATE_COMPLETE;
    
    /* Send final ACK */
    if (response && response_len) {
        response[0] = OTA_CMD_ACK;
        response[1] = 0x00;  /* Success */
        response[2] = 0x00;
        *response_len = 3;
    }
    
    return true;
}

/* Handle OTA_CMD_ABORT */
static bool ota_handle_abort(const uint8_t *data, uint16_t length,
                              uint8_t *response, uint16_t *response_len)
{
    (void)data;
    (void)length;
    
    ota_reset_state();
    s_ota.state = OTA_STATE_ABORTED;
    
    if (response && response_len) {
        response[0] = OTA_CMD_ACK;
        response[1] = 0x00;
        *response_len = 2;
    }
    
    return true;
}

/* Handle OTA_CMD_VERIFY */
static bool ota_handle_verify(const uint8_t *data, uint16_t length,
                               uint8_t *response, uint16_t *response_len)
{
    (void)data;
    (void)length;
    
    if (s_ota.state == OTA_STATE_COMPLETE && s_ota.crc_valid) {
        if (response && response_len) {
            response[0] = OTA_CMD_ACK;
            response[1] = 0x00;  /* Verified */
            *response_len = 2;
        }
        return true;
    }
    
    if (response && response_len) {
        response[0] = OTA_CMD_NAK;
        response[1] = 0x01;  /* Not verified */
        *response_len = 2;
    }
    return false;
}

/* Handle OTA_CMD_GET_INFO */
static bool ota_handle_get_info(const uint8_t *data, uint16_t length,
                                 uint8_t *response, uint16_t *response_len)
{
    (void)data;
    (void)length;
    
    if (response && response_len) {
        uint8_t i = 0;
        
        response[i++] = OTA_CMD_ACK;
        response[i++] = (uint8_t)s_ota.state;
        response[i++] = (uint8_t)s_ota.progress;
        response[i++] = (uint8_t)(s_ota.total_size & 0xFF);
        response[i++] = (uint8_t)((s_ota.total_size >> 8) & 0xFF);
        response[i++] = (uint8_t)((s_ota.total_size >> 16) & 0xFF);
        response[i++] = (uint8_t)((s_ota.total_size >> 24) & 0xFF);
        response[i++] = (uint8_t)(s_ota.downloaded_size & 0xFF);
        response[i++] = (uint8_t)((s_ota.downloaded_size >> 8) & 0xFF);
        response[i++] = (uint8_t)((s_ota.downloaded_size >> 16) & 0xFF);
        response[i++] = (uint8_t)((s_ota.downloaded_size >> 24) & 0xFF);
        
        *response_len = i;
    }
    
    return true;
}

/* ======================================================================== */
/* Public Command Processing                                                 */
/* ======================================================================== */

bool ota_process_command(uint8_t cmd, const uint8_t *data, uint16_t length,
                         uint8_t *response, uint16_t *response_len)
{
    if (!s_ota.initialized) {
        return false;
    }
    
    switch (cmd) {
    case OTA_CMD_START:
        return ota_handle_start(data, length, response, response_len);
        
    case OTA_CMD_DATA:
        return ota_handle_data(data, length, response, response_len);
        
    case OTA_CMD_END:
        return ota_handle_end(data, length, response, response_len);
        
    case OTA_CMD_ABORT:
        return ota_handle_abort(data, length, response, response_len);
        
    case OTA_CMD_VERIFY:
        return ota_handle_verify(data, length, response, response_len);
        
    case OTA_CMD_GET_INFO:
        return ota_handle_get_info(data, length, response, response_len);
        
    default:
        ota_set_error(OTA_ERROR_INVALID_CMD);
        return false;
    }
}

/* Process a raw OTA packet with CRC validation */
bool ota_process_packet(const uint8_t *packet, uint16_t length)
{
    uint8_t cmd;
    uint16_t data_len;
    uint32_t calc_crc, recv_crc;
    
    if (length < 5) {
        return false;
    }
    
    cmd = packet[0];
    data_len = (uint16_t)packet[1] | ((uint16_t)packet[2] << 8);
    
    if (length < 3 + data_len + 4) {
        return false;
    }
    
    /* Validate CRC (last 4 bytes) */
    recv_crc = (uint32_t)packet[3 + data_len] | 
               ((uint32_t)packet[3 + data_len + 1] << 8) |
               ((uint32_t)packet[3 + data_len + 2] << 16) |
               ((uint32_t)packet[3 + data_len + 3] << 24);
    
    calc_crc = crc32_calculate(packet, 3 + data_len);
    
    if (calc_crc != recv_crc) {
        ota_set_error(OTA_ERROR_CRC_MISMATCH);
        return false;
    }
    
    /* Process the command */
    return ota_process_command(cmd, &packet[3], data_len, NULL, NULL);
}

void ota_abort(void)
{
    if (s_ota.state != OTA_STATE_IDLE) {
        ota_reset_state();
        s_ota.state = OTA_STATE_ABORTED;
    }
}

void ota_reboot(void)
{
    /* Software reset */
    /* On CIU32F003x5, use the system reset request */
    NVIC_SystemReset();
}

/* Check if a valid OTA image exists (for bootloader use) */
bool ota_check_valid_image(void)
{
    ota_metadata_t meta;
    uint32_t *src;
    uint32_t i;
    
    /* Read metadata from flash */
    src = (uint32_t *)&meta;
    for (i = 0; i < (uint32_t)sizeof(ota_metadata_t) / 4; i++) {
        src[i] = flash_read_word(OTA_METADATA_ADDR + (i * 4));
    }
    
    /* Check magic number */
    if (meta.magic != OTA_METADATA_MAGIC) {
        return false;
    }
    
    /* Check state */
    if (meta.state != (uint32_t)OTA_STATE_COMPLETE) {
        return false;
    }
    
    /* Verify the image by computing CRC */
    {
        uint32_t crc = 0xFFFFFFFF;
        uint32_t j;
        
        for (j = 0; j < meta.image_size; j++) {
            uint32_t word = flash_read_word(APP_START_ADDR + (j & ~3));
            uint8_t byte = (uint8_t)(word >> ((j & 3) * 8));
            crc = (crc >> 8) ^ s_crc32_table[(crc ^ byte) & 0xFF];
        }
        crc ^= 0xFFFFFFFF;
        
        return (crc == meta.image_crc);
    }
}

/* Clear OTA metadata (used by bootloader after successful boot) */
void ota_clear_metadata(void)
{
    flash_erase_page(OTA_METADATA_ADDR);
}

/* ======================================================================== */
/* SPI OTA Interface (for future wireless module integration)                */
/* ======================================================================== */

/* SPI OTA protocol:
 *   The external module (e.g., nRF24L01+ or WiFi module) communicates
 *   with the WFOC controller via SPI. The protocol is designed to be
 *   simple and robust.
 *
 *   SPI Transaction Format:
 *     - Master (external module) initiates with CSN low
 *     - First byte: Command (OTA_CMD_*)
 *     - Next bytes: Data (length depends on command)
 *     - Last byte: CRC-8 of all previous bytes
 *     - Master releases CSN after transaction
 *
 *   Commands:
 *     - OTA_CMD_START:     Start OTA session
 *     - OTA_CMD_DATA:      Send firmware data
 *     - OTA_CMD_END:       End OTA session
 *     - OTA_CMD_VERIFY:    Request firmware verification
 *     - OTA_CMD_GET_INFO:  Get OTA status
 *     - OTA_CMD_ABORT:     Abort OTA session
 *
 *   Each command is acknowledged with an ACK/NAK response.
 *   The response format:
 *     - First byte: OTA_CMD_ACK or OTA_CMD_NAK
 *     - Second byte: Status (0x00 = success, 0x01 = error)
 *     - Additional bytes: Data (for GET_INFO) */

/* CRC-8 calculation for SPI packets */
static uint8_t crc8_calculate(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0;
    uint8_t i, j;

    for (i = 0; i < length; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

/* SPI OTA command handler - processes a complete SPI transaction */
static bool ota_spi_handle_command(uint8_t cmd, const uint8_t *data,
                                    uint8_t length, uint8_t *response,
                                    uint8_t *response_len)
{
    uint16_t resp_len_u16 = 0;
    bool result;

    if (!s_ota.initialized) {
        if (response && response_len) {
            response[0] = OTA_CMD_NAK;
            response[1] = 0x02;  /* Not initialized */
            *response_len = 2;
        }
        return false;
    }

    /* Handle command based on type.
     * The ota_handle_* functions use uint16_t for response length,
     * so we use a local uint16_t variable and truncate to uint8_t
     * afterwards (SPI max packet is 256 bytes). */
    switch (cmd) {
    case OTA_CMD_START:
        result = ota_handle_start(data, length, response, &resp_len_u16);
        break;

    case OTA_CMD_DATA:
        result = ota_handle_data(data, length, response, &resp_len_u16);
        break;

    case OTA_CMD_END:
        result = ota_handle_end(data, length, response, &resp_len_u16);
        break;

    case OTA_CMD_ABORT:
        result = ota_handle_abort(data, length, response, &resp_len_u16);
        break;

    case OTA_CMD_VERIFY:
        result = ota_handle_verify(data, length, response, &resp_len_u16);
        break;

    case OTA_CMD_GET_INFO:
        result = ota_handle_get_info(data, length, response, &resp_len_u16);
        break;

    default:
        /* Unknown command */
        if (response && response_len) {
            response[0] = OTA_CMD_NAK;
            response[1] = 0x03;  /* Invalid command */
            *response_len = 2;
        }
        ota_set_error(OTA_ERROR_INVALID_CMD);
        return false;
    }

    /* Copy response length (truncate to uint8_t for SPI) */
    if (response_len) {
        *response_len = (resp_len_u16 > 255) ? 255 : (uint8_t)resp_len_u16;
    }

    return result;
}

/* Process an SPI OTA transaction
 * This is the main entry point for SPI-based OTA.
 * The external module should call this after receiving an SPI packet.
 *
 * Parameters:
 *   spi_rx_data: Received data from SPI (including CRC)
 *   spi_rx_len:  Length of received data
 *   spi_tx_data: Buffer for response data
 *   spi_tx_len:  Pointer to store response length
 *
 * Returns: true if command was processed successfully */
bool ota_spi_process(const uint8_t *spi_rx_data, uint8_t spi_rx_len,
                      uint8_t *spi_tx_data, uint8_t *spi_tx_len)
{
    uint8_t cmd;
    uint8_t data_len;
    uint8_t calc_crc, recv_crc;

    if (spi_rx_data == NULL || spi_rx_len < 2) {
        if (spi_tx_data && spi_tx_len) {
            spi_tx_data[0] = OTA_CMD_NAK;
            spi_tx_data[1] = 0x04;  /* Invalid packet */
            *spi_tx_len = 2;
        }
        return false;
    }

    /* Parse packet: [CMD][LEN][DATA...][CRC] */
    cmd = spi_rx_data[0];
    data_len = spi_rx_data[1];

    /* Validate packet length */
    if (spi_rx_len < 2 + data_len + 1) {
        if (spi_tx_data && spi_tx_len) {
            spi_tx_data[0] = OTA_CMD_NAK;
            spi_tx_data[1] = 0x04;  /* Invalid packet */
            *spi_tx_len = 2;
        }
        return false;
    }

    /* Validate CRC (last byte) */
    recv_crc = spi_rx_data[2 + data_len];
    calc_crc = crc8_calculate(spi_rx_data, 2 + data_len);

    if (calc_crc != recv_crc) {
        /* CRC mismatch */
        if (spi_tx_data && spi_tx_len) {
            spi_tx_data[0] = OTA_CMD_NAK;
            spi_tx_data[1] = 0x05;  /* CRC error */
            *spi_tx_len = 2;
        }
        ota_set_error(OTA_ERROR_CRC_MISMATCH);
        return false;
    }

    /* Process the command */
    return ota_spi_handle_command(cmd, &spi_rx_data[2], data_len,
                                   spi_tx_data, spi_tx_len);
}

/* SPI OTA start - begins OTA session via SPI interface (legacy) */
bool ota_spi_start(const ota_start_data_t *start_data)
{
    if (!start_data) {
        ota_set_error(OTA_ERROR_INVALID_SIZE);
        return false;
    }

    if (start_data->image_size == 0 ||
        start_data->image_size > OTA_MAX_FIRMWARE_SIZE) {
        ota_set_error(OTA_ERROR_INVALID_SIZE);
        return false;
    }

    /* Reset for new download */
    ota_reset_state();

    /* Store metadata */
    s_ota.total_size = start_data->image_size;
    s_ota.expected_crc = start_data->image_crc;
    s_ota.version_major = start_data->version_major;
    s_ota.version_minor = start_data->version_minor;
    s_ota.version_patch = start_data->version_patch;
    s_ota.hw_version = start_data->hardware_version;
    s_ota.state = OTA_STATE_DOWNLOADING;
    s_ota.calculated_crc = 0xFFFFFFFF;

    return true;
}

/* SPI OTA data - receives firmware data via SPI (legacy) */
bool ota_spi_data(const uint8_t *data, uint16_t length)
{
    uint32_t remaining;

    if (s_ota.state != OTA_STATE_DOWNLOADING) {
        ota_set_error(OTA_ERROR_INVALID_STATE);
        return false;
    }

    if (!data || length == 0) {
        ota_set_error(OTA_ERROR_INVALID_SIZE);
        return false;
    }

    remaining = s_ota.total_size - s_ota.downloaded_size;
    if (length > remaining) {
        ota_set_error(OTA_ERROR_INVALID_SIZE);
        return false;
    }

    /* Erase page if needed */
    if (!ota_erase_next_page()) {
        ota_set_error(OTA_ERROR_WRITE_FAILED);
        return false;
    }

    /* Write directly to flash */
    memcpy(s_staging, data, length);
    if (!ota_write_chunk(s_staging, length)) {
        ota_set_error(OTA_ERROR_WRITE_FAILED);
        return false;
    }

    /* Update state */
    s_ota.downloaded_size += length;

    /* Update progress */
    if (s_ota.total_size > 0) {
        s_ota.progress = (uint8_t)((s_ota.downloaded_size * 100) / s_ota.total_size);
    }

    return true;
}

/* SPI OTA end - finishes OTA session via SPI (legacy) */
bool ota_spi_end(void)
{
    if (s_ota.state != OTA_STATE_DOWNLOADING) {
        ota_set_error(OTA_ERROR_INVALID_STATE);
        return false;
    }

    s_ota.state = OTA_STATE_VERIFYING;

    /* Verify CRC by reading back firmware */
    if (!ota_verify_firmware()) {
        ota_set_error(OTA_ERROR_CRC_MISMATCH);
        return false;
    }

    s_ota.crc_valid = true;

    /* Save OTA metadata */
    if (!ota_save_metadata()) {
        ota_set_error(OTA_ERROR_WRITE_FAILED);
        return false;
    }

    s_ota.state = OTA_STATE_COMPLETE;
    return true;
}

/* ======================================================================== */
/* SPI Remote Control Protocol (future expansion)                            */
/* ======================================================================== */
/*
 * In the future, the SPI interface can be used for:
 *   1. Remote control receiver (e.g., 2.4GHz RC protocol)
 *   2. Wireless OTA (e.g., WiFi/BLE module)
 *   3. Telemetry streaming to external devices
 *
 * The SPI protocol should support:
 *   - Configuration read/write
 *   - Real-time telemetry streaming
 *   - Motor control commands
 *   - Firmware upgrade
 *
 * This framework is ready for integration with external SPI devices.
 */
