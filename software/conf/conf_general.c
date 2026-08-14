/*
 * conf_general.c - Global Configuration Manager
 *
 * Holds the live configuration instances and the flash persistence layer.
 * Uses the Flash EEPROM emulation driver (flash.h) for parameter storage.
 *
 * Storage strategy:
 *   - mc_configuration_t and app_configuration_t are stored in separate
 *     EEPROM slots with independent wear-leveling (2 pages each).
 *   - Each entry has a magic number + version + CRC-32 for data integrity.
 *   - Maximum data size: 960 bytes per structure (limited by EEPROM_DATA_MAX_SIZE).
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include "conf/conf_general.h"
#include "driver/mcu/flash.h"
#include <string.h>

/* ======================================================================== */
/* Global Configuration Instances                                            */
/* ======================================================================== */
mc_configuration_t  mc_conf;
app_configuration_t app_conf;

/* ======================================================================== */
/* Internal: Magic numbers for configuration validation                       */
/* ======================================================================== */
#define CONF_MAGIC_MC   0x57464F43UL   /* "WFOC" in ASCII */
#define CONF_MAGIC_APP  0x41505031UL   /* "APP1" in ASCII */

/* Configuration storage header */
typedef struct {
    uint32_t magic;       /* Magic number to identify data type */
    uint16_t version;     /* Structure version for future migration */
    uint16_t length;      /* Payload length */
    uint32_t crc;         /* CRC-32 of payload */
} conf_storage_header_t;

/* ======================================================================== */
/* Initialisation                                                            */
/* ======================================================================== */
void conf_general_init(void)
{
    /* 1. Conservative defaults for this hardware. */
    conf_default_mc(&mc_conf);
    conf_default_app(&app_conf);

    /* 2. Attempt to load a persisted configuration from flash.
     *    The EEPROM driver handles wear-leveling and CRC validation.
     *    EEPROM must be initialized before this is called. */
    {
        mc_configuration_t stored_mc;
        app_configuration_t stored_app;

        if (conf_general_read_mc_conf(&stored_mc)) {
            mc_conf = stored_mc;
        }
        if (conf_general_read_app_conf(&stored_app)) {
            app_conf = stored_app;
        }
    }
}

/* ======================================================================== */
/* Persistence Implementation                                                  */
/* ======================================================================== */

void conf_general_save(void)
{
    /* Save both configurations to Flash EEPROM (separate slots) */
    (void)conf_general_write_mc_conf(&mc_conf);
    (void)conf_general_write_app_conf(&app_conf);
}

bool conf_general_read_mc_conf(mc_configuration_t *conf)
{
    conf_storage_header_t header;
    uint8_t buffer[EEPROM_DATA_MAX_SIZE];
    uint16_t actual_length;
    uint32_t data_offset;

    if (sizeof(mc_configuration_t) > EEPROM_DATA_MAX_SIZE) {
        return false;  /* Structure too large for EEPROM */
    }

    /* Read raw data from the MC slot */
    if (!eeprom_read(EEPROM_SLOT_MC, buffer, sizeof(buffer), &actual_length)) {
        return false;
    }

    /* Check minimum length */
    if (actual_length < sizeof(conf_storage_header_t)) {
        return false;
    }

    /* Parse header */
    memcpy(&header, buffer, sizeof(conf_storage_header_t));

    /* Validate magic number */
    if (header.magic != CONF_MAGIC_MC) {
        return false;
    }

    /* Validate length */
    if (header.length != sizeof(mc_configuration_t)) {
        return false;
    }

    /* Check if there's enough data */
    data_offset = sizeof(conf_storage_header_t);
    if (actual_length < data_offset + header.length) {
        return false;
    }

    /* Verify CRC */
    {
        uint32_t calc_crc = crc32_calculate(
            &buffer[data_offset], header.length);
        if (calc_crc != header.crc) {
            return false;
        }
    }

    /* Copy data to output */
    memcpy(conf, &buffer[data_offset], header.length);
    return true;
}

bool conf_general_read_app_conf(app_configuration_t *conf)
{
    conf_storage_header_t header;
    uint8_t buffer[EEPROM_DATA_MAX_SIZE];
    uint16_t actual_length;
    uint32_t data_offset;

    if (sizeof(app_configuration_t) > EEPROM_DATA_MAX_SIZE) {
        return false;
    }

    /* Read raw data from the APP slot */
    if (!eeprom_read(EEPROM_SLOT_APP, buffer, sizeof(buffer), &actual_length)) {
        return false;
    }

    /* Check minimum length */
    if (actual_length < sizeof(conf_storage_header_t)) {
        return false;
    }

    /* Parse header */
    memcpy(&header, buffer, sizeof(conf_storage_header_t));

    /* Validate magic number */
    if (header.magic != CONF_MAGIC_APP) {
        return false;
    }

    /* Validate length */
    if (header.length != sizeof(app_configuration_t)) {
        return false;
    }

    /* Check if there's enough data */
    data_offset = sizeof(conf_storage_header_t);
    if (actual_length < data_offset + header.length) {
        return false;
    }

    /* Verify CRC */
    {
        uint32_t calc_crc = crc32_calculate(
            &buffer[data_offset], header.length);
        if (calc_crc != header.crc) {
            return false;
        }
    }

    /* Copy data to output */
    memcpy(conf, &buffer[data_offset], header.length);
    return true;
}

bool conf_general_write_mc_conf(const mc_configuration_t *conf)
{
    conf_storage_header_t header;
    uint8_t buffer[EEPROM_DATA_MAX_SIZE];
    uint16_t total_length;

    if (conf == NULL) {
        return false;
    }

    if (sizeof(mc_configuration_t) > EEPROM_DATA_MAX_SIZE) {
        return false;
    }

    /* Prepare header */
    header.magic = CONF_MAGIC_MC;
    header.version = 1;
    header.length = (uint16_t)sizeof(mc_configuration_t);
    header.crc = crc32_calculate(
        (const uint8_t *)conf, sizeof(mc_configuration_t));

    /* Assemble buffer: header + payload */
    total_length = sizeof(conf_storage_header_t) + sizeof(mc_configuration_t);
    memcpy(buffer, &header, sizeof(conf_storage_header_t));
    memcpy(&buffer[sizeof(conf_storage_header_t)],
           conf, sizeof(mc_configuration_t));

    /* Write to MC slot (its own wear-leveled pages) */
    if (!eeprom_write(EEPROM_SLOT_MC, buffer, total_length)) {
        return false;
    }

    /* Update live copy */
    if (conf != &mc_conf) {
        mc_conf = *conf;
    }

    return true;
}

bool conf_general_write_app_conf(const app_configuration_t *conf)
{
    conf_storage_header_t header;
    uint8_t buffer[EEPROM_DATA_MAX_SIZE];
    uint16_t total_length;

    if (conf == NULL) {
        return false;
    }

    if (sizeof(app_configuration_t) > EEPROM_DATA_MAX_SIZE) {
        return false;
    }

    /* Prepare header */
    header.magic = CONF_MAGIC_APP;
    header.version = 1;
    header.length = (uint16_t)sizeof(app_configuration_t);
    header.crc = crc32_calculate(
        (const uint8_t *)conf, sizeof(app_configuration_t));

    /* Assemble buffer: header + payload */
    total_length = sizeof(conf_storage_header_t) + sizeof(app_configuration_t);
    memcpy(buffer, &header, sizeof(conf_storage_header_t));
    memcpy(&buffer[sizeof(conf_storage_header_t)],
           conf, sizeof(app_configuration_t));

    /* Write to APP slot (its own wear-leveled pages) */
    if (!eeprom_write(EEPROM_SLOT_APP, buffer, total_length)) {
        return false;
    }

    /* Update live copy */
    if (conf != &app_conf) {
        app_conf = *conf;
    }

    return true;
}
