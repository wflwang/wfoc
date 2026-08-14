/*
 * flash.h - Flash Driver for WFOC Motor Controller
 *
 * Implements a software EEPROM emulation layer using on-chip Flash
 * for parameter storage. The CIU32F003x5 has 32KB Flash with 1KB page size.
 *
 * Features:
 *   - Wear-leveling with independent slot pairs (MC config + APP config)
 *   - CRC-32 verification for data integrity
 *   - Atomic read/write operations
 *   - Power-loss safe (write to new page, then mark old obsolete)
 *
 * Flash layout (32KB total):
 *   - Bootloader area: 0x0000 - 0x0FFF (4KB, reserved)
 *   - Application code: 0x1000 - 0x6FFF (24KB)
 *   - EEPROM slot 0 (MC): pages @ 0x7000, 0x7400 (2KB, wear-leveled)
 *   - EEPROM slot 1 (APP): pages @ 0x7800, 0x7C00 (2KB, wear-leveled)
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* Flash Layout Constants                                                   */
/* ======================================================================== */

/* Flash memory map */
#define FLASH_BASE_ADDR        0x08000000UL
#define FLASH_PAGE_SIZE        1024UL     /* 1KB page size */
#define FLASH_TOTAL_SIZE       32768UL    /* 32KB total Flash */

/* Flash memory map - 32KB total (32 pages of 1KB each):
 *   - 0x0000 - 0x0FFF:   Bootloader (4 pages, reserved)
 *   - 0x1000 - 0x6FFF:   Application code (24 pages, 24KB)
 *   - 0x7000 - 0x73FF:   EEPROM Slot 0 - MC config (page 0)
 *   - 0x7400 - 0x77FF:   EEPROM Slot 0 - MC config (page 1, wear-leveled)
 *   - 0x7800 - 0x7BFF:   EEPROM Slot 1 - APP config (page 0)
 *   - 0x7C00 - 0x7FFF:   OTA Metadata (page 1, wear-leveled)
 *
 * NOTE: Slot 0 (MC config) uses 2 pages for wear-leveling
 *       Slot 1 (APP config) uses 1 page at 0x7800 only
 *       OTA metadata uses page at 0x7C00
 *       Total: 4 + 24 + 2 + 1 + 1 = 32 pages! CORRECT! */

/* Application start/end addresses */
#define APP_START_ADDR         0x08001000UL  /* 4KB bootloader reserved */
#define APP_MAX_SIZE           0x6000UL     /* 24KB max application */

/* OTA download buffer (in-RAM staging; application area is 24KB) */
#define OTA_MAX_FIRMWARE_SIZE  0x6000UL

/* EEPROM slot definitions */
/* Slot 0 (MC config): 2 pages for wear-leveling */
#define EEPROM_SLOT0_PAGE0     0x08007000UL   /* MC config page A */
#define EEPROM_SLOT0_PAGE1     0x08007400UL   /* MC config page B */

/* Slot 1 (APP config): 1 page only (rarely written)
 * The second page (0x7C00) is used for OTA metadata instead */
#define EEPROM_SLOT1_PAGE0     0x08007800UL   /* APP config page */
#define EEPROM_SLOT1_PAGE1     0x08007800UL   /* Same page (no wear-leveling) */

/* OTA metadata stored at page 0x7C00 */
#define OTA_METADATA_ADDR      0x08007C00UL

/* Maximum size of data per slot (limited by 1KB page - header overhead) */
#define EEPROM_DATA_MAX_SIZE   960       /* Bytes max per slot payload */

/* Flash keys for unlock/lock */
#define FLASH_KEY1             0x45670123UL
#define FLASH_KEY2             0xCDEF89ABUL

/* ======================================================================== */
/* EEPROM Data Structure                                                    */
/* ======================================================================== */

/* EEPROM page status markers */
#define EEPROM_PAGE_ERASED     0xFFFF    /* Empty/unused page */
#define EEPROM_PAGE_ACTIVE     0x0001    /* Active page with valid data */
#define EEPROM_PAGE_OBSOLETE   0x0002    /* Old page, can be erased */

/* EEPROM storage header (at start of each page) */
typedef struct {
    uint16_t status;       /* Page status marker */
    uint16_t length;       /* Length of data payload */
    uint32_t sequence;     /* Sequence number for wear-leveling */
    uint32_t crc;          /* CRC-32 of data payload */
} eeprom_header_t;

/* ======================================================================== */
/* Public API                                                               */
/* ======================================================================== */

/* Initialize Flash driver. Must be called before any other flash function. */
void flash_init(void);

/* Unlock Flash for erase/program operations. */
void flash_unlock(void);

/* Lock Flash to prevent accidental writes. */
void flash_lock(void);

/* Erase a single Flash page. Returns true on success. */
bool flash_erase_page(uint32_t page_addr);

/* Program a word (4 bytes) to Flash. Returns true on success.
 * addr must be word-aligned. */
bool flash_program_word(uint32_t addr, uint32_t data);

/* Read a word from Flash. */
uint32_t flash_read_word(uint32_t addr);

/* ======================================================================== */
/* EEPROM Emulation API (Slot-based)                                        */
/* ======================================================================== */

/* EEPROM slot IDs (used with slot-based API) */
typedef enum {
    EEPROM_SLOT_MC  = 0,  /* Motor controller configuration */
    EEPROM_SLOT_APP = 1,  /* Application configuration */
    EEPROM_SLOT_MAX
} eeprom_slot_t;

/* Initialize all EEPROM slots. Checks for valid pages and performs
 * garbage collection if needed. Call after flash_init() but before
 * any slot read/write. */
void eeprom_init(void);

/* Write data to a specific EEPROM slot.
 * The slot's pages are wear-leveled automatically.
 * Returns true on success. */
bool eeprom_write(eeprom_slot_t slot, const void *data, uint16_t length);

/* Read data from a specific EEPROM slot.
 * Returns true if valid data was found.
 * Returns false if no valid data exists or CRC check fails. */
bool eeprom_read(eeprom_slot_t slot, void *data, uint16_t max_length,
                 uint16_t *actual_length);

/* Erase all EEPROM slots (factory reset). */
void eeprom_erase_all(void);

/* Get the current EEPROM sequence number for a slot (wear-leveling status). */
uint32_t eeprom_get_sequence(eeprom_slot_t slot);

/* ======================================================================== */
/* CRC Utility                                                              */
/* ======================================================================== */

/* CRC-32 lookup table (IEEE 802.3 polynomial).
 * Exposed for use by modules that need per-byte CRC computation
 * without calling crc32_calculate() (e.g., verifying a flashed image). */
extern const uint32_t s_crc32_table[256];

/* Calculate CRC-32 for a data buffer. */
uint32_t crc32_calculate(const uint8_t *data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_H */
