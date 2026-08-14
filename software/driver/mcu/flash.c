/*
 * flash.c - Flash Driver with EEPROM Emulation for WFOC
 *
 * Implements on-chip Flash operations and a wear-leveled EEPROM
 * emulation layer for parameter persistence.
 *
 * The CIU32F003x5 Flash memory characteristics:
 *   - 32KB total capacity
 *   - 1KB page size (erase granularity)
 *   - 32-bit word programming (write granularity)
 *   - Minimum erase cycles: 10,000 (extended)
 *
 * EEPROM Emulation Strategy (per slot):
 *   - Each slot uses 2 pages alternately for wear-leveling
 *   - Each write goes to the inactive page with incremented sequence
 *   - After writing, the old page is marked as obsolete
 *   - On power-up, the page with the highest valid sequence is read
 *
 * Slots:
 *   - Slot 0 (MC config):   pages @ 0x7000, 0x7400
 *   - Slot 1 (APP config):  pages @ 0x7800, 0x7C00
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include "driver/mcu/flash.h"
#include "config/hwconf.h"
#include <string.h>

/* ======================================================================== */
/* Internal State                                                           */
/* ======================================================================== */

/* Per-slot state for wear-leveling tracking */
typedef struct {
    bool      initialized;
    uint32_t  current_seq;      /* Sequence number of active page */
    uint32_t  active_page;      /* Address of active page */
    uint32_t  inactive_page;    /* Address of inactive page */
    uint16_t  data_length;      /* Length of stored data */
    bool      valid;            /* Whether valid data exists */
} eeprom_slot_state_t;

static eeprom_slot_state_t s_slots[EEPROM_SLOT_MAX];

/* Slot page mapping: pages A and B for each slot */
static const uint32_t s_slot_pages[EEPROM_SLOT_MAX][2] = {
    { EEPROM_SLOT0_PAGE0, EEPROM_SLOT0_PAGE1 },  /* MC config */
    { EEPROM_SLOT1_PAGE0, EEPROM_SLOT1_PAGE1 },  /* APP config */
};

/* ======================================================================== */
/* CRC-32 Implementation                                                    */
/* ======================================================================== */

/* CRC-32 lookup table (IEEE 802.3 polynomial)
 * Externally visible for OTA verification */
const uint32_t s_crc32_table[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA,
    0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91B, 0x97D2D988,
    0x09B64C2B, 0x7EB17CBD, 0xE7B82D09, 0x90BF1D9F,
    0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE,
    0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC,
    0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
    0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C, 0xDBBBB9D6, 0xACBCB980,
    0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116,
    0x21B4F6B5, 0x56B3C423, 0xCFDA9F99, 0xB8DCAE0F,
    0x2002A854, 0x5705A092, 0xCE0C5F78, 0xB90B6EEE,
    0x220A144D, 0x550D04DB, 0xCC047561, 0xBB0345F7,
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A,
    0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818,
    0x7F6A0D6B, 0x086D3D2D, 0x91649C97, 0xE6634C01,
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
    0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C,
    0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2,
    0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
    0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
    0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
    0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086,
    0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90C, 0x29D9C99A, 0xB0D09820, 0xC7D7D8B6,
    0x59B33D15, 0x2EB40D83, 0xB7BD5C39, 0xC0BA6CA0,
    0x4BC0C977, 0x3CC7F9E1, 0xA5CEA85B, 0xD2C998CD,
    0x4CBFCD5B, 0x3BB8FD2D, 0xA2B1AE77, 0xD5B69EE1,
    0x4D02014C, 0x3A0531DA, 0xA30C6060, 0xD40B50F6,
    0x4A6FDC55, 0x3D68EC63, 0xA461BD79, 0xD3668DEF,
    0x56C1D359, 0x21C6E3CF, 0xB8CFB275, 0xCFC882E3,
    0x51A61740, 0x26A127D6, 0xBF0A766C, 0xC80D46FA,
    0x580A5B6B, 0x2F0D2BFD, 0xB604DA40, 0xC103EA4A,
    0x5F679F72, 0x2860AFE4, 0xB168FE5D, 0xC661AEC8
};

uint32_t crc32_calculate(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint32_t i = 0; i < length; i++) {
        uint8_t index = (uint8_t)((crc ^ data[i]) & 0xFF);
        crc = (crc >> 8) ^ s_crc32_table[index];
    }
    
    return crc ^ 0xFFFFFFFF;
}

/* ======================================================================== */
/* Flash Low-Level Operations                                                */
/* ======================================================================== */

void flash_init(void)
{
    /* Flash is ready after reset - no initialization needed */
}

void flash_unlock(void)
{
    /* Unlock Flash by writing the correct key sequence */
    FLASH_REG->KEYR = FLASH_KEY1;
    FLASH_REG->KEYR = FLASH_KEY2;
}

void flash_lock(void)
{
    /* Lock Flash by setting the LOCK bit */
    FLASH_REG->CR |= FLASH_CR_LOCK_BIT;
}

bool flash_erase_page(uint32_t page_addr)
{
    uint32_t page_num;
    
    /* Calculate page number from address */
    page_num = (page_addr - FLASH_BASE_ADDR) / FLASH_PAGE_SIZE;
    
    /* Check alignment */
    if ((page_addr % FLASH_PAGE_SIZE) != 0) {
        return false;
    }
    
    /* Unlock Flash */
    flash_unlock();
    
    /* Set PER bit and page number */
    FLASH_REG->CR |= FLASH_CR_PER_BIT;
    FLASH_REG->AR = page_addr;
    
    /* Start erase */
    FLASH_REG->CR |= FLASH_CR_STRE_BIT;
    
    /* Wait for erase to complete */
    while (FLASH_REG->SR & FLASH_SR_BUSY_BIT) {
        /* spin wait */
    }
    
    /* Check for errors */
    if (FLASH_REG->SR & FLASH_SR_EOP_BIT) {
        FLASH_REG->SR = FLASH_SR_EOP_BIT;  /* clear EOP flag */
    } else {
        FLASH_REG->CR &= ~FLASH_CR_PER_BIT;
        flash_lock();
        return false;
    }
    
    /* Clear PER bit */
    FLASH_REG->CR &= ~FLASH_CR_PER_BIT;
    
    /* Verify erase */
    if (flash_read_word(page_addr) != 0xFFFFFFFF) {
        flash_lock();
        return false;
    }
    
    flash_lock();
    return true;
}

bool flash_program_word(uint32_t addr, uint32_t data)
{
    /* Check alignment */
    if ((addr % 4) != 0) {
        return false;
    }
    
    /* Unlock Flash */
    flash_unlock();
    
    /* Set PG bit */
    FLASH_REG->CR |= FLASH_CR_PG_BIT;
    
    /* Program the word */
    *(volatile uint32_t *)addr = data;
    
    /* Wait for programming to complete */
    while (FLASH_REG->SR & FLASH_SR_BUSY_BIT) {
        /* spin wait */
    }
    
    /* Check for errors */
    if (FLASH_REG->SR & FLASH_SR_EOP_BIT) {
        FLASH_REG->SR = FLASH_SR_EOP_BIT;  /* clear EOP flag */
    } else {
        FLASH_REG->CR &= ~FLASH_CR_PG_BIT;
        flash_lock();
        return false;
    }
    
    /* Clear PG bit */
    FLASH_REG->CR &= ~FLASH_CR_PG_BIT;
    
    /* Verify */
    if (flash_read_word(addr) != data) {
        flash_lock();
        return false;
    }
    
    flash_lock();
    return true;
}

uint32_t flash_read_word(uint32_t addr)
{
    return *(volatile uint32_t *)addr;
}

/* ======================================================================== */
/* EEPROM Emulation Implementation                                           */
/* ======================================================================== */

/* Read header from a page and validate it */
static bool eeprom_read_header(uint32_t page_addr, eeprom_header_t *header)
{
    /* Read header fields (16 bytes = 4 words) */
    uint32_t *dst = (uint32_t *)header;
    for (int i = 0; i < 4; i++) {
        dst[i] = flash_read_word(page_addr + (i * 4));
    }
    
    /* Validate status */
    if (header->status != EEPROM_PAGE_ACTIVE) {
        return false;
    }
    
    /* Validate length */
    if (header->length > EEPROM_DATA_MAX_SIZE) {
        return false;
    }
    
    return true;
}

/* Read data from a page (after header) */
static bool eeprom_read_data(uint32_t page_addr, uint8_t *data, uint16_t length)
{
    uint32_t data_addr = page_addr + sizeof(eeprom_header_t);
    uint16_t i;
    
    for (i = 0; i < length; i += 4) {
        uint32_t word = flash_read_word(data_addr + i);
        uint8_t *p = (uint8_t *)&word;
        
        for (uint8_t j = 0; j < 4 && (i + j) < length; j++) {
            data[i + j] = p[j];
        }
    }
    
    return true;
}

/* Write header and data to a page */
static bool eeprom_write_page(uint32_t page_addr, const eeprom_header_t *header,
                               const uint8_t *data)
{
    uint32_t *src = (uint32_t *)header;
    uint16_t i;
    
    /* Write header (16 bytes = 4 words) */
    for (i = 0; i < 4; i++) {
        if (!flash_program_word(page_addr + (i * 4), src[i])) {
            return false;
        }
    }
    
    /* Write data */
    uint32_t data_addr = page_addr + sizeof(eeprom_header_t);
    uint16_t data_words = (header->length + 3) / 4;
    
    for (i = 0; i < data_words; i++) {
        uint32_t word = 0;
        uint8_t *p = (uint8_t *)&word;
        uint16_t offset = i * 4;
        
        for (uint8_t j = 0; j < 4 && (offset + j) < header->length; j++) {
            p[j] = data[offset + j];
        }
        
        if (!flash_program_word(data_addr + (i * 4), word)) {
            return false;
        }
    }
    
    return true;
}

/* Mark a page as obsolete (write new status and sequence) */
static bool eeprom_mark_obsolete(uint32_t page_addr, uint32_t new_sequence)
{
    eeprom_header_t new_header;
    
    /* Read current header */
    uint32_t *src = (uint32_t *)&new_header;
    for (int i = 0; i < 4; i++) {
        src[i] = flash_read_word(page_addr + (i * 4));
    }
    
    /* Update status and sequence */
    new_header.status = EEPROM_PAGE_OBSOLETE;
    new_header.sequence = new_sequence;
    
    /* Write back updated header */
    for (int i = 0; i < 4; i++) {
        if (!flash_program_word(page_addr + (i * 4), src[i])) {
            return false;
        }
    }
    
    return true;
}

/* Initialize a single slot's state */
static void eeprom_init_slot(eeprom_slot_t slot)
{
    eeprom_slot_state_t *state = &s_slots[slot];
    uint32_t page0_addr = s_slot_pages[slot][0];
    uint32_t page1_addr = s_slot_pages[slot][1];
    bool wear_leveled = (page0_addr != page1_addr);
    eeprom_header_t header0, header1;
    bool page0_valid, page1_valid;

    state->initialized = true;
    state->valid = false;
    state->current_seq = 0;
    state->data_length = 0;
    state->active_page = page0_addr;
    state->inactive_page = page1_addr;

    if (!wear_leveled) {
        /* Single-page slot (e.g., APP config) */
        page0_valid = eeprom_read_header(page0_addr, &header0);

        if (page0_valid) {
            state->current_seq = header0.sequence;
            state->data_length = header0.length;
            state->valid = true;
        } else {
            /* Erase the page to start fresh */
            flash_erase_page(page0_addr);
        }
        return;
    }

    /* Multi-page slot with wear-leveling */
    page0_valid = eeprom_read_header(page0_addr, &header0);
    page1_valid = eeprom_read_header(page1_addr, &header1);

    if (page0_valid && page1_valid) {
        /* Both pages have valid data - use the one with higher sequence */
        if (header0.sequence >= header1.sequence) {
            state->active_page = page0_addr;
            state->inactive_page = page1_addr;
            state->current_seq = header0.sequence;
            state->data_length = header0.length;
        } else {
            state->active_page = page1_addr;
            state->inactive_page = page0_addr;
            state->current_seq = header1.sequence;
            state->data_length = header1.length;
        }
        state->valid = true;
    } else if (page0_valid) {
        /* Only page 0 is valid */
        state->active_page = page0_addr;
        state->inactive_page = page1_addr;
        state->current_seq = header0.sequence;
        state->data_length = header0.length;
        state->valid = true;

        /* Erase page 1 (it's either empty or invalid) */
        flash_erase_page(page1_addr);
    } else if (page1_valid) {
        /* Only page 1 is valid */
        state->active_page = page1_addr;
        state->inactive_page = page0_addr;
        state->current_seq = header1.sequence;
        state->data_length = header1.length;
        state->valid = true;

        /* Erase page 0 (it's either empty or invalid) */
        flash_erase_page(page0_addr);
    } else {
        /* No valid data - erase both pages to start fresh */
        flash_erase_page(page0_addr);
        flash_erase_page(page1_addr);
    }
}

void eeprom_init(void)
{
    for (int slot = 0; slot < EEPROM_SLOT_MAX; slot++) {
        eeprom_init_slot((eeprom_slot_t)slot);
    }
}

bool eeprom_write(eeprom_slot_t slot, const void *data, uint16_t length)
{
    eeprom_slot_state_t *state;
    eeprom_header_t header;
    uint32_t new_active_page, old_active_page;
    bool wear_leveled;

    if (slot >= EEPROM_SLOT_MAX) {
        return false;
    }

    state = &s_slots[slot];

    if (!state->initialized) {
        return false;
    }

    if (data == NULL || length == 0) {
        return false;
    }

    if (length > EEPROM_DATA_MAX_SIZE) {
        return false;
    }

    /* Check if slot uses wear-leveling (different pages) */
    wear_leveled = (s_slot_pages[slot][0] != s_slot_pages[slot][1]);

    if (wear_leveled) {
        /* Wear-leveled slot: alternate between pages */
        old_active_page = state->active_page;
        new_active_page = state->inactive_page;
    } else {
        /* Single-page slot: reuse the same page */
        old_active_page = state->active_page;
        new_active_page = state->active_page;
    }

    /* Prepare header */
    header.status = EEPROM_PAGE_ACTIVE;
    header.length = length;
    header.sequence = state->current_seq + 1;
    header.crc = crc32_calculate((const uint8_t *)data, length);

    /* Erase the new active page first */
    if (!flash_erase_page(new_active_page)) {
        return false;
    }

    /* Write data to the new active page */
    if (!eeprom_write_page(new_active_page, &header, (const uint8_t *)data)) {
        return false;
    }

    /* Mark old page as obsolete (only for wear-leveled slots) */
    if (wear_leveled && old_active_page != new_active_page) {
        if (!eeprom_mark_obsolete(old_active_page, header.sequence)) {
            /* Non-fatal: continue anyway */
        }
    }

    /* Update state */
    if (wear_leveled) {
        state->active_page = new_active_page;
        state->inactive_page = old_active_page;
    }
    state->current_seq = header.sequence;
    state->data_length = length;
    state->valid = true;

    return true;
}

bool eeprom_read(eeprom_slot_t slot, void *data, uint16_t max_length,
                 uint16_t *actual_length)
{
    eeprom_slot_state_t *state;
    eeprom_header_t header;
    
    if (slot >= EEPROM_SLOT_MAX) {
        return false;
    }
    
    state = &s_slots[slot];
    
    if (!state->initialized || !state->valid) {
        return false;
    }
    
    /* Read and verify header */
    if (!eeprom_read_header(state->active_page, &header)) {
        return false;
    }
    
    if (header.length > max_length) {
        return false;
    }
    
    /* Read data */
    if (!eeprom_read_data(state->active_page, (uint8_t *)data, header.length)) {
        return false;
    }
    
    /* Verify CRC */
    {
        uint32_t calc_crc = crc32_calculate((const uint8_t *)data, header.length);
        if (calc_crc != header.crc) {
            return false;
        }
    }
    
    if (actual_length != NULL) {
        *actual_length = header.length;
    }
    
    return true;
}

void eeprom_erase_all(void)
{
    for (int slot = 0; slot < EEPROM_SLOT_MAX; slot++) {
        eeprom_slot_state_t *state = &s_slots[slot];
        uint32_t page0_addr = s_slot_pages[slot][0];
        uint32_t page1_addr = s_slot_pages[slot][1];
        
        if (!state->initialized) {
            continue;
        }
        
        flash_erase_page(page0_addr);
        flash_erase_page(page1_addr);
        
        state->active_page = page0_addr;
        state->inactive_page = page1_addr;
        state->current_seq = 0;
        state->data_length = 0;
        state->valid = false;
    }
}

uint32_t eeprom_get_sequence(eeprom_slot_t slot)
{
    if (slot >= EEPROM_SLOT_MAX) {
        return 0;
    }
    return s_slots[slot].current_seq;
}
