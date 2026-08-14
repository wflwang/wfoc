/*
 * packet.c - VESC-style Packet Framing
 *
 * State-machine driven receive path and a blocking transmit path with
 * CRC-16/CCITT protection. Designed to be driven byte-by-byte from a UART
 * receive interrupt; the assembled command is dispatched via the installed
 * rx_callback.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include "packet/packet.h"
#include <string.h>

/* ======================================================================== */
/* Single Global Handler Instance                                            */
/* ======================================================================== */
/* One UART => one handler. Keeping the instance file-scope matches the
 * board's single-stream topology and avoids heap allocation. */
static packet_t s_pkt;

/* Reused for the transmit path. packet_send runs in thread context only. */
static uint8_t s_tx_payload[PACKET_MAX_PAYLOAD];

/* ======================================================================== */
/* CRC-16 / CCITT (poly 0x1021, init 0xFFFF, no final XOR)                   */
/* ======================================================================== */
uint16_t packet_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint32_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)(((uint16_t)data[i]) << 8);
        for (uint8_t b = 0U; b < 8U; b++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ======================================================================== */
/* Initialisation                                                            */
/* ======================================================================== */
void packet_init(packet_rx_fn rx_callback, packet_tx_byte_fn tx_byte)
{
    s_pkt.state        = PACKET_STATE_WAIT_START;
    s_pkt.payload_len  = 0U;
    s_pkt.payload_idx  = 0U;
    s_pkt.crc_hi       = 0U;
    s_pkt.long_packet  = false;
    s_pkt.rx_callback  = rx_callback;
    s_pkt.tx_byte      = tx_byte;
}

/* ======================================================================== */
/* Receive State Machine                                                     */
/* ======================================================================== */
void packet_process_byte(uint8_t byte)
{
    packet_t *p = &s_pkt;

    switch (p->state) {
    case PACKET_STATE_WAIT_START:
        if (byte == PACKET_START_BYTE) {
            p->long_packet = false;
            p->state = PACKET_STATE_WAIT_LEN;
        } else if (byte == PACKET_START_BYTE_LONG) {
            p->long_packet = true;
            p->state = PACKET_STATE_WAIT_LEN;
        }
        /* Any other byte: remain waiting for a start byte. */
        break;

    case PACKET_STATE_WAIT_LEN:
        if (p->long_packet) {
            p->payload_len = (uint16_t)((uint16_t)byte << 8);
            p->state = PACKET_STATE_WAIT_LEN2;
        } else {
            p->payload_len = byte;
            p->payload_idx = 0U;
            if (p->payload_len == 0U) {
                p->state = PACKET_STATE_READ_CRC_HI;
            } else if (p->payload_len > PACKET_MAX_PAYLOAD) {
                p->state = PACKET_STATE_WAIT_START;   /* oversize: resync */
            } else {
                p->state = PACKET_STATE_READ_PAYLOAD;
            }
        }
        break;

    case PACKET_STATE_WAIT_LEN2:
        p->payload_len |= (uint16_t)byte;
        p->payload_idx = 0U;
        if (p->payload_len == 0U) {
            p->state = PACKET_STATE_READ_CRC_HI;
        } else if (p->payload_len > PACKET_MAX_PAYLOAD) {
            p->state = PACKET_STATE_WAIT_START;
        } else {
            p->state = PACKET_STATE_READ_PAYLOAD;
        }
        break;

    case PACKET_STATE_READ_PAYLOAD:
        p->payload[p->payload_idx++] = byte;
        if (p->payload_idx >= p->payload_len) {
            p->state = PACKET_STATE_READ_CRC_HI;
        }
        break;

    case PACKET_STATE_READ_CRC_HI:
        p->crc_hi = byte;
        p->state = PACKET_STATE_READ_CRC_LO;
        break;

    case PACKET_STATE_READ_CRC_LO: {
        uint16_t crc_calc = packet_crc16(p->payload, p->payload_len);
        uint16_t crc_recv = (uint16_t)(((uint16_t)p->crc_hi << 8) | byte);
        if (crc_calc == crc_recv) {
            p->state = PACKET_STATE_WAIT_STOP;
        } else {
            p->state = PACKET_STATE_WAIT_START;   /* CRC mismatch: resync */
        }
        break;
    }

    case PACKET_STATE_WAIT_STOP:
        if (byte == PACKET_STOP_BYTE) {
            if (p->rx_callback != NULL) {
                if (p->payload_len >= 1U) {
                    p->rx_callback(p->payload[0],
                                   &p->payload[1],
                                   (uint32_t)(p->payload_len - 1U));
                } else {
                    p->rx_callback(0U, NULL, 0U);
                }
            }
        }
        p->state = PACKET_STATE_WAIT_START;
        break;

    default:
        p->state = PACKET_STATE_WAIT_START;
        break;
    }
}

/* ======================================================================== */
/* Transmit Path                                                             */
/* ======================================================================== */
void packet_send(uint8_t command, const uint8_t *data, uint32_t len)
{
    if (s_pkt.tx_byte == NULL) {
        return;
    }

    /* Total payload = command byte + data bytes. */
    if (len > (uint32_t)(PACKET_MAX_PAYLOAD - 1U)) {
        len = (uint32_t)(PACKET_MAX_PAYLOAD - 1U);
    }

    uint16_t total = (uint16_t)(len + 1U);

    /* Build payload buffer for CRC. */
    s_tx_payload[0] = command;
    if ((len > 0U) && (data != NULL)) {
        memcpy(&s_tx_payload[1], data, len);
    }

    uint16_t crc = packet_crc16(s_tx_payload, total);

    /* Header: short or long framing. */
    if (total < 256U) {
        s_pkt.tx_byte(PACKET_START_BYTE);
        s_pkt.tx_byte((uint8_t)total);
    } else {
        s_pkt.tx_byte(PACKET_START_BYTE_LONG);
        s_pkt.tx_byte((uint8_t)(total >> 8));
        s_pkt.tx_byte((uint8_t)(total & 0xFFU));
    }

    /* Payload. */
    for (uint16_t i = 0U; i < total; i++) {
        s_pkt.tx_byte(s_tx_payload[i]);
    }

    /* CRC + stop. */
    s_pkt.tx_byte((uint8_t)(crc >> 8));
    s_pkt.tx_byte((uint8_t)(crc & 0xFFU));
    s_pkt.tx_byte(PACKET_STOP_BYTE);
}
