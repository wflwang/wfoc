/*
 * packet.h - VESC-style Packet Framing
 *
 * Length-prefixed binary framing with CRC-16/CCITT integrity check and
 * start/stop byte synchronisation. Used by the UART application interface
 * (app_uart.c) to transport comm_packet_t commands and their payloads.
 *
 * Wire format:
 *   Short payload (length < 256):
 *     [0x02] [len] [payload...] [crc_hi] [crc_lo] [0x03]
 *   Long payload (length >= 256):
 *     [0x03] [len_hi] [len_lo] [payload...] [crc_hi] [crc_lo] [0x03]
 *   where payload = [command] [data...] and len counts command + data bytes.
 *   CRC-16/CCITT (poly 0x1021, init 0xFFFF) covers the payload, big-endian.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* Constants                                                                 */
/* ======================================================================== */
#define PACKET_MAX_PAYLOAD      512     /* command + data bytes              */
#define PACKET_START_BYTE       0x02    /* short frame (< 256 bytes)         */
#define PACKET_START_BYTE_LONG  0x03    /* long frame  (>= 256 bytes)        */
#define PACKET_STOP_BYTE        0x03    /* frame terminator                  */

/* ======================================================================== */
/* Callback Types                                                            */
/* ======================================================================== */
/* Called once per transmitted byte. Provide a non-weak override in the
 * platform UART driver to push bytes to the hardware. */
typedef void (*packet_tx_byte_fn)(uint8_t byte);

/* Called when a complete, CRC-valid frame is received.
 *  command: payload[0]
 *  data:    pointer to payload[1..], may be NULL when len == 0
 *  len:     number of data bytes (excludes the command byte) */
typedef void (*packet_rx_fn)(uint8_t command, const uint8_t *data, uint32_t len);

/* ======================================================================== */
/* Receiver State Machine                                                    */
/* ======================================================================== */
typedef enum {
    PACKET_STATE_WAIT_START = 0,
    PACKET_STATE_WAIT_LEN,
    PACKET_STATE_WAIT_LEN2,        /* low byte of long-packet length       */
    PACKET_STATE_READ_PAYLOAD,
    PACKET_STATE_READ_CRC_HI,
    PACKET_STATE_READ_CRC_LO,
    PACKET_STATE_WAIT_STOP,
} packet_state_t;

typedef struct {
    packet_state_t    state;
    uint8_t           payload[PACKET_MAX_PAYLOAD];
    uint16_t          payload_len;   /* expected payload length            */
    uint16_t          payload_idx;   /* payload bytes received so far      */
    uint8_t           crc_hi;        /* high byte of received CRC          */
    bool              long_packet;   /* true = 2-byte length frame         */
    packet_rx_fn      rx_callback;
    packet_tx_byte_fn tx_byte;
} packet_t;

/* ======================================================================== */
/* Public API                                                                */
/* ======================================================================== */
/* Initialise the (single) packet handler. Install a receive callback and a
 * transmit function (may be NULL for listen-only operation). */
void packet_init(packet_rx_fn rx_callback, packet_tx_byte_fn tx_byte);

/* Feed one received byte into the state machine. Safe to call from the UART
 * RX interrupt handler. */
void packet_process_byte(uint8_t byte);

/* Build and transmit a framed packet. Selects short/long framing based on
 * the total payload size. No-op if no tx_byte function is installed. */
void packet_send(uint8_t command, const uint8_t *data, uint32_t len);

/* Compute CRC-16/CCITT (poly 0x1021, init 0xFFFF) over len bytes. */
uint16_t packet_crc16(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* PACKET_H */
