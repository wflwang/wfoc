/*
 * uart.h - UART Driver for CIU32F003x5 (USART1)
 *
 * USART1 on PC0 (TX) / PB5 (RX), 8N1, ring-buffer RX,
 * interrupt-driven TX with ring buffer.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "ciu32f003x.h"
#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* RX ring buffer size (bytes)                                               */
/* ======================================================================== */
#define UART_RX_BUF_SIZE    256U

/* ======================================================================== */
/* Initialization                                                            */
/* ======================================================================== */
/*! Initialize USART1 on PC0(TX)/PB5(RX) at the given baud rate.
 *  Configures GPIO, computes BRR, enables RX/TX and the RXNE interrupt,
 *  and enables USART1 in the NVIC. */
void uart_init(uint32_t baud);

/* ======================================================================== */
/* TX functions                                                              */
/* ======================================================================== */
/*! Transmit a single byte (blocking - polls TXE flag). */
void uart_tx_byte(uint8_t byte);

/*! Transmit a byte buffer via the interrupt-driven TX ring buffer.
 *  Returns immediately; transmission proceeds in the background. */
void uart_tx_buf(const uint8_t *buf, uint16_t len);

/*! Called from USART1_IRQHandler when TXE fires.
 *  Feeds the next byte from the TX ring buffer into TDR.
 *  Weak stub in interrupts.c is overridden here. */
void uart_tx_drain(void);

/*! Block until all pending TX data has been fully sent (waits for TC). */
void uart_flush_tx(void);

/* ======================================================================== */
/* RX functions                                                              */
/* ======================================================================== */
/*! Pop one byte from the RX ring buffer.
 *  Returns 0 if the buffer is empty; check uart_rx_available() first. */
uint8_t uart_rx_byte(void);

/*! Returns non-zero if at least one byte is waiting in the RX buffer. */
uint8_t uart_rx_available(void);

/*! Returns the number of bytes currently stored in the RX buffer. */
uint16_t uart_rx_count(void);

/*! Push a received byte into the RX ring buffer (called from ISR).
 *  Not intended for direct use by application code. */
void uart_rx_push(uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* UART_H */