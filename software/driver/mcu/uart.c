/*
 * uart.c - UART Driver for CIU32F003x5 (USART1)
 *
 * USART1 on PC0 (TX) / PB5 (RX), 8N1, ring-buffer RX,
 * interrupt-driven TX with ring buffer.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include "uart.h"
#include "gpio.h"
#include "mcu_init.h"

/* ======================================================================== */
/* RX ring buffer                                                            */
/* ======================================================================== */
static volatile uint8_t s_rx_buf[UART_RX_BUF_SIZE];
static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;

/* ======================================================================== */
/* TX ring buffer                                                            */
/* ======================================================================== */
#define UART_TX_BUF_SIZE    256U
static volatile uint8_t s_tx_buf[UART_TX_BUF_SIZE];
static volatile uint8_t s_tx_head;
static volatile uint8_t s_tx_tail;

/* ======================================================================== */
/* Initialization                                                            */
/* ======================================================================== */
void uart_init(uint32_t baud)
{
    /* Enable GPIO clocks for TX (PC0) and RX (PB5) ports */
    SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOCEN_Msk | RCC_IOPENR_GPIOBEN_Msk);
    /* Enable USART1 clock on APB2 */
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN_Msk);

    /* Configure GPIO: PC0 = USART1_TX (AF1), PB5 = USART1_RX (AF1) */
    gpio_init(UART_TX_PORT, UART_TX_PIN, GPIO_MODE_AF, GPIO_PULL_UP,
              GPIO_SPEED_HIGH, GPIO_AF_USART1);
    gpio_init(UART_RX_PORT, UART_RX_PIN, GPIO_MODE_AF, GPIO_PULL_UP,
              GPIO_SPEED_HIGH, GPIO_AF_USART1);

    /* Compute BRR for oversampling-by-16: fCK / baud */
    uint32_t brr = MCU_CLK_HZ / baud;

    /* Disable USART1 while configuring */
    USART_DISABLE(USART1);

    /* Set baud rate register */
    USART1->BRR = brr;

    /* CR2: 1 stop bit, default polarity/phase (8N1) */
    USART1->CR2 = 0;

    /* CR3: no flow control, no DMA */
    USART1->CR3 = 0;

    /* CR1: enable UE, RE, TE. Interrupts are enabled in two steps. */
    USART1->CR1 = USART_CR1_UE_Msk | USART_CR1_RE_Msk | USART_CR1_TE_Msk;

    /* Clear any stale flags after enabling UE */
    WRITE_REG(USART1->ICR,
              USART_ISR_PE_Msk  | USART_ISR_FE_Msk  |
              USART_ISR_NE_Msk  | USART_ISR_ORE_Msk |
              USART_ISR_IDLE_Msk| USART_ISR_RXNE_Msk|
              USART_ISR_TC_Msk  | USART_ISR_TXE_Msk);

    /* Enable RXNE interrupt (TXE is enabled on demand via uart_tx_buf) */
    SET_BIT(USART1->CR1, USART_CR1_RXNEIE_Msk);

    /* Enable USART1 global interrupt in NVIC */
    NVIC_EnableIRQ(USART1_IRQn);
}

/* ======================================================================== */
/* TX - blocking single byte                                                 */
/* ======================================================================== */
void uart_tx_byte(uint8_t byte)
{
    while (READ_BIT(USART1->ISR, USART_ISR_TXE_Msk) == 0U) {
        /* spin until TX data register empty */
    }
    USART1->TDR = byte;
}

/* ======================================================================== */
/* TX - interrupt-driven buffer                                             */
/* ======================================================================== */
void uart_tx_buf(const uint8_t *buf, uint16_t len)
{
    if ((len == 0U) || (buf == 0)) {
        return;
    }

    uint32_t primask = mcu_critical_enter();

    uint8_t tx_was_empty = (s_tx_head == s_tx_tail);

    for (uint16_t i = 0U; i < len; i++) {
        uint8_t next = (uint8_t)((s_tx_head + 1U) & (UART_TX_BUF_SIZE - 1U));
        if (next == s_tx_tail) {
            break;   /* TX buffer full - discard remainder */
        }
        s_tx_buf[s_tx_head] = buf[i];
        s_tx_head = next;
    }

    /* If the TX ring was empty before, kick off transmission via TXE ISR. */
    if (tx_was_empty && (s_tx_head != s_tx_tail)) {
        SET_BIT(USART1->CR1, USART_CR1_TXEIE_Msk);
    }

    mcu_critical_exit(primask);
}

/* ======================================================================== */
/* TX drain (called from USART1_IRQHandler on TXE)                           */
/* ======================================================================== */
void uart_tx_drain(void)
{
    if (s_tx_head != s_tx_tail) {
        USART1->TDR = s_tx_buf[s_tx_tail];
        s_tx_tail = (uint8_t)((s_tx_tail + 1U) & (UART_TX_BUF_SIZE - 1U));
    } else {
        /* Buffer drained - disable TXE until new data arrives */
        CLEAR_BIT(USART1->CR1, USART_CR1_TXEIE_Msk);
    }
}

/* ======================================================================== */
/* Flush TX                                                                  */
/* ======================================================================== */
void uart_flush_tx(void)
{
    /* Wait for the TX ring buffer to be drained by the ISR */
    while (s_tx_head != s_tx_tail) {
        /* TXE interrupt is doing the work */
    }
    /* Wait for the last byte to finish transmitting */
    while (READ_BIT(USART1->ISR, USART_ISR_TC_Msk) == 0U) {
        /* spin */
    }
}

/* ======================================================================== */
/* RX ring buffer                                                            */
/* ======================================================================== */
void uart_rx_push(uint8_t byte)
{
    uint8_t next = (uint8_t)(s_rx_head + 1U);
    if (next == s_rx_tail) {
        return;   /* RX buffer full - discard byte */
    }
    s_rx_buf[s_rx_head] = byte;
    s_rx_head = next;
}

uint8_t uart_rx_byte(void)
{
    if (s_rx_head == s_rx_tail) {
        return 0U;   /* nothing available */
    }
    uint8_t byte = s_rx_buf[s_rx_tail];
    s_rx_tail++;
    return byte;
}

uint8_t uart_rx_available(void)
{
    return (uint8_t)(s_rx_head != s_rx_tail);
}

uint16_t uart_rx_count(void)
{
    /* uint8_t subtraction wraps correctly for a power-of-2 sized ring. */
    return (uint16_t)((uint8_t)(s_rx_head - s_rx_tail));
}