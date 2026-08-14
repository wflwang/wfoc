/*
 * app_uart.c - UART Control Interface (VESC-style Binary Protocol)
 *
 * Transports comm_packet_t commands over a UART using the packet.c framer.
 * Implemented commands cover firmware identification, real-time telemetry,
 * setpoint control, full motor/app configuration read-back and write-back,
 * motor parameter detection (stubbed) and ASCII terminal commands.
 *
 * All multi-byte fields are big-endian (network order), matching VESC.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include "app.h"
#include "conf/conf_general.h"
#include "config/fw_version.h"
#include "config/hwconf.h"
#include "packet/packet.h"
#include "driver/mcu/mcu_init.h"
#include <string.h>

/* ======================================================================== */
/* Platform Hooks (weak - override in the UART driver)                       */
/* ======================================================================== */
/* Push one byte to the UART TX register/FIFO. Override with a real driver. */
__attribute__((weak)) void uart_tx_byte(uint8_t byte) { (void)byte; }

/* Feed one byte from the UART RX ISR into the packet decoder. The platform
 * driver calls this; provided here so test harnesses can inject bytes. */
void app_uart_rx_byte(uint8_t byte);

/* Reboot the MCU. Override with the real NVIC reset handler. */
__attribute__((weak)) void system_reboot(void) {}

/* Run motor parameter detection. Override in mcpwm.c / detect.c. Returns the
 * number of result bytes written into out (0 = not available). */
__attribute__((weak)) uint32_t detect_motor_param(uint8_t *out, uint32_t max)
{ (void)out; (void)max; return 0U; }

/* ======================================================================== */
/* Buffer Helpers (big-endian)                                               */
/* ======================================================================== */
static void buf_append_u8(uint8_t *buf, uint8_t v, uint32_t *idx)
{
    buf[(*idx)++] = v;
}

static void buf_append_u16(uint8_t *buf, uint16_t v, uint32_t *idx)
{
    buf[(*idx)++] = (uint8_t)(v >> 8);
    buf[(*idx)++] = (uint8_t)(v & 0xFFU);
}

static void buf_append_i16(uint8_t *buf, int16_t v, uint32_t *idx)
{
    buf_append_u16(buf, (uint16_t)v, idx);
}

static void buf_append_u32(uint8_t *buf, uint32_t v, uint32_t *idx)
{
    buf[(*idx)++] = (uint8_t)(v >> 24);
    buf[(*idx)++] = (uint8_t)(v >> 16);
    buf[(*idx)++] = (uint8_t)(v >> 8);
    buf[(*idx)++] = (uint8_t)(v & 0xFFU);
}

static void buf_append_i32(uint8_t *buf, int32_t v, uint32_t *idx)
{
    buf_append_u32(buf, (uint32_t)v, idx);
}

/* 32-bit float scaled to signed int32:  value_i32 = (int32_t)(f * scale) */
static void buf_append_float32(uint8_t *buf, float f, float scale, uint32_t *idx)
{
    int32_t v = (int32_t)(f * scale);
    buf_append_i32(buf, v, idx);
}

/* 16-bit float scaled to signed int16. */
static void buf_append_float16(uint8_t *buf, float f, float scale, uint32_t *idx)
{
    int16_t v = (int16_t)(f * scale);
    buf_append_i16(buf, v, idx);
}

static uint8_t  buf_read_u8(const uint8_t *buf, uint32_t *idx)  { return buf[(*idx)++]; }
static uint16_t buf_read_u16(const uint8_t *buf, uint32_t *idx)
{
    uint16_t v = (uint16_t)(((uint16_t)buf[*idx] << 8) | buf[*idx + 1U]);
    *idx += 2U; return v;
}
static uint32_t buf_read_u32(const uint8_t *buf, uint32_t *idx)
{
    uint32_t v = ((uint32_t)buf[*idx] << 24) | ((uint32_t)buf[*idx + 1U] << 16) |
                 ((uint32_t)buf[*idx + 2U] << 8) | (uint32_t)buf[*idx + 3U];
    *idx += 4U; return v;
}
static int32_t buf_read_i32(const uint8_t *buf, uint32_t *idx)
{
    return (int32_t)buf_read_u32(buf, idx);
}
static float buf_read_float32(const uint8_t *buf, float scale, uint32_t *idx)
{
    return (float)buf_read_i32(buf, idx) / scale;
}

/* ======================================================================== */
/* Local State                                                               */
/* ======================================================================== */
#define UART_WATCHDOG_TIMEOUT_MS  2000U
static uint32_t s_last_alive_ms;       /* last COMM_ALIVE / any rx time    */
static bool     s_watchdog_armed;

/* High-speed streaming telemetry. When armed by COMM_GET_VALUES_SELECTIVE,
 * the main loop autonomously transmits the requested fields at rate_ms
 * intervals - no per-frame round-trip needed. rate_ms == 0 disables it.    */
static uint32_t s_stream_mask;         /* bitmask of TELEMETRY_MASK_*      */
static uint16_t s_stream_rate_ms;      /* send interval (0 = off)          */
static uint32_t s_stream_last_ms;      /* last streaming send time         */

/* ======================================================================== */
/* Configuration Serialisation                                              */
/* ======================================================================== */
/* Encode every field of mc_configuration_t in declaration order. */
static uint32_t serialize_mc_conf(const mc_configuration_t *c, uint8_t *buf)
{
    uint32_t i = 0U;
    buf_append_float32(buf, c->motor_r,            1e6f, &i);
    buf_append_float32(buf, c->motor_l,            1e6f, &i);
    buf_append_float32(buf, c->motor_flux_linkage, 1e6f, &i);
    buf_append_u8   (buf, c->motor_poles,                 &i);
    buf_append_u8   (buf, c->comm_mode,                   &i);
    buf_append_float32(buf, c->foc_current_kp,      1e6f, &i);
    buf_append_float32(buf, c->foc_current_ki,      1e6f, &i);
    buf_append_float32(buf, c->foc_speed_kp,        1e6f, &i);
    buf_append_float32(buf, c->foc_speed_ki,        1e6f, &i);
    buf_append_float32(buf, c->foc_duty_kp,         1e6f, &i);
    buf_append_float32(buf, c->foc_duty_ki,         1e6f, &i);
    buf_append_float32(buf, c->foc_current_filter,  1e6f, &i);
    buf_append_float32(buf, c->foc_fw_current_max,  1e6f, &i);
    buf_append_float32(buf, c->foc_fw_duty_start,   1e6f, &i);
    buf_append_float32(buf, c->foc_fw_ramp_time,   1e6f, &i);
    buf_append_float32(buf, c->foc_overmod_factor, 1e6f, &i);
    buf_append_u8   (buf, c->foc_notch_en,                 &i);
    buf_append_float32(buf, c->foc_notch_freq,      1e3f, &i);
    buf_append_float32(buf, c->foc_notch_bw,        1e3f, &i);
    buf_append_u8   (buf, c->observer_type,                &i);
    buf_append_float32(buf, c->observer_gain,       1e6f, &i);
    buf_append_float32(buf, c->observer_pll_kp,      1e6f, &i);
    buf_append_float32(buf, c->observer_pll_ki,     1e6f, &i);
    buf_append_float32(buf, c->hfi_amplitude,        1e3f, &i);
    buf_append_float32(buf, c->hfi_freq,             1e3f, &i);
    buf_append_float32(buf, c->l_max_duty,          1e6f, &i);
    buf_append_float32(buf, c->l_min_duty,          1e6f, &i);
    buf_append_float32(buf, c->l_max_speed,          1e0f, &i);
    buf_append_float32(buf, c->l_min_speed,          1e0f, &i);
    buf_append_float32(buf, c->l_max_current,        1e3f, &i);
    buf_append_float32(buf, c->l_min_current,        1e3f, &i);
    buf_append_float32(buf, c->l_in_current_max,     1e3f, &i);
    buf_append_float32(buf, c->l_in_current_min,     1e3f, &i);
    buf_append_float32(buf, c->l_watt_max,           1e3f, &i);
    buf_append_float32(buf, c->l_watt_min,           1e3f, &i);
    buf_append_float32(buf, c->l_temp_motor_max,     1e2f, &i);
    buf_append_float32(buf, c->l_temp_mosfet_max,    1e2f, &i);
    buf_append_u16  (buf, c->pwm_freq,                      &i);
    buf_append_u16  (buf, c->deadtime,                      &i);
    buf_append_u8   (buf, c->bldc_bemf_mode,                &i);
    buf_append_float32(buf, c->bldc_bemf_k,          1e6f, &i);
    buf_append_float32(buf, c->bldc_comm_time,       1e6f, &i);
    buf_append_u8   (buf, c->sound_mode,                    &i);
    buf_append_u16  (buf, c->sound_freq,                   &i);
    buf_append_u16  (buf, c->sound_time,                   &i);
    return i;
}

/* Decode every field into *c. Returns true if enough bytes were present. */
static bool deserialize_mc_conf(mc_configuration_t *c, const uint8_t *buf, uint32_t len)
{
    uint32_t i = 0U;
    if (len < 4U) { return false; }
    c->motor_r            = buf_read_float32(buf, 1e6f, &i);
    c->motor_l            = buf_read_float32(buf, 1e6f, &i);
    c->motor_flux_linkage = buf_read_float32(buf, 1e6f, &i);
    c->motor_poles        = buf_read_u8(buf, &i);
    c->comm_mode          = buf_read_u8(buf, &i);
    c->foc_current_kp     = buf_read_float32(buf, 1e6f, &i);
    c->foc_current_ki     = buf_read_float32(buf, 1e6f, &i);
    c->foc_speed_kp       = buf_read_float32(buf, 1e6f, &i);
    c->foc_speed_ki       = buf_read_float32(buf, 1e6f, &i);
    c->foc_duty_kp        = buf_read_float32(buf, 1e6f, &i);
    c->foc_duty_ki        = buf_read_float32(buf, 1e6f, &i);
    c->foc_current_filter = buf_read_float32(buf, 1e6f, &i);
    c->foc_fw_current_max = buf_read_float32(buf, 1e6f, &i);
    c->foc_fw_duty_start  = buf_read_float32(buf, 1e6f, &i);
    c->foc_fw_ramp_time  = buf_read_float32(buf, 1e6f, &i);
    c->foc_overmod_factor= buf_read_float32(buf, 1e6f, &i);
    c->foc_notch_en      = buf_read_u8(buf, &i);
    c->foc_notch_freq    = buf_read_float32(buf, 1e3f, &i);
    c->foc_notch_bw      = buf_read_float32(buf, 1e3f, &i);
    c->observer_type     = buf_read_u8(buf, &i);
    c->observer_gain     = buf_read_float32(buf, 1e6f, &i);
    c->observer_pll_kp   = buf_read_float32(buf, 1e6f, &i);
    c->observer_pll_ki   = buf_read_float32(buf, 1e6f, &i);
    c->hfi_amplitude     = buf_read_float32(buf, 1e3f, &i);
    c->hfi_freq          = buf_read_float32(buf, 1e3f, &i);
    c->l_max_duty        = buf_read_float32(buf, 1e6f, &i);
    c->l_min_duty        = buf_read_float32(buf, 1e6f, &i);
    c->l_max_speed       = buf_read_float32(buf, 1e0f, &i);
    c->l_min_speed       = buf_read_float32(buf, 1e0f, &i);
    c->l_max_current     = buf_read_float32(buf, 1e3f, &i);
    c->l_min_current     = buf_read_float32(buf, 1e3f, &i);
    c->l_in_current_max  = buf_read_float32(buf, 1e3f, &i);
    c->l_in_current_min  = buf_read_float32(buf, 1e3f, &i);
    c->l_watt_max        = buf_read_float32(buf, 1e3f, &i);
    c->l_watt_min        = buf_read_float32(buf, 1e3f, &i);
    c->l_temp_motor_max  = buf_read_float32(buf, 1e2f, &i);
    c->l_temp_mosfet_max = buf_read_float32(buf, 1e2f, &i);
    c->pwm_freq          = buf_read_u16(buf, &i);
    c->deadtime          = buf_read_u16(buf, &i);
    c->bldc_bemf_mode    = buf_read_u8(buf, &i);
    c->bldc_bemf_k       = buf_read_float32(buf, 1e6f, &i);
    c->bldc_comm_time    = buf_read_float32(buf, 1e6f, &i);
    c->sound_mode        = buf_read_u8(buf, &i);
    c->sound_freq        = buf_read_u16(buf, &i);
    c->sound_time        = buf_read_u16(buf, &i);
    return true;
}

static uint32_t serialize_app_conf(const app_configuration_t *c, uint8_t *buf)
{
    uint32_t i = 0U;
    buf_append_u8 (buf, c->app_mode,         &i);
    buf_append_float32(buf, c->ppm_max, 1e6f, &i);
    buf_append_float32(buf, c->ppm_min, 1e6f, &i);
    buf_append_float32(buf, c->ppm_ramp,1e6f, &i);
    buf_append_u32 (buf, c->uart_baud,        &i);
    return i;
}

static bool deserialize_app_conf(app_configuration_t *c, const uint8_t *buf, uint32_t len)
{
    uint32_t i = 0U;
    if (len < 5U) { return false; }
    c->app_mode  = buf_read_u8(buf, &i);
    c->ppm_max   = buf_read_float32(buf, 1e6f, &i);
    c->ppm_min   = buf_read_float32(buf, 1e6f, &i);
    c->ppm_ramp  = buf_read_float32(buf, 1e6f, &i);
    c->uart_baud = buf_read_u32(buf, &i);
    return true;
}

/* ======================================================================== */
/* Response Helpers                                                          */
/* ======================================================================== */
static void send_command(comm_packet_t cmd, const uint8_t *data, uint32_t len)
{
    packet_send((uint8_t)cmd, data, len);
}

/* ======================================================================== */
/* Command Handlers                                                          */
/* ======================================================================== */
static void cmd_fw_version(void)
{
    uint8_t buf[16];
    uint32_t i = 0U;
    buf_append_u8(buf, FW_VERSION_MAJOR, &i);
    buf_append_u8(buf, FW_VERSION_MINOR, &i);
    buf_append_u8(buf, FW_VERSION_PATCH, &i);
    /* Hardware name string (null-terminated). */
    const char *hw = HW_NAME;
    for (uint32_t k = 0U; hw[k] != '\0' && i < sizeof(buf); k++) {
        buf_append_u8(buf, (uint8_t)hw[k], &i);
    }
    buf_append_u8(buf, 0U, &i);
    send_command(COMM_FW_VERSION, buf, i);
}

static void cmd_get_values(void)
{
    motor_state_t *st = app_get_state();
    uint8_t buf[64];
    uint32_t i = 0U;

    /* Temperature x100 deg C */
    buf_append_float16(buf, st->temp_mosfet, 1e2f, &i);
    buf_append_float16(buf, st->temp_motor,  1e2f, &i);
    /* Motor current x100 A */
    buf_append_float16(buf, st->iq, 1e2f, &i);
    /* Input current x100 A (approx = power / vbus) */
    float i_in = (st->vbus > 1.0f) ? (st->power / st->vbus) : 0.0f;
    buf_append_float16(buf, i_in, 1e2f, &i);
    /* D/Q axis currents x100 A */
    buf_append_float16(buf, st->id, 1e2f, &i);
    buf_append_float16(buf, st->iq, 1e2f, &i);
    /* 3-phase currents x100 A */
    buf_append_float16(buf, st->ia, 1e2f, &i);
    buf_append_float16(buf, st->ib, 1e2f, &i);
    buf_append_float16(buf, st->ic, 1e2f, &i);
    /* 3-phase voltages x100 V */
    buf_append_float16(buf, st->va, 1e2f, &i);
    buf_append_float16(buf, st->vb, 1e2f, &i);
    buf_append_float16(buf, st->vc, 1e2f, &i);
    /* D/Q axis voltages x100 V */
    buf_append_float16(buf, st->vd, 1e2f, &i);
    buf_append_float16(buf, st->vq, 1e2f, &i);
    /* Duty x1000 (0..1) */
    buf_append_float16(buf, st->duty, 1e3f, &i);
    /* Speed (ERPM) */
    buf_append_i32(buf, (int32_t)st->speed, &i);
    /* Rotor position x1e4 rad */
    buf_append_float32(buf, st->position, 1e4f, &i);
    /* Input voltage x10 V */
    buf_append_float16(buf, st->vbus, 1e1f, &i);
    /* Consumed charge x1e4 Ah */
    buf_append_i32(buf, (int32_t)(st->ah * 1e4f), &i);
    /* Consumed energy x1e4 Wh */
    buf_append_i32(buf, (int32_t)(st->energy * 1e4f), &i);
    /* Power x1e4 W */
    buf_append_i32(buf, (int32_t)(st->power * 1e4f), &i);
    /* Fault code */
    buf_append_u8(buf, st->fault, &i);

    send_command(COMM_GET_VALUES, buf, i);
}

/* ======================================================================== */
/* Selective Telemetry (high-speed streaming)                                */
/* ======================================================================== */
/* Packs only the fields selected by mask into a compact buffer and sends a
 * COMM_GET_VALUES_SELECTIVE frame. Each selected field is appended in a
 * fixed order (lowest mask bit first) so the host can decode without any
 * length-prefix per field. Uses 16-bit scaled ints to keep packets small.
 *
 * This is the key to high-rate printing: the ISR only updates motor_state
 * (already happening at PWM rate), and the main loop calls this either on
 * demand or periodically via the streaming timer - never from the ISR.    */
static void telemetry_send_selective(uint32_t mask)
{
    motor_state_t *st = app_get_state();
    uint8_t buf[80];
    uint32_t i = 0U;

    /* Echo the mask first so the host knows which fields follow. */
    buf_append_u32(buf, mask, &i);

    if (mask & TELEMETRY_MASK_IA)    buf_append_float16(buf, st->ia,       1e3f, &i);
    if (mask & TELEMETRY_MASK_IB)    buf_append_float16(buf, st->ib,       1e3f, &i);
    if (mask & TELEMETRY_MASK_IC)    buf_append_float16(buf, st->ic,       1e3f, &i);
    if (mask & TELEMETRY_MASK_VA)    buf_append_float16(buf, st->va,       1e2f, &i);
    if (mask & TELEMETRY_MASK_VB)    buf_append_float16(buf, st->vb,       1e2f, &i);
    if (mask & TELEMETRY_MASK_VC)    buf_append_float16(buf, st->vc,       1e2f, &i);
    if (mask & TELEMETRY_MASK_ID)    buf_append_float16(buf, st->id,       1e3f, &i);
    if (mask & TELEMETRY_MASK_IQ)    buf_append_float16(buf, st->iq,       1e3f, &i);
    if (mask & TELEMETRY_MASK_VD)    buf_append_float16(buf, st->vd,       1e2f, &i);
    if (mask & TELEMETRY_MASK_VQ)    buf_append_float16(buf, st->vq,       1e2f, &i);
    if (mask & TELEMETRY_MASK_DUTY)  buf_append_float16(buf, st->duty,     1e3f, &i);
    if (mask & TELEMETRY_MASK_SPEED) buf_append_i32    (buf, (int32_t)st->speed,  &i);
    if (mask & TELEMETRY_MASK_POSITION) buf_append_float32(buf, st->position, 1e4f, &i);
    if (mask & TELEMETRY_MASK_VBUS)  buf_append_float16(buf, st->vbus,     1e2f, &i);
    if (mask & TELEMETRY_MASK_TEMP_MOS) buf_append_float16(buf, st->temp_mosfet, 1e2f, &i);
    if (mask & TELEMETRY_MASK_TEMP_MOT) buf_append_float16(buf, st->temp_motor,  1e2f, &i);
    if (mask & TELEMETRY_MASK_POWER) buf_append_float32(buf, st->power,    1e3f, &i);

    send_command(COMM_GET_VALUES_SELECTIVE, buf, i);
}

/* Handle an incoming COMM_GET_VALUES_SELECTIVE request.
 * Payload layout (all big-endian):
 *   [mask:u32] [rate_ms:u16]
 * If rate_ms > 0, streaming is armed: the main loop will autonomously send
 * the selected fields every rate_ms milliseconds. If rate_ms == 0, only a
 * single response is sent and streaming is disabled.                     */
static void cmd_get_values_selective(const uint8_t *data, uint32_t len)
{
    uint32_t mask = 0U;
    uint16_t rate_ms = 0U;

    if (len >= 4U) {
        uint32_t i = 0U;
        mask = buf_read_u32(data, &i);
        if (len >= 6U) {
            rate_ms = buf_read_u16(data, &i);
        }
    }

    /* Configure streaming state. */
    s_stream_mask    = mask;
    s_stream_rate_ms = rate_ms;
    s_stream_last_ms = mcu_millis();

    /* Always reply immediately with one frame. */
    telemetry_send_selective(mask);
}

static void cmd_get_mcconf(void)
{
    uint8_t buf[PACKET_MAX_PAYLOAD];
    uint32_t n = serialize_mc_conf(&mc_conf, buf);
    send_command(COMM_GET_MCCONF, buf, n);
}

static void cmd_set_mcconf(const uint8_t *data, uint32_t len)
{
    mc_configuration_t tmp = mc_conf;
    if (deserialize_mc_conf(&tmp, data, len)) {
        (void)conf_general_write_mc_conf(&tmp);
    }
    /* Acknowledge. */
    send_command(COMM_SET_MCCONF, NULL, 0U);
}

static void cmd_get_appconf(void)
{
    uint8_t buf[PACKET_MAX_PAYLOAD];
    uint32_t n = serialize_app_conf(&app_conf, buf);
    send_command(COMM_GET_APPCONF, buf, n);
}

static void cmd_set_appconf(const uint8_t *data, uint32_t len)
{
    app_configuration_t tmp = app_conf;
    if (deserialize_app_conf(&tmp, data, len)) {
        (void)conf_general_write_app_conf(&tmp);
    }
    send_command(COMM_SET_APPCONF, NULL, 0U);
}

static void cmd_detect_motor_param(void)
{
    uint8_t buf[16];
    uint32_t n = detect_motor_param(buf, sizeof(buf));
    send_command(COMM_DETECT_MOTOR_PARAM, buf, n);
}

static void cmd_terminal(const uint8_t *data, uint32_t len)
{
    /* Build a null-terminated command string. */
    static char cmd[64];
    uint32_t n = (len < (sizeof(cmd) - 1U)) ? len : (sizeof(cmd) - 1U);
    memcpy(cmd, data, n);
    cmd[n] = '\0';

    const char *reply = "";
    if (strcmp(cmd, "help") == 0) {
        reply = "stop restart get_param set_param sound faults alive";
    } else if (strcmp(cmd, "stop") == 0) {
        app_stop();
        reply = "ok";
    } else if (strcmp(cmd, "restart") == 0) {
        system_reboot();
        reply = "ok";
    } else if (strcmp(cmd, "faults") == 0) {
        reply = "ok";
    } else if (strcmp(cmd, "alive") == 0) {
        s_last_alive_ms = mcu_millis();
        s_watchdog_armed = false;
        reply = "ok";
    } else if (strcmp(cmd, "get_param") == 0) {
        reply = "ok";
    } else {
        reply = "unknown";
    }

    /* Echo the reply as the response payload (ASCII). */
    uint32_t rl = 0U;
    while (reply[rl] != '\0') { rl++; }
    send_command(COMM_TERMINAL_CMD, (const uint8_t *)reply, rl);
}

/* ======================================================================== */
/* Packet Receive Callback                                                   */
/* ======================================================================== */
static void uart_packet_rx(uint8_t command, const uint8_t *data, uint32_t len)
{
    /* Any received frame feeds the watchdog. */
    s_last_alive_ms = mcu_millis();
    s_watchdog_armed = false;

    switch ((comm_packet_t)command) {
    case COMM_FW_VERSION:       cmd_fw_version();              break;
    case COMM_GET_VALUES:       cmd_get_values();              break;
    case COMM_GET_VALUES_SELECTIVE: cmd_get_values_selective(data, len); break;
    case COMM_SET_DUTY:
        if (len >= 4) {
            uint32_t i = 0U;
            app_set_duty(buf_read_float32(data, 1e5f, &i));
        }
        break;
    case COMM_SET_CURRENT:
        if (len >= 4) {
            uint32_t i = 0U;
            app_set_current(buf_read_float32(data, 1e3f, &i));
        }
        break;
    case COMM_SET_CURRENT_BRAKE:
        if (len >= 4) {
            uint32_t i = 0U;
            app_set_brake(buf_read_float32(data, 1e3f, &i));
        }
        break;
    case COMM_SET_RPM:
        if (len >= 4) {
            uint32_t i = 0U;
            app_set_speed(buf_read_float32(data, 1e0f, &i));
        }
        break;
    case COMM_SET_POS:
        if (len >= 4) {
            uint32_t i = 0U;
            app_set_position(buf_read_float32(data, 1e6f, &i));
        }
        break;
    case COMM_GET_MCCONF:       cmd_get_mcconf();              break;
    case COMM_SET_MCCONF:       cmd_set_mcconf(data, len);      break;
    case COMM_GET_APPCONF:     cmd_get_appconf();             break;
    case COMM_SET_APPCONF:     cmd_set_appconf(data, len);     break;
    case COMM_DETECT_MOTOR_PARAM: cmd_detect_motor_param();    break;
    case COMM_TERMINAL_CMD:    cmd_terminal(data, len);        break;
    case COMM_ALIVE:
        /* Watchdog feed only. */
        break;
    case COMM_REBOOT:
        system_reboot();
        break;
    default:
        /* Unknown command: ignore. */
        break;
    }
}

/* ======================================================================== */
/* Public API                                                                */
/* ======================================================================== */
void app_uart_init(void)
{
    packet_init(uart_packet_rx, uart_tx_byte);
    s_last_alive_ms = mcu_millis();
    s_watchdog_armed = false;
    s_stream_mask = 0U;
    s_stream_rate_ms = 0U;
    s_stream_last_ms = 0U;
}

void app_uart_rx_byte(uint8_t byte)
{
    packet_process_byte(byte);
}

void app_uart_process(void)
{
    /* Watchdog: if a tool opened the port (armed) and then went silent,
     * bring the motor to a safe stop. The first received frame arms the
     * watchdog; subsequent frames keep feeding it. */
    if (mcu_millis() - s_last_alive_ms > UART_WATCHDOG_TIMEOUT_MS) {
        if (!s_watchdog_armed) {
            s_watchdog_armed = true;
        }
    } else {
        s_watchdog_armed = false;
    }

    if (s_watchdog_armed) {
        /* Recover by stopping the motor. Re-arm stays until next frame. */
        app_stop();
    }

    /* High-speed streaming telemetry: if a host armed streaming with a
     * non-zero rate, autonomously emit the selected fields at that rate.
     * This avoids per-frame round-trips and gives the highest print rate
     * the UART baud allows. Data is read live from motor_state which is
     * refreshed by the ADC ISR - no extra copy needed.                    */
    if ((s_stream_rate_ms != 0U) && (s_stream_mask != 0U)) {
        uint32_t now = mcu_millis();
        if ((now - s_stream_last_ms) >= s_stream_rate_ms) {
            s_stream_last_ms = now;
            telemetry_send_selective(s_stream_mask);
        }
    }
}
