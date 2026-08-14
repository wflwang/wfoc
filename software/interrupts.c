/*
 * interrupts.c - Interrupt Handlers for WFOC Motor Controller
 *
 * Centralizes all ISR implementations:
 *   - ADC1_IRQHandler:      FOC / BLDC real-time current loop (PWM rate)
 *   - USART1_IRQHandler:    UART RX / TX
 *   - TIM2_IRQHandler:      PPM pulse-width capture
 *   - SysTick_Handler:      1 ms tick counter
 *   - HardFault_Handler:    Illegal-access / bus fault trap
 *   - Default_Handler:      Catch-all for unconfigured interrupts
 *
 * The FOC current loop is entirely fixed-point (Q16.16) - no floating-point
 * arithmetic is used inside the ADC ISR.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include <stdint.h>

#include "ciu32f003x.h"
#include "mcu_init.h"
#include "adc.h"
#include "hwconf.h"
#include "board_config.h"

#include "conf/datatypes.h"
#include "motor/bldc.h"
#include "motor/observer.h"
#include "motor/foc.h"
#include "util/fixedpoint.h"
#include "driver/mcu/pwm.h"
#include "driver/mcu/uart.h"
#include "driver/mcu/timer.h"

/* ======================================================================== */
/* External Globals (defined in main.c / conf_general.c / mcu_init.c)        */
/* ======================================================================== */
extern mc_configuration_t  mc_conf;       /* conf/conf_general.c             */
extern motor_state_t       motor_state;   /* main.c                          */
extern foc_state_t         foc_state;     /* main.c                          */
extern observer_state_t    observer_state;/* main.c                          */
extern bldc_state_t        bldc_state;    /* main.c                          */

/* SysTick millisecond counter (defined in mcu_init.c, incremented here). */
extern volatile uint32_t s_tick_ms;

/* ======================================================================== */
/* Forward Declarations - subsystem hooks implemented elsewhere              */
/* ======================================================================== */
extern void uart_rx_push(uint8_t byte);      /* driver/mcu/uart.c          */

/* PPM pulse-width storage (updated by TIM2_IRQHandler).
 * ppm_get_pulse_ms() overrides the weak stub in app/app_ppm.c. */
static volatile uint32_t s_ppm_pulse_ticks = 0;

/* ======================================================================== */
/* Local Helper Forward Declarations                                         */
/* ======================================================================== */
static void foc_run_loop(q16_t ialpha, q16_t ibeta, q16_t vbus,
                         uint16_t angle);
static void svpwm_set_duty(q16_t valpha, q16_t vbeta, q16_t vbus);
static void bldc_run_foc_dispatch(uint16_t vol_u, uint16_t vol_v,
                                  uint16_t vol_w, uint16_t vbus_raw,
                                  uint16_t temp_raw);

/* ======================================================================== */
/* Q16.16 Conversion Constants (precomputed from board_config.h)             */
/* ======================================================================== */
/* Current:  I = (ADC - offset) * Vref / (ADCres * R_shunt * OpAmp_gain)
 *   = (ADC - offset) * 3.3 / (4096 * 0.005 * 20) = 0.00806 A/LSB
 *   Q16.16: 0.00806 * 65536 ~= 528                                      */
#define CURRENT_GAIN_Q16     ((q16_t)528)

/* Bus voltage: V = ADC * Vref * divider_ratio / ADCres
 *   = ADC * 3.3 * 11 / 4096 = 0.00886 V/LSB
 *   Q16.16: 0.00886 * 65536 ~= 580                                      */
#define VBUS_GAIN_Q16        ((q16_t)580)

/* Phase voltage: V = ADC * Vref / ADCres
 *   = ADC * 3.3 / 4096 = 0.000806 V/LSB
 *   Q16.16: 0.000806 * 65536 ~= 53                                      */
#define PHASE_VOLT_GAIN_Q16  ((q16_t)53)

/* 1/sqrt(3) in Q16.16 (Clarke transform beta axis) */
#define Q16_1_SQRT3          ((q16_t)37837)

/* ======================================================================== */
/* Default Handler                                                           */
/* ======================================================================== */
/* All unused interrupts alias here. An infinite loop lets a debugger
 * inspect the fault; in production a watchdog reset will follow.            */
void Default_Handler(void)
{
    for (;;) {
        /* spin - wait for watchdog or debugger */
    }
}

/* ======================================================================== */
/* HardFault Handler                                                         */
/* ======================================================================== */
void HardFault_Handler(void)
{
    /* A hard fault is unrecoverable. Record the fault and loop so the
     * independent watchdog can reset the MCU.                               */
    motor_state.fault = FAULT_DRV_FAULT;
    for (;;) {
        /* spin - IWDG will reset the MCU */
    }
}

/* ======================================================================== */
/* SysTick Handler - 1 ms Tick Counter                                       */
/* ======================================================================== */
void SysTick_Handler(void)
{
    s_tick_ms++;
}

/* ======================================================================== */
/* ADC1 IRQ Handler - Main FOC / BLDC Control Loop                           */
/* ======================================================================== */
/* Triggered by the ADC end-of-sequence interrupt at the PWM rate (20 kHz).
 * Reads all 7 ADC channels, runs the rotor observer, executes either the
 * FOC current loop (Clarke -> Park -> PI -> inverse Park -> SVPWM) or the
 * BLDC six-step commutation, and writes the PWM duty cycles.
 *
 * ALL arithmetic is Q16.16 fixed-point - no floating-point.                 */
void ADC1_IRQHandler(void)
{
    /* --- Clear ADC interrupt flags --- */
    if (READ_BIT(ADC1->ISR, ADC_ISR_EOS_Msk) != 0U) {
        SET_BIT(ADC1->ISR, ADC_ISR_EOS_Msk);   /* write-1-to-clear */
    }
    if (READ_BIT(ADC1->ISR, ADC_ISR_OVR_Msk) != 0U) {
        SET_BIT(ADC1->ISR, ADC_ISR_OVR_Msk);
    }

    /* ================================================================== */
    /* Step 1: Read ADC channels from the DMA buffer                      */
    /* ================================================================== */
    uint16_t ia_raw   = adc_get_raw(ADC_IDX_IA);
    uint16_t ib_raw   = adc_get_raw(ADC_IDX_IB);
    uint16_t vol_u    = adc_get_raw(ADC_IDX_VOL_U);
    uint16_t vol_v    = adc_get_raw(ADC_IDX_VOL_V);
    uint16_t vol_w    = adc_get_raw(ADC_IDX_VOL_W);
    uint16_t vbus_raw = adc_get_raw(ADC_IDX_VBUS);
    uint16_t temp_raw = adc_get_raw(ADC_IDX_TEMP);

    /* ================================================================== */
    /* Step 2: Convert to physical values in Q16.16                        */
    /* ================================================================== */
    /* Phase currents: subtract zero-current offset, apply gain.           */
    int16_t ia_corr = (int16_t)ia_raw - (int16_t)adc_get_offset_a();
    int16_t ib_corr = (int16_t)ib_raw - (int16_t)adc_get_offset_b();

    q16_t ia = Q16_MUL(INT_TO_Q16((int32_t)ia_corr), CURRENT_GAIN_Q16);
    q16_t ib = Q16_MUL(INT_TO_Q16((int32_t)ib_corr), CURRENT_GAIN_Q16);
    q16_t ic = -ia - ib;          /* 2-shunt: ic = -(ia + ib)            */

    /* Bus voltage. */
    q16_t vbus = Q16_MUL(INT_TO_Q16((int32_t)vbus_raw), VBUS_GAIN_Q16);

    /* Phase voltages (ADC-sensed, relative to ground). */
    q16_t va = Q16_MUL(INT_TO_Q16((int32_t)vol_u), PHASE_VOLT_GAIN_Q16);
    q16_t vb = Q16_MUL(INT_TO_Q16((int32_t)vol_v), PHASE_VOLT_GAIN_Q16);
    q16_t vc = Q16_MUL(INT_TO_Q16((int32_t)vol_w), PHASE_VOLT_GAIN_Q16);

    /* ================================================================== */
    /* Step 3: Clarke transform (3-phase -> 2-phase)                       */
    /* ================================================================== */
    /* Done before the observer so it can consume ialpha/ibeta directly.  */
    /*   ialpha = ia
     *   ibeta  = (ia + 2*ib) / sqrt(3)                                   */
    q16_t ialpha = ia;
    q16_t ibeta  = Q16_MUL(ia + (ib << 1), Q16_1_SQRT3);

    foc_state.ialpha = ialpha;
    foc_state.ibeta  = ibeta;

    /* ================================================================== */
    /* Step 4: Run rotor observer to obtain the electrical angle + speed   */
    /* ================================================================== */
    /* observer_run_isr uses the PREVIOUS cycle's voltage commands (still
     * in foc_state.valpha / vbeta) together with this cycle's currents. */
    observer_run_isr(&observer_state,
                      foc_state.valpha, foc_state.vbeta,
                      ialpha, ibeta, vbus);

    uint16_t angle = observer_get_angle(&observer_state);

    /* Convert observer electrical speed (rad/s Q16.16) to mechanical
     * ERPM (Q16.16) using integer arithmetic.
     *   ERPM = rad/s * 60 / (2*pi*poles)
     * In Q16.16: speed_erpm_q16 = speed_rad_q16 * 60 / (2*pi*poles)
     * We precompute 60/(2*pi) = 9.5493 in Q16 = 625836 then / poles.   */
    q16_t speed_rad = observer_get_speed(&observer_state);
    foc_state.speed_rad = speed_rad;
    {
        /* rad/s -> ERPM scale in Q16.16 = 60/(2*pi) ~ 9.5493 */
        q16_t radps_to_erpm_perpole = (q16_t)625836;
        int32_t poles = (int32_t)mc_conf.motor_poles;
        if (poles < 1) poles = 1;
        /* Q16_MUL handles the shift by 16 automatically;
         * we then divide by poles. */
        q16_t speed_erpm = Q16_MUL(speed_rad, radps_to_erpm_perpole) / poles;
        foc_state.speed_erpm = speed_erpm;
    }

    /* --- Low-speed gain scheduling (ISR-safe) ---
     * Blends current-loop + PLL gains toward their low-speed values
     * when |ERPM| < foc_low_speed_thr, with hysteresis.  The extended
     * variant also forwards the updated PLL gains to the observer so
     * the PLL stays locked at low BEMF.                                  */
    foc_low_speed_update_with_observer(&foc_state, &mc_conf,
                                       foc_state.speed_erpm,
                                       (void *)&observer_state);

    /* ================================================================== */
    /* Step 5: Dispatch to FOC or BLDC commutation                        */
    /* ================================================================== */
    if (mc_conf.comm_mode == (uint8_t)COMM_MODE_BLDC) {
        /* --- BLDC six-step mode --- */
        bldc_run_foc_dispatch(vol_u, vol_v, vol_w, vbus_raw, temp_raw);
    } else {
        /* --- FOC mode ---
         * New foc module handles Park / PI / decoupling / BEMF-FF /
         * inverse-Park.  We still call through foc_run_loop for
         * backwards compatibility; it now forwards to foc_current_loop_isr. */
        foc_run_loop(ialpha, ibeta, vbus, angle);
    }

    /* ================================================================== */
    /* Step 6: Update motor state (telemetry)                              */
    /* ================================================================== */
    /* Convert the fixed-point results to float for the application layer.
     * These are simple conversions at the tail of the ISR; the core FOC
     * math above is pure integer.                                        */
    motor_state.ia   = Q16_TO_FLOAT(ia);
    motor_state.ib   = Q16_TO_FLOAT(ib);
    motor_state.ic   = Q16_TO_FLOAT(ic);
    motor_state.va   = Q16_TO_FLOAT(va);
    motor_state.vb   = Q16_TO_FLOAT(vb);
    motor_state.vc   = Q16_TO_FLOAT(vc);
    motor_state.id   = Q16_TO_FLOAT(foc_state.id);
    motor_state.iq   = Q16_TO_FLOAT(foc_state.iq);
    motor_state.vd   = Q16_TO_FLOAT(foc_state.vd);
    motor_state.vq   = Q16_TO_FLOAT(foc_state.vq);
    motor_state.vbus = Q16_TO_FLOAT(vbus);
    motor_state.position = (float)angle * (6.28318530718f / 65536.0f);
    motor_state.speed    = foc_radps_to_erpm(speed_rad, mc_conf.motor_poles);

    /* --- Temperature conversion (slow; update only every 100th ISR) ---
     * The motor / MOSFET temperatures are low-bandwidth quantities, so
     * we compute them at 200 Hz (every 100th PWM cycle = 5 ms) using
     * float math.  The rest of the ISR remains integer-only.           */
    {
        static uint8_t s_temp_divider = 0;
        if (s_temp_divider == 0U) {
            /* NTC thermistor: 10 kΩ at 25°C, B=3950, divider 10 kΩ
             * Vout = Vref * R_ntc / (R_ntc + R_divider)
             * R_ntc = R_divider * Vout / (Vref - Vout)
             * Then T = 1 / (1/T25 + (1/B) * ln(R_ntc / R25)) - 273.15     */
            float vref = 3.3f;
            float vout = (float)temp_raw * (vref / 4095.0f);
            float r_ntc;
            if (vout > 0.001f && vout < (vref - 0.001f)) {
                r_ntc = NTC_R_DIVIDER * vout / (vref - vout);
                float inv_t = 1.0f / 298.15f +
                    (1.0f / NTC_BETA) * __builtin_logf(r_ntc / NTC_R25);
                motor_state.temp_mosfet = 1.0f / inv_t - 273.15f;
            } else {
                motor_state.temp_mosfet = 25.0f;
            }
            /* Motor temperature: estimate from MOSFET temp with a
             * thermal lag filter and a typical 10 degC offset.  This
             * is a placeholder until an actual motor-mounted NTC is
             * available.                                                   */
            float t_mot_est = motor_state.temp_mosfet + 10.0f;
            motor_state.temp_motor = motor_state.temp_motor
                + 0.05f * (t_mot_est - motor_state.temp_motor);
        }
        s_temp_divider++;
    }
}

/* ======================================================================== */
/* FOC Current Loop (called from ADC1_IRQHandler)                            */
/* ======================================================================== */
/* Wrapper around foc_current_loop_isr() from motor/foc.c.  Keeps the same
 * signature used previously; the inner implementation now includes d/q axis
 * decoupling, back-EMF feedforward and on-line gain scheduling.
 * After the current loop we apply SVPWM to produce 3-phase duty cycles.      */
static void foc_run_loop(q16_t ialpha, q16_t ibeta, q16_t vbus,
                         uint16_t angle)
{
    foc_current_loop_isr(&foc_state, &mc_conf,
                         ialpha, ibeta, vbus, angle);

    /* Convert the alpha-beta voltage commands to 3-phase duty cycles
     * and write to the TIM1 compare registers. */
    svpwm_set_duty(foc_state.valpha, foc_state.vbeta, vbus);
}

/* ======================================================================== */
/* SVPWM - Space Vector PWM with Midpoint Injection                          */
/* ======================================================================== */
/* Converts alpha-beta voltage commands to three PWM duty cycles and writes
 * them to TIM1 CCR1/CCR2/CCR3. Uses the min/max midpoint injection method
 * which is equivalent to SVPWM and computationally cheap on Cortex-M0+.    */
static void svpwm_set_duty(q16_t valpha, q16_t vbeta, q16_t vbus)
{
    /* Cached reciprocal of vbus. A 64-bit division on Cortex-M0+ costs
     * tens of cycles; vbus changes slowly, so we recompute 1/vbus only
     * when the bus voltage has moved by more than ~0.4%.               */
    static q16_t s_vbus_cache = 0;
    static q16_t s_vbus_inv   = 0;

    /* Inverse Clarke: alpha-beta -> 3-phase phase voltages. */
    q16_t va = valpha;
    q16_t vb = -Q16_MUL(valpha, Q16_HALF) + Q16_MUL(vbeta, Q16_SQRT3_2);
    q16_t vc = -Q16_MUL(valpha, Q16_HALF) - Q16_MUL(vbeta, Q16_SQRT3_2);

    /* SVPWM midpoint injection: subtract (max + min) / 2 from each phase. */
    q16_t vmax = va;
    q16_t vmin = va;
    if (vb > vmax) vmax = vb;
    if (vb < vmin) vmin = vb;
    if (vc > vmax) vmax = vc;
    if (vc < vmin) vmin = vc;

    q16_t vmid = (vmax + vmin) >> 1;   /* (vmax + vmin) / 2 */

    va -= vmid;
    vb -= vmid;
    vc -= vmid;

    /* Convert phase voltage to duty ratio:  duty = v / vbus + 0.5
     * Recompute the reciprocal only when vbus drifts. */
    q16_t diff = vbus - s_vbus_cache;
    if (diff < 0) diff = -diff;
    if (s_vbus_cache == 0 || diff > (s_vbus_cache >> 8)) {
        s_vbus_cache = vbus;
        if (vbus > 0) {
            s_vbus_inv = Q16_DIV(Q16_ONE, vbus);
        }
    }
    q16_t vbus_inv = s_vbus_inv;

    q16_t duty_a = Q16_MUL(va, vbus_inv) + Q16_HALF;
    q16_t duty_b = Q16_MUL(vb, vbus_inv) + Q16_HALF;
    q16_t duty_c = Q16_MUL(vc, vbus_inv) + Q16_HALF;

    /* Saturate to [0, Q16_ONE]. */
    duty_a = q16_sat(duty_a, 0, Q16_ONE);
    duty_b = q16_sat(duty_b, 0, Q16_ONE);
    duty_c = q16_sat(duty_c, 0, Q16_ONE);

    /* Convert Q16.16 duty ratio to timer compare value:  CCR = duty * ARR. */
    uint32_t arr = TIM1->ARR;
    q16_t arr_q16 = INT_TO_Q16((int32_t)arr);

    uint16_t ccr_a = (uint16_t)Q16_TO_INT(Q16_MUL(duty_a, arr_q16));
    uint16_t ccr_b = (uint16_t)Q16_TO_INT(Q16_MUL(duty_b, arr_q16));
    uint16_t ccr_c = (uint16_t)Q16_TO_INT(Q16_MUL(duty_c, arr_q16));

    /* Write compare registers (preloaded via OCxPE, update on next ARR). */
    TIM1->CCR1 = ccr_a;
    TIM1->CCR2 = ccr_b;
    TIM1->CCR3 = ccr_c;
}

/* ======================================================================== */
/* BLDC Dispatch Helper (called from ADC1_IRQHandler)                        */
/* ======================================================================== */
/* Reorders the ADC samples into the layout expected by bldc_run_isr() and
 * invokes the six-step commutation.                                        */
static void bldc_run_foc_dispatch(uint16_t vol_u, uint16_t vol_v,
                                  uint16_t vol_w, uint16_t vbus_raw,
                                  uint16_t temp_raw)
{
    uint16_t adc_values[BLDC_ADC_COUNT];

    adc_values[BLDC_ADC_IA]   = adc_get_raw(ADC_IDX_IA);
    adc_values[BLDC_ADC_IB]   = adc_get_raw(ADC_IDX_IB);
    adc_values[BLDC_ADC_VA]   = vol_u;
    adc_values[BLDC_ADC_VB]   = vol_v;
    adc_values[BLDC_ADC_VC]   = vol_w;
    adc_values[BLDC_ADC_VBUS] = vbus_raw;
    adc_values[BLDC_ADC_TEMP] = temp_raw;

    bldc_run_isr(&bldc_state, adc_values);
}

/* ======================================================================== */
/* USART1 IRQ Handler - UART RX / TX                                         */
/* ======================================================================== */
void USART1_IRQHandler(void)
{
    uint32_t isr = USART1->ISR;

    /* --- Receive: data register not empty --- */
    if ((isr & USART_ISR_RXNE_Msk) != 0U) {
        uint8_t byte = (uint8_t)(USART1->RDR & 0xFFU);
        uart_rx_push(byte);
    }

    /* --- Transmit: data register empty - feed next byte --- */
    if ((isr & USART_ISR_TXE_Msk) != 0U) {
        uart_tx_drain();
    }

    /* --- Transmit complete --- */
    if ((isr & USART_ISR_TC_Msk) != 0U) {
        WRITE_REG(USART1->ICR, USART_ISR_TC_Msk);   /* clear TC flag */
    }

    /* --- Clear error flags (overrun, noise, framing, parity) --- */
    if ((isr & (USART_ISR_ORE_Msk | USART_ISR_NE_Msk |
                USART_ISR_FE_Msk  | USART_ISR_PE_Msk)) != 0U) {
        WRITE_REG(USART1->ICR, USART_ISR_ORE_Msk | USART_ISR_NE_Msk |
                               USART_ISR_FE_Msk  | USART_ISR_PE_Msk);
    }
}

/* ======================================================================== */
/* TIM2 IRQ Handler - PPM Pulse-Width Capture                                */
/* ======================================================================== */
void TIM2_IRQHandler(void)
{
    /* Capture/Compare 1 interrupt (rising / falling edge on PB4). */
    if ((TIM2->SR & TIM_SR_CC1IF_Msk) != 0U) {
        s_ppm_pulse_ticks = TIM2->CCR1;   /* save captured count            */
        SET_BIT(TIM2->SR, TIM_SR_CC1IF_Msk);   /* clear CC1 flag */
    }

    /* Update event (counter overflow). */
    if ((TIM2->SR & TIM_SR_UIF_Msk) != 0U) {
        SET_BIT(TIM2->SR, TIM_SR_UIF_Msk);     /* clear update flag */
    }
}

/* ======================================================================== */
/* PPM Pulse-Width Accessor (strong override of weak stub in app_ppm.c)      */
/* ======================================================================== */
/* Returns the latest PPM pulse width in milliseconds.  Assumes TIM2 runs at
 * 1 MHz (24 MHz HCLK / prescaler 24 = 1 us per tick), so ms = ticks / 1000.
 * Adjust the divisor if timer_init_ppm() uses a different prescaler.       */
float ppm_get_pulse_ms(void)
{
    return (float)s_ppm_pulse_ticks / 1000.0f;
}

/* ======================================================================== */
/* UART TX Drain - Weak Stub (overridden by driver/mcu/uart.c)               */
/* ======================================================================== */
/* Called from USART1_IRQHandler when the TX register is empty. The real
 * implementation in the UART driver feeds the next byte from the TX ring
 * buffer. Until that driver exists this weak no-op keeps the link clean.    */
__attribute__((weak)) void uart_tx_drain(void)
{
    /* no-op - overridden by driver/mcu/uart.c */
}
