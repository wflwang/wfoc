/*
 * main.c - WFOC Motor Controller Firmware Entry Point
 *
 * Initializes the MCU, peripherals, motor control subsystems and runs the
 * main application loop.  The real-time FOC current loop executes from the
 * ADC1 ISR (see interrupts.c); this main loop only handles the lower-rate
 * application layer (PPM / UART commands, duty / speed / position targets,
 * configuration management) and feeds the watchdog.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include <stdint.h>

#include "hwconf.h"
#include "fw_version.h"
#include "conf/datatypes.h"
#include "conf/conf_general.h"       /* mc_conf, app_conf, conf_general_init */
#include "motor/bldc.h"
#include "motor/observer.h"          /* observer_state_t, observer_init      */
#include "motor/foc.h"               /* foc_init, foc_update_temp_comp, ...  */
#include "app/app.h"                 /* app_init, app_process                */
#include "driver/mcu/flash.h"        /* Flash EEPROM emulation               */
#include "driver/mcu/ota.h"          /* OTA upgrade module                    */
#include "driver/mcu/spi.h"          /* SPI driver for future expansion      */

#include "driver/mcu/mcu_init.h"
#include "driver/mcu/adc.h"
#include "driver/mcu/gpio.h"
#include "driver/mcu/pwm.h"
#include "driver/mcu/uart.h"
#include "driver/mcu/timer.h"

/* ======================================================================== */
/* Forward Declarations - modules implemented elsewhere                      */
/* ======================================================================== */
extern void watchdog_feed(void);          /* driver/mcu/timer.c - IWDG kick  */

/* ======================================================================== */
/* Global Instances                                                          */
/* ======================================================================== */
/* mc_conf and app_conf are owned by conf_general.c (declared extern via
 * conf_general.h).  The real-time state singletons below are shared between
 * the application layer (main loop) and the ISR control loop.              */

motor_state_t        motor_state;         /* Real-time motor telemetry       */
foc_state_t          foc_state;           /* FOC current-loop internal state */
observer_state_t     observer_state;      /* Rotor observer state            */
bldc_state_t         bldc_state;          /* BLDC six-step runtime state     */

/* ======================================================================== */
/* FOC State Initialization                                                  */
/* ======================================================================== */
/* Delegates to foc_init() from motor/foc.c which configures the PI
 * controllers, LPFs, low-speed scheduler and temperature-compensated
 * motor parameters.  Float is used here (non-ISR context).               */
static void foc_init_state(foc_state_t *foc, mc_configuration_t *conf)
{
    foc_init(foc, conf);
}

/* ======================================================================== */
/* Main Entry Point                                                          */
/* ======================================================================== */
int main(void)
{
    /* --- Stage 1: MCU core (clock, flash, NVIC, SysTick) --- */
    mcu_init();

    /* --- Stage 2: Board pin muxing (ADC, PWM, UART, PPM, SWD) --- */
    gpio_config_board();

    /* --- Stage 3: Analog front-end (ADC + DMA, synchronized to TIM1) --- */
    adc_init();
    /* Measure zero-current ADC offsets while the power stage is still off.
     * This must happen before pwm_enable() so no current flows. */
    adc_calibrate_offsets();

    /* --- Stage 4: Power-stage timer (TIM1 3-phase center PWM) --- */
    pwm_init(PWM_FREQ_HZ);

    /* --- Stage 5: Communication (USART1) --- */
    uart_init(UART_BAUDRATE);

    /* --- Stage 6: PPM pulse-width capture (TIM2 input capture) --- */
    timer_init_ppm();

    /* --- Stage 7: Flash driver and EEPROM emulation ---
     * Must be initialized BEFORE conf_general_init() so that
     * stored parameters can be loaded from flash.            */
    flash_init();
    eeprom_init();

    /* --- Stage 8: Load persistent configuration from flash ---
     * EEPROM must be initialized (Stage 7) before this call.  */
    conf_general_init();

    /* --- Stage 9: Initialize FOC state (PI controllers, LPFs) --- */
    foc_init_state(&foc_state, &mc_conf);

    /* --- Stage 10: Initialize BLDC six-step commutation --- */
    bldc_init(&mc_conf);

    /* --- Stage 11: Application layer (PPM / UART / duty / speed modes) --- */
    app_init(&mc_conf, &app_conf);

    /* --- Stage 12: Rotor position observer (SMO / MXLEMMA / HFI) --- */
    observer_init(&observer_state, &mc_conf);

    /* --- Stage 13: OTA module initialization --- */
    ota_init();

    /* --- Stage 14: SPI initialization (reserved for remote/OTA) ---
     * SPI is initialized but not actively used. Ready for future
     * remote control receiver or wireless OTA module.
     * Note: MISO is externally connected to MOSI for half-duplex. */
    spi_init(SPI_CLK_DIV_16, SPI_MODE_0, 0);

    /* --- Stage 15: Start watchdog --- */
    watchdog_init();

    /* --- Enable PWM outputs (safe start, all duty cycles are zero) --- */
    pwm_enable();

    /* --- Enable global interrupts: ISRs are now live --- */
    mcu_irq_enable();

    /* ==================================================================== */
    /* Main Application Loop                                                */
    /* ==================================================================== */
    /* The FOC current loop runs at PWM rate from ADC1_IRQHandler; this loop
     * handles the slower application logic and feeds the watchdog.
     *
     * Slow-rate (non-ISR) FOC tasks scheduled here:
     *   - 1 kHz : startup ramp, speed loop, field weakening
     *   - 100 Hz: position loop, temperature compensation
     *                                                                       */
    uint32_t last_speed_tick = 0;      /* last tick for speed loop        */
    uint32_t last_temp_tick  = 0;      /* last tick for position/temp      */

    for (;;) {
        uint32_t now = s_tick_ms;  /* volatile read once per iteration  */

        /* --- 1 kHz FOC outer loops --- */
        if ((uint32_t)(now - last_speed_tick) >= 1U) {
            last_speed_tick = now;

            if (mc_conf.comm_mode != (uint8_t)COMM_MODE_BLDC) {
                /* Order matters:
                 * 1. Position loop (100 Hz effective, but called at 1kHz)
                 * 2. Startup ramp
                 * 3. Speed loop
                 * 4. Field weakening */

                /* Position loop (only if enabled) */
                foc_position_loop(&foc_state, &mc_conf);

                /* Startup ramp */
                foc_start_ramp_step(&foc_state, &mc_conf);

                /* Speed loop */
                foc_speed_loop(&foc_state, &mc_conf);

                /* Field weakening (adjusts id_set) */
                foc_field_weakening(&foc_state, &mc_conf);
            }
        }

        /* --- 100 Hz background tasks --- */
        if ((uint32_t)(now - last_temp_tick) >= 10U) {
            last_temp_tick = now;

            if (mc_conf.comm_mode != (uint8_t)COMM_MODE_BLDC) {
                /* Temperature compensation update */
                foc_update_temp_comp(&foc_state, &mc_conf,
                                     motor_state.temp_motor,
                                     motor_state.temp_mosfet);

                /* Update position measurement from observer
                 * This integrates speed to get position estimate */
                foc_state.pos_meas = motor_state.position;
            }
        }

        /* --- Application layer (PPM / UART / duty / speed modes) --- */
        app_process();
        watchdog_feed();
    }

    /* Unreachable */
    return 0;
}
