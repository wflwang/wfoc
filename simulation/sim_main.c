/*
 * sim_main.c - WFOC FOC Loop Simulation on Host PC
 *
 * Validates that the fixed-point FOC algorithm (same code path as the
 * ADC1 ISR in interrupts.c) can spin a simulated PMSM motor.
 *
 * Simulation approach:
 *   1. A simple PMSM motor model (float, for simulation only) responds
 *      to 3-phase duty cycles with phase currents.
 *   2. The FOC loop uses the REAL fixedpoint.c library (Q16.16) -
 *      identical math to the firmware ISR.
 *   3. Open-loop angle ramp for the first N cycles (like VESC detection),
 *      then switch to closed-loop using a simplified observer (atan2 of
 *      flux linkage).
 *   4. Print angle, speed, Iq, duty cycles at intervals to verify the
 *      motor accelerates.
 *
 * Build:  gcc -O2 -o sim_foc.exe sim_main.c ../software/util/fixedpoint.c -lm
 * Run:    ./sim_foc.exe
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include <stdio.h>
#include <math.h>
#include <stdint.h>

/* Include the REAL fixed-point library used by the firmware */
#include "../software/util/fixedpoint.h"

/* ======================================================================== */
/* Simulation Parameters                                                     */
/* ======================================================================== */

#define SIM_PWM_FREQ        20000       /* 20 kHz PWM                         */
#define SIM_DT              (1.0f / SIM_PWM_FREQ)  /* 50 us per step          */
#define SIM_TOTAL_STEPS     6000        /* 300 ms total simulation            */
#define SIM_PRINT_INTERVAL  200         /* Print every 200 steps (10ms)       */

/* Motor parameters (typical small BLDC/PMSM) */
#define MOTOR_R             0.5f        /* Phase resistance (ohm)             */
#define MOTOR_L             0.0005f     /* Phase inductance (H) = 0.5 mH      */
#define MOTOR_FLUX          0.005f      /* Flux linkage (Wb)                  */
#define MOTOR_POLES         7           /* 7 pole pairs (14 poles)            */
#define MOTOR_INERTIA       0.0001f     /* Rotor inertia (kg*m^2)            */
#define MOTOR_FRICTION      0.0001f     /* Viscous friction (N*m*s/rad)      */
#define MOTOR_VBUS          24.0f       /* Bus voltage (V)                    */

/* FOC gains */
#define FOC_CURRENT_KP      0.3f        /* Current loop KP                    */
#define FOC_CURRENT_KI      200.0f      /* Current loop KI                    */
#define FOC_IQ_TARGET       5.0f        /* Target Iq current (A)              */
#define FOC_CURRENT_FILTER  0.3f        /* Current LPF alpha                  */

/* Open-loop parameters */
#define OPENLOOP_STEPS      1500        /* 75 ms open-loop ramp               */
#define OPENLOOP_IQ         3.0f        /* Open-loop Iq (A)                   */
#define OPENLOOP_SPEED_INIT 30.0f       /* Initial open-loop speed (rad/s)    */
#define OPENLOOP_SPEED_RAMP 500.0f      /* Speed ramp rate (rad/s^2)          */

/* Test mode: 0=with observer, 1=perfect angle (sensor) */
#define SIM_USE_PERFECT_ANGLE 1

/* ======================================================================== */
/* PMSM Motor Model (float, simulation only)                                 */
/* ======================================================================== */
/* Simulates a PMSM in the alpha-beta frame.
 *
 *   valpha = R * ialpha + L * dialpha/dt - omega_e * flux * sin(theta_e)
 *   vbeta  = R * ibeta  + L * dibeta/dt  + omega_e * flux * cos(theta_e)
 *
 *   Mechanical:
 *   J * domega_m/dt = 1.5 * p * flux * iq - B * omega_m
 *   omega_e = p * omega_m
 *   theta_e += omega_e * dt
 */

typedef struct {
    /* Electrical state */
    float ialpha, ibeta;       /* Phase currents in alpha-beta             */
    float theta_e;             /* Electrical angle (rad)                   */
    float omega_e;             /* Electrical speed (rad/s)                 */

    /* Mechanical state */
    float omega_m;             /* Mechanical speed (rad/s)                 */
    float theta_m;             /* Mechanical angle (rad)                   */

    /* Motor parameters */
    float R, L, flux;
    float poles;
    float J, B;
} pmsm_model_t;

static void pmsm_init(pmsm_model_t *m)
{
    m->ialpha = 0.0f;
    m->ibeta  = 0.0f;
    m->theta_e = 0.0f;
    m->omega_e = 0.0f;
    m->omega_m = 0.0f;
    m->theta_m = 0.0f;

    m->R     = MOTOR_R;
    m->L     = MOTOR_L;
    m->flux  = MOTOR_FLUX;
    m->poles = MOTOR_POLES;
    m->J     = MOTOR_INERTIA;
    m->B     = MOTOR_FRICTION;
}

/* Simulate one PWM cycle.
 * Inputs: duty_a, duty_b, duty_c (0.0 to 1.0), vbus (V)
 * Outputs: updates motor state, fills ia, ib (phase currents in A) */
static void pmsm_step(pmsm_model_t *m, float duty_a, float duty_b,
                      float duty_c, float vbus, float dt,
                      float *ia, float *ib)
{
    /* Convert duty cycles to phase voltages (relative to bus midpoint) */
    float va = (duty_a - 0.5f) * vbus;
    float vb = (duty_b - 0.5f) * vbus;
    float vc = (duty_c - 0.5f) * vbus;

    /* Clarke transform: 3-phase -> alpha-beta */
    float valpha = va;
    float vbeta  = (va + 2.0f * vb) / 1.7320508f;  /* / sqrt(3) */

    /* Back-EMF */
    float bemf_alpha = -m->omega_e * m->flux * sinf(m->theta_e);
    float bemf_beta  =  m->omega_e * m->flux * cosf(m->theta_e);

    /* Electrical dynamics: v = R*i + L*di/dt + bemf
     * di/dt = (v - R*i - bemf) / L */
    float dialpha_dt = (valpha - m->R * m->ialpha - bemf_alpha) / m->L;
    float dibeta_dt  = (vbeta  - m->R * m->ibeta  - bemf_beta)  / m->L;

    m->ialpha += dialpha_dt * dt;
    m->ibeta  += dibeta_dt * dt;

    /* Inverse Clarke: alpha-beta -> 3-phase (for current output) */
    *ia = m->ialpha;
    *ib = (-m->ialpha / 2.0f) + (m->ibeta * 0.8660254f);  /* sqrt(3)/2 */

    /* Park transform to get Iq for torque calculation */
    float sin_e = sinf(m->theta_e);
    float cos_e = cosf(m->theta_e);
    float id = m->ialpha * cos_e + m->ibeta * sin_e;
    float iq = -m->ialpha * sin_e + m->ibeta * cos_e;

    /* Mechanical dynamics: J*domega/dt = Kt*iq - B*omega */
    float kt = 1.5f * m->poles * m->flux;
    float torque = kt * iq - m->B * m->omega_m;
    m->omega_m += (torque / m->J) * dt;
    m->theta_m += m->omega_m * dt;

    /* Update electrical angle */
    m->omega_e = m->omega_m * m->poles;
    m->theta_e += m->omega_e * dt;

    /* Wrap electrical angle to [0, 2*pi) */
    while (m->theta_e >= 2.0f * 3.14159265f) m->theta_e -= 2.0f * 3.14159265f;
    while (m->theta_e < 0.0f) m->theta_e += 2.0f * 3.14159265f;
}

/* ======================================================================== */
/* FOC Loop (Q16.16 fixed-point - same as firmware ISR)                      */
/* ======================================================================== */
/* This replicates the foc_run_loop() + svpwm_set_duty() from interrupts.c,
 * using the actual fixedpoint.c library.                                    */

typedef struct {
    q16_t ialpha, ibeta;
    q16_t id, iq;
    q16_t id_set, iq_set;
    q16_t vd, vq;
    q16_t valpha, vbeta;
    q16_t duty_a, duty_b, duty_c;
    q16_pi_t pi_id;
    q16_pi_t pi_iq;
    q16_lpf_t lpf_id;
    q16_lpf_t lpf_iq;
} sim_foc_t;

static void sim_foc_init(sim_foc_t *foc)
{
    foc->ialpha = 0; foc->ibeta = 0;
    foc->id = 0; foc->iq = 0;
    foc->id_set = 0; foc->iq_set = 0;
    foc->vd = 0; foc->vq = 0;
    foc->valpha = 0; foc->vbeta = 0;
    foc->duty_a = 0; foc->duty_b = 0; foc->duty_c = 0;

    /* PI controllers - same params as firmware defaults */
    q16_pi_init(&foc->pi_id,
                FLOAT_TO_Q16(FOC_CURRENT_KP),
                FLOAT_TO_Q16(FOC_CURRENT_KI * SIM_DT),
                FLOAT_TO_Q16(20.0f),    /* integral limit */
                FLOAT_TO_Q16(MOTOR_VBUS * 0.5f));  /* output limit */

    q16_pi_init(&foc->pi_iq,
                FLOAT_TO_Q16(FOC_CURRENT_KP),
                FLOAT_TO_Q16(FOC_CURRENT_KI * SIM_DT),
                FLOAT_TO_Q16(20.0f),
                FLOAT_TO_Q16(MOTOR_VBUS * 0.5f));

    /* LPF on measured currents */
    foc->lpf_id.alpha_q16 = FLOAT_TO_Q16(FOC_CURRENT_FILTER);
    foc->lpf_id.y = 0;
    foc->lpf_iq.alpha_q16 = FLOAT_TO_Q16(FOC_CURRENT_FILTER);
    foc->lpf_iq.y = 0;
}

/* Run one FOC cycle. Returns duty cycles in 0.0-1.0 float.
 * angle: rotor electrical angle (uint16_t 0-65535 = 0-360 deg)
 * ia, ib: measured phase currents (Q16.16 Amperes)
 * vbus: bus voltage (Q16.16 Volts) */
static void sim_foc_run(sim_foc_t *foc, uint16_t angle,
                        q16_t ia, q16_t ib, q16_t vbus,
                        float *out_da, float *out_db, float *out_dc)
{
    /* --- Clarke transform --- */
    /* ialpha = ia, ibeta = (ia + 2*ib) / sqrt(3) */
    q16_t ialpha = ia;
    q16_t ibeta  = Q16_MUL(ia + (ib << 1), ((q16_t)37837));  /* 1/sqrt(3) Q16 */

    foc->ialpha = ialpha;
    foc->ibeta  = ibeta;

    /* --- Park transform --- */
    q16_t sin_val, cos_val;
    q16_sincos(angle, &sin_val, &cos_val);

    q16_t id = Q16_MUL(ialpha, cos_val) + Q16_MUL(ibeta, sin_val);
    q16_t iq = Q16_MUL(-ialpha, sin_val) + Q16_MUL(ibeta, cos_val);

    /* LPF on measured currents */
    id = q16_lpf_update(&foc->lpf_id, id);
    iq = q16_lpf_update(&foc->lpf_iq, iq);

    foc->id = id;
    foc->iq = iq;

    /* --- PI current controllers --- */
    q16_t vd = q16_pi_update(&foc->pi_id, foc->id_set - id);
    q16_t vq = q16_pi_update(&foc->pi_iq, foc->iq_set - iq);

    foc->vd = vd;
    foc->vq = vq;

    /* --- Inverse Park --- */
    q16_t valpha = Q16_MUL(vd, cos_val) - Q16_MUL(vq, sin_val);
    q16_t vbeta  = Q16_MUL(vd, sin_val) + Q16_MUL(vq, cos_val);

    foc->valpha = valpha;
    foc->vbeta  = vbeta;

    /* --- SVPWM (midpoint injection) --- */
    q16_t va = valpha;
    q16_t vb = -Q16_MUL(valpha, Q16_HALF) + Q16_MUL(vbeta, Q16_SQRT3_2);
    q16_t vc = -Q16_MUL(valpha, Q16_HALF) - Q16_MUL(vbeta, Q16_SQRT3_2);

    /* Find max and min */
    q16_t vmax = va, vmin = va;
    if (vb > vmax) vmax = vb;
    if (vb < vmin) vmin = vb;
    if (vc > vmax) vmax = vc;
    if (vc < vmin) vmin = vc;

    q16_t vmid = (vmax + vmin) >> 1;

    va -= vmid;
    vb -= vmid;
    vc -= vmid;

    /* Convert to duty: duty = v / vbus + 0.5 */
    q16_t vbus_inv = Q16_DIV(Q16_ONE, vbus);

    q16_t duty_a = Q16_MUL(va, vbus_inv) + Q16_HALF;
    q16_t duty_b = Q16_MUL(vb, vbus_inv) + Q16_HALF;
    q16_t duty_c = Q16_MUL(vc, vbus_inv) + Q16_HALF;

    /* Saturate to [0, 1] */
    duty_a = q16_sat(duty_a, 0, Q16_ONE);
    duty_b = q16_sat(duty_b, 0, Q16_ONE);
    duty_c = q16_sat(duty_c, 0, Q16_ONE);

    foc->duty_a = duty_a;
    foc->duty_b = duty_b;
    foc->duty_c = duty_c;

    *out_da = Q16_TO_FLOAT(duty_a);
    *out_db = Q16_TO_FLOAT(duty_b);
    *out_dc = Q16_TO_FLOAT(duty_c);
}

/* ======================================================================== */
/* Simplified Observer (for closed-loop transition)                           */
/* ======================================================================== */
/* Uses the alpha-beta back-EMF to estimate angle via atan2.
 * In real firmware this is done by the SMO/MXLEMMA observer. */
static uint16_t sim_estimate_angle(float ialpha, float ibeta,
                                   float valpha, float vbeta,
                                   float R, float L, float omega_e,
                                   float flux, float dt)
{
    /* Estimate back-EMF: e = v - R*i - L*di/dt
     * Simplified: e_alpha = valpha - R*ialpha
     *             e_beta  = vbeta  - R*ibeta
     * (ignoring L*di/dt for simplicity in this simulation) */
    float e_alpha = valpha - R * ialpha;
    float e_beta  = vbeta  - R * ibeta;

    /* Angle from back-EMF:
     * e_alpha = -omega * flux * sin(theta)
     * e_beta  =  omega * flux * cos(theta)
     * theta = atan2(-e_alpha, e_beta) */
    if (fabsf(e_beta) < 0.001f && fabsf(e_alpha) < 0.001f) {
        return 0;  /* Not enough EMF yet */
    }

    float theta_est = atan2f(-e_alpha, e_beta);
    if (theta_est < 0.0f) theta_est += 2.0f * 3.14159265f;

    return (uint16_t)(theta_est * 65536.0f / (2.0f * 3.14159265f));
}

/* ======================================================================== */
/* Main Simulation                                                           */
/* ======================================================================== */
int main(void)
{
    printf("=== WFOC FOC Loop Simulation ===\n");
    printf("Motor: R=%.3f ohm, L=%.4f H, flux=%.4f Wb, poles=%d\n",
           MOTOR_R, MOTOR_L, MOTOR_FLUX, MOTOR_POLES);
    printf("Bus: %.1f V, PWM: %d Hz, dt: %.1f us\n",
           MOTOR_VBUS, SIM_PWM_FREQ, SIM_DT * 1e6f);
    printf("Target Iq: %.1f A\n", FOC_IQ_TARGET);
#if SIM_USE_PERFECT_ANGLE
    printf("Mode: PERFECT ANGLE (sensor) - validates FOC loop only\n\n");
#else
    printf("Mode: OBSERVER - validates FOC + observer\n\n");
#endif

    /* Initialize motor model */
    pmsm_model_t motor;
    pmsm_init(&motor);

    /* Initialize FOC loop */
    sim_foc_t foc;
    sim_foc_init(&foc);

    /* Simulation variables */
    uint16_t openloop_angle = 0;
    uint16_t est_angle = 0;
    int closed_loop = 0;
    float da = 0.5f, db = 0.5f, dc = 0.5f;
    float openloop_speed = OPENLOOP_SPEED_INIT;

    /* Q16 constants for conversions */
    q16_t vbus_q16 = FLOAT_TO_Q16(MOTOR_VBUS);

    printf("Step   | Angle(deg) | Speed(RPM) |   Iq(A)  |   Id(A)  | DutyA | Mode\n");
    printf("-------|------------|------------|----------|----------|-------|------\n");

    for (int step = 0; step < SIM_TOTAL_STEPS; step++) {

        /* --- Determine angle source (open-loop or closed-loop) --- */
        uint16_t foc_angle;
        if (step < OPENLOOP_STEPS) {
            /* Open-loop: gradual angle ramp (like VESC detection) */
            /* Ramp up the forced speed */
            openloop_speed += OPENLOOP_SPEED_RAMP * SIM_DT;
            if (openloop_speed > 300.0f) openloop_speed = 300.0f;

            openloop_angle += (uint16_t)(openloop_speed * 65536.0f / (2.0f * 3.14159265f) * SIM_DT);
            foc_angle = openloop_angle;
            foc.iq_set = FLOAT_TO_Q16(OPENLOOP_IQ);
            foc.id_set = 0;
            closed_loop = 0;
        } else {
            /* Closed-loop */
            if (!closed_loop) {
                printf("  >>> Switching to closed-loop at step %d (%.1f ms)\n",
                       step, step * SIM_DT * 1000.0f);
                closed_loop = 1;
            }

#if SIM_USE_PERFECT_ANGLE
            /* Use motor model's actual angle (perfect sensor) */
            foc_angle = (uint16_t)(motor.theta_e * 65536.0f / (2.0f * 3.14159265f));
#else
            /* Use observer estimate */
            est_angle = sim_estimate_angle(
                Q16_TO_FLOAT(foc.ialpha), Q16_TO_FLOAT(foc.ibeta),
                Q16_TO_FLOAT(foc.valpha), Q16_TO_FLOAT(foc.vbeta),
                MOTOR_R, MOTOR_L, motor.omega_e, MOTOR_FLUX, SIM_DT);
            foc_angle = est_angle;
#endif
            foc.iq_set = FLOAT_TO_Q16(FOC_IQ_TARGET);
            foc.id_set = 0;
        }

        /* --- Run FOC loop (fixed-point, same as firmware) --- */
        /* Convert motor model currents to Q16 */
        q16_t ia_q16 = FLOAT_TO_Q16(motor.ialpha);
        /* Inverse Clarke to get ib from alpha-beta:
         * ib = -ia/2 + ibeta * sqrt(3)/2
         * But motor model already has ibeta, so compute ib directly */
        float ib_float = (-motor.ialpha / 2.0f) + (motor.ibeta * 0.8660254f);
        q16_t ib_q16 = FLOAT_TO_Q16(ib_float);

        sim_foc_run(&foc, foc_angle, ia_q16, ib_q16, vbus_q16,
                    &da, &db, &dc);

        /* --- Step motor model with the computed duty cycles --- */
        float ia_meas, ib_meas;
        pmsm_step(&motor, da, db, dc, MOTOR_VBUS, SIM_DT, &ia_meas, &ib_meas);

        /* --- Print at intervals --- */
        if (step % SIM_PRINT_INTERVAL == 0) {
            float angle_deg = (float)foc_angle * 360.0f / 65536.0f;
            float speed_rpm = motor.omega_m * 60.0f / (2.0f * 3.14159265f);
            float iq_float = Q16_TO_FLOAT(foc.iq);
            float id_float = Q16_TO_FLOAT(foc.id);

            printf("%6d | %8.1f   | %8.1f   | %8.2f | %8.2f | %5.2f  | %s\n",
                   step, angle_deg, speed_rpm, iq_float, id_float, da,
                   closed_loop ? "CLOSED" : "OPEN");
        }
    }

    /* --- Final summary --- */
    float final_rpm = motor.omega_m * 60.0f / (2.0f * 3.14159265f);
    float final_iq = Q16_TO_FLOAT(foc.iq);
    float final_id = Q16_TO_FLOAT(foc.id);

    printf("\n=== Simulation Results ===\n");
    printf("Final mechanical speed:  %.1f RPM\n", final_rpm);
    printf("Final electrical speed:  %.1f rad/s (%.1f Hz)\n",
           motor.omega_e, motor.omega_e / (2.0f * 3.14159265f));
    printf("Final Iq (torque):       %.2f A (target: %.1f A)\n",
           final_iq, closed_loop ? FOC_IQ_TARGET : OPENLOOP_IQ);
    printf("Final Id (flux):         %.2f A (should be ~0)\n", final_id);
    printf("Final duty cycle A:      %.3f\n", da);
    printf("Final duty cycle B:      %.3f\n", db);
    printf("Final duty cycle C:      %.3f\n", dc);
    printf("Closed-loop active:      %s\n", closed_loop ? "YES" : "NO");

    if (final_rpm > 100.0f) {
        printf("\n>>> SUCCESS: Motor is spinning! (%.1f RPM)\n", final_rpm);
    } else if (final_rpm > 10.0f) {
        printf("\n>>> PARTIAL: Motor is moving slowly (%.1f RPM)\n", final_rpm);
    } else {
        printf("\n>>> FAIL: Motor did not spin (%.1f RPM)\n", final_rpm);
    }

    return 0;
}
