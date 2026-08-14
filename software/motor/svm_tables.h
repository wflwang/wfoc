/*
 * svm_tables.h - Space Vector PWM Lookup Tables (Q16.16)
 *
 * Precomputed SVPWM sector tables and duty-cycle permutation maps so the
 * FOC ISR does not need to recompute sector logic every cycle. The on-the-fly
 * T1/T2 calculation in foc.c uses the Q16_SQRT3 / Q16_SQRT3_2 constants from
 * fixedpoint.h together with the permutation table defined here.
 *
 * Target: ARM Cortex-M0+ (CIU32F003x5) - No FPU.
 * All values are Q16.16 fixed-point (no floating-point).
 *
 * SVPWM background:
 *   The alpha-beta voltage vector is decomposed into two adjacent active
 *   vectors (T1, T2) plus zero vectors (T0/2 each). The six sectors span 60
 *   electrical degrees each:
 *
 *     Sector 1: 0   - 60  deg   vectors V1(100) V2(110)
 *     Sector 2: 60  - 120 deg   vectors V2(110) V3(010)
 *     Sector 3: 120 - 180 deg   vectors V3(010) V4(011)
 *     Sector 4: 180 - 240 deg   vectors V4(011) V5(001)
 *     Sector 5: 240 - 300 deg   vectors V5(001) V6(101)
 *     Sector 6: 300 - 360 deg   vectors V6(101) V1(100)
 *
 *   After T1/T2 are computed, the three phase duty cycles (Ta, Tb, Tc) are
 *   assigned to phases A, B, C in a sector-dependent order. The permutation
 *   table below encodes that order so the ISR performs only table lookups
 *   and a few multiplies.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under the MIT License
 */

#ifndef SVM_TABLES_H
#define SVM_TABLES_H

#include <stdint.h>
#include "util/fixedpoint.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* Phase Index Helpers (mirror BLDC_PHASE_* when bldc.h is not included)     */
/* ======================================================================== */
#ifndef BLDC_PHASE_A
#define BLDC_PHASE_A        0
#define BLDC_PHASE_B        1
#define BLDC_PHASE_C        2
#endif

/* ======================================================================== */
/* SVPWM Sector Count                                                         */
/* ======================================================================== */
#define SVM_SECTOR_COUNT     6      /* Sectors 1..6 (stored 0..5)            */
#define SVM_SECTOR_ANGLE     10923  /* 65536 / 6 = 60 deg in angle_16 units */

/* ======================================================================== */
/* Duty-Cycle Permutation Table                                              */
/* ======================================================================== */
/* After the FOC ISR computes the three raw duty-slot widths:
 *
 *   t_a = (T0/2)            (smallest, zero-vector split)
 *   t_b = (T0/2) + T1       (middle)
 *   t_c = (T0/2) + T1 + T2  (largest)
 *
 * they must be assigned to the physical phases (A, B, C) depending on the
 * active sector. Each entry maps {t_a, t_b, t_c} -> phase index.
 *
 *   svm_phase_perm[sector][0] = phase that receives t_a  (smallest)
 *   svm_phase_perm[sector][1] = phase that receives t_b  (middle)
 *   svm_phase_perm[sector][2] = phase that receives t_c  (largest)
 *
 * Phase indices: 0 = A, 1 = B, 2 = C.
 */
static const uint8_t svm_phase_perm[SVM_SECTOR_COUNT][3] = {
    /* S1 (0-60):    A=max, B=mid, C=min   vectors V1(100) V2(110) */
    { BLDC_PHASE_C, BLDC_PHASE_B, BLDC_PHASE_A },
    /* S2 (60-120):  B=max, A=mid, C=min   vectors V2(110) V3(010) */
    { BLDC_PHASE_C, BLDC_PHASE_A, BLDC_PHASE_B },
    /* S3 (120-180): B=max, C=mid, A=min   vectors V3(010) V4(011) */
    { BLDC_PHASE_A, BLDC_PHASE_C, BLDC_PHASE_B },
    /* S4 (180-240): C=max, B=mid, A=min   vectors V4(011) V5(001) */
    { BLDC_PHASE_A, BLDC_PHASE_B, BLDC_PHASE_C },
    /* S5 (240-300): C=max, A=mid, B=min   vectors V5(001) V6(101) */
    { BLDC_PHASE_B, BLDC_PHASE_A, BLDC_PHASE_C },
    /* S6 (300-360): A=max, C=mid, B=min   vectors V6(101) V1(100) */
    { BLDC_PHASE_B, BLDC_PHASE_C, BLDC_PHASE_A },
};

/* ======================================================================== */
/* Sector Selection from Angle (uint16_t 0-65535 = 0-360 deg)                 */
/* ======================================================================== */
static inline uint8_t svm_sector_from_angle(uint16_t angle)
{
    return (uint8_t)(angle / SVM_SECTOR_ANGLE);   /* 0..5 */
}

/* ======================================================================== */
/* Sector Determination from X, Y, Z (legacy sign-based method)              */
/* ======================================================================== */
/* The classic SVPWM sector-finding algorithm computes three auxiliary values:
 *
 *   X = Q16_SQRT3 * Vbeta                              (Q16.16)
 *   Y = (Q16_SQRT3 * Vbeta + 3 * Valpha) / 2           (Q16.16)
 *   Z = (Q16_SQRT3 * Vbeta - 3 * Valpha) / 2 ... sign-flipped variant
 *
 * and inspects the signs of X, Y, Z to pick the sector. The lookup below maps
 * the 3-bit sign pattern (N = N2<<2 | N1<<1 | N0, where Ni=1 if value>0) to
 * the sector index 1..6 (0 is unused / invalid).
 *
 *   N  | Sector
 *   ---+-------
 *    3 |   1
 *    1 |   2
 *    5 |   3
 *    4 |   4
 *    6 |   5
 *    2 |   6
 */
static const uint8_t svm_sector_from_n[8] = {
    0,  /* N=0 unused */
    2,  /* N=1 */
    6,  /* N=2 */
    1,  /* N=3 */
    4,  /* N=4 */
    3,  /* N=5 */
    5,  /* N=6 */
    0,  /* N=7 unused */
};

/* ======================================================================== */
/* T1 / T2 Selection per Sector                                              */
/* ======================================================================== */
/* Once the sector is known, the durations of the two adjacent active vectors
 * are selected from {X, Y, Z}. This table gives, for each sector (0-indexed),
 * which of X/Y/Z is T1 and which is T2:
 *
 *   Sector 1: T1 =  Z, T2 =  Y
 *   Sector 2: T1 =  X, T2 = -Z
 *   Sector 3: T1 = -Z, T2 =  X
 *   Sector 4: T1 = -Y, T2 = -X
 *   Sector 5: T1 =  Y, T2 = -X
 *   Sector 6: T1 = -X, T2 = -Y
 *
 * Encoded as: svm_t1_sel[sector] = { 0=X, 1=Y, 2=Z } (sign handled by caller)
 */
static const uint8_t svm_t1_sel[SVM_SECTOR_COUNT] = { 2, 0, 2, 1, 1, 0 };
static const uint8_t svm_t2_sel[SVM_SECTOR_COUNT] = { 1, 2, 0, 0, 2, 1 };

/* Sign multipliers for T1/T2 per sector (+1 or -1 in plain int). */
static const int8_t svm_t1_sign[SVM_SECTOR_COUNT] = { +1, +1, -1, -1, +1, -1 };
static const int8_t svm_t2_sign[SVM_SECTOR_COUNT] = { +1, -1, +1, -1, -1, +1 };

/* ======================================================================== */
/* SVPWM Saturation Limits                                                   */
/* ======================================================================== */
/* Maximum reference vector magnitude for linear (non-overmodulated) SVPWM:
 *   |Vref|_max = Vbus / sqrt(3)
 * In Q16.16 the normalization factor 1/sqrt(3) = 37837 (Q16_ONE / Q16_SQRT3).
 */
#define SVM_INV_SQRT3_Q16   37837   /* 65536 / sqrt(3), used for linear limit */

/* Overmodulation starts beyond this magnitude (sector-sixth boundary). */
#define SVM_OVERMOD_LIMIT   (Q16_ONE - (Q16_ONE >> 4))   /* 0.9375 in Q16    */

#ifdef __cplusplus
}
#endif

#endif /* SVM_TABLES_H */
