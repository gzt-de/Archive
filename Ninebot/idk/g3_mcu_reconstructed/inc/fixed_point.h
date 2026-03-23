/**
 * @file fixed_point.h
 * @brief Q15/Q30 fixed-point math types, macros, and utility declarations.
 *
 * The G3 MCU firmware uses Q15 fixed-point extensively:
 *   1.0  = 0x8000  = 32768
 *   0.5  = 0x4000  = 16384
 *  -1.0  = 0x8000  (as signed int16)
 *
 * For 32-bit Q15: the integer part is in bits [30:15], fractional in [14:0].
 */

#ifndef FIXED_POINT_H
#define FIXED_POINT_H

#include <stdint.h>

/* ---- Q15 fixed-point ---------------------------------------------------- */

typedef int32_t  q15_t;     /* 32-bit Q15: 1 sign + 16 int + 15 frac */
typedef int32_t  q30_t;     /* 32-bit Q30: 1 sign + 1 int  + 30 frac */
typedef int16_t  q15s_t;    /* 16-bit Q15: 1 sign + 0 int  + 15 frac */

#define Q15_ONE         32768       /* 1.0 in Q15 */
#define Q15_HALF        16384       /* 0.5 in Q15 */
#define Q15_SHIFT       15
#define Q15_MAX         0x7FFFFFFF
#define Q15_MIN         (-0x7FFFFFFF - 1)

/* Convert integer to Q15 */
#define INT_TO_Q15(x)   ((q15_t)((x) << Q15_SHIFT))

/* Saturated Q15 multiply: (a * b) >> 15 */
static inline q15_t q15_mul(q15_t a, q15_t b)
{
    int64_t tmp = (int64_t)a * b;
    return (q15_t)(tmp >> Q15_SHIFT);
}

/* Saturated Q15 multiply with rounding */
static inline q15_t q15_mul_rnd(q15_t a, q15_t b)
{
    int64_t tmp = (int64_t)a * b + (1 << (Q15_SHIFT - 1));
    return (q15_t)(tmp >> Q15_SHIFT);
}

/* Q30 multiply: (a * b) >> 30 */
static inline q30_t q30_mul(q30_t a, q30_t b)
{
    int64_t tmp = (int64_t)a * b;
    return (q30_t)(tmp >> 30);
}

/* Q15 saturating add */
static inline q15_t q15_sat_add(q15_t a, q15_t b)
{
    int64_t sum = (int64_t)a + b;
    if (sum > Q15_MAX) return Q15_MAX;
    if (sum < Q15_MIN) return Q15_MIN;
    return (q15_t)sum;
}

/* Q15 division: (a << 15) / b */
q15_t q15_div(q15_t a, q15_t b);

/* Q30 division: (a << 30) / b */
q30_t q30_div(q30_t a, q30_t b);

/* ---- Software float (single-precision) ---------------------------------- */
/* These mirror the IDA-decompiled __aeabi_f* functions */

uint32_t soft_float_add(uint32_t a, uint32_t b);       /* sub_80012B8 */
uint32_t soft_float_sub(uint32_t a, uint32_t b);       /* sub_800135C */
uint32_t soft_float_mul(uint32_t a, uint32_t b);       /* sub_8001368 */
uint32_t soft_float_div(uint32_t a, uint32_t b);       /* sub_8001436 */
int32_t  soft_float_to_int(uint32_t f);                 /* sub_800180A */
uint32_t soft_float_from_int(int32_t i);                /* sub_8001D2C */
uint32_t soft_float_ldexp(uint32_t f, int exp);         /* sub_80013CC */

/* Software double-precision */
uint64_t soft_double_add(uint64_t a, uint64_t b);      /* sub_80014B2 */
uint64_t soft_double_sub(uint64_t a, uint64_t b);      /* sub_80015F4 */
uint64_t soft_double_mul(uint64_t a, uint64_t b);      /* sub_8001600 */
uint64_t soft_double_div(uint64_t a, uint64_t b);      /* sub_80016E4 */
int32_t  soft_double_to_int(uint64_t d);                /* sub_8001864 */

/* ---- Fast trigonometry (lookup-table based) ------------------------------ */

/**
 * Fast sine using lookup table at loc_800EAD0 (512-entry, Q15).
 * Input: angle in Q15 electrical radians (full circle = 2^32).
 * Output: Q15 sine value.
 * Maps to sub_8001AF8 / sub_8001C38.
 */
q15_t fast_sin_q15(int32_t angle);
q15_t fast_cos_q15(int32_t angle);

/**
 * Integer square root (Q15 input, Q15 output).
 * Maps to sub_8001CC0. Uses Newton-Raphson with LUT at loc_800DAD0.
 */
q15_t fast_sqrt_q15(uint32_t val);

/* ---- Utility ------------------------------------------------------------- */

/**
 * Clamp value between min and max.
 */
static inline int32_t clamp_i32(int32_t val, int32_t lo, int32_t hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static inline int32_t abs_i32(int32_t val)
{
    return (val < 0) ? -val : val;
}

/**
 * Check for addition overflow (maps to sub_8002874).
 */
static inline int sat_add_check(int32_t a, int32_t b)
{
    if (a > 0 && b > 0 && (0x7FFFFFFF - b) < a) return 1;
    if (a < 0 && b < 0 && (int32_t)(0x80000000U - (uint32_t)b) > a) return -1;
    return 0;
}

#endif /* FIXED_POINT_H */
