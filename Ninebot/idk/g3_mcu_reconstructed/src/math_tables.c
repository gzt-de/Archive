/**
 * @file math_tables.c
 * @brief Fixed-point math implementations and lookup tables.
 *
 * Reconstructed from:
 *   sub_8001AF8  — fast_sin_q15()   (with phase offset)
 *   sub_8001C38  — fast_cos_q15()   (without phase offset)
 *   sub_8001CC0  — fast_sqrt_q15()  (Newton-Raphson with LUT)
 *   sub_8001B84  — q15_div()
 *   sub_8001D50  — q30_div()
 *   sub_8001C14  — q15_mul_sat()
 *   sub_8001DE0  — q30_mul()
 *   sub_8001DF0  — q30_mul_rnd()
 *   sub_8001AA0  — lz_decompress()  (for decompressing embedded data)
 *
 * Lookup tables:
 *   loc_800EAD0  — 512-entry sine table (Q31)
 *   loc_800DAD0  — inverse sqrt initial guess table
 *   loc_800F30C  — ADC-to-temperature conversion (236 entries)
 */

#include <stdint.h>
#include "fixed_point.h"

/* ========================================================================== */
/*  Sine lookup table (512 entries, Q31)                                       */
/*  Located at 0x800EAD0 in flash.                                             */
/*  PLACEHOLDER — actual values from firmware flash dump needed.               */
/* ========================================================================== */

const int32_t sin_table_512[512] = {
    /* Entry[i] = (int32_t)(sin(i * pi / 1024) * 2^31)
     * Only first quadrant stored; symmetry used for others.
     * These are placeholder values computed from the mathematical formula. */
    0, 6588397, 13176491, 19763880, 26350264, 32935339, 39518805, 46100360,
    52679700, 59256527, 65830535, 72401428, 78968901, 85532660, 92092407,
    98647847, /* ... remaining 496 entries ... */
    /* Fill with: (int32_t)(sin(i * M_PI / 1024.0) * 2147483648.0) */
    [16 ... 511] = 0  /* Placeholder — needs actual flash data */
};

/* ========================================================================== */
/*  Inverse square root initial guess table                                    */
/*  Located at 0x800DAD0 in flash. 1024 entries.                              */
/* ========================================================================== */

const uint32_t isqrt_table[1024] = {
    /* Entry[i] = (uint32_t)(1.0 / sqrt((i+0.5)/1024.0) * 2^30)
     * Used for Newton-Raphson sqrt convergence. */
    [0 ... 1023] = 0x40000000  /* Placeholder — needs actual flash data */
};

/* ========================================================================== */
/*  ADC-to-Temperature lookup table (236 entries)                              */
/*  Located at 0x800F30C. Maps 12-bit ADC reading to temperature.             */
/*  Temperature = table_index - 30 (in °C).                                   */
/* ========================================================================== */

const int16_t adc_temp_table[236] = {
    /* NTC thermistor curve, 12-bit ADC values.
     * Index 0 = -30°C, Index 30 = 0°C, Index 55 = 25°C, etc.
     * Placeholder linear approximation: */
    [0]   = 4050,   /* -30°C */
    [30]  = 3900,   /* 0°C */
    [55]  = 3200,   /* 25°C */
    [80]  = 2400,   /* 50°C */
    [105] = 1600,   /* 75°C */
    [130] = 900,    /* 100°C */
    [155] = 500,    /* 125°C */
    [180] = 250,    /* 150°C */
    [205] = 120,    /* 175°C */
    [235] = 50,     /* 205°C */
    /* Linear interpolation between defined points */
    /* Remaining entries filled with 0 as placeholder */
};

/* ========================================================================== */
/*  AHB/APB prescaler tables                                                   */
/*  Located at 0x800F2EC / 0x800F2D4 area.                                    */
/* ========================================================================== */

const uint8_t ahb_prescaler_table[16] = {
    0, 0, 0, 0, 0, 0, 0, 0,    /* Div 1 */
    1, 2, 3, 4,                 /* Div 2, 4, 8, 16 */
    6, 7, 8, 9                  /* Div 64, 128, 256, 512 */
};

const uint8_t apb_prescaler_table[8] = {
    0, 0, 0, 0,    /* Div 1 */
    1, 2, 3, 4     /* Div 2, 4, 8, 16 */
};

/* ========================================================================== */
/*  Fast sine (sub_8001AF8 / sub_8001C38)                                      */
/* ========================================================================== */

/**
 * Fast Q15 sine using 512-entry lookup table with linear interpolation.
 *
 * The algorithm (reconstructed from sub_8001AF8):
 *   1. Convert angle to table index using magic number 1367130551
 *      (= 2^32 / (2*pi), maps radians to uint32 range)
 *   2. Add 0x40000000 (pi/2 offset for cosine variant in sub_8001AF8)
 *   3. If in negative half (carry from <<2), flip: val = 0x80000000 - val
 *   4. Check carry from <<1 for sign
 *   5. Index = val >> 22 (top 10 bits → 1024 positions in table)
 *   6. If index==0: use simple linear approximation
 *   7. Otherwise: look up sin[index] and sin[512-index] (cos),
 *      interpolate using fractional part
 *   8. Apply sign from step 4
 *   9. Shift result >>16 to Q15
 */
q15_t fast_sin_q15(int32_t angle)
{
    /* Step 1: Convert to uint32 angle representation */
    int64_t tmp = (int64_t)ANGLE_TO_UINT32 * angle;
    int32_t phase = (int32_t)((tmp >> 32) << 16) + (int32_t)((uint32_t)(ANGLE_TO_UINT32 * (uint32_t)angle) >> 16);

    /* Step 2: Add pi/2 offset (for sine; cosine skips this) */
    phase += 0x40000000;

    /* Step 3: Fold into first quadrant */
    int neg = 0;
    if ((uint32_t)phase >= 0x80000000U) {  /* Check carry from <<2 */
        phase = 0x80000000 - phase;
    }
    if (phase < 0) {
        neg = 1;
    }

    uint32_t uphase = (uint32_t)(phase * 2);  /* <<1 */
    uint32_t index = uphase >> 22;

    int32_t result;
    if (index == 0) {
        /* Small angle: sin(x) ≈ x */
        result = (int32_t)(((uint64_t)3373255991U * uphase) >> 47);
    } else {
        /* Table lookup with interpolation */
        int32_t sin_val = sin_table_512[index];
        int32_t cos_val = sin_table_512[512 - index];
        uint32_t frac = (uint32_t)(((uint64_t)3373259426U * (uphase << 10)) >> 40);

        /* Interpolate: result = sin + (cos - sin*frac/2) * frac */
        int32_t interp = (int32_t)(((int64_t)(cos_val - (int32_t)(((uint64_t)sin_val * frac) >> 33)) * frac) >> 32);
        result = (int32_t)((uint32_t)(interp + sin_val) >> 1);
    }

    if (neg)
        result = -result;

    /* Convert from Q31-ish to Q15 */
    return (q15_t)((result >> 16) + ((result >> 15) & 1));
}

/**
 * Fast Q15 cosine — same as sine but without the pi/2 offset.
 * Maps to sub_8001C38.
 */
q15_t fast_cos_q15(int32_t angle)
{
    /* Same algorithm as fast_sin_q15 but without the +0x40000000 offset */
    return fast_sin_q15(angle + (int32_t)(0x40000000U / ANGLE_TO_UINT32 * 32768));
}

/* ========================================================================== */
/*  Fast integer square root (sub_8001CC0)                                     */
/* ========================================================================== */

/**
 * Q15 square root using Newton-Raphson with lookup table seed.
 *
 * sub_8001CC0 algorithm:
 *   1. Count leading zeros (clz)
 *   2. If zero → return 0
 *   3. Normalize input: x = input << clz
 *   4. Initial guess from isqrt_table[x >> 22]
 *   5. Two Newton-Raphson iterations:
 *      y = 2 * ((−0x40000000 − (x*y)>>32 * y)>>32 * y)>>32
 *   6. Final multiply: result = x * y
 *   7. If (clz+15) is odd, multiply by sqrt(2) ≈ 3037000500/2^32
 *   8. Shift right by (clz+15)/2
 */
q15_t fast_sqrt_q15(uint32_t val)
{
    if (val == 0) return 0;

    int clz = __builtin_clz(val);
    uint32_t x = val << clz;

    /* Initial guess from table */
    uint32_t y = isqrt_table[x >> 22];

    /* Newton-Raphson iteration 1 */
    uint32_t xy = (uint32_t)(((uint64_t)x * y) >> 32);
    uint32_t correction = (uint32_t)((uint64_t)(0xC0000000U - (uint32_t)(((uint64_t)xy * y) >> 32)) * y >> 32);
    y = 2 * correction;

    /* Newton-Raphson iteration 2 */
    xy = (uint32_t)(((uint64_t)x * y) >> 32);
    correction = (uint32_t)((uint64_t)(0xC0000000U - (uint32_t)(((uint64_t)xy * y) >> 32)) * y >> 32);
    y = 2 * correction;

    /* Final: result = x * y */
    uint32_t result = (uint32_t)(((uint64_t)x * y) >> 32);

    /* Adjust for odd shift */
    int shift = clz + 15;
    if (shift & 1) {
        result = (uint32_t)(((uint64_t)3037000500U * result) >> 32);
    }
    shift >>= 1;

    /* Shift with rounding */
    return (q15_t)((result >> shift) + ((result >> (shift - 1)) & 1));
}

/* ========================================================================== */
/*  Q15 division (sub_8001B84)                                                 */
/* ========================================================================== */

/**
 * Q15 fixed-point division: (a << 15) / b with saturation.
 *
 * Algorithm:
 *   1. Handle b==0 → return 0x7FFFFFFF
 *   2. Determine sign
 *   3. Make both positive
 *   4. Use multi-step long division with clz-based shifts
 *   5. Apply sign
 *   6. Saturate at ±0x7FFFFFFF
 */
q15_t q15_div(q15_t a, q15_t b)
{
    if (b == 0) return 0x7FFFFFFF;

    int neg = ((a ^ b) & 0x80000000) == 0;
    if (a < 0) a = -a;
    if (b < 0) b = -(q15_t)(uint32_t)b;

    /* Find safe shift amount */
    int shift = __builtin_clz(a);
    if (shift > 15) shift = 15;

    uint32_t num = (uint32_t)a << shift;
    int remaining = 15 - shift;

    uint32_t quotient = num / (uint32_t)b;

    /* Check for overflow */
    if (remaining >= __builtin_clz(quotient))
        return neg ? 0x7FFFFFFF : 0x80000000;

    uint32_t remainder = num - (uint32_t)b * quotient;

    /* Refine with remaining bits */
    while (remaining > 0) {
        int rshift = __builtin_clz(remainder);
        if (rshift > remaining) rshift = remaining;

        remaining -= rshift;
        uint32_t scaled = remainder << rshift;
        uint32_t partial = scaled / (uint32_t)b;
        remainder = scaled % (uint32_t)b;
        quotient = (quotient << rshift) + partial;
    }

    q15_t result = (q15_t)quotient;
    return neg ? result : -result;
}

/**
 * Q30 division (sub_8001D50) — same algorithm with 30-bit shift.
 */
q30_t q30_div(q30_t a, q30_t b)
{
    if (b == 0) return 0x7FFFFFFF;

    int neg = ((a ^ b) & 0x80000000) == 0;
    if (a < 0) a = -a;
    if (b < 0) b = -(q30_t)(uint32_t)b;

    int shift = __builtin_clz(a);
    if (shift > 30) shift = 30;

    uint32_t num = (uint32_t)a << shift;
    int remaining = 30 - shift;

    uint32_t quotient = num / (uint32_t)b;

    if (remaining >= __builtin_clz(quotient))
        return neg ? 0x7FFFFFFF : 0x80000000;

    uint32_t remainder = num - (uint32_t)b * quotient;

    while (remaining > 0) {
        int rshift = __builtin_clz(remainder);
        if (rshift > remaining) rshift = remaining;
        remaining -= rshift;
        uint32_t scaled = remainder << rshift;
        quotient = (quotient << rshift) + scaled / (uint32_t)b;
        remainder = scaled % (uint32_t)b;
    }

    q30_t result = (q30_t)quotient;
    return neg ? result : -result;
}

/* ========================================================================== */
/*  Q15 saturating multiply (sub_8001C14)                                      */
/* ========================================================================== */

/**
 * Q15 multiply with saturation detection.
 * If result overflows Q15 range, returns ±0x7FFFFFFF.
 */
q15_t q15_mul_sat(q15_t a, q15_t b)
{
    int64_t tmp = (int64_t)a * b;
    int32_t hi = (int32_t)(tmp >> (Q15_SHIFT + 15));

    /* Check for overflow: if high bits are not all-0 or all-1 */
    if ((hi + 1) >> 1) {
        return (hi >> 31) ^ 0x7FFFFFFF;
    }

    return (q15_t)((tmp >> Q15_SHIFT) + ((tmp >> (Q15_SHIFT - 1)) & 1));
}

/* ========================================================================== */
/*  Integer-to-float conversion (sub_8001D2C)                                  */
/* ========================================================================== */

/**
 * Convert signed integer to IEEE 754 single-precision float.
 * This is the __aeabi_i2f software float function.
 */
uint32_t soft_float_from_int(int32_t val)
{
    if (val == 0) return 0;

    uint32_t sign = val & 0x80000000;
    if (val < 0) val = -val;

    int clz = __builtin_clz(val);

    /* Mantissa: shift left by clz, then take top 23 bits */
    uint32_t mantissa = ((uint32_t)val << clz) >> 8;

    /* Exponent: 127 + (31 - clz) = 158 - clz */
    uint8_t exponent = (uint8_t)(158 - clz);

    return sign | (mantissa & 0x807FFFFF) | ((uint32_t)exponent << 23);
}

/* ========================================================================== */
/*  LZ decompression (sub_8001AA0)                                             */
/* ========================================================================== */

/**
 * Simple LZ-style decompression for embedded data.
 * Used to decompress lookup tables or other constant data from flash.
 *
 * Format: each chunk has a control byte:
 *   bits [2:0] = literal_count (0 means next byte is count)
 *   bits [7:4] = zero_count (0 means next byte is count)
 *   bit  3     = match flag (1 = copy from backref, 0 = emit zeros)
 *
 * If match: next byte = offset, copy (zero_count+2) bytes from offset back
 * If no match: emit zero_count zero bytes
 */
int lz_decompress(const uint8_t *src, uint8_t *dst, int dst_size)
{
    uint8_t *end = dst + dst_size;

    do {
        uint8_t ctrl = *src++;
        int lit_count = ctrl & 7;
        if (lit_count == 0) {
            lit_count = *src++;
        }

        int rep_count = ctrl >> 4;
        if (rep_count == 0) {
            rep_count = *src++;
        }

        /* Copy literal bytes */
        while (--lit_count > 0) {
            *dst++ = *src++;
        }

        if (ctrl & 8) {
            /* Back-reference match */
            uint8_t offset = *src++;
            uint8_t *ref = dst - offset;
            int copy_count = rep_count + 2;
            while (--copy_count >= 0) {
                *dst++ = *ref++;
            }
        } else {
            /* Zero fill */
            while (--rep_count >= 0) {
                *dst++ = 0;
            }
        }
    } while (dst < end);

    return 0;
}
