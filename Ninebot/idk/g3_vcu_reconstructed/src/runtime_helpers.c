/**
 * @file runtime_helpers.c
 * @brief Compiler Runtime Helpers - Soft Float, memcpy, memset, memcmp
 *
 * The STM32F103 (Cortex-M3) has no hardware FPU, so all floating-point
 * operations are done in software. The IDA decompilation shows the
 * compiler-generated soft-float routines for IEEE 754 single-precision.
 *
 * Reconstructed from:
 * sub_800124C = __aeabi_memcpy (optimized word-aligned copy)
 * sub_8001270 = __aeabi_memset (fill with byte)
 * sub_800127E = __aeabi_memclr (fill with zero)
 * sub_8001282 = memset wrapper
 * sub_8001294 = memcmp
 * sub_80012AE = __aeabi_fadd (float addition)
 * sub_8001352 = __aeabi_fsub (float subtraction, negate + add)
 * sub_8001358 = __aeabi_frsub (reverse subtract)
 * sub_800135E = __aeabi_fmul (float multiplication)
 * sub_80013C2 = __aeabi_fscale (float scale by power of 2)
 * sub_8001436 = __aeabi_fdiv (float division)
 * sub_8001600 = __aeabi_f2iz (float to int)
 * sub_8001632 = __aeabi_f2uiz (float to unsigned int)
 * sub_800165A = __aeabi_i2f (int to float)
 * sub_8001698 = __aeabi_ui2f (unsigned int to float)
 * sub_80016BE = __aeabi_fcmplt (float compare less-than)
 * sub_80011AC = __aeabi_uldivmod (unsigned 64-bit divide)
 */

#include <stdint.h>
#include <stddef.h>

/* ============================================================================
 * Memory Operations
 * ============================================================================ */

/* sub_800124C - Optimized memcpy with word-aligned fast path */
void *rt_memcpy(void *dst, const void *src, size_t n)
{
    uint8_t *d = (uint8_t *)dst;
    const uint8_t *s = (const uint8_t *)src;

    /* If both pointers are word-aligned, copy 4 bytes at a time */
    if (!(((uintptr_t)d | (uintptr_t)s) & 3)) {
        while (n >= 4) {
            *(uint32_t *)d = *(const uint32_t *)s;
            d += 4;
            s += 4;
            n -= 4;
        }
    }

    /* Copy remaining bytes */
    while (n--) {
        *d++ = *s++;
    }

    return dst;
}

/* sub_8001270 - memset */
void *rt_memset(void *dst, int c, size_t n)
{
    uint8_t *d = (uint8_t *)dst;
    while (n--) {
        *d++ = (uint8_t)c;
    }
    return dst;
}

/* sub_800127E - memclr (memset with 0) */
void *rt_memclr(void *dst, size_t n)
{
    return rt_memset(dst, 0, n);
}

/* sub_8001282 - memset wrapper (returns original pointer) */
void *rt_memset_ptr(void *dst, int c, size_t n)
{
    rt_memset(dst, c, n);
    return dst;
}

/* sub_8001294 - memcmp */
int rt_memcmp(const void *s1, const void *s2, size_t n)
{
    const uint8_t *p1 = (const uint8_t *)s1;
    const uint8_t *p2 = (const uint8_t *)s2;

    for (size_t i = 0; i < n; i++) {
        int diff = p1[i] - p2[i];
        if (diff)
            return diff;
    }
    return 0;
}

/* ============================================================================
 * Soft-Float IEEE 754 Single Precision
 *
 * These are the compiler-generated software floating point routines.
 * On a real build targeting Cortex-M3, these would be provided by the
 * compiler runtime library (libgcc or equivalent).
 *
 * We provide standard C implementations here since the decompiled
 * assembly-level implementations are not meaningful as C code.
 * ============================================================================ */

typedef union {
    float    f;
    uint32_t u;
    int32_t  i;
} float_bits_t;

/* sub_80012AE - Float addition */
float rt_fadd(float a, float b)
{
    return a + b;
}

/* sub_8001352 - Float subtraction (implemented as negate-b + add) */
float rt_fsub(float a, float b)
{
    return a - b;
}

/* sub_8001358 - Reverse subtract */
float rt_frsub(float a, float b)
{
    return b - a;
}

/* sub_800135E - Float multiplication */
float rt_fmul(float a, float b)
{
    return a * b;
}

/* sub_8001436 - Float division */
float rt_fdiv(float a, float b)
{
    if (b == 0.0f)
        return 0.0f;
    return a / b;
}

/* sub_80013C2 - Scale float by power of 2 */
float rt_fscale(float a, int exp)
{
    float_bits_t fb;
    fb.f = a;

    if (fb.u * 2 == 0)     /* Zero or denormalized */
        return a;

    int cur_exp = (int)((fb.u >> 23) & 0xFF);
    if (exp > -cur_exp) {
        fb.u += (uint32_t)exp << 23;
    } else {
        return 0.0f;
    }
    return fb.f;
}

/* sub_8001600 - Float to signed int */
int32_t rt_f2i(float a)
{
    return (int32_t)a;
}

/* sub_8001632 - Float to unsigned int */
uint32_t rt_f2ui(float a)
{
    if (a < 0.0f) return 0;
    return (uint32_t)a;
}

/* sub_800165A - Signed int to float */
float rt_i2f(int32_t a)
{
    return (float)a;
}

/* sub_8001698 - Unsigned int to float */
float rt_ui2f(uint32_t a)
{
    return (float)a;
}

/* sub_80016BE - Float compare less-than */
int rt_fcmplt(float a, float b)
{
    return (a < b) ? 1 : 0;
}

/* ============================================================================
 * 64-bit Division (sub_80011AC)
 *
 * Software 64-bit unsigned division for platforms without hardware divider.
 * ============================================================================ */
uint64_t rt_uldiv(uint64_t dividend, uint64_t divisor)
{
    if (divisor == 0)
        return 0;

    uint64_t quotient = 0;
    for (int bit = 63; bit >= 0; bit--) {
        if ((dividend >> bit) >= divisor) {
            dividend -= divisor << bit;
            quotient += (uint64_t)1 << bit;
        }
    }
    return quotient;
}

/* ============================================================================
 * Array Compare (sub_8009744)
 *
 * Byte-by-byte comparison, returns 1 if equal, 0 if different.
 * ============================================================================ */
int rt_array_compare(const uint8_t *a, const uint8_t *b, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        if (a[i] != b[i])
            return 0;
        /* Note: original uses uint8_t counter, wraps at 256 */
    }
    return 1;
}
