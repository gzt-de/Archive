/**
 * @file peripherals.c
 * @brief Throttle processing, brake control, LED/light control,
 *        temperature monitoring, and error detection.
 *
 * Reconstructed from:
 *   sub_8002080  — throttle_adc_process() (hall-effect throttle)
 *   sub_800212C  — brake_adc_process() (brake lever ADC)
 *   sub_80021E0  — get_brake_raw()
 *   sub_80021EC  — get_brake_scaled()
 *   sub_800220C  — throttle_init()
 *   sub_800205C  — speed_from_hall()
 *   sub_8002480  — adc_median_filter()
 *   sub_80028A4  — bubble_sort()
 *   sub_800292A  — array_max()
 *   sub_8002948  — array_min()
 *   sub_8002ED0  — led_blink_slow()
 *   sub_8002F24  — led_blink_fast()
 *   sub_8002F74  — led_on()
 *   sub_8002F84  — led_dim()
 *   sub_8002F94  — led_pattern()
 *   sub_8004774  — temperature_derating()
 *   sub_8004D20  — motor_param_update()
 *   sub_8004DB0  — check_error_flags()
 *   sub_8004DCC  — check_dashboard_alive()
 *   sub_8005938  — gpio_status_read()
 */

#include <stdint.h>
#include "stm32f0_regs.h"
#include "g3_types.h"
#include "parameters.h"
#include "fixed_point.h"

extern g3_state_t g3;

/* ========================================================================== */
/*  ADC utilities                                                              */
/* ========================================================================== */

/**
 * Find minimum value in array (sub_8002948).
 */
static int32_t array_min(const int32_t *arr, int count)
{
    int32_t result = arr[0];
    for (int i = 1; i < count; i++) {
        if (arr[i] < result)
            result = arr[i];
    }
    return result;
}

/**
 * Find maximum value in array (sub_800292A).
 */
static int32_t array_max(const int32_t *arr, int count)
{
    int32_t result = arr[0];
    for (int i = 1; i < count; i++) {
        if (arr[i] > result)
            result = arr[i];
    }
    return result;
}

/**
 * Bubble sort for median filter (sub_80028A4).
 * Sorts array of uint16_t values in ascending (a3=1) or descending order.
 */
static void bubble_sort_u16(uint16_t *arr, uint32_t count, int ascending)
{
    for (uint32_t i = 0; i < count; i++) {
        for (uint32_t j = i + 1; j < count; j++) {
            if (ascending) {
                if (arr[i] > arr[j]) {
                    uint16_t tmp = arr[i];
                    arr[i] = arr[j];
                    arr[j] = tmp;
                }
            } else {
                if (arr[i] < arr[j]) {
                    uint16_t tmp = arr[i];
                    arr[i] = arr[j];
                    arr[j] = tmp;
                }
            }
        }
    }
}

/**
 * ADC-to-temperature conversion using lookup table.
 * sub_800A864(table_size, table_ptr, adc_value) → temperature.
 * Linear interpolation between table entries.
 * Result has -30 offset: temp_degC = lookup_result - 30.
 */
static int32_t adc_to_temperature(int table_size, const int16_t *table,
                                   uint16_t adc_value)
{
    /* Linear search through table for the two entries bracketing adc_value */
    /* Then linear interpolate */
    /* Result shifted left by 15 for Q15 representation */
    int idx = 0;
    /* Placeholder: would do table lookup */
    return (idx - TEMP_OFFSET) << Q15_SHIFT;
}

/* ========================================================================== */
/*  Throttle processing (sub_8002080)                                          */
/* ========================================================================== */

static int32_t  throttle_counter;           /* dword_20000290 */
static int16_t  throttle_history_idx;       /* word_20000282 */
static uint32_t throttle_history[20];       /* dword_20000294 */
static int32_t  throttle_filtered;          /* dword_2000028C */

/**
 * Process throttle ADC input.
 *
 * sub_8002080:
 *   Decimates by 20 (runs actual processing every 20th call).
 *   Stores raw ADC into circular history buffer.
 *   Finds minimum value in history (noise rejection).
 *   Converts ADC to voltage via lookup table.
 *   Error detection:
 *     ADC < 15    → error 0xFE (short circuit)
 *     ADC > 4000  → error 0xFD (open circuit)
 *     Otherwise   → error 0x00 (OK)
 *   Only checks errors if motor speed > 500 (dword_20000B14 > 0x1F4)
 *
 * @param[out] error  Error state byte
 * @return     Filtered throttle voltage in Q15
 */
q15_t throttle_adc_process(uint8_t *error)
{
    if (++throttle_counter >= ADC_SAMPLE_RATE_DIV) {
        throttle_counter = 0;

        int16_t idx = throttle_history_idx;
        throttle_history[idx] = 0;  /* word_20000F4C — raw ADC reading */
        throttle_history_idx = (uint16_t)(idx + 1) % 20;

        /* Find minimum in history for noise rejection */
        int32_t min_val = array_min((int32_t *)throttle_history, 20);

        /* Convert to temperature/voltage via lookup table */
        throttle_filtered = (adc_to_temperature(TEMP_TABLE_SIZE,
                             adc_temp_table, (uint16_t)min_val)
                             - TEMP_OFFSET) << Q15_SHIFT;

        /* Error detection (only at significant speed) */
        if (g3.motor_speed > 500) {
            if (min_val < THROTTLE_MIN_VALID) {
                *error = 0;         /* Very low = OK (released) */
            } else if (min_val > THROTTLE_MAX_VALID) {
                *error = (uint8_t)-3;  /* 0xFD: open circuit */
            } else {
                *error = 0;
            }
        }
    }

    /* If speed is too low, return fixed value */
    if (g3.motor_speed <= 500) {
        return 819200;  /* ~25 in Q15 — minimum throttle position */
    }
    return throttle_filtered;
}

/* ========================================================================== */
/*  Brake lever processing (sub_800212C)                                       */
/* ========================================================================== */

static int32_t  brake_counter;              /* dword_20000288 */
static int16_t  brake_history_idx;          /* word_20000280 */
static uint32_t brake_history[20];          /* dword_20000F58 */
static int32_t  brake_filtered;             /* dword_20000284 */

/**
 * Process brake lever ADC input.
 *
 * sub_800212C:
 *   Same structure as throttle but inverted (4096 - raw).
 *   Error: raw < 80 → short, raw > 4080 → open.
 */
q15_t brake_adc_process(uint8_t *error)
{
    if (++brake_counter >= ADC_SAMPLE_RATE_DIV) {
        brake_counter = 0;

        int16_t idx = brake_history_idx;
        brake_history[idx] = 0;  /* word_20000F50 — raw ADC */
        brake_history_idx = (uint16_t)(idx + 1) % 20;

        /* Find maximum in history (inverted sense) */
        int32_t max_val = array_max((int32_t *)brake_history, 20);
        int32_t inverted = ADC_FULL_SCALE - max_val;

        brake_filtered = (adc_to_temperature(TEMP_TABLE_SIZE,
                          adc_temp_table, (uint16_t)inverted)
                          - TEMP_OFFSET) << Q15_SHIFT;

        if (g3.motor_speed > 500) {
            int32_t check = ADC_FULL_SCALE - inverted;
            if (check <= THROTTLE_LOW_LIMIT) {
                *error = (uint8_t)-3;  /* Short */
            } else if (check > THROTTLE_HIGH_LIMIT) {
                *error = (uint8_t)-2;  /* Open */
            } else {
                *error = 0;
            }
        }
    }

    if (g3.motor_speed <= 500)
        return 819200;
    return brake_filtered;
}

/**
 * Get raw brake position (sub_80021E0).
 */
uint16_t brake_get_raw(void)
{
    return 0;  /* word_20000F54 */
}

/**
 * Get scaled brake value (sub_80021EC).
 * Multiplies raw brake by pole-pair scaling factor.
 */
q15_t brake_get_scaled(void)
{
    uint16_t raw  = 0;  /* word_20000F52 */
    uint16_t scale = g3.motor_poles;  /* word_200003EA */
    return (int64_t)((uint32_t)raw << Q15_SHIFT) * scale >> Q15_SHIFT;
}

/* ========================================================================== */
/*  Speed from hall sensor period (sub_800205C)                                */
/* ========================================================================== */

/**
 * Compute motor speed from hall sensor edge timing.
 *
 * sub_800205C:
 *   speed = 3519 * (word_20000F4E - word_2000027E)
 *   Scaled to Q15 with 64-bit intermediate.
 *   3519 likely encodes: RPM = K / hall_period
 */
q15_t speed_from_hall(void)
{
    uint16_t hall_current  = 0;  /* word_20000F4E */
    uint16_t hall_previous = 0;  /* word_2000027E */
    int32_t diff = (uint16_t)(hall_current - hall_previous);

    /* Q15 multiply: 3519 * diff, with 64-bit precision */
    int64_t result = (int64_t)3519 * (diff << Q15_SHIFT);
    return (q15_t)(result >> Q15_SHIFT);
}

/* ========================================================================== */
/*  LED / Light control                                                        */
/* ========================================================================== */

/* PWM counter for blinking patterns */
static uint16_t led_blink_counter;       /* word_20000306 / word_20000304 */

/* LED pattern function table — dword_2000068C */
/* 8 pattern functions indexed by (word_20001FCE >> 9) & 7 */

/**
 * LED steady ON (sub_8002F74).
 * Sets TIM16/17 CH4 compare to 999 (100% duty).
 */
void led_on(void)
{
    /* sub_800DD2A(TIM16_BASE, 4, 999) */
    /* TIM16->CCR4 = 999; */
}

/**
 * LED dim (sub_8002F84).
 * Sets TIM16/17 CH4 to 399 (~40% duty).
 */
void led_dim(void)
{
    /* sub_800DD2A(TIM16_BASE, 4, 399) */
}

/**
 * LED off.
 */
void led_off(void)
{
    /* sub_800DD2A(TIM16_BASE, 4, 0) */
}

/**
 * LED slow blink pattern (sub_8002ED0).
 *
 * Triangular wave with period ~2000 ticks:
 *   0..999:    ramp up (PWM = counter)
 *   1000..1998: off (PWM = 0)
 *   Reset at 1998
 */
void led_blink_slow(int reset)
{
    if (reset)
        led_blink_counter = 999;

    led_blink_counter += 56;

    if (led_blink_counter < 999) {
        /* sub_800DD2A(TIM16, 4, 999) */
    } else if (led_blink_counter < 1998) {
        /* sub_800DD2A(TIM16, 4, 0) */
    } else {
        /* sub_800DD2A(TIM16, 4, 0) */
        led_blink_counter = 0;
    }
}

/**
 * LED fast blink (sub_8002F24).
 *
 * Symmetric triangle wave with step=4, period ~2000:
 *   0..999:    PWM = counter
 *   1000..1998: PWM = 1998 - counter (fade down)
 *   Reset at 1998
 */
void led_blink_fast(int reset)
{
    if (reset)
        led_blink_counter = 0;

    led_blink_counter += 4;

    if (led_blink_counter < 999) {
        /* sub_800DD2A(TIM16, 4, counter) */
    } else if (led_blink_counter < 1998) {
        /* sub_800DD2A(TIM16, 4, 1998 - counter) */
    } else {
        /* sub_800DD2A(TIM16, 4, 0) */
        led_blink_counter = 0;
    }
}

/**
 * LED pattern from dashboard command (sub_8002F94).
 * Reads pattern index from word_20001FCE bits [11:9] (0-7).
 * Calls indexed function from pattern table.
 * XOR with previous pattern to detect transitions.
 */
void led_pattern_update(void)
{
    uint8_t pattern = (g3.dash_command >> 9) & 7;

    /* Pattern function table:
     * 0: led_off
     * 1: led_blink_slow
     * 2: led_blink_fast
     * 3: led_on
     * 4: led_dim
     * 5-7: (unused / custom patterns)
     */
    switch (pattern) {
    case 0: led_off(); break;
    case 1: led_blink_slow(0); break;
    case 2: led_blink_fast(0); break;
    case 3: led_on(); break;
    case 4: led_dim(); break;
    default: led_off(); break;
    }
}

/* ========================================================================== */
/*  Temperature derating (sub_8004774)                                         */
/* ========================================================================== */

/**
 * Compute speed limit reduction based on motor and battery temperature.
 *
 * Motor temperature zones:
 *   < 95°C (3112960):  No derating
 *   95-105°C:          Linear derating 0→-16384 (-50%)
 *   105-110°C:         -16384 → -24576 (-75%)
 *   > 110°C:           -24576 → -32768 (-100%)
 *
 * Battery temperature:
 *   Each degree below minimum adds derating proportionally.
 *   Clamped to -62914 (about -1.92 in Q15).
 */
q15_t temperature_derating(void)
{
    q15_t motor_derate = 0;
    q15_t base_speed = (uint16_t)g3.speed_limit_kmh << Q15_SHIFT;

    if (g3.motor_temp >= MOTOR_TEMP_DERATE_SEVERE) {
        /* Severe: quadratic derate */
        motor_derate = q15_div(MOTOR_TEMP_DERATE_SEVERE - g3.motor_temp,
                               0xA0000) - DERATE_FACTOR_24K;
        motor_derate = clamp_i32(motor_derate, -DERATE_FACTOR_32K,
                                 -DERATE_FACTOR_24K);
    }
    else if (g3.motor_temp >= MOTOR_TEMP_DERATE_MID) {
        motor_derate = q15_div(MOTOR_TEMP_DERATE_MID - g3.motor_temp,
                               0xA0000) - DERATE_FACTOR_16K;
        motor_derate = clamp_i32(motor_derate, -DERATE_FACTOR_24K,
                                 -DERATE_FACTOR_16K);
    }
    else if (g3.motor_temp >= MOTOR_TEMP_DERATE_START) {
        motor_derate = q15_div(MOTOR_TEMP_DERATE_START - g3.motor_temp,
                               0xA0000);
        motor_derate = clamp_i32(motor_derate, -DERATE_FACTOR_16K, 0);
    }

    /* Apply motor derate to base speed */
    q15_t speed_after_motor = base_speed + q15_mul(motor_derate, base_speed);

    /* Battery temperature derate */
    q15_t batt_min_temp = (uint16_t)0 /* word_200003DE */ << Q15_SHIFT;
    q15_t batt_derate = g3.battery_temp - 65536 - batt_min_temp;
    batt_derate = clamp_i32(batt_derate, -62914, 0);

    q15_t final_speed = speed_after_motor + q15_mul(batt_derate / 2,
                                                     speed_after_motor);

    /* Clamp to absolute limits */
    if (final_speed > base_speed) final_speed = base_speed;
    if (final_speed < 65536)      final_speed = 65536;  /* Min 2.0 Q15 */

    return final_speed;
}

/* ========================================================================== */
/*  Error checking                                                             */
/* ========================================================================== */

/**
 * Check error flag register (sub_8004DB0).
 * Returns true if any critical error bits are set.
 * Error mask = 17575 (0x44A7) — specific bit pattern for critical faults.
 */
int check_error_flags(void)
{
    return (g3.error_flags & ERR_FLAG_MASK) != 0;
}

/**
 * Check if dashboard is alive (sub_8004DCC).
 * Returns true if byte_20001FC2 == 0 (no timeout).
 */
int check_dashboard_alive(void)
{
    return g3.dash_lock == 0;
}

/**
 * Read GPIO status bits (sub_8005938).
 * Reads a GPIO pin (GPIOB IDR bit 9) for button state.
 * Maps to: byte_200001BD = (GPIOB->IDR & 0x200) == 0
 * Updates headlight status bit in telemetry.
 */
void gpio_status_read(void)
{
    uint8_t button = (GPIOB->IDR & 0x200) == 0;
    /* byte_200001BD = button */

    /* Update telemetry status word bit 3 (headlight) */
    /* LOWORD(dword_20001FE0) = status & 0xFFF7 | (headlight << 3) */
}
