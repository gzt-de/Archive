/**
 * @file parameters.h
 * @brief Firmware tuning constants and configuration parameters.
 *
 * Values extracted from immediate constants in the decompiled firmware.
 * Q15 values shown with approximate real-world equivalents in comments.
 */

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "fixed_point.h"

/* ========================================================================== */
/*  System clock                                                               */
/* ========================================================================== */

#define SYSCLK_FREQ_HZ         48000000U
#define HSI_FREQ_HZ            8000000U
#define HSE_FREQ_HZ            8000000U     /* If external crystal present */
#define HSI48_FREQ_HZ          48000000U

/* ========================================================================== */
/*  Motor PWM                                                                  */
/* ========================================================================== */

#define PWM_FREQUENCY_HZ       14400       /* sub_8002250: 14400000 / 1000 */
#define PWM_DEADTIME_NS        500         /* Approximate */
#define TIM1_ARR_VALUE         999         /* sub_800DC62 arg */
#define TIM1_PRESCALER         143         /* sub_800DC62 arg: 48MHz/(143+1)=333kHz? */

/* ========================================================================== */
/*  ADC                                                                        */
/* ========================================================================== */

#define ADC_CHANNELS            5
#define ADC_MEDIAN_WINDOW       5
#define ADC_HISTORY_SIZE        39          /* sub_8002480 history depth */
#define ADC_SAMPLE_RATE_DIV     20          /* sub_8002080 / sub_800212C decimation */
#define ADC_VREF_MV             3300
#define ADC_RESOLUTION_BITS     12
#define ADC_FULL_SCALE          4096

/* ADC-to-temperature calibration */
#define TEMP_TABLE_SIZE         236         /* sub_800A864 table size */
#define TEMP_OFFSET             30          /* sub_800A864: "- 30" */

/* Throttle ADC thresholds */
#define THROTTLE_LOW_LIMIT      80          /* sub_800212C: < 80 = short */
#define THROTTLE_HIGH_LIMIT     4080        /* sub_800212C: > 4080 = open */
#define THROTTLE_MIN_VALID      15          /* sub_8002080 */
#define THROTTLE_MAX_VALID      4000        /* sub_8002080 */

/* ========================================================================== */
/*  Motor control parameters                                                   */
/* ========================================================================== */

/* Default PID tuning (when flash config invalid) — sub_80032DC */
#define DEFAULT_KP              (196608)    /* 6.0 in Q15 */
#define DEFAULT_KI              (655360)    /* 20.0 in Q15 */
#define DEFAULT_KD              (78643)     /* 2.4 in Q15 */
#define DEFAULT_ACCEL           (327680)    /* 10.0 in Q15 */
#define DEFAULT_DECEL           (163840)    /* 5.0 in Q15 */
#define DEFAULT_FW_START        (6553)      /* 0.2 in Q15 */
#define DEFAULT_REGEN           (32768)     /* 1.0 in Q15 */
#define DEFAULT_BRAKE           (32768)     /* 1.0 in Q15 */

/* Observer bandwidth / gain */
#define OBSERVER_BW             32705134    /* sub_8002990 arg */
#define OBSERVER_GAIN           39008532    /* sub_80072BC arg */
#define OBSERVER_FILTER         1009476480  /* sub_80072BC arg (float ~38.0) */

/* Speed thresholds */
#define SPEED_ZERO_THRESHOLD    (32768)     /* 1.0 in Q15 — below this = stopped */
#define SPEED_MIN_FOR_FOC       (491520)    /* ~15 in Q15 — min for sensorless */
#define SPEED_PUSH_THRESHOLD    (1638)      /* sub_8003940: push detect */

/* Current limits */
#define CURRENT_LIMIT_MIN       (3276)      /* 0.1 in Q15 */
#define CURRENT_LIMIT_MAX       (65536)     /* 2.0 in Q15 */

/* ========================================================================== */
/*  Speed mode configurations                                                  */
/* ========================================================================== */

/* Mode speed limits — values found in ride control functions */
#define SPEED_LIMIT_ECO_Q15     (98304)     /* ~3.0 in Q15 (Q15 RPM) */
#define SPEED_LIMIT_NORMAL_Q15  (131072)    /* ~4.0 in Q15 */
#define SPEED_LIMIT_SPORT_Q15   (196608)    /* ~6.0 in Q15 */
#define SPEED_LIMIT_PUSH_Q15    (32768)     /* ~1.0 in Q15 */

/* ========================================================================== */
/*  Temperature protection                                                     */
/* ========================================================================== */

/* Motor temperature thresholds (in ADC units, ~converted to degC) */
#define MOTOR_TEMP_DERATE_START     3112960     /* ~95°C — start derating */
#define MOTOR_TEMP_DERATE_MID       3440640     /* ~105°C — 50% derate */
#define MOTOR_TEMP_DERATE_SEVERE    3604480     /* ~110°C — severe derate */

/* Battery temperature thresholds */
#define BATT_TEMP_DERATE_THRESHOLD  62914       /* sub_8004774 */

/* Temperature derating amounts (Q15) */
#define DERATE_FACTOR_16K           16384       /* 50% */
#define DERATE_FACTOR_24K           24576       /* 75% */
#define DERATE_FACTOR_32K           32768       /* 100% */

/* ========================================================================== */
/*  Protocol                                                                   */
/* ========================================================================== */

/* CAN bus */
#define CAN_BAUD_RATE           250000      /* Typical for scooter CAN */
#define CAN_ID_MASK_29BIT       0x1FFFFFFFU

/* UART (Ninebot serial) */
#define UART_BAUD_RATE          115200
#define NINEBOT_FRAME_HEADER    0x5AA5      /* Typical Ninebot framing */
#define NINEBOT_MAX_PAYLOAD     252

/* Protocol addresses */
#define ADDR_DASHBOARD          0x20
#define ADDR_MCU                0x21
#define ADDR_BMS                0x22
#define ADDR_BLE                0x23

/* ========================================================================== */
/*  Watchdog                                                                   */
/* ========================================================================== */

/* sub_800E29C: IWDG reload = 0xCCCC (enable), prescaler via sub_800E28C */
#define IWDG_RELOAD_KEY         0xCCCCU     /* 52428 decimal */
#define IWDG_ENABLE_KEY         0x5555U     /* 21845 decimal */
#define IWDG_PRESCALER_DIV      7           /* /256 prescaler */

/* ========================================================================== */
/*  Miscellaneous                                                              */
/* ========================================================================== */

/* Flash unlock key — sub_800CC3C */
#define FLASH_UNLOCK_KEY        0xCDEF89ABU /* (-839939669 as signed) */

/* Angle conversion: electrical angle magic number */
/* 1367130551 = 2^32 / (2 * pi) ≈ 683565275.5... actually this is
   2^32 * (1/(2*pi)) for mapping radians to full uint32 range */
#define ANGLE_TO_UINT32         1367130551U

/* Odometer distance clamp */
#define ODOMETER_CLAMP_POS      521011
#define ODOMETER_CLAMP_NEG      (-521011)

/* DMA/I2C config base */
#define I2C_SLAVE_ADDR_BASE     0x20000FA8U  /* unk_20000FA8 */

/* Error flag bits — dword_20000B10 */
#define ERR_FLAG_MASK           17575       /* sub_8004DB0 check */

#endif /* PARAMETERS_H */
