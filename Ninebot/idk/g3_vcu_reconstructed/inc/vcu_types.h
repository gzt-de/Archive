/**
 * @file vcu_types.h
 * @brief Ninebot Max G3 VCU - Type definitions and global declarations
 *
 * Reconstructed from IDA Pro decompilation output.
 * Target MCU: STM32F103 (Cortex-M3, 72MHz)
 * Flash base: 0x08000000, SRAM base: 0x20000000
 */

#ifndef VCU_TYPES_H
#define VCU_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * Compiler / Platform Helpers
 * ============================================================================ */
#ifndef __packed
#define __packed __attribute__((packed))
#endif

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* ============================================================================
 * Hardware Register Base Addresses (STM32F103)
 * ============================================================================ */
#define RCC_BASE            0x40021000
#define RCC_CR              (*(volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_CFGR            (*(volatile uint32_t *)(RCC_BASE + 0x04))
#define RCC_CIR             (*(volatile uint32_t *)(RCC_BASE + 0x08))
#define RCC_BDCR            (*(volatile uint32_t *)(RCC_BASE + 0x1C))
#define RCC_CSR             (*(volatile uint32_t *)(RCC_BASE + 0x30))

#define GPIOA_BASE          0x40010800
#define GPIOB_BASE          0x40010C00
#define GPIOC_BASE          0x40011000
#define GPIOD_BASE          0x40011400
#define GPIOE_BASE          0x40011800  /* Some pins referenced at 0x40011C0C */

#define GPIO_CRL_OFFSET     0x00
#define GPIO_CRH_OFFSET     0x04
#define GPIO_IDR_OFFSET     0x08
#define GPIO_ODR_OFFSET     0x0C
#define GPIO_BSRR_OFFSET    0x10
#define GPIO_BRR_OFFSET     0x14

#define GPIOA_ODR           (*(volatile uint32_t *)(GPIOA_BASE + GPIO_ODR_OFFSET))
#define GPIOB_IDR           (*(volatile uint32_t *)(GPIOB_BASE + GPIO_IDR_OFFSET))
#define GPIOB_ODR           (*(volatile uint32_t *)(GPIOB_BASE + GPIO_ODR_OFFSET))
#define GPIOC_IDR           (*(volatile uint32_t *)(GPIOC_BASE + GPIO_IDR_OFFSET))
#define GPIOC_ODR           (*(volatile uint32_t *)(GPIOC_BASE + GPIO_ODR_OFFSET))
#define GPIOE_ODR           (*(volatile uint32_t *)(GPIOE_BASE + GPIO_ODR_OFFSET))

#define FLASH_BASE_REG      0x40022000
#define IWDG_BASE           0x40003000
#define IWDG_KR             (*(volatile uint32_t *)(IWDG_BASE + 0x00))
#define IWDG_PR             (*(volatile uint32_t *)(IWDG_BASE + 0x04))

#define PWR_BASE            0x40007000
#define PWR_CR              (*(volatile uint32_t *)(PWR_BASE + 0x00))
#define PWR_CSR             (*(volatile uint32_t *)(PWR_BASE + 0x04))
#define BKP_BASE            0x40006C00

#define I2C1_BASE           0x40005400
#define I2C2_BASE           0x40005800

#define USART1_BASE         0x40013800
#define USART2_BASE         0x40004400
#define USART3_BASE         0x40004800

#define TIM1_BASE           0x40012C00
#define TIM2_BASE           0x40000000
#define TIM3_BASE           0x40000400
#define TIM4_BASE           0x40000800

#define ADC1_BASE           0x40012400
#define ADC2_BASE           0x40012800

#define CAN1_BASE           0x40006400

#define SCB_VTOR            (*(volatile uint32_t *)0xE000ED08)

#define UID_BASE            0x1FFFF7E8  /* Unique device ID */

/* ============================================================================
 * Ninebot Serial Protocol Constants
 * ============================================================================ */
#define SERIAL_HEADER_0     0x5A    /* 'Z' */
#define SERIAL_HEADER_1     0xA5
#define SERIAL_MAX_PAYLOAD  0xF2

/* ============================================================================
 * Scooter State Machine
 * ============================================================================ */
typedef enum {
    SCOOTER_STATE_STANDBY       = 0,   /* Off / standby, waiting for power-on */
    SCOOTER_STATE_STARTING      = 1,   /* Powering on / initializing */
    SCOOTER_STATE_PAIRING       = 2,   /* BLE pairing / authentication */
    SCOOTER_STATE_READY         = 3,   /* Ready to ride / normal operation */
    SCOOTER_STATE_LOCKING       = 4,   /* Lock procedure active */
    SCOOTER_STATE_CRUISE        = 5,   /* Cruise control / IoT mode */
    SCOOTER_STATE_SHUTDOWN      = 6,   /* Shutdown sequence */
    SCOOTER_STATE_LOCKED        = 7,   /* Locked (alarm armed) */
} scooter_state_t;

/* ============================================================================
 * Ride Mode
 * ============================================================================ */
typedef enum {
    RIDE_MODE_ECO       = 1,
    RIDE_MODE_NORMAL    = 2,
    RIDE_MODE_SPORT     = 3,
    RIDE_MODE_CUSTOM    = 5,
} ride_mode_t;

/* ============================================================================
 * Error / Fault Codes
 * ============================================================================ */
typedef enum {
    FAULT_NONE              = 0,
    FAULT_THROTTLE          = 10,
    FAULT_BRAKE_SENSOR      = 11,
    FAULT_MOTOR_PHASE       = 13,
    FAULT_COMMS_TIMEOUT     = 14,
    FAULT_COMMS_LOST        = 15,
    FAULT_OVER_VOLTAGE      = 18,
    FAULT_UNDER_VOLTAGE     = 20,
    FAULT_OVER_TEMP         = 21,
    FAULT_CONTROLLER_TEMP   = 22,
    FAULT_OVER_CURRENT      = 24,
    FAULT_BMS_COMMS         = 25,
    FAULT_EXT_BMS_COMMS     = 30,
    FAULT_BRAKE_LEVER       = 35,
    FAULT_MOTOR_HALL        = 55,
} fault_code_t;

/* ============================================================================
 * CAN Bus Message Types (Ninebot proprietary protocol)
 * The protocol uses 0xEC (236) for firmware upload packets and 0xEB (235) 
 * for firmware data blocks
 * ============================================================================ */
#define CAN_MSG_TYPE_FW_CTRL    0xEC
#define CAN_MSG_TYPE_FW_DATA    0xEB

/* ============================================================================
 * Serial TX Ring Buffer
 * ============================================================================ */
#define SERIAL_TX_BUF_SLOTS     8
#define SERIAL_TX_BUF_SLOT_SIZE 256

typedef struct __packed {
    uint8_t  read_idx;          /* byte_20000008[0] - read pointer */
    uint8_t  write_idx;         /* byte_20000008[1] - write pointer */
    uint8_t  free_slots;        /* byte_20000008[2] - available slots */
    uint8_t  busy;              /* byte_20000008[3] - lock flag */
    struct {
        uint32_t  length;
        uint8_t   data[SERIAL_TX_BUF_SLOT_SIZE - 4];
    } slots[SERIAL_TX_BUF_SLOTS];
} serial_tx_buf_t;

/* ============================================================================
 * Button State (direction buttons for menu/walk-assist)
 * ============================================================================ */
typedef struct __packed {
    uint8_t  up_pressed;        /* byte_20000D89 */
    uint8_t  up_state;
    uint8_t  down_pressed;      /* byte_20000D79 */
    uint8_t  down_state;
    uint8_t  power_pressed;     /* byte_20000D99 */
    uint8_t  power_state;
    uint8_t  fn_pressed;        /* byte_20000DA9 */
    uint8_t  fn_state;
    uint8_t  long_press;        /* byte_20000DB9 */
} button_state_t;

/* ============================================================================
 * Firmware Update Session
 * ============================================================================ */
typedef struct __packed {
    uint8_t   src_addr;
    uint8_t   dst_addr;
    uint8_t   state;            /* 1=idle, 3=sending, 5=receiving, 9=verify, 10=active */
    uint8_t   pad;
    uint32_t  buffer_ptr;
    uint16_t  max_size;
    uint16_t  write_offset;
    uint16_t  total_size;
    uint16_t  chunk_offset;
    uint16_t  total_chunks;
    uint16_t  chunk_size;
    uint16_t  retry_timeout;
    uint16_t  retry_counter;
    uint16_t  max_retries;
    uint16_t  checksum;
} fw_update_session_t;

/* ============================================================================
 * ADC Channel Assignments (Phase currents, voltages, temp)
 * ============================================================================ */
typedef enum {
    ADC_CH_THROTTLE     = 0,
    ADC_CH_BRAKE        = 1,
    ADC_CH_BATTERY_V    = 2,
    ADC_CH_TEMP_BOARD   = 3,
} adc_channel_t;

/* ============================================================================
 * Task / Scheduler
 * ============================================================================ */
#define MAX_TASKS   8

typedef void (*task_func_t)(int a1, int a2, int a3, int a4);

typedef struct {
    task_func_t  callback;
    uint16_t     period_ms;
    uint16_t     counter;
    uint8_t      priority;
    uint8_t      enabled;
} task_entry_t;

/* ============================================================================
 * BMS (Battery Management System) Data
 * ============================================================================ */
typedef struct __packed {
    uint16_t voltage;           /* word_20003624 - pack voltage in 10mV */
    int16_t  current;           /* word_20003626 - current in 10mA (signed) */
    uint16_t remaining_cap;     /* word_20003628 */
    uint16_t speed;             /* word_2000362A - speed in 0.01 km/h or similar */
    uint16_t total_distance;    /* word_2000362C */
    uint16_t temperature;       /* word_2000362E */
    uint16_t flags0;            /* word_20003630 */
    uint16_t flags1;            /* word_20003632 */
    uint16_t soc;               /* word_20003634 - state of charge */
    uint16_t cell_voltage;      /* word_20003636 */
} bms_data_t;

/* ============================================================================
 * Motor Controller Data
 * ============================================================================ */
typedef struct __packed {
    uint16_t speed_rpm;         /* word_200036E4 */
    uint16_t phase_current;     /* word_200036E6 */
    uint32_t hall_state;        /* dword_20003708 */
    uint16_t temperature;       /* word_2000394A */
    uint16_t error_code;        /* word_2000394C */
    uint16_t battery_soc;       /* word_2000394E */
    uint16_t ext_batt_soc;      /* word_20003950 */
} motor_ctrl_data_t;

/* ============================================================================
 * Scooter Configuration (stored in flash)
 * ============================================================================ */
typedef struct __packed {
    uint16_t max_speed_eco;
    uint16_t max_speed_normal;
    uint16_t max_speed_sport;
    uint16_t cruise_control_speed;
    uint16_t wheel_diameter;
    uint16_t speed_limit;       /* word_2000319E - configurable speed limit */
    uint16_t flags;             /* word_20003144, word_20003146, word_20003148, word_2000314A */
    uint16_t region_flags;      /* speed limit enforcement */
    uint8_t  padding[64];
} scooter_config_t;

/* ============================================================================
 * Runtime Scooter Data (volatile state in SRAM)
 * ============================================================================ */
typedef struct {
    /* State machine */
    uint8_t   state;                /* byte_20000820 - scooter_state_t */
    uint8_t   prev_state;           /* byte_20000B99 */
    uint8_t   sub_state;            /* byte_20000821 */

    /* Ride mode */
    uint8_t   ride_mode;            /* byte_20000ECF */
    uint8_t   headlight_mode;       /* byte_20000ED8 */

    /* Speed / throttle */
    uint16_t  current_speed;        /* word_2000362A */
    uint16_t  throttle_raw;         /* word_20000E36 */
    uint16_t  brake_raw;            /* word_20000E34 */
    uint16_t  throttle_filtered;    /* word_20000E3A */
    uint16_t  brake_filtered;       /* word_20000E38 */

    /* Diagnostics / sensors */
    uint8_t   sensor_error_flags;   /* byte_20000A70 */
    uint8_t   throttle_err_cnt;     /* byte_20000A76 */
    uint8_t   brake_err_cnt;        /* byte_20000A77 */
    uint8_t   headlight_on;         /* byte_20000A78 */
    uint8_t   tail_light_on;        /* byte_20000A79 */

    /* Comms watchdog counters */
    uint16_t  esc_comms_timeout;    /* word_20000A7A */
    uint16_t  bms_comms_timeout;    /* word_20000A7E */
    uint16_t  ble_comms_timeout;    /* word_20000A7C */

    /* Power-on / shutdown helpers */
    uint8_t   power_on_flag;        /* byte_20003997 */
    uint8_t   shutdown_flag;        /* byte_20003986 */
    uint8_t   hw_ready;             /* byte_20003984 */
    uint8_t   lock_flag;            /* byte_20003987 */
    uint8_t   alarm_state;          /* byte_20003989 */

    /* Authentication / BLE */
    uint8_t   auth_state;           /* byte_200039B7 */
    uint8_t   auth_step;            /* byte_20000EED */
    uint8_t   auth_retry;           /* byte_200039BD */
    uint8_t   auth_locked;          /* byte_200039B9 */
    uint32_t  auth_challenge;       /* dword_2000397C */
    uint32_t  auth_expected;        /* dword_200031EE */
    uint32_t  auth_response_buf;    /* dword_200039BE */
    uint16_t  auth_key_index;       /* word_200039C8 */
    uint8_t   auth_key_len;         /* byte_200039BC */

    /* Speed mode switch / region */
    uint8_t   speed_mode;           /* byte_20003316 */
    uint16_t  speed_mode_timer;     /* word_2000331A */

    /* Charging */
    uint8_t   charger_connected;    /* byte_20003917 */
    uint8_t   ext_batt_connected;   /* byte_20003932 */
    uint8_t   charge_state;         /* byte_20003931 */
    uint8_t   cruise_mode;          /* byte_20003982 */

    /* IoT / BLE unlock */
    uint8_t   iot_lock_state;       /* byte_2000396D */
    uint8_t   iot_lock_cmd;         /* byte_2000396E */

    /* Walk-assist */
    uint8_t   walk_assist_active;   /* byte_20003980 */
    uint8_t   walk_assist_mode;     /* byte_20003981 */

    /* Misc flags */
    uint16_t  status_word0;         /* word_20003144 */
    uint16_t  status_word1;         /* word_20003146 */
    uint16_t  status_word2;         /* word_20003148 */
    uint16_t  status_word3;         /* word_2000314A */
    uint16_t  led_status;           /* word_2000395E */
    uint16_t  error_status;         /* word_20003958 */

    /* Timers */
    uint16_t  auto_off_timer;       /* word_200039AA */
    uint32_t  sleep_timer;          /* dword_200039AC */
    uint32_t  cruise_timer;         /* dword_200039B0 */
    uint16_t  shutdown_delay;       /* word_200039CC */

    /* Trip / Odometer */
    uint32_t  trip_time;            /* dword_20003708 (upper half = date) */
    uint16_t  first_use_date;       /* word_200032DE */

    /* Unique ID cache */
    uint32_t  uid[3];               /* dword_200032C0..C8 */
} scooter_runtime_t;

/* ============================================================================
 * Data Logging Ring Buffer
 * ============================================================================ */
#define LOG_ENTRY_SIZE      28      /* 14 x int16 per entry */
#define LOG_MAX_ENTRIES     200

typedef struct __packed {
    int16_t  fields[14];            /* Various logged parameters */
} log_entry_t;

typedef struct {
    uint16_t    magic;              /* 0x515C = "QZ" */
    uint16_t    write_index;        /* word_20006432 */
    log_entry_t entries[LOG_MAX_ENTRIES];
} data_log_t;

/* ============================================================================
 * Flash Memory Layout (Ninebot dual-bank config storage)
 * ============================================================================ */
#define FLASH_CONFIG_BANK_A     0x0800E7FE  /* 134345214 decimal */
#define FLASH_CONFIG_BANK_B     0x0800EBFE  /* 134346238 decimal */
#define FLASH_CONFIG_DATA_A     0x0800E800  /* 134345728 decimal */
#define FLASH_CONFIG_DATA_B     0x0800E400  /* 134344704 decimal */
#define FLASH_CONFIG_SIZE       0x200       /* 512 bytes per bank */

/* Flash sector table - addresses stored in dword_20006080[] */
#define FLASH_SECTOR_COUNT      80          /* Approximate, from firmware code */

/* ============================================================================
 * Global Variable Externs 
 * (These map to SRAM addresses 0x20000000+ discovered in decompilation)
 * ============================================================================ */

/* Serial TX buffer at byte_20000008 */
extern serial_tx_buf_t      g_serial_tx_buf;

/* Button states */
extern volatile uint8_t     g_btn_up;           /* byte_20000D89 */
extern volatile uint8_t     g_btn_down;         /* byte_20000D79 */
extern volatile uint8_t     g_btn_power;        /* byte_20000D99 */
extern volatile uint8_t     g_btn_fn;           /* byte_20000DA9 */
extern volatile uint8_t     g_btn_long;         /* byte_20000DB9 */

/* Main scooter runtime */
extern scooter_runtime_t    g_scooter;

/* BMS data received from ESC/BMS controllers */
extern bms_data_t           g_bms;

/* Motor controller data */
extern motor_ctrl_data_t    g_motor;

/* Configuration (loaded from flash) */
extern int16_t              g_config_buf[256];  /* word_2000310C */
extern int16_t              g_config_word;      /* word_2000330A */

/* Data logging */
extern data_log_t           g_data_log;         /* word_2000641A..end */

/* Flash sector address table */
extern uint32_t             g_flash_sectors[FLASH_SECTOR_COUNT]; /* dword_20006080 */

/* Task scheduler flag */
extern volatile uint8_t     g_systick_flag;     /* byte_2000399F */

/* UART stack pointer for bootloader jump */
extern uint8_t              g_uart_stack[];      /* unk_20005FFE */

/* Firmware update session */
extern fw_update_session_t  g_fw_session;

/* Float constants used in speed limiting */
extern uint32_t             g_speed_limit_f;    /* dword_20005D00 - 1065353216 = 1.0f */
extern uint32_t             g_brake_limit_f;    /* dword_20005D04 - 1065353216 = 1.0f */

/* RCC backup (saved before sleep) */
extern uint32_t             g_rcc_cfgr_backup;  /* dword_200039D4 */
extern uint32_t             g_rcc_cr_backup;    /* dword_200039D0 */

/* Authentication */
extern uint32_t             g_auth_calc_buf;    /* dword_20000EFC */
extern uint16_t             g_auth_counter;     /* word_20000EF8 */
extern uint16_t             g_auth_delay;       /* word_20000EF4 */
extern uint16_t             g_auth_cycle;       /* word_20000EF6 */
extern uint8_t              g_auth_seq;         /* byte_20000EEE */

/* Password / key entry buffer */
extern uint16_t             g_pw_display0;      /* word_200033D0 */
extern uint16_t             g_pw_display1;      /* word_200033D4 */

/* Maintenance flag (90-day check) */
extern uint8_t              g_maint_flag;       /* byte_20003993 */
extern uint16_t             g_maint_notify;     /* word_200034AE */

/* Speed limit scheduling */
extern uint16_t             g_speed_sched_start;  /* word_200031A2 */
extern uint16_t             g_speed_sched_end;    /* word_200031A4 */

/* Misc counters */
extern uint16_t             g_ble_watchdog;     /* word_2000399A */
extern uint16_t             g_comms_retry_cnt;  /* word_200039D8 */
extern uint16_t             g_speed_display;    /* word_200031C6 */
extern uint16_t             g_trip_distance;    /* word_200031BA */
extern uint16_t             g_total_odo;        /* word_200031C8 */

/* ESC/BMS specific words */
extern uint16_t             g_esc_status0;      /* word_20003948 */
extern uint16_t             g_led_word;         /* word_2000395E */

#endif /* VCU_TYPES_H */
