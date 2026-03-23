/**
 * @file main.c
 * @brief Ninebot Max G3 VCU - Entry Point and Task Scheduler
 *
 * System initialization and cooperative task scheduler.
 * The VCU runs 5 periodic tasks at different intervals:
 *
 *   Task 0: main_task          (100ms) - State machine, ride control
 *   Task 1: esc_comms_task     (100ms) - ESC/motor controller protocol
 *   Task 2: can_periodic_task  (100ms) - CAN bus message cycling
 *   Task 3: fast_sensor_task   (10ms)  - ADC sampling, button debounce
 *   Task 4: auth_timer_task    (10ms)  - Authentication timing
 *
 * The main loop:
 *   1. Runs fast update functions (UART TX drain, scheduler tick)
 *   2. Checks for systick flag (set by SysTick ISR)
 *   3. On flag: runs data logging / watchdog
 *
 * Reconstructed from:
 * sub_800D818 = main (entry point, never returns)
 * sub_8009688 = scheduler_register_task
 * sub_8009644 = scheduler_tick
 * sub_8008D8C = esc_protocol_update
 * sub_8008D30 = ble_protocol_update
 * sub_8005920 = uart_check_rx
 */

#include "vcu_types.h"
#include "hal_drivers.h"

/* ============================================================================
 * Global Variable Definitions
 * ============================================================================ */

/* Main scooter runtime state */
scooter_runtime_t       g_scooter;

/* BMS / Motor data */
bms_data_t              g_bms;
motor_ctrl_data_t       g_motor;

/* Configuration buffer */
int16_t                 g_config_buf[256];

/* Data logging */
data_log_t              g_data_log;

/* Flash sector table - pseudo values (actual values embedded in firmware) */
uint32_t g_flash_sectors[FLASH_SECTOR_COUNT] = {
    0x08000000, 0x08001000, 0x08002000, 0x08003000,    /* Bootloader region */
    0x08004000, 0x08005000, 0x08006000, 0x08007000,    /* Application region */
    0x08008000, 0x08009000, 0x0800A000, 0x0800B000,
    0x0800C000, 0x0800D000, 0x0800E000, 0x0800F000,    /* Config/OTA region */
    /* ... remaining sectors filled as needed */
};

/* Serial buffers */
uint8_t g_serial_rx_active;
uint8_t g_serial_tx_count;
uint8_t g_serial_tx_pending;

/* Button states */
volatile uint8_t g_btn_up;
volatile uint8_t g_btn_down;
volatile uint8_t g_btn_power;
volatile uint8_t g_btn_fn;
volatile uint8_t g_btn_long;

/* Systick flag */
volatile uint8_t g_systick_flag;

/* UART stack area */
uint8_t g_uart_stack[16];

/* RCC backup */
uint32_t g_rcc_cfgr_backup;
uint32_t g_rcc_cr_backup;

/* ============================================================================
 * Task Scheduler
 * ============================================================================ */
#define MAX_SCHEDULER_TASKS 8

static task_entry_t scheduler_tasks[MAX_SCHEDULER_TASKS];
static uint8_t scheduler_task_count = 0;

/* sub_8009688 - Register a periodic task */
void scheduler_register_task(uint8_t id, uint16_t period_ms,
                              uint8_t priority, task_func_t callback)
{
    if (id >= MAX_SCHEDULER_TASKS)
        return;

    scheduler_tasks[id].callback = callback;
    scheduler_tasks[id].period_ms = period_ms;
    scheduler_tasks[id].counter = 0;
    scheduler_tasks[id].priority = priority;
    scheduler_tasks[id].enabled = 1;

    if (id >= scheduler_task_count)
        scheduler_task_count = id + 1;
}

/* sub_8009644 - Tick the scheduler, execute any ready tasks */
int scheduler_tick(void)
{
    for (uint8_t i = 0; i < scheduler_task_count; i++) {
        task_entry_t *t = &scheduler_tasks[i];
        if (!t->enabled || !t->callback)
            continue;

        t->counter++;
        if (t->counter >= t->period_ms) {
            t->counter = 0;
            t->callback(0, 0, 0, 0);
        }
    }
    return 0;
}

/* ============================================================================
 * Forward Declarations for Sub-System Init Functions
 * ============================================================================ */
extern int  rcc_init_clocks(void);          /* sub_800E158 */
extern int  config_init_device_id(void);    /* sub_800A770 partial */
extern int  uart_init_serial(void);         /* sub_800649C */
extern int  uart_init_esc(void);            /* sub_80058D4 */
extern int  can_init(void);                 /* sub_8006DB8 */
extern int  adc_init(void);                 /* sub_800DE0C */
extern void systick_init(void);             /* sub_800CF30 */
extern int  config_load_from_flash(void);   /* sub_8006074 partial via sub_8008E58 */

extern int  serial_process_tx(void);        /* sub_80057D8 */
extern void uart_process_rx(uint8_t *buf, uint32_t usart_base, uint32_t flags);
extern void iwdg_reload(void);
extern void data_logging_update(void);

/* Task callbacks */
extern void main_task(int, int, int, int);          /* sub_8004E48 */
extern void esc_comms_task(int, int, int, int);     /* sub_80098C0 */
extern void can_periodic_task(int, int, int, int);  /* sub_800E2F0 */
extern void fast_sensor_task(int, int, int, int);   /* sub_800CFE8 */
extern void auth_timer_task(int, int, int, int);    /* sub_8001888 */

/* ============================================================================
 * SysTick Handler
 *
 * Sets a flag for the main loop. Actual ISR would be in the vector table.
 * ============================================================================ */
void SysTick_Handler(void)
{
    g_systick_flag = 1;
}

/* ============================================================================
 * Main Entry Point (sub_800D818)
 *
 * Never returns. Initializes all peripherals, registers tasks, then enters
 * the main loop.
 * ============================================================================ */
void __attribute__((noreturn)) main(void)
{
    /* Set vector table offset (bootloader offset = 0x1000) */
    SCB_VTOR = 4096;

    /* -----------------------------------------------------------------------
     * Phase 1: Core peripheral initialization
     * ----------------------------------------------------------------------- */
    rcc_init_clocks();              /* sub_800E158 - HSE/PLL to 72MHz */
    config_init_device_id();        /* sub_800A770 - read UID, init I2C */
    uart_init_serial();             /* sub_800649C - BLE/App UART */

    /* Setup serial TX buffer: 
     * sub_8002398 - likely initializes the serial ring buffer */
    g_serial_tx_buf.read_idx = 0;
    g_serial_tx_buf.write_idx = 0;
    g_serial_tx_buf.free_slots = SERIAL_TX_BUF_SLOTS;
    g_serial_tx_buf.busy = 0;

    /* sub_8005C60 - Flash init / stack setup */
    /* Uses g_uart_stack at 0x20005FFE, size 0x400000 */

    uart_init_esc();                /* sub_80058D4 - ESC/motor UART */
    can_init();                     /* sub_8006DB8 - CAN bus */
    adc_init();                     /* sub_800DE0C - ADC channels */
    config_load_from_flash();       /* sub_8008E58 -> sub_8006074 */
    systick_init();                 /* sub_800CF30 - SysTick for scheduler */

    /* -----------------------------------------------------------------------
     * Phase 2: Register periodic tasks
     * ----------------------------------------------------------------------- */
    /*              ID  Period  Prio  Callback */
    scheduler_register_task(0, 10, 0, main_task);           /* 100ms via 10x10ms */
    scheduler_register_task(1, 10, 0, esc_comms_task);      /* 100ms */
    scheduler_register_task(2, 10, 0, can_periodic_task);   /* 100ms */
    scheduler_register_task(3,  1, 0, fast_sensor_task);    /* 10ms */
    scheduler_register_task(4,  1, 0, auth_timer_task);     /* 10ms */

    /* -----------------------------------------------------------------------
     * Phase 3: Initial state setup
     * ----------------------------------------------------------------------- */
    /* byte_20000A82 = 0 - BLE module alive flag */
    g_scooter.ble_comms_timeout = 0;

    /* word_200039AA = 500 - auto-off timer initial value (50 seconds) */
    g_scooter.auto_off_timer = 500;

    /* byte_20000ED8 = (byte_20000ED8 & 0xFC) + 1 - headlight init */
    g_scooter.headlight_mode = (g_scooter.headlight_mode & 0xFC) + 1;

    /* -----------------------------------------------------------------------
     * Phase 4: Main loop (never exits)
     * ----------------------------------------------------------------------- */
    while (1) {
        /* Fast-loop: drain UART TX, tick scheduler, process protocols */
        do {
            /* sub_8005920 - check for UART RX data */
            /* uart_check_rx(); */

            /* sub_8009644 - run scheduler */
            scheduler_tick();

            /* sub_8008D8C - ESC protocol state machine */
            /* esc_protocol_update(); */

            /* sub_8008D30 - BLE protocol state machine */
            /* ble_protocol_update(); */

            /* sub_80057D8 - drain serial TX queue */
            serial_process_tx();

            /* sub_8009A4C - process UART DMA RX */
            uart_process_rx(g_serial_tx_buf.slots[0].data,
                            0x40004800,  /* USART3_BASE - placeholder */
                            0);

        } while (!g_systick_flag);

        /* Slow-loop: runs once per systick (1ms or 10ms depending on config) */
        g_systick_flag = 0;
        iwdg_reload();      /* sub_800E81C - feed watchdog */
    }
}

/* ============================================================================
 * Interrupt Vector Table Handlers
 *
 * These are the default handlers from the top of the decompilation.
 * Most are infinite loops (unhandled), except the reset handler.
 * ============================================================================ */

void __attribute__((noreturn)) Reset_Handler(void)
{
    main();     /* sub_8001178 -> sub_800D818 */
}

void __attribute__((noreturn)) NMI_Handler(void)         { while(1); }
void __attribute__((noreturn)) HardFault_Handler(void)   { while(1); }
void __attribute__((noreturn)) MemManage_Handler(void)   { while(1); }
void __attribute__((noreturn)) BusFault_Handler(void)    { while(1); }
void __attribute__((noreturn)) UsageFault_Handler(void)  { while(1); }
void __attribute__((noreturn)) SVC_Handler(void)         { while(1); }
void __attribute__((noreturn)) DebugMon_Handler(void)    { while(1); }
void __attribute__((noreturn)) PendSV_Handler(void)      { while(1); }

/* ============================================================================
 * Weak stubs for tasks not fully reconstructed
 * ============================================================================ */
void __attribute__((weak)) esc_comms_task(int a, int b, int c, int d)
{ (void)a; (void)b; (void)c; (void)d; }

void __attribute__((weak)) can_periodic_task(int a, int b, int c, int d)
{ (void)a; (void)b; (void)c; (void)d; }

void __attribute__((weak)) fast_sensor_task(int a, int b, int c, int d)
{ (void)a; (void)b; (void)c; (void)d; }

void __attribute__((weak)) auth_timer_task(int a, int b, int c, int d)
{ (void)a; (void)b; (void)c; (void)d; }

int __attribute__((weak)) state_event_notify(int event, ...)
{ (void)event; return 0; }

void __attribute__((weak)) esc_periodic_update(void) {}
void __attribute__((weak)) ble_periodic_update(void) {}
int  __attribute__((weak)) scheduler_check_tasks(void) { return 0; }
int  __attribute__((weak)) timer_service_update(void) { return 0; }
int  __attribute__((weak)) fault_check_update(void) { return 0; }
int  __attribute__((weak)) iot_lock_update(void) { return 0; }
void __attribute__((weak)) firmware_update_check(void) {}

/* State handler stubs - these would each be substantial functions */
void __attribute__((weak)) state_standby_handler(void)  {}  /* sub_800B0A8 */
void __attribute__((weak)) state_starting_handler(void) {}  /* sub_800B3A0 */
void __attribute__((weak)) state_pairing_handler(void)  {}  /* sub_800AF64 */
void __attribute__((weak)) state_ready_handler(void)    {}  /* sub_800ADE4 */
void __attribute__((weak)) state_locking_handler(void)  {}  /* sub_800AC34 */
void __attribute__((weak)) state_cruise_handler(void)   {}  /* sub_800B4B4 */
void __attribute__((weak)) state_shutdown_handler(void) {}  /* sub_800B148 */
void __attribute__((weak)) state_locked_handler(void)   {}  /* sub_800AD80 */
