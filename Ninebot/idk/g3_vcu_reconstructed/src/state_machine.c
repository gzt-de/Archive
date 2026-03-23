/**
 * @file state_machine.c
 * @brief Ninebot Max G3 VCU - Main State Machine
 *
 * The scooter operates in 8 states:
 *   0 = STANDBY:   Off/idle, waiting for power button or BLE wake
 *   1 = STARTING:  Power-on sequence, initializing peripherals
 *   2 = PAIRING:   BLE authentication/pairing active
 *   3 = READY:     Normal riding operation
 *   4 = LOCKING:   Lock sequence in progress
 *   5 = CRUISE:    IoT/connected mode (BLE-locked riding)
 *   6 = SHUTDOWN:  Shutdown sequence
 *   7 = LOCKED:    Locked state with alarm armed
 *
 * Reconstructed from sub_8004E48 (main task), which is a 100ms periodic task.
 *
 * State handler functions:
 *   sub_800B0A8 = state_standby_handler
 *   sub_800B3A0 = state_starting_handler
 *   sub_800AF64 = state_pairing_handler
 *   sub_800ADE4 = state_ready_handler
 *   sub_800AC34 = state_locking_handler
 *   sub_800B4B4 = state_cruise_handler
 *   sub_800B148 = state_shutdown_handler
 *   sub_800AD80 = state_locked_handler
 */

#include "vcu_types.h"
#include "hal_drivers.h"

/* ============================================================================
 * External References
 * ============================================================================ */
extern scooter_runtime_t    g_scooter;
extern bms_data_t           g_bms;
extern motor_ctrl_data_t    g_motor;
extern serial_tx_buf_t      g_serial_tx_buf;

extern volatile uint8_t     g_btn_up;
extern volatile uint8_t     g_btn_down;
extern volatile uint8_t     g_btn_power;
extern volatile uint8_t     g_btn_fn;
extern volatile uint8_t     g_btn_long;

/* Forward declarations for sub-modules */
extern void esc_periodic_update(void);          /* sub_8006D88 */
extern void ble_periodic_update(void);          /* sub_800A100 */
extern int  scheduler_check_tasks(void);        /* sub_8009B8C */
extern int  timer_service_update(void);         /* sub_800960C */
extern int  auth_process(int, int, int, int);   /* sub_800192C */
extern int  fault_check_update(void);           /* sub_8003BAC */
extern int  iot_lock_update(void);              /* sub_800A184 */
extern int  headlight_state_machine(void);      /* sub_8002DA4 */
extern void firmware_update_check(void);        /* sub_8005E28 */
extern int  state_event_notify(int event, ...); /* sub_800A044 */

extern int  led_set_pattern(int pattern);
extern void led_clear_all(void);                /* sub_8008AC0 */
extern void led_mode_reset(void);               /* sub_800713C */

/* State handler functions */
extern void state_standby_handler(void);    /* sub_800B0A8 */
extern void state_starting_handler(void);   /* sub_800B3A0 */
extern void state_pairing_handler(void);    /* sub_800AF64 */
extern void state_ready_handler(void);      /* sub_800ADE4 */
extern void state_locking_handler(void);    /* sub_800AC34 */
extern void state_cruise_handler(void);     /* sub_800B4B4 */
extern void state_shutdown_handler(void);   /* sub_800B148 */
extern void state_locked_handler(void);     /* sub_800AD80 */

/* ============================================================================
 * State Transition Event IDs (parameter to sub_800A044)
 * ============================================================================ */
#define EVENT_SHUTDOWN_COMPLETE     0
#define EVENT_AUTH_COMPLETE         1
#define EVENT_POWER_ON_START        2
#define EVENT_COMMS_TIMEOUT         11
#define EVENT_REMOTE_LOCK           15
#define EVENT_BLE_DISCONNECT        16
#define EVENT_CRUISE_START          19
#define EVENT_CHARGE_TIMEOUT        21

/* ============================================================================
 * Main Task (sub_8004E48)
 *
 * This is the primary periodic task, called every 100ms by the scheduler.
 * It runs the state machine and all periodic updates.
 * ============================================================================ */
void main_task(int a1, int a2, int a3, int a4)
{
    uint16_t tmp_buf[22];
    tmp_buf[0] = 0;

    /* -----------------------------------------------------------------------
     * State transition detection - run init code when state changes
     * ----------------------------------------------------------------------- */
    if (g_scooter.prev_state != g_scooter.state) {
        led_clear_all();
        led_mode_reset();

        g_scooter.lock_flag = 0;
        g_scooter.sleep_timer = 0;
        g_scooter.hw_ready = 0;
        g_scooter.error_status &= ~4;
        g_scooter.error_status &= ~8;

        /* Reset various flags on state entry */
        g_scooter.cruise_mode = 0;        /* byte_20003995 = 1 (enable monitoring) */
        g_scooter.walk_assist_mode = 0;   /* byte_20003935 */
        g_scooter.headlight_mode &= 0;    /* word_20000EDA = 200 */
        g_scooter.cruise_timer = 0;
        g_scooter.walk_assist_active = 0; /* byte_200039B4 */
        g_scooter.iot_lock_cmd = 0;       /* byte_20003982 */
        g_scooter.auth_retry = 0;
        g_scooter.maint_flag = 0;

        g_scooter.auto_off_timer = 0;     /* word_20000BAC */
        g_scooter.shutdown_delay = 600;

        if (g_scooter.state != SCOOTER_STATE_STARTING) {
            /* byte_20003994 = 0 except when entering STARTING */
        }

        g_scooter.prev_state = g_scooter.state;
    }

    /* -----------------------------------------------------------------------
     * Periodic sub-system updates (run in every state)
     * ----------------------------------------------------------------------- */
    esc_periodic_update();
    ble_periodic_update();
    scheduler_check_tasks();
    timer_service_update();
    auth_process(0, 0, 0, 0);
    fault_check_update();
    iot_lock_update();
    headlight_state_machine();

    /* Check if external battery connected state changed */
    if ((g_bms.flags1 & 8) != g_scooter.ext_batt_connected) {
        g_scooter.ext_batt_connected = g_bms.flags1 & 8;
        if ((g_bms.flags1 & 8) == 8) {
            /* External battery just connected */
            /* byte_20000B9A = 1 */
        }
    }

    /* IoT lock activation timeout check */
    if (g_scooter.iot_lock_state == 4) {    /* byte_200039A4 */
        /* Count up to 100 cycles (10 seconds) then trigger pattern 19 */
    }

    /* Auth retry check */
    if (g_scooter.auth_locked) {            /* byte_200039A5 */
        /* 3 strikes -> reset auth, notify BLE */
    }

    /* Firmware update mode intercepts everything */
    if (g_scooter.fw_update_active) {       /* byte_20003990 - pseudo */
        firmware_update_check();
        return;
    }

    /* -----------------------------------------------------------------------
     * State machine dispatch
     * ----------------------------------------------------------------------- */
    switch (g_scooter.state) {

    /* === STATE 0: STANDBY ================================================ */
    case SCOOTER_STATE_STANDBY:
        state_standby_handler();

        /* Check for power-on triggers */
        if ((g_scooter.power_on_flag || g_scooter.alarm_state == 2 ||
             g_btn_down || g_scooter.ext_batt_connected) &&
            g_scooter.auto_off_timer < 300)
        {
            if (!g_scooter.auto_off_timer)
                state_event_notify(EVENT_POWER_ON_START);

            g_scooter.auto_off_timer++;

            /* Safety check: if voltages are dangerously low, go to shutdown */
            if (g_scooter.auto_off_timer >= 300 &&
                (g_scooter.status_word3 & 0x100) &&
                g_btn_down)
            {
                g_scooter.state = SCOOTER_STATE_SHUTDOWN;
                g_scooter.power_on_flag = 0;
                g_scooter.shutdown_flag = 1;
            }
            return;
        }

        /* More power-on condition checks */
        if (g_scooter.power_on_flag || g_scooter.alarm_state == 2 ||
            g_btn_down || g_scooter.ext_batt_connected)
        {
            if (g_scooter.power_on_flag) {
                g_scooter.power_on_flag = 0;
            }

            /* Determine initial ride mode based on config */
            if (g_scooter.status_word1 & 4) {
                if (g_scooter.auth_locked == 1) {   /* byte_200039B6 */
                    /* Paired device - skip auth */
                    g_btn_down = 0;
                }
                g_scooter.ride_mode = RIDE_MODE_PAIRING;
            } else {
                g_scooter.ride_mode = RIDE_MODE_READY;
            }

            /* If IoT connected and charger present -> cruise mode */
            if ((g_bms.flags1 & 8) && (g_scooter.status_word0 & 0x800)) {
                g_scooter.state = SCOOTER_STATE_CRUISE;
                g_scooter.cruise_timer = 100;
            }

            g_scooter.alarm_state = 0;
        }
        else if (g_scooter.alarm_state == 4 || !g_scooter.charger_connected)
        {
            /* Auto-off timer */
            if (g_scooter.auto_off_timer) {
                g_scooter.auto_off_timer--;
            } else if (!g_scooter.charger_connected) {
                g_scooter.alarm_state = 0;
                g_scooter.state = SCOOTER_STATE_SHUTDOWN;
                g_scooter.shutdown_flag = 1;
            }
        }

        if (g_scooter.ride_mode) {
            g_scooter.state = SCOOTER_STATE_STARTING;
        }
        break;

    /* === STATE 1: STARTING =============================================== */
    case SCOOTER_STATE_STARTING:
        state_starting_handler();

        if (g_scooter.hw_ready) {
            if (g_scooter.ride_mode == 3 || g_scooter.ride_mode == 1) {
                /* Go directly to READY */
                g_scooter.state = SCOOTER_STATE_READY;
            } else if (g_scooter.ride_mode == 2) {
                /* Need pairing */
                g_scooter.state = SCOOTER_STATE_PAIRING;
            }
        }
        break;

    /* === STATE 2: PAIRING ================================================ */
    case SCOOTER_STATE_PAIRING:
        state_pairing_handler();

        if (g_scooter.hw_ready) {
            if (g_scooter.auth_locked == 1) {   /* byte_200039BB */
                /* Auth succeeded -> READY */
                g_scooter.state = SCOOTER_STATE_READY;
            } else if (g_scooter.auth_locked == 2) {
                /* Auth failed -> LOCKING */
                g_scooter.state = SCOOTER_STATE_LOCKING;
            } else if (g_scooter.lock_flag) {
                /* Lock requested */
                g_scooter.state = SCOOTER_STATE_SHUTDOWN;
            }
        }
        break;

    /* === STATE 3: READY (Normal Riding) ================================== */
    case SCOOTER_STATE_READY:
        state_ready_handler();

        if (g_scooter.hw_ready) {
            if (g_bms.flags1 & 8) {
                /* External battery connected -> might need shutdown */
                g_scooter.state = SCOOTER_STATE_SHUTDOWN;
            } else if (g_scooter.lock_flag) {
                g_scooter.state = SCOOTER_STATE_SHUTDOWN;
            }
        }
        break;

    /* === STATE 4: LOCKING ================================================ */
    case SCOOTER_STATE_LOCKING:
        state_locking_handler();

        if (g_scooter.hw_ready) {
            if (g_btn_down == 1 || g_scooter.power_on_flag == 1) {
                /* Button press during lock -> back to standby */
                g_btn_down = 0;
                g_scooter.state = SCOOTER_STATE_STANDBY;
                g_scooter.power_on_flag = 1;
                g_scooter.auth_locked = 1;
            } else if (g_scooter.walk_assist_active == 1 ||
                       g_scooter.lock_flag ||
                       g_scooter.auth_locked == 1)  /* byte_200039BA */
            {
                g_scooter.walk_assist_active = 0;
                g_scooter.state = SCOOTER_STATE_SHUTDOWN;
                if (g_scooter.auth_locked == 1)
                    g_scooter.shutdown_flag = 1;
            }
        }
        break;

    /* === STATE 5: CRUISE (IoT Connected) ================================= */
    case SCOOTER_STATE_CRUISE:
        state_cruise_handler();

        if (g_scooter.hw_ready &&
            !(g_bms.flags1 & 8) &&
            !g_scooter.walk_assist_mode)
        {
            g_scooter.state = SCOOTER_STATE_SHUTDOWN;
        }
        break;

    /* === STATE 6: SHUTDOWN =============================================== */
    case SCOOTER_STATE_SHUTDOWN:
        state_shutdown_handler();

        if (g_scooter.hw_ready && g_scooter.shutdown_flag) {
            if ((g_bms.flags1 & 8) && (g_scooter.status_word0 & 0x800)) {
                /* IoT mode still active - go to cruise instead */
                gpio_clear_bits(GPIOA_BASE, 1024);
                /* sub_8002988 - disable motor */
                pwr_set_keep_alive(1);
                gpio_set_bits(GPIOA_BASE, 2048);
                g_scooter.state = SCOOTER_STATE_CRUISE;
            } else {
                g_scooter.alarm_state = 0;
                g_scooter.state = SCOOTER_STATE_LOCKED;
            }
        }
        break;

    /* === STATE 7: LOCKED (Alarm Armed) =================================== */
    case SCOOTER_STATE_LOCKED:
        state_locked_handler();

        if (g_scooter.hw_ready &&
            (g_scooter.alarm_state || g_scooter.charger_connected))
        {
            if (g_scooter.alarm_state == 1) {
                g_scooter.alarm_state = 0;
                g_scooter.state = SCOOTER_STATE_LOCKING;
                g_scooter.walk_assist_active = 0;
            } else {
                g_scooter.state = SCOOTER_STATE_STANDBY;
            }
        }
        break;

    default:
        break;
    }
}

/* ============================================================================
 * Kick Detection / Motion Sensor (sub_8004B58)
 *
 * Monitors accelerometer/hall sensor to detect if scooter is being kicked.
 * Used for alarm triggering in LOCKED state and walk-assist activation.
 * ============================================================================ */
static uint8_t kick_debounce = 0;  /* byte_20000E53 */

int kick_detection_update(void)
{
    int result = 0;

    /* In SHUTDOWN or LOCKED states, motion = 0 */
    if (g_scooter.state == SCOOTER_STATE_LOCKED ||
        g_scooter.state == SCOOTER_STATE_SHUTDOWN)
    {
        g_scooter.walk_assist_active = 0;   /* byte_20003988 */
        return 0;
    }

    /* In READY or PAIRING, detect motion */
    if (g_scooter.state == SCOOTER_STATE_READY ||
        g_scooter.state == SCOOTER_STATE_PAIRING)
    {
        if (kick_debounce > 20) {
            if (g_scooter.walk_assist_active != 2)
                return g_scooter.walk_assist_active;
            result = 1;
        } else {
            result = 2;
        }
        g_scooter.walk_assist_active = result;
    }

    return result;
}
