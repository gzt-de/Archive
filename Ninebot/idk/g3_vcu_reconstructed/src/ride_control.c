/**
 * @file ride_control.c
 * @brief Ride Mode, Speed Control, Headlight, LED Management
 *
 * Reconstructed from:
 * sub_8002DA4 = headlight_state_machine
 * sub_8002E84 = led_status_update
 * sub_8002C9C = charger_detection
 * sub_8002D5C = sleep_timer_update
 * sub_8003BAC = speed_control_update (fault handling portion)
 * sub_80072D0 = ride_mode_validate
 * sub_8008B80 = led_set_pattern
 * sub_8008AC0 = led_clear_all
 * sub_800713C = led_mode_reset
 * sub_8007364 = data_logging_update
 * sub_8007494 = ble_lock_state_update
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
extern data_log_t           g_data_log;

extern int serial_enqueue_tx(serial_tx_buf_t *buf, uint8_t src, uint8_t dst,
                              uint8_t len, uint8_t cmd, uint8_t subcmd,
                              const void *payload);
extern int can_send_message(uint8_t dst, uint8_t src, uint8_t len,
                             uint8_t cmd, uint8_t subcmd, void *data);
extern int state_event_notify(int event, ...);
extern void fault_activate(int code);
extern void fault_deactivate(int code);

/* ============================================================================
 * LED Pattern Controller
 *
 * The LED patterns are indexed by a pattern ID.
 * The VCU sends LED commands to the dashboard display via serial/CAN.
 * ============================================================================ */

static uint8_t led_current_pattern;     /* byte_20003930 or similar */

/* sub_8008B80 */
int led_set_pattern(int pattern)
{
    led_current_pattern = (uint8_t)pattern;
    /* The actual LED pattern data would be sent to the dashboard controller */
    /* via serial_enqueue_tx or can_send_message */
    return 0;
}

/* sub_8008AC0 */
void led_clear_all(void)
{
    led_current_pattern = 0;
    /* Clear all LED states on dashboard */
}

/* sub_800713C */
void led_mode_reset(void)
{
    /* Reset LED mode to default for current state */
}

/* ============================================================================
 * Headlight State Machine (sub_8002DA4)
 *
 * Controls headlight on/off and auto-light based on ambient sensor.
 * 
 * headlight_mode (byte_20000ED8) encodes multiple state bits:
 *   Bits [1:0] = Main state (1=manual, 2=auto_on, 3=auto_ready)
 *   Bits [3:2] = Sub-state (direction of transition)
 *   Bits [5:4] = Light mode (0=off, 1=dim, 2=bright)
 *   Bits [7:6] = Reserved
 *
 * Auto-light uses a hysteresis timer (word_20000EDA = 200 cycles = 20s)
 * ============================================================================ */
int headlight_state_machine(void)
{
    uint8_t mode;

    /* If in firmware update mode, skip */
    if (g_scooter.fw_update_active)     /* byte_20003910 - pseudo */
        return 0;

    mode = g_scooter.headlight_mode & 0xF3;

    if (g_scooter.charger_connected) {  /* byte_20003916 */
        /* Charger connected - transition to charging light mode */
        if (g_scooter.headlight_timer < 400) {
            g_scooter.headlight_timer++;
        } else {
            g_scooter.charger_connected = 0;
            g_scooter.status_word0 |= 0x200;
            g_scooter.headlight_timer = 200;
            g_scooter.headlight_mode = (mode + 4) & 0xFC | 2;
        }
    } else {
        if (g_scooter.headlight_timer) {
            g_scooter.headlight_timer--;
        } else {
            g_scooter.headlight_mode = mode + 8;
            g_scooter.status_word0 &= ~0x200;
            g_scooter.headlight_timer = 200;
            if ((g_scooter.headlight_mode & 3) == 1)
                g_scooter.headlight_mode = (mode + 4) & 0xFC | 2;
        }
    }

    /* Determine light output */
    if ((g_scooter.headlight_mode & 3) == 2) {
        /* Auto-light ready */
        g_scooter.headlight_mode |= 3;
        uint8_t sub = (g_scooter.headlight_mode >> 2) & 3;
        if (sub != 1)
            g_scooter.ride_mode = 2;    /* byte_20000ECF */
        else
            g_scooter.ride_mode = sub;
    }

    /* Final light mode resolution */
    if ((g_scooter.headlight_mode & 3) == 3) {
        if (g_scooter.ride_mode == 1) {
            /* Check ambient sensor / time-based schedule */
            if (((g_scooter.headlight_mode >> 2) & 3) == 2) {
                g_scooter.headlight_mode |= 0x30;
                g_scooter.speed_mode_timer = 0;
                g_scooter.ride_mode = 0;
            } else {
                uint8_t lm = g_scooter.headlight_mode & 0xCF;
                if (g_scooter.speed_mode_timer >= 5)
                    g_scooter.headlight_mode = lm + 16;    /* Dim */
                else
                    g_scooter.headlight_mode = lm + 32;    /* Bright */
            }
        } else if (g_scooter.ride_mode == 2) {
            g_scooter.speed_mode_timer = 0;
            g_scooter.headlight_mode &= 0xCF;
        }

        /* Write final light mode to display register */
        g_scooter.speed_mode = (g_scooter.headlight_mode >> 4) & 3;
    }

    return 0;
}

/* ============================================================================
 * LED Status Update (sub_8002E84)
 *
 * Updates the LED status word based on charger, BLE, and ride state.
 * LED status (word_2000395E) bit fields:
 *   Bits [12:9]: LED mode indicator
 *     0x000 = off
 *     0x200 = BLE connected indicator
 *     0x400 = charging indicator  
 *     0x600 = ride active indicator
 *     0x800 = cruise/IoT indicator
 * ============================================================================ */
void led_status_update(void)
{
    uint16_t led_val;

    /* Handle BLE disconnect detection */
    if (g_scooter.ext_batt_connected == 1 && !g_scooter.charge_state) {
        /* byte_20000B8B transition detection */
        g_scooter.led_status &= 0xF1FF;
    }

    /* External battery indicator via CAN/BLE */
    if (g_bms.flags1 & 8) {
        /* IoT flags available */
        if (!(g_scooter.status_word3 & 2))
            goto clear_led;

        if (g_scooter.cruise_mode) {
            if (g_scooter.current_speed) {
                led_val = (g_scooter.led_status & 0xF1FF) + 0x600;
            } else {
                goto clear_led;
            }
        } else if (g_scooter.charge_state != 1) {
            goto clear_led;
        } else {
            led_val = (g_scooter.led_status & 0xF1FF) + 0x200;
        }
    } else {
        /* No external battery / standalone mode */
        if (g_scooter.tail_light_on)    /* byte_20000A79 */
            return;

        if (g_scooter.charge_state == 1) {
            if (!g_scooter.charger_connected)
                goto check_riding;
        } else if (!g_scooter.charger_connected) {
            /* Check GPIO for charger/trigger detection */
            if ((GPIOB_IDR & 0x2000) == 0 || g_scooter.iot_lock_state != 1) {
                if (!(GPIOE_ODR & 0x40))
                    goto clear_led;
            }
            /* Check for charger via pin */
check_riding:
            led_val = (g_scooter.led_status & 0xF1FF) + 0x400;
            goto set_led;
        }

        /* Riding with various BLE/IoT states */
        if (g_scooter.iot_lock_state == 1 ||
            g_scooter.iot_lock_state == 2 ||
            g_scooter.iot_lock_state == 5 ||
            !g_scooter.speed_display)
        {
            led_val = (g_scooter.led_status & 0xF1FF) + 0x600;
        } else {
            led_val = (g_scooter.led_status & 0xF1FF) + 0x800;
        }
    }

set_led:
    g_scooter.led_status = led_val;
    return;

clear_led:
    g_scooter.led_status &= 0xF1FF;
}

/* ============================================================================
 * Charger Detection (sub_8002C9C)
 *
 * Monitors charger connection state and handles auto-lock on charge.
 * ============================================================================ */
void charger_detection(int charge_pin_state)
{
    static uint8_t charge_debounce = 0;     /* byte_20000BBD */
    static uint16_t charge_timer = 0;       /* word_20000BEA */

    if (g_scooter.status_word0 & 0x800) {
        /* IoT connected mode */
        charge_timer = 0;
        if (!g_scooter.charge_state)     /* byte_20003933 */
            g_scooter.charge_state = 1;
        charge_debounce = 0;
    } else {
        if (charge_pin_state == 1) {
            charge_debounce++;
            if (charge_debounce >= 50) {
                charge_debounce = 0;
                /* Send charge notification */
                serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 14, 3, 0,
                                   &g_scooter.status_word0);
                state_event_notify(11, 0, 0, 0);   /* COMMS event */
            }
        } else {
            charge_debounce = 0;
        }

        if (!g_scooter.charge_state && g_scooter.charger_connected == 1) {
            charge_timer++;
            if (charge_timer >= 300) {
                charge_timer = 0;
                g_scooter.iot_lock_cmd = 2;
                g_scooter.charge_state = 1;
                state_event_notify(21, 0, 0, 0);
            }
        } else {
            charge_timer = 0;
        }

        if (charge_pin_state == 3) {
            fault_activate(15);
            return;
        }
    }
    fault_deactivate(15);
}

/* ============================================================================
 * Sleep Timer Update (sub_8002D5C)
 *
 * Manages auto-shutdown countdown. When timer expires, sets state to SHUTDOWN.
 * ============================================================================ */
void sleep_timer_update(int timeout_value)
{
    if (g_scooter.sleep_timer) {
        g_scooter.sleep_timer--;
        if (g_scooter.sleep_timer == 0) {
            g_scooter.state = SCOOTER_STATE_SHUTDOWN;
        }
    } else {
        g_scooter.sleep_timer = timeout_value;
    }
}

/* ============================================================================
 * Ride Mode Validator (sub_80072D0)
 *
 * Ensures the ride mode is valid, defaults to NORMAL if not.
 * ============================================================================ */
void ride_mode_validate(uint8_t *mode)
{
    if (*mode != RIDE_MODE_ECO &&
        *mode != RIDE_MODE_NORMAL &&
        *mode != RIDE_MODE_SPORT &&
        *mode != RIDE_MODE_CUSTOM)
    {
        *mode = RIDE_MODE_NORMAL;
    }
}

/* ============================================================================
 * Data Logging Update (sub_8007364)
 *
 * Records periodic ride data (speed, voltage, current, temps, etc.)
 * to a RAM ring buffer with a magic header for integrity.
 *
 * Log format: 14 x int16 fields per entry, up to 200 entries.
 * Magic: 0x515C ("QZ")
 * ============================================================================ */
void data_logging_update(void)
{
    static uint8_t log_interval = 0;    /* byte_20000F09 */

    /* Validate magic header */
    if (g_data_log.magic != 0x515C) {
        g_data_log.write_index = 0;
        g_data_log.magic = 0x515C;

        /* Clear all entries */
        for (uint16_t i = 1; i < LOG_MAX_ENTRIES; i++)
            g_data_log.entries[i].fields[13] = 0;
    }

    log_interval++;
    if (log_interval <= 10)
        return;

    log_interval = 0;

    /* Snapshot current telemetry into the next log entry */
    uint16_t idx = g_data_log.write_index + 1;
    log_entry_t *entry = &g_data_log.entries[idx];

    entry->fields[0]  = (uint8_t)g_scooter.state;          /* Ride state */
    entry->fields[1]  = (uint8_t)g_scooter.ride_mode;      /* Ride mode */
    entry->fields[2]  = g_bms.temperature;                  /* BMS temp */
    entry->fields[3]  = g_bms.cell_voltage;                 /* Cell voltage */
    entry->fields[4]  = g_motor.phase_current;              /* Phase current */
    entry->fields[5]  = g_motor.speed_rpm;                  /* Motor RPM */
    entry->fields[6]  = g_bms.voltage;                      /* Pack voltage */
    entry->fields[7]  = g_bms.current;                      /* Pack current */
    entry->fields[8]  = g_data_log.entries[0].fields[0];    /* Speed avg */
    entry->fields[9]  = g_data_log.entries[0].fields[1];    /* Trip distance */
    entry->fields[10] = g_motor.battery_soc;                /* SOC */
    entry->fields[11] = g_scooter.current_speed;            /* Speed */
    entry->fields[12] = g_scooter.total_odo;                /* Odometer */
    entry->fields[13] = 0;                                  /* Reserved */
}

/* ============================================================================
 * BLE Lock State Update (sub_8007494)
 *
 * Processes BLE lock/unlock commands and manages lock state transitions.
 * ============================================================================ */
void ble_lock_state_update(void)
{
    if (g_btn_up != 1)
        return;

    g_data_log.write_index++;

    if (!(g_scooter.status_word0 & 0x800)) {
        /* No IoT connection - can't process lock commands */
        return;
    }

    if (g_scooter.esc_status0 & 4) {       /* word_20003948 bit 2 */
        /* ESC reports error - trigger diagnostic mode */
        g_scooter.auth_step = 1;            /* byte_20000DE9 */
        g_btn_up = 0;
        return;
    }

    /* Process lock state machine */
    switch (g_scooter.iot_lock_cmd) {
        case 1:
            g_scooter.iot_lock_cmd = 2;
            led_set_pattern(18);
            break;

        case 2:
            if (g_scooter.status_word2 & 0x80) {
                g_scooter.iot_lock_cmd = 5;
                led_set_pattern(18);
            }
            break;

        case 3:
            if ((g_scooter.status_word1 & 0x10) && g_scooter.comms_retry_cnt < 5) {
                g_scooter.iot_lock_cmd = 1;
                led_set_pattern(18);
            } else {
                g_scooter.iot_lock_cmd = 2;
            }
            break;

        case 5:
            if (g_scooter.status_word2 & 0x100) {
                g_scooter.iot_lock_cmd = 3;
                led_set_pattern(18);
            }
            goto check_lock_19;
            break;

        default:
            g_scooter.iot_lock_cmd = 2;
            break;
    }

check_lock_19:
    g_btn_up = 0;
}

/* ============================================================================
 * Walk-Assist Speed Limit Configuration
 *
 * Walk-assist mode and speed schedule handling (sub_8003BAC partial)
 * ============================================================================ */
void walk_assist_update(void)
{
    uint32_t speed_data[4] = {0};

    /* Check if speed schedule is active */
    if (g_scooter.status_word3 & 0x10) {
        /* Time-based speed limit check */
        if (g_scooter.walk_assist_mode == 18 && !g_scooter.walk_assist_active)
            g_scooter.walk_assist_active = 1;
    }

    /* Process speed schedule changes */
    if (g_scooter.status_word3 & 0x80) {
        /* Build speed limit message */
        uint32_t combined = speed_data[0] & 0xFFC00000;
        combined |= (g_scooter.speed_sched_start & 0x3F);
        combined |= ((g_scooter.speed_sched_start >> 8) & 0x1F) << 6;
        combined |= (g_scooter.speed_sched_end & 0x3F) << 11;
        combined |= ((g_scooter.speed_sched_end >> 8) & 0x1F) << 17;

        speed_data[0] = combined | 0x1000000;

        can_send_message(62, 7, 4, 3, 135, speed_data);
        g_scooter.status_word3 &= ~0x80;
        g_scooter.status_word3 &= ~0x10;
        g_scooter.status_word3 &= ~0x20;
        g_scooter.walk_assist_mode = 1;
    }
}
