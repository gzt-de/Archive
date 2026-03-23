/**
 * @file sensors.c
 * @brief Sensor Processing - Throttle, Brake, Voltage, Temperature
 *
 * Reconstructed from:
 * sub_8002BD0 = sensor_adc_update (reads all ADC channels and checks thresholds)
 * sub_8004780 = fault_monitor_update (checks for error conditions, timeouts)
 * sub_8005484 = sensor_reset_all
 * sub_80054B0 = speed_limit_reset
 * sub_80054E8 = brake_control_reset
 * sub_8004A44 = rtc_date_check (maintenance interval checking)
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

extern int serial_enqueue_tx(serial_tx_buf_t *buf, uint8_t src, uint8_t dst,
                              uint8_t len, uint8_t cmd, uint8_t subcmd,
                              const void *payload);
extern void fault_set(int code);        /* sub_80095F8 */
extern void fault_clear(int code);      /* sub_80053D8 */
extern void fault_activate(int code);   /* sub_8009584 */
extern void fault_deactivate(int code); /* sub_80052F4 */

/* Threshold: 0x190 = 400 in decimal (ADC counts, approx 0.32V at 3.3V/4096) */
#define ADC_SENSOR_MIN_THRESHOLD    0x190

/* ============================================================================
 * Sensor ADC Update (sub_8002BD0)
 *
 * Reads throttle, brake, voltage, and temperature ADC channels.
 * Performs sanity checks on sensor values and sets error flags.
 *
 * Error flags in g_scooter.sensor_error_flags (byte_20000A70):
 *   Bit 0: Throttle/brake sensor error (main)
 *   Bit 1: Voltage/temperature sensor error (secondary)
 * ============================================================================ */
void sensor_adc_update(int mode)
{
    g_scooter.throttle_raw = adc_read_throttle();       /* word_20000E36 */
    g_scooter.throttle_filtered = adc_read_brake();     /* word_20000E3A */
    g_scooter.brake_raw = adc_read_voltage();           /* word_20000E34 */
    g_scooter.brake_filtered = adc_read_temperature();  /* word_20000E38 */

    if (mode == 0)
        return;

    if (mode == 1) {
        /* Check throttle + brake pair for sensor health */
        if (g_scooter.throttle_raw >= ADC_SENSOR_MIN_THRESHOLD &&
            g_scooter.throttle_filtered >= ADC_SENSOR_MIN_THRESHOLD)
        {
            /* Both sensors in valid range - clear error */
            g_scooter.sensor_error_flags &= ~1;
            g_scooter.throttle_err_cnt = 0;
        } else {
            /* Sensor reading too low - debounce error */
            g_scooter.throttle_err_cnt++;
            if (g_scooter.throttle_err_cnt >= 16) {
                g_scooter.throttle_err_cnt = 0;
                g_scooter.sensor_error_flags |= 1;
            }
        }
    } else if (mode == 2 || mode == 3) {
        /* Mode 2/3: check all four channels */
        if (mode == 3) {
            /* Throttle pair */
            if (g_scooter.throttle_raw >= ADC_SENSOR_MIN_THRESHOLD &&
                g_scooter.throttle_filtered >= ADC_SENSOR_MIN_THRESHOLD)
            {
                g_scooter.sensor_error_flags &= ~1;
                g_scooter.throttle_err_cnt = 0;
            } else {
                g_scooter.throttle_err_cnt++;
                if (g_scooter.throttle_err_cnt >= 16) {
                    g_scooter.throttle_err_cnt = 0;
                    g_scooter.sensor_error_flags |= 1;
                }
            }
        }

        /* Voltage + temperature pair */
        if (g_scooter.brake_raw >= ADC_SENSOR_MIN_THRESHOLD &&
            g_scooter.brake_filtered >= ADC_SENSOR_MIN_THRESHOLD)
        {
            g_scooter.sensor_error_flags &= ~2;
            g_scooter.brake_err_cnt = 0;
        } else {
            g_scooter.brake_err_cnt++;
            if (g_scooter.brake_err_cnt >= 16) {
                g_scooter.brake_err_cnt = 0;
                g_scooter.sensor_error_flags |= 2;
            }
        }
    }
}

/* ============================================================================
 * Reset All Sensors (sub_8005484)
 * ============================================================================ */
void sensor_reset_all(void)
{
    g_scooter.sensor_error_flags = 0;
    g_scooter.throttle_err_cnt = 0;
    g_scooter.brake_err_cnt = 0;
    g_scooter.headlight_on = 0;
    g_scooter.tail_light_on = 0;
    /* word_20000A72 = 0 */
    /* byte_2000398E = 0 */
    /* sub_8009954() - reset speed controller */
    /* sub_80036DC() - reset brake controller */
}

/* ============================================================================
 * Fault Monitor Update (sub_8004780)
 *
 * Comprehensive fault detection, called every 100ms.
 * Monitors communication timeouts, sensor readings, temperature, etc.
 * ============================================================================ */
void fault_monitor_update(void)
{
    /* Increment communication watchdog counters */
    if (g_scooter.esc_comms_timeout < 500)
        g_scooter.esc_comms_timeout++;

    if (g_scooter.bms_comms_timeout < 1500)
        g_scooter.bms_comms_timeout++;

    if (g_scooter.ble_comms_timeout < 7500)
        g_scooter.ble_comms_timeout++;

    /* --- ESC Communication Check --- */
    /* If no ESC data received for 3 seconds (300 * 10ms) */
    if (g_scooter.esc_comms_timeout >= 300)
        fault_set(FAULT_COMMS_TIMEOUT);         /* Fault 10 */
    else
        fault_clear(FAULT_COMMS_TIMEOUT);

    /* --- BMS Communication Check --- */
    if (g_scooter.bms_comms_timeout >= 500)
        fault_activate(FAULT_OVER_VOLTAGE);     /* Fault 18 */
    else
        fault_deactivate(FAULT_OVER_VOLTAGE);

    /* --- BLE Module Communication --- */
    if (g_scooter.ble_comms_timeout >= 300) {
        fault_activate(FAULT_BMS_COMMS);        /* Fault 25 */

        g_scooter.ble_watchdog++;
        if (g_scooter.ble_watchdog >= 1000) {
            /* BLE module appears dead - reset it */
            /* sub_800B830() */
            /* sub_80097DC(0) - power cycle BLE */
            /* sub_800C814(0x3E8) */
            /* sub_80097DC(1) */
            g_scooter.ble_watchdog = 0;
            g_scooter.ble_comms_timeout = 0;
        }
    } else {
        g_scooter.ble_watchdog = 0;
        fault_deactivate(FAULT_BMS_COMMS);

        /* Extended BLE timeout for power save */
        if (g_scooter.ble_comms_timeout >= 6000) {
            g_scooter.ble_comms_timeout = 0;
            fault_activate(FAULT_BMS_COMMS);
        }
    }

    /* --- Motor Controller Errors --- */
    if (g_motor.error_code == 11)
        fault_set(FAULT_BRAKE_SENSOR);
    else
        fault_clear(FAULT_BRAKE_SENSOR);

    if (g_motor.error_code == 13)
        fault_set(FAULT_MOTOR_PHASE);
    else
        fault_clear(FAULT_MOTOR_PHASE);

    if (g_motor.error_code == 18)
        fault_set(FAULT_OVER_VOLTAGE);
    else
        fault_clear(FAULT_OVER_VOLTAGE);

    if (g_motor.error_code == 24)
        fault_set(FAULT_OVER_CURRENT);
    else
        fault_clear(FAULT_OVER_CURRENT);

    /* --- Brake Lever Engaged While Riding --- */
    if (g_scooter.state == SCOOTER_STATE_READY && g_scooter.charge_state == 1)
        fault_set(FAULT_BRAKE_LEVER);
    else
        fault_clear(FAULT_BRAKE_LEVER);

    /* --- Motor Hall Sensor Error --- */
    if (g_motor.error_code == 55)
        fault_set(FAULT_MOTOR_HALL);
    else
        fault_clear(FAULT_MOTOR_HALL);

    /* --- Battery Temperature --- */
    if (g_motor.temperature == 253 || g_motor.temperature == 254) {
        /* Temperature sensor error / out of range */
        static uint8_t temp_err_cnt = 0;
        temp_err_cnt++;
        if (temp_err_cnt >= 100) {
            temp_err_cnt = 0;
            fault_activate(FAULT_UNDER_VOLTAGE);    /* Fault 20 */
        }
    } else {
        fault_deactivate(FAULT_UNDER_VOLTAGE);

        if (g_motor.temperature > 180) {
            static uint8_t ovt_cnt = 0;
            ovt_cnt++;
            if (ovt_cnt >= 100) {
                ovt_cnt = 0;
                fault_activate(FAULT_OVER_TEMP);    /* Fault 21 */
            }
        } else {
            fault_deactivate(FAULT_OVER_TEMP);
        }
    }

    /* --- External BMS Communication --- */
    if ((g_bms.flags1 & 8) && (g_bms.flags0 & 0x1000)) {
        static uint8_t ext_bms_cnt = 0;
        ext_bms_cnt++;
        if (ext_bms_cnt >= 100) {
            ext_bms_cnt = 0;
            fault_set(FAULT_EXT_BMS_COMMS);
        }
    } else {
        fault_clear(FAULT_EXT_BMS_COMMS);
    }

    /* --- Controller Temperature --- */
    if (g_motor.ext_batt_soc > 135 &&
        g_motor.ext_batt_soc != 253 &&
        g_motor.ext_batt_soc != 254)
    {
        static uint8_t ctrl_temp_cnt = 0;
        ctrl_temp_cnt++;
        if (ctrl_temp_cnt >= 100) {
            ctrl_temp_cnt = 0;
            fault_activate(FAULT_CONTROLLER_TEMP);
        }
    } else {
        fault_deactivate(FAULT_CONTROLLER_TEMP);
    }

    /* --- Update headlight/taillight status --- */
    g_scooter.headlight_on = g_scooter.tail_light_on;
    g_scooter.tail_light_on = g_scooter.headlight_on;
}

/* ============================================================================
 * RTC / Date Check (sub_8004A44)
 *
 * Checks if 90 days have passed since first use for maintenance reminder.
 * Date is encoded in bits of dword_20003708:
 *   Bits [15:10] = Year offset
 *   Bits [9:6]   = Month
 *   Bits [5:1]   = Day
 * ============================================================================ */
void rtc_date_check(void)
{
    uint16_t first_date = g_scooter.first_use_date;
    uint16_t current_date = (uint16_t)(g_scooter.trip_time >> 16);

    /* Initialize first-use date if not set */
    if (!first_date && current_date) {
        g_scooter.first_use_date = current_date;
    }

    /* Extract date fields */
    uint16_t first_year = first_date >> 10;
    uint16_t curr_year  = current_date >> 10;
    uint16_t first_month = (first_date >> 6) & 0xF;
    uint16_t curr_month  = (current_date >> 6) & 0xF;
    uint16_t first_day  = (first_date >> 1) & 0x1F;
    uint16_t curr_day   = (current_date >> 1) & 0x1F;

    /* Calculate approximate days elapsed */
    int days_elapsed = 365 * (curr_year - first_year) +
                       30 * curr_month + curr_day -
                       (30 * first_month + first_day);

    /* Check if IoT features enabled and maintenance not yet flagged */
    if ((g_scooter.status_word0 & 0x800) && (g_scooter.status_word1 & 8)) {
        if (!g_scooter.maint_flag && days_elapsed >= 90) {
            g_scooter.maint_flag = 1;
            g_scooter.maint_notify = 1;
            g_scooter.status_word0 |= 8;

            /* Notify via serial */
            serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 2, 2, 209,
                               &g_scooter.maint_notify);
        }
    }
}

/* ============================================================================
 * Speed Limit Reset (sub_80054B0)
 *
 * Resets speed limiting factors to 1.0 (no limiting).
 * ============================================================================ */
void speed_limit_reset(void)
{
    /* 1065353216 = 0x3F800000 = 1.0f in IEEE 754 */
    g_scooter.speed_limit_f = 0x3F800000;  /* dword_20005D00 */
    g_scooter.brake_limit_f = 0x3F800000;  /* dword_20005D04 */
    /* Reset various speed limit control variables */
}

/* ============================================================================
 * Speed Limit Validator (sub_8002D84)
 *
 * Validates configured speed limit is within allowed range.
 * ============================================================================ */
void speed_limit_validate(void)
{
    if (g_scooter.speed_limit < 1 || g_scooter.speed_limit > 30)
        g_scooter.speed_limit = 5;
}

/* ============================================================================
 * Weak stubs for fault management functions
 * ============================================================================ */
void __attribute__((weak)) fault_set(int code)      { (void)code; }
void __attribute__((weak)) fault_clear(int code)    { (void)code; }
void __attribute__((weak)) fault_activate(int code) { (void)code; }
void __attribute__((weak)) fault_deactivate(int code) { (void)code; }
