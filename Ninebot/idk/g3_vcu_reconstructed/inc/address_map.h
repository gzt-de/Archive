/**
 * @file address_map.h
 * @brief IDA Address -> Reconstructed Symbol Mapping
 *
 * This file documents the mapping between IDA Pro decompiled function
 * addresses and their reconstructed names and locations.
 *
 * Format: IDA_ADDR | Reconstructed Name | File | Notes
 */

#ifndef ADDRESS_MAP_H
#define ADDRESS_MAP_H

/*
 * =============================================================================
 * INTERRUPT VECTORS (0x8001178 - 0x80011A0)
 * =============================================================================
 * sub_8001178  | Reset_Handler           | main.c           | -> main()
 * sub_8001190  | NMI_Handler             | main.c           | while(1)
 * sub_8001192  | HardFault_Handler       | main.c           | while(1)
 * sub_8001194  | MemManage_Handler       | main.c           | while(1)
 * sub_8001196  | BusFault_Handler        | main.c           | while(1)
 * sub_8001198  | UsageFault_Handler      | main.c           | while(1)
 * sub_800119A  | SVC_Handler             | main.c           | while(1)
 * sub_800119C  | DebugMon_Handler        | main.c           | while(1)
 * sub_800119E  | PendSV_Handler          | main.c           | while(1)
 * sub_80011A0  | (reserved)              | main.c           | while(1)
 *
 * =============================================================================
 * COMPILER RUNTIME (0x80011AC - 0x80016BE)
 * =============================================================================
 * sub_80011AC  | rt_uldiv                | runtime_helpers.c | 64-bit division
 * sub_800120E  | (shift left helper)     | runtime_helpers.c | used by uldiv
 * sub_800122C  | (shift right helper)    | runtime_helpers.c | used by uldiv
 * sub_800124C  | rt_memcpy               | runtime_helpers.c | word-aligned copy
 * sub_8001270  | rt_memset               | runtime_helpers.c | fill memory
 * sub_800127E  | rt_memclr               | runtime_helpers.c | zero memory
 * sub_8001282  | rt_memset_ptr           | runtime_helpers.c | memset returning ptr
 * sub_8001294  | rt_memcmp               | runtime_helpers.c | memory compare
 * sub_80012AE  | rt_fadd                 | runtime_helpers.c | float add
 * sub_8001352  | rt_fsub                 | runtime_helpers.c | float subtract
 * sub_8001358  | rt_frsub                | runtime_helpers.c | float reverse sub
 * sub_800135E  | rt_fmul                 | runtime_helpers.c | float multiply
 * sub_80013C2  | rt_fscale               | runtime_helpers.c | float ldexp
 * sub_8001436  | rt_fdiv                 | runtime_helpers.c | float divide
 * sub_8001600  | rt_f2i                  | runtime_helpers.c | float->int
 * sub_8001632  | rt_f2ui                 | runtime_helpers.c | float->uint
 * sub_800165A  | rt_i2f                  | runtime_helpers.c | int->float
 * sub_8001698  | rt_ui2f                 | runtime_helpers.c | uint->float
 * sub_80016BE  | rt_fcmplt               | runtime_helpers.c | float compare <
 *
 * =============================================================================
 * AUTHENTICATION (0x800192C - 0x8001C10)
 * =============================================================================
 * sub_800192C  | auth_process            | authentication.c  | Main auth handler
 * sub_8001BD4  | auth_record_keypress    | authentication.c  | Button press record
 * sub_8001C10  | auth_verify_sequence    | authentication.c  | Key verify
 * sub_8007120  | auth_compute_key_length | authentication.c  | Derive key len
 * sub_8007834  | auth_compute_hash       | authentication.c  | CRC32-like hash
 *
 * =============================================================================
 * HEX / STRING UTILS (0x8001E68 - 0x8001EB4)
 * =============================================================================
 * sub_8001E68  | is_hex_char             | config_manager.c  | Hex char validator
 *
 * =============================================================================
 * SENSOR / ADC (0x80018FC - 0x8002BD0)
 * =============================================================================
 * sub_80018FC  | adc_read_throttle       | hal_drivers.c     | ADC channel 0
 * sub_8001908  | adc_read_brake          | hal_drivers.c     | ADC channel 1
 * sub_8001914  | adc_read_voltage        | hal_drivers.c     | ADC channel 2
 * sub_8001920  | adc_read_temperature    | hal_drivers.c     | ADC channel 3
 * sub_8002BD0  | sensor_adc_update       | sensors.c         | ADC polling + checks
 *
 * =============================================================================
 * RIDE CONTROL (0x8002C9C - 0x8002E84)
 * =============================================================================
 * sub_8002C9C  | charger_detection       | ride_control.c    | Charge pin monitor
 * sub_8002D5C  | sleep_timer_update      | ride_control.c    | Auto-off timer
 * sub_8002D84  | speed_limit_validate    | sensors.c         | Config validation
 * sub_8002DA4  | headlight_state_machine | ride_control.c    | Light controller
 * sub_8002E84  | led_status_update       | ride_control.c    | LED status word
 *
 * =============================================================================
 * FAULT MONITORING (0x8003BAC - 0x8004780)
 * =============================================================================
 * sub_8003BAC  | speed_control_update    | sensors.c (stub)  | Speed/fault checks
 * sub_8004780  | fault_monitor_update    | sensors.c         | Error condition scan
 * sub_8004A44  | rtc_date_check          | sensors.c         | Maintenance timer
 *
 * =============================================================================
 * STATE MACHINE (0x8004E48)
 * =============================================================================
 * sub_8004E48  | main_task               | state_machine.c   | 100ms main task
 * sub_800B0A8  | state_standby_handler   | (stub)            | State 0
 * sub_800B3A0  | state_starting_handler  | (stub)            | State 1
 * sub_800AF64  | state_pairing_handler   | (stub)            | State 2
 * sub_800ADE4  | state_ready_handler     | (stub)            | State 3
 * sub_800AC34  | state_locking_handler   | (stub)            | State 4
 * sub_800B4B4  | state_cruise_handler    | (stub)            | State 5
 * sub_800B148  | state_shutdown_handler  | (stub)            | State 6
 * sub_800AD80  | state_locked_handler    | (stub)            | State 7
 *
 * =============================================================================
 * SERIAL PROTOCOL (0x8005514 - 0x80055E4)
 * =============================================================================
 * sub_8005514  | serial_checksum         | serial_protocol.c | CRC16 checksum
 * sub_800552E  | serial_build_frame      | serial_protocol.c | Build Ninebot frame
 * sub_800557E  | serial_enqueue_tx       | serial_protocol.c | Queue TX message
 * sub_80057D8  | serial_process_tx       | serial_protocol.c | Drain TX buffer
 * sub_80055E4  | serial_dispatch_rx      | serial_protocol.c | Route RX messages
 *
 * =============================================================================
 * FLASH OPERATIONS (0x8005C60 - 0x8005FC0)
 * =============================================================================
 * sub_8005C60  | flash_read_to_ram       | hal_drivers.c     | Flash -> RAM copy
 * sub_8005D18  | flash_write_from_ram    | hal_drivers.c     | RAM -> Flash write
 * sub_8005E04  | config_compute_crc      | config_manager.c  | Config CRC16
 * sub_8005F5C  | flash_pages_for_size    | hal_drivers.c     | Size -> page count
 * sub_8005F70  | flash_erase_range       | hal_drivers.c     | Erase flash pages
 * sub_8005FAE  | flash_read_to_ram       | hal_drivers.c     | (alias)
 * sub_8005FC0  | flash_program_verified  | hal_drivers.c     | Write + verify
 *
 * =============================================================================
 * GPIO / LED (0x8006004 - 0x800605C)
 * =============================================================================
 * sub_8006004  | pwr_set_led_driver      | hal_drivers.c     | LED power control
 * sub_800605C  | pwr_set_keep_alive      | hal_drivers.c     | System power hold
 *
 * =============================================================================
 * CONFIGURATION (0x8006074 - 0x80061D4)
 * =============================================================================
 * sub_8006074  | config_load_from_flash  | config_manager.c  | Dual-bank loader
 * sub_80061D4  | config_firmware_info_read| config_manager.c  | FW metadata
 *
 * =============================================================================
 * I2C (0x8006288 - 0x80062FE)
 * =============================================================================
 * sub_8006288  | i2c_write_byte          | hal_drivers.c     | I2C register write
 * sub_80062C4  | i2c_read_byte           | hal_drivers.c     | I2C register read
 * sub_80062FE  | i2c_write_multi         | hal_drivers.c     | Multi-byte write
 * sub_800634E  | i2c_read_multi          | hal_drivers.c     | Multi-byte read
 *
 * =============================================================================
 * RIDE MODE (0x80072D0 - 0x8007494)
 * =============================================================================
 * sub_80072BC  | (led mode set)          | ride_control.c    | LED mode helper
 * sub_80072D0  | ride_mode_validate      | ride_control.c    | Mode validation
 * sub_80072F0  | (rcc config)            | hal_drivers.c     | Clock + I2C init
 * sub_8007364  | data_logging_update     | ride_control.c    | Ring buffer logger
 * sub_8007494  | ble_lock_state_update   | ride_control.c    | BLE lock FSM
 * sub_8007538  | (GPIO check)            | hal_drivers.c     | Read GPIO pin
 *
 * =============================================================================
 * LED CONTROL (0x8008AC0 - 0x8008B80)
 * =============================================================================
 * sub_8008AC0  | led_clear_all           | ride_control.c    | Clear all LEDs
 * sub_8008B80  | led_set_pattern         | ride_control.c    | Set LED pattern
 *
 * =============================================================================
 * SCHEDULER (0x8009644 - 0x80096F4)
 * =============================================================================
 * sub_8009644  | scheduler_tick          | main.c            | Run ready tasks
 * sub_8009688  | scheduler_register_task | main.c            | Add task
 * sub_80096F4  | i2c_set_speed           | hal_drivers.c     | I2C clock config
 * sub_8009744  | rt_array_compare        | runtime_helpers.c | Byte array compare
 * sub_8009768  | rcc_reset_to_hsi        | hal_drivers.c     | Clock reset
 *
 * =============================================================================
 * CAN BUS (0x800CF30 - 0x800D4D0)
 * =============================================================================
 * sub_800CF30  | systick_init            | main.c (stub)     | SysTick setup
 * sub_800CFF4  | can_send_message        | can_protocol.c    | Send CAN frame
 * sub_800D4D0  | can_fw_update_handler   | can_protocol.c    | FW update via CAN
 * sub_800D784  | can_fw_send_block       | can_protocol.c    | Send FW data block
 * sub_800D908  | can_build_standard_msg  | can_protocol.c    | Build CAN frame
 * sub_800D964  | can_build_nack_msg      | can_protocol.c    | Build NACK frame
 * sub_800D9A4  | can_build_multiframe_msg| can_protocol.c    | Multi-frame msg
 *
 * =============================================================================
 * HAL DRIVERS (0x800CE4C - 0x800D370)
 * =============================================================================
 * sub_800CE4C  | flash_set_latency       | hal_drivers.c     | Flash wait states
 * sub_800CE70  | flash_write_halfword    | hal_drivers.c     | Flash program 16bit
 * sub_800CE94  | flash_lock              | hal_drivers.c     | Lock flash
 * sub_800CEEC  | flash_erase_page        | hal_drivers.c     | Erase 1KB page
 * sub_800CF18  | flash_unlock            | hal_drivers.c     | Unlock flash
 * sub_800D0F0  | gpio_clear_bits         | hal_drivers.c     | GPIO BRR
 * sub_800D0F4  | gpio_set_bits           | hal_drivers.c     | GPIO BSRR
 * sub_800D0F8  | gpio_read_input         | hal_drivers.c     | GPIO IDR pin
 * sub_800D110  | gpio_configure_pin      | hal_drivers.c     | GPIO CRL/CRH
 * sub_800D20C  | gpio_read_bits          | hal_drivers.c     | GPIO IDR masked
 * sub_800D330  | pwr_enter_stop_mode     | hal_drivers.c     | Low-power stop
 * sub_800D370  | bkp_read                | hal_drivers.c     | Backup reg read
 *
 * =============================================================================
 * SYSTEM INIT (0x800D818 - 0x800E208)
 * =============================================================================
 * sub_800D818  | main                    | main.c            | Entry point
 * sub_800DE0C  | adc_init                | hal_drivers.c     | ADC setup
 * sub_800DE50  | adc_trigger_conversion  | hal_drivers.c     | ADC start
 * sub_800DE7C  | (ADC calibrate)         | hal_drivers.c     | ADC calibration
 * sub_800E158  | rcc_init_clocks         | hal_drivers.c     | 72MHz clock init
 * sub_800E1C8  | (peripheral init)       | hal_drivers.c     | Timer/DMA init
 * sub_800E208  | rcc_set_pll_config      | hal_drivers.c     | PLL configuration
 *
 * =============================================================================
 * WATCHDOG (0x800E81C - 0x800E85C)
 * =============================================================================
 * sub_800E81C  | iwdg_reload             | hal_drivers.c     | Feed watchdog
 * sub_800E82C  | iwdg_init (prescaler)   | hal_drivers.c     | WDG setup
 * sub_800E83C  | iwdg_init (reload)      | hal_drivers.c     | WDG reload value
 * sub_800E84C  | iwdg_init (start)       | hal_drivers.c     | WDG enable
 *
 * =============================================================================
 * SRAM VARIABLE MAP (key variables)
 * =============================================================================
 * 0x20000008   | g_serial_tx_buf         | Serial TX ring buffer
 * 0x20000820   | g_scooter.state         | Main state machine
 * 0x20000A70   | g_scooter.sensor_error_flags | Sensor error bits
 * 0x20000A7A   | g_scooter.esc_comms_timeout  | ESC watchdog counter
 * 0x20000D79   | g_btn_down              | Down button state
 * 0x20000D89   | g_btn_up                | Up button state
 * 0x20000D99   | g_btn_power             | Power button state
 * 0x20000DA9   | g_btn_fn                | Function button state
 * 0x20000ED8   | g_scooter.headlight_mode| Headlight state machine
 * 0x20000EFC   | g_auth_calc_buf         | Auth computation buffer
 * 0x200031EE   | g_auth_expected         | Expected auth hash
 * 0x2000310C   | g_config_buf            | Config data buffer (512B)
 * 0x20003144   | g_scooter.status_word0  | System status flags
 * 0x20003624   | g_bms.voltage           | Battery pack voltage
 * 0x2000362A   | g_bms.speed             | Vehicle speed
 * 0x20003708   | g_scooter.trip_time     | Trip time / RTC date
 * 0x2000397C   | g_auth_challenge        | Auth random challenge
 * 0x2000399F   | g_systick_flag          | Main loop sync flag
 * 0x200039BE   | g_auth_response_buf     | Auth key press buffer
 * 0x2000641A   | g_data_log              | Data logging ring buffer
 * 0x20006080   | g_flash_sectors         | Flash sector address table
 */

#endif /* ADDRESS_MAP_H */
