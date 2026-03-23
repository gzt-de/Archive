/**
 * @file function_map.h
 * @brief Cross-reference: IDA decompiler addresses → reconstructed function names.
 *
 * This file maps every significant function from the IDA pseudo-C output
 * to its reconstructed name and location in the source tree.
 *
 * Format: IDA_addr → file.c :: function_name (brief description)
 */

#ifndef FUNCTION_MAP_H
#define FUNCTION_MAP_H

/*
 * ===========================================================================
 *  STARTUP & RUNTIME
 * ===========================================================================
 *
 * sub_8001178  → main.c          :: Reset_Handler (thunk to main)
 * sub_8001190  → main.c          :: NMI_Handler (infinite loop)
 * sub_8001192  → main.c          :: HardFault_Handler
 * sub_8001194  → main.c          :: SVC_Handler
 * sub_8001196  → main.c          :: PendSV_Handler
 * sub_8001198  → main.c          :: SysTick_Handler
 * sub_800119A  → main.c          :: Default_Handler
 * sub_800119C  → main.c          :: Default_Handler
 * sub_80011A0  → main.c          :: Default_Handler
 * sub_8001A7C  → main.c          :: __libc_init_array (C runtime startup)
 * sub_800D4D8  → main.c          :: main() — MAIN ENTRY POINT
 *
 * ===========================================================================
 *  SOFTWARE FLOATING POINT (ARM EABI)
 * ===========================================================================
 *
 * sub_80011AC  → math_tables.c   :: __aeabi_uldivmod (uint64 division)
 * sub_800120E  → math_tables.c   :: __aeabi_ldivmod (int64 division)
 * sub_8001270  → (builtin)       :: memcpy
 * sub_8001294  → (builtin)       :: memset
 * sub_80012A2  → (builtin)       :: bzero (memset to 0)
 * sub_80012A6  → (builtin)       :: memset wrapper
 * sub_80012B8  → math_tables.c   :: __aeabi_fadd (float add)
 * sub_800135C  → math_tables.c   :: __aeabi_fsub (float sub) [negate + add]
 * sub_8001362  → math_tables.c   :: __aeabi_frsub (float reverse sub)
 * sub_8001368  → math_tables.c   :: __aeabi_fmul (float multiply)
 * sub_80013CC  → math_tables.c   :: __aeabi_fldexp (float ldexp)
 * sub_80013E4  → math_tables.c   :: __aeabi_f2d (float to double)
 * sub_80013F6  → math_tables.c   :: __aeabi_d2f (double to float)
 * sub_8001410  → math_tables.c   :: __aeabi_lmul (fixed-point Q16 mul)
 * sub_8001436  → math_tables.c   :: __aeabi_fdiv (float divide)
 * sub_80014B2  → math_tables.c   :: __aeabi_dadd (double add)
 * sub_80015F4  → math_tables.c   :: __aeabi_dsub (double sub)
 * sub_8001600  → math_tables.c   :: __aeabi_dmul (double multiply)
 * sub_80016E4  → math_tables.c   :: __aeabi_ddiv (double divide)
 * sub_80017C2  → math_tables.c   :: __aeabi_dldexp (double ldexp)
 * sub_80017F0  → math_tables.c   :: __aeabi_i2d (int to double)
 * sub_800180A  → math_tables.c   :: __aeabi_f2iz (float to signed int)
 * sub_800183C  → math_tables.c   :: __aeabi_f2uiz (float to unsigned int)
 * sub_8001864  → math_tables.c   :: __aeabi_d2iz (double to signed int)
 * sub_80018A2  → math_tables.c   :: __aeabi_i2f_shifted (int to float, <<29)
 * sub_80018C8  → math_tables.c   :: __aeabi_fcmplt helper
 * sub_80018DC  → math_tables.c   :: __aeabi_fcmplt helper (duplicate)
 * sub_80018F0  → (builtin)       :: __aeabi_llsl (64-bit left shift)
 * sub_800190E  → (builtin)       :: __aeabi_llsr (64-bit logical right shift)
 * sub_800192E  → (builtin)       :: __aeabi_lasr (64-bit arithmetic right shift)
 * sub_80019C0  → math_tables.c   :: soft_float_round_result
 * sub_80019DE  → math_tables.c   :: soft_float_normalize_pack
 * sub_8001D2C  → math_tables.c   :: soft_float_from_int (__aeabi_i2f)
 *
 * ===========================================================================
 *  FIXED-POINT MATH & TRIGONOMETRY
 * ===========================================================================
 *
 * sub_8001AF8  → math_tables.c   :: fast_sin_q15 (with pi/2 offset)
 * sub_8001B84  → math_tables.c   :: q15_div (Q15 fixed-point division)
 * sub_8001C14  → math_tables.c   :: q15_mul_sat (Q15 saturating multiply)
 * sub_8001C38  → math_tables.c   :: fast_cos_q15 (no offset variant)
 * sub_8001CC0  → math_tables.c   :: fast_sqrt_q15 (Newton-Raphson sqrt)
 * sub_8001D50  → math_tables.c   :: q30_div (Q30 division)
 * sub_8001DE0  → fixed_point.h   :: q30_mul (inline)
 * sub_8001DF0  → fixed_point.h   :: q30_mul_rnd (with rounding)
 * sub_8001AA0  → math_tables.c   :: lz_decompress
 *
 * ===========================================================================
 *  PID CONTROLLER
 * ===========================================================================
 *
 * sub_8001DFC  → (inline)        :: pid3_init (set PID gains, enable)
 * sub_8001E3E  → motor/foc.c     :: pid3_update (3rd-order PID tick)
 *
 * ===========================================================================
 *  ADC & SENSOR PROCESSING
 * ===========================================================================
 *
 * sub_800141C  → main.c          :: hall_init (thunk to sub_8001F94)
 * sub_8001F18  → drivers          :: i2c_dma_init
 * sub_8001F94  → drivers          :: adc_dma_init (TIM1 triggered ADC)
 * sub_800204C  → main.c          :: adc_init (I2C + ADC combo init)
 * sub_800205C  → peripherals.c   :: speed_from_hall
 * sub_8002080  → peripherals.c   :: throttle_adc_process
 * sub_800212C  → peripherals.c   :: brake_adc_process
 * sub_80021E0  → peripherals.c   :: brake_get_raw
 * sub_80021EC  → peripherals.c   :: brake_get_scaled
 * sub_800220C  → peripherals.c   :: throttle_init
 * sub_8002480  → peripherals.c   :: adc_median_filter
 * sub_800252C  → motor/foc.c     :: foc_set_hall_sector
 * sub_80025BC  → motor/foc.c     :: foc_measure_currents
 * sub_80028A4  → peripherals.c   :: bubble_sort_u16
 * sub_800292A  → peripherals.c   :: array_max
 * sub_8002948  → peripherals.c   :: array_min
 * sub_8002966  → g3_types.h      :: rate_limit (inline)
 *
 * ===========================================================================
 *  FOC / MOTOR CONTROL
 * ===========================================================================
 *
 * sub_8002224  → motor/foc.c     :: foc_current_control_entry
 * sub_8002250  → motor/foc.c     :: foc_pwm_config (14.4kHz setup)
 * sub_8002794  → drivers          :: adc_channel_config
 * sub_8002874  → fixed_point.h   :: sat_add_check (inline)
 * sub_8002990  → ride_ctrl.c     :: foc_pid_init (configure PID + observer)
 * sub_8002A04  → motor/foc.c     :: foc_observer_param_select
 * sub_8002A18  → motor/foc.c     :: foc_get_torque_ref
 * sub_8002A1E  → motor/foc.c     :: foc_set_config_block
 * sub_8002A4E  → ride_ctrl.c     :: foc_speed_ctrl_entry (thunk)
 * sub_8002A54  → motor/foc.c     :: foc_current_limiter
 * sub_8002B54  → motor/foc.c     :: foc_speed_controller_tick (BIG)
 * sub_8002E8C  → motor/foc.c     :: foc_tick (PID + filter update)
 * sub_80030A8  → motor/foc.c     :: foc_current_control_entry (thunk)
 * sub_80030AC  → motor/foc.c     :: foc_measure_speed
 * sub_80030D8  → motor/foc.c     :: speed_with_temp_compensation
 * sub_8003064  → motor/foc.c     :: speed_to_rpm
 * sub_8003090  → motor/foc.c     :: is_motor_running
 * sub_8003118  → motor/foc.c     :: get_hall_speed_raw
 * sub_800312C  → main.c          :: calibration_sequence
 * sub_8004920  → motor/foc.c     :: get_motor_speed_q15
 * sub_8004974  → motor/foc.c     :: get_flux_d
 * sub_80049C8  → motor/foc.c     :: get_flux_compensated
 * sub_8004A84  → motor/foc.c     :: is_dual_battery_ok
 * sub_8004ACC  → motor/foc.c     :: foc_apply_params
 * sub_8004CFC  → ride_ctrl.c     :: freeze_position
 * sub_8004D20  → motor/foc.c     :: motor_param_update
 *
 * ===========================================================================
 *  RIDE CONTROL STATE MACHINE
 * ===========================================================================
 *
 * sub_80031BC  → main.c          :: system_init
 * sub_80032DC  → ride_ctrl.c     :: ride_ctrl_init (BIG INIT)
 * sub_8003394  → ride_ctrl.c     :: ride_ctrl_get_params
 * sub_8003568  → ride_ctrl.c     :: mode_off
 * sub_8003584  → ride_ctrl.c     :: is_error_state
 * sub_80035C4  → ride_ctrl.c     :: mode_sport
 * sub_800396C  → ride_ctrl.c     :: mode_locked
 * sub_8003940  → ride_ctrl.c     :: push_detected
 * sub_8003A18  → ride_ctrl.c     :: cruise_control
 * sub_8003B9C  → ride_ctrl.c     :: brake_process
 * sub_8003BDC  → ride_ctrl.c     :: throttle_process
 * sub_8003C2C  → ride_ctrl.c     :: mode_idle
 * sub_8003C90  → ride_ctrl.c     :: mode_eco
 * sub_8003E90  → ride_ctrl.c     :: mode_ride (BIGGEST FUNCTION)
 * sub_8004738  → ride_ctrl.c     :: get_brake_limit
 * sub_8004774  → peripherals.c   :: temperature_derating
 * sub_80048AC  → motor/foc.c     :: max_speed_from_speed
 * sub_8004A2C  → ride_ctrl.c     :: get_max_speed
 * sub_8004A4C  → ride_ctrl.c     :: get_brake_position
 * sub_8004A58  → ride_ctrl.c     :: get_cruise_speed
 * sub_8004A64  → ride_ctrl.c     :: is_cruise_enabled
 * sub_8004ABC  → ride_ctrl.c     :: is_regen_enabled
 * sub_8004CA4  → ride_ctrl.c     :: set_speed_limit
 * sub_8004DB0  → peripherals.c   :: check_error_flags
 * sub_8004DCC  → peripherals.c   :: check_dashboard_alive
 * sub_8004DE0  → ride_ctrl.c     :: ride_ctrl_process (MAIN TICK)
 * sub_8004F74  → ride_ctrl.c     :: first_ride_check
 * sub_8004FBC  → ride_ctrl.c     :: first_ride_init
 * sub_8004FF8  → ride_ctrl.c     :: post_mode_processing
 * sub_80050E4  → ride_ctrl.c     :: apply_final_limits
 * sub_8005124  → ride_ctrl.c     :: speed_filter_1
 * sub_80051A8  → ride_ctrl.c     :: apply_mode_to_foc
 * sub_8005224  → ride_ctrl.c     :: speed_filter_2
 * sub_800538C  → ride_ctrl.c     :: error_recovery
 * sub_80053E8  → ride_ctrl.c     :: safety_checks
 * sub_8005464  → ride_ctrl.c     :: regen_calc
 * sub_80054B8  → ride_ctrl.c     :: speed_limit_enforce
 * sub_800552C  → ride_ctrl.c     :: mode_config_apply
 * sub_80055F8  → ride_ctrl.c     :: direction_check
 * sub_800562C  → ride_ctrl.c     :: eco_mode_check
 * sub_800564C  → ride_ctrl.c     :: power_on_check
 * sub_80056D4  → ride_ctrl.c     :: speed_limit_calc
 * sub_8005750  → ride_ctrl.c     :: speed_config_copy
 * sub_8005798  → ride_ctrl.c     :: brake_force_calc
 * sub_8005898  → ride_ctrl.c     :: regen_brake_enabled
 * sub_80058E0  → ride_ctrl.c     :: mode_transition
 * sub_8005938  → peripherals.c   :: gpio_status_read
 * sub_8005968  → peripherals.c   :: observer_update
 *
 * ===========================================================================
 *  LED / LIGHT CONTROL
 * ===========================================================================
 *
 * sub_8002ED0  → peripherals.c   :: led_blink_slow
 * sub_8002F24  → peripherals.c   :: led_blink_fast
 * sub_8002F74  → peripherals.c   :: led_on
 * sub_8002F84  → peripherals.c   :: led_dim
 * sub_8002F94  → peripherals.c   :: led_pattern_update
 * sub_8002FC0  → drivers          :: tim_led_init
 * sub_8003054  → peripherals.c   :: led_off
 *
 * ===========================================================================
 *  PROTOCOL (UART + CAN)
 * ===========================================================================
 *
 * sub_800DFF6  → protocol.c      :: ninebot_send_frame
 * sub_800E03C  → protocol.c      :: can_rx_handler
 * sub_800E0E8  → protocol.c      :: telemetry_update
 * sub_800E1F0  → protocol.c      :: telemetry_extended_update
 * sub_800CCA8  → protocol.c      :: can_protocol_init
 * sub_800CCFC  → protocol.c      :: can_frame_dispatch
 * sub_800CD5C  → protocol.c      :: can_protocol_poll
 * sub_800CD68  → protocol.c      :: can_send_message
 * sub_800C224  → drivers          :: can_transmit
 * sub_800CB04  → protocol.c      :: checksum_compute
 * sub_800C680  → protocol.c      :: serial_register_read
 * sub_80031F4  → protocol.c      :: firmware_update_check
 * sub_800CBB2  → protocol.c      :: flash_write_page
 * sub_800CC54  → protocol.c      :: flash_write_halfwords
 *
 * ===========================================================================
 *  PERIPHERAL DRIVERS
 * ===========================================================================
 *
 * sub_800648C  → drivers          :: gpio_config_pin
 * sub_800C4C4  → drivers          :: rcc_periph_clock_cmd
 * sub_800C4EC  → drivers          :: rcc_periph_clock_cmd (duplicate)
 * sub_800C274  → drivers          :: rcc_ahb_reset
 * sub_800C328  → drivers          :: rcc_get_clocks
 * sub_800C454  → drivers          :: rcc_flag_check
 * sub_800C478  → drivers          :: rcc_hse_bypass
 * sub_800C488  → drivers          :: rcc_wait_hse_ready
 * sub_800C614  → main.c          :: clock_config (RCC deinit part)
 * sub_800C514  → main.c          :: clock_config (PLL config part)
 * sub_800C660  → drivers          :: rcc_sysclk_select
 * sub_800C670  → drivers          :: rcc_get_sysclk_source
 * sub_800B632  → drivers          :: tim_base_init
 * sub_800B65E  → drivers          :: tim_struct_init
 * sub_800B66C  → drivers          :: tim_enable
 * sub_800B676  → drivers          :: tim_status_flag
 * sub_800B686  → drivers          :: tim_clear_flag
 * sub_800B690  → drivers          :: tim_update_check
 * sub_800B6AA  → drivers          :: tim_interrupt_enable
 * sub_800B6C8  → drivers          :: tim_dma_enable
 * sub_800B6E2  → drivers          :: tim_oc_config
 * sub_800B8B0  → drivers          :: tim_oc_preload
 * sub_800B8EE  → drivers          :: tim_set_prescaler
 * sub_800B8FA  → drivers          :: tim_channel_config
 * sub_800BA52  → drivers          :: tim_read_capture
 * sub_800BA7E  → drivers          :: tim_arr_preload
 * sub_800DC62  → drivers          :: tim_pwm_init
 * sub_800DD2A  → drivers          :: tim_set_compare
 * sub_800DD54  → drivers          :: tim_set_repetition
 * sub_800DD4A  → drivers          :: tim_set_autoreload
 * sub_800DD5E  → drivers          :: tim_output_enable
 * sub_800DEA8  → drivers          :: tim_main_output_enable
 * sub_800DFB2  → drivers          :: tim_oc_struct_init
 * sub_800DEC2  → drivers          :: tim_oc_init
 * sub_800DCD4  → drivers          :: tim_oc_preload_config
 * sub_800DFE2  → drivers          :: tim_clock_division
 * sub_800DFEC  → drivers          :: tim_counter_mode
 * sub_800DFD8  → drivers          :: tim_set_prescaler_reg
 * sub_800DFCE  → drivers          :: tim_set_bit
 * sub_800E052  → drivers          :: tim_bdtr_config_bit6
 * sub_800E05C  → drivers          :: tim_bdtr_config_bit7
 * sub_800E070  → drivers          :: uart_baudrate_config
 * sub_800E0D4  → drivers          :: tim_smcr_bit2
 * sub_800E0DE  → drivers          :: tim_smcr_bit3
 * sub_800E28C  → main.c          :: watchdog_set_prescaler
 * sub_800E29C  → main.c          :: watchdog_init
 * sub_800E2AC  → main.c          :: watchdog_feed
 * sub_800E2C4  → main.c          :: watchdog_set_reload
 * sub_800BFE8  → drivers          :: can_init
 * sub_800BC78  → drivers          :: can_filter_init
 * sub_800C208  → drivers          :: can_deinit
 * sub_800CB28  → drivers          :: flash_set_status
 * sub_800CB34  → drivers          :: flash_write_halfword
 * sub_800CB58  → drivers          :: flash_lock
 * sub_800CB68  → drivers          :: flash_get_status
 * sub_800CB94  → drivers          :: flash_wait_ready
 * sub_800CBF8  → (builtin)       :: byte_copy
 * sub_800CC10  → protocol.c      :: flash_erase_page
 * sub_800CC3C  → protocol.c      :: flash_unlock
 * sub_800CF24  → drivers          :: nvic_enable_irq
 * sub_800CA54  → drivers          :: swap_u16
 *
 * ===========================================================================
 *  DEBOUNCE / TIMER UTILITIES
 * ===========================================================================
 *
 * sub_80099D0  → ride_ctrl.c     :: debounce_timer_init
 * sub_80098E4  → ride_ctrl.c     :: debounce_timer_tick
 * sub_80099FE  → ride_ctrl.c     :: debounce_timer_reset
 * sub_8009A0A  → ride_ctrl.c     :: debounce_timer_stop
 *
 * ===========================================================================
 *  OBSERVER / FILTER
 * ===========================================================================
 *
 * sub_80070D8  → motor/foc.c     :: lowpass2_update
 * sub_800710C  → motor/foc.c     :: lowpass2_output
 * sub_80071F8  → ride_ctrl.c     :: observer_init
 * sub_800723E  → ride_ctrl.c     :: observer_config
 * sub_800727A  → ride_ctrl.c     :: lowpass2_filter_tick
 * sub_80072AE  → ride_ctrl.c     :: observer_set_gains
 * sub_80072BC  → ride_ctrl.c     :: observer_set_bandwidth
 * sub_80072DC  → ride_ctrl.c     :: observer_start
 * sub_80075FC  → motor/foc.c     :: angle_estimator_reset
 * sub_80078BE  → motor/foc.c     :: foc_mode_select
 *
 * ===========================================================================
 *  SPEED OBSERVER / SENSORLESS
 * ===========================================================================
 *
 * sub_8007C30  → ride_ctrl.c     :: speed_observer_config
 * sub_8007C64  → ride_ctrl.c     :: speed_observer_set_limits
 * sub_8007D24  → ride_ctrl.c     :: speed_observer_update
 *
 * ===========================================================================
 *  MISCELLANEOUS
 * ===========================================================================
 *
 * sub_80032B0  → main.c          :: can_reinit
 * sub_80032C8  → main.c          :: can_post_init
 * sub_80032D6  → ride_ctrl.c     :: ride_ctrl_debug_entry (thunk)
 * sub_80085A8  → ride_ctrl.c     :: ride_ctrl_debug
 * sub_8008298  → ride_ctrl.c     :: observer_mode_set
 * sub_8008438  → peripherals.c   :: observer_update
 * sub_8008934  → protocol.c      :: serial_send_command
 * sub_8008EC4  → main.c          :: systick_init
 * sub_8008F04  → motor/foc.c     :: get_timer_value
 * sub_8008FD4  → motor/foc.c     :: timer_reset
 * sub_8008FE0  → motor/foc.c     :: timer_get_raw
 * sub_8009024  → motor/foc.c     :: timer_is_running
 * sub_8009528  → motor/foc.c     :: timer_set_period
 * sub_800973C  → motor/foc.c     :: hall_get_period
 * sub_8009744  → motor/foc.c     :: hall_timer_tick
 * sub_8009774  → motor/foc.c     :: hall_get_speed
 * sub_8009ED0  → main.c          :: nvic_config
 * sub_800A4F8  → ride_ctrl.c     :: throttle_observer_read
 * sub_800A4FC  → ride_ctrl.c     :: throttle_observer_level
 * sub_800A500  → ride_ctrl.c     :: throttle_observer_filtered
 * sub_800A514  → ride_ctrl.c     :: throttle_observer_output
 * sub_800A520  → ride_ctrl.c     :: throttle_observer_reset
 * sub_800A63C  → ride_ctrl.c     :: throttle_observer_enable
 * sub_800A644  → main.c          :: dma_init
 * sub_800A864  → peripherals.c   :: adc_to_temperature
 * sub_800A968  → main.c          :: speed_observer_init
 * sub_800CE10  → protocol.c      :: get_extended_status
 * sub_800CFA0  → protocol.c      :: can_poll_handler
 * sub_800CFC8  → protocol.c      :: can_dispatch_handler
 * sub_800D040  → protocol.c      :: can_multi_frame_send
 * sub_800D078  → protocol.c      :: can_handler_init_entry
 * sub_800D604  → protocol.c      :: can_frame_build
 * sub_800CDCC  → protocol.c      :: serial_checksum_verify
 * sub_8005E44  → protocol.c      :: uart_dma_send
 * sub_8005F90  → main.c          :: uart_init
 * sub_80064E8  → ride_ctrl.c     :: cruise_state_update
 * sub_80063B6  → protocol.c      :: flash_page_count
 * sub_800610E  → motor/foc.c     :: pwm_set_frequency
 */

#endif /* FUNCTION_MAP_H */
