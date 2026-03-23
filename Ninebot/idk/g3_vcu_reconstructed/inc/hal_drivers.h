/**
 * @file hal_drivers.h
 * @brief STM32F103 Hardware Abstraction Layer
 *
 * Reconstructed register-level peripheral drivers.
 */

#ifndef HAL_DRIVERS_H
#define HAL_DRIVERS_H

#include "vcu_types.h"

/* ============================================================================
 * GPIO
 * ============================================================================ */

/** Set bits in GPIO ODR (sub_800D0F4) */
void gpio_set_bits(uint32_t gpio_base, uint32_t bits);

/** Clear bits in GPIO ODR (sub_800D0F0) */
void gpio_clear_bits(uint32_t gpio_base, uint32_t bits);

/** Read specific bits from GPIO IDR (sub_800D20C) */
uint32_t gpio_read_bits(uint32_t gpio_base, uint32_t bits);

/** Read GPIO input pin (sub_800D0F8) */
uint32_t gpio_read_input(uint32_t gpio_base, uint32_t pin);

/** Configure GPIO pin mode (sub_800D110) */
void gpio_configure_pin(uint32_t gpio_base, uint8_t pin, uint8_t mode, uint8_t cnf);

/* ============================================================================
 * RCC (Reset and Clock Control)
 * ============================================================================ */

/** Initialize system clocks - HSE, PLL to 72MHz (sub_800E158) */
int rcc_init_clocks(void);

/** Reset clock configuration to HSI default (sub_8009768) */
int rcc_reset_to_hsi(void);

/** Configure PLL multiplier (sub_800E208) */
void rcc_set_pll_config(uint32_t config);

/** Backup and restore RCC registers around sleep */
void rcc_backup_config(void);
void rcc_restore_config(void);

/* ============================================================================
 * Flash Memory
 * ============================================================================ */

/** Unlock flash for write (sub_800CF18) */
void flash_unlock(void);

/** Lock flash after write (sub_800CE94) */
void flash_lock(void);

/** Erase flash page (sub_800CEEC) - returns 3 on success */
int flash_erase_page(uint32_t address);

/** Write half-word to flash (sub_800CE70) - returns 3 on success */
int flash_write_halfword(uint16_t *addr, uint16_t data);

/** Set flash wait states (sub_800CE4C) */
void flash_set_latency(uint32_t flags, uint32_t value);

/** Read flash data into RAM buffer (sub_8005FAE / sub_8005C60) */
void flash_read_to_ram(uint32_t src_addr, void *dst, uint32_t size);

/** Write RAM buffer to flash (sub_8005D18) */
int flash_write_from_ram(const void *src, uint32_t dst_addr, uint32_t size);

/** Erase flash range (sub_8005F70) */
bool flash_erase_range(uint32_t address, uint32_t size);

/** Compute number of pages for given size (sub_8005F5C) */
uint32_t flash_pages_for_size(uint32_t size);

/** Program flash (combined erase + write) for firmware update (sub_8005FC0) */
bool flash_program_verified(uint16_t *dst, uint16_t *src, uint32_t size);

/* ============================================================================
 * IWDG (Independent Watchdog)
 * ============================================================================ */

/** Feed the watchdog (sub_800E81C) */
void iwdg_reload(void);

/** Initialize watchdog with given prescaler (sub_800E82C / sub_800E83C) */
void iwdg_init(uint32_t prescaler, uint32_t reload);

/* ============================================================================
 * ADC
 * ============================================================================ */

/** Initialize ADC1 (sub_800DE0C) */
int adc_init(void);

/** Read single ADC channel (sub_80018FC, sub_8001908, sub_8001914, sub_8001920) */
uint16_t adc_read_throttle(void);
uint16_t adc_read_brake(void);
uint16_t adc_read_voltage(void);
uint16_t adc_read_temperature(void);

/** Trigger ADC conversion (sub_800DE50) */
int adc_trigger_conversion(int channel);

/* ============================================================================
 * UART / USART
 * ============================================================================ */

/** Initialize UART for serial comms (sub_800649C, sub_80058D4) */
int uart_init_serial(void);
int uart_init_esc(void);

/** Transmit data on serial UART (sub_8005820) */
int uart_serial_transmit(uint8_t *data, uint16_t length);

/** UART interrupt handler / DMA (sub_8009A4C) */
void uart_process_rx(uint8_t *buf, uint32_t usart_base, uint32_t flags);

/* ============================================================================
 * I2C
 * ============================================================================ */

/** Initialize I2C bus (sub_800A770) */
int i2c_init(void);

/** I2C write byte (sub_8006288) */
int i2c_write_byte(int dev_addr, int reg);

/** I2C read byte (sub_80062C4) */
int i2c_read_byte(int dev_addr, int reg);

/** I2C write multiple (sub_80062FE) */
int i2c_write_multi(int dev_addr, int reg_start, int count);

/** I2C read multiple (sub_800634E) */
int i2c_read_multi(int dev_addr, int reg_start, int count);

/** I2C low-level start/stop/byte transfer (sub_8008F9C) */
int i2c_transfer_byte(int data, int flags);

/** I2C bus speed selection (sub_80096F4) */
int i2c_set_speed(int speed_mode);

/* ============================================================================
 * CAN Bus
 * ============================================================================ */

/** Initialize CAN peripheral (sub_6DB8) */
int can_init(void);

/** Build and send CAN message (sub_800CFF4) */
int can_send_message(uint8_t dst, uint8_t src, uint8_t len,
                     uint8_t cmd, uint8_t subcmd, void *data);

/** CAN RX handler (called from interrupt) */
void can_rx_handler(void *msg);

/* ============================================================================
 * Timer / PWM
 * ============================================================================ */

/** Configure system tick for task scheduler (sub_800CF30) */
void systick_init(void);

/** Configure general-purpose timer (sub_800C860) */
void timer_init(uint32_t period_us);

/** Timer-based delay (sub_800E062 / similar) */
void delay_ms(uint32_t ms);

/* ============================================================================
 * Power Management
 * ============================================================================ */

/** Enter low-power stop mode (sub_800D330) */
void pwr_enter_stop_mode(void);

/** Enable backup domain access */
void pwr_enable_bkp_access(void);

/** Configure power-on / power-off GPIO (sub_800605C) */
int pwr_set_keep_alive(int enable);

/** Set LED driver enable (sub_8006004) */
int pwr_set_led_driver(int enable);

/* ============================================================================
 * BKP (Backup Registers / RTC)
 * ============================================================================ */

/** Read backup register (sub_800D370) */
uint16_t bkp_read(uint8_t index);

/** Write backup register (sub_800D390) */
void bkp_write(uint8_t index, uint16_t value);

#endif /* HAL_DRIVERS_H */
