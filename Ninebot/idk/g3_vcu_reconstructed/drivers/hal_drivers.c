/**
 * @file hal_drivers.c
 * @brief STM32F103 low-level peripheral drivers
 *
 * Reconstructed from IDA Pro decompilation.
 * sub_800D0F0 = gpio_clear_bits
 * sub_800D0F4 = gpio_set_bits
 * sub_800D0F8 = gpio_read_input
 * sub_800D110 = gpio_configure_pin
 * sub_800D20C = gpio_read_bits
 * sub_800CE4C = flash_set_latency
 * sub_800CE70 = flash_write_halfword
 * sub_800CE94 = flash_lock
 * sub_800CEEC = flash_erase_page
 * sub_800CF18 = flash_unlock
 * sub_800E158 = rcc_init_clocks
 * sub_800E81C = iwdg_reload
 * sub_800D330 = pwr_enter_stop_mode
 */

#include "hal_drivers.h"

/* ============================================================================
 * GPIO Implementations
 * ============================================================================ */

/* sub_800D0F4 - Set bits in GPIO ODR register */
void gpio_set_bits(uint32_t gpio_base, uint32_t bits)
{
    volatile uint32_t *bsrr = (volatile uint32_t *)(gpio_base + GPIO_BSRR_OFFSET);
    *bsrr = bits;
}

/* sub_800D0F0 - Clear bits in GPIO ODR register */
void gpio_clear_bits(uint32_t gpio_base, uint32_t bits)
{
    volatile uint32_t *brr = (volatile uint32_t *)(gpio_base + GPIO_BRR_OFFSET);
    *brr = bits;
}

/* sub_800D20C - Read bits from GPIO IDR */
uint32_t gpio_read_bits(uint32_t gpio_base, uint32_t bits)
{
    volatile uint32_t *idr = (volatile uint32_t *)(gpio_base + GPIO_IDR_OFFSET);
    return *idr & bits;
}

/* sub_800D0F8 - Read a specific GPIO pin */
uint32_t gpio_read_input(uint32_t gpio_base, uint32_t pin)
{
    volatile uint32_t *idr = (volatile uint32_t *)(gpio_base + GPIO_IDR_OFFSET);
    return (*idr >> pin) & 1;
}

/* sub_800D110 - Configure a GPIO pin's mode and CNF bits
 * For STM32F103, CRL handles pins 0-7, CRH handles pins 8-15.
 * Each pin takes 4 bits: [CNF1:CNF0:MODE1:MODE0]
 */
void gpio_configure_pin(uint32_t gpio_base, uint8_t pin, uint8_t mode, uint8_t cnf)
{
    volatile uint32_t *cr;
    uint32_t shift;
    uint32_t tmp;

    if (pin < 8) {
        cr = (volatile uint32_t *)(gpio_base + GPIO_CRL_OFFSET);
        shift = pin * 4;
    } else {
        cr = (volatile uint32_t *)(gpio_base + GPIO_CRH_OFFSET);
        shift = (pin - 8) * 4;
    }

    tmp = *cr;
    tmp &= ~(0xF << shift);
    tmp |= ((uint32_t)((cnf << 2) | mode) & 0xF) << shift;
    *cr = tmp;
}

/* ============================================================================
 * RCC (Clock) Implementations
 * ============================================================================ */

/* sub_800E158 - Initialize system clocks
 * Configure HSE -> PLL -> 72MHz SYSCLK
 * AHB prescaler = /1, APB1 = /2 (36MHz), APB2 = /1 (72MHz)
 */
int rcc_init_clocks(void)
{
    /* Enable HSE oscillator */
    RCC_CR |= (1 << 16);   /* HSEON */

    /* Wait for HSE ready */
    while (!(RCC_CR & (1 << 17)))
        ;

    /* Configure flash wait states for 72MHz: 2 wait states */
    flash_set_latency(0x00000032, 0);  /* FLASH_ACR = PRFTBE | LATENCY_2 */

    /* Configure PLL: HSE as source, multiply by 9 -> 8MHz * 9 = 72MHz */
    RCC_CFGR &= 0xFFC0FFFF;
    RCC_CFGR |= (0x07 << 18);  /* PLLMUL = 9 */
    RCC_CFGR |= (1 << 16);     /* PLLSRC = HSE */

    /* Set AHB prescaler /1, APB1 /2, APB2 /1 */
    RCC_CFGR &= ~(0xF << 4);   /* HPRE = /1 */
    RCC_CFGR &= ~(0x7 << 8);
    RCC_CFGR |= (0x4 << 8);    /* PPRE1 = /2 */
    RCC_CFGR &= ~(0x7 << 11);  /* PPRE2 = /1 */

    /* Enable PLL */
    RCC_CR |= (1 << 24);       /* PLLON */
    while (!(RCC_CR & (1 << 25)))
        ;

    /* Switch SYSCLK to PLL */
    RCC_CFGR &= ~0x3;
    RCC_CFGR |= 0x2;           /* SW = PLL */
    while ((RCC_CFGR & 0xC) != 0x8)
        ;

    /* Enable peripheral clocks */
    /* APB2: GPIOA, GPIOB, GPIOC, GPIOD, AFIO, ADC1, USART1 */
    *(volatile uint32_t *)(RCC_BASE + 0x18) |= 0x0000FC7D;
    /* APB1: TIM2, TIM3, USART2, USART3, I2C1, CAN, PWR, BKP */
    *(volatile uint32_t *)(RCC_BASE + 0x1C) |= 0x18E60003;
    /* AHB: DMA1 */
    *(volatile uint32_t *)(RCC_BASE + 0x14) |= 0x00000001;

    return 0;
}

/* sub_8009768 - Reset RCC to defaults (for bootloader jump) */
int rcc_reset_to_hsi(void)
{
    /* Reset backup domain clock */
    RCC_BDCR |= 0x10000000;
    *(volatile uint32_t *)(PWR_BASE + 0x50) |= 4;
    RCC_BDCR &= ~0x10000000;

    /* Switch to HSI */
    RCC_CR |= 1;               /* HSION */
    while (!(RCC_CR & 2))
        ;

    /* Clear SW bits -> HSI as SYSCLK */
    RCC_CFGR &= 0xFFFFFFFC;
    while ((RCC_CFGR & 0xC) != 0)
        ;

    /* Disable PLL, HSE */
    RCC_CR &= 0xFEF2FFFF;
    RCC_CFGR = 0;

    /* Restore flash configuration */
    /* FLASH_ACR values from decompilation: 7952 = 0x1F10, 0x100000 */
    *(volatile uint32_t *)(FLASH_BASE_REG + 0x0C) = 7952;
    *(volatile uint32_t *)(FLASH_BASE_REG + 0x10) = 0x100000;
    *(volatile uint32_t *)(FLASH_BASE_REG + 0x08) = 10420224;

    /* Set VTOR to base flash */
    SCB_VTOR = 0x08000000;

    return 0x08000000;
}

void rcc_backup_config(void)
{
    extern uint32_t g_rcc_cfgr_backup;
    extern uint32_t g_rcc_cr_backup;
    g_rcc_cfgr_backup = RCC_CFGR;
    g_rcc_cr_backup = RCC_CR;
}

void rcc_restore_config(void)
{
    extern uint32_t g_rcc_cfgr_backup;
    extern uint32_t g_rcc_cr_backup;
    if (g_rcc_cfgr_backup && g_rcc_cr_backup) {
        if (RCC_CFGR != g_rcc_cfgr_backup || RCC_CR != g_rcc_cr_backup) {
            RCC_CFGR = g_rcc_cfgr_backup;
            RCC_CR = g_rcc_cr_backup;
        }
    }
}

void rcc_set_pll_config(uint32_t config)
{
    RCC_CFGR = (RCC_CFGR & 0xFFFF3FFF) | ((config & 3) << 14);
    uint32_t result = (RCC_CFGR & 0xEFFFFFFF) | (((config >> 2) & 1) << 28);
    RCC_CFGR = result;
}

/* ============================================================================
 * Flash Memory
 * ============================================================================ */

#define FLASH_KEYR  (*(volatile uint32_t *)(FLASH_BASE_REG + 0x04))
#define FLASH_SR    (*(volatile uint32_t *)(FLASH_BASE_REG + 0x0C))
#define FLASH_CR_R  (*(volatile uint32_t *)(FLASH_BASE_REG + 0x10))
#define FLASH_AR    (*(volatile uint32_t *)(FLASH_BASE_REG + 0x14))

/* sub_800CF18 */
void flash_unlock(void)
{
    FLASH_KEYR = 0x45670123;
    FLASH_KEYR = 0xCDEF89AB;
}

/* sub_800CE94 */
void flash_lock(void)
{
    FLASH_CR_R |= (1 << 7);    /* LOCK bit */
}

/* sub_800CEEC - Erase one 1KB page, returns 3 on success */
int flash_erase_page(uint32_t address)
{
    /* Wait for BSY */
    while (FLASH_SR & 1)
        ;

    FLASH_CR_R |= (1 << 1);    /* PER = 1 (page erase) */
    FLASH_AR = address;
    FLASH_CR_R |= (1 << 6);    /* STRT */

    /* Wait for completion */
    while (FLASH_SR & 1)
        ;

    FLASH_CR_R &= ~(1 << 1);   /* PER = 0 */

    /* Verify erased (check EOP) */
    if (FLASH_SR & (1 << 5)) {
        FLASH_SR = (1 << 5);    /* Clear EOP */
        return 3;
    }
    return 0;
}

/* sub_800CE70 - Write a half-word to flash, returns 3 on success */
int flash_write_halfword(uint16_t *addr, uint16_t data)
{
    while (FLASH_SR & 1)
        ;

    FLASH_CR_R |= (1 << 0);    /* PG = 1 (programming) */
    *addr = data;

    while (FLASH_SR & 1)
        ;

    FLASH_CR_R &= ~(1 << 0);

    if (*addr == data)
        return 3;
    return 0;
}

void flash_set_latency(uint32_t flags, uint32_t value)
{
    volatile uint32_t *acr = (volatile uint32_t *)(FLASH_BASE_REG + 0x00);
    *acr = (*acr & ~flags) | value;
}

void flash_read_to_ram(uint32_t src_addr, void *dst, uint32_t size)
{
    const uint8_t *src = (const uint8_t *)src_addr;
    uint8_t *d = (uint8_t *)dst;
    for (uint32_t i = 0; i < size; i++)
        d[i] = src[i];
}

int flash_write_from_ram(const void *src, uint32_t dst_addr, uint32_t size)
{
    /* Not fully reconstructed - uses flash_write_halfword in a loop */
    const uint16_t *s = (const uint16_t *)src;
    uint16_t *d = (uint16_t *)dst_addr;
    for (uint32_t i = 0; i < size / 2; i++) {
        if (flash_write_halfword(&d[i], s[i]) != 3)
            return 0;
    }
    return 1;
}

uint32_t flash_pages_for_size(uint32_t size)
{
    if (size & 0x3FF)   /* size % 1024 != 0 */
        return (size >> 10) + 1;
    else
        return size >> 10;
}

bool flash_erase_range(uint32_t address, uint32_t size)
{
    uint32_t pages = flash_pages_for_size(size);
    flash_unlock();
    flash_set_latency(20, 0);

    uint32_t i;
    for (i = 0; i < pages; i++) {
        if (flash_erase_page(address + (i << 10)) != 3)
            break;
    }
    flash_lock();
    return (i == pages);
}

bool flash_program_verified(uint16_t *dst, uint16_t *src, uint32_t size)
{
    flash_unlock();
    uint32_t i;
    for (i = 0; i < size; i += 2) {
        if (flash_write_halfword(dst, *src) != 3 || *dst != *src)
            break;
        dst++;
        src++;
    }
    flash_lock();
    return (i >= size);
}

/* ============================================================================
 * IWDG (Watchdog)
 * ============================================================================ */

/* sub_800E81C - Kick the watchdog */
void iwdg_reload(void)
{
    IWDG_KR = 0xAAAA;  /* Reload key */
}

void iwdg_init(uint32_t prescaler, uint32_t reload)
{
    IWDG_KR = 0x5555;  /* Enable access */
    IWDG_PR = prescaler;
    *(volatile uint32_t *)(IWDG_BASE + 0x08) = reload;
    IWDG_KR = 0xCCCC;  /* Start watchdog */
}

/* ============================================================================
 * ADC
 * ============================================================================ */

/* sub_800DE0C */
int adc_init(void)
{
    /* Configure ADC1 - details from decompilation are sparse, using typical STM32F1 init */
    volatile uint32_t *adc_cr2 = (volatile uint32_t *)(ADC1_BASE + 0x08);
    volatile uint32_t *adc_cr1 = (volatile uint32_t *)(ADC1_BASE + 0x04);
    volatile uint32_t *adc_smpr = (volatile uint32_t *)(ADC1_BASE + 0x0C);

    /* Power on ADC */
    *adc_cr2 |= (1 << 0);      /* ADON */
    /* Small delay for stabilization */
    for (volatile int i = 0; i < 1000; i++);
    /* Calibrate */
    *adc_cr2 |= (1 << 2);      /* CAL */
    while (*adc_cr2 & (1 << 2))
        ;

    return 0;
}

/* sub_80018FC */
uint16_t adc_read_throttle(void)
{
    volatile uint32_t *sqr3 = (volatile uint32_t *)(ADC1_BASE + 0x34);
    volatile uint32_t *cr2 = (volatile uint32_t *)(ADC1_BASE + 0x08);
    volatile uint32_t *sr = (volatile uint32_t *)(ADC1_BASE + 0x00);
    volatile uint32_t *dr = (volatile uint32_t *)(ADC1_BASE + 0x4C);

    *sqr3 = ADC_CH_THROTTLE;
    *cr2 |= (1 << 22);         /* SWSTART */
    while (!(*sr & (1 << 1)))   /* Wait EOC */
        ;
    return (uint16_t)*dr;
}

/* sub_8001908 */
uint16_t adc_read_brake(void)
{
    volatile uint32_t *sqr3 = (volatile uint32_t *)(ADC1_BASE + 0x34);
    volatile uint32_t *cr2 = (volatile uint32_t *)(ADC1_BASE + 0x08);
    volatile uint32_t *sr = (volatile uint32_t *)(ADC1_BASE + 0x00);
    volatile uint32_t *dr = (volatile uint32_t *)(ADC1_BASE + 0x4C);

    *sqr3 = ADC_CH_BRAKE;
    *cr2 |= (1 << 22);
    while (!(*sr & (1 << 1)))
        ;
    return (uint16_t)*dr;
}

/* sub_8001914 */
uint16_t adc_read_voltage(void)
{
    volatile uint32_t *sqr3 = (volatile uint32_t *)(ADC1_BASE + 0x34);
    volatile uint32_t *cr2 = (volatile uint32_t *)(ADC1_BASE + 0x08);
    volatile uint32_t *sr = (volatile uint32_t *)(ADC1_BASE + 0x00);
    volatile uint32_t *dr = (volatile uint32_t *)(ADC1_BASE + 0x4C);

    *sqr3 = ADC_CH_BATTERY_V;
    *cr2 |= (1 << 22);
    while (!(*sr & (1 << 1)))
        ;
    return (uint16_t)*dr;
}

/* sub_8001920 */
uint16_t adc_read_temperature(void)
{
    volatile uint32_t *sqr3 = (volatile uint32_t *)(ADC1_BASE + 0x34);
    volatile uint32_t *cr2 = (volatile uint32_t *)(ADC1_BASE + 0x08);
    volatile uint32_t *sr = (volatile uint32_t *)(ADC1_BASE + 0x00);
    volatile uint32_t *dr = (volatile uint32_t *)(ADC1_BASE + 0x4C);

    *sqr3 = ADC_CH_TEMP_BOARD;
    *cr2 |= (1 << 22);
    while (!(*sr & (1 << 1)))
        ;
    return (uint16_t)*dr;
}

int adc_trigger_conversion(int channel)
{
    volatile uint32_t *sqr3 = (volatile uint32_t *)(ADC1_BASE + 0x34);
    volatile uint32_t *cr2 = (volatile uint32_t *)(ADC1_BASE + 0x08);
    *sqr3 = (uint32_t)channel;
    *cr2 |= (1 << 22);
    return 0;
}

/* ============================================================================
 * Power Management
 * ============================================================================ */

/* sub_800D330 - Enter STOP mode */
void pwr_enter_stop_mode(void)
{
    /* Set SLEEPDEEP in SCB->SCR */
    *(volatile uint32_t *)0xE000ED10 |= (1 << 2);

    /* Set PWR_CR.PDDS = 0 (STOP mode, not standby) */
    PWR_CR &= ~(1 << 1);
    /* Set LPDS for low-power regulator */
    PWR_CR |= (1 << 0);

    /* WFI */
    __asm volatile ("wfi");

    /* Clear SLEEPDEEP after wakeup */
    *(volatile uint32_t *)0xE000ED10 &= ~(1 << 2);
}

void pwr_enable_bkp_access(void)
{
    PWR_CR |= (1 << 8);    /* DBP - Disable backup domain write protection */
}

/* sub_800605C */
int pwr_set_keep_alive(int enable)
{
    if (enable == 1)
        return (int)(intptr_t)gpio_set_bits(GPIOA_BASE, 4);
    else
        return (int)(intptr_t)gpio_clear_bits(GPIOA_BASE, 4);
    return 0;
}

/* sub_8006004 */
int pwr_set_led_driver(int enable)
{
    if (enable != 1) {
        gpio_clear_bits(GPIOB_BASE + 0x400, 64);  /* GPIOC */
        g_scooter.led_status = g_scooter.led_status & 0xFE7F;
        return 0;
    }
    gpio_set_bits(GPIOB_BASE + 0x400, 64);

    if (g_scooter.iot_lock_state != 2 &&
        g_scooter.iot_lock_state != 1 &&
        g_scooter.iot_lock_state != 5) {
        if (g_scooter.status_word3 & 4) {
            g_scooter.led_status = (g_scooter.led_status & 0xFE7F) + 128;
        }
    }
    return 0;
}

/* ============================================================================
 * BKP (Backup Registers)
 * ============================================================================ */

/* sub_800D370 */
uint16_t bkp_read(uint8_t index)
{
    /* STM32F103 BKP data registers at BKP_BASE + 0x04 + (index*4) for DR1-DR10 */
    volatile uint16_t *bkp_dr = (volatile uint16_t *)(BKP_BASE + 0x04 + index * 4);
    return *bkp_dr;
}

/* sub_800D390 */
void bkp_write(uint8_t index, uint16_t value)
{
    pwr_enable_bkp_access();
    volatile uint16_t *bkp_dr = (volatile uint16_t *)(BKP_BASE + 0x04 + index * 4);
    *bkp_dr = value;
}

/* ============================================================================
 * Delay
 * ============================================================================ */

void delay_ms(uint32_t ms)
{
    /* Simple busy-wait; real implementation might use SysTick */
    volatile uint32_t count = ms * 8000;  /* ~72MHz / 9 cycles per loop */
    while (count--)
        ;
}
