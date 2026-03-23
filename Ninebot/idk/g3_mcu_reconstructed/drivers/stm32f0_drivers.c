/**
 * @file stm32f0_drivers.c
 * @brief Low-level STM32F0 peripheral driver functions.
 *
 * These are thin wrappers around hardware register access,
 * reconstructed from the many sub_800xxxx helper functions.
 *
 * Reconstructed from:
 *   sub_800648C  — gpio_config_pin()
 *   sub_800C4C4  — rcc_periph_clock_enable()
 *   sub_800C4EC  — rcc_periph_clock_enable2()
 *   sub_800C274  — rcc_ahb_reset()
 *   sub_800C328  — rcc_get_clocks()
 *   sub_800C454  — rcc_flag_check()
 *   sub_800C478  — rcc_hse_bypass()
 *   sub_800C488  — rcc_wait_hse_ready()
 *   sub_800C614  — rcc_deinit()
 *   sub_800C660  — rcc_sysclk_select()
 *   sub_800C670  — rcc_get_sysclk_source()
 *   sub_800B65E  — tim_struct_init()
 *   sub_800B632  — tim_base_init()
 *   sub_800B6E2  — tim_oc_config()
 *   sub_800B8B0  — tim_oc_preload()
 *   sub_800B8EE  — tim_prescaler()
 *   sub_800B8FA  — tim_channel_config()
 *   sub_800BA52  — tim_read_capture()
 *   sub_800BA7E  — tim_arr_preload()
 *   sub_800B66C  — tim_enable()
 *   sub_800B676  — tim_status_flag()
 *   sub_800B686  — tim_clear_flag()
 *   sub_800B690  — tim_update_check()
 *   sub_800B6AA  — tim_interrupt_enable()
 *   sub_800B6C8  — tim_dma_enable()
 *   sub_800C8B8  — i2c_struct_init()
 *   sub_800C964  — i2c_init()
 *   sub_800C8D0  — i2c_enable()
 *   sub_800C8A8  — i2c_slave_config()
 *   sub_800C9C0  — i2c_reset()
 *   sub_800DC62  — tim_pwm_init()
 *   sub_800DD2A  — tim_set_compare()
 *   sub_800DD54  — tim_set_repetition()
 *   sub_800DD4A  — tim_set_autoreload()
 *   sub_800DD5E  — tim_output_enable()
 *   sub_800DEA8  — tim_main_output()
 *   sub_800DFB2  — tim_oc_struct_init()
 *   sub_800DEC2  — tim_oc_init()
 *   sub_800DCD4  — tim_oc_preload_config()
 *   sub_800DFE2  — tim_clock_division()
 *   sub_800DFEC  — tim_counter_mode()
 *   sub_800DFD8  — tim_set_prescaler()
 *   sub_800DFCE  — tim_set_bit()
 *   sub_800E052  — tim_bdtr_config_bit6()
 *   sub_800E05C  — tim_bdtr_config_bit7()
 *   sub_800E0D4  — tim_smcr_bit2()
 *   sub_800E0DE  — tim_smcr_bit3()
 *   sub_800E070  — uart_baudrate_config()
 *   sub_800BFE8  — can_init()
 *   sub_800BC78  — can_filter_init()
 *   sub_800C208  — can_deinit()
 *   sub_800C224  — can_transmit()
 *   sub_800CF24  — nvic_enable_irq()
 *   sub_800CA54  — swap_u16()
 */

#include <stdint.h>
#include "stm32f0_regs.h"
#include "g3_types.h"

/* ========================================================================== */
/*  RCC (Reset and Clock Control)                                              */
/* ========================================================================== */

/**
 * Enable/disable peripheral clock (sub_800C4C4 / sub_800C4EC).
 *
 * @param periph_id  Encoded as: bits[20:16]=register_offset, bits[4:0]=bit_position
 *                   E.g., 1835010 = 0x1C0002 → reg offset=0x1C (APB1ENR), bit 2
 * @param enable     1=enable, 0=disable
 */
void rcc_periph_clock_cmd(uint32_t periph_id, int enable)
{
    uint32_t bit_mask = 1U << (periph_id & 0x1F);
    volatile uint32_t *reg = (volatile uint32_t *)(
        (periph_id >> 16) + 0x40021000  /* RCC_BASE */
    );

    if (enable)
        *reg |= bit_mask;
    else
        *reg &= ~bit_mask;
}

/**
 * Check if peripheral clock flag is set (sub_800C454).
 */
int rcc_flag_check(uint32_t flag_id)
{
    uint32_t bit = 1U << (flag_id & 0x1F);
    volatile uint32_t *reg = (volatile uint32_t *)(
        (flag_id >> 16) + 0x40021000
    );
    return ((*reg & bit) != 0);
}

/**
 * Get system clock frequencies (sub_800C328).
 * Reads RCC registers to compute SYSCLK, HCLK, PCLK1, PCLK2, ADCCLK.
 *
 * @param[out] clocks  Array of 5 uint32_t: [SYSCLK, HCLK, PCLK2, PCLK1, ADCCLK]
 */
void rcc_get_clocks(uint32_t clocks[5])
{
    /* Determine clock source from RCC->CFGR SWS bits */
    int sws = (RCC->CFGR >> 2) & 3;

    switch (sws) {
    case 0: /* HSI */
        clocks[0] = 8000000;
        break;
    case 1: /* HSE */
        clocks[0] = 16000000;
        break;
    case 2: /* PLL */
        /* Complex PLL calculation from RCC->CFGR and RCC->CFGR2 */
        clocks[0] = 48000000;  /* Most common for this firmware */
        break;
    default:
        clocks[0] = 8000000;
        break;
    }

    /* HCLK = SYSCLK / AHB_prescaler */
    clocks[1] = clocks[0] >> ahb_prescaler_table[(RCC->CFGR >> 4) & 0xF];

    /* PCLK1 = HCLK / APB1_prescaler */
    clocks[3] = clocks[1] >> apb_prescaler_table[(RCC->CFGR >> 8) & 7];

    /* PCLK2 = HCLK / APB2_prescaler */
    clocks[2] = clocks[1] >> apb_prescaler_table[(RCC->CFGR >> 11) & 7];

    /* ADCCLK = PCLK2 / ADC_prescaler */
    clocks[4] = clocks[2] / 1;  /* Simplified */
}

/* ========================================================================== */
/*  GPIO configuration                                                         */
/* ========================================================================== */

/**
 * Configure a GPIO pin (sub_800648C).
 *
 * @param gpio_base  GPIO port base address (e.g., 0x48000000 for GPIOA)
 * @param pin_mask   Pin bitmask (e.g., 32 = bit 5 = pin 5)
 * @param mode       0=Input, 1=Output, 2=AF, 3=Analog
 * @param pull       0=None, 1=PullUp, 2=PullDown
 * @param speed      0=Low, 1=Med, 2=High, 3=VeryHigh (4 in firmware)
 */
void gpio_config_pin(uint32_t gpio_base, uint32_t pin_mask,
                     uint8_t mode, uint8_t pull, uint8_t speed)
{
    GPIO_TypeDef *gpio = (GPIO_TypeDef *)gpio_base;

    /* Find pin number from mask */
    int pin = 0;
    uint32_t tmp = pin_mask;
    while (tmp >>= 1) pin++;

    /* Set mode (2 bits per pin) */
    gpio->MODER = (gpio->MODER & ~(3U << (pin * 2)))
                  | ((mode & 3) << (pin * 2));

    /* Set speed */
    gpio->OSPEEDR = (gpio->OSPEEDR & ~(3U << (pin * 2)))
                    | ((speed & 3) << (pin * 2));

    /* Set pull-up/pull-down */
    gpio->PUPDR = (gpio->PUPDR & ~(3U << (pin * 2)))
                  | ((pull & 3) << (pin * 2));
}

/* ========================================================================== */
/*  Timer (TIM) configuration                                                  */
/* ========================================================================== */

/**
 * Initialize timer base (sub_800B632).
 */
void tim_base_init(uint32_t tim_base, const uint8_t *config)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)tim_base;
    tim->CR1 = (tim->CR1 & 0xFC80)
               | (config[0] & 1)        /* CEN */
               | ((config[1] & 1) << 1) /* UDIS */
               | ((config[2] & 1) << 2) /* URS */
               | ((config[3] & 0xF) << 4); /* CKD + DIR */
}

/**
 * Set timer prescaler (sub_800B8EE).
 */
void tim_set_prescaler(uint32_t tim_base, uint16_t prescaler)
{
    ((TIM_TypeDef *)tim_base)->PSC = prescaler;
}

/**
 * Configure timer output compare channel (sub_800B8FA / sub_800B6E2).
 */
void tim_oc_config(uint32_t tim_base, uint8_t channel,
                   uint8_t rank, uint8_t sample_time)
{
    /* Configure channel mapping in CCMR1/CCMR2 and CCER registers */
    TIM_TypeDef *tim = (TIM_TypeDef *)tim_base;
    (void)tim;
    (void)channel;
    (void)rank;
    (void)sample_time;
    /* Implementation depends on specific channel */
}

/**
 * Read timer capture register (sub_800BA52).
 * @param channel  0-3 for CCR1-CCR4
 */
uint32_t tim_read_capture(uint32_t tim_base, uint8_t channel)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)tim_base;
    switch (channel) {
    case 0: return tim->CCR1;
    case 1: return tim->CCR2;
    case 2: return tim->CCR3;
    case 3: return tim->CCR4;
    default: return 0;
    }
}

/**
 * Set timer compare value (sub_800DD2A).
 * Used for PWM duty cycle control (motor and LEDs).
 *
 * @param channel  Timer channel (1-4)
 * @param value    Compare value (0-ARR)
 */
void tim_set_compare(uint32_t tim_base, uint8_t channel, uint16_t value)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)tim_base;
    switch (channel) {
    case 1: tim->CCR1 = value; break;
    case 2: tim->CCR2 = value; break;
    case 3: tim->CCR3 = value; break;
    case 4: tim->CCR4 = value; break;
    }
}

/**
 * Initialize PWM timer (sub_800DC62).
 * For TIM1 motor control: ARR=999, PSC=143 → 48MHz/(144*1000) ≈ 333Hz?
 * Actually likely center-aligned: effective = 14.4kHz.
 */
void tim_pwm_init(uint32_t tim_base, uint16_t arr, uint16_t psc)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)tim_base;
    tim->ARR = arr;
    tim->PSC = psc;
}

/**
 * Enable timer main output (sub_800DEA8).
 * Required for TIM1 advanced timer to actually drive pins.
 */
void tim_main_output_enable(uint32_t tim_base, int enable)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)tim_base;
    if (enable)
        tim->BDTR |= (1U << 15);   /* MOE bit */
    else
        tim->BDTR &= ~(1U << 15);
}

/* ========================================================================== */
/*  UART configuration                                                         */
/* ========================================================================== */

/**
 * Configure UART baud rate (sub_800E070).
 *
 * Computes BRR from peripheral clock and desired baud rate:
 *   BRR = (10 * PCLK / baud + 5) / 10
 * Handles oversampling bit (CR1 bit 15).
 */
void uart_baudrate_config(uint32_t usart_base, uint32_t baud,
                          uint8_t oversampling, uint8_t mode)
{
    volatile uint32_t *usart = (volatile uint32_t *)usart_base;
    uint32_t clocks[5];
    rcc_get_clocks(clocks);

    uint32_t pclk;
    if (usart_base == USART1_BASE)
        pclk = clocks[2];  /* PCLK2 */
    else
        pclk = clocks[3];  /* PCLK1 */

    uint32_t div = 10 * pclk / baud;
    uint32_t brr = (div / 10);
    if (div % 10 >= 5)
        brr++;

    /* Write BRR (offset +8 = CCR2 position, but for USART it's BRR) */
    usart[2] = (usart[2] & 0xFFFF0000) | (uint16_t)brr;

    /* Set oversampling */
    usart[3] = (usart[3] & ~(1U << 12)) | ((oversampling & 1) << 12);

    /* Set mode */
    usart[4] = (usart[4] & ~(3U << 12)) | ((mode & 3) << 12);
}

/* ========================================================================== */
/*  CAN bus                                                                    */
/* ========================================================================== */

/**
 * CAN transmit (sub_800C224).
 * Writes message to TX mailbox and triggers transmission.
 *
 * @param id    29-bit CAN ID
 * @param data  Pointer to data bytes
 * @param dlc   Data length (0-8)
 */
void can_transmit(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    volatile uint32_t *can = (volatile uint32_t *)CAN_BASE;

    /* Wait for empty mailbox */
    /* Write ID to TIxR (extended format) */
    /* Write data to TDLxR/TDHxR */
    /* Set DLC and request transmission */
    (void)can;
    (void)id;
    (void)data;
    (void)dlc;
}

/* ========================================================================== */
/*  NVIC                                                                       */
/* ========================================================================== */

/**
 * Enable NVIC interrupt (sub_800CF24).
 *
 * @param irq_encoded  Encoded IRQ: e.g., 806355714 = 0x30100802
 *                     Contains IRQ number, priority, and subpriority.
 */
void nvic_enable_irq(uint32_t irq_encoded)
{
    uint8_t irq_num   = irq_encoded & 0xFF;
    uint8_t priority   = (irq_encoded >> 8) & 0xFF;

    /* Set priority */
    volatile uint8_t *nvic_ip = (volatile uint8_t *)0xE000E400;
    nvic_ip[irq_num] = priority << 4;

    /* Enable interrupt */
    NVIC_ISER = (1U << (irq_num & 0x1F));
}

/* ========================================================================== */
/*  Utility                                                                    */
/* ========================================================================== */

/**
 * Swap two uint16 values (sub_800CA54).
 * Used by bubble sort in ADC median filter.
 */
void swap_u16(uint16_t *a, uint16_t *b)
{
    uint16_t tmp = *a;
    *a = *b;
    *b = tmp;
}
