/**
 * @file stm32f0_regs.h
 * @brief STM32F0xx peripheral register definitions (reconstructed)
 *
 * Derived from memory-mapped I/O addresses found in the decompiled firmware.
 * These match the STM32F0xx reference manual (RM0091).
 */

#ifndef STM32F0_REGS_H
#define STM32F0_REGS_H

#include <stdint.h>

/* ========================================================================== */
/*  Base addresses                                                             */
/* ========================================================================== */

#define PERIPH_BASE         0x40000000U
#define APB1_BASE           (PERIPH_BASE + 0x00000000U)
#define APB2_BASE           (PERIPH_BASE + 0x00010000U)
#define AHB1_BASE           (PERIPH_BASE + 0x00020000U)
#define AHB2_BASE           (PERIPH_BASE + 0x08000000U)

/* ========================================================================== */
/*  Peripheral base addresses                                                  */
/* ========================================================================== */

#define TIM2_BASE           (APB1_BASE + 0x0000)
#define TIM3_BASE           (APB1_BASE + 0x0400)
#define TIM6_BASE           (APB1_BASE + 0x1000)
#define TIM7_BASE           (APB1_BASE + 0x1400)
#define TIM14_BASE          (APB1_BASE + 0x2000)
#define IWDG_BASE           (APB1_BASE + 0x3000)   /* 0x40003000 */
#define USART2_BASE         (APB1_BASE + 0x4400)
#define I2C1_BASE           (APB1_BASE + 0x5400)
#define CAN_BASE            (APB1_BASE + 0x6400)    /* 0x40006400 — bxCAN */

#define SYSCFG_BASE         (APB2_BASE + 0x0000)
#define EXTI_BASE           (APB2_BASE + 0x0400)    /* 0x40010400 */
#define ADC_BASE            (APB2_BASE + 0x2400)
#define TIM1_BASE           (APB2_BASE + 0x2C00)    /* 0x40012C00 — Motor PWM */
#define SPI1_BASE           (APB2_BASE + 0x3000)
#define USART1_BASE         (APB2_BASE + 0x3800)
#define TIM15_BASE          (APB2_BASE + 0x4000)    /* 0x40014000 */
#define TIM16_BASE          (APB2_BASE + 0x4400)
#define TIM17_BASE          (APB2_BASE + 0x4800)

#define DMA_BASE            (AHB1_BASE + 0x0000)    /* 0x40020000 */
#define RCC_BASE            (AHB1_BASE + 0x1000)    /* 0x40021000 */
#define FLASH_IF_BASE       (AHB1_BASE + 0x2000)    /* 0x40022000 */

#define GPIOA_BASE          (AHB2_BASE + 0x0000)    /* 0x48000000 */
#define GPIOB_BASE          (AHB2_BASE + 0x0400)
#define GPIOC_BASE          (AHB2_BASE + 0x0800)
#define GPIOD_BASE          (AHB2_BASE + 0x0C00)
#define GPIOF_BASE          (AHB2_BASE + 0x1400)

/* ========================================================================== */
/*  Register structures                                                        */
/* ========================================================================== */

typedef struct {
    volatile uint32_t CR;           /* 0x00 */
    volatile uint32_t CFGR;         /* 0x04 */
    volatile uint32_t CIR;          /* 0x08 */
    volatile uint32_t APB2RSTR;     /* 0x0C */
    volatile uint32_t APB1RSTR;     /* 0x10 */
    volatile uint32_t AHBENR;       /* 0x14 */
    volatile uint32_t APB2ENR;      /* 0x18 */
    volatile uint32_t APB1ENR;      /* 0x1C */
    volatile uint32_t BDCR;         /* 0x20 */
    volatile uint32_t CSR;          /* 0x24 */
    volatile uint32_t AHBRSTR;      /* 0x28 */
    volatile uint32_t CFGR2;        /* 0x2C */
    volatile uint32_t CFGR3;        /* 0x30 */
    volatile uint32_t CR2;          /* 0x34 */
} RCC_TypeDef;

typedef struct {
    volatile uint32_t MODER;        /* 0x00 */
    volatile uint32_t OTYPER;       /* 0x04 */
    volatile uint32_t OSPEEDR;      /* 0x08 */
    volatile uint32_t PUPDR;        /* 0x0C */
    volatile uint32_t IDR;          /* 0x10 */
    volatile uint32_t ODR;          /* 0x14 */
    volatile uint32_t BSRR;         /* 0x18 */
    volatile uint32_t LCKR;         /* 0x1C */
    volatile uint32_t AFR[2];       /* 0x20-0x24 */
    volatile uint32_t BRR;          /* 0x28 */
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t CR1;          /* 0x00 */
    volatile uint32_t CR2;          /* 0x04 */
    volatile uint32_t SMCR;         /* 0x08 */
    volatile uint32_t DIER;         /* 0x0C */
    volatile uint32_t SR;           /* 0x10 */
    volatile uint32_t EGR;          /* 0x14 */
    volatile uint32_t CCMR1;        /* 0x18 */
    volatile uint32_t CCMR2;        /* 0x1C */
    volatile uint32_t CCER;         /* 0x20 */
    volatile uint32_t CNT;          /* 0x24 */
    volatile uint32_t PSC;          /* 0x28 */
    volatile uint32_t ARR;          /* 0x2C */
    volatile uint32_t RCR;          /* 0x30 */
    volatile uint32_t CCR1;         /* 0x34 */
    volatile uint32_t CCR2;         /* 0x38 */
    volatile uint32_t CCR3;         /* 0x3C */
    volatile uint32_t CCR4;         /* 0x40 */
    volatile uint32_t BDTR;         /* 0x44 */
    volatile uint32_t DCR;          /* 0x48 */
    volatile uint32_t DMAR;         /* 0x4C */
} TIM_TypeDef;

typedef struct {
    volatile uint32_t ISR;          /* 0x00 */
    volatile uint32_t IER;          /* 0x04 */
    volatile uint32_t CR;           /* 0x08 */
    volatile uint32_t CFGR1;        /* 0x0C */
    volatile uint32_t CFGR2;        /* 0x10 */
    volatile uint32_t SMPR;         /* 0x14 */
    volatile uint32_t RESERVED1;
    volatile uint32_t RESERVED2;
    volatile uint32_t TR;           /* 0x20 */
    volatile uint32_t RESERVED3;
    volatile uint32_t CHSELR;       /* 0x28 */
    volatile uint32_t RESERVED4[5];
    volatile uint32_t DR;           /* 0x40 */
} ADC_TypeDef;

typedef struct {
    volatile uint32_t MCR;          /* 0x000 — CAN master control */
    volatile uint32_t MSR;          /* 0x004 */
    volatile uint32_t TSR;          /* 0x008 */
    volatile uint32_t RF0R;         /* 0x00C */
    volatile uint32_t RF1R;         /* 0x010 */
    volatile uint32_t IER;          /* 0x014 */
    volatile uint32_t ESR;          /* 0x018 */
    volatile uint32_t BTR;          /* 0x01C */
    volatile uint32_t RESERVED0[88];
    volatile uint32_t TI0R;         /* 0x180 */
    volatile uint32_t TDT0R;        /* 0x184 */
    volatile uint32_t TDL0R;        /* 0x188 */
    volatile uint32_t TDH0R;        /* 0x18C */
    /* ... more TX/RX mailboxes ... */
} CAN_TypeDef;

typedef struct {
    volatile uint32_t KR;           /* 0x00 — Key register */
    volatile uint32_t PR;           /* 0x04 — Prescaler */
    volatile uint32_t RLR;          /* 0x08 — Reload */
    volatile uint32_t SR;           /* 0x0C — Status */
} IWDG_TypeDef;

typedef struct {
    volatile uint32_t ACR;          /* 0x00 */
    volatile uint32_t KEYR;         /* 0x04 */
    volatile uint32_t OPTKEYR;      /* 0x08 */
    volatile uint32_t SR;           /* 0x0C */
    volatile uint32_t CR;           /* 0x10 */
    volatile uint32_t AR;           /* 0x14 */
    volatile uint32_t RESERVED;
    volatile uint32_t OBR;          /* 0x1C */
    volatile uint32_t WRPR;         /* 0x20 */
} FLASH_TypeDef;

typedef struct {
    volatile uint32_t CR1;          /* 0x00 */
    volatile uint32_t CR2;          /* 0x04 */
    volatile uint32_t CR3;          /* 0x08 */
    volatile uint32_t BRR;          /* 0x0C */
    volatile uint32_t RESERVED1;
    volatile uint32_t RTOR;         /* 0x14 */
    volatile uint32_t RQR;          /* 0x18 */
    volatile uint32_t ISR;          /* 0x1C */
    volatile uint32_t ICR;          /* 0x20 */
    volatile uint32_t RDR;          /* 0x24 */
    volatile uint32_t TDR;          /* 0x28 */
} USART_TypeDef;

/* ========================================================================== */
/*  Peripheral instances                                                       */
/* ========================================================================== */

#define RCC         ((RCC_TypeDef   *)  RCC_BASE)
#define GPIOA       ((GPIO_TypeDef  *)  GPIOA_BASE)
#define GPIOB       ((GPIO_TypeDef  *)  GPIOB_BASE)
#define GPIOC       ((GPIO_TypeDef  *)  GPIOC_BASE)
#define TIM1        ((TIM_TypeDef   *)  TIM1_BASE)
#define TIM2        ((TIM_TypeDef   *)  TIM2_BASE)
#define TIM3        ((TIM_TypeDef   *)  TIM3_BASE)
#define TIM15       ((TIM_TypeDef   *)  TIM15_BASE)
#define TIM16       ((TIM_TypeDef   *)  TIM16_BASE)
#define TIM17       ((TIM_TypeDef   *)  TIM17_BASE)
#define ADC1        ((ADC_TypeDef   *)  ADC_BASE)
#define CAN1        ((CAN_TypeDef   *)  CAN_BASE)
#define IWDG        ((IWDG_TypeDef  *)  IWDG_BASE)
#define FLASH_IF    ((FLASH_TypeDef *)  FLASH_IF_BASE)
#define USART1      ((USART_TypeDef *)  USART1_BASE)
#define USART2      ((USART_TypeDef *)  USART2_BASE)

/* ========================================================================== */
/*  Bit definitions (commonly used in firmware)                                */
/* ========================================================================== */

/* RCC_CR */
#define RCC_CR_HSION        (1U << 0)
#define RCC_CR_HSIRDY       (1U << 1)
#define RCC_CR_HSEON        (1U << 16)
#define RCC_CR_HSERDY       (1U << 17)
#define RCC_CR_PLLON        (1U << 24)
#define RCC_CR_PLLRDY       (1U << 25)

/* RCC_CFGR */
#define RCC_CFGR_SW_Msk     (3U << 0)
#define RCC_CFGR_SWS_Msk    (3U << 2)
#define RCC_CFGR_PLLSRC     (1U << 16)
#define RCC_CFGR_PLLXTPRE   (1U << 17)

/* IWDG */
#define IWDG_KEY_RELOAD     0xAAAAU
#define IWDG_KEY_ENABLE     0xCCCCU
#define IWDG_KEY_UNLOCK     0x5555U

/* FLASH */
#define FLASH_KEY1          0x45670123U
#define FLASH_KEY2          0xCDEF89ABU     /* Note: 0xCDEF89AB = ~0x32106754+1... */
                                            /* Firmware uses 0xCDEF89AB (-839939669 signed) */

/* NVIC */
#define NVIC_ISER           (*(volatile uint32_t *)0xE000E100U)
#define NVIC_ICER           (*(volatile uint32_t *)0xE000E180U)
#define SCB_AIRCR           (*(volatile uint32_t *)0xE000ED0CU)

#endif /* STM32F0_REGS_H */
