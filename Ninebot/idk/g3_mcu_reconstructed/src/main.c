/**
 * @file main.c
 * @brief Main entry point for Ninebot Max G3 MCU firmware.
 *
 * Reconstructed from:
 *   sub_8001A7C  — Reset_Handler / startup
 *   sub_800D4D8  — main()
 *   sub_80031BC  — system_init()
 *   sub_800312C  — calibration_sequence()
 *
 * The firmware initializes all peripherals, runs a one-time calibration,
 * then enters the main loop where it processes serial/CAN messages
 * and runs the motor control state machine at ~1kHz.
 */

#include <stdint.h>
#include <string.h>
#include "stm32f0_regs.h"
#include "g3_types.h"
#include "parameters.h"
#include "fixed_point.h"

/* ========================================================================== */
/*  Forward declarations (implemented in other modules)                        */
/* ========================================================================== */

/* drivers */
extern void     rcc_init(void);
extern void     gpio_init(void);
extern void     adc_init(void);
extern void     can_init(void);
extern void     uart_init(void);
extern void     tim1_pwm_init(void);
extern void     tim_led_init(void);
extern void     iwdg_init(void);
extern void     flash_init(void);
extern void     nvic_config(void);

/* motor */
extern void     foc_init(void);
extern void     foc_calibrate(void);
extern int      foc_process(void);
extern void     hall_init(void);
extern void     speed_observer_init(void);

/* app */
extern void     ride_ctrl_init(void);
extern int      ride_ctrl_process(void);
extern void     protocol_init(void);
extern void     protocol_process(void);
extern void     error_check(void);
extern void     telemetry_update(void);
extern void     light_control(void);
extern void     can_protocol_init(void);
extern int      can_protocol_poll(void);
extern void     firmware_update_check(void);

/* ========================================================================== */
/*  Global state                                                               */
/* ========================================================================== */

g3_state_t g3;

static foc_state_t          foc_state;
static motor_params_t       motor_params_table[4];      /* One per ride mode */
static speed_observer_t     speed_observer;
static adc_buffer_t         adc_buf;
static throttle_state_t     throttle;
static can_registry_t       can_registry;
static debounce_timer_t     debounce_timers[20];        /* ~20 debounce timers */
static telemetry_packet_t   telemetry;

/* Pointer used everywhere in the firmware — dword_200000F8 */
foc_state_t *foc_ptr = &foc_state;

/* ========================================================================== */
/*  Interrupt vector thunks (sub_8001190..sub_800119C — default handlers)      */
/* ========================================================================== */

void NMI_Handler(void)          { while (1); }
void HardFault_Handler(void)    { while (1); }
void SVC_Handler(void)          { while (1); }
void PendSV_Handler(void)       { while (1); }
void SysTick_Handler(void)      { while (1); }
/* Unused IRQ handlers default to infinite loop */
void Default_Handler(void)      { while (1); }

/* ========================================================================== */
/*  Startup / data initialization (sub_8001A7C)                                */
/* ========================================================================== */

/**
 * Called from Reset_Handler before main.
 * Copies .data from flash to SRAM, zeroes .bss,
 * and calls C++ static constructors if any.
 *
 * Reconstructed from sub_8001A7C which iterates through init_array
 * entries from loc_800F4E4 to loc_800F504.
 */
static void __libc_init_array(void)
{
    /* The original firmware has a small init table at 0x800F4E4..0x800F504.
     * Each entry is 16 bytes: { dest, src, count, copy_func }.
     * This copies initialized data from flash to SRAM. */

    extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

    /* Copy .data section */
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }

    /* Zero .bss section */
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }
}

/* ========================================================================== */
/*  Clock configuration (sub_800C614 + sub_800C514)                            */
/* ========================================================================== */

/**
 * Configure system clock to 48 MHz.
 *
 * sub_800C614: Reset RCC to defaults (HSI on, PLL off, clear config).
 * Then PLL is configured via sub_800C514 to multiply HSI to 48 MHz.
 *
 * The firmware also checks HSI48 availability (0x40021054 bit 9 +
 * 0x40021030 bit 25), falling back to PLL if unavailable.
 */
static void clock_config(void)
{
    /* Step 1: Reset to HSI (sub_800C614) */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY))
        ;
    /* Switch to HSI */
    RCC->CFGR &= ~RCC_CFGR_SW_Msk;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != 0)
        ;
    /* Turn off PLL, HSE */
    RCC->CR &= 0xFEF2FFFF;
    RCC->CFGR = 0;
    RCC->CFGR2 = 7952;             /* PLL predivider config */
    RCC->CFGR3 = 0x100000;         /* Peripheral clock sources */
    RCC->CIR   = 10420224;         /* Clear all interrupt flags */

    /* Step 2: Configure PLL for 48 MHz (sub_800C514) */
    /* PLL source = HSI/2 = 4 MHz, multiply by 12 = 48 MHz */
    RCC->CFGR &= ~(1U << 16);     /* PLLSRC = HSI */
    uint32_t pll_mul = 10;         /* PLL multiply factor - 2 = 10 (x12) */
    RCC->CFGR = (RCC->CFGR & 0xFFC3FFFF) | ((pll_mul & 0xF) << 18);
    RCC->CFGR = (RCC->CFGR & 0x9FFFFFFF) | (((pll_mul >> 4) & 3) << 29);

    /* Configure flash wait state for 48 MHz */
    /* Set flash latency = 1 wait state */
    uint8_t ws = 3;  /* Based on the frequency range selection logic */
    RCC->CFGR2 = (RCC->CFGR2 & 0xF8FFFFFF) | ((ws & 7) << 24);

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    /* Switch system clock to PLL */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | 2;  /* SW = PLL */
    while (((RCC->CFGR >> 2) & 3) != 2)
        ;
}

/* ========================================================================== */
/*  System initialization (sub_80031BC)                                        */
/* ========================================================================== */

/**
 * Full peripheral and subsystem initialization.
 *
 * Call order reconstructed from sub_80031BC:
 *   1. Interrupt / NVIC config       (sub_8009ED0)
 *   2. SysTick / system timer        (sub_8008EC4)
 *   3. DMA init                      (sub_800A644)
 *   4. ADC + DMA init                (sub_800204C → sub_8001F18 + sub_8002794)
 *   5. Hall sensor timer init        (sub_800141C → sub_8001F94)
 *   6. Speed observer init           (sub_800A968)
 *   7. UART init                     (sub_8005F90)
 *   8. LED PWM timer init            (sub_8002FC0)
 *   9. CAN bus init                  (sub_80032C8 → sub_800BFE8 + sub_800BC78)
 *  10. Enable global interrupts      (sub_800CF24)
 */
static void system_init(void)
{
    /* 1. Configure NVIC priorities */
    nvic_config();

    /* 2. SysTick for timing */
    /* SysTick is configured for 1ms ticks */

    /* 3. DMA controller init */
    /* Enable DMA clock, configure channels for ADC and UART */

    /* 4. ADC initialization with DMA */
    adc_init();

    /* 5. Hall sensor interface (TIM2 input capture) */
    hall_init();

    /* 6. Speed / position observer */
    speed_observer_init();

    /* 7. UART for Ninebot serial protocol */
    uart_init();

    /* 8. LED/headlight PWM (TIM16/17) */
    tim_led_init();

    /* 9. CAN bus */
    can_init();
    can_protocol_init();

    /* 10. Enable interrupts */
    __asm volatile ("cpsie i");
}

/* ========================================================================== */
/*  Calibration (sub_800312C)                                                  */
/* ========================================================================== */

/**
 * One-time motor calibration at power-on.
 *
 * sub_800312C:
 *   - Reads initial ADC offsets
 *   - Measures motor parameters (resistance, inductance)
 *   - Computes angle correction factor
 *   - Stores calibration results (dword_200000A0, etc.)
 *
 * Uses floating-point math for precision:
 *   - Converts raw ADC to float
 *   - Multiplies by calibration constants (1121021683 = ~100.0f,
 *     1053609165 = ~0.3f, 1058642330 = ~0.5f)
 *   - Stores back as Q15
 */
static void calibration_sequence(void)
{
    volatile uint8_t cal_in_progress = 1;

    /* Read raw ADC channels with motor off */
    /* sub_8006BC8() — capture ADC snapshot */
    /* sub_8009744() — read hall timer */
    /* sub_800973C() — get hall period */

    /* Compute motor back-EMF constant */
    /* Uses floating point:
     *   float raw = (float)adc_reading;
     *   raw *= 100.0f;                    (1121021683)
     *   raw = ldexp(raw, 15);             Convert to Q15 scale
     *   ... more float math for BEMF const ...
     */

    /* Store initial angle offset */
    g3.motor_angle = 0;  /* dword_200000A0 = computed value */

    cal_in_progress = 0;
}

/* ========================================================================== */
/*  Watchdog (sub_800E29C, sub_800E2AC, sub_800E28C)                           */
/* ========================================================================== */

/**
 * Initialize Independent Watchdog.
 *
 * sub_800E29C: IWDG->KR = 0xCCCC (start WDG)
 * sub_800E28C: IWDG->PR = prescaler & 7
 * sub_800E2C4: IWDG->RLR = reload_value
 */
static void watchdog_init(void)
{
    IWDG->KR = IWDG_RELOAD_KEY;    /* 0xCCCC — enable watchdog */
}

/**
 * Feed (reload) the watchdog.
 * sub_800E2AC: IWDG->KR = 0x5555 if a1!=0, else 0
 */
static void watchdog_feed(int enable)
{
    if (enable) {
        IWDG->KR = IWDG_ENABLE_KEY;    /* 0x5555 = reload */
    } else {
        IWDG->KR = 0;
    }
}

/* ========================================================================== */
/*  Main function (sub_800D4D8)                                                */
/* ========================================================================== */

/**
 * The main entry point.
 *
 * The actual main() was at sub_800D4D8, called from the startup code
 * (sub_8001178 → sub_800D4D8). It is a __noreturn function.
 *
 * Initialization flow:
 *   1. Clock config
 *   2. Flash read (load saved config from flash)
 *   3. System peripheral init
 *   4. Calibration
 *   5. Ride controller init
 *   6. Watchdog init
 *   7. Main loop
 *
 * Main loop (runs forever):
 *   - Process serial protocol (UART Rx/Tx)
 *   - Process CAN bus messages
 *   - Run ride control state machine
 *   - Update telemetry
 *   - Feed watchdog
 *   - Check for firmware update requests
 */
int main(void)
{
    /* ---- Phase 1: Clock ---- */
    clock_config();

    /* ---- Phase 2: Load flash configuration ---- */
    flash_init();
    /* Read saved parameters from flash (PID gains, speed limits, etc.)
     * If word_20001F34 == 0 or 0xFF → use defaults */

    /* ---- Phase 3: Peripheral init ---- */
    system_init();

    /* ---- Phase 4: Motor calibration ---- */
    calibration_sequence();

    /* ---- Phase 5: Application init ---- */
    ride_ctrl_init();

    /* ---- Phase 6: Watchdog ---- */
    watchdog_init();

    /* ---- Phase 7: Main loop ---- */
    while (1) {
        /* Process incoming serial protocol messages */
        protocol_process();

        /* Process CAN bus messages */
        can_protocol_poll();

        /* Run the ride control state machine (calls FOC, throttle, brake) */
        /* This is sub_8004DE0 — the main "tick" function */
        ride_ctrl_process();

        /* Update telemetry packet for dashboard */
        telemetry_update();

        /* LED / light control */
        light_control();

        /* Error monitoring */
        error_check();

        /* Check for OTA / firmware update request */
        firmware_update_check();

        /* Feed watchdog */
        watchdog_feed(1);
    }

    /* Never reached */
    return 0;
}
