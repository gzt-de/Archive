# Ninebot Max G3 — MCU Firmware Reconstruction

## Overview

This is a best-effort reconstruction of the Ninebot Max G3 motor controller (MCU)
firmware, reverse-engineered from IDA Pro pseudo-C decompilation output (~17,000 lines,
531 functions).

**Target MCU:** STM32F0xx series (Cortex-M0, base address 0x08000000)
**Clock:** 48 MHz (HSI48 / PLL from 8 MHz HSI)

## Architecture

The firmware implements a complete **Field-Oriented Control (FOC)** BLDC/PMSM motor
controller with the following subsystems:

| Module | Description |
|---|---|
| `main.c` | Entry point, system init, main loop |
| `motor/foc.c` | Field-Oriented Control: Clarke/Park transforms, SVM, current loops |
| `motor/speed_ctrl.c` | Speed controller, PID, ramp generation |
| `motor/observer.c` | Sensorless rotor position observer / flux estimator |
| `motor/hall.c` | Hall sensor interface and commutation |
| `motor/pwm.c` | Timer/PWM generation for 3-phase bridge |
| `app/ride_ctrl.c` | Ride state machine: idle, eco, normal, sport, push, cruise |
| `app/brake.c` | Regenerative braking, EBS (electronic braking) |
| `app/throttle.c` | Throttle processing, filtering, safety checks |
| `app/lights.c` | LED / headlight / taillight control |
| `app/error.c` | Error detection and fault handling |
| `protocol/ninebot_serial.c` | Ninebot serial protocol (UART) — register read/write |
| `protocol/can_bus.c` | CAN bus communication with dashboard/BMS |
| `drivers/stm32f0_*.c` | Low-level STM32F0 peripheral drivers |
| `config/parameters.h` | Tuning parameters, speed limits, current limits |

## Fixed-Point Math

The firmware uses **Q15 fixed-point** arithmetic throughout (1.0 = 32768 = 0x8000).
Some functions use Q30. Software floating-point routines are present for parameter
conversions but not in the real-time control loop.

## Key Constants (Q15 unless noted)

- Motor pole pairs: derived from electrical angle calculations
- Speed limit: configurable via serial protocol (word_200003D4, etc.)
- Current limit: configurable per ride mode
- Battery voltage: monitored via ADC
- Temperature: monitored for derating

## Memory Map

| Region | Address | Usage |
|---|---|---|
| Flash | 0x08000000 | Code + const data |
| SRAM | 0x20000000 | Variables, buffers, state |
| Periph | 0x40000000 | STM32 peripherals |

### Key Peripheral Addresses

| Address | Peripheral |
|---|---|
| 0x40003000 | IWDG (Independent Watchdog) |
| 0x40006400 | bxCAN |
| 0x40010400 | EXTI |
| 0x40011000 | GPIO (various ports) |
| 0x40012C00 | TIM1 (motor PWM) |
| 0x40014000 | TIM15/16/17 |
| 0x40020000 | DMA |
| 0x40021000 | RCC |
| 0x40022000 | Flash interface |

## Build

This is a reference reconstruction, NOT directly compilable production firmware.
To build for an actual STM32F0 target you would need:

1. ARM GCC toolchain (`arm-none-eabi-gcc`)
2. STM32 CMSIS headers
3. Linker script for the specific STM32F0 variant
4. A `Makefile` or CMake project

A skeleton `Makefile` is provided for reference.

## Disclaimer

This reconstruction is for **educational and research purposes only**.
Embedded firmware structures, calibration data, and lookup tables that were not
recoverable from the decompilation are filled with placeholder values.
