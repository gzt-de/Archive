# Ninebot Max G3 - VCU Firmware Reconstruction

## Overview

This is a reconstructed source code project for the **Ninebot Max G3** Vehicle Control Unit (VCU), reverse-engineered from IDA Pro decompilation output of the original firmware binary.

**Target MCU:** STM32F103 (ARM Cortex-M3, 72MHz, 128KB Flash, 20KB SRAM)

## Project Structure

```
g3_vcu_reconstructed/
├── inc/
│   ├── vcu_types.h         # All type definitions, structs, enums, hardware addresses
│   ├── hal_drivers.h       # Hardware abstraction layer declarations
│   └── address_map.h       # IDA address → symbol mapping (453 functions documented)
├── src/
│   ├── main.c              # Entry point, task scheduler, initialization
│   ├── state_machine.c     # 8-state scooter state machine (core logic)
│   ├── serial_protocol.c   # Ninebot UART serial protocol (0x5A 0xA5 framing)
│   ├── can_protocol.c      # CAN bus protocol for ESC/BMS communication
│   ├── authentication.c    # BLE challenge-response lock authentication
│   ├── sensors.c           # ADC reading, fault detection, maintenance
│   ├── ride_control.c      # Speed limiting, headlights, LEDs, ride modes
│   ├── config_manager.c    # Dual-bank flash config, device identity
│   └── runtime_helpers.c   # Soft-float, memcpy/memset (compiler runtime)
├── drivers/
│   └── hal_drivers.c       # GPIO, Flash, RCC, ADC, I2C, IWDG implementations
└── Makefile                # ARM GCC build reference
```

## Architecture

### State Machine (8 States)

| State | Name     | Description |
|-------|----------|-------------|
| 0     | STANDBY  | Off/idle, waiting for power-on trigger |
| 1     | STARTING | Power-on sequence, peripheral init |
| 2     | PAIRING  | BLE authentication / password entry |
| 3     | READY    | Normal riding operation |
| 4     | LOCKING  | Lock procedure active |
| 5     | CRUISE   | IoT connected mode (BLE-locked riding) |
| 6     | SHUTDOWN | Shutdown sequence, save config |
| 7     | LOCKED   | Locked with alarm armed |

### Task Scheduler

The VCU uses a cooperative scheduler with 5 periodic tasks:

| Task | Period | Function |
|------|--------|----------|
| 0    | 100ms  | Main state machine, ride control |
| 1    | 100ms  | ESC/motor controller communication |
| 2    | 100ms  | CAN bus periodic messages |
| 3    | 10ms   | Fast ADC sampling, button debounce |
| 4    | 10ms   | Authentication timing |

### Communication Buses

- **UART1:** BLE module (Ninebot serial protocol, 0x5A/0xA5 framing)
- **UART2/3:** ESC motor controller
- **CAN:** BMS, external battery, dashboard display
- **I2C:** EEPROM / accelerometer (for alarm detection)

### Serial Protocol Frame

```
[0x5A] [0xA5] [LEN] [SRC] [DST] [CMD] [SUBCMD] [DATA...] [CRC16_LO] [CRC16_HI]
```

Address assignments:
- `0x00` - Dashboard/Display
- `0x02` - BMS (Battery Management)
- `0x04` - ESC (Motor Controller)
- `0x06/07` - External battery
- `0x16` (22) - VCU (this unit)
- `0x23` (35) - Configuration handler
- `0x3E` (62) - BLE module
- `0x3F` (63) - IoT module

### Authentication

The G3 uses a button-press challenge-response system:
1. Scooter generates random challenge → derives expected key length (4-8 presses)
2. User enters sequence using up/down/power/fn buttons
3. VCU computes CRC32-like hash of the button sequence
4. Match = unlock, 3 failures = lockout with increasing timeout

## Reconstruction Notes

### What's Complete
- Full state machine with all 8 states and transitions
- Serial protocol (framing, checksum, TX queue, RX dispatch)
- CAN bus protocol (standard/multiframe messages, firmware update)
- Authentication system (challenge-response, lockout logic)
- Sensor/ADC reading with fault detection
- Configuration dual-bank flash storage
- All HAL drivers (GPIO, Flash, RCC, ADC, I2C, IWDG, Power)
- Complete IDA address mapping for ~200 key functions

### What Uses Placeholder/Pseudo Values
- `g_flash_sectors[]` - Flash sector address table (needs real values from binary)
- Individual state handler implementations (`state_standby_handler`, etc.) - provided as weak stubs
- Some ESC/BMS protocol details (proprietary data interpretation)
- LED pattern data tables
- Speed curve / acceleration tables
- I2C device addresses and register maps for accelerometer/EEPROM

### Known Limitations
- Some function cross-references may be approximate
- Float arithmetic uses standard C (original is hand-optimized ARM assembly)
- UART DMA configuration not fully reconstructed
- Timer interrupt configuration is simplified
- ~250 of 453 original functions have detailed reconstructions; the rest are stubs or referenced

## Building

This is a reference project. To build a working binary:
1. Install ARM GCC toolchain (`arm-none-eabi-gcc`)
2. Add an STM32F103 linker script (`stm32f103_flash.ld`)
3. Add startup assembly file
4. Run `make`

## License

This reconstruction is for educational and research purposes only.
