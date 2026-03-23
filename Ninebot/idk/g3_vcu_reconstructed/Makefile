# =============================================================================
# Ninebot Max G3 VCU - Reconstructed Firmware
# Target: STM32F103 (Cortex-M3, 72MHz, 128KB Flash, 20KB SRAM)
#
# This Makefile is for reference/documentation. Building a working binary
# would require the ARM GCC toolchain and proper linker script.
# =============================================================================

# Toolchain
CC      = arm-none-eabi-gcc
AS      = arm-none-eabi-as
LD      = arm-none-eabi-ld
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size

# Target
TARGET  = g3_vcu

# Directories
SRC_DIR = src
DRV_DIR = drivers
INC_DIR = inc
BUILD_DIR = build

# Source files
SRCS = \
    $(SRC_DIR)/main.c \
    $(SRC_DIR)/state_machine.c \
    $(SRC_DIR)/serial_protocol.c \
    $(SRC_DIR)/can_protocol.c \
    $(SRC_DIR)/authentication.c \
    $(SRC_DIR)/sensors.c \
    $(SRC_DIR)/ride_control.c \
    $(SRC_DIR)/config_manager.c \
    $(SRC_DIR)/runtime_helpers.c \
    $(DRV_DIR)/hal_drivers.c

# Include paths
INCLUDES = -I$(INC_DIR)

# Compiler flags
CFLAGS = \
    -mcpu=cortex-m3 \
    -mthumb \
    -Os \
    -g \
    -Wall \
    -Wextra \
    -ffunction-sections \
    -fdata-sections \
    -DSTM32F103xB \
    $(INCLUDES)

# Linker flags
LDFLAGS = \
    -mcpu=cortex-m3 \
    -mthumb \
    -Wl,--gc-sections \
    -nostartfiles \
    -T stm32f103_flash.ld

# Object files
OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(notdir $(SRCS)))

# Default target
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin
	$(SIZE) $(BUILD_DIR)/$(TARGET).elf

# Compile
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(DRV_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Link
$(BUILD_DIR)/$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) $^ -o $@

# Binary
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Clean
clean:
	rm -rf $(BUILD_DIR)

# Info
info:
	@echo "Source files:"
	@echo "  $(SRCS)"
	@echo ""
	@echo "Project structure:"
	@echo "  inc/          - Header files (types, address map, HAL)"
	@echo "  src/          - Application source files"
	@echo "  drivers/      - Hardware abstraction layer"
	@echo ""
	@echo "Key files:"
	@echo "  main.c            - Entry point, task scheduler"
	@echo "  state_machine.c   - 8-state scooter state machine"
	@echo "  serial_protocol.c - Ninebot UART protocol"
	@echo "  can_protocol.c    - CAN bus communication"
	@echo "  authentication.c  - BLE lock/unlock authentication"
	@echo "  sensors.c         - ADC, fault detection"
	@echo "  ride_control.c    - Speed, headlight, LED, ride modes"
	@echo "  config_manager.c  - Flash config storage, device ID"
	@echo "  runtime_helpers.c - Soft-float, memcpy, memset"

.PHONY: all clean info
