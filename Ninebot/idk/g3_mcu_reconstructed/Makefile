# =============================================================================
#  Ninebot Max G3 MCU Firmware — Skeleton Makefile
#  Target: STM32F0xx (ARM Cortex-M0)
#
#  This is a REFERENCE Makefile. The reconstructed code is not expected
#  to compile directly — it requires real CMSIS headers, a proper linker
#  script, and the actual lookup table data from the firmware flash dump.
# =============================================================================

# Toolchain
PREFIX   = arm-none-eabi-
CC       = $(PREFIX)gcc
AS       = $(PREFIX)as
LD       = $(PREFIX)ld
OBJCOPY  = $(PREFIX)objcopy
SIZE     = $(PREFIX)size

# Target MCU
MCU      = -mcpu=cortex-m0 -mthumb

# Directories
SRC_DIR  = src
INC_DIR  = inc
MOT_DIR  = motor
APP_DIR  = app
DRV_DIR  = drivers
PRO_DIR  = protocol
CFG_DIR  = config
BUILD    = build

# Compiler flags
CFLAGS   = $(MCU) -std=c11 -Os -g
CFLAGS  += -Wall -Wextra -Wno-unused-parameter
CFLAGS  += -ffunction-sections -fdata-sections
CFLAGS  += -I$(INC_DIR) -I$(CFG_DIR)
CFLAGS  += -DSTM32F072xB   # Adjust for actual chip variant

# Linker flags
LDFLAGS  = $(MCU) -Wl,--gc-sections
LDFLAGS += -T stm32f0.ld   # Linker script (not provided)
LDFLAGS += -nostartfiles

# Source files
SOURCES  = $(SRC_DIR)/main.c \
           $(SRC_DIR)/math_tables.c \
           $(MOT_DIR)/foc.c \
           $(APP_DIR)/ride_ctrl.c \
           $(APP_DIR)/peripherals.c \
           $(PRO_DIR)/protocol.c \
           $(DRV_DIR)/stm32f0_drivers.c

OBJECTS  = $(patsubst %.c,$(BUILD)/%.o,$(SOURCES))

# Output
TARGET   = g3_mcu

# =============================================================================
#  Rules
# =============================================================================

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin
	$(SIZE) $(BUILD)/$(TARGET).elf

$(BUILD)/$(TARGET).elf: $(OBJECTS)
	@mkdir -p $(dir $@)
	$(CC) $(LDFLAGS) -o $@ $^

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -rf $(BUILD)

# Flash with ST-Link (if available)
flash: $(BUILD)/$(TARGET).bin
	st-flash write $< 0x08000000

.PHONY: all clean flash
