# ========================================================================
# Makefile - WFOC Motor Controller Firmware Build System
#
# Target:  ARM Cortex-M0+ (CIU32F003x5) using arm-none-eabi-gcc
# Usage:
#   make              - build everything
#   make HW=WFoc_V1   - select hardware target (default: WFoc_V1)
#   make clean        - remove build artifacts
#   make flash        - flash wfoc.hex via OpenOCD
#   make size         - print section sizes
#
# WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
# Copyright (c) 2026 wflwang
# Licensed under MIT License
# ========================================================================

# ========================================================================
# Toolchain
# ========================================================================
CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size
OPENOCD = openocd

# ========================================================================
# Build Output
# ========================================================================
TARGET  = wfoc
BUILD   = build

# ========================================================================
# Hardware Target Selection
# ========================================================================
# Select board:  make HW=WFoc_V1   (default)
# This defines -DHW_WFoc_V1 which selects board_config.h in hwconf.h
HW      ?= WFoc_V1

# ========================================================================
# Source Directories
# ========================================================================
SRC_DIRS = . driver/mcu motor app conf packet util

# C sources from all directories (wildcard silently skips empty dirs)
C_SOURCES = $(foreach dir,$(SRC_DIRS),$(wildcard $(dir)/*.c))

# Assembly sources
S_SOURCES = startup/startup_ciu32f003x.s

# Linker script
LDSCRIPT  = startup/linker.ld

# ========================================================================
# Include Paths
# ========================================================================
INCLUDES = -I. \
           -Iconfig \
           -Idriver/mcu \
           -Imotor \
           -Iapp \
           -Iconf \
           -Ipacket \
           -Iutil

# ========================================================================
# Compiler Flags
# ========================================================================
MCU_FLAGS = -mcpu=cortex-m0plus -mthumb

CFLAGS  = $(MCU_FLAGS) \
          -Os \
          -ffunction-sections -fdata-sections \
          -std=gnu11 \
          -Wall -Wextra \
          -g \
          -DHW_$(HW) \
          $(INCLUDES)

# Assembly flags (allow C preprocessor on .s files if needed)
ASFLAGS = $(MCU_FLAGS) \
          -x assembler-with-cpp \
          $(INCLUDES)

# ========================================================================
# Linker Flags
# ========================================================================
LDFLAGS = $(MCU_FLAGS) \
          -specs=nano.specs \
          -lc -lnosys \
          -T$(LDSCRIPT) \
          -Wl,--gc-sections \
          -Wl,-Map=$(BUILD)/$(TARGET).map \
          -Wl,--print-memory-usage

# ========================================================================
# Object Files
# ========================================================================
OBJECTS = $(patsubst %.c,$(BUILD)/%.o,$(C_SOURCES)) \
          $(patsubst %.s,$(BUILD)/%.o,$(S_SOURCES))

# ========================================================================
# Output Artifacts
# ========================================================================
ELF = $(BUILD)/$(TARGET).elf
HEX = $(BUILD)/$(TARGET).hex
BIN = $(BUILD)/$(TARGET).bin

# ========================================================================
# Build Rules
# ========================================================================
.PHONY: all clean flash size

all: $(HEX) $(BIN)

# --- Link ---
$(ELF): $(OBJECTS)
	@mkdir -p $(BUILD)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@
	@echo ">>> Linked: $@"

# --- ELF to HEX ---
$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@
	@echo ">>> Generated: $@"

# --- ELF to BIN ---
$(BIN): $(ELF)
	$(OBJCOPY) -O binary -S $< $@
	@echo ">>> Generated: $@"

# --- Compile C source ---
$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@
	@echo ">>> CC: $<"

# --- Assemble .s source ---
$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(ASFLAGS) -c $< -o $@
	@echo ">>> AS: $<"

# ========================================================================
# Utility Targets
# ========================================================================
size: $(ELF)
	$(SIZE) $<

flash: $(HEX)
	$(OPENOCD) -f interface/stlink.cfg -f target/ciu32f003x.cfg \
	    -c "program $(HEX) verify reset exit 0x08000000"

clean:
	@rm -rf $(BUILD)
	@echo ">>> Cleaned build artifacts"

# ========================================================================
# Debug / Info
# ========================================================================
info:
	@echo "HW target:  $(HW)"
	@echo "C sources:  $(C_SOURCES)"
	@echo "S sources:  $(S_SOURCES)"
	@echo "Objects:    $(OBJECTS)"
	@echo "Defines:    -DHW_$(HW)"

.PHONY: info
