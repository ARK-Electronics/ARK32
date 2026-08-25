# In-tree G431 CAN bootloader for the ARK 12S CAN ESC (ARK_G431_CAN).
# Built as AM32_G431_BOOTLOADER_ARKG4_CAN_V18 and installed into ../Bootloaders/.
#
# HAL/CMSIS come from the app tree (../Mcu/g431/Drivers). Local Mcu/g431 holds
# only bootloader-specific files (blutil, eeprom, startup, fault stubs).

QUIET ?= @

ROOT := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
REPO := $(ROOT)/..

include $(REPO)/make/tools.mk
# tools.mk paths are repo-relative; this Makefile is invoked from bootloader/.
ifeq ($(filter /%,$(ARM_SDK_PREFIX)),)
ARM_SDK_PREFIX := $(REPO)/$(ARM_SDK_PREFIX)
endif

IDENTIFIER := AM32
MCU := G431
PART := STM32G431xx
BOARD := ARK_G431_CAN
PIN := PB4
TAG := ARKG4
BOOTLOADER_VERSION := $(shell $(FGREP) "define BOOTLOADER_VERSION" $(ROOT)/Inc/version.h | $(CUT) -d" " -f3)

CC := $(ARM_SDK_PREFIX)gcc
OBJCOPY := $(ARM_SDK_PREFIX)objcopy

HAL := $(REPO)/Mcu/g431
BL_MCU := $(ROOT)/Mcu/g431

OBJ := $(ROOT)/obj
BASENAME := $(IDENTIFIER)_$(MCU)_BOOTLOADER_$(TAG)_CAN_V$(BOOTLOADER_VERSION)
ELF := $(OBJ)/$(BASENAME).elf
BIN := $(OBJ)/$(BASENAME).bin
HEX := $(OBJ)/$(BASENAME).hex
MAP := $(OBJ)/$(BASENAME).map
DEP := $(OBJ)/$(BASENAME).d
INSTALL_BIN := $(REPO)/Bootloaders/$(BASENAME).bin

SRC_HAL := \
	$(HAL)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_gpio.c \
	$(HAL)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_rcc.c \
	$(HAL)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_utils.c \
	$(HAL)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_pwr.c \
	$(HAL)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_tim.c \
	$(HAL)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_exti.c
SRC_MCU := $(wildcard $(BL_MCU)/Src/*.c) $(wildcard $(BL_MCU)/Startup/*.s)
SRC_BL := $(wildcard $(ROOT)/bootloader/*.[cs])
DSDL_SRC := $(addprefix $(ROOT)/bootloader/DroneCAN/dsdl_generated/src/, \
	uavcan.equipment.esc.RawCommand.c \
	uavcan.protocol.debug.LogLevel.c \
	uavcan.protocol.debug.LogMessage.c \
	uavcan.protocol.dynamic_node_id.Allocation.c \
	uavcan.protocol.file.BeginFirmwareUpdate_req.c \
	uavcan.protocol.file.BeginFirmwareUpdate_res.c \
	uavcan.protocol.file.Error.c \
	uavcan.protocol.file.Path.c \
	uavcan.protocol.file.Read_req.c \
	uavcan.protocol.file.Read_res.c \
	uavcan.protocol.GetNodeInfo_req.c \
	uavcan.protocol.GetNodeInfo_res.c \
	uavcan.protocol.HardwareVersion.c \
	uavcan.protocol.NodeStatus.c \
	uavcan.protocol.param.Empty.c \
	uavcan.protocol.param.ExecuteOpcode_req.c \
	uavcan.protocol.param.ExecuteOpcode_res.c \
	uavcan.protocol.param.GetSet_req.c \
	uavcan.protocol.param.GetSet_res.c \
	uavcan.protocol.param.NumericValue.c \
	uavcan.protocol.param.Value.c \
	uavcan.protocol.SoftwareVersion.c)
SRC_CAN := $(wildcard $(ROOT)/bootloader/DroneCAN/*.[cs]) \
	$(DSDL_SRC) \
	$(wildcard $(ROOT)/bootloader/DroneCAN/libcanard/*.c)
SRC := $(SRC_HAL) $(SRC_MCU) $(SRC_BL) $(SRC_CAN)

CFLAGS := \
	-mcpu=cortex-m4 -mthumb \
	-fsingle-precision-constant -fomit-frame-pointer -ffast-math --specs=nosys.specs \
	-g3 -Os -ffunction-sections -fdata-sections -funsigned-char \
	-flto \
	-Wall -Wextra -Wundef -Werror -Wno-unused-parameter \
	-include $(ROOT)/Inc/targets.h \
	-I$(ROOT)/Inc \
	-I$(BL_MCU)/Inc \
	-I$(HAL)/Drivers/STM32G4xx_HAL_Driver/Inc \
	-I$(HAL)/Drivers/CMSIS/Include \
	-I$(HAL)/Drivers/CMSIS/Device/ST/STM32G4xx/Include \
	-I$(ROOT)/bootloader/DroneCAN \
	-I$(ROOT)/bootloader/DroneCAN/libcanard \
	-I$(ROOT)/bootloader/DroneCAN/dsdl_generated/include \
	-D$(PART) -DMCU_$(MCU) -DAM32_MCU=\"$(MCU)\" \
	-DHSE_VALUE=8000000 \
	-DUSE_FULL_LL_DRIVER -DPREFETCH_ENABLE=1 \
	-DBOOTLOADER -D$(BOARD) \
	-DDRONECAN_SUPPORT=1 -DBOARD_FLASH_SIZE=128 \
	-DCANARD_ENABLE_CANFD=1

LDFLAGS := \
	-specs=nano.specs -lnosys \
	-Wl,--gc-sections -Wl,--print-memory-usage -flto \
	-T$(ROOT)/bootloader/ldscript_bl_CAN.ld \
	-Wl,-Map=$(MAP)

.PHONY: all install clean AM32_G431_BOOTLOADER_ARKG4_CAN

all: install

AM32_G431_BOOTLOADER_ARKG4_CAN: install

$(OBJ):
	$(QUIET)$(MKDIR) -p $(OBJ)

$(ELF): $(SRC) | $(OBJ)
	$(QUIET)echo building $(BASENAME)
	$(QUIET)$(CC) $(CFLAGS) $(LDFLAGS) -MMD -MP -MF $(DEP) -o $@ $(SRC)

$(BIN): $(ELF)
	$(QUIET)$(OBJCOPY) -O binary $< $@
	$(QUIET)$(OBJCOPY) $< -O ihex $(HEX)

$(INSTALL_BIN): $(BIN)
	$(QUIET)echo installing $(notdir $@)
	$(QUIET)$(CP) -f $< $@

install: $(INSTALL_BIN)

clean:
	$(QUIET)$(RM) -rf $(OBJ)

-include $(DEP)
