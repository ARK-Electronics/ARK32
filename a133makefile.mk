MCU := A133
PART := MCXA133

MCU_LC := $(call lc,$(MCU))

TARGETS_$(MCU) := $(call get_targets,$(MCU))

HAL_FOLDER_$(MCU) := $(HAL_FOLDER)/$(MCU_LC)

MCU_$(MCU) := -mcpu=cortex-m33+nodsp -mthumb
LDSCRIPT_$(MCU) := $(wildcard $(HAL_FOLDER_$(MCU))/*.ld)

SRC_BASE_DIR_$(MCU) := \
	$(HAL_FOLDER_$(MCU))/Startup \
	$(HAL_FOLDER_$(MCU))/Drivers/src

SRC_DIR_$(MCU) := $(SRC_BASE_DIR_$(MCU)) \
	$(HAL_FOLDER_$(MCU))/Src

CFLAGS_$(MCU) := \
	-I$(HAL_FOLDER_$(MCU))/Inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/inc \
	-I$(HAL_FOLDER_$(MCU))/CMSIS

CFLAGS_$(MCU) += \
	 -D$(PART) \
	 -DCPU_MCXA133VFM
# MCXA133 ships only in VFM / VFT (48-pin) packages. The CPU macro only
# selects the fsl_device_registers.h series guard: VFM and VFT both map
# to MCXA133_SERIES, so the choice is cosmetic for this HAL. Any other
# CPU_MCXA133* macro #errors.

SRC_$(MCU) := $(foreach dir,$(SRC_DIR_$(MCU)),$(wildcard $(dir)/*.[cs]))
