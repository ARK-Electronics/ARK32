
QUIET = @

# tools
CC = $(ARM_SDK_PREFIX)gcc
OBJCOPY = $(ARM_SDK_PREFIX)objcopy
ECHO = echo

# common variables
IDENTIFIER := AM32

# Folders
HAL_FOLDER := Mcu
MAIN_SRC_DIR := Src
MAIN_INC_DIR := Inc

SRC_DIRS_COMMON := $(MAIN_SRC_DIR)

# Working directories
ROOT := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

# include the rules for OS independence
include $(ROOT)/make/tools.mk

# supported MCU types

MCU_TYPES := E230 F031 F051 F415 F421 G071 L431 G431 V203 G031 A153 SITL

MCU_TYPE := NONE

# Function to include makefile for each MCU type
define INCLUDE_MCU_MAKEFILES
$(foreach MCU_TYPE,$(MCU_TYPES),$(eval include $(call lc,$(MCU_TYPE))makefile.mk))
endef
$(call INCLUDE_MCU_MAKEFILES)

# additional libs
LIBS := -lnosys

# extract version from Inc/version.h
VERSION_MAJOR := $(shell $(FGREP) "define VERSION_MAJOR" $(MAIN_INC_DIR)/version.h | $(CUT) -d" " -f3 )
VERSION_MINOR := $(shell $(FGREP) "define VERSION_MINOR" $(MAIN_INC_DIR)/version.h | $(CUT) -d" " -f3 )
# optional ARK ship patch (artifact names only), e.g. 1 -> 3.0.1-ark
VERSION_PATCH := $(shell $(FGREP) "define VERSION_PATCH" $(MAIN_INC_DIR)/version.h | $(CUT) -d" " -f3 )
# optional fork tag (quoted string in version.h), e.g. ark -> 3.0.1-ark
VERSION_TAG := $(shell $(FGREP) "define VERSION_TAG" $(MAIN_INC_DIR)/version.h | $(CUT) -d\" -f2 )

# Artifact version: MAJOR.MINOR[.PATCH][-TAG]. TAG marks ARK-fork builds vs upstream.
FIRMWARE_VERSION := $(VERSION_MAJOR).$(VERSION_MINOR)$(if $(VERSION_PATCH),.$(VERSION_PATCH))$(if $(VERSION_TAG),-$(VERSION_TAG))

# Compiler options
#
# Global -Os keeps flash/RAM in check on small MCUs (esp. F051). Hot paths
# (RAM_FUNC / selected phase code) force -O3 via attributes or file pragmas —
# a trailing -O3 on the single-shot link line would re-optimize *all* TUs.
# Note: global -O3 was tried for main-loop rate but breaks hold100 free-run
# (armed/input drop near ~97% throttle on ARK F051); keep -Os + selective -O3.

CFLAGS_BASE := -fsingle-precision-constant -fomit-frame-pointer -ffast-math
CFLAGS_BASE += -I$(MAIN_INC_DIR) -g3 -Os -ffunction-sections --specs=nosys.specs
CFLAGS_BASE += -Wall -Wundef -Wextra -Werror -Wno-unused-parameter -Wno-stringop-truncation

CFLAGS_COMMON := $(CFLAGS_BASE)

# Link time optimization, on by default for the cross-compiled firmware.
#
# The firmware is compiled and linked in one gcc invocation, so -flto here covers
# both halves and needs no other build changes. SITL overrides CFLAGS_COMMON and
# is unaffected.
#
# On Cortex-M0 the call overhead is high and the app region is small: LTO recovers
# ~4.1 KiB of the F051's 27 KiB app region. Carrying it as the default rather than
# switching it on for whichever feature needs the space keeps one codegen
# configuration in tree, so what CI and the HWCI bench measure is what ships.
#
# Applies to every cross-compiled MCU (not only F051). scripts/check-codegen-ark.sh
# (make codegen-check-ark / CI) builds ARK_4IN1_F051: it asserts F051 hot
# ISR entry points + known RAM_FUNC callees still
# land in RAM when they remain as symbols, and that externally-read sections
# (.file_name, .app_signature) are non-empty. It does not replace HWCI for
# functional timing risk — global -O3 previously broke hold100 free-run on
# ARK F051; LTO is a different knob but still wants a bench smoke before it is
# trusted as the ship default.
#
# This does interact with the selective -O3 above: RAM_FUNC carries
# optimize("O3") plus noclone, and the noclone is there precisely so LTO
# constant-propagation clones cannot escape .ramfunc.
#
# Escape hatch: LTO=0 builds without it, for bisecting a suspected miscompile
# without editing this file.
LTO ?= 1
ifeq ($(LTO),1)
CFLAGS_COMMON += -flto
endif

# Hardware-CI performance instrumentation (opt-in, off by default).
# Build with `make <TARGET> HWCI_PERF=1` to emit the hwci_perf RAM struct that
# the hardware-CI harness (see hwci/) reads over SWD. Production/release builds
# leave this unset and are completely unaffected.
ifeq ($(HWCI_PERF),1)
CFLAGS_COMMON += -DHWCI_PERF
endif

# Linker options
LDFLAGS_COMMON := -specs=nano.specs $(LIBS) -Wl,--gc-sections -Wl,--print-memory-usage

# Search source files (top-level Src only — not recursive into DroneCAN/)
SRC_COMMON_ALL := $(foreach dir,$(SRC_DIRS_COMMON),$(wildcard $(dir)/*.[cs]))

# Optional translation units: only linked when the product needs them.
# (Empty #ifdef stubs still cost flash/link time on F051.)
SRC_OPTIONAL_BRUSHED := $(MAIN_SRC_DIR)/brushed.c
SRC_OPTIONAL_HWCI    := $(MAIN_SRC_DIR)/hwci_perf.c
SRC_COMMON_BASE := $(filter-out $(SRC_OPTIONAL_BRUSHED) $(SRC_OPTIONAL_HWCI),$(SRC_COMMON_ALL))

# App-side bootloader update. The image is a committed bootloader .bin pulled in
# with .incbin (Src/bl_image.S) rather than a generated C array, so the linked
# bytes stay verifiable against the ARK32-bootloader release they came from
# (see Bootloaders/README.md). .S, not .[cs]: the wildcard above skips it.
# F051 embeds by default (including HWCI_PERF=1); LTO leaves enough flash for
# both the 4 KiB image and the perf struct. Kill switches for size A/Bs:
#   make ARK_4IN1_F051 EMBED_BOOTLOADER=0
#   make ARK_4IN1_F051 NO_EMBED_BL=1
#   make ARK_4IN1_F051 HWCI_PERF=1 NO_EMBED_BL=1
# Bumping the bootloader means dropping the new .bin (from the release .hex)
# and editing this one line if the name changes.
SRC_OPTIONAL_BL_IMAGE := $(MAIN_SRC_DIR)/bl_image.S
# ARK 4IN1: PB4 signal + PA15 nSLEEP low in BL (ARK32-bootloader #4).
# Do not use the generic …_PB4 blob here — first boot would rewrite the
# nSLEEP-off BL back to stock if they differ.
BL_IMAGE_F051 := Bootloaders/AM32_F051_BOOTLOADER_ARK4IN1_V18.bin
# 0x08000000..ORIGIN(FLASH_VECTAB); the F051 linker script asserts the match.
BL_REGION_SIZE_F051 := 4096
# Default on; set EMBED_BOOTLOADER=0 or NO_EMBED_BL=1 to strip the image.
EMBED_BOOTLOADER ?= 1

# configure some directories that are relative to wherever ROOT_DIR is located
OBJ := obj
BIN_DIR := $(ROOT)/$(OBJ)

# Function to check for _CAN / _BRUSHED product suffixes in the target name
has_can_suffix = $(findstring _CAN,$1)
has_brushed_suffix = $(findstring BRUSHED,$1)

# find the SVD files
$(foreach MCU,$(MCU_TYPES),$(eval SVD_$(MCU) := $(wildcard $(HAL_FOLDER_$(MCU))/*.svd)))

.PHONY : clean all binary $(foreach MCU,$(MCU_TYPES),$(call lc,$(MCU)))
# Host-native SITL is opt-in (`make AM32_SITL_CAN` / `make sitl`), not part of
# the cross-compiled `make all` matrix used by Linux firmware CI.
ALL_TARGETS := $(foreach MCU,$(filter-out SITL,$(MCU_TYPES)),$(TARGETS_$(MCU)))
all : $(ALL_TARGETS)

# create targets for compiling one mcu type, eg "make f421"
define CREATE_TARGET
$(call lc,$(1)) : $$(TARGETS_$(1))
endef
$(foreach MCU,$(MCU_TYPES),$(eval $(call CREATE_TARGET,$(MCU))))

clean :
	@echo Removing $(OBJ) directory
	@$(RM) -rf $(OBJ)

#####################
# main firmware build
define CREATE_BUILD_TARGET
$(2)_BASENAME = $(BIN_DIR)/$(IDENTIFIER)_$(2)_$(FIRMWARE_VERSION)

# native (SITL) targets build to an executable elf, no bin/hex conversion
$(2) : $$($(2)_BASENAME).$(if $(NATIVE_$(1)),elf,bin)

# get MCU specific compiler, objcopy and link script or use the ARM SDK one
$(eval xCC := $(if $($(MCU)_CC), $($(MCU)_CC), $(CC)))
$(eval xOBJCOPY := $(if $($(MCU)_OBJCOPY), $($(MCU)_OBJCOPY), $(OBJCOPY)))

# Generate bin and hex files from elf
$$($(2)_BASENAME).bin: $$($(2)_BASENAME).elf
	echo building BIN $$@
	@$(ECHO) Generating $$(notdir $$@)
	$(QUIET)$(xOBJCOPY) -O binary $$(<) $$@
	$(QUIET)python3 Src/DroneCAN/set_app_signature.py $$@ $$(<)
	$(QUIET)$(xOBJCOPY) $$(<) -O ihex $$(@:.bin=.hex)
	$(QUIET)$(CP) -f $$(<) $(OBJ)$(DSEP)debug.elf > $(NUL)

# check for CAN support
$(eval xLDSCRIPT := $$(if $$(call has_can_suffix,$$(2)),$(LDSCRIPT_CAN_$(1)),$(LDSCRIPT_$(1))))
$(eval xCFLAGS := $$(if $$(call has_can_suffix,$$(2)),$(CFLAGS_CAN_$(1))))
$(eval xSRC := $$(if $$(call has_can_suffix,$$(2)),$(SRC_CAN_$(1))))

# Embed the bootloader image on F051 by default (release and HWCI_PERF). Either
# EMBED_BOOTLOADER=0 or NO_EMBED_BL=1 strips it for size emergencies / pure A/Bs.
$(eval xEMBED_BL := $(if $(filter F051,$(1)),$(if $(or $(filter 0,$(EMBED_BOOTLOADER)),$(filter 1,$(NO_EMBED_BL))),,1)))

# Per-target app sources: drop brushed/hwci unless the product asks for them
$(eval SRC_APP_$(2) := $(SRC_COMMON_BASE)$(if $(call has_brushed_suffix,$(2)), $(SRC_OPTIONAL_BRUSHED))$(if $(filter 1,$(HWCI_PERF)), $(SRC_OPTIONAL_HWCI))$(if $(xEMBED_BL), $(SRC_OPTIONAL_BL_IMAGE)))

# allow an MCU type to override the common compiler/linker flags (used by SITL
# for a native build) and to have no linker script
$(eval xCFLAGS_COMMON := $(if $(CFLAGS_COMMON_$(1)),$(CFLAGS_COMMON_$(1)),$(CFLAGS_COMMON)))
$(eval xLDFLAGS_COMMON := $(if $(LDFLAGS_COMMON_$(1)),$(LDFLAGS_COMMON_$(1)),$(LDFLAGS_COMMON)))

# BL_IMAGE_FILE is repo-relative and resolved by the assembler against the cwd,
# which make always sets to the repo root. The 4 KiB image only fits because LTO
# is on by default (see CFLAGS_COMMON above); it does not turn LTO on itself.
CFLAGS_$(2) = -DAM32_MCU=\"$(MCU)\" $(MCU_$(1)) -D$(2) $(CFLAGS_$(1)) $(xCFLAGS_COMMON) $(xCFLAGS) \
	$(if $(xEMBED_BL),-DEMBED_BOOTLOADER -DBL_IMAGE_FILE=\"$(BL_IMAGE_F051)\" -DBL_REGION_SIZE=$(BL_REGION_SIZE_F051))
LDFLAGS_$(2) = $(xLDFLAGS_COMMON) $(LDFLAGS_$(1)) $(if $(xLDSCRIPT),-T$(xLDSCRIPT))

-include $$($(2)_BASENAME).d

# Cross targets require the pinned xPack GCC 15 (see make/tools.mk). SITL is native.
# The bootloader .bin is listed explicitly: .incbin happens in the assembler, so
# it never shows up in the -MMD depfile and swapping the image would otherwise
# not rebuild anything.
$$($(2)_BASENAME).elf: $(if $(NATIVE_$(1)),,arm_sdk_check) $$(SRC_APP_$(2)) $$(SRC_$(1)) $(xSRC) $(if $(xEMBED_BL),$(BL_IMAGE_F051))
	@$(ECHO) Compiling $$(notdir $$@)
	$(QUIET)$(MKDIR) -p $(OBJ)
	$(QUIET)$(xCC) $$(CFLAGS_$(2)) $$(LDFLAGS_$(2)) -MMD -MP -MF $$(@:.elf=.d) -o $$(@) $$(SRC_APP_$(2)) $$(SRC_$(1)) $(xSRC) $(LDLIBS_$(1))
# we copy debug.elf to give us a constant debug target for vscode
# this means the debug button will always debug the last target built
	$(if $(SVD_$(1)),$(QUIET)$(CP) -f $$(SVD_$(1)) $(OBJ)/debug.svd)
# also copy the openocd.cfg from the MCU directory to obj/openocd.cfg for auto config of Cortex-Debug
# in vscode
	$(if $(NATIVE_$(1)),,$(QUIET)$(CP) -f Mcu$(DSEP)$(call lc,$(1))$(DSEP)openocd.cfg $(OBJ)$(DSEP)openocd.cfg > $(NUL))
endef
$(foreach MCU,$(MCU_TYPES),$(foreach TARGET,$(TARGETS_$(MCU)), $(eval $(call CREATE_BUILD_TARGET,$(MCU),$(TARGET)))))

# include the targets for installing tools
include $(ROOT)/make/tools_install.mk

# Fail fast if the pinned xPack Arm GCC is missing or not GCC 15.x.
# Used as a prereq on every cross-compile firmware target.
.PHONY: arm_sdk_check
arm_sdk_check:
	$(QUIET)bash scripts/check-arm-sdk.sh "$(ARM_SDK_PREFIX)" "$(XPACK_GCC_VER)"

# useful target to list all of the board targets so you can see what
# make target to use for your board
targets:
	$(QUIET)echo List of targets. To build a target use 'make TARGETNAME'
	$(QUIET)echo $(ALL_TARGETS)

# Static analysis (cppcheck) of the ARK F051 control path. Fails on
# error/warning; style findings are printed but advisory. See
# scripts/cppcheck-ark.sh and scripts/cppcheck-suppressions.txt.
.PHONY : cppcheck
cppcheck:
	$(QUIET)bash scripts/cppcheck-ark.sh

# Assert the codegen invariants LTO can break silently: F051 hot ISR entry
# points (+ known RAM_FUNC callees when still out-of-line) resident in RAM, and
# externally-read sections (.file_name, .app_signature) not emptied by DCE.
# See scripts/check-codegen-ark.sh.
.PHONY : codegen-check-ark
codegen-check-ark:
	$(QUIET)$(MAKE) -B ARK_4IN1_F051
	$(QUIET)bash scripts/check-codegen-ark.sh $(OBJ)/$(IDENTIFIER)_ARK_4IN1_F051_$(FIRMWARE_VERSION).elf

# Build ARK F051 and enforce flash/RAM headroom (F051 is tight).
# -B forces a rebuild so a prior image is not size-checked by mistake.
.PHONY : size-check-ark
# Worst case is HWCI_PERF=1 with the embedded bootloader (default): it carries
# both the perf struct and the 4 KiB .bl_image. That bounds release flash/RAM.
# A second release-only build still runs so a pure release link regression is
# not hidden by HWCI-only code paths. Strip the image with NO_EMBED_BL=1 /
# EMBED_BOOTLOADER=0 if you need a headroom A/B outside this gate.
size-check-ark:
	$(QUIET)$(ECHO) "--- ARK_4IN1_F051 HWCI_PERF=1 (embedded bootloader, default) ---"
	$(QUIET)$(MAKE) -B ARK_4IN1_F051 HWCI_PERF=1
	$(QUIET)bash scripts/check-size-ark.sh
	$(QUIET)$(ECHO) "--- ARK_4IN1_F051 release (embedded bootloader) ---"
	$(QUIET)$(MAKE) -B ARK_4IN1_F051
	$(QUIET)bash scripts/check-size-ark.sh

# Production full-flash images: bootloader + app + factory EEPROM defaults.
# See factory/README.md. Replaces flash-BL / flash-app / configurator / dump.
.PHONY : factory-image factory-image-f051 factory-image-g431-can factory-image-check
FACTORY_F051_PRODUCT := ARK_4IN1_F051
FACTORY_F051_BASENAME := $(OBJ)/$(IDENTIFIER)_$(FACTORY_F051_PRODUCT)_$(FIRMWARE_VERSION)
FACTORY_F051_DEFAULTS := factory/ARK_4IN1_F051_eeprom_defaults.json
FACTORY_G431_PRODUCT := ARK_G431_CAN
FACTORY_G431_BASENAME := $(OBJ)/$(IDENTIFIER)_$(FACTORY_G431_PRODUCT)_$(FIRMWARE_VERSION)
FACTORY_G431_DEFAULTS := factory/ARK_G431_CAN_eeprom_defaults.json
# Optional: drop a G431 CAN bootloader .bin here to embed it in the full image.
# When missing, the 16 KiB BL region is 0xFF-padded (flash BL separately).
BL_IMAGE_G431_CAN ?= $(firstword $(wildcard Bootloaders/AM32_G431*_CAN*.bin Bootloaders/AM32_G431*CAN*.bin))

factory-image-f051: $(FACTORY_F051_PRODUCT)
	$(QUIET)$(ECHO) "Building factory full-flash image for $(FACTORY_F051_PRODUCT)"
	$(QUIET)python3 scripts/build_factory_image.py \
		--defaults $(FACTORY_F051_DEFAULTS) \
		--bootloader $(BL_IMAGE_F051) \
		--app $(FACTORY_F051_BASENAME).bin \
		--version-h $(MAIN_INC_DIR)/version.h \
		--out-bin $(FACTORY_F051_BASENAME).factory.bin \
		--out-hex $(FACTORY_F051_BASENAME).factory.hex \
		--out-eeprom $(FACTORY_F051_BASENAME).eeprom.bin

factory-image-g431-can: $(FACTORY_G431_PRODUCT)
	$(QUIET)$(ECHO) "Building factory full-flash image for $(FACTORY_G431_PRODUCT)"
	$(QUIET)python3 scripts/build_factory_image.py \
		--defaults $(FACTORY_G431_DEFAULTS) \
		$(if $(BL_IMAGE_G431_CAN),--bootloader $(BL_IMAGE_G431_CAN),--allow-empty-bootloader) \
		--app $(FACTORY_G431_BASENAME).bin \
		--version-h $(MAIN_INC_DIR)/version.h \
		--out-bin $(FACTORY_G431_BASENAME).factory.bin \
		--out-hex $(FACTORY_G431_BASENAME).factory.hex \
		--out-eeprom $(FACTORY_G431_BASENAME).eeprom.bin

# Default target builds both ARK production images.
factory-image: factory-image-f051 factory-image-g431-can

# Build + layout/defaults gate used by CI (.github/workflows/static-analysis.yml).
factory-image-check: factory-image
	$(QUIET)$(ECHO) "--- factory check $(FACTORY_F051_PRODUCT) ---"
	$(QUIET)FACTORY_PRODUCT=$(FACTORY_F051_PRODUCT) \
		FACTORY_DEFAULTS=$(FACTORY_F051_DEFAULTS) \
		BL_IMAGE=$(BL_IMAGE_F051) \
		bash scripts/check-factory-image-ark.sh
	$(QUIET)$(ECHO) "--- factory check $(FACTORY_G431_PRODUCT) ---"
	$(QUIET)FACTORY_PRODUCT=$(FACTORY_G431_PRODUCT) \
		FACTORY_DEFAULTS=$(FACTORY_G431_DEFAULTS) \
		BL_IMAGE="$(BL_IMAGE_G431_CAN)" \
		bash scripts/check-factory-image-ark.sh

# Code formatting (clang-format ≈ PX4 astyle/Linux look; see .clang-format).
# Same target names as PX4:
#   make format          — rewrite sources in place
#   make check_format    — CI: fail if any file would change
#   make format_changed  — rewrite only files changed vs origin/ark-release
.PHONY : format check_format format_changed
format:
	$(QUIET)bash scripts/format.sh

check_format:
	$(QUIET)bash scripts/format.sh --check

format_changed:
	$(QUIET)bash scripts/format.sh --changed

