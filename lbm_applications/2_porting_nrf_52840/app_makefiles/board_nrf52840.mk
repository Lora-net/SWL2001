##############################################################################
# Definitions for the nRF52840 PCA10056 board
##############################################################################
$(info ************  board_nRF52840 LA +++++++++ ************)

SDK_ROOT := mcu_drivers/nRF5_SDK_17.1.0_ddde560_Driver
HAL_NRF_DIR  := smtc_hal_nrf52840

#-----------------------------------------------------------------------------
# Compilation flags
#-----------------------------------------------------------------------------

#MCU compilation flags
MCU_FLAGS ?= -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -falign-functions=16 -fno-strict-aliasing -mhard-float


BOARD_C_DEFS =  \
	-DNRF52840_XXAA\
 	-DBOARD_PCA10056 \
 	-DBSP_DEFINES_ONLY \
 	-DCONFIG_GPIO_AS_PINRESET \
 	-DFLOAT_ABI_HARD \
	-DDEBUG \
	-D__HEAP_SIZE=8192 \
	-D__STACK_SIZE=8192 \

ASFLAGS += \
	-DNRF52840_XXAA \
	-DBOARD_PCA10056 \
	-DBSP_DEFINES_ONLY \
	-DCONFIG_GPIO_AS_PINRESET \
	-DFLOAT_ABI_HARD \
	-D__HEAP_SIZE=8192 \
	-D__STACK_SIZE=8192

LDFLAGS += -L$(SDK_ROOT)/modules/nrfx/mdk
BOARD_LDSCRIPT =  $(HAL_NRF_DIR)/generic_gcc_nrf52.ld \


OUTPUT_DIRECTORY := build_lr1110_PCA10056


# $(OUTPUT_DIRECTORY)/nrf52840_xxaa.out: \
#   LINKER_SCRIPT  := gcc_nrf52.ld

#-----------------------------------------------------------------------------
# Hardware-specific sources
#-----------------------------------------------------------------------------

BOARD_ASM_SOURCES = $(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52840.s

BOARD_C_SOURCES = \
  $(SDK_ROOT)/components/boards/boards.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/atomic/nrf_atomic.c \
  $(SDK_ROOT)/components/libraries/queue/nrf_queue.c \
  $(SDK_ROOT)/components/libraries/ringbuf/nrf_ringbuf.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_uart.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_clock.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_rng.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_spi.c \
  $(SDK_ROOT)/modules/nrfx/soc/nrfx_atomic.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/prs/nrfx_prs.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_uart.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_uarte.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_clock.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_rng.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_rtc.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_gpiote.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_wdt.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_spi.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_nvmc.c \
  $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52840.c \
  $(SDK_ROOT)/modules/nrfx/hal/nrf_nvmc.c \
  $(HAL_NRF_DIR)/smtc_hal_flash.c \
  $(HAL_NRF_DIR)/smtc_hal_gpio.c \
  $(HAL_NRF_DIR)/smtc_hal_mcu.c \
  $(HAL_NRF_DIR)/smtc_hal_rng.c \
  $(HAL_NRF_DIR)/smtc_hal_rtc.c \
  $(HAL_NRF_DIR)/smtc_hal_spi.c \
  $(HAL_NRF_DIR)/smtc_hal_trace.c \
  $(HAL_NRF_DIR)/smtc_hal_uart.c \
  $(HAL_NRF_DIR)/smtc_hal_watchdog.c \
  smtc_modem_hal/smtc_modem_hal.c \

# Include folders common to all targets
BOARD_C_INCLUDES = \
  -I$(SDK_ROOT)/components \
  -I$(SDK_ROOT)/modules/nrfx/mdk \
  -I$(SDK_ROOT)/components/libraries/strerror \
  -I$(SDK_ROOT)/components/toolchain/cmsis/include \
  -I$(SDK_ROOT)/components/libraries/util \
  -I$(SDK_ROOT)/components/libraries/ringbuf \
  -I$(SDK_ROOT)/modules/nrfx/hal \
  -I$(SDK_ROOT)/components/libraries/bsp \
  -I$(SDK_ROOT)/components/libraries/uart \
  -I$(SDK_ROOT)/components/libraries/log \
  -I$(SDK_ROOT)/modules/nrfx \
  -I$(SDK_ROOT)/components/libraries/experimental_section_vars \
  -I$(SDK_ROOT)/integration/nrfx/legacy \
  -I$(SDK_ROOT)/components/libraries/delay \
  -I$(SDK_ROOT)/integration/nrfx \
  -I$(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd \
  -I$(SDK_ROOT)/components/boards \
  -I$(SDK_ROOT)/components/libraries/memobj \
  -I$(SDK_ROOT)/modules/nrfx/drivers/include \
  -I$(SDK_ROOT)/components/libraries/log/src \
  -I$(SDK_ROOT)/components/libraries/atomic \
  -I$(SDK_ROOT)/components/libraries/queue \
  -I$(HAL_NRF_DIR) \



# Libraries common to all targets
LIB_FILES += \

# Optimization flags
# OPT = -O0 -g3
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
# CFLAGS += $(OPT)
# CFLAGS += -DDEBUG
# CFLAGS += -DDEBUG_NRF
# CFLAGS += -DBOARD_PCA10056
# CFLAGS += -DBSP_DEFINES_ONLY
# CFLAGS += -DCONFIG_GPIO_AS_PINRESET
# CFLAGS += -DFLOAT_ABI_HARD
# CFLAGS += -DNRF52840_XXAA
# CFLAGS += -mcpu=cortex-m4
# CFLAGS += -mthumb -mabi=aapcs
# CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# CFLAGS += -Wall -Werror
# keep every function in a separate section, this allows linker to discard unused ones
# CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
# CFLAGS += -fno-builtin -fshort-enums

CFLAGS += -Wno-expansion-to-defined

# C++ flags common to all targets
CXXFLAGS += $(OPT)
# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBOARD_PCA10056
ASMFLAGS += -DBSP_DEFINES_ONLY
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF52840_XXAA

# Linker flags
# LDFLAGS += $(OPT)
# LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_ROOT)/modules/nrfx/mdk -T$(LINKER_SCRIPT)
# LDFLAGS += -mcpu=cortex-m4
# LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# # let linker dump unused sections
# LDFLAGS += -Wl,--gc-sections
# # use newlib in nano version
# LDFLAGS += --specs=nano.specs

nrf52840_xxaa: CFLAGS += -D__HEAP_SIZE=8192
nrf52840_xxaa: CFLAGS += -D__STACK_SIZE=8192
nrf52840_xxaa: ASMFLAGS += -D__HEAP_SIZE=8192
nrf52840_xxaa: ASMFLAGS += -D__STACK_SIZE=8192

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm


# .PHONY: default help

# Default target - first one defined
# default: nrf52840_xxaa

# Print all targets that can be built
# help:
# 	@echo following targets are available:
# 	@echo		nrf52840_xxaa
# 	@echo		sdk_config - starting external tool for editing sdk_config.h
# 	@echo		flash      - flashing binary

# TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc


# include $(TEMPLATE_PATH)/Makefile.common

# $(foreach target, $(TARGETS), $(call define_target, $(target)))

# .PHONY: flash erase

# # Flash the program
flash: default
	@echo Flashing: $(OUTPUT_DIRECTORY)/app_lr1110.hex
	nrfjprog -f nrf52 --program $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex --sectorerase
	nrfjprog -f nrf52 --reset

# erase:
# 	nrfjprog -f nrf52 --eraseall

# SDK_CONFIG_FILE := ../config/sdk_config.h
# CMSIS_CONFIG_TOOL := $(SDK_ROOT)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar
# sdk_config:
# 	java -jar $(CMSIS_CONFIG_TOOL) $(SDK_CONFIG_FILE)
