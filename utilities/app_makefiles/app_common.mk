##############################################################################
# Common rules and definitions
##############################################################################
-include app_makefiles/app_options.mk

#-----------------------------------------------------------------------------
# Build system binaries
#-----------------------------------------------------------------------------
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC  = $(GCC_PATH)/$(PREFIX)gcc
CPP = $(GCC_PATH)/$(PREFIX)g++
AS  = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP  = $(GCC_PATH)/$(PREFIX)objcopy
SZ  = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
CPP = $(PREFIX)g++
AS = $(PREFIX)g++ -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#-----------------------------------------------------------------------------
# Board selection
#-----------------------------------------------------------------------------

ifeq ($(BOARD),NUCLEO_L476)
-include app_makefiles/board_L476.mk
endif

#-----------------------------------------------------------------------------
# Define target build directory
#-----------------------------------------------------------------------------
BUILD_TARGET = $(APPTARGET_ROOT)_$(TARGET)
BUILD_DIR = $(APPBUILD_ROOT)_$(TARGET)

BASIC_MODEM_BUILD = $(LORA_BASICS_MODEM)/build
BASIC_MODEM_LIB = $(BASIC_MODEM_BUILD)/basic_modem.a

#-----------------------------------------------------------------------------
# Multithread is disabled if verbose build, for readable logs
#-----------------------------------------------------------------------------
ifeq ($(VERBOSE),yes)
MULTITHREAD = no
endif
ifeq ($(SIZE),yes)
MULTITHREAD = no
endif

ifeq ($(MULTITHREAD),no)
MTHREAD_FLAG =
else
MTHREAD_FLAG = -j
endif

include $(LORA_BASICS_MODEM)/makefiles/regions.mk

#-----------------------------------------------------------------------------
# Update target name wrt. compilation options
#-----------------------------------------------------------------------------
ifdef REGION
BUILD_TARGET := $(BUILD_TARGET)_$(REGION)
endif

ifeq ($(TARGET_RADIO),lr1110)
ifeq ($(CRYPTO),LR11XX)
BUILD_TARGET := $(BUILD_TARGET)_lr11xx_crypto
BUILD_DIR := $(BUILD_DIR)_lr11xx_crypto
endif # LR11XX
ifeq ($(CRYPTO),LR11XX_WITH_CREDENTIALS)
BUILD_TARGET := $(BUILD_TARGET)_lr11xx_crypto_with_cred
BUILD_DIR := $(BUILD_DIR)_lr11xx_crypto_with_cred
endif # LR11XX_WITH_CREDENTIALS
ifeq ($(USE_LR11XX_CRC_SPI), yes)
BUILD_TARGET := $(BUILD_TARGET)_crc_spi
BUILD_DIR := $(BUILD_DIR)_crc_spi
endif
endif # lr1110

ifeq ($(TARGET_RADIO),lr1120)
ifeq ($(CRYPTO),LR11XX)
BUILD_TARGET := $(BUILD_TARGET)_lr11xx_crypto
BUILD_DIR := $(BUILD_DIR)_lr11xx_crypto
endif # LR11XX
ifeq ($(CRYPTO),LR11XX_WITH_CREDENTIALS)
BUILD_TARGET := $(BUILD_TARGET)_lr11xx_crypto_with_cred
BUILD_DIR := $(BUILD_DIR)_lr11xx_crypto_with_cred
endif # LR11XX_WITH_CREDENTIALS
ifeq ($(USE_LR11XX_CRC_SPI), yes)
BUILD_TARGET := $(BUILD_TARGET)_crc_spi
BUILD_DIR := $(BUILD_DIR)_crc_spi
endif
endif # lr1120

ifeq ($(DEBUG),yes)
BUILD_TARGET := $(BUILD_TARGET)_debug
endif


# Clean up commas
COMMA := ,
BUILD_TARGET := $(subst $(COMMA),_,$(BUILD_TARGET))

#-----------------------------------------------------------------------------
# Debug
#-----------------------------------------------------------------------------
ifeq ($(DEBUG),yes)
APP_OPT += -g
MODEM_OPT += -g
endif

#-----------------------------------------------------------------------------
# Dump memory usage to a log file
#-----------------------------------------------------------------------------
ifeq ($(LOG_MEM), yes)
MEMLOG_FILE := $(BUILD_DIR)/mem_usage.log
MEMLOG = | tee $(MEMLOG_FILE)
else
MEMLOG =
endif

#-----------------------------------------------------------------------------
# Git information
# Thanks to https://nullpointer.io/post/easily-embed-version-information-in-software-releases/
#-----------------------------------------------------------------------------
ifneq (, $(shell which git))
GIT_VERSION := $(shell git --no-pager describe --tags --always)
GIT_COMMIT  := $(shell git rev-parse --verify HEAD)
GIT_DATE    := $(firstword $(shell git --no-pager show --date=iso-strict --format="%ad" --name-only))
BUILD_DATE  := $(shell date --iso=seconds)
SHORT_DATE  := $(shell date +%Y-%m-%d-%H-%M)

# If working tree is dirty, append dirty flag
ifneq ($(strip $(shell git status --porcelain 2>/dev/null)),)
GIT_VERSION := $(GIT_VERSION)--dirty
endif
endif # git

#-----------------------------------------------------------------------------
# Compilation flags
#-----------------------------------------------------------------------------
# Basic compilation flags
WFLAG += \
	-Wall \
	-Wextra \
	-Wno-unused-parameter \
	-Wpedantic \
	-fomit-frame-pointer \
	-fno-unroll-loops \
	-ffast-math \
	-ftree-vectorize

# Allow linker to not link unused functions
WFLAG += \
	-ffunction-sections \
	-fdata-sections

# Generate .su files for stack use analysis
WFLAG += -fstack-usage

#Link-time optimization
#WFLAG += --lto

# AS defines
AS_DEFS =

# Assembly flags
ASFLAGS += -fno-builtin $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(APP_OPT) $(WFLAG)

COMMON_C_DEFS += \
	-DGIT_VERSION=\"$(GIT_VERSION)\" \
	-DGIT_COMMIT=\"$(GIT_COMMIT)\" \
	-DGIT_DATE=\"$(GIT_DATE)\" \
	-DBUILD_DATE=\"$(BUILD_DATE)\"

ifneq ($(MODEM_APP),nc)
COMMON_C_DEFS += \
	-DMAKEFILE_APP=${MODEM_APP}
endif

ifeq ($(MODEM_APP),HW_MODEM)
COMMON_C_DEFS += \
	-DHW_MODEM_ENABLED
endif

ifeq ($(APP_TRACE),yes)
COMMON_C_DEFS += \
	-DHAL_DBG_TRACE=1
endif
ifeq ($(APP_TRACE),no)
COMMON_C_DEFS += \
	-DHAL_DBG_TRACE=0
endif

ifeq ($(PERF_TEST),yes)
COMMON_C_DEFS += \
	-DPERF_TEST_ENABLED
endif

ifeq ($(TEST_BYPASS_JOIN_DUTY_CYCLE),yes)
COMMON_C_DEFS += \
	-DTEST_BYPASS_JOIN_DUTY_CYCLE
endif

ifeq ($(ALLOW_FUOTA),yes)
COMMON_C_DEFS += \
	-DALLOW_FUOTA
LBM_BUILD_OPTIONS += LBM_FUOTA=yes LBM_FUOTA_VERSION=$(FUOTA_VERSION)
endif

ifneq ($(LBM_NB_OF_STACK),1)
COMMON_C_DEFS += \
	-DMULTISTACK
endif


LFS_C_DEFS += -DLFS_CONFIG=lfs_config.h
LFS_C_DEFS += -DLFS_NO_MALLOC
#LFS_C_DEFS += -DLFS_YES_TRACE 		# WARNING there are BIG printf strings that generate HardFaults

CFLAGS += -fno-builtin $(MCU_FLAGS) $(BOARD_C_DEFS) $(COMMON_C_DEFS) $(MODEM_C_DEFS) $(APP_OPT) $(WFLAG) -MMD -MP -MF"$(@:%.o=%.d)"
CFLAGS += -falign-functions=4
CFLAGS += -std=c17

#-----------------------------------------------------------------------------
# Link flags
#-----------------------------------------------------------------------------
# libraries
LIBS += -lstdc++ -lsupc++ -lm -lc -lnosys

LIBDIR =

LDFLAGS += $(MCU_FLAGS)
LDFLAGS += --specs=nano.specs
LDFLAGS += --specs=nosys.specs
LDFLAGS += -T$(BOARD_LDSCRIPT) $(LIBDIR) $(LIBS)
LDFLAGS += -Wl,--cref # Cross-reference table
LDFLAGS += -Wl,--print-memory-usage # Display ram/flash memory usage
LDFLAGS += -Wl,--gc-sections # Garbage collect unused sections


#-----------------------------------------------------------------------------
# User application sources
#-----------------------------------------------------------------------------
USER_APP_C_SOURCES += \
	user_app/main.c \
	user_app/git_version.c

ifeq ($(MODEM_APP),nc)
USER_APP_C_SOURCES += \
	user_app/main_examples/main_exti.c
endif

ifeq ($(MODEM_APP),EXAMPLE_EXTI)
USER_APP_C_SOURCES += \
	user_app/main_examples/main_exti.c
endif

ifeq ($(MODEM_APP),EXAMPLE_PORTING_TESTS)
USER_APP_C_SOURCES += \
	user_app/main_examples/main_porting_tests.c
endif

ifeq ($(MODEM_APP),HW_MODEM)
USER_APP_C_SOURCES += \
	user_app/hw_modem/main_hw_modem.c\
	user_app/hw_modem/hw_modem.c\
	user_app/hw_modem/cmd_parser.c
endif

ifneq ($(MODEM_APP),HW_MODEM)
COMMON_C_INCLUDES += \
	-Iuser_app/main_examples
else
COMMON_C_INCLUDES += \
	-Iuser_app/hw_modem
endif

# For this specific example, a radio access is needed to mimic modem behavior, exceptionnaly include internal folder of lbm
ifeq ($(MODEM_APP),EXAMPLE_PORTING_TESTS)
MODEM_C_INCLUDES += \
	-I$(LORA_BASICS_MODEM)/smtc_modem_core/smtc_ralf/src
endif

#-----------------------------------------------------------------------------
# Common sources
#-----------------------------------------------------------------------------

RADIO_HAL_C_SOURCES += \
	user_app/radio_hal/radio_utilities.c

COMMON_C_INCLUDES +=  \
	-Iuser_app\
	-Iuser_app/radio_hal\
	-I$(LORA_BASICS_MODEM)/smtc_modem_api\
	-I$(LORA_BASICS_MODEM)/smtc_modem_hal


#-----------------------------------------------------------------------------
# Gather everything
#-----------------------------------------------------------------------------

C_INCLUDES += $(BOARD_C_INCLUDES) $(COMMON_C_INCLUDES) $(MODEM_C_INCLUDES) 

C_SOURCES = \
	$(USER_APP_C_SOURCES) \
	$(BOARD_C_SOURCES) \
	$(RADIO_DRIVER_C_SOURCES) \
	$(LITTLEFS_C_SOURCES) \
	$(RADIO_HAL_C_SOURCES)

ASM_SOURCES = $(BOARD_ASM_SOURCES)

vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

#-----------------------------------------------------------------------------

app_build: $(BUILD_DIR)/$(BUILD_TARGET).elf $(BUILD_DIR)/$(BUILD_TARGET).hex $(BUILD_DIR)/$(BUILD_TARGET).bin
	$(call success,$@)

#-----------------------------------------------------------------------------
# list of C objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

BASIC_MODEM_OBJECTS = $(BASIC_MODEM_LIB)

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(call build,'CC',$<)
	$(SILENT)$(CC) -c $(CFLAGS) $(C_INCLUDES) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@
ifeq ($(SIZE),yes)
	$(SZ) $@
endif

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(call build,'AS',$<)
	$(SILENT)$(AS) -c $(ASFLAGS) $< -o $@
ifeq ($(SIZE),yes)
	$(SZ) $@
endif

.PHONY: $(BASIC_MODEM_LIB)
$(BASIC_MODEM_LIB):
	$(MAKE) -C $(LORA_BASICS_MODEM) basic_modem RADIO=$(TARGET_RADIO) MODEM_TRACE=$(LBM_TRACE) MCU_FLAGS="$(MCU_FLAGS)" OPT="$(MODEM_OPT)" CRYPTO=$(CRYPTO) NB_OF_STACK=$(LBM_NB_OF_STACK) $(LBM_BUILD_OPTIONS) VERBOSE=$(VERBOSE) $(MTHREAD_FLAG)

$(BUILD_DIR)/$(BUILD_TARGET).elf: $(OBJECTS) Makefile $(BASIC_MODEM_LIB)
	$(call build,'CC',$@)
	$(SILENT)$(CC) $(OBJECTS) $(BASIC_MODEM_OBJECTS) $(LDFLAGS) -Wl,-Map=$(BUILD_DIR)/$(BUILD_TARGET).map -o $@ $(MEMLOG)
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(call build,'HEX',$@)
	$(SILENT)$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(call build,'BIN',$@)
	$(SILENT)$(BIN) $< $@

$(BUILD_DIR):
	$(SILENT)mkdir $@

#-----------------------------------------------------------------------------
# Clean
#-----------------------------------------------------------------------------
clean_target:
	-rm -fR $(BUILD_DIR)*

clean_modem:
	$(MAKE) -C $(LORA_BASICS_MODEM) clean_$(TARGET_RADIO) CRYPTO=$(CRYPTO) MODEM_TRACE=$(LBM_TRACE)
