##############################################################################
# Common rules and definitions
##############################################################################

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
GCOV = $(GCC_PATH)/$(PREFIX)gcov
else
CC = $(PREFIX)gcc
CPP = $(PREFIX)g++
AS = $(PREFIX)g++ -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
GCOV = $(PREFIX)gcov
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
GCOVR = gcovr

#-----------------------------------------------------------------------------
# Board selection
#-----------------------------------------------------------------------------

include makefiles/board_L476.mk

#-----------------------------------------------------------------------------
# Define target build directory
#-----------------------------------------------------------------------------
TARGET_MODEM = $(APPTARGET_ROOT)_$(TARGET)
BUILD_DIR_MODEM = $(APPBUILD_ROOT)_$(TARGET)

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
TARGET_MODEM := $(TARGET_MODEM)_$(REGION)
endif

ifeq ($(COVERAGE), RADIO)
TARGET_MODEM := $(TARGET_MODEM)_cov_radio
endif
ifeq ($(COVERAGE), MODEM)
TARGET_MODEM := $(TARGET_MODEM)_cov_modem
endif

ifeq ($(RADIO),lr1110)
ifeq ($(CRYPTO),LR11XX)
TARGET_MODEM := $(TARGET_MODEM)_hw_crypto
BUILD_DIR_MODEM := $(BUILD_DIR_MODEM)_hw_crypto
endif # LR11XX
ifeq ($(CRYPTO),LR11XX_WITH_CREDENTIALS)
TARGET_MODEM := $(TARGET_MODEM)_hw_crypto
BUILD_DIR_MODEM := $(BUILD_DIR_MODEM)_hw_crypto
endif # LR11XX_WITH_CREDENTIALS
ifeq ($(USE_LR11XX_CRC_SPI), yes)
TARGET_MODEM := $(TARGET_MODEM)_crc_spi
BUILD_DIR_MODEM := $(BUILD_DIR_MODEM)_crc_spi
endif
endif # lr1110

ifeq ($(RADIO),lr1120)
ifeq ($(CRYPTO),LR11XX)
TARGET_MODEM := $(TARGET_MODEM)_hw_crypto
BUILD_DIR_MODEM := $(BUILD_DIR_MODEM)_hw_crypto
endif # LR11XX
ifeq ($(CRYPTO),LR11XX_WITH_CREDENTIALS)
TARGET_MODEM := $(TARGET_MODEM)_hw_crypto
BUILD_DIR_MODEM := $(BUILD_DIR_MODEM)_hw_crypto
endif # LR11XX_WITH_CREDENTIALS
ifeq ($(USE_LR11XX_CRC_SPI), yes)
TARGET_MODEM := $(TARGET_MODEM)_crc_spi
BUILD_DIR_MODEM := $(BUILD_DIR_MODEM)_crc_spi
endif
endif # lr1120

ifeq ($(DEBUG),yes)
TARGET_MODEM := $(TARGET_MODEM)_debug
endif

# Clean up commas
COMMA := ,
TARGET_MODEM := $(subst $(COMMA),_,$(TARGET_MODEM))

#-----------------------------------------------------------------------------
# Debug
#-----------------------------------------------------------------------------
ifeq ($(DEBUG),yes)
OPT = -O0 -ggdb3 -gdwarf
else
OPT = -Os
endif

export DEBUG

#-----------------------------------------------------------------------------
# Dump memory usage to a log file
#-----------------------------------------------------------------------------
ifeq ($(LOG_MEM), yes)
MEMLOG_FILE := $(BUILD_DIR_MODEM)/mem_usage.log
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

FULL_PATH   :=$(RELEASE_PATH)/${SHORT_DATE}-${GIT_COMMIT}

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
	-mabi=aapcs \
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
ASFLAGS += -fno-builtin $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) $(WFLAG)

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

CFLAGS += -fno-builtin $(MCU) $(BOARD_C_DEFS) $(COMMON_C_DEFS) $(MODEM_C_DEFS) $(BOARD_C_INCLUDES) $(COMMON_C_INCLUDES) $(MODEM_C_INCLUDES) $(OPT) $(WFLAG) -MMD -MP -MF"$(@:%.o=%.d)"
CFLAGS += -falign-functions=4
CFLAGS += -std=c17

#-----------------------------------------------------------------------------
# Link flags
#-----------------------------------------------------------------------------
# libraries
LIBS += -lstdc++ -lsupc++ -lm -lc -lnosys

LIBDIR =

LDFLAGS += $(MCU) 
LDFLAGS += --specs=nano.specs 
LDFLAGS += --specs=nosys.specs
LDFLAGS += -T$(BOARD_LDSCRIPT) $(LIBDIR) $(LIBS) $(COVERAGE_LDFLAGS)
LDFLAGS += -Wl,--cref # Cross-reference table
LDFLAGS += -Wl,--print-memory-usage # Display ram/flash memory usage
LDFLAGS += -Wl,--gc-sections # Garbage collect unused sections


#-----------------------------------------------------------------------------
# User application sources
#-----------------------------------------------------------------------------
USER_APP_C_SOURCES += \
	user_app/main.c \
	user_app/git_version.c

ifeq ($(MODEM_APP),EXAMPLE_EXTI)
USER_APP_C_SOURCES += \
	user_app/main_examples/main_exti.c
endif

ifeq ($(MODEM_APP),nc)
USER_APP_C_SOURCES += \
	user_app/main_examples/main_exti.c
endif

ifeq ($(MODEM_APP),EXAMPLE_TX_BEACON)
USER_APP_C_SOURCES += \
	user_app/main_examples/main_tx_beacon.c
endif

ifeq ($(MODEM_APP),EXAMPLE_LR_FHSS)
USER_APP_C_SOURCES += \
	user_app/main_examples/main_lr_fhss.c
endif

COMMON_C_INCLUDES += \
	-Iuser_app/main_examples

#-----------------------------------------------------------------------------
# LittleFS, used by coverage
#-----------------------------------------------------------------------------
LITTLEFS_C_SOURCES += \
	user_app/littlefs/lfs.c \
	user_app/littlefs/lfs_util.c

#-----------------------------------------------------------------------------
# Common sources
#-----------------------------------------------------------------------------
COMMON_C_INCLUDES +=  \
	-Iuser_app\
	-Iuser_app/radio_hal\
	-I$(LORA_BASICS_MODEM)/smtc_modem_api\
	-I$(LORA_BASICS_MODEM)/smtc_modem_hal

#-----------------------------------------------------------------------------
# Region sources and defines
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# Gather everything
#-----------------------------------------------------------------------------
C_SOURCES = \
	$(USER_APP_C_SOURCES) \
	$(BOARD_C_SOURCES) \
	$(RADIO_DRIVER_C_SOURCES) \
	$(RADIO_HAL_C_SOURCES) 

ASM_SOURCES = $(BOARD_ASM_SOURCES)

vpath %.c $(sort $(dir $(C_SOURCES)))

#-----------------------------------------------------------------------------
example:
ifeq ($(RADIO),nc)
	$(call echo_error,"No radio selected! Please specified the target radio using RADIO=sx128x or RADIO=sx1261 or RADIO=sx1262 or RADIO=lr1110 or RADIO=lr1120")
else
	$(MAKE) example_build
endif 


example_build: $(BUILD_DIR_MODEM)/$(TARGET_MODEM).elf $(BUILD_DIR_MODEM)/$(TARGET_MODEM).hex $(BUILD_DIR_MODEM)/$(TARGET_MODEM).bin
	$(call success,$@)


#-----------------------------------------------------------------------------
OBJECTS = $(addprefix $(BUILD_DIR_MODEM)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR_MODEM)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# For debug build, Basic modem objects
#ifneq ($(DEBUG),no)
#BASIC_MODEM_OBJECTS = $(shell find -L $(BASIC_MODEM_BUILD)/latest -name '*.o')
#else
BASIC_MODEM_OBJECTS = $(BASIC_MODEM_LIB)
#endif


$(BUILD_DIR_MODEM)/gcov/%.o: %.c Makefile | $(BUILD_DIR_MODEM)/gcov
	$(call build,'CC-GCOV',$<)
	$(SILENT)$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR_MODEM)/$(notdir $(<:.c=.lst)) $< -o $@
ifeq ($(SIZE),yes)
	$(SZ) $@
endif

$(BUILD_DIR_MODEM)/%.o: %.c Makefile | $(BUILD_DIR_MODEM)
	$(call build,'CC',$<)
	$(SILENT)$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR_MODEM)/$(notdir $(<:.c=.lst)) $< -o $@
ifeq ($(SIZE),yes)
	$(SZ) $@
endif

$(BUILD_DIR_MODEM)/%.o: %.s Makefile | $(BUILD_DIR_MODEM)
	$(call build,'AS',$<)
	$(SILENT)$(AS) -c $(ASFLAGS) $< -o $@
ifeq ($(SIZE),yes)
	$(SZ) $@
endif

.PHONY: $(BASIC_MODEM_LIB)
$(BASIC_MODEM_LIB):
	$(MAKE) -C $(LORA_BASICS_MODEM) basic_modem $(MTHREAD_FLAG)

$(BUILD_DIR_MODEM)/$(TARGET_MODEM).elf: $(OBJECTS) Makefile $(BASIC_MODEM_LIB)
	$(call build,'CC',$@)
	$(SILENT)$(CC) $(OBJECTS) $(BASIC_MODEM_OBJECTS) $(LDFLAGS) -Wl,-Map=$(BUILD_DIR_MODEM)/$(TARGET_MODEM).map -o $@ $(MEMLOG)
	$(SZ) $@

$(BUILD_DIR_MODEM)/%.hex: $(BUILD_DIR_MODEM)/%.elf | $(BUILD_DIR_MODEM)
	$(call build,'HEX',$@)
	$(SILENT)$(HEX) $< $@

$(BUILD_DIR_MODEM)/%.bin: $(BUILD_DIR_MODEM)/%.elf | $(BUILD_DIR_MODEM)
	$(call build,'BIN',$@)
	$(SILENT)$(BIN) $< $@

$(BUILD_DIR_MODEM)/gcov:
	$(SILENT)mkdir -p $@

$(BUILD_DIR_MODEM):
	$(SILENT)mkdir $@

$(COVERAGE_OUTPUT_DIR):
	$(SILENT)mkdir $@

#-----------------------------------------------------------------------------
# Debug print rules
#-----------------------------------------------------------------------------
debug_region:
	$(call echo,"Region $(REGION)")
	$(call echo,"	REGION_AS_923 	$(REGION_AS_923)")
	$(call echo,"	REGION_AU_915 	$(REGION_AU_915)")
	$(call echo,"	REGION_CN_470 	$(REGION_CN_470)")
	$(call echo,"	REGION_CN_470_RP_1_0 	$(REGION_CN_470_RP_1_0)")
	$(call echo,"	REGION_EU_868 	$(REGION_EU_868)")
	$(call echo,"	REGION_IN_865 	$(REGION_IN_865)")
	$(call echo,"	REGION_KR_920 	$(REGION_KR_920)")
	$(call echo,"	REGION_RU_864 	$(REGION_RU_864)")
	$(call echo,"	REGION_US_915 	$(REGION_US_915)")

debug_target:
	$(call echo,"Target $(TARGET)")
	$(call echo,"Build directory $(BUILD_DIR_MODEM)")
	$(call echo,"Binary $(TARGET_MODEM)")
	$(call echo,"Basic modem build $(BASIC_MODEM_BUILD)")
	$(call echo,"Basic modem lib $(BASIC_MODEM_LIB)")
	$(call echo,"Coverage Prefix Strip $(COVERAGE_PREFIX_STRIP)")

debug_sources:
	$(call echo,"USER_APP_C_SOURCES	$(USER_APP_C_SOURCES)")
	$(call echo,"BOARD_C_SOURCES	$(BOARD_C_SOURCES)")
	$(call echo,"RADIO_DRIVER_C_SOURCES	$(RADIO_DRIVER_C_SOURCES)")
	$(call echo,"SMTC_RAL_C_SOURCES	$(SMTC_RAL_C_SOURCES)")
	$(call echo,"SMTC_RALF_C_SOURCES	$(SMTC_RALF_C_SOURCES)")
	$(call echo,"LITTLEFS_C_SOURCES	$(LITTLEFS_C_SOURCES)")
	$(call echo,"RADIO_HAL_C_SOURCES	$(RADIO_HAL_C_SOURCES)")
	$(call echo,"RADIO_PLANNER_C_SOURCES	$(RADIO_PLANNER_C_SOURCES)")
	$(call echo,"SMTC_MODEM_CORE_C_SOURCES	$(SMTC_MODEM_CORE_C_SOURCES)")
	$(call echo,"SMTC_MODEM_SERVICES_C_SOURCES	$(SMTC_MODEM_SERVICES_C_SOURCES)")
	$(call echo,"SMTC_MODEM_CRYPTO_C_SOURCES	$(SMTC_MODEM_CRYPTO_C_SOURCES)")
	$(call echo,"LR1MAC_C_SOURCES	$(LR1MAC_C_SOURCES)")
	$(call echo,"COMMON_C_INCLUDES	$(COMMON_C_INCLUDES)")

debug_flags:
	$(call echo,"MODEM_C_DEFS	$(MODEM_C_DEFS)")

debug_params:
	$(call echo,"TARGET	$(TARGET)")
	$(call echo,"DEBUG	$(DEBUG)")
	$(call echo,"COVERAGE	$(COVERAGE)")
	$(call echo,"DRIVE	$(DRIVE)")
	$(call echo,"RADIO	$(RADIO)")
	$(call echo,"MODEM_APP	$(MODEM_APP)")
	$(call echo,"MODEM_TRACE	$(MODEM_TRACE)")
	$(call echo,"APP_TRACE	$(APP_TRACE)")
	$(call echo,"OPT	$(OPT)")

debug_objects:
	$(call echo,"OBJECTS	$(OBJECTS)")
	$(call echo,"BASIC_MODEM_OBJECTS	$(BASIC_MODEM_OBJECTS)")

debug: debug_target debug_region debug_sources debug_flags debug_params debug_objects

#-----------------------------------------------------------------------------
# Flash by copying on ST-Link mounted on WSL
#-----------------------------------------------------------------------------
flash: 
ifneq ($(DRIVE),nc) 
ifneq ($(RADIO),nc)
ifneq ($(shell cd /mnt/${DRIVE} && echo -n yes),yes)
	sudo mkdir -p /mnt/${DRIVE}
endif
	$(${DRIVE}/DETAILS.TXT):
		sudo mount -t drvfs ${DRIVE}: /mnt/${DRIVE}
	cp $(BUILD_DIR_MODEM)/$(TARGET_MODEM).bin  /mnt/${DRIVE}/ ; sudo umount /mnt/${DRIVE}
else	
	$(call warn,"No radio selected! Please specified the target radio using RADIO=sx128x or RADIO=sx1261 or RADIO=sx1262 or RADIO=lr1110 or RADIO=lr1120")
endif
else
	$(call echo_error,"No Drive letter specified: please use compiling option: DRIVE=your_drive_letter")
	$(call echo_error,"Example: make flash_sx128x DRIVE=g")
endif


#-----------------------------------------------------------------------------
# Clean
#-----------------------------------------------------------------------------
clean_target:
	-rm -fR $(BUILD_DIR_MODEM)*
