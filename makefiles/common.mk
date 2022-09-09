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
AR  = $(GCC_PATH)/$(PREFIX)ar
CC  = $(GCC_PATH)/$(PREFIX)gcc
CPP = $(GCC_PATH)/$(PREFIX)g++
AS  = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP  = $(GCC_PATH)/$(PREFIX)objcopy
SZ  = $(GCC_PATH)/$(PREFIX)size
GCOV = $(GCC_PATH)/$(PREFIX)gcov
else
AR = $(PREFIX)ar
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

-include makefiles/cortex_m4.mk


#-----------------------------------------------------------------------------
# Define target build directory
#-----------------------------------------------------------------------------
TARGET_MODEM = $(TARGET_ROOT)_$(TARGET)
BUILD_DIR_MODEM = $(BUILD_ROOT)/$(TARGET)

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

-include makefiles/regions.mk

#-----------------------------------------------------------------------------
# Update target name wrt. compilation options
#-----------------------------------------------------------------------------
ifdef REGION
TARGET_MODEM := $(TARGET_MODEM)_$(REGION)
endif

ifeq ($(COVERAGE), RADIO)
BUILD_DIR_MODEM = $(BUILD_ROOT)/$(TARGET)/cov_radio
TARGET_MODEM := $(TARGET_MODEM)_cov_radio
endif
ifeq ($(COVERAGE), MODEM)
BUILD_DIR_MODEM = $(BUILD_ROOT)/$(TARGET)/cov_modem
TARGET_MODEM := $(TARGET_MODEM)_cov_modem
endif

ifeq ($(RADIO),lr1110)
ifeq ($(CRYPTO),LR11XX)
TARGET_MODEM := $(TARGET_MODEM)_lr11xx_crypto
BUILD_DIR_MODEM := $(BUILD_DIR_MODEM)_lr11xx_crypto
endif # LR11XX
ifeq ($(CRYPTO),LR11XX_WITH_CREDENTIALS)
TARGET_MODEM := $(TARGET_MODEM)_lr11xx_crypto_with_cred
BUILD_DIR_MODEM := $(BUILD_DIR_MODEM)_lr11xx_crypto_with_cred
endif # LR11XX_WITH_CREDENTIALS
endif # lr1110

ifeq ($(RADIO),lr1120)
ifeq ($(CRYPTO),LR11XX)
TARGET_MODEM := $(TARGET_MODEM)_lr11xx_crypto
BUILD_DIR_MODEM := $(BUILD_DIR_MODEM)_lr11xx_crypto
endif # LR11XX
ifeq ($(CRYPTO),LR11XX_WITH_CREDENTIALS)
TARGET_MODEM := $(TARGET_MODEM)_lr11xx_crypto_with_cred
BUILD_DIR_MODEM := $(BUILD_DIR_MODEM)_lr11xx_crypto_with_cred
endif # LR11XX_WITH_CREDENTIALS
endif # lr1120

ifeq ($(MIDDLEWARE),yes)
TARGET_MODEM := $(TARGET_MODEM)_middleware
BUILD_DIR_MODEM := $(BUILD_DIR_MODEM)_middleware
endif

ifeq ($(MODEM_TRACE), yes)
TARGET_MODEM := $(TARGET_MODEM)_trace
BUILD_DIR_MODEM := $(BUILD_DIR_MODEM)/trace
else
TARGET_MODEM := $(TARGET_MODEM)_notrace
BUILD_DIR_MODEM := $(BUILD_DIR_MODEM)/notrace
endif

# Clean up commas
COMMA := ,
TARGET_MODEM := $(subst $(COMMA),_,$(TARGET_MODEM))

#-----------------------------------------------------------------------------
# Coverage
#-----------------------------------------------------------------------------
COVERAGE_OUTPUT_DIR = $(BUILD_DIR_MODEM)/gcov

COVERAGE_ARCHIVE = $(BUILD_ROOT)/$(TARGET_MODEM)_gcov.tar.gz

#COVERAGE_PREFIX_STRIP = $(words $(subst /, ,$(realpath $(shell pwd))))
COVERAGE_PREFIX_STRIP = $(words $(subst /, ,$(realpath $(COVERAGE_OUTPUT_DIR))))

# Compilation flags needed for coverage support
COVERAGE_CFLAGS = -fprofile-arcs -ftest-coverage 

# When compiling with coverage we should disable optimizations
ifneq ($(COVERAGE),no) 
#OPT = -O0 -g
DEBUG = yes
COVERAGE_LDFLAGS = -fprofile-arcs
LIBS += -lgcov
else
#OPT = -Os -g
COVERAGE_LDFLAGS = 
endif

#-----------------------------------------------------------------------------
# Debug
#-----------------------------------------------------------------------------
ifeq ($(DEBUG),yes)
OPT = -O0 -ggdb3 -gdwarf
else
OPT = -Os
endif

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
# Bypass LoRaWAN network and use testbench to uplink/downlink frames
#-----------------------------------------------------------------------------
ifeq ($(BYPASS),yes)
BYPASS_FLAGS := '-DLORAWAN_BYPASS_ENABLED'
else
BYPASS_FLAGS :=
endif

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
	-ftree-vectorize \
	$(BYPASS_FLAGS)

# Allow linker to not link unused functions
WFLAG += \
	-ffunction-sections \
	-fdata-sections 

# Generate .su files for stack use analysis
WFLAG += -fstack-usage 

# Change symbols path to please debug tools
CURRENT_DIR := $(shell basename $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST)))))
WFLAG += -ffile-prefix-map==$(CURRENT_DIR)/

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

ifeq ($(MODEM_TRACE),yes)
COMMON_C_DEFS += \
	-DMODEM_HAL_DBG_TRACE=1
ifeq ($(MODEM_DEEP_TRACE),yes)
COMMON_C_DEFS += \
	-DMODEM_HAL_DEEP_DBG_TRACE=1
endif
ifeq ($(MODEM_DEEP_TRACE),no)
COMMON_C_DEFS += \
	-DMODEM_HAL_DEEP_DBG_TRACE=0
endif
endif

ifeq ($(MODEM_TRACE),no)
COMMON_C_DEFS += \
	-DMODEM_HAL_DBG_TRACE=0
endif

ifeq ($(PERF_TEST),yes)
COMMON_C_DEFS += \
	-DPERF_TEST_ENABLED
endif

ifeq ($(MIDDLEWARE),yes)
COMMON_C_DEFS += \
	-DTASK_EXTENDED_1 \
	-DTASK_EXTENDED_2 \
	-DENABLE_FAST_CLOCK_SYNC
endif

ifeq ($(ADD_D2D),yes)
COMMON_C_DEFS += \
	-DSMTC_D2D
endif

ifeq ($(ADD_MULTICAST),yes)
COMMON_C_DEFS += \
	-DSMTC_MULTICAST
endif

ifeq ($(ADD_SMTC_STREAM),yes)
COMMON_C_DEFS += \
	-DADD_SMTC_STREAM
endif

ifeq ($(ADD_SMTC_FILE_UPLOAD),yes)
COMMON_C_DEFS += \
	-DADD_SMTC_FILE_UPLOAD
endif

ifeq ($(ADD_SMTC_ALC_SYNC),yes)
COMMON_C_DEFS += \
	-DADD_SMTC_ALC_SYNC
endif


CFLAGS += -fno-builtin $(MCU_FLAGS) $(BOARD_C_DEFS) $(COMMON_C_DEFS) $(MODEM_C_DEFS) $(BOARD_C_INCLUDES) $(COMMON_C_INCLUDES) $(MODEM_C_INCLUDES) $(OPT) $(WFLAG) -MMD -MP -MF"$(@:%.o=%.d)"
CFLAGS += -falign-functions=4
CFLAGS += -std=c17

ifneq ($(COVERAGE),no) 
CFLAGS += -DCOVERAGE_ENABLED 
endif
#-----------------------------------------------------------------------------
# Link flags
#-----------------------------------------------------------------------------
# libraries
LIBS += -lstdc++ -lsupc++ -lm -lc -lnosys

#-----------------------------------------------------------------------------
# Common sources
#-----------------------------------------------------------------------------
SMTC_MODEM_CORE_C_SOURCES += \
	smtc_modem_core/lorawan_api/lorawan_api.c \
	smtc_modem_core/device_management/dm_downlink.c \
	smtc_modem_core/device_management/modem_context.c\
	smtc_modem_core/modem_core/smtc_modem.c\
	smtc_modem_core/modem_core/smtc_modem_test.c\
	smtc_modem_core/modem_services/fifo_ctrl.c\
	smtc_modem_core/modem_services/modem_utilities.c \
	smtc_modem_core/modem_services/smtc_modem_services_hal.c\
	smtc_modem_core/modem_services/lorawan_certification.c\
	smtc_modem_core/modem_supervisor/modem_supervisor.c

ifeq ($(ADD_SMTC_ALC_SYNC),yes)
SMTC_MODEM_CORE_C_SOURCES += \
	smtc_modem_core/modem_services/smtc_clock_sync.c
endif

ifeq ($(ADD_SMTC_STREAM),yes)
SMTC_MODEM_SERVICES_C_SOURCES += \
	smtc_modem_core/smtc_modem_services/src/stream/stream.c\
	smtc_modem_core/smtc_modem_services/src/stream/rose.c
endif

ifeq ($(ADD_SMTC_FILE_UPLOAD),yes)
SMTC_MODEM_SERVICES_C_SOURCES += \
	smtc_modem_core/smtc_modem_services/src/file_upload/file_upload.c
endif

ifeq ($(ADD_SMTC_ALC_SYNC),yes)
SMTC_MODEM_SERVICES_C_SOURCES += \
	smtc_modem_core/smtc_modem_services/src/alc_sync/alc_sync.c
endif


LR1MAC_C_SOURCES += \
	smtc_modem_core/lr1mac/src/lr1_stack_mac_layer.c\
	smtc_modem_core/lr1mac/src/lr1mac_core.c\
	smtc_modem_core/lr1mac/src/lr1mac_utilities.c\
	smtc_modem_core/lr1mac/src/smtc_real/src/smtc_real.c\
	smtc_modem_core/lr1mac/src/services/smtc_duty_cycle.c\
	smtc_modem_core/lr1mac/src/services/smtc_lbt.c\
	smtc_modem_core/lr1mac/src/lr1mac_class_c/lr1mac_class_c.c\
	smtc_modem_core/lr1mac/src/lr1mac_class_b/smtc_beacon_sniff.c\
	smtc_modem_core/lr1mac/src/lr1mac_class_b/smtc_ping_slot.c

ifeq ($(ADD_D2D),yes)
LR1MAC_C_SOURCES += \
	smtc_modem_core/lr1mac/src/lr1mac_class_b/smtc_d2d.c
endif

ifeq ($(ADD_MULTICAST),yes)
LR1MAC_C_SOURCES += \
	smtc_modem_core/lr1mac/src/services/smtc_multicast.c
endif

SMTC_MODEM_CRYPTO_C_SOURCES += \
	smtc_modem_core/smtc_modem_crypto/smtc_modem_crypto.c

RADIO_PLANNER_C_SOURCES += \
	smtc_modem_core/radio_planner/src/radio_planner.c\
	smtc_modem_core/radio_planner/src/radio_planner_hal.c

COMMON_C_INCLUDES +=  \
	-I.\
	-Ismtc_modem_api\
	-Ismtc_modem_core\
	-Ismtc_modem_core/modem_config\
	-Ismtc_modem_core/modem_core\
	-Ismtc_modem_core/modem_supervisor\
	-Ismtc_modem_core/device_management\
	-Ismtc_modem_core/modem_services\
	-Ismtc_modem_core/lorawan_api\
	-Ismtc_modem_core/smtc_modem_services/headers\
	-Ismtc_modem_core/smtc_modem_services/src\
	-Ismtc_modem_core/smtc_modem_services/src/stream\
	-Ismtc_modem_core/smtc_modem_services/src/file_upload\
	-Ismtc_modem_core/smtc_modem_services/src/alc_sync\
	-Ismtc_modem_core/smtc_modem_services\
	-Ismtc_modem_core/smtc_ral/src\
	-Ismtc_modem_core/smtc_ralf/src\
	-Ismtc_modem_core/lr1mac\
	-Ismtc_modem_core/lr1mac/src\
	-Ismtc_modem_core/lr1mac/src/services\
	-Ismtc_modem_core/lr1mac/src/lr1mac_class_c\
	-Ismtc_modem_core/lr1mac/src/lr1mac_class_b\
	-Ismtc_modem_core/radio_planner/src\
	-Ismtc_modem_core/smtc_modem_crypto\
	-Ismtc_modem_core/smtc_modem_crypto/smtc_secure_element\
	-Ismtc_modem_core/lorawan_api\
	-Ismtc_modem_core/lr1mac/src/smtc_real/src\
	-Ismtc_modem_hal



#-----------------------------------------------------------------------------
# Gather everything
#-----------------------------------------------------------------------------
ifeq ($(COVERAGE),RADIO)
C_SOURCES_COVERAGE = \
	$(RADIO_DRIVER_C_SOURCES) \
	$(SMTC_RAL_C_SOURCES) \
	$(SMTC_RALF_C_SOURCES) \
	$(LR1MAC_C_SOURCES)	\
	$(RADIO_PLANNER_C_SOURCES)

C_SOURCES = \
	$(SMTC_MODEM_CORE_C_SOURCES) \
	$(SMTC_MODEM_SERVICES_C_SOURCES) \
	$(SMTC_MODEM_CRYPTO_C_SOURCES)
endif
ifeq ($(COVERAGE),MODEM)
C_SOURCES_COVERAGE = \
	$(SMTC_MODEM_CORE_C_SOURCES) \
	$(SMTC_MODEM_SERVICES_C_SOURCES) \
	$(SMTC_MODEM_CRYPTO_C_SOURCES)

C_SOURCES = \
	$(RADIO_DRIVER_C_SOURCES) \
	$(SMTC_RAL_C_SOURCES) \
	$(SMTC_RALF_C_SOURCES) \
	$(LR1MAC_C_SOURCES)	\
	$(RADIO_PLANNER_C_SOURCES)
endif

ifeq ($(COVERAGE),no)
C_SOURCES_COVERAGE = 
C_SOURCES = \
	$(RADIO_DRIVER_C_SOURCES) \
	$(SMTC_RAL_C_SOURCES) \
	$(SMTC_RALF_C_SOURCES) \
	$(RADIO_PLANNER_C_SOURCES) \
	$(SMTC_MODEM_CORE_C_SOURCES) \
	$(SMTC_MODEM_SERVICES_C_SOURCES) \
	$(SMTC_MODEM_CRYPTO_C_SOURCES) \
	$(LR1MAC_C_SOURCES)
endif

ASM_SOURCES = $(BOARD_ASM_SOURCES)

vpath %.c $(sort $(dir $(C_SOURCES_COVERAGE)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

#-----------------------------------------------------------------------------
basic_modem:
ifeq ($(RADIO),nc)
	$(call echo_error,"No radio selected! Please specified the target radio  using RADIO=radio_name option")
else
	$(MAKE) basic_modem_build
endif 

.PHONY: basic_modem_build

basic_modem_build: $(BUILD_ROOT)/$(TARGET_MODEM).a
	$(SILENT) cp $< $(BUILD_ROOT)/$(TARGET_ROOT).a
ifneq ($(COVERAGE),no)
	$(MAKE) $(COVERAGE_ARCHIVE)
endif
	$(SILENT) rm -rf $(BUILD_ROOT)/latest
	$(SILENT) ln -s $(realpath $(BUILD_DIR_MODEM)) $(BUILD_ROOT)/latest
	$(call success,$@ $<)


#-----------------------------------------------------------------------------
# list of C objects
COVERAGE_OBJECTS = $(addprefix $(COVERAGE_OUTPUT_DIR)/, $(notdir $(C_SOURCES_COVERAGE:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES_COVERAGE)))

OBJECTS = $(addprefix $(BUILD_DIR_MODEM)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR_MODEM)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Mark .o intermediate files as secondary target (or precious)
# Without this, $(MAKE) will remove intermediate files and slow down rebuilds
.SECONDARY: $(COVERAGE_OBJECTS)
.SECONDARY: $(OBJECTS)

$(COVERAGE_OUTPUT_DIR)/%.o: %.c Makefile | $(COVERAGE_OUTPUT_DIR)
	$(call build,'CC-GCOV',$<)
	$(SILENT)$(CC) -c $(CFLAGS) $(COVERAGE_CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR_MODEM)/$(notdir $(<:.c=.lst)) $< -o $@
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


$(BUILD_DIR_MODEM)/%.a: $(OBJECTS) $(COVERAGE_OBJECTS) Makefile | $(BUILD_DIR_MODEM)
	$(call build,'LIB',$@)
	$(SILENT)$(AR) rcs $@ $(OBJECTS) $(COVERAGE_OBJECTS)
	$(SZ) -t $@

$(BUILD_ROOT)/$(TARGET_MODEM).a: $(BUILD_DIR_MODEM)/$(TARGET_MODEM).a
	$(call build,'LIB',$@)
	$(SILENT) cp $< $@

$(BUILD_DIR_MODEM):
	$(SILENT)mkdir -p $@

$(COVERAGE_ARCHIVE): $(COVERAGE_OBJECTS)
	$(call build,'TAR',$@)
	$(SILENT) tar czf $@ -C $(BUILD_DIR_MODEM) gcov

$(COVERAGE_OUTPUT_DIR):
	$(SILENT)mkdir -p $@

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

debug_sources:
	$(call echo,"RADIO_DRIVER_C_SOURCES	$(RADIO_DRIVER_C_SOURCES)")
	$(call echo,"SMTC_RAL_C_SOURCES	$(SMTC_RAL_C_SOURCES)")
	$(call echo,"SMTC_RALF_C_SOURCES	$(SMTC_RALF_C_SOURCES)")
	$(call echo,"RADIO_HAL_C_SOURCES	$(RADIO_HAL_C_SOURCES)")
	$(call echo,"RADIO_PLANNER_C_SOURCES	$(RADIO_PLANNER_C_SOURCES)")
	$(call echo,"SMTC_MODEM_CORE_C_SOURCES	$(SMTC_MODEM_CORE_C_SOURCES)")
	$(call echo,"SMTC_MODEM_SERVICES_C_SOURCES	$(SMTC_MODEM_SERVICES_C_SOURCES)")
	$(call echo,"SMTC_MODEM_CRYPTO_C_SOURCES	$(SMTC_MODEM_CRYPTO_C_SOURCES)")
	$(call echo,"LR1MAC_C_SOURCES	$(LR1MAC_C_SOURCES)")
	$(call echo,"COMMON_C_INCLUDES	$(COMMON_C_INCLUDES)")

debug_flags:
	$(call echo,"MODEM_C_DEFS	$(MODEM_C_DEFS)")

debug: debug_target debug_region debug_sources debug_flags

#-----------------------------------------------------------------------------
# Clean
#-----------------------------------------------------------------------------
clean_target:
	-rm -fR $(BUILD_DIR_MODEM)
	-rm -fR $(BUILD_ROOT)/$(TARGET_MODEM).a
	-rm -fR $(BUILD_ROOT)/$(TARGET_ROOT).a
	-rm -fR *_gcov.tar.gz
