##############################################################################
# Common rules and definitions
##############################################################################

-include makefiles/options.mk

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
else
AR = $(PREFIX)ar
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

ifeq ($(CRYPTO),LR11XX)
TARGET_MODEM := $(TARGET_MODEM)_lr11xx_crypto
BUILD_DIR_MODEM := $(BUILD_DIR_MODEM)_lr11xx_crypto
endif # LR11XX
ifeq ($(CRYPTO),LR11XX_WITH_CREDENTIALS)
TARGET_MODEM := $(TARGET_MODEM)_lr11xx_crypto_with_cred
BUILD_DIR_MODEM := $(BUILD_DIR_MODEM)_lr11xx_crypto_with_cred
endif # LR11XX_WITH_CREDENTIALS

ifeq ($(MODEM_TRACE), yes)
TARGET_MODEM := $(TARGET_MODEM)_trace
BUILD_DIR_MODEM := $(BUILD_DIR_MODEM)/trace
else
TARGET_MODEM := $(TARGET_MODEM)_notrace
BUILD_DIR_MODEM := $(BUILD_DIR_MODEM)/notrace
endif

#-----------------------------------------------------------------------------
# Debug
#-----------------------------------------------------------------------------
ifeq ($(DEBUG),yes)
OPT += -g
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
	-fno-unroll-loops \
	-ffast-math \
	-ftree-vectorize

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
	-DBUILD_DATE=\"$(BUILD_DATE)\"\
	-DNUMBER_OF_STACKS=$(NB_OF_STACK)

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

ifeq ($(LBM_CLASS_B),yes)
COMMON_C_DEFS += \
	-DADD_CLASS_B
endif
ifeq ($(LBM_CLASS_C),yes)
COMMON_C_DEFS += \
	-DADD_CLASS_C
endif

ifeq ($(LBM_MULTICAST),yes)
COMMON_C_DEFS += \
	-DSMTC_MULTICAST
endif

ifeq ($(LBM_FUOTA),yes)
COMMON_C_DEFS += \
	-DADD_FUOTA \
	-DADD_CLASS_B \
	-DADD_CLASS_C \
	-DADD_SMTC_ALC_SYNC \
	-DSMTC_MULTICAST
    ifneq ($(FUOTA_MAXIMUM_NB_OF_FRAGMENTS),nc)
    COMMON_C_DEFS += \
    	-DFRAG_MAX_NB=$(FUOTA_MAXIMUM_NB_OF_FRAGMENTS)
    endif
    ifneq ($(FUOTA_MAXIMUM_SIZE_OF_FRAGMENTS),nc)
    COMMON_C_DEFS += \
       	-DFRAG_MAX_SIZE=$(FUOTA_MAXIMUM_SIZE_OF_FRAGMENTS)
    endif
    ifneq ($(FUOTA_MAXIMUM_FRAG_REDUNDANCY),nc)
    COMMON_C_DEFS += \
       	-DFRAG_MAX_REDUNDANCY=$(FUOTA_MAXIMUM_FRAG_REDUNDANCY)
    endif
	ifeq ($(LBM_FUOTA_ENABLE_FMP),yes)
    COMMON_C_DEFS += \
        -DENABLE_FUOTA_FMP
	endif
	ifeq ($(LBM_FUOTA_ENABLE_MPA),yes)
    COMMON_C_DEFS += \
        -DENABLE_FUOTA_MPA
	endif
else
    ifeq ($(LBM_ALC_SYNC),yes)
    COMMON_C_DEFS += \
    	-DADD_SMTC_ALC_SYNC
    endif
endif

ifeq ($(ALLOW_CSMA_BUILD),yes)
ifeq ($(LBM_CSMA),yes)
COMMON_C_DEFS += \
	-DADD_CSMA
ifeq ($(USE_CSMA_BY_DEFAULT),yes)
COMMON_C_DEFS += \
	-DENABLE_CSMA_BY_DEFAULT
endif
endif
endif

CFLAGS += $(BOARD_C_DEFS) $(COMMON_C_DEFS) $(MODEM_C_DEFS)
CFLAGS += $(BOARD_C_INCLUDES) $(COMMON_C_INCLUDES) $(MODEM_C_INCLUDES) 
CFLAGS += -fno-builtin $(MCU_FLAGS) $(EXTRAFLAGS) $(OPT) $(WFLAG) -MMD -MP -MF"$(@:%.o=%.d)"
CFLAGS += -falign-functions=4
CFLAGS += -std=c17

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
	smtc_modem_core/smtc_modem.c\
	smtc_modem_core/smtc_modem_test.c\
	smtc_modem_core/modem_utilities/modem_event_utilities.c\
	smtc_modem_core/modem_utilities/fifo_ctrl.c\
	smtc_modem_core/modem_utilities/modem_core.c \
	smtc_modem_core/modem_supervisor/modem_supervisor_light.c\
	smtc_modem_core/lorawan_packages/lorawan_certification.c\
	smtc_modem_core/lorawan_manager/lorawan_join_management.c\
	smtc_modem_core/lorawan_manager/lorawan_send_management.c\
	smtc_modem_core/lorawan_manager/lorawan_cid_request_management.c\
	smtc_modem_core/lorawan_manager/lorawan_dwn_ack_management.c

LR1MAC_C_SOURCES += \
	smtc_modem_core/lr1mac/src/lr1_stack_mac_layer.c\
	smtc_modem_core/lr1mac/src/lr1mac_core.c\
	smtc_modem_core/lr1mac/src/lr1mac_utilities.c\
	smtc_modem_core/lr1mac/src/smtc_real/src/smtc_real.c\
	smtc_modem_core/lr1mac/src/services/smtc_duty_cycle.c\
	smtc_modem_core/lr1mac/src/services/smtc_lbt.c


ifeq ($(LBM_CLASS_C),yes)
LR1MAC_C_SOURCES += \
	smtc_modem_core/lr1mac/src/lr1mac_class_c/lr1mac_class_c.c
endif

ifeq ($(LBM_CLASS_B),yes)
LR1MAC_C_SOURCES += \
	smtc_modem_core/lorawan_packages/lorawan_beacon_tx_service_example.c\
	smtc_modem_core/lorawan_manager/lorawan_class_b_management.c\
	smtc_modem_core/lr1mac/src/lr1mac_class_b/smtc_beacon_sniff.c\
	smtc_modem_core/lr1mac/src/lr1mac_class_b/smtc_ping_slot.c
endif	

ifeq ($(LBM_MULTICAST),yes)
LR1MAC_C_SOURCES += \
	smtc_modem_core/lr1mac/src/services/smtc_multicast/smtc_multicast.c
endif

ifeq ($(LBM_FUOTA),yes)
    ifeq ($(LBM_FUOTA_VERSION),1)
    SMTC_MODEM_CORE_C_SOURCES += \
    	smtc_modem_core/lorawan_packages/fuota_packages/fragmentation_helper.c\
    	smtc_modem_core/lorawan_packages/fuota_packages/lorawan_fragmentation_package.c\
    	smtc_modem_core/lorawan_packages/fuota_packages/lorawan_alcsync.c\
    	smtc_modem_core/lorawan_packages/fuota_packages/lorawan_remote_multicast_setup_package.c
		ifeq ($(LBM_FUOTA_ENABLE_FMP),yes)
        SMTC_MODEM_CORE_C_SOURCES += \
    	    smtc_modem_core/lorawan_packages/fuota_packages/lorawan_fmp_package.c
		endif
		ifeq ($(LBM_FUOTA_ENABLE_MPA),yes)
        SMTC_MODEM_CORE_C_SOURCES += \
    	    lora_basics_modem/smtc_modem_core/lorawan_packages/fuota_packages/lorawan_mpa_package.c
		endif
    endif
    ifeq ($(LBM_FUOTA_VERSION),2)
    SMTC_MODEM_CORE_C_SOURCES += \
    	smtc_modem_core/lorawan_packages/fuota_packages_v2/fragmentation_helper.c\
    	smtc_modem_core/lorawan_packages/fuota_packages_v2/lorawan_fragmentation_package.c\
    	smtc_modem_core/lorawan_packages/fuota_packages_v2/lorawan_alcsync.c\
    	smtc_modem_core/lorawan_packages/fuota_packages_v2/lorawan_remote_multicast_setup_package.c
		ifeq ($(LBM_FUOTA_ENABLE_FMP),yes)
        SMTC_MODEM_CORE_C_SOURCES += \
    	    smtc_modem_core/lorawan_packages/fuota_packages_v2/lorawan_fmp_package.c
		endif
		ifeq ($(LBM_FUOTA_ENABLE_MPA),yes)
        SMTC_MODEM_CORE_C_SOURCES += \
    	    lora_basics_modem/smtc_modem_core/lorawan_packages/fuota_packages_v2/lorawan_mpa_package.c
		endif
    endif
LR1MAC_C_SOURCES += \
	smtc_modem_core/lr1mac/src/lr1mac_class_c/lr1mac_class_c.c\
	smtc_modem_core/lr1mac/src/lr1mac_class_b/smtc_beacon_sniff.c\
	smtc_modem_core/lr1mac/src/lr1mac_class_b/smtc_ping_slot.c \
	smtc_modem_core/lr1mac/src/services/smtc_multicast/smtc_multicast.c
else
    ifeq ($(LBM_ALC_SYNC),yes)
        ifeq ($(LBM_ALC_SYNC_VERSION),1)
        SMTC_MODEM_CORE_C_SOURCES += \
        	smtc_modem_core/lorawan_packages/fuota_packages/lorawan_alcsync.c
        endif
        ifeq ($(LBM_ALC_SYNC_VERSION),2)
        SMTC_MODEM_CORE_C_SOURCES += \
        	smtc_modem_core/lorawan_packages/fuota_packages_v2/lorawan_alcsync.c
        endif
    endif 
endif

ifeq ($(ALLOW_CSMA_BUILD),yes)
ifeq ($(LBM_CSMA),yes)
LR1MAC_C_SOURCES += \
	smtc_modem_core/lr1mac/src/services/smtc_lora_cad_bt.c
endif
endif

SMTC_MODEM_CRYPTO_C_SOURCES += \
	smtc_modem_core/smtc_modem_crypto/smtc_modem_crypto.c

RADIO_PLANNER_C_SOURCES += \
	smtc_modem_core/radio_planner/src/radio_planner.c

COMMON_C_INCLUDES +=  \
	-I.\
	-Ismtc_modem_api\
	-Ismtc_modem_core\
	-Ismtc_modem_core/modem_supervisor\
	-Ismtc_modem_core/modem_utilities\
	-Ismtc_modem_core/lorawan_packages\
	-Ismtc_modem_core/lorawan_manager\
	-Ismtc_modem_core/lorawan_api\
	-Ismtc_modem_core/smtc_ral/src\
	-Ismtc_modem_core/smtc_ralf/src\
	-Ismtc_modem_core/lr1mac\
	-Ismtc_modem_core/lr1mac/src\
	-Ismtc_modem_core/lr1mac/src/services\
	-Ismtc_modem_core/radio_planner/src\
	-Ismtc_modem_core/smtc_modem_crypto\
	-Ismtc_modem_core/smtc_modem_crypto/smtc_secure_element\
	-Ismtc_modem_core/lorawan_api\
	-Ismtc_modem_core/lr1mac/src/smtc_real/src\
	-Ismtc_modem_hal

ifeq ($(LBM_CLASS_B),yes)
COMMON_C_INCLUDES +=  \
	-Ismtc_modem_core/lr1mac/src/lr1mac_class_b
endif

ifeq ($(LBM_CLASS_C),yes)
COMMON_C_INCLUDES +=  \
	-Ismtc_modem_core/lr1mac/src/lr1mac_class_c
endif

ifeq ($(LBM_MULTICAST),yes)
COMMON_C_INCLUDES +=  \
	-Ismtc_modem_core/lr1mac/src/services/smtc_multicast
endif

ifeq ($(LBM_FUOTA),yes)
    ifeq ($(LBM_FUOTA_VERSION),1)
        COMMON_C_INCLUDES +=  \
        	-Ismtc_modem_core/lorawan_packages/fuota_packages
    endif
    ifeq ($(LBM_FUOTA_VERSION),2)
        COMMON_C_INCLUDES +=  \
        	-Ismtc_modem_core/lorawan_packages/fuota_packages_v2
    endif
COMMON_C_INCLUDES +=  \
	-Ismtc_modem_core/lr1mac/src/lr1mac_class_b \
	-Ismtc_modem_core/lr1mac/src/lr1mac_class_c \
	-Ismtc_modem_core/lr1mac/src/services/smtc_multicast 
else
    ifeq ($(LBM_ALC_SYNC),yes)
        ifeq ($(LBM_ALC_SYNC_VERSION),1)
        COMMON_C_INCLUDES +=  \
        	-Ismtc_modem_core/lorawan_packages/fuota_packages
        endif
        ifeq ($(LBM_ALC_SYNC_VERSION),2)
        COMMON_C_INCLUDES +=  \
        	-Ismtc_modem_core/lorawan_packages/fuota_packages_v2
        endif
    endif
endif

#-----------------------------------------------------------------------------
# Gather everything
#-----------------------------------------------------------------------------
C_SOURCES = \
	$(RADIO_DRIVER_C_SOURCES) \
	$(SMTC_RAL_C_SOURCES) \
	$(SMTC_RALF_C_SOURCES) \
	$(RADIO_PLANNER_C_SOURCES) \
	$(SMTC_MODEM_CORE_C_SOURCES) \
	$(SMTC_MODEM_CRYPTO_C_SOURCES) \
	$(LR1MAC_C_SOURCES)

ASM_SOURCES = $(BOARD_ASM_SOURCES)

vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

#-----------------------------------------------------------------------------
.PHONY: basic_modem_build

basic_modem_build: $(BUILD_ROOT)/$(TARGET_MODEM).a
	$(SILENT) cp $< $(BUILD_ROOT)/$(TARGET_ROOT).a
	$(SILENT) rm -rf $(BUILD_ROOT)/latest
	$(SILENT) ln -s $(realpath $(BUILD_DIR_MODEM)) $(BUILD_ROOT)/latest
	$(call success,$@ $<)


#-----------------------------------------------------------------------------
# list of C objects

OBJECTS = $(addprefix $(BUILD_DIR_MODEM)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR_MODEM)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Mark .o intermediate files as secondary target (or precious)
# Without this, $(MAKE) will remove intermediate files and slow down rebuilds
.SECONDARY: $(OBJECTS)

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


$(BUILD_DIR_MODEM)/%.a: $(OBJECTS) Makefile | $(BUILD_DIR_MODEM)
	$(call build,'LIB',$@)
	$(SILENT)$(AR) rcs $@ $(OBJECTS)
	$(SZ) -t $@

$(BUILD_ROOT)/$(TARGET_MODEM).a: $(BUILD_DIR_MODEM)/$(TARGET_MODEM).a
	$(call build,'LIB',$@)
	$(SILENT) cp $< $@

$(BUILD_DIR_MODEM):
	$(SILENT)mkdir -p $@

#-----------------------------------------------------------------------------
# Clean
#-----------------------------------------------------------------------------
clean_target:
	-rm -fR $(BUILD_DIR_MODEM)
	-rm -fR $(BUILD_ROOT)/$(TARGET_MODEM).a
	-rm -fR $(BUILD_ROOT)/$(TARGET_ROOT).a
