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
LBM_TARGET = $(TARGET_ROOT)_$(TARGET)
LBM_BUILD_DIR = $(BUILD_ROOT)/$(TARGET)

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
-include makefiles/relay.mk

#-----------------------------------------------------------------------------
# Update target name wrt. compilation options
#-----------------------------------------------------------------------------

ifeq ($(CRYPTO),LR11XX)
LBM_TARGET := $(LBM_TARGET)_lr11xx_crypto
LBM_BUILD_DIR := $(LBM_BUILD_DIR)_lr11xx_crypto
endif # LR11XX
ifeq ($(CRYPTO),LR11XX_WITH_CREDENTIALS)
LBM_TARGET := $(LBM_TARGET)_lr11xx_crypto_with_cred
LBM_BUILD_DIR := $(LBM_BUILD_DIR)_lr11xx_crypto_with_cred
endif # LR11XX_WITH_CREDENTIALS

ifeq ($(MODEM_TRACE), yes)
LBM_TARGET := $(LBM_TARGET)_trace
LBM_BUILD_DIR := $(LBM_BUILD_DIR)/trace
else
LBM_TARGET := $(LBM_TARGET)_notrace
LBM_BUILD_DIR := $(LBM_BUILD_DIR)/notrace
endif

ifeq ($(LBM_GEOLOCATION),yes)
#In case geolocation is selected, force the selection of store and forward feature
LBM_STORE_AND_FORWARD=yes
endif

#-----------------------------------------------------------------------------
# Debug and optimization
#-----------------------------------------------------------------------------

ifeq ($(DEBUG),yes)
LBM_OPT +=$(DEBUG_OPT) -g
else
LBM_OPT = $(OPT)
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

#Link-time optimization
#WFLAG += --lto

# AS defines
AS_DEFS =

# Assembly flags
ASFLAGS += -fno-builtin $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(LBM_OPT) $(WFLAG)

# Common C definitions
LBM_C_DEFS += \
	-DNUMBER_OF_STACKS=$(NB_OF_STACK)

ifeq ($(MODEM_TRACE),yes)
LBM_C_DEFS += \
	-DMODEM_HAL_DBG_TRACE=1
ifeq ($(MODEM_DEEP_TRACE),yes)
LBM_C_DEFS += \
	-DMODEM_HAL_DEEP_DBG_TRACE=1
endif
ifeq ($(MODEM_DEEP_TRACE),no)
LBM_C_DEFS += \
	-DMODEM_HAL_DEEP_DBG_TRACE=0
endif
endif

ifeq ($(MODEM_TRACE),no)
LBM_C_DEFS += \
	-DMODEM_HAL_DBG_TRACE=0
endif

ifeq ($(PERF_TEST),yes)
LBM_C_DEFS += \
	-DPERF_TEST_ENABLED
endif

ifeq ($(LBM_STREAM),yes)
LBM_C_DEFS += \
    -DADD_SMTC_STREAM
endif

ifeq ($(LBM_LFU),yes)
LBM_C_DEFS += \
    -DADD_SMTC_LFU
endif

ifeq ($(LBM_DEVICE_MANAGEMENT),yes)
LBM_C_DEFS += \
    -DADD_SMTC_CLOUD_DEVICE_MANAGEMENT
endif

ifeq ($(LBM_STORE_AND_FORWARD),yes)
LBM_C_DEFS += \
    -DADD_SMTC_STORE_AND_FORWARD
endif

ifeq ($(LBM_FUOTA),yes)
LBM_C_DEFS += \
	-DADD_FUOTA=$(LBM_FUOTA_VERSION) \
	-DADD_CLASS_B \
	-DADD_CLASS_C \
	-DSMTC_MULTICAST \
	-DADD_SMTC_ALC_SYNC
    ifneq ($(FUOTA_MAXIMUM_NB_OF_FRAGMENTS),nc)
    LBM_C_DEFS += \
    	-DFRAG_MAX_NB=$(FUOTA_MAXIMUM_NB_OF_FRAGMENTS)
    endif
    ifneq ($(FUOTA_MAXIMUM_SIZE_OF_FRAGMENTS),nc)
    LBM_C_DEFS += \
       	-DFRAG_MAX_SIZE=$(FUOTA_MAXIMUM_SIZE_OF_FRAGMENTS)
    endif
    ifneq ($(FUOTA_MAXIMUM_FRAG_REDUNDANCY),nc)
    LBM_C_DEFS += \
       	-DFRAG_MAX_REDUNDANCY=$(FUOTA_MAXIMUM_FRAG_REDUNDANCY)
    endif
	ifeq ($(LBM_FUOTA_ENABLE_FMP),yes)
    LBM_C_DEFS += \
        -DENABLE_FUOTA_FMP
	endif
	ifeq ($(LBM_FUOTA_ENABLE_MPA),yes)
    LBM_C_DEFS += \
        -DENABLE_FUOTA_MPA
	endif
else
    ifeq ($(LBM_ALC_SYNC),yes)
    LBM_C_DEFS += \
    	-DADD_SMTC_ALC_SYNC
    endif
    ifeq ($(LBM_CLASS_B),yes)
    LBM_C_DEFS += \
    	-DADD_CLASS_B
    ifeq ($(LBM_BEACON_TX),yes)
    LBM_C_DEFS += \
    	-DMODEM_BEACON_APP
    endif
    endif
    ifeq ($(LBM_CLASS_C),yes)
    LBM_C_DEFS += \
    	-DADD_CLASS_C
    endif

    ifeq ($(LBM_MULTICAST),yes)
    LBM_C_DEFS += \
    	-DSMTC_MULTICAST
    endif
endif

ifeq ($(ALLOW_CSMA_BUILD),yes)
ifeq ($(LBM_CSMA),yes)
LBM_C_DEFS += \
	-DADD_CSMA
ifeq ($(USE_CSMA_BY_DEFAULT),yes)
LBM_C_DEFS += \
	-DENABLE_CSMA_BY_DEFAULT
endif
endif
endif

ifeq ($(TEST_BYPASS_JOIN_DUTY_CYCLE),yes)
LBM_C_DEFS += \
	-DTEST_BYPASS_JOIN_DUTY_CYCLE
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
	smtc_modem_core/smtc_modem.c\
	smtc_modem_core/smtc_modem_test.c\
	smtc_modem_core/modem_utilities/modem_event_utilities.c\
	smtc_modem_core/modem_utilities/fifo_ctrl.c\
	smtc_modem_core/modem_utilities/modem_core.c \
	smtc_modem_core/modem_supervisor/modem_supervisor_light.c\
	smtc_modem_core/modem_supervisor/modem_tx_protocol_manager.c\
	smtc_modem_core/lorawan_packages/lorawan_certification/lorawan_certification.c\
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

ifeq ($(LBM_FUOTA),yes)

    ifeq ($(LBM_FUOTA_VERSION),1)
    SMTC_MODEM_CORE_C_SOURCES += \
    	smtc_modem_core/lorawan_packages/fragmented_data_block_transport/v1.0.0/fragmentation_helper_v1.0.0.c\
    	smtc_modem_core/lorawan_packages/fragmented_data_block_transport/v1.0.0/lorawan_fragmentation_package_v1.0.0.c\
    	smtc_modem_core/lorawan_packages/application_layer_clock_synchronization/v1.0.0/lorawan_alcsync_v1.0.0.c\
    	smtc_modem_core/lorawan_packages/remote_multicast_setup/v1.0.0/lorawan_remote_multicast_setup_package_v1.0.0.c
    endif

    ifeq ($(LBM_FUOTA_VERSION),2)
    SMTC_MODEM_CORE_C_SOURCES += \
    	smtc_modem_core/lorawan_packages/fragmented_data_block_transport/v2.0.0/fragmentation_helper_v2.0.0.c\
    	smtc_modem_core/lorawan_packages/fragmented_data_block_transport/v2.0.0/lorawan_fragmentation_package_v2.0.0.c\
    	smtc_modem_core/lorawan_packages/application_layer_clock_synchronization/v2.0.0/lorawan_alcsync_v2.0.0.c\
		smtc_modem_core/lorawan_packages/remote_multicast_setup/v2.0.0/lorawan_remote_multicast_setup_package_v2.0.0.c\
		smtc_modem_core/smtc_modem_crypto/soft_secure_element/cmac.c\
		smtc_modem_core/smtc_modem_crypto/soft_secure_element/aes.c
    endif

	ifeq ($(LBM_FUOTA_ENABLE_FMP),yes)
	SMTC_MODEM_CORE_C_SOURCES += \
		smtc_modem_core/lorawan_packages/firmware_management_protocol/lorawan_fmp_package.c
	endif

	ifeq ($(LBM_FUOTA_ENABLE_MPA),yes)
	SMTC_MODEM_CORE_C_SOURCES += \
		smtc_modem_core/lorawan_packages/multi_package_access/lorawan_mpa_package.c
	endif


    LR1MAC_C_SOURCES += \
	smtc_modem_core/lorawan_manager/lorawan_class_b_management.c\
	smtc_modem_core/lr1mac/src/lr1mac_class_b/smtc_beacon_sniff.c\
	smtc_modem_core/lr1mac/src/lr1mac_class_b/smtc_ping_slot.c\
	smtc_modem_core/lr1mac/src/lr1mac_class_c/lr1mac_class_c.c\
	smtc_modem_core/lr1mac/src/services/smtc_multicast/smtc_multicast.c
else
    ifeq ($(LBM_ALC_SYNC),yes)
        ifeq ($(LBM_ALC_SYNC_VERSION),1)
        SMTC_MODEM_CORE_C_SOURCES += \
        	smtc_modem_core/lorawan_packages/application_layer_clock_synchronization/v1.0.0/lorawan_alcsync_v1.0.0.c
        endif
        ifeq ($(LBM_ALC_SYNC_VERSION),2)
        SMTC_MODEM_CORE_C_SOURCES += \
        	smtc_modem_core/lorawan_packages/application_layer_clock_synchronization/v2.0.0/lorawan_alcsync_v2.0.0.c
        endif
    endif

	ifeq ($(LBM_CLASS_B),yes)
    LR1MAC_C_SOURCES += \
		smtc_modem_core/lorawan_manager/lorawan_class_b_management.c\
		smtc_modem_core/lr1mac/src/lr1mac_class_b/smtc_beacon_sniff.c\
		smtc_modem_core/lr1mac/src/lr1mac_class_b/smtc_ping_slot.c
	ifeq ($(LBM_BEACON_TX),yes)
    SMTC_MODEM_CORE_C_SOURCES+=\
		smtc_modem_core/modem_services/beacon_tx_service/lorawan_beacon_tx_service_example.c
	endif
	endif

	ifeq ($(LBM_CLASS_C),yes)
    LR1MAC_C_SOURCES += \
	    smtc_modem_core/lr1mac/src/lr1mac_class_c/lr1mac_class_c.c
    endif

	ifeq ($(LBM_MULTICAST),yes)
	LR1MAC_C_SOURCES += \
		smtc_modem_core/lr1mac/src/services/smtc_multicast/smtc_multicast.c
	endif
endif

ifeq ($(LBM_STREAM),yes)
SMTC_MODEM_CORE_C_SOURCES += \
	smtc_modem_core/modem_services/stream_packages/rose.c \
	smtc_modem_core/modem_services/stream_packages/stream.c
endif

ifeq ($(LBM_LFU),yes)
SMTC_MODEM_CORE_C_SOURCES += \
    smtc_modem_core/modem_services/lfu_service/file_upload.c
endif

ifeq ($(LBM_DEVICE_MANAGEMENT),yes)
SMTC_MODEM_CORE_C_SOURCES += \
	smtc_modem_core/modem_services/cloud_dm_package/cloud_dm_package.c
endif

ifeq ($(LBM_STORE_AND_FORWARD),yes)
SMTC_MODEM_CORE_C_SOURCES += \
	smtc_modem_core/modem_utilities/circularfs.c\
	smtc_modem_core/modem_services/store_and_forward/store_and_forward_flash.c
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

LBM_C_INCLUDES +=  \
	-I.\
	-Ismtc_modem_api\
	-Ismtc_modem_core\
	-Ismtc_modem_core/logging\
	-Ismtc_modem_core/modem_supervisor\
	-Ismtc_modem_core/modem_utilities\
	-Ismtc_modem_core/lorawan_packages\
	-Ismtc_modem_core/lorawan_packages/lorawan_certification\
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


ifeq ($(LBM_FUOTA),yes)
    ifeq ($(LBM_FUOTA_VERSION),1)
        LBM_C_INCLUDES +=  \
        	-Ismtc_modem_core/lorawan_packages/application_layer_clock_synchronization\
        	-Ismtc_modem_core/lorawan_packages/fragmented_data_block_transport\
        	-Ismtc_modem_core/lorawan_packages/fragmented_data_block_transport/v1.0.0\
        	-Ismtc_modem_core/lorawan_packages/remote_multicast_setup
    endif
    ifeq ($(LBM_FUOTA_VERSION),2)
        LBM_C_INCLUDES +=  \
        	-Ismtc_modem_core/lorawan_packages/application_layer_clock_synchronization\
        	-Ismtc_modem_core/lorawan_packages/fragmented_data_block_transport\
        	-Ismtc_modem_core/lorawan_packages/fragmented_data_block_transport/v2.0.0\
        	-Ismtc_modem_core/lorawan_packages/remote_multicast_setup\
        	-Ismtc_modem_core/smtc_modem_crypto/soft_secure_element
    endif
	ifeq ($(LBM_FUOTA_ENABLE_FMP),yes)
        LBM_C_INCLUDES +=  \
        	-Ismtc_modem_core/lorawan_packages/firmware_management_protocol
	endif
	ifeq ($(LBM_FUOTA_ENABLE_MPA),yes)
        LBM_C_INCLUDES +=  \
        	-Ismtc_modem_core/lorawan_packages/multi_package_access
	endif
LBM_C_INCLUDES +=  \
	-Ismtc_modem_core/lr1mac/src/lr1mac_class_b \
	-Ismtc_modem_core/lr1mac/src/lr1mac_class_c \
	-Ismtc_modem_core/lr1mac/src/services/smtc_multicast
else
    ifeq ($(LBM_ALC_SYNC),yes)
        LBM_C_INCLUDES +=  \
        	-Ismtc_modem_core/lorawan_packages/application_layer_clock_synchronization
    endif
    ifeq ($(LBM_CLASS_B),yes)
    LBM_C_INCLUDES +=  \
    	-Ismtc_modem_core/lr1mac/src/lr1mac_class_b
    ifeq ($(LBM_BEACON_TX),yes)
    LBM_C_INCLUDES +=  \
    	-Ismtc_modem_core/modem_services/beacon_tx_service
    endif
    endif

    ifeq ($(LBM_CLASS_C),yes)
    LBM_C_INCLUDES +=  \
    	-Ismtc_modem_core/lr1mac/src/lr1mac_class_c
    endif

    ifeq ($(LBM_MULTICAST),yes)
    LBM_C_INCLUDES +=  \
    	-Ismtc_modem_core/lr1mac/src/services/smtc_multicast
    endif
endif

ifeq ($(LBM_STREAM),yes)
LBM_C_INCLUDES += \
	-Ismtc_modem_core/modem_services \
	-Ismtc_modem_core/modem_services/stream_packages
endif

ifeq ($(LBM_LFU),yes)
LBM_C_INCLUDES += \
	-Ismtc_modem_core/modem_services \
	-Ismtc_modem_core/modem_services/lfu_service
endif

ifeq ($(LBM_DEVICE_MANAGEMENT),yes)
LBM_C_INCLUDES += \
	-Ismtc_modem_core/modem_services \
	-Ismtc_modem_core/modem_services/cloud_dm_package
endif

ifeq ($(LBM_STORE_AND_FORWARD),yes)
LBM_C_INCLUDES += \
	-Ismtc_modem_core/modem_services \
	-Ismtc_modem_core/modem_services/store_and_forward
endif



#-----------------------------------------------------------------------------
# Gather everything
#-----------------------------------------------------------------------------

LBM_CFLAGS += $(LBM_C_DEFS) $(RELAY_C_DEFS)
LBM_CFLAGS += $(LBM_C_INCLUDES) $(RELAY_C_INCLUDES)
LBM_CFLAGS += -fno-builtin $(MCU_FLAGS) $(EXTRAFLAGS) $(LBM_OPT) $(WFLAG) -MMD -MP -MF"$(@:%.o=%.d)"
LBM_CFLAGS += -falign-functions=4
LBM_CFLAGS += -std=c17

LBM_C_SOURCES = \
	$(RADIO_DRIVER_C_SOURCES) \
	$(SMTC_RAL_C_SOURCES) \
	$(SMTC_RALF_C_SOURCES) \
	$(RADIO_PLANNER_C_SOURCES) \
	$(SMTC_MODEM_CORE_C_SOURCES) \
	$(SMTC_MODEM_CRYPTO_C_SOURCES) \
	$(RELAY_C_SOURCES) \
	$(LR1MAC_C_SOURCES)

MODEM_ASM_SOURCES = $(BOARD_ASM_SOURCES)

vpath %.c $(sort $(dir $(LBM_C_SOURCES)))

#-----------------------------------------------------------------------------
.PHONY: basic_modem_build

basic_modem_build: $(BUILD_ROOT)/$(LBM_TARGET).a
	$(SILENT) cp $< $(BUILD_ROOT)/$(TARGET_ROOT).a
	$(SILENT) rm -rf $(BUILD_ROOT)/latest
	$(SILENT) ln -s $(realpath $(LBM_BUILD_DIR)) $(BUILD_ROOT)/latest
	$(call success,$@ $<)


#-----------------------------------------------------------------------------
# list of C objects

OBJECTS = $(addprefix $(LBM_BUILD_DIR)/,$(notdir $(LBM_C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(LBM_C_SOURCES)))

# list of ASM program objects
OBJECTS += $(addprefix $(LBM_BUILD_DIR)/,$(notdir $(MODEM_ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(MODEM_ASM_SOURCES)))

# Mark .o intermediate files as secondary target (or precious)
# Without this, $(MAKE) will remove intermediate files and slow down rebuilds
.SECONDARY: $(OBJECTS)

$(LBM_BUILD_DIR)/%.o: %.c Makefile | $(LBM_BUILD_DIR)
	$(call build,'CC',$<)
	$(SILENT)$(CC) -c $(LBM_CFLAGS) -Wa,-a,-ad,-alms=$(LBM_BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@
ifeq ($(SIZE),yes)
	$(SZ) $@
endif

$(LBM_BUILD_DIR)/%.o: %.s Makefile | $(LBM_BUILD_DIR)
	$(call build,'AS',$<)
	$(SILENT)$(AS) -c $(ASFLAGS) $< -o $@
ifeq ($(SIZE),yes)
	$(SZ) $@
endif


$(LBM_BUILD_DIR)/%.a: $(OBJECTS) Makefile | $(LBM_BUILD_DIR)
	$(call build,'LIB',$@)
	$(SILENT)$(AR) rcs $@ $(OBJECTS)
	$(SZ) -t $@

$(BUILD_ROOT)/$(LBM_TARGET).a: $(LBM_BUILD_DIR)/$(LBM_TARGET).a
	$(call build,'LIB',$@)
	$(SILENT) cp $< $@

$(LBM_BUILD_DIR):
	$(SILENT)mkdir -p $@

#-----------------------------------------------------------------------------
# Clean
#-----------------------------------------------------------------------------
clean_target:
	-rm -fR $(LBM_BUILD_DIR)
	-rm -fR $(BUILD_ROOT)/$(LBM_TARGET).a
	-rm -fR $(BUILD_ROOT)/$(TARGET_ROOT).a
