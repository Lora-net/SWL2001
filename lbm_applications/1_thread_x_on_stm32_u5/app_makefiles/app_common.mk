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
LD  = $(GCC_PATH)/$(PREFIX)-ld
NM      = $(GCC_PATH)/$(PREFIX)-nm
OBJDUMP = $(GCC_PATH)/$(PREFIX)-objdump
OBJCOPY = $(GCC_PATH)/$(PREFIX)-objcopy
else
CC = $(PREFIX)gcc
CPP = $(PREFIX)g++
AS = $(PREFIX)g++ -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
LD = $(PREFIX)-ld
NM      = $(PREFIX)-nm
OBJDUMP = $(PREFIX)-objdump
OBJCOPY = $(PREFIX)-objcopy
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#-----------------------------------------------------------------------------
# Board selection
#-----------------------------------------------------------------------------



ifeq ($(BOARD),NUCLEO_U575)
-include app_makefiles/board_U575.mk
BOARD_TARGET=u5
# AS defines
AS_DEFS = $(BOARD_AS_DEFS)
endif

#-----------------------------------------------------------------------------
# Define target build directory
#-----------------------------------------------------------------------------
BUILD_TARGET = $(APPTARGET_ROOT)_$(TARGET)
BUILD_DIR = $(APPBUILD_ROOT)_$(TARGET)_$(BOARD_TARGET)

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
# Compilation flags
#-----------------------------------------------------------------------------

# Lora Basics Modem user specific added flags
LBM_FLAGS?=

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

# Assembly flags
ASFLAGS += -fno-builtin $(MCU_FLAGS) $(AS_DEFS) $(AS_INCLUDES) $(APP_OPT) $(WFLAG)

# Common C definitions
COMMON_C_DEFS += 

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

ifeq ($(ALLOW_FUOTA),yes)
COMMON_C_DEFS += \
	-DUSE_FLASH_READ_MODIFY_WRITE\
	-DUSE_FUOTA
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
	main.c

USER_APP_C_SOURCES += \
	main_examples/app_threadx.c\
	main_examples/app_azure_rtos.c\
	main_examples/hw_modem.c\
	main_examples/cmd_parser.c\
	main_examples/main_thread_x.c
COMMON_C_INCLUDES += \
	-Imain_examples


ifneq ($(MODEM_APP),HW_MODEM)
COMMON_C_INCLUDES += \
	-Imain_examples
endif


#-----------------------------------------------------------------------------
# Common sources
#-----------------------------------------------------------------------------

RADIO_HAL_C_SOURCES += \
	radio_hal/radio_utilities.c

COMMON_C_INCLUDES +=  \
	-I.\
	-Iradio_hal\
	-Ismtc_modem_hal\
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

# For debug build, Basic modem objects
#ifneq ($(DEBUG),no)
#BASIC_MODEM_OBJECTS = $(shell find -L $(BASIC_MODEM_BUILD)/latest -name '*.o')
#else
BASIC_MODEM_OBJECTS = $(BASIC_MODEM_LIB)
#endif


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
	$(MAKE) -C $(LORA_BASICS_MODEM) basic_modem RADIO=$(TARGET_RADIO) MODEM_TRACE=$(LBM_TRACE) MCU_FLAGS="$(MCU_FLAGS)"  EXTRAFLAGS="$(LBM_FLAGS)" OPT="$(MODEM_OPT)" CRYPTO=$(CRYPTO) NB_OF_STACK=$(LBM_NB_OF_STACK) $(LBM_BUILD_OPTIONS) VERBOSE=$(VERBOSE) $(MTHREAD_FLAG)

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
