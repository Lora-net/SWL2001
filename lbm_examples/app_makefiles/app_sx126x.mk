##############################################################################
# Definitions for the SX126x tranceiver
##############################################################################
-include app_makefiles/app_options.mk

ifeq ($(TARGET_RADIO),sx1261)
TARGET = sx1261
endif
ifeq ($(TARGET_RADIO),sx1262)
TARGET = sx1262
endif
ifeq ($(TARGET_RADIO),sx1268)
TARGET = sx1268
endif

#-----------------------------------------------------------------------------
# Common sources
#-----------------------------------------------------------------------------

RADIO_HAL_C_SOURCES += \
	radio_hal/sx126x_hal.c \
	radio_hal/ral_sx126x_bsp.c

#-----------------------------------------------------------------------------
# Includes
#-----------------------------------------------------------------------------
# This is needed for the following:
# - sx126x_hal.h
# - ral_sx126x_bsp.h
MODEM_C_INCLUDES =  \
	-I$(LORA_BASICS_MODEM)/smtc_modem_core/radio_drivers/sx126x_driver/src\
	-I$(LORA_BASICS_MODEM)/smtc_modem_core/smtc_ral/src

#-----------------------------------------------------------------------------
# Region
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# Radio specific compilation flags
#-----------------------------------------------------------------------------
COMMON_C_DEFS += \
	-DSX126X 

ifeq ($(TARGET_RADIO),sx1262)
COMMON_C_DEFS += \
    -DSX1262
endif

ifeq ($(TARGET_RADIO),sx1268)
COMMON_C_DEFS += \
    -DSX1268
endif
