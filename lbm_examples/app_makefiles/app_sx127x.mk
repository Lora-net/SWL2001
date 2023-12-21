##############################################################################
# Definitions for the SX126x tranceiver
##############################################################################
-include app_makefiles/app_options.mk

ifeq ($(TARGET_RADIO),sx1272)
TARGET = sx1272
endif
ifeq ($(TARGET_RADIO),sx1276)
TARGET = sx1276
endif

#-----------------------------------------------------------------------------
# Common sources
#-----------------------------------------------------------------------------

RADIO_HAL_C_SOURCES += \
	radio_hal/sx127x_hal.c \
	radio_hal/ral_sx127x_bsp.c

#-----------------------------------------------------------------------------
# Includes
#-----------------------------------------------------------------------------
# This is needed for the following:
# - sx127x_hal.h
# - ral_sx127x_bsp.h
MODEM_C_INCLUDES =  \
	-I$(LORA_BASICS_MODEM)/smtc_modem_core/radio_drivers/sx127x_driver/src\
	-I$(LORA_BASICS_MODEM)/smtc_modem_core/smtc_ral/src

#-----------------------------------------------------------------------------
# Region
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# Radio specific compilation flags
#-----------------------------------------------------------------------------
MODEM_C_DEFS += \
	-DSX127X 

ifeq ($(TARGET_RADIO),sx1272)
MODEM_C_DEFS += \
    -DSX1272
endif

ifeq ($(TARGET_RADIO),sx1276)
MODEM_C_DEFS += \
    -DSX1276
endif
