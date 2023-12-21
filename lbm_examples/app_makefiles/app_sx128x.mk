##############################################################################
# Definitions for the SX128x tranceiver
##############################################################################
-include app_makefiles/app_options.mk

TARGET = sx128x


RADIO_HAL_C_SOURCES += \
	radio_hal/sx128x_hal.c\
	radio_hal/ral_sx128x_bsp.c

#-----------------------------------------------------------------------------
# Includes
#-----------------------------------------------------------------------------
# This is needed for the following:
# - sx128x_hal.h
# - ral_sx128x_bsp.h
MODEM_C_INCLUDES =  \
	-I$(LORA_BASICS_MODEM)/smtc_modem_core/radio_drivers/sx128x_driver/src\
	-I$(LORA_BASICS_MODEM)/smtc_modem_core/smtc_ral/src

#-----------------------------------------------------------------------------
# Radio specific compilation flags
#-----------------------------------------------------------------------------
COMMON_C_DEFS += \
	-DSX128X 
