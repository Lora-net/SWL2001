##############################################################################
# Definitions for the SX128x tranceiver
##############################################################################
TARGET = sx128x


RADIO_HAL_C_SOURCES += \
	user_app/radio_hal/sx128x_hal.c\
	user_app/radio_hal/ral_sx128x_bsp.c

#-----------------------------------------------------------------------------
# Includes
#-----------------------------------------------------------------------------
MODEM_C_INCLUDES =  \
	-I$(LORA_BASICS_MODEM)/smtc_modem_core/radio_drivers/sx128x_driver/src\
	-I$(LORA_BASICS_MODEM)/smtc_modem_core/smtc_ralf/src \
	-I$(LORA_BASICS_MODEM)/smtc_modem_core/smtc_ral/src

#-----------------------------------------------------------------------------
# Radio specific compilation flags
#-----------------------------------------------------------------------------
COMMON_C_DEFS += \
	-DSX128X 
