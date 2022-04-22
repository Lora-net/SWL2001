##############################################################################
# Definitions for the LR11XX tranceiver
##############################################################################
ifeq ($(RADIO),lr1110)
TARGET = lr1110
endif
ifeq ($(RADIO),lr1120)
TARGET = lr1120
endif

#-----------------------------------------------------------------------------
# User application sources
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# Common sources
#-----------------------------------------------------------------------------


RADIO_HAL_C_SOURCES += \
	user_app/radio_hal/lr11xx_hal.c\
	user_app/radio_hal/ral_lr11xx_bsp.c\
	user_app/radio_hal/lr11xx_pa_pwr_cfg.c

#-----------------------------------------------------------------------------
# Includes
#-----------------------------------------------------------------------------
# XXX This is needed for the following:
# - lr11xx_hal.h
# - ral_defs.f
# - ralf_defs.f
MODEM_C_INCLUDES =  \
	-I$(LORA_BASICS_MODEM)/smtc_modem_core/radio_drivers/lr11xx_driver/src \
	-I$(LORA_BASICS_MODEM)/smtc_modem_core/smtc_ralf/src \
	-I$(LORA_BASICS_MODEM)/smtc_modem_core/smtc_ral/src

#-----------------------------------------------------------------------------
# Radio specific compilation flags
#-----------------------------------------------------------------------------
MODEM_C_DEFS += \
	-DLR11XX\
	-DLR11XX_TRANSCEIVER

ifeq ($(USE_LR11XX_CRC_SPI),yes)
MODEM_C_DEFS += \
	-DUSE_LR11XX_CRC_OVER_SPI
endif

# GNSS USE
ifeq ($(USE_GNSS),yes)
MODEM_C_DEFS += \
	-DENABLE_MODEM_GNSS_FEATURE
endif
