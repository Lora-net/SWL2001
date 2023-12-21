##############################################################################
# Definitions for the LR11XX tranceiver
##############################################################################
-include app_makefiles/app_options.mk

ifeq ($(TARGET_RADIO),lr1110)
TARGET = lr1110
endif
ifeq ($(TARGET_RADIO),lr1120)
TARGET = lr1120
endif
ifeq ($(TARGET_RADIO),lr1121)
TARGET = lr1121
endif

#-----------------------------------------------------------------------------
# Options
#-----------------------------------------------------------------------------
APP_GEOLOCATION?=no

#-----------------------------------------------------------------------------
# User application sources
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# Common sources
#-----------------------------------------------------------------------------
RADIO_HAL_C_SOURCES += \
	radio_hal/lr11xx_hal.c\
	radio_hal/ral_lr11xx_bsp.c

#-----------------------------------------------------------------------------
# Includes
#-----------------------------------------------------------------------------
# This is needed for the following:
# - lr11xx_hal.h
# - ral_lr11xx_bsp.h
MODEM_C_INCLUDES =  \
	-I$(LORA_BASICS_MODEM)/smtc_modem_core/radio_drivers/lr11xx_driver/src \
	-I$(LORA_BASICS_MODEM)/smtc_modem_core/smtc_ral/src

#-----------------------------------------------------------------------------
# Radio specific compilation flags
#-----------------------------------------------------------------------------
COMMON_C_DEFS += \
	-DLR11XX\
	-DLR11XX_TRANSCEIVER\
	-DLR11XX_DISABLE_WARNINGS

ifeq ($(TARGET_RADIO),lr1120)
COMMON_C_DEFS += \
	-DLR1120
ifeq ($(APP_GEOLOCATION),yes)
COMMON_C_DEFS += \
	-DADD_APP_GEOLOCATION
endif
endif

ifeq ($(TARGET_RADIO),lr1110)
COMMON_C_DEFS += \
	-DLR1110
ifeq ($(APP_GEOLOCATION),yes)
COMMON_C_DEFS += \
	-DADD_APP_GEOLOCATION
endif
endif

ifeq ($(TARGET_RADIO),lr1121)
COMMON_C_DEFS += \
	-DLR1121
endif

ifeq ($(USE_LR11XX_CRC_SPI),yes)
COMMON_C_DEFS += \
	-DUSE_LR11XX_CRC_OVER_SPI
endif

ifneq ($(CRYPTO),SOFT)
COMMON_C_DEFS += \
	-DUSE_LR11XX_CRYPTO
endif

ifeq ($(CRYPTO),LR11XX_WITH_CREDENTIALS)
COMMON_C_DEFS += \
	-DUSE_LR11XX_CREDENTIALS
endif # LR11XX_WITH_CREDENTIALS
