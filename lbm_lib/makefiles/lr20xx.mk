##############################################################################
# Definitions for the LR11XX tranceiver
##############################################################################
-include makefiles/options.mk

ifeq ($(RADIO),lr2021)
TARGET = lr2021
endif

# Allow modem options
ALLOW_CSMA_BUILD = yes


#-----------------------------------------------------------------------------
# Radio specific sources
#-----------------------------------------------------------------------------

RADIO_DRIVER_C_SOURCES += \
	smtc_modem_core/radio_drivers/lr20xx_driver/src/lr20xx_driver_version.c \
	smtc_modem_core/radio_drivers/lr20xx_driver/src/lr20xx_radio_common.c \
	smtc_modem_core/radio_drivers/lr20xx_driver/src/lr20xx_radio_fifo.c \
	smtc_modem_core/radio_drivers/lr20xx_driver/src/lr20xx_radio_flrc.c \
	smtc_modem_core/radio_drivers/lr20xx_driver/src/lr20xx_radio_fsk.c \
	smtc_modem_core/radio_drivers/lr20xx_driver/src/lr20xx_radio_lora.c \
	smtc_modem_core/radio_drivers/lr20xx_driver/src/lr20xx_radio_lr_fhss.c \
	smtc_modem_core/radio_drivers/lr20xx_driver/src/lr20xx_regmem.c \
	smtc_modem_core/radio_drivers/lr20xx_driver/src/lr20xx_rttof.c \
	smtc_modem_core/radio_drivers/lr20xx_driver/src/lr20xx_system.c \
	smtc_modem_core/radio_drivers/lr20xx_driver/src/lr20xx_workarounds.c

SMTC_RAL_C_SOURCES += \
	smtc_modem_core/smtc_ral/src/ral_lr20xx.c

SMTC_RALF_C_SOURCES += \
	smtc_modem_core/smtc_ralf/src/ralf_lr20xx.c

SMTC_MODEM_CRYPTO_C_SOURCES += \
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/aes.c\
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/cmac.c\
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/soft_se.c

#-----------------------------------------------------------------------------
# Includes
#-----------------------------------------------------------------------------
LBM_C_INCLUDES =  \
	-Ismtc_modem_core/radio_drivers/lr20xx_driver/inc

#-----------------------------------------------------------------------------
# Radio specific compilation flags
#-----------------------------------------------------------------------------
LBM_C_DEFS += \
	-DLR20XX

ifeq ($(TARGET_RADIO),lr2021)
COMMON_C_DEFS += \
	-DLR2021
endif
