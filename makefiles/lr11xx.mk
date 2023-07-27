##############################################################################
# Definitions for the LR11XX tranceiver
##############################################################################
-include makefiles/options.mk

ifeq ($(RADIO),lr1110)
TARGET = lr1110
endif
ifeq ($(RADIO),lr1120)
TARGET = lr1120
endif

# Allow modem options
ALLOW_CSMA_BUILD = yes

#-----------------------------------------------------------------------------
# Radio specific sources
#-----------------------------------------------------------------------------

RADIO_DRIVER_C_SOURCES += \
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_bootloader.c\
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_crypto_engine.c\
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_driver_version.c\
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_radio.c\
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_regmem.c\
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_system.c\
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_wifi.c\
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_lr_fhss.c\
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_gnss.c

SMTC_RAL_C_SOURCES += \
	smtc_modem_core/smtc_ral/src/ral_lr11xx.c

SMTC_RALF_C_SOURCES += \
	smtc_modem_core/smtc_ralf/src/ralf_lr11xx.c


ifeq ($(CRYPTO),LR11XX)
SMTC_MODEM_CRYPTO_C_SOURCES += \
	smtc_modem_core/smtc_modem_crypto/lr11xx_crypto_engine/lr11xx_ce.c
endif # LR11XX

ifeq ($(CRYPTO),LR11XX_WITH_CREDENTIALS)
SMTC_MODEM_CRYPTO_C_SOURCES += \
	smtc_modem_core/smtc_modem_crypto/lr11xx_crypto_engine/lr11xx_ce.c
endif # LR11XX_WITH_CREDENTIALS

ifeq ($(CRYPTO),SOFT)
SMTC_MODEM_CRYPTO_C_SOURCES += \
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/aes.c\
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/cmac.c\
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/soft_se.c
endif # soft_crypto

#-----------------------------------------------------------------------------
# Includes
#-----------------------------------------------------------------------------
MODEM_C_INCLUDES =  \
	-Ismtc_modem_core/radio_drivers/lr11xx_driver/src 

ifeq ($(CRYPTO),LR11XX)
MODEM_C_INCLUDES += \
	-Ismtc_modem_core/smtc_modem_crypto/lr11xx_crypto_engine
endif # LR11XX

ifeq ($(CRYPTO),LR11XX_WITH_CREDENTIALS)
MODEM_C_INCLUDES += \
	-Ismtc_modem_core/smtc_modem_crypto/lr11xx_crypto_engine
endif # LR11XX_WITH_CREDENTIALS

ifeq ($(CRYPTO),SOFT)
MODEM_C_INCLUDES += \
	-Ismtc_modem_core/smtc_modem_crypto/soft_secure_element
endif # soft_crypto

#-----------------------------------------------------------------------------
# Radio specific compilation flags
#-----------------------------------------------------------------------------
MODEM_C_DEFS += \
	-DLR11XX\
	-DLR11XX_TRANSCEIVER\
	-DLR11XX_DISABLE_WARNINGS

ifeq ($(CRYPTO),LR11XX)
MODEM_C_DEFS += \
	-DUSE_LR11XX_CE
endif # LR11XX

ifeq ($(CRYPTO),LR11XX_WITH_CREDENTIALS)
MODEM_C_DEFS += \
	-DUSE_LR11XX_CE \
	-DUSE_PRE_PROVISIONED_FEATURES
endif # LR11XX_WITH_CREDENTIALS
