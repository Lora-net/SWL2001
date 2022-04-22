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
# Common sources
#-----------------------------------------------------------------------------

ifeq ($(USE_GNSS),yes)
SMTC_MODEM_SERVICES_C_SOURCES += \
	smtc_modem_core/smtc_modem_services/src/almanac_update/almanac_update.c
endif

RADIO_DRIVER_C_SOURCES += \
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_bootloader.c\
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_crypto_engine.c\
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_driver_version.c\
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_radio.c\
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_regmem.c\
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_system.c\
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_wifi.c\
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_lr_fhss.c
ifeq ($(USE_GNSS),yes)
RADIO_DRIVER_C_SOURCES += \
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_gnss.c
endif

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
	-DLR11XX_TRANSCEIVER

ifeq ($(RADIO),lr1120)
MODEM_C_DEFS += \
	-DLR1120
endif

ifeq ($(CRYPTO),LR11XX)
MODEM_C_DEFS += \
	-DUSE_LR11XX_CE
endif # LR11XX

ifeq ($(CRYPTO),LR11XX_WITH_CREDENTIALS)
MODEM_C_DEFS += \
	-DUSE_LR11XX_CE \
	-DUSE_PRE_PROVISIONED_FEATURES
endif # LR11XX_WITH_CREDENTIALS

# GNSS USE
ifeq ($(USE_GNSS),yes)
MODEM_C_DEFS += \
	-DENABLE_MODEM_GNSS_FEATURE
endif
