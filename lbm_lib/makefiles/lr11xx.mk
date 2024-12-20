##############################################################################
# Definitions for the LR11XX transceiver
##############################################################################
-include makefiles/options.mk

ifeq ($(RADIO),lr1110)
TARGET = lr1110
endif
ifeq ($(RADIO),lr1120)
TARGET = lr1120
endif
ifeq ($(RADIO),lr1121)
TARGET = lr1121
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
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_lr_fhss.c

ifneq ($(RADIO),lr1121)
RADIO_DRIVER_C_SOURCES += \
	smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_wifi.c\
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

ifeq ($(LBM_ALMANAC),yes)
SMTC_MODEM_CORE_C_SOURCES += \
	smtc_modem_core/modem_services/almanac_packages/almanac.c
endif

ifeq ($(LBM_GEOLOCATION),yes)
SMTC_MODEM_CORE_C_SOURCES += \
	smtc_modem_core/geolocation_services/mw_common.c\
	smtc_modem_core/geolocation_services/mw_gnss_scan.c\
	smtc_modem_core/geolocation_services/mw_gnss_send.c\
	smtc_modem_core/geolocation_services/mw_gnss_almanac.c\
	smtc_modem_core/geolocation_services/gnss_helpers.c\
	smtc_modem_core/geolocation_services/mw_wifi_scan.c\
	smtc_modem_core/geolocation_services/mw_wifi_send.c\
	smtc_modem_core/geolocation_services/wifi_helpers.c
endif

#-----------------------------------------------------------------------------
# Includes
#-----------------------------------------------------------------------------
LBM_C_INCLUDES =  \
	-Ismtc_modem_core/radio_drivers/lr11xx_driver/src

ifeq ($(CRYPTO),LR11XX)
LBM_C_INCLUDES += \
	-Ismtc_modem_core/smtc_modem_crypto/lr11xx_crypto_engine
endif # LR11XX

ifeq ($(CRYPTO),LR11XX_WITH_CREDENTIALS)
LBM_C_INCLUDES += \
	-Ismtc_modem_core/smtc_modem_crypto/lr11xx_crypto_engine
endif # LR11XX_WITH_CREDENTIALS

ifeq ($(CRYPTO),SOFT)
LBM_C_INCLUDES += \
	-Ismtc_modem_core/smtc_modem_crypto/soft_secure_element
endif # soft_crypto

ifeq ($(LBM_ALMANAC),yes)
LBM_C_INCLUDES += \
	-Ismtc_modem_core/modem_services \
	-Ismtc_modem_core/modem_services/almanac_packages
endif

ifeq ($(LBM_GEOLOCATION),yes)
LBM_C_INCLUDES += \
	-Ismtc_modem_core/geolocation_services
endif

#-----------------------------------------------------------------------------
# Radio specific compilation flags
#-----------------------------------------------------------------------------
LBM_C_DEFS += \
	-DLR11XX\
	-DLR11XX_TRANSCEIVER\
	-DLR11XX_DISABLE_WARNINGS

ifeq ($(CRYPTO),LR11XX)
LBM_C_DEFS += \
	-DUSE_LR11XX_CE
endif # LR11XX

ifeq ($(CRYPTO),LR11XX_WITH_CREDENTIALS)
LBM_C_DEFS += \
	-DUSE_LR11XX_CE \
	-DUSE_PRE_PROVISIONED_FEATURES
endif # LR11XX_WITH_CREDENTIALS

ifeq ($(LBM_ALMANAC),yes)
LBM_C_DEFS += \
    -DADD_ALMANAC
endif

ifeq ($(LBM_GEOLOCATION),yes)
LBM_C_DEFS += \
    -DADD_LBM_GEOLOCATION
endif
