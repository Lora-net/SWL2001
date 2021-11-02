##############################################################################
# Definitions for the LR1110 tranceiver
##############################################################################
TARGET = lr1110


#-----------------------------------------------------------------------------
# Common sources
#-----------------------------------------------------------------------------

SMTC_MODEM_CORE_C_SOURCES += \
	smtc_modem_core/modem_core/smtc_modem_api_lr1110_crypto_engine.c\
	smtc_modem_core/modem_core/smtc_modem_api_lr1110_system.c

ifeq ($(USE_GNSS),yes)
SMTC_MODEM_SERVICES_C_SOURCES += \
	smtc_modem_core/smtc_modem_services/src/almanac_update/almanac_update.c
endif

RADIO_DRIVER_C_SOURCES += \
	smtc_modem_core/radio_drivers/lr1110_driver/src/lr1110_bootloader.c\
	smtc_modem_core/radio_drivers/lr1110_driver/src/lr1110_crypto_engine.c\
	smtc_modem_core/radio_drivers/lr1110_driver/src/lr1110_driver_version.c\
	smtc_modem_core/radio_drivers/lr1110_driver/src/lr1110_radio.c\
	smtc_modem_core/radio_drivers/lr1110_driver/src/lr1110_regmem.c\
	smtc_modem_core/radio_drivers/lr1110_driver/src/lr1110_system.c\
	smtc_modem_core/radio_drivers/lr1110_driver/src/lr1110_wifi.c
ifeq ($(USE_GNSS),yes)
RADIO_DRIVER_C_SOURCES += \
	smtc_modem_core/radio_drivers/lr1110_driver/src/lr1110_gnss.c
endif


SMTC_RAL_C_SOURCES += \
	smtc_modem_core/smtc_ral/src/ral_lr1110.c

SMTC_RALF_C_SOURCES += \
	smtc_modem_core/smtc_ralf/src/ralf_lr1110.c


ifeq ($(CRYPTO),LR1110)
SMTC_MODEM_CRYPTO_C_SOURCES += \
	smtc_modem_core/smtc_modem_crypto/lr1110_secure_element/lr1110_se.c
endif # LR1110

ifeq ($(CRYPTO),LR1110_WITH_CREDENTIALS)
SMTC_MODEM_CRYPTO_C_SOURCES += \
	smtc_modem_core/smtc_modem_crypto/lr1110_secure_element/lr1110_se.c
endif # LR1110_WITH_CREDENTIALS

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
	-Ismtc_modem_core/radio_drivers/lr1110_driver/src \

ifeq ($(CRYPTO),LR1110)
MODEM_C_INCLUDES += \
	-Ismtc_modem_core/smtc_modem_crypto/lr1110_secure_element
endif # LR1110

ifeq ($(CRYPTO),LR1110_WITH_CREDENTIALS)
MODEM_C_INCLUDES += \
	-Ismtc_modem_core/smtc_modem_crypto/lr1110_secure_element
endif # LR1110_WITH_CREDENTIALS

ifeq ($(CRYPTO),SOFT)
MODEM_C_INCLUDES += \
	-Ismtc_modem_core/smtc_modem_crypto/soft_secure_element
endif # soft_crypto


#-----------------------------------------------------------------------------
# Region
#-----------------------------------------------------------------------------
ifeq ($(CHINA_DEMO),yes)
REGION_CN_470_RP_1_0 = yes
endif # china_demo

ifeq ($(CHINA_DEMO),yes)
MODEM_C_DEFS += \
	-DCHINA_RP_1_DEMO
endif # china_demo

ifeq ($(HYBRID_CHINA),yes)
MODEM_C_DEFS += \
    -DHYBRID_CN470_MONO_CHANNEL
endif # hybrid_china

#-----------------------------------------------------------------------------
# Radio specific compilation flags
#-----------------------------------------------------------------------------
MODEM_C_DEFS += \
	-DLR1110\
	-DLR1110_TRANSCEIVER

ifeq ($(CRYPTO),LR1110)
MODEM_C_DEFS += \
	-DUSE_LR1110_SE
endif # LR1110

ifeq ($(CRYPTO),LR1110_WITH_CREDENTIALS)
MODEM_C_DEFS += \
	-DUSE_LR1110_SE \
	-DUSE_PRE_PROVISIONED_FEATURES
endif # LR1110

# GNSS USE
ifeq ($(USE_GNSS),yes)
MODEM_C_DEFS += \
	-DENABLE_MODEM_GNSS_FEATURE
endif
