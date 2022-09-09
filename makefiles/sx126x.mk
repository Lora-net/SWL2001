##############################################################################
# Definitions for the SX128x tranceiver
##############################################################################
ifeq ($(RADIO),sx1261)
TARGET = sx1261
endif
ifeq ($(RADIO),sx1262)
TARGET = sx1262
endif
ifeq ($(RADIO),sx1268)
TARGET = sx1268
endif

RADIO_DRIVER_C_SOURCES +=  \
	smtc_modem_core/radio_drivers/sx126x_driver/src/sx126x.c\
	smtc_modem_core/radio_drivers/sx126x_driver/src/sx126x_lr_fhss.c\
	smtc_modem_core/radio_drivers/sx126x_driver/src/lr_fhss_mac.c

SMTC_RAL_C_SOURCES += \
	smtc_modem_core/smtc_ral/src/ral_sx126x.c

SMTC_RALF_C_SOURCES += \
	smtc_modem_core/smtc_ralf/src/ralf_sx126x.c

SMTC_MODEM_CRYPTO_C_SOURCES += \
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/aes.c\
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/cmac.c\
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/soft_se.c

#-----------------------------------------------------------------------------
# Includes
#-----------------------------------------------------------------------------
MODEM_C_INCLUDES =  \
	-Ismtc_modem_core/radio_drivers/sx126x_driver/src\
	-Ismtc_modem_core/smtc_modem_crypto/soft_secure_element

#-----------------------------------------------------------------------------
# Region
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# Radio specific compilation flags
#-----------------------------------------------------------------------------
MODEM_C_DEFS += \
	-DSX126X 

ifeq ($(RADIO),sx1262)
MODEM_C_DEFS += \
    -DSX1262
endif

ifeq ($(RADIO),sx1268)
MODEM_C_DEFS += \
    -DSX1268
endif
