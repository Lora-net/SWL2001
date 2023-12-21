##############################################################################
# Definitions for the SX126x tranceiver
##############################################################################
-include makefiles/options.mk

ifeq ($(RADIO),sx1272)
TARGET = sx1272
endif
ifeq ($(RADIO),sx1276)
TARGET = sx1276
endif

RADIO_DRIVER_C_SOURCES +=  \
	smtc_modem_core/radio_drivers/sx127x_driver/src/sx127x.c\

SMTC_RAL_C_SOURCES += \
	smtc_modem_core/smtc_ral/src/ral_sx127x.c

SMTC_RALF_C_SOURCES += \
	smtc_modem_core/smtc_ralf/src/ralf_sx127x.c

SMTC_MODEM_CRYPTO_C_SOURCES += \
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/aes.c\
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/cmac.c\
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/soft_se.c

#-----------------------------------------------------------------------------
# Includes
#-----------------------------------------------------------------------------
LBM_C_INCLUDES =  \
	-Ismtc_modem_core/radio_drivers/sx127x_driver/src\
	-Ismtc_modem_core/smtc_modem_crypto/soft_secure_element

#-----------------------------------------------------------------------------
# Region
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# Radio specific compilation flags
#-----------------------------------------------------------------------------
LBM_C_DEFS += \
	-DSX127X 

ifeq ($(RADIO),sx1272)
LBM_C_DEFS += \
    -DSX1272
endif

ifeq ($(RADIO),sx1276)
LBM_C_DEFS += \
    -DSX1276
endif
