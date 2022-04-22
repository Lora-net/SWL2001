##############################################################################
# Definitions for the SX128x tranceiver
##############################################################################
TARGET = sx128x

RADIO_DRIVER_C_SOURCES +=  \
	smtc_modem_core/radio_drivers/sx128x_driver/src/sx128x.c

SMTC_RAL_C_SOURCES += \
	smtc_modem_core/smtc_ral/src/ral_sx128x.c

SMTC_RALF_C_SOURCES += \
	smtc_modem_core/smtc_ralf/src/ralf_sx128x.c

LR1MAC_C_SOURCES += \
	smtc_modem_core/lr1mac/src/smtc_real/src/region_ww2g4.c 

SMTC_MODEM_CRYPTO_C_SOURCES += \
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/aes.c\
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/cmac.c\
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/soft_se.c

#-----------------------------------------------------------------------------
# Includes
#-----------------------------------------------------------------------------
MODEM_C_INCLUDES =  \
	-Ismtc_modem_core/radio_drivers/sx128x_driver/src\
	-Ismtc_modem_core/smtc_modem_crypto/soft_secure_element

#-----------------------------------------------------------------------------
# Radio specific compilation flags
#-----------------------------------------------------------------------------
MODEM_C_DEFS += \
	-DSX128X 
