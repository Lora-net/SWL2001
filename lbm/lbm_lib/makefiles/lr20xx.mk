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
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_driver_version.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_pram.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_radio_bluetooth_le.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_radio_bpsk.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_radio_ook.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_radio_oqpsk_15_4.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_radio_wi_sun.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_radio_wm_bus.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_radio_z_wave.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_radio_common.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_radio_fifo.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_radio_flrc.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_radio_fsk.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_radio_lora.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_radio_lr_fhss.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_regmem.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_rttof.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_system.c \
	$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/src/lr20xx_workarounds.c

SMTC_RAL_C_SOURCES += \
	$(SMTC_RADIO_INTERFACE_DIR)/smtc_ral/src/ral_lr20xx.c

SMTC_RALF_C_SOURCES += \
	$(SMTC_RADIO_INTERFACE_DIR)/smtc_ralf/src/ralf_lr20xx.c

SMTC_MODEM_CRYPTO_C_SOURCES += \
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/aes.c\
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/cmac.c\
	smtc_modem_core/smtc_modem_crypto/soft_secure_element/soft_se.c

#-----------------------------------------------------------------------------
# Includes
#-----------------------------------------------------------------------------
LBM_C_INCLUDES =  \
	-I$(SMTC_RADIO_DRIVERS_DIR)/lr20xx_driver/inc

#-----------------------------------------------------------------------------
# Radio specific compilation flags
#-----------------------------------------------------------------------------
LBM_C_DEFS += \
	-DLR20XX \
	-DRAL_DRIVER_DIRECT_DISPATCH \
	-DRAL_DRIVER_DIRECT_DISPATCH_LR20XX

ifeq ($(RADIO),lr2021)
LBM_C_DEFS += \
	-DLR2021
endif
