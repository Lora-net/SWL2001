##############################################################################
# Definitions for the Relay
##############################################################################

ifeq ($(RELAY_RX_ENABLE),yes)
RELAY_C_SOURCES +=  \
    smtc_modem_core/lr1mac/src/relay/common/wake_on_radio.c \
    smtc_modem_core/lr1mac/src/relay/common/relay_real.c \
    smtc_modem_core/lr1mac/src/relay/common/wake_on_radio_ral.c \
    smtc_modem_core/lr1mac/src/relay/common/relay_mac_parser.c \
    smtc_modem_core/lr1mac/src/relay/relay_rx/relay_rx_mac_parser.c \
    smtc_modem_core/lr1mac/src/relay/relay_rx/relay_rx.c \
	smtc_modem_core/modem_services/relay_service/lorawan_relay_rx_service.c

endif
   
#-----------------------------------------------------------------------------
# Includes
#-----------------------------------------------------------------------------
RELAY_C_INCLUDES =  \
    -Ismtc_modem_core/lr1mac/src/relay/common \
    -Ismtc_modem_core/lr1mac/src/relay/relay_rx\
	-Ismtc_modem_core/modem_services/relay_service



#-----------------------------------------------------------------------------
# Region
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# Radio specific compilation flags
#-----------------------------------------------------------------------------
ifeq ($(RELAY_RX_ENABLE),yes)
RELAY_C_DEFS += \
    -DRELAY_RX
endif
