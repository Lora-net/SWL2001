# ----------------------------------------------------------------------------
# @file module.mk
#
# Contains list of source files to be compiled in this module.
# ----------------------------------------------------------------------------

MODULE_C_SOURCES = \
    device_management/dm_downlink.c \
    device_management/modem_context.c \
    lorawan_api/lorawan_api.c \
    modem_core/smtc_modem_test.c \
    modem_core/smtc_modem.c \
    modem_services/fifo_ctrl.c \
    modem_services/modem_utilities.c \
    modem_services/smtc_modem_services_hal.c \
    modem_services/smtc_clock_sync.c \
    modem_supervisor/modem_supervisor.c \

MODULE_C_INCLUDES = \
    . \
    device_management \
    lorawan_api \
    modem_core \
    modem_services \
    modem_supervisor \
