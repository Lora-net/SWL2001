# SPDX-License-Identifier: BSD-3-Clause-Clear

add_library(smtc_modem_hal_implem OBJECT smtc_modem_hal.c)

target_link_libraries(smtc_modem_hal_implem PUBLIC
    LoRaBasicsModemApi
    smtc_hal
    smtc_modem_hal_interface
)

# For tx_user.h
target_include_directories(smtc_modem_hal_implem PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../main_examples
)
