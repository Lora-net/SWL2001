# SPDX-License-Identifier: BSD-3-Clause-Clear

add_library(smtc_modem_hal_implem OBJECT smtc_modem_hal.c)

target_link_libraries(smtc_modem_hal_implem PUBLIC
    LoRaBasicsModemApi
    smtc_hal
    smtc_modem_hal_interface
)

# Our smtc_modem_hal is directly including sx127x.h
if(RADIO_FAMILY STREQUAL sx127x)
    target_include_directories(smtc_modem_hal_implem PRIVATE
        $<TARGET_PROPERTY:${radio_driver_library},INCLUDE_DIRECTORIES>
    )
endif()
