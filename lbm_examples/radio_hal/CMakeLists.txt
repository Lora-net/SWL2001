# SPDX-License-Identifier: BSD-3-Clause-Clear

add_library(radio_hal OBJECT
    ${RADIO_FAMILY}_hal.c
    ral_${RADIO_FAMILY}_bsp.c
    radio_utilities.c
)

target_link_libraries(radio_hal PUBLIC
    smtc_hal
    smtc_modem_hal_implem
    smtc_ral::smtc_ral_${RADIO_FAMILY}
)

target_include_directories(radio_hal PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)
