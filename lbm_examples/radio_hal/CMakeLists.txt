# SPDX-License-Identifier: BSD-3-Clause-Clear

set(radio_hal_extra_sources "")
if(RADIO_FAMILY STREQUAL lr20xx)
    list(APPEND radio_hal_extra_sources pa_tables_lr20xx_common.c)
endif()

add_library(radio_hal OBJECT
    ${RADIO_FAMILY}_hal.c
    ral_${RADIO_FAMILY}_bsp.c
    radio_utilities.c
    ${radio_hal_extra_sources}
)

target_link_libraries(radio_hal PUBLIC
    smtc_hal
    smtc_modem_hal_implem
    smtc_ral::smtc_ral_${RADIO_FAMILY}
)

target_include_directories(radio_hal PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)
