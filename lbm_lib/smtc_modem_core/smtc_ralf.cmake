# SPDX-License-Identifier: BSD-3-Clause-Clear

set(SMTC_RALF_DIR ${CMAKE_CURRENT_LIST_DIR}/smtc_ralf)

add_library(lora_basics_modem_ralf OBJECT
    ${SMTC_RALF_DIR}/src/ralf_${RADIO_FAMILY}.c
)

target_include_directories(lora_basics_modem_ralf INTERFACE
    ${SMTC_RALF_DIR}/src
)

target_link_libraries(lora_basics_modem_ralf
    smtc_ral::smtc_ral_${RADIO_FAMILY}
)
