# SPDX-License-Identifier: BSD-3-Clause-Clear

set(SMTC_MODEM_HAL_DIR ${CMAKE_CURRENT_LIST_DIR})

add_library(smtc_modem_hal_interface INTERFACE)
target_include_directories(smtc_modem_hal_interface INTERFACE ${SMTC_MODEM_HAL_DIR})
