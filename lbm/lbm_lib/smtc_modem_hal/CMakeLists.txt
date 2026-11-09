# SPDX-License-Identifier: BSD-3-Clause-Clear

set(SMTC_MODEM_HAL_DIR ${CMAKE_CURRENT_LIST_DIR})

add_library(smtc_modem_hal_interface INTERFACE)
target_include_directories(smtc_modem_hal_interface INTERFACE ${SMTC_MODEM_HAL_DIR})

# Link this lib with smtc_core_hal_interface, so that the LBM users only have to #include "smtc_modem_hal.h"
# And as smtc_modem_hal.h does #include "smtc_core_hal.h"
target_link_libraries(smtc_modem_hal_interface INTERFACE smtc_core_hal_interface)
