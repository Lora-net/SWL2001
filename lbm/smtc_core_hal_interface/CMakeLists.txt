# SPDX-License-Identifier: BSD-3-Clause-Clear

set(SMTC_CORE_HAL_INTERFACE_DIR ${CMAKE_CURRENT_LIST_DIR})

set(SMTC_CORE_HAL_PATH ${CMAKE_CURRENT_LIST_DIR} PARENT_SCOPE)

add_library(smtc_core_hal_interface INTERFACE)
target_include_directories(smtc_core_hal_interface INTERFACE ${SMTC_CORE_HAL_INTERFACE_DIR})

# Optional: Disable modem HAL debug trace for every target including smtc_core_hal_trace.h
# Disable with -DMODEM_HAL_DBG_TRACE=OFF
option(MODEM_HAL_DBG_TRACE "Enable modem HAL debug trace output" ON)
if(NOT MODEM_HAL_DBG_TRACE)
    target_compile_definitions(smtc_core_hal_interface INTERFACE MODEM_HAL_DBG_TRACE=0)
    message(STATUS "MODEM_HAL_DBG_TRACE: OFF")
endif()
