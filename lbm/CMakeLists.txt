# SPDX-License-Identifier: BSD-3-Clause-Clear

cmake_minimum_required(VERSION 3.25)

project(lora_basics_modem
    VERSION 4.9.0
    DESCRIPTION "Semtech standard implementation of the LoRaWAN specifications"
    LANGUAGES C
)

set(CMAKE_C_STANDARD 17)
set(CMAKE_LINK_LIBRARIES_ONLY_TARGETS ON)

set(LBM_RADIO "" CACHE STRING "The LBM radio to use")
if(NOT DEFINED LBM_RADIO OR LBM_RADIO STREQUAL "")
   message(FATAL_ERROR "You must select the radio with -DLBM_RADIO=<selected radio>")
endif()
message(STATUS "From LBM top-dir - LBM_RADIO : ${LBM_RADIO}")

# The user enters LBM_RADIO, so we pass it as SMTC_RADIO to smtc_radio_interface
set(SMTC_RADIO ${LBM_RADIO})
# Some lower repos (e.g smtc_ral) use RAC_RADIO so we have to set it aswell - will be fixed in a future revision
set(RAC_RADIO ${LBM_RADIO})

# Include smtc_radio_interface to build radio_drivers/ral/ralf
add_subdirectory(smtc_radio_interface)
# smtc_radio_interface exposes RADIO_UPPER, RADIO_FAMILY_UPPER, RADIO_FAMILY
# These might need to be exposed to the caller CMakeLists.txt
if(NOT PROJECT_IS_TOP_LEVEL)
    set(RADIO_FAMILY ${RADIO_FAMILY} PARENT_SCOPE)
    set(RADIO_UPPER ${RADIO_UPPER} PARENT_SCOPE)
    set(RADIO_FAMILY_UPPER ${RADIO_FAMILY_UPPER} PARENT_SCOPE)
endif()

message(DEBUG "RAL path: '${SMTC_RAL_PATH}'.")
message(DEBUG "RALF path: '${SMTC_RALF_PATH}'.")
message(DEBUG "Radio drivers path: '${SMTC_RADIO_DRIVERS_PATH}'.")

# Create smtc_basic_hal_interface library
add_subdirectory(smtc_core_hal_interface)
message(DEBUG "Core HAL path: '${SMTC_CORE_HAL_PATH}'.")

# radio_planner (reports SMTC_RADIO_PLANNER_PATH) - creates "radio_planner" target
add_subdirectory(radio_planner)
target_compile_definitions(radio_planner PUBLIC USE_LBM=1)
message(DEBUG "Radio Planner path: '${SMTC_RADIO_PLANNER_PATH}'.")

# Add LBM
# As this repo is legacy LBM repo, we have to add following line to properly instanciate objects
set(LBM_LEGACY ON)
add_subdirectory(lbm_lib)

# radio_planner is built outside of lbm_lib, so it does not inherit the modem trace definitions set on lora_basics_modem.
# Give it the same ones (LBM_MODEM_TRACE and LBM_MODEM_TRACE_DEEP are defined by lbm_lib/options.cmake)
if(LBM_MODEM_TRACE)
    target_compile_definitions(radio_planner PRIVATE MODEM_HAL_DBG_TRACE=1)
else()
    target_compile_definitions(radio_planner PRIVATE MODEM_HAL_DBG_TRACE=0)
endif()

if(LBM_MODEM_TRACE AND LBM_MODEM_TRACE_DEEP)
    target_compile_definitions(radio_planner PRIVATE MODEM_HAL_DEEP_DBG_TRACE=1)
else()
    target_compile_definitions(radio_planner PRIVATE MODEM_HAL_DEEP_DBG_TRACE=0)
endif()
