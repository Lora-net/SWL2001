# SPDX-License-Identifier: BSD-3-Clause-Clear

cmake_minimum_required(VERSION 3.25)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/smtc_hal_nrf52840/cmake_nrf52840_toolchain.cmake)

project(lbm_examples
    DESCRIPTION "LoRa Basics Modem Nordic nRF52840 porting application"
    LANGUAGES C ASM
)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_LINK_LIBRARIES_ONLY_TARGETS OFF)

add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)

# Set default build type
set(default_build_type "MinSizeRel")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)

    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Disable fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE OFF)

# Enable ccache
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

# Generate a compile_commands.json for IDEs / clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################################################################################
# Options

set(APP "" CACHE STRING "The example to build")
set(APPS hw_modem lctt_certif periodical_uplink porting_tests)
set_property(CACHE APP PROPERTY STRINGS ${APPS})
if(APP STREQUAL "")
    message(FATAL_ERROR "You need to define an -DAPP= from the list ${APPS}")
endif()

set(APP_REGION "" CACHE STRING "The region to use with the app")

option(USE_LR11XX_CRC_SPI "Use CRC over SPI with LR11XX")

option(APP_DEBUG "Debug the example app")

option(APP_TRACE "choose to enable or disable application trace print (default: trace is ON)" ON)

option(LEGACY_EVK_LR2021 "Use legacy EVK board pinout for LR2021" OFF)

################################################################################
# Run the example cmake file to fetch LBM build options

add_executable(lbm_example.elf)

add_subdirectory(main_examples)

################################################################################
# Set values and load LBM lib
# Build for all examples
set(LBM_CSMA ON CACHE BOOL "")
# Enabled for all examples
set(LBM_CSMA_BY_DEFAULT ON CACHE BOOL "")


# TODO: How to do that properly?
add_subdirectory(../../lbm_lib lbm_lib)

################################################################################

add_subdirectory(${SMTC_HAL_DIR})
add_subdirectory(smtc_modem_hal)
add_subdirectory(radio_hal)

################################################################################
# Compile options

string(TOUPPER ${APP} APP_UPPER)
target_compile_definitions(lbm_example.elf PRIVATE MAKEFILE_APP=${APP_UPPER})

if(NOT APP_REGION STREQUAL "")
    target_compile_definitions(lbm_example.elf PRIVATE MODEM_EXAMPLE_REGION=${APP_REGION})
endif()

if(USE_LR11XX_CRC_SPI)
    target_compile_definitions(radio_hal PRIVATE USE_LR11XX_CRC_OVER_SPI)
endif()

if(APP_DEBUG)
    target_compile_definitions(smtc_hal PRIVATE HW_DEBUG_PROBE=1)
endif()

if(APP STREQUAL "hw_modem")
    target_compile_definitions(smtc_hal PUBLIC HW_MODEM_ENABLED)
endif()

if(APP_TRACE)
    target_compile_definitions(smtc_hal PUBLIC HAL_DBG_TRACE=1)
else()
    target_compile_definitions(smtc_hal PUBLIC HAL_DBG_TRACE=0)
endif()

if(LEGACY_EVK_LR2021)
    target_compile_definitions(radio_hal PRIVATE LEGACY_EVK_LR2021=1)
endif()

# All this is derived from what LBM enabled

string(TOUPPER ${LBM_RADIO} RADIO_UPPER)
target_compile_definitions(smtc_modem_hal_implem PUBLIC ${RADIO_UPPER})
target_compile_definitions(smtc_hal PUBLIC ${RADIO_UPPER})

string(TOUPPER ${RADIO_FAMILY} RADIO_FAMILY_UPPER)
target_compile_definitions(smtc_modem_hal_implem PUBLIC ${RADIO_FAMILY_UPPER})
target_compile_definitions(smtc_hal PUBLIC ${RADIO_FAMILY_UPPER})

if(RADIO_FAMILY STREQUAL lr11xx)
    # This define is for LR11XX **outside** modem-e
    target_compile_definitions(smtc_hal PUBLIC LR11XX_TRANSCEIVER)
endif()

# TODO: remove once it's removed from smtc_modem_api.h
if(RADIO_FAMILY STREQUAL "LR11XX")
    target_compile_definitions(smtc_modem_hal_implem PUBLIC LR11XX_TRANSCEIVER)
endif()

if(NOT LBM_NUMBER_OF_STACKS STREQUAL "1")
    target_compile_definitions(smtc_hal PUBLIC MULTISTACK)
endif()

if(LBM_GEOLOCATION)
    target_compile_definitions(smtc_modem_hal_implem PUBLIC ADD_APP_GEOLOCATION)
endif()

if(LBM_RELAY_TX)
    target_compile_definitions(smtc_modem_hal_implem PUBLIC USE_RELAY_TX)
endif()

if(LBM_RELAY_RX)
    target_compile_definitions(smtc_modem_hal_implem PUBLIC USE_RELAY_RX)
endif()

if(LBM_FUOTA)
    target_compile_definitions(smtc_hal PUBLIC USE_FUOTA USE_FLASH_READ_MODIFY_WRITE)
    target_compile_definitions(smtc_modem_hal_implem PUBLIC USE_FUOTA USE_FLASH_READ_MODIFY_WRITE)
endif()

if(LBM_STORE_AND_FORWARD)
    target_compile_definitions(smtc_modem_hal_implem PUBLIC USE_STORE_AND_FORWARD)
endif()

if(LBM_PERF_TEST)
    target_compile_definitions(lbm_example.elf PUBLIC PERF_TEST_ENABLED)
endif()

if(LBM_CRYPTO MATCHES "^LR11XX")
    target_compile_definitions(lbm_example.elf PUBLIC USE_LR11XX_CRYPTO)
endif()
if(LBM_CRYPTO MATCHES "^LR11XX_WITH_CREDENTIALS$")
    target_compile_definitions(lbm_example.elf PUBLIC USE_LR11XX_CREDENTIALS)
endif()

if(LBM_CSMA)
    target_compile_definitions(lbm_example.elf PUBLIC ALLOW_CSMA)
    if(LBM_CSMA_BY_DEFAULT)
        target_compile_definitions(lbm_example.elf PUBLIC ALLOW_CSMA_AND_ENABLE_AT_BOOT)
    endif()
endif()

################################################################################

target_sources(lbm_example.elf PRIVATE
    main.c
    # $<TARGET_OBJECTS:st_hal>
    $<TARGET_OBJECTS:smtc_hal>
)

target_link_libraries(lbm_example.elf PRIVATE
    lora_basics_modem_core
    radio_hal
    smtc_modem_hal_implem
)

add_custom_command(
    TARGET lbm_example.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex "$<TARGET_FILE:lbm_example.elf>" "${CMAKE_CURRENT_BINARY_DIR}/lbm_example.hex"
    COMMENT "Creating hex..."
)

add_custom_command(
    TARGET lbm_example.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary -S "$<TARGET_FILE:lbm_example.elf>" "${CMAKE_CURRENT_BINARY_DIR}/lbm_example.bin"
    COMMENT "Creating bin..."
)

add_custom_target(flash
    DEPENDS lbm_example.elf
    COMMAND nrfjprog -f nrf52 --program ${CMAKE_CURRENT_BINARY_DIR}/lbm_example.hex --sectorerase
    COMMAND nrfjprog -f nrf52 --reset
    USES_TERMINAL
)