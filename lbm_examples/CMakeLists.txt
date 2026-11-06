# SPDX-License-Identifier: BSD-3-Clause-Clear

cmake_minimum_required(VERSION 3.25)

if(BOARD STREQUAL NUCLEO_L073)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/smtc_hal_l0_LL/cmake_stm32l0_toolchain.cmake)
elseif(BOARD STREQUAL NUCLEO_L476)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/smtc_hal_l4/cmake_stm32l4_toolchain.cmake)
else()
    message(FATAL_ERROR "You have to define a -DBOARD=NUCLEO_L476 or NUCLEO_L073")
endif()
set_property(CACHE BOARD PROPERTY STRINGS "NUCLEO_L476;NUCLEO_L073")

project(lbm_examples
    DESCRIPTION "LoRa Basics Modem examples"
    LANGUAGES C ASM
)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_LINK_LIBRARIES_ONLY_TARGETS ON)

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
# Project Options

set(APP "" CACHE STRING "The example to build")
set(APPS hw_modem lctt_certif periodical_uplink porting_tests)
set_property(CACHE APP PROPERTY STRINGS ${APPS})
if(APP STREQUAL "")
    message(FATAL_ERROR "You need to define an -DAPP= from the list ${APPS}")
endif()

set(APP_REGION "" CACHE STRING "The region to use with the app")

option(USE_LR11XX_CRC_SPI "Use CRC over SPI with LR11XX")

option(APP_DEBUG "Debug the example app" OFF)

option(APP_TRACE "choose to enable or disable application trace print (default: trace is ON)" ON)

option(LEGACY_EVK_LR2021 "Use legacy EVK board pinout for LR2021" OFF)

################################################################################
# First build the HAL that might set useful variables

add_subdirectory(${SMTC_HAL_DIR})

################################################################################
# Set values and load LBM lib

# Build for all examples
set(LBM_CSMA ON CACHE BOOL "")
# Enabled for all examples
set(LBM_CSMA_BY_DEFAULT ON CACHE BOOL "")

if(APP STREQUAL hw_modem)
    # Set LBM build options
    set(LBM_ALC_SYNC ON CACHE BOOL "")
    set(LBM_CLASS_B ON CACHE BOOL "")
    set(LBM_CLASS_C ON CACHE BOOL "")
    set(LBM_CSMA ON CACHE BOOL "")
    # set(LBM_CSMA_BY_DEFAULT ON CACHE BOOL "")
    set(LBM_DEVICE_MANAGEMENT ON CACHE BOOL "")
    set(LBM_LFU ON CACHE BOOL "")
    set(LBM_MULTICAST ON CACHE BOOL "")
    # set(LBM_RELAY_RX ON CACHE BOOL "")
    # set(LBM_RELAY_TX ON CACHE BOOL "")
    set(LBM_STREAM ON CACHE BOOL "")

    # set(LBM_BEACON_TX ON CACHE BOOL "")

    set(LBM_FUOTA ON CACHE BOOL "")
    set(LBM_FUOTA_MPA ON CACHE BOOL "")

    if(BOARD STREQUAL NUCLEO_L073)
        set(LBM_ALMANAC OFF CACHE BOOL "")
        set(LBM_STORE_AND_FORWARD OFF CACHE BOOL "")
    else()
        set(LBM_ALMANAC ON CACHE BOOL "")
        set(LBM_GEOLOCATION ON CACHE BOOL "")
        set(LBM_STORE_AND_FORWARD ON CACHE BOOL "")
    endif()
endif()

if(HAL_FLASH_UNAVAILABLE)
    set(LBM_STORE_AND_FORWARD OFF CACHE BOOL "" FORCE)
    set(LBM_GEOLOCATION OFF CACHE BOOL "" FORCE)
endif()

# TODO: How to do that properly?
add_subdirectory(../lbm_lib lbm_lib)

################################################################################
# Example libs and executable

# Those libraries depend on lbm_lib
add_subdirectory(smtc_modem_hal)
add_subdirectory(radio_hal)

add_executable(lbm_example.elf)

if(APP STREQUAL "hw_modem")
    add_subdirectory(hw_modem)
else()
    add_subdirectory(main_examples)
endif()


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

# need for sx127x compilation
if(RADIO_FAMILY STREQUAL sx127x)
    target_link_libraries(smtc_hal PRIVATE ${radio_driver_library})
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
    $<TARGET_OBJECTS:st_hal>
    $<TARGET_OBJECTS:smtc_hal>
)

target_link_libraries(lbm_example.elf PRIVATE
    lora_basics_modem_core
    radio_hal
    smtc_modem_hal_implem
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../")
include(GetGitRevisionDescription)

git_version(GIT_VERSION)
target_compile_definitions(lbm_example.elf PUBLIC -DGIT_VERSION=\"${GIT_VERSION}\")

get_git_head_revision(GIT_REFSPEC GIT_SHA1 "ALLOW_LOOKING_ABOVE_CMAKE_SOURCE_DIR")
target_compile_definitions(lbm_example.elf PUBLIC -DGIT_COMMIT=\"${GIT_SHA1}\")

git_commit_date(GIT_DATE)
target_compile_definitions(lbm_example.elf PUBLIC -DGIT_DATE=\"${GIT_DATE}\")

string(TIMESTAMP BUILD_DATE "%Y-%m-%d %H-%M")
target_compile_definitions(lbm_example.elf PUBLIC -DBUILD_DATE=\"${BUILD_DATE}\")


add_custom_command(
    TARGET lbm_example.elf PRE_LINK
    COMMAND ${CMAKE_SIZE} ${CMAKE_BINARY_DIR}/lbm_lib/smtc_modem_core/liblora_basics_modem_core.a
    COMMENT "Displays library liblora_basics_modem_core.a size details"
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
    # needs stlink-tools
    DEPENDS lbm_example.elf
    COMMAND st-flash --connect-under-reset write ${CMAKE_CURRENT_BINARY_DIR}/lbm_example.bin 0x8000000
    USES_TERMINAL
)

add_custom_target(flash_copy
    DEPENDS lbm_example.elf
    COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/lbm_example.bin /media/$ENV{USER}/NODE_L476RG/.
    USES_TERMINAL
)