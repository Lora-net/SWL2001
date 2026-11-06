# SPDX-License-Identifier: BSD-3-Clause-Clear

cmake_minimum_required(VERSION 3.25)

project(lora_basics_modem
    VERSION 4.9.0
    DESCRIPTION "Semtech standard implementation of the LoRaWAN specifications"
    LANGUAGES C
)

set(CMAKE_C_STANDARD 17)

set(CMAKE_LINK_LIBRARIES_ONLY_TARGETS ON)

# Set target radio
set(sx126x_radios sx1261 sx1262 sx1268)
set(sx127x_radios sx1272 sx1276)
set(sx128x_radios sx1280 sx1281)
set(lr11xx_radios lr1110 lr1120 lr1121)
set(lr20xx_radios lr2021)
set(known_radios ${sx126x_radios} ${sx127x_radios} ${sx128x_radios} ${lr11xx_radios} ${lr20xx_radios})

set(LBM_RADIO "" CACHE STRING "The radio to target")
set_property(CACHE LBM_RADIO PROPERTY STRINGS ${known_radios})
if(NOT DEFINED LBM_RADIO OR LBM_RADIO STREQUAL "")
    message(FATAL_ERROR "You must select the radio with -DLBM_RADIO=\n"
        "Available radios are: ${known_radios}\n"
        "Example: -DLBM_RADIO=sx1261")
endif()
if(NOT LBM_RADIO IN_LIST known_radios)
    message(FATAL_ERROR "Unknown radio ${LBM_RADIO}")
endif()

# Set target radio family
if(LBM_RADIO IN_LIST sx126x_radios)
    set(RADIO_FAMILY sx126x)
elseif(LBM_RADIO IN_LIST sx127x_radios)
    set(RADIO_FAMILY sx127x)
elseif(LBM_RADIO IN_LIST sx128x_radios)
    set(RADIO_FAMILY sx128x)
elseif(LBM_RADIO IN_LIST lr11xx_radios)
    set(RADIO_FAMILY lr11xx)
elseif(LBM_RADIO IN_LIST lr20xx_radios)
    set(RADIO_FAMILY lr20xx)
else()
    message(FATAL_ERROR "Error in cmake logic, radio family not found!")
endif()

message("")
message(STATUS "Selected radio ${LBM_RADIO} in family ${RADIO_FAMILY}\n")

# Report the stuff to the parent scope, calling project etc
if(NOT PROJECT_IS_TOP_LEVEL)
    set(LBM_RADIO ${LBM_RADIO} PARENT_SCOPE)
    set(RADIO_FAMILY ${RADIO_FAMILY} PARENT_SCOPE)
endif()

string(TOUPPER ${LBM_RADIO} RADIO_UPPER)
string(TOUPPER ${RADIO_FAMILY} RADIO_FAMILY_UPPER)

include(options.cmake)

set(LORA_BASICS_MODEM_DIR ${CMAKE_CURRENT_LIST_DIR})

add_subdirectory(smtc_modem_api)
add_subdirectory(smtc_modem_core)
add_subdirectory(smtc_modem_hal)

# This library contains all objects from all libraries under lbm_lib
add_library(lora_basics_modem_combined EXCLUDE_FROM_ALL
    $<TARGET_OBJECTS:${radio_driver_library}>
    $<TARGET_OBJECTS:smtc_modem_crypto>
    $<TARGET_OBJECTS:smtc_ral::smtc_ral_${RADIO_FAMILY}>
    $<TARGET_OBJECTS:smtc_ralf::smtc_ralf_${RADIO_FAMILY}>
    $<TARGET_OBJECTS:lora_basics_modem_core>
)

if(RADIO_FAMILY STREQUAL "lr20xx")
target_sources(lora_basics_modem_combined PUBLIC
        $<TARGET_OBJECTS:lr20xx_driver_lora>
        $<TARGET_OBJECTS:lr20xx_driver_fsk>
        $<TARGET_OBJECTS:lr20xx_driver_flrc>
        $<TARGET_OBJECTS:lr20xx_driver_lr_fhss>
        $<TARGET_OBJECTS:lr20xx_driver_rttof>
    )
endif()

# This is the public library
add_library(LoRaBasicsModemApi INTERFACE)
target_include_directories(LoRaBasicsModemApi INTERFACE
    ${LORA_BASICS_MODEM_DIR}/smtc_modem_api
    ${LORA_BASICS_MODEM_DIR}/smtc_modem_core/logging
    ${LORA_BASICS_MODEM_DIR}/smtc_modem_hal
)

# target_link_libraries(LoRaBasicsModemApi INTERFACE lora_basics_modem_core)


add_library(LoRaBasicsModem::LoRaBasicsModem ALIAS LoRaBasicsModemApi)
install(TARGETS LoRaBasicsModemApi
    EXPORT lora_basics_modem_targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
