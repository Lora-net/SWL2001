# SPDX-License-Identifier: BSD-3-Clause-Clear

cmake_minimum_required(VERSION 3.25)

project(lora_basics_modem
    VERSION 4.9.3
    DESCRIPTION "Semtech standard implementation of the LoRaWAN specifications"
    LANGUAGES C
)

set(CMAKE_C_STANDARD 17)
set(CMAKE_LINK_LIBRARIES_ONLY_TARGETS ON)

# From here, the RADIO_FAMILY, RADIO_FAMILY_UPPER and RADIO_UPPER must have been set
# This is typically done when including smtc_radio_interface
if(NOT DEFINED RADIO_FAMILY OR RADIO_FAMILY STREQUAL "" OR NOT DEFINED RADIO_FAMILY_UPPER OR RADIO_FAMILY_UPPER STREQUAL "" OR NOT DEFINED RADIO_UPPER OR RADIO_UPPER STREQUAL "")
    message(FATAL_ERROR "RADIO_FAMILY, RADIO_FAMILY_UPPER or RADIO_UPPER not defined. Exiting.")
endif()

# LBM_RADIO must have been also set
set(LBM_RADIO "" CACHE STRING "The LBM radio to use")
if(NOT DEFINED LBM_RADIO OR LBM_RADIO STREQUAL "")
    message(FATAL_ERROR "LBM_RADIO isn't defined. "
                        "Please define it before entering this file.")
endif()

message(STATUS "lbm_lib - LBM_RADIO = ${LBM_RADIO}, "
                "RADIO_FAMILY = ${RADIO_FAMILY}, "
                "RADIO_FAMILY_UPPER = ${RADIO_FAMILY_UPPER}, "
                "RADIO_UPPER = ${RADIO_UPPER}"
        )

include(options.cmake)

set(LORA_BASICS_MODEM_DIR ${CMAKE_CURRENT_LIST_DIR})

add_subdirectory(smtc_modem_hal)
add_subdirectory(smtc_modem_api)
add_subdirectory(smtc_modem_core)

# The user links to the "lora_basics_modem" target

# if(RADIO_FAMILY STREQUAL "lr20xx")
# add_library(lora_basics_modem_combined STATIC)
# target_sources(lora_basics_modem_combined PUBLIC
#         $<TARGET_OBJECTS:lr20xx_driver_lora>
#         $<TARGET_OBJECTS:lr20xx_driver_fsk>
#         $<TARGET_OBJECTS:lr20xx_driver_flrc>
#         $<TARGET_OBJECTS:lr20xx_driver_lr_fhss>
#     )
# endif()

# This is the public library
# add_library(LoRaBasicsModemApi INTERFACE)
# target_include_directories(LoRaBasicsModemApi INTERFACE
#     ${LORA_BASICS_MODEM_DIR}/smtc_modem_api
#     # ${LORA_BASICS_MODEM_DIR}/smtc_modem_core/logging 
# )

# target_link_libraries(LoRaBasicsModemApi INTERFACE lora_basics_modem)


# add_library(LoRaBasicsModem::LoRaBasicsModem ALIAS LoRaBasicsModemApi)
# install(TARGETS LoRaBasicsModemApi
#     EXPORT lora_basics_modem_targets
#     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
#     INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
# )

# export(TARGETS LoRaBasicsModemApi
#     FILE LoRaBasicsModemConfig.cmake
#     NAMESPACE LoRaBasicsModem::

# )

# install(EXPORT lora_basics_modem_targets
#     FILE LoRaBasicsModemConfig.cmake
#     NAMESPACE LoRaBasicsModem::
#     DESTINATION lib/cmake/MyLib
# )
