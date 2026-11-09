# SPDX-License-Identifier: BSD-3-Clause-Clear

cmake_minimum_required(VERSION 3.25)

# The user must have set SMTC_RADIO

# Set target radio
set(sx127x_radios sx1272 sx1276)
set(sx126x_radios sx1261 sx1262 sx1268)
set(sx128x_radios sx1280 sx1281)
set(lr11xx_radios lr1110 lr1120 lr1121)
set(lr20xx_radios lr2012 lr2021 lr2022)
set(udp_pf_radios udp_pf CACHE INTERNAL "UDP packet forwarder (virtual radio)")
set(known_radios ${sx127x_radios} ${sx126x_radios} ${sx128x_radios} ${lr11xx_radios} ${lr20xx_radios} ${udp_pf_radios})

set(SMTC_RADIO "" CACHE STRING "The Semtech radio to use" FORCE)
set_property(CACHE SMTC_RADIO PROPERTY STRINGS ${known_radios})
if((NOT DEFINED SMTC_RADIO) OR (SMTC_RADIO STREQUAL ""))
    message(FATAL_ERROR "The SMTC_RADIO CMake variable must be set to a known radio.\n"
        "Available radios are: ${known_radios}\n"
        "Please check that top-calling CMakeFile properly sets the SMTC_RADIO variable.\n"
        "Typically, it will be with -DLBM_RADIO=lr2021 or -DRAC_RADIO=lr2021")
endif()
if(NOT SMTC_RADIO IN_LIST known_radios)
    message(FATAL_ERROR "Unknown radio ${SMTC_RADIO}")
endif()

# Set target radio family
if(SMTC_RADIO IN_LIST sx126x_radios)
    set(RADIO_FAMILY sx126x)
elseif(SMTC_RADIO IN_LIST sx127x_radios)
    set(RADIO_FAMILY sx127x)
elseif(SMTC_RADIO IN_LIST sx128x_radios)
    set(RADIO_FAMILY sx128x)
elseif(SMTC_RADIO IN_LIST lr11xx_radios)
    set(RADIO_FAMILY lr11xx)
elseif(SMTC_RADIO IN_LIST lr20xx_radios)
    set(RADIO_FAMILY lr20xx)
elseif(RAC_RADIO IN_LIST udp_pf_radios)
    set(RADIO_FAMILY udp_pf)
else()
    message(FATAL_ERROR "Error in cmake logic, radio family not found!")
endif()

message("")
message(STATUS "Selected radio ${SMTC_RADIO} in family ${RADIO_FAMILY}\n")

string(TOUPPER ${SMTC_RADIO} RADIO_UPPER)
string(TOUPPER ${RADIO_FAMILY} RADIO_FAMILY_UPPER)

# Report RADIO_FAMILY up as it might be used for special linking 
#if(NOT PROJECT_IS_TOP_LEVEL)
    set(RADIO_FAMILY ${RADIO_FAMILY} PARENT_SCOPE)
    set(RADIO_UPPER ${RADIO_UPPER} PARENT_SCOPE)
    set(RADIO_FAMILY_UPPER ${RADIO_FAMILY_UPPER} PARENT_SCOPE)
#endif()

# Report ral/ralf pathes to the calling, so that they may link/include directories
# TODO : directly the include folder ?
#if(NOT PROJECT_IS_TOP_LEVEL)
    set(SMTC_RAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/smtc_ral" PARENT_SCOPE)
    set(SMTC_RALF_PATH "${CMAKE_CURRENT_SOURCE_DIR}/smtc_ralf" PARENT_SCOPE)
#endif()

# Build radio drivers/RAL/RALF according to RADIO_FAMILY

# If RADIO_DRV_EXT_PATH is externally set, use this one instead of current dir one
# Please note than, when using external radio driver folder, this folder needs to expose radio_driver_library.
set(RADIO_DRV_EXT_PATH "" CACHE PATH "External low-level path for radio drivers")
if(RADIO_DRV_EXT_PATH)
    set(SMTC_RADIO_PATH_LOCAL ${RADIO_DRV_EXT_PATH})
else()
    set(SMTC_RADIO_PATH_LOCAL "${CMAKE_CURRENT_SOURCE_DIR}/radio_drivers")
endif()
#if(NOT PROJECT_IS_TOP_LEVEL)
    set(SMTC_RADIO_DRIVERS_PATH ${SMTC_RADIO_PATH_LOCAL} PARENT_SCOPE) # Set to parent scope for linking/include
#endif()
# message(STATUS "Radio driver path: '${SMTC_RADIO_PATH_LOCAL}'.")
add_subdirectory("${SMTC_RADIO_PATH_LOCAL}" radio_drivers)

add_subdirectory(smtc_ral)
# Link driver only if not UDP virtual radio (which has no real driver)
if(NOT ${RADIO_FAMILY} STREQUAL "udp_pf")
    target_link_libraries(smtc_ral_${RADIO_FAMILY} PUBLIC ${radio_driver_library})
endif()

# Direct dispatch: RAL/RALF call ral_<family>_*/ralf_<family>_* directly instead of going through function pointers.
# The defines change the ral_t/ralf_t layout, so they are PUBLIC to reach every target that includes ral.h/ralf.h.
option(RAL_DRIVER_DIRECT_DISPATCH "RAL/RALF call the radio family driver directly instead of through function pointers" ON)
if(RAL_DRIVER_DIRECT_DISPATCH)
    if(${RADIO_FAMILY} STREQUAL "udp_pf")
        message(FATAL_ERROR "RAL_DRIVER_DIRECT_DISPATCH is not supported for udp_pf")
    endif()
    target_compile_definitions(smtc_ral_${RADIO_FAMILY} PUBLIC
        RAL_DRIVER_DIRECT_DISPATCH
        RAL_DRIVER_DIRECT_DISPATCH_${RADIO_FAMILY_UPPER}
    )
endif()

# Special links for LR20xx - TODO add variables upper to link only when necessary
# E.G when higher levels use LoRa and FLRC, use LR20XX_LORA and LR20XX_FLRC and link only necessary libs here
if(${RADIO_FAMILY} MATCHES "lr20xx")
    target_link_libraries(smtc_ral_lr20xx PUBLIC
        lr20xx_driver::lr20xx_driver_rttof
        lr20xx_driver::lr20xx_driver_lora
        lr20xx_driver::lr20xx_driver_fsk
        lr20xx_driver::lr20xx_driver_lr_fhss
        lr20xx_driver::lr20xx_driver_flrc
        lr20xx_driver::lr20xx_driver_bluetooth_le
    )
endif()

add_subdirectory(smtc_ralf)

target_link_libraries(smtc_ralf_${RADIO_FAMILY} PUBLIC smtc_ral_${RADIO_FAMILY})
