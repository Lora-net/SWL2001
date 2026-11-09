# SPDX-License-Identifier: BSD-3-Clause-Clear

#-----------------------------------------------------------------------------
# Region selection. They are all enabled by default, unless one is selected via the REGION flag
#-----------------------------------------------------------------------------

set(all_regions AS_923 AU_915 CN_470 CN_470_RP_1_0 EU_868 IN_865 KR_920 RU_864 US_915 WW_2G4)
if("${LBM_REGIONS}" STREQUAL "ALL")
    set(LBM_REGIONS ${all_regions})
endif()

# Ensure only supported regions are in the list
foreach(region IN LISTS LBM_REGIONS)
    if(NOT "${region}" IN_LIST all_regions)
        message(FATAL_ERROR "Unknown region ${region}!")
    endif()
endforeach()


# sx128x only supports 2.4GHz
if(${RADIO_FAMILY} STREQUAL sx128x)
    if(NOT REGIONS STREQUAL "WW_2G4")
        if(LBM_CMAKE_CONFIG_AUTO)
            message(STATUS "The sx128x family only supports 2.4GHz, setting LBM_REGIONS=WW_2G4.")
            set(LBM_REGIONS "WW_2G4")
        else()
            message(FATAL_ERROR "The sx128x family only supports 2.4GHz, please set LBM_REGIONS=WW_2G4")
        endif()
    endif()
endif()

# Only sx128x, lr1120, lr1121, lr2021, lr2022 support 2.4GHz
set(RADIOS_SUPPORT_2G4 sx1280 sx1281 lr1120 lr1121 lr2021 lr2022)
if(NOT ${LBM_RADIO} IN_LIST RADIOS_SUPPORT_2G4)
    message(STATUS "Removing WW_2G4 region from enabled regions list (incompatible radio).")
    list(REMOVE_ITEM LBM_REGIONS WW_2G4)
endif()


if(NOT LBM_REGIONS)
    message(FATAL_ERROR "No region was enabled! Maybe please check compatibility between radio and regions.")
endif()

#-----------------------------------------------------------------------------
# Region sources and defines
#-----------------------------------------------------------------------------

# Add real sources in lora_basics_modem target
target_include_directories(lora_basics_modem PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

target_sources(lora_basics_modem PRIVATE src/smtc_real.c)

foreach(region IN LISTS LBM_REGIONS)
    string(TOLOWER "${region}" region_lower)
    target_sources(lora_basics_modem PRIVATE src/region_${region_lower}.c)
    target_compile_definitions(lora_basics_modem PRIVATE REGION_${region})
    # radio_planner (duty_cycle.c) also depends on the selected regions
    target_compile_definitions(radio_planner PRIVATE REGION_${region})
endforeach()

# Custom case for 2.4GHz
if(WW_2G4 IN_LIST LBM_REGIONS)
    target_compile_definitions(lora_basics_modem PRIVATE WW_2G4_SINGLE_DATARATE)
endif()

#-----------------------------------------------------------------------------
# Regional Parameters Version
#-----------------------------------------------------------------------------

set(RP_VERSION "RP2_103" CACHE STRING "The Regional Parameters version")
set_property(CACHE RP_VERSION PROPERTY STRINGS RP2_101 RP2_103 RP2_104 RP2_105)

target_compile_definitions(lora_basics_modem PRIVATE ${RP_VERSION})
