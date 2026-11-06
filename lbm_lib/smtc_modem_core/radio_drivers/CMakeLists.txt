# SPDX-License-Identifier: BSD-3-Clause-Clear

if(RADIO_FAMILY STREQUAL "lr11xx" AND LBM_RADIO STREQUAL "lr1121")
    set(LR11XX_DRIVER_GEOLOCATION OFF)
endif()

if(RADIO_FAMILY STREQUAL "sx127x")
    set(ENABLE_${RADIO_UPPER} ON)
endif()


if(RADIO_FAMILY STREQUAL "sx126x" )
    set(SX126X_ENABLE_LR_FHSS ON)
endif()

add_subdirectory(${RADIO_FAMILY}_driver)

set(radio_driver_library "${RADIO_FAMILY}_driver::${RADIO_FAMILY}_driver" CACHE STRING "")
