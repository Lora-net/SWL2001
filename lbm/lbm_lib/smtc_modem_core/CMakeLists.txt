# SPDX-License-Identifier: BSD-3-Clause-Clear

# The list of internal include paths required to build the different parts of smtc_modem_core
set(SMTC_MODEM_CORE_INCLUDE_DIRS
    ${LORA_BASICS_MODEM_DIR}
    ${CMAKE_CURRENT_LIST_DIR}
    # ${CMAKE_CURRENT_LIST_DIR}/logging
    ${CMAKE_CURRENT_LIST_DIR}/modem_supervisor
    ${CMAKE_CURRENT_LIST_DIR}/modem_utilities
    ${CMAKE_CURRENT_LIST_DIR}/lorawan_packages
    ${CMAKE_CURRENT_LIST_DIR}/lorawan_packages/lorawan_certification
    ${CMAKE_CURRENT_LIST_DIR}/lorawan_manager
    ${CMAKE_CURRENT_LIST_DIR}/lorawan_api

    ${CMAKE_CURRENT_LIST_DIR}/lr1mac
    ${CMAKE_CURRENT_LIST_DIR}/lr1mac/src
    ${CMAKE_CURRENT_LIST_DIR}/lr1mac/src/services
  
    ${SMTC_RADIO_PLANNER_PATH}/src
    ${SMTC_RAL_PATH}/src
    ${SMTC_RALF_PATH}/src
    ${SMTC_RADIO_DRIVERS_PATH}
    
    ${CMAKE_CURRENT_LIST_DIR}/smtc_modem_crypto
    ${CMAKE_CURRENT_LIST_DIR}/smtc_modem_crypto/smtc_secure_element
)

# Logging library
# add_library(smtc_modem_logging INTERFACE)
# target_include_directories(smtc_modem_logging INTERFACE ${CMAKE_CURRENT_LIST_DIR}/logging)



# add_subdirectory(smtc_modem_crypto)
include(smtc_modem_crypto.cmake)

add_library(lora_basics_modem STATIC)
target_compile_definitions(lora_basics_modem PRIVATE USE_LBM)

# Interface library - This one is used when upper levers (e.g Radio Planner) needs properties defined in LBM
# to build. 
add_library(lora_basics_modem_external_props INTERFACE)
target_compile_definitions(lora_basics_modem_external_props INTERFACE USE_LBM)

#target_link_libraries(lora_basics_modem PUBLIC smtc_ralf::smtc_ralf_${RADIO_FAMILY})

add_subdirectory(lr1mac/src/smtc_real)

target_sources(lora_basics_modem PRIVATE
    lorawan_api/lorawan_api.c
    lorawan_manager/lorawan_cid_request_management.c
    lorawan_manager/lorawan_dwn_ack_management.c
    lorawan_manager/lorawan_join_management.c
    lorawan_manager/lorawan_send_management.c
    lorawan_packages/lorawan_certification/lorawan_certification.c
    lr1mac/src/lr1_stack_mac_layer.c
    lr1mac/src/lr1mac_core.c
    lr1mac/src/lr1mac_utilities.c
    lr1mac/src/services/smtc_duty_cycle.c
    lr1mac/src/services/smtc_lbt.c
    modem_supervisor/modem_supervisor_light.c
    modem_supervisor/modem_tx_protocol_manager.c
    modem_utilities/fifo_ctrl.c
    modem_utilities/modem_core.c
    modem_utilities/modem_event_utilities.c

    smtc_modem_test.c
    smtc_modem.c
)

target_include_directories(lora_basics_modem PRIVATE ${SMTC_MODEM_CORE_INCLUDE_DIRS})

# API - PUBLIC
target_include_directories(lora_basics_modem PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../smtc_modem_api>
    $<INSTALL_INTERFACE:include>
)

# 
target_link_libraries(lora_basics_modem PRIVATE
    smtc_modem_crypto
    radio_planner
    # smtc_modem_hal_interface
)

target_link_libraries(lora_basics_modem PUBLIC
    smtc_ralf::smtc_ralf_${RADIO_FAMILY}
    smtc_ral::smtc_ral_${RADIO_FAMILY}
    smtc_modem_hal_interface
)

# Link driver only if not UDP virtual radio (which has no real driver)
if(NOT RADIO_FAMILY STREQUAL "udp_pf")
    target_link_libraries(lora_basics_modem PRIVATE
        ${RADIO_FAMILY}_driver
    )
endif()

target_compile_definitions(lora_basics_modem PRIVATE ${RADIO_UPPER} ${RADIO_FAMILY_UPPER})

# Add definition for LBM legacy contextes handling
if(LBM_LEGACY)
    target_compile_definitions(lora_basics_modem PRIVATE MODEM_RP_RALF_RAL_INTERNAL)
endif()

if(RADIO_FAMILY STREQUAL lr11xx)
    # This define is for LR11XX **outside** modem-e
    target_compile_definitions(lora_basics_modem PRIVATE LR11XX_TRANSCEIVER)
endif()

if(RADIO_FAMILY STREQUAL lr20xx)
    target_link_libraries(lora_basics_modem PRIVATE
        lr20xx_driver_lora
        lr20xx_driver_fsk
        lr20xx_driver_flrc
        lr20xx_driver_lr_fhss
    )

    # TODO : move following line to RAL CMakeLists.txt
    target_link_libraries(smtc_ral_${RADIO_FAMILY} PRIVATE
        lr20xx_driver_lora
        lr20xx_driver_fsk
        lr20xx_driver_flrc
        lr20xx_driver_lr_fhss
    )
endif()

# Link radio_planner to interface lib (propagate compile definitions)
# PUBLIC: consumers of radio_planner (smtc_rac, smtc_flrc_protocol, ...) must see the
# same ADD_CLASS_B/ADD_CSMA/ADD_RELAY_RX/ADD_RELAY_TX/ADD_CLASS_C defines as radio_planner
# itself, since they share the radio_planner_hook_id_defs.h enum whose values depend on them.
target_link_libraries(radio_planner PUBLIC lora_basics_modem_external_props)
# LBM also uses

################################################################################
# Compile definitions
if(LBM_CRYPTO MATCHES "^LR11XX")
    target_compile_definitions(lora_basics_modem PRIVATE USE_LR11XX_CE)
endif()

if(LBM_MODEM_TRACE)
    target_compile_definitions(lora_basics_modem PUBLIC MODEM_HAL_DBG_TRACE=1)
    target_compile_definitions(smtc_modem_crypto PUBLIC MODEM_HAL_DBG_TRACE=1)
else()
    target_compile_definitions(lora_basics_modem PUBLIC MODEM_HAL_DBG_TRACE=0)
    target_compile_definitions(smtc_modem_crypto PUBLIC MODEM_HAL_DBG_TRACE=0)
endif()

if(LBM_MODEM_TRACE AND LBM_MODEM_TRACE_DEEP)
    target_compile_definitions(lora_basics_modem PUBLIC MODEM_HAL_DEEP_DBG_TRACE=1)
    target_compile_definitions(smtc_modem_crypto PUBLIC MODEM_HAL_DEEP_DBG_TRACE=1)
else()
    target_compile_definitions(lora_basics_modem PUBLIC MODEM_HAL_DEEP_DBG_TRACE=0)
    target_compile_definitions(smtc_modem_crypto PUBLIC MODEM_HAL_DEEP_DBG_TRACE=0)
endif()

target_compile_definitions(lora_basics_modem PRIVATE NUMBER_OF_STACKS=${LBM_NUMBER_OF_STACKS})

# RP_MARGIN_DELAY can be overridden from parent cmake or command line
# Commented because not needed, as smtc_modem_core links radio_planner that target_compile_defs(RP_MARGIN_DELAY) as PUBLIC
# if(NOT DEFINED RP_MARGIN_DELAY)
#     set(RP_MARGIN_DELAY "8")
# endif()
# target_compile_definitions(lora_basics_modem PRIVATE RP_MARGIN_DELAY=${RP_MARGIN_DELAY})

if(LBM_CLASS_B)
    target_compile_definitions(lora_basics_modem PUBLIC ADD_CLASS_B)
    target_compile_definitions(lora_basics_modem_external_props INTERFACE ADD_CLASS_B)
    target_include_directories(lora_basics_modem PRIVATE
        lr1mac/src/lr1mac_class_b
    )
    target_sources(lora_basics_modem PRIVATE
        lorawan_manager/lorawan_class_b_management.c
        lr1mac/src/lr1mac_class_b/smtc_beacon_sniff.c
        lr1mac/src/lr1mac_class_b/smtc_ping_slot.c
    )
endif()

if(LBM_CLASS_C)
    message(STATUS "Adding CLASS_C")
    target_compile_definitions(lora_basics_modem PUBLIC ADD_CLASS_C)
    target_compile_definitions(lora_basics_modem_external_props INTERFACE ADD_CLASS_C)
    target_include_directories(lora_basics_modem PRIVATE
        lr1mac/src/lr1mac_class_c
    )
    target_sources(lora_basics_modem PRIVATE
        lr1mac/src/lr1mac_class_c/lr1mac_class_c.c
    )
endif()

if(LBM_MULTICAST)
    target_compile_definitions(lora_basics_modem PUBLIC SMTC_MULTICAST)
    target_include_directories(lora_basics_modem PRIVATE
        lr1mac/src/services/smtc_multicast
    )
    target_sources(lora_basics_modem PRIVATE
        lr1mac/src/services/smtc_multicast/smtc_multicast.c
    )
endif()

if(LBM_FUOTA AND LBM_MULTICAST)
    set(FUOTA_MULTICAST_SOURCES_V1 lorawan_packages/remote_multicast_setup/v1.0.0/lorawan_remote_multicast_setup_package_v1.0.0.c)
    set(FUOTA_MULTICAST_SOURCES_V2 lorawan_packages/remote_multicast_setup/v2.0.0/lorawan_remote_multicast_setup_package_v2.0.0.c)
    target_sources(lora_basics_modem PRIVATE
        ${FUOTA_MULTICAST_SOURCES_V${LBM_FUOTA_VERSION}}
    )
endif()

if(LBM_CSMA)
    target_compile_definitions(lora_basics_modem PUBLIC ADD_CSMA)
    target_compile_definitions(lora_basics_modem_external_props INTERFACE ADD_CSMA)

    if(LBM_CSMA_BY_DEFAULT)
        target_compile_definitions(lora_basics_modem PRIVATE ENABLE_CSMA_BY_DEFAULT)
        target_compile_definitions(lora_basics_modem_external_props INTERFACE ENABLE_CSMA_BY_DEFAULT)
    endif()

    target_sources(lora_basics_modem PRIVATE
        lr1mac/src/services/smtc_lora_cad_bt.c
    )
endif()

if(LBM_ALC_SYNC)
    target_compile_definitions(lora_basics_modem PUBLIC ADD_SMTC_ALC_SYNC)
    target_compile_definitions(lora_basics_modem_external_props INTERFACE ADD_SMTC_ALC_SYNC)
    target_include_directories(lora_basics_modem PRIVATE
        lorawan_packages/application_layer_clock_synchronization
    )
    set(ALC_SYNC_SOURCES_V1 lorawan_packages/application_layer_clock_synchronization/v1.0.0/lorawan_alcsync_v1.0.0.c)
    set(ALC_SYNC_SOURCES_V2 lorawan_packages/application_layer_clock_synchronization/v2.0.0/lorawan_alcsync_v2.0.0.c)
    target_sources(lora_basics_modem PRIVATE
        ${ALC_SYNC_SOURCES_V${LBM_ALC_SYNC_VERSION}}
    )
endif()

if(LBM_FUOTA)
    target_compile_definitions(lora_basics_modem PUBLIC ADD_FUOTA=${LBM_FUOTA_VERSION})
    target_compile_definitions(lora_basics_modem_external_props INTERFACE ADD_FUOTA=${LBM_FUOTA_VERSION})
    set(FUOTA_INCLUDE_V1
        lorawan_packages/fragmented_data_block_transport/v1.0.0
    )
    set(FUOTA_INCLUDE_V2
        lorawan_packages/fragmented_data_block_transport/v2.0.0
    )
    target_include_directories(lora_basics_modem PRIVATE
        lorawan_packages/application_layer_clock_synchronization
        lorawan_packages/fragmented_data_block_transport
        lorawan_packages/remote_multicast_setup
        ${FUOTA_INCLUDE_V${LBM_FUOTA_VERSION}}
    )

    set(FUOTA_SOURCES_V1
        lorawan_packages/fragmented_data_block_transport/v1.0.0/fragmentation_helper_v1.0.0.c
        lorawan_packages/fragmented_data_block_transport/v1.0.0/lorawan_fragmentation_package_v1.0.0.c
    )
    set(FUOTA_SOURCES_V2
        lorawan_packages/fragmented_data_block_transport/v2.0.0/fragmentation_helper_v2.0.0.c
        lorawan_packages/fragmented_data_block_transport/v2.0.0/lorawan_fragmentation_package_v2.0.0.c
    )
    target_sources(lora_basics_modem PRIVATE
        ${FUOTA_SOURCES_V${LBM_FUOTA_VERSION}}
    )

    if(LBM_FUOTA_FMP)
        target_compile_definitions(lora_basics_modem PUBLIC ENABLE_FUOTA_FMP)
        target_compile_definitions(lora_basics_modem_external_props INTERFACE ENABLE_FUOTA_FMP)
        target_include_directories(lora_basics_modem PRIVATE
            lorawan_packages/firmware_management_protocol
        )
        target_sources(lora_basics_modem PRIVATE
            lorawan_packages/firmware_management_protocol/lorawan_fmp_package.c
        )
    endif()

    if(LBM_FUOTA_MPA)
        target_compile_definitions(lora_basics_modem PUBLIC ENABLE_FUOTA_MPA)
        target_compile_definitions(lora_basics_modem_external_props INTERFACE ENABLE_FUOTA_MPA)
        target_include_directories(lora_basics_modem PRIVATE
            lorawan_packages/multi_package_access
        )
        target_sources(lora_basics_modem PRIVATE
            lorawan_packages/multi_package_access/lorawan_mpa_package.c
        )
    endif()

    if(NOT "${LBM_FUOTA_FRAGMENTS_MAX_NUM}" STREQUAL "")
        target_compile_definitions(lora_basics_modem PUBLIC
            FRAG_MAX_NB=${LBM_FUOTA_FRAGMENTS_MAX_NUM}
        )
        target_compile_definitions(lora_basics_modem_external_props INTERFACE 
            FRAG_MAX_NB=${LBM_FUOTA_FRAGMENTS_MAX_NUM}
        )
    endif()

    if(NOT "${LBM_FUOTA_FRAGMENTS_MAX_SIZE}" STREQUAL "")
        target_compile_definitions(lora_basics_modem PUBLIC
            FRAG_MAX_SIZE=${LBM_FUOTA_FRAGMENTS_MAX_SIZE}
        )
        target_compile_definitions(lora_basics_modem_external_props INTERFACE
            FRAG_MAX_SIZE=${LBM_FUOTA_FRAGMENTS_MAX_SIZE}
        )
    endif()

    if(NOT "${LBM_FUOTA_FRAGMENTS_MAX_REDUNDANCY}" STREQUAL "")
        target_compile_definitions(lora_basics_modem PUBLIC
            FRAG_MAX_REDUNDANCY=${LBM_FUOTA_FRAGMENTS_MAX_REDUNDANCY}
        )
        target_compile_definitions(lora_basics_modem_external_props INTERFACE
            FRAG_MAX_REDUNDANCY=${LBM_FUOTA_FRAGMENTS_MAX_REDUNDANCY}
        )
    endif()
endif()

if(LBM_ALMANAC)
    target_compile_definitions(lora_basics_modem PUBLIC ADD_ALMANAC)
    target_compile_definitions(lora_basics_modem_external_props INTERFACE ADD_ALMANAC)
    target_include_directories(lora_basics_modem PRIVATE
        modem_services
        modem_services/almanac_packages
    )
    target_sources(lora_basics_modem PRIVATE
        modem_services/almanac_packages/almanac.c
    )
endif()

if(LBM_STREAM)
    target_compile_definitions(lora_basics_modem PUBLIC ADD_SMTC_STREAM)
    target_compile_definitions(lora_basics_modem_external_props INTERFACE ADD_SMTC_STREAM)
    target_include_directories(lora_basics_modem PRIVATE
        modem_services
        modem_services/stream_packages
    )
    target_sources(lora_basics_modem PRIVATE
        modem_services/stream_packages/rose.c
        modem_services/stream_packages/stream.c
    )
endif()

if(LBM_LFU)
    target_compile_definitions(lora_basics_modem PUBLIC ADD_SMTC_LFU)
    target_compile_definitions(lora_basics_modem_external_props INTERFACE ADD_SMTC_LFU)
    target_include_directories(lora_basics_modem PRIVATE
        modem_services
        modem_services/lfu_service
    )
    target_sources(lora_basics_modem PRIVATE
        modem_services/lfu_service/file_upload.c
    )
endif()

if(LBM_DEVICE_MANAGEMENT)
    target_compile_definitions(lora_basics_modem PUBLIC ADD_SMTC_CLOUD_DEVICE_MANAGEMENT)
    target_compile_definitions(lora_basics_modem_external_props INTERFACE ADD_SMTC_CLOUD_DEVICE_MANAGEMENT)
    target_include_directories(lora_basics_modem PRIVATE
        modem_services
        modem_services/cloud_dm_package
    )
    target_sources(lora_basics_modem PRIVATE
        modem_services/cloud_dm_package/cloud_dm_package.c
    )
endif()

if(LBM_GEOLOCATION)
    target_compile_definitions(lora_basics_modem PUBLIC ADD_LBM_GEOLOCATION)
    target_compile_definitions(lora_basics_modem_external_props INTERFACE ADD_LBM_GEOLOCATION)
    target_include_directories(lora_basics_modem PUBLIC
        geolocation_services
    )
    target_sources(lora_basics_modem PRIVATE
        geolocation_services/gnss_helpers.c
        geolocation_services/mw_common.c
        geolocation_services/mw_gnss_almanac.c
        geolocation_services/mw_gnss_almanac_full_update.c
        geolocation_services/mw_gnss_scan.c
        geolocation_services/mw_gnss_send.c
        geolocation_services/mw_wifi_scan.c
        geolocation_services/mw_wifi_send.c
        geolocation_services/wifi_helpers.c
    )
endif()

if(LBM_STORE_AND_FORWARD)
    target_compile_definitions(lora_basics_modem PUBLIC ADD_SMTC_STORE_AND_FORWARD)
    target_compile_definitions(lora_basics_modem_external_props INTERFACE ADD_SMTC_STORE_AND_FORWARD)
    target_include_directories(lora_basics_modem PRIVATE
        modem_services
        modem_services/store_and_forward
    )
    target_sources(lora_basics_modem PRIVATE
        modem_utilities/circularfs.c
        modem_services/store_and_forward/store_and_forward_flash.c
    )
endif()

if(LBM_RELAY_RX)
    target_compile_definitions(lora_basics_modem PUBLIC ADD_RELAY_RX)
    target_compile_definitions(lora_basics_modem_external_props INTERFACE ADD_RELAY_RX)
    target_include_directories(lora_basics_modem PRIVATE
        lr1mac/src/relay/common
        lr1mac/src/relay/relay_rx
        lr1mac/src/relay/relay_tx
        modem_services/relay_service
    )
    target_sources(lora_basics_modem PRIVATE
        lr1mac/src/relay/common/relay_mac_parser.c
        lr1mac/src/relay/common/relay_real.c
        lr1mac/src/relay/common/wake_on_radio_ral.c
        lr1mac/src/relay/common/wake_on_radio.c
        lr1mac/src/relay/relay_rx/relay_rx_mac_parser.c
        lr1mac/src/relay/relay_rx/relay_rx.c
        modem_services/relay_service/lorawan_relay_rx_service.c
    )
endif()

if(LBM_RELAY_TX)
    target_compile_definitions(lora_basics_modem PUBLIC ADD_RELAY_TX)
    target_compile_definitions(lora_basics_modem_external_props INTERFACE ADD_RELAY_TX)
    target_include_directories(lora_basics_modem PRIVATE
        lr1mac/src/relay/common
        lr1mac/src/relay/relay_rx
        lr1mac/src/relay/relay_tx
        modem_services/relay_service
    )
    target_sources(lora_basics_modem PRIVATE
        lr1mac/src/relay/common/relay_mac_parser.c
        lr1mac/src/relay/common/relay_real.c
        lr1mac/src/relay/common/wake_on_radio_ral.c
        lr1mac/src/relay/common/wake_on_radio.c
        lr1mac/src/relay/relay_tx/relay_tx_mac_parser.c
        lr1mac/src/relay/relay_tx/relay_tx.c
        modem_services/relay_service/lorawan_relay_tx_service.c
    )
endif()

if(LBM_BEACON_TX)
    target_compile_definitions(lora_basics_modem PUBLIC MODEM_BEACON_APP)
    target_compile_definitions(lora_basics_modem_external_props INTERFACE MODEM_BEACON_APP)
    target_include_directories(lora_basics_modem PRIVATE
        modem_services/beacon_tx_service
    )
    target_sources(lora_basics_modem PRIVATE
        modem_services/beacon_tx_service/lorawan_beacon_tx_service_example.c
    )
endif()

if(LBM_PERF_TEST)
    target_compile_definitions(lora_basics_modem PUBLIC PERF_TEST_ENABLED)
    target_compile_definitions(lora_basics_modem_external_props INTERFACE PERF_TEST_ENABLED)
endif()

if(LBM_TEST_BYPASS_JOIN_DUTY_CYCLES)
    target_compile_definitions(lora_basics_modem PRIVATE TEST_BYPASS_JOIN_DUTY_CYCLE)
endif()
