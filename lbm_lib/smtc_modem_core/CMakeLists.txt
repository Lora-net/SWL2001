# SPDX-License-Identifier: BSD-3-Clause-Clear

# The list of internal include paths required to build the different parts of smtc_modem_core
set(SMTC_MODEM_CORE_INCLUDE_DIRS
    ${LORA_BASICS_MODEM_DIR}
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/modem_supervisor
    ${CMAKE_CURRENT_LIST_DIR}/modem_utilities
    ${CMAKE_CURRENT_LIST_DIR}/lorawan_packages/lorawan_certification
    ${CMAKE_CURRENT_LIST_DIR}/lorawan_manager
    ${CMAKE_CURRENT_LIST_DIR}/lorawan_api
    ${CMAKE_CURRENT_LIST_DIR}/lr1mac
    ${CMAKE_CURRENT_LIST_DIR}/lr1mac/src
    ${CMAKE_CURRENT_LIST_DIR}/lr1mac/src/services
    ${CMAKE_CURRENT_LIST_DIR}/lr1mac/src/smtc_real/src
    ${CMAKE_CURRENT_LIST_DIR}/radio_planner/src
    ${CMAKE_CURRENT_LIST_DIR}/smtc_modem_crypto
    ${CMAKE_CURRENT_LIST_DIR}/smtc_modem_crypto/smtc_secure_element
)

# Logging library
add_library(smtc_modem_logging INTERFACE)
target_include_directories(smtc_modem_logging INTERFACE ${CMAKE_CURRENT_LIST_DIR}/logging)

add_subdirectory(radio_drivers)

add_subdirectory(smtc_ral)
target_link_libraries(smtc_ral_${RADIO_FAMILY} PUBLIC ${radio_driver_library})

add_subdirectory(smtc_ralf)
target_link_libraries(smtc_ralf_${RADIO_FAMILY} PRIVATE smtc_ral_${RADIO_FAMILY})

# add_subdirectory(smtc_modem_crypto)
include(smtc_modem_crypto.cmake)

add_library(lora_basics_modem_core STATIC)
target_link_libraries(lora_basics_modem_core PUBLIC smtc_ralf::smtc_ralf_${RADIO_FAMILY})

add_subdirectory(lr1mac/src/smtc_real)

target_sources(lora_basics_modem_core PRIVATE
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
    radio_planner/src/radio_planner.c
    smtc_modem_test.c
    smtc_modem.c
)

target_include_directories(lora_basics_modem_core PRIVATE ${SMTC_MODEM_CORE_INCLUDE_DIRS})

target_link_libraries(lora_basics_modem_core PRIVATE
    smtc_modem_logging
    smtc_modem_crypto
    smtc_ral::smtc_ral_${RADIO_FAMILY}
    smtc_ralf::smtc_ralf_${RADIO_FAMILY}
    smtc_modem_api_interface
    smtc_modem_hal_interface
)

target_compile_definitions(lora_basics_modem_core PRIVATE ${RADIO_UPPER} ${RADIO_FAMILY_UPPER})

if(RADIO_FAMILY STREQUAL lr11xx)
    # This define is for LR11XX **outside** modem-e
    target_compile_definitions(lora_basics_modem_core PRIVATE LR11XX_TRANSCEIVER)
endif()

if(RADIO_FAMILY STREQUAL lr20xx)
    target_link_libraries(lora_basics_modem_core PRIVATE
        lr20xx_driver_lora
        lr20xx_driver_fsk
        lr20xx_driver_flrc
        lr20xx_driver_lr_fhss
        lr20xx_driver_rttof
    )

    target_link_libraries(smtc_ral_${RADIO_FAMILY} PRIVATE
        lr20xx_driver_lora
        lr20xx_driver_fsk
        lr20xx_driver_flrc
        lr20xx_driver_lr_fhss
        lr20xx_driver_rttof
    )
endif()

################################################################################
# Compile definitions
if(LBM_CRYPTO MATCHES "^LR11XX")
    target_compile_definitions(lora_basics_modem_core PRIVATE USE_LR11XX_CE)
endif()

if(LBM_MODEM_TRACE)
    target_compile_definitions(lora_basics_modem_core PUBLIC MODEM_HAL_DBG_TRACE=1)
    target_compile_definitions(smtc_modem_crypto PUBLIC MODEM_HAL_DBG_TRACE=1)
else()
    target_compile_definitions(lora_basics_modem_core PUBLIC MODEM_HAL_DBG_TRACE=0)
    target_compile_definitions(smtc_modem_crypto PUBLIC MODEM_HAL_DBG_TRACE=0)
endif()

if(LBM_MODEM_TRACE AND LBM_MODEM_TRACE_DEEP)
    target_compile_definitions(lora_basics_modem_core PUBLIC MODEM_HAL_DEEP_DBG_TRACE=1)
    target_compile_definitions(smtc_modem_crypto PUBLIC MODEM_HAL_DEEP_DBG_TRACE=1)
else()
    target_compile_definitions(lora_basics_modem_core PUBLIC MODEM_HAL_DEEP_DBG_TRACE=0)
    target_compile_definitions(smtc_modem_crypto PUBLIC MODEM_HAL_DEEP_DBG_TRACE=0)
endif()

target_compile_definitions(lora_basics_modem_core PRIVATE NUMBER_OF_STACKS=${LBM_NUMBER_OF_STACKS})

if(LBM_CLASS_B)
    target_compile_definitions(lora_basics_modem_core PRIVATE ADD_CLASS_B)
    target_include_directories(lora_basics_modem_core PRIVATE
        lr1mac/src/lr1mac_class_b
    )
    target_sources(lora_basics_modem_core PRIVATE
        lorawan_manager/lorawan_class_b_management.c
        lr1mac/src/lr1mac_class_b/smtc_beacon_sniff.c
        lr1mac/src/lr1mac_class_b/smtc_ping_slot.c
    )
endif()

if(LBM_CLASS_C)
    target_compile_definitions(lora_basics_modem_core PRIVATE ADD_CLASS_C)
    target_include_directories(lora_basics_modem_core PRIVATE
        lr1mac/src/lr1mac_class_c
    )
    target_sources(lora_basics_modem_core PRIVATE
        lr1mac/src/lr1mac_class_c/lr1mac_class_c.c
    )
endif()

if(LBM_MULTICAST)
    target_compile_definitions(lora_basics_modem_core PRIVATE SMTC_MULTICAST)
    target_include_directories(lora_basics_modem_core PRIVATE
        lr1mac/src/services/smtc_multicast
    )
    target_sources(lora_basics_modem_core PRIVATE
        lr1mac/src/services/smtc_multicast/smtc_multicast.c
    )
endif()

if(LBM_FUOTA AND LBM_MULTICAST)
    set(FUOTA_MULTICAST_SOURCES_V1 lorawan_packages/remote_multicast_setup/v1.0.0/lorawan_remote_multicast_setup_package_v1.0.0.c)
    set(FUOTA_MULTICAST_SOURCES_V2 lorawan_packages/remote_multicast_setup/v2.0.0/lorawan_remote_multicast_setup_package_v2.0.0.c)
    target_sources(lora_basics_modem_core PRIVATE
        ${FUOTA_MULTICAST_SOURCES_V${LBM_FUOTA_VERSION}}
    )
endif()

if(LBM_CSMA)
    target_compile_definitions(lora_basics_modem_core PRIVATE ADD_CSMA)

    if(LBM_CSMA_BY_DEFAULT)
        target_compile_definitions(lora_basics_modem_core PRIVATE ENABLE_CSMA_BY_DEFAULT)
    endif()

    target_sources(lora_basics_modem_core PRIVATE
        lr1mac/src/services/smtc_lora_cad_bt.c
    )
endif()

if(LBM_ALC_SYNC)
    target_compile_definitions(lora_basics_modem_core PRIVATE ADD_SMTC_ALC_SYNC)
    target_include_directories(lora_basics_modem_core PRIVATE
        lorawan_packages/application_layer_clock_synchronization
    )
    set(ALC_SYNC_SOURCES_V1 lorawan_packages/application_layer_clock_synchronization/v1.0.0/lorawan_alcsync_v1.0.0.c)
    set(ALC_SYNC_SOURCES_V2 lorawan_packages/application_layer_clock_synchronization/v2.0.0/lorawan_alcsync_v2.0.0.c)
    target_sources(lora_basics_modem_core PRIVATE
        ${ALC_SYNC_SOURCES_V${LBM_ALC_SYNC_VERSION}}
    )
endif()

if(LBM_FUOTA)
    target_compile_definitions(lora_basics_modem_core PRIVATE ADD_FUOTA=${LBM_FUOTA_VERSION})
    set(FUOTA_INCLUDE_V1
        lorawan_packages/fragmented_data_block_transport/v1.0.0
    )
    set(FUOTA_INCLUDE_V2
        lorawan_packages/fragmented_data_block_transport/v2.0.0
    )
    target_include_directories(lora_basics_modem_core PRIVATE
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
    target_sources(lora_basics_modem_core PRIVATE
        ${FUOTA_SOURCES_V${LBM_FUOTA_VERSION}}
    )

    if(LBM_FUOTA_FMP)
        target_compile_definitions(lora_basics_modem_core PRIVATE ENABLE_FUOTA_FMP)
        target_include_directories(lora_basics_modem_core PRIVATE
            lorawan_packages/firmware_management_protocol
        )
        target_sources(lora_basics_modem_core PRIVATE
            lorawan_packages/firmware_management_protocol/lorawan_fmp_package.c
        )
    endif()

    if(LBM_FUOTA_MPA)
        target_compile_definitions(lora_basics_modem_core PRIVATE ENABLE_FUOTA_MPA)
        target_include_directories(lora_basics_modem_core PRIVATE
            lorawan_packages/multi_package_access
        )
        target_sources(lora_basics_modem_core PRIVATE
            lorawan_packages/multi_package_access/lorawan_mpa_package.c
        )
    endif()

    if(NOT "${LBM_FUOTA_FRAGMENTS_MAX_NUM}" STREQUAL "")
        target_compile_definitions(lora_basics_modem_core PRIVATE
            FRAG_MAX_NB=${LBM_FUOTA_FRAGMENTS_MAX_NUM}
        )
    endif()

    if(NOT "${LBM_FUOTA_FRAGMENTS_MAX_SIZE}" STREQUAL "")
        target_compile_definitions(lora_basics_modem_core PRIVATE
            FRAG_MAX_SIZE=${LBM_FUOTA_FRAGMENTS_MAX_SIZE}
        )
    endif()

    if(NOT "${LBM_FUOTA_FRAGMENTS_MAX_REDUNDANCY}" STREQUAL "")
        target_compile_definitions(lora_basics_modem_core PRIVATE
            FRAG_MAX_REDUNDANCY=${LBM_FUOTA_FRAGMENTS_MAX_REDUNDANCY}
        )
    endif()
endif()

if(LBM_ALMANAC)
    target_compile_definitions(lora_basics_modem_core PRIVATE ADD_ALMANAC)
    target_include_directories(lora_basics_modem_core PRIVATE
        modem_services
        modem_services/almanac_packages
    )
    target_sources(lora_basics_modem_core PRIVATE
        modem_services/almanac_packages/almanac.c
    )
endif()

if(LBM_STREAM)
    target_compile_definitions(lora_basics_modem_core PRIVATE ADD_SMTC_STREAM)
    target_include_directories(lora_basics_modem_core PRIVATE
        modem_services
        modem_services/stream_packages
    )
    target_sources(lora_basics_modem_core PRIVATE
        modem_services/stream_packages/rose.c
        modem_services/stream_packages/stream.c
    )
endif()

if(LBM_LFU)
    target_compile_definitions(lora_basics_modem_core PRIVATE ADD_SMTC_LFU)
    target_include_directories(lora_basics_modem_core PRIVATE
        modem_services
        modem_services/lfu_service
    )
    target_sources(lora_basics_modem_core PRIVATE
        modem_services/lfu_service/file_upload.c
    )
endif()

if(LBM_DEVICE_MANAGEMENT)
    target_compile_definitions(lora_basics_modem_core PRIVATE ADD_SMTC_CLOUD_DEVICE_MANAGEMENT)
    target_include_directories(lora_basics_modem_core PRIVATE
        modem_services
        modem_services/cloud_dm_package
    )
    target_sources(lora_basics_modem_core PRIVATE
        modem_services/cloud_dm_package/cloud_dm_package.c
    )
endif()

if(LBM_GEOLOCATION)
    target_compile_definitions(lora_basics_modem_core PRIVATE ADD_LBM_GEOLOCATION)
    target_include_directories(lora_basics_modem_core PUBLIC
        geolocation_services
    )
    target_sources(lora_basics_modem_core PRIVATE
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
    target_compile_definitions(lora_basics_modem_core PRIVATE ADD_SMTC_STORE_AND_FORWARD)
    target_include_directories(lora_basics_modem_core PRIVATE
        modem_services
        modem_services/store_and_forward
    )
    target_sources(lora_basics_modem_core PRIVATE
        modem_utilities/circularfs.c
        modem_services/store_and_forward/store_and_forward_flash.c
    )
endif()

if(LBM_RELAY_RX)
    target_compile_definitions(lora_basics_modem_core PRIVATE ADD_RELAY_RX)
    target_include_directories(lora_basics_modem_core PRIVATE
        lr1mac/src/relay/common
        lr1mac/src/relay/relay_rx
        lr1mac/src/relay/relay_tx
        modem_services/relay_service
    )
    target_sources(lora_basics_modem_core PRIVATE
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
    target_compile_definitions(lora_basics_modem_core PRIVATE ADD_RELAY_TX)
    target_include_directories(lora_basics_modem_core PRIVATE
        lr1mac/src/relay/common
        lr1mac/src/relay/relay_rx
        lr1mac/src/relay/relay_tx
        modem_services/relay_service
    )
    target_sources(lora_basics_modem_core PRIVATE
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
    target_compile_definitions(lora_basics_modem_core PRIVATE MODEM_BEACON_APP)
    target_include_directories(lora_basics_modem_core PRIVATE
        modem_services/beacon_tx_service
    )
    target_sources(lora_basics_modem_core PRIVATE
        modem_services/beacon_tx_service/lorawan_beacon_tx_service_example.c
    )
endif()

if(LBM_PERF_TEST)
    target_compile_definitions(lora_basics_modem_core PRIVATE PERF_TEST_ENABLED)
endif()

if(LBM_TEST_BYPASS_JOIN_DUTY_CYCLES)
    target_compile_definitions(lora_basics_modem_core PRIVATE TEST_BYPASS_JOIN_DUTY_CYCLE)
endif()
