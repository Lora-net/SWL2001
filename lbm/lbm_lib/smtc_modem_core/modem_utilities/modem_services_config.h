/*!
 * @file      modem_service_config.h
 *
 * @brief     share functions + context of the soft modem .
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __MODEM_SERVICES_CONFIG_H__
#define __MODEM_SERVICES_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "lorawan_certification.h"
#include "lorawan_join_management.h"
#include "lorawan_dwn_ack_management.h"

#if defined( ADD_SMTC_ALC_SYNC )
#include "lorawan_alcsync.h"
#endif

#if defined( MODEM_BEACON_APP )
#include "lorawan_beacon_tx_service_example.h"
#endif

#if defined( ADD_RELAY_RX )
#include "lorawan_relay_rx_service.h"
#endif

#if defined( ADD_RELAY_TX )
#include "lorawan_relay_tx_service.h"
#endif

#if defined( ADD_LBM_GEOLOCATION )
#include "mw_gnss_scan.h"
#include "mw_gnss_send.h"
#include "mw_gnss_almanac.h"
#include "mw_wifi_scan.h"
#include "mw_wifi_send.h"
#endif

#if defined( ADD_FUOTA )
#include "lorawan_fragmentation_package.h"
#include "lorawan_remote_multicast_setup_package.h"
#ifdef ENABLE_FUOTA_FMP
#include "lorawan_fmp_package.h"
#endif
#ifdef ENABLE_FUOTA_MPA
#include "lorawan_mpa_package.h"
#endif
#endif

#if defined( ADD_ALMANAC )
#include "almanac.h"
#endif

#if defined( ADD_SMTC_STREAM )
#include "stream.h"
#endif

#if defined( ADD_SMTC_CLOUD_DEVICE_MANAGEMENT )
#include "cloud_dm_package.h"
#endif

#if defined( ADD_SMTC_LFU )
#include "file_upload.h"
#endif

#if defined( ADD_SMTC_STORE_AND_FORWARD )
#include "store_and_forward_flash.h"
#include "store_and_forward.h"
#endif

typedef struct modem_service_config_s
{
    uint8_t service_id;  // Start to 0 for new type of services, increment this number for multiple instantiation of the
                         // same type of service
    uint8_t stack_id;    // The linked LoRaWAN stack to this service
    void ( *callbacks_init_service )( uint8_t* service_id, uint8_t task_id,
                                      uint8_t ( **callback )( lr1_stack_mac_down_data_t* ),
                                      void ( **callback_on_launch )( void* ), void ( **callback_on_update )( void* ),
                                      void** callback_context );

} modem_service_config_t;

static modem_service_config_t modem_service_config[] = {
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = lorawan_certification_services_init },
#ifdef ADD_RELAY_RX
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = lorawan_relay_rx_services_init },
#endif
#if defined( ADD_RELAY_TX )
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = lorawan_relay_tx_services_init },
#endif
#ifdef ADD_SMTC_ALC_SYNC
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = lorawan_alcsync_services_init },
#endif
#ifdef ADD_FUOTA
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = lorawan_fragmentation_package_services_init },
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = lorawan_remote_multicast_setup_package_services_init },
#ifdef ENABLE_FUOTA_FMP
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = lorawan_fmp_package_services_init },
#endif
#ifdef ENABLE_FUOTA_MPA
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = lorawan_mpa_package_services_init },
#endif
#endif
#ifdef ADD_ALMANAC
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = almanac_services_init },
#endif
#ifdef ADD_SMTC_STREAM
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = stream_services_init },
#endif
#ifdef ADD_SMTC_CLOUD_DEVICE_MANAGEMENT
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = cloud_dm_services_init },
#endif
#ifdef ADD_SMTC_LFU
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = lfu_services_init },
#endif
#ifdef ADD_LBM_GEOLOCATION
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = mw_gnss_scan_services_init },
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = mw_gnss_send_services_init },
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = mw_gnss_almanac_services_init },
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = mw_wifi_scan_services_init },
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = mw_wifi_send_services_init },
#endif
#ifdef MODEM_BEACON_APP
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = lorawan_beacon_tx_example_service_init },
#endif
#ifdef ADD_SMTC_STORE_AND_FORWARD
    { .service_id = 0, .stack_id = 0, .callbacks_init_service = store_and_forward_flash_services_init },
// { .service_id = 0, .stack_id = 0, .callbacks_init_service = store_and_forward_services_init },
#endif
};

#define NUMBER_OF_SERVICES ( sizeof modem_service_config / sizeof modem_service_config[0] )

#ifdef __cplusplus
}
#endif

#endif  // __SOFT_MODEM_CONTEXT_H__
