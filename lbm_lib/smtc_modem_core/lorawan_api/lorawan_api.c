/*!
 * \file      lorawan_api.c
 *
 * \brief     Lorawan abstraction layer functions.
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"

#include "lr1mac_core.h"

#include "smtc_duty_cycle.h"
#if defined( ADD_CSMA )
#include "smtc_lora_cad_bt.h"
#endif  // ADD_CSMA
#if defined( SMTC_MULTICAST )
#include "smtc_multicast.h"
#endif  // SMTC_MULTICAST
#if defined( ADD_CLASS_C )
#include "lr1mac_class_c.h"
#endif
#if defined( ADD_CLASS_B )
#include "smtc_ping_slot.h"
#include "smtc_beacon_sniff.h"
#endif
#include "modem_core.h"
#include "smtc_real.h"
#include "smtc_secure_element.h"
#include "smtc_modem_crypto.h"
#include "lorawan_api.h"

static struct
{
    lr1_stack_mac_t lr1_mac_obj[NUMBER_OF_STACKS];
    smtc_real_t     real_obj[NUMBER_OF_STACKS];

#if defined( ADD_CLASS_C )
    lr1mac_class_c_t class_c_obj[NUMBER_OF_STACKS];
#endif  // ADD_CLASS_C

#if defined( ADD_CLASS_B )
    smtc_lr1_beacon_t lr1_beacon_obj[NUMBER_OF_STACKS];
    smtc_ping_slot_t  ping_slot_obj[NUMBER_OF_STACKS];
#endif  // ADD_CLASS_B

#if defined( SMTC_MULTICAST )
    smtc_multicast_t multicast_obj[NUMBER_OF_STACKS];
#endif  // SMTC_MULTICAST

} lr1mac_core_context;

lorawan_down_metadata_t lorawan_down_metadata;

#define lr1_mac_obj lr1mac_core_context.lr1_mac_obj

#define real_obj lr1mac_core_context.real_obj
#define class_c_obj lr1mac_core_context.class_c_obj

#define lr1_beacon_obj lr1mac_core_context.lr1_beacon_obj
#define ping_slot_obj lr1mac_core_context.ping_slot_obj

#if defined( SMTC_MULTICAST )
#define multicast_obj lr1mac_core_context.multicast_obj
#endif

void lorawan_api_init( radio_planner_t* rp, uint8_t stack_id,
                       void ( *lr1mac_downlink_callback )( lr1_stack_mac_down_data_t* push_context ) )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );

    lr1mac_core_init( &lr1_mac_obj[stack_id], &real_obj[stack_id], rp, ACTIVATION_MODE_OTAA, lr1mac_downlink_callback,
                      &stack_id );

#if defined( ADD_CLASS_C ) || defined( ADD_CLASS_B )
    lr1mac_rx_session_param_t* multicast_rx_sessions    = NULL;
    uint8_t                    nb_multicast_rx_sessions = 0;
#endif
#if defined( SMTC_MULTICAST )
    multicast_rx_sessions    = multicast_obj[stack_id].rx_session_param;
    nb_multicast_rx_sessions = LR1MAC_MC_NUMBER_OF_SESSION;
    smtc_multicast_init( &multicast_obj[stack_id], stack_id );
#endif

#if defined( ADD_CLASS_C )
    lr1mac_class_c_init( &class_c_obj[stack_id], &lr1_mac_obj[stack_id], multicast_rx_sessions,
                         nb_multicast_rx_sessions, rp, RP_HOOK_ID_CLASS_C + stack_id,
                         ( void ( * )( void* ) ) lr1mac_class_c_mac_rp_callback, &class_c_obj[stack_id],
                         lr1mac_downlink_callback );
#endif

#if defined( ADD_CLASS_B )
    smtc_ping_slot_init( &ping_slot_obj[stack_id], &lr1_mac_obj[stack_id], multicast_rx_sessions,
                         nb_multicast_rx_sessions, rp, RP_HOOK_ID_CLASS_B_PING_SLOT + stack_id,
                         ( void ( * )( void* ) ) smtc_ping_slot_mac_rp_callback, &ping_slot_obj[stack_id],
                         lr1mac_downlink_callback );
    smtc_beacon_sniff_init( &lr1_beacon_obj[stack_id], &ping_slot_obj[stack_id], &lr1_mac_obj[stack_id], rp,
                            RP_HOOK_ID_CLASS_B_BEACON + stack_id, lr1mac_downlink_callback );
#endif
}

smtc_real_region_types_t lorawan_api_get_region( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_get_region( &lr1_mac_obj[stack_id] );
}

status_lorawan_t lorawan_api_set_region( smtc_real_region_types_t region_type, uint8_t stack_id )
{
    status_lorawan_t ret;
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    ret = lr1mac_core_set_region( &lr1_mac_obj[stack_id], region_type );
    if( ret != OKLORAWAN )
    {
        return ret;
    }

#if defined( REGION_EU_868 )
    if( region_type == SMTC_REAL_REGION_EU_868 )
    {
        // Configure duty-cycle object
        for( int i = 0; i < BAND_EU868_MAX; i++ )
        {
            smtc_duty_cycle_config( BAND_EU868_MAX, i, duty_cycle_by_band_eu_868[i],
                                    frequency_range_by_band_eu_868[i][0], frequency_range_by_band_eu_868[i][1] );
        }
    }
#endif
#if defined( REGION_RU_864 )
    if( region_type == SMTC_REAL_REGION_RU_864 )
    {
        // Configure duty-cycle object
        for( int i = 0; i < BAND_RU864_MAX; i++ )
        {
            smtc_duty_cycle_config( BAND_RU864_MAX, i, duty_cycle_by_band_ru_864[i],
                                    frequency_range_by_band_ru_864[i][0], frequency_range_by_band_ru_864[i][1] );
        }
    }
#endif

    // Enable duty-cycle constraint if one region need to support the duty cycle
    // Remark: In multi-stack EU868 and IN865 there is a bands overlapping, EU868 has a duty-cycle but not IN865, in
    // this case the duty cycle will be enabled
    smtc_dtc_enablement_type_t dtc_supported = SMTC_DTC_FULL_DISABLED;
    for( uint8_t i = 0; i < NUMBER_OF_STACKS; i++ )
    {
        if( smtc_real_is_dtc_supported( lr1_mac_obj[i].real ) == true )
        {
            dtc_supported = SMTC_DTC_ENABLED;
            break;
        }
    }
    smtc_duty_cycle_enable_set( dtc_supported );
    return ret;
}

status_lorawan_t lorawan_api_payload_send( uint8_t fport, bool fport_enabled, const uint8_t* data, uint8_t data_len,
                                           lr1mac_layer_param_t packet_type, uint32_t target_time_ms, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_payload_send( &lr1_mac_obj[stack_id], fport, fport_enabled, data, data_len, packet_type,
                                     target_time_ms );
}

status_lorawan_t lorawan_api_payload_send_at_time( uint8_t fport, bool fport_enabled, const uint8_t* data,
                                                   uint8_t data_len, lr1mac_layer_param_t packet_type,
                                                   uint32_t target_time_ms, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_payload_send_at_time( &lr1_mac_obj[stack_id], fport, fport_enabled, data, data_len, packet_type,
                                             target_time_ms );
}

status_lorawan_t lorawan_api_send_stack_cid_req( uint8_t* cid_req_list, uint8_t cid_req_list_size,
                                                 uint32_t target_time_ms, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_send_stack_cid_req( &lr1_mac_obj[stack_id], cid_req_list, cid_req_list_size, target_time_ms );
}

status_lorawan_t lorawan_api_join( uint32_t target_time_ms, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_join( &lr1_mac_obj[stack_id], target_time_ms );
}

join_status_t lorawan_api_isjoined( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1_mac_joined_status_get( &lr1_mac_obj[stack_id] );
}

void lorawan_api_join_status_clear( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    lr1mac_core_join_status_clear( &lr1_mac_obj[stack_id] );
}

status_lorawan_t lorawan_api_dr_strategy_set( dr_strategy_t dr_strategy, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return ( lr1mac_core_dr_strategy_set( &lr1_mac_obj[stack_id], dr_strategy ) );
}

dr_strategy_t lorawan_api_dr_strategy_get( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_dr_strategy_get( &lr1_mac_obj[stack_id] );
}

void lorawan_api_dr_custom_set( uint8_t* custom_dr, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    smtc_real_set_custom_dr_distribution( lr1_mac_obj[stack_id].real, custom_dr );
}

void lorawan_api_dr_custom_get( uint8_t* custom_dr, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    smtc_real_get_custom_dr_distribution( lr1_mac_obj[stack_id].real, custom_dr );
}

void lorawan_api_dr_join_distribution_set( uint8_t* custom_dr, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    smtc_real_set_join_dr_distribution( lr1_mac_obj[stack_id].real, custom_dr );
}

lr1mac_states_t lorawan_api_process( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_process( &lr1_mac_obj[stack_id] );
}

void lorawan_api_context_load( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    lr1mac_core_context_load( &lr1_mac_obj[stack_id] );
}

void lorawan_api_factory_reset( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    lr1mac_core_context_factory_reset( &lr1_mac_obj[stack_id] );
}

lr1mac_activation_mode_t lorawan_api_get_activation_mode( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_get_activation_mode( &lr1_mac_obj[stack_id] );
}

void lorawan_api_set_activation_mode( lr1mac_activation_mode_t activation_mode, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    lr1mac_core_set_activation_mode( &lr1_mac_obj[stack_id], activation_mode );
}

uint32_t lorawan_api_next_max_payload_length_get( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_next_max_payload_length_get( &lr1_mac_obj[stack_id] );
}

uint32_t lorawan_api_devaddr_get( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_devaddr_get( &lr1_mac_obj[stack_id] );
}

status_lorawan_t lorawan_api_devaddr_set( uint32_t dev_addr, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_devaddr_set( &lr1_mac_obj[stack_id], dev_addr );
}

status_lorawan_t lorawan_api_get_deveui( uint8_t* dev_eui, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    status_lorawan_t rc = OKLORAWAN;

    if( smtc_secure_element_get_deveui( dev_eui, stack_id ) != SMTC_SE_RC_SUCCESS )
    {
        rc = ERRORLORAWAN;
    }
    return rc;
}

status_lorawan_t lorawan_api_set_deveui( const uint8_t* dev_eui, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    status_lorawan_t rc = OKLORAWAN;

    if( smtc_secure_element_set_deveui( dev_eui, stack_id ) != SMTC_SE_RC_SUCCESS )
    {
        rc = ERRORLORAWAN;
    }
    return rc;
}

status_lorawan_t lorawan_api_get_joineui( uint8_t* join_eui, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    status_lorawan_t rc = OKLORAWAN;
    if( smtc_secure_element_get_joineui( join_eui, stack_id ) != SMTC_SE_RC_SUCCESS )
    {
        rc = ERRORLORAWAN;
    }
    return rc;
}

status_lorawan_t lorawan_api_set_joineui( const uint8_t* join_eui, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    status_lorawan_t rc = OKLORAWAN;
    if( smtc_secure_element_set_joineui( join_eui, stack_id ) != SMTC_SE_RC_SUCCESS )
    {
        rc = ERRORLORAWAN;
    }
    return rc;
}

status_lorawan_t lorawan_api_set_appkey( const uint8_t* app_key, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    // in lorawan 1.0.x SMTC_SE_APP_KEY is for GEN_APP_KEY(used for multicast features)
    if( smtc_modem_crypto_set_key( SMTC_SE_APP_KEY, app_key, stack_id ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        return ERRORLORAWAN;
    }
    return OKLORAWAN;
}

status_lorawan_t lorawan_api_set_nwkkey( const uint8_t* nwk_key, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    // in lorawan 1.0.x SMTC_SE_NWK_KEY is for APP_KEY
    if( smtc_modem_crypto_set_key( SMTC_SE_NWK_KEY, nwk_key, stack_id ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        return ERRORLORAWAN;
    }
    return OKLORAWAN;
}

uint8_t lorawan_api_next_power_get( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_next_power_get( &lr1_mac_obj[stack_id] );
}

uint8_t lorawan_api_next_dr_get( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_next_dr_get( &lr1_mac_obj[stack_id] );
}

modulation_type_t lorawan_api_get_modulation_type_from_datarate( uint8_t stack_id, uint8_t datarate )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return smtc_real_get_modulation_type_from_datarate( lr1_mac_obj[stack_id].real, datarate );
}

uint32_t lorawan_api_next_frequency_get( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_next_frequency_get( &lr1_mac_obj[stack_id] );
}

uint8_t lorawan_api_max_tx_dr_get( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return smtc_real_get_max_tx_channel_dr( lr1_mac_obj[stack_id].real );
}

uint16_t lorawan_api_mask_tx_dr_channel_up_dwell_time_check( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return smtc_real_mask_tx_dr_channel_up_dwell_time_check( lr1_mac_obj[stack_id].real );
}

uint8_t lorawan_api_min_tx_dr_get( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return smtc_real_get_min_tx_channel_dr( lr1_mac_obj[stack_id].real );
}

lr1mac_states_t lorawan_api_state_get( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_state_get( &lr1_mac_obj[stack_id] );
}

uint16_t lorawan_api_devnonce_get( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_devnonce_get( &lr1_mac_obj[stack_id] );
}

uint32_t lorawan_api_next_join_time_second_get( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_next_join_time_second_get( &lr1_mac_obj[stack_id] );
}

bool lorawan_api_is_dtc_supported( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return smtc_real_is_dtc_supported( lr1_mac_obj[stack_id].real );
}

uint32_t lorawan_api_next_network_free_duty_cycle_ms_get( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1_stack_network_next_free_duty_cycle_ms_get( &lr1_mac_obj[stack_id] );
}

uint32_t lorawan_api_fcnt_up_get( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_fcnt_up_get( &lr1_mac_obj[stack_id] );
}

#if defined( ADD_CLASS_C )
void lorawan_api_class_c_enabled( bool enable, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    lr1mac_class_c_enabled( &class_c_obj[stack_id], enable );

    if( lorawan_api_isjoined( stack_id ) == JOINED )
    {
        lr1mac_class_c_start( &class_c_obj[stack_id] );
    }
}

bool lorawan_api_class_c_is_running( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_class_c_is_running( &class_c_obj[stack_id] );
}

void lorawan_api_class_c_start( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    lr1mac_class_c_start( &class_c_obj[stack_id] );
}

void lorawan_api_class_c_stop( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    lr1mac_class_c_stop( &class_c_obj[stack_id] );
}
#endif

lorawan_multicast_rc_t lorawan_api_multicast_set_group_session_keys( uint8_t       mc_group_id,
                                                                     const uint8_t mc_ntw_skey[LORAWAN_KEY_SIZE],
                                                                     const uint8_t mc_app_skey[LORAWAN_KEY_SIZE],
                                                                     uint8_t       stack_id )
{
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) smtc_multicast_set_group_keys( &multicast_obj[stack_id], mc_group_id, mc_ntw_skey,
                                                                     mc_app_skey );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_set_group_address( uint8_t mc_group_id, uint32_t mc_group_address,
                                                                uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) smtc_multicast_set_group_address( &multicast_obj[stack_id], mc_group_id,
                                                                        mc_group_address );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_get_group_address( uint8_t mc_group_id, uint32_t* mc_group_address,
                                                                uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) smtc_multicast_get_group_address( &multicast_obj[stack_id], mc_group_id,
                                                                        mc_group_address );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_set_fcnt_down_range( uint8_t mc_group_id, uint32_t mc_min_fcnt_down,
                                                                  uint32_t mc_max_fcnt_down, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) smtc_multicast_set_fcnt_down_range( &multicast_obj[stack_id], mc_group_id,
                                                                          mc_min_fcnt_down, mc_max_fcnt_down );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_get_fcnt_down_range( uint8_t mc_group_id, uint32_t* mc_min_fcnt_down,
                                                                  uint32_t* mc_max_fcnt_down, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) smtc_multicast_get_fcnt_down_range( &multicast_obj[stack_id], mc_group_id,
                                                                          mc_min_fcnt_down, mc_max_fcnt_down );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_derive_group_keys( uint8_t       mc_group_id,
                                                                const uint8_t mc_key_encrypted[LORAWAN_KEY_SIZE],
                                                                uint8_t       stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) smtc_multicast_derive_group_keys( &multicast_obj[stack_id], mc_group_id,
                                                                        mc_key_encrypted );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_get_running_status( uint8_t mc_group_id, bool* session_running,
                                                                 uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) smtc_multicast_get_running_status( &multicast_obj[stack_id], mc_group_id,
                                                                         session_running );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_c_get_session_status( uint8_t mc_group_id, bool* is_session_started,
                                                                   uint32_t* freq, uint8_t* dr, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
#if defined( SMTC_MULTICAST ) && defined( ADD_CLASS_C )
    return ( lorawan_multicast_rc_t ) lr1mac_class_c_multicast_get_session_status( &class_c_obj[stack_id], mc_group_id,
                                                                                   is_session_started, freq, dr );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_c_start_session( uint8_t mc_group_id, uint32_t freq, uint8_t dr,
                                                              uint8_t stack_id )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "Start multicast class C session with group_id = %d\n", mc_group_id );
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
#if defined( SMTC_MULTICAST ) && defined( ADD_CLASS_C )
    return ( lorawan_multicast_rc_t ) lr1mac_class_c_multicast_start_session( &class_c_obj[stack_id], mc_group_id, freq,
                                                                              dr );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_c_stop_session( uint8_t mc_group_id, uint8_t stack_id )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "Stop multicast class C session with group_id = %d\n", mc_group_id );
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
#if defined( SMTC_MULTICAST ) && defined( ADD_CLASS_C )
    return ( lorawan_multicast_rc_t ) lr1mac_class_c_multicast_stop_session( &class_c_obj[stack_id], mc_group_id );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_c_stop_all_sessions( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
#if defined( SMTC_MULTICAST ) && defined( ADD_CLASS_C )
    return ( lorawan_multicast_rc_t ) lr1mac_class_c_multicast_stop_all_sessions( &class_c_obj[stack_id] );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_b_get_session_status( uint8_t mc_group_id, bool* is_session_started,
                                                                   bool* waiting_beacon_to_start, uint32_t* freq,
                                                                   uint8_t* dr, uint8_t* ping_slot_periodicity,
                                                                   uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
#if defined( SMTC_MULTICAST ) && defined( ADD_CLASS_B )
    return ( lorawan_multicast_rc_t ) smtc_ping_slot_multicast_b_get_session_status(
        &ping_slot_obj[stack_id], mc_group_id, is_session_started, waiting_beacon_to_start, freq, dr,
        ping_slot_periodicity );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_b_start_session( uint8_t mc_group_id, uint32_t freq, uint8_t dr,
                                                              uint8_t ping_slot_periodicity, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
#if defined( SMTC_MULTICAST ) && defined( ADD_CLASS_B )
    return ( lorawan_multicast_rc_t ) smtc_ping_slot_multicast_b_start_session( &ping_slot_obj[stack_id], mc_group_id,
                                                                                freq, dr, ping_slot_periodicity );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_b_stop_session( uint8_t mc_group_id, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
#if defined( SMTC_MULTICAST ) && defined( ADD_CLASS_B )
    return ( lorawan_multicast_rc_t ) smtc_ping_slot_multicast_b_stop_session( &ping_slot_obj[stack_id], mc_group_id );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_b_stop_all_sessions( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
#if defined( SMTC_MULTICAST ) && defined( ADD_CLASS_B )
    return ( lorawan_multicast_rc_t ) smtc_ping_slot_multicast_b_stop_all_sessions( &ping_slot_obj[stack_id] );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

void lorawan_api_set_no_rx_packet_threshold( uint16_t no_rx_packet_reset_threshold, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    lr1mac_core_set_no_rx_packet_threshold( &lr1_mac_obj[stack_id], no_rx_packet_reset_threshold );
}

uint16_t lorawan_api_get_no_rx_packet_threshold( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_get_no_rx_packet_threshold( &lr1_mac_obj[stack_id] );
}

uint16_t lorawan_api_get_current_adr_ack_cnt( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_get_current_adr_ack_cnt( &lr1_mac_obj[stack_id] );
}

void lorawan_api_reset_no_rx_packet_in_mobile_mode_cnt( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    lr1mac_core_reset_no_rx_packet_in_mobile_mode_cnt( &lr1_mac_obj[stack_id] );
}

uint16_t lorawan_api_get_current_no_rx_packet_in_mobile_mode_cnt( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_get_current_no_rx_packet_in_mobile_mode( &lr1_mac_obj[stack_id] );
}

uint16_t lorawan_api_get_current_no_rx_packet_cnt( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_get_current_no_rx_packet_cnt( &lr1_mac_obj[stack_id] );
}

uint32_t lorawan_api_get_current_no_rx_packet_cnt_since_s( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_get_current_no_rx_packet_cnt_since_s( &lr1_mac_obj[stack_id] );
}

bool lorawan_api_join_duty_cycle_backoff_bypass_get( uint8_t stack_id )
{
    return lr1mac_core_join_duty_cycle_backoff_bypass_get( &lr1_mac_obj[stack_id] );
}

void lorawan_api_join_duty_cycle_backoff_bypass_set( uint8_t stack_id, bool enable )
{
    lr1mac_core_join_duty_cycle_backoff_bypass_set( &lr1_mac_obj[stack_id], enable );
}

void lorawan_api_modem_certification_set( uint8_t enable, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    lr1mac_core_certification_set( &lr1_mac_obj[stack_id], enable );
}

uint8_t lorawan_api_modem_certification_is_enabled( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_certification_get( &lr1_mac_obj[stack_id] );
}

/*!
 * \brief  return true if stack receive a link adr request
 * \remark reset the flag automatically each time the upper layer call this function
 * \param [in]  void
 * \param [out] bool
 */
bool lorawan_api_available_link_adr_get( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_available_link_adr_get( &lr1_mac_obj[stack_id] );
}
lr1_stack_mac_t* lorawan_api_stack_mac_get( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return ( &lr1_mac_obj[stack_id] );
}

void lorawan_api_set_network_type( bool network_type, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    uint8_t sync_word = ( network_type == true ) ? smtc_real_get_public_sync_word( lr1_mac_obj[stack_id].real )
                                                 : smtc_real_get_private_sync_word( lr1_mac_obj[stack_id].real );

    smtc_real_set_sync_word( lr1_mac_obj[stack_id].real, sync_word );
}
bool lorawan_api_get_network_type( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    uint8_t sync_word = smtc_real_get_sync_word( lr1_mac_obj[stack_id].real );
    return ( ( sync_word == smtc_real_get_public_sync_word( lr1_mac_obj[stack_id].real ) ) ? true : false );
}

uint8_t lorawan_api_nb_trans_get( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1_stack_nb_trans_get( &lr1_mac_obj[stack_id] );
}

status_lorawan_t lorawan_api_nb_trans_set( uint8_t nb_trans, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1_stack_nb_trans_set( &lr1_mac_obj[stack_id], nb_trans );
}

uint32_t lorawan_api_get_crystal_error( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1_stack_get_crystal_error( &lr1_mac_obj[stack_id] );
}

void lorawan_api_set_crystal_error( uint32_t crystal_error, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    lr1_stack_set_crystal_error( &lr1_mac_obj[stack_id], crystal_error );
}

lr1mac_version_t lorawan_api_get_spec_version( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_get_lorawan_version( &lr1_mac_obj[stack_id] );
}

lr1mac_version_t lorawan_api_get_regional_parameters_version( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return smtc_real_get_regional_parameters_version( );
}

bool lorawan_api_convert_rtc_to_gps_epoch_time( uint32_t rtc_ms, uint32_t* seconds_since_epoch,
                                                uint32_t* fractional_second, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_convert_rtc_to_gps_epoch_time( &lr1_mac_obj[stack_id], rtc_ms, seconds_since_epoch,
                                                      fractional_second );
}

bool lorawan_api_is_time_valid( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_is_time_valid( &lr1_mac_obj[stack_id] );
}

uint32_t lorawan_api_get_timestamp_last_device_time_ans_s( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_get_timestamp_last_device_time_ans_s( &lr1_mac_obj[stack_id] );
}

uint32_t lorawan_api_get_time_left_connection_lost( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_get_time_left_connection_lost( &lr1_mac_obj[stack_id] );
}

status_lorawan_t lorawan_api_set_device_time_invalid_delay_s( uint32_t delay_s, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1_mac_core_set_device_time_invalid_delay_s( &lr1_mac_obj[stack_id], delay_s );
}

uint32_t lorawan_api_get_device_time_invalid_delay_s( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1_mac_core_get_device_time_invalid_delay_s( &lr1_mac_obj[stack_id] );
}

status_lorawan_t lorawan_api_get_link_check_ans( uint8_t* margin, uint8_t* gw_cnt, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1_mac_core_get_link_check_ans( &lr1_mac_obj[stack_id], margin, gw_cnt );
}

status_lorawan_t lorawan_api_get_device_time_req_status( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1_mac_core_get_device_time_req_status( &lr1_mac_obj[stack_id] );
}

#if defined( ADD_CLASS_B )
void lorawan_api_class_b_enabled( bool enable, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    smtc_beacon_class_b_enable_service( &lr1_beacon_obj[stack_id], enable );

    if( ( lorawan_api_isjoined( stack_id ) == JOINED ) && ( enable == true ) )
    {
        smtc_beacon_sniff_start( &lr1_beacon_obj[stack_id] );
    }
}
bool lorawan_api_class_b_enabled_get( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return ( smtc_beacon_class_b_enable_get( &lr1_beacon_obj[stack_id] ) );
}
void lorawan_api_beacon_sniff_start( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    smtc_beacon_sniff_start( &lr1_beacon_obj[stack_id] );
}

void lorawan_api_beacon_sniff_stop( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    smtc_beacon_sniff_stop( &lr1_beacon_obj[stack_id] );
}

void lorawan_api_beacon_get_statistics( smtc_beacon_statistics_t* beacon_statistics, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    smtc_beacon_sniff_get_statistics( &lr1_beacon_obj[stack_id], beacon_statistics );
}
#endif

status_lorawan_t lorawan_api_get_ping_slot_info_req_status( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1_mac_core_get_ping_slot_info_req_status( &lr1_mac_obj[stack_id] );
}

status_lorawan_t lorawan_api_set_ping_slot_periodicity( uint8_t ping_slot_periodicity, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_set_ping_slot_periodicity( &lr1_mac_obj[stack_id], ping_slot_periodicity );
}

uint8_t lorawan_api_get_ping_slot_periodicity( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_get_ping_slot_periodicity( &lr1_mac_obj[stack_id] );
}

bool lorawan_api_get_class_b_status( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_get_class_b_status( &lr1_mac_obj[stack_id] );
}

void lorawan_api_lora_dr_to_sf_bw( uint8_t in_dr, uint8_t* out_sf, lr1mac_bandwidth_t* out_bw, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    smtc_real_lora_dr_to_sf_bw( lr1_mac_obj[stack_id].real, in_dr, out_sf, out_bw );
}

uint8_t lorawan_api_get_frequency_factor( uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return smtc_real_get_frequency_factor( lr1_mac_obj[stack_id].real );
}

status_lorawan_t lorawan_api_set_adr_ack_limit_delay( uint8_t adr_ack_limit, uint8_t adr_ack_delay, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    return lr1mac_core_set_adr_ack_limit_delay( &lr1_mac_obj[stack_id], adr_ack_limit, adr_ack_delay );
}

void lorawan_api_get_adr_ack_limit_delay( uint8_t* adr_ack_limit, uint8_t* adr_ack_delay, uint8_t stack_id )
{
    PANIC_IF_STACK_ID_TOO_HIGH( stack_id );
    lr1mac_core_get_adr_ack_limit_delay( &lr1_mac_obj[stack_id], adr_ack_limit, adr_ack_delay );
}

bool lorawan_api_get_current_enabled_frequencies_list( uint8_t* number_of_freq, uint32_t* freq_list, uint8_t max_size,
                                                       uint8_t stack_id )
{
    return smtc_real_get_current_enabled_frequency_list( lr1_mac_obj[stack_id].real, number_of_freq, freq_list,
                                                         max_size );
}
void lorawan_api_tx_ack_bit_set( uint8_t stack_id, bool enable )
{
    lr1_stack_mac_tx_ack_bit_set( &lr1_mac_obj[stack_id], enable );
}
bool lorawan_api_tx_ack_bit_get( uint8_t stack_id )
{
    return ( lr1_stack_mac_tx_ack_bit_get( &lr1_mac_obj[stack_id] ) );
}

void lorawan_api_set_no_rx_windows( uint8_t stack_id, uint8_t disable_rx_windows )
{
    lr1mac_core_set_no_rx_windows( &lr1_mac_obj[stack_id], disable_rx_windows );
}

uint8_t lorawan_api_get_no_rx_windows( uint8_t stack_id )
{
    return lr1mac_core_get_no_rx_windows( &lr1_mac_obj[stack_id] );
}

status_lorawan_t lorawan_api_is_frequency_valid( uint8_t stack_id, uint32_t freq )
{
    return smtc_real_is_frequency_valid( lr1_mac_obj[stack_id].real, freq );
}

status_lorawan_t lorawan_api_is_datarate_valid( uint8_t stack_id, uint8_t dr )
{
    return smtc_real_is_rx_dr_valid( lr1_mac_obj[stack_id].real, dr );
}
void lorawan_api_core_abort( uint8_t stack_id )
{
    lr1mac_core_abort( &lr1_mac_obj[stack_id] );
}
status_lorawan_t lorawan_api_update_join_channel( uint8_t stack_id )
{
    return ( lr1mac_core_update_join_channel( &lr1_mac_obj[stack_id] ) );
}
status_lorawan_t lorawan_api_update_next_tx_channel( uint8_t stack_id )
{
    return ( lr1mac_core_update_next_tx_channel( &lr1_mac_obj[stack_id] ) );
}
uint32_t lorawan_api_get_time_of_nwk_ans( uint8_t stack_id )
{
    return lr1mac_core_get_time_of_nwk_ans( &lr1_mac_obj[stack_id] );
}
void lorawan_api_set_time_of_nwk_ans( uint8_t stack_id, uint32_t target_time )
{
    lr1mac_core_set_time_of_nwk_ans( &lr1_mac_obj[stack_id], target_time );
}
void lorawan_api_set_next_tx_at_time( uint8_t stack_id, bool is_send_at_time )
{
    lr1mac_core_set_next_tx_at_time( &lr1_mac_obj[stack_id], is_send_at_time );
}
void lorawan_api_set_join_status( uint8_t stack_id, join_status_t join_status )
{
    lr1mac_core_set_join_status( &lr1_mac_obj[stack_id], join_status );
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
