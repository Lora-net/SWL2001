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

#include "lorawan_api.h"

#include "smtc_modem_hal_dbg_trace.h"
#include "lorawan_certification.h"
#include "lr1mac_core.h"
#include "smtc_modem_hal.h"
#include "smtc_d2d.h"
#include "smtc_lbt.h"
#include "smtc_duty_cycle.h"
#include "smtc_multicast.h"
#include "lr1mac_class_c.h"
#include "smtc_ping_slot.h"
#include "smtc_beacon_sniff.h"
#include "modem_utilities.h"
#include "fifo_ctrl.h"
#include "smtc_real.h"
#include "smtc_modem_api.h"
#include "modem_supervisor.h"
#include "modem_context.h"
#include "smtc_secure_element.h"
#include "smtc_modem_crypto.h"

static struct
{
    lr1_stack_mac_t   lr1_mac_obj;
    smtc_real_t       real;
    smtc_lbt_t        lbt_obj;
    smtc_dtc_t        duty_cycle_obj;
    lr1mac_class_c_t  class_c_obj;
    fifo_ctrl_t       fifo_ctrl_obj;
    smtc_lr1_beacon_t lr1_beacon_obj;
    smtc_ping_slot_t  ping_slot_obj;
#if defined( SMTC_MULTICAST )
    smtc_multicast_t multicast_obj;
#endif
#if defined( SMTC_D2D )
    smtc_class_b_d2d_t class_b_d2d_obj;
#endif
    lorawan_certification_t lorawan_certif_obj;
} lr1mac_core_context;

#define FIFO_LORAWAN_SIZE 512
uint8_t fifo_buffer[FIFO_LORAWAN_SIZE];

#define lr1_mac_obj lr1mac_core_context.lr1_mac_obj
#define lbt_obj lr1mac_core_context.lbt_obj
#define real lr1mac_core_context.real
#define duty_cycle_obj lr1mac_core_context.duty_cycle_obj
#define class_c_obj lr1mac_core_context.class_c_obj
#define fifo_ctrl_obj lr1mac_core_context.fifo_ctrl_obj
#define lorawan_certif_obj lr1mac_core_context.lorawan_certif_obj
#define lr1_beacon_obj lr1mac_core_context.lr1_beacon_obj
#define ping_slot_obj lr1mac_core_context.ping_slot_obj
#define multicast_obj lr1mac_core_context.multicast_obj
#define class_b_d2d_obj lr1mac_core_context.class_b_d2d_obj

static void lorawan_api_class_a_downlink_callback( lr1_stack_mac_t* lr1_mac_object );
static void lorawan_api_class_c_downlink_callback( lr1mac_class_c_t* class_c_object );
static void lorawan_api_class_b_downlink_callback( smtc_ping_slot_t* class_b_object );
static void lorawan_api_class_b_beacon_callback( smtc_lr1_beacon_t* class_b_beacon_object );
#if defined( SMTC_D2D )
static void lorawan_api_class_b_d2d_tx_event_callback( smtc_class_b_d2d_t* class_b_d2d_object );
#endif

void lorawan_api_init( radio_planner_t* rp )
{
    smtc_real_region_types_t smtc_real_region_types = SMTC_REAL_REGION_UNKNOWN;

#if defined( REGION_EU_868 )
    smtc_real_region_types = SMTC_REAL_REGION_EU_868;
#elif defined( REGION_US_915 )
    smtc_real_region_types = SMTC_REAL_REGION_US_915;
#elif defined( REGION_AU_915 )
    smtc_real_region_types = SMTC_REAL_REGION_AU_915;
#elif defined( REGION_CN_470 )
    smtc_real_region_types = SMTC_REAL_REGION_CN_470;
#elif defined( REGION_CN_470_RP_1_0 )
    smtc_real_region_types = SMTC_REAL_REGION_CN_470_RP_1_0;
#elif defined( REGION_AS_923 )
    smtc_real_region_types = SMTC_REAL_REGION_AS_923;
#elif defined( REGION_IN_865 )
    smtc_real_region_types = SMTC_REAL_REGION_IN_865;
#elif defined( REGION_KR_920 )
    smtc_real_region_types = SMTC_REAL_REGION_KR_920;
#elif defined( REGION_RU_864 )
    smtc_real_region_types = SMTC_REAL_REGION_RU_864;
#elif defined( REGION_WW2G4 )
    smtc_real_region_types = SMTC_REAL_REGION_WW2G4;
#else
#error "Please select supported region"
#endif

    // init lr1mac core
    lr1mac_core_init( &lr1_mac_obj, &real, &lbt_obj, &duty_cycle_obj, rp, ACTIVATION_MODE_OTAA, smtc_real_region_types,
                      ( void ( * )( void* ) ) lorawan_api_class_a_downlink_callback, &lr1_mac_obj );

    fifo_ctrl_init( &fifo_ctrl_obj, fifo_buffer, FIFO_LORAWAN_SIZE );

#if defined( SMTC_MULTICAST )
    smtc_multicast_init( &multicast_obj );

    lr1mac_class_c_init( &class_c_obj, &lr1_mac_obj, &multicast_obj, rp, RP_HOOK_ID_CLASS_C,
                         ( void ( * )( void* ) ) lr1mac_class_c_mac_rp_callback, &class_c_obj,
                         ( void ( * )( void* ) ) lorawan_api_class_c_downlink_callback, &class_c_obj );
    smtc_ping_slot_init( &ping_slot_obj, &lr1_mac_obj, &multicast_obj, rp, RP_HOOK_ID_CLASS_B_PING_SLOT,
                         ( void ( * )( void* ) ) smtc_ping_slot_mac_rp_callback, &ping_slot_obj,
                         ( void ( * )( void* ) ) lorawan_api_class_b_downlink_callback, &ping_slot_obj );
#else
    lr1mac_class_c_init( &class_c_obj, &lr1_mac_obj, NULL, rp, RP_HOOK_ID_CLASS_C,
                         ( void ( * )( void* ) ) lr1mac_class_c_mac_rp_callback, &class_c_obj,
                         ( void ( * )( void* ) ) lorawan_api_class_c_downlink_callback, &class_c_obj );
    smtc_ping_slot_init( &ping_slot_obj, &lr1_mac_obj, NULL, rp, RP_HOOK_ID_CLASS_B_PING_SLOT,
                         ( void ( * )( void* ) ) smtc_ping_slot_mac_rp_callback, &ping_slot_obj,
                         ( void ( * )( void* ) ) lorawan_api_class_b_downlink_callback, &ping_slot_obj );
#endif

    smtc_beacon_sniff_init( &lr1_beacon_obj, &ping_slot_obj, &lr1_mac_obj, rp, RP_HOOK_ID_CLASS_B_BEACON,
                            ( void ( * )( void* ) ) lorawan_api_class_b_beacon_callback, &lr1_beacon_obj );

#if defined( SMTC_D2D )
    smtc_class_b_d2d_init( &class_b_d2d_obj, &ping_slot_obj, RP_HOOK_ID_CLASS_B_D2D,
                           ( void ( * )( void* ) ) lorawan_api_class_b_d2d_tx_event_callback, &class_b_d2d_obj );
#endif

    lorawan_certification_init( &lorawan_certif_obj );
}

void lorawan_api_class_a_downlink_callback( lr1_stack_mac_t* lr1_mac_object )
{
    if( modem_supervisor_update_downlink_frame( lr1_mac_object->rx_payload, lr1_mac_object->rx_payload_size,
                                                &( lr1_mac_object->rx_metadata ), false ) )
    {
        if( fifo_ctrl_set( &fifo_ctrl_obj, lr1_mac_object->rx_payload, lr1_mac_object->rx_payload_size,
                           &( lr1_mac_object->rx_metadata ), sizeof( lr1mac_down_metadata_t ) ) != FIFO_STATUS_OK )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Fifo problem\n" );
            return;
        }
        else
        {
            fifo_ctrl_print_stat( &fifo_ctrl_obj );
        }
    }
}

void lorawan_api_class_c_downlink_callback( lr1mac_class_c_t* class_c_object )
{
    if( modem_supervisor_update_downlink_frame( class_c_object->rx_payload, class_c_object->rx_payload_size,
                                                &( class_c_object->rx_metadata ), class_c_object->tx_ack_bit ) )
    {
        if( fifo_ctrl_set( &fifo_ctrl_obj, class_c_object->rx_payload, class_c_object->rx_payload_size,
                           &( class_c_object->rx_metadata ), sizeof( lr1mac_down_metadata_t ) ) != FIFO_STATUS_OK )
        {
            smtc_modem_hal_mcu_panic( "Fifo problem\n" );
            return;
        }
        else
        {
            fifo_ctrl_print_stat( &fifo_ctrl_obj );
        }
    }
}
void lorawan_api_class_b_downlink_callback( smtc_ping_slot_t* class_b_object )
{
    if( modem_supervisor_update_downlink_frame( class_b_object->rx_payload, class_b_object->rx_payload_size,
                                                &( class_b_object->rx_metadata ), class_b_object->tx_ack_bit ) )
    {
        if( fifo_ctrl_set( &fifo_ctrl_obj, class_b_object->rx_payload, class_b_object->rx_payload_size,
                           &( class_b_object->rx_metadata ), sizeof( lr1mac_down_metadata_t ) ) != FIFO_STATUS_OK )
        {
            smtc_modem_hal_mcu_panic( "Fifo problem\n" );
            return;
        }
        else
        {
            fifo_ctrl_print_stat( &fifo_ctrl_obj );
        }
    }
}

void lorawan_api_class_b_beacon_callback( smtc_lr1_beacon_t* class_b_beacon_object )
{
    if( modem_supervisor_update_downlink_frame( class_b_beacon_object->beacon_buffer,
                                                class_b_beacon_object->beacon_buffer_length,
                                                &( class_b_beacon_object->beacon_metadata.rx_metadata ), 0 ) )
    {
        if( fifo_ctrl_set( &fifo_ctrl_obj, class_b_beacon_object->beacon_buffer,
                           class_b_beacon_object->beacon_buffer_length,
                           &( class_b_beacon_object->beacon_metadata.rx_metadata ),
                           sizeof( lr1mac_down_metadata_t ) ) != FIFO_STATUS_OK )
        {
            smtc_modem_hal_mcu_panic( "Fifo problem\n" );
            return;
        }
        else
        {
            fifo_ctrl_print_stat( &fifo_ctrl_obj );
        }
    }
}

#if defined( SMTC_D2D )
void lorawan_api_class_b_d2d_tx_event_callback( smtc_class_b_d2d_t* class_b_d2d_object )
{
    // All transmission(s) performed set to true
    bool tx_done = ( class_b_d2d_object->nb_trans_cnt == 0 ) ? true : false;

    modem_context_set_class_b_d2d_last_metadata( class_b_d2d_object->multi_cast_group_id, tx_done,
                                                 class_b_d2d_object->nb_trans_cnt );
}
#endif

smtc_real_region_types_t lorawan_api_get_region( void )
{
    return lr1mac_core_get_region( &lr1_mac_obj );
}

status_lorawan_t lorawan_api_set_region( smtc_real_region_types_t region_type )
{
    return lr1mac_core_set_region( &lr1_mac_obj, region_type );
}

status_lorawan_t lorawan_api_payload_send( uint8_t fport, bool fport_enabled, const uint8_t* data, uint8_t data_len,
                                           uint8_t packet_type, uint32_t target_time_ms )
{
    return lr1mac_core_payload_send( &lr1_mac_obj, fport, fport_enabled, data, data_len, packet_type, target_time_ms );
}

status_lorawan_t lorawan_api_payload_send_at_time( uint8_t fport, bool fport_enabled, const uint8_t* data,
                                                   uint8_t data_len, uint8_t packet_type, uint32_t target_time_ms )
{
    return lr1mac_core_payload_send_at_time( &lr1_mac_obj, fport, fport_enabled, data, data_len, packet_type,
                                             target_time_ms );
}

status_lorawan_t lorawan_api_send_stack_cid_req( cid_from_device_t cid_req )
{
    return lr1mac_core_send_stack_cid_req( &lr1_mac_obj, cid_req );
}

status_lorawan_t lorawan_api_join( uint32_t target_time_ms )
{
    return lr1mac_core_join( &lr1_mac_obj, target_time_ms );
}

join_status_t lorawan_api_isjoined( void )
{
    return lr1_mac_joined_status_get( &lr1_mac_obj );
}

void lorawan_api_join_status_clear( void )
{
    lr1mac_core_join_status_clear( &lr1_mac_obj );
}

status_lorawan_t lorawan_api_dr_strategy_set( dr_strategy_t dr_strategy )
{
    return ( lr1mac_core_dr_strategy_set( &lr1_mac_obj, dr_strategy ) );
}

dr_strategy_t lorawan_api_dr_strategy_get( void )
{
    return lr1mac_core_dr_strategy_get( &lr1_mac_obj );
}

void lorawan_api_dr_custom_set( uint32_t* custom_dr )
{
    lr1mac_core_dr_custom_set( &lr1_mac_obj, custom_dr );
}

lr1mac_states_t lorawan_api_process( void )
{
    return lr1mac_core_process( &lr1_mac_obj );
}

void lorawan_api_context_load( void )
{
    lr1mac_core_context_load( &lr1_mac_obj );
}

void lorawan_api_context_save( void )
{
    lr1mac_core_context_save( &lr1_mac_obj );
}

int16_t lorawan_api_last_snr_get( void )
{
    return lr1mac_core_last_snr_get( &lr1_mac_obj );
}

int16_t lorawan_api_last_rssi_get( void )
{
    return lr1mac_core_last_rssi_get( &lr1_mac_obj );
}

void lorawan_api_factory_reset( void )
{
    lr1mac_core_factory_reset( &lr1_mac_obj );
}

lr1mac_activation_mode_t lorawan_api_get_activation_mode( void )
{
    return lr1mac_core_get_activation_mode( &lr1_mac_obj );
}

void lorawan_api_set_activation_mode( lr1mac_activation_mode_t activation_mode )
{
    lr1mac_core_set_activation_mode( &lr1_mac_obj, activation_mode );
}

uint32_t lorawan_api_next_max_payload_length_get( void )
{
    return lr1mac_core_next_max_payload_length_get( &lr1_mac_obj );
}

uint32_t lorawan_api_devaddr_get( void )
{
    return lr1mac_core_devaddr_get( &lr1_mac_obj );
}

status_lorawan_t lorawan_api_get_deveui( uint8_t* dev_eui )
{
    status_lorawan_t rc = OKLORAWAN;

    if( smtc_secure_element_get_deveui( dev_eui ) != SMTC_SE_RC_SUCCESS )
    {
        rc = ERRORLORAWAN;
    }
    return rc;
}

status_lorawan_t lorawan_api_set_deveui( const uint8_t* dev_eui )
{
    status_lorawan_t rc = OKLORAWAN;

    if( smtc_secure_element_set_deveui( dev_eui ) != SMTC_SE_RC_SUCCESS )
    {
        rc = ERRORLORAWAN;
    }
    return rc;
}

status_lorawan_t lorawan_api_set_appkey( const uint8_t* app_key )
{
    // in lorawan 1.0.x SMTC_SE_NWK_KEY is for APP_KEY
    if( smtc_modem_crypto_set_key( SMTC_SE_NWK_KEY, app_key ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        return ERRORLORAWAN;
    }
    // in lorawan 1.0.x SMTC_SE_APP_KEY is for GEN_APP_KEY(useful in case of multicast features)
    if( smtc_modem_crypto_set_key( SMTC_SE_APP_KEY, app_key ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        return ERRORLORAWAN;
    }
    return OKLORAWAN;
}

status_lorawan_t lorawan_api_get_joineui( uint8_t* join_eui )
{
    status_lorawan_t rc = OKLORAWAN;
    if( smtc_secure_element_get_joineui( join_eui ) != SMTC_SE_RC_SUCCESS )
    {
        rc = ERRORLORAWAN;
    }
    return rc;
}

status_lorawan_t lorawan_api_set_joineui( const uint8_t* join_eui )
{
    status_lorawan_t rc = OKLORAWAN;
    if( smtc_secure_element_set_joineui( join_eui ) != SMTC_SE_RC_SUCCESS )
    {
        rc = ERRORLORAWAN;
    }
    return rc;
}

uint8_t lorawan_api_next_power_get( void )
{
    return lr1mac_core_next_power_get( &lr1_mac_obj );
}

uint8_t lorawan_api_next_dr_get( void )
{
    return lr1mac_core_next_dr_get( &lr1_mac_obj );
}

uint32_t lorawan_api_next_frequency_get( void )
{
    return lr1mac_core_next_frequency_get( &lr1_mac_obj );
}

uint8_t lorawan_api_max_tx_dr_get( void )
{
    return smtc_real_get_max_tx_channel_dr( &lr1_mac_obj );
}

uint16_t lorawan_api_mask_tx_dr_channel_up_dwell_time_check( void )
{
    return smtc_real_mask_tx_dr_channel_up_dwell_time_check( &lr1_mac_obj );
}

uint8_t lorawan_api_min_tx_dr_get( void )
{
    return smtc_real_get_min_tx_channel_dr( &lr1_mac_obj );
}

lr1mac_states_t lorawan_api_state_get( void )
{
    return lr1mac_core_state_get( &lr1_mac_obj );
}

uint16_t lorawan_api_nb_reset_get( void )
{
    return lr1mac_core_nb_reset_get( &lr1_mac_obj );
}

uint16_t lorawan_api_devnonce_get( void )
{
    return lr1mac_core_devnonce_get( &lr1_mac_obj );
}

receive_win_t lorawan_api_rx_window_get( void )
{
    return lr1mac_core_rx_window_get( &lr1_mac_obj );
}

uint32_t lorawan_api_next_join_time_second_get( void )
{
    return lr1mac_core_next_join_time_second_get( &lr1_mac_obj );
}

int32_t lorawan_api_next_free_duty_cycle_ms_get( void )
{
    return lr1mac_core_next_free_duty_cycle_ms_get( &lr1_mac_obj );
}

status_lorawan_t lorawan_api_duty_cycle_enable_set( smtc_dtc_enablement_type_t dtc_type )
{
    if( smtc_duty_cycle_enable_set( lr1_mac_obj.dtc_obj, dtc_type ) == true )
    {
        return OKLORAWAN;
    }
    return ERRORLORAWAN;
}

smtc_dtc_enablement_type_t lorawan_api_duty_cycle_enable_get( void )
{
    return smtc_duty_cycle_enable_get( lr1_mac_obj.dtc_obj );
}

uint32_t lorawan_api_fcnt_up_get( void )
{
    return lr1mac_core_fcnt_up_get( &lr1_mac_obj );
}

uint8_t lorawan_api_rp_hook_id_get( void )
{
    uint8_t hook_id;
    rp_hook_get_id( lr1mac_core_rp_get( &lr1_mac_obj ), &lr1_mac_obj, &hook_id );
    return hook_id;
}

void lorawan_api_class_c_enabled( bool enable )
{
    lr1mac_class_c_enabled( &class_c_obj, enable );

    if( lorawan_api_isjoined( ) == JOINED )
    {
        lr1mac_class_c_start( &class_c_obj );
    }
}

void lorawan_api_class_c_start( void )
{
    lr1mac_class_c_start( &class_c_obj );
}

void lorawan_api_class_c_stop( void )
{
    lr1mac_class_c_stop( &class_c_obj );
}

lorawan_multicast_rc_t lorawan_api_multicast_set_group_session_keys( uint8_t       mc_group_id,
                                                                     const uint8_t mc_ntw_skey[LORAWAN_KEY_SIZE],
                                                                     const uint8_t mc_app_skey[LORAWAN_KEY_SIZE] )
{
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) smtc_multicast_set_group_keys( &multicast_obj, mc_group_id, mc_ntw_skey,
                                                                     mc_app_skey );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_set_group_address( uint8_t mc_group_id, uint32_t mc_group_address )
{
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) smtc_multicast_set_group_address( &multicast_obj, mc_group_id, mc_group_address );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_get_group_address( uint8_t mc_group_id, uint32_t* mc_group_address )
{
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) smtc_multicast_get_group_address( &multicast_obj, mc_group_id, mc_group_address );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_get_running_status( uint8_t mc_group_id, bool* session_running )
{
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) smtc_multicast_get_running_status( &multicast_obj, mc_group_id, session_running );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_c_get_session_status( uint8_t mc_group_id, bool* is_session_started,
                                                                   uint32_t* freq, uint8_t* dr )
{
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) lr1mac_class_c_multicast_get_session_status( &class_c_obj, mc_group_id,
                                                                                   is_session_started, freq, dr );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_c_start_session( uint8_t mc_group_id, uint32_t freq, uint8_t dr )
{
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) lr1mac_class_c_multicast_start_session( &class_c_obj, mc_group_id, freq, dr );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_c_stop_session( uint8_t mc_group_id )
{
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) lr1mac_class_c_multicast_stop_session( &class_c_obj, mc_group_id );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_c_stop_all_sessions( void )
{
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) lr1mac_class_c_multicast_stop_all_sessions( &class_c_obj );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_b_get_session_status( uint8_t mc_group_id, bool* is_session_started,
                                                                   bool* waiting_beacon_to_start, uint32_t* freq,
                                                                   uint8_t* dr, uint8_t* ping_slot_periodicity )
{
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) smtc_ping_slot_multicast_b_get_session_status(
        &ping_slot_obj, mc_group_id, is_session_started, waiting_beacon_to_start, freq, dr, ping_slot_periodicity );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_b_start_session( uint8_t mc_group_id, uint32_t freq, uint8_t dr,
                                                              uint8_t ping_slot_periodicity )
{
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) smtc_ping_slot_multicast_b_start_session( &ping_slot_obj, mc_group_id, freq, dr,
                                                                                ping_slot_periodicity );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_b_stop_session( uint8_t mc_group_id )
{
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) smtc_ping_slot_multicast_b_stop_session( &ping_slot_obj, mc_group_id );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

lorawan_multicast_rc_t lorawan_api_multicast_b_stop_all_sessions( void )
{
#if defined( SMTC_MULTICAST )
    return ( lorawan_multicast_rc_t ) smtc_ping_slot_multicast_b_stop_all_sessions( &ping_slot_obj );
#else
    return LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED;
#endif
}

uint8_t lorawan_api_rx_ack_bit_get( void )
{
    return lr1mac_core_rx_ack_bit_get( &lr1_mac_obj );
}

uint8_t lorawan_api_rx_fpending_bit_get( void )
{
    return lr1mac_core_rx_fpending_bit_get( &lr1_mac_obj );
}

void lorawan_api_set_no_rx_packet_threshold( uint16_t no_rx_packet_reset_threshold )
{
    lr1mac_core_set_no_rx_packet_threshold( &lr1_mac_obj, no_rx_packet_reset_threshold );
}

uint16_t lorawan_api_get_no_rx_packet_threshold( void )
{
    return lr1mac_core_get_no_rx_packet_threshold( &lr1_mac_obj );
}

uint16_t lorawan_api_get_current_adr_ack_cnt( void )
{
    return lr1mac_core_get_current_adr_ack_cnt( &lr1_mac_obj );
}

void lorawan_api_reset_no_rx_packet_in_mobile_mode_cnt( void )
{
    lr1mac_core_reset_no_rx_packet_in_mobile_mode_cnt( &lr1_mac_obj );
}

uint16_t lorawan_api_get_current_no_rx_packet_in_mobile_mode_cnt( void )
{
    return lr1mac_core_get_current_no_rx_packet_in_mobile_mode( &lr1_mac_obj );
}

uint16_t lorawan_api_get_current_no_rx_packet_cnt( void )
{
    return lr1mac_core_get_current_no_rx_packet_cnt( &lr1_mac_obj );
}

void lorawan_api_modem_certification_set( uint8_t enable )
{
    lr1mac_core_certification_set( &lr1_mac_obj, enable );
    lorawan_certification_set_enabled( &lorawan_certif_obj, enable );
    lorawan_api_set_status_push_network_downlink_to_user( enable );
}

bool lorawan_api_certification_is_enabled( void )
{
    return lorawan_certification_get_enabled( &lorawan_certif_obj );
}

uint16_t lorawan_api_certification_get_ul_periodicity( void )
{
    return lorawan_certification_get_ul_periodicity( &lorawan_certif_obj );
}

bool lorawan_api_certification_get_frame_type( void )
{
    return lorawan_certification_get_frame_type( &lorawan_certif_obj );
}

void lorawan_api_certification_get_cw_config( uint16_t* timeout_s, uint32_t* frequency, int8_t* tx_power )
{
    lorawan_certification_get_cw_config( &lorawan_certif_obj, timeout_s, frequency, tx_power );
}

bool lorawan_api_certification_is_cw_running( void )
{
    return lorawan_certification_is_cw_running( &lorawan_certif_obj );
}

void lorawan_api_certification_cw_set_as_stopped( void )
{
    lorawan_certification_cw_set_as_stopped( &lorawan_certif_obj );
}

bool lorawan_api_certification_get_beacon_rx_status_ind_ctrl( void )
{
    return lorawan_certification_get_beacon_rx_status_ind_ctrl( &lorawan_certif_obj );
}

uint8_t lorawan_api_modem_certification_is_enabled( void )
{
    return lr1mac_core_certification_get( &lr1_mac_obj );
}

lorawan_certification_class_t lorawan_api_certification_get_requested_class( void )
{
    return lorawan_certification_get_requested_class( &lorawan_certif_obj );
}

lorawan_certification_parser_ret_t lorawan_api_certification( uint8_t* rx_buffer, uint8_t rx_buffer_length,
                                                              uint8_t* tx_buffer, uint8_t* tx_buffer_length,
                                                              uint8_t* tx_fport )
{
    return lorawan_certification_parser( &lorawan_certif_obj, rx_buffer, rx_buffer_length, tx_buffer, tx_buffer_length,
                                         tx_fport );
}

void lorawan_api_certification_build_beacon_rx_status_ind( uint8_t* beacon_buffer, uint8_t beacon_buffer_length,
                                                           uint8_t* tx_buffer, uint8_t* tx_buffer_length, int8_t rssi,
                                                           int8_t snr, uint8_t beacon_dr, uint32_t beacon_freq )
{
    lorawan_certification_build_beacon_rx_status_ind( &lorawan_certif_obj, beacon_buffer, beacon_buffer_length,
                                                      tx_buffer, tx_buffer_length, rssi, snr, beacon_dr, beacon_freq );
}
/*!
 * \brief  return true if stack receive a link adr request
 * \remark reset the flag automatically each time the upper layer call this function
 * \param [in]  void
 * \param [out] bool
 */
bool lorawan_api_available_link_adr_get( void )
{
    return lr1mac_core_available_link_adr_get( &lr1_mac_obj );
}
lr1_stack_mac_t* lorawan_api_stack_mac_get( void )
{
    return ( &lr1_mac_obj );
}

fifo_ctrl_t* lorawan_api_get_fifo_obj( void )
{
    return &fifo_ctrl_obj;
}

void lorawan_api_set_network_type( bool network_type )
{
    uint8_t sync_word = ( network_type == true ) ? smtc_real_get_public_sync_word( &lr1_mac_obj )
                                                 : smtc_real_get_private_sync_word( &lr1_mac_obj );

    smtc_real_set_sync_word( &lr1_mac_obj, sync_word );
}
bool lorawan_api_get_network_type( void )
{
    uint8_t sync_word = smtc_real_get_sync_word( &lr1_mac_obj );
    return ( ( sync_word == smtc_real_get_public_sync_word( &lr1_mac_obj ) ) ? true : false );
}

uint8_t lorawan_api_nb_trans_get( void )
{
    return lr1_stack_nb_trans_get( &lr1_mac_obj );
}

status_lorawan_t lorawan_api_nb_trans_set( uint8_t nb_trans )
{
    return lr1_stack_nb_trans_set( &lr1_mac_obj, nb_trans );
}

uint32_t lorawan_api_get_crystal_error( void )
{
    return lr1_stack_get_crystal_error( &lr1_mac_obj );
}

void lorawan_api_set_crystal_error( uint32_t crystal_error )
{
    lr1_stack_set_crystal_error( &lr1_mac_obj, crystal_error );
}

lr1mac_version_t lorawan_api_get_spec_version( void )
{
    return lr1mac_core_get_lorawan_version( &lr1_mac_obj );
}

lr1mac_version_t lorawan_api_get_regional_parameters_version( void )
{
    return smtc_real_get_regional_parameters_version( );
}

bool lorawan_api_convert_rtc_to_gps_epoch_time( uint32_t rtc_ms, uint32_t* seconds_since_epoch,
                                                uint32_t* fractional_second )
{
    return lr1mac_core_convert_rtc_to_gps_epoch_time( &lr1_mac_obj, rtc_ms, seconds_since_epoch, fractional_second );
}

bool lorawan_api_is_time_valid( void )
{
    return lr1mac_core_is_time_valid( &lr1_mac_obj );
}

uint32_t lorawan_api_get_timestamp_last_device_time_ans_s( void )
{
    return lr1mac_core_get_timestamp_last_device_time_ans_s( &lr1_mac_obj );
}

uint32_t lorawan_api_get_time_left_connection_lost( void )
{
    return lr1mac_core_get_time_left_connection_lost( &lr1_mac_obj );
}

void lorawan_api_set_device_time_callback( void ( *device_time_callback )( void* context, uint32_t rx_timestamp_s ),
                                           void* context, uint32_t rx_timestamp_s )
{
    lr1mac_core_set_device_time_callback( &lr1_mac_obj, ( void ( * )( void*, uint32_t ) ) device_time_callback, context,
                                          rx_timestamp_s );
}

status_lorawan_t lorawan_api_set_device_time_invalid_delay_s( uint32_t delay_s )
{
    return lr1_mac_core_set_device_time_invalid_delay_s( &lr1_mac_obj, delay_s );
}

uint32_t lorawan_api_get_device_time_invalid_delay_s( void )
{
    return lr1_mac_core_get_device_time_invalid_delay_s( &lr1_mac_obj );
}

status_lorawan_t lorawan_api_get_link_check_ans( uint8_t* margin, uint8_t* gw_cnt )
{
    return lr1_mac_core_get_link_check_ans( &lr1_mac_obj, margin, gw_cnt );
}

status_lorawan_t lorawan_api_get_device_time_req_status( void )
{
    return lr1_mac_core_get_device_time_req_status( &lr1_mac_obj );
}

void lorawan_api_lbt_set_parameters( uint32_t listen_duration_ms, int16_t threshold_dbm, uint32_t bw_hz )
{
    smtc_lbt_set_parameters( &lbt_obj, listen_duration_ms, threshold_dbm, bw_hz );
}

void lorawan_api_lbt_get_parameters( uint32_t* listen_duration_ms, int16_t* threshold_dbm, uint32_t* bw_hz )
{
    smtc_lbt_get_parameters( &lbt_obj, listen_duration_ms, threshold_dbm, bw_hz );
}

void lorawan_api_lbt_set_state( bool enable )
{
    smtc_lbt_set_state( &lbt_obj, enable );
}

bool lorawan_api_lbt_get_state( void )
{
    return smtc_lbt_get_state( &lbt_obj );
}

void lorawan_api_class_b_enabled( bool enable )
{
    smtc_beacon_class_b_enable_service( &lr1_beacon_obj, enable );

    if( ( lorawan_api_isjoined( ) == JOINED ) && ( enable == true ) )
    {
        smtc_beacon_sniff_start( &lr1_beacon_obj );
    }
}

void lorawan_api_beacon_sniff_start( void )
{
    smtc_beacon_sniff_start( &lr1_beacon_obj );
}

void lorawan_api_beacon_sniff_stop( void )
{
    smtc_beacon_sniff_stop( &lr1_beacon_obj );
}

void lorawan_api_beacon_get_metadata( smtc_beacon_metadata_t* beacon_metadata )
{
    smtc_beacon_sniff_get_metadata( &lr1_beacon_obj, beacon_metadata );
}

status_lorawan_t lorawan_api_get_ping_slot_info_req_status( void )
{
    return lr1_mac_core_get_ping_slot_info_req_status( &lr1_mac_obj );
}

status_lorawan_t lorawan_api_set_ping_slot_periodicity( uint8_t ping_slot_periodicity )
{
    return lr1mac_core_set_ping_slot_periodicity( &lr1_mac_obj, ping_slot_periodicity );
}

uint8_t lorawan_api_get_ping_slot_periodicity( void )
{
    return lr1mac_core_get_ping_slot_periodicity( &lr1_mac_obj );
}

bool lorawan_api_get_class_b_status( void )
{
    return lr1mac_core_get_class_b_status( &lr1_mac_obj );
}

void lorawan_api_lora_dr_to_sf_bw( uint8_t in_dr, uint8_t* out_sf, lr1mac_bandwidth_t* out_bw )
{
    smtc_real_lora_dr_to_sf_bw( &lr1_mac_obj, in_dr, out_sf, out_bw );
}

uint8_t lorawan_api_get_frequency_factor( void )
{
    return smtc_real_get_frequency_factor( &lr1_mac_obj );
}

bool lorawan_api_get_status_push_network_downlink_to_user( void )
{
    return lr1mac_core_get_status_push_network_downlink_to_user( &lr1_mac_obj );
}

void lorawan_api_set_status_push_network_downlink_to_user( bool enable )
{
    lr1mac_core_set_status_push_network_downlink_to_user( &lr1_mac_obj, enable );
}

status_lorawan_t lorawan_api_set_adr_ack_limit_delay( uint8_t adr_ack_limit, uint8_t adr_ack_delay )
{
    return lr1mac_core_set_adr_ack_limit_delay( &lr1_mac_obj, adr_ack_limit, adr_ack_delay );
}

void lorawan_api_get_adr_ack_limit_delay( uint8_t* adr_ack_limit, uint8_t* adr_ack_delay )
{
    lr1mac_core_get_adr_ack_limit_delay( &lr1_mac_obj, adr_ack_limit, adr_ack_delay );
}

smtc_class_b_d2d_status_t lorawan_api_class_b_d2d_request_tx( rx_session_type_t multi_cast_group_id, uint8_t fport,
                                                              uint8_t priority, const uint8_t* payload,
                                                              uint8_t payload_size, uint8_t nb_rep,
                                                              uint16_t nb_ping_slot_tries, uint8_t* ping_slots_mask,
                                                              uint8_t ping_slots_mask_size )
{
#if defined( SMTC_D2D )
    return smtc_class_b_d2d_request_tx( &class_b_d2d_obj, multi_cast_group_id, fport, priority, payload, payload_size,
                                        nb_rep, nb_ping_slot_tries, ping_slots_mask, ping_slots_mask_size );
#else
    return SMTC_CLASS_B_D2D_ERROR;
#endif
}

uint8_t lorawan_api_class_b_d2d_next_max_payload_length_get( rx_session_type_t multi_cast_group_id )
{
#if defined( SMTC_D2D )
    return smtc_class_b_d2d_next_max_payload_length_get( &class_b_d2d_obj, multi_cast_group_id );
#else
    return 0;
#endif
}

#if defined( LR1110_MODEM_E )
void lorawan_rp_callback_api( radio_planner_t* rp )
{
    rp_radio_irq_callback( rp );
}
#endif  // LR1110_MODEM_E
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
