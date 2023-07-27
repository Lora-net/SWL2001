/*!
 * \file      modem_core.c
 *
 * \brief     Utilities for modem management
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

#include "modem_core.h"
#include "modem_event_utilities.h"

#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_real.h"
#include "lorawan_api.h"
#include "smtc_modem_api.h"
#include "smtc_modem_utilities.h"
#include "smtc_duty_cycle.h"

#include "lr1mac_utilities.h"
#include "modem_supervisor_light.h"
#include "modem_services_config.h"
#include "lorawan_join_management.h"
#include "lorawan_dwn_ack_management.h"
#include "lorawan_cid_request_management.h"
#include "lorawan_beacon_tx_service_example.h"
#include "lorawan_class_b_management.h"
#include "lorawan_send_management.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#ifndef MAX
/*!
 * \brief Returns the maximum value between a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval maxValue Maximum value
 */
#define MAX( a, b ) ( ( a ) > ( b ) ) ? ( a ) : ( b )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define FIFO_LORAWAN_SIZE 512

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
#define NUMBER_OF_LORAWAN_MANAGEMENT_TASKS 5
struct
{
    modem_downlink_msg_t modem_dwn_pkt;
    radio_planner_t*     modem_rp;
    const void*          modem_radio_ctx;  // save lr11xx user radio context needed to perform direct access to radio
    bool                 is_modem_in_test_mode;
    uint32_t             user_alarm;
    fifo_ctrl_t          fifo_ctrl_obj;
    uint8_t              fifo_buffer[FIFO_LORAWAN_SIZE];
    uint8_t ( *downlink_services_callback[NUMBER_OF_SERVICES + NUMBER_OF_LORAWAN_MANAGEMENT_TASKS] )(
        lr1_stack_mac_down_data_t* rx_down_data );
} modem_ctx_light;
#define UNUSED_VALUE 0xff
#define fifo_ctrl_obj modem_ctx_light.fifo_ctrl_obj
#define fifo_buffer modem_ctx_light.fifo_buffer

#define modem_dwn_pkt modem_ctx_light.modem_dwn_pkt
#define dm_pending_dl modem_ctx_light.dm_pending_dl
#define modem_rp modem_ctx_light.modem_rp
#define modem_radio_ctx modem_ctx_light.modem_radio_ctx
#define is_modem_in_test_mode modem_ctx_light.is_modem_in_test_mode
#define user_alarm modem_ctx_light.user_alarm

#define downlink_services_callback modem_ctx_light.downlink_services_callback

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static void modem_downlink_callback( lr1_stack_mac_down_data_t* rx_down_data );
// static void check_class_b_to_generate_event( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void modem_context_init_light( void ( *callback )( void ), radio_planner_t* rp )
{
    void ( *callback_on_launch_temp )( void* );
    void ( *callback_on_update_temp )( void* );
    void* context_callback_tmp;

    modem_rp = rp;
    modem_event_init( callback );

    // Init duty-cycle object to 0
    smtc_duty_cycle_init( );

    for( uint8_t stack_id = 0; stack_id < NUMBER_OF_STACKS; stack_id++ )
    {
        lorawan_api_init( rp, stack_id, ( void ( * )( lr1_stack_mac_down_data_t* ) ) modem_downlink_callback );

        lorawan_api_dr_strategy_set( STATIC_ADR_MODE, stack_id );
        lorawan_api_join_status_clear( stack_id );

        // to init duty cycle
        smtc_modem_region_t region;
        smtc_modem_get_region( stack_id, &region );
        smtc_modem_set_region( stack_id, region );
    }

    uint8_t index_tmp = 0;
    lorawan_send_management_services_init( ( uint8_t* ) UNUSED_VALUE, UNUSED_VALUE,
                                           &downlink_services_callback[index_tmp++], &callback_on_launch_temp,
                                           &callback_on_update_temp, &context_callback_tmp );
    modem_supervisor_init_callback( SEND_TASK, callback_on_launch_temp, callback_on_update_temp, context_callback_tmp );
    lorawan_join_management_services_init( ( uint8_t* ) UNUSED_VALUE, UNUSED_VALUE,
                                           &downlink_services_callback[index_tmp++], &callback_on_launch_temp,
                                           &callback_on_update_temp, &context_callback_tmp );

    modem_supervisor_init_callback( JOIN_TASK, callback_on_launch_temp, callback_on_update_temp, context_callback_tmp );
    lorawan_dwn_ack_management_init( ( uint8_t* ) UNUSED_VALUE, UNUSED_VALUE, &downlink_services_callback[index_tmp++],
                                     &callback_on_launch_temp, &callback_on_update_temp, &context_callback_tmp );
    modem_supervisor_init_callback( RETRIEVE_DL_TASK, callback_on_launch_temp, callback_on_update_temp,
                                    context_callback_tmp );
    lorawan_cid_request_management_init( ( uint8_t* ) UNUSED_VALUE, UNUSED_VALUE,
                                         &downlink_services_callback[index_tmp++], &callback_on_launch_temp,
                                         &callback_on_update_temp, &context_callback_tmp );
    modem_supervisor_init_callback( CID_REQ_TASK, callback_on_launch_temp, callback_on_update_temp,
                                    context_callback_tmp );
#ifdef ADD_CLASS_B
    lorawan_class_b_management_services_init( ( uint8_t* ) UNUSED_VALUE, UNUSED_VALUE,
                                              &downlink_services_callback[index_tmp++], &callback_on_launch_temp,
                                              &callback_on_update_temp, &context_callback_tmp );
    modem_supervisor_init_callback( CLASS_B_MANAGEMENT_TASK, callback_on_launch_temp, callback_on_update_temp,
                                    context_callback_tmp );
#endif

    task_id_t task_id_tmp;
    uint8_t   cpt_of_services_init = SERVICE_ID0_TASK;
    for( uint8_t i = 0; i < NUMBER_OF_SERVICES; i++ )
    {
        task_id_tmp = cpt_of_services_init + ( NUMBER_OF_TASKS * modem_service_config[i].stack_id );

        modem_service_config[i].callbacks_init_service(
            &modem_service_config[i].service_id, task_id_tmp,
            &downlink_services_callback[i + NUMBER_OF_LORAWAN_MANAGEMENT_TASKS], &callback_on_launch_temp,
            &callback_on_update_temp, &context_callback_tmp );

        modem_supervisor_init_callback( cpt_of_services_init, callback_on_launch_temp, callback_on_update_temp,
                                        context_callback_tmp );
        cpt_of_services_init++;
    }

    // save radio planner pointer for suspend/resume features

    is_modem_in_test_mode = false;
    user_alarm            = 0x7FFFFFFF;
    fifo_ctrl_init( &fifo_ctrl_obj, fifo_buffer, FIFO_LORAWAN_SIZE );
}

fifo_ctrl_t* modem_context_get_fifo_obj( void )
{
    return &fifo_ctrl_obj;
}

void modem_empty_callback( void* ctx )
{
    // SMTC_MODEM_HAL_TRACE_ERROR( " empty call back \n" );
}

void modem_set_test_mode_status( bool enable )
{
    is_modem_in_test_mode = enable;
}

bool modem_get_test_mode_status( void )
{
    return is_modem_in_test_mode;
}
uint32_t modem_get_user_alarm( void )
{
    return ( user_alarm );
}
void modem_set_user_alarm( uint32_t alarm )
{
    user_alarm = alarm;
}

void modem_set_radio_ctx( const void* radio_ctx )
{
    modem_radio_ctx = radio_ctx;
}

const void* modem_get_radio_ctx( void )
{
    return modem_radio_ctx;
}
radio_planner_t* modem_get_rp( void )
{
    return modem_rp;
}
uint32_t crc( const uint8_t* buf, int len )
{
    uint32_t crc = 0xFFFFFFFF;
    while( len-- > 0 )
    {
        crc = crc ^ *buf++;
        for( int i = 0; i < 8; i++ )
        {
            uint32_t mask = -( crc & 1 );
            crc           = ( crc >> 1 ) ^ ( 0xEDB88320 & mask );
        }
    }
    return ~crc;
}

int32_t modem_duty_cycle_get_status( uint8_t stack_id )
{
    int32_t dtc_ms     = 0;
    int32_t region_dtc = 0;
    int32_t nwk_dtc    = lorawan_api_next_network_free_duty_cycle_ms_get( stack_id );

    if( smtc_duty_cycle_enable_get( ) == SMTC_DTC_ENABLED )
    {
        uint8_t  number_of_freq = 0;
        uint32_t freq_list[16]  = { 0 };  // Generally region with duty cycle support 16 channels only

        if( lorawan_api_get_current_enabled_frequencies_list(
                &number_of_freq, freq_list, sizeof( freq_list ) / sizeof( freq_list[0] ), stack_id ) == true )
        {
            region_dtc = smtc_duty_cycle_get_next_free_time_ms( number_of_freq, freq_list );
        }

        if( nwk_dtc == 0 )
        {
            dtc_ms = region_dtc;
        }
        else
        {
            dtc_ms = MAX( nwk_dtc, region_dtc );
        }
    }
    else
    {
        dtc_ms = nwk_dtc;
    }
    return dtc_ms;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void modem_downlink_callback( lr1_stack_mac_down_data_t* rx_down_data )
{
    uint8_t                  downlink_used_by_services = 0;
    smtc_modem_dl_metadata_t metadata                  = { 0 };

    metadata.stack_id     = rx_down_data->stack_id;
    metadata.snr          = rx_down_data->rx_metadata.rx_snr << 2;
    metadata.window       = ( smtc_modem_dl_window_t ) ( rx_down_data->rx_metadata.rx_window );
    metadata.fport        = rx_down_data->rx_metadata.rx_fport;
    metadata.fpending_bit = rx_down_data->rx_metadata.rx_fpending_bit;
    metadata.frequency_hz = rx_down_data->rx_metadata.rx_frequency_hz;
    metadata.datarate     = rx_down_data->rx_metadata.rx_datarate;

    if( rx_down_data->rx_metadata.rx_rssi > 63 )
    {
        metadata.rssi = 127;
    }
    else if( rx_down_data->rx_metadata.rx_rssi < -128 )
    {
        metadata.rssi = -128;
    }
    else
    {
        metadata.rssi = ( int8_t ) ( rx_down_data->rx_metadata.rx_rssi + 64 );
    }

    for( uint8_t i = 0; i < NUMBER_OF_SERVICES + NUMBER_OF_LORAWAN_MANAGEMENT_TASKS; i++ )
    {
        downlink_used_by_services += downlink_services_callback[i]( rx_down_data );
    }

    if( rx_down_data->rx_metadata.rx_window == RECEIVE_NONE )
    {
        return;
    }

    // none services used the downlink data for itself then push it into the user fifo
    if( ( downlink_used_by_services == 0 ) && ( rx_down_data->rx_metadata.rx_fport != 0 ) )
    {
        if( fifo_ctrl_set( &fifo_ctrl_obj, rx_down_data->rx_payload, rx_down_data->rx_payload_size, &metadata,
                           sizeof( smtc_modem_dl_metadata_t ) ) != FIFO_STATUS_OK )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Fifo problem\n" );
            return;
        }
        else
        {
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_DOWNDATA, 0, rx_down_data->stack_id );
            fifo_ctrl_print_stat( &fifo_ctrl_obj );
        }
    }
}

/* --- EOF ------------------------------------------------------------------ */
