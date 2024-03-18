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
#define UNUSED_VALUE 0xff
#ifdef ADD_CLASS_B
#define NUMBER_OF_LORAWAN_MANAGEMENT_TASKS 5
#else
#define NUMBER_OF_LORAWAN_MANAGEMENT_TASKS 4
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct modem_ctx_s
{
    uint32_t reset_counter;
    uint8_t  cloud_dm_port;
    uint8_t  rfu[7];
    uint32_t crc;  // !! crc MUST be the last field of the structure !!
} modem_ctx_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
struct
{
    modem_downlink_msg_t modem_dwn_pkt;
    radio_planner_t*     modem_rp;
    const void*          modem_radio_ctx;  // save lr11xx user radio context needed to perform direct access to radio
    bool                 is_modem_in_test_mode;
    uint32_t             user_alarm;
    fifo_ctrl_t          fifo_ctrl_obj;
    uint8_t              fifo_buffer[FIFO_LORAWAN_SIZE];
    uint8_t              ( *downlink_services_callback[NUMBER_OF_SERVICES + NUMBER_OF_LORAWAN_MANAGEMENT_TASKS] )(
        lr1_stack_mac_down_data_t* rx_down_data );
    uint32_t modem_reset_counter;
} modem_ctx_light;

#define modem_dwn_pkt modem_ctx_light.modem_dwn_pkt
#define modem_rp modem_ctx_light.modem_rp
#define modem_radio_ctx modem_ctx_light.modem_radio_ctx
#define is_modem_in_test_mode modem_ctx_light.is_modem_in_test_mode
#define user_alarm modem_ctx_light.user_alarm
#define fifo_ctrl_obj modem_ctx_light.fifo_ctrl_obj
#define fifo_buffer modem_ctx_light.fifo_buffer
#define downlink_services_callback modem_ctx_light.downlink_services_callback
#define modem_reset_counter modem_ctx_light.modem_reset_counter

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
    void  ( *callback_on_launch_temp )( void* );
    void  ( *callback_on_update_temp )( void* );
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
        smtc_real_region_types_t region = lorawan_api_get_region( stack_id );
        lorawan_api_set_region( region, stack_id );
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

    // load modem context
    modem_load_modem_context( );
    // Increment reset counter
    modem_reset_counter++;
}

fifo_ctrl_t* modem_context_get_fifo_obj( void )
{
    return &fifo_ctrl_obj;
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

void modem_empty_callback( void* ctx )
{
    // Use for suspend/resume radio access
}

bool modem_suspend_radio_access( void )
{
    rp_radio_params_t fake_radio_params = { 0 };

    rp_task_t rp_task = {
        .hook_id                    = RP_HOOK_ID_SUSPEND,
        .type                       = RP_TASK_TYPE_NONE,
        .launch_task_callbacks      = modem_empty_callback,
        .schedule_task_low_priority = false,
        .start_time_ms              = smtc_modem_hal_get_time_in_ms( ) + 4,
        .duration_time_ms           = 20000,
        .state                      = RP_TASK_STATE_SCHEDULE,

    };

    rp_hook_status_t status = rp_task_enqueue( modem_rp, &rp_task, NULL, 0, &fake_radio_params );

    if( status != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to suspend radio accesswith following error code: %x\n", status );
        return false;
    }
    return true;
}

bool modem_resume_radio_access( void )
{
    bool status = true;

    if( rp_task_abort( modem_rp, RP_HOOK_ID_SUSPEND ) == RP_HOOK_STATUS_OK )
    {
        // force a call of rp_callback to re-arbitrate the radio planner before the next loop
        rp_callback( modem_rp );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to abort suspend/resume hook\n" );
        status = false;
    }

    return status;
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

uint8_t modem_get_status( uint8_t stack_id )
{
    uint8_t modem_status = 0;

    modem_status = ( smtc_modem_hal_crashlog_get_status( ) == true ) ? ( modem_status | SMTC_MODEM_STATUS_CRASH )
                                                                     : ( modem_status & ~SMTC_MODEM_STATUS_CRASH );

#if defined( ADD_SMTC_CLOUD_DEVICE_MANAGEMENT )
    modem_status =
        ( ( modem_supervisor_get_modem_mute_with_priority_parameter( stack_id ) == TASK_VERY_HIGH_PRIORITY ) )
            ? ( modem_status | SMTC_MODEM_STATUS_MUTE )
            : ( modem_status & ~SMTC_MODEM_STATUS_MUTE );
#endif

    modem_status = ( lorawan_api_isjoined( stack_id ) == JOINED ) ? ( modem_status | SMTC_MODEM_STATUS_JOINED )
                                                                  : ( modem_status & ~SMTC_MODEM_STATUS_JOINED );

    modem_status = ( modem_supervisor_get_modem_is_suspended( stack_id ) == true )
                       ? ( modem_status | SMTC_MODEM_STATUS_SUSPEND )
                       : ( modem_status & ~SMTC_MODEM_STATUS_SUSPEND );

#if defined( ADD_SMTC_LFU )
    modem_status = ( file_upload_get_status( stack_id ) == true ) ? ( modem_status | SMTC_MODEM_STATUS_UPLOAD )
                                                                  : ( modem_status & ~SMTC_MODEM_STATUS_UPLOAD );
#endif

    modem_status = ( lorawan_api_isjoined( stack_id ) == JOINING ) ? ( modem_status | SMTC_MODEM_STATUS_JOINING )
                                                                   : ( modem_status & ~SMTC_MODEM_STATUS_JOINING );

#if defined( ADD_SMTC_STREAM )
    modem_status = ( stream_get_status( stack_id ) == true ) ? ( modem_status | SMTC_MODEM_STATUS_STREAM )
                                                             : ( modem_status & ~SMTC_MODEM_STATUS_STREAM );
#endif

    return ( modem_status );
}

void modem_store_modem_context( void )
{
    modem_ctx_t ctx = { 0 };

    // Restore current saved context
    smtc_modem_hal_context_restore( CONTEXT_MODEM, 0, ( uint8_t* ) &ctx, sizeof( ctx ) );

    // Check if some values have changed
    if( ctx.reset_counter != modem_reset_counter )
    {
        ctx.reset_counter = modem_reset_counter;
        ctx.crc           = crc( ( uint8_t* ) &ctx, sizeof( ctx ) - sizeof( ctx.crc ) );

        smtc_modem_hal_context_store( CONTEXT_MODEM, 0, ( uint8_t* ) &ctx, sizeof( ctx ) );
        // dummy context reading to ensure context store is done before exiting the function
        smtc_modem_hal_context_restore( CONTEXT_MODEM, 0, ( uint8_t* ) &ctx, sizeof( ctx ) );
    }
}

void modem_load_modem_context( void )
{
    modem_ctx_t ctx = { 0 };
    smtc_modem_hal_context_restore( CONTEXT_MODEM, 0, ( uint8_t* ) &ctx, sizeof( ctx ) );

    if( crc( ( uint8_t* ) &ctx, sizeof( ctx ) - sizeof( ctx.crc ) ) == ctx.crc )
    {
        modem_reset_counter = ctx.reset_counter;
    }
    else
    {
        ctx.reset_counter = 0;
        smtc_modem_hal_context_store( CONTEXT_MODEM, 0, ( uint8_t* ) &ctx, sizeof( ctx ) );
        // dummy context reading to ensure context store is done before exiting the function
        smtc_modem_hal_context_restore( CONTEXT_MODEM, 0, ( uint8_t* ) &ctx, sizeof( ctx ) );
    }
}

uint32_t modem_get_reset_counter( void )
{
    return modem_reset_counter;
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
