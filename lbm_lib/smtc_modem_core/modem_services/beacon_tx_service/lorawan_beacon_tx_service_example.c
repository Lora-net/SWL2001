/**
 * @file      lorawan_beacon_tx_service_example.c
 *
 * @brief     LoRaWAN Application Layer Clock Synchronization V1.0.0 Implementation
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
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
#include "lorawan_beacon_tx_service_example.h"
#include "modem_core.h"
#include "modem_supervisor_light.h"
#include "smtc_modem_api.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "lorawan_api.h"
#include "radio_planner.h"
#include "radio_planner_hook_id_defs.h"
#include "modem_tx_protocol_manager.h"
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )
#define NUMBER_MAX_OF_TX_BEACON_OBJ 1

/**
 * @brief Check is the index is valid before accessing the object
 *
 */
#define IS_VALID_OBJECT_ID( x )                                             \
    do                                                                      \
    {                                                                       \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_MAX_OF_TX_BEACON_OBJ ); \
    } while( 0 )

/**
 * @brief Check is the index is valid before accessing the object
 *
 */
#define IS_VALID_STACK_ID( x )                                   \
    do                                                           \
    {                                                            \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_OF_STACKS ); \
    } while( 0 )

#define IS_SERVICE_INITIALIZED( )                                                               \
    do                                                                                          \
    {                                                                                           \
        if( lorawan_beacon_tx_example_obj.initialized == false )                                \
        {                                                                                       \
            SMTC_MODEM_HAL_TRACE_WARNING( "lorawan_beacon_tx_example service not launched\n" ); \
            break;                                                                              \
        }                                                                                       \
    } while( 0 )
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

static lorawan_beacon_tx_example_t lorawan_beacon_tx_example_obj;
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static void start_user_beacon_callback( void* context );
static void end_user_beacon_callback( void* status );
static void start_user_beacon( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void lorawan_beacon_tx_example_service_init( uint8_t* service_id, uint8_t task_id,
                                             uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                             void ( **on_launch_callback )( void* ),
                                             void ( **on_update_callback )( void* ), void** context_callback )
{
    IS_VALID_OBJECT_ID( *service_id );
    lorawan_beacon_tx_example_obj.task_id     = task_id;
    lorawan_beacon_tx_example_obj.stack_id    = CURRENT_STACK;
    lorawan_beacon_tx_example_obj.enabled     = false;
    lorawan_beacon_tx_example_obj.initialized = true;
    *downlink_callback                        = lorawan_beacon_tx_example_service_downlink_handler;
    *on_launch_callback                       = lorawan_beacon_tx_example_service_on_launch;
    *on_update_callback                       = lorawan_beacon_tx_example_service_on_update;
    *context_callback                         = ( void* ) service_id;
    rp_hook_init( modem_get_rp( ), RP_HOOK_ID_DIRECT_RP_ACCESS, ( void ( * )( void* ) )( end_user_beacon_callback ),
                  modem_get_rp( ) );
    //  lorawan_beacon_tx_example_add_task (CURRENT_STACK);
}

void lorawan_beacon_tx_example_service_on_launch( void* context_callback )
{
    IS_SERVICE_INITIALIZED( );
    if( lorawan_api_is_time_valid( lorawan_beacon_tx_example_obj.stack_id ) == false )
    {
        uint8_t cid_buffer[]     = { DEVICE_TIME_REQ };
        uint8_t cid_request_size = 1;  // request only ping slot if time is already valid

        tx_protocol_manager_request( TX_PROTOCOL_TRANSMIT_CID, 0, false, cid_buffer, cid_request_size, 0,
                                     smtc_modem_hal_get_time_in_ms( ), lorawan_beacon_tx_example_obj.stack_id );
    }
}

void lorawan_beacon_tx_example_service_on_update( void* context_callback )
{
    IS_SERVICE_INITIALIZED( );
    if( ( lorawan_api_isjoined( lorawan_beacon_tx_example_obj.stack_id ) != JOINED ) ||
        ( lorawan_api_is_time_valid( lorawan_beacon_tx_example_obj.stack_id ) == false ) )
    {
        lorawan_beacon_tx_example_add_task( lorawan_beacon_tx_example_obj.stack_id );
    }
    else
    {
        lorawan_beacon_tx_example_obj.enabled = true;
        start_user_beacon( );
    }
}

uint8_t lorawan_beacon_tx_example_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    return MODEM_DOWNLINK_UNCONSUMED;
}

void lorawan_beacon_tx_example_add_task( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    IS_SERVICE_INITIALIZED( );
    smodem_task task       = { 0 };
    task.id                = lorawan_beacon_tx_example_obj.task_id;
    task.stack_id          = lorawan_beacon_tx_example_obj.stack_id;
    task.priority          = TASK_MEDIUM_HIGH_PRIORITY;
    task.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + 10;
    modem_supervisor_add_task( &task );
    lorawan_beacon_tx_example_obj.enabled = true;
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */
#define TX_BEACON_PREAMBLE_LENGTH_SYMB ( 10 )
#define TX_BEACON_FREQ_EU_868 ( 869525000 )  // Hz
#define TX_BEACON_PERIOD_S 128
static uint32_t tx_beacon_epoch_time = 0;
static uint16_t crc16_tx_beacon( uint8_t* data, uint16_t size )
{
    uint16_t crc = 0;
    if( size == 0 )
        return 0;

    for( unsigned int i = 0; i < size; ++i )
    {
        uint16_t dbyte = data[i];
        crc ^= dbyte << 8;

        for( unsigned char j = 0; j < 8; ++j )
        {
            uint16_t mix = crc & 0x8000;
            crc          = ( crc << 1 );
            if( mix )
                crc = crc ^ 0x1021;
        }
    }
    return crc;
}
static void start_user_beacon_callback( void* context )
{
    ralf_params_lora_t lora_param;
    memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );
    lora_param.sync_word                   = 0x34;
    lora_param.mod_params.cr               = RAL_LORA_CR_4_5;
    lora_param.pkt_params.header_type      = RAL_LORA_PKT_IMPLICIT;
    lora_param.pkt_params.pld_len_in_bytes = 17;
    lora_param.pkt_params.crc_is_on        = false;
    lora_param.pkt_params.invert_iq_is_on  = false;
    lora_param.mod_params.sf               = RAL_LORA_SF9;
    lora_param.mod_params.bw               = RAL_LORA_BW_125_KHZ;
    lora_param.rf_freq_in_hz               = TX_BEACON_FREQ_EU_868;
    lora_param.mod_params.ldro = ral_compute_lora_ldro( lora_param.mod_params.sf, lora_param.mod_params.bw );
    lora_param.pkt_params.preamble_len_in_symb = 10;

    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_set_ant_switch( true );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ralf_setup_lora( ( modem_get_rp( )->radio ), &lora_param ) == RAL_STATUS_OK );
    ral_set_dio_irq_params( &( modem_get_rp( )->radio->ral ), RAL_IRQ_TX_DONE );
    uint8_t payload[17];
    memset( payload, 0, 17 );

    payload[2]            = ( tx_beacon_epoch_time ) & 0x000000FF;
    payload[3]            = ( tx_beacon_epoch_time >> 8 ) & 0x000000FF;
    payload[4]            = ( tx_beacon_epoch_time >> 16 ) & 0x000000FF;
    payload[5]            = ( tx_beacon_epoch_time >> 24 ) & 0x000000FF;
    uint16_t computed_crc = crc16_tx_beacon( payload, 6 );
    payload[6]            = ( computed_crc ) & 0x00FF;
    payload[7]            = ( computed_crc >> 8 ) & 0x00FF;
    ral_set_pkt_payload( &( modem_get_rp( )->radio->ral ), payload, 17 );
    ral_set_tx( &( modem_get_rp( )->radio->ral ) );
}

static void end_user_beacon_callback( void* status )
{
    start_user_beacon( );
}

static void start_user_beacon( void )
{
    uint32_t gps_time_s;
    uint32_t gps_fractional_s;
    uint32_t time_ms;
    lorawan_api_convert_rtc_to_gps_epoch_time( smtc_modem_hal_get_time_in_ms( ), &gps_time_s, &gps_fractional_s,
                                               lorawan_beacon_tx_example_obj.stack_id );
    time_ms              = 1000 * ( TX_BEACON_PERIOD_S - ( gps_time_s % TX_BEACON_PERIOD_S ) );
    tx_beacon_epoch_time = gps_time_s + TX_BEACON_PERIOD_S;
    time_ms += smtc_modem_hal_get_time_in_ms( );
    time_ms -= gps_fractional_s;
    rp_task_t rp_task                      = { 0 };
    rp_task.hook_id                        = RP_HOOK_ID_DIRECT_RP_ACCESS;
    rp_task.state                          = RP_TASK_STATE_SCHEDULE;
    rp_task.start_time_ms                  = time_ms;
    rp_task.duration_time_ms               = 100;
    rp_task.type                           = RP_TASK_TYPE_TX_LORA;
    rp_task.launch_task_callbacks          = start_user_beacon_callback;
    rp_radio_params_t fake_rp_radio_params = { 0 };
    if( rp_task_enqueue( modem_get_rp( ), &rp_task, NULL, 0, &fake_rp_radio_params ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "beacon START ERROR \n" );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Tx Beacon at %d\n", tx_beacon_epoch_time );
    }
}

/* --- EOF ------------------------------------------------------------------ */
