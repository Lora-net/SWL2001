/**
 * @file    lorawan_relay_rx_service.c
 *
 * @brief   Relay RX service, handle forward message, automatic start/stop of the relay, ...
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2023. All rights reserved.
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
#include "lorawan_relay_rx_service.h"
#include "modem_core.h"
#include "modem_supervisor_light.h"
#include "modem_event_utilities.h"
#include "lorawan_send_management.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_api.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "lorawan_api.h"

#include "relay_rx_api.h"
#include "relay_def.h"
#include "relay_real.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )
#define NUMBER_MAX_OF_LORAWAN_RELAY_RX_OBJ 1  // modify in case of multiple obj

/**
 * @brief Check is the index is valid before accessing ALCSync object
 *
 */
#define IS_VALID_OBJECT_ID( x )                                                    \
    do                                                                             \
    {                                                                              \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_MAX_OF_LORAWAN_RELAY_RX_OBJ ); \
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

/**
 * @brief Check is the service is initialized before accessing the object
 *
 */
#define IS_SERVICE_INITIALIZED( )                                                     \
    do                                                                                \
    {                                                                                 \
        if( relay_rx_obj.initialized == false )                                       \
        {                                                                             \
            SMTC_MODEM_HAL_TRACE_WARNING( "relay_rx_obj service not initialized\n" ); \
            return;                                                                   \
        }                                                                             \
    } while( 0 )
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */
typedef enum service_fwd_e
{
    SERVICE_FWD_DONE,
    SERVICE_FWD_FWD_JOIN,
    SERVICE_FWD_FWD_UL,
    SERVICE_FWD_EMPTY_UL,
    SERVICE_FWD_DL,
} service_fwd_t;

/**
 * @brief LoRaWAN template Object
 *
 * @struct lorawan_relay_rx_s
 *
 */
typedef struct lorawan_relay_rx_s
{
    uint8_t       stack_id;
    uint8_t       task_id;
    bool          initialized;
    uint8_t       buffer[255];
    uint8_t       buffer_len;
    uint32_t      time_to_tx;
    service_fwd_t service_state;
    bool          relay_running_flag_prev;
} lorawan_relay_rx_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static lorawan_relay_rx_t relay_rx_obj = { 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
/**
 * @brief Callback called at task launch
 *
 * @param context_callback
 */
static void lorawan_relay_rx_service_on_launch( void* context );

/**
 * @brief Callback called at the end of the task
 *
 * @param context_callback
 */
static void lorawan_relay_rx_service_on_update( void* context );

/**
 * @brief Callback to handle the downlink received by the LoRaWAN layer
 *
 * @param rx_down_data
 */
static uint8_t lorawan_relay_rx_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void lorawan_relay_rx_services_init( uint8_t* service_id, uint8_t task_id,
                                     uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                     void ( **on_lunch_callback )( void* ), void ( **on_update_callback )( void* ),
                                     void** context_callback )
{
    IS_VALID_OBJECT_ID( *service_id );
    *downlink_callback                   = lorawan_relay_rx_service_downlink_handler;
    *on_lunch_callback                   = lorawan_relay_rx_service_on_launch;
    *on_update_callback                  = lorawan_relay_rx_service_on_update;
    *context_callback                    = ( void* ) service_id;
    relay_rx_obj.task_id                 = task_id;
    relay_rx_obj.stack_id                = CURRENT_STACK;
    relay_rx_obj.initialized             = true;
    relay_rx_obj.service_state           = SERVICE_FWD_DONE;
    relay_rx_obj.relay_running_flag_prev = false;

    SMTC_MODEM_HAL_TRACE_PRINTF( " lorawan_relay_rx_services_init task_id %d, service_id %d, CURRENT_STACK:%d \n",
                                 task_id, *service_id, CURRENT_STACK );

    lr1_stack_mac_t* lr1_ptr = lorawan_api_stack_mac_get( CURRENT_STACK );

    const uint32_t ppm_error = lr1_stack_get_crystal_error( lr1_ptr );

    wor_ack_ppm_error_t ppm_relay = WOR_ACK_PPM_ERROR_40PPM;

    if( ppm_error < 10 )
    {
        ppm_relay = WOR_ACK_PPM_ERROR_10PPM;
    }
    else if( ppm_error < 20 )
    {
        ppm_relay = WOR_ACK_PPM_ERROR_20PPM;
    }
    else if( ppm_error < 30 )
    {
        ppm_relay = WOR_ACK_PPM_ERROR_30PPM;
    }

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( relay_init( lr1_ptr, ppm_relay, WOR_ACK_CAD_TO_RX_8SYMB ) == true );
}

void lorawan_relay_rx_fwd_uplink( uint8_t stack_id, const uint8_t* data, uint8_t data_len, uint32_t time_tx,
                                  bool is_join )
{
    IS_VALID_STACK_ID( stack_id );
    IS_SERVICE_INITIALIZED( );

    // SMTC_MODEM_HAL_TRACE_PRINTF( "Relay RX service add fwd ul task %d\n", smtc_modem_hal_get_time_in_ms( ) );
    smodem_task task_relay = {
        .id                = relay_rx_obj.task_id,
        .stack_id          = stack_id,
        .priority          = TASK_HIGH_PRIORITY,
        .time_to_execute_s = smtc_modem_hal_get_time_in_s( ),
    };

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( modem_supervisor_add_task( &task_relay ) == TASK_VALID );

    memcpy( relay_rx_obj.buffer, data, data_len );
    relay_rx_obj.buffer_len    = data_len;
    relay_rx_obj.time_to_tx    = time_tx;
    relay_rx_obj.service_state = ( is_join == true ) ? SERVICE_FWD_FWD_JOIN : SERVICE_FWD_FWD_UL;
    relay_stop( true );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lorawan_relay_rx_service_on_launch( void* context )
{
    // SMTC_MODEM_HAL_TRACE_PRINTF( "Relay RX service Launch %d\n", smtc_modem_hal_get_time_in_ms( ) );
    IS_SERVICE_INITIALIZED( );

    if( ( relay_rx_obj.service_state == SERVICE_FWD_FWD_JOIN ) || ( relay_rx_obj.service_state == SERVICE_FWD_FWD_UL ) )
    {
        lorawan_api_payload_send( FPORT_RELAY, true, relay_rx_obj.buffer, relay_rx_obj.buffer_len, UNCONF_DATA_UP,
                                  relay_rx_obj.time_to_tx, RELAY_STACK_ID );
        // lorawan_api_payload_send_at_time( FPORT_RELAY, true, buffer, buffer_len, UNCONF_DATA_UP, time_to_tx,
        //                                   RELAY_STACK_ID );
    }
    else if( relay_rx_obj.service_state == SERVICE_FWD_EMPTY_UL )
    {
        lorawan_api_payload_send( 0, false, relay_rx_obj.buffer, 0, UNCONF_DATA_UP,
                                  smtc_modem_hal_get_time_in_ms( ) + 300, RELAY_STACK_ID );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Relay RX Launch No action\n" );
    }
}

static void lorawan_relay_rx_service_on_update( void* context )
{
    IS_SERVICE_INITIALIZED( );
    // SMTC_MODEM_HAL_TRACE_PRINTF( "Relay RX service Update %d\n", smtc_modem_hal_get_time_in_ms( ) );

    if( ( relay_rx_obj.service_state == SERVICE_FWD_FWD_JOIN ) || ( relay_rx_obj.service_state == SERVICE_FWD_FWD_UL ) )
    {
        // No DL has been received, so no RXR windows, restart Relay CAD
        // If a DL has been received relay_rx_obj.service_state is SERVICE_FWD_DONE and relay CAD will restarted after
        // DL on RXR
        relay_rx_obj.service_state = SERVICE_FWD_DONE;
        relay_start( );
    }
}

static uint8_t lorawan_relay_rx_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    bool relay_running_flag = relay_rx_get_flag_started( );
    if( relay_rx_obj.relay_running_flag_prev != relay_running_flag )
    {
        relay_rx_obj.relay_running_flag_prev = relay_running_flag;
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_RELAY_RX_RUNNING, relay_running_flag,
                                          rx_down_data->stack_id );
    }

    // SMTC_MODEM_HAL_TRACE_PRINTF( "Relay RX service DL handler\n" );
    if( ( ( relay_rx_obj.service_state == SERVICE_FWD_FWD_JOIN ) ||
          ( relay_rx_obj.service_state == SERVICE_FWD_FWD_UL ) ) &&
        ( rx_down_data->rx_metadata.rx_fport_present == true ) &&
        ( rx_down_data->rx_metadata.rx_fport == FPORT_RELAY ) )
    {
        relay_fwd_dl( rx_down_data->stack_id, rx_down_data->rx_payload, rx_down_data->rx_payload_size );

        if( relay_rx_obj.service_state == SERVICE_FWD_FWD_JOIN )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Join forwarded -> send empty uplink\n" );
            smodem_task task_relay = {
                .id       = relay_rx_obj.task_id,
                .stack_id = rx_down_data->stack_id,
                .priority = TASK_MEDIUM_HIGH_PRIORITY,
                // The EmptyUplink must be send after the RxR window,
                // relay_rx_obj.time_to_tx is timestamped on the rx_uplink+50ms,
                // this downlink service is called after the timestamp
                .time_to_execute_s = ( relay_rx_obj.time_to_tx + ( RXR_WINDOWS_DELAY_S + 1 ) * 1000UL ) / 1000UL,
            };

            if( modem_supervisor_add_task( &task_relay ) == TASK_VALID )
            {
                relay_rx_obj.service_state = SERVICE_FWD_EMPTY_UL;
            }
            else
            {
                relay_rx_obj.service_state = SERVICE_FWD_DONE;
            }
        }
        else
        {
            relay_rx_obj.service_state = SERVICE_FWD_DONE;
        }

        return MODEM_DOWNLINK_CONSUMED;
    }

    return MODEM_DOWNLINK_UNCONSUMED;
}

/* --- EOF ------------------------------------------------------------------ */
