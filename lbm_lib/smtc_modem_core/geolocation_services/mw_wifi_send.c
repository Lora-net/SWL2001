/**
 * @file      mw_wifi_send.c
 *
 * @brief     Wi-Fi Middleware
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

#include "mw_wifi_send.h"

#include "modem_core.h"
#include "modem_supervisor_light.h"
#include "modem_event_utilities.h"
#include "lorawan_send_management.h"
#include "smtc_modem_api.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "lorawan_api.h"
#include "radio_planner.h"
#include "radio_planner_hook_id_defs.h"
#if defined( ADD_SMTC_STORE_AND_FORWARD )
#include "store_and_forward.h"
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )
#define NUMBER_MAX_OF_MW_WIFI_SEND_OBJ 1

/**
 * @brief Check is the index is valid before accessing the object
 */
#define IS_VALID_OBJECT_ID( x )                                                \
    do                                                                         \
    {                                                                          \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_MAX_OF_MW_WIFI_SEND_OBJ ); \
    } while( 0 )

/**
 * @brief Check is the index is valid before accessing the object
 */
#define IS_VALID_STACK_ID( x )                                   \
    do                                                           \
    {                                                            \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_OF_STACKS ); \
    } while( 0 )

#define IS_SERVICE_INITIALIZED( )                                                \
    do                                                                           \
    {                                                                            \
        if( mw_wifi_send_obj.initialized == false )                              \
        {                                                                        \
            SMTC_MODEM_HAL_TRACE_WARNING( "Wi-Fi send service not launched\n" ); \
            break;                                                               \
        }                                                                        \
    } while( 0 )

/**
 * @brief Deep debug traces (ON/OFF)
 */
#ifndef WIFI_SEND_DEEP_DBG_TRACE
#define WIFI_SEND_DEEP_DBG_TRACE MODEM_HAL_FEATURE_OFF
#endif
#if( WIFI_SEND_DEEP_DBG_TRACE )
#define WIFI_SEND_TRACE_PRINTF_DEBUG( ... ) SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ )
#define WIFI_SEND_TRACE_ARRAY_DEBUG( ... ) SMTC_MODEM_HAL_TRACE_ARRAY( __VA_ARGS__ )
#else
#define WIFI_SEND_TRACE_PRINTF_DEBUG( ... )
#define WIFI_SEND_TRACE_ARRAY_DEBUG( ... )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */


/**
 * @brief Minimal number of detected access point in a scan result to consider the scan valid
 */
#define WIFI_SCAN_NB_AP_MIN ( 3 )

/**
 * @brief Size in bytes to store the RSSI of a detected WiFi Access-Point
 */
#define WIFI_AP_RSSI_SIZE ( 1 )

/**
 * @brief Size in bytes of the payload tag to indicate frame format (as defined by LR1110 WiFi positioning protocol of
 * LoRaCloud)
 */
#define WIFI_TAG_SIZE ( 1 )

/**
 * @brief Default LoRaWAN port to send uplink with Wi-Fi scan results
 */
#define WIFI_DEFAULT_UPLINK_PORT ( 197 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Description of the Wi-Fi send service context
 */
typedef struct mw_wifi_send_s
{
    uint8_t stack_id;
    uint8_t task_id;
    bool    initialized;
    bool    is_busy;
    uint8_t fport;  //!< LoRaWAN port on which to send NAV packets
    bool    pending_evt_terminated;
    /* NAV related fields */
    const wifi_scan_all_result_t*      wifi_results;    //!< WiFi results to be sent over the air
    uint8_t                            nb_scans_sent;   //!< Number of scans already sent
    smtc_modem_geolocation_send_mode_t send_mode;       //!< Send mode to be used: uplink, store&fwd, bypass
    smtc_modem_wifi_payload_format_t   payload_format;  //!< Selected payload format (MAC or MAC+RSSI)
} mw_wifi_send_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/**
 * @brief Object to hold the current service context
 */
static mw_wifi_send_t mw_wifi_send_obj = { 0 };

/*!
 * @brief The buffer containing results to be sent over the air
 */
static uint8_t wifi_result_buffer[WIFI_TAG_SIZE + ( ( WIFI_AP_RSSI_SIZE + WIFI_AP_ADDRESS_SIZE ) * WIFI_MAX_RESULTS )];

/*!
 * @brief Current result buffer size
 */
static uint8_t wifi_result_buffer_size = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Service callbacks
 */
static void    mw_wifi_send_service_on_launch( void* context_callback );
static void    mw_wifi_send_service_on_update( void* context_callback );
static uint8_t mw_wifi_send_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/**
 * @brief Send an event to the user application to notify for sequence progress.
 */
static void send_event( smtc_modem_event_type_t event );

/**
 * @brief Print event data to console
 */
static void trace_print_event_data_terminated( const smtc_modem_wifi_event_data_terminated_t* data );

/**
 * @brief Prepare the buffer for data to be sent
 */
static void prepare_tx_buffer( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void mw_wifi_send_services_init( uint8_t* service_id, uint8_t task_id,
                                 uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                 void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                                 void** context_callback )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "mw_wifi_send_services_init task_id %d, service_id %d, CURRENT_STACK:%d \n", task_id,
                                 *service_id, CURRENT_STACK );

    IS_VALID_OBJECT_ID( *service_id );
    mw_wifi_send_obj.task_id     = task_id;
    mw_wifi_send_obj.stack_id    = CURRENT_STACK;
    mw_wifi_send_obj.initialized = true;
    *downlink_callback           = mw_wifi_send_service_downlink_handler;
    *on_launch_callback          = mw_wifi_send_service_on_launch;
    *on_update_callback          = mw_wifi_send_service_on_update;
    *context_callback            = ( void* ) modem_supervisor_get_task( );

    mw_wifi_send_obj.wifi_results   = NULL;
    mw_wifi_send_obj.nb_scans_sent  = 0;
    mw_wifi_send_obj.fport          = WIFI_DEFAULT_UPLINK_PORT;
    mw_wifi_send_obj.is_busy        = false;
    mw_wifi_send_obj.send_mode      = SMTC_MODEM_SEND_MODE_UPLINK;
    mw_wifi_send_obj.payload_format = SMTC_MODEM_WIFI_PAYLOAD_MAC;
}

void mw_wifi_send_add_task( const wifi_scan_all_result_t* wifi_results )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "mw_wifi_send_add_task: add task in supervisor\n" );

    IS_SERVICE_INITIALIZED( );

    /* Clear pending events */
    mw_wifi_send_obj.pending_evt_terminated = false;

    /* Set send context */
    mw_wifi_send_obj.wifi_results  = wifi_results;
    mw_wifi_send_obj.nb_scans_sent = 0;

    if( ( mw_wifi_send_obj.send_mode == SMTC_MODEM_SEND_MODE_BYPASS ) ||
        ( mw_wifi_send_obj.wifi_results->nbr_results < WIFI_SCAN_NB_AP_MIN ) )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "mw_wifi_send_add_task: no scan to be sent\n" );
        send_event( SMTC_MODEM_EVENT_WIFI_TERMINATED );
    }
    else
    {
        /* Prepare the task */
        smodem_task task = { 0 };
        task.id          = mw_wifi_send_obj.task_id;
        task.stack_id    = mw_wifi_send_obj.stack_id;
        if( mw_wifi_send_obj.send_mode == SMTC_MODEM_SEND_MODE_STORE_AND_FORWARD )
        {
            /* we just push data to the store&fwd service, no TX from this service. */
            task.priority = TASK_BYPASS_DUTY_CYCLE;
        }
        else
        {
            task.priority = TASK_HIGH_PRIORITY;
        }
        task.time_to_execute_s = smtc_modem_hal_get_time_in_s( );
        modem_supervisor_add_task( &task );
    }
}

bool mw_wifi_send_is_busy( )
{
    return mw_wifi_send_obj.is_busy;
}

smtc_modem_return_code_t mw_wifi_get_event_data_terminated( smtc_modem_wifi_event_data_terminated_t* data )
{
    if( data == NULL )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "mw_wifi_get_event_data_terminated: Provided pointer is NULL\n" );
        return SMTC_MODEM_RC_INVALID;
    }

    if( mw_wifi_send_obj.pending_evt_terminated == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "mw_wifi_get_event_data_terminated: no TERMINATED event pending\n" );
        return SMTC_MODEM_RC_FAIL;
    }

    memset( data, 0, sizeof( smtc_modem_wifi_event_data_terminated_t ) );

    data->nb_scans_sent = mw_wifi_send_obj.nb_scans_sent;

    trace_print_event_data_terminated( data );

    return SMTC_MODEM_RC_OK;
}

void mw_wifi_set_port( uint8_t port )
{
    mw_wifi_send_obj.fport = port;
}

smtc_modem_return_code_t mw_wifi_set_send_mode( smtc_modem_geolocation_send_mode_t send_mode )
{
    mw_wifi_send_obj.send_mode = send_mode;

#if !defined( ADD_SMTC_STORE_AND_FORWARD )
    if( send_mode == SMTC_MODEM_SEND_MODE_STORE_AND_FORWARD )
    {
        return SMTC_MODEM_RC_INVALID;
    }
#endif
    return SMTC_MODEM_RC_OK;
}

void mw_wifi_set_payload_format( smtc_modem_wifi_payload_format_t format )
{
    mw_wifi_send_obj.payload_format = format;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void mw_wifi_send_service_on_launch( void* context_callback )
{
    status_lorawan_t send_status  = ERRORLORAWAN;
    stask_manager*   task_manager = ( stask_manager* ) context_callback;

    WIFI_SEND_TRACE_PRINTF_DEBUG( "mw_wifi_send_service_on_launch\n" );

    IS_SERVICE_INITIALIZED( );

    mw_wifi_send_obj.is_busy = true;

    prepare_tx_buffer( );

    if( mw_wifi_send_obj.send_mode == SMTC_MODEM_SEND_MODE_UPLINK )
    {
        send_status = tx_protocol_manager_request (TX_PROTOCOL_TRANSMIT_LORA,
            mw_wifi_send_obj.fport, true, wifi_result_buffer, wifi_result_buffer_size, UNCONF_DATA_UP,
            smtc_modem_hal_get_time_in_ms( )  , mw_wifi_send_obj.stack_id );

        if( send_status == OKLORAWAN )
        {
            task_manager->modem_task[mw_wifi_send_obj.task_id].task_context = true;
            SMTC_MODEM_HAL_TRACE_INFO( "Wi-Fi Tx on FPort %d\n", mw_wifi_send_obj.fport );
            SMTC_MODEM_HAL_TRACE_ARRAY( "Wi-Fi Tx", wifi_result_buffer, wifi_result_buffer_size );
        }
        else
        {
            task_manager->modem_task[mw_wifi_send_obj.task_id].task_context = false;
            SMTC_MODEM_HAL_TRACE_ERROR( "Wi-Fi Tx: tx_protocol_manager_request (TX_PROTOCOL_TRANSMIT_LORA,) failed.\n" );
        }
    }
    else
    {
#if defined( ADD_SMTC_STORE_AND_FORWARD )
        store_and_forward_flash_rc_t store_status;
        store_status = store_and_forward_flash_add_data( mw_wifi_send_obj.stack_id, mw_wifi_send_obj.fport, false,
                                                         wifi_result_buffer, wifi_result_buffer_size );

        if( store_status == STORE_AND_FORWARD_FLASH_RC_OK )
        {
            task_manager->modem_task[mw_wifi_send_obj.task_id].task_context = true;
            SMTC_MODEM_HAL_TRACE_INFO( "Wi-Fi Payload pushed to store&fwd on FPort %d\n", mw_wifi_send_obj.fport );
            SMTC_MODEM_HAL_TRACE_ARRAY( "Wi-Fi Payload", wifi_result_buffer, wifi_result_buffer_size );
        }
        else
        {
            task_manager->modem_task[mw_wifi_send_obj.task_id].task_context = false;
            SMTC_MODEM_HAL_TRACE_ERROR( "Wi-Fi: failed to push payload to store&fwd - internal code: %x\n",
                                        store_status );
        }
#endif
    }
}

static void mw_wifi_send_service_on_update( void* context_callback )
{
    stask_manager* task_manager = ( stask_manager* ) context_callback;

    WIFI_SEND_TRACE_PRINTF_DEBUG( "mw_wifi_send_service_on_update\n" );

    IS_SERVICE_INITIALIZED( );

    /* Update context if the call to the lorawan stack send succeeded */
    if( task_manager->modem_task[mw_wifi_send_obj.task_id].task_context == true )
    {
        mw_wifi_send_obj.nb_scans_sent += 1;
    }

    send_event( SMTC_MODEM_EVENT_WIFI_TERMINATED );

    mw_wifi_send_obj.is_busy = false;
}

static uint8_t mw_wifi_send_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    WIFI_SEND_TRACE_PRINTF_DEBUG( "mw_wifi_send_service_downlink_handler\n" );

    return MODEM_DOWNLINK_UNCONSUMED;
}

static void send_event( smtc_modem_event_type_t event )
{
    if( event == SMTC_MODEM_EVENT_WIFI_TERMINATED )
    {
        mw_wifi_send_obj.pending_evt_terminated = true;
    }
    increment_asynchronous_msgnumber( event, 0, mw_wifi_send_obj.stack_id );
}

static void trace_print_event_data_terminated( const smtc_modem_wifi_event_data_terminated_t* data )
{
    if( data != NULL )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "TERMINATED info:\n" );
        SMTC_MODEM_HAL_TRACE_PRINTF( "-- number of scans sent: %u\n", data->nb_scans_sent );
    }
}

static void prepare_tx_buffer( void )
{
    /* Add the payload format tag */
    wifi_result_buffer_size                     = 0;
    wifi_result_buffer[wifi_result_buffer_size] = mw_wifi_send_obj.payload_format;
    wifi_result_buffer_size += WIFI_TAG_SIZE;

    /* Concatenate all results in send buffer */
    for( uint8_t i = 0; i < mw_wifi_send_obj.wifi_results->nbr_results; i++ )
    {
        /* Copy Access Point measured RSSI in result buffer (if requested) */
        if( mw_wifi_send_obj.payload_format == SMTC_MODEM_WIFI_PAYLOAD_MAC_RSSI )
        {
            wifi_result_buffer[wifi_result_buffer_size] = mw_wifi_send_obj.wifi_results->results[i].rssi;
            wifi_result_buffer_size += WIFI_AP_RSSI_SIZE;
        }
        /* Copy Access Point MAC address in result buffer */
        memcpy( &wifi_result_buffer[wifi_result_buffer_size], mw_wifi_send_obj.wifi_results->results[i].mac_address,
                WIFI_AP_ADDRESS_SIZE );
        wifi_result_buffer_size += WIFI_AP_ADDRESS_SIZE;
    }
}

/* --- EOF ------------------------------------------------------------------ */
