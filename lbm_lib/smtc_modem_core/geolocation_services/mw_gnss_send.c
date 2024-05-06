/**
 * @file      mw_gnss_send.c
 *
 * @brief     GNSS Middleware
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

#include "mw_gnss_send.h"

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
#define NUMBER_MAX_OF_MW_GNSS_SEND_OBJ 1

/**
 * @brief Check is the index is valid before accessing the object
 */
#define IS_VALID_OBJECT_ID( x )                                                \
    do                                                                         \
    {                                                                          \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_MAX_OF_MW_GNSS_SEND_OBJ ); \
    } while( 0 )

/**
 * @brief Check is the index is valid before accessing the object
 */
#define IS_VALID_STACK_ID( x )                                   \
    do                                                           \
    {                                                            \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_OF_STACKS ); \
    } while( 0 )

#define IS_SERVICE_INITIALIZED( )                                               \
    do                                                                          \
    {                                                                           \
        if( mw_gnss_send_obj.initialized == false )                             \
        {                                                                       \
            SMTC_MODEM_HAL_TRACE_WARNING( "gnss send service not launched\n" ); \
            break;                                                              \
        }                                                                       \
    } while( 0 )

/**
 * @brief Deep debug traces (ON/OFF)
 */
#ifndef GNSS_SEND_DEEP_DBG_TRACE
#define GNSS_SEND_DEEP_DBG_TRACE MODEM_HAL_FEATURE_OFF
#endif
#if( GNSS_SEND_DEEP_DBG_TRACE )
#define GNSS_SEND_TRACE_PRINTF_DEBUG( ... ) SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ )
#define GNSS_SEND_TRACE_ARRAY_DEBUG( ... ) SMTC_MODEM_HAL_TRACE_ARRAY( __VA_ARGS__ )
#else
#define GNSS_SEND_TRACE_PRINTF_DEBUG( ... )
#define GNSS_SEND_TRACE_ARRAY_DEBUG( ... )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#ifndef MODEM_MIN_RANDOM_DELAY_MS
#define MODEM_MIN_RANDOM_DELAY_MS 200
#endif

#ifndef MODEM_MAX_RANDOM_DELAY_MS
#define MODEM_MAX_RANDOM_DELAY_MS 3000
#endif

/**
 * @brief LoRaWAN port used for uplinks of the GNSS scan results
 */
#define GNSS_DEFAULT_UPLINK_PORT ( 192 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Description of the GNSS send service context
 */
typedef struct mw_gnss_send_s
{
    uint8_t stack_id;
    uint8_t task_id;
    bool    initialized;
    uint8_t fport;  //!< LoRaWAN port on which to send NAV packets
    bool    is_busy;
    bool    pending_evt_terminated;
    /* NAV related fields */
    const navgroup_t*                  nav_group;      //!< GNSS NAV Group to be sent over the air
    uint8_t                            nb_scans_sent;  //!< Number of scans (NAV message) already sent
    smtc_modem_geolocation_send_mode_t send_mode;      //!< Send mode to be used: uplink, store&fwd, bypass
} mw_gnss_send_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/**
 * @brief Object to hold the current service context
 */
static mw_gnss_send_t mw_gnss_send_obj = { 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Service callbacks
 */
static void    mw_gnss_send_service_on_launch( void* context_callback );
static void    mw_gnss_send_service_on_update( void* context_callback );
static uint8_t mw_gnss_send_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/**
 * @brief Program the supervisor task for sending the next GNSS scan result.
 */
static void mw_gnss_send_next( void );

/**
 * @brief Send an event to the user application to notify for sequence progress.
 */
static void send_event( smtc_modem_event_type_t event );

/**
 * @brief Print event data to console
 */
static void trace_print_event_data_terminated( const smtc_modem_gnss_event_data_terminated_t* data );

/**
 * @brief Prepare the buffer for data to be sent
 */
static void prepare_tx_buffer( uint8_t** tx_buffer, uint8_t* tx_buffer_size, uint8_t* port );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void mw_gnss_send_services_init( uint8_t* service_id, uint8_t task_id,
                                 uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                 void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                                 void** context_callback )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "mw_gnss_send_services_init task_id %d, service_id %d, CURRENT_STACK:%d \n", task_id,
                                 *service_id, CURRENT_STACK );

    IS_VALID_OBJECT_ID( *service_id );
    mw_gnss_send_obj.task_id     = task_id;
    mw_gnss_send_obj.stack_id    = CURRENT_STACK;
    mw_gnss_send_obj.initialized = true;
    *downlink_callback           = mw_gnss_send_service_downlink_handler;
    *on_launch_callback          = mw_gnss_send_service_on_launch;
    *on_update_callback          = mw_gnss_send_service_on_update;
    *context_callback            = ( void* ) modem_supervisor_get_task( );

    mw_gnss_send_obj.nav_group     = NULL;
    mw_gnss_send_obj.nb_scans_sent = 0;
    mw_gnss_send_obj.fport         = GNSS_DEFAULT_UPLINK_PORT;
    mw_gnss_send_obj.is_busy       = false;
    mw_gnss_send_obj.send_mode     = SMTC_MODEM_SEND_MODE_UPLINK;
}

void mw_gnss_send_add_task( const navgroup_t* nav_group )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "mw_gnss_send_add_task: add task in supervisor\n" );

    IS_SERVICE_INITIALIZED( );

    /* Clear pending events */
    mw_gnss_send_obj.pending_evt_terminated = false;

    /* Set NAV group send context */
    mw_gnss_send_obj.nav_group     = nav_group;
    mw_gnss_send_obj.nb_scans_sent = 0;

    if( ( mw_gnss_send_obj.send_mode == SMTC_MODEM_SEND_MODE_BYPASS ) || ( nav_group->nb_scans_valid == 0 ) )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "mw_gnss_send_add_task: no scan to be sent\n" );
        send_event( SMTC_MODEM_EVENT_GNSS_TERMINATED );
    }
    else
    {
        /* Prepare the task */
        mw_gnss_send_next( );
    }
}

bool mw_gnss_send_is_busy( )
{
    return mw_gnss_send_obj.is_busy;
}

smtc_modem_return_code_t mw_gnss_get_event_data_terminated( smtc_modem_gnss_event_data_terminated_t* data )
{
    if( data == NULL )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "mw_gnss_get_event_data_terminated: Provided pointer is NULL\n" );
        return SMTC_MODEM_RC_INVALID;
    }

    if( mw_gnss_send_obj.pending_evt_terminated == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "mw_gnss_get_event_data_terminated: no TERMINATED event pending\n" );
        return SMTC_MODEM_RC_FAIL;
    }

    memset( data, 0, sizeof( smtc_modem_gnss_event_data_terminated_t ) );

    data->nb_scans_sent = mw_gnss_send_obj.nb_scans_sent;

    trace_print_event_data_terminated( data );

    return SMTC_MODEM_RC_OK;
}

void mw_gnss_set_port( uint8_t port )
{
    mw_gnss_send_obj.fport = port;
}

smtc_modem_return_code_t mw_gnss_set_send_mode( smtc_modem_geolocation_send_mode_t send_mode )
{
    mw_gnss_send_obj.send_mode = send_mode;

#if !defined( ADD_SMTC_STORE_AND_FORWARD )
    if( send_mode == SMTC_MODEM_SEND_MODE_STORE_AND_FORWARD )
    {
        return SMTC_MODEM_RC_INVALID;
    }
#endif
    return SMTC_MODEM_RC_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void mw_gnss_send_service_on_launch( void* context_callback )
{
    uint8_t          port;
    uint8_t*         tx_buffer;
    uint8_t          tx_buffer_size;
    status_lorawan_t send_status  = ERRORLORAWAN;
    stask_manager*   task_manager = ( stask_manager* ) context_callback;

    GNSS_SEND_TRACE_PRINTF_DEBUG( "mw_gnss_send_service_on_launch\n" );

    IS_SERVICE_INITIALIZED( );

    mw_gnss_send_obj.is_busy = true;

    prepare_tx_buffer( &tx_buffer, &tx_buffer_size, &port );

    if( mw_gnss_send_obj.send_mode == SMTC_MODEM_SEND_MODE_UPLINK )
    {
        send_status = tx_protocol_manager_request (TX_PROTOCOL_TRANSMIT_LORA, port, true, tx_buffer, tx_buffer_size, UNCONF_DATA_UP,
                                                smtc_modem_hal_get_time_in_ms( ) ,
                                                mw_gnss_send_obj.stack_id );

        /* Set result in task_context for later check */
        if( send_status == OKLORAWAN )
        {
            task_manager->modem_task[mw_gnss_send_obj.task_id].task_context = true;
            SMTC_MODEM_HAL_TRACE_INFO( "GNSS Tx [%d] on FPort %d\n", mw_gnss_send_obj.nb_scans_sent, port );
            SMTC_MODEM_HAL_TRACE_ARRAY( "GNSS Tx", tx_buffer, tx_buffer_size );
        }
        else
        {
            task_manager->modem_task[mw_gnss_send_obj.task_id].task_context = false;
            SMTC_MODEM_HAL_TRACE_ERROR( "GNSS Tx [%d]: tx_protocol_manager_request (TX_PROTOCOL_TRANSMIT_LORA,) failed.\n",
                                        mw_gnss_send_obj.nb_scans_sent );
        }
    }
    else
    {
#if defined( ADD_SMTC_STORE_AND_FORWARD )
        store_and_forward_flash_rc_t store_status;
        store_status =
            store_and_forward_flash_add_data( mw_gnss_send_obj.stack_id, port, false, tx_buffer, tx_buffer_size );

        if( store_status == STORE_AND_FORWARD_FLASH_RC_OK )
        {
            task_manager->modem_task[mw_gnss_send_obj.task_id].task_context = true;
            SMTC_MODEM_HAL_TRACE_INFO( "GNSS Payload pushed to store&fwd on FPort %d\n", port );
            SMTC_MODEM_HAL_TRACE_ARRAY( "GNSS Payload", tx_buffer, tx_buffer_size );
        }
        else
        {
            task_manager->modem_task[mw_gnss_send_obj.task_id].task_context = false;
            SMTC_MODEM_HAL_TRACE_ERROR( "GNSS: failed to push payload to store&fwd - internal code: %x\n",
                                        store_status );
        }
#endif
    }
}

static void mw_gnss_send_service_on_update( void* context_callback )
{
    stask_manager* task_manager = ( stask_manager* ) context_callback;

    GNSS_SEND_TRACE_PRINTF_DEBUG( "mw_gnss_send_service_on_update\n" );

    IS_SERVICE_INITIALIZED( );

    /* Update context if the call to the lorawan stack send succeeded */
    if( task_manager->modem_task[mw_gnss_send_obj.task_id].task_context == true )
    {
        mw_gnss_send_obj.nb_scans_sent += 1;
    }

    if( ( task_manager->modem_task[mw_gnss_send_obj.task_id].task_context == false ) ||
        ( mw_gnss_send_obj.nb_scans_sent == mw_gnss_send_obj.nav_group->nb_scans_valid ) )
    {
        /* Stop send sequence if an error occurred or if everything has been sent */
        send_event( SMTC_MODEM_EVENT_GNSS_TERMINATED );
    }
    else
    {
        mw_gnss_send_next( );
    }

    mw_gnss_send_obj.is_busy = false;
}

static uint8_t mw_gnss_send_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    GNSS_SEND_TRACE_PRINTF_DEBUG( "mw_gnss_send_service_downlink_handler\n" );

    return MODEM_DOWNLINK_UNCONSUMED;
}

static void mw_gnss_send_next( void )
{
    GNSS_SEND_TRACE_PRINTF_DEBUG( "mw_gnss_send_next\n" );

    smodem_task task = { 0 };
    task.id          = mw_gnss_send_obj.task_id;
    task.stack_id    = mw_gnss_send_obj.stack_id;
    if( mw_gnss_send_obj.send_mode == SMTC_MODEM_SEND_MODE_STORE_AND_FORWARD )
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

static void send_event( smtc_modem_event_type_t event )
{
    if( event == SMTC_MODEM_EVENT_GNSS_TERMINATED )
    {
        mw_gnss_send_obj.pending_evt_terminated = true;
    }

    increment_asynchronous_msgnumber( event, 0, mw_gnss_send_obj.stack_id );
}

static void trace_print_event_data_terminated( const smtc_modem_gnss_event_data_terminated_t* data )
{
    if( data != NULL )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "TERMINATED info:\n" );
        SMTC_MODEM_HAL_TRACE_PRINTF( "-- number of scans sent: %u\n", data->nb_scans_sent );
    }
}

static void prepare_tx_buffer( uint8_t** tx_buffer, uint8_t* tx_buffer_size, uint8_t* port )
{
    uint8_t src_idx = mw_gnss_send_obj.nb_scans_sent;

    /* Prepare buffer for TX */
    *tx_buffer      = ( uint8_t* ) mw_gnss_send_obj.nav_group->scans[src_idx].results_buffer;
    *tx_buffer_size = mw_gnss_send_obj.nav_group->scans[src_idx].results_size;
    *port           = mw_gnss_send_obj.fport;
}

/* --- EOF ------------------------------------------------------------------ */
