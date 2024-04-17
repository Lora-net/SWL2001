/**
 * @file      mw_wifi_scan.c
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
#include <math.h>

#include "mw_wifi_scan.h"
#include "wifi_helpers.h"
#include "wifi_helpers_defs.h"
#include "geolocation_bsp.h"
#include "mw_common.h"

#include "modem_core.h"
#include "modem_supervisor_light.h"
#include "modem_event_utilities.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_modem_geolocation_api.h"
#include "radio_planner.h"
#include "radio_planner_hook_id_defs.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )
#define NUMBER_MAX_OF_MW_WIFI_TASK_OBJ 1

/**
 * @brief Check is the index is valid before accessing the object
 */
#define IS_VALID_OBJECT_ID( x )                                                \
    do                                                                         \
    {                                                                          \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_MAX_OF_MW_WIFI_TASK_OBJ ); \
    } while( 0 )

/**
 * @brief Check is the index is valid before accessing the object
 */
#define IS_VALID_STACK_ID( x )                                   \
    do                                                           \
    {                                                            \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_OF_STACKS ); \
    } while( 0 )

#define IS_SERVICE_INITIALIZED( )                                           \
    do                                                                      \
    {                                                                       \
        if( mw_wifi_task_obj.initialized == false )                         \
        {                                                                   \
            SMTC_MODEM_HAL_TRACE_WARNING( "Wi-Fi service not launched\n" ); \
            break;                                                          \
        }                                                                   \
    } while( 0 )

/**
 * @brief Deep debug traces (ON/OFF)
 */
#ifndef WIFI_SCAN_DEEP_DBG_TRACE
#define WIFI_SCAN_DEEP_DBG_TRACE MODEM_HAL_FEATURE_OFF
#endif
#if( WIFI_SCAN_DEEP_DBG_TRACE )
#define WIFI_SCAN_TRACE_PRINTF_DEBUG( ... ) SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ )
#define WIFI_SCAN_TRACE_ARRAY_DEBUG( ... ) SMTC_MODEM_HAL_TRACE_ARRAY( __VA_ARGS__ )
#else
#define WIFI_SCAN_TRACE_PRINTF_DEBUG( ... )
#define WIFI_SCAN_TRACE_ARRAY_DEBUG( ... )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Description of the Wi-Fi scan service context
 */
typedef struct mw_wifi_task_s
{
    uint8_t  stack_id;
    uint8_t  task_id;
    uint8_t  rp_hook_id;
    bool     initialized;
    bool     scan_sequence_started;
    uint32_t scan_start_time;
    uint32_t scan_end_time;
    bool     pending_evt_scan_done;
} mw_wifi_task_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/**
 * @brief Object to hold the current service context
 */
static mw_wifi_task_t mw_wifi_task_obj = { 0 };

/*!
 * @brief Results of the current Wi-Fi scan
 */
static wifi_scan_all_result_t wifi_results;

/*!
 * @brief WiFi scan settings
 */
wifi_settings_t wifi_settings = { .channels            = 0x3FFF, /* all channels enabled */
                                  .types               = LR11XX_WIFI_TYPE_SCAN_B_G_N,
                                  .max_results         = WIFI_MAX_RESULTS,
                                  .timeout_per_channel = WIFI_TIMEOUT_PER_CHANNEL_DEFAULT,
                                  .timeout_per_scan    = WIFI_TIMEOUT_PER_SCAN_DEFAULT };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Service callbacks
 */
static void    mw_wifi_scan_service_on_launch( void* context_callback );
static void    mw_wifi_scan_service_on_update( void* context_callback );
static uint8_t mw_wifi_scan_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/**
 * @brief Callback called by the radio planner when radio access is granted to the service
 */
static void wifi_rp_task_launch( void* context );

/**
 * @brief Callback called by the radio planner when task has been completed (scan done, aborted...)
 */
static void wifi_rp_task_done( void* status );

/**
 * @brief Process on task completion
 */
static void wifi_scan_task_done( void );

/**
 * @brief Send an event to user to notify for progress
 */
static void send_event( smtc_modem_event_type_t event );

/**
 * @brief Helper function to print event data to console
 */
static void trace_print_event_data_scan_done( const smtc_modem_wifi_event_data_scan_done_t* data );

/**
 * @brief Helper function to sort scan results by descending power order
 */
static void smtc_wifi_sort_results( wifi_scan_all_result_t* wifi_results );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void mw_wifi_scan_services_init( uint8_t* service_id, uint8_t task_id,
                                 uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                 void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                                 void** context_callback )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "mw_wifi_scan_services_init task_id %d, service_id %d, CURRENT_STACK:%d \n", task_id,
                                 *service_id, CURRENT_STACK );

    IS_VALID_OBJECT_ID( *service_id );

    /* Service context */
    mw_wifi_task_obj.task_id     = task_id;
    mw_wifi_task_obj.stack_id    = CURRENT_STACK;
    mw_wifi_task_obj.rp_hook_id  = RP_HOOK_ID_DIRECT_RP_ACCESS_WIFI;
    mw_wifi_task_obj.initialized = true;
    *downlink_callback           = mw_wifi_scan_service_downlink_handler;
    *on_launch_callback          = mw_wifi_scan_service_on_launch;
    *on_update_callback          = mw_wifi_scan_service_on_update;
    *context_callback            = ( void* ) service_id;
    rp_hook_init( modem_get_rp( ), mw_wifi_task_obj.rp_hook_id, ( void ( * )( void* ) )( wifi_rp_task_done ),
                  modem_get_rp( ) );
}

smtc_modem_return_code_t mw_wifi_scan_add_task( uint32_t start_delay_s )
{
    uint32_t now = smtc_modem_hal_get_time_in_s( );
    SMTC_MODEM_HAL_TRACE_PRINTF( "mw_wifi_scan_add_task: add task in supervisor at %u + %u s\n", now, start_delay_s );

    IS_SERVICE_INITIALIZED( );
    smodem_task task       = { 0 };
    task.id                = mw_wifi_task_obj.task_id;
    task.stack_id          = mw_wifi_task_obj.stack_id;
    task.priority          = TASK_BYPASS_DUTY_CYCLE; /* No TX for this service */
    task.time_to_execute_s = now + start_delay_s;
    if( modem_supervisor_add_task( &task ) != TASK_VALID )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to add Wi-Fi scan supervisor task\n" );
        return SMTC_MODEM_RC_FAIL;
    }

    /* Clear pending events */
    mw_wifi_task_obj.pending_evt_scan_done = false;

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t mw_wifi_scan_remove_task( void )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "mw_wifi_scan_remove_task\n" );

    IS_SERVICE_INITIALIZED( );

    if( mw_wifi_task_obj.scan_sequence_started == true )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "WIFI scan sequence started, too late to cancel\n" );
        return SMTC_MODEM_RC_BUSY;
    }

    if( modem_supervisor_remove_task( mw_wifi_task_obj.task_id ) != TASK_VALID )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to remove Wi-Fi scan supervisor task\n" );
        return SMTC_MODEM_RC_FAIL;
    }

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t mw_wifi_get_event_data_scan_done( smtc_modem_wifi_event_data_scan_done_t* data )
{
    if( data == NULL )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "mw_wifi_get_event_data_scan_done: Provided pointer is NULL\n" );
        return SMTC_MODEM_RC_INVALID;
    }

    if( mw_wifi_task_obj.pending_evt_scan_done == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "mw_wifi_get_event_data_scan_done: no SCAN_DONE event pending\n" );
        return SMTC_MODEM_RC_FAIL;
    }

    memcpy( data, &wifi_results, sizeof( smtc_modem_wifi_event_data_scan_done_t ) );

    trace_print_event_data_scan_done( data );

    return SMTC_MODEM_RC_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void mw_wifi_scan_service_on_launch( void* context_callback )
{
    WIFI_SCAN_TRACE_PRINTF_DEBUG( "mw_wifi_scan_service_on_launch\n" );

    IS_SERVICE_INITIALIZED( );

    /* From now, the scan sequence can not be cancelled */
    mw_wifi_task_obj.scan_sequence_started = true;

    mw_wifi_task_obj.scan_start_time = 0;
    mw_wifi_task_obj.scan_end_time   = 0;

    rp_task_t rp_task                      = { 0 };
    rp_task.hook_id                        = mw_wifi_task_obj.rp_hook_id;
    rp_task.state                          = RP_TASK_STATE_ASAP;
    rp_task.start_time_ms                  = smtc_modem_hal_get_time_in_ms( ) + 300; /* 300ms for scheduling delay */
    rp_task.duration_time_ms               = 10 * 1000; /* Provision 10 seconds for WiFi scan duration */
    rp_task.type                           = RP_TASK_TYPE_WIFI_SNIFF;
    rp_task.launch_task_callbacks          = wifi_rp_task_launch;
    rp_radio_params_t fake_rp_radio_params = { 0 };
    if( rp_task_enqueue( modem_get_rp( ), &rp_task, NULL, 0, &fake_rp_radio_params ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to enqueue RP task for Wi-Fi scan\n" );
        SMTC_MODEM_HAL_PANIC( );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Enqueued RP task for Wi-Fi scan (hook_id #%d)\n", rp_task.hook_id );
    }
}

static void mw_wifi_scan_service_on_update( void* context_callback )
{
    WIFI_SCAN_TRACE_PRINTF_DEBUG( "mw_wifi_scan_service_on_update\n" );

    IS_SERVICE_INITIALIZED( );
}

static uint8_t mw_wifi_scan_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    WIFI_SCAN_TRACE_PRINTF_DEBUG( "mw_wifi_scan_service_downlink_handler\n" );

    return MODEM_DOWNLINK_UNCONSUMED;
}

static void trace_print_scan_results( const wifi_scan_all_result_t* results )
{
    if( results != NULL )
    {
        for( uint8_t i = 0; i < results->nbr_results; i++ )
        {
            for( uint8_t j = 0; j < WIFI_AP_ADDRESS_SIZE; j++ )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "%02X ", results->results[i].mac_address[j] );
            }
            SMTC_MODEM_HAL_TRACE_PRINTF( " -- Channel: %d", results->results[i].channel );
            SMTC_MODEM_HAL_TRACE_PRINTF( " -- Type: %d", results->results[i].type );
            SMTC_MODEM_HAL_TRACE_PRINTF( " -- RSSI: %d", results->results[i].rssi );
            SMTC_MODEM_HAL_TRACE_PRINTF( " -- Origin: " );
            switch( results->results[i].origin )
            {
            case LR11XX_WIFI_ORIGIN_BEACON_FIX_AP:
                SMTC_MODEM_HAL_TRACE_PRINTF( "FIXED\n" );
                break;
            case LR11XX_WIFI_ORIGIN_BEACON_MOBILE_AP:
                SMTC_MODEM_HAL_TRACE_PRINTF( "MOBILE\n" );
                break;
            default:
                SMTC_MODEM_HAL_TRACE_PRINTF( "UNKNOWN\n" );
                break;
            }
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n" );
    }
}

static void trace_print_event_data_scan_done( const smtc_modem_wifi_event_data_scan_done_t* data )
{
    if( data != NULL )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "SCAN_DONE info:\n" );
        SMTC_MODEM_HAL_TRACE_PRINTF( "-- number of results: %u\n", data->nbr_results );
        SMTC_MODEM_HAL_TRACE_PRINTF( "-- power consumption: %u nah\n", data->power_consumption_nah );
        SMTC_MODEM_HAL_TRACE_PRINTF( "-- scan duration: %u ms\n", data->scan_duration_ms );
        trace_print_scan_results( data );
    }
}

static void wifi_rp_task_launch( void* context )
{
    mw_wifi_task_obj.scan_start_time = smtc_modem_hal_get_time_in_ms( );
    SMTC_MODEM_HAL_TRACE_INFO( "Wi-Fi task launch at %u\n", mw_wifi_task_obj.scan_start_time );

    /* Reset previous results */
    memset( &wifi_results, 0, sizeof wifi_results );

    if( mw_radio_configure_for_scan( modem_get_radio_ctx( ) ) == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "wifi_rp_task_launch: mw_radio_configure_for_scan() failed\n" );
        rp_task_abort( modem_get_rp( ), RP_HOOK_ID_DIRECT_RP_ACCESS_WIFI );
        return;
    }

    /* Set Wi-Fi scan settings */
    smtc_wifi_settings_init( &wifi_settings );

    /* Start Wi-Fi scan */
    if( smtc_wifi_start_scan( modem_get_radio_ctx( ) ) != MW_RC_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "RP_TASK_WIFI - failed to start Wi-Fi scan, abort task\n" );
        rp_task_abort( modem_get_rp( ), RP_HOOK_ID_DIRECT_RP_ACCESS_WIFI );
        /* When aborting the task, the RP will call the end_task_callback() with SMTC_RP_RADIO_ABORTED status. */
        return;
    }
}

static void wifi_rp_task_done( void* status )
{
    uint32_t    tcurrent_ms;
    rp_status_t rp_status;

    mw_wifi_task_obj.scan_end_time = smtc_modem_hal_get_time_in_ms( );
    SMTC_MODEM_HAL_TRACE_INFO( "wifi_rp_task_done at %u (duration:%d ms)\n", mw_wifi_task_obj.scan_end_time,
                               mw_wifi_task_obj.scan_end_time - mw_wifi_task_obj.scan_start_time );

    /* WIFI scan completed or aborted - first thing to be done */
    smtc_wifi_scan_ended( );

    rp_get_status( ( radio_planner_t* ) status, mw_wifi_task_obj.rp_hook_id, &tcurrent_ms, &rp_status );
    if( rp_status == RP_STATUS_TASK_ABORTED )
    {
        /**/
        /* Do not perform any radio access here */
        /**/

        SMTC_MODEM_HAL_TRACE_WARNING( "Wi-Fi: RP_STATUS_TASK_ABORTED\n" );
        send_event( SMTC_MODEM_EVENT_WIFI_SCAN_DONE );
        send_event( SMTC_MODEM_EVENT_WIFI_TERMINATED );
        return;
    }
    else if( rp_status == RP_STATUS_WIFI_SCAN_DONE )
    {
        wifi_results.scan_duration_ms = mw_wifi_task_obj.scan_end_time - mw_wifi_task_obj.scan_start_time;

        /* Get results */
        wifi_scan_task_done( );

        /* Set the radio back to sleep when all radio accesses are done */
        mw_radio_set_sleep( modem_get_radio_ctx( ) );

        /* Notify application */
        send_event( SMTC_MODEM_EVENT_WIFI_SCAN_DONE );

        /* Send results over LoRaWAN */
        mw_wifi_send_add_task( &wifi_results );

        /* The scan sequence has completed */
        mw_wifi_task_obj.scan_sequence_started = false;
    }
    else
    {
        /* Should not happen */
        SMTC_MODEM_HAL_TRACE_ERROR( "Wi-Fi RP task - Unknown status %d\n", rp_status );
        SMTC_MODEM_HAL_PANIC( "Unexpected RP status" );
    }
}

static void wifi_scan_task_done( void )
{
    /* Wi-Fi scan completed, get and display the results */
    smtc_wifi_get_results( modem_get_radio_ctx( ), &wifi_results );

    /* Get scan power consumption */
    smtc_wifi_get_power_consumption( modem_get_radio_ctx( ), &wifi_results.power_consumption_nah );

    /* Sort results */
    smtc_wifi_sort_results( &wifi_results );
}

static void smtc_wifi_sort_results( wifi_scan_all_result_t* wifi_results )
{
    uint8_t                nb_results_sort = 0;
    wifi_scan_all_result_t wifi_results_tmp;

    WIFI_SCAN_TRACE_PRINTF_DEBUG( "Filter and Sort Wi-Fi results:\n" );

    /* raw data */
#if WIFI_SCAN_DEEP_DBG_TRACE == MODEM_HAL_FEATURE_ON
    SMTC_MODEM_HAL_TRACE_PRINTF( "Raw data:\n" );
    trace_print_scan_results( wifi_results );
#endif

    /* Remove AP */
    for( uint8_t index = 0; index < wifi_results->nbr_results; index++ )
    {
        if( wifi_results->results[index].origin == LR11XX_WIFI_ORIGIN_BEACON_FIX_AP )
        {
            wifi_results_tmp.results[nb_results_sort] = wifi_results->results[index];
            nb_results_sort++;
        }
    }

    /* Update main structure */
    for( uint8_t index = 0; index < nb_results_sort; index++ )
    {
        wifi_results->results[index] = wifi_results_tmp.results[index];
    }

    if( wifi_results->nbr_results != nb_results_sort )
    {
        wifi_results->nbr_results = nb_results_sort;

        /* FILTERED data */
#if WIFI_SCAN_DEEP_DBG_TRACE == MODEM_HAL_FEATURE_ON
        SMTC_MODEM_HAL_TRACE_PRINTF( "filtered data:\n" );
        trace_print_scan_results( wifi_results );
#endif
    }
    else
    {
        WIFI_SCAN_TRACE_PRINTF_DEBUG( "Nothing to filter\n" );
    }

    /* remove extra point if there are */
    if( wifi_results->nbr_results > WIFI_MAX_RESULTS_TO_SEND )
    {
        int i, j;
        for( i = 0; i < wifi_results->nbr_results - 1; i++ )
        {
            for( j = i + 1; j < wifi_results->nbr_results; j++ )
            {
                if( wifi_results->results[i].rssi < wifi_results->results[j].rssi )
                {
                    wifi_results_tmp.results[0] = wifi_results->results[i];
                    wifi_results->results[i]    = wifi_results->results[j];
                    wifi_results->results[j]    = wifi_results_tmp.results[0];
                }
            }
        }
        wifi_results->nbr_results = WIFI_MAX_RESULTS_TO_SEND;

        /* SORTED data */
#if WIFI_SCAN_DEEP_DBG_TRACE == MODEM_HAL_FEATURE_ON
        SMTC_MODEM_HAL_TRACE_PRINTF( "Sorted data:\n" );
        trace_print_scan_results( wifi_results );
#endif
    }
    else
    {
        WIFI_SCAN_TRACE_PRINTF_DEBUG( "Nothing to sort\n" );
    }
}

static void send_event( smtc_modem_event_type_t event )
{
    if( event == SMTC_MODEM_EVENT_WIFI_SCAN_DONE )
    {
        mw_wifi_task_obj.pending_evt_scan_done = true;
    }
    increment_asynchronous_msgnumber( event, 0, mw_wifi_task_obj.stack_id );
}

/* --- EOF ------------------------------------------------------------------ */
