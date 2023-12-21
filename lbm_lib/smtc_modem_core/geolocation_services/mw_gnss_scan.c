/**
 * @file      mw_gnss_scan.c
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
#include <math.h>

#include "mw_gnss_defs.h"
#include "mw_gnss_scan.h"
#include "mw_gnss_send.h"
#include "geolocation_bsp.h"
#include "mw_common.h"
#include "gnss_helpers.h"

#include "modem_core.h"
#include "modem_supervisor_light.h"
#include "modem_event_utilities.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_modem_geolocation_api.h"
#include "radio_planner.h"
#include "radio_planner_hook_id_defs.h"

#include "lr11xx_system.h"
#include "lr11xx_gnss.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )
#define NUMBER_MAX_OF_MW_GNSS_TASK_OBJ 1

/**
 * @brief Check is the index is valid before accessing the object
 */
#define IS_VALID_OBJECT_ID( x )                                                \
    do                                                                         \
    {                                                                          \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_MAX_OF_MW_GNSS_TASK_OBJ ); \
    } while( 0 )

/**
 * @brief Check is the index is valid before accessing the object
 */
#define IS_VALID_STACK_ID( x )                                   \
    do                                                           \
    {                                                            \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_OF_STACKS ); \
    } while( 0 )

#define IS_SERVICE_INITIALIZED( )                                          \
    do                                                                     \
    {                                                                      \
        if( mw_gnss_task_obj.initialized == false )                        \
        {                                                                  \
            SMTC_MODEM_HAL_TRACE_WARNING( "gnss service not launched\n" ); \
            break;                                                         \
        }                                                                  \
    } while( 0 )

/**
 * @brief Deep debug traces (ON/OFF)
 */
#ifndef GNSS_SCAN_DEEP_DBG_TRACE
#define GNSS_SCAN_DEEP_DBG_TRACE MODEM_HAL_FEATURE_OFF
#endif
#if( GNSS_SCAN_DEEP_DBG_TRACE )
#define GNSS_SCAN_TRACE_PRINTF_DEBUG( ... ) SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ )
#define GNSS_SCAN_TRACE_ARRAY_DEBUG( ... ) SMTC_MODEM_HAL_TRACE_ARRAY( __VA_ARGS__ )
#else
#define GNSS_SCAN_TRACE_PRINTF_DEBUG( ... )
#define GNSS_SCAN_TRACE_ARRAY_DEBUG( ... )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * @brief Rough maximum duration of an assisted scan for 2 constellations, in seconds
 */
#define GNSS_SCAN_DURATION_ASSISTED_S ( 20 )

/**
 * @brief Rough maximum duration of an autonomous scan for 2 constellations, in seconds
 */
#define GNSS_SCAN_DURATION_AUTONOMOUS_S ( 80 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Description of a GNSS scan mode
 */
typedef struct
{
    uint32_t scan_group_delay;  //!< The delay between the end of a scan and the start of the next one, in seconds
    uint8_t  scan_group_size;   //!< The number of scans in the scan group
} gnss_mw_mode_desc_t;

/**
 * @brief Description of the GNSS scan service context
 */
typedef struct mw_gnss_task_s
{
    /* service generic status */
    uint8_t stack_id;
    uint8_t task_id;
    uint8_t rp_hook_id;
    bool    initialized;
    bool    self_aborted;
    /* current configuration */
    lr11xx_gnss_constellation_mask_t constellations_mask;
    bool                             scan_aggregate;
    /* current gnss status */
    uint8_t                          current_token;
    smtc_modem_gnss_mode_t           current_mode_index;
    uint32_t                         current_almanac_crc;
    bool                             scan_sequence_started;
    lr11xx_gnss_scan_mode_launched_t last_scan_mode;
    bool                             pending_evt_scan_done;
    bool                             last_navgroup_valid;
} mw_gnss_task_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/**
 * @brief Object to hold the current service context
 */
static mw_gnss_task_t mw_gnss_task_obj = { 0 };

/**
 * @brief Current NAV group being populated by scan sequence
 */
static navgroup_t navgroup;

/*!
 * @brief Pre-defined scan modes to be selected by the user depending on the use case (STATIC, MOBILE...)
 */
static gnss_mw_mode_desc_t modes[__SMTC_MODEM_GNSS_MODE__SIZE] = {
    { .scan_group_delay = 15, .scan_group_size = 2 }, /* SMTC_MODEM_GNSS_MODE_STATIC */
    { .scan_group_delay = 0, .scan_group_size = 2 },  /* SMTC_MODEM_GNSS_MODE_MOBILE */
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Service callbacks
 */
static void    mw_gnss_scan_service_on_launch( void* context_callback );
static void    mw_gnss_scan_service_on_update( void* context_callback );
static uint8_t mw_gnss_scan_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/**
 * @brief Callback called by the radio planner when radio access is granted to the service
 */
static void gnss_rp_task_launch( void* context );

/**
 * @brief Callback called by the radio planner when task has been completed (scan done, aborted...)
 */
static void gnss_rp_task_done( void* status );

/**
 * @brief Process on task completion
 */
static mw_return_code_t gnss_scan_task_done( bool* navgroup_complete );

/**
 * @brief Program the next scan task
 */
static void gnss_scan_next( uint32_t delay_s );

/**
 * @brief Send an event to user to notify for progress
 */
static void send_event( smtc_modem_event_type_t event );

/**
 * @brief Send SCAN_DONE event, prepare buffer with metadata and send results
 */
static void terminate_navgroup( void );

/**
 * @brief Increment the NAV group token [0x02..0x1F]
 */
static void increment_token( void );

/**
 * @brief Helper function to print event data to console
 */
static void trace_print_event_data_scan_done( const smtc_modem_gnss_event_data_scan_done_t* data );

/**
 * @brief Helper function to print power consumption debug data to console
 */
#if GNSS_ENABLE_POWER_DEBUG_INFO
static void trace_print_cumulative_timing_and_power_consumption(
    const lr11xx_gnss_cumulative_timing_t* cumulative_timing, const uint32_t power_consumption_nah,
    const uint32_t power_consumption_nwh );
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void mw_gnss_scan_services_init( uint8_t* service_id, uint8_t task_id,
                                 uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                 void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                                 void** context_callback )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "mw_gnss_scan_services_init task_id %d, service_id %d, CURRENT_STACK:%d \n", task_id,
                                 *service_id, CURRENT_STACK );

    IS_VALID_OBJECT_ID( *service_id );

    /* Initialize global variables */
    memset( &mw_gnss_task_obj, 0, sizeof( mw_gnss_task_t ) );

    /* Service context */
    mw_gnss_task_obj.task_id     = task_id;
    mw_gnss_task_obj.stack_id    = CURRENT_STACK;
    mw_gnss_task_obj.rp_hook_id  = RP_HOOK_ID_DIRECT_RP_ACCESS_GNSS;
    mw_gnss_task_obj.initialized = true;
    *downlink_callback           = mw_gnss_scan_service_downlink_handler;
    *on_launch_callback          = mw_gnss_scan_service_on_launch;
    *on_update_callback          = mw_gnss_scan_service_on_update;
    *context_callback            = ( void* ) service_id;
    rp_hook_init( modem_get_rp( ), mw_gnss_task_obj.rp_hook_id, ( void ( * )( void* ) )( gnss_rp_task_done ),
                  modem_get_rp( ) );

    /* Configuration */
    mw_gnss_task_obj.constellations_mask = LR11XX_GNSS_GPS_MASK | LR11XX_GNSS_BEIDOU_MASK;

    /* Scan context */
    mw_gnss_task_obj.current_token =
        ( uint8_t ) smtc_modem_hal_get_random_nb_in_range( 2, 0x1F ); /* 5-bits token with 0x00 and 0x01 excluded */
    mw_gnss_task_obj.last_scan_mode = LR11XX_GNSS_LAST_SCAN_MODE_AUTONOMOUS_NO_TIME_NO_AP;
}

smtc_modem_return_code_t mw_gnss_scan_add_task( smtc_modem_gnss_mode_t mode, uint32_t start_delay_s )
{
    uint32_t now = smtc_modem_hal_get_time_in_s( );
    SMTC_MODEM_HAL_TRACE_PRINTF( "mw_gnss_scan_add_task: add task in supervisor at %u + %u s\n", now, start_delay_s );

    IS_SERVICE_INITIALIZED( );
    smodem_task task       = { 0 };
    task.id                = mw_gnss_task_obj.task_id;
    task.stack_id          = mw_gnss_task_obj.stack_id;
    task.priority          = TASK_BYPASS_DUTY_CYCLE; /* No TX for this service */
    task.time_to_execute_s = now + start_delay_s;
    if( modem_supervisor_add_task( &task ) != TASK_VALID )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to add GNSS scan supervisor task\n" );
        return SMTC_MODEM_RC_FAIL;
    }

    mw_gnss_task_obj.current_mode_index = mode;

    /* Clear navgroup context */
    memset( &navgroup, 0, sizeof( navgroup_t ) );

    /* Clear pending events */
    mw_gnss_task_obj.pending_evt_scan_done = false;

    /* Increment NAV group token if needed */
    if( ( mw_gnss_task_obj.last_navgroup_valid == true ) && ( mw_gnss_task_obj.scan_aggregate == false ) )
    {
        increment_token( );
    }

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t mw_gnss_scan_remove_task( void )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "mw_gnss_scan_remove_task\n" );

    IS_SERVICE_INITIALIZED( );

    if( mw_gnss_task_obj.scan_sequence_started == true )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "GNSS scan sequence started, too late to cancel\n" );
        return SMTC_MODEM_RC_BUSY;
    }

    if( modem_supervisor_remove_task( mw_gnss_task_obj.task_id ) != TASK_VALID )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to remove GNSS scan supervisor task\n" );
        return SMTC_MODEM_RC_FAIL;
    }

    return SMTC_MODEM_RC_OK;
}

void mw_gnss_scan_aggregate( bool aggregate )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "mw_gnss_scan_aggregate(%d)\n", aggregate );

    mw_gnss_task_obj.scan_aggregate = aggregate;

    /* Increment token for next aggregated group */
    if( aggregate == true )
    {
        increment_token( );
    }
}

smtc_modem_return_code_t mw_gnss_get_event_data_scan_done( smtc_modem_gnss_event_data_scan_done_t* data )
{
    if( data == NULL )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "mw_gnss_get_event_data_scan_done: Provided pointer is NULL\n" );
        return SMTC_MODEM_RC_INVALID;
    }

    if( mw_gnss_task_obj.pending_evt_scan_done == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "mw_gnss_get_event_data_scan_done: no SCAN_DONE event pending\n" );
        return SMTC_MODEM_RC_FAIL;
    }

    memset( data, 0, sizeof( smtc_modem_gnss_event_data_scan_done_t ) );

    data->is_valid             = ( navgroup.nb_scans_valid > 0 );
    data->token                = navgroup.token;
    data->timestamp            = navgroup.timestamp;
    data->nb_scans_valid       = navgroup.nb_scans_valid;
    data->navgroup_duration_ms = navgroup.end_time_ms - navgroup.start_time_ms;
    for( uint8_t i = 0; i < navgroup.nb_scans_valid; i++ )
    {
        data->scans[i].nav                = &navgroup.scans[i].results_buffer[GNSS_SCAN_METADATA_SIZE];
        data->scans[i].nav_size           = navgroup.scans[i].results_size - GNSS_SCAN_METADATA_SIZE;
        data->scans[i].timestamp          = navgroup.scans[i].gps_timestamp;
        data->scans[i].nb_svs             = navgroup.scans[i].nb_detected_svs;
        data->scans[i].info_svs           = &navgroup.scans[i].info_svs[0];
        data->scans[i].aiding_position    = navgroup.scans[i].aiding_position;
        data->scans[i].scan_mode_launched = navgroup.scans[i].scan_mode_launched;
        data->scans[i].scan_duration_ms   = navgroup.scans[i].end_time_ms - navgroup.scans[i].start_time_ms;
    }

    for( uint8_t i = 0; i < navgroup.nb_scans_total; i++ )
    {
        data->power_consumption_nah += navgroup.scans[i].power_consumption_nah;
    }

    data->context.mode        = mw_gnss_task_obj.current_mode_index;
    data->context.almanac_crc = mw_gnss_task_obj.current_almanac_crc;

    if( ( navgroup.nb_scans_valid == 0 ) && ( navgroup.scans[0].indoor_detected == true ) )
    {
        data->indoor_detected = true;
    }
    else
    {
        data->indoor_detected = false;  // Don't consider as indoor if there is a valid scan in the group
    }

    trace_print_event_data_scan_done( data );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t mw_gnss_set_constellations( smtc_modem_gnss_constellation_t constellations )
{
    switch( constellations )
    {
    case SMTC_MODEM_GNSS_CONSTELLATION_GPS:
        mw_gnss_task_obj.constellations_mask = LR11XX_GNSS_GPS_MASK;
        break;
    case SMTC_MODEM_GNSS_CONSTELLATION_BEIDOU:
        mw_gnss_task_obj.constellations_mask = LR11XX_GNSS_BEIDOU_MASK;
        break;
    case SMTC_MODEM_GNSS_CONSTELLATION_GPS_BEIDOU:
        mw_gnss_task_obj.constellations_mask = LR11XX_GNSS_GPS_MASK | LR11XX_GNSS_BEIDOU_MASK;
        break;
    default:
        return SMTC_MODEM_RC_INVALID;
    }

    return SMTC_MODEM_RC_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void mw_gnss_scan_service_on_launch( void* context_callback )
{
    uint32_t time_ms;

    GNSS_SCAN_TRACE_PRINTF_DEBUG( "mw_gnss_scan_service_on_launch\n" );

    IS_SERVICE_INITIALIZED( );

    /* From now, the scan sequence can not be cancelled */
    mw_gnss_task_obj.scan_sequence_started = true;

    time_ms = smtc_modem_hal_get_time_in_ms( ) + 300; /* 300ms for scheduling delay */

    rp_task_t rp_task        = { 0 };
    rp_task.hook_id          = mw_gnss_task_obj.rp_hook_id;
    rp_task.state            = RP_TASK_STATE_ASAP;
    rp_task.start_time_ms    = time_ms;
    rp_task.duration_time_ms = ( ( mw_gnss_task_obj.last_scan_mode == LR11XX_GNSS_LAST_SCAN_MODE_ASSISTED )
                                     ? GNSS_SCAN_DURATION_ASSISTED_S
                                     : GNSS_SCAN_DURATION_AUTONOMOUS_S ) *
                               1000;
    rp_task.type                           = RP_TASK_TYPE_GNSS_SNIFF;
    rp_task.launch_task_callbacks          = gnss_rp_task_launch;
    rp_radio_params_t fake_rp_radio_params = { 0 };
    if( rp_task_enqueue( modem_get_rp( ), &rp_task, NULL, 0, &fake_rp_radio_params ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to enqueue RP task for GNSS scan\n" );
        SMTC_MODEM_HAL_PANIC( );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Enqueued RP task for GNSS scan at %u (duration %u s)\n", time_ms,
                                     rp_task.duration_time_ms / 1000 );
    }
}

static void mw_gnss_scan_service_on_update( void* context_callback )
{
    GNSS_SCAN_TRACE_PRINTF_DEBUG( "mw_gnss_scan_service_on_update\n" );

    IS_SERVICE_INITIALIZED( );
}

static uint8_t mw_gnss_scan_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    GNSS_SCAN_TRACE_PRINTF_DEBUG( "mw_gnss_scan_service_downlink_handler\n" );

    return MODEM_DOWNLINK_UNCONSUMED;
}

static void gnss_scan_next( uint32_t delay_s )
{
    GNSS_SCAN_TRACE_PRINTF_DEBUG( "gnss_scan_next\n" );

    uint32_t time_ms  = smtc_modem_hal_get_time_in_ms( ) + 300; /* 300ms for scheduling delay */
    uint32_t delay_ms = delay_s * 1000;

    rp_task_t rp_task        = { 0 };
    rp_task.hook_id          = mw_gnss_task_obj.rp_hook_id;
    rp_task.state            = RP_TASK_STATE_ASAP;
    rp_task.start_time_ms    = time_ms + delay_ms;
    rp_task.duration_time_ms = ( ( mw_gnss_task_obj.last_scan_mode == LR11XX_GNSS_LAST_SCAN_MODE_ASSISTED )
                                     ? GNSS_SCAN_DURATION_ASSISTED_S
                                     : GNSS_SCAN_DURATION_AUTONOMOUS_S ) *
                               1000;
    rp_task.type                           = RP_TASK_TYPE_GNSS_SNIFF;
    rp_task.launch_task_callbacks          = gnss_rp_task_launch;
    rp_radio_params_t fake_rp_radio_params = { 0 };
    if( rp_task_enqueue( modem_get_rp( ), &rp_task, NULL, 0, &fake_rp_radio_params ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to enqueue RP task for GNSS scan\n" );
        SMTC_MODEM_HAL_PANIC( );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Enqueued RP task for GNSS scan at %u + %u (duration %u s)\n", time_ms, delay_ms,
                                     rp_task.duration_time_ms / 1000 );
    }
}

static void trace_print_event_data_scan_done( const smtc_modem_gnss_event_data_scan_done_t* data )
{
    if( data != NULL )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "SCAN_DONE info:\n" );
        SMTC_MODEM_HAL_TRACE_PRINTF( "-- token: 0x%02X\n", data->token );
        SMTC_MODEM_HAL_TRACE_PRINTF( "-- is_valid: %d\n", data->is_valid );
        SMTC_MODEM_HAL_TRACE_PRINTF( "-- timestamp: %u\n", data->timestamp );
        SMTC_MODEM_HAL_TRACE_PRINTF( "-- number of valid scans: %u\n", data->nb_scans_valid );
        for( uint8_t scan_idx = 0; scan_idx < data->nb_scans_valid; scan_idx++ )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "-- scan[%d][%u]\n", scan_idx, data->scans[scan_idx].timestamp );
            SMTC_MODEM_HAL_TRACE_ARRAY( "   NAV", data->scans[scan_idx].nav, data->scans[scan_idx].nav_size );
            SMTC_MODEM_HAL_TRACE_PRINTF( "   %u SV detected:\n", data->scans[scan_idx].nb_svs );
            for( uint8_t i = 0; i < data->scans[scan_idx].nb_svs; i++ )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "   SV_ID %u:\t%ddB\n", data->scans[scan_idx].info_svs[i].satellite_id,
                                             data->scans[scan_idx].info_svs[i].cnr );
            }
            smtc_gnss_trace_print_position( "   aiding position: ", &( data->scans[scan_idx].aiding_position ) );
            SMTC_MODEM_HAL_TRACE_PRINTF(
                "   scan mode launched: %s\n",
                smtc_gnss_scan_mode_launched_enum2str( data->scans[scan_idx].scan_mode_launched ) );

            SMTC_MODEM_HAL_TRACE_PRINTF( "   duration: %u ms\n", data->scans[scan_idx].scan_duration_ms );
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "-- power consumption: %u nah\n", data->power_consumption_nah );
        SMTC_MODEM_HAL_TRACE_PRINTF( "-- mode: %d\n", data->context.mode );
        SMTC_MODEM_HAL_TRACE_PRINTF( "-- almanac CRC: 0X%08X\n", data->context.almanac_crc );
        SMTC_MODEM_HAL_TRACE_PRINTF( "-- indoor detected: %d\n", data->indoor_detected );
        SMTC_MODEM_HAL_TRACE_PRINTF( "-- NAV group duration: %u ms\n", data->navgroup_duration_ms );
    }
}

#if GNSS_ENABLE_POWER_DEBUG_INFO
static void trace_print_cumulative_timing_and_power_consumption(
    const lr11xx_gnss_cumulative_timing_t* cumulative_timing, const uint32_t power_consumption_nah,
    const uint32_t power_consumption_nwh )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "cumulative_timing info:\n" );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--init %d ms\n", ( cumulative_timing->init * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--phase1_gps_capture %d ms\n",
                                 ( cumulative_timing->phase1_gps_capture * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--phase1_gps_process %d ms\n",
                                 ( cumulative_timing->phase1_gps_process * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--multiscan_gps_capture %d ms\n",
                                 ( cumulative_timing->multiscan_gps_capture * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--multiscan_gps_process %d ms\n",
                                 ( cumulative_timing->multiscan_gps_process * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--multiscan_gps_sleep_32k %d ms\n",
                                 ( cumulative_timing->multiscan_gps_sleep_32k * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--phase1_beidou_capture %d ms\n",
                                 ( cumulative_timing->phase1_beidou_capture * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--phase1_beidou_process %d ms\n",
                                 ( cumulative_timing->phase1_beidou_process * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--multiscan_beidou_capture %d ms\n",
                                 ( cumulative_timing->multiscan_beidou_capture * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--multiscan_beidou_process %d ms\n",
                                 ( cumulative_timing->multiscan_beidou_process * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--multiscan_beidou_sleep_32k %d ms\n",
                                 ( cumulative_timing->multiscan_beidou_sleep_32k * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--demod_capture %d ms\n", ( cumulative_timing->demod_capture * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--demod_process %d ms\n", ( cumulative_timing->demod_process * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--demod_sleep_32k %d ms\n", ( cumulative_timing->demod_sleep_32k * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--demod_sleep_32m %d ms\n", ( cumulative_timing->demod_sleep_32m * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--total_gps_capture %d ms\n",
                                 ( cumulative_timing->total_gps_capture * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--total_gps_process %d ms\n",
                                 ( cumulative_timing->total_gps_process * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--total_gps_sleep_32k %d ms\n",
                                 ( cumulative_timing->total_gps_sleep_32k * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--total_gps_sleep_32m %d ms\n",
                                 ( cumulative_timing->total_gps_sleep_32m * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--total_gps %d ms\n", ( cumulative_timing->total_gps * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--total_beidou_capture %d ms\n",
                                 ( cumulative_timing->total_beidou_capture * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--total_beidou_process %d ms\n",
                                 ( cumulative_timing->total_beidou_process * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--total_beidou_sleep_32k %d ms\n",
                                 ( cumulative_timing->total_beidou_sleep_32k * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--total_beidou_sleep_32m %d ms\n",
                                 ( cumulative_timing->total_beidou_sleep_32m * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--total_beidou %d ms\n", ( cumulative_timing->total_beidou * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--total_capture %d ms\n", ( cumulative_timing->total_capture * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--total_process %d ms\n", ( cumulative_timing->total_process * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--total_sleep_32k %d ms\n", ( cumulative_timing->total_sleep_32k * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--total_sleep_32m %d ms\n", ( cumulative_timing->total_sleep_32m * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--total %d\n", ( cumulative_timing->total * 1000 ) / 32768 );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--last_capture_size_32k_cnt %d\n", cumulative_timing->last_capture_size_32k_cnt );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--constellation_demod %d\n\n", cumulative_timing->constellation_demod );

    SMTC_MODEM_HAL_TRACE_PRINTF( "--power_consumption_nah %d\n", power_consumption_nah );
    SMTC_MODEM_HAL_TRACE_PRINTF( "--power_consumption_nwh %d\n\n", power_consumption_nwh );
}
#endif

static void gnss_rp_task_launch( void* context )
{
    lr11xx_status_t lr11xx_status;
    uint32_t        now;

    now = smtc_modem_hal_get_time_in_ms( );

    mw_gnss_task_obj.self_aborted = false;

    /* Store start time of current scan */
    navgroup.scans[navgroup.nb_scans_valid].start_time_ms = now;
    SMTC_MODEM_HAL_TRACE_INFO( "gnss task launch at %u for navgroup.scan[%d]\n",
                               navgroup.scans[navgroup.nb_scans_valid].start_time_ms, navgroup.nb_scans_valid );

    /* Store start time of NAV group */
    if( navgroup.nb_scans_total == 0 )
    {
        navgroup.start_time_ms = now;
        GNSS_SCAN_TRACE_PRINTF_DEBUG( "NAV group start time: %u ms\n", navgroup.start_time_ms );
    }

    /* Prepare for scan */
    lr11xx_status = lr11xx_system_set_dio_irq_params( modem_get_radio_ctx( ), LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE,
                                                      LR11XX_SYSTEM_IRQ_NONE );
    if( lr11xx_status != LR11XX_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "gnss_rp_task_launch: Failed to set irq params\n" );
        mw_gnss_task_obj.self_aborted = true; /* Stop the current NAV group */
        rp_task_abort( modem_get_rp( ), RP_HOOK_ID_DIRECT_RP_ACCESS_GNSS );
        return;
    }

    lr11xx_status =
        lr11xx_gnss_set_constellations_to_use( modem_get_radio_ctx( ), mw_gnss_task_obj.constellations_mask );
    if( lr11xx_status != LR11XX_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "gnss_rp_task_launch: Failed to set constellations\n" );
        mw_gnss_task_obj.self_aborted = true; /* Stop the current NAV group */
        rp_task_abort( modem_get_rp( ), RP_HOOK_ID_DIRECT_RP_ACCESS_GNSS );
        return;
    }

    /* Configure the board for gnss scan */
    geolocation_bsp_gnss_prescan_actions( );

    /* Check if the elapsed time since previous scan is too long (for non STATIC mode) */
    if( ( mw_gnss_task_obj.current_mode_index != SMTC_MODEM_GNSS_MODE_STATIC ) && ( navgroup.nb_scans_valid > 0 ) )
    {
        uint32_t time_since_end_of_previous_scan_ms = now - navgroup.scans[navgroup.nb_scans_valid - 1].end_time_ms;
        if( ( navgroup.scans[navgroup.nb_scans_valid - 1].scan_mode_launched != LR11XX_GNSS_LAST_SCAN_MODE_ASSISTED ) ||
            ( ( time_since_end_of_previous_scan_ms - ( modes[mw_gnss_task_obj.current_mode_index].scan_group_delay *
                                                       1000 ) ) > 2000 ) ) /* tolerate less than 2s delay */
        {
            GNSS_SCAN_TRACE_PRINTF_DEBUG( "Stop scan group:\n" );
            GNSS_SCAN_TRACE_PRINTF_DEBUG( " - previous scan mode: %s\n",
                                          smtc_gnss_scan_mode_launched_enum2str(
                                              navgroup.scans[navgroup.nb_scans_valid - 1].scan_mode_launched ) );
            GNSS_SCAN_TRACE_PRINTF_DEBUG( " - elapsed time since end of previous valid scan: %u ms\n",
                                          time_since_end_of_previous_scan_ms );
            mw_gnss_task_obj.self_aborted = true; /* Stop the current NAV group */
            rp_task_abort( modem_get_rp( ), RP_HOOK_ID_DIRECT_RP_ACCESS_GNSS );
            return;
        }
    }

    if( mw_radio_configure_for_scan( modem_get_radio_ctx( ) ) == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "gnss_rp_task_launch: mw_radio_configure_for_scan() failed\n" );
        mw_gnss_task_obj.self_aborted = true; /* Stop the current NAV group */
        rp_task_abort( modem_get_rp( ), RP_HOOK_ID_DIRECT_RP_ACCESS_GNSS );
        return;
    }

    /* Start scan */
    uint8_t scan_parameters = LR11XX_GNSS_RESULTS_DOPPLER_ENABLE_MASK | LR11XX_GNSS_RESULTS_DOPPLER_MASK |
                              LR11XX_GNSS_RESULTS_DEMODULATE_TIME_MASK;
    lr11xx_status = lr11xx_gnss_scan( modem_get_radio_ctx( ), LR11XX_GNSS_OPTION_MID_EFFORT, scan_parameters,
                                      GNSS_RESULT_NB_SVS_MAX );
    if( lr11xx_status != LR11XX_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "lr11xx_gnss_scan() failed\n" );
        mw_gnss_task_obj.self_aborted = true; /* Stop the current NAV group */
        rp_task_abort( modem_get_rp( ), RP_HOOK_ID_DIRECT_RP_ACCESS_GNSS );
        return;
    }
}

static void gnss_rp_task_done( void* status )
{
    uint32_t    tcurrent_ms;
    rp_status_t rp_status;
    uint32_t    now = smtc_modem_hal_get_time_in_ms( );

    navgroup.scans[navgroup.nb_scans_valid].end_time_ms = now;
    SMTC_MODEM_HAL_TRACE_INFO(
        "gnss_rp_task_done at %u (duration:%u ms)\n", navgroup.scans[navgroup.nb_scans_valid].end_time_ms,
        navgroup.scans[navgroup.nb_scans_valid].end_time_ms - navgroup.scans[navgroup.nb_scans_valid].start_time_ms );

    geolocation_bsp_gnss_postscan_actions( );

    rp_get_status( ( radio_planner_t* ) status, mw_gnss_task_obj.rp_hook_id, &tcurrent_ms, &rp_status );
    if( rp_status == RP_STATUS_TASK_ABORTED )
    {
        /**/
        /* Do not perform any radio access here */
        /**/

        if( mw_gnss_task_obj.self_aborted == false )
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "GNSS scan task aborted by RP\n" );
            gnss_scan_next( modes[mw_gnss_task_obj.current_mode_index].scan_group_delay );
            return;
        }
        else
        {
            mw_gnss_task_obj.self_aborted = false;

            SMTC_MODEM_HAL_TRACE_PRINTF( "GNSS scan task self aborted\n" );

            /* Update NAV group info before sending event */
            navgroup.end_time_ms = now;
            GNSS_SCAN_TRACE_PRINTF_DEBUG( "NAV group end time: %u ms\n", navgroup.end_time_ms );

            /* Stop the current NAV group and send results */
            terminate_navgroup( );

            /* The scan sequence has completed */
            mw_gnss_task_obj.scan_sequence_started = false;
        }
    }
    else if( rp_status == RP_STATUS_GNSS_SCAN_DONE )
    {
        bool             scan_done = false;
        mw_return_code_t mw_status = gnss_scan_task_done( &scan_done );
        mw_radio_set_sleep( modem_get_radio_ctx( ) ); /* Set the radio back to sleep when all radio accesses are done */
        if( ( mw_status == MW_RC_OK ) && ( scan_done == false ) )
        {
            gnss_scan_next( modes[mw_gnss_task_obj.current_mode_index].scan_group_delay );
        }
        else
        {
            /* Set NAV group end time */
            navgroup.end_time_ms = now;
            GNSS_SCAN_TRACE_PRINTF_DEBUG( "NAV group end time: %u ms\n", navgroup.end_time_ms );

            /* Stop the current NAV group and send results */
            terminate_navgroup( );

            /* The scan sequence has completed */
            mw_gnss_task_obj.scan_sequence_started = false;
        }
    }
    else
    {
        /* Should not happen */
        SMTC_MODEM_HAL_TRACE_ERROR( "GNSS RP task - Unknown status %d\n", rp_status );
        SMTC_MODEM_HAL_PANIC( "Unexpected RP status" );
    }
}

static mw_return_code_t gnss_scan_task_done( bool* navgroup_complete )
{
    lr11xx_gnss_time_t                               gps_time;
    lr11xx_gnss_solver_assistance_position_t         aiding_pos;
    uint16_t                                         scan_result_size;
    lr11xx_gnss_context_status_bytestream_t          context_status_bytestream;
    lr11xx_gnss_context_status_t                     context_status;
    lr11xx_gnss_cumulative_timing_t                  cumulative_timing;
    uint32_t                                         power_consumption_nah;
    uint32_t                                         power_consumption_nwh;
    lr11xx_gnss_instantaneous_power_consumption_ua_t instantaneous_power_consumption_ua;

    /* Initialize output parameters */
    *navgroup_complete = true; /* initialized to true to end the NAV group in case of any error */

    /* Set current scan index in NAV group */
    uint8_t scan_index = navgroup.nb_scans_valid;

    /* Get current gps time (nb of seconds modulo 1024 weeks) */
    MW_RETURN_ON_FAILURE( lr11xx_gnss_read_time( modem_get_radio_ctx( ), &gps_time ) == LR11XX_STATUS_OK );
    if( gps_time.error_code == LR11XX_GNSS_READ_TIME_STATUS_NO_ERROR )
    {
        GNSS_SCAN_TRACE_PRINTF_DEBUG( "GPS time: %u (0x%08X)\n", gps_time.gps_time_s, gps_time.gps_time_s );
        navgroup.scans[scan_index].gps_timestamp = gps_time.gps_time_s;
        navgroup.timestamp =
            gps_time.gps_time_s; /* set NAV group global timestamp with the last scan tentative (even if not valid) */
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "GPS time: no time available - error code 0x%02X (%s)\n", gps_time.error_code,
                                      smtc_gnss_read_time_error_code_enum2str( gps_time.error_code ) );
    }

    /* Get scan mode used for this scan (autonomous, assisted... ) */
    MW_RETURN_ON_FAILURE( lr11xx_gnss_read_last_scan_mode_launched(
                              modem_get_radio_ctx( ), &( mw_gnss_task_obj.last_scan_mode ) ) == LR11XX_STATUS_OK );
    GNSS_SCAN_TRACE_PRINTF_DEBUG( "Last GNSS scan mode launched: %s\n",
                                  smtc_gnss_scan_mode_launched_enum2str( mw_gnss_task_obj.last_scan_mode ) );
    navgroup.scans[scan_index].scan_mode_launched = mw_gnss_task_obj.last_scan_mode;

    /* Get current almanac CRC */
    MW_RETURN_ON_FAILURE( lr11xx_gnss_get_context_status( modem_get_radio_ctx( ), context_status_bytestream ) ==
                          LR11XX_STATUS_OK );
    MW_RETURN_ON_FAILURE( lr11xx_gnss_parse_context_status_buffer( context_status_bytestream, &context_status ) ==
                          LR11XX_STATUS_OK );
    mw_gnss_task_obj.current_almanac_crc = context_status.global_almanac_crc;

    /* Get results */
    MW_RETURN_ON_FAILURE( lr11xx_gnss_get_result_size( modem_get_radio_ctx( ), &scan_result_size ) ==
                          LR11XX_STATUS_OK );
    MW_RETURN_ON_FAILURE( lr11xx_gnss_read_results( modem_get_radio_ctx( ), navgroup.scans[scan_index].results_buffer,
                                                    scan_result_size ) == LR11XX_STATUS_OK );
    GNSS_SCAN_TRACE_ARRAY_DEBUG( "NAV3", navgroup.scans[scan_index].results_buffer, scan_result_size );

    /* Sanity check: at least 2 bytes for a scan result */
    MW_RETURN_ON_FAILURE( scan_result_size >= 2 );

    /* Fetch the detected SVs */
    MW_RETURN_ON_FAILURE(
        lr11xx_gnss_get_nb_detected_satellites( modem_get_radio_ctx( ), &navgroup.scans[scan_index].nb_detected_svs ) ==
        LR11XX_STATUS_OK );

    /* Get details about all detected SVs (even if not given in NAV) */
    if( navgroup.scans[scan_index].nb_detected_svs > 0 )
    {
        MW_RETURN_ON_FAILURE(
            lr11xx_gnss_get_detected_satellites( modem_get_radio_ctx( ), navgroup.scans[scan_index].nb_detected_svs,
                                                 navgroup.scans[scan_index].info_svs ) == LR11XX_STATUS_OK );
    }

    if( navgroup.scans[scan_index].results_buffer[0] == LR11XX_GNSS_DESTINATION_HOST )
    {
        const lr11xx_gnss_message_host_status_t status_code_raw =
            ( lr11xx_gnss_message_host_status_t ) navgroup.scans[scan_index].results_buffer[1];
        switch( status_code_raw )
        {
        case LR11XX_GNSS_HOST_NO_SATELLITE_DETECTED:
            SMTC_MODEM_HAL_TRACE_INFO( "GNSS: NO_SATELLITE_DETECTED\n" );
            SMTC_MODEM_HAL_TRACE_WARNING( "Indoor detected?\n" );
            navgroup.scans[scan_index].indoor_detected = true;
            break;
        case LR11XX_GNSS_HOST_ASSISTANCE_POSITION_POSSIBLY_WRONG_BUT_FAILS_TO_UPDATE:
            SMTC_MODEM_HAL_TRACE_WARNING( "GNSS: ASSISTANCE_POSITION_POSSIBLY_WRONG_BUT_FAILS_TO_UPDATE\n" );
            SMTC_MODEM_HAL_TRACE_INFO( "Reset current position\n" );
            /* Reset current position and let LR11xx restart with autonomous scan to recover */
            MW_RETURN_ON_FAILURE( lr11xx_gnss_reset_position( modem_get_radio_ctx( ) ) == LR11XX_STATUS_OK );
            break;
        default:
            SMTC_MODEM_HAL_TRACE_INFO( "GNSS message to host: %s\n",
                                       smtc_gnss_message_host_status_enum2str( status_code_raw ) );
            break;
        }
    }

    /* Get current assistance position (could have been reset above) */
    MW_RETURN_ON_FAILURE( lr11xx_gnss_read_assistance_position( modem_get_radio_ctx( ), &aiding_pos ) ==
                          LR11XX_STATUS_OK );
    smtc_gnss_trace_print_position( "Assistance Position: ", &aiding_pos );
    navgroup.scans[scan_index].aiding_position = aiding_pos;

    /* Get Power consumption */
    MW_RETURN_ON_FAILURE( lr11xx_gnss_read_cumulative_timing( modem_get_radio_ctx( ), &cumulative_timing ) ==
                          LR11XX_STATUS_OK );
    geolocation_bsp_gnss_get_consumption( &instantaneous_power_consumption_ua );
    lr11xx_gnss_compute_power_consumption( &cumulative_timing, &instantaneous_power_consumption_ua,
                                           &power_consumption_nah, &power_consumption_nwh );
    navgroup.scans[scan_index].power_consumption_nah = power_consumption_nah;
#if GNSS_ENABLE_POWER_DEBUG_INFO
    trace_print_cumulative_timing_and_power_consumption( &cumulative_timing, power_consumption_nah,
                                                         power_consumption_nwh );
#endif

    /* Update number of total scan tentative for this NAV group */
    navgroup.nb_scans_total += 1;

    /* Stop scan group if no time available */
    if( gps_time.error_code != LR11XX_GNSS_READ_TIME_STATUS_NO_ERROR )
    {
        return MW_RC_OK;
    }

    /* Return results */
    if( ( navgroup.scans[scan_index].results_buffer[0] != LR11XX_GNSS_DESTINATION_HOST ) &&
        ( navgroup.scans[scan_index].nb_detected_svs >= 3 ) )
    {
        /* Add valid scan to navgroup */
        navgroup.nb_scans_valid += 1;

        /* Erase destination byte, will be used for navgroup metadata */
        if( GNSS_SCAN_METADATA_SIZE != 1 )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "NAVGROUP metadata size is not 1, need to adjust result buffer !\n" );
            return MW_RC_FAILED;
        }
        navgroup.scans[scan_index].results_buffer[0] = 0x00;

        /* Update scan size */
        navgroup.scans[scan_index].results_size = scan_result_size - 1 + GNSS_SCAN_METADATA_SIZE;
    }

    /* Check if the NAV group is complete or not */
    if( ( navgroup.scans[scan_index].indoor_detected == false ) &&
        ( navgroup.nb_scans_total < modes[mw_gnss_task_obj.current_mode_index].scan_group_size ) )
    {
        *navgroup_complete = false;
    }

    /* Sanity check */
    if( navgroup.nb_scans_valid > GNSS_NAVGROUP_SIZE_MAX )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "OVERFLOW? (%s:%d)\n", __func__, __LINE__ );
        return MW_RC_FAILED;
    }

    return MW_RC_OK;
}

static void send_event( smtc_modem_event_type_t event )
{
    if( event == SMTC_MODEM_EVENT_GNSS_SCAN_DONE )
    {
        mw_gnss_task_obj.pending_evt_scan_done = true;
    }
    increment_asynchronous_msgnumber( event, 0, mw_gnss_task_obj.stack_id );
}

static void terminate_navgroup( void )
{
    /* Save current scan group token to be used for SCAN_DONE event */
    navgroup.token = mw_gnss_task_obj.current_token;

    /* Notify application */
    send_event( SMTC_MODEM_EVENT_GNSS_SCAN_DONE );

    /* Save navgroup status for next scan to know if token needs to be incremented */
    if( navgroup.nb_scans_valid > 0 )
    {
        mw_gnss_task_obj.last_navgroup_valid = true;
    }
    else
    {
        mw_gnss_task_obj.last_navgroup_valid = false;
    }

    /* Update navgroup frames metadata before sending over the air */
    for( uint8_t i = 0; i < navgroup.nb_scans_valid; i++ )
    {
        bool is_last                        = ( i == ( navgroup.nb_scans_valid - 1 ) ? true : false );
        navgroup.scans[i].results_buffer[0] = ( is_last << 7 ) | ( navgroup.token & 0x1F );
    }

    /* Send results */
    mw_gnss_send_add_task( &navgroup );
}

static void increment_token( void )
{
    mw_gnss_task_obj.current_token = ( mw_gnss_task_obj.current_token + 1 ) % 0x20; /* roll-over on 5-bits */

    /* Exclude 0x00 and 0x01 values */
    if( mw_gnss_task_obj.current_token == 0 )
    {
        mw_gnss_task_obj.current_token = 2;
    }
}

/* --- EOF ------------------------------------------------------------------ */
