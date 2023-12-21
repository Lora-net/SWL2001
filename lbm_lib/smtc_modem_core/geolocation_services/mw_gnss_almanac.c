/**
 * @file      mw_gnss_almanac.c
 *
 * @brief     GNSS almanac demodulation service
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

#include "geolocation_bsp.h"
#include "mw_common.h"
#include "mw_gnss_defs.h"
#include "mw_gnss_almanac.h"
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
#define NUMBER_MAX_OF_MW_GNSS_ALMANAC_TASK_OBJ 1

#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )

/**
 * @brief Check is the index is valid before accessing the object
 */
#define IS_VALID_OBJECT_ID( x )                                                        \
    do                                                                                 \
    {                                                                                  \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_MAX_OF_MW_GNSS_ALMANAC_TASK_OBJ ); \
    } while( 0 )

/**
 * @brief Check is the index is valid before accessing the object
 */
#define IS_VALID_STACK_ID( x )                                   \
    do                                                           \
    {                                                            \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_OF_STACKS ); \
    } while( 0 )

#define IS_SERVICE_INITIALIZED( )                                                  \
    do                                                                             \
    {                                                                              \
        if( mw_gnss_almanac_task_obj.initialized == false )                        \
        {                                                                          \
            SMTC_MODEM_HAL_TRACE_WARNING( "gnss almanac service not launched\n" ); \
            break;                                                                 \
        }                                                                          \
    } while( 0 )

/**
 * @brief A bort the Radio Planner task, set the flag, and return
 */
#define RP_TASK_ABORT_AND_RETURN( )                                                 \
    do                                                                              \
    {                                                                               \
        mw_gnss_almanac_task_obj.self_aborted = true;                               \
        rp_task_abort( modem_get_rp( ), RP_HOOK_ID_DIRECT_RP_ACCESS_GNSS_ALMANAC ); \
        return;                                                                     \
    } while( 0 )

/**
 * @brief Deep debug traces (ON/OFF)
 */
#ifndef GNSS_ALMANAC_DEEP_DBG_TRACE
#define GNSS_ALMANAC_DEEP_DBG_TRACE MODEM_HAL_FEATURE_OFF
#endif
#if( GNSS_ALMANAC_DEEP_DBG_TRACE )
#define GNSS_ALMANAC_TRACE_PRINTF_DEBUG( ... ) SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ )
#define GNSS_ALMANAC_TRACE_ARRAY_DEBUG( ... ) SMTC_MODEM_HAL_TRACE_ARRAY( __VA_ARGS__ )
#else
#define GNSS_ALMANAC_TRACE_PRINTF_DEBUG( ... )
#define GNSS_ALMANAC_TRACE_ARRAY_DEBUG( ... )
#endif

#define RP_NOW_MS ( smtc_modem_hal_get_time_in_ms( ) + 4 ) /* 4ms to be below RP MARGIN_DELAY*/

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * @brief Delay to relaunch a read_status when no time is available or when the service has been aborted by the Radio
 * Planner.
 */
#define ALMANAC_STATUS_CHECK_PERIOD_NO_TIME_S ( 5 * 60 ) /* 5 minutes */

/**
 * @brief Delay to postpone the almanac update when bad sky conditions have been detected for a constellation.
 */
#define ALMANAC_STATUS_CHECK_PERIOD_BAD_SKY_S ( 60 * 60 ) /* 1 hour */

/**
 * @brief Delay to relaunch a read_status when time_accuracy is too low. The service expects a GNSS scan to have
 * successfully demodulated TOW in the previous 20 minutes.
 */
#define ALMANAC_STATUS_CHECK_PERIOD_TIME_ACCURACY_S ( 10 * 60 ) /* 10 minutes */

/**
 * @brief Delay to check for almanac status when the almanac has been fully updated.
 */
#define ALMANAC_STATUS_CHECK_PERIOD_DEFAULT_S ( 8 * 60 * 60 ) /* 8 hours */

/**
 * @brief Maximum elapsed time allowed between 2 consecutive calls to lr11xx_gnss_read_time().
 * It is necessary for the LR11xx to maintain its internal time and handle roll-over.
 */
#define ALMANAC_READ_TIME_THRESHOLD_MAX ( 24 * 60 * 60 ) /* 24 hours */

/**
 * @brief Pre-delay to start a scan for an almanac
 */
#define ALMANAC_UPDATE_FROM_SAT_WARMUP_DURATION_MS ( 1300 ) /* 1.3 seconds */

/**
 * @brief Margin delay to add to warmup for GPS
 */
#define ALMANAC_UPDATE_FROM_SAT_GPS_MARGIN_DURATION_MS ( 1000 ) /* 1 second */

/**
 * @brief Force almanac RP task to be high priority if too many aborts occurred
 */
#define ALMANAC_UPDATE_ABORTED_BY_RP_THRESHOLD_FOR_LOW_PRIORITY ( 10 )

/**
 * @brief Maximum duration of a scan for almanac, in seconds
 */
#define ALMANAC_UPDATE_SCAN_DURATION_S ( 16 )

/**
 * @brief The number of SV that can remain not updated for a constellation to consider that the update is complete
 */
#define ALMANAC_UPDATE_NB_SV_TO_BE_UPDATED_THRESHOLD ( 2 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Almanac internal task type
 */
typedef enum mw_gnss_almanac_task_type_e
{
    ALMANAC_TASK_TYPE_READ_STATUS = 0,  //!< task for read status, with low timing precision constraint
    ALMANAC_TASK_TYPE_SCAN        = 1   //!< task for almanac scan, with high timing precision constraint
} mw_gnss_almanac_task_type_t;

/**
 * @brief Structure to describe the context of the almanac update service
 */
typedef struct mw_gnss_almanac_task_s
{
    uint8_t  stack_id;
    uint8_t  task_id;
    uint8_t  rp_hook_id;
    bool     initialized;
    bool     self_aborted;
    uint8_t  nb_rp_abort;
    bool     pending_evt_update_done;
    uint32_t gps_update_postpone_time_s;
    uint32_t beidou_update_postpone_time_s;
    uint32_t last_read_time_s;
    /* Configuration */
    lr11xx_gnss_constellation_mask_t constellations_enabled;
} mw_gnss_almanac_task_t;

/**
 * @brief Structure to describe the configuration for the next almanac to be updated
 * @struct mw_gnss_almanac_next_update_s
 */
typedef struct mw_gnss_almanac_next_update_s
{
    mw_gnss_almanac_task_type_t type;  //!< the type of the next almanac task
    uint32_t time_ms;  //!< the duration in milliseconds before the next start subframe where to catch the new almanac.
    uint32_t time_s;   //!< the delay in seconds to run the next supervisor task for almanac read status.
    lr11xx_gnss_constellation_t constellation;  //!< the constellation to be updated

} mw_gnss_almanac_next_update_t;

/**
 * @brief Structure to describe the almanac update status
 */
typedef struct mw_gnss_almanac_update_status_s
{
    /* Update status */
    smtc_modem_gnss_almanac_update_status_t status_gps;
    smtc_modem_gnss_almanac_update_status_t status_beidou;
    uint8_t                                 remaining_sv_to_be_updated_gps;
    uint8_t                                 remaining_sv_to_be_updated_beidou;
    uint8_t                                 total_sv_to_be_updated_gps;
    uint8_t                                 total_sv_to_be_updated_beidou;
    uint8_t                                 update_progress_gps;
    uint8_t                                 update_progress_beidou;
    uint32_t                                power_consumption_nah;
    lr11xx_gnss_doppler_solver_error_code_t last_doppler_solver_status;
    /* Statistics */
    uint32_t stat_nb_update_from_sat_done;
    uint32_t stat_nb_update_from_sat_success;
    uint32_t stat_nb_aborted_by_rp;
    uint32_t stat_cumulative_timings_s;
} mw_gnss_almanac_update_status_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/**
 * @brief Object to hold the current service context
 */
static mw_gnss_almanac_task_t mw_gnss_almanac_task_obj = { 0 };

/**
 * @brief Next almanac update task configuration
 */
static mw_gnss_almanac_next_update_t mw_gnss_almanac_next_update;

/**
 * @brief Almanac update status to be returned to the user when the task has been completed
 */
static mw_gnss_almanac_update_status_t mw_gnss_almanac_update_status;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Service callbacks
 */
static void    mw_gnss_almanac_service_on_launch( void* context_callback );
static void    mw_gnss_almanac_service_on_update( void* context_callback );
static uint8_t mw_gnss_almanac_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/**
 * @brief Program the next task (supervisor or radio planner)
 */
static void mw_gnss_almanac_next( void );

/**
 * @brief Program the next supervisor task with delay given in seconds
 */
static void mw_gnss_almanac_next_supervisor( uint32_t delay_s );

/**
 * @brief Program the next radio planner task with delay given in milliseconds
 */
static void mw_gnss_almanac_next_rp( uint32_t delay_ms );

/**
 * @brief Callback called by the radio planner when radio access is granted to the service
 */
static void gnss_almanac_rp_task_launch( void* context );

/**
 * @brief Callback called by the radio planner when task has been completed (scan done, aborted...)
 */
static void gnss_almanac_rp_task_done( void* status );

/**
 * @brief Process on task completion
 */
static mw_return_code_t gnss_almanac_task_done( void );

/**
 * @brief Configure the next update task based on last almanac status read
 */
static mw_gnss_almanac_next_update_t select_next_update( const lr11xx_gnss_read_almanac_status_t* almanac_status );

/**
 * @brief Send an event to user to notify for progress
 */
static void send_event( smtc_modem_event_type_t event );

/**
 * @brief Update the current almanac status context with given status
 */
static void update_current_almanac_status_context( const lr11xx_gnss_read_almanac_status_t* almanac_status );

/*
 * -----------------------------------------------------------------------------
 * --- HELPER FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Helper function to print almanac status to console
 */
static void print_almanac_status( const lr11xx_gnss_read_almanac_status_t* almanac_status );

/**
 * @brief Helper function to print event data to console
 */
static void trace_print_event_data_almanac_update(
    const smtc_modem_almanac_demodulation_event_data_almanac_update_t* data );

/**
 * @brief Helper function to count bits set in a uint32_t
 */
static uint32_t count_set_bits( uint32_t bitmask );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void mw_gnss_almanac_services_init( uint8_t* service_id, uint8_t task_id,
                                    uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                    void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                                    void** context_callback )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "mw_gnss_almanac_services_init task_id %d, service_id %d, CURRENT_STACK:%d \n",
                                 task_id, *service_id, CURRENT_STACK );

    IS_VALID_OBJECT_ID( *service_id );

    /* Initialize global variables */
    memset( &mw_gnss_almanac_task_obj, 0, sizeof( mw_gnss_almanac_task_t ) );
    memset( &mw_gnss_almanac_next_update, 0, sizeof( mw_gnss_almanac_next_update_t ) );
    memset( &mw_gnss_almanac_update_status, 0, sizeof( mw_gnss_almanac_update_status_t ) );

    /* Service context */
    mw_gnss_almanac_task_obj.task_id     = task_id;
    mw_gnss_almanac_task_obj.stack_id    = CURRENT_STACK;
    mw_gnss_almanac_task_obj.rp_hook_id  = RP_HOOK_ID_DIRECT_RP_ACCESS_GNSS_ALMANAC;
    mw_gnss_almanac_task_obj.initialized = true;
    *downlink_callback                   = mw_gnss_almanac_service_downlink_handler;
    *on_launch_callback                  = mw_gnss_almanac_service_on_launch;
    *on_update_callback                  = mw_gnss_almanac_service_on_update;
    *context_callback                    = ( void* ) service_id;
    rp_hook_init( modem_get_rp( ), mw_gnss_almanac_task_obj.rp_hook_id,
                  ( void ( * )( void* ) )( gnss_almanac_rp_task_done ), modem_get_rp( ) );

    /* Configuration */
    mw_gnss_almanac_task_obj.constellations_enabled = LR11XX_GNSS_GPS_MASK | LR11XX_GNSS_BEIDOU_MASK;
}

void mw_gnss_almanac_add_task( void )
{
    IS_SERVICE_INITIALIZED( );

    mw_gnss_almanac_next_supervisor( 0 );
}

smtc_modem_return_code_t mw_gnss_almanac_get_event_almanac_update(
    smtc_modem_almanac_demodulation_event_data_almanac_update_t* data )
{
    if( data == NULL )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "mw_gnss_almanac_get_event_almanac_update: Provided pointer is NULL\n" );
        return SMTC_MODEM_RC_INVALID;
    }

    if( mw_gnss_almanac_task_obj.pending_evt_update_done == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR(
            "mw_gnss_almanac_get_event_almanac_update: no ALMANAC_DEMOD_UPDATE event pending\n" );
        return SMTC_MODEM_RC_FAIL;
    }

    memset( data, 0, sizeof( smtc_modem_almanac_demodulation_event_data_almanac_update_t ) );

    /* Update status */
    data->status_gps             = mw_gnss_almanac_update_status.status_gps;
    data->status_beidou          = mw_gnss_almanac_update_status.status_beidou;
    data->update_progress_gps    = mw_gnss_almanac_update_status.update_progress_gps;
    data->update_progress_beidou = mw_gnss_almanac_update_status.update_progress_beidou;
    data->power_consumption_nah  = mw_gnss_almanac_update_status.power_consumption_nah;

    /* Statistics */
    data->stat_nb_update_from_sat_done    = mw_gnss_almanac_update_status.stat_nb_update_from_sat_done;
    data->stat_nb_update_from_sat_success = mw_gnss_almanac_update_status.stat_nb_update_from_sat_success;
    data->stat_nb_aborted_by_rp           = mw_gnss_almanac_update_status.stat_nb_aborted_by_rp;
    data->stat_cumulative_timings_s       = mw_gnss_almanac_update_status.stat_cumulative_timings_s;

    trace_print_event_data_almanac_update( data );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t mw_gnss_almanac_set_constellations( smtc_modem_gnss_constellation_t constellations )
{
    switch( constellations )
    {
    case SMTC_MODEM_GNSS_CONSTELLATION_GPS:
        mw_gnss_almanac_task_obj.constellations_enabled = LR11XX_GNSS_GPS_MASK;
        break;
    case SMTC_MODEM_GNSS_CONSTELLATION_BEIDOU:
        mw_gnss_almanac_task_obj.constellations_enabled = LR11XX_GNSS_BEIDOU_MASK;
        break;
    case SMTC_MODEM_GNSS_CONSTELLATION_GPS_BEIDOU:
        mw_gnss_almanac_task_obj.constellations_enabled = LR11XX_GNSS_GPS_MASK | LR11XX_GNSS_BEIDOU_MASK;
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

static void mw_gnss_almanac_service_on_launch( void* context_callback )
{
    GNSS_ALMANAC_TRACE_PRINTF_DEBUG( "mw_gnss_almanac_service_on_launch\n" );

    IS_SERVICE_INITIALIZED( );

    rp_task_t rp_task                      = { 0 };
    rp_task.hook_id                        = mw_gnss_almanac_task_obj.rp_hook_id;
    rp_task.state                          = RP_TASK_STATE_ASAP;
    rp_task.start_time_ms                  = RP_NOW_MS;
    rp_task.duration_time_ms               = ALMANAC_UPDATE_SCAN_DURATION_S * 1000;
    rp_task.type                           = RP_TASK_TYPE_GNSS_SNIFF;
    rp_task.launch_task_callbacks          = gnss_almanac_rp_task_launch;
    rp_radio_params_t fake_rp_radio_params = { 0 };
    if( rp_task_enqueue( modem_get_rp( ), &rp_task, NULL, 0, &fake_rp_radio_params ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to enqueue RP task for GNSS almanac demodulation\n" );
        SMTC_MODEM_HAL_PANIC( );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Enqueued RP task (ASAP) for GNSS almanac demodulation (hook_id #%d)\n",
                                     rp_task.hook_id );
    }
}

static void mw_gnss_almanac_service_on_update( void* context_callback )
{
    GNSS_ALMANAC_TRACE_PRINTF_DEBUG( "mw_gnss_almanac_service_on_update\n" );

    IS_SERVICE_INITIALIZED( );
}

static uint8_t mw_gnss_almanac_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    GNSS_ALMANAC_TRACE_PRINTF_DEBUG( "mw_gnss_almanac_service_downlink_handler\n" );

    return MODEM_DOWNLINK_UNCONSUMED;
}

static void mw_gnss_almanac_next( void )
{
    if( mw_gnss_almanac_next_update.type == ALMANAC_TASK_TYPE_READ_STATUS )
    {
        mw_gnss_almanac_next_supervisor( mw_gnss_almanac_next_update.time_s );
    }
    else
    {
        mw_gnss_almanac_next_rp( mw_gnss_almanac_next_update.time_ms );
    }
}

static void mw_gnss_almanac_next_supervisor( uint32_t delay_s )
{
    GNSS_ALMANAC_TRACE_PRINTF_DEBUG( "mw_gnss_almanac_next_supervisor\n" );

    smodem_task task       = { 0 };
    task.id                = mw_gnss_almanac_task_obj.task_id;
    task.stack_id          = mw_gnss_almanac_task_obj.stack_id;
    task.priority          = TASK_BYPASS_DUTY_CYCLE; /* No TX for this service */
    uint32_t now_s         = smtc_modem_hal_get_time_in_s( );
    task.time_to_execute_s = now_s + delay_s;
    if( modem_supervisor_add_task( &task ) != TASK_VALID )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to add task in supervisor for next GNSS almanac status check\n" );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Added task in supervisor for next GNSS almanac status check at %u + %u s\n",
                                     now_s, delay_s );
    }
}

static void mw_gnss_almanac_next_rp( uint32_t delay_ms )
{
    rp_task_t rp_task = { 0 };

    GNSS_ALMANAC_TRACE_PRINTF_DEBUG( "mw_gnss_almanac_next_rp\n" );

    /* Set RP task priority depending on doppler solver status */
    rp_task.schedule_task_low_priority = false;
    if( ( mw_gnss_almanac_update_status.status_gps != SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_UNKNOWN ) &&
        ( mw_gnss_almanac_update_status.status_beidou != SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_UNKNOWN ) )
    {
        /* As soon as assisted scan is possible, set almanac to low priority */
        if( mw_gnss_almanac_update_status.last_doppler_solver_status == LR11XX_GNSS_DOPPLER_SOLVER_NO_ERROR )
        {
            if( mw_gnss_almanac_task_obj.nb_rp_abort < ALMANAC_UPDATE_ABORTED_BY_RP_THRESHOLD_FOR_LOW_PRIORITY )
            {
                /* GNSS scan will have higher priority from now */
                rp_task.schedule_task_low_priority = true;
            }
            else
            {
                /* If almanac demod has been aborted too much, take a chance to pass by keeping high priority over GNSS
                 * scan for next RP task */
                SMTC_MODEM_HAL_TRACE_PRINTF( "Almanac demod: too many aborts, keep high priority for next RP task\n" );
                mw_gnss_almanac_task_obj.nb_rp_abort = 0;
            }
        }
    }

    rp_task.hook_id                        = mw_gnss_almanac_task_obj.rp_hook_id;
    rp_task.state                          = RP_TASK_STATE_SCHEDULE; /* almanac demod time must not be delayed */
    uint32_t now_ms                        = RP_NOW_MS;
    rp_task.start_time_ms                  = now_ms + delay_ms;
    rp_task.duration_time_ms               = ALMANAC_UPDATE_SCAN_DURATION_S * 1000;
    rp_task.type                           = RP_TASK_TYPE_GNSS_SNIFF;
    rp_task.launch_task_callbacks          = gnss_almanac_rp_task_launch;
    rp_radio_params_t fake_rp_radio_params = { 0 };
    if( rp_task_enqueue( modem_get_rp( ), &rp_task, NULL, 0, &fake_rp_radio_params ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to enqueue RP task for next GNSS almanac demodulation\n" );
        SMTC_MODEM_HAL_PANIC( );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF(
            "Enqueued RP task (SCHEDULED %s priority) for next GNSS almanac demodulation at %u + %u ms\n",
            ( rp_task.schedule_task_low_priority ) ? "low" : "high", now_ms, delay_ms );
    }
}

static void gnss_almanac_rp_task_launch( void* context )
{
    lr11xx_status_t                   lr11xx_status;
    lr11xx_gnss_read_almanac_status_t almanac_status;
    lr11xx_gnss_time_t                lr11xx_time;

    SMTC_MODEM_HAL_TRACE_INFO( "gnss almanac task launched\n" );

    mw_gnss_almanac_task_obj.self_aborted            = false;
    mw_gnss_almanac_task_obj.pending_evt_update_done = false;

    /* WARNING: Read time at least once every 24h to let the LR11xx handle clock roll-over */
    lr11xx_status = lr11xx_gnss_read_time( modem_get_radio_ctx( ), &lr11xx_time );
    if( lr11xx_status != LR11XX_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "gnss_almanac_rp_task_launch: lr11xx_gnss_read_time failed\n" );
        RP_TASK_ABORT_AND_RETURN( );
    }
    if( lr11xx_time.error_code != LR11XX_GNSS_READ_TIME_STATUS_NO_ERROR )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "No time available: %s\n",
                                      smtc_gnss_read_time_error_code_enum2str( lr11xx_time.error_code ) );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "read_time (rc:%d, time:%u, accuracy:%u ms)\n", lr11xx_time.error_code,
                                     lr11xx_time.gps_time_s, lr11xx_time.time_accuracy / 1000 );
        /* Check if read_time() has not been called for too long */
        uint32_t now_s = smtc_modem_hal_get_time_in_s( );
        if( mw_gnss_almanac_task_obj.last_read_time_s != 0 )
        {
            GNSS_ALMANAC_TRACE_PRINTF_DEBUG( "Elapsed time since last read_time(): %us\n",
                                             now_s - mw_gnss_almanac_task_obj.last_read_time_s );
            if( ( now_s - mw_gnss_almanac_task_obj.last_read_time_s ) > ALMANAC_READ_TIME_THRESHOLD_MAX )
            {
                SMTC_MODEM_HAL_TRACE_WARNING( "Too long since last read_time: reset LR11xx time\n" );
                lr11xx_status = lr11xx_gnss_reset_time( modem_get_radio_ctx( ) );
                if( lr11xx_status != LR11XX_STATUS_OK )
                {
                    SMTC_MODEM_HAL_TRACE_ERROR( "gnss_almanac_rp_task_launch: lr11xx_gnss_reset_time failed\n" );
                    RP_TASK_ABORT_AND_RETURN( );
                }
            }
        }
        mw_gnss_almanac_task_obj.last_read_time_s = now_s;
    }

    /* Selected constellation(s) and get corresponding almanac status */
    lr11xx_status = lr11xx_gnss_set_constellations_to_use( modem_get_radio_ctx( ),
                                                           mw_gnss_almanac_task_obj.constellations_enabled );
    if( lr11xx_status != LR11XX_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "gnss_almanac_rp_task_launch: lr11xx_gnss_set_constellations_to_use failed\n" );
        RP_TASK_ABORT_AND_RETURN( );
    }
    lr11xx_status = lr11xx_gnss_read_almanac_status( modem_get_radio_ctx( ), &almanac_status );
    if( lr11xx_status != LR11XX_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "gnss_almanac_rp_task_launch: lr11xx_gnss_read_almanac_status failed\n" );
        RP_TASK_ABORT_AND_RETURN( );
    }
    update_current_almanac_status_context( &almanac_status ); /* Update current global context */
    GNSS_ALMANAC_TRACE_PRINTF_DEBUG( "Almanac status for GPS: %s\n",
                                     smtc_gnss_almanac_status_enum2str( almanac_status.status_gps ) );
    GNSS_ALMANAC_TRACE_PRINTF_DEBUG( "Almanac status for BDS: %s\n",
                                     smtc_gnss_almanac_status_enum2str( almanac_status.status_beidou ) );

    if( ( mw_gnss_almanac_next_update.type == ALMANAC_TASK_TYPE_READ_STATUS ) ||
        ( almanac_status.status_gps == LR11XX_GNSS_INTERNAL_ACCURACY_TOO_LOW ) ||
        ( almanac_status.status_beidou ==
          LR11XX_GNSS_INTERNAL_ACCURACY_TOO_LOW ) ) /* no scan programmed, or time accuracy is too low */
    {
        /* Configure next update */
        mw_gnss_almanac_next_update = select_next_update( &almanac_status );

        /* No scan to launch right now, stop current task */
        RP_TASK_ABORT_AND_RETURN( );
    }
    else /* launch scan for almanac */
    {
        /* Prepare for scan */
        lr11xx_status = lr11xx_system_set_dio_irq_params( modem_get_radio_ctx( ), LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE,
                                                          LR11XX_SYSTEM_IRQ_NONE );
        if( lr11xx_status != LR11XX_STATUS_OK )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "gnss_almanac_rp_task_launch: lr11xx_system_set_dio_irq_params failed\n" );
            RP_TASK_ABORT_AND_RETURN( );
        }

        /* Configure the board for almanac scan */
        geolocation_bsp_gnss_prescan_actions( );

        if( mw_radio_configure_for_scan( modem_get_radio_ctx( ) ) == false )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "gnss_almanac_rp_task_launch: mw_radio_configure_for_scan() failed\n" );
            RP_TASK_ABORT_AND_RETURN( );
        }

        SMTC_MODEM_HAL_TRACE_INFO( "=> launch almanac update from sat for %s\n",
                                   smtc_gnss_constellation_enum2str( mw_gnss_almanac_next_update.constellation ) );
        lr11xx_status = lr11xx_gnss_almanac_update_from_sat(
            modem_get_radio_ctx( ), mw_gnss_almanac_next_update.constellation, LR11XX_GNSS_OPTION_LOW_EFFORT );
        if( lr11xx_status != LR11XX_STATUS_OK )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "gnss_almanac_rp_task_launch: lr11xx_gnss_almanac_update_from_sat() failed\n" );

            /* Reset time for next almanac status check */
            memset( &mw_gnss_almanac_next_update, 0, sizeof( mw_gnss_almanac_next_update_t ) );
            mw_gnss_almanac_next_update.type   = ALMANAC_TASK_TYPE_READ_STATUS;
            mw_gnss_almanac_next_update.time_s = ALMANAC_STATUS_CHECK_PERIOD_DEFAULT_S;

            /* Abort current RP task (will be completed with RP_STATUS_TASK_ABORTED) */
            RP_TASK_ABORT_AND_RETURN( );
        }
        /* Update statistics */
        mw_gnss_almanac_update_status.stat_nb_update_from_sat_done += 1;
    }
}

static void gnss_almanac_rp_task_done( void* status )
{
    uint32_t         tcurrent_ms;
    rp_status_t      rp_status;
    mw_return_code_t mw_status;

    SMTC_MODEM_HAL_TRACE_INFO( "gnss_almanac_rp_task_done\n" );

    geolocation_bsp_gnss_postscan_actions( );

    rp_get_status( ( radio_planner_t* ) status, mw_gnss_almanac_task_obj.rp_hook_id, &tcurrent_ms, &rp_status );
    if( rp_status == RP_STATUS_TASK_ABORTED )
    {
        /* !!!! Do not perform any radio access here !!!! */

        if( mw_gnss_almanac_task_obj.self_aborted == false ) /* Aborted by RP */
        {
            mw_gnss_almanac_task_obj.nb_rp_abort += 1;
            /* reset next update time */
            mw_gnss_almanac_next_update.type    = ALMANAC_TASK_TYPE_READ_STATUS;
            mw_gnss_almanac_next_update.time_s  = ALMANAC_STATUS_CHECK_PERIOD_NO_TIME_S;
            mw_gnss_almanac_next_update.time_ms = 0;
            /* Update statistics */
            mw_gnss_almanac_update_status.stat_nb_aborted_by_rp += 1;
            SMTC_MODEM_HAL_TRACE_WARNING( "ALMANAC task aborted by RP\n" );
        }
        else
        {
            mw_gnss_almanac_task_obj.self_aborted = false;
            SMTC_MODEM_HAL_TRACE_PRINTF( "ALMANAC task self aborted\n" );
        }

        /* Notify application */
        send_event( SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE );

        /* Program next check/scan */
        mw_gnss_almanac_next( );
        return;
    }
    else if( rp_status == RP_STATUS_GNSS_SCAN_DONE )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "ALMANAC: RP_STATUS_GNSS_SCAN_DONE\n" );

        mw_status = gnss_almanac_task_done( );
        mw_radio_set_sleep( modem_get_radio_ctx( ) ); /* Set the radio back to sleep when all radio accesses are done */
        if( mw_status != MW_RC_OK )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "gnss_almanac_task_done Failed\n" );
        }

        /* Notify application */
        send_event( SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE );

        /* Program next check/scan */
        mw_gnss_almanac_next( );
    }
    else
    {
        /* Should not happen */
        SMTC_MODEM_HAL_TRACE_ERROR( "GNSS ALMANAC RP task - Unknown status %d\n", rp_status );
        SMTC_MODEM_HAL_PANIC( "Unexpected RP status" );
    }
}

static mw_return_code_t gnss_almanac_task_done( void )
{
    lr11xx_gnss_demod_status_t                       demod_status;
    lr11xx_gnss_demod_info_t                         demod_info;
    lr11xx_gnss_read_almanac_status_t                almanac_status;
    lr11xx_gnss_doppler_solver_result_t              doppler_solver_results;
    lr11xx_gnss_cumulative_timing_t                  cumulative_timing;
    uint32_t                                         power_consumption_nah;
    uint32_t                                         power_consumption_nwh;
    lr11xx_gnss_instantaneous_power_consumption_ua_t instantaneous_power_consumption_ua;
    uint8_t                                          nb_detected_svs;
    lr11xx_gnss_detected_satellite_t                 info_svs[2 * GNSS_NB_SVS_PER_CONSTELLATION_MAX];
    lr11xx_gnss_solver_assistance_position_t         aiding_pos;

    /* Get power consumption of last almanac demod */
    MW_RETURN_ON_FAILURE( lr11xx_gnss_read_cumulative_timing( modem_get_radio_ctx( ), &cumulative_timing ) ==
                          LR11XX_STATUS_OK );
    GNSS_ALMANAC_TRACE_PRINTF_DEBUG( "Cumulative timings: %u s\n", cumulative_timing.total / 32768 );
    /* Update statistics */
    mw_gnss_almanac_update_status.stat_cumulative_timings_s += ( cumulative_timing.total / 32768 );

    geolocation_bsp_gnss_get_consumption( &instantaneous_power_consumption_ua );
    lr11xx_gnss_compute_power_consumption( &cumulative_timing, &instantaneous_power_consumption_ua,
                                           &power_consumption_nah, &power_consumption_nwh );

    mw_gnss_almanac_update_status.power_consumption_nah = power_consumption_nah;
    GNSS_ALMANAC_TRACE_PRINTF_DEBUG( "Almanac: power consumption: %u nah\n", power_consumption_nah );

    /* Get almanac demodulation status */
    MW_RETURN_ON_FAILURE( lr11xx_gnss_read_demod_status( modem_get_radio_ctx( ), &demod_status, &demod_info ) ==
                          LR11XX_STATUS_OK );
    if( demod_status < 0 )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Almanac demodulation status: %s\n",
                                      smtc_gnss_almanac_demod_status_enum2str( demod_status ) );
    }
    else
    {
        /* Update statistics */
        mw_gnss_almanac_update_status.stat_nb_update_from_sat_success += 1;
        GNSS_ALMANAC_TRACE_PRINTF_DEBUG( "Almanac demodulation status: %s\n",
                                         smtc_gnss_almanac_demod_status_enum2str( demod_status ) );
    }

    /* Get detected SVs (for info) */
    MW_RETURN_ON_FAILURE( lr11xx_gnss_get_nb_detected_satellites( modem_get_radio_ctx( ), &nb_detected_svs ) ==
                          LR11XX_STATUS_OK );
    if( nb_detected_svs > 0 )
    {
        MW_RETURN_ON_FAILURE( lr11xx_gnss_get_detected_satellites( modem_get_radio_ctx( ), nb_detected_svs,
                                                                   info_svs ) == LR11XX_STATUS_OK );
    }
    GNSS_ALMANAC_TRACE_PRINTF_DEBUG( "ALMANAC SCAN DONE: %u SV detected:\n", nb_detected_svs );
    for( uint8_t i = 0; i < nb_detected_svs; i++ )
    {
        GNSS_ALMANAC_TRACE_PRINTF_DEBUG( "   SV_ID %u:\t%ddB\n", info_svs[i].satellite_id, info_svs[i].cnr );
    }

    /* Check if an aiding position could be computed by the doppler solver (indicates level of almanac update) */
    MW_RETURN_ON_FAILURE( lr11xx_gnss_read_doppler_solver_result( modem_get_radio_ctx( ), &doppler_solver_results ) ==
                          LR11XX_STATUS_OK );
    GNSS_ALMANAC_TRACE_PRINTF_DEBUG(
        "Doppler solver result after almanac demod: %s\n",
        smtc_gnss_doppler_solver_error_code_enum2str( doppler_solver_results.error_code ) );
    mw_gnss_almanac_update_status.last_doppler_solver_status = doppler_solver_results.error_code;

    /* Get current assistance position (could have been reset above) */
    MW_RETURN_ON_FAILURE( lr11xx_gnss_read_assistance_position( modem_get_radio_ctx( ), &aiding_pos ) ==
                          LR11XX_STATUS_OK );
    smtc_gnss_trace_print_position( "Almanac: Assistance Position: ", &aiding_pos );

    /* Select the constellations enabled for almanac demod */
    /* lr11xx_gnss_read_almanac_status() will ignore constellation not selected */
    MW_RETURN_ON_FAILURE( lr11xx_gnss_set_constellations_to_use( modem_get_radio_ctx( ),
                                                                 mw_gnss_almanac_task_obj.constellations_enabled ) ==
                          LR11XX_STATUS_OK );

    /* Get almanac status for selected constellation(s) */
    MW_RETURN_ON_FAILURE( lr11xx_gnss_read_almanac_status( modem_get_radio_ctx( ), &almanac_status ) ==
                          LR11XX_STATUS_OK );
    print_almanac_status( &almanac_status );

    /* Update current global context */
    update_current_almanac_status_context( &almanac_status );

    /* In case of bad sky conditions for a particular constellation, postpone update for this constellation */
    if( ( demod_status == LR11XX_GNSS_NO_SAT_FOUND ) || ( demod_status == LR11XX_GNSS_NO_ACTIVATED_SAT_IN_SV_LIST ) ||
        ( demod_status == LR11XX_GNSS_NO_DEMOD_BDS_ALMANAC_SV31_43 ) )
    {
        lr11xx_gnss_constellation_t last_scan_constellation = mw_gnss_almanac_next_update.constellation;
        if( ( last_scan_constellation == LR11XX_GNSS_GPS_MASK ) &&
            ( mw_gnss_almanac_task_obj.gps_update_postpone_time_s == 0 ) )
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "ALMANAC: postpone GPS update due to bad sky detected\n" );
            mw_gnss_almanac_task_obj.gps_update_postpone_time_s = smtc_modem_hal_get_time_in_s( );
        }
        else if( ( last_scan_constellation == LR11XX_GNSS_BEIDOU_MASK ) &&
                 ( mw_gnss_almanac_task_obj.beidou_update_postpone_time_s == 0 ) )
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "ALMANAC: postpone BEIDOU update due to bad sky detected\n" );
            mw_gnss_almanac_task_obj.beidou_update_postpone_time_s = smtc_modem_hal_get_time_in_s( );
        }
    }

    /* Set time for next update */
    mw_gnss_almanac_next_update = select_next_update( &almanac_status );

    return MW_RC_OK;
}

static mw_gnss_almanac_next_update_t select_next_update( const lr11xx_gnss_read_almanac_status_t* almanac_status )
{
    bool                          skip_gps            = false;
    bool                          skip_beidou         = false;
    bool                          gps_needs_update    = false;
    bool                          beidou_needs_update = false;
    mw_gnss_almanac_next_update_t next_update         = { 0 };

    /* Check if any constellation update has been postponed for bad conditions */
    uint32_t now = smtc_modem_hal_get_time_in_s( );
    if( mw_gnss_almanac_task_obj.gps_update_postpone_time_s > 0 )
    {
        if( ( now - mw_gnss_almanac_task_obj.gps_update_postpone_time_s ) >= ALMANAC_STATUS_CHECK_PERIOD_BAD_SKY_S )
        {
            /* reset postpone time */
            mw_gnss_almanac_task_obj.gps_update_postpone_time_s = 0;
        }
        else
        {
            /* Ignore GPS constellation for now */
            skip_gps = true;
        }
    }
    if( mw_gnss_almanac_task_obj.beidou_update_postpone_time_s > 0 )
    {
        if( ( now - mw_gnss_almanac_task_obj.beidou_update_postpone_time_s ) >= ALMANAC_STATUS_CHECK_PERIOD_BAD_SKY_S )
        {
            /* reset postpone time */
            mw_gnss_almanac_task_obj.beidou_update_postpone_time_s = 0;
        }
        else
        {
            /* Ignore BEIDOU constellation for now */
            skip_beidou = true;
        }
    }
    if( ( skip_gps == true ) && ( skip_beidou == true ) )
    {
        uint32_t min_elapsed_time = now - MIN( mw_gnss_almanac_task_obj.gps_update_postpone_time_s,
                                               mw_gnss_almanac_task_obj.beidou_update_postpone_time_s );
        if( min_elapsed_time <= ALMANAC_STATUS_CHECK_PERIOD_BAD_SKY_S ) /* Sanity check */
        {
            next_update.time_s = ALMANAC_STATUS_CHECK_PERIOD_BAD_SKY_S - min_elapsed_time;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_WARNING(
                "select_next_update: min_elapsed_time > ALMANAC_STATUS_CHECK_PERIOD_BAD_SKY_S\n" );
            next_update.time_s = ALMANAC_STATUS_CHECK_PERIOD_BAD_SKY_S;
        }
        next_update.type = ALMANAC_TASK_TYPE_READ_STATUS;
        return next_update;
    }

    /* Check if GPS should be updated */
    if( ( almanac_status->status_gps == LR11XX_GNSS_AT_LEAST_ONE_SAT_MUST_BE_UPDATED ) &&
        ( almanac_status->nb_sat_gps_to_update > ALMANAC_UPDATE_NB_SV_TO_BE_UPDATED_THRESHOLD ) &&
        ( skip_gps == false ) )
    {
        gps_needs_update = true;
    }
    /* Check if BEIDOU should be updated */
    if( ( almanac_status->status_beidou == LR11XX_GNSS_AT_LEAST_ONE_SAT_MUST_BE_UPDATED ) &&
        ( almanac_status->nb_sat_beidou_to_update > ALMANAC_UPDATE_NB_SV_TO_BE_UPDATED_THRESHOLD ) &&
        ( skip_beidou == false ) )
    {
        beidou_needs_update = true;
    }
    /* A constellation is considered fully updated when only ALMANAC_UPDATE_NB_SV_TO_BE_UPDATED_THRESHOLD remain to be
     * updated */

    /* Configure next update */
    if( ( gps_needs_update == true ) && ( beidou_needs_update == true ) )
    {
        if( almanac_status->next_gps_time_sat_to_update <= almanac_status->next_beidou_time_sat_to_update )
        {
            next_update.type          = ALMANAC_TASK_TYPE_SCAN;
            next_update.time_ms       = almanac_status->next_gps_time_sat_to_update;
            next_update.constellation = LR11XX_GNSS_GPS_MASK;
        }
        else
        {
            next_update.type          = ALMANAC_TASK_TYPE_SCAN;
            next_update.time_ms       = almanac_status->next_beidou_time_sat_to_update;
            next_update.constellation = LR11XX_GNSS_BEIDOU_MASK;
        }
    }
    else if( gps_needs_update == true )
    {
        next_update.type          = ALMANAC_TASK_TYPE_SCAN;
        next_update.time_ms       = almanac_status->next_gps_time_sat_to_update;
        next_update.constellation = LR11XX_GNSS_GPS_MASK;
    }
    else if( beidou_needs_update == true )
    {
        next_update.type          = ALMANAC_TASK_TYPE_SCAN;
        next_update.time_ms       = almanac_status->next_beidou_time_sat_to_update;
        next_update.constellation = LR11XX_GNSS_BEIDOU_MASK;
    }
    else
    {
        /* Program next read almanac status later */
        next_update.type = ALMANAC_TASK_TYPE_READ_STATUS;
        if( ( almanac_status->status_gps == LR11XX_GNSS_NO_SAT_TO_UPDATE ) &&
            ( almanac_status->status_beidou == LR11XX_GNSS_NO_SAT_TO_UPDATE ) )
        {
            next_update.time_s = ALMANAC_STATUS_CHECK_PERIOD_DEFAULT_S;
            GNSS_ALMANAC_TRACE_PRINTF_DEBUG( "=> No almanac update required\n" );
        }
        else if( ( almanac_status->status_gps == LR11XX_GNSS_NO_TIME_SET ) ||
                 ( almanac_status->status_beidou == LR11XX_GNSS_NO_TIME_SET ) )
        {
            next_update.time_s = ALMANAC_STATUS_CHECK_PERIOD_NO_TIME_S;
            GNSS_ALMANAC_TRACE_PRINTF_DEBUG( "=> No time set\n" );
        }
        else if( ( almanac_status->status_gps == LR11XX_GNSS_INTERNAL_ACCURACY_TOO_LOW ) ||
                 ( almanac_status->status_beidou == LR11XX_GNSS_INTERNAL_ACCURACY_TOO_LOW ) )
        {
            next_update.time_s = ALMANAC_STATUS_CHECK_PERIOD_TIME_ACCURACY_S;
            GNSS_ALMANAC_TRACE_PRINTF_DEBUG( "=> bad almanac status\n" );
        }
        else
        {
            next_update.time_s = ALMANAC_STATUS_CHECK_PERIOD_DEFAULT_S;
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "Check almanac status again in %ds\n", next_update.time_s );
    }

    /* Set time for next almanac scan */
    if( next_update.type == ALMANAC_TASK_TYPE_SCAN )
    {
        /* Set warmup time if there is enough time */
        if( next_update.time_ms > ALMANAC_UPDATE_FROM_SAT_WARMUP_DURATION_MS )
        {
            next_update.time_ms -= ALMANAC_UPDATE_FROM_SAT_WARMUP_DURATION_MS;

            /* Add 1 second margin for GPS if possible */
            if( next_update.constellation == LR11XX_GNSS_GPS_MASK )
            {
                /* for GPS:
                    - if next GPS demod time is > 2.3s, launch 2.3s in advance
                    - if next GPS demod time is between 2.3s and 1.3s, launch right now */
                if( next_update.time_ms > ALMANAC_UPDATE_FROM_SAT_GPS_MARGIN_DURATION_MS )
                {
                    next_update.time_ms -= ALMANAC_UPDATE_FROM_SAT_GPS_MARGIN_DURATION_MS;
                }
                else
                {
                    next_update.time_ms = 0; /* try right now to increase demodulation success chances */
                }
            }
        }
        else
        {
            /* for GPS & Beidou:
                - if next demod time is < 1.3s, launch right now */
            next_update.time_ms = 0;
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "=> Next almanac update for %s in %u ms\n",
                                     smtc_gnss_constellation_enum2str( next_update.constellation ),
                                     next_update.time_ms );
    }

    return next_update;
}

static void send_event( smtc_modem_event_type_t event )
{
    if( event == SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE )
    {
        mw_gnss_almanac_task_obj.pending_evt_update_done = true;
    }
    increment_asynchronous_msgnumber( event, 0, mw_gnss_almanac_task_obj.stack_id );
}

static void update_current_almanac_status_context( const lr11xx_gnss_read_almanac_status_t* almanac_status )
{
    /* GPS */
    if( almanac_status->status_gps >= 0 )
    {
        if( almanac_status->status_gps == LR11XX_GNSS_NO_SAT_TO_UPDATE )
        {
            mw_gnss_almanac_update_status.status_gps = SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_COMPLETED;
            mw_gnss_almanac_update_status.remaining_sv_to_be_updated_gps = 0;
        }
        else
        {
            mw_gnss_almanac_update_status.status_gps = SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_NOT_COMPLETED;
            mw_gnss_almanac_update_status.remaining_sv_to_be_updated_gps = almanac_status->nb_sat_gps_to_update;
        }
        mw_gnss_almanac_update_status.total_sv_to_be_updated_gps =
            count_set_bits( almanac_status->sat_id_gps_activated );
        if( mw_gnss_almanac_update_status.remaining_sv_to_be_updated_gps <=
            mw_gnss_almanac_update_status.total_sv_to_be_updated_gps )
        {
            if( mw_gnss_almanac_update_status.total_sv_to_be_updated_gps > 0 )
            {
                mw_gnss_almanac_update_status.update_progress_gps =
                    ( uint16_t )( 100 - ( mw_gnss_almanac_update_status.remaining_sv_to_be_updated_gps * 100 /
                                          mw_gnss_almanac_update_status.total_sv_to_be_updated_gps ) );
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_ERROR( " total_sv_to_be_updated_gps is equal to 0\n" );
            }
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR(
                "remaining_sv_to_be_updated_gps should be <= total_sv_to_be_updated_gps (%u - %u)\n",
                mw_gnss_almanac_update_status.remaining_sv_to_be_updated_gps,
                mw_gnss_almanac_update_status.total_sv_to_be_updated_gps );
        }
    }

    /* BEIDOU */
    if( almanac_status->status_beidou >= 0 )
    {
        if( almanac_status->status_beidou == LR11XX_GNSS_NO_SAT_TO_UPDATE )
        {
            mw_gnss_almanac_update_status.status_beidou = SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_COMPLETED;
            mw_gnss_almanac_update_status.remaining_sv_to_be_updated_beidou = 0;
        }
        else
        {
            mw_gnss_almanac_update_status.status_beidou = SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_NOT_COMPLETED;
            mw_gnss_almanac_update_status.remaining_sv_to_be_updated_beidou = almanac_status->nb_sat_beidou_to_update;
        }
        mw_gnss_almanac_update_status.total_sv_to_be_updated_beidou =
            count_set_bits( almanac_status->sat_id_beidou_activated[0] & 0xFFFFFFE0 ) +
            count_set_bits( almanac_status->sat_id_beidou_activated[1] &
                            0x03FFFFFF ); /* Ignore SV 1-5 and 59-63 (geostationary) */
        if( mw_gnss_almanac_update_status.remaining_sv_to_be_updated_beidou <=
            mw_gnss_almanac_update_status.total_sv_to_be_updated_beidou )
        {
            if( mw_gnss_almanac_update_status.total_sv_to_be_updated_beidou > 0 )
            {
                mw_gnss_almanac_update_status.update_progress_beidou =
                    ( uint16_t )( 100 - ( mw_gnss_almanac_update_status.remaining_sv_to_be_updated_beidou * 100 /
                                          mw_gnss_almanac_update_status.total_sv_to_be_updated_beidou ) );
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_ERROR( " total_sv_to_be_updated_beidou is equal to 0\n" );
            }
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR(
                "remaining_sv_to_be_updated_beidou should be <= total_sv_to_be_updated_beidou (%u - %u)\n",
                mw_gnss_almanac_update_status.remaining_sv_to_be_updated_beidou,
                mw_gnss_almanac_update_status.total_sv_to_be_updated_beidou );
        }
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- HELPER FUNCTIONS DEFINITION ---------------------------------------------
 */

static void print_almanac_status( const lr11xx_gnss_read_almanac_status_t* almanac_status )
{
    uint8_t nb_to_be_updated_gps    = 0;
    uint8_t nb_to_be_updated_beidou = 0;

    /* GPS status */
    if( almanac_status->status_gps < 0 )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Almanac status for GPS: %s\n",
                                      smtc_gnss_almanac_status_enum2str( almanac_status->status_gps ) );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Almanac status for GPS:\n" );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  status: %s\n",
                                     smtc_gnss_almanac_status_enum2str( almanac_status->status_gps ) );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  nb sat to update: %u/%u\n", almanac_status->nb_sat_gps_to_update,
                                     count_set_bits( almanac_status->sat_id_gps_activated ) );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  time sat to update: %u ms\n", almanac_status->next_gps_time_sat_to_update );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  list sat to update: 0x%08X\n", almanac_status->sat_id_gps_to_update );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  list sat activated: 0x%08X\n", almanac_status->sat_id_gps_activated );
        /* GPS update status: 1 -> 32*/
        SMTC_MODEM_HAL_TRACE_PRINTF( "  sv_id :" );
        for( uint8_t satellite = 0; satellite < 32; ++satellite )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( " %2u", satellite + 1 );
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n" );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  status:" );
        for( uint8_t satellite = 0; satellite < 32; ++satellite )
        {
            uint32_t status = ( ( almanac_status->sat_id_gps_to_update ) >> satellite ) & 1;
            SMTC_MODEM_HAL_TRACE_PRINTF( "  %u", status );
            if( status == 1 )
            {
                nb_to_be_updated_gps += 1;
            }
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n" );
    }

    /* BEIDOU status */
    if( almanac_status->status_beidou < 0 )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Almanac status for BEIDOU: %s\n",
                                      smtc_gnss_almanac_status_enum2str( almanac_status->status_beidou ) );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Almanac status for BEIDOU:\n" );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  status: %s\n",
                                     smtc_gnss_almanac_status_enum2str( almanac_status->status_beidou ) );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  nb sat to update: %u/%u\n", almanac_status->nb_sat_beidou_to_update,
                                     count_set_bits( almanac_status->sat_id_beidou_activated[0] & 0xFFFFFFE0 ) +
                                         count_set_bits( almanac_status->sat_id_beidou_activated[1] & 0x03FFFFFF ) );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  time sat to update: %u ms\n", almanac_status->next_beidou_time_sat_to_update );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  next subframe ID start: %u\n", almanac_status->next_beidou_subframe_id_start );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  sat ID to update in sub 4: %u\n",
                                     almanac_status->next_beidou_sat_id_to_update_in_sub_4 );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  sat ID to update in sub 5: %u\n",
                                     almanac_status->next_beidou_sat_id_to_update_in_sub_5 );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  next Am ID: %u\n", almanac_status->next_am_id );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  list sat to update[0]: 0x%08X\n", almanac_status->sat_id_beidou_to_update[0] );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  list sat to update[1]: 0x%08X\n", almanac_status->sat_id_beidou_to_update[1] );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  list sat activated[0]: 0x%08X\n", almanac_status->sat_id_beidou_activated[0] );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  list sat activated[1]: 0x%08X\n", almanac_status->sat_id_beidou_activated[1] );
        /* Beidou update status: 1 -> 32*/
        SMTC_MODEM_HAL_TRACE_PRINTF( "  sv_id :" );
        for( uint8_t satellite = 0; satellite < 32; ++satellite )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( " %2u", satellite + 1 );
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n" );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  status:" );
        for( uint8_t satellite = 0; satellite < 32; ++satellite )
        {
            uint32_t status = ( ( almanac_status->sat_id_beidou_to_update[0] ) >> satellite ) & 1;
            SMTC_MODEM_HAL_TRACE_PRINTF( "  %u", status );
            if( status == 1 )
            {
                nb_to_be_updated_beidou += 1;
            }
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n" );
        /* Beidou update status: 33 -> 63*/
        SMTC_MODEM_HAL_TRACE_PRINTF( "  sv_id :" );
        for( uint8_t satellite = 0; satellite < 32; ++satellite )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( " %2u", satellite + 33 );
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n" );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  status:" );
        for( uint8_t satellite = 0; satellite < 32; ++satellite )
        {
            uint32_t status = ( ( almanac_status->sat_id_beidou_to_update[1] ) >> satellite ) & 1;
            SMTC_MODEM_HAL_TRACE_PRINTF( "  %u", status );
            if( status == 1 )
            {
                nb_to_be_updated_beidou += 1;
            }
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n" );
    }
}

static void trace_print_event_data_almanac_update(
    const smtc_modem_almanac_demodulation_event_data_almanac_update_t* data )
{
    if( data != NULL )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "ALMANAC_DEMOD_UPDATE info:\n" );
        SMTC_MODEM_HAL_TRACE_PRINTF( "-- status GPS: " );
        switch( data->status_gps )
        {
        case SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_UNKNOWN:
            SMTC_MODEM_HAL_TRACE_PRINTF( "UNKNOWN\n" );
            break;
        case SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_COMPLETED:
            SMTC_MODEM_HAL_TRACE_PRINTF( "COMPLETED\n" );
            break;
        case SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_NOT_COMPLETED:
            SMTC_MODEM_HAL_TRACE_PRINTF( "NOT COMPLETED\n" );
            break;
        default:
            break;
        }
        if( data->status_gps != SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_UNKNOWN )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "   update progress: %u%%\n", data->update_progress_gps );
        }

        SMTC_MODEM_HAL_TRACE_PRINTF( "-- status BEIDOU: " );
        switch( data->status_beidou )
        {
        case SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_UNKNOWN:
            SMTC_MODEM_HAL_TRACE_PRINTF( "UNKNOWN\n" );
            break;
        case SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_COMPLETED:
            SMTC_MODEM_HAL_TRACE_PRINTF( "COMPLETED\n" );
            break;
        case SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_NOT_COMPLETED:
            SMTC_MODEM_HAL_TRACE_PRINTF( "NOT COMPLETED\n" );
            break;
        default:
            break;
        }
        if( data->status_beidou != SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_UNKNOWN )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "   update progress: %u%%\n", data->update_progress_beidou );
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "-- statistics:\n" );
        SMTC_MODEM_HAL_TRACE_PRINTF( "   nb update from sat done: %u\n", data->stat_nb_update_from_sat_done );
        SMTC_MODEM_HAL_TRACE_PRINTF( "   nb update from sat success: %u\n", data->stat_nb_update_from_sat_success );
        SMTC_MODEM_HAL_TRACE_PRINTF( "   nb aborted by RP: %u\n", data->stat_nb_aborted_by_rp );
        SMTC_MODEM_HAL_TRACE_PRINTF( "   cumulative timing: %u s\n", data->stat_cumulative_timings_s );
    }
}

static uint32_t count_set_bits( uint32_t bitmask )
{
    uint32_t count = 0;

    while( bitmask )
    {
        bitmask &= ( bitmask - 1 );
        count++;
    }

    return count;
}

/* --- EOF ------------------------------------------------------------------ */
