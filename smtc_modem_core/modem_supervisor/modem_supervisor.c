/*!
 * \file      modem_supervisor.c
 *
 * \brief     soft modem task scheduler
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
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "modem_supervisor.h"

#include "smtc_modem_hal_dbg_trace.h"
#include "lorawan_certification.h"

#include "smtc_modem_hal.h"
#include "modem_context.h"

#include "lorawan_api.h"
#include "dm_downlink.h"
#include "smtc_modem_api.h"

#include "radio_planner.h"
#include "smtc_modem_test_api.h"
#include "smtc_secure_element.h"

// services
#if defined( ADD_SMTC_FILE_UPLOAD )
#include "file_upload.h"
#endif  // ADD_SMTC_FILE_UPLOAD

#if defined( ADD_SMTC_STREAM )
#include "stream.h"
#endif  // ADD_SMTC_STREAM

#include "alc_sync.h"
#include "smtc_clock_sync.h"

#if defined( LR1110_MODEM_E )
#include "pool_mem.h"
#include "host_irq.h"
#include "fragmented_data_block.h"
#endif  // LR1110_MODEM_E

/*
 *-----------------------------------------------------------------------------------
 * --- PUBLIC MACROS ----------------------------------------------------------------
 */

/*!
 * \brief Returns the minimum value between a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval minValue Minimum value
 */
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )

/**
 * @brief Math Abs macro
 */
#define ABS( N ) ( ( N < 0 ) ? ( -N ) : ( N ) )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

#if !defined( LR1110_MODEM_E )
static lr1mac_states_t LpState = LWPSTATE_IDLE;
static stask_manager   task_manager;
static bool            is_pending_dm_status_payload_periodic = false;
static bool            is_pending_dm_status_payload_now      = false;
static bool            is_first_dm_after_join                = true;
static bool            send_task_update_needed               = false;

static void ( *app_callback )( void ) = NULL;

#if defined( ADD_SMTC_ALC_SYNC )
static alc_sync_ctx_t*   alc_sync_context   = NULL;
static clock_sync_ctx_t* clock_sync_context = NULL;
#endif  // ADD_SMTC_ALC_SYNC

#if defined( ADD_SMTC_STREAM )
static rose_t* ROSE = NULL;
#endif  // ADD_SMTC_STREAM
#if defined( ADD_SMTC_FILE_UPLOAD )

static file_upload_t* file_upload_context = NULL;
#endif  // ADD_SMTC_FILE_UPLOAD

// Used for LoRaWAN Certification
static uint8_t user_payload_length           = 10;
static uint8_t user_payload[242]             = { 0 };
static uint8_t user_port                     = 1;
static bool    certification_data_is_pending = false;

// Used for class B
static bool class_b_bit = false;
#else

struct
{
    lr1mac_states_t LpState;
    stask_manager   task_manager;
    bool            is_pending_dm_status_payload_periodic;
    bool            is_pending_dm_status_payload_now;
    bool            is_first_dm_after_join;
    bool            send_task_update_needed;
    void ( *app_callback )( void );

#if defined( ADD_SMTC_ALC_SYNC )
    alc_sync_ctx_t*   alc_sync_context;
    clock_sync_ctx_t* clock_sync_context;
#endif  // ADD_SMTC_ALC_SYNC

#if defined( ADD_SMTC_STREAM )
    rose_t*           ROSE;
#endif  // ADD_SMTC_STREAM

#if defined( ADD_SMTC_FILE_UPLOAD )
    file_upload_t*    file_upload_context;
#endif  // ADD_SMTC_FILE_UPLOAD

    // Used for LoRaWAN Certification
    uint8_t user_payload_length;
    uint8_t user_payload[242];
    uint8_t user_port;
    bool    certification_data_is_pending;

    // Used for class B
    bool class_b_bit;
} modem_supervisor_context;

// clang-format off
#define LpState                                 modem_supervisor_context.LpState
#define task_manager                            modem_supervisor_context.task_manager
#define is_pending_dm_status_payload_periodic   modem_supervisor_context.is_pending_dm_status_payload_periodic
#define is_pending_dm_status_payload_now        modem_supervisor_context.is_pending_dm_status_payload_now
#define is_first_dm_after_join                  modem_supervisor_context.is_first_dm_after_join
#define send_task_update_needed                 modem_supervisor_context.send_task_update_needed
#define app_callback                            modem_supervisor_context.app_callback

#if defined( ADD_SMTC_ALC_SYNC )
#define alc_sync_context                        modem_supervisor_context.alc_sync_context
#define clock_sync_context                      modem_supervisor_context.clock_sync_context
#endif  // ADD_SMTC_ALC_SYNC

#if defined( ADD_SMTC_STREAM )
#define ROSE                                    modem_supervisor_context.ROSE
#endif  // ADD_SMTC_STREAM

#if defined( ADD_SMTC_FILE_UPLOAD )
#define file_upload_context                     modem_supervisor_context.file_upload_context
#endif  // ADD_SMTC_FILE_UPLOAD

// Used for LoRaWAN Certification
#define user_payload_length                     modem_supervisor_context.user_payload_length
#define user_payload                            modem_supervisor_context.user_payload
#define user_port                               modem_supervisor_context.user_port
#define certification_data_is_pending           modem_supervisor_context.certification_data_is_pending

// Used for class B
#define class_b_bit                             modem_supervisor_context.class_b_bit

// clang-format on
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief function call when the certification mode is enabled
 *
 */
static void certification_event_handler( void );

static void check_class_b_to_generate_event( void );
static void backoff_mobile_static( void );
static void send_task_update( uint8_t event_type );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void modem_supervisor_init( void ( *callback )( void ), radio_planner_t* rp,
                            smtc_modem_services_t* smtc_modem_services_ctx )
{
    LpState                               = LWPSTATE_IDLE;
    is_pending_dm_status_payload_periodic = true;  // Set to true to send at least the first DM after Join
    is_pending_dm_status_payload_now      = false;
    is_first_dm_after_join                = true;
    send_task_update_needed               = false;
    app_callback                          = callback;

#if defined( ADD_SMTC_ALC_SYNC )
    alc_sync_context   = &( smtc_modem_services_ctx->alc_sync_ctx );
    clock_sync_context = &( smtc_modem_services_ctx->clock_sync_ctx );
#endif  // ADD_SMTC_ALC_SYNC

#if defined( ADD_SMTC_STREAM )
    ROSE = &( smtc_modem_services_ctx->stream_ROSE_ctx );
#endif  // ADD_SMTC_STREAM

#if defined( ADD_SMTC_FILE_UPLOAD )
    file_upload_context = &( smtc_modem_services_ctx->file_upload_ctx );
#endif  // ADD_SMTC_FILE_UPLOAD

    // Used for LoRaWAN Certification
    user_payload_length = 10;
    memset( user_payload, 0, sizeof( user_payload ) );
    user_port                     = 1;
    certification_data_is_pending = false;
    class_b_bit                   = false;

    lorawan_api_init( rp );
    lorawan_api_dr_strategy_set( STATIC_ADR_MODE );
    lorawan_api_join_status_clear( );

    modem_context_init( );
    modem_event_init( );
    modem_supervisor_init_task( );
    modem_load_context( );

#if defined( ADD_SMTC_ALC_SYNC )
    alc_sync_init( alc_sync_context );
    clock_sync_init( clock_sync_context, alc_sync_context );
#endif  // ADD_SMTC_ALC_SYNC

#if defined( LR1110_MODEM_E ) && defined( ADD_SMTC_PATCH_UPDATE )
    frag_init( );
#endif  // LR1110_MODEM_E && ADD_SMTC_PATCH_UPDATE

    set_modem_start_time_s( smtc_modem_hal_get_time_in_s( ) );

    // save used rp in context in case of future needs (suspend/resume)
    modem_context_set_modem_rp( rp );

    // if a crash happened before set crash bit in status
    if( smtc_modem_hal_get_crashlog_status( ) == true )
    {
        set_modem_status_reset_after_crash( true );
    }
    else
    {
        set_modem_status_reset_after_crash( false );
    }

    // Event EVENT_RESET must be done at the end of init !!
    increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_RESET, 0 );
}

void modem_supervisor_init_task( void )
{
    for( int i = 0; i < NUMBER_OF_TASKS; i++ )
    {
        task_manager.modem_task[i].priority = TASK_FINISH;
        task_manager.modem_task[i].id       = ( task_id_t ) i;
    }
    task_manager.next_task_id = IDLE_TASK;
}

eTask_priority modem_supervisor_get_task_priority( task_id_t id )
{
    if( id < NUMBER_OF_TASKS )
    {
        return ( task_manager.modem_task[id].priority );
    }
    else
    {
        return ( TASK_FINISH );
    }
}

eTask_valid_t modem_supervisor_remove_task( task_id_t id )
{
    if( id < NUMBER_OF_TASKS )
    {
        task_manager.modem_task[id].priority = TASK_FINISH;
        return TASK_VALID;
    }
    SMTC_MODEM_HAL_TRACE_ERROR( "modem_supervisor_remove_task id = %d unknown\n", id );
    return TASK_NOT_VALID;
}

eTask_valid_t modem_supervisor_add_task( smodem_task* task )
{
    // the modem supervisor always accept a new task.
    // in case of a previous task is already enqueue , the new task remove the old one.
    // as soon as a task has been elected by the modem supervisor , the task is managed by the stack itself and a new
    // task could be added inside the modem supervisor.
    if( task->id < NUMBER_OF_TASKS )
    {
        task_manager.modem_task[task->id].time_to_execute_s = task->time_to_execute_s;
        task_manager.modem_task[task->id].priority          = task->priority;
        task_manager.modem_task[task->id].fPort             = task->fPort;
        task_manager.modem_task[task->id].fPort_present     = task->fPort_present;
        task_manager.modem_task[task->id].dataIn            = task->dataIn;
        task_manager.modem_task[task->id].sizeIn            = task->sizeIn;
        task_manager.modem_task[task->id].PacketType        = task->PacketType;

        return TASK_VALID;
    }
    SMTC_MODEM_HAL_TRACE_ERROR( "modem_supervisor_add_task id = %d unknown\n", task->id );
    return TASK_NOT_VALID;
}

void modem_supervisor_launch_task( task_id_t id )
{
    status_lorawan_t send_status = ERRORLORAWAN;
    switch( id )
    {
    case JOIN_TASK:
        // SMTC_MODEM_HAL_TRACE_INFO( "JOIN_TASK\n" );
        if( get_join_state( ) == MODEM_JOINED )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "DEVICE ALREADY JOINED or TRY TO JOIN \n" );
            task_manager.next_task_id = IDLE_TASK;
        }
        else if( get_join_state( ) == MODEM_JOIN_ONGOING )
        {
            lorawan_api_join( smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
        }
        break;

    case SEND_TASK_EXTENDED_1:
    case SEND_TASK_EXTENDED_2:
    case SEND_TASK: {
        send_status = lorawan_api_payload_send(
            task_manager.modem_task[id].fPort, task_manager.modem_task[id].fPort_present,
            task_manager.modem_task[id].dataIn, task_manager.modem_task[id].sizeIn,
            ( task_manager.modem_task[id].PacketType == TX_CONFIRMED ) ? CONF_DATA_UP : UNCONF_DATA_UP,
            smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );

        if( send_status == OKLORAWAN )
        {
            send_task_update_needed = true;
            SMTC_MODEM_HAL_TRACE_PRINTF(
                " User Tx LORa %s %d \n",
                ( task_manager.modem_task[id].fPort_present == true ) ? "on FPort" : "No FPort",
                task_manager.modem_task[id].fPort );
        }
        else
        {
            send_task_update_needed = false;
            SMTC_MODEM_HAL_TRACE_WARNING( "The payload can't be send! internal code: %x\n", send_status );
        }
        break;
    }

    case DM_TASK:
        if( get_join_state( ) == MODEM_JOINED )
        {
            if( lorawan_api_modem_certification_is_enabled( ) == true )
            {
                break;
            }
            if( get_modem_dm_interval_second( ) == 0 )
            {
                break;
            }
            uint8_t  max_payload = lorawan_api_next_max_payload_length_get( );
            uint8_t  payload[242];
            uint8_t  payload_length;
            uint32_t info_bitfield_periodic;
            uint32_t info_bitfield_periodictmp;

            if( ( is_first_dm_after_join == true ) && ( is_pending_dm_status_payload_periodic == true ) )
            {
                info_bitfield_periodictmp = modem_get_dm_info_bitfield_periodic( );
                SMTC_MODEM_HAL_TRACE_PRINTF( " info bit field = %x\n", info_bitfield_periodictmp );
                info_bitfield_periodic =
                    info_bitfield_periodictmp +
                    ( ( ( info_bitfield_periodictmp & ( 1 << DM_INFO_RSTCOUNT ) ) == 0 ) ? ( 1 << DM_INFO_RSTCOUNT )
                                                                                         : 0 ) +
                    ( ( ( info_bitfield_periodictmp & ( 1 << DM_INFO_SESSION ) ) == 0 ) ? ( 1 << DM_INFO_SESSION )
                                                                                        : 0 ) +
                    ( ( ( info_bitfield_periodictmp & ( 1 << DM_INFO_FIRMWARE ) ) == 0 ) ? ( 1 << DM_INFO_FIRMWARE )
                                                                                         : 0 );

                modem_set_dm_info_bitfield_periodic( info_bitfield_periodic );
                SMTC_MODEM_HAL_TRACE_PRINTF( " info bit field = %x\n", info_bitfield_periodic );
                is_pending_dm_status_payload_periodic =
                    dm_status_payload( payload, &payload_length, max_payload, DM_INFO_PERIODIC );

                send_status =
                    lorawan_api_payload_send( get_modem_dm_port( ), true, payload, payload_length, UNCONF_DATA_UP,
                                              smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );

                if( send_status == OKLORAWAN )
                {
                    if( is_pending_dm_status_payload_periodic == false )
                    {
                        is_first_dm_after_join = false;
                    }
                    modem_set_dm_info_bitfield_periodic( info_bitfield_periodictmp );
                    SMTC_MODEM_HAL_TRACE_ARRAY( "payload DM ", payload, payload_length );
                    SMTC_MODEM_HAL_TRACE_PRINTF( " on Port %d\n", get_modem_dm_port( ) );
                }
                else
                {
                    SMTC_MODEM_HAL_TRACE_WARNING( "Periodic DM can't be send! internal code: %x\n", send_status );
                }
            }
            else
            {
                if( ( get_modem_dm_interval_second( ) > 0 ) && ( modem_get_dm_info_bitfield_periodic( ) > 0 ) )
                {
                    is_pending_dm_status_payload_periodic =
                        dm_status_payload( payload, &payload_length, max_payload, DM_INFO_PERIODIC );

                    send_status =
                        lorawan_api_payload_send( get_modem_dm_port( ), true, payload, payload_length, UNCONF_DATA_UP,
                                                  smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );

                    if( send_status == OKLORAWAN )
                    {
                        SMTC_MODEM_HAL_TRACE_ARRAY( "DM ", payload, payload_length );
                        SMTC_MODEM_HAL_TRACE_PRINTF( " on Port %d\n", get_modem_dm_port( ) );
                    }
                    else
                    {
                        SMTC_MODEM_HAL_TRACE_WARNING( "Periodic DM can't be send! internal code: %x\n", send_status );
                    }
                }
            }
        }
        break;
    case DM_TASK_NOW:
        if( get_join_state( ) == MODEM_JOINED )
        {
            uint8_t max_payload = lorawan_api_next_max_payload_length_get( );
            uint8_t payload[242];
            uint8_t payload_length;
            is_pending_dm_status_payload_now = dm_status_payload( payload, &payload_length, max_payload, DM_INFO_NOW );

            send_status = lorawan_api_payload_send( get_modem_dm_port( ), true, payload, payload_length, UNCONF_DATA_UP,
                                                    smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
            if( send_status == OKLORAWAN )
            {
                SMTC_MODEM_HAL_TRACE_ARRAY( "DM Req ", payload, payload_length );
                SMTC_MODEM_HAL_TRACE_PRINTF( " on Port %d\n", get_modem_dm_port( ) );
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_WARNING( "Requested DM can't be send! internal code: %x\n", send_status );
            }
        }
        break;
    case CRASH_LOG_TASK:
        if( get_join_state( ) == MODEM_JOINED )
        {
            uint8_t payload[CRASH_LOG_SIZE + 1];
            uint8_t payload_length = CRASH_LOG_SIZE + 1;
            // first set the dm byte corresponding to crashlog
            payload[0] = DM_INFO_CRASHLOG;
            // get the stored crashlog
            smtc_modem_hal_restore_crashlog( &payload[1] );

            SMTC_MODEM_HAL_TRACE_ARRAY( "DM Req CRASH LOG ", payload, payload_length );
            uint8_t max_payload = lorawan_api_next_max_payload_length_get( );
            payload_length      = ( payload_length > max_payload ) ? max_payload : payload_length;
            send_status = lorawan_api_payload_send( get_modem_dm_port( ), true, payload, payload_length, UNCONF_DATA_UP,
                                                    smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
            if( send_status == OKLORAWAN )
            {
                // crashlog will be sent => set availability to false
                smtc_modem_hal_set_crashlog_status( false );
            }
        }
        break;
#if defined( ADD_SMTC_FILE_UPLOAD )
    case FILE_UPLOAD_TASK: {
        int32_t file_upload_chunk_size         = 0;
        uint8_t file_upload_chunk_payload[242] = { 0 };
        if( get_join_state( ) != MODEM_JOINED )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "DEVICE NOT JOIN \n" );
            break;
        }
        else if( modem_get_upload_state( ) != MODEM_UPLOAD_ON_GOING )

        {
            SMTC_MODEM_HAL_TRACE_ERROR( "FileUpload not init \n" );
            break;
        }
        uint32_t max_payload_size = lorawan_api_next_max_payload_length_get( );
        file_upload_chunk_size =
            file_upload_get_fragment( file_upload_context, file_upload_chunk_payload,
                                      ( max_payload_size > 100 ) ? 100 : max_payload_size, lorawan_api_fcnt_up_get( ) );
        if( file_upload_chunk_size > 0 )
        {
            send_status =
                lorawan_api_payload_send( get_modem_dm_port( ), true, file_upload_chunk_payload, file_upload_chunk_size,
                                          UNCONF_DATA_UP, smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
        }
        else
        {
            // something prevents fragment to be constructed (max payload size < 11 due to mac answers in fopts and
            // shall be uplinked first)
            send_status = lorawan_api_payload_send( 0, false, NULL, 0, UNCONF_DATA_UP,
                                                    smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
        }

        break;
    }
#endif  // ADD_SMTC_FILE_UPLOAD

#if defined( ADD_SMTC_STREAM )
    case STREAM_TASK: {
        uint8_t              stream_payload[242] = { 0 };
        uint8_t              fragment_size;
        uint32_t             frame_cnt;
        stream_return_code_t stream_rc;
        uint8_t              tx_buff_offset = 0;

        // SMTC_MODEM_HAL_TRACE_MSG( "Supervisor launch STREAM_TASK\n" );

        if( get_join_state( ) != MODEM_JOINED )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "DEVICE NOT JOINED \n" );
            break;
        }
        else if( modem_get_stream_state( ) != MODEM_STREAM_INIT )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "Streaming not initialized \n" );
            break;
        }

        // check first if stream runs on dm port and if yes add dm code
        if( get_modem_dm_port( ) == modem_get_stream_port( ) )
        {
            stream_payload[tx_buff_offset] = DM_INFO_STREAM;
            tx_buff_offset++;
        }

        // XXX Check if a streaming session is already active
        fragment_size = lorawan_api_next_max_payload_length_get( ) - tx_buff_offset;
        frame_cnt     = lorawan_api_fcnt_up_get( );
        stream_rc     = stream_get_fragment( ROSE, &stream_payload[tx_buff_offset], frame_cnt, &fragment_size );
        // TODO Is this enough to ensure we send everything?
        if( stream_rc == STREAM_OK && fragment_size > 0 )
        {
            send_status = lorawan_api_payload_send( task_manager.modem_task[id].fPort, true, stream_payload,
                                                    fragment_size + tx_buff_offset, UNCONF_DATA_UP,
                                                    smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
        }
        else
        {
            // Insufficient data or streaming done
            set_modem_status_streaming( false );
            SMTC_MODEM_HAL_TRACE_WARNING( "Stream get fragment FAILED\n" );
        }
        break;
    }
#endif  // ADD_SMTC_STREAM
    case MUTE_TASK: {
        if( get_modem_muted( ) == MODEM_TEMPORARY_MUTE )
        {
            dm_set_number_of_days_mute( dm_get_number_of_days_mute( ) - 1 );
        }
        break;
    }
    case RETRIEVE_DL_TASK: {
        dm_dl_opportunities_config_t retrieve;
        get_dm_retrieve_pending_dl( &retrieve );
        if( retrieve.up_count > 0 )
        {
            send_status = lorawan_api_payload_send( 0, false, NULL, 0, UNCONF_DATA_UP,
                                                    smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
        }
        break;
    }
#if defined( ADD_SMTC_ALC_SYNC )
    case CLOCK_SYNC_TIME_REQ_TASK: {
        if( lorawan_api_modem_certification_is_enabled( ) == true )
        {
            break;
        }
        if( get_join_state( ) == MODEM_JOINED )
        {
            clock_sync_request( clock_sync_context );
        }

        break;
    }
    case ALC_SYNC_ANS_TASK: {
        if( lorawan_api_modem_certification_is_enabled( ) == true )
        {
            break;
        }
        uint8_t  max_payload = lorawan_api_next_max_payload_length_get( );
        uint8_t  tx_buffer_out[ALC_SYNC_TX_PAYLOAD_SIZE_MAX + 1];  // +1 is the DM_INFO_ALCSYNC ID
        uint8_t  tx_buffer_length_out = 0;
        uint32_t target_send_time     = smtc_modem_hal_get_time_in_s( ) + 2;
        uint8_t  tx_buff_offset       = 0;

        // check first if alc_sync runs on dm port and if yes add dm code
        if( get_modem_dm_port( ) == clock_sync_get_alcsync_port( clock_sync_context ) )
        {
            tx_buffer_out[tx_buff_offset] = DM_INFO_ALCSYNC;
            tx_buff_offset++;
        }

        // use target send time with both local compensation and previous alcsync compensation to create payload
        alc_sync_create_uplink_payload( alc_sync_context,
                                        target_send_time + smtc_modem_hal_get_time_compensation_in_s( ) +
                                            alc_sync_get_time_correction_second( alc_sync_context ),
                                        true, true, max_payload - tx_buff_offset, &tx_buffer_out[tx_buff_offset],
                                        &tx_buffer_length_out );

        // compute final size according to previous offset
        tx_buffer_length_out += tx_buff_offset;

        if( tx_buffer_length_out > 0 )
        {
            send_status = lorawan_api_payload_send_at_time( clock_sync_get_alcsync_port( clock_sync_context ), true,
                                                            tx_buffer_out, tx_buffer_length_out, UNCONF_DATA_UP,
                                                            target_send_time * 1000 );
            // reset alcsync reception bool
            alc_sync_context->is_sync_dl_received = false;
        }
        break;
    }
#endif  // ADD_SMTC_ALC_SYNC
    case LINK_CHECK_REQ_TASK:
        lorawan_api_send_stack_cid_req( LINK_CHECK_REQ );
        break;
    case DEVICE_TIME_REQ_TASK:
        SMTC_MODEM_HAL_TRACE_WARNING( "DEVICE TIME REQUEST\n" );
        lorawan_api_send_stack_cid_req( DEVICE_TIME_REQ );
        break;
    case PING_SLOT_INFO_REQ_TASK:
        SMTC_MODEM_HAL_TRACE_WARNING( "PING SLOT REQUEST\n" );
        lorawan_api_send_stack_cid_req( PING_SLOT_INFO_REQ );
        break;
#if defined( LR1110_MODEM_E ) && defined( ADD_SMTC_PATCH_UPDATE )
    case FRAG_TASK: {
        uint8_t max_payload = lorawan_api_next_max_payload_length_get( );

        uint8_t tx_buffer_out[FRAG_UPLINK_LENGTH_MAX];
        uint8_t tx_buffer_length_out = FRAG_UPLINK_LENGTH_MAX;

        dm_frag_uplink_payload( max_payload, tx_buffer_out, &tx_buffer_length_out );

        SMTC_MODEM_HAL_TRACE_ARRAY( "FRAG_TASK", tx_buffer_out, tx_buffer_length_out );
        if( tx_buffer_length_out > 0 )
        {
            lorawan_api_payload_send( get_modem_frag_port( ), true, tx_buffer_out, tx_buffer_length_out, UNCONF_DATA_UP,
                                      smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
        }
        break;
    }
#endif  // LR1110_MODEM_E && ADD_SMTC_PATCH_UPDATE
    case USER_TASK: {
        send_status = lorawan_api_payload_send(
            get_modem_dm_port( ), true, task_manager.modem_task[id].dataIn, task_manager.modem_task[id].sizeIn,
            task_manager.modem_task[id].PacketType, smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
        break;
    }
#if defined( LR1110_MODEM_E ) && defined( _MODEM_E_GNSS_ENABLE )
    case DM_ALM_DBG_ANS: {
        uint8_t max_payload = lorawan_api_next_max_payload_length_get( );
        uint8_t payload[242];
        uint8_t payload_length;

        dm_alm_dbg_uplink_payload( max_payload, payload, &payload_length );

        SMTC_MODEM_HAL_TRACE_ARRAY( "DM_ALM_DBG_ANS", payload, payload_length );
        if( payload_length > 0 )
        {
            send_status = lorawan_api_payload_send( get_modem_dm_port( ), true, payload, payload_length, UNCONF_DATA_UP,
                                                    smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
        }
        break;
    }
#endif  // LR1110_MODEM_E && _MODEM_E_GNSS_ENABLE

    default:
        break;
    }

    if( send_status == OKLORAWAN )
    {
        decrement_dm_retrieve_pending_dl( );
    }

    modem_supervisor_remove_task( id );
}

void modem_supervisor_update_task( task_id_t id )
{
    // modem_supervisor_update_downlink_frame( );
    switch( id )
    {
    case JOIN_TASK:
        if( ( get_join_state( ) == MODEM_JOIN_ONGOING ) && ( lorawan_api_isjoined( ) == JOINED ) )
        {
            set_modem_status_joining( false );
            set_modem_status_modem_joined( true );
            // as soon as modem is joined, modem send has to sent a dm report every DM_PERIOD_AFTER_JOIN
            is_first_dm_after_join                = true;
            is_pending_dm_status_payload_periodic = true;
            modem_supervisor_add_task_dm_status( DM_PERIOD_AFTER_JOIN );

            // reset dm tag number to prevent using wrong id
            modem_context_reset_dm_tag_number( );

#if defined( ADD_SMTC_ALC_SYNC )
            // If clock sync service activated => initiate a new request
            if( clock_sync_is_enabled( clock_sync_context ) == true )
            {
                modem_supervisor_add_task_clock_sync_time_req( DM_PERIOD_AFTER_JOIN + 10 +
                                                               smtc_modem_hal_get_random_nb_in_range( 0, 5 ) );
                clock_sync_reset_nb_time_req( clock_sync_context );
            }
#endif  // ADD_SMTC_ALC_SYNC
            if( smtc_modem_hal_get_crashlog_status( ) == true )
            {
                modem_supervisor_add_task_crash_log( DM_PERIOD_AFTER_JOIN + 30 +
                                                     smtc_modem_hal_get_random_nb_in_range( 5, 10 ) );
            }
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_JOINED, 0 );
            lorawan_api_class_c_start( );
        }
        else if( get_join_state( ) == MODEM_JOIN_ONGOING )
        {
            set_modem_status_joining( false );
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_JOINFAIL, 0 );
            modem_supervisor_add_task_join( );  // relaunch join only if no leave cmd
        }
        else if( ( lorawan_api_isjoined( ) == JOINED ) )  //
        {
            set_modem_status_joining( false );
            lorawan_api_join_status_clear( );  // => manage case leave cmd between end of join and join succeed
        }

        // Disable the duty cycle in stack case of disabled by host
        if( modem_get_duty_cycle_disabled_by_host( ) == true )
        {
            lorawan_api_duty_cycle_enable_set( SMTC_DTC_FULL_DISABLED );
        }
        break;
    case DM_TASK:

        if( get_modem_dm_interval_second( ) > 0 )
        {
            if( is_first_dm_after_join == true )
            {
                modem_supervisor_add_task_dm_status( smtc_modem_hal_get_random_nb_in_range( 10, 15 ) );
            }
            else
            {
                modem_supervisor_add_task_dm_status( get_modem_dm_interval_second( ) );
            }
        }
        break;
    case DM_TASK_NOW:
        if( is_pending_dm_status_payload_now )
        {
            modem_supervisor_add_task_dm_status_now( );
        }
        break;

    case SEND_TASK:
        send_task_update( SMTC_MODEM_EVENT_TXDONE );
        break;

    case SEND_TASK_EXTENDED_1:
        ( modem_get_extended_callback( 1 ) )( );
        break;

    case SEND_TASK_EXTENDED_2:
        ( modem_get_extended_callback( 2 ) )( );
        break;
#if defined( ADD_SMTC_FILE_UPLOAD )
    case FILE_UPLOAD_TASK: {
        modem_upload_state_t modem_upload_state = modem_get_upload_state( );
        if( modem_upload_state == MODEM_UPLOAD_ON_GOING )
        {
            if( ( file_upload_is_data_remaining( file_upload_context ) == true ) )
            {
                // There is still upload that need to be sent => add a new task
                modem_supervisor_add_task_file_upload( file_upload_get_average_delay_in_s( file_upload_context ) );
            }
            else
            {
                // Nothing left to be sent => abort upload and generate event
                SMTC_MODEM_HAL_TRACE_WARNING( "File upload ended without server confirmation \n" );
                increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_UPLOADDONE, 0x00 );
                set_modem_status_file_upload( false );
                modem_set_upload_state( MODEM_UPLOAD_FINISHED );
            }
        }
        break;
    }
#endif  // ADD_SMTC_FILE_UPLOAD

#if defined( ADD_SMTC_STREAM )
    case STREAM_TASK: {
        // SMTC_MODEM_HAL_TRACE_MSG( "Supervisor update STREAM_TASK\n" );
        if( stream_data_pending( ROSE ) )
        {
            modem_supervisor_add_task_stream( );
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "Streaming DONE\n" );
            set_modem_status_streaming( false );
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_STREAMDONE, 0 );
        }
        break;
    }
#endif  // ADD_SMTC_STREAM
    case MUTE_TASK: {
        if( get_modem_muted( ) == MODEM_TEMPORARY_MUTE )
        {
            modem_supervisor_add_task_modem_mute( );
        }
        break;
    }
    case RETRIEVE_DL_TASK: {
        dm_dl_opportunities_config_t retrieve;
        get_dm_retrieve_pending_dl( &retrieve );
        if( retrieve.up_count > 0 )
        {
            modem_supervisor_add_task_retrieve_dl( retrieve.up_delay );
        }

        break;
    }
#if defined( ADD_SMTC_ALC_SYNC )
    case CLOCK_SYNC_TIME_REQ_TASK: {
        clock_sync_callback( clock_sync_context, 0 );  // TODO change 0

        if( clock_sync_context->sync_service_type == CLOCK_SYNC_ALC )
        {
            // Answer with time sync not received, create a new downlink opportunities
            if( clock_sync_is_time_valid( clock_sync_context ) == false )
            {
                dm_dl_opportunities_config_t retrieve;
                get_dm_retrieve_pending_dl( &retrieve );
                if( retrieve.up_count == 0 )
                {
                    retrieve.up_delay = smtc_modem_hal_get_random_nb_in_range( 3, 8 );
                    set_dm_retrieve_pending_dl( 1, retrieve.up_delay );
                    modem_supervisor_add_task_retrieve_dl( retrieve.up_delay );
                }
            }
        }

        break;
    }
    case ALC_SYNC_ANS_TASK:
        if( alc_sync_get_nb_transmission( alc_sync_context ) > 0 )
        {
            modem_supervisor_add_task_alc_sync_ans( smtc_modem_hal_get_random_nb_in_range( 128, 150 ) );
        }
        break;
#endif  // ADD_SMTC_ALC_SYNC

    case LINK_CHECK_REQ_TASK: {
        uint8_t margin;
        uint8_t gw_cnt;

        smtc_modem_event_link_check_status_t link_check_status = SMTC_MODEM_EVENT_LINK_CHECK_NOT_RECEIVED;

        if( lorawan_api_get_link_check_ans( &margin, &gw_cnt ) == OKLORAWAN )
        {
            link_check_status = SMTC_MODEM_EVENT_LINK_CHECK_RECEIVED;
        }
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_LINK_CHECK, link_check_status );

        break;
    }
    case DEVICE_TIME_REQ_TASK: {
        smtc_modem_event_time_status_t time_updated_status = SMTC_MODEM_EVENT_TIME_NOT_VALID;

        if( lorawan_api_get_device_time_req_status( ) == OKLORAWAN )
        {
            time_updated_status = SMTC_MODEM_EVENT_TIME_VALID;
        }
        else
        {
            if( lorawan_api_is_time_valid( ) == true )
            {
                time_updated_status = SMTC_MODEM_EVENT_TIME_VALID_BUT_NOT_SYNC;
            }
        }
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TIME, time_updated_status );
        break;
    }
    case PING_SLOT_INFO_REQ_TASK: {
        if( lorawan_api_get_ping_slot_info_req_status( ) == OKLORAWAN )
        {
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO,
                                              SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_ANSWERED );
        }
        else
        {
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO,
                                              SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_NOT_ANSWERED );
        }
        break;
    }
    default:
        break;
    }
}

uint32_t modem_supervisor_scheduler( void )
{
    // Check first if stack state is idle
    if( lorawan_api_state_get( ) != LWPSTATE_IDLE )
    {
        return ( CALL_LR1MAC_PERIOD_MS );
    }

    if( task_manager.next_task_id != IDLE_TASK )
    {
        modem_supervisor_update_task( task_manager.next_task_id );
        task_manager.next_task_id = IDLE_TASK;
    }

    eTask_priority next_task_priority = TASK_FINISH;
    int32_t        next_task_time     = MODEM_MAX_TIME;

    // Find the highest priority task in the past
    for( task_id_t i = 0; i < NUMBER_OF_TASKS; i++ )
    {
        if( task_manager.modem_task[i].priority != TASK_FINISH )
        {
            int32_t next_task_time_tmp =
                ( int32_t )( task_manager.modem_task[i].time_to_execute_s - smtc_modem_hal_get_time_in_s( ) );

            if( ( next_task_time_tmp <= 0 ) && ( task_manager.modem_task[i].priority < next_task_priority ) )
            {
                next_task_priority        = task_manager.modem_task[i].priority;
                next_task_time            = next_task_time_tmp;
                task_manager.next_task_id = ( task_id_t ) i;
            }
        }
    }

    // No task in the past was found, select the least in the future for wake up
    if( next_task_priority == TASK_FINISH )
    {
        for( task_id_t i = 0; i < NUMBER_OF_TASKS; i++ )
        {
            if( task_manager.modem_task[i].priority != TASK_FINISH )
            {
                int32_t next_task_time_tmp =
                    ( int32_t )( task_manager.modem_task[i].time_to_execute_s - smtc_modem_hal_get_time_in_s( ) );
                if( next_task_time_tmp < next_task_time )
                {
                    next_task_time            = next_task_time_tmp;
                    task_manager.next_task_id = ( task_id_t ) i;
                }
            }
        }
    }

    if( next_task_time > 0 )
    {
        task_manager.sleep_duration = next_task_time;
        task_manager.next_task_id   = IDLE_TASK;
        return ( next_task_time * 1000 );
    }
    else
    {
        modem_supervisor_launch_task( task_manager.next_task_id );
        return 0;
    }
}

uint32_t modem_supervisor_engine( void )
{
    // manage reset requested by the host
    if( get_modem_reset_requested( ) == true )
    {
        smtc_modem_hal_disable_modem_irq( );

        smtc_modem_hal_reset_mcu( );
    }

    uint32_t alarm                 = modem_get_user_alarm( );
    int32_t  user_alarm_in_seconds = MODEM_MAX_ALARM_S;
    // manage the user alarm
    if( alarm != 0 )
    {
        user_alarm_in_seconds = ( int32_t )( alarm - smtc_modem_hal_get_time_in_s( ) );

        if( user_alarm_in_seconds <= 0 )
        {
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_ALARM, 0 );
            modem_set_user_alarm( 0 );
            user_alarm_in_seconds = MODEM_MAX_ALARM_S;
            if( *app_callback != NULL )
            {
                app_callback( );
            }
        }
    }

    // case Lorawan stack already in use
    LpState = lorawan_api_state_get( );

    if( ( LpState != LWPSTATE_IDLE ) && ( LpState != LWPSTATE_ERROR ) && ( LpState != LWPSTATE_INVALID ) )
    {
        LpState = lorawan_api_process( );
        return ( CALL_LR1MAC_PERIOD_MS );
    }

    backoff_mobile_static( );
    check_class_b_to_generate_event( );

    // Call modem_supervisor_update_task to update asynchronous messages number
    if( task_manager.next_task_id != IDLE_TASK )
    {
        modem_supervisor_update_task( task_manager.next_task_id );
        task_manager.next_task_id = IDLE_TASK;
    }

    uint8_t msgnumber_tmp;
    do
    {
        if( get_asynchronous_msgnumber( ) > 0 )
        {
            if( lorawan_api_modem_certification_is_enabled( ) == true )
            {
                certification_event_handler( );
            }
            else if( *app_callback != NULL )
            {
                app_callback( );
            }
        }
        msgnumber_tmp = get_asynchronous_msgnumber( );
    } while( msgnumber_tmp < get_asynchronous_msgnumber( ) );

    uint32_t sleep_time = 0;
    int32_t  dtc_ms     = lorawan_api_next_free_duty_cycle_ms_get( );
    if( dtc_ms > 0 )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Duty Cycle, remaining time: %dms\n", dtc_ms );
        sleep_time = ( uint32_t ) dtc_ms;
    }
    // Don't call the supervisor if modem is suspend
    else if( ( get_modem_suspend( ) == MODEM_NOT_SUSPEND ) && ( get_modem_muted( ) != MODEM_INFINITE_MUTE ) )
    {
        // you reach this step only if lorawan stack is in idle mode and TOA is available (no dtc blocking)
        sleep_time = modem_supervisor_scheduler( );
    }
    else
    {
        // Set Max time (7FFFFFFF>>10 ie /1024 to be compatible with *1000)
        sleep_time = 0x7FFFFFFE;
    }

    alarm = modem_get_user_alarm( );
    if( alarm != 0 )
    {
        user_alarm_in_seconds = ( int32_t )( alarm - smtc_modem_hal_get_time_in_s( ) );
        if( user_alarm_in_seconds <= 0 )
        {
            user_alarm_in_seconds = 0;
        }
    }

    sleep_time = MIN( sleep_time, ( uint32_t ) user_alarm_in_seconds * 1000 );
    // SMTC_MODEM_HAL_TRACE_INFO( "Next task in %d\n", sleep_time );
    return ( sleep_time );
}

void check_class_b_to_generate_event( void )
{
    bool class_b_bit_stack = lorawan_api_get_class_b_status( );

    if( class_b_bit != class_b_bit_stack )
    {
        class_b_bit = class_b_bit_stack;

        dm_dl_opportunities_config_t retrieve;
        get_dm_retrieve_pending_dl( &retrieve );
        if( retrieve.up_count == 0 )
        {
            set_dm_retrieve_pending_dl( 1, retrieve.up_delay );
        }
        modem_supervisor_add_task_retrieve_dl( 1 );

        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_CLASS_B_STATUS, class_b_bit );
    }
}

void backoff_mobile_static( void )
{
    uint16_t                 nb_usr_adr_mobile_timeout;
    uint16_t                 nb_current_usr_adr_mobile_timeout;
    smtc_modem_adr_profile_t usr_adr_profile;

    usr_adr_profile                   = get_modem_adr_profile( );
    nb_usr_adr_mobile_timeout         = modem_get_adr_mobile_timeout_config( );
    nb_current_usr_adr_mobile_timeout = lorawan_api_get_current_no_rx_packet_in_mobile_mode_cnt( );

    // if nb_usr_adr_mobile_timeout = 0 the feature switch from mobile to static mode isn't activated
    if( ( ( nb_current_usr_adr_mobile_timeout > nb_usr_adr_mobile_timeout ) &&
          ( usr_adr_profile != SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED ) && ( nb_usr_adr_mobile_timeout != 0 ) ) ||
        ( ( usr_adr_profile != SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED ) &&
          ( lorawan_api_dr_strategy_get( ) == STATIC_ADR_MODE ) ) )
    {
        set_modem_adr_profile( SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED, NULL, 0 );
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TIMEOUT_ADR_CHANGED, 0 );
    }
    if( ( usr_adr_profile == SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED ) &&
        ( modem_available_new_link_adr_request( ) == true ) )
    {
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_NEW_LINK_ADR, 0 );
    }
}

uint8_t modem_supervisor_update_downlink_frame( uint8_t* data, uint8_t data_length, lr1mac_down_metadata_t* metadata,
                                                bool ack_requested )
{
    modem_downlink_msg_t dwnframe;

    // Class C Downlink Confirmed can trig a dl retrieve task to acked the frame
    if( ack_requested == true )
    {
        dm_dl_opportunities_config_t retrieve;
        get_dm_retrieve_pending_dl( &retrieve );
        if( retrieve.up_count == 0 )
        {
            set_dm_retrieve_pending_dl( 1, retrieve.up_delay );
        }
        modem_supervisor_add_task_retrieve_dl( 1 );
    }

    set_modem_downlink_frame( data, data_length, metadata );
    get_modem_downlink_frame( &dwnframe );
    if( metadata->rx_window == RECEIVE_ON_RXBEACON )
    {
        return 1;
    }
    else if( dwnframe.port == get_modem_dm_port( ) )
    {
        dm_downlink( dwnframe.data, dwnframe.length );
    }
#if defined( ADD_SMTC_ALC_SYNC )
    else if( dwnframe.port == clock_sync_get_alcsync_port( clock_sync_context ) )
    {
        if( ( metadata->rx_window == RECEIVE_ON_RX1 ) || ( metadata->rx_window == RECEIVE_ON_RX2 ) ||
            ( metadata->rx_window == RECEIVE_ON_RXB ) || ( metadata->rx_window == RECEIVE_ON_RXC ) )
        {
            uint8_t alc_sync_status = alc_sync_parser( alc_sync_context, dwnframe.data, dwnframe.length );

            if( ( ( alc_sync_status >> ALC_SYNC_APP_TIME_ANS ) & 0x1 ) == 1 )
            {
                // an alcsync dl with time was received => update flag
                alc_sync_context->is_sync_dl_received = true;

                increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TIME, SMTC_MODEM_EVENT_TIME_VALID );

                // Remove all alc sync task.
                modem_supervisor_remove_task_clock_sync( );

                if( clock_sync_is_enabled( clock_sync_context ) == true )
                {
                    int32_t tmp_rand = 0;

                    uint32_t tmp_delay = MIN( clock_sync_get_interval_second( clock_sync_context ),
                                              clock_sync_get_time_left_connection_lost( clock_sync_context ) );
                    do
                    {
                        tmp_rand = smtc_modem_hal_get_signed_random_nb_in_range( -30, 30 );
                    } while( ( tmp_rand < 0 ) && ( ABS( tmp_rand ) > tmp_delay ) );

                    modem_supervisor_add_task_clock_sync_time_req( tmp_delay + tmp_rand );
                }
            }

            if( alc_sync_status & ( ~( 1 << ALC_SYNC_APP_TIME_ANS ) ) )
            {
                if( ( ( alc_sync_status >> ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ ) & 0x1 ) == 1 )
                {
                    if( clock_sync_is_enabled( clock_sync_context ) == true )
                    {
                        int32_t tmp_rand = 0;

                        // If periodic time request is configured, add task to handle it
                        uint32_t tmp_delay = MIN( clock_sync_get_interval_second( clock_sync_context ),
                                                  clock_sync_get_time_left_connection_lost( clock_sync_context ) );
                        do
                        {
                            tmp_rand = smtc_modem_hal_get_signed_random_nb_in_range( -30, 30 );
                        } while( ( tmp_rand < 0 ) && ( ABS( tmp_rand ) > tmp_delay ) );

                        modem_supervisor_add_task_clock_sync_time_req( tmp_delay + tmp_rand );
                    }
                }
                // When a request requiring an answer is requested, add task to handle it
                modem_supervisor_add_task_alc_sync_ans( 1 );
            }
        }
    }
#endif  // ADD_SMTC_ALC_SYNC

#if defined( LR1110_MODEM_E ) && defined( ADD_SMTC_PATCH_UPDATE )
    else if( dwnframe.port == get_modem_frag_port( ) )
    {
        int8_t frag_status = frag_parser( dwnframe.data, dwnframe.length );
        if( frag_status & FRAG_CMD_ERROR )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "ERROR: Failed to parse frag message\n" );
        }
        else if( frag_status != 0x00 )
        {
            // An answer to a request, or a request is required, add a task for it
            modem_supervisor_add_task_frag( 1 );
        }
        else
        {
            // Nothing to do
        }
    }
#endif  // LR1110_MODEM_E && ADD_SMTC_PATCH_UPDATE
    else
    {
        return 1;
    }
    return 0;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */
static void certification_event_handler( void )
{
    smtc_modem_event_t current_event;
    uint8_t            event_pending_count;
    uint8_t            stack_id = 0;

    uint8_t rx_payload[255] = { 0 };
    uint8_t rx_payload_size = 0;

    uint8_t rx_port               = 0;
    uint8_t new_certification_cmd = 0;

    bool is_request_tx = false;

    // Continue to read modem event until all event has been processed
    do
    {
        // Read modem event
        smtc_modem_get_event( &current_event, &event_pending_count );

        SMTC_MODEM_HAL_TRACE_WARNING( "Event 0x%x\n", current_event.event_type );
        switch( current_event.event_type )
        {
        case SMTC_MODEM_EVENT_RESET:
            smtc_secure_element_restore_context( );
            // Schedule a Join LoRaWAN network
            smtc_modem_join_network( stack_id );
            // Enable certification mode
            smtc_modem_set_certification_mode( stack_id, true );
            break;

        case SMTC_MODEM_EVENT_ALARM:
            if( lorawan_api_certification_is_cw_running( ) == true )
            {
                lorawan_api_certification_cw_set_as_stopped( );
                smtc_modem_test_stop( );
            }
            else
            {
                // If the stack is not idle, set a new alarm in 1s
                if( lorawan_api_state_get( ) != LWPSTATE_IDLE )
                {
                    smtc_modem_alarm_start_timer( 1 );
                }
                else
                {
                    smtc_modem_alarm_start_timer( lorawan_api_certification_get_ul_periodicity( ) );
                    is_request_tx = true;
                }
            }

            break;

        case SMTC_MODEM_EVENT_JOINED: {
            SMTC_MODEM_HAL_TRACE_INFO( "Modem is joined, program an alarm\n" );
            smtc_modem_alarm_start_timer( smtc_modem_hal_get_random_nb_in_range( 3, 5 ) );

            // Disable the duty cycle when LoRaWAN certification mode is enabled
            smtc_modem_test_duty_cycle_app_activate( false );

            // Enable push class B beacon to user fifo
            lorawan_api_set_status_push_network_downlink_to_user( true );
        }
        break;

        case SMTC_MODEM_EVENT_TXDONE:
            switch( current_event.event_data.txdone.status )
            {
            case SMTC_MODEM_EVENT_TXDONE_SENT:
                // Certification Mode, repeat DlCounter with the same value
                if( ( lorawan_api_certification_is_enabled( ) == true ) && ( new_certification_cmd == 0 ) )
                {
                    new_certification_cmd = 2;
                }
                break;
            case SMTC_MODEM_EVENT_TXDONE_CONFIRMED:
                // ACK downlink must be count for Certification Mode
                if( lorawan_api_certification_is_enabled( ) == true )
                {
                    new_certification_cmd = 1;
                }
                break;

            case SMTC_MODEM_EVENT_TXDONE_NOT_SENT:
                user_payload_length = 0;
                break;

            default:
                break;
            }
            break;

        case SMTC_MODEM_EVENT_DOWNDATA: {
            rx_payload_size = ( uint8_t ) current_event.event_data.downdata.length;
            memcpy( rx_payload, current_event.event_data.downdata.data, rx_payload_size );
            SMTC_MODEM_HAL_TRACE_ARRAY( "DOWNDATA", rx_payload, rx_payload_size );

            if( current_event.event_data.downdata.window != SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXBEACON )
            {
                rx_port = current_event.event_data.downdata.fport;
                SMTC_MODEM_HAL_TRACE_PRINTF( "on port %u\n", rx_port );
                if( ( rx_port == 224 ) || ( lorawan_api_certification_is_enabled( ) == true ) )
                {
                    new_certification_cmd = 1;
                }
            }
            break;
        }
        break;
#if defined( ADD_SMTC_FILE_UPLOAD )
        case SMTC_MODEM_EVENT_UPLOADDONE:
            break;
#endif  // ADD_SMTC_FILE_UPLOAD
        case SMTC_MODEM_EVENT_SETCONF:
            break;

        case SMTC_MODEM_EVENT_MUTE:
            break;

#if defined( ADD_SMTC_STREAM )
        case SMTC_MODEM_EVENT_STREAMDONE:
            break;
#endif  // ADD_SMTC_STREAM

        case SMTC_MODEM_EVENT_JOINFAIL:
            SMTC_MODEM_HAL_TRACE_WARNING( "Join failed \n" );
            break;

        case SMTC_MODEM_EVENT_TIMEOUT_ADR_CHANGED:
            break;

        case SMTC_MODEM_EVENT_NEW_LINK_ADR:
            break;

        case SMTC_MODEM_EVENT_TIME:
        case SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO:
            if( lorawan_api_certification_get_requested_class( ) == LORAWAN_CERTIFICATION_CLASS_B )
            {
                if( ( lorawan_api_get_ping_slot_info_req_status( ) == OKLORAWAN ) &&
#if defined( ADD_SMTC_ALC_SYNC )
                    ( clock_sync_is_time_valid( clock_sync_context ) == true ) )
#else   // ADD_SMTC_ALC_SYNC
                    ( lorawan_api_is_time_valid( ) == true ) )
#endif  // ADD_SMTC_ALC_SYNC
                {
                    SMTC_MODEM_HAL_TRACE_PRINTF( "Certif enable classB\n" );
                    lorawan_api_class_b_enabled( true );
                }
                else
                {
                    SMTC_MODEM_HAL_TRACE_PRINTF( "Certif classB could not be enabled\n" );
#if defined( ADD_SMTC_ALC_SYNC )
                    if( clock_sync_is_time_valid( clock_sync_context ) == false )
#else   // ADD_SMTC_ALC_SYNC
                    if( lorawan_api_is_time_valid( ) == false )
#endif  // ADD_SMTC_ALC_SYNC
                    {
                        modem_supervisor_add_task_device_time_req( 1 );
                    }
                    if( lorawan_api_get_ping_slot_info_req_status( ) != OKLORAWAN )
                    {
                        modem_supervisor_add_task_ping_slot_info_req( 2 );
                    }
                }
            }
            break;
        default:
            SMTC_MODEM_HAL_TRACE_ERROR( "Unknown event %u\n", current_event.event_type );
            break;
        }
    } while( event_pending_count > 0 );

    if( new_certification_cmd == 1 )
    {
        smtc_modem_dm_set_info_interval( SMTC_MODEM_DM_INFO_INTERVAL_IN_SECOND, 0x00 );

        lorawan_certification_parser_ret_t status_certif =
            lorawan_api_certification( rx_payload, rx_payload_size, user_payload, &user_payload_length, &user_port );

        SMTC_MODEM_HAL_TRACE_PRINTF( "status_certif 0x%x\n", status_certif );
        switch( status_certif )
        {
        case LORAWAN_CERTIFICATION_RET_CERTIF_UL:
            break;
        case LORAWAN_CERTIFICATION_RET_LINK_CHECK:
            is_request_tx = false;
            // lorawan_api_send_stack_cid_req( LINK_CHECK_REQ );
            break;
        case LORAWAN_CERTIFICATION_RET_DEVICE_TIME:
            is_request_tx = false;
            // lorawan_api_send_stack_cid_req( DEVICE_TIME_REQ );
            break;
        case LORAWAN_CERTIFICATION_RET_PING_SLOT:
            is_request_tx = false;
            // lorawan_api_send_stack_cid_req( PING_SLOT_INFO_REQ );
            break;
        case LORAWAN_CERTIFICATION_RET_TX_CW: {
            uint16_t timeout;
            uint32_t frequency;
            int8_t   tx_power;
            smtc_modem_leave_network( stack_id );
            lorawan_api_certification_get_cw_config( &timeout, &frequency, &tx_power );
            smtc_modem_alarm_start_timer( timeout );
            smtc_modem_test_start( );
            smtc_modem_test_tx_cw( frequency, tx_power );
            break;
        }
        case LORAWAN_CERTIFICATION_RET_SWITCH_CLASS: {
            lorawan_certification_class_t certif_class = lorawan_api_certification_get_requested_class( );
            if( certif_class == LORAWAN_CERTIFICATION_CLASS_A )
            {
                lorawan_api_class_b_enabled( false );
                lorawan_api_class_c_enabled( false );
            }
            else if( certif_class == LORAWAN_CERTIFICATION_CLASS_B )
            {
                lorawan_api_class_c_enabled( false );

#if defined( ADD_SMTC_ALC_SYNC )
                if( clock_sync_is_time_valid( clock_sync_context ) == false )
#else   // ADD_SMTC_ALC_SYNC
                if( lorawan_api_is_time_valid( ) == false )
#endif  // ADD_SMTC_ALC_SYNC
                {
                    modem_supervisor_add_task_device_time_req( 0 );
                    // modem_supervisor_add_task_clock_sync_time_req( 0 );
                }
                if( lorawan_api_get_ping_slot_info_req_status( ) != OKLORAWAN )
                {
                    modem_supervisor_add_task_ping_slot_info_req( 0 );
                }
            }
            else if( certif_class == LORAWAN_CERTIFICATION_CLASS_C )
            {
                lorawan_api_class_b_enabled( false );
                lorawan_api_class_c_enabled( true );
            }

            break;
        }
        case LORAWAN_CERTIFICATION_RET_NOTHING:
        case LORAWAN_CERTIFICATION_RET_APP_UL:
        default:
            user_port           = 1;
            user_payload_length = 1;  // lorawan_api_next_max_payload_length_get( );
            memset( user_payload, 1, user_payload_length );
            break;
        }

        if( !lorawan_api_certification_is_enabled( ) )
        {
            smtc_modem_dm_set_info_interval( SMTC_MODEM_DM_INFO_INTERVAL_IN_SECOND, 60 );
        }
        new_certification_cmd         = 0;
        certification_data_is_pending = true;
    }

    if( current_event.event_data.downdata.window == SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXBEACON )
    {
        if( lorawan_api_certification_get_beacon_rx_status_ind_ctrl( ) == true )
        {
            user_payload_length = 0;
            // Send data beacon to testing tool
            lorawan_api_certification_build_beacon_rx_status_ind(
                rx_payload, rx_payload_size, user_payload, &user_payload_length,
                current_event.event_data.downdata.rssi - 64, current_event.event_data.downdata.snr >> 2,
                current_event.event_data.downdata.datarate, current_event.event_data.downdata.frequency_hz );

            SMTC_MODEM_HAL_TRACE_ARRAY( "BEACON to testing tool", user_payload, user_payload_length );
            user_port     = 224;
            is_request_tx = true;
        }
    }
    else if( certification_data_is_pending == false )
    {
        user_port           = 2;
        user_payload_length = 1;  // lorawan_api_next_max_payload_length_get( );
        memset( user_payload, 2, user_payload_length );
    }

    if( is_request_tx == true )
    {
        is_request_tx                 = false;
        certification_data_is_pending = false;
        smtc_modem_request_uplink( stack_id, user_port, lorawan_api_certification_get_frame_type( ), user_payload,
                                   user_payload_length );
    }
}
static void send_task_update( uint8_t event_type )
{
    if( send_task_update_needed == true )
    {
        if( lorawan_api_rx_ack_bit_get( ) == 1 )
        {
            increment_asynchronous_msgnumber( event_type, MODEM_TX_SUCCESS_WITH_ACK );
        }
        else
        {
            increment_asynchronous_msgnumber( event_type, MODEM_TX_SUCCESS );
        }
    }
    else
    {
        increment_asynchronous_msgnumber( event_type, MODEM_TX_FAILED );
    }

    // Re-enable the duty cycle in case of Emergency Tx was sent and dutycycle not disabled by host
    if( modem_get_duty_cycle_disabled_by_host( ) == true )
    {
        lorawan_api_duty_cycle_enable_set( SMTC_DTC_FULL_DISABLED );
    }
    else
    {
        lorawan_api_duty_cycle_enable_set( SMTC_DTC_ENABLED );
    }
}