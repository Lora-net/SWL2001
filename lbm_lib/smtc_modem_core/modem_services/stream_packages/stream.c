/*!
 * @file      stream.c
 *
 * @brief     streaming code implementation
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
#include <stdint.h>  // C99 types
#include <string.h>

#include "modem_core.h"
#include "modem_supervisor_light.h"
#include "smtc_modem_api.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "lorawan_api.h"
#include "modem_event_utilities.h"
#include "device_management_defs.h"

#include "rose.h"
#include "stream.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )
#define NUMBER_MAX_OF_STREAM_OBJ 1  // modify in case of multiple obj

#define MODEM_TASK_DELAY_MS ( smtc_modem_hal_get_random_nb_in_range( 200, 3000 ) )

/**
 * @brief Check is the index is valid before accessing object
 *
 */
#define IS_VALID_OBJECT_ID( x )                                          \
    do                                                                   \
    {                                                                    \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_MAX_OF_STREAM_OBJ ); \
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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct stream_s
{
    uint8_t stack_id;
    uint8_t task_id;

    status_lorawan_t send_status;
    rose_t           ROSE;
    bool             is_stream_init;
    uint8_t          port;
    bool             follow_dm_port;
    bool             encryption;
    bool             is_data_streaming;  //!<  stream task is on going
} stream_ctx_t;

typedef struct stream_service_ctx_s
{
    stream_ctx_t stream_ctx[NUMBER_MAX_OF_STREAM_OBJ];
} stream_service_ctx_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static stream_service_ctx_t stream_service_ctx;
#define stream_ctx stream_service_ctx.stream_ctx

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Get the service context linked to a stack_id
 *
 * @param [in]  stack_id    Stack identifier
 * @param [out] service_id  Associated occurrence of this service
 * @return stream_ctx_t*    stream object context
 */
static stream_ctx_t* stream_get_ctx_from_stack_id( uint8_t stack_id, uint8_t* service_id );

/*!
 * @brief   Enqueue task in supervisor
 *
 * @param [in] ctx                  stream object context
 */
static void stream_add_task( stream_ctx_t* ctx );

/*!
 * @brief   Indicates if data is pending for uplink
 *
 * @param [in] ROSE*                Pointer to Stream context
 * @retval bool                     True if data is pending
 */
static bool stream_data_pending( rose_t* ROSE );

/*!
 * @brief   Get a new stream fragment to uplink
 *
 * @param [in] ROSE*                Pointer to Stream context
 * @param [out] buf                 Pointer to a buffer where the fragment will be written
 * @param [in] frag_ctn             LoRa Frame Counter that will be used in the uplink message
 * @param [inout] len               As input: Max length of the buffer
 *                                  As output: Effective length filled by the fragment
 *
 * @retval stream_return_code_t     STREAM_OK if successful,
 *                                  STREAM_FAIL if incorrect pointers,
 *                                  STREAM_BADSIZE if the buffer is too small,
 *                                  STREAM_OVERRUN if the underlying ROSE buffer has overrun
 */
static stream_return_code_t stream_get_fragment( rose_t* ROSE, uint8_t* buf, uint32_t frag_ctn, uint8_t* len );

/*!
 * @brief   Process a downlink stream command SCMD.
 *
 * @param [in] ctx                  stream object context
 * @param [in] payload              Pointer to a buffer containing the command
 * @param [in] len                  Length of the command
 *
 * @retval stream_return_code_t     STREAM_OK if successful,
 *                                  STREAM_UNKNOWN_SCMD if the command is not correct
 */
static stream_return_code_t stream_process_dn_frame( stream_ctx_t* ctx, const uint8_t* payload, uint8_t len );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void stream_services_init( uint8_t* service_id, uint8_t task_id,
                           uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                           void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                           void** context_callback )
{
    stream_ctx_t* ctx = &stream_ctx[*service_id];
    memset( ctx, 0, sizeof( stream_ctx_t ) );

    IS_VALID_OBJECT_ID( *service_id );
    *downlink_callback  = stream_service_downlink_handler;
    *on_launch_callback = stream_service_on_launch;
    *on_update_callback = stream_service_on_update;
    *context_callback   = ( void* ) service_id;
    ctx->task_id        = task_id;
    ctx->stack_id       = CURRENT_STACK;
    ctx->ROSE.stack_id  = CURRENT_STACK;
    ctx->port           = DM_PORT;
    SMTC_MODEM_HAL_TRACE_WARNING( "%s\n", __func__ );
}

void stream_service_on_launch( void* service_id )
{
    uint8_t idx = *( ( uint8_t* ) service_id );
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " %d service_id %d \n", __func__, idx );

    IS_VALID_OBJECT_ID( idx );

    uint8_t              stream_payload[242] = { 0 };
    uint8_t              fragment_size;
    uint32_t             frame_cnt;
    stream_return_code_t stream_rc;
    uint8_t              tx_buff_offset = 0;

    if( lorawan_api_isjoined( stream_ctx[idx].stack_id ) != JOINED )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "DEVICE NOT JOINED \n" );
        return;
    }
    if( stream_ctx[idx].is_stream_init == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Streaming not initialized \n" );
        return;
    }

    // check first if stream runs on dm port and if yes add dm code
    if( stream_ctx[idx].follow_dm_port == true )
    {
#if defined( ADD_SMTC_CLOUD_DEVICE_MANAGEMENT )
        stream_ctx[idx].port = cloud_dm_get_dm_port( stream_ctx[idx].stack_id );
#endif
        stream_payload[tx_buff_offset] = DM_INFO_STREAM;
        tx_buff_offset++;
    }

    // XXX Check if a streaming session is already active
    fragment_size = lorawan_api_next_max_payload_length_get( stream_ctx[idx].stack_id ) - tx_buff_offset;
    frame_cnt     = lorawan_api_fcnt_up_get( stream_ctx[idx].stack_id );
    stream_rc =
        stream_get_fragment( &stream_ctx[idx].ROSE, &stream_payload[tx_buff_offset], frame_cnt, &fragment_size );

    // TODO Is this enough to ensure we send everything?
    if( ( stream_rc == STREAM_OK ) && ( fragment_size > 0 ) )
    {
        stream_ctx[idx].send_status = tx_protocol_manager_request (TX_PROTOCOL_TRANSMIT_LORA,
            stream_ctx[idx].port, true, stream_payload, fragment_size + tx_buff_offset, UNCONF_DATA_UP,
            smtc_modem_hal_get_time_in_ms( )  , stream_ctx[idx].stack_id );
    }
    else
    {
        // TODO
        // Insufficient data or streaming done
        stream_ctx[idx].is_data_streaming = false;
        SMTC_MODEM_HAL_TRACE_WARNING( "Stream get fragment FAILED\n" );
    }
}

void stream_service_on_update( void* service_id )
{
    uint8_t idx = *( ( uint8_t* ) service_id );
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " %s service_id %d \n", __func__, idx );
    IS_VALID_OBJECT_ID( idx );

    if( stream_data_pending( &stream_ctx[idx].ROSE ) )
    {
        stream_add_task( &stream_ctx[idx] );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Streaming DONE\n" );
        stream_ctx[idx].is_data_streaming = false;

        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_STREAM_DONE, 0, stream_ctx[idx].stack_id );
    }
}

uint8_t stream_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    uint8_t stack_id = rx_down_data->stack_id;

    if( rx_down_data->rx_metadata.rx_window == RECEIVE_NONE )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    uint8_t       service_id;
    stream_ctx_t* ctx = stream_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( ctx->is_stream_init == false )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    uint8_t dm_port;
#if defined( ADD_SMTC_CLOUD_DEVICE_MANAGEMENT )
    dm_port = cloud_dm_get_dm_port( stack_id );
#else
    dm_port = DM_PORT;
#endif

    if( ( rx_down_data->rx_metadata.rx_fport_present == true ) && ( rx_down_data->rx_metadata.rx_fport == dm_port ) &&
        ( rx_down_data->rx_payload_size > DM_DOWNLINK_HEADER_LENGTH ) &&
        ( ( dm_opcode_t ) rx_down_data->rx_payload[2] == DM_STREAM ) )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "%s\n", __func__ );

        if( stream_process_dn_frame( ctx, &rx_down_data->rx_payload[3],
                                     rx_down_data->rx_payload_size - DM_DOWNLINK_HEADER_LENGTH ) != STREAM_OK )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad DM Stream downlink\n" );
        }

        return MODEM_DOWNLINK_CONSUMED;
    }

    return MODEM_DOWNLINK_UNCONSUMED;
}

stream_return_code_t stream_init( uint8_t stack_id, uint8_t f_port, bool encryption, uint8_t redundancy_ratio_percent )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t       service_id;
    stream_ctx_t* ctx = stream_get_ctx_from_stack_id( stack_id, &service_id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ctx != NULL );

    // First reset stream service
    memset( &ctx->ROSE, 0, sizeof( rose_t ) );

    // prepare stream module
    if( ROSE_init( &ctx->ROSE, ROSE_DEFAULT_WL, ROSE_DEFAULT_MINFREE, redundancy_ratio_percent, 1 ) != ROSE_OK )
    {
        return STREAM_FAIL;
    }

    if( encryption == true )
    {
        ROSE_enable_encryption( &ctx->ROSE );
    }

    if( f_port == 0 )
    {
        f_port = cloud_dm_get_dm_port( ctx->stack_id );
    }

    if( f_port == cloud_dm_get_dm_port( ctx->stack_id ) )
    {
        ctx->follow_dm_port = true;
    }
    else
    {
        ctx->follow_dm_port = false;
    }

    ctx->port           = f_port;
    ctx->encryption     = encryption;
    ctx->is_stream_init = true;

    // Remove previous ongoing stream task to avoid event generation
    modem_supervisor_remove_task( ctx->task_id );

    return STREAM_OK;
}

bool stream_encrypted_mode( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t       service_id;
    stream_ctx_t* ctx = stream_get_ctx_from_stack_id( stack_id, &service_id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ctx != NULL );
    return ctx->encryption;
}

bool stream_get_init_status( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t       service_id;
    stream_ctx_t* ctx = stream_get_ctx_from_stack_id( stack_id, &service_id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ctx != NULL );
    return ctx->is_stream_init;
}

bool stream_get_status( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t       service_id;
    stream_ctx_t* ctx = stream_get_ctx_from_stack_id( stack_id, &service_id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ctx != NULL );
    return ( ctx->is_data_streaming );
}

// (only allowed when joined)
stream_return_code_t stream_add_data( uint8_t stack_id, const uint8_t* data, uint8_t len )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t       service_id;
    stream_ctx_t* ctx = stream_get_ctx_from_stack_id( stack_id, &service_id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ctx != NULL );

    // for now only one stream supported
    int err = 0;

    if( data == NULL )
    {
        return STREAM_FAIL;
    }

    if( len == 0 )
    {
        return STREAM_BADSIZE;
    }

    // check data record length
    err = ROSE_addRecord( &ctx->ROSE, &data[0], len );
    if( err == ROSE_BAD_DATALEN )
    {
        return STREAM_BADSIZE;
    }
    if( err == ROSE_OVERRUN )
    {
        return STREAM_BUSY;
    }
    if( err != ROSE_OK )
    {
        return STREAM_FAIL;
    }

    stream_add_task( ctx );

    // Update is_data_streaming
    ctx->is_data_streaming = true;

    return STREAM_OK;
}

void stream_status( uint8_t stack_id, uint16_t* pending, uint16_t* free )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t       service_id;
    stream_ctx_t* ctx = stream_get_ctx_from_stack_id( stack_id, &service_id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ctx != NULL );

    if( pending != NULL )
    {
        *pending = ROSE_getPending( &ctx->ROSE );
    }
    if( free != NULL )
    {
        *free = ROSE_getFree( &ctx->ROSE );
    }
}

uint8_t stream_get_port( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t       service_id;
    stream_ctx_t* ctx = stream_get_ctx_from_stack_id( stack_id, &service_id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ctx != NULL );
    return ( ctx->port );
}

uint8_t stream_get_rr( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t       service_id;
    stream_ctx_t* ctx = stream_get_ctx_from_stack_id( stack_id, &service_id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ctx != NULL );
    return ( ctx->ROSE.rr );
}

void stream_set_rr( uint8_t stack_id, uint8_t stream_rr )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t       service_id;
    stream_ctx_t* ctx = stream_get_ctx_from_stack_id( stack_id, &service_id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ctx != NULL );
    ctx->ROSE.rr = stream_rr;
}

void stream_service_stop( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t       service_id;
    stream_ctx_t* ctx = stream_get_ctx_from_stack_id( stack_id, &service_id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ctx != NULL );

    // Reset Rose buff
    memset( &ctx->ROSE, 0, sizeof( rose_t ) );
    // Remove previous ongoing stream task to avoid event generation
    modem_supervisor_remove_task( ctx->task_id );
    // Reset service state to NOT_INIT
    ctx->is_stream_init = false;
    // Reset is_data_streaming status
    ctx->is_data_streaming = false;
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static stream_ctx_t* stream_get_ctx_from_stack_id( uint8_t stack_id, uint8_t* service_id )
{
    stream_ctx_t* ctx = NULL;
    for( uint8_t i = 0; i < NUMBER_MAX_OF_STREAM_OBJ; i++ )
    {
        if( stream_ctx[i].stack_id == stack_id )
        {
            ctx         = &stream_ctx[i];
            *service_id = i;
            break;
        }
    }

    return ctx;
}

static void stream_add_task( stream_ctx_t* ctx )
{
    smodem_task stream_task       = { 0 };
    stream_task.id                = ctx->task_id;
    stream_task.stack_id          = ctx->stack_id;
    stream_task.priority          = TASK_MEDIUM_HIGH_PRIORITY;
    stream_task.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + ( MODEM_TASK_DELAY_MS / 1000 );

    modem_supervisor_add_task( &stream_task );
}

static bool stream_data_pending( rose_t* ROSE )
{
    return ROSE_getStatus( ROSE ) == ROSE_PENDTX;
}

static stream_return_code_t stream_get_fragment( rose_t* ROSE, uint8_t* buf, uint32_t frag_ctn, uint8_t* len )
{
    int rose_rc;

    if( buf == NULL || len == NULL )
    {
        return STREAM_FAIL;
    }

    if( *len == 0 )
    {
        return STREAM_BADSIZE;
    }

    // Initialize return buffer
    memset( buf, 0xff, *len );

    rose_rc = ROSE_getData( ROSE, frag_ctn, buf, len );

    switch( rose_rc )
    {
    case ROSE_LFRAME_SIZE:
        return STREAM_BADSIZE;
    case ROSE_OVERRUN:
        return STREAM_OVERRUN;
    }

    // LOG_PRINTF( "stream_get_fragment (%d) [", *len );
    // LOG_PACKARRAY( "", buf, *len );
    // LOG_MSG( "]\n" );
    return STREAM_OK;
}

static stream_return_code_t stream_process_dn_frame( stream_ctx_t* ctx, const uint8_t* payload, uint8_t len )
{
    int rc;
    if( payload == NULL )
    {
        return STREAM_FAIL;
    }

    rc = ROSE_processDnFrame( &ctx->ROSE, payload, len );
    if( rc == ROSE_NOTFORME )
    {
        return STREAM_UNKNOWN_SCMD;
    }
    return STREAM_OK;
}

/* --- EOF ------------------------------------------------------------------ */
