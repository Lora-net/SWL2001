/*!
 * \file      file_upload.c
 *
 * \brief     File upload implementation
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
#include "file_upload.h"

#include "lorawan_api.h"

#include "modem_event_utilities.h"
#include "device_management_defs.h"
#include "modem_supervisor_light.h"
#include "modem_core.h"
#include "modem_event_utilities.h"
#include "modem_services_common.h"

#include "smtc_modem_crypto.h"

#include "smtc_modem_api.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"

#include <stdbool.h>  // bool type
#include <stdint.h>   // C99 types
#include <string.h>   //memcpy

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )
#define NUMBER_MAX_OF_LFU_OBJ 1  // modify in case of multiple obj

/**
 * @brief Check is the index is valid before accessing object
 *
 */
#define IS_VALID_OBJECT_ID( x )                                       \
    do                                                                \
    {                                                                 \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_MAX_OF_LFU_OBJ ); \
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

// SHA-256

#undef ROR
#undef CH
#undef MAJ
#undef EP0
#undef EP1
#undef SIG0
#undef SIG1

#define ROR( a, b ) ( ( ( a ) >> ( b ) ) | ( ( a ) << ( 32 - ( b ) ) ) )

#define CH( x, y, z ) ( ( ( x ) & ( y ) ) ^ ( ~( x ) & ( z ) ) )
#define MAJ( x, y, z ) ( ( ( x ) & ( y ) ) ^ ( ( x ) & ( z ) ) ^ ( ( y ) & ( z ) ) )
#define EP0( x ) ( ROR( x, 2 ) ^ ROR( x, 13 ) ^ ROR( x, 22 ) )
#define EP1( x ) ( ROR( x, 6 ) ^ ROR( x, 11 ) ^ ROR( x, 25 ) )
#define SIG0( x ) ( ROR( x, 7 ) ^ ROR( x, 18 ) ^ ( ( x ) >> 3 ) )
#define SIG1( x ) ( ROR( x, 17 ) ^ ROR( x, 19 ) ^ ( ( x ) >> 10 ) )

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define ENDIAN_n2b32( x ) __builtin_bswap32( x )
#else
#define ENDIAN_n2b32( x ) ( x )
#endif



/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

// file upload header size
#define FILE_UPLOAD_HEADER_SIZE ( 12 )

// Length of filedone frame
#define FILE_UPLOAD_FILEDONE_FRAME_LENGTH ( 1 )

// File upload maximum size
#ifndef FILE_UPLOAD_MAX_SIZE
#define FILE_UPLOAD_MAX_SIZE ( ( 8 * 1024 ) - FILE_UPLOAD_HEADER_SIZE )
#endif

// number of words per chunk
#define CHUNK_NW ( 2 )

#define FILE_UPLOAD_TOKEN 0x0E
#define FILE_UPLOAD_DIRECTION 0x40

#define UPLOAD_SID 0

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */
/**
 * @brief File upload service state
 *
 * @enum lfu_state_t
 */
typedef enum lfu_state_e
{
    LFU_NOT_INIT = 0,     //!< The file upload is not initialized
    LFU_INIT_AND_FILLED,  //!< The file upload is initialized and filled with data
    LFU_START_REQUESTED,  //!< A start was requested by user but the service is waiting to be launched by supervisor
    LFU_ON_GOING,         //!< The upload is in progress
    LFU_FINISHED,         //!< The upload process is finished
} lfu_state_t;

typedef struct file_upload_s
{
    uint8_t   sid;                   // Session Id (2bits)
    uint16_t  average_delay;         // average frame transmission rate/delay
    uint8_t   port;                  // applicative port on which the upload is done
    bool      encrypt_with_appskey;  // file upload encryption option
    uint8_t   session_counter;       // session counter
    uint32_t* file_buf;              // data buffer
    uint32_t  file_len;              // file len
    uint32_t  header[3];             // Current file upload header
    uint16_t  cct;                   // chunk count
    uint16_t  cntx;                  // chunk transmission count
    uint8_t   fntx;                  // frame transmission count

} file_upload_t;

typedef struct lfu_ctx_s
{
    uint8_t          stack_id;
    uint8_t          task_id;
    file_upload_t    lfu;
    lfu_state_t      state;
    status_lorawan_t send_status;
    uint8_t          sctr;
} lfu_ctx_t;

typedef struct lfu_service_ctx_s
{
    lfu_ctx_t lfu_ctx[NUMBER_MAX_OF_LFU_OBJ];
} lfu_service_ctx_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lfu_service_ctx_t lfu_service_ctx;
#define lfu_ctx lfu_service_ctx.lfu_ctx

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

// Service management
static void       lfu_service_on_launch( void* service_id );
static void       lfu_service_on_update( void* service_id );
static uint8_t    lfu_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );
static lfu_ctx_t* lfu_get_ctx_from_stack_id( uint8_t stack_id, uint8_t* service_id );
static void       lfu_add_task( lfu_ctx_t* ctx, uint32_t delay_in_s );

// file upload management
/**
 * @brief Process the downlink frame FILEDONE
 *
 * @param [in] file_upload  Pointer to File Upload context
 * @param [in] payload      Pointer to a buffer containing the data
 * @param [in] len          Length of the data
 * @return file_upload_return_code_t FILE_UPLOAD_OK if the filedone corresponds to current session
 */
file_upload_return_code_t file_upload_process_file_done_frame( file_upload_t* file_upload, const uint8_t* payload,
                                                               uint8_t len );

/**
 * @brief Once the file is attached to the current upload session, a preparation must be called before start
 *
 * @param [in] file_upload Pointer to File Upload context
 * @return file_upload_return_code_t
 */
file_upload_return_code_t file_upload_prepare_upload( lfu_ctx_t* ctx );

/**
 * @brief File upload fragment generation
 *
 * @param [in] file_upload Pointer to File Upload context
 * @param [in] buf         buffer that will contain the fragment
 * @param [in] len         buffer size
 * @param [in] fcnt        frame counter
 * @return int32_t Return the number of pending byte(s)
 */
int32_t file_upload_get_fragment( file_upload_t* file_upload, uint8_t* buf, int32_t len, uint32_t fcnt );

/**
 * @brief Check if there are remaining file data that need to be sent
 *
 * @param [in] file_upload Pointer to File Upload context
 * @return true
 * @return false
 */
bool file_upload_is_data_remaining( file_upload_t* file_upload );

// Algo
static uint32_t phash( uint32_t x );
static uint32_t checkbits( uint32_t cid, uint32_t cct, uint32_t i );
static void     function_xor( uint32_t* dst, uint32_t* src, int32_t nw );
static void     gen_chunk( file_upload_t* file_upload, uint32_t* dst, uint32_t* src, uint32_t cct, uint32_t cid );

/**
 * @brief Compute SHA256
 *
 * @param [in] hash Contains the computed hash
 * @param [in] msg  input buffer
 * @param [in] len  input buffer length
 */
static void sha256( uint32_t* hash, const uint8_t* msg, uint32_t len );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- SERVICE MANAGEMENT ------------------------------------------------------
 */

void lfu_services_init( uint8_t* service_id, uint8_t task_id,
                        uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                        void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                        void** context_callback )
{
    lfu_ctx_t* ctx = &lfu_ctx[*service_id];
    memset( ctx, 0, sizeof( lfu_ctx_t ) );

    IS_VALID_OBJECT_ID( *service_id );
    *downlink_callback  = lfu_service_downlink_handler;
    *on_launch_callback = lfu_service_on_launch;
    *on_update_callback = lfu_service_on_update;
    *context_callback   = ( void* ) service_id;

    ctx->task_id  = task_id;
    ctx->stack_id = CURRENT_STACK;
    ctx->state    = LFU_NOT_INIT;
    memset( &ctx->lfu, 0, sizeof( file_upload_t ) );

    SMTC_MODEM_HAL_TRACE_WARNING( "%s\n", __func__ );
}

/*
 * -----------------------------------------------------------------------------
 * --- LFU FUNCTIONS -----------------------------------------------------------
 */

file_upload_return_code_t file_upload_init( uint8_t stack_id, const uint8_t* file, uint32_t file_len,
                                            uint16_t average_delay, uint8_t port, bool encryption )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t    service_id;
    lfu_ctx_t* ctx = lfu_get_ctx_from_stack_id( stack_id, &service_id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ctx != NULL );

    if( file_len > FILE_UPLOAD_MAX_SIZE )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FileUpload is too large (%d > %d )\n", file_len, FILE_UPLOAD_MAX_SIZE );
        return FILE_UPLOAD_ERROR_SIZE;
    }
    if( file_len == 0 )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "File Upload size shall be different from 0\n" );
        return FILE_UPLOAD_ERROR_SIZE;
    }
    if( ctx->state != LFU_NOT_INIT )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "File Upload still in going\n" );
        return FILE_UPLOAD_ERROR_BUSY;
    }

    uint16_t sz_tmp = file_len + FILE_UPLOAD_HEADER_SIZE;
    uint32_t cct    = ( sz_tmp + ( ( 4 * CHUNK_NW ) - 1 ) ) / ( 4 * CHUNK_NW );

    ctx->lfu.sid                  = UPLOAD_SID & 0x3;
    ctx->lfu.session_counter      = ( ctx->lfu.session_counter + 1 ) & 0xf;
    ctx->lfu.encrypt_with_appskey = encryption;
    ctx->lfu.average_delay        = average_delay;
    ctx->lfu.port                 = port;
    ctx->lfu.file_len             = file_len;
    ctx->lfu.file_buf             = ( uint32_t* ) file;
    ctx->lfu.cct                  = cct;
    ctx->lfu.cntx                 = 0;
    ctx->lfu.fntx                 = 0;
    ctx->lfu.header[0] =
        ( port ) + ( encryption << 8 ) + ( ( file_len & 0xFF ) << 16 ) + ( ( ( file_len & 0xFF00 ) >> 8 ) << 24 );

    ctx->state = LFU_INIT_AND_FILLED;

    SMTC_MODEM_HAL_TRACE_PRINTF( "File Upload Init done: cipher_mode: %d, size:%d, average_delay:%d, index:%d\n",
                                 encryption, file_len, average_delay, port );

    return FILE_UPLOAD_OK;
}

file_upload_return_code_t file_upload_start( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t    service_id;
    lfu_ctx_t* ctx = lfu_get_ctx_from_stack_id( stack_id, &service_id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ctx != NULL );

    if( ctx->state == LFU_ON_GOING )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FileUpload still in progress..\n" );
        return FILE_UPLOAD_ERROR_BUSY;
    }
    if( ctx->state != LFU_INIT_AND_FILLED )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "File upload session not initialized\n" );
        return FILE_UPLOAD_ERROR_NOT_INIT;
    }

    // add the first upload task in scheduler
    lfu_add_task( ctx, smtc_modem_hal_get_random_nb_in_range( 200, 3000 ) / 1000 );

    // Now update state to START_REQUESTED
    ctx->state = LFU_START_REQUESTED;
    return FILE_UPLOAD_OK;
}

file_upload_return_code_t file_upload_reset( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t    service_id;
    lfu_ctx_t* ctx = lfu_get_ctx_from_stack_id( stack_id, &service_id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ctx != NULL );

    if( ctx->state == LFU_NOT_INIT )
    {
        return FILE_UPLOAD_ERROR_NOT_INIT;
    }
    SMTC_MODEM_HAL_TRACE_WARNING( "File Upload Cancel and session reset!\n" );

    // remove on going task
    modem_supervisor_remove_task( ctx->task_id );

    // Reset state
    ctx->state = LFU_NOT_INIT;

    return FILE_UPLOAD_OK;
}

bool file_upload_get_status( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t    service_id;
    lfu_ctx_t* ctx = lfu_get_ctx_from_stack_id( stack_id, &service_id );
    if( ctx == NULL )
    {
        return false;
    }
    return ( ( ctx->state == LFU_ON_GOING ) || ( ctx->state == LFU_START_REQUESTED ) ) ? true : false;
}

void file_upload_stop_service( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t    service_id;
    lfu_ctx_t* ctx = lfu_get_ctx_from_stack_id( stack_id, &service_id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ctx != NULL );

    // remove on going task
    modem_supervisor_remove_task( ctx->task_id );

    // Reset state
    ctx->state = LFU_NOT_INIT;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

// LFU service management
static void lfu_service_on_launch( void* service_id )
{
    uint8_t idx = *( ( uint8_t* ) service_id );
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " %d service_id %d \n", __func__, idx );
    IS_VALID_OBJECT_ID( idx );

    int32_t file_upload_chunk_size         = 0;
    uint8_t file_upload_chunk_payload[242] = { 0 };

    if( lorawan_api_isjoined( lfu_ctx[idx].stack_id ) != JOINED )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "DEVICE NOT JOIN \n" );
        return;
    }
    if( lfu_ctx[idx].state == LFU_START_REQUESTED )
    {
        // first time task is handled, prepare the upload

        file_upload_prepare_upload( &lfu_ctx[idx] );
        lfu_ctx[idx].state = LFU_ON_GOING;
    }

    if( lfu_ctx[idx].state != LFU_ON_GOING )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "No File upload on going \n" );
        return;
    }
    uint32_t max_payload_size = lorawan_api_next_max_payload_length_get( lfu_ctx[idx].stack_id );
    file_upload_chunk_size    = file_upload_get_fragment( &lfu_ctx[idx].lfu, file_upload_chunk_payload,
                                                       ( max_payload_size > 100 ) ? 100 : max_payload_size,
                                                          lorawan_api_fcnt_up_get( lfu_ctx[idx].stack_id ) );
    if( file_upload_chunk_size > 0 )
    {
        uint8_t dm_port;
#if defined( ADD_SMTC_CLOUD_DEVICE_MANAGEMENT )
        dm_port = cloud_dm_get_dm_port( lfu_ctx[idx].stack_id );
#else
        dm_port = DM_PORT;
#endif
        lfu_ctx[idx].send_status =
            tx_protocol_manager_request (TX_PROTOCOL_TRANSMIT_LORA, dm_port, true, file_upload_chunk_payload, file_upload_chunk_size, UNCONF_DATA_UP,
                                      smtc_modem_hal_get_time_in_ms( )  , lfu_ctx[idx].stack_id );
    }
    else
    {
        // something prevents fragment to be constructed (max payload size < 11 due to mac answers in fopts and
        // shall be uplinked first)
        lfu_ctx[idx].send_status =
            tx_protocol_manager_request (TX_PROTOCOL_TRANSMIT_LORA, 0, false, NULL, 0, UNCONF_DATA_UP,
                                      smtc_modem_hal_get_time_in_ms( )  , lfu_ctx[idx].stack_id );
    }
}

static void lfu_service_on_update( void* service_id )
{
    uint8_t idx = *( ( uint8_t* ) service_id );
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " %s service_id %d \n", __func__, idx );
    IS_VALID_OBJECT_ID( idx );

    if( lfu_ctx[idx].state == LFU_ON_GOING )
    {
        if( ( file_upload_is_data_remaining( &lfu_ctx[idx].lfu ) == true ) )
        {
            // There is still upload that need to be sent => add a new task
            lfu_add_task( &lfu_ctx[idx], lfu_ctx[idx].lfu.average_delay );
        }
        else
        {
            // Nothing left to be sent => abort upload and generate event
            SMTC_MODEM_HAL_TRACE_WARNING( "File upload ended without server confirmation \n" );
            // Reset service state to uninit
            lfu_ctx[idx].state = LFU_NOT_INIT;
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_UPLOAD_DONE, SMTC_MODEM_EVENT_UPLOAD_DONE_ABORTED,
                                              lfu_ctx[idx].stack_id );
        }
    }
    else if( lfu_ctx[idx].state == LFU_FINISHED )
    {
        // Reset service state to uninit
        lfu_ctx[idx].state = LFU_NOT_INIT;
        // file upload is finished with server confirmation, notify user
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_UPLOAD_DONE, SMTC_MODEM_EVENT_UPLOAD_DONE_SUCCESSFUL,
                                          lfu_ctx[idx].stack_id );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "lfu update task with no LFU on going \n" );
    }
}

static uint8_t lfu_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    uint8_t stack_id = rx_down_data->stack_id;

    uint8_t service_id;

    lfu_ctx_t* ctx = lfu_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
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
        ( ( dm_opcode_t ) rx_down_data->rx_payload[2] == DM_FILE_DONE ) )
    {
        if( ctx->state != LFU_ON_GOING )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "No FileUpload ongoing\n" );
            return MODEM_DOWNLINK_UNCONSUMED;
        }

        // Process downlink
        if( file_upload_process_file_done_frame( &ctx->lfu, &rx_down_data->rx_payload[3],
                                                 rx_down_data->rx_payload_size - DM_DOWNLINK_HEADER_LENGTH ) !=
            FILE_UPLOAD_OK )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "DM_FILE_DONE bad session_counter or bad message\n" );
        }
        else
        {
            // file upload is done with server confirmation
            SMTC_MODEM_HAL_TRACE_INFO( "File upload DONE with server confirmation \n" );

            // Reset service state to uninit
            ctx->state = LFU_FINISHED;
        }
        return MODEM_DOWNLINK_CONSUMED;
    }
    return MODEM_DOWNLINK_UNCONSUMED;
}

static lfu_ctx_t* lfu_get_ctx_from_stack_id( uint8_t stack_id, uint8_t* service_id )
{
    lfu_ctx_t* ctx = NULL;
    for( uint8_t i = 0; i < NUMBER_MAX_OF_LFU_OBJ; i++ )
    {
        if( lfu_ctx[i].stack_id == stack_id )
        {
            ctx         = &lfu_ctx[i];
            *service_id = i;
            break;
        }
    }

    return ctx;
}

static void lfu_add_task( lfu_ctx_t* ctx, uint32_t delay_in_s )
{
    smodem_task lfu_task = { 0 };

    lfu_task.id                = ctx->task_id;
    lfu_task.stack_id          = ctx->stack_id;
    lfu_task.priority          = TASK_MEDIUM_HIGH_PRIORITY;
    lfu_task.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + delay_in_s;

    modem_supervisor_add_task( &lfu_task );
}

// LFU functionalities
file_upload_return_code_t file_upload_prepare_upload( lfu_ctx_t* ctx )
{
    uint32_t hash[8];
    sha256( hash, ( unsigned char* ) ctx->lfu.file_buf, ctx->lfu.file_len );
    ctx->lfu.header[1] = hash[0];
    ctx->lfu.header[2] = hash[1];

    if( ctx->lfu.encrypt_with_appskey == true )
    {
        // encrypt using AppSKey with "upload" category and file size and hash as diversification data
        uint8_t nonce[14] = { 0 };

        nonce[0] = 0x01;

        nonce[5]  = FILE_UPLOAD_DIRECTION;
        nonce[6]  = ctx->lfu.file_len & 0xFF;
        nonce[7]  = ( ctx->lfu.file_len >> 8 ) & 0xFF;
        nonce[8]  = ( ctx->lfu.file_len >> 16 ) & 0xFF;
        nonce[9]  = ( ctx->lfu.file_len >> 24 ) & 0xFF;
        nonce[10] = hash[0] & 0xFF;
        nonce[11] = ( hash[0] >> 8 ) & 0xFF;
        nonce[12] = ( hash[0] >> 16 ) & 0xFF;
        nonce[13] = ( hash[0] >> 24 ) & 0xFF;
        if( smtc_modem_crypto_service_encrypt( ( uint8_t* ) ctx->lfu.file_buf, ctx->lfu.file_len, nonce,
                                               ( uint8_t* ) ctx->lfu.file_buf,
                                               ctx->stack_id ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
        {
            SMTC_MODEM_HAL_PANIC( "Encryption of lfu failed\n" );
        }

        // compute hash over encrypted data
        sha256( hash, ( unsigned char* ) ctx->lfu.file_buf, ctx->lfu.file_len );

        // hash over plain data (first byte)
        ctx->lfu.header[2] = ctx->lfu.header[1];
        // hash over encrypted data (first byte)
        ctx->lfu.header[1] = hash[0];
    }
    return FILE_UPLOAD_OK;
}

int32_t file_upload_get_fragment( file_upload_t* file_upload, uint8_t* buf, int32_t len, uint32_t fcnt )
{
    if( ( len - 3 ) < ( CHUNK_NW * 4 ) )
    {
        return 0;
    }
    len = len - 3;
    // discriminator (16bit little endian): 2bit session id, 4bit session
    // counter, 10bit chunk count-1
    uint32_t d = ( ( file_upload->sid & 0x03 ) << 14 ) | ( ( file_upload->session_counter & 0x0F ) << 10 ) |
                 ( ( file_upload->cct - 1 ) & 0x03FF );
    int32_t n     = 0;
    buf[n++]      = FILE_UPLOAD_TOKEN;
    buf[n++]      = d;
    buf[n++]      = d >> 8;
    uint32_t  cid = phash( fcnt );
    uint32_t* src = &file_upload->file_buf[0];

    while( len >= ( CHUNK_NW * 4 ) )
    {
        uint32_t tmp[CHUNK_NW];
        gen_chunk( file_upload, tmp, src, file_upload->cct, cid++ );
        memcpy( buf + n, tmp, CHUNK_NW * 4 );
        n += ( CHUNK_NW * 4 );
        len -= ( CHUNK_NW * 4 );
    }
    if( n > 0 )
    {
        file_upload->cntx += ( n - 3 ) / ( CHUNK_NW * 4 );  // update number of chunks sent
        if( file_upload->fntx < 255 )
        {
            file_upload->fntx += 1;  // update number of frames sent
        }
    }
    return n;
}

bool file_upload_is_data_remaining( file_upload_t* file_upload )
{
    // limit number of chunks sent to twice the chunk count but send minimum three frames
    return ( ( file_upload->fntx < 3 ) || ( file_upload->cntx < ( 2 * file_upload->cct ) ) );
}

file_upload_return_code_t file_upload_process_file_done_frame( file_upload_t* file_upload, const uint8_t* payload,
                                                               uint8_t len )
{
    if( len != FILE_UPLOAD_FILEDONE_FRAME_LENGTH )
    {
        return FILE_UPLOAD_ERROR_DL;
    }

    // TODO: check if session id is also present in downlink and compare it to current

    // check if sctr value in filedone message corresponds to current sctr
    if( ( payload[0] & 0xf ) == file_upload->session_counter )
    {
        return FILE_UPLOAD_OK;
    }
    else
    {
        return FILE_UPLOAD_ERROR_DL;
    }
}

static void gen_chunk( file_upload_t* file_upload, uint32_t* dst, uint32_t* src, uint32_t cct, uint32_t cid )
{
    memset( dst, 0, CHUNK_NW * 4 );
    uint32_t bits = 0;  // initialized to make compiler happy
    for( uint32_t i = 0; i < cct; i++ )
    {
        if( ( i & 31 ) == 0 )
        {
            bits = checkbits( cid, cct, i >> 5 );
        }
        if( bits == 0 )
        {
            continue;
        }
        if( bits & 1 )
        {
            if( i == 0 )
            {
                uint32_t tmp[2];
                tmp[0] = file_upload->header[0];
                tmp[1] = file_upload->header[1];
                function_xor( dst, tmp, CHUNK_NW );
            }
            else if( i == 1 )
            {
                uint32_t tmp[2];
                tmp[0] = file_upload->header[2];
                tmp[1] = *( src );
                function_xor( dst, tmp, CHUNK_NW );
            }
            else
            {
                function_xor( dst, src + ( CHUNK_NW * i ) - 3, CHUNK_NW );
            }
        }
        bits >>= 1;
    }
}

// 32bit pseudo hash
static uint32_t phash( uint32_t x )
{
    x = ( ( x >> 16 ) ^ x ) * 0x45d9f3b;
    x = ( ( x >> 16 ) ^ x ) * 0x45d9f3b;
    x = ( ( x >> 16 ) ^ x );
    return x;
}

static uint32_t checkbits( uint32_t cid, uint32_t cct, uint32_t i )
{
    uint32_t ncw = ( cct + 31 ) >> 5;  // number of checkwords per chunk
    return phash( cid * ncw + i );
}

static void function_xor( uint32_t* dst, uint32_t* src, int32_t nw )
{
    while( nw-- > 0 )
    {
        *dst++ ^= *src++;
    }
}

static void sha256_do( uint32_t* state, const uint8_t* block )
{
    static const uint32_t K[64] = { 0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
                                    0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
                                    0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
                                    0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
                                    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
                                    0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
                                    0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
                                    0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
                                    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
                                    0xc67178f2 };

    uint32_t a, b, c, d, e, f, g, h, i, j, t1, t2, w[64];

    for( i = 0, j = 0; i < 16; i++, j += 4 )
    {
        w[i] = ( block[j] << 24 ) | ( block[j + 1] << 16 ) | ( block[j + 2] << 8 ) | ( block[j + 3] );
    }
    for( ; i < 64; i++ )
    {
        w[i] = SIG1( w[i - 2] ) + w[i - 7] + SIG0( w[i - 15] ) + w[i - 16];
    }

    a = state[0];
    b = state[1];
    c = state[2];
    d = state[3];
    e = state[4];
    f = state[5];
    g = state[6];
    h = state[7];

    for( i = 0; i < 64; i++ )
    {
        t1 = h + EP1( e ) + CH( e, f, g ) + K[i] + w[i];
        t2 = EP0( a ) + MAJ( a, b, c );
        h  = g;
        g  = f;
        f  = e;
        e  = d + t1;
        d  = c;
        c  = b;
        b  = a;
        a  = t1 + t2;
    }

    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
    state[5] += f;
    state[6] += g;
    state[7] += h;
}

static void sha256( uint32_t* hash, const uint8_t* msg, uint32_t len )
{
    uint32_t state[8] = {
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
    };

    uint32_t bitlen = len << 3;
    while( 1 )
    {
        if( len < 64 )
        {
            union
            {
                uint8_t  bytes[64];
                uint32_t words[16];
            } tmp;
            memset( tmp.words, 0, sizeof( tmp ) );
            memcpy( tmp.bytes, msg, len );
            tmp.bytes[len] = 0x80;
            if( len < 56 )
            {
            last:
                tmp.words[15] = ENDIAN_n2b32( bitlen );
                sha256_do( state, tmp.bytes );
                int i;
                for( i = 0; i < 8; i++ )
                {
                    hash[i] = ENDIAN_n2b32( state[i] );
                }
                break;
            }
            else
            {
                sha256_do( state, tmp.bytes );
                memset( tmp.words, 0, sizeof( tmp ) );
                goto last;
            }
        }
        else
        {
            sha256_do( state, msg );
            msg += 64;
            len -= 64;
        }
    }
}

/* --- EOF ------------------------------------------------------------------ */
