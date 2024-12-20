/**
 * @file      lorawan_mpa_package.c
 *
 * @brief     Implements the LoRa-Alliance Multi-Package Access Protocol package
 *            Specification:
 * https://resources.lora-alliance.org/document/ts007-1-0-0-multi-package-access
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

#include "modem_event_utilities.h"
#include "lorawan_send_management.h"
#include "modem_supervisor_light.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "modem_core.h"
#include "lr1mac_defs.h"
#include "lorawan_api.h"
#include "lorawan_mpa_package.h"
#include "lorawan_alcsync.h"
#include "lorawan_remote_multicast_setup_package.h"
#include "lorawan_fragmentation_package.h"
#include "lorawan_fmp_package.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*!
 * \brief Returns the minimum value between a and b
 *
 * \param [in] a 1st value
 * \param [in] b 2nd value
 * \retval minValue Minimum value
 */
#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

/**
 * @brief Number of object
 *
 */

#define NUMBER_OF_MPA_PACKAGE_OBJ 1
/**
 * @brief Compute current LoRaWAN Stack from the supervisor task_id
 *
 */
#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )
#define CURRENT_TASK_CONTEXT \
    ( modem_supervisor_get_task( ) )->modem_task[( modem_supervisor_get_task( ) )->next_task_id].task_context
/**
 * @brief Check is the index is valid before accessing object
 *
 */
#define IS_VALID_OBJECT_ID( x )                                   \
    do                                                            \
    {                                                             \
        if( x >= NUMBER_OF_MPA_PACKAGE_OBJ )                      \
        {                                                         \
            SMTC_MODEM_HAL_PANIC( "not valid service_id %d", x ); \
        }                                                         \
    } while( 0 )

#define IS_VALID_PKG_CMD( x )                                                    \
    do                                                                           \
    {                                                                            \
        if( ( mpa_package_rx_buffer_index + x ) > mpa_package_rx_buffer_length ) \
        {                                                                        \
            SMTC_MODEM_HAL_TRACE_ERROR( "%u\n", mpa_package_rx_buffer_length );  \
            return MPA_STATUS_ERROR;                                             \
        }                                                                        \
    } while( 0 )

/*!
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * LoRaWAN Application Layer Fragmented Data Block Transport Specification
 */

#define MPA_PORT 225
#define MPA_ID 0
#define MPA_VERSION 1
#define MPA_NB_PACKAGES 5

/**
 * @brief Request message sizes (with header)
 */
#define MPA_PKG_VERSION_REQ_SIZE ( 1 )
#define MPA_DEV_PACKAGE_REQ_SIZE ( 1 )
#define MPA_MULTI_PACK_BUFFER_REQ_SIZE ( 3 )

/**
 * @brief Answer message sizes (with header)
 */
#define MPA_PKG_VERSION_ANS_SIZE ( 3 )
#define MPA_DEV_PACKAGE_ANS_SIZE ( 14 )
#define MPA_MULTI_PACK_BUFFER_FRAG_SIZE ( 128 )

#define MPA_SIZE_ANS_MAX MPA_MULTI_PACK_BUFFER_FRAG_SIZE

#define MPA_DEV_PKG_HDR_SIZE ( 2 )  // CommandID + Nb Packages
#define MPA_FRAG_HDR_SIZE ( 2 )     // CommandID + BaseByte
#define MPA_TOKEN_SIZE ( 1 )

static const uint8_t mpa_req_cmd_size[3] = { MPA_PKG_VERSION_REQ_SIZE, MPA_DEV_PACKAGE_REQ_SIZE,
                                             MPA_MULTI_PACK_BUFFER_REQ_SIZE };
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum mpa_package_ans_e
{
    MPA_PKG_VERSION_ANS        = 0x00,
    MPA_DEV_PACKAGE_ANS        = 0x01,
    MPA_MULTI_PACK_BUFFER_FRAG = 0x02,
} mpa_package_ans_t;

typedef enum mpa_package_req_e
{
    MPA_PKG_VERSION_REQ       = 0x00,
    MPA_DEV_PACKAGE_REQ       = 0x01,
    MPA_MULTI_PACK_BUFFER_REQ = 0x02,
} mpa_package_req_t;

typedef enum
{
    MPA_STATUS_OK,
    MPA_STATUS_ERROR
} mpa_status_t;

typedef enum
{
    EMPTY_TASK_MASK = 0,
    ANS_CMD_TASK_MASK
} mpa_supervisor_task_types_t;

typedef enum mpa_multi_pack_buffer_req_status_e
{
    MPA_MULTI_PACK_BUFFER_REQ_NOT_RCV = 0x00,
    MPA_MULTI_PACK_BUFFER_REQ_OK      = 0x01,
    MPA_MULTI_PACK_BUFFER_REQ_PENDING = 0x02,
    MPA_MULTI_PACK_BUFFER_REQ_ERROR   = 0xFF,
} mpa_multi_pack_buffer_req_status_t;

typedef struct lorawan_mpa_package_s
{
    uint8_t stack_id;
    uint8_t task_id;
    bool    enabled;

    // Uplink buffer ans
    uint8_t mpa_tx_payload_ans[MPA_SIZE_ANS_MAX - MPA_FRAG_HDR_SIZE - MPA_TOKEN_SIZE];
    uint8_t mpa_tx_payload_ans_size;
    uint8_t mpa_tx_payload_ans_size_cpy;

    uint8_t mpa_task_ctx_mask;
    uint8_t token;

    mpa_multi_pack_buffer_req_status_t mpa_multi_pack_buffer_req_status;
    uint8_t                            start_byte;
    uint8_t                            stop_byte;
    uint8_t                            start_byte_pending;
    uint8_t                            stop_byte_pending;
    uint8_t                            nb_cmd_parsed;
} lorawan_mpa_package_ctx_t;

/* -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/**
 * @brief The parser for this package
 *
 * @param stack_id
 * @param payload_in
 * @param nb_bytes_read_payload_in
 * @param rx_window
 * @param payload_out
 * @param payload_out_length
 * @param max_payload_out_length
 * @param rx_timestamp_ms
 * @return true
 * @return false
 */
static bool mpa_package_parser_internal( uint8_t stack_id, uint8_t* payload_in, uint8_t* nb_bytes_read_payload_in,
                                         receive_win_t rx_window, uint8_t* payload_out, uint8_t* payload_out_length,
                                         const uint8_t max_payload_out_length, uint32_t rx_timestamp_ms );

/**
 * @brief Get the PackageID, Version_id and PackageFPort
 *
 * @param [out] pkt_id
 * @param [out] pkt_version
 * @param [out] pkt_port
 */
typedef void ( *ptr_func_service_get_id )( uint8_t*, uint8_t*, uint8_t* );

/**
 * @brief Get the Service ID, version and FPort for each package
 */
static ptr_func_service_get_id mpa_service_get_id[MPA_NB_PACKAGES] = {
    lorawan_mpa_package_service_get_id,
    lorawan_alcsync_service_get_id,
    lorawan_remote_multicast_setup_package_service_get_id,
    lorawan_fragmentation_package_service_get_id,
    lorawan_fmp_package_service_get_id,
};

/**
 * @brief Generic type for all injector
 *
 * @param [in]  stack_id                  The requested stack_id
 * @param [in]  payload_in                The payload that will be parsed
 * @param [out] nb_bytes_read_payload_in  The number of bytes read in the parsed payload
 * @param [in]  rx_window                 The window that received the downlink
 * @param [out] payload_out               The returned payload
 * @param [out] payload_out_length        The number of bytes in returned payload
 * @param [in]  max_payload_out_length    The size max the payload_out could be contains
 * @param [in]  rx_timestamp_ms           The timestamp of the received window
 * @return true     successfully parsed
 * @return false    parsing error
 */
typedef bool ( *ptr_func_mpa_injector )( uint8_t stack_id, uint8_t* payload_in, uint8_t* nb_bytes_read_payload_in,
                                         receive_win_t rx_window, uint8_t* payload_out, uint8_t* payload_out_length,
                                         const uint8_t max_payload_out_length, uint32_t rx_timestamp_ms );

static ptr_func_mpa_injector mpa_service_push_payload_to_targeted_package[MPA_NB_PACKAGES] = {
    mpa_package_parser_internal,                         // set parser func
    lorawan_alcsync_mpa_injector,                        // alc sync
    lorawan_remote_multicast_setup_mpa_injector,         // remote multicast
    lorawan_fragmentation_package_service_mpa_injector,  // fragmentation
    lorawan_fmp_mpa_injector                             // firmware management
};

/**
 * @brief MPA context
 */
static lorawan_mpa_package_ctx_t lorawan_mpa_package_ctx[NUMBER_OF_MPA_PACKAGE_OBJ];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Return the true is the rx_window is multicast
 *
 * @param rx_window
 * @return true
 * @return false
 */
static bool is_received_on_multicast_window( receive_win_t rx_window );

/**
 * @brief Multi-package parser
 *
 * @param ctx
 * @param mpa_package_rx_buffer
 * @param mpa_package_rx_buffer_length
 * @param rx_window
 * @param stack_id
 * @param event_status
 * @param increment_event
 * @return mpa_status_t
 */
static mpa_status_t mpa_package_parser( lorawan_mpa_package_ctx_t* ctx, uint8_t* mpa_package_rx_buffer,
                                        uint8_t mpa_package_rx_buffer_length, receive_win_t rx_window, uint8_t stack_id,
                                        uint32_t rx_timestamp_ms, uint8_t* event_status, bool* increment_event );

/**
 * @brief Get the service context linked to a stack_id
 *
 * @param [in]  stack_id                Stack identifier
 * @param [out] service_id              Associated occurrence of this service
 * @return lorawan_mpa_package_ctx_t*   Package object context
 */
static lorawan_mpa_package_ctx_t* lorawan_mpa_package_get_ctx_from_stack_id( uint8_t stack_id, uint8_t* service_id );

static void mpa_add_task( lorawan_mpa_package_ctx_t* ctx, uint32_t rtc_target_s );
/**
 * @brief parse the command received from the application server
 *
 * @param [in] ctx
 * @param [in] mpa_package_rx_buffer        buffer that will be decoded
 * @param [in] mpa_package_rx_buffer_length buffer length
 * @param [in] mpa_package_rx_window        window type to filter unicast/multicast
 * @return frag_status_t
 */

#if defined( FUOTA_BUILT_IN_TEST )
static bool mpa_test( void );
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lorawan_mpa_package_services_init( uint8_t* service_id, uint8_t task_id,
                                        uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                        void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                                        void** context_callback )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG(
        " lorawan_mpa_package_services_init task_id %d, service_id %d, CURRENT_STACK:%d \n", task_id, *service_id,
        CURRENT_STACK );

    IS_VALID_OBJECT_ID( *service_id );

    lorawan_mpa_package_ctx_t* ctx = &lorawan_mpa_package_ctx[*service_id];
    memset( ctx, 0, sizeof( lorawan_mpa_package_ctx_t ) );

    *downlink_callback  = lorawan_mpa_package_service_downlink_handler;
    *on_launch_callback = lorawan_mpa_package_service_on_launch;
    *on_update_callback = lorawan_mpa_package_service_on_update;
    *context_callback   = ( void* ) ctx;

    ctx->task_id   = task_id;
    ctx->stack_id  = CURRENT_STACK;
    ctx->enabled   = true;
    ctx->stop_byte = MPA_SIZE_ANS_MAX - 1;

#if defined( FUOTA_BUILT_IN_TEST )
    mpa_test( );
#endif
}

void lorawan_mpa_package_service_on_launch( void* service_id )
{
    lorawan_mpa_package_ctx_t* ctx = ( lorawan_mpa_package_ctx_t* ) service_id;

    if( ( ctx->mpa_task_ctx_mask & ANS_CMD_TASK_MASK ) == ANS_CMD_TASK_MASK )
    {
        uint8_t tmp_ans[MPA_SIZE_ANS_MAX];
        uint8_t tmp_ans_size = 0;

        uint8_t max_payload_size = lorawan_api_next_max_payload_length_get( ctx->stack_id );

        // If the multi_pack_buffer_req has an error status
        if( ctx->mpa_multi_pack_buffer_req_status == MPA_MULTI_PACK_BUFFER_REQ_ERROR )
        {
            tmp_ans[tmp_ans_size++] = MPA_MULTI_PACK_BUFFER_FRAG;             // Command ID
            tmp_ans[tmp_ans_size++] = ctx->mpa_multi_pack_buffer_req_status;  // Status fail
        }

        // Fragment the payload if payload is too big to be sent in one frame
        // or if the Application server request a specific fragment
        else if( ( ctx->mpa_tx_payload_ans_size_cpy > ( max_payload_size - MPA_FRAG_HDR_SIZE - MPA_TOKEN_SIZE ) ) ||
                 ( ctx->mpa_multi_pack_buffer_req_status == MPA_MULTI_PACK_BUFFER_REQ_OK ) )
        {
            // Set to true if we have to fragment the payload and no MultiPackBufferReq where received
            ctx->mpa_multi_pack_buffer_req_status = MPA_MULTI_PACK_BUFFER_REQ_OK;

            max_payload_size -= ( MPA_FRAG_HDR_SIZE + MPA_TOKEN_SIZE );
            // Compute the length of data that will be send
            max_payload_size = MIN( max_payload_size, MIN( ( uint32_t ) ctx->mpa_tx_payload_ans_size_cpy,
                                                           ( uint32_t ) ( ctx->stop_byte - ctx->start_byte ) ) );

            tmp_ans[tmp_ans_size++] = MPA_MULTI_PACK_BUFFER_FRAG;  // Command ID
            tmp_ans[tmp_ans_size++] = ctx->start_byte;             // Base byte

            memcpy( &tmp_ans[tmp_ans_size], &ctx->mpa_tx_payload_ans[ctx->start_byte],
                    max_payload_size );        // MPA answers
            tmp_ans_size += max_payload_size;  // add size MPA answers
        }
        else
        {
            memcpy( tmp_ans, ctx->mpa_tx_payload_ans, ctx->mpa_tx_payload_ans_size_cpy );  // MPA answers
            tmp_ans_size = ctx->mpa_tx_payload_ans_size_cpy;
        }

        if( tmp_ans_size > 0 )
        {
            tmp_ans[tmp_ans_size++] = ctx->token;  // Token

            SMTC_MODEM_HAL_TRACE_PRINTF( " lorawan_mpa_package launch ANS_CMD_TASK n" );
            SMTC_MODEM_HAL_TRACE_ARRAY( "MPA ans", tmp_ans, tmp_ans_size );
            if( tx_protocol_manager_request(
                    TX_PROTOCOL_TRANSMIT_LORA, MPA_PORT, true, tmp_ans, tmp_ans_size, UNCONF_DATA_UP,
                    smtc_modem_hal_get_time_in_ms( ) + smtc_modem_hal_get_random_nb_in_range( 0, 2000 ),
                    ctx->stack_id ) == OKLORAWAN )
            {
                // The multi_pack_buffer_req status error was sent, it can be cleared
                if( ctx->mpa_multi_pack_buffer_req_status == MPA_MULTI_PACK_BUFFER_REQ_ERROR )
                {
                    ctx->mpa_multi_pack_buffer_req_status = MPA_MULTI_PACK_BUFFER_REQ_NOT_RCV;
                }

                if( ctx->mpa_multi_pack_buffer_req_status == MPA_MULTI_PACK_BUFFER_REQ_OK )
                {
                    ctx->start_byte += max_payload_size;
                    ctx->mpa_tx_payload_ans_size_cpy -= max_payload_size;

                    // All the fragmented or request data was sent, multi_pack_buffer_req status can be cleared
                    if( ( ctx->mpa_tx_payload_ans_size_cpy == 0 ) || ( ctx->start_byte >= ctx->stop_byte ) )
                    {
                        ctx->mpa_multi_pack_buffer_req_status = MPA_MULTI_PACK_BUFFER_REQ_NOT_RCV;
                        ctx->mpa_task_ctx_mask &= ~( ANS_CMD_TASK_MASK );
                    }
                }
                else
                {
                    ctx->mpa_task_ctx_mask &= ~( ANS_CMD_TASK_MASK );
                }
            }
        }
    }
}

void lorawan_mpa_package_service_on_update( void* service_id )
{
    lorawan_mpa_package_ctx_t* ctx = ( lorawan_mpa_package_ctx_t* ) service_id;

    if( ( ctx->mpa_task_ctx_mask & ANS_CMD_TASK_MASK ) == ANS_CMD_TASK_MASK )
    {
        mpa_add_task( ctx, smtc_modem_hal_get_time_in_s( ) + smtc_modem_hal_get_random_nb_in_range( 0, 2 ) );
    }
}

uint8_t lorawan_mpa_package_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    uint8_t stack_id = rx_down_data->stack_id;

    uint8_t                    service_id;
    lorawan_mpa_package_ctx_t* ctx = lorawan_mpa_package_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( stack_id >= NUMBER_OF_MPA_PACKAGE_OBJ )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "%s: stack id not valid %u \n", __func__, stack_id );
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( ctx->enabled != true )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( ( rx_down_data->rx_metadata.rx_fport_present == true ) && ( rx_down_data->rx_metadata.rx_fport == MPA_PORT ) )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "lorawan_mpa_package_service_downlink_handler receive data on port %d\n",
                                     MPA_PORT );
        uint8_t      event_status;
        bool         increment_event;
        mpa_status_t mpa_status = mpa_package_parser(
            ctx, rx_down_data->rx_payload, rx_down_data->rx_payload_size, rx_down_data->rx_metadata.rx_window, stack_id,
            rx_down_data->rx_metadata.timestamp_ms, &event_status, &increment_event );
        // check if answer have to been transmit
        if( ( mpa_status == MPA_STATUS_OK ) && ( ctx->mpa_task_ctx_mask != EMPTY_TASK_MASK ) )
        {
            mpa_add_task( ctx, smtc_modem_hal_get_time_in_s( ) );
        }

        return MODEM_DOWNLINK_CONSUMED;
    }

    return MODEM_DOWNLINK_UNCONSUMED;
}

void lorawan_mpa_package_service_get_id( uint8_t* pkt_id, uint8_t* pkt_version, uint8_t* pkt_port )
{
    *pkt_id      = MPA_ID;
    *pkt_version = MPA_VERSION;
    *pkt_port    = MPA_PORT;
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static lorawan_mpa_package_ctx_t* lorawan_mpa_package_get_ctx_from_stack_id( uint8_t stack_id, uint8_t* service_id )
{
    lorawan_mpa_package_ctx_t* ctx = NULL;
    for( uint8_t i = 0; i < NUMBER_OF_MPA_PACKAGE_OBJ; i++ )
    {
        if( lorawan_mpa_package_ctx[i].stack_id == stack_id )
        {
            ctx         = &lorawan_mpa_package_ctx[i];
            *service_id = i;
            break;
        }
    }
    return ctx;
}

static void mpa_add_task( lorawan_mpa_package_ctx_t* ctx, uint32_t rtc_target_s )
{
    smodem_task task       = { 0 };
    task.id                = ctx->task_id;
    task.stack_id          = ctx->stack_id;
    task.priority          = TASK_MEDIUM_HIGH_PRIORITY;
    task.time_to_execute_s = rtc_target_s;
    if( modem_supervisor_add_task( &task ) != TASK_VALID )
    {
        SMTC_MODEM_HAL_PANIC( "Task not valid\n" );
    }
}

static mpa_status_t mpa_package_parser( lorawan_mpa_package_ctx_t* ctx, uint8_t* mpa_package_rx_buffer,
                                        uint8_t mpa_package_rx_buffer_length, receive_win_t rx_window, uint8_t stack_id,
                                        uint32_t rx_timestamp_ms, uint8_t* event_status, bool* increment_event )
{
    uint8_t tmp_tx_ans[30];  // The bigger payload length answer is in Remote Multicast Setup Package and need 30 bytes
                             // REMOTE_MULTICAST_SETUP_MC_GROUP_SETUP_REQ_SIZE
    uint8_t mpa_package_rx_buffer_index = 0;
    uint8_t ans_index                   = 0;

    uint8_t                            pkg_id_tmp;
    uint8_t                            pkg_id_tmp_previous                  = MPA_ID;
    mpa_multi_pack_buffer_req_status_t mpa_multi_pack_buffer_req_status_bkp = ctx->mpa_multi_pack_buffer_req_status;

    *increment_event = false;

    ctx->mpa_multi_pack_buffer_req_status = MPA_MULTI_PACK_BUFFER_REQ_NOT_RCV;

    SMTC_MODEM_HAL_TRACE_ARRAY( "MPA rcv buff", mpa_package_rx_buffer, mpa_package_rx_buffer_length );

    ctx->nb_cmd_parsed = 0;

    while( ( mpa_package_rx_buffer_length - MPA_TOKEN_SIZE ) > mpa_package_rx_buffer_index )
    {
        if( ( mpa_package_rx_buffer[mpa_package_rx_buffer_index] & 0x80 ) == 0x0 )
        {
            // case first byte is cmd_id and pkt_id = 0 or multiple cmd_id below the same pkt_id
            pkg_id_tmp = pkg_id_tmp_previous;
        }
        else
        {
            pkg_id_tmp                           = mpa_package_rx_buffer[mpa_package_rx_buffer_index++];
            pkg_id_tmp_previous                  = pkg_id_tmp;
            ctx->mpa_tx_payload_ans[ans_index++] = pkg_id_tmp;  // Package ID followed by the answers
        }
        uint8_t payload_out_length       = 0;
        uint8_t nb_bytes_read_payload_in = 0;

        if( ( pkg_id_tmp & 0x7F ) >= MPA_NB_PACKAGES )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "MPA parser ID unknown 0x%02x \n", ( pkg_id_tmp & 0x7F ) );
            return MPA_STATUS_ERROR;
        }

        ctx->nb_cmd_parsed++;

        if( mpa_service_push_payload_to_targeted_package[pkg_id_tmp & 0x7F](
                ctx->stack_id, &mpa_package_rx_buffer[mpa_package_rx_buffer_index], &nb_bytes_read_payload_in,
                rx_window, tmp_tx_ans, &payload_out_length, sizeof( tmp_tx_ans ), rx_timestamp_ms ) == false )
        {
            return MPA_STATUS_ERROR;
        }

        // Copy temp answer payload only if mpa_tx_payload_ans is not yet full
        if( ( ( ans_index + payload_out_length ) < sizeof( ctx->mpa_tx_payload_ans ) ) &&
            ( payload_out_length <= sizeof( tmp_tx_ans ) ) )
        {
            memcpy( &ctx->mpa_tx_payload_ans[ans_index], tmp_tx_ans, payload_out_length );  // Copy all answers
            ans_index += payload_out_length;
        }

        // Go to the next command_id or packet_id
        mpa_package_rx_buffer_index += nb_bytes_read_payload_in;
    }

    if( ctx->mpa_multi_pack_buffer_req_status == MPA_MULTI_PACK_BUFFER_REQ_PENDING )
    {
        if( ctx->nb_cmd_parsed == 1 )
        {  // MultiPackBufferReq is the only command present
            ctx->mpa_multi_pack_buffer_req_status = MPA_MULTI_PACK_BUFFER_REQ_OK;
            ctx->start_byte                       = ctx->start_byte_pending;
            ctx->stop_byte                        = ctx->stop_byte_pending;
        }
        else
        {  // Ignore downlink
            ctx->mpa_multi_pack_buffer_req_status = mpa_multi_pack_buffer_req_status_bkp;
            return MPA_STATUS_OK;
        }
    }

    if( ctx->mpa_multi_pack_buffer_req_status == MPA_MULTI_PACK_BUFFER_REQ_NOT_RCV )
    {
        ctx->token = mpa_package_rx_buffer[mpa_package_rx_buffer_length - 1] & 0x03;

        ctx->start_byte              = 0;
        ctx->stop_byte               = MPA_SIZE_ANS_MAX - MPA_FRAG_HDR_SIZE - MPA_TOKEN_SIZE;
        ctx->mpa_tx_payload_ans_size = ans_index;
    }

    ctx->mpa_tx_payload_ans_size_cpy = ctx->mpa_tx_payload_ans_size;

    if( ( ctx->mpa_tx_payload_ans_size > 0 ) ||
        ( ctx->mpa_multi_pack_buffer_req_status != MPA_MULTI_PACK_BUFFER_REQ_NOT_RCV ) )
    {
        ctx->mpa_task_ctx_mask |= ANS_CMD_TASK_MASK;
    }
    return MPA_STATUS_OK;
}

static bool is_received_on_multicast_window( receive_win_t rx_window )
{
    if( ( ( rx_window >= RECEIVE_ON_RXC_MC_GRP0 ) && ( rx_window <= RECEIVE_ON_RXC_MC_GRP3 ) ) ||
        ( ( rx_window >= RECEIVE_ON_RXB_MC_GRP0 ) && ( rx_window <= RECEIVE_ON_RXB_MC_GRP3 ) ) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

static bool mpa_package_parser_internal( uint8_t stack_id, uint8_t* payload_in, uint8_t* nb_bytes_read_payload_in,
                                         receive_win_t rx_window, uint8_t* payload_out, uint8_t* payload_out_length,
                                         const uint8_t max_payload_out_length, uint32_t rx_timestamp_ms )
{
    uint8_t mpa_package_rx_buffer_index = 0;
    uint8_t ans_index                   = 0;
    uint8_t ans_buffer_tmp[MPA_DEV_PKG_HDR_SIZE + ( MPA_NB_PACKAGES * 3 )];  // Max length for DevPackageReq = Header +
                                                                             // NbPackage * (ID,version,FPort)

    // If these messages are received on a multicast address, the end-device SHALL drop them silently
    if( is_received_on_multicast_window( rx_window ) == true )
    {
        return false;
    }

    uint8_t                    service_id;
    lorawan_mpa_package_ctx_t* ctx = lorawan_mpa_package_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
    {
        return false;
    }

    if( stack_id >= NUMBER_OF_MPA_PACKAGE_OBJ )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "%s: stack id not valid %u \n", __func__, stack_id );
        return false;
    }

    if( ctx->enabled != true )
    {
        return false;
    }

    SMTC_MODEM_HAL_TRACE_WARNING( "MPA CiD 0x%02x\n", payload_in[mpa_package_rx_buffer_index] );

    uint8_t cmd_id = payload_in[mpa_package_rx_buffer_index];
    if( cmd_id >= sizeof( mpa_req_cmd_size ) )
    {
        return false;
    }
    *nb_bytes_read_payload_in = mpa_req_cmd_size[cmd_id];

    switch( cmd_id )
    {
    case MPA_PKG_VERSION_REQ:
    {
        ans_buffer_tmp[ans_index++] = MPA_PKG_VERSION_ANS;
        ans_buffer_tmp[ans_index++] = MPA_ID;
        ans_buffer_tmp[ans_index++] = MPA_VERSION;
        break;
    }

    case MPA_DEV_PACKAGE_REQ:
    {
        uint8_t pkt_id;
        uint8_t pkt_version;
        uint8_t pkt_port;

        ans_buffer_tmp[ans_index++] = MPA_DEV_PACKAGE_ANS;
        ans_buffer_tmp[ans_index++] = MPA_NB_PACKAGES & 0x07;

        for( uint8_t i = 0; i < MPA_NB_PACKAGES; i++ )
        {
            mpa_service_get_id[i]( &pkt_id, &pkt_version, &pkt_port );
            ans_buffer_tmp[ans_index++] = pkt_id;
            ans_buffer_tmp[ans_index++] = pkt_version;
            ans_buffer_tmp[ans_index++] = pkt_port;
        }
        break;
    }
    case MPA_MULTI_PACK_BUFFER_REQ:
    {
        ctx->start_byte_pending = payload_in[mpa_package_rx_buffer_index + 1];
        ctx->stop_byte_pending  = payload_in[mpa_package_rx_buffer_index + 2];

        ctx->mpa_multi_pack_buffer_req_status = MPA_MULTI_PACK_BUFFER_REQ_PENDING;

        // Start byte is greater than the payload answer
        if( ctx->start_byte_pending > ( ctx->mpa_tx_payload_ans_size - 1 ) )
        {
            ctx->mpa_multi_pack_buffer_req_status = MPA_MULTI_PACK_BUFFER_REQ_ERROR;
        }

        // Stop byte is smaller than the start byte
        if( ctx->stop_byte_pending < ctx->start_byte_pending )
        {
            ctx->mpa_multi_pack_buffer_req_status = MPA_MULTI_PACK_BUFFER_REQ_ERROR;
        }

        if( ctx->stop_byte_pending > MPA_SIZE_ANS_MAX - MPA_FRAG_HDR_SIZE - MPA_TOKEN_SIZE )
        {
            ctx->stop_byte_pending = MPA_SIZE_ANS_MAX - MPA_FRAG_HDR_SIZE - MPA_TOKEN_SIZE;
        }
        break;
    }
    default:
        SMTC_MODEM_HAL_TRACE_ERROR( "parser mpa_package_parser_internal 0x%x ?\n",
                                    payload_in[mpa_package_rx_buffer_index] );
        return false;
        break;
    }

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ans_index <= max_payload_out_length );
    memcpy( payload_out, ans_buffer_tmp, ans_index );
    *payload_out_length = ans_index;

    SMTC_MODEM_HAL_TRACE_WARNING( "MPA CiD (byte read %u)\n", *nb_bytes_read_payload_in );

    return true;
}

#if defined( FUOTA_BUILT_IN_TEST )
/* add static function for tests purpose */
static bool mpa_test( void )
{
    return true;
}
#endif
/* --- EOF ------------------------------------------------------------------ */
