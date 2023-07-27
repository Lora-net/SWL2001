/**
 * @file      lorawan_mpa_package.c
 *
 * @brief      Implements the LoRa-Alliance fragmented data block transport package
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
#include "lorawan_api.h"
#include "lorawan_mpa_package.h"
#include "lorawan_fragmentation_package.h"
#include "lorawan_remote_multicast_setup_package.h"
#include "lorawan_alcsync.h"
#include "lorawan_fmp_package.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
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
 * \brief Get a particular bit value from a byte
 *
 * \param [IN] b Any byte from which we want a bit value
 * \param [IN] p Position of the bit in the byte [0..7]
 * \param [IN] n Number of bits we want to get
 * \retval The value corresponding the requested bits
 */
#define TAKE_N_BITS_FROM( b, p, n ) ( ( ( b ) >> ( p ) ) & ( ( 1 << ( n ) ) - 1 ) )

// Return byte b from value v
#define BYTE( v, b ) ( ( ( v ) >> ( 8 * ( b ) ) ) & 0xFF )

/*
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

// Request message sizes (with header)

#define MPA_PKG_VERSION_REQ_SIZE ( 1 )
#define MPA_DEV_VERSION_REQ_SIZE ( 1 )
#define MPA_MULTI_PACK_BUFFER_REQ_SIZE ( 128 )

#define MPA_PKG_VERSION_ANS_SIZE ( 3 )
#define MPA_DEV_VERSION_ANS_SIZE ( 14 )
#define MPA_MULTI_PACK_BUFFER_ANS_SIZE ( 128 )

#define MPA_SIZE_ANS_MAX MPA_MULTI_PACK_BUFFER_ANS_SIZE

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum mpa_package_ans_e
{
    MPA_PKG_VERSION_ANS       = 0x00,
    MPA_DEV_VERSION_ANS       = 0x01,
    MPA_MULTI_PACK_BUFFER_ANS = 0x02,

} mpa_package_ans_t;

typedef enum mpa_package_req_e
{
    MPA_PKG_VERSION_REQ       = 0x00,
    MPA_DEV_VERSION_REQ       = 0x01,
    MPA_MULTI_PACK_BUFFER_REQ = 0x02,

} mpa_package_req_t;

typedef enum
{
    EMPTY_TASK_MASK   = 0,
    ANS_CMD_TASK_MASK = 0x1,
} mpa_supervisor_task_types_t;

typedef struct lorawan_mpa_package_s
{
    uint8_t stack_id;
    uint8_t task_id;
    // Uplink buffer ans
    uint8_t                     mpa_tx_payload_ans[MPA_SIZE_ANS_MAX];
    uint8_t                     mpa_tx_payload_ans_size;
    uint16_t                    mpa_ans_delay;
    mpa_supervisor_task_types_t mpa_task_ctx_mask;
    mpa_supervisor_task_types_t mpa_current_task_ctx;
    bool                        enabled;
} lorawan_mpa_package_ctx_t;

typedef enum
{
    MPA_STATUS_OK,
    MPA_STATUS_ERROR
} mpa_status_t;

/* -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static mpa_status_t mpa_package_parser( lorawan_mpa_package_ctx_t* ctx, uint8_t* mpa_package_rx_buffer,
                                        uint8_t mpa_package_rx_buffer_length, receive_win_t rx_window, uint8_t stack_id,
                                        uint8_t* event_status, bool* increment_event );
static bool         mpa_package_parser_internal( uint8_t stack_id, uint8_t* payload_in, receive_win_t rx_window,
                                                 uint8_t* payload_out, uint8_t* payload_out_length );
typedef void ( *ptr_func )( uint8_t*, uint8_t*, uint8_t* );
static ptr_func mpa_service_get_id[MPA_NB_PACKAGES] = {
    lorawan_mpa_package_service_get_id,
    lorawan_alcsync_service_get_id,
    lorawan_remote_multicast_setup_package_service_get_id,
    lorawan_fragmentation_package_service_get_id,
    lorawan_fmp_package_service_get_id,
};
typedef bool ( *ptr_func2 )( uint8_t stack_id, uint8_t* payload_in, receive_win_t rx_window, uint8_t* payload_out,
                             uint8_t* payload_out_length );
static ptr_func2 mpa_service_push_payload_to_targeted_package[MPA_NB_PACKAGES] = {
    mpa_package_parser_internal,  // set parser func
    //  lorawan_alcsync_package_service_mpa_injector,
    lorawan_fragmentation_package_service_mpa_injector, lorawan_remote_multicast_setup_mpa_injector,

    lorawan_fragmentation_package_service_mpa_injector, lorawan_fmp_mpa_injector
    //  lorawan_fmp_package_service_mpa_injector,
};
static lorawan_mpa_package_ctx_t lorawan_mpa_package_ctx[NUMBER_OF_MPA_PACKAGE_OBJ];
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
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

    memset( &lorawan_mpa_package_ctx[*service_id], 0, sizeof( lorawan_mpa_package_ctx_t ) );

    *downlink_callback  = lorawan_mpa_package_service_downlink_handler;
    *on_launch_callback = lorawan_mpa_package_service_on_launch;
    *on_update_callback = lorawan_mpa_package_service_on_update;
    *context_callback   = ( void* ) &( lorawan_mpa_package_ctx[*service_id] );

    lorawan_mpa_package_ctx[*service_id].task_id  = task_id;
    lorawan_mpa_package_ctx[*service_id].stack_id = CURRENT_STACK;
    lorawan_mpa_package_ctx[*service_id].enabled  = true;

#if defined( FUOTA_BUILT_IN_TEST )
    mpa_test( );
#endif

}

void lorawan_mpa_package_service_on_launch( void* service_id )
{
    lorawan_mpa_package_ctx_t* ctx = ( lorawan_mpa_package_ctx_t* ) service_id;

    if( ( ctx->mpa_task_ctx_mask & ANS_CMD_TASK_MASK ) == ANS_CMD_TASK_MASK )
    {
        ctx->mpa_current_task_ctx = ANS_CMD_TASK_MASK;
        SMTC_MODEM_HAL_TRACE_PRINTF( " lorawan_mpa_package launch ANS_CMD_TASK n" );
        if( ctx->mpa_tx_payload_ans_size > 0 )
        {
            lorawan_api_payload_send(
                MPA_PORT, true, ctx->mpa_tx_payload_ans, ctx->mpa_tx_payload_ans_size, UNCONF_DATA_UP,
                smtc_modem_hal_get_time_in_ms( ) + smtc_modem_hal_get_random_nb_in_range( 0, ctx->mpa_ans_delay ),
                ctx->stack_id );
            ctx->mpa_tx_payload_ans_size = 0;
        }
        return;
    }
}

void lorawan_mpa_package_service_on_update( void* service_id )
{
    lorawan_mpa_package_ctx_t* ctx = ( lorawan_mpa_package_ctx_t* ) service_id;
    switch( ctx->mpa_current_task_ctx )
    {
    case ANS_CMD_TASK_MASK: {
        ctx->mpa_task_ctx_mask &= ~( ANS_CMD_TASK_MASK );
        break;
    }
    default:
        break;
    }
}
uint8_t lorawan_mpa_package_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    uint8_t stack_id = rx_down_data->stack_id;

    lorawan_mpa_package_ctx_t* ctx = NULL;
    for( uint8_t i = 0; i < NUMBER_OF_MPA_PACKAGE_OBJ; i++ )
    {
        if( lorawan_mpa_package_ctx[i].stack_id == stack_id )
        {
            ctx = &lorawan_mpa_package_ctx[i];

            break;
        }
    }

    if( ctx == NULL )
    {
        return false;
    }

    if( stack_id >= NUMBER_OF_MPA_PACKAGE_OBJ )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "stack id not valid %u \n", stack_id );
        return false;
    }

    if( ctx->enabled != true )
    {
        return false;
    }

    if( ( rx_down_data->rx_metadata.rx_fport_present == true ) && ( rx_down_data->rx_metadata.rx_fport == MPA_PORT ) )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "lorawan_mpa_package_service_downlink_handler receive data on port %d\n",
                                     MPA_PORT );
        uint8_t      event_status;
        bool         increment_event;
        mpa_status_t mpa_status =
            mpa_package_parser( ctx, rx_down_data->rx_payload, rx_down_data->rx_payload_size,
                                rx_down_data->rx_metadata.rx_window, stack_id, &event_status, &increment_event );
        // check if answer have to been transmit
        if( ( mpa_status == MPA_STATUS_OK ) && ( ctx->mpa_task_ctx_mask != EMPTY_TASK_MASK ) )
        {
            mpa_add_task( ctx, smtc_modem_hal_get_time_in_s( ) );
        }

        return true;
    }

    return false;
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

static void mpa_add_task( lorawan_mpa_package_ctx_t* ctx, uint32_t rtc_target_s )

{
    smodem_task task       = { 0 };
    task.id                = ctx->task_id;
    task.stack_id          = ctx->stack_id;
    task.priority          = TASK_HIGH_PRIORITY;
    task.time_to_execute_s = rtc_target_s;
    if( modem_supervisor_add_task( &task ) != TASK_VALID )
    {
        SMTC_MODEM_HAL_PANIC( "Task not valid\n" );
    }
}

static mpa_status_t mpa_package_parser( lorawan_mpa_package_ctx_t* ctx, uint8_t* mpa_package_rx_buffer,
                                        uint8_t mpa_package_rx_buffer_length, receive_win_t rx_window, uint8_t stack_id,
                                        uint8_t* event_status, bool* increment_event )
{
    uint8_t mpa_package_rx_buffer_index = 0;
    uint8_t ans_index                   = 0;

    *increment_event             = false;
    ctx->mpa_tx_payload_ans_size = 0;
    ctx->mpa_ans_delay           = 0;
    uint8_t pkt_id_tmp;
    uint8_t pkt_id_tmp_previous          = MPA_ID;
    ctx->mpa_tx_payload_ans[ans_index++] = MPA_MULTI_PACK_BUFFER_ANS;
    ctx->mpa_tx_payload_ans[ans_index++] = 0;
    while( mpa_package_rx_buffer_length > mpa_package_rx_buffer_index )
    {
        if( ( mpa_package_rx_buffer[mpa_package_rx_buffer_index] & 0x80 ) == 0x0 )
        {
            // case first byte is cmd_id and pkt_id = 0
            pkt_id_tmp = pkt_id_tmp_previous;
        }
        else
        {
            pkt_id_tmp                           = mpa_package_rx_buffer[mpa_package_rx_buffer_index++];
            ctx->mpa_tx_payload_ans[ans_index++] = pkt_id_tmp;
        }
        uint8_t payload_out_length;
        mpa_service_push_payload_to_targeted_package[pkt_id_tmp](
            ctx->stack_id, &mpa_package_rx_buffer[mpa_package_rx_buffer_index], rx_window,
            &( ctx->mpa_tx_payload_ans[ans_index] ), &payload_out_length );
        ans_index += payload_out_length;
    }
    uint8_t token                        = mpa_package_rx_buffer[mpa_package_rx_buffer_length - 1];
    ctx->mpa_tx_payload_ans[ans_index++] = token;
    ctx->mpa_tx_payload_ans_size         = ans_index;
    return MPA_STATUS_OK;
}
static bool mpa_package_parser_internal( uint8_t stack_id, uint8_t* payload_in, receive_win_t rx_window,
                                         uint8_t* payload_out, uint8_t* payload_out_length )
{
    uint8_t mpa_package_rx_buffer_index = 0;
    uint8_t ans_index                   = 0;

    switch( payload_in[mpa_package_rx_buffer_index] )
    {
    case MPA_PKG_VERSION_REQ: {
        payload_out[ans_index++] = MPA_PKG_VERSION_ANS;
        payload_out[ans_index++] = MPA_ID;
        payload_out[ans_index++] = MPA_VERSION;
        *payload_out_length      = ans_index;
        break;
    }

    case MPA_DEV_VERSION_REQ: {
        uint8_t pkt_id;
        uint8_t pkt_version;
        uint8_t pkt_port;
        payload_out[ans_index++] = MPA_DEV_VERSION_ANS;
        payload_out[ans_index++] = MPA_NB_PACKAGES;
        for( uint8_t i = 0; i < MPA_NB_PACKAGES; i++ )
        {
            mpa_service_get_id[i]( &pkt_id, &pkt_version, &pkt_port );
            payload_out[ans_index++] = pkt_id;
            payload_out[ans_index++] = pkt_version;
            payload_out[ans_index++] = pkt_port;
        }
        *payload_out_length = ans_index;
        break;
    }
    }

    // If this message was received on a multicast address, the end-device MUST check that the
    // multicast address used was enabled at the creation of the fragmentation session through the
    // McGroupBitMask field of the FragSessionSetup command. If not, the frame SHALL be
    // silently dropped.

    return true;
}

#if defined( FUOTA_BUILT_IN_TEST )
/* add static fonction for tests purpose */
static bool mpa_test( void )
{
    return true;
}
#endif
/* --- EOF ------------------------------------------------------------------ */
