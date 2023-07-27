/**
 * @file      lorawan_fragmentation_package.c
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
#include "lorawan_fragmentation_package.h"
#include "fragmentation_helper.h"
#include "cmac.h"

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
 * @brief Number of FRAGMENTED_PACKAGE object
 *
 */

#define NUMBER_OF_FRAGMENTED_PACKAGE_OBJ 1
/**
 * @brief Compute current LoRaWAN Stack from the supervisor task_id
 *
 */
#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )

/**
 * @brief Check is the index is valid before accessing object
 *
 */
#define IS_VALID_OBJECT_ID( x )                                   \
    do                                                            \
    {                                                             \
        if( x >= NUMBER_OF_FRAGMENTED_PACKAGE_OBJ )               \
        {                                                         \
            SMTC_MODEM_HAL_PANIC( "not valid service_id %d", x ); \
        }                                                         \
    } while( 0 )

#define IS_VALID_PKG_CMD( x )                                                                        \
    do                                                                                               \
    {                                                                                                \
        if( ( fragmentation_package_rx_buffer_index + x ) > fragmentation_package_rx_buffer_length ) \
        {                                                                                            \
            SMTC_MODEM_HAL_TRACE_ERROR( "%u\n", fragmentation_package_rx_buffer_length );            \
            return FRAG_STATUS_ERROR;                                                                \
        }                                                                                            \
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

#define FRAGMENTATION_PORT 201
#define FRAGMENTATION_ID 3
#define FRAGMENTATION_VERSION 2
#define FRAGMENTATION_MAX_NB_SESSIONS 4
// Request message sizes (with header)
#define FRAGMENTATION_PKG_VERSION_REQ_SIZE ( 1 )
#define FRAGMENTATION_SESSION_STATUS_REQ_SIZE ( 2 )
#define FRAGMENTATION_SESSION_SETUP_REQ_SIZE ( 17 )
#define FRAGMENTATION_SESSION_DELETE_REQ_SIZE ( 2 )
#define FRAGMENTATION_DATA_BLOCK_RECEIVED_REQ_SIZE ( 2 )
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum fragmented_package_ans_e
{
    FRAGMENTATION_PKG_VERSION_ANS         = 0x00,
    FRAGMENTATION_SESSION_STATUS_ANS      = 0x01,
    FRAGMENTATION_SESSION_SETUP_ANS       = 0x02,
    FRAGMENTATION_SESSION_DELETE_ANS      = 0x03,
    FRAGMENTATION_DATA_BLOCK_RECEIVED_ANS = 0x04,
} fragmented_package_ans_t;

typedef enum fragmented_package_req_e
{
    FRAGMENTATION_PKG_VERSION_REQ         = 0x00,
    FRAGMENTATION_SESSION_STATUS_REQ      = 0x01,
    FRAGMENTATION_SESSION_SETUP_REQ       = 0x02,
    FRAGMENTATION_SESSION_DELETE_REQ      = 0x03,
    FRAGMENTATION_DATA_BLOCK_RECEIVED_REQ = 0x04,
    FRAGMENTATION_DATA_FRAGMENT           = 0x08,
} fragmented_package_req_t;
static uint8_t frag_req_cmd_size[4] = { FRAGMENTATION_PKG_VERSION_REQ_SIZE, FRAGMENTATION_SESSION_STATUS_REQ_SIZE,
                                        FRAGMENTATION_SESSION_SETUP_REQ_SIZE, FRAGMENTATION_SESSION_DELETE_REQ };
typedef struct lorawan_fragmentation_package_s
{
    uint8_t stack_id;
    uint8_t task_id;
    // Uplink buffer ans
    uint8_t  fragmentation_tx_payload_ans[FRAG_SIZE_ANS_MAX];
    uint8_t  fragmentation_tx_payload_ans_size;
    uint16_t fragmentation_ans_delay;
    bool     enabled;
} lorawan_fragmentation_package_ctx_t;

typedef struct frag_group_data_s
{
    bool is_active;
    struct
    {
        uint8_t mc_group_bit_mask;
        uint8_t frag_index;
    } frag_session;
    uint16_t frag_nb;
    uint8_t  frag_size;
    struct
    {
        uint8_t block_ack_delay;
        uint8_t frag_algo;
        uint8_t ack_reception;
    } control;
    uint8_t  padding;
    uint32_t descriptor;
    uint16_t session_cnt;

    uint32_t mic;
} frag_group_data_t;

#define frag_decoder_status_t FragDecoderStatus_t
typedef struct frag_session_data_s
{
    frag_group_data_t     frag_group_data;
    frag_decoder_status_t frag_decoder_status;
    int32_t               frag_decoder_process_status;

} frag_session_data_t;
uint8_t                       nb_transmit_ans = 1;
static frag_session_data_t    frag_session_data[FRAGMENTATION_MAX_NB_SESSIONS];
static FragDecoderCallbacks_t frag_decoder_callback;
static int8_t                 frag_decoder_write( uint32_t addr, uint8_t* data, uint32_t size );
static int8_t                 frag_decoder_read( uint32_t addr, uint8_t* data, uint32_t size );
static uint8_t                compute_data_block_integrity_ckeck( uint8_t frag_index, uint8_t stack_id );

/* -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lorawan_fragmentation_package_ctx_t lorawan_fragmentation_package_ctx[NUMBER_OF_FRAGMENTED_PACKAGE_OBJ];
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static void lorawan_fragmentation_add_task( uint8_t service_id );
/**
 * @brief parse the command received from the application server
 *
 * @param [in] ctx
 * @param [in] fragmentation_package_rx_buffer        buffer that will be decoded
 * @param [in] fragmentation_package_rx_buffer_length buffer length
 * @param [in] fragmentation_package_                 window type to filter unicast/multicast
 * @return frag_status_t
 */
static frag_status_t fragmentation_package_parser( lorawan_fragmentation_package_ctx_t* ctx,
                                                   uint8_t*                             fragmentation_package_rx_buffer,
                                                   uint8_t       fragmentation_package_rx_buffer_length,
                                                   receive_win_t fragmentation_package_rx_window, uint8_t stack_id );
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lorawan_fragmentation_package_services_init( uint8_t* service_id, uint8_t task_id,
                                                  uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                                  void ( **on_launch_callback )( void* ),
                                                  void ( **on_update_callback )( void* ), void** context_callback )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG(
        " lorawan_fragmentation_package_services_init task_id %d, service_id %d, CURRENT_STACK:%d \n", task_id,
        *service_id, CURRENT_STACK );

    IS_VALID_OBJECT_ID( *service_id );

    memset( &lorawan_fragmentation_package_ctx[*service_id], 0, sizeof( lorawan_fragmentation_package_ctx_t ) );

    *downlink_callback  = lorawan_fragmentation_package_service_downlink_handler;
    *on_launch_callback = lorawan_fragmentation_package_service_on_launch;
    *on_update_callback = lorawan_fragmentation_package_service_on_update;
    *context_callback   = ( void* ) service_id;

    lorawan_fragmentation_package_ctx[*service_id].task_id  = task_id;
    lorawan_fragmentation_package_ctx[*service_id].stack_id = CURRENT_STACK;
    lorawan_fragmentation_package_ctx[*service_id].enabled  = true;
    frag_decoder_callback.FragDecoderWrite                  = frag_decoder_write;
    frag_decoder_callback.FragDecoderRead                   = frag_decoder_read;
    for( int i = 0; i < FRAGMENTATION_MAX_NB_SESSIONS; i++ )
    {
        frag_session_data[i].frag_group_data.session_cnt = 0xFFFF;
    }
    // fragmentation_package_init( &lorawan_fragmentation_package_ctx[*service_id] );
}

void lorawan_fragmentation_package_service_on_launch( void* service_id )
{
    lorawan_api_payload_send(
        FRAGMENTATION_PORT, true,
        lorawan_fragmentation_package_ctx[*( uint8_t* ) service_id].fragmentation_tx_payload_ans,
        lorawan_fragmentation_package_ctx[*( uint8_t* ) service_id].fragmentation_tx_payload_ans_size, UNCONF_DATA_UP,
        smtc_modem_hal_get_time_in_ms( ) +
            smtc_modem_hal_get_random_nb_in_range(
                0, lorawan_fragmentation_package_ctx[*( uint8_t* ) service_id].fragmentation_ans_delay ),
        lorawan_fragmentation_package_ctx[*( uint8_t* ) service_id].stack_id );
}

void lorawan_fragmentation_package_service_on_update( void* service_id )
{
    if( nb_transmit_ans > 1 )
    {
        nb_transmit_ans--;
        lorawan_fragmentation_add_task( *( uint8_t* ) service_id );
    }
}

uint8_t lorawan_fragmentation_package_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    uint8_t stack_id = rx_down_data->stack_id;

    uint8_t                              service_id;
    lorawan_fragmentation_package_ctx_t* ctx = NULL;
    for( uint8_t i = 0; i < NUMBER_OF_FRAGMENTED_PACKAGE_OBJ; i++ )
    {
        if( lorawan_fragmentation_package_ctx[i].stack_id == stack_id )
        {
            ctx        = &lorawan_fragmentation_package_ctx[i];
            service_id = i;
            break;
        }
    }

    if( ctx == NULL )
    {
        return false;
    }

    if( stack_id >= NUMBER_OF_FRAGMENTED_PACKAGE_OBJ )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "stack id not valid %u \n", stack_id );
        return false;
    }

    if( ctx->enabled != true )
    {
        return false;
    }

    if( ( rx_down_data->rx_metadata.rx_fport_present == true ) &&
        ( rx_down_data->rx_metadata.rx_fport == FRAGMENTATION_PORT ) )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "lorawan_fragmentation_package_service_downlink_handler receive data on port %d\n",
                                     FRAGMENTATION_PORT );
        frag_status_t frag_status =
            fragmentation_package_parser( ctx, rx_down_data->rx_payload, rx_down_data->rx_payload_size,
                                          rx_down_data->rx_metadata.rx_window, stack_id );
        // check if answer have to been transmit
        if( ( frag_status == FRAG_STATUS_OK ) && ( ctx->fragmentation_tx_payload_ans_size > 0 ) )
        {
            lorawan_fragmentation_add_task( service_id );
        }
        return true;
    }

    return false;
}
void lorawan_fragmentation_package_service_get_id( uint8_t* pkt_id, uint8_t* pkt_version, uint8_t* pkt_port )
{
    *pkt_id      = FRAGMENTATION_ID;
    *pkt_version = FRAGMENTATION_VERSION;
    *pkt_port    = FRAGMENTATION_PORT;
}
bool lorawan_fragmentation_package_service_mpa_injector( uint8_t stack_id, uint8_t* payload_in, receive_win_t rx_window,
                                                         uint8_t* payload_out, uint8_t* payload_out_length )
{
    lorawan_fragmentation_package_ctx_t* ctx = NULL;
    *payload_out_length                      = 0;
    for( uint8_t i = 0; i < NUMBER_OF_FRAGMENTED_PACKAGE_OBJ; i++ )
    {
        if( lorawan_fragmentation_package_ctx[i].stack_id == stack_id )
        {
            ctx = &lorawan_fragmentation_package_ctx[i];
            break;
        }
    }

    if( ctx == NULL )
    {
        return false;
    }

    if( stack_id >= NUMBER_OF_FRAGMENTED_PACKAGE_OBJ )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "stack id not valid %u \n", stack_id );
        return false;
    }

    if( ctx->enabled != true )
    {
        return false;
    }
    // check cmd id to find cmd_id length special case for data_fragment
    uint8_t cmd_id = payload_in[0];
    if( cmd_id <= FRAGMENTATION_DATA_BLOCK_RECEIVED_ANS )
    {
        frag_status_t frag_status =
            fragmentation_package_parser( ctx, payload_in, frag_req_cmd_size[cmd_id], rx_window, stack_id );
        if( ( frag_status == FRAG_STATUS_OK ) && ( ctx->fragmentation_tx_payload_ans_size > 0 ) )
        {
            *payload_out_length = ctx->fragmentation_tx_payload_ans_size;
            memcpy( payload_out, ctx->fragmentation_tx_payload_ans, ctx->fragmentation_tx_payload_ans_size );
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_ERROR(
            " MPA Package implementation doesn't support to receive fragments , Fragment have to be received directly "
            "by the fragmantation package itself\n" );
        return false;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lorawan_fragmentation_add_task( uint8_t service_id )
{
    IS_VALID_OBJECT_ID( service_id );

    smodem_task task       = { 0 };
    task.id                = lorawan_fragmentation_package_ctx[service_id].task_id;
    task.stack_id          = lorawan_fragmentation_package_ctx[service_id].stack_id;
    task.priority          = TASK_HIGH_PRIORITY;
    task.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + smtc_modem_hal_get_random_nb_in_range( 0, 3 );
    if( modem_supervisor_add_task( &task ) != TASK_VALID )
    {
        SMTC_MODEM_HAL_PANIC( "Task not valid\n" );
    }
}

static frag_status_t fragmentation_package_parser( lorawan_fragmentation_package_ctx_t* ctx,
                                                   uint8_t*                             fragmentation_package_rx_buffer,
                                                   uint8_t       fragmentation_package_rx_buffer_length,
                                                   receive_win_t fragmentation_package_rx_window, uint8_t stack_id )
{
    uint8_t fragmentation_package_rx_buffer_index = 0;
    uint8_t ans_index                             = 0;
    nb_transmit_ans                               = 1;
    ctx->fragmentation_tx_payload_ans_size        = 0;
    ctx->fragmentation_ans_delay                  = 0;

    while( fragmentation_package_rx_buffer_length > fragmentation_package_rx_buffer_index )
    {
        switch( fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index] )
        {
        case FRAGMENTATION_PKG_VERSION_REQ: {
            IS_VALID_PKG_CMD( FRAGMENTATION_PKG_VERSION_REQ_SIZE );
            fragmentation_package_rx_buffer_index += FRAGMENTATION_PKG_VERSION_REQ_SIZE;
            if( fragmentation_package_rx_window > RECEIVE_ON_RXC )
            {
                // Multicast channel. Don't process command.
                break;
            }
            ctx->fragmentation_tx_payload_ans[ans_index++] = FRAGMENTATION_PKG_VERSION_ANS;
            ctx->fragmentation_tx_payload_ans[ans_index++] = FRAGMENTATION_ID;
            ctx->fragmentation_tx_payload_ans[ans_index++] = FRAGMENTATION_VERSION;
            break;
        }

        case FRAGMENTATION_SESSION_STATUS_REQ: {
            IS_VALID_PKG_CMD( FRAGMENTATION_SESSION_STATUS_REQ_SIZE );
            uint8_t frag_index   = fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 1];
            uint8_t participants = frag_index & 0x01;
            fragmentation_package_rx_buffer_index += FRAGMENTATION_SESSION_STATUS_REQ_SIZE;
            frag_index >>= 1;
            frag_session_data[frag_index].frag_decoder_status = FragDecoderGetStatus( );
            uint16_t nb_frag_received = frag_session_data[frag_index].frag_decoder_status.FragNbRx -
                                        frag_session_data[frag_index].frag_decoder_status.FragNbLost;

            if( ( participants == 1 ) ||
                ( ( participants == 0 ) && ( frag_session_data[frag_index].frag_decoder_status.FragNbLost > 0 ) ) )
            {
                uint8_t status = 0;
                if( frag_session_data[frag_index].frag_group_data.is_active == false )
                {
                    status = 0x04;
                }
                status += ( frag_session_data[frag_index].frag_decoder_status.MatrixError & 0x01 );
                if( frag_session_data[frag_index].frag_decoder_status.MissingFrag == 0 )
                {
                    status += ( compute_data_block_integrity_ckeck( frag_index, ctx->stack_id ) << 1 );
                }
                ctx->fragmentation_tx_payload_ans[ans_index++] = FRAGMENTATION_SESSION_STATUS_ANS;
                ctx->fragmentation_tx_payload_ans[ans_index++] = status;
                ctx->fragmentation_tx_payload_ans[ans_index++] = nb_frag_received & 0xFF;
                ctx->fragmentation_tx_payload_ans[ans_index++] =
                    ( frag_index << 6 ) | ( ( nb_frag_received >> 8 ) & 0x3F );
                ctx->fragmentation_tx_payload_ans[ans_index++] =
                    frag_session_data[frag_index].frag_decoder_status.MissingFrag;

                // Fetch the co-efficient value required to calculate delay of that respective session.
                ctx->fragmentation_ans_delay =
                    1 << ( frag_session_data[frag_index].frag_group_data.control.block_ack_delay + 4 );
            }
            break;
        }
        case FRAGMENTATION_SESSION_SETUP_REQ: {
            IS_VALID_PKG_CMD( FRAGMENTATION_SESSION_SETUP_REQ_SIZE );

            if( fragmentation_package_rx_window > RECEIVE_ON_RXC )
            {
                fragmentation_package_rx_buffer_index += FRAGMENTATION_SESSION_SETUP_REQ_SIZE;
                // Multicast channel. Don't process command.
                break;
            }

            frag_session_data_t frag_session_data_tmp = { 0 };

            // smtc_modem_hal_context_restore( CONTEXT_FUOTA, 0, (uint8_t
            // *)(&(frag_session_data_tmp.frag_group_data.session_cnt)), 2); // restore session_counter

            uint8_t status = 0x00;

            frag_session_data_tmp.frag_group_data.frag_session.mc_group_bit_mask =
                fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 1] & 0xF;
            frag_session_data_tmp.frag_group_data.frag_session.frag_index =
                fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 1] & 0x30;
            frag_session_data_tmp.frag_group_data.frag_nb =
                ( fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 2] << 0 ) & 0x00FF;
            frag_session_data_tmp.frag_group_data.frag_nb |=
                ( fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 3] << 8 ) & 0xFF00;
            frag_session_data_tmp.frag_group_data.frag_size =
                fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 4];
            frag_session_data_tmp.frag_group_data.control.block_ack_delay =
                fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 5] & 0x7;
            frag_session_data_tmp.frag_group_data.control.frag_algo =
                fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 5] & 0x38;
            frag_session_data_tmp.frag_group_data.control.ack_reception =
                fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 5] & 0x40;
            frag_session_data_tmp.frag_group_data.padding =
                fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 6];
            frag_session_data_tmp.frag_group_data.descriptor =
                ( fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 7] << 0 ) & 0x000000FF;
            frag_session_data_tmp.frag_group_data.descriptor +=
                ( fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 8] << 8 ) & 0x0000FF00;
            frag_session_data_tmp.frag_group_data.descriptor +=
                ( fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 9] << 16 ) & 0x00FF0000;
            frag_session_data_tmp.frag_group_data.descriptor +=
                ( fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 10] << 24 ) & 0xFF000000;
            frag_session_data_tmp.frag_group_data.session_cnt +=
                ( fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 11] << 0 ) & 0x000000FF;
            frag_session_data_tmp.frag_group_data.session_cnt +=
                ( fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 12] << 8 ) & 0x0000FF00;
            frag_session_data_tmp.frag_group_data.mic =
                ( fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 13] << 0 ) & 0x000000FF;
            frag_session_data_tmp.frag_group_data.mic +=
                ( fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 14] << 8 ) & 0x0000FF00;
            frag_session_data_tmp.frag_group_data.mic +=
                ( fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 15] << 16 ) & 0x00FF0000;
            frag_session_data_tmp.frag_group_data.mic +=
                ( fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 16] << 24 ) & 0xFF000000;

            fragmentation_package_rx_buffer_index += FRAGMENTATION_SESSION_SETUP_REQ_SIZE;
            SMTC_MODEM_HAL_TRACE_PRINTF( " \nfrag_nb = %d , \nfrag_size = %d \n ",
                                         frag_session_data_tmp.frag_group_data.frag_nb,
                                         frag_session_data_tmp.frag_group_data.frag_size );
            if( frag_session_data_tmp.frag_group_data.control.frag_algo > 0 )
            {
                status |= 0x01;  // Encoding unsupported
            }

            if( ( frag_session_data_tmp.frag_group_data.frag_nb > FRAG_MAX_NB ) ||
                ( frag_session_data_tmp.frag_group_data.frag_size > FRAG_MAX_SIZE ) ||
                ( ( frag_session_data_tmp.frag_group_data.frag_nb * frag_session_data_tmp.frag_group_data.frag_size ) >
                  FragDecoderGetMaxFileSize( ) ) )
            {
                status |= 0x02;  // Not enough Memory
            }

            status |= ( frag_session_data_tmp.frag_group_data.frag_session.frag_index << 6 ) & 0xC0;
            if( frag_session_data_tmp.frag_group_data.frag_session.frag_index >= FRAGMENTATION_MAX_NB_SESSIONS )
            {
                status |= 0x04;  // frag_session index not supported
            }

            // Descriptor is not really defined in the specification
            // Not clear how to handle this.
            // Currently the descriptor is always correct
            if( frag_session_data_tmp.frag_group_data.descriptor != 0x01020304 )
            {
                // status |= 0x08; // Wrong Descriptor
            }
            if( frag_session_data[frag_session_data_tmp.frag_group_data.frag_session.frag_index]
                    .frag_group_data.session_cnt >= frag_session_data_tmp.frag_group_data.session_cnt )
            {
                status |= 0x10;  // session replay
            }
            if( ( status & 0x0F ) == 0 )
            {
                // The frag_sessionSetup is accepted
                //   smtc_modem_hal_context_store( CONTEXT_FUOTA, 0,  (uint8_t
                //   *)(&(frag_session_data_tmp.frag_group_data.session_cnt)), 2 );
                frag_session_data_tmp.frag_group_data.is_active   = true;
                frag_session_data_tmp.frag_decoder_process_status = FRAG_SESSION_ONGOING;
                frag_session_data[frag_session_data_tmp.frag_group_data.frag_session.frag_index] =
                    frag_session_data_tmp;

                FragDecoderInit( frag_session_data_tmp.frag_group_data.frag_nb,
                                 frag_session_data_tmp.frag_group_data.frag_size, &frag_decoder_callback );
            }
            ctx->fragmentation_tx_payload_ans[ans_index++] = FRAGMENTATION_SESSION_SETUP_ANS;
            ctx->fragmentation_tx_payload_ans[ans_index++] = status;

            break;
        }
        case FRAGMENTATION_SESSION_DELETE_REQ: {
            IS_VALID_PKG_CMD( FRAGMENTATION_SESSION_DELETE_REQ_SIZE );
            if( fragmentation_package_rx_window > RECEIVE_ON_RXC )
            {
                fragmentation_package_rx_buffer_index += FRAGMENTATION_SESSION_DELETE_REQ_SIZE;
                break;
            }
            uint8_t status = 0x00;
            uint8_t id     = fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 1] & 0x03;
            fragmentation_package_rx_buffer_index += FRAGMENTATION_SESSION_DELETE_REQ_SIZE;
            status |= id;
            if( ( id >= FRAGMENTATION_MAX_NB_SESSIONS ) ||
                ( frag_session_data[id].frag_group_data.is_active == false ) )
            {
                status |= 0x04;  // Session does not exist
            }
            else
            {
                // Delete session
                frag_session_data[id].frag_group_data.is_active = false;
            }
            ctx->fragmentation_tx_payload_ans[ans_index++] = FRAGMENTATION_SESSION_DELETE_ANS;
            ctx->fragmentation_tx_payload_ans[ans_index++] = status;

            break;
        }
        case FRAGMENTATION_DATA_FRAGMENT: {
            uint8_t  frag_index   = 0;
            uint16_t frag_counter = 0;

            frag_counter = ( fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 1] ) & 0x00FF;
            frag_counter |=
                ( fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 2] << 8 ) & 0xFF00;

            frag_index = ( frag_counter >> 14 ) & 0x03;
            frag_counter &= 0x3FFF;

            if( frag_session_data[frag_index].frag_decoder_process_status == FRAG_SESSION_ONGOING )
            {
                SMTC_MODEM_HAL_TRACE_ARRAY( "FUOTA FRAG = ", fragmentation_package_rx_buffer,
                                            fragmentation_package_rx_buffer_length );
                frag_session_data[frag_index].frag_decoder_process_status = FragDecoderProcess(
                    frag_counter, &fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 3] );
                frag_session_data[frag_index].frag_decoder_status = FragDecoderGetStatus( );
                SMTC_MODEM_HAL_TRACE_PRINTF(
                    "\nFuota session info : \nCurrent fragment index = %d,\nCurrent fragment counter = %d,\nNumber of "
                    "missed packets = %d,\n",
                    frag_index, frag_session_data[frag_index].frag_decoder_status.FragNbRx,
                    frag_session_data[frag_index].frag_decoder_status.FragNbLost );
            }
            if( frag_session_data[frag_index].frag_decoder_process_status >= FRAG_SESSION_FINISHED_SUCCESSFULLY )
            {
                if( frag_session_data[frag_index].frag_decoder_process_status == FRAG_SESSION_FINISHED_SUCCESSFULLY )
                {
                    if( frag_session_data[frag_index].frag_group_data.control.ack_reception == 1 )
                    {
                        uint8_t status_tmp =
                            ( frag_index & 0x3 ) +
                            ( ( compute_data_block_integrity_ckeck( frag_index, ctx->stack_id ) << 2 ) & 0x4 );
                        ctx->fragmentation_tx_payload_ans[ans_index++] = FRAGMENTATION_DATA_BLOCK_RECEIVED_REQ;
                        ctx->fragmentation_tx_payload_ans[ans_index++] = status_tmp;
                        nb_transmit_ans                                = 3;
                    }
                    SMTC_MODEM_HAL_TRACE_PRINTF( " FILE RECONSTRUCTS SUCCESSFULLY !\n" );
                }
                else
                {
                    SMTC_MODEM_HAL_TRACE_PRINTF( " FUOTA SESSION FAILED !\n" );
                }
                uint8_t status = ( frag_session_data[frag_index].frag_decoder_process_status < 255 )
                                     ? frag_session_data[frag_index].frag_decoder_process_status
                                     : 255;
                increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_LORAWAN_FUOTA_DONE, status, stack_id );
                frag_session_data[frag_index].frag_decoder_process_status = FRAG_SESSION_NOT_STARTED;
                frag_session_data[frag_index].frag_group_data.is_active   = false;
            }
            fragmentation_package_rx_buffer_index += frag_session_data[frag_index].frag_group_data.frag_size + 3;
            break;
        }
        case FRAGMENTATION_DATA_BLOCK_RECEIVED_ANS: {
            // uint8_t  frag_index   = 0;
            // frag_index = ( fragmentation_package_rx_buffer[fragmentation_package_rx_buffer_index + 1] ) & 0x3;
            //
            break;
        }
        }
    }
    // If this message was received on a multicast address, the end-device MUST check that the
    // multicast address used was enabled at the creation of the fragmentation session through the
    // McGroupBitMask field of the FragSessionSetup command. If not, the frame SHALL be
    // silently dropped.

    ctx->fragmentation_tx_payload_ans_size = ans_index;

    return FRAG_STATUS_OK;
}

static int8_t frag_decoder_write( uint32_t addr, uint8_t* data, uint32_t size )
{
    smtc_modem_hal_context_store( CONTEXT_FUOTA, addr, data, size );  //
    return 0;
}

static int8_t frag_decoder_read( uint32_t addr, uint8_t* data, uint32_t size )
{
    smtc_modem_hal_context_restore( CONTEXT_FUOTA, addr, data, size );
    return 0;
}

typedef struct soft_se_key_s
{
    smtc_se_key_identifier_t key_id;                       //!< Key identifier
    uint8_t                  key_value[SMTC_SE_KEY_SIZE];  //!< Key value
} soft_se_key_t;

static uint8_t compute_data_block_integrity_ckeck( uint8_t frag_index, uint8_t stack_id )
{
    uint8_t b0[16]  = { 0 };
    uint8_t key[16] = { 0 };
    // Derive key
    b0[0] = 0x30;

    if( smtc_secure_element_aes_encrypt( b0, 16, SMTC_SE_APP_KEY, key, stack_id ) != SMTC_SE_RC_SUCCESS )
    {
        return 0;
    }

    uint32_t size = frag_session_data[frag_index].frag_group_data.frag_nb *
                        frag_session_data[frag_index].frag_group_data.frag_size -
                    frag_session_data[frag_index].frag_group_data.padding;

    b0[0] = 0x49;
    b0[1] = frag_session_data[frag_index].frag_group_data.session_cnt & 0xFF;
    b0[2] = ( frag_session_data[frag_index].frag_group_data.session_cnt >> 8 ) & 0xFF;

    b0[3] = frag_index;

    b0[4] = frag_session_data[frag_index].frag_group_data.descriptor & 0xFF;
    b0[5] = ( frag_session_data[frag_index].frag_group_data.descriptor >> 8 ) & 0xFF;
    b0[6] = ( frag_session_data[frag_index].frag_group_data.descriptor >> 16 ) & 0xFF;
    b0[7] = ( frag_session_data[frag_index].frag_group_data.descriptor >> 24 ) & 0xFF;

    b0[8]  = 0x00;
    b0[9]  = 0x00;
    b0[10] = 0x00;
    b0[11] = 0x00;

    b0[12] = size & 0xFF;
    b0[13] = ( size >> 8 ) & 0xFF;
    b0[14] = ( size >> 16 ) & 0xFF;
    b0[15] = ( size >> 24 ) & 0xFF;

    // Compute received data block MIC
    AES_CMAC_CTX aes_cmac_ctx;
    uint8_t      cmac[16];
    uint8_t      d = 0;

    AES_CMAC_Init( &aes_cmac_ctx );
    AES_CMAC_SetKey( &aes_cmac_ctx, key );
    AES_CMAC_Update( &aes_cmac_ctx, b0, 16 );
    for( uint32_t i = 0; i < size; i++ )
    {
        frag_decoder_read( i, &d, 1 );
        AES_CMAC_Update( &aes_cmac_ctx, &d, 1 );
    }
    AES_CMAC_Final( cmac, &aes_cmac_ctx );
    uint32_t computed_mic = ( uint32_t ) ( ( uint32_t ) cmac[3] << 24 | ( uint32_t ) cmac[2] << 16 |
                                           ( uint32_t ) cmac[1] << 8 | ( uint32_t ) cmac[0] );

    uint32_t target_mic = frag_session_data[frag_index].frag_group_data.mic;
    SMTC_MODEM_HAL_TRACE_PRINTF( "computed mic = %x\n and target mic = %x\n", computed_mic, target_mic );
    if( computed_mic == target_mic )
    {
        return 0;
    }
    else
    {
        return 1;
    }
}
/* --- EOF ------------------------------------------------------------------ */
