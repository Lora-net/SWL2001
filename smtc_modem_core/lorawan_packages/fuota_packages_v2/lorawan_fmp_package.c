/**
 * @file      lorawan_fmp_package.c
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
#include "lorawan_fmp_package.h"
#include "modem_core.h"
#include "smtc_modem_api.h"

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

#define NUMBER_OF_FMP_PACKAGE_OBJ 1
/**
 * @brief Compute current LoRaWAN Stack from the supervisor task_id
 *
 */
#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )
#define CURRENT_TASK_CONTEXT \
    ( modem_supervisor_get_task( ) )->modem_task[( modem_supervisor_get_task( ) )->next_task_id].task_context
/**
 * @brief Check if the index is valid
 *
 */
#define IS_VALID_OBJECT_ID( x )                                   \
    do                                                            \
    {                                                             \
        if( x >= NUMBER_OF_FMP_PACKAGE_OBJ )                      \
        {                                                         \
            SMTC_MODEM_HAL_PANIC( "not valid service_id %d", x ); \
        }                                                         \
    } while( 0 )

#define IS_VALID_PKG_CMD( x )                                                    \
    do                                                                           \
    {                                                                            \
        if( ( fmp_package_rx_buffer_index + x ) > fmp_package_rx_buffer_length ) \
        {                                                                        \
            SMTC_MODEM_HAL_TRACE_ERROR( "%u\n", fmp_package_rx_buffer_length );  \
            return FMP_STATUS_ERROR;                                             \
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

#define FMP_PORT 203
#define FMP_ID 4
#define FMP_VERSION 1

// Request message sizes (with header)

#define FMP_PKG_VERSION_REQ_SIZE ( 1 )
#define FMP_DEV_VERSION_REQ_SIZE ( 1 )
#define FMP_DEV_REBOOT_TIME_REQ_SIZE ( 5 )
#define FMP_DEV_REBOOT_COUNT_DOWN_REQ_SIZE ( 4 )
#define FMP_DEV_UPGRADE_IMAGE_REQ_SIZE ( 1 )
#define FMP_DEV_DELETE_IMAGE_REQ_SIZE ( 5 )

#define FMP_PKG_VERSION_ANS_SIZE ( 3 )
#define FMP_DEV_VERSION_ANS_SIZE ( 9 )
#define FMP_DEV_REBOOT_TIME_ANS_SIZE ( 5 )
#define FMP_DEV_REBOOT_COUNT_DOWN_ANS_SIZE ( 4 )
#define FMP_DEV_UPGRADE_IMAGE_ANS_SIZE ( 2 )
#define FMP_DEV_DELETE_IMAGE_ANS_SIZE ( 2 )

#define FMP_SIZE_ANS_MAX FMP_DEV_VERSION_ANS_SIZE
#define VALID_FIRMWARE_UPGRADE_IMAGE ( 3 )
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum fmp_package_ans_e
{
    FMP_PKG_VERSION_ANS           = 0x00,
    FMP_DEV_VERSION_ANS           = 0x01,
    FMP_DEV_REBOOT_TIME_ANS       = 0x02,
    FMP_DEV_REBOOT_COUNT_DOWN_ANS = 0x03,
    FMP_DEV_UPGRADE_IMAGE_ANS     = 0x04,
    FMP_DEV_DELETE_IMAGE_ANS      = 0x05,
} fmp_package_ans_t;

typedef enum fmp_package_req_e
{
    FMP_PKG_VERSION_REQ           = 0x00,
    FMP_DEV_VERSION_REQ           = 0x01,
    FMP_DEV_REBOOT_TIME_REQ       = 0x02,
    FMP_DEV_REBOOT_COUNT_DOWN_REQ = 0x03,
    FMP_DEV_UPGRADE_IMAGE_REQ     = 0x04,
    FMP_DEV_DELETE_IMAGE_REQ      = 0x05,
} fmp_package_req_t;

static uint8_t fmp_req_cmd_size[6] = { FMP_PKG_VERSION_REQ_SIZE,       FMP_DEV_VERSION_REQ_SIZE,
                                       FMP_DEV_REBOOT_TIME_REQ_SIZE,   FMP_DEV_REBOOT_COUNT_DOWN_REQ_SIZE,
                                       FMP_DEV_UPGRADE_IMAGE_REQ_SIZE, FMP_DEV_DELETE_IMAGE_REQ_SIZE };
typedef enum
{
    EMPTY_TASK_MASK             = 0,
    ANS_CMD_TASK_MASK           = 0x1,
    REQUEST_TIME_SYNC_TASK_MASK = 0x2,
    REQUEST_REBOOT_TASK_MASK    = 0x4,

} fmp_supervisor_task_types_t;

typedef struct lorawan_fmp_package_s
{
    uint8_t stack_id;
    uint8_t task_id;
    // Uplink buffer ans
    uint8_t                     fmp_tx_payload_ans[FMP_SIZE_ANS_MAX];
    uint8_t                     fmp_tx_payload_ans_size;
    uint16_t                    fmp_ans_delay;
    uint32_t                    fmp_rtc_target_time_for_reboot;
    uint32_t                    fmp_fw_to_delete_version;
    fmp_supervisor_task_types_t fmp_task_ctx_mask;
    fmp_supervisor_task_types_t fmp_current_task_ctx;
    bool                        request_time_sync;
    bool                        enabled;
} lorawan_fmp_package_ctx_t;

typedef enum
{
    FMP_STATUS_OK,
    FMP_STATUS_ERROR
} fmp_status_t;

/* -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lorawan_fmp_package_ctx_t lorawan_fmp_package_ctx[NUMBER_OF_FMP_PACKAGE_OBJ];
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static void fmp_add_task( lorawan_fmp_package_ctx_t* ctx, uint32_t rtc_target_s );
/**
 * @brief parse the command received from the application server
 *
 * @param [in] ctx
 * @param [in] fmp_package_rx_buffer        buffer that will be decoded
 * @param [in] fmp_package_rx_buffer_length buffer length
 * @param [in] fmp_package_rx_window        window type to filter unicast/multicast
 * @return frag_status_t
 */
static fmp_status_t fmp_package_parser( lorawan_fmp_package_ctx_t* ctx, uint8_t* fmp_package_rx_buffer,
                                        uint8_t fmp_package_rx_buffer_length, uint8_t stack_id, uint8_t* event_status,
                                        bool* increment_event );
static bool         get_gps_time( uint32_t* gps_time_s, uint8_t stack_id );
#if defined( FUOTA_BUILT_IN_TEST )
static bool         fmp_test( void );
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lorawan_fmp_package_services_init( uint8_t* service_id, uint8_t task_id,
                                        uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                        void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                                        void** context_callback )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG(
        " lorawan_fmp_package_services_init task_id %d, service_id %d, CURRENT_STACK:%d \n", task_id, *service_id,
        CURRENT_STACK );

    IS_VALID_OBJECT_ID( *service_id );

    memset( &lorawan_fmp_package_ctx[*service_id], 0, sizeof( lorawan_fmp_package_ctx_t ) );

    *downlink_callback  = lorawan_fmp_package_service_downlink_handler;
    *on_launch_callback = lorawan_fmp_package_service_on_launch;
    *on_update_callback = lorawan_fmp_package_service_on_update;
    *context_callback   = ( void* ) &( lorawan_fmp_package_ctx[*service_id] );

    lorawan_fmp_package_ctx[*service_id].task_id  = task_id;
    lorawan_fmp_package_ctx[*service_id].stack_id = CURRENT_STACK;
    lorawan_fmp_package_ctx[*service_id].enabled  = true;

#if defined( FUOTA_BUILT_IN_TEST )
    fmp_test( );
#endif
}

void lorawan_fmp_package_service_on_launch( void* service_id )
{
    lorawan_fmp_package_ctx_t* ctx      = ( lorawan_fmp_package_ctx_t* ) service_id;
    uint8_t                    stack_id = ctx->stack_id;

    if( ( ctx->fmp_task_ctx_mask & ANS_CMD_TASK_MASK ) == ANS_CMD_TASK_MASK )
    {
        ctx->fmp_current_task_ctx = ANS_CMD_TASK_MASK;
        SMTC_MODEM_HAL_TRACE_PRINTF( " lorawan_fmp_package launch ANS_CMD_TASK n" );
        if( ctx->fmp_tx_payload_ans_size > 0 )
        {
            lorawan_api_payload_send(
                FMP_PORT, true, ctx->fmp_tx_payload_ans, ctx->fmp_tx_payload_ans_size, UNCONF_DATA_UP,
                smtc_modem_hal_get_time_in_ms( ) + smtc_modem_hal_get_random_nb_in_range( 0, ctx->fmp_ans_delay ),
                ctx->stack_id );
            ctx->fmp_tx_payload_ans_size = 0;
        }
        return;
    }
    if( ( ctx->fmp_task_ctx_mask & REQUEST_TIME_SYNC_TASK_MASK ) == REQUEST_TIME_SYNC_TASK_MASK )
    {
        ctx->fmp_current_task_ctx = REQUEST_TIME_SYNC_TASK_MASK;
        SMTC_MODEM_HAL_TRACE_PRINTF( " lorawan_fmp_package launch REQUEST_TIME_SYNC_TASK n" );
        ctx->request_time_sync             = false;
        cid_from_device_t cid_buffer[]     = { DEVICE_TIME_REQ };
        uint8_t           cid_request_size = 1;
        lorawan_api_send_stack_cid_req( cid_buffer, cid_request_size, stack_id );
        return;
    }
    if( ( ctx->fmp_task_ctx_mask & REQUEST_REBOOT_TASK_MASK ) == REQUEST_REBOOT_TASK_MASK )
    {
        ctx->fmp_current_task_ctx = REQUEST_REBOOT_TASK_MASK;
        SMTC_MODEM_HAL_TRACE_PRINTF( " lorawan_fmp_package launch REQUEST_REBOOT_TASK_MASK n" );

        return;
    }
}

void lorawan_fmp_package_service_on_update( void* service_id )
{
    lorawan_fmp_package_ctx_t* ctx = ( lorawan_fmp_package_ctx_t* ) service_id;
    switch( ctx->fmp_current_task_ctx )
    {
    case ANS_CMD_TASK_MASK: {
        ctx->fmp_task_ctx_mask &= ~( ANS_CMD_TASK_MASK );
        break;
    }
    case REQUEST_TIME_SYNC_TASK_MASK: {
        // by choice of implementation time is requested only one time
        ctx->fmp_task_ctx_mask &= ~( REQUEST_TIME_SYNC_TASK_MASK );
        break;
    }
    case REQUEST_REBOOT_TASK_MASK: {
        if( ( int ) ( smtc_modem_hal_get_time_in_s( ) - ctx->fmp_rtc_target_time_for_reboot ) < 2 )
        {
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_FIRMWARE_MANAGEMENT,
                                              SMTC_MODEM_EVENT_FMP_REBOOT_IMMEDIATELY, ctx->stack_id );
            ctx->fmp_task_ctx_mask &=
                ~( REQUEST_REBOOT_TASK_MASK );  // don't remove this task if the target time isn't yet reach
        }
        break;
    }
    default:
        break;
    }
    if( ctx->fmp_task_ctx_mask != EMPTY_TASK_MASK )
    {
        uint32_t rtc_target_tmp = ( ctx->fmp_task_ctx_mask == REQUEST_REBOOT_TASK_MASK )
                                      ? ctx->fmp_rtc_target_time_for_reboot
                                      : smtc_modem_hal_get_time_in_s( );
        fmp_add_task( ctx, rtc_target_tmp );
    }
}
uint8_t lorawan_fmp_package_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    uint8_t                    stack_id = rx_down_data->stack_id;
    lorawan_fmp_package_ctx_t* ctx      = NULL;
    for( uint8_t i = 0; i < NUMBER_OF_FMP_PACKAGE_OBJ; i++ )
    {
        if( lorawan_fmp_package_ctx[i].stack_id == stack_id )
        {
            ctx = &lorawan_fmp_package_ctx[i];
            break;
        }
    }

    if( ctx == NULL )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( stack_id >= NUMBER_OF_FMP_PACKAGE_OBJ )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "stack id not valid %u \n", stack_id );
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( ctx->enabled != true )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( ( rx_down_data->rx_metadata.rx_fport_present == true ) && ( rx_down_data->rx_metadata.rx_fport == FMP_PORT ) )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "lorawan_fmp_package_service_downlink_handler receive data on port %d\n",
                                     FMP_PORT );
        uint8_t      event_status;
        bool         increment_event;
        fmp_status_t fmp_status = fmp_package_parser( ctx, rx_down_data->rx_payload, rx_down_data->rx_payload_size,
                                                      stack_id, &event_status, &increment_event );
        // check if answer have to been transmit
        if( ( fmp_status == FMP_STATUS_OK ) && ( ctx->fmp_task_ctx_mask != EMPTY_TASK_MASK ) )
        {
            fmp_add_task( ctx, smtc_modem_hal_get_time_in_s( ) );
        }
        if( increment_event == true )
        {
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_FIRMWARE_MANAGEMENT, event_status, stack_id );
        }
        task_id_t current_task_id = modem_supervisor_get_task( )->next_task_id;

        if( current_task_id != ctx->task_id )
        {
            // To generate event in case receive downlink when another service is running
            lorawan_fmp_package_service_on_update( ctx );
        }
        return MODEM_DOWNLINK_CONSUMED;
    }

    return MODEM_DOWNLINK_UNCONSUMED;
}

void lorawan_fmp_package_service_get_id( uint8_t* pkt_id, uint8_t* pkt_version, uint8_t* pkt_port )
{
    *pkt_id      = FMP_ID;
    *pkt_version = FMP_VERSION;
    *pkt_port    = FMP_PORT;
}
bool lorawan_fmp_mpa_injector( uint8_t stack_id, uint8_t* payload_in, receive_win_t rx_window, uint8_t* payload_out,
                               uint8_t* payload_out_length )
{
    *payload_out_length            = 0;
    lorawan_fmp_package_ctx_t* ctx = NULL;
    for( uint8_t i = 0; i < NUMBER_OF_FMP_PACKAGE_OBJ; i++ )
    {
        if( lorawan_fmp_package_ctx[i].stack_id == stack_id )
        {
            ctx = &lorawan_fmp_package_ctx[i];
            break;
        }
    }

    if( ctx == NULL )
    {
        return false;
    }

    if( stack_id >= NUMBER_OF_FMP_PACKAGE_OBJ )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "stack id not valid %u \n", stack_id );
        return false;
    }

    if( ctx->enabled != true )
    {
        return false;
    }
    uint8_t      cmd_id = payload_in[0];
    uint8_t      event_status;
    bool         increment_event;
    fmp_status_t fmp_status =
        fmp_package_parser( ctx, payload_in, fmp_req_cmd_size[cmd_id], stack_id, &event_status, &increment_event );
    // check if answer have to been transmit
    if( ( fmp_status == FMP_STATUS_OK ) && ( ctx->fmp_task_ctx_mask == ANS_CMD_TASK_MASK ) )
    {
        *payload_out_length = ctx->fmp_tx_payload_ans_size;
        memcpy( payload_out, ctx->fmp_tx_payload_ans, ctx->fmp_tx_payload_ans_size );
        return true;
    }
    if( ( fmp_status == FMP_STATUS_OK ) && ( ctx->fmp_task_ctx_mask != EMPTY_TASK_MASK ) &&
        ( ctx->fmp_task_ctx_mask != ANS_CMD_TASK_MASK ) )
    {
        fmp_add_task( ctx, smtc_modem_hal_get_time_in_s( ) );
    }
    if( increment_event == true )
    {
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_FIRMWARE_MANAGEMENT, event_status, stack_id );
    }
    task_id_t current_task_id = modem_supervisor_get_task( )->next_task_id;

    if( current_task_id != ctx->task_id )
    {
        // To generate event in case receive downlink when another service is running
        lorawan_fmp_package_service_on_update( ctx );
    }
    return false;
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void fmp_add_task( lorawan_fmp_package_ctx_t* ctx, uint32_t rtc_target_s )

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

static fmp_status_t fmp_package_parser( lorawan_fmp_package_ctx_t* ctx, uint8_t* fmp_package_rx_buffer,
                                        uint8_t fmp_package_rx_buffer_length, uint8_t stack_id, uint8_t* event_status,
                                        bool* increment_event )
{
    uint8_t  fmp_package_rx_buffer_index = 0;
    uint8_t  ans_index                   = 0;
    uint32_t gps_time_s;
    *increment_event             = false;
    ctx->fmp_tx_payload_ans_size = 0;
    ctx->fmp_ans_delay           = 0;
    ctx->fmp_task_ctx_mask |= REQUEST_REBOOT_TASK_MASK;
    while( fmp_package_rx_buffer_length > fmp_package_rx_buffer_index )
    {
        switch( fmp_package_rx_buffer[fmp_package_rx_buffer_index] )
        {
        case FMP_PKG_VERSION_REQ: {
            IS_VALID_PKG_CMD( FMP_PKG_VERSION_REQ_SIZE );
            fmp_package_rx_buffer_index += FMP_PKG_VERSION_REQ_SIZE;

            ctx->fmp_tx_payload_ans[ans_index++] = FMP_PKG_VERSION_ANS;
            ctx->fmp_tx_payload_ans[ans_index++] = FMP_ID;
            ctx->fmp_tx_payload_ans[ans_index++] = FMP_VERSION;
            ctx->fmp_tx_payload_ans_size         = FMP_PKG_VERSION_ANS_SIZE;
            break;
        }

        case FMP_DEV_VERSION_REQ: {
            IS_VALID_PKG_CMD( FMP_DEV_VERSION_REQ_SIZE );
            fmp_package_rx_buffer_index += FMP_DEV_VERSION_REQ_SIZE;
            uint32_t hw_version                  = smtc_modem_hal_get_hw_version_for_fuota( );
            uint32_t fw_version                  = smtc_modem_hal_get_fw_version_for_fuota( );
            ctx->fmp_tx_payload_ans[ans_index++] = FMP_DEV_VERSION_ANS;
            ctx->fmp_tx_payload_ans[ans_index++] = fw_version & 0xFF;
            ctx->fmp_tx_payload_ans[ans_index++] = ( fw_version >> 8 ) & 0xFF;
            ctx->fmp_tx_payload_ans[ans_index++] = ( fw_version >> 16 ) & 0xFF;
            ctx->fmp_tx_payload_ans[ans_index++] = ( fw_version >> 24 ) & 0xFF;
            ctx->fmp_tx_payload_ans[ans_index++] = hw_version & 0xFF;
            ctx->fmp_tx_payload_ans[ans_index++] = ( hw_version >> 8 ) & 0xFF;
            ctx->fmp_tx_payload_ans[ans_index++] = ( hw_version >> 16 ) & 0xFF;
            ctx->fmp_tx_payload_ans[ans_index++] = ( hw_version >> 24 ) & 0xFF;
            ctx->fmp_tx_payload_ans_size         = FMP_DEV_VERSION_ANS_SIZE;
            break;
        }
        case FMP_DEV_REBOOT_TIME_REQ: {
            IS_VALID_PKG_CMD( FMP_DEV_REBOOT_TIME_REQ_SIZE );
            uint32_t fmp_reboot_time = fmp_package_rx_buffer[fmp_package_rx_buffer_index + 1] +
                                       ( fmp_package_rx_buffer[fmp_package_rx_buffer_index + 2] << 8 ) +
                                       ( fmp_package_rx_buffer[fmp_package_rx_buffer_index + 3] << 16 ) +
                                       ( fmp_package_rx_buffer[fmp_package_rx_buffer_index + 4] << 24 );
            fmp_package_rx_buffer_index += FMP_DEV_REBOOT_TIME_REQ_SIZE;

            if( fmp_reboot_time == 0 )
            {
                // in this case no response

                *increment_event = true;
                *event_status    = SMTC_MODEM_EVENT_FMP_REBOOT_IMMEDIATELY;
                ctx->fmp_task_ctx_mask &= ~( REQUEST_REBOOT_TASK_MASK );
            }
            if( fmp_reboot_time == 0xFFFFFFFF )
            {
                *increment_event                     = true;
                *event_status                        = SMTC_MODEM_EVENT_FMP_CANCEL_REBOOT;
                ctx->fmp_tx_payload_ans[ans_index++] = FMP_DEV_REBOOT_TIME_ANS;
                ctx->fmp_tx_payload_ans[ans_index++] = 0xFF;
                ctx->fmp_tx_payload_ans[ans_index++] = 0xFF;
                ctx->fmp_tx_payload_ans[ans_index++] = 0xFF;
                ctx->fmp_tx_payload_ans[ans_index++] = 0xFF;
                ctx->fmp_tx_payload_ans_size         = FMP_DEV_REBOOT_TIME_ANS_SIZE;
            }
            if( ( fmp_reboot_time > 0 ) && ( fmp_reboot_time < 0xFFFFFFFF ) )
            {
                if( get_gps_time( &gps_time_s, stack_id ) == true )
                {
                    if( fmp_reboot_time > gps_time_s )
                    {  // Reboot time field indicates a time in future
                        ctx->fmp_task_ctx_mask |= REQUEST_REBOOT_TASK_MASK;
                        uint32_t temp                        = fmp_reboot_time - gps_time_s;
                        ctx->fmp_rtc_target_time_for_reboot  = smtc_modem_hal_get_time_in_s( ) + temp;
                        ctx->fmp_tx_payload_ans[ans_index++] = FMP_DEV_REBOOT_TIME_ANS;
                        ctx->fmp_tx_payload_ans[ans_index++] = temp & 0xFF;
                        ctx->fmp_tx_payload_ans[ans_index++] = ( temp >> 8 ) & 0xFF;
                        ctx->fmp_tx_payload_ans[ans_index++] = ( temp >> 16 ) & 0xFF;
                        ctx->fmp_tx_payload_ans[ans_index++] = ( temp >> 24 ) & 0xFF;
                        ctx->fmp_tx_payload_ans_size         = FMP_DEV_REBOOT_TIME_ANS_SIZE;
                    }
                    else
                    {  // Reboot time field indicates a time in past
                        ctx->fmp_tx_payload_ans[ans_index++] = FMP_DEV_REBOOT_TIME_ANS;
                        ctx->fmp_tx_payload_ans[ans_index++] = 0;
                        ctx->fmp_tx_payload_ans[ans_index++] = 0;
                        ctx->fmp_tx_payload_ans[ans_index++] = 0;
                        ctx->fmp_tx_payload_ans[ans_index++] = 0;
                        ctx->fmp_tx_payload_ans_size         = FMP_DEV_REBOOT_TIME_ANS_SIZE;
                    }
                }
                else  // gps time isn't available
                {
                    ctx->fmp_task_ctx_mask |= REQUEST_TIME_SYNC_TASK_MASK;
                    ctx->fmp_tx_payload_ans[ans_index++] = FMP_DEV_REBOOT_TIME_ANS;
                    ctx->fmp_tx_payload_ans[ans_index++] = 0;
                    ctx->fmp_tx_payload_ans[ans_index++] = 0;
                    ctx->fmp_tx_payload_ans[ans_index++] = 0;
                    ctx->fmp_tx_payload_ans[ans_index++] = 0;
                    ctx->fmp_tx_payload_ans_size         = FMP_DEV_REBOOT_TIME_ANS_SIZE;
                }
            }
            break;
        }
        case FMP_DEV_REBOOT_COUNT_DOWN_REQ: {
            IS_VALID_PKG_CMD( FMP_DEV_REBOOT_COUNT_DOWN_REQ_SIZE );
            uint32_t fmp_count_down = fmp_package_rx_buffer[fmp_package_rx_buffer_index + 1] +
                                      ( fmp_package_rx_buffer[fmp_package_rx_buffer_index + 2] << 8 ) +
                                      ( fmp_package_rx_buffer[fmp_package_rx_buffer_index + 3] << 16 );

            fmp_package_rx_buffer_index += FMP_DEV_REBOOT_COUNT_DOWN_REQ_SIZE;

            if( fmp_count_down == 0 )
            {
                // in this case no response

                *increment_event = true;
                *event_status    = SMTC_MODEM_EVENT_FMP_REBOOT_IMMEDIATELY;
                ctx->fmp_task_ctx_mask &= ~( REQUEST_REBOOT_TASK_MASK );
            }
            if( fmp_count_down == 0xFFFFFF )
            {
                *increment_event                     = true;
                *event_status                        = SMTC_MODEM_EVENT_FMP_CANCEL_REBOOT;
                ctx->fmp_tx_payload_ans[ans_index++] = FMP_DEV_REBOOT_COUNT_DOWN_ANS;
                ctx->fmp_tx_payload_ans[ans_index++] = 0xFF;
                ctx->fmp_tx_payload_ans[ans_index++] = 0xFF;
                ctx->fmp_tx_payload_ans[ans_index++] = 0xFF;

                ctx->fmp_tx_payload_ans_size = FMP_DEV_REBOOT_COUNT_DOWN_ANS_SIZE;
            }
            if( ( fmp_count_down > 0 ) && ( fmp_count_down < 0xFFFFFF ) )
            {
                // Reboot time field indicates a time in future

                ctx->fmp_task_ctx_mask |= REQUEST_REBOOT_TASK_MASK;
                ctx->fmp_rtc_target_time_for_reboot  = smtc_modem_hal_get_time_in_s( ) + fmp_count_down;
                ctx->fmp_tx_payload_ans[ans_index++] = FMP_DEV_REBOOT_COUNT_DOWN_ANS;
                ctx->fmp_tx_payload_ans[ans_index++] = fmp_count_down & 0xFF;
                ctx->fmp_tx_payload_ans[ans_index++] = ( fmp_count_down >> 8 ) & 0xFF;
                ctx->fmp_tx_payload_ans[ans_index++] = ( fmp_count_down >> 16 ) & 0xFF;
                ctx->fmp_tx_payload_ans_size         = FMP_DEV_REBOOT_COUNT_DOWN_ANS_SIZE;
            }
            break;
        }
        case FMP_DEV_UPGRADE_IMAGE_REQ: {
            IS_VALID_PKG_CMD( FMP_DEV_UPGRADE_IMAGE_REQ_SIZE );
            fmp_package_rx_buffer_index += FMP_DEV_UPGRADE_IMAGE_REQ_SIZE;

            ctx->fmp_tx_payload_ans[ans_index++] = FMP_DEV_UPGRADE_IMAGE_ANS;
            ctx->fmp_tx_payload_ans[ans_index++] = smtc_modem_hal_get_fw_status_available_for_fuota( );
            ctx->fmp_tx_payload_ans_size         = FMP_DEV_UPGRADE_IMAGE_ANS_SIZE;
            if( smtc_modem_hal_get_fw_status_available_for_fuota( ) == VALID_FIRMWARE_UPGRADE_IMAGE )
            {
                uint32_t next_fw_verion              = smtc_modem_hal_get_next_fw_version_for_fuota( );
                ctx->fmp_tx_payload_ans[ans_index++] = next_fw_verion & 0xFF;
                ctx->fmp_tx_payload_ans[ans_index++] = ( next_fw_verion >> 8 ) & 0xFF;
                ctx->fmp_tx_payload_ans[ans_index++] = ( next_fw_verion >> 16 ) & 0xFF;
                ctx->fmp_tx_payload_ans[ans_index++] = ( next_fw_verion >> 24 ) & 0xFF;
                ctx->fmp_tx_payload_ans_size         = FMP_DEV_UPGRADE_IMAGE_ANS_SIZE + 4;
            }
            break;
        }
        case FMP_DEV_DELETE_IMAGE_REQ: {
            IS_VALID_PKG_CMD( FMP_DEV_DELETE_IMAGE_REQ_SIZE );
            ctx->fmp_fw_to_delete_version = fmp_package_rx_buffer[fmp_package_rx_buffer_index + 1] +
                                            ( fmp_package_rx_buffer[fmp_package_rx_buffer_index + 2] << 8 ) +
                                            ( fmp_package_rx_buffer[fmp_package_rx_buffer_index + 3] << 16 ) +
                                            ( fmp_package_rx_buffer[fmp_package_rx_buffer_index + 4] << 24 );
            fmp_package_rx_buffer_index += FMP_DEV_DELETE_IMAGE_REQ_SIZE;
            ctx->fmp_tx_payload_ans[ans_index++] = FMP_DEV_DELETE_IMAGE_ANS;
            ctx->fmp_tx_payload_ans[ans_index++] =
                smtc_modem_hal_get_fw_delete_status_for_fuota( ctx->fmp_fw_to_delete_version );
            ctx->fmp_tx_payload_ans_size = FMP_DEV_DELETE_IMAGE_ANS_SIZE;

            break;
        }
        }
    }
    // If this message was received on a multicast address, the end-device MUST check that the
    // multicast address used was enabled at the creation of the fragmentation session through the
    // McGroupBitMask field of the FragSessionSetup command. If not, the frame SHALL be
    // silently dropped.

    ctx->fmp_tx_payload_ans_size = ans_index;

    return FMP_STATUS_OK;
}

static bool get_gps_time( uint32_t* gps_time_s, uint8_t stack_id )
{
    uint32_t gps_fractional_s;

    if( lorawan_api_is_time_valid( stack_id ) == true )
    {
        lorawan_api_convert_rtc_to_gps_epoch_time( smtc_modem_hal_get_time_in_ms( ), gps_time_s, &gps_fractional_s,
                                                   stack_id );
        return true;
    }
    else
    {
        return ( ( lorawan_alcsync_get_gps_time_second( stack_id, gps_time_s ) == ALC_SYNC_OK ) ? true : false );
    }
}

#if defined( FUOTA_BUILT_IN_TEST )
/* add static fonction for tests purpose */
static bool fmp_test( void )
{
    uint8_t test_vector_input[][6] = {
        { 1, 0 },                          // get package version
        { 1, 1 },                          // get dev version
        { 5, 2, 0, 0, 0, 0 },              // test reboot time with reboot time = 0
        { 5, 2, 0xFF, 0xFF, 0xFF, 0xFF },  // test reboot time with reboot time = 0xFFFFFFFF
        { 5, 2, 0, 0, 0xDB, 0x50 },        // test reboot time with valid GPS time but no time in the device
        { 4, 3, 0, 0, 0, 0 },              // test reboot countdown  with count down time = 0
        { 4, 3, 0xFF, 0xFF, 0xFF },        // test reboot countdown  with count down time = 0xFFFFFFFF
        { 4, 3, 0, 0, 17 },                // test reboot countdown  with count down time = 17s
        { 1, 4 },                          // get next fw version
        { 5, 5, 1, 2, 3, 4 },              // delete image with bad firmware version
        { 5, 5, 0x17, 0x01, 0x19, 0x73 },  // delete image with good firmware version
    };
    uint8_t test_vector_output[][10] = {
        { 3, 0, 4, 1 },
        { 9, 1, 0x44, 0x33, 0x22, 0x11, 0x78, 0x56, 0x34, 0x12 },
        { 0 },                 // Device shouldn't answer in case of reboot time = 0
        { 5, 2, 0xFF, 0xFF, 0xFF, 0xFF },
        { 5, 2, 0, 0, 0, 0 },  // reboot time is valid but device isn't clk sync => response = 0
        { 0 },                 // Device shouldn't answer in case of reboot time = 0
        { 4, 3, 0xFF, 0xFF, 0xFF },
        { 4, 3, 0, 0, 17 },    // reboot time is valid but device isn't clk sync => response = 0
        { 6, 4, 3, 0x17, 0x01, 0x19, 0x73 },
        { 2, 5, 2 },
        { 2, 5, 0 },
    };
    uint8_t test_vector_event_output[][2] = {
        { false, 0 },
        { false, 0 },
        { true, SMTC_MODEM_EVENT_FMP_REBOOT_IMMEDIATELY },
        { true, SMTC_MODEM_EVENT_FMP_CANCEL_REBOOT },
        { false, 0 },
        { true, SMTC_MODEM_EVENT_FMP_REBOOT_IMMEDIATELY },
        { true, SMTC_MODEM_EVENT_FMP_CANCEL_REBOOT },
        { false, 0 },
        { false, 0 },
        { false, 0 },
        { false, 0 },
    };

    uint8_t                    nb_errors = 0;
    lorawan_fmp_package_ctx_t* ctx       = NULL;
    uint8_t                    stack_id  = 0;
    for( uint8_t i = 0; i < NUMBER_OF_FMP_PACKAGE_OBJ; i++ )
    {
        if( lorawan_fmp_package_ctx[i].stack_id == stack_id )
        {
            ctx = &lorawan_fmp_package_ctx[i];
            break;
        }
    }
    uint8_t event_status;
    bool    increment_event;
    for( int i = 0; i < 11; i++ )
    {
        fmp_package_parser( ctx, &test_vector_input[i][1], test_vector_input[i][0], stack_id, &event_status,
                            &increment_event );
        if( ( test_vector_output[i][0] == 0 ) && ( ctx->fmp_tx_payload_ans_size > 0 ) )
        {
            nb_errors++;
            SMTC_MODEM_HAL_TRACE_ERROR( "fmp test failed\n" );
        }
        if( ( increment_event != test_vector_event_output[i][0] ) ||
            ( ( increment_event == true ) && ( event_status != test_vector_event_output[i][1] ) ) )
        {
            nb_errors++;
            SMTC_MODEM_HAL_TRACE_ERROR( "fmp test failed due to event\n" );
        }
        if( ( ( memcmp( ctx->fmp_tx_payload_ans, &test_vector_output[i][1], test_vector_output[i][0] ) ) ||
              ( test_vector_output[i][0] != ctx->fmp_tx_payload_ans_size ) ) &&
            ( test_vector_output[i][0] == 0 ) )
        {
            nb_errors++;
            SMTC_MODEM_HAL_TRACE_ERROR( "fmp test failed\n" );
        }
    }
    if( nb_errors == 0 )
    {
        SMTC_MODEM_HAL_TRACE_INFO( " FMP TEST OK \n" );
        return true;
    }
    else
    {
        return false;
    }
}
#endif
/* --- EOF ------------------------------------------------------------------ */
