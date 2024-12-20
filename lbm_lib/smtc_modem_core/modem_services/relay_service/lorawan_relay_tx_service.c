/**
 * @file    lorawan_relay_tx_service.c
 *
 * @brief   Relay TX service. Handle automatic activation of relay mode.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2023. All rights reserved.
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
#include "lorawan_relay_tx_service.h"
#include "modem_core.h"
#include "modem_supervisor_light.h"
#include "lorawan_send_management.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_api.h"
#include "modem_event_utilities.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "lorawan_api.h"

#include "relay_tx_api.h"
#include "relay_def.h"
#include "relay_real.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )
#define NUMBER_OF_RELAY_TX_PACKAGE_OBJ 1  // modify in case of multiple obj

/**
 * @brief Check is the index is valid before accessing ALCSync object
 *
 */
#define IS_VALID_OBJECT_ID( x )                                                \
    do                                                                         \
    {                                                                          \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_OF_RELAY_TX_PACKAGE_OBJ ); \
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

/**
 * @brief Check is the service is enabled before accessing the object
 *
 */
#define IS_SERVICE_enabled( )                                                     \
    do                                                                            \
    {                                                                             \
        if( relay_tx_ctx.enabled == false )                                       \
        {                                                                         \
            SMTC_MODEM_HAL_TRACE_WARNING( "relay_tx_ctx service not enabled\n" ); \
            return;                                                               \
        }                                                                         \
    } while( 0 )
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief LoRaWAN template Object
 *
 * @struct lorawan_relay_tx_s
 *
 */
typedef struct lorawan_relay_tx_ctx_s
{
    uint8_t stack_id;
    uint8_t task_id;
    bool    enabled;
    uint8_t spare[62];

} lorawan_relay_tx_ctx_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static lorawan_relay_tx_ctx_t relay_tx_ctx[NUMBER_OF_RELAY_TX_PACKAGE_OBJ] = { 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
/**
 * @brief Callback called at task launch
 *
 * @param context_callback
 */
static void lorawan_relay_tx_service_on_launch( void* context );

/**
 * @brief Callback called at the end of the task
 *
 * @param context_callback
 */
static void lorawan_relay_tx_service_on_update( void* context );

/**
 * @brief Callback to handle the downlink received by the LoRaWAN layer
 *
 * @param rx_down_data
 */
static uint8_t lorawan_relay_tx_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void lorawan_relay_tx_services_init( uint8_t* service_id, uint8_t task_id,
                                     uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                     void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                                     void** context_callback )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( " lorawan_relay_tx_services_init task_id %d, service_id %d, CURRENT_STACK:%d \n",
                                 task_id, *service_id, CURRENT_STACK );

    IS_VALID_OBJECT_ID( *service_id );

    lorawan_relay_tx_ctx_t* ctx = &relay_tx_ctx[*service_id];
    memset( ctx, 0, sizeof( lorawan_relay_tx_ctx_t ) );

    *downlink_callback  = lorawan_relay_tx_service_downlink_handler;
    *on_launch_callback = lorawan_relay_tx_service_on_launch;
    *on_update_callback = lorawan_relay_tx_service_on_update;
    *context_callback   = ( void* ) &( relay_tx_ctx[*service_id] );

    relay_tx_ctx[*service_id].task_id  = task_id;
    relay_tx_ctx[*service_id].stack_id = CURRENT_STACK;
    relay_tx_ctx[*service_id].enabled  = true;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lorawan_relay_tx_service_on_launch( void* context )
{
    // empty function
}

static void lorawan_relay_tx_service_on_update( void* context )
{
    // empty function
}

static uint8_t lorawan_relay_tx_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    const uint8_t stack_id = rx_down_data->stack_id;

    lorawan_relay_tx_ctx_t* ctx = NULL;
    for( uint8_t i = 0; i < NUMBER_OF_RELAY_TX_PACKAGE_OBJ; i++ )
    {
        if( relay_tx_ctx[i].stack_id == stack_id )
        {
            ctx = &relay_tx_ctx[i];
            break;
        }
    }

    if( ctx == NULL )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( stack_id >= NUMBER_OF_RELAY_TX_PACKAGE_OBJ )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "%s: stack id not valid %u \n", __func__, stack_id );
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( ctx->enabled != true )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    lr1_stack_mac_t*  lr1_ptr    = lorawan_api_stack_mac_get( ctx->stack_id );
    relay_tx_config_t relay_conf = { 0 };
    smtc_relay_tx_get_config( stack_id, &relay_conf );

    if( rx_down_data->rx_metadata.rx_window == RECEIVE_ON_RXR )
    {
        smtc_relay_tx_data_receive_on_rxr( stack_id );
    }

    if( relay_conf.activation == RELAY_TX_ACTIVATION_MODE_DYNAMIC )
    {
        if( rx_down_data->rx_metadata.rx_window == RECEIVE_NONE )
        {
            if( lr1_ptr->no_rx_packet_count >= relay_conf.smart_level )
            {
                if( smtc_relay_tx_is_enable( stack_id ) == false )
                {
                    smtc_relay_tx_enable( stack_id );
                    increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_RELAY_TX_DYNAMIC, 1, lr1_ptr->stack_id );
                }
            }
        }
        else if( rx_down_data->rx_metadata.rx_window != RECEIVE_ON_RXR )
        {
            if( smtc_relay_tx_is_enable( stack_id ) == true )
            {
                smtc_relay_tx_disable( stack_id );
                increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_RELAY_TX_DYNAMIC, 0, lr1_ptr->stack_id );
            }
        }
    }

    return MODEM_DOWNLINK_UNCONSUMED;
}

/* --- EOF ------------------------------------------------------------------ */
