/**
 * @file      store_and_forward.c
 *
 * @brief     Device management for Semtech cloud service
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
#include "fifo_ctrl.h"
#include "modem_core.h"
#include "smtc_modem_api.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "modem_supervisor_light.h"
#include "lorawan_api.h"
#include "modem_event_utilities.h"
#include "device_management_defs.h"
#include "store_and_forward.h"

#if defined( USE_LR11XX_CE )
#include "lr11xx_system.h"
#endif  // USE_LR11XX_CE

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )
#define NUMBER_MAX_OF_STORE_AND_FORWARD_OBJ 1  // modify in case of multiple obj

#define MODEM_TASK_DELAY_MS ( smtc_modem_hal_get_random_nb_in_range( 200, 3000 ) )

/**
 * @brief Check is the index is valid before accessing ALCSync object
 *
 */
#define IS_VALID_OBJECT_ID( x )                                                     \
    do                                                                              \
    {                                                                               \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_MAX_OF_STORE_AND_FORWARD_OBJ ); \
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
 * @brief Check is the service is initialized before accessing the object
 *
 */
#define IS_SERVICE_INITIALIZED( x )                                                            \
    do                                                                                         \
    {                                                                                          \
        if( store_and_forward_obj[x].initialized == false )                                    \
        {                                                                                      \
            SMTC_MODEM_HAL_TRACE_WARNING( "store_and_forward_obj service not initialized\n" ); \
            return;                                                                            \
        }                                                                                      \
    } while( 0 )

/**
 * @brief Fifo size
 *
 */
#define STORE_AND_FORWARD_FIFO_SIZE ( 2048 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief  Fifo metadata structure
 */
typedef struct store_and_forward_metadata_s
{
    uint8_t  fport;
    bool     confirmed;
    uint32_t lifetime_s;
    uint32_t timestamp_s;
} store_and_forward_metadata_t;

/**
 * @brief LoRaWAN template Object
 *
 * @struct store_and_forward_s
 *
 */
typedef struct store_and_forward_s
{
    uint8_t stack_id;
    uint8_t task_id;
    bool    enabled;
    bool    initialized;

    fifo_ctrl_t fifo_ctrl_obj;
    uint8_t     fifo_buffer[STORE_AND_FORWARD_FIFO_SIZE];

    uint8_t                      sending_data[SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH];
    uint16_t                     sending_data_len;
    store_and_forward_metadata_t sending_metadata;
    uint8_t                      sending_metadata_len;
    uint32_t                     sending_first_try_timestamp_s;
    uint32_t                     sending_try_cpt;
    uint32_t                     sending_periodicity;
} store_and_forward_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static store_and_forward_t store_and_forward_obj[NUMBER_MAX_OF_STORE_AND_FORWARD_OBJ];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief   Enqueue task in supervisor
 *
 * @param [in] ctx                  stream object context
 * @param [in] delay_to_execute_s   duration before the next execution of this task
 */
static void store_and_forward_add_task( store_and_forward_t* ctx, uint32_t delay_to_execute_s );

/**
 * @brief Callback called at task launch
 *
 * @param context_callback
 */
static void store_and_forward_service_on_launch( void* context );

/**
 * @brief Callback called at the end of the task
 *
 * @param context_callback
 */
static void store_and_forward_service_on_update( void* context );

/**
 * @brief Callback to handle the downlink received by the LoRaWAN layer
 *
 * @param rx_down_data
 */
static uint8_t store_and_forward_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/**
 * @brief Get the service context linked to a stack_id
 *
 * @param [in]  stack_id                Stack identifier
 * @param [out] service_id              associated occurrence of this service
 * @return store_and_forward_t*                  Cloud dm object context
 */
static store_and_forward_t* store_and_forward_get_ctx_from_stack_id( uint8_t stack_id, uint8_t* service_id );

/**
 * @brief Compute the next delay to send the next tentative
 *
 * @param ctx
 * @return uint32_t
 */
static uint32_t store_and_forward_compute_next_delay_s( store_and_forward_t* ctx );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void store_and_forward_services_init( uint8_t* service_id, uint8_t task_id,
                                      uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                      void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                                      void** context_callback )
{
    IS_VALID_OBJECT_ID( *service_id );

    store_and_forward_t* ctx = &store_and_forward_obj[*service_id];
    memset( ctx, 0, sizeof( store_and_forward_t ) );

    *downlink_callback  = store_and_forward_service_downlink_handler;
    *on_launch_callback = store_and_forward_service_on_launch;
    *on_update_callback = store_and_forward_service_on_update;
    *context_callback   = ( void* ) service_id;

    ctx->task_id             = task_id;
    ctx->stack_id            = CURRENT_STACK;
    ctx->enabled             = false;
    ctx->initialized         = true;
    ctx->sending_periodicity = 0;

    fifo_ctrl_init( &ctx->fifo_ctrl_obj, &ctx->fifo_buffer, STORE_AND_FORWARD_FIFO_SIZE );
}

void store_and_forward_services_enable( uint8_t stack_id, bool enabled )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t              service_id;
    store_and_forward_t* ctx = store_and_forward_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
    {
        return;
    }

    if( ctx->enabled != enabled )
    {
        ctx->enabled = enabled;

        if( enabled == true )
        {
            if( lorawan_api_isjoined( ctx->stack_id ) == JOINED )
            {
                store_and_forward_add_task( ctx, smtc_modem_hal_get_random_nb_in_range( 5, 10 ) );
            }
        }
        else
        {
            modem_supervisor_remove_task( ctx->task_id );
        }
    }
}

store_and_forward_rc_t store_and_forward_add_data( uint8_t fport, const uint8_t* data, uint8_t data_len, bool confirmed,
                                                   uint32_t lifetime_s, uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t              service_id;
    store_and_forward_t* ctx = store_and_forward_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx->enabled == false )
    {
        return STORE_AND_FORWARD_ERROR;
    }

    if( ( data_len == 0 ) || ( data_len > SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH ) )
    {
        return STORE_AND_FORWARD_ERROR;
    }

    if( ( fport == 0 ) || ( fport >= 224 ) )
    {
        return STORE_AND_FORWARD_ERROR;
    }

    store_and_forward_metadata_t metadata = { 0 };
    metadata.fport                        = fport;
    metadata.confirmed                    = confirmed;
    metadata.lifetime_s                   = lifetime_s;
    metadata.timestamp_s                  = smtc_modem_hal_get_time_in_s( );

    uint16_t fifo_free_space = fifo_ctrl_get_free_space( &ctx->fifo_ctrl_obj );
    if( ( data_len + sizeof( store_and_forward_metadata_t ) + 3 ) > fifo_free_space )  // +3 needed by internal fifo
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "store and forward oldest data will be lost\n" );
    }

    if( fifo_ctrl_set( &ctx->fifo_ctrl_obj, data, data_len, &metadata, sizeof( store_and_forward_metadata_t ) ) !=
        FIFO_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "store and forward fifo problem\n" );
        return STORE_AND_FORWARD_ERROR;
    }

    fifo_ctrl_print_stat( &ctx->fifo_ctrl_obj );

    uint16_t nb_of_data = fifo_ctrl_get_nb_elt( &ctx->fifo_ctrl_obj );
    // first data added, launch the service for the first sending, then it will be automatic in update task
    if( nb_of_data == 1 )
    {
        if( lorawan_api_isjoined( ctx->stack_id ) == JOINED )
        {
            store_and_forward_add_task( ctx, 0 );
        }
    }

    SMTC_MODEM_HAL_TRACE_PRINTF( "store and forward nb_of_data in fifo %u\n", nb_of_data );

    return STORE_AND_FORWARD_OK;
}

void store_and_forward_clear_data( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t              service_id;
    store_and_forward_t* ctx = store_and_forward_get_ctx_from_stack_id( stack_id, &service_id );
    fifo_ctrl_clear( &ctx->fifo_ctrl_obj );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void store_and_forward_service_on_launch( void* service_id )
{
    uint8_t idx = *( ( uint8_t* ) service_id );
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " %s service_id %d \n", __func__, idx );

    IS_SERVICE_INITIALIZED( idx );
    uint8_t stack_id = store_and_forward_obj[idx].stack_id;

    // If not join -> exit
    if( lorawan_api_isjoined( stack_id ) != JOINED )
    {
        return;
    }

    // If service not enabled -> exit
    if( store_and_forward_obj[idx].enabled == false )
    {
        return;
    }

    uint32_t rtc_ms = smtc_modem_hal_get_time_in_ms( )  ;

    // TODO check payload len and adjust the datarate with custom profile
    uint8_t max_payload = lorawan_api_next_max_payload_length_get( stack_id );

    // get_nb of data in fifo
    uint16_t nb_of_data = fifo_ctrl_get_nb_elt( &store_and_forward_obj[idx].fifo_ctrl_obj );

    if( ( store_and_forward_obj[idx].sending_data_len == 0 ) && ( nb_of_data > 0 ) )
    {
        fifo_ctrl_get( &store_and_forward_obj[idx].fifo_ctrl_obj, store_and_forward_obj[idx].sending_data,
                       &store_and_forward_obj[idx].sending_data_len, SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH,
                       &store_and_forward_obj[idx].sending_metadata, &store_and_forward_obj[idx].sending_metadata_len,
                       sizeof( store_and_forward_metadata_t ) );

        store_and_forward_obj[idx].sending_first_try_timestamp_s = rtc_ms / 1000;
        store_and_forward_obj[idx].sending_try_cpt               = 0;
        SMTC_MODEM_HAL_TRACE_WARNING( "store en fw get new data\n" );
    }

    if( store_and_forward_obj[idx].sending_data_len > 0 )
    {
        status_lorawan_t send_status = tx_protocol_manager_request (TX_PROTOCOL_TRANSMIT_LORA,
            store_and_forward_obj[idx].sending_metadata.fport, true, store_and_forward_obj[idx].sending_data,
            store_and_forward_obj[idx].sending_data_len,
            ( store_and_forward_obj[idx].sending_metadata.confirmed == true ) ? CONF_DATA_UP : UNCONF_DATA_UP, rtc_ms,
            stack_id );

        if( send_status == OKLORAWAN )
        {
            store_and_forward_obj[idx].sending_try_cpt++;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_WARNING( " %s service_id %d data not send 0x%x\n", __func__, idx, send_status );
        }
        SMTC_MODEM_HAL_TRACE_WARNING( "store en fw try (%u) to send \n", store_and_forward_obj[idx].sending_try_cpt );
    }
}

static void store_and_forward_service_on_update( void* service_id )
{
    uint8_t idx = *( ( uint8_t* ) service_id );
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " %s service_id %d \n", __func__, idx );

    IS_SERVICE_INITIALIZED( idx );
    uint8_t stack_id = store_and_forward_obj[idx].stack_id;

    if( lorawan_api_isjoined( stack_id ) != JOINED )
    {
        return;
    }
    if( store_and_forward_obj[idx].enabled == false )
    {
        return;
    }

    uint16_t nb_of_data = fifo_ctrl_get_nb_elt( &store_and_forward_obj[idx].fifo_ctrl_obj );

    if( store_and_forward_obj[idx].sending_data_len != 0 )
    {
        store_and_forward_obj[idx].sending_periodicity++;
        SMTC_MODEM_HAL_TRACE_WARNING( "store en fw data not acked sending_periodicity:%u\n",
                                      store_and_forward_obj[idx].sending_periodicity );
    }

    if( ( nb_of_data > 0 ) || store_and_forward_obj[idx].sending_data_len != 0 )
    {
        uint32_t delay_tmp = store_and_forward_compute_next_delay_s( &store_and_forward_obj[idx] );

        SMTC_MODEM_HAL_TRACE_WARNING( "store en fw data delay_tmp:%u\n", delay_tmp );
        store_and_forward_add_task( &store_and_forward_obj[idx], delay_tmp );
    }
}

static uint8_t store_and_forward_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    uint8_t              stack_id = rx_down_data->stack_id;
    uint8_t              service_id;
    store_and_forward_t* ctx = store_and_forward_get_ctx_from_stack_id( stack_id, &service_id );

    task_id_t current_task_id = modem_supervisor_get_task( )->next_task_id;
    // If a downlink is received outside the running service, the downlink is not an ack but there is network coverage
    if( current_task_id != ctx->task_id )
    {
        uint16_t nb_of_data = fifo_ctrl_get_nb_elt( &ctx->fifo_ctrl_obj );

        if( ( ctx->sending_data_len > 0 ) || ( nb_of_data > 0 ) )
        {
            store_and_forward_add_task( ctx, MODEM_TASK_DELAY_MS );
        }
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( ( rx_down_data->rx_metadata.rx_window != RECEIVE_NONE ) && ( ctx->sending_data_len != 0 ) &&
        ( ( ( rx_down_data->rx_metadata.rx_ack_bit == true ) && ( ctx->sending_metadata.confirmed == true ) ) ||
          ( ctx->sending_metadata.confirmed == false ) ) )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "%s\n", __func__ );
        SMTC_MODEM_HAL_TRACE_PRINTF( "uplink acked after %u tentative\n", ctx->sending_try_cpt );

        // reset the data len to inform the packet has been acked by the network
        ctx->sending_data_len    = 0;
        ctx->sending_periodicity = 0;
        return MODEM_DOWNLINK_CONSUMED;
    }

    return MODEM_DOWNLINK_UNCONSUMED;
}

static store_and_forward_t* store_and_forward_get_ctx_from_stack_id( uint8_t stack_id, uint8_t* service_id )
{
    store_and_forward_t* ctx = NULL;
    for( uint8_t i = 0; i < NUMBER_MAX_OF_STORE_AND_FORWARD_OBJ; i++ )
    {
        if( store_and_forward_obj[i].stack_id == stack_id )
        {
            ctx         = &store_and_forward_obj[i];
            *service_id = i;
            break;
        }
    }

    return ctx;
}

static void store_and_forward_add_task( store_and_forward_t* ctx, uint32_t delay_to_execute_s )
{
    if( ctx->enabled == false )
    {
        return;
    }
    smodem_task task_dm       = { 0 };
    task_dm.id                = ctx->task_id;
    task_dm.stack_id          = ctx->stack_id;
    task_dm.priority          = TASK_LOW_PRIORITY;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + delay_to_execute_s;

    modem_supervisor_add_task( &task_dm );
}

static uint32_t store_and_forward_compute_next_delay_s( store_and_forward_t* ctx )
{
    // TODO implement the right algorithm to compute the delay between each retry to send the same packet not acked
    uint32_t delay_s = ctx->sending_periodicity * smtc_modem_hal_get_random_nb_in_range( 1, 5 );
    return delay_s;
}

/* --- EOF ------------------------------------------------------------------ */
