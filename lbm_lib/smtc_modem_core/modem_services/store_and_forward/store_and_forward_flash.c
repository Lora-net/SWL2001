/**
 * @file      store_and_forward_flash.c
 *
 * @brief     Store data in FiFo NVM and send it when network is available
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
#include "circularfs.h"
#include "modem_core.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "modem_supervisor_light.h"
#include "lorawan_api.h"
#include "store_and_forward_flash.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Send all data as fast as possible without Rx1/Rx2 windows ;-)
 */
#define FIFO_BURST_SENDING ( false )

/**
 * @brief Version of data structure in FiFo
 */
#define LOG_ENTRY_VERSION ( 1 )

/**
 * @brief data length in byte in FiFo
 */
#define DATA_SIZE_MAX ( 51 )  // ( SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH )

/**
 * @brief Acknowledgment requested every N send
 */
#ifndef STORE_AND_FORWARD_ACK_PERIOD
#define STORE_AND_FORWARD_ACK_PERIOD ( 10 )
#endif

/**
 * @brief Delay maximum to try to retransmit a data
 */
#ifndef STORE_AND_FORWARD_DELAY_MAX_S
#define STORE_AND_FORWARD_DELAY_MAX_S ( 3600 )
#endif

#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )
#define NUMBER_MAX_OF_STORE_AND_FORWARD_OBJ 1  // modify in case of multiple obj

#if( FIFO_BURST_SENDING == true )
#define MODEM_TASK_DELAY_MS ( 100 )
#else
#define MODEM_TASK_DELAY_MS ( smtc_modem_hal_get_random_nb_in_range( 200, 3000 ) )
#endif

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
#define IS_SERVICE_INITIALIZED( x )                                                                  \
    do                                                                                               \
    {                                                                                                \
        if( store_and_forward_flash_obj[x].initialized == false )                                    \
        {                                                                                            \
            SMTC_MODEM_HAL_TRACE_WARNING( "store_and_forward_flash_obj service not initialized\n" ); \
            return;                                                                                  \
        }                                                                                            \
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
 * @brief  Data structure stored in NVM slot
 */
typedef struct store_and_forward_data_s
{
    uint8_t  data[DATA_SIZE_MAX];
    uint8_t  data_len;
    uint8_t  fport;
    bool     confirmed;
    uint16_t crc;  // !! crc MUST be the last field of the structure !!

#if( ( ( DATA_SIZE_MAX + 5 ) % 8 ) != 0 )
    uint8_t padding[( ( DATA_SIZE_MAX + 5 ) % 8 )];
#endif
} store_and_forward_flash_data_t;

/**
 * @brief LoRaWAN template Object
 *
 * @struct store_and_forward_flash_s
 */
typedef struct store_and_forward_flash_s
{
    uint8_t                         stack_id;
    uint8_t                         task_id;
    store_and_forward_flash_state_t enabled;
    bool                            initialized;

    uint32_t sending_first_try_timestamp_s;
    uint32_t sending_try_cpt;
    bool     sending_with_ack;
    uint8_t  ack_period_count;

    struct circularfs fs;

} store_and_forward_flash_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static store_and_forward_flash_t         store_and_forward_flash_obj[NUMBER_MAX_OF_STORE_AND_FORWARD_OBJ];
static struct circularfs_flash_partition flash_obj;

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
static void store_and_forward_flash_add_task( store_and_forward_flash_t* ctx, uint32_t delay_to_execute_s );

/**
 * @brief Callback called at task launch
 *
 * @param context_callback
 */
static void store_and_forward_flash_service_on_launch( void* context );

/**
 * @brief Callback called at the end of the task
 *
 * @param context_callback
 */
static void store_and_forward_flash_service_on_update( void* context );

/**
 * @brief Callback to handle the downlink received by the LoRaWAN layer
 *
 * @param rx_down_data
 */
static uint8_t store_and_forward_flash_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/**
 * @brief Get the service context linked to a stack_id
 *
 * @param [in]  stack_id                Stack identifier
 * @param [out] service_id              associated occurrence of this service
 * @return store_and_forward_flash_t*   Object context
 */
static store_and_forward_flash_t* store_and_forward_flash_get_ctx_from_stack_id( uint8_t  stack_id,
                                                                                 uint8_t* service_id );

/**
 * @brief Compute the next delay to send the next tentative
 *
 * @param ctx
 * @return uint32_t
 */
static uint32_t store_and_forward_flash_compute_next_delay_s( store_and_forward_flash_t* ctx );

/**
 * @brief Erase sector in flash
 *
 * @param flash
 * @param address address of the sector
 * @return int
 */
static int32_t op_sector_erase( struct circularfs_flash_partition* flash, uint32_t address );

/**
 * @brief Write data in flash
 *
 * @param flash
 * @param address  address of requested data in flash
 * @param data  buffer containing data
 * @param size  buffer length
 * @return int
 */
static int32_t op_program( struct circularfs_flash_partition* flash, uint32_t address, const void* data,
                           uint32_t size );

/**
 * @brief Read data in flash
 *
 * @param flash
 * @param address address of requested data in flash
 * @param data buffer containing data
 * @param size buffer length
 * @return int
 */
static int32_t op_read( struct circularfs_flash_partition* flash, uint32_t address, void* data, uint32_t size );

/**
 * @brief Compute crc
 *
 * @param [in] buf          input buffer
 * @param [in] len          input buffer length
 * @return [out] uint32_t   computed crc
 */
static uint32_t crc_store_and_fwd( const uint8_t* buf, int len );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void store_and_forward_flash_services_init( uint8_t* service_id, uint8_t task_id,
                                            uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                            void ( **on_launch_callback )( void* ),
                                            void ( **on_update_callback )( void* ), void** context_callback )
{
    IS_VALID_OBJECT_ID( *service_id );

    flash_obj.sector_size  = smtc_modem_hal_flash_get_page_size( );
    flash_obj.sector_count = smtc_modem_hal_store_and_forward_get_number_of_pages( );
    flash_obj.sector_erase = op_sector_erase;
    flash_obj.program      = op_program;
    flash_obj.read         = op_read;

    store_and_forward_flash_t* ctx = &store_and_forward_flash_obj[*service_id];
    memset( ctx, 0, sizeof( store_and_forward_flash_t ) );

    *downlink_callback  = store_and_forward_flash_service_downlink_handler;
    *on_launch_callback = store_and_forward_flash_service_on_launch;
    *on_update_callback = store_and_forward_flash_service_on_update;
    *context_callback   = ( void* ) service_id;

    ctx->task_id     = task_id;
    ctx->stack_id    = CURRENT_STACK;
    ctx->enabled     = false;
    ctx->initialized = true;

    /* Always call circularfs_init first. */
    circularfs_init( &ctx->fs, &flash_obj, LOG_ENTRY_VERSION, sizeof( store_and_forward_flash_data_t ) );
    // SMTC_MODEM_HAL_TRACE_PRINTF( "# format filesystem...\n" );
    // circularfs_format( &ctx->fs );

    /* Scan and/or format before any data operations. */
    SMTC_MODEM_HAL_TRACE_PRINTF( "Store and fwd # scanning for filesystem...\n" );
    if( circularfs_scan( &ctx->fs ) == 0 )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Store and fwd # found existing filesystem, usage: %d/%d\n",
                                     circularfs_count_estimate( &ctx->fs ), circularfs_capacity( &ctx->fs ) );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Store and fwd # no valid filesystem found, formatting.\n" );
        circularfs_format( &ctx->fs, false );
    }

    circularfs_dump( &ctx->fs );
}

store_and_forward_flash_rc_t store_and_forward_flash_set_state( uint8_t                         stack_id,
                                                                store_and_forward_flash_state_t enabled )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t                    service_id;
    store_and_forward_flash_t* ctx = store_and_forward_flash_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
    {
        return STORE_AND_FORWARD_FLASH_RC_INVALID;
    }

    if( enabled > STORE_AND_FORWARD_SUSPEND )
    {
        return STORE_AND_FORWARD_FLASH_RC_INVALID;
    }

    if( ctx->enabled != enabled )
    {
        ctx->enabled = enabled;

        if( enabled == STORE_AND_FORWARD_ENABLE )
        {
            ctx->sending_first_try_timestamp_s = 0;
            ctx->sending_try_cpt               = 0;
            if( ( lorawan_api_isjoined( ctx->stack_id ) == JOINED ) && ( circularfs_count_estimate( &ctx->fs ) > 0 ) )
            {
                store_and_forward_flash_add_task( ctx, smtc_modem_hal_get_random_nb_in_range( 5, 10 ) );
            }
        }
        else
        {
            modem_supervisor_remove_task( ctx->task_id );
        }
    }
    return STORE_AND_FORWARD_FLASH_RC_OK;
}

store_and_forward_flash_rc_t store_and_forward_flash_add_data( uint8_t stack_id, uint8_t fport, bool confirmed,
                                                               const uint8_t* payload, uint8_t payload_length )

{
    IS_VALID_STACK_ID( stack_id );
    uint8_t                    service_id;
    store_and_forward_flash_t* ctx = store_and_forward_flash_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx->enabled == STORE_AND_FORWARD_DISABLE )
    {
        return STORE_AND_FORWARD_FLASH_RC_FAIL;
    }

    if( ( payload_length == 0 ) || ( payload_length > DATA_SIZE_MAX ) )
    {
        return STORE_AND_FORWARD_FLASH_RC_INVALID;
    }

    if( ( fport == 0 ) || ( fport >= 224 ) )
    {
        return STORE_AND_FORWARD_FLASH_RC_INVALID;
    }

#if defined( ADD_SMTC_CLOUD_DEVICE_MANAGEMENT )
    if( fport == cloud_dm_get_dm_port( stack_id ) )
    {
        return STORE_AND_FORWARD_FLASH_RC_INVALID;
    }
#endif

    store_and_forward_flash_data_t entry = { 0 };
    memcpy( entry.data, payload, payload_length );
    entry.data_len  = payload_length;
    entry.fport     = fport;
    entry.confirmed = confirmed;
    entry.crc       = crc_store_and_fwd( ( uint8_t* ) &entry, sizeof( entry ) - sizeof( entry.crc ) ) & 0xFFFF;

    if( circularfs_append( &ctx->fs, &entry ) != 0 )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Store and fwd fifo problem\n" );
        return STORE_AND_FORWARD_FLASH_RC_FAIL;
    }

    int32_t nb_of_data = circularfs_count_estimate_from_last_fetch( &ctx->fs );

    // first data added, launch the service for the first sending, then it will be automatic in update task
    if( ( nb_of_data == 1 ) || ( ctx->sending_first_try_timestamp_s == 0 ) )
    {
        if( lorawan_api_isjoined( ctx->stack_id ) == JOINED )
        {
            store_and_forward_flash_add_task( ctx, 0 );
        }
    }

    SMTC_MODEM_HAL_TRACE_PRINTF( "Store and fwd nb_of_data in fifo %u\n", nb_of_data );

    return STORE_AND_FORWARD_FLASH_RC_OK;
}

void store_and_forward_flash_clear_data( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t                    service_id;
    store_and_forward_flash_t* ctx = store_and_forward_flash_get_ctx_from_stack_id( stack_id, &service_id );

    SMTC_MODEM_HAL_TRACE_PRINTF( "Store and fwd # format filesystem...\n" );
    circularfs_format( &ctx->fs, true );
}

struct circularfs* store_and_forward_flash_get_fs_object( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t                    service_id;
    store_and_forward_flash_t* ctx = store_and_forward_flash_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx != NULL )
    {
        return &ctx->fs;
    }

    return NULL;
}

store_and_forward_flash_rc_t store_and_forward_flash_get_number_of_free_slot( uint8_t stack_id, uint32_t* capacity,
                                                                              uint32_t* free_slot )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t                    service_id;
    store_and_forward_flash_t* ctx = store_and_forward_flash_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
    {
        return STORE_AND_FORWARD_FLASH_RC_INVALID;
    }

    *capacity  = circularfs_capacity( &ctx->fs );
    *free_slot = circularfs_free_slot_estimate( &ctx->fs );
    return STORE_AND_FORWARD_FLASH_RC_OK;
}

store_and_forward_flash_state_t store_and_forward_flash_get_state( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t                    service_id;
    store_and_forward_flash_t* ctx = store_and_forward_flash_get_ctx_from_stack_id( stack_id, &service_id );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ctx != NULL );

    return ctx->enabled;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void store_and_forward_flash_service_on_launch( void* service_id )
{
    uint8_t idx = *( ( uint8_t* ) service_id );
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " %s service_id %d \n", __func__, idx );

    IS_SERVICE_INITIALIZED( idx );
    uint8_t stack_id = store_and_forward_flash_obj[idx].stack_id;

    // If not join -> exit
    if( lorawan_api_isjoined( stack_id ) != JOINED )
    {
        return;
    }

    // If service not enabled -> exit
    if( store_and_forward_flash_obj[idx].enabled != STORE_AND_FORWARD_ENABLE )
    {
        return;
    }

    bool     is_crc_ok    = false;
    int32_t  fetch_status = -1;
    uint32_t rtc_ms       = smtc_modem_hal_get_time_in_ms( );

    // TODO check payload len and adjust the datarate with custom profile
    // uint8_t max_payload = lorawan_api_next_max_payload_length_get( stack_id );

    store_and_forward_flash_data_t entry = { 0 };

    // circularfs_dump( &store_and_forward_flash_obj[idx].fs );

    // While there are available data and a wrong CRC get the next data
    do
    {
        fetch_status = circularfs_fetch( &store_and_forward_flash_obj[idx].fs, &entry );
        if( fetch_status == 0 )
        {
            if( ( crc_store_and_fwd( ( uint8_t* ) &entry, sizeof( entry ) - sizeof( entry.crc ) ) & 0xFFFF ) ==
                entry.crc )
            {
                is_crc_ok = true;
                if( store_and_forward_flash_obj[idx].sending_try_cpt == 0 )
                {
                    store_and_forward_flash_obj[idx].sending_first_try_timestamp_s = rtc_ms / 1000;
                }

#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
                int32_t capacity  = circularfs_capacity( &store_and_forward_flash_obj[idx].fs );
                int32_t free_slot = circularfs_free_slot_estimate( &store_and_forward_flash_obj[idx].fs );
                SMTC_MODEM_HAL_TRACE_PRINTF( "Store and fwd get data, free: %d/%d \n", free_slot, capacity );
#endif
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_WARNING( "Store and fwd corrupted data, bad CRC !\n" );
            }
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "Store and fwd no data !\n" );
        }
    } while( ( fetch_status == 0 ) && ( is_crc_ok == false ) );

    if( ( entry.data_len > 0 ) && ( is_crc_ok == true ) )
    {
        store_and_forward_flash_obj[idx].sending_with_ack = entry.confirmed;

        if( store_and_forward_flash_obj[idx].ack_period_count >= STORE_AND_FORWARD_ACK_PERIOD )
        {
            store_and_forward_flash_obj[idx].sending_with_ack = true;
        }

#if( FIFO_BURST_SENDING == true )
        uint32_t tmp = circularfs_count_estimate_from_last_fetch( &store_and_forward_flash_obj[idx].fs );
        if( ( tmp > 0 ) && ( store_and_forward_flash_obj[idx].sending_with_ack == false ) )
        {
            uint8_t           dr = lorawan_api_next_dr_get( store_and_forward_flash_obj[idx].stack_id );
            modulation_type_t modulation =
                lorawan_api_get_modulation_type_from_datarate( store_and_forward_flash_obj[idx].stack_id, dr );

            if( modulation == LORA )
            {
                uint8_t            out_sf;
                lr1mac_bandwidth_t out_bw;
                lorawan_api_lora_dr_to_sf_bw( dr, &out_sf, &out_bw, store_and_forward_flash_obj[idx].stack_id );
                if( out_sf < 9 )
                {
                    lorawan_api_set_no_rx_windows( store_and_forward_flash_obj[idx].stack_id, true );
                }
            }
            else if( modulation == FSK )
            {
                lorawan_api_set_no_rx_windows( store_and_forward_flash_obj[idx].stack_id, true );
            }
        }
#endif

        status_lorawan_t send_status = tx_protocol_manager_request(
            TX_PROTOCOL_TRANSMIT_LORA, entry.fport, true, entry.data, entry.data_len,
            ( store_and_forward_flash_obj[idx].sending_with_ack == true ) ? CONF_DATA_UP : UNCONF_DATA_UP, rtc_ms,
            stack_id );

        if( send_status == OKLORAWAN )
        {
            store_and_forward_flash_obj[idx].sending_try_cpt++;
            if( store_and_forward_flash_obj[idx].ack_period_count < STORE_AND_FORWARD_ACK_PERIOD )
            {
                store_and_forward_flash_obj[idx].ack_period_count++;
            }
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_WARNING( " %s service_id %d data not send 0x%x\n", __func__, idx, send_status );
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "Store and fwd try (%u) to send \n",
                                     store_and_forward_flash_obj[idx].sending_try_cpt );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "Store and fwd nothing to send \n" );
    }
}

static void store_and_forward_flash_service_on_update( void* service_id )
{
    uint8_t idx = *( ( uint8_t* ) service_id );
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " %s service_id %d \n", __func__, idx );

    IS_SERVICE_INITIALIZED( idx );
    uint8_t stack_id = store_and_forward_flash_obj[idx].stack_id;

    if( lorawan_api_isjoined( stack_id ) != JOINED )
    {
        return;
    }

    // If service not enabled -> exit
    if( store_and_forward_flash_obj[idx].enabled != STORE_AND_FORWARD_ENABLE )
    {
        return;
    }

#if( FIFO_BURST_SENDING == true )
    lorawan_api_set_no_rx_windows( store_and_forward_flash_obj[idx].stack_id, false );
#endif

    int32_t nb_of_data = circularfs_count_estimate_from_last_fetch( &store_and_forward_flash_obj[idx].fs );

    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "Store and fwd nb_of_data not read:%u\n", nb_of_data );

    if( ( nb_of_data > 0 ) || store_and_forward_flash_obj[idx].sending_try_cpt != 0 )
    {
        uint32_t delay_tmp_s = store_and_forward_flash_compute_next_delay_s( &store_and_forward_flash_obj[idx] );

        SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "store en fwd next execute:%u (s)\n", delay_tmp_s );
        store_and_forward_flash_add_task( &store_and_forward_flash_obj[idx], delay_tmp_s );
    }
}

static uint8_t store_and_forward_flash_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    uint8_t                    stack_id = rx_down_data->stack_id;
    uint8_t                    service_id;
    store_and_forward_flash_t* ctx = store_and_forward_flash_get_ctx_from_stack_id( stack_id, &service_id );

    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " %s\n", __func__ );

    task_id_t current_task_id = modem_supervisor_get_task( )->next_task_id;
    // If a downlink is received outside the running service, the downlink is not an ack but there is network
    // coverage
    if( current_task_id != ctx->task_id )
    {
        if( rx_down_data->rx_metadata.rx_window != RECEIVE_NONE )
        {
            int32_t nb_of_data = circularfs_count_estimate_from_last_fetch( &ctx->fs );

            if( ( ( ctx->sending_try_cpt > 0 ) || ( nb_of_data > 0 ) ) && ( ctx->enabled == STORE_AND_FORWARD_ENABLE ) )
            {
                store_and_forward_flash_add_task( ctx, MODEM_TASK_DELAY_MS / 1000 );
                SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "Store and fwd not for me, but data in fifo, enqueue task\n" );
            }
        }
    }
    else
    {
        if( ctx->sending_try_cpt != 0 )
        {
            if( ctx->sending_with_ack == false )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "Store and fwd uplink wo ack\n" );
                ctx->sending_try_cpt = 0;
            }
            else
            {
                if( rx_down_data->rx_metadata.rx_ack_bit == true )
                {
                    SMTC_MODEM_HAL_TRACE_PRINTF( "Store and fwd uplink acked after %u tentative\n",
                                                 ctx->sending_try_cpt );
                    ctx->sending_try_cpt  = 0;
                    ctx->ack_period_count = 0;
                    circularfs_discard( &ctx->fs );
                }
                else
                {
                    SMTC_MODEM_HAL_TRACE_PRINTF( "Store and fwd uplink not yet acked after %u tentative\n",
                                                 ctx->sending_try_cpt );
                    circularfs_rewind( &ctx->fs );
                }
            }
        }
    }

    return MODEM_DOWNLINK_UNCONSUMED;
}

static store_and_forward_flash_t* store_and_forward_flash_get_ctx_from_stack_id( uint8_t stack_id, uint8_t* service_id )
{
    store_and_forward_flash_t* ctx = NULL;
    for( uint8_t i = 0; i < NUMBER_MAX_OF_STORE_AND_FORWARD_OBJ; i++ )
    {
        if( store_and_forward_flash_obj[i].stack_id == stack_id )
        {
            ctx         = &store_and_forward_flash_obj[i];
            *service_id = i;
            break;
        }
    }

    return ctx;
}

static void store_and_forward_flash_add_task( store_and_forward_flash_t* ctx, uint32_t delay_to_execute_s )
{
    // If service not enabled -> exit
    if( ctx->enabled != STORE_AND_FORWARD_ENABLE )
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

static uint32_t store_and_forward_flash_compute_next_delay_s( store_and_forward_flash_t* ctx )
{
    // TODO implement the right algorithm to compute the delay between each retry to send the same packet not acked
    if( ctx->sending_try_cpt == 0 )
    {
        return 0;
    }
    // Exponential backoff (2^sending_try_cpt * 60s)
    uint32_t delay_s = ( 1 << ctx->sending_try_cpt ) * 60;
    if( delay_s > STORE_AND_FORWARD_DELAY_MAX_S )
    {
        delay_s = STORE_AND_FORWARD_DELAY_MAX_S;
    }
    return delay_s + smtc_modem_hal_get_random_nb_in_range( 0, 60 );
}

static int32_t op_sector_erase( struct circularfs_flash_partition* flash, uint32_t address )
{
    ( void ) flash;
    smtc_modem_hal_context_flash_pages_erase( CONTEXT_STORE_AND_FORWARD, address, 1 );
    return 0;
}

static int32_t op_program( struct circularfs_flash_partition* flash, uint32_t address, const void* data, uint32_t size )
{
    ( void ) flash;
    smtc_modem_hal_context_store( CONTEXT_STORE_AND_FORWARD, address, data, size );

    // dummy context reading to ensure context store is done before exiting the function
    uint8_t dummy_read[sizeof( store_and_forward_flash_data_t )];
    smtc_modem_hal_context_restore( CONTEXT_STORE_AND_FORWARD, address, dummy_read, size );
    return size;
}

static int32_t op_read( struct circularfs_flash_partition* flash, uint32_t address, void* data, uint32_t size )
{
    ( void ) flash;
    smtc_modem_hal_context_restore( CONTEXT_STORE_AND_FORWARD, address, data, size );
    return size;
}

static uint32_t crc_store_and_fwd( const uint8_t* buf, int len )
{
    uint32_t crc = 0xFFFFFFFF;
    while( len-- > 0 )
    {
        crc = crc ^ *buf++;
        for( int i = 0; i < 8; i++ )
        {
            uint32_t mask = -( crc & 1 );
            crc           = ( crc >> 1 ) ^ ( 0xEDB88320 & mask );
        }
    }
    return ~crc;
}
/* --- EOF ------------------------------------------------------------------ */
