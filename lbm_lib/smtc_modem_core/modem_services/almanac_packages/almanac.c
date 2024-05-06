/**
 * @file      almanac.c
 *
 * @brief     LoRaWAN Application Layer Clock Synchronization V1.0.0 Implementation
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
#include "almanac.h"
#include "modem_core.h"
#include "modem_supervisor_light.h"
#include "smtc_modem_api.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "lorawan_api.h"
#include "modem_core.h"
#include "device_management_defs.h"
#include "lr11xx_gnss.h"
#include "radio_planner_hook_id_defs.h"
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )
#define NUMBER_MAX_OF_ALMANAC_OBJ 1  // modify in case of multiple obj


/**
 * @brief Check is the index is valid before accessing ALCSync object
 *
 */
#define IS_VALID_OBJECT_ID( x )                                           \
    do                                                                    \
    {                                                                     \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_MAX_OF_ALMANAC_OBJ ); \
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
#define IS_SERVICE_INITIALIZED( )                                                    \
    do                                                                               \
    {                                                                                \
        if( almanac_obj.initialized == false )                                       \
        {                                                                            \
            SMTC_MODEM_HAL_TRACE_WARNING( "almanac_obj service not initialized\n" ); \
            return;                                                                  \
        }                                                                            \
    } while( 0 )

/**
 * @brief Check is the service is enabled
 *
 */
#define IS_SERVICE_ENABLED( )                                                    \
    do                                                                           \
    {                                                                            \
        if( almanac_obj.enabled == false )                                       \
        {                                                                        \
            SMTC_MODEM_HAL_TRACE_WARNING( "almanac_obj service not enabled\n" ); \
            return;                                                              \
        }                                                                        \
    } while( 0 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

static almanac_t almanac_obj;
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static void rp_end_almanac_callback( void* status );
static void rp_start_almanac_callback( void* context );
static void request_access_to_rp_4_almanac_update( void );
void        add_almanac_task( uint32_t delays_s );
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void almanac_services_init( uint8_t* service_id, uint8_t task_id,
                            uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                            void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                            void** context_callback )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "almanac_services_init\n" );

    IS_VALID_OBJECT_ID( *service_id );
    *downlink_callback                         = almanac_service_downlink_handler;
    *on_launch_callback                        = almanac_service_on_launch;
    *on_update_callback                        = almanac_service_on_update;
    *context_callback                          = ( void* ) service_id;
    almanac_obj.task_id                        = task_id;
    almanac_obj.stack_id                       = CURRENT_STACK;
    almanac_obj.enabled                        = false;
    almanac_obj.initialized                    = true;
    almanac_obj.almanac_dw_buffer_size         = 0;
    almanac_obj.up_delay                       = 0;
    almanac_obj.up_count                       = 0;
    almanac_obj.get_almanac_status_from_lr11xx = false;
    almanac_obj.rp_hook_id                     = RP_HOOK_ID_DIRECT_RP_ACCESS_4_ALMANAC + CURRENT_STACK;
    rp_hook_init( modem_get_rp( ), almanac_obj.rp_hook_id, ( void ( * )( void* ) )( rp_end_almanac_callback ),
                  modem_get_rp( ) );
}

void almanac_service_on_launch( void* context )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "almanac_service_on_launch\n" );

    IS_SERVICE_ENABLED( );
    almanac_obj.almanac_status_from_lr11xx[1] = DM_INFO_ALMSTATUS;

    uint8_t dm_port;
#if defined( ADD_SMTC_CLOUD_DEVICE_MANAGEMENT )
    dm_port = cloud_dm_get_dm_port( almanac_obj.stack_id );
#else
    dm_port = DM_PORT;
#endif

    tx_protocol_manager_request (TX_PROTOCOL_TRANSMIT_LORA, dm_port, true, &almanac_obj.almanac_status_from_lr11xx[1],
                              SERVICE_LR11XX_GNSS_CONTEXT_STATUS_LENGTH - 1, UNCONF_DATA_UP,
                              smtc_modem_hal_get_time_in_ms( )  , almanac_obj.stack_id );
    if( almanac_obj.up_count > 0 )
    {
        almanac_obj.up_count--;
    }
}

void almanac_service_on_update( void* context )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "almanac_service_on_update\n" );

    IS_SERVICE_INITIALIZED( );
    IS_SERVICE_ENABLED( );
    request_access_to_rp_4_almanac_update( );
}

uint8_t almanac_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "almanac_service_downlink_handler\n" );

    if( ( almanac_obj.enabled == false ) || ( almanac_obj.initialized == false ) )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }
    uint8_t dm_port;
#if defined( ADD_SMTC_CLOUD_DEVICE_MANAGEMENT )
    dm_port = cloud_dm_get_dm_port( almanac_obj.stack_id );
#else
    dm_port = DM_PORT;
#endif
    if( ( rx_down_data->rx_metadata.rx_fport_present == true ) &&
        ( rx_down_data->rx_payload_size > DM_DOWNLINK_HEADER_LENGTH ) &&
        ( rx_down_data->rx_metadata.rx_fport == dm_port ) &&
        ( ( dm_opcode_t ) rx_down_data->rx_payload[2] == DM_ALM_UPDATE ) )
    {
        almanac_obj.up_count               = rx_down_data->rx_payload[0];
        almanac_obj.up_delay               = rx_down_data->rx_payload[1];
        almanac_obj.almanac_dw_buffer_size = rx_down_data->rx_payload_size - DM_DOWNLINK_HEADER_LENGTH + 1;
        memcpy( almanac_obj.almanac_dw_buffer, &rx_down_data->rx_payload[2], almanac_obj.almanac_dw_buffer_size );

        task_id_t current_task_id = modem_supervisor_get_task( )->next_task_id;
        if( current_task_id != almanac_obj.task_id )
        {
            request_access_to_rp_4_almanac_update( );
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "almanac_service_downlink_handler => consumed\n" );
        return MODEM_DOWNLINK_CONSUMED;
    }

    return MODEM_DOWNLINK_UNCONSUMED;
}

void start_almanac_service( uint8_t stack_id )
{
    almanac_obj.enabled  = true;
    almanac_obj.up_count = ALMANAC_UP_COUNT_INIT;
    request_access_to_rp_4_almanac_update( );
}

void stop_almanac_service( uint8_t stack_id )
{
    almanac_obj.enabled = false;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void add_almanac_task( uint32_t delays_s )
{
    IS_SERVICE_INITIALIZED( );
    IS_SERVICE_ENABLED( );
    smodem_task task       = { 0 };
    task.id                = almanac_obj.task_id;
    task.stack_id          = almanac_obj.stack_id;
    task.priority          = TASK_MEDIUM_HIGH_PRIORITY;
    task.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + delays_s;
    modem_supervisor_add_task( &task );
}

static void rp_end_almanac_callback( void* status )
{
    IS_SERVICE_ENABLED( );
    if( almanac_obj.get_almanac_status_from_lr11xx == true )
    {  // almanach have been updated/read inside the lr11xx
        almanac_obj.get_almanac_status_from_lr11xx = false;
        uint32_t delay_s                           = ( almanac_obj.up_count > 0 ) ? ALMANAC_1SECOND : ALMANAC_PERIOD_S;
        add_almanac_task( delay_s );
    }
    else
    {  // task has been aborted by an other task in the radio planner
        request_access_to_rp_4_almanac_update( );
    }
}

static void rp_start_almanac_callback( void* context )
{
    lr11xx_gnss_context_status_t context_status;

    IS_SERVICE_ENABLED( );

    if( almanac_obj.almanac_dw_buffer_size > 0 )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "=> Push almanac update to LR11xx\n" );
        lr11xx_gnss_push_dmc_msg( modem_get_radio_ctx( ), almanac_obj.almanac_dw_buffer,
                                  almanac_obj.almanac_dw_buffer_size );
        almanac_obj.almanac_dw_buffer_size = 0;
    }
    lr11xx_gnss_get_context_status( modem_get_radio_ctx( ), &almanac_obj.almanac_status_from_lr11xx[0] );
    almanac_obj.get_almanac_status_from_lr11xx = true;
    lr11xx_gnss_parse_context_status_buffer( almanac_obj.almanac_status_from_lr11xx, &context_status );
    SMTC_MODEM_HAL_TRACE_PRINTF( "=> Almanac CRC: 0x%08X\n", context_status.global_almanac_crc );

    rp_task_abort( modem_get_rp( ), almanac_obj.rp_hook_id );
}

static void request_access_to_rp_4_almanac_update( void )
{
    IS_SERVICE_ENABLED( );
    uint32_t  time_ms                      = smtc_modem_hal_get_time_in_ms( ) + 1000;
    rp_task_t rp_task                      = { 0 };
    rp_task.hook_id                        = almanac_obj.rp_hook_id;
    rp_task.state                          = RP_TASK_STATE_ASAP;
    rp_task.start_time_ms                  = time_ms;
    rp_task.duration_time_ms               = 100;
    rp_task.type                           = RP_TASK_TYPE_NONE;
    rp_task.launch_task_callbacks          = rp_start_almanac_callback;
    rp_radio_params_t fake_rp_radio_params = { 0 };
    if( rp_task_enqueue( modem_get_rp( ), &rp_task, NULL, 0, &fake_rp_radio_params ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to enqueue RP task for almanac update\n" );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Enqueued RP task for almanac update\n" );
    }
}

/* --- EOF ------------------------------------------------------------------ */
