/**
 * @file      lorawan_class_b_management.c
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
#include "lorawan_class_b_management.h"
#include "modem_core.h"
#include "modem_supervisor_light.h"
#include "smtc_modem_api.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "lorawan_api.h"
#include "modem_event_utilities.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define VIRTUAL_TASK_ID ( ( stask_manager* ) context )->next_task_id
#define STACK_ID_CURRENT_TASK ( ( stask_manager* ) context )->modem_task[VIRTUAL_TASK_ID].stack_id

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
#define IS_SERVICE_INITIALIZED( x )                                                               \
    do                                                                                            \
    {                                                                                             \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( lorawan_class_b_management_obj[x].initialized == true ); \
    } while( 0 )

/**
 * @brief Check is the service is enabled before accessing the object
 *
 */
#define IS_SERVICE_ENABLED( x )                                  \
    do                                                           \
    {                                                            \
        if( lorawan_class_b_management_obj[x].enabled == false ) \
        {                                                        \
            return;                                              \
        }                                                        \
    } while( 0 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
#define NUMBER_MAX_OF_CLASS_B_MANAGEMENT_OBJ NUMBER_OF_STACKS
#define PERIOD_CLASS_B_MANAGEMENT_CHECK_S 130

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief LoRaWAN template Object
 *
 * @struct lorawan_class_b_management_s
 *
 */

typedef struct lorawan_class_b_management_s
{
    uint8_t stack_id;
    uint8_t task_id;
    bool    initialized;
    bool    enabled;
    bool    previous_tx_class_b_bit;
    bool    current_tx_class_b_bit;
    uint8_t spare[4];
} lorawan_class_b_management_t;

lorawan_class_b_management_t lorawan_class_b_management_obj[NUMBER_MAX_OF_CLASS_B_MANAGEMENT_OBJ];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
/**
 * @brief Callback called at task launch
 *
 * @param context_callback
 */
static void lorawan_class_b_management_service_on_launch( void* context );

/**
 * @brief Callback called at the end of the task
 *
 * @param context_callback
 */
static void lorawan_class_b_management_service_on_update( void* context );

/**
 * @brief Callback to handle the downlink received by the LoRaWAN layer
 *
 * @param rx_down_data
 */
static uint8_t lorawan_class_b_management_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void lorawan_class_b_management_services_init( uint8_t* service_id, uint8_t task_id,
                                               uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                               void ( **on_launch_callback )( void* ),
                                               void ( **on_update_callback )( void* ), void** context_callback )
{
    *downlink_callback  = lorawan_class_b_management_service_downlink_handler;
    *on_launch_callback = lorawan_class_b_management_service_on_launch;
    *on_update_callback = lorawan_class_b_management_service_on_update;
    *context_callback   = ( void* ) modem_supervisor_get_task( );
    for( uint8_t i = 0; i < NUMBER_MAX_OF_CLASS_B_MANAGEMENT_OBJ; i++ )
    {
        lorawan_class_b_management_obj[i].task_id                 = task_id;
        lorawan_class_b_management_obj[i].stack_id                = i;
        lorawan_class_b_management_obj[i].enabled                 = false;
        lorawan_class_b_management_obj[i].initialized             = true;
        lorawan_class_b_management_obj[i].previous_tx_class_b_bit = false;
        lorawan_class_b_management_obj[i].current_tx_class_b_bit  = false;
    }
}

void lorawan_class_b_management_enable( uint8_t stack_id, bool enable, uint8_t delay )
{
    IS_VALID_STACK_ID( stack_id );
    if( lorawan_class_b_management_obj[stack_id].enabled != enable )
    {
        if( enable == true )
        {
            lorawan_class_b_management_add_task( stack_id, delay );
        }
        else
        {
            lorawan_api_class_b_enabled( false, stack_id );
            lorawan_class_b_management_remove_task( stack_id );
            lorawan_class_b_management_obj[stack_id].previous_tx_class_b_bit = false;
            lorawan_class_b_management_obj[stack_id].current_tx_class_b_bit  = false;
        }
        lorawan_class_b_management_obj[stack_id].enabled = enable;
    }
}

void lorawan_class_b_management_add_task( uint8_t stack_id, uint8_t delay )
{
    IS_VALID_STACK_ID( stack_id );
    IS_SERVICE_INITIALIZED( stack_id );

    smodem_task task_class_b       = { 0 };
    task_class_b.id                = CLASS_B_MANAGEMENT_TASK + ( NUMBER_OF_TASKS * stack_id );
    task_class_b.stack_id          = stack_id;
    task_class_b.priority          = TASK_MEDIUM_HIGH_PRIORITY;
    task_class_b.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + delay;
    modem_supervisor_add_task( &task_class_b );
}

void lorawan_class_b_management_remove_task( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    modem_supervisor_remove_task( CLASS_B_MANAGEMENT_TASK + ( NUMBER_OF_TASKS * stack_id ) );
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lorawan_class_b_management_service_on_launch( void* context )
{
    IS_SERVICE_INITIALIZED( STACK_ID_CURRENT_TASK );
    IS_SERVICE_ENABLED( STACK_ID_CURRENT_TASK );
    smtc_beacon_statistics_t beacon_statistics;
    lorawan_api_beacon_get_statistics( &beacon_statistics, STACK_ID_CURRENT_TASK );
    uint8_t cid_buffer[]     = { PING_SLOT_INFO_REQ, DEVICE_TIME_REQ };
    uint8_t cid_request_size = 0;
    if( lorawan_api_is_time_valid( STACK_ID_CURRENT_TASK ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Miss Time info to launch class B \n" );
        cid_buffer[cid_request_size] = DEVICE_TIME_REQ;
        cid_request_size++;
    }
    if( lorawan_api_get_ping_slot_info_req_status( STACK_ID_CURRENT_TASK ) != OKLORAWAN )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Miss ping slot info answer to launch class B \n" );
        cid_buffer[cid_request_size] = PING_SLOT_INFO_REQ;
        cid_request_size++;
    }
    if( cid_request_size > 0 )
    {
        tx_protocol_manager_request( TX_PROTOCOL_TRANSMIT_CID, 0, false, cid_buffer, cid_request_size, 0,
                                     smtc_modem_hal_get_time_in_ms( ), STACK_ID_CURRENT_TASK );
    }
    else if( lorawan_class_b_management_obj[STACK_ID_CURRENT_TASK].previous_tx_class_b_bit !=
             lorawan_class_b_management_obj[STACK_ID_CURRENT_TASK].current_tx_class_b_bit )
    {
        // send empty payload without port to advertise NS if class B is enabled or disabled
        tx_protocol_manager_request( TX_PROTOCOL_TRANSMIT_LORA, 1, false, NULL, 0, UNCONF_DATA_UP,
                                     smtc_modem_hal_get_time_in_ms( ), STACK_ID_CURRENT_TASK );
    }
}

static void lorawan_class_b_management_service_on_update( void* context )
{
    IS_SERVICE_INITIALIZED( STACK_ID_CURRENT_TASK );
    IS_SERVICE_ENABLED( STACK_ID_CURRENT_TASK );

    if( ( lorawan_api_is_time_valid( STACK_ID_CURRENT_TASK ) == true ) &&
        ( lorawan_api_get_ping_slot_info_req_status( STACK_ID_CURRENT_TASK ) == OKLORAWAN ) &&
        ( lorawan_api_class_b_enabled_get( STACK_ID_CURRENT_TASK ) == false ) )
    {
        lorawan_api_class_b_enabled( true, STACK_ID_CURRENT_TASK );
        SMTC_MODEM_HAL_TRACE_PRINTF( " Start beacon search to launch class B stack %u \n", STACK_ID_CURRENT_TASK );
    }

    bool class_b_bit_temp = lorawan_api_get_class_b_status( STACK_ID_CURRENT_TASK );

    lorawan_class_b_management_obj[STACK_ID_CURRENT_TASK].previous_tx_class_b_bit =
        lorawan_class_b_management_obj[STACK_ID_CURRENT_TASK].current_tx_class_b_bit;

    if( lorawan_class_b_management_obj[STACK_ID_CURRENT_TASK].previous_tx_class_b_bit != class_b_bit_temp )
    {
        ( class_b_bit_temp == true )
            ? increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_CLASS_B_STATUS, SMTC_MODEM_EVENT_CLASS_B_READY,
                                                STACK_ID_CURRENT_TASK )
            : increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_CLASS_B_STATUS, SMTC_MODEM_EVENT_CLASS_B_NOT_READY,
                                                STACK_ID_CURRENT_TASK );
        lorawan_class_b_management_add_task( STACK_ID_CURRENT_TASK, 0 );
    }
    else
    {
        lorawan_class_b_management_add_task( STACK_ID_CURRENT_TASK, PERIOD_CLASS_B_MANAGEMENT_CHECK_S );
    }

    lorawan_class_b_management_obj[STACK_ID_CURRENT_TASK].current_tx_class_b_bit = class_b_bit_temp;
}

static uint8_t lorawan_class_b_management_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    if( rx_down_data->rx_metadata.rx_window != RECEIVE_ON_RXBEACON )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    uint8_t stack_id = rx_down_data->stack_id;

    lorawan_class_b_management_t* ctx = NULL;
    for( uint8_t i = 0; i < NUMBER_MAX_OF_CLASS_B_MANAGEMENT_OBJ; i++ )
    {
        if( lorawan_class_b_management_obj[i].stack_id == stack_id )
        {
            ctx = &lorawan_class_b_management_obj[i];
            break;
        }
    }

    if( ctx == NULL )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( stack_id >= NUMBER_MAX_OF_CLASS_B_MANAGEMENT_OBJ )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "%s: stack id not valid %u \n", __func__, stack_id );
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( ctx->enabled != true )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    task_id_t current_task_id = modem_supervisor_get_task( )->next_task_id;
    if( current_task_id != ctx->task_id )
    {
        // To generate event in case receive downlink when another service is running
        lorawan_class_b_management_add_task( stack_id, 0 );
    }

    return MODEM_DOWNLINK_CONSUMED;
}

/* --- EOF ------------------------------------------------------------------ */
