/**
 * @file      lorawan_send_management.c
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
#include "lorawan_send_management.h"
#include "modem_supervisor_light.h"
#include "smtc_modem_api.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "lorawan_api.h"
#include "modem_core.h"
#include "modem_event_utilities.h"
#include "smtc_duty_cycle.h"
#include "modem_tx_protocol_manager.h"

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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
#ifndef MODEM_MIN_RANDOM_DELAY_MS
#define MODEM_MIN_RANDOM_DELAY_MS 200
#endif

#ifndef MODEM_MAX_RANDOM_DELAY_MS
#define MODEM_MAX_RANDOM_DELAY_MS 3000
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */
typedef struct lorawan_send_management_s
{
    uint8_t rx_ack_bit_context;
    uint8_t payload[242];
    uint8_t payload_length;
    uint8_t fport;
    bool    fport_present;
    bool    packet_type;

} lorawan_send_management_t;

static lorawan_send_management_t lorawan_send_management_obj[NUMBER_OF_STACKS];

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
static void lorawan_send_management_service_on_launch( void* service_id );

/**
 * @brief Callback called at the end of the task
 *
 * @param context_callback
 */
static void lorawan_send_management_service_on_update( void* service_id );

/**
 * @brief Callback to handle the downlink received by the LoRaWAN layer
 *
 * @param rx_down_data
 */
static uint8_t lorawan_send_management_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void lorawan_send_management_services_init( uint8_t* service_id, uint8_t task_id,
                                            uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                            void ( **on_launch_callback )( void* ),
                                            void ( **on_update_callback )( void* ), void** context_callback )
{
    *downlink_callback  = lorawan_send_management_service_downlink_handler;
    *on_launch_callback = lorawan_send_management_service_on_launch;
    *on_update_callback = lorawan_send_management_service_on_update;
    *context_callback   = ( void* ) modem_supervisor_get_task( );
}

void lorawan_send_add_task( uint8_t stack_id, uint8_t f_port, bool send_fport, bool confirmed, const uint8_t* payload,
                            uint8_t payload_length, bool emergency, uint32_t delay_s )

{
    IS_VALID_STACK_ID( stack_id );
    smodem_task task_send = { 0 };
    if( emergency == true )
    {
        task_send.priority = TASK_HIGH_PRIORITY;
        smtc_duty_cycle_enable_set( SMTC_DTC_PARTIAL_DISABLED );
    }
    else
    {
        task_send.priority = TASK_MEDIUM_HIGH_PRIORITY;
    }
    if( payload != NULL )
    {
        lorawan_send_management_obj[stack_id].payload_length = payload_length;
        memcpy( lorawan_send_management_obj[stack_id].payload, payload, payload_length );
    }
    else
    {
        lorawan_send_management_obj[stack_id].payload_length = 0;
    }

    lorawan_send_management_obj[stack_id].fport         = f_port;
    lorawan_send_management_obj[stack_id].fport_present = send_fport;
    lorawan_send_management_obj[stack_id].packet_type   = confirmed;

    task_send.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + delay_s;
    task_send.stack_id          = stack_id;
    task_send.id                = SEND_TASK + ( NUMBER_OF_TASKS * stack_id );

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( modem_supervisor_add_task( &task_send ) == TASK_VALID );
}

void lorawan_send_remove_task( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    modem_supervisor_remove_task( SEND_TASK + ( NUMBER_OF_TASKS * stack_id ) );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lorawan_send_management_service_on_launch( void* context )
{
    status_lorawan_t send_status                                          = ERRORLORAWAN;
    stask_manager*   task_manager                                         = ( stask_manager* ) context;
    lorawan_send_management_obj[STACK_ID_CURRENT_TASK].rx_ack_bit_context = 0;

    send_status = tx_protocol_manager_request(
        TX_PROTOCOL_TRANSMIT_LORA, lorawan_send_management_obj[STACK_ID_CURRENT_TASK].fport,
        lorawan_send_management_obj[STACK_ID_CURRENT_TASK].fport_present,
        lorawan_send_management_obj[STACK_ID_CURRENT_TASK].payload,
        lorawan_send_management_obj[STACK_ID_CURRENT_TASK].payload_length,
        ( lorawan_send_management_obj[STACK_ID_CURRENT_TASK].packet_type == true ) ? CONF_DATA_UP : UNCONF_DATA_UP,
        smtc_modem_hal_get_time_in_ms( ), STACK_ID_CURRENT_TASK );

    if( send_status == OKLORAWAN )
    {
        task_manager->modem_task[VIRTUAL_TASK_ID].task_context = true;
        SMTC_MODEM_HAL_TRACE_PRINTF(
            " User LoRaWAN tx %s %d \n",
            ( lorawan_send_management_obj[STACK_ID_CURRENT_TASK].fport_present == true ) ? "on FPort" : "No FPort",
            lorawan_send_management_obj[STACK_ID_CURRENT_TASK].fport );
    }
    else
    {
        task_manager->modem_task[VIRTUAL_TASK_ID].task_context = false;
        SMTC_MODEM_HAL_TRACE_WARNING( "The payload can't be send! internal code: %x\n", send_status );
    }
}

static void lorawan_send_management_service_on_update( void* context )
{
    stask_manager* task_manager = ( stask_manager* ) context;
    if( task_manager->modem_task[VIRTUAL_TASK_ID].priority == TASK_HIGH_PRIORITY )
    {
        smtc_duty_cycle_enable_set( SMTC_DTC_ENABLED );
    }

    if( task_manager->modem_task[VIRTUAL_TASK_ID].task_enabled == true )
    {
        if( ( task_manager->modem_task[VIRTUAL_TASK_ID].task_context == true ) &&
            ( tx_protocol_manager_tx_is_aborted( ) == false ) )
        {
            if( lorawan_send_management_obj[STACK_ID_CURRENT_TASK].rx_ack_bit_context == 1 )
            {
                increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TXDONE, MODEM_TX_SUCCESS_WITH_ACK,
                                                  STACK_ID_CURRENT_TASK );
            }
            else
            {
                increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TXDONE, MODEM_TX_SUCCESS, STACK_ID_CURRENT_TASK );
            }
        }
        else
        {
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TXDONE, MODEM_TX_FAILED, STACK_ID_CURRENT_TASK );
        }
    }
}

static uint8_t lorawan_send_management_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
#if defined( ADD_RELAY_TX )
    if( ( rx_down_data->rx_metadata.rx_window == RECEIVE_ON_RX2 ) ||
        ( rx_down_data->rx_metadata.rx_window == RECEIVE_ON_RX1 ) ||
        ( rx_down_data->rx_metadata.rx_window == RECEIVE_ON_RXR ) )
#else
    if( ( rx_down_data->rx_metadata.rx_window == RECEIVE_ON_RX2 ) ||
        ( rx_down_data->rx_metadata.rx_window == RECEIVE_ON_RX1 ) )
#endif
    {
        if( lorawan_send_management_obj[rx_down_data->stack_id].rx_ack_bit_context != 1 )
        {
            lorawan_send_management_obj[rx_down_data->stack_id].rx_ack_bit_context =
                rx_down_data->rx_metadata.rx_ack_bit;
        }
    }

    return MODEM_DOWNLINK_UNCONSUMED;
}

/* --- EOF ------------------------------------------------------------------ */
