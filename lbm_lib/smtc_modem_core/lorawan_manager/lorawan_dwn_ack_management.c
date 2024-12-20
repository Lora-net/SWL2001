/**
 * @file      lorawan_dwn_ack_management.c
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
#include "lorawan_dwn_ack_management.h"
#include "modem_supervisor_light.h"
#include "smtc_modem_api.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "lorawan_send_management.h"
#include "lorawan_api.h"
#include "modem_core.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define VIRTUAL_TASK_ID ( ( stask_manager* ) context )->next_task_id
#define STACK_ID_CURRENT_TASK ( ( stask_manager* ) context )->modem_task[VIRTUAL_TASK_ID].stack_id
#define CURRENT_TASK_TIME ( ( stask_manager* ) context )->modem_task[VIRTUAL_TASK_ID].time_to_execute_s

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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

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
static void lorawan_dwn_ack_management_on_launch( void* context_callback );

/**
 * @brief Callback called at the end of the task
 *
 * @param context_callback
 */
static void lorawan_dwn_ack_management_on_update( void* context_callback );

/**
 * @brief Callback to handle the downlink received by the LoRaWAN layer
 *
 * @param rx_down_data
 */
static uint8_t lorawan_dwn_ack_management_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void lorawan_dwn_ack_management_init( uint8_t* service_id, uint8_t task_id,
                                      uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                      void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                                      void** context_callback )
{
    *downlink_callback  = lorawan_dwn_ack_management_downlink_handler;
    *on_launch_callback = lorawan_dwn_ack_management_on_launch;
    *on_update_callback = lorawan_dwn_ack_management_on_update;
    *context_callback   = ( void* ) modem_supervisor_get_task( );
}

void lorawan_dwn_ack_add_task( uint8_t stack_id, uint32_t time_to_execute )
{
    IS_VALID_STACK_ID( stack_id );
    smodem_task task       = { 0 };
    task.id                = RETRIEVE_DL_TASK + ( NUMBER_OF_TASKS * stack_id );
    task.stack_id          = stack_id;
    task.priority          = TASK_MEDIUM_HIGH_PRIORITY;
    task.time_to_execute_s = time_to_execute;
    if( lorawan_api_isjoined( stack_id ) == JOINED )
    {
        modem_supervisor_add_task( &task );
    }
}

void lorawan_dwn_ack_remove_task( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    modem_supervisor_remove_task( RETRIEVE_DL_TASK + ( NUMBER_OF_TASKS * stack_id ) );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lorawan_dwn_ack_management_on_launch( void* context )
{
    if( ( smtc_modem_hal_get_time_in_s( ) <= ( CURRENT_TASK_TIME + 2 ) ) &&
        ( lorawan_api_tx_ack_bit_get( STACK_ID_CURRENT_TASK ) ) )
    {
        tx_protocol_manager_request( TX_PROTOCOL_TRANSMIT_LORA, 1, false, NULL, 0, UNCONF_DATA_UP,
                                     smtc_modem_hal_get_time_in_ms( ), STACK_ID_CURRENT_TASK );
    }
    else
    {
        lorawan_api_tx_ack_bit_set( STACK_ID_CURRENT_TASK, false );
    }
}

static void lorawan_dwn_ack_management_on_update( void* context )
{
}

static uint8_t lorawan_dwn_ack_management_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    if( ( ( rx_down_data->rx_metadata.rx_window == RECEIVE_ON_RX1 ) ||
          ( rx_down_data->rx_metadata.rx_window == RECEIVE_ON_RX2 ) ||
          ( rx_down_data->rx_metadata.rx_window == RECEIVE_ON_RXB ) ||
          ( rx_down_data->rx_metadata.rx_window == RECEIVE_ON_RXC ) ) &&
        ( rx_down_data->rx_metadata.tx_ack_bit == true ) )
    {
        // time to execute is set to 0 secondes to answer before 8 secondes as required in lorawan and because
        // tx randomness could be up to 6s (+ TOA)

        lorawan_dwn_ack_add_task( rx_down_data->stack_id, smtc_modem_hal_get_time_in_s( ) );
    }
    return MODEM_DOWNLINK_UNCONSUMED;
}

/* --- EOF ------------------------------------------------------------------ */
