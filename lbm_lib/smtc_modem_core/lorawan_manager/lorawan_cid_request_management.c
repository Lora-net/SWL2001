/**
 * @file      lorawan_cid_request_management.c
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
#include "lorawan_cid_request_management.h"
#include "smtc_modem_api.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "modem_supervisor_light.h"
#include "lorawan_api.h"
#include "modem_core.h"
#include "modem_event_utilities.h"
#include "modem_tx_protocol_manager.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define VIRTUAL_TASK_ID ( ( stask_manager* ) context )->next_task_id
#define STACK_ID_CURRENT_TASK ( ( stask_manager* ) context )->modem_task[VIRTUAL_TASK_ID].stack_id
#define CURRENT_TASK_CONTEXT ( ( stask_manager* ) context )->modem_task[VIRTUAL_TASK_ID].task_context

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
#define MAX_NUMBER_OF_CIQ_REQUEST 3

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
static void lorawan_cid_request_management_on_launch( void* context_callback );

/**
 * @brief Callback called at the end of the task
 *
 * @param context_callback
 */
static void lorawan_cid_request_management_on_update( void* context_callback );

/**
 * @brief Callback to handle the downlink received by the LoRaWAN layer
 *
 * @param rx_down_data
 */
static uint8_t lorawan_cid_request_management_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void lorawan_cid_request_management_init( uint8_t* service_id, uint8_t task_id,
                                          uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                          void ( **on_launch_callback )( void* ),
                                          void ( **on_update_callback )( void* ), void** context_callback )
{
    *downlink_callback  = lorawan_cid_request_management_downlink_handler;
    *on_launch_callback = lorawan_cid_request_management_on_launch;
    *on_update_callback = lorawan_cid_request_management_on_update;
    *context_callback   = ( void* ) modem_supervisor_get_task( );
}

lorawan_management_rc_t lorawan_cid_request_add_task( uint8_t stack_id, uint8_t cid_request_mask, uint32_t delay_s )
{
    IS_VALID_STACK_ID( stack_id );
    if( ( cid_request_mask == 0 ) || ( cid_request_mask > 7 ) )
    {
        return LORAWAN_MANAGEMENT_ERR;
    }
    stask_manager* context  = modem_supervisor_get_task( );
    smodem_task    task_cid = { 0 };

    task_cid.id                = CID_REQ_TASK + ( NUMBER_OF_TASKS * stack_id );
    task_cid.stack_id          = stack_id;
    task_cid.priority          = TASK_MEDIUM_HIGH_PRIORITY;
    task_cid.task_context      = context->modem_task[task_cid.id].task_context | cid_request_mask;
    task_cid.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + delay_s;
    SMTC_MODEM_HAL_TRACE_PRINTF( "cid_request_mask:0x%x\n", cid_request_mask );
    modem_supervisor_add_task( &task_cid );

    return LORAWAN_MANAGEMENT_OK;
}

void lorawan_cid_request_remove_task( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    modem_supervisor_remove_task( CID_REQ_TASK + ( NUMBER_OF_TASKS * stack_id ) );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lorawan_cid_request_management_on_launch( void* context )
{
    uint8_t cid_buffer[MAX_NUMBER_OF_CIQ_REQUEST];
    uint8_t cid_request_size = 0;
    if( CURRENT_TASK_CONTEXT & SMTC_MODEM_LORAWAN_MAC_REQ_LINK_CHECK )
    {
        cid_buffer[cid_request_size] = LINK_CHECK_REQ;
        cid_request_size++;
    }
    if( CURRENT_TASK_CONTEXT & SMTC_MODEM_LORAWAN_MAC_REQ_DEVICE_TIME )
    {
        cid_buffer[cid_request_size] = DEVICE_TIME_REQ;
        cid_request_size++;
    }
    if( CURRENT_TASK_CONTEXT & SMTC_MODEM_LORAWAN_MAC_REQ_PING_SLOT_INFO )
    {
        cid_buffer[cid_request_size] = PING_SLOT_INFO_REQ;
        cid_request_size++;
    }

    if( ( cid_request_size > 0 ) && ( cid_request_size <= MAX_NUMBER_OF_CIQ_REQUEST ) )
    {
        tx_protocol_manager_request( TX_PROTOCOL_TRANSMIT_CID, 0, false, cid_buffer, cid_request_size, 0,
                                     smtc_modem_hal_get_time_in_ms( ), STACK_ID_CURRENT_TASK );
    }
}

static void lorawan_cid_request_management_on_update( void* context )
{
    uint8_t data_in_tmp  = CURRENT_TASK_CONTEXT;
    CURRENT_TASK_CONTEXT = 0;
    if( data_in_tmp & SMTC_MODEM_LORAWAN_MAC_REQ_DEVICE_TIME )
    {
        smtc_modem_event_mac_request_status_t dev_time_status = SMTC_MODEM_EVENT_MAC_REQUEST_NOT_ANSWERED;
        if( lorawan_api_get_device_time_req_status( STACK_ID_CURRENT_TASK ) == OKLORAWAN )
        {
            dev_time_status = SMTC_MODEM_EVENT_MAC_REQUEST_ANSWERED;
        }
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_LORAWAN_MAC_TIME, dev_time_status, STACK_ID_CURRENT_TASK );
    }
    if( data_in_tmp & SMTC_MODEM_LORAWAN_MAC_REQ_LINK_CHECK )
    {
        uint8_t                               margin;
        uint8_t                               gw_cnt;
        smtc_modem_event_mac_request_status_t link_check_status = SMTC_MODEM_EVENT_MAC_REQUEST_NOT_ANSWERED;
        if( lorawan_api_get_link_check_ans( &margin, &gw_cnt, STACK_ID_CURRENT_TASK ) == OKLORAWAN )
        {
            link_check_status = SMTC_MODEM_EVENT_MAC_REQUEST_ANSWERED;
        }
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_LINK_CHECK, link_check_status, STACK_ID_CURRENT_TASK );
    }
    if( data_in_tmp & SMTC_MODEM_LORAWAN_MAC_REQ_PING_SLOT_INFO )
    {
        smtc_modem_event_mac_request_status_t ping_slot_info_status = SMTC_MODEM_EVENT_MAC_REQUEST_NOT_ANSWERED;
        if( lorawan_api_get_ping_slot_info_req_status( STACK_ID_CURRENT_TASK ) == OKLORAWAN )
        {
            ping_slot_info_status = SMTC_MODEM_EVENT_MAC_REQUEST_ANSWERED;
        }
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO, ping_slot_info_status,
                                          STACK_ID_CURRENT_TASK );
    }
}

static uint8_t lorawan_cid_request_management_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    return MODEM_DOWNLINK_UNCONSUMED;
}

/* --- EOF ------------------------------------------------------------------ */
