/**
 * @file      lorawan_join_management.c
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
#include "lorawan_join_management.h"
#include "modem_supervisor_light.h"
#include "smtc_modem_api.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "lorawan_api.h"
#include "modem_core.h"
#include "modem_event_utilities.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define VIRTUAL_TASK_ID ( ( stask_manager* ) context )->next_task_id
#define STACK_ID_CURRENT_TASK ( ( stask_manager* ) context )->modem_task[VIRTUAL_TASK_ID].stack_id
#define NAP 0
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
static void lorawan_join_management_service_on_launch( void* service_id );

/**
 * @brief Callback called at the end of the task
 *
 * @param context_callback
 */
static void lorawan_join_management_service_on_update( void* service_id );

/**
 * @brief Callback to handle the downlink received by the LoRaWAN layer
 *
 * @param rx_down_data
 */
static uint8_t lorawan_join_management_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/**
 * @brief Enqueue a new join
 *
 * @param stack_id
 */
static void    lorawan_join_internal_add_task( uint8_t stack_id, uint32_t current_time_s );
static uint8_t nap = 0;
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void lorawan_join_management_services_init( uint8_t* service_id, uint8_t task_id,
                                            uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                            void ( **on_launch_callback )( void* ),
                                            void ( **on_update_callback )( void* ), void** context_callback )
{
    *downlink_callback  = lorawan_join_management_service_downlink_handler;
    *on_launch_callback = lorawan_join_management_service_on_launch;
    *on_update_callback = lorawan_join_management_service_on_update;
    *context_callback   = ( void* ) modem_supervisor_get_task( );
}

void lorawan_join_add_task( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    stask_manager* context = modem_supervisor_get_task( );
    if( context->modem_task[JOIN_TASK + ( NUMBER_OF_TASKS * stack_id )].priority != TASK_FINISH )
    {
        return;
    }
    lorawan_join_internal_add_task( stack_id, smtc_modem_hal_get_time_in_s( ) );
}

void lorawan_join_remove_task( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    modem_supervisor_remove_task( JOIN_TASK + ( NUMBER_OF_TASKS * stack_id ) );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lorawan_join_management_service_on_launch( void* context )
{
    stask_manager* task_manager = ( stask_manager* ) context;
    if( lorawan_api_isjoined( STACK_ID_CURRENT_TASK ) == JOINED )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "DEVICE ALREADY JOINED\n" );
    }
    else
    {
        lorawan_api_set_join_status( STACK_ID_CURRENT_TASK, JOINING );
        status_lorawan_t status = tx_protocol_manager_request(
            TX_PROTOCOL_JOIN_LORA, NAP, NAP, &nap, NAP, NAP, smtc_modem_hal_get_time_in_ms( ), STACK_ID_CURRENT_TASK );
        if( status == OKLORAWAN )
        {
            task_manager->modem_task[VIRTUAL_TASK_ID].task_context = SUCCESS_TO_LAUNCH_JOIN;
        }
        else
        {
            // can happen in specific cases such as no more duty cycle for wor tx when relay tx is activated
            task_manager->modem_task[VIRTUAL_TASK_ID].task_context = FAIL_TO_LAUNCH_JOIN;
        }
    }
}

static void lorawan_join_management_service_on_update( void* context )
{
    stask_manager* task_manager = ( stask_manager* ) context;

    if( lorawan_api_isjoined( STACK_ID_CURRENT_TASK ) == JOINED )
    {
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_JOINED, 0, STACK_ID_CURRENT_TASK );
    }
    else
    {
        lorawan_api_set_join_status( STACK_ID_CURRENT_TASK, JOINING );
        if( task_manager->modem_task[VIRTUAL_TASK_ID].task_enabled == true )
        {
            if( task_manager->modem_task[VIRTUAL_TASK_ID].task_context == SUCCESS_TO_LAUNCH_JOIN )
            {
                lorawan_join_internal_add_task( STACK_ID_CURRENT_TASK, smtc_modem_hal_get_time_in_s( ) );
            }
            else
            {
                lorawan_join_internal_add_task( STACK_ID_CURRENT_TASK, smtc_modem_hal_get_time_in_s( ) + 120 );
            }
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_JOINFAIL, 0, STACK_ID_CURRENT_TASK );
        }
    }

    // In any case store the modem context here to avoid extrem access to nvm near end of battery life
    modem_store_modem_context( );
}

static uint8_t lorawan_join_management_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    return MODEM_DOWNLINK_UNCONSUMED;
}

static void lorawan_join_internal_add_task( uint8_t stack_id, uint32_t current_time_s )
{
    smodem_task task_join = { 0 };
    task_join.id          = JOIN_TASK + ( NUMBER_OF_TASKS * stack_id );
    task_join.stack_id    = stack_id;
    task_join.priority    = TASK_MEDIUM_HIGH_PRIORITY;

    task_join.time_to_execute_s = smtc_modem_hal_get_random_nb_in_range( 0, 5 );

#if defined( TEST_BYPASS_JOIN_DUTY_CYCLE )
    SMTC_MODEM_HAL_TRACE_WARNING( "BYPASS JOIN DUTY CYCLE activated\n" );
    task_join.time_to_execute_s += current_time_s;
#else
    if( ( lorawan_api_modem_certification_is_enabled( stack_id ) == true ) ||
        ( lorawan_api_join_duty_cycle_backoff_bypass_get( stack_id ) == true ) )
    {
        task_join.time_to_execute_s += current_time_s;
    }
    else
    {
        task_join.time_to_execute_s += lorawan_api_next_join_time_second_get( stack_id );
    }
#endif

    if( ( int32_t ) ( task_join.time_to_execute_s - current_time_s ) <= 0 )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " Start a new join sequence now on stack %u\n", stack_id );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " Start a new join sequence in %d seconds on stack %u\n",
                                     task_join.time_to_execute_s - current_time_s, stack_id );
    }

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( modem_supervisor_add_task( &task_join ) == TASK_VALID );
}
/* --- EOF ------------------------------------------------------------------ */
