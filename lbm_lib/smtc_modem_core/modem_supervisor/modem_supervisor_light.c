/*!
 * \file      modem_supervisor_light.c
 *
 * \brief     soft modem task scheduler
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
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
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "modem_supervisor_light.h"
#include "lorawan_send_management.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "lorawan_certification.h"
#include "smtc_modem_hal.h"
#include "modem_core.h"
#include "lorawan_api.h"
#include "smtc_modem_api.h"
#include "smtc_modem_test_api.h"
#include "smtc_duty_cycle.h"
#include "modem_event_utilities.h"
#include "modem_tx_protocol_manager.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR1MAC_PERIOD_FAILSAFE_MS 120000
#define LR1MAC_PERIOD_RETRANS_MS 1000
#define MODEM_MAX_ALARM_S 0x7FFFFFFF
#define SUPERVISOR_PERIOD_FAILSAFE_S 120

/*
 *-----------------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------------
 */
#define STACK_ID_CURRENT_TASK task_manager.modem_task[task_manager.next_task_id].stack_id
#define CURRENT_TASK_ID task_manager.next_task_id - ( NUMBER_OF_TASKS * STACK_ID_CURRENT_TASK )

/*!
 * \brief Returns the minimum value between a and b
 *
 * \param [in] a 1st value
 * \param [in] b 2nd value
 * \retval minValue Minimum value
 */
#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

#ifndef MAX
/*!
 * \brief Returns the maximum value between a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval maxValue Maximum value
 */
#define MAX( a, b ) ( ( a ) > ( b ) ) ? ( a ) : ( b )
#endif

#ifndef ABS
/**
 * @brief Math Abs macro
 */
#define ABS( N ) ( ( N < 0 ) ? ( -N ) : ( N ) )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

struct
{
    stask_manager task_manager;

    void* supervisor_context_callback[NUMBER_OF_TASKS];
    void ( *supervisor_on_launch_func[NUMBER_OF_TASKS] )( void* );
    void ( *supervisor_on_update_func[NUMBER_OF_TASKS] )( void* );

    bool is_duty_cycle_constraint_enabled[NUMBER_OF_STACKS];
} modem_supervisor_context;

/* clang-format off */
#define task_manager modem_supervisor_context.task_manager

#define supervisor_context_callback modem_supervisor_context.supervisor_context_callback
#define supervisor_on_launch_func modem_supervisor_context.supervisor_on_launch_func
#define supervisor_on_update_func modem_supervisor_context.supervisor_on_update_func

#define is_duty_cycle_constraint_enabled modem_supervisor_context.is_duty_cycle_constraint_enabled

/* clang-format on */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static uint32_t supervisor_check_user_alarm( void );
static uint32_t supervisor_run_lorawan_engine( uint8_t stack_id );
static uint32_t supervisor_find_next_task( void );

static void supervisor_idle_task_on_launch( void* context );
static void supervisor_idle_task_on_update( void* context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void modem_supervisor_init( void )
{
    memset( &task_manager, 0, sizeof( stask_manager ) );

    for( uint8_t i = 0; i < NUMBER_OF_TASKS * NUMBER_OF_STACKS; i++ )
    {
        task_manager.modem_task[i].priority       = TASK_FINISH;
        task_manager.modem_task[i].id             = ( task_id_t ) i;
        task_manager.modem_task[i].stack_id       = 0;
        task_manager.modem_task[i].updated_locked = false;
    }
    task_manager.next_task_id = IDLE_TASK;

    for( uint8_t i = 0; i < NUMBER_OF_TASKS; i++ )
    {
        supervisor_on_launch_func[i]   = NULL;
        supervisor_on_update_func[i]   = NULL;
        supervisor_context_callback[i] = NULL;
    }

    for( uint8_t i = 0; i < NUMBER_OF_STACKS; i++ )
    {
        task_manager.modem_mute_with_priority[i] = TASK_LOW_PRIORITY;
        is_duty_cycle_constraint_enabled[i]      = false;
    }

    modem_supervisor_init_callback( IDLE_TASK, supervisor_idle_task_on_launch, supervisor_idle_task_on_update,
                                    &task_manager );
}

void modem_supervisor_init_callback( const task_id_t task_id, void ( *callback_on_launch )( void* ),
                                     void ( *callback_on_update )( void* ), void* context_callback )
{
    if( ( supervisor_on_launch_func[task_id] != NULL ) || ( supervisor_on_update_func[task_id] != NULL ) ||
        ( supervisor_context_callback[task_id] != NULL ) || ( callback_on_launch == NULL ) ||
        ( callback_on_update == NULL ) || ( context_callback == NULL ) )
    {
        SMTC_MODEM_HAL_PANIC( );
    }
    else
    {
        supervisor_on_launch_func[task_id]   = callback_on_launch;
        supervisor_on_update_func[task_id]   = callback_on_update;
        supervisor_context_callback[task_id] = context_callback;
    }
}

void modem_supervisor_abort_tasks_in_range( uint16_t id_min, uint16_t id_max )
{
    if( id_max > NUMBER_OF_TASKS * NUMBER_OF_STACKS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "%s id_max not valid\n", __func__ );
        return;
    }

    if( id_min > id_max )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "%s id_min > id_max\n", __func__ );
        return;
    }

    for( uint16_t i = id_min; i <= id_max; i++ )
    {
        modem_supervisor_remove_task( i );
    }

    if( ( id_min <= task_manager.next_task_id ) && ( task_manager.next_task_id <= id_max ) )
    {
        task_manager.next_task_id = IDLE_TASK;
    }
}

task_valid_t modem_supervisor_remove_task( uint16_t id )
{
    if( id < NUMBER_OF_TASKS * NUMBER_OF_STACKS )
    {
        task_manager.modem_task[id].priority     = TASK_FINISH;
        task_manager.modem_task[id].task_enabled = false;
        return TASK_VALID;
    }
    SMTC_MODEM_HAL_TRACE_ERROR( "modem_supervisor_remove_task id = %d unknown\n", id );
    return TASK_NOT_VALID;
}

task_valid_t modem_supervisor_add_task( smodem_task* task )
{
    // the modem supervisor always accept a new task.
    // in case of a previous task is already enqueue , the new task remove the old one.
    // as soon as a task has been elected by the modem supervisor , the task is managed by the stack itself and a new
    // task could be added inside the modem supervisor.

    if( task->id < NUMBER_OF_TASKS * NUMBER_OF_STACKS )
    {
        uint8_t task_index                                    = task->id;
        task_manager.modem_task[task_index].time_to_execute_s = task->time_to_execute_s;
        task_manager.modem_task[task_index].priority          = task->priority;
        task_manager.modem_task[task_index].stack_id          = task->stack_id;
        task_manager.modem_task[task_index].task_context      = task->task_context;
        task_manager.modem_task[task_index].task_enabled      = true;
        task_manager.modem_task[task_index].updated_locked    = task->updated_locked;
        return TASK_VALID;
    }
    SMTC_MODEM_HAL_TRACE_ERROR( "modem_supervisor_add_task id = %d unknown\n", task->id );
    return TASK_NOT_VALID;
}

stask_manager* modem_supervisor_get_task( void )
{
    return ( &task_manager );
}

// backoff_mobile_static( ); @todo//
// todo check_class_b_to_generate_event( );

uint32_t modem_supervisor_engine( void )
{
    uint32_t sleep_time       = 0;
    uint32_t sleep_time_alarm = 0;
    sleep_time                = tx_protocol_manager_is_busy( );
    sleep_time_alarm          = supervisor_check_user_alarm( );
    if( sleep_time > 0 )
    {
        sleep_time = MIN( sleep_time, sleep_time_alarm * 1000 );
        return ( sleep_time );
    }
    sleep_time = supervisor_run_lorawan_engine( STACK_ID_CURRENT_TASK );

    if( sleep_time > 0 )
    {
        sleep_time = MIN( sleep_time, sleep_time_alarm * 1000 );
        return ( sleep_time );
    }

    if( task_manager.modem_task[task_manager.next_task_id].updated_locked == true )
    {
        if( ( int32_t ) ( smtc_modem_hal_get_time_in_s( ) -
                          task_manager.modem_task[task_manager.next_task_id].launched_timestamp -
                          SUPERVISOR_PERIOD_FAILSAFE_S ) > 0 )
        {
            SMTC_MODEM_HAL_PANIC( "Supervisor FAILSAFE EVENT OCCUR task:0x%x)\n", task_manager.next_task_id );
        }
        else
        {
            return SUPERVISOR_PERIOD_FAILSAFE_S * 1000;
        }
    }
    if( task_manager.next_task_id != IDLE_TASK )
    {
        supervisor_on_update_func[CURRENT_TASK_ID]( supervisor_context_callback[CURRENT_TASK_ID] );
        task_manager.next_task_id = IDLE_TASK;
    }
    sleep_time = supervisor_find_next_task( );

    if( sleep_time == 0 )  // launch task
    {
        task_manager.modem_task[task_manager.next_task_id].launched_timestamp = smtc_modem_hal_get_time_in_s( );
        supervisor_on_launch_func[CURRENT_TASK_ID]( supervisor_context_callback[CURRENT_TASK_ID] );
        task_manager.modem_task[task_manager.next_task_id].priority = TASK_FINISH;
    }
    uint32_t alarm                 = modem_get_user_alarm( );
    int32_t  user_alarm_in_seconds = MODEM_MAX_ALARM_S / 1000;
    if( alarm != 0 )
    {
        user_alarm_in_seconds = ( int32_t ) ( alarm - smtc_modem_hal_get_time_in_s( ) );
        if( user_alarm_in_seconds <= 0 )
        {
            user_alarm_in_seconds = 0;
        }
    }
    sleep_time = MIN( sleep_time, ( uint32_t ) user_alarm_in_seconds * 1000 );
    return ( sleep_time );
}

bool modem_supervisor_get_modem_is_suspended( uint8_t stack_id )
{
    return task_manager.modem_is_suspended[stack_id];
}

void modem_supervisor_set_modem_is_suspended( bool suspend, uint8_t stack_id )
{
    task_manager.modem_is_suspended[stack_id] = suspend;
}

task_priority_t modem_supervisor_get_modem_mute_with_priority_parameter( uint8_t stack_id )
{
    return task_manager.modem_mute_with_priority[stack_id];
}

void modem_supervisor_set_modem_mute_with_priority_parameter( task_priority_t priority_level, uint8_t stack_id )
{
    if( priority_level < TASK_FINISH )
    {
        task_manager.modem_mute_with_priority[stack_id] = priority_level;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static uint32_t supervisor_check_user_alarm( void )
{
    uint32_t alarm                 = modem_get_user_alarm( );
    int32_t  user_alarm_in_seconds = MODEM_MAX_ALARM_S;
    // manage the user alarm
    if( alarm != 0 )
    {
        user_alarm_in_seconds = ( int32_t ) ( alarm - smtc_modem_hal_get_time_in_s( ) );

        if( user_alarm_in_seconds <= 0 )
        {
            modem_set_user_alarm( 0 );
            user_alarm_in_seconds = MODEM_MAX_ALARM_S;
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_ALARM, 0, 0xFF );
        }
    }

    return ( uint32_t ) user_alarm_in_seconds;
}

static uint32_t supervisor_run_lorawan_engine( uint8_t stack_id )
{
    uint32_t sleep_time = 0;

    lr1mac_states_t lorawan_state = lorawan_api_process( stack_id );

    if( lorawan_state == LWPSTATE_TX_WAIT )
    {
        tx_protocol_manager_lr1mac_stand_alone_tx( );
        sleep_time = ( LR1MAC_PERIOD_RETRANS_MS );
    }
    else if( ( lorawan_state != LWPSTATE_IDLE ) && ( lorawan_state != LWPSTATE_ERROR ) )
    {
        sleep_time = ( LR1MAC_PERIOD_FAILSAFE_MS );
    }

    return sleep_time;
}

static uint32_t supervisor_find_next_task( void )
{
    // Find stacks that can continue to send uplink frame in regard of duty-cycle regulation
    int32_t dtc_ms                            = MODEM_MAX_TIME;
    uint8_t available_stack[NUMBER_OF_STACKS] = { 0 };

    for( uint8_t i = 0; i < NUMBER_OF_STACKS; i++ )
    {
        int32_t dtc_ms_tmp = modem_duty_cycle_get_status( i );
        if( dtc_ms_tmp <= 0 )
        {
            available_stack[i] = 1;
        }
        if( dtc_ms > dtc_ms_tmp )
        {
            dtc_ms = dtc_ms_tmp;
        }

        // Generate event for duty-cycle busy
        if( available_stack[i] == 0 )
        {
            if( is_duty_cycle_constraint_enabled[i] == false )
            {
                is_duty_cycle_constraint_enabled[i] = true;
                increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_REGIONAL_DUTY_CYCLE, 1, i );
            }
        }
        // Generate event for duty-cycle free
        else
        {
            if( is_duty_cycle_constraint_enabled[i] == true )
            {
                is_duty_cycle_constraint_enabled[i] = false;
                increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_REGIONAL_DUTY_CYCLE, 0, i );
            }
        }
    }

    task_priority_t next_task_priority = TASK_FINISH;
    int32_t         next_task_time     = MODEM_MAX_TIME;

    // Find the highest priority task in the past
    for( uint8_t stack_id = 0; stack_id < NUMBER_OF_STACKS; stack_id++ )
    {
        for( task_id_t k = 0; k < NUMBER_OF_TASKS; k++ )
        {
            uint8_t i = ( stack_id * NUMBER_OF_TASKS ) + k;
            if( ( task_manager.modem_task[i].priority != TASK_FINISH ) &&
                ( task_manager.modem_task[i].priority <= task_manager.modem_mute_with_priority[stack_id] ) &&
                ( task_manager.modem_is_suspended[stack_id] == false ) )
            {
                int32_t next_task_time_tmp =
                    ( int32_t ) ( task_manager.modem_task[i].time_to_execute_s - smtc_modem_hal_get_time_in_s( ) );

                if( ( next_task_time_tmp <= 0 ) && ( task_manager.modem_task[i].priority <= next_task_priority ) &&
                    ( next_task_time_tmp <= next_task_time ) &&
                    ( ( available_stack[stack_id] == 1 ) ||
                      ( ( available_stack[stack_id] == 0 ) &&
                        ( task_manager.modem_task[i].priority == TASK_BYPASS_DUTY_CYCLE ) ) ) )
                {
                    next_task_priority        = task_manager.modem_task[i].priority;
                    next_task_time            = next_task_time_tmp;
                    task_manager.next_task_id = ( task_id_t ) i;
                }
            }
        }
    }

    // No task in the past was found, select the least in the future for wake up
    if( next_task_priority == TASK_FINISH )
    {
        for( uint8_t stack_id = 0; stack_id < NUMBER_OF_STACKS; stack_id++ )
        {
            for( task_id_t k = 0; k < NUMBER_OF_TASKS; k++ )
            {
                uint8_t i = ( stack_id * NUMBER_OF_TASKS ) + k;
                if( ( task_manager.modem_task[i].priority != TASK_FINISH ) &&
                    ( task_manager.modem_task[i].priority <= task_manager.modem_mute_with_priority[stack_id] ) &&
                    ( task_manager.modem_is_suspended[stack_id] == false ) )
                {
                    int32_t next_task_time_tmp =
                        ( int32_t ) ( task_manager.modem_task[i].time_to_execute_s - smtc_modem_hal_get_time_in_s( ) );
                    if( ( next_task_time_tmp < next_task_time ) &&
                        ( ( available_stack[stack_id] == 1 ) ||
                          ( ( available_stack[stack_id] == 0 ) &&
                            ( task_manager.modem_task[i].priority == TASK_BYPASS_DUTY_CYCLE ) ) ) )
                    {
                        next_task_time            = next_task_time_tmp;
                        task_manager.next_task_id = ( task_id_t ) i;
                    }
                }
            }
        }
    }
    if( ( dtc_ms > 0 ) && ( next_task_time == MODEM_MAX_TIME ) )
    {
        SMTC_MODEM_HAL_TRACE_WARNING_DEBUG( "Duty Cycle, remaining time: %dms\n", dtc_ms );
        task_manager.next_task_id = IDLE_TASK;
        return ( dtc_ms );
    }

    if( next_task_time > 0 )
    {
        task_manager.next_task_id = IDLE_TASK;
        return ( next_task_time * 1000 );
    }
    else
    {
        return 0;
    }
}

static void supervisor_idle_task_on_launch( void* context )
{
}
static void supervisor_idle_task_on_update( void* context )
{
}
