/*!
 * \file      radio_planner.c
 *
 * \brief     Radio planner implementation
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

#include <stdlib.h>
#include <stdio.h>
#include "radio_planner.h"
#include "smtc_modem_hal_dbg_trace.h"

#include <string.h>

//
// Private planner utilities declaration
//

/**
 * @brief rp_task_free to free a task
 *
 * @param rp  pointer to the radioplaner object itself
 * @param task pointer to the task that function free
 */
static void rp_task_free( const radio_planner_t* rp, rp_task_t* task );

/**
 * @brief rp_task_update_time update task time
 *
 * @param rp pointer to the radioplaner object itself
 * @param now the current time in ms
 */
static void rp_task_update_time( radio_planner_t* rp, uint32_t now );

/**
 * @brief rp_task_arbiter the core of the radio planer
 *
 * @param rp pointer to the radioplaner object itself
 * @param caller_func_name input just for print
 */
static void rp_task_arbiter( radio_planner_t* rp, const char* caller_func_name );

/**
 * @brief rp_irq_get_status get the radio status after irq
 *
 * @param  pointer to the radioplaner object itself
 * @param hook_id id of the targeted hook
 */
static void rp_irq_get_status( radio_planner_t* rp, const uint8_t hook_id );

/**
 * @brief rp_task_compute_ranking compute the ranking of the different tasks inside the radio planer
 *
 * @param rp pointer to the radioplaner object itself
 */
static void rp_task_compute_ranking( radio_planner_t* rp );

/**
 * @brief rp_task_launch_current call  the launch callback of the new running task
 *
 * @param rp  pointer to the radioplaner object itself
 */
static void rp_task_launch_current( radio_planner_t* rp );

/**
 * @brief rp_task_select_next select the next most priority task
 *
 * @param rp pointer to the radioplaner object itself
 * @param now the current time in ms
 * @return uint8_t
 */
static uint8_t rp_task_select_next( radio_planner_t* rp, const uint32_t now );

/**
 * @brief rp_task_get_next return if there is a task to schedule or no more task
 *
 * @param rp pointer to the radioplaner object itself
 * @param duration return the delay to the next task (to set the timer)
 * @param task_id return the task id of the next task to launch
 * @param now the current time
 * @return rp_next_state_status_t return if it have to set a timer or if there is no more task in rp
 */
static rp_next_state_status_t rp_task_get_next( radio_planner_t* rp, uint32_t* duration, uint8_t* task_id,
                                                const uint32_t now );
/**
 * @brief rp_task_find_highest_priority utilities to classify priority task (value 0 is the highest priority)
 *
 * @param vector a vector of value to classify from the lowest to the highest
 * @param length size of the vector
 * @return uint8_t index of the vector to indicate the position inside the vector of the minimum value
 */
static uint8_t rp_task_find_highest_priority( uint8_t* vector, uint8_t length );

/**
 * @brief rp_get_pkt_payload get the receive payload
 *
 * @param rp pointer to the radioplaner object itself
 * @param task the task containing the received payload
 * @return rp_hook_status_t
 */
rp_hook_status_t rp_get_pkt_payload( radio_planner_t* rp, const rp_task_t* task );

/**
 * @brief rp_set_alarm configure the radio planer timer
 *
 * @param rp pointer to the radioplaner object itself
 * @param alarm_in_ms delay in ms (relative value)
 */
static void rp_set_alarm( radio_planner_t* rp, const uint32_t alarm_in_ms );

/**
 * @brief rp_timer_irq function call by the timer callback
 *
 * @param rp pointer to the radioplaner object itself
 */
static void rp_timer_irq( radio_planner_t* rp );

/**
 * @brief rp_task_call_aborted excute the callback of the aborted tasks
 *
 * @param rp pointer to the radioplaner object itself
 */
static void rp_task_call_aborted( radio_planner_t* rp );
/**
 * @brief rp_consumption_statistics_updated compute the statistic (power consumption)
 *
 * @param rp pointer to the radioplaner object itself
 * @param hook_id hook id on which statistics are perform
 * @param time the current time in ms
 */
static void rp_consumption_statistics_updated( radio_planner_t* rp, const uint8_t hook_id, const uint32_t time );
/**
 * @brief rp_radio_irq radio callback
 *
 * @param rp pointer to the radioplaner object itself
 */

static void rp_radio_irq( radio_planner_t* rp );
/**
 * @brief rp_timer_irq_callback timer callback
 *
 * @param obj pointer to the radioplaner object itself
 */
static void rp_timer_irq_callback( void* obj );

/**
 * @brief rp_hook_callback call the callback associated to the id
 *
 * @param rp pointer to the radioplaner object itself
 * @param id target hook id
 */
static void rp_hook_callback( radio_planner_t* rp, uint8_t id );

/**
 * @brief rp_task_print debug print function for rp
 *
 * @param rp pointer to the radioplaner object itself
 * @param task target task to print
 */
static void rp_task_print( const radio_planner_t* rp, const rp_task_t* task );

//
// Public planner API implementation
//

void rp_init( radio_planner_t* rp, const ralf_t* radio )
{
    memset( rp, 0, sizeof( radio_planner_t ) );
    rp->radio = radio;

    for( int32_t i = 0; i < RP_NB_HOOKS; i++ )
    {
        rp->tasks[i].hook_id                    = i;
        rp->tasks[i].type                       = RP_TASK_TYPE_NONE;
        rp->tasks[i].launch_task_callbacks      = NULL;
        rp->tasks[i].state                      = RP_TASK_STATE_FINISHED;
        rp->tasks[i].schedule_task_low_priority = false;
        rp->hooks[i]                            = NULL;
        rp->tasks[i].launch_task_callbacks      = NULL;
        rp->hook_callbacks[i]                   = NULL;
        rp->status[i]                           = RP_STATUS_TASK_INIT;
    }
    rp->priority_task.type  = RP_TASK_TYPE_NONE;
    rp->priority_task.state = RP_TASK_STATE_FINISHED;
    rp_stats_init( &rp->stats );

    rp->next_state_status = RP_STATUS_NO_MORE_TASK_SCHEDULE;
    rp->margin_delay      = RP_MARGIN_DELAY;
}

rp_hook_status_t rp_hook_init( radio_planner_t* rp, const uint8_t id, void ( *callback )( void* context ), void* hook )
{
    if( id >= RP_NB_HOOKS )
    {
        smtc_modem_hal_mcu_panic( );
        return RP_HOOK_STATUS_ID_ERROR;
    }
    if( ( rp->hook_callbacks[id] != NULL ) || ( callback == NULL ) )
    {
        smtc_modem_hal_mcu_panic( );
        return RP_HOOK_STATUS_ID_ERROR;
    }
    rp->status[id]         = RP_STATUS_TASK_INIT;
    rp->hook_callbacks[id] = callback;
    rp->hooks[id]          = hook;
    return RP_HOOK_STATUS_OK;
}

rp_hook_status_t rp_release_hook( radio_planner_t* rp, uint8_t id )
{
    if( id >= RP_NB_HOOKS )
    {
        smtc_modem_hal_mcu_panic( );
        return RP_HOOK_STATUS_ID_ERROR;
    }

    rp->hook_callbacks[id]                   = NULL;
    rp->tasks[id].schedule_task_low_priority = false;
    return RP_HOOK_STATUS_OK;
}

rp_hook_status_t rp_hook_get_id( const radio_planner_t* rp, const void* hook, uint8_t* id )
{
    for( int32_t i = 0; i < RP_NB_HOOKS; i++ )
    {
        if( hook == rp->hooks[i] )
        {
            *id = i;
            return RP_HOOK_STATUS_OK;
        }
    }
    smtc_modem_hal_mcu_panic( );
    return RP_HOOK_STATUS_ID_ERROR;
}

rp_hook_status_t rp_task_enqueue( radio_planner_t* rp, const rp_task_t* task, uint8_t* payload, uint16_t payload_size,
                                  const rp_radio_params_t* radio_params )
{
    uint8_t hook_id = task->hook_id;
    if( hook_id >= RP_NB_HOOKS )
    {
        smtc_modem_hal_mcu_panic( );
        return RP_HOOK_STATUS_ID_ERROR;
    }
    if( ( task->launch_task_callbacks == NULL ) || ( rp->hook_callbacks[hook_id] == NULL ) )
    {
        smtc_modem_hal_mcu_panic( );
        return RP_HOOK_STATUS_ID_ERROR;
    }
    if( ( task->state ) > RP_TASK_STATE_ASAP )
    {
        smtc_modem_hal_mcu_panic( " task invalid\n" );
        return RP_HOOK_STATUS_ID_ERROR;
    }
    uint32_t now = rp_hal_get_time_in_ms( );

    if( ( task->state == RP_TASK_STATE_SCHEDULE ) && ( ( ( int32_t )( task->start_time_ms - now ) <= 0 ) ) )
    {
        return RP_TASK_STATUS_SCHEDULE_TASK_IN_PAST;
    }

    if( rp->tasks[hook_id].state == RP_TASK_STATE_RUNNING )
    {
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( " RP: Task enqueue impossible. Task is already running\n" );
        return RP_TASK_STATUS_ALREADY_RUNNING;
    }
    rp_hal_critical_section_begin( );
    if( rp->tasks[hook_id].state != RP_TASK_STATE_FINISHED )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " RP: WARNING Task is already running\n" );
    }
    rp->status[hook_id]       = RP_STATUS_TASK_INIT;
    rp->tasks[hook_id]        = *task;
    rp->radio_params[hook_id] = *radio_params;
    rp->payload[hook_id]      = payload;
    rp->payload_size[hook_id] = payload_size;
    if( rp->tasks[hook_id].schedule_task_low_priority == true )
    {
        rp->tasks[hook_id].priority = ( RP_TASK_STATE_ASAP * RP_NB_HOOKS ) + hook_id;
    }
    else
    {
        rp->tasks[hook_id].priority = ( rp->tasks[hook_id].state * RP_NB_HOOKS ) + hook_id;
    }
    rp->tasks[hook_id].start_time_init_ms = rp->tasks[hook_id].start_time_ms;
    SMTC_MODEM_HAL_RP_TRACE_PRINTF( "RP: Task #%u enqueue with #%u priority\n", hook_id, rp->tasks[hook_id].priority );
    rp_task_compute_ranking( rp );
    if( rp->semaphore_radio == 0 )
    {
        rp_task_arbiter( rp, __func__ );
    }
    rp_hal_critical_section_end( );
    return RP_HOOK_STATUS_OK;
}

rp_hook_status_t rp_task_abort( radio_planner_t* rp, const uint8_t hook_id )
{
    SMTC_MODEM_HAL_RP_TRACE_PRINTF( " RP: rp_task_abort \n" );
    rp_hal_critical_section_begin( );
    if( hook_id >= RP_NB_HOOKS )
    {
        rp_hal_critical_section_end( );
        smtc_modem_hal_mcu_panic( );
        return RP_HOOK_STATUS_ID_ERROR;
    }

    if( rp->tasks[hook_id].state > RP_TASK_STATE_ABORTED )
    {
        rp_hal_critical_section_end( );
        return RP_HOOK_STATUS_OK;
    }

    if( rp->tasks[hook_id].state == RP_TASK_STATE_RUNNING )
    {
        rp_radio_irq( rp );
    }
    else
    {
        rp->tasks[hook_id].state = RP_TASK_STATE_ABORTED;

        if( rp->semaphore_radio == 0 )
        {
            rp_task_arbiter( rp, __func__ );
        }
    }
    rp_hal_critical_section_end( );
    return RP_HOOK_STATUS_OK;
}

void rp_get_status( const radio_planner_t* rp, const uint8_t id, uint32_t* irq_timestamp_ms, rp_status_t* status )
{
    if( id >= RP_NB_HOOKS )
    {
        rp_hal_critical_section_end( );
        smtc_modem_hal_mcu_panic( );
        return;
    }
    *irq_timestamp_ms = rp->irq_timestamp_ms[id];
    *status           = rp->status[id];
}

void rp_get_and_clear_raw_radio_irq( radio_planner_t* rp, const uint8_t id, ral_irq_t* raw_radio_irq )
{
    if( id >= RP_NB_HOOKS )
    {
        rp_hal_critical_section_end( );
        smtc_modem_hal_mcu_panic( );
        return;
    }
    *raw_radio_irq        = rp->raw_radio_irq[id];
    rp->raw_radio_irq[id] = 0;
}
rp_stats_t rp_get_stats( const radio_planner_t* rp )
{
    return rp->stats;
}

void rp_radio_irq( radio_planner_t* rp )
{
    if( rp->tasks[rp->radio_task_id].state < RP_TASK_STATE_ABORTED )
    {
        rp->semaphore_radio = 1;

        uint32_t irq_timestamp_100us               = rp_hal_get_radio_irq_timestamp_in_100us( );
        rp->irq_timestamp_100us[rp->radio_task_id] = irq_timestamp_100us;
        rp->irq_timestamp_ms[rp->radio_task_id]    = rp_hal_get_time_in_ms( );
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( " RP: INFO - Radio IRQ received for hook #%u\n", rp->radio_task_id );

        rp_irq_get_status( rp, rp->radio_task_id );
        if( rp->status[rp->radio_task_id] == RP_STATUS_LR_FHSS_HOP )
        {
            return;
        }

        // Tx can be performed only if no activity detected on channel
        if( ( rp->status[rp->radio_task_id] == RP_STATUS_CAD_NEGATIVE ) &&
            ( rp->tasks[rp->radio_task_id].type == RP_TASK_TYPE_CAD_TO_TX ) )
        {
            rp_hook_callback( rp, rp->radio_task_id );
            return;
        }

        // Rx can be performed if activity detected on channel
        if( ( rp->status[rp->radio_task_id] == RP_STATUS_CAD_POSITIVE ) &&
            ( rp->tasks[rp->radio_task_id].type == RP_TASK_TYPE_CAD_TO_RX ) )
        {
            rp_hook_callback( rp, rp->radio_task_id );
            return;
        }
        rp_consumption_statistics_updated( rp, rp->radio_task_id, rp->irq_timestamp_ms[rp->radio_task_id] );

        // Have to call rp_task_free before rp_hook_callback because the callback can enqueued a task and so call the
        // arbiter
        rp_task_free( rp, &rp->tasks[rp->radio_task_id] );
        smtc_modem_hal_assert( ral_set_sleep( &( rp->radio->ral ), true ) == RAL_STATUS_OK );
        rp_hook_callback( rp, rp->radio_task_id );

        rp_task_call_aborted( rp );

        rp->semaphore_radio = 0;

        rp_task_arbiter( rp, __func__ );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " radio planner it but no more task activated\n" );
    }

    // Shut Down the TCXO
    smtc_modem_hal_stop_radio_tcxo( );
}
//
// Private planner utilities implementation
//

static void rp_task_free( const radio_planner_t* rp, rp_task_t* task )
{
    task->hook_id            = RP_NB_HOOKS;
    task->start_time_ms      = 0;
    task->start_time_init_ms = 0;
    task->duration_time_ms   = 0;
    //   task->type               = RP_TASK_TYPE_NONE; doesn't clear for suspend feature
    task->state                      = RP_TASK_STATE_FINISHED;
    task->schedule_task_low_priority = false;
}

static void rp_task_update_time( radio_planner_t* rp, uint32_t now )
{
    for( int32_t i = 0; i < RP_NB_HOOKS; i++ )
    {
        if( rp->tasks[i].state == RP_TASK_STATE_ASAP )
        {
            if( ( int32_t )( now - rp->tasks[i].start_time_init_ms ) > 0 )
            {
                rp->tasks[i].start_time_ms = now;
            }
            // An asap task is automatically switch in schedule task after RP_TASK_ASAP_TO_SCHEDULE_TRIG_TIME ms

            if( ( int32_t )( now - rp->tasks[i].start_time_init_ms ) > RP_TASK_ASAP_TO_SCHEDULE_TRIG_TIME )
            {
                rp->tasks[i].state = RP_TASK_STATE_SCHEDULE;
                // Schedule the task @ now + RP_TASK_RE_SCHEDULE_OFFSET_TIME
                // seconds
                rp->tasks[i].start_time_ms = now + RP_TASK_RE_SCHEDULE_OFFSET_TIME;
                if( rp->tasks[i].schedule_task_low_priority == true )
                {
                    rp->tasks[i].priority = ( RP_TASK_STATE_ASAP * RP_NB_HOOKS ) + i;
                }
                else
                {
                    rp->tasks[i].priority = ( rp->tasks[i].state * RP_NB_HOOKS ) + i;
                }

                SMTC_MODEM_HAL_RP_TRACE_PRINTF( "RP: WARNING - SWITCH TASK FROM ASAP TO SCHEDULE \n" );
                rp_task_compute_ranking( rp );
            }
        }
    }

    if( ( rp->tasks[rp->radio_task_id].state == RP_TASK_STATE_RUNNING ) &&
        ( ( rp->tasks[rp->radio_task_id].type == RP_TASK_TYPE_RX_LORA ) ||
          ( rp->tasks[rp->radio_task_id].type == RP_TASK_TYPE_RX_FSK ) ) )
    {
        rp->tasks[rp->radio_task_id].duration_time_ms =
            now + rp->margin_delay + 2 - rp->tasks[rp->radio_task_id].start_time_ms;
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( " RP: Extended duration of radio task #%u time to %lu ms\n", rp->radio_task_id,
                                        now );
    }
}

static void rp_task_arbiter( radio_planner_t* rp, const char* caller_func_name )
{
    uint32_t now = rp_hal_get_time_in_ms( );

    // Update time for ASAP task to now. But, also extended duration in case of running task is a RX task
    rp_task_update_time( rp, now );

    // Select the high priority task
    if( rp_task_select_next( rp, now ) == RP_SOMETHING_TO_DO )
    {  // Next task exists
        int32_t delay = ( int32_t )( rp->priority_task.start_time_ms - now );
        SMTC_MODEM_HAL_RP_TRACE_PRINTF(
            " RP: Arbiter has been called by %s and priority-task #%d, timer hook #%d, delay %d, now %d\n ",
            caller_func_name, rp->priority_task.hook_id, rp->timer_hook_id, delay, now );

        // Case where the high priority task is in the past, error case
        if( delay < 0 )
        {  // The high priority task is in the past, error case
            if( rp->priority_task.state != RP_TASK_STATE_RUNNING )
            {
                rp->stats.rp_error++;
                SMTC_MODEM_HAL_TRACE_ERROR( " RP: ERROR - delay #%d - hook #%d\n", delay, rp->priority_task.hook_id );

                rp->tasks[rp->priority_task.hook_id].state = RP_TASK_STATE_ABORTED;
            }
        }
        // Case where the high priority task is in the future
        else if( ( uint32_t ) delay > rp->margin_delay )
        {  // The high priority task is in the future
            SMTC_MODEM_HAL_RP_TRACE_PRINTF( " RP: High priority task is in the future\n" );
        }
        // Case where the high priority task is now
        else
        {
            if( rp->tasks[rp->radio_task_id].state == RP_TASK_STATE_RUNNING )
            {  // Radio is already running
                if( rp->tasks[rp->radio_task_id].hook_id != rp->priority_task.hook_id )
                {  // priority task not equal to radio task => abort radio task
                    rp->tasks[rp->radio_task_id].state = RP_TASK_STATE_ABORTED;
                    SMTC_MODEM_HAL_RP_TRACE_PRINTF( "RP: Abort running task with hook #%u\n", rp->radio_task_id );

                    smtc_modem_hal_assert( ral_set_standby( &( rp->radio->ral ), RAL_STANDBY_CFG_RC ) ==
                                           RAL_STATUS_OK );
                    smtc_modem_hal_assert( ral_clear_irq_status( &( rp->radio->ral ), RAL_IRQ_ALL ) == RAL_STATUS_OK );

                    rp_hal_irq_clear_pending( );

                    smtc_modem_hal_assert( ral_set_sleep( &( rp->radio->ral ), true ) == RAL_STATUS_OK );

                    // Shut Down the TCXO
                    smtc_modem_hal_stop_radio_tcxo( );

                    rp_consumption_statistics_updated( rp, rp->radio_task_id, rp_hal_get_time_in_ms( ) );

                    rp->radio_task_id                  = rp->priority_task.hook_id;
                    rp->tasks[rp->radio_task_id].state = RP_TASK_STATE_RUNNING;
                    rp_task_launch_current( rp );
                }  // else case already managed during enqueue task
            }
            else
            {  // Radio is sleeping start priority task on radio
                rp->radio_task_id                  = rp->priority_task.hook_id;
                rp->tasks[rp->radio_task_id].state = RP_TASK_STATE_RUNNING;
                rp_task_launch_current( rp );
            }
        }
        // Timer has expired on a not priority task => Have to abort this task
        int32_t tmp = ( int32_t )( rp->tasks[rp->timer_hook_id].start_time_ms - now );

        if( tmp > 0 )
        {
            if( ( ( uint32_t ) tmp < rp->margin_delay ) && ( rp->next_state_status == RP_STATUS_HAVE_TO_SET_TIMER ) &&
                ( rp->timer_hook_id != rp->priority_task.hook_id ) &&
                ( rp->tasks[rp->timer_hook_id].state == RP_TASK_STATE_SCHEDULE ) )
            {
                SMTC_MODEM_HAL_TRACE_WARNING( " RP: Aborted task with hook #%u - not a priority task\n ",
                                              rp->timer_hook_id );
                rp->tasks[rp->timer_hook_id].state = RP_TASK_STATE_ABORTED;
            }
        }
        // Execute the garbage collection if the radio isn't running
        if( rp->tasks[rp->radio_task_id].state != RP_TASK_STATE_RUNNING )
        {
            rp_task_call_aborted( rp );
        }

        // Set the Timer to the next Task
        rp->next_state_status = rp_task_get_next( rp, &rp->timer_value, &rp->timer_hook_id, rp_hal_get_time_in_ms( ) );

        if( rp->next_state_status == RP_STATUS_HAVE_TO_SET_TIMER )
        {
            if( rp->timer_value > rp->margin_delay )
            {
                rp_set_alarm( rp, rp->timer_value - rp->margin_delay );
            }
            else
            {
                rp_set_alarm( rp, 1 );
            }
        }
    }
    else
    {  // No more tasks in the radio planner
        rp_task_call_aborted( rp );
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( " RP: No more active tasks\n" );
    }
}

static void rp_irq_get_status( radio_planner_t* rp, const uint8_t hook_id )
{
    ral_irq_t radio_irq = 0;

    if( ( rp->tasks[hook_id].type == RP_TASK_TYPE_LBT ) || ( rp->tasks[hook_id].type == RP_TASK_TYPE_WIFI_SNIFF ) ||
        ( rp->tasks[hook_id].type == RP_TASK_TYPE_GNSS_SNIFF ) )
    {
        return;
    }

    if( ral_get_and_clear_irq_status( &( rp->radio->ral ), &radio_irq ) != RAL_STATUS_OK )
    {
        smtc_modem_hal_mcu_panic( );
    }

    // Do not modify the order of the next if / else if process
    rp->raw_radio_irq[hook_id] = radio_irq;
    if( ( radio_irq & RAL_IRQ_TX_DONE ) == RAL_IRQ_TX_DONE )
    {
        rp->status[hook_id] = RP_STATUS_TX_DONE;
        if( rp->priority_task.type == RP_TASK_TYPE_TX_LR_FHSS )
        {
            smtc_modem_hal_assert( ral_lr_fhss_handle_tx_done( &rp->radio->ral,
                                                               &rp->radio_params[hook_id].tx.lr_fhss.ral_lr_fhss_params,
                                                               NULL ) == RAL_STATUS_OK );
        }
    }
    else if( ( ( radio_irq & RAL_IRQ_RX_HDR_ERROR ) == RAL_IRQ_RX_HDR_ERROR ) ||
             ( ( radio_irq & RAL_IRQ_RX_CRC_ERROR ) == RAL_IRQ_RX_CRC_ERROR ) )
    {
        rp->status[hook_id] = RP_STATUS_RX_CRC_ERROR;
    }
    else if( ( radio_irq & RAL_IRQ_RX_TIMEOUT ) == RAL_IRQ_RX_TIMEOUT )
    {
        rp->status[hook_id] = RP_STATUS_RX_TIMEOUT;
    }
    else if( ( radio_irq & RAL_IRQ_RX_DONE ) == RAL_IRQ_RX_DONE )
    {
        rp->status[hook_id] = RP_STATUS_RX_PACKET;

        if( rp_get_pkt_payload( rp, &rp->tasks[hook_id] ) == RP_HOOK_STATUS_ID_ERROR )
        {
            smtc_modem_hal_mcu_panic( );
            return;
        }
    }
    else if( ( radio_irq & RAL_IRQ_CAD_OK ) == RAL_IRQ_CAD_OK )
    {
        rp->status[hook_id] = RP_STATUS_CAD_POSITIVE;
    }
    else if( ( radio_irq & RAL_IRQ_CAD_DONE ) == RAL_IRQ_CAD_DONE )
    {
        rp->status[hook_id] = RP_STATUS_CAD_NEGATIVE;
    }
    else if( ( radio_irq & RAL_IRQ_LR_FHSS_HOP ) == RAL_IRQ_LR_FHSS_HOP )
    {
        rp->status[hook_id] = RP_STATUS_LR_FHSS_HOP;
        smtc_modem_hal_assert(
            ral_lr_fhss_handle_hop( &rp->radio->ral, &rp->radio_params[hook_id].tx.lr_fhss.ral_lr_fhss_params,
                                    ( ral_lr_fhss_memory_state_t ) rp->radio_params[hook_id].lr_fhss_state ) ==
            RAL_STATUS_OK );
    }
    else if( ( radio_irq & RAL_IRQ_WIFI_SCAN_DONE ) == RAL_IRQ_WIFI_SCAN_DONE )
    {
        rp->status[hook_id] = RP_STATUS_WIFI_SCAN_DONE;
    }
    else if( ( radio_irq & RAL_IRQ_GNSS_SCAN_DONE ) == RAL_IRQ_GNSS_SCAN_DONE )
    {
        rp->status[hook_id] = RP_STATUS_GNSS_SCAN_DONE;
    }
    else
    {
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( " RP: ERROR - IRQ source 0x%04X unknown\n", radio_irq );
        rp->status[hook_id] = RP_STATUS_TASK_ABORTED;
    }
}

static void rp_task_compute_ranking( radio_planner_t* rp )
{
    uint8_t rank;
    uint8_t ranks_temp[RP_NB_HOOKS];

    for( int32_t i = 0; i < RP_NB_HOOKS; i++ )
    {
        ranks_temp[i] = rp->tasks[i].priority;
    }
    for( int32_t i = 0; i < RP_NB_HOOKS; i++ )
    {
        rank             = rp_task_find_highest_priority( &( ranks_temp[0] ), RP_NB_HOOKS );
        ranks_temp[rank] = 0xFF;
        rp->rankings[i]  = rank;
    }
}

static void rp_task_launch_current( radio_planner_t* rp )
{
    uint8_t id = rp->radio_task_id;
    SMTC_MODEM_HAL_RP_TRACE_PRINTF( " RP: Launch task #%u and start radio state %u, type %u\n", id, rp->tasks[id].state,
                                    rp->tasks[id].type );
    if( rp->tasks[id].launch_task_callbacks == NULL )
    {
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( " RP: ERROR - launch_task_callbacks == NULL \n" );
    }
    else
    {
        rp_task_print( rp, &rp->tasks[id] );
        rp->tasks[id].launch_task_callbacks( ( void* ) rp );
    }
}

static uint8_t rp_task_select_next( radio_planner_t* rp, const uint32_t now )
{
    uint8_t  hook_to_exe_tmp      = 0xFF;
    uint32_t hook_time_to_exe_tmp = 0;
    uint32_t time_tmp             = 0;
    uint8_t  rank                 = 0;
    uint8_t  hook_id              = 0;

    for( hook_id = 0; hook_id < RP_NB_HOOKS; hook_id++ )
    {  // Garbage collector
        if( ( rp->tasks[hook_id].state == RP_TASK_STATE_SCHEDULE ) &&
            ( ( ( int32_t )( rp->tasks[hook_id].start_time_ms - now ) < 0 ) ) )
        {
            rp->tasks[hook_id].state = RP_TASK_STATE_ABORTED;
        }
    }
    for( hook_id = 0; hook_id < RP_NB_HOOKS; hook_id++ )
    {
        rank = rp->rankings[hook_id];
        if( ( ( rp->tasks[rank].state < RP_TASK_STATE_RUNNING ) &&
              ( ( int32_t )( rp->tasks[rank].start_time_ms - now ) >= 0 ) ) ||
            ( rp->tasks[rank].state == RP_TASK_STATE_RUNNING ) )
        {
            hook_to_exe_tmp      = rp->tasks[rank].hook_id;
            hook_time_to_exe_tmp = rp->tasks[rank].start_time_ms;
            break;
        }
    }
    if( hook_id == RP_NB_HOOKS )
    {
        return RP_NO_MORE_TASK;
    }

    for( int32_t i = hook_id; i < RP_NB_HOOKS; i++ )
    {
        rank = rp->rankings[i];
        if( ( ( rp->tasks[rank].state < RP_TASK_STATE_RUNNING ) &&
              ( ( int32_t )( rp->tasks[rank].start_time_ms - now ) >= 0 ) ) ||
            ( rp->tasks[rank].state == RP_TASK_STATE_RUNNING ) )
        {
            time_tmp = rp->tasks[rank].start_time_ms + rp->tasks[rank].duration_time_ms;

            int32_t tmp = ( int32_t )( time_tmp - hook_time_to_exe_tmp );
            if( ( tmp < 0 ) && ( ( int32_t )( time_tmp - now ) >= 0 ) )
            {
                hook_to_exe_tmp      = rp->tasks[rank].hook_id;
                hook_time_to_exe_tmp = rp->tasks[rank].start_time_ms;
            }
        }
    }
    rp->priority_task = rp->tasks[hook_to_exe_tmp];
    return RP_SOMETHING_TO_DO;
}

static rp_next_state_status_t rp_task_get_next( radio_planner_t* rp, uint32_t* duration, uint8_t* task_id,
                                                const uint32_t now )
{
    uint8_t  hook_id  = 0;
    uint8_t  index    = 0;
    uint32_t time_tmp = now;

    for( hook_id = 0; hook_id < RP_NB_HOOKS; hook_id++ )
    {  // Garbage collector
        if( ( rp->tasks[hook_id].state == RP_TASK_STATE_SCHEDULE ) &&
            ( ( ( int32_t )( rp->tasks[hook_id].start_time_ms - time_tmp ) < 0 ) ) )
        {
            rp->tasks[hook_id].state = RP_TASK_STATE_ABORTED;
        }
    }
    for( hook_id = 0; hook_id < RP_NB_HOOKS; hook_id++ )
    {
        if( ( rp->tasks[hook_id].state < RP_TASK_STATE_RUNNING ) &&
            ( ( ( int32_t )( rp->tasks[hook_id].start_time_ms - time_tmp ) >= 0 ) ) )
        {
            time_tmp = rp->tasks[hook_id].start_time_ms;
            index    = hook_id;
            break;
        }
    }
    if( hook_id == RP_NB_HOOKS )
    {
        return RP_STATUS_NO_MORE_TASK_SCHEDULE;
    }

    for( uint8_t i = hook_id; i < RP_NB_HOOKS; i++ )
    {
        if( ( rp->tasks[i].state < RP_TASK_STATE_RUNNING ) &&
            ( ( int32_t )( rp->tasks[i].start_time_ms - time_tmp ) < 0 ) &&
            ( ( int32_t )( rp->tasks[i].start_time_ms - now ) >= 0 ) )
        {
            time_tmp = rp->tasks[i].start_time_ms;
            index    = i;
        }
    }
    *task_id  = index;
    *duration = time_tmp - now;
    return RP_STATUS_HAVE_TO_SET_TIMER;
}

static uint8_t rp_task_find_highest_priority( uint8_t* vector, uint8_t length )
{
    uint8_t priority_high = 0xFF;
    uint8_t index         = 0;

    for( int32_t i = 0; i < length; i++ )
    {
        if( vector[i] <= priority_high )
        {
            priority_high = vector[i];
            index         = i;
        }
    }
    return index;
}

rp_hook_status_t rp_get_pkt_payload( radio_planner_t* rp, const rp_task_t* task )
{
    rp_hook_status_t status = RP_HOOK_STATUS_OK;
    uint8_t          id     = task->hook_id;

    if( ( task->type == RP_TASK_TYPE_USER ) || ( task->type == RP_TASK_TYPE_NONE ) )
    {
        return status;  // don't catch the payload in case of user task
    }
    smtc_modem_hal_assert( ral_get_pkt_payload( &( rp->radio->ral ), rp->payload_size[id], rp->payload[id],
                                                &rp->payload_size[id] ) == RAL_STATUS_OK );

    if( ( task->type == RP_TASK_TYPE_RX_LORA ) || ( task->type == RP_TASK_TYPE_CAD_TO_RX ) )
    {
        rp->radio_params[id].pkt_type = RAL_PKT_TYPE_LORA;
        status                        = RP_HOOK_STATUS_OK;

        smtc_modem_hal_assert( ral_get_lora_rx_pkt_status(
                                   &( rp->radio->ral ), &rp->radio_params[id].rx.lora_pkt_status ) == RAL_STATUS_OK );
    }
    else if( task->type == RP_TASK_TYPE_RX_FSK )
    {
        rp->radio_params[id].pkt_type = RAL_PKT_TYPE_GFSK;
        status                        = RP_HOOK_STATUS_OK;

        smtc_modem_hal_assert( ral_get_gfsk_rx_pkt_status(
                                   &( rp->radio->ral ), &rp->radio_params[id].rx.gfsk_pkt_status ) == RAL_STATUS_OK );
    }
    else
    {
        status = RP_HOOK_STATUS_ID_ERROR;
    }

    return status;
}

static void rp_set_alarm( radio_planner_t* rp, const uint32_t alarm_in_ms )
{
    rp_hal_timer_stop( );
    rp_hal_timer_start( rp, alarm_in_ms, rp_timer_irq_callback );
}

static void rp_timer_irq( radio_planner_t* rp )
{
    rp_task_arbiter( rp, __func__ );
}

static void rp_task_call_aborted( radio_planner_t* rp )
{
    for( int32_t i = 0; i < RP_NB_HOOKS; i++ )
    {
        if( rp->tasks[i].state == RP_TASK_STATE_ABORTED )
        {
            SMTC_MODEM_HAL_RP_TRACE_PRINTF( " RP: INFO - Aborted hook # %d callback\n", i );
            rp->stats.task_hook_aborted_nb[i]++;
            rp_task_free( rp, &rp->tasks[i] );
            rp->status[i] = RP_STATUS_TASK_ABORTED;
            rp_hook_callback( rp, i );
        }
    }
}

//
// Radio planner callbacks
//

void rp_radio_irq_callback( void* obj )
{
    rp_radio_irq( ( radio_planner_t* ) obj );
}

static void rp_timer_irq_callback( void* obj )
{
    rp_timer_irq( ( radio_planner_t* ) obj );
}

static void rp_hook_callback( radio_planner_t* rp, uint8_t id )
{
    if( id >= RP_NB_HOOKS )
    {
        smtc_modem_hal_mcu_panic( );
        return;
    }
    if( rp->hook_callbacks[id] == NULL )
    {
        smtc_modem_hal_mcu_panic( );
        return;
    }
    rp->hook_callbacks[id]( rp->hooks[id] );
}

//
// Private debug utilities implementation
//

static void rp_task_print( const radio_planner_t* rp, const rp_task_t* task )
{
    SMTC_MODEM_HAL_RP_TRACE_PRINTF( "\nRP- INFO - Radio task #%u  running - Timer task #%u running  - Hook ID #%u -",
                                    rp->radio_task_id, rp->timer_task_id, task->hook_id );
    switch( task->type )
    {
    case RP_TASK_TYPE_RX_LORA:
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( " TASK_RX_LORA " );
        break;
    case RP_TASK_TYPE_RX_FSK:
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( " TASK_RX_FSK " );
        break;
    case RP_TASK_TYPE_TX_LORA:
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( " TASK_TX_LORA " );
        break;
    case RP_TASK_TYPE_TX_FSK:
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( " TASK_TX_FSK " );
        break;
    case RP_TASK_TYPE_TX_LR_FHSS:
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( " TASK_TX_LR_FHSS " );
        break;
    case RP_TASK_TYPE_CAD:
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( " TASK_CAD " );
        break;
    case RP_TASK_TYPE_CAD_TO_TX:
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( " TASK_CAD_TO_TX " );
        break;
    case RP_TASK_TYPE_CAD_TO_RX:
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( " TASK_CAD_TO_RX " );
        break;
    case RP_TASK_TYPE_NONE:
    case RP_TASK_TYPE_GNSS_SNIFF:
    case RP_TASK_TYPE_WIFI_SNIFF:
    case RP_TASK_TYPE_GNSS_RSSI:
    case RP_TASK_TYPE_WIFI_RSSI:
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( " TASK_EMPTY " );
        break;
    default:
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( " TASK_ERROR " );
        break;
    };
    SMTC_MODEM_HAL_RP_TRACE_PRINTF( " - start time @%lu - priority #%u\n", task->start_time_ms, task->priority );
}

static void rp_consumption_statistics_updated( radio_planner_t* rp, const uint8_t hook_id, const uint32_t time )
{
    uint32_t micro_ampere_radio = 0, micro_ampere_process = 0;
    uint32_t radio_t = 0, process_t = 0;

    if( rp->tasks[hook_id].type == RP_TASK_TYPE_RX_LORA )
    {
        ral_get_lora_rx_consumption_in_ua( &( rp->radio->ral ), rp->radio_params[hook_id].rx.lora.mod_params.bw, false,
                                           &micro_ampere_radio );
    }
    else if( rp->tasks[hook_id].type == RP_TASK_TYPE_RX_FSK )
    {
        ral_get_gfsk_rx_consumption_in_ua( &( rp->radio->ral ), rp->radio_params[hook_id].rx.gfsk.mod_params.br_in_bps,
                                           rp->radio_params[hook_id].rx.gfsk.mod_params.bw_dsb_in_hz, false,
                                           &micro_ampere_radio );
    }
    else if( rp->tasks[hook_id].type == RP_TASK_TYPE_TX_LORA )
    {
        ral_get_tx_consumption_in_ua( &( rp->radio->ral ), rp->radio_params[hook_id].tx.lora.output_pwr_in_dbm,
                                      rp->radio_params[hook_id].tx.lora.rf_freq_in_hz, &micro_ampere_radio );
    }
    else if( rp->tasks[hook_id].type == RP_TASK_TYPE_TX_FSK )
    {
        ral_get_tx_consumption_in_ua( &( rp->radio->ral ), rp->radio_params[hook_id].tx.gfsk.output_pwr_in_dbm,
                                      rp->radio_params[hook_id].tx.gfsk.rf_freq_in_hz, &micro_ampere_radio );
    }
    // else if( rp->tasks[hook_id].type == RP_TASK_TYPE_TX_LR_FHSS )  // TODO uncomment when LR-FHSS consumption will be
    // developed
    // {
    //     ral_get_tx_consumption_in_ua( &( rp->radio->ral ), rp->radio_params[hook_id].tx.lr_fhss.output_pwr_in_dbm,
    //                                   rp->radio_params[hook_id].tx.lr_fhss.ral_lr_fhss_params.rf_freq_in_hz,
    //                                   &micro_ampere_radio );
    // }
#if defined( LR1110_MODEM_E ) && defined( _MODEM_E_GNSS_ENABLE )
    else if( ( rp->tasks[hook_id].type == RP_TASK_TYPE_GNSS_SNIFF ) ||
             ( rp->tasks[hook_id].type == RP_TASK_TYPE_GNSS_RSSI ) )
    {
        rp_hal_get_gnss_conso_us( &radio_t, &process_t );
        micro_ampere_radio   = 10000;
        micro_ampere_process = 5000;
    }
#endif  // LR1110_MODEM_E && _MODEM_E_GNSS_ENABLE

#if defined( LR1110_MODEM_E ) && defined( _MODEM_E_WIFI_ENABLE )
    else if( ( rp->tasks[hook_id].type == RP_TASK_TYPE_WIFI_SNIFF ) ||
             ( rp->tasks[hook_id].type == RP_TASK_TYPE_WIFI_RSSI ) )
    {
        rp_hal_get_wifi_conso_us( &radio_t, &process_t );
        micro_ampere_radio   = 11000;
        micro_ampere_process = 3000;
    }
#endif  // LR1110_MODEM_E && _MODEM_E_WIFI_ENABLE

    if( ( rp->tasks[hook_id].type == RP_TASK_TYPE_GNSS_SNIFF ) ||
        ( rp->tasks[hook_id].type == RP_TASK_TYPE_GNSS_RSSI ) ||
        ( rp->tasks[hook_id].type == RP_TASK_TYPE_WIFI_SNIFF ) ||
        ( rp->tasks[hook_id].type == RP_TASK_TYPE_WIFI_RSSI ) )
    {
        rp_stats_sniff_update( &rp->stats, time, radio_t, process_t, hook_id, micro_ampere_radio,
                               micro_ampere_process );
    }
    else
    {
        rp_stats_update( &rp->stats, time, hook_id, micro_ampere_radio );
    }
}
