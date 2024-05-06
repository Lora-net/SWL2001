/*!
 * \file      modem_supervisor_light.h
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

#ifndef __MODEM_SUPERVISOR_LIGHT__H
#define __MODEM_SUPERVISOR_LIGHT__H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "radio_planner.h"
#include "lr1_stack_mac_layer.h"
#include "modem_services_config.h"
#include "modem_tx_protocol_manager.h"
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * \typedef task_id_t
 * \brief   Descriptor of all the tasks mange by the supervisor
 */
typedef enum
{

    IDLE_TASK,         //!< mean no more active task schedule
    SEND_TASK,         //!< task managed by the application such as sensor uplink for example
    JOIN_TASK,         //!< task managed by the modem itself to join a network
    RETRIEVE_DL_TASK,  //!< task managed by the modem to create downlink opportunities
    CID_REQ_TASK,  //!< task managed by the modem to launch a Network Link Check / Dev Time/ Ping Slot Info Request(s)
    CLASS_B_MANAGEMENT_TASK,  //!< task managed by class_b_management services to launch the class b (time sync, ping
                              //!< slot info request ...)
    SERVICE_ID0_TASK,         //!< task(s) reserved for services
    NUMBER_OF_TASKS = SERVICE_ID0_TASK + NUMBER_OF_SERVICES,  //!< number of tasks
} task_id_t;

/*!
 * \typedef task_priority_t
 * \brief   Descriptor of priorities for task
 */
typedef enum
{
    TASK_BYPASS_DUTY_CYCLE,     //!< even in case of duty cycle limitation the task with this priority are still active,
                                //!< used by gnss scan
    TASK_VERY_HIGH_PRIORITY,    //!< Very high priority
    TASK_HIGH_PRIORITY,         //!< High priority, RESERVED for Emergency Tx only
    TASK_MEDIUM_HIGH_PRIORITY,  //!< Medium priority
    TASK_LOW_PRIORITY,          //!< Low priority
    TASK_FINISH,                //!< task finished
} task_priority_t;

/**
 * @brief supervisor Return Codes
 *
 */
typedef enum supervisor_return_code_e
{
    SUPERVISOR_RC_OK     = 0x00,  //!< command executed without errors
    SUPERVISOR_RC_NOT_OK = 0x01,  //!< command executed with errors
} supervisor_return_code_t;
/*!
 * \typedef task_valid_t
 * \brief   task valid or note
 */
typedef enum task_valid_e
{
    TASK_VALID,     //!< Task valid
    TASK_NOT_VALID  //!< Task not valid
} task_valid_t;

/*!
 * \typedef smodem_task
 * \brief   Supervisor task description
 */
typedef struct smodem_task
{
    task_id_t       id;                 //!< Type ID of the task
    uint8_t         stack_id;           //!< Stack Id in case of multistack
    uint32_t        time_to_execute_s;  //!< The date to execute the task in second
    task_priority_t priority;           //!< The priority
    uint8_t         task_context;       //!< Parameters to store a small context of the task
    bool            task_enabled;       //!< Parameters to store context running or not of the task
    bool            updated_locked;
    uint32_t        launched_timestamp;
} smodem_task;

/*!
 * \typedef stask_manager
 * \brief   Supervisor task manager
 */
typedef struct stask_manager
{
    smodem_task     modem_task[NUMBER_OF_TASKS * NUMBER_OF_STACKS];
    task_id_t       next_task_id;
    bool            modem_is_suspended[NUMBER_OF_STACKS];
    task_priority_t modem_mute_with_priority[NUMBER_OF_STACKS];
} stask_manager;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief   Supervisor Initialization
 * \remark
 * \retval None
 */
void modem_supervisor_init( void );

/*!
 * \brief Supervisor Engine
 * \retval return the maximum delay in ms at which time the engine MUST be recalled
 */
uint32_t modem_supervisor_engine( void );

/**
 * \brief Abort all task in range id_min id_max
 *
 * \remark The range is to abort only task for a specific stack
 *
 * \param id_min
 * \param id_max
 */
void modem_supervisor_abort_tasks_in_range( uint16_t id_min, uint16_t id_max );

/*!
 * \brief   Remove a task in supervisor
 * \param [in]  id   - Task id
 * \retval task_valid_t
 */
task_valid_t modem_supervisor_remove_task( uint16_t id );

/*!
 * \brief   Add a task in supervisor
 * \remark
 * \param task*  smodem_task
 * \retval task_valid_t
 */
task_valid_t modem_supervisor_add_task( smodem_task* task );
/*!
 * \brief   Add a launch/update callback in supervisor for a given task_id
 * \remark
 * \param
 * \retval void
 */
void modem_supervisor_init_callback( const task_id_t task_id, void ( *callback_on_launch )( void* ),
                                     void ( *callback_on_update )( void* ), void* context_callback );

/*!
 * \brief  get task_manager
 * \remark
 * \param
 * \retval void
 */
stask_manager* modem_supervisor_get_task( void );

/**
 * @brief Get the suspend modem status of a stack_id
 *
 * @param stack_id
 * @return true the stack is suspended
 * @return false the stack is not suspended
 */
bool modem_supervisor_get_modem_is_suspended( uint8_t stack_id );

/**
 * @brief Set the suspend modem status of a stack_id
 *
 * @param suspend
 * @param stack_id
 */
void modem_supervisor_set_modem_is_suspended( bool suspend, uint8_t stack_id );

/**
 * @brief Get the allowed priority mode for a stack
 * @remark all task with a priority under the set value will not be executed
 *
 * @param stack_id
 * @return task_priority_t
 */
task_priority_t modem_supervisor_get_modem_mute_with_priority_parameter( uint8_t stack_id );

/**
 * @brief Set the allowed priority mode for a stack
 * @remark all task with a priority under the set value will not be executed
 *
 * @param priority_level
 * @param stack_id
 */
void modem_supervisor_set_modem_mute_with_priority_parameter( task_priority_t priority_level, uint8_t stack_id );

#ifdef __cplusplus
}
#endif

#endif  //__MODEM_SUPERVISOR_LIGHT__H
