/*!
 * \file      modem_supervisor.h
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

#ifndef __MODEM_SUPERVISOR__H
#define __MODEM_SUPERVISOR__H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "radio_planner.h"
#include "lr1_stack_mac_layer.h"

#if defined( ADD_SMTC_ALC_SYNC )
#include "smtc_clock_sync.h"
#include "alc_sync.h"
#endif  // ADD_SMTC_ALC_SYNC

#if defined( ADD_SMTC_STREAM )
#include "stream.h"
#endif  // ADD_SMTC_STREAM

#if defined( ADD_SMTC_FILE_UPLOAD )
#include "file_upload.h"
#endif  // ADD_SMTC_FILE_UPLOAD

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
    SEND_TASK,    //!< task managed by the application such as sensor uplink for example
    JOIN_TASK,    //!< task managed by the modem itself to join a network
    DM_TASK,      //!< task managed by the modem itself to report periodically status
    DM_TASK_NOW,  //!< task managed by the modem when requested by the host or the cloud to report status
#if defined( ADD_SMTC_FILE_UPLOAD )
    FILE_UPLOAD_TASK,  //!< task initiate by the application layer but manage by the modem itself to transfer "big file"
#endif                 // ADD_SMTC_FILE_UPLOAD
    IDLE_TASK,         //!< mean no more active task schedule
    MUTE_TASK,         //!< task managed by the modem to un-mute the modem
    RETRIEVE_DL_TASK,  //!< task managed by the modem to create downlink opportunities
#if defined( ADD_SMTC_STREAM )
    STREAM_TASK,  //!< task initiated by the application layer, but managed by the modem itself to transfer long streams
#endif            // ADD_SMTC_STREAM
#if defined( ADD_SMTC_ALC_SYNC )
    CLOCK_SYNC_TIME_REQ_TASK,  //!< task managed by the modem to launch Application Layer Clock Synchronisation
    ALC_SYNC_ANS_TASK,         //!< task managed by the modem to launch Application Layer Clock Synchronisation answer
#endif                         // ADD_SMTC_ALC_SYNC
    FRAG_TASK,                 //!< task managed by the modem to launch Fragmented Data Block uplink
    USER_TASK,                //!< task manage by the modem to launch a user callback (use also for wifi and gnss tasks)
    DM_ALM_DBG_ANS,           //!< task managed by the modem to launch almanac debug answer
    CRASH_LOG_TASK,           //!< task managed by the modem to launch crash log
    LINK_CHECK_REQ_TASK,      //!< task managed by the modem to launch a Network Link Check Request
    DEVICE_TIME_REQ_TASK,     //!< task managed by the modem to launch a Network Device Time Request synchronisation
    PING_SLOT_INFO_REQ_TASK,  //!< task managed by the modem to launch a Network Ping Slot Info Request for class B
    SEND_TASK_EXTENDED_1,     //!< task managed by the application dedicated for middleware gnss/wifi
    SEND_TASK_EXTENDED_2,     //!< task managed by the application dedicated for middleware gnss/wifi
    NUMBER_OF_TASKS           //!< number of tasks
} task_id_t;

/*!
 * \typedef eTask_priority
 * \brief   Descriptor of priorities for task
 */
typedef enum
{
    TASK_VERY_HIGH_PRIORITY,    //!< Very high priority, RESERVED for Emergency Tx only
    TASK_HIGH_PRIORITY,         //!< High priority
    TASK_MEDIUM_HIGH_PRIORITY,  //!< Medium priority
    TASK_LOW_PRIORITY,          //!< Low priority
    TASK_FINISH,                //!< task finished
} eTask_priority;

/*!
 * \typedef eTask_valid_t
 * \brief   task valid or note
 */
typedef enum eTask_valid
{
    TASK_VALID,     //!< Task valid
    TASK_NOT_VALID  //!< Task not valid
} eTask_valid_t;

typedef enum e_tx_mode
{
    TX_UNCONFIRMED = 0x00,  //!< Tx packet in Unconfirmed mode
    TX_CONFIRMED   = 0x01   //!< Tx packet in Confirmed mode
} e_tx_mode_t;

/*!
 * \typedef smodem_task
 * \brief   Supervisor task description
 */
typedef struct smodem_task
{
    task_id_t      id;                 //!< Type ID of the task
    uint32_t       time_to_execute_s;  //!< The date to execute the task in second
    eTask_priority priority;           //!< The priority
    uint8_t        fPort;              //!< LoRaWAN frame port
    bool           fPort_present;      //!< LoRaWAN frame port
    const uint8_t* dataIn;             //!< Data in task
    uint8_t        sizeIn;             //!< Data length in byte(s)
    uint8_t        PacketType;         //!< LoRaWAN packet type ( Tx confirmed/Unconfirmed )
} smodem_task;

/*!
 * \typedef stask_manager
 * \brief   Supervisor task manager
 */
typedef struct stask_manager
{
    smodem_task modem_task[NUMBER_OF_TASKS];
    task_id_t   current_task_id;
    task_id_t   next_task_id;
    uint32_t    sleep_duration;

} stask_manager;

typedef struct smtc_modem_services_s
{
#if defined( ADD_SMTC_ALC_SYNC )
    alc_sync_ctx_t   alc_sync_ctx;
    clock_sync_ctx_t clock_sync_ctx;
#endif  // ADD_SMTC_ALC_SYNC

#if defined( ADD_SMTC_STREAM )
    rose_t stream_ROSE_ctx;
#endif  // ADD_SMTC_STREAM

#if defined( ADD_SMTC_FILE_UPLOAD )
    file_upload_t file_upload_ctx;
#endif  // ADD_SMTC_FILE_UPLOAD
} smtc_modem_services_t;
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief   Supervisor Initialization
 * \remark
 * \param [in]  callback*   - pointer to the callback
 * \param [in]  rp*         - pointer to the radio planner
 * \retval None
 */
void modem_supervisor_init( void ( *callback )( void ), radio_planner_t* rp,
                            smtc_modem_services_t* smtc_modem_services_ctx );

/*!
 * \brief modem_supervisor_init_task
 * \retval
 */
void modem_supervisor_init_task( void );
/*!
 * \brief Supervisor Engine
 * \retval return the maximum delay in ms at which time the engine MUST be recalled
 */
uint32_t modem_supervisor_engine( void );

/*!
 * \brief   Init all task to Idle
 * \retval none
 */
void init_task( void );

eTask_priority modem_supervisor_get_task_priority( task_id_t id );

/*!
 * \brief   Remove a task in supervisor
 * \param [in]  id   - Task id
 * \retval eTask_valid_t
 */
eTask_valid_t modem_supervisor_remove_task( task_id_t id );

/*!
 * \brief   Add a task in supervisor
 * \remark
 * \param task*  smodem_task
 * \retval eTask_valid_t
 */
eTask_valid_t modem_supervisor_add_task( smodem_task* task );

/**
 * @brief
 *
 * @param data
 * @param data_length
 * @param metadata
 * @param ack_requested
 * @return uint8_t
 */

uint8_t modem_supervisor_update_downlink_frame( uint8_t* data, uint8_t data_length, lr1mac_down_metadata_t* metadata,
                                                bool ack_requested );
#ifdef __cplusplus
}
#endif

#endif  //__MODEM_SUPERVISOR__H
