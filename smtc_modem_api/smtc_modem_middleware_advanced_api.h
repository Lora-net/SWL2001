/**
 * @file      smtc_modem_middleware_advanced_api.h
 *
 * @brief     Modem middleware/advanced API description
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

#ifndef SMTC_MODEM_MIDDLEWARE_ADVANCED_API_H__
#define SMTC_MODEM_MIDDLEWARE_ADVANCED_API_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "smtc_modem_api.h"

#include <stdint.h>   // standard types
#include <stdbool.h>  // bool type

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Size of the device to device ping slots mask
 */
#define SMTC_MODEM_D2D_PING_SLOTS_MASK_SIZE 16

/**
 * @defgroup Advanced Event codes definitions
 * @{
 */

#define SMTC_MODEM_EVENT_D2D_CLASS_B_TX_DONE 0x15  //!< Device to device uplink process finished
#define SMTC_MODEM_EVENT_MIDDLEWARE_1 0x16         //!< Reserved for Middleware
#define SMTC_MODEM_EVENT_MIDDLEWARE_2 0x17         //!< Reserved for Middleware
#define SMTC_MODEM_EVENT_MIDDLEWARE_3 0x18         //!< Reserved for Middleware
/**
 * @}
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Radio Planner task types
 */
typedef enum smtc_modem_rp_task_types_e
{
    SMTC_MODEM_RP_TASK_STATE_SCHEDULE,
    SMTC_MODEM_RP_TASK_STATE_ASAP,
} smtc_modem_rp_task_types_t;

/**
 * @brief Radio Planner task id
 */
typedef enum smtc_modem_rp_task_id_e
{
    SMTC_MODEM_RP_TASK_ID0,
    SMTC_MODEM_RP_TASK_ID1,
    SMTC_MODEM_RP_TASK_ID2,
} smtc_modem_rp_task_id_t;

/**
 * @brief Radio planner radio status
 */
typedef enum smtc_modem_rp_radio_status_e
{
    SMTC_RP_RADIO_RX_ERROR       = 0,
    SMTC_RP_RADIO_CAD_OK         = 1,
    SMTC_RP_RADIO_CAD_DONE       = 2,
    SMTC_RP_RADIO_TX_DONE        = 3,
    SMTC_RP_RADIO_RX_DONE        = 4,
    SMTC_RP_RADIO_RX_TIMEOUT     = 5,
    SMTC_RP_RADIO_WIFI_SCAN_DONE = 6,
    SMTC_RP_RADIO_GNSS_SCAN_DONE = 7,
    SMTC_RP_RADIO_ABORTED        = 8,
    SMTC_RP_RADIO_UNKNOWN        = 9,
} smtc_modem_rp_radio_status_t;

/**
 * @brief Structure containing status of rp user radio access operation
 */
typedef struct smtc_modem_rp_status_e
{
    uint8_t                      id;            //!< Radio Planner task id
    smtc_modem_rp_radio_status_t status;        //!< Radio Planner status
    uint16_t                     raw_irq;       //!< Radio raw irq
    uint32_t                     timestamp_ms;  //!< Radio irq timestamp
} smtc_modem_rp_status_t;

/**
 * @brief Radio Planner task definition
 */
typedef struct smtc_modem_rp_task_s
{
    smtc_modem_rp_task_types_t type;              //!< Radio Planner type of task
    uint32_t                   start_time_ms;     //!< The chosen task start time in ms
    uint32_t                   duration_time_ms;  //!< The task duration time in ms
    smtc_modem_rp_task_id_t    id;                //!< The id of the operation
    void ( *launch_task_callback )( void* );      //!< The function that will be called when the task is granted
    void ( *end_task_callback )( smtc_modem_rp_status_t* status );  //!< The status of the operation
} smtc_modem_rp_task_t;

/**
 * @brief Device to device priority definition
 */
typedef enum smtc_modem_d2d_priority_e
{
    SMTC_MODEM_D2D_CLASS_PRIORITY_LOW = 0,
    SMTC_MODEM_D2D_CLASS_PRIORITY_MEDIUM,
    SMTC_MODEM_D2D_CLASS_PRIORITY_HIGH,
} smtc_modem_d2d_priority_t;

/**
 * @brief Device to device uplink configuration structure
 */
typedef struct smtc_modem_d2d_class_b_uplink_config_e
{
    smtc_modem_d2d_priority_t priority;                            //!< The priority of the D2D uplink
    uint8_t                   nb_rep;                              //!< The number of repetitions for the D2D uplink
    uint16_t                  nb_ping_slot_tries;                  //!< The number of ping slot tries before stop
    uint8_t ping_slots_mask[SMTC_MODEM_D2D_PING_SLOTS_MASK_SIZE];  //!< The mask defining autorised ping slots for this
                                                                   //!< uplink (shall be in line with current ping slot
                                                                   //!< periodicity)
} smtc_modem_d2d_class_b_uplink_config_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Add a user task in radio planner
 *
 * @param [in] rp_task  Structure holding radio planner task information
 *
 * @return smtc_modem_return_code_t as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL               A user task is already running in radio planner
 */
smtc_modem_return_code_t smtc_modem_rp_add_user_radio_access_task( smtc_modem_rp_task_t* rp_task );

/**
 * @brief Abort a user task in radio planner
 *
 * @param [in] user_task_id  ID of the user task to abort
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_FAIL               Wrong user_task_id
 */
smtc_modem_return_code_t smtc_modem_rp_abort_user_radio_access_task( uint8_t user_task_id );

/**
 * @brief Request a LoRaWAN extended uplink
 *
 * @remark This feature is introduced for future middleware layer, it isn't recommended for the user to call this
 * function. This feature require a special compilation option to be activated.
 *
 * @param [in] stack_id                  Stack identifier
 * @param [in] fport                     LoRaWAN FPort on which the uplink is done
 * @param [in] confirmed                 Message type (true: confirmed, false: unconfirmed)
 * @param [in] payload                   Data to be sent
 * @param [in] payload_length            Number of bytes from payload to be sent
 * @param [in] extended_uplink_id        ID of the queue for extended uplink should be equal to 1 or 2
 * @param [in] lbm_notification_callback Notification callback (for lbm to notify middleware layer when tx is finished)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter fport is out of the [1:223] range or equal to dm_fport, or
 *                                         extended_uplink_id not equal to 1 or 2
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_request_extended_uplink( uint8_t stack_id, uint8_t f_port, bool confirmed,
                                                             const uint8_t* payload, uint8_t payload_length,
                                                             uint8_t extended_uplink_id,
                                                             void ( *lbm_notification_callback )( void ) );

/**
 * @brief Abort a LoRaWAN extended uplink
 *
 * @remark This feature is introduced for future middleware layer, it isn't recommended for the user to call this
 * function. This feature require a special compilation option to be activated.
 *
 * @param [in] stack_id           Stack identifier
 * @param [in] extended_uplink_id ID of the queue for extended uplink should be equal to 1 or 2
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           extended_uplink_id not equal to 1 or 2
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_abort_extended_uplink( uint8_t stack_id, uint8_t extended_uplink_id );

/**
 * @brief Request a device to device uplink
 *
 * @remark The uplink will be sent as soon as possible in the first available ping slot according to chosen
 * ping_slots_mask. It will be repeated nb_rep times in following acceptable slots.
 *
 * @param [in] stack_id        The stack identifier
 * @param [in] mc_grp_id       The multicast group identifier
 * @param [in] d2d_config      The device to device specific uplink configuration structure
 * @param [in] fport           The LoRaWAN FPort on which the uplink is done
 * @param [in] payload         The data to be sent
 * @param [in] payload_length  The number of bytes from payload to be sent
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID            \p fport is out of the [1:223] range
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL               Modem is not available (suspended, muted or not joined) or no multicast
 *                                          session is running on this group id
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID   Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_d2d_class_b_request_uplink( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                                smtc_modem_d2d_class_b_uplink_config_t* d2d_config,
                                                                uint8_t fport, const uint8_t* payload,
                                                                uint8_t payload_length );

/**
 * @brief Get the maximum payload size that can be used for a device to device uplink on chosen multicast group
 *
 * @param [in] stack_id             The stack identifier
 * @param [in]  mc_grp_id           The multicast group identifier
 * @param [out] tx_max_payload_size The maximum payload size in byte
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p tx_max_payload_size is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted or not joined) or no multicast
 *                                         session is running on this group id
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_d2d_class_b_get_tx_max_payload( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                                    uint8_t* tx_max_payload_size );

/*!
 * \brief increment a middleware asynchronous event
 *
 * \param [in] event_type type of asynchronous message
 * \param [in] status     status of asynchronous message
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p event_type isn't a middleware event type
 */
smtc_modem_return_code_t smtc_modem_increment_event_middleware( uint8_t event_type, uint8_t status );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_MODEM_MIDDLEWARE_ADVANCED_API_H__

/* --- EOF ------------------------------------------------------------------ */
