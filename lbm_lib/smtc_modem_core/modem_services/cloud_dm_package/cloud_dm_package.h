/**
 * @file      cloud_dm_package.h
 *
 * @brief     LoRaWAN template
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

#ifndef CLOUD_DM_PACKAGE_H
#define CLOUD_DM_PACKAGE_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include "device_management_defs.h"
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

////////////////////////////////////////////////////////
//////////// Init service object ///////////////////////
////////////////////////////////////////////////////////

/**
 * @brief Init a new LoRaWAN template services object
 *
 * @param service_id
 * @param task_id
 * @param downlink_callback
 * @param on_launch_callback
 * @param on_update_callback
 * @return bool
 */
void cloud_dm_services_init( uint8_t* service_id, uint8_t task_id,
                             uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                             void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                             void** context_callback );

/**
 * @brief Enable the cloud Device Management service
 *
 * @param [in] stack_id                           - Stack identifier
 * @param [in] enabled                            - true: enable and start service, false: disable and stop service
 */
void cloud_dm_services_enable( uint8_t stack_id, bool enabled );

/**
 * @brief Set modem DM port
 * @remark  This command sets the device management port.
 *
 * @param [in] stack_id                           - Stack identifier
 * @param [in] port                               - DM port
 * @return dm_rc_t
 */
dm_rc_t cloud_dm_set_dm_port( uint8_t stack_id, uint8_t port );

/**
 * @brief   Get DM port
 * @remark  This command gets the device management port.
 *
 * @param stack_id                                - Stack identifier
 * @return uint8_t
 */
uint8_t cloud_dm_get_dm_port( uint8_t stack_id );

/**
 * @brief   get the DM fields included in the periodic DM status messages
 *
 * @param [in] stack_id                          - Stack identifier
 * @param [out] dm_fields_payload                - Returned array that contains fields included in the periodic
 *                                                 DM status messages.
 * @param [out] dm_field_length                  - the len of the dm payload
 * @param [in] rate                              - cf: dm_info_rate_t
 * @return dm_rc_t
 */
dm_rc_t cloud_dm_get_info_field( uint8_t stack_id, uint8_t* dm_fields_payload, uint8_t* dm_field_length,
                                 dm_info_rate_t rate );

/*!
 * @brief   This command sets the default info fields to be included in the periodic DM status messages.
 *          The set is specified as list of field codes as defined in Uplink Message Format.
 *          An empty set is valid and will effectively disable the DM status message.
 *
 * @param   [in]  stack_id                    - Stack identifier
 * @param   [in]  requested_info_list         - Array of bytes with requested DM code in each bytes
 * @param   [in]  len                         - Number of byte that composed requested_info_list
 * @param   [in]  flag                        - dm_info_rate_t: If DM_INFO_NOW: set bitfield given by the user with
 *                                                                  GetInfo command,
 *                                                              Else: set bitfield for saved context with SetDmInfo
 * @retval dm_rc_t               - Return DM_ERROR in case of failure, else false DM_OK
 */
dm_rc_t cloud_dm_set_info_field( uint8_t stack_id, const uint8_t* requested_info_list, uint8_t len,
                                 dm_info_rate_t flag );

/*!
 * @brief set modem dm interval
 * @remark  This command sets the device management reporting interval.
 *          The periodic status reporting interval field is encoded in one
 *          byte where the two top-most bits specify the unit :
 *          (00 - sec, 01 - day, 10 - hour, 11 - min),
 *          and the lower six bits the value 0-63.
 *          A value of zero disables the periodic status reporting
 *
 * @param   [in]  stack_id                - Stack identifier
 * @param   [in]  interval                - Set DM interval
 * @retval dm_rc_t                        - Return dm_rc_t
 */
dm_rc_t cloud_dm_set_dm_interval( uint8_t stack_id, uint8_t interval );

/*!
 * @brief get modem dm interval
 * @remark  This command returns the device management reporting interval.
 *          The periodic status reporting interval field is encoded in one
 *          byte where the two top-most bits specify the unit :
 *          (00 - sec, 01 - day, 10 - hour, 11 - min),
 *          and the lower six bits the value 0-63.
 *          A value of zero disables the periodic status reporting
 *
 * @param   [in]  stack_id              - Stack identifier
 * @retval uint8_t                      - Return the DM interval
 */
uint8_t cloud_dm_get_dm_interval( uint8_t stack_id );

/*!
 * @brief   Set application-specific status in DM
 * @remark  This commands sets application-specific status information to be reported to the DM service.
 *
 * @param   [out]   app_status*         - App status payload
 * @retval void
 */
void cloud_dm_set_modem_user_app_status( uint8_t stack_id, const uint8_t* app_status );

/*!
 * @brief   Get application-specific status in DM
 * @remark  This commands gets application-specific status information
 *
 * @param   [in]    stack_id            - Stack identifier
 * @param   [out]   app_status*         - App status payload
 * @retval   void
 */
void cloud_dm_get_modem_user_app_status( uint8_t stack_id, uint8_t* app_status );

#ifdef __cplusplus
}
#endif

#endif  // CLOUD_DM_PACKAGE_H

/* --- EOF ------------------------------------------------------------------ */
