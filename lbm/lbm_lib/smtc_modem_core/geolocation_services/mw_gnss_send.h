/**
 * @file      mw_gnss_send.h
 *
 * @brief     GNSS middleware
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

#ifndef MW_GNSS_SEND_H
#define MW_GNSS_SEND_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_modem_geolocation_api.h"

#include "lr1_stack_mac_layer.h"

#include "mw_gnss_defs.h"

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
 * @brief Initialize GNSS send service
 *
 * @param service_id
 * @param task_id
 * @param downlink_callback
 * @param on_launch_callback
 * @param on_update_callback
 */
void mw_gnss_send_services_init( uint8_t* service_id, uint8_t task_id,
                                 uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                 void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                                 void** context_callback );

/**
 * @brief Add a request in the modem supervisor for sending the provided GNSS scan results over the air.
 *
 * @param [in] nav_group        A pointer to the NAV group to be sent over the air.
 */
void mw_gnss_send_add_task( const navgroup_t* nav_group );

/**
 * @brief Indicates if a send sequence has started.
 */
bool mw_gnss_send_is_busy( );

/**
 * @brief Retrieve the data associated with the SMTC_MODEM_EVENT_GNSS_TERMINATED event
 *
 * @param [out] data Pointer to the data associated with the event
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK         Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID    Given pointer to hold results data is NULL
 * @retval SMTC_MODEM_RC_FAIL       No SMTC_MODEM_EVENT_GNSS_TERMINATED event pending
 */
smtc_modem_return_code_t mw_gnss_get_event_data_terminated( smtc_modem_gnss_event_data_terminated_t* data );

/**
 * @brief Set the LoRaWAN port on which to send the GNSS scan results uplinks
 *
 * @param [in] port LoRaWAN port
 */
void mw_gnss_set_port( uint8_t port );

/**
 * @brief Select the send mode to be used to send NAV group results. By default the scan groups are sent by direct
 * LoRaWAN uplinks but it can be replace by the store and forward service, or be bypassed (no send).
 *
 * @param [in] send_mode \ref smtc_modem_geolocation_send_mode_t
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK         Send mode has been correctly selected
 * @retval SMTC_MODEM_RC_INVALID    Store & Forward service is not available (not compiled)
 * compiled
 */
smtc_modem_return_code_t mw_gnss_set_send_mode( smtc_modem_geolocation_send_mode_t send_mode );

#ifdef __cplusplus
}
#endif

#endif  // MW_GNSS_SEND_H

/* --- EOF ------------------------------------------------------------------ */
