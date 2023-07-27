/*!
 * \file      modem_event_utilities.h
 *
 * \brief     manage asynchronous event to notify the host layer.
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

#ifndef __SOFT_MODEM_EVENT_UTILITIES_H__
#define __SOFT_MODEM_EVENT_UTILITIES_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "smtc_modem_api.h"
#include "smtc_modem_hal.h"
#include "lr1mac_defs.h"

#define MODEM_NUMBER_OF_EVENTS SMTC_MODEM_EVENT_MAX  // number of possible events in modem

/*!
 * \brief init context of event
 *
 * \param [in]  ( *callback ) user callback when event occurs
 *
 * \return clear the context
 */
void modem_event_init( void ( *callback )( void ) );
/*!
 * \brief For each type of event have to manage a counter in case of overrun
 *
 * \param [in] event_type Type of event
 *
 * \return The number of asynchronous events for this type of event since the last get event cmd
 */
uint8_t get_modem_event_count( smtc_modem_event_type_t event_type );
/*!
 * \brief For each type of event have to manage a status
 *
 * \param [in] event_type  type of event
 *
 * \return The last status of asynchronous event for this type of event since the last get event cmd
 */
uint8_t get_modem_event_status( uint8_t event_type );

/*!
 * \brief set a value in the counter of a type of asynchronous event
 *
 * \param [in] event_type type of event
 * \param [in] value      value's type of event
 * \param [in] status     status of event
 */
void set_modem_event_count_and_status( uint8_t event_type, uint8_t value, uint8_t status );

/*!
 * \brief increment the counter of a type of asynchronous event
 *
 * \param [in] event_type type of asynchronous message
 * \param [in] status     status of asynchronous message
 */
void increment_modem_event_count_and_status( uint8_t event_type, uint8_t status );

/*!
 * \brief decrement the asynchronous message number
 * \retval void
 */
void decrement_asynchronous_msgnumber( void );

/*!
 * \brief increment the asynchronous message number
 *
 * \param [in] event_type type of asynchronous message
 * \param [in] status     status of asynchronous message
 */
void increment_asynchronous_msgnumber( uint8_t event_type, uint8_t status, uint8_t stack_id );

/*!
 * \brief get asynchronous message number
 *
 * \return The number of asynchronous message
 */
uint8_t get_asynchronous_msgnumber( void );

/**
 * @brief Get the last message event if asynchronous msgnumber is positive
 *
 * @param stack_id The Stack Identifier
 * @return uint8_t the last available modem event
 */
uint8_t get_last_msg_event( uint8_t* stack_id );

#ifdef __cplusplus
}
#endif

#endif  //