/**
 * @file      store_and_forward.h
 *
 * @brief     Store and Forward API
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

#ifndef STORE_AND_FORWARD_H
#define STORE_AND_FORWARD_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
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

/**
 * @brief Definition of return codes for store and forward functions
 *
 * @enum store_and_forward_rc_t
 */
typedef enum store_and_forward_rc_e
{
    STORE_AND_FORWARD_ERROR,  //!< Error during function
    STORE_AND_FORWARD_OK,     //!< Function executed without error
} store_and_forward_rc_t;

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
void store_and_forward_services_init( uint8_t* service_id, uint8_t task_id,
                                      uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                      void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                                      void** context_callback );

/**
 * @brief Enable the Store and Forward service
 *
 * @param [in] stack_id                           - Stack identifier
 * @param [in] enabled                            - true: enable and start service, false: disable and stop service
 */
void store_and_forward_services_enable( uint8_t stack_id, bool enabled );

/**
 * @brief
 *
 * @param fport             Uplink port
 * @param data              User payload
 * @param data_len          User payload length
 * @param confirmed       User packet type : confirmed      Message type (true: confirmed, false: unconfirmed)
 * @param lifetime_s        lifetime in second of the data, after this time the data will be trashed if not acked by the
 * network
 * @param stack_id          Stack identifier
 * @return store_and_forward_rc_t
 */
store_and_forward_rc_t store_and_forward_add_data( uint8_t fport, const uint8_t* data, uint8_t data_len, bool confirmed,
                                                   uint32_t lifetime_s, uint8_t stack_id );

/**
 * @brief Clear all data stored in buffer
 *
 * @param stack_id
 */
void store_and_forward_clear_data( uint8_t stack_id );

#ifdef __cplusplus
}
#endif

#endif  // STORE_AND_FORWARD_H

/* --- EOF ------------------------------------------------------------------ */
