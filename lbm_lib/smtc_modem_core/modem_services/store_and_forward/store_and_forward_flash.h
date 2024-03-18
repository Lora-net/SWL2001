/**
 * @file      store_and_forward_flash.h
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

#ifndef STORE_AND_FORWARD_FLASH_H
#define STORE_AND_FORWARD_FLASH_H

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
 * @enum store_and_forward_flash_rc_t
 */
typedef enum store_and_forward_flash_rc_e
{
    STORE_AND_FORWARD_FLASH_RC_OK,       //!< Function executed without error
    STORE_AND_FORWARD_FLASH_RC_INVALID,  //!< Invalid parameters
    STORE_AND_FORWARD_FLASH_RC_FAIL,     //!< Fail to execute the function
} store_and_forward_flash_rc_t;

/**
 * @brief Definition of Store and forward enablement
 *
 */
typedef enum store_and_forward_flash_state_e
{
    STORE_AND_FORWARD_DISABLE = 0,  // Service disabled
    STORE_AND_FORWARD_ENABLE  = 1,  // Service enabled
    STORE_AND_FORWARD_SUSPEND = 2,  // Service enabled, but send data is suspend
} store_and_forward_flash_state_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

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
void store_and_forward_flash_services_init( uint8_t* service_id, uint8_t task_id,
                                            uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                            void ( **on_launch_callback )( void* ),
                                            void ( **on_update_callback )( void* ), void** context_callback );

/**
 * @brief Set the state of the store and forward service
 *
 * @param [in] stack_id   Stack identifier
 * @param [in] enabled    @ref store_and_forward_flash_state_t
 * @return store_and_forward_flash_rc_t
 */
store_and_forward_flash_rc_t store_and_forward_flash_set_state( uint8_t                         stack_id,
                                                                store_and_forward_flash_state_t enabled );

/**
 * @brief Get the state of the store and forward service
 *
 * @param [in] stack_id Stack identifier
 * @return store_and_forward_flash_state_t
 */
store_and_forward_flash_state_t store_and_forward_flash_get_state( uint8_t stack_id );

/**
 * @brief Add data to the NVM FiFo
 *
 * @param [in] stack_id       Stack identifier
 * @param [in] fport          LoRaWAN FPort on which the uplink will be done
 * @param [in] confirmed      Message type (true: confirmed, false: unconfirmed)
 * @param [in] payload        Data to be stored and forwarded
 * @param [in] payload_length Number of bytes from payload to be stored and forwarded
 * @return store_and_forward_flash_rc_t
 */
store_and_forward_flash_rc_t store_and_forward_flash_add_data( uint8_t stack_id, uint8_t fport, bool confirmed,
                                                               const uint8_t* payload, uint8_t payload_length );

/**
 * @brief Clear all data stored in NVM
 *
 * @param stack_id      Stack identifier
 */
void store_and_forward_flash_clear_data( uint8_t stack_id );

/**
 * @brief Get the fifo capacity and the number of free slots before data loss by overwriting the slot already in use
 *
 * @param [in]  stack_id  Stack identifier
 * @param [out] capacity  Capacity of the fifo (number of slot)
 * @param [out] free_slot Number of free slot
 * @return store_and_forward_flash_rc_t
 */
store_and_forward_flash_rc_t store_and_forward_flash_get_number_of_free_slot( uint8_t stack_id, uint32_t* capacity,
                                                                              uint32_t* free_slot );

/**
 * @brief Get the File system object
 *
 * @param stack_id      Stack identifier
 * @return struct circularfs* the pointer to the object managing the FiFo in NVM
 */
struct circularfs* store_and_forward_flash_get_fs_object( uint8_t stack_id );

#ifdef __cplusplus
}
#endif

#endif  // STORE_AND_FORWARD_FLASH_H

/* --- EOF ------------------------------------------------------------------ */
