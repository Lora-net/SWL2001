/*!
 * @file      stream.h
 *
 * @brief     streaming code definition
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

#ifndef __STREAM_H__
#define __STREAM_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include "rose.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */
#define STREAM_UPLINK_HEADER 0x14
#define STREAM_DOWNLINK_HEADER 0x08

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief   Available return codes used by the Stream API
 */
typedef enum stream_return_code_e
{
    STREAM_OK = 0,
    STREAM_BADSIZE,
    STREAM_TOOSMALL,
    STREAM_BUSY,
    STREAM_FAIL,
    STREAM_OVERRUN,
    STREAM_UNKNOWN_SCMD,
} stream_return_code_t;

/*!
 * @brief   Encryption mode to use to encrypt the stream contents
 */
typedef enum stream_encrypt_mode_e
{
    STREAM_NOT_ENCRYPTED = 0,
    STREAM_ENCRYPTED     = 0x01,
} stream_encrypt_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- SERVICE MANAGEMENT ------------------------------------------------------
 */
void    stream_services_init( uint8_t* service_id, uint8_t task_id,
                              uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                              void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                              void** context_callback );
void    stream_service_on_launch( void* service_id );
void    stream_service_on_update( void* service_id );
uint8_t stream_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/*
 * -----------------------------------------------------------------------------
 * --- STREAM ------------------------------------------------------------------
 */

/*!
 * @brief   Initialize a new streaming session
 *
 * @param [in] stack_id             Stack identifier
 * @retval stream_return_code_t     STREAM_OK or STREAM_FAIL
 */
stream_return_code_t stream_init( uint8_t stack_id, uint8_t f_port, bool encryption, uint8_t redundancy_ratio_percent );

/**
 * @brief Check if the encryption is enabled
 *
 * @param [in] stack_id             Stack identifier
 * @return true
 * @return false
 */
bool stream_encrypted_mode( uint8_t stack_id );

/**
 * @brief Get the initialization status of the stream
 *
 * @param [in] stack_id Stack identifier
 * @return true Stream has been initialized
 * @return false Otherwize
 */
bool stream_get_init_status( uint8_t stack_id );

/**
 * @brief Get the current active status of stream service
 *
 * @param [in] stack_id Stack identifier
 * @return true if there is a data in stream buffer or ongoing upload
 * @return false otherwize
 */
bool stream_get_status( uint8_t stack_id );

/*!
 * @brief   Add new data to be sent by the streaming session
 *
 * @param [in] stack_id             Stack identifier
 * @param [in] data                 Pointer to a buffer containing the new data
 * @param [in] len                  Length of the buffer
 *
 * @retval stream_return_code_t     STREAM_OK if successful,
 *                                  STREAM_FAIL if incorrect pointers,
 *                                  STREAM_BADSIZE if the buffer is too small,
 *                                  STREAM_BUSY if the underlying ROSE buffer is full and
 *                                      can not contain the additional data,
 *                                  STREAM_OVERRUN if the underlying ROSE buffer has overrun
 */
stream_return_code_t stream_add_data( uint8_t stack_id, const uint8_t* data, uint8_t len );

/*!
 * @brief   Get current status of the stream contents
 *
 * @param [in]  stack_id            Stack identifier
 * @param [out] pending             Pointer to store the amount of pending bytes to uplink
 * @param [out] free                Pointer to store the smount of free space in the underlying buffer
 *
 * @remark Any of the pointers can be NULL if the information is not needed
 *
 * @retval void
 */
void stream_status( uint8_t stack_id, uint16_t* pending, uint16_t* free );

/*!
 * @brief    get the stream port
 * @param   [in]  void
 * @retval  [out] port
 */
uint8_t stream_get_port( uint8_t stack_id );

/**
 * @brief get stream redundancy
 *
 * @param [in] stack_id            Stack identifier
 * @retval stream_rr stream redundancy
 */
uint8_t stream_get_rr( uint8_t stack_id );

/**
 * @brief set stream redundancy
 *
 * @param [in] stack_id            Stack identifier
 * @param [in] stream_rr           stream redundancy
 * @retval void
 */
void stream_set_rr( uint8_t stack_id, uint8_t stream_rr );

/**
 * @brief Stop properly stream service
 *
 * @param [in] stack_id Stack identifier
 */
void stream_service_stop( uint8_t stack_id );

#endif  // __STREAM_H__

/* --- EOF ------------------------------------------------------------------ */
