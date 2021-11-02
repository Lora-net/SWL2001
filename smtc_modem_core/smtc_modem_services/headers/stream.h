/*!
 * \file      stream.h
 *
 * \brief     streaming code definition
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
 * \brief   Available return codes used by the Stream API
 */
typedef enum stream_return_code
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
 * \brief   Encryption mode to use to encrypt the stream contents
 */
typedef enum stream_encrypt_mode
{
    STREAM_NOT_ENCRYPTED = 0,
    STREAM_ENCRYPTED     = 0x01,
} stream_encrypt_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief   Initialize a new streaming session
 *
 * \param [in] ROSE*                Pointer to Stream context
 * \retval stream_return_code_t     STREAM_OK or STREAM_FAIL
 */
stream_return_code_t stream_init( rose_t* ROSE );

/*!
 * \brief   Enable encryption for newly initialized streaming session
 * \remark This function MUST be called immediately after stream_init
 *
 * \param [in] ROSE*                Pointer to Stream context
 * \retval stream_return_code_t     STREAM_OK or STREAM_FAIL
 */
stream_return_code_t stream_enable_encryption( rose_t* ROSE );

/*!
 * \brief   Indicates if data is pending for uplink
 *
 * \param [in] ROSE*                Pointer to Stream context
 * \retval bool                     True if data is pending
 */
bool stream_data_pending( rose_t* ROSE );

/*!
 * \brief   Add new data to be sent by the streaming session
 *
 * \param [in] ROSE*                Pointer to Stream context
 * \param [in] data                 Pointer to a buffer containing the new data
 * \param [in] len                  Length of the buffer
 *
 * \retval stream_return_code_t     STREAM_OK if successful,
 *                                  STREAM_FAIL if incorrect pointers,
 *                                  STREAM_BADSIZE if the buffer is too small,
 *                                  STREAM_BUSY if the underlying ROSE buffer is full and
 *                                      can not contain the additional data,
 *                                  STREAM_OVERRUN if the underlying ROSE buffer has overrun
 */
stream_return_code_t stream_add_data( rose_t* ROSE, const uint8_t* data, uint8_t len );

/*!
 * \brief   Get a new stream fragment to uplink
 *
 * \param [in] ROSE*                Pointer to Stream context
 * \param [out] buf                 Pointer to a buffer where the fragment will be written
 * \param [in] frag_ctn             LoRa Frame Counter that will be used in the uplink message
 * \param [inout] len               As input: Max length of the buffer
 *                                  As output: Effective length filled by the fragment
 *
 * \retval stream_return_code_t     STREAM_OK if successful,
 *                                  STREAM_FAIL if incorrect pointers,
 *                                  STREAM_BADSIZE if the buffer is too small,
 *                                  STREAM_OVERRUN if the underlying ROSE buffer has overrun
 */
stream_return_code_t stream_get_fragment( rose_t* ROSE, uint8_t* buf, uint32_t frag_ctn, uint8_t* len );

/*!
 * \brief   Get current status of the stream contents
 *
 * \param [in] ROSE*                Pointer to Stream context
 * \param [out] pending             Pointer to store the amount of pending bytes to uplink
 * \param [out] free                Pointer to store the smount of free space in the underlying buffer
 *
 * \remark Any of the pointers can be NULL if the information is not needed
 *
 * \retval void
 */
void stream_status( rose_t* ROSE, uint16_t* pending, uint16_t* free );

/*!
 * \brief   Process a downlink stream command SCMD.
 *
 * \param [in] ROSE*                Pointer to Stream context
 * \param [in] payload              Pointer to a buffer containing the command
 * \param [in] len                  Length of the command
 *
 * \retval stream_return_code_t     STREAM_OK if successful,
 *                                  STREAM_UNKNOWN_SCMD if the command is not correct
 */
stream_return_code_t stream_process_dn_frame( rose_t* ROSE, const uint8_t* payload, uint8_t len );

/**
 * @brief get stream redundancy
 *
 * @param [in] ROSE*                Pointer to Stream context
 * \retval stream_rr stream redundancy
 */
uint8_t stream_get_rr( rose_t* ROSE );

/**
 * @brief set stream redundancy
 *
 * @param [in] ROSE*                Pointer to Stream context
 * @param [in] stream_rr           stream redundancy
 * \retval void
 */
void stream_set_rr( rose_t* ROSE, uint8_t stream_rr );

/**
 * @brief Reset stream context
 *
 * @param [in] ROSE Pointer to Stream context
 * @return stream_return_code_t
 */
stream_return_code_t stream_reset( rose_t* ROSE );

#endif  // __STREAM_H__

/* --- EOF ------------------------------------------------------------------ */
