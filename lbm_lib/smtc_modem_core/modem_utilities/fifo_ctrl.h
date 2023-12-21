/*!
 * \file      fifo_ctrl.h
 *
 * \brief     FIFO manager
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

#ifndef __FIFO_CTRL_H__
#define __FIFO_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>  // C99 types

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

// Return status fo get/set function
typedef enum fifo_return_status_e
{
    FIFO_STATUS_OK,                // Return is OK
    FIFO_STATUS_PARAM_ERROR,       // Only for get function
    FIFO_STATUS_BUFFER_EMPTY,      // Only for get function
    FIFO_STATUS_BUFFER_TOO_SMALL,  // For get: not enough space in buffer to read data from fifo
                                   // For set: fifo is not big enough to save data + metadata
} fifo_return_status_t;

// Internal structure to manage fifo - don't modify it
typedef struct fifo_ctrl_s
{
    uint8_t* buffer;
    uint16_t buffer_size;
    uint16_t read_offset;
    uint16_t write_offset;
    uint16_t free_space;
    uint16_t nb_element;

    // Stat
    uint32_t write_cnt;
    uint32_t read_cnt;
    uint32_t drop_cnt;
} fifo_ctrl_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Init the fifo
 *
 * @param ctrl          Fifo manager
 * @param buffer        Buffer to link to the fifo manager
 * @param buffer_size   Buffer size
 * @param metadata_size Size of metadata that will be provide with each message
 */
void fifo_ctrl_init( fifo_ctrl_t* ctrl, uint8_t* buffer, const uint16_t buffer_size );

/**
 * @brief Reset fifo manager (all datas & metadatas will be lost)
 *
 * @param ctrl Fifo to reset
 */
void fifo_ctrl_clear( fifo_ctrl_t* ctrl );

/**
 * @brief Display stat of the fifo (free space, nb element, drop counter, ....)
 *
 * @param ctrl
 */
void fifo_ctrl_print_stat( const fifo_ctrl_t* ctrl );

/**
 * @brief Return number of message stored in the fifo
 *
 * @param ctrl          fifo manager
 * @return uint16_t     number of messages in the fifo
 */
uint16_t fifo_ctrl_get_nb_elt( const fifo_ctrl_t* ctrl );

/**
 * @brief Return free space of the fifo
 *      Free space is use to store Size, metadata and data, not only data
 * @param ctrl  fifo manager
 * @return uint16_t bytes available
 */
uint16_t fifo_ctrl_get_free_space( const fifo_ctrl_t* ctrl );

/**
 * @brief Read oldest element in fifo
 *
 * @param ctrl                  fifo manager
 * @param buffer                buffer to save data
 * @param data_len              length of read data
 * @param data_buffer_size      size of buffer
 * @param metadata              pointer to save metadata
 * @param metadata_len          length of metadata
 * @param metadata_buffer_size  size of metadata buffer
 * @return fifo_return_status_t return status
 */
fifo_return_status_t fifo_ctrl_get( fifo_ctrl_t* ctrl, uint8_t* buffer, uint16_t* data_len,
                                    const uint16_t data_buffer_size, void* metadata, uint8_t* metadata_len,
                                    const uint8_t metadata_buffer_size );

/**
 * @brief Save a new element in the fifo
 *      If there is not enough free space, the oldest element will be removed
 *
 * @param ctrl          fifo manager
 * @param buffer        buffer to save
 * @param buffer_len    size of buffer
 * @param metadata      metadata to save
 * @param metadata_len  length of metadata
 * @return fifo_return_status_t return status
 */
fifo_return_status_t fifo_ctrl_set( fifo_ctrl_t* ctrl, const uint8_t* buffer, const uint16_t buffer_len,
                                    const void* metadata, const uint8_t metadata_len );

#ifdef __cplusplus
}
#endif

#endif  // __FIFO_CTRL_H__

/* --- EOF ------------------------------------------------------------------ */
