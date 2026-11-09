/*!
 * \file      file_upload.h
 *
 * \brief     File upload API
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

#ifndef __FILE_UPLOAD_H__
#define __FILE_UPLOAD_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "lr1_stack_mac_layer.h"
#include "smtc_modem_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef enum file_upload_return_code_e
{
    FILE_UPLOAD_OK,
    FILE_UPLOAD_ERROR_NOT_INIT,
    FILE_UPLOAD_ERROR_BUSY,
    FILE_UPLOAD_ERROR_SIZE,
    FILE_UPLOAD_ERROR_DL,
} file_upload_return_code_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

static const smtc_modem_return_code_t lfu_rc_lut[] = {
    [FILE_UPLOAD_OK]             = SMTC_MODEM_RC_OK,
    [FILE_UPLOAD_ERROR_NOT_INIT] = SMTC_MODEM_RC_NOT_INIT,
    [FILE_UPLOAD_ERROR_BUSY]     = SMTC_MODEM_RC_BUSY,
    [FILE_UPLOAD_ERROR_SIZE]     = SMTC_MODEM_RC_INVALID,
};

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- SERVICE MANAGEMENT ------------------------------------------------------
 */

void lfu_services_init( uint8_t* service_id, uint8_t task_id,
                        uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                        void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                        void** context_callback );

/*
 * -----------------------------------------------------------------------------
 * --- LFU FUNCTIONS -----------------------------------------------------------
 */

/**
 * @brief Create the file upload session
 *
 * @param [in] stack_id       Stack Identifier
 * @param [in] file           File
 * @param [in] file_len       size of file
 * @param [in] average_delay  average delay between each uplink frame
 * @param [in] port           applicative where the data will be forwarded
 * @param [in] encryption     Encryption with appskey option
 * @return file_upload_return_code_t
 */
file_upload_return_code_t file_upload_init( uint8_t stack_id, const uint8_t* file, uint32_t file_len,
                                            uint16_t average_delay, uint8_t port, bool encryption );

/**
 * @brief Start uploading
 *
 * @param [in] stack_id Stack Identifier
 * @return file_upload_return_code_t
 */
file_upload_return_code_t file_upload_start( uint8_t stack_id );

/**
 * @brief Stop and reset current upload session
 *
 * @param [in] stack_id Stack Identifier
 * @return file_upload_return_code_t
 */
file_upload_return_code_t file_upload_reset( uint8_t stack_id );

/**
 * @brief Get upload status
 *
 * @param [in] stack_id  Stack Identifier
 * @return true/false
 */
bool file_upload_get_status( uint8_t stack_id );

/**
 * @brief Stop properly lfu service
 *
 * @param [in] stack_id Stack Identifier
 */
void file_upload_stop_service( uint8_t stack_id );

#ifdef __cplusplus
}
#endif

#endif  // __FILE_UPLOAD_H__

/* --- EOF ------------------------------------------------------------------ */
