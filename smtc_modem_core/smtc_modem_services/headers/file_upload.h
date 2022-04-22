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

#include "file_upload_defs.h"
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef enum file_upload_return_code_e
{
    FILE_UPLOAD_OK,
    FILE_UPLOAD_ERROR
} file_upload_return_code_t;

/*!
 * \typedef file_upload_encrypt_mode_t
 * \brief   File Upload Encrypt Mode
 */
typedef enum file_upload_encrypt_mode
{
    FILE_UPLOAD_NOT_ENCRYPTED = 0x00,  //!< File Upload not encrypted
    FILE_UPLOAD_ENCRYPTED     = 0x01   //!< File Upload encrypted
} file_upload_encrypt_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

typedef struct file_upload_s
{
    uint8_t                    sid;              // Session Id (2bits)
    uint16_t                   average_delay;    // average frame transmission rate/delay
    uint8_t                    port;             // applicative port on which the upload is done
    file_upload_encrypt_mode_t encrypt_mode;     // file upload encryptio mode
    uint8_t                    session_counter;  // session counter
    uint32_t*                  file_buf;         // data buffer
    uint32_t                   file_len;         // file len
    uint32_t                   header[3];        // Current file upload header
    uint16_t                   cct;              // chunk count
    uint16_t                   cntx;             // chunk transmission count
    uint8_t                    fntx;             // frame transmission count

} file_upload_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Create the file upload session
 *
 * @param [in] file_upload     Pointer to File Upload context
 * @param [in] sid             session ID
 * @param [in] sz              size of file
 * @param [in] average_delay   average delay between each uplink frame
 * @param [in] port            applicative where the data will be forwarded
 * @param [in] encryption      Encryption type
 * @param [in] session_counter Upload session counter
 * @return file_upload_return_code_t
 */
file_upload_return_code_t file_upload_init( file_upload_t* file_upload, uint32_t session_id, uint32_t file_len,
                                            uint16_t average_delay, uint8_t port, uint8_t encryption,
                                            uint8_t session_counter );

/**
 * @brief Process the downlink frame FILEDONE
 *
 * @param [in] file_upload  Pointer to File Upload context
 * @param [in] payload      Pointer to a buffer containing the data
 * @param [in] len          Length of the data
 * @return file_upload_return_code_t FILE_UPLOAD_OK if the filedone corresponds to current session
 */
file_upload_return_code_t file_upload_process_file_done_frame( file_upload_t* file_upload, const uint8_t* payload,
                                                               uint8_t len );

/**
 * @brief Once the file is attached to the current upload session, a preparation must be called before start
 *
 * @param [in] file_upload Pointer to File Upload context
 * @return file_upload_return_code_t
 */
file_upload_return_code_t file_upload_prepare_upload( file_upload_t* file_upload );

/**
 * @brief get current configured average delay in seconds
 *
 * @param [in] file_upload Pointer to File Upload context
 * @return uint32_t The average delay
 */
uint32_t file_upload_get_average_delay_in_s( file_upload_t* file_upload );

/**
 * @brief File upload fragment generation
 *
 * @param [in] file_upload Pointer to File Upload context
 * @param [in] buf         buffer that will contain the fragment
 * @param [in] len         buffer size
 * @param [in] fcnt        frame counter
 * @return int32_t Return the number of pending byte(s)
 */
int32_t file_upload_get_fragment( file_upload_t* file_upload, uint8_t* buf, int32_t len, uint32_t fcnt );

/**
 * @brief Check if there are remaining file data that need to be sent
 *
 * @param [in] file_upload Pointer to File Upload context
 * @return true
 * @return false
 */
bool file_upload_is_data_remaining( file_upload_t* file_upload );

/*!
 * \brief   return file_upload_attach_payload_buffer
 * \remark
 *
 * \param  [in]     file_upload*             - Pointer to File Upload context
 * \param  [in]     file*
 * \retval          revoidturn
 */
void file_upload_attach_file_buffer( file_upload_t* file_upload, const uint8_t* file );

#ifdef __cplusplus
}
#endif

#endif  // __FILE_UPLOAD_H__

/* --- EOF ------------------------------------------------------------------ */
