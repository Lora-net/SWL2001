/*!
 * \file      fragmented_data_block.h
 *
 * \brief     LoRaWAN Fragmented Data Block Transport protocol
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

#ifndef __FRAGMENTED_DATA_BLOCK_H__
#define __FRAGMENTED_DATA_BLOCK_H__

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
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define FRAG_PACKAGE_IDENTIFIER ( 3 )
#define FRAG_PACKAGE_VERSION ( 2 )

#define FRAG_DOWNLINK_HEADER_LENGTH ( 1 )

// Answer message sizes (with header)
#define FRAG_PACKAGE_VERSION_ANS_SIZE ( 3 )
#define FRAG_SESSION_STATUS_ANS_SIZE ( 5 )
#define FRAG_SESSION_SETUP_ANS_SIZE ( 2 )
#define FRAG_SESSION_DELETE_ANS_SIZE ( 2 )
#define FRAG_DATA_BLOCK_RECEIVED_ANS_SIZE ( 2 )

// Request message sizes (with header)
#define FRAG_PACKAGE_VERSION_REQ_SIZE ( 1 )
#define FRAG_SESSION_STATUS_REQ_SIZE ( 2 )
#define FRAG_SESSION_SETUP_REQ_SIZE ( 17 )
#define FRAG_SESSION_DELETE_REQ_SIZE ( 2 )
#define FRAG_DATA_BLOCK_RECEIVED_REQ_SIZE ( 2 )

// An uplink may contain several commands (5 max). The maximum uplink length is as follows:
//      - PackageVersionAns : 2 bytes + cmd
//      - FragSessionStatusAns : 4 bytes + cmd
//      - FragSessionSetupAns : 1 byte + cmd
//      - FragSessionDeleteAns : 1 byte + cmd
//      - FragDataBlockReceivedReq : 1 byte + cmd
#define FRAG_UPLINK_LENGTH_MAX                                                                     \
    ( FRAG_PACKAGE_VERSION_ANS_SIZE + FRAG_SESSION_STATUS_ANS_SIZE + FRAG_SESSION_SETUP_ANS_SIZE + \
      FRAG_SESSION_DELETE_ANS_SIZE + FRAG_DATA_BLOCK_RECEIVED_REQ_SIZE )

#define FRAG_UPLINK_REQ_MAX ( 5 )  // Maximum 5 requests per message (same request cannot be repeated)

// Bits of frag session status
#define FRAG_SESSION_STATUS_MEMORY_ERROR 0
#define FRAG_SESSION_STATUS_MIC_ERROR 1
#define FRAG_SESSION_STATUS_NO_SESSION 2
#define FRAG_SESSION_STATUS_SIGN_ERROR 6
#define FRAG_SESSION_STATUS_CRC_FW_ERROR 7

// Bits of frag session setup answer
#define FRAG_SESSION_SETUP_ALGO_UNSUPPORTED 0
#define FRAG_SESSION_SETUP_NO_MEMORY 1
#define FRAG_SESSION_SETUP_INDEX_UNSUPPORTED 2
#define FRAG_SESSION_SETUP_WRONG_DESCRIPTOR 3
#define FRAG_SESSION_SETUP_SESSIONCNT_REPLAY 4

// Bits of frag session delete answer
#define FRAG_SESSION_DELETE_NO_SESSION 2

// Bits of data block received request
#define FRAG_RECEIVED_DATA_BLOCK_MIC_ERROR 2
#define FRAG_RECEIVED_DATA_BLOC_SIGN_ERROR 6
#define FRAG_RECEIVED_DATA_BLOC_CRC_FW_ERROR 7

#define FRAG_DATA_BLOCK_SIZE_MAX ( 30 * 1024 )  // Target 30KB: TODO refine actual space available for defrag
#define FLASH_BASE ( uint32_t ) 0x80000
#define FLASH_DELTA_UPDATE ( uint32_t ) 0xB6800

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef enum frag_cmd_length_valid
{
    FRAG_CMD_LENGTH_VALID,      //!< The length of the command is valid
    FRAG_CMD_LENGTH_NOT_VALID,  //!< The length of the command is not valid
    FRAG_CMD_NOT_VALID,         //!< The command is not valid
} e_frag_cmd_length_valid;

typedef enum frag_error
{
    FRAG_BADSIZE = -2,
    FRAG_INVALID = -1,
    FRAG_ERROR   = 0,
    FRAG_OK
} e_frag_error_t;

typedef enum file_error
{
    FILE_BADSIZE = -2,
    FILE_INVALID = -1,
    FILE_ERROR   = 0,
    FILE_OK
} e_file_error_t;

typedef enum frag_cmd
{
    FRAG_CMD_PACKAGE_VERSION          = 0x00,
    FRAG_CMD_FRAG_SESSION_STATUS      = 0x01,
    FRAG_CMD_FRAG_SESSION_SETUP       = 0x02,
    FRAG_CMD_FRAG_SESSION_DELETE      = 0x03,
    FRAG_CMD_FRAG_DATA_BLOCK_RECEIVED = 0x04,
    FRAG_CMD_FRAG_DATA_FRAGMENT       = 0x08,
    FRAG_CMD_MAX,           //!< Number of elements
    FRAG_CMD_ERROR = 0x80,  //!< Used when the command is unknown
} e_frag_cmd_t;

typedef struct frag_cmd_input
{
    e_frag_cmd_t cmd_name;    //!< command name
    uint8_t*     buffer;      //!< command data
    uint8_t      buffer_len;  //!< command data length in byte(s)
} s_frag_cmd_input_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

void frag_init( void );

int8_t frag_parser( uint8_t* frag_buffer, uint8_t frag_buffer_len );

void frag_construct_package_version_answer( void );
void frag_construct_frag_session_status_answer( void );
void frag_construct_frag_session_setup_answer( void );
void frag_construct_frag_session_delete_answer( void );
void frag_construct_data_block_received_request( void );

void frag_construct_uplink_payload( void );

e_frag_error_t frag_get_tx_buffer( uint8_t* tx_buffer_out, uint8_t* tx_buffer_length_out );
void           frag_set_max_length_up_payload( uint8_t max_payload );
bool           frag_uplink_pending( void );
// uint32_t       frag_get_data_buffer( uint8_t** data );
uint16_t frag_get_nb_frag_received( void );
uint16_t frag_get_nb_frag( void );
uint16_t frag_get_session_counter( void );
#ifdef __cplusplus
}
#endif

#endif  // __FRAGMENTED_DATA_BLOCK_H__

/* --- EOF ------------------------------------------------------------------ */
