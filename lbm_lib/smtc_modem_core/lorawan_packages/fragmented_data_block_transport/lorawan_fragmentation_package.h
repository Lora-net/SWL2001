/**
 * @file      lorawan_fragmentation_package.h
 *
 * @brief     Common header file for LoRa-Alliance fragmented data block transport package
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

#ifndef LORAWAN_FRAGMENTATION_PACKAGE_H
#define LORAWAN_FRAGMENTATION_PACKAGE_H

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

// An uplink may contain several commands (5 max). The maximum uplink length is as follows:
//      - PackageVersionAns : 2 bytes + cmd
//      - FragSessionStatusAns : 4 bytes + cmd
//      - FragSessionSetupAns : 1 byte + cmd
//      - FragSessionDeleteAns : 1 byte + cmd
//      - FragDataBlockReceivedReq : 1 byte + cmd
#define FRAG_UPLINK_LENGTH_MAX                                                     \
    ( FRAGMENTATION_PKG_VERSION_ANS_SIZE + FRAGMENTATION_SESSION_STATUS_ANS_SIZE + \
      FRAGMENTATION_SESSION_SETUP_ANS_SIZE + FRAGMENTATION_SESSION_DELETE_ANS_SIZE )

#define FRAG_UPLINK_REQ_MAX ( 5 )  // Maximum 5 requests per message (same request cannot be repeated)
#define FRAG_SIZE_ANS_MAX ( 15 )   // Maximum 15 bytes ans per message
// Bits of frag session status
#define FRAGMENTATION_SESSION_STATUS_MEMORY_ERROR 0
#define FRAGMENTATION_SESSION_STATUS_MIC_ERROR 1
#define FRAGMENTATION_SESSION_STATUS_NO_SESSION 2
#define FRAGMENTATION_SESSION_STATUS_SIGN_ERROR 6
#define FRAGMENTATION_SESSION_STATUS_CRC_FW_ERROR 7

// Bits of frag session setup answer
#define FRAGMENTATION_SESSION_SETUP_ALGO_UNSUPPORTED 0
#define FRAGMENTATION_SESSION_SETUP_NO_MEMORY 1
#define FRAGMENTATION_SESSION_SETUP_INDEX_UNSUPPORTED 2
#define FRAGMENTATION_SESSION_SETUP_WRONG_DESCRIPTOR 3
#define FRAGMENTATION_SESSION_SETUP_SESSIONCNT_REPLAY 4

// Bits of frag session delete answer
#define FRAGMENTATION_SESSION_DELETE_NO_SESSION 2

typedef enum
{
    FRAG_STATUS_OK,
    FRAG_STATUS_ERROR
} frag_status_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

////////////////////////////////////////////////////////
//////////// Init service object ///////////////////////
////////////////////////////////////////////////////////

/**
 * @brief Init a new LoRaWAN ALCSync services object
 *
 * @param service_id
 * @param task_id
 * @param downlink_callback
 * @param on_launch_callback
 * @param on_update_callback
 */
void lorawan_fragmentation_package_services_init( uint8_t* service_id, uint8_t task_id,
                                                  uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                                  void ( **on_launch_callback )( void* ),
                                                  void ( **on_update_callback )( void* ), void** context_callback );

/**
 * @brief Callback called at task launch
 *
 * @param context_callback
 */
void lorawan_fragmentation_package_service_on_launch( void* service_id );

/**
 * @brief Callback called at the end of the task
 *
 * @param context_callback
 */
void lorawan_fragmentation_package_service_on_update( void* service_id );

/**
 * @brief Callback to handle the downlink received by the LoRaWAN layer
 *
 * @param rx_down_data
 */
uint8_t lorawan_fragmentation_package_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/**
 * @brief Get the frag session counter
 *
 * @param [in] frag_index
 * @param [out] session_cnt
 * @return uint8_t
 */
uint8_t lorawan_fragmentation_package_get_frag_session_count( uint8_t frag_index, uint16_t* session_cnt );

/**
 * @brief Get the package information
 *
 * @param [out] pkt_id        Package ID
 * @param [out] pkt_version   Package Version
 * @param [out] pkt_port      Package FPort
 */
void lorawan_fragmentation_package_service_get_id( uint8_t* pkt_id, uint8_t* pkt_version, uint8_t* pkt_port );

/**
 * @brief MultiPackage Access Protocol injector API
 *
 * @param [in]  stack_id                  The requested stack_id
 * @param [in]  payload_in                The payload that will be parsed
 * @param [out] nb_bytes_read_payload_in  The number of bytes read in the parsed payload
 * @param [in]  rx_window                 The window that received the downlink
 * @param [out] payload_out               The returned payload
 * @param [out] payload_out_length        The number of bytes in returned payload
 * @param [in]  max_payload_out_length    The size max the payload_out could be contains
 * @param [in]  rx_timestamp_ms           The timestamp of the received window
 * @return true     successfully parsed
 * @return false    parsing error
 */
bool lorawan_fragmentation_package_service_mpa_injector( uint8_t stack_id, uint8_t* payload_in,
                                                         uint8_t* nb_bytes_read_payload_in, receive_win_t rx_window,
                                                         uint8_t* payload_out, uint8_t* payload_out_length,
                                                         const uint8_t max_payload_out_length,
                                                         uint32_t      rx_timestamp_ms );

/**
 * @brief Get the downloaded file size
 *
 * @param stack_id   The requested stack_id
 * @param file_size  The file size in bytes
 */
void lorawan_fragmentation_package_get_file_size( uint8_t stack_id, uint32_t* file_size );

#ifdef __cplusplus
}
#endif

#endif  // LORAWAN_FRAGMENTATION_PACKAGE_H

/* --- EOF ------------------------------------------------------------------ */
