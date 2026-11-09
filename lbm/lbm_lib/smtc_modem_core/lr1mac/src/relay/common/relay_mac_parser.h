/*!
 * \file    relay_mac_parser.h
 *
 * \brief   General parser of relay MAC command
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
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
#ifndef RELAY_MAC_PARSER_H
#define RELAY_MAC_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif
/*
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include "lr1_stack_mac_layer.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
/*!
 * \brief Get a particular bit value from a byte
 *
 * \param [IN] b Any multibyte from which we want a bit value
 * \param [IN] p Position of the bit in the multibyte [0..31]
 * \param [IN] n Number of bits we want to get
 * \retval The value corresponding the requested bits
 */
#define TAKE_N_BITS_FROM( b, p, n ) ( ( ( b ) >> ( p ) ) & ( ( 1 << ( n ) ) - 1 ) )
#define RELAY_CLEAR_BIT( b, n ) ( ( b ) & ~( ( 1 << ( n ) ) ) )

typedef enum
{
    SECOND_CH_ACK_OFFSET_0KHZ,
    SECOND_CH_ACK_OFFSET_2KHZ,
    SECOND_CH_ACK_OFFSET_4KHZ,
    SECOND_CH_ACK_OFFSET_8KHZ,
    SECOND_CH_ACK_OFFSET_16KHZ,
    SECOND_CH_ACK_OFFSET_32KHZ,
    SECOND_CH_ACK_OFFSET__LAST_ELT,
} relay_second_ch_ack_offset_t;

typedef enum relay_cid_e
{
    RELAY_CID_CONFIG_RELAY      = 0x40,
    RELAY_CID_CONFIG_ED         = 0x41,
    RELAY_CID_JOINREQ_FILTER    = 0x42,
    RELAY_CID_ADD_TRUSTED_ED    = 0x43,
    RELAY_CID_REMOVE_TRUSTED_ED = 0x44,
    RELAY_CID_FWD_LIMIT         = 0x45,
    RELAY_CID_NOTIFY            = 0x46,
} relay_cid_t;

typedef struct relay_cmd_parser_s
{
    const uint8_t* cmd_buffer;     // Buffer with the MAC command. (Can contain more than one)
    uint8_t        cmd_len;        // Buffer length in byte
    uint8_t        cmd_min_len;    // Minimal length command for a particular CID
    uint8_t*       out_buff;       // Output buffer for MAC command response
    uint8_t        out_len;        // Output length
    uint8_t        valid_req_ret;  // Expected return status if all parameters are valid
} relay_cmd_parser_t;

typedef struct relay_tx_cid_info_s
{
    uint8_t cid_req;                                               // CID opcode
    uint8_t cid_req_min_size;                                      // CID min size request
    uint8_t valid_req_ret;                                         // Valid return code if all parameters are valid.
    bool ( *parse_cmd )( lr1_stack_mac_t*, relay_cmd_parser_t* );  // Parsing function for this CID
    char* name;                                                    // Friendly debug name
} relay_cid_info_t;
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE VARIABLES -------------------------------------------------------------
 */

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DECLARATION -------------------------------------------------
 */

/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS DEFINITIONS --------------------------------------------------
 */

/**
 * @brief Parse MAC command for relay (RX or TX)
 *
 * @param[in,out]   lr1_mac lr1mac object
 * @param[in]       cid     table definition of all mac command
 * @param[in]       cid_len table len
 * @return true     MAC command is known
 * @return false    MAC command is unknown
 */
bool relay_mac_parser( lr1_stack_mac_t* lr1_mac, const relay_cid_info_t* cid, const uint8_t cid_len );

#ifdef __cplusplus
}
#endif
#endif
