/*!
 * \file    relay_mac_parser.c
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

/*
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */

#include "relay_mac_parser.h"
#include "smtc_modem_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DECLARATION -------------------------------------------------
 */

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE VARIABLES -------------------------------------------------------------
 */

/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS DEFINITIONS --------------------------------------------------
 */

bool relay_mac_parser( lr1_stack_mac_t* lr1_mac, const relay_cid_info_t* cid, const uint8_t cid_len )
{
    if( lr1_mac->nwk_payload_size <= lr1_mac->nwk_payload_index )
    {
        return false;
    }

    for( uint8_t i = 0; i < cid_len; i++ )
    {
        if( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index] == cid[i].cid_req )
        {
            lr1_mac->nwk_payload_index += 1;  // Remove CID

            const uint8_t cmd_len_remaining = lr1_mac->nwk_payload_size - lr1_mac->nwk_payload_index;

            relay_cmd_parser_t cmd_to_parse = {
                .cmd_buffer  = lr1_mac->nwk_payload + lr1_mac->nwk_payload_index,
                .cmd_min_len = cid[i].cid_req_min_size,

                .out_buff = lr1_mac->tx_fopts_data + lr1_mac->tx_fopts_length + 1,
                .out_len  = 0,

                .cmd_len       = cmd_len_remaining,
                .valid_req_ret = cid[i].valid_req_ret,
            };

            SMTC_MODEM_HAL_TRACE_PRINTF( "Relay MAC command : 0x%02x - %s\n", cid[i].cid_req, cid[i].name );

            if( ( cmd_len_remaining >= cid[i].cid_req_min_size ) && cid[i].parse_cmd( lr1_mac, &cmd_to_parse ) == true )
            {
                lr1_mac->nwk_payload_index += ( cmd_len_remaining - cmd_to_parse.cmd_len );
                lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length] = cid[i].cid_req;
                lr1_mac->tx_fopts_length += cmd_to_parse.out_len + 1;
            }
            else
            {
                // Set payload index to the end in case of parsing error
                lr1_mac->nwk_payload_index = lr1_mac->nwk_payload_size;
            }
            return true;
        }
    }
    return false;
}
