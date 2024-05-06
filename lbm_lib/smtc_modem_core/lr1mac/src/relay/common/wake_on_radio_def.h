/*!
 * \file    wake_on_radio_def.h
 *
 * \brief   WOR and WOR ACK enum and define
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
#ifndef _WAKE_ON_RADIO_DEF_H_
#define _WAKE_ON_RADIO_DEF_H_

#ifdef _cplusplus
extern "C" {
#endif

#include "relay_def.h"

#include <stdint.h>
#include <stdbool.h>

#define WOR_ACK_UPLINK_GET_TOFFSET( a ) ( ( uint16_t )( ( a ) &0x07FF ) )
#define WOR_ACK_UPLINK_GET_CADP( a ) ( ( uint8_t )( ( ( a ) >> 11 ) & 0x0007 ) )
#define WOR_ACK_UPLINK_GET_XTAL( a ) ( ( uint8_t )( ( ( a ) >> 14 ) & 0x0003 ) )
#define WOR_ACK_UPLINK_GET_GTW_DR( a ) ( ( uint8_t )( ( ( a ) >> 16 ) & 0x000F ) )
#define WOR_ACK_UPLINK_GET_FWD( a ) ( ( uint8_t )( ( ( a ) >> 20 ) & 0x0003 ) )
#define WOR_ACK_UPLINK_GET_CAD_RX( a ) ( ( uint8_t )( ( ( a ) >> 22 ) & 0x0003 ) )

#define WOR_ACK_UPLINK_SET_TOFFSET( a ) ( ( uint32_t )( ( a ) &0x07FF ) )
#define WOR_ACK_UPLINK_SET_CADP( a ) ( ( uint32_t )( ( ( a ) &0x0007 ) << 11 ) )
#define WOR_ACK_UPLINK_SET_XTAL( a ) ( ( uint32_t )( ( ( a ) &0x0003 ) << 14 ) )
#define WOR_ACK_UPLINK_SET_GTW_DR( a ) ( ( uint32_t )( ( ( a ) &0x00F ) << 16 ) )
#define WOR_ACK_UPLINK_SET_FWD( a ) ( ( uint32_t )( ( ( a ) &0x003 ) << 20 ) )
#define WOR_ACK_UPLINK_SET_CAD_RX( a ) ( ( uint32_t )( ( ( a ) &0x003 ) << 22 ) )

#define DELAY_WOR_TO_WORACK_MS ( 50 )
#define DELAY_WORACK_TO_UPLINK_MS ( 50 )
#define DELAY_WOR_TO_JOINREQ_MS ( 50 )
#define RELAY_FWD_DELAY ( 50 )

typedef enum wor_join_request_byte_order_e
{
    WOR_JOINREQ_HEADER     = 0,
    WOR_JOINREQ_DR_PL      = 1,
    WOR_JOINREQ_FREQ_7_0   = 2,
    WOR_JOINREQ_FREQ_15_8  = 3,
    WOR_JOINREQ_FREQ_23_16 = 4,
    WOR_JOINREQ_LENGTH     = 5
} wor_join_request_byte_order_t;

typedef enum wor_standard_uplink_byte_order_e
{
    WOR_UPLINK_HEADER        = 0,
    WOR_UPLINK_DEVADDR_7_0   = 1,
    WOR_UPLINK_DEVADDR_15_8  = 2,
    WOR_UPLINK_DEVADDR_23_16 = 3,
    WOR_UPLINK_DEVADDR_31_24 = 4,
    WOR_UPLINK_PAYLOAD_ENC_1 = 5,
    WOR_UPLINK_PAYLOAD_ENC_2 = 6,
    WOR_UPLINK_PAYLOAD_ENC_3 = 7,
    WOR_UPLINK_PAYLOAD_ENC_4 = 8,
    WOR_UPLINK_FCNT_7_0      = 9,
    WOR_UPLINK_FCNT_15_8     = 10,
    WOR_UPLINK_MIC_1         = 11,
    WOR_UPLINK_MIC_2         = 12,
    WOR_UPLINK_MIC_3         = 13,
    WOR_UPLINK_MIC_4         = 14,
    WOR_UPLINK_LENGTH        = 15
} wor_standard_uplink_byte_order_t;

typedef enum wor_ack_description_byte_order_e
{
    WOR_ACK_PAYLOAD_ENC_1 = 0,
    WOR_ACK_PAYLOAD_ENC_2 = 1,
    WOR_ACK_PAYLOAD_ENC_3 = 2,
    WOR_ACK_MIC_1         = 3,
    WOR_ACK_MIC_2         = 4,
    WOR_ACK_MIC_3         = 5,
    WOR_ACK_MIC_4         = 6,
    WOR_ACK_LENGTH        = 7
} wor_ack_description_byte_order_t;

typedef enum wor_ack_fwd_limit_e
{
    WOR_ACK_FWD_LIMIT_OK          = 0,
    WOR_ACK_FWD_LIMIT_RETRY_30MIN = 1,
    WOR_ACK_FWD_LIMIT_RETRY_60MIN = 2,
    WOR_ACK_FWD_LIMIT_DISABLE     = 3,
} wor_ack_fwd_limit_t;

#ifdef _cplusplus
}
#endif
#endif
