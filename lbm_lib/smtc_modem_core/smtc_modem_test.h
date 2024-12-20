/*!
 * \file      modem_test.h
 *
 * \brief     soft modem task scheduler
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

#ifndef __SMTC_MODEM_TEST__H
#define __SMTC_MODEM_TEST__H

#ifdef __cplusplus
extern "C" {
#endif
#include "radio_planner.h"
#include "lr1_stack_mac_layer.h"
#include "smtc_lbt.h"
/*!
 * \typedef modem_test_context_t
 * \brief   Test context
 */
typedef struct modem_test_context
{
    radio_planner_t*  rp;                      //!< Radio planner instance
    lr1_stack_mac_t*  lr1_mac_obj;             //!< Lorawan lr1mac instance
    uint8_t           hook_id;                 //!< Lorawan lr1mac hook id used for test
    uint8_t           tx_rx_payload[255];      //!< Transmit/Received buffer
    int16_t           last_rx_payload_rssi;    //!< Placeholder for last rssi
    int16_t           last_rx_payload_snr;     //!< Placeholder for last snr
    uint8_t           last_rx_payload_length;  //!< Placeholder for last payload length
    int16_t           rssi;                    //!< Placeholder for mean rssi
    bool              rssi_ready;              //!< True when rssi mean test is finished
    uint32_t          total_rx_packets;        //!< Number of received packet
    bool              random_payload;          //!< True in case of random payload
    uint32_t          tx_frequency;            //!< True in case of random payload
    ral_lora_sf_t     sf;                      //!< True in case of random payload
    ral_lora_bw_t     bw;                      //!< True in case of random payload
    bool              invert_iq;               //!< True in case of random payload
    uint32_t          nb_of_repetition;        //!< Used for multiple tx in test mode
    modulation_type_t modulation_type;         //!< modulation type
    uint32_t          delay_ms;                //!< delay between two repetition tx in test mode
    smtc_lbt_t        lbt_obj;
} modem_test_context_t;

modem_test_context_t* smtc_modem_test_get_context( void );
status_lorawan_t      test_mode_cb_tpm( uint8_t* payload, uint8_t payload_length, bool abort );
#ifdef __cplusplus
}
#endif

#endif