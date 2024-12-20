/*!
 * \file    relay_tx_mac_parser.c
 *
 * \brief   MAC command parser for Relay TX
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

#include "relay_tx_mac_parser.h"
#include "relay_mac_parser.h"
#include "relay_tx_api.h"

#include "smtc_modem_hal_dbg_trace.h"
#include "lorawan_api.h"
#include "smtc_real.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

typedef enum relay_tx_cid_e
{
    RELAY_TX_CID_CONFIG_ED = 0x41,
} relay_tx_cid_t;

#define RELAY_TX_CID_CONFIG_ED_ACK_BIT_SECOND_CH_FREQUENCY ( 0 )
#define RELAY_TX_CID_CONFIG_ED_ACK_BIT_SECOND_CH_OFFSET ( 3 )
#define RELAY_TX_CID_CONFIG_ED_ACK_BIT_SECOND_CH_IDX ( 2 )
#define RELAY_TX_CID_CONFIG_ED_ACK_BIT_SECOND_CH_DR ( 1 )

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DECLARATION -------------------------------------------------
 */

/**
 * @brief Parse MAC Command 0x41 - RELAY_CID_CONFIG_ED
 *
 * @param[in]       lr1_mac LoRaWAN stack pointer
 * @param[in,out]   cmd     Mac command info
 * @return true             MAC command succesfuly decoded
 * @return false            Incorrect MAC command
 */
static bool relay_tx_mac_update_config_ed_parser( lr1_stack_mac_t* lr1_mac, relay_cmd_parser_t* cmd );

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE VARIABLES -------------------------------------------------------------
 */

const relay_cid_info_t relay_tx_cid_infos[] = {
    {
        .cid_req          = RELAY_CID_CONFIG_ED,
        .name             = "CONFIG_ED",
        .parse_cmd        = relay_tx_mac_update_config_ed_parser,
        .cid_req_min_size = 6,
        .valid_req_ret    = 0x0F,
    },
};

const uint8_t  nb_relay_mac_cmd_relay_tx     = ( sizeof( relay_tx_cid_infos ) / sizeof( relay_cid_info_t ) );
const uint16_t ack_offset_khz_realy_tx[]     = { 0, 200, 400, 800, 1600, 3200 };


/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS DEFINITIONS --------------------------------------------------
 */

bool relay_tx_mac_parser( lr1_stack_mac_t* lr1_mac )
{
    return relay_mac_parser( lr1_mac, relay_tx_cid_infos, nb_relay_mac_cmd_relay_tx );
}

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DEFINITIONS -------------------------------------------------
 */

static bool relay_tx_mac_update_config_ed_parser( lr1_stack_mac_t* lr1_mac, relay_cmd_parser_t* cmd )
{
    uint8_t        ack_ret       = cmd->valid_req_ret;
    const uint16_t settings      = ( uint16_t )( cmd->cmd_buffer[1] + ( cmd->cmd_buffer[2] << 8 ) );
    const uint8_t  second_ch_idx = TAKE_N_BITS_FROM( settings, 7, 2 );

    relay_tx_config_t config = { .smart_level = 8 << TAKE_N_BITS_FROM( cmd->cmd_buffer[0], 0, 2 ),
                                 .activation  = TAKE_N_BITS_FROM( cmd->cmd_buffer[0], 2, 2 ),
                                 .backoff     = TAKE_N_BITS_FROM( settings, 9, 6 ) };

    // Parse second channel index

    if( second_ch_idx == 0 )
    {
        config.second_ch_enable = false;
        SMTC_MODEM_HAL_TRACE_PRINTF( "No additionnal channel \n" );
    }
    else if( second_ch_idx == 1 )
    {
        config.second_ch_enable = true;

        config.second_ch.freq_hz =
            ( uint32_t )( cmd->cmd_buffer[3] + ( cmd->cmd_buffer[4] << 8 ) + ( cmd->cmd_buffer[5] << 16 ) ) * 100;
        SMTC_MODEM_HAL_TRACE_PRINTF( "Second channel freq: %d Hz\n", config.second_ch.freq_hz );

        if( smtc_real_is_frequency_valid( lr1_mac->real, config.second_ch.freq_hz ) != OKLORAWAN )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( " --> Invalid value\n" );
            ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_TX_CID_CONFIG_ED_ACK_BIT_SECOND_CH_FREQUENCY );
        }

        config.second_ch.dr = TAKE_N_BITS_FROM( settings, 3, 4 );
        SMTC_MODEM_HAL_TRACE_PRINTF( "Second channel DR: %d \n", config.second_ch.dr );

        if( smtc_real_is_tx_dr_valid( lr1_mac->real, config.second_ch.dr ) != OKLORAWAN )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( " --> Invalid value\n" );
            ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_TX_CID_CONFIG_ED_ACK_BIT_SECOND_CH_DR );
        }
        else if( smtc_real_get_modulation_type_from_datarate( lr1_mac->real, config.second_ch.dr ) != LORA )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( " --> Invalid value (NON LORA)\n" );
            ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_TX_CID_CONFIG_ED_ACK_BIT_SECOND_CH_DR );
        }

        uint8_t ack_offset_khz_idx = TAKE_N_BITS_FROM( settings, 0, 3 );

        if( ack_offset_khz_idx >= SECOND_CH_ACK_OFFSET__LAST_ELT )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Second channel ACK offset: Invalid offset (%d)\n", ack_offset_khz_idx );
            ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_TX_CID_CONFIG_ED_ACK_BIT_SECOND_CH_OFFSET );
        }
        else
        {
            config.second_ch.ack_freq_hz = config.second_ch.freq_hz + ack_offset_khz_realy_tx[ack_offset_khz_idx] * 1000;

            SMTC_MODEM_HAL_TRACE_PRINTF( "Second channel ACK freq: %d Hz\n", config.second_ch.ack_freq_hz );
            if( smtc_real_is_frequency_valid( lr1_mac->real, config.second_ch.ack_freq_hz ) != OKLORAWAN )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( " --> Invalid value\n" );
                ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_TX_CID_CONFIG_ED_ACK_BIT_SECOND_CH_OFFSET );
            }
        }
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Additionnal channel index not supported\n" );
        ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_TX_CID_CONFIG_ED_ACK_BIT_SECOND_CH_IDX );
    }

    if( ack_ret == cmd->valid_req_ret )
    {
        if( smtc_relay_tx_update_config( lr1_mac->stack_id, &config ) == true )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "New config has been accepted\n" );
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Config has been refused by LBM\n" );
            ack_ret = 0;  // config has been refused
        }
    }

    cmd->cmd_len -= cmd->cmd_min_len;
    cmd->out_buff[0] = ack_ret;
    cmd->out_len     = 1;
    return true;
}
