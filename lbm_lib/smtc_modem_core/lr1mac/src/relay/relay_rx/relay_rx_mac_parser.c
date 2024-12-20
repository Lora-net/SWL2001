/*!
 * \file    relay_rx_mac_parser.c
 *
 * \brief   MAC command parser for Relay RX
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

#include "relay_rx_mac_parser.h"
#include "relay_mac_parser.h"
#include "relay_rx_api.h"
#include "relay_real.h"
#include "smtc_real.h"
#include "smtc_modem_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

#ifndef MAX
#define MAX( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
#define RELAY_CID_CONFIG_RELAY_ACK_BIT_CAD_PERIOD ( 5 )
#define RELAY_CID_CONFIG_RELAY_ACK_BIT_DEFAULT_CH_IDX ( 4 )
#define RELAY_CID_CONFIG_RELAY_ACK_BIT_SECOND_CH_IDX ( 3 )
#define RELAY_CID_CONFIG_RELAY_ACK_BIT_SECOND_CH_DR ( 2 )
#define RELAY_CID_CONFIG_RELAY_ACK_BIT_SECOND_CH_OFFSET ( 1 )
#define RELAY_CID_CONFIG_RELAY_ACK_BIT_SECOND_CH_FREQUENCY ( 0 )

#define RELAY_CID_JOINREQ_FWD_FILTER_LIST_ACK_BIT_ACTION ( 0 )
#define RELAY_CID_JOINREQ_FWD_FILTER_LIST_ACK_BIT_LEN ( 1 )
#define RELAY_CID_JOINREQ_FWD_FILTER_LIST_ACK_BIT_COMBINED_RULE ( 2 )

#define RELAY_CID_REMOVE_TRUSTED_ED_ACK_BIT_IDX ( 0 )

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DECLARATION -------------------------------------------------
 */

/**
 * @brief Parse MAC Command 0x40 - RELAY_CID_CONFIG_RELAY
 *
 * @param[in]       lr1_mac LoRaWAN stack pointer
 * @param[in,out]   cmd     Mac command info
 * @return true             MAC command successfully decoded
 * @return false            Incorrect MAC command
 */
static bool relay_mac_update_config_relay_parser( lr1_stack_mac_t* lr1_mac, relay_cmd_parser_t* cmd );

/**
 * @brief Parse MAC Command 0x42 - RELAY_CID_JOINREQ_FILTER
 *
 * @param[in]       lr1_mac LoRaWAN stack pointer
 * @param[in,out]   cmd     Mac command info
 * @return true             MAC command successfully decoded
 * @return false            Incorrect MAC command
 */
static bool relay_mac_update_joinreq_list_parser( lr1_stack_mac_t* lr1_mac, relay_cmd_parser_t* cmd );

/**
 * @brief Parse MAC Command 0x43 - RELAY_CID_ADD_TRUSTED_ED
 *
 * @param[in]       lr1_mac LoRaWAN stack pointer
 * @param[in,out]   cmd     Mac command info
 * @return true             MAC command successfully decoded
 * @return false            Incorrect MAC command
 */
static bool relay_mac_add_trusted_ed_parser( lr1_stack_mac_t* lr1_mac, relay_cmd_parser_t* cmd );

/**
 * @brief Parse MAC Command 0x44 - RELAY_CID_REMOVE_TRUSTED_ED
 *
 * @param[in]       lr1_mac LoRaWAN stack pointer
 * @param[in,out]   cmd     Mac command info
 * @return true             MAC command successfully decoded
 * @return false            Incorrect MAC command
 */
static bool relay_mac_manage_trusted_ed_parser( lr1_stack_mac_t* lr1_mac, relay_cmd_parser_t* cmd );

/**
 * @brief Parse MAC Command 0x45 - RELAY_CID_FWD_LIMIT
 *
 * @param[in]       lr1_mac LoRaWAN stack pointer
 * @param[in,out]   cmd     Mac command info
 * @return true             MAC command successfully decoded
 * @return false            Incorrect MAC command
 */
static bool relay_mac_manage_fwd_list_parser( lr1_stack_mac_t* lr1_mac, relay_cmd_parser_t* cmd );

/**
 * @brief Concatenate 4 LSB bytes into uint32
 *
 * @param buffer[in]    Input buffer
 * @return uint32_t     value
 */
static uint32_t concatenate_byte_lsb_to_uint32( const uint8_t* buffer );

static uint32_t get_symbol_time_ms( smtc_real_t* real, uint8_t dr );
/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE VARIABLES -------------------------------------------------------------
 */

const relay_cid_info_t relay_rx_cid_infos[] = {
    {
        .cid_req          = RELAY_CID_CONFIG_RELAY,
        .name             = "CONFIG_RELAY",
        .parse_cmd        = relay_mac_update_config_relay_parser,
        .cid_req_min_size = 5,
        .valid_req_ret    = 0x3F,
    },
    {
        .cid_req          = RELAY_CID_JOINREQ_FILTER,
        .name             = "JOINREQ_FILTER",
        .parse_cmd        = relay_mac_update_joinreq_list_parser,
        .cid_req_min_size = 2,  // 2 bytes + variable (up to 16)
        .valid_req_ret    = 0x07,
    },
    {
        .cid_req          = RELAY_CID_ADD_TRUSTED_ED,
        .name             = "ADD_TRUSTED_ED",
        .parse_cmd        = relay_mac_add_trusted_ed_parser,
        .cid_req_min_size = 26,
        // .valid_req_ret    = No return for this cmd

    },
    {
        .cid_req          = RELAY_CID_REMOVE_TRUSTED_ED,
        .name             = "REMOVE_TRUSTED_ED",
        .parse_cmd        = relay_mac_manage_trusted_ed_parser,
        .cid_req_min_size = 1,
        .valid_req_ret    = 0x01,
    },
    {
        .cid_req          = RELAY_CID_FWD_LIMIT,
        .name             = "FWD_LIMIT",
        .parse_cmd        = relay_mac_manage_fwd_list_parser,
        .cid_req_min_size = 5,
        // .valid_req_ret    = No return for this cmd
    },
};
const uint8_t  nb_relay_mac_cmd = ( sizeof( relay_rx_cid_infos ) / sizeof( relay_cid_info_t ) );
const uint16_t ack_offset_khz[SECOND_CH_ACK_OFFSET__LAST_ELT] = { 0, 200, 400, 800, 1600, 3200 };
const uint16_t convert_bucket_size_factor[]                   = { 1, 2, 4, 12 };

#if ( MODEM_HAL_DBG_TRACE )
const char* cad_period_str[]       = { "1 s", "500 ms", "250 ms", "100 ms", "50 ms", "20 ms" };
const char* ack_offset_khz_str[]   = { "0 kHz", "2 kHz", "4 kHz", "8 kHz", "16 kHz", "32 kHz" };
const char* action_fwd_str[]       = { "FWD_CLEAR", "FWD_FORWARD", "FWD_FILTER", "FWD_RFU" };
const char* limit_fwd_str[]        = { "Overall ", "Global  ", "Notify  ", "Join Req" };
const char* rst_limit_action_str[] = { "Set to 0", "Set to reload rate", "Set to max val", "Dont change" };
#endif
/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS DEFINITIONS --------------------------------------------------
 */

bool relay_rx_mac_parser( lr1_stack_mac_t* lr1_mac )
{
    return relay_mac_parser( lr1_mac, relay_rx_cid_infos, nb_relay_mac_cmd );
}

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DEFINITIONS -------------------------------------------------
 */

static bool relay_mac_update_config_relay_parser( lr1_stack_mac_t* lr1_mac, relay_cmd_parser_t* cmd )
{
    uint8_t        ack_ret        = cmd->valid_req_ret;
    const uint16_t settings_relay = ( uint16_t ) ( cmd->cmd_buffer[0] + ( cmd->cmd_buffer[1] << 8 ) );
    const bool     start_stop     = TAKE_N_BITS_FROM( settings_relay, 13, 1 );

    if( start_stop == false )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Stop relay\n" );
        relay_stop( false );
        relay_rx_set_flag_started( false );
    }
    else
    {
        relay_config_t config        = { 0 };
        uint32_t       cad_period_ms = 1000;
        uint8_t        second_ch_idx = TAKE_N_BITS_FROM( settings_relay, 7, 2 );

        // Parse CAD Period
        config.cad_period = TAKE_N_BITS_FROM( settings_relay, 10, 3 );

        if( config.cad_period >= WOR_CAD_PERIOD_RFU )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "CAD period : wrong value \n" );
            ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_CONFIG_RELAY_ACK_BIT_CAD_PERIOD );
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "CAD period : %s\n", cad_period_str[config.cad_period] );
            cad_period_ms = wor_convert_cad_period_in_ms( config.cad_period ) >> second_ch_idx;
        }

        uint8_t default_chx_idx = TAKE_N_BITS_FROM( settings_relay, 9, 1 );
        if( smtc_relay_get_default_channel_config( lr1_mac->real, default_chx_idx, &( config.channel_cfg[0].dr ),
                                                   &( config.channel_cfg[0].freq_hz ),
                                                   &( config.channel_cfg[0].ack_freq_hz ) ) != OKLORAWAN )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Default channel : Index %d is not supported\n", default_chx_idx );
            ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_CONFIG_RELAY_ACK_BIT_DEFAULT_CH_IDX );
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Default channel : Index %d (DR %d - WOR %d Hz - ACK %d Hz)\n",
                                         default_chx_idx, config.channel_cfg[0].dr, config.channel_cfg[0].freq_hz,
                                         config.channel_cfg[0].ack_freq_hz );

            const uint32_t symb_toa = 2 * get_symbol_time_ms( lr1_mac->real, config.channel_cfg[0].dr ) + 10;

            if( cad_period_ms < symb_toa )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "CAD period too short \n" );
                ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_CONFIG_RELAY_ACK_BIT_CAD_PERIOD );
            }
        }

        // Parse second channel index

        if( second_ch_idx == 0 )
        {
            config.nb_wor_channel = 1;
            SMTC_MODEM_HAL_TRACE_PRINTF( "No additionnal channel \n" );
        }
        else if( second_ch_idx == 1 )
        {
            config.nb_wor_channel = 2;
            config.channel_cfg[1].freq_hz =
                ( uint32_t ) ( cmd->cmd_buffer[2] + ( cmd->cmd_buffer[3] << 8 ) + ( cmd->cmd_buffer[4] << 16 ) ) * 100;
            SMTC_MODEM_HAL_TRACE_PRINTF( "Second channel freq: %d Hz\n", config.channel_cfg[1].freq_hz );

            if( smtc_real_is_frequency_valid( lr1_mac->real, config.channel_cfg[1].freq_hz ) != OKLORAWAN )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "Second channel freq: Invalid value\n" );
                ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_CONFIG_RELAY_ACK_BIT_SECOND_CH_FREQUENCY );
            }

            config.channel_cfg[1].dr = TAKE_N_BITS_FROM( settings_relay, 3, 4 );
            SMTC_MODEM_HAL_TRACE_PRINTF( "Second channel DR: %d \n", config.channel_cfg[1].dr );

            if( smtc_real_is_tx_dr_valid( lr1_mac->real, config.channel_cfg[1].dr ) != OKLORAWAN )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "Second channel DR: Invalid value\n" );
                ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_CONFIG_RELAY_ACK_BIT_SECOND_CH_DR );
            }
            else if( smtc_real_get_modulation_type_from_datarate( lr1_mac->real, config.channel_cfg[1].dr ) != LORA )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "Second channel DR: Non LORA\n" );
                ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_CONFIG_RELAY_ACK_BIT_SECOND_CH_DR );
            }
            else
            {
                const uint32_t symb_toa = 2 * get_symbol_time_ms( lr1_mac->real, config.channel_cfg[1].dr ) + 10;
                if( cad_period_ms < symb_toa )
                {
                    SMTC_MODEM_HAL_TRACE_PRINTF( "CAD period 2 too short \n" );
                    ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_CONFIG_RELAY_ACK_BIT_CAD_PERIOD );
                }
            }

            uint8_t ack_offset_khz_idx = TAKE_N_BITS_FROM( settings_relay, 0, 3 );

            if( ack_offset_khz_idx < SECOND_CH_ACK_OFFSET__LAST_ELT )
            {
                config.channel_cfg[1].ack_freq_hz =
                    config.channel_cfg[1].freq_hz + ack_offset_khz[ack_offset_khz_idx] * 1000;

                SMTC_MODEM_HAL_TRACE_PRINTF( "Second channel ACK offset: %s\n",
                                             ack_offset_khz_str[ack_offset_khz_idx] );

                if( smtc_real_is_frequency_valid( lr1_mac->real, config.channel_cfg[1].ack_freq_hz ) != OKLORAWAN )
                {
                    SMTC_MODEM_HAL_TRACE_PRINTF( "Second channel ACK freq: Invalid value (%d Hz)\n",
                                                 config.channel_cfg[1].ack_freq_hz );
                    ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_CONFIG_RELAY_ACK_BIT_SECOND_CH_OFFSET );
                }
                else
                {
                    SMTC_MODEM_HAL_TRACE_PRINTF( "Second channel ACK freq: %d Hz\n",
                                                 config.channel_cfg[1].ack_freq_hz );
                }
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "Second channel ACK offset: Invalid offset (%d)\n", ack_offset_khz_idx );
                ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_CONFIG_RELAY_ACK_BIT_SECOND_CH_OFFSET );
            }
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Additionnal channel index not supported\n" );
            ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_CONFIG_RELAY_ACK_BIT_SECOND_CH_IDX );
        }

        if( ack_ret == cmd->valid_req_ret )
        {
            if( relay_update_config( &config ) == true )
            {
                relay_start( );
                relay_rx_set_flag_started( true );
                SMTC_MODEM_HAL_TRACE_PRINTF( "New config has been accepted and relay has started\n" );
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "Wrong config\n" );
                ack_ret = 0;  // config has been refused
            }
        }
    }

    cmd->cmd_len -= cmd->cmd_min_len;

    cmd->out_buff[0] = ack_ret;
    cmd->out_len     = 1;
    return true;
}

static bool relay_mac_update_joinreq_list_parser( lr1_stack_mac_t* lr1_mac, relay_cmd_parser_t* cmd )
{
    uint8_t        ack_ret           = cmd->valid_req_ret;
    const uint16_t filter_list_param = ( uint16_t ) ( cmd->cmd_buffer[0] + ( cmd->cmd_buffer[1] << 8 ) );
    const uint8_t  len               = TAKE_N_BITS_FROM( filter_list_param, 0, 5 );
    const uint8_t  action            = TAKE_N_BITS_FROM( filter_list_param, 5, 2 );
    const uint8_t  idx               = TAKE_N_BITS_FROM( filter_list_param, 7, 4 );

    // Variable size command, check param len
    if( ( cmd->cmd_min_len + len ) > cmd->cmd_len )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Not enough param\n" );
        return false;
    }

    if( len > 16 )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Lengh is too long (%d)\n", len );
        ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_JOINREQ_FWD_FILTER_LIST_ACK_BIT_LEN );
    }

    if( action >= RELAY_FILTER_FWD_TYPE_RFU )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Action is not supported (%d)\n", action );
        ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_JOINREQ_FWD_FILTER_LIST_ACK_BIT_ACTION );
    }

    if( idx == 0 )
    {
        if( len != 0 )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Rule 0 should have a 0 len\n" );
            ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_JOINREQ_FWD_FILTER_LIST_ACK_BIT_COMBINED_RULE );
        }

        if( ( action != RELAY_FILTER_FWD_TYPE_FILTER ) && ( action != RELAY_FILTER_FWD_TYPE_FORWARD ) )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Rule 0 should be filter or forward\n" );
            ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_JOINREQ_FWD_FILTER_LIST_ACK_BIT_COMBINED_RULE );
        }
    }
    else  // idx != 0
    {
        if( ( len == 0 ) && ( action != RELAY_FILTER_FWD_TYPE_CLEAR ) )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Rule 1-15 should not have a 0 len with FWD or FILTER\n" );
            ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_JOINREQ_FWD_FILTER_LIST_ACK_BIT_COMBINED_RULE );
        }

        if( ( len != 0 ) && ( action == RELAY_FILTER_FWD_TYPE_CLEAR ) )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Rule 1-15 should have a 0 len with CLEAR\n" );
            ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_JOINREQ_FWD_FILTER_LIST_ACK_BIT_COMBINED_RULE );
        }
    }

    if( ack_ret == cmd->valid_req_ret )
    {
        uint8_t join_eui[8];
        uint8_t dev_eui[8];
        uint8_t len_join_eui = MIN( 8, len );
        uint8_t len_dev_eui  = ( ( len > 8 ) ? len - 8 : 0 );

        for( uint8_t i = 0; i < len_join_eui; i++ )
        {
            join_eui[i] = cmd->cmd_buffer[2 + len - 1 - i];
        }

        for( uint8_t i = 0; i < len_dev_eui; i++ )
        {
            dev_eui[i] = cmd->cmd_buffer[2 + len - 1 - i - len_join_eui];
        }

        SMTC_MODEM_HAL_TRACE_PRINTF( "Update rule %d with %s \n", idx, action_fwd_str[action] );
        if( len > 0 )
        {
            SMTC_MODEM_HAL_TRACE_PACKARRAY( "Join EUI ", join_eui, len_join_eui );
        }
        if( len > 8 )
        {
            SMTC_MODEM_HAL_TRACE_PACKARRAY( "Dev EUI  ", dev_eui, len_dev_eui );
        }

        relay_fwd_join_request_update_rule( idx, join_eui, len_join_eui, dev_eui, len_dev_eui, action );
    }

    cmd->cmd_len -= ( cmd->cmd_min_len + len );
    cmd->out_buff[0] = ack_ret;
    cmd->out_len     = 1;
    return true;
}

static bool relay_mac_add_trusted_ed_parser( lr1_stack_mac_t* lr1_mac, relay_cmd_parser_t* cmd )
{
    const uint8_t  idx         = TAKE_N_BITS_FROM( cmd->cmd_buffer[0], 0, 4 );
    const uint8_t  reload_rate = TAKE_N_BITS_FROM( cmd->cmd_buffer[1], 0, 6 );
    const uint8_t  factor      = TAKE_N_BITS_FROM( cmd->cmd_buffer[1], 6, 2 );
    const uint32_t dev_addr    = concatenate_byte_lsb_to_uint32( cmd->cmd_buffer + 2 );
    const uint32_t wfcnt32     = concatenate_byte_lsb_to_uint32( cmd->cmd_buffer + 6 );

    const bool unlimited_fwd = ( reload_rate == 63 ) ? true : false;

    uint8_t wor_s_key[16];
    // Manage WOR S KEY as MSB
    memcpy( wor_s_key, cmd->cmd_buffer + 10, 16 );

    // // Manage WOR S KEY as LSB
    // for( uint8_t i = 0; i < 16; i++ )
    // {
    //     wor_s_key[15 - i] = cmd->cmd_buffer[10 + i];
    // }

    SMTC_MODEM_HAL_TRACE_PRINTF( "Idx       %d\n", idx );
    SMTC_MODEM_HAL_TRACE_PRINTF( "DevAddr   0x%0X\n", dev_addr );
    SMTC_MODEM_HAL_TRACE_PRINTF( "WFCnt32   %d\n", wfcnt32 );
    SMTC_MODEM_HAL_TRACE_PACKARRAY( "WOR S KEY ", wor_s_key, 16 );
    if( unlimited_fwd == true )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Bucket    Unlimited\n" );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Bucket    %d x %d\n", reload_rate, convert_bucket_size_factor[factor] );
    }

    relay_fwd_uplink_add_device( idx, dev_addr, wor_s_key, unlimited_fwd, reload_rate,
                                 convert_bucket_size_factor[factor], wfcnt32 );

    cmd->cmd_len -= cmd->cmd_min_len;
    cmd->out_len = 0;  // No output param
    return true;
}

static bool relay_mac_manage_trusted_ed_parser( lr1_stack_mac_t* lr1_mac, relay_cmd_parser_t* cmd )
{
    uint8_t       ack_ret = cmd->valid_req_ret;
    const uint8_t idx     = TAKE_N_BITS_FROM( cmd->cmd_buffer[0], 0, 4 );
    const uint8_t action  = TAKE_N_BITS_FROM( cmd->cmd_buffer[0], 4, 1 );

    uint32_t wfcnt32 = 0;

    if( relay_fwd_uplink_read_wfcnt32( idx, &wfcnt32 ) == false )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Idx : invalid value (%d)\n", idx );
        ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_REMOVE_TRUSTED_ED_ACK_BIT_IDX );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Idx :     %d\n", idx );
        SMTC_MODEM_HAL_TRACE_PRINTF( "WFCnt32 : %d\n", wfcnt32 );
        SMTC_MODEM_HAL_TRACE_PRINTF( "Action :  %s\n", ( action == 0 ? "read" : "remove" ) );

        if( action != 0 )
        {
            if( relay_fwd_uplink_remove_device( idx ) == false )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "Idx : invalid value (failed to remove)\n" );
                ack_ret = RELAY_CLEAR_BIT( ack_ret, RELAY_CID_REMOVE_TRUSTED_ED_ACK_BIT_IDX );
            }
        }
    }

    cmd->cmd_len -= cmd->cmd_min_len;

    cmd->out_buff[0] = ack_ret;
    cmd->out_buff[1] = ( wfcnt32 >> 0 ) & 0xFF;
    cmd->out_buff[2] = ( wfcnt32 >> 8 ) & 0xFF;
    cmd->out_buff[3] = ( wfcnt32 >> 16 ) & 0xFF;
    cmd->out_buff[4] = ( wfcnt32 >> 24 ) & 0xFF;
    cmd->out_len     = 5;
    return true;
}

static bool relay_mac_manage_fwd_list_parser( lr1_stack_mac_t* lr1_mac, relay_cmd_parser_t* cmd )
{
    const uint32_t reload_rate = concatenate_byte_lsb_to_uint32( cmd->cmd_buffer );
    const uint8_t  factor      = cmd->cmd_buffer[4];

    const relay_forward_limit_action_t reset_limit =
        ( relay_forward_limit_action_t ) TAKE_N_BITS_FROM( reload_rate, 28, 2 );

    SMTC_MODEM_HAL_TRACE_PRINTF( "Reset : %s\n", rst_limit_action_str[reset_limit] );

    for( uint8_t i = 0; i < LIMIT__LAST_ELT; i++ )
    {
        const uint16_t limit_reload_rate = TAKE_N_BITS_FROM( reload_rate, 7 * i, 7 );
        const uint8_t  limit_factor      = convert_bucket_size_factor[TAKE_N_BITS_FROM( factor, 2 * i, 2 )];
        const bool     unlimited_fwd     = ( limit_reload_rate == 127 ) ? true : false;

        if( unlimited_fwd == true )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "- %s Unlimited\n", limit_fwd_str[i] );
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( " - %s %d x %d\n", limit_fwd_str[i], limit_reload_rate, limit_factor );
        }

        relay_fwd_update_fwd_limit( i, reset_limit, unlimited_fwd, limit_reload_rate, limit_factor );
    }

    cmd->cmd_len -= cmd->cmd_min_len;

    cmd->out_len = 0;  // No output param
    return true;
}

static uint32_t concatenate_byte_lsb_to_uint32( const uint8_t* buffer )
{
    return ( uint32_t ) ( ( ( uint32_t ) buffer[0] ) + ( ( uint32_t ) buffer[1] << 8 ) +
                          ( ( uint32_t ) buffer[2] << 16 ) + ( ( uint32_t ) buffer[3] << 24 ) );
}

static uint32_t get_symbol_time_ms( smtc_real_t* real, uint8_t dr )
{
    uint8_t            sf;
    lr1mac_bandwidth_t bw;
    smtc_real_lora_dr_to_sf_bw( real, dr, &sf, &bw );

    uint32_t toa_symb = 1 << sf;

    switch( bw )
    {
    case BW125:
        toa_symb = toa_symb / 125;
        break;
    case BW250:
        toa_symb = toa_symb / 250;
        break;
    case BW500:
        toa_symb = toa_symb / 500;
        break;
    case BW800:
        toa_symb = toa_symb / 800;
        break;
    case BW1600:
        toa_symb = toa_symb / 1600;
        break;

    default:
        break;
    }

    SMTC_MODEM_HAL_TRACE_PRINTF( "TOA symbol DR%d : %d ms\n", dr, toa_symb );

    return toa_symb;
}