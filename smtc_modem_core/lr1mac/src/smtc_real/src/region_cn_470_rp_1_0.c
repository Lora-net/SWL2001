/**
 * \file      region_cn_470_rp_1_0.c
 *
 * \brief     region_cn_470_rp_1_0 abstraction layer implementation
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <string.h>  // memcpy
#include "lr1mac_utilities.h"
#include "smtc_modem_hal.h"
#include "region_cn_470_rp_1_0_defs.h"
#include "region_cn_470_rp_1_0.h"
#include "smtc_modem_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define real_ctx real->real_ctx
#define real_const real->real_const

#define dr_bitfield_tx_channel real->region.cn470_rp_1_0.dr_bitfield_tx_channel
#define channel_index_enabled real->region.cn470_rp_1_0.channel_index_enabled
#define dr_distribution_init real->region.cn470_rp_1_0.dr_distribution_init
#define dr_distribution real->region.cn470_rp_1_0.dr_distribution
#define unwrapped_channel_mask real->region.cn470_rp_1_0.unwrapped_channel_mask

#define snapshot_bank_tx_mask real->region.cn470_rp_1_0.snapshot_bank_tx_mask
// Private region_cn_470_rp_1_0 utilities declaration
//

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
#if defined( HYBRID_CN470_MONO_CHANNEL )
uint32_t freq_tx_cn470_mono_channel_mhz = 471100000;
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void region_cn_470_rp_1_0_init( smtc_real_t* real )
{
    real_const.const_number_of_tx_channel         = NUMBER_OF_TX_CHANNEL_CN_470_RP_1_0;
    real_const.const_number_of_rx_channel         = NUMBER_OF_RX_CHANNEL_CN_470_RP_1_0;
    real_const.const_number_of_channel_bank       = BANK_MAX_CN470_RP_1_0;
    real_const.const_join_accept_delay1           = JOIN_ACCEPT_DELAY1_CN_470_RP_1_0;
    real_const.const_received_delay1              = RECEIVE_DELAY1_CN_470_RP_1_0;
    real_const.const_tx_power_dbm                 = TX_POWER_EIRP_CN_470_RP_1_0 - 2;  // EIRP to ERP
    real_const.const_max_tx_power_idx             = MAX_TX_POWER_IDX_CN_470_RP_1_0;
    real_const.const_adr_ack_limit                = ADR_ACK_LIMIT_CN_470_RP_1_0;
    real_const.const_adr_ack_delay                = ADR_ACK_DELAY_CN_470_RP_1_0;
    real_const.const_datarate_offsets             = &datarate_offsets_cn_470_rp_1_0[0][0];
    real_const.const_datarate_backoff             = &datarate_backoff_cn_470_rp_1_0[0];
    real_const.const_ack_timeout                  = ACK_TIMEOUT_CN_470_RP_1_0;
    real_const.const_freq_min                     = FREQMIN_CN_470_RP_1_0;
    real_const.const_freq_max                     = FREQMAX_CN_470_RP_1_0;
    real_const.const_rx2_freq                     = RX2_FREQ_CN_470_RP_1_0;
    real_const.const_frequency_factor             = FREQUENCY_FACTOR_CN_470_RP_1_0;
    real_const.const_rx2_dr_init                  = RX2DR_INIT_CN_470_RP_1_0;
    real_const.const_sync_word_private            = SYNC_WORD_PRIVATE_CN_470_RP_1_0;
    real_const.const_sync_word_public             = SYNC_WORD_PUBLIC_CN_470_RP_1_0;
    real_const.const_sync_word_gfsk               = ( uint8_t* ) SYNC_WORD_GFSK_CN_470_RP_1_0;
    real_const.const_min_tx_dr                    = MIN_TX_DR_CN_470_RP_1_0;
    real_const.const_max_tx_dr                    = MAX_TX_DR_CN_470_RP_1_0;
    real_const.const_min_tx_dr_limit              = MIN_TX_DR_LIMIT_CN_470_RP_1_0;
    real_const.const_number_of_tx_dr              = NUMBER_OF_TX_DR_CN_470_RP_1_0;
    real_const.const_min_rx_dr                    = MIN_RX_DR_CN_470_RP_1_0;
    real_const.const_max_rx_dr                    = MAX_RX_DR_CN_470_RP_1_0;
    real_const.const_number_rx1_dr_offset         = NUMBER_RX1_DR_OFFSET_CN_470_RP_1_0;
    real_const.const_dr_bitfield                  = DR_BITFIELD_SUPPORTED_CN_470_RP_1_0;
    real_const.const_default_tx_dr_bit_field      = DEFAULT_TX_DR_BIT_FIELD_CN_470_RP_1_0;
    real_const.const_tx_param_setup_req_supported = TX_PARAM_SETUP_REQ_SUPPORTED_CN_470_RP_1_0;
    real_const.const_new_channel_req_supported    = NEW_CHANNEL_REQ_SUPPORTED_CN_470_RP_1_0;
    real_const.const_dtc_supported                = DTC_SUPPORTED_CN_470_RP_1_0;
    real_const.const_lbt_supported                = LBT_SUPPORTED_CN_470_RP_1_0;
    real_const.const_lbt_sniff_duration_ms        = LBT_SNIFF_DURATION_MS_CN_470_RP_1_0;
    real_const.const_lbt_threshold_dbm            = LBT_THRESHOLD_DBM_CN_470_RP_1_0;
    real_const.const_lbt_bw_hz                    = LBT_BW_HZ_CN_470_RP_1_0;
    real_const.const_max_payload_m                = &M_cn_470_rp_1_0[0];
    real_const.const_coding_rate                  = RAL_LORA_CR_4_5;
    real_const.const_mobile_longrange_dr_distri   = &MOBILE_LONGRANGE_DR_DISTRIBUTION_CN_470_RP_1_0[0];
    real_const.const_mobile_lowpower_dr_distri    = &MOBILE_LOWPER_DR_DISTRIBUTION_CN_470_RP_1_0[0];
    real_const.const_join_dr_distri               = &JOIN_DR_DISTRIBUTION_CN_470_RP_1_0[0];
    real_const.const_default_dr_distri            = &DEFAULT_DR_DISTRIBUTION_CN_470_RP_1_0[0];
    real_const.const_cf_list_type_supported       = CF_LIST_SUPPORTED_CN_470_RP_1_0;
    real_const.const_beacon_dr                    = BEACON_DR_CN_470_RP_1_0;

    real_ctx.tx_frequency_channel_ctx   = NULL;
    real_ctx.rx1_frequency_channel_ctx  = NULL;
    real_ctx.channel_index_enabled_ctx  = &channel_index_enabled[0];
    real_ctx.unwrapped_channel_mask_ctx = &unwrapped_channel_mask[0];
    real_ctx.dr_bitfield_tx_channel_ctx = &dr_bitfield_tx_channel[0];
    real_ctx.dr_distribution_init_ctx   = &dr_distribution_init[0];
    real_ctx.dr_distribution_ctx        = &dr_distribution[0];

    memset1( dr_distribution_init, 0, real_const.const_number_of_tx_dr );
    memset1( dr_distribution, 0, real_const.const_number_of_tx_dr );

    // Enable all unwrapped channels
    memset1( &unwrapped_channel_mask[0], 0xFF, BANK_MAX_CN470_RP_1_0 );

    snapshot_bank_tx_mask = BANK_0_125_CN470_RP_1_0;
}

void region_cn_470_rp_1_0_config( smtc_real_t* real )
{
    // Tx 125 kHz channels
    for( uint8_t i = 0; i < real_const.const_number_of_tx_channel; i++ )
    {
        // Enable default datarate
        dr_bitfield_tx_channel[i] = real_const.const_default_tx_dr_bit_field;

        SMTC_PUT_BIT8( channel_index_enabled, i, CHANNEL_ENABLED );

        SMTC_MODEM_HAL_TRACE_PRINTF( "TX - idx:%u, freq: %d, dr: 0x%x,\n%s", i,
                                     region_cn_470_rp_1_0_get_tx_frequency_channel( real, i ),
                                     dr_bitfield_tx_channel[i], ( ( i % 8 ) == 7 ) ? "---\n" : "" );
    }
#if MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON
    // Rx 500 kHz channels
    for( uint8_t i = 0; i < real_const.const_number_of_rx_channel; i++ )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "RX - idx:%u, freq: %d, dr_min: %u, dr_max: %u\n%s", i,
                                     region_cn_470_rp_1_0_get_rx1_frequency_channel( real, i ), MIN_RX_DR_CN_470_RP_1_0,
                                     MAX_RX_DR_CN_470_RP_1_0, ( ( i % 8 ) == 7 ) ? "---\n" : "" );
    }
#endif
}

status_lorawan_t region_cn_470_rp_1_0_get_join_next_channel( smtc_real_t* real, uint8_t tx_data_rate,
                                                             uint32_t* out_tx_frequency, uint32_t* out_rx1_frequency )
{
#if defined( HYBRID_CN470_MONO_CHANNEL )
    uint8_t err           = true;
    uint8_t ch_index_mono = 0;

    for( uint8_t i = 0; i < real_const.const_number_of_tx_channel; i++ )
    {
        if( freq_tx_cn470_mono_channel_mhz == region_cn_470_rp_1_0_get_tx_frequency_channel( real, i ) )
        {
            err           = false;
            ch_index_mono = i;
            break;
        }
    }
    if( err == true )
    {
        smtc_modem_hal_lr1mac_panic( );
    }
    *out_tx_frequency  = region_cn_470_rp_1_0_get_tx_frequency_channel( real, ch_index_mono );
    *out_rx1_frequency = region_cn_470_rp_1_0_get_rx1_frequency_channel( real, ch_index_mono );

    return OKLORAWAN;
#endif

    cn_470_rp_1_0_channels_bank_t bank_tmp_cnt = 0;
    uint8_t                       active_channel_nb;
    uint8_t                       active_channel_index[NUMBER_OF_TX_CHANNEL_CN_470_RP_1_0];
    do
    {
        if( snapshot_bank_tx_mask >= BANK_MAX_CN470_RP_1_0 )
        {
            snapshot_bank_tx_mask = BANK_0_125_CN470_RP_1_0;
        }

        active_channel_nb = 0;
        for( uint8_t i = snapshot_bank_tx_mask * 8; i < ( ( snapshot_bank_tx_mask * 8 ) + 8 ); i++ )
        {
            if( ( SMTC_GET_BIT8( channel_index_enabled, i ) == CHANNEL_ENABLED ) &&
                ( SMTC_GET_BIT16( &dr_bitfield_tx_channel[i], tx_data_rate ) == 1 ) )
            {
                active_channel_index[active_channel_nb] = i;
                active_channel_nb++;
            }
        }
        snapshot_bank_tx_mask++;
        bank_tmp_cnt++;
    } while( ( active_channel_nb == 0 ) && ( bank_tmp_cnt < BANK_MAX_CN470_RP_1_0 ) );

    if( active_channel_nb == 0 )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "NO CHANNELS AVAILABLE \n" );
        return ERRORLORAWAN;
    }

    uint8_t temp        = 0xFF;
    uint8_t channel_idx = 0;

    temp        = ( smtc_modem_hal_get_random_nb_in_range( 0, ( active_channel_nb - 1 ) ) ) % active_channel_nb;
    channel_idx = active_channel_index[temp];

    if( channel_idx >= real_const.const_number_of_tx_channel )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "INVALID CHANNEL  active channel = %d and random channel = %d \n",
                                     active_channel_nb, temp );
        return ERRORLORAWAN;
    }

    *out_tx_frequency  = region_cn_470_rp_1_0_get_tx_frequency_channel( real, channel_idx );
    *out_rx1_frequency = region_cn_470_rp_1_0_get_rx1_frequency_channel( real, channel_idx );

    return OKLORAWAN;
}

status_lorawan_t region_cn_470_rp_1_0_get_next_channel( smtc_real_t* real, uint8_t tx_data_rate,
                                                        uint32_t* out_tx_frequency, uint32_t* out_rx1_frequency )
{
#if defined( HYBRID_CN470_MONO_CHANNEL )
    uint8_t err           = true;
    uint8_t ch_index_mono = 0;

    for( uint8_t i = 0; i < real_const.const_number_of_tx_channel; i++ )
    {
        if( freq_tx_cn470_mono_channel_mhz == region_cn_470_rp_1_0_get_tx_frequency_channel( real, i ) )
        {
            err           = false;
            ch_index_mono = i;
            break;
        }
    }
    if( err == true )
    {
        smtc_modem_hal_lr1mac_panic( );
    }
    *out_tx_frequency  = region_cn_470_rp_1_0_get_tx_frequency_channel( real, ch_index_mono );
    *out_rx1_frequency = region_cn_470_rp_1_0_get_rx1_frequency_channel( real, ch_index_mono );

    return OKLORAWAN;
#endif
    uint8_t active_channel_nb = 0;
    uint8_t active_channel_index[NUMBER_OF_TX_CHANNEL_CN_470_RP_1_0];

    for( uint8_t i = 0; i < real_const.const_number_of_tx_channel; i++ )
    {
        if( SMTC_GET_BIT8( channel_index_enabled, i ) == CHANNEL_ENABLED )
        {
            if( SMTC_GET_BIT16( &dr_bitfield_tx_channel[i], tx_data_rate ) == 1 )
            {
                active_channel_index[active_channel_nb] = i;
                active_channel_nb++;
            }
        }
    }

    if( active_channel_nb == 0 )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "NO CHANNELS AVAILABLE \n" );
        return ERRORLORAWAN;
    }
    uint8_t temp        = ( smtc_modem_hal_get_random_nb_in_range( 0, ( active_channel_nb - 1 ) ) ) % active_channel_nb;
    uint8_t channel_idx = 0;
    channel_idx         = active_channel_index[temp];
    if( channel_idx >= real_const.const_number_of_tx_channel )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "INVALID CHANNEL  active channel = %d and random channel = %d \n",
                                     active_channel_nb, temp );
        return ERRORLORAWAN;
    }
    else
    {
        *out_tx_frequency  = region_cn_470_rp_1_0_get_tx_frequency_channel( real, channel_idx );
        *out_rx1_frequency = region_cn_470_rp_1_0_get_rx1_frequency_channel( real, channel_idx );
    }
    return OKLORAWAN;
}

uint8_t region_cn_470_rp_1_0_get_number_of_chmask_in_cflist( smtc_real_t* real )
{
    return 6;
}

status_channel_t region_cn_470_rp_1_0_build_channel_mask( smtc_real_t* real, uint8_t channel_mask_cntl,
                                                          uint16_t channel_mask )
{
    status_channel_t status = OKCHANNEL;
    SMTC_MODEM_HAL_TRACE_PRINTF( "ChCtrl = 0x%u, ChMask = 0x%04x\n", channel_mask_cntl, channel_mask );
    switch( channel_mask_cntl )
    {
    case 0:  // Channels 0 to 15
    case 1:  // Channels 16 to 31
    case 2:  // Channels 32 to 47
    case 3:  // Channels 48 to 63
    case 4:  // Channels 64 to 79
    case 5:  // Channels 80 to 95

        memcpy1( unwrapped_channel_mask + ( channel_mask_cntl * 2 ), ( uint8_t* ) &channel_mask, 2 );

        // Check if all enabled channels has a valid frequency
        for( uint8_t i = 0; i < real_const.const_number_of_tx_channel; i++ )
        {
            if( ( SMTC_GET_BIT8( unwrapped_channel_mask, i ) == CHANNEL_ENABLED ) &&
                ( region_cn_470_rp_1_0_get_tx_frequency_channel( real, i ) == 0 ) )
            {
                status = ERROR_CHANNEL_MASK;  // this status is used only for the last multiple link adr req
                break;                        // break for loop
            }
        }
        break;
    case 6:
        // Enable all channels
        memset1( unwrapped_channel_mask, 0xFF, BANK_MAX_CN470_RP_1_0 );
        break;
    default:
        status = ERROR_CHANNEL_CNTL;
        break;
    }

#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
    SMTC_MODEM_HAL_TRACE_PRINTF( "unwrapped channel tx mask = 0x" );
    for( uint8_t i = 0; i < BANK_MAX_CN470_RP_1_0; i++ )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "%02x ", unwrapped_channel_mask[i] );
    }
    SMTC_MODEM_HAL_TRACE_PRINTF( " \n" );
#endif

    // check if all channels are disabled, return ERROR_CHANNEL_MASK
    if( SMTC_ARE_CLR_BYTE8( unwrapped_channel_mask, BANK_MAX_CN470_RP_1_0 ) == true )
    {
        status = ERROR_CHANNEL_MASK;
    }
    return ( status );
}

void region_cn_470_rp_1_0_enable_all_channels_with_valid_freq( smtc_real_t* real )
{
    for( uint8_t i = 0; i < real_const.const_number_of_tx_channel; i++ )
    {
        SMTC_PUT_BIT8( channel_index_enabled, i, CHANNEL_ENABLED );
        dr_bitfield_tx_channel[i] = DEFAULT_TX_DR_BIT_FIELD_CN_470_RP_1_0;
    }
}

modulation_type_t region_cn_470_rp_1_0_get_modulation_type_from_datarate( uint8_t datarate )
{
    if( datarate <= 5 )
    {
        return LORA;
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( );
    }
    return LORA;  // never reach
}

void region_cn_470_rp_1_0_lora_dr_to_sf_bw( uint8_t in_dr, uint8_t* out_sf, lr1mac_bandwidth_t* out_bw )
{
    if( in_dr <= 5 )
    {
        *out_sf = datarates_to_sf_cn_470_rp_1_0[in_dr];
        *out_bw = datarates_to_bandwidths_cn_470_rp_1_0[in_dr];
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( );
    }
}

uint32_t region_cn_470_rp_1_0_get_tx_frequency_channel( smtc_real_t* real, uint8_t index )
{
    return ( DEFAULT_TX_FREQ_CN_470_RP_1_0 + ( index * DEFAULT_TX_STEP_CN_470_RP_1_0 ) );
}

uint32_t region_cn_470_rp_1_0_get_rx1_frequency_channel( smtc_real_t* real, uint8_t index )
{
    return ( DEFAULT_RX_FREQ_CN_470_RP_1_0 +
             ( ( index % NUMBER_OF_RX_CHANNEL_CN_470_RP_1_0 ) * DEFAULT_RX_STEP_CN_470_RP_1_0 ) );
}

uint32_t region_cn_470_rp_1_0_get_rx_beacon_frequency_channel( smtc_real_t* real, uint32_t gps_time_s )
{
    uint8_t index = ( uint32_t )( floorf( gps_time_s / 128 ) ) % 8;
    return ( BEACON_FREQ_START_CN_470_RP_1_0 + ( ( index % 8 ) * BEACON_STEP_CN_470_RP_1_0 ) );
}

uint32_t region_cn_470_rp_1_0_get_rx_ping_slot_frequency_channel( smtc_real_t* real, uint32_t gps_time_s,
                                                                  uint32_t dev_addr )
{
    uint8_t index = ( dev_addr + ( uint32_t )( floorf( gps_time_s / 128 ) ) ) % 8;
    return ( PING_SLOT_FREQ_START_CN_470_RP_1_0 + ( ( index % 8 ) * PING_SLOT_STEP_CN_470_RP_1_0 ) );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
