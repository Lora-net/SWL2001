/**
 * \file      region_ru_864.c
 *
 * \brief     region_ru_864 abstraction layer implementation
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
#include "smtc_duty_cycle.h"
#include "region_ru_864_defs.h"
#include "region_ru_864.h"
#include "smtc_modem_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define real_ctx lr1_mac->real->real_ctx

#define tx_frequency_channel lr1_mac->real->region.ru864.tx_frequency_channel
#define rx1_frequency_channel lr1_mac->real->region.ru864.rx1_frequency_channel
#define dr_bitfield_tx_channel lr1_mac->real->region.ru864.dr_bitfield_tx_channel
#define channel_index_enabled lr1_mac->real->region.ru864.channel_index_enabled
#define dr_distribution_init lr1_mac->real->region.ru864.dr_distribution_init
#define dr_distribution lr1_mac->real->region.ru864.dr_distribution
#define unwrapped_channel_mask lr1_mac->real->region.ru864.unwrapped_channel_mask

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

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

void region_ru_864_config( lr1_stack_mac_t* lr1_mac )
{
    const_number_of_tx_channel         = NUMBER_OF_CHANNEL_RU_864;
    const_number_of_rx_channel         = NUMBER_OF_CHANNEL_RU_864;
    const_number_of_boot_tx_channel    = NUMBER_OF_BOOT_TX_CHANNEL_RU_864;
    const_number_of_channel_bank       = BANK_MAX_RU864;
    const_join_accept_delay1           = JOIN_ACCEPT_DELAY1_RU_864;
    const_received_delay1              = RECEIVE_DELAY1_RU_864;
    const_tx_power_dbm                 = TX_POWER_EIRP_RU_864 - 2;  // EIRP to ERP
    const_max_tx_power_idx             = MAX_TX_POWER_IDX_RU_864;
    const_adr_ack_limit                = ADR_ACK_LIMIT_RU_864;
    const_adr_ack_delay                = ADR_ACK_DELAY_RU_864;
    const_datarate_backoff             = &datarate_backoff_ru_864[0];
    const_ack_timeout                  = ACK_TIMEOUT_RU_864;
    const_freq_min                     = FREQMIN_RU_864;
    const_freq_max                     = FREQMAX_RU_864;
    const_rx2_freq                     = RX2_FREQ_RU_864;
    const_frequency_factor             = FREQUENCY_FACTOR_RU_864;
    const_rx2_dr_init                  = RX2DR_INIT_RU_864;
    const_sync_word_private            = SYNC_WORD_PRIVATE_RU_864;
    const_sync_word_public             = SYNC_WORD_PUBLIC_RU_864;
    const_sync_word_gfsk               = ( uint8_t* ) SYNC_WORD_GFSK_RU_864;
    const_min_tx_dr                    = MIN_DR_RU_864;
    const_max_tx_dr                    = MAX_DR_RU_864;
    const_min_tx_dr_limit              = MIN_TX_DR_LIMIT_RU_864;
    const_number_of_tx_dr              = NUMBER_OF_TX_DR_RU_864;
    const_min_rx_dr                    = MIN_DR_RU_864;
    const_max_rx_dr                    = MAX_DR_RU_864;
    const_number_rx1_dr_offset         = NUMBER_RX1_DR_OFFSET_RU_864;
    const_dr_bitfield                  = DR_BITFIELD_SUPPORTED_RU_864;
    const_default_tx_dr_bit_field      = DEFAULT_TX_DR_BIT_FIELD_RU_864;
    const_tx_param_setup_req_supported = TX_PARAM_SETUP_REQ_SUPPORTED_RU_864;
    const_new_channel_req_supported    = NEW_CHANNEL_REQ_SUPPORTED_RU_864;
    const_dtc_supported                = DTC_SUPPORTED_RU_864;
    const_dtc_number_of_band           = BAND_RU864_MAX;
    const_dtc_by_band                  = &duty_cycle_by_band_ru_864[0];
    const_lbt_supported                = LBT_SUPPORTED_RU_864;
    const_lbt_sniff_duration_ms        = LBT_SNIFF_DURATION_MS_RU_864;
    const_lbt_threshold_dbm            = LBT_THRESHOLD_DBM_RU_864;
    const_lbt_bw_hz                    = LBT_BW_HZ_RU_864;
    const_max_payload_m                = &M_ru_864[0];
    const_coding_rate                  = RAL_LORA_CR_4_5;
    const_mobile_longrange_dr_distri   = &MOBILE_LONGRANGE_DR_DISTRIBUTION_RU_864[0];
    const_mobile_lowpower_dr_distri    = &MOBILE_LOWPER_DR_DISTRIBUTION_RU_864[0];
    const_join_dr_distri               = &JOIN_DR_DISTRIBUTION_RU_864[0];
    const_default_dr_distri            = &DEFAULT_DR_DISTRIBUTION_RU_864[0];
    const_cf_list_type_supported       = CF_LIST_SUPPORTED_RU_864;
    const_beacon_dr                    = BEACON_DR_RU_864;
    const_beacon_frequency             = BEACON_FREQ_RU_864;
    const_ping_slot_frequency          = PING_SLOT_FREQ_RU_864;

    real_ctx.tx_frequency_channel_ctx   = &tx_frequency_channel[0];
    real_ctx.rx1_frequency_channel_ctx  = &rx1_frequency_channel[0];
    real_ctx.channel_index_enabled_ctx  = &channel_index_enabled[0];
    real_ctx.unwrapped_channel_mask_ctx = &unwrapped_channel_mask[0];
    real_ctx.dr_bitfield_tx_channel_ctx = &dr_bitfield_tx_channel[0];
    real_ctx.dr_distribution_init_ctx   = &dr_distribution_init[0];
    real_ctx.dr_distribution_ctx        = &dr_distribution[0];

    memset1( dr_distribution_init, 1, const_number_of_tx_dr );
    memset1( dr_distribution, 0, const_number_of_tx_dr );

    // Configure duty-cycle object
    for( int i = 0; i < const_dtc_number_of_band; i++ )
    {
        smtc_duty_cycle_config( lr1_mac->dtc_obj, const_dtc_number_of_band, i, duty_cycle_by_band_ru_864[i],
                                frequency_range_by_band_ru_864[i][0], frequency_range_by_band_ru_864[i][1] );
    }
}

void region_ru_864_init( lr1_stack_mac_t* lr1_mac )
{
    for( int i = 0; i < const_number_of_tx_channel; i++ )
    {
        tx_frequency_channel[i]  = 0;
        rx1_frequency_channel[i] = 0;
        SMTC_PUT_BIT8( channel_index_enabled, i, CHANNEL_DISABLED );

        // Enable default datarate for all channels
        dr_bitfield_tx_channel[i] = const_default_tx_dr_bit_field;
    }

    // Set Tx/Rx default Freq and enable channels
    for( uint8_t i = 0; i < const_number_of_boot_tx_channel; i++ )
    {
        tx_frequency_channel[i]  = default_freq_ru_864[i];
        rx1_frequency_channel[i] = default_freq_ru_864[i];
        SMTC_PUT_BIT8( channel_index_enabled, i, CHANNEL_ENABLED );
    }

    // Enable all unwrapped channels
    memset1( &unwrapped_channel_mask[0], 0xFF, BANK_MAX_RU864 );
}

status_lorawan_t region_ru_864_get_join_next_channel( lr1_stack_mac_t* lr1_mac )
{
    return region_ru_864_get_next_channel( lr1_mac );
}

status_lorawan_t region_ru_864_get_next_channel( lr1_stack_mac_t* lr1_mac )
{
    uint8_t active_channel_nb = 0;
    uint8_t active_channel_index[NUMBER_OF_CHANNEL_RU_864];

    for( uint8_t i = 0; i < const_number_of_tx_channel; i++ )
    {
        if( SMTC_GET_BIT8( channel_index_enabled, i ) == CHANNEL_ENABLED )
        {
            if( ( smtc_duty_cycle_is_channel_free( lr1_mac->dtc_obj, tx_frequency_channel[i] ) == true ) &&
                ( SMTC_GET_BIT16( &dr_bitfield_tx_channel[i], lr1_mac->tx_data_rate ) == 1 ) )
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
    if( channel_idx >= const_number_of_tx_channel )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "INVALID CHANNEL  active channel = %d and random channel = %d \n",
                                     active_channel_nb, temp );
        return ERRORLORAWAN;
    }
    else
    {
        lr1_mac->tx_frequency  = tx_frequency_channel[channel_idx];
        lr1_mac->rx1_frequency = rx1_frequency_channel[channel_idx];
    }
    return OKLORAWAN;
}

void region_ru_864_set_rx_config( lr1_stack_mac_t* lr1_mac, rx_win_type_t type )
{
    if( type == RX1 )
    {
        lr1_mac->rx_data_rate = datarate_offsets_ru_864[lr1_mac->tx_data_rate][lr1_mac->rx1_dr_offset];
    }
    else if( type == RX2 )
    {
        lr1_mac->rx_data_rate = lr1_mac->rx2_data_rate;
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "INVALID RX TYPE \n" );
    }
}
void region_ru_864_set_channel_mask( lr1_stack_mac_t* lr1_mac )
{
    // Copy all unwrapped channels in channel enable
    memcpy1( channel_index_enabled, unwrapped_channel_mask, BANK_MAX_RU864 );

    for( uint8_t i = 0; i < const_number_of_tx_channel; i++ )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " %d ", SMTC_GET_BIT8( channel_index_enabled, i ) );
    }
    SMTC_MODEM_HAL_TRACE_MSG( " \n" );
}

status_channel_t region_ru_864_build_channel_mask( lr1_stack_mac_t* lr1_mac, uint8_t channel_mask_cntl,
                                                   uint16_t channel_mask )
{
    status_channel_t status = OKCHANNEL;
    switch( channel_mask_cntl )
    {
    case 0:
        memcpy1( unwrapped_channel_mask + ( channel_mask_cntl * 2 ), ( uint8_t* ) &channel_mask, 2 );

        // Check if all enabled channels has a valid frequency
        for( uint8_t i = 0; i < const_number_of_tx_channel; i++ )
        {
            if( ( SMTC_GET_BIT8( unwrapped_channel_mask, i ) == CHANNEL_ENABLED ) && ( tx_frequency_channel[i] == 0 ) )
            {
                status = ERROR_CHANNEL_MASK;  // this status is used only for the last multiple link adr req
                break;                        // break for loop
            }
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "UnwrappedChannelMask = 0x" );
        for( uint8_t i = 0; i < BANK_MAX_RU864; i++ )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "%x ", unwrapped_channel_mask[i] );
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( ", ChMask = 0x%x\n", channel_mask );
        break;
    case 6:
        memset1( unwrapped_channel_mask, 0x00, BANK_MAX_RU864 );
        for( uint8_t i = 0; i < const_number_of_tx_channel; i++ )
        {
            if( tx_frequency_channel[i] > 0 )
            {
                SMTC_PUT_BIT8( unwrapped_channel_mask, i, CHANNEL_ENABLED );
            }
        }
        break;
    default:
        status = ERROR_CHANNEL_CNTL;
        break;
    }

    // check if all channels are disabled, return ERROR_CHANNEL_MASK
    if( SMTC_ARE_CLR_BYTE8( unwrapped_channel_mask, BANK_MAX_RU864 ) == true )
    {
        status = ERROR_CHANNEL_MASK;
    }

    return ( status );
}

modulation_type_t region_ru_864_get_modulation_type_from_datarate( uint8_t datarate )
{
    if( datarate <= 6 )
    {
        return LORA;
    }
    else if( datarate == 7 )
    {
        return FSK;
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( );
    }
    return LORA;  // never reach
}

void region_ru_864_lora_dr_to_sf_bw( uint8_t in_dr, uint8_t* out_sf, lr1mac_bandwidth_t* out_bw )
{
    if( in_dr <= 6 )
    {
        *out_sf = datarates_to_sf_ru_864[in_dr];
        *out_bw = datarates_to_bandwidths_ru_864[in_dr];
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( );
    }
}

void region_ru_864_fsk_dr_to_bitrate( uint8_t in_dr, uint8_t* out_bitrate )
{
    if( in_dr == 7 )
    {
        *out_bitrate = 50;  // Kbit
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
