/**
 * \file      region_us_915.c
 *
 * \brief     region_us_915 abstraction layer implementation
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

#include <string.h>  // memcpy
#include "lr1mac_utilities.h"
#include "smtc_modem_hal.h"
#include "region_us_915_defs.h"
#include "region_us_915.h"
#include "smtc_modem_hal_dbg_trace.h"

typedef enum ch_mask_after_join_e
{
    ch_mask_after_join_init = 0,
    ch_mask_after_join_8ch,   // just after join if no ChMask received
    ch_mask_after_join_56ch,  // just after join + first 8ch if no ChMask received
    ch_mask_after_join_full   // nominal way, ChMask received or not
} ch_mask_after_join_t;

#define real_ctx lr1_mac->real.real_ctx

#define dr_bitfield_tx_channel lr1_mac->real.region.us915.dr_bitfield_tx_channel
#define channel_index_enabled lr1_mac->real.region.us915.channel_index_enabled
#define dr_distribution_init lr1_mac->real.region.us915.dr_distribution_init
#define dr_distribution lr1_mac->real.region.us915.dr_distribution
#define unwrapped_channel_mask lr1_mac->real.region.us915.unwrapped_channel_mask
#define first_ch_mask_received lr1_mac->real.region.us915.first_ch_mask_received

#define snapshot_channel_tx_mask lr1_mac->real.region.us915.snapshot_channel_tx_mask
#define snapshot_bank_tx_mask lr1_mac->real.region.us915.snapshot_bank_tx_mask

// Private region_us_915 utilities declaration
//

/**
 * @brief init Channel mask after the join accept
 *
 * @param lr1_mac
 */
static void region_us_915_channel_mask_set_after_join( lr1_stack_mac_t* lr1_mac );

void region_us_915_config( lr1_stack_mac_t* lr1_mac )
{
    const_number_of_tx_channel         = NUMBER_OF_TX_CHANNEL_US_915;
    const_number_of_rx_channel         = NUMBER_OF_RX_CHANNEL_US_915;
    const_number_of_channel_bank       = BANK_MAX_US915;
    const_join_accept_delay1           = JOIN_ACCEPT_DELAY1_US_915;
    const_received_delay1              = RECEIVE_DELAY1_US_915;
    const_tx_power_dbm                 = TX_POWER_EIRP_US_915;
    const_max_tx_power_idx             = MAX_TX_POWER_IDX_US_915;
    const_adr_ack_limit                = ADR_ACK_LIMIT_US_915;
    const_adr_ack_delay                = ADR_ACK_DELAY_US_915;
    const_ack_timeout                  = ACK_TIMEOUT_US_915;
    const_freq_min                     = FREQMIN_US_915;
    const_freq_max                     = FREQMAX_US_915;
    const_rx2_freq                     = RX2_FREQ_US_915;
    const_frequency_factor             = FREQUENCY_FACTOR_US_915;
    const_rx2_dr_init                  = RX2DR_INIT_US_915;
    const_sync_word_private            = SYNC_WORD_PRIVATE_US_915;
    const_sync_word_public             = SYNC_WORD_PUBLIC_US_915;
    const_min_tx_dr                    = MIN_TX_DR_US_915;
    const_max_tx_dr                    = MAX_TX_DR_US_915;
    const_min_tx_dr_limit              = MIN_TX_DR_LIMIT_US_915;
    const_max_tx_default_dr            = MAX_TX_DEFAULT_DR_US915;
    const_number_of_tx_dr              = NUMBER_OF_TX_DR_US_915;
    const_min_rx_dr                    = MIN_RX_DR_US_915;
    const_max_rx_dr                    = MAX_RX_DR_US_915;
    const_max_rx1_dr_offset            = MAX_RX1_DR_OFSSET_US_915;
    const_dr_bitfield                  = DR_BITFIELD_SUPPORTED_US_915;
    const_tx_param_setup_req_supported = TX_PARAM_SETUP_REQ_SUPPORTED_US_915;
    const_new_channel_req_supported    = NEW_CHANNEL_REQ_SUPPORTED_US_915;
    const_dtc_supported                = DTC_SUPPORTED_US_915;
    const_lbt_supported                = LBT_SUPPORTED_US_915;
    const_max_payload_m                = &M_us_915[0];
    const_max_payload_n                = &N_us_915[0];
    const_coding_rate                  = RAL_LORA_CR_4_5;
    const_mobile_longrange_dr_distri   = &MOBILE_LONGRANGE_DR_DISTRIBUTION_US_915[0];
    const_mobile_lowpower_dr_distri    = &MOBILE_LOWPER_DR_DISTRIBUTION_US_915[0];
    const_join_dr_distri               = &JOIN_DR_DISTRIBUTION_US_915[0];
    const_default_dr_distri            = &DEFAULT_DR_DISTRIBUTION_US_915[0];
    const_cf_list_type_supported       = CF_LIST_SUPPORTED_US_915;

    real_ctx.tx_frequency_channel_ctx   = NULL;
    real_ctx.rx1_frequency_channel_ctx  = NULL;
    real_ctx.channel_index_enabled_ctx  = &channel_index_enabled[0];
    real_ctx.unwrapped_channel_mask_ctx = &unwrapped_channel_mask[0];
    real_ctx.dr_bitfield_tx_channel_ctx = &dr_bitfield_tx_channel[0];
    real_ctx.dr_distribution_init_ctx   = &dr_distribution_init[0];
    real_ctx.dr_distribution_ctx        = &dr_distribution[0];

    memset1( dr_distribution_init, 0, const_number_of_tx_dr );
    memset1( dr_distribution, 0, const_number_of_tx_dr );

    // Enable all channels
    memset1( &unwrapped_channel_mask[0], 0xFF, BANK_MAX_US915 );
    memset1( &snapshot_channel_tx_mask[0], 0xFF, BANK_MAX_US915 );

    snapshot_bank_tx_mask = 0;
}

void region_us_915_init( lr1_stack_mac_t* lr1_mac )
{
    // Tx 125 kHz channels
    for( uint8_t i = 0; i < NUMBER_OF_TX_CHANNEL_US_915 - 8; i++ )
    {
        SMTC_PUT_BIT8( channel_index_enabled, i, CHANNEL_ENABLED );

        // Enable default datarate
        dr_bitfield_tx_channel[i] = DEFAULT_TX_DR_125_BIT_FIELD_US_915;

        SMTC_MODEM_HAL_TRACE_PRINTF( "TX - idx:%u, freq: %d, dr: 0x%x,\n%s", i,
                                     region_us_915_get_tx_frequency_channel( lr1_mac, i ), dr_bitfield_tx_channel[i],
                                     ( ( i % 8 ) == 7 ) ? "---\n" : "" );
    }
    // Tx 500 kHz channels
    for( uint8_t i = NUMBER_OF_TX_CHANNEL_US_915 - 8; i < NUMBER_OF_TX_CHANNEL_US_915; i++ )
    {
        SMTC_PUT_BIT8( channel_index_enabled, i, CHANNEL_ENABLED );
        // Enable default datarate
        dr_bitfield_tx_channel[i] = DEFAULT_TX_DR_500_BIT_FIELD_US_915;

        SMTC_MODEM_HAL_TRACE_PRINTF( "TX - idx:%u, freq: %d, dr: 0x%x,\n%s", i,
                                     region_us_915_get_tx_frequency_channel( lr1_mac, i ), dr_bitfield_tx_channel[i],
                                     ( ( i % 8 ) == 7 ) ? "---\n" : "" );
    }
#if MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON
    // Rx 500 kHz channels
    for( uint8_t i = 0; i < NUMBER_OF_RX_CHANNEL_US_915; i++ )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "RX - idx:%u, freq: %d, dr_min: %u, dr_max: %u\n%s", i,
                                     region_us_915_get_rx1_frequency_channel( lr1_mac, i ), MIN_RX_DR_US_915,
                                     MAX_RX_DR_US_915, ( ( i % 8 ) == 7 ) ? "---\n" : "" );
    }
#endif

    first_ch_mask_received = ch_mask_after_join_init;
}

status_lorawan_t region_us_915_is_acceptable_tx_dr( lr1_stack_mac_t* lr1_mac, uint8_t dr,
                                                    bool is_ch_mask_from_link_adr )
{
    status_lorawan_t status                      = ERRORLORAWAN;
    uint8_t          number_channels_125_enabled = 0;

    uint8_t* ch_mask_to_check = ( is_ch_mask_from_link_adr == true ) ? unwrapped_channel_mask : channel_index_enabled;

    // 125 kHz channels
    for( uint8_t i = 0; i < const_number_of_tx_channel - 8; i++ )
    {
        if( SMTC_GET_BIT8( ch_mask_to_check, i ) == CHANNEL_ENABLED )
        {
            if( SMTC_GET_BIT16( &dr_bitfield_tx_channel[i], dr ) == 1 )
            {
                status = OKLORAWAN;
                number_channels_125_enabled++;
            }
        }
    }
    // 500 kHz channels
    for( uint8_t i = NUMBER_OF_TX_CHANNEL_US_915 - 8; i < NUMBER_OF_TX_CHANNEL_US_915; i++ )
    {
        if( SMTC_GET_BIT8( ch_mask_to_check, i ) == CHANNEL_ENABLED )
        {
            if( SMTC_GET_BIT16( &dr_bitfield_tx_channel[i], dr ) == 1 )
            {
                status = OKLORAWAN;
                break;
            }
        }
    }

    if( dr < MAX_TX_DR_US_915 )
    {
        // FCC 15.247 paragraph F mandates to hop on at least 2 125 kHz channels
        if( number_channels_125_enabled < 2 )
        {
            status = ERRORLORAWAN;
        }
    }

    if( status == ERRORLORAWAN )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Not acceptable data rate\n" );
    }
    return ( status );
}

status_lorawan_t region_us_915_get_join_next_channel( lr1_stack_mac_t* lr1_mac )
{
    if( snapshot_bank_tx_mask > BANK_8_500_US915 )
    {
        snapshot_bank_tx_mask = BANK_0_125_US915;
    }
    // if all 125kHz channels were used, reset the snapshots
    if( SMTC_ARE_CLR_BYTE8( snapshot_channel_tx_mask, BANK_8_500_US915 ) == true )
    {
        for( us_915_channels_bank_t i = 0; i < BANK_8_500_US915; i++ )
        {
            snapshot_channel_tx_mask[i] = channel_index_enabled[i];
        }
    }
    // if all 500kHz channels were used, reset the snapshots
    if( snapshot_channel_tx_mask[BANK_8_500_US915] == 0 )
    {
        snapshot_channel_tx_mask[BANK_8_500_US915] = channel_index_enabled[BANK_8_500_US915];
    }

    uint8_t active_channel_nb;
    uint8_t active_channel_index[NUMBER_OF_TX_CHANNEL_US_915];
    do
    {
        active_channel_nb = 0;
        for( uint8_t i = snapshot_bank_tx_mask * 8; i < ( ( snapshot_bank_tx_mask * 8 ) + 8 ); i++ )
        {
            if( snapshot_bank_tx_mask == BANK_8_500_US915 )
            {
                if( ( SMTC_GET_BIT8( snapshot_channel_tx_mask, i ) == CHANNEL_ENABLED ) &&
                    ( SMTC_GET_BIT8( channel_index_enabled, i ) == CHANNEL_ENABLED ) )
                {
                    active_channel_index[active_channel_nb] = i;
                    active_channel_nb++;
                }
            }
            else
            {
                if( ( SMTC_GET_BIT8( snapshot_channel_tx_mask, i ) == CHANNEL_ENABLED ) &&
                    ( SMTC_GET_BIT8( channel_index_enabled, i ) == CHANNEL_ENABLED ) )
                {
                    active_channel_index[active_channel_nb] = i;
                    active_channel_nb++;
                }
            }
        }
        snapshot_bank_tx_mask++;
    } while( ( active_channel_nb == 0 ) && ( snapshot_bank_tx_mask <= BANK_8_500_US915 ) );

    if( active_channel_nb == 0 )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "NO CHANNELS AVAILABLE \n" );
        return ERRORLORAWAN;
    }

    uint8_t temp        = 0xFF;
    uint8_t channel_idx = 0;
    if( snapshot_bank_tx_mask > BANK_8_500_US915 )
    {
        // active_channel_index[0] contains the first available 500KHz channel
        channel_idx = active_channel_index[0];
    }
    else
    {
        temp        = ( smtc_modem_hal_get_random_nb_in_range( 0, ( active_channel_nb - 1 ) ) ) % active_channel_nb;
        channel_idx = active_channel_index[temp];
    }

    if( channel_idx >= NUMBER_OF_TX_CHANNEL_US_915 )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "INVALID CHANNEL  active channel = %d and random channel = %d \n",
                                     active_channel_nb, temp );
        return ERRORLORAWAN;
    }

    // Mask the channel used, to be remove for the next selection
    SMTC_PUT_BIT8( snapshot_channel_tx_mask, channel_idx, CHANNEL_DISABLED );
    if( snapshot_bank_tx_mask > BANK_8_500_US915 )
    {
        lr1_mac->tx_data_rate = MAX_TX_DR_US_915;
    }
    else
    {
        lr1_mac->tx_data_rate = MIN_TX_DR_US_915;
    }

    lr1_mac->tx_frequency  = region_us_915_get_tx_frequency_channel( lr1_mac, channel_idx );
    lr1_mac->rx1_frequency = region_us_915_get_rx1_frequency_channel( lr1_mac, channel_idx );

#if MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON
    SMTC_MODEM_HAL_TRACE_PRINTF( "snapshot channel 125 tx mask\n" );
    for( uint8_t i = 0; i < NUMBER_OF_TX_CHANNEL_US_915 - 8; i++ )
    {
        uint8_t test = SMTC_GET_BIT8( snapshot_channel_tx_mask, i );
        SMTC_MODEM_HAL_TRACE_PRINTF( "%u%s", test, ( ( i % 8 ) == 7 ) ? " \n" : "" );
    }
    SMTC_MODEM_HAL_TRACE_PRINTF( "snapshot channel 500 tx mask\n" );
    for( uint8_t i = 0; i < 8; i++ )
    {
        uint8_t test = SMTC_GET_BIT8( &snapshot_channel_tx_mask[BANK_8_500_US915], i );
        SMTC_MODEM_HAL_TRACE_PRINTF( "%u%s", test, ( ( i % 8 ) == 7 ) ? " \n" : "" );
    }
#endif

    return OKLORAWAN;
}

status_lorawan_t region_us_915_get_next_channel( lr1_stack_mac_t* lr1_mac )
{
    // if all channels were used, reset the snapshots
    if( ( SMTC_ARE_CLR_BYTE8( snapshot_channel_tx_mask, BANK_8_500_US915 ) == true ) &&
        ( first_ch_mask_received == ch_mask_after_join_full ) )
    {
        for( us_915_channels_bank_t i = 0; i < BANK_8_500_US915; i++ )
        {
            snapshot_channel_tx_mask[i] = channel_index_enabled[i];
        }
    }

    if( ( snapshot_channel_tx_mask[BANK_8_500_US915] == 0 ) && ( first_ch_mask_received == ch_mask_after_join_full ) )
    {
        snapshot_channel_tx_mask[BANK_8_500_US915] = channel_index_enabled[BANK_8_500_US915];
    }

    if( ( ( SMTC_ARE_CLR_BYTE8( snapshot_channel_tx_mask, BANK_8_500_US915 ) == true ) ||
          ( snapshot_channel_tx_mask[BANK_8_500_US915] == 0 ) ) &&
        ( first_ch_mask_received == ch_mask_after_join_56ch ) )
    {
        memset1( unwrapped_channel_mask, 0xFF, const_number_of_channel_bank );
        region_us_915_set_channel_mask( lr1_mac );
    }

    if( ( ( SMTC_ARE_CLR_BYTE8( snapshot_channel_tx_mask, BANK_8_500_US915 ) == true ) ||
          ( snapshot_channel_tx_mask[BANK_8_500_US915] == 0 ) ) &&
        ( first_ch_mask_received <= ch_mask_after_join_8ch ) )
    {
        region_us_915_init_after_join_snapshot_channel_mask( lr1_mac );
    }

    uint8_t active_channel_nb = 0;
    uint8_t active_channel_index[NUMBER_OF_TX_CHANNEL_US_915];
    for( uint8_t i = 0; i < NUMBER_OF_TX_CHANNEL_US_915; i++ )
    {
        if( ( SMTC_GET_BIT8( snapshot_channel_tx_mask, i ) == CHANNEL_ENABLED ) &&
            ( SMTC_GET_BIT8( channel_index_enabled, i ) == CHANNEL_ENABLED ) &&
            ( SMTC_GET_BIT16( &dr_bitfield_tx_channel[i], lr1_mac->tx_data_rate ) == 1 ) )
        {
            active_channel_index[active_channel_nb] = i;
            active_channel_nb++;
        }
    }
    if( active_channel_nb == 0 )
    {
        smtc_modem_hal_lr1mac_panic( "NO CHANNELS AVAILABLE\n" );
    }

    uint8_t temp        = ( smtc_modem_hal_get_random_nb_in_range( 0, ( active_channel_nb - 1 ) ) ) % active_channel_nb;
    uint8_t channel_idx = active_channel_index[temp];
    if( channel_idx >= NUMBER_OF_TX_CHANNEL_US_915 )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "INVALID CHANNEL  active channel = %d and random channel = %d \n",
                                     active_channel_nb, temp );
        return ERRORLORAWAN;
    }

    // Mask the channel used, to be remove for the next selection
    SMTC_PUT_BIT8( snapshot_channel_tx_mask, channel_idx, CHANNEL_DISABLED );

    lr1_mac->tx_frequency  = region_us_915_get_tx_frequency_channel( lr1_mac, channel_idx );
    lr1_mac->rx1_frequency = region_us_915_get_rx1_frequency_channel( lr1_mac, channel_idx );

#if MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON
    SMTC_MODEM_HAL_TRACE_PRINTF( "snapshot channel 125 tx mask\n" );
    for( uint8_t i = 0; i < NUMBER_OF_TX_CHANNEL_US_915 - 8; i++ )
    {
        uint8_t test = SMTC_GET_BIT8( snapshot_channel_tx_mask, i );
        SMTC_MODEM_HAL_TRACE_PRINTF( "%u%s", test, ( ( i % 8 ) == 7 ) ? " \n" : "" );
    }
    SMTC_MODEM_HAL_TRACE_PRINTF( "snapshot channel 500 tx mask\n" );
    for( uint8_t i = 0; i < 8; i++ )
    {
        uint8_t test = SMTC_GET_BIT8( &snapshot_channel_tx_mask[BANK_8_500_US915], i );
        SMTC_MODEM_HAL_TRACE_PRINTF( "%u%s", test, ( ( i % 8 ) == 7 ) ? " \n" : "" );
    }
#endif

    return OKLORAWAN;
}

void region_us_915_set_rx_config( lr1_stack_mac_t* lr1_mac, rx_win_type_t type )
{
    if( type == RX1 )
    {
        lr1_mac->rx_data_rate = datarate_offsets_us_915[lr1_mac->tx_data_rate][lr1_mac->rx1_dr_offset];
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
void region_us_915_set_channel_mask( lr1_stack_mac_t* lr1_mac )
{
    region_us_915_channel_mask_set_after_join( lr1_mac );

    first_ch_mask_received = ch_mask_after_join_full;
}

void region_us_915_init_join_snapshot_channel_mask( lr1_stack_mac_t* lr1_mac )
{
    memset1( snapshot_channel_tx_mask, 0xFF, BANK_MAX_US915 );
    snapshot_bank_tx_mask = 0;
}

void region_us_915_init_after_join_snapshot_channel_mask( lr1_stack_mac_t* lr1_mac )
{
    us_915_channels_bank_t ch_mask_block = 0;

    for( us_915_channels_bank_t i = 0; i < BANK_MAX_US915; i++ )
    {
        unwrapped_channel_mask[i] = 0x00;
    }

    uint8_t            tx_sf;
    lr1mac_bandwidth_t tx_bw;
    region_us_915_lora_dr_to_sf_bw( lr1_mac->tx_data_rate, &tx_sf, &tx_bw );

    if( tx_bw == BW125 )
    {
        ch_mask_block =
            ( us_915_channels_bank_t )( ( lr1_mac->tx_frequency - 902300000 ) /
                                        ( 1600000 ) );  // 1600000 = 8 ch * 200000 MHz, the gap in each block
    }
    else if( tx_bw == BW500 )
    {
        ch_mask_block = ( us_915_channels_bank_t )( ( ( lr1_mac->tx_frequency - 903000000 ) / 1600000 ) % 8 );
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( "invalid BW %d", tx_bw );
    }

    // Block are defined from 0 to 8
    if( ch_mask_block >= BANK_MAX_US915 )
    {
        smtc_modem_hal_lr1mac_panic( "frequency block out of range %d\n", ch_mask_block );
    }

    if( first_ch_mask_received == ch_mask_after_join_init )
    {
        // 125 kHz channels, init the right block only
        unwrapped_channel_mask[ch_mask_block] = 0xFF;

        // 500 kHz channels, init the corresponding 500kHz frequency to this block
        SMTC_PUT_BIT8( &unwrapped_channel_mask[BANK_8_500_US915], ch_mask_block, CHANNEL_ENABLED );
    }
    else if( first_ch_mask_received == ch_mask_after_join_8ch )
    {
        // 125 kHz channels, init all blocks, except the previously set
        for( us_915_channels_bank_t i = 0; i < BANK_8_500_US915; i++ )
        {
            unwrapped_channel_mask[i] = 0xFF;
        }
        unwrapped_channel_mask[ch_mask_block] = 0x00;

        // 500 kHz channels, init all 500kHz channels, except the previously set
        unwrapped_channel_mask[BANK_8_500_US915] = ( 0xFF & ~( 1 << ch_mask_block ) );
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( "bad sate\n" );
    }

    // Apply computed channel mask after join
    region_us_915_channel_mask_set_after_join( lr1_mac );
}

status_channel_t region_us_915_build_channel_mask( lr1_stack_mac_t* lr1_mac, uint8_t channel_mask_cntl,
                                                   uint16_t channel_mask )
{
    status_channel_t status = OKCHANNEL;
    SMTC_MODEM_HAL_TRACE_PRINTF( "ChCtrl = 0x%u, ChMask = 0x%04x\n", channel_mask_cntl, channel_mask );
    switch( channel_mask_cntl )
    {
    // 125 KHz channels
    case 0:
    case 1:
    case 2:
    case 3:
        memcpy1( unwrapped_channel_mask + ( channel_mask_cntl * 2 ), ( uint8_t* ) &channel_mask, 2 );
        break;
    // 500 KHz channels
    case 4:
        memcpy1( &unwrapped_channel_mask[BANK_8_500_US915], ( uint8_t* ) &channel_mask, 1 );
        break;
    // bank of channels
    case 5:
        // Run over each bank
        for( uint8_t i = 0; i < BANK_8_500_US915; i++ )
        {
            if( ( ( channel_mask >> i ) & 0x01 ) == CHANNEL_ENABLED )
            {
                unwrapped_channel_mask[i] = 0xFF;
            }
            else
            {
                unwrapped_channel_mask[i] = 0x00;
            }
        }

        break;
    // All 125 kHz ON ChMask applies to channels 64 to 71
    case 6:
        // Enable all 125KHz channels
        memset1( unwrapped_channel_mask, 0xFF, BANK_8_500_US915 );

        // Enable 500KHz channels
        memcpy1( &unwrapped_channel_mask[BANK_8_500_US915], ( uint8_t* ) &channel_mask, 1 );
        break;
    // All 125 kHz OFF ChMask applies to channels 64 to 71
    case 7:
        // Disable all 125KHz channels
        memset1( unwrapped_channel_mask, 0x00, BANK_8_500_US915 );

        // Enable 500KHz channels
        memcpy1( &unwrapped_channel_mask[BANK_8_500_US915], ( uint8_t* ) &channel_mask, 1 );
        break;
    default:
        status = ERROR_CHANNEL_CNTL;
        break;
    }

    // Check if all enabled channels has a valid frequency
    for( uint8_t i = 0; i < const_number_of_tx_channel; i++ )
    {
        if( ( SMTC_GET_BIT8( unwrapped_channel_mask, i ) == CHANNEL_ENABLED ) &&
            ( region_us_915_get_tx_frequency_channel( lr1_mac, i ) == 0 ) )
        {
            status = ERROR_CHANNEL_MASK;  // this status is used only for the last multiple link adr req
            break;                        // break for loop
        }
    }

#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
    SMTC_MODEM_HAL_TRACE_PRINTF( "unwrapped channel 125 tx mask = 0x" );
    for( uint8_t i = BANK_0_125_US915; i < BANK_8_500_US915; i++ )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "%02x ", unwrapped_channel_mask[i] );
    }
    SMTC_MODEM_HAL_TRACE_PRINTF( " \n" );
    SMTC_MODEM_HAL_TRACE_PRINTF( "unwrapped channel 500 tx mask = 0x%02x\n", unwrapped_channel_mask[BANK_8_500_US915] );
#endif

    // check if all channels are disabled, return ERROR_CHANNEL_MASK
    if( SMTC_ARE_CLR_BYTE8( unwrapped_channel_mask, BANK_MAX_US915 ) == true )
    {
        status = ERROR_CHANNEL_MASK;
    }

    return ( status );
}

void region_us_915_enable_all_channels_with_valid_freq( lr1_stack_mac_t* lr1_mac )
{
    // Tx 125 kHz channels
    for( uint8_t i = 0; i < NUMBER_OF_TX_CHANNEL_US_915 - 8; i++ )
    {
        SMTC_PUT_BIT8( channel_index_enabled, i, CHANNEL_ENABLED );
        dr_bitfield_tx_channel[i] = DEFAULT_TX_DR_125_BIT_FIELD_US_915;
    }
    // Tx 500 kHz channels
    for( uint8_t i = NUMBER_OF_TX_CHANNEL_US_915 - 8; i < NUMBER_OF_TX_CHANNEL_US_915; i++ )
    {
        SMTC_PUT_BIT8( channel_index_enabled, i, CHANNEL_ENABLED );
        dr_bitfield_tx_channel[i] = DEFAULT_TX_DR_500_BIT_FIELD_US_915;
    }
}

modulation_type_t region_us_915_get_modulation_type_from_datarate( uint8_t datarate )
{
    modulation_type_t modulation;
    if( ( datarate <= 4 ) || ( ( datarate >= 8 ) && ( datarate <= 13 ) ) )
    {
        modulation = LORA;
    }
    else if( ( datarate == 5 ) || ( datarate == 6 ) )
    {
        // TODO LR_FHSS
        // modulation = ;
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( );
    }
    return modulation;
}

void region_us_915_lora_dr_to_sf_bw( uint8_t in_dr, uint8_t* out_sf, lr1mac_bandwidth_t* out_bw )
{
    if( ( in_dr <= 4 ) || ( ( in_dr >= 8 ) && ( in_dr <= 13 ) ) )
    {
        *out_sf = datarates_to_sf_us_915[in_dr];
        *out_bw = datarates_to_bandwidths_us_915[in_dr];
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( );
    }
}

uint32_t region_us_915_get_tx_frequency_channel( lr1_stack_mac_t* lr1_mac, uint8_t index )
{
    uint32_t freq = 0;
    // 500KHz channels
    if( index >= const_number_of_tx_channel - 8 )
    {
        freq = DEFAULT_TX_FREQ_500_START_US_915 + ( ( index % 8 ) * DEFAULT_TX_STEP_500_US_915 );
    }
    // 125KHz channels
    else
    {
        freq = DEFAULT_TX_FREQ_125_START_US_915 + ( index * DEFAULT_TX_STEP_125_US_915 );
    }
    return freq;
}

uint32_t region_us_915_get_rx1_frequency_channel( lr1_stack_mac_t* lr1_mac, uint8_t index )
{
    return ( DEFAULT_RX_FREQ_500_START_US_915 +
             ( ( index % NUMBER_OF_RX_CHANNEL_US_915 ) * DEFAULT_RX_STEP_500_US_915 ) );
}

void region_us_915_rx_dr_to_sf_bw( uint8_t dr, uint8_t* sf, lr1mac_bandwidth_t* bw, modulation_type_t* modulation_type )
{
    *modulation_type = LORA;
    if( ( dr >= MIN_RX_DR_US_915 ) && ( dr <= MAX_RX_DR_US_915 ) )
    {
        *sf = datarates_to_sf_us_915[dr];
        *bw = datarates_to_bandwidths_us_915[dr];
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( );
    }
}

uint8_t region_us_915_sf_bw_to_dr( lr1_stack_mac_t* lr1_mac, uint8_t sf, uint8_t bw )
{
    if( bw == BW_RFU )
    {
        smtc_modem_hal_lr1mac_panic( "Invalid Bandwith %u RFU\n", bw );
    }
    if( ( sf < 7 ) || ( sf > 12 ) )
    {
        smtc_modem_hal_lr1mac_panic( "Invalid sf %u\n", sf );
    }
    for( uint8_t i = 0; i < sizeof( datarates_to_sf_us_915 ); i++ )
    {
        if( ( datarates_to_sf_us_915[i] == sf ) && ( datarates_to_bandwidths_us_915[i] == bw ) )
        {
            return i;
        }
    }
    smtc_modem_hal_lr1mac_panic( "Invalid Datarate\n" );
    return 0;  // never reach => avoid warning
}

uint32_t region_us_915_get_rx_beacon_frequency_channel( lr1_stack_mac_t* lr1_mac, uint32_t gps_time_s )
{
    uint8_t index = ( uint32_t )( floorf( gps_time_s / 128 ) ) % 8;
    return ( BEACON_FREQ_START_US_915 + ( ( index % 8 ) * BEACON_STEP_US_915 ) );
}

uint32_t region_us_915_get_rx_ping_slot_frequency_channel( lr1_stack_mac_t* lr1_mac, uint32_t gps_time_s,
                                                           uint32_t dev_addr )
{
    uint8_t index = ( dev_addr + ( uint32_t )( floorf( gps_time_s / 128 ) ) ) % 8;
    return ( PING_SLOT_FREQ_START_US_915 + ( ( index % 8 ) * PING_SLOT_STEP_US_915 ) );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */
static void region_us_915_channel_mask_set_after_join( lr1_stack_mac_t* lr1_mac )
{
    // Copy all unwrapped channels in channel enable and in snapshot
    memcpy1( channel_index_enabled, unwrapped_channel_mask, BANK_MAX_US915 );
    memcpy1( snapshot_channel_tx_mask, unwrapped_channel_mask, BANK_MAX_US915 );

#if( BSP_DBG_TRACE == BSP_FEATURE_ON )
    SMTC_MODEM_HAL_TRACE_MSG( "Ch 125kHz\n" );
    for( uint8_t i = 0; i < NUMBER_OF_TX_CHANNEL_US_915 - 8; i++ )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " %d ", SMTC_GET_BIT8( channel_index_enabled, i ) );
    }
    SMTC_MODEM_HAL_TRACE_MSG( " \n" );
    SMTC_MODEM_HAL_TRACE_MSG( "Ch 500kHz\n" );
    for( uint8_t i = NUMBER_OF_TX_CHANNEL_US_915 - 8; i < NUMBER_OF_TX_CHANNEL_US_915; i++ )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " %d ", SMTC_GET_BIT8( channel_index_enabled, i ) );
    }
    SMTC_MODEM_HAL_TRACE_MSG( " \n" );
#endif

    first_ch_mask_received++;
}