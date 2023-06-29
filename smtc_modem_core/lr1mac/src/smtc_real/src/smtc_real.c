/**
 * \file      smtc_real.c
 *
 * \brief     Region Abstraction Layer (REAL) API implementation
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

#include "smtc_real.h"
#include "smtc_real_defs.h"

#include "lr1mac_utilities.h"
#include "lr1_stack_mac_layer.h"

#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_modem_hal.h"

#include "smtc_duty_cycle.h"
#include "smtc_lbt.h"
#include "lr1mac_config.h"

#if defined( REGION_WW2G4 )
#include "region_ww2g4.h"
#endif
#if defined( REGION_EU_868 )
#include "region_eu_868.h"
#endif
#if defined( REGION_AS_923 )
#include "region_as_923.h"
#endif
#if defined( REGION_US_915 )
#include "region_us_915.h"
#endif
#if defined( REGION_AU_915 )
#include "region_au_915.h"
#endif
#if defined( REGION_CN_470 )
#include "region_cn_470.h"
#endif
#if defined( REGION_CN_470_RP_1_0 )
#include "region_cn_470_rp_1_0.h"
#endif
#if defined( REGION_IN_865 )
#include "region_in_865.h"
#endif
#if defined( REGION_KR_920 )
#include "region_kr_920.h"
#endif
#if defined( REGION_RU_864 )
#include "region_ru_864.h"
#endif
#if !defined( REGION_WW2G4 ) && !defined( REGION_EU_868 ) && !defined( REGION_AS_923 ) && !defined( REGION_US_915 ) && \
    !defined( REGION_AU_915 ) && !defined( REGION_CN_470 ) && !defined( REGION_CN_470_RP_1_0 ) &&                      \
    !defined( REGION_IN_865 ) && !defined( REGION_KR_920 ) && !defined( REGION_RU_864 )
#error "Unknown region selected..."
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define real_ctx real->real_ctx
#define real_const real->real_const

#define tx_frequency_channel_ctx real_ctx.tx_frequency_channel_ctx
#define rx1_frequency_channel_ctx real_ctx.rx1_frequency_channel_ctx
#define channel_index_enabled_ctx real_ctx.channel_index_enabled_ctx
#define unwrapped_channel_mask_ctx real_ctx.unwrapped_channel_mask_ctx
#define dr_bitfield_tx_channel_ctx real_ctx.dr_bitfield_tx_channel_ctx
#define dr_distribution_init_ctx real_ctx.dr_distribution_init_ctx
#define dr_distribution_ctx real_ctx.dr_distribution_ctx
#define uplink_dwell_time_ctx real_ctx.uplink_dwell_time_ctx
#define downlink_dwell_time_ctx real_ctx.downlink_dwell_time_ctx

smtc_real_status_t smtc_real_is_supported_region( smtc_real_region_types_t region_type )
{
    for( uint8_t i = 0; i < SMTC_REAL_REGION_LIST_LENGTH; i++ )
    {
        if( smtc_real_region_list[i] == region_type )
        {
            return SMTC_REAL_STATUS_OK;
        }
    }

    SMTC_MODEM_HAL_TRACE_ERROR( "Invalid Region 0x%02x\n", region_type );
    return SMTC_REAL_STATUS_UNSUPPORTED_FEATURE;
}

void smtc_real_init( smtc_real_t* real, smtc_real_region_types_t region_type )
{
    if( smtc_real_is_supported_region( region_type ) != SMTC_REAL_STATUS_OK )
    {
        smtc_modem_hal_lr1mac_panic( );
    }
    real->region_type = region_type;

    // Init all real_const.const_xxx to 0
    memset( &( real_const ), 0, sizeof( smtc_real_const_t ) );

    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_init( real );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_init( real );
        break;
    }
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923: {
        region_as_923_init( real, 1 );
        break;
    }
    case SMTC_REAL_REGION_AS_923_GRP2: {
        region_as_923_init( real, 2 );
        break;
    }
    case SMTC_REAL_REGION_AS_923_GRP3: {
        region_as_923_init( real, 3 );
        break;
    }
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4: {
        region_as_923_init( real, 4 );
        break;
    }
#endif
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_init( real );
        break;
    }
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915: {
        region_au_915_init( real );
        break;
    }
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470: {
        region_cn_470_init( real );
        break;
    }
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0: {
        region_cn_470_rp_1_0_init( real );
        break;
    }
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865: {
        region_in_865_init( real );
        break;
    }
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920: {
        region_kr_920_init( real );
        break;
    }
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864: {
        region_ru_864_init( real );
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }

    real_ctx.sync_word_ctx = real_const.const_sync_word_public;
}

void smtc_real_config( smtc_real_t* real )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_config( real );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_config( real );
        break;
    }
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
    {
        region_as_923_config( real );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_config( real );
        break;
    }
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915: {
        region_au_915_config( real );
        break;
    }
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470: {
        region_cn_470_config( real );
        break;
    }
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0: {
        region_cn_470_rp_1_0_config( real );
        break;
    }
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865: {
        region_in_865_config( real );
        break;
    }
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920: {
        region_kr_920_config( real );
        break;
    }
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864: {
        region_ru_864_config( real );
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }

    uplink_dwell_time_ctx   = real_const.const_uplink_dwell_time;
    downlink_dwell_time_ctx = false;
}

void smtc_real_config_session( smtc_real_t* real )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915:
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915:
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_US_915 ) || \
    defined( REGION_AU_915 ) || defined( REGION_IN_865 ) || defined( REGION_KR_920 ) || defined( REGION_RU_864 )
    {
        // Not used for these regions
        break;
    }
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470: {
        region_cn_470_config_session( real );
        break;
    }
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0: {
        // Not used for these regions
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
}

void smtc_real_set_dr_distribution( smtc_real_t* real, uint8_t adr_mode, uint8_t* out_nb_trans, uint32_t* adr_custom )
{
    switch( adr_mode )
    {
    case MOBILE_LONGRANGE_DR_DISTRIBUTION:
#if !defined( HYBRID_CN470_MONO_CHANNEL )
        memcpy( dr_distribution_init_ctx, real_const.const_mobile_longrange_dr_distri,
                real_const.const_number_of_tx_dr );
        memcpy( dr_distribution_ctx, dr_distribution_init_ctx, real_const.const_number_of_tx_dr );
        *out_nb_trans = 3;
        break;
#endif
    case MOBILE_LOWPER_DR_DISTRIBUTION:
#if !defined( HYBRID_CN470_MONO_CHANNEL )
        memcpy( dr_distribution_init_ctx, real_const.const_mobile_lowpower_dr_distri,
                real_const.const_number_of_tx_dr );
        memcpy( dr_distribution_ctx, dr_distribution_init_ctx, real_const.const_number_of_tx_dr );
        *out_nb_trans = 3;
        break;
#endif
    case JOIN_DR_DISTRIBUTION:
#if !defined( HYBRID_CN470_MONO_CHANNEL )
        memcpy( dr_distribution_init_ctx, real_const.const_join_dr_distri, real_const.const_number_of_tx_dr );
        memcpy( dr_distribution_ctx, dr_distribution_init_ctx, real_const.const_number_of_tx_dr );
        *out_nb_trans = 1;
        break;
#endif
    case USER_DR_DISTRIBUTION:
        for( uint8_t i = 0; i < real_const.const_number_of_tx_dr; i++ )
        {
            if( i < 8 )
                dr_distribution_init_ctx[i] = ( adr_custom[0] >> ( ( 7 - i ) * 4 ) ) & 0x0F;
            else
            {
                uint8_t tmpi                = i % 8;
                dr_distribution_init_ctx[i] = ( adr_custom[1] >> ( ( 7 - tmpi ) * 4 ) ) & 0x0F;
            }
        }
        memcpy( dr_distribution_ctx, dr_distribution_init_ctx, real_const.const_number_of_tx_dr );
        *out_nb_trans = BSP_USER_NUMBER_OF_RETRANSMISSION;
        break;
    default:
        memcpy( dr_distribution_init_ctx, real_const.const_default_dr_distri, real_const.const_number_of_tx_dr );
        memcpy( dr_distribution_ctx, dr_distribution_init_ctx, real_const.const_number_of_tx_dr );
        *out_nb_trans = 1;
        break;
    }
}

status_lorawan_t smtc_real_get_next_tx_dr( smtc_real_t* real, join_status_t join_status, dr_strategy_t* adr_mode_select,
                                           uint8_t* tx_data_rate, uint8_t tx_data_rate_adr, bool* adr_enable )
{
    for( int j = 0; j < 224; j++ )  // return error after 224 trials
    {
        if( ( *adr_mode_select == STATIC_ADR_MODE ) && ( join_status == JOINED ) )
        {
            if( uplink_dwell_time_ctx == true )
            {
                *tx_data_rate = ( tx_data_rate_adr < real_const.const_min_tx_dr_limit )
                                    ? real_const.const_min_tx_dr_limit
                                    : tx_data_rate_adr;
            }
            else
            {
                *tx_data_rate = tx_data_rate_adr;
            }
            *adr_enable = 1;
        }
        else
        {
            uint8_t distri_sum = 0;
            for( uint8_t i = 0; i < real_const.const_number_of_tx_dr; i++ )
            {
                distri_sum += dr_distribution_ctx[i];
            }
            if( distri_sum == 0 )
            {
                memcpy1( dr_distribution_ctx, dr_distribution_init_ctx, real_const.const_number_of_tx_dr );
            }

            // Watchdog protection and dr_distribution_ctx is checked in for loop few line above
            uint8_t new_dr = 0;
            do
            {
                new_dr =
                    ( smtc_modem_hal_get_random_nb_in_range( real_const.const_min_tx_dr, real_const.const_max_tx_dr ) %
                      ( real_const.const_max_tx_dr + 1 ) );
            } while( dr_distribution_ctx[new_dr] == 0 );

            *tx_data_rate = new_dr;
            dr_distribution_ctx[new_dr]--;
            *adr_enable = 0;
        }
        *tx_data_rate = ( *tx_data_rate > real_const.const_max_tx_dr ) ? real_const.const_max_tx_dr : *tx_data_rate;

        uint16_t dr_tmp = smtc_real_mask_tx_dr_channel_up_dwell_time_check( real );
        if( SMTC_GET_BIT16( &dr_tmp, *tx_data_rate ) == 1 )
        {
            return OKLORAWAN;
        }
    }

    // No valid custom ADR found, force ADR to network controlled
    *adr_mode_select = STATIC_ADR_MODE;
    if( uplink_dwell_time_ctx == true )
    {
        *tx_data_rate = ( tx_data_rate_adr < real_const.const_min_tx_dr_limit ) ? real_const.const_min_tx_dr_limit
                                                                                : tx_data_rate_adr;
    }
    else
    {
        *tx_data_rate = tx_data_rate_adr;
    }
    *adr_enable = 1;
    return OKLORAWAN;
}

cf_list_type_t smtc_real_cf_list_type_supported( smtc_real_t* real )
{
    return ( real_const.const_cf_list_type_supported );
}

status_lorawan_t smtc_real_update_cflist( smtc_real_t* real, uint8_t* cf_list )
{
    status_lorawan_t status       = OKLORAWAN;
    cf_list_type_t   cf_list_type = cf_list[15];

    if( smtc_real_cf_list_type_supported( real ) != cf_list_type )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "INVALID CFList type for this region (%d)\n", cf_list_type );
        status = ERRORLORAWAN;
    }

    if( status == OKLORAWAN )
    {
        switch( cf_list_type )
        {
        case CF_LIST_FREQ: {
            for( uint8_t i = 0; i < 5; i++ )
            {
                tx_frequency_channel_ctx[real_const.const_number_of_boot_tx_channel + i] =
                    smtc_real_decode_freq_from_buf( real, &cf_list[0 + ( 3 * i )] );
                rx1_frequency_channel_ctx[real_const.const_number_of_boot_tx_channel + i] =
                    tx_frequency_channel_ctx[real_const.const_number_of_boot_tx_channel + i];

                if( smtc_real_is_nwk_received_tx_frequency_valid(
                        real, tx_frequency_channel_ctx[real_const.const_number_of_boot_tx_channel + i] ) == OKLORAWAN &&
                    tx_frequency_channel_ctx[real_const.const_number_of_boot_tx_channel + i] != 0 )
                {
                    // Enable default datarate for all added channels
                    dr_bitfield_tx_channel_ctx[real_const.const_number_of_boot_tx_channel + i] =
                        real_const.const_default_tx_dr_bit_field;

                    // Enable Channel
                    SMTC_PUT_BIT8( channel_index_enabled_ctx, ( real_const.const_number_of_boot_tx_channel + i ),
                                   CHANNEL_ENABLED );

                    SMTC_MODEM_HAL_TRACE_PRINTF(
                        "MacTxFrequency [%d] = %d \n", i,
                        tx_frequency_channel_ctx[real_const.const_number_of_boot_tx_channel + i] );

                    SMTC_MODEM_HAL_TRACE_PRINTF( "MacDataRateChannel [%d] = ", i );
                    for( uint8_t dr = 0; dr < 16; dr++ )
                    {
                        SMTC_MODEM_HAL_TRACE_PRINTF(
                            " %d",
                            SMTC_GET_BIT16( &dr_bitfield_tx_channel_ctx[real_const.const_number_of_boot_tx_channel + i],
                                            dr ) );
                    }
                    SMTC_MODEM_HAL_TRACE_PRINTF( " \n" );

                    SMTC_MODEM_HAL_TRACE_PRINTF( "MacChannelIndexEnabled [%d] = %d \n", i,
                                                 SMTC_GET_BIT8( channel_index_enabled_ctx,
                                                                ( real_const.const_number_of_boot_tx_channel + i ) ) );
                }
                else
                {
                    tx_frequency_channel_ctx[real_const.const_number_of_boot_tx_channel + i]  = 0;
                    rx1_frequency_channel_ctx[real_const.const_number_of_boot_tx_channel + i] = 0;
                    SMTC_PUT_BIT8( channel_index_enabled_ctx, ( real_const.const_number_of_boot_tx_channel + i ),
                                   CHANNEL_DISABLED );

                    SMTC_MODEM_HAL_TRACE_WARNING( "INVALID TX FREQUENCY IN CFLIST OR CFLIST EMPTY \n" );
                }
            }
        }
        break;
        case CF_LIST_CH_MASK: {
            uint8_t          ch_mash_i      = smtc_real_get_number_of_chmask_in_cflist( real );
            status_channel_t channel_status = OKCHANNEL;
            uint16_t         channel_mask;

            smtc_real_init_channel_mask( real );

            for( uint8_t i = 0; i < ch_mash_i; i++ )
            {
                channel_mask   = cf_list[0 + ( 2 * i )] + ( cf_list[1 + ( 2 * i )] << 8 );
                channel_status = smtc_real_build_channel_mask( real, i, channel_mask );

                if( channel_status == ERROR_CHANNEL_CNTL )
                {  // Test ChannelCNTL not defined
                    SMTC_MODEM_HAL_TRACE_MSG( "INVALID CHANNEL CNTL IN JOIN\n" );
                    break;
                }
                // Valid global channel mask
                if( channel_status == ERROR_CHANNEL_MASK )
                {  // Test Channelmask enables a not defined channel or Channelmask = 0
                    SMTC_MODEM_HAL_TRACE_MSG( "INVALID CHANNEL MASK IN JOIN\n" );
                    break;
                }
            }
            if( channel_status == OKCHANNEL )
            {
                smtc_real_set_channel_mask( real );
            }
            break;
        }
        default:
            SMTC_MODEM_HAL_TRACE_WARNING( "INVALID CFLIST, MUST CONTAINS FREQ OR CHMASK \n" );
            status = ERRORLORAWAN;
            break;
        }
    }
    return status;
}

uint8_t smtc_real_get_number_of_chmask_in_cflist( smtc_real_t* real )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_IN_865 ) || \
    defined( REGION_KR_920 ) || defined( REGION_RU_864 )
    {
        return 0;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915:
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915:
#endif
#if defined( REGION_US_915 ) || defined( REGION_AU_915 )
    {
        return 5;
    }
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470: {
        return region_cn_470_get_number_of_chmask_in_cflist( real );
    }
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0: {
        return region_cn_470_rp_1_0_get_number_of_chmask_in_cflist( real );
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

status_lorawan_t smtc_real_get_next_channel( smtc_real_t* real, smtc_dtc_t* dtc_obj, uint8_t tx_data_rate,
                                             uint32_t* out_tx_frequency, uint32_t* out_rx1_frequency )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_get_next_channel( real, tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_get_next_channel( real, dtc_obj, tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
    {
        return region_as_923_get_next_channel( real, tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_get_next_channel( real, tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915: {
        return region_au_915_get_next_channel( real, tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470: {
        return region_cn_470_get_next_channel( real, tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0: {
        return region_cn_470_rp_1_0_get_next_channel( real, tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865: {
        return region_in_865_get_next_channel( real, tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920: {
        return region_kr_920_get_next_channel( real, tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864: {
        return region_ru_864_get_next_channel( real, dtc_obj, tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

status_lorawan_t smtc_real_get_join_next_channel( smtc_real_t* real, smtc_dtc_t* dtc_obj, uint8_t* tx_data_rate,
                                                  uint32_t* out_tx_frequency, uint32_t* out_rx1_frequency,
                                                  uint32_t* out_rx2_frequency )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_get_join_next_channel( real, *tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_get_join_next_channel( real, dtc_obj, *tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
    {
        return region_as_923_get_join_next_channel( real, *tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_get_join_next_channel( real, tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915: {
        return region_au_915_get_join_next_channel( real, tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470: {
        return region_cn_470_get_join_next_channel( real, *tx_data_rate, out_tx_frequency, out_rx1_frequency,
                                                    out_rx2_frequency );
    }
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0: {
        return region_cn_470_rp_1_0_get_join_next_channel( real, *tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865: {
        return region_in_865_get_join_next_channel( real, *tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920: {
        return region_kr_920_get_join_next_channel( real, *tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864: {
        return region_ru_864_get_join_next_channel( real, dtc_obj, *tx_data_rate, out_tx_frequency, out_rx1_frequency );
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

uint8_t smtc_real_get_rx1_datarate_config( smtc_real_t* real, uint8_t tx_data_rate, uint8_t rx1_dr_offset )
{
    uint8_t max   = real_const.const_number_of_tx_dr * real_const.const_number_rx1_dr_offset;
    uint8_t index = ( tx_data_rate * real_const.const_number_rx1_dr_offset ) + rx1_dr_offset;

    ;
#if defined( REGION_AS_923 )
    if( real->region_type == SMTC_REAL_REGION_AS_923 )
    {
        max *= ( downlink_dwell_time_ctx + 1 );
        index += ( downlink_dwell_time_ctx * real_const.const_number_of_tx_dr * real_const.const_number_rx1_dr_offset );
    }
#endif

    if( index >= max )
    {
        smtc_modem_hal_lr1mac_panic( );
    }
    return real_const.const_datarate_offsets[index];
}

int8_t smtc_real_convert_power_cmd( smtc_real_t* real, uint8_t power_cmd, uint8_t max_erp_dbm )
{
    if( power_cmd > real_const.const_max_tx_power_idx )
    {
        return max_erp_dbm;
        SMTC_MODEM_HAL_TRACE_WARNING( "INVALID %d \n", power_cmd );
    }
    else
    {
        int8_t pwr_tmp = max_erp_dbm - ( 2 * power_cmd );
        return ( pwr_tmp < 0 ) ? 0 : pwr_tmp;
    }
}

void smtc_real_set_channel_mask( smtc_real_t* real )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470:
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0:
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif

#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( RP2_103 ) || \
    defined( REGION_CN_470 ) || defined( REGION_CN_470_RP_1_0 ) || defined( REGION_IN_865 ) ||               \
    defined( REGION_KR_920 ) || defined( REGION_RU_864 )
        // Copy all unwrapped channels in channel enable
        memcpy1( channel_index_enabled_ctx, unwrapped_channel_mask_ctx, real_const.const_number_of_channel_bank );

        for( uint8_t i = 0; i < real_const.const_number_of_tx_channel; i++ )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( " %d ", SMTC_GET_BIT8( channel_index_enabled_ctx, i ) );
        }
        SMTC_MODEM_HAL_TRACE_MSG( " \n" );
        break;
#endif

#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_set_channel_mask( real );
        break;
    }
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915: {
        region_au_915_set_channel_mask( real );
        break;
    }
#endif

    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
}

void smtc_real_init_channel_mask( smtc_real_t* real )
{
    memset1( unwrapped_channel_mask_ctx, 0xFF, real_const.const_number_of_channel_bank );
}

void smtc_real_init_join_snapshot_channel_mask( smtc_real_t* real )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470:
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0:
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_CN_470 ) || \
    defined( REGION_CN_470_RP_1_0 ) || defined( REGION_IN_865 ) || defined( REGION_KR_920 ) ||                     \
    defined( REGION_RU_864 )
    {
        // Not used for these regions
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_init_join_snapshot_channel_mask( real );
        break;
    }
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915: {
        region_au_915_init_join_snapshot_channel_mask( real );
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
}

void smtc_real_init_after_join_snapshot_channel_mask( smtc_real_t* real, uint8_t tx_data_rate, uint32_t tx_frequency )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470:
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0:
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_CN_470 ) || \
    defined( REGION_CN_470_RP_1_0 ) || defined( REGION_IN_865 ) || defined( REGION_KR_920 ) ||                     \
    defined( REGION_RU_864 )
    {
        // Not used for these regions
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_init_after_join_snapshot_channel_mask( real, tx_data_rate, tx_frequency );
        break;
    }
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915: {
        region_au_915_init_after_join_snapshot_channel_mask( real, tx_data_rate, tx_frequency );
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
}

status_channel_t smtc_real_build_channel_mask( smtc_real_t* real, uint8_t ch_mask_cntl, uint16_t ch_mask )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_build_channel_mask( real, ch_mask_cntl, ch_mask );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_build_channel_mask( real, ch_mask_cntl, ch_mask );
    }
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
    {
        return region_as_923_build_channel_mask( real, ch_mask_cntl, ch_mask );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_build_channel_mask( real, ch_mask_cntl, ch_mask );
    }
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915: {
        return region_au_915_build_channel_mask( real, ch_mask_cntl, ch_mask );
    }
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470: {
        return region_cn_470_build_channel_mask( real, ch_mask_cntl, ch_mask );
    }
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0: {
        return region_cn_470_rp_1_0_build_channel_mask( real, ch_mask_cntl, ch_mask );
    }
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865: {
        return region_in_865_build_channel_mask( real, ch_mask_cntl, ch_mask );
    }
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920: {
        return region_kr_920_build_channel_mask( real, ch_mask_cntl, ch_mask );
    }
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864: {
        return region_ru_864_build_channel_mask( real, ch_mask_cntl, ch_mask );
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
    return ERROR_CHANNEL;  // never reach => avoid warning
}

uint8_t smtc_real_decrement_dr_simulation( smtc_real_t* real, uint8_t tx_data_rate_adr )
{
    bool    is_valid_dr          = false;
    uint8_t data_rate_simulation = tx_data_rate_adr;

    // while( ( data_rate_simulation > real_const.const_min_tx_dr ) && ( is_valid_dr == 0 ) )
    while( data_rate_simulation > real_const.const_min_tx_dr )
    {
        uint8_t index = ( uplink_dwell_time_ctx * real_const.const_number_of_tx_dr ) + data_rate_simulation;
        if( index > ( real_const.const_max_tx_dr * ( uplink_dwell_time_ctx + 1 ) ) )
        {
            smtc_modem_hal_lr1mac_panic( );
        }
        data_rate_simulation = real_const.const_datarate_backoff[index];

        if( smtc_real_is_tx_dr_acceptable( real, data_rate_simulation, false ) == OKLORAWAN )
        {
            is_valid_dr = true;
            break;
        }
    }

    // no lower valid DR found
    if( is_valid_dr == false )
    {
        return tx_data_rate_adr;
    }

    return data_rate_simulation;
}

void smtc_real_decrement_dr( smtc_real_t* real, dr_strategy_t adr_mode_select, uint8_t* tx_data_rate_adr,
                             int8_t* tx_power, uint8_t* nb_trans )
{
    if( adr_mode_select != STATIC_ADR_MODE )
    {
        return;
    }

    if( *tx_power < real_const.const_tx_power_dbm )
    {
        *tx_power = real_const.const_tx_power_dbm;
        return;
    }

    if( *tx_data_rate_adr != smtc_real_get_min_tx_channel_dr( real ) )
    {
        *tx_data_rate_adr = smtc_real_decrement_dr_simulation( real, *tx_data_rate_adr );
        return;
    }

    //  nb_trans must set to 1 and all default channels re-enabled
    *nb_trans = 1;
    // reach this step only if tx_dr = MinDr => enable default channel
    smtc_real_enable_all_channels_with_valid_freq( real );

    // When all channels are available, lower datarate could be present in case where only DR6 and DR7 was present
    // previously
    *tx_data_rate_adr = smtc_real_decrement_dr_simulation( real, *tx_data_rate_adr );
}

void smtc_real_enable_all_channels_with_valid_freq( smtc_real_t* real )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_IN_865 ) || \
    defined( REGION_KR_920 ) || defined( REGION_RU_864 )
    {
        for( uint8_t i = 0; i < real_const.const_number_of_tx_channel; i++ )
        {
            if( ( tx_frequency_channel_ctx[i] != 0 ) &&
                ( SMTC_GET_BIT8( channel_index_enabled_ctx, i ) == CHANNEL_DISABLED ) )
            {
                SMTC_PUT_BIT8( channel_index_enabled_ctx, i, CHANNEL_ENABLED );
                dr_bitfield_tx_channel_ctx[i] = real_const.const_default_tx_dr_bit_field;
            }
        }
        break;
    }
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470: {
        region_cn_470_enable_all_channels_with_valid_freq( real );
        break;
    }
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0: {
        region_cn_470_rp_1_0_enable_all_channels_with_valid_freq( real );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_enable_all_channels_with_valid_freq( real );
        break;
    }
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915: {
        region_au_915_enable_all_channels_with_valid_freq( real );
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
}

status_lorawan_t smtc_real_is_rx1_dr_offset_valid( smtc_real_t* real, uint8_t rx1_dr_offset )
{
    status_lorawan_t status = OKLORAWAN;
    if( rx1_dr_offset >= real_const.const_number_rx1_dr_offset )
    {
        status = ERRORLORAWAN;
        SMTC_MODEM_HAL_TRACE_MSG( "RECEIVE AN INVALID RX1 DR OFFSET \n" );
    }
    return ( status );
}

status_lorawan_t smtc_real_is_rx_dr_valid( smtc_real_t* real, uint8_t dr )
{
    if( ( dr >= real_const.const_min_rx_dr ) && ( dr <= real_const.const_max_rx_dr ) )
    {
        if( SMTC_GET_BIT16( &real_const.const_dr_bitfield, dr ) == 1 )
        {
            return ( OKLORAWAN );
        }
    }
    SMTC_MODEM_HAL_TRACE_WARNING( "Invalid Rx datarate %d\n", dr );

    return ( ERRORLORAWAN );
}

status_lorawan_t smtc_real_is_tx_dr_valid( smtc_real_t* real, uint8_t dr )
{
    if( ( dr >= real_const.const_min_tx_dr ) && ( dr <= real_const.const_max_tx_dr ) )
    {
        if( SMTC_GET_BIT16( &real_const.const_dr_bitfield, dr ) == 1 )
        {
            return ( OKLORAWAN );
        }
    }
    SMTC_MODEM_HAL_TRACE_WARNING( "Invalid Tx datarate %d\n", dr );

    return ( ERRORLORAWAN );
}

status_lorawan_t smtc_real_is_tx_dr_acceptable( smtc_real_t* real, uint8_t dr, bool is_ch_mask_from_link_adr )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470:
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0:
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_CN_470 ) || \
    defined( REGION_CN_470_RP_1_0 ) || defined( REGION_IN_865 ) || defined( REGION_KR_920 ) ||                     \
    defined( REGION_RU_864 )
    {
        uint8_t* ch_mask_to_check =
            ( is_ch_mask_from_link_adr == true ) ? unwrapped_channel_mask_ctx : channel_index_enabled_ctx;

        if( uplink_dwell_time_ctx == true )
        {
            if( dr < real_const.const_min_tx_dr_limit )
            {
                return ERRORLORAWAN;
            }
        }

        for( uint8_t i = 0; i < real_const.const_number_of_tx_channel; i++ )
        {
            if( SMTC_GET_BIT8( ch_mask_to_check, i ) == CHANNEL_ENABLED )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "ch%d - dr field 0x%04x\n", i, dr_bitfield_tx_channel_ctx[i] );
                if( SMTC_GET_BIT16( &dr_bitfield_tx_channel_ctx[i], dr ) == 1 )
                {
                    return ( OKLORAWAN );
                }
            }
        }

        SMTC_MODEM_HAL_TRACE_WARNING( "Not acceptable data rate\n" );
        return ( ERRORLORAWAN );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915:
        return region_us_915_is_acceptable_tx_dr( real, dr, is_ch_mask_from_link_adr );
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915:
        return region_au_915_is_acceptable_tx_dr( real, dr, is_ch_mask_from_link_adr, uplink_dwell_time_ctx );
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

status_lorawan_t smtc_real_is_nwk_received_tx_frequency_valid( smtc_real_t* real, uint32_t frequency )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_IN_865 ) || \
    defined( REGION_KR_920 ) || defined( REGION_RU_864 )
    {
        status_lorawan_t status = OKLORAWAN;
        if( frequency == 0 )
        {
            return ( status );
        }
        status = smtc_real_is_frequency_valid( real, frequency );
        return ( status );
    }
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470:
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0:
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915:
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915:
#endif
#if defined( REGION_CN_470 ) || defined( REGION_CN_470_RP_1_0 ) || defined( REGION_US_915 ) || defined( REGION_AU_915 )
    {
        return ( ERRORLORAWAN );
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

status_lorawan_t smtc_real_is_channel_index_valid( smtc_real_t* real, uint8_t channel_index )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_IN_865 ) || \
    defined( REGION_KR_920 ) || defined( REGION_RU_864 )
    {
        status_lorawan_t status = OKLORAWAN;
        if( ( channel_index < real_const.const_number_of_boot_tx_channel ) ||
            ( channel_index >= real_const.const_number_of_tx_channel ) )
        {
            status = ERRORLORAWAN;
            SMTC_MODEM_HAL_TRACE_WARNING( "RECEIVE AN INVALID Channel Index Cmd = %d\n", channel_index );
        }
        return ( status );
    }
#endif

#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470:
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0:
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915:
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915:
#endif
#if defined( REGION_CN_470 ) || defined( REGION_CN_470_RP_1_0 ) || defined( REGION_US_915 ) || defined( REGION_AU_915 )
    {
        return ( ERRORLORAWAN );
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

status_lorawan_t smtc_real_is_payload_size_valid( smtc_real_t* real, uint8_t dr, uint8_t size,
                                                  direction_frame_t direction_frame, uint8_t tx_fopts_current_length )
{
    bool dwell_time_enabled = ( direction_frame == UP_LINK ) ? uplink_dwell_time_ctx : downlink_dwell_time_ctx;
    if( ( real_const.const_tx_param_setup_req_supported == false ) && ( dwell_time_enabled != 0 ) )
    {
        smtc_modem_hal_lr1mac_panic( );
    }

    uint8_t index = ( dwell_time_enabled * real_const.const_number_of_tx_dr ) + dr;

#if defined( REGION_AU_915 )
    if( real->region_type == SMTC_REAL_REGION_AU_915 )
    {
        // *2 because the array contains Tx and Rx datarate
        index = ( dwell_time_enabled * real_const.const_number_of_tx_dr * 2 ) + dr;
    }
#endif

    status_lorawan_t status = ( ( size + tx_fopts_current_length ) > ( real_const.const_max_payload_m[index] - 8 ) )
                                  ? ERRORLORAWAN
                                  : OKLORAWAN;
    if( status == ERRORLORAWAN )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Invalid size (data:%d + FOpts:%d) > %d for dr: %d\n", size,
                                     tx_fopts_current_length, ( real_const.const_max_payload_m[index] - 8 ), dr );
    }
    return ( status );
}

void smtc_real_set_tx_frequency_channel( smtc_real_t* real, uint32_t tx_freq, uint8_t channel_index )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_IN_865 ) || \
    defined( REGION_KR_920 ) || defined( REGION_RU_864 )
        if( channel_index >= real_const.const_number_of_tx_channel )
        {
            smtc_modem_hal_lr1mac_panic( );
        }
        else
        {
            tx_frequency_channel_ctx[channel_index] = tx_freq;
        }
        break;
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470:
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0:
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915:
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915:
#endif
#if defined( REGION_CN_470 ) || defined( REGION_CN_470_RP_1_0 ) || defined( REGION_US_915 ) || defined( REGION_AU_915 )
    {
        // Not supported
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
}

status_lorawan_t smtc_real_set_rx1_frequency_channel( smtc_real_t* real, uint32_t rx_freq, uint8_t channel_index )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_IN_865 ) || \
    defined( REGION_KR_920 ) || defined( REGION_RU_864 )
        if( channel_index >= real_const.const_number_of_rx_channel )
        {
            smtc_modem_hal_lr1mac_panic( );
        }
        else
        {
            rx1_frequency_channel_ctx[channel_index] = rx_freq;
        }
        return OKLORAWAN;
        break;
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470:
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0:
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915:
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915:
#endif
#if defined( REGION_CN_470 ) || defined( REGION_CN_470_RP_1_0 ) || defined( REGION_US_915 ) || defined( REGION_AU_915 )
    {
        // Not supported
        return ERRORLORAWAN;
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

void smtc_real_set_channel_dr( smtc_real_t* real, uint8_t channel_index, uint8_t dr_min, uint8_t dr_max )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_IN_865 ) || \
    defined( REGION_KR_920 ) || defined( REGION_RU_864 )
        if( channel_index >= real_const.const_number_of_tx_channel )
        {
            smtc_modem_hal_lr1mac_panic( );
        }
        else
        {
            dr_bitfield_tx_channel_ctx[channel_index] = 0;
            for( uint8_t i = dr_min; i <= dr_max; i++ )
            {
                uint8_t tmp_dr = SMTC_GET_BIT16( &real_const.const_dr_bitfield, i );
                SMTC_PUT_BIT16( &dr_bitfield_tx_channel_ctx[channel_index], i, tmp_dr );
            }
        }
        break;
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470:
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0:
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915:
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915:
#endif
#if defined( REGION_CN_470 ) || defined( REGION_CN_470_RP_1_0 ) || defined( REGION_US_915 ) || defined( REGION_AU_915 )
    {
        // Not supported
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
}

void smtc_real_set_channel_enabled( smtc_real_t* real, uint8_t enable, uint8_t channel_index )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_IN_865 ) || \
    defined( REGION_KR_920 ) || defined( REGION_RU_864 )
        if( channel_index >= real_const.const_number_of_tx_channel )
        {
            smtc_modem_hal_lr1mac_panic( );
        }
        else
        {
            SMTC_PUT_BIT8( channel_index_enabled_ctx, channel_index, enable );
        }
        break;
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470:
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0:
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915:
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915:
#endif
#if defined( REGION_CN_470 ) || defined( REGION_CN_470_RP_1_0 ) || defined( REGION_US_915 ) || defined( REGION_AU_915 )
    {
        // Not supported
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
}

uint32_t smtc_real_get_tx_channel_frequency( smtc_real_t* real, uint8_t channel_index )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_IN_865 ) || \
    defined( REGION_KR_920 ) || defined( REGION_RU_864 )
        if( channel_index >= real_const.const_number_of_tx_channel )
        {
            smtc_modem_hal_lr1mac_panic( );
        }
        return ( tx_frequency_channel_ctx[channel_index] );
        break;
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470: {
        return region_cn_470_get_tx_frequency_channel( real, channel_index );
        break;
    }
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0: {
        return region_cn_470_rp_1_0_get_tx_frequency_channel( real, channel_index );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_get_tx_frequency_channel( real, channel_index );
        break;
    }
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915: {
        return region_au_915_get_tx_frequency_channel( real, channel_index );
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

uint32_t smtc_real_get_rx1_channel_frequency( smtc_real_t* real, uint8_t channel_index )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_IN_865 ) || \
    defined( REGION_KR_920 ) || defined( REGION_RU_864 )
        if( channel_index >= real_const.const_number_of_rx_channel )
        {
            smtc_modem_hal_lr1mac_panic( );
        }
        return ( rx1_frequency_channel_ctx[channel_index] );
        break;
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470: {
        return region_cn_470_get_rx1_frequency_channel( real, channel_index );
        break;
    }
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0: {
        return region_cn_470_rp_1_0_get_rx1_frequency_channel( real, channel_index );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_get_rx1_frequency_channel( real, channel_index );
        break;
    }
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915: {
        return region_au_915_get_rx1_frequency_channel( real, channel_index );
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

uint8_t smtc_real_get_min_tx_channel_dr( smtc_real_t* real )
{
    uint8_t min_dr = real_const.const_max_tx_dr;  // start with the max dr and search a dr inferior
    for( uint8_t i = 0; i < real_const.const_number_of_tx_channel; i++ )
    {
        if( ( SMTC_GET_BIT8( channel_index_enabled_ctx, i ) == CHANNEL_ENABLED ) )
        {
            for( uint8_t dr = real_const.const_min_tx_dr; dr <= real_const.const_max_tx_dr; dr++ )
            {
                if( ( SMTC_GET_BIT16( &dr_bitfield_tx_channel_ctx[i], dr ) == 1 ) && ( min_dr > dr ) )
                {
                    min_dr = dr;
                }
            }

            if( min_dr == real_const.const_min_tx_dr )
            {
                break;  // DR found is the smallest
            }
        }
    }

    if( uplink_dwell_time_ctx == true )
    {
        min_dr = MAX( min_dr, real_const.const_min_tx_dr_limit );
    }

    return ( min_dr );
}

uint8_t smtc_real_get_max_tx_channel_dr( smtc_real_t* real )
{
    uint8_t max_dr = real_const.const_min_tx_dr;  // start with the min dr and search a dr superior
    for( uint8_t i = 0; i < real_const.const_number_of_tx_channel; i++ )
    {
        if( ( SMTC_GET_BIT8( channel_index_enabled_ctx, i ) == CHANNEL_ENABLED ) )
        {
            for( uint8_t dr = 0; dr <= real_const.const_max_tx_dr; dr++ )
            {
                if( ( SMTC_GET_BIT16( &dr_bitfield_tx_channel_ctx[i], dr ) == 1 ) && ( max_dr < dr ) )
                {
                    max_dr = dr;
                }
            }

            if( max_dr == real_const.const_max_tx_dr )
            {
                break;  // DR found is the bigest
            }
        }
    }

    return ( max_dr );
}

uint16_t smtc_real_mask_tx_dr_channel( smtc_real_t* real )
{
    uint16_t dr_mask = 0;
    for( uint8_t i = 0; i < real_const.const_number_of_tx_channel; i++ )
    {
        if( SMTC_GET_BIT8( channel_index_enabled_ctx, i ) == CHANNEL_ENABLED )
        {
            dr_mask |= dr_bitfield_tx_channel_ctx[i];
        }
    }

    return dr_mask;
}

uint16_t smtc_real_mask_tx_dr_channel_up_dwell_time_check( smtc_real_t* real )
{
    uint16_t dr_mask = smtc_real_mask_tx_dr_channel( real );
    if( uplink_dwell_time_ctx == true )
    {
        for( uint8_t i = 0; i < real_const.const_min_tx_dr_limit; i++ )
        {
            SMTC_PUT_BIT16( &dr_mask, i, false );
        }
    }

    return dr_mask;
}

uint8_t smtc_real_get_preamble_len( const smtc_real_t* real, uint8_t sf )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        if( ( sf == 5 ) || ( sf == 6 ) )
        {
            return 12;
        }
        else
        {
            return 8;
        }
        break;
    }
#endif
    default:
        return 8;
        break;
    }
}

status_lorawan_t smtc_real_is_channel_mask_for_mobile_mode( const smtc_real_t* real )
{
    status_lorawan_t status        = ERRORLORAWAN;
    uint8_t          min_mobile_dr = real_const.const_min_tx_dr;
    uint8_t          max_mobile_dr = real_const.const_max_tx_dr;

    // search min datarate init
    for( int i = 0; i < real_const.const_number_of_tx_dr; i++ )
    {
        if( dr_distribution_init_ctx[i] > 0 )
        {
            min_mobile_dr = i;
            break;
        }
    }
    if( uplink_dwell_time_ctx == true )
    {
        min_mobile_dr = MAX( min_mobile_dr, real_const.const_min_tx_dr_limit );
    }

    // search max datarate init
    for( int i = real_const.const_number_of_tx_dr - 1; i <= 0; i-- )
    {
        if( dr_distribution_init_ctx[i] > 0 )
        {
            max_mobile_dr = i;
            break;
        }
    }

    for( int i = 0; i < real_const.const_number_of_tx_channel; i++ )
    {
        if( SMTC_GET_BIT8( unwrapped_channel_mask_ctx, i ) == CHANNEL_ENABLED )
        {
            for( uint8_t dr = real_const.const_min_tx_dr; dr <= real_const.const_max_tx_dr; dr++ )
            {
                if( SMTC_GET_BIT16( &dr_bitfield_tx_channel_ctx[i], dr ) == 1 )
                {
                    if( ( dr >= min_mobile_dr ) && ( dr <= max_mobile_dr ) )
                    {
                        return ( OKLORAWAN );
                    }
                }
            }
        }
    }
    SMTC_MODEM_HAL_TRACE_WARNING( "Not acceptable data rate in mobile mode\n" );
    return ( status );
}

modulation_type_t smtc_real_get_modulation_type_from_datarate( smtc_real_t* real, uint8_t datarate )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_get_modulation_type_from_datarate( datarate );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_get_modulation_type_from_datarate( datarate );
        break;
    }
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
    {
        return region_as_923_get_modulation_type_from_datarate( datarate );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_get_modulation_type_from_datarate( datarate );
        break;
    }
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915: {
        return region_au_915_get_modulation_type_from_datarate( datarate );
        break;
    }
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470: {
        return region_cn_470_get_modulation_type_from_datarate( datarate );
        break;
    }
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0: {
        return region_cn_470_rp_1_0_get_modulation_type_from_datarate( datarate );
        break;
    }
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865: {
        return region_in_865_get_modulation_type_from_datarate( datarate );
        break;
    }
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920: {
        return region_kr_920_get_modulation_type_from_datarate( datarate );
        break;
    }
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864: {
        return region_ru_864_get_modulation_type_from_datarate( datarate );
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
    return 0;  // never reach => avoid warning
}
void smtc_real_lora_dr_to_sf_bw( smtc_real_t* real, uint8_t in_dr, uint8_t* out_sf, lr1mac_bandwidth_t* out_bw )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_lora_dr_to_sf_bw( in_dr, out_sf, out_bw );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_lora_dr_to_sf_bw( in_dr, out_sf, out_bw );
        break;
    }
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
    {
        region_as_923_lora_dr_to_sf_bw( in_dr, out_sf, out_bw );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_lora_dr_to_sf_bw( in_dr, out_sf, out_bw );
        break;
    }
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915: {
        region_au_915_lora_dr_to_sf_bw( in_dr, out_sf, out_bw );
        break;
    }
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470: {
        region_cn_470_lora_dr_to_sf_bw( in_dr, out_sf, out_bw );
        break;
    }
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0: {
        region_cn_470_rp_1_0_lora_dr_to_sf_bw( in_dr, out_sf, out_bw );
        break;
    }
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865: {
        region_in_865_lora_dr_to_sf_bw( in_dr, out_sf, out_bw );
        break;
    }
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920: {
        region_kr_920_lora_dr_to_sf_bw( in_dr, out_sf, out_bw );
        break;
    }
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864: {
        region_ru_864_lora_dr_to_sf_bw( in_dr, out_sf, out_bw );
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
}

void smtc_real_fsk_dr_to_bitrate( smtc_real_t* real, uint8_t in_dr, uint8_t* out_bitrate )
{
    switch( real->region_type )
    {
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_fsk_dr_to_bitrate( in_dr, out_bitrate );
        break;
    }
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
    {
        region_as_923_fsk_dr_to_bitrate( in_dr, out_bitrate );
        break;
    }
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470: {
        region_cn_470_fsk_dr_to_bitrate( in_dr, out_bitrate );
        break;
    }
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865: {
        region_in_865_fsk_dr_to_bitrate( in_dr, out_bitrate );
        break;
    }
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864: {
        region_ru_864_fsk_dr_to_bitrate( in_dr, out_bitrate );
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
}

void smtc_real_lr_fhss_dr_to_cr_bw( smtc_real_t* real, uint8_t in_dr, lr_fhss_v1_cr_t* out_cr, lr_fhss_v1_bw_t* out_bw )
{
    switch( real->region_type )
    {
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_lr_fhss_dr_to_cr_bw( in_dr, out_cr, out_bw );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_lr_fhss_dr_to_cr_bw( in_dr, out_cr, out_bw );
        break;
    }
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915: {
        region_au_915_lr_fhss_dr_to_cr_bw( in_dr, out_cr, out_bw );
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
}

lr_fhss_hc_t smtc_real_lr_fhss_get_header_count( lr_fhss_v1_cr_t in_cr )
{
    if( in_cr == LR_FHSS_V1_CR_1_3 )
    {
        return LR_FHSS_HC_3;
    }
    else if( in_cr == LR_FHSS_V1_CR_2_3 )
    {
        return LR_FHSS_HC_2;
    }

    smtc_modem_hal_lr1mac_panic( );
    return 0;
}

lr_fhss_v1_grid_t smtc_real_lr_fhss_get_grid( smtc_real_t* real )
{
    switch( real->region_type )
    {
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return LR_FHSS_V1_GRID_3906_HZ;
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915:
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915:
#endif
#if defined( REGION_US_915 ) || defined( REGION_AU_915 )
    {
        return LR_FHSS_V1_GRID_25391_HZ;
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
    return -1;  // never reach => avoid warning
}

uint8_t smtc_real_get_number_of_enabled_channels_for_a_datarate( smtc_real_t* real, uint8_t datarate )
{
    uint8_t channel_counter = 0;

    for( uint8_t i = 0; i < real_const.const_number_of_tx_channel; i++ )
    {
        if( SMTC_GET_BIT8( channel_index_enabled_ctx, i ) == CHANNEL_ENABLED )
        {
            if( SMTC_GET_BIT16( &dr_bitfield_tx_channel_ctx[i], datarate ) == 1 )
            {
                channel_counter++;
            }
        }
    }
    return channel_counter;
}

int8_t smtc_real_clamp_output_power_eirp_vs_freq_and_dr( smtc_real_t* real, int8_t tx_power, uint32_t tx_frequency,
                                                         uint8_t datarate )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915:
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470:
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0:
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_AU_915 ) || \
    defined( REGION_CN_470 ) || defined( REGION_CN_470_RP_1_0 ) || defined( REGION_IN_865 ) ||                     \
    defined( REGION_RU_864 )
    {
        return tx_power;
        break;
    }
#endif

#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        if( datarate == DR4 )
        {
            return MIN( tx_power, 26 );
        }
        else if( smtc_real_get_number_of_enabled_channels_for_a_datarate( real, datarate ) < 50 )
        {
            return MIN( tx_power, 21 );
        }
        return tx_power;
        break;
    }
#endif

#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920: {
        if( tx_frequency < 922000000 )
        {
            return MIN( tx_power, 10 );  // if freq < 922MHz, Max output power is limited to 10 dBm
        }
        else
        {
            return MIN( tx_power, TX_POWER_EIRP_KR_920 );  // else Max output power is limited to 14 dBm
        }
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
    return 0;  // never reach => avoid warning
}

uint8_t smtc_real_get_current_enabled_frequency_list( smtc_real_t* real, uint8_t* number_of_freq, uint32_t* freq_list,
                                                      uint8_t max_size )
{
    *number_of_freq = 0;
    if( max_size < real_const.const_number_of_tx_channel )
    {
        return false;
    }
    for( int i = 0; i < real_const.const_number_of_tx_channel; i++ )
    {
        if( SMTC_GET_BIT8( channel_index_enabled_ctx, i ) == CHANNEL_ENABLED )
        {
            freq_list[*number_of_freq] = tx_frequency_channel_ctx[i];
            *number_of_freq += 1;
        }
    }
    return true;
}

/*************************************************************************/
/*                      Const init in region                             */
/*************************************************************************/
bool smtc_real_is_tx_param_setup_req_supported( smtc_real_t* real )
{
    return ( real_const.const_tx_param_setup_req_supported );
}

bool smtc_real_is_new_channel_req_supported( smtc_real_t* real )
{
    return ( real_const.const_new_channel_req_supported );
}

uint8_t smtc_real_get_rx1_join_delay( smtc_real_t* real )
{
    return ( real_const.const_join_accept_delay1 );
}

uint8_t smtc_real_get_rx2_join_dr( smtc_real_t* real )
{
    return ( real_const.const_rx2_dr_init );
}

uint8_t smtc_real_get_frequency_factor( smtc_real_t* real )
{
    return ( real_const.const_frequency_factor );
}

ral_lora_cr_t smtc_real_get_coding_rate( smtc_real_t* real )
{
    return ( real_const.const_coding_rate );
}

uint8_t smtc_real_get_adr_ack_delay( smtc_real_t* real )
{
    return ( real_const.const_adr_ack_delay );
}

uint8_t smtc_real_get_adr_ack_limit( smtc_real_t* real )
{
    return ( real_const.const_adr_ack_limit );
}

uint8_t smtc_real_get_public_sync_word( smtc_real_t* real )
{
    return ( real_const.const_sync_word_public );
}

uint8_t smtc_real_get_private_sync_word( smtc_real_t* real )
{
    return ( real_const.const_sync_word_private );
}

uint8_t smtc_real_get_sync_word( smtc_real_t* real )
{
    return ( real_ctx.sync_word_ctx );
}

void smtc_real_set_sync_word( smtc_real_t* real, uint8_t sync_word )
{
    real_ctx.sync_word_ctx = sync_word;
}

uint8_t* smtc_real_get_gfsk_sync_word( smtc_real_t* real )
{
#if defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_CN_470 ) || \
    defined( REGION_CN_470_RP_1_0 ) || defined( REGION_IN_865 ) || defined( REGION_RU_864 )
    return ( uint8_t* ) real_const.const_sync_word_gfsk;
#endif
    smtc_modem_hal_lr1mac_panic( );
    return 0;  // never reach => avoid warning
}

uint8_t* smtc_real_get_lr_fhss_sync_word( smtc_real_t* real )
{
#if defined( REGION_EU_868 ) || defined( REGION_AU_915 ) || defined( REGION_US_915 )
    return ( uint8_t* ) real_const.const_sync_word_lr_fhss;
#endif
    smtc_modem_hal_lr1mac_panic( );
    return 0;  // never reach => avoid warning
}

bool smtc_real_is_dtc_supported( const smtc_real_t* real )
{
    return real_const.const_dtc_supported;
}

bool smtc_real_is_lbt_supported( const smtc_real_t* real )
{
    return real_const.const_lbt_supported;
}

uint32_t smtc_real_get_lbt_duration_ms( const smtc_real_t* real )
{
    if( real_const.const_lbt_supported == true )
    {
        return real_const.const_lbt_sniff_duration_ms;
    }
    smtc_modem_hal_lr1mac_panic( );
    return 0;  // never reach => avoid warning
}

int16_t smtc_real_get_lbt_threshold_dbm( const smtc_real_t* real )
{
    if( real_const.const_lbt_supported == true )
    {
        return real_const.const_lbt_threshold_dbm;
    }
    smtc_modem_hal_lr1mac_panic( );
    return 0;  // never reach => avoid warning
}
uint32_t smtc_real_get_lbt_bw_hz( const smtc_real_t* real )
{
    if( real_const.const_lbt_supported == true )
    {
        return real_const.const_lbt_bw_hz;
    }
    smtc_modem_hal_lr1mac_panic( );
    return 0;  // never reach => avoid warning
}
uint8_t smtc_real_get_max_payload_size( smtc_real_t* real, uint8_t dr, direction_frame_t direction_frame )
{
    bool dwell_time_enabled = ( direction_frame == UP_LINK ) ? uplink_dwell_time_ctx : downlink_dwell_time_ctx;

    if( ( real_const.const_tx_param_setup_req_supported == false ) && ( dwell_time_enabled != 0 ) )
    {
        smtc_modem_hal_lr1mac_panic( );
    }

    uint8_t index = ( dwell_time_enabled * real_const.const_number_of_tx_dr ) + dr;

#if defined( REGION_AU_915 )
    if( real->region_type == SMTC_REAL_REGION_AU_915 )
    {
        // *2 because the array contains Tx and Rx datarate
        index = ( dwell_time_enabled * real_const.const_number_of_tx_dr * 2 ) + dr;
    }
#endif

    return ( real_const.const_max_payload_m[index] );
}

uint8_t smtc_real_get_default_max_eirp( smtc_real_t* real )
{
    return ( real_const.const_tx_power_dbm );
}

uint8_t smtc_real_get_beacon_dr( smtc_real_t* real )
{
    return ( real_const.const_beacon_dr );
}

bool smtc_real_is_beacon_hopping( smtc_real_t* real )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_IN_865 ) || \
    defined( REGION_RU_864 ) || defined( SMTC_REAL_REGION_KR_920 )
    {
        return false;
        break;
    }
#endif

#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915:
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915:
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470:
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0:
#endif
#if defined( REGION_US_915 ) || defined( REGION_AU_915 ) || defined( REGION_CN_470 ) || defined( REGION_CN_470_RP_1_0 )
    {
        return true;
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
    return 0;  // never reach => avoid warning
}

uint32_t smtc_real_get_beacon_frequency( smtc_real_t* real, uint32_t gps_time_s )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_IN_865 ) || \
    defined( REGION_RU_864 ) || defined( SMTC_REAL_REGION_KR_920 )
    {
        return real_const.const_beacon_frequency;
        break;
    }
#endif

#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_get_rx_beacon_frequency_channel( real, gps_time_s );
        break;
    }
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915: {
        return region_au_915_get_rx_beacon_frequency_channel( real, gps_time_s );
        break;
    }
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470: {
        return region_cn_470_get_rx_beacon_frequency_channel( real, gps_time_s );
        break;
    }
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0: {
        return region_cn_470_rp_1_0_get_rx_beacon_frequency_channel( real, gps_time_s );
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
    return 0;  // never reach => avoid warning
}

uint32_t smtc_real_get_ping_slot_frequency( smtc_real_t* real, uint32_t gps_time_s, uint32_t dev_addr )
{
    switch( real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
#endif
#if defined( REGION_AS_923 )
    case SMTC_REAL_REGION_AS_923:
    case SMTC_REAL_REGION_AS_923_GRP2:
    case SMTC_REAL_REGION_AS_923_GRP3:
#if defined( RP2_103 )
    case SMTC_REAL_REGION_AS_923_GRP4:
#endif
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865:
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864:
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920:
#endif
#if defined( REGION_WW2G4 ) || defined( REGION_EU_868 ) || defined( REGION_AS_923 ) || defined( REGION_IN_865 ) || \
    defined( REGION_RU_864 ) || defined( SMTC_REAL_REGION_KR_920 )
    {
        return real_const.const_ping_slot_frequency;
        break;
    }
#endif

#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_get_rx_ping_slot_frequency_channel( real, gps_time_s, dev_addr );
        break;
    }
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915: {
        return region_au_915_get_rx_ping_slot_frequency_channel( real, gps_time_s, dev_addr );
        break;
    }
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470: {
        return region_cn_470_get_rx_ping_slot_frequency_channel( real, gps_time_s, dev_addr );
        break;
    }
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0: {
        return region_cn_470_rp_1_0_get_rx_ping_slot_frequency_channel( real, gps_time_s, dev_addr );
        break;
    }
#endif
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
    return 0;  // never reach => avoid warning
}

uint8_t smtc_real_get_ping_slot_datarate( smtc_real_t* real )
{
    return real_const.const_beacon_dr;  // Return the default ping slot datarate, it's the same for beacon and ping-slot
}

void smtc_real_set_uplink_dwell_time( smtc_real_t* real, bool dwell_time )
{
    uplink_dwell_time_ctx = dwell_time;
}

void smtc_real_set_downlink_dwell_time( smtc_real_t* real, bool dwell_time )
{
    downlink_dwell_time_ctx = dwell_time;
}

uint32_t smtc_real_decode_freq_from_buf( smtc_real_t* real, uint8_t freq_buf[3] )
{
    uint32_t freq = ( freq_buf[0] ) + ( freq_buf[1] << 8 ) + ( freq_buf[2] << 16 );
    freq *= real_const.const_frequency_factor;
    return freq;
}

status_lorawan_t smtc_real_is_frequency_valid( smtc_real_t* real, uint32_t frequency )
{
    status_lorawan_t status = OKLORAWAN;
    if( ( frequency > real_const.const_freq_max ) || ( frequency < real_const.const_freq_min ) )
    {
        status = ERRORLORAWAN;
        SMTC_MODEM_HAL_TRACE_WARNING( "INVALID FREQUENCY = %d\n", frequency );
    }
    return ( status );
}

status_lorawan_t smtc_real_is_tx_power_valid( smtc_real_t* real, uint8_t power )
{
    status_lorawan_t status = OKLORAWAN;
    if( ( power > real_const.const_max_tx_power_idx ) )
    {
        status = ERRORLORAWAN;
        SMTC_MODEM_HAL_TRACE_WARNING( "RECEIVE AN INVALID Power Cmd = %d\n", power );
    }
    return ( status );
}

lr1mac_version_t smtc_real_get_regional_parameters_version( void )
{
    lr1mac_version_t version = {
        .major = RP_VERSION_MAJOR, .minor = RP_VERSION_MINOR, .patch = RP_VERSION_PATCH, .revision = RP_VERSION_REVISION
    };
    return version;
}

uint32_t smtc_real_get_symbol_duration_us( smtc_real_t* real, uint8_t datarate )
{
    modulation_type_t modulation_type = smtc_real_get_modulation_type_from_datarate( real, datarate );
    uint32_t          bw_temp         = 125;
    if( modulation_type == LORA )
    {
        uint8_t            sf;
        lr1mac_bandwidth_t bw;
        smtc_real_lora_dr_to_sf_bw( real, datarate, &sf, &bw );
        // Use lr1mac_utilities_get_symb_time_us
        switch( bw )
        {
        case BW125:
            bw_temp = 125;
            break;
        case BW250:
            bw_temp = 250;
            break;
        case BW500:
            bw_temp = 500;
            break;
        case BW800:
            bw_temp = 800;
            break;
        default:
            smtc_modem_hal_mcu_panic( " invalid BW " );
            break;
        }
        return ( ( ( uint32_t )( ( 1 << sf ) * 1000 ) / bw_temp ) );
    }
    else
    {
        uint8_t kbitrate;
        smtc_real_fsk_dr_to_bitrate( real, datarate, &kbitrate );
        return ( 8000 / ( kbitrate ) );  // 1 symbol equals 1 byte
    }
}

void smtc_real_get_rx_window_parameters( smtc_real_t* real, uint8_t datarate, uint32_t rx_delay_ms,
                                         uint16_t* rx_window_symb, uint32_t* rx_timeout_symb_in_ms,
                                         uint32_t* rx_timeout_preamble_locked_in_ms, uint8_t rx_done_incertitude,
                                         uint32_t crystal_error )
{
    uint32_t          tsymbol_us              = smtc_real_get_symbol_duration_us( real, datarate );
    uint32_t          min_rx_symb_duration_ms = MIN_RX_WINDOW_DURATION_MS + rx_done_incertitude;
    modulation_type_t modulation_type         = smtc_real_get_modulation_type_from_datarate( real, datarate );

    if( modulation_type == FSK )
    {
        min_rx_symb_duration_ms += 2;
    }

    *rx_timeout_symb_in_ms =
        MAX( ( ( ( ( rx_delay_ms * 2 * crystal_error ) / 1000 ) + ( MIN_RX_WINDOW_SYMB * tsymbol_us ) ) / 1000 ),
             min_rx_symb_duration_ms );

    *rx_window_symb =
        MIN( MAX( ( ( *rx_timeout_symb_in_ms * 1000 ) / tsymbol_us ), MIN_RX_WINDOW_SYMB ), MAX_RX_WINDOW_SYMB );

    // Because the hardware allows an even number of symbols
    if( ( ( *rx_window_symb % 2 ) == 1 ) && ( *rx_window_symb != MAX_RX_WINDOW_SYMB ) )  //
    {
        *rx_window_symb = *rx_window_symb + 1;
    }

    *rx_timeout_symb_in_ms = MAX( ( *rx_window_symb * tsymbol_us ) / 1000, MIN_RX_WINDOW_DURATION_MS );

    *rx_timeout_preamble_locked_in_ms = 3000;

#if defined( SX128X )
    // rx timeout is used to simuate a symb timeout in sx128x (need to open preamb + sync +header)
    *rx_timeout_preamble_locked_in_ms =
        MAX( ceilf( ( ( ( float ) *rx_window_symb + 16.25f ) * tsymbol_us ) ) / 1000, MIN_RX_WINDOW_DURATION_MS );
    *rx_timeout_symb_in_ms = *rx_timeout_preamble_locked_in_ms;
#endif
}

void smtc_real_get_rx_start_time_offset_ms( smtc_real_t* real, uint8_t datarate, int8_t board_delay_ms,
                                            uint16_t rx_window_symb, int32_t* rx_offset_ms )
{
    modulation_type_t modulation_type = smtc_real_get_modulation_type_from_datarate( real, datarate );
    int32_t           tsymbol_us      = ( int32_t ) smtc_real_get_symbol_duration_us( real, datarate );

    if( modulation_type == FSK )
    {
        *rx_offset_ms = ( ( tsymbol_us * -1 * ( rx_window_symb / 2 ) ) / 1000 ) - ( int32_t ) board_delay_ms;
    }
    else
    {
        *rx_offset_ms =
            ( ( tsymbol_us * ( 1 - ( ( ( int32_t ) rx_window_symb - MIN_RX_WINDOW_SYMB ) / 2 ) ) ) / 1000 ) -
            ( int32_t ) board_delay_ms;
    }
    // SMTC_MODEM_HAL_TRACE_PRINTF(
    //    "rx_start_target -> datarate:%d, rx_window_symb:%u, rx_offset_ms:%d, board_delay_ms:%d\n", datarate,
    //   rx_window_symb, *rx_offset_ms, board_delay_ms );
}