/*!
 * @file      lr20xx_workarounds.c
 *
 * @brief     System driver workaround implementation for LR20XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
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

#include <stdbool.h>
#include "lr20xx_workarounds.h"
#include "lr20xx_hal.h"
#include "lr20xx_regmem.h"
#include "lr20xx_radio_fsk_common_types.h"
#include "lr20xx_system.h"
#include "lr20xx_radio_ook.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR20XX_WORKAROUND_BLUETOOTH_LE_PHY_CODED_SYNCWORDS ( 7 )

#define LR20XX_WORKAROUND_LORA_SX1276_COMPATIBILITY_REGISTER_ADDRESS ( 0x00F30A14 )
#define LR20XX_WORKAROUND_LORA_SX1276_COMPATIBILITY_REGISTER_MASK ( 3 << 18 )

#define LR20XX_WORKAROUND_LORA_FREQ_HOP_SX1276_COMPATIBILITY_REGISTER_ADDRESS ( 0x00F30A24 )
#define LR20XX_WORKAROUND_LORA_FREQ_HOP_SX1276_COMPATIBILITY_REGISTER_MASK ( 1 << 18 )

#define LR20XX_WORKAROUND_OOK_DETECTION_THRESHOLD_REGISTER_ADDRESS ( 0x00F30E14 )
#define LR20XX_WORKAROUND_OOK_DETECTION_THRESHOLD_REGISTER_MASK ( 0x7F << 20 )

#define LR20XX_WORKAROUND_RTTOF_RF_FREQ_ADDRESS ( 0x00F40144 )
#define LR20XX_WORKAROUND_RTTOF_RF_FREQ_MASK ( 0x3F )

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

/**
 * @brief Helper function to write the appropriate field to store LoRa SX1276 compatibility parameter
 *
 * @param context Chip implementation context
 * @param value True to enable the compatibility mode, false to disable it
 * @return Operation status
 */
static lr20xx_status_t lr20xx_workaround_lora_sx1276_compatibility_write_value( const void* context, bool value );

/**
 * @brief Helper function to write the appropriate field to store LoRa frequency hopping SX1276 compatibility parameter
 *
 * @param context Chip implementation context
 * @param value True to enable the compatibility mode, false to disable it
 * @return Operation status
 */
static lr20xx_status_t lr20xx_workaround_lora_frequency_hopping_sx1276_compatibility_write_value( const void* context,
                                                                                                  bool        value );

/**
 * @brief Read the configured SF value configured
 *
 * This command is to be used only when disabling the SX1276 LoRa compatibility mode.
 *
 * @param context Chip implementation context
 * @param [out] sf The configure SF
 * @return Operation status
 */
static lr20xx_status_t lr20xx_workaround_lora_sx1276_compatibility_read_sf_value( const void* context, uint8_t* sf );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr20xx_status_t lr20xx_workarounds_bluetooth_le_phy_coded_syncwords( const void* context )
{
    const uint8_t cbuffer[LR20XX_WORKAROUND_BLUETOOTH_LE_PHY_CODED_SYNCWORDS] = { 0x02, 0x30, 0x01, 0x20,
                                                                                  0x00, 0x09, 0x00 };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_WORKAROUND_BLUETOOTH_LE_PHY_CODED_SYNCWORDS,
                                                 0, 0 );
}

lr20xx_status_t lr20xx_workarounds_lora_enable_sx1276_compatibility_mode( const void* context )
{
    return lr20xx_workaround_lora_sx1276_compatibility_write_value( context, true );
}

lr20xx_status_t lr20xx_workarounds_lora_disable_sx1276_compatibility_mode( const void* context )
{
    // 1. Get the currently configured SF value
    uint8_t               sf            = 0;
    const lr20xx_status_t get_sf_status = lr20xx_workaround_lora_sx1276_compatibility_read_sf_value( context, &sf );

    // 2. Modify the compatibility mode value depending on currently configured SF
    if( get_sf_status == LR20XX_STATUS_OK )
    {
        return lr20xx_workaround_lora_sx1276_compatibility_write_value( context, ( ( sf <= 6 ) ? true : false ) );
    }
    else
    {
        return get_sf_status;
    }
}

lr20xx_status_t lr20xx_workarounds_lora_sx1276_compatibility_mode_store_retention_mem( const void* context,
                                                                                       uint8_t     slot )
{
    return lr20xx_system_add_register_to_retention_mem( context, slot,
                                                        LR20XX_WORKAROUND_LORA_SX1276_COMPATIBILITY_REGISTER_ADDRESS );
}

lr20xx_status_t lr20xx_workarounds_lora_freq_hop_enable_sx1276_compatiblity_mode( const void* context )
{
    return lr20xx_workaround_lora_frequency_hopping_sx1276_compatibility_write_value( context, true );
}

lr20xx_status_t lr20xx_workarounds_lora_freq_hop_disable_sx1276_compatiblity_mode( const void* context )
{
    return lr20xx_workaround_lora_frequency_hopping_sx1276_compatibility_write_value( context, false );
}

lr20xx_status_t lr20xx_workarounds_lora_freq_hop_sx1276_compatiblity_mode_store_retention_mem( const void* context,
                                                                                               uint8_t     slot )
{
    return lr20xx_system_add_register_to_retention_mem(
        context, slot, LR20XX_WORKAROUND_LORA_FREQ_HOP_SX1276_COMPATIBILITY_REGISTER_ADDRESS );
}

lr20xx_status_t lr20xx_workarounds_ook_set_detection_threshold_level( const void* context, int16_t threshold_level_db )
{
    const int threshold_db = threshold_level_db + 10 + 64;
    return lr20xx_regmem_write_regmem32_mask(
        context, LR20XX_WORKAROUND_OOK_DETECTION_THRESHOLD_REGISTER_ADDRESS,
        LR20XX_WORKAROUND_OOK_DETECTION_THRESHOLD_REGISTER_MASK,
        ( ( ( uint32_t ) threshold_db ) << 20u ) & LR20XX_WORKAROUND_OOK_DETECTION_THRESHOLD_REGISTER_MASK );
}

int16_t lr20xx_workarounds_ook_get_default_detection_threshold_level( lr20xx_radio_fsk_common_bw_t bw )
{
    switch( bw )
    {
    case LR20XX_RADIO_FSK_COMMON_RX_BW_3_500_HZ:
    {
        return -135;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_4_200_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_4_300_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_4_500_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_4_800_HZ:
    {
        return -134;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_5_200_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_5_600_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_5_800_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_6_000_HZ:
    {
        return -133;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_6_900_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_7_400_HZ:
    {
        return -132;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_8_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_8_300_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_8_700_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_8_900_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_9_600_HZ:
    {
        return -131;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_10_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_11_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_12_000_HZ:
    {
        return -130;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_13_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_14_000_HZ:
    {
        return -129;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_16_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_17_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_19_000_HZ:
    {
        return -128;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_20_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_22_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_23_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_24_000_HZ:
    {
        return -127;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_27_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_29_000_HZ:
    {
        return -126;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_32_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_33_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_34_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_35_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_38_000_HZ:
    {
        return -125;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_41_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_44_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_46_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_48_000_HZ:
    {
        return -124;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_55_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_59_000_HZ:
    {
        return -123;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_64_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_66_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_69_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_71_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_76_000_HZ:
    {
        return -122;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_83_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_89_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_92_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_96_000_HZ:
    {
        return -121;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_111_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_119_000_HZ:
    {
        return -120;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_128_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_133_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_138_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_142_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_153_000_HZ:
    {
        return -119;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_166_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_178_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_185_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_192_000_HZ:
    {
        return -118;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_222_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_238_000_HZ:
    {
        return -117;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_256_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_266_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_277_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_285_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_307_000_HZ:
    {
        return -116;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_333_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_357_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_370_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_384_000_HZ:
    {
        return -115;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_444_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_476_000_HZ:
    {
        return -114;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_512_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_533_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_555_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_571_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_615_000_HZ:
    {
        return -113;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_666_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_714_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_740_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_769_000_HZ:
    {
        return -112;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_888_000_HZ:
    {
        return -111;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_1_111_000_HZ:
    {
        return -110;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_1_333_000_HZ:
    {
        return -109;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_2_222_000_HZ:
    {
        return -107;
    }
    case LR20XX_RADIO_FSK_COMMON_RX_BW_2_666_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_2_857_000_HZ:
    case LR20XX_RADIO_FSK_COMMON_RX_BW_3_076_000_HZ:
    {
        return -106;
    }
    default:
    {
        return 0;
    }
    }
}

lr20xx_status_t lr20xx_workarounds_rttof_truncate_pll_freq_step( const void* context )
{
    return lr20xx_regmem_write_regmem32_mask( context, LR20XX_WORKAROUND_RTTOF_RF_FREQ_ADDRESS,
                                              LR20XX_WORKAROUND_RTTOF_RF_FREQ_MASK, 0 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

lr20xx_status_t lr20xx_workaround_lora_sx1276_compatibility_write_value( const void* context, bool value )
{
    return lr20xx_regmem_write_regmem32_mask( context, LR20XX_WORKAROUND_LORA_SX1276_COMPATIBILITY_REGISTER_ADDRESS,
                                              LR20XX_WORKAROUND_LORA_SX1276_COMPATIBILITY_REGISTER_MASK,
                                              ( value ? ( 1 << 19 ) : 0 ) );
}

lr20xx_status_t lr20xx_workaround_lora_frequency_hopping_sx1276_compatibility_write_value( const void* context,
                                                                                           bool        value )
{
    return lr20xx_regmem_write_regmem32_mask(
        context, LR20XX_WORKAROUND_LORA_FREQ_HOP_SX1276_COMPATIBILITY_REGISTER_ADDRESS,
        LR20XX_WORKAROUND_LORA_FREQ_HOP_SX1276_COMPATIBILITY_REGISTER_MASK, ( value ? ( 1 << 18 ) : 0 ) );
}

lr20xx_status_t lr20xx_workaround_lora_sx1276_compatibility_read_sf_value( const void* context, uint8_t* sf )
{
    uint32_t              raw_register_value = 0;
    const lr20xx_status_t read_status        = lr20xx_regmem_read_regmem32(
               context, LR20XX_WORKAROUND_LORA_SX1276_COMPATIBILITY_REGISTER_ADDRESS, &raw_register_value, 1 );
    if( read_status == LR20XX_STATUS_OK )
    {
        *sf = raw_register_value & 0x0f;
    }
    return read_status;
}

/* --- EOF ------------------------------------------------------------------ */
