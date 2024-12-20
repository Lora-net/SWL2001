/*!
 * \file    relay_real.c
 *
 * \brief   Relay regional parameter function
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
#include <stdint.h>
#include <stdbool.h>

#include "relay_real.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_duty_cycle.h"
#include "smtc_modem_hal.h"
#if defined( REGION_AS_923 )
/**
 * @brief Return default value for relay channel for AS_923
 *
 * @param[in]   idx         Index of the chanel (0 or 1)
 * @param[out]  datarate    Datarate for the channel index idx
 * @param[out]  freq_wor    Frequency in Hz for WOR
 * @param[out]  freq_ack    Frequency in Hz for WOR ACK
 * @return OKLORAWAN a valid config exist for this channel
 * @return ERRORLORAWAN no valid config exist for this channel
 */
static status_lorawan_t region_as_923_relay_get_default( uint8_t idx, uint8_t* datarate, uint32_t* freq_wor,
                                                         uint32_t* freq_ack )
{
    switch( idx )
    {
    case 0:
        *freq_wor = 923600000;
        *freq_ack = 923800000;
        break;

    default:
        return ERRORLORAWAN;
        break;
    }

    *datarate = 3;
    return OKLORAWAN;
}
#endif

#if defined( REGION_AU_915 )
/**
 * @brief Return default value for relay channel for AU_915
 *
 * @param[in]   idx         Index of the chanel (0 or 1)
 * @param[out]  datarate    Datarate for the channel index idx
 * @param[out]  freq_wor    Frequency in Hz for WOR
 * @param[out]  freq_ack    Frequency in Hz for WOR ACK
 * @return OKLORAWAN a valid config exist for this channel
 * @return ERRORLORAWAN no valid config exist for this channel
 */
static status_lorawan_t region_au_915_relay_get_default( uint8_t idx, uint8_t* datarate, uint32_t* freq_wor,
                                                         uint32_t* freq_ack )
{
    switch( idx )
    {
    case 0:
        *freq_wor = 916700000;
        *freq_ack = 918300000;
        break;

    case 1:
        *freq_wor = 919900000;
        *freq_ack = 921500000;
        break;

    default:
        return ERRORLORAWAN;
        break;
    }

    *datarate = 10;
    return OKLORAWAN;
}
#endif

#if defined( REGION_CN_470_RP_1_0 )
/**
 * @brief Return default value for relay channel for CN_470_RP_1_0
 *
 * @param[in]   idx         Index of the chanel (0 or 1)
 * @param[out]  datarate    Datarate for the channel index idx
 * @param[out]  freq_wor    Frequency in Hz for WOR
 * @param[out]  freq_ack    Frequency in Hz for WOR ACK
 * @return OKLORAWAN a valid config exist for this channel
 * @return ERRORLORAWAN no valid config exist for this channel
 */
static status_lorawan_t region_cn_470_rp_1_0_relay_get_default( uint8_t idx, uint8_t* datarate, uint32_t* freq_wor,
                                                                uint32_t* freq_ack )
{
    switch( idx )
    {
    case 0:
        *freq_wor = 472100000;
        *freq_ack = 485300000;
        break;

    case 1:
        *freq_wor = 494900000;
        *freq_ack = 505500000;
        break;

    default:
        return ERRORLORAWAN;
        break;
    }

    *datarate = 3;
    return OKLORAWAN;
}
#endif

#if defined( REGION_CN_470 )
/**
 * @brief Return default value for relay channel for CN_470
 *
 * @param[in]   idx         Index of the chanel (0 or 1)
 * @param[out]  datarate    Datarate for the channel index idx
 * @param[out]  freq_wor    Frequency in Hz for WOR
 * @param[out]  freq_ack    Frequency in Hz for WOR ACK
 * @return OKLORAWAN a valid config exist for this channel
 * @return ERRORLORAWAN no valid config exist for this channel
 */
static status_lorawan_t region_cn_470_relay_get_default( uint8_t idx, uint8_t* datarate, uint32_t* freq_wor,
                                                         uint32_t* freq_ack )
{
    switch( idx )
    {
    case 0:
        *freq_wor = 472100000;
        *freq_ack = 485300000;
        break;

    case 1:
        *freq_wor = 494900000;
        *freq_ack = 505500000;
        break;

    default:
        return ERRORLORAWAN;
        break;
    }

    *datarate = 3;
    return OKLORAWAN;
}
#endif

#if defined( REGION_IN_865 )
/**
 * @brief Return default value for relay channel for IN_865
 *
 * @param[in]   idx         Index of the chanel (0 or 1)
 * @param[out]  datarate    Datarate for the channel index idx
 * @param[out]  freq_wor    Frequency in Hz for WOR
 * @param[out]  freq_ack    Frequency in Hz for WOR ACK
 * @return OKLORAWAN a valid config exist for this channel
 * @return ERRORLORAWAN no valid config exist for this channel
 */
static status_lorawan_t region_in_865_relay_get_default( uint8_t idx, uint8_t* datarate, uint32_t* freq_wor,
                                                         uint32_t* freq_ack )
{
    switch( idx )
    {
    case 0:
        *freq_wor = 866000000;
        *freq_ack = 866200000;
        break;

    case 1:
        *freq_wor = 866700000;
        *freq_ack = 866900000;
        break;

    default:
        return ERRORLORAWAN;
        break;
    }

    *datarate = 3;
    return OKLORAWAN;
}
#endif

#if defined( REGION_KR_920 )
/**
 * @brief Return default value for relay channel for KR_920
 *
 * @param[in]   idx         Index of the chanel (0 or 1)
 * @param[out]  datarate    Datarate for the channel index idx
 * @param[out]  freq_wor    Frequency in Hz for WOR
 * @param[out]  freq_ack    Frequency in Hz for WOR ACK
 * @return OKLORAWAN a valid config exist for this channel
 * @return ERRORLORAWAN no valid config exist for this channel
 */
static status_lorawan_t region_kr_920_relay_get_default( uint8_t idx, uint8_t* datarate, uint32_t* freq_wor,
                                                         uint32_t* freq_ack )
{
    switch( idx )
    {
    case 0:
        *freq_wor = 922700000;
        *freq_ack = 922900000;
        break;

    case 1:
        *freq_wor = 923100000;
        *freq_ack = 923300000;
        break;

    default:
        return ERRORLORAWAN;
        break;
    }

    *datarate = 3;
    return OKLORAWAN;
}
#endif

#if defined( REGION_RU_864 )
/**
 * @brief Return default value for relay channel for RU_864
 *
 * @param[in]   idx         Index of the chanel (0 or 1)
 * @param[out]  datarate    Datarate for the channel index idx
 * @param[out]  freq_wor    Frequency in Hz for WOR
 * @param[out]  freq_ack    Frequency in Hz for WOR ACK
 * @return OKLORAWAN a valid config exist for this channel
 * @return ERRORLORAWAN no valid config exist for this channel
 */
static status_lorawan_t region_ru_864_relay_get_default( uint8_t idx, uint8_t* datarate, uint32_t* freq_wor,
                                                         uint32_t* freq_ack )
{
    switch( idx )
    {
    case 0:
        *freq_wor = 866100000;
        *freq_ack = 866300000;
        break;

    case 1:
        *freq_wor = 866500000;
        *freq_ack = 866900000;
        break;

    default:
        return ERRORLORAWAN;
        break;
    }
    *datarate = 3;

    return OKLORAWAN;
}
#endif

#if defined( REGION_US_915 )
/**
 * @brief Return default value for relay channel for US_915
 *
 * @param[in]   idx         Index of the chanel (0 or 1)
 * @param[out]  datarate    Datarate for the channel index idx
 * @param[out]  freq_wor    Frequency in Hz for WOR
 * @param[out]  freq_ack    Frequency in Hz for WOR ACK
 * @return OKLORAWAN a valid config exist for this channel
 * @return ERRORLORAWAN no valid config exist for this channel
 */
static status_lorawan_t region_us_915_relay_get_default( uint8_t idx, uint8_t* datarate, uint32_t* freq_wor,
                                                         uint32_t* freq_ack )
{
    switch( idx )
    {
    case 0:
        *freq_wor = 916700000;
        *freq_ack = 918300000;
        break;

    case 1:
        *freq_wor = 919900000;
        *freq_ack = 921500000;
        break;

    default:
        return ERRORLORAWAN;
        break;
    }

    *datarate = 10;

    return OKLORAWAN;
}
#endif

#if defined( REGION_EU_868 )
/**
 * @brief Return default value for relay channel for EU_868
 *
 * @param[in]   idx         Index of the chanel (0 or 1)
 * @param[out]  datarate    Datarate for the channel index idx
 * @param[out]  freq_wor    Frequency in Hz for WOR
 * @param[out]  freq_ack    Frequency in Hz for WOR ACK
 * @return OKLORAWAN a valid config exist for this channel
 * @return ERRORLORAWAN no valid config exist for this channel
 */
static status_lorawan_t region_eu_868_relay_get_default( uint8_t idx, uint8_t* datarate, uint32_t* freq_wor,
                                                         uint32_t* freq_ack )
{
    switch( idx )
    {
    case 0:
        *freq_wor = 865100000;
        *freq_ack = 865300000;
        break;

    case 1:
        *freq_wor = 865500000;
        *freq_ack = 865900000;
        break;

    default:
        return ERRORLORAWAN;
        break;
    }
    *datarate = 3;

    return OKLORAWAN;
}
#endif

#if defined( REGION_WW2G4 )
/**
 * @brief Return default value for relay channel for WW2G4
 *
 * @param[in]   idx         Index of the chanel (0 or 1)
 * @param[out]  datarate    Datarate for the channel index idx
 * @param[out]  freq_wor    Frequency in Hz for WOR
 * @param[out]  freq_ack    Frequency in Hz for WOR ACK
 * @return OKLORAWAN a valid config exist for this channel
 * @return ERRORLORAWAN no valid config exist for this channel
 */
static status_lorawan_t region_ww2g4_relay_get_default( uint8_t idx, uint8_t* datarate, uint32_t* freq_wor,
                                                        uint32_t* freq_ack )
{
    switch( idx )
    {
    case 0:
        *freq_wor = 2423000000;
        *freq_ack = 2477000000;
        break;

    case 1:
        *freq_wor = 2423000000;
        *freq_ack = 2477000000;
        break;

    default:
        return ERRORLORAWAN;
        break;
    }

    *datarate = 3;
    return OKLORAWAN;
}
#endif

status_lorawan_t smtc_relay_get_default_channel_config( const smtc_real_t* real, const uint8_t idx, uint8_t* datarate,
                                                        uint32_t* freq_wor, uint32_t* freq_ack )
{
    switch( real->region_type )
    {
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_relay_get_default( idx, datarate, freq_wor, freq_ack );
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
        const status_lorawan_t status = region_as_923_relay_get_default( idx, datarate, freq_wor, freq_ack );
        *freq_wor += real->real_const.const_frequency_offset_hz;
        *freq_ack += real->real_const.const_frequency_offset_hz;
        return status;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_relay_get_default( idx, datarate, freq_wor, freq_ack );
    }
#endif
#if defined( REGION_AU_915 )
    case SMTC_REAL_REGION_AU_915: {
        return region_au_915_relay_get_default( idx, datarate, freq_wor, freq_ack );
    }
#endif
#if defined( REGION_CN_470 )
    case SMTC_REAL_REGION_CN_470: {
        return region_cn_470_relay_get_default( idx, datarate, freq_wor, freq_ack );
    }
#endif
#if defined( REGION_CN_470_RP_1_0 )
    case SMTC_REAL_REGION_CN_470_RP_1_0: {
        return region_cn_470_rp_1_0_relay_get_default( idx, datarate, freq_wor, freq_ack );
    }
#endif
#if defined( REGION_IN_865 )
    case SMTC_REAL_REGION_IN_865: {
        return region_in_865_relay_get_default( idx, datarate, freq_wor, freq_ack );
    }
#endif
#if defined( REGION_KR_920 )
    case SMTC_REAL_REGION_KR_920: {
        return region_kr_920_relay_get_default( idx, datarate, freq_wor, freq_ack );
    }
#endif
#if defined( REGION_RU_864 )
    case SMTC_REAL_REGION_RU_864: {
        return region_ru_864_relay_get_default( idx, datarate, freq_wor, freq_ack );
    }
#endif

#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_relay_get_default( idx, datarate, freq_wor, freq_ack );
        break;
    }
#endif
    default:
        SMTC_MODEM_HAL_PANIC( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}