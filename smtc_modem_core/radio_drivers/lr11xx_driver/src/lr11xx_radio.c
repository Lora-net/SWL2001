/*!
 * @file      lr11xx_radio.c
 *
 * @brief     Radio driver implementation for LR11XX
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

#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "lr11xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Management of the high ACP workaround
 */
#ifdef LR11XX_DISABLE_HIGH_ACP_WORKAROUND
#define LR11XX_RADIO_APPLY_HIGH_ACP_WORKAROUND( context ) LR11XX_STATUS_OK
#else
#define LR11XX_RADIO_APPLY_HIGH_ACP_WORKAROUND( context ) lr11xx_radio_apply_high_acp_workaround( context )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR11XX_RADIO_RESET_STATS_CMD_LENGTH ( 2 )
#define LR11XX_RADIO_GET_STATS_CMD_LENGTH ( 2 )
#define LR11XX_RADIO_GET_PKT_TYPE_CMD_LENGTH ( 2 )
#define LR11XX_RADIO_GET_RXBUFFER_STATUS_CMD_LENGTH ( 2 )
#define LR11XX_RADIO_GET_PKT_STATUS_CMD_LENGTH ( 2 )
#define LR11XX_RADIO_GET_RSSI_INST_CMD_LENGTH ( 2 )
#define LR11XX_RADIO_SET_GFSK_SYNC_WORD_CMD_LENGTH ( 2 + LR11XX_RADIO_GFSK_SYNC_WORD_LENGTH )
#define LR11XX_RADIO_SET_LORA_PUBLIC_NETWORK_CMD_LENGTH ( 2 + 8 )
#define LR11XX_RADIO_SET_RX_CMD_LENGTH ( 2 + 3 )
#define LR11XX_RADIO_SET_TX_CMD_LENGTH ( 2 + 3 )
#define LR11XX_RADIO_SET_RF_FREQUENCY_CMD_LENGTH ( 2 + 4 )
#define LR11XX_RADIO_SET_AUTO_TX_RX_CMD_LENGTH ( 2 + 7 )
#define LR11XX_RADIO_SET_CAD_PARAMS_CMD_LENGTH ( 2 + 7 )
#define LR11XX_RADIO_SET_PKT_TYPE_CMD_LENGTH ( 2 + 1 )
#define LR11XX_RADIO_SET_MODULATION_PARAMS_GFSK_CMD_LENGTH ( 2 + 10 )
#define LR11XX_RADIO_SET_MODULATION_PARAMS_LORA_CMD_LENGTH ( 2 + 4 )
#define LR11XX_RADIO_SET_PKT_PARAM_GFSK_CMD_LENGTH ( 2 + 9 )
#define LR11XX_RADIO_SET_PKT_PARAM_LORA_CMD_LENGTH ( 2 + 6 )
#define LR11XX_RADIO_SET_TX_PARAMS_CMD_LENGTH ( 2 + 2 )
#define LR11XX_RADIO_SET_PKT_ADDRESS_CMD_LENGTH ( 2 + 2 )
#define LR11XX_RADIO_SET_RX_TX_FALLBACK_MODE_CMD_LENGTH ( 2 + 1 )
#define LR11XX_RADIO_SET_RX_DUTY_CYCLE_MODE_CMD_LENGTH ( 2 + 7 )
#define LR11XX_RADIO_SET_PA_CFG_CMD_LENGTH ( 2 + 4 )
#define LR11XX_RADIO_STOP_TIMEOUT_ON_PREAMBLE_CMD_LENGTH ( 2 + 1 )
#define LR11XX_RADIO_SET_CAD_CMD_LENGTH ( 2 )
#define LR11XX_RADIO_SET_TX_CW_CMD_LENGTH ( 2 )
#define LR11XX_RADIO_SET_TX_INFINITE_PREAMBLE_CMD_LENGTH ( 2 )
#define LR11XX_RADIO_SET_LORA_SYNC_TIMEOUT_CMD_LENGTH ( 2 + 1 )
#define LR11XX_RADIO_SET_GFSK_CRC_PARAMS_CMD_LENGTH ( 2 + 8 )
#define LR11XX_RADIO_SET_GFSK_WHITENING_CMD_LENGTH ( 2 + 2 )
#define LR11XX_RADIO_SET_RX_BOOSTED_LENGTH ( 2 + 1 )
#define LR11XX_RADIO_SET_RSSI_CALIBRATION_LENGTH ( 2 + 11 )
#define LR11XX_RADIO_SET_LORA_SYNC_WORD_CMD_LENGTH ( 2 + 1 )
#define LR11XX_RADIO_GET_LORA_RX_INFO_CMD_LENGTH ( 2 )

/**
 * @brief Internal RTC frequency
 */
#define LR11XX_RTC_FREQ_IN_HZ 32768UL

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for radio-related operations
 */
enum
{
    LR11XX_RADIO_RESET_STATS_OC               = 0x0200,
    LR11XX_RADIO_GET_STATS_OC                 = 0x0201,
    LR11XX_RADIO_GET_PKT_TYPE_OC              = 0x0202,
    LR11XX_RADIO_GET_RXBUFFER_STATUS_OC       = 0x0203,
    LR11XX_RADIO_GET_PKT_STATUS_OC            = 0x0204,
    LR11XX_RADIO_GET_RSSI_INST_OC             = 0x0205,
    LR11XX_RADIO_SET_GFSK_SYNC_WORD_OC        = 0x0206,
    LR11XX_RADIO_SET_LORA_PUBLIC_NETWORK_OC   = 0x0208,
    LR11XX_RADIO_SET_RX_OC                    = 0x0209,
    LR11XX_RADIO_SET_TX_OC                    = 0x020A,
    LR11XX_RADIO_SET_RF_FREQUENCY_OC          = 0x020B,
    LR11XX_RADIO_AUTOTXRX_OC                  = 0x020C,
    LR11XX_RADIO_SET_CAD_PARAMS_OC            = 0x020D,
    LR11XX_RADIO_SET_PKT_TYPE_OC              = 0x020E,
    LR11XX_RADIO_SET_MODULATION_PARAM_OC      = 0x020F,
    LR11XX_RADIO_SET_PKT_PARAM_OC             = 0x0210,
    LR11XX_RADIO_SET_TX_PARAMS_OC             = 0x0211,
    LR11XX_RADIO_SET_PKT_ADRS_OC              = 0x0212,
    LR11XX_RADIO_SET_RX_TX_FALLBACK_MODE_OC   = 0x0213,
    LR11XX_RADIO_SET_RX_DUTY_CYCLE_OC         = 0x0214,
    LR11XX_RADIO_SET_PA_CFG_OC                = 0x0215,
    LR11XX_RADIO_STOP_TIMEOUT_ON_PREAMBLE_OC  = 0x0217,
    LR11XX_RADIO_SET_CAD_OC                   = 0x0218,
    LR11XX_RADIO_SET_TX_CW_OC                 = 0x0219,
    LR11XX_RADIO_SET_TX_INFINITE_PREAMBLE_OC  = 0x021A,
    LR11XX_RADIO_SET_LORA_SYNC_TIMEOUT_OC     = 0x021B,
    LR11XX_RADIO_SET_GFSK_CRC_PARAMS_OC       = 0x0224,
    LR11XX_RADIO_SET_GFSK_WHITENING_PARAMS_OC = 0x0225,
    LR11XX_RADIO_SET_RX_BOOSTED_OC            = 0x0227,
    LR11XX_RADIO_SET_RSSI_CALIBRATION_OC      = 0x0229,
    LR11XX_RADIO_SET_LORA_SYNC_WORD_OC        = 0x022B,
    LR11XX_RADIO_GET_LORA_RX_INFO_OC          = 0x0230,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Get the CRC length in byte from the corresponding GFSK radio parameter
 *
 * @param [in] crc_type GFSK CRC parameter
 *
 * @returns CRC length in byte
 */
static inline uint32_t lr11xx_radio_get_gfsk_crc_len_in_bytes( lr11xx_radio_gfsk_crc_type_t crc_type );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_status_t lr11xx_radio_reset_stats( const void* context )
{
    const uint8_t cbuffer[LR11XX_RADIO_RESET_STATS_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_RESET_STATS_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_RESET_STATS_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_RESET_STATS_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_radio_get_gfsk_stats( const void* context, lr11xx_radio_stats_gfsk_t* stats )
{
    const uint8_t cbuffer[LR11XX_RADIO_GET_STATS_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_GET_STATS_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_GET_STATS_OC >> 0 ),
    };
    uint8_t rbuffer[sizeof( lr11xx_radio_stats_gfsk_t )] = { 0x00 };

    const lr11xx_status_t status = ( lr11xx_status_t ) lr11xx_hal_read(
        context, cbuffer, LR11XX_RADIO_GET_STATS_CMD_LENGTH, rbuffer, sizeof( lr11xx_radio_stats_gfsk_t ) );

    if( status == LR11XX_STATUS_OK )
    {
        stats->nb_pkt_received  = ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1];
        stats->nb_pkt_crc_error = ( ( uint16_t ) rbuffer[2] << 8 ) + ( uint16_t ) rbuffer[3];
        stats->nb_pkt_len_error = ( ( uint16_t ) rbuffer[4] << 8 ) + ( uint16_t ) rbuffer[5];
    }

    return status;
}

lr11xx_status_t lr11xx_radio_get_lora_stats( const void* context, lr11xx_radio_stats_lora_t* stats )
{
    const uint8_t cbuffer[LR11XX_RADIO_GET_STATS_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_GET_STATS_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_GET_STATS_OC >> 0 ),
    };
    uint8_t rbuffer[sizeof( lr11xx_radio_stats_lora_t )] = { 0x00 };

    const lr11xx_status_t status = ( lr11xx_status_t ) lr11xx_hal_read(
        context, cbuffer, LR11XX_RADIO_GET_STATS_CMD_LENGTH, rbuffer, sizeof( lr11xx_radio_stats_lora_t ) );

    if( status == LR11XX_STATUS_OK )
    {
        stats->nb_pkt_received     = ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1];
        stats->nb_pkt_crc_error    = ( ( uint16_t ) rbuffer[2] << 8 ) + ( uint16_t ) rbuffer[3];
        stats->nb_pkt_header_error = ( ( uint16_t ) rbuffer[4] << 8 ) + ( uint16_t ) rbuffer[5];
        stats->nb_pkt_falsesync    = ( ( uint16_t ) rbuffer[6] << 8 ) + ( uint16_t ) rbuffer[7];
    }

    return status;
}

lr11xx_status_t lr11xx_radio_get_pkt_type( const void* context, lr11xx_radio_pkt_type_t* pkt_type )
{
    const uint8_t cbuffer[LR11XX_RADIO_GET_PKT_TYPE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_GET_PKT_TYPE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_GET_PKT_TYPE_OC >> 0 ),
    };
    uint8_t pkt_type_raw = 0;

    const lr11xx_status_t status =
        ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_RADIO_GET_PKT_TYPE_CMD_LENGTH, &pkt_type_raw, 1 );

    if( status == LR11XX_STATUS_OK )
    {
        *pkt_type = ( lr11xx_radio_pkt_type_t ) pkt_type_raw;
    }

    return status;
}

lr11xx_status_t lr11xx_radio_get_rx_buffer_status( const void*                      context,
                                                   lr11xx_radio_rx_buffer_status_t* rx_buffer_status )
{
    const uint8_t cbuffer[LR11XX_RADIO_GET_RXBUFFER_STATUS_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_GET_RXBUFFER_STATUS_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_GET_RXBUFFER_STATUS_OC >> 0 ),
    };
    uint8_t rbuffer[sizeof( *rx_buffer_status )] = { 0x00 };

    const lr11xx_status_t status = ( lr11xx_status_t ) lr11xx_hal_read(
        context, cbuffer, LR11XX_RADIO_GET_RXBUFFER_STATUS_CMD_LENGTH, rbuffer, sizeof( *rx_buffer_status ) );

    if( status == LR11XX_STATUS_OK )
    {
        rx_buffer_status->pld_len_in_bytes     = rbuffer[0];
        rx_buffer_status->buffer_start_pointer = rbuffer[1];
    }

    return status;
}

lr11xx_status_t lr11xx_radio_get_gfsk_pkt_status( const void* context, lr11xx_radio_pkt_status_gfsk_t* pkt_status )
{
    const uint8_t cbuffer[LR11XX_RADIO_GET_PKT_STATUS_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_GET_PKT_STATUS_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_GET_PKT_STATUS_OC >> 0 ),
    };
    uint8_t rbuffer[4] = { 0x00 };

    const lr11xx_status_t status =
        ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_RADIO_GET_PKT_STATUS_CMD_LENGTH, rbuffer, 4 );

    if( status == LR11XX_STATUS_OK )
    {
        pkt_status->rssi_sync_in_dbm = -( int8_t ) ( rbuffer[0] >> 1 );
        pkt_status->rssi_avg_in_dbm  = -( int8_t ) ( rbuffer[1] >> 1 );
        pkt_status->rx_len_in_bytes  = rbuffer[2];
        pkt_status->is_addr_err      = ( ( rbuffer[3] & 0x20 ) != 0 ) ? true : false;
        pkt_status->is_crc_err       = ( ( rbuffer[3] & 0x10 ) != 0 ) ? true : false;
        pkt_status->is_len_err       = ( ( rbuffer[3] & 0x08 ) != 0 ) ? true : false;
        pkt_status->is_abort_err     = ( ( rbuffer[3] & 0x04 ) != 0 ) ? true : false;
        pkt_status->is_received      = ( ( rbuffer[3] & 0x02 ) != 0 ) ? true : false;
        pkt_status->is_sent          = ( ( rbuffer[3] & 0x01 ) != 0 ) ? true : false;
    }

    return status;
}

lr11xx_status_t lr11xx_radio_get_lora_pkt_status( const void* context, lr11xx_radio_pkt_status_lora_t* pkt_status )
{
    const uint8_t cbuffer[LR11XX_RADIO_GET_PKT_STATUS_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_GET_PKT_STATUS_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_GET_PKT_STATUS_OC >> 0 ),
    };
    uint8_t rbuffer[3] = { 0x00 };

    const lr11xx_status_t status =
        ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_RADIO_GET_PKT_STATUS_CMD_LENGTH, rbuffer, 3 );

    if( status == LR11XX_STATUS_OK )
    {
        pkt_status->rssi_pkt_in_dbm        = -( int8_t ) ( rbuffer[0] >> 1 );
        pkt_status->snr_pkt_in_db          = ( ( ( int8_t ) rbuffer[1] ) + 2 ) >> 2;
        pkt_status->signal_rssi_pkt_in_dbm = -( int8_t ) ( rbuffer[2] >> 1 );
    }

    return status;
}

lr11xx_status_t lr11xx_radio_get_rssi_inst( const void* context, int8_t* rssi_in_dbm )
{
    const uint8_t cbuffer[LR11XX_RADIO_GET_RSSI_INST_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_GET_RSSI_INST_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_GET_RSSI_INST_OC >> 0 ),
    };
    uint8_t rssi = 0;

    const lr11xx_status_t status = ( lr11xx_status_t ) lr11xx_hal_read(
        context, cbuffer, LR11XX_RADIO_GET_RSSI_INST_CMD_LENGTH, &rssi, sizeof( rssi ) );

    if( status == LR11XX_STATUS_OK )
    {
        *rssi_in_dbm = -( int8_t ) ( rssi >> 1 );
    }

    return status;
}

lr11xx_status_t lr11xx_radio_set_gfsk_sync_word( const void*   context,
                                                 const uint8_t gfsk_sync_word[LR11XX_RADIO_GFSK_SYNC_WORD_LENGTH] )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_GFSK_SYNC_WORD_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_GFSK_SYNC_WORD_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_GFSK_SYNC_WORD_OC >> 0 ),
        gfsk_sync_word[0],
        gfsk_sync_word[1],
        gfsk_sync_word[2],
        gfsk_sync_word[3],
        gfsk_sync_word[4],
        gfsk_sync_word[5],
        gfsk_sync_word[6],
        gfsk_sync_word[7],
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_GFSK_SYNC_WORD_CMD_LENGTH, 0, 0 );
}

#ifndef LR11XX_DISABLE_WARNINGS
#warning \
    "The function lr11xx_radio_set_lora_sync_word replaces the \
deprecated function lr11xx_radio_set_lora_public_network. \
lr11xx_radio_set_lora_sync_word, however, is incompatible \
with chip firmware versions prior to 0x303. For those legacy chips \
only, please use lr11xx_radio_set_lora_public_network. \
To deactivate this warning, define C preprocessor symbol \
LR11XX_DISABLE_WARNINGS."
#endif
lr11xx_status_t lr11xx_radio_set_lora_sync_word( const void* context, const uint8_t sync_word )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_LORA_SYNC_WORD_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_LORA_SYNC_WORD_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_LORA_SYNC_WORD_OC >> 0 ),
        sync_word,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_LORA_SYNC_WORD_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_radio_set_lora_public_network( const void*                            context,
                                                      const lr11xx_radio_lora_network_type_t network_type )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_LORA_PUBLIC_NETWORK_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_LORA_PUBLIC_NETWORK_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_LORA_PUBLIC_NETWORK_OC >> 0 ),
        ( uint8_t ) network_type,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_LORA_SYNC_WORD_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_radio_set_rx( const void* context, const uint32_t timeout_in_ms )
{
    const uint32_t timeout_in_rtc_step = lr11xx_radio_convert_time_in_ms_to_rtc_step( timeout_in_ms );

    return lr11xx_radio_set_rx_with_timeout_in_rtc_step( context, timeout_in_rtc_step );
}

lr11xx_status_t lr11xx_radio_set_rx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout )
{
    lr11xx_status_t status                                  = LR11XX_STATUS_ERROR;
    const uint8_t   cbuffer[LR11XX_RADIO_SET_RX_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_RX_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_RX_OC >> 0 ),
        ( uint8_t ) ( timeout >> 16 ),
        ( uint8_t ) ( timeout >> 8 ),
        ( uint8_t ) ( timeout >> 0 ),
    };

    do
    {
        status = LR11XX_RADIO_APPLY_HIGH_ACP_WORKAROUND( context );
        if( status != LR11XX_STATUS_OK )
        {
            break;
        }

        status = ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_RX_CMD_LENGTH, 0, 0 );
        if( status != LR11XX_STATUS_OK )
        {
            break;
        }
    } while( 0 );

    return status;
}

lr11xx_status_t lr11xx_radio_set_tx( const void* context, const uint32_t timeout_in_ms )
{
    const uint32_t timeout_in_rtc_step = lr11xx_radio_convert_time_in_ms_to_rtc_step( timeout_in_ms );

    return lr11xx_radio_set_tx_with_timeout_in_rtc_step( context, timeout_in_rtc_step );
}

lr11xx_status_t lr11xx_radio_set_tx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step )
{
    lr11xx_status_t status = LR11XX_STATUS_ERROR;

    const uint8_t cbuffer[LR11XX_RADIO_SET_TX_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_TX_OC >> 8 ), ( uint8_t ) ( LR11XX_RADIO_SET_TX_OC >> 0 ),
        ( uint8_t ) ( timeout_in_rtc_step >> 16 ),   ( uint8_t ) ( timeout_in_rtc_step >> 8 ),
        ( uint8_t ) ( timeout_in_rtc_step >> 0 ),
    };

    do
    {
        status = LR11XX_RADIO_APPLY_HIGH_ACP_WORKAROUND( context );
        if( status != LR11XX_STATUS_OK )
        {
            break;
        }

        status = ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_TX_CMD_LENGTH, 0, 0 );
        if( status != LR11XX_STATUS_OK )
        {
            break;
        }
    } while( 0 );

    return status;
}

lr11xx_status_t lr11xx_radio_set_rf_freq( const void* context, const uint32_t freq_in_hz )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_RF_FREQUENCY_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_RF_FREQUENCY_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_RF_FREQUENCY_OC >> 0 ),
        ( uint8_t ) ( freq_in_hz >> 24 ),
        ( uint8_t ) ( freq_in_hz >> 16 ),
        ( uint8_t ) ( freq_in_hz >> 8 ),
        ( uint8_t ) ( freq_in_hz >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_RF_FREQUENCY_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_radio_auto_tx_rx( const void* context, const uint32_t delay,
                                         const lr11xx_radio_intermediary_mode_t intermediary_mode,
                                         const uint32_t                         timeout )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_AUTO_TX_RX_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_AUTOTXRX_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_AUTOTXRX_OC >> 0 ),
        ( uint8_t ) ( delay >> 16 ),
        ( uint8_t ) ( delay >> 8 ),
        ( uint8_t ) ( delay ),
        ( uint8_t ) intermediary_mode,
        ( uint8_t ) ( timeout >> 16 ),
        ( uint8_t ) ( timeout >> 8 ),
        ( uint8_t ) ( timeout ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_AUTO_TX_RX_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_radio_set_cad_params( const void* context, const lr11xx_radio_cad_params_t* cad_params )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_CAD_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_CAD_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_CAD_PARAMS_OC >> 0 ),
        cad_params->cad_symb_nb,
        cad_params->cad_detect_peak,
        cad_params->cad_detect_min,
        ( uint8_t ) cad_params->cad_exit_mode,
        ( uint8_t ) ( cad_params->cad_timeout >> 16 ),
        ( uint8_t ) ( cad_params->cad_timeout >> 8 ),
        ( uint8_t ) ( cad_params->cad_timeout ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_CAD_PARAMS_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_radio_set_pkt_type( const void* context, const lr11xx_radio_pkt_type_t pkt_type )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_PKT_TYPE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_PKT_TYPE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_PKT_TYPE_OC >> 0 ),
        ( uint8_t ) pkt_type,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_PKT_TYPE_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_radio_set_gfsk_mod_params( const void*                           context,
                                                  const lr11xx_radio_mod_params_gfsk_t* mod_params )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_MODULATION_PARAMS_GFSK_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_MODULATION_PARAM_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_MODULATION_PARAM_OC >> 0 ),
        ( uint8_t ) ( mod_params->br_in_bps >> 24 ),
        ( uint8_t ) ( mod_params->br_in_bps >> 16 ),
        ( uint8_t ) ( mod_params->br_in_bps >> 8 ),
        ( uint8_t ) ( mod_params->br_in_bps >> 0 ),
        ( uint8_t ) mod_params->pulse_shape,
        ( uint8_t ) mod_params->bw_dsb_param,
        ( uint8_t ) ( mod_params->fdev_in_hz >> 24 ),
        ( uint8_t ) ( mod_params->fdev_in_hz >> 16 ),
        ( uint8_t ) ( mod_params->fdev_in_hz >> 8 ),
        ( uint8_t ) ( mod_params->fdev_in_hz >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_MODULATION_PARAMS_GFSK_CMD_LENGTH,
                                                 0, 0 );
}

lr11xx_status_t lr11xx_radio_set_lora_mod_params( const void*                           context,
                                                  const lr11xx_radio_mod_params_lora_t* mod_params )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_MODULATION_PARAMS_LORA_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_MODULATION_PARAM_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_MODULATION_PARAM_OC >> 0 ),
        ( uint8_t ) mod_params->sf,
        ( uint8_t ) mod_params->bw,
        ( uint8_t ) mod_params->cr,
        ( uint8_t ) mod_params->ldro,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_MODULATION_PARAMS_LORA_CMD_LENGTH,
                                                 0, 0 );
}

lr11xx_status_t lr11xx_radio_set_gfsk_pkt_params( const void*                           context,
                                                  const lr11xx_radio_pkt_params_gfsk_t* pkt_params )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_PKT_PARAM_GFSK_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_PKT_PARAM_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_PKT_PARAM_OC >> 0 ),
        ( uint8_t ) ( pkt_params->preamble_len_in_bits >> 8 ),
        ( uint8_t ) ( pkt_params->preamble_len_in_bits >> 0 ),
        ( uint8_t ) ( pkt_params->preamble_detector ),
        pkt_params->sync_word_len_in_bits,
        ( uint8_t ) ( pkt_params->address_filtering ),
        ( uint8_t ) ( pkt_params->header_type ),
        pkt_params->pld_len_in_bytes,
        ( uint8_t ) ( pkt_params->crc_type ),
        ( uint8_t ) ( pkt_params->dc_free ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_PKT_PARAM_GFSK_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_radio_set_lora_pkt_params( const void*                           context,
                                                  const lr11xx_radio_pkt_params_lora_t* pkt_params )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_PKT_PARAM_LORA_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_PKT_PARAM_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_PKT_PARAM_OC >> 0 ),
        ( uint8_t ) ( pkt_params->preamble_len_in_symb >> 8 ),
        ( uint8_t ) ( pkt_params->preamble_len_in_symb >> 0 ),
        ( uint8_t ) ( pkt_params->header_type ),
        pkt_params->pld_len_in_bytes,
        ( uint8_t ) ( pkt_params->crc ),
        ( uint8_t ) ( pkt_params->iq ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_PKT_PARAM_LORA_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_radio_set_tx_params( const void* context, const int8_t pwr_in_dbm,
                                            const lr11xx_radio_ramp_time_t ramp_time )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_TX_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_TX_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_TX_PARAMS_OC >> 0 ),
        ( uint8_t ) pwr_in_dbm,
        ( uint8_t ) ramp_time,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_TX_PARAMS_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_radio_set_pkt_address( const void* context, const uint8_t node_address,
                                              const uint8_t broadcast_address )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_PKT_ADDRESS_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_PKT_ADRS_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_PKT_ADRS_OC >> 0 ),
        node_address,
        broadcast_address,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_PKT_ADDRESS_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_radio_set_rx_tx_fallback_mode( const void*                         context,
                                                      const lr11xx_radio_fallback_modes_t fallback_mode )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_RX_TX_FALLBACK_MODE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_RX_TX_FALLBACK_MODE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_RX_TX_FALLBACK_MODE_OC >> 0 ),
        fallback_mode,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_RX_TX_FALLBACK_MODE_CMD_LENGTH, 0,
                                                 0 );
}

lr11xx_status_t lr11xx_radio_set_rx_duty_cycle( const void* context, const uint32_t rx_period_in_ms,
                                                const uint32_t                          sleep_period_in_ms,
                                                const lr11xx_radio_rx_duty_cycle_mode_t mode )
{
    const uint32_t rx_period_in_rtc_step    = lr11xx_radio_convert_time_in_ms_to_rtc_step( rx_period_in_ms );
    const uint32_t sleep_period_in_rtc_step = lr11xx_radio_convert_time_in_ms_to_rtc_step( sleep_period_in_ms );

    return lr11xx_radio_set_rx_duty_cycle_with_timings_in_rtc_step( context, rx_period_in_rtc_step,
                                                                    sleep_period_in_rtc_step, mode );
}

lr11xx_status_t lr11xx_radio_set_rx_duty_cycle_with_timings_in_rtc_step( const void*    context,
                                                                         const uint32_t rx_period_in_rtc_step,
                                                                         const uint32_t sleep_period_in_rtc_step,
                                                                         const lr11xx_radio_rx_duty_cycle_mode_t mode )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_RX_DUTY_CYCLE_MODE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_RX_DUTY_CYCLE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_RX_DUTY_CYCLE_OC >> 0 ),
        ( uint8_t ) ( rx_period_in_rtc_step >> 16 ),
        ( uint8_t ) ( rx_period_in_rtc_step >> 8 ),
        ( uint8_t ) ( rx_period_in_rtc_step >> 0 ),
        ( uint8_t ) ( sleep_period_in_rtc_step >> 16 ),
        ( uint8_t ) ( sleep_period_in_rtc_step >> 8 ),
        ( uint8_t ) ( sleep_period_in_rtc_step >> 0 ),
        ( uint8_t ) mode,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_RX_DUTY_CYCLE_MODE_CMD_LENGTH, 0,
                                                 0 );
}

lr11xx_status_t lr11xx_radio_set_pa_cfg( const void* context, const lr11xx_radio_pa_cfg_t* pa_cfg )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_PA_CFG_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_PA_CFG_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_PA_CFG_OC >> 0 ),
        ( uint8_t ) pa_cfg->pa_sel,
        ( uint8_t ) pa_cfg->pa_reg_supply,
        pa_cfg->pa_duty_cycle,
        pa_cfg->pa_hp_sel,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_PA_CFG_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_radio_stop_timeout_on_preamble( const void* context, const bool stop_timeout_on_preamble )
{
    const uint8_t cbuffer[LR11XX_RADIO_STOP_TIMEOUT_ON_PREAMBLE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_STOP_TIMEOUT_ON_PREAMBLE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_STOP_TIMEOUT_ON_PREAMBLE_OC >> 0 ),
        ( uint8_t ) stop_timeout_on_preamble,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_STOP_TIMEOUT_ON_PREAMBLE_CMD_LENGTH, 0,
                                                 0 );
}

lr11xx_status_t lr11xx_radio_set_cad( const void* context )
{
    lr11xx_status_t status                                   = LR11XX_STATUS_ERROR;
    const uint8_t   cbuffer[LR11XX_RADIO_SET_CAD_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_CAD_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_CAD_OC >> 0 ),
    };

    do
    {
        status = LR11XX_RADIO_APPLY_HIGH_ACP_WORKAROUND( context );
        if( status != LR11XX_STATUS_OK )
        {
            break;
        }

        status = ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_CAD_CMD_LENGTH, 0, 0 );
        if( status != LR11XX_STATUS_OK )
        {
            break;
        }
    } while( 0 );

    return status;
}

lr11xx_status_t lr11xx_radio_set_tx_cw( const void* context )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_TX_CW_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_TX_CW_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_TX_CW_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_TX_CW_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_radio_set_tx_infinite_preamble( const void* context )
{
    lr11xx_status_t status                                                    = LR11XX_STATUS_ERROR;
    const uint8_t   cbuffer[LR11XX_RADIO_SET_TX_INFINITE_PREAMBLE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_TX_INFINITE_PREAMBLE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_TX_INFINITE_PREAMBLE_OC >> 0 ),
    };

    do
    {
        status = LR11XX_RADIO_APPLY_HIGH_ACP_WORKAROUND( context );
        if( status != LR11XX_STATUS_OK )
        {
            break;
        }

        status = ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer,
                                                       LR11XX_RADIO_SET_TX_INFINITE_PREAMBLE_CMD_LENGTH, 0, 0 );
        if( status != LR11XX_STATUS_OK )
        {
            break;
        }
    } while( 0 );

    return status;
}

lr11xx_status_t lr11xx_radio_set_lora_sync_timeout( const void* context, const uint8_t nb_symbol )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_LORA_SYNC_TIMEOUT_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_LORA_SYNC_TIMEOUT_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_LORA_SYNC_TIMEOUT_OC >> 0 ),
        nb_symbol,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_LORA_SYNC_TIMEOUT_CMD_LENGTH, 0,
                                                 0 );
}

lr11xx_status_t lr11xx_radio_set_gfsk_crc_params( const void* context, const uint32_t seed, const uint32_t polynomial )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_GFSK_CRC_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_GFSK_CRC_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_GFSK_CRC_PARAMS_OC >> 0 ),
        ( uint8_t ) ( seed >> 24 ),
        ( uint8_t ) ( seed >> 16 ),
        ( uint8_t ) ( seed >> 8 ),
        ( uint8_t ) ( seed >> 0 ),
        ( uint8_t ) ( polynomial >> 24 ),
        ( uint8_t ) ( polynomial >> 16 ),
        ( uint8_t ) ( polynomial >> 8 ),
        ( uint8_t ) ( polynomial >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_GFSK_CRC_PARAMS_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_radio_set_gfsk_whitening_seed( const void* context, const uint16_t seed )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_GFSK_WHITENING_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_GFSK_WHITENING_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_GFSK_WHITENING_PARAMS_OC >> 0 ),
        ( uint8_t ) ( seed >> 8 ),
        ( uint8_t ) ( seed >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_GFSK_WHITENING_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_radio_cfg_rx_boosted( const void* context, const bool enable_boost_mode )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_RX_BOOSTED_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_RX_BOOSTED_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_RX_BOOSTED_OC >> 0 ),
        ( enable_boost_mode == true ) ? 0x01 : 0x00,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_RX_BOOSTED_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_radio_set_rssi_calibration( const void*                                  context,
                                                   const lr11xx_radio_rssi_calibration_table_t* rssi_cal_table )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_RSSI_CALIBRATION_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_RSSI_CALIBRATION_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_RSSI_CALIBRATION_OC >> 0 ),
        ( uint8_t ) ( ( ( rssi_cal_table->gain_tune.g5 & 0x0F ) << 4 ) + ( rssi_cal_table->gain_tune.g4 & 0x0F ) ),
        ( uint8_t ) ( ( ( rssi_cal_table->gain_tune.g7 & 0x0F ) << 4 ) + ( rssi_cal_table->gain_tune.g6 & 0x0F ) ),
        ( uint8_t ) ( ( ( rssi_cal_table->gain_tune.g9 & 0x0F ) << 4 ) + ( rssi_cal_table->gain_tune.g8 & 0x0F ) ),
        ( uint8_t ) ( ( ( rssi_cal_table->gain_tune.g11 & 0x0F ) << 4 ) + ( rssi_cal_table->gain_tune.g10 & 0x0F ) ),
        ( uint8_t ) ( ( ( rssi_cal_table->gain_tune.g13 & 0x0F ) << 4 ) + ( rssi_cal_table->gain_tune.g12 & 0x0F ) ),
        ( uint8_t ) ( ( ( rssi_cal_table->gain_tune.g13hp2 & 0x0F ) << 4 ) +
                      ( rssi_cal_table->gain_tune.g13hp1 & 0x0F ) ),
        ( uint8_t ) ( ( ( rssi_cal_table->gain_tune.g13hp4 & 0x0F ) << 4 ) +
                      ( rssi_cal_table->gain_tune.g13hp3 & 0x0F ) ),
        ( uint8_t ) ( ( ( rssi_cal_table->gain_tune.g13hp6 & 0x0F ) << 4 ) +
                      ( rssi_cal_table->gain_tune.g13hp5 & 0x0F ) ),
        ( uint8_t ) ( rssi_cal_table->gain_tune.g13hp7 & 0x0F ),
        ( uint8_t ) ( rssi_cal_table->gain_offset >> 8 ),
        ( uint8_t ) ( rssi_cal_table->gain_offset >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RADIO_SET_RSSI_CALIBRATION_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_radio_get_gfsk_rx_bandwidth( uint32_t bw_in_hz, lr11xx_radio_gfsk_bw_t* bw_parameter )
{
    if( bw_in_hz <= 4800 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_4800;
    }
    else if( bw_in_hz <= 5800 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_5800;
    }
    else if( bw_in_hz <= 7300 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_7300;
    }
    else if( bw_in_hz <= 9700 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_9700;
    }
    else if( bw_in_hz <= 11700 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_11700;
    }
    else if( bw_in_hz <= 14600 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_14600;
    }
    else if( bw_in_hz <= 19500 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_19500;
    }
    else if( bw_in_hz <= 23400 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_23400;
    }
    else if( bw_in_hz <= 29300 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_29300;
    }
    else if( bw_in_hz <= 39000 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_39000;
    }
    else if( bw_in_hz <= 46900 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_46900;
    }
    else if( bw_in_hz <= 58600 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_58600;
    }
    else if( bw_in_hz <= 78200 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_78200;
    }
    else if( bw_in_hz <= 93800 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_93800;
    }
    else if( bw_in_hz <= 117300 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_117300;
    }
    else if( bw_in_hz <= 156200 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_156200;
    }
    else if( bw_in_hz <= 187200 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_187200;
    }
    else if( bw_in_hz <= 234300 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_234300;
    }
    else if( bw_in_hz <= 312000 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_312000;
    }
    else if( bw_in_hz <= 373600 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_373600;
    }
    else if( bw_in_hz <= 467000 )
    {
        *bw_parameter = LR11XX_RADIO_GFSK_BW_467000;
    }
    else
    {
        return LR11XX_STATUS_ERROR;
    }

    return LR11XX_STATUS_OK;
}

uint32_t lr11xx_radio_get_lora_time_on_air_numerator( const lr11xx_radio_pkt_params_lora_t* pkt_p,
                                                      const lr11xx_radio_mod_params_lora_t* mod_p )
{
    const int32_t pld_len_in_bytes = pkt_p->pld_len_in_bytes;
    const int32_t sf               = mod_p->sf;
    const bool    pld_is_fix       = pkt_p->header_type == LR11XX_RADIO_LORA_PKT_IMPLICIT;

    int32_t fine_synch        = ( sf <= 6 ) ? 1 : 0;
    bool    long_interleaving = ( mod_p->cr > 4 );

    int32_t total_bytes_nb = pld_len_in_bytes + ( ( pkt_p->crc == LR11XX_RADIO_LORA_CRC_ON ) ? 2 : 0 );
    int32_t tx_bits_symbol = sf - 2 * ( mod_p->ldro != 0 ? 1 : 0 );

    int32_t ceil_numerator;
    int32_t ceil_denominator;

    uint32_t intermed;

    int32_t symbols_nb_data;
    int32_t tx_infobits_header;
    int32_t tx_infobits_payload;

    if( long_interleaving )
    {
        const int32_t fec_rate_numerator   = 4;
        const int32_t fec_rate_denominator = ( mod_p->cr + ( mod_p->cr == 7 ? 1 : 0 ) );

        if( pld_is_fix )
        {
            int32_t tx_bits_symbol_start = sf - 2 + 2 * fine_synch;
            if( 8 * total_bytes_nb * fec_rate_denominator <= 7 * fec_rate_numerator * tx_bits_symbol_start )
            {
                ceil_numerator   = 8 * total_bytes_nb * fec_rate_denominator;
                ceil_denominator = fec_rate_numerator * tx_bits_symbol_start;
            }
            else
            {
                int32_t tx_codedbits_header = tx_bits_symbol_start * 8;
                ceil_numerator = 8 * fec_rate_numerator * tx_bits_symbol + 8 * total_bytes_nb * fec_rate_denominator -
                                 fec_rate_numerator * tx_codedbits_header;
                ceil_denominator = fec_rate_numerator * tx_bits_symbol;
            }
        }
        else
        {
            tx_infobits_header = ( sf * 4 + fine_synch * 8 - 28 ) & ~0x07;
            if( tx_infobits_header < 8 * total_bytes_nb )
            {
                if( tx_infobits_header > 8 * pld_len_in_bytes )
                {
                    tx_infobits_header = 8 * pld_len_in_bytes;
                }
            }
            tx_infobits_payload = 8 * total_bytes_nb - tx_infobits_header;
            if( tx_infobits_payload < 0 )
            {
                tx_infobits_payload = 0;
            }

            ceil_numerator   = tx_infobits_payload * fec_rate_denominator + 8 * fec_rate_numerator * tx_bits_symbol;
            ceil_denominator = fec_rate_numerator * tx_bits_symbol;
        }
    }
    else
    {
        tx_infobits_header = sf * 4 + fine_synch * 8 - 8;

        if( !pld_is_fix )
        {
            tx_infobits_header -= 20;
        }

        tx_infobits_payload = 8 * total_bytes_nb - tx_infobits_header;

        if( tx_infobits_payload < 0 )
            tx_infobits_payload = 0;

        ceil_numerator   = tx_infobits_payload;
        ceil_denominator = 4 * tx_bits_symbol;
    }

    symbols_nb_data = ( ( ceil_numerator + ceil_denominator - 1 ) / ceil_denominator );
    if( !long_interleaving )
    {
        symbols_nb_data = symbols_nb_data * ( mod_p->cr + 4 ) + 8;
    }
    intermed = pkt_p->preamble_len_in_symb + 4 + 2 * fine_synch + symbols_nb_data;

    return ( uint32_t ) ( ( 4 * intermed + 1 ) * ( 1 << ( sf - 2 ) ) ) - 1;
}

uint32_t lr11xx_radio_get_lora_bw_in_hz( lr11xx_radio_lora_bw_t bw )
{
    uint32_t bw_in_hz = 0;

    switch( bw )
    {
    case LR11XX_RADIO_LORA_BW_10:
        bw_in_hz = 10417UL;
        break;
    case LR11XX_RADIO_LORA_BW_15:
        bw_in_hz = 15625UL;
        break;
    case LR11XX_RADIO_LORA_BW_20:
        bw_in_hz = 20833UL;
        break;
    case LR11XX_RADIO_LORA_BW_31:
        bw_in_hz = 31250UL;
        break;
    case LR11XX_RADIO_LORA_BW_41:
        bw_in_hz = 41667UL;
        break;
    case LR11XX_RADIO_LORA_BW_62:
        bw_in_hz = 62500UL;
        break;
    case LR11XX_RADIO_LORA_BW_125:
        bw_in_hz = 125000UL;
        break;
    case LR11XX_RADIO_LORA_BW_250:
        bw_in_hz = 250000UL;
        break;
    case LR11XX_RADIO_LORA_BW_500:
        bw_in_hz = 500000UL;
        break;
    case LR11XX_RADIO_LORA_BW_200:
        bw_in_hz = 203000UL;
        break;
    case LR11XX_RADIO_LORA_BW_400:
        bw_in_hz = 406000UL;
        break;
    case LR11XX_RADIO_LORA_BW_800:
        bw_in_hz = 812000UL;
        break;
    }

    return bw_in_hz;
}

uint32_t lr11xx_radio_get_lora_time_on_air_in_ms( const lr11xx_radio_pkt_params_lora_t* pkt_p,
                                                  const lr11xx_radio_mod_params_lora_t* mod_p )
{
    uint32_t numerator   = 1000U * lr11xx_radio_get_lora_time_on_air_numerator( pkt_p, mod_p );
    uint32_t denominator = lr11xx_radio_get_lora_bw_in_hz( mod_p->bw );
    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

uint32_t lr11xx_radio_get_gfsk_time_on_air_numerator( const lr11xx_radio_pkt_params_gfsk_t* pkt_p )
{
    uint8_t header_len_in_bits;

    switch( pkt_p->header_type )
    {
    case LR11XX_RADIO_GFSK_PKT_FIX_LEN:
    {
        header_len_in_bits = 0;
        break;
    }
    case LR11XX_RADIO_GFSK_PKT_VAR_LEN:
    {
        header_len_in_bits = 8;
        break;
    }
    case LR11XX_RADIO_GFSK_PKT_VAR_LEN_SX128X_COMP:
    {
        header_len_in_bits = 9;
        break;
    }
    default:
    {
        return 0;
    }
    }

    return pkt_p->preamble_len_in_bits + header_len_in_bits + pkt_p->sync_word_len_in_bits +
           ( ( pkt_p->pld_len_in_bytes +
               ( pkt_p->address_filtering == LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE ? 0 : 1 ) +
               lr11xx_radio_get_gfsk_crc_len_in_bytes( pkt_p->crc_type ) )
             << 3 );
}

uint32_t lr11xx_radio_get_gfsk_time_on_air_in_ms( const lr11xx_radio_pkt_params_gfsk_t* pkt_p,
                                                  const lr11xx_radio_mod_params_gfsk_t* mod_p )
{
    uint32_t numerator   = 1000U * lr11xx_radio_get_gfsk_time_on_air_numerator( pkt_p );
    uint32_t denominator = mod_p->br_in_bps;

    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

uint32_t lr11xx_radio_convert_time_in_ms_to_rtc_step( uint32_t time_in_ms )
{
    return ( uint32_t ) ( time_in_ms * LR11XX_RTC_FREQ_IN_HZ / 1000 );
}

lr11xx_status_t lr11xx_radio_get_lora_rx_info( const void* context, bool* is_crc_present, lr11xx_radio_lora_cr_t* cr )
{
    const uint8_t cbuffer[LR11XX_RADIO_GET_LORA_RX_INFO_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_GET_LORA_RX_INFO_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_GET_LORA_RX_INFO_OC >> 0 ),
    };
    uint8_t rbuffer;

    const lr11xx_status_t status =
        ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_RADIO_GET_LORA_RX_INFO_CMD_LENGTH, &rbuffer, 1 );

    if( status == LR11XX_STATUS_OK )
    {
        *is_crc_present = ( ( ( rbuffer & ( 0x01 << 4 ) ) != 0 ) ) ? true : false;
        *cr             = ( lr11xx_radio_lora_cr_t ) ( rbuffer & 0x07 );
    }

    return status;
}

lr11xx_status_t lr11xx_radio_apply_high_acp_workaround( const void* context )
{
    return lr11xx_regmem_write_regmem32_mask( context, 0x00F30054, 1 << 30, 0 << 30 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static inline uint32_t lr11xx_radio_get_gfsk_crc_len_in_bytes( lr11xx_radio_gfsk_crc_type_t crc_type )
{
    switch( crc_type )
    {
    case LR11XX_RADIO_GFSK_CRC_OFF:
        return 0;
    case LR11XX_RADIO_GFSK_CRC_1_BYTE:
        return 1;
    case LR11XX_RADIO_GFSK_CRC_2_BYTES:
        return 2;
    case LR11XX_RADIO_GFSK_CRC_1_BYTE_INV:
        return 1;
    case LR11XX_RADIO_GFSK_CRC_2_BYTES_INV:
        return 2;
    }

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
