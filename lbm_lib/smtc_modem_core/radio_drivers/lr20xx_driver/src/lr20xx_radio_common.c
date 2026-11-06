/*!
 * @file      lr20xx_radio_common.c
 *
 * @brief     Radio common driver implementation for LR20XX
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
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr20xx_radio_common.h"
#include "lr20xx_regmem.h"
#include "lr20xx_workarounds.h"
#include "lr20xx_hal.h"
#include <stdlib.h>
#include <stddef.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Internal RTC frequency
 */
#define LR20XX_RTC_FREQ_IN_HZ ( 32768UL )

/*!
 * @brief Frequency step in Hz used to compute the front end calibration parameter
 *
 * @see lr20xx_radio_common_calibrate_front_end_helper
 */
#define LR20XX_RADIO_COMMON_FRONT_END_CALIBRATION_STEP_IN_HZ ( 4000000u )

/**
 * Register address holding the LQI value
 */
#define LR20XX_RADIO_COMMON_REGISTER_LQI ( 0xF30C38 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR20XX_RADIO_COMMON_CALIBRATE_FRONT_END_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_COMMON_SET_RF_FREQ_CMD_LENGTH ( 2 + 4 )
#define LR20XX_RADIO_COMMON_SET_RX_PATH_CMD_LENGTH ( 2 + 2 )
#define LR20XX_RADIO_COMMON_SET_PA_CFG_CMD_LENGTH ( 2 + 3 )
#define LR20XX_RADIO_COMMON_SET_TX_PARAMS_CMD_LENGTH ( 2 + 2 )
#define LR20XX_RADIO_COMMON_SET_RSSI_CALIBRATION_CMD_LENGTH ( 2 + 1 )
#define LR20XX_RADIO_COMMON_SET_RX_TX_FALLBACK_MODE_CMD_LENGTH ( 2 + 1 )
#define LR20XX_RADIO_COMMON_SET_PKT_TYPE_CMD_LENGTH ( 2 + 1 )
#define LR20XX_RADIO_COMMON_GET_PKT_TYPE_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_COMMON_SET_RX_TIMEOUT_STOP_EVENT_CMD_LENGTH ( 2 + 1 )
#define LR20XX_RADIO_COMMON_RESET_RX_STATS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_COMMON_GET_RX_STATS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_COMMON_GET_RSSI_INST_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_COMMON_SET_RX_CMD_LENGTH ( 2 + 3 )
#define LR20XX_RADIO_COMMON_SET_RX_WITH_DEFAULT_TIMEOUT_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_COMMON_SET_TX_CMD_LENGTH ( 2 + 3 )
#define LR20XX_RADIO_COMMON_SET_TX_WITH_DEFAULT_TIMEOUT_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_COMMON_SET_TX_TEST_MODE_CMD_LENGTH ( 2 + 1 )
#define LR20XX_RADIO_COMMON_SEL_PA_CMD_LENGTH ( 2 + 1 )
#define LR20XX_RADIO_COMMON_SET_RX_DUTY_CYCLE_CMD_LENGTH ( 2 + 7 )
#define LR20XX_RADIO_COMMON_CONFIGURE_AUTO_TX_RX_CMD_LENGTH ( 2 + 8 )
#define LR20XX_RADIO_COMMON_GET_RX_PACKET_LENGTH_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_COMMON_SET_DEFAULT_RX_TX_TIMEOUT_CMD_LENGTH ( 2 + 6 )
#define LR20XX_RADIO_COMMON_SET_TIMESTAMP_SOURCE_CMD_LENGTH ( 2 + 1 )
#define LR20XX_RADIO_COMMON_GET_TIMESTAMP_VALUE_CMD_LENGTH ( 2 + 1 )
#define LR20XX_RADIO_COMMON_SET_CCA_CMD_LENGTH ( 2 + 3 )
#define LR20XX_RADIO_COMMON_GET_CCA_RESULT_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_COMMON_SET_AGC_GAIN_CMD_LENGTH ( 2 + 1 )
#define LR20XX_RADIO_COMMON_SET_CAD_PARAMS_CMD_LENGTH ( 2 + 8 )
#define LR20XX_RADIO_COMMON_SET_CAD_CMD_LENGTH ( 2 )

#define LR20XX_RADIO_COMMON_RSSI_CALIBRATION_SINGLE_LENGTH ( 81u )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for radio related operations
 */
enum
{
    LR20XX_RADIO_COMMON_CALIBRATE_FRONT_END_OC       = 0x0123,
    LR20XX_RADIO_COMMON_SET_RF_FREQ_OC               = 0x0200,
    LR20XX_RADIO_COMMON_SET_RX_PATH_OC               = 0x0201,
    LR20XX_RADIO_COMMON_SET_PA_CFG_OC                = 0x0202,
    LR20XX_RADIO_COMMON_SET_TX_PARAMS_OC             = 0x0203,
    LR20XX_RADIO_COMMON_SET_RSSI_CALIBRATION_OC      = 0x0205,
    LR20XX_RADIO_COMMON_SET_RX_TX_FALLBACK_MODE_OC   = 0x0206,
    LR20XX_RADIO_COMMON_SET_PKT_TYPE_OC              = 0x0207,
    LR20XX_RADIO_COMMON_GET_PKT_TYPE_OC              = 0x0208,
    LR20XX_RADIO_COMMON_SET_RX_TIMEOUT_STOP_EVENT_OC = 0x0209,
    LR20XX_RADIO_COMMON_RESET_RX_STATS_OC            = 0x020A,
    LR20XX_RADIO_COMMON_GET_RSSI_INST_OC             = 0x020B,
    LR20XX_RADIO_COMMON_SET_RX_OC                    = 0x020C,
    LR20XX_RADIO_COMMON_SET_TX_OC                    = 0x020D,
    LR20XX_RADIO_COMMON_SET_TX_TEST_MODE_OC          = 0x020E,
    LR20XX_RADIO_COMMON_SEL_PA_OC                    = 0x020F,
    LR20XX_RADIO_COMMON_SET_RX_DUTY_CYCLE_OC         = 0x0210,
    LR20XX_RADIO_COMMON_CONFIGURE_AUTO_TX_RX         = 0x0211,
    LR20XX_RADIO_COMMON_GET_RX_PACKET_LENGTH_OC      = 0x0212,
    LR20XX_RADIO_COMMON_SET_DEFAULT_RX_TX_TIMEOUT_OC = 0x0215,
    LR20XX_RADIO_COMMON_SET_TIMESTAMP_SOURCE_OC      = 0x0216,
    LR20XX_RADIO_COMMON_GET_TIMESTAMP_VALUE_OC       = 0x0217,
    LR20XX_RADIO_COMMON_SET_CCA_OC                   = 0x0218,
    LR20XX_RADIO_COMMON_GET_CCA_RESULT_OC            = 0x0219,
    LR20XX_RADIO_COMMON_SET_AGC_GAIN_OC              = 0x021A,
    LR20XX_RADIO_COMMON_SET_CAD_PARAMETERS_OC        = 0x021B,
    LR20XX_RADIO_COMMON_SET_CAD_OC                   = 0x021C,
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
 * @brief Serialize an RSSI calibration item into an array
 *
 * @param array Pointer to the array to write to. It is up to the caller to ensure the array is long enough to store the
 * serialized item
 * @param rssi_calibration_item Pointer to the RSSI calibration item to serialize. It is up to the caller to ensure it
 * points to an actual item.
 * @return uint8_t* Pointer to the next memory slot to write
 */
uint8_t* lr20xx_radio_common_serialize_rssi_calibration_item(
    uint8_t* array, const lr20xx_radio_common_rssi_calibration_gain_item_t* rssi_calibration_item );

/**
 * @brief Serialize an RSSI calibration table into an array
 *
 * @param array Pointer to the array to write to. It is up to the caller to ensure the array is long enough to store the
 * serialized table
 * @param rssi_calibration_table Pointer to the calibration table to serialize. Can be NULL, in which case nothing is
 * written to the array
 * @return uint8_t* Pointer to the next memory slot to write
 */
uint8_t* lr20xx_radio_common_serialize_rssi_calibration_table(
    uint8_t* array, const lr20xx_radio_common_rssi_calibration_gain_table_t* rssi_calibration_table );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr20xx_status_t lr20xx_radio_common_calibrate_front_end(
    const void* context, const lr20xx_radio_common_raw_front_end_calibration_value_t* front_end_calibration_values,
    uint8_t n_front_end_calibration_values )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_CALIBRATE_FRONT_END_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_CALIBRATE_FRONT_END_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_CALIBRATE_FRONT_END_OC >> 0 )
    };

    uint8_t raw_front_end_calibration_values[6] = { 0 };
    for( uint8_t rx_path_frequency_index = 0; rx_path_frequency_index < n_front_end_calibration_values;
         rx_path_frequency_index++ )
    {
        raw_front_end_calibration_values[rx_path_frequency_index * 2] =
            ( uint8_t ) ( ( uint16_t )( front_end_calibration_values[rx_path_frequency_index] ) >> 8 );
        raw_front_end_calibration_values[rx_path_frequency_index * 2 + 1] =
            ( uint8_t ) ( ( uint16_t )( front_end_calibration_values[rx_path_frequency_index] ) );
    }

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_CALIBRATE_FRONT_END_CMD_LENGTH,
                                                 raw_front_end_calibration_values,
                                                 ( uint16_t )( n_front_end_calibration_values * 2 ) );
}

lr20xx_status_t lr20xx_radio_common_calibrate_front_end_helper(
    const void* context, const lr20xx_radio_common_front_end_calibration_value_t* front_end_calibration_structures,
    uint8_t n_front_end_calibration_structures )
{
    lr20xx_radio_common_raw_front_end_calibration_value_t raw_calibration_values[3] = { 0 };

    for( uint8_t front_end_calibration_value_index = 0;
         front_end_calibration_value_index < n_front_end_calibration_structures; front_end_calibration_value_index++ )
    {
        const uint32_t freq_hz = front_end_calibration_structures[front_end_calibration_value_index].frequency_in_hertz;
        const lr20xx_radio_common_rx_path_t rx_path =
            front_end_calibration_structures[front_end_calibration_value_index].rx_path;
        // Perform a ceil() to get a value for freq_4mhz corresponding to a frequency higher than or equal to freq_hz
        const uint16_t freq_4mhz =
            ( uint16_t ) ( ( freq_hz + LR20XX_RADIO_COMMON_FRONT_END_CALIBRATION_STEP_IN_HZ - 1u ) /
                          LR20XX_RADIO_COMMON_FRONT_END_CALIBRATION_STEP_IN_HZ );
        raw_calibration_values[front_end_calibration_value_index] =
            ( uint16_t ) ( ( ( rx_path == LR20XX_RADIO_COMMON_RX_PATH_HF ) ? 0x8000u : 0x0000u ) | freq_4mhz );
    }

    return lr20xx_radio_common_calibrate_front_end( context, raw_calibration_values,
                                                    n_front_end_calibration_structures );
}

uint32_t lr20xx_radio_common_convert_time_in_ms_to_rtc_step( uint32_t time_in_ms )
{
    return ( uint32_t ) ( time_in_ms * LR20XX_RTC_FREQ_IN_HZ / 1000 );
}

lr20xx_status_t lr20xx_radio_common_set_rf_freq( const void* context, uint32_t freq_in_hz )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_RF_FREQ_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_RF_FREQ_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_RF_FREQ_OC >> 0 ),
        ( uint8_t ) ( freq_in_hz >> 24 ),
        ( uint8_t ) ( freq_in_hz >> 16 ),
        ( uint8_t ) ( freq_in_hz >> 8 ),
        ( uint8_t ) ( freq_in_hz >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_SET_RF_FREQ_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_set_rx_path( const void* context, lr20xx_radio_common_rx_path_t rx_path,
                                                 lr20xx_radio_common_rx_path_boost_mode_t boost_mode )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_RX_PATH_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_RX_PATH_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_RX_PATH_OC >> 0 ),
        ( uint8_t ) rx_path,
        ( uint8_t ) boost_mode,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_SET_RX_PATH_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_set_pa_cfg( const void* context, const lr20xx_radio_common_pa_cfg_t* pa_cfg )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_PA_CFG_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_PA_CFG_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_PA_CFG_OC >> 0 ),
        ( uint8_t ) ( ( ( uint8_t ) pa_cfg->pa_sel << 7 ) + ( uint8_t ) pa_cfg->pa_lf_mode ),
        ( uint8_t ) ( ( pa_cfg->pa_lf_duty_cycle << 4 ) + pa_cfg->pa_lf_slices ),
        pa_cfg->pa_hf_duty_cycle,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_SET_PA_CFG_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_set_tx_params( const void* context, const int8_t power_half_dbm,
                                                   const lr20xx_radio_common_ramp_time_t ramp_time )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_TX_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_TX_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_TX_PARAMS_OC >> 0 ),
        ( uint8_t ) power_half_dbm,
        ( uint8_t ) ramp_time,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_SET_TX_PARAMS_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_set_rssi_calibration(
    const void* context, const lr20xx_radio_common_rssi_calibration_gain_table_t* rssi_cal_table_lf,
    const lr20xx_radio_common_rssi_calibration_gain_table_t* rssi_cal_table_hf )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_RSSI_CALIBRATION_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_RSSI_CALIBRATION_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_RSSI_CALIBRATION_OC >> 0 ),
        ( uint8_t ) ( ( ( rssi_cal_table_lf == NULL ) ? 0x00 : 0x01 ) | ( ( rssi_cal_table_hf == NULL ) ? 0x00 : 0x02 ) )
    };

    uint8_t  raw_rssi_table[LR20XX_RADIO_COMMON_RSSI_CALIBRATION_SINGLE_LENGTH * 2] = { 0 };
    uint8_t* raw_rssi_table_pointer                                                 = raw_rssi_table;
    uint16_t raw_rssi_table_length                                                  = 0;

    if( rssi_cal_table_lf != NULL )
    {
        raw_rssi_table_pointer =
            lr20xx_radio_common_serialize_rssi_calibration_table( raw_rssi_table_pointer, rssi_cal_table_lf );
        raw_rssi_table_length =
            ( uint16_t )( raw_rssi_table_length + LR20XX_RADIO_COMMON_RSSI_CALIBRATION_SINGLE_LENGTH );
    }
    if( rssi_cal_table_hf != NULL )
    {
        raw_rssi_table_pointer =
            lr20xx_radio_common_serialize_rssi_calibration_table( raw_rssi_table_pointer, rssi_cal_table_hf );
        raw_rssi_table_length =
            ( uint16_t )( raw_rssi_table_length + LR20XX_RADIO_COMMON_RSSI_CALIBRATION_SINGLE_LENGTH );
    }

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_SET_RSSI_CALIBRATION_CMD_LENGTH,
                                                 raw_rssi_table, raw_rssi_table_length );
}

lr20xx_status_t lr20xx_radio_common_set_rx_tx_fallback_mode( const void*                                context,
                                                             const lr20xx_radio_common_fallback_modes_t fallback_mode )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_RX_TX_FALLBACK_MODE_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_RX_TX_FALLBACK_MODE_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_RX_TX_FALLBACK_MODE_OC >> 0 ),
        ( uint8_t ) fallback_mode,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer,
                                                 LR20XX_RADIO_COMMON_SET_RX_TX_FALLBACK_MODE_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_set_pkt_type( const void* context, lr20xx_radio_common_pkt_type_t pkt_type )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_PKT_TYPE_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_PKT_TYPE_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_PKT_TYPE_OC >> 0 ),
        ( uint8_t ) pkt_type,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_SET_PKT_TYPE_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_get_pkt_type( const void* context, lr20xx_radio_common_pkt_type_t* pkt_type )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_GET_PKT_TYPE_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_GET_PKT_TYPE_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_GET_PKT_TYPE_OC >> 0 ),
    };

    uint8_t               pkt_type_raw = 0;
    const lr20xx_status_t status       = ( lr20xx_status_t ) lr20xx_hal_read(
              context, cbuffer, LR20XX_RADIO_COMMON_GET_PKT_TYPE_CMD_LENGTH, &pkt_type_raw, 1 );

    if( status == LR20XX_STATUS_OK )
    {
        *pkt_type = ( lr20xx_radio_common_pkt_type_t ) pkt_type_raw;
    }
    return status;
}

lr20xx_status_t lr20xx_radio_common_set_rx_timeout_stop_event( const void* context,
                                                               const bool  is_stopped_on_preamble_detection )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_RX_TIMEOUT_STOP_EVENT_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_RX_TIMEOUT_STOP_EVENT_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_RX_TIMEOUT_STOP_EVENT_OC >> 0 ),
        ( is_stopped_on_preamble_detection == true ) ? 0x01 : 0x00,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer,
                                                 LR20XX_RADIO_COMMON_SET_RX_TIMEOUT_STOP_EVENT_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_reset_rx_stats( const void* context )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_RESET_RX_STATS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_RESET_RX_STATS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_RESET_RX_STATS_OC >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_RESET_RX_STATS_CMD_LENGTH, 0,
                                                 0 );
}

lr20xx_status_t lr20xx_radio_common_get_rssi_inst( const void* context, int16_t* rssi_in_dbm, uint8_t* half_dbm_count )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_GET_RSSI_INST_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_GET_RSSI_INST_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_GET_RSSI_INST_OC >> 0 ),
    };
    uint8_t rssi_raw[2] = { 0 };

    const lr20xx_status_t status = ( lr20xx_status_t ) lr20xx_hal_read(
        context, cbuffer, LR20XX_RADIO_COMMON_GET_RSSI_INST_CMD_LENGTH, rssi_raw, 2 );

    if( status == LR20XX_STATUS_OK )
    {
        *rssi_in_dbm = ( int16_t )( -( ( int16_t )( rssi_raw[0] ) ) );
        if( half_dbm_count != NULL )
        {
            *half_dbm_count = rssi_raw[1] & 0x01;
        }
    }

    return status;
}

lr20xx_status_t lr20xx_radio_common_set_rx( const void* context, const uint32_t timeout_in_ms )
{
    return lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(
        context, lr20xx_radio_common_convert_time_in_ms_to_rtc_step( timeout_in_ms ) );
}

lr20xx_status_t lr20xx_radio_common_set_rx_with_timeout_in_rtc_step( const void*    context,
                                                                     const uint32_t timeout_in_rtc_step )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_RX_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_RX_OC >> 8 ), ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_RX_OC >> 0 ),
        ( uint8_t ) ( timeout_in_rtc_step >> 16 ),          ( uint8_t ) ( timeout_in_rtc_step >> 8 ),
        ( uint8_t ) ( timeout_in_rtc_step >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_SET_RX_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_set_rx_with_default_timeout( const void* context )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_RX_WITH_DEFAULT_TIMEOUT_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_RX_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_RX_OC >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer,
                                                 LR20XX_RADIO_COMMON_SET_RX_WITH_DEFAULT_TIMEOUT_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_set_tx( const void* context, const uint32_t timeout_in_ms )
{
    return lr20xx_radio_common_set_tx_with_timeout_in_rtc_step(
        context, lr20xx_radio_common_convert_time_in_ms_to_rtc_step( timeout_in_ms ) );
}

lr20xx_status_t lr20xx_radio_common_set_tx_with_timeout_in_rtc_step( const void*    context,
                                                                     const uint32_t timeout_in_rtc_step )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_TX_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_TX_OC >> 8 ), ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_TX_OC >> 0 ),
        ( uint8_t ) ( timeout_in_rtc_step >> 16 ),          ( uint8_t ) ( timeout_in_rtc_step >> 8 ),
        ( uint8_t ) ( timeout_in_rtc_step >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_SET_TX_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_set_tx_with_default_timeout( const void* context )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_TX_WITH_DEFAULT_TIMEOUT_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_TX_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_TX_OC >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer,
                                                 LR20XX_RADIO_COMMON_SET_TX_WITH_DEFAULT_TIMEOUT_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_set_tx_test_mode( const void* context, lr20xx_radio_common_tx_test_mode_t mode )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_TX_TEST_MODE_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_TX_TEST_MODE_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_TX_TEST_MODE_OC >> 0 ),
        ( uint8_t ) mode,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_SET_TX_TEST_MODE_CMD_LENGTH, 0,
                                                 0 );
}

lr20xx_status_t lr20xx_radio_common_select_pa( const void* context, lr20xx_radio_common_pa_selection_t sel )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SEL_PA_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SEL_PA_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SEL_PA_OC >> 0 ),
        ( uint8_t ) sel,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_SEL_PA_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_set_rx_duty_cycle( const void* context, const uint32_t rx_period_in_ms,
                                                       const uint32_t sleep_period_in_ms,
                                                       const lr20xx_radio_common_rx_duty_cycle_mode_t mode )
{
    return lr20xx_radio_common_set_rx_duty_cycle_with_timing_in_rtc_step(
        context, lr20xx_radio_common_convert_time_in_ms_to_rtc_step( rx_period_in_ms ),
        lr20xx_radio_common_convert_time_in_ms_to_rtc_step( sleep_period_in_ms ), mode );
}

lr20xx_status_t lr20xx_radio_common_set_rx_duty_cycle_with_timing_in_rtc_step(
    const void* context, const uint32_t rx_period_in_rtc_step, const uint32_t sleep_period_in_rtc_step,
    const lr20xx_radio_common_rx_duty_cycle_mode_t mode )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_RX_DUTY_CYCLE_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_RX_DUTY_CYCLE_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_RX_DUTY_CYCLE_OC >> 0 ),
        ( uint8_t ) ( rx_period_in_rtc_step >> 16 ),
        ( uint8_t ) ( rx_period_in_rtc_step >> 8 ),
        ( uint8_t ) ( rx_period_in_rtc_step >> 0 ),
        ( uint8_t ) ( sleep_period_in_rtc_step >> 16 ),
        ( uint8_t ) ( sleep_period_in_rtc_step >> 8 ),
        ( uint8_t ) ( sleep_period_in_rtc_step >> 0 ),
        ( uint8_t ) ( mode << 4 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_SET_RX_DUTY_CYCLE_CMD_LENGTH, 0,
                                                 0 );
}

lr20xx_status_t lr20xx_radio_common_configure_auto_tx_rx(
    const void* context, const lr20xx_radio_common_auto_tx_rx_configuration_t* configuration )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_CONFIGURE_AUTO_TX_RX_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_COMMON_CONFIGURE_AUTO_TX_RX >> 8 ),
        ( uint8_t )( LR20XX_RADIO_COMMON_CONFIGURE_AUTO_TX_RX >> 0 ),
        ( uint8_t )( ( ( uint8_t ) configuration->condition ) |
                     ( ( configuration->disable_on_failure == true ) ? 0x80 : 0x00 ) ),
        ( uint8_t )( configuration->tx_rx_timeout_in_rtc_step >> 16 ),
        ( uint8_t )( configuration->tx_rx_timeout_in_rtc_step >> 8 ),
        ( uint8_t )( configuration->tx_rx_timeout_in_rtc_step >> 0 ),
        ( uint8_t )( configuration->delay_in_tick >> 24 ),
        ( uint8_t )( configuration->delay_in_tick >> 16 ),
        ( uint8_t )( configuration->delay_in_tick >> 8 ),
        ( uint8_t )( configuration->delay_in_tick >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_CONFIGURE_AUTO_TX_RX_CMD_LENGTH,
                                                 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_get_rx_packet_length( const void* context, uint16_t* pkt_len )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_GET_RX_PACKET_LENGTH_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_GET_RX_PACKET_LENGTH_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_GET_RX_PACKET_LENGTH_OC >> 0 ),
    };

    uint8_t pkt_len_loc[2] = { 0 };

    const lr20xx_status_t status = ( lr20xx_status_t ) lr20xx_hal_read(
        context, cbuffer, LR20XX_RADIO_COMMON_GET_RX_PACKET_LENGTH_CMD_LENGTH, pkt_len_loc, 2 );

    if( status == LR20XX_STATUS_OK )
    {
        *pkt_len = ( uint16_t ) ( ( ( ( uint16_t ) pkt_len_loc[0] ) << 8 ) + ( uint16_t ) pkt_len_loc[1] );
    }

    return status;
}

lr20xx_status_t lr20xx_radio_common_set_default_rx_tx_timeout( const void* context, uint32_t rx_timeout_in_ms,
                                                               uint32_t tx_timeout_in_ms )
{
    return lr20xx_radio_common_set_default_rx_tx_timeout_in_rtc_step(
        context, lr20xx_radio_common_convert_time_in_ms_to_rtc_step( rx_timeout_in_ms ),
        lr20xx_radio_common_convert_time_in_ms_to_rtc_step( tx_timeout_in_ms ) );
}

lr20xx_status_t lr20xx_radio_common_set_default_rx_tx_timeout_in_rtc_step( const void* context,
                                                                           uint32_t    rx_timeout_in_rtc_step,
                                                                           uint32_t    tx_timeout_in_rtc_step )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_DEFAULT_RX_TX_TIMEOUT_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_DEFAULT_RX_TX_TIMEOUT_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_DEFAULT_RX_TX_TIMEOUT_OC >> 0 ),
        ( uint8_t ) ( rx_timeout_in_rtc_step >> 16 ),
        ( uint8_t ) ( rx_timeout_in_rtc_step >> 8 ),
        ( uint8_t ) ( rx_timeout_in_rtc_step >> 0 ),
        ( uint8_t ) ( tx_timeout_in_rtc_step >> 16 ),
        ( uint8_t ) ( tx_timeout_in_rtc_step >> 8 ),
        ( uint8_t ) ( tx_timeout_in_rtc_step >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer,
                                                 LR20XX_RADIO_COMMON_SET_DEFAULT_RX_TX_TIMEOUT_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_set_timestamp_source( const void*                              context,
                                                          lr20xx_radio_common_timestamp_cfg_slot_t cfg_slot,
                                                          lr20xx_radio_common_timestamp_source_t   source )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_TIMESTAMP_SOURCE_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_TIMESTAMP_SOURCE_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_TIMESTAMP_SOURCE_OC >> 0 ),
        ( uint8_t ) ( ( ( uint8_t ) cfg_slot << 4 ) + ( uint8_t ) source ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_SET_TIMESTAMP_SOURCE_CMD_LENGTH,
                                                 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_get_elapsed_time_in_tick( const void*                              context,
                                                              lr20xx_radio_common_timestamp_cfg_slot_t cfg_slot,
                                                              uint32_t* elapsed_time_in_tick )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_GET_TIMESTAMP_VALUE_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_GET_TIMESTAMP_VALUE_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_GET_TIMESTAMP_VALUE_OC >> 0 ),
        ( uint8_t ) cfg_slot,
    };

    uint8_t elapsed_time_raw[4] = { 0 };

    const lr20xx_status_t status = ( lr20xx_status_t ) lr20xx_hal_read(
        context, cbuffer, LR20XX_RADIO_COMMON_GET_TIMESTAMP_VALUE_CMD_LENGTH, elapsed_time_raw, 4 );

    if( status == LR20XX_STATUS_OK )
    {
        *elapsed_time_in_tick = ( ( uint32_t ) elapsed_time_raw[0] << 24 ) +
                                ( ( uint32_t ) elapsed_time_raw[1] << 16 ) + ( ( uint32_t ) elapsed_time_raw[2] << 8 ) +
                                ( ( uint32_t ) elapsed_time_raw[3] << 0 );
    }

    return status;
}

lr20xx_status_t lr20xx_radio_common_set_cca( const void* context, const uint32_t duration )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_CCA_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_CCA_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_CCA_OC >> 0 ),
        ( uint8_t ) ( duration >> 16 ),
        ( uint8_t ) ( duration >> 8 ),
        ( uint8_t ) ( duration >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_SET_CCA_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_get_cca_result( const void* context, lr20xx_radio_common_cca_res_t* cca_res )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_GET_CCA_RESULT_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_GET_CCA_RESULT_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_GET_CCA_RESULT_OC >> 0 ),
    };

    uint8_t cca_res_raw[4] = { 0 };

    const lr20xx_status_t status = ( lr20xx_status_t ) lr20xx_hal_read(
        context, cbuffer, LR20XX_RADIO_COMMON_GET_RX_PACKET_LENGTH_CMD_LENGTH, cca_res_raw, 4 );

    if( status == LR20XX_STATUS_OK )
    {
        cca_res->min = ( int16_t )( -( ( int16_t ) cca_res_raw[0] ) );
        cca_res->max = ( int16_t )( -( ( int16_t ) cca_res_raw[1] ) );
        cca_res->avg = ( int16_t )( -( ( int16_t ) cca_res_raw[2] ) );
    }

    return status;
}

lr20xx_status_t lr20xx_radio_common_set_agc_gain( const void* context, lr20xx_radio_common_gain_step_t gain )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_AGC_GAIN_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_AGC_GAIN_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_AGC_GAIN_OC >> 0 ),
        ( uint8_t ) gain,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_SET_AGC_GAIN_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_set_cad_params( const void*                             context,
                                                    const lr20xx_radio_common_cad_params_t* params )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_CAD_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_CAD_PARAMETERS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_CAD_PARAMETERS_OC >> 0 ),
        ( uint8_t ) ( params->timeout >> 16 ),
        ( uint8_t ) ( params->timeout >> 8 ),
        ( uint8_t ) ( params->timeout >> 0 ),
        params->threshold,
        ( uint8_t ) params->exit_mode,
        ( uint8_t ) ( params->tx_rx_timeout >> 16 ),
        ( uint8_t ) ( params->tx_rx_timeout >> 8 ),
        ( uint8_t ) ( params->tx_rx_timeout >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_SET_CAD_PARAMS_CMD_LENGTH, 0,
                                                 0 );
}

lr20xx_status_t lr20xx_radio_common_set_cad( const void* context )
{
    const uint8_t cbuffer[LR20XX_RADIO_COMMON_SET_CAD_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_CAD_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_COMMON_SET_CAD_OC >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_COMMON_SET_CAD_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_common_get_lqi( const void* context, lr20xx_radio_common_lqi_value_t* lqi )
{
    uint32_t              raw_register_value = 0;
    const lr20xx_status_t read_register_status =
        lr20xx_regmem_read_regmem32( context, LR20XX_RADIO_COMMON_REGISTER_LQI, &raw_register_value, 1 );
    if( read_register_status == LR20XX_STATUS_OK )
    {
        // The LQI is on the 8 LSBits of the register value
        const uint8_t raw_lqi_value = ( uint8_t ) raw_register_value;

        // The raw LQI is given as counter of 1/4 dB. So divide by 4 to get the integer part, and the 2 LSBits give the
        // number of 1/4 counts
        lqi->lqi_db                 = ( raw_lqi_value / 4 );
        lqi->lqi_quarter_db_counter = ( raw_lqi_value & 0x03 );
    }
    return read_register_status;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

uint8_t* lr20xx_radio_common_serialize_rssi_calibration_item(
    uint8_t* array, const lr20xx_radio_common_rssi_calibration_gain_item_t* rssi_calibration_item )
{
    array[0] = ( uint8_t ) ( rssi_calibration_item->gain_value >> 8 );
    array[1] = ( uint8_t ) rssi_calibration_item->gain_value;
    array[2] = rssi_calibration_item->noise_figure;
    return array + 3;
}

uint8_t* lr20xx_radio_common_serialize_rssi_calibration_table(
    uint8_t* array, const lr20xx_radio_common_rssi_calibration_gain_table_t* rssi_calibration_table )
{
    if( rssi_calibration_table != NULL )
    {
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g1 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g2 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g3 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g4 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g5 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g6 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g7 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g8 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g9 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g10 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g11 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g12_boost0 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g12_boost1 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g12_boost2 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g12_boost3 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g12_boost4 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g12_boost5 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g12_boost6 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g12_boost7 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g13_boost0 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g13_boost1 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g13_boost2 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g13_boost3 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g13_boost4 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g13_boost5 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g13_boost6 );
        array = lr20xx_radio_common_serialize_rssi_calibration_item( array, &rssi_calibration_table->g13_boost7 );
    }
    else
    {
        // Do nothing on purpose
    }
    return array;
}

/* --- EOF ------------------------------------------------------------------ */
