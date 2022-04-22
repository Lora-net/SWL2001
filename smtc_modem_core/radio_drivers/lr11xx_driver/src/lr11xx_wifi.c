/*!
 * @file      lr11xx_wifi.c
 *
 * @brief     Wi-Fi passive scan driver implementation for LR11XX
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

#include "lr11xx_wifi.h"
#include "lr11xx_system_types.h"
#include "lr11xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#ifndef MIN
#define MIN( a, b ) ( ( a > b ) ? b : a )
#endif  // MIN

/*!
 * @brief Check if a value is in between min and max - included
 */
#define IS_BETWEEN( value, min, max ) ( ( min <= value ) && ( value <= max ) )

/*!
 * @brief Check if a value is in between 0x80 and 0xBF - included
 */
#define IS_BETWEEN_0x80_AND_0xBF( value ) IS_BETWEEN( value, 0x80, 0xBF )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR11XX_WIFI_BASIC_COMPLETE_RESULT_SIZE ( 22 )
#define LR11XX_WIFI_BASIC_MAC_TYPE_CHANNEL_RESULT_SIZE ( 9 )

#define LR11XX_WIFI_MAX_SIZE_PER_SPI( single_size ) \
    ( single_size * ( LR11XX_WIFI_MAX_RESULT_PER_TRANSACTION( single_size ) ) )

#define LR11XX_WIFI_MAX_RESULT_PER_TRANSACTION( single_size ) \
    ( MIN( ( LR11XX_WIFI_READ_RESULT_LIMIT ) / ( single_size ), LR11XX_WIFI_N_RESULTS_MAX_PER_CHUNK ) )

#define LR11XX_WIFI_ALL_CUMULATIVE_TIMING_SIZE ( 16 )
#define LR11XX_WIFI_VERSION_SIZE ( 2 )
#define LR11XX_WIFI_READ_RESULT_LIMIT ( 1020 )
#define LR11XX_WIFI_COUNTRY_RESULT_LENGTH_SIZE ( 1 )
#define LR11XX_WIFI_EXTENDED_COMPLETE_RESULT_SIZE ( 79 )
#define LR11XX_WIFI_SCAN_SINGLE_COUNTRY_CODE_RESULT_SIZE ( 10 )
#define LR11XX_WIFI_MAX_COUNTRY_CODE_RESULT_SIZE \
    ( LR11XX_WIFI_MAX_COUNTRY_CODE * LR11XX_WIFI_SCAN_SINGLE_COUNTRY_CODE_RESULT_SIZE )

// Command length
#define LR11XX_WIFI_SCAN_CMD_LENGTH ( 2 + 9 )
#define LR11XX_WIFI_SEARCH_COUNTRY_CODE_CMD_LENGTH ( 2 + 7 )
#define LR11XX_WIFI_SCAN_TIME_LIMIT_CMD_LENGTH ( 2 + 9 )
#define LR11XX_WIFI_COUNTRY_CODE_TIME_LIMIT_CMD_LENGTH ( 2 + 7 )
#define LR11XX_WIFI_GET_RESULT_SIZE_CMD_LENGTH ( 2 )
#define LR11XX_WIFI_READ_RESULT_CMD_LENGTH ( 2 + 3 )
#define LR11XX_WIFI_RESET_CUMUL_TIMING_CMD_LENGTH ( 2 )
#define LR11XX_WIFI_READ_CUMUL_TIMING_CMD_LENGTH ( 2 )
#define LR11XX_WIFI_GET_SIZE_COUNTRY_RESULT_CMD_LENGTH ( 2 )
#define LR11XX_WIFI_READ_COUNTRY_CODE_CMD_LENGTH ( 2 + 2 )
#define LR11XX_WIFI_CFG_TIMESTAMP_AP_PHONE_CMD_LENGTH ( 2 + 4 )
#define LR11XX_WIFI_GET_VERSION_CMD_LENGTH ( 2 )

/*!
 * @brief Wi-Fi scan power consumption
 *
 * @note these numbers are given for information, it should be modified according to the used hardware.
 */
#define LR11XX_WIFI_CORRELATION_UA ( 12000 )
#define LR11XX_WIFI_CAPTURE_UA ( 12000 )
#define LR11XX_WIFI_DEMODULATION_UA ( 4000 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for Wi-Fi-related operations
 */
enum
{
    LR11XX_WIFI_SCAN_OC                         = 0x0300,
    LR11XX_WIFI_SCAN_TIME_LIMIT                 = 0x0301,
    LR11XX_WIFI_SEARCH_COUNTRY_CODE_OC          = 0x0302,
    LR11XX_WIFI_COUNTRY_CODE_TIME_LIMIT_OC      = 0x0303,
    LR11XX_WIFI_GET_RESULT_SIZE_OC              = 0x0305,
    LR11XX_WIFI_READ_RESULT_OC                  = 0x0306,
    LR11XX_WIFI_RESET_CUMUL_TIMING_OC           = 0x0307,
    LR11XX_WIFI_READ_CUMUL_TIMING_OC            = 0x0308,
    LR11XX_WIFI_GET_SIZE_COUNTRY_RESULT_OC      = 0x0309,
    LR11XX_WIFI_READ_COUNTRY_CODE_OC            = 0x030A,
    LR11XX_WIFI_CONFIGURE_TIMESTAMP_AP_PHONE_OC = 0x030B,
    LR11XX_WIFI_GET_VERSION_OC                  = 0x0320,
};

/*!
 * @brief Wi-Fi scan results interface
 */
typedef union
{
    lr11xx_wifi_basic_complete_result_t*         basic_complete;
    lr11xx_wifi_basic_mac_type_channel_result_t* basic_mac_type_channel;
    lr11xx_wifi_extended_full_result_t*          extended_complete;
} lr11xx_wifi_result_interface_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Return a uint16 value by reading a buffer of uint8 from index.
 *
 * This function interpret the array MSB first. It is equivalent to:
 * return array[index] * 256 + array[index+1]
 *
 * @returns The uint16 value
 */
static uint16_t uint16_from_array( const uint8_t* array, const uint16_t index );

/*!
 * @brief Return a uint64 value by reading a buffer of uint8 from index.
 *
 * This function interpret the array MSB first.
 *
 * @returns The uint64 value
 */
static uint64_t uint64_from_array( const uint8_t* array, const uint16_t index );

/*!
 * @brief Propagate the result buffer interpretation depending on the format_code selected
 *
 * @see interpret_basic_complete_result_from_buffer, interpret_basic_mac_type_channel_result_from_buffer,
 * interpret_extended_full_result_from_buffer
 */
static void generic_results_interpreter( const uint8_t n_result_to_parse, const uint8_t index_result_start_writing,
                                         const uint8_t* buffer, lr11xx_wifi_result_interface_t result_interface,
                                         const lr11xx_wifi_result_format_t format_code );

/*!
 * @brief Parse basic complete result
 */
static void interpret_basic_complete_result_from_buffer( const uint8_t  nb_results,
                                                         const uint8_t  index_result_start_writing,
                                                         const uint8_t* buffer,
                                                         lr11xx_wifi_basic_complete_result_t* result );

/*!
 * @brief Parse basic MAC - type - channel result
 */
static void interpret_basic_mac_type_channel_result_from_buffer( const uint8_t  nb_results,
                                                                 const uint8_t  index_result_start_writing,
                                                                 const uint8_t* buffer,
                                                                 lr11xx_wifi_basic_mac_type_channel_result_t* result );

/*!
 * @brief Parse extended full result
 */
static void interpret_extended_full_result_from_buffer( const uint8_t nb_results,
                                                        const uint8_t index_result_start_writing, const uint8_t* buffer,
                                                        lr11xx_wifi_extended_full_result_t* result );

/*!
 * @brief Parse basic MAC - type - channel result
 */
static lr11xx_status_t fetch_and_aggregate_all_results( const void* context, const uint8_t index_result_start,
                                                        const uint8_t                     nb_results,
                                                        const uint8_t                     nb_results_per_chunk_max,
                                                        const lr11xx_wifi_result_format_t result_format_code,
                                                        uint8_t*                          result_buffer,
                                                        lr11xx_wifi_result_interface_t    result_structures );

/*!
 * @brief Share the size of a result format
 *
 * @returns Size in byte of the format given as parameter
 */
static uint8_t lr11xx_wifi_get_result_size_from_format( const lr11xx_wifi_result_format_t format );

/*!
 * @brief Fetch results from the radio after a successful Wi-Fi passive scan
 *
 * @returns Operation status
 */
static lr11xx_hal_status_t lr11xx_wifi_read_results_helper( const void* context, const uint8_t start_index,
                                                            const uint8_t n_elem, uint8_t* buffer,
                                                            const lr11xx_wifi_result_format_t result_format );

/*!
 * @brief Extract Wi-Fi MAC address from a buffer
 */
static void lr11xx_wifi_read_mac_address_from_buffer( const uint8_t* buffer, const uint16_t index_in_buffer,
                                                      lr11xx_wifi_mac_address_t mac_address );

/*!
 * @brief Share the format code corresponding to a result format
 *
 * @returns Format code
 */
static uint8_t lr11xx_wifi_get_format_code( const lr11xx_wifi_result_format_t format );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_status_t lr11xx_wifi_scan( const void* context, const lr11xx_wifi_signal_type_scan_t signal_type,
                                  const lr11xx_wifi_channel_mask_t channels, const lr11xx_wifi_mode_t scan_mode,
                                  const uint8_t max_results, const uint8_t nb_scan_per_channel,
                                  const uint16_t timeout_in_ms, const bool abort_on_timeout )
{
    const uint8_t cbuffer[LR11XX_WIFI_SCAN_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_WIFI_SCAN_OC >> 8 ),
        ( uint8_t ) ( LR11XX_WIFI_SCAN_OC >> 0 ),
        ( uint8_t ) signal_type,
        ( uint8_t ) ( channels >> 8 ),
        ( uint8_t ) ( channels >> 0 ),
        ( uint8_t ) scan_mode,
        max_results,
        nb_scan_per_channel,
        ( uint8_t ) ( timeout_in_ms >> 8 ),
        ( uint8_t ) ( timeout_in_ms >> 0 ),
        ( uint8_t ) ( ( abort_on_timeout == true ) ? 1 : 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_WIFI_SCAN_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_wifi_search_country_code( const void* context, const lr11xx_wifi_channel_mask_t channels_mask,
                                                 const uint8_t nb_max_results, const uint8_t nb_scan_per_channel,
                                                 const uint16_t timeout_in_ms, const bool abort_on_timeout )
{
    const uint8_t cbuffer[LR11XX_WIFI_SEARCH_COUNTRY_CODE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_WIFI_SEARCH_COUNTRY_CODE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_WIFI_SEARCH_COUNTRY_CODE_OC >> 0 ),
        ( uint8_t ) ( channels_mask >> 8 ),
        ( uint8_t ) ( channels_mask >> 0 ),
        nb_max_results,
        nb_scan_per_channel,
        ( uint8_t ) ( timeout_in_ms >> 8 ),
        ( uint8_t ) ( timeout_in_ms >> 0 ),
        ( uint8_t ) ( ( abort_on_timeout == true ) ? 1 : 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_WIFI_SEARCH_COUNTRY_CODE_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_wifi_scan_time_limit( const void* radio, const lr11xx_wifi_signal_type_scan_t signal_type,
                                             const lr11xx_wifi_channel_mask_t channels,
                                             const lr11xx_wifi_mode_t scan_mode, const uint8_t max_results,
                                             const uint16_t timeout_per_channel_ms, const uint16_t timeout_per_scan_ms )
{
    const uint8_t cbuffer[LR11XX_WIFI_SCAN_TIME_LIMIT_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_WIFI_SCAN_TIME_LIMIT >> 8 ),
        ( uint8_t ) ( LR11XX_WIFI_SCAN_TIME_LIMIT >> 0 ),
        ( uint8_t ) signal_type,
        ( uint8_t ) ( channels >> 8 ),
        ( uint8_t ) ( channels >> 0 ),
        ( uint8_t ) scan_mode,
        max_results,
        ( uint8_t ) ( timeout_per_channel_ms >> 8 ),
        ( uint8_t ) ( timeout_per_channel_ms >> 0 ),
        ( uint8_t ) ( timeout_per_scan_ms >> 8 ),
        ( uint8_t ) ( timeout_per_scan_ms >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( radio, cbuffer, LR11XX_WIFI_SCAN_TIME_LIMIT_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_wifi_search_country_code_time_limit( const void*                      radio,
                                                            const lr11xx_wifi_channel_mask_t channels,
                                                            const uint8_t                    max_results,
                                                            const uint16_t                   timeout_per_channel_ms,
                                                            const uint16_t                   timeout_per_scan_ms )
{
    const uint8_t cbuffer[LR11XX_WIFI_COUNTRY_CODE_TIME_LIMIT_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_WIFI_COUNTRY_CODE_TIME_LIMIT_OC >> 8 ),
        ( uint8_t ) ( LR11XX_WIFI_COUNTRY_CODE_TIME_LIMIT_OC >> 0 ),
        ( uint8_t ) ( channels >> 8 ),
        ( uint8_t ) ( channels >> 0 ),
        ( uint8_t ) max_results,
        ( uint8_t ) ( timeout_per_channel_ms >> 8 ),
        ( uint8_t ) ( timeout_per_channel_ms >> 0 ),
        ( uint8_t ) ( timeout_per_scan_ms >> 8 ),
        ( uint8_t ) ( timeout_per_scan_ms >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( radio, cbuffer, LR11XX_WIFI_COUNTRY_CODE_TIME_LIMIT_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_wifi_get_nb_results( const void* context, uint8_t* nb_results )
{
    const uint8_t cbuffer[LR11XX_WIFI_GET_RESULT_SIZE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_WIFI_GET_RESULT_SIZE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_WIFI_GET_RESULT_SIZE_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_WIFI_GET_RESULT_SIZE_CMD_LENGTH, nb_results,
                                                sizeof( *nb_results ) );
}

lr11xx_status_t lr11xx_wifi_read_basic_complete_results( const void* context, const uint8_t start_result_index,
                                                         const uint8_t                        nb_results,
                                                         lr11xx_wifi_basic_complete_result_t* results )
{
    uint8_t       result_buffer[LR11XX_WIFI_MAX_SIZE_PER_SPI( LR11XX_WIFI_BASIC_COMPLETE_RESULT_SIZE )] = { 0 };
    const uint8_t nb_results_per_chunk_max =
        LR11XX_WIFI_MAX_RESULT_PER_TRANSACTION( LR11XX_WIFI_BASIC_COMPLETE_RESULT_SIZE );

    lr11xx_wifi_result_interface_t result_interface = { 0 };
    result_interface.basic_complete                 = results;

    return fetch_and_aggregate_all_results( context, start_result_index, nb_results, nb_results_per_chunk_max,
                                            LR11XX_WIFI_RESULT_FORMAT_BASIC_COMPLETE, result_buffer, result_interface );
}

lr11xx_status_t lr11xx_wifi_read_basic_mac_type_channel_results( const void* context, const uint8_t start_result_index,
                                                                 const uint8_t nb_results,
                                                                 lr11xx_wifi_basic_mac_type_channel_result_t* results )
{
    uint8_t       result_buffer[LR11XX_WIFI_MAX_SIZE_PER_SPI( LR11XX_WIFI_BASIC_MAC_TYPE_CHANNEL_RESULT_SIZE )] = { 0 };
    const uint8_t nb_results_per_chunk_max =
        LR11XX_WIFI_MAX_RESULT_PER_TRANSACTION( LR11XX_WIFI_BASIC_MAC_TYPE_CHANNEL_RESULT_SIZE );

    lr11xx_wifi_result_interface_t result_interface = { 0 };
    result_interface.basic_mac_type_channel         = results;

    return fetch_and_aggregate_all_results( context, start_result_index, nb_results, nb_results_per_chunk_max,
                                            LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL, result_buffer,
                                            result_interface );
}

lr11xx_status_t lr11xx_wifi_read_extended_full_results( const void* radio, const uint8_t start_result_index,
                                                        const uint8_t                       nb_results,
                                                        lr11xx_wifi_extended_full_result_t* results )
{
    uint8_t       result_buffer[LR11XX_WIFI_MAX_SIZE_PER_SPI( LR11XX_WIFI_EXTENDED_COMPLETE_RESULT_SIZE )] = { 0 };
    const uint8_t nb_results_per_chunk_max =
        LR11XX_WIFI_MAX_RESULT_PER_TRANSACTION( LR11XX_WIFI_EXTENDED_COMPLETE_RESULT_SIZE );

    lr11xx_wifi_result_interface_t result_interface = { 0 };
    result_interface.extended_complete              = results;

    return fetch_and_aggregate_all_results( radio, start_result_index, nb_results, nb_results_per_chunk_max,
                                            LR11XX_WIFI_RESULT_FORMAT_EXTENDED_FULL, result_buffer, result_interface );
}

lr11xx_status_t lr11xx_wifi_reset_cumulative_timing( const void* context )
{
    const uint8_t cbuffer[LR11XX_WIFI_RESET_CUMUL_TIMING_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_WIFI_RESET_CUMUL_TIMING_OC >> 8 ),
        ( uint8_t ) ( LR11XX_WIFI_RESET_CUMUL_TIMING_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_WIFI_RESET_CUMUL_TIMING_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_wifi_read_cumulative_timing( const void* context, lr11xx_wifi_cumulative_timings_t* timing )
{
    const uint8_t cbuffer[LR11XX_WIFI_READ_CUMUL_TIMING_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_WIFI_READ_CUMUL_TIMING_OC >> 8 ),
        ( uint8_t ) ( LR11XX_WIFI_READ_CUMUL_TIMING_OC >> 0 ),
    };
    uint8_t buffer_out[LR11XX_WIFI_ALL_CUMULATIVE_TIMING_SIZE] = { 0 };

    const lr11xx_hal_status_t hal_status = lr11xx_hal_read( context, cbuffer, LR11XX_WIFI_READ_CUMUL_TIMING_CMD_LENGTH,
                                                            buffer_out, LR11XX_WIFI_ALL_CUMULATIVE_TIMING_SIZE );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        timing->rx_detection_us =
            ( buffer_out[0] << 24 ) + ( buffer_out[1] << 16 ) + ( buffer_out[2] << 8 ) + buffer_out[3];
        timing->rx_correlation_us =
            ( buffer_out[4] << 24 ) + ( buffer_out[5] << 16 ) + ( buffer_out[6] << 8 ) + buffer_out[7];
        timing->rx_capture_us =
            ( buffer_out[8] << 24 ) + ( buffer_out[9] << 16 ) + ( buffer_out[10] << 8 ) + buffer_out[11];
        timing->demodulation_us =
            ( buffer_out[12] << 24 ) + ( buffer_out[13] << 16 ) + ( buffer_out[14] << 8 ) + buffer_out[15];
    }
    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_wifi_get_nb_country_code_results( const void* context, uint8_t* country_result_size )
{
    const uint8_t cbuffer[LR11XX_WIFI_GET_SIZE_COUNTRY_RESULT_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_WIFI_GET_SIZE_COUNTRY_RESULT_OC >> 8 ),
        ( uint8_t ) ( LR11XX_WIFI_GET_SIZE_COUNTRY_RESULT_OC >> 0 ),
    };
    uint8_t rbuffer[LR11XX_WIFI_COUNTRY_RESULT_LENGTH_SIZE] = { 0 };

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_WIFI_GET_SIZE_COUNTRY_RESULT_CMD_LENGTH, rbuffer,
                         LR11XX_WIFI_COUNTRY_RESULT_LENGTH_SIZE );
    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        ( *country_result_size ) = rbuffer[0];
    }
    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_wifi_read_country_code_results( const void* context, const uint8_t start_result_index,
                                                       const uint8_t               nb_country_results,
                                                       lr11xx_wifi_country_code_t* country_code_results )
{
    const uint8_t cbuffer[LR11XX_WIFI_READ_COUNTRY_CODE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_WIFI_READ_COUNTRY_CODE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_WIFI_READ_COUNTRY_CODE_OC >> 0 ),
        start_result_index,
        nb_country_results,
    };
    uint8_t        rbuffer[LR11XX_WIFI_MAX_COUNTRY_CODE_RESULT_SIZE] = { 0 };
    const uint16_t country_code_result_size_to_read =
        nb_country_results * LR11XX_WIFI_SCAN_SINGLE_COUNTRY_CODE_RESULT_SIZE;

    const lr11xx_hal_status_t hal_status = lr11xx_hal_read( context, cbuffer, LR11XX_WIFI_READ_COUNTRY_CODE_CMD_LENGTH,
                                                            rbuffer, country_code_result_size_to_read );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        for( uint8_t result_index = 0; result_index < nb_country_results; result_index++ )
        {
            const uint8_t               local_index = result_index * LR11XX_WIFI_SCAN_SINGLE_COUNTRY_CODE_RESULT_SIZE;
            lr11xx_wifi_country_code_t* local_country_code_result = &country_code_results[result_index];

            local_country_code_result->country_code[0]   = rbuffer[local_index + 0];
            local_country_code_result->country_code[1]   = rbuffer[local_index + 1];
            local_country_code_result->io_regulation     = rbuffer[local_index + 2];
            local_country_code_result->channel_info_byte = rbuffer[local_index + 3];

            for( uint8_t field_mac_index = 0; field_mac_index < LR11XX_WIFI_MAC_ADDRESS_LENGTH; field_mac_index++ )
            {
                local_country_code_result->mac_address[field_mac_index] =
                    rbuffer[local_index + ( LR11XX_WIFI_MAC_ADDRESS_LENGTH - field_mac_index - 1 ) + 4];
            }
        }
    }
    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_wifi_cfg_timestamp_ap_phone( const void* context, uint32_t timestamp_in_s )
{
    const uint8_t cbuffer[LR11XX_WIFI_CFG_TIMESTAMP_AP_PHONE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_WIFI_CONFIGURE_TIMESTAMP_AP_PHONE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_WIFI_CONFIGURE_TIMESTAMP_AP_PHONE_OC >> 0 ),
        ( uint8_t ) ( timestamp_in_s >> 24 ),
        ( uint8_t ) ( timestamp_in_s >> 16 ),
        ( uint8_t ) ( timestamp_in_s >> 8 ),
        ( uint8_t ) ( timestamp_in_s >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_WIFI_CFG_TIMESTAMP_AP_PHONE_CMD_LENGTH, 0,
                                                 0 );
}

lr11xx_status_t lr11xx_wifi_read_version( const void* context, lr11xx_wifi_version_t* wifi_version )
{
    const uint8_t cbuffer[LR11XX_WIFI_GET_VERSION_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_WIFI_GET_VERSION_OC >> 8 ),
        ( uint8_t ) ( LR11XX_WIFI_GET_VERSION_OC >> 0 ),
    };
    uint8_t                   rbuffer[LR11XX_WIFI_VERSION_SIZE] = { 0 };
    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_WIFI_GET_VERSION_CMD_LENGTH, rbuffer, LR11XX_WIFI_VERSION_SIZE );
    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        wifi_version->major = rbuffer[0];
        wifi_version->minor = rbuffer[1];
    }
    return ( lr11xx_status_t ) hal_status;
}

uint8_t lr11xx_wifi_get_nb_results_max_per_chunk( void )
{
    return ( uint8_t ) LR11XX_WIFI_N_RESULTS_MAX_PER_CHUNK;
}

void lr11xx_wifi_parse_channel_info( const lr11xx_wifi_channel_info_byte_t channel_info, lr11xx_wifi_channel_t* channel,
                                     bool* rssi_validity, lr11xx_wifi_mac_origin_t* mac_origin_estimation )
{
    ( *channel )               = lr11xx_wifi_extract_channel_from_info_byte( channel_info );
    ( *mac_origin_estimation ) = ( lr11xx_wifi_mac_origin_t ) ( ( channel_info & 0x30 ) >> 4 );
    ( *rssi_validity )         = ( ( channel_info & 0x40 ) == 0 ) ? true : false;
}

lr11xx_wifi_channel_t lr11xx_wifi_extract_channel_from_info_byte( const lr11xx_wifi_channel_info_byte_t channel_info )
{
    return ( lr11xx_wifi_channel_t ) ( channel_info & 0x0F );
}

void lr11xx_wifi_parse_frame_type_info( const lr11xx_wifi_frame_type_info_byte_t frame_type_info,
                                        lr11xx_wifi_frame_type_t*                frame_type,
                                        lr11xx_wifi_frame_sub_type_t* frame_sub_type, bool* to_ds, bool* from_ds )
{
    ( *frame_type )     = ( lr11xx_wifi_frame_type_t ) ( ( frame_type_info >> 6 ) & 0x03 );
    ( *frame_sub_type ) = ( lr11xx_wifi_frame_sub_type_t ) ( ( frame_type_info >> 2 ) & 0x0F );
    ( *to_ds )          = ( bool ) ( ( frame_type_info >> 1 ) & 0x01 );
    ( *from_ds )        = ( bool ) ( frame_type_info & 0x01 );
}

void lr11xx_wifi_parse_data_rate_info( const lr11xx_wifi_datarate_info_byte_t data_rate_info,
                                       lr11xx_wifi_signal_type_result_t*      wifi_signal_type,
                                       lr11xx_wifi_datarate_t*                wifi_data_rate )
{
    ( *wifi_signal_type ) = lr11xx_wifi_extract_signal_type_from_data_rate_info( data_rate_info );
    ( *wifi_data_rate )   = ( lr11xx_wifi_datarate_t ) ( data_rate_info >> 2 );
}

lr11xx_wifi_signal_type_result_t lr11xx_wifi_extract_signal_type_from_data_rate_info(
    const lr11xx_wifi_datarate_info_byte_t data_rate_info )
{
    return ( lr11xx_wifi_signal_type_result_t ) ( data_rate_info & 0x03 );
}

uint64_t lr11xx_wifi_get_consumption( lr11xx_system_reg_mode_t regulator, lr11xx_wifi_cumulative_timings_t timing )
{
    uint64_t wifi_scan_consumption_uah = 0;

    wifi_scan_consumption_uah = ( timing.rx_capture_us * LR11XX_WIFI_CAPTURE_UA ) +
                                ( timing.demodulation_us * LR11XX_WIFI_DEMODULATION_UA ) +
                                ( timing.rx_correlation_us * LR11XX_WIFI_CORRELATION_UA );

    wifi_scan_consumption_uah =
        wifi_scan_consumption_uah /
        ( 3600000000 - ( timing.rx_capture_us + timing.demodulation_us + timing.rx_correlation_us ) );

    if( regulator == LR11XX_SYSTEM_REG_MODE_LDO )
    {
        wifi_scan_consumption_uah *= 2;
    }

    return wifi_scan_consumption_uah;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static lr11xx_hal_status_t lr11xx_wifi_read_results_helper( const void* context, const uint8_t start_index,
                                                            const uint8_t n_elem, uint8_t* buffer,
                                                            const lr11xx_wifi_result_format_t result_format )
{
    const uint8_t  size_single_elem   = lr11xx_wifi_get_result_size_from_format( result_format );
    const uint8_t  result_format_code = lr11xx_wifi_get_format_code( result_format );
    const uint8_t  cbuffer[LR11XX_WIFI_READ_RESULT_CMD_LENGTH] = { ( uint8_t ) ( LR11XX_WIFI_READ_RESULT_OC >> 8 ),
                                                                  ( uint8_t ) ( LR11XX_WIFI_READ_RESULT_OC & 0x00FF ),
                                                                  start_index, n_elem, result_format_code };
    const uint16_t size_total                                  = n_elem * size_single_elem;
    return lr11xx_hal_read( context, cbuffer, LR11XX_WIFI_READ_RESULT_CMD_LENGTH, buffer, size_total );
}

static uint16_t uint16_from_array( const uint8_t* array, const uint16_t index )
{
    return ( uint16_t ) ( array[index] << 8 ) + ( ( uint16_t ) ( array[index + 1] ) );
}

static uint64_t uint64_from_array( const uint8_t* array, const uint16_t index )
{
    return ( ( uint64_t ) ( array[index] ) << 56 ) + ( ( uint64_t ) ( array[index + 1] ) << 48 ) +
           ( ( uint64_t ) ( array[index + 2] ) << 40 ) + ( ( uint64_t ) ( array[index + 3] ) << 32 ) +
           ( ( uint64_t ) ( array[index + 4] ) << 24 ) + ( ( uint64_t ) ( array[index + 5] ) << 16 ) +
           ( ( uint64_t ) ( array[index + 6] ) << 8 ) + ( uint64_t ) ( array[index + 7] );
}

static void lr11xx_wifi_read_mac_address_from_buffer( const uint8_t* buffer, const uint16_t index_in_buffer,
                                                      lr11xx_wifi_mac_address_t mac_address )
{
    for( uint8_t field_mac_index = 0; field_mac_index < LR11XX_WIFI_MAC_ADDRESS_LENGTH; field_mac_index++ )
    {
        mac_address[field_mac_index] = buffer[index_in_buffer + field_mac_index];
    }
}

static uint8_t lr11xx_wifi_get_format_code( const lr11xx_wifi_result_format_t format )
{
    uint8_t format_code = 0x00;
    switch( format )
    {
    case LR11XX_WIFI_RESULT_FORMAT_BASIC_COMPLETE:
    {
        format_code = 0x01;
        break;
    }
    case LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL:
    {
        format_code = 0x04;
        break;
    }
    case LR11XX_WIFI_RESULT_FORMAT_EXTENDED_FULL:
    {
        format_code = 0x01;
        break;
    }
    }
    return format_code;
}

static uint8_t lr11xx_wifi_get_result_size_from_format( const lr11xx_wifi_result_format_t format )
{
    uint8_t result_size = 0;
    switch( format )
    {
    case LR11XX_WIFI_RESULT_FORMAT_BASIC_COMPLETE:
    {
        result_size = LR11XX_WIFI_BASIC_COMPLETE_RESULT_SIZE;
        break;
    }
    case LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL:
    {
        result_size = LR11XX_WIFI_BASIC_MAC_TYPE_CHANNEL_RESULT_SIZE;
        break;
    }
    case LR11XX_WIFI_RESULT_FORMAT_EXTENDED_FULL:
    {
        result_size = LR11XX_WIFI_EXTENDED_COMPLETE_RESULT_SIZE;
        break;
    }
    }
    return result_size;
}

static lr11xx_status_t fetch_and_aggregate_all_results( const void* context, const uint8_t index_result_start,
                                                        const uint8_t                     nb_results,
                                                        const uint8_t                     nb_results_per_chunk_max,
                                                        const lr11xx_wifi_result_format_t result_format_code,
                                                        uint8_t*                          result_buffer,
                                                        lr11xx_wifi_result_interface_t    result_structures )
{
    uint8_t index_to_read              = index_result_start;
    uint8_t index_result_start_writing = 0;
    uint8_t remaining_results          = nb_results;

    lr11xx_hal_status_t hal_status = LR11XX_HAL_STATUS_OK;
    while( remaining_results > 0 )
    {
        uint8_t results_to_read = MIN( remaining_results, nb_results_per_chunk_max );

        lr11xx_hal_status_t local_hal_status = lr11xx_wifi_read_results_helper( context, index_to_read, results_to_read,
                                                                                result_buffer, result_format_code );
        if( local_hal_status != LR11XX_HAL_STATUS_OK )
        {
            return ( lr11xx_status_t ) local_hal_status;
        }

        generic_results_interpreter( results_to_read, index_result_start_writing, result_buffer, result_structures,
                                     result_format_code );

        // Reset the content of the result_buffer in case there are still results to fetch
        {
            const uint16_t result_buffer_size =
                LR11XX_WIFI_MAX_SIZE_PER_SPI( lr11xx_wifi_get_result_size_from_format( result_format_code ) );
            for( uint16_t index = 0; index < result_buffer_size; index++ )
            {
                result_buffer[index] = 0;
            }
        }

        index_to_read += results_to_read;
        index_result_start_writing += results_to_read;
        remaining_results -= results_to_read;
    }
    return ( lr11xx_status_t ) hal_status;
}

static void generic_results_interpreter( const uint8_t n_result_to_parse, const uint8_t index_result_start_writing,
                                         const uint8_t* buffer, lr11xx_wifi_result_interface_t result_interface,
                                         const lr11xx_wifi_result_format_t format_code )
{
    switch( format_code )
    {
    case LR11XX_WIFI_RESULT_FORMAT_BASIC_COMPLETE:
    {
        interpret_basic_complete_result_from_buffer( n_result_to_parse, index_result_start_writing, buffer,
                                                     result_interface.basic_complete );
        break;
    }

    case LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL:
    {
        interpret_basic_mac_type_channel_result_from_buffer( n_result_to_parse, index_result_start_writing, buffer,
                                                             result_interface.basic_mac_type_channel );
        break;
    }

    case LR11XX_WIFI_RESULT_FORMAT_EXTENDED_FULL:
    {
        interpret_extended_full_result_from_buffer( n_result_to_parse, index_result_start_writing, buffer,
                                                    result_interface.extended_complete );
        break;
    }
    }
}

static void interpret_basic_complete_result_from_buffer( const uint8_t  nb_results,
                                                         const uint8_t  index_result_start_writing,
                                                         const uint8_t* buffer,
                                                         lr11xx_wifi_basic_complete_result_t* result )
{
    for( uint8_t result_index = 0; result_index < nb_results; result_index++ )
    {
        const uint16_t                       local_index_start = LR11XX_WIFI_BASIC_COMPLETE_RESULT_SIZE * result_index;
        lr11xx_wifi_basic_complete_result_t* local_wifi_result = &result[index_result_start_writing + result_index];
        local_wifi_result->data_rate_info_byte                 = buffer[local_index_start + 0];
        local_wifi_result->channel_info_byte                   = buffer[local_index_start + 1];
        local_wifi_result->rssi                                = buffer[local_index_start + 2];
        local_wifi_result->frame_type_info_byte                = buffer[local_index_start + 3];
        lr11xx_wifi_read_mac_address_from_buffer( buffer, local_index_start + 4, local_wifi_result->mac_address );
        local_wifi_result->phi_offset       = uint16_from_array( buffer, local_index_start + 10 );
        local_wifi_result->timestamp_us     = uint64_from_array( buffer, local_index_start + 12 );
        local_wifi_result->beacon_period_tu = uint16_from_array( buffer, local_index_start + 20 );
    }
}

static void interpret_basic_mac_type_channel_result_from_buffer( const uint8_t  nb_results,
                                                                 const uint8_t  index_result_start_writing,
                                                                 const uint8_t* buffer,
                                                                 lr11xx_wifi_basic_mac_type_channel_result_t* result )
{
    for( uint8_t result_index = 0; result_index < nb_results; result_index++ )
    {
        const uint16_t local_index_start = LR11XX_WIFI_BASIC_MAC_TYPE_CHANNEL_RESULT_SIZE * result_index;
        lr11xx_wifi_basic_mac_type_channel_result_t* local_wifi_result =
            &result[index_result_start_writing + result_index];
        local_wifi_result->data_rate_info_byte = buffer[local_index_start + 0];
        local_wifi_result->channel_info_byte   = buffer[local_index_start + 1];
        local_wifi_result->rssi                = buffer[local_index_start + 2];
        lr11xx_wifi_read_mac_address_from_buffer( buffer, local_index_start + 3, local_wifi_result->mac_address );
    }
}

void interpret_extended_full_result_from_buffer( const uint8_t nb_results, const uint8_t index_result_start_writing,
                                                 const uint8_t* buffer, lr11xx_wifi_extended_full_result_t* result )
{
    for( uint8_t result_index = 0; result_index < nb_results; result_index++ )
    {
        const uint16_t local_index_start = LR11XX_WIFI_EXTENDED_COMPLETE_RESULT_SIZE * result_index;
        lr11xx_wifi_extended_full_result_t* local_wifi_result = &result[index_result_start_writing + result_index];

        local_wifi_result->data_rate_info_byte = buffer[local_index_start + 0];
        local_wifi_result->channel_info_byte   = buffer[local_index_start + 1];
        local_wifi_result->rssi                = buffer[local_index_start + 2];
        local_wifi_result->rate                = buffer[local_index_start + 3];
        local_wifi_result->service             = uint16_from_array( buffer, local_index_start + 4 );
        local_wifi_result->length              = uint16_from_array( buffer, local_index_start + 6 );
        local_wifi_result->frame_control       = uint16_from_array( buffer, local_index_start + 8 );
        lr11xx_wifi_read_mac_address_from_buffer( buffer, local_index_start + 10, local_wifi_result->mac_address_1 );
        lr11xx_wifi_read_mac_address_from_buffer( buffer, local_index_start + 16, local_wifi_result->mac_address_2 );
        lr11xx_wifi_read_mac_address_from_buffer( buffer, local_index_start + 22, local_wifi_result->mac_address_3 );
        local_wifi_result->timestamp_us     = uint64_from_array( buffer, local_index_start + 28 );
        local_wifi_result->beacon_period_tu = uint16_from_array( buffer, local_index_start + 36 );
        local_wifi_result->seq_control      = uint16_from_array( buffer, local_index_start + 38 );
        for( uint8_t ssid_index = 0; ssid_index < LR11XX_WIFI_RESULT_SSID_LENGTH; ssid_index++ )
        {
            local_wifi_result->ssid_bytes[ssid_index] = buffer[local_index_start + ssid_index + 40];
        }
        local_wifi_result->current_channel               = buffer[local_index_start + 72];
        local_wifi_result->country_code[0]               = buffer[local_index_start + 73];
        local_wifi_result->country_code[1]               = buffer[local_index_start + 74];
        local_wifi_result->io_regulation                 = buffer[local_index_start + 75];
        local_wifi_result->fcs_check_byte.is_fcs_checked = ( ( buffer[local_index_start + 76] & 0x01 ) == 0x01 );
        local_wifi_result->fcs_check_byte.is_fcs_ok      = ( ( buffer[local_index_start + 76] & 0x02 ) == 0x02 );
        local_wifi_result->phi_offset                    = uint16_from_array( buffer, local_index_start + 77 );
    }
}

bool lr11xx_wifi_is_well_formed_utf8_byte_sequence( const uint8_t* buffer, const uint8_t length )
{
    uint8_t index = 0;

    while( index < length )
    {
        if( IS_BETWEEN( buffer[index], 0x00, 0x7F ) )
        {
            index += 1;
            continue;
        }

        if( length - index >= 2 )
        {
            if( IS_BETWEEN( buffer[index], 0xC2, 0xDF ) && IS_BETWEEN_0x80_AND_0xBF( buffer[index + 1] ) )
            {
                index += 2;
                continue;
            }

            if( length - index >= 3 )
            {
                if( ( buffer[index] == 0xE0 ) && IS_BETWEEN( buffer[index + 1], 0xA0, 0xBF ) &&
                    IS_BETWEEN_0x80_AND_0xBF( buffer[index + 2] ) )
                {
                    index += 3;
                    continue;
                }
                else if( IS_BETWEEN( buffer[index], 0xE1, 0xEC ) && IS_BETWEEN_0x80_AND_0xBF( buffer[index + 1] ) &&
                         IS_BETWEEN_0x80_AND_0xBF( buffer[index + 2] ) )
                {
                    index += 3;
                    continue;
                }
                else if( ( buffer[index] == 0xED ) && IS_BETWEEN( buffer[index + 1], 0x80, 0x9F ) &&
                         IS_BETWEEN_0x80_AND_0xBF( buffer[index + 2] ) )
                {
                    index += 3;
                    continue;
                }
                else if( IS_BETWEEN( buffer[index], 0xEE, 0xEF ) && IS_BETWEEN_0x80_AND_0xBF( buffer[index + 1] ) &&
                         IS_BETWEEN_0x80_AND_0xBF( buffer[index + 2] ) )
                {
                    index += 3;
                    continue;
                }

                if( length - index >= 4 )
                {
                    if( ( buffer[index] == 0xF0 ) && IS_BETWEEN( buffer[index + 1], 0x90, 0xBF ) &&
                        IS_BETWEEN_0x80_AND_0xBF( buffer[index + 2] ) && IS_BETWEEN_0x80_AND_0xBF( buffer[index + 3] ) )
                    {
                        index += 4;
                        continue;
                    }
                    else if( IS_BETWEEN( buffer[index], 0xF1, 0xF3 ) && IS_BETWEEN_0x80_AND_0xBF( buffer[index + 1] ) &&
                             IS_BETWEEN_0x80_AND_0xBF( buffer[index + 2] ) &&
                             IS_BETWEEN_0x80_AND_0xBF( buffer[index + 3] ) )
                    {
                        index += 4;
                        continue;
                    }
                    else if( ( buffer[index] == 0xF4 ) && IS_BETWEEN( buffer[index + 1], 0x80, 0x8F ) &&
                             IS_BETWEEN_0x80_AND_0xBF( buffer[index + 2] ) &&
                             IS_BETWEEN_0x80_AND_0xBF( buffer[index + 3] ) )
                    {
                        index += 4;
                        continue;
                    }
                }
            }
        }

        return false;
    }

    return true;
}

bool lr11xx_wifi_are_scan_mode_result_format_compatible( lr11xx_wifi_mode_t          scan_mode,
                                                         lr11xx_wifi_result_format_t result_format )
{
    switch( scan_mode )
    {
    case LR11XX_WIFI_SCAN_MODE_BEACON:
    case LR11XX_WIFI_SCAN_MODE_BEACON_AND_PKT:
    {
        switch( result_format )
        {
        case LR11XX_WIFI_RESULT_FORMAT_BASIC_COMPLETE:
        case LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL:
        {
            return true;
        }

        default:
        {
            return false;
        }
        }
        break;
    }

    case LR11XX_WIFI_SCAN_MODE_FULL_BEACON:
    case LR11XX_WIFI_SCAN_MODE_UNTIL_SSID:
    {
        switch( result_format )
        {
        case LR11XX_WIFI_RESULT_FORMAT_EXTENDED_FULL:
        {
            return true;
        }
        default:
        {
            return false;
        }
        }
    }

    default:
    {
        return false;
    }
    }
}

/* --- EOF ------------------------------------------------------------------ */
