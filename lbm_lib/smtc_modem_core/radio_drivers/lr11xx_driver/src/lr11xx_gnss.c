/*!
 * @file      lr11xx_gnss.c
 *
 * @brief     GNSS scan driver implementation for LR11XX
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

#include "lr11xx_gnss.h"
#include "lr11xx_regmem.h"
#include "lr11xx_system_types.h"
#include "lr11xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR11XX_GNSS_READ_GNSS_RSSI_TEST_CMD_LENGTH ( 2 + 2 )
#define LR11XX_GNSS_SET_CONSTELLATION_CMD_LENGTH ( 2 + 1 )
#define LR11XX_GNSS_READ_CONSTELLATION_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_SET_ALMANAC_UPDATE_CMD_LENGTH ( 2 + 1 )
#define LR11XX_GNSS_READ_ALMANAC_UPDATE_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_SET_FREQ_SEARCH_SPACE_CMD_LENGTH ( 2 + 1 )
#define LR11XX_GNSS_READ_FREQ_SEARCH_SPACE_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_READ_FW_VERSION_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_READ_SUPPORTED_CONSTELLATION_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_SET_SCAN_MODE_CMD_LENGTH ( 2 + 1 )
#define LR11XX_GNSS_SCAN_CMD_LENGTH ( 2 + 3 )
#define LR11XX_GNSS_SCAN_GET_RES_SIZE_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_SCAN_READ_RES_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_ALMANAC_UPDATE_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_ALMANAC_READ_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_SET_ASSISTANCE_POSITION_CMD_LENGTH ( 2 + 4 )
#define LR11XX_GNSS_READ_ASSISTANCE_POSITION_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_PUSH_SOLVER_MSG_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_PUSH_DM_MSG_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_GET_CONTEXT_STATUS_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_GET_NB_SV_SATELLITES_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_GET_SV_SATELLITES_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_READ_ALMANAC_PER_SATELLITE_CMD_LENGTH ( 2 + 2 )
#define LR11XX_GNSS_GET_SV_VISIBLE_CMD_LENGTH ( 2 + 9 )
#define LR11XX_GNSS_GET_SV_VISIBLE_DOPPLER_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_FETCH_TIME_CMD_LENGTH ( 2 + 2 )
#define LR11XX_GNSS_READ_TIME_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_RESET_TIME_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_RESET_POSITION_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_READ_WEEK_NUMBER_ROLLOVER_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_READ_DOPPLER_SOLVER_RESULT_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_READ_ALMANAC_STATUS_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_ALMANAC_UPDATE_FROM_SAT_CMD_LENGTH ( 2 + 2 )
#define LR11XX_GNSS_READ_DEMOD_STATUS_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_READ_CUMULATIVE_TIMING_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_SET_TIME_CMD_LENGTH ( 2 + 6 )
#define LR11XX_GNSS_CONFIG_DELAY_RESET_AP_CMD_LENGTH ( 2 + 3 )
#define LR11XX_GNSS_CONFIG_READ_RESET_AP_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_CONFIG_READ_KEEP_SYNC_STATUS_CMD_LENGTH ( 2 + 1 )
#define LR11XX_GNSS_CONFIG_ALMANAC_UPDATE_PERIOD_CMD_LENGTH ( 2 + 4 )
#define LR11XX_GNSS_READ_ALMANAC_UPDATE_PERIOD_CMD_LENGTH ( 2 + 2 )
#define LR11XX_GNSS_GET_SV_SYNC_CMD_LENGTH ( 2 + 1 )
#define LR11XX_GNSS_WRITE_GPS_BIT_MASK_SAT_ACTIVATED_CMD_LENGTH ( 2 + 5 )
#define LR11XX_GNSS_WRITE_BEIDOU_BIT_MASK_SAT_ACTIVATED_CMD_LENGTH ( 2 + 9 )
#define LR11XX_GNSS_READ_LAST_SCAN_MODE_LAUNCHED_CMD_LENGTH ( 2 )

#define LR11XX_GNSS_READ_GNSS_RSSI_TEST_READ_RBUFFER_LENGTH ( 2 )
#define LR11XX_GNSS_ALMANAC_READ_RBUFFER_LENGTH ( 6 )
#define LR11XX_GNSS_ALMANAC_DATE_LENGTH ( 2 )
#define LR11XX_GNSS_ALMANAC_UPDATE_MAX_NB_OF_BLOCKS \
    ( ( LR11XX_CMD_LENGTH_MAX - LR11XX_GNSS_ALMANAC_UPDATE_CMD_LENGTH ) / LR11XX_GNSS_SINGLE_ALMANAC_WRITE_SIZE )
#define LR11XX_GNSS_READ_ALMANAC_TEMPBUFFER_SIZE_BYTE ( 47 )
#define LR11XX_GNSS_MAX_DETECTED_SV ( 32 )
#define LR11XX_GNSS_DETECTED_SV_SINGLE_LENGTH ( 4 )
#define LR11XX_GNSS_MAX_DETECTED_SV_BUFFER_LENGTH \
    ( LR11XX_GNSS_MAX_DETECTED_SV * LR11XX_GNSS_DETECTED_SV_SINGLE_LENGTH )
#define LR11XX_GNSS_READ_FIRMWARE_VERSION_RBUFFER_LENGTH ( 2 )
#define LR11XX_GNSS_READ_TIME_RBUFFER_LENGTH ( 12 )
#define LR11XX_GNSS_READ_WEEK_NUMBER_ROLLOVER_RBUFFER_LENGTH ( 2 )
#define LR11XX_GNSS_READ_DEMOD_STATUS_RBUFFER_LENGTH ( 2 )
#define LR11XX_GNSS_READ_CUMULATIVE_TIMING_RBUFFER_LENGTH ( 125 )
#define LR11XX_GNSS_DOPPLER_SOLVER_RES_RBUFFER_LENGTH ( 18 )
#define LR11XX_GNSS_READ_DELAY_RESET_AP_RBUFFER_LENGTH ( 3 )
#define LR11XX_GNSS_READ_ALMANAC_STATUS_RBUFFER_LENGTH ( 53 )
#define LR11XX_GNSS_READ_KEEP_SYNC_STATUS_RBUFFER_LENGTH ( 5 )
#define LR11XX_GNSS_READ_ALMANAC_UPDATE_PERIOD_RBUFFER_LENGTH ( 2 )

#define LR11XX_GNSS_SCALING_LATITUDE 90
#define LR11XX_GNSS_SCALING_LONGITUDE 180
#define LR11XX_GNSS_SNR_TO_CNR_OFFSET ( 31 )

#define LR11XX_GNSS_SCAN_RESULT_DESTINATION_INDEX ( 0 )

/*!
 * @brief GNSS scan power consumption
 *
 * @note these numbers are given for information, it should be modified according to the used hardware.
 */
#define LR11XX_GNSS_RADIO_ACQUISITION_GPS_UA_DCDC ( 15000 )
#define LR11XX_GNSS_RADIO_ACQUISITION_BEIDOU_UA_DCDC ( 16500 )
#define LR11XX_GNSS_COMPUTATION_UA_DCDC ( 3100 )
#define LR11XX_GNSS_RADIO_ACQUISITION_GPS_UA_LDO ( 24500 )
#define LR11XX_GNSS_RADIO_ACQUISITION_BEIDOU_UA_LDO ( 27300 )
#define LR11XX_GNSS_COMPUTATION_UA_LDO ( 5000 )
#define LR11XX_GNSS_LF_CLOCK_VALUE ( 32768 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for GNSS-related operations
 */
enum
{
    LR11XX_GNSS_READ_GNSS_RSSI_TEST_OC          = 0x0222,
    LR11XX_GNSS_SET_CONSTELLATION_OC            = 0x0400,  //!< Set the constellation to use
    LR11XX_GNSS_READ_CONSTELLATION_OC           = 0x0401,  //!< Read the used constellations
    LR11XX_GNSS_SET_ALMANAC_UPDATE_OC           = 0x0402,  //!< Set almanac update configuration
    LR11XX_GNSS_READ_ALMANAC_UPDATE_OC          = 0x0403,  //!< Read the almanac update configuration
    LR11XX_GNSS_SET_FREQ_SEARCH_SPACE_OC        = 0x0404,  //!< Set the frequency search space
    LR11XX_GNSS_READ_FREQ_SEARCH_SPACE_OC       = 0x0405,  //!< Read the frequency search space
    LR11XX_GNSS_READ_FW_VERSION_OC              = 0x0406,  //!< Read the firmware version
    LR11XX_GNSS_READ_SUPPORTED_CONSTELLATION_OC = 0x0407,  //!< Read the supported constellations
    LR11XX_GNSS_SET_SCAN_MODE_OC                = 0x0408,  //!< Define single or double capture
    LR11XX_GNSS_SCAN_OC                         = 0x040B,  //!< Launch the scan
    LR11XX_GNSS_SCAN_GET_RES_SIZE_OC            = 0x040C,  //!< Get the size of the output payload
    LR11XX_GNSS_SCAN_READ_RES_OC                = 0x040D,  //!< Read the byte stream
    LR11XX_GNSS_ALMANAC_UPDATE_OC               = 0x040E,  //!< Update the almanac
    LR11XX_GNSS_ALMANAC_READ_OC                 = 0x040F,  //!< Read all almanacs
    LR11XX_GNSS_SET_ASSISTANCE_POSITION_OC      = 0x0410,  //!< Set the assistance position
    LR11XX_GNSS_READ_ASSISTANCE_POSITION_OC     = 0x0411,  //!< Read the assistance position
    LR11XX_GNSS_PUSH_SOLVER_MSG_OC              = 0x0414,  //!< Push messages coming from the solver
    LR11XX_GNSS_PUSH_DM_MSG_OC                  = 0x0415,  //!< Push messages coming from the device management
    LR11XX_GNSS_GET_CONTEXT_STATUS_OC           = 0x0416,  //!< Read the context
    LR11XX_GNSS_GET_NB_SATELLITES_OC            = 0x0417,  //!< Get the number of satellites detected during a scan
    LR11XX_GNSS_GET_SATELLITES_OC               = 0x0418,  //!< Get the list of satellites detected during a scan
    LR11XX_GNSS_READ_ALMANAC_PER_SATELLITE_OC   = 0x041A,  //!< Read the almanac of given satellites
    LR11XX_GNSS_GET_SV_VISIBLE_OC               = 0x041F,  //!< Get the number of visible SV from a date and a position
    LR11XX_GNSS_GET_SV_VISIBLE_DOPPLER_OC       = 0x0420,  //!< Get visible SV ID and corresponding doppler value
    LR11XX_GNSS_READ_LAST_SCAN_MODE_LAUNCHED_OC = 0x0426,  //!< Get the type of scan launched during the last scan
    LR11XX_GNSS_FETCH_TIME_OC                   = 0x0432,  //!< Start the time acquisition/domulation
    LR11XX_GNSS_READ_TIME_OC                    = 0x0434,  //!< Read time from LR11XX
    LR11XX_GNSS_RESET_TIME_OC                   = 0x0435,  //!< Reset the internal time
    LR11XX_GNSS_RESET_POSITION_OC               = 0x0437,  //!< Reset the location and the history Doppler buffer
    LR11XX_GNSS_READ_WEEK_NUMBER_ROLLOVER_OC    = 0x0438,  //!< Read the week number rollover
    LR11XX_GNSS_READ_DEMOD_STATUS_OC            = 0x0439,  //!< Read demod status
    LR11XX_GNSS_READ_CUMULATIVE_TIMING_OC       = 0x044A,  //!< Read cumulative timing
    LR11XX_GNSS_SET_TIME_OC                     = 0x044B,  //!< Set the GPS time
    LR11XX_GNSS_CONFIG_DELAY_RESET_AP_OC        = 0x044D,  //!< Configures the time delay in sec
    LR11XX_GNSS_READ_DOPPLER_SOLVER_RESULT_OC =
        0x044F,                                   //!< Read the assisted position based on the internal doppler solver
    LR11XX_GNSS_READ_DELAY_RESET_AP_OC = 0x0453,  //!< Read the time delay in sec
    LR11XX_GNSS_ALMANAC_UPDATE_FROM_SAT_OC =
        0x0454,  //!< Launches one scan to download from satellite almanac parameters broadcasted
    LR11XX_GNSS_READ_KEEP_SYNC_STATUS_OC = 0x0456,  //!< Read the number of visible satellites and the time elapsed from
                                                    //!< last detected satellite list update
    LR11XX_GNSS_READ_ALMANAC_STATUS_OC          = 0x0457,  //!< Returns the actual state of almanac GPS and Beidou.
    LR11XX_GNSS_CONFIG_ALMANAC_UPDATE_PERIOD_OC = 0x0463,  //!< Configures the almanac update period
    LR11XX_GNSS_READ_ALMANAC_UPDATE_PERIOD_OC   = 0x0464,  //!< Read the almanac update period
    LR11XX_GNSS_GET_SV_SYNC_OC = 0x0466,  //!< Returns the list of satellite for the next keep sync scan
    LR11XX_GNSS_WRITE_BIT_MASK_SAT_ACTIVATED_OC =
        0x0472,  //!< Configures the ability of the LR11xx to search almanac for each satellites
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
 * @brief Helper function that convert an array of uint8_t into a uint32_t single value
 *
 * @warning It is up to the caller to ensure that value points to an array of at least sizeof(uint32_t) elements.
 *
 * @param [in] value Array of uint8_t to be translated into a uint32_t
 *
 * @returns 32-bit value
 */
static uint32_t lr11xx_gnss_uint8_to_uint32( uint8_t value[4] );

/*!
 * @brief Returns the minimum of the operand given as parameter and the maximum allowed number of blocks
 *
 * @param [in] operand Size to compare
 *
 * @returns Minimum between operand and @ref LR11XX_GNSS_ALMANAC_UPDATE_MAX_NB_OF_BLOCKS
 */
static uint16_t lr11xx_gnss_get_min_from_operand_and_max_nb_of_blocks( uint16_t operand );

/*!
 * @brief Get the almanac base address and size
 *
 * @param [in]  context Chip implementation context
 * @param [out] address Start address of the almanac in memory
 * @param [out] size    Size of the almanac in byte
 *
 * @returns Operation status
 */
static lr11xx_status_t lr11xx_gnss_get_almanac_address_size( const void* context, uint32_t* address, uint16_t* size );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_status_t lr11xx_gnss_get_result_size( const void* context, uint16_t* result_size )
{
    const uint8_t cbuffer[LR11XX_GNSS_SCAN_GET_RES_SIZE_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_SCAN_GET_RES_SIZE_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_SCAN_GET_RES_SIZE_OC >> 0 ),
    };
    uint8_t rbuffer[sizeof( uint16_t )] = { 0 };

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_SCAN_GET_RES_SIZE_CMD_LENGTH, rbuffer, sizeof( uint16_t ) );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *result_size = ( ( uint16_t ) rbuffer[0] << 8 ) + ( ( uint16_t ) rbuffer[1] );
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_gnss_read_results( const void* context, uint8_t* result_buffer,
                                          const uint16_t result_buffer_size )
{
    const uint8_t cbuffer[LR11XX_GNSS_SCAN_READ_RES_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_SCAN_READ_RES_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_SCAN_READ_RES_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_SCAN_READ_RES_CMD_LENGTH, result_buffer,
                                                result_buffer_size );
}

lr11xx_status_t lr11xx_gnss_almanac_update( const void* context, const uint8_t* blocks, const uint8_t nb_of_blocks )
{
    const uint8_t cbuffer[LR11XX_GNSS_ALMANAC_UPDATE_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_ALMANAC_UPDATE_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_ALMANAC_UPDATE_OC >> 0 ),
    };

    uint16_t remaining_nb_of_blocks = nb_of_blocks;

    while( remaining_nb_of_blocks > 0 )
    {
        const uint16_t nb_of_blocks_to_write =
            lr11xx_gnss_get_min_from_operand_and_max_nb_of_blocks( remaining_nb_of_blocks );

        const uint8_t* blocks_to_write =
            blocks + ( nb_of_blocks - remaining_nb_of_blocks ) * LR11XX_GNSS_SINGLE_ALMANAC_WRITE_SIZE;

        const lr11xx_hal_status_t status =
            lr11xx_hal_write( context, cbuffer, LR11XX_GNSS_ALMANAC_UPDATE_CMD_LENGTH, blocks_to_write,
                              nb_of_blocks_to_write * LR11XX_GNSS_SINGLE_ALMANAC_WRITE_SIZE );

        if( status != LR11XX_HAL_STATUS_OK )
        {
            return ( lr11xx_status_t ) status;
        }

        remaining_nb_of_blocks -= nb_of_blocks_to_write;
    }

    return LR11XX_STATUS_OK;
}

lr11xx_status_t lr11xx_gnss_read_almanac( const void*                                context,
                                          lr11xx_gnss_almanac_full_read_bytestream_t almanac_bytestream )
{
    uint32_t        almanac_address = 0;
    uint16_t        almanac_size    = 0;
    lr11xx_status_t status          = lr11xx_gnss_get_almanac_address_size( context, &almanac_address, &almanac_size );
    if( status != LR11XX_STATUS_OK )
    {
        return status;
    }

    const uint8_t N_READ_ALMANAC_REGMEM32 = 15;

    for( uint8_t index_regmem32 = 0; index_regmem32 < N_READ_ALMANAC_REGMEM32; index_regmem32++ )
    {
        const uint16_t local_bytestream_index_burst =
            index_regmem32 * LR11XX_GNSS_READ_ALMANAC_TEMPBUFFER_SIZE_BYTE * 4;
        uint32_t temporary_buffer[LR11XX_GNSS_READ_ALMANAC_TEMPBUFFER_SIZE_BYTE] = { 0 };

        const lr11xx_status_t local_status = lr11xx_regmem_read_regmem32(
            context, almanac_address, temporary_buffer, LR11XX_GNSS_READ_ALMANAC_TEMPBUFFER_SIZE_BYTE );
        if( local_status != LR11XX_STATUS_OK )
        {
            return local_status;
        }

        almanac_address += ( LR11XX_GNSS_READ_ALMANAC_TEMPBUFFER_SIZE_BYTE * 4 );

        for( uint8_t index_local_temp = 0; index_local_temp < LR11XX_GNSS_READ_ALMANAC_TEMPBUFFER_SIZE_BYTE;
             index_local_temp++ )
        {
            const uint16_t local_bytestream_index          = local_bytestream_index_burst + ( index_local_temp * 4 );
            almanac_bytestream[local_bytestream_index + 0] = ( uint8_t )( temporary_buffer[index_local_temp] >> 0 );
            almanac_bytestream[local_bytestream_index + 1] = ( uint8_t )( temporary_buffer[index_local_temp] >> 8 );
            almanac_bytestream[local_bytestream_index + 2] = ( uint8_t )( temporary_buffer[index_local_temp] >> 16 );
            almanac_bytestream[local_bytestream_index + 3] = ( uint8_t )( temporary_buffer[index_local_temp] >> 24 );
        }
    }
    return status;
}

lr11xx_status_t lr11xx_gnss_get_almanac_age_for_satellite( const void* context, const lr11xx_gnss_satellite_id_t sv_id,
                                                           uint16_t* almanac_age )
{
    uint32_t              almanac_base_address = 0;
    uint16_t              almanac_size         = 0;
    const lr11xx_status_t status_get_almanac_address_size =
        lr11xx_gnss_get_almanac_address_size( context, &almanac_base_address, &almanac_size );

    if( status_get_almanac_address_size != LR11XX_STATUS_OK )
    {
        return status_get_almanac_address_size;
    }

    const uint16_t offset_almanac_date                               = sv_id * LR11XX_GNSS_SINGLE_ALMANAC_READ_SIZE + 1;
    uint8_t        raw_almanac_date[LR11XX_GNSS_ALMANAC_DATE_LENGTH] = { 0 };

    const lr11xx_status_t status_read_mem = lr11xx_regmem_read_mem8(
        context, almanac_base_address + offset_almanac_date, raw_almanac_date, LR11XX_GNSS_ALMANAC_DATE_LENGTH );
    if( status_read_mem == LR11XX_STATUS_OK )
    {
        // Note: the memory on LR11XX is LSB first. As the 2-byte wide almanac age is obtained by calling the _mem8, the
        // conversion to uint16_t here is done LSB first
        ( *almanac_age ) = ( ( ( uint16_t ) raw_almanac_date[1] ) << 8 ) + ( ( ( uint16_t ) raw_almanac_date[0] ) );
    }
    return status_read_mem;
}

lr11xx_status_t lr11xx_gnss_get_almanac_address_size( const void* context, uint32_t* address, uint16_t* size )
{
    const uint8_t cbuffer[LR11XX_GNSS_ALMANAC_READ_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_ALMANAC_READ_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_ALMANAC_READ_OC >> 0 ),
    };
    uint8_t rbuffer[LR11XX_GNSS_ALMANAC_READ_RBUFFER_LENGTH] = { 0 };

    const lr11xx_hal_status_t hal_status = lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_ALMANAC_READ_CMD_LENGTH,
                                                            rbuffer, LR11XX_GNSS_ALMANAC_READ_RBUFFER_LENGTH );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *address = lr11xx_gnss_uint8_to_uint32( &rbuffer[0] );
        *size    = ( ( ( uint16_t ) rbuffer[4] ) << 8 ) | ( ( uint16_t ) rbuffer[5] );
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_gnss_push_solver_msg( const void* context, const uint8_t* payload, const uint16_t payload_size )
{
    const uint8_t cbuffer[LR11XX_GNSS_PUSH_SOLVER_MSG_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_PUSH_SOLVER_MSG_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_PUSH_SOLVER_MSG_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_GNSS_PUSH_SOLVER_MSG_CMD_LENGTH, payload,
                                                 payload_size );
}

lr11xx_status_t lr11xx_gnss_set_constellations_to_use( const void*                            context,
                                                       const lr11xx_gnss_constellation_mask_t constellation_to_use )
{
    const uint8_t cbuffer[LR11XX_GNSS_SET_CONSTELLATION_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_SET_CONSTELLATION_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_SET_CONSTELLATION_OC >> 0 ),
        ( uint8_t ) constellation_to_use,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_GNSS_SET_CONSTELLATION_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_gnss_read_used_constellations( const void*                       context,
                                                      lr11xx_gnss_constellation_mask_t* constellations_used )
{
    const uint8_t cbuffer[LR11XX_GNSS_READ_CONSTELLATION_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_READ_CONSTELLATION_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_CONSTELLATION_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_READ_CONSTELLATION_CMD_LENGTH,
                                                constellations_used, sizeof( *constellations_used ) );
}

lr11xx_status_t lr11xx_gnss_set_almanac_update( const void*                            context,
                                                const lr11xx_gnss_constellation_mask_t constellations_to_update )
{
    const uint8_t cbuffer[LR11XX_GNSS_SET_ALMANAC_UPDATE_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_SET_ALMANAC_UPDATE_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_SET_ALMANAC_UPDATE_OC >> 0 ),
        ( uint8_t ) constellations_to_update,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_GNSS_SET_ALMANAC_UPDATE_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_gnss_read_almanac_update( const void*                       context,
                                                 lr11xx_gnss_constellation_mask_t* constellations_to_update )
{
    const uint8_t cbuffer[LR11XX_GNSS_READ_ALMANAC_UPDATE_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_READ_ALMANAC_UPDATE_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_ALMANAC_UPDATE_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_READ_ALMANAC_UPDATE_CMD_LENGTH,
                                                constellations_to_update, sizeof( *constellations_to_update ) );
}

lr11xx_status_t lr11xx_gnss_set_freq_search_space( const void*                           radio,
                                                   const lr11xx_gnss_freq_search_space_t freq_search_space )
{
    const uint8_t cbuffer[LR11XX_GNSS_SET_FREQ_SEARCH_SPACE_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_SET_FREQ_SEARCH_SPACE_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_SET_FREQ_SEARCH_SPACE_OC >> 0 ),
        ( uint8_t ) freq_search_space,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( radio, cbuffer, LR11XX_GNSS_SET_FREQ_SEARCH_SPACE_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_gnss_read_freq_search_space( const void*                      radio,
                                                    lr11xx_gnss_freq_search_space_t* freq_search_space )
{
    const uint8_t cbuffer[LR11XX_GNSS_READ_FREQ_SEARCH_SPACE_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_READ_FREQ_SEARCH_SPACE_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_FREQ_SEARCH_SPACE_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( radio, cbuffer, LR11XX_GNSS_READ_FREQ_SEARCH_SPACE_CMD_LENGTH,
                                                ( uint8_t* ) freq_search_space, sizeof( uint8_t ) );
}

lr11xx_status_t lr11xx_gnss_read_firmware_version( const void* context, lr11xx_gnss_version_t* version )
{
    const uint8_t cbuffer[LR11XX_GNSS_READ_FW_VERSION_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_READ_FW_VERSION_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_FW_VERSION_OC >> 0 ),
    };
    uint8_t rbuffer[LR11XX_GNSS_READ_FIRMWARE_VERSION_RBUFFER_LENGTH] = { 0 };

    const lr11xx_hal_status_t hal_status = lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_READ_FW_VERSION_CMD_LENGTH,
                                                            rbuffer, LR11XX_GNSS_READ_FIRMWARE_VERSION_RBUFFER_LENGTH );

    version->gnss_firmware = rbuffer[0];
    version->gnss_almanac  = rbuffer[1];
    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_gnss_read_supported_constellations( const void*                       context,
                                                           lr11xx_gnss_constellation_mask_t* supported_constellations )
{
    const uint8_t cbuffer[LR11XX_GNSS_READ_SUPPORTED_CONSTELLATION_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_READ_SUPPORTED_CONSTELLATION_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_SUPPORTED_CONSTELLATION_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_READ_SUPPORTED_CONSTELLATION_CMD_LENGTH,
                                                supported_constellations, sizeof( *supported_constellations ) );
}

lr11xx_status_t lr11xx_gnss_set_scan_mode( const void* context, const lr11xx_gnss_scan_mode_t scan_mode )
{
    const uint8_t cbuffer[LR11XX_GNSS_SET_SCAN_MODE_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_SET_SCAN_MODE_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_SET_SCAN_MODE_OC >> 0 ),
        ( uint8_t ) scan_mode,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_GNSS_SET_SCAN_MODE_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_gnss_scan( const void* context, const lr11xx_gnss_search_mode_t effort_mode,
                                  const uint8_t gnss_input_parameters, const uint8_t nb_sat )
{
    const uint8_t cbuffer[LR11XX_GNSS_SCAN_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_SCAN_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_SCAN_OC >> 0 ),
        ( uint8_t ) effort_mode,
        gnss_input_parameters,
        nb_sat,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_GNSS_SCAN_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_gnss_set_assistance_position(
    const void* context, const lr11xx_gnss_solver_assistance_position_t* assistance_position )
{
    const int16_t latitude  = ( ( assistance_position->latitude * 2048 ) / LR11XX_GNSS_SCALING_LATITUDE );
    const int16_t longitude = ( ( assistance_position->longitude * 2048 ) / LR11XX_GNSS_SCALING_LONGITUDE );
    const uint8_t cbuffer[LR11XX_GNSS_SET_ASSISTANCE_POSITION_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_SET_ASSISTANCE_POSITION_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_SET_ASSISTANCE_POSITION_OC >> 0 ),
        ( uint8_t )( latitude >> 8 ),
        ( uint8_t )( latitude >> 0 ),
        ( uint8_t )( longitude >> 8 ),
        ( uint8_t )( longitude >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_GNSS_SET_ASSISTANCE_POSITION_CMD_LENGTH, 0,
                                                 0 );
}

lr11xx_status_t lr11xx_gnss_read_assistance_position( const void*                               context,
                                                      lr11xx_gnss_solver_assistance_position_t* assistance_position )
{
    uint8_t       position_buffer[4] = { 0x00 };
    int16_t       position_tmp;
    const uint8_t cbuffer[LR11XX_GNSS_READ_ASSISTANCE_POSITION_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_READ_ASSISTANCE_POSITION_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_ASSISTANCE_POSITION_OC >> 0 ),
    };

    const lr11xx_hal_status_t hal_status = lr11xx_hal_read(
        context, cbuffer, LR11XX_GNSS_READ_ASSISTANCE_POSITION_CMD_LENGTH, position_buffer, sizeof( position_buffer ) );

    position_tmp                  = ( int16_t )( ( ( uint16_t ) position_buffer[0] << 8 ) + position_buffer[1] );
    assistance_position->latitude = ( ( float ) ( position_tmp ) *LR11XX_GNSS_SCALING_LATITUDE ) / 2048;

    position_tmp                   = ( int16_t )( ( ( uint16_t ) position_buffer[2] << 8 ) + position_buffer[3] );
    assistance_position->longitude = ( ( float ) ( position_tmp ) *LR11XX_GNSS_SCALING_LONGITUDE ) / 2048;

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_gnss_push_dmc_msg( const void* context, uint8_t* dmc_msg, uint16_t dmc_msg_len )
{
    const uint8_t cbuffer[LR11XX_GNSS_PUSH_DM_MSG_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_PUSH_DM_MSG_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_PUSH_DM_MSG_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_GNSS_PUSH_DM_MSG_CMD_LENGTH, dmc_msg,
                                                 dmc_msg_len );
}

lr11xx_status_t lr11xx_gnss_get_context_status( const void*                             context,
                                                lr11xx_gnss_context_status_bytestream_t context_status )
{
    const uint8_t cbuffer[LR11XX_GNSS_GET_CONTEXT_STATUS_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_GET_CONTEXT_STATUS_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_GET_CONTEXT_STATUS_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_GET_CONTEXT_STATUS_CMD_LENGTH,
                                                context_status, LR11XX_GNSS_CONTEXT_STATUS_LENGTH );
}

lr11xx_status_t lr11xx_gnss_get_nb_detected_satellites( const void* context, uint8_t* nb_detected_satellites )
{
    const uint8_t cbuffer[LR11XX_GNSS_GET_NB_SV_SATELLITES_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_GET_NB_SATELLITES_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_GET_NB_SATELLITES_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_GET_NB_SV_SATELLITES_CMD_LENGTH,
                                                nb_detected_satellites, 1 );
}

lr11xx_status_t lr11xx_gnss_get_detected_satellites(
    const void* context, const uint8_t nb_detected_satellites,
    lr11xx_gnss_detected_satellite_t* detected_satellite_id_snr_doppler )
{
    const uint8_t max_satellites_to_fetch =
        ( LR11XX_GNSS_MAX_DETECTED_SV > nb_detected_satellites ) ? nb_detected_satellites : LR11XX_GNSS_MAX_DETECTED_SV;
    const uint16_t read_size = max_satellites_to_fetch * LR11XX_GNSS_DETECTED_SV_SINGLE_LENGTH;
    uint8_t        result_buffer[LR11XX_GNSS_MAX_DETECTED_SV_BUFFER_LENGTH] = { 0 };

    const uint8_t cbuffer[LR11XX_GNSS_GET_SV_SATELLITES_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_GET_SATELLITES_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_GET_SATELLITES_OC >> 0 ),
    };

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_GET_SV_SATELLITES_CMD_LENGTH, result_buffer, read_size );
    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        for( uint8_t index_satellite = 0; index_satellite < max_satellites_to_fetch; index_satellite++ )
        {
            const uint16_t local_result_buffer_index = index_satellite * LR11XX_GNSS_DETECTED_SV_SINGLE_LENGTH;
            lr11xx_gnss_detected_satellite_t* local_satellite_result =
                &detected_satellite_id_snr_doppler[index_satellite];

            local_satellite_result->satellite_id = result_buffer[local_result_buffer_index];
            local_satellite_result->cnr = result_buffer[local_result_buffer_index + 1] + LR11XX_GNSS_SNR_TO_CNR_OFFSET;
            local_satellite_result->doppler = ( int16_t )( ( result_buffer[local_result_buffer_index + 2] << 8 ) +
                                                           ( result_buffer[local_result_buffer_index + 3] ) );
        }
    }
    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_gnss_read_almanac_per_satellites( const void* context, uint8_t sv_id_init, uint8_t n_sv,
                                                         uint8_t* almanacs )
{
    const uint8_t cbuffer[LR11XX_GNSS_READ_ALMANAC_PER_SATELLITE_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_READ_ALMANAC_PER_SATELLITE_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_ALMANAC_PER_SATELLITE_OC >> 0 ), sv_id_init, n_sv
    };

    const uint16_t n_bytes_to_read = n_sv * LR11XX_GNSS_SINGLE_ALMANAC_READ_SIZE;

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_READ_ALMANAC_PER_SATELLITE_CMD_LENGTH,
                                                almanacs, n_bytes_to_read );
}

lr11xx_status_t lr11xx_gnss_read_gnss_rssi_test( const void* context, int8_t* rssi_gnss_dbm )
{
    const uint8_t gnss_rssi_path_test                                 = 0x09;
    const uint8_t cbuffer[LR11XX_GNSS_READ_GNSS_RSSI_TEST_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_READ_GNSS_RSSI_TEST_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_GNSS_RSSI_TEST_OC >> 0 ),
        gnss_rssi_path_test,
        0,
    };

    uint8_t                   rssi_gnss_raw[LR11XX_GNSS_READ_GNSS_RSSI_TEST_READ_RBUFFER_LENGTH] = { 0 };
    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_READ_GNSS_RSSI_TEST_CMD_LENGTH, rssi_gnss_raw,
                         LR11XX_GNSS_READ_GNSS_RSSI_TEST_READ_RBUFFER_LENGTH );
    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *rssi_gnss_dbm = rssi_gnss_raw[1];
    }
    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_gnss_parse_context_status_buffer(
    const lr11xx_gnss_context_status_bytestream_t context_status_bytestream,
    lr11xx_gnss_context_status_t*                 context_status )
{
    lr11xx_status_t status = LR11XX_STATUS_ERROR;

    if( ( ( lr11xx_gnss_destination_t ) context_status_bytestream[0] == LR11XX_GNSS_DESTINATION_DMC ) &&
        ( ( lr11xx_gnss_message_dmc_opcode_t ) context_status_bytestream[1] == LR11XX_GNSS_DMC_STATUS ) )
    {
        context_status->firmware_version = context_status_bytestream[2];

        context_status->global_almanac_crc =
            ( ( uint32_t ) context_status_bytestream[3] ) + ( ( uint32_t ) context_status_bytestream[4] << 8 ) +
            ( ( uint32_t ) context_status_bytestream[5] << 16 ) + ( ( uint32_t ) context_status_bytestream[6] << 24 );

        context_status->error_code = ( lr11xx_gnss_error_code_t )( context_status_bytestream[7] >> 4 );

        context_status->almanac_update_gps =
            ( ( context_status_bytestream[7] & LR11XX_GNSS_DMC_ALMANAC_UPDATE_GPS_MASK ) != 0 ) ? true : false;

        context_status->almanac_update_beidou =
            ( ( context_status_bytestream[7] & LR11XX_GNSS_DMC_ALMANAC_UPDATE_BEIDOU_MASK ) != 0 ) ? true : false;

        context_status->freq_search_space = ( lr11xx_gnss_freq_search_space_t )(
            ( ( ( context_status_bytestream[7] & LR11XX_GNSS_DMC_FREQUENCY_SEARCH_SPACE_MSB_MASK ) >>
                LR11XX_GNSS_DMC_FREQUENCY_SEARCH_SPACE_MSB_POS )
              << 1 ) +
            ( ( context_status_bytestream[8] & LR11XX_GNSS_DMC_FREQUENCY_SEARCH_SPACE_LSB_MASK ) >>
              LR11XX_GNSS_DMC_FREQUENCY_SEARCH_SPACE_LSB_POS ) );

        status = LR11XX_STATUS_OK;
    }

    return status;
}

lr11xx_status_t lr11xx_gnss_get_result_destination( const uint8_t* result_buffer, const uint16_t result_buffer_size,
                                                    lr11xx_gnss_destination_t* destination )
{
    lr11xx_status_t status = LR11XX_STATUS_ERROR;

    if( result_buffer_size != 0 )
    {
        switch( result_buffer[LR11XX_GNSS_SCAN_RESULT_DESTINATION_INDEX] )
        {
        case LR11XX_GNSS_DESTINATION_HOST:
        {
            *destination = LR11XX_GNSS_DESTINATION_HOST;
            status       = LR11XX_STATUS_OK;
            break;
        }
        case LR11XX_GNSS_DESTINATION_SOLVER:
        {
            *destination = LR11XX_GNSS_DESTINATION_SOLVER;
            status       = LR11XX_STATUS_OK;
            break;
        }
        case LR11XX_GNSS_DESTINATION_DMC:
        {
            *destination = LR11XX_GNSS_DESTINATION_DMC;
            status       = LR11XX_STATUS_OK;
            break;
        }
        }
    }

    return status;
}

uint16_t lr11xx_gnss_compute_almanac_age( uint16_t almanac_date,
                                          uint16_t nb_days_between_epoch_and_last_gps_time_rollover,
                                          uint16_t nb_days_since_epoch )
{
    return nb_days_since_epoch - ( almanac_date + nb_days_between_epoch_and_last_gps_time_rollover );
}

lr11xx_status_t lr11xx_gnss_get_nb_visible_satellites(
    const void* context, const lr11xx_gnss_date_t date,
    const lr11xx_gnss_solver_assistance_position_t* assistance_position,
    const lr11xx_gnss_constellation_t constellation, uint8_t* nb_visible_sv )
{
    const int16_t latitude  = ( ( assistance_position->latitude * 2048 ) / LR11XX_GNSS_SCALING_LATITUDE );
    const int16_t longitude = ( ( assistance_position->longitude * 2048 ) / LR11XX_GNSS_SCALING_LONGITUDE );
    const uint8_t cbuffer[LR11XX_GNSS_GET_SV_VISIBLE_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_GET_SV_VISIBLE_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_GET_SV_VISIBLE_OC >> 0 ),
        ( uint8_t )( date >> 24 ),
        ( uint8_t )( date >> 16 ),
        ( uint8_t )( date >> 8 ),
        ( uint8_t )( date >> 0 ),
        ( uint8_t )( latitude >> 8 ),
        ( uint8_t )( latitude >> 0 ),
        ( uint8_t )( longitude >> 8 ),
        ( uint8_t )( longitude >> 0 ),
        ( uint8_t )( constellation - 1 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_GET_SV_VISIBLE_CMD_LENGTH, nb_visible_sv,
                                                1 );
}

lr11xx_status_t lr11xx_gnss_get_visible_satellites( const void* context, const uint8_t nb_visible_satellites,
                                                    lr11xx_gnss_visible_satellite_t* visible_satellite_id_doppler )
{
    uint8_t        result_buffer[12 * 5] = { 0 };
    const uint16_t read_size             = nb_visible_satellites * 5;

    const uint8_t cbuffer[LR11XX_GNSS_GET_SV_VISIBLE_DOPPLER_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_GET_SV_VISIBLE_DOPPLER_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_GET_SV_VISIBLE_DOPPLER_OC >> 0 ),
    };

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_GET_SV_VISIBLE_DOPPLER_CMD_LENGTH, result_buffer, read_size );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        for( uint8_t index_satellite = 0; index_satellite < nb_visible_satellites; index_satellite++ )
        {
            const uint16_t                   local_result_buffer_index = index_satellite * 5;
            lr11xx_gnss_visible_satellite_t* local_satellite_result    = &visible_satellite_id_doppler[index_satellite];

            local_satellite_result->satellite_id  = result_buffer[local_result_buffer_index];
            local_satellite_result->doppler       = ( int16_t )( ( result_buffer[local_result_buffer_index + 1] << 8 ) +
                                                           ( result_buffer[local_result_buffer_index + 2] ) );
            local_satellite_result->doppler_error = ( int16_t )( ( result_buffer[local_result_buffer_index + 3] << 8 ) +
                                                                 ( result_buffer[local_result_buffer_index + 4] ) );
        }
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_gnss_fetch_time( const void* context, const lr11xx_gnss_search_mode_t effort_mode,
                                        const lr11xx_gnss_fetch_time_option_t option )
{
    const uint8_t cbuffer[LR11XX_GNSS_FETCH_TIME_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_FETCH_TIME_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_FETCH_TIME_OC >> 0 ),
        effort_mode,
        option,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_GNSS_FETCH_TIME_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_gnss_read_time( const void* context, lr11xx_gnss_time_t* time )
{
    uint8_t       rbuffer[LR11XX_GNSS_READ_TIME_RBUFFER_LENGTH] = { 0x00 };
    const uint8_t cbuffer[LR11XX_GNSS_READ_TIME_CMD_LENGTH]     = {
        ( uint8_t )( LR11XX_GNSS_READ_TIME_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_TIME_OC >> 0 ),
    };

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_READ_TIME_CMD_LENGTH, rbuffer, sizeof( rbuffer ) );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        time->error_code = ( lr11xx_gnss_read_time_status_t ) rbuffer[0];
        time->gps_time_s = lr11xx_gnss_uint8_to_uint32( &rbuffer[1] );
        time->nb_us_in_s =
            ( uint32_t )( rbuffer[5] << 16 ) | ( uint32_t )( rbuffer[6] << 8 ) | ( uint32_t )( rbuffer[7] );
        time->nb_us_in_s /= 16;
        time->time_accuracy = lr11xx_gnss_uint8_to_uint32( &rbuffer[8] );
        time->time_accuracy /= 16;
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_gnss_reset_time( const void* context )
{
    const uint8_t cbuffer[LR11XX_GNSS_RESET_TIME_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_RESET_TIME_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_RESET_TIME_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_GNSS_RESET_TIME_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_gnss_reset_position( const void* context )
{
    const uint8_t cbuffer[LR11XX_GNSS_RESET_POSITION_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_RESET_POSITION_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_RESET_POSITION_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_GNSS_RESET_POSITION_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_gnss_read_week_number_rollover( const void*                                context,
                                                       lr11xx_gnss_week_number_rollover_status_t* wn_rollover_status,
                                                       uint8_t*                                   wn_number_rollover )
{
    uint8_t       rbuffer[LR11XX_GNSS_READ_WEEK_NUMBER_ROLLOVER_RBUFFER_LENGTH] = { 0x00 };
    const uint8_t cbuffer[LR11XX_GNSS_READ_WEEK_NUMBER_ROLLOVER_CMD_LENGTH]     = {
        ( uint8_t )( LR11XX_GNSS_READ_WEEK_NUMBER_ROLLOVER_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_WEEK_NUMBER_ROLLOVER_OC >> 0 ),
    };

    const lr11xx_hal_status_t hal_status = lr11xx_hal_read(
        context, cbuffer, LR11XX_GNSS_READ_WEEK_NUMBER_ROLLOVER_CMD_LENGTH, rbuffer, sizeof( rbuffer ) );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *wn_rollover_status = ( lr11xx_gnss_week_number_rollover_status_t ) rbuffer[0];
        *wn_number_rollover = rbuffer[1];
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_gnss_read_demod_status( const void* context, lr11xx_gnss_demod_status_t* demod_status,
                                               lr11xx_gnss_demod_info_t* demod_info )
{
    uint8_t       rbuffer[LR11XX_GNSS_READ_DEMOD_STATUS_RBUFFER_LENGTH] = { 0x00 };
    const uint8_t cbuffer[LR11XX_GNSS_READ_DEMOD_STATUS_CMD_LENGTH]     = {
        ( uint8_t )( LR11XX_GNSS_READ_DEMOD_STATUS_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_DEMOD_STATUS_OC >> 0 ),
    };

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_READ_DEMOD_STATUS_CMD_LENGTH, rbuffer, sizeof( rbuffer ) );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *demod_status               = ( lr11xx_gnss_demod_status_t ) rbuffer[0];
        demod_info->word_sync_found = rbuffer[1] & 0x01;
        demod_info->first_tow_found = ( rbuffer[1] >> 1 ) & 0x01;
        demod_info->wn_demodulated  = ( rbuffer[1] >> 2 ) & 0x01;
        demod_info->wn_found        = ( rbuffer[1] >> 3 ) & 0x01;
        demod_info->sub1_found      = ( rbuffer[1] >> 4 ) & 0x01;
        demod_info->sub4_found      = ( rbuffer[1] >> 5 ) & 0x01;
        demod_info->sub5_found      = ( rbuffer[1] >> 6 ) & 0x01;
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_gnss_read_cumulative_timing( const void*                      context,
                                                    lr11xx_gnss_cumulative_timing_t* cumulative_timing )
{
    uint8_t       rbuffer[LR11XX_GNSS_READ_CUMULATIVE_TIMING_RBUFFER_LENGTH] = { 0x00 };
    const uint8_t cbuffer[LR11XX_GNSS_READ_CUMULATIVE_TIMING_CMD_LENGTH]     = {
        ( uint8_t )( LR11XX_GNSS_READ_CUMULATIVE_TIMING_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_CUMULATIVE_TIMING_OC >> 0 ),
    };

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_READ_CUMULATIVE_TIMING_CMD_LENGTH, rbuffer, sizeof( rbuffer ) );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        cumulative_timing->init                       = lr11xx_gnss_uint8_to_uint32( &rbuffer[0] );
        cumulative_timing->phase1_gps_capture         = lr11xx_gnss_uint8_to_uint32( &rbuffer[4] );
        cumulative_timing->phase1_gps_process         = lr11xx_gnss_uint8_to_uint32( &rbuffer[8] );
        cumulative_timing->multiscan_gps_capture      = lr11xx_gnss_uint8_to_uint32( &rbuffer[12] );
        cumulative_timing->multiscan_gps_process      = lr11xx_gnss_uint8_to_uint32( &rbuffer[16] );
        cumulative_timing->multiscan_gps_sleep_32k    = lr11xx_gnss_uint8_to_uint32( &rbuffer[20] );
        cumulative_timing->phase1_beidou_capture      = lr11xx_gnss_uint8_to_uint32( &rbuffer[24] );
        cumulative_timing->phase1_beidou_process      = lr11xx_gnss_uint8_to_uint32( &rbuffer[28] );
        cumulative_timing->multiscan_beidou_capture   = lr11xx_gnss_uint8_to_uint32( &rbuffer[32] );
        cumulative_timing->multiscan_beidou_process   = lr11xx_gnss_uint8_to_uint32( &rbuffer[36] );
        cumulative_timing->multiscan_beidou_sleep_32k = lr11xx_gnss_uint8_to_uint32( &rbuffer[40] );
        cumulative_timing->demod_capture              = lr11xx_gnss_uint8_to_uint32( &rbuffer[44] );
        cumulative_timing->demod_process              = lr11xx_gnss_uint8_to_uint32( &rbuffer[48] );
        cumulative_timing->demod_sleep_32k            = lr11xx_gnss_uint8_to_uint32( &rbuffer[52] );
        cumulative_timing->demod_sleep_32m            = lr11xx_gnss_uint8_to_uint32( &rbuffer[56] );
        cumulative_timing->total_gps_capture          = lr11xx_gnss_uint8_to_uint32( &rbuffer[60] );
        cumulative_timing->total_gps_process          = lr11xx_gnss_uint8_to_uint32( &rbuffer[64] );
        cumulative_timing->total_gps_sleep_32k        = lr11xx_gnss_uint8_to_uint32( &rbuffer[68] );
        cumulative_timing->total_gps_sleep_32m        = lr11xx_gnss_uint8_to_uint32( &rbuffer[72] );
        cumulative_timing->total_gps                  = lr11xx_gnss_uint8_to_uint32( &rbuffer[76] );
        cumulative_timing->total_beidou_capture       = lr11xx_gnss_uint8_to_uint32( &rbuffer[80] );
        cumulative_timing->total_beidou_process       = lr11xx_gnss_uint8_to_uint32( &rbuffer[84] );
        cumulative_timing->total_beidou_sleep_32k     = lr11xx_gnss_uint8_to_uint32( &rbuffer[88] );
        cumulative_timing->total_beidou_sleep_32m     = lr11xx_gnss_uint8_to_uint32( &rbuffer[92] );
        cumulative_timing->total_beidou               = lr11xx_gnss_uint8_to_uint32( &rbuffer[96] );
        cumulative_timing->total_capture              = lr11xx_gnss_uint8_to_uint32( &rbuffer[100] );
        cumulative_timing->total_process              = lr11xx_gnss_uint8_to_uint32( &rbuffer[104] );
        cumulative_timing->total_sleep_32k            = lr11xx_gnss_uint8_to_uint32( &rbuffer[108] );
        cumulative_timing->total_sleep_32m            = lr11xx_gnss_uint8_to_uint32( &rbuffer[112] );
        cumulative_timing->total                      = lr11xx_gnss_uint8_to_uint32( &rbuffer[116] );
        cumulative_timing->last_capture_size_32k_cnt  = lr11xx_gnss_uint8_to_uint32( &rbuffer[120] );
        cumulative_timing->constellation_demod        = rbuffer[124];
    }

    return ( lr11xx_status_t ) hal_status;
}

void lr11xx_gnss_compute_power_consumption(
    const lr11xx_gnss_cumulative_timing_t*                  cumulative_timing,
    const lr11xx_gnss_instantaneous_power_consumption_ua_t* instantaneous_power_consumption_ua,
    uint32_t* power_consumption_nah, uint32_t* power_consumption_nwh )
{
    float e_init =
        ( ( float ) instantaneous_power_consumption_ua->init_ua / 1000.0 * ( float ) cumulative_timing->init ) /
        32768.0;

    float e_gps_capture = ( ( ( float ) instantaneous_power_consumption_ua->phase1_gps_capture_ua / 1000.0 *
                              ( float ) cumulative_timing->phase1_gps_capture ) +
                            ( ( float ) instantaneous_power_consumption_ua->multiscan_gps_capture_ua / 1000.0 *
                              ( float ) cumulative_timing->multiscan_gps_capture ) ) /
                          32768.0;

    float e_gps_process = ( ( ( float ) instantaneous_power_consumption_ua->phase1_gps_process_ua / 1000.0 *
                              ( float ) cumulative_timing->phase1_gps_process ) +
                            ( ( float ) instantaneous_power_consumption_ua->multiscan_gps_process_ua / 1000.0 *
                              ( float ) cumulative_timing->multiscan_gps_process ) ) /
                          32768.0;

    float e_gps_sleep_32k = ( ( float ) instantaneous_power_consumption_ua->sleep_32k_ua / 1000.0 *
                              ( float ) cumulative_timing->multiscan_gps_sleep_32k ) /
                            32768.0;

    float e_tot_gps = e_gps_capture + e_gps_process + e_gps_sleep_32k;

    float e_beidou_capture = ( ( ( float ) instantaneous_power_consumption_ua->phase1_beidou_capture_ua / 1000.0 *
                                 ( float ) cumulative_timing->phase1_beidou_capture ) +
                               ( ( float ) instantaneous_power_consumption_ua->multiscan_beidou_capture_ua / 1000.0 *
                                 ( float ) cumulative_timing->multiscan_beidou_capture ) ) /
                             32768.0;
    float e_beidou_process = ( ( ( float ) instantaneous_power_consumption_ua->phase1_beidou_process_ua / 1000.0 *
                                 ( float ) cumulative_timing->phase1_beidou_process ) +
                               ( ( float ) instantaneous_power_consumption_ua->multiscan_beidou_process_ua / 1000.0 *
                                 ( float ) cumulative_timing->multiscan_beidou_process ) ) /
                             32768.0;
    float e_beidou_sleep_32k =
        ( instantaneous_power_consumption_ua->sleep_32k_ua / 1000.0 * cumulative_timing->multiscan_beidou_sleep_32k ) /
        32768.0;

    float e_tot_beidou = e_beidou_capture + e_beidou_process + e_beidou_sleep_32k;

    float e_demod = 0;
    if( cumulative_timing->constellation_demod == 0 )  // Demodulation has been done on GPS constellation
    {
        e_demod = ( ( ( float ) instantaneous_power_consumption_ua->multiscan_gps_capture_ua / 1000.0 *
                      ( float ) cumulative_timing->demod_capture ) +
                    ( ( float ) instantaneous_power_consumption_ua->multiscan_gps_process_ua / 1000.0 *
                      ( float ) cumulative_timing->demod_process ) +
                    ( ( float ) instantaneous_power_consumption_ua->sleep_32k_ua / 1000.0 *
                      ( float ) cumulative_timing->demod_sleep_32k ) +
                    ( ( float ) instantaneous_power_consumption_ua->demod_sleep_32m_ua / 1000.0 *
                      ( float ) cumulative_timing->demod_sleep_32m ) ) /
                  32768.0;
    }
    else  // Domulation has been done on Beidou constellation
    {
        e_demod = ( ( ( float ) instantaneous_power_consumption_ua->multiscan_beidou_capture_ua / 1000.0 *
                      ( float ) cumulative_timing->demod_capture ) +
                    ( ( float ) instantaneous_power_consumption_ua->multiscan_beidou_process_ua / 1000.0 *
                      ( float ) cumulative_timing->demod_process ) +
                    ( ( float ) instantaneous_power_consumption_ua->sleep_32k_ua / 1000.0 *
                      ( float ) cumulative_timing->demod_sleep_32k ) +
                    ( ( float ) instantaneous_power_consumption_ua->demod_sleep_32m_ua / 1000.0 *
                      ( float ) cumulative_timing->demod_sleep_32m ) ) /
                  32768.0;
    }

    float power_consumption_uah_tmp = ( ( e_init + e_tot_gps + e_tot_beidou + e_demod ) * 1000 ) /
                                      ( 3600.0 - ( ( float ) cumulative_timing->total / 32768.0 ) );

    *power_consumption_nah = ( uint32_t )( power_consumption_uah_tmp * 1000 );
    *power_consumption_nwh =
        ( uint32_t )( power_consumption_uah_tmp * instantaneous_power_consumption_ua->board_voltage_mv );
}

lr11xx_status_t lr11xx_gnss_set_time( const void* context, const uint32_t time, const uint16_t time_accuracy )
{
    const uint8_t cbuffer[LR11XX_GNSS_SET_TIME_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_SET_TIME_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_SET_TIME_OC >> 0 ),
        ( uint8_t )( time >> 24 ),
        ( uint8_t )( time >> 16 ),
        ( uint8_t )( time >> 8 ),
        ( uint8_t ) time,
        ( uint8_t )( time_accuracy >> 8 ),
        ( uint8_t ) time_accuracy,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_GNSS_SET_TIME_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_gnss_config_delay_reset_assistance_position( const void* context, const uint32_t delay )
{
    const uint8_t cbuffer[LR11XX_GNSS_CONFIG_DELAY_RESET_AP_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_CONFIG_DELAY_RESET_AP_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_CONFIG_DELAY_RESET_AP_OC >> 0 ),
        ( uint8_t )( delay >> 16 ),
        ( uint8_t )( delay >> 8 ),
        ( uint8_t ) delay,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_GNSS_CONFIG_DELAY_RESET_AP_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_gnss_read_doppler_solver_result( const void*                          context,
                                                        lr11xx_gnss_doppler_solver_result_t* doppler_solver_result )
{
    uint8_t       rbuffer[LR11XX_GNSS_DOPPLER_SOLVER_RES_RBUFFER_LENGTH]     = { 0x00 };
    const uint8_t cbuffer[LR11XX_GNSS_READ_DOPPLER_SOLVER_RESULT_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_READ_DOPPLER_SOLVER_RESULT_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_DOPPLER_SOLVER_RESULT_OC >> 0 ),
    };

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_READ_DOPPLER_SOLVER_RESULT_CMD_LENGTH, rbuffer,
                         LR11XX_GNSS_DOPPLER_SOLVER_RES_RBUFFER_LENGTH );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        doppler_solver_result->error_code         = ( lr11xx_gnss_doppler_solver_error_code_t ) rbuffer[0];
        doppler_solver_result->nb_sv_used         = rbuffer[1];
        doppler_solver_result->one_shot_latitude  = ( uint16_t )( rbuffer[2] << 8 ) | ( uint16_t )( rbuffer[3] );
        doppler_solver_result->one_shot_longitude = ( uint16_t )( rbuffer[4] << 8 ) | ( uint16_t )( rbuffer[5] );
        doppler_solver_result->one_shot_accuracy  = ( uint16_t )( rbuffer[6] << 8 ) | ( uint16_t )( rbuffer[7] );
        doppler_solver_result->one_shot_xtal_ppb  = ( uint16_t )( rbuffer[8] << 8 ) | ( uint16_t )( rbuffer[9] );
        doppler_solver_result->filtered_latitude  = ( uint16_t )( rbuffer[10] << 8 ) | ( uint16_t )( rbuffer[11] );
        doppler_solver_result->filtered_longitude = ( uint16_t )( rbuffer[12] << 8 ) | ( uint16_t )( rbuffer[13] );
        doppler_solver_result->one_shot_accuracy  = ( uint16_t )( rbuffer[14] << 8 ) | ( uint16_t )( rbuffer[15] );
        doppler_solver_result->one_shot_xtal_ppb  = ( uint16_t )( rbuffer[16] << 8 ) | ( uint16_t )( rbuffer[17] );
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_gnss_read_delay_reset_assistance_position( const void* context, uint32_t* delay )
{
    uint8_t       rbuffer[LR11XX_GNSS_READ_DELAY_RESET_AP_RBUFFER_LENGTH] = { 0x00 };
    const uint8_t cbuffer[LR11XX_GNSS_CONFIG_READ_RESET_AP_CMD_LENGTH]    = {
        ( uint8_t )( LR11XX_GNSS_READ_DELAY_RESET_AP_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_DELAY_RESET_AP_OC >> 0 ),
    };

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_CONFIG_READ_RESET_AP_CMD_LENGTH, rbuffer,
                         LR11XX_GNSS_READ_DELAY_RESET_AP_RBUFFER_LENGTH );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *delay = ( uint32_t )( rbuffer[0] << 16 ) | ( uint32_t )( rbuffer[1] << 8 ) | ( uint32_t )( rbuffer[2] );
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_gnss_almanac_update_from_sat( const void*                            context,
                                                     const lr11xx_gnss_constellation_mask_t constellation_mask,
                                                     const lr11xx_gnss_search_mode_t        effort_mode )
{
    const uint8_t cbuffer[LR11XX_GNSS_ALMANAC_UPDATE_FROM_SAT_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_ALMANAC_UPDATE_FROM_SAT_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_ALMANAC_UPDATE_FROM_SAT_OC >> 0 ), ( uint8_t ) effort_mode,
        ( uint8_t ) constellation_mask
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_GNSS_ALMANAC_UPDATE_FROM_SAT_CMD_LENGTH, 0,
                                                 0 );
}

lr11xx_status_t lr11xx_gnss_read_keep_sync_status( const void*                            context,
                                                   const lr11xx_gnss_constellation_mask_t constellation_mask,
                                                   uint8_t* nb_visible_sat, uint32_t* time_elapsed )
{
    uint8_t       rbuffer[LR11XX_GNSS_READ_KEEP_SYNC_STATUS_RBUFFER_LENGTH]    = { 0x00 };
    const uint8_t cbuffer[LR11XX_GNSS_CONFIG_READ_KEEP_SYNC_STATUS_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_READ_KEEP_SYNC_STATUS_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_KEEP_SYNC_STATUS_OC >> 0 ),
        ( uint8_t ) constellation_mask,
    };

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_CONFIG_READ_KEEP_SYNC_STATUS_CMD_LENGTH, rbuffer,
                         LR11XX_GNSS_READ_KEEP_SYNC_STATUS_RBUFFER_LENGTH );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *nb_visible_sat = rbuffer[0];
        *time_elapsed   = lr11xx_gnss_uint8_to_uint32( &rbuffer[1] );
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_gnss_read_almanac_status( const void*                        context,
                                                 lr11xx_gnss_read_almanac_status_t* almanac_status )
{
    uint8_t rbuffer[LR11XX_GNSS_READ_ALMANAC_STATUS_RBUFFER_LENGTH] = { 0x00 };

    const uint8_t cbuffer[LR11XX_GNSS_READ_ALMANAC_STATUS_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_READ_ALMANAC_STATUS_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_ALMANAC_STATUS_OC >> 0 ),
    };

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_READ_ALMANAC_STATUS_CMD_LENGTH, rbuffer,
                         LR11XX_GNSS_READ_ALMANAC_STATUS_RBUFFER_LENGTH );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        almanac_status->status_gps                            = ( lr11xx_gnss_almanac_status_t ) rbuffer[0];
        almanac_status->next_gps_time_sat_to_update           = lr11xx_gnss_uint8_to_uint32( &rbuffer[1] );
        almanac_status->next_gps_nb_subframe_to_demodulate    = rbuffer[5];
        almanac_status->next_gps_sat_id_to_update_in_sub_4    = rbuffer[6];
        almanac_status->next_gps_sat_id_to_update_in_sub_5    = rbuffer[7];
        almanac_status->next_gps_subframe_id_start            = rbuffer[8];
        almanac_status->nb_sat_gps_to_update                  = rbuffer[9];
        almanac_status->sat_id_gps_to_update                  = lr11xx_gnss_uint8_to_uint32( &rbuffer[10] );
        almanac_status->sat_id_gps_activated                  = lr11xx_gnss_uint8_to_uint32( &rbuffer[14] );
        almanac_status->status_beidou                         = ( lr11xx_gnss_almanac_status_t ) rbuffer[18];
        almanac_status->next_beidou_time_sat_to_update        = lr11xx_gnss_uint8_to_uint32( &rbuffer[19] );
        almanac_status->next_beidou_nb_subframe_to_demodulate = rbuffer[23];
        almanac_status->next_beidou_sat_id_to_update_in_sub_4 = rbuffer[24];
        almanac_status->next_beidou_sat_id_to_update_in_sub_5 = rbuffer[25];
        almanac_status->next_beidou_subframe_id_start         = rbuffer[26];
        almanac_status->nb_sat_beidou_to_update               = rbuffer[27];
        almanac_status->sat_id_beidou_to_update[0]            = lr11xx_gnss_uint8_to_uint32( &rbuffer[28] );
        almanac_status->sat_id_beidou_to_update[1]            = lr11xx_gnss_uint8_to_uint32( &rbuffer[32] );
        almanac_status->sat_id_beidou_activated[0]            = lr11xx_gnss_uint8_to_uint32( &rbuffer[36] );
        almanac_status->sat_id_beidou_activated[1]            = lr11xx_gnss_uint8_to_uint32( &rbuffer[40] );
        almanac_status->sat_id_beidou_black_list[0]           = lr11xx_gnss_uint8_to_uint32( &rbuffer[44] );
        almanac_status->sat_id_beidou_black_list[1]           = lr11xx_gnss_uint8_to_uint32( &rbuffer[48] );
        almanac_status->next_am_id                            = rbuffer[52];
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_gnss_config_almanac_update_period( const void*                            context,
                                                          const lr11xx_gnss_constellation_mask_t constellation_mask,
                                                          const lr11xx_gnss_sv_type_t sv_type, const uint16_t period )
{
    const uint8_t cbuffer[LR11XX_GNSS_CONFIG_ALMANAC_UPDATE_PERIOD_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_CONFIG_ALMANAC_UPDATE_PERIOD_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_CONFIG_ALMANAC_UPDATE_PERIOD_OC >> 0 ),
        ( uint8_t ) constellation_mask,
        ( uint8_t ) sv_type,
        ( uint8_t )( period >> 8 ),
        ( uint8_t )( period >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_GNSS_CONFIG_ALMANAC_UPDATE_PERIOD_CMD_LENGTH,
                                                 0, 0 );
}

lr11xx_status_t lr11xx_gnss_read_almanac_update_period( const void*                            context,
                                                        const lr11xx_gnss_constellation_mask_t constellation_mask,
                                                        const lr11xx_gnss_sv_type_t sv_type, uint16_t* period )
{
    uint8_t rbuffer[LR11XX_GNSS_READ_ALMANAC_UPDATE_PERIOD_RBUFFER_LENGTH] = { 0x00 };

    const uint8_t cbuffer[LR11XX_GNSS_READ_ALMANAC_UPDATE_PERIOD_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_READ_ALMANAC_UPDATE_PERIOD_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_ALMANAC_UPDATE_PERIOD_OC >> 0 ),
        ( uint8_t ) constellation_mask,
        ( uint8_t ) sv_type,
    };

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_READ_ALMANAC_UPDATE_PERIOD_CMD_LENGTH, rbuffer,
                         LR11XX_GNSS_READ_ALMANAC_UPDATE_PERIOD_RBUFFER_LENGTH );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *period = ( rbuffer[0] << 8 ) | rbuffer[1];
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_gnss_get_sv_sync( const void* context, const lr11xx_gnss_constellation_mask_t constellation_mask,
                                         const uint8_t nb_sv_to_get, uint8_t* sv_sync_list )
{
    const uint8_t cbuffer[LR11XX_GNSS_GET_SV_SYNC_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_GET_SV_SYNC_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_GET_SV_SYNC_OC >> 0 ),
        ( uint8_t ) constellation_mask,
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_GET_SV_SYNC_CMD_LENGTH, sv_sync_list,
                                                nb_sv_to_get );
}

lr11xx_status_t lr11xx_gnss_set_gps_bit_mask_sat_activated( const void* context, const uint32_t gps_sat_activated_1_32 )
{
    const uint8_t cbuffer[LR11XX_GNSS_WRITE_GPS_BIT_MASK_SAT_ACTIVATED_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_WRITE_BIT_MASK_SAT_ACTIVATED_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_WRITE_BIT_MASK_SAT_ACTIVATED_OC >> 0 ),
        ( uint8_t ) LR11XX_GNSS_GPS_MASK,
        ( uint8_t )( gps_sat_activated_1_32 >> 24 ),
        ( uint8_t )( gps_sat_activated_1_32 >> 16 ),
        ( uint8_t )( gps_sat_activated_1_32 >> 8 ),
        ( uint8_t )( gps_sat_activated_1_32 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer,
                                                 LR11XX_GNSS_WRITE_GPS_BIT_MASK_SAT_ACTIVATED_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_gnss_set_beidou_bit_mask_sat_activated( const void*    context,
                                                               const uint32_t beidou_sat_activated_1_32,
                                                               const uint32_t beidou_sat_activated_33_63 )
{
    const uint8_t cbuffer[LR11XX_GNSS_WRITE_BEIDOU_BIT_MASK_SAT_ACTIVATED_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_WRITE_BIT_MASK_SAT_ACTIVATED_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_WRITE_BIT_MASK_SAT_ACTIVATED_OC >> 0 ),
        ( uint8_t ) LR11XX_GNSS_BEIDOU_MASK,
        ( uint8_t )( beidou_sat_activated_1_32 >> 24 ),
        ( uint8_t )( beidou_sat_activated_1_32 >> 16 ),
        ( uint8_t )( beidou_sat_activated_1_32 >> 8 ),
        ( uint8_t )( beidou_sat_activated_1_32 ),
        ( uint8_t )( beidou_sat_activated_33_63 >> 24 ),
        ( uint8_t )( beidou_sat_activated_33_63 >> 16 ),
        ( uint8_t )( beidou_sat_activated_33_63 >> 8 ),
        ( uint8_t )( beidou_sat_activated_33_63 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer,
                                                 LR11XX_GNSS_WRITE_BEIDOU_BIT_MASK_SAT_ACTIVATED_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_gnss_read_last_scan_mode_launched( const void*                       context,
                                                          lr11xx_gnss_scan_mode_launched_t* last_scan_mode )
{
    uint8_t rbuffer[1] = { 0x00 };

    const uint8_t cbuffer[LR11XX_GNSS_READ_LAST_SCAN_MODE_LAUNCHED_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_GNSS_READ_LAST_SCAN_MODE_LAUNCHED_OC >> 8 ),
        ( uint8_t )( LR11XX_GNSS_READ_LAST_SCAN_MODE_LAUNCHED_OC >> 0 ),
    };

    const lr11xx_hal_status_t hal_status = lr11xx_hal_read(
        context, cbuffer, LR11XX_GNSS_READ_LAST_SCAN_MODE_LAUNCHED_CMD_LENGTH, rbuffer, sizeof( rbuffer ) );

    *last_scan_mode = ( lr11xx_gnss_scan_mode_launched_t ) rbuffer[0];

    return ( lr11xx_status_t ) hal_status;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

uint32_t lr11xx_gnss_uint8_to_uint32( uint8_t value[4] )
{
    return ( ( ( uint32_t ) value[0] ) << 24 ) + ( ( ( uint32_t ) value[1] ) << 16 ) +
           ( ( ( uint32_t ) value[2] ) << 8 ) + ( ( ( uint32_t ) value[3] ) );
}

uint16_t lr11xx_gnss_get_min_from_operand_and_max_nb_of_blocks( uint16_t operand )
{
    if( operand > LR11XX_GNSS_ALMANAC_UPDATE_MAX_NB_OF_BLOCKS )
    {
        return LR11XX_GNSS_ALMANAC_UPDATE_MAX_NB_OF_BLOCKS;
    }
    else
    {
        return operand;
    }
}

/* --- EOF ------------------------------------------------------------------ */
