/*!
 * @file      lr1110_gnss.c
 *
 * @brief     GNSS scan driver implementation for LR1110
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

#include "lr1110_gnss.h"
#include "lr1110_regmem.h"
#include "lr1110_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR1110_GNSS_SET_CONSTELLATION_CMD_LENGTH ( 2 + 1 )
#define LR1110_GNSS_READ_CONSTELLATION_CMD_LENGTH ( 2 )
#define LR1110_GNSS_SET_ALMANAC_UPDATE_CMD_LENGTH ( 2 + 1 )
#define LR1110_GNSS_READ_ALMANAC_UPDATE_CMD_LENGTH ( 2 )
#define LR1110_GNSS_READ_FW_VERSION_CMD_LENGTH ( 2 )
#define LR1110_GNSS_READ_SUPPORTED_CONSTELLATION_CMD_LENGTH ( 2 )
#define LR1110_GNSS_SET_SCAN_MODE_CMD_LENGTH ( 2 + 1 )
#define LR1110_GNSS_SCAN_AUTONOMOUS_CMD_LENGTH ( 2 + 7 )
#define LR1110_GNSS_SCAN_ASSISTED_CMD_LENGTH ( 2 + 7 )
#define LR1110_GNSS_SCAN_GET_RES_SIZE_CMD_LENGTH ( 2 )
#define LR1110_GNSS_SCAN_READ_RES_CMD_LENGTH ( 2 )
#define LR1110_GNSS_ALMANAC_UPDATE_CMD_LENGTH ( 2 )
#define LR1110_GNSS_ALMANAC_READ_CMD_LENGTH ( 2 )
#define LR1110_GNSS_SET_ASSISTANCE_POSITION_CMD_LENGTH ( 2 + 4 )
#define LR1110_GNSS_READ_ASSISTANCE_POSITION_CMD_LENGTH ( 2 )
#define LR1110_GNSS_SET_XTAL_ERROR_CMD_LENGTH ( 2 + 2 )
#define LR1110_GNSS_READ_XTAL_ERROR_CMD_LENGTH ( 2 )
#define LR1110_GNSS_PUSH_SOLVER_MSG_CMD_LENGTH ( 2 )
#define LR1110_GNSS_PUSH_DM_MSG_CMD_LENGTH ( 2 )
#define LR1110_GNSS_GET_CONTEXT_STATUS_CMD_LENGTH ( 2 )
#define LR1110_GNSS_SCAN_GET_TIMINGS_CMD_LENGTH ( 2 )
#define LR1110_GNSS_GET_NB_SV_SATELLITES_CMD_LENGTH ( 2 )
#define LR1110_GNSS_GET_SV_SATELLITES_CMD_LENGTH ( 2 )

#define LR1110_GNSS_ALMANAC_READ_RBUFFER_LENGTH ( 6 )
#define LR1110_GNSS_ALMANAC_DATE_LENGTH ( 2 )
#define LR1110_GNSS_ALMANAC_UPDATE_MAX_NB_OF_BLOCKS \
    ( ( LR1110_CMD_LENGTH_MAX - LR1110_GNSS_ALMANAC_UPDATE_CMD_LENGTH ) / LR1110_GNSS_SINGLE_ALMANAC_WRITE_SIZE )
#define LR1110_GNSS_READ_ALMANAC_TEMPBUFFER_SIZE_BYTE ( 47 )
#define LR1110_GNSS_SCAN_GET_TIMINGS_RBUFFER_LENGTH ( 8 )
#define LR1110_GNSS_MAX_DETECTED_SV ( 32 )
#define LR1110_GNSS_DETECTED_SV_SINGLE_LENGTH ( 4 )
#define LR1110_GNSS_MAX_DETECTED_SV_BUFFER_LENGTH \
    ( LR1110_GNSS_MAX_DETECTED_SV * LR1110_GNSS_DETECTED_SV_SINGLE_LENGTH )
#define LR1110_GNSS_READ_FIRMWARE_VERSION_RBUFFER_LENGTH ( 2 )

#define LR1110_GNSS_SCALING_LATITUDE 90
#define LR1110_GNSS_SCALING_LONGITUDE 180
#define LR1110_GNSS_SNR_TO_CNR_OFFSET ( 31 )

#define LR1110_GNSS_SCAN_RESULT_DESTINATION_INDEX ( 0 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for GNSS-related operations
 */
enum
{
    LR1110_GNSS_SET_CONSTELLATION_OC            = 0x0400,  //!< Set the constellation to use
    LR1110_GNSS_READ_CONSTELLATION_OC           = 0x0401,  //!< Read the used constellations
    LR1110_GNSS_SET_ALMANAC_UPDATE_OC           = 0x0402,  //!< Set almanac update configuration
    LR1110_GNSS_READ_ALMANAC_UPDATE_OC          = 0x0403,  //!< Read the almanac update configuration
    LR1110_GNSS_READ_FW_VERSION_OC              = 0x0406,  //!< Read the firmware version
    LR1110_GNSS_READ_SUPPORTED_CONSTELLATION_OC = 0x0407,  //!< Read the supported constellations
    LR1110_GNSS_SET_SCAN_MODE_OC                = 0x0408,  //!< Define single or double capture
    LR1110_GNSS_SCAN_AUTONOMOUS_OC              = 0x0409,  //!< Launch an autonomous scan
    LR1110_GNSS_SCAN_ASSISTED_OC                = 0x040A,  //!< Launch an assisted scan
    LR1110_GNSS_SCAN_GET_RES_SIZE_OC            = 0x040C,  //!< Get the size of the output payload
    LR1110_GNSS_SCAN_READ_RES_OC                = 0x040D,  //!< Read the byte stream
    LR1110_GNSS_ALMANAC_UPDATE_OC               = 0x040E,  //!< Update the almanac
    LR1110_GNSS_ALMANAC_READ_OC                 = 0x040F,  //!< Read all almanacs
    LR1110_GNSS_SET_ASSISTANCE_POSITION_OC      = 0x0410,  //!< Set the assistance position
    LR1110_GNSS_READ_ASSISTANCE_POSITION_OC     = 0x0411,  //!< Read the assistance position
    LR1110_GNSS_SET_XTAL_ERROR_OC               = 0x0412,  //!< Set the xtal accuracy
    LR1110_GNSS_READ_XTAL_ERROR_OC              = 0x0413,  //!< Read the xtal accuracy
    LR1110_GNSS_PUSH_SOLVER_MSG_OC              = 0x0414,  //!< Push messages coming from the solver
    LR1110_GNSS_PUSH_DM_MSG_OC                  = 0x0415,  //!< Push messages coming from the device management
    LR1110_GNSS_GET_CONTEXT_STATUS_OC           = 0x0416,  //!< Read the context
    LR1110_GNSS_GET_NB_SATELLITES_OC            = 0x0417,  //!< Get the number of satellites detected during a scan
    LR1110_GNSS_GET_SATELLITES_OC               = 0x0418,  //!< Get the list of satellites detected during a scan
    LR1110_GNSS_GET_TIMINGS_OC                  = 0x0419,  //!< Get the time spent in signal acquisition and analysis
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
static uint32_t lr1110_gnss_uint8_to_uint32( uint8_t value[4] );

/*!
 * @brief Returns the minimum of the operand given as parameter and the maximum allowed number of blocks
 *
 * @param [in] operand Size to compare
 *
 * @returns Minimum between operand and @ref LR1110_GNSS_ALMANAC_UPDATE_MAX_NB_OF_BLOCKS
 */
static uint16_t lr1110_gnss_get_min_from_operand_and_max_nb_of_blocks( uint16_t operand );

/*!
 * @brief Get the almanac base address and size
 *
 * @param [in]  context Chip implementation context
 * @param [out] address Start address of the almanac in memory
 * @param [out] size    Size of the almanac in byte
 *
 * @returns Operation status
 */
static lr1110_status_t lr1110_gnss_get_almanac_address_size( const void* context, uint32_t* address, uint16_t* size );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr1110_status_t lr1110_gnss_get_result_size( const void* context, uint16_t* result_size )
{
    const uint8_t cbuffer[LR1110_GNSS_SCAN_GET_RES_SIZE_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_SCAN_GET_RES_SIZE_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_SCAN_GET_RES_SIZE_OC >> 0 ),
    };
    uint8_t rbuffer[sizeof( uint16_t )] = { 0 };

    const lr1110_hal_status_t hal_status =
        lr1110_hal_read( context, cbuffer, LR1110_GNSS_SCAN_GET_RES_SIZE_CMD_LENGTH, rbuffer, sizeof( uint16_t ) );

    if( hal_status == LR1110_HAL_STATUS_OK )
    {
        *result_size = ( ( uint16_t ) rbuffer[0] << 8 ) + ( ( uint16_t ) rbuffer[1] );
    }

    return ( lr1110_status_t ) hal_status;
}

lr1110_status_t lr1110_gnss_read_results( const void* context, uint8_t* result_buffer,
                                          const uint16_t result_buffer_size )
{
    const uint8_t cbuffer[LR1110_GNSS_SCAN_READ_RES_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_SCAN_READ_RES_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_SCAN_READ_RES_OC >> 0 ),
    };

    return ( lr1110_status_t ) lr1110_hal_read( context, cbuffer, LR1110_GNSS_SCAN_READ_RES_CMD_LENGTH, result_buffer,
                                                result_buffer_size );
}

lr1110_status_t lr1110_gnss_get_timings( const void* context, lr1110_gnss_timings_t* timings )
{
    const uint8_t cbuffer[LR1110_GNSS_SCAN_GET_TIMINGS_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_GET_TIMINGS_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_GET_TIMINGS_OC >> 0 ),
    };
    uint8_t                   rbuffer[LR1110_GNSS_SCAN_GET_TIMINGS_RBUFFER_LENGTH] = { 0 };
    const lr1110_hal_status_t hal_status = lr1110_hal_read( context, cbuffer, LR1110_GNSS_SCAN_GET_TIMINGS_CMD_LENGTH,
                                                            rbuffer, LR1110_GNSS_SCAN_GET_TIMINGS_RBUFFER_LENGTH );

    timings->computation_ms = lr1110_gnss_uint8_to_uint32( &rbuffer[0] ) / 1000;
    timings->radio_ms       = lr1110_gnss_uint8_to_uint32( &rbuffer[4] ) / 1000;
    return ( lr1110_status_t ) hal_status;
}

lr1110_status_t lr1110_gnss_almanac_update( const void* context, const uint8_t* blocks, const uint8_t nb_of_blocks )
{
    const uint8_t cbuffer[LR1110_GNSS_ALMANAC_UPDATE_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_ALMANAC_UPDATE_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_ALMANAC_UPDATE_OC >> 0 ),
    };

    uint16_t remaining_nb_of_blocks = nb_of_blocks;

    while( remaining_nb_of_blocks > 0 )
    {
        const uint16_t nb_of_blocks_to_write =
            lr1110_gnss_get_min_from_operand_and_max_nb_of_blocks( remaining_nb_of_blocks );

        const uint8_t* blocks_to_write =
            blocks + ( nb_of_blocks - remaining_nb_of_blocks ) * LR1110_GNSS_SINGLE_ALMANAC_WRITE_SIZE;

        const lr1110_hal_status_t status =
            lr1110_hal_write( context, cbuffer, LR1110_GNSS_ALMANAC_UPDATE_CMD_LENGTH, blocks_to_write,
                              nb_of_blocks_to_write * LR1110_GNSS_SINGLE_ALMANAC_WRITE_SIZE );

        if( status != LR1110_HAL_STATUS_OK )
        {
            return ( lr1110_status_t ) status;
        }

        remaining_nb_of_blocks -= nb_of_blocks_to_write;
    }

    return LR1110_STATUS_OK;
}

lr1110_status_t lr1110_gnss_read_almanac( const void*                                context,
                                          lr1110_gnss_almanac_full_read_bytestream_t almanac_bytestream )
{
    uint32_t        almanac_address = 0;
    uint16_t        almanac_size    = 0;
    lr1110_status_t status          = lr1110_gnss_get_almanac_address_size( context, &almanac_address, &almanac_size );
    if( status != LR1110_STATUS_OK )
    {
        return status;
    }

    const uint8_t N_READ_ALMANAC_REGMEM32 = 15;

    for( uint8_t index_regmem32 = 0; index_regmem32 < N_READ_ALMANAC_REGMEM32; index_regmem32++ )
    {
        const uint16_t local_bytestream_index_burst =
            index_regmem32 * LR1110_GNSS_READ_ALMANAC_TEMPBUFFER_SIZE_BYTE * 4;
        uint32_t temporary_buffer[LR1110_GNSS_READ_ALMANAC_TEMPBUFFER_SIZE_BYTE] = { 0 };

        const lr1110_status_t local_status = lr1110_regmem_read_regmem32(
            context, almanac_address, temporary_buffer, LR1110_GNSS_READ_ALMANAC_TEMPBUFFER_SIZE_BYTE );
        if( local_status != LR1110_STATUS_OK )
        {
            return local_status;
        }

        almanac_address += ( LR1110_GNSS_READ_ALMANAC_TEMPBUFFER_SIZE_BYTE * 4 );

        for( uint8_t index_local_temp = 0; index_local_temp < LR1110_GNSS_READ_ALMANAC_TEMPBUFFER_SIZE_BYTE;
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

lr1110_status_t lr1110_gnss_get_almanac_age_for_satellite( const void* context, const lr1110_gnss_satellite_id_t sv_id,
                                                           uint16_t* almanac_age )
{
    uint32_t              almanac_base_address = 0;
    uint16_t              almanac_size         = 0;
    const lr1110_status_t status_get_almanac_address_size =
        lr1110_gnss_get_almanac_address_size( context, &almanac_base_address, &almanac_size );

    if( status_get_almanac_address_size != LR1110_STATUS_OK )
    {
        return status_get_almanac_address_size;
    }

    const uint16_t offset_almanac_date                               = sv_id * LR1110_GNSS_SINGLE_ALMANAC_READ_SIZE + 1;
    uint8_t        raw_almanac_date[LR1110_GNSS_ALMANAC_DATE_LENGTH] = { 0 };

    const lr1110_status_t status_read_mem = lr1110_regmem_read_mem8(
        context, almanac_base_address + offset_almanac_date, raw_almanac_date, LR1110_GNSS_ALMANAC_DATE_LENGTH );
    if( status_read_mem == LR1110_STATUS_OK )
    {
        // Note: the memory on LR1110 is LSB first. As the 2-byte wide almanac age is obtained by calling the _mem8, the
        // conversion to uint16_t here is done LSB first
        ( *almanac_age ) =
            ( ( ( uint16_t ) raw_almanac_date[1] ) << 8 ) + ( ( ( uint16_t ) raw_almanac_date[0] ) << 0 );
    }
    return status_read_mem;
}

lr1110_status_t lr1110_gnss_get_almanac_address_size( const void* context, uint32_t* address, uint16_t* size )
{
    const uint8_t cbuffer[LR1110_GNSS_ALMANAC_READ_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_ALMANAC_READ_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_ALMANAC_READ_OC >> 0 ),
    };
    uint8_t rbuffer[LR1110_GNSS_ALMANAC_READ_RBUFFER_LENGTH] = { 0 };

    const lr1110_hal_status_t hal_status = lr1110_hal_read( context, cbuffer, LR1110_GNSS_ALMANAC_READ_CMD_LENGTH,
                                                            rbuffer, LR1110_GNSS_ALMANAC_READ_RBUFFER_LENGTH );

    if( hal_status == LR1110_HAL_STATUS_OK )
    {
        *address = lr1110_gnss_uint8_to_uint32( &rbuffer[0] );
        *size    = ( ( ( uint16_t ) rbuffer[4] ) << 8 ) | ( ( uint16_t ) rbuffer[5] );
    }

    return ( lr1110_status_t ) hal_status;
}

lr1110_status_t lr1110_gnss_push_solver_msg( const void* context, const uint8_t* payload, const uint16_t payload_size )
{
    const uint8_t cbuffer[LR1110_GNSS_PUSH_SOLVER_MSG_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_PUSH_SOLVER_MSG_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_PUSH_SOLVER_MSG_OC >> 0 ),
    };

    return ( lr1110_status_t ) lr1110_hal_write( context, cbuffer, LR1110_GNSS_PUSH_SOLVER_MSG_CMD_LENGTH, payload,
                                                 payload_size );
}

lr1110_status_t lr1110_gnss_set_constellations_to_use( const void*                            context,
                                                       const lr1110_gnss_constellation_mask_t constellation_to_use )
{
    const uint8_t cbuffer[LR1110_GNSS_SET_CONSTELLATION_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_SET_CONSTELLATION_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_SET_CONSTELLATION_OC >> 0 ),
        constellation_to_use,
    };

    return ( lr1110_status_t ) lr1110_hal_write( context, cbuffer, LR1110_GNSS_SET_CONSTELLATION_CMD_LENGTH, 0, 0 );
}

lr1110_status_t lr1110_gnss_read_used_constellations( const void*                       context,
                                                      lr1110_gnss_constellation_mask_t* constellations_used )
{
    const uint8_t cbuffer[LR1110_GNSS_READ_CONSTELLATION_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_READ_CONSTELLATION_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_READ_CONSTELLATION_OC >> 0 ),
    };

    return ( lr1110_status_t ) lr1110_hal_read( context, cbuffer, LR1110_GNSS_READ_CONSTELLATION_CMD_LENGTH,
                                                constellations_used, sizeof( *constellations_used ) );
}

lr1110_status_t lr1110_gnss_set_almanac_update( const void*                            context,
                                                const lr1110_gnss_constellation_mask_t constellations_to_update )
{
    const uint8_t cbuffer[LR1110_GNSS_SET_ALMANAC_UPDATE_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_SET_ALMANAC_UPDATE_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_SET_ALMANAC_UPDATE_OC >> 0 ),
        constellations_to_update,
    };

    return ( lr1110_status_t ) lr1110_hal_write( context, cbuffer, LR1110_GNSS_SET_ALMANAC_UPDATE_CMD_LENGTH, 0, 0 );
}

lr1110_status_t lr1110_gnss_read_almanac_update( const void*                       context,
                                                 lr1110_gnss_constellation_mask_t* constellations_to_update )
{
    const uint8_t cbuffer[LR1110_GNSS_READ_ALMANAC_UPDATE_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_READ_ALMANAC_UPDATE_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_READ_ALMANAC_UPDATE_OC >> 0 ),
    };

    return ( lr1110_status_t ) lr1110_hal_read( context, cbuffer, LR1110_GNSS_READ_ALMANAC_UPDATE_CMD_LENGTH,
                                                constellations_to_update, sizeof( *constellations_to_update ) );
}

lr1110_status_t lr1110_gnss_read_firmware_version( const void* context, lr1110_gnss_version_t* version )
{
    const uint8_t cbuffer[LR1110_GNSS_READ_FW_VERSION_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_READ_FW_VERSION_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_READ_FW_VERSION_OC >> 0 ),
    };
    uint8_t rbuffer[LR1110_GNSS_READ_FIRMWARE_VERSION_RBUFFER_LENGTH] = { 0 };

    const lr1110_hal_status_t hal_status = lr1110_hal_read( context, cbuffer, LR1110_GNSS_READ_FW_VERSION_CMD_LENGTH,
                                                            rbuffer, LR1110_GNSS_READ_FIRMWARE_VERSION_RBUFFER_LENGTH );

    version->gnss_firmware = rbuffer[0];
    version->gnss_almanac  = rbuffer[1];
    return ( lr1110_status_t ) hal_status;
}

lr1110_status_t lr1110_gnss_read_supported_constellations( const void*                       context,
                                                           lr1110_gnss_constellation_mask_t* supported_constellations )
{
    const uint8_t cbuffer[LR1110_GNSS_READ_SUPPORTED_CONSTELLATION_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_READ_SUPPORTED_CONSTELLATION_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_READ_SUPPORTED_CONSTELLATION_OC >> 0 ),
    };

    return ( lr1110_status_t ) lr1110_hal_read( context, cbuffer, LR1110_GNSS_READ_SUPPORTED_CONSTELLATION_CMD_LENGTH,
                                                supported_constellations, sizeof( *supported_constellations ) );
}

lr1110_status_t lr1110_gnss_set_scan_mode( const void* context, const lr1110_gnss_scan_mode_t scan_mode )
{
    const uint8_t cbuffer[LR1110_GNSS_SET_SCAN_MODE_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_SET_SCAN_MODE_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_SET_SCAN_MODE_OC >> 0 ),
        ( uint8_t ) scan_mode,
    };

    return ( lr1110_status_t ) lr1110_hal_write( context, cbuffer, LR1110_GNSS_SET_SCAN_MODE_CMD_LENGTH, 0, 0 );
}

lr1110_status_t lr1110_gnss_scan_autonomous( const void* context, const lr1110_gnss_date_t date,
                                             const lr1110_gnss_search_mode_t effort_mode,
                                             const uint8_t gnss_input_parameters, const uint8_t nb_sat )
{
    const uint8_t cbuffer[LR1110_GNSS_SCAN_AUTONOMOUS_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_SCAN_AUTONOMOUS_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_SCAN_AUTONOMOUS_OC >> 0 ),
        ( uint8_t )( date >> 24 ),
        ( uint8_t )( date >> 16 ),
        ( uint8_t )( date >> 8 ),
        ( uint8_t )( date >> 0 ),
        ( uint8_t ) effort_mode,
        gnss_input_parameters,
        nb_sat,
    };

    return ( lr1110_status_t ) lr1110_hal_write( context, cbuffer, LR1110_GNSS_SCAN_AUTONOMOUS_CMD_LENGTH, 0, 0 );
}

lr1110_status_t lr1110_gnss_scan_assisted( const void* context, const lr1110_gnss_date_t date,
                                           const lr1110_gnss_search_mode_t effort_mode,
                                           const uint8_t gnss_input_parameters, const uint8_t nb_sat )
{
    const uint8_t cbuffer[LR1110_GNSS_SCAN_ASSISTED_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_SCAN_ASSISTED_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_SCAN_ASSISTED_OC >> 0 ),
        ( uint8_t )( date >> 24 ),
        ( uint8_t )( date >> 16 ),
        ( uint8_t )( date >> 8 ),
        ( uint8_t )( date >> 0 ),
        ( uint8_t ) effort_mode,
        gnss_input_parameters,
        nb_sat,
    };

    return ( lr1110_status_t ) lr1110_hal_write( context, cbuffer, LR1110_GNSS_SCAN_ASSISTED_CMD_LENGTH, 0, 0 );
}

lr1110_status_t lr1110_gnss_set_assistance_position(
    const void* context, const lr1110_gnss_solver_assistance_position_t* assistance_position )
{
    const int16_t latitude  = ( ( assistance_position->latitude * 2048 ) / LR1110_GNSS_SCALING_LATITUDE );
    const int16_t longitude = ( ( assistance_position->longitude * 2048 ) / LR1110_GNSS_SCALING_LONGITUDE );
    const uint8_t cbuffer[LR1110_GNSS_SET_ASSISTANCE_POSITION_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_SET_ASSISTANCE_POSITION_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_SET_ASSISTANCE_POSITION_OC >> 0 ),
        ( uint8_t )( latitude >> 8 ),
        ( uint8_t )( latitude >> 0 ),
        ( uint8_t )( longitude >> 8 ),
        ( uint8_t )( longitude >> 0 ),
    };

    return ( lr1110_status_t ) lr1110_hal_write( context, cbuffer, LR1110_GNSS_SET_ASSISTANCE_POSITION_CMD_LENGTH, 0,
                                                 0 );
}

lr1110_status_t lr1110_gnss_read_assistance_position( const void*                               context,
                                                      lr1110_gnss_solver_assistance_position_t* assistance_position )
{
    uint8_t       position_buffer[4] = { 0x00 };
    int16_t       position_tmp;
    const uint8_t cbuffer[LR1110_GNSS_READ_ASSISTANCE_POSITION_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_READ_ASSISTANCE_POSITION_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_READ_ASSISTANCE_POSITION_OC >> 0 ),
    };

    const lr1110_hal_status_t hal_status = lr1110_hal_read(
        context, cbuffer, LR1110_GNSS_READ_ASSISTANCE_POSITION_CMD_LENGTH, position_buffer, sizeof( position_buffer ) );

    position_tmp                  = ( ( ( uint16_t ) position_buffer[0] << 8 ) + position_buffer[1] );
    assistance_position->latitude = ( ( float ) ( position_tmp ) *LR1110_GNSS_SCALING_LATITUDE ) / 2048;

    position_tmp                   = ( ( ( uint16_t ) position_buffer[2] << 8 ) + position_buffer[3] );
    assistance_position->longitude = ( ( float ) ( position_tmp ) *LR1110_GNSS_SCALING_LONGITUDE ) / 2048;

    return ( lr1110_status_t ) hal_status;
}

lr1110_status_t lr1110_gnss_set_xtal_error( const void* context, const float xtal_error_in_ppm )
{
    const uint8_t cbuffer[LR1110_GNSS_SET_XTAL_ERROR_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_SET_XTAL_ERROR_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_SET_XTAL_ERROR_OC >> 0 ),
        ( uint8_t )( ( int16_t )( ( xtal_error_in_ppm * 32768 ) / 40 ) >> 8 ),
        ( uint8_t )( ( int16_t )( ( xtal_error_in_ppm * 32768 ) / 40 ) >> 0 ),
    };

    return ( lr1110_status_t ) lr1110_hal_write( context, cbuffer, LR1110_GNSS_SET_XTAL_ERROR_CMD_LENGTH, 0, 0 );
}

lr1110_status_t lr1110_gnss_read_xtal_error( const void* context, float* xtal_error_in_ppm )
{
    uint8_t       xtal_error_buffer[2] = { 0x00 };
    int16_t       xtal_error_temp;
    const uint8_t cbuffer[LR1110_GNSS_READ_XTAL_ERROR_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_READ_XTAL_ERROR_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_READ_XTAL_ERROR_OC >> 0 ),
    };

    const lr1110_hal_status_t hal_status = lr1110_hal_read( context, cbuffer, LR1110_GNSS_READ_XTAL_ERROR_CMD_LENGTH,
                                                            xtal_error_buffer, sizeof( xtal_error_buffer ) );

    xtal_error_temp    = ( ( ( uint16_t ) xtal_error_buffer[0] << 8 ) + xtal_error_buffer[1] );
    *xtal_error_in_ppm = ( ( float ) ( xtal_error_temp ) *40 ) / 32768;
    return ( lr1110_status_t ) hal_status;
}

lr1110_status_t lr1110_gnss_push_dmc_msg( const void* context, uint8_t* dmc_msg, uint16_t dmc_msg_len )
{
    const uint8_t cbuffer[LR1110_GNSS_PUSH_DM_MSG_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_PUSH_DM_MSG_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_PUSH_DM_MSG_OC >> 0 ),
    };

    return ( lr1110_status_t ) lr1110_hal_write( context, cbuffer, LR1110_GNSS_PUSH_DM_MSG_CMD_LENGTH, dmc_msg,
                                                 dmc_msg_len );
}

lr1110_status_t lr1110_gnss_get_context_status( const void*                             context,
                                                lr1110_gnss_context_status_bytestream_t context_status )
{
    const uint8_t cbuffer[LR1110_GNSS_GET_CONTEXT_STATUS_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_GET_CONTEXT_STATUS_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_GET_CONTEXT_STATUS_OC >> 0 ),
    };

    return ( lr1110_status_t ) lr1110_hal_read( context, cbuffer, LR1110_GNSS_GET_CONTEXT_STATUS_CMD_LENGTH,
                                                context_status, LR1110_GNSS_CONTEXT_STATUS_LENGTH );
}

lr1110_status_t lr1110_gnss_get_nb_detected_satellites( const void* context, uint8_t* nb_detected_satellites )
{
    const uint8_t cbuffer[LR1110_GNSS_GET_NB_SV_SATELLITES_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_GET_NB_SATELLITES_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_GET_NB_SATELLITES_OC >> 0 ),
    };

    return ( lr1110_status_t ) lr1110_hal_read( context, cbuffer, LR1110_GNSS_GET_NB_SV_SATELLITES_CMD_LENGTH,
                                                nb_detected_satellites, 1 );
}

lr1110_status_t lr1110_gnss_get_detected_satellites(
    const void* context, const uint8_t nb_detected_satellites,
    lr1110_gnss_detected_satellite_t* detected_satellite_id_snr_doppler )
{
    const uint8_t max_satellites_to_fetch =
        ( LR1110_GNSS_MAX_DETECTED_SV > nb_detected_satellites ) ? nb_detected_satellites : LR1110_GNSS_MAX_DETECTED_SV;
    const uint16_t read_size = max_satellites_to_fetch * LR1110_GNSS_DETECTED_SV_SINGLE_LENGTH;
    uint8_t        result_buffer[LR1110_GNSS_MAX_DETECTED_SV_BUFFER_LENGTH] = { 0 };

    const uint8_t cbuffer[LR1110_GNSS_GET_SV_SATELLITES_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_GET_SATELLITES_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_GET_SATELLITES_OC >> 0 ),
    };

    const lr1110_hal_status_t hal_status =
        lr1110_hal_read( context, cbuffer, LR1110_GNSS_GET_SV_SATELLITES_CMD_LENGTH, result_buffer, read_size );
    if( hal_status == LR1110_HAL_STATUS_OK )
    {
        for( uint8_t index_satellite = 0; index_satellite < max_satellites_to_fetch; index_satellite++ )
        {
            const uint16_t local_result_buffer_index = index_satellite * LR1110_GNSS_DETECTED_SV_SINGLE_LENGTH;
            lr1110_gnss_detected_satellite_t* local_satellite_result =
                &detected_satellite_id_snr_doppler[index_satellite];

            local_satellite_result->satellite_id = result_buffer[local_result_buffer_index];
            local_satellite_result->cnr = result_buffer[local_result_buffer_index + 1] + LR1110_GNSS_SNR_TO_CNR_OFFSET;
            local_satellite_result->doppler = ( int16_t )( ( result_buffer[local_result_buffer_index + 2] << 8 ) +
                                                           ( result_buffer[local_result_buffer_index + 3] << 0 ) );
        }
    }
    return ( lr1110_status_t ) hal_status;
}

lr1110_status_t lr1110_gnss_parse_context_status_buffer(
    const lr1110_gnss_context_status_bytestream_t context_status_bytestream,
    lr1110_gnss_context_status_t*                 context_status )
{
    lr1110_status_t status = LR1110_STATUS_ERROR;

    if( ( ( lr1110_gnss_destination_t ) context_status_bytestream[0] == LR1110_GNSS_DESTINATION_DMC ) &&
        ( ( lr1110_gnss_message_dmc_opcode_t ) context_status_bytestream[1] == LR1110_GNSS_DMC_STATUS ) )
    {
        context_status->firmware_version = context_status_bytestream[2];

        context_status->global_almanac_crc =
            ( ( uint32_t ) context_status_bytestream[3] << 0 ) + ( ( uint32_t ) context_status_bytestream[4] << 8 ) +
            ( ( uint32_t ) context_status_bytestream[5] << 16 ) + ( ( uint32_t ) context_status_bytestream[6] << 24 );

        context_status->error_code = ( lr1110_gnss_error_code_t )( context_status_bytestream[7] >> 4 );

        context_status->almanac_update_gps =
            ( ( context_status_bytestream[7] & LR1110_GNSS_DMC_ALMANAC_UPDATE_GPS_MASK ) != 0 ) ? true : false;

        context_status->almanac_update_beidou =
            ( ( context_status_bytestream[7] & LR1110_GNSS_DMC_ALMANAC_UPDATE_BEIDOU_MASK ) != 0 ) ? true : false;

        context_status->freq_search_space = ( lr1110_gnss_freq_search_space_t )(
            ( ( ( context_status_bytestream[7] & LR1110_GNSS_DMC_FREQUENCY_SEARCH_SPACE_MSB_MASK ) >>
                LR1110_GNSS_DMC_FREQUENCY_SEARCH_SPACE_MSB_POS )
              << 1 ) +
            ( ( context_status_bytestream[8] & LR1110_GNSS_DMC_FREQUENCY_SEARCH_SPACE_LSB_MASK ) >>
              LR1110_GNSS_DMC_FREQUENCY_SEARCH_SPACE_LSB_POS ) );

        status = LR1110_STATUS_OK;
    }

    return status;
}

lr1110_status_t lr1110_gnss_get_result_destination( const uint8_t* result_buffer, const uint16_t result_buffer_size,
                                                    lr1110_gnss_destination_t* destination )
{
    lr1110_status_t status = LR1110_STATUS_ERROR;

    if( result_buffer_size != 0 )
    {
        switch( result_buffer[LR1110_GNSS_SCAN_RESULT_DESTINATION_INDEX] )
        {
        case LR1110_GNSS_DESTINATION_HOST:
        {
            *destination = LR1110_GNSS_DESTINATION_HOST;
            status       = LR1110_STATUS_OK;
            break;
        }
        case LR1110_GNSS_DESTINATION_SOLVER:
        {
            *destination = LR1110_GNSS_DESTINATION_SOLVER;
            status       = LR1110_STATUS_OK;
            break;
        }
        case LR1110_GNSS_DESTINATION_DMC:
        {
            *destination = LR1110_GNSS_DESTINATION_DMC;
            status       = LR1110_STATUS_OK;
            break;
        }
        }
    }

    return status;
}

uint16_t lr1110_gnss_compute_almanac_age( uint16_t almanac_date,
                                          uint16_t nb_days_between_epoch_and_last_gps_time_rollover,
                                          uint16_t nb_days_since_epoch )
{
    return nb_days_since_epoch - ( almanac_date + nb_days_between_epoch_and_last_gps_time_rollover );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

uint32_t lr1110_gnss_uint8_to_uint32( uint8_t* value )
{
    return ( ( ( uint32_t ) value[0] ) << 24 ) + ( ( ( uint32_t ) value[1] ) << 16 ) +
           ( ( ( uint32_t ) value[2] ) << 8 ) + ( ( ( uint32_t ) value[3] ) << 0 );
}

uint16_t lr1110_gnss_get_min_from_operand_and_max_nb_of_blocks( uint16_t operand )
{
    if( operand > LR1110_GNSS_ALMANAC_UPDATE_MAX_NB_OF_BLOCKS )
    {
        return LR1110_GNSS_ALMANAC_UPDATE_MAX_NB_OF_BLOCKS;
    }
    else
    {
        return operand;
    }
}

/* --- EOF ------------------------------------------------------------------ */
