/*!
 * @file      lr11xx_regmem.c
 *
 * @brief     Register/memory driver implementation for LR11XX
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

#include "lr11xx_regmem.h"
#include "lr11xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR11XX_REGMEM_CLEAR_RXBUFFER_CMD_LENGTH 2
#define LR11XX_REGMEM_WRITE_REGMEM32_CMD_LENGTH ( 2 + 4 )
#define LR11XX_REGMEM_READ_REGMEM32_CMD_LENGTH ( 2 + 4 + 1 )
#define LR11XX_REGMEM_WRITE_MEM8_CMD_LENGTH ( 2 + 4 )
#define LR11XX_REGMEM_READ_MEM8_CMD_LENGTH ( 2 + 4 + 1 )
#define LR11XX_REGMEM_WRITE_BUFFER8_CMD_LENGTH ( 2 )
#define LR11XX_REGMEM_READ_BUFFER8_CMD_LENGTH ( 2 + 2 )
#define LR11XX_REGMEM_WRITE_REGMEM32_MASK_CMD_LENGTH ( 2 + 4 + 4 + 4 )

#define LR11XX_REGMEM_BUFFER_SIZE_MAX ( 256 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for register and memory related operations
 */
enum
{
    LR11XX_REGMEM_WRITE_REGMEM32_OC      = 0x0105,
    LR11XX_REGMEM_READ_REGMEM32_OC       = 0x0106,
    LR11XX_REGMEM_WRITE_MEM8_OC          = 0x0107,
    LR11XX_REGMEM_READ_MEM8_OC           = 0x0108,
    LR11XX_REGMEM_WRITE_BUFFER8_OC       = 0x0109,
    LR11XX_REGMEM_READ_BUFFER8_OC        = 0x010A,
    LR11XX_REGMEM_CLEAR_RXBUFFER_OC      = 0x010B,
    LR11XX_REGMEM_WRITE_REGMEM32_MASK_OC = 0x010C,
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
 * @brief Helper function that fill both cbuffer with opcode and memory address
 *
 * It is typically used in read/write regmem32 functions.
 *
 * @warning It is up to the caller to ensure cbuffer is big enough to contain opcode and address!
 */
static void lr11xx_regmem_fill_cbuffer_opcode_address( uint8_t* cbuffer, uint16_t opcode, uint32_t address );

/*!
 * @brief Helper function that fill both cbuffer with opcode memory address, and data length to read
 *
 * It is typically used in read functions.
 *
 * @warning It is up to the caller to ensure cbuffer is big enough to contain opcode and address!
 */
static void lr11xx_regmem_fill_cbuffer_opcode_address_length( uint8_t* cbuffer, uint16_t opcode, uint32_t address,
                                                              uint8_t length );

/*!
 * @brief Helper function that fill both cbuffer with data
 *
 * It is typically used in write write regmem32 functions.
 *
 * @warning It is up to the caller to ensure cdata is big enough to contain all data!
 */
static void lr11xx_regmem_fill_cdata( uint8_t* cdata, const uint32_t* data, uint8_t data_length );

/*!
 * @brief Helper function that fill both cbuffer and cdata buffers with opcode, memory address and data
 *
 * It is typically used to factorize and write regmem32 operations. Behind the scene it calls the other helpers
 * lr11xx_regmem_fill_cbuffer_opcode_address and lr11xx_regmem_fill_cdata.
 *
 * @warning It is up to the caller to ensure cbuffer and cdata are big enough to contain their respective information!
 */
static void lr11xx_regmem_fill_cbuffer_cdata_opcode_address_data( uint8_t* cbuffer, uint8_t* cdata, uint16_t opcode,
                                                                  uint32_t address, const uint32_t* data,
                                                                  uint8_t data_length );

/*!
 * @brief Helper function that convert an array of uint8_t into an array of uint32_t
 *
 * Typically used in the read function returning uint32_t array.
 *
 * @warning It is up to the caller to ensure the raw_buffer is of length at least "out_buffer_length *
 * sizeof(uint32_t)"!
 */
static void lr11xx_regmem_fill_out_buffer_from_raw_buffer( uint32_t* out_buffer, const uint8_t* raw_buffer,
                                                           uint8_t out_buffer_length );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_status_t lr11xx_regmem_write_regmem32( const void* context, const uint32_t address, const uint32_t* buffer,
                                              const uint8_t length )
{
    uint8_t cbuffer[LR11XX_REGMEM_WRITE_REGMEM32_CMD_LENGTH];
    uint8_t cdata[LR11XX_REGMEM_BUFFER_SIZE_MAX];

    lr11xx_regmem_fill_cbuffer_cdata_opcode_address_data( cbuffer, cdata, LR11XX_REGMEM_WRITE_REGMEM32_OC, address,
                                                          buffer, length );

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_REGMEM_WRITE_REGMEM32_CMD_LENGTH, cdata,
                                                 length * sizeof( uint32_t ) );
}

lr11xx_status_t lr11xx_regmem_read_regmem32( const void* context, const uint32_t address, uint32_t* buffer,
                                             const uint8_t length )
{
    uint8_t         cbuffer[LR11XX_REGMEM_READ_REGMEM32_CMD_LENGTH];
    lr11xx_status_t status = LR11XX_STATUS_ERROR;

    lr11xx_regmem_fill_cbuffer_opcode_address_length( cbuffer, LR11XX_REGMEM_READ_REGMEM32_OC, address, length );

    status = ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_REGMEM_READ_REGMEM32_CMD_LENGTH,
                                                  ( uint8_t* ) buffer, length * sizeof( uint32_t ) );

    if( status == LR11XX_STATUS_OK )
    {
        lr11xx_regmem_fill_out_buffer_from_raw_buffer( buffer, ( const uint8_t* ) buffer, length );
    }

    return status;
}

lr11xx_status_t lr11xx_regmem_write_mem8( const void* context, const uint32_t address, const uint8_t* buffer,
                                          const uint8_t length )
{
    uint8_t cbuffer[LR11XX_REGMEM_WRITE_MEM8_CMD_LENGTH];

    lr11xx_regmem_fill_cbuffer_opcode_address( cbuffer, LR11XX_REGMEM_WRITE_MEM8_OC, address );

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_REGMEM_WRITE_MEM8_CMD_LENGTH, buffer,
                                                 length );
}

lr11xx_status_t lr11xx_regmem_read_mem8( const void* context, const uint32_t address, uint8_t* buffer,
                                         const uint8_t length )
{
    uint8_t cbuffer[LR11XX_REGMEM_READ_MEM8_CMD_LENGTH];

    lr11xx_regmem_fill_cbuffer_opcode_address_length( cbuffer, LR11XX_REGMEM_READ_MEM8_OC, address, length );

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_REGMEM_READ_MEM8_CMD_LENGTH, buffer, length );
}

lr11xx_status_t lr11xx_regmem_write_buffer8( const void* context, const uint8_t* buffer, const uint8_t length )
{
    const uint8_t cbuffer[LR11XX_REGMEM_WRITE_BUFFER8_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_REGMEM_WRITE_BUFFER8_OC >> 8 ),
        ( uint8_t ) ( LR11XX_REGMEM_WRITE_BUFFER8_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_REGMEM_WRITE_BUFFER8_CMD_LENGTH, buffer,
                                                 length );
}

lr11xx_status_t lr11xx_regmem_read_buffer8( const void* context, uint8_t* buffer, const uint8_t offset,
                                            const uint8_t length )
{
    const uint8_t cbuffer[LR11XX_REGMEM_READ_BUFFER8_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_REGMEM_READ_BUFFER8_OC >> 8 ),
        ( uint8_t ) ( LR11XX_REGMEM_READ_BUFFER8_OC >> 0 ),
        offset,
        length,
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_REGMEM_READ_BUFFER8_CMD_LENGTH, buffer,
                                                length );
}

lr11xx_status_t lr11xx_regmem_clear_rxbuffer( const void* context )
{
    const uint8_t cbuffer[LR11XX_REGMEM_CLEAR_RXBUFFER_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_REGMEM_CLEAR_RXBUFFER_OC >> 8 ),
        ( uint8_t ) ( LR11XX_REGMEM_CLEAR_RXBUFFER_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_REGMEM_CLEAR_RXBUFFER_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_regmem_write_regmem32_mask( const void* context, const uint32_t address, const uint32_t mask,
                                                   const uint32_t data )
{
    uint8_t cbuffer[LR11XX_REGMEM_WRITE_REGMEM32_MASK_CMD_LENGTH];

    lr11xx_regmem_fill_cbuffer_opcode_address( cbuffer, LR11XX_REGMEM_WRITE_REGMEM32_MASK_OC, address );

    cbuffer[6] = ( uint8_t ) ( mask >> 24 );
    cbuffer[7] = ( uint8_t ) ( mask >> 16 );
    cbuffer[8] = ( uint8_t ) ( mask >> 8 );
    cbuffer[9] = ( uint8_t ) ( mask >> 0 );

    cbuffer[10] = ( uint8_t ) ( data >> 24 );
    cbuffer[11] = ( uint8_t ) ( data >> 16 );
    cbuffer[12] = ( uint8_t ) ( data >> 8 );
    cbuffer[13] = ( uint8_t ) ( data >> 0 );

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_REGMEM_WRITE_REGMEM32_MASK_CMD_LENGTH, 0, 0 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void lr11xx_regmem_fill_cbuffer_opcode_address( uint8_t* cbuffer, uint16_t opcode, uint32_t address )
{
    cbuffer[0] = ( uint8_t ) ( opcode >> 8 );
    cbuffer[1] = ( uint8_t ) ( opcode >> 0 );

    cbuffer[2] = ( uint8_t ) ( address >> 24 );
    cbuffer[3] = ( uint8_t ) ( address >> 16 );
    cbuffer[4] = ( uint8_t ) ( address >> 8 );
    cbuffer[5] = ( uint8_t ) ( address >> 0 );
}

void lr11xx_regmem_fill_cbuffer_opcode_address_length( uint8_t* cbuffer, uint16_t opcode, uint32_t address,
                                                       uint8_t length )
{
    lr11xx_regmem_fill_cbuffer_opcode_address( cbuffer, opcode, address );
    cbuffer[6] = length;
}

void lr11xx_regmem_fill_cdata( uint8_t* cdata, const uint32_t* data, uint8_t data_length )
{
    for( uint16_t index = 0; index < data_length; index++ )
    {
        uint8_t* cdata_local = &cdata[index * sizeof( uint32_t )];

        cdata_local[0] = ( uint8_t ) ( data[index] >> 24 );
        cdata_local[1] = ( uint8_t ) ( data[index] >> 16 );
        cdata_local[2] = ( uint8_t ) ( data[index] >> 8 );
        cdata_local[3] = ( uint8_t ) ( data[index] >> 0 );
    }
}

void lr11xx_regmem_fill_cbuffer_cdata_opcode_address_data( uint8_t* cbuffer, uint8_t* cdata, uint16_t opcode,
                                                           uint32_t address, const uint32_t* data, uint8_t data_length )
{
    lr11xx_regmem_fill_cbuffer_opcode_address( cbuffer, opcode, address );
    lr11xx_regmem_fill_cdata( cdata, data, data_length );
}

void lr11xx_regmem_fill_out_buffer_from_raw_buffer( uint32_t* out_buffer, const uint8_t* raw_buffer,
                                                    uint8_t out_buffer_length )
{
    for( uint8_t out_index = 0; out_index < out_buffer_length; out_index++ )
    {
        const uint8_t* raw_buffer_local = &raw_buffer[out_index * 4];

        out_buffer[out_index] = ( ( uint32_t ) raw_buffer_local[0] << 24 ) +
                                ( ( uint32_t ) raw_buffer_local[1] << 16 ) + ( ( uint32_t ) raw_buffer_local[2] << 8 ) +
                                ( ( uint32_t ) raw_buffer_local[3] << 0 );
    }
}

/* --- EOF ------------------------------------------------------------------ */
