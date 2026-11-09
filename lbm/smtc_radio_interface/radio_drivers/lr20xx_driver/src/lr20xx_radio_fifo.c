/*!
 * @file      lr20xx_radio_fifo.c
 *
 * @brief     Radio FiFo driver definition for LR20XX
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

#include "lr20xx_radio_fifo.h"
#include "lr20xx_hal.h"
#include "lr20xx_regmem.h"
#include "lr20xx_system.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR20XX_RADIO_FIFO_TX_FIFO_ADDRESS ( 0x00F3002C )
#define LR20XX_RADIO_FIFO_TX_FIFO_SIZE_ADDRESS ( 0x00F30034 )
#define LR20XX_RADIO_FIFO_RX_FIFO_ADDRESS ( 0x00F30028 )
#define LR20XX_RADIO_FIFO_RX_FIFO_SIZE_ADDRESS ( 0x00F30030 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR20XX_RADIO_FIFO_READ_RX_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_FIFO_WRITE_TX_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_FIFO_CLEAR_IRQ_FLAGS_CMD_LENGTH ( 2 + 2 )
#define LR20XX_RADIO_FIFO_CFG_IRQ_CMD_LENGTH ( 2 + 10 )
#define LR20XX_RADIO_FIFO_GET_IRQ_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_FIFO_GET_RX_LEVEL_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_FIFO_GET_TX_LEVEL_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_FIFO_CLEAR_RX_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_FIFO_CLEAR_TX_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_FIFO_GET_AND_CLEAR_IRQ_FLAGS_CMD_LENGTH ( 2 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for register and memory related operations
 */
enum
{
    LR20XX_RADIO_FIFO_READ_RX_OC                 = 0x0001,
    LR20XX_RADIO_FIFO_WRITE_TX_OC                = 0x0002,
    LR20XX_RADIO_FIFO_CLEAR_FIFO_IRQ_FLAGS_OC    = 0x0114,
    LR20XX_RADIO_FIFO_CFG_IRQ_OC                 = 0x011A,
    LR20XX_RADIO_FIFO_GET_IRQ_OC                 = 0x011B,
    LR20XX_RADIO_FIFO_GET_RX_LEVEL_OC            = 0x011C,
    LR20XX_RADIO_FIFO_GET_TX_LEVEL_OC            = 0x011D,
    LR20XX_RADIO_FIFO_CLEAR_RX_OC                = 0x011E,
    LR20XX_RADIO_FIFO_GET_AND_CLEAR_IRQ_FLAGS_OC = 0x012E,
    LR20XX_RADIO_FIFO_CLEAR_TX_OC                = 0x011F,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr20xx_status_t lr20xx_radio_fifo_read_rx( const void* context, uint8_t* buffer, const uint16_t length )
{
    const uint8_t cbuffer[LR20XX_RADIO_FIFO_READ_RX_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_FIFO_READ_RX_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_FIFO_READ_RX_OC >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_direct_read_fifo( context, cbuffer, LR20XX_RADIO_FIFO_READ_RX_CMD_LENGTH,
                                                            buffer, length );
}

lr20xx_status_t lr20xx_radio_fifo_write_tx( const void* context, const uint8_t* buffer, const uint16_t length )
{
    const uint8_t cbuffer[LR20XX_RADIO_FIFO_WRITE_TX_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_FIFO_WRITE_TX_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_FIFO_WRITE_TX_OC >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_FIFO_WRITE_TX_CMD_LENGTH, buffer,
                                                 length );
}

lr20xx_status_t lr20xx_radio_fifo_clear_rx( const void* context )
{
    const uint8_t cbuffer[LR20XX_RADIO_FIFO_CLEAR_RX_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_FIFO_CLEAR_RX_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_FIFO_CLEAR_RX_OC >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_FIFO_CLEAR_RX_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_fifo_clear_tx( const void* context )
{
    const uint8_t cbuffer[LR20XX_RADIO_FIFO_CLEAR_TX_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_FIFO_CLEAR_TX_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_FIFO_CLEAR_TX_OC >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_FIFO_CLEAR_TX_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_fifo_get_rx_level( const void* context, uint16_t* fifo_level )
{
    const uint8_t cbuffer[LR20XX_RADIO_FIFO_GET_RX_LEVEL_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_FIFO_GET_RX_LEVEL_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_FIFO_GET_RX_LEVEL_OC >> 0 ),
    };
    uint8_t fifo_level_uint8[2] = { 0 };

    const lr20xx_status_t status = ( lr20xx_status_t ) lr20xx_hal_read(
        context, cbuffer, LR20XX_RADIO_FIFO_GET_RX_LEVEL_CMD_LENGTH, fifo_level_uint8, 2 );

    if( status == LR20XX_STATUS_OK )
    {
        *fifo_level = ( uint16_t )( ( ( uint16_t ) fifo_level_uint8[0] << 8 ) + fifo_level_uint8[1] );
    }

    return status;
}

lr20xx_status_t lr20xx_radio_fifo_get_tx_level( const void* context, uint16_t* fifo_level )
{
    const uint8_t cbuffer[LR20XX_RADIO_FIFO_GET_TX_LEVEL_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_FIFO_GET_TX_LEVEL_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_FIFO_GET_TX_LEVEL_OC >> 0 ),
    };
    uint8_t fifo_level_uint8[2] = { 0 };

    const lr20xx_status_t status = ( lr20xx_status_t ) lr20xx_hal_read(
        context, cbuffer, LR20XX_RADIO_FIFO_GET_TX_LEVEL_CMD_LENGTH, fifo_level_uint8, 2 );

    if( status == LR20XX_STATUS_OK )
    {
        *fifo_level = ( uint16_t )( ( ( uint16_t ) fifo_level_uint8[0] << 8 ) + fifo_level_uint8[1] );
    }

    return status;
}

lr20xx_status_t lr20xx_radio_fifo_cfg_irq( const void* context, lr20xx_radio_fifo_flag_t rx_fifo_irq_enable,
                                           lr20xx_radio_fifo_flag_t tx_fifo_irq_enable, uint16_t rx_fifo_high_threshold,
                                           uint16_t tx_fifo_low_threshold, uint16_t rx_fifo_low_threshold,
                                           uint16_t tx_fifo_high_threshold )
{
    const uint8_t cbuffer[LR20XX_RADIO_FIFO_CFG_IRQ_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_FIFO_CFG_IRQ_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_FIFO_CFG_IRQ_OC >> 0 ),
        rx_fifo_irq_enable,
        tx_fifo_irq_enable,
        ( uint8_t )( rx_fifo_high_threshold >> 8 ),
        ( uint8_t )( rx_fifo_high_threshold >> 0 ),
        ( uint8_t )( tx_fifo_low_threshold >> 8 ),
        ( uint8_t )( tx_fifo_low_threshold >> 0 ),
        ( uint8_t )( rx_fifo_low_threshold >> 8 ),
        ( uint8_t )( rx_fifo_low_threshold >> 0 ),
        ( uint8_t )( tx_fifo_high_threshold >> 8 ),
        ( uint8_t )( tx_fifo_high_threshold >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_FIFO_CFG_IRQ_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_fifo_clear_irq_flags( const void* context, lr20xx_radio_fifo_flag_t rx_fifo_flags_to_clear,
                                                   lr20xx_radio_fifo_flag_t tx_fifo_flags_to_clear )
{
    const uint8_t cbuffer[LR20XX_RADIO_FIFO_CLEAR_IRQ_FLAGS_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_FIFO_CLEAR_FIFO_IRQ_FLAGS_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_FIFO_CLEAR_FIFO_IRQ_FLAGS_OC >> 0 ),
        rx_fifo_flags_to_clear,
        tx_fifo_flags_to_clear,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_FIFO_CLEAR_IRQ_FLAGS_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_fifo_get_irq( const void* context, lr20xx_radio_fifo_flag_t* rx_fifo_flags,
                                           lr20xx_radio_fifo_flag_t* tx_fifo_flags )
{
    const uint8_t cbuffer[LR20XX_RADIO_FIFO_GET_IRQ_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_FIFO_GET_IRQ_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_FIFO_GET_IRQ_OC >> 0 ),
    };

    uint8_t rbuffer[2] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RADIO_FIFO_GET_IRQ_CMD_LENGTH, rbuffer, 2 );

    if( status == LR20XX_STATUS_OK )
    {
        *rx_fifo_flags = rbuffer[0];
        *tx_fifo_flags = rbuffer[1];
    }

    return status;
}

lr20xx_status_t lr20xx_radio_fifo_get_and_clear_irq_flags( const void* context, lr20xx_radio_fifo_flag_t* rx_fifo_flags,
                                                           lr20xx_radio_fifo_flag_t* tx_fifo_flags )
{
    const uint8_t cbuffer[LR20XX_RADIO_FIFO_GET_AND_CLEAR_IRQ_FLAGS_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_FIFO_GET_AND_CLEAR_IRQ_FLAGS_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_FIFO_GET_AND_CLEAR_IRQ_FLAGS_OC >> 0 ),
    };
    uint8_t flags[2] = { 0 };

    const lr20xx_status_t status = ( lr20xx_status_t ) lr20xx_hal_read(
        context, cbuffer, LR20XX_RADIO_FIFO_GET_AND_CLEAR_IRQ_FLAGS_CMD_LENGTH, flags, 2 );

    if( status == LR20XX_STATUS_OK )
    {
        *rx_fifo_flags = flags[0];
        *tx_fifo_flags = flags[1];
    }

    return status;
}

lr20xx_status_t lr20xx_radio_fifo_configure_1024_byte_tx_fifo( const void* context )
{
    const uint32_t tx_fifo_1024_memory_address = 0x00804000;
    RETURN_STATUS_ON_NOT_OK(
        lr20xx_regmem_write_regmem32( context, LR20XX_RADIO_FIFO_TX_FIFO_ADDRESS, &tx_fifo_1024_memory_address, 1 ) );

    const uint32_t tx_fifo_size = 0x000003FC;
    return lr20xx_regmem_write_regmem32( context, LR20XX_RADIO_FIFO_TX_FIFO_SIZE_ADDRESS, &tx_fifo_size, 1 );
}

lr20xx_status_t lr20xx_radio_fifo_1024_byte_tx_fifo_store_retention_mem( const void* context,
                                                                         uint8_t     retention_slot_address,
                                                                         uint8_t     retention_slot_size )
{
    // Store the registers configuration for the TX FIFOs location in retention
    RETURN_STATUS_ON_NOT_OK( lr20xx_system_add_register_to_retention_mem( context, retention_slot_address,
                                                                          LR20XX_RADIO_FIFO_TX_FIFO_ADDRESS ) );

    // Store the registers configuration for the TX FIFOs size in retention
    return lr20xx_system_add_register_to_retention_mem( context, retention_slot_size,
                                                        LR20XX_RADIO_FIFO_TX_FIFO_SIZE_ADDRESS );
}

lr20xx_status_t lr20xx_radio_fifo_configure_1024_byte_rx_fifo( const void* context )
{
    const uint32_t rx_fifo_1024_memory_address = 0x00804400;
    RETURN_STATUS_ON_NOT_OK(
        lr20xx_regmem_write_regmem32( context, LR20XX_RADIO_FIFO_RX_FIFO_ADDRESS, &rx_fifo_1024_memory_address, 1 ) );

    const uint32_t rx_fifo_size = 0x000003E8;
    return lr20xx_regmem_write_regmem32( context, LR20XX_RADIO_FIFO_RX_FIFO_SIZE_ADDRESS, &rx_fifo_size, 1 );
}

lr20xx_status_t lr20xx_radio_fifo_1024_byte_rx_fifo_store_retention_mem( const void* context,
                                                                         uint8_t     retention_slot_address,
                                                                         uint8_t     retention_slot_size )
{
    // Store the registers configuration for the RX FIFOs location in retention
    RETURN_STATUS_ON_NOT_OK( lr20xx_system_add_register_to_retention_mem( context, retention_slot_address,
                                                                          LR20XX_RADIO_FIFO_RX_FIFO_ADDRESS ) );

    // Store the registers configuration for the RX FIFOs size in retention
    return lr20xx_system_add_register_to_retention_mem( context, retention_slot_size,
                                                        LR20XX_RADIO_FIFO_RX_FIFO_SIZE_ADDRESS );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
