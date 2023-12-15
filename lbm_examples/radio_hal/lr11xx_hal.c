/*!
 * \file      lr11xx_hal.c
 *
 * \brief     Implements the lr11xx radio HAL functions
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

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "lr11xx_hal.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_spi.h"
#include "smtc_hal_mcu.h"

#include "modem_pinout.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum
{
    RADIO_SLEEP,
    RADIO_AWAKE
} radio_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static volatile radio_mode_t radio_mode = RADIO_AWAKE;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Wait until radio busy pin returns to 0
 */
void lr11xx_hal_wait_on_busy( void );

/**
 * @brief Check if device is ready to receive spi transaction.
 * @remark If the device is in sleep mode, it will awake it and wait until it is ready
 */
void lr11xx_hal_check_device_ready( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_hal_status_t lr11xx_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
#if defined( USE_LR11XX_CRC_OVER_SPI )
    // First compute crc
    uint8_t cmd_crc = lr11xx_hal_compute_crc( 0xFF, command, command_length );
    cmd_crc         = lr11xx_hal_compute_crc( cmd_crc, data, data_length );
#endif

    lr11xx_hal_check_device_ready( );

    // Put NSS low to start spi transaction
    hal_gpio_set_value( RADIO_NSS, 0 );
    for( uint16_t i = 0; i < command_length; i++ )
    {
        hal_spi_in_out( RADIO_SPI_ID, command[i] );
    }
    for( uint16_t i = 0; i < data_length; i++ )
    {
        hal_spi_in_out( RADIO_SPI_ID, data[i] );
    }

#if defined( USE_LR11XX_CRC_OVER_SPI )
    // Add crc byte at the end of the transaction
    hal_spi_in_out( RADIO_SPI_ID, cmd_crc );
#endif

    // Put NSS high as the spi transaction is finished
    hal_gpio_set_value( RADIO_NSS, 1 );

    // LR11XX_SYSTEM_SET_SLEEP_OC=0x011B opcode. In sleep mode the radio busy line is held at 1 => do not test it
    if( ( command[0] == 0x01 ) && ( command[1] == 0x1B ) )
    {
        radio_mode = RADIO_SLEEP;

        // add a incompressible delay to prevent trying to wake the radio before it is full asleep
        hal_mcu_wait_us( 500 );
    }

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
#if defined( USE_LR11XX_CRC_OVER_SPI )
    // First compute crc
    uint8_t cmd_crc = lr11xx_hal_compute_crc( 0xFF, command, command_length );
#endif

    lr11xx_hal_check_device_ready( );

    // Put NSS low to start spi transaction
    hal_gpio_set_value( RADIO_NSS, 0 );
    for( uint16_t i = 0; i < command_length; i++ )
    {
        hal_spi_in_out( RADIO_SPI_ID, command[i] );
    }

#if defined( USE_LR11XX_CRC_OVER_SPI )
    // Add crc byte at the end of the transaction
    hal_spi_in_out( RADIO_SPI_ID, cmd_crc );
#endif

    hal_gpio_set_value( RADIO_NSS, 1 );

    if( data_length > 0 )
    {
        lr11xx_hal_check_device_ready( );
        hal_gpio_set_value( RADIO_NSS, 0 );

        // dummy read
#if defined( USE_LR11XX_CRC_OVER_SPI )
        // save dummy for crc calculation
        const uint8_t dummy = hal_spi_in_out( RADIO_SPI_ID, 0 );
#else
        hal_spi_in_out( RADIO_SPI_ID, 0 );
#endif

        for( uint16_t i = 0; i < data_length; i++ )
        {
            data[i] = hal_spi_in_out( RADIO_SPI_ID, 0 );
        }

#if defined( USE_LR11XX_CRC_OVER_SPI )
        // read crc sent by lr11xx at the end of the transaction
        const uint8_t rx_crc = hal_spi_in_out( RADIO_SPI_ID, 0 );
#endif

        // Put NSS high as the spi transaction is finished
        hal_gpio_set_value( RADIO_NSS, 1 );

#if defined( USE_LR11XX_CRC_OVER_SPI )
        // check crc value
        uint8_t computed_crc = lr11xx_hal_compute_crc( 0xFF, &dummy, 1 );
        computed_crc         = lr11xx_hal_compute_crc( computed_crc, data, data_length );
        if( rx_crc != computed_crc )
        {
            SMTC_HAL_TRACE_ERROR( "\x1B[0;31mERROR: lr11xx read function - error on received crc\n" );
            return LR11XX_HAL_STATUS_ERROR;
        }
#endif
    }

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_direct_read( const void* context, uint8_t* data, const uint16_t data_length )
{
    lr11xx_hal_check_device_ready( );

    // Put NSS low to start spi transaction
    hal_gpio_set_value( RADIO_NSS, 0 );

    for( uint16_t i = 0; i < data_length; i++ )
    {
        data[i] = hal_spi_in_out( RADIO_SPI_ID, 0 );
    }

#if defined( USE_LR11XX_CRC_OVER_SPI )
    // read crc sent by lr11xx by sending one more NOP
    const uint8_t rx_crc = hal_spi_in_out( RADIO_SPI_ID, 0 );
#endif

    hal_gpio_set_value( RADIO_NSS, 1 );

#if defined( USE_LR11XX_CRC_OVER_SPI )
    // check crc value
    uint8_t computed_crc = lr11xx_hal_compute_crc( 0xFF, data, data_length );
    if( rx_crc != computed_crc )
    {
        SMTC_HAL_TRACE_ERROR( "\x1B[0;31mERROR: lr11xx read function - error on received crc\n" );
        return LR11XX_HAL_STATUS_ERROR;
    }
#endif

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_reset( const void* context )
{
    hal_gpio_set_value( RADIO_NRST, 0 );
    hal_mcu_wait_us( 5000 );
    hal_gpio_set_value( RADIO_NRST, 1 );
    hal_mcu_wait_us( 5000 );

    // Wait 200ms until internal lr11xx fw is ready
    hal_mcu_wait_us( 200000 );
    radio_mode = RADIO_AWAKE;

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_wakeup( const void* context )
{
    lr11xx_hal_check_device_ready( );
    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_abort_blocking_cmd( const void* context )
{
    uint8_t command[4] = { 0 };

    // Put NSS low to start spi transaction
    hal_gpio_set_value( RADIO_NSS, 0 );
    for( uint16_t i = 0; i < sizeof( command ); i++ )
    {
        hal_spi_in_out( RADIO_SPI_ID, command[i] );
    }

    // Put NSS high as the spi transaction is finished
    hal_gpio_set_value( RADIO_NSS, 1 );

    lr11xx_hal_wait_on_busy( );

    return LR11XX_HAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void lr11xx_hal_wait_on_busy( void )
{
    while( hal_gpio_get_value( RADIO_BUSY_PIN ) == 1 )
    {
    };
}

void lr11xx_hal_check_device_ready( void )
{
    if( radio_mode != RADIO_SLEEP )
    {
        lr11xx_hal_wait_on_busy( );
    }
    else
    {
        // Busy is HIGH in sleep mode, wake-up the device with a small glitch on NSS
        hal_gpio_set_value( RADIO_NSS, 0 );
        hal_gpio_set_value( RADIO_NSS, 1 );
        lr11xx_hal_wait_on_busy( );
        radio_mode = RADIO_AWAKE;
    }
}

/* --- EOF ------------------------------------------------------------------ */
