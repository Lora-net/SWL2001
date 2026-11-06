/**
 * @file      ral_lr20xx_bsp.c
 *
 * @brief     HAL implementation for LR20xx radio chip.
 *
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

#include <stddef.h>
#include "lr20xx_hal.h"
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
static void lr20xx_hal_wait_on_busy( void );

/**
 * @brief Check if device is ready to receive spi transaction.
 * @remark If the device is in sleep mode, it will awake it and wait until it is ready
 */
static void lr20xx_hal_check_device_ready( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr20xx_hal_status_t lr20xx_hal_reset( const void* radio )
{
    hal_gpio_set_value( RADIO_NRST, 0 );
    // wait for 1ms
    hal_mcu_wait_us( 1000 );
    hal_gpio_set_value( RADIO_NRST, 1 );

    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_wakeup( const void* radio )
{
    // Busy is HIGH in sleep mode, wake-up the device with a small glitch on NSS
    hal_gpio_set_value( RADIO_NSS, 0 );
    // wait for 1ms
    hal_mcu_wait_us( 1000 );
    hal_gpio_set_value( RADIO_NSS, 1 );
    radio_mode = RADIO_AWAKE;
    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_read( const void* radio, const uint8_t* cbuffer, const uint16_t cbuffer_length,
                                     uint8_t* rbuffer, const uint16_t rbuffer_length )
{
    uint8_t dummy_bytes[2] = { 0x00, 0x00 };

    lr20xx_hal_check_device_ready( );

    // Put NSS low to start spi transaction
    hal_gpio_set_value( RADIO_NSS, 0 );
    hal_spi_in_out( RADIO_SPI_ID, cbuffer, cbuffer_length, NULL, 0 );
    hal_gpio_set_value( RADIO_NSS, 1 );

    if( rbuffer_length > 0 )
    {
        lr20xx_hal_wait_on_busy( );
        hal_gpio_set_value( RADIO_NSS, 0 );
        // Send dummy bytes
        hal_spi_in_out( RADIO_SPI_ID, 0, 0, dummy_bytes, sizeof( dummy_bytes ) );

        // data read from the radio could be more than 255 bytes but not more than 512 bytes
        if( rbuffer_length > 255 )
        {
            hal_spi_in_out( RADIO_SPI_ID, 0, 0, rbuffer, 255 );
            hal_spi_in_out( RADIO_SPI_ID, 0, 0, &rbuffer[255], rbuffer_length - 255 );
        }
        else
        {
            hal_spi_in_out( RADIO_SPI_ID, 0, 0, rbuffer, rbuffer_length );
        }
        // Put NSS high as the spi transaction is finished
        hal_gpio_set_value( RADIO_NSS, 1 );
    }

    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_write( const void* radio, const uint8_t* cbuffer, const uint16_t cbuffer_length,
                                      const uint8_t* cdata, const uint16_t cdata_length )
{
    lr20xx_hal_check_device_ready( );

    // Put NSS low to start spi transaction
    hal_gpio_set_value( RADIO_NSS, 0 );

    hal_spi_in_out( RADIO_SPI_ID, cbuffer, cbuffer_length, NULL, 0 );
    // data write to the radio could be more than 255 bytes but not more than 512 bytes
    if( cdata_length > 255 )
    {
        hal_spi_in_out( RADIO_SPI_ID, cdata, 255, NULL, 0 );
        hal_spi_in_out( RADIO_SPI_ID, &cdata[255], cdata_length - 255, NULL, 0 );
    }
    else
    {
        if( cdata_length > 0 )
        {
            hal_spi_in_out( RADIO_SPI_ID, cdata, cdata_length, NULL, 0 );
        }
    }

    // Put NSS high as the spi transaction is finished
    hal_gpio_set_value( RADIO_NSS, 1 );

    // Check if command sent is a sleep command LR20XX_SYSTEM_SET_SLEEP_MODE_OC = 0x0127 and save context
    if( ( cbuffer[0] == 0x01 ) && ( cbuffer[1] == 0x27 ) )
    {
        radio_mode = RADIO_SLEEP;

        // add a incompressible delay to prevent trying to wake the radio before it is full asleep
        // TODO check if needed
        hal_mcu_wait_us( 500 );
    }
    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_direct_read( const void* radio, uint8_t* data, const uint16_t data_length )
{
    lr20xx_hal_check_device_ready( );

    // Put NSS low to start spi transaction
    hal_gpio_set_value( RADIO_NSS, 0 );

    // data read from the radio could be more than 255 bytes but not more than 512 bytes
    if( data_length > 255 )
    {
        hal_spi_in_out( RADIO_SPI_ID, 0, 0, data, 255 );
        hal_spi_in_out( RADIO_SPI_ID, 0, 0, &data[255], data_length - 255 );
    }
    else
    {
        hal_spi_in_out( RADIO_SPI_ID, 0, 0, data, data_length );
    }

    // Put NSS high as the spi transaction is finished
    hal_gpio_set_value( RADIO_NSS, 1 );

    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_direct_read_fifo( const void* radio, const uint8_t* command,
                                                 const uint16_t command_length, uint8_t* data,
                                                 const uint16_t data_length )
{
    lr20xx_hal_check_device_ready( );

    // Put NSS low to start spi transaction
    hal_gpio_set_value( RADIO_NSS, 0 );

    // Send command
    hal_spi_in_out( RADIO_SPI_ID, command, command_length, NULL, 0 );

    // Send data
    hal_spi_in_out( RADIO_SPI_ID, 0, 0, data, data_length );

    // Put NSS high as the spi transaction is finished
    hal_gpio_set_value( RADIO_NSS, 1 );

    return LR20XX_HAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lr20xx_hal_wait_on_busy( void )
{
    while( hal_gpio_get_value( RADIO_BUSY_PIN ) == 1 )
    {
    };
}

static void lr20xx_hal_check_device_ready( void )
{
    if( radio_mode != RADIO_SLEEP )
    {
        lr20xx_hal_wait_on_busy( );
    }
    else
    {
        // Busy is HIGH in sleep mode, wake-up the device with a small glitch on NSS
        hal_gpio_set_value( RADIO_NSS, 0 );
        // wait for 1ms
        hal_mcu_wait_us( 1000 );
        hal_gpio_set_value( RADIO_NSS, 1 );
        lr20xx_hal_wait_on_busy( );
        radio_mode = RADIO_AWAKE;
    }
}

/* --- EOF ------------------------------------------------------------------ */
