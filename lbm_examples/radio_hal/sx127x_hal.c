/*!
 * \file      sx127x_hal.c
 *
 * \brief     Implements the sx127x radio HAL functions
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

#include "sx127x.h"
#include "sx127x_hal.h"

#include "smtc_hal_gpio.h"
#include "smtc_hal_spi.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_lp_timer.h"
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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static bool               is_timer_started = false;
static hal_lp_timer_irq_t tmr_irq;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

sx127x_radio_id_t sx127x_hal_get_radio_id( const sx127x_t* radio )
{
#if defined( SX1272 )
    return SX127X_RADIO_ID_SX1272;
#elif defined( SX1276 )
    return SX127X_RADIO_ID_SX1276;
#else
#error "Please define the radio to be used"
#endif
}

void sx127x_hal_dio_irq_attach( const sx127x_t* radio )
{
    static hal_gpio_irq_t radio_dio_0_irq;
    static hal_gpio_irq_t radio_dio_1_irq;
    static hal_gpio_irq_t radio_dio_2_irq;

    radio_dio_0_irq.context  = ( void* ) radio;
    radio_dio_0_irq.pin      = RADIO_DIO_0;
    radio_dio_0_irq.callback = radio->dio_0_irq_handler;
    hal_gpio_irq_attach( &radio_dio_0_irq );

    radio_dio_1_irq.context  = ( void* ) radio;
    radio_dio_1_irq.pin      = RADIO_DIO_1;
    radio_dio_1_irq.callback = radio->dio_1_irq_handler;
    hal_gpio_irq_attach( &radio_dio_1_irq );

    radio_dio_2_irq.context  = ( void* ) radio;
    radio_dio_2_irq.pin      = RADIO_DIO_2;
    radio_dio_2_irq.callback = radio->dio_2_irq_handler;
    hal_gpio_irq_attach( &radio_dio_2_irq );
}

sx127x_hal_status_t sx127x_hal_write( const sx127x_t* radio, const uint16_t address, const uint8_t* data,
                                      const uint16_t data_len )
{
    CRITICAL_SECTION_BEGIN( );

    hal_gpio_set_value( RADIO_NSS, 0 );

    hal_spi_in_out( RADIO_SPI_ID, address | 0x80 );
    for( uint16_t i = 0; i < data_len; i++ )
    {
        hal_spi_in_out( RADIO_SPI_ID, data[i] );
    }

    hal_gpio_set_value( RADIO_NSS, 1 );

    CRITICAL_SECTION_END( );

    return SX127X_HAL_STATUS_OK;
}

sx127x_hal_status_t sx127x_hal_read( const sx127x_t* radio, const uint16_t address, uint8_t* data,
                                     const uint16_t data_len )
{
    CRITICAL_SECTION_BEGIN( );

    hal_gpio_set_value( RADIO_NSS, 0 );

    hal_spi_in_out( RADIO_SPI_ID, address & ( ~0x80 ) );
    for( uint16_t i = 0; i < data_len; i++ )
    {
        data[i] = hal_spi_in_out( RADIO_SPI_ID, 0 );
    }

    hal_gpio_set_value( RADIO_NSS, 1 );

    CRITICAL_SECTION_END( );

    return SX127X_HAL_STATUS_OK;
}

void sx127x_hal_reset( const sx127x_t* radio )
{
#if defined( SX1272 )
    // Set RESET pin to 1
    hal_gpio_init_out( RADIO_NRST, 1 );
#elif defined( SX1276 )
    // Set RESET pin to 0
    hal_gpio_init_out( RADIO_NRST, 0 );
#else
#error "Please define the radio to be used"
#endif

    // Wait 1 ms
    hal_mcu_wait_us( 1000 );

    // Configure RESET pin as input
    hal_gpio_init_in( RADIO_NRST, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_OFF, NULL );

    hal_mcu_wait_us( 6000 );
}

uint32_t sx127x_hal_get_dio_1_pin_state( const sx127x_t* radio )
{
    return hal_gpio_get_value( RADIO_DIO_1 );
}

sx127x_hal_status_t sx127x_hal_timer_start( const sx127x_t* radio, const uint32_t time_in_ms,
                                            void ( *callback )( void* context ) )
{
    tmr_irq.context  = ( void* ) radio;
    tmr_irq.callback = callback;
    hal_lp_timer_start( HAL_LP_TIMER_ID_2, time_in_ms, &tmr_irq );
    is_timer_started = true;
    return SX127X_HAL_STATUS_OK;
}

sx127x_hal_status_t sx127x_hal_timer_stop( const sx127x_t* radio )
{
    hal_lp_timer_stop( HAL_LP_TIMER_ID_2 );
    is_timer_started = false;
    return SX127X_HAL_STATUS_OK;
}

bool sx127x_hal_timer_is_started( const sx127x_t* radio )
{
    return is_timer_started;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
