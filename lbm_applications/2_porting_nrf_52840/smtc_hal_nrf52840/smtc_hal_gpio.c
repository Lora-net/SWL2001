/*!
 * \file      smtc_hal_gpio.c
 *
 * \brief     GPIO Hardware Abstraction Layer implementation
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
#include <stdbool.h>  // bool type
#include <stdint.h>   // C99 types

#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "smtc_hal_gpio.h"

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

/*!
 * GPIO setup data structure
 */
// typedef struct bsp_gpio_s {
//   uint8_t pin;
//   uint32_t mode;
//   uint32_t pull;
//   uint32_t speed;
//   uint32_t alternate;
// } gpio_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * Array holding attached IRQ gpio data context
 */
static hal_gpio_irq_t const* gpio_irq[47];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void gpiote_event_handler( nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action );

/*!
 * Generic gpio initialization
 *
 * \param [in/out] gpio  Holds MCU gpio parameters
 * \param [in]     value Initial MCU pit value
 * \param [in/out] irq   Pointer to IRQ data context.
 *                         NULL when setting gpio as output
 */
// static void gpio_init(const gpio_t *gpio, const uint32_t value,
//                       const hal_gpio_irq_t *irq);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

//
// MCU input pin Handling
//

void hal_gpio_init_in( const uint8_t pin, const hal_gpio_pull_mode_t pull_mode, const hal_gpio_irq_mode_t irq_mode,
                       hal_gpio_irq_t* irq )
{
    const uint32_t modes[] = { GPIOTE_CONFIG_POLARITY_None, GPIOTE_CONFIG_POLARITY_LoToHi,
                               GPIOTE_CONFIG_POLARITY_HiToLo, GPIOTE_CONFIG_POLARITY_Toggle };

    const uint32_t pulls[] = { NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_PULLDOWN };

    nrfx_gpiote_in_config_t config = {
        .sense      = ( nrf_gpiote_polarity_t ) modes[irq_mode],
        .pull       = pulls[pull_mode],
        .is_watcher = 0,
        .hi_accuracy =
            false,  // check the NRFX_GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS number (IT radio, button, HW_modem, ..)
        .skip_gpio_setup = false
    };

    nrf_drv_gpiote_in_init( pin, &config, gpiote_event_handler );

    if( irq != NULL )
    {
        irq->pin = pin;
        hal_gpio_irq_attach( irq );
        nrfx_gpiote_in_event_enable( pin, true );
    }
    else
    {
        nrfx_gpiote_in_event_enable( pin, false );
    }
}

void hal_gpio_init_out( const uint8_t pin, const uint32_t value )
{
    nrf_gpio_cfg( pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1,
                  NRF_GPIO_PIN_NOSENSE );

    nrf_gpio_pin_write( pin, value );
}

void hal_gpio_irq_attach( const hal_gpio_irq_t* irq )
{
    if( ( irq != NULL ) && ( irq->callback != NULL ) )
    {
        gpio_irq[( irq->pin )] = irq;
        nrfx_gpiote_in_event_enable( irq->pin, true );
    }
}

void hal_gpio_irq_deatach( const hal_gpio_irq_t* irq )
{
    if( irq != NULL )
    {
        gpio_irq[( irq->pin )] = NULL;
        nrfx_gpiote_in_event_enable( irq->pin, false );
    }
}

void hal_gpio_irq_enable( void )
{
    NVIC_EnableIRQ( GPIOTE_IRQn );
}

void hal_gpio_irq_disable( void )
{
    NVIC_DisableIRQ( GPIOTE_IRQn );
}

//
// MCU pin state control
//

void hal_gpio_set_value( const uint8_t pin, const uint32_t value )
{
    nrf_gpio_pin_write( pin, value );
}

uint32_t hal_gpio_get_value( const uint8_t pin )
{
    return nrf_gpio_pin_read( pin );
}

void hal_gpio_clear_pending_irq( const uint8_t pin )
{
    NVIC_ClearPendingIRQ( GPIOTE_IRQn );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */
/* GPIOTE event is used only to start periodic timer when first button is
 * activated. */
static void gpiote_event_handler( nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action )
{
    uint8_t callback_index = pin;

    if( ( gpio_irq[callback_index] != NULL ) && ( gpio_irq[callback_index]->callback != NULL ) )
    {
        gpio_irq[callback_index]->callback( gpio_irq[callback_index]->context );
    }
}

/* --- EOF ------------------------------------------------------------------ */
