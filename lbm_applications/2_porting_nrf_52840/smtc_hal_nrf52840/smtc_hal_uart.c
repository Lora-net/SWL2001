/*!
 * \file      smtc_hal_uart.c
 *
 * \brief     UART Hardware Abstraction Layer implementation
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

#include "nrf_uarte.h"
#include "nrf_drv_uart.h"

#include "modem_pinout.h"
#include "smtc_hal_uart.h"

#include "sdk_common.h"
#include "nrf_assert.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define UART_TX_BUF_SIZE 256 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256 /**< UART RX buffer size. */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static uint8_t rx_buffer[1];

// static uint8_t modem_received_buff[50];

static volatile bool tx_done[2];
static volatile bool rx_done[2];

static nrf_drv_uart_t uart0_inst = NRF_DRV_UART_INSTANCE( 0 );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void uart_event_handler( nrf_drv_uart_event_t* p_event, void* p_context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void uart0_init( void )
{
    nrf_drv_uart_config_t config = NRF_DRV_UART_DEFAULT_CONFIG;
    config.baudrate              = ( nrf_uart_baudrate_t ) NRF_UARTE_BAUDRATE_115200;
    config.hwfc                  = NRF_UART_HWFC_DISABLED;
    config.interrupt_priority    = APP_IRQ_PRIORITY_MID;
    config.parity                = NRF_UART_PARITY_EXCLUDED;
    config.pselcts               = Px_NC;  // not connected
    config.pselrts               = Px_NC;  // not connected
    config.pselrxd               = DEBUG_UART_RX;
    config.pseltxd               = DEBUG_UART_TX;
    config.p_context             = &uart0_inst;

    rx_done[0] = false;

    nrf_drv_uart_init( &uart0_inst, &config, uart_event_handler );
    nrf_drv_uart_rx_disable( &uart0_inst );
}

void uart0_deinit( void )
{
    nrf_drv_uart_uninit( &uart0_inst );
}

void uart0_tx( uint8_t* buff, uint8_t len )
{
    tx_done[0] = false;
    nrf_drv_uart_tx( &uart0_inst, buff, len );
    // wait Tx done event
    while( tx_done[0] == false )
    {
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void uart_event_handler( nrf_drv_uart_event_t* p_event, void* p_context )
{
    nrf_drv_uart_t* uart_inst = ( nrf_drv_uart_t* ) p_context;
    if( p_event->type == NRF_DRV_UART_EVT_RX_DONE )
    {
        // Received bytes counter has to be checked, because there could be event from RXTO interrupt
        if( p_event->data.rxtx.bytes )
        {
            rx_done[uart_inst->inst_idx] = true;
            ( void ) nrf_drv_uart_rx( uart_inst, rx_buffer, 1 );
        }
    }
    else if( p_event->type == NRF_DRV_UART_EVT_ERROR )
    {
        ( void ) nrf_drv_uart_rx( uart_inst, rx_buffer, 1 );
    }
    else if( p_event->type == NRF_DRV_UART_EVT_TX_DONE )
    {
        // Last byte from FIFO transmitted, notify the application.
        // Notify that new data is available if this was first byte put in the buffer.
        tx_done[uart_inst->inst_idx] = true;
    }
}

/* --- EOF ------------------------------------------------------------------ */
