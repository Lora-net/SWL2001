/*!
 * \file      hw_modem.c
 *
 * \brief     handle hw part of the modem
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

#include "smtc_hal_dbg_trace.h"

#include "hw_modem.h"
#include "cmd_parser.h"
#include "modem_pinout.h"
#include "smtc_modem_utilities.h"
#include "smtc_hal_uart.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_gpio.h"

#include <string.h>  // for memset

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define HW_MODEM_RX_BUFF_MAX_LENGTH 259

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum
{
    HW_MODEM_LP_ENABLE,
    HW_MODEM_LP_DISABLE_ONCE,
    HW_MODEM_LP_DISABLE,
} hw_modem_lp_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static uint8_t            modem_response_buff[HW_MODEM_RX_BUFF_MAX_LENGTH];
static uint8_t            modem_received_buff[HW_MODEM_RX_BUFF_MAX_LENGTH];
static uint8_t            response_length;
static volatile bool      hw_cmd_available             = false;
static volatile bool      is_hw_modem_ready_to_receive = true;
static hal_gpio_irq_t     wakeup_line_irq              = { 0 };
static hw_modem_lp_mode_t lp_mode                      = HW_MODEM_LP_ENABLE;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief prepare and start the reception of the command on a uart using a dma
 * @param [none]
 * @return [none]
 */
void hw_modem_start_reception( void );

/**
 * @brief function that will be called every time the COMMAND line in asserted or de-asserted by the host
 * @param *context  unused context
 * @return none
 */
void wakeup_line_irq_handler( void* context );

/**
 * @brief function that will be called by the soft modem engine each time an async event is available
 * @param *context  unused context
 * @return none
 */
void hw_modem_event_handler( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hw_modem_init( void )
{
    // init hw modem pins
    hal_gpio_init_out( HW_MODEM_EVENT_PIN, 0 );
    hal_gpio_init_out( HW_MODEM_BUSY_PIN, 1 );

    // init irq on COMMAND pin
    wakeup_line_irq.pin      = HW_MODEM_COMMAND_PIN;
    wakeup_line_irq.context  = NULL;
    wakeup_line_irq.callback = wakeup_line_irq_handler;
    hal_gpio_init_in( HW_MODEM_COMMAND_PIN, BSP_GPIO_PULL_MODE_UP, BSP_GPIO_IRQ_MODE_RISING_FALLING, &wakeup_line_irq );

    memset( modem_response_buff, 0, HW_MODEM_RX_BUFF_MAX_LENGTH );
    hw_cmd_available             = false;
    is_hw_modem_ready_to_receive = true;

    // init the soft modem
    smtc_modem_init( &hw_modem_event_handler );

#if defined( PERF_TEST_ENABLED )
    SMTC_HAL_TRACE_WARNING( "HARDWARE MODEM RUNNING PERF TEST MODE\n" );
#endif
}

void hw_modem_start_reception( void )
{
    memset( modem_received_buff, 0xFF, HW_MODEM_RX_BUFF_MAX_LENGTH );

    // during the receive process the hw modem cannot accept an other cmd, prevent it
    is_hw_modem_ready_to_receive = false;

    // receive on dma
    hw_modem_uart_dma_start_rx( modem_received_buff, HW_MODEM_RX_BUFF_MAX_LENGTH );

    // indicate to bridge or host that the modem is ready to receive on uart
    hal_gpio_set_value( HW_MODEM_BUSY_PIN, 0 );
}

void hw_modem_process_cmd( void )
{
    uint8_t              cmd_length = 0xFF;
    cmd_response_t       output;
    cmd_input_t          input;
    cmd_serial_rc_code_t rc_code;

    // check if not false detection (0xFF is default filled buff value)
    if( modem_received_buff[0] < 0xFF )
    {
        cmd_length  = modem_received_buff[1];
        uint8_t crc = 0;
        for( int i = 0; i < cmd_length + 2; i++ )
        {
            crc = crc ^ modem_received_buff[i];
        }
        uint8_t       calculated_crc = crc;
        uint8_t       cmd_crc        = modem_received_buff[cmd_length + 2];
        host_cmd_id_t cmd_id         = ( host_cmd_id_t ) modem_received_buff[0];

        if( calculated_crc != cmd_crc )
        {
            rc_code         = CMD_RC_FRAME_ERROR;
            response_length = 0;
            SMTC_HAL_TRACE_PRINTF( "Cmd with bad crc %x / %x", calculated_crc, cmd_crc );
        }
        else if( ( modem_received_buff[cmd_length + 3] != 0xFF ) && ( cmd_length != 0xFF ) )
        {
            // Too Many cmd enqueued
            rc_code         = CMD_RC_FRAME_ERROR;
            response_length = 0;
            SMTC_HAL_TRACE_WARNING( " Extra data after the command\n" );
        }
        else  // go into soft modem
        {
            SMTC_HAL_TRACE_ARRAY( "Cmd input uart", modem_received_buff, cmd_length + 2 );
            input.cmd_code = cmd_id;
            input.length   = cmd_length;
            input.buffer   = &modem_received_buff[2];
            output.buffer  = &modem_response_buff[2];
            parse_cmd( &input, &output );
            rc_code         = output.return_code;
            response_length = output.length;
        }

        modem_response_buff[0] = rc_code;
        modem_response_buff[1] = response_length;

        SMTC_HAL_TRACE_ARRAY( "Cmd output on uart", modem_response_buff, response_length + 2 );

        // now the hw modem can accept new commands
        is_hw_modem_ready_to_receive = true;
        hw_cmd_available             = false;

        // set busy pin to indicate to bridge or host that the hw_modem answer will be soon sent
        hal_gpio_set_value( HW_MODEM_BUSY_PIN, 1 );

        // wait to to bridge delay
        hal_mcu_wait_us( 1000 );

        for( int i = 0; i < response_length + 2; i++ )
        {
            crc = crc ^ modem_response_buff[i];
        }
        modem_response_buff[response_length + 2] = crc;

        hw_modem_uart_tx( modem_response_buff, response_length + 3 );
    }
    else
    {
        // now the hw modem can accept new commands
        is_hw_modem_ready_to_receive = true;
        hw_cmd_available             = false;

        // set busy pin to indicate to bridge or host that the hw_modem answer will be soon sent
        hal_gpio_set_value( HW_MODEM_BUSY_PIN, 1 );
    }
}

bool hw_modem_is_a_cmd_available( void )
{
    return hw_cmd_available;
}

bool hw_modem_is_low_power_ok( void )
{
    if( lp_mode == HW_MODEM_LP_ENABLE )
    {
        return true;
    }
    else if( lp_mode == HW_MODEM_LP_DISABLE_ONCE )
    {
        // next time lp  will be ok
        lp_mode = HW_MODEM_LP_ENABLE;
        return false;
    }
    else
    {
        return false;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void wakeup_line_irq_handler( void* context )
{
    if( ( hal_gpio_get_value( HW_MODEM_COMMAND_PIN ) == 0 ) && ( is_hw_modem_ready_to_receive == true ) )
    {
        // start receiving uart with dma
        hw_modem_start_reception( );

        // force exit of stop mode
        lp_mode = HW_MODEM_LP_DISABLE;

        // TEMPORARY WORKAROUND to avoid issue for print in hw_modem_process_cmd function
        hal_mcu_wait_us( 2000 );
    }
    if( ( hal_gpio_get_value( HW_MODEM_COMMAND_PIN ) == 1 ) && ( is_hw_modem_ready_to_receive == false ) )
    {
        // stop uart on dma reception
        hw_modem_uart_dma_stop_rx( );

        // inform that a command has arrived
        hw_cmd_available = true;

        // force one more loop in main loop and then re-enable low power feature
        lp_mode = HW_MODEM_LP_DISABLE_ONCE;
    }
}

void hw_modem_event_handler( void )
{
    // raise the event line to indicate to host that events are available
    hal_gpio_set_value( HW_MODEM_EVENT_PIN, 1 );
    SMTC_HAL_TRACE_MSG_COLOR( "Event available\n", HAL_DBG_TRACE_COLOR_BLUE );
}

/* --- EOF ------------------------------------------------------------------ */
