/*!
 * \file      smtc_hal_mcu.c
 *
 * \brief     MCU Hardware Abstraction Layer implementation
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
#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#endif

#include <stdbool.h>  // bool type
#include <stdint.h>   // C99 types

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"

#include "modem_pinout.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_rng.h"
#include "smtc_hal_rtc.h"
#include "smtc_hal_spi.h"
#include "smtc_hal_uart.h"
#include "smtc_hal_watchdog.h"
#include "smtc_modem_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

// Low power is enabled (0 will disable it)
#define LOW_POWER_MODE 1
// 1 to enable debug with probe (ie do not de init pins)
#ifndef HW_DEBUG_PROBE
#define HW_DEBUG_PROBE 0
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void mcu_gpio_init( void );
static void sleep_handler( void );
static void lpm_enter_sleep_mode( void );
static void lpm_exit_sleep_mode( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_mcu_critical_section_begin( uint32_t* mask )
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void hal_mcu_critical_section_end( uint32_t* mask )
{
    __set_PRIMASK( *mask );
}

void hal_mcu_disable_irq( void )
{
    __disable_irq( );
}

void hal_mcu_enable_irq( void )
{
    __enable_irq( );
}

void hal_mcu_init( void )
{
    hal_rtc_init( );
    hal_watchdog_init( );
    mcu_gpio_init( );
    hal_rng_init( );
    hal_spi_init( 0 );

#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
    uart0_init( );
#endif
}

void hal_mcu_reset( void )
{
    __disable_irq( );
    NVIC_SystemReset( );  // Restart system
}

void hal_mcu_wait_us( const int32_t microseconds )
{
    nrf_delay_us( microseconds );
}

void hal_mcu_set_sleep_for_ms( const int32_t milliseconds )
{
    if( milliseconds <= 0 )
    {
        return;
    }

    hal_rtc_wakeup_timer_set_ms( milliseconds );
    sleep_handler( );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION
 * --------------------------------------------
 */
static void mcu_gpio_init( void )
{
    //   uint32_t err_code;
    nrf_drv_gpiote_init( );
    //   VERIFY_SUCCESS(err_code);

    hal_gpio_init_out( RADIO_NSS, 1 );
    hal_gpio_init_in( RADIO_BUSY_PIN, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_OFF, NULL );

    // Here init only the pin as an exti rising and the callback will be attached
    // later
    hal_gpio_init_in( RADIO_DIOX, BSP_GPIO_PULL_MODE_DOWN, BSP_GPIO_IRQ_MODE_RISING, NULL );
    hal_gpio_init_out( RADIO_NRST, 1 );
#if defined( SX128X )
    hal_gpio_init_out( RADIO_ANTENNA_SWITCH, 1 );
#elif defined( LR11XX_TRANSCEIVER ) && defined( ENABLE_MODEM_GNSS_FEATURE )
    hal_gpio_init_out( RADIO_LNA_CTRL, 0 );
    hal_gpio_init_out( SMTC_LED_TX, 0 );
    hal_gpio_init_out( SMTC_LED_RX, 0 );
#elif defined( SX126X )
    // If the sx126x drives the rf switch with dio2, just put the SX126X_RADIO_RF_SWITCH_CTRL in pull up
    hal_gpio_init_in( SX126X_RADIO_RF_SWITCH_CTRL, BSP_GPIO_PULL_MODE_UP, BSP_GPIO_IRQ_MODE_OFF, NULL );
#endif
}

/**
 * @brief Either enters Low Power Stop Mode or calls WFI
 *
 * @note ARM exits the function when waking up
 *
 */
static void lpm_enter_sleep_mode( void )
{
#if( LOW_POWER_MODE == 1 )
    // Deinit periph & enter Stop Mode
    hal_spi_de_init( RADIO_SPI_ID );
    hal_rng_stop( );

#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
    uart0_deinit( );
#endif

#endif  // LOW_POWER_MODE

    __WFI( );
}

/**
 * @brief Wakes from Low Power Stop Mode or from WFI
 *
 */
static void lpm_exit_sleep_mode( void )
{
    hal_gpio_irq_disable( );

#if( LOW_POWER_MODE == 1 )
    // Initialize UART
#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
    uart0_init( );
#endif

    // Initialize SPI
    hal_spi_init( 0 );
    hal_rng_start( );
    hal_gpio_irq_enable( );
#endif  // LOW_POWER_MODE
}

/**
 * @brief Sleep handler
 *
 */
static void sleep_handler( void )
{
    // If an interrupt has occurred after entering critical section, it is kept pending
    // and cortex will not enter low power anyway

    lpm_enter_sleep_mode( );
    lpm_exit_sleep_mode( );
}

#if( LOW_POWER_MODE == 1 )

#endif