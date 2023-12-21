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

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_hal_mcu.h"
#include "modem_pinout.h"

#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_bus.h"

#include "smtc_hal_uart.h"
#include "smtc_hal_rtc.h"
#include "smtc_hal_spi.h"
#include "smtc_hal_lp_timer.h"
#include "smtc_hal_watchdog.h"

#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#endif
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
static void system_clock_config( void );
// static void mcu_start_systick( void );
static void mcu_gpio_init( void );
static void lpm_enter_sleep_mode( void );
static void lpm_exit_sleep_mode( void );
static void sleep_handler( void );

#if( LOW_POWER_MODE == 1 )
static void lpm_mcu_deinit( void );
static void lpm_mcu_reinit( void );
static void system_clock_re_config_after_stop( void );

#endif

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
    // System interrupt init
    // SVC_IRQn interrupt configuration
    NVIC_SetPriority( SVC_IRQn, 0 );
    // PendSV_IRQn interrupt configuration
    NVIC_SetPriority( PendSV_IRQn, 0 );
    // SysTick_IRQn interrupt configuration
    NVIC_SetPriority( SysTick_IRQn, 0 );

    // Initialize clocks
    system_clock_config( );

    // Initialize watchdog
    hal_watchdog_init( );

#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
    // Initialize Uart for debug traces
    trace_uart_init( );
#endif
    // Initialize GPIOs
    mcu_gpio_init( );

    // Initialize Low Power Timer
    hal_lp_timer_init( HAL_LP_TIMER_ID_1 );

    // Initialize SPI for radio
    hal_spi_init( RADIO_SPI_ID, RADIO_SPI_MOSI, RADIO_SPI_MISO, RADIO_SPI_SCLK );

    // Initialize RTC (for real time and wut)
    hal_rtc_init( );
}

void hal_mcu_reset( void )
{
    __disable_irq( );
    NVIC_SystemReset( );  // Restart system
}

void __attribute__( ( optimize( "O0" ) ) ) hal_mcu_wait_us( const int32_t microseconds )
{
    // Work @80MHz
    const uint32_t nb_nop = microseconds * 1000 / 561;  // TODO validate value by measures (value comes from first lbm)
    for( uint32_t i = 0; i < nb_nop; i++ )
    {
        __NOP( );
    }
}

void hal_mcu_set_sleep_for_ms( const int32_t milliseconds )
{
    if( milliseconds <= 0 )
    {
        return;
    }

    hal_rtc_wakeup_timer_set_ms( milliseconds );
    sleep_handler( );
    // stop wake up timer
    hal_rtc_wakeup_timer_stop( );
}

#ifdef USE_FULL_ASSERT
/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line
 * number where the assert_param error has occurred. Input          : - file:
 * pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    // User can add his own implementation to report the file name and line
    // number,
    // ex: printf("Wrong parameters value: file %s on line %lu\r\n", file, line)

    SMTC_HAL_TRACE_PRINTF( "Wrong parameters value: file %s on line %lu\r\n", ( const char* ) file, line );
    // Infinite loop
    while( 1 )
    {
    }
}
#endif

void SysTick_Handler( void )
{
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void system_clock_config( void )
{
    LL_FLASH_SetLatency( LL_FLASH_LATENCY_1 );

    if( LL_FLASH_GetLatency( ) != LL_FLASH_LATENCY_1 )
    {
        mcu_panic( );
    }
    LL_PWR_SetRegulVoltageScaling( LL_PWR_REGU_VOLTAGE_SCALE1 );
    LL_RCC_HSI_Enable( );

    /* Wait till HSI is ready */
    while( LL_RCC_HSI_IsReady( ) != 1 )
    {
    }
    LL_RCC_HSI_SetCalibTrimming( 16 );
    LL_RCC_LSI_Enable( );

    /* Wait till LSI is ready */
    while( LL_RCC_LSI_IsReady( ) != 1 )
    {
    }

    LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_PWR );
    LL_PWR_EnableBkUpAccess( );
    LL_RCC_ForceBackupDomainReset( );
    LL_RCC_ReleaseBackupDomainReset( );
    // LL_RCC_LSE_SetDriveCapability( LL_RCC_LSEDRIVE_LOW );
    LL_RCC_LSE_Enable( );

    /* Wait till LSE is ready */
    while( LL_RCC_LSE_IsReady( ) != 1 )
    {
    }
    LL_RCC_SetRTCClockSource( LL_RCC_RTC_CLKSOURCE_LSE );
    // LL_RCC_EnableRTC( );
    LL_RCC_PLL_ConfigDomain_SYS( LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3 );
    LL_RCC_PLL_Enable( );

    /* Wait till PLL is ready */
    while( LL_RCC_PLL_IsReady( ) != 1 )
    {
    }
    LL_RCC_SetAHBPrescaler( LL_RCC_SYSCLK_DIV_1 );
    LL_RCC_SetAPB1Prescaler( LL_RCC_APB1_DIV_1 );
    LL_RCC_SetAPB2Prescaler( LL_RCC_APB2_DIV_1 );
    LL_RCC_SetSysClkSource( LL_RCC_SYS_CLKSOURCE_PLL );

    /* Wait till System clock is ready */
    while( LL_RCC_GetSysClkSource( ) != LL_RCC_SYS_CLKSOURCE_STATUS_PLL )
    {
    }

    LL_Init1msTick( 32000000 );

    LL_SYSTICK_SetClkSource( LL_SYSTICK_CLKSOURCE_HCLK );
    LL_SetSystemCoreClock( 32000000 );
    LL_RCC_SetUSARTClockSource( LL_RCC_USART2_CLKSOURCE_PCLK1 );

    LL_RCC_SetLPTIMClockSource( LL_RCC_LPTIM1_CLKSOURCE_LSE );
    LL_RCC_SetRNGClockSource( LL_RCC_RNG_CLKSOURCE_PLL );
}

static void mcu_gpio_init( void )
{
#if( HW_DEBUG_PROBE == 1 )
    // Enable debug in sleep/stop/standby
    LL_DBGMCU_EnableDBGSleepMode( );
    LL_DBGMCU_EnableDBGStopMode( );
    LL_DBGMCU_EnableDBGStandbyMode( );
#endif

    hal_gpio_init_out( RADIO_NSS, 1 );
    hal_gpio_init_in( RADIO_BUSY_PIN, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_OFF, NULL );
    // Here init only the pin as an exti rising and the callback will be attached later
    hal_gpio_init_in( RADIO_DIOX, BSP_GPIO_PULL_MODE_DOWN, BSP_GPIO_IRQ_MODE_RISING, NULL );
    hal_gpio_init_out( RADIO_NRST, 1 );
#if defined( SX128X )
    hal_gpio_init_out( RADIO_ANTENNA_SWITCH, 1 );
#elif defined( LR11XX_TRANSCEIVER ) && defined( ENABLE_MODEM_GNSS_FEATURE )
    hal_gpio_init_out( RADIO_LNA_CTRL, 0 );
#elif defined( SX126X )
    // If the sx126x drives the rf switch with dio2, just put the SX126X_RADIO_RF_SWITCH_CTRL in pull up
    hal_gpio_init_in( SX126X_RADIO_RF_SWITCH_CTRL, BSP_GPIO_PULL_MODE_UP, BSP_GPIO_IRQ_MODE_OFF, NULL );
#endif
}

/**
 * @brief Enters Low Power Stop Mode
 *
 * @note ARM exits the function when waking up
 *
 */
static void lpm_enter_sleep_mode( void )
{
#if( LOW_POWER_MODE == 1 )

    lpm_mcu_deinit( );

    // Clear wake up flag
    SET_BIT( PWR->CR, PWR_CR_CWUF );

    // Enable ultra low power mode
    LL_PWR_EnableUltraLowPower( );

    // Set the regulator to low power before setting MODE_STOP.
    LL_PWR_SetRegulModeLP( LL_PWR_REGU_LPMODES_LOW_POWER );

    // Set STOP mode when CPU enters deepsleep
    LL_PWR_SetPowerMode( LL_PWR_MODE_STOP );

    // Set SLEEPDEEP bit of Cortex System Control Register */
    LL_LPM_EnableDeepSleep( );
#endif
    // Request Wait For Interrupt */
    __WFI( );
}

/**
 * @brief Exists Low Power Stop Mode
 *
 */
static void lpm_exit_sleep_mode( void )
{
#if( LOW_POWER_MODE == 1 )
    // Initializes the peripherals
    lpm_mcu_reinit( );
#endif
}

/**
 * @brief Sleep handler
 *
 */
static void sleep_handler( void )
{
    // first stop systick to avoid getting pending irq while going in stop mode
    LL_SYSTICK_DisableIT( );

    /*!
     * If an interrupt has occurred after entering the critical section, it is kept pending
     * and cortex will not enter low power anyway
     */

    lpm_enter_sleep_mode( );
    lpm_exit_sleep_mode( );

    // re start systick
    LL_SYSTICK_EnableIT( );
}

#if( LOW_POWER_MODE == 1 )

/**
 * @brief De-init periph begore going in sleep mode
 *
 */
static void lpm_mcu_deinit( void )
{
    hal_spi_de_init( RADIO_SPI_ID );

#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
    trace_uart_deinit( );
#endif
}

/**
 * @brief Re-init MCU clock after a wait in stop mode 2
 *
 */
static void lpm_mcu_reinit( void )
{
    // Reconfig needed OSC and PLL
    system_clock_re_config_after_stop( );

    // Initialize UART
#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
    trace_uart_init( );
#endif

    // Initialize SPI
    hal_spi_init( RADIO_SPI_ID, RADIO_SPI_MOSI, RADIO_SPI_MISO, RADIO_SPI_SCLK );
}

static void system_clock_re_config_after_stop( void )
{
    LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_PWR );
    LL_PWR_SetRegulVoltageScaling( LL_PWR_REGU_VOLTAGE_SCALE1 );

    // Enable HSI
    LL_RCC_HSI_Enable( );

    // Wait till HSI is ready
    while( LL_RCC_HSI_IsReady( ) != 1 )
    {
    }

    // Enable PLL
    LL_RCC_PLL_Enable( );

    // Wait till PLL is ready
    while( LL_RCC_PLL_IsReady( ) != 1 )
    {
    }

    // Select PLL as system clock source
    LL_RCC_SetSysClkSource( LL_RCC_SYS_CLKSOURCE_PLL );

    // Wait till PLL is used as system clock source
    while( LL_RCC_GetSysClkSource( ) != LL_RCC_SYS_CLKSOURCE_STATUS_PLL )
    {
    }
}

#endif  // LOW_POWER_MODE==1

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler( void )
{
    SMTC_HAL_TRACE_ERROR( "\x1B[0;31m" );  // red color
    SMTC_HAL_TRACE_ERROR( "HARDFAULT_Handler\n" );
    SMTC_HAL_TRACE_ERROR( "\x1B[0m" );  // default color
    while( 1 )
    {
    }
}

/* --- EOF ------------------------------------------------------------------ */
