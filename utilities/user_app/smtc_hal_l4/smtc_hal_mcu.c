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

#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_utils.h"

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
#define HW_DEBUG_PROBE 0

/*!
 * Watchdog counter reload value during sleep
 *
 * \remark The period must be lower than MCU watchdog period
 */
#define WATCHDOG_RELOAD_PERIOD_SECONDS 20
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */
// Low Power options
typedef enum low_power_mode_e
{
    LOW_POWER_ENABLE,
    LOW_POWER_DISABLE,
    LOW_POWER_DISABLE_ONCE
} low_power_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static volatile bool             exit_wait       = false;
static volatile low_power_mode_t lp_current_mode = LOW_POWER_ENABLE;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static void system_clock_config( void );
static void mcu_gpio_init( void );

#if( LOW_POWER_MODE == 1 )
static void lpm_mcu_deinit( void );
static void lpm_mcu_reinit( void );
static void lpm_enter_stop_mode( void );
static void lpm_exit_stop_mode( void );
static void lpm_handler( void );
#else
static bool no_low_power_wait( const int32_t milliseconds );
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
    HAL_Init( );             // Initialize MCU HAL library
    system_clock_config( );  // Initialize clocks
    mcu_gpio_init( );        // Initialize GPIOs

#if defined( HW_MODEM_ENABLED )
    uart4_init( );
#endif
#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
    uart2_init( );
#endif

    hal_lp_timer_init( );
    hal_spi_init( RADIO_SPI_ID, RADIO_SPI_MOSI, RADIO_SPI_MISO, RADIO_SPI_SCLK );
    hal_rtc_init( );

    // Initialize watchdog
    hal_watchdog_init( );
}

void hal_mcu_reset( void )
{
    __disable_irq( );
    NVIC_SystemReset( );  // Restart system
}

void __attribute__( ( optimize( "O0" ) ) ) hal_mcu_wait_us( const int32_t microseconds )
{
    // Work @80MHz
    const uint32_t nb_nop = microseconds * 1000 / 137;
    for( uint32_t i = 0; i < nb_nop; i++ )
    {
        __NOP( );
    }
}

void hal_mcu_set_sleep_for_ms( const int32_t milliseconds )
{
    bool last_sleep_loop = false;

    if( milliseconds <= 0 )
    {
        return;
    }

    if( lp_current_mode == LOW_POWER_DISABLE_ONCE )
    {
        lp_current_mode = LOW_POWER_ENABLE;
        return;
    }
    int32_t time_counter = milliseconds;

    watchdog_reload( );

#if( LOW_POWER_MODE == 1 )
    if( lp_current_mode == LOW_POWER_ENABLE )
    {
        do
        {
            if( ( time_counter > ( WATCHDOG_RELOAD_PERIOD_SECONDS * 1000 ) ) )
            {
                time_counter -= WATCHDOG_RELOAD_PERIOD_SECONDS * 1000;
                hal_rtc_wakeup_timer_set_ms( WATCHDOG_RELOAD_PERIOD_SECONDS * 1000 );
            }
            else
            {
                hal_rtc_wakeup_timer_set_ms( time_counter );
                // if the sleep time is less than the wdog reload period, this is the last sleep loop
                last_sleep_loop = true;
            }
            lpm_handler( );
            watchdog_reload( );
        } while( ( hal_rtc_has_wut_irq_happened( ) == true ) && ( last_sleep_loop == false ) );
        if( last_sleep_loop == false )
        {
            // in case sleep mode is interrupted by an other irq than the wake up timer, stop it and exit
            hal_rtc_wakeup_timer_stop( );
        }
    }
#else
    while( ( time_counter > ( WATCHDOG_RELOAD_PERIOD_SECONDS * 1000 ) ) && ( lp_current_mode == LOW_POWER_ENABLE ) )
    {
        time_counter -= WATCHDOG_RELOAD_PERIOD_SECONDS * 1000;
        if( ( no_low_power_wait( WATCHDOG_RELOAD_PERIOD_SECONDS * 1000 ) == true ) ||
            ( lp_current_mode != LOW_POWER_ENABLE ) )
        {
            // wait function was interrupted, inturrupt here also
            watchdog_reload( );
            return;
        }
        watchdog_reload( );
    }
    if( lp_current_mode == LOW_POWER_ENABLE )
    {
        no_low_power_wait( time_counter );
        watchdog_reload( );
    }
#endif
}

void hal_mcu_disable_low_power_wait( void )
{
    exit_wait       = true;
    lp_current_mode = LOW_POWER_DISABLE;
}

void hal_mcu_enable_low_power_wait( void )
{
    exit_wait       = false;
    lp_current_mode = LOW_POWER_ENABLE;
}

void hal_mcu_disable_once_low_power_wait( void )
{
    exit_wait       = true;
    lp_current_mode = LOW_POWER_DISABLE_ONCE;
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
    HAL_IncTick( );
    HAL_SYSTICK_IRQHandler( );
}

void HAL_MspInit( void )
{
    // System interrupt init
    HAL_NVIC_SetPriority( MemoryManagement_IRQn, 0, 0 );
    HAL_NVIC_SetPriority( BusFault_IRQn, 0, 0 );
    HAL_NVIC_SetPriority( UsageFault_IRQn, 0, 0 );
    HAL_NVIC_SetPriority( SVCall_IRQn, 0, 0 );
    HAL_NVIC_SetPriority( DebugMonitor_IRQn, 0, 0 );
    HAL_NVIC_SetPriority( PendSV_IRQn, 0, 0 );
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void system_clock_config( void )
{
    // Configure the main internal regulator output voltage
    RCC_OscInitTypeDef       rcc_osc_init_struct = { 0 };
    RCC_ClkInitTypeDef       rcc_clk_init_struct = { 0 };
    RCC_PeriphCLKInitTypeDef periph_clk_init     = { 0 };

    // Set low drive on LSE to reduce conso
    HAL_PWR_EnableBkUpAccess( );
    __HAL_RCC_LSEDRIVE_CONFIG( RCC_LSEDRIVE_LOW );

    // HSI is enabled after System reset, activate PLL with HSI as source
    rcc_osc_init_struct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
    rcc_osc_init_struct.LSEState            = RCC_LSE_ON;
    rcc_osc_init_struct.HSIState            = RCC_HSI_ON;
    rcc_osc_init_struct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    rcc_osc_init_struct.LSIState            = RCC_LSI_ON;
    rcc_osc_init_struct.PLL.PLLState        = RCC_PLL_ON;
    rcc_osc_init_struct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    rcc_osc_init_struct.PLL.PLLM            = 1;
    rcc_osc_init_struct.PLL.PLLN            = 10;
    rcc_osc_init_struct.PLL.PLLP            = RCC_PLLP_DIV7;
    rcc_osc_init_struct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    rcc_osc_init_struct.PLL.PLLR            = RCC_PLLR_DIV2;

    if( HAL_RCC_OscConfig( &rcc_osc_init_struct ) != HAL_OK )
    {
        mcu_panic( );  // Initialization Error
    }

    // Initializes the CPU, AHB and APB busses clocks
    rcc_clk_init_struct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    rcc_clk_init_struct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clk_init_struct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    rcc_clk_init_struct.APB1CLKDivider = RCC_HCLK_DIV1;
    rcc_clk_init_struct.APB2CLKDivider = RCC_HCLK_DIV1;
    if( HAL_RCC_ClockConfig( &rcc_clk_init_struct, FLASH_LATENCY_4 ) != HAL_OK )
    {
        mcu_panic( );  // Initialization Error
    }

    periph_clk_init.PeriphClockSelection =
        RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_LPTIM1 | RCC_PERIPHCLK_RNG | RCC_PERIPHCLK_ADC;
    periph_clk_init.RTCClockSelection       = RCC_RTCCLKSOURCE_LSE;
    periph_clk_init.Lptim1ClockSelection    = RCC_LPTIM1CLKSOURCE_LSE;
    periph_clk_init.AdcClockSelection       = RCC_ADCCLKSOURCE_SYSCLK;
    periph_clk_init.RngClockSelection       = RCC_RNGCLKSOURCE_PLLSAI1;
    periph_clk_init.PLLSAI1.PLLSAI1Source   = RCC_PLLSOURCE_HSI;
    periph_clk_init.PLLSAI1.PLLSAI1M        = 1;
    periph_clk_init.PLLSAI1.PLLSAI1N        = 12;
    periph_clk_init.PLLSAI1.PLLSAI1P        = RCC_PLLP_DIV7;
    periph_clk_init.PLLSAI1.PLLSAI1Q        = RCC_PLLQ_DIV4;
    periph_clk_init.PLLSAI1.PLLSAI1R        = RCC_PLLR_DIV2;
    periph_clk_init.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;

#if( BSP_USE_PRINTF_UART == BSP_FEATURE_ON )
    periph_clk_init.PeriphClockSelection |= RCC_PERIPHCLK_USART2;
    periph_clk_init.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
#endif
#if( BSP_USE_USER_UART == BSP_FEATURE_ON )
    periph_clk_init.PeriphClockSelection |= RCC_PERIPHCLK_UART4;
    periph_clk_init.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
#endif

    if( HAL_RCCEx_PeriphCLKConfig( &periph_clk_init ) != HAL_OK )
    {
        mcu_panic( );  // Initialization Error
    }

    // Enable Power Clock
    __HAL_RCC_PWR_CLK_ENABLE( );

    // Ensure that HSI is wake-up system clock
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG( RCC_STOP_WAKEUPCLOCK_HSI );
}

static void mcu_gpio_init( void )
{
#if( HW_DEBUG_PROBE == 1 )
    // Enable debug in sleep/stop/standby
    HAL_DBGMCU_EnableDBGSleepMode( );
    HAL_DBGMCU_EnableDBGStopMode( );
    HAL_DBGMCU_EnableDBGStandbyMode( );
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

#if( LOW_POWER_MODE == 1 )

/**
 * @brief Enters Low Power Stop Mode
 *
 * @note ARM exits the function when waking up
 *
 */
static void lpm_enter_stop_mode( void )
{
    CRITICAL_SECTION_BEGIN( );

    // Deinit periph & enter Stop Mode
    lpm_mcu_deinit( );
    HAL_PWREx_EnterSTOP2Mode( PWR_STOPENTRY_WFI );

    CRITICAL_SECTION_END( );
}

/**
 * @brief Exists Low Power Stop Mode
 *
 */
static void lpm_exit_stop_mode( void )
{
    // Disable IRQ while the MCU is not running on HSI
    CRITICAL_SECTION_BEGIN( );

    // Initializes the peripherals
    lpm_mcu_reinit( );

    CRITICAL_SECTION_END( );
}

/**
 * @brief Low power handler
 *
 */
static void lpm_handler( void )
{
    // stop systick to avoid getting pending irq while going in stop mode
    // Systick is automatically restart when going out of sleep
    HAL_SuspendTick( );

    __disable_irq( );
    // If an interrupt has occurred after __disable_irq( ), it is kept pending
    // and cortex will not enter low power anyway

    lpm_enter_stop_mode( );
    lpm_exit_stop_mode( );

    __enable_irq( );
    HAL_ResumeTick( );
}

/**
 * @brief De-init periph begore going in sleep mode
 *
 */
static void lpm_mcu_deinit( void )
{
    hal_spi_de_init( RADIO_SPI_ID );

#if defined( HW_MODEM_ENABLED )
    uart4_deinit( );
#endif
#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
    uart2_deinit( );
#endif
}

/**
 * @brief Re-init MCU clock after a wait in stop mode 2
 *
 */
static void lpm_mcu_reinit( void )
{
    RCC_ClkInitTypeDef rcc_clk_init_struct = { 0 };
    RCC_OscInitTypeDef rcc_osc_init_struct = { 0 };
    uint32_t           flash_latency       = 0;

    // Enable Power Control clock
    __HAL_RCC_PWR_CLK_ENABLE( );

    // Get the Oscillators configuration according to the internal RCC registers
    HAL_RCC_GetOscConfig( &rcc_osc_init_struct );

    // Enable PLL
    rcc_osc_init_struct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
    rcc_osc_init_struct.PLL.PLLState   = RCC_PLL_ON;
    if( HAL_RCC_OscConfig( &rcc_osc_init_struct ) != HAL_OK )
    {
        mcu_panic( );
    }

    // Get the Clocks configuration according to the internal RCC registers
    HAL_RCC_GetClockConfig( &rcc_clk_init_struct, &flash_latency );

    // Select PLL as system clock source and keep HCLK, PCLK1 and PCLK2 clocks dividers as before
    rcc_clk_init_struct.ClockType    = RCC_CLOCKTYPE_SYSCLK;
    rcc_clk_init_struct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    if( HAL_RCC_ClockConfig( &rcc_clk_init_struct, flash_latency ) != HAL_OK )
    {
        mcu_panic( );
    }

    // Initialize UART
#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
    uart2_init( );
#endif
#if defined( HW_MODEM_ENABLED )
    uart4_init( );
#endif

    // Initialize SPI
    hal_spi_init( RADIO_SPI_ID, RADIO_SPI_MOSI, RADIO_SPI_MISO, RADIO_SPI_SCLK );
}

#else  // ie LOW_POWER_MODE == 0

/**
 * @brief Fake a wait but doesn't go in sleep mode
 *
 * @param milliseconds number of ms to wait
 * @return true If wait has been interrupt
 * @return false if wait has not been interrupt
 */
static bool no_low_power_wait( const int32_t milliseconds )
{
    uint32_t start_time = smtc_modem_hal_get_time_in_ms( );

    while( smtc_modem_hal_get_time_in_ms( ) < ( start_time + milliseconds ) )
    {
        // interruptible wait for 10ms
        HAL_Delay( 10 );
        if( exit_wait == true )
        {
            // stop wait/lp function and return immediatly
            exit_wait = false;
            return true;
        }
    }
    return false;
}
#endif

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
