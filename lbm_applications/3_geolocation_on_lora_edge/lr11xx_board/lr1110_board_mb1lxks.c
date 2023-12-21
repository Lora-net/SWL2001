/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

/*
 * LR11xx radio dependencies
 */
#include "lr11xx_system_types.h"
#include "lr11xx_gnss_types.h"

/*
 * LR11xx shield dependencies
 */
#include "modem_pinout.h"

/*
 * Host MCU dependencies
 */
#include "smtc_hal_gpio.h"
#include "smtc_hal_mcu.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum
{
    LR11XX_EVK_LED_TX,
    LR11XX_EVK_LED_RX,
    LR11XX_EVK_LED_COUNT
} lr11xx_evk_led_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * @brief LED TX MASK
 */
#define LED_TX_MASK ( 1 << LR11XX_EVK_LED_TX )

/*!
 * @brief LED RX MASK
 */
#define LED_RX_MASK ( 1 << LR11XX_EVK_LED_RX )

/*!
 * @brief LED ALL MASK
 */
#define LED_ALL_MASK ( LED_TX_MASK | LED_RX_MASK )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void leds_on( uint8_t leds )
{
    if( leds & LED_TX_MASK )
    {
        /* LED TX */
        hal_gpio_set_value( SMTC_LED_TX, 1 );
    }
    if( leds & LED_RX_MASK )
    {
        /* LED RX */
        hal_gpio_set_value( SMTC_LED_RX, 1 );
    }
}

void leds_off( uint8_t leds )
{
    if( leds & LED_TX_MASK )
    {
        /* LED TX */
        hal_gpio_set_value( SMTC_LED_TX, 0 );
    }
    if( leds & LED_RX_MASK )
    {
        /* LED RX */
        hal_gpio_set_value( SMTC_LED_RX, 0 );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

uint32_t smtc_board_get_led_tx_mask( void )
{
    return LED_TX_MASK;
}

uint32_t smtc_board_get_led_rx_mask( void )
{
    return LED_RX_MASK;
}

uint32_t smtc_board_get_led_all_mask( void )
{
    return LED_ALL_MASK;
}

void smtc_board_led_set( uint32_t led_mask, bool turn_on )
{
    if( turn_on )
    {
        leds_on( led_mask );
    }
    else
    {
        leds_off( led_mask );
    }
}

void smtc_board_led_pulse( uint32_t led_mask, bool turn_on, uint32_t duration_ms )
{
    if( turn_on )
    {
        leds_on( led_mask );
        hal_mcu_wait_us( duration_ms * 1000 );
        leds_off( led_mask );
    }
    else
    {
        leds_off( led_mask );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- GEOLOCATION MIDDLEWARE BSP FUNCTIONS DEFINITION -------------------------
 */

void geolocation_bsp_gnss_prescan_actions( void )
{
    hal_gpio_set_value( SMTC_LED_SCAN, 1 );
    hal_gpio_set_value( RADIO_LNA_CTRL, 1 ); /* LNA on */
}

void geolocation_bsp_gnss_postscan_actions( void )
{
    hal_gpio_set_value( SMTC_LED_SCAN, 0 );
    hal_gpio_set_value( RADIO_LNA_CTRL, 0 ); /* LNA off */
}

void geolocation_bsp_wifi_prescan_actions( void )
{
    hal_gpio_set_value( SMTC_LED_SCAN, 1 );
}

void geolocation_bsp_wifi_postscan_actions( void )
{
    hal_gpio_set_value( SMTC_LED_SCAN, 0 );
}

lr11xx_system_lfclk_cfg_t geolocation_bsp_get_lr11xx_lf_clock_cfg( void )
{
    return LR11XX_SYSTEM_LFCLK_XTAL;
}

void geolocation_bsp_get_lr11xx_reg_mode( const void* context, lr11xx_system_reg_mode_t* reg_mode )
{
    *reg_mode = LR11XX_SYSTEM_REG_MODE_DCDC;
}

void geolocation_bsp_gnss_get_consumption(
    lr11xx_gnss_instantaneous_power_consumption_ua_t* instantaneous_power_consumption_ua )
{
    /* These value are for EVK board in DC DC mode with Xtal 32KHz and a TCXO 32MHz*/
    instantaneous_power_consumption_ua->board_voltage_mv              = 3300;
    instantaneous_power_consumption_ua->init_ua                       = 3150;
    instantaneous_power_consumption_ua->phase1_gps_capture_ua         = 11900;
    instantaneous_power_consumption_ua->phase1_gps_process_ua         = 3340;
    instantaneous_power_consumption_ua->multiscan_gps_capture_ua      = 10700;
    instantaneous_power_consumption_ua->multiscan_gps_process_ua      = 4180;
    instantaneous_power_consumption_ua->phase1_beidou_capture_ua      = 13500;
    instantaneous_power_consumption_ua->phase1_beidou_process_ua      = 3190;
    instantaneous_power_consumption_ua->multiscan_beidou_capture_ua   = 12600;
    instantaneous_power_consumption_ua->multiscan_beidou_process_ua   = 3430;
    instantaneous_power_consumption_ua->sleep_32k_ua                  = 1210;
    instantaneous_power_consumption_ua->demod_sleep_32m_ua            = 2530;
}

/* --- EOF ------------------------------------------------------------------ */