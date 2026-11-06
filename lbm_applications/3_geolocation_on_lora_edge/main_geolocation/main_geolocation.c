/*!
 * \file      main_geolocation.c
 *
 * \brief     main program for GNSS & Wi-Fi geolocation example
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

#include "main.h"

#include "smtc_modem_api.h"
#include "smtc_modem_geolocation_api.h"
#include "smtc_modem_utilities.h"

#include "smtc_modem_hal.h"
#include "smtc_hal_dbg_trace.h"

#include "example_options.h"

#include "smtc_hal_mcu.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_watchdog.h"

#include "lr11xx_system.h"

#include "lr1110_board.h"

#include "modem_pinout.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Returns the minimum value between a and b
 *
 * @param [in] a 1st value
 * @param [in] b 2nd value
 * @retval Minimum value
 */
#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * Stack id value (multistacks modem is not yet available)
 */
#define STACK_ID 0

/**
 * @brief Stack credentials
 */
#if !defined( USE_LR11XX_CREDENTIALS )
static const uint8_t user_dev_eui[8]  = USER_LORAWAN_DEVICE_EUI;
static const uint8_t user_join_eui[8] = USER_LORAWAN_JOIN_EUI;
static const uint8_t user_app_key[16] = USER_LORAWAN_APP_KEY;
#endif

/**
 * @brief Watchdog counter reload value during sleep (The period must be lower than MCU watchdog period (here 32s))
 */
#define WATCHDOG_RELOAD_PERIOD_MS ( 20000 )

#define CUSTOM_NB_TRANS ( 3 )
#define ADR_CUSTOM_LIST { 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3 }

static const uint8_t adr_custom_list[16] = ADR_CUSTOM_LIST;
static const uint8_t custom_nb_trans     = CUSTOM_NB_TRANS;

#define KEEP_ALIVE_PORT ( 2 )
#define KEEP_ALIVE_PERIOD_S ( 3600 / 2 )
#define KEEP_ALIVE_SIZE ( 4 )
uint8_t keep_alive_payload[KEEP_ALIVE_SIZE] = { 0x00 };

/*!
 * @brief Defines the delay before starting the next scan sequence, value in [s].
 */
#define GEOLOCATION_GNSS_SCAN_PERIOD_S ( 5 * 60 )
#define GEOLOCATION_WIFI_SCAN_PERIOD_S ( 3 * 60 )

/*!
 * @brief Time during which a LED is turned on when pulse, in [ms]
 */
#define LED_PERIOD_MS ( 250 )

/**
 * @brief Supported LR11XX radio firmware
 */
#define LR1110_FW_VERSION 0x0401
#define LR1120_FW_VERSION 0x0201

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum almanac_demod_service_state
{
    ADS_STATE_INIT,
    ADS_STATE_STARTED,
    ADS_STATE_STOPPED
} ads_state_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static uint8_t                  rx_payload[SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH] = { 0 };  // Buffer for rx payload
static uint8_t                  rx_payload_size = 0;      // Size of the payload in the rx_payload buffer
static smtc_modem_dl_metadata_t rx_metadata     = { 0 };  // Metadata of downlink
static uint8_t                  rx_remaining    = 0;      // Remaining downlink payload in modem

#if defined( USE_LR11XX_CREDENTIALS )
static uint8_t chip_eui[SMTC_MODEM_EUI_LENGTH] = { 0 };
static uint8_t chip_pin[SMTC_MODEM_PIN_LENGTH] = { 0 };
#endif

static volatile bool user_button_is_press = false;  // Flag for button status

static ads_state_t ads_state = ADS_STATE_INIT;  // State of the almanac demodulation service

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief User callback for modem event
 *
 *  This callback is called every time an event ( see smtc_modem_event_t ) appears in the modem.
 *  Several events may have to be read from the modem when this callback is called.
 */
static void modem_event_callback( void );

/**
 * @brief User callback for button EXTI
 *
 * @param context Define by the user at the init
 */
static void user_button_callback( void* context );

/**
 * @brief Read the LR11xx firmware version to ensure it is compatible with the almanac update
 */
static bool check_lr11xx_fw_version( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Example to send a user payload on an external event
 *
 */
void main_geolocation( void )
{
    uint32_t sleep_time_ms = 0;

    // Disable IRQ to avoid unwanted behaviour during init
    hal_mcu_disable_irq( );

    // Configure all the µC periph (clock, gpio, timer, ...)
    hal_mcu_init( );

    SMTC_HAL_TRACE_INFO( "GEOLOCATION example is starting\n" );

    // Init the modem and use modem_event_callback as event callback, please note that the callback will be
    // called immediately after the first call to smtc_modem_run_engine because of the reset detection
    smtc_modem_init( &modem_event_callback );

    // Configure Nucleo blue button as EXTI
    hal_gpio_irq_t nucleo_blue_button = {
        .pin      = EXTI_BUTTON,
        .context  = NULL,                  // context pass to the callback - not used in this example
        .callback = user_button_callback,  // callback called when EXTI is triggered
    };
    hal_gpio_init_in( EXTI_BUTTON, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_FALLING, &nucleo_blue_button );

    // Re-enable IRQ
    hal_mcu_enable_irq( );

    while( 1 )
    {
        // Check button
        if( user_button_is_press == true )
        {
            user_button_is_press = false;
            if( ads_state == ADS_STATE_STARTED )
            {
                ads_state = ADS_STATE_STOPPED;
                smtc_modem_almanac_demodulation_stop( STACK_ID );
            }
            else
            {
                ads_state = ADS_STATE_STARTED;
                smtc_modem_almanac_demodulation_start( STACK_ID );
            }
        }

        // Modem process launch
        sleep_time_ms = smtc_modem_run_engine( );

        // Atomically check sleep conditions
        hal_mcu_disable_irq( );
        if( ( user_button_is_press == false ) && ( smtc_modem_is_irq_flag_pending( ) == false ) )
        {
            hal_watchdog_reload( );
            hal_mcu_set_sleep_for_ms( MIN( sleep_time_ms, WATCHDOG_RELOAD_PERIOD_MS ) );
        }
        hal_watchdog_reload( );
        hal_mcu_enable_irq( );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/**
 * @brief User callback for modem event
 *
 *  This callback is called every time an event ( see smtc_modem_event_t ) appears in the modem.
 *  Several events may have to be read from the modem when this callback is called.
 */
static void modem_event_callback( void )
{
    smtc_modem_event_t                                          current_event;
    uint8_t                                                     event_pending_count  = 0;
    uint8_t                                                     stack_id             = STACK_ID;
    smtc_modem_gnss_event_data_scan_done_t                      gnss_scan_done_data  = { 0 };
    smtc_modem_gnss_event_data_terminated_t                     gnss_terminated_data = { 0 };
    smtc_modem_almanac_demodulation_event_data_almanac_update_t almanac_update_data  = { 0 };
    smtc_modem_wifi_event_data_scan_done_t                      wifi_scan_done_data  = { 0 };
    smtc_modem_wifi_event_data_terminated_t                     wifi_terminated_data = { 0 };

    // Continue to read modem event until all event has been processed
    do
    {
        // Read modem event
        smtc_modem_get_event( &current_event, &event_pending_count );

        switch( current_event.event_type )
        {
        case SMTC_MODEM_EVENT_RESET:
            SMTC_HAL_TRACE_INFO( "Event received: RESET\n" );
            if( check_lr11xx_fw_version( ) != true )
            {
                SMTC_HAL_TRACE_ERROR( "LR11xx firmware version is not compatible with this example\n" );
                break;
            }

#if !defined( USE_LR11XX_CREDENTIALS )
            /* Set user credentials */
            smtc_modem_set_deveui( stack_id, user_dev_eui );
            smtc_modem_set_joineui( stack_id, user_join_eui );
            smtc_modem_set_nwkkey( stack_id, user_app_key );
#else
            // Get internal credentials
            smtc_modem_get_chip_eui( stack_id, chip_eui );
            SMTC_HAL_TRACE_ARRAY( "CHIP_EUI", chip_eui, SMTC_MODEM_EUI_LENGTH );
            smtc_modem_get_pin( stack_id, chip_pin );
            SMTC_HAL_TRACE_ARRAY( "CHIP_PIN", chip_pin, SMTC_MODEM_PIN_LENGTH );
#endif
            /* Set user region */
            smtc_modem_set_region( stack_id, MODEM_EXAMPLE_REGION );
            /* Schedule a Join LoRaWAN network */
            smtc_modem_join_network( stack_id );
            /* Configure almanac demodulation service */
            smtc_modem_almanac_demodulation_set_constellations( stack_id, SMTC_MODEM_GNSS_CONSTELLATION_GPS_BEIDOU );
            /* Set GNSS and Wi-Fi send mode */
            smtc_modem_store_and_forward_flash_clear_data( stack_id );
            smtc_modem_store_and_forward_set_state( stack_id, SMTC_MODEM_STORE_AND_FORWARD_ENABLE );
            smtc_modem_gnss_send_mode( stack_id, SMTC_MODEM_SEND_MODE_STORE_AND_FORWARD );
            smtc_modem_wifi_send_mode( stack_id, SMTC_MODEM_SEND_MODE_UPLINK );
            /* Program Wi-Fi scan */
            smtc_modem_wifi_set_scan_mode( stack_id, SMTC_MODEM_WIFI_SCAN_MODE_MAC );
            smtc_modem_wifi_scan( stack_id, 0 );
            /* Program GNSS scan */
            smtc_modem_gnss_set_constellations( stack_id, SMTC_MODEM_GNSS_CONSTELLATION_GPS_BEIDOU );
            smtc_modem_gnss_scan( stack_id, SMTC_MODEM_GNSS_MODE_MOBILE, 0 );
            /* Notify user with leds */
            smtc_board_led_set( smtc_board_get_led_tx_mask( ), true );
            break;

        case SMTC_MODEM_EVENT_ALARM:
            SMTC_HAL_TRACE_INFO( "Event received: ALARM\n" );
            smtc_modem_request_uplink( stack_id, KEEP_ALIVE_PORT, false, keep_alive_payload, KEEP_ALIVE_SIZE );
            smtc_modem_alarm_start_timer( KEEP_ALIVE_PERIOD_S );
            break;

        case SMTC_MODEM_EVENT_JOINED:
            SMTC_HAL_TRACE_INFO( "Event received: JOINED\n" );
            /* Notify user with leds */
            smtc_board_led_set( smtc_board_get_led_tx_mask( ), false );
            smtc_board_led_pulse( smtc_board_get_led_rx_mask( ), true, LED_PERIOD_MS );
            /* Set custom ADR profile for geolocation */
            smtc_modem_adr_set_profile( stack_id, SMTC_MODEM_ADR_PROFILE_CUSTOM, adr_custom_list );
            smtc_modem_set_nb_trans( stack_id, custom_nb_trans );
            /* Start time for regular uplink */
            smtc_modem_alarm_start_timer( KEEP_ALIVE_PERIOD_S );
            break;

        case SMTC_MODEM_EVENT_TXDONE:
            SMTC_HAL_TRACE_INFO( "Event received: TXDONE (%d)\n", current_event.event_data.txdone.status );
            SMTC_HAL_TRACE_INFO( "Transmission done\n" );
            break;

        case SMTC_MODEM_EVENT_DOWNDATA:
            SMTC_HAL_TRACE_INFO( "Event received: DOWNDATA\n" );
            // Get downlink data
            smtc_modem_get_downlink_data( rx_payload, &rx_payload_size, &rx_metadata, &rx_remaining );
            SMTC_HAL_TRACE_PRINTF( "Data received on port %u\n", rx_metadata.fport );
            SMTC_HAL_TRACE_ARRAY( "Received payload", rx_payload, rx_payload_size );
            break;

        case SMTC_MODEM_EVENT_JOINFAIL:
            SMTC_HAL_TRACE_INFO( "Event received: JOINFAIL\n" );
            SMTC_HAL_TRACE_WARNING( "Join request failed \n" );
            break;

        case SMTC_MODEM_EVENT_ALCSYNC_TIME:
            SMTC_HAL_TRACE_INFO( "Event received: TIME\n" );
            break;

        case SMTC_MODEM_EVENT_GNSS_SCAN_DONE:
            SMTC_HAL_TRACE_INFO( "Event received: GNSS_SCAN_DONE\n" );
            /* Get event data */
            smtc_modem_gnss_get_event_data_scan_done( stack_id, &gnss_scan_done_data );
            /* Start almanac demodulation service if the radio is synchronized with GPS time */
            if( ( gnss_scan_done_data.time_available == true ) && ( ads_state == ADS_STATE_INIT ) )
            {
                ads_state = ADS_STATE_STARTED;
                smtc_modem_almanac_demodulation_start( stack_id );
            }
            break;

        case SMTC_MODEM_EVENT_GNSS_TERMINATED:
            SMTC_HAL_TRACE_INFO( "Event received: GNSS_TERMINATED\n" );
            /* Notify user with leds */
            smtc_board_led_pulse( smtc_board_get_led_tx_mask( ), true, LED_PERIOD_MS );
            /* Get event data */
            smtc_modem_gnss_get_event_data_terminated( stack_id, &gnss_terminated_data );
            /* launch next scan */
            smtc_modem_gnss_scan( stack_id, SMTC_MODEM_GNSS_MODE_MOBILE, GEOLOCATION_GNSS_SCAN_PERIOD_S );
            break;

        case SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE:
            SMTC_HAL_TRACE_INFO( "Event received: GNSS_ALMANAC_DEMOD_UPDATE\n" );
            smtc_modem_almanac_demodulation_get_event_data_almanac_update( stack_id, &almanac_update_data );
            /* Store progress in keep alive payload */
            keep_alive_payload[0] = almanac_update_data.update_progress_gps;
            keep_alive_payload[1] = almanac_update_data.update_progress_beidou;
            break;

        case SMTC_MODEM_EVENT_WIFI_SCAN_DONE:
            SMTC_HAL_TRACE_INFO( "Event received: WIFI_SCAN_DONE\n" );
            /* Get event data */
            smtc_modem_wifi_get_event_data_scan_done( stack_id, &wifi_scan_done_data );
            break;

        case SMTC_MODEM_EVENT_WIFI_TERMINATED:
            SMTC_HAL_TRACE_INFO( "Event received: WIFI_TERMINATED\n" );
            /* Notify user with leds */
            smtc_board_led_pulse( smtc_board_get_led_tx_mask( ), true, LED_PERIOD_MS );
            /* Get event data */
            smtc_modem_wifi_get_event_data_terminated( stack_id, &wifi_terminated_data );
            /* launch next scan */
            smtc_modem_wifi_scan( stack_id, GEOLOCATION_WIFI_SCAN_PERIOD_S );
            break;

        case SMTC_MODEM_EVENT_LINK_CHECK:
            SMTC_HAL_TRACE_INFO( "Event received: LINK_CHECK\n" );
            break;

        case SMTC_MODEM_EVENT_CLASS_B_STATUS:
            SMTC_HAL_TRACE_INFO( "Event received: CLASS_B_STATUS\n" );
            break;

        case SMTC_MODEM_EVENT_REGIONAL_DUTY_CYCLE:
            SMTC_HAL_TRACE_INFO( "Event received: SMTC_MODEM_EVENT_REGIONAL_DUTY_CYCLE\n" );
            break;

        default:
            SMTC_HAL_TRACE_ERROR( "Unknown event %u\n", current_event.event_type );
            break;
        }
    } while( event_pending_count > 0 );
}

static void user_button_callback( void* context )
{
    SMTC_HAL_TRACE_INFO( "Button pushed\n" );

    ( void ) context;  // Not used in the example - avoid warning

    static uint32_t last_press_timestamp_ms = 0;

    // Debounce the button press, avoid multiple triggers
    if( ( int32_t ) ( smtc_modem_hal_get_time_in_ms( ) - last_press_timestamp_ms ) > 500 )
    {
        last_press_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
        user_button_is_press    = true;
    }
}

static bool check_lr11xx_fw_version( void )
{
    lr11xx_status_t         status;
    lr11xx_system_version_t lr11xx_fw_version;

    /* suspend modem to get access to the radio */
    smtc_modem_suspend_radio_communications( true );

    status = lr11xx_system_get_version( NULL, &lr11xx_fw_version );
    if( status != LR11XX_STATUS_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to get LR11XX firmware version\n" );
        smtc_modem_suspend_radio_communications( false );
        return false;
    }

    if( ( lr11xx_fw_version.type == LR11XX_SYSTEM_VERSION_TYPE_LR1110 ) &&
        ( lr11xx_fw_version.fw < LR1110_FW_VERSION ) )
    {
        SMTC_HAL_TRACE_ERROR( "Wrong LR1110 firmware version, expected 0x%04X, got 0x%04X\n", LR1110_FW_VERSION,
                              lr11xx_fw_version.fw );
        smtc_modem_suspend_radio_communications( false );
        return false;
    }
    if( ( lr11xx_fw_version.type == LR11XX_SYSTEM_VERSION_TYPE_LR1120 ) &&
        ( lr11xx_fw_version.fw < LR1120_FW_VERSION ) )
    {
        SMTC_HAL_TRACE_ERROR( "Wrong LR1120 firmware version, expected 0x%04X, got 0x%04X\n", LR1120_FW_VERSION,
                              lr11xx_fw_version.fw );
        smtc_modem_suspend_radio_communications( false );
        return false;
    }

    /* release radio to the modem */
    smtc_modem_suspend_radio_communications( false );
    SMTC_HAL_TRACE_INFO( "LR11XX FW: 0x%04X, type: 0x%02X\n", lr11xx_fw_version.fw, lr11xx_fw_version.type );
    return true;
}

/* --- EOF ------------------------------------------------------------------ */
