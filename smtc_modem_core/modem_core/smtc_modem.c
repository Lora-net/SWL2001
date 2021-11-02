/*!
 * \file      smtc_modem.c
 *
 * \brief     modem implementation (functions of generic api, extension, test and utilities)
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

#include "smtc_modem_api.h"
#include "smtc_basic_modem_lr1110_api_extension.h"
#include "smtc_modem_test_api.h"

#include "smtc_modem_hal_dbg_trace.h"
#include "modem_supervisor.h"
#include "modem_context.h"
#include "smtc_real_defs.h"
#include "lorawan_api.h"
#include "alc_sync.h"
#include "file_upload.h"
#include "stream.h"
#include "radio_planner.h"
#include "ral.h"
#include "smtc_modem_utilities.h"
#include "modem_utilities.h"
#include "smtc_modem_crypto.h"
#include "lora_basics_modem_version.h"
#include "ralf.h"

#if defined( LR1110_MODEM_E )
#include "smtc_modem_e_api_extension.h"
#include "smtc_rtc_compensation.h"
#include "pool_mem.h"
#include "smtc_crypto_se.h"
#if defined( _MODEM_E_WIFI_ENABLE )
#include "wifi_ctrl_api.h"
#endif  // _MODEM_E_WIFI_ENABLE
#if defined( _MODEM_E_GNSS_ENABLE )
#include "gnss_ctrl_api.h"
#endif  //_MODEM_E_GNSS_ENABLE
#endif  // LR1110_MODEM_E

#if defined( LR1110_TRANSCEIVER )
#include "smtc_basic_modem_lr1110_api_extension.h"
#include "smtc_modem_api_lr1110_system.h"
#endif  // LR1110_TRANSCEIVER

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define RETURN_BUSY_IF_TEST_MODE( )                 \
    do                                              \
    {                                               \
        if( true == modem_get_test_mode_status( ) ) \
        {                                           \
            return SMTC_MODEM_RC_BUSY;              \
        }                                           \
    } while( 0 )

#define RETURN_INVALID_IF_NULL( x )       \
    do                                    \
    {                                     \
        if( NULL == ( x ) )               \
        {                                 \
            return SMTC_MODEM_RC_INVALID; \
        }                                 \
    } while( 0 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define MAX_CRYSTAL_ERROR 10

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

#if !defined( LR1110_MODEM_E )
static uint8_t   modem_buffer[242];
static uint32_t* upload_pdata;
static uint32_t  upload_size;

static radio_planner_t modem_radio_planner;

smtc_modem_services_t smtc_modem_services_ctx;

// LBT temporary configuration save
static uint32_t lbt_config_listen_duration_ms = 0;
static int16_t  lbt_config_threshold_dbm      = 0;
static uint32_t lbt_config_bw_hz              = 0;
static bool     lbt_config_available          = false;

// user_radio_access
static rp_status_t user_radio_irq_status;
static uint32_t    user_radio_irq_timestamp;

#ifdef LORAWAN_BYPASS_ENABLED
static bool stream_bypass_enabled = false;
#endif  // LORAWAN_BYPASS_ENABLED

#else  // !defined( LR1110_MODEM_E )

uint8_t modem_buffer[242];
struct
{
    uint32_t* upload_pdata;
    uint32_t  upload_size;
    uint32_t  upload_spare;
#ifdef LORAWAN_BYPASS_ENABLED
    bool      stream_bypass_enabled = false;
#endif  // LORAWAN_BYPASS_ENABLED
} modem_api_context;

// clang-format off
#define upload_pdata                modem_api_context.upload_pdata
#define upload_size                 modem_api_context.upload_size
#ifdef LORAWAN_BYPASS_ENABLED
#define stream_bypass_enabled       modem_api_context.stream_bypass_enabled
#endif  // LORAWAN_BYPASS_ENABLED
// clang-format on

radio_planner_t       modem_radio_planner;
smtc_modem_services_t smtc_modem_services_ctx;

#endif  // !defined( LR1110_MODEM_E )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static bool modem_port_reserved( uint8_t f_port );

static smtc_modem_return_code_t smtc_modem_get_dm_status_with_rate( uint8_t* dm_fields_payload,
                                                                    uint8_t* dm_field_length, e_dm_info_rate_t rate );

static smtc_modem_return_code_t smtc_modem_set_dm_status_with_rate( const uint8_t* dm_fields_payload,
                                                                    uint8_t dm_field_length, e_dm_info_rate_t rate );

static bool is_modem_connected( );

static smtc_modem_return_code_t smtc_modem_send_empty_tx( uint8_t f_port, bool f_port_present, bool confirmed );

static smtc_modem_return_code_t smtc_modem_send_tx( uint8_t f_port, bool confirmed, const uint8_t* payload,
                                                    uint8_t payload_length, bool emergency );

static smtc_modem_event_user_radio_access_status_t convert_rp_to_user_radio_access_status( rp_status_t rp_status );

void empty_callback( void* ctx );
void user_radio_access_callback( void* ctx );
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/* ------------ Modem Utilities ------------*/

void smtc_modem_init( const ralf_t* radio, void ( *callback )( void ) )
{
#ifdef LORAWAN_BYPASS_ENABLED
    stream_bypass_enabled = false;
#endif
    // init radio and put it in sleep mode
    ral_reset( &( radio->ral ) );
    ral_init( &( radio->ral ) );
    ral_set_sleep( &( radio->ral ), true );

#if defined( LR1110_TRANSCEIVER )
    // Save modem radio context in case of direct access to radio by the modem
    modem_context_set_modem_radio_ctx( radio->ral.context );
#endif  // LR1110_TRANSCEIVER

    // init radio planner and attach corresponding radio irq
    rp_init( &modem_radio_planner, radio );

#if !defined( LR1110_MODEM_E )
    smtc_modem_hal_irq_config_radio_irq( rp_radio_irq_callback, &modem_radio_planner );
#endif

    // init modem supervisor

#if defined( LR1110_MODEM_E )
    smtc_rtc_compensation_init( &modem_radio_planner, RTC_CLK_COMPENSATION_ID_RP );
#endif
    rp_hook_init( &modem_radio_planner, RP_HOOK_ID_SUSPEND, ( void ( * )( void* ) )( empty_callback ),
                  &modem_radio_planner );
    rp_hook_init( &modem_radio_planner, RP_HOOK_ID_USER_SUSPEND, ( void ( * )( void* ) )( user_radio_access_callback ),
                  &modem_radio_planner ); /* user_radio_access_callback called when interrupt occurs */
    modem_supervisor_init( callback, &modem_radio_planner, &smtc_modem_services_ctx );
    smtc_secure_element_init( );
}

uint32_t smtc_modem_run_engine( void )
{
    uint8_t nb_downlink = fifo_ctrl_get_nb_elt( lorawan_api_get_fifo_obj( ) );

    if( nb_downlink > 0 )
    {
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_DOWNDATA, 0 );
        set_modem_event_count_and_status( SMTC_MODEM_EVENT_DOWNDATA, nb_downlink, 0 );
    }

    return modem_supervisor_engine( );
}

/* ------------ Modem Generic Api ------------*/

smtc_modem_return_code_t smtc_modem_get_event( smtc_modem_event_t* event, uint8_t* event_pending_count )

{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( event );
    RETURN_INVALID_IF_NULL( event_pending_count );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    const uint8_t            event_count = get_asynchronous_msgnumber( );

    if( event_count > MODEM_NUMBER_OF_EVENTS )
    {
        smtc_modem_hal_mcu_panic( "asynchronous_msgnumber overlap" );
    }
    else if( event_count > 0 )
    {
        event->event_type    = get_last_msg_event( );
        event->missed_events = get_modem_event_count( event->event_type ) - 1;

        SMTC_MODEM_HAL_TRACE_PRINTF( "Event ID: %d, Missed: %d\n", event->event_type, event->missed_events );

        *event_pending_count = event_count - 1;

        switch( event->event_type )
        {
        case SMTC_MODEM_EVENT_RESET:
            event->event_data.reset.count = lorawan_api_nb_reset_get( );
            break;
        case SMTC_MODEM_EVENT_DOWNDATA: {
            lr1mac_down_metadata_t metadata;
            uint8_t                metadata_len;

            fifo_ctrl_get( lorawan_api_get_fifo_obj( ), event->event_data.downdata.data,
                           &( event->event_data.downdata.length ), SMTC_MODEM_MAX_DOWNLINK_LENGTH, &metadata,
                           &metadata_len, sizeof( lr1mac_down_metadata_t ) );

            if( ( metadata.rx_rssi >= -128 ) && ( metadata.rx_rssi <= 63 ) )
            {
                event->event_data.downdata.rssi = ( int8_t )( metadata.rx_rssi + 64 );
            }
            else if( metadata.rx_rssi > 63 )
            {
                event->event_data.downdata.rssi = 127;
            }
            else if( metadata.rx_rssi < -128 )
            {
                event->event_data.downdata.rssi = -128;
            }

            event->event_data.downdata.snr    = metadata.rx_snr << 2;
            event->event_data.downdata.window = ( smtc_modem_event_downdata_window_t ) metadata.rx_window;
            event->event_data.downdata.fport  = metadata.rx_fport;
            break;
        }
        case SMTC_MODEM_EVENT_UPLOADDONE:
            event->event_data.uploaddone.status =
                ( smtc_modem_event_uploaddone_status_t ) get_modem_event_status( event->event_type );
            break;
        case SMTC_MODEM_EVENT_TXDONE:
            event->event_data.txdone.status =
                ( smtc_modem_event_txdone_status_t ) get_modem_event_status( event->event_type );
            break;
        case SMTC_MODEM_EVENT_SETCONF:
            event->event_data.setconf.tag =
                ( smtc_modem_event_setconf_tag_t ) get_modem_event_status( event->event_type );
            break;
        case SMTC_MODEM_EVENT_MUTE:
            event->event_data.mute.status =
                ( get_modem_muted( ) == MODEM_NOT_MUTE ) ? SMTC_MODEM_EVENT_MUTE_OFF : SMTC_MODEM_EVENT_MUTE_ON;
            break;
        case SMTC_MODEM_EVENT_TIME:
            event->event_data.time.status = get_modem_event_status( event->event_type );
            break;
        case SMTC_MODEM_EVENT_LINK_CHECK:
            lorawan_api_get_link_check_ans( &event->event_data.link_check.margin,
                                            &event->event_data.link_check.gw_cnt );
            event->event_data.link_check.status =
                ( smtc_modem_event_link_check_status_t ) get_modem_event_status( event->event_type );
            break;
        case SMTC_MODEM_EVENT_USER_RADIO_ACCESS:
            event->event_data.user_radio_access.timestamp_ms = user_radio_irq_timestamp;
            event->event_data.user_radio_access.status =
                convert_rp_to_user_radio_access_status( user_radio_irq_status );
            break;
        case SMTC_MODEM_EVENT_ALMANAC_UPDATE:
            event->event_data.almanac_update.status =
                ( smtc_modem_event_almanac_update_status_t ) get_modem_event_status( event->event_type );
            break;
#if defined( _MODEM_E_WIFI_ENABLE ) && defined( LR1110_MODEM_E )
        case SMTC_MODEM_EVENT_WIFI: {
            uint16_t size = WifiGetSize( );
            WifiReadResults( );
            //// size = 1000;
            /// for (int i = 0 ; i < 1000; i ++)
            //{
            //  POOL_MEM.WIFI_MEM.Pool_mem.Buffer_tx.EventModem[i]=(i%255);
            //}
            *data_length = size;
            memcpy( data, ( uint8_t* ) ( &( POOL_MEM.WIFI_MEM.Pool_mem.Buffer_tx.EventModem[0] ) + 4 ), *data_length );
        }
        break;
#endif
#if defined( _MODEM_E_GNSS_ENABLE ) && defined( LR1110_MODEM_E )
        case SMTC_MODEM_EVENT_GNSS: {
            *data_length = GnssGetSize( );
            memcpy( data, ( uint8_t* ) &( POOL_MEM.GNSS_MEM.Buf_data[0] ), *data_length );
        }
        break;
#endif
        case SMTC_MODEM_EVENT_ALARM:
        case SMTC_MODEM_EVENT_JOINED:
        case SMTC_MODEM_EVENT_STREAMDONE:
        case SMTC_MODEM_EVENT_JOINFAIL:
        case SMTC_MODEM_EVENT_TIMEOUT_ADR_CHANGED:
        case SMTC_MODEM_EVENT_NEW_LINK_ADR:
        default:
            break;
        }
        // Reset the status after get the value
        set_modem_event_count_and_status( event->event_type, 0, 0 );
        decrement_asynchronous_msgnumber( );
    }
    else
    {
        // No event is available
        event->event_type = SMTC_MODEM_EVENT_NONE;
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_modem_version( smtc_modem_version_t* firmware_version )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( firmware_version );

    firmware_version->major = LORA_BASICS_MODEM_FW_VERSION_MAJOR;
    firmware_version->minor = LORA_BASICS_MODEM_FW_VERSION_MINOR;
    firmware_version->patch = LORA_BASICS_MODEM_FW_VERSION_PATCH;

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_lorawan_version( smtc_modem_lorawan_version_t* lorawan_version )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( lorawan_version );

    lr1mac_version_t lr_version = lorawan_api_get_spec_version( );
    lorawan_version->major      = lr_version.major;
    lorawan_version->minor      = lr_version.minor;
    lorawan_version->patch      = lr_version.patch;
    lorawan_version->revision   = lr_version.revision;

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_regional_params_version( smtc_modem_lorawan_version_t* regional_params_version )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( regional_params_version );

    lr1mac_version_t rp_version = lorawan_api_get_regional_parameters_version( );

    regional_params_version->major    = rp_version.major;
    regional_params_version->minor    = rp_version.minor;
    regional_params_version->patch    = rp_version.patch;
    regional_params_version->revision = rp_version.revision;

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_reset( void )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    set_modem_reset_requested( true );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_factory_reset( void )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    lorawan_api_factory_reset( );
    modem_context_factory_reset( );
    set_modem_reset_requested( true );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_charge( uint32_t* charge_mah )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( charge_mah );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    *charge_mah                          = get_modem_charge_ma_h( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_reset_charge( void )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    reset_modem_charge( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_tx_power_offset_db( uint8_t stack_id, int8_t* tx_pwr_offset_db )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( tx_pwr_offset_db );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    *tx_pwr_offset_db                    = modem_context_get_tx_power_offset_db( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_tx_power_offset_db( uint8_t stack_id, int8_t tx_pwr_offset_db )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    if( ( tx_pwr_offset_db < -30 ) || ( tx_pwr_offset_db > 30 ) )
    {
        return SMTC_MODEM_RC_INVALID;
    }
    modem_context_set_tx_power_offset_db( tx_pwr_offset_db );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_time_start_sync_service( uint8_t                        stack_id,
                                                             smtc_modem_time_sync_service_t sync_service )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( sync_service > SMTC_MODEM_TIME_ALC_SYNC )
    {
        return SMTC_MODEM_RC_INVALID;
    }

    if( clock_sync_is_enabled( &( smtc_modem_services_ctx.clock_sync_ctx ) ) == false )
    {
        clock_sync_set_enabled( &( smtc_modem_services_ctx.clock_sync_ctx ), true,
                                ( clock_sync_service_t ) sync_service );

        // Activate only if modem is Joined
        if( get_join_state( ) == MODEM_JOINED )
        {
            // Activate only if not running
            if( modem_supervisor_is_clock_sync_running( ) == false )
            {
                modem_supervisor_add_task_clock_sync_time_req( 1 );

                // Force no synchronisation
                clock_sync_set_sync_lost( &( smtc_modem_services_ctx.clock_sync_ctx ) );
            }
        }
    }
    else
    {
        return_code = SMTC_MODEM_RC_FAIL;
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_time_stop_sync_service( uint8_t stack_id )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    if( clock_sync_is_enabled( &( smtc_modem_services_ctx.clock_sync_ctx ) ) == true )
    {
        clock_sync_set_enabled( &( smtc_modem_services_ctx.clock_sync_ctx ), false,
                                smtc_modem_services_ctx.clock_sync_ctx.sync_service_type );

        modem_supervisor_remove_task_clock_sync( );

        // Force no synchronisation
        clock_sync_set_sync_lost( &( smtc_modem_services_ctx.clock_sync_ctx ) );
    }
    else
    {
        return_code = SMTC_MODEM_RC_FAIL;
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_time( uint32_t* gps_time_s, uint32_t* gps_fractional_s )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( gps_time_s );
    RETURN_INVALID_IF_NULL( gps_fractional_s );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    if( clock_sync_is_time_valid( &( smtc_modem_services_ctx.clock_sync_ctx ) ) == true )
    {
        clock_sync_get_gps_time_second( &( smtc_modem_services_ctx.clock_sync_ctx ), gps_time_s, gps_fractional_s );
    }
    else
    {
        gps_time_s       = 0;
        gps_fractional_s = 0;
        return_code      = SMTC_MODEM_RC_NO_TIME;
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_time_set_alcsync_fport( uint8_t clock_sync_fport )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( clock_sync_set_alcsync_port( &( smtc_modem_services_ctx.clock_sync_ctx ), clock_sync_fport ) != 0 )
    {
        return_code = SMTC_MODEM_RC_INVALID;
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_time_trigger_sync_request( uint8_t                        stack_id,
                                                               smtc_modem_time_sync_service_t sync_service )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    if( sync_service != SMTC_MODEM_TIME_MAC_SYNC )
    {
        return SMTC_MODEM_RC_INVALID;
    }

    if( is_modem_connected( ) == false )
    {
        return_code = SMTC_MODEM_RC_FAIL;
    }
    else
    {
        modem_supervisor_add_task_device_time_req( 0 );
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_time_get_alcsync_fport( uint8_t* clock_sync_port )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( clock_sync_port );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    *clock_sync_port                     = clock_sync_get_alcsync_port( &( smtc_modem_services_ctx.clock_sync_ctx ) );

    return return_code;
}

smtc_modem_return_code_t smtc_modem_time_set_sync_interval_s( uint32_t sync_interval_s )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    if( clock_sync_set_interval_second( &( smtc_modem_services_ctx.clock_sync_ctx ), sync_interval_s ) !=
        CLOCK_SYNC_OK )
    {
        return_code = SMTC_MODEM_RC_INVALID;
    }

    return return_code;
}
smtc_modem_return_code_t smtc_modem_time_get_sync_interval_s( uint32_t* sync_interval_s )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    *sync_interval_s = clock_sync_get_interval_second( &( smtc_modem_services_ctx.clock_sync_ctx ) );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_time_set_sync_invalid_delay_s( uint32_t sync_invalid_delay_s )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( clock_sync_set_invalid_time_delay_s( &( smtc_modem_services_ctx.clock_sync_ctx ), sync_invalid_delay_s ) !=
        CLOCK_SYNC_OK )
    {
        return_code = SMTC_MODEM_RC_INVALID;
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_time_get_sync_invalid_delay_s( uint32_t* sync_invalid_delay_s )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    *sync_invalid_delay_s = clock_sync_get_invalid_time_delay_s( &( smtc_modem_services_ctx.clock_sync_ctx ) );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_status( uint8_t stack_id, smtc_modem_status_mask_t* status_mask )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( status_mask );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    *status_mask                         = ( smtc_modem_status_mask_t ) get_modem_status( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_alarm_start_timer( uint32_t alarm_s )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    modem_set_user_alarm( ( alarm_s > 0 ) ? ( smtc_modem_hal_get_time_in_s( ) + alarm_s ) : 0 );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_alarm_clear_timer( void )
{
    RETURN_BUSY_IF_TEST_MODE( );

    if( modem_get_user_alarm( ) == 0 )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Alarm clear timer impossible: no alarm timer is currently running" )
        return SMTC_MODEM_RC_NOT_INIT;
    }
    else
    {
        modem_set_user_alarm( 0 );
        return SMTC_MODEM_RC_OK;
    }
}

smtc_modem_return_code_t smtc_modem_alarm_get_remaining_time( uint32_t* remaining_time_in_s )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( remaining_time_in_s );

    if( modem_get_user_alarm( ) == 0 )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Alarm get remaining impossible: no alarm timer is currently running" )
        return SMTC_MODEM_RC_NOT_INIT;
    }
    else
    {
        int32_t abs_remaining_time = ( int32_t )( modem_get_user_alarm( ) - smtc_modem_hal_get_time_in_s( ) );

        *remaining_time_in_s = ( abs_remaining_time > 0 ) ? ( abs_remaining_time ) : 0;
        return SMTC_MODEM_RC_OK;
    }
}

smtc_modem_return_code_t smtc_modem_get_joineui( uint8_t stack_id, uint8_t joineui[SMTC_MODEM_EUI_LENGTH] )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( joineui );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    lorawan_api_get_joineui( joineui );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_joineui( uint8_t stack_id, const uint8_t joineui[SMTC_MODEM_EUI_LENGTH] )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( joineui );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    if( get_join_state( ) != MODEM_NOT_JOINED )  // the modem have to be leave from the network to modify the key
    {
        return_code = SMTC_MODEM_RC_BUSY;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else
    {
        lorawan_api_set_joineui( joineui );
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_deveui( uint8_t stack_id, uint8_t deveui[SMTC_MODEM_EUI_LENGTH] )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( deveui );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    lorawan_api_get_deveui( deveui );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_deveui( uint8_t stack_id, const uint8_t deveui[SMTC_MODEM_EUI_LENGTH] )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( deveui );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( get_join_state( ) != MODEM_NOT_JOINED )  // the modem have to be leave from the network to modify the key
    {
        return_code = SMTC_MODEM_RC_BUSY;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else
    {
        lorawan_api_set_deveui( deveui );
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_nwkkey( uint8_t stack_id, const uint8_t nwkkey[SMTC_MODEM_KEY_LENGTH] )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( nwkkey );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( get_join_state( ) != MODEM_NOT_JOINED )  // the modem have to be leave from the network to modify the key
    {
        return_code = SMTC_MODEM_RC_BUSY;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else
    {
        modem_context_set_appkey( nwkkey );
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_class( uint8_t stack_id, smtc_modem_class_t* lorawan_class )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( lorawan_class );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    *lorawan_class                       = get_modem_class( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_class( uint8_t stack_id, smtc_modem_class_t lorawan_class )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( set_modem_class( lorawan_class ) == SET_ERROR )
    {
        return_code = SMTC_MODEM_RC_INVALID;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call with class not valid\n", __func__ );
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_multicast_set_grp_config( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                              uint32_t      mc_grp_addr,
                                                              const uint8_t mc_nwk_skey[SMTC_MODEM_KEY_LENGTH],
                                                              const uint8_t mc_app_skey[SMTC_MODEM_KEY_LENGTH] )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( mc_nwk_skey );
    RETURN_INVALID_IF_NULL( mc_app_skey );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc =
        lorawan_api_multicast_set_group_config( mc_grp_id, mc_grp_addr, mc_nwk_skey, mc_app_skey );

    switch( rc )
    {
    case LORAWAN_MC_RC_OK:
        modem_rc = SMTC_MODEM_RC_OK;
        break;
    case LORAWAN_MC_RC_ERROR_BAD_ID:
        modem_rc = SMTC_MODEM_RC_INVALID;
        break;
    case LORAWAN_MC_RC_ERROR_BUSY:
        // intentional fallthrought
    case LORAWAN_MC_RC_ERROR_CRYPTO:
        // intentional fallthrought
    default:
        modem_rc = SMTC_MODEM_RC_FAIL;
        break;
    }
    return modem_rc;
}

smtc_modem_return_code_t smtc_modem_multicast_get_grp_config( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                              uint32_t* mc_grp_addr )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( mc_grp_addr );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_get_group_config( mc_grp_id, mc_grp_addr );

    switch( rc )
    {
    case LORAWAN_MC_RC_OK:
        modem_rc = SMTC_MODEM_RC_OK;
        break;
    case LORAWAN_MC_RC_ERROR_BAD_ID:
        modem_rc = SMTC_MODEM_RC_INVALID;
        break;
    default:
        modem_rc = SMTC_MODEM_RC_FAIL;
        break;
    }
    return modem_rc;
}

smtc_modem_return_code_t smtc_modem_multicast_start_session( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                             uint32_t freq, uint8_t dr )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_start_session( mc_grp_id, freq, dr );

    switch( rc )
    {
    case LORAWAN_MC_RC_OK:
        modem_rc = SMTC_MODEM_RC_OK;
        break;
    case LORAWAN_MC_RC_ERROR_PARAM:
        // intentional fallthrought
    case LORAWAN_MC_RC_ERROR_INCOMPATIBLE_SESSION:
        // intentional fallthrought
    case LORAWAN_MC_RC_ERROR_BAD_ID:
        modem_rc = SMTC_MODEM_RC_INVALID;
        break;
    case LORAWAN_MC_RC_ERROR_NOT_INIT:
        modem_rc = SMTC_MODEM_RC_NOT_INIT;
        break;
    case LORAWAN_MC_RC_ERROR_BUSY:
        // intentional fallthrought
    default:
        modem_rc = SMTC_MODEM_RC_FAIL;
        break;
    }
    return modem_rc;
}

smtc_modem_return_code_t smtc_modem_multicast_get_session_status( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                                  bool* is_session_started, uint32_t* freq,
                                                                  uint8_t* dr )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( is_session_started );
    RETURN_INVALID_IF_NULL( freq );
    RETURN_INVALID_IF_NULL( dr );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_get_session_status( mc_grp_id, is_session_started, freq, dr );

    switch( rc )
    {
    case LORAWAN_MC_RC_OK:
        modem_rc = SMTC_MODEM_RC_OK;
        break;
    case LORAWAN_MC_RC_ERROR_BAD_ID:
        modem_rc = SMTC_MODEM_RC_INVALID;
        break;
    default:
        modem_rc = SMTC_MODEM_RC_FAIL;
        break;
    }
    return modem_rc;
}

smtc_modem_return_code_t smtc_modem_multicast_stop_session( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_stop_session( mc_grp_id );

    switch( rc )
    {
    case LORAWAN_MC_RC_OK:
        modem_rc = SMTC_MODEM_RC_OK;
        break;
    case LORAWAN_MC_RC_ERROR_BAD_ID:
        modem_rc = SMTC_MODEM_RC_INVALID;
        break;
    default:
        modem_rc = SMTC_MODEM_RC_FAIL;
        break;
    }
    return modem_rc;
}

smtc_modem_return_code_t smtc_modem_multicast_stop_all_sessions( uint8_t stack_id )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_stop_all_sessions( );

    switch( rc )
    {
    case LORAWAN_MC_RC_OK:
        modem_rc = SMTC_MODEM_RC_OK;
        break;
    default:
        modem_rc = SMTC_MODEM_RC_FAIL;
        break;
    }
    return modem_rc;
}

smtc_modem_return_code_t smtc_modem_get_region( uint8_t stack_id, smtc_modem_region_t* region )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( region );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    *region                              = get_modem_region( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_region( uint8_t stack_id, smtc_modem_region_t region )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( get_join_state( ) != MODEM_NOT_JOINED )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
        return SMTC_MODEM_RC_BUSY;
    }

    if( set_modem_region( region ) == SET_ERROR )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call with region not valid\n", __func__ );
        return SMTC_MODEM_RC_INVALID;
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_adr_get_profile( uint8_t stack_id, smtc_modem_adr_profile_t* adr_profile )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( adr_profile );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    *adr_profile                         = get_modem_adr_profile( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_adr_set_profile( uint8_t stack_id, smtc_modem_adr_profile_t adr_profile,
                                                     const uint8_t adr_custom_data[SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH] )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    e_set_error_t            status;

    if( adr_profile == SMTC_MODEM_ADR_PROFILE_CUSTOM )
    {
        // Check parameters
        if( NULL == adr_custom_data )
        {
            return SMTC_MODEM_RC_INVALID;
        }

        status = set_modem_adr_profile( adr_profile, adr_custom_data, 16 );
    }
    else if( ( adr_profile == SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED ) ||
             ( adr_profile == SMTC_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE ) ||
             ( adr_profile == SMTC_MODEM_ADR_PROFILE_MOBILE_LOW_POWER ) )
    {
        status = set_modem_adr_profile( adr_profile, adr_custom_data, 0 );
    }
    else
    {
        status = SET_ERROR;
    }
    if( ( adr_profile == SMTC_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE ) ||
        ( adr_profile == SMTC_MODEM_ADR_PROFILE_MOBILE_LOW_POWER ) || ( adr_profile == SMTC_MODEM_ADR_PROFILE_CUSTOM ) )
    {
        // reset current adr mobile count
        modem_reset_current_adr_mobile_count( );
    }
    if( status == SET_ERROR )
    {
        return_code = SMTC_MODEM_RC_INVALID;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call with adr profile not valid\n", __func__ );
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_available_datarates( uint8_t stack_id, uint16_t* available_datarates_mask )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( available_datarates_mask );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    *available_datarates_mask            = lorawan_api_mask_tx_dr_channel_up_dwell_time_check( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_dm_get_fport( uint8_t* dm_fport )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( dm_fport );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    *dm_fport                            = get_modem_dm_port( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_dm_set_fport( uint8_t dm_fport )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( set_modem_dm_port( dm_fport ) == SET_ERROR )
    {
        return_code = SMTC_MODEM_RC_INVALID;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call with DM port not valid\n", __func__ );
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_dm_get_info_interval( smtc_modem_dm_info_interval_format_t* format,
                                                          uint8_t*                              interval )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( format );
    RETURN_INVALID_IF_NULL( interval );

    smtc_modem_return_code_t return_code    = SMTC_MODEM_RC_OK;
    uint8_t                  modem_interval = 0;

    modem_interval = get_modem_dm_interval( );

    *format   = ( smtc_modem_dm_info_interval_format_t )( ( modem_interval >> 6 ) & 0x03 );
    *interval = modem_interval & 0x3F;

    return return_code;
}

smtc_modem_return_code_t smtc_modem_dm_set_info_interval( smtc_modem_dm_info_interval_format_t format,
                                                          uint8_t                              interval )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code    = SMTC_MODEM_RC_OK;
    uint8_t                  modem_interval = 0;

    if( ( format > SMTC_MODEM_DM_INFO_INTERVAL_IN_MINUTE ) || ( interval > 0x3F ) )
    {
        return_code = SMTC_MODEM_RC_INVALID;
    }
    else
    {
        modem_interval = ( ( ( uint8_t ) format << 6 ) & 0xC0 ) | ( interval & 0x3F );

        if( set_modem_dm_interval( modem_interval ) == SET_ERROR )
        {
            return_code = SMTC_MODEM_RC_INVALID;
            SMTC_MODEM_HAL_TRACE_ERROR( "%s call but interval not valid\n", __func__ );
        }
        else
        {
            modem_supervisor_add_task_dm_status( get_modem_dm_interval_second( ) );
        }
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_dm_get_info_fields( uint8_t* dm_fields_payload, uint8_t* dm_field_length )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( dm_fields_payload );
    RETURN_INVALID_IF_NULL( dm_field_length );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    return_code = smtc_modem_get_dm_status_with_rate( dm_fields_payload, dm_field_length, DM_INFO_PERIODIC );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_dm_set_info_fields( const uint8_t* dm_fields_payload, uint8_t dm_field_length )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( dm_fields_payload );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    return_code = smtc_modem_set_dm_status_with_rate( dm_fields_payload, dm_field_length, DM_INFO_PERIODIC );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_dm_request_single_uplink( const uint8_t* dm_fields_payload,
                                                              uint8_t        dm_field_length )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( dm_fields_payload );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( is_modem_connected( ) == false )
    {
        return_code = SMTC_MODEM_RC_FAIL;
    }
    else
    {
        return_code = smtc_modem_set_dm_status_with_rate( dm_fields_payload, dm_field_length, DM_INFO_NOW );
        if( return_code == SMTC_MODEM_RC_OK )
        {
            modem_supervisor_add_task_dm_status_now( );
        }
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_dm_set_user_data( const uint8_t user_data[SMTC_MODEM_DM_USER_DATA_LENGTH] )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( user_data );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    set_modem_appstatus( user_data );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_dm_get_user_data( uint8_t user_data[SMTC_MODEM_DM_USER_DATA_LENGTH] )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( user_data );

    get_modem_appstatus( user_data );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_join_network( uint8_t stack_id )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    if( lorawan_api_get_activation_mode( ) == ACTIVATION_MODE_ABP )
    {
        return_code = SMTC_MODEM_RC_FAIL;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is configured in ABP mode\n", __func__ );
    }
    else if( get_join_state( ) != MODEM_NOT_JOINED )  // the modem have to be leave from the network to join
    {
        return_code = SMTC_MODEM_RC_BUSY;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else if( get_modem_muted( ) == MODEM_INFINITE_MUTE )
    {
        return_code = SMTC_MODEM_RC_FAIL;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s Modem is muted\n", __func__ );
    }
    else if( get_modem_suspend( ) == MODEM_SUSPEND )
    {
        return_code = SMTC_MODEM_RC_FAIL;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s Modem is suspend\n", __func__ );
    }
    else
    {
        modem_supervisor_add_task_join( );
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_leave_network( uint8_t stack_id )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    lorawan_api_class_c_enabled( false );
    set_modem_status_modem_joined( false );
    lorawan_api_join_status_clear( );
    set_modem_status_joining( false );

    // set stream and file upload status to false
    set_modem_status_file_upload( false );
    set_modem_status_streaming( false );

    // re init task to retrieve a clean env
    modem_supervisor_init_task( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_suspend_radio_communications( bool suspend )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( set_modem_suspend( suspend ) == SET_ERROR )
    {
        return_code = SMTC_MODEM_RC_INVALID;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but suspend value is not valid\n", __func__ );
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_next_tx_max_payload( uint8_t stack_id, uint8_t* tx_max_payload_size )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( tx_max_payload_size );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( get_join_state( ) == MODEM_JOINED )
    {
        *tx_max_payload_size = lorawan_api_next_max_payload_length_get( );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is not join\n", __func__ );
        *tx_max_payload_size = 255;
        return_code          = SMTC_MODEM_RC_FAIL;
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_request_uplink( uint8_t stack_id, uint8_t f_port, bool confirmed,
                                                    const uint8_t* payload, uint8_t payload_length )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( payload );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    return_code                          = smtc_modem_send_tx( f_port, confirmed, payload, payload_length, false );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_request_emergency_uplink( uint8_t stack_id, uint8_t f_port, bool confirmed,
                                                              const uint8_t* payload, uint8_t payload_length )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( payload );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    return_code                          = smtc_modem_send_tx( f_port, confirmed, payload, payload_length, true );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_request_empty_uplink( uint8_t stack_id, bool send_fport, uint8_t fport,
                                                          bool confirmed )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    return_code                          = smtc_modem_send_empty_tx( fport, send_fport, confirmed );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_file_upload_init( uint8_t stack_id, uint8_t index,
                                                      smtc_modem_file_upload_cipher_mode_t cipher_mode,
                                                      const uint8_t* file, uint16_t file_length,
                                                      uint32_t average_delay_s )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    if( file_length == 0 )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Upload initialization fails: size = 0 is not allowed\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    else if( cipher_mode > SMTC_MODEM_FILE_UPLOAD_AES_WITH_APPSKEY )
    {
        return SMTC_MODEM_RC_INVALID;
    }
    else if( ( modem_get_upload_state( ) == MODEM_UPLOAD_INIT_AND_FILLED ) ||
             ( modem_get_upload_state( ) == MODEM_UPLOAD_ON_GOING ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "File Upload still in going\n" );
        return SMTC_MODEM_RC_BUSY;
    }
    else if( file == NULL )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Upload file data is null\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    // save current file size and buff  (to keep hw modem compatiblity if needed)
    upload_size  = file_length;
    upload_pdata = ( uint32_t* ) file;

    // get the next modem upload session counter
    uint8_t next_session_counter = modem_context_compute_and_get_next_dm_upload_sctr( );

    if( file_upload_init( &( smtc_modem_services_ctx.file_upload_ctx ), UPLOAD_SID, ( uint32_t ) file_length,
                          average_delay_s, index, ( uint8_t ) cipher_mode, next_session_counter ) != FILE_UPLOAD_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Upload initialization fails\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    SMTC_MODEM_HAL_TRACE_PRINTF( "%s, cipher_mode: %d, size:%d, average_delay:%d, session counter:%d", __func__,
                                 cipher_mode, file_length, average_delay_s, next_session_counter );
    // attach the file
    file_upload_attach_file_buffer( &( smtc_modem_services_ctx.file_upload_ctx ), file );

    modem_set_upload_state( MODEM_UPLOAD_INIT_AND_FILLED );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_file_upload_start( uint8_t stack_id )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    if( is_modem_connected( ) == false )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    if( modem_get_upload_state( ) == MODEM_UPLOAD_ON_GOING )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FileUpload still in progress..\n" );
        return SMTC_MODEM_RC_BUSY;
    }
    if( modem_get_upload_state( ) != MODEM_UPLOAD_INIT_AND_FILLED )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "File upload session not initialized\n" );
        return SMTC_MODEM_RC_NOT_INIT;
    }

    // ready to prepare the file to be uploaded
    file_upload_prepare_upload( &( smtc_modem_services_ctx.file_upload_ctx ) );

    // add the first upload task in scheduler
    modem_supervisor_add_task_file_upload( smtc_modem_hal_get_random_nb_in_range( 0, 2 ) );

    modem_set_upload_state( MODEM_UPLOAD_ON_GOING );
    // set modem file upload status to true
    set_modem_status_file_upload( true );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_file_upload_reset( uint8_t stack_id )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    if( ( modem_get_upload_state( ) == MODEM_UPLOAD_NOT_INIT ) ||
        ( modem_get_upload_state( ) == MODEM_UPLOAD_FINISHED ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "No file upload session is on going\n" );
        return SMTC_MODEM_RC_NOT_INIT;
    }

    SMTC_MODEM_HAL_TRACE_WARNING( "File Upload Cancel and session reset!\n" );
    modem_set_upload_state( MODEM_UPLOAD_NOT_INIT );

    set_modem_status_file_upload( false );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_stream_init( uint8_t stack_id, uint8_t fport,
                                                 smtc_modem_stream_cipher_mode_t cipher_mode,
                                                 uint8_t                         redundancy_ratio_percent )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    // Check parameters validity
    if( modem_port_reserved( fport ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "STREAM_INIT fport invalid\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    if( cipher_mode > SMTC_MODEM_STREAM_AES_WITH_APPSKEY )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "STREAM_INIT encryption mode invalid\n" );
        return SMTC_MODEM_RC_INVALID;
    }

    // If parameter fport is set to 0 => use current dm port
    if( fport == 0 )
    {
        SMTC_MODEM_HAL_TRACE_MSG( "STREAM_INIT Using default DM port\n" );
        fport = get_modem_dm_port( );
    }

    // First reset stream service
    stream_reset( &( smtc_modem_services_ctx.stream_ROSE_ctx ) );

    // initialize stream session
    if( stream_init( &( smtc_modem_services_ctx.stream_ROSE_ctx ) ) != STREAM_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "STREAM_INIT FAILED\n" );
        return SMTC_MODEM_RC_FAIL;
    }
    // enable encryption if needed
    if( cipher_mode == SMTC_MODEM_STREAM_AES_WITH_APPSKEY )
    {
        if( stream_enable_encryption( &( smtc_modem_services_ctx.stream_ROSE_ctx ) ) != STREAM_OK )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "STREAM_INIT ENCRYPTION FAILED\n" );
            return SMTC_MODEM_RC_FAIL;
        }
    }

    // Remove previous ongoing stream task to avoid event generation
    modem_supervisor_remove_task( STREAM_TASK );

    modem_set_stream_port( fport );
    stream_set_rr( &( smtc_modem_services_ctx.stream_ROSE_ctx ), redundancy_ratio_percent );
    modem_set_stream_encryption( cipher_mode == SMTC_MODEM_STREAM_AES_WITH_APPSKEY );
    modem_set_stream_state( MODEM_STREAM_INIT );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_stream_add_data( uint8_t stack_id, const uint8_t* data, uint8_t len )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( data );

    // Check if modem is joined, not suspended or muted
    if( is_modem_connected( ) == false )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    // No existing stream
    if( modem_get_stream_state( ) == MODEM_STREAM_NOT_INIT )
    {
        smtc_modem_return_code_t rc;
        // Start new unencrypted session with rr to 110% on dm port
        rc = smtc_modem_stream_init( stack_id, get_modem_dm_port( ), SMTC_MODEM_STREAM_NO_CIPHER, 110 );
        if( rc != SMTC_MODEM_RC_OK )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "Stream implicit init failed\n" );
            return rc;
        }
    }

    stream_return_code_t stream_rc = stream_add_data( &( smtc_modem_services_ctx.stream_ROSE_ctx ), data, len );

    switch( stream_rc )
    {
    case STREAM_BADSIZE:
        SMTC_MODEM_HAL_TRACE_ERROR( "STREAM ADD DATA: Invalid length\n" );
        return SMTC_MODEM_RC_INVALID;
    case STREAM_BUSY:
        SMTC_MODEM_HAL_TRACE_ERROR( "STREAM ADD DATA: Buffer is full\n" );
        return SMTC_MODEM_RC_BUSY;
    case STREAM_FAIL:
        SMTC_MODEM_HAL_TRACE_ERROR( "STREAM ADD DATA: No data record provided\n" );
        return SMTC_MODEM_RC_FAIL;
    default:
        break;
    }

#ifdef LORAWAN_BYPASS_ENABLED
    if( stream_bypass_enabled == true )
    {
        SMTC_MODEM_HAL_TRACE_INFO( "STREAM_SEND BYPASS [OK]\n" );
        return SMTC_MODEM_RC_OK;
    }
#endif  // LORAWAN_BYPASS_ENABLED

    modem_supervisor_add_task_stream( );

    SMTC_MODEM_HAL_TRACE_INFO( "STREAM_SEND [OK]\n" );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_stream_status( uint8_t stack_id, uint16_t* pending, uint16_t* free )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( pending );
    RETURN_INVALID_IF_NULL( free );

    if( modem_get_stream_state( ) == MODEM_STREAM_NOT_INIT )
    {
        return SMTC_MODEM_RC_NOT_INIT;
    }

    stream_status( &( smtc_modem_services_ctx.stream_ROSE_ctx ), pending, free );
    return SMTC_MODEM_RC_OK;
}

#ifdef LORAWAN_BYPASS_ENABLED
/*
 * When the bypass is enabled, don't send anything via LORAWAN.
 * Instead, send/receive locally via these functions
 */

/*
 * Enable and Disable bypass
 */
void modem_stream_bypass_enable( bool enabled )
{
    stream_bypass_enabled = enabled;
    SMTC_MODEM_HAL_TRACE_PRINTF( "STREAM BYPASS %d\n", stream_bypass_enabled );
}

/*
 * Get one full uplink payload
 */
smtc_modem_return_code_t smtc_modem_stream_bypass_get_fragment( uint8_t* buffer, uint32_t frag_cnt, uint8_t* len )
{
    if( !stream_bypass_enabled )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    if( !stream_data_pending( ) )
    {
        return SMTC_MODEM_RC_BAD_SIZE;
    }
    if( STREAM_OK != stream_get_fragment( buffer, frag_cnt, len ) )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
}

/*
 * Send a downlink payload
 */
smtc_modem_return_code_t smtc_modem_stream_bypass_send_downlink( uint8_t* buffer, uint8_t len )
{
    if( !stream_bypass_enabled )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    if( STREAM_OK != stream_process_dn_frame( buffer, len ) )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
}

#endif  // LORAWAN_BYPASS_ENABLED

smtc_modem_return_code_t smtc_modem_set_certification_mode( uint8_t stack_id, bool enable )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    lorawan_api_modem_certification_set( ( uint8_t ) enable );
    smtc_secure_element_store_context( );

    // Enable Dutycycle when certification mode is leaved
    if( enable == false )
    {
        smtc_modem_test_duty_cycle_app_activate( true );
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_certification_mode( uint8_t stack_id, bool* enable )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( enable );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    *enable                              = lorawan_api_modem_certification_is_enabled( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_connection_timeout_set_thresholds( uint8_t  stack_id,
                                                                       uint16_t nb_of_uplinks_before_network_controlled,
                                                                       uint16_t nb_of_uplinks_before_reset )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    modem_set_adr_mobile_timeout_config( nb_of_uplinks_before_network_controlled );

    if( lorawan_api_no_rx_packet_count_config_set( nb_of_uplinks_before_reset ) != OKLORAWAN )
    {
        return_code = SMTC_MODEM_RC_INVALID;
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_connection_timeout_get_thresholds(
    uint8_t stack_id, uint16_t* nb_of_uplinks_before_network_controlled, uint16_t* nb_of_uplinks_before_reset )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( nb_of_uplinks_before_network_controlled );
    RETURN_INVALID_IF_NULL( nb_of_uplinks_before_reset );

    *nb_of_uplinks_before_network_controlled = modem_get_adr_mobile_timeout_config( );
    *nb_of_uplinks_before_reset              = lorawan_api_no_rx_packet_count_config_get( );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_connection_timeout_get_current_values(
    uint8_t stack_id, uint16_t* nb_of_uplinks_before_network_controlled, uint16_t* nb_of_uplinks_before_reset )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( nb_of_uplinks_before_network_controlled );
    RETURN_INVALID_IF_NULL( nb_of_uplinks_before_reset );

    *nb_of_uplinks_before_network_controlled = modem_get_current_adr_mobile_count( );
    *nb_of_uplinks_before_reset              = lorawan_api_no_rx_packet_count_current_get( );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_duty_cycle_status( int32_t* duty_cycle_status_ms )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( duty_cycle_status_ms );

    *duty_cycle_status_ms = -1 * lorawan_api_next_free_duty_cycle_ms_get( );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_suspend_before_user_radio_access( void )
{
    // First check if modem is in test mode
    if( modem_get_test_mode_status( ) == true )
    {
        return SMTC_MODEM_RC_BUSY;
    }

    // Put modem in suspended mode to prevent scheduler to be called
    smtc_modem_suspend_radio_communications( true );

    SMTC_MODEM_HAL_TRACE_PRINTF( "Suspend modem user radio access\n" );

    // Protect radio access with a suspension of all other task in radio planner (put an infinite empty task)
    return ( modem_context_suspend_user_radio_access( RP_TASK_TYPE_NONE ) == true ) ? SMTC_MODEM_RC_OK
                                                                                    : SMTC_MODEM_RC_FAIL;
}

smtc_modem_return_code_t smtc_modem_resume_after_user_radio_access( void )
{
    RETURN_BUSY_IF_TEST_MODE( );

    SMTC_MODEM_HAL_TRACE_PRINTF( "Resume modem user radio access\n" );

    // First stop the radio_planner suspension (always RC_OK)
    modem_context_resume_user_radio_access( );

    // Then put the modem in NOT_SUSPENDED mode to relaunch the scheduler (always RC_OK)
    smtc_modem_suspend_radio_communications( false );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_stack_state( uint8_t stack_id, smtc_modem_stack_state_t* stack_state )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( stack_state );

    lr1mac_states_t lr1mac_state = lorawan_api_state_get( );
    if( ( lr1mac_state == LWPSTATE_IDLE ) ||
        ( lr1mac_state == LWPSTATE_TX_WAIT ) )  // second test for the case of nbtrans> 0
    {
        *stack_state = SMTC_MODEM_STACK_STATE_IDLE;
    }
    else
    {
        *stack_state = SMTC_MODEM_STACK_STATE_BUSY;
    }

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_set_network_type( uint8_t stack_id, bool network_type )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    modem_context_set_network_type( network_type );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_network_type( uint8_t stack_id, bool* network_type )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( network_type );

    *network_type = modem_context_get_network_type( );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_lbt_set_parameters( uint8_t stack_id, uint32_t listen_duration_ms,
                                                        int16_t threshold_dbm, uint32_t bw_hz )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    SMTC_MODEM_HAL_TRACE_PRINTF( "LBT, duration:%d, threshold:%d, bw:%d\n", listen_duration_ms, threshold_dbm, bw_hz );
    lbt_config_listen_duration_ms = listen_duration_ms;
    lbt_config_threshold_dbm      = threshold_dbm;
    lbt_config_bw_hz              = bw_hz;
    lbt_config_available          = true;
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_lbt_get_parameters( uint8_t stack_id, uint32_t* listen_duration_ms,
                                                        int16_t* threshold_dbm, uint32_t* bw_hz )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( listen_duration_ms );
    RETURN_INVALID_IF_NULL( threshold_dbm );
    RETURN_INVALID_IF_NULL( bw_hz );

    *listen_duration_ms = lbt_config_listen_duration_ms;
    *threshold_dbm      = lbt_config_threshold_dbm;
    *bw_hz              = lbt_config_bw_hz;
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_lbt_set_state( uint8_t stack_id, bool enable )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    if( enable == true )
    {
        // check if a configuration was set before
        if( lbt_config_available == true )
        {
            smtc_lbt_configure( lorawan_api_stack_mac_get( )->lbt_obj, lbt_config_listen_duration_ms,
                                lbt_config_threshold_dbm, lbt_config_bw_hz );
            return SMTC_MODEM_RC_OK;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "A configuration shall be set before starting LBT \n" );
            return SMTC_MODEM_RC_FAIL;
        }
    }
    else
    {
        smtc_lbt_disable( lorawan_api_stack_mac_get( )->lbt_obj );
        return SMTC_MODEM_RC_OK;
    }
}

smtc_modem_return_code_t smtc_modem_lbt_get_state( uint8_t stack_id, bool* enabled )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( enabled );

    *enabled = smtc_lbt_is_enable( lorawan_api_stack_mac_get( )->lbt_obj );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_set_nb_trans( uint8_t stack_id, uint8_t nb_trans )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    if( lorawan_api_nb_trans_set( nb_trans ) == OKLORAWAN )
    {
        return SMTC_MODEM_RC_OK;
    }
    else
    {
        return SMTC_MODEM_RC_INVALID;
    }
}

smtc_modem_return_code_t smtc_modem_get_nb_trans( uint8_t stack_id, uint8_t* nb_trans )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( nb_trans );

    *nb_trans = lorawan_api_nb_trans_get( );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_set_crystal_error( uint32_t crystal_error_per_thousand )
{
    RETURN_BUSY_IF_TEST_MODE( );

    if(crystal_error_per_thousand > MAX_CRYSTAL_ERROR)
    {
        return SMTC_MODEM_RC_INVALID;
    }

    lorawan_api_set_crystal_error( crystal_error_per_thousand );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_crystal_error( uint32_t* crystal_error_per_thousand )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( crystal_error_per_thousand );

    *crystal_error_per_thousand = lorawan_api_get_crystal_error( );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_lorawan_request_link_check( uint8_t stack_id )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    if( is_modem_connected( ) == false )
    {
        return_code = SMTC_MODEM_RC_FAIL;
    }
    else
    {
        modem_supervisor_add_task_link_check_req( 0 );
    }

    return return_code;
}

/* ------------ Basic Modem LR1110 Extension functions ------------*/

smtc_modem_return_code_t smtc_modem_get_pin( uint8_t stack_id, uint8_t chip_pin[4] )
{
    UNUSED( stack_id );

    RETURN_BUSY_IF_TEST_MODE( );

#if defined( USE_LR1110_SE )
    lr1110_system_uid_t      deveui;
    lr1110_system_join_eui_t joineui;
    lorawan_api_get_deveui( ( uint8_t* ) deveui );
    lorawan_api_get_joineui( ( uint8_t* ) joineui );

    lr1110_status_t status = smtc_modem_lr1110_system_read_pin_custom_eui( modem_context_get_modem_radio_ctx( ), deveui,
                                                                           joineui, 0, chip_pin );

    // when pin code is read, a new key derivation is done in lr1110 so a external app_key is used it will be lost
    // and shall be updated once more. Corrupt the key crc so that update is possible
    modem_context_appkey_is_derived( );

    if( status != LR1110_STATUS_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
#else
    return SMTC_MODEM_RC_FAIL;
#endif
}

smtc_modem_return_code_t smtc_modem_get_chip_eui( uint8_t stack_id, uint8_t chip_eui[8] )
{
    UNUSED( stack_id );

    // First check if modem is in test mode
    if( modem_get_test_mode_status( ) == true )
    {
        return SMTC_MODEM_RC_BUSY;
    }

#if defined( USE_LR1110_SE )
    lr1110_status_t status = smtc_modem_lr1110_system_read_uid( modem_context_get_modem_radio_ctx( ), chip_eui );

    if( status != LR1110_STATUS_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
#else
    return SMTC_MODEM_RC_FAIL;
#endif
}

smtc_modem_return_code_t smtc_modem_derive_keys( uint8_t stack_id )
{
    UNUSED( stack_id );

    RETURN_BUSY_IF_TEST_MODE( );
    if( get_join_state( ) != MODEM_NOT_JOINED )
    {
        return SMTC_MODEM_RC_BUSY;
    }

#if defined( USE_LR1110_SE )
    lr1110_system_uid_t      deveui;
    lr1110_system_join_eui_t joineui;
    lr1110_system_pin_t      pin;

    lorawan_api_get_deveui( ( uint8_t* ) deveui );
    lorawan_api_get_joineui( ( uint8_t* ) joineui );

    // Read pin code force a key derivation
    lr1110_status_t status = smtc_modem_lr1110_system_read_pin( modem_context_get_modem_radio_ctx( ), pin );

    // when pin code is read, a new key derivation is done in lr1110 so a external app_key is used it will be lost
    // and shall be updated once more. Corrupt the key crc so that update is possible
    modem_context_appkey_is_derived( );

    if( status != LR1110_STATUS_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
#elif defined( LR1110_MODEM_E )
    uint8_t deveui[8];
    uint8_t appeui[8];
    lorawan_api_get_deveui( deveui );
    lorawan_api_get_joineui( appeui );

    if( smtc_crypto_se_derive_keys( deveui, appeui ) == 0 )
    {
        modem_context_appkey_is_derived( );
        return SMTC_MODEM_RC_OK;
    }
    return SMTC_MODEM_RC_FAIL;
#else
    return SMTC_MODEM_RC_FAIL;
#endif
}

/* ------------ Modem-E Api Extension functions ------------*/
#if defined( LR1110_MODEM_E )
smtc_modem_return_code_t smtc_modem_stream_get_redundancy_ratio( uint8_t stack_id, uint8_t* stream_rr )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    *stream_rr = stream_get_rr( &( smtc_modem_services_ctx.stream_ROSE_ctx ) );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_stream_set_redundancy_ratio( uint8_t stack_id, uint8_t redundancy_ratio_percent )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    stream_set_rr( &( smtc_modem_services_ctx.stream_ROSE_ctx ), redundancy_ratio_percent );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_set_default_key( uint8_t default_app_key[16], uint8_t default_dev_eui[8],
                                                     uint8_t default_join_eui[8] )
{
    RETURN_BUSY_IF_TEST_MODE( );

    lorawan_api_set_default_key( default_app_key, default_dev_eui, default_join_eui );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_list_regions( uint8_t* region_list, uint8_t* nb_region )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    memcpy( region_list, smtc_real_region_list, SMTC_REAL_REGION_LIST_LENGTH );
    *nb_region = SMTC_REAL_REGION_LIST_LENGTH;
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_rf_output( rf_output_t rf_output )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( modem_set_rfo_pa( rf_output ) != SET_OK )
    {
        return_code = SMTC_MODEM_RC_INVALID;
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_time_available( )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code       = SMTC_MODEM_RC_INVALID;
    uint32_t                 gps_time_s        = 0;
    uint32_t                 fractional_second = 0;

    clock_sync_get_gps_time_second( &( smtc_modem_services_ctx.clock_sync_ctx ), &gps_time_s, &fractional_second );
    if( gps_time_s > 0 )
    {
        return_code = SMTC_MODEM_RC_OK;
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_time( uint32_t gps_time_s )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    clock_sync_set_gps_time( &( smtc_modem_services_ctx.clock_sync_ctx ), gps_time_s );

    return return_code;
}
void empty_task_launch_callback_for_rp( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    smtc_modem_hal_start_radio_tcxo( );
    rp_stats_set_none_timestamp( &rp->stats, rp_hal_timestamp_get( ) );
    SMTC_MODEM_HAL_TRACE_PRINTF( "launch task empty\n" );
}

void modem_suspend_rp( e_sniff_mode_t sniff_mode )
{
    rp_radio_params_t fake_radio_params = { 0 };
    rp_task_t         rp_task;
    uint8_t           fake_payload[2]   = { 0 };
    uint16_t          fake_payload_size = 2;
    rp_task.hook_id                     = 0;
    rp_task.duration_time_ms            = 20000;
    rp_task.state                       = RP_TASK_STATE_SCHEDULE;
    switch( sniff_mode )
    {
#if defined( _MODEM_E_GNSS_ENABLE )
    case GNSS_SNIFF_MODE:
        rp_task.type = RP_TASK_TYPE_GNSS_SNIFF;
        break;
#endif  // _MODEM_E_GNSS_ENABLE
#if defined( _MODEM_E_WIFI_ENABLE )
    case WIFI_SNIFF_MODE:
        rp_task.type = RP_TASK_TYPE_WIFI_SNIFF;
        break;
#endif  // _MODEM_E_WIFI_ENABLE
#if defined( _MODEM_E_GNSS_ENABLE )
    case GNSS_TEST_RSSI:
        rp_task.type = RP_TASK_TYPE_GNSS_RSSI;
        break;
#endif  // _MODEM_E_GNSS_ENABLE
#if defined( _MODEM_E_WIFI_ENABLE )
    case WIFI_TEST_RSSI:
        rp_task.type = RP_TASK_TYPE_WIFI_RSSI;
        break;
#endif  // _MODEM_E_WIFI_ENABLE
    default:
        rp_task.type = RP_TASK_TYPE_NONE;
        break;
    }
    rp_task.launch_task_callbacks = empty_task_launch_callback_for_rp;
    rp_task.start_time_ms         = smtc_modem_hal_get_time_in_ms( ) + 4;
    rp_task_enqueue( &modem_radio_planner, &rp_task, fake_payload, fake_payload_size, &fake_radio_params );
}
void modem_resume_rp( void )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "Resume rp\n" );
    rp_task_abort( &modem_radio_planner, 0 );
    smtc_modem_hal_stop_radio_tcxo( );
}

smtc_modem_return_code_t smtc_modem_set_output_power_lut( uint8_t config[30] )
{
    modem_context_set_power_config_lut( config );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_output_power_lut( uint8_t config[30] )
{
    // get the internal lut
    modem_power_config_t* config_lut = modem_context_get_power_config_lut( );

    // Fill the tab
    for( uint8_t i = 0; i < POWER_CONFIG_LUT_SIZE; i++ )
    {
        config[5 * i]         = ( uint32_t ) config_lut[i].expected_power;
        config[( 5 * i ) + 1] = ( uint32_t ) config_lut[i].configured_power;
        config[( 5 * i ) + 2] = config_lut[i].pa_param1;
        config[( 5 * i ) + 3] = config_lut[i].pa_param2;
        config[( 5 * i ) + 4] = config_lut[i].pa_ramp_time;
    }
    return SMTC_MODEM_RC_OK;
}

#endif  // LR1110_MODEM_E

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static bool modem_port_reserved( uint8_t f_port )
{
    return ( f_port >= 224 );
}

static smtc_modem_return_code_t smtc_modem_get_dm_status_with_rate( uint8_t* dm_fields_payload,
                                                                    uint8_t* dm_field_length, e_dm_info_rate_t rate )
{
    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    *dm_field_length                     = get_dm_info_tag_list( dm_fields_payload, rate );
    return return_code;
}

static smtc_modem_return_code_t smtc_modem_set_dm_status_with_rate( const uint8_t* dm_fields_payload,
                                                                    uint8_t dm_field_length, e_dm_info_rate_t rate )
{
    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( set_dm_info( dm_fields_payload, dm_field_length, rate ) == SET_ERROR )
    {
        return_code = SMTC_MODEM_RC_INVALID;
    }

    return return_code;
}

static bool is_modem_connected( )
{
    bool ret = true;

    if( get_modem_muted( ) != MODEM_NOT_MUTE )
    {
        ret = false;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s Modem is muted for %d day(s)\n", __func__, dm_get_number_of_days_mute( ) );
    }
    else if( get_modem_suspend( ) == MODEM_SUSPEND )
    {
        ret = false;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s Modem is suspend\n", __func__ );
    }
    else if( ( get_join_state( ) != MODEM_JOINED ) && ( lorawan_api_get_activation_mode( ) == ACTIVATION_MODE_OTAA ) )
    {
        ret = false;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s Stack not joined or OTA device %d \n", __func__, get_join_state( ) );
    }
    return ( ret );
}

static smtc_modem_return_code_t smtc_modem_send_empty_tx( uint8_t f_port, bool f_port_present, bool confirmed )
{
    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    smodem_task              task_send;

    if( is_modem_connected( ) == false )
    {
        return_code = SMTC_MODEM_RC_FAIL;
    }
    else if( ( f_port_present == true ) &&
             ( ( ( ( f_port == 0 ) || ( f_port >= 224 ) ) && !lorawan_api_modem_certification_is_enabled( ) ) ||
               ( f_port == get_modem_dm_port( ) ) ) )
    {
        return_code = SMTC_MODEM_RC_INVALID;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s port %d is forbidden \n", __func__, f_port );
    }
    else
    {
        task_send.priority          = TASK_HIGH_PRIORITY;
        task_send.id                = SEND_TASK;
        task_send.fPort             = f_port;
        task_send.fPort_present     = f_port_present;
        task_send.PacketType        = confirmed;
        task_send.sizeIn            = 0;
        task_send.time_to_execute_s = smtc_modem_hal_get_time_in_s( );

        if( modem_supervisor_add_task( &task_send ) != TASK_VALID )
        {
            return_code = SMTC_MODEM_RC_FAIL;
        }
    }
    return return_code;
}

static smtc_modem_return_code_t smtc_modem_send_tx( uint8_t f_port, bool confirmed, const uint8_t* payload,
                                                    uint8_t payload_length, bool emergency )
{
    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    smodem_task              task_send;

    if( is_modem_connected( ) == false )
    {
        return_code = SMTC_MODEM_RC_FAIL;
    }

    else if( ( ( ( f_port == 0 ) || ( f_port >= 224 ) ) && !lorawan_api_modem_certification_is_enabled( ) ) ||
             ( f_port == get_modem_dm_port( ) ) )
    {
        return_code = SMTC_MODEM_RC_INVALID;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s port %d is forbidden \n", __func__, f_port );
    }
    else
    {
        if( emergency == true )
        {
            task_send.priority = TASK_VERY_HIGH_PRIORITY;
            lorawan_api_duty_cycle_enable_set( SMTC_DTC_PARTIAL_DISABLED );
        }
        else
        {
            task_send.priority = TASK_HIGH_PRIORITY;
        }

        memcpy( modem_buffer, payload, payload_length );

        task_send.id                = SEND_TASK;
        task_send.fPort             = f_port;
        task_send.fPort_present     = true;
        task_send.PacketType        = confirmed;
        task_send.dataIn            = modem_buffer;
        task_send.sizeIn            = payload_length;
        task_send.time_to_execute_s = smtc_modem_hal_get_time_in_s( );

        // SMTC_MODEM_HAL_TRACE_INFO( "add task user tx payload with payload size = %d \n ", payload_length );
        if( modem_supervisor_add_task( &task_send ) != TASK_VALID )
        {
            return_code = SMTC_MODEM_RC_FAIL;
        }
    }

    //@todo indicate that can't accept a new task because task already enqueued     //   SetModemBusy ();

    return return_code;
}

static smtc_modem_event_user_radio_access_status_t convert_rp_to_user_radio_access_status( rp_status_t rp_status )
{
    smtc_modem_event_user_radio_access_status_t user_radio_access_status = SMTC_MODEM_EVENT_USER_RADIO_ACCESS_UNKNOWN;

    switch( rp_status )
    {
    case RP_STATUS_RX_CRC_ERROR:
        user_radio_access_status = SMTC_MODEM_EVENT_USER_RADIO_ACCESS_RX_ERROR;
        break;
    case RP_STATUS_CAD_POSITIVE:
        user_radio_access_status = SMTC_MODEM_EVENT_USER_RADIO_ACCESS_CAD_OK;
        break;
    case RP_STATUS_CAD_NEGATIVE:
        user_radio_access_status = SMTC_MODEM_EVENT_USER_RADIO_ACCESS_CAD_DONE;
        break;
    case RP_STATUS_TX_DONE:
        user_radio_access_status = SMTC_MODEM_EVENT_USER_RADIO_ACCESS_TX_DONE;
        break;
    case RP_STATUS_RX_PACKET:
        user_radio_access_status = SMTC_MODEM_EVENT_USER_RADIO_ACCESS_RX_DONE;
        break;
    case RP_STATUS_RX_TIMEOUT:
        user_radio_access_status = SMTC_MODEM_EVENT_USER_RADIO_ACCESS_RX_TIMEOUT;
        break;
    case RP_STATUS_WIFI_SCAN_DONE:
        user_radio_access_status = SMTC_MODEM_EVENT_USER_RADIO_ACCESS_WIFI_SCAN_DONE;
        break;
    case RP_STATUS_GNSS_SCAN_DONE:
        user_radio_access_status = SMTC_MODEM_EVENT_USER_RADIO_ACCESS_GNSS_SCAN_DONE;
        break;
    default:
        user_radio_access_status = SMTC_MODEM_EVENT_USER_RADIO_ACCESS_UNKNOWN;
        break;
    }
    return user_radio_access_status;
}

/*
 * -----------------------------------------------------------------------------
 * --- CALLBACK FUNCTIONS DEFINITION -------------------------------------------
 */

void empty_callback( void* ctx )
{
}

void user_radio_access_callback( void* ctx )
{
    radio_planner_t* rp = ( radio_planner_t* ) ctx;
    rp_get_status( rp, RP_HOOK_ID_USER_SUSPEND, &user_radio_irq_timestamp, &user_radio_irq_status );

    switch( user_radio_irq_status )
    {
    case RP_STATUS_TASK_ABORTED:
        SMTC_MODEM_HAL_TRACE_INFO( "User radio access callback: ignored status %d\n", user_radio_irq_status );
        break;
    default:
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_USER_RADIO_ACCESS, 0 );
        break;
    }
}

/* --- EOF ------------------------------------------------------------------ */
