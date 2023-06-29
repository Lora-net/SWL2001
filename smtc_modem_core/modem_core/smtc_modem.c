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
#include "smtc_modem_middleware_advanced_api.h"
#include "smtc_modem_test_api.h"

#include "smtc_modem_hal_dbg_trace.h"
#include "modem_supervisor.h"
#include "modem_context.h"
#include "smtc_real_defs.h"
#include "lorawan_api.h"

#if defined( ADD_SMTC_ALC_SYNC )
#include "alc_sync.h"
#endif  // ADD_SMTC_ALC_SYNC

#if defined( ADD_SMTC_FILE_UPLOAD )
#include "file_upload.h"
#endif  // ADD_SMTC_FILE_UPLOAD

#if defined( ADD_SMTC_STREAM )
#include "stream.h"
#endif  // ADD_SMTC_STREAM

#include "radio_planner.h"
#include "ral.h"
#include "smtc_modem_utilities.h"
#include "modem_utilities.h"
#include "smtc_modem_crypto.h"
#include "lora_basics_modem_version.h"
#include "ralf.h"

#if defined( LR1110_MODEM_E )
// #include "smtc_modem_e_api_extension.h" // TODO GDG I removed but what about this ?
#include "smtc_basic_modem_lr11xx_api_extension.h"
#include "smtc_rtc_compensation.h"
#include "pool_mem.h"
#include "smtc_modem_e_internal_utilities.h"
// #include "smtc_crypto_se.h"              // TODO GDG I removed but what about this ?
#if defined( _MODEM_E_WIFI_ENABLE )
#include "wifi_ctrl_api.h"
#endif  // _MODEM_E_WIFI_ENABLE
#if defined( _MODEM_E_GNSS_ENABLE )
#include "gnss_ctrl_api.h"
#endif  //_MODEM_E_GNSS_ENABLE
#endif  // LR1110_MODEM_E

#if defined( LR11XX_TRANSCEIVER ) || defined( LR1110_MODEM_E )
#include "smtc_basic_modem_lr11xx_api_extension.h"
#endif  // LR11XX_TRANSCEIVER || LR1110_MODEM_E

#if defined( USE_LR11XX_CE )
#include "lr11xx_system.h"
#endif  // USE_LR11XX_CE

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

#if !defined( LR1110_MODEM_E )
#define MODEM_FW_VERSION_MAJOR LORA_BASICS_MODEM_FW_VERSION_MAJOR
#define MODEM_FW_VERSION_MINOR LORA_BASICS_MODEM_FW_VERSION_MINOR
#define MODEM_FW_VERSION_PATCH LORA_BASICS_MODEM_FW_VERSION_PATCH
#else
#define MODEM_FW_VERSION_MAJOR 1
#define MODEM_FW_VERSION_MINOR 1
#define MODEM_FW_VERSION_PATCH 8
#endif

#define MODEM_MAX_ALARM_VALUE_S ( 864000 )  // 10 days in seconds

/**
 * @brief Maximum payload size in byte of LoRaWAN payload
 */
#define SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH 242
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

// LBT configuration status
static bool lbt_config_available = false;

// user_radio_access
static rp_status_t user_radio_irq_status;
static uint32_t    user_radio_irq_timestamp;
static void ( *user_end_task_callback_0 )( smtc_modem_rp_status_t* status ) = NULL;
static void ( *user_end_task_callback_1 )( smtc_modem_rp_status_t* status ) = NULL;
static void ( *user_end_task_callback_2 )( smtc_modem_rp_status_t* status ) = NULL;

#ifdef LORAWAN_BYPASS_ENABLED
static bool stream_bypass_enabled = false;
#endif  // LORAWAN_BYPASS_ENABLED

#else  // !defined( LR1110_MODEM_E )

uint8_t modem_buffer[242];
#if defined( ADD_SMTC_FILE_UPLOAD )
static struct
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
#endif  // ADD_SMTC_FILE_UPLOAD

radio_planner_t       modem_radio_planner;
smtc_modem_services_t smtc_modem_services_ctx;

// user_radio_access
struct
{
    // LBT configuration status
    bool        lbt_config_available;
    rp_status_t user_radio_irq_status;
    uint32_t    user_radio_irq_timestamp;
    uint16_t    spare;
#if !defined( LR1110_MODEM_E )
    void ( *user_end_task_callback_0 )( smtc_modem_rp_status_t* status );
    void ( *user_end_task_callback_1 )( smtc_modem_rp_status_t* status );
    void ( *user_end_task_callback_2 )( smtc_modem_rp_status_t* status );
#endif  // !LR1110_MODEM_E
} smtc_modem_ctx;

#define lbt_config_available smtc_modem_ctx.lbt_config_available
#define user_radio_irq_status smtc_modem_ctx.user_radio_irq_status
#define user_radio_irq_timestamp smtc_modem_ctx.user_radio_irq_timestamp
#define user_end_task_callback_0 smtc_modem_ctx.user_end_task_callback_0
#define user_end_task_callback_1 smtc_modem_ctx.user_end_task_callback_1
#define user_end_task_callback_2 smtc_modem_ctx.user_end_task_callback_2
#endif  // !defined( LR1110_MODEM_E )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static bool modem_port_reserved( uint8_t f_port );

static smtc_modem_return_code_t smtc_modem_get_dm_status_with_rate( uint8_t* dm_fields_payload,
                                                                    uint8_t* dm_field_length, dm_info_rate_t rate );

static smtc_modem_return_code_t smtc_modem_set_dm_status_with_rate( const uint8_t* dm_fields_payload,
                                                                    uint8_t dm_field_length, dm_info_rate_t rate );

static bool is_modem_connected( );

static smtc_modem_return_code_t smtc_modem_send_empty_tx( uint8_t f_port, bool f_port_present, bool confirmed );

static smtc_modem_return_code_t smtc_modem_send_tx( uint8_t f_port, bool confirmed, const uint8_t* payload,
                                                    uint8_t payload_length, bool emergency, uint8_t tx_buffer_id );

smtc_modem_event_user_radio_access_status_t convert_rp_to_user_radio_access_status( rp_status_t rp_status );
smtc_modem_rp_radio_status_t                convert_rp_to_user_radio_access_rp_status( rp_status_t rp_status );

void empty_callback( void* ctx );
void user_radio_access_callback( void* ctx );

#if !defined( LR1110_MODEM_E )
void callback_rp_user_radio_access_0( void* ctx );
void callback_rp_user_radio_access_1( void* ctx );
void callback_rp_user_radio_access_2( void* ctx );
#endif  // !LR1110_MODEM_E

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/* ------------ Modem Utilities ------------*/
smtc_modem_event_user_radio_access_status_t convert_rp_to_user_radio_access_status( rp_status_t rp_status );

void smtc_modem_init( const ralf_t* radio, void ( *callback_event )( void ) )
{
    SMTC_MODEM_HAL_TRACE_INFO( "Modem Initialization\n" );

#ifdef LORAWAN_BYPASS_ENABLED
    stream_bypass_enabled = false;
#endif
    // init radio and put it in sleep mode
    ral_reset( &( radio->ral ) );
    ral_init( &( radio->ral ) );
    ral_set_sleep( &( radio->ral ), true );

    // Save modem radio context in case of direct access to radio by the modem
    modem_context_set_modem_radio_ctx( radio->ral.context );

    // init radio planner and attach corresponding radio irq
    rp_init( &modem_radio_planner, radio );

#if !defined( LR1110_MODEM_E )
    smtc_modem_hal_irq_config_radio_irq( rp_radio_irq_callback, &modem_radio_planner );
#endif

    // init modem supervisor

#if defined( LR1110_MODEM_E )
    smtc_rtc_compensation_init( &modem_radio_planner, RP_HOOK_ID_RTC_COMPENSATION );
#endif
    rp_hook_init( &modem_radio_planner, RP_HOOK_ID_SUSPEND, ( void ( * )( void* ) )( empty_callback ),
                  &modem_radio_planner );
#if !defined( LR1110_MODEM_E )
    rp_hook_init( &modem_radio_planner, RP_HOOK_ID_USER_SUSPEND, ( void ( * )( void* ) )( user_radio_access_callback ),
                  &modem_radio_planner ); /* user_radio_access_callback called when interrupt occurs */
    rp_hook_init( &modem_radio_planner, RP_HOOK_ID_USER_SUSPEND_0,
                  ( void ( * )( void* ) )( callback_rp_user_radio_access_0 ), &modem_radio_planner );
    rp_hook_init( &modem_radio_planner, RP_HOOK_ID_USER_SUSPEND_1,
                  ( void ( * )( void* ) )( callback_rp_user_radio_access_1 ), &modem_radio_planner );
    rp_hook_init( &modem_radio_planner, RP_HOOK_ID_USER_SUSPEND_2,
                  ( void ( * )( void* ) )( callback_rp_user_radio_access_2 ), &modem_radio_planner );
#endif  // !LR1110_MODEM_E
    modem_supervisor_init( callback_event, &modem_radio_planner, &smtc_modem_services_ctx );
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

        // SMTC_MODEM_HAL_TRACE_PRINTF( "Event ID: %d, Missed: %d\n", event->event_type, event->missed_events );

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

            event->event_data.downdata.snr          = metadata.rx_snr << 2;
            event->event_data.downdata.window       = ( smtc_modem_event_downdata_window_t ) metadata.rx_window;
            event->event_data.downdata.fport        = metadata.rx_fport;
            event->event_data.downdata.fpending_bit = metadata.rx_fpending_bit;
            event->event_data.downdata.frequency_hz = metadata.rx_frequency_hz;
            event->event_data.downdata.datarate     = metadata.rx_datarate;
            break;
        }
#if defined( ADD_SMTC_FILE_UPLOAD )
        case SMTC_MODEM_EVENT_UPLOADDONE:
            event->event_data.uploaddone.status =
                ( smtc_modem_event_uploaddone_status_t ) get_modem_event_status( event->event_type );
            break;
#endif  // ADD_SMTC_FILE_UPLOAD
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
            event->event_data.wifi_event_status.data_length = size;
            memcpy( event->event_data.wifi_event_status.data,
                    ( uint8_t* ) ( &( POOL_MEM.WIFI_MEM.Pool_mem.Buffer_tx.EventModem[0] ) + 4 ), size );
        }
        break;
#endif
#if defined( _MODEM_E_GNSS_ENABLE ) && defined( LR1110_MODEM_E )
        case SMTC_MODEM_EVENT_GNSS: {
            uint16_t size                                   = GnssGetSize( );
            event->event_data.gnss_event_status.data_length = size;
            memcpy( event->event_data.gnss_event_status.data, ( uint8_t* ) &( POOL_MEM.GNSS_MEM.Buf_data[0] ), size );
        }
        break;
#endif
        case SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO:
            event->event_data.class_b_ping_slot_info.status =
                ( smtc_modem_event_class_b_ping_slot_status_t ) get_modem_event_status( event->event_type );
            break;
        case SMTC_MODEM_EVENT_CLASS_B_STATUS:
            event->event_data.class_b_status.status =
                ( smtc_modem_event_class_b_status_t ) get_modem_event_status( event->event_type );
            break;
#if defined( SMTC_D2D )
        case SMTC_MODEM_EVENT_D2D_CLASS_B_TX_DONE: {
            modem_context_class_b_d2d_t class_b_d2d;
            modem_context_get_class_b_d2d_last_metadata( &class_b_d2d );
            event->event_data.d2d_class_b_tx_done.mc_grp_id         = class_b_d2d.mc_grp_id;
            event->event_data.d2d_class_b_tx_done.nb_trans_not_send = class_b_d2d.nb_trans_not_send;
            event->event_data.d2d_class_b_tx_done.status =
                ( smtc_modem_d2d_class_b_tx_done_status_t ) get_modem_event_status( event->event_type );
            break;
        }
#endif  // SMTC_D2D
        case SMTC_MODEM_EVENT_MIDDLEWARE_1:
        case SMTC_MODEM_EVENT_MIDDLEWARE_2:
        case SMTC_MODEM_EVENT_MIDDLEWARE_3:
            event->event_data.middleware_event_status.status = get_modem_event_status( event->event_type );
            break;
        case SMTC_MODEM_EVENT_ALARM:
        case SMTC_MODEM_EVENT_JOINED:
#if defined( ADD_SMTC_STREAM )
        case SMTC_MODEM_EVENT_STREAMDONE:
#endif  // ADD_SMTC_STREAM
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

    firmware_version->major = MODEM_FW_VERSION_MAJOR;
    firmware_version->minor = MODEM_FW_VERSION_MINOR;
    firmware_version->patch = MODEM_FW_VERSION_PATCH;

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
#if defined( ADD_SMTC_ALC_SYNC )
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
#else   //  ADD_SMTC_ALC_SYNC
    return SMTC_MODEM_RC_FAIL;
#endif  //  ADD_SMTC_ALC_SYNC
}

smtc_modem_return_code_t smtc_modem_time_stop_sync_service( uint8_t stack_id )
{
#if defined( ADD_SMTC_ALC_SYNC )
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
#else   //  ADD_SMTC_ALC_SYNC
    return SMTC_MODEM_RC_FAIL;
#endif  //  ADD_SMTC_ALC_SYNC
}

smtc_modem_return_code_t smtc_modem_get_time( uint32_t* gps_time_s, uint32_t* gps_fractional_s )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( gps_time_s );
    RETURN_INVALID_IF_NULL( gps_fractional_s );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

#if defined( ADD_SMTC_ALC_SYNC )
    if( clock_sync_is_time_valid( &( smtc_modem_services_ctx.clock_sync_ctx ) ) == true )
    {
        clock_sync_get_gps_time_second( &( smtc_modem_services_ctx.clock_sync_ctx ), gps_time_s, gps_fractional_s );
    }
    else
    {
        *gps_time_s       = 0;
        *gps_fractional_s = 0;
        return_code       = SMTC_MODEM_RC_NO_TIME;
    }

#else   //  ADD_SMTC_ALC_SYNC
    if( lorawan_api_convert_rtc_to_gps_epoch_time( smtc_modem_hal_get_time_in_ms( ), gps_time_s, gps_fractional_s ) ==
        true )
    {
        return_code = SMTC_MODEM_RC_OK;
    }
    else
    {
        *gps_time_s       = 0;
        *gps_fractional_s = 0;
        return_code       = SMTC_MODEM_RC_NO_TIME;
    }
#endif  //  ADD_SMTC_ALC_SYNC
    return return_code;
}

smtc_modem_return_code_t smtc_modem_time_set_alcsync_fport( uint8_t clock_sync_fport )
{
#if defined( ADD_SMTC_ALC_SYNC )
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( clock_sync_set_alcsync_port( &( smtc_modem_services_ctx.clock_sync_ctx ), clock_sync_fport ) != 0 )
    {
        return_code = SMTC_MODEM_RC_INVALID;
    }

    return return_code;
#else   //  ADD_SMTC_ALC_SYNC
    return SMTC_MODEM_RC_FAIL;
#endif  //  ADD_SMTC_ALC_SYNC
}

smtc_modem_return_code_t smtc_modem_time_trigger_sync_request( uint8_t stack_id )
{
#if defined( ADD_SMTC_ALC_SYNC )
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    if( is_modem_connected( ) == false )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    // Check if a time sync service is enabled
    if( clock_sync_is_enabled( &( smtc_modem_services_ctx.clock_sync_ctx ) ) == true )
    {
        modem_supervisor_add_task_clock_sync_time_req( 1 );
    }
    else
    {
        return_code = SMTC_MODEM_RC_FAIL;
    }

    return return_code;
#else   //  ADD_SMTC_ALC_SYNC
    modem_supervisor_add_task_device_time_req( 1 );
    return SMTC_MODEM_RC_OK;
#endif  //  ADD_SMTC_ALC_SYNC
}

smtc_modem_return_code_t smtc_modem_time_get_alcsync_fport( uint8_t* clock_sync_port )
{
#if defined( ADD_SMTC_ALC_SYNC )
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( clock_sync_port );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    *clock_sync_port                     = clock_sync_get_alcsync_port( &( smtc_modem_services_ctx.clock_sync_ctx ) );

    return return_code;
#else   //  ADD_SMTC_ALC_SYNC
    return SMTC_MODEM_RC_FAIL;
#endif  //  ADD_SMTC_ALC_SYNC
}

smtc_modem_return_code_t smtc_modem_time_set_sync_interval_s( uint32_t sync_interval_s )
{
#if defined( ADD_SMTC_ALC_SYNC )
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    if( clock_sync_set_interval_second( &( smtc_modem_services_ctx.clock_sync_ctx ), sync_interval_s ) !=
        CLOCK_SYNC_OK )
    {
        return_code = SMTC_MODEM_RC_INVALID;
    }

    return return_code;
#else   //  ADD_SMTC_ALC_SYNC
    return SMTC_MODEM_RC_FAIL;
#endif  //  ADD_SMTC_ALC_SYNC
}
smtc_modem_return_code_t smtc_modem_time_get_sync_interval_s( uint32_t* sync_interval_s )
{
#if defined( ADD_SMTC_ALC_SYNC )
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( sync_interval_s );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    *sync_interval_s = clock_sync_get_interval_second( &( smtc_modem_services_ctx.clock_sync_ctx ) );
    return return_code;
#else   //  ADD_SMTC_ALC_SYNC
    return SMTC_MODEM_RC_FAIL;
#endif  //  ADD_SMTC_ALC_SYNC
}

smtc_modem_return_code_t smtc_modem_time_set_sync_invalid_delay_s( uint32_t sync_invalid_delay_s )
{
#if defined( ADD_SMTC_ALC_SYNC )
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( clock_sync_set_invalid_time_delay_s( &( smtc_modem_services_ctx.clock_sync_ctx ), sync_invalid_delay_s ) !=
        CLOCK_SYNC_OK )
    {
        return_code = SMTC_MODEM_RC_INVALID;
    }

    return return_code;
#else   //  ADD_SMTC_ALC_SYNC

    if( lorawan_api_set_device_time_invalid_delay_s( sync_invalid_delay_s ) != OKLORAWAN )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
#endif  //  ADD_SMTC_ALC_SYNC
}

smtc_modem_return_code_t smtc_modem_time_get_sync_invalid_delay_s( uint32_t* sync_invalid_delay_s )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( sync_invalid_delay_s );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

#if defined( ADD_SMTC_ALC_SYNC )
    *sync_invalid_delay_s = clock_sync_get_invalid_time_delay_s( &( smtc_modem_services_ctx.clock_sync_ctx ) );
#else   //  ADD_SMTC_ALC_SYNC
    *sync_invalid_delay_s = lorawan_api_get_device_time_invalid_delay_s( );
#endif  //  ADD_SMTC_ALC_SYNC

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
    if( alarm_s > MODEM_MAX_ALARM_VALUE_S )
    {
        return SMTC_MODEM_RC_INVALID;
    }

    modem_set_user_alarm( ( alarm_s > 0 ) ? ( smtc_modem_hal_get_time_in_s( ) + alarm_s ) : 0 );
    return SMTC_MODEM_RC_OK;
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

    if( lorawan_api_get_joineui( joineui ) != OKLORAWAN )
    {
        return_code = SMTC_MODEM_RC_FAIL;
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_joineui( uint8_t stack_id, const uint8_t joineui[SMTC_MODEM_EUI_LENGTH] )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( joineui );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    // Check join status: the modem shall not be joined to modify the joineui
    if( get_join_state( ) != MODEM_NOT_JOINED )
    {
        return_code = SMTC_MODEM_RC_BUSY;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else
    {
        if( lorawan_api_set_joineui( joineui ) != OKLORAWAN )
        {
            return_code = SMTC_MODEM_RC_FAIL;
        }
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_deveui( uint8_t stack_id, uint8_t deveui[SMTC_MODEM_EUI_LENGTH] )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( deveui );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    if( lorawan_api_get_deveui( deveui ) != OKLORAWAN )
    {
        return_code = SMTC_MODEM_RC_FAIL;
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_deveui( uint8_t stack_id, const uint8_t deveui[SMTC_MODEM_EUI_LENGTH] )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( deveui );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    // Check join status: the modem shall not be joined to modify the deveui
    if( get_join_state( ) != MODEM_NOT_JOINED )
    {
        return_code = SMTC_MODEM_RC_BUSY;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else
    {
        if( lorawan_api_set_deveui( deveui ) != OKLORAWAN )
        {
            return_code = SMTC_MODEM_RC_FAIL;
        }
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_nwkkey( uint8_t stack_id, const uint8_t nwkkey[SMTC_MODEM_KEY_LENGTH] )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( nwkkey );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    // Check join status: the modem shall not be joined to modify the key
    if( get_join_state( ) != MODEM_NOT_JOINED )
    {
        return_code = SMTC_MODEM_RC_BUSY;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else
    {
        if( modem_context_set_appkey( nwkkey ) != MODEM_CTX_RC_SUCCESS )
        {
            return_code = SMTC_MODEM_RC_FAIL;
        }
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

    switch( lorawan_class )
    {
    case SMTC_MODEM_CLASS_A: {
        lorawan_api_class_b_enabled( false );
        lorawan_api_class_c_enabled( false );
        break;
    }
    case SMTC_MODEM_CLASS_B: {
#if defined( ADD_SMTC_ALC_SYNC )
        if( clock_sync_is_time_valid( &( smtc_modem_services_ctx.clock_sync_ctx ) ) == false )

        {
            SMTC_MODEM_HAL_TRACE_ERROR( "set to class b is refused : modem is not time synced" );
            return ( SMTC_MODEM_RC_FAIL );
        }
#else   // ADD_SMTC_ALC_SYNC
        if( lorawan_api_is_time_valid( ) == false )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "set to class b is refused : modem is not time synced" );
            return ( SMTC_MODEM_RC_FAIL );
        }
#endif  // ADD_SMTC_ALC_SYNC
        if( get_join_state( ) != MODEM_JOINED )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "set to class b is refused : modem is not joined" );
            return ( SMTC_MODEM_RC_FAIL );
        }
        lorawan_api_class_b_enabled( true );
        lorawan_api_class_c_enabled( false );
        break;
    }
    case SMTC_MODEM_CLASS_C: {
        lorawan_api_class_b_enabled( false );
        lorawan_api_class_c_enabled( true );
        break;
    }
    default:
        return SMTC_MODEM_RC_INVALID;
        break;
    }
    set_modem_class( lorawan_class );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_multicast_set_grp_config( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                              uint32_t      mc_grp_addr,
                                                              const uint8_t mc_nwk_skey[SMTC_MODEM_KEY_LENGTH],
                                                              const uint8_t mc_app_skey[SMTC_MODEM_KEY_LENGTH] )
{
#if defined( SMTC_MULTICAST )
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( mc_nwk_skey );
    RETURN_INVALID_IF_NULL( mc_app_skey );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_set_group_address( mc_grp_id, mc_grp_addr );

    if( rc == LORAWAN_MC_RC_OK )
    {
        rc = lorawan_api_multicast_set_group_session_keys( mc_grp_id, mc_nwk_skey, mc_app_skey );
    }

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
#else   // SMTC_MULTICAST
    return SMTC_MODEM_RC_FAIL;
#endif  // SMTC_MULTICAST
}

smtc_modem_return_code_t smtc_modem_multicast_get_grp_config( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                              uint32_t* mc_grp_addr )
{
#if defined( SMTC_MULTICAST )
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( mc_grp_addr );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_get_group_address( mc_grp_id, mc_grp_addr );

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
#else   // SMTC_MULTICAST
    return SMTC_MODEM_RC_FAIL;
#endif  // SMTC_MULTICAST
}

smtc_modem_return_code_t smtc_modem_multicast_class_c_start_session( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                                     uint32_t freq, uint8_t dr )
{
#if defined( SMTC_MULTICAST )
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_c_start_session( mc_grp_id, freq, dr );

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
    case LORAWAN_MC_RC_ERROR_BUSY:
        // intentional fallthrought
    case LORAWAN_MC_RC_ERROR_CLASS_NOT_ENABLED:
        // intentional fallthrought
    default:
        modem_rc = SMTC_MODEM_RC_FAIL;
        break;
    }
    return modem_rc;
#else   // SMTC_MULTICAST
    return SMTC_MODEM_RC_FAIL;
#endif  // SMTC_MULTICAST
}

smtc_modem_return_code_t smtc_modem_multicast_class_c_get_session_status( uint8_t                stack_id,
                                                                          smtc_modem_mc_grp_id_t mc_grp_id,
                                                                          bool* is_session_started, uint32_t* freq,
                                                                          uint8_t* dr )
{
#if defined( SMTC_MULTICAST )
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( is_session_started );
    RETURN_INVALID_IF_NULL( freq );
    RETURN_INVALID_IF_NULL( dr );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_c_get_session_status( mc_grp_id, is_session_started, freq, dr );

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
#else   // SMTC_MULTICAST
    return SMTC_MODEM_RC_FAIL;
#endif  // SMTC_MULTICAST
}

smtc_modem_return_code_t smtc_modem_multicast_class_c_stop_session( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id )
{
#if defined( SMTC_MULTICAST )
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_c_stop_session( mc_grp_id );

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
#else   // SMTC_MULTICAST
    return SMTC_MODEM_RC_FAIL;
#endif  // SMTC_MULTICAST
}

smtc_modem_return_code_t smtc_modem_multicast_class_c_stop_all_sessions( uint8_t stack_id )
{
#if defined( SMTC_MULTICAST )
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_c_stop_all_sessions( );

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
#else   // SMTC_MULTICAST
    return SMTC_MODEM_RC_FAIL;
#endif  // SMTC_MULTICAST
}

smtc_modem_return_code_t smtc_modem_multicast_class_b_start_session(
    uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id, uint32_t freq, uint8_t dr,
    smtc_modem_class_b_ping_slot_periodicity_t ping_slot_periodicity )
{
#if defined( SMTC_MULTICAST )
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_b_start_session( mc_grp_id, freq, dr, ping_slot_periodicity );

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
    case LORAWAN_MC_RC_ERROR_BUSY:
        // intentional fallthrought
    case LORAWAN_MC_RC_ERROR_CLASS_NOT_ENABLED:
        // intentional fallthrought
    default:
        modem_rc = SMTC_MODEM_RC_FAIL;
        break;
    }
    return modem_rc;
#else   // SMTC_MULTICAST
    return SMTC_MODEM_RC_FAIL;
#endif  // SMTC_MULTICAST
}

smtc_modem_return_code_t smtc_modem_multicast_class_b_get_session_status(
    uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id, bool* is_session_started, bool* is_session_waiting_for_beacon,
    uint32_t* freq, uint8_t* dr, smtc_modem_class_b_ping_slot_periodicity_t* ping_slot_periodicity )

{
#if defined( SMTC_MULTICAST )
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( is_session_started );
    RETURN_INVALID_IF_NULL( is_session_waiting_for_beacon );
    RETURN_INVALID_IF_NULL( freq );
    RETURN_INVALID_IF_NULL( dr );
    RETURN_INVALID_IF_NULL( ping_slot_periodicity );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_b_get_session_status(
        mc_grp_id, is_session_started, is_session_waiting_for_beacon, freq, dr, ping_slot_periodicity );

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
#else   // SMTC_MULTICAST
    return SMTC_MODEM_RC_FAIL;
#endif  // SMTC_MULTICAST
}

smtc_modem_return_code_t smtc_modem_multicast_class_b_stop_session( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id )
{
#if defined( SMTC_MULTICAST )
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_b_stop_session( mc_grp_id );

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
#else   // SMTC_MULTICAST
    return SMTC_MODEM_RC_FAIL;
#endif  // SMTC_MULTICAST
}

smtc_modem_return_code_t smtc_modem_multicast_class_b_stop_all_sessions( uint8_t stack_id )
{
#if defined( SMTC_MULTICAST )
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_b_stop_all_sessions( );

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
#else   // SMTC_MULTICAST
    return SMTC_MODEM_RC_FAIL;
#endif  // SMTC_MULTICAST
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

    if( get_join_state( ) != MODEM_NOT_JOINED )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
        return SMTC_MODEM_RC_BUSY;
    }

    if( set_modem_region( region ) == DM_ERROR )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call with region not valid\n", __func__ );
        return SMTC_MODEM_RC_INVALID;
    }

    return SMTC_MODEM_RC_OK;
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
    dm_rc_t                  status;

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
        status = DM_ERROR;
    }
    if( ( adr_profile == SMTC_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE ) ||
        ( adr_profile == SMTC_MODEM_ADR_PROFILE_MOBILE_LOW_POWER ) || ( adr_profile == SMTC_MODEM_ADR_PROFILE_CUSTOM ) )
    {
        // reset current adr mobile count
        lorawan_api_reset_no_rx_packet_in_mobile_mode_cnt( );
    }
    if( status == DM_ERROR )
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
    if( set_modem_dm_port( dm_fport ) == DM_ERROR )
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

        if( set_modem_dm_interval( modem_interval ) == DM_ERROR )
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

    modem_leave( );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_suspend_radio_communications( bool suspend )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( set_modem_suspend( suspend ) == DM_ERROR )
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
        *tx_max_payload_size = 0;
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
    return_code                          = smtc_modem_send_tx( f_port, confirmed, payload, payload_length, false, 0 );
    return return_code;
}
smtc_modem_return_code_t smtc_modem_request_extended_uplink( uint8_t stack_id, uint8_t f_port, bool confirmed,
                                                             const uint8_t* payload, uint8_t payload_length,
                                                             uint8_t extended_uplink_id,
                                                             void ( *lbm_notification_callback )( void ) )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( payload );
    RETURN_INVALID_IF_NULL( lbm_notification_callback );
    if( extended_uplink_id == 1 )
    {
#ifndef TASK_EXTENDED_1
        return SMTC_MODEM_RC_INVALID;
#endif
    }
    if( extended_uplink_id == 2 )
    {
#ifndef TASK_EXTENDED_2
        return SMTC_MODEM_RC_INVALID;
#endif
    }

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( ( extended_uplink_id == 1 ) || ( extended_uplink_id == 2 ) )
    {
        modem_set_extended_callback( lbm_notification_callback, extended_uplink_id );
        return_code = smtc_modem_send_tx( f_port, confirmed, payload, payload_length, false, extended_uplink_id );
    }
    else
    {
        return_code = SMTC_MODEM_RC_INVALID;
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_abort_extended_uplink( uint8_t stack_id, uint8_t extended_uplink_id )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    if( extended_uplink_id == 1 )
    {
#ifndef TASK_EXTENDED_1
        return SMTC_MODEM_RC_INVALID;
#endif
    }
    if( extended_uplink_id == 2 )
    {
#ifndef TASK_EXTENDED_2
        return SMTC_MODEM_RC_INVALID;
#endif
    }
    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( extended_uplink_id == 1 )
    {
        modem_supervisor_remove_task( SEND_TASK_EXTENDED_1 );
    }
    else if( extended_uplink_id == 2 )
    {
        modem_supervisor_remove_task( SEND_TASK_EXTENDED_2 );
    }
    else
    {
        return_code = SMTC_MODEM_RC_INVALID;
    }
    return return_code;
}
smtc_modem_return_code_t smtc_modem_request_emergency_uplink( uint8_t stack_id, uint8_t f_port, bool confirmed,
                                                              const uint8_t* payload, uint8_t payload_length )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( payload );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    return_code                          = smtc_modem_send_tx( f_port, confirmed, payload, payload_length, true, 0 );
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
#if defined( ADD_SMTC_FILE_UPLOAD )
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
#else   // ADD_SMTC_FILE_UPLOAD
    return SMTC_MODEM_RC_FAIL;
#endif  // ADD_SMTC_FILE_UPLOAD
}

smtc_modem_return_code_t smtc_modem_file_upload_start( uint8_t stack_id )
{
#if defined( ADD_SMTC_FILE_UPLOAD )
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
#else   // ADD_SMTC_FILE_UPLOAD
    return SMTC_MODEM_RC_FAIL;
#endif  // ADD_SMTC_FILE_UPLOAD
}

smtc_modem_return_code_t smtc_modem_file_upload_reset( uint8_t stack_id )
{
#if defined( ADD_SMTC_FILE_UPLOAD )
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
#else   // ADD_SMTC_FILE_UPLOAD
    return SMTC_MODEM_RC_FAIL;
#endif  // ADD_SMTC_FILE_UPLOAD
}

smtc_modem_return_code_t smtc_modem_stream_init( uint8_t stack_id, uint8_t fport,
                                                 smtc_modem_stream_cipher_mode_t cipher_mode,
                                                 uint8_t                         redundancy_ratio_percent )
{
#if defined( ADD_SMTC_STREAM )
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
#else   // ADD_SMTC_STREAM
    return SMTC_MODEM_RC_FAIL;
#endif  // ADD_SMTC_STREAM
}

smtc_modem_return_code_t smtc_modem_stream_add_data( uint8_t stack_id, const uint8_t* data, uint8_t len )
{
#if defined( ADD_SMTC_STREAM )
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
#else   // ADD_SMTC_STREAM
    return SMTC_MODEM_RC_FAIL;
#endif  // ADD_SMTC_STREAM
}

smtc_modem_return_code_t smtc_modem_stream_status( uint8_t stack_id, uint16_t* pending, uint16_t* free )
{
#if defined( ADD_SMTC_STREAM )
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
#else   // ADD_SMTC_STREAM
    return SMTC_MODEM_RC_FAIL;
#endif  // ADD_SMTC_STREAM
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

smtc_modem_return_code_t smtc_modem_increment_event_middleware( uint8_t event_type, uint8_t status )
{
    if( ( event_type < SMTC_MODEM_EVENT_MIDDLEWARE_1 ) || ( event_type > SMTC_MODEM_EVENT_MIDDLEWARE_3 ) )
    {
        return SMTC_MODEM_RC_INVALID;
    }
    increment_asynchronous_msgnumber( event_type, status );
    return SMTC_MODEM_RC_OK;
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
    lorawan_api_set_no_rx_packet_threshold( nb_of_uplinks_before_reset );

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
    *nb_of_uplinks_before_reset              = lorawan_api_get_no_rx_packet_threshold( );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_connection_timeout_get_current_values(
    uint8_t stack_id, uint16_t* nb_of_uplinks_before_network_controlled, uint16_t* nb_of_uplinks_before_reset )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( nb_of_uplinks_before_network_controlled );
    RETURN_INVALID_IF_NULL( nb_of_uplinks_before_reset );

    *nb_of_uplinks_before_network_controlled = lorawan_api_get_current_no_rx_packet_in_mobile_mode_cnt( );
    *nb_of_uplinks_before_reset              = lorawan_api_get_current_adr_ack_cnt( );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_lorawan_get_lost_connection_counter( uint8_t   stack_id,
                                                                         uint16_t* lost_connection_cnt )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( lost_connection_cnt );
    *lost_connection_cnt = lorawan_api_get_current_no_rx_packet_cnt( );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_duty_cycle_status( int32_t* duty_cycle_status_ms )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( duty_cycle_status_ms );

    *duty_cycle_status_ms = -1 * lorawan_api_next_free_duty_cycle_ms_get( );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_rp_abort_user_radio_access_task( uint8_t user_task_id )
{
#if !defined( LR1110_MODEM_E )
    rp_hook_status_t status = RP_HOOK_STATUS_ID_ERROR;
    switch( user_task_id )
    {
    case SMTC_MODEM_RP_TASK_ID0:
        status = rp_task_abort( &modem_radio_planner, RP_HOOK_ID_USER_SUSPEND_0 );
        break;
    case SMTC_MODEM_RP_TASK_ID1:
        status = rp_task_abort( &modem_radio_planner, RP_HOOK_ID_USER_SUSPEND_1 );
        break;
    case SMTC_MODEM_RP_TASK_ID2:
        status = rp_task_abort( &modem_radio_planner, RP_HOOK_ID_USER_SUSPEND_2 );
        break;
    default:
        return SMTC_MODEM_RC_INVALID;
        break;
    }
    return ( status == RP_HOOK_STATUS_OK ) ? SMTC_MODEM_RC_OK : SMTC_MODEM_RC_FAIL;
#else   // !LR1110_MODEM_E
    return SMTC_MODEM_RC_FAIL;
#endif  // !LR1110_MODEM_E
}

smtc_modem_return_code_t smtc_modem_rp_add_user_radio_access_task( smtc_modem_rp_task_t* rp_task )
{
#if !defined( LR1110_MODEM_E )
    RETURN_BUSY_IF_TEST_MODE( );

    rp_radio_params_t fake_radio_params = { 0 };
    uint8_t           user_hook_id_temp = 0;
    switch( rp_task->id )
    {
    case SMTC_MODEM_RP_TASK_ID0:
        user_end_task_callback_0 = rp_task->end_task_callback;
        user_hook_id_temp        = RP_HOOK_ID_USER_SUSPEND_0;
        break;
    case SMTC_MODEM_RP_TASK_ID1:
        user_end_task_callback_1 = rp_task->end_task_callback;
        user_hook_id_temp        = RP_HOOK_ID_USER_SUSPEND_1;
        break;
    case SMTC_MODEM_RP_TASK_ID2:
        user_end_task_callback_2 = rp_task->end_task_callback;
        user_hook_id_temp        = RP_HOOK_ID_USER_SUSPEND_2;
        break;
    default:
        return SMTC_MODEM_RC_INVALID;
        break;
    }

    rp_task_t rp_task_tmp = { .hook_id               = user_hook_id_temp,
                              .launch_task_callbacks = rp_task->launch_task_callback,
                              .duration_time_ms      = rp_task->duration_time_ms,
                              .state = ( rp_task->type == SMTC_MODEM_RP_TASK_STATE_SCHEDULE ) ? RP_TASK_STATE_SCHEDULE
                                                                                              : RP_TASK_STATE_ASAP,
                              .schedule_task_low_priority = false,
                              .start_time_ms              = rp_task->start_time_ms };

    rp_hook_status_t status = rp_task_enqueue( &modem_radio_planner, &rp_task_tmp, NULL, 0, &fake_radio_params );

    return ( status == RP_HOOK_STATUS_OK ) ? SMTC_MODEM_RC_OK : SMTC_MODEM_RC_FAIL;
#else   // !LR1110_MODEM_E
    return SMTC_MODEM_RC_FAIL;
#endif  // !LR1110_MODEM_E
}

smtc_modem_return_code_t smtc_modem_suspend_before_user_radio_access( void )
{
#if !defined( LR1110_MODEM_E )
    RETURN_BUSY_IF_TEST_MODE( );

    // Put modem in suspended mode to prevent scheduler to be called
    smtc_modem_suspend_radio_communications( true );

    SMTC_MODEM_HAL_TRACE_PRINTF( "Suspend modem user radio access\n" );

    // Protect radio access with a suspension of all other task in radio planner (put an infinite empty task)
    return ( modem_context_suspend_user_radio_access( RP_TASK_TYPE_NONE ) == true ) ? SMTC_MODEM_RC_OK
                                                                                    : SMTC_MODEM_RC_FAIL;
#else   // !LR1110_MODEM_E
    return SMTC_MODEM_RC_FAIL;
#endif  // !LR1110_MODEM_E
}

smtc_modem_return_code_t smtc_modem_resume_after_user_radio_access( void )
{
#if !defined( LR1110_MODEM_E )
    RETURN_BUSY_IF_TEST_MODE( );

    SMTC_MODEM_HAL_TRACE_PRINTF( "Resume modem user radio access\n" );

    // First stop the radio_planner suspension (always RC_OK)
    modem_context_resume_user_radio_access( );

    // Then put the modem in NOT_SUSPENDED mode to relaunch the scheduler (always RC_OK)
    smtc_modem_suspend_radio_communications( false );

    return SMTC_MODEM_RC_OK;
#else   // !LR1110_MODEM_E
    return SMTC_MODEM_RC_FAIL;
#endif  // !LR1110_MODEM_E
}

smtc_modem_return_code_t smtc_modem_get_stack_state( uint8_t stack_id, smtc_modem_stack_state_t* stack_state )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( stack_state );

    lr1mac_states_t lr1mac_state = lorawan_api_state_get( );
    if( lr1mac_state == LWPSTATE_IDLE )
    {
        *stack_state = SMTC_MODEM_STACK_STATE_IDLE;
    }
    else if( lr1mac_state == LWPSTATE_TX_WAIT )
    {
        *stack_state = SMTC_MODEM_STACK_STATE_TX_WAIT;
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

    lorawan_api_lbt_set_parameters( listen_duration_ms, threshold_dbm, bw_hz );

    lbt_config_available = true;
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

    lorawan_api_lbt_get_parameters( listen_duration_ms, threshold_dbm, bw_hz );

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
            lorawan_api_lbt_set_state( true );
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
        lorawan_api_lbt_set_state( false );
        return SMTC_MODEM_RC_OK;
    }
}

smtc_modem_return_code_t smtc_modem_lbt_get_state( uint8_t stack_id, bool* enabled )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( enabled );

    *enabled = lorawan_api_lbt_get_state( );
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

smtc_modem_return_code_t smtc_modem_set_crystal_error_ppm( uint32_t crystal_error_ppm )
{
    RETURN_BUSY_IF_TEST_MODE( );

    lorawan_api_set_crystal_error( crystal_error_ppm );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_crystal_error_ppm( uint32_t* crystal_error_ppm )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( crystal_error_ppm );

    *crystal_error_ppm = lorawan_api_get_crystal_error( );
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

smtc_modem_return_code_t smtc_modem_lorawan_class_b_request_ping_slot_info( uint8_t stack_id )
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
        modem_supervisor_add_task_ping_slot_info_req( 0 );
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_class_b_set_ping_slot_periodicity(
    uint8_t stack_id, smtc_modem_class_b_ping_slot_periodicity_t ping_slot_periodicity )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    if( lorawan_api_set_ping_slot_periodicity( ( uint8_t ) ping_slot_periodicity ) == OKLORAWAN )
    {
        return SMTC_MODEM_RC_OK;
    }
    return SMTC_MODEM_RC_INVALID;
}

smtc_modem_return_code_t smtc_modem_class_b_get_ping_slot_periodicity(
    uint8_t stack_id, smtc_modem_class_b_ping_slot_periodicity_t* ping_slot_periodicity )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( ping_slot_periodicity );

    *ping_slot_periodicity = ( smtc_modem_class_b_ping_slot_periodicity_t ) lorawan_api_get_ping_slot_periodicity( );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_d2d_class_b_request_uplink( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                                smtc_modem_d2d_class_b_uplink_config_t* d2d_config,
                                                                uint8_t fport, const uint8_t* payload,
                                                                uint8_t payload_length )
{
#if defined( SMTC_MULTICAST ) && defined( SMTC_D2D )
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    if( ( fport == 0 ) || ( fport >= 224 ) || ( fport == get_modem_dm_port( ) ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "%s port %d is forbidden \n", __func__, fport );
        return SMTC_MODEM_RC_INVALID;
    }

    smtc_class_b_d2d_status_t d2d_rc;
    smtc_modem_return_code_t  modem_rc;
    bool                      session_running = false;
    lorawan_multicast_rc_t    mc_rc           = lorawan_api_multicast_get_running_status( mc_grp_id, &session_running );

    switch( mc_rc )
    {
    case LORAWAN_MC_RC_OK:
        modem_rc = SMTC_MODEM_RC_FAIL;
        if( session_running == true )
        {
            d2d_rc = lorawan_api_class_b_d2d_request_tx(
                mc_grp_id + 1, fport, d2d_config->priority, payload, payload_length, d2d_config->nb_rep,
                d2d_config->nb_ping_slot_tries, d2d_config->ping_slots_mask, 16 );
            if( d2d_rc == SMTC_CLASS_B_D2D_OK )
            {
                modem_rc = SMTC_MODEM_RC_OK;
            }
        }
        break;
    case LORAWAN_MC_RC_ERROR_BAD_ID:
        modem_rc = SMTC_MODEM_RC_INVALID;
        break;
    default:
        modem_rc = SMTC_MODEM_RC_FAIL;
        break;
    }
    return modem_rc;
#else   // SMTC_MULTICAST && SMTC_D2D
    return SMTC_MODEM_RC_FAIL;
#endif  // SMTC_MULTICAST && SMTC_D2D
}

smtc_modem_return_code_t smtc_modem_d2d_class_b_get_tx_max_payload( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                                    uint8_t* tx_max_payload_size )
{
#if defined( SMTC_MULTICAST ) && defined( SMTC_D2D )
    *tx_max_payload_size = 0;

    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( tx_max_payload_size );

    bool                   session_running = false;
    lorawan_multicast_rc_t rc              = lorawan_api_multicast_get_running_status( mc_grp_id, &session_running );

    if( rc == LORAWAN_MC_RC_OK )
    {
        if( session_running == true )
        {
            *tx_max_payload_size = lorawan_api_class_b_d2d_next_max_payload_length_get( mc_grp_id + 1 );
        }
    }

    smtc_modem_return_code_t modem_rc;
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
#else   // SMTC_MULTICAST && SMTC_D2D
    return SMTC_MODEM_RC_FAIL;
#endif  // SMTC_MULTICAST && SMTC_D2D
}

smtc_modem_return_code_t smtc_modem_get_network_frame_pending_status(
    uint8_t stack_id, smtc_modem_frame_pending_bit_status_t* frame_pending_bit_status )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( frame_pending_bit_status );

    *frame_pending_bit_status = ( smtc_modem_frame_pending_bit_status_t ) lorawan_api_rx_fpending_bit_get( );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_set_adr_ack_limit_delay( uint8_t stack_id, uint8_t adr_ack_limit,
                                                             uint8_t adr_ack_delay )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    if( lorawan_api_set_adr_ack_limit_delay( adr_ack_limit, adr_ack_delay ) != OKLORAWAN )
    {
        return SMTC_MODEM_RC_INVALID;
    }
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_adr_ack_limit_delay( uint8_t stack_id, uint8_t* adr_ack_limit,
                                                             uint8_t* adr_ack_delay )
{
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( adr_ack_limit );
    RETURN_INVALID_IF_NULL( adr_ack_delay );

    lorawan_api_get_adr_ack_limit_delay( adr_ack_limit, adr_ack_delay );
    return SMTC_MODEM_RC_OK;
}

/* ------------ Basic Modem LR11XX Extension functions ------------*/

smtc_modem_return_code_t smtc_modem_get_pin( uint8_t stack_id, uint8_t chip_pin[4] )
{
    UNUSED( stack_id );

    RETURN_BUSY_IF_TEST_MODE( );

#if defined( USE_LR11XX_CE )
    lr11xx_system_uid_t      deveui;
    lr11xx_system_join_eui_t joineui;

    if( lorawan_api_get_deveui( ( uint8_t* ) deveui ) != OKLORAWAN )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    if( lorawan_api_get_joineui( ( uint8_t* ) joineui ) != OKLORAWAN )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    // lr11xx operation needed: suspend modem radio access to secure this direct access
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );

    lr11xx_status_t status =
        lr11xx_system_read_pin_custom_eui( modem_context_get_modem_radio_ctx( ), deveui, joineui, 0, chip_pin );

    // lr11xx operation done: resume modem radio access
    modem_context_resume_radio_access( );

    // when pin code is read, a new key derivation is done in lr11xx so a external app_key is used it will be lost
    // and shall be updated once more. Corrupt the key crc so that update is possible
    modem_context_appkey_is_derived( );

    if( status != LR11XX_STATUS_OK )
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

#if defined( USE_LR11XX_CE )

    // lr11xx operation needed: suspend modem radio access to secure this direct access
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    lr11xx_status_t status = lr11xx_system_read_uid( modem_context_get_modem_radio_ctx( ), chip_eui );
    // lr11xx operation done: resume modem radio access
    modem_context_resume_radio_access( );
    if( status != LR11XX_STATUS_OK )
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

#if defined( USE_LR11XX_CE )
    lr11xx_system_uid_t      deveui;
    lr11xx_system_join_eui_t joineui;
    lr11xx_system_pin_t      pin;

    if( lorawan_api_get_deveui( ( uint8_t* ) deveui ) != OKLORAWAN )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    if( lorawan_api_get_joineui( ( uint8_t* ) joineui ) != OKLORAWAN )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    // lr11xx operation needed: suspend modem radio access to secure this direct access
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );

    // Read pin code with current EUIs forces a key derivation
    lr11xx_status_t status =
        lr11xx_system_read_pin_custom_eui( modem_context_get_modem_radio_ctx( ), deveui, joineui, 0, pin );

    // lr11xx operation done: resume modem radio access
    modem_context_resume_radio_access( );

    // when pin code is read, a new key derivation is done in lr11xx so a external app_key is used it will be lost
    // and shall be updated once more. Corrupt the key crc so that update is possible
    modem_context_appkey_is_derived( );

    if( status != LR11XX_STATUS_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
#else
    return SMTC_MODEM_RC_FAIL;
#endif
}

/* ------------ Modem-E Api Extension functions ------------*/
#if defined( LR1110_MODEM_E )
smtc_modem_return_code_t smtc_modem_stream_get_redundancy_ratio( uint8_t stack_id, uint8_t* stream_rr )
{
#if defined( ADD_SMTC_STREAM )
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    *stream_rr = stream_get_rr( &( smtc_modem_services_ctx.stream_ROSE_ctx ) );
    return SMTC_MODEM_RC_OK;
#else   // ADD_SMTC_STREAM
    return SMTC_MODEM_RC_FAIL;
#endif  // ADD_SMTC_STREAM
}

smtc_modem_return_code_t smtc_modem_stream_set_redundancy_ratio( uint8_t stack_id, uint8_t redundancy_ratio_percent )
{
#if defined( ADD_SMTC_STREAM )
    UNUSED( stack_id );
    RETURN_BUSY_IF_TEST_MODE( );

    stream_set_rr( &( smtc_modem_services_ctx.stream_ROSE_ctx ), redundancy_ratio_percent );
    return SMTC_MODEM_RC_OK;
#else   // ADD_SMTC_STREAM
    return SMTC_MODEM_RC_FAIL;
#endif  // ADD_SMTC_STREAM
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
    if( modem_set_rfo_pa( rf_output ) != DM_OK )
    {
        return_code = SMTC_MODEM_RC_INVALID;
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_time_available( void )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_NO_TIME;

#if defined( ADD_SMTC_ALC_SYNC )
    if( clock_sync_is_time_valid( &( smtc_modem_services_ctx.clock_sync_ctx ) ) == true )
    {
        return_code = SMTC_MODEM_RC_OK;
    }
#else   // ADD_SMTC_ALC_SYNC
    if( lorawan_api_is_time_valid( ) == true )
    {
        return_code = SMTC_MODEM_RC_OK;
    }
#endif  // ADD_SMTC_ALC_SYNC

    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_time( uint32_t gps_time_s )
{
#if defined( ADD_SMTC_ALC_SYNC )
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    clock_sync_set_gps_time( &( smtc_modem_services_ctx.clock_sync_ctx ), gps_time_s );

    return return_code;
#else   // ADD_SMTC_ALC_SYNC
    return SMTC_MODEM_RC_FAIL;
#endif  // ADD_SMTC_ALC_SYNC
}
void empty_task_launch_callback_for_rp( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    smtc_modem_hal_start_radio_tcxo( );
    rp_stats_set_none_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
    SMTC_MODEM_HAL_TRACE_PRINTF( "launch task empty\n" );
}

void smtc_modem_suspend_rp( e_sniff_mode_t sniff_mode )
{
    rp_radio_params_t fake_radio_params = { 0 };
    rp_task_t         rp_task           = { 0 };
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
void smtc_modem_resume_rp( void )
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
                                                                    uint8_t* dm_field_length, dm_info_rate_t rate )
{
    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    *dm_field_length                     = get_dm_info_tag_list( dm_fields_payload, rate );
    return return_code;
}

static smtc_modem_return_code_t smtc_modem_set_dm_status_with_rate( const uint8_t* dm_fields_payload,
                                                                    uint8_t dm_field_length, dm_info_rate_t rate )
{
    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    if( set_dm_info( dm_fields_payload, dm_field_length, rate ) == DM_ERROR )
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
    smodem_task              task_send   = { 0 };

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
                                                    uint8_t payload_length, bool emergency, uint8_t tx_buffer_id )
{
    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    smodem_task              task_send   = { 0 };

    if( is_modem_connected( ) == false )
    {
        return_code = SMTC_MODEM_RC_FAIL;
    }
    else if( payload_length > SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH )
    {
        return_code = SMTC_MODEM_RC_INVALID;
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
        switch( tx_buffer_id )
        {
        case 0:
            memcpy( modem_buffer, payload, payload_length );
            task_send.id     = SEND_TASK;
            task_send.dataIn = modem_buffer;
            break;
        case 1:
            task_send.id     = SEND_TASK_EXTENDED_1;
            task_send.dataIn = payload;
            break;

        case 2:
            task_send.dataIn = payload;
            task_send.id     = SEND_TASK_EXTENDED_2;
            break;

        default:
            return SMTC_MODEM_RC_FAIL;
        }
        task_send.fPort             = f_port;
        task_send.fPort_present     = true;
        task_send.PacketType        = confirmed;
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

smtc_modem_event_user_radio_access_status_t convert_rp_to_user_radio_access_status( rp_status_t rp_status )
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
    case RP_STATUS_TASK_ABORTED:
        user_radio_access_status = SMTC_MODEM_EVENT_USER_RADIO_ACCESS_ABORTED;
        break;
    default:
        user_radio_access_status = SMTC_MODEM_EVENT_USER_RADIO_ACCESS_UNKNOWN;
        break;
    }
    return user_radio_access_status;
}

smtc_modem_rp_radio_status_t convert_rp_to_user_radio_access_rp_status( rp_status_t rp_status )
{
    smtc_modem_rp_radio_status_t modem_rp_radio_status = SMTC_RP_RADIO_UNKNOWN;

    switch( rp_status )
    {
    case RP_STATUS_RX_CRC_ERROR:
        modem_rp_radio_status = SMTC_RP_RADIO_RX_ERROR;
        break;
    case RP_STATUS_CAD_POSITIVE:
        modem_rp_radio_status = SMTC_RP_RADIO_CAD_OK;
        break;
    case RP_STATUS_CAD_NEGATIVE:
        modem_rp_radio_status = SMTC_RP_RADIO_CAD_DONE;
        break;
    case RP_STATUS_TX_DONE:
        modem_rp_radio_status = SMTC_RP_RADIO_TX_DONE;
        break;
    case RP_STATUS_RX_PACKET:
        modem_rp_radio_status = SMTC_RP_RADIO_RX_DONE;
        break;
    case RP_STATUS_RX_TIMEOUT:
        modem_rp_radio_status = SMTC_RP_RADIO_RX_TIMEOUT;
        break;
    case RP_STATUS_WIFI_SCAN_DONE:
        modem_rp_radio_status = SMTC_RP_RADIO_WIFI_SCAN_DONE;
        break;
    case RP_STATUS_GNSS_SCAN_DONE:
        modem_rp_radio_status = SMTC_RP_RADIO_GNSS_SCAN_DONE;
        break;
    case RP_STATUS_TASK_ABORTED:
        modem_rp_radio_status = SMTC_RP_RADIO_ABORTED;
        break;
    default:
        modem_rp_radio_status = SMTC_RP_RADIO_UNKNOWN;
        break;
    }
    return modem_rp_radio_status;
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

    rp_get_status( rp, rp->radio_task_id, &user_radio_irq_timestamp, &user_radio_irq_status );

    switch( user_radio_irq_status )
    {
    case RP_STATUS_TASK_ABORTED:
        SMTC_MODEM_HAL_TRACE_INFO( "User radio access callback: ignored ABORTED status \n" );
        break;
    default:
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_USER_RADIO_ACCESS, 0 );
        break;
    }
}

#if !defined( LR1110_MODEM_E )
void callback_rp_user_radio_access_0( void* ctx )
{
    radio_planner_t*       rp              = ( radio_planner_t* ) ctx;
    smtc_modem_rp_status_t modem_rp_status = { 0 };
    uint32_t               rp_timestamp    = 0;
    ral_irq_t              rp_radio_irq    = 0;
    rp_status_t            rp_status;

    rp_get_status( rp, RP_HOOK_ID_USER_SUSPEND_0, &rp_timestamp, &rp_status );
    rp_get_and_clear_raw_radio_irq( rp, RP_HOOK_ID_USER_SUSPEND_0, &rp_radio_irq );

    modem_rp_status.id           = SMTC_MODEM_RP_TASK_ID0;
    modem_rp_status.timestamp_ms = rp_timestamp;
    modem_rp_status.status       = convert_rp_to_user_radio_access_rp_status( rp_status );
    modem_rp_status.raw_irq      = ( uint16_t ) rp_radio_irq;

    // call user callback
    if( *user_end_task_callback_0 != NULL )
    {
        user_end_task_callback_0( &modem_rp_status );
    }
}

void callback_rp_user_radio_access_1( void* ctx )
{
    radio_planner_t*       rp              = ( radio_planner_t* ) ctx;
    smtc_modem_rp_status_t modem_rp_status = { 0 };
    uint32_t               rp_timestamp    = 0;
    ral_irq_t              rp_radio_irq    = 0;
    rp_status_t            rp_status;

    rp_get_status( rp, RP_HOOK_ID_USER_SUSPEND_1, &rp_timestamp, &rp_status );
    rp_get_and_clear_raw_radio_irq( rp, RP_HOOK_ID_USER_SUSPEND_1, &rp_radio_irq );

    modem_rp_status.id           = SMTC_MODEM_RP_TASK_ID1;
    modem_rp_status.timestamp_ms = rp_timestamp;
    modem_rp_status.status       = convert_rp_to_user_radio_access_rp_status( rp_status );
    modem_rp_status.raw_irq      = ( uint16_t ) rp_radio_irq;

    // call user callback
    if( *user_end_task_callback_1 != NULL )
    {
        user_end_task_callback_1( &modem_rp_status );
    }
}

void callback_rp_user_radio_access_2( void* ctx )
{
    radio_planner_t*       rp              = ( radio_planner_t* ) ctx;
    smtc_modem_rp_status_t modem_rp_status = { 0 };
    uint32_t               rp_timestamp    = 0;
    ral_irq_t              rp_radio_irq    = 0;
    rp_status_t            rp_status;

    rp_get_status( rp, RP_HOOK_ID_USER_SUSPEND_2, &rp_timestamp, &rp_status );
    rp_get_and_clear_raw_radio_irq( rp, RP_HOOK_ID_USER_SUSPEND_2, &rp_radio_irq );

    modem_rp_status.id           = SMTC_MODEM_RP_TASK_ID2;
    modem_rp_status.timestamp_ms = rp_timestamp;
    modem_rp_status.status       = convert_rp_to_user_radio_access_rp_status( rp_status );
    modem_rp_status.raw_irq      = ( uint16_t ) rp_radio_irq;

    // call user callback
    if( *user_end_task_callback_2 != NULL )
    {
        user_end_task_callback_2( &modem_rp_status );
    }
}
#endif  // !LR1110_MODEM_E

/* --- EOF ------------------------------------------------------------------ */
