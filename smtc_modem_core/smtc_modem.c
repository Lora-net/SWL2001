/*!
 * \file      smtc_modem_light.c
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
#include "smtc_modem_test_api.h"
#include "lorawan_management_defs.h"
#include "lorawan_send_management.h"
#include "lorawan_cid_request_management.h"
#include "lorawan_class_b_management.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "modem_supervisor_light.h"
#include "modem_core.h"
#include "smtc_real_defs.h"
#include "lorawan_api.h"
#include "smtc_duty_cycle.h"

#include "radio_planner.h"
#include "ral.h"
#include "ralf.h"
#include "smtc_modem_utilities.h"
#include "modem_event_utilities.h"
#include "modem_core.h"
#include "smtc_modem_crypto.h"
#include "lora_basics_modem_version.h"

#if defined( REGION_EU_868 )
#include "region_eu_868_defs.h"
#endif
#if defined( REGION_RU_864 )
#include "region_ru_864_defs.h"
#endif

#if defined( USE_LR11XX_CE )
#include "lr11xx_system.h"
#endif  // USE_LR11XX_CE

#if defined( SX128X )
#include "ralf_sx128x.h"
#elif defined( SX126X )
#include "ralf_sx126x.h"
#elif defined( LR11XX )
#include "ralf_lr11xx.h"
#elif defined( SX127X )
#include "ralf_sx127x.h"
#endif

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

#define MODEM_FW_VERSION_MAJOR LORA_BASICS_MODEM_FW_VERSION_MAJOR
#define MODEM_FW_VERSION_MINOR LORA_BASICS_MODEM_FW_VERSION_MINOR
#define MODEM_FW_VERSION_PATCH LORA_BASICS_MODEM_FW_VERSION_PATCH

#define MODEM_KEY_CRC_STATUS_VALID ( 0 )
#define MODEM_KEY_CRC_STATUS_INVALID ( 1 )

#define MODEM_MAX_ALARM_VALUE_S ( 864000 )  // 10 days in seconds

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct modem_key_ctx_s
{
    uint8_t  appkey_crc_status;
    uint32_t appkey_crc;
    uint32_t rfu[3];
    uint32_t crc;  // !! crc MUST be the last field of the structure !!
} modem_key_ctx_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

radio_planner_t modem_radio_planner;

#if defined( SX128X )
ralf_t modem_radio = RALF_SX128X_INSTANTIATE( NULL );
#elif defined( SX126X )
ralf_t modem_radio = RALF_SX126X_INSTANTIATE( NULL );
#elif defined( LR11XX )
ralf_t modem_radio = RALF_LR11XX_INSTANTIATE( NULL );
#elif defined( SX127X )
#include "sx127x.h"
static sx127x_t sx127x;
ralf_t          modem_radio = RALF_SX127X_INSTANTIATE( &sx127x );
#else
#error "Please select radio board.."
#endif

struct
{
    int16_t  modem_key_status;
    uint32_t modem_key_crc;
} smtc_modem_ctx;

#define modem_key_status smtc_modem_ctx.modem_key_status
#define modem_key_crc smtc_modem_ctx.modem_key_crc

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static smtc_modem_return_code_t smtc_modem_send_tx( uint8_t stack_id, uint8_t f_port, bool confirmed,
                                                    const uint8_t* payload, uint8_t payload_length, bool emergency );

static smtc_modem_return_code_t smtc_modem_custom_dr_distribution_to_tab(
    uint16_t mask_dr_allowed, const uint8_t adr_custom_data[SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH],
    uint8_t adr_distribution[SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH] );

void empty_callback( void* ctx )
{
}

#if defined( USE_LR11XX_CE )
static void modem_store_key_context( void );
static void modem_load_appkey_context( void );
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/* ------------ Modem Utilities ------------*/

void smtc_modem_init( void ( *callback_event )( void ) )
{
    SMTC_MODEM_HAL_TRACE_INFO( "Modem Initialization\n" );

    // init radio and put it in sleep mode
    ral_reset( &( modem_radio.ral ) );
    ral_init( &( modem_radio.ral ) );
    ral_set_sleep( &( modem_radio.ral ), true );
    smtc_modem_hal_set_ant_switch( false );
    // init radio planner and attach corresponding radio irq
    rp_init( &modem_radio_planner, &modem_radio );

    smtc_modem_hal_irq_config_radio_irq( rp_radio_irq_callback, &modem_radio_planner );

    rp_hook_init( &modem_radio_planner, RP_HOOK_ID_SUSPEND, ( void ( * )( void* ) )( empty_callback ),
                  &modem_radio_planner );

    smtc_secure_element_init( );
    modem_supervisor_init( );
    modem_context_init_light( callback_event, &modem_radio_planner );
#if defined( USE_LR11XX_CE )
    modem_load_appkey_context( );
#endif
    // Event EVENT_RESET must be done at the end of init !!
    increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_RESET, 0, 0xFF );
}

uint32_t smtc_modem_run_engine( void )
{
    rp_callback( &modem_radio_planner );
    return modem_supervisor_engine( );
}

void smtc_modem_set_radio_context( const void* radio_ctx )
{
#if defined( SX1272 ) || defined( SX1276 )
    // update modem_radio context with provided one
    ( ( sx127x_t* ) modem_radio.ral.context )->hal_context = radio_ctx;
#else
    // update modem_radio context with provided one
    modem_radio.ral.context = radio_ctx;
#endif
    // Save modem radio context in case of direct access to radio by the modem
    modem_set_radio_ctx( modem_radio.ral.context );
}

const void* smtc_modem_get_radio_context( void )
{
    // Get radio context
    return modem_radio.ral.context;
}

bool smtc_modem_is_irq_flag_pending( void )
{
    return rp_get_irq_flag( &modem_radio_planner );
}

/* ------------ Modem Generic Api ------------*/

smtc_modem_return_code_t smtc_modem_get_joineui( uint8_t stack_id, uint8_t joineui[SMTC_MODEM_EUI_LENGTH] )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( joineui );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    if( lorawan_api_get_joineui( joineui, stack_id ) != OKLORAWAN )
    {
        return_code = SMTC_MODEM_RC_FAIL;
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_joineui( uint8_t stack_id, const uint8_t joineui[SMTC_MODEM_EUI_LENGTH] )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( joineui );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    // Check join status: the modem shall not be joined to modify the joineui
    if( lorawan_api_isjoined( stack_id ) != NOT_JOINED )
    {
        return_code = SMTC_MODEM_RC_BUSY;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else
    {
        if( lorawan_api_set_joineui( joineui, stack_id ) != OKLORAWAN )
        {
            return_code = SMTC_MODEM_RC_FAIL;
        }
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_deveui( uint8_t stack_id, uint8_t deveui[SMTC_MODEM_EUI_LENGTH] )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( deveui );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    if( lorawan_api_get_deveui( deveui, stack_id ) != OKLORAWAN )
    {
        return_code = SMTC_MODEM_RC_FAIL;
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_deveui( uint8_t stack_id, const uint8_t deveui[SMTC_MODEM_EUI_LENGTH] )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( deveui );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    // Check join status: the modem shall not be joined to modify the deveui
    if( lorawan_api_isjoined( stack_id ) != NOT_JOINED )
    {
        return_code = SMTC_MODEM_RC_BUSY;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else
    {
        if( lorawan_api_set_deveui( deveui, stack_id ) != OKLORAWAN )
        {
            return_code = SMTC_MODEM_RC_FAIL;
        }
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_nwkkey( uint8_t stack_id, const uint8_t nwkkey[SMTC_MODEM_KEY_LENGTH] )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( nwkkey );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    // Check join status: the modem shall not be joined to modify the key
    if( lorawan_api_isjoined( stack_id ) != NOT_JOINED )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
        return SMTC_MODEM_RC_BUSY;
    }

// To prevent too much flash access first check crc on key in case of Hardware Secure element
#if defined( USE_LR11XX_CE )
    uint32_t new_crc = crc( nwkkey, 16 );

    if( ( modem_key_status == MODEM_KEY_CRC_STATUS_INVALID ) || ( modem_key_crc != new_crc ) )
    {
        modem_key_crc    = new_crc;
        modem_key_status = MODEM_KEY_CRC_STATUS_VALID;
#endif
        if( lorawan_api_set_appkey( nwkkey, stack_id ) != OKLORAWAN )
        {
            return_code = SMTC_MODEM_RC_FAIL;
        }
#if defined( USE_LR11XX_CE )
        else
        {
            // Store appkey crc and status
            modem_store_key_context( );
        }
    }
#endif
    return return_code;
}
smtc_modem_return_code_t smtc_modem_get_region( uint8_t stack_id, smtc_modem_region_t* region )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( region );

    *region = ( smtc_modem_region_t ) lorawan_api_get_region( stack_id );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_set_region( uint8_t stack_id, smtc_modem_region_t region )
{
    RETURN_BUSY_IF_TEST_MODE( );

    if( lorawan_api_isjoined( stack_id ) != NOT_JOINED )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
        return SMTC_MODEM_RC_BUSY;
    }
    if( lorawan_api_set_region( ( smtc_real_region_types_t ) region, stack_id ) != OKLORAWAN )
    {
        return SMTC_MODEM_RC_INVALID;
    }

#if defined( REGION_EU_868 )
    if( region == SMTC_MODEM_REGION_EU_868 )
    {
        // Configure duty-cycle object
        for( int i = 0; i < BAND_EU868_MAX; i++ )
        {
            smtc_duty_cycle_config( BAND_EU868_MAX, i, duty_cycle_by_band_eu_868[i],
                                    frequency_range_by_band_eu_868[i][0], frequency_range_by_band_eu_868[i][1] );
        }
    }
#endif
#if defined( REGION_RU_864 )

    if( region == SMTC_MODEM_REGION_RU_864 )
    {
        // Configure duty-cycle object
        for( int i = 0; i < BAND_RU864_MAX; i++ )
        {
            smtc_duty_cycle_config( BAND_RU864_MAX, i, duty_cycle_by_band_ru_864[i],
                                    frequency_range_by_band_ru_864[i][0], frequency_range_by_band_ru_864[i][1] );
        }
    }
#endif

    // Enable duty-cycle constraint if one region need to support the duty cycle
    // Remark: In multistack EU868 and IN865 there is a bands overlapping, EU868 has a duty-cycle but not IN865, in this
    // case the duty cycle will be enabled
    bool dtc_supported = false;
    for( uint8_t i = 0; i < NUMBER_OF_STACKS; i++ )
    {
        if( lorawan_api_is_dtc_supported( i ) == true )
        {
            dtc_supported = true;
            break;
        }
    }

    smtc_duty_cycle_enable_set( dtc_supported );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_set_nb_trans( uint8_t stack_id, uint8_t nb_trans )
{
    RETURN_BUSY_IF_TEST_MODE( );

    if( lorawan_api_isjoined( stack_id ) != JOINED )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    if( lorawan_api_nb_trans_set( nb_trans, stack_id ) == OKLORAWAN )
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
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( nb_trans );

    *nb_trans = lorawan_api_nb_trans_get( stack_id );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_adr_set_join_distribution(
    uint8_t stack_id, const uint8_t dr_custom_distribution_data[SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH] )
{
    RETURN_BUSY_IF_TEST_MODE( );

    if( lorawan_api_isjoined( stack_id ) != NOT_JOINED )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    // Check parameters
    if( NULL == dr_custom_distribution_data )
    {
        return SMTC_MODEM_RC_INVALID;
    }

    uint16_t mask_dr_allowed = lorawan_api_mask_tx_dr_channel_up_dwell_time_check( stack_id );
    uint8_t  adr_distribution_tab[SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH] = { 0 };

    smtc_modem_return_code_t conversion_status =
        smtc_modem_custom_dr_distribution_to_tab( mask_dr_allowed, dr_custom_distribution_data, adr_distribution_tab );
    if( conversion_status != SMTC_MODEM_RC_OK )
    {
        return conversion_status;
    }
    lorawan_api_dr_join_distribution_set( adr_distribution_tab, stack_id );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_adr_set_profile(
    uint8_t stack_id, smtc_modem_adr_profile_t adr_profile,
    const uint8_t dr_custom_distribution_data[SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH] )
{
    RETURN_BUSY_IF_TEST_MODE( );

    if( lorawan_api_isjoined( stack_id ) != JOINED )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    if( adr_profile == SMTC_MODEM_ADR_PROFILE_CUSTOM )
    {
        // Check parameters
        if( NULL == dr_custom_distribution_data )
        {
            return SMTC_MODEM_RC_INVALID;
        }
    }
    if( ( adr_profile == SMTC_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE ) ||
        ( adr_profile == SMTC_MODEM_ADR_PROFILE_MOBILE_LOW_POWER ) || ( adr_profile == SMTC_MODEM_ADR_PROFILE_CUSTOM ) )
    {
        // reset current adr mobile count
        lorawan_api_reset_no_rx_packet_in_mobile_mode_cnt( stack_id );
    }
    status_lorawan_t status = ERRORLORAWAN;

    switch( adr_profile )
    {
    case SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED:
        // update profile in lorawan stack
        status = lorawan_api_dr_strategy_set( STATIC_ADR_MODE, stack_id );
        break;
    case SMTC_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE:
        // update profile in lorawan stack
        status = lorawan_api_dr_strategy_set( MOBILE_LONGRANGE_DR_DISTRIBUTION, stack_id );
        break;
    case SMTC_MODEM_ADR_PROFILE_MOBILE_LOW_POWER:
        // update profile in lorawan stack
        status = lorawan_api_dr_strategy_set( MOBILE_LOWPER_DR_DISTRIBUTION, stack_id );
        break;
    case SMTC_MODEM_ADR_PROFILE_CUSTOM: {
        uint16_t mask_dr_allowed = lorawan_api_mask_tx_dr_channel_up_dwell_time_check( stack_id );
        uint8_t  adr_distribution_tab[SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH] = { 0 };

        smtc_modem_return_code_t conversion_status = smtc_modem_custom_dr_distribution_to_tab(
            mask_dr_allowed, dr_custom_distribution_data, adr_distribution_tab );

        if( conversion_status != SMTC_MODEM_RC_OK )
        {
            return conversion_status;
        }
        lorawan_api_dr_custom_set( adr_distribution_tab, stack_id );

        // update profile in lorawan stack
        status = lorawan_api_dr_strategy_set( USER_DR_DISTRIBUTION, stack_id );
        break;
    }
    default: {
        SMTC_MODEM_HAL_TRACE_ERROR( "Unknown adr profile %d\n ", adr_profile );
        return SMTC_MODEM_RC_INVALID;
    }
    break;
    }

    if( status == ERRORLORAWAN )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call with adr profile not valid\n", __func__ );
        return SMTC_MODEM_RC_INVALID;
    }
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_available_datarates( uint8_t stack_id, uint16_t* available_datarates_mask )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( available_datarates_mask );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    *available_datarates_mask            = lorawan_api_mask_tx_dr_channel_up_dwell_time_check( stack_id );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_join_network( uint8_t stack_id )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    if( lorawan_api_isjoined( stack_id ) != NOT_JOINED )  // the modem have to be leave from the network to join
    {
        return_code = SMTC_MODEM_RC_BUSY;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else if( lorawan_api_get_activation_mode( stack_id ) == ACTIVATION_MODE_ABP )
    {
        lorawan_api_join( 0, stack_id );
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_JOINED, 0, stack_id );
        return_code = SMTC_MODEM_RC_OK;
    }
    else
    {
        // Launch OTAA task
        lorawan_join_add_task( stack_id );
    }

    return return_code;
}
smtc_modem_return_code_t smtc_modem_request_uplink( uint8_t stack_id, uint8_t f_port, bool confirmed,
                                                    const uint8_t* payload, uint8_t payload_length )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( payload );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    return_code = smtc_modem_send_tx( stack_id, f_port, confirmed, payload, payload_length, false );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_event( smtc_modem_event_t* event, uint8_t* event_pending_count )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( event );
    RETURN_INVALID_IF_NULL( event_pending_count );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    const uint8_t            event_count = get_asynchronous_msgnumber( );

    if( event_count > MODEM_NUMBER_OF_EVENTS )
    {
        SMTC_MODEM_HAL_PANIC( "asynchronous_msgnumber overlap" );
    }
    else if( event_count > 0 )
    {
        uint8_t stack_id;
        event->event_type    = ( smtc_modem_event_type_t ) get_last_msg_event( &stack_id );
        event->stack_id      = stack_id;
        event->missed_events = get_modem_event_count( event->event_type ) - 1;

        *event_pending_count = event_count - 1;

        switch( event->event_type )
        {
        case SMTC_MODEM_EVENT_TXDONE:
            event->event_data.txdone.status =
                ( smtc_modem_event_txdone_status_t ) get_modem_event_status( event->event_type );
            break;

        case SMTC_MODEM_EVENT_LINK_CHECK:
            event->event_data.link_check.status =
                ( smtc_modem_event_mac_request_status_t ) get_modem_event_status( event->event_type );
            break;

        case SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO:
            event->event_data.class_b_ping_slot_info.status =
                ( smtc_modem_event_mac_request_status_t ) get_modem_event_status( event->event_type );
            break;

        case SMTC_MODEM_EVENT_CLASS_B_STATUS:
            event->event_data.class_b_status.status =
                ( smtc_modem_event_class_b_status_t ) get_modem_event_status( event->event_type );
            break;

        case SMTC_MODEM_EVENT_LORAWAN_MAC_TIME:
            event->event_data.lorawan_mac_time.status =
                ( smtc_modem_event_mac_request_status_t ) get_modem_event_status( event->event_type );
            break;
        case SMTC_MODEM_EVENT_LORAWAN_FUOTA_DONE:
            event->event_data.fuota_status.successful =
                ( get_modem_event_status( event->event_type ) == 0 ) ? true : false;
            break;

        case SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_C:
            event->event_data.new_multicast_class_c.group_id = get_modem_event_status( event->event_type );
            break;

        case SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_B:
            event->event_data.new_multicast_class_b.group_id = get_modem_event_status( event->event_type );
            break;

#if defined( ENABLE_FUOTA_FMP )
        case SMTC_MODEM_EVENT_FIRMWARE_MANAGEMENT:
            event->event_data.fmp.status = get_modem_event_status( event->event_type );
            break;
#endif

        case SMTC_MODEM_EVENT_RESET:
        case SMTC_MODEM_EVENT_DOWNDATA:
        case SMTC_MODEM_EVENT_ALARM:
        case SMTC_MODEM_EVENT_JOINED:
        case SMTC_MODEM_EVENT_JOINFAIL:
        case SMTC_MODEM_EVENT_ALCSYNC_TIME:
        case SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_C:
        case SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_B:
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
        return_code = SMTC_MODEM_RC_NO_EVENT;
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_downlink_data( uint8_t  buff[SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH],
                                                       uint8_t* length, smtc_modem_dl_metadata_t* metadata,
                                                       uint8_t* remaining_data_nb )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( buff );
    RETURN_INVALID_IF_NULL( length );
    RETURN_INVALID_IF_NULL( metadata );
    RETURN_INVALID_IF_NULL( remaining_data_nb );

    smtc_modem_return_code_t rc       = SMTC_MODEM_RC_OK;
    fifo_ctrl_t*             fifo_obj = modem_context_get_fifo_obj( );

    // get_nb of data in fifo
    uint16_t nb_of_data = fifo_ctrl_get_nb_elt( fifo_obj );

    // if some data is available in fifo
    if( nb_of_data == 0 )
    {
        rc = SMTC_MODEM_RC_FAIL;
    }
    else
    {
        uint8_t  metadata_len;
        uint16_t fifo_buff_length = 0;
        fifo_ctrl_get( fifo_obj, buff, &fifo_buff_length, SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH, metadata,
                       &metadata_len, sizeof( smtc_modem_dl_metadata_t ) );
        // Length of LoRaWAN packet cannot exceed 242
        *length            = ( uint8_t ) fifo_buff_length;
        *remaining_data_nb = nb_of_data - 1;
        rc                 = SMTC_MODEM_RC_OK;
    }

    return rc;
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

smtc_modem_return_code_t smtc_modem_lorawan_get_lost_connection_counter( uint8_t   stack_id,
                                                                         uint16_t* lost_connection_cnt )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( lost_connection_cnt );
    *lost_connection_cnt = lorawan_api_get_current_no_rx_packet_cnt( stack_id );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_set_certification_mode( uint8_t stack_id, bool enable )
{
    RETURN_BUSY_IF_TEST_MODE( );

    if( lorawan_certification_set_enabled( stack_id, enable ) != LORAWAN_CERTIFICATION_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    if( enable == true )
    {
        smtc_secure_element_store_context( );
    }
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_request_emergency_uplink( uint8_t stack_id, uint8_t fport, bool confirmed,
                                                              const uint8_t* payload, uint8_t payload_length )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( payload );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    return_code = smtc_modem_send_tx( stack_id, fport, confirmed, payload, payload_length, true );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_request_empty_uplink( uint8_t stack_id, bool send_fport, uint8_t fport,
                                                          bool confirmed )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;

    if( lorawan_api_isjoined( stack_id ) != JOINED )
    {
        return_code = SMTC_MODEM_RC_FAIL;
    }
    else if( ( send_fport == true ) &&
             ( ( ( fport == 0 ) || ( fport >= 224 ) ) && !lorawan_api_modem_certification_is_enabled( stack_id ) ) )
    {
        return_code = SMTC_MODEM_RC_INVALID;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s port %d is forbidden \n", __func__, fport );
    }
    else
    {
        lorawan_send_add_task( stack_id, fport, send_fport, confirmed, NULL, 0, false, 0 );
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_leave_network( uint8_t stack_id )
{
    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_set_class( stack_id, SMTC_MODEM_CLASS_A );
    modem_supervisor_abort_tasks_in_range( stack_id * NUMBER_OF_TASKS, ( ( stack_id + 1 ) * NUMBER_OF_TASKS ) - 1 );
    lorawan_api_join_status_clear( stack_id );

    return return_code;
}

smtc_modem_return_code_t smtc_modem_suspend_radio_communications( bool suspend )
{
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    for( uint8_t i = 0; i < NUMBER_OF_STACKS; i++ )
    {
        modem_supervisor_set_modem_is_suspended( suspend, i );
    }
    // TODO suspend Radio planner

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
        int32_t abs_remaining_time = ( int32_t ) ( modem_get_user_alarm( ) - smtc_modem_hal_get_time_in_s( ) );

        *remaining_time_in_s = ( abs_remaining_time > 0 ) ? ( abs_remaining_time ) : 0;
        return SMTC_MODEM_RC_OK;
    }
}
/* ------------ Basic Modem LR11XX Extension functions ------------*/

#if defined( USE_LR11XX_CE )
smtc_modem_return_code_t smtc_modem_get_pin( uint8_t stack_id, uint8_t chip_pin[4] )
{
    RETURN_BUSY_IF_TEST_MODE( );
    if( lorawan_api_isjoined( stack_id ) != NOT_JOINED )
    {
        return SMTC_MODEM_RC_BUSY;
    }

    lr11xx_system_uid_t      deveui;
    lr11xx_system_join_eui_t joineui;

    if( lorawan_api_get_deveui( ( uint8_t* ) deveui, stack_id ) != OKLORAWAN )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    if( lorawan_api_get_joineui( ( uint8_t* ) joineui, stack_id ) != OKLORAWAN )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    lr11xx_status_t status = lr11xx_system_read_pin_custom_eui( modem_get_radio_ctx( ), deveui, joineui, 0, chip_pin );

    // when pin code is read, a new key derivation is done in lr11xx so a external key is used it will be lost
    // and shall be updated once more. Corrupt the key crc so that update is possible
    modem_key_status = MODEM_KEY_CRC_STATUS_INVALID;
    modem_store_key_context( );

    if( status != LR11XX_STATUS_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_chip_eui( uint8_t stack_id, uint8_t chip_eui[8] )
{
    RETURN_BUSY_IF_TEST_MODE( );

    lr11xx_status_t status = lr11xx_system_read_uid( modem_get_radio_ctx( ), chip_eui );

    if( status != LR11XX_STATUS_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_derive_keys( uint8_t stack_id )
{
    RETURN_BUSY_IF_TEST_MODE( );
    if( lorawan_api_isjoined( stack_id ) != NOT_JOINED )
    {
        return SMTC_MODEM_RC_BUSY;
    }

    lr11xx_system_uid_t      deveui;
    lr11xx_system_join_eui_t joineui;
    lr11xx_system_pin_t      pin;

    if( lorawan_api_get_deveui( ( uint8_t* ) deveui, stack_id ) != OKLORAWAN )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    if( lorawan_api_get_joineui( ( uint8_t* ) joineui, stack_id ) != OKLORAWAN )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    // Read pin code with current EUIs forces a key derivation
    lr11xx_status_t status = lr11xx_system_read_pin_custom_eui( modem_get_radio_ctx( ), deveui, joineui, 0, pin );

    // when pin code is read, a new key derivation is done in lr11xx so a external key is used it will be lost
    // and shall be updated once more. Corrupt the key crc so that update is possible
    modem_key_status = MODEM_KEY_CRC_STATUS_INVALID;
    modem_store_key_context( );

    if( status != LR11XX_STATUS_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
}
#endif  // USE_LR11XX_CE

/*
 * -----------------------------------------------------------------------------
 * ----------- NETWORK MANAGEMENT MODEM FUNCTIONS ------------------------------
 */

smtc_modem_return_code_t smtc_modem_get_next_tx_max_payload( uint8_t stack_id, uint8_t* tx_max_payload_size )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( tx_max_payload_size );

    if( lorawan_api_isjoined( stack_id ) != JOINED )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is not join\n", __func__ );
        *tx_max_payload_size = 0;
        return SMTC_MODEM_RC_FAIL;
    }

    *tx_max_payload_size = lorawan_api_next_max_payload_length_get( stack_id );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_duty_cycle_status( uint8_t stack_id, int32_t* duty_cycle_status_ms )
{
    RETURN_BUSY_IF_TEST_MODE( );
    *duty_cycle_status_ms = -1 * modem_duty_cycle_get_status( stack_id );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_set_network_type( uint8_t stack_id, bool network_type )
{
    RETURN_BUSY_IF_TEST_MODE( );
    lorawan_api_set_network_type( network_type, stack_id );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_enabled_datarates( uint8_t stack_id, uint16_t* enabled_datarates_mask )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( enabled_datarates_mask );

    *enabled_datarates_mask = lorawan_api_mask_tx_dr_channel_up_dwell_time_check( stack_id );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_set_adr_ack_limit_delay( uint8_t stack_id, uint8_t adr_ack_limit,
                                                             uint8_t adr_ack_delay )
{
    RETURN_BUSY_IF_TEST_MODE( );

    if( lorawan_api_set_adr_ack_limit_delay( adr_ack_limit, adr_ack_delay, stack_id ) != OKLORAWAN )
    {
        return SMTC_MODEM_RC_INVALID;
    }
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_adr_ack_limit_delay( uint8_t stack_id, uint8_t* adr_ack_limit,
                                                             uint8_t* adr_ack_delay )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( adr_ack_limit );
    RETURN_INVALID_IF_NULL( adr_ack_delay );

    lorawan_api_get_adr_ack_limit_delay( adr_ack_limit, adr_ack_delay, stack_id );
    return SMTC_MODEM_RC_OK;
}

/*
 * -----------------------------------------------------------------------------
 * ----------- BOARD MANAGEMENT MODEM FUNCTIONS --------------------------------
 */

smtc_modem_return_code_t smtc_modem_set_crystal_error_ppm( uint32_t crystal_error_ppm )
{
    RETURN_BUSY_IF_TEST_MODE( );
    // set same crystal error for each stack
    for( uint8_t i = 0; i < NUMBER_OF_STACKS; i++ )
    {
        lorawan_api_set_crystal_error( crystal_error_ppm, i );
    }
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_lbt_set_parameters( uint8_t stack_id, uint32_t listen_duration_ms,
                                                        int16_t threshold_dbm, uint32_t bw_hz )
{
    RETURN_BUSY_IF_TEST_MODE( );

    SMTC_MODEM_HAL_TRACE_PRINTF( "LBT, duration:%d, threshold:%d, bw:%d\n", listen_duration_ms, threshold_dbm, bw_hz );

    lorawan_api_lbt_set_parameters( listen_duration_ms, threshold_dbm, bw_hz, stack_id );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_lbt_get_parameters( uint8_t stack_id, uint32_t* listen_duration_ms,
                                                        int16_t* threshold_dbm, uint32_t* bw_hz )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( listen_duration_ms );
    RETURN_INVALID_IF_NULL( threshold_dbm );
    RETURN_INVALID_IF_NULL( bw_hz );

    lorawan_api_lbt_get_parameters( listen_duration_ms, threshold_dbm, bw_hz, stack_id );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_lbt_set_state( uint8_t stack_id, bool enable )
{
    RETURN_BUSY_IF_TEST_MODE( );

    lorawan_api_lbt_set_state( enable, stack_id );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_lbt_get_state( uint8_t stack_id, bool* enabled )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( enabled );

    *enabled = lorawan_api_lbt_get_state( stack_id );
    return SMTC_MODEM_RC_OK;
}

#if defined( ADD_CSMA )
smtc_modem_return_code_t smtc_modem_csma_set_state( uint8_t stack_id, bool enable )
{
    RETURN_BUSY_IF_TEST_MODE( );
    lorawan_api_lora_cad_bt_set_state( enable, stack_id );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_csma_get_state( uint8_t stack_id, bool* enabled )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( enabled );

    *enabled = lorawan_api_lora_cad_bt_get_state( stack_id );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_csma_set_parameters( uint8_t stack_id, uint8_t nb_bo_max, bool bo_enabled,
                                                         uint8_t max_ch_change )
{
    RETURN_BUSY_IF_TEST_MODE( );
    lorawan_api_lora_cad_bt_set_parameters( nb_bo_max, bo_enabled, max_ch_change, stack_id );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_csma_get_parameters( uint8_t stack_id, uint8_t* max_ch_change, bool* bo_enabled,
                                                         uint8_t* nb_bo_max )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( max_ch_change );
    RETURN_INVALID_IF_NULL( bo_enabled );
    RETURN_INVALID_IF_NULL( nb_bo_max );
    lorawan_api_lora_cad_bt_get_parameters( max_ch_change, bo_enabled, nb_bo_max, stack_id );
    return SMTC_MODEM_RC_OK;
}
#endif  // ADD_CSMA

smtc_modem_return_code_t smtc_modem_get_charge( uint32_t* charge_mah )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( charge_mah );

    uint32_t total_consumption = modem_radio_planner.stats.tx_total_consumption_ma +
                                 modem_radio_planner.stats.rx_total_consumption_ma +
                                 modem_radio_planner.stats.none_total_consumption_ma;
    *charge_mah = total_consumption / 3600000;
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_reset_charge( void )
{
    RETURN_BUSY_IF_TEST_MODE( );

    rp_stats_init( &modem_radio_planner.stats );
    return SMTC_MODEM_RC_OK;
}

/*
 * -----------------------------------------------------------------------------
 * ----------- LORAWAN PACKAGES FUNCTIONS --------------------------------------
 */
#if defined( ADD_SMTC_ALC_SYNC )
smtc_modem_return_code_t smtc_modem_start_alcsync_service( uint8_t stack_id )
{
    RETURN_BUSY_IF_TEST_MODE( );
    if( lorawan_api_isjoined( stack_id ) != JOINED )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    lorawan_alcsync_set_enabled( stack_id, true );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_stop_alcsync_service( uint8_t stack_id )
{
    RETURN_BUSY_IF_TEST_MODE( );
    if( lorawan_api_isjoined( stack_id ) != JOINED )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    lorawan_alcsync_set_enabled( stack_id, false );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_alcsync_time( uint8_t stack_id, uint32_t* gps_time_s )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( gps_time_s );

    if( lorawan_alcsync_get_gps_time_second( stack_id, gps_time_s ) != ALC_SYNC_OK )
    {
        return SMTC_MODEM_RC_NO_TIME;
    }
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_trigger_alcsync_request( uint8_t stack_id )
{
    RETURN_BUSY_IF_TEST_MODE( );
    if( lorawan_api_isjoined( stack_id ) != JOINED )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    if( lorawan_alcsync_request_sync( stack_id, true ) != ALC_SYNC_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
}

#endif  // ADD_SMTC_ALC_SYNC

smtc_modem_return_code_t smtc_modem_trig_lorawan_mac_request( uint8_t                               stack_id,
                                                              smtc_modem_lorawan_mac_request_mask_t cid_request_mask )
{
    RETURN_BUSY_IF_TEST_MODE( );
    if( lorawan_api_isjoined( stack_id ) != JOINED )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    if( lorawan_cid_request_add_task( stack_id, cid_request_mask, 0 ) == LORAWAN_MANAGEMENT_OK )
    {
        return SMTC_MODEM_RC_OK;
    }
    return SMTC_MODEM_RC_INVALID;
}

smtc_modem_return_code_t smtc_modem_get_lorawan_mac_time( uint8_t stack_id, uint32_t* gps_time_s,
                                                          uint32_t* gps_fractional_s )
{
    RETURN_BUSY_IF_TEST_MODE( );

    // Check first if a lorawan dev time answer timestamp is available (proof that a network time was once
    // downlinked)
    if( lorawan_api_get_timestamp_last_device_time_ans_s( stack_id ) == 0 )
    {
        return SMTC_MODEM_RC_NO_TIME;
    }

    // A time is available: convert it and return to user
    lorawan_api_convert_rtc_to_gps_epoch_time( smtc_modem_hal_get_time_in_ms( ), gps_time_s, gps_fractional_s,
                                               stack_id );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_lorawan_link_check_data( uint8_t stack_id, uint8_t* margin, uint8_t* gw_cnt )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( margin );
    RETURN_INVALID_IF_NULL( gw_cnt );

    if( lorawan_api_get_link_check_ans( margin, gw_cnt, stack_id ) != OKLORAWAN )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    else
    {
        return SMTC_MODEM_RC_OK;
    }
}

/*
 * -----------------------------------------------------------------------------
 * ----------- CLASS B/C MODEM FUNCTIONS ---------------------------------------
 */

smtc_modem_return_code_t smtc_modem_set_class( uint8_t stack_id, smtc_modem_class_t lorawan_class )
{
    RETURN_BUSY_IF_TEST_MODE( );
    if( lorawan_api_isjoined( stack_id ) != JOINED )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    switch( lorawan_class )
    {
    case SMTC_MODEM_CLASS_A: {
#ifdef ADD_CLASS_B
        lorawan_class_b_management_enable( stack_id, false, 0 );
#endif
#ifdef ADD_CLASS_C
        lorawan_api_class_c_enabled( false, stack_id );
#endif
        break;
    }
#ifdef ADD_CLASS_B
    case SMTC_MODEM_CLASS_B: {
#ifdef ADD_CLASS_C
        lorawan_api_class_c_enabled( false, stack_id );
#endif  // ADD_CLASS_C
        lorawan_class_b_management_enable( stack_id, true, 0 );
        break;
    }
#endif  // ADD_CLASS_B
#ifdef ADD_CLASS_C
    case SMTC_MODEM_CLASS_C: {
#ifdef ADD_CLASS_B
        lorawan_class_b_management_enable( stack_id, false, 0 );
#endif  // ADD_CLASS_B
        lorawan_api_class_c_enabled( true, stack_id );
        break;
    }
#endif  // ADD_CLASS_C
    default:
        return SMTC_MODEM_RC_INVALID;
        break;
    }
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_class_b_set_ping_slot_periodicity(
    uint8_t stack_id, smtc_modem_class_b_ping_slot_periodicity_t ping_slot_periodicity )
{
    RETURN_BUSY_IF_TEST_MODE( );
    if( lorawan_api_isjoined( stack_id ) != JOINED )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    if( lorawan_api_set_ping_slot_periodicity( ping_slot_periodicity, stack_id ) != OKLORAWAN )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_class_b_get_ping_slot_periodicity(
    uint8_t stack_id, smtc_modem_class_b_ping_slot_periodicity_t* ping_slot_periodicity )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( ping_slot_periodicity );

    *ping_slot_periodicity =
        ( smtc_modem_class_b_ping_slot_periodicity_t ) lorawan_api_get_ping_slot_periodicity( stack_id );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_multicast_set_grp_config( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                              uint32_t      mc_grp_addr,
                                                              const uint8_t mc_nwk_skey[SMTC_MODEM_KEY_LENGTH],
                                                              const uint8_t mc_app_skey[SMTC_MODEM_KEY_LENGTH] )
{
#if defined( SMTC_MULTICAST )
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( mc_nwk_skey );
    RETURN_INVALID_IF_NULL( mc_app_skey );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_set_group_address( mc_grp_id, mc_grp_addr, stack_id );

    if( rc == LORAWAN_MC_RC_OK )
    {
        rc = lorawan_api_multicast_set_group_session_keys( mc_grp_id, mc_nwk_skey, mc_app_skey, stack_id );
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
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( mc_grp_addr );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_get_group_address( mc_grp_id, mc_grp_addr, stack_id );

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
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_c_start_session( mc_grp_id, freq, dr, stack_id );

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
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( is_session_started );
    RETURN_INVALID_IF_NULL( freq );
    RETURN_INVALID_IF_NULL( dr );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc =
        lorawan_api_multicast_c_get_session_status( mc_grp_id, is_session_started, freq, dr, stack_id );

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
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_c_stop_session( mc_grp_id, stack_id );

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
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_c_stop_all_sessions( stack_id );

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
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc =
        lorawan_api_multicast_b_start_session( mc_grp_id, freq, dr, ping_slot_periodicity, stack_id );

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
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( is_session_started );
    RETURN_INVALID_IF_NULL( is_session_waiting_for_beacon );
    RETURN_INVALID_IF_NULL( freq );
    RETURN_INVALID_IF_NULL( dr );
    RETURN_INVALID_IF_NULL( ping_slot_periodicity );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_b_get_session_status(
        mc_grp_id, is_session_started, is_session_waiting_for_beacon, freq, dr, ping_slot_periodicity, stack_id );

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
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_b_stop_session( mc_grp_id, stack_id );

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
    RETURN_BUSY_IF_TEST_MODE( );

    smtc_modem_return_code_t modem_rc;
    lorawan_multicast_rc_t   rc = lorawan_api_multicast_b_stop_all_sessions( stack_id );

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

/*
 * -----------------------------------------------------------------------------
 * ---------------------- DEBUG PURPOSE FUNCTIONS  -----------------------------
 */

smtc_modem_return_code_t smtc_modem_debug_set_duty_cycle_state( bool enable )
{
    RETURN_BUSY_IF_TEST_MODE( );

    if( smtc_duty_cycle_enable_set( ( enable == false ) ? SMTC_DTC_FULL_DISABLED : SMTC_DTC_ENABLED ) != SMTC_DTC_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_debug_connect_with_abp( uint8_t stack_id, uint32_t dev_addr,
                                                            uint8_t nwk_skey[SMTC_MODEM_KEY_LENGTH],
                                                            uint8_t app_skey[SMTC_MODEM_KEY_LENGTH] )
{
    RETURN_BUSY_IF_TEST_MODE( );
    RETURN_INVALID_IF_NULL( nwk_skey );
    RETURN_INVALID_IF_NULL( app_skey );

    lorawan_api_set_activation_mode( ACTIVATION_MODE_ABP, stack_id );

    if( lorawan_api_devaddr_set( dev_addr, stack_id ) != OKLORAWAN )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    if( smtc_secure_element_set_key( SMTC_SE_NWK_S_ENC_KEY, nwk_skey, stack_id ) != SMTC_SE_RC_SUCCESS )
    {
        return SMTC_MODEM_RC_INVALID;
    }
    if( smtc_secure_element_set_key( SMTC_SE_APP_S_KEY, app_skey, stack_id ) != SMTC_SE_RC_SUCCESS )
    {
        return SMTC_MODEM_RC_INVALID;
    }

    if( smtc_modem_join_network( stack_id ) != SMTC_MODEM_RC_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static smtc_modem_return_code_t smtc_modem_send_tx( uint8_t stack_id, uint8_t f_port, bool confirmed,
                                                    const uint8_t* payload, uint8_t payload_length, bool emergency )
{
    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    SMTC_MODEM_HAL_TRACE_INFO( "add task user \n" );
    if( lorawan_api_isjoined( stack_id ) != JOINED )
    {
        return_code = SMTC_MODEM_RC_FAIL;
    }
    else if( payload_length > SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH )
    {
        return_code = SMTC_MODEM_RC_INVALID;
    }
    else if( ( ( ( f_port == 0 ) || ( f_port >= 224 ) ) && !lorawan_api_modem_certification_is_enabled( stack_id ) ) )
    {
        return_code = SMTC_MODEM_RC_INVALID;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s port %d is forbidden \n", __func__, f_port );
    }
    else
    {
        lorawan_send_add_task( stack_id, f_port, true, confirmed, payload, payload_length, emergency, 0 );
    }

    //@todo indicate that can't accept a new task because task already enqueued     //   SetModemBusy ();

    return return_code;
}

static smtc_modem_return_code_t smtc_modem_custom_dr_distribution_to_tab(
    uint16_t mask_dr_allowed, const uint8_t adr_custom_data[SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH],
    uint8_t adr_distribution[SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH] )
{
    uint8_t cpt_tmp = 0;

    memset( adr_distribution, 0, SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH );
    for( uint8_t i = 0; i < SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH; i++ )
    {
        if( adr_custom_data[i] > 15 )  // DR are defined from 0 to 15 by definition in LoRaWAN spec
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "ADR with DataRate out of range\n" );
            return SMTC_MODEM_RC_INVALID;
        }
        if( ( ( mask_dr_allowed >> adr_custom_data[i] ) & 0x01 ) == 1 )
        {
            cpt_tmp++;
            adr_distribution[adr_custom_data[i]]++;
        }
    }
    if( cpt_tmp == 0 )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "ADR with a bad DataRate value\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    return SMTC_MODEM_RC_OK;
}

#if defined( USE_LR11XX_CE )
static void modem_store_key_context( void )
{
    modem_key_ctx_t ctx = {
        .appkey_crc_status = modem_key_status,
        .appkey_crc        = modem_key_crc,
        .rfu               = { 0 },
    };
    ctx.crc = crc( ( uint8_t* ) &ctx, sizeof( ctx ) - 4 );
    smtc_modem_hal_context_store( CONTEXT_MODEM, 0, ( uint8_t* ) &ctx, sizeof( ctx ) );
    // dummy context reading to ensure context store is done before exiting the function
    smtc_modem_hal_context_restore( CONTEXT_LORAWAN_STACK, 0, ( uint8_t* ) &ctx, sizeof( ctx ) );
}

static void modem_load_appkey_context( void )
{
    modem_key_ctx_t ctx;
    smtc_modem_hal_context_restore( CONTEXT_MODEM, 0, ( uint8_t* ) &ctx, sizeof( ctx ) );

    if( crc( ( uint8_t* ) &ctx, sizeof( ctx ) - 4 ) == ctx.crc )
    {
        modem_key_status = ctx.appkey_crc_status;
        modem_key_crc    = ctx.appkey_crc;
    }
    else
    {
        // crc of the context is not good anymore => restore key stasus to default
        ctx.appkey_crc        = 0;
        ctx.appkey_crc_status = MODEM_KEY_CRC_STATUS_INVALID;
        ctx.crc               = crc( ( uint8_t* ) &ctx, sizeof( ctx ) - 4 );
        smtc_modem_hal_context_store( CONTEXT_MODEM, 0, ( uint8_t* ) &ctx, sizeof( ctx ) );
        // dummy context reading to ensure context store is done before exiting the function
        smtc_modem_hal_context_restore( CONTEXT_LORAWAN_STACK, 0, ( uint8_t* ) &ctx, sizeof( ctx ) );
    }
}
#endif

/* --- EOF ------------------------------------------------------------------ */
