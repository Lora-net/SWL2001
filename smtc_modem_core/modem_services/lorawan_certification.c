/**
 * @file      lorawan_certification.c
 *
 * @brief     LoRaWAN Certification Protocol Implementation, Package Identifier 6, Package Version 1
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

#include "lorawan_certification.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_api.h"
#include "modem_services_common.h"
#include "lorawan_api.h"
#include "smtc_modem_test_api.h"
#include "modem_context.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lorawan_certification_init( lorawan_certification_t* lorawan_certification )
{
    memset( lorawan_certification, 0, sizeof( lorawan_certification_t ) );
    lorawan_certification->ul_periodicity = 5;
}

void lorawan_certification_set_enabled( lorawan_certification_t* lorawan_certification, bool enabled )
{
    if( ( lorawan_certification->enabled == false ) && ( enabled == true ) )
    {
        lorawan_certification_init( lorawan_certification );
    }
    lorawan_certification->enabled = enabled;
}

bool lorawan_certification_get_enabled( lorawan_certification_t* lorawan_certification )
{
    return lorawan_certification->enabled;
}

uint16_t lorawan_certification_get_ul_periodicity( lorawan_certification_t* lorawan_certification )
{
    return lorawan_certification->ul_periodicity;
}

bool lorawan_certification_get_frame_type( lorawan_certification_t* lorawan_certification )
{
    return lorawan_certification->frame_type;
}

void lorawan_certification_get_cw_config( lorawan_certification_t* lorawan_certification, uint16_t* timeout_s,
                                          uint32_t* frequency, int8_t* tx_power )
{
    *timeout_s = lorawan_certification->cw_timeout_s;
    *frequency = lorawan_certification->cw_frequency;
    *tx_power  = lorawan_certification->cw_tx_power;
}

bool lorawan_certification_is_cw_running( lorawan_certification_t* lorawan_certification )
{
    return lorawan_certification->cw_running;
}

void lorawan_certification_cw_set_as_stopped( lorawan_certification_t* lorawan_certification )
{
    lorawan_certification->cw_running = false;
}

bool lorawan_certification_get_beacon_rx_status_ind_ctrl( lorawan_certification_t* lorawan_certification )
{
    return lorawan_certification->beacon_rx_status_ind_ctrl;
}

lorawan_certification_class_t lorawan_certification_get_requested_class(
    lorawan_certification_t* lorawan_certification )
{
    return lorawan_certification->class_requested;
}

lorawan_certification_parser_ret_t lorawan_certification_parser( lorawan_certification_t* lorawan_certification,
                                                                 uint8_t* rx_buffer, uint8_t rx_buffer_length,
                                                                 uint8_t* tx_buffer, uint8_t* tx_buffer_length,
                                                                 uint8_t* tx_fport )
{
    uint8_t rx_buffer_index = 0;
    *tx_buffer_length       = 0;

    if( lorawan_certification->enabled == false )
    {
        return LORAWAN_CERTIFICATION_RET_NOTHING;
    }

    // count empty packet (ACK)
    lorawan_certification->rx_app_cnt++;

    if( rx_buffer_length == 0 )
    {
        return LORAWAN_CERTIFICATION_RET_APP_UL;
    }

    switch( rx_buffer[rx_buffer_index] )
    {
    case LORAWAN_CERTIFICATION_PACKAGE_VERSION_REQ:
        if( rx_buffer_length == LORAWAN_CERTIFICATION_PACKAGE_VERSION_REQ_SIZE )
        {
            tx_buffer[( *tx_buffer_length )++] = LORAWAN_CERTIFICATION_PACKAGE_VERSION_ANS;
            tx_buffer[( *tx_buffer_length )++] = LORAWAN_CERTIFICATION_PACKAGE_IDENTIFIER;
            tx_buffer[( *tx_buffer_length )++] = LORAWAN_CERTIFICATION_PACKAGE_VERSION;

            *tx_fport = 224;
            return LORAWAN_CERTIFICATION_RET_CERTIF_UL;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }
        break;

    case LORAWAN_CERTIFICATION_DUT_REST_REQ:
        if( rx_buffer_length == LORAWAN_CERTIFICATION_DUT_REST_REQ_SIZE )
        {
            smtc_modem_hal_reset_mcu( );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }

        break;
    case LORAWAN_CERTIFICATION_DUT_JOIN_REQ:
        if( rx_buffer_length == LORAWAN_CERTIFICATION_DUT_JOIN_REQ_SIZE )
        {
            lorawan_api_set_activation_mode( ACTIVATION_MODE_OTAA );
            set_modem_status_modem_joined( false );
            lorawan_api_join_status_clear( );
            modem_supervisor_add_task_join( );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }

        break;
    case LORAWAN_CERTIFICATION_SWITCH_CLASS_REQ:
        if( rx_buffer_length == LORAWAN_CERTIFICATION_SWITCH_CLASS_REQ_SIZE )
        {
            if( rx_buffer[1] == ( uint8_t ) LORAWAN_CERTIFICATION_CLASS_A )
            {
                lorawan_certification->class_requested = LORAWAN_CERTIFICATION_CLASS_A;
            }
            else if( rx_buffer[1] == ( uint8_t ) LORAWAN_CERTIFICATION_CLASS_B )
            {
                lorawan_certification->class_requested = LORAWAN_CERTIFICATION_CLASS_B;
            }
            else if( rx_buffer[1] == ( uint8_t ) LORAWAN_CERTIFICATION_CLASS_C )
            {
                lorawan_certification->class_requested = LORAWAN_CERTIFICATION_CLASS_C;
            }
            return LORAWAN_CERTIFICATION_RET_SWITCH_CLASS;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }

        break;
    case LORAWAN_CERTIFICATION_ADR_BIT_CHANGE_REQ:
        if( rx_buffer_length == LORAWAN_CERTIFICATION_ADR_BIT_CHANGE_REQ_SIZE )
        {
            if( rx_buffer[1] == ( uint8_t ) LORAWAN_CERTIFICATION_ADR_OFF )
            {
                uint8_t adr_custom_data[16] = { 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
                                                0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05 };
                smtc_modem_adr_set_profile( 1, SMTC_MODEM_ADR_PROFILE_CUSTOM, adr_custom_data );
            }
            else
            {
                smtc_modem_adr_set_profile( 1, SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED, NULL );
            }
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }

        break;
    case LORAWAN_CERTIFICATION_REGIONAL_DUTY_CYCLE_CTRL_REQ:
        if( rx_buffer_length == LORAWAN_CERTIFICATION_REGIONAL_DUTY_CYCLE_CTRL_REQ_SIZE )
        {
            if( rx_buffer[1] == ( uint8_t ) LORAWAN_CERTIFICATION_DUTY_CYCLE_OFF )
            {
                smtc_modem_test_duty_cycle_app_activate( false );
            }
            else
            {
                smtc_modem_test_duty_cycle_app_activate( true );
            }
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }

        break;
    case LORAWAN_CERTIFICATION_TX_PERIODICITY_CHANGE_REQ:
        if( rx_buffer_length == LORAWAN_CERTIFICATION_TX_PERIODICITY_CHANGE_REQ_SIZE )
        {
            lorawan_certification->ul_periodicity = ( uint8_t ) lorawan_certification_periodicity_table[rx_buffer[1]];
            smtc_modem_alarm_start_timer( lorawan_certification->ul_periodicity );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }

        break;
    case LORAWAN_CERTIFICATION_TX_FRAME_CTRL_REQ:
        if( rx_buffer_length <= LORAWAN_CERTIFICATION_TX_FRAME_CTRL_REQ_SIZE )
        {
            if( rx_buffer[1] == ( uint8_t ) LORAWAN_CERTIFICATION_FRAME_TYPE_UNCONFIRMED )
            {
                lorawan_certification->frame_type = false;
            }
            else if( rx_buffer[1] == ( uint8_t ) LORAWAN_CERTIFICATION_FRAME_TYPE_CONFIRMED )
            {
                lorawan_certification->frame_type = true;
            }
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }

        break;
    case LORAWAN_CERTIFICATION_ECHO_PLAY_REQ:
        if( rx_buffer_length <= LORAWAN_CERTIFICATION_ECHO_PLAY_REQ_SIZE )
        {
            uint8_t max_len = MIN( rx_buffer_length, lorawan_api_next_max_payload_length_get( ) - *tx_buffer_length );

            // The first byte is the command ID
            max_len -= 1;

            tx_buffer[( *tx_buffer_length )++] = LORAWAN_CERTIFICATION_ECHO_PLAY_ANS;
            for( uint8_t i = 0; i < max_len; i++ )
            {
                tx_buffer[( *tx_buffer_length )++] = ( rx_buffer[i + 1] + 1 ) % 256;
            }

            *tx_fport = 224;
            return LORAWAN_CERTIFICATION_RET_CERTIF_UL;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }

        break;
    case LORAWAN_CERTIFICATION_RX_APP_CNT_REQ:
        if( rx_buffer_length == LORAWAN_CERTIFICATION_RX_APP_CNT_REQ_SIZE )
        {
            tx_buffer[( *tx_buffer_length )++] = LORAWAN_CERTIFICATION_RX_APP_CNT_ANS;
            tx_buffer[( *tx_buffer_length )++] = ( lorawan_certification->rx_app_cnt & 0xFF );
            tx_buffer[( *tx_buffer_length )++] = ( lorawan_certification->rx_app_cnt >> 8 ) & 0xFF;

            *tx_fport = 224;
            return LORAWAN_CERTIFICATION_RET_CERTIF_UL;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }

        break;
    case LORAWAN_CERTIFICATION_RX_APP_CNT_RESET_REQ:
        if( rx_buffer_length == LORAWAN_CERTIFICATION_RX_APP_CNT_RESET_REQ_SIZE )
        {
            lorawan_certification->rx_app_cnt = 0;
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }

        break;
    case LORAWAN_CERTIFICATION_LINK_CHECK_REQ:

        if( rx_buffer_length == LORAWAN_CERTIFICATION_LINK_CHECK_REQ_SIZE )
        {
            modem_supervisor_add_task_link_check_req( 0 );
            return LORAWAN_CERTIFICATION_RET_LINK_CHECK;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }

        break;
    case LORAWAN_CERTIFICATION_DEVICE_TIME_REQ:
        if( rx_buffer_length == LORAWAN_CERTIFICATION_DEVICE_TIME_REQ_SIZE )
        {
            modem_supervisor_add_task_device_time_req( 0 );
            return LORAWAN_CERTIFICATION_RET_DEVICE_TIME;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }

        break;
    case LORAWAN_CERTIFICATION_PING_SLOT_INFO_REQ:
        if( rx_buffer_length == LORAWAN_CERTIFICATION_PING_SLOT_INFO_REQ_SIZE )
        {
            lorawan_api_set_ping_slot_periodicity( rx_buffer[1] );
            modem_supervisor_add_task_ping_slot_info_req( 0 );
            return LORAWAN_CERTIFICATION_RET_PING_SLOT;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }

        break;
    case LORAWAN_CERTIFICATION_BEACON_RX_STATUS_IND_CTRL:
        if( rx_buffer_length == LORAWAN_CERTIFICATION_BEACON_RX_STATUS_IND_CTRL_SIZE )
        {
            lorawan_certification->beacon_rx_status_ind_ctrl = rx_buffer[1];
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }
        break;
    case LORAWAN_CERTIFICATION_BEACON_CNT_REQ:
        if( rx_buffer_length == LORAWAN_CERTIFICATION_BEACON_CNT_REQ_SIZE )
        {
            tx_buffer[( *tx_buffer_length )++] = LORAWAN_CERTIFICATION_BEACON_CNT_ANS;
            tx_buffer[( *tx_buffer_length )++] = ( lorawan_certification->rx_beacon_cnt & 0xFF );
            tx_buffer[( *tx_buffer_length )++] = ( lorawan_certification->rx_beacon_cnt >> 8 ) & 0xFF;

            *tx_fport = 224;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }
        break;
    case LORAWAN_CERTIFICATION_BEACON_CNT_RST_REQ:
        if( rx_buffer_length == LORAWAN_CERTIFICATION_BEACON_CNT_RST_REQ_SIZE )
        {
            lorawan_certification->rx_beacon_cnt = 0;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }
        break;
    case LORAWAN_CERTIFICATION_TX_CW_REQ:
        if( rx_buffer_length == LORAWAN_CERTIFICATION_TX_CW_REQ_SIZE )
        {
            lorawan_certification->cw_running   = true;
            lorawan_certification->cw_timeout_s = ( rx_buffer[2] << 8 ) + rx_buffer[1];
            lorawan_certification->cw_frequency =
                ( ( rx_buffer[5] << 16 ) + ( rx_buffer[4] << 8 ) + rx_buffer[3] ) * 100;
            lorawan_certification->cw_tx_power = rx_buffer[6];

            // smtc_modem_leave_network( 0 );  // TODO STACK_ID not hard coded
            // smtc_modem_alarm_start_timer( lorawan_certification->cw_timeout_s );
            // smtc_modem_test_start( );
            // smtc_modem_test_tx_cw( lorawan_certification->cw_frequency, lorawan_certification->cw_tx_power );

            return LORAWAN_CERTIFICATION_RET_TX_CW;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }

        break;
    case LORAWAN_CERTIFICATION_DUT_FPORT_224_DISABLE_REQ:
        if( rx_buffer_length == LORAWAN_CERTIFICATION_DUT_FPORT_224_DISABLE_REQ_SIZE )
        {
            lorawan_certification->enabled = false;
            lorawan_api_modem_certification_set( false );
            smtc_modem_hal_reset_mcu( );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }

        break;
    case LORAWAN_CERTIFICATION_DUT_VERSION_REQ:
        if( rx_buffer_length == LORAWAN_CERTIFICATION_DUT_VERSION_REQ_SIZE )
        {
            // Ignore command if the payload length is greater than the max payload length allowed
            // by the region and datarate
            if( lorawan_api_next_max_payload_length_get( ) >= LORAWAN_CERTIFICATION_DUT_VERSION_ANS_SIZE )
            {
                tx_buffer[( *tx_buffer_length )++] = LORAWAN_CERTIFICATION_DUT_VERSION_ANS;

                smtc_modem_version_t         modem_version;
                smtc_modem_lorawan_version_t lorawan_version;
                smtc_modem_lorawan_version_t rp_version;

                smtc_modem_get_modem_version( &modem_version );
                smtc_modem_get_lorawan_version( &lorawan_version );
                smtc_modem_get_regional_params_version( &rp_version );

                // Firmware Version
                tx_buffer[( *tx_buffer_length )++] = modem_version.major;
                tx_buffer[( *tx_buffer_length )++] = modem_version.minor;
                tx_buffer[( *tx_buffer_length )++] = modem_version.patch;
                tx_buffer[( *tx_buffer_length )++] = 0;  // Revision

                // LoRaWAN Version
                tx_buffer[( *tx_buffer_length )++] = lorawan_version.major;
                tx_buffer[( *tx_buffer_length )++] = lorawan_version.minor;
                tx_buffer[( *tx_buffer_length )++] = lorawan_version.patch;
                tx_buffer[( *tx_buffer_length )++] = lorawan_version.revision;

                // Regional Parameters Version
                tx_buffer[( *tx_buffer_length )++] = rp_version.major;
                tx_buffer[( *tx_buffer_length )++] = rp_version.minor;
                tx_buffer[( *tx_buffer_length )++] = rp_version.patch;
                tx_buffer[( *tx_buffer_length )++] = rp_version.revision;

                *tx_fport = 224;
                return LORAWAN_CERTIFICATION_RET_CERTIF_UL;
            }
        }
        else
        {
            LOG_ERROR( "bad size\n" );
            return LORAWAN_CERTIFICATION_RET_NOTHING;
        }
        break;

    default:
        LOG_ERROR( "%s Illegal state\n ", __func__ );
        break;
    }
    return LORAWAN_CERTIFICATION_RET_NOTHING;
}

void lorawan_certification_build_beacon_rx_status_ind( lorawan_certification_t* lorawan_certification,
                                                       uint8_t* beacon_buffer, uint8_t beacon_buffer_length,
                                                       uint8_t* tx_buffer, uint8_t* tx_buffer_length, int8_t rssi,
                                                       int8_t snr, uint8_t beacon_dr, uint32_t beacon_freq )
{
    smtc_beacon_metadata_t beacon_metadata;
    lorawan_api_beacon_get_metadata( &beacon_metadata );

    if( beacon_metadata.last_beacon_lost_consecutively == 0 )
    {
        lorawan_certification->rx_beacon_cnt++;
    }

    uint8_t            beacon_sf;
    lr1mac_bandwidth_t beacon_bw;
    lorawan_api_lora_dr_to_sf_bw( beacon_dr, &beacon_sf, &beacon_bw );

    uint32_t epoch_time   = smtc_decode_beacon_epoch_time( beacon_buffer, beacon_sf );
    uint8_t  beacon_param = smtc_decode_beacon_param( beacon_buffer, beacon_sf );

    beacon_freq = beacon_freq / lorawan_api_get_frequency_factor( );

    tx_buffer[( *tx_buffer_length )++] = LORAWAN_CERTIFICATION_BEACON_RX_STATUS_IND;
    tx_buffer[( *tx_buffer_length )++] = 1;                                                     // Todo state ?
    tx_buffer[( *tx_buffer_length )++] = lorawan_certification->rx_beacon_cnt & 0xFF;           // RxCnt
    tx_buffer[( *tx_buffer_length )++] = ( lorawan_certification->rx_beacon_cnt >> 8 ) & 0xFF;  // RxCnt
    tx_buffer[( *tx_buffer_length )++] = beacon_freq & 0xFF;                                    // Beacon freq
    tx_buffer[( *tx_buffer_length )++] = ( beacon_freq >> 8 ) & 0xFF;                           // Beacon freq
    tx_buffer[( *tx_buffer_length )++] = ( beacon_freq >> 16 ) & 0xFF;                          // Beacon freq
    tx_buffer[( *tx_buffer_length )++] = beacon_dr;                                             // Beacon DR
    tx_buffer[( *tx_buffer_length )++] = rssi & 0xFF;                                           // Beacon RSSI
    tx_buffer[( *tx_buffer_length )++] = ( rssi >> 8 ) & 0xFF;                                  // Beacon RSSI
    tx_buffer[( *tx_buffer_length )++] = snr;                                                   // Beacon SNR
    tx_buffer[( *tx_buffer_length )++] = beacon_param;                                          // Param
    tx_buffer[( *tx_buffer_length )++] = epoch_time & 0xFF;                                     // Time
    tx_buffer[( *tx_buffer_length )++] = ( epoch_time >> 8 ) & 0xFF;                            // Time
    tx_buffer[( *tx_buffer_length )++] = ( epoch_time >> 16 ) & 0xFF;                           // Time
    tx_buffer[( *tx_buffer_length )++] = ( epoch_time >> 14 ) & 0xFF;                           // Time

    smtc_decode_beacon_gw_specific( beacon_buffer, beacon_sf, &tx_buffer[*tx_buffer_length] );  // GwSpecific 7 bytes
    *tx_buffer_length += 7;
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
