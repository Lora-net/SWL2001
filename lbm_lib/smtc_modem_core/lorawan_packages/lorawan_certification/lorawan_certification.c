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

#include "modem_supervisor_light.h"
#include "lorawan_api.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_duty_cycle.h"
#include "smtc_modem_test_api.h"
#include "modem_core.h"
#include "lora_basics_modem_version.h"
#include "lorawan_certification.h"
#include "modem_tx_protocol_manager.h"

#ifdef ADD_FUOTA
#include "lorawan_fragmentation_package.h"
#endif

#if defined( ADD_RELAY_TX )
#include "smtc_modem_relay_api.h"
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define NUMBER_OF_CERTIF_OBJ 1

/*!
 * \brief Returns the minimum value between a and b
 *
 * \param [in] a 1st value
 * \param [in] b 2nd value
 * \retval minValue Minimum value
 */
#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )

/**
 * @brief Check is the index is valid before accessing the object
 *
 */
#define IS_VALID_OBJECT_ID( x )                                      \
    do                                                               \
    {                                                                \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_OF_CERTIF_OBJ ); \
    } while( 0 )

/**
 * @brief Check is the index is valid before accessing the object
 *
 */
#define IS_VALID_STACK_ID( x )                  \
    do                                          \
    {                                           \
        if( x >= NUMBER_OF_STACKS )             \
        {                                       \
            return LORAWAN_CERTIFICATION_ERROR; \
        }                                       \
    } while( 0 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */
typedef struct certif_service_ctx_s
{
    uint32_t                timestamp_launch_ms[NUMBER_OF_CERTIF_OBJ];
    lorawan_certification_t lorawan_certification_obj[NUMBER_OF_CERTIF_OBJ];
} certif_service_ctx_t;

static certif_service_ctx_t certif_service_ctx;

#define timestamp_launch_ms certif_service_ctx.timestamp_launch_ms
#define lorawan_certification_obj certif_service_ctx.lorawan_certification_obj

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Callback called at task launch
 *
 * @param context_callback
 */
static void lorawan_certification_service_on_launch( void* service_id );

/**
 * @brief Callback called at the end of the task
 *
 * @param context_callback
 */
static void lorawan_certification_service_on_update( void* service_id );

/**
 * @brief Callback to handle the downlink received by the LoRaWAN layer
 *
 * @param rx_down_data
 */
static uint8_t lorawan_certification_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/**
 * @brief Get the service context linked to a stack_id
 *
 * @param [in] stack_id                     Stack identifier
 * @param [out] service_id                  Associated occurrence of this service
 * @return lorawan_certification_t*     Object context
 */
static lorawan_certification_t* lorawan_certification_get_ctx_from_stack_id( uint8_t stack_id, uint8_t* service_id );

/**
 * @brief Initialize the LoRaWAN Certification Package
 *
 * @param lorawan_certification
 */
static void lorawan_certification_init( lorawan_certification_t* lorawan_certification );

/**
 * @brief
 *
 * @param lorawan_certification
 * @param rx_buffer
 * @param rx_buffer_length
 * @param tx_buffer
 * @param tx_buffer_length
 * @return lorawan_certification_requested_tx_type_t
 */
static lorawan_certification_requested_tx_type_t lorawan_certification_parser(
    lorawan_certification_t* lorawan_certification, uint8_t* rx_buffer, uint8_t rx_buffer_length, uint8_t* tx_buffer,
    uint8_t* tx_buffer_length );

/**
 * @brief Build Class B Beacon Status Indication frame
 *
 * @param lorawan_certification
 * @param beacon_buffer
 * @param beacon_buffer_length
 * @param tx_buffer
 * @param tx_buffer_length
 * @param rssi
 * @param snr
 * @param beacon_dr
 * @param beacon_freq
 */
static void lorawan_certification_build_beacon_rx_status_ind( lorawan_certification_t* lorawan_certification,
                                                              uint8_t* beacon_buffer, uint8_t beacon_buffer_length,
                                                              uint8_t* tx_buffer, uint8_t* tx_buffer_length,
                                                              int8_t rssi, int8_t snr, uint8_t beacon_dr,
                                                              uint32_t beacon_freq );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lorawan_certification_services_init( uint8_t* service_id, uint8_t task_id,
                                          uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                          void ( **on_launch_callback )( void* ),
                                          void ( **on_update_callback )( void* ), void** context_callback )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG(
        " lorawan_certification_services_init task_id %d, service_id %d, CURRENT_STACK:%d \n", task_id, *service_id,
        CURRENT_STACK );
    IS_VALID_OBJECT_ID( *service_id );

    lorawan_certification_t* ctx = &lorawan_certification_obj[*service_id];
    memset( ctx, 0, sizeof( lorawan_certification_t ) );

    *downlink_callback  = lorawan_certification_service_downlink_handler;
    *on_launch_callback = lorawan_certification_service_on_launch;
    *on_update_callback = lorawan_certification_service_on_update;
    *context_callback   = ( void* ) service_id;
    ctx->task_id        = task_id;
    ctx->stack_id       = CURRENT_STACK;
    ctx->enabled        = lorawan_api_modem_certification_is_enabled( CURRENT_STACK );

    if( ctx->enabled == true )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "***********************************************\n" );
        SMTC_MODEM_HAL_TRACE_WARNING( "  LoRaWAN CERTIFICATION is ENABLED on stack %d\n", ctx->stack_id );
        SMTC_MODEM_HAL_TRACE_WARNING( "***********************************************\n" );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "LoRaWAN Certification is disabled on stack %d\n", ctx->stack_id );
    }

    lorawan_certification_init( ctx );

    if( ctx->enabled == true )
    {
        smtc_secure_element_restore_context( CURRENT_STACK );
        smodem_task task_certif       = { 0 };
        task_certif.id                = ctx->task_id;
        task_certif.stack_id          = ctx->stack_id;
        task_certif.priority          = TASK_MEDIUM_HIGH_PRIORITY;
        task_certif.time_to_execute_s = smtc_modem_hal_get_time_in_s( );

        modem_supervisor_add_task( &task_certif );
    }
}

lorawan_certification_ret_t lorawan_certification_get_enabled( uint8_t stack_id, bool* enabled )
{
    IS_VALID_STACK_ID( stack_id );

    uint8_t                  service_id;
    lorawan_certification_t* ctx = lorawan_certification_get_ctx_from_stack_id( stack_id, &service_id );
    if( ctx == NULL )
    {
        *enabled = false;
        return LORAWAN_CERTIFICATION_ERROR;
    }
    *enabled = ctx->enabled;
    return LORAWAN_CERTIFICATION_OK;
}

lorawan_certification_ret_t lorawan_certification_set_enabled( uint8_t stack_id, bool enabled )
{
    IS_VALID_STACK_ID( stack_id );

    uint8_t                  service_id;
    lorawan_certification_t* ctx = lorawan_certification_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
    {
        return LORAWAN_CERTIFICATION_ERROR;
    }

    if( ctx->enabled != enabled )
    {
        ctx->enabled = enabled;
        SMTC_MODEM_HAL_TRACE_INFO( "LoRaWAN Certification is %s on stack %d \n",
                                   ( ctx->enabled == true ? "enabled" : "disabled" ), ctx->stack_id );

        lorawan_api_modem_certification_set( ctx->enabled, ctx->stack_id );

        lorawan_certification_init( ctx );

        if( ctx->enabled == true )
        {
            smodem_task task_certif       = { 0 };
            task_certif.id                = ctx->task_id;
            task_certif.stack_id          = ctx->stack_id;
            task_certif.priority          = TASK_MEDIUM_HIGH_PRIORITY;
            task_certif.time_to_execute_s = smtc_modem_hal_get_time_in_s( );

            modem_supervisor_add_task( &task_certif );
        }
    }
    return LORAWAN_CERTIFICATION_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lorawan_certification_service_on_launch( void* service_id )
{
    uint8_t idx = *( ( uint8_t* ) service_id );
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " certification_service_on_launch service_id %d \n", idx );

    IS_VALID_OBJECT_ID( idx );

    timestamp_launch_ms[idx] = smtc_modem_hal_get_time_in_ms( );

    if( lorawan_certification_obj[idx].cw_running == true )
    {
        lorawan_certification_obj[idx].cw_running = false;
        smtc_modem_test_stop( );
    }

    if( lorawan_api_isjoined( lorawan_certification_obj[idx].stack_id ) != JOINED )
    {
        lorawan_certification_obj[idx].is_tx_requested = LORAWAN_CERTIFICATION_JOIN_REQ;
    }

    status_lorawan_t status_lorawan = ERRORLORAWAN;
    switch( lorawan_certification_obj[idx].is_tx_requested )
    {
    case LORAWAN_CERTIFICATION_TX_CERTIF_REQ:
    {
        status_lorawan = tx_protocol_manager_request(
            TX_PROTOCOL_TRANSMIT_LORA_CERTIFICATION, LORAWAN_CERTIFICATION_FPORT,
            ( lorawan_certification_obj[idx].tx_buffer_length == 0 ) ? false : true,
            lorawan_certification_obj[idx].tx_buffer, lorawan_certification_obj[idx].tx_buffer_length,
            ( lorawan_certification_obj[idx].frame_type == false ) ? UNCONF_DATA_UP : CONF_DATA_UP,
            timestamp_launch_ms[idx], lorawan_certification_obj[idx].stack_id );
        break;
    }
    case LORAWAN_CERTIFICATION_TX_MAC_REQ:
    {
        status_lorawan = tx_protocol_manager_request(
            TX_PROTOCOL_TRANSMIT_CID, 0, false, lorawan_certification_obj[idx].cid_req_list,
            lorawan_certification_obj[idx].cid_req_list_size, 0, timestamp_launch_ms[idx],
            lorawan_certification_obj[idx].stack_id );
        lorawan_certification_obj[idx].cid_req_list_size = 0;
        break;
    }
    case LORAWAN_CERTIFICATION_JOIN_REQ:
    {
        if( lorawan_api_isjoined( lorawan_certification_obj[idx].stack_id ) != JOINED )
        {
            lorawan_join_add_task( lorawan_certification_obj[idx].stack_id );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_NO_TX_REQ:
    default:
    {
        uint8_t buffer_tx_tmp[] = { 0x11, 0x12, 0x13, 0x14 };
        status_lorawan          = tx_protocol_manager_request(
            TX_PROTOCOL_TRANSMIT_LORA_CERTIFICATION, 1, true, buffer_tx_tmp, sizeof( buffer_tx_tmp ),
            ( lorawan_certification_obj[idx].frame_type == false ) ? UNCONF_DATA_UP : CONF_DATA_UP,
            timestamp_launch_ms[idx], lorawan_certification_obj[idx].stack_id );
        break;
    }
    }
    if( ( status_lorawan == OKLORAWAN ) &&
        ( lorawan_certification_obj[idx].is_tx_requested != LORAWAN_CERTIFICATION_JOIN_REQ ) )
    {
        lorawan_certification_obj[idx].is_tx_requested = LORAWAN_CERTIFICATION_NO_TX_REQ;
    }
}

static void lorawan_certification_service_on_update( void* service_id )
{
    uint8_t idx = *( ( uint8_t* ) service_id );
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " certification_service_on_update service_id %d (en:%u) \n", idx,
                                       lorawan_certification_obj[idx].enabled );
    IS_VALID_OBJECT_ID( idx );

    smodem_task task_certif = { 0 };
    task_certif.id          = lorawan_certification_obj[idx].task_id;
    task_certif.priority    = TASK_MEDIUM_HIGH_PRIORITY;
    task_certif.stack_id    = lorawan_certification_obj[idx].stack_id;

    if( ( lorawan_api_isjoined( lorawan_certification_obj[idx].stack_id ) == JOINED ) &&
        ( lorawan_certification_obj[idx].is_tx_requested == LORAWAN_CERTIFICATION_JOIN_REQ ) )
    {
        lorawan_certification_obj[idx].is_tx_requested = LORAWAN_CERTIFICATION_NO_TX_REQ;
    }

    if( lorawan_certification_obj[idx].enabled == true )
    {
        if( lorawan_certification_obj[idx].cw_running == true )
        {
            task_certif.time_to_execute_s = ( timestamp_launch_ms[idx] + smtc_modem_hal_get_time_in_ms( ) +
                                              ( lorawan_certification_obj[idx].cw_timeout_s * 1000UL ) ) /
                                            1000;
        }
        else
        {
            task_certif.time_to_execute_s =
                ( timestamp_launch_ms[idx] + ( lorawan_certification_obj[idx].ul_periodicity_s * 1000UL ) ) / 1000UL;
        }

        modem_supervisor_add_task( &task_certif );
    }
}

static uint8_t lorawan_certification_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    uint8_t stack_id = rx_down_data->stack_id;

    if( stack_id >= NUMBER_OF_CERTIF_OBJ )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "%s: stack id not valid %u \n", __func__, stack_id );
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    uint8_t service_id = 0xFF;
    for( uint8_t i = 0; i < NUMBER_OF_CERTIF_OBJ; i++ )
    {
        if( lorawan_certification_obj[i].stack_id == stack_id )
        {
            service_id = i;
            break;
        }
    }

    if( service_id == 0xFF )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( lorawan_certification_obj[service_id].enabled == false )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( rx_down_data->rx_metadata.is_a_join_accept == true )
    {
        task_id_t current_task_id = modem_supervisor_get_task( )->next_task_id;

        if( current_task_id != lorawan_certification_obj[service_id].task_id )
        {
            lorawan_certification_service_on_update( &service_id );
        }

#if defined( ADD_SMTC_ALC_SYNC )
        lorawan_alcsync_set_enabled( stack_id, true );
#endif
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( rx_down_data->rx_metadata.rx_window == RECEIVE_ON_RXBEACON )
    {
        if( lorawan_certification_obj[service_id].beacon_rx_status_ind_ctrl == true )
        {
            // Send data beacon to testing tool
            lorawan_certification_build_beacon_rx_status_ind(
                &lorawan_certification_obj[service_id], rx_down_data->rx_payload, rx_down_data->rx_payload_size,
                lorawan_certification_obj[service_id].tx_buffer,
                &lorawan_certification_obj[service_id].tx_buffer_length, rx_down_data->rx_metadata.rx_rssi,
                rx_down_data->rx_metadata.rx_snr, rx_down_data->rx_metadata.rx_datarate,
                rx_down_data->rx_metadata.rx_frequency_hz );

            SMTC_MODEM_HAL_TRACE_ARRAY( "BEACON to testing tool", lorawan_certification_obj[service_id].tx_buffer,
                                        lorawan_certification_obj[service_id].tx_buffer_length );
            lorawan_certification_obj[service_id].is_tx_requested = LORAWAN_CERTIFICATION_TX_CERTIF_REQ;
        }
    }
    else
    {
        // count empty packet (like ACK) and all downlink frames except from Fport 0
        if( ( rx_down_data->rx_metadata.rx_ack_bit == true ) ||
            ( ( rx_down_data->rx_metadata.rx_fport_present == true ) && ( rx_down_data->rx_metadata.rx_fport != 0 ) ) )
        {
            lorawan_certification_obj[service_id].rx_app_cnt++;
        }
        else
        {
            return MODEM_DOWNLINK_UNCONSUMED;
        }

        if( ( rx_down_data->rx_metadata.rx_fport_present == false ) || ( rx_down_data->rx_metadata.rx_fport == 0 ) )
        {
            return MODEM_DOWNLINK_UNCONSUMED;
        }

        SMTC_MODEM_HAL_TRACE_PRINTF( "on port %u\n", rx_down_data->rx_metadata.rx_fport );
        if( rx_down_data->rx_metadata.rx_fport == LORAWAN_CERTIFICATION_FPORT )
        {
            lorawan_certification_obj[service_id].is_tx_requested = lorawan_certification_parser(
                &lorawan_certification_obj[service_id], rx_down_data->rx_payload, rx_down_data->rx_payload_size,
                lorawan_certification_obj[service_id].tx_buffer,
                &lorawan_certification_obj[service_id].tx_buffer_length );

            task_id_t current_task_id = modem_supervisor_get_task( )->next_task_id;
            if( current_task_id != lorawan_certification_obj[service_id].task_id )
            {
                // To recompute time for the next certif uplink in case received downlink when another service is
                // running
                lorawan_certification_service_on_update( &service_id );
            }
        }
    }
    return MODEM_DOWNLINK_CONSUMED;
}

static lorawan_certification_t* lorawan_certification_get_ctx_from_stack_id( uint8_t stack_id, uint8_t* service_id )
{
    lorawan_certification_t* ctx = NULL;
    for( uint8_t i = 0; i < NUMBER_OF_CERTIF_OBJ; i++ )
    {
        if( lorawan_certification_obj[i].stack_id == stack_id )
        {
            ctx         = &lorawan_certification_obj[i];
            *service_id = i;
            break;
        }
    }

    return ctx;
}

static void lorawan_certification_init( lorawan_certification_t* lorawan_certification )
{
    lorawan_certification->ul_periodicity_s =
        lorawan_certification_periodicity_table_s[LORAWAN_CERTIFICATION_PERIODICITY_5];
    lorawan_certification->frame_type = false;

    lorawan_certification->is_tx_requested = LORAWAN_CERTIFICATION_JOIN_REQ;
}

static lorawan_certification_requested_tx_type_t lorawan_certification_parser(
    lorawan_certification_t* lorawan_certification, uint8_t* rx_buffer, uint8_t rx_buffer_length, uint8_t* tx_buffer,
    uint8_t* tx_buffer_length )
{
    uint8_t rx_buffer_index = 0;
    *tx_buffer_length       = 0;

    lorawan_certification_requested_tx_type_t ret = LORAWAN_CERTIFICATION_NO_TX_REQ;

    if( rx_buffer_length == 0 )
    {
        return ret;
    }

    SMTC_MODEM_HAL_TRACE_ARRAY( "certif parser buffer", rx_buffer, rx_buffer_length );
    lorawan_certification_cid_tcl_t cid = ( lorawan_certification_cid_tcl_t ) rx_buffer[rx_buffer_index];
    switch( cid )
    {
    case LORAWAN_CERTIFICATION_PACKAGE_VERSION_REQ:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_PACKAGE_VERSION_REQ_SIZE )
        {
            tx_buffer[( *tx_buffer_length )++] = LORAWAN_CERTIFICATION_PACKAGE_VERSION_ANS;
            tx_buffer[( *tx_buffer_length )++] = LORAWAN_CERTIFICATION_PACKAGE_IDENTIFIER;
            tx_buffer[( *tx_buffer_length )++] = LORAWAN_CERTIFICATION_PACKAGE_VERSION;

            ret = LORAWAN_CERTIFICATION_TX_CERTIF_REQ;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_DUT_RESET_REQ:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_DUT_RESET_REQ_SIZE )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Certif mcu reset\n" );
            smtc_modem_hal_reset_mcu( );
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_DUT_JOIN_REQ:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_DUT_JOIN_REQ_SIZE )
        {
            // leave network
            lorawan_api_join_status_clear( lorawan_certification_obj->stack_id );
            lorawan_api_set_activation_mode( ACTIVATION_MODE_OTAA, lorawan_certification_obj->stack_id );
            ret = LORAWAN_CERTIFICATION_JOIN_REQ;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_SWITCH_CLASS_REQ:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_SWITCH_CLASS_REQ_SIZE )
        {
            if( rx_buffer[1] <= ( uint8_t ) LORAWAN_CERTIFICATION_CLASS_C )
            {
                smtc_modem_set_class( lorawan_certification->stack_id, ( smtc_modem_class_t ) rx_buffer[1] );
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "certif unknown class\n" );
            }
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_ADR_BIT_CHANGE_REQ:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_ADR_BIT_CHANGE_REQ_SIZE )
        {
            if( rx_buffer[1] == ( uint8_t ) LORAWAN_CERTIFICATION_ADR_OFF )
            {
                uint8_t adr_custom_data[16] = { 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
                                                0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03 };
                smtc_modem_adr_set_profile( lorawan_certification_obj->stack_id, SMTC_MODEM_ADR_PROFILE_CUSTOM,
                                            adr_custom_data );
            }
            else
            {
                lorawan_api_dr_strategy_set( STATIC_ADR_MODE, lorawan_certification_obj->stack_id );
            }
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_REGIONAL_DUTY_CYCLE_CTRL_REQ:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_REGIONAL_DUTY_CYCLE_CTRL_REQ_SIZE )
        {
            if( rx_buffer[1] == ( uint8_t ) LORAWAN_CERTIFICATION_DUTY_CYCLE_OFF )
            {
                smtc_duty_cycle_enable_set( SMTC_DTC_FULL_DISABLED );
            }
            else
            {
                smtc_duty_cycle_enable_set( SMTC_DTC_ENABLED );
            }
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_TX_PERIODICITY_CHANGE_REQ:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_TX_PERIODICITY_CHANGE_REQ_SIZE )
        {
            lorawan_certification->ul_periodicity_s = lorawan_certification_periodicity_table_s[rx_buffer[1]];
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_TX_FRAME_CTRL_REQ:
    {
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
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }

        break;
    }
    case LORAWAN_CERTIFICATION_ECHO_PLAY_REQ:
    {
        if( rx_buffer_length <= LORAWAN_CERTIFICATION_ECHO_PLAY_REQ_SIZE )
        {
            uint8_t max_len =
                MIN( rx_buffer_length,
                     lorawan_api_next_max_payload_length_get( lorawan_certification->stack_id ) - *tx_buffer_length );

            // The first byte is the command ID
            max_len -= 1;

            tx_buffer[( *tx_buffer_length )++] = LORAWAN_CERTIFICATION_ECHO_PLAY_ANS;
            for( uint8_t i = 0; i < max_len; i++ )
            {
                tx_buffer[( *tx_buffer_length )++] = ( rx_buffer[i + 1] + 1 ) % 256;
            }

            ret = LORAWAN_CERTIFICATION_TX_CERTIF_REQ;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_RX_APP_CNT_REQ:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_RX_APP_CNT_REQ_SIZE )
        {
            tx_buffer[( *tx_buffer_length )++] = LORAWAN_CERTIFICATION_RX_APP_CNT_ANS;
            tx_buffer[( *tx_buffer_length )++] = ( lorawan_certification->rx_app_cnt & 0xFF );
            tx_buffer[( *tx_buffer_length )++] = ( lorawan_certification->rx_app_cnt >> 8 ) & 0xFF;

            ret = LORAWAN_CERTIFICATION_TX_CERTIF_REQ;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_RX_APP_CNT_RESET_REQ:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_RX_APP_CNT_RESET_REQ_SIZE )
        {
            lorawan_certification->rx_app_cnt = 0;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_LINK_CHECK_REQ:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_LINK_CHECK_REQ_SIZE )
        {
            if( lorawan_certification->cid_req_list_size < sizeof( lorawan_certification->cid_req_list ) )
            {
                lorawan_certification->cid_req_list[lorawan_certification->cid_req_list_size++] = LINK_CHECK_REQ;
                ret = LORAWAN_CERTIFICATION_TX_MAC_REQ;
            }
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_DEVICE_TIME_REQ:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_DEVICE_TIME_REQ_SIZE )
        {
            if( lorawan_certification->cid_req_list_size < sizeof( lorawan_certification->cid_req_list ) )
            {
                lorawan_certification->cid_req_list[lorawan_certification->cid_req_list_size++] = DEVICE_TIME_REQ;
                ret = LORAWAN_CERTIFICATION_TX_MAC_REQ;
            }
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_PING_SLOT_INFO_REQ:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_PING_SLOT_INFO_REQ_SIZE )
        {
            lorawan_api_set_ping_slot_periodicity( rx_buffer[1], lorawan_certification->stack_id );
            if( lorawan_certification->cid_req_list_size < sizeof( lorawan_certification->cid_req_list ) )
            {
                lorawan_certification->cid_req_list[lorawan_certification->cid_req_list_size++] = PING_SLOT_INFO_REQ;
                ret = LORAWAN_CERTIFICATION_TX_MAC_REQ;
            }
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_BEACON_RX_STATUS_IND_CTRL:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_BEACON_RX_STATUS_IND_CTRL_SIZE )
        {
            lorawan_certification->beacon_rx_status_ind_ctrl = rx_buffer[1];
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_BEACON_CNT_REQ:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_BEACON_CNT_REQ_SIZE )
        {
            tx_buffer[( *tx_buffer_length )++] = LORAWAN_CERTIFICATION_BEACON_CNT_ANS;
            tx_buffer[( *tx_buffer_length )++] = ( lorawan_certification->rx_beacon_cnt & 0xFF );
            tx_buffer[( *tx_buffer_length )++] = ( lorawan_certification->rx_beacon_cnt >> 8 ) & 0xFF;

            ret = LORAWAN_CERTIFICATION_TX_CERTIF_REQ;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_FRAG_SESSION_CNT_REQ:
    {
#if ( ADD_FUOTA == 2 )
        if( rx_buffer_length == LORAWAN_CERTIFICATION_FRAG_SESSION_CNT_REQ_SIZE )
        {
            uint8_t  frag_index  = rx_buffer[1] & 0x03;
            uint16_t session_cnt = 0;

            uint8_t status = lorawan_fragmentation_package_get_frag_session_count( frag_index, &session_cnt );
            tx_buffer[( *tx_buffer_length )++] = LORAWAN_CERTIFICATION_FRAG_SESSION_CNT_ANS;
            tx_buffer[( *tx_buffer_length )++] = status;
            if( ( status & 0x04 ) == 0 )
            {
                tx_buffer[( *tx_buffer_length )++] = session_cnt & 0xFF;
                tx_buffer[( *tx_buffer_length )++] = ( session_cnt >> 8 ) & 0xFF;
            }

            ret = LORAWAN_CERTIFICATION_TX_CERTIF_REQ;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
#else
        SMTC_MODEM_HAL_TRACE_ERROR( "Not implemented\n" );
#endif
        break;
    }

    case LORAWAN_CERTIFICATION_RELAY_MODE_CTRL_REQ:
    {
#if defined( ADD_RELAY_TX )
        if( rx_buffer_length == LORAWAN_CERTIFICATION_RELAY_MODE_CTRL_SIZE )
        {
            if( rx_buffer[1] == ( uint8_t ) LORAWAN_CERTIFICATION_RELAY_TX_OFF )
            {
                smtc_modem_relay_tx_disable( lorawan_certification_obj->stack_id );
            }
            else
            {
                smtc_modem_relay_tx_config_t user_relay_config = { 0 };
                user_relay_config.second_ch_enable             = false;
                user_relay_config.activation                   = SMTC_MODEM_RELAY_TX_ACTIVATION_MODE_ED_CONTROLLED;
                user_relay_config.number_of_miss_wor_ack_to_switch_in_nosync_mode = 1;
                user_relay_config.smart_level                                     = 5;
                user_relay_config.backoff                                         = 4;

                smtc_modem_relay_tx_enable( lorawan_certification_obj->stack_id, &user_relay_config );
            }
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
#else
        SMTC_MODEM_HAL_TRACE_ERROR( "RELAY_TX not implemented\n" );
#endif
        break;
    }
    case LORAWAN_CERTIFICATION_BEACON_CNT_RST_REQ:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_BEACON_CNT_RST_REQ_SIZE )
        {
            lorawan_certification->rx_beacon_cnt = 0;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_TX_CW_REQ:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_TX_CW_REQ_SIZE )
        {
            lorawan_certification->cw_running   = true;
            lorawan_certification->cw_timeout_s = ( rx_buffer[2] << 8 ) + rx_buffer[1];
            lorawan_certification->cw_frequency =
                ( ( rx_buffer[5] << 16 ) + ( rx_buffer[4] << 8 ) + rx_buffer[3] ) * 100;
            lorawan_certification->cw_tx_power = rx_buffer[6];

            // Leave network
            lorawan_api_join_status_clear( lorawan_certification_obj->stack_id );

            // Start Test mode + tx continuous wave
            smtc_modem_test_start( );
            smtc_modem_test_tx_cw( lorawan_certification->cw_frequency, lorawan_certification->cw_tx_power );
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_DUT_FPORT_224_DISABLE_REQ:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_DUT_FPORT_224_DISABLE_REQ_SIZE )
        {
            lorawan_certification->enabled = false;
            lorawan_api_modem_certification_set( false, lorawan_certification->stack_id );
            smtc_modem_hal_reset_mcu( );
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    case LORAWAN_CERTIFICATION_DUT_VERSION_REQ:
    {
        if( rx_buffer_length == LORAWAN_CERTIFICATION_DUT_VERSION_REQ_SIZE )
        {
            // Ignore command if the payload length is greater than the max payload length allowed
            // by the region and datarate
            if( lorawan_api_next_max_payload_length_get( lorawan_certification->stack_id ) >=
                LORAWAN_CERTIFICATION_DUT_VERSION_ANS_SIZE )
            {
                tx_buffer[( *tx_buffer_length )++] = LORAWAN_CERTIFICATION_DUT_VERSION_ANS;

                lr1mac_version_t lorawan_version = lorawan_api_get_spec_version( lorawan_certification->stack_id );
                lr1mac_version_t rp_version =
                    lorawan_api_get_regional_parameters_version( lorawan_certification->stack_id );

                // Firmware Version
                tx_buffer[( *tx_buffer_length )++] = LORA_BASICS_MODEM_FW_VERSION_MAJOR;
                tx_buffer[( *tx_buffer_length )++] = LORA_BASICS_MODEM_FW_VERSION_MINOR;
                tx_buffer[( *tx_buffer_length )++] = LORA_BASICS_MODEM_FW_VERSION_PATCH;
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

                ret = LORAWAN_CERTIFICATION_TX_CERTIF_REQ;
            }
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "bad size\n" );
        }
        break;
    }
    default:
        SMTC_MODEM_HAL_TRACE_ERROR( "%s Illegal state(%u)\n ", __func__, cid );
        SMTC_MODEM_HAL_TRACE_ARRAY( "rx_buffer", rx_buffer, rx_buffer_length );
        break;
    }
    return ret;
}

static void lorawan_certification_build_beacon_rx_status_ind( lorawan_certification_t* lorawan_certification,
                                                              uint8_t* beacon_buffer, uint8_t beacon_buffer_length,
                                                              uint8_t* tx_buffer, uint8_t* tx_buffer_length,
                                                              int8_t rssi, int8_t snr, uint8_t beacon_dr,
                                                              uint32_t beacon_freq )
{
#if defined( ADD_CLASS_B )
    smtc_beacon_statistics_t beacon_statistics;
    lorawan_api_beacon_get_statistics( &beacon_statistics, lorawan_certification->stack_id );

    if( beacon_statistics.last_beacon_lost_consecutively == 0 )
    {
        lorawan_certification->rx_beacon_cnt++;
    }

    uint8_t            beacon_sf;
    lr1mac_bandwidth_t beacon_bw;
    lorawan_api_lora_dr_to_sf_bw( beacon_dr, &beacon_sf, &beacon_bw, lorawan_certification->stack_id );

    uint32_t epoch_time   = smtc_decode_beacon_epoch_time( beacon_buffer, beacon_sf );
    uint8_t  beacon_param = smtc_decode_beacon_param( beacon_buffer, beacon_sf );

    beacon_freq = beacon_freq / lorawan_api_get_frequency_factor( lorawan_certification->stack_id );

    *tx_buffer_length = 0;

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

    smtc_decode_beacon_gw_specific( beacon_buffer, beacon_sf,
                                    &tx_buffer[*tx_buffer_length] );  // GwSpecific 7 bytes
    *tx_buffer_length += 7;
#endif
}

/* --- EOF ------------------------------------------------------------------ */
