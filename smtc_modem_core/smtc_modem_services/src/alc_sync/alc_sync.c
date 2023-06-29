/*!
 * \file      alc_sync.c
 *
 * \brief     LoRaWAN Application Layer Clock Synchronization Specification
 * v1.0.0
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "alc_sync.h"

#include <stdbool.h>  // bool type
#include <stdint.h>   // C99 types
#include <string.h>   //memcpy

#include "modem_services_common.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
static const char alc_sync_bad_size_str[] = "ALC Sync payload bad size";
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */
#define APP_TIME_ANS_TIME_CORRECTION_BYTE 0
#define APP_TIME_ANS_TOKEN_BYTE 4

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Construct the package version answer
 *
 * @param [in] alc_sync_ctx_t *ctx          clock sync pointer context
 * @return None
 */
static void alc_sync_construct_package_version_answer( alc_sync_ctx_t* ctx );

/**
 * @brief Construct the applicative time request
 *
 * @param [in] alc_sync_ctx_t *ctx  Clock sync pointer context
 * @param [in] device_time          DeviceTime is the current end-device clock and is expressed as the time in seconds
 *                                  since 00:00:00, Sunday 6 th of January 1980 (start of the GPS epoch) modulo 2^32
 * @param [in] ans_required         If the AnsRequired bit is set to 1 the end-device expects an answer whether its
 *                                  clock is well synchronized or not. If this bit is set to 0, this signals to the AS
 *                                  that it only needs to answer if the end-device clock is de-synchronized.
 * @param [in] force_resync_status  If True and NbTransmissions > 0: AppTimeReq will be generate
 *                                  Else: AppTimeReq will be generate what ever the NbTransmissions value
 *
 * @return None
 */
static void alc_sync_construct_app_time_request( alc_sync_ctx_t* ctx, uint32_t device_time, uint8_t ans_required,
                                                 uint8_t force_resync_status );

/**
 * @brief Construct the applicative time periodicity answer
 *
 * @param [in] alc_sync_ctx_t *ctx  Clock sync pointer context
 * @param status                    NotSupported bit is set to 1 if the end-device’s application does not accept a
 *                                  periodicity set by the application server and manages the clock synchronization
 *                                  process and periodicity itself
 * @param time                      Is the current end-device’s clock time captured immediately before the transmission
 *                                  of the radio message
 * @return None
 */
static void alc_sync_construct_app_time_periodicity_answer( alc_sync_ctx_t* ctx, uint8_t status, uint32_t time );

/**
 * @brief Get ALC Sync Tx buffer
 *
 * @param [in] alc_sync_ctx_t *ctx      Clock sync pointer context
 * @param [out] tx_buffer_out*          Contains the output payload for the Application Server
 * @param [out] tx_buffer_length_out*   Contains the output payload length
 * @return void
 */
static void alc_sync_get_tx_buffer( alc_sync_ctx_t* ctx, uint8_t* tx_buffer_out, uint8_t* tx_buffer_length_out );

/**
 * @brief Check if the Tx ALC sync buffer is full
 * @param cmd_size Check is the command size can be added to the current Tx
 * buffer
 * @return bool
 */
static inline bool is_alc_sync_tx_buffer_not_full( alc_sync_ctx_t* ctx, uint8_t cmd_size );

/**
 * @brief Decode the applicative time answer
 * @param buffer* Contains the app time answer
 * @return None
 */
static void alc_sync_decode_app_time_ans( alc_sync_ctx_t* ctx, uint8_t* buffer );

/**
 * @brief Decode the app time periodicity
 * @param buffer* Contains the app time periodicity
 * @return None
 */
static void alc_sync_decode_device_app_time_periodicity_req( alc_sync_ctx_t* ctx, uint8_t* buffer );

/**
 * @brief Decode the force device resync request
 * @param buffer* Contains the resync parameters
 * @return None
 */
static void alc_sync_decode_force_device_resync_req( alc_sync_ctx_t* ctx, uint8_t* buffer );

/**
 * @brief Update ALC Sync last rtc timestamp with the current RTC
 * @return void
 */
static void alc_sync_update_timestamp_last_correction_s( alc_sync_ctx_t* ctx );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void alc_sync_init( alc_sync_ctx_t* ctx )
{
    ctx->req_status                        = 0;
    ctx->tx_payload_index                  = 0;
    ctx->nb_transmission                   = 0;
    ctx->token_req                         = 0;
    ctx->periodicity_s                     = ALC_SYNC_DEFAULT_REQUEST_PERIOD_S;
    ctx->time_correction_s                 = 0;
    ctx->timestamp_last_correction_s       = 0;
    ctx->max_length_up_payload             = ALC_SYNC_TX_PAYLOAD_SIZE_MAX;
    ctx->port                              = ALC_SYNC_DEFAULT_PORT;
    ctx->sync_status                       = ALC_SYNC_NO_SYNC;
    ctx->delay_before_time_no_more_valid_s = ALC_SYNC_DEFAULT_S_SINCE_LAST_CORRECTION;
    ctx->is_sync_dl_received               = false;
}

uint8_t alc_sync_parser( alc_sync_ctx_t* ctx, uint8_t* alc_sync_rx_buffer, uint8_t alc_sync_rx_buffer_length )
{
    uint8_t alc_sync_rx_buffer_index = 0;
    ctx->tx_payload_index            = 0;
    ctx->req_status                  = 0;

    while( alc_sync_rx_buffer_length > alc_sync_rx_buffer_index )
    {
        switch( alc_sync_rx_buffer[alc_sync_rx_buffer_index] )
        {
        case ALC_SYNC_PACKAGE_VERSION_REQ:
            if( ( alc_sync_rx_buffer_index + ALC_SYNC_PACKAGE_VERSION_REQ_SIZE ) <= alc_sync_rx_buffer_length )
            {
                ctx->req_status |= ( 1 << ALC_SYNC_PACKAGE_VERSION_REQ );
            }
            else
            {
                LOG_ERROR( "%s\n", alc_sync_bad_size_str );
            }
            alc_sync_rx_buffer_index += ALC_SYNC_PACKAGE_VERSION_REQ_SIZE;
            break;

        case ALC_SYNC_APP_TIME_ANS:
            if( ( alc_sync_rx_buffer_index + ALC_SYNC_APP_TIME_ANS_SIZE ) <= alc_sync_rx_buffer_length )
            {
                alc_sync_decode_app_time_ans( ctx, &alc_sync_rx_buffer[alc_sync_rx_buffer_index + 1] );
                ctx->req_status |= ( 1 << ALC_SYNC_APP_TIME_ANS );
            }
            else
            {
                LOG_ERROR( "%s\n", alc_sync_bad_size_str );
            }
            alc_sync_rx_buffer_index += ALC_SYNC_APP_TIME_ANS_SIZE;
            break;

        case ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ:
            if( ( alc_sync_rx_buffer_index + ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ_SIZE ) <=
                alc_sync_rx_buffer_length )
            {
                alc_sync_decode_device_app_time_periodicity_req( ctx,
                                                                 &alc_sync_rx_buffer[alc_sync_rx_buffer_index + 1] );
                ctx->req_status |= ( 1 << ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ );
            }
            else
            {
                LOG_ERROR( "%s\n", alc_sync_bad_size_str );
            }
            alc_sync_rx_buffer_index += ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ_SIZE;
            break;

        case ALC_SYNC_FORCE_DEVICE_RESYNC_REQ:
            if( ( alc_sync_rx_buffer_index + ALC_SYNC_FORCE_DEVICE_RESYNC_REQ_SIZE ) <= alc_sync_rx_buffer_length )
            {
                alc_sync_decode_force_device_resync_req( ctx, &alc_sync_rx_buffer[alc_sync_rx_buffer_index + 1] );

                // If the NbTransmissions field is 0, the command SHALL be
                // silently discarded
                if( ctx->nb_transmission > 0 )
                {
                    ctx->req_status |= ( 1 << ALC_SYNC_FORCE_DEVICE_RESYNC_REQ );
                }
            }
            else
            {
                LOG_ERROR( "%s\n", alc_sync_bad_size_str );
            }
            alc_sync_rx_buffer_index += ALC_SYNC_FORCE_DEVICE_RESYNC_REQ_SIZE;
            break;

        default:
            LOG_ERROR( "%s Illegal state\n ", __func__ );
            alc_sync_rx_buffer_length = 0;
            break;
        }
    }
    return ctx->req_status;
}

uint32_t alc_sync_get_interval_second( alc_sync_ctx_t* ctx )
{
    return ctx->periodicity_s;
}

bool alc_sync_set_interval_second( alc_sync_ctx_t* ctx, uint32_t interval_s )
{
    if( interval_s >= ctx->delay_before_time_no_more_valid_s )
    {
        return false;
    }
    ctx->periodicity_s = interval_s;
    return true;
}

int32_t alc_sync_get_time_correction_second( alc_sync_ctx_t* ctx )
{
    // LOG_INFO( "GET TIME correction %d\n", ctx->time_correction_s );
    return ctx->time_correction_s;
}

void alc_sync_set_time_correction_second( alc_sync_ctx_t* ctx, int32_t time_correction_s )
{
    // LOG_INFO( "SET TIME correction %d\n", time_correction_s );
    ctx->time_correction_s = time_correction_s;
    alc_sync_update_timestamp_last_correction_s( ctx );
    ctx->sync_status = ALC_SYNC_MANUAL_SYNC;
}

uint32_t alc_sync_get_gps_time_second( alc_sync_ctx_t* ctx )
{
    uint32_t gps_time_s = smtc_modem_services_get_time_s( ) + ctx->time_correction_s;
    // LOG_INFO( "GET GPS time %d\n", gps_time_s );
    return gps_time_s;
}

uint32_t alc_sync_get_time_left_connection_lost( alc_sync_ctx_t* ctx )
{
    uint32_t rtc_s                        = smtc_modem_services_get_time_s( );
    uint32_t time_since_last_correction_s = 0, time_left_connection_lost = ctx->delay_before_time_no_more_valid_s;

    if( ctx->timestamp_last_correction_s > 0 )
    {
        time_since_last_correction_s = rtc_s - ctx->timestamp_last_correction_s;
        if( time_since_last_correction_s <= ctx->delay_before_time_no_more_valid_s )
        {
            time_left_connection_lost = ctx->delay_before_time_no_more_valid_s - time_since_last_correction_s;
            // manage the wrapping case shouldn't occur if it occur launch a sync in 1h
            if( time_left_connection_lost > ctx->delay_before_time_no_more_valid_s )
            {
                time_left_connection_lost = 3600;
            }
        }
    }

    return ( time_left_connection_lost );
}

bool is_alc_sync_done( alc_sync_ctx_t* ctx )
{
    bool b_ret = false;
    if( ( ctx->sync_status == ALC_SYNC_MANUAL_SYNC ) || ( ctx->sync_status == ALC_SYNC_NETWORK_SYNC_DONE ) )
    {
        b_ret = true;
    }
    return b_ret;
}

bool is_alc_sync_time_valid( alc_sync_ctx_t* ctx )
{
    bool     b_ret = true;
    uint32_t rtc_s = smtc_modem_services_get_time_s( );

    if( ( ctx->timestamp_last_correction_s == 0 ) ||
        ( ( rtc_s - ctx->timestamp_last_correction_s ) > ctx->delay_before_time_no_more_valid_s ) )
    {
        b_ret = false;
    }
    return b_ret;
}

bool alc_sync_set_valid_delay_second( alc_sync_ctx_t* ctx, uint32_t delay_s )
{
    if( delay_s > ALC_SYNC_DEFAULT_S_SINCE_LAST_CORRECTION )
    {
        return false;
    }
    else
    {
        ctx->delay_before_time_no_more_valid_s = delay_s;
    }
    return true;
}

uint32_t alc_sync_get_valid_delay_second( alc_sync_ctx_t* ctx )
{
    return ctx->delay_before_time_no_more_valid_s;
}

void alc_sync_set_sync_lost( alc_sync_ctx_t* ctx )
{
    ctx->sync_status                 = ALC_SYNC_NO_SYNC;
    ctx->timestamp_last_correction_s = 0;
}

uint8_t alc_sync_get_nb_transmission( alc_sync_ctx_t* ctx )
{
    return ctx->nb_transmission;
}

uint8_t alc_sync_get_token_req( alc_sync_ctx_t* ctx )
{
    return ctx->token_req;
}

void alc_sync_set_max_length_up_payload( alc_sync_ctx_t* ctx, uint8_t max_payload )
{
    ctx->max_length_up_payload = max_payload;
}

uint32_t alc_sync_get_timestamp_last_correction_s( alc_sync_ctx_t* ctx )
{
    return ctx->timestamp_last_correction_s;
}

uint32_t alc_sync_get_timestamp_ans_requested_s( alc_sync_ctx_t* ctx )
{
    if( 3 * ctx->periodicity_s <= ctx->delay_before_time_no_more_valid_s )
    {
        // The answer requested period is based on 3 repetition of the clock
        // sync request
        return ( ctx->delay_before_time_no_more_valid_s - 3 * ctx->periodicity_s + 30 );
    }
    else
    {
        return ( ctx->periodicity_s + 30 );
    }
}

void alc_sync_create_uplink_payload( alc_sync_ctx_t* alc_ctx, uint32_t alc_sync_time, uint8_t app_time_ans_required,
                                     uint8_t force_resync_status, uint8_t max_payload_length, uint8_t* payload,
                                     uint8_t* payload_len )
{
    alc_sync_set_max_length_up_payload( alc_ctx, max_payload_length );
    alc_sync_construct_package_version_answer( alc_ctx );
    alc_sync_construct_app_time_request( alc_ctx, alc_sync_time, app_time_ans_required, force_resync_status );
    alc_sync_construct_app_time_periodicity_answer( alc_ctx, false, alc_sync_time );
    alc_sync_get_tx_buffer( alc_ctx, payload, payload_len );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void alc_sync_construct_package_version_answer( alc_sync_ctx_t* ctx )
{
    if( ( ( ctx->req_status >> ALC_SYNC_PACKAGE_VERSION_REQ ) & 0x01 ) == 0 )
    {
        return;
    }

    if( is_alc_sync_tx_buffer_not_full( ctx, ALC_SYNC_PACKAGE_VERSION_ANS_SIZE ) == false )
    {
        LOG_ERROR( "ctx->tx_payload buffer is full\n" );
        return;
    }

    ctx->req_status &= ~( 1 << ALC_SYNC_PACKAGE_VERSION_REQ );

    ctx->tx_payload[ctx->tx_payload_index++] = ALC_SYNC_PACKAGE_VERSION_ANS;
    ctx->tx_payload[ctx->tx_payload_index++] = ALC_PACKAGE_IDENTIFIER;
    ctx->tx_payload[ctx->tx_payload_index++] = ALC_PACKAGE_VERSION;
}

static void alc_sync_construct_app_time_request( alc_sync_ctx_t* ctx, uint32_t device_time, uint8_t ans_required,
                                                 uint8_t force_resync_status )
{
    if( force_resync_status == true )
    {
        // When Force Resync, if the NbTransmissions field is 0, the command
        // SHALL be silently discarded
        if( ctx->nb_transmission == 0 )
        {
            return;
        }
        else
        {
            ans_required = 0;
        }
    }

    if( is_alc_sync_tx_buffer_not_full( ctx, ALC_SYNC_APP_TIME_REQ_SIZE ) == false )
    {
        LOG_ERROR( "ctx->tx_payload buffer is full\n" );
        return;
    }

    if( ctx->nb_transmission > 0 )
    {
        ctx->nb_transmission--;
    }

    ctx->tx_payload[ctx->tx_payload_index++] = ALC_SYNC_APP_TIME_REQ;
    ctx->tx_payload[ctx->tx_payload_index++] = device_time & 0xFF;
    ctx->tx_payload[ctx->tx_payload_index++] = ( device_time >> 8 ) & 0xFF;
    ctx->tx_payload[ctx->tx_payload_index++] = ( device_time >> 16 ) & 0xFF;
    ctx->tx_payload[ctx->tx_payload_index++] = ( device_time >> 24 ) & 0xFF;
    ctx->tx_payload[ctx->tx_payload_index++] = ( ( ans_required & 0x01 ) << 4 ) + ( ctx->token_req & 0x0F );
}

static void alc_sync_construct_app_time_periodicity_answer( alc_sync_ctx_t* ctx, uint8_t status, uint32_t time )
{
    if( ( ( ctx->req_status >> ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ ) & 0x01 ) == 0 )
    {
        return;
    }

    if( is_alc_sync_tx_buffer_not_full( ctx, ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_ANS_SIZE ) == false )
    {
        LOG_ERROR( "ctx->tx_payload buffer is full\n" );
        return;
    }

    ctx->req_status &= ~( 1 << ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ );

    ctx->tx_payload[ctx->tx_payload_index++] = ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_ANS;
    ctx->tx_payload[ctx->tx_payload_index++] = status & 0x01;
    ctx->tx_payload[ctx->tx_payload_index++] = time & 0xFF;
    ctx->tx_payload[ctx->tx_payload_index++] = ( time >> 8 ) & 0xFF;
    ctx->tx_payload[ctx->tx_payload_index++] = ( time >> 16 ) & 0xFF;
    ctx->tx_payload[ctx->tx_payload_index++] = ( time >> 24 ) & 0xFF;
}

static void alc_sync_get_tx_buffer( alc_sync_ctx_t* ctx, uint8_t* tx_buffer_out, uint8_t* tx_buffer_length_out )
{
    memcpy( tx_buffer_out, ctx->tx_payload, ctx->tx_payload_index );
    *tx_buffer_length_out      = ctx->tx_payload_index;
    ctx->tx_payload_index      = 0;                             // Reset tx index after data reached
    ctx->max_length_up_payload = ALC_SYNC_TX_PAYLOAD_SIZE_MAX;  // Reinit the max length authorized
}

static void alc_sync_update_timestamp_last_correction_s( alc_sync_ctx_t* ctx )
{
    ctx->timestamp_last_correction_s = smtc_modem_services_get_time_s( );
}

static inline bool is_alc_sync_tx_buffer_not_full( alc_sync_ctx_t* ctx, uint8_t cmd_size )
{
    return ( ( ctx->tx_payload_index + cmd_size ) <= MIN( ALC_SYNC_TX_PAYLOAD_SIZE_MAX, ctx->max_length_up_payload )
                 ? true
                 : false );
}

void alc_sync_decode_app_time_ans( alc_sync_ctx_t* ctx, uint8_t* buffer )
{
    if( ctx->token_req == ( buffer[APP_TIME_ANS_TOKEN_BYTE] & 0x0F ) )
    {
        ctx->token_req = ( ctx->token_req + 1 ) & 0x0F;
        ctx->time_correction_s += ( buffer[APP_TIME_ANS_TIME_CORRECTION_BYTE] ) +
                                  ( buffer[APP_TIME_ANS_TIME_CORRECTION_BYTE + 1] << 8 ) +
                                  ( buffer[APP_TIME_ANS_TIME_CORRECTION_BYTE + 2] << 16 ) +
                                  ( buffer[APP_TIME_ANS_TIME_CORRECTION_BYTE + 3] << 24 );

        // The end - device stops re-transmissions of the AppTimeReq if a valid
        // AppTimeAns is received
        ctx->nb_transmission = 0;

        // Sync done
        ctx->sync_status = ALC_SYNC_NETWORK_SYNC_DONE;

        // Timestamp the last received data
        alc_sync_update_timestamp_last_correction_s( ctx );

        LOG_INFO( "ALC Sync time correction %d s -> new GPS Time: %d s\n", ctx->time_correction_s,
                  ctx->timestamp_last_correction_s + ctx->time_correction_s );
    }
    else
    {
        LOG_WARN( "ALC Sync token mismatch: %d - %d\n", ctx->token_req, ( buffer[4] & 0x0F ) );
    }
}

void alc_sync_decode_device_app_time_periodicity_req( alc_sync_ctx_t* ctx, uint8_t* buffer )
{
    // The actual periodicity in seconds is 128*2^Period
    ctx->periodicity_s = 128 << ( buffer[0] & 0x0F );
    // To avoid a periodicity greater than the invalid delay previously set by the user
    alc_sync_set_valid_delay_second( ctx, MIN( MAX( ctx->periodicity_s * 3, ctx->delay_before_time_no_more_valid_s ),
                                               ALC_SYNC_DEFAULT_S_SINCE_LAST_CORRECTION ) );
}

void alc_sync_decode_force_device_resync_req( alc_sync_ctx_t* ctx, uint8_t* buffer )
{
    ctx->nb_transmission = ( buffer[0] & 0x07 );
}

/* --- EOF ------------------------------------------------------------------ */
