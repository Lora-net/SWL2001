/*!
 * \file      smtc_clock_sync.c
 *
 * \brief
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
#include <string.h>

#include "alc_sync.h"
#include "smtc_clock_sync.h"
#include "modem_services_common.h"
#include "modem_context.h"
#include "lr1mac_defs.h"
#include "lorawan_api.h"

#if defined( CLOCK_SYNC_GPS_EPOCH_CONVERT )
#include <time.h>
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACRO -----------------------------------------------------------
 */

/**
 * @brief Math Abs macro
 */
#define ABS( N ) ( ( N < 0 ) ? ( -N ) : ( N ) )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define CLOCK_SYNC_ALC_UPDATED_DELAY_S 3

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
void clock_sync_reset( clock_sync_ctx_t* ctx );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void clock_sync_init( clock_sync_ctx_t* ctx, alc_sync_ctx_t* alc_ctx )
{
    ctx->enabled                     = false;
    ctx->periodicity_s               = CLOCK_SYNC_DEFAULT_REQUEST_PERIOD_S;
    ctx->time_correction_s           = 0;
    ctx->timestamp_last_correction_s = 0;
    ctx->nb_time_req                 = 0;
    ctx->alcsync_port                = get_modem_dm_port( );
    ctx->sync_service_type           = CLOCK_SYNC_MAC;

    ctx->sync_status         = CLOCK_SYNC_NO_SYNC;
    ctx->seconds_since_epoch = 0;
    ctx->fractional_second   = 0;

    ctx->alc_ctx = alc_ctx;
}

void clock_sync_set_enabled( clock_sync_ctx_t* ctx, bool enable, clock_sync_service_t sync_service )
{
    // Reset clock sync context before starting service
    clock_sync_reset( ctx );
    ctx->enabled           = enable;
    ctx->sync_service_type = sync_service;
}

bool clock_sync_is_enabled( clock_sync_ctx_t* ctx )
{
    return ctx->enabled;
}

uint8_t clock_sync_get_alcsync_port( clock_sync_ctx_t* ctx )
{
    return ctx->alcsync_port;
}

clock_sync_service_t clock_sync_get_current_service( clock_sync_ctx_t* ctx )
{
    return ctx->sync_service_type;
}

clock_sync_ret_t clock_sync_set_alcsync_port( clock_sync_ctx_t* ctx, uint8_t port )
{
    if( port == 0 || port >= 224 )
    {
        return CLOCK_SYNC_ERR;
    }
    ctx->alcsync_port = port;
    return CLOCK_SYNC_OK;
}

void clock_sync_callback( clock_sync_ctx_t* ctx, uint32_t rx_timestamp_s )
{
    uint32_t interval_s = 0;

    if( ctx->sync_service_type == CLOCK_SYNC_MAC )
    {
        ctx->timestamp_last_correction_s = lorawan_api_get_timestamp_last_device_time_ans_s( );
        ctx->sync_status                 = CLOCK_SYNC_NETWORK_SYNC_DONE;
    }

#if defined( CLOCK_SYNC_GPS_EPOCH_CONVERT )
    if( clock_sync_is_time_valid( ctx ) == true )
    {
        uint32_t gps_time_s        = 0;
        uint32_t fractional_second = 0;

        clock_sync_get_gps_time_second( ctx, &gps_time_s, &fractional_second );
        clock_sync_convert_gps_epoch_to_unix_epoch( gps_time_s );
    }
#endif

    if( clock_sync_is_done( ctx ) )
    {
        // Synchronized with the network
        if( clock_sync_is_time_valid( ctx ) == false )
        {
            // synchronisation lost
            // Send event
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TIME, SMTC_MODEM_EVENT_TIME_NOT_VALID );

            clock_sync_set_sync_lost( ctx );

            ctx->nb_time_req = 1;

            interval_s = MIN( CLOCK_SYNC_PERIOD1_RETRY, clock_sync_get_interval_second( ctx ) );
        }
        else
        {
            if( ctx->sync_service_type == CLOCK_SYNC_ALC )
            {
                // in case ALCSYNC is the selected service check if no downlink happened
                if( ctx->alc_ctx->is_sync_dl_received == false )
                {
                    increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TIME, SMTC_MODEM_EVENT_TIME_VALID_BUT_NOT_SYNC );
                }
                else
                {
                    increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TIME, SMTC_MODEM_EVENT_TIME_VALID );
                }
            }
            else
            {
                smtc_modem_event_time_status_t time_updated_status = SMTC_MODEM_EVENT_TIME_VALID_BUT_NOT_SYNC;
                if( lorawan_api_get_device_time_req_status( ) == OKLORAWAN )
                {
                    time_updated_status = SMTC_MODEM_EVENT_TIME_VALID;
                }
                increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TIME, time_updated_status );
            }

            if( clock_sync_get_interval_second( ctx ) > 0 )
            {
                interval_s = clock_sync_get_interval_second( ctx );
            }
        }
    }
    else
    {
        if( ctx->nb_time_req < CLOCK_SYNC_NB_REQ_PERIOD1 )
        {
            interval_s = MIN( CLOCK_SYNC_PERIOD1_RETRY, clock_sync_get_interval_second( ctx ) );
        }
        else if( ctx->nb_time_req < ( CLOCK_SYNC_NB_REQ_PERIOD1 + CLOCK_SYNC_NB_REQ_PERIOD2 ) )
        {
            interval_s = MIN( CLOCK_SYNC_PERIOD2_RETRY, clock_sync_get_interval_second( ctx ) );
        }
        else
        {
            // Since we are not yet synchronized we keep the period to max 36h
            interval_s = MIN( CLOCK_SYNC_PERIOD_RETRY, clock_sync_get_interval_second( ctx ) );
        }
    }

    interval_s = MIN( interval_s, clock_sync_get_time_left_connection_lost( ctx ) );

    if( ctx->enabled == true )
    {
        int32_t tmp_rand = 0;
        do
        {
            tmp_rand = smtc_modem_hal_get_signed_random_nb_in_range( -30, 30 );
        } while( ( tmp_rand < 0 ) && ( ABS( tmp_rand ) > interval_s ) );

        modem_supervisor_add_task_clock_sync_time_req( interval_s + tmp_rand );
    }
}

bool clock_sync_get_gps_time_second( clock_sync_ctx_t* ctx, uint32_t* gps_time_in_s, uint32_t* fractional_second )
{
    bool ret = false;

    if( ctx->sync_status == CLOCK_SYNC_MANUAL_SYNC )
    {
        *gps_time_in_s = ctx->seconds_since_epoch +
                         ( smtc_modem_hal_get_time_in_s( ) - ctx->timestamp_last_correction_s ) +
                         smtc_modem_hal_get_time_compensation_in_s( );
        // No fractional second feature is available in case of manual sync
        *fractional_second = 0;
    }
    else
    {
        if( ctx->sync_service_type == CLOCK_SYNC_MAC )
        {
            ret                = lorawan_api_convert_rtc_to_gps_epoch_time( smtc_modem_hal_get_time_in_ms( ),
                                                             &ctx->seconds_since_epoch, &ctx->fractional_second );
            *gps_time_in_s     = ctx->seconds_since_epoch;  // Todo manage wrapping
            *fractional_second = ctx->fractional_second;
        }
        else
        {
            ret            = clock_sync_is_time_valid( ctx );
            *gps_time_in_s = alc_sync_get_gps_time_second( ctx->alc_ctx );
            // No fractional second feature is available in alcsync service
            *fractional_second = 0;
        }
    }

    return ret;
}

void clock_sync_set_gps_time( clock_sync_ctx_t* ctx, uint32_t gps_time_s )
{
    if( ctx->sync_service_type == CLOCK_SYNC_MAC )
    {
        ctx->seconds_since_epoch         = gps_time_s;
        ctx->fractional_second           = 0;
        ctx->timestamp_last_correction_s = smtc_modem_hal_get_time_in_s( );
        ctx->sync_status                 = CLOCK_SYNC_MANUAL_SYNC;
    }
    else
    {
        alc_sync_set_time_correction_second( ctx->alc_ctx,
                                             ( int32_t )( gps_time_s - smtc_modem_hal_get_time_in_s( ) ) );
    }
}

bool clock_sync_is_done( clock_sync_ctx_t* ctx )
{
    bool b_ret = false;
    if( ctx->sync_service_type == CLOCK_SYNC_MAC )
    {
        if( ( ctx->sync_status == CLOCK_SYNC_MANUAL_SYNC ) || ( ctx->sync_status == CLOCK_SYNC_NETWORK_SYNC_DONE ) )
        {
            b_ret = true;
        }
    }
    else
    {
        b_ret = is_alc_sync_done( ctx->alc_ctx );
    }
    return b_ret;
}

bool clock_sync_is_time_valid( clock_sync_ctx_t* ctx )
{
    bool b_ret = true;

    if( ctx->sync_service_type == CLOCK_SYNC_MAC )
    {
        if( ctx->sync_status == CLOCK_SYNC_MANUAL_SYNC )
        {
            uint32_t rtc_s = smtc_modem_hal_get_time_in_s( );

            if( ( ctx->timestamp_last_correction_s != 0 ) &&
                ( ( int32_t )( rtc_s - ctx->timestamp_last_correction_s -
                               lorawan_api_get_device_time_invalid_delay_s( ) ) < 0 ) )
            {
                b_ret = true;
            }
            else
            {
                b_ret = false;
            }
        }
        else
        {
            b_ret = lorawan_api_is_time_valid( );
        }
    }
    else
    {
        b_ret = is_alc_sync_time_valid( ctx->alc_ctx );
    }

    return b_ret;
}

void clock_sync_set_sync_lost( clock_sync_ctx_t* ctx )
{
    if( ctx->sync_service_type == CLOCK_SYNC_MAC )
    {
        ctx->sync_status                 = CLOCK_SYNC_NO_SYNC;
        ctx->timestamp_last_correction_s = 0;
    }
    else
    {
        alc_sync_set_sync_lost( ctx->alc_ctx );
    }
}

uint32_t clock_sync_get_interval_second( clock_sync_ctx_t* ctx )
{
    uint32_t ret;
    if( ctx->sync_service_type == CLOCK_SYNC_MAC )
    {
        ret = ctx->periodicity_s;
    }
    else
    {
        ret = alc_sync_get_interval_second( ctx->alc_ctx );
    }
    return ret;
}

clock_sync_ret_t clock_sync_set_interval_second( clock_sync_ctx_t* ctx, uint32_t interval_s )
{
    // The interval is set in MAC_SYNC and ALC_SYNC to allow to switch of service without reconfigure all parameters

    clock_sync_ret_t ret = CLOCK_SYNC_OK;
    // if( ctx->sync_service_type == CLOCK_SYNC_MAC )
    // {
    if( interval_s >= lorawan_api_get_device_time_invalid_delay_s( ) )
    {
        ret = CLOCK_SYNC_ERR;
    }
    else
    {
        ctx->periodicity_s = interval_s;
    }
    // }
    // else
    // {
    if( alc_sync_set_interval_second( ctx->alc_ctx, interval_s ) != true )
    {
        ret = CLOCK_SYNC_ERR;
    }
    // }
    return ret;
}

clock_sync_ret_t clock_sync_set_invalid_time_delay_s( clock_sync_ctx_t* ctx, uint32_t delay_s )
{
    clock_sync_ret_t ret = CLOCK_SYNC_OK;
    if( delay_s > ALC_SYNC_DEFAULT_S_SINCE_LAST_CORRECTION )
    {
        ret = CLOCK_SYNC_ERR;
    }
    else if( delay_s <= clock_sync_get_interval_second( ctx ) )
    {
        ret = CLOCK_SYNC_ERR;
    }

    if( ret == CLOCK_SYNC_OK )
    {
        // The delay_s is set in MAC_SYNC and ALC_SYNC to allow to switch of service without reconfigure all parameters

        // if( ctx->sync_service_type == CLOCK_SYNC_MAC )
        // {
        if( lorawan_api_set_device_time_invalid_delay_s( delay_s ) != OKLORAWAN )
        {
            ret = CLOCK_SYNC_ERR;
        }
        // }
        // else
        // {
        if( alc_sync_set_valid_delay_second( ctx->alc_ctx, delay_s ) == false )
        {
            ret = CLOCK_SYNC_ERR;
        }
        // }
    }

    return ret;
}

uint32_t clock_sync_get_invalid_time_delay_s( clock_sync_ctx_t* ctx )
{
    uint32_t delay_s;
    if( ctx->sync_service_type == CLOCK_SYNC_MAC )
    {
        delay_s = lorawan_api_get_device_time_invalid_delay_s( );
    }
    else
    {
        delay_s = alc_sync_get_valid_delay_second( ctx->alc_ctx );
    }
    return delay_s;
}

void clock_sync_reset_nb_time_req( clock_sync_ctx_t* ctx )
{
    ctx->nb_time_req = 0;
}

uint32_t clock_sync_get_time_left_connection_lost( clock_sync_ctx_t* ctx )
{
    uint32_t ret = 0;
    if( ctx->sync_service_type == CLOCK_SYNC_MAC )
    {
        ret = lorawan_api_get_time_left_connection_lost( );
    }
    else
    {
        ret = alc_sync_get_time_left_connection_lost( ctx->alc_ctx );
    }
    return ret;
}

clock_sync_ret_t clock_sync_request( clock_sync_ctx_t* ctx )
{
    clock_sync_ret_t ret         = CLOCK_SYNC_ERR;
    status_lorawan_t send_status = ERRORLORAWAN;

    if( ctx->sync_service_type == CLOCK_SYNC_MAC )
    {
        send_status = lorawan_api_send_stack_cid_req( DEVICE_TIME_REQ );
    }
    else
    {
        uint8_t max_payload = lorawan_api_next_max_payload_length_get( );
        uint8_t tx_buffer_out[ALC_SYNC_TX_PAYLOAD_SIZE_MAX + 1];  // +1 is the DM_INFO_ALCSYNC ID
        uint8_t tx_buffer_length_out = 0;

        // The randomness value is used because the frame must be sent at time
        uint32_t target_send_time = smtc_modem_hal_get_time_in_s( ) + smtc_modem_hal_get_random_nb_in_range( 1, 3 );
        uint8_t  app_time_ans_required = false;
        uint8_t  tx_buff_offset        = 0;
        // AnsRequired bit set to one in both cases: synchronisation lost or the last 30 days
        // synchronisation lost
        if( ( !clock_sync_is_done( ctx ) ) ||
            ( clock_sync_is_done( ctx ) &&
              ( ( smtc_modem_hal_get_time_in_s( ) - alc_sync_get_timestamp_last_correction_s( ctx->alc_ctx ) ) >
                ( ALC_SYNC_DEFAULT_S_SINCE_LAST_CORRECTION >> 1 ) ) ) )
        {
            app_time_ans_required = true;
        }
        // check first if alc_sync runs on dm port and if yes add dm code
        if( get_modem_dm_port( ) == clock_sync_get_alcsync_port( ctx ) )
        {
            tx_buffer_out[tx_buff_offset] = DM_INFO_ALCSYNC;
            tx_buff_offset++;
        }
        // use target send time with both local compensation and previous alcsync compensation to create payload
        alc_sync_create_uplink_payload( ctx->alc_ctx,
                                        target_send_time + smtc_modem_hal_get_time_compensation_in_s( ) +
                                            alc_sync_get_time_correction_second( ctx->alc_ctx ),
                                        app_time_ans_required, false, max_payload, &tx_buffer_out[tx_buff_offset],
                                        &tx_buffer_length_out );
        // compute final size according to previous offset
        tx_buffer_length_out += tx_buff_offset;
        if( tx_buffer_length_out > 0 )
        {
            send_status =
                lorawan_api_payload_send_at_time( clock_sync_get_alcsync_port( ctx ), true, tx_buffer_out,
                                                  tx_buffer_length_out, UNCONF_DATA_UP, target_send_time * 1000 );

            // reset alcsync reception bool
            ctx->alc_ctx->is_sync_dl_received = false;
        }
    }

    if( send_status == OKLORAWAN )
    {
        ret = CLOCK_SYNC_OK;
        ctx->nb_time_req++;
    }

    return ret;
}

#if defined( CLOCK_SYNC_GPS_EPOCH_CONVERT )
void clock_sync_convert_gps_epoch_to_unix_epoch( uint32_t gps_epoch )
{
    struct tm ts;
    char      buf[30];
    time_t    rawtime = gps_epoch + CLOCK_SYNC_UNIX_GPS_EPOCH_OFFSET + CLOCK_SYNC_LEAP_SECOND;

    // Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
    ts = *localtime( &rawtime );
    strftime( buf, sizeof( buf ), "%a %Y-%m-%d %H:%M:%S %Z", &ts );
    LOG_INFO( "%s\n", buf );
}
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void clock_sync_reset( clock_sync_ctx_t* ctx )
{
    ctx->time_correction_s           = 0;
    ctx->timestamp_last_correction_s = 0;
    ctx->nb_time_req                 = 0;

    ctx->sync_status         = CLOCK_SYNC_NO_SYNC;
    ctx->seconds_since_epoch = 0;
    ctx->fractional_second   = 0;
}

/* --- EOF ------------------------------------------------------------------ */
