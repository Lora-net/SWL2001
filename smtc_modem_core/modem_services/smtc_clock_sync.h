/*!
 * \file      smtc_clock_sync.h
 *
 * \brief     Clock syncronization implementation
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
#ifndef __SMTC_CLOCK_SYNC_H__
#define __SMTC_CLOCK_SYNC_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>    // C99 types
#include <stdbool.h>   // bool type
#include "alc_sync.h"  // alc_sync_ctx_t

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

// #define CLOCK_SYNC_GPS_EPOCH_CONVERT
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
// clang-format off
#define CLOCK_SYNC_DEFAULT_REQUEST_PERIOD_S               ( 129600UL )    // 36 hours
#define CLOCK_SYNC_DEFAULT_S_SINCE_LAST_CORRECTION        ( 5184000UL )    // 3600s*24h*60d

#if defined(ENABLE_FAST_CLOCK_SYNC)
#define CLOCK_SYNC_NB_REQ_PERIOD1                         ( 20 )
#define CLOCK_SYNC_PERIOD1_RETRY                          ( 1 )
#else
#define CLOCK_SYNC_NB_REQ_PERIOD1                         ( 3 )
#define CLOCK_SYNC_PERIOD1_RETRY                          ( 128 )
#endif
#define CLOCK_SYNC_NB_REQ_PERIOD2                         ( 6 )
#define CLOCK_SYNC_PERIOD2_RETRY                          ( 14400 )   // 4 hours
#define CLOCK_SYNC_PERIOD_RETRY                           ( 129600 )  // 36 hours


#if defined( CLOCK_SYNC_GPS_EPOCH_CONVERT )
#define CLOCK_SYNC_LEAP_SECOND                            ( -18 )
#define CLOCK_SYNC_UNIX_GPS_EPOCH_OFFSET                  ( 315964800UL ) // 00:00:00, Sunday 6 th of January 1980 (start of the GPS epoch)
#endif
// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef enum clock_sync_ret_e
{
    CLOCK_SYNC_OK  = 0,
    CLOCK_SYNC_ERR = 1,
} clock_sync_ret_t;

typedef enum clock_sync_status_e
{
    CLOCK_SYNC_NO_SYNC           = 0,
    CLOCK_SYNC_MANUAL_SYNC       = 1,
    CLOCK_SYNC_NETWORK_SYNC_DONE = 2,
} clock_sync_status_t;

/**
 * @brief Clock Synchronization services
 */
typedef enum clock_sync_service_e
{
    CLOCK_SYNC_MAC = 0,
    CLOCK_SYNC_ALC = 1,
} clock_sync_service_t;

typedef struct clock_sync_ctx_s
{
    bool                 enabled;
    uint8_t              alcsync_port;
    uint32_t             periodicity_s;
    uint32_t             timestamp_last_correction_s;
    uint32_t             nb_time_req;
    clock_sync_service_t sync_service_type;

    // From ALC Sync
    int32_t time_correction_s;
    // From Stack
    uint32_t seconds_since_epoch;
    uint32_t fractional_second;

    clock_sync_status_t sync_status;

    alc_sync_ctx_t* alc_ctx;

} clock_sync_ctx_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief clock_sync_init Initialize clock sync variables to default value
 *
 * @param [in] clock_sync_ctx_t *ctx          Clock sync pointer context
 * @param [in] alc_sync_ctx_t   *alc_ctx      ALC sync pointer context
 * @return void
 */
void clock_sync_init( clock_sync_ctx_t* ctx, alc_sync_ctx_t* alc_ctx );

/**
 * @brief
 *
 * @param ctx                               Clock sync pointer context
 * @param enable                            true: Enable the service, else disable
 * @param sync_service                      select the service that need to be activated, Network or Applicative layer
 */
void clock_sync_set_enabled( clock_sync_ctx_t* ctx, bool enable, clock_sync_service_t sync_service );

/**
 * @brief
 *
 * @param ctx                               Clock sync pointer context
 * @return true                             Service is enabled
 * @return false                            Service is disabled
 */
bool clock_sync_is_enabled( clock_sync_ctx_t* ctx );

/**
 * @brief Get Alcsync port
 *
 * @remark Port 202: ALC clock sync is used
 * @remark Port DeviceManagement: ALC clock sync is used is Semtech cloud
 *
 * @param [in] clock_sync_ctx_t *ctx      Clock sync pointer context
 * @return uint8_t clock_sync_port
 */
uint8_t clock_sync_get_alcsync_port( clock_sync_ctx_t* ctx );

/**
 * @brief Get the current configured service
 *
 * @param ctx
 * @return clock_sync_service_t
 */
clock_sync_service_t clock_sync_get_current_service( clock_sync_ctx_t* ctx );

/**
 * @brief Set Alcsync port
 *
 * @remark Port 202: ALC clock sync is used
 * @remark Port DeviceManagement: ALC clock sync is used is Semtech cloud
 *
 * @param [in] clock_sync_ctx_t *ctx      Clock sync pointer context
 * @return clock_sync_ret_t clock_sync_port
 */
clock_sync_ret_t clock_sync_set_alcsync_port( clock_sync_ctx_t* ctx, uint8_t port );

/**
 * @brief
 *
 */
void clock_sync_callback( clock_sync_ctx_t* ctx, uint32_t rx_timestamp_s );

/**
 * @brief
 *
 * @param ctx
 * @return uint32_t
 */

/**
 * @brief Get the gps time and fractional second
 *
 * @param [in] ctx                Clock synchronization context
 * @param [out] gps_time_in_s     The gps time in second
 * @param [out] fractional_second The fractional second
 * @return true                   Time is valid
 * @return false                  Time is not valid
 */
bool clock_sync_get_gps_time_second( clock_sync_ctx_t* ctx, uint32_t* gps_time_in_s, uint32_t* fractional_second );

/**
 * @brief
 *
 * @param ctx
 * @param gps_time_s
 */
void clock_sync_set_gps_time( clock_sync_ctx_t* ctx, uint32_t gps_time_s );

/**
 * @brief
 *
 * @param ctx
 * @return true
 * @return false
 */
bool clock_sync_is_done( clock_sync_ctx_t* ctx );

/**
 * @brief
 *
 * @param ctx
 * @return true
 * @return false
 */
bool clock_sync_is_time_valid( clock_sync_ctx_t* ctx );

/**
 * @brief
 *
 * @param ctx
 */
void clock_sync_set_sync_lost( clock_sync_ctx_t* ctx );

/**
 * @brief
 *
 * @param ctx
 * @return uint32_t
 */
uint32_t clock_sync_get_interval_second( clock_sync_ctx_t* ctx );

/**
 * @brief
 *
 * @param ctx
 * @param interval_s
 * @return clock_sync_ret_t
 */
clock_sync_ret_t clock_sync_set_interval_second( clock_sync_ctx_t* ctx, uint32_t interval_s );

/**
 * @brief
 *
 * @param ctx
 * @param delay_s
 * @return clock_sync_ret_t
 */
clock_sync_ret_t clock_sync_set_invalid_time_delay_s( clock_sync_ctx_t* ctx, uint32_t delay_s );

/**
 * @brief
 *
 * @param ctx
 * @return uint32_t
 */
uint32_t clock_sync_get_invalid_time_delay_s( clock_sync_ctx_t* ctx );

/**
 * @brief
 *
 * @param ctx
 */
void clock_sync_reset_nb_time_req( clock_sync_ctx_t* ctx );

/**
 * @brief Get the time left before concider time is no more valid
 *
 * @param ctx
 * @return uint32_t
 */
uint32_t clock_sync_get_time_left_connection_lost( clock_sync_ctx_t* ctx );

/**
 * @brief
 *
 * @param ctx
 * @return clock_sync_ret_t
 */
clock_sync_ret_t clock_sync_request( clock_sync_ctx_t* ctx );

#if defined( CLOCK_SYNC_GPS_EPOCH_CONVERT )
void clock_sync_convert_gps_epoch_to_unix_epoch( uint32_t gps_epoch );
#endif

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_CLOCK_SYNC_H__

/* --- EOF ------------------------------------------------------------------ */
