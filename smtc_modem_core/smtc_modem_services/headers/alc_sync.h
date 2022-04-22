/*!
 * \file      alc_sync.h
 *
 * \brief     LoRaWAN Application Layer Clock Synchronization API
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
#ifndef __SMTC_ALC_SYNC_H__
#define __SMTC_ALC_SYNC_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
// clang-format off


#define ALC_SYNC_DEFAULT_PORT                           ( 202 )

#define ALC_PACKAGE_IDENTIFIER                          ( 1 )   // For the "clock synchronization package" this identifier is 1
#define ALC_PACKAGE_VERSION                             ( 1 )

#define ALC_SYNC_PACKAGE_VERSION_REQ_SIZE               ( 1 )
#define ALC_SYNC_PACKAGE_VERSION_ANS_SIZE               ( 3 )
#define ALC_SYNC_APP_TIME_REQ_SIZE                      ( 6 )
#define ALC_SYNC_APP_TIME_ANS_SIZE                      ( 6 )
#define ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ_SIZE   ( 2 )
#define ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_ANS_SIZE   ( 6 )
#define ALC_SYNC_FORCE_DEVICE_RESYNC_REQ_SIZE           ( 2 )

#define ALC_SYNC_TX_PAYLOAD_SIZE_MAX                    ( ALC_SYNC_PACKAGE_VERSION_ANS_SIZE             \
                                                        + ALC_SYNC_APP_TIME_REQ_SIZE                    \
                                                        + ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_ANS_SIZE )

#define ALC_SYNC_DEFAULT_REQUEST_PERIOD_S               ( 129600UL )   // 36 hours
#define ALC_SYNC_DEFAULT_S_SINCE_LAST_CORRECTION        ( 4233600UL )    // 49 days (49d×24h×3600s)

// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef struct alc_sync_ctx_s
{
    bool     enabled;
    uint8_t  req_status;
    uint8_t  tx_payload_index;
    uint8_t  tx_payload[ALC_SYNC_TX_PAYLOAD_SIZE_MAX];
    uint8_t  nb_transmission;
    uint8_t  token_req;
    uint8_t  max_length_up_payload;
    uint8_t  port;
    uint32_t periodicity_s;
    int32_t  time_correction_s;
    uint32_t timestamp_last_correction_s;
    uint32_t delay_before_time_no_more_valid_s;
    uint8_t  sync_status;
    bool     is_sync_dl_received;
} alc_sync_ctx_t;

/**
 * @brief ALC Sync Command ID Request
 *
 * @enum alc_sync_cid_req_t
 */
typedef enum alc_sync_cid_req_e
{
    ALC_SYNC_PACKAGE_VERSION_REQ             = 0,
    ALC_SYNC_APP_TIME_REQ                    = 1,
    ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ = 2,
    ALC_SYNC_FORCE_DEVICE_RESYNC_REQ         = 3,
    ALC_SYNC_NB_CMD_REQ
} alc_sync_cid_req_t;

/**
 * @brief ALC Sync Command ID Answer
 *
 * @enum alc_sync_cid_ans_t
 */
typedef enum alc_sync_cid_ans_e
{
    ALC_SYNC_PACKAGE_VERSION_ANS             = 0,
    ALC_SYNC_APP_TIME_ANS                    = 1,
    ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_ANS = 2,
    ALC_SYNC_NB_CMD_ANS
} alc_sync_cid_ans_t;

typedef enum alc_sync_status_e
{
    ALC_SYNC_NO_SYNC           = 0,
    ALC_SYNC_MANUAL_SYNC       = 1,
    ALC_SYNC_NETWORK_SYNC_DONE = 2,
} alc_sync_status_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief alc_sync_init Initialize ALC sync variables to default value
 *
 * @param [in] alc_sync_ctx_t *ctx          Clock sync pointer context
 * @return void
 */
void alc_sync_init( alc_sync_ctx_t* ctx );

/**
 * @brief alc_sync_parser parse the command received from the application server
 *
 * @param [in] alc_sync_ctx_t *ctx          Clock sync pointer context
 * @param [in] alc_sync_rx_buffer*          buffer that will be decoded
 * @param [in] alc_sync_rx_buffer_length    buffer length
 *
 * @return uint8_t                          Return a bitfield where each bit corresponding to a requested command
 */
uint8_t alc_sync_parser( alc_sync_ctx_t* ctx, uint8_t* alc_sync_rx_buffer, uint8_t alc_sync_rx_buffer_length );

/**
 * @brief Get ALC Sync periodicity
 *
 * @param [in] alc_sync_ctx_t *ctx      Clock sync pointer context
 * @return uint32_t alc_sync_periodicity_s
 */
uint32_t alc_sync_get_interval_second( alc_sync_ctx_t* ctx );

/**
 * @brief Set ALC Sync periodicity
 *
 * @param [in] alc_sync_ctx_t *ctx      Clock sync pointer context
 * @param interval_s                    Periodicity in seconds
 * @return true
 * @return false
 */
bool alc_sync_set_interval_second( alc_sync_ctx_t* ctx, uint32_t interval_s );

/**
 * @brief Get ALC Sync time correction
 * @remark return 0 for invalid correction
 * @param [in] alc_sync_ctx_t *ctx      Clock sync pointer context
 * @return uint8_t alc_sync_time_correction
 */
int32_t alc_sync_get_time_correction_second( alc_sync_ctx_t* ctx );

/**
 * @brief Set ALC Sync time correction
 *
 * @param [in] alc_sync_ctx_t *ctx      Clock sync pointer context
 * @param [in] time_correction_s        Set Correction in seconds
 * @return void
 */
void alc_sync_set_time_correction_second( alc_sync_ctx_t* ctx, int32_t time_correction_s );

/**
 * @brief Get ALC Sync GPS time
 * @remark return 0 for invalid time
 *
 * @param [in] alc_sync_ctx_t *ctx      Clock sync pointer context
 * @return uint32_t gps_time
 */
uint32_t alc_sync_get_gps_time_second( alc_sync_ctx_t* ctx );

/**
 * @brief Get ALC Sync time left in second before connection lost
 * @remark return 0 when no synchonisation
 *
 * @param [in] alc_sync_ctx_t *ctx      Clock sync pointer context
 * @return uint32_t time
 */
uint32_t alc_sync_get_time_left_connection_lost( alc_sync_ctx_t* ctx );

/**
 * @brief Get ALC Sync status (synchronisation done or not)
 *
 * @param [in] alc_sync_ctx_t *ctx      Clock sync pointer context
 * @return bool sync_done status
 */
bool is_alc_sync_done( alc_sync_ctx_t* ctx );

/**
 * @brief Get ALC Sync time validation (synchronisation done or not)
 *
 * @param [in] alc_sync_ctx_t *ctx      Clock sync pointer context
 * @return bool sync_done status
 */
bool is_alc_sync_time_valid( alc_sync_ctx_t* ctx );

/**
 * @brief Set the delays in second before concider that the time is no more sync
 *
 * @param [in] alc_sync_ctx_t *ctx      Clock sync pointer context
 * @param [in] delay_s
 * @return true
 * @return false
 */
bool alc_sync_set_valid_delay_second( alc_sync_ctx_t* ctx, uint32_t delay_s );

/**
 * @brief Get the delays in second before concider that the time is no more sync
 *
 * @param [in] alc_sync_ctx_t *ctx      Clock sync pointer context
 * @return uint32_t
 */
uint32_t alc_sync_get_valid_delay_second( alc_sync_ctx_t* ctx );

/**
 * @brief Set ALC Sync in synchronisation lost status
 *
 * @param [in] alc_sync_ctx_t *ctx      Clock sync pointer context
 * @return void
 */
void alc_sync_set_sync_lost( alc_sync_ctx_t* ctx );

/**
 * @brief Get ALC Sync time request nb transmission
 *
 * @param [in] alc_sync_ctx_t *ctx      Clock sync pointer context
 * @return uint8_t alc_sync_nb_transmission
 */
uint8_t alc_sync_get_nb_transmission( alc_sync_ctx_t* ctx );

/**
 * @brief Get ALC Sync token
 *
 * @param [in] alc_sync_ctx_t *ctx      Clock sync pointer context
 * @return uint8_t alc_sync_token_req   if greater to 0, the time is probably sync
 */
uint8_t alc_sync_get_token_req( alc_sync_ctx_t* ctx );

/**
 * @brief Set ALC Sync max payload length to construct the uplink frame
 *
 * @param [in] alc_sync_ctx_t *ctx      Clock sync pointer context
 * @param [in] max_payload              max payload that can be constructed
 * @return void
 */
void alc_sync_set_max_length_up_payload( alc_sync_ctx_t* ctx, uint8_t max_payload );

/**
 * @brief Get ALC Sync last rtc timestamp when received time sync
 *
 * @param [in] alc_sync_ctx_t *ctx      Clock sync pointer context
 * @return uint32_t                     timestamp last correction secondes
 */
uint32_t alc_sync_get_timestamp_last_correction_s( alc_sync_ctx_t* ctx );

/**
 * @brief Get ALC Sync timestamp when the answer requested bit must be activated
 *
 * @param [in] alc_sync_ctx_t *ctx      Clock sync pointer context
 * @return uint32_t
 */
uint32_t alc_sync_get_timestamp_ans_requested_s( alc_sync_ctx_t* ctx );

/*!
 * \brief   Create ALC Sync uplink payload
 *
 * \param   [in]  alc_sync_time             - Indicate the time when the payload will be really send
 * \param   [in]  app_time_ans_required     - True/False Set AnsRequired in AppTimeReq if AppTimeReq is present in
 *                                            uplink
 * \param   [in]  force_resync_status       - If true the NbTransmission set by ForceDeviceResyncReq must be
 * \param   [in]  max_payload_length        - final max length of the constructed payload
 * \param   [out] request_msg *             - Returned array that contains one or more concatenated ALC Sync data
 * \param   [out] payload_len *         - Returned array length
 * checked \retval void
 */
void alc_sync_create_uplink_payload( alc_sync_ctx_t* alc_ctx, uint32_t alc_sync_time, uint8_t app_time_ans_required,
                                     uint8_t force_resync_status, uint8_t max_payload_length, uint8_t* payload,
                                     uint8_t* payload_len );

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_ALC_SYNC_H__

/* --- EOF ------------------------------------------------------------------ */
