/*!
 * \file    wake_on_radio.h
 *
 * \brief   WOR and WOR ACK generation function
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
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
#ifndef WAKE_ON_RADIO_H
#define WAKE_ON_RADIO_H

#ifdef _cplusplus
extern "C" {
#endif

/*
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */
#include <stdint.h>
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */
#define MAX_WOR_CH ( 2 )
#define CHANNEL_TABLE_SIZE ( 16 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef enum wor_cad_periodicity_e
{
    WOR_CAD_PERIOD_1S,
    WOR_CAD_PERIOD_500MS,
    WOR_CAD_PERIOD_250MS,
    WOR_CAD_PERIOD_100MS,
    WOR_CAD_PERIOD_50MS,
    WOR_CAD_PERIOD_20MS,
    WOR_CAD_PERIOD_RFU,
} wor_cad_periodicity_t;

typedef enum wor_ack_ppm_error_e
{
    WOR_ACK_PPM_ERROR_10PPM,
    WOR_ACK_PPM_ERROR_20PPM,
    WOR_ACK_PPM_ERROR_30PPM,
    WOR_ACK_PPM_ERROR_40PPM,
} wor_ack_ppm_error_t;

typedef enum wor_ack_cad_to_rx_e
{
    WOR_ACK_CAD_TO_RX_2SYMB,
    WOR_ACK_CAD_TO_RX_4SYMB,
    WOR_ACK_CAD_TO_RX_6SYMB,
    WOR_ACK_CAD_TO_RX_8SYMB,
} wor_ack_cad_to_rx_t;

typedef enum wor_ack_forward_e
{
    WOR_ACK_FORWARD_OK,
    WOR_ACK_FORWARD_RETRY_30MIN,
    WOR_ACK_FORWARD_RETRY_60MIN,
    WOR_ACK_FORWARD_DISABLED
} wor_ack_forward_t;

typedef enum wor_msg_type_e
{
    WOR_MSG_TYPE_JOIN_REQUEST    = 0,
    WOR_MSG_TYPE_STANDARD_UPLINK = 1,
} wor_msg_type_t;

typedef struct wor_rf_infos_e
{
    uint32_t freq_hz;
    uint8_t  dr;
    int16_t  rssi_in_dbm;
    int16_t  snr_in_db;
    int16_t  signal_rssi_in_db;
} wor_rf_infos_t;

typedef struct wor_join_request_e
{
    uint32_t freq_hz;
    uint8_t  dr;
} wor_join_request_t;
typedef struct wor_uplink_e
{
    uint32_t freq_hz;
    uint8_t  dr;
    uint32_t devaddr;
    uint32_t fcnt;
    uint8_t  enc_data[4];
} wor_uplink_t;

typedef struct wor_infos_s
{
    wor_msg_type_t wor_type;
    union
    {
        wor_join_request_t join_request;
        wor_uplink_t       uplink;
    };

    wor_rf_infos_t rf_infos;
} wor_infos_t;

typedef struct wor_mic_infos_s
{
    uint32_t dev_addr;
    uint32_t wfcnt;
} wor_mic_infos_t;

typedef struct wor_ack_infos_s
{
    uint8_t               dr_relay_gtw;
    uint16_t              t_offset;
    wor_ack_ppm_error_t   relay_ppm;
    wor_cad_periodicity_t period;
    wor_ack_cad_to_rx_t   cad_to_rx;
    wor_ack_forward_t     relay_fwd;
} wor_ack_infos_t;

typedef struct wor_ack_mic_info_s
{
    uint32_t dev_addr;
    uint32_t wfcnt;
    uint8_t  datarate;
    uint32_t frequency_hz;
} wor_ack_mic_info_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

//-----------------------
// WOR frame function
/**
 * @brief Generate WOR payload to be send
 *
 * MIC is computed, doesn't need to add it
 *
 * @param[out]  buffer  Buffer to fill with WOR frame. Buffer has to be at least  15 bytes
 * @param[in]   wor     WOR infos to put in buffer
 * @return uint8_t      Length of ouput buffer
 */
uint8_t wor_generate_wor( uint8_t* buffer, const wor_infos_t* wor );

/**
 * @brief Compute MIC for WOR payload
 *
 * Only used in relay RX to check MIC
 *
 * @param[in]   mic_info        Info required to compute the MIC
 * @param[in]   wor_enc         WOR payload
 * @param[in]   wor_s_int_key   WOR Session Integrity Key
 * @return uint32_t MIC value
 */
uint32_t wor_compute_mic_wor( const wor_mic_infos_t* mic_info, const uint8_t* wor_enc,
                              const uint8_t wor_s_int_key[16] );

//-----------------------
// WOR ACK frame function

/**
 * @brief Compute WOR ACK MIC
 *
 * @param[in]   mic_info        Info required to compute MIC
 * @param[in]   ack_uplink_enc  WOR ACK encrypted payload
 * @param[in]   wor_s_int_key   WOR Session Integrity Key
 * @return uint32_t             MIC value
 */
uint32_t wor_compute_mic_ack( const wor_ack_mic_info_t* mic_info, const uint8_t* ack_uplink_enc,
                              const uint8_t wor_s_int_key[16] );

/**
 * @brief Decode WOR ACK data
 *
 * @param[in]   buffer          Input buffer with WOR ACK frame
 * @param[in]   mic_infos       Info required to compute MIC
 * @param[out]  ack             WOR ACK infos to filled
 * @param[in]   wor_s_enc_key   WOR Session Encryption Key
 */
void wor_decrypt_ack( const uint8_t* buffer, const wor_ack_mic_info_t* mic_infos, wor_ack_infos_t* ack,
                      const uint8_t wor_s_enc_key[16] );

/**
 * @brief Extract MIC from the WOR ACK payload
 *
 * Only used in relay TX to check MIC
 *
 * @param[in]   wor_ack_buffer  Input buffer with WOR ACK frame. Should be at least 7 bytes.
 * @return uint32_t     MIC value
 */
uint32_t wor_extract_mic_ack( const uint8_t* wor_ack_buffer );

//-----------------------
// Generic WOR function

/**
 * @brief Allow a device to derived relay session key from its network session key
 *
 * Only used in relay TX.
 * Key are stored directly in crypto module.
 * @param[in]   dev_addr    DevAddr of the device
 */
void wor_derive_root_skey( uint32_t dev_addr );

/**
 * @brief Convert enum CAD periodicity in ms value
 *
 * @param[in]   period  CAD period
 * @return uint16_t     CAD period in ms
 */
uint16_t wor_convert_cad_period_in_ms( const wor_cad_periodicity_t period );

/**
 * @brief Convert enum PPM in value in PPM
 *
 * @param[in]   ppm     PPM value with enum definition
 * @return uint8_t      PPM value in PPM
 */
uint8_t wor_convert_ppm( const wor_ack_ppm_error_t ppm );

/**
 * @brief Convert enum CAD_TO_RX in value in symbols
 *
 * @param[in]   cad_to_rx   CAD TO RX with enum definition
 * @return uint8_t          CAD TO RX in symbols
 */
uint8_t wor_convert_cadtorx( const wor_ack_cad_to_rx_t cad_to_rx );

#ifdef _cplusplus
}
#endif
#endif
