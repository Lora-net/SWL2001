/**
 * @file      lr11xx_ranging.h
 *
 * @brief     Ranging driver definition for LR11XX
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

#ifndef LR11XX_RANGING_H
#define LR11XX_RANGING_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr11xx_ranging_types.h"
#include "lr11xx_radio_types.h"
#include "lr11xx_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Length in byte of the ranging result
 */
#define LR11XX_RANGING_RESULT_LENGTH ( 4 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTION PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Set the ranging address for this subordinate device.
 *
 * @param [in] context      Chip implementation context
 * @param [in] address      32-bit subordinate address (default is 0x00000019)
 * @param [in] check_length Number of bytes to be checked when comparing the device's
 *                          address with request address value contained in received
 *                          ranging frames (valid range 1..4, default is 4)
 *
 * @returns Operation status
 *
 * @note The address set by this function is only used in subordinate mode, that is,
 * when receiving ranging requests. While processing received request packets,
 * the ranging subordinate compares @p check_length bytes (LSB first) of
 * the request address with its own address. Packets with non-matching request
 * addresses are discarded.
 */
lr11xx_status_t lr11xx_ranging_set_address( const void* context, const uint32_t address, const uint8_t check_length );

/**
 * @brief Set the ranging address used for requests sent in manager mode.
 *
 * @param [in] context         Chip implementation context
 * @param [in] request_address 32-bit request address (default is 0x00000019)
 *
 * @returns Operation status
 *
 * @note The request address set by this function is only used in manager mode,
 * that is, when sending ranging requests. The @p request_address is copied
 * into the corresponding field in the next ranging request sent.
 */
lr11xx_status_t lr11xx_ranging_set_request_address( const void* context, const uint32_t request_address );

/**
 * @brief Set the transceiver RX/TX delay indicator to be compensated during ranging.
 *
 * The transceiver hardware induces a delay depending on the physical layer
 * configuration (bandwidth, spreading factor). To achieve the desired ranging
 * accuracy, this delay needs to be compensated by a calibration value.
 *
 * @param [in] context Chip implementation context
 * @param [in] delay_indicator   Delay value corresponding to the used bandwidth and spreading factor
 *
 * @returns lr11xx_status_t Operation status
 *
 * @note The same delay_indicator value needs to be configured in both manager and subordinate devices.
 */
lr11xx_status_t lr11xx_ranging_set_rx_tx_delay_indicator( const void* context, const uint32_t delay_indicator );

/**
 * @brief Configure ranging specific parameters.
 *
 * It is recommended to always call this command when configuring the ranging operation with @p nb_symbols = 15.
 * This value balances the ranging accuracy and power consumption.
 *
 * @param [in] context    Chip implementation context
 * @param [in] nb_symbols Number of symbols contained in responses sent by subordinates
 *
 * @returns lr11xx_status_t Operation status
 *
 * @note The ranging parameters need to be configured in both manager and subordinate devices.
 */
lr11xx_status_t lr11xx_ranging_set_parameters( const void* context, const uint8_t nb_symbols );

/**
 * @brief Get the ranging result on the manager device.
 *
 * Retrieve the ranging result item corresponding to the given item type @p type.
 *
 * @param [in]  context Chip implementation context
 * @param [in]  type    Result item type to be retrieved
 * @param [out] result  Result data buffer
 *
 * @returns lr11xx_status_t Operation status
 *
 * @note This function is only available on devices in manager mode after
 * the ranging is terminated.
 */
lr11xx_status_t lr11xx_ranging_get_raw_result( const void* context, const lr11xx_ranging_result_type_t type,
                                               uint8_t result[LR11XX_RANGING_RESULT_LENGTH] );

/**
 * @brief Get the recommended hardware delay indicator on ranging TX/RX.
 *
 * @param [in]  bw    LoRa bandwidth used for ranging
 * @param [in]  sf    LoRa spreading factor used for ranging
 * @param [out] delay_indicator Recommended hardware delay indicator for the given physical layer configuration
 *
 * @returns Operation status
 * @retval LR11XX_STATUS_OK on success
 * @retval LR11XX_STATUS_ERROR if the given physical layer configuration is invalid
 */
lr11xx_status_t lr11xx_ranging_get_recommended_rx_tx_delay_indicator( lr11xx_radio_lora_bw_t bw,
                                                                      lr11xx_radio_lora_sf_t sf,
                                                                      uint32_t*              delay_indicator );

/**
 * @brief Set recommended hardware delay indicator on ranging TX/RX for SF/BW.
 *
 * This helper function calls first @p lr11xx_ranging_get_recommended_rx_tx_delay_indicator to get the recommended delay
 * indicator value for the given SF/BW, then it calls @p lr11xx_ranging_set_rx_tx_delay_indicator to configure the
 * LR11xx.
 *
 * @param [in] context Chip implementation context
 * @param [in]  bw    LoRa bandwidth used for ranging
 * @param [in]  sf    LoRa spreading factor used for ranging
 *
 * @returns Operation status
 * @retval LR11XX_STATUS_OK on success
 * @retval LR11XX_STATUS_ERROR if the given physical layer configuration is invalid or other error
 */
lr11xx_status_t lr11xx_ranging_set_recommended_rx_tx_delay_indicator( const void* context, lr11xx_radio_lora_bw_t bw,
                                                                      lr11xx_radio_lora_sf_t sf );

/**
 * @brief Convert the raw distance result obtained from the device to a distance result [m].
 *
 * This function is meaningful only to convert a ranging result obtained by calling @p lr11xx_ranging_get_raw_result
 * with type set to @p LR11XX_RANGING_RESULT_TYPE_RAW
 *
 * @param [in] ranging_bw Bandwidth used during ranging
 * @param [in] raw_distance_buf Buffer containing the raw distance result
 *
 * @returns int32_t Distance result [m]
 *
 * @see lr11xx_ranging_get_raw_result
 *
 * @note The caller must ensure that the @p ranging_bw parameter is one of the supported ones,
 * i.e., #LR11XX_RADIO_LORA_BW_125, #LR11XX_RADIO_LORA_BW_250, #LR11XX_RADIO_LORA_BW_500.
 */
int32_t lr11xx_ranging_distance_raw_to_meter( lr11xx_radio_lora_bw_t ranging_bw,
                                              const uint8_t          raw_distance_buf[LR11XX_RANGING_RESULT_LENGTH] );

/**
 * @brief Convert the raw RSSI result obtained from the device to an RSSI result.
 *
 * This function is meaningful only to convert a ranging result obtained by calling @p lr11xx_ranging_get_raw_result
 * with type set to @p LR11XX_RANGING_RESULT_TYPE_RSSI
 *
 * @param [in] raw_rssi_buf Buffer containing the raw RSSI result
 *
 * @returns int8_t RSSI result [dBm]
 *
 * @see lr11xx_ranging_get_raw_result
 */
static inline int8_t lr11xx_ranging_rssi_raw_to_value( const uint8_t raw_rssi_buf[LR11XX_RANGING_RESULT_LENGTH] )
{
    // Only the last byte is meaningful
    return -( int8_t )( raw_rssi_buf[3] >> 1 );
}

#ifdef __cplusplus
}
#endif

#endif  // LR11XX_RANGING_H

/* --- EOF ------------------------------------------------------------------ */
