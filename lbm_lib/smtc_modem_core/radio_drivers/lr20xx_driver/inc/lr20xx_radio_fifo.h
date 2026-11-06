/*!
 * @file      lr20xx_radio_fifo.h
 *
 * @brief     Radio FiFo driver definition for LR20XX
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

#ifndef LR20XX_RADIO_FIFO_H
#define LR20XX_RADIO_FIFO_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr20xx_status.h"
#include "lr20xx_radio_fifo_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Read data from RX First in First out (FiFo) radio memory
 *
 * The RX FiFo radio memory contains packet received or being received.
 *
 * @param [in] context Chip implementation context
 * @param [in] buffer The buffer to be filled with data read from RX FiFo. It is up to the caller to ensure it is at
 * least @p length bytes long.
 * @param [in] length The number of bytes to read from RX FiFo
 *
 * @returns Operation status
 *
 * @see lr20xx_radio_fifo_write_tx
 */
lr20xx_status_t lr20xx_radio_fifo_read_rx( const void* context, uint8_t* buffer, const uint16_t length );

/*!
 * @brief Write data to TX First in First out (FiFo) radio memory
 *
 * The TX FiFo radio memory contains packet to send.
 *
 * @param [in] context Chip implementation context
 * @param [in] buffer The buffer to be written to TX FiFo. It is up to the caller to ensure it is at least
 * @p length bytes long.
 * @param [in] length The number of bytes to write to TX FiFo
 *
 * @returns Operation status
 *
 * @see lr20xx_radio_fifo_read_rx
 */
lr20xx_status_t lr20xx_radio_fifo_write_tx( const void* context, const uint8_t* buffer, const uint16_t length );

/*!
 * @brief Clear Rx FIFO
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_fifo_clear_rx( const void* context );

/*!
 * @brief Clear Tx FIFO
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_fifo_clear_tx( const void* context );

/*!
 * @brief Get Rx FIFO level
 *
 * @param [in] context Chip implementation context
 * @param [out] fifo_level Rx FIFO level in byte
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_fifo_get_rx_level( const void* context, uint16_t* fifo_level );

/*!
 * @brief Get Tx FIFO level
 *
 * @param [in] context Chip implementation context
 * @param [out] fifo_level Tx FIFO level in byte
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_fifo_get_tx_level( const void* context, uint16_t* fifo_level );

/*!
 * @brief Configure FIFO events and threshold levels triggering a FIFO interrupt in Rx and Tx
 *
 * @remark When configured, the FIFO interrupts are triggered if the FIFO level crosses the threshold in the correct
 * direction. Therefore if a threshold related IRQ is cleared, it will be raised again only if the FIFO level crosses
 * the threshold on the correct direction.
 *
 * @param [in] context Chip implementation context
 * @param [in] rx_fifo_irq_enable FIFO events triggering an interrupt in Rx
 * @param [in] tx_fifo_irq_enable FIFO events triggering an interrupt in Tx
 * @param [in] rx_fifo_high_threshold Rx FIFO threshold above which an interrupt (if
 * LR20XX_RADIO_FIFO_FLAG_THRESHOLD_HIGH is enabled) is triggered
 * @param [in] tx_fifo_low_threshold Tx FIFO threshold below which an interrupt (if
 * LR20XX_RADIO_FIFO_FLAG_THRESHOLD_LOW is enabled) is triggered
 * @param [in] rx_fifo_low_threshold Rx FIFO threshold below which an interrupt (if
 * LR20XX_RADIO_FIFO_FLAG_THRESHOLD_LOW is enabled) is triggered
 * @param [in] tx_fifo_high_threshold Tx FIFO threshold above which an interrupt (if
 * LR20XX_RADIO_FIFO_FLAG_THRESHOLD_HIGH is enabled) is triggered
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_fifo_cfg_irq( const void* context, lr20xx_radio_fifo_flag_t rx_fifo_irq_enable,
                                           lr20xx_radio_fifo_flag_t tx_fifo_irq_enable, uint16_t rx_fifo_high_threshold,
                                           uint16_t tx_fifo_low_threshold, uint16_t rx_fifo_low_threshold,
                                           uint16_t tx_fifo_high_threshold );

/*!
 * @brief Clear specific IRQ flags for both Rx and Tx FIFO
 *
 * @param [in] context Chip implementation context
 * @param [in] rx_fifo_flags_to_clear Rx FIFO IRQ flags to clear
 * @param [in] tx_fifo_flags_to_clear Tx FIFO IRQ flags to clear
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_fifo_clear_irq_flags( const void* context, lr20xx_radio_fifo_flag_t rx_fifo_flags_to_clear,
                                                   lr20xx_radio_fifo_flag_t tx_fifo_flags_to_clear );

/*!
 * @brief Get FIFO events triggering a FIFO interrupt in Rx and Tx
 *
 * @param [in] context Chip implementation context
 * @param [out] rx_fifo_flags FIFO events triggering an interrupt in Rx
 * @param [out] tx_fifo_flags FIFO events triggering an interrupt in Tx
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_fifo_get_irq( const void* context, lr20xx_radio_fifo_flag_t* rx_fifo_flags,
                                           lr20xx_radio_fifo_flag_t* tx_fifo_flags );

/*!
 * @brief Clear and return FiFo IRQ flags
 *
 * @param [in] context Chip implementation context
 * @param [out] rx_fifo_flags Rx FiFo flags
 * @param [out] tx_fifo_flags Tx FiFo flags
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_fifo_get_and_clear_irq_flags( const void* context, lr20xx_radio_fifo_flag_t* rx_fifo_flags,
                                                           lr20xx_radio_fifo_flag_t* tx_fifo_flags );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_FIFO_H

/* --- EOF ------------------------------------------------------------------ */
