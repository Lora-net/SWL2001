/*!
 * @file      lr20xx_radio_common.h
 *
 * @brief     Radio common driver definition for LR20XX
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

#ifndef LR20XX_RADIO_COMMON_H
#define LR20XX_RADIO_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr20xx_status.h"
#include "lr20xx_radio_common_types.h"
#include <stdint.h>
#include <stdbool.h>

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
 * @brief Executes front end calibration procedure on given raw frequencies and Rx path
 *
 * The front end calibration calibrates:
 *   - the ADC offset
 *   - the poly-phase filter
 *   - the image
 * This function can be called only if the chip is neither in Rx nor Tx states.
 *
 * Upon completion, the chip will return to the same mode it was before calling this command.
 * Potential calibration issues can be read out with lr20xx_system_get_errors command.
 *
 * Up to three calibration configuration values can be given.
 * Only the provided and non-zero frequencies are calibrated.
 *
 * It is advised to configure calibration so that RF frequencies used during RF operations are at most 50MHz away from
 * a calibrated RF frequency.
 *
 * If no calibration configuration is given, then one front end calibration is executed on the next 4MHz multiple of the
 * currently configured RF frequency.
 *
 * @param [in] context Chip implementation context
 * @param [in] front_end_calibration_values Array of front end calibration configuration. It is up to the caller to
 * ensure that it has at least n_rx_path_frequency elements.
 * @param [in] n_front_end_calibration_values Number of front end calibration values to consider. Valid values are [0:3]
 * included.
 *
 * @returns Operation status
 *
 * @see lr20xx_system_get_errors, lr20xx_radio_common_calibrate_front_end_helper
 */
lr20xx_status_t lr20xx_radio_common_calibrate_front_end(
    const void* context, const lr20xx_radio_common_raw_front_end_calibration_value_t* front_end_calibration_values,
    uint8_t n_front_end_calibration_values );

/*!
 * @brief Helper function to execute front end calibration procedure
 *
 * This function really is a helper function that converts the front end calibration structures in argument to the
 * corresponding raw values, and calls @ref lr20xx_radio_common_calibrate_front_end.
 * For each given front end calibration frequency, the actual calibration frequency used is the next frequency multiple
 * of 4MHz following the given frequency.
 *
 * @param [in] context Chip implementation context
 * @param [in] front_end_calibration_structures Array of front end calibration configuration structures. It is up to the
 * user that it contains at least n_rx_path_frequency items.
 * @param [in] n_front_end_calibration_structures Number of front end calibration structures to consider. Valid values
 * are [0:3] included.
 *
 * @returns Operation status
 *
 * @see lr20xx_radio_common_calibrate_front_end
 */
lr20xx_status_t lr20xx_radio_common_calibrate_front_end_helper(
    const void* context, const lr20xx_radio_common_front_end_calibration_value_t* front_end_calibration_structures,
    uint8_t n_front_end_calibration_structures );

/**
 * @brief Helper function that computes the number of RTC steps from a given time in millisecond
 *
 * @param [in] time_in_ms Time in millisecond
 *
 * @returns Number of RTC steps
 */
uint32_t lr20xx_radio_common_convert_time_in_ms_to_rtc_step( uint32_t time_in_ms );

/*!
 * @brief Set the RF frequency to be used
 *
 * @param [in] context Chip implementation context
 * @param [in] freq_in_hz RF frequency in Hertz
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_rf_freq( const void* context, uint32_t freq_in_hz );

/*!
 * @brief Select the Rx path and set the boost mode
 *
 * @param [in] context Chip implementation context
 * @param [in] rx_path Rx path to be used
 * @param [in] boost_mode Boost mode applied to selected Rx path
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_rx_path( const void* context, lr20xx_radio_common_rx_path_t rx_path,
                                                 lr20xx_radio_common_rx_path_boost_mode_t boost_mode );

/*!
 * @brief Set the Power Amplifier configuration
 *
 * It must be called prior using @ref lr20xx_radio_common_set_tx_params.
 *
 * @param [in] context Chip implementation context
 * @param [in] pa_cfg The structure for PA configuration
 *
 * @see lr20xx_radio_common_set_tx_params
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_pa_cfg( const void* context, const lr20xx_radio_common_pa_cfg_t* pa_cfg );

/*!
 * @brief Set the parameters for TX power and power amplifier ramp time
 *
 * @ref lr20xx_radio_common_set_pa_cfg must be called prior calling lr20xx_radio_common_set_tx_params.
 *
 * The range of possible TX output power values depends on PA selected with @ref
 * lr20xx_radio_common_set_pa_cfg :
 *   - for @ref LR20XX_RADIO_COMMON_PA_SEL_LF : power value goes from -9.5dBm to +22dBm
 *       (ie. @p power_half_dbm from 0xED to 0x2C)
 *   - for @ref LR20XX_RADIO_COMMON_PA_SEL_HF : power value goes from -19.5dBm to +12dBm
 *       (ie. @p power_half_dbm from 0xD9 to 0x18)
 *
 * @param [in] context Chip implementation context
 * @param [in] power_half_dbm TX output power raw value, as 0.5dBm steps (so twice the value in dBm)
 * @param [in] ramp_time Ramping time configuration
 *
 * @see lr20xx_radio_common_set_pa_cfg
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_tx_params( const void* context, const int8_t power_half_dbm,
                                                   const lr20xx_radio_common_ramp_time_t ramp_time );

/*!
 * @brief Set RSSI calibration table(s)
 *
 * @param [in] context Chip implementation context
 * @param [in] rssi_cal_table_lf Pointer to RSSI calibration table for low frequency path. Can be NULL, in which case
 * this path is not configured
 * @param [in] rssi_cal_table_hf Pointer to RSSI calibration table for high frequency path. Can be NULL, in which case
 * this path is not configured
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_rssi_calibration(
    const void* context, const lr20xx_radio_common_rssi_calibration_gain_table_t* rssi_cal_table_lf,
    const lr20xx_radio_common_rssi_calibration_gain_table_t* rssi_cal_table_hf );

/*!
 * @brief Configure the chip mode shall be in after transmission or reception operation
 *
 * @remark The configured fallback mode is applied as soon as the chip leaves Tx / Rx mode:
 *   - after a successful transmission
 *   - after a successful reception if not set in continuous mode
 *   - after a successful reception in duty cycle mode
 *   - after a CAD operation (depending on configured exit mode)
 *   - when a timeout occurs
 *   - during automatic Tx/Rx (see @ref lr20xx_radio_common_configure_auto_tx_rx), both after the first Rx/Tx and the
 * second Tx/Rx
 *
 * @param [in] context Chip implementation context
 * @param [in] fallback_mode Chip mode to enter after transmission or reception operation
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_rx_tx_fallback_mode( const void*                                context,
                                                             const lr20xx_radio_common_fallback_modes_t fallback_mode );

/*!
 * @brief Set the packet type to be used
 *
 * @remark This command has to be sent prior to any modulation related configuration command
 *
 * @param [in] context Chip implementation context
 * @param [in] pkt_type Packet type to be configured
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_pkt_type( const void* context, lr20xx_radio_common_pkt_type_t pkt_type );

/*!
 * @brief Get the packet type currently in use
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_type Packet type currently in use
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_get_pkt_type( const void* context, lr20xx_radio_common_pkt_type_t* pkt_type );

/*!
 * @brief Set the event on which the Rx timeout is stopped
 *
 * Depending on the configuration, Rx timeout is stopped either on the detection of the following events:
 * - LoRa header detection (or Rx done in implicit mode) / GFSK syncword detection
 * - Preamble detection
 *
 * @param [in] context Chip implementation context
 * @param [in] is_stopped_on_preamble_detection If true, the timer stops on preamble detection
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_rx_timeout_stop_event( const void* context,
                                                               const bool  is_stopped_on_preamble_detection );

/*!
 * @brief Reset internal Rx stats
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_reset_rx_stats( const void* context );

/*!
 * @brief Get the instantaneous RSSI while the transceiver is in reception mode
 *
 * This command can be used during reception of a packet
 *
 * The instantaneous RSSI can be obtained with 0.5 dBm accuracy thanks to the output argument half_dbm_count, which is
 * either 0 or 1, using the following formula:
 *
 * RSSI = rssi_in_dbm - ( half_dbm_count * 0.5 )
 *
 * The pointer half_dbm_count can be NULL, in which case the value is not returned.
 *
 * @param [in] context Chip implementation context
 * @param [out] rssi_in_dbm Instantaneous RSSI.
 * @param [out] half_dbm_count Count of 0.5 dBm to subtract to value in dBm. Can be NULL.
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_get_rssi_inst( const void* context, int16_t* rssi_in_dbm, uint8_t* half_dbm_count );

/*!
 * @brief Start RX operations with a timeout in millisecond
 *
 * @remark To set the radio in Rx continuous mode, refer to @ref lr20xx_radio_common_set_rx_with_timeout_in_rtc_step
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_in_ms Timeout configuration in millisecond for RX operation
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_rx( const void* context, const uint32_t timeout_in_ms );

/*!
 * @brief Start RX operations with a timeout in RTC step
 *
 * The timeout duration is obtained by:
 * \f$ timeout\_duration\_ms = timeout\_in\_rtc\_step \times \frac{1}{32.768} \f$
 *
 * Maximal timeout value is 0xFFFFFE, which gives a maximal timeout of 511 seconds.
 *
 * The timeout argument can also have the following special values:
 * <table>
 * <tr><th> Special values </th><th> Meaning </th>
 * <tr><td> 0x000000 </td><td> RX single - transceiver stays in RX mode until a packet is received </td>
 * <tr><td> 0xFFFFFF </td><td> RX continuous - transceiver stays in RX mode even after reception of a packet </td>
 * </table>
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_in_rtc_step Timeout configuration in RTC step for RX operation
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_rx_with_timeout_in_rtc_step( const void*    context,
                                                                     const uint32_t timeout_in_rtc_step );

/*!
 * @brief Start RX operations with a pre-configured default timeout
 *
 * @remark The timeout has to be configured by calling either @ref lr20xx_radio_common_set_default_rx_tx_timeout or @ref
 * lr20xx_radio_common_set_default_rx_tx_timeout_in_rtc_step
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_rx_with_default_timeout( const void* context );

/*!
 * @brief Start transmission operation with a timeout in millisecond
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_in_ms Timeout configuration in millisecond for RX operation
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_tx( const void* context, const uint32_t timeout_in_ms );

/*!
 * @brief Start transmission operation with a timeout in RTC step
 *
 * The timeout duration is obtained by:
 * \f$ timeout\_duration\_ms = timeout_in_rtc_step \times \frac{1}{32.768} \f$
 *
 * Maximal timeout value is 0xFFFFFF, which gives a maximal timeout of 511 seconds.
 *
 * If \p timeout_in_rtc_step is set to 0, then no timeout is used.
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_in_rtc_step Timeout configuration in RTC step for TX operation
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_tx_with_timeout_in_rtc_step( const void*    context,
                                                                     const uint32_t timeout_in_rtc_step );

/*!
 * @brief Start TX operations with a pre-configured default timeout
 *
 * @remark The timeout has to be configured by calling either @ref lr20xx_radio_common_set_default_rx_tx_timeout or @ref
 * lr20xx_radio_common_set_default_rx_tx_timeout_in_rtc_step
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_tx_with_default_timeout( const void* context );

/*!
 * @brief Set the transceiver into a Tx test mode.
 *
 * @param [in] context Chip implementation context
 * @param [in] mode Test mode
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_tx_test_mode( const void* context, lr20xx_radio_common_tx_test_mode_t mode );

/*!
 * @brief Select the Power Amplifier to use
 *
 * @remark Configuration has to be applied first by calling @ref lr20xx_radio_common_set_pa_cfg
 *
 * @param [in] context Chip implementation context
 * @param [in] sel Power amplifier selection
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_select_pa( const void* context, lr20xx_radio_common_pa_selection_t sel );

/*!
 * @brief Configure and start a Rx Duty Cycle operation with timings in millisecond
 *
 * @remark This function computes timings in RTC step from values given in millisecond and then calls @ref
 * lr20xx_radio_common_set_rx_duty_cycle_with_timing_in_rtc_step
 *
 * @param [in] context Chip implementation context
 * @param [in] rx_period_in_ms Rx period in millisecond
 * @param [in] sleep_period_in_ms Sleep period in millisecond
 * @param [in] mode Operation mode used during Rx phase
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_rx_duty_cycle( const void* context, const uint32_t rx_period_in_ms,
                                                       const uint32_t sleep_period_in_ms,
                                                       const lr20xx_radio_common_rx_duty_cycle_mode_t mode );

/*!
 * @brief Configure and start a Rx Duty Cycle operation with timings in RTC step
 *
 * It executes the following steps:
 *     1. Reception - enters reception state for duration defined by @p rx_period_in_rtc_step:
 *         - @p mode = LR20XX_RADIO_COMMON_RX_DUTY_CYCLE_MODE_RX: regular Rx mode
 *         - @p mode = LR20XX_RADIO_COMMON_RX_DUTY_CYCLE_MODE_CAD (LoRa only) : CAD mode
 *     2. Depending on the over-the-air activity detection (either preamble detection or valid CAD):
 *         - In case of positive over-the-air detection, the Rx period timeout is restarted with the value
 *           \f$2 \times rx_period_in_rtc_step + sleep_period_in_rtc_step\f$
 *         - else, the transceiver goes into sleep mode with retention for a duration defined by @p
 * sleep_period_in_rtc_step
 *     3. On wake-up, the transceiver restarts the process to step 1
 *
 * The loop described above is terminated in the following cases:
 *     - a packet is received during a Rx window - the chip goes back to fallback mode configured with @ref
 * lr20xx_radio_common_set_rx_tx_fallback_mode
 *     - a call to @ref lr20xx_system_set_standby_mode is done during a Rx window
 *     - a call to @ref lr20xx_system_wakeup is done during a sleep phase - to prevent a possible race condition from
 * happening when the call is performed during the boot phase, it is recommended to call @ref
 * lr20xx_system_set_standby_mode when BUSY is going low
 *
 * @remark If @p mode is set to @ref LR20XX_RADIO_COMMON_RX_DUTY_CYCLE_MODE_CAD, CAD parameters have to be defined
 * before calling this function
 *
 * @param [in] context Chip implementation context
 * @param [in] rx_period_in_rtc_step Rx period in RTC step
 * @param [in] sleep_period_in_rtc_step Sleep period in RTC step
 * @param [in] mode Operation mode used during Rx phase
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_rx_duty_cycle_with_timing_in_rtc_step(
    const void* context, const uint32_t rx_period_in_rtc_step, const uint32_t sleep_period_in_rtc_step,
    const lr20xx_radio_common_rx_duty_cycle_mode_t mode );

/**
 * @brief Configure the automatic Tx operation after Rx, or automatic Rx operation after Tx
 *
 * This feature allows the chip to automatically execute a Tx operation after an Rx one; or to automatically execute an
 * Rx operation after a Tx one.
 *
 * The order of operation depends on the mode manually requested after issuing this command:
 *   - If the radio is set to Tx mode, then an automatic Rx will be executed;
 *   - If the radio is set to Rx mode, then an automatic Tx will be executed.
 *
 * This feature is similar to a call to @ref lr20xx_radio_common_set_tx_with_timeout_in_rtc_step (or @ref
 * lr20xx_radio_common_set_rx_with_timeout_in_rtc_step) after the given delay_in_tick. Therefore to fine tune the
 * instant of first bit automatically sent over-the-air (or reception window opening) other delays have to be taken into
 * account when determining the delay_in_tick value. For instance, but not limited to:
 *   - PA ramp-up
 *   - TCXO start time (if applicable)
 *   - Configured fallback mode
 *   - Radio state switching time
 *
 * When the automatic Tx/Rx is enabled, the chip is in the state configured by @ref
 * lr20xx_radio_common_set_rx_tx_fallback_mode between the end of Rx (or Tx) operation and the start of the next
 * automatic Tx (or Rx) operation.
 *
 * Calling @ref lr20xx_radio_common_configure_auto_tx_rx with condition being @ref LR20XX_RADIO_COMMON_AUTO_TX_RX_OFF
 * disables the automatic Tx or Rx behavior. Doing so after end of Rx (or Tx) operation and start of automatic Tx (or
 * Rx) also cancels the automatic Tx or Rx operation.
 *
 * Once the automatic operation triggers, the feature is automatically disabled. So that to engage again an automatic
 * operation after a manual one, the @ref lr20xx_radio_common_configure_auto_tx_rx must be called to enable it again.
 *
 * @param context Chip implementation context
 * @param configuration The configuration of the automatic Tx/Rx
 *
 * @see lr20xx_radio_common_set_tx_with_timeout_in_rtc_step, lr20xx_radio_common_set_rx_with_timeout_in_rtc_step,
 * lr20xx_radio_common_set_rx_tx_fallback_mode
 *
 * @return lr20xx_status_t
 */
lr20xx_status_t lr20xx_radio_common_configure_auto_tx_rx(
    const void* context, const lr20xx_radio_common_auto_tx_rx_configuration_t* configuration );

/*!
 * @brief Get the length in byte of the last received packet
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_len Length in byte of the last received packet
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_get_rx_packet_length( const void* context, uint16_t* pkt_len );

/*!
 * @brief Set default timeout values for RX and TX operations
 *
 * @param [in] context Chip implementation context
 * @param [in] rx_timeout_in_ms Timeout configuration in millisecond for RX operation
 * @param [in] tx_timeout_in_ms Timeout configuration in millisecond for TX operation
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_default_rx_tx_timeout( const void* context, uint32_t rx_timeout_in_ms,
                                                               uint32_t tx_timeout_in_ms );

/*!
 * @brief Set default timeout values for RX and TX operations
 *
 * @remark Special values defined for @ref lr20xx_radio_common_set_rx_with_timeout_in_rtc_step and @ref
 * lr20xx_radio_common_set_tx_with_timeout_in_rtc_step are also applicable here
 *
 * @param [in] context Chip implementation context
 * @param [in] rx_timeout_in_rtc_step Timeout configuration in RTC step for RX operation
 * @param [in] tx_timeout_in_rtc_step Timeout configuration in RTC step for TX operation
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_default_rx_tx_timeout_in_rtc_step( const void* context,
                                                                           uint32_t    rx_timeout_in_rtc_step,
                                                                           uint32_t    tx_timeout_in_rtc_step );

/*!
 * @brief Set a timestamp source for a given configuration slot
 *
 * @remark This command configure a source linked to a radio event that will then be used by @ref
 * lr20xx_radio_common_get_elapsed_time_in_tick to compute the elapsed time
 *
 * @param [in] context Chip implementation context
 * @param [in] cfg_slot Configuration slot
 * @param [in] source Timestamp source
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_timestamp_source( const void*                              context,
                                                          lr20xx_radio_common_timestamp_cfg_slot_t cfg_slot,
                                                          lr20xx_radio_common_timestamp_source_t   source );

/*!
 * @brief Get the elapsed time since radio event registered at given configuration slot
 *
 * @remark This is the time elapsed between the event configured with @ref lr20xx_radio_common_set_timestamp_source and
 * the NSS falling edge of this request
 *
 * @remark That radio must not be put in sleep mode between the configured event and the call to this function
 *
 * @param [in] context Chip implementation context
 * @param [in] cfg_slot Configuration slot
 * @param [out] elapsed_time_in_tick Elapsed time in 32MHz tick
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_get_elapsed_time_in_tick( const void*                              context,
                                                              lr20xx_radio_common_timestamp_cfg_slot_t cfg_slot,
                                                              uint32_t* elapsed_time_in_tick );

/*!
 * @brief Launch a CCA (Clear Channel Assessment) operation
 *
 * @param [in] context Chip implementation context
 * @param [in] duration CCA duration in 32MHz step
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_cca( const void* context, const uint32_t duration );

/*!
 * @brief Get the CCA values once the operation is over
 *
 * @param [in] context Chip implementation context
 * @param [out] cca_res Structure holding CCA result
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_get_cca_result( const void* context, lr20xx_radio_common_cca_res_t* cca_res );

/*!
 * @brief Set the gain to be used by the AGC (Automatic Gain Control)
 *
 * @param [in] context Chip implementation context
 * @param [in] gain Gain
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_agc_gain( const void* context, lr20xx_radio_common_gain_step_t gain );

/*!
 * @brief Set non-LoRa CAD parameters
 *
 * @remark This command is not applicable if the packet type is set to LoRa
 *
 * @param [in] context Chip implementation context
 * @param [in] params CAD parameters
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_cad_params( const void*                             context,
                                                    const lr20xx_radio_common_cad_params_t* params );

/*!
 * @brief Set the chip in non-LoRa CAD mode
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_common_set_cad( const void* context );

/**
 * @brief Get the Link Quality Indicator (LQI) of latest detected packet
 *
 * This function is only valid if the latest received packet is an FSK based modulation:
 * - FSK
 * - Bluetooth_LE
 * - OQPSK 15.4
 * - Wi-SUN
 * - Wireless M-Bus
 * - Z-Wave
 *
 * The value returned corresponds to the latest detected packet. It is valid from the packet detection (corresponding to
 * @ref LR20XX_SYSTEM_IRQ_PREAMBLE_DETECTED raised if enabled) until next Rx attempt (through call to @ref
 * lr20xx_radio_common_set_rx or @ref lr20xx_radio_common_set_rx_with_timeout_in_rtc_step for instance).
 *
 * @param[in] context Chip implementation context
 * @param[out] lqi The LQI value
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_common_get_lqi( const void* context, lr20xx_radio_common_lqi_value_t* lqi );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_COMMON_H

/* --- EOF ------------------------------------------------------------------ */
