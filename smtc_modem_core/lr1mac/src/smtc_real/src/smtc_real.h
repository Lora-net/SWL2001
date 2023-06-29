/**
 * \file      smtc_real.h
 *
 * \brief     Region Abstraction Layer (REAL) API definition
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
#ifndef REAL_H
#define REAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "lr1mac_defs.h"
#include "smtc_real_defs.h"
#include "lr1_stack_mac_layer.h"

/*
 * ============================================================================
 * API definitions
 * ============================================================================
 */

/**
 * @brief Check if a region is supported by the stack
 *
 * @param region_type
 * @return smtc_real_status_t
 */
smtc_real_status_t smtc_real_is_supported_region( smtc_real_region_types_t region_type );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_init( smtc_real_t* real, smtc_real_region_types_t region_type );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_config( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_config_session( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_set_dr_distribution( smtc_real_t* real, uint8_t adr_mode, uint8_t* out_nb_trans, uint32_t* adr_custom );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] status_lorawan_t
 */
status_lorawan_t smtc_real_get_next_tx_dr( smtc_real_t* real, join_status_t join_status, dr_strategy_t* adr_mode_select,
                                           uint8_t* tx_data_rate, uint8_t tx_data_rate_adr, bool* adr_enable );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] status_lorawan_t
 */
uint16_t smtc_real_mask_tx_dr_channel( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] status_lorawan_t
 */
uint16_t smtc_real_mask_tx_dr_channel_up_dwell_time_check( smtc_real_t* real );

/**
 * @brief
 *
 * @param real
 * @return cf_list_type_t
 */
cf_list_type_t smtc_real_cf_list_type_supported( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_update_cflist( smtc_real_t* real, uint8_t* cf_list );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_get_number_of_chmask_in_cflist( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_get_next_channel( smtc_real_t* real, smtc_dtc_t* dtc_obj, uint8_t tx_data_rate,
                                             uint32_t* out_tx_frequency, uint32_t* out_rx1_frequency );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_get_join_next_channel( smtc_real_t* real, smtc_dtc_t* dtc_obj, uint8_t* tx_data_rate,
                                                  uint32_t* out_tx_frequency, uint32_t* out_rx1_frequency,
                                                  uint32_t* out_rx2_frequency );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_get_rx1_datarate_config( smtc_real_t* real, uint8_t tx_data_rate, uint8_t rx1_dr_offset );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
int8_t smtc_real_convert_power_cmd( smtc_real_t* real, uint8_t power_cmd, uint8_t max_erp_dbm );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_set_channel_mask( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_init_channel_mask( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_init_join_snapshot_channel_mask( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_init_after_join_snapshot_channel_mask( smtc_real_t* real, uint8_t tx_data_rate, uint32_t tx_frequency );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_channel_t smtc_real_build_channel_mask( smtc_real_t* real, uint8_t ch_mask_cntl, uint16_t ch_mask );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_decrement_dr_simulation( smtc_real_t* real, uint8_t tx_data_rate_adr );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_decrement_dr( smtc_real_t* real, dr_strategy_t adr_mode_select, uint8_t* tx_data_rate_adr,
                             int8_t* tx_power, uint8_t* nb_trans );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_enable_all_channels_with_valid_freq( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_is_rx1_dr_offset_valid( smtc_real_t* real, uint8_t rx1_dr_offset );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_is_rx_dr_valid( smtc_real_t* real, uint8_t dr );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_is_tx_dr_valid( smtc_real_t* real, uint8_t dr );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_is_tx_dr_acceptable( smtc_real_t* real, uint8_t dr, bool is_ch_mask_from_link_adr );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_is_nwk_received_tx_frequency_valid( smtc_real_t* real, uint32_t frequency );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_is_channel_index_valid( smtc_real_t* real, uint8_t channel_index );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_is_payload_size_valid( smtc_real_t* real, uint8_t dr, uint8_t size,
                                                  direction_frame_t direction_frame, uint8_t tx_fopts_current_length );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_set_tx_frequency_channel( smtc_real_t* real, uint32_t tx_freq, uint8_t index );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_set_rx1_frequency_channel( smtc_real_t* real, uint32_t rx_freq, uint8_t index );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_set_channel_dr( smtc_real_t* real, uint8_t channel_index, uint8_t dr_min, uint8_t dr_max );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_set_channel_enabled( smtc_real_t* real, uint8_t enable, uint8_t channel_index );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint32_t smtc_real_get_tx_channel_frequency( smtc_real_t* real, uint8_t index );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint32_t smtc_real_get_rx1_channel_frequency( smtc_real_t* real, uint8_t index );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_get_min_tx_channel_dr( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_get_max_tx_channel_dr( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
uint8_t smtc_real_get_preamble_len( const smtc_real_t* real, uint8_t sf );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
status_lorawan_t smtc_real_is_channel_mask_for_mobile_mode( const smtc_real_t* real );

/*************************************************************************/
/*                      Const init in region                             */
/*************************************************************************/

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
bool smtc_real_is_tx_param_setup_req_supported( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
bool smtc_real_is_new_channel_req_supported( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_get_rx1_join_delay( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_get_rx2_join_dr( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_get_frequency_factor( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
ral_lora_cr_t smtc_real_get_coding_rate( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_get_adr_ack_delay( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_get_adr_ack_limit( smtc_real_t* real );

/**
 * @brief
 *
 * @param real
 * @return uint8_t
 */
uint8_t smtc_real_get_public_sync_word( smtc_real_t* real );

/**
 * @brief
 *
 * @param real
 * @return uint8_t
 */
uint8_t smtc_real_get_private_sync_word( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_get_sync_word( smtc_real_t* real );

/**
 * @brief
 *
 * @param real
 * @param sync_word
 */
void smtc_real_set_sync_word( smtc_real_t* real, uint8_t sync_word );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t* smtc_real_get_gfsk_sync_word( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t* smtc_real_get_lr_fhss_sync_word( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_get_max_payload_size( smtc_real_t* real, uint8_t dr, direction_frame_t direction_frame );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_get_default_max_eirp( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_get_beacon_dr( smtc_real_t* real );

/**
 * @brief Check if in current region beacon hops in frequency
 * @remark use to check validity of freq 0 in class B multicast session
 *
 * @param [in] real
 * @return true
 * @return false
 */
bool smtc_real_is_beacon_hopping( smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint32_t smtc_real_get_beacon_frequency( smtc_real_t* real, uint32_t gps_time_s );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint32_t smtc_real_get_ping_slot_frequency( smtc_real_t* real, uint32_t gps_time_s, uint32_t dev_addr );

/**
 * @brief
 *
 * @param real
 * @return uint8_t
 */
uint8_t smtc_real_get_ping_slot_datarate( smtc_real_t* real );

/**
 * @brief
 *
 * @param real
 * @param dwell_time
 */
void smtc_real_set_uplink_dwell_time( smtc_real_t* real, bool dwell_time );

/**
 * @brief
 *
 * @param real
 * @param dwell_time
 */
void smtc_real_set_downlink_dwell_time( smtc_real_t* real, bool dwell_time );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint32_t smtc_real_decode_freq_from_buf( smtc_real_t* real, uint8_t freq_buf[3] );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_is_frequency_valid( smtc_real_t* real, uint32_t frequency );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_is_tx_power_valid( smtc_real_t* real, uint8_t power );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
bool smtc_real_is_dtc_supported( const smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
bool smtc_real_is_lbt_supported( const smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
uint32_t smtc_real_get_lbt_duration_ms( const smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
int16_t smtc_real_get_lbt_threshold_dbm( const smtc_real_t* real );

/**
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
uint32_t smtc_real_get_lbt_bw_hz( const smtc_real_t* real );

/**
 * @brief Get the corresponding RF modulation from a Datarate
 *
 * @param real
 * @param datarate
 * @return modulation_type_t
 */
modulation_type_t smtc_real_get_modulation_type_from_datarate( smtc_real_t* real, uint8_t datarate );

/**
 * @brief Convert LoRaWAN Datarate to LoRa SF and BW
 *
 * @param real
 * @param in_dr
 * @param out_sf
 * @param out_bw
 */
void smtc_real_lora_dr_to_sf_bw( smtc_real_t* real, uint8_t in_dr, uint8_t* out_sf, lr1mac_bandwidth_t* out_bw );

/**
 * @brief Convert LoRaWAN Datarate to FSK bitrate
 *
 * @param real
 * @param in_dr
 * @param out_bitrate
 */
void smtc_real_fsk_dr_to_bitrate( smtc_real_t* real, uint8_t in_dr, uint8_t* out_bitrate );

/**
 * @brief Convert LoRaWAN Datarate to LR-FHSS CR and BW
 *
 * @param real
 * @param in_dr
 * @param out_cr
 * @param out_bw
 */
void smtc_real_lr_fhss_dr_to_cr_bw( smtc_real_t* real, uint8_t in_dr, lr_fhss_v1_cr_t* out_cr,
                                    lr_fhss_v1_bw_t* out_bw );

/**
 * @brief Get LR-FHSS header count from LR-FHSS CR
 *
 * @param real
 * @param in_cr
 * @return lr_fhss_hc_t
 */
lr_fhss_hc_t smtc_real_lr_fhss_get_header_count( lr_fhss_v1_cr_t in_cr );

/**
 * @brief Get LR-FHSS grid
 *
 * @param  real
 * @return lr_fhss_v1_grid_t
 *
 */
lr_fhss_v1_grid_t smtc_real_lr_fhss_get_grid( smtc_real_t* real );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
int8_t smtc_real_clamp_output_power_eirp_vs_freq_and_dr( smtc_real_t* real, int8_t tx_power, uint32_t tx_frequency,
                                                         uint8_t datarate );

/**
 * @brief
 *
 * @param real
 * @param number_of_freq
 * @param freq_list
 * @param max_size
 * @return uint8_t
 */
uint8_t smtc_real_get_current_enabled_frequency_list( smtc_real_t* real, uint8_t* number_of_freq, uint32_t* freq_list,
                                                      uint8_t max_size );

/**
 * @brief
 *
 * @param real
 * @return lr1mac_version_t
 */
lr1mac_version_t smtc_real_get_regional_parameters_version( void );

/**
 * @brief
 *
 * @param
 * @return
 */
uint32_t smtc_real_get_symbol_duration_us( smtc_real_t* real, uint8_t datarate );

/**
 * @brief
 *
 * @param
 * @return
 */
void smtc_real_get_rx_window_parameters( smtc_real_t* real, uint8_t datarate, uint32_t rx_delay_ms,
                                         uint16_t* rx_window_symb, uint32_t* rx_timeout_symb_in_ms,
                                         uint32_t* rx_timeout_preamble_locked_in_ms, uint8_t rx_done_incertitude,
                                         uint32_t crystal_error );
/**
 * @brief
 *
 * @param
 * @return
 */

void smtc_real_get_rx_start_time_offset_ms( smtc_real_t* real, uint8_t datarate, int8_t board_delay_ms,
                                            uint16_t rx_window_symb, int32_t* rx_offset_ms );

#ifdef __cplusplus
}
#endif

#endif  // REAL_H
