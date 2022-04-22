/**
 * \file      region_eu_868.h
 *
 * \brief     region_eu_868  abstraction layer definition
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

#ifndef REGION_EU_868_H
#define REGION_EU_868_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

#include "smtc_real_defs.h"
#include "lr1mac_defs.h"
#include "lr1_stack_mac_layer.h"

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

/**
 * @brief Congigure the region with default value and pointers
 *
 * @param lr1_mac
 */
void region_eu_868_config( lr1_stack_mac_t* lr1_mac );

/**
 * @brief Initialize the regional boot parameter
 * @remark must be called before each join request
 *
 * @param lr1_mac
 */
void region_eu_868_init( lr1_stack_mac_t* lr1_mac );

/**
 * @brief Get the next channel for the future uplink
 *
 * @param lr1_mac
 * @return status_lorawan_t
 */
status_lorawan_t region_eu_868_get_next_channel( lr1_stack_mac_t* lr1_mac );

/**
 * @brief Get the next channel for the future join request
 *
 * @param lr1_mac
 * @return status_lorawan_t
 */
status_lorawan_t region_eu_868_get_join_next_channel( lr1_stack_mac_t* lr1_mac );

/**
 * @brief Set the datarate for Rx1 and Rx2
 *
 * @param lr1_mac
 * @param type
 */
void region_eu_868_set_rx_config( lr1_stack_mac_t* lr1_mac, rx_win_type_t type );

/**
 * @brief Set the channel mask received by the LinkADRReq
 *
 * @param lr1_mac
 */
void region_eu_868_set_channel_mask( lr1_stack_mac_t* lr1_mac );

/**
 * @brief Decrypt and build the Channel Mask from multiple atomic LinkADRReq
 *
 * @param lr1_mac
 * @param ChMaskCntl
 * @param ChMask
 * @return status_channel_t
 */
status_channel_t region_eu_868_build_channel_mask( lr1_stack_mac_t* lr1_mac, uint8_t ChMaskCntl, uint16_t ChMask );

/**
 * @brief Get the corresponding RF modulation from a Datarate
 *
 * @param datarate
 * @return modulation_type_t
 */
modulation_type_t region_eu_868_get_modulation_type_from_datarate( uint8_t datarate );

/**
 * @brief Convert LoRaWAN Datarate to LoRa SF and BW
 *
 * @param in_dr
 * @param out_sf
 * @param out_bw
 */
void region_eu_868_lora_dr_to_sf_bw( uint8_t in_dr, uint8_t* out_sf, lr1mac_bandwidth_t* out_bw );

/**
 * @brief Convert LoRaWAN Datarate to FSK bitrate
 *
 * @param lr1_mac
 * @param in_dr
 * @param out_bitrate
 */
void region_eu_868_fsk_dr_to_bitrate( uint8_t in_dr, uint8_t* out_bitrate );

/**
 * @brief Convert LoRaWAN Datarate to LR-FHSS CR and BW
 *
 * @param [in]  in_dr
 * @param [out] out_cr
 * @param [out] out_bw
 */
void region_eu_868_lr_fhss_dr_to_cr_bw( uint8_t in_dr, lr_fhss_v1_cr_t* out_cr, lr_fhss_v1_bw_t* out_bw );

#ifdef __cplusplus
}
#endif

#endif  // REGION_EU_868_H

/* --- EOF ------------------------------------------------------------------ */
