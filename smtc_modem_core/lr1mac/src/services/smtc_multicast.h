/*!
 * \file      smtc_multicast.h
 *
 * \brief     Multicast implementation
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
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __SMTC_MULTICAST_H__
#define __SMTC_MULTICAST_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "lr1mac_defs.h"
#include "smtc_secure_element.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
// clang-format off
#define LR1MAC_MC_NUMBER_OF_SESSION  ( RX_SESSION_COUNT - 1 ) // Remove the unicast session
#define LR1MAC_MC_NO_DATARATE 0xFF
// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef enum smtc_multicast_config_rc_e
{
    SMTC_MC_RC_OK,
    SMTC_MC_RC_ERROR_BAD_ID,
    SMTC_MC_RC_ERROR_BUSY,
    SMTC_MC_RC_ERROR_CRYPTO,
    SMTC_MC_RC_ERROR_PARAM,
    SMTC_MC_RC_ERROR_INCOMPATIBLE_SESSION,
    SMTC_MC_RC_ERROR_CLASS_NOT_ENABLED,
} smtc_multicast_config_rc_t;

typedef struct smtc_multicast_s
{
    lr1mac_rx_session_param_t rx_session_param[LR1MAC_MC_NUMBER_OF_SESSION];
} smtc_multicast_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief
 *
 * @param multicast_obj
 */
void smtc_multicast_init( smtc_multicast_t* multicast_obj );

/**
 * @brief Configure a multicast group address
 *
 * @param [in] multicast_obj    The multicast object
 * @param [in] mc_group_id      The multicast group id
 * @param [in] mc_group_address The chosen multicast address for group id
 * @return smtc_multicast_config_rc_t
 */
smtc_multicast_config_rc_t smtc_multicast_set_group_address( smtc_multicast_t* multicast_obj, uint8_t mc_group_id,
                                                             uint32_t mc_group_address );

/**
 * @brief Get the address of the chosen multicast group
 *
 * @param [in] multicast_obj     The multicast object
 * @param [in] mc_group_id       The multicast group id
 * @param [out] mc_group_address The current group multicast address
 * @return smtc_multicast_config_rc_t
 */
smtc_multicast_config_rc_t smtc_multicast_get_group_address( smtc_multicast_t* multicast_obj, uint8_t mc_group_id,
                                                             uint32_t* mc_group_address );

/**
 * @brief Configure a multicast group session keys
 *
 * @param [in] multicast_obj The multicast object
 * @param [in] mc_group_id   The multicast group id
 * @param [in] mc_ntw_skey   The multicast network session key for the group
 * @param [in] mc_app_skey   The multicast application session key for the group
 * @return smtc_multicast_config_rc_t
 */
smtc_multicast_config_rc_t smtc_multicast_set_group_keys( smtc_multicast_t* multicast_obj, uint8_t mc_group_id,
                                                          const uint8_t mc_ntw_skey[SMTC_SE_KEY_SIZE],
                                                          const uint8_t mc_app_skey[SMTC_SE_KEY_SIZE] );

/**
 * @brief Get if the multicast session is running
 *
 * @param multicast_obj
 * @param mc_group_id
 * @param session_running
 * @return smtc_multicast_config_rc_t
 */
smtc_multicast_config_rc_t smtc_multicast_get_running_status( smtc_multicast_t* multicast_obj, uint8_t mc_group_id,
                                                              bool* session_running );

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_MULTICAST_H__

/* --- EOF ------------------------------------------------------------------ */
