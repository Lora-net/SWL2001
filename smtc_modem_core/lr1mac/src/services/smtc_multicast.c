/*!
 * \file      smtc_multicast.c
 *
 * \brief     Multicast implementation
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

#include "smtc_multicast.h"

#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"

#include "smtc_modem_crypto.h"

#include <string.h>  //for memset
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct smtc_multicast_key_s
{
    smtc_se_key_identifier_t mc_app_skey;
    smtc_se_key_identifier_t mc_ntw_skey;
} smtc_multicast_key_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static smtc_multicast_key_t smtc_mc_skey_tab[LR1MAC_MC_NUMBER_OF_SESSION] = {
    {
        .mc_app_skey = SMTC_SE_MC_APP_S_KEY_0,
        .mc_ntw_skey = SMTC_SE_MC_NWK_S_KEY_0,
    },
    {
        .mc_app_skey = SMTC_SE_MC_APP_S_KEY_1,
        .mc_ntw_skey = SMTC_SE_MC_NWK_S_KEY_1,
    },
    {
        .mc_app_skey = SMTC_SE_MC_APP_S_KEY_2,
        .mc_ntw_skey = SMTC_SE_MC_NWK_S_KEY_2,
    },
    {
        .mc_app_skey = SMTC_SE_MC_APP_S_KEY_3,
        .mc_ntw_skey = SMTC_SE_MC_NWK_S_KEY_3,
    },
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void smtc_multicast_init( smtc_multicast_t* multicast_obj )
{
    memset( multicast_obj, 0, sizeof( smtc_multicast_t ) );

    // set rx_session_param multicast keys to no_key, freq to no freq, and datarate to no datarate
    for( uint8_t i = 0; i < LR1MAC_MC_NUMBER_OF_SESSION; i++ )
    {
        multicast_obj->rx_session_param[i].fcnt_dwn       = ~0;
        multicast_obj->rx_session_param[i].app_skey       = smtc_mc_skey_tab[i].mc_app_skey;
        multicast_obj->rx_session_param[i].nwk_skey       = smtc_mc_skey_tab[i].mc_ntw_skey;
        multicast_obj->rx_session_param[i].rx_data_rate   = LR1MAC_MC_NO_DATARATE;
        multicast_obj->rx_session_param[i].rx_window_symb = 6;
        multicast_obj->rx_session_param[i].fpending_bit   = MULTICAST_WO_FPENDING;
    }
}

smtc_multicast_config_rc_t smtc_multicast_set_group_address( smtc_multicast_t* multicast_obj, uint8_t mc_group_id,
                                                             uint32_t mc_group_address )
{
    // Check if multicast group id is in acceptable range
    if( mc_group_id > ( LR1MAC_MC_NUMBER_OF_SESSION - 1 ) )
    {
        return SMTC_MC_RC_ERROR_BAD_ID;
    }
    // check if there is an ongoing multicast session on this group_id
    if( ( multicast_obj->rx_session_param[mc_group_id].enabled == true ) ||
        ( multicast_obj->rx_session_param[mc_group_id].waiting_beacon_to_start == true ) )
    {
        return SMTC_MC_RC_ERROR_BUSY;
    }

    // save config in rx_session_param tab
    multicast_obj->rx_session_param[mc_group_id].dev_addr = mc_group_address;

    return SMTC_MC_RC_OK;
}

smtc_multicast_config_rc_t smtc_multicast_get_group_address( smtc_multicast_t* multicast_obj, uint8_t mc_group_id,
                                                             uint32_t* mc_group_address )

{
    // Check if multicast group id is in acceptable range
    if( mc_group_id > ( LR1MAC_MC_NUMBER_OF_SESSION - 1 ) )
    {
        return SMTC_MC_RC_ERROR_BAD_ID;
    }

    *mc_group_address = multicast_obj->rx_session_param[mc_group_id].dev_addr;
    return SMTC_MC_RC_OK;
}

smtc_multicast_config_rc_t smtc_multicast_set_group_keys( smtc_multicast_t* multicast_obj, uint8_t mc_group_id,
                                                          const uint8_t mc_ntw_skey[SMTC_SE_KEY_SIZE],
                                                          const uint8_t mc_app_skey[SMTC_SE_KEY_SIZE] )
{
    // Check if multicast group id is in acceptable range
    if( mc_group_id > ( LR1MAC_MC_NUMBER_OF_SESSION - 1 ) )
    {
        return SMTC_MC_RC_ERROR_BAD_ID;
    }
    // check if there is an ongoing multicast session on this group_id
    if( ( multicast_obj->rx_session_param[mc_group_id].enabled == true ) ||
        ( multicast_obj->rx_session_param[mc_group_id].waiting_beacon_to_start == true ) )
    {
        return SMTC_MC_RC_ERROR_BUSY;
    }

    // Save multicast keys
    if( smtc_modem_crypto_set_key( smtc_mc_skey_tab[mc_group_id].mc_ntw_skey, mc_ntw_skey ) !=
        SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Error setting multicast ntw_skey for group:%d\n", mc_group_id );
        return SMTC_MC_RC_ERROR_CRYPTO;
    }

    if( smtc_modem_crypto_set_key( smtc_mc_skey_tab[mc_group_id].mc_app_skey, mc_app_skey ) !=
        SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Error setting multicast ntw_skey for group:%d\n", mc_group_id );
        return SMTC_MC_RC_ERROR_CRYPTO;
    }

    return SMTC_MC_RC_OK;
}

smtc_multicast_config_rc_t smtc_multicast_get_running_status( smtc_multicast_t* multicast_obj, uint8_t mc_group_id,
                                                              bool* session_running )
{
    // Check if multicast group id is in acceptable range
    if( mc_group_id > ( LR1MAC_MC_NUMBER_OF_SESSION - 1 ) )
    {
        return SMTC_MC_RC_ERROR_BAD_ID;
    }
    // check if there is an ongoing multicast session on this group_id
    *session_running = multicast_obj->rx_session_param[mc_group_id].enabled;

    return SMTC_MC_RC_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
