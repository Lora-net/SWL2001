/*!
 * \file      lr1mac_class_c.c
 *
 * \brief     LoRaWAN Class C
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

#include "lr1mac_class_c.h"

#include "smtc_modem_hal_dbg_trace.h"
#include "radio_planner.h"
#include "lr1mac_defs.h"
#include "lr1mac_utilities.h"
#include "lr1_stack_mac_layer.h"
#include "lr1mac_core.h"
#include "smtc_real.h"

#include "smtc_modem_crypto.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * @brief Not defined frequency value
 */
#define LR1MAC_CLASS_C_MC_NO_FREQUENCY 0

/**
 * @brief Not defined datarate value
 */
#define LR1MAC_CLASS_C_MC_NO_DATARATE 0xFF

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct lr1mac_class_c_multicast_key_s
{
    smtc_se_key_identifier_t mc_app_skey;
    smtc_se_key_identifier_t mc_ntw_skey;
} lr1mac_class_c_multicast_key_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
lr1mac_class_c_multicast_key_t lr1mac_class_mc_skey_tab[LR1MAC_NUMBER_OF_MC_SESSION] = {
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
static rx_packet_type_t lr1mac_class_c_mac_rx_frame_decode( lr1mac_class_c_t* class_c_obj );
static void             lr1mac_class_c_rp_callback( lr1mac_class_c_t* class_c_obj );
static int              lr1mac_class_c_mac_downlink_check_under_it( lr1mac_class_c_t* class_c_obj );
static void             lr1mac_class_c_set_keys( lr1mac_class_c_t* class_c_obj );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lr1mac_class_c_init( lr1mac_class_c_t* class_c_obj, lr1_stack_mac_t* lr1_mac, radio_planner_t* rp,
                          uint8_t class_c_id_rp, void ( *rx_callback )( void* rx_context ), void* rx_context,
                          void ( *push_callback )( void* push_context ), void* push_context )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "class_c_obj INIT\n" );
    memset( class_c_obj, 0, sizeof( lr1mac_class_c_t ) );

    class_c_obj->lr1_mac       = lr1_mac;
    class_c_obj->rp            = rp;
    class_c_obj->class_c_id4rp = class_c_id_rp;  //@none protection if this id already used by un other task
    class_c_obj->enabled       = false;
    class_c_obj->started       = false;
    class_c_obj->rx_callback   = rx_callback;
    class_c_obj->rx_context    = rx_context;
    class_c_obj->push_callback = push_callback;
    class_c_obj->push_context  = push_context;

    // set rx_session_param multicast keys to no_key, freq to no freq, and datarate to no datarate
    for( uint8_t i = 0; i < LR1MAC_NUMBER_OF_MC_SESSION; i++ )
    {
        class_c_obj->rx_session_param[i + 1].app_skey     = SMTC_SE_NO_KEY;
        class_c_obj->rx_session_param[i + 1].nwk_skey     = SMTC_SE_NO_KEY;
        class_c_obj->rx_session_param[i + 1].rx_frequency = LR1MAC_CLASS_C_MC_NO_FREQUENCY;
        class_c_obj->rx_session_param[i + 1].rx_data_rate = LR1MAC_CLASS_C_MC_NO_DATARATE;
    }

    class_c_obj->rx_session_param[RX_SESSION_UNICAST].enabled = true;

    rp_release_hook( class_c_obj->rp, class_c_obj->class_c_id4rp );
    rp_hook_init( class_c_obj->rp, class_c_obj->class_c_id4rp, ( void ( * )( void* ) )( lr1mac_class_c_rp_callback ),
                  class_c_obj );
}

void lr1mac_class_c_enabled( lr1mac_class_c_t* class_c_obj, bool enable )
{
    if( enable == false )
    {
        lr1mac_class_c_stop( class_c_obj );
    }

    class_c_obj->enabled = enable;
}

void lr1mac_class_c_stop( lr1mac_class_c_t* class_c_obj )
{
    if( class_c_obj->enabled == false )
    {
        return;
    }
    class_c_obj->started = false;
    rp_task_abort( class_c_obj->rp, class_c_obj->class_c_id4rp );
    class_c_obj->receive_window_type = RECEIVE_NONE;
}

void lr1mac_class_c_start( lr1mac_class_c_t* class_c_obj )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "class_c_obj START (%d)\n", class_c_obj->started );
    if( class_c_obj->enabled == false )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "class_c_obj disabled\n" );
        return;
    }
    if( class_c_obj->rx_callback == NULL )
    {
        smtc_modem_hal_mcu_panic( "class_c_obj bad initialization \n" );
    }

    // copy context from LR1MAC class A for the unicast session
    class_c_obj->rx_session_param[RX_SESSION_UNICAST].dev_addr     = class_c_obj->lr1_mac->dev_addr;
    class_c_obj->rx_session_param[RX_SESSION_UNICAST].fcnt_dwn     = class_c_obj->lr1_mac->fcnt_dwn;
    class_c_obj->rx_session_param[RX_SESSION_UNICAST].rx_data_rate = class_c_obj->lr1_mac->rx2_data_rate;
    class_c_obj->rx_session_param[RX_SESSION_UNICAST].rx_frequency = class_c_obj->lr1_mac->rx2_frequency;

    class_c_obj->rx_session_param_ptr = NULL;
    for( rx_session_type_t i = 0; i < LR1MAC_NUMBER_OF_RXC_SESSION; i++ )
    {
        if( class_c_obj->rx_session_param[i].enabled == true )
        {
            class_c_obj->rx_session_param_ptr = &class_c_obj->rx_session_param[i];
        }
    }

    if( class_c_obj->rx_session_param_ptr == NULL )
    {
        smtc_modem_hal_lr1mac_panic( "no RxC session enabled\n" );
    }

    rp_radio_params_t rp_radio_params = { 0 };
    rp_radio_params.rx.timeout_in_ms  = RAL_RX_TIMEOUT_CONTINUOUS_MODE;

    uint8_t            sf;
    lr1mac_bandwidth_t bw;
    modulation_type_t  modulation_type;
    smtc_real_rx_dr_to_sf_bw( class_c_obj->lr1_mac, class_c_obj->rx_session_param_ptr->rx_data_rate, &sf, &bw,
                              &modulation_type );

    if( modulation_type == LORA )
    {
        ralf_params_lora_t lora_param;
        memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );

        lora_param.sync_word       = smtc_real_get_sync_word( class_c_obj->lr1_mac );
        lora_param.symb_nb_timeout = 0;
        lora_param.rf_freq_in_hz   = class_c_obj->rx_session_param_ptr->rx_frequency;

        lora_param.pkt_params.header_type      = RAL_LORA_PKT_EXPLICIT;
        lora_param.pkt_params.pld_len_in_bytes = 255;
        lora_param.pkt_params.crc_is_on        = false;
        lora_param.pkt_params.invert_iq_is_on  = true;
        lora_param.pkt_params.preamble_len_in_symb =
            smtc_real_get_preamble_len( class_c_obj->lr1_mac, lora_param.mod_params.sf );

        lora_param.mod_params.cr   = smtc_real_get_coding_rate( class_c_obj->lr1_mac );
        lora_param.mod_params.sf   = ( ral_lora_sf_t ) sf;
        lora_param.mod_params.bw   = ( ral_lora_bw_t ) bw;
        lora_param.mod_params.ldro = ral_compute_lora_ldro( lora_param.mod_params.sf, lora_param.mod_params.bw );

        rp_radio_params.pkt_type = RAL_PKT_TYPE_LORA;
        rp_radio_params.rx.lora  = lora_param;
    }
    else if( modulation_type == FSK )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "MODULATION FSK\n" );
        ralf_params_gfsk_t gfsk_param;
        memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

        gfsk_param.sync_word      = smtc_real_get_gfsk_sync_word( class_c_obj->lr1_mac );
        gfsk_param.dc_free_is_on  = true;
        gfsk_param.whitening_seed = GFSK_WHITENING_SEED;
        gfsk_param.crc_seed       = GFSK_CRC_SEED;
        gfsk_param.crc_polynomial = GFSK_CRC_POLYNOMIAL;
        gfsk_param.rf_freq_in_hz  = class_c_obj->rx_session_param_ptr->rx_frequency;

        gfsk_param.pkt_params.header_type           = RAL_GFSK_PKT_VAR_LEN;
        gfsk_param.pkt_params.pld_len_in_bytes      = 255;
        gfsk_param.pkt_params.preamble_len_in_bits  = 40;
        gfsk_param.pkt_params.sync_word_len_in_bits = 24;
        gfsk_param.pkt_params.dc_free               = RAL_GFSK_DC_FREE_WHITENING;
        gfsk_param.pkt_params.crc_type              = RAL_GFSK_CRC_2_BYTES_INV;

        gfsk_param.mod_params.fdev_in_hz   = 25000;
        gfsk_param.mod_params.bw_dsb_in_hz = 100000;
        gfsk_param.mod_params.pulse_shape  = RAL_GFSK_PULSE_SHAPE_BT_1;
        gfsk_param.mod_params.br_in_bps    = sf * 1000;

        rp_radio_params.pkt_type = RAL_PKT_TYPE_GFSK;
        rp_radio_params.rx.gfsk  = gfsk_param;
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( "MODULATION NOT SUPPORTED\n" );
    }

    rp_task_t rp_task;
    rp_task.hook_id          = class_c_obj->class_c_id4rp;
    rp_task.state            = RP_TASK_STATE_ASAP;
    rp_task.start_time_ms    = smtc_modem_hal_get_time_in_ms( );
    rp_task.duration_time_ms = LR1MAC_RCX_MIN_DURATION_MS;

    if( rp_radio_params.pkt_type == RAL_PKT_TYPE_LORA )
    {
        rp_task.type                  = RP_TASK_TYPE_RX_LORA;
        rp_task.launch_task_callbacks = lr1_stack_mac_rx_lora_launch_callback_for_rp;
    }
    else
    {
        rp_task.type                  = RP_TASK_TYPE_RX_FSK;
        rp_task.launch_task_callbacks = lr1_stack_mac_rx_gfsk_launch_callback_for_rp;
    }

    if( rp_task_enqueue( class_c_obj->rp, &rp_task, class_c_obj->rx_payload, 255, &rp_radio_params ) !=
        RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "class_c_obj START ERREUR \n" );
    }

    class_c_obj->started             = true;
    class_c_obj->receive_window_type = RECEIVE_NONE;
}

static void lr1mac_class_c_rp_callback( lr1mac_class_c_t* class_c_obj )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "%s\n", __func__ );

    rp_status_t rp_status = class_c_obj->rp->status[class_c_obj->class_c_id4rp];
    if( rp_status == RP_STATUS_RX_PACKET )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "--> RP_STATUS_RX_PACKET\n" );
        class_c_obj->rx_callback( class_c_obj->rx_context );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "--> %d\n", rp_status );
    }

    if( class_c_obj->started == true )
    {
        lr1mac_class_c_start( class_c_obj );
    }
}

void lr1mac_class_c_mac_rp_callback( lr1mac_class_c_t* class_c_obj )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "%s\n", __func__ );

    int      status = OKLORAWAN;
    uint32_t tcurrent_ms;
    uint8_t  from_hook_id;

    rp_hook_get_id( class_c_obj->rp, class_c_obj, &from_hook_id );
    rp_get_status( class_c_obj->rp, from_hook_id, &tcurrent_ms, &( class_c_obj->planner_status ) );

    switch( class_c_obj->planner_status )
    {
    case RP_STATUS_TX_DONE:
        break;

    case RP_STATUS_RX_PACKET:

        // save rssi and snr
        class_c_obj->rx_metadata.timestamp = tcurrent_ms;
        class_c_obj->rx_metadata.rx_snr = class_c_obj->rp->radio_params[from_hook_id].rx.lora_pkt_status.snr_pkt_in_db;
        class_c_obj->rx_metadata.rx_rssi =
            class_c_obj->rp->radio_params[from_hook_id].rx.lora_pkt_status.rssi_pkt_in_dbm;
        class_c_obj->rx_payload_size = ( uint8_t ) class_c_obj->rp->payload_size[from_hook_id];

        SMTC_MODEM_HAL_TRACE_PRINTF( "payload size receive = %u, snr = %d , rssi = %d\n", class_c_obj->rx_payload_size,
                                     class_c_obj->rp->radio_params[from_hook_id].rx.lora_pkt_status.snr_pkt_in_db,
                                     class_c_obj->rp->radio_params[from_hook_id].rx.lora_pkt_status.rssi_pkt_in_dbm );

        SMTC_MODEM_HAL_TRACE_ARRAY( "RxC Payload", class_c_obj->rx_payload, class_c_obj->rx_payload_size );

        status = lr1mac_class_c_mac_downlink_check_under_it( class_c_obj );

        if( status == OKLORAWAN )
        {
            // take also multicast rx in count in window type
            class_c_obj->receive_window_type = RECEIVE_ON_RXC + ( uint8_t ) class_c_obj->rx_session_type;
            // deprecated ( class_c_obj->receive_window_type == RECEIVE_NACK ) ? RECEIVE_ACK_ON_RX2 : RECEIVE_ON_RX2;

            class_c_obj->valid_rx_packet = lr1mac_class_c_mac_rx_frame_decode( class_c_obj );

            SMTC_MODEM_HAL_TRACE_PRINTF( "Receive a downlink RXC for Hook Id = %d\n", from_hook_id );

            if( class_c_obj->valid_rx_packet == USER_RX_PACKET )
            {
                SMTC_MODEM_HAL_TRACE_ARRAY( "RxC app Payload", class_c_obj->rx_payload, class_c_obj->rx_payload_size );

                class_c_obj->push_callback( class_c_obj->push_context );
            }
        }
        class_c_obj->valid_rx_packet = NO_MORE_VALID_RX_PACKET;

        break;
    case RP_STATUS_RX_CRC_ERROR:
        SMTC_MODEM_HAL_TRACE_PRINTF( "lr1mac RxC CRC ERROR\n" );
        break;

    case RP_STATUS_RX_TIMEOUT:
        SMTC_MODEM_HAL_TRACE_PRINTF( "lr1mac RxC Timeout \n" );
        break;
    case RP_STATUS_TASK_ABORTED:
        SMTC_MODEM_HAL_TRACE_PRINTF( "lr1mac RxC aborted by the radioplanner \n" );
        break;
    default:
        SMTC_MODEM_HAL_TRACE_PRINTF( "lr1mac RxC receive It RADIO error %u\n", class_c_obj->planner_status );
        break;
    }
}

lr1mac_multicast_config_rc_t lr1mac_class_c_multicast_set_group_config( lr1mac_class_c_t* class_c_obj,
                                                                        uint8_t mc_group_id, uint32_t mc_group_address,
                                                                        const uint8_t mc_ntw_skey[SMTC_SE_KEY_SIZE],
                                                                        const uint8_t mc_app_skey[SMTC_SE_KEY_SIZE] )
{
    // Check if multicast group id is in acceptable range
    if( mc_group_id > ( LR1MAC_NUMBER_OF_MC_SESSION - 1 ) )
    {
        return LR1MAC_MC_RC_ERROR_BAD_ID;
    }
    // check if there is an ongoing mylticast session on this group_id
    if( class_c_obj->rx_session_param[mc_group_id + 1].enabled == true )
    {
        return LR1MAC_MC_RC_ERROR_BUSY;
    }

    // save config in rx_session_param tab
    class_c_obj->rx_session_param[mc_group_id + 1].dev_addr = mc_group_address;

    // Save multicast keys
    if( smtc_modem_crypto_set_key( lr1mac_class_mc_skey_tab[mc_group_id].mc_ntw_skey, mc_ntw_skey ) !=
        SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Error setting multicast ntw_skey for group:%d\n", mc_group_id );
        return LR1MAC_MC_RC_ERROR_CRYPTO;
    }

    if( smtc_modem_crypto_set_key( lr1mac_class_mc_skey_tab[mc_group_id].mc_app_skey, mc_app_skey ) !=
        SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Error setting multicast ntw_skey for group:%d\n", mc_group_id );
        return LR1MAC_MC_RC_ERROR_CRYPTO;
    }

    // TODO: remove dynamic assignment and use a LUT
    class_c_obj->rx_session_param[mc_group_id + 1].app_skey = lr1mac_class_mc_skey_tab[mc_group_id].mc_app_skey;
    class_c_obj->rx_session_param[mc_group_id + 1].nwk_skey = lr1mac_class_mc_skey_tab[mc_group_id].mc_ntw_skey;

    return LR1MAC_MC_RC_OK;
}

lr1mac_multicast_config_rc_t lr1mac_class_c_multicast_get_group_config( lr1mac_class_c_t* class_c_obj,
                                                                        uint8_t           mc_group_id,
                                                                        uint32_t*         mc_group_address )

{
    // Check if multicast group id is in acceptable range
    if( mc_group_id > ( LR1MAC_NUMBER_OF_MC_SESSION - 1 ) )
    {
        return LR1MAC_MC_RC_ERROR_BAD_ID;
    }

    *mc_group_address = class_c_obj->rx_session_param[mc_group_id + 1].dev_addr;
    return LR1MAC_MC_RC_OK;
}

lr1mac_multicast_config_rc_t lr1mac_class_c_multicast_start_session( lr1mac_class_c_t* class_c_obj, uint8_t mc_group_id,
                                                                     uint32_t freq, uint8_t dr )
{
    // Check if multicast group id is in acceptable range
    if( mc_group_id > ( LR1MAC_NUMBER_OF_MC_SESSION - 1 ) )
    {
        return LR1MAC_MC_RC_ERROR_BAD_ID;
    }

    // check if there is an ongoing multicast session on this group_id
    if( class_c_obj->rx_session_param[mc_group_id + 1].enabled == true )
    {
        return LR1MAC_MC_RC_ERROR_BUSY;
    }

    // Check if multicast group has been configured ie has a valid key id (different from SMTC_SE_NO_KEY)
    if( class_c_obj->rx_session_param[mc_group_id + 1].app_skey == SMTC_SE_NO_KEY )
    {
        return LR1MAC_MC_RC_ERROR_NOT_INIT;
    }

    // Check if frequency and datarate are acceptable
    if( ( smtc_real_is_rx_frequency_valid( class_c_obj->lr1_mac, freq ) != OKLORAWAN ) ||
        ( smtc_real_is_rx_dr_valid( class_c_obj->lr1_mac, dr ) != OKLORAWAN ) )
    {
        return LR1MAC_MC_RC_ERROR_PARAM;
    }

    // Search for the first active multicast session
    uint32_t mc_session = 0;
    while( ( mc_session < LR1MAC_NUMBER_OF_MC_SESSION ) )
    {
        if( ( class_c_obj->rx_session_param[mc_session + 1].enabled ) == true )
        {
            // an active multicast session was found => break
            break;
        }
        else
        {
            mc_session++;
        }
    }
    if( mc_session < LR1MAC_NUMBER_OF_MC_SESSION )
    {
        // at least one session is already enabled
        // Check if param are compatible with already enabled, if not do not accept session
        if( ( class_c_obj->rx_session_param[mc_session + 1].rx_frequency != freq ) ||
            ( class_c_obj->rx_session_param[mc_session + 1].rx_data_rate != dr ) )
        {
            return LR1MAC_MC_RC_ERROR_INCOMPATIBLE_SESSION;
        }
    }

    // Save param (for first session or compatible with already enabled session )
    class_c_obj->rx_session_param[mc_group_id + 1].rx_frequency = freq;
    class_c_obj->rx_session_param[mc_group_id + 1].rx_data_rate = dr;

    // Set the enable bit to true to activate the session
    class_c_obj->rx_session_param[mc_group_id + 1].enabled = true;

    // Stop current unicast if param differs
    if( ( class_c_obj->rx_session_param[RX_SESSION_UNICAST].rx_frequency != freq ) ||
        ( class_c_obj->rx_session_param[RX_SESSION_UNICAST].rx_data_rate != dr ) )
    {
        class_c_obj->rx_session_param[RX_SESSION_UNICAST].enabled = false;
        // Abort current continuous reception (will be automatically restarted in rp abort callback)
        rp_task_abort( class_c_obj->rp, class_c_obj->class_c_id4rp );
    }

    return LR1MAC_MC_RC_OK;
}

lr1mac_multicast_config_rc_t lr1mac_class_c_multicast_get_session_status( lr1mac_class_c_t* class_c_obj,
                                                                          uint8_t mc_group_id, bool* is_session_started,
                                                                          uint32_t* freq, uint8_t* dr )
{
    // Check if multicast group id is in acceptable range
    if( mc_group_id > ( LR1MAC_NUMBER_OF_MC_SESSION - 1 ) )
    {
        return LR1MAC_MC_RC_ERROR_BAD_ID;
    }

    *is_session_started = class_c_obj->rx_session_param[mc_group_id + 1].enabled;
    *freq               = class_c_obj->rx_session_param[mc_group_id + 1].rx_frequency;
    *dr                 = class_c_obj->rx_session_param[mc_group_id + 1].rx_data_rate;

    return LR1MAC_MC_RC_OK;
}

lr1mac_multicast_config_rc_t lr1mac_class_c_multicast_stop_session( lr1mac_class_c_t* class_c_obj, uint8_t mc_group_id )
{
    // Check if multicast group id is in acceptable range
    if( mc_group_id > ( LR1MAC_NUMBER_OF_MC_SESSION - 1 ) )
    {
        return LR1MAC_MC_RC_ERROR_BAD_ID;
    }

    // Set the enable bit to false to indicate that the session is stopped
    class_c_obj->rx_session_param[mc_group_id + 1].enabled = false;

    // Reset frequency and datarate to their not init values
    class_c_obj->rx_session_param[mc_group_id + 1].rx_frequency = 0;
    class_c_obj->rx_session_param[mc_group_id + 1].rx_data_rate = 0xFF;

    uint8_t enabled_multicast_sessions = 0;

    // Check if there is still an enabled multicast session
    for( uint8_t i = 0; i < LR1MAC_NUMBER_OF_MC_SESSION; i++ )
    {
        if( class_c_obj->rx_session_param[i + 1].enabled == true )
        {
            enabled_multicast_sessions++;
        }
    }

    // No more enabled multicast session => re enable unicast session
    if( enabled_multicast_sessions == 0 )
    {
        // Enable unicast session
        class_c_obj->rx_session_param[RX_SESSION_UNICAST].enabled = true;
        // Abort current continuous reception for multicast session
        rp_task_abort( class_c_obj->rp, class_c_obj->class_c_id4rp );
        // a new rx c with unicast param task will be enqueue automatically if class C is still active
    }
    else
    {
        // At least 1 multicast session is still active, do nothing
    }
    return LR1MAC_MC_RC_OK;
}

lr1mac_multicast_config_rc_t lr1mac_class_c_multicast_stop_all_sessions( lr1mac_class_c_t* class_c_obj )
{
    uint8_t active_sessions = 0;

    for( uint8_t i = 0; i < LR1MAC_NUMBER_OF_MC_SESSION; i++ )
    {
        if( class_c_obj->rx_session_param[i + 1].enabled == true )
        {
            // Set the enable bit to false to indicate that the session is stopped
            class_c_obj->rx_session_param[i + 1].enabled = false;
            // Reset frequency and datarate to their not init values
            class_c_obj->rx_session_param[i + 1].rx_frequency = 0;
            class_c_obj->rx_session_param[i + 1].rx_data_rate = 0xFF;
            // Increment the counter of active sessions
            active_sessions++;
        }
    }

    if( active_sessions != 0 )
    {
        // As there is no more multicast sessions enabled => restart unicast session
        // a new rx c with unicast param task will be enqueue automatically if class C is still active
        class_c_obj->rx_session_param[RX_SESSION_UNICAST].enabled = true;
        // Abort current continuous reception for multicast sessions
        rp_task_abort( class_c_obj->rp, class_c_obj->class_c_id4rp );
    }
    return LR1MAC_MC_RC_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static int lr1mac_class_c_mac_downlink_check_under_it( lr1mac_class_c_t* class_c_obj )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "%s\n", __func__ );
    int status = OKLORAWAN;

    class_c_obj->rx_session_type = RX_SESSION_NONE;

    // check Mtype
    uint8_t rx_ftype_tmp = class_c_obj->rx_payload[0] >> 5;
    if( ( rx_ftype_tmp == JOIN_REQUEST ) || ( rx_ftype_tmp == JOIN_ACCEPT ) || ( rx_ftype_tmp == UNCONF_DATA_UP ) ||
        ( rx_ftype_tmp == CONF_DATA_UP ) || ( rx_ftype_tmp == REJOIN_REQUEST ) || ( rx_ftype_tmp == PROPRIETARY ) )
    {
        status += ERRORLORAWAN;
        SMTC_MODEM_HAL_TRACE_PRINTF( " BAD Ftype = %u for RX Frame \n", rx_ftype_tmp );
        class_c_obj->rx_session_type = RX_SESSION_NONE;
    }
    // check devaddr
    if( ( class_c_obj->lr1_mac->join_status == JOINED ) && ( status == OKLORAWAN ) )
    {
        uint32_t dev_addr_tmp = class_c_obj->rx_payload[1] + ( class_c_obj->rx_payload[2] << 8 ) +
                                ( class_c_obj->rx_payload[3] << 16 ) + ( class_c_obj->rx_payload[4] << 24 );

        for( rx_session_type_t i = 0; i < LR1MAC_NUMBER_OF_RXC_SESSION; i++ )
        {
            if( ( dev_addr_tmp == class_c_obj->rx_session_param[i].dev_addr ) &&
                ( class_c_obj->rx_session_param[i].enabled == true ) )
            {
                class_c_obj->rx_session_type = i;
                break;
            }
        }

        if( class_c_obj->rx_session_type < LR1MAC_NUMBER_OF_RXC_SESSION )
        {
            lr1mac_class_c_set_keys( class_c_obj );
            class_c_obj->rx_session_param_ptr = &class_c_obj->rx_session_param[class_c_obj->rx_session_type];
        }
        else
        {
            status += ERRORLORAWAN;
            class_c_obj->rx_session_type = RX_SESSION_NONE;
            for( rx_session_type_t i = 0; i < LR1MAC_NUMBER_OF_RXC_SESSION; i++ )
            {
                SMTC_MODEM_HAL_TRACE_INFO( " BAD DevAddr = %x for RX Frame and %x \n \n",
                                           class_c_obj->rx_session_param[i].dev_addr, dev_addr_tmp );
            }
        }
    }
    else
    {
        class_c_obj->rx_session_type = RX_SESSION_NONE;
    }

    if( status != OKLORAWAN )
    {
        class_c_obj->rx_payload_size = 0;
    }

    return ( status );
}

void lr1mac_class_c_set_keys( lr1mac_class_c_t* class_c_obj )
{
    switch( class_c_obj->rx_session_type )
    {
    case RX_SESSION_UNICAST:
        class_c_obj->rx_session_param[RX_SESSION_UNICAST].nwk_skey = SMTC_SE_NWK_S_ENC_KEY;
        class_c_obj->rx_session_param[RX_SESSION_UNICAST].app_skey = SMTC_SE_APP_S_KEY;
        break;
    case RX_SESSION_MULTICAST_G0:
        class_c_obj->rx_session_param[class_c_obj->rx_session_type].nwk_skey = SMTC_SE_MC_NWK_S_KEY_0;
        class_c_obj->rx_session_param[class_c_obj->rx_session_type].app_skey = SMTC_SE_MC_APP_S_KEY_0;
        break;
    case RX_SESSION_MULTICAST_G1:
        class_c_obj->rx_session_param[class_c_obj->rx_session_type].nwk_skey = SMTC_SE_MC_NWK_S_KEY_1;
        class_c_obj->rx_session_param[class_c_obj->rx_session_type].app_skey = SMTC_SE_MC_APP_S_KEY_1;
        break;
    case RX_SESSION_MULTICAST_G2:
        class_c_obj->rx_session_param[class_c_obj->rx_session_type].nwk_skey = SMTC_SE_MC_NWK_S_KEY_2;
        class_c_obj->rx_session_param[class_c_obj->rx_session_type].app_skey = SMTC_SE_MC_APP_S_KEY_2;
        break;
    case RX_SESSION_MULTICAST_G3:
        class_c_obj->rx_session_param[class_c_obj->rx_session_type].nwk_skey = SMTC_SE_MC_NWK_S_KEY_3;
        class_c_obj->rx_session_param[class_c_obj->rx_session_type].app_skey = SMTC_SE_MC_APP_S_KEY_3;
        break;
    case RX_SESSION_NONE:
    default:
        smtc_modem_hal_lr1mac_panic( );
        break;
    }
}
static rx_packet_type_t lr1mac_class_c_mac_rx_frame_decode( lr1mac_class_c_t* class_c_obj )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "%s\n", __func__ );
    int              status         = OKLORAWAN;
    rx_packet_type_t rx_packet_type = NO_MORE_VALID_RX_PACKET;
    uint32_t         mic_in;
    uint8_t          rx_ftype;
    uint8_t          rx_major;

    status += lr1mac_rx_payload_min_size_check( class_c_obj->rx_payload_size );
    status += lr1mac_rx_payload_max_size_check( class_c_obj->lr1_mac, class_c_obj->rx_payload_size,
                                                class_c_obj->rx_session_param_ptr->rx_data_rate );
    if( status != OKLORAWAN )
    {
        return NO_MORE_VALID_RX_PACKET;
    }

    status += lr1mac_rx_mhdr_extract( class_c_obj->rx_payload, &rx_ftype, &rx_major, &class_c_obj->tx_ack_bit );
    if( status != OKLORAWAN )
    {
        return NO_MORE_VALID_RX_PACKET;
    }

    /************************************************************************/
    /*               Case : the receive packet is not a JoinResponse */
    /************************************************************************/
    uint16_t fcnt_dwn_tmp       = 0;
    uint32_t fcnt_dwn_stack_tmp = class_c_obj->rx_session_param_ptr->fcnt_dwn;

    status += lr1mac_rx_fhdr_extract(
        class_c_obj->rx_payload, class_c_obj->rx_payload_size, &( class_c_obj->rx_fopts_length ), &fcnt_dwn_tmp,
        class_c_obj->rx_session_param_ptr->dev_addr, &( class_c_obj->rx_metadata.rx_fport ),
        &( class_c_obj->rx_payload_empty ), &( class_c_obj->rx_fctrl ), class_c_obj->rx_fopts );

    if( status == OKLORAWAN )
    {
        status = lr1mac_fcnt_dwn_accept( fcnt_dwn_tmp, &fcnt_dwn_stack_tmp );
    }
    if( status == OKLORAWAN )
    {
        class_c_obj->rx_payload_size = class_c_obj->rx_payload_size - MICSIZE;
        memcpy1( ( uint8_t* ) &mic_in, &class_c_obj->rx_payload[class_c_obj->rx_payload_size], MICSIZE );

        if( smtc_modem_crypto_verify_mic( &class_c_obj->rx_payload[0], class_c_obj->rx_payload_size,
                                          class_c_obj->rx_session_param_ptr->nwk_skey,
                                          class_c_obj->rx_session_param_ptr->dev_addr, 1, fcnt_dwn_stack_tmp,
                                          mic_in ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
        {
            status = ERRORLORAWAN;
        }
    }
    if( status == OKLORAWAN )
    {
        class_c_obj->rx_session_param_ptr->fcnt_dwn = fcnt_dwn_stack_tmp;
        class_c_obj->lr1_mac->fcnt_dwn              = class_c_obj->rx_session_param[RX_SESSION_UNICAST].fcnt_dwn;

        if( class_c_obj->rx_payload_empty == 0 )  // rx payload not empty
        {
            class_c_obj->rx_payload_size = class_c_obj->rx_payload_size - FHDROFFSET - 1 - class_c_obj->rx_fopts_length;

            if( class_c_obj->rx_metadata.rx_fport == 0 )
            {  // receive a mac management frame Fport 0

                SMTC_MODEM_HAL_TRACE_WARNING( " Receive an not valid packet RxC on port zero\n" );
            }
            else
            {
                if( smtc_modem_crypto_payload_decrypt(
                        &class_c_obj->rx_payload[FHDROFFSET + 1 + class_c_obj->rx_fopts_length],
                        class_c_obj->rx_payload_size, class_c_obj->rx_session_param_ptr->app_skey,
                        class_c_obj->rx_session_param_ptr->dev_addr, 1, class_c_obj->rx_session_param_ptr->fcnt_dwn,
                        &class_c_obj->rx_payload[0] ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
                {
                    smtc_modem_hal_lr1mac_panic( "Crypto error during payload decryption\n" );
                }
                if( class_c_obj->rx_fopts_length != 0 )
                {
                    SMTC_MODEM_HAL_TRACE_WARNING( " Receive an not valid packet RxC FOpts\n" );
                    status = ERRORLORAWAN;
                }
                else
                {
                    rx_packet_type                    = USER_RX_PACKET;
                    class_c_obj->available_app_packet = LORA_RX_PACKET_AVAILABLE;
                }
            }
        }
        /*
            Receive an empty user payload
            => if rx_fopts_length > 0 set rx_packet_type = USERRX_FOPTSPACKET and copy fopts data
            => notify the upper layer that the stack have received a payload : ack_bit is set to 1
        */
        else
        {
            if( class_c_obj->rx_fopts_length != 0 )
            {
                SMTC_MODEM_HAL_TRACE_WARNING( " Receive an not valid packet RxC FOpts\n" );
                status = ERRORLORAWAN;
            }
            else
            {
                rx_packet_type = USER_RX_PACKET;
            }
        }
    }

    if( status == OKLORAWAN )
    {
        class_c_obj->rx_ftype = rx_ftype;
        class_c_obj->rx_major = rx_major;
        if( lr1mac_core_next_free_duty_cycle_ms_get( class_c_obj->lr1_mac ) <= 0 )
        {
            class_c_obj->lr1_mac->tx_ack_bit = class_c_obj->tx_ack_bit;
        }
    }

    SMTC_MODEM_HAL_TRACE_PRINTF( " RxC rx_packet_type = %d \n", rx_packet_type );
    return ( rx_packet_type );
}
/* --- EOF ------------------------------------------------------------------ */
