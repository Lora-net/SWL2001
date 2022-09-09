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

#define RX_SESSION_PARAM_CURRENT class_c_obj->rx_session_param[class_c_obj->rx_session_index]

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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static rx_packet_type_t lr1mac_class_c_mac_rx_frame_decode( lr1mac_class_c_t* class_c_obj );
static void             lr1mac_class_c_rp_callback( lr1mac_class_c_t* class_c_obj );
static int              lr1mac_class_c_mac_downlink_check_under_it( lr1mac_class_c_t* class_c_obj );
static void             lr1mac_class_c_launch( lr1mac_class_c_t* class_c_obj );
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lr1mac_class_c_init( lr1mac_class_c_t* class_c_obj, lr1_stack_mac_t* lr1_mac, smtc_multicast_t* multicast_obj,
                          radio_planner_t* rp, uint8_t class_c_id_rp, void ( *rx_callback )( void* rx_context ),
                          void* rx_context, void ( *push_callback )( void* push_context ), void* push_context )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "class_c_obj INIT\n" );
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

    class_c_obj->rx_session_param_unicast.enabled  = true;
    class_c_obj->rx_session_param_unicast.nwk_skey = SMTC_SE_NWK_S_ENC_KEY;
    class_c_obj->rx_session_param_unicast.app_skey = SMTC_SE_APP_S_KEY;

    class_c_obj->rx_session_param[RX_SESSION_UNICAST] = &class_c_obj->rx_session_param_unicast;

    if( multicast_obj != NULL )
    {
        // start to 1 because index 0 is set with lorawan class A value
        for( uint8_t i = 0; i < LR1MAC_MC_NUMBER_OF_SESSION; i++ )
        {
            class_c_obj->rx_session_param[i + 1] = &multicast_obj->rx_session_param[i];
        }
    }

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
#if defined( SMTC_MULTICAST )
    lr1mac_class_c_multicast_stop_all_sessions( class_c_obj );
#endif
    rp_task_abort( class_c_obj->rp, class_c_obj->class_c_id4rp );
    class_c_obj->rx_metadata.rx_window = RECEIVE_NONE;
}

void lr1mac_class_c_start( lr1mac_class_c_t* class_c_obj )
{
    if( class_c_obj->started == false )
    {
        lr1mac_class_c_launch( class_c_obj );
    }
}

void lr1mac_class_c_launch( lr1mac_class_c_t* class_c_obj )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "class_c_obj START (%d)\n", class_c_obj->started );
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
    class_c_obj->rx_session_param[RX_SESSION_UNICAST]->dev_addr     = class_c_obj->lr1_mac->dev_addr;
    class_c_obj->rx_session_param[RX_SESSION_UNICAST]->rx_data_rate = class_c_obj->lr1_mac->rx2_data_rate;
    class_c_obj->rx_session_param[RX_SESSION_UNICAST]->rx_frequency = class_c_obj->lr1_mac->rx2_frequency;

    class_c_obj->rx_session_index = RX_SESSION_COUNT;
    for( rx_session_type_t i = 0; i < LR1MAC_NUMBER_OF_RXC_SESSION; i++ )
    {
        if( class_c_obj->rx_session_param[i]->enabled == true )
        {
            class_c_obj->rx_session_index = i;
            break;
        }
    }

    if( class_c_obj->rx_session_index == RX_SESSION_COUNT )
    {
        smtc_modem_hal_lr1mac_panic( "no RxC session enabled\n" );
    }

    rp_radio_params_t rp_radio_params = { 0 };
    rp_radio_params.rx.timeout_in_ms  = 120000;

    modulation_type_t modulation_type =
        smtc_real_get_modulation_type_from_datarate( class_c_obj->lr1_mac, RX_SESSION_PARAM_CURRENT->rx_data_rate );

    if( modulation_type == LORA )
    {
        uint8_t            sf;
        lr1mac_bandwidth_t bw;
        smtc_real_lora_dr_to_sf_bw( class_c_obj->lr1_mac, RX_SESSION_PARAM_CURRENT->rx_data_rate, &sf, &bw );

        ralf_params_lora_t lora_param;
        memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );

        lora_param.sync_word       = smtc_real_get_sync_word( class_c_obj->lr1_mac );
        lora_param.symb_nb_timeout = 0;
        lora_param.rf_freq_in_hz   = RX_SESSION_PARAM_CURRENT->rx_frequency;

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
        uint8_t kbitrate;
        smtc_real_fsk_dr_to_bitrate( class_c_obj->lr1_mac, RX_SESSION_PARAM_CURRENT->rx_data_rate, &kbitrate );
        ralf_params_gfsk_t gfsk_param;
        memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

        gfsk_param.sync_word      = smtc_real_get_gfsk_sync_word( class_c_obj->lr1_mac );
        gfsk_param.dc_free_is_on  = true;
        gfsk_param.whitening_seed = GFSK_WHITENING_SEED;
        gfsk_param.crc_seed       = GFSK_CRC_SEED;
        gfsk_param.crc_polynomial = GFSK_CRC_POLYNOMIAL;
        gfsk_param.rf_freq_in_hz  = RX_SESSION_PARAM_CURRENT->rx_frequency;

        gfsk_param.pkt_params.header_type           = RAL_GFSK_PKT_VAR_LEN;
        gfsk_param.pkt_params.pld_len_in_bytes      = 255;
        gfsk_param.pkt_params.preamble_len_in_bits  = 40;
        gfsk_param.pkt_params.sync_word_len_in_bits = 24;
        gfsk_param.pkt_params.dc_free               = RAL_GFSK_DC_FREE_WHITENING;
        gfsk_param.pkt_params.crc_type              = RAL_GFSK_CRC_2_BYTES_INV;

        gfsk_param.mod_params.fdev_in_hz   = 25000;
        gfsk_param.mod_params.bw_dsb_in_hz = 100000;
        gfsk_param.mod_params.pulse_shape  = RAL_GFSK_PULSE_SHAPE_BT_1;
        gfsk_param.mod_params.br_in_bps    = kbitrate * 1000;

        rp_radio_params.pkt_type = RAL_PKT_TYPE_GFSK;
        rp_radio_params.rx.gfsk  = gfsk_param;
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( "MODULATION NOT SUPPORTED\n" );
    }

    rp_task_t rp_task        = { 0 };
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
    else
    {
        class_c_obj->started = true;
    }
    class_c_obj->rx_metadata.rx_window = RECEIVE_NONE;
}

static void lr1mac_class_c_rp_callback( lr1mac_class_c_t* class_c_obj )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "%s\n", __func__ );

    rp_status_t rp_status = class_c_obj->rp->status[class_c_obj->class_c_id4rp];
    if( rp_status == RP_STATUS_RX_PACKET )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "--> RP_STATUS_RX_PACKET\n" );
        class_c_obj->rx_callback( class_c_obj->rx_context );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "--> %d\n", rp_status );
    }

    if( class_c_obj->started == true )
    {
        lr1mac_class_c_launch( class_c_obj );
    }
}

void lr1mac_class_c_mac_rp_callback( lr1mac_class_c_t* class_c_obj )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "%s\n", __func__ );

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
            class_c_obj->valid_rx_packet = lr1mac_class_c_mac_rx_frame_decode( class_c_obj );

            SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "Receive a downlink RXC for Hook Id = %d\n", from_hook_id );

            if( class_c_obj->valid_rx_packet == USER_RX_PACKET )
            {
                SMTC_MODEM_HAL_TRACE_ARRAY_DEBUG( "RxC app Payload", class_c_obj->rx_payload,
                                                  class_c_obj->rx_payload_size );
                class_c_obj->rx_metadata.rx_datarate     = RX_SESSION_PARAM_CURRENT->rx_data_rate;
                class_c_obj->rx_metadata.rx_frequency_hz = RX_SESSION_PARAM_CURRENT->rx_frequency;
                // take also multicast rx in count in window type
                class_c_obj->rx_metadata.rx_window = RECEIVE_ON_RXC + ( uint8_t ) class_c_obj->rx_session_index;

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

#if defined( SMTC_MULTICAST )
smtc_multicast_config_rc_t lr1mac_class_c_multicast_start_session( lr1mac_class_c_t* class_c_obj, uint8_t mc_group_id,
                                                                   uint32_t freq, uint8_t dr )
{
    // Class C must be running to start multicast
    if( class_c_obj->enabled == false )
    {
        return SMTC_MC_RC_ERROR_CLASS_NOT_ENABLED;
    }

    // Check if multicast group id is in acceptable range
    if( mc_group_id > ( LR1MAC_MC_NUMBER_OF_SESSION - 1 ) )
    {
        return SMTC_MC_RC_ERROR_BAD_ID;
    }

    // check if there is an ongoing multicast session on this group_id
    if( class_c_obj->rx_session_param[mc_group_id + 1]->enabled == true )
    {
        return SMTC_MC_RC_ERROR_BUSY;
    }

    // Check if frequency and datarate are acceptable
    if( ( smtc_real_is_frequency_valid( class_c_obj->lr1_mac, freq ) != OKLORAWAN ) ||
        ( smtc_real_is_rx_dr_valid( class_c_obj->lr1_mac, dr ) != OKLORAWAN ) )
    {
        return SMTC_MC_RC_ERROR_PARAM;
    }

    // Search for the first active multicast session
    uint32_t mc_session = 0;
    while( ( mc_session < LR1MAC_MC_NUMBER_OF_SESSION ) )
    {
        if( ( class_c_obj->rx_session_param[mc_session + 1]->enabled ) == true )
        {
            // an active multicast session was found => break
            break;
        }
        else
        {
            mc_session++;
        }
    }
    if( mc_session < LR1MAC_MC_NUMBER_OF_SESSION )
    {
        // at least one session is already enabled
        // Check if param are compatible with already enabled, if not do not accept session
        if( ( class_c_obj->rx_session_param[mc_session + 1]->rx_frequency != freq ) ||
            ( class_c_obj->rx_session_param[mc_session + 1]->rx_data_rate != dr ) )
        {
            return SMTC_MC_RC_ERROR_INCOMPATIBLE_SESSION;
        }
    }

    // Save param (for first session or compatible with already enabled session )
    class_c_obj->rx_session_param[mc_group_id + 1]->rx_frequency = freq;
    class_c_obj->rx_session_param[mc_group_id + 1]->rx_data_rate = dr;

    // Reset session fcntdown counter
    class_c_obj->rx_session_param[mc_group_id + 1]->fcnt_dwn = ~0;

    // Set the enable bit to true to activate the session
    class_c_obj->rx_session_param[mc_group_id + 1]->enabled = true;

    // Stop current unicast if param differs
    if( ( class_c_obj->rx_session_param[RX_SESSION_UNICAST]->rx_frequency != freq ) ||
        ( class_c_obj->rx_session_param[RX_SESSION_UNICAST]->rx_data_rate != dr ) )
    {
        class_c_obj->rx_session_param[RX_SESSION_UNICAST]->enabled = false;
        // Abort current continuous reception (will be automatically restarted in rp abort callback)
        rp_task_abort( class_c_obj->rp, class_c_obj->class_c_id4rp );
    }

    return SMTC_MC_RC_OK;
}

smtc_multicast_config_rc_t lr1mac_class_c_multicast_stop_session( lr1mac_class_c_t* class_c_obj, uint8_t mc_group_id )
{
    // Check if multicast group id is in acceptable range
    if( mc_group_id > ( LR1MAC_MC_NUMBER_OF_SESSION - 1 ) )
    {
        return SMTC_MC_RC_ERROR_BAD_ID;
    }

    // Set the enable bit to false to indicate that the session is stopped
    class_c_obj->rx_session_param[mc_group_id + 1]->enabled = false;

    // Reset frequency and datarate to their not init values
    class_c_obj->rx_session_param[mc_group_id + 1]->rx_frequency = 0;
    class_c_obj->rx_session_param[mc_group_id + 1]->rx_data_rate = 0xFF;

    uint8_t enabled_multicast_sessions = 0;

    // Check if there is still an enabled multicast session
    for( uint8_t i = 0; i < LR1MAC_MC_NUMBER_OF_SESSION; i++ )
    {
        if( class_c_obj->rx_session_param[i + 1]->enabled == true )
        {
            enabled_multicast_sessions++;
        }
    }

    // No more enabled multicast session => re enable unicast session
    if( enabled_multicast_sessions == 0 )
    {
        // Enable unicast session
        class_c_obj->rx_session_param[RX_SESSION_UNICAST]->enabled = true;
        // Abort current continuous reception for multicast session
        rp_task_abort( class_c_obj->rp, class_c_obj->class_c_id4rp );
        // a new rx c with unicast param task will be enqueue automatically if class C is still active
    }
    else
    {
        // At least 1 multicast session is still active, do nothing
    }
    return SMTC_MC_RC_OK;
}

smtc_multicast_config_rc_t lr1mac_class_c_multicast_stop_all_sessions( lr1mac_class_c_t* class_c_obj )
{
    uint8_t active_sessions = 0;

    for( uint8_t i = 0; i < LR1MAC_MC_NUMBER_OF_SESSION; i++ )
    {
        if( class_c_obj->rx_session_param[i + 1]->enabled == true )
        {
            // Set the enable bit to false to indicate that the session is stopped
            class_c_obj->rx_session_param[i + 1]->enabled = false;
            // Reset frequency and datarate to their not init values
            class_c_obj->rx_session_param[i + 1]->rx_frequency = 0;
            class_c_obj->rx_session_param[i + 1]->rx_data_rate = LR1MAC_MC_NO_DATARATE;
            // Increment the counter of active sessions
            active_sessions++;
        }
    }

    if( active_sessions != 0 )
    {
        // As there is no more multicast sessions enabled => restart unicast session
        // a new rx c with unicast param task will be enqueue automatically if class C is still active
        class_c_obj->rx_session_param[RX_SESSION_UNICAST]->enabled = true;
        // Abort current continuous reception for multicast sessions
        rp_task_abort( class_c_obj->rp, class_c_obj->class_c_id4rp );
    }
    return SMTC_MC_RC_OK;
}

smtc_multicast_config_rc_t lr1mac_class_c_multicast_get_session_status( lr1mac_class_c_t* class_c_obj,
                                                                        uint8_t mc_group_id, bool* is_session_started,
                                                                        uint32_t* freq, uint8_t* dr )
{
    // Check if multicast group id is in acceptable range
    if( mc_group_id > ( LR1MAC_MC_NUMBER_OF_SESSION - 1 ) )
    {
        return SMTC_MC_RC_ERROR_BAD_ID;
    }

    *is_session_started = class_c_obj->rx_session_param[mc_group_id + 1]->enabled;
    *freq               = class_c_obj->rx_session_param[mc_group_id + 1]->rx_frequency;
    *dr                 = class_c_obj->rx_session_param[mc_group_id + 1]->rx_data_rate;

    return SMTC_MC_RC_OK;
}
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static int lr1mac_class_c_mac_downlink_check_under_it( lr1mac_class_c_t* class_c_obj )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "%s\n", __func__ );
    int status = OKLORAWAN;

    class_c_obj->rx_session_index = RX_SESSION_COUNT;

    // check Mtype
    uint8_t rx_ftype_tmp = class_c_obj->rx_payload[0] >> 5;
    if( ( rx_ftype_tmp == JOIN_REQUEST ) || ( rx_ftype_tmp == JOIN_ACCEPT ) || ( rx_ftype_tmp == UNCONF_DATA_UP ) ||
        ( rx_ftype_tmp == CONF_DATA_UP ) || ( rx_ftype_tmp == REJOIN_REQUEST ) || ( rx_ftype_tmp == PROPRIETARY ) )
    {
        status += ERRORLORAWAN;
        SMTC_MODEM_HAL_TRACE_PRINTF( " BAD Ftype = %u for RX Frame \n", rx_ftype_tmp );
    }
    // check devaddr
    if( ( class_c_obj->lr1_mac->join_status == JOINED ) && ( status == OKLORAWAN ) )
    {
        uint32_t dev_addr_tmp = class_c_obj->rx_payload[1] + ( class_c_obj->rx_payload[2] << 8 ) +
                                ( class_c_obj->rx_payload[3] << 16 ) + ( class_c_obj->rx_payload[4] << 24 );

        for( rx_session_type_t i = 0; i < LR1MAC_NUMBER_OF_RXC_SESSION; i++ )
        {
            if( ( dev_addr_tmp == class_c_obj->rx_session_param[i]->dev_addr ) &&
                ( class_c_obj->rx_session_param[i]->enabled == true ) )
            {
                class_c_obj->rx_session_index = i;
                break;
            }
        }

        if( class_c_obj->rx_session_index >= LR1MAC_NUMBER_OF_RXC_SESSION )
        {
            status += ERRORLORAWAN;
            class_c_obj->rx_session_index = RX_SESSION_COUNT;
            for( rx_session_type_t i = 0; i < LR1MAC_NUMBER_OF_RXC_SESSION; i++ )
            {
                SMTC_MODEM_HAL_TRACE_INFO( " BAD DevAddr = %x for RX Frame and %x \n \n",
                                           class_c_obj->rx_session_param[i]->dev_addr, dev_addr_tmp );
            }
        }
    }

    if( status != OKLORAWAN )
    {
        class_c_obj->rx_payload_size = 0;
    }

    return ( status );
}

static rx_packet_type_t lr1mac_class_c_mac_rx_frame_decode( lr1mac_class_c_t* class_c_obj )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "%s\n", __func__ );
    int              status         = OKLORAWAN;
    rx_packet_type_t rx_packet_type = NO_MORE_VALID_RX_PACKET;
    uint32_t         mic_in;
    uint8_t          rx_ftype;
    uint8_t          rx_major;

    status += lr1mac_rx_payload_min_size_check( class_c_obj->rx_payload_size );
    status += lr1mac_rx_payload_max_size_check( class_c_obj->lr1_mac, class_c_obj->rx_payload_size,
                                                RX_SESSION_PARAM_CURRENT->rx_data_rate );

    if( status != OKLORAWAN )
    {
        return NO_MORE_VALID_RX_PACKET;
    }

    status += lr1mac_rx_mhdr_extract( class_c_obj->rx_payload, &rx_ftype, &rx_major, &class_c_obj->tx_ack_bit );
    if( status != OKLORAWAN )
    {
        return NO_MORE_VALID_RX_PACKET;
    }

    if( class_c_obj->rx_session_index != RX_SESSION_UNICAST )
    {
        if( ( class_c_obj->tx_ack_bit == true ) || ( rx_ftype == CONF_DATA_UP ) )
        {
            return NO_MORE_VALID_RX_PACKET;
        }
    }

    /************************************************************************/
    /*               Case : the receive packet is not a JoinResponse */
    /************************************************************************/

    // Read Fcntdown in lr1mac in case of the downlink came from unicast dev address
    class_c_obj->rx_session_param[RX_SESSION_UNICAST]->fcnt_dwn = class_c_obj->lr1_mac->fcnt_dwn;

    uint16_t fcnt_dwn_tmp       = 0;
    uint32_t fcnt_dwn_stack_tmp = RX_SESSION_PARAM_CURRENT->fcnt_dwn;

    status += lr1mac_rx_fhdr_extract(
        class_c_obj->rx_payload, class_c_obj->rx_payload_size, &( class_c_obj->rx_fopts_length ), &fcnt_dwn_tmp,
        RX_SESSION_PARAM_CURRENT->dev_addr, &( class_c_obj->rx_metadata.rx_fport ), &( class_c_obj->rx_payload_empty ),
        &( class_c_obj->rx_fctrl ), class_c_obj->rx_fopts );

    if( status == OKLORAWAN )
    {
        status = lr1mac_fcnt_dwn_accept( fcnt_dwn_tmp, &fcnt_dwn_stack_tmp );
    }
    if( status == OKLORAWAN )
    {
        class_c_obj->rx_payload_size = class_c_obj->rx_payload_size - MICSIZE;
        memcpy1( ( uint8_t* ) &mic_in, &class_c_obj->rx_payload[class_c_obj->rx_payload_size], MICSIZE );

        if( smtc_modem_crypto_verify_mic( &class_c_obj->rx_payload[0], class_c_obj->rx_payload_size,
                                          RX_SESSION_PARAM_CURRENT->nwk_skey, RX_SESSION_PARAM_CURRENT->dev_addr, 1,
                                          fcnt_dwn_stack_tmp, mic_in ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
        {
            status = ERRORLORAWAN;
        }
    }
    if( status == OKLORAWAN )
    {
        // Set FPending bit in stack
        // !!!! SHALL NOT USED IN CLASS C
        // class_c_obj->lr1_mac->rx_fpending_bit_current = ( class_c_obj->rx_fctrl >> 4 ) & 0x01;
        // class_c_obj->rx_metadata.rx_fpending_bit      = class_c_obj->lr1_mac->rx_fpending_bit_current;

        RX_SESSION_PARAM_CURRENT->fcnt_dwn = fcnt_dwn_stack_tmp;
        class_c_obj->lr1_mac->fcnt_dwn     = class_c_obj->rx_session_param[RX_SESSION_UNICAST]->fcnt_dwn;

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
                        class_c_obj->rx_payload_size, RX_SESSION_PARAM_CURRENT->app_skey,
                        RX_SESSION_PARAM_CURRENT->dev_addr, 1, RX_SESSION_PARAM_CURRENT->fcnt_dwn,
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

    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " RxC rx_packet_type = %d \n", rx_packet_type );
    return ( rx_packet_type );
}
/* --- EOF ------------------------------------------------------------------ */
