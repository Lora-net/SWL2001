/*!
 * \file      smtc_ping_slot.c
 *
 * \brief     Ping Slot management for LoRaWAN class B devices
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_ping_slot.h"
#include "radio_planner.h"
#include "lr1mac_defs.h"
#include "lr1mac_utilities.h"
#include "lr1_stack_mac_layer.h"
#include "lr1mac_core.h"
#include "lr1mac_defs.h"
#include "smtc_real.h"
#include "smtc_secure_element.h"
#include "smtc_modem_crypto.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define RX_SESSION_PARAM ping_slot_obj->rx_session_param
#define RX_SESSION_PARAM_CURRENT ping_slot_obj->rx_session_param[ping_slot_obj->rx_session_index]

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

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

/**
 * @brief Decode the mac layer of the Rx frame
 *
 * @param ping_slot_obj
 * @return rx_packet_type_t
 */
static rx_packet_type_t smtc_ping_slot_mac_rx_frame_decode( smtc_ping_slot_t* ping_slot_obj );

/**
 * @brief Get the windows timeout in ms to listen the preamble
 *
 * @param ping_slot_obj
 * @param nb_rx_window_symb
 * @param datarate
 * @return uint32_t
 */
static uint32_t smtc_ping_slot_get_duration_timeout_ms( smtc_ping_slot_t* ping_slot_obj, uint16_t nb_rx_window_symb,
                                                        uint8_t datarate );

/**
 * @brief Check the mac header validity (type, devaddr, ...)
 *
 * @param ping_slot_obj
 * @return int
 */
static int smtc_ping_slot_mac_downlink_check( smtc_ping_slot_t* ping_slot_obj );

/**
 * @brief Compute the next ping offset for each session (unicast, multicast)
 *
 * @param ping_slot_obj
 * @param timestamp     // new ping offset will be in future in regards of this timestamp
 */
static void smtc_ping_slot_compute_next_ping_offset_time( smtc_ping_slot_t* ping_slot_obj, uint32_t timestamp );

/**
 * @brief Search the next ping slot Rx window
 *
 * @remark the priority between to Rx ping slot is check here
 *
 * @param ping_slot_obj
 * @param timestamp_rtc
 */
static void smtc_ping_slot_search_closest_ping_offset_time( smtc_ping_slot_t* ping_slot_obj, uint32_t timestamp_rtc );

/**
 * @brief Compute Time On Air of a payload size
 *
 * @param lr1_mac
 * @param datarate      // Datarate of payload
 * @param payload_size  // Size of the payload
 * @return uint32_t
 */
static uint32_t smtc_ping_slot_compute_downlink_toa( lr1_stack_mac_t* lr1_mac, uint8_t datarate, uint8_t payload_size );

/**
 * @brief Configure the radio at time to open the ping slot
 *
 * @param rp_void
 */
static void ping_slot_mac_rx_lora_launch_callback_for_rp( void* rp_void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void smtc_ping_slot_init( smtc_ping_slot_t* ping_slot_obj, lr1_stack_mac_t* lr1_mac, smtc_multicast_t* multicast_obj,
                          radio_planner_t* rp, uint8_t ping_slot_id_rp, void ( *rx_callback )( void* rx_context ),
                          void* rx_context, void ( *push_callback )( void* push_context ), void* push_context )
{
    memset( ping_slot_obj, 0, sizeof( smtc_ping_slot_t ) );

    ping_slot_obj->lr1_mac                      = lr1_mac;
    ping_slot_obj->rp                           = rp;
    ping_slot_obj->ping_slot_id4rp              = ping_slot_id_rp;
    ping_slot_obj->rx_callback                  = rx_callback;
    ping_slot_obj->rx_context                   = rx_context;
    ping_slot_obj->push_callback                = push_callback;
    ping_slot_obj->push_context                 = push_context;
    ping_slot_obj->d2d_check_fcnt_down_callback = NULL;

    rp_release_hook( ping_slot_obj->rp, ping_slot_obj->ping_slot_id4rp );
    rp_hook_init( ping_slot_obj->rp, ping_slot_id_rp, ( void ( * )( void* ) )( smtc_ping_slot_rp_callback ),
                  ping_slot_obj );

    ping_slot_obj->rx_session_param_unicast.enabled        = true;
    ping_slot_obj->rx_session_param_unicast.fpending_bit   = UNICAST_FPENDING;
    ping_slot_obj->rx_session_param_unicast.rx_window_symb = MIN_PING_SLOT_WINDOW_SYMB;
    ping_slot_obj->rx_session_param_unicast.nwk_skey       = SMTC_SE_NWK_S_ENC_KEY;
    ping_slot_obj->rx_session_param_unicast.app_skey       = SMTC_SE_APP_S_KEY;

    ping_slot_obj->rx_session_param[RX_SESSION_UNICAST] = &ping_slot_obj->rx_session_param_unicast;
    ping_slot_obj->d2d_callback                         = NULL;

    if( multicast_obj != NULL )
    {
        // start to 1 because index 0 is set with lorawan class A value
        for( uint8_t i = 0; i < LR1MAC_MC_NUMBER_OF_SESSION; i++ )
        {
            ping_slot_obj->rx_session_param[i + 1] = &multicast_obj->rx_session_param[i];
        }
    }
}

void smtc_ping_slot_stop( smtc_ping_slot_t* ping_slot_obj )
{
    if( ping_slot_obj->enabled == false )
    {
        return;
    }
    ping_slot_obj->enabled = false;
#if defined( SMTC_MULTICAST )
    smtc_ping_slot_multicast_b_stop_all_sessions( ping_slot_obj );
#endif
    rp_task_abort( ping_slot_obj->rp, ping_slot_obj->ping_slot_id4rp );

    // Sent empty uplink at the upper layer to inform Network that class B is disabled
    ping_slot_obj->lr1_mac->tx_class_b_bit = 0;
}

void smtc_ping_slot_init_after_beacon( smtc_ping_slot_t* ping_slot_obj, uint32_t beacon_timestamp,
                                       uint32_t next_beacon_timestamp, uint32_t beacon_reserved_ms,
                                       uint32_t beacon_guard_ms, uint32_t beacon_epoch_time )
{
    ping_slot_obj->next_beacon_timestamp = next_beacon_timestamp;
    ping_slot_obj->beacon_reserved_ms    = beacon_reserved_ms;
    ping_slot_obj->beacon_guard_ms       = beacon_guard_ms;

    ping_slot_obj->rx_session_param[RX_SESSION_UNICAST]->dev_addr = ping_slot_obj->lr1_mac->dev_addr;

    ping_slot_obj->rx_session_param[RX_SESSION_UNICAST]->ping_slot_periodicity =
        ping_slot_obj->lr1_mac->ping_slot_periodicity_ans;

    for( rx_session_type_t i = 0; i < LR1MAC_NUMBER_OF_CLASS_B_SESSION; i++ )
    {
        if( ping_slot_obj->rx_session_param[i]->waiting_beacon_to_start == true )
        {
            ping_slot_obj->rx_session_param[i]->waiting_beacon_to_start = false;
            ping_slot_obj->rx_session_param[i]->enabled                 = true;
        }

        // Compute Ping Slot for each enabled session
        if( ping_slot_obj->rx_session_param[i]->enabled == true )
        {
            ping_slot_obj->rx_session_param[i]->ping_slot_parameters.ping_number =
                1 << ( 7 - ping_slot_obj->rx_session_param[i]->ping_slot_periodicity );
            ping_slot_obj->rx_session_param[i]->ping_slot_parameters.ping_period =
                1 << ( 5 + ping_slot_obj->rx_session_param[i]->ping_slot_periodicity );

            ping_slot_obj->rx_session_param[i]->ping_slot_parameters.ping_offset_time_100us =
                smtc_ping_slot_compute_first_slot(
                    beacon_timestamp, beacon_reserved_ms, beacon_epoch_time,
                    ping_slot_obj->rx_session_param[i]->dev_addr,
                    ping_slot_obj->rx_session_param[i]->ping_slot_parameters.ping_period );
            ping_slot_obj->rx_session_param[i]->ping_slot_parameters.ping_offset_time =
                ping_slot_obj->rx_session_param[i]->ping_slot_parameters.ping_offset_time_100us / 10;
        }
    }
}

/* function call by beacon_sniff obj at the begining of each beacon period
 */
void smtc_ping_slot_start( smtc_ping_slot_t* ping_slot_obj )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "Start Ping Slot\n" );

    if( ping_slot_obj->rx_callback == NULL )
    {
        smtc_modem_hal_mcu_panic( "ping_slot_obj bad initialization \n" );
    }

    if( lr1mac_core_is_time_valid( ping_slot_obj->lr1_mac ) == false )
    {
        smtc_ping_slot_stop( ping_slot_obj );
        SMTC_MODEM_HAL_TRACE_WARNING( "Ping Slot not started, time sync is not valid\n" );
        return;
    }

    if( ping_slot_obj->lr1_mac->ping_slot_info_user_req != USER_MAC_REQ_ACKED )
    {
        smtc_ping_slot_stop( ping_slot_obj );
        SMTC_MODEM_HAL_TRACE_WARNING( "Ping Slot not started, PinSlotInfoReq is needed\n" );
        return;
    }

    uint32_t          timestamp_rtc;
    uint32_t          ping_slot_seconds_since_epoch;
    uint32_t          ping_slot_fractional_second;
    uint32_t          ping_slot_freq;
    uint8_t           ping_slot_dr;
    modulation_type_t modulation_type;
    rp_radio_params_t rp_radio_params = { 0 };
    uint32_t          rx_timeout_symb_in_ms_tmp;
    uint32_t          rx_timeout_symb_locked_in_ms_tmp;
    rp_task_t         rp_task = { 0 };
    int32_t           rx_offset_ms_tmp;
    rp_hook_status_t  rp_status;

    do
    {
        timestamp_rtc = smtc_modem_hal_get_time_in_ms( );
        smtc_ping_slot_compute_next_ping_offset_time( ping_slot_obj, timestamp_rtc );
        if( ping_slot_obj->rx_session_param[RX_SESSION_UNICAST]->enabled == true )
        {
            // copy context from LR1MAC class A for the unicast session
            ping_slot_obj->rx_session_param[RX_SESSION_UNICAST]->dev_addr = ping_slot_obj->lr1_mac->dev_addr;

            lr1mac_core_convert_rtc_to_gps_epoch_time(
                ping_slot_obj->lr1_mac,
                ping_slot_obj->rx_session_param[RX_SESSION_UNICAST]->ping_slot_parameters.ping_offset_time,
                &ping_slot_seconds_since_epoch, &ping_slot_fractional_second );

            // If the frequency is not 0, the network changed it
            ping_slot_obj->rx_session_param[RX_SESSION_UNICAST]->rx_frequency =
                ( ping_slot_obj->lr1_mac->ping_slot_freq_hz != 0 )
                    ? ping_slot_obj->lr1_mac->ping_slot_freq_hz
                    : smtc_real_get_ping_slot_frequency(
                          ping_slot_obj->lr1_mac->real, ping_slot_seconds_since_epoch,
                          ping_slot_obj->rx_session_param[RX_SESSION_UNICAST]->dev_addr );

            ping_slot_obj->rx_session_param[RX_SESSION_UNICAST]->rx_data_rate = ping_slot_obj->lr1_mac->ping_slot_dr;

            // If class B bit is set to 0, enabled it
            // and sent an empty uplink to inform Network that ping slot are ready
            if( ping_slot_obj->lr1_mac->tx_class_b_bit == 0 )
            {
                ping_slot_obj->lr1_mac->tx_class_b_bit = 1;
            }
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "Ping Slot unicast not started\n" );
            return;
        }

        smtc_ping_slot_search_closest_ping_offset_time( ping_slot_obj, timestamp_rtc );

        if( ping_slot_obj->rx_session_index == RX_SESSION_COUNT )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Ping Slot no more session\n" );
            return;
        }

        ping_slot_obj->enabled = true;

        lr1mac_core_convert_rtc_to_gps_epoch_time( ping_slot_obj->lr1_mac,
                                                   RX_SESSION_PARAM_CURRENT->ping_slot_parameters.ping_offset_time,
                                                   &ping_slot_seconds_since_epoch, &ping_slot_fractional_second );

        ping_slot_freq =
            ( RX_SESSION_PARAM_CURRENT->rx_frequency != 0 )
                ? RX_SESSION_PARAM_CURRENT->rx_frequency
                : smtc_real_get_ping_slot_frequency( ping_slot_obj->lr1_mac->real, ping_slot_seconds_since_epoch,
                                                     RX_SESSION_PARAM_CURRENT->dev_addr );

        ping_slot_dr = RX_SESSION_PARAM_CURRENT->rx_data_rate;

        modulation_type = smtc_real_get_modulation_type_from_datarate( ping_slot_obj->lr1_mac->real, ping_slot_dr );

        smtc_real_get_rx_window_parameters( ping_slot_obj->lr1_mac->real, RX_SESSION_PARAM_CURRENT->rx_data_rate,
                                            ( RX_SESSION_PARAM_CURRENT->ping_slot_parameters.ping_offset_time -
                                              ping_slot_obj->last_valid_rx_beacon_ms ),
                                            &RX_SESSION_PARAM_CURRENT->rx_window_symb, &rx_timeout_symb_in_ms_tmp,
                                            &rx_timeout_symb_locked_in_ms_tmp, RX_BEACON_TIMESTAMP_ERROR,
                                            ping_slot_obj->lr1_mac->crystal_error );

        if( modulation_type == LORA )
        {
            uint8_t            sf;
            lr1mac_bandwidth_t bw;
            smtc_real_lora_dr_to_sf_bw( ping_slot_obj->lr1_mac->real, ping_slot_dr, &sf, &bw );

            ralf_params_lora_t lora_param;
            memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );

            lora_param.sync_word       = smtc_real_get_sync_word( ping_slot_obj->lr1_mac->real );
            lora_param.symb_nb_timeout = RX_SESSION_PARAM_CURRENT->rx_window_symb;
            lora_param.rf_freq_in_hz   = ping_slot_freq;

            lora_param.pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT;
            lora_param.pkt_params.pld_len_in_bytes     = 255;
            lora_param.pkt_params.crc_is_on            = false;
            lora_param.pkt_params.invert_iq_is_on      = true;
            lora_param.pkt_params.preamble_len_in_symb = 255;

            lora_param.mod_params.cr = smtc_real_get_coding_rate( ping_slot_obj->lr1_mac->real );
            lora_param.mod_params.sf = ( ral_lora_sf_t ) sf;
            lora_param.mod_params.bw = ( ral_lora_bw_t ) bw;

            lora_param.mod_params.ldro = ral_compute_lora_ldro( lora_param.mod_params.sf, lora_param.mod_params.bw );

            rp_radio_params.pkt_type         = RAL_PKT_TYPE_LORA;
            rp_radio_params.rx.lora          = lora_param;
            rp_radio_params.rx.timeout_in_ms = rx_timeout_symb_locked_in_ms_tmp;
        }
        else if( modulation_type == FSK )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "MODULATION FSK\n" );
            uint8_t kbitrate;
            smtc_real_fsk_dr_to_bitrate( ping_slot_obj->lr1_mac->real, ping_slot_dr, &kbitrate );

            ralf_params_gfsk_t gfsk_param;
            memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

            gfsk_param.sync_word      = smtc_real_get_gfsk_sync_word( ping_slot_obj->lr1_mac->real );
            gfsk_param.dc_free_is_on  = true;
            gfsk_param.whitening_seed = GFSK_WHITENING_SEED;
            gfsk_param.crc_seed       = GFSK_CRC_SEED;
            gfsk_param.crc_polynomial = GFSK_CRC_POLYNOMIAL;
            gfsk_param.rf_freq_in_hz  = ping_slot_freq;

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

            rp_radio_params.pkt_type         = RAL_PKT_TYPE_GFSK;
            rp_radio_params.rx.gfsk          = gfsk_param;
            rp_radio_params.rx.timeout_in_ms = rx_timeout_symb_in_ms_tmp;
        }
        else
        {
            smtc_modem_hal_lr1mac_panic( "MODULATION NOT SUPPORTED\n" );
        }

        rp_task.hook_id                    = ping_slot_obj->ping_slot_id4rp;
        rp_task.state                      = RP_TASK_STATE_SCHEDULE;
        rp_task.schedule_task_low_priority = true;
        int8_t board_delay_ms =
            smtc_modem_hal_get_radio_tcxo_startup_delay_ms( ) + smtc_modem_hal_get_board_delay_ms( );
        smtc_real_get_rx_start_time_offset_ms( ping_slot_obj->lr1_mac->real, RX_SESSION_PARAM_CURRENT->rx_data_rate,
                                               board_delay_ms, RX_SESSION_PARAM_CURRENT->rx_window_symb,
                                               &rx_offset_ms_tmp );
        rp_task.start_time_ms = RX_SESSION_PARAM_CURRENT->ping_slot_parameters.ping_offset_time + rx_offset_ms_tmp +
                                ( RX_BEACON_TIMESTAMP_ERROR >> 1 );
        rp_task.start_time_100us = RX_SESSION_PARAM_CURRENT->ping_slot_parameters.ping_offset_time_100us +
                                   rx_offset_ms_tmp * 10 + ( RX_BEACON_TIMESTAMP_ERROR >> 1 ) * 10;

        rp_task.duration_time_ms = smtc_ping_slot_get_duration_timeout_ms(
            ping_slot_obj, RX_SESSION_PARAM_CURRENT->rx_window_symb, RX_SESSION_PARAM_CURRENT->rx_data_rate );

        if( rp_radio_params.pkt_type == RAL_PKT_TYPE_LORA )
        {
            rp_task.type                  = RP_TASK_TYPE_RX_LORA;
            rp_task.launch_task_callbacks = ping_slot_mac_rx_lora_launch_callback_for_rp;
        }
        else
        {
            rp_task.type                  = RP_TASK_TYPE_RX_FSK;
            rp_task.launch_task_callbacks = lr1_stack_mac_rx_gfsk_launch_callback_for_rp;
        }
        rp_status = rp_task_enqueue( ping_slot_obj->rp, &rp_task, ping_slot_obj->rx_payload, 255, &rp_radio_params );
    } while( rp_status == RP_TASK_STATUS_SCHEDULE_TASK_IN_PAST );
    if( rp_status != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "ping_slot_obj START ERROR \n" );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "ping_slot_obj devaddr:%x START at %d, freq:%u, dr:%d, PingNb:%d\n",
                                     RX_SESSION_PARAM_CURRENT->dev_addr,
                                     RX_SESSION_PARAM_CURRENT->ping_slot_parameters.ping_offset_time, ping_slot_freq,
                                     ping_slot_dr, RX_SESSION_PARAM_CURRENT->ping_slot_parameters.ping_number );
        if( ping_slot_obj->d2d_callback != NULL )
        {
            ping_slot_obj->d2d_callback( ping_slot_obj->d2d_context );
        }
    }

    ping_slot_obj->rx_metadata.rx_window = RECEIVE_NONE;
}

void smtc_ping_slot_rp_callback( smtc_ping_slot_t* ping_slot_obj )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "%s\n", __func__ );

    rp_status_t rp_status = ping_slot_obj->rp->status[ping_slot_obj->ping_slot_id4rp];
    if( rp_status == RP_STATUS_RX_PACKET )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "--> RP_STATUS_RX_PACKET\n" );
        ping_slot_obj->rx_callback( ping_slot_obj->rx_context );
    }
    else
    {
        // have to burn the current ping slot and the potential collide ping slot (could appear if an other task with
        // higher priority is enqueued in the radioplanner, in this case ping slot is still in the future but aborted by
        // the rp)
        if( rp_status == RP_STATUS_TASK_ABORTED )
        {
            smtc_ping_slot_compute_next_ping_offset_time(
                ping_slot_obj,
                RX_SESSION_PARAM_CURRENT->ping_slot_parameters.ping_offset_time +
                    smtc_ping_slot_get_duration_timeout_ms( ping_slot_obj, RX_SESSION_PARAM_CURRENT->rx_window_symb,
                                                            RX_SESSION_PARAM_CURRENT->rx_data_rate ) );
        }
        SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "--> %d\n", rp_status );
    }

    if( ping_slot_obj->enabled == true )
    {
        smtc_ping_slot_start( ping_slot_obj );
    }
}

void smtc_ping_slot_mac_rp_callback( smtc_ping_slot_t* ping_slot_obj )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "%s\n", __func__ );

    uint32_t tcurrent_ms;
    uint8_t  from_hook_id;

    rp_hook_get_id( ping_slot_obj->rp, ping_slot_obj, &from_hook_id );
    rp_get_status( ping_slot_obj->rp, from_hook_id, &tcurrent_ms, &( ping_slot_obj->planner_status ) );

    switch( ping_slot_obj->planner_status )
    {
    case RP_STATUS_TX_DONE:
        break;

    case RP_STATUS_RX_PACKET: {
        int status;
        ping_slot_obj->last_toa = 0;

        // save rssi and snr
        ping_slot_obj->rx_metadata.timestamp = tcurrent_ms;
        ping_slot_obj->rx_metadata.rx_snr =
            ping_slot_obj->rp->radio_params[from_hook_id].rx.lora_pkt_status.snr_pkt_in_db;
        ping_slot_obj->rx_metadata.rx_rssi =
            ping_slot_obj->rp->radio_params[from_hook_id].rx.lora_pkt_status.rssi_pkt_in_dbm;
        ping_slot_obj->rx_payload_size = ( uint8_t ) ping_slot_obj->rp->payload_size[from_hook_id];

        SMTC_MODEM_HAL_TRACE_PRINTF( "payload size receive = %u, snr = %d , rssi = %d\n",
                                     ping_slot_obj->rx_payload_size,
                                     ping_slot_obj->rp->radio_params[from_hook_id].rx.lora_pkt_status.snr_pkt_in_db,
                                     ping_slot_obj->rp->radio_params[from_hook_id].rx.lora_pkt_status.rssi_pkt_in_dbm );

        SMTC_MODEM_HAL_TRACE_ARRAY( "RxB Payload", ping_slot_obj->rx_payload, ping_slot_obj->rx_payload_size );

        status = smtc_ping_slot_mac_downlink_check( ping_slot_obj );

        if( status == OKLORAWAN )
        {
            ping_slot_obj->valid_rx_packet = smtc_ping_slot_mac_rx_frame_decode( ping_slot_obj );

            SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "Receive a downlink RXB for Hook Id = %d\n", from_hook_id );

            if( ping_slot_obj->valid_rx_packet == USER_RX_PACKET )
            {
                SMTC_MODEM_HAL_TRACE_ARRAY( "RxB app Payload", ping_slot_obj->rx_payload,
                                            ping_slot_obj->rx_payload_size );

                ping_slot_obj->last_toa = smtc_ping_slot_compute_downlink_toa(
                    ping_slot_obj->lr1_mac, RX_SESSION_PARAM_CURRENT->rx_data_rate,
                    ping_slot_obj->rp->payload_size[ping_slot_obj->ping_slot_id4rp] );

                ping_slot_obj->rx_metadata.rx_datarate     = RX_SESSION_PARAM_CURRENT->rx_data_rate;
                ping_slot_obj->rx_metadata.rx_frequency_hz = RX_SESSION_PARAM_CURRENT->rx_frequency;
                ping_slot_obj->rx_metadata.rx_window = RECEIVE_ON_RXB + ( uint8_t ) ping_slot_obj->rx_session_index;

                ping_slot_obj->push_callback( ping_slot_obj->push_context );
            }
        }
        ping_slot_obj->valid_rx_packet = NO_MORE_VALID_RX_PACKET;

        break;
    }
    case RP_STATUS_RX_CRC_ERROR:
        SMTC_MODEM_HAL_TRACE_PRINTF( "lr1mac RxB CRC ERROR\n" );
        break;

    case RP_STATUS_RX_TIMEOUT:
        SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "lr1mac RxB Timeout \n" );
        break;
    case RP_STATUS_TASK_ABORTED:
        SMTC_MODEM_HAL_TRACE_PRINTF( "lr1mac RxB aborted by the radioplanner \n" );
        break;
    default:
        SMTC_MODEM_HAL_TRACE_PRINTF( "lr1mac RxB receive It RADIO error %u\n", ping_slot_obj->planner_status );
        break;
    }
}

uint32_t smtc_ping_slot_compute_first_slot( uint32_t beacon_time_received_100us, uint32_t beacon_reserved_ms,
                                            uint32_t beacon_epoch_time, uint32_t dev_addr, uint16_t ping_period )
{
    uint8_t rand[16] = { 0 };
    if( smtc_modem_crypto_get_class_b_rand( beacon_epoch_time, dev_addr, rand ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        smtc_modem_hal_lr1mac_panic(
            "Crypto error while getting class B rand number for ping slot offset computation" );
    }

    uint32_t ret = ( beacon_time_received_100us + 10 * beacon_reserved_ms +
                     ( ( rand[0] + ( rand[1] << 8 ) ) % ping_period ) * 300 );
    return ret;
}

#if defined( SMTC_MULTICAST )
smtc_multicast_config_rc_t smtc_ping_slot_multicast_b_start_session( smtc_ping_slot_t* ping_slot_obj,
                                                                     uint8_t mc_group_id, uint32_t freq, uint8_t dr,
                                                                     uint8_t ping_slot_periodicity )
{
    // Class B must be running to start multicast
    if( ping_slot_obj->enabled == false )
    {
        return SMTC_MC_RC_ERROR_CLASS_NOT_ENABLED;
    }

    // Check if multicast group id is in acceptable range
    if( mc_group_id > ( LR1MAC_MC_NUMBER_OF_SESSION - 1 ) )
    {
        return SMTC_MC_RC_ERROR_BAD_ID;
    }

    // check if there is an ongoing enabled or a wait for beacon multicast session on this group_id
    if( ( ping_slot_obj->rx_session_param[mc_group_id + 1]->enabled == true ) ||
        ( ping_slot_obj->rx_session_param[mc_group_id + 1]->waiting_beacon_to_start == true ) )
    {
        return SMTC_MC_RC_ERROR_BUSY;
    }

    // Check if freq = 0 is acceptable
    if( ( freq == 0 ) && !smtc_real_is_beacon_hopping( ping_slot_obj->lr1_mac->real ) )
    {
        // freq = 0 is only acceptable in regions in which beacon is freq hopping
        return SMTC_MC_RC_ERROR_PARAM;
    }

    // Check if non null frequency is acceptable
    if( ( freq != 0 ) && ( smtc_real_is_frequency_valid( ping_slot_obj->lr1_mac->real, freq ) != OKLORAWAN ) )
    {
        return SMTC_MC_RC_ERROR_PARAM;
    }

    // Check if datarate is acceptable
    if( ( smtc_real_is_rx_dr_valid( ping_slot_obj->lr1_mac->real, dr ) != OKLORAWAN ) )
    {
        return SMTC_MC_RC_ERROR_PARAM;
    }

    // Save param (for first session or compatible with already enabled session )
    ping_slot_obj->rx_session_param[mc_group_id + 1]->rx_frequency          = freq;
    ping_slot_obj->rx_session_param[mc_group_id + 1]->rx_data_rate          = dr;
    ping_slot_obj->rx_session_param[mc_group_id + 1]->ping_slot_periodicity = ping_slot_periodicity;

    // Reset session fcntdown counter
    ping_slot_obj->rx_session_param[mc_group_id + 1]->fcnt_dwn = ~0;

    // Set the enable bit to true to activate the session
    ping_slot_obj->rx_session_param[mc_group_id + 1]->waiting_beacon_to_start = true;

    return SMTC_MC_RC_OK;
}

smtc_multicast_config_rc_t smtc_ping_slot_multicast_b_stop_session( smtc_ping_slot_t* ping_slot_obj,
                                                                    uint8_t           mc_group_id )
{
    // Check if multicast group id is in acceptable range
    if( mc_group_id > ( LR1MAC_MC_NUMBER_OF_SESSION - 1 ) )
    {
        return SMTC_MC_RC_ERROR_BAD_ID;
    }

    // Set the enable bit to false to indicate that the session is stopped
    ping_slot_obj->rx_session_param[mc_group_id + 1]->enabled                 = false;
    ping_slot_obj->rx_session_param[mc_group_id + 1]->waiting_beacon_to_start = false;

    if( ping_slot_obj->rx_session_index == ( mc_group_id + 1 ) )
    {
        rp_task_abort( ping_slot_obj->rp, ping_slot_obj->ping_slot_id4rp );
    }

    // Reset frequency and datarate to their not init values
    ping_slot_obj->rx_session_param[mc_group_id + 1]->rx_frequency = 0;
    ping_slot_obj->rx_session_param[mc_group_id + 1]->rx_data_rate = LR1MAC_MC_NO_DATARATE;

    return SMTC_MC_RC_OK;
}

smtc_multicast_config_rc_t smtc_ping_slot_multicast_b_stop_all_sessions( smtc_ping_slot_t* ping_slot_obj )
{
    smtc_multicast_config_rc_t status = SMTC_MC_RC_OK;

    for( uint8_t i = 0; i < LR1MAC_MC_NUMBER_OF_SESSION; i++ )
    {
        status = smtc_ping_slot_multicast_b_stop_session( ping_slot_obj, i );
        if( status != SMTC_MC_RC_OK )
        {
            break;
        }
    }

    return status;
}

smtc_multicast_config_rc_t smtc_ping_slot_multicast_b_get_session_status( smtc_ping_slot_t* ping_slot_obj,
                                                                          uint8_t mc_group_id, bool* is_session_started,
                                                                          bool* waiting_beacon_to_start, uint32_t* freq,
                                                                          uint8_t* dr, uint8_t* ping_slot_periodicity )
{
    // Check if multicast group id is in acceptable range
    if( mc_group_id > ( LR1MAC_MC_NUMBER_OF_SESSION - 1 ) )
    {
        return SMTC_MC_RC_ERROR_BAD_ID;
    }

    *is_session_started      = ping_slot_obj->rx_session_param[mc_group_id + 1]->enabled;
    *waiting_beacon_to_start = ping_slot_obj->rx_session_param[mc_group_id + 1]->waiting_beacon_to_start;
    *freq                    = ping_slot_obj->rx_session_param[mc_group_id + 1]->rx_frequency;
    *dr                      = ping_slot_obj->rx_session_param[mc_group_id + 1]->rx_data_rate;
    *ping_slot_periodicity   = ping_slot_obj->rx_session_param[mc_group_id + 1]->ping_slot_periodicity;

    return SMTC_MC_RC_OK;
}
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static int smtc_ping_slot_mac_downlink_check( smtc_ping_slot_t* ping_slot_obj )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "%s\n", __func__ );
    int status = OKLORAWAN;

    // check Mtype
    uint8_t rx_ftype_tmp = ping_slot_obj->rx_payload[0] >> 5;
    if( ( rx_ftype_tmp == JOIN_REQUEST ) || ( rx_ftype_tmp == JOIN_ACCEPT ) || ( rx_ftype_tmp == UNCONF_DATA_UP ) ||
        ( rx_ftype_tmp == CONF_DATA_UP ) || ( rx_ftype_tmp == REJOIN_REQUEST ) || ( rx_ftype_tmp == PROPRIETARY ) )
    {
        status += ERRORLORAWAN;
        SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " BAD Ftype = %u for RX Frame \n", rx_ftype_tmp );
    }

    // check devaddr
    if( ( ping_slot_obj->lr1_mac->join_status == JOINED ) && ( status == OKLORAWAN ) )
    {
        uint32_t dev_addr_tmp = ping_slot_obj->rx_payload[1] + ( ping_slot_obj->rx_payload[2] << 8 ) +
                                ( ping_slot_obj->rx_payload[3] << 16 ) + ( ping_slot_obj->rx_payload[4] << 24 );

        if( RX_SESSION_PARAM_CURRENT->dev_addr != dev_addr_tmp )
        {
            status += ERRORLORAWAN;
            SMTC_MODEM_HAL_TRACE_INFO( " BAD DevAddr = %x for RX Frame and %x\n\n", RX_SESSION_PARAM_CURRENT->dev_addr,
                                       dev_addr_tmp );
        }
    }
    else
    {
        ping_slot_obj->rx_session_index = RX_SESSION_COUNT;
    }

    if( status != OKLORAWAN )
    {
        ping_slot_obj->rx_payload_size = 0;
    }

    return ( status );
}

static void smtc_ping_slot_compute_next_ping_offset_time( smtc_ping_slot_t* ping_slot_obj, uint32_t timestamp )
{
    // Compute the new ping slot for each session
    for( rx_session_type_t i = 0; i < LR1MAC_NUMBER_OF_CLASS_B_SESSION; i++ )
    {
        if( ping_slot_obj->rx_session_param[i]->enabled == true )
        {
            // While ( (ping slot in past) AND (ping_number > 0)
            while( ( ( int32_t )( ping_slot_obj->rx_session_param[i]->ping_slot_parameters.ping_offset_time -
                                  timestamp ) <= 0 ) &&
                   ( ping_slot_obj->rx_session_param[i]->ping_slot_parameters.ping_number > 0 ) )
            {
                ping_slot_obj->rx_session_param[i]->ping_slot_parameters.ping_number--;
                ping_slot_obj->rx_session_param[i]->ping_slot_parameters.ping_offset_time +=
                    ( ping_slot_obj->rx_session_param[i]->ping_slot_parameters.ping_period * 30 );
                ping_slot_obj->rx_session_param[i]->ping_slot_parameters.ping_offset_time_100us +=
                    ( ping_slot_obj->rx_session_param[i]->ping_slot_parameters.ping_period * 300 );
            }
        }
    }
}

static void smtc_ping_slot_search_closest_ping_offset_time( smtc_ping_slot_t* ping_slot_obj, uint32_t timestamp_rtc )
{
    ping_slot_obj->rx_session_index = RX_SESSION_COUNT;

    // Init with a first ping offset in future, else we will never found the next closest offset in future
    for( rx_session_type_t i = 0; i < LR1MAC_NUMBER_OF_CLASS_B_SESSION; i++ )
    {
        if( RX_SESSION_PARAM[i]->enabled == true )
        {
            if( ( int32_t )( ping_slot_obj->rx_session_param[i]->ping_slot_parameters.ping_offset_time -
                             ( ping_slot_obj->next_beacon_timestamp - ping_slot_obj->beacon_guard_ms ) ) <= 0 )
            {
                if( ( ( int32_t )( RX_SESSION_PARAM[i]->ping_slot_parameters.ping_offset_time - timestamp_rtc ) > 0 ) )
                {
                    ping_slot_obj->rx_session_index = i;

                    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "Ping Slot session %d enabled", i );
                    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "--> offset %u, init\n",
                                                       RX_SESSION_PARAM[i]->ping_slot_parameters.ping_offset_time );
                    break;
                }
            }
        }
    }

    // No more ping slot available
    if( ping_slot_obj->rx_session_index == RX_SESSION_COUNT )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " No more ping slot available \n" );
        return;
    }

    // Search the next closest ping slot
    for( rx_session_type_t i = ping_slot_obj->rx_session_index; i < LR1MAC_NUMBER_OF_CLASS_B_SESSION; i++ )
    {
        if( ping_slot_obj->rx_session_index == i )
        {
            continue;
        }
        // Search Ping Slot for each enabled session
        if( RX_SESSION_PARAM[i]->enabled == true )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "Ping Slot session %d enabled", i );
            SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "--> offset %u, ",
                                               RX_SESSION_PARAM[i]->ping_slot_parameters.ping_offset_time );

            // Ignore session if all ping number were used
            if( ( ( int32_t )( RX_SESSION_PARAM[i]->ping_slot_parameters.ping_offset_time - timestamp_rtc ) < 0 ) &&
                ( RX_SESSION_PARAM[i]->ping_slot_parameters.ping_number == 0 ) )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " no more ping slot for session %d\n", i );
                continue;
            }
            if( ( int32_t )( ping_slot_obj->rx_session_param[i]->ping_slot_parameters.ping_offset_time -
                             ( ping_slot_obj->next_beacon_timestamp - ping_slot_obj->beacon_guard_ms ) ) >= 0 )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " no more ping slot for session (guard) %d\n", i );
                continue;
            }

            // t1 = t0
            if( ( int32_t )( RX_SESSION_PARAM[i]->ping_slot_parameters.ping_offset_time -
                             RX_SESSION_PARAM_CURRENT->ping_slot_parameters.ping_offset_time ) == 0 )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "!!!Ping Slot collision t1 = t0 !!!!\n" );

                // The new ping slot has more priority
                if( RX_SESSION_PARAM_CURRENT->fpending_bit < RX_SESSION_PARAM[i]->fpending_bit )
                {
                    ping_slot_obj->rx_session_index = i;
                }
                // Priority is the same, DevAddr SHALL take priority
                else if( RX_SESSION_PARAM_CURRENT->fpending_bit == RX_SESSION_PARAM[i]->fpending_bit )
                {
                    if( RX_SESSION_PARAM_CURRENT->dev_addr < RX_SESSION_PARAM[i]->dev_addr )
                    {
                        SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " next time %u ",
                                                           RX_SESSION_PARAM[i]->ping_slot_parameters.ping_offset_time );

                        ping_slot_obj->rx_session_index = i;
                    }
                }
            }
            // t1 < t0
            else if( ( int32_t )( RX_SESSION_PARAM[i]->ping_slot_parameters.ping_offset_time -
                                  RX_SESSION_PARAM_CURRENT->ping_slot_parameters.ping_offset_time ) < 0 )
            {
                // ( t1 + delay1 ) < t0
                if( ( int32_t )( RX_SESSION_PARAM[i]->ping_slot_parameters.ping_offset_time +
                                 smtc_ping_slot_get_duration_timeout_ms( ping_slot_obj,
                                                                         RX_SESSION_PARAM[i]->rx_window_symb,
                                                                         RX_SESSION_PARAM[i]->rx_data_rate ) -
                                 RX_SESSION_PARAM_CURRENT->ping_slot_parameters.ping_offset_time ) < 0 )
                {
                    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " first time %u",
                                                       RX_SESSION_PARAM[i]->ping_slot_parameters.ping_offset_time );

                    ping_slot_obj->rx_session_index = i;
                }
                else  // collision: t1 will end after t0 start
                {
                    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "!!!Ping Slot collision t1/t0 !!!!\n" );
                    // The new ping slot is more priority
                    if( RX_SESSION_PARAM_CURRENT->fpending_bit < RX_SESSION_PARAM[i]->fpending_bit )
                    {
                        ping_slot_obj->rx_session_index = i;
                    }
                    // Priority is the same, DevAddr SHALL take priority
                    else if( RX_SESSION_PARAM_CURRENT->fpending_bit == RX_SESSION_PARAM[i]->fpending_bit )
                    {
                        if( RX_SESSION_PARAM_CURRENT->dev_addr < RX_SESSION_PARAM[i]->dev_addr )
                        {
                            SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG(
                                " next time %u ", RX_SESSION_PARAM[i]->ping_slot_parameters.ping_offset_time );

                            ping_slot_obj->rx_session_index = i;
                        }
                    }
                }
            }
            else  // t0 < t1
            {
                // ( t0 + delay0 ) < t1
                if( ( int32_t )( RX_SESSION_PARAM_CURRENT->ping_slot_parameters.ping_offset_time +
                                 smtc_ping_slot_get_duration_timeout_ms( ping_slot_obj,
                                                                         RX_SESSION_PARAM_CURRENT->rx_window_symb,
                                                                         RX_SESSION_PARAM_CURRENT->rx_data_rate ) -
                                 RX_SESSION_PARAM[i]->ping_slot_parameters.ping_offset_time ) < 0 )
                {
                    // Keep t0
                }
                else  // collision: t0 will end after t1 start
                {
                    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "!!!Ping Slot collision t0/t1 !!!!\n" );
                    // The new ping slot has more priority
                    if( RX_SESSION_PARAM_CURRENT->fpending_bit < RX_SESSION_PARAM[i]->fpending_bit )
                    {
                        ping_slot_obj->rx_session_index = i;
                    }
                    // Priority is the same, DevAddr SHALL take priority
                    else if( RX_SESSION_PARAM_CURRENT->fpending_bit == RX_SESSION_PARAM[i]->fpending_bit )
                    {
                        if( RX_SESSION_PARAM_CURRENT->dev_addr < RX_SESSION_PARAM[i]->dev_addr )
                        {
                            SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG(
                                " next time %u ", RX_SESSION_PARAM[i]->ping_slot_parameters.ping_offset_time );

                            ping_slot_obj->rx_session_index = i;
                        }
                    }
                }
            }
            SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "\n" );
        }
    }
}

static rx_packet_type_t smtc_ping_slot_mac_rx_frame_decode( smtc_ping_slot_t* ping_slot_obj )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "%s\n", __func__ );
    int              status         = OKLORAWAN;
    rx_packet_type_t rx_packet_type = NO_MORE_VALID_RX_PACKET;
    uint32_t         mic_in;
    uint8_t          rx_ftype;
    uint8_t          rx_major;

    status += lr1mac_rx_payload_min_size_check( ping_slot_obj->rx_payload_size );
    status += lr1mac_rx_payload_max_size_check( ping_slot_obj->lr1_mac, ping_slot_obj->rx_payload_size,
                                                RX_SESSION_PARAM_CURRENT->rx_data_rate );
    if( status != OKLORAWAN )
    {
        return NO_MORE_VALID_RX_PACKET;
    }

    status += lr1mac_rx_mhdr_extract( ping_slot_obj->rx_payload, &rx_ftype, &rx_major, &ping_slot_obj->tx_ack_bit );
    if( status != OKLORAWAN )
    {
        return NO_MORE_VALID_RX_PACKET;
    }

    if( ping_slot_obj->rx_session_index != RX_SESSION_UNICAST )
    {
        if( ( ping_slot_obj->tx_ack_bit == true ) || ( rx_ftype == CONF_DATA_UP ) )
        {
            return NO_MORE_VALID_RX_PACKET;
        }
    }

    /************************************************************************/
    /*               Case : the receive packet is not a JoinResponse */
    /************************************************************************/

    // Read Fcntdown in lr1mac in case of the downlink came from unicast device address
    ping_slot_obj->rx_session_param[RX_SESSION_UNICAST]->fcnt_dwn = ping_slot_obj->lr1_mac->fcnt_dwn;

    uint16_t fcnt_dwn_tmp       = 0;
    uint32_t fcnt_dwn_stack_tmp = RX_SESSION_PARAM_CURRENT->fcnt_dwn;

    status += lr1mac_rx_fhdr_extract(
        ping_slot_obj->rx_payload, ping_slot_obj->rx_payload_size, &( ping_slot_obj->rx_fopts_length ), &fcnt_dwn_tmp,
        RX_SESSION_PARAM_CURRENT->dev_addr, &( ping_slot_obj->rx_metadata.rx_fport ),
        &( ping_slot_obj->rx_payload_empty ), &( ping_slot_obj->rx_fctrl ), ping_slot_obj->rx_fopts );

    if( status == OKLORAWAN )
    {
        status = lr1mac_fcnt_dwn_accept( fcnt_dwn_tmp, &fcnt_dwn_stack_tmp );
    }
    if( status == OKLORAWAN )
    {
        ping_slot_obj->rx_payload_size = ping_slot_obj->rx_payload_size - MICSIZE;
        memcpy1( ( uint8_t* ) &mic_in, &ping_slot_obj->rx_payload[ping_slot_obj->rx_payload_size], MICSIZE );

        if( smtc_modem_crypto_verify_mic( &ping_slot_obj->rx_payload[0], ping_slot_obj->rx_payload_size,
                                          RX_SESSION_PARAM_CURRENT->nwk_skey, RX_SESSION_PARAM_CURRENT->dev_addr, 1,
                                          fcnt_dwn_stack_tmp, mic_in ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
        {
            if( ping_slot_obj->d2d_check_fcnt_down_callback != NULL )
            {
                status =
                    ping_slot_obj->d2d_check_fcnt_down_callback( ( void* ) ping_slot_obj, &fcnt_dwn_stack_tmp, mic_in );
            }
            else
            {
                status = ERRORLORAWAN;
            }
        }
    }
    if( status == OKLORAWAN )
    {
        RX_SESSION_PARAM_CURRENT->fcnt_dwn = fcnt_dwn_stack_tmp;
        SMTC_MODEM_HAL_TRACE_WARNING( " fcnt_tmp = %d\n ", RX_SESSION_PARAM_CURRENT->fcnt_dwn );
        ping_slot_obj->lr1_mac->fcnt_dwn = ping_slot_obj->rx_session_param[RX_SESSION_UNICAST]->fcnt_dwn;

        // Set FPending bit in metadata
        ping_slot_obj->rx_metadata.rx_fpending_bit = ( ping_slot_obj->rx_fctrl >> DL_FPENDING_BIT ) & 0x01;

        // Find current ping slot group and set the fpending prioritization
        if( ping_slot_obj->rx_session_index == RX_SESSION_UNICAST )
        {
            if( ping_slot_obj->rx_metadata.rx_fpending_bit == true )
            {
                RX_SESSION_PARAM_CURRENT->fpending_bit = UNICAST_FPENDING;
            }
            else
            {
                RX_SESSION_PARAM_CURRENT->fpending_bit = UNICAST_WO_FPENDING;
            }
        }
        else
        {
            if( ping_slot_obj->rx_metadata.rx_fpending_bit == true )
            {
                RX_SESSION_PARAM_CURRENT->fpending_bit = MULTICAST_FPENDING;
            }
            else
            {
                RX_SESSION_PARAM_CURRENT->fpending_bit = MULTICAST_WO_FPENDING;
            }
        }

        if( ping_slot_obj->rx_payload_empty == 0 )  // rx payload not empty
        {
            ping_slot_obj->rx_payload_size =
                ping_slot_obj->rx_payload_size - FHDROFFSET - 1 - ping_slot_obj->rx_fopts_length;

            if( ping_slot_obj->rx_metadata.rx_fport == 0 )
            {  // receive a mac management frame Fport 0

                SMTC_MODEM_HAL_TRACE_WARNING( " Receive an not valid packet RxB on port zero\n" );
            }
            else
            {
                if( smtc_modem_crypto_payload_decrypt(
                        &ping_slot_obj->rx_payload[FHDROFFSET + 1 + ping_slot_obj->rx_fopts_length],
                        ping_slot_obj->rx_payload_size, RX_SESSION_PARAM_CURRENT->app_skey,
                        RX_SESSION_PARAM_CURRENT->dev_addr, 1, RX_SESSION_PARAM_CURRENT->fcnt_dwn,
                        &ping_slot_obj->rx_payload[0] ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
                {
                    smtc_modem_hal_lr1mac_panic( "Crypto error during payload decryption\n" );
                }
                if( ping_slot_obj->rx_fopts_length != 0 )
                {
                    SMTC_MODEM_HAL_TRACE_WARNING( " Receive an not valid packet RxB FOpts\n" );
                    status = ERRORLORAWAN;
                }
                else
                {
                    rx_packet_type                      = USER_RX_PACKET;
                    ping_slot_obj->available_app_packet = LORA_RX_PACKET_AVAILABLE;
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
            if( ping_slot_obj->rx_fopts_length != 0 )
            {
                SMTC_MODEM_HAL_TRACE_WARNING( " Receive an not valid packet RxB FOpts\n" );
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
        ping_slot_obj->rx_ftype = rx_ftype;
        ping_slot_obj->rx_major = rx_major;

        // We check the duty cycle before set the ACKbit in stack for the next uplink
        // because we have only 8s to answer the Confirmed Downlink
        if( lr1mac_core_next_free_duty_cycle_ms_get( ping_slot_obj->lr1_mac ) <= 0 )
        {
            ping_slot_obj->lr1_mac->tx_ack_bit = ping_slot_obj->tx_ack_bit;
        }
    }

    SMTC_MODEM_HAL_TRACE_PRINTF( " RxB rx_packet_type = %d \n", rx_packet_type );

    return ( rx_packet_type );
}

static uint32_t smtc_ping_slot_get_duration_timeout_ms( smtc_ping_slot_t* ping_slot_obj, uint16_t nb_rx_window_symb,
                                                        uint8_t datarate )
{
    uint32_t time_symb_ms =
        ( nb_rx_window_symb * smtc_real_get_symbol_duration_us( ping_slot_obj->lr1_mac->real, datarate ) ) / 1000;
    return ( ( time_symb_ms < 1 ) ? 1 : time_symb_ms );
}

static uint32_t smtc_ping_slot_compute_downlink_toa( lr1_stack_mac_t* lr1_mac, uint8_t datarate, uint8_t payload_size )
{
    uint32_t toa = 0;

    modulation_type_t modulation_type = smtc_real_get_modulation_type_from_datarate( lr1_mac->real, datarate );

    if( modulation_type == LORA )
    {
        uint8_t            sf;
        lr1mac_bandwidth_t bw;
        smtc_real_lora_dr_to_sf_bw( lr1_mac->real, datarate, &sf, &bw );

        ralf_params_lora_t lora_param;
        memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );

        lora_param.mod_params.sf = ( ral_lora_sf_t ) sf;
        lora_param.mod_params.bw = ( ral_lora_bw_t ) bw;
        lora_param.mod_params.cr = smtc_real_get_coding_rate( lr1_mac->real );
        lora_param.pkt_params.preamble_len_in_symb =
            smtc_real_get_preamble_len( lr1_mac->real, lora_param.mod_params.sf );
        lora_param.pkt_params.header_type      = RAL_LORA_PKT_EXPLICIT;
        lora_param.pkt_params.pld_len_in_bytes = payload_size;
        lora_param.pkt_params.crc_is_on        = false;
        lora_param.mod_params.ldro = ral_compute_lora_ldro( lora_param.mod_params.sf, lora_param.mod_params.bw );

        toa = ral_get_lora_time_on_air_in_ms( ( &lr1_mac->rp->radio->ral ), ( &lora_param.pkt_params ),
                                              ( &lora_param.mod_params ) );
    }
    else if( modulation_type == FSK )
    {
        uint8_t bitrate;
        smtc_real_fsk_dr_to_bitrate( lr1_mac->real, datarate, &bitrate );

        ralf_params_gfsk_t gfsk_param;
        memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

        gfsk_param.mod_params.br_in_bps            = bitrate * 1000;
        gfsk_param.pkt_params.preamble_len_in_bits = 40;
        gfsk_param.pkt_params.header_type          = RAL_GFSK_PKT_VAR_LEN;
        gfsk_param.pkt_params.pld_len_in_bytes     = payload_size;
        gfsk_param.pkt_params.crc_type             = RAL_GFSK_CRC_2_BYTES_INV;

        toa = ral_get_gfsk_time_on_air_in_ms( ( &lr1_mac->rp->radio->ral ), ( &gfsk_param.pkt_params ),
                                              ( &gfsk_param.mod_params ) );
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( "TX MODULATION NOT SUPPORTED\n" );
    }

    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "Toa = %d\n", toa );
    return toa;
}

static void ping_slot_mac_rx_lora_launch_callback_for_rp( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;
    smtc_modem_hal_assert( ralf_setup_lora( rp->radio, &rp->radio_params[id].rx.lora ) == RAL_STATUS_OK );
    smtc_modem_hal_assert( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT |
                                                                            RAL_IRQ_RX_HDR_ERROR |
                                                                            RAL_IRQ_RX_CRC_ERROR ) == RAL_STATUS_OK );
    // Wait the exact time
    while( ( int32_t )( rp->tasks[id].start_time_100us - smtc_modem_hal_get_time_in_100us( ) ) > 0 )
    {
    }
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_assert( ral_set_rx( &( rp->radio->ral ), rp->radio_params[id].rx.timeout_in_ms ) == RAL_STATUS_OK );
    rp_stats_set_rx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
}
/* --- EOF ------------------------------------------------------------------ */
