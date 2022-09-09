/*!
 * \file      lr1_stack_mac_layer.c
 *
 * \brief     LoRaWan stack mac layer definition
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
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "lr1_stack_mac_layer.h"

#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_real.h"
#include "lr1mac_utilities.h"
#include "radio_planner.h"
#include "lorawan_api.h"
#include "smtc_modem_hal.h"
#include "lr1mac_config.h"
#include "smtc_modem_crypto.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 *-----------------------------------------------------------------------------------
 * --- PRIVATE MACROS ---------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
static const char* smtc_name_rx_windows[] = { "RX1", "RX2" };
static const char* smtc_name_bw[]         = { "BW007", "BW010", "BW015", "BW020", "BW031", "BW041", "BW062",
                                      "BW125", "BW200", "BW250", "BW400", "BW500", "BW800", "BW1600" };
#endif

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE VARIABLES -------------------------------------------------------------
 */

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTION DECLARATION --------------------------------------------------
 */

static status_lorawan_t lr1_stack_mac_downlink_check_under_it( lr1_stack_mac_t* lr1_mac );
static void             mac_header_set( lr1_stack_mac_t* lr1_mac );
static void             frame_header_set( lr1_stack_mac_t* lr1_mac );

static void             link_check_parser( lr1_stack_mac_t* lr1_mac );
static void             link_adr_parser( lr1_stack_mac_t* lr1_mac, uint8_t nb_link_adr_req );
static void             duty_cycle_parser( lr1_stack_mac_t* lr1_mac );
static void             rx_param_setup_parser( lr1_stack_mac_t* lr1_mac );
static void             dev_status_parser( lr1_stack_mac_t* lr1_mac );
static void             new_channel_parser( lr1_stack_mac_t* lr1_mac );
static void             rx_timing_setup_parser( lr1_stack_mac_t* lr1_mac );
static void             tx_param_setup_parser( lr1_stack_mac_t* lr1_mac );
static void             dl_channel_parser( lr1_stack_mac_t* lr1_mac );
static status_lorawan_t device_time_ans_parser( lr1_stack_mac_t* lr1_mac );
static void             beacon_freq_req_parser( lr1_stack_mac_t* lr1_mac );
static void             ping_slot_channel_req_parser( lr1_stack_mac_t* lr1_mac );
static status_lorawan_t ping_slot_info_ans_parser( lr1_stack_mac_t* lr1_mac );

/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTION DEFINITIONS ---------------------------------------------------
 */

void lr1_stack_mac_init( lr1_stack_mac_t* lr1_mac, lr1mac_activation_mode_t activation_mode,
                         smtc_real_region_types_t region )
{
    lr1_mac->tx_major_bits                            = LORAWANR1;
    lr1_mac->radio_process_state                      = RADIOSTATE_IDLE;
    lr1_mac->next_time_to_join_seconds                = 0;
    lr1_mac->join_status                              = NOT_JOINED;
    lr1_mac->type_of_ans_to_send                      = NOFRAME_TOSEND;
    lr1_mac->activation_mode                          = activation_mode;
    lr1_mac->nb_trans                                 = 1;
    lr1_mac->available_app_packet                     = NO_LORA_RXPACKET_AVAILABLE;
    lr1_mac->real->region_type                        = region;
    lr1_mac->is_lorawan_modem_certification_enabled   = false;
    lr1_mac->isr_tx_done_radio_timestamp              = 0;
    lr1_mac->dev_nonce                                = 0;
    lr1_mac->nb_of_reset                              = 0;
    lr1_mac->adr_mode_select                          = STATIC_ADR_MODE;
    lr1_mac->adr_mode_select_tmp                      = STATIC_ADR_MODE;
    lr1_mac->adr_custom[0]                            = BSP_USER_DR_DISTRIBUTION_PARAMETERS;
    lr1_mac->adr_custom[1]                            = 0;
    lr1_mac->current_win                              = RX1;
    lr1_mac->seconds_since_epoch                      = 0;
    lr1_mac->fractional_second                        = 0;
    lr1_mac->timestamp_last_device_time_ans_s         = 0;
    lr1_mac->timestamp_tx_done_device_time_req_ms     = 0;
    lr1_mac->timestamp_tx_done_device_time_req_ms_tmp = 0;
    lr1_mac->device_time_callback                     = NULL;
    lr1_mac->device_time_callback_context             = NULL;
    memset( lr1_mac->fine_tune_board_setting_delay_ms, 0, sizeof( lr1_mac->fine_tune_board_setting_delay_ms ) );
    memset( lr1_mac->join_nonce, 0xFF, sizeof( lr1_mac->join_nonce ) );

    lr1_stack_mac_session_init( lr1_mac );
}

void lr1_stack_mac_session_init( lr1_stack_mac_t* lr1_mac )
{
    lr1_mac->fcnt_dwn                          = ~0;
    lr1_mac->fcnt_up                           = 0;
    lr1_mac->retry_join_cpt                    = 0;
    lr1_mac->no_rx_packet_count_in_mobile_mode = 0;
    lr1_mac->no_rx_packet_count                = 0;
    lr1_mac->adr_ack_cnt                       = 0;
    lr1_mac->tx_fopts_current_length           = 0;
    lr1_mac->tx_fopts_length                   = 0;
    lr1_mac->tx_fopts_lengthsticky             = 0;
    lr1_mac->nwk_ans_size                      = 0;
    lr1_mac->nwk_payload_size                  = 0;
    lr1_mac->nwk_payload_index                 = 0;
    lr1_mac->max_duty_cycle_index              = 0;
    lr1_mac->tx_duty_cycle_time_off_ms         = 0;
    lr1_mac->tx_duty_cycle_timestamp_ms        = smtc_modem_hal_get_time_in_ms( );
    lr1_mac->available_link_adr                = false;
    lr1_mac->link_check_user_req               = USER_MAC_REQ_NOT_REQUESTED;
    lr1_mac->device_time_user_req              = USER_MAC_REQ_NOT_REQUESTED;
    lr1_mac->ping_slot_info_user_req           = USER_MAC_REQ_NOT_REQUESTED;
    lr1_mac->link_check_margin                 = 0;
    lr1_mac->link_check_gw_cnt                 = 0;
    lr1_mac->tx_ack_bit                        = 0;
    lr1_mac->tx_class_b_bit                    = 0;
}

/**************************************************************************************************/
/*                                  build lorawan frame */
/*                                  encrypt lorawan frame */
/*                       enqueue tx frame in radioplanner to proceed transmit */
/*                                                                                                */
/**************************************************************************************************/

void lr1_stack_mac_tx_frame_build( lr1_stack_mac_t* lr1_mac )
{
    uint8_t tx_fopts_length = 0;
    if( lr1_mac->tx_fport != PORTNWK )
    {
        tx_fopts_length = lr1_mac->tx_fopts_current_length;
    }

    lr1_mac->tx_fctrl = ( lr1_mac->adr_enable << 7 ) + ( lr1_mac->adr_ack_req << 6 ) + ( lr1_mac->tx_ack_bit << 5 ) +
                        ( lr1_mac->tx_class_b_bit << 4 ) + ( tx_fopts_length & 0x0F );
    lr1_mac->tx_ack_bit = 0;

    mac_header_set( lr1_mac );
    frame_header_set( lr1_mac );
    lr1_mac->tx_payload_size = lr1_mac->app_payload_size + FHDROFFSET + lr1_mac->tx_fport_present + tx_fopts_length;
}

void lr1_stack_mac_tx_frame_encrypt( lr1_stack_mac_t* lr1_mac )
{
    uint8_t tx_fopts_length = 0;
    if( lr1_mac->tx_fport != PORTNWK )
    {
        tx_fopts_length = lr1_mac->tx_fopts_current_length;
    }

    if( smtc_modem_crypto_payload_encrypt(
            &lr1_mac->tx_payload[FHDROFFSET + lr1_mac->tx_fport_present + tx_fopts_length], lr1_mac->app_payload_size,
            ( lr1_mac->tx_fport == PORTNWK ) ? SMTC_SE_NWK_S_ENC_KEY : SMTC_SE_APP_S_KEY, lr1_mac->dev_addr, UP_LINK,
            lr1_mac->fcnt_up, &lr1_mac->tx_payload[FHDROFFSET + lr1_mac->tx_fport_present + tx_fopts_length] ) !=
        SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        smtc_modem_hal_lr1mac_panic( "Crypto error during payload encryption\n" );
    }

    if( smtc_modem_crypto_compute_and_add_mic( &lr1_mac->tx_payload[0], lr1_mac->tx_payload_size, SMTC_SE_NWK_S_ENC_KEY,
                                               lr1_mac->dev_addr, UP_LINK,
                                               lr1_mac->fcnt_up ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        smtc_modem_hal_lr1mac_panic( "Crypto error during mic computation\n" );
    }
    lr1_mac->tx_payload_size = lr1_mac->tx_payload_size + 4;
}
void lr1_stack_mac_tx_radio_free_lbt( lr1_stack_mac_t* lr1_mac )
{
    lr1_mac->radio_process_state = RADIOSTATE_TX_ON;
    lr1_mac->rtc_target_timer_ms = smtc_modem_hal_get_time_in_ms( ) + lr1_mac->rp->margin_delay;
    lr1_mac->send_at_time        = true;
    lr1_stack_mac_tx_radio_start( lr1_mac );
}
void lr1_stack_mac_radio_busy_lbt( lr1_stack_mac_t* lr1_mac )
{
    lr1_mac->radio_process_state = RADIOSTATE_IDLE;
    lr1_mac->rtc_target_timer_ms = smtc_modem_hal_get_time_in_ms( ) + lr1_mac->rp->margin_delay;
    smtc_real_get_next_channel( lr1_mac );
}
void lr1_stack_mac_radio_abort_lbt( lr1_stack_mac_t* lr1_mac )
{
    lr1_mac->radio_process_state = RADIOSTATE_ABORTED_BY_RP;
}
void lr1_stack_mac_tx_lora_launch_callback_for_rp( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;

    smtc_modem_hal_assert( ralf_setup_lora( rp->radio, &rp->radio_params[id].tx.lora ) == RAL_STATUS_OK );
    smtc_modem_hal_assert( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_TX_DONE ) == RAL_STATUS_OK );
    smtc_modem_hal_assert( ral_set_pkt_payload( &( rp->radio->ral ), rp->payload[id], rp->payload_size[id] ) ==
                           RAL_STATUS_OK );
    // Wait the exact expected time (ie target - tcxo startup delay)
    while( ( int32_t )( rp->tasks[id].start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) > 0 )
    {
        // Do nothing
    }
    // At this time only tcxo startup delay is remaining
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_assert( ral_set_tx( &( rp->radio->ral ) ) == RAL_STATUS_OK );
    rp_stats_set_tx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
}

void lr1_stack_mac_tx_gfsk_launch_callback_for_rp( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;

    smtc_modem_hal_assert( ralf_setup_gfsk( rp->radio, &rp->radio_params[id].tx.gfsk ) == RAL_STATUS_OK );
    smtc_modem_hal_assert( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_TX_DONE ) == RAL_STATUS_OK );
    smtc_modem_hal_assert( ral_set_pkt_payload( &( rp->radio->ral ), rp->payload[id], rp->payload_size[id] ) ==
                           RAL_STATUS_OK );
    // Wait the exact expected time (ie target - tcxo startup delay)
    while( ( int32_t )( rp->tasks[id].start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) > 0 )
    {
    }
    // At this time only tcxo startup delay is remaining
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_assert( ral_set_tx( &( rp->radio->ral ) ) == RAL_STATUS_OK );
    rp_stats_set_tx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
}

void lr1_stack_mac_tx_lr_fhss_launch_callback_for_rp( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;

    // Initialize LR-FHSS
    smtc_modem_hal_assert(
        ral_lr_fhss_init( &( rp->radio->ral ), &rp->radio_params[id].tx.lr_fhss.ral_lr_fhss_params ) == RAL_STATUS_OK );
    smtc_modem_hal_assert(
        ral_set_tx_cfg( &( rp->radio->ral ), rp->radio_params[id].tx.lr_fhss.output_pwr_in_dbm,
                        rp->radio_params[id].tx.lr_fhss.ral_lr_fhss_params.center_frequency_in_hz ) == RAL_STATUS_OK );
    smtc_modem_hal_assert( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_TX_DONE | RAL_IRQ_LR_FHSS_HOP ) ==
                           RAL_STATUS_OK );
    smtc_modem_hal_assert( ral_lr_fhss_build_frame( &( rp->radio->ral ),
                                                    &rp->radio_params[id].tx.lr_fhss.ral_lr_fhss_params,
                                                    ( ral_lr_fhss_memory_state_t ) rp->radio_params[id].lr_fhss_state,
                                                    rp->radio_params[id].tx.lr_fhss.hop_sequence_id, rp->payload[id],
                                                    rp->payload_size[id] ) == RAL_STATUS_OK );
    // Wait the exact expected time (ie target - tcxo startup delay)
    while( ( int32_t )( rp->tasks[id].start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) > 0 )
    {
        // Do nothing
    }
    // At this time only tcxo startup delay is remaining
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_assert( ral_set_tx( &( rp->radio->ral ) ) == RAL_STATUS_OK );
    rp_stats_set_tx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
}

void lr1_stack_mac_rx_lora_launch_callback_for_rp( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;

    smtc_modem_hal_assert( ralf_setup_lora( rp->radio, &rp->radio_params[id].rx.lora ) == RAL_STATUS_OK );
    smtc_modem_hal_assert( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT |
                                                                            RAL_IRQ_RX_HDR_ERROR |
                                                                            RAL_IRQ_RX_CRC_ERROR ) == RAL_STATUS_OK );
    // Wait the exact expected time (ie target - tcxo startup delay)
    while( ( int32_t )( rp->tasks[id].start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) > 0 )
    {
    }
    // At this time only tcxo startup delay is remaining
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_assert( ral_set_rx( &( rp->radio->ral ), rp->radio_params[id].rx.timeout_in_ms ) == RAL_STATUS_OK );
    rp_stats_set_rx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
}

void lr1_stack_mac_rx_gfsk_launch_callback_for_rp( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;

    smtc_modem_hal_assert( ralf_setup_gfsk( rp->radio, &rp->radio_params[id].rx.gfsk ) == RAL_STATUS_OK );
    smtc_modem_hal_assert( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT |
                                                                            RAL_IRQ_RX_CRC_ERROR ) == RAL_STATUS_OK );
    // Wait the exact expected time (ie target - tcxo startup delay)
    while( ( int32_t )( rp->tasks[id].start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) > 0 )
    {
    }
    // At this time only tcxo startup delay is remaining
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_assert( ral_set_rx( &( rp->radio->ral ), rp->radio_params[id].rx.timeout_in_ms ) == RAL_STATUS_OK );
    rp_stats_set_rx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
}

void lr1_stack_mac_tx_radio_start( lr1_stack_mac_t* lr1_mac )
{
    rp_radio_params_t radio_params = { 0 };
    rp_task_t         rp_task      = { 0 };
    uint32_t          toa          = 0;

    modulation_type_t tx_modulation_type =
        smtc_real_get_modulation_type_from_datarate( lr1_mac, lr1_mac->tx_data_rate );

    if( tx_modulation_type == LORA )
    {
        uint8_t            tx_sf;
        lr1mac_bandwidth_t tx_bw;
        smtc_real_lora_dr_to_sf_bw( lr1_mac, lr1_mac->tx_data_rate, &tx_sf, &tx_bw );

        ralf_params_lora_t lora_param;
        memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );

        lora_param.rf_freq_in_hz     = lr1_mac->tx_frequency;
        lora_param.sync_word         = smtc_real_get_sync_word( lr1_mac );
        lora_param.output_pwr_in_dbm = smtc_real_clamp_output_power_eirp_vs_freq_and_dr(
            lr1_mac, lr1_mac->tx_power, lr1_mac->tx_frequency, lr1_mac->tx_data_rate );

        lora_param.mod_params.sf   = ( ral_lora_sf_t ) tx_sf;
        lora_param.mod_params.bw   = ( ral_lora_bw_t ) tx_bw;
        lora_param.mod_params.cr   = smtc_real_get_coding_rate( lr1_mac );
        lora_param.mod_params.ldro = ral_compute_lora_ldro( lora_param.mod_params.sf, lora_param.mod_params.bw );

        lora_param.pkt_params.preamble_len_in_symb = smtc_real_get_preamble_len( lr1_mac, lora_param.mod_params.sf );
        lora_param.pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT;
        lora_param.pkt_params.pld_len_in_bytes     = lr1_mac->tx_payload_size;
        lora_param.pkt_params.crc_is_on            = true;
        lora_param.pkt_params.invert_iq_is_on      = false;

        radio_params.pkt_type = RAL_PKT_TYPE_LORA;
        radio_params.tx.lora  = lora_param;

        toa = ral_get_lora_time_on_air_in_ms( ( &lr1_mac->rp->radio->ral ), ( &lora_param.pkt_params ),
                                              ( &lora_param.mod_params ) );

        rp_task.type                  = RP_TASK_TYPE_TX_LORA;
        rp_task.launch_task_callbacks = lr1_stack_mac_tx_lora_launch_callback_for_rp;
    }
    else if( tx_modulation_type == FSK )
    {
        uint8_t tx_bitrate;
        smtc_real_fsk_dr_to_bitrate( lr1_mac, lr1_mac->tx_data_rate, &tx_bitrate );

        ralf_params_gfsk_t gfsk_param;
        memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

        gfsk_param.dc_free_is_on     = true;
        gfsk_param.whitening_seed    = GFSK_WHITENING_SEED;
        gfsk_param.crc_seed          = GFSK_CRC_SEED;
        gfsk_param.crc_polynomial    = GFSK_CRC_POLYNOMIAL;
        gfsk_param.rf_freq_in_hz     = lr1_mac->tx_frequency;
        gfsk_param.sync_word         = smtc_real_get_gfsk_sync_word( lr1_mac );
        gfsk_param.output_pwr_in_dbm = smtc_real_clamp_output_power_eirp_vs_freq_and_dr(
            lr1_mac, lr1_mac->tx_power, lr1_mac->tx_frequency, lr1_mac->tx_data_rate );

        gfsk_param.pkt_params.header_type           = RAL_GFSK_PKT_VAR_LEN;
        gfsk_param.pkt_params.pld_len_in_bytes      = lr1_mac->tx_payload_size;
        gfsk_param.pkt_params.preamble_len_in_bits  = 40;
        gfsk_param.pkt_params.sync_word_len_in_bits = 24;
        gfsk_param.pkt_params.dc_free               = RAL_GFSK_DC_FREE_WHITENING;
        gfsk_param.pkt_params.crc_type              = RAL_GFSK_CRC_2_BYTES_INV;

        gfsk_param.mod_params.fdev_in_hz   = 25000;
        gfsk_param.mod_params.br_in_bps    = tx_bitrate * 1000;
        gfsk_param.mod_params.bw_dsb_in_hz = 100000;
        gfsk_param.mod_params.pulse_shape  = RAL_GFSK_PULSE_SHAPE_BT_1;

        radio_params.pkt_type = RAL_PKT_TYPE_GFSK;
        radio_params.tx.gfsk  = gfsk_param;

        toa = ral_get_gfsk_time_on_air_in_ms( ( &lr1_mac->rp->radio->ral ), ( &gfsk_param.pkt_params ),
                                              ( &gfsk_param.mod_params ) );

        rp_task.type                  = RP_TASK_TYPE_TX_FSK;
        rp_task.launch_task_callbacks = lr1_stack_mac_tx_gfsk_launch_callback_for_rp;
    }
    else if( tx_modulation_type == LR_FHSS )
    {
        lr_fhss_v1_cr_t tx_cr;
        lr_fhss_v1_bw_t tx_bw;
        smtc_real_lr_fhss_dr_to_cr_bw( lr1_mac, lr1_mac->tx_data_rate, &tx_cr, &tx_bw );

        ralf_params_lr_fhss_t lr_fhss_param;
        memset( &lr_fhss_param, 0, sizeof( ralf_params_lr_fhss_t ) );

        lr_fhss_param.output_pwr_in_dbm = smtc_real_clamp_output_power_eirp_vs_freq_and_dr(
            lr1_mac, lr1_mac->tx_power, lr1_mac->tx_frequency, lr1_mac->tx_data_rate );
        uint32_t nb_max_hop_sequence =
            ral_lr_fhss_get_hop_sequence_count( &lr1_mac->rp->radio->ral, &lr_fhss_param.ral_lr_fhss_params );
        lr_fhss_param.hop_sequence_id = smtc_modem_hal_get_random_nb_in_range( 0, nb_max_hop_sequence );
        lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.modulation_type = LR_FHSS_V1_MODULATION_TYPE_GMSK_488;
        lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.cr              = tx_cr;
        lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.grid            = smtc_real_lr_fhss_get_grid( lr1_mac );
        lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.enable_hopping  = true;
        lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.bw              = tx_bw;
        lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.header_count    = smtc_real_lr_fhss_get_header_count( tx_cr );
        lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.sync_word       = smtc_real_get_lr_fhss_sync_word( lr1_mac );
        lr_fhss_param.ral_lr_fhss_params.center_frequency_in_hz         = lr1_mac->tx_frequency;
        lr_fhss_param.ral_lr_fhss_params.device_offset                  = 0;

        radio_params.tx.lr_fhss = lr_fhss_param;

        ral_lr_fhss_get_time_on_air_in_ms( ( &lr1_mac->rp->radio->ral ), &lr_fhss_param.ral_lr_fhss_params,
                                           lr1_mac->tx_payload_size, &toa );

        // SMTC_MODEM_HAL_TRACE_PRINTF( "  Hop ID = %d\n", lr_fhss_param.hop_sequence_id );

        rp_task.type                  = RP_TASK_TYPE_TX_LR_FHSS;
        rp_task.launch_task_callbacks = lr1_stack_mac_tx_lr_fhss_launch_callback_for_rp;
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( "TX MODULATION NOT SUPPORTED\n" );
    }

    uint8_t my_hook_id;
    if( rp_hook_get_id( lr1_mac->rp, lr1_mac, &my_hook_id ) != RP_HOOK_STATUS_OK )
    {
        smtc_modem_hal_lr1mac_panic( );
    }
    rp_task.hook_id          = my_hook_id;
    rp_task.duration_time_ms = toa;
    rp_task.start_time_ms    = lr1_mac->rtc_target_timer_ms - smtc_modem_hal_get_radio_tcxo_startup_delay_ms( );
    if( lr1_mac->send_at_time == true )
    {
        lr1_mac->send_at_time = false;  // reinit the flag
        rp_task.state         = RP_TASK_STATE_SCHEDULE;
    }
    else
    {
        rp_task.state = RP_TASK_STATE_ASAP;
    }
    lr1_mac->radio_process_state = RADIOSTATE_TX_ON;
    if( rp_task_enqueue( lr1_mac->rp, &rp_task, lr1_mac->tx_payload, lr1_mac->tx_payload_size, &radio_params ) !=
        RP_HOOK_STATUS_OK )
    {
        lr1_mac->radio_process_state = RADIOSTATE_ABORTED_BY_RP;
        SMTC_MODEM_HAL_TRACE_PRINTF( "Radio planner hook %d is busy\n", my_hook_id );
    }
}

void lr1_stack_mac_rx_radio_start( lr1_stack_mac_t* lr1_mac, const rx_win_type_t type, const uint32_t time_to_start )
{
    uint32_t          rx_frequency       = 0;
    uint8_t           rx_datarate        = lr1_mac->rx_data_rate;
    modulation_type_t rx_modulation_type = smtc_real_get_modulation_type_from_datarate( lr1_mac, rx_datarate );

    rp_radio_params_t radio_params = { 0 };
    lr1_mac->current_win           = type;

    switch( type )
    {
    case RX1:
        rx_frequency = lr1_mac->rx1_frequency;
        break;
    case RX2:
        rx_frequency = lr1_mac->rx2_frequency;
        break;
    default:
        smtc_modem_hal_lr1mac_panic( "RX windows unknow\n" );
        break;
    }

    if( rx_modulation_type == LORA )
    {
        uint8_t            rx_sf;
        lr1mac_bandwidth_t rx_bw;
        smtc_real_lora_dr_to_sf_bw( lr1_mac, rx_datarate, &rx_sf, &rx_bw );

        ralf_params_lora_t lora_param;
        memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );

        lora_param.sync_word       = smtc_real_get_sync_word( lr1_mac );
        lora_param.symb_nb_timeout = lr1_mac->rx_window_symb;
        lora_param.rf_freq_in_hz   = rx_frequency;

        lora_param.mod_params.cr   = smtc_real_get_coding_rate( lr1_mac );
        lora_param.mod_params.sf   = ( ral_lora_sf_t ) rx_sf;
        lora_param.mod_params.bw   = ( ral_lora_bw_t ) rx_bw;
        lora_param.mod_params.ldro = ral_compute_lora_ldro( lora_param.mod_params.sf, lora_param.mod_params.bw );

        lora_param.pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT;
        lora_param.pkt_params.pld_len_in_bytes     = 255;
        lora_param.pkt_params.crc_is_on            = false;
        lora_param.pkt_params.invert_iq_is_on      = true;
        lora_param.pkt_params.preamble_len_in_symb = smtc_real_get_preamble_len( lr1_mac, lora_param.mod_params.sf );

        radio_params.pkt_type         = RAL_PKT_TYPE_LORA;
        radio_params.rx.lora          = lora_param;
        radio_params.rx.timeout_in_ms = lr1_mac->rx_timeout_ms;
    }
    else if( rx_modulation_type == FSK )
    {
        uint8_t rx_bitrate;
        smtc_real_fsk_dr_to_bitrate( lr1_mac, rx_datarate, &rx_bitrate );

        ralf_params_gfsk_t gfsk_param;
        memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

        gfsk_param.rf_freq_in_hz  = rx_frequency;
        gfsk_param.sync_word      = smtc_real_get_gfsk_sync_word( lr1_mac );
        gfsk_param.dc_free_is_on  = true;
        gfsk_param.whitening_seed = GFSK_WHITENING_SEED;
        gfsk_param.crc_seed       = GFSK_CRC_SEED;
        gfsk_param.crc_polynomial = GFSK_CRC_POLYNOMIAL;

        gfsk_param.pkt_params.header_type           = RAL_GFSK_PKT_VAR_LEN;
        gfsk_param.pkt_params.pld_len_in_bytes      = 255;
        gfsk_param.pkt_params.preamble_len_in_bits  = 40;
        gfsk_param.pkt_params.sync_word_len_in_bits = 24;
        gfsk_param.pkt_params.dc_free               = RAL_GFSK_DC_FREE_WHITENING;
        gfsk_param.pkt_params.crc_type              = RAL_GFSK_CRC_2_BYTES_INV;

        gfsk_param.mod_params.fdev_in_hz   = 25000;
        gfsk_param.mod_params.bw_dsb_in_hz = 100000;
        gfsk_param.mod_params.pulse_shape  = RAL_GFSK_PULSE_SHAPE_BT_1;
        gfsk_param.mod_params.br_in_bps    = rx_bitrate * 1000;

        radio_params.pkt_type         = RAL_PKT_TYPE_GFSK;
        radio_params.rx.gfsk          = gfsk_param;
        radio_params.rx.timeout_in_ms = lr1_mac->rx_timeout_symb_in_ms;
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( "MODULATION NOT SUPPORTED\n" );
    }

    uint8_t my_hook_id;
    if( rp_hook_get_id( lr1_mac->rp, lr1_mac, &my_hook_id ) != RP_HOOK_STATUS_OK )
    {
        smtc_modem_hal_lr1mac_panic( );
    }

    rp_task_t rp_task = {
        .hook_id = my_hook_id,
        .type    = ( radio_params.pkt_type == RAL_PKT_TYPE_LORA ) ? RP_TASK_TYPE_RX_LORA : RP_TASK_TYPE_RX_FSK,
        .launch_task_callbacks = ( radio_params.pkt_type == RAL_PKT_TYPE_LORA )
                                     ? lr1_stack_mac_rx_lora_launch_callback_for_rp
                                     : lr1_stack_mac_rx_gfsk_launch_callback_for_rp,
        .state            = RP_TASK_STATE_SCHEDULE,
        .start_time_ms    = time_to_start,
        .duration_time_ms = lr1_mac->rx_timeout_symb_in_ms,
    };

    if( rp_task_enqueue( lr1_mac->rp, &rp_task, lr1_mac->rx_payload, 255, &radio_params ) == RP_HOOK_STATUS_OK )
    {
        lr1_mac->radio_process_state = RADIOSTATE_RX_ON;

        SMTC_MODEM_HAL_TRACE_PRINTF( "\n  Open RX%d for Hook Id = %d", type - RX1 + 1, my_hook_id );

        if( radio_params.pkt_type == RAL_PKT_TYPE_LORA )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF(
                "  %s LoRa at %u ms: freq:%u, SF%u, %s, sync word = 0x%02x\n", smtc_name_rx_windows[type],
                time_to_start, radio_params.rx.lora.rf_freq_in_hz, radio_params.rx.lora.mod_params.sf,
                smtc_name_bw[radio_params.rx.lora.mod_params.bw], smtc_real_get_sync_word( lr1_mac ) );
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "  %s FSK freq:%d\n", smtc_name_rx_windows[type],
                                         radio_params.rx.gfsk.rf_freq_in_hz );
        }
    }
    else
    {
        lr1_mac->radio_process_state = RADIOSTATE_ABORTED_BY_RP;
        SMTC_MODEM_HAL_TRACE_PRINTF( "Radio planner hook %d is busy\n", my_hook_id );
    }
}

void lr1_stack_mac_rp_callback( lr1_stack_mac_t* lr1_mac )
{
    uint32_t tcurrent_ms;
    uint8_t  my_hook_id;
    rp_hook_get_id( lr1_mac->rp, lr1_mac, &my_hook_id );
    rp_get_status( lr1_mac->rp, my_hook_id, &tcurrent_ms, &( lr1_mac->planner_status ) );

    switch( lr1_mac->planner_status )
    {
    case RP_STATUS_TX_DONE:
        lr1_mac->isr_tx_done_radio_timestamp = tcurrent_ms;  //@info Timestamp only on txdone it
        break;

    case RP_STATUS_RX_PACKET:
        // save rssi and snr
        lr1_mac->rx_metadata.timestamp = tcurrent_ms;
        lr1_mac->rx_metadata.rx_snr    = lr1_mac->rp->radio_params[my_hook_id].rx.lora_pkt_status.snr_pkt_in_db;
        lr1_mac->rx_metadata.rx_rssi   = lr1_mac->rp->radio_params[my_hook_id].rx.lora_pkt_status.rssi_pkt_in_dbm;
        lr1_mac->rx_payload_size       = ( uint8_t ) lr1_mac->rp->payload_size[my_hook_id];

        SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "payload size receive = %u, snr = %d , rssi = %d\n",
                                           lr1_mac->rx_payload_size,
                                           lr1_mac->rp->radio_params[my_hook_id].rx.lora_pkt_status.snr_pkt_in_db,
                                           lr1_mac->rp->radio_params[my_hook_id].rx.lora_pkt_status.rssi_pkt_in_dbm );

        if( lr1_stack_mac_downlink_check_under_it( lr1_mac ) != OKLORAWAN )
        {  // Case receive a packet but it isn't a valid packet
            SMTC_MODEM_HAL_TRACE_MSG( "Receive a packet But rejected and too late to restart\n" );
            lr1_mac->planner_status  = RP_STATUS_RX_TIMEOUT;
            lr1_mac->rx_payload_size = 0;
        }
        break;

    case RP_STATUS_RX_CRC_ERROR:
        SMTC_MODEM_HAL_TRACE_PRINTF( "RP_STATUS_RX_CRC_ERROR\n" );
        break;

    case RP_STATUS_RX_TIMEOUT: {
#ifndef BSP_LR1MAC_DISABLE_FINE_TUNE
        uint32_t rx_timestamp_calibration = tcurrent_ms;
        uint32_t rx_delay_ms;

        if( lr1_mac->current_win == RX1 )
        {
            rx_delay_ms = lr1_mac->rx1_delay_s;
        }
        else
        {
            rx_delay_ms = lr1_mac->rx1_delay_s + 1;
        }
        rx_delay_ms *= 1000;

        int32_t error_fine_tune = rx_timestamp_calibration -
                                  ( lr1_mac->isr_tx_done_radio_timestamp + rx_delay_ms +
                                    lr1_mac->rx_timeout_symb_in_ms + lr1_mac->rx_offset_ms ) -
                                  lr1_mac->fine_tune_board_setting_delay_ms[lr1_mac->rx_data_rate] -
                                  smtc_modem_hal_get_radio_tcxo_startup_delay_ms( ) -
                                  smtc_modem_hal_get_board_delay_ms( );

        SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG(
            "DR%u Fine tune correction (ms) = %d, error fine tune (ms) = %d, lr1_mac->rx_offset_ms = %d\n",
            lr1_mac->rx_data_rate, lr1_mac->fine_tune_board_setting_delay_ms[lr1_mac->rx_data_rate], error_fine_tune,
            lr1_mac->rx_offset_ms );

        if( error_fine_tune < 0 )
        {
            lr1_mac->fine_tune_board_setting_delay_ms[lr1_mac->rx_data_rate] -= 1;
        }
        else if( error_fine_tune > 0 )
        {
            lr1_mac->fine_tune_board_setting_delay_ms[lr1_mac->rx_data_rate] += 1;
        }

#endif  // BSP_LR1MAC_DISABLE_FINE_TUNE
    }
    break;

    case RP_STATUS_TASK_ABORTED:
        SMTC_MODEM_HAL_TRACE_PRINTF( "lr1mac task aborted by the radioplanner\n" );
        break;

    default:
        SMTC_MODEM_HAL_TRACE_PRINTF( "receive It RADIO error %u\n", lr1_mac->planner_status );
        break;
    }

    switch( lr1_mac->radio_process_state )
    {
    case RADIOSTATE_TX_ON:
        lr1_mac->radio_process_state  = RADIOSTATE_TX_FINISHED;
        lr1_mac->available_app_packet = NO_LORA_RXPACKET_AVAILABLE;
        break;

    case RADIOSTATE_RX_ON:
        lr1_mac->radio_process_state = RADIOSTATE_RX_FINISHED;
        break;

    case RADIOSTATE_RX_FINISHED:
    case RADIOSTATE_TX_FINISHED:
        // You should not receive an IT in this radio state
    default:
        if( lr1_mac->planner_status != RP_STATUS_TASK_ABORTED )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "lr1_mac->planner_status %d", lr1_mac->planner_status );
            smtc_modem_hal_lr1mac_panic( );
        }
        break;
    }
    if( lr1_mac->planner_status == RP_STATUS_TASK_ABORTED )
    {
        lr1_mac->radio_process_state = RADIOSTATE_ABORTED_BY_RP;
    }
}

void lr1_stack_mac_rx_timer_configure( lr1_stack_mac_t* lr1_mac, const rx_win_type_t type )
{
    const uint32_t tcurrent_ms = smtc_modem_hal_get_time_in_ms( );
    bool           is_type_ok  = true;
    uint32_t       delay_ms;

    smtc_real_set_rx_config( lr1_mac, type );

    switch( type )
    {
    case RX1:
        delay_ms = lr1_mac->rx1_delay_s;
        break;

    case RX2:
        delay_ms = lr1_mac->rx1_delay_s + 1;
        break;

    default:
        is_type_ok = false;
        smtc_modem_hal_lr1mac_panic( "RX windows unknow\n" );
        break;
    }

    delay_ms *= 1000;

    if( is_type_ok == true )
    {
        uint8_t            sf;
        lr1mac_bandwidth_t bw;
        uint8_t            kbitrate;

        modulation_type_t rx_modulation_type =
            smtc_real_get_modulation_type_from_datarate( lr1_mac, lr1_mac->rx_data_rate );

        if( rx_modulation_type == LORA )
        {
            smtc_real_lora_dr_to_sf_bw( lr1_mac, lr1_mac->rx_data_rate, &sf, &bw );
        }
        else if( rx_modulation_type == FSK )
        {
            smtc_real_fsk_dr_to_bitrate( lr1_mac, lr1_mac->rx_data_rate, &kbitrate );
        }
        else
        {
            smtc_modem_hal_lr1mac_panic( "MODULATION NOT SUPPORTED\n" );
        }

        uint32_t board_delay_ms = smtc_modem_hal_get_radio_tcxo_startup_delay_ms( ) +
                                  +smtc_modem_hal_get_board_delay_ms( ) +
                                  lr1_mac->fine_tune_board_setting_delay_ms[lr1_mac->rx_data_rate];

        smtc_real_get_rx_window_parameters( lr1_mac, lr1_mac->rx_data_rate, delay_ms, &lr1_mac->rx_window_symb,
                                            &lr1_mac->rx_timeout_symb_in_ms, &lr1_mac->rx_timeout_ms, 0 );
        smtc_real_get_rx_start_time_offset_ms( lr1_mac, lr1_mac->rx_data_rate, board_delay_ms, lr1_mac->rx_window_symb,
                                               &lr1_mac->rx_offset_ms );

        SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG(
            "rx_offset_ms:%d, rx_timeout_symb_in_ms:%d, rx_window_symb: %d, board_delay_ms:%d\n", lr1_mac->rx_offset_ms,
            lr1_mac->rx_timeout_symb_in_ms, lr1_mac->rx_window_symb, board_delay_ms );

        // Do not factorize the talarm_ms, the if does not check the same value
        uint32_t talarm_ms = delay_ms + lr1_mac->isr_tx_done_radio_timestamp - tcurrent_ms;
        if( ( int32_t )( talarm_ms - lr1_mac->rx_offset_ms ) < 0 )
        {
            lr1_mac->radio_process_state = RADIOSTATE_RX_FINISHED;
        }
        else
        {
            lr1_stack_mac_rx_radio_start( lr1_mac, type, tcurrent_ms + talarm_ms + lr1_mac->rx_offset_ms );
            SMTC_MODEM_HAL_TRACE_PRINTF( "  Timer will expire in %d ms\n", ( talarm_ms + lr1_mac->rx_offset_ms ) );
        }
    }
}

rx_packet_type_t lr1_stack_mac_rx_frame_decode( lr1_stack_mac_t* lr1_mac )
{
    int              status         = OKLORAWAN;
    rx_packet_type_t rx_packet_type = NO_MORE_VALID_RX_PACKET;
    uint32_t         mic_in;
    uint8_t          rx_ftype;
    uint8_t          rx_major;
    uint8_t          tx_ack_bit;
    uint32_t         fcnt_dwn_stack_tmp = 0;

    status += lr1mac_rx_payload_min_size_check( lr1_mac->rx_payload_size );
    status += lr1mac_rx_payload_max_size_check( lr1_mac, lr1_mac->rx_payload_size, lr1_mac->rx_data_rate );

    if( status != OKLORAWAN )
    {
        return NO_MORE_VALID_RX_PACKET;
    }
    status += lr1mac_rx_mhdr_extract( lr1_mac->rx_payload, &rx_ftype, &rx_major, &tx_ack_bit );
    if( status != OKLORAWAN )
    {
        return NO_MORE_VALID_RX_PACKET;
    }
    //**********************************************************************
    //                 Case : the receive packet is a JoinResponse
    //**********************************************************************

    if( rx_ftype == JOIN_ACCEPT )
    {
        smtc_modem_crypto_return_code_t rc;
        rc = smtc_modem_crypto_process_join_accept( &lr1_mac->rx_payload[0], lr1_mac->rx_payload_size,
                                                    &lr1_mac->rx_payload[0] );
        if( rc == SMTC_MODEM_CRYPTO_RC_SUCCESS )
        {
            lr1_mac->no_rx_packet_count_in_mobile_mode = 0;
            lr1_mac->no_rx_packet_count                = 0;
            rx_packet_type                             = JOIN_ACCEPT_PACKET;
            lr1_mac->rx_payload_size                   = lr1_mac->rx_payload_size - MICSIZE;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( " Process join accept failed with code = %d\n", rc );
        }
    }
    else
    {
        //**********************************************************************
        //               Case : the receive packet is not a JoinResponse
        //**********************************************************************
        uint16_t fcnt_dwn_tmp = 0;
#if defined( PERF_TEST_ENABLED )
        fcnt_dwn_stack_tmp = 0;
#else
        fcnt_dwn_stack_tmp = lr1_mac->fcnt_dwn;
#endif
        status += lr1mac_rx_fhdr_extract( lr1_mac->rx_payload, lr1_mac->rx_payload_size, &( lr1_mac->rx_fopts_length ),
                                          &fcnt_dwn_tmp, lr1_mac->dev_addr, &( lr1_mac->rx_metadata.rx_fport ),
                                          &( lr1_mac->rx_payload_empty ), &( lr1_mac->rx_fctrl ), lr1_mac->rx_fopts );
        if( status == OKLORAWAN )
        {
            status = lr1mac_fcnt_dwn_accept( fcnt_dwn_tmp, &fcnt_dwn_stack_tmp );
        }
        if( status == OKLORAWAN )
        {
            lr1_mac->rx_payload_size = lr1_mac->rx_payload_size - MICSIZE;
            memcpy1( ( uint8_t* ) &mic_in, &lr1_mac->rx_payload[lr1_mac->rx_payload_size], MICSIZE );

            if( smtc_modem_crypto_verify_mic( &lr1_mac->rx_payload[0], lr1_mac->rx_payload_size, SMTC_SE_NWK_S_ENC_KEY,
                                              lr1_mac->dev_addr, 1, fcnt_dwn_stack_tmp,
                                              mic_in ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
            {
                status = ERRORLORAWAN;
            }
        }
        if( status == OKLORAWAN )
        {
            // reset retransmission counter if received on RX1 or RX2
            lr1_mac->nb_trans_cpt = 1;

            // test the ack bit when tx_mtype == CONF_DATA_UP
            if( ( ( ( lr1_mac->rx_fctrl >> DL_ACK_BIT ) & 0x01 ) == 0x01 ) && ( lr1_mac->tx_mtype == CONF_DATA_UP ) )
            {
                lr1_mac->rx_ack_bit = 1;
            }

            // FPending bit
            lr1_mac->rx_fpending_bit_current     = ( lr1_mac->rx_fctrl >> DL_FPENDING_BIT ) & 0x01;
            lr1_mac->rx_metadata.rx_fpending_bit = lr1_mac->rx_fpending_bit_current;
            if( lr1_mac->current_win == RX1 )
            {
                lr1_mac->rx_metadata.rx_frequency_hz = lr1_mac->rx1_frequency;
            }
            else if( lr1_mac->current_win == RX2 )
            {
                lr1_mac->rx_metadata.rx_frequency_hz = lr1_mac->rx2_frequency;
            }
            else
            {
                smtc_modem_hal_lr1mac_panic( "Rx Window invalid\n" );
            }

            lr1_mac->rx_metadata.rx_datarate = lr1_mac->rx_data_rate;

            if( lr1_mac->rx_payload_empty == 0 )  // rx payload not empty
            {
                lr1_mac->rx_payload_size = lr1_mac->rx_payload_size - FHDROFFSET - 1 - lr1_mac->rx_fopts_length;

                // Receive a management frame
                // => set rx_packet_type = NWKRXPACKET
                // => if ack bit is set to one : notify the upper layer that the stack have received an ack bit
                // => set available_app_packet to LORA_RX_PACKET_AVAILABLE with length = 0

                if( lr1_mac->rx_metadata.rx_fport == 0 )
                {  // receive a mac management frame without fopts
                    if( lr1_mac->rx_fopts_length == 0 )
                    {
                        if( smtc_modem_crypto_payload_decrypt(
                                &lr1_mac->rx_payload[FHDROFFSET + 1], lr1_mac->rx_payload_size, SMTC_SE_NWK_S_ENC_KEY,
                                lr1_mac->dev_addr, 1, fcnt_dwn_stack_tmp,
                                &lr1_mac->nwk_payload[0] ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
                        {
                            smtc_modem_hal_lr1mac_panic( "Crypto error during payload decryption\n" );
                        }
                        if( lr1_mac->rx_payload_size > NWK_MAC_PAYLOAD_MAX_SIZE )
                        {
                            SMTC_MODEM_HAL_TRACE_WARNING( " Receive too many nwk frames\n" );
                        }
                        else
                        {
                            lr1_mac->nwk_payload_size = lr1_mac->rx_payload_size;
                            rx_packet_type            = NWKRXPACKET;
                        }
                    }
                    else
                    {
                        status = ERRORLORAWAN;
                        SMTC_MODEM_HAL_TRACE_WARNING( " Receive an not valid packet with FOpts bytes on port zero\n" );
                    }
                }
                else
                {
                    // Receive a app frame with size > 0
                    // =>  if rx_fopts_length > 0 set rx_packet_type = USERRX_FOPTSPACKET and copy fopts data
                    // =>  notify the upper layer that the stack have received a payload : set available_app_packet
                    //     to LORA_RX_PACKET_AVAILABLE with length > 0
                    if( smtc_modem_crypto_payload_decrypt(
                            &lr1_mac->rx_payload[FHDROFFSET + 1 + lr1_mac->rx_fopts_length], lr1_mac->rx_payload_size,
                            SMTC_SE_APP_S_KEY, lr1_mac->dev_addr, 1, fcnt_dwn_stack_tmp,
                            &lr1_mac->rx_payload[0] ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
                    {
                        smtc_modem_hal_lr1mac_panic( "Crypto error during payload decryption\n" );
                    }

                    if( lr1_mac->rx_fopts_length != 0 )
                    {
                        memcpy1( lr1_mac->nwk_payload, lr1_mac->rx_fopts, lr1_mac->rx_fopts_length );
                        lr1_mac->nwk_payload_size = lr1_mac->rx_fopts_length;
                        rx_packet_type            = USERRX_FOPTSPACKET;
                    }
                    else
                    {
                        rx_packet_type = USER_RX_PACKET;
                    }
                    lr1_mac->available_app_packet = LORA_RX_PACKET_AVAILABLE;
                }
            }
            else
            {
                // Receive an empty user payload
                // => if rx_fopts_length > 0 set rx_packet_type = USERRX_FOPTSPACKET and copy fopts data
                // => notify the upper layer that the stack have received a payload : ack_bit is set to 1
                if( lr1_mac->rx_fopts_length != 0 )
                {
                    memcpy1( lr1_mac->nwk_payload, lr1_mac->rx_fopts, lr1_mac->rx_fopts_length );
                    lr1_mac->nwk_payload_size = lr1_mac->rx_fopts_length;
                    rx_packet_type            = USERRX_FOPTSPACKET;
                }
                else
                {
                    rx_packet_type = USER_RX_PACKET;
                }
            }
        }
    }

    if( status == OKLORAWAN )
    {
        lr1_mac->rx_ftype   = rx_ftype;
        lr1_mac->rx_major   = rx_major;
        lr1_mac->tx_ack_bit = tx_ack_bit;

        lr1_mac->fcnt_dwn                          = fcnt_dwn_stack_tmp;
        lr1_mac->adr_ack_cnt                       = 0;  // reset adr counter, receive a valid frame.
        lr1_mac->no_rx_packet_count_in_mobile_mode = 0;
        lr1_mac->no_rx_packet_count                = 0;
        lr1_mac->tx_fopts_current_length           = 0;  // reset the fopts of the sticky set in payload
        lr1_mac->tx_fopts_lengthsticky             = 0;  // reset the fopts of the sticky cmd received on a valide frame
                                                         // if received on RX1 or RX2
    }

    SMTC_MODEM_HAL_TRACE_PRINTF( " rx_packet_type = %d\n", rx_packet_type );
    return ( rx_packet_type );
}

void lr1_stack_mac_update_tx_done( lr1_stack_mac_t* lr1_mac )
{
    if( ( lr1_mac->no_rx_packet_count_in_mobile_mode < 0xFFFF ) && ( lr1_mac->adr_mode_select != STATIC_ADR_MODE ) )
    {
        lr1_mac->no_rx_packet_count_in_mobile_mode++;
    }

    if( lr1_mac->no_rx_packet_count < 0xFFFF )
    {
        lr1_mac->no_rx_packet_count++;
    }

    if( lr1_mac->link_check_user_req == USER_MAC_REQ_REQUESTED )
    {
        lr1_mac->link_check_user_req = USER_MAC_REQ_SENT;
    }

    if( lr1_mac->device_time_user_req == USER_MAC_REQ_REQUESTED )
    {
        lr1_mac->device_time_user_req                     = USER_MAC_REQ_SENT;
        lr1_mac->timestamp_tx_done_device_time_req_ms_tmp = lr1_mac->isr_tx_done_radio_timestamp;
    }

    if( lr1_mac->ping_slot_info_user_req == USER_MAC_REQ_REQUESTED )
    {
        lr1_mac->ping_slot_info_user_req = USER_MAC_REQ_SENT;
    }
}

void lr1_stack_mac_update( lr1_stack_mac_t* lr1_mac )
{
    lr1_mac->adr_ack_limit       = lr1_mac->adr_ack_limit_init;
    lr1_mac->adr_ack_delay       = lr1_mac->adr_ack_delay_init;
    lr1_mac->type_of_ans_to_send = NOFRAME_TOSEND;

    if( lr1_mac->join_status == JOINING )
    {
        // get current timestamp to check which duty cycle will be applied
        uint32_t current_time_s = smtc_modem_hal_get_time_in_s( );

        lr1_mac->retry_join_cpt++;

        if( current_time_s < ( lr1_mac->first_join_timestamp + 3600 ) )
        {
            // during first hour after first join try => duty cycle of 1/100 ie 36s over 1 hour
            lr1_mac->next_time_to_join_seconds = current_time_s + ( lr1_stack_toa_get( lr1_mac ) ) / 10;
            // ts=cur_ts+(toa_s*100) = cur_ts + (toa_ms / 1000) * 100 = cur_ts + toa_ms/10
        }
        else if( current_time_s < ( lr1_mac->first_join_timestamp + 36000 + 3600 ) )
        {
            // during the 10 hours following first hour after first join try => duty cycle of 1/1000 ie 36s over 10
            // hours
            lr1_mac->next_time_to_join_seconds = current_time_s + ( lr1_stack_toa_get( lr1_mac ) );
            // ts=cur_ts+(toa_s*1000) = cur_ts + (toa_ms / 1000) * 1000 = cur_ts + toa_ms
        }
        else
        {
            // Following the first 11 hours after first join try => duty cycle of 1/10000 ie 8.7s over 24 hours
            lr1_mac->next_time_to_join_seconds = current_time_s + ( lr1_stack_toa_get( lr1_mac ) ) * 10;
            // ts=cur_ts+(toa_s*10000) = cur_ts + (toa_ms / 1000) * 10000 = cur_ts + toa_ms*10
        }

        // Now join status can be set as not joined
        lr1_mac->join_status = NOT_JOINED;
    }

    if( lr1_mac->adr_ack_cnt >= lr1_mac->adr_ack_limit )
    {
        lr1_mac->adr_ack_req = 1;
    }
    else
    {
        lr1_mac->adr_ack_req = 0;
    }

    if( lr1_mac->nb_trans_cpt <= 1 )
    {
        // could also be set to 1 if receive valid ans
        lr1_mac->nb_trans_cpt = 1;  // error case shouldn't exist
        lr1_mac->fcnt_up++;
        lr1_mac->adr_ack_cnt++;  // increment adr counter each new uplink frame
    }
    else
    {
        lr1_mac->type_of_ans_to_send = USRFRAME_TORETRANSMIT;
        lr1_mac->nb_trans_cpt--;
    }

    if( lr1_mac->adr_ack_cnt >= lr1_mac->adr_ack_limit + lr1_mac->adr_ack_delay )
    {
        // In case of retransmission, if the packet is too long for the next DR, don't decrease the DR
        if( lr1_mac->type_of_ans_to_send == USRFRAME_TORETRANSMIT )
        {
            uint8_t dr_tmp = smtc_real_decrement_dr_simulation( lr1_mac );
            if( smtc_real_is_payload_size_valid( lr1_mac, dr_tmp, lr1_mac->app_payload_size,
                                                 lr1_mac->uplink_dwell_time ) == OKLORAWAN )
            {
                smtc_real_decrement_dr( lr1_mac );
                lr1_mac->adr_ack_cnt = lr1_mac->adr_ack_limit;
            }
        }
        else
        {
            smtc_real_decrement_dr( lr1_mac );
            lr1_mac->adr_ack_cnt = lr1_mac->adr_ack_limit;
        }
    }

    if( ( lr1_mac->adr_ack_cnt >= lr1_mac->no_rx_packet_reset_threshold ) &&
        ( lr1_mac->no_rx_packet_reset_threshold > 0 ) )
    {
        smtc_modem_hal_lr1mac_panic( "Reach max tx frame without dl, ul cnt:%d\n", lr1_mac->adr_ack_cnt );
    }

    // If tx_fopts_length > tx_fopts_lengthsticky, first uplink with Answer(s),
    // put in Network to not penalize the App Payload
    if( ( lr1_mac->tx_fopts_length >= lr1_mac->tx_fopts_lengthsticky ) && ( lr1_mac->tx_fopts_length > 0 ) )
    {
        lr1_mac->nwk_ans_size = lr1_mac->tx_fopts_length;
        memcpy1( lr1_mac->nwk_ans, lr1_mac->tx_fopts_data, lr1_mac->tx_fopts_length );
        lr1_mac->type_of_ans_to_send = NWKFRAME_TOSEND;
        lr1_mac->nb_trans_cpt        = lr1_mac->nb_trans;
    }
    else  // Concerns sticky commands, put in FOpts
    {
        lr1_mac->tx_fopts_current_length = lr1_mac->tx_fopts_lengthsticky;
        memcpy1( lr1_mac->tx_fopts_current_data, lr1_mac->tx_fopts_datasticky, lr1_mac->tx_fopts_lengthsticky );
    }
    lr1_mac->tx_fopts_length = 0;

    if( lr1_mac->join_status == JOINED )
    {
        if( lr1_mac->type_of_ans_to_send != USRFRAME_TORETRANSMIT )
        {
            status_lorawan_t status = smtc_real_get_next_dr( lr1_mac );
            if( status == ERRORLORAWAN )
            {
                smtc_modem_hal_mcu_panic( " Data Rate incompatible with channel mask\n" );
            }
        }
    }
    switch( lr1_mac->type_of_ans_to_send )
    {
    case NOFRAME_TOSEND:
        break;
    case NWKFRAME_TOSEND:
        if( smtc_real_is_payload_size_valid( lr1_mac, lr1_mac->tx_data_rate, lr1_mac->nwk_ans_size,

                                             lr1_mac->uplink_dwell_time ) != OKLORAWAN )
        {
            lr1_mac->nwk_ans_size = lr1_stack_mac_cmd_ans_cut(
                lr1_mac->nwk_ans, lr1_mac->nwk_ans_size,
                smtc_real_get_max_payload_size( lr1_mac, lr1_mac->tx_data_rate, lr1_mac->uplink_dwell_time ) -
                    FHDROFFSET );
        }
        lr1_mac->tx_fport_present = true;
        memcpy1( &lr1_mac->tx_payload[FHDROFFSET + lr1_mac->tx_fport_present], lr1_mac->nwk_ans,
                 lr1_mac->nwk_ans_size );
        lr1_mac->app_payload_size = lr1_mac->nwk_ans_size;
        lr1_mac->tx_fport         = PORTNWK;
        lr1_mac->tx_mtype         = UNCONF_DATA_UP;  //@note Mtype have to be confirm
        lr1_stack_mac_tx_frame_build( lr1_mac );
        lr1_stack_mac_tx_frame_encrypt( lr1_mac );
        break;

    case USERACK_TOSEND:
        break;
    }
}

status_lorawan_t lr1_stack_mac_cmd_parse( lr1_stack_mac_t* lr1_mac )
{
    uint8_t cmd_identifier;

    lr1_mac->nwk_payload_index     = 0;
    lr1_mac->nwk_ans_size          = 0;
    lr1_mac->tx_fopts_length       = 0;
    lr1_mac->tx_fopts_lengthsticky = 0;

    while( lr1_mac->nwk_payload_size > lr1_mac->nwk_payload_index )
    {  //@note MacNwkPayloadSize and lr1_mac->nwk_payload[0] are updated in Parser's method

        if( lr1_mac->tx_fopts_length > DEVICE_MAC_PAYLOAD_MAX_SIZE )
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "too much cmd in the payload\n" );
            return ( ERRORLORAWAN );
        }
        cmd_identifier = lr1_mac->nwk_payload[lr1_mac->nwk_payload_index];
        switch( cmd_identifier )
        {
        case LINK_CHECK_ANS:
            if( lr1_mac->link_check_user_req == USER_MAC_REQ_SENT )
            {
                lr1_mac->link_check_user_req = USER_MAC_REQ_ACKED;
                link_check_parser( lr1_mac );
            }
            break;

        case LINK_ADR_REQ: {
            uint8_t nb_link_adr_req = 0;
            // extract the number of multiple link adr req specification in LoRaWan1.0.2
            while( ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( nb_link_adr_req * LINK_ADR_REQ_SIZE )] ==
                     LINK_ADR_REQ ) &&
                   ( lr1_mac->nwk_payload_index + ( ( nb_link_adr_req + 1 ) * LINK_ADR_REQ_SIZE ) <=
                     lr1_mac->nwk_payload_size ) )
            {
                nb_link_adr_req++;
            }
            link_adr_parser( lr1_mac, nb_link_adr_req );
            break;
        }

        case DUTY_CYCLE_REQ:
            duty_cycle_parser( lr1_mac );  //@note send answer but do nothing
            break;

        case RXPARRAM_SETUP_REQ:
            rx_param_setup_parser( lr1_mac );
            break;

        case DEV_STATUS_REQ:
            dev_status_parser( lr1_mac );  //@note  Done but margin have no sense to be implemented
            break;

        case NEW_CHANNEL_REQ:
            new_channel_parser( lr1_mac );
            break;

        case RXTIMING_SETUP_REQ:
            rx_timing_setup_parser( lr1_mac );
            break;

        case TXPARAM_SETUP_REQ:
            tx_param_setup_parser( lr1_mac );
            break;

        case DL_CHANNEL_REQ:
            dl_channel_parser( lr1_mac );
            break;

        case DEVICE_TIME_ANS: {
            if( lr1_mac->device_time_user_req == USER_MAC_REQ_SENT )
            {
                if( device_time_ans_parser( lr1_mac ) == OKLORAWAN )
                {
                    lr1_mac->device_time_user_req                 = USER_MAC_REQ_ACKED;
                    lr1_mac->timestamp_tx_done_device_time_req_ms = lr1_mac->timestamp_tx_done_device_time_req_ms_tmp;
                    lr1_mac->timestamp_last_device_time_ans_s     = smtc_modem_hal_get_time_in_s( );
                    if( lr1_mac->device_time_callback != NULL )
                    {
                        lr1_mac->device_time_callback( lr1_mac->device_time_callback_context,
                                                       lr1_mac->timestamp_last_device_time_ans_s );
                    }
                }
            }
            break;
        }

        // Class B
        case PING_SLOT_INFO_ANS: {
            if( lr1_mac->ping_slot_info_user_req == USER_MAC_REQ_SENT )
            {
                if( ping_slot_info_ans_parser( lr1_mac ) == OKLORAWAN )
                {
                    lr1_mac->ping_slot_periodicity_ans = lr1_mac->ping_slot_periodicity_req;
                    lr1_mac->ping_slot_info_user_req   = USER_MAC_REQ_ACKED;
                }
            }
            break;
        }

        case PING_SLOT_CHANNEL_REQ:
            ping_slot_channel_req_parser( lr1_mac );
            break;

        case BEACON_FREQ_REQ:
            beacon_freq_req_parser( lr1_mac );
            break;

        default:
            lr1_mac->nwk_payload_size = 0;
            SMTC_MODEM_HAL_TRACE_PRINTF( " Unknown mac command %02x\n", cmd_identifier );
            break;
        }
    }

    return OKLORAWAN;
}
void lr1_stack_mac_join_request_build( lr1_stack_mac_t* lr1_mac )
{
    uint8_t dev_eui[SMTC_SE_EUI_SIZE];
    uint8_t join_eui[SMTC_SE_EUI_SIZE];

    smtc_secure_element_get_joineui( join_eui );
    smtc_secure_element_get_deveui( dev_eui );

    SMTC_MODEM_HAL_TRACE_ARRAY( "DevEUI", dev_eui, 8 );
    SMTC_MODEM_HAL_TRACE_ARRAY( "JoinEUI", join_eui, 8 );
    if( lr1_mac->dev_nonce < 0xFFFF )
    {
#if defined( PERF_TEST_ENABLED )
        lr1_mac->dev_nonce = 0;
#else
        lr1_mac->dev_nonce += 1;
#endif
    }
    SMTC_MODEM_HAL_TRACE_PRINTF( "DevNonce 0x%x\n", lr1_mac->dev_nonce );
    lr1_mac->tx_mtype     = JOIN_REQUEST;
    lr1_mac->nb_trans_cpt = 1;
    lr1_mac->nb_trans     = 1;
    mac_header_set( lr1_mac );
    for( int i = 0; i < 8; i++ )
    {
        lr1_mac->tx_payload[1 + i] = join_eui[7 - i];
        lr1_mac->tx_payload[9 + i] = dev_eui[7 - i];
    }
    lr1_mac->tx_payload[17]  = ( uint8_t )( ( lr1_mac->dev_nonce & 0x00FF ) );
    lr1_mac->tx_payload[18]  = ( uint8_t )( ( lr1_mac->dev_nonce & 0xFF00 ) >> 8 );
    lr1_mac->tx_payload_size = 19;
    uint32_t mic;
    //    FcntUp = 1;
    if( smtc_modem_crypto_compute_join_mic( &lr1_mac->tx_payload[0], lr1_mac->tx_payload_size, &mic ) !=
        SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        smtc_modem_hal_lr1mac_panic( "Crypto error during join mic computation\n" );
    }

    memcpy1( &lr1_mac->tx_payload[lr1_mac->tx_payload_size], ( uint8_t* ) &mic, 4 );
    lr1_mac->tx_payload_size = lr1_mac->tx_payload_size + 4;
}

status_lorawan_t lr1_stack_mac_join_accept( lr1_stack_mac_t* lr1_mac )
{
    uint8_t  join_nonce[6];    // JoinNonce + NetID
    uint32_t join_nonce_tmp;   // JoinNonce only
    uint32_t join_nonce_prev;  // JoinNonce only
    uint32_t i;

    memcpy1( join_nonce, &lr1_mac->rx_payload[1], 6 );

#if defined( PERF_TEST_ENABLED )
    lr1_mac->join_nonce[0] = 0;
    lr1_mac->join_nonce[1] = 0;
    lr1_mac->join_nonce[2] = 0;
#endif

    join_nonce_prev = lr1_mac->join_nonce[0];
    join_nonce_prev |= lr1_mac->join_nonce[1] << 8;
    join_nonce_prev |= lr1_mac->join_nonce[2] << 16;

    join_nonce_tmp = join_nonce[0];
    join_nonce_tmp |= join_nonce[1] << 8;
    join_nonce_tmp |= join_nonce[2] << 16;

    // TODO check devnonce as a counter !
    if( join_nonce_tmp == join_nonce_prev )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "JoinNonce invalid (new:%u == prev:%u)\n", join_nonce_tmp, join_nonce_prev );
        return ERRORLORAWAN;
    }
    memcpy1( lr1_mac->join_nonce, join_nonce, 6 );
    smtc_modem_crypto_derive_skeys( &join_nonce[0], &join_nonce[3], lr1_mac->dev_nonce );

    if( lr1_mac->rx_payload_size > 13 )  // MIC has been removed (17 bytes - 4 bytes MIC)
    {                                    // cflist are presents
        for( i = 0; i < 16; i++ )
        {
            lr1_mac->cf_list[i] = lr1_mac->rx_payload[13 + i];
        }

        smtc_real_update_cflist( lr1_mac );
    }
    else
    {
        smtc_real_init_after_join_snapshot_channel_mask( lr1_mac );
    }

    lr1_mac->dev_addr = ( lr1_mac->rx_payload[7] + ( lr1_mac->rx_payload[8] << 8 ) + ( lr1_mac->rx_payload[9] << 16 ) +
                          ( lr1_mac->rx_payload[10] << 24 ) );
    lr1_mac->rx1_dr_offset = ( lr1_mac->rx_payload[11] & 0x70 ) >> 4;
    lr1_mac->rx2_data_rate = ( lr1_mac->rx_payload[11] & 0x0F );
    lr1_mac->rx1_delay_s   = ( lr1_mac->rx_payload[12] & 0x0F );
    if( lr1_mac->rx1_delay_s == 0 )
    {
        lr1_mac->rx1_delay_s = 1;  // Lorawan standart define 0 such as a delay of 1
    }

    if( smtc_real_is_rx1_dr_offset_valid( lr1_mac, lr1_mac->rx1_dr_offset ) != OKLORAWAN )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "JoinAccept invalid Rx1DrOffset %d\n", lr1_mac->rx1_dr_offset );
        return ERRORLORAWAN;
    }

    if( smtc_real_is_rx_dr_valid( lr1_mac, lr1_mac->rx2_data_rate ) != OKLORAWAN )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "JoinAccept invalid Rx2Datarate %d\n", lr1_mac->rx2_data_rate );
        return ERRORLORAWAN;
    }

    lr1_mac->join_status = JOINED;

    lr1_stack_mac_session_init( lr1_mac );
    smtc_real_init_session( lr1_mac );

    SMTC_MODEM_HAL_TRACE_PRINTF( " DevAddr= %x\n", lr1_mac->dev_addr );
    SMTC_MODEM_HAL_TRACE_PRINTF( " MacRx1DataRateOffset= %d\n", lr1_mac->rx1_dr_offset );
    SMTC_MODEM_HAL_TRACE_PRINTF( " MacRx2DataRate= %d\n", lr1_mac->rx2_data_rate );
    SMTC_MODEM_HAL_TRACE_PRINTF( " MacRx1Delay= %d\n", lr1_mac->rx1_delay_s );
    SMTC_MODEM_HAL_TRACE_MSG( " Save In Flash After Join succeed\n" );

    return OKLORAWAN;
}

int32_t lr1_stack_network_next_free_duty_cycle_ms_get( lr1_stack_mac_t* lr1_mac )
{
    int32_t time_off_left = 0;

    if( lr1_mac->tx_duty_cycle_time_off_ms > 0 )
    {
        uint32_t delta_t;
        uint32_t rtc_now = smtc_modem_hal_get_time_in_ms( );
        if( rtc_now >= lr1_mac->tx_duty_cycle_timestamp_ms )
        {
            delta_t = rtc_now - lr1_mac->tx_duty_cycle_timestamp_ms;
        }
        else
        {
            delta_t = 0xFFFFFFFFUL - lr1_mac->tx_duty_cycle_timestamp_ms;
            delta_t += rtc_now;
        }

        if( delta_t > lr1_mac->tx_duty_cycle_time_off_ms )
        {
            time_off_left = 0;
        }
        else
        {
            time_off_left = lr1_mac->tx_duty_cycle_time_off_ms - delta_t;
        }
    }
    return time_off_left;
}

uint32_t lr1_stack_toa_get( lr1_stack_mac_t* lr1_mac )
{
    uint32_t toa = 0;

    modulation_type_t tx_modulation_type =
        smtc_real_get_modulation_type_from_datarate( lr1_mac, lr1_mac->tx_data_rate );

    if( tx_modulation_type == LORA )
    {
        uint8_t            tx_sf;
        lr1mac_bandwidth_t tx_bw;
        smtc_real_lora_dr_to_sf_bw( lr1_mac, lr1_mac->tx_data_rate, &tx_sf, &tx_bw );

        ralf_params_lora_t lora_param;
        memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );

        lora_param.mod_params.sf   = ( ral_lora_sf_t ) tx_sf;
        lora_param.mod_params.bw   = ( ral_lora_bw_t ) tx_bw;
        lora_param.mod_params.cr   = smtc_real_get_coding_rate( lr1_mac );
        lora_param.mod_params.ldro = ral_compute_lora_ldro( lora_param.mod_params.sf, lora_param.mod_params.bw );

        lora_param.pkt_params.crc_is_on            = true;
        lora_param.pkt_params.invert_iq_is_on      = false;
        lora_param.pkt_params.pld_len_in_bytes     = lr1_mac->tx_payload_size;
        lora_param.pkt_params.preamble_len_in_symb = smtc_real_get_preamble_len( lr1_mac, lora_param.mod_params.sf );
        lora_param.pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT;

        toa = ral_get_lora_time_on_air_in_ms( ( &lr1_mac->rp->radio->ral ), ( &lora_param.pkt_params ),
                                              ( &lora_param.mod_params ) );
    }
    else if( tx_modulation_type == FSK )
    {
        uint8_t tx_bitrate;
        smtc_real_fsk_dr_to_bitrate( lr1_mac, lr1_mac->tx_data_rate, &tx_bitrate );

        ralf_params_gfsk_t gfsk_param;
        memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

        gfsk_param.rf_freq_in_hz                    = lr1_mac->tx_frequency;
        gfsk_param.sync_word                        = smtc_real_get_gfsk_sync_word( lr1_mac );
        gfsk_param.dc_free_is_on                    = true;
        gfsk_param.whitening_seed                   = GFSK_WHITENING_SEED;
        gfsk_param.crc_seed                         = GFSK_CRC_SEED;
        gfsk_param.crc_polynomial                   = GFSK_CRC_POLYNOMIAL;
        gfsk_param.mod_params.fdev_in_hz            = 25000;
        gfsk_param.mod_params.br_in_bps             = tx_bitrate * 1000;
        gfsk_param.mod_params.bw_dsb_in_hz          = 100000;
        gfsk_param.pkt_params.pld_len_in_bytes      = lr1_mac->tx_payload_size;
        gfsk_param.pkt_params.preamble_len_in_bits  = 40;
        gfsk_param.pkt_params.header_type           = RAL_GFSK_PKT_VAR_LEN;
        gfsk_param.pkt_params.sync_word_len_in_bits = 24;
        gfsk_param.pkt_params.dc_free               = RAL_GFSK_DC_FREE_WHITENING;
        gfsk_param.pkt_params.crc_type              = RAL_GFSK_CRC_2_BYTES_INV;

        toa = ral_get_gfsk_time_on_air_in_ms( ( &lr1_mac->rp->radio->ral ), ( &gfsk_param.pkt_params ),
                                              ( &gfsk_param.mod_params ) );
    }
    else if( tx_modulation_type == LR_FHSS )
    {
        lr_fhss_v1_cr_t tx_cr;
        lr_fhss_v1_bw_t tx_bw;
        smtc_real_lr_fhss_dr_to_cr_bw( lr1_mac, lr1_mac->tx_data_rate, &tx_cr, &tx_bw );

        ralf_params_lr_fhss_t lr_fhss_param;
        memset( &lr_fhss_param, 0, sizeof( ralf_params_lr_fhss_t ) );

        lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.cr             = tx_cr;
        lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.enable_hopping = true;
        lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.header_count   = smtc_real_lr_fhss_get_header_count( tx_cr );

        ral_lr_fhss_get_time_on_air_in_ms( ( &lr1_mac->rp->radio->ral ), &lr_fhss_param.ral_lr_fhss_params,
                                           lr1_mac->tx_payload_size, &toa );
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( "TX MODULATION NOT SUPPORTED\n" );
    }
    return toa;
}

uint8_t lr1_stack_nb_trans_get( lr1_stack_mac_t* lr1_mac )
{
    return ( lr1_mac->nb_trans );
}

status_lorawan_t lr1_stack_nb_trans_set( lr1_stack_mac_t* lr1_mac, uint8_t nb_trans )
{
    if( lr1_mac->adr_mode_select == STATIC_ADR_MODE )
    {
        return ERRORLORAWAN;
    }

    if( ( nb_trans > 0 ) && ( nb_trans < 16 ) )
    {
        lr1_mac->nb_trans = nb_trans;
        return OKLORAWAN;
    }

    return ERRORLORAWAN;
}

uint32_t lr1_stack_get_crystal_error( lr1_stack_mac_t* lr1_mac )
{
    return lr1_mac->crystal_error;
}

void lr1_stack_set_crystal_error( lr1_stack_mac_t* lr1_mac, uint32_t crystal_error )
{
    lr1_mac->crystal_error = crystal_error;
}

/*
 *-----------------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITIONS ------------------------------------------------
 */

static status_lorawan_t lr1_stack_mac_downlink_check_under_it( lr1_stack_mac_t* lr1_mac )
{
    // check Mtype
    uint8_t rx_ftype_tmp = lr1_mac->rx_payload[0] >> 5;
    if( ( rx_ftype_tmp == JOIN_REQUEST ) || ( rx_ftype_tmp == UNCONF_DATA_UP ) || ( rx_ftype_tmp == CONF_DATA_UP ) ||
        ( rx_ftype_tmp == REJOIN_REQUEST ) || ( rx_ftype_tmp == PROPRIETARY ) )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " BAD Ftype = %u for RX Frame\n", rx_ftype_tmp );
        return ERRORLORAWAN;
    }
    // check devaddr
    if( lr1_mac->join_status == JOINED )
    {
        uint32_t dev_addr_tmp = lr1_mac->rx_payload[1] + ( lr1_mac->rx_payload[2] << 8 ) +
                                ( lr1_mac->rx_payload[3] << 16 ) + ( lr1_mac->rx_payload[4] << 24 );

        if( lr1_mac->dev_addr != dev_addr_tmp )
        {
            SMTC_MODEM_HAL_TRACE_INFO( " BAD DevAddr = %x for RX Frame and %x\n\n", lr1_mac->dev_addr, dev_addr_tmp );
            return ERRORLORAWAN;
        }
    }

    return OKLORAWAN;
}

static void mac_header_set( lr1_stack_mac_t* lr1_mac )
{
    lr1_mac->tx_payload[0] = ( ( lr1_mac->tx_mtype & 0x7 ) << 5 ) + ( lr1_mac->tx_major_bits & 0x3 );
}

static void frame_header_set( lr1_stack_mac_t* lr1_mac )
{
    lr1_mac->tx_payload[1] = ( uint8_t )( ( lr1_mac->dev_addr & 0x000000FF ) );
    lr1_mac->tx_payload[2] = ( uint8_t )( ( lr1_mac->dev_addr & 0x0000FF00 ) >> 8 );
    lr1_mac->tx_payload[3] = ( uint8_t )( ( lr1_mac->dev_addr & 0x00FF0000 ) >> 16 );
    lr1_mac->tx_payload[4] = ( uint8_t )( ( lr1_mac->dev_addr & 0xFF000000 ) >> 24 );
    lr1_mac->tx_payload[5] = lr1_mac->tx_fctrl;
    lr1_mac->tx_payload[6] = ( uint8_t )( ( lr1_mac->fcnt_up & 0x000000FF ) );
    lr1_mac->tx_payload[7] = ( uint8_t )( ( lr1_mac->fcnt_up & 0x0000FF00 ) >> 8 );

    if( lr1_mac->tx_fport == PORTNWK )
    {
        lr1_mac->tx_payload[8] = lr1_mac->tx_fport;
    }
    else
    {
        for( int i = 0; i < lr1_mac->tx_fopts_current_length; i++ )
        {
            lr1_mac->tx_payload[8 + i] = lr1_mac->tx_fopts_current_data[i];
        }
        if( lr1_mac->tx_fport_present == true )
        {
            lr1_mac->tx_payload[8 + lr1_mac->tx_fopts_current_length] = lr1_mac->tx_fport;
        }
    }
}

/************************************************************************************************/
/*                    Private NWK MANAGEMENTS Methods */
/************************************************************************************************/
static void link_check_parser( lr1_stack_mac_t* lr1_mac )
{
    if( lr1_mac->nwk_payload_index + LINK_CHECK_ANS_SIZE > lr1_mac->nwk_payload_size )
    {
        lr1_mac->nwk_payload_size = 0;
        return;
    }
    SMTC_MODEM_HAL_TRACE_PRINTF( " Margin = %d, GwCnt = %d\n", lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1],
                                 lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2] );

    lr1_mac->link_check_margin = lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1];
    lr1_mac->link_check_gw_cnt = lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2];

    lr1_mac->nwk_payload_index += LINK_CHECK_ANS_SIZE;
}

/**********************************************************************************************************************/
/*                                               Private NWK MANAGEMENTS :                                            */
/* LinkADR                                                                                                            */
/*  Note : describe multiple adr specification                                                                        */
/*                                                                                                                    */
/*  Step 1 : Create a "unwrapped channel mask" in case of multiple adr cmd with both Channem Mask and ChannnelMaskCntl*/
/*       2 : Extract from the last adr cmd datarate candidate                                                         */
/*       3 : Extract from the last adr cmd TxPower candidate                                                          */
/*       4 : Extract from the last adr cmd NBRetry candidate                                                          */
/*       5 : Check errors cases (described below)                                                                     */
/*       6 : If No error Set new channel mask, txpower,datarate and nbretry                                           */
/*       7 : Compute duplicated LinkAdrAns                                                                            */
/*                                                                                                                    */
/*  Error cases    1 : Channel Cntl mask RFU for each adr cmd (in case of  multiple cmd)                              */
/*                 2 : Undefined channel ( freq = 0 ) for active bit in the unwrapped channel mask                    */
/*                 3 : Unwrapped channel mask = 0 (none active channel)                                               */
/*                 4 : For the last adr cmd not valid tx power                                                        */
/*                 5 : For the last adr cmd not valid datarate                                                        */
/*                     ( datarate > dRMax or datarate < dRMin for all active channel )                                */
/**********************************************************************************************************************/
static void link_adr_parser( lr1_stack_mac_t* lr1_mac, uint8_t nb_link_adr_req )
{
    if( nb_link_adr_req == 0 )
    {
        lr1_mac->nwk_payload_size = 0;
        return;
    }

    for( uint8_t i = 0; i < nb_link_adr_req; i++ )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "%u/%u - Cmd link_adr_parser = %02x %02x %02x %02x\n", i, nb_link_adr_req,
                                     lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( i * LINK_ADR_REQ_SIZE ) + 1],
                                     lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( i * LINK_ADR_REQ_SIZE ) + 2],
                                     lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( i * LINK_ADR_REQ_SIZE ) + 3],
                                     lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( i * LINK_ADR_REQ_SIZE ) + 4] );
    }
    uint8_t status_ans = 0x7;  // initialised for ans answer ok

    // Check channel mask
    for( uint8_t i = 0; i < nb_link_adr_req; i++ )
    {
        uint16_t channel_mask_temp =
            lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( i * LINK_ADR_REQ_SIZE ) + 2] +
            ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( i * LINK_ADR_REQ_SIZE ) + 3] << 8 );

        uint8_t ch_mask_cntl_temp =
            ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( i * LINK_ADR_REQ_SIZE ) + 4] & 0x70 ) >> 4;
        SMTC_MODEM_HAL_TRACE_PRINTF( "%u - MULTIPLE LINK ADR REQ , channel mask = 0x%x , ChMAstCntl = 0x%x\n", i,
                                     channel_mask_temp, ch_mask_cntl_temp );

        if( smtc_real_build_channel_mask( lr1_mac, ch_mask_cntl_temp, channel_mask_temp ) == ERROR_CHANNEL_CNTL )
        {  // Test ChannelCNTL not defined
            status_ans &= 0x6;
            SMTC_MODEM_HAL_TRACE_WARNING( "INVALID CHANNEL CNTL\n" );
        }
    }

    // At This point global temporary channel mask is built and validated
    if( lr1_mac->adr_mode_select != STATIC_ADR_MODE )
    {
        if( status_ans == 7 )  // mean none ERROR_CHANNEL_CNTL or  ERROR_CHANNEL_MASK so valid channel mask only bit
                               // 1 is really tested
        {
            // check if new proposal channel mask is compatible with our mobile distribution
            if( smtc_real_is_channel_mask_for_mobile_mode( lr1_mac ) == ERRORLORAWAN )
            {
                status_ans = 0x0;  // reject the cmd because even if the channel mask is valid it is not compatible with
                                   // our current adr distribution
                SMTC_MODEM_HAL_TRACE_WARNING( "INVALID CHANNEL MASK in Mobile Mode\n" );
            }
            else
            {
                // new channel mask is acceptable :
                smtc_real_set_channel_mask( lr1_mac );
                // set this flag at true to notify upper layer that end device has received a valid link adr
                lr1_mac->available_link_adr = true;
                status_ans                  = 0x1;  // Only the ChMask could be acked, other parameters are discarded
            }
        }
        else  // rejected because channel mask not valid
        {
            status_ans = 0x0;
            SMTC_MODEM_HAL_TRACE_WARNING( "INVALID CHANNEL MASK\n" );
        }
    }
    else  // static mode
    {
        // Valid the last DataRate
        uint8_t dr_tmp =
            ( ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( ( nb_link_adr_req - 1 ) * LINK_ADR_REQ_SIZE ) + 1] &
                0xF0 ) >>
              4 );

        // If datarate requested is 0x0F, ignore the value
        if( dr_tmp != 0x0F )
        {
            if( smtc_real_is_tx_dr_acceptable( lr1_mac, dr_tmp, true ) == ERRORLORAWAN )
            {  // Test Channelmask enables a not defined channel
                status_ans &= 0x5;
                SMTC_MODEM_HAL_TRACE_WARNING( "INVALID DATARATE\n" );
            }
        }

        // Valid the last TxPower  And Prepare Ans
        uint8_t tx_power_tmp =
            ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( ( nb_link_adr_req - 1 ) * LINK_ADR_REQ_SIZE ) + 1] &
              0x0F );
        // If power id is 0x0F, ignore the value
        if( tx_power_tmp != 0x0F )
        {
            if( smtc_real_is_tx_power_valid( lr1_mac, tx_power_tmp ) == ERRORLORAWAN )
            {  // Test tx power
                status_ans &= 0x3;
                SMTC_MODEM_HAL_TRACE_WARNING( "INVALID TXPOWER\n" );
            }
        }

        uint8_t nb_trans_tmp =
            ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( ( nb_link_adr_req - 1 ) * LINK_ADR_REQ_SIZE ) + 4] &
              0x0F );

        // Update the mac parameters if case of no error
        if( status_ans == 0x7 )
        {
            smtc_real_set_channel_mask( lr1_mac );
            // If power id is 0x0F, ignore the value
            if( tx_power_tmp != 0x0F )
            {
                smtc_real_set_power( lr1_mac, tx_power_tmp );
            }
            lr1_mac->nb_trans = ( nb_trans_tmp == 0 ) ? 1 : nb_trans_tmp;
            // If datarate requested is 0x0F, ignore the value
            if( dr_tmp != 0x0F )
            {
                lr1_mac->tx_data_rate_adr = dr_tmp;
            }
            lr1_mac->available_link_adr = true;
            SMTC_MODEM_HAL_TRACE_PRINTF( "MacTxDataRateAdr = %d\n", lr1_mac->tx_data_rate_adr );
            SMTC_MODEM_HAL_TRACE_PRINTF( "MacTxPower = %d\n", lr1_mac->tx_power );
            SMTC_MODEM_HAL_TRACE_PRINTF( "MacNbTrans = %d\n", lr1_mac->nb_trans );
        }
    }

    lr1_mac->nwk_payload_index += ( nb_link_adr_req * LINK_ADR_REQ_SIZE );

    // Prepare repeated Ans
    if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
    {
        for( uint8_t i = 0; i < nb_link_adr_req; i++ )
        {
            lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length + ( i * LINK_ADR_ANS_SIZE )] = LINK_ADR_ANS;  // copy Cid
            lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length + ( i * LINK_ADR_ANS_SIZE ) + 1] = status_ans;
        }
        lr1_mac->tx_fopts_length += ( nb_link_adr_req * LINK_ADR_ANS_SIZE );
    }
}

/**********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * rx_param_setup_parser                       */
/**********************************************************************************************************************/
static void rx_param_setup_parser( lr1_stack_mac_t* lr1_mac )
{
    if( lr1_mac->nwk_payload_index + RXPARRAM_SETUP_REQ_SIZE > lr1_mac->nwk_payload_size )
    {
        lr1_mac->nwk_payload_size = 0;
        return;
    }
    SMTC_MODEM_HAL_TRACE_PRINTF(
        " Cmd rx_param_setup_parser = %x %x %x %x\n", lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2], lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 3],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 4] );

    uint8_t status_ans = 0x7;  // initialised for ans answer ok

    // Valid Rx1DrOffset And Prepare Ans
    uint8_t rx1_dr_offset_temp = ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] & 0x70 ) >> 4;
    if( smtc_real_is_rx1_dr_offset_valid( lr1_mac, rx1_dr_offset_temp ) == ERRORLORAWAN )
    {
        status_ans &= 0x6;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID RX1DROFFSET\n" );
    }

    // Valid MacRx2Dr And Prepare Ans
    uint8_t rx2_dr_temp = ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] & 0x0F );
    if( smtc_real_is_rx_dr_valid( lr1_mac, rx2_dr_temp ) == ERRORLORAWAN )
    {
        status_ans &= 0x5;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID RX2DR\n" );
    }

    // Valid MacRx2Frequency And Prepare Ans
    uint32_t rx2_frequency_temp =
        smtc_real_decode_freq_from_buf( lr1_mac, &lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2] );

    if( smtc_real_is_frequency_valid( lr1_mac, rx2_frequency_temp ) == ERRORLORAWAN )
    {
        status_ans &= 0x3;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID RX2 FREQUENCY\n" );
    }

    // Update the mac parameters if case of no error
    if( status_ans == 0x7 )
    {
        lr1_mac->rx1_dr_offset = rx1_dr_offset_temp;
        lr1_mac->rx2_data_rate = rx2_dr_temp;
        lr1_mac->rx2_frequency = rx2_frequency_temp;
        SMTC_MODEM_HAL_TRACE_PRINTF( "MacRx1DataRateOffset = %d\n", lr1_mac->rx1_dr_offset );
        SMTC_MODEM_HAL_TRACE_PRINTF( "MacRx2DataRate = %d\n", lr1_mac->rx2_data_rate );
        SMTC_MODEM_HAL_TRACE_PRINTF( "MacRx2Frequency = %d\n", lr1_mac->rx2_frequency );
    }

    lr1_mac->nwk_payload_index += RXPARRAM_SETUP_REQ_SIZE;

    // Prepare Ans
    if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
    {
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length]     = RXPARRAM_SETUP_ANS;
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length + 1] = status_ans;
        lr1_mac->tx_fopts_length += RXPARRAM_SETUP_ANS_SIZE;

        if( ( lr1_mac->tx_fopts_lengthsticky + RXPARRAM_SETUP_ANS_SIZE ) <= 15 )  // Max byte in sticky command
        {
            lr1_mac->tx_fopts_datasticky[lr1_mac->tx_fopts_lengthsticky]     = RXPARRAM_SETUP_ANS;
            lr1_mac->tx_fopts_datasticky[lr1_mac->tx_fopts_lengthsticky + 1] = status_ans;
            lr1_mac->tx_fopts_lengthsticky += RXPARRAM_SETUP_ANS_SIZE;
        }
    }
}

/**********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * duty_cycle_parser                          */
/**********************************************************************************************************************/
static void duty_cycle_parser( lr1_stack_mac_t* lr1_mac )
{
    if( lr1_mac->nwk_payload_index + DUTY_CYCLE_REQ_SIZE > lr1_mac->nwk_payload_size )
    {
        lr1_mac->nwk_payload_size = 0;
        return;
    }
    SMTC_MODEM_HAL_TRACE_PRINTF( "Cmd duty_cycle_parser %x\n", lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] );
    lr1_mac->max_duty_cycle_index = ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] & 0x0F );

    lr1_mac->nwk_payload_index += DUTY_CYCLE_REQ_SIZE;
    // Prepare Ans
    if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
    {
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length] = DUTY_CYCLE_ANS;  // copy Cid
        lr1_mac->tx_fopts_length += DUTY_CYCLE_ANS_SIZE;
    }
}
/**********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * dev_status_parser                          */
/**********************************************************************************************************************/

static void dev_status_parser( lr1_stack_mac_t* lr1_mac )
{
    if( lr1_mac->nwk_payload_index + DEV_STATUS_REQ_SIZE > lr1_mac->nwk_payload_size )
    {
        lr1_mac->nwk_payload_size = 0;
        return;
    }
    uint8_t my_hook_id;
    rp_hook_get_id( lr1_mac->rp, lr1_mac, &my_hook_id );
    SMTC_MODEM_HAL_TRACE_MSG( "Receive a dev status req\n" );
    lr1_mac->nwk_payload_index += DEV_STATUS_REQ_SIZE;

    if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
    {
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length]     = DEV_STATUS_ANS;  // copy Cid
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length + 1] = smtc_modem_hal_get_battery_level( );
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length + 2] =
            ( lr1_mac->rp->radio_params[my_hook_id].rx.lora_pkt_status.snr_pkt_in_db ) & 0x3F;
        lr1_mac->tx_fopts_length += DEV_STATUS_ANS_SIZE;
    }
}
/**********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * new_channel_parser                         */
/**********************************************************************************************************************/
static void new_channel_parser( lr1_stack_mac_t* lr1_mac )
{
    if( lr1_mac->nwk_payload_index + NEW_CHANNEL_REQ_SIZE > lr1_mac->nwk_payload_size )
    {
        lr1_mac->nwk_payload_size = 0;
        return;
    }
    SMTC_MODEM_HAL_TRACE_PRINTF(
        " Cmd new_channel_parser = %x %x %x %x %x\n", lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2], lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 3],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 4], lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 5] );

    if( !smtc_real_is_new_channel_req_supported( lr1_mac ) )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "NewChannelReq is not supported for this region\n" );
        lr1_mac->nwk_payload_index += NEW_CHANNEL_REQ_SIZE;
        return;
    }

    uint8_t status_ans = 0x3;  // initialized for ans answer ok

    // Valid Channel Index
    uint8_t channel_index_temp = lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1];
    if( smtc_real_is_channel_index_valid( lr1_mac, channel_index_temp ) == ERRORLORAWAN )
    {
        status_ans &= 0x0;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID CHANNEL INDEX\n" );
    }

    // Valid Frequency
    uint32_t frequency_temp =
        smtc_real_decode_freq_from_buf( lr1_mac, &lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2] );
    if( smtc_real_is_nwk_received_tx_frequency_valid( lr1_mac, frequency_temp ) == ERRORLORAWAN )
    {
        status_ans &= 0x2;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID FREQUENCY\n" );
    }

    // Valid DRMIN/MAX
    uint8_t dr_range_min_temp = lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 5] & 0xF;
    if( smtc_real_is_tx_dr_valid( lr1_mac, dr_range_min_temp ) == ERRORLORAWAN )
    {
        status_ans &= 0x1;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID DR MIN\n" );
    }

    uint8_t dr_range_max_temp = ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 5] & 0xF0 ) >> 4;
    if( smtc_real_is_tx_dr_valid( lr1_mac, dr_range_max_temp ) == ERRORLORAWAN )
    {
        status_ans &= 0x1;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID DR MAX\n" );
    }

    if( dr_range_max_temp < dr_range_min_temp )
    {
        status_ans &= 0x1;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID DR MAX < DR MIN\n" );
    }

    // Update the mac parameters if there is no error
    if( status_ans == 0x3 )
    {
        smtc_real_set_tx_frequency_channel( lr1_mac, frequency_temp, channel_index_temp );
        smtc_real_set_rx1_frequency_channel( lr1_mac, frequency_temp, channel_index_temp );

        smtc_real_set_channel_dr( lr1_mac, channel_index_temp, dr_range_min_temp, dr_range_max_temp );
        if( frequency_temp == 0 )
        {
            smtc_real_set_channel_enabled( lr1_mac, CHANNEL_DISABLED, channel_index_temp );
        }
        else
        {
            smtc_real_set_channel_enabled( lr1_mac, CHANNEL_ENABLED, channel_index_temp );
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "MacTxFrequency [ %d ] = %d, DrMin = %d, DrMax = %d\n", channel_index_temp,
                                     smtc_real_get_tx_channel_frequency( lr1_mac, channel_index_temp ),
                                     dr_range_min_temp, dr_range_max_temp );
    }

    lr1_mac->nwk_payload_index += NEW_CHANNEL_REQ_SIZE;

    // Prepare Ans
    if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
    {
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length]     = NEW_CHANNEL_ANS;  // copy Cid
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length + 1] = status_ans;
        lr1_mac->tx_fopts_length += NEW_CHANNEL_ANS_SIZE;
    }
}
/*********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * rx_timing_setup_parser                     */
/*********************************************************************************************************************/

static void rx_timing_setup_parser( lr1_stack_mac_t* lr1_mac )
{
    if( lr1_mac->nwk_payload_index + RXTIMING_SETUP_REQ_SIZE > lr1_mac->nwk_payload_size )
    {
        lr1_mac->nwk_payload_size = 0;
        return;
    }
    SMTC_MODEM_HAL_TRACE_PRINTF( "Cmd rx_timing_setup_parser = %x\n",
                                 lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] );
    lr1_mac->rx1_delay_s = ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] & 0xF );
    if( lr1_mac->rx1_delay_s == 0 )
    {
        lr1_mac->rx1_delay_s = 1;  // Lorawan standart define 0 such as a delay of 1
    }

    lr1_mac->nwk_payload_index += RXTIMING_SETUP_REQ_SIZE;

    // Prepare Ans
    if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
    {
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length] = RXTIMING_SETUP_ANS;
        lr1_mac->tx_fopts_length += RXTIMING_SETUP_ANS_SIZE;

        if( ( lr1_mac->tx_fopts_lengthsticky + RXTIMING_SETUP_ANS_SIZE ) <= 15 )  // Max byte in sticky command
        {
            lr1_mac->tx_fopts_datasticky[lr1_mac->tx_fopts_lengthsticky] = RXTIMING_SETUP_ANS;
            lr1_mac->tx_fopts_lengthsticky += RXTIMING_SETUP_ANS_SIZE;
        }
    }
}

/*********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * tx_param_setup_parser                  */
/*********************************************************************************************************************/

static void tx_param_setup_parser( lr1_stack_mac_t* lr1_mac )
{
    if( lr1_mac->nwk_payload_index + TXPARAM_SETUP_REQ_SIZE > lr1_mac->nwk_payload_size )
    {
        lr1_mac->nwk_payload_size = 0;
        return;
    }
    SMTC_MODEM_HAL_TRACE_PRINTF( "Cmd tx_param_setup_parser = %x\n",
                                 lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] );

    if( !smtc_real_is_tx_param_setup_req_supported( lr1_mac ) == true )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TxParamSetupReq is not supported for this region\n" );
        lr1_mac->nwk_payload_index += TXPARAM_SETUP_REQ_SIZE;
        return;
    }

    uint8_t max_erp_dbm_tmp =
        smtc_real_max_eirp_dbm_from_idx[( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] & 0x0F )] - 2;

    lr1_mac->max_erp_dbm         = ( max_erp_dbm_tmp > const_tx_power_dbm ) ? const_tx_power_dbm : max_erp_dbm_tmp;
    lr1_mac->uplink_dwell_time   = ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] & 0x10 ) >> 4;
    lr1_mac->downlink_dwell_time = ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] & 0x20 ) >> 5;

    lr1_mac->nwk_payload_index += TXPARAM_SETUP_REQ_SIZE;

    // Prepare Ans
    if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
    {
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length] = TXPARAM_SETUP_ANS;
        lr1_mac->tx_fopts_length += TXPARAM_SETUP_ANS_SIZE;

        if( ( lr1_mac->tx_fopts_lengthsticky + TXPARAM_SETUP_ANS_SIZE ) <= 15 )  // Max byte in sticky command
        {
            lr1_mac->tx_fopts_datasticky[lr1_mac->tx_fopts_lengthsticky] = TXPARAM_SETUP_ANS;  // copy Cid
            lr1_mac->tx_fopts_lengthsticky += TXPARAM_SETUP_ANS_SIZE;
        }
    }
}

/*********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * dl_channel_parser                        */
/*********************************************************************************************************************/

static void dl_channel_parser( lr1_stack_mac_t* lr1_mac )
{
    if( lr1_mac->nwk_payload_index + DL_CHANNEL_REQ_SIZE > lr1_mac->nwk_payload_size )
    {
        lr1_mac->nwk_payload_size = 0;
        return;
    }
    SMTC_MODEM_HAL_TRACE_PRINTF(
        "Cmd dl_channel_parser = %x %x %x %x \n", lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2], lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 3],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 4] );

    if( !smtc_real_is_new_channel_req_supported( lr1_mac ) )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "DlChannelReq is not supported for this region\n" );
        lr1_mac->nwk_payload_index += DL_CHANNEL_REQ_SIZE;
        return;
    }

    uint8_t status_ans = 0x3;  // initialised for ans answer ok

    // Valid Channel Index
    uint8_t channel_index_temp = lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1];
    if( smtc_real_get_tx_channel_frequency( lr1_mac, channel_index_temp ) == 0 )
    {
        status_ans &= 0x1;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID CHANNEL INDEX\n" );
    }
    // Valid Frequency
    uint32_t frequency_temp =
        smtc_real_decode_freq_from_buf( lr1_mac, &lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2] );
    if( smtc_real_is_frequency_valid( lr1_mac, frequency_temp ) == ERRORLORAWAN )
    {
        status_ans &= 0x2;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID FREQUENCY\n" );
    }

    // Update the mac parameters if case of no error
    if( status_ans == 0x3 )
    {
        if( smtc_real_set_rx1_frequency_channel( lr1_mac, frequency_temp, channel_index_temp ) != OKLORAWAN )
        {
            status_ans = 0;
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "MacRx1Frequency [ %u ] = %d\n", channel_index_temp,
                                     smtc_real_get_rx1_channel_frequency( lr1_mac, channel_index_temp ) );
    }

    lr1_mac->nwk_payload_index += DL_CHANNEL_REQ_SIZE;

    // Prepare Ans
    if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
    {
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length]     = DL_CHANNEL_ANS;
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length + 1] = status_ans;
        lr1_mac->tx_fopts_length += DL_CHANNEL_ANS_SIZE;

        if( ( lr1_mac->tx_fopts_lengthsticky + DL_CHANNEL_ANS_SIZE ) <= 15 )  // Max byte in sticky command
        {
            lr1_mac->tx_fopts_datasticky[lr1_mac->tx_fopts_lengthsticky]     = DL_CHANNEL_ANS;
            lr1_mac->tx_fopts_datasticky[lr1_mac->tx_fopts_lengthsticky + 1] = status_ans;
            lr1_mac->tx_fopts_lengthsticky += DL_CHANNEL_ANS_SIZE;
        }
    }
}

status_lorawan_t lr1mac_rx_payload_max_size_check( lr1_stack_mac_t* lr1_mac, uint8_t size, uint8_t rx_datarate )
{
    uint8_t size_max =
        smtc_real_get_max_payload_size( lr1_mac, rx_datarate, lr1_mac->downlink_dwell_time ) + 1 + 4;  // + MHDR + CRC
    if( size > size_max )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "size (%d)> size_max (%d)", size, size_max );
        return ERRORLORAWAN;
    }

    return OKLORAWAN;
}
/*********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * device_time_ans_parser                        */
/*********************************************************************************************************************/

static status_lorawan_t device_time_ans_parser( lr1_stack_mac_t* lr1_mac )
{
    if( lr1_mac->nwk_payload_index + DEVICE_TIME_ANS_SIZE > lr1_mac->nwk_payload_size )
    {
        lr1_mac->nwk_payload_size = 0;
        return ERRORLORAWAN;
    }
    SMTC_MODEM_HAL_TRACE_PRINTF(
        "Cmd device_time_ans_parser = %x %x %x %x %x\n", lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2], lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 3],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 4], lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 5] );

    lr1_mac->seconds_since_epoch = lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1];
    lr1_mac->seconds_since_epoch |= ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2] << 8 );
    lr1_mac->seconds_since_epoch |= ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 3] << 16 );
    lr1_mac->seconds_since_epoch |= ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 4] << 24 );

    lr1_mac->fractional_second = ( uint32_t )( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 5] * 1000 ) >> 8;

    SMTC_MODEM_HAL_TRACE_PRINTF( "SecondsSinceEpoch %u, FractionalSecond %u\n", lr1_mac->seconds_since_epoch,
                                 lr1_mac->fractional_second );

    lr1_mac->nwk_payload_index += DEVICE_TIME_ANS_SIZE;

    return OKLORAWAN;
}

/*********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * beacon_freq_req_parser                        */
/*********************************************************************************************************************/

static void beacon_freq_req_parser( lr1_stack_mac_t* lr1_mac )
{
    if( lr1_mac->nwk_payload_index + BEACON_FREQ_REQ_SIZE > lr1_mac->nwk_payload_size )
    {
        lr1_mac->nwk_payload_size = 0;
        return;
    }
    SMTC_MODEM_HAL_TRACE_PRINTF(
        "Cmd beacon_freq_req_parser = %x %x %x %x\n", lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2], lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 3],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 4] );

    uint8_t status_ans = 0x1;  // initialized for ans answer ok

    // Valid Frequency
    uint32_t frequency_temp =
        smtc_real_decode_freq_from_buf( lr1_mac, &lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] );

    // A frequency of 0 instructs the end-device that it SHALL use the default frequency plan
    if( frequency_temp != 0 )
    {
        if( smtc_real_is_frequency_valid( lr1_mac, frequency_temp ) == ERRORLORAWAN )
        {
            status_ans &= 0x0;
            SMTC_MODEM_HAL_TRACE_MSG( "INVALID BEACON FREQUENCY\n" );
        }
    }

    // Update the mac parameters if case of no error
    if( status_ans == 0x1 )
    {
        lr1_mac->beacon_freq_hz = frequency_temp;
        SMTC_MODEM_HAL_TRACE_PRINTF( "MacBeaconFrequency %d\n", ( lr1_mac->beacon_freq_hz != 0 )
                                                                    ? lr1_mac->beacon_freq_hz
                                                                    : smtc_real_get_beacon_frequency(
                                                                          lr1_mac, smtc_modem_hal_get_time_in_ms( ) ) );
    }

    lr1_mac->nwk_payload_index += BEACON_FREQ_REQ_SIZE;

    // Prepare Ans
    if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
    {
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length]     = BEACON_FREQ_ANS;  // copy Cid
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length + 1] = status_ans;
        lr1_mac->tx_fopts_length += BEACON_FREQ_ANS_SIZE;
    }
}

/*********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * ping_slot_channel_req_parser                        */
/*********************************************************************************************************************/
static void ping_slot_channel_req_parser( lr1_stack_mac_t* lr1_mac )
{
    if( lr1_mac->nwk_payload_index + PING_SLOT_CHANNEL_REQ_SIZE > lr1_mac->nwk_payload_size )
    {
        lr1_mac->nwk_payload_size = 0;
        return;
    }
    SMTC_MODEM_HAL_TRACE_PRINTF(
        " Cmd ping_slot_channel_req_parser = %x %x %x %x\n", lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2], lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 3],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 4] );

    uint8_t status_ans = 0x3;  // initialised for ans answer ok

    // Valid Frequency And Prepare Ans
    uint32_t frequency_temp =
        smtc_real_decode_freq_from_buf( lr1_mac, &lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] );

    // A frequency of 0 instructs the end-device that it SHALL use the default frequency plan
    if( frequency_temp != 0 )
    {
        if( smtc_real_is_frequency_valid( lr1_mac, frequency_temp ) == ERRORLORAWAN )
        {
            status_ans &= 0x2;
            SMTC_MODEM_HAL_TRACE_MSG( "INVALID PING SLOT FREQUENCY\n" );
        }
    }

    // Valid Datarate And Prepare Ans
    uint8_t dr_temp = lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 4] & 0x0F;
    if( smtc_real_is_rx_dr_valid( lr1_mac, dr_temp ) == ERRORLORAWAN )
    {
        status_ans &= 0x01;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID PING SLOT DR\n" );
    }

    // Update the mac parameters if case of no error
    if( status_ans == 0x3 )
    {
        lr1_mac->ping_slot_freq_hz = frequency_temp;
        lr1_mac->ping_slot_dr      = dr_temp;

        SMTC_MODEM_HAL_TRACE_PRINTF( "MacPingSlotDataRate = %d\n", lr1_mac->ping_slot_dr );
        SMTC_MODEM_HAL_TRACE_PRINTF( "MacPingSlotFrequency = %d\n", lr1_mac->ping_slot_freq_hz );
    }

    lr1_mac->nwk_payload_index += PING_SLOT_CHANNEL_REQ_SIZE;

    // Prepare Ans
    if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
    {
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length]     = PING_SLOT_CHANNEL_ANS;
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length + 1] = status_ans;
        lr1_mac->tx_fopts_length += PING_SLOT_CHANNEL_ANS_SIZE;

        if( ( lr1_mac->tx_fopts_lengthsticky + PING_SLOT_CHANNEL_ANS_SIZE ) <= 15 )  // Max byte in sticky command
        {
            lr1_mac->tx_fopts_datasticky[lr1_mac->tx_fopts_lengthsticky]     = PING_SLOT_CHANNEL_ANS;
            lr1_mac->tx_fopts_datasticky[lr1_mac->tx_fopts_lengthsticky + 1] = status_ans;
            lr1_mac->tx_fopts_lengthsticky += PING_SLOT_CHANNEL_ANS_SIZE;
        }
    }
}

/*********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * ping_slot_info_ans_parser                        */
/*********************************************************************************************************************/

static status_lorawan_t ping_slot_info_ans_parser( lr1_stack_mac_t* lr1_mac )
{
    if( lr1_mac->nwk_payload_index + PING_SLOT_INFO_ANS_SIZE > lr1_mac->nwk_payload_size )
    {
        lr1_mac->nwk_payload_size = 0;
        return ERRORLORAWAN;
    }

    SMTC_MODEM_HAL_TRACE_PRINTF( " PingSlotInfoAns\n" );
    lr1_mac->nwk_payload_index += PING_SLOT_INFO_ANS_SIZE;
    return OKLORAWAN;
}

/* --- EOF ------------------------------------------------------------------ */
