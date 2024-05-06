/*!
 * \file    wake_on_radio_ral.c
 *
 * \brief   WOR and WOR ACK radio function
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

/*
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */

#include "lr1mac_defs.h"
#include "ral.h"
#include "ralf_defs.h"
#include "ralf.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_modem_hal.h"
#include "smtc_real.h"
#include "wake_on_radio_ral.h"
#include "radio_planner_stats.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE VARIABLES -------------------------------------------------------------
 */

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DECLARATION -------------------------------------------------
 */
/**
 * @brief Function called by RP to send WOR and WOR ACK
 *
 * @param rp_void radio planner pointer
 */
static void wor_ral_callback_start_tx( void* rp_void );

/**
 * @brief Function called by RP to receive WOR, WOR ACK and LoRaWAN uplink
 *
 * @param rp_void
 */
static void wor_ral_callback_start_rx( void* rp_void );

/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS DEFINITIONS --------------------------------------------------
 */

bool wor_schedule_tx_wor( uint8_t hook_id, radio_planner_t* rp, wor_tx_param_t* wor_param )
{
    rp_radio_params_t   param = { .pkt_type = RAL_PKT_TYPE_LORA };
    ralf_params_lora_t* lora  = &( param.tx.lora );

    lora->symb_nb_timeout   = 8;
    lora->sync_word         = wor_param->sync_word;
    lora->rf_freq_in_hz     = wor_param->freq_hz;
    lora->output_pwr_in_dbm = wor_param->output_pwr_in_dbm;

    lora->mod_params.cr   = wor_param->cr;
    lora->mod_params.sf   = ( ral_lora_sf_t ) wor_param->sf;
    lora->mod_params.bw   = ( ral_lora_bw_t ) wor_param->bw;
    lora->mod_params.ldro = ral_compute_lora_ldro( lora->mod_params.sf, lora->mod_params.bw );

    lora->pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT;
    lora->pkt_params.pld_len_in_bytes     = wor_param->payload_len;
    lora->pkt_params.crc_is_on            = true;
    lora->pkt_params.invert_iq_is_on      = true;
    lora->pkt_params.preamble_len_in_symb = wor_param->preamble_len_symb;

    const uint32_t toa_wor_ms =
        ral_get_lora_time_on_air_in_ms( &rp->radio->ral, &param.tx.lora.pkt_params, &param.tx.lora.mod_params );

    const rp_task_t rp_task = {
        .hook_id               = hook_id,
        .launch_task_callbacks = wor_ral_callback_start_tx,
        .type                  = RP_TASK_TYPE_TX_LORA,
        .duration_time_ms      = toa_wor_ms,
        .start_time_ms         = wor_param->start_time_ms,
        .state                 = ( wor_param->wor_at_time == true ) ? RP_TASK_STATE_SCHEDULE : RP_TASK_STATE_ASAP,
    };

    if( rp_task_enqueue( rp, &rp_task, wor_param->payload, wor_param->payload_len, &param ) == RP_HOOK_STATUS_OK )
    {
        return true;
    }

    SMTC_MODEM_HAL_TRACE_PRINTF( "RP is busy (smtc_relay_tx_send_wor)\n" );
    return false;
}

void wor_ral_init_tx_ack( smtc_real_t* real, uint8_t dr, uint32_t freq_hz, uint8_t payload_len,
                          rp_radio_params_t* param )
{
    const modulation_type_t modulation_type = smtc_real_get_modulation_type_from_datarate( real, dr );
    if( modulation_type == LORA )
    {
        uint8_t             sf;
        lr1mac_bandwidth_t  bw;
        ralf_params_lora_t* lora = &param->tx.lora;

        smtc_real_lora_dr_to_sf_bw( real, dr, &sf, &bw );

        lora->rf_freq_in_hz = freq_hz;

        lora->output_pwr_in_dbm = smtc_real_clamp_output_power_eirp_vs_freq_and_dr(
            real, smtc_real_get_default_max_eirp( real ), freq_hz, dr );
        lora->sync_word                       = smtc_real_get_sync_word( real );
        lora->mod_params.sf                   = ( ral_lora_sf_t ) sf;
        lora->mod_params.bw                   = ( ral_lora_bw_t ) bw;
        lora->mod_params.cr                   = smtc_real_get_coding_rate( real );
        lora->mod_params.ldro                 = ral_compute_lora_ldro( lora->mod_params.sf, lora->mod_params.bw );
        lora->pkt_params.preamble_len_in_symb = smtc_real_get_preamble_len( real, lora->mod_params.sf );
        lora->pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT;
        lora->pkt_params.pld_len_in_bytes     = payload_len;
        lora->pkt_params.crc_is_on            = true;
        lora->pkt_params.invert_iq_is_on      = true;

        param->pkt_type = RAL_PKT_TYPE_LORA;
    }
    else
    {
        SMTC_MODEM_HAL_PANIC( "TX MODULATION NOT SUPPORTED\n" );
    }
}

bool wor_schedule_rx_wor_ack( uint8_t hook_id, radio_planner_t* rp, wor_ack_rx_param_t* wor_ack_param )
{
    rp_radio_params_t   param = { 0 };
    ralf_params_lora_t* lora  = &( param.rx.lora );
    lora->sync_word           = wor_ack_param->sync_word;
    lora->symb_nb_timeout     = 10;
    lora->rf_freq_in_hz       = wor_ack_param->freq_hz;

    lora->mod_params.cr   = wor_ack_param->cr;
    lora->mod_params.sf   = ( ral_lora_sf_t ) wor_ack_param->sf;
    lora->mod_params.bw   = ( ral_lora_bw_t ) wor_ack_param->bw;
    lora->mod_params.ldro = ral_compute_lora_ldro( lora->mod_params.sf, lora->mod_params.bw );

    lora->pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT;
    lora->pkt_params.pld_len_in_bytes     = wor_ack_param->payload_len;
    lora->pkt_params.crc_is_on            = true;
    lora->pkt_params.invert_iq_is_on      = true;
    lora->pkt_params.preamble_len_in_symb = wor_ack_param->preamble_len_in_symb;

    param.pkt_type         = RAL_PKT_TYPE_LORA;
    param.rx.timeout_in_ms = wor_ack_param->toa;

    const rp_task_t rp_task = {
        .hook_id               = hook_id,
        .state                 = RP_TASK_STATE_SCHEDULE,
        .type                  = RP_TASK_TYPE_RX_LORA,
        .start_time_ms         = wor_ack_param->start_time_ms,
        .duration_time_ms      = wor_ack_param->toa,
        .launch_task_callbacks = wor_ral_callback_start_rx,
    };

    if( rp_task_enqueue( rp, &rp_task, wor_ack_param->payload, 255, &param ) == RP_HOOK_STATUS_OK )
    {
        return true;
    }
    SMTC_MODEM_HAL_TRACE_MSG( "RP is busy (relay_tx_receive_ack)\n" );
    return false;
}

static void wor_ral_callback_start_tx( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;

    // No need to manage FSK as WOR & WOR ACK are LORA only
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ralf_setup_lora( rp->radio, &rp->radio_params[id].tx.lora ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_TX_DONE ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
        ral_set_pkt_payload( &( rp->radio->ral ), rp->payload[id], rp->payload_buffer_size[id] ) == RAL_STATUS_OK );

    // Wait the exact time
    while( ( int32_t )( rp->tasks[id].start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) >= 0 )
    {
        // Do nothing
    }
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_set_ant_switch( true );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_tx( &( rp->radio->ral ) ) == RAL_STATUS_OK );
    rp_stats_set_tx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
}

static void wor_ral_callback_start_rx( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;

    if( rp->radio_params[id].pkt_type == RAL_PKT_TYPE_LORA )
    {
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ralf_setup_lora( rp->radio, &rp->radio_params[id].rx.lora ) == RAL_STATUS_OK );
    }
    else if( rp->radio_params[id].pkt_type == RAL_PKT_TYPE_GFSK )
    {
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ralf_setup_gfsk( rp->radio, &rp->radio_params[id].rx.gfsk ) == RAL_STATUS_OK );
    }
    else
    {
        SMTC_MODEM_HAL_PANIC( );
    }

    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
        ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
                                                         RAL_IRQ_RX_CRC_ERROR ) == RAL_STATUS_OK );

    // Wait the exact time
    while( ( int32_t )( rp->tasks[id].start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) > 0 )
    {
    }
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_set_ant_switch( false );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_rx( &( rp->radio->ral ), rp->radio_params[id].rx.timeout_in_ms ) ==
                                     RAL_STATUS_OK );

    rp_stats_set_rx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
}
