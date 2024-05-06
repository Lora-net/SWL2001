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

#include "wake_on_radio.h"
#include "wake_on_radio_ral.h"

#include "lr1mac_utilities.h"

#include "smtc_modem_hal_dbg_trace.h"
#include <string.h>

void wor_ral_init_tx_wor( smtc_real_t* real, uint8_t dr, uint32_t freq_hz, uint16_t preamble_len_symb,
                          uint8_t payload_len, rp_radio_params_t* param )
{
    // assert_param( param != NULL );

    modulation_type_t modulation_type = smtc_real_get_modulation_type_from_datarate( real, dr );

    if( modulation_type == LORA )
    {
        uint8_t             sf;
        lr1mac_bandwidth_t  bw;
        ralf_params_lora_t* lora = &param->tx.lora;

        smtc_real_lora_dr_to_sf_bw( real, dr, &sf, &bw );

        lora->sync_word         = smtc_real_get_sync_word( real );
        lora->symb_nb_timeout   = 8;
        lora->rf_freq_in_hz     = freq_hz;
        lora->output_pwr_in_dbm = smtc_real_clamp_output_power_eirp_vs_freq_and_dr(
            real, smtc_real_get_default_max_eirp( real ), freq_hz, dr );

        lora->mod_params.cr   = smtc_real_get_coding_rate( real );
        lora->mod_params.sf   = ( ral_lora_sf_t ) sf;
        lora->mod_params.bw   = ( ral_lora_bw_t ) bw;
        lora->mod_params.ldro = ral_compute_lora_ldro( lora->mod_params.sf, lora->mod_params.bw );

        lora->pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT;
        lora->pkt_params.pld_len_in_bytes     = payload_len;
        lora->pkt_params.crc_is_on            = true;
        lora->pkt_params.invert_iq_is_on      = true;
        lora->pkt_params.preamble_len_in_symb = preamble_len_symb;

        param->pkt_type = RAL_PKT_TYPE_LORA;
    }
    else
    {
        SMTC_MODEM_HAL_PANIC( "MODULATION NOT SUPPORTED\n" );
    }
}

void wor_ral_init_rx_wor( smtc_real_t* real, uint8_t dr, uint32_t freq_hz, wor_cad_periodicity_t cad_period,
                          uint8_t max_payload, rp_radio_params_t* param )
{
    const modulation_type_t modulation_type = smtc_real_get_modulation_type_from_datarate( real, dr );
    if( modulation_type == LORA )
    {
        uint8_t             sf;
        lr1mac_bandwidth_t  bw;
        ralf_params_lora_t* lora = &param->rx.lora;

        smtc_real_lora_dr_to_sf_bw( real, dr, &sf, &bw );

        const uint32_t symb_time_us =
            lr1mac_utilities_get_symb_time_us( 1, ( ral_lora_sf_t ) sf, ( ral_lora_bw_t ) bw );

        lora->symb_nb_timeout = 10;
        lora->sync_word       = smtc_real_get_sync_word( real );
        lora->rf_freq_in_hz   = freq_hz;

        lora->mod_params.cr   = smtc_real_get_coding_rate( real );
        lora->mod_params.sf   = ( ral_lora_sf_t ) sf;
        lora->mod_params.bw   = ( ral_lora_bw_t ) bw;
        lora->mod_params.ldro = ral_compute_lora_ldro( lora->mod_params.sf, lora->mod_params.bw );

        lora->pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT;
        lora->pkt_params.pld_len_in_bytes     = max_payload;
        lora->pkt_params.crc_is_on            = true;
        lora->pkt_params.invert_iq_is_on      = true;
        lora->pkt_params.preamble_len_in_symb = wor_convert_cad_period_in_ms( cad_period ) * 1000 / symb_time_us + 30;

        param->pkt_type = RAL_PKT_TYPE_LORA;
    }
    else
    {
        SMTC_MODEM_HAL_PANIC( "MODULATION NOT SUPPORTED\n" );
    }
}

void wor_ral_init_cad( smtc_real_t* real, uint8_t dr, wor_cad_periodicity_t cad_period, bool is_first,
                       uint32_t wor_toa_ms, ral_lora_cad_params_t* param )
{
    uint8_t            sf;
    lr1mac_bandwidth_t bw;

    smtc_real_lora_dr_to_sf_bw( real, dr, &sf, &bw );

#if defined( LR11XX_TRANSCEIVER )

    const uint8_t det_peak_bw500[] = { 65, 70, 77, 85, 78, 80, 79, 82 };
    const uint8_t det_peak_bw250[] = { 60, 61, 64, 72, 63, 71, 73, 75 };
    const uint8_t det_peak_bw125[] = { 56, 52, 52, 58, 58, 62, 66, 68 };
#else
    // following value from "SX126X CAD performance evaluation V2_1.pdf"
    const uint8_t det_peak_bw125[] = { 21, 22, 22, 22, 23, 24, 25, 28 };
    const uint8_t det_peak_bw250[] = { 21, 22, 22, 22, 23, 24, 25, 28 };
    const uint8_t det_peak_bw500[] = { 21, 22, 22, 23, 23, 24, 26, 30 };
#endif

    if( ( bw == BW500 ) && ( sf >= 5 ) )
    {
        param->cad_det_peak_in_symb = det_peak_bw500[sf - 5];
    }
    else if( ( bw == BW250 ) && ( sf >= 5 ) )
    {
        param->cad_det_peak_in_symb = det_peak_bw250[sf - 5];
    }
    else if( ( bw == BW125 ) && ( sf >= 5 ) )
    {
        param->cad_det_peak_in_symb = det_peak_bw125[sf - 5];
    }
    else
    {
        SMTC_MODEM_HAL_PANIC( "DR%d SF%d BW%d \n", dr, sf, bw );
    }

    param->cad_det_min_in_symb = 10;
    param->cad_timeout_in_ms   = wor_convert_cad_period_in_ms( cad_period ) + wor_toa_ms;
    param->cad_timeout_in_ms += ( param->cad_timeout_in_ms >> 4 );  // Add a bit of margin

    if( is_first == true )
    {
        param->cad_exit_mode = RAL_LORA_CAD_ONLY;
        param->cad_symb_nb   = RAL_LORA_CAD_01_SYMB;
    }
    else
    {
        param->cad_exit_mode = RAL_LORA_CAD_RX;

        if( bw == BW500 )
        {
            param->cad_symb_nb = ( sf == RAL_LORA_SF12 ) ? RAL_LORA_CAD_08_SYMB : RAL_LORA_CAD_04_SYMB;
        }
        else if( bw == BW250 )
        {
            param->cad_symb_nb = ( sf <= RAL_LORA_SF8 ) ? RAL_LORA_CAD_08_SYMB : RAL_LORA_CAD_04_SYMB;
        }
        else if( bw == BW125 )
        {
            param->cad_symb_nb = ( sf <= RAL_LORA_SF8 ) ? RAL_LORA_CAD_02_SYMB : RAL_LORA_CAD_04_SYMB;
        }
    }
}

void wor_ral_init_rx_msg( smtc_real_t* real, uint8_t max_payload, uint8_t dr, uint32_t freq_hz,
                          rp_radio_params_t* param )
{
    modulation_type_t modulation_type = smtc_real_get_modulation_type_from_datarate( real, dr );
    if( modulation_type == LORA )
    {
        uint8_t             sf;
        lr1mac_bandwidth_t  bw;
        ralf_params_lora_t* lora = &param->rx.lora;

        smtc_real_lora_dr_to_sf_bw( real, dr, &sf, &bw );

        const uint32_t symb_time_us =
            lr1mac_utilities_get_symb_time_us( 1, ( ral_lora_sf_t ) sf, ( ral_lora_bw_t ) bw );

        lora->sync_word       = smtc_real_get_sync_word( real );
        lora->symb_nb_timeout = 10 + 1000 * smtc_modem_hal_get_radio_tcxo_startup_delay_ms( ) / symb_time_us;
        lora->rf_freq_in_hz   = freq_hz;

        lora->pkt_params.pld_len_in_bytes = max_payload;
        lora->mod_params.cr               = smtc_real_get_coding_rate( real );
        lora->mod_params.sf               = ( ral_lora_sf_t ) sf;
        lora->mod_params.bw               = ( ral_lora_bw_t ) bw;
        lora->mod_params.ldro             = ral_compute_lora_ldro( lora->mod_params.sf, lora->mod_params.bw );

        lora->pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT;
        lora->pkt_params.crc_is_on            = true;
        lora->pkt_params.invert_iq_is_on      = false;
        lora->pkt_params.preamble_len_in_symb = smtc_real_get_preamble_len( real, lora->mod_params.sf );

        param->pkt_type = RAL_PKT_TYPE_LORA;
    }
    else if( modulation_type == FSK )
    {
        ralf_params_gfsk_t* gfsk = &param->rx.gfsk;
        uint8_t             tx_bitrate;
        smtc_real_fsk_dr_to_bitrate( real, dr, &tx_bitrate );

        gfsk->sync_word      = smtc_real_get_gfsk_sync_word( real );
        gfsk->crc_seed       = GFSK_CRC_SEED;
        gfsk->crc_polynomial = GFSK_CRC_POLYNOMIAL;
        gfsk->rf_freq_in_hz  = freq_hz;

        gfsk->pkt_params.pld_len_in_bytes      = 255;
        gfsk->pkt_params.header_type           = RAL_GFSK_PKT_VAR_LEN;
        gfsk->pkt_params.preamble_len_in_bits  = 40;
        gfsk->pkt_params.sync_word_len_in_bits = 24;
        gfsk->pkt_params.dc_free               = RAL_GFSK_DC_FREE_WHITENING;
        gfsk->pkt_params.crc_type              = RAL_GFSK_CRC_2_BYTES_INV;

        gfsk->mod_params.fdev_in_hz   = 25000;
        gfsk->mod_params.bw_dsb_in_hz = 100000;
        gfsk->mod_params.pulse_shape  = RAL_GFSK_PULSE_SHAPE_BT_1;
        gfsk->mod_params.br_in_bps    = tx_bitrate * 1000;

        param->pkt_type = RAL_PKT_TYPE_GFSK;
    }
    else
    {
        SMTC_MODEM_HAL_PANIC( "MODULATION NOT SUPPORTED\n" );
    }
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

void wor_ral_callback_start_cad( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    const uint8_t    id = rp->radio_task_id;

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ralf_setup_lora( rp->radio, &rp->radio_params[id].rx.lora ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
        ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_CAD_DONE | RAL_IRQ_CAD_OK | RAL_IRQ_RX_DONE |
                                                         RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
                                                         RAL_IRQ_RX_CRC_ERROR ) == RAL_STATUS_OK );

    ral_set_lora_cad_params( &( rp->radio->ral ), &rp->radio_params[id].rx.cad );

    // Wait the exact time
    while( ( int32_t )( rp->tasks[id].start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) > 0 )
    {
    }
    // At this time only tcxo startup delay is remaining
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_set_ant_switch( false );

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_lora_cad( &( rp->radio->ral ) ) == RAL_STATUS_OK );
    rp_stats_set_rx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
}

void wor_ral_callback_start_tx( void* rp_void )
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
    // At this time only tcxo startup delay is remaining
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_set_ant_switch( true );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_tx( &( rp->radio->ral ) ) == RAL_STATUS_OK );
    rp_stats_set_tx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
    // SMTC_MODEM_HAL_TRACE_MSG( "End TX\n" );
}

void wor_ral_callback_start_rx( void* rp_void )
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
    // At this time only tcxo startup delay is remaining
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_set_ant_switch( false );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_rx( &( rp->radio->ral ), rp->radio_params[id].rx.timeout_in_ms ) ==
                                     RAL_STATUS_OK );

    rp_stats_set_rx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
}
