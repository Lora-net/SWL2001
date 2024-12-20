/*!
 * \file    relay_tx.c
 *
 * \brief   Main function to interract with the relay TX (Enable, disable, configure,...)
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
#include "relay_tx_api.h"
#include "wake_on_radio.h"
#include "wake_on_radio_ral.h"
#include "radio_planner.h"
#include "relay_real.h"
#include "lr1mac_core.h"
#include "lr1mac_defs.h"
#include "lr1mac_utilities.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_modem_api.h"
#include "modem_event_utilities.h"
#include "lorawan_api.h"
#include "smtc_real.h"
#include "smtc_duty_cycle.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 */
#define DEFAULT_PPM_RELAY ( 40 )
#define DEFAULT_CAD_TO_RX ( 8 )
#define DEFAULT_CAD_PERIOD ( WOR_CAD_PERIOD_1S )
#define DEFAULT_ACTIVATION_MODE ( RELAY_TX_ACTIVATION_MODE_ED_CONTROLED )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct relay_tx_s
{
    smtc_real_t*     real;
    radio_planner_t* rp;
    uint8_t          stack_id;
    void ( *free_callback )( void* );
    void ( *busy_callback )( void* );
    void ( *abort_callback )( void* );
    relay_tx_sync_status_t sync_status;  // Relay sync status
    bool                   is_enable;    // Is relay enable

    relay_tx_activation_mode_t activation_mode;
    relay_tx_channel_config_t  default_ch_config;

    // Information compute after receiving a valid WOR ACK
    uint32_t ref_timestamp_ms;  // Estimated time when the relay performed CAD

    uint8_t               ref_channel_idx;  // Channel use
    uint8_t               ref_default_idx;   // Index (0 or 1) of the default channel
    wor_cad_periodicity_t ref_cad_period;   // Real period used by relay (default is 1s)

    // Information about the last WOR frame
    uint32_t last_preamble_len_symb;  // preamble length in symbol
    uint32_t last_preamble_len_ms;    // preamble length in ms
    uint32_t last_timestamp_ms;       // Time of send
    uint8_t  last_ch_idx;             // Channel used (default or additionnal)
    uint8_t  last_default_idx;         // Index (0 or 1) of the default channel

    uint8_t relay_xtal_drift_ppm;  // xtal drift of the relay
    uint8_t relay_cad_to_rx;       // Delay for a relay to switch from CAD to RX

    uint8_t  buffer_len;
    uint8_t  buffer[15];           // WOR & WOR ACK Buffer (15 is the biggest length of WOR and WOR ACK)
    uint32_t fcnt;                 // WOR FCNT
    uint32_t toa_ack[MAX_WOR_CH];  // Time On Air - used to send LR1 frame if WOR ACK isn't received
    uint32_t time_tx_done;         // Time when WOR has ended
    uint8_t  miss_wor_ack_cnt;     // Counter to manage synchronisation status
    uint8_t  backoff_cnt;          // Number of WOR without valid WOR ACK
    uint8_t  lr1_payload_len;

    bool               last_ack_valid;
    wor_ack_infos_t    last_ack;
    bool               need_key_derivation;
    relay_tx_config_t  relay_tx_config;
    wor_ack_mic_info_t ack_mic_info;

    uint32_t target_timer_lr1;
} relay_tx_t;

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE VARIABLES -------------------------------------------------------------
 */
static relay_tx_t relay_tx_declare[NUMBER_OF_STACKS];

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DECLARATION -------------------------------------------------
 */

/**
 * @brief Decode the WOR ACK
 *
 * @param[in]   infos       Relay TX status
 * @param[out]  ack         WOR ACK infos
 * @return true     WOR ACK is valid and has been decoded
 * @return false    Invalid WOR ACK
 */
static bool relay_tx_check_decode_ack( uint8_t relay_stack_id, wor_ack_infos_t* ack );

/**
 * @brief Radio planner callback called for WOR and WOR ACK
 *
 * @param[in]   lr1mac      LoRaWAN stack pointer
 */
static void relay_tx_callback_rp( uint8_t* relay_stack_id );

/**
 * @brief Update synchronisation of relay tx
 *
 * @param[in]   lr1mac      Param
 */
static void relay_tx_update_sync_status( uint8_t relay_stack_id, relay_tx_sync_status_t new_status );

/**
 * @brief Print current relay configuration
 *
 * @param[in]   lr1mac      Param
 */
static void relay_tx_print_conf( uint8_t relay_stack_id );

/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS DEFINITIONS --------------------------------------------------
 */
bool smtc_relay_tx_init( uint8_t relay_stack_id, radio_planner_t* rp, smtc_real_t* real,
                         void ( *free_callback )( void* free_context ), void*   free_context,
                         void ( *busy_callback )( void* busy_context ), void*   busy_context,
                         void ( *abort_callback )( void* abort_context ), void* abort_context )
{
    relay_tx_t* infos = &( relay_tx_declare[relay_stack_id] );
    memset( infos, 0, sizeof( relay_tx_t ) );

    if( ( free_callback == NULL ) || ( abort_callback == NULL ) )
    {
        SMTC_MODEM_HAL_PANIC( "smtc_relay_tx_init bad init\n" );
    }
    if( smtc_modem_hal_get_radio_tcxo_startup_delay_ms( ) >= DELAY_WOR_TO_WORACK_MS )
    {
        SMTC_MODEM_HAL_PANIC( "TCXO delay not compatible with relay mode \n" );
    }
    rp_release_hook( rp, RP_HOOK_ID_RELAY_TX + relay_stack_id );
    if( rp_hook_init( rp, RP_HOOK_ID_RELAY_TX + relay_stack_id, ( void ( * )( void* ) )( relay_tx_callback_rp ),
                      &( infos->stack_id ) ) != RP_HOOK_STATUS_OK )
    {
        return false;
    }

    infos->stack_id             = relay_stack_id;
    infos->real                 = real;
    infos->free_callback        = free_callback;
    infos->busy_callback        = busy_callback;
    infos->abort_callback       = abort_callback;
    infos->rp                   = rp;
    infos->is_enable            = false;
    infos->relay_xtal_drift_ppm = DEFAULT_PPM_RELAY;
    infos->relay_cad_to_rx      = DEFAULT_CAD_TO_RX;
    infos->ref_cad_period       = DEFAULT_CAD_PERIOD;
    infos->activation_mode      = DEFAULT_ACTIVATION_MODE;
    infos->last_default_idx      = 1;

    memset( &( infos->relay_tx_config ), 0, sizeof( relay_tx_config_t ) );
    infos->relay_tx_config.second_ch_enable                                = false;
    infos->relay_tx_config.number_of_miss_wor_ack_to_switch_in_nosync_mode = 8;
    infos->relay_tx_config.activation                                      = DEFAULT_ACTIVATION_MODE;
    relay_tx_print_conf( relay_stack_id );
    smtc_relay_get_default_channel_config( infos->real, 0, &infos->default_ch_config.dr,
                                           &infos->default_ch_config.freq_hz, &infos->default_ch_config.ack_freq_hz );

    rp_radio_params_t radio_params = { 0 };
    wor_ral_init_tx_ack( infos->real, infos->default_ch_config.dr, infos->default_ch_config.freq_hz, WOR_ACK_LENGTH,
                         &radio_params );

    infos->toa_ack[0] = ral_get_lora_time_on_air_in_ms( &infos->rp->radio->ral, &radio_params.tx.lora.pkt_params,
                                                        &radio_params.tx.lora.mod_params );

    return true;
}

void smtc_relay_tx_disable( uint8_t relay_stack_id )
{
    relay_tx_t* infos = &( relay_tx_declare[relay_stack_id] );

    if( infos->is_enable == true )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Disable relay TX\n" );
        infos->is_enable = false;
    }
}

void smtc_relay_tx_enable( uint8_t relay_stack_id )
{
    relay_tx_t* infos = &( relay_tx_declare[relay_stack_id] );

    if( ( infos->is_enable != true ) && ( infos->activation_mode != RELAY_TX_ACTIVATION_MODE_DISABLED ) )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Enable relay TX\n" );
        infos->is_enable = true;
    }
}

bool smtc_relay_tx_is_enable( uint8_t relay_stack_id )
{
    relay_tx_t* infos = &( relay_tx_declare[relay_stack_id] );
    return infos->is_enable;
}

relay_tx_sync_status_t smtc_relay_tx_get_sync_status( uint8_t relay_stack_id )
{
    relay_tx_t* infos = &( relay_tx_declare[relay_stack_id] );

    return infos->sync_status;
}

bool smtc_relay_tx_update_config( uint8_t relay_stack_id, const relay_tx_config_t* relay_config )
{
    relay_tx_t* infos = &( relay_tx_declare[relay_stack_id] );

    if( relay_config->activation == RELAY_TX_ACTIVATION_MODE_DISABLED )
    {
        infos->activation_mode = RELAY_TX_ACTIVATION_MODE_DISABLED;

        infos->is_enable = false;
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_RELAY_TX_MODE, infos->activation_mode, relay_stack_id );
    }
    if( relay_config->second_ch_enable == true )
    {
        if( smtc_real_is_frequency_valid( infos->real, relay_config->second_ch.freq_hz ) != OKLORAWAN )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Invalid second channel freq_hz (%d)\n", relay_config->second_ch.freq_hz );
            return false;
        }
        if( smtc_real_is_frequency_valid( infos->real, relay_config->second_ch.ack_freq_hz ) != OKLORAWAN )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Invalid second channel ack_freq_hz (%d)\n",
                                         relay_config->second_ch.ack_freq_hz );
            return false;
        }

        if( smtc_real_get_modulation_type_from_datarate( infos->real, relay_config->second_ch.dr ) != LORA )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Invalid second channel dr (%d)\n", relay_config->second_ch.dr );
            return false;
        }
    }
    uint8_t temp_number_of_miss_wor_ack_to_switch_in_nosync_mode = 8;
    if( relay_config->activation == RELAY_TX_ACTIVATION_MODE_ED_CONTROLED )
    {
        temp_number_of_miss_wor_ack_to_switch_in_nosync_mode =
            infos->relay_tx_config.number_of_miss_wor_ack_to_switch_in_nosync_mode;
    }

    memcpy( &( infos->relay_tx_config ), relay_config, sizeof( relay_tx_config_t ) );
    infos->relay_tx_config.number_of_miss_wor_ack_to_switch_in_nosync_mode =
        temp_number_of_miss_wor_ack_to_switch_in_nosync_mode;
    if( infos->activation_mode != infos->relay_tx_config.activation )
    {
        infos->activation_mode = infos->relay_tx_config.activation;
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_RELAY_TX_MODE, infos->activation_mode, relay_stack_id );
    }

    if( infos->relay_tx_config.activation == RELAY_TX_ACTIVATION_MODE_ENABLE )
    {
        infos->is_enable = true;
    }

    if( infos->relay_tx_config.second_ch_enable == true )
    {
        rp_radio_params_t radio_params = { 0 };
        wor_ral_init_tx_ack( infos->real, infos->relay_tx_config.second_ch.dr, infos->relay_tx_config.second_ch.freq_hz,
                             WOR_ACK_LENGTH, &radio_params );

        infos->toa_ack[1] = ral_get_lora_time_on_air_in_ms( &infos->rp->radio->ral, &radio_params.tx.lora.pkt_params,
                                                            &radio_params.tx.lora.mod_params );
    }

    relay_tx_print_conf( relay_stack_id );
    return true;
}

void smtc_relay_tx_get_config( uint8_t relay_stack_id, relay_tx_config_t* config )
{
    relay_tx_t* infos = &( relay_tx_declare[relay_stack_id] );
    if( config != NULL )
    {
        memcpy( config, &( infos->relay_tx_config ), sizeof( relay_tx_config_t ) );
    }
}

bool smtc_relay_tx_prepare_wor( uint8_t relay_stack_id, uint32_t target_time, const wor_lr1_infos_t* lr1_infos,
                                wor_tx_prepare_t* wor_tx )
{
    relay_tx_t* infos = &( relay_tx_declare[relay_stack_id] );

    const uint32_t cad_period_ms = wor_convert_cad_period_in_ms( infos->ref_cad_period );

    if( ( int32_t ) ( target_time - smtc_modem_hal_get_time_in_ms( ) - 1000 ) < 0 )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "smtc_relay_tx_prepare_wor too late\n" );
        return false;
    }

    uint32_t ref_timestamp = infos->ref_timestamp_ms;
    // -----------------------------------------------------------------
    // Choose channel (default or additional)
    infos->last_ch_idx = 0;
    if( infos->sync_status == RELAY_TX_SYNC_STATUS_INIT )
    {
        // Manage the 2 values of the default channel
        infos->last_default_idx = ( infos->last_default_idx == 0 ? 1 : 0 );

        smtc_relay_get_default_channel_config( infos->real, infos->last_default_idx, &infos->default_ch_config.dr,
                                               &infos->default_ch_config.freq_hz,
                                               &infos->default_ch_config.ack_freq_hz );
    }
    else
    {
        if( infos->relay_tx_config.second_ch_enable == true )
        {
            infos->last_ch_idx = 1;  // Always use additional channel if one is defined
            if( infos->ref_channel_idx == 0 )
            {
                // If last channel used was the default -> remove half cad period
                ref_timestamp -= cad_period_ms >> 1;
            }
             smtc_duty_cycle_update( );
            if( smtc_duty_cycle_is_channel_free(  infos->relay_tx_config.second_ch.freq_hz) == false )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "no more duty cycle for second_ch wor channel\n" );
                return false;
            }
        }
        else
        {
            smtc_relay_get_default_channel_config( infos->real, infos->last_default_idx, &infos->default_ch_config.dr,
                                                   &infos->default_ch_config.freq_hz,
                                                   &infos->default_ch_config.ack_freq_hz );
        }
    }

    // -----------------------------------------------------------------
    // Estimate drift error
    uint32_t drift_error_ms = cad_period_ms;  // init drift with CAD period (max value)
    // Don't compute drift error for at time message because it will have to use the max preamble
    if( infos->sync_status == RELAY_TX_SYNC_STATUS_SYNC )
    {
        drift_error_ms = ( lr1_infos->crystal_error_ppm + infos->relay_xtal_drift_ppm ) * 2;
        drift_error_ms *= ( target_time + cad_period_ms - ref_timestamp );
        drift_error_ms /= 1000000;

        SMTC_MODEM_HAL_TRACE_PRINTF( "Drift error : %d ms\n", drift_error_ms );

        if( drift_error_ms >= cad_period_ms )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Drift error too big (>%d) -> Go to UNSYNC\n", cad_period_ms );

            drift_error_ms = cad_period_ms;
            relay_tx_update_sync_status( relay_stack_id, RELAY_TX_SYNC_STATUS_UNSYNC );
        }
    }

    const relay_tx_channel_config_t* conf =
        ( infos->last_ch_idx == 0 ) ? &infos->default_ch_config : &( infos->relay_tx_config.second_ch );

    if( lr1_infos->is_join == false )
    {
        const wor_infos_t wor = {
            .wor_type             = WOR_MSG_TYPE_JOIN_REQUEST,
            .join_request.freq_hz = lr1_infos->freq_hz,
            .join_request.dr      = lr1_infos->dr,
        };

        infos->buffer_len          = wor_generate_wor( infos->buffer, &wor );
        infos->need_key_derivation = true;
    }
    else
    {
        infos->fcnt += 1;
        if( infos->need_key_derivation == true )
        {
            infos->need_key_derivation = false;
            wor_derive_root_skey( lr1_infos->dev_addr );
        }
        const wor_infos_t wor = {
            .wor_type         = WOR_MSG_TYPE_STANDARD_UPLINK,
            .uplink.freq_hz   = lr1_infos->freq_hz,
            .uplink.dr        = lr1_infos->dr,
            .uplink.devaddr   = lr1_infos->dev_addr,
            .uplink.fcnt      = infos->fcnt,
            .rf_infos.freq_hz = conf->freq_hz,
            .rf_infos.dr      = conf->dr,
        };
        // store wor frequency and wor datarate for future mic wor_ack computation
        infos->ack_mic_info.wor_frequency_hz = wor.uplink.freq_hz;
        infos->ack_mic_info.wor_datarate     = wor.uplink.dr;

        infos->buffer_len = wor_generate_wor( infos->buffer, &wor );
    }

    // -----------------------------------------------------------------
    // Compute preamble len based on drift_error
    lr1mac_bandwidth_t bw;
    uint8_t            sf;
    smtc_real_lora_dr_to_sf_bw( infos->real, conf->dr, &sf, &bw );

    const uint32_t symb_time_us = lr1mac_utilities_get_symb_time_us( 1, ( ral_lora_sf_t ) sf, ( ral_lora_bw_t ) bw );

    infos->last_preamble_len_symb = drift_error_ms * 1000 / symb_time_us + 1 + 6 + infos->relay_cad_to_rx;
    //+1 to round up, +6 minimum symbol for reception + delay to switch CAD->RX -> always >8

    infos->last_preamble_len_ms = infos->last_preamble_len_symb * symb_time_us / 1000 + 1;

    // SMTC_MODEM_HAL_TRACE_PRINTF( "WOR: Preamble %d symb (%d ms) at DR%d %d Hz\n", infos->last_preamble_len_symb,
    //                              infos->last_preamble_len_ms, conf->dr, conf->freq_hz );

    // -----------------------------------------------------------------
    // Estimate next slot
    if( infos->sync_status == RELAY_TX_SYNC_STATUS_SYNC )
    {
        wor_tx->at_time = true;
        // For sync device (preamble < cad period) --> estimate next real slot
        uint32_t find_n = target_time + drift_error_ms - ref_timestamp;
        find_n /= cad_period_ms;
        find_n += 2;  // +1 to get next integer and +1 to get some margin

        const uint32_t t_next = find_n * cad_period_ms + ref_timestamp;

        infos->last_timestamp_ms = t_next - ( drift_error_ms >> 1 );

        //  SMTC_MODEM_HAL_TRACE_PRINTF( "WOR find_n %d -  T_next %d\n", find_n, t_next );
    }
    else
    {
        wor_tx->at_time          = false;
        infos->last_timestamp_ms = target_time;
    }

    wor_tx->freq_hz        = conf->freq_hz;
    wor_tx->target_time_ms = infos->last_timestamp_ms - smtc_modem_hal_get_radio_tcxo_startup_delay_ms( );
    wor_tx->dr             = conf->dr;
    smtc_real_lora_dr_to_sf_bw( infos->real, conf->dr, &wor_tx->sf, ( lr1mac_bandwidth_t* ) &wor_tx->bw );
    return true;
}

bool smtc_relay_tx_send_wor( uint8_t relay_stack_id, const wor_tx_prepare_t* wor_tx )
{
    if( ( int32_t ) ( wor_tx->target_time_ms - smtc_modem_hal_get_time_in_ms( ) ) < 0 )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "wor_tx->target_time_ms %d , smtc_modem_hal_get_time_in_ms( ) = %d diff= %d \n",
                                     wor_tx->target_time_ms, smtc_modem_hal_get_time_in_ms( ),
                                     smtc_modem_hal_get_time_in_ms( ) - wor_tx->target_time_ms );
        return false;
    }

    relay_tx_t* infos = &( relay_tx_declare[relay_stack_id] );

    infos->last_timestamp_ms = wor_tx->target_time_ms;  // update internal timestamp if upper layer has move it

    wor_tx_param_t wor_param = {
        .sf                = wor_tx->sf,
        .bw                = wor_tx->bw,
        .freq_hz           = wor_tx->freq_hz,
        .preamble_len_symb = infos->last_preamble_len_symb,
        .payload           = infos->buffer,
        .payload_len       = infos->buffer_len,
        .wor_at_time       = wor_tx->at_time,
        .start_time_ms     = wor_tx->target_time_ms,
        .sync_word         = smtc_real_get_sync_word( infos->real ),
        .cr                = smtc_real_get_coding_rate( infos->real ),
        .output_pwr_in_dbm = smtc_real_clamp_output_power_eirp_vs_freq_and_dr(
            infos->real, smtc_real_get_default_max_eirp( infos->real ), wor_tx->freq_hz, wor_tx->dr ),
    };
    // -----------------------------------------------------------------
    // Load WOR frame into radio planner
    if( wor_schedule_tx_wor( RP_HOOK_ID_RELAY_TX + relay_stack_id, infos->rp, &wor_param ) == true )
    {
        return true;
    }
    return false;
}

void smtc_relay_tx_get_rxr_param( uint8_t relay_stack_id, uint8_t tx_dr, uint8_t* dr, uint32_t* freq )
{
    relay_tx_t* infos = &( relay_tx_declare[relay_stack_id] );

    if( dr != NULL )
    {
        *dr = smtc_real_get_rx1_datarate_config( infos->real, tx_dr, 0 );
    }

    if( freq != NULL )
    {
        const relay_tx_channel_config_t* conf =
            ( infos->last_ch_idx == 0 ) ? &infos->default_ch_config : &( infos->relay_tx_config.second_ch );

        *freq = conf->freq_hz;
    }
}

uint8_t smtc_relay_get_tx_max_payload( uint8_t relay_stack_id )
{
    relay_tx_t* infos = &( relay_tx_declare[relay_stack_id] );
    uint8_t     dr_relay_gtw =
        smtc_real_get_min_tx_channel_dr( infos->real );  // If no ACK has been received, supposed min DR

    if( infos->last_ack_valid == true )
    {
        dr_relay_gtw = infos->last_ack.dr_relay_gtw;
    }

    uint8_t max = smtc_real_get_max_payload_size( infos->real, dr_relay_gtw, UP_LINK );

    if( max > RELAY_OVERHEAD_FORWARD )
    {
        max -= RELAY_OVERHEAD_FORWARD;
    }
    else
    {
        max = 0;
    }

    return max;
}

uint32_t smtc_relay_tx_get_crystal_error( uint8_t relay_stack_id )
{
    relay_tx_t* infos = &( relay_tx_declare[relay_stack_id] );

    return infos->relay_xtal_drift_ppm;
}

void smtc_relay_tx_data_receive_on_rxr( uint8_t relay_stack_id )
{
    relay_tx_t* infos = &( relay_tx_declare[relay_stack_id] );

    if( infos->sync_status == RELAY_TX_SYNC_STATUS_INIT )
    {
        relay_tx_update_sync_status( relay_stack_id, RELAY_TX_SYNC_STATUS_UNSYNC );
    }
}

int32_t smtc_relay_tx_free_duty_cycle_ms_get( uint8_t relay_stack_id )
{
    relay_tx_t* infos        = &( relay_tx_declare[relay_stack_id] );
    uint32_t    tx_freq_list = infos->default_ch_config.freq_hz;
    if( infos->relay_tx_config.second_ch_enable == true )
    {
        tx_freq_list = infos->relay_tx_config.second_ch.freq_hz;
    }
    int32_t relay_tx_duty_cycle =
        smtc_relay_tx_is_enable( relay_stack_id ) ? smtc_duty_cycle_get_next_free_time_ms( 1, &tx_freq_list ) : 0;
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "relay_tx_duty_cycle = %d\n", relay_tx_duty_cycle );
    return ( relay_tx_duty_cycle );
}
/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DEFINITIONS -------------------------------------------------
 */

static bool relay_tx_check_decode_ack( uint8_t relay_stack_id, wor_ack_infos_t* ack )
{
    relay_tx_t* infos = &( relay_tx_declare[relay_stack_id] );

    // SMTC_MODEM_HAL_TRACE_ARRAY( "RX WOR ACK", infos->buffer, infos->buffer_len );

    if( infos->buffer_len != WOR_ACK_LENGTH )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "WOR ACK: Wrong size (%d)\n", infos->buffer_len );
        return false;
    }

    const relay_tx_channel_config_t* conf =
        ( infos->last_ch_idx == 0 ) ? &infos->default_ch_config : &( infos->relay_tx_config.second_ch );

    infos->ack_mic_info.dev_addr = lorawan_api_devaddr_get( relay_stack_id );
    infos->ack_mic_info.wfcnt    = infos->fcnt;

    // SMTC_MODEM_HAL_TRACE_PRINTF( "frequency_hz = %d, infos->last_ch_idx = %d ,  .dev_addr   = %x, datarate = %d\n",
    // conf->ack_freq_hz,infos->last_ch_idx,lorawan_api_devaddr_get( relay_stack_id ),conf->dr);
    //  Key is set to NULL because it is already save in the crypto element (soft or hard)
    const uint32_t mic_calc    = wor_compute_mic_ack( &( infos->ack_mic_info ), infos->buffer, NULL );
    const uint32_t mic_receive = wor_extract_mic_ack( infos->buffer );

    if( mic_calc != mic_receive )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "WOR ACK: Wrong MIC (0x%04x vs 0x%04x)\n", mic_calc, mic_receive );

        return false;
    }

    infos->ack_mic_info.frequency_hz = conf->ack_freq_hz;
    infos->ack_mic_info.datarate     = conf->dr;
    wor_decrypt_ack( infos->buffer, &( infos->ack_mic_info ), ack, NULL );
    return true;
}

static void relay_tx_callback_rp( uint8_t* stack_id )
{
    rp_status_t   rp_status;
    uint32_t      timestamp_irq;
    bool          has_to_send_data      = false;
    bool          cancel_lr1mac_process = false;
    const uint8_t relay_stack_id        = *stack_id;
    relay_tx_t*   infos                 = &( relay_tx_declare[relay_stack_id] );

    rp_get_status( infos->rp, RP_HOOK_ID_RELAY_TX + relay_stack_id, &timestamp_irq, &rp_status );

    switch( rp_status )
    {
    case RP_STATUS_TX_DONE: {
        infos->time_tx_done = timestamp_irq;
        infos->miss_wor_ack_cnt += 1;
        infos->backoff_cnt += 1;
        // WOR has been send !
        if( lorawan_api_isjoined( relay_stack_id ) == JOINED )
        {
            wor_ack_rx_param_t wor_ack;
            memset( &wor_ack, 0, sizeof( wor_ack_rx_param_t ) );
            const relay_tx_channel_config_t* conf =
                ( infos->last_ch_idx == 0 ) ? &infos->default_ch_config : &( infos->relay_tx_config.second_ch );

            wor_ack.start_time_ms =
                infos->time_tx_done + DELAY_WOR_TO_WORACK_MS - smtc_modem_hal_get_radio_tcxo_startup_delay_ms( );
            wor_ack.toa         = infos->toa_ack[infos->last_ch_idx];
            wor_ack.freq_hz     = conf->ack_freq_hz;
            wor_ack.payload     = infos->buffer;
            wor_ack.payload_len = WOR_ACK_LENGTH;
            wor_ack.cr          = smtc_real_get_coding_rate( infos->real );
            wor_ack.sync_word   = smtc_real_get_sync_word( infos->real );

            smtc_real_lora_dr_to_sf_bw( infos->real, conf->dr, &wor_ack.sf, ( lr1mac_bandwidth_t* ) &wor_ack.bw );
            wor_ack.preamble_len_in_symb = smtc_real_get_preamble_len( infos->real, wor_ack.sf );

            if( wor_schedule_rx_wor_ack( RP_HOOK_ID_RELAY_TX + relay_stack_id, infos->rp, &wor_ack ) == false )
            {
                if( infos->backoff_cnt <= infos->relay_tx_config.backoff )
                {
                    cancel_lr1mac_process = true;
                }
                else
                {
                    // Send data but reset backoff counter to only resend lr1mac in X frames
                    infos->backoff_cnt = 0;

                    // WOR ACK has NOT been received, Use TX done irq timestamp + TOA of WOR ACK
                    has_to_send_data = true;
                }
            }
            infos->target_timer_lr1 =
                infos->time_tx_done + infos->toa_ack[infos->last_ch_idx] + DELAY_WOR_TO_WORACK_MS +
                DELAY_WORACK_TO_UPLINK_MS;  // don't add "- smtc_modem_hal_get_radio_tcxo_startup_delay_ms( )" due to
                                            // the fact that lr1mac do it by itself-
                                            // smtc_modem_hal_get_radio_tcxo_startup_delay_ms( );
        }
        else
        {
            // WOR join Req has been send, send uplink
            has_to_send_data = true;
            infos->target_timer_lr1 =
                timestamp_irq +
                DELAY_WOR_TO_JOINREQ_MS;  // don't add "- smtc_modem_hal_get_radio_tcxo_startup_delay_ms(
                                          // )" due to the fact that lr1mac do it by itself
        }
        break;
    }
    case RP_STATUS_RX_PACKET: {
        wor_ack_infos_t ack;

        infos->buffer_len = infos->rp->rx_payload_size[RP_HOOK_ID_RELAY_TX + relay_stack_id];

        if( relay_tx_check_decode_ack( relay_stack_id, &ack ) != true )
        {
            if( infos->backoff_cnt <= infos->relay_tx_config.backoff )
            {
                cancel_lr1mac_process = true;
            }
            else
            {
                // Send data but reset backoff counter to only resend lr1mac in X frames
                infos->backoff_cnt = 0;

                // WOR ACK has NOT been received, Use TX done irq timestamp + TOA of WOR ACK
                has_to_send_data = true;
            }
        }
        else
        {
            infos->miss_wor_ack_cnt = 0;
            infos->backoff_cnt      = 0;
            infos->last_ack         = ack;
            infos->last_ack_valid   = true;
            relay_tx_update_sync_status( relay_stack_id, RELAY_TX_SYNC_STATUS_SYNC );
            infos->ref_cad_period       = ack.period;
            infos->ref_channel_idx      = infos->last_ch_idx;
            infos->ref_default_idx       = infos->last_default_idx;
            infos->relay_xtal_drift_ppm = wor_convert_ppm( ack.relay_ppm );
            infos->relay_cad_to_rx      = wor_convert_cadtorx( ack.cad_to_rx );

            infos->ref_timestamp_ms = infos->last_timestamp_ms + infos->last_preamble_len_ms - ack.t_offset;

            // Remark : Uncomment following line if you want to abort the LoRaWAN uplink if the relay DR is too small
            // const uint8_t max_fwd_payload = smtc_relay_get_tx_max_payload( relay_stack_id );
            // if( (infos->lr1mac->tx_payload_size - 5 ) > max_fwd_payload )  // Remove 5 fort MHDR and MIC
            // {
            //     SMTC_MODEM_HAL_TRACE_PRINTF( "WOR ACK: Payload is too big for relay (%d > %d)\n",
            //                                  infos->lr1mac->tx_payload_size, max_fwd_payload );
            //     cancel_lr1mac_process = true;
            // }
            if( ack.relay_fwd != WOR_ACK_FORWARD_OK )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "WOR ACK: Relay fwd is disabled (%d)\n", ack.relay_fwd );
                cancel_lr1mac_process = true;
            }

            if( cancel_lr1mac_process == false )
            {
                // WOR ACK has been received, Use RX done irq timestamp
                has_to_send_data = true;
            }
        }

        break;
    }
    case RP_STATUS_RX_TIMEOUT:
        // manage if we want to continue even if we don't have received the WOR ACK
        if( infos->backoff_cnt <= infos->relay_tx_config.backoff )
        {
            cancel_lr1mac_process = true;
        }
        else
        {
            // Send data but reset backoff counter to only resend lr1mac in X frames
            infos->backoff_cnt = 0;

            // WOR ACK has NOT been received, Use TX done irq timestamp + TOA of WOR ACK
            has_to_send_data = true;
        }
        break;

    default:
        SMTC_MODEM_HAL_TRACE_PRINTF( "Relay TX CB - status %d\n", rp_status );
        cancel_lr1mac_process = true;
        break;
    }

    if( ( infos->miss_wor_ack_cnt >= infos->relay_tx_config.number_of_miss_wor_ack_to_switch_in_nosync_mode ) &&
        ( infos->relay_tx_config.number_of_miss_wor_ack_to_switch_in_nosync_mode != 0 ) )
    {
        if( infos->sync_status == RELAY_TX_SYNC_STATUS_UNSYNC )
        {
            relay_tx_update_sync_status( relay_stack_id, RELAY_TX_SYNC_STATUS_INIT );

            infos->last_ack_valid       = false;
            infos->ref_cad_period       = DEFAULT_CAD_PERIOD;
            infos->relay_xtal_drift_ppm = DEFAULT_PPM_RELAY;
            infos->relay_cad_to_rx      = DEFAULT_CAD_TO_RX;
        }
        else if( infos->sync_status == RELAY_TX_SYNC_STATUS_SYNC )
        {
            relay_tx_update_sync_status( relay_stack_id, RELAY_TX_SYNC_STATUS_UNSYNC );
        }
    }

    if( cancel_lr1mac_process == true )
    {
        infos->abort_callback( NULL );
    }
    else if( has_to_send_data == true )
    {
        cb_return_param_t cb_param = {
            .abort      = false,
            .lr1_timing = infos->target_timer_lr1,
            .stack_id   = relay_stack_id,
        };

        infos->free_callback( ( void* ) &cb_param );
    }
}

static void relay_tx_update_sync_status( uint8_t relay_stack_id, relay_tx_sync_status_t new_status )
{
    relay_tx_t* infos = &( relay_tx_declare[relay_stack_id] );

    if( infos->sync_status != new_status )
    {
        infos->miss_wor_ack_cnt = 0;
        infos->sync_status      = new_status;
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_RELAY_TX_SYNC, new_status, relay_stack_id );
    }
}

static void relay_tx_print_conf( uint8_t relay_stack_id )
{
    relay_tx_t* infos = &( relay_tx_declare[relay_stack_id] );
#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
    const char* name_activation[] = { "DISABLED", "ENABLE", "DYNAMIC", "ED_CONTROLED" };
#endif
    SMTC_MODEM_HAL_TRACE_PRINTF( "\n------------------------------\n" );
    SMTC_MODEM_HAL_TRACE_PRINTF( "END DEVICE RELAY CONFIGURATION\n" );
    SMTC_MODEM_HAL_TRACE_PRINTF( "Activation :  %s\n", name_activation[infos->relay_tx_config.activation] );
    SMTC_MODEM_HAL_TRACE_PRINTF( "Smart level : %d\n", infos->relay_tx_config.smart_level );
    SMTC_MODEM_HAL_TRACE_PRINTF( "Backoff :     %d\n", infos->relay_tx_config.backoff );
    SMTC_MODEM_HAL_TRACE_PRINTF( "2nd channel : %s\n",
                                 ( infos->relay_tx_config.second_ch_enable == true ? "yes" : "no" ) );
    if( infos->relay_tx_config.second_ch_enable == true )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " - DR :   %d\n", infos->relay_tx_config.second_ch.dr );
        SMTC_MODEM_HAL_TRACE_PRINTF( " - Freq : %d\n", infos->relay_tx_config.second_ch.freq_hz );
        SMTC_MODEM_HAL_TRACE_PRINTF( " - Ack :  %d\n", infos->relay_tx_config.second_ch.ack_freq_hz );
    }
    SMTC_MODEM_HAL_TRACE_PRINTF( "------------------------------\n\n" );
}
