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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 */
#define DEFAULT_PPM_RELAY ( 40 )
#define DEFAULT_PPM_ED ( 25 )  // todo move to user space
#define DEFAULT_CAD_TO_RX ( 8 )
#define DEFAULT_CAD_PERIOD ( WOR_CAD_PERIOD_1S )
#define DEFAULT_ACTIVATION_MODE ( RELAY_TX_ACTIVATION_MODE_ED_CONTROLED )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */
typedef enum wor_sync_status_e
{
    WOR_SYNC_STATUS_INIT,
    WOR_SYNC_STATUS_UNSYNC,
    WOR_SYNC_STATUS_SYNC,
} wor_sync_status_t;
typedef struct relay_tx_infos_s
{
    lr1_stack_mac_t*  lr1mac;       // Associated lr1mac stack
    wor_sync_status_t sync_status;  // Relay sync status
    bool              is_enable;    // Is relay enable

    relay_tx_activation_mode_t activation_mode;
    relay_tx_channel_config_t  default_ch_config;

    // Information compute after receiving a valid WOR ACK
    uint32_t              ref_timestamp_ms;  // Estimated time when the relay performed CAD
    uint8_t               ref_channel_idx;   // Channel use
    uint8_t               ref_defaut_idx;    // Index (0 or 1) of the default channel
    wor_cad_periodicity_t ref_cad_period;    // Real period used by relay (default is 1s)

    // Information about the last WOR frame
    uint32_t last_preamble_len_symb;  // preamble length in symbol
    uint32_t last_preamble_len_ms;    // preamble length in ms
    uint32_t last_timestamp_ms;       // Time of send
    uint8_t  last_ch_idx;             // Channel used (default or additionnal)
    uint8_t  last_defaut_idx;         // Index (0 or 1) of the default channel

    uint8_t relay_xtal_drift_ppm;  // xtal drift of the relay
    uint8_t ed_xtal_drift_ppm;     // xtal drift of the end-device
    uint8_t relay_cad_to_rx;       // Delay for a relay to switch from CAD to RX

    uint8_t  buffer_len;
    uint8_t  buffer[15];           // WOR & WOR ACK Buffer (15 is the biggest length of WOR and WOR ACK)
    uint32_t fcnt;                 // WOR FCNT
    uint32_t toa_ack[MAX_WOR_CH];  // Time On Air - used to send LR1 frame if WOR ACK isn't received
    uint32_t time_tx_done;         // Time when WOR has ended
    uint8_t  unsync_cnt;           // Counter to manage synchronisation status
    uint8_t  backoff_cnt;          // Number of WOR without valid WOR ACK
    bool     wor_is_at_time;       // WOR need to send at time or not

    bool            last_ack_valid;
    wor_ack_infos_t last_ack;
    bool            need_key_derivation;
} relay_tx_infos_t;

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE VARIABLES -------------------------------------------------------------
 */
static relay_tx_infos_t  relay_tx_infos  = { 0 };
static relay_tx_config_t relay_tx_config = { 0 };

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DECLARATION -------------------------------------------------
 */

/**
 * @brief Configure the ED to receive WOR ACK
 *
 */
static void relay_tx_receive_ack( void );

/**
 * @brief Decode the WOR ACK
 *
 * @param[in]   infos       Relay TX status
 * @param[out]  ack         WOR ACK infos
 * @return true     WOR ACK is valid and has been decoded
 * @return false    Invalid WOR ACK
 */
static bool relay_tx_check_decode_ack( const relay_tx_infos_t* infos, wor_ack_infos_t* ack );

/**
 * @brief Radio planner callback called for WOR and WOR ACK
 *
 * @param[in]   lr1mac      LoRaWAN stack pointer
 */
static void relay_tx_callback_rp( lr1_stack_mac_t* lr1mac );

/**
 * @brief Print current relay configuration
 *
 * @param[in]   lr1mac      Param
 */
static void relay_tx_print_conf( lr1_stack_mac_t* lr1mac );

/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS DEFINITIONS --------------------------------------------------
 */
bool smtc_relay_tx_init( lr1_stack_mac_t* lr1mac )
{
    static bool do_once = false;

    if( do_once == false )
    {
        do_once = true;

        if( rp_hook_init( lr1mac->rp, RP_HOOK_ID_RELAY_TX, ( void ( * )( void* ) )( relay_tx_callback_rp ), lr1mac ) !=
            RP_HOOK_STATUS_OK )
        {
            return false;
        }
        memset( &relay_tx_infos, 0, sizeof( relay_tx_infos ) );
        relay_tx_infos.lr1mac               = lr1mac;
        relay_tx_infos.is_enable            = false;
        relay_tx_infos.relay_xtal_drift_ppm = DEFAULT_PPM_RELAY;
        relay_tx_infos.ed_xtal_drift_ppm    = DEFAULT_PPM_ED;
        relay_tx_infos.relay_cad_to_rx      = DEFAULT_CAD_TO_RX;
        relay_tx_infos.ref_cad_period       = DEFAULT_CAD_PERIOD;
        relay_tx_infos.activation_mode      = DEFAULT_ACTIVATION_MODE;
        relay_tx_infos.last_defaut_idx      = 1;

        smtc_relay_get_default_channel_config( lr1mac->real, 0, &relay_tx_infos.default_ch_config.dr,
                                               &relay_tx_infos.default_ch_config.freq_hz,
                                               &relay_tx_infos.default_ch_config.ack_freq_hz );

        rp_radio_params_t radio_params = { 0 };
        wor_ral_init_tx_ack( relay_tx_infos.lr1mac->real, relay_tx_infos.default_ch_config.dr,
                             relay_tx_infos.default_ch_config.freq_hz, WOR_ACK_LENGTH, &radio_params );

        relay_tx_infos.toa_ack[0] =
            ral_get_lora_time_on_air_in_ms( &relay_tx_infos.lr1mac->rp->radio->ral, &radio_params.tx.lora.pkt_params,
                                            &radio_params.tx.lora.mod_params );
    }

    return true;
}

void smtc_relay_tx_disable( lr1_stack_mac_t* lr1mac )
{
    if( ( relay_tx_config.activation == RELAY_TX_ACTIVATION_MODE_ED_CONTROLED ) ||
        ( relay_tx_config.activation == RELAY_TX_ACTIVATION_MODE_DYNAMIC ) )
    {
        if( relay_tx_infos.is_enable == true )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Disable relay TX\n" );
            relay_tx_infos.is_enable = false;
        }
    }
}

void smtc_relay_tx_enable( lr1_stack_mac_t* lr1mac )
{
    if( ( relay_tx_config.activation == RELAY_TX_ACTIVATION_MODE_ED_CONTROLED ) ||
        ( relay_tx_config.activation == RELAY_TX_ACTIVATION_MODE_DYNAMIC ) )

    {
        if( relay_tx_infos.is_enable != true )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Enable relay TX\n" );
            relay_tx_infos.is_enable = true;
        }
    }
}

bool smtc_relay_tx_is_enable( lr1_stack_mac_t* lr1mac )
{
    return relay_tx_infos.is_enable;
}

bool smtc_relay_tx_update_config( lr1_stack_mac_t* lr1mac, const relay_tx_config_t* config )
{
    if( config->activation == RELAY_TX_ACTIVATION_MODE_DYNAMIC )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Dynamic not yet managed \n" );
        return false;
    }
    if( config->activation == RELAY_TX_ACTIVATION_MODE_DISABLED )
    {
        relay_tx_infos.activation_mode = RELAY_TX_ACTIVATION_MODE_DISABLED;
        relay_tx_infos.is_enable       = false;
        return true;
    }

    if( config->second_ch_enable == true )
    {
        if( smtc_real_is_frequency_valid( lr1mac->real, config->second_ch.freq_hz ) != OKLORAWAN )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Invalid second channel freq_hz (%d)\n", config->second_ch.freq_hz );
            return false;
        }
        if( smtc_real_is_frequency_valid( lr1mac->real, config->second_ch.ack_freq_hz ) != OKLORAWAN )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Invalid second channel ack_freq_hz (%d)\n", config->second_ch.ack_freq_hz );
            return false;
        }

        if( smtc_real_get_modulation_type_from_datarate( lr1mac->real, config->second_ch.dr ) != LORA )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Invalid second channel dr (%d)\n", config->second_ch.dr );
            return false;
        }
    }

    relay_tx_infos.activation_mode = config->activation;
    relay_tx_config                = *config;

    if( config->activation == RELAY_TX_ACTIVATION_MODE_ENABLE )
    {
        relay_tx_infos.is_enable = true;
    }

    if( relay_tx_config.second_ch_enable == true )
    {
        rp_radio_params_t radio_params = { 0 };
        wor_ral_init_tx_ack( relay_tx_infos.lr1mac->real, relay_tx_config.second_ch.dr,
                             relay_tx_config.second_ch.freq_hz, WOR_ACK_LENGTH, &radio_params );

        relay_tx_infos.toa_ack[1] =
            ral_get_lora_time_on_air_in_ms( &relay_tx_infos.lr1mac->rp->radio->ral, &radio_params.tx.lora.pkt_params,
                                            &radio_params.tx.lora.mod_params );
    }

    relay_tx_print_conf( lr1mac );
    return true;
}

void smtc_relay_tx_get_config( lr1_stack_mac_t* lr1mac, relay_tx_config_t* config )
{
    if( config != NULL )
    {
        *config = relay_tx_config;
    }
}

void smtc_relay_tx_send_wor( lr1_stack_mac_t* lr1mac )
{
    const uint32_t now_ms        = smtc_modem_hal_get_time_in_ms( ) + 20;  // +20 is a generic delay
    const uint32_t cad_period_ms = wor_convert_cad_period_in_ms( relay_tx_infos.ref_cad_period );

    uint32_t ref_timestamp = relay_tx_infos.ref_timestamp_ms;
    // -----------------------------------------------------------------
    // Choose channel (default or additionnal)
    relay_tx_infos.last_ch_idx = 0;
    if( relay_tx_infos.sync_status == WOR_SYNC_STATUS_INIT )
    {
        // Todo - Manage the 2 value of the default channel
        // relay_tx_infos.last_defaut_idx = ( relay_tx_infos.last_defaut_idx == 0 ? 1 : 0 );
        relay_tx_infos.last_defaut_idx = 0;
        SMTC_MODEM_HAL_PANIC_ON_FAILURE(
            smtc_relay_get_default_channel_config( lr1mac->real, relay_tx_infos.last_defaut_idx,
                                                   &relay_tx_infos.default_ch_config.dr,
                                                   &relay_tx_infos.default_ch_config.freq_hz,
                                                   &relay_tx_infos.default_ch_config.ack_freq_hz ) == OKLORAWAN );
    }
    else
    {
        if( relay_tx_config.second_ch_enable == true )
        {
            relay_tx_infos.last_ch_idx = 1;  // Always use additionnal channel if one is defined
            if( relay_tx_infos.ref_channel_idx == 0 )
            {
                // If last channel used was the default -> remove hal cad period
                ref_timestamp -= cad_period_ms >> 1;
            }
        }
        else
        {
            SMTC_MODEM_HAL_PANIC_ON_FAILURE(
                smtc_relay_get_default_channel_config( lr1mac->real, relay_tx_infos.last_defaut_idx,
                                                       &relay_tx_infos.default_ch_config.dr,
                                                       &relay_tx_infos.default_ch_config.freq_hz,
                                                       &relay_tx_infos.default_ch_config.ack_freq_hz ) == OKLORAWAN );
        }
    }

    // -----------------------------------------------------------------
    // Estimate drift error
    uint32_t drift_error_ms = cad_period_ms;  // init drift with CAD period (max value)
    // Dont compute drift error for at time message because it will have to use the max preamble
    if( ( relay_tx_infos.sync_status == WOR_SYNC_STATUS_SYNC ) && ( relay_tx_infos.lr1mac->send_at_time == false ) )
    {
        drift_error_ms = ( relay_tx_infos.ed_xtal_drift_ppm + relay_tx_infos.relay_xtal_drift_ppm );
        drift_error_ms *= ( now_ms + cad_period_ms - ref_timestamp );
        drift_error_ms /= 1000000;

        SMTC_MODEM_HAL_TRACE_PRINTF( "Drift error : %d ms\n", drift_error_ms );

        if( drift_error_ms >= cad_period_ms )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Drift error too big (>%d) -> Go to UNSYNC\n", cad_period_ms );

            relay_tx_infos.sync_status = WOR_SYNC_STATUS_UNSYNC;
            relay_tx_infos.unsync_cnt  = 0;
            drift_error_ms             = cad_period_ms;
        }
    }

    const relay_tx_channel_config_t* conf =
        ( relay_tx_infos.last_ch_idx == 0 ) ? &relay_tx_infos.default_ch_config : &relay_tx_config.second_ch;
    ;

    if( relay_tx_infos.lr1mac->join_status != JOINED )
    {
        const wor_infos_t wor = {
            .wor_type             = WOR_MSG_TYPE_JOIN_REQUEST,
            .join_request.freq_hz = lr1mac->tx_frequency,
            .join_request.dr      = lr1mac->tx_data_rate,
        };

        relay_tx_infos.buffer_len          = wor_generate_wor( relay_tx_infos.buffer, &wor );
        relay_tx_infos.need_key_derivation = true;
    }
    else
    {
        if( relay_tx_infos.need_key_derivation == true )
        {
            relay_tx_infos.need_key_derivation = false;
            wor_derive_root_skey( lr1mac->dev_addr );
        }
        const wor_infos_t wor = {
            .wor_type         = WOR_MSG_TYPE_STANDARD_UPLINK,
            .uplink.freq_hz   = lr1mac->tx_frequency,
            .uplink.dr        = lr1mac->tx_data_rate,
            .uplink.devaddr   = relay_tx_infos.lr1mac->dev_addr,
            .uplink.fcnt      = relay_tx_infos.fcnt,
            .rf_infos.freq_hz = conf->freq_hz,
            .rf_infos.dr      = conf->dr,
        };

        relay_tx_infos.buffer_len = wor_generate_wor( relay_tx_infos.buffer, &wor );
    }

    // -----------------------------------------------------------------
    // Compute preamble len based on drif_error

    lr1mac_bandwidth_t bw;
    uint8_t            sf;
    smtc_real_lora_dr_to_sf_bw( lr1mac->real, conf->dr, &sf, &bw );

    const uint32_t symb_time_us = lr1mac_utilities_get_symb_time_us( 1, ( ral_lora_sf_t ) sf, ( ral_lora_bw_t ) bw );

    relay_tx_infos.last_preamble_len_symb =
        drift_error_ms * 1000 / symb_time_us + 1 + 6 + relay_tx_infos.relay_cad_to_rx;
    //+1 to round up, +6 minimun symbol for reception + delay to switch CAD->RX -> always >8

    relay_tx_infos.last_preamble_len_ms = relay_tx_infos.last_preamble_len_symb * symb_time_us / 1000;

    SMTC_MODEM_HAL_TRACE_PRINTF( "WOR: Preamble %d symb (%d ms) at DR%d %d Hz\n", relay_tx_infos.last_preamble_len_symb,
                                 relay_tx_infos.last_preamble_len_ms, conf->dr, conf->freq_hz );

    rp_radio_params_t rp_radio_params = { 0 };
    wor_ral_init_tx_wor( relay_tx_infos.lr1mac->real, conf->dr, conf->freq_hz, relay_tx_infos.last_preamble_len_symb,
                         relay_tx_infos.buffer_len, &rp_radio_params );

    const uint32_t toa_wor_ms =
        ral_get_lora_time_on_air_in_ms( &relay_tx_infos.lr1mac->rp->radio->ral, &rp_radio_params.tx.lora.pkt_params,
                                        &rp_radio_params.tx.lora.mod_params );

    // -----------------------------------------------------------------
    // Update delay between end of WOR and start of LoRaWAN message
    uint32_t delay_wor_lora = DELAY_WOR_TO_JOINREQ_MS;
    if( relay_tx_infos.lr1mac->join_status == JOINED )
    {
        delay_wor_lora =
            DELAY_WOR_TO_WORACK_MS + DELAY_WORACK_TO_UPLINK_MS + relay_tx_infos.toa_ack[relay_tx_infos.last_ch_idx];
    }

    // -----------------------------------------------------------------
    // Estimate next slot
    if( relay_tx_infos.lr1mac->send_at_time == true )
    {
        // For now send at time message will use max preamble size to avoid too complex calcul
        relay_tx_infos.wor_is_at_time    = true;
        relay_tx_infos.last_timestamp_ms = relay_tx_infos.lr1mac->rtc_target_timer_ms - delay_wor_lora - toa_wor_ms;
    }
    else if( relay_tx_infos.sync_status == WOR_SYNC_STATUS_SYNC )
    {
        relay_tx_infos.wor_is_at_time = true;
        // For sync device (preamble < cad period) --> estimate next real slot
        uint32_t find_n = now_ms + drift_error_ms - ref_timestamp;
        find_n /= cad_period_ms;
        find_n += 2;  // +1 to get next integer and +1 to get some margin

        const uint32_t t_next = find_n * cad_period_ms + ref_timestamp;

        relay_tx_infos.last_timestamp_ms = t_next - ( drift_error_ms >> 1 );

        // SMTC_MODEM_HAL_TRACE_PRINTF( "WOR find_n %d -  T_next %d\n", find_n, t_next );
    }
    else
    {
        relay_tx_infos.wor_is_at_time    = false;
        relay_tx_infos.last_timestamp_ms = now_ms + 20;
    }

    // -----------------------------------------------------------------
    // Build WOR frame based on the LoRaWAN uplink or Join Request

    // SMTC_MODEM_HAL_TRACE_ARRAY( "WOR TX", relay_tx_infos.buffer, relay_tx_infos.buffer_len );
    // SMTC_MODEM_HAL_TRACE_PRINTF( "WOR T_start %d - TOA %dms - Delay %dms\n", relay_tx_infos.last_timestamp_ms,
    //                              rp_task.duration_time_ms, delay_wor_lora );

    const rp_task_t rp_task = {
        .hook_id               = RP_HOOK_ID_RELAY_TX,
        .launch_task_callbacks = wor_ral_callback_start_tx,
        .type                  = RP_TASK_TYPE_TX_LORA,
        .duration_time_ms      = toa_wor_ms,
        .start_time_ms         = relay_tx_infos.last_timestamp_ms,
        .state = ( relay_tx_infos.wor_is_at_time == true ) ? RP_TASK_STATE_SCHEDULE : RP_TASK_STATE_ASAP,
    };

    if( rp_task_enqueue( relay_tx_infos.lr1mac->rp, &rp_task, relay_tx_infos.buffer, relay_tx_infos.buffer_len,
                         &rp_radio_params ) == RP_HOOK_STATUS_OK )
    {
        relay_tx_infos.fcnt += 1;
        relay_tx_infos.backoff_cnt += 1;
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "RP is busy (smtc_relay_tx_send_wor)\n" );
    }
}

void smtc_relay_tx_get_rxr_param( lr1_stack_mac_t* lr1mac, uint8_t* dr, uint32_t* freq )
{
    if( dr != NULL )
    {
        *dr = smtc_real_get_rx1_datarate_config( lr1mac->real, lr1mac->tx_data_rate, 0 );
    }

    if( freq != NULL )
    {
        const relay_tx_channel_config_t* conf =
            ( relay_tx_infos.last_ch_idx == 0 ) ? &relay_tx_infos.default_ch_config : &relay_tx_config.second_ch;

        *freq = conf->freq_hz;
    }
}

uint8_t smtc_relay_get_tx_max_payload( lr1_stack_mac_t* lr1mac )
{
    uint8_t dr_relay_gtw = 0;  // If no ACK has been received, supposed DR0
    if( relay_tx_infos.last_ack_valid == true )
    {
        dr_relay_gtw = relay_tx_infos.last_ack.dr_relay_gtw;
    }

    uint8_t max = smtc_real_get_max_payload_size( relay_tx_infos.lr1mac->real, dr_relay_gtw, UP_LINK );

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

uint32_t smtc_relay_tx_get_crystal_error( lr1_stack_mac_t* lr1mac )
{
    return relay_tx_infos.relay_xtal_drift_ppm;
}

void smtc_relay_tx_data_receive_on_rxr( lr1_stack_mac_t* lr1mac )
{
    if( relay_tx_infos.sync_status == WOR_SYNC_STATUS_INIT )
    {
        relay_tx_infos.sync_status = WOR_SYNC_STATUS_UNSYNC;
    }
}

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DEFINITIONS -------------------------------------------------
 */
static void relay_tx_receive_ack( void )
{
    const relay_tx_channel_config_t* conf =
        ( relay_tx_infos.last_ch_idx == 0 ) ? &relay_tx_infos.default_ch_config : &relay_tx_config.second_ch;

    const rp_task_t rp_task = {
        .hook_id               = RP_HOOK_ID_RELAY_TX,
        .state                 = RP_TASK_STATE_SCHEDULE,
        .type                  = RP_TASK_TYPE_RX_LORA,
        .start_time_ms         = relay_tx_infos.time_tx_done + DELAY_WOR_TO_WORACK_MS,
        .duration_time_ms      = relay_tx_infos.toa_ack[relay_tx_infos.last_ch_idx],
        .launch_task_callbacks = wor_ral_callback_start_rx,
    };

    rp_radio_params_t radio_params = {
        .rx.timeout_in_ms = rp_task.duration_time_ms,
    };
    wor_ral_init_rx_ack( relay_tx_infos.lr1mac->real, conf->dr, conf->ack_freq_hz, WOR_ACK_LENGTH, &radio_params );

    // SMTC_MODEM_HAL_TRACE_PRINTF( "RX WOR ACK at DR%d at %d Hz\n", conf->dr, conf->ack_freq_hz );

    if( rp_task_enqueue( relay_tx_infos.lr1mac->rp, &rp_task, relay_tx_infos.buffer, 255, &radio_params ) !=
        RP_HOOK_STATUS_OK )

    {
        SMTC_MODEM_HAL_TRACE_MSG( "RP is busy (relay_tx_receive_ack)\n" );
    }
}

static bool relay_tx_check_decode_ack( const relay_tx_infos_t* infos, wor_ack_infos_t* ack )
{
    // SMTC_MODEM_HAL_TRACE_ARRAY( "RX WOR ACK", infos->buffer, infos->buffer_len );

    if( infos->buffer_len != WOR_ACK_LENGTH )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "WOR ACK: Wrong size (%d)\n", infos->buffer_len );
        return false;
    }

    const wor_ack_mic_info_t ack_mic_info = {
        .dev_addr     = infos->lr1mac->dev_addr,
        .wfcnt        = infos->fcnt - 1,
        .frequency_hz = infos->lr1mac->tx_frequency,
        .datarate     = infos->lr1mac->tx_data_rate,
    };

    // Key is set to NULL because it is already save in the crypto element (soft or hard)
    const uint32_t mic_calc    = wor_compute_mic_ack( &ack_mic_info, infos->buffer, NULL );
    const uint32_t mic_receive = wor_extract_mic_ack( infos->buffer );

    if( mic_calc != mic_receive )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "WOR ACK: Wrong MIC (0x%04x vs 0x%04x)\n", mic_calc, mic_receive );
        return false;
    }

    wor_decrypt_ack( infos->buffer, &ack_mic_info, ack, NULL );
    return true;
}

static void relay_tx_callback_rp( lr1_stack_mac_t* lr1mac )
{
    rp_status_t rp_status;
    uint32_t    timestamp_irq;
    bool        has_to_send_data      = false;
    bool        cancel_lr1mac_process = false;

    rp_get_status( lr1mac->rp, RP_HOOK_ID_RELAY_TX, &timestamp_irq, &rp_status );

    switch( rp_status )
    {
    case RP_STATUS_TX_DONE: {
        relay_tx_infos.time_tx_done = timestamp_irq;
        // WOR has been send !
        if( relay_tx_infos.sync_status == WOR_SYNC_STATUS_UNSYNC )
        {
            relay_tx_infos.unsync_cnt += 1;
        }

        if( relay_tx_infos.lr1mac->join_status == JOINED )
        {
            relay_tx_receive_ack( );
        }
        else
        {
            // WOR join Req has been send, send uplink
            lr1mac->rtc_target_timer_ms = timestamp_irq + DELAY_WOR_TO_JOINREQ_MS;
            has_to_send_data            = true;
        }
        break;
    }
    case RP_STATUS_RX_PACKET: {
        wor_ack_infos_t ack;

        relay_tx_infos.buffer_len = lr1mac->rp->rx_payload_size[RP_HOOK_ID_RELAY_TX];

        if( relay_tx_check_decode_ack( &relay_tx_infos, &ack ) != true )
        {
            cancel_lr1mac_process = true;
        }
        else
        {
            relay_tx_infos.last_ack       = ack;
            relay_tx_infos.last_ack_valid = true;
            relay_tx_infos.backoff_cnt    = 0;
            relay_tx_infos.sync_status    = WOR_SYNC_STATUS_SYNC;

            const uint8_t max_fwd_payload = smtc_relay_get_tx_max_payload( relay_tx_infos.lr1mac );

            if( ( relay_tx_infos.lr1mac->tx_payload_size - 5 ) > max_fwd_payload )  // Remove 5 fort MHDR and MIC
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "WOR ACK: Payload is too big for relay (%d > %d)\n",
                                             relay_tx_infos.lr1mac->tx_payload_size, max_fwd_payload );
                cancel_lr1mac_process = true;
            }
            if( ack.relay_fwd != WOR_ACK_FORWARD_OK )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "WOR ACK: Relay fwd is disabled (%d)\n", ack.relay_fwd );
                cancel_lr1mac_process = true;
            }

            if( cancel_lr1mac_process == false )
            {
                // WOR ACK has been received, Use RX done irq timestamp
                relay_tx_infos.lr1mac->rtc_target_timer_ms = timestamp_irq + DELAY_WOR_TO_WORACK_MS;
                relay_tx_infos.ref_cad_period              = ack.period;
                relay_tx_infos.ref_channel_idx             = relay_tx_infos.last_ch_idx;
                relay_tx_infos.ref_defaut_idx              = relay_tx_infos.last_defaut_idx;
                relay_tx_infos.relay_xtal_drift_ppm        = wor_convert_ppm( ack.relay_ppm );
                relay_tx_infos.relay_cad_to_rx             = wor_convert_cadtorx( ack.cad_to_rx );
                relay_tx_infos.ref_timestamp_ms =
                    relay_tx_infos.last_timestamp_ms + relay_tx_infos.last_preamble_len_ms - ack.t_offset;

                has_to_send_data = true;
            }
        }

        break;
    }
    case RP_STATUS_RX_TIMEOUT: {
        // manage if we want to continue even if we don't have received the WOR ACK
        if( relay_tx_infos.backoff_cnt < relay_tx_config.backoff )
        {
            cancel_lr1mac_process = true;
        }
        else
        {
            // Send data but reset backoff counter to only resend lr1mac in X frames
            relay_tx_infos.backoff_cnt = 0;
            has_to_send_data           = true;
            // WOR ACK has NOT been received, Use TX done irq timestamp + TOA of WOR ACK
            relay_tx_infos.lr1mac->rtc_target_timer_ms = relay_tx_infos.time_tx_done +
                                                         relay_tx_infos.toa_ack[relay_tx_infos.last_ch_idx] +
                                                         DELAY_WOR_TO_WORACK_MS + DELAY_WORACK_TO_UPLINK_MS;
        }
        break;
    }
    default:
        SMTC_MODEM_HAL_TRACE_PRINTF( "Relay TX CB - status %d\n", rp_status );
        cancel_lr1mac_process = true;
        break;
    }

    if( cancel_lr1mac_process == true )
    {
        SMTC_MODEM_HAL_TRACE_MSG( "WOR: Cancel LR1mac task\n" );
        lr1_stack_mac_radio_abort_lbt( lr1mac );
    }
    else if( has_to_send_data == true )
    {
        if( smtc_lbt_get_state( lr1mac->lbt_obj ) == true )
        {
            smtc_lbt_listen_channel( ( lr1mac->lbt_obj ), lr1mac->tx_frequency, lr1mac->send_at_time,
                                     lr1mac->rtc_target_timer_ms - RP_MARGIN_DELAY, lr1_stack_toa_get( lr1mac ) );
        }
        else
        {
            lr1mac->send_at_time = true;
            lr1_stack_mac_tx_radio_start( lr1mac );
        }
    }

    if( ( relay_tx_infos.sync_status == WOR_SYNC_STATUS_UNSYNC ) && ( relay_tx_infos.unsync_cnt >= 8 ) )
    {
        SMTC_MODEM_HAL_TRACE_MSG( "WOR: Return to init state\n" );
        relay_tx_infos.sync_status          = WOR_SYNC_STATUS_INIT;
        relay_tx_infos.last_ack_valid       = false;
        relay_tx_infos.ref_cad_period       = DEFAULT_CAD_PERIOD;
        relay_tx_infos.relay_xtal_drift_ppm = DEFAULT_PPM_RELAY;
        relay_tx_infos.relay_cad_to_rx      = DEFAULT_CAD_TO_RX;
    }
}

static void relay_tx_print_conf( lr1_stack_mac_t* lr1mac )
{
    const char* name_activation[] = { "DISABLED", "ENABLE", "DYNAMIC", "ED_CONTROLED" };

    SMTC_MODEM_HAL_TRACE_PRINTF( "\n------------------------------\n" );
    SMTC_MODEM_HAL_TRACE_PRINTF( "END DEVICE RELAY CONFIGURATION\n" );
    SMTC_MODEM_HAL_TRACE_PRINTF( "Activation :  %s\n", name_activation[relay_tx_config.activation] );
    SMTC_MODEM_HAL_TRACE_PRINTF( "Smart level : %d\n", relay_tx_config.smart_level );
    SMTC_MODEM_HAL_TRACE_PRINTF( "Backoff :     %d\n", relay_tx_config.backoff );
    SMTC_MODEM_HAL_TRACE_PRINTF( "2nd channel : %s\n", ( relay_tx_config.second_ch_enable == true ? "yes" : "no" ) );
    if( relay_tx_config.second_ch_enable == true )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " - DR :   %d\n", relay_tx_config.second_ch.dr );
        SMTC_MODEM_HAL_TRACE_PRINTF( " - Freq : %d\n", relay_tx_config.second_ch.freq_hz );
        SMTC_MODEM_HAL_TRACE_PRINTF( " - Ack :  %d\n", relay_tx_config.second_ch.ack_freq_hz );
    }
    SMTC_MODEM_HAL_TRACE_PRINTF( "------------------------------\n\n" );
}
