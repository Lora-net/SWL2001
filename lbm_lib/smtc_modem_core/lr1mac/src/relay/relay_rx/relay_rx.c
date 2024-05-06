/*!
 * \file    relay_rx.c
 *
 * \brief   Main function to interract with the relay RX (start,stop, configure,...)
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2023. All rights reserved.
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
#include "relay_rx_api.h"
#include "relay_real.h"
#include "relay_def.h"
#include "wake_on_radio.h"
#include "wake_on_radio_def.h"
#include "wake_on_radio_ral.h"
#include "radio_planner.h"
#include "lorawan_api.h"
#include "lr1mac_utilities.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "modem_core.h"
#include "lorawan_relay_rx_service.h"
#include "relay_mac_parser.h"
#include "smtc_duty_cycle.h"
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 */
#define ROOT_WOR_SKEY_LEN ( 16 )
#define JOINREQ_EUI_LEN ( 16 )
#define SIZE_TAB_DEV_ADDR_LIST ( 16 )
#define SIZE_TAB_JOIN_REQ_LIST ( 16 )
#define MAX_UINT16 ( 0x0000FFFF )

#define RELAY_FWD_UPLINK_SET_METADATA_WOR_CH( a ) ( ( uint32_t )( ( ( a ) &0x0003 ) << 16 ) )
#define RELAY_FWD_UPLINK_SET_METADATA_UPLINK_RSSI( a ) ( ( uint32_t )( ( ( a ) &0x007F ) << 9 ) )
#define RELAY_FWD_UPLINK_SET_METADATA_UPLINK_SNR( a ) ( ( uint32_t )( ( ( a ) &0x001F ) << 4 ) )
#define RELAY_FWD_UPLINK_SET_METADATA_UPLINK_DR( a ) ( ( uint32_t )( ( a ) &0x000F ) )

#define RELAY_NOTIFY_POWER_LEVEL_UPLINK_RSSI( a ) ( ( uint16_t )( ( ( a ) &0x7F ) << 5 ) )
#define RELAY_NOTIFY_POWER_LEVEL_UPLINK_SNR( a ) ( ( uint16_t )( ( a ) &0x001F ) )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */
typedef enum relay_fwd_uplink_byte_order_e
{
    RELAY_FWD_UPLINK_BYTE_ORDER_UPLINK_METADATA_7_0,
    RELAY_FWD_UPLINK_BYTE_ORDER_UPLINK_METADATA_15_8,
    RELAY_FWD_UPLINK_BYTE_ORDER_UPLINK_METADATA_23_16,
    RELAY_FWD_UPLINK_BYTE_ORDER_UPLINK_FREQUENCY_7_0,
    RELAY_FWD_UPLINK_BYTE_ORDER_UPLINK_FREQUENCY_15_8,
    RELAY_FWD_UPLINK_BYTE_ORDER_UPLINK_FREQUENCY_23_16,
    RELAY_FWD_UPLINK_BYTE_ORDER_UPLINK_PAYLOAD,  // First index of the payload
} relay_fwd_uplink_byte_order_t;

typedef enum relay_notify_byte_order_e
{
    RELAY_NOTIFY_BYTE_CID_REQ,
    RELAY_NOTIFY_BYTE_ORDER_DEV_ADDR_7_0,
    RELAY_NOTIFY_BYTE_ORDER_DEV_ADDR_15_8,
    RELAY_NOTIFY_BYTE_ORDER_DEV_ADDR_23_16,
    RELAY_NOTIFY_BYTE_ORDER_DEV_ADDR_31_24,
    RELAY_NOTIFY_BYTE_ORDER_POWER_LEVEL_7_0,
    RELAY_NOTIFY_BYTE_ORDER_POWER_LEVEL_15_8,
    RELAY_NOTIFY_BYTE_ORDER_POWER_LEVEL_LENGTH,  // Not an element
} relay_notify_byte_order_t;

typedef enum cad_state_e
{
    CAD_STATE_NO_CFG,
    CAD_STATE_INIT,
    CAD_STATE_ENQUEUE_CAD,
    CAD_STATE_WAIT_CAD_COMPLETION,
    CAD_STATE_WAIT_WOR_COMPLETION,
    CAD_STATE_WAIT_TX_ACK_COMPLETION,
    CAD_STATE_WAIT_RX_DATA_COMPLETION,
} cad_state_t;

typedef struct relay_fwd_uplink_list_s
{
    bool               in_use;
    uint32_t           dev_addr;
    uint32_t           wfcnt32;
    uint8_t            wor_s_int_key[ROOT_WOR_SKEY_LEN];
    uint8_t            wor_s_enc_key[ROOT_WOR_SKEY_LEN];
    relay_fwd_config_t fwd_cfg;
} relay_fwd_uplink_list_t;

typedef struct relay_fwd_join_list_s
{
    relay_filter_fwd_type_t action;
    // Concatenation of JOIN EUI + DEV EUI : MSB(JOIN EUI) ... LSB(JOIN EUI) - MSB(DEV EUI) ... LSB(DEV EUI)
    uint8_t eui[JOINREQ_EUI_LEN];
    uint8_t len;  // Len of eui (len(JOIN EUI) + len(DEV EUI))
} relay_fwd_join_list_t;

typedef struct relay_infos_s
{
    lr1_stack_mac_t*           lr1mac;
    const ralf_t*              radio;
    bool                       is_started;
    cad_state_t                state;
    ral_lora_rx_pkt_status_t   rx_status;
    smtc_dtc_enablement_type_t dtc_state;

    uint32_t last_cad_ms;
    uint32_t next_cad_ms;
    uint32_t rx_wor_timestamp_ms;
    uint32_t rx_uplink_timestamp_ms;
    uint32_t tx_ack_timestamp_ms;
    uint32_t wor_toa_ms[MAX_WOR_CH];
    uint32_t wor_preamble_ms[MAX_WOR_CH];
    int16_t  t_irq_comp_wor_ms[MAX_WOR_CH];

    uint16_t buffer_length;
    uint8_t  buffer[255];  // Uplink and downlink buffer

    const relay_channel_config_t* current_cfg;
    uint8_t                       rx_msg_devaddr_idx;
    uint8_t                       current_ch_idx;
    uint16_t                      t_offset;
    wor_ack_cad_to_rx_t           cad_to_rx;
    wor_ack_ppm_error_t           error_ppm;
} relay_infos_t;

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE VARIABLES -------------------------------------------------------------
 */
static relay_config_t          relay_config                                 = { 0 };
static relay_fwd_config_t      relay_fwd_cnt[LIMIT__LAST_ELT]               = { 0 };
static relay_fwd_join_list_t   device_list_join[SIZE_TAB_JOIN_REQ_LIST]     = { 0 };
static relay_fwd_uplink_list_t device_list_dev_addr[SIZE_TAB_DEV_ADDR_LIST] = { 0 };
static relay_infos_t           relay_info                                   = { 0 };
static relay_stats_t           relay_stat                                   = { 0 };
static wor_infos_t             relay_wor_info                               = { 0 };

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DECLARATION -------------------------------------------------
 */

/**
 * @brief Relay callback called by radio planner to manage CAD, WOR, WOR ACK and RX Uplink
 *
 * @param[in]   context     No used (NULL)
 */
static void callback_relay( void* context );

/**
 * @brief Relay callback to manage downlink on RXR
 *
 * @param[in]   context     No used (NULL)
 */
static void callback_relay_fwd( void* context );

/**
 * @brief Main relay FSM
 *
 * @param[in]   irq_timestamp_ms    Current time in ms
 * @param[in]   cad_success         True if CAD is positive
 * @param[in]   rx_success          True if a packet has been received
 */
static void manage_fsm( const uint32_t irq_timestamp_ms, const bool cad_success, const bool rx_success );

/**
 * @brief Enqueue the next CAD
 *
 * @param[in]   config      Relay configuration
 * @param[in]   info        Relay status
 */
static void config_enqueue_next_cad( const relay_config_t* config, relay_infos_t* info );

/**
 * @brief Configure the radio to perform a CAD follow by reception
 *
 * @param[in]   config      Relay configuration
 * @param[in]   info        Relay status
 */
static void config_cad_to_rx_wor( const relay_config_t* config, relay_infos_t* info );

/**
 * @brief Program the reception of the LoRaWAN Uplink
 *
 * @param[in]   config          Relay configuration
 * @param[in]   wor             Infos from the WOR
 * @param[in]   timestamp_wor   Timestamp when WOR has been received
 */
static void config_enqueue_rx_msg( const relay_config_t* config, const wor_infos_t* wor,
                                   const uint32_t timestamp_lr1_ul );

/**
 * @brief Program the emission of the WOR ACK
 *
 * @param[in]       config      Relay configuration
 * @param[in]       wor         WOR infos
 * @param[in]       time_rx     Timestamp to send the WOR ACK
 * @param[in,out]   info        Relay info
 * @param[in]       devaddr_idx ED Dev ADDR
 */
static void config_enqueue_ack( const relay_config_t* config, const wor_infos_t* wor, const uint32_t time_rx,
                                relay_infos_t* info, const uint8_t devaddr_idx );

/**
 * @brief Compute the difference between the IRQ TX Done on the ED and RX Done on the relay
 *
 * @param[in]   datarate    Datarate of the message
 * @param[in]   payload_len Length of the messages
 * @return int16_t  Difference in µS
 */
static int16_t ral_compute_rx_done_correction( uint8_t datarate, uint16_t payload_len );

/**
 * @brief Check if received MIC is valid
 *
 * @param[in]   wor         WOR infos
 * @param[in]   mic_receive MIC received
 * @param[out]  device_idx  If MIC is valid, contain the index in the trusted tables
 * @return true     MIC is valid (device_idx has a value between 0 and 15)
 * @return false    MIC is invalid (device_idx is set to 16)
 */
static bool is_mic_wor_valid( const wor_infos_t* wor, uint32_t mic_receive, uint8_t* device_idx );

/**
 * @brief Check if the relay is autorised to forward a new message
 *
 * @param[in]   wor         WOR infos
 * @param[in]   device_idx  ED index in the trusted table
 * @return true     Relay is allow to forward
 * @return false    Relay is NOT allow to forward
 */
static bool check_fwd_limitation( const wor_infos_t* wor, uint8_t device_idx );

/**
 * @brief Check if the relay could forward this Join Request
 *
 * @param[in]   deveui      ED Dev EUI
 * @param[in]   joineui     ED Join EUI
 * @return true     Relay is allow to forward
 * @return false    Relay is NOT allow to forward
 */
static bool check_forward_filter_joinreq( const uint8_t joineui[8], const uint8_t deveui[8] );

/**
 * @brief Get the datarate of the LoRaWAN uplink
 *
 * @param[in]   wor     WOR infos
 * @return uint8_t  Datarate value
 */
static uint8_t get_ul_dr( const wor_infos_t* wor );

/**
 * @brief Get the frequency of the LoRaWAN uplink
 *
 * @param[in]   wor     WOR infos
 * @return uint8_t  frequency value
 */
static uint32_t get_ul_freq( const wor_infos_t* wor );

/**
 * @brief Decrement the forward limit counter
 *
 * @param[in]   limit_counter   Limit counter
 */
static void decrement_fwd_counter( relay_fwd_config_t* limit_counter );

/**
 * @brief Relay has received an WOR Uplink and is not able to valid the MIC
 *
 * @param[in]   wor         WOR infos
 */
static void notify_unknonw_ed( const wor_infos_t* wor );

/**
 * @brief Relay has received a LoRaWAN Uplink and need to send it
 *
 * @param[in]   wor         Wor infos
 * @param[in]   config      Relay config
 * @param[in]   info        Relay status
 */
static void fwd_rx_msg( const wor_infos_t* wor, const relay_config_t* config, const relay_infos_t* info );

/**
 * @brief Relay Callback to send downlink on RXR
 *
 * @param[in]   rp_void     Radio planner pointer
 */
static void relay_rxr_tx_launch_callback( void* rp_void );

/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS DEFINITIONS --------------------------------------------------
 */

bool relay_init( lr1_stack_mac_t* lr1mac, wor_ack_ppm_error_t error_ppm, wor_ack_cad_to_rx_t cad_to_rx )
{
    if( rp_hook_init( lr1mac->rp, RP_HOOK_ID_RELAY_RX_CAD, callback_relay, NULL ) != RP_HOOK_STATUS_OK )
    {
        return false;
    }

    if( rp_hook_init( lr1mac->rp, RP_HOOK_ID_RELAY_FORWARD_RXR, callback_relay_fwd, NULL ) != RP_HOOK_STATUS_OK )
    {
        return false;
    }

    // Clear credential in forwarding list
    for( uint32_t i = 0; i < SIZE_TAB_DEV_ADDR_LIST; i++ )
    {
        device_list_dev_addr[i].in_use = false;
    }

    // Clean rules for join request forward
    device_list_join[0].action = RELAY_FILTER_FWD_TYPE_FORWARD;  // default is forward
    for( uint32_t i = 1; i < SIZE_TAB_JOIN_REQ_LIST; i++ )
    {
        device_list_join[i].action = RELAY_FILTER_FWD_TYPE_CLEAR;
    }

    relay_info.lr1mac    = lr1mac;
    relay_info.radio     = lr1mac->rp->radio;
    relay_info.error_ppm = error_ppm;
    relay_info.cad_to_rx = cad_to_rx;
    relay_info.state     = CAD_STATE_NO_CFG;

    // Set fwd limit with recommanded value in specification
    relay_fwd_cnt[LIMIT_OVERALL].reload_rate       = 8;
    relay_fwd_cnt[LIMIT_GLOBAL_UPLINK].reload_rate = 8;
    relay_fwd_cnt[LIMIT_NOTIFY].reload_rate        = 4;
    relay_fwd_cnt[LIMIT_JOINREQ].reload_rate       = 4;

    for( uint8_t i = 0; i < LIMIT__LAST_ELT; i++ )
    {
        relay_fwd_cnt[i].token_available = relay_fwd_cnt[i].reload_rate;
        relay_fwd_cnt[i].bucket_size     = relay_fwd_cnt[i].reload_rate * 2;  // Default bucket size is x2 reload rate
        relay_fwd_cnt[i].unlimited_fwd   = false;
    }

    return true;
}

bool relay_stop( bool stop_to_fwd )
{
    if( rp_task_abort( relay_info.lr1mac->rp, RP_HOOK_ID_RELAY_RX_CAD ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Failed to abort RP CAD \n" );
        return false;
    }

    if( rp_task_abort( relay_info.lr1mac->rp, RP_HOOK_ID_RELAY_FORWARD_RXR ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Failed to abort RP FWD RXR\n" );
        return false;
    }

    if( relay_info.state != CAD_STATE_NO_CFG )
    {
        relay_info.state = CAD_STATE_INIT;
    }
    SMTC_MODEM_HAL_TRACE_PRINTF( "Stop relay CAD\n" );
    relay_info.is_started = false;

    if( stop_to_fwd == true )
    {
        relay_info.dtc_state = smtc_duty_cycle_enable_get( );

        if( relay_info.dtc_state != SMTC_DTC_FULL_DISABLED )
        {
            smtc_duty_cycle_enable_set( SMTC_DTC_PARTIAL_DISABLED );
        }
    }

    return true;
}

bool relay_start( void )
{
    if( relay_info.state != CAD_STATE_INIT )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Start CAD status (%d)\n", relay_info.state );
        return false;
    }

    const smtc_dtc_enablement_type_t dtc_state = smtc_duty_cycle_enable_get( );
    if( dtc_state == SMTC_DTC_PARTIAL_DISABLED )
    {
        smtc_duty_cycle_enable_set( relay_info.dtc_state );
    }

    relay_info.current_ch_idx = relay_config.nb_wor_channel;  // to force to start at index 0
    relay_info.is_started     = true;

    config_enqueue_next_cad( &relay_config, &relay_info );

    const relay_channel_config_t* ch_cfg = &relay_config.channel_cfg[relay_info.current_ch_idx];

    SMTC_MODEM_HAL_TRACE_PRINTF( "Start CAD at %d ms every %d ms at DR%d %d Hz\n", relay_info.next_cad_ms,
                                 wor_convert_cad_period_in_ms( relay_config.cad_period ), ch_cfg->dr, ch_cfg->freq_hz );

    return true;
}

void relay_fwd_dl( uint8_t stack_id, const uint8_t* buffer, uint8_t len )
{
    memcpy( relay_info.buffer, buffer, len );
    relay_info.buffer_length = len;

    smtc_real_t* real = relay_info.lr1mac->real;

    // Downlink on RXR use same frequency as WOR frame and same DR as LoRaWAN uplink
    const uint8_t  ul_dr   = get_ul_dr( &relay_wor_info );
    const uint8_t  dl_dr   = smtc_real_get_rx1_datarate_config( real, ul_dr, 0 );
    const uint32_t dl_freq = relay_config.channel_cfg[relay_info.current_ch_idx].freq_hz;

    // Downlink could be LORA or FSK
    const modulation_type_t modulation_type = smtc_real_get_modulation_type_from_datarate( real, dl_dr );
    rp_radio_params_t       radio_params    = { 0 };

    rp_task_t rp_task = {
        .start_time_ms = relay_info.rx_uplink_timestamp_ms + RXR_WINDOWS_DELAY_S * 1000 -
                         smtc_modem_hal_get_radio_tcxo_startup_delay_ms( ),
        .hook_id               = RP_HOOK_ID_RELAY_FORWARD_RXR,
        .state                 = RP_TASK_STATE_SCHEDULE,
        .launch_task_callbacks = relay_rxr_tx_launch_callback,

    };

    if( modulation_type == LORA )
    {
        uint8_t             sf;
        lr1mac_bandwidth_t  bw;
        ralf_params_lora_t* lora_param = &radio_params.tx.lora;

        smtc_real_lora_dr_to_sf_bw( real, dl_dr, &sf, &bw );

        lora_param->rf_freq_in_hz     = dl_freq;
        lora_param->sync_word         = smtc_real_get_sync_word( real );
        lora_param->output_pwr_in_dbm = smtc_real_clamp_output_power_eirp_vs_freq_and_dr(
            real, smtc_real_get_default_max_eirp( real ), lora_param->rf_freq_in_hz, dl_dr );

        lora_param->mod_params.sf   = ( ral_lora_sf_t ) sf;
        lora_param->mod_params.bw   = ( ral_lora_bw_t ) bw;
        lora_param->mod_params.cr   = RAL_LORA_CR_4_5;
        lora_param->mod_params.ldro = ral_compute_lora_ldro( lora_param->mod_params.sf, lora_param->mod_params.bw );

        lora_param->pkt_params.preamble_len_in_symb = 8;
        lora_param->pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT;
        lora_param->pkt_params.pld_len_in_bytes     = len;
        lora_param->pkt_params.crc_is_on            = true;
        lora_param->pkt_params.invert_iq_is_on      = true;

        radio_params.pkt_type = RAL_PKT_TYPE_LORA;

        rp_task.type = RP_TASK_TYPE_TX_LORA;
        rp_task.duration_time_ms =
            ral_get_lora_time_on_air_in_ms( &relay_info.radio->ral, &lora_param->pkt_params, &lora_param->mod_params );

        SMTC_MODEM_HAL_TRACE_PRINTF( "LORA Tx on RXR %d bytes at DR%d  %d Hz - toa %d\n", len, dl_dr,
                                     radio_params.tx.lora.rf_freq_in_hz, rp_task.duration_time_ms );
    }
    else if( modulation_type == FSK )
    {
        uint8_t kbitrate;
        smtc_real_fsk_dr_to_bitrate( real, dl_dr, &kbitrate );

        ralf_params_gfsk_t* gfsk_param = &radio_params.tx.gfsk;

        gfsk_param->whitening_seed    = GFSK_WHITENING_SEED;
        gfsk_param->crc_seed          = GFSK_CRC_SEED;
        gfsk_param->crc_polynomial    = GFSK_CRC_POLYNOMIAL;
        gfsk_param->rf_freq_in_hz     = relay_config.channel_cfg[relay_info.current_ch_idx].freq_hz;
        gfsk_param->sync_word         = smtc_real_get_gfsk_sync_word( real );
        gfsk_param->output_pwr_in_dbm = smtc_real_clamp_output_power_eirp_vs_freq_and_dr(
            real, smtc_real_get_default_max_eirp( real ), gfsk_param->rf_freq_in_hz, dl_dr );

        gfsk_param->pkt_params.header_type           = RAL_GFSK_PKT_VAR_LEN;
        gfsk_param->pkt_params.pld_len_in_bytes      = len;
        gfsk_param->pkt_params.preamble_len_in_bits  = 40;
        gfsk_param->pkt_params.sync_word_len_in_bits = 24;
        gfsk_param->pkt_params.dc_free               = RAL_GFSK_DC_FREE_WHITENING;
        gfsk_param->pkt_params.crc_type              = RAL_GFSK_CRC_2_BYTES_INV;

        gfsk_param->mod_params.fdev_in_hz   = 25000;
        gfsk_param->mod_params.bw_dsb_in_hz = 100000;
        gfsk_param->mod_params.pulse_shape  = RAL_GFSK_PULSE_SHAPE_BT_1;
        gfsk_param->mod_params.br_in_bps    = kbitrate * 1000;

        radio_params.pkt_type = RAL_PKT_TYPE_GFSK;

        rp_task.type = RP_TASK_TYPE_TX_FSK;
        rp_task.duration_time_ms =
            ral_get_gfsk_time_on_air_in_ms( &relay_info.radio->ral, &gfsk_param->pkt_params, &gfsk_param->mod_params );

        SMTC_MODEM_HAL_TRACE_PRINTF( "FSK Tx on RXR %d kbit at %d Hz - toa %d\n", kbitrate,
                                     radio_params.tx.lora.rf_freq_in_hz, rp_task.duration_time_ms );
    }
    else
    {
        SMTC_MODEM_HAL_PANIC( "MODULATION NOT SUPPORTED\n" );
    }

    if( rp_task_enqueue( relay_info.lr1mac->rp, &rp_task, relay_info.buffer, relay_info.buffer_length,
                         &radio_params ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "RP is busy,fail in relay_fwd_dl\n" );
        relay_start( );
    }
}

bool relay_update_config( const relay_config_t* config )
{
    if( config == NULL )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "relay config null pointer\n" );
        return false;
    }
    if( config->cad_period >= WOR_CAD_PERIOD_RFU )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Wrong cad_period\n" );
        return false;
    }
    if( config->nb_wor_channel > MAX_WOR_CH )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Wrong nb_wor_channel (too high)\n" );
        return false;
    }
    if( config->nb_wor_channel == 0 )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Wrong nb_wor_channel (too low)\n" );
        return false;
    }

    for( uint8_t i = 0; i < config->nb_wor_channel; i++ )
    {
        if( smtc_real_get_modulation_type_from_datarate( relay_info.lr1mac->real, config->channel_cfg[i].dr ) != LORA )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Invalid dr (%d) on ch %d\n", config->channel_cfg[i].dr, i );
            return false;
        }
        if( smtc_real_is_frequency_valid( relay_info.lr1mac->real, config->channel_cfg[i].freq_hz ) != OKLORAWAN )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Invalid freq wor %d  on ch %d\n", config->channel_cfg[i].freq_hz, i );
            return false;
        }
        if( smtc_real_is_frequency_valid( relay_info.lr1mac->real, config->channel_cfg[i].ack_freq_hz ) != OKLORAWAN )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Invalid freq wor ack %d on ch %d\n", config->channel_cfg[i].ack_freq_hz, i );
            return false;
        }
    }

    // Memorise if relay is running to restart it (or not) at the end
    // bool relay_is_running = false;
    // if( relay_info.state > CAD_STATE_INIT )
    // {
    //     relay_stop( false );
    //     relay_is_running = true;
    // }

    // All parameters are valid -> save config
    relay_config = *config;

    // Compute TOA of WOR and IRQ timing compensation
    rp_radio_params_t param;
    for( uint8_t i = 0; i < config->nb_wor_channel; i++ )
    {
        wor_ral_init_tx_wor( relay_info.lr1mac->real, relay_config.channel_cfg[i].dr,
                             relay_config.channel_cfg[i].freq_hz, 0, WOR_UPLINK_LENGTH, &param );
        // Use WOR_UPLINK_LENGTH because WOR ACK is only send to uplink and there is almost no difference in t irq
        // compensation between a join and an uplink

        relay_info.t_irq_comp_wor_ms[i] =
            ral_compute_rx_done_correction( relay_config.channel_cfg[i].dr, WOR_UPLINK_LENGTH ) / 1000;

        relay_info.wor_toa_ms[i] = ral_get_lora_time_on_air_in_ms( &relay_info.radio->ral, &param.tx.lora.pkt_params,
                                                                   &param.tx.lora.mod_params );
    }

    if( relay_info.state == CAD_STATE_NO_CFG )
    {
        relay_info.state = CAD_STATE_INIT;
    }

    return true;
}

void relay_fwd_join_request_update_rule( const uint8_t idx, const uint8_t* join_eui, const uint8_t len_join_eui,
                                         const uint8_t* dev_eui, const uint8_t len_dev_eui,
                                         const relay_filter_fwd_type_t action )
{
    // Rules are checked in parser
    if( ( idx < SIZE_TAB_JOIN_REQ_LIST ) && ( len_join_eui <= 8 ) && ( len_dev_eui <= 8 ) )
    {
        memcpy( device_list_join[idx].eui, join_eui, len_join_eui );
        memcpy( device_list_join[idx].eui + len_join_eui, dev_eui, len_dev_eui );
        device_list_join[idx].len    = len_join_eui + len_dev_eui;
        device_list_join[idx].action = action;
    }
}

bool relay_fwd_uplink_add_device( const uint8_t idx, const uint32_t dev_addr, const uint8_t* root_wor_skey,
                                  const bool unlimited_fwd, const uint8_t bucket_factor, const uint8_t reload_rate,
                                  const uint32_t wfcnt32 )
{
    if( idx >= SIZE_TAB_DEV_ADDR_LIST )
    {
        return false;
    }

    relay_fwd_uplink_list_t* device = &device_list_dev_addr[idx];

    wor_derive_keys( root_wor_skey, dev_addr, device->wor_s_int_key, device->wor_s_enc_key );

    device->dev_addr = dev_addr;
    device->wfcnt32  = wfcnt32;
    device->in_use   = true;

    device->fwd_cfg.unlimited_fwd   = unlimited_fwd;
    device->fwd_cfg.reload_rate     = reload_rate;
    device->fwd_cfg.bucket_size     = bucket_factor * reload_rate;
    device->fwd_cfg.token_available = bucket_factor * reload_rate;

    return true;
}

bool relay_fwd_uplink_remove_device( const uint8_t idx )
{
    if( ( idx < SIZE_TAB_DEV_ADDR_LIST ) && ( device_list_dev_addr[idx].in_use == true ) )
    {
        device_list_dev_addr[idx].in_use = false;
        return true;
    }

    return false;
}

bool relay_fwd_uplink_read_wfcnt32( const uint8_t idx, uint32_t* wfcnt32 )
{
    if( ( idx < SIZE_TAB_DEV_ADDR_LIST ) && ( device_list_dev_addr[idx].in_use == true ) )
    {
        if( wfcnt32 != NULL )
        {
            *wfcnt32 = device_list_dev_addr[idx].wfcnt32;
        }
        return true;
    }

    return false;
}

void relay_get_stats( relay_stats_t* stat )
{
    if( stat != NULL )
    {
        *stat = relay_stat;
    }
}

void relay_print_stats( void )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "------------------------\nRelay stat at %d s\n------------------------\n",
                                 smtc_modem_hal_get_time_in_s( ) );
    SMTC_MODEM_HAL_TRACE_PRINTF( " - Cad1       %d \n", relay_stat.nb_cad1 );
    SMTC_MODEM_HAL_TRACE_PRINTF( " - Cad2       %d \n", relay_stat.nb_cad2 );
    SMTC_MODEM_HAL_TRACE_PRINTF( " - Cad2 OK    %d \n", relay_stat.nb_cad2_ok );
    SMTC_MODEM_HAL_TRACE_PRINTF( " - WOR fail   %d \n", relay_stat.nb_wor_fail );
    SMTC_MODEM_HAL_TRACE_PRINTF( " - WOR ok     %d \n", relay_stat.nb_wor_ok );
    SMTC_MODEM_HAL_TRACE_PRINTF( " - Rx_ok      %d \n", relay_stat.nb_rx_ok );
    SMTC_MODEM_HAL_TRACE_PRINTF( " - Rx_fail    %d \n", relay_stat.nb_rx_fail );
    SMTC_MODEM_HAL_TRACE_PRINTF( " - Ack_tx     %d \n", relay_stat.nb_ack_tx );
}

void relay_fwd_update_fwd_limit( const relay_forward_limit_list_t limit, const relay_forward_limit_action_t action,
                                 const bool unlimitted, const uint16_t reload_rate, const uint8_t factor )
{
    if( limit < LIMIT__LAST_ELT )
    {
        relay_fwd_cnt[limit].unlimited_fwd = unlimitted;
        relay_fwd_cnt[limit].reload_rate   = reload_rate;
        relay_fwd_cnt[limit].bucket_size   = reload_rate * factor;

        switch( action )
        {
        case LIMIT_CNT_SET_TO_0:
            relay_fwd_cnt[limit].token_available = 0;
            break;
        case LIMIT_CNT_SET_TO_RELOAD_RATE:
            relay_fwd_cnt[limit].token_available = reload_rate;
            break;
        case LIMIT_CNT_SET_TO_MAX_VAL:
            relay_fwd_cnt[limit].token_available = reload_rate * factor;
            break;
        default:
        case LIMIT_CNT_DONT_CHANGE:
            break;
        }
    }
}

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DEFINITIONS -------------------------------------------------
 */
static void callback_relay_fwd( void* context )
{
    uint32_t    irq_timestamp_ms;
    rp_status_t planner_status;

    rp_get_status( relay_info.lr1mac->rp, RP_HOOK_ID_RELAY_FORWARD_RXR, &irq_timestamp_ms, &planner_status );

    switch( planner_status )
    {
    case RP_STATUS_TX_DONE:
        SMTC_MODEM_HAL_TRACE_PRINTF( "TX DONE on RXR\n" );
        break;

    case RP_STATUS_TASK_ABORTED:
        SMTC_MODEM_HAL_TRACE_WARNING( "Fwd Task aborted\n" );
        break;

    default:
        SMTC_MODEM_HAL_TRACE_WARNING( "callback_relay_fwd default switch: 0x%x\n", planner_status );
        break;
    }
    relay_start( );
}

static void callback_relay( void* context )
{
    uint32_t    irq_timestamp_ms;
    rp_status_t planner_status;
    bool        cad_success = false;
    bool        rx_success  = false;

    rp_get_status( relay_info.lr1mac->rp, RP_HOOK_ID_RELAY_RX_CAD, &irq_timestamp_ms, &planner_status );

    switch( planner_status )
    {
    case RP_STATUS_CAD_POSITIVE:
        cad_success = true;
        break;

    case RP_STATUS_RX_PACKET: {
        uint8_t                       sf;
        lr1mac_bandwidth_t            bw;
        const relay_channel_config_t* channel_cfg = &relay_config.channel_cfg[relay_info.current_ch_idx];
        smtc_real_lora_dr_to_sf_bw( relay_info.lr1mac->real, channel_cfg->dr, &sf, &bw );

        rx_success               = true;
        relay_info.buffer_length = relay_info.lr1mac->rp->rx_payload_size[RP_HOOK_ID_RELAY_RX_CAD];
        relay_info.rx_status     = relay_info.lr1mac->rp->radio_params[RP_HOOK_ID_RELAY_RX_CAD].rx.lora_pkt_status;
    }
    break;

    case RP_STATUS_CAD_NEGATIVE:
    case RP_STATUS_RX_TIMEOUT:
    case RP_STATUS_RX_CRC_ERROR:
    case RP_STATUS_TX_DONE:
        break;

    case RP_STATUS_TASK_ABORTED:
        SMTC_MODEM_HAL_TRACE_WARNING( "Relay Task aborted\n" );
        relay_info.state = CAD_STATE_ENQUEUE_CAD;
        break;

    default:
        SMTC_MODEM_HAL_TRACE_WARNING( "callback_relay default switch: 0x%x\n", planner_status );
        break;
    }

    manage_fsm( irq_timestamp_ms, cad_success, rx_success );
}

static void manage_fsm( const uint32_t irq_timestamp_ms, const bool cad_success, const bool rx_success )
{
    switch( relay_info.state )
    {
    case CAD_STATE_INIT:
        // Do nothing
        break;

    case CAD_STATE_ENQUEUE_CAD:
        config_enqueue_next_cad( &relay_config, &relay_info );
        break;

    case CAD_STATE_WAIT_CAD_COMPLETION:
        relay_stat.nb_cad1 += 1;
        if( cad_success == false )
        {
            // CAD has failed, program the next CAD
            config_enqueue_next_cad( &relay_config, &relay_info );
        }
        else
        {
            // CAD has succeed, radio is still for us so configure cad to rx
            config_cad_to_rx_wor( &relay_config, &relay_info );
            relay_stat.nb_cad2 += 1;
        }
        break;

    case CAD_STATE_WAIT_WOR_COMPLETION: {
        if( cad_success == true )
        {
            relay_stat.nb_cad2_ok += 1;

            // This is an intermediate interrupt that should be ignore has the main wor message is being received
            return;
        }
        // Here complete message has been received or radio is in timeout

        if( rx_success == false )
        {
            // Failed to received WOR frame
            relay_stat.nb_wor_fail += 1;
            config_enqueue_next_cad( &relay_config, &relay_info );
            return;
        }

        // SMTC_MODEM_HAL_TRACE_ARRAY( "WOR RX ", relay_info.buffer, relay_info.buffer_length );

        if( wor_extract_wor_info( relay_info.buffer, relay_info.buffer_length, &relay_wor_info ) == false )
        {
            SMTC_MODEM_HAL_TRACE_MSG( "Bad WOR\n" );
            config_enqueue_next_cad( &relay_config, &relay_info );
            return;
        }
        // A correct WOR has been received (MIC not yet checked)
        relay_stat.nb_wor_ok += 1;
        bool mic_is_valid = false;

        // Save RF WOR infos
        relay_wor_info.rf_infos.freq_hz           = relay_config.channel_cfg[relay_info.current_ch_idx].freq_hz;
        relay_wor_info.rf_infos.dr                = relay_config.channel_cfg[relay_info.current_ch_idx].dr;
        relay_wor_info.rf_infos.rssi_in_dbm       = relay_info.rx_status.rssi_pkt_in_dbm;
        relay_wor_info.rf_infos.signal_rssi_in_db = relay_info.rx_status.signal_rssi_pkt_in_dbm;
        relay_wor_info.rf_infos.snr_in_db         = relay_info.rx_status.snr_pkt_in_db;

        // Raz to wrong value
        // devaddr_idx = SIZE_TAB_DEV_ADDR_LIST;
        relay_info.rx_msg_devaddr_idx = SIZE_TAB_DEV_ADDR_LIST;

        if( relay_wor_info.wor_type == WOR_MSG_TYPE_STANDARD_UPLINK )
        {
            const uint32_t mic_receive = wor_extract_mic_wor_uplink( relay_info.buffer );
            mic_is_valid = is_mic_wor_valid( &relay_wor_info, mic_receive, &relay_info.rx_msg_devaddr_idx );
            if( mic_is_valid == true )
            {
                wor_decode_wor_enc_data( relay_wor_info.uplink.enc_data, &relay_wor_info.uplink,
                                         &relay_wor_info.rf_infos,
                                         device_list_dev_addr[relay_info.rx_msg_devaddr_idx].wor_s_enc_key );
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "Invalid MIC\n" );
            }
        }

        if( ( relay_wor_info.wor_type == WOR_MSG_TYPE_JOIN_REQUEST ) ||
            ( ( relay_wor_info.wor_type == WOR_MSG_TYPE_STANDARD_UPLINK ) && ( mic_is_valid == true ) ) )
        {
            const uint8_t  ul_dr   = get_ul_dr( &relay_wor_info );
            const uint32_t ul_freq = get_ul_freq( &relay_wor_info );

            if( smtc_real_is_tx_dr_valid( relay_info.lr1mac->real, ul_dr ) != OKLORAWAN )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "WOR Wrong DR (%d)\n", ul_dr );
                config_enqueue_next_cad( &relay_config, &relay_info );
                return;
            }
            if( smtc_real_is_frequency_valid( relay_info.lr1mac->real, ul_freq ) != OKLORAWAN )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "WOR wrong freq (%d)\n", ul_freq );
                config_enqueue_next_cad( &relay_config, &relay_info );
                return;
            }
        }

        if( check_fwd_limitation( &relay_wor_info, relay_info.rx_msg_devaddr_idx ) != true )
        {
            SMTC_MODEM_HAL_TRACE_MSG( "Forward limitation exceed\n" );
            config_enqueue_next_cad( &relay_config, &relay_info );
            return;
        }

        // Compensate RX TX done jitter
        relay_info.rx_wor_timestamp_ms = irq_timestamp_ms - relay_info.t_irq_comp_wor_ms[relay_info.current_ch_idx];
        // SMTC_MODEM_HAL_TRACE_PRINTF( " Rx WOR %d %d %d\n", relay_info.rx_wor_timestamp_ms, irq_timestamp_ms,
        //                              relay_info.t_irq_comp_wor_ms[relay_info.current_ch_idx] );

        if( relay_wor_info.wor_type == WOR_MSG_TYPE_JOIN_REQUEST )
        {
            // SMTC_MODEM_HAL_TRACE_MSG( "WOR Join Request receive\n" );
            config_enqueue_rx_msg( &relay_config, &relay_wor_info,
                                   relay_info.rx_wor_timestamp_ms + DELAY_WOR_TO_JOINREQ_MS );
            return;
        }
        else if( relay_wor_info.wor_type == WOR_MSG_TYPE_STANDARD_UPLINK )
        {
            relay_info.t_offset = relay_info.rx_wor_timestamp_ms - relay_info.last_cad_ms -
                                  relay_info.wor_toa_ms[relay_info.current_ch_idx];

            // SMTC_MODEM_HAL_TRACE_MSG( "WOR Standard Uplink receive:\n" );
            // SMTC_MODEM_HAL_TRACE_PRINTF( " - Dev Addr : 0x%08X\n", relay_wor_info.uplink.devaddr );
            // SMTC_MODEM_HAL_TRACE_PRINTF( " - FCNT     : %d\n", relay_wor_info.uplink.fcnt );
            // SMTC_MODEM_HAL_TRACE_PRINTF( " - MIC      : %s\n", ( mic_is_valid ? " ok" : "nok" ) );
            // SMTC_MODEM_HAL_TRACE_PRINTF( " - T offset : %d ( %d - %d - %d)\n", relay_info.t_offset,
            //                              relay_info.rx_wor_timestamp_ms, relay_info.last_cad_ms,
            //                              relay_info.wor_toa_ms[relay_info.current_ch_idx] );

            if( mic_is_valid == true )
            {
                relay_stat.nb_ack_tx += 1;
                config_enqueue_ack( &relay_config, &relay_wor_info, relay_info.rx_wor_timestamp_ms, &relay_info,
                                    relay_info.rx_msg_devaddr_idx );
                return;
            }
            else
            {
                notify_unknonw_ed( &relay_wor_info );
                config_enqueue_next_cad( &relay_config, &relay_info );
                SMTC_MODEM_HAL_TRACE_PRINTF( "Unknown ED (0x%08X) \n", relay_wor_info.uplink.devaddr );
                return;
            }
        }
        else
        {
            SMTC_MODEM_HAL_PANIC( );  // never happen - manage earlier
        }
    }
    break;

    case CAD_STATE_WAIT_TX_ACK_COMPLETION:
        relay_info.tx_ack_timestamp_ms = irq_timestamp_ms;
        config_enqueue_rx_msg( &relay_config, &relay_wor_info,
                               relay_info.tx_ack_timestamp_ms + DELAY_WORACK_TO_UPLINK_MS );
        break;

    case CAD_STATE_WAIT_RX_DATA_COMPLETION:
        if( rx_success == false )
        {
            relay_stat.nb_rx_fail += 1;
            SMTC_MODEM_HAL_TRACE_MSG( "Missed data :( \n" );
            config_enqueue_next_cad( &relay_config, &relay_info );
        }
        else
        {
            relay_stat.nb_rx_ok += 1;
            // SMTC_MODEM_HAL_TRACE_MSG( "RX sucess :)\n" );
            bool fwd_msg = true;

            if( relay_wor_info.wor_type == WOR_MSG_TYPE_JOIN_REQUEST )
            {
                fwd_msg = check_forward_filter_joinreq( relay_info.buffer + 1, relay_info.buffer + 9 );
            }

            uint16_t max_relayed_len =
                smtc_real_get_max_payload_size( relay_info.lr1mac->real, relay_info.lr1mac->tx_data_rate, UP_LINK );

            if( max_relayed_len > RELAY_OVERHEAD_FORWARD )
            {
                max_relayed_len -= RELAY_OVERHEAD_FORWARD;
            }
            else
            {
                max_relayed_len = 0;
            }

            if( relay_info.buffer_length > max_relayed_len )
            {
                fwd_msg = false;
            }

            if( ( relay_info.buffer_length + RELAY_OVERHEAD_FORWARD ) > 255 )
            {
                fwd_msg = false;
            }

            // Standard uplink has already been check to forward (or not) when WOR MIC is check
            const int16_t rx_correct_us =
                ral_compute_rx_done_correction( get_ul_dr( &relay_wor_info ), relay_info.buffer_length );
            relay_info.rx_uplink_timestamp_ms = irq_timestamp_ms - ( rx_correct_us / 1000 );

            if( fwd_msg == true )
            {
                fwd_rx_msg( &relay_wor_info, &relay_config, &relay_info );
            }
            else
            {
                config_enqueue_next_cad( &relay_config, &relay_info );
            }
        }
        break;

    default:
        SMTC_MODEM_HAL_PANIC( "Impossible value\n" );
        break;
    }
}

static void config_enqueue_next_cad( const relay_config_t* config, relay_infos_t* info )
{
    if( relay_info.is_started == false )
    {
        SMTC_MODEM_HAL_TRACE_MSG( "Enqueue CAD but stop\n" );
        return;
    }

    const int32_t duty_cycle_ms = modem_duty_cycle_get_status( RELAY_STACK_ID );
    uint32_t      actual_ms     = smtc_modem_hal_get_time_in_ms( ) + 10;
    if( duty_cycle_ms > 0 )  // No DTC available until X ms -> move "now" to a time where DTC is OK
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "DTC - wait %d ms to re-enable relay\n", duty_cycle_ms );
        actual_ms += duty_cycle_ms;
    }

    const uint32_t cad_period_ms     = wor_convert_cad_period_in_ms( config->cad_period ) / config->nb_wor_channel;
    uint32_t       next_cad_start_ms = info->last_cad_ms;

    while( ( ( int ) ( next_cad_start_ms - actual_ms ) <= 0 ) )
    {
        next_cad_start_ms += cad_period_ms;
        info->current_ch_idx += 1;
    }
    // SMTC_MODEM_HAL_TRACE_PRINTF( "Next CAD at %d\n", next_cad_start_ms );

    info->current_ch_idx %= config->nb_wor_channel;
    info->next_cad_ms = next_cad_start_ms;
    info->last_cad_ms = next_cad_start_ms;

    // default value of default channel even if cad period is smaller
    wor_cad_periodicity_t cad_period = WOR_CAD_PERIOD_1S;
    if( info->current_ch_idx != 0 )
    {
        cad_period = config->cad_period;
    }

    rp_radio_params_t             rx_param    = { 0 };
    const relay_channel_config_t* channel_cfg = &config->channel_cfg[info->current_ch_idx];
    wor_ral_init_rx_wor( relay_info.lr1mac->real, channel_cfg->dr, channel_cfg->freq_hz, cad_period,
                         MAX( ( uint8_t ) WOR_JOINREQ_LENGTH, ( uint8_t ) WOR_UPLINK_LENGTH ), &rx_param );
    wor_ral_init_cad( relay_info.lr1mac->real, channel_cfg->dr, cad_period, true,
                      relay_info.wor_toa_ms[info->current_ch_idx], &rx_param.rx.cad );

    const rp_task_t rp_task_cad = {
        .hook_id                    = RP_HOOK_ID_RELAY_RX_CAD,
        .type                       = RP_TASK_TYPE_CAD_TO_RX,
        .state                      = RP_TASK_STATE_SCHEDULE,
        .schedule_task_low_priority = true,
        .duration_time_ms           = 20000,
        .start_time_ms              = info->next_cad_ms - smtc_modem_hal_get_radio_tcxo_startup_delay_ms( ),
        .launch_task_callbacks      = wor_ral_callback_start_cad,
    };

    if( rp_task_enqueue( relay_info.lr1mac->rp, &rp_task_cad, info->buffer, 255, &rx_param ) == RP_HOOK_STATUS_OK )
    {
        info->state = CAD_STATE_WAIT_CAD_COMPLETION;
    }
    else
    {
        SMTC_MODEM_HAL_PANIC( "RP is busy, fail in config_enqueue_next_cad\n" );
    }
}

static void config_cad_to_rx_wor( const relay_config_t* config, relay_infos_t* info )
{
    const wor_cad_periodicity_t   cad_period  = ( info->current_ch_idx != 0 ) ? config->cad_period : WOR_CAD_PERIOD_1S;
    const relay_channel_config_t* channel_cfg = &config->channel_cfg[info->current_ch_idx];

    ral_lora_cad_params_t cad_param = { 0 };

    wor_ral_init_cad( relay_info.lr1mac->real, channel_cfg->dr, cad_period, false,
                      relay_info.wor_toa_ms[info->current_ch_idx], &cad_param );

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_lora_cad_params( &( relay_info.radio->ral ), &cad_param ) ==
                                     RAL_STATUS_OK );

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_lora_cad( &( relay_info.radio->ral ) ) == RAL_STATUS_OK );

    info->state = CAD_STATE_WAIT_WOR_COMPLETION;
}

static void config_enqueue_rx_msg( const relay_config_t* config, const wor_infos_t* wor,
                                   const uint32_t timestamp_lr1_ul )
{
    const uint8_t  ul_dr   = get_ul_dr( &relay_wor_info );
    const uint32_t ul_freq = get_ul_freq( &relay_wor_info );

    // SMTC_MODEM_HAL_TRACE_PRINTF( "Config RX at DR%d at %d Hz\n", ul_dr, ul_freq );

    // Max payload to receive: min value of payload size between (ED/Relay) and (Relay/GTW minus relay overhead)
    uint8_t max_rx =
        smtc_real_get_max_payload_size( relay_info.lr1mac->real, relay_info.lr1mac->tx_data_rate, UP_LINK );
    if( max_rx > RELAY_OVERHEAD_FORWARD )
    {
        max_rx -= RELAY_OVERHEAD_FORWARD;
        max_rx = MIN( smtc_real_get_max_payload_size( relay_info.lr1mac->real, ul_dr, UP_LINK ), max_rx );
    }
    else
    {
        max_rx = 0;
    }

    rp_radio_params_t rp_radio_params = { 0 };
    wor_ral_init_rx_msg( relay_info.lr1mac->real, max_rx, ul_dr, ul_freq, &rp_radio_params );

    rp_task_t rx_task = {
        .hook_id               = RP_HOOK_ID_RELAY_RX_CAD,
        .state                 = RP_TASK_STATE_SCHEDULE,
        .start_time_ms         = timestamp_lr1_ul - smtc_modem_hal_get_radio_tcxo_startup_delay_ms( ),
        .launch_task_callbacks = wor_ral_callback_start_rx,
    };

    if( rp_radio_params.pkt_type == RAL_PKT_TYPE_LORA )
    {
        rx_task.type             = RP_TASK_TYPE_RX_LORA;
        rx_task.duration_time_ms = ral_get_lora_time_on_air_in_ms(
            &relay_info.radio->ral, &rp_radio_params.rx.lora.pkt_params, &rp_radio_params.rx.lora.mod_params );
    }
    else if( rp_radio_params.pkt_type == RAL_PKT_TYPE_GFSK )
    {
        rx_task.type             = RP_TASK_TYPE_RX_FSK;
        rx_task.duration_time_ms = ral_get_gfsk_time_on_air_in_ms(
            &relay_info.radio->ral, &rp_radio_params.rx.gfsk.pkt_params, &rp_radio_params.rx.gfsk.mod_params );
    }
    else
    {
        SMTC_MODEM_HAL_PANIC( "RX message modulation unsupported\n" );
    }

    rp_radio_params.rx.timeout_in_ms = rx_task.duration_time_ms;

    if( rp_task_enqueue( relay_info.lr1mac->rp, &rx_task, relay_info.buffer, 255, &rp_radio_params ) ==
        RP_HOOK_STATUS_OK )
    {
        relay_info.state = CAD_STATE_WAIT_RX_DATA_COMPLETION;
    }
    else
    {
        SMTC_MODEM_HAL_PANIC( "RP is busy\n" );
    }
}

static void config_enqueue_ack( const relay_config_t* config, const wor_infos_t* wor, const uint32_t time_rx,
                                relay_infos_t* info, const uint8_t devaddr_idx )
{
    // SMTC_MODEM_HAL_TRACE_MSG( "Prepare ACK \n" );

    bool fwd_ok = true;

    if( modem_duty_cycle_get_status( RELAY_STACK_ID ) > 0 )
    {
        fwd_ok = false;
    }
    else if( check_fwd_limitation( wor, devaddr_idx ) == false )
    {
        fwd_ok = false;
    }

    const wor_ack_infos_t ack = {
        .dr_relay_gtw = info->lr1mac->tx_data_rate,
        .t_offset     = info->t_offset,
        .period       = config->cad_period,
        .relay_ppm    = info->error_ppm,
        .cad_to_rx    = info->cad_to_rx,
        .relay_fwd    = ( fwd_ok == true ? WOR_ACK_FORWARD_OK : WOR_ACK_FORWARD_RETRY_60MIN ),
    };

    const relay_channel_config_t* channel_cfg = &config->channel_cfg[info->current_ch_idx];

    const wor_ack_mic_info_t mic_info = {
        .dev_addr             = device_list_dev_addr[devaddr_idx].dev_addr,
        .wfcnt                = device_list_dev_addr[devaddr_idx].wfcnt32,
        .wor_datarate         = channel_cfg->dr,
        .wor_frequency_hz     = channel_cfg->freq_hz,
        .wor_ack_datarate     = channel_cfg->dr,
        .wor_ack_frequency_hz = channel_cfg->ack_freq_hz,
    };

    info->buffer_length =
        wor_generate_ack( info->buffer, &ack, &mic_info, device_list_dev_addr[devaddr_idx].wor_s_int_key,
                          device_list_dev_addr[devaddr_idx].wor_s_enc_key );

    rp_radio_params_t radio_params = { 0 };
    wor_ral_init_tx_ack( relay_info.lr1mac->real, channel_cfg->dr, channel_cfg->ack_freq_hz, info->buffer_length,
                         &radio_params );

    const rp_task_t rp_task = {
        .start_time_ms         = time_rx + DELAY_WOR_TO_WORACK_MS - smtc_modem_hal_get_radio_tcxo_startup_delay_ms( ),
        .hook_id               = RP_HOOK_ID_RELAY_RX_CAD,
        .state                 = RP_TASK_STATE_SCHEDULE,
        .launch_task_callbacks = wor_ral_callback_start_tx,
        .type                  = RP_TASK_TYPE_TX_LORA,
        .duration_time_ms      = ral_get_lora_time_on_air_in_ms(
            ( &relay_info.radio->ral ), ( &radio_params.tx.lora.pkt_params ), ( &radio_params.tx.lora.mod_params ) ),
    };

    // SMTC_MODEM_HAL_TRACE_ARRAY( "TX WOR ACK", info->buffer, info->buffer_length );
    // SMTC_MODEM_HAL_TRACE_PRINTF( "At DR%d at %d Hz - toa %d\n", channel_cfg->dr, channel_cfg->ack_freq_hz,
    //                              rp_task.duration_time_ms );

    if( rp_task_enqueue( info->lr1mac->rp, &rp_task, relay_info.buffer, relay_info.buffer_length, &radio_params ) ==
        RP_HOOK_STATUS_OK )
    {
        relay_info.state = CAD_STATE_WAIT_TX_ACK_COMPLETION;
    }
    else
    {
        SMTC_MODEM_HAL_PANIC( "RP is busy,fail in config_enqueue_ack" );
    }
}

static bool is_mic_wor_valid( const wor_infos_t* wor, uint32_t mic_receive, uint8_t* device_idx )
{
    for( uint8_t i = 0; i < SIZE_TAB_DEV_ADDR_LIST; i++ )
    {
        if( ( device_list_dev_addr[i].in_use == true ) && ( device_list_dev_addr[i].dev_addr == wor->uplink.devaddr ) )
        {
            uint32_t fcnt = device_list_dev_addr[i].wfcnt32;
            // Check if rollover on fcnt
            if( wor->uplink.fcnt <= ( uint16_t )( fcnt & MAX_UINT16 ) )
            {
                fcnt += MAX_UINT16;
            }
            // Clear 16 LSB and update with receive value
            fcnt &= ~( MAX_UINT16 );
            fcnt += wor->uplink.fcnt;

            const wor_mic_infos_t wor_mic_info = {
                .dev_addr = device_list_dev_addr[i].dev_addr,
                .wfcnt    = fcnt,
            };

            const uint32_t mic_calc =
                wor_compute_mic_wor( &wor_mic_info, wor->uplink.enc_data, device_list_dev_addr[i].wor_s_int_key );

            if( mic_receive == mic_calc )
            {
                *device_idx                     = i;
                device_list_dev_addr[i].wfcnt32 = fcnt;
                return true;
            }
        }
    }

    return false;
}

static void fwd_rx_msg( const wor_infos_t* wor, const relay_config_t* config, const relay_infos_t* info )
{
    if( wor->wor_type == WOR_MSG_TYPE_JOIN_REQUEST )
    {
        decrement_fwd_counter( &relay_fwd_cnt[LIMIT_JOINREQ] );
    }
    else if( wor->wor_type == WOR_MSG_TYPE_STANDARD_UPLINK )
    {
        decrement_fwd_counter( &relay_fwd_cnt[LIMIT_GLOBAL_UPLINK] );
        decrement_fwd_counter( &device_list_dev_addr[info->rx_msg_devaddr_idx].fwd_cfg );
    }
    else
    {
        SMTC_MODEM_HAL_PANIC( );
    }

    decrement_fwd_counter( &relay_fwd_cnt[LIMIT_OVERALL] );

    SMTC_MODEM_HAL_TRACE_PRINTF( "UL metadata - %d bytes at DR%d %dHz, WOR ch%d\n", info->buffer_length,
                                 get_ul_dr( &relay_wor_info ), get_ul_freq( &relay_wor_info ), info->current_ch_idx );
    SMTC_MODEM_HAL_TRACE_PRINTF( "rssi %d - snr %d - signal %d\n", info->rx_status.rssi_pkt_in_dbm,
                                 info->rx_status.snr_pkt_in_db, info->rx_status.signal_rssi_pkt_in_dbm );

    int16_t  tmp;
    uint32_t uplink_metadata = 0;
    uplink_metadata |= RELAY_FWD_UPLINK_SET_METADATA_WOR_CH( info->current_ch_idx );

    tmp = info->rx_status.rssi_pkt_in_dbm;
    tmp = MIN( -15, tmp );
    tmp = MAX( -142, tmp );
    uplink_metadata |= RELAY_FWD_UPLINK_SET_METADATA_UPLINK_RSSI( ( uint8_t )( -( tmp + 15 ) ) );

    tmp = info->rx_status.snr_pkt_in_db;
    tmp = MIN( 11, tmp );
    tmp = MAX( -20, tmp );
    uplink_metadata |= RELAY_FWD_UPLINK_SET_METADATA_UPLINK_SNR( ( uint8_t )( tmp + 20 ) );

    uplink_metadata |= RELAY_FWD_UPLINK_SET_METADATA_UPLINK_DR( get_ul_dr( &relay_wor_info ) );

    uint8_t buffer[255];
    buffer[RELAY_FWD_UPLINK_BYTE_ORDER_UPLINK_METADATA_7_0]   = ( uint8_t )( uplink_metadata );
    buffer[RELAY_FWD_UPLINK_BYTE_ORDER_UPLINK_METADATA_15_8]  = ( uint8_t )( uplink_metadata >> 8 );
    buffer[RELAY_FWD_UPLINK_BYTE_ORDER_UPLINK_METADATA_23_16] = ( uint8_t )( uplink_metadata >> 16 );

    const uint32_t freq_step = get_ul_freq( &relay_wor_info ) / 100;

    buffer[RELAY_FWD_UPLINK_BYTE_ORDER_UPLINK_FREQUENCY_7_0]   = ( uint8_t )( freq_step );
    buffer[RELAY_FWD_UPLINK_BYTE_ORDER_UPLINK_FREQUENCY_15_8]  = ( uint8_t )( freq_step >> 8 );
    buffer[RELAY_FWD_UPLINK_BYTE_ORDER_UPLINK_FREQUENCY_23_16] = ( uint8_t )( freq_step >> 16 );

    memcpy( buffer + RELAY_FWD_UPLINK_BYTE_ORDER_UPLINK_PAYLOAD, info->buffer, info->buffer_length );

    lorawan_relay_rx_fwd_uplink(
        RELAY_STACK_ID, buffer, info->buffer_length + RELAY_FWD_UPLINK_BYTE_ORDER_UPLINK_PAYLOAD,
        info->rx_uplink_timestamp_ms + RELAY_FWD_DELAY, ( wor->wor_type == WOR_MSG_TYPE_JOIN_REQUEST ? true : false ) );
}

static void notify_unknonw_ed( const wor_infos_t* wor )
{
    decrement_fwd_counter( &relay_fwd_cnt[LIMIT_OVERALL] );
    decrement_fwd_counter( &relay_fwd_cnt[LIMIT_NOTIFY] );

    int16_t  tmp;
    uint16_t power_level = 0;

    tmp = wor->rf_infos.rssi_in_dbm;
    tmp = MIN( -15, tmp );
    tmp = MAX( -142, tmp );
    power_level |= RELAY_NOTIFY_POWER_LEVEL_UPLINK_RSSI( ( uint8_t )( -( tmp + 15 ) ) );

    tmp = wor->rf_infos.snr_in_db;
    tmp = MIN( 11, tmp );
    tmp = MAX( -20, tmp );
    power_level |= RELAY_NOTIFY_POWER_LEVEL_UPLINK_SNR( ( uint8_t )( tmp + 20 ) );

    uint8_t buffer[RELAY_NOTIFY_BYTE_ORDER_POWER_LEVEL_LENGTH];

    buffer[RELAY_NOTIFY_BYTE_CID_REQ]                = RELAY_CID_NOTIFY;
    buffer[RELAY_NOTIFY_BYTE_ORDER_DEV_ADDR_7_0]     = ( uint8_t )( wor->uplink.devaddr );
    buffer[RELAY_NOTIFY_BYTE_ORDER_DEV_ADDR_15_8]    = ( uint8_t )( wor->uplink.devaddr >> 8 );
    buffer[RELAY_NOTIFY_BYTE_ORDER_DEV_ADDR_23_16]   = ( uint8_t )( wor->uplink.devaddr >> 16 );
    buffer[RELAY_NOTIFY_BYTE_ORDER_DEV_ADDR_31_24]   = ( uint8_t )( wor->uplink.devaddr >> 24 );
    buffer[RELAY_NOTIFY_BYTE_ORDER_POWER_LEVEL_7_0]  = ( uint8_t )( power_level );
    buffer[RELAY_NOTIFY_BYTE_ORDER_POWER_LEVEL_15_8] = ( uint8_t )( power_level >> 8 );

    lorawan_api_payload_send( 0, true, buffer, RELAY_NOTIFY_BYTE_ORDER_POWER_LEVEL_LENGTH, UNCONF_DATA_UP,
                              smtc_modem_hal_get_time_in_ms( ) + 20, RELAY_STACK_ID );
}

static int16_t ral_compute_rx_done_correction( uint8_t datarate, uint16_t payload_len )
{
    const uint16_t delay_fft_sf[12 - 5 + 1] = { 23, 45, 91, 181, 362, 724, 1448, 2896 };  //(1<<sf)/sqrt(2)

    uint8_t            sf               = 0;       // Invalid value
    lr1mac_bandwidth_t bw               = BW_RFU;  // Invalid value
    uint32_t           delay_filter_irq = 0;
    uint8_t            clock_shift      = 0;
    uint8_t            ppm              = 0;

    if( smtc_real_get_modulation_type_from_datarate( relay_info.lr1mac->real, datarate ) != LORA )
    {
        return 0;
    }
    smtc_real_lora_dr_to_sf_bw( relay_info.lr1mac->real, datarate, &sf, &bw );

    // CRC is ON and Header explicit and CR45 for SX126x
    payload_len += 2;  // For the CRC

    switch( bw )
    {
    case RAL_LORA_BW_500_KHZ:
        delay_filter_irq = -3 + ( delay_fft_sf[sf - 5] >> 2 );
        clock_shift      = 4;
        break;

    case RAL_LORA_BW_250_KHZ:
        delay_filter_irq = 12 + ( delay_fft_sf[sf - 5] >> 1 );
        clock_shift      = 3;
        if( sf == 12 )
        {
            ppm = 2;
        }
        break;

    case RAL_LORA_BW_125_KHZ:
        delay_filter_irq = 38 + delay_fft_sf[sf - 5];
        clock_shift      = 2;
        if( sf >= 11 )
        {
            ppm = 2;
        }
        break;

    default:
        return 0;
    }

    const uint32_t fft_delay   = ( sf + 1 ) << ( sf + 1 );
    const uint32_t demap_delay = ( 1 << sf );
    const uint8_t  fine_sync   = ( sf <= 6 ) ? 4 : 6;
    const uint32_t dec_delay   = ( ( ( payload_len << 1 ) - sf + fine_sync ) % ( sf - ppm ) ) * 80 + 5;
    const uint32_t phy_delay   = ( ( fft_delay + demap_delay + dec_delay - 1 ) >> clock_shift ) + 1;
    const uint32_t delay       = delay_filter_irq + phy_delay;

    return delay;
}

static bool check_fwd_limitation( const wor_infos_t* wor, uint8_t device_idx )
{
    static bool     is_first_init  = true;
    static uint32_t relay_fwd_time = 0;
    const uint32_t  current_hour   = smtc_modem_hal_get_time_in_ms( ) / 3600000;

    if( is_first_init == true )
    {
        is_first_init  = false;
        relay_fwd_time = current_hour;
    }

    // SMTC_MODEM_HAL_TRACE_PRINTF( "Check FWD Limit %d vs %d\n", relay_fwd_time, current_hour );
    if( current_hour != relay_fwd_time )
    {
        uint16_t diff_hour = 0;
        if( current_hour > relay_fwd_time )
        {
            diff_hour = current_hour - relay_fwd_time;
        }
        else
        {
            // 1193 is 0xFFFFFFFF/3600000
            // There is no +1 as 1193 is 167s from the max of 0xFFFFFFFF
            diff_hour = current_hour + 1193 - relay_fwd_time;
        }

        relay_fwd_time = current_hour;

        for( uint8_t i = 0; i < LIMIT__LAST_ELT; i++ )
        {
            relay_fwd_cnt[i].token_available += relay_fwd_cnt[i].reload_rate * diff_hour;
            if( relay_fwd_cnt[i].token_available > relay_fwd_cnt[i].bucket_size )
            {
                relay_fwd_cnt[i].token_available = relay_fwd_cnt[i].bucket_size;
            }
        }

        for( uint16_t i = 0; i < SIZE_TAB_DEV_ADDR_LIST; i++ )
        {
            relay_fwd_uplink_list_t* device = &device_list_dev_addr[i];

            if( device->in_use == true )
            {
                device->fwd_cfg.token_available += device->fwd_cfg.reload_rate * diff_hour;

                if( device->fwd_cfg.token_available > device->fwd_cfg.bucket_size )
                {
                    device->fwd_cfg.token_available = device->fwd_cfg.bucket_size;
                }
            }
        }
    }

    // SMTC_MODEM_HAL_TRACE_PRINTF( "- Overall      : %d\n", relay_fwd_cnt[0].token_available );
    // SMTC_MODEM_HAL_TRACE_PRINTF( "- GlobalUplink : %d\n", relay_fwd_cnt[1].token_available );
    // SMTC_MODEM_HAL_TRACE_PRINTF( "- Notify       : %d\n", relay_fwd_cnt[2].token_available );
    // SMTC_MODEM_HAL_TRACE_PRINTF( "- JoinReq      : %d\n", relay_fwd_cnt[3].token_available );

    if( ( relay_fwd_cnt[LIMIT_OVERALL].unlimited_fwd == false ) &&
        ( relay_fwd_cnt[LIMIT_OVERALL].token_available == 0 ) )
    {
        return false;
    }

    if( wor->wor_type == WOR_MSG_TYPE_JOIN_REQUEST )
    {
        if( ( relay_fwd_cnt[LIMIT_JOINREQ].unlimited_fwd == false ) &&
            ( relay_fwd_cnt[LIMIT_JOINREQ].token_available == 0 ) )
        {
            return false;
        }
    }
    else if( wor->wor_type == WOR_MSG_TYPE_STANDARD_UPLINK )
    {
        if( ( relay_fwd_cnt[LIMIT_GLOBAL_UPLINK].unlimited_fwd == false ) &&
            ( relay_fwd_cnt[LIMIT_GLOBAL_UPLINK].token_available == 0 ) )
        {
            return false;
        }

        if( device_idx != SIZE_TAB_DEV_ADDR_LIST )  // Known ED with valid MIC on WOR
        {
            if( ( device_list_dev_addr[device_idx].fwd_cfg.unlimited_fwd == false ) &&
                ( device_list_dev_addr[device_idx].fwd_cfg.token_available == 0 ) )
            {
                return false;
            }
        }
        else
        {
            if( ( relay_fwd_cnt[LIMIT_NOTIFY].unlimited_fwd == false ) &&
                ( relay_fwd_cnt[LIMIT_NOTIFY].token_available == 0 ) )
            {
                return false;
            }
        }
    }

    return true;
}

static bool check_forward_filter_joinreq( const uint8_t joineui[8], const uint8_t deveui[8] )
{
    bool    msg_to_fwd = ( device_list_join[0].action == RELAY_FILTER_FWD_TYPE_FORWARD ) ? true : false;
    uint8_t max_match  = 0;

    for( uint16_t i = 1; i < SIZE_TAB_JOIN_REQ_LIST; i++ )
    {
        if( ( device_list_join[i].action != RELAY_FILTER_FWD_TYPE_CLEAR ) && ( device_list_join[i].len >= max_match ) )
        {
            const uint8_t compare_length_joineui = MIN( device_list_join[i].len, 8 );
            bool          is_a_match             = true;

            for( uint16_t j = 0; j < compare_length_joineui; j++ )
            {
                if( device_list_join[i].eui[j] != joineui[7 - j] )
                {
                    is_a_match = false;
                }
            }

            if( is_a_match == true )  // -> JOIN EUI is a match
            {
                if( device_list_join[i].len > 8 )
                {
                    const uint8_t compare_length_deveui = device_list_join[i].len - 8;
                    for( uint16_t j = 0; j < compare_length_deveui; j++ )
                    {
                        // Compare DEV EUI
                        if( device_list_join[i].eui[j + 8] != deveui[7 - j] )
                        {
                            is_a_match = false;
                        }
                    }
                }

                if( is_a_match == true )  // Check of JOIN EUI & DEV EUI is still a match
                {
                    max_match  = device_list_join[i].len;
                    msg_to_fwd = ( device_list_join[i].action == RELAY_FILTER_FWD_TYPE_FILTER ) ? false : true;
                }
            }
        }
    }

    // SMTC_MODEM_HAL_TRACE_PRINTF( "JoinRequest will be %s\n", ( msg_to_fwd ? "forward" : "filter" ) );

    return msg_to_fwd;
}

static void relay_rxr_tx_launch_callback( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    const uint8_t    id = rp->radio_task_id;

    if( rp->radio_params[id].pkt_type == RAL_PKT_TYPE_LORA )
    {
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ralf_setup_lora( rp->radio, &rp->radio_params[id].tx.lora ) == RAL_STATUS_OK );
    }
    else
    {
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ralf_setup_gfsk( rp->radio, &rp->radio_params[id].tx.gfsk ) == RAL_STATUS_OK );
    }

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_TX_DONE ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE(
        ral_set_pkt_payload( &( rp->radio->ral ), rp->payload[id], rp->payload_buffer_size[id] ) == RAL_STATUS_OK );
    // Wait the exact expected time (ie target - tcxo startup delay)
    while( ( int32_t )( rp->tasks[id].start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) > 0 )
    {
        // Do nothing
    }
    // At this time only tcxo startup delay is remaining
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_set_ant_switch( true );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_tx( &( rp->radio->ral ) ) == RAL_STATUS_OK );
    rp_stats_set_tx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
}

static uint8_t get_ul_dr( const wor_infos_t* wor )
{
    return ( ( relay_wor_info.wor_type == WOR_MSG_TYPE_JOIN_REQUEST ) ? relay_wor_info.join_request.dr
                                                                      : relay_wor_info.uplink.dr );
}
static uint32_t get_ul_freq( const wor_infos_t* wor )
{
    return ( ( relay_wor_info.wor_type == WOR_MSG_TYPE_JOIN_REQUEST ) ? relay_wor_info.join_request.freq_hz
                                                                      : relay_wor_info.uplink.freq_hz );
}

static void decrement_fwd_counter( relay_fwd_config_t* limit_counter )
{
    if( ( limit_counter->unlimited_fwd == false ) && ( limit_counter->token_available > 0 ) )
    {
        limit_counter->token_available -= 1;
    }
}
