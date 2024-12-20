/*!
 * \file      smtc_modem_test.c
 *
 * \brief     modem test functions
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

#include "smtc_modem_test_api.h"
#include "smtc_modem_api.h"

#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_modem_utilities.h"
#include "radio_planner.h"
#include "lorawan_api.h"
#include "modem_core.h"
#include "lr1mac_core.h"
#include "smtc_real.h"
#include "smtc_duty_cycle.h"
#include "smtc_modem_hal.h"
#include "smtc_secure_element.h"
#include "smtc_lbt.h"
#include "smtc_lora_cad_bt.h"
#include "ralf.h"
#include "radio_planner.h"
#include "radio_planner_hook_id_defs.h"
#include "modem_tx_protocol_manager.h"
#include "smtc_modem_test.h"
#include "modem_event_utilities.h"
#if defined( SX128X )
#include "sx128x_hal.h"
#elif defined( SX126X )
#include "sx126x_hal.h"
#elif defined( LR11XX )
#include "lr11xx_hal.h"
#elif defined( SX127X )
#include "sx127x_hal.h"
#else
#error "Please select radio board.."
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define TEST_STACK_ID_0 0
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
/*!
 * \brief   LoRaWAN GFSK sync word
 */
static const uint8_t sync_word_gfsk[] = { 0xC1, 0x94, 0xC1 };

/*!
 * \brief   LoRaWAN LR_FHSS sync word
 */
static const uint8_t sync_word_lr_fhss[] = { 0x2C, 0x0F, 0x79, 0x95 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

modem_test_context_t modem_test_context;

static rp_task_t         rp_task;
static rp_radio_params_t rp_radio_params;
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static void modem_test_enqueue_task( uint8_t* payload, uint8_t payload_length );
/*!
 * \brief   Empty callback for test
 * \retval [out]    context*                  - modem_test_context_t
 */

void modem_test_empty_callback( modem_test_context_t* context );

void modem_test_compute_rssi_callback( modem_test_context_t* context );

/*!
 * \brief   Callback for test tx
 * \retval [out]    context*                  - modem_test_context_t
 */
void modem_test_tx_callback( modem_test_context_t* context );

/*!
 * \brief   Callback for test rx
 * \retval [out]    context*                  - modem_test_context_t
 */
void modem_test_rx_callback( modem_test_context_t* context );

/*!
 * \brief   Callback to configure Tx Continues Wave by Radio Planner
 * \retval [in]    rp_void*                   - radio planner context
 */
void test_mode_cw_callback_for_rp( void* rp_void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_modem_return_code_t smtc_modem_test_start( void )
{
    if( modem_get_test_mode_status( ) == true )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TST MODE: ALREADY STARTED\n" );
        return SMTC_MODEM_RC_BUSY;
    }

    if( lorawan_api_isjoined( TEST_STACK_ID_0 ) != NOT_JOINED )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "TST MODE: not available if joined\n" );
        return SMTC_MODEM_RC_FAIL;
    }

    modem_set_test_mode_status( true );
    SMTC_MODEM_HAL_TRACE_INFO( "TST MODE: START\n" );
    memset( &modem_test_context, 0, sizeof( modem_test_context_t ) );
    memset( &rp_task, 0, sizeof( rp_task_t ) );
    memset( &rp_radio_params, 0, sizeof( rp_radio_params_t ) );

    modem_test_context.rp = modem_get_rp( );
    // modem_test_context.lr1_mac_obj = lorawan_api_stack_mac_get( TEST_STACK_ID_0 );
    modem_test_context.hook_id = RP_HOOK_ID_TEST_MODE;

    rp_release_hook( modem_test_context.rp, modem_test_context.hook_id );
    // lorawan_api_init( modem_test_context.rp, TEST_STACK_ID_0, NULL );

    rp_hook_init( modem_test_context.rp, modem_test_context.hook_id,
                  ( void ( * )( void* ) )( modem_test_empty_callback ), &modem_test_context );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_stop( void )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }

    if( smtc_modem_test_nop( true ) != SMTC_MODEM_RC_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    rp_release_hook( modem_test_context.rp, modem_test_context.hook_id );
    modem_set_test_mode_status( false );

    // Re enable failsafe on radio planner
    rp_disable_failsafe( modem_test_context.rp, false );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_tx_hop( void )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    return SMTC_MODEM_RC_FAIL;
}

smtc_modem_return_code_t smtc_modem_test_nop( bool reset_radio )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    modem_test_context.nb_of_repetition = 1;
    rp_task_abort( modem_test_context.rp, modem_test_context.hook_id );
    // force a call of rp_callback to re-arbitrate the radio planner before the next loop
    rp_callback( modem_test_context.rp );
    if( reset_radio )
    {
        smtc_modem_test_radio_reset( );
    }
    // Re enable failsafe on radio planner
    rp_disable_failsafe( modem_test_context.rp, false );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_tx_lora( uint8_t* payload, uint8_t payload_length, uint32_t frequency_hz,
                                                  int8_t tx_power_dbm, ral_lora_sf_t sf, ral_lora_bw_t bw,
                                                  ral_lora_cr_t cr, smtc_modem_test_mode_sync_word_t sync_word,
                                                  bool invert_iq, bool crc_is_on, ral_lora_pkt_len_modes_t header_type,
                                                  uint32_t preamble_size, uint32_t nb_of_tx, uint32_t delay_ms )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    if( ( sf < RAL_LORA_SF5 ) || ( sf > RAL_LORA_SF12 ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid sf %d\n", sf );
        return SMTC_MODEM_RC_INVALID;
    }
    if( bw > RAL_LORA_BW_1600_KHZ )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid bw %d\n", bw );
        return SMTC_MODEM_RC_INVALID;
    }
    if( ( cr < RAL_LORA_CR_4_5 ) || ( cr > RAL_LORA_CR_LI_4_8 ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid cr %d\n", cr );
        return SMTC_MODEM_RC_INVALID;
    }
    if( payload_length == 0 )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid payload_length %d\n", payload_length );
        return SMTC_MODEM_RC_INVALID;
    }
    modem_test_context.tx_frequency     = frequency_hz;
    modem_test_context.sf               = sf;
    modem_test_context.bw               = bw;
    modem_test_context.invert_iq        = invert_iq;
    modem_test_context.nb_of_repetition = nb_of_tx;
    modem_test_context.delay_ms         = delay_ms;
    modem_test_context.modulation_type  = LORA;
    // config lora parameters
    ralf_params_lora_t lora_param;
    memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );
    lora_param.rf_freq_in_hz                   = frequency_hz;
    lora_param.output_pwr_in_dbm               = tx_power_dbm;
    lora_param.sync_word                       = sync_word;
    lora_param.pkt_params.preamble_len_in_symb = preamble_size;
    lora_param.pkt_params.header_type          = header_type;
    lora_param.pkt_params.pld_len_in_bytes     = payload_length;
    lora_param.pkt_params.crc_is_on            = crc_is_on;
    lora_param.pkt_params.invert_iq_is_on      = invert_iq;
    lora_param.mod_params.sf                   = sf;
    lora_param.mod_params.bw                   = bw;
    lora_param.mod_params.cr                   = cr;
    lora_param.mod_params.ldro = ral_compute_lora_ldro( lora_param.mod_params.sf, lora_param.mod_params.bw );

    // config radio parameters
    rp_radio_params.pkt_type = RAL_PKT_TYPE_LORA;
    rp_radio_params.tx.lora  = lora_param;

    SMTC_MODEM_HAL_TRACE_PRINTF( "LoRa Tx - Freq:%u, Power:%d, sf:%u, bw:%u, length:%u, nb_of_repetition:%u\n",
                                 frequency_hz, tx_power_dbm, rp_radio_params.tx.lora.mod_params.sf,
                                 rp_radio_params.tx.lora.mod_params.bw, payload_length,
                                 modem_test_context.nb_of_repetition );

    rp_task.type                  = RP_TASK_TYPE_TX_LORA;
    rp_task.launch_task_callbacks = lr1_stack_mac_tx_lora_launch_callback_for_rp;
    rp_task.duration_time_ms =
        ral_get_lora_time_on_air_in_ms( &( modem_test_context.rp->radio->ral ), &( rp_radio_params.tx.lora.pkt_params ),
                                        &( rp_radio_params.tx.lora.mod_params ) );

    modem_test_enqueue_task( payload, payload_length );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_tx_fsk( uint8_t* payload, uint8_t payload_length, uint32_t frequency_hz,
                                                 int8_t tx_power_dbm, uint32_t nb_of_tx, uint32_t delay_ms )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }

    if( payload_length == 0 )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid payload_length %d\n", payload_length );
        return SMTC_MODEM_RC_INVALID;
    }
    modem_test_context.tx_frequency     = frequency_hz;
    modem_test_context.nb_of_repetition = nb_of_tx;
    modem_test_context.delay_ms         = delay_ms;
    modem_test_context.modulation_type  = FSK;

    ralf_params_gfsk_t gfsk_param;
    memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

    gfsk_param.rf_freq_in_hz     = frequency_hz;
    gfsk_param.output_pwr_in_dbm = tx_power_dbm;
    gfsk_param.sync_word         = ( uint8_t* ) sync_word_gfsk;
    gfsk_param.whitening_seed    = GFSK_WHITENING_SEED;
    gfsk_param.crc_seed          = GFSK_CRC_SEED;
    gfsk_param.crc_polynomial    = GFSK_CRC_POLYNOMIAL;

    gfsk_param.pkt_params.header_type           = RAL_GFSK_PKT_VAR_LEN;
    gfsk_param.pkt_params.pld_len_in_bytes      = payload_length;
    gfsk_param.pkt_params.preamble_len_in_bits  = 40;
    gfsk_param.pkt_params.preamble_detector     = RAL_GFSK_PREAMBLE_DETECTOR_MIN_16BITS;
    gfsk_param.pkt_params.sync_word_len_in_bits = 24;
    gfsk_param.pkt_params.crc_type              = RAL_GFSK_CRC_2_BYTES_INV;
    gfsk_param.pkt_params.dc_free               = RAL_GFSK_DC_FREE_WHITENING;

    gfsk_param.mod_params.br_in_bps    = 50000;
    gfsk_param.mod_params.fdev_in_hz   = 25000;
    gfsk_param.mod_params.bw_dsb_in_hz = 100000;
    gfsk_param.mod_params.pulse_shape  = RAL_GFSK_PULSE_SHAPE_BT_1;

    rp_radio_params.pkt_type = RAL_PKT_TYPE_GFSK;
    rp_radio_params.tx.gfsk  = gfsk_param;

    SMTC_MODEM_HAL_TRACE_PRINTF( "GFSK Tx - Freq:%d, Power:%d, length:%u\n", frequency_hz, tx_power_dbm,
                                 payload_length );

    rp_task.type                  = RP_TASK_TYPE_TX_FSK;
    rp_task.launch_task_callbacks = lr1_stack_mac_tx_gfsk_launch_callback_for_rp;
    rp_task.duration_time_ms =
        ral_get_gfsk_time_on_air_in_ms( &( modem_test_context.rp->radio->ral ), &( rp_radio_params.tx.gfsk.pkt_params ),
                                        &( rp_radio_params.tx.gfsk.mod_params ) );

    SMTC_MODEM_HAL_TRACE_PRINTF( "nb_of_repetition:%u\n", modem_test_context.nb_of_repetition );

    modem_test_enqueue_task( payload, payload_length );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_tx_lrfhss( uint8_t* payload, uint8_t payload_length, uint32_t frequency_hz,
                                                    int8_t tx_power_dbm, lr_fhss_v1_cr_t tx_cr, lr_fhss_v1_bw_t tx_bw,
                                                    lr_fhss_v1_grid_t tx_grid, bool enable_hopping, uint32_t nb_of_tx,
                                                    uint32_t delay_ms )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }

    if( ( tx_cr != LR_FHSS_V1_CR_1_3 ) && ( tx_cr != LR_FHSS_V1_CR_2_3 ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid cr %d\n", tx_cr );
        return SMTC_MODEM_RC_INVALID;
    }
    if( tx_bw > LR_FHSS_V1_BW_1574219_HZ )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid bw %d\n", tx_bw );
        return SMTC_MODEM_RC_INVALID;
    }
    if( tx_grid > LR_FHSS_V1_GRID_3906_HZ )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid grid %d\n", tx_grid );
        return SMTC_MODEM_RC_INVALID;
    }
    if( payload_length == 0 )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid payload_length %d\n", payload_length );
        return SMTC_MODEM_RC_INVALID;
    }

    modem_test_context.tx_frequency     = frequency_hz;
    modem_test_context.nb_of_repetition = nb_of_tx;
    modem_test_context.delay_ms         = delay_ms;
    modem_test_context.modulation_type  = LR_FHSS;
    ralf_params_lr_fhss_t lr_fhss_param;
    memset( &lr_fhss_param, 0, sizeof( ralf_params_lr_fhss_t ) );

    lr_fhss_param.output_pwr_in_dbm = tx_power_dbm;

    lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.modulation_type = LR_FHSS_V1_MODULATION_TYPE_GMSK_488;
    lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.cr              = tx_cr;
    lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.grid            = tx_grid;
    lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.enable_hopping  = enable_hopping;
    lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.bw              = tx_bw;
    lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.header_count    = smtc_real_lr_fhss_get_header_count( tx_cr );
    lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.sync_word       = ( uint8_t* ) sync_word_lr_fhss;

    lr_fhss_param.ral_lr_fhss_params.center_frequency_in_hz = frequency_hz;
    lr_fhss_param.ral_lr_fhss_params.device_offset          = 0;

    unsigned int nb_max_hop_sequence = 0;
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_lr_fhss_get_hop_sequence_count( &( modem_test_context.rp->radio->ral ),
                                                                         &( lr_fhss_param.ral_lr_fhss_params ),
                                                                         &nb_max_hop_sequence ) == RAL_STATUS_OK );
    lr_fhss_param.hop_sequence_id = smtc_modem_hal_get_random_nb_in_range( 0, ( uint32_t ) nb_max_hop_sequence - 1 );

    rp_radio_params.tx.lr_fhss = lr_fhss_param;

    SMTC_MODEM_HAL_TRACE_PRINTF( "LR_FHSS Tx - Freq:%u, Power:%d, grid:%u, bw:%u, cr:%u, length:%u\n", frequency_hz,
                                 tx_power_dbm, lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.grid,
                                 lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.bw,
                                 lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.cr, payload_length );

    ral_lr_fhss_get_time_on_air_in_ms( &( modem_test_context.rp->radio->ral ), &( lr_fhss_param.ral_lr_fhss_params ),
                                       payload_length, &rp_task.duration_time_ms );

    SMTC_MODEM_HAL_TRACE_PRINTF( "  Hop ID = %d\n", lr_fhss_param.hop_sequence_id );

    rp_task.type                  = RP_TASK_TYPE_TX_LR_FHSS;
    rp_task.launch_task_callbacks = lr1_stack_mac_tx_lr_fhss_launch_callback_for_rp;

    SMTC_MODEM_HAL_TRACE_PRINTF( "nb_of_repetition:%u\n", modem_test_context.nb_of_repetition );

    modem_test_enqueue_task( payload, payload_length );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_tx_cw( uint32_t frequency_hz, int8_t tx_power_dbm )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }

    ralf_params_lora_t lora_param;
    modem_test_context.tx_frequency = frequency_hz;
    memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );

    lora_param.rf_freq_in_hz     = frequency_hz;
    lora_param.output_pwr_in_dbm = tx_power_dbm;
    lora_param.mod_params.sf     = RAL_LORA_SF12;

    if( frequency_hz >= 2400000000 )
    {
        lora_param.mod_params.bw = RAL_LORA_BW_800_KHZ;
        lora_param.mod_params.cr = RAL_LORA_CR_LI_4_8;
        lora_param.sync_word     = 0x12;
    }
    else
    {
        lora_param.mod_params.bw = RAL_LORA_BW_125_KHZ;
        lora_param.mod_params.cr = RAL_LORA_CR_4_5;
        lora_param.sync_word     = 0x34;
    }

    rp_radio_params.pkt_type = RAL_PKT_TYPE_LORA;
    rp_radio_params.tx.lora  = lora_param;

    rp_task.hook_id               = modem_test_context.hook_id;
    rp_task.state                 = RP_TASK_STATE_ASAP;
    rp_task.start_time_ms         = smtc_modem_hal_get_time_in_ms( ) + 2;
    rp_task.duration_time_ms      = 2000;
    rp_task.type                  = RP_TASK_TYPE_TX_LORA;
    rp_task.launch_task_callbacks = test_mode_cw_callback_for_rp;

    // First disable failsafe check for radio planner as the task can be longer than failsafe value
    rp_disable_failsafe( modem_test_context.rp, true );

    if( tx_protocol_manager_request( TX_PROTOCOL_TRANSMIT_TEST_MODE, 0, false, modem_test_context.tx_rx_payload, 0, 0,
                                     rp_task.start_time_ms, TEST_STACK_ID_0 ) == ERRORLORAWAN )
    {
        modem_test_empty_callback( NULL );
    }

    SMTC_MODEM_HAL_TRACE_PRINTF( "Tx CW - Freq:%u, Power:%d\n", frequency_hz, lora_param.output_pwr_in_dbm );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_rx_lora( uint32_t frequency_hz, ral_lora_sf_t sf, ral_lora_bw_t bw,
                                                  ral_lora_cr_t cr, smtc_modem_test_mode_sync_word_t sync_word,
                                                  bool invert_iq, bool crc_is_on, ral_lora_pkt_len_modes_t header_type,
                                                  uint32_t preamble_size, uint8_t symb_nb_timeout )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    if( ( sf < RAL_LORA_SF5 ) || ( sf > RAL_LORA_SF12 ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid sf %d\n", sf );
        return SMTC_MODEM_RC_INVALID;
    }
    if( bw > RAL_LORA_BW_1600_KHZ )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid bw %d\n", bw );
        return SMTC_MODEM_RC_INVALID;
    }
    if( ( cr < RAL_LORA_CR_4_5 ) || ( cr > RAL_LORA_CR_LI_4_8 ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid cr %d\n", cr );
        return SMTC_MODEM_RC_INVALID;
    }

    // reset number of received packets
    modem_test_context.total_rx_packets = 0;

    // config  lora parameters
    ralf_params_lora_t lora_param;
    memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );
    lora_param.rf_freq_in_hz                   = frequency_hz;
    lora_param.sync_word                       = sync_word;
    lora_param.pkt_params.preamble_len_in_symb = preamble_size;
    lora_param.pkt_params.header_type          = header_type;
    lora_param.pkt_params.crc_is_on            = crc_is_on;
    lora_param.pkt_params.invert_iq_is_on      = invert_iq;
    lora_param.mod_params.sf                   = sf;
    lora_param.mod_params.bw                   = bw;
    lora_param.mod_params.cr                   = cr;
    lora_param.mod_params.ldro = ral_compute_lora_ldro( lora_param.mod_params.sf, lora_param.mod_params.bw );
    lora_param.symb_nb_timeout = symb_nb_timeout;
    lora_param.pkt_params.pld_len_in_bytes = 255;
    // config radio parameters
    rp_radio_params_t rp_radio_params = { 0 };
    rp_radio_params.rx.timeout_in_ms  = RAL_RX_TIMEOUT_CONTINUOUS_MODE;
    rp_radio_params.pkt_type          = RAL_PKT_TYPE_LORA;
    rp_radio_params.rx.lora           = lora_param;

    // config radio planner task parameters
    rp_task.hook_id               = modem_test_context.hook_id;
    rp_task.state                 = RP_TASK_STATE_ASAP;
    rp_task.type                  = RP_TASK_TYPE_RX_LORA;
    rp_task.launch_task_callbacks = lr1_stack_mac_rx_lora_launch_callback_for_rp;
    rp_task.start_time_ms         = smtc_modem_hal_get_time_in_ms( ) + 20;
    rp_task.duration_time_ms      = 20;  // will be extended by the radio planner
    rp_disable_failsafe( modem_test_context.rp, true );
    rp_release_hook( modem_test_context.rp, modem_test_context.hook_id );
    rp_hook_init( modem_test_context.rp, modem_test_context.hook_id, ( void ( * )( void* ) )( modem_test_rx_callback ),
                  &modem_test_context );

    if( rp_task_enqueue( modem_test_context.rp, &rp_task, modem_test_context.tx_rx_payload, 255, &rp_radio_params ) !=
        RP_HOOK_STATUS_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_rx_fsk_continuous( uint32_t frequency_hz )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }

    // reset number of received packets
    modem_test_context.total_rx_packets = 0;

    rp_radio_params_t rp_radio_params = { 0 };
    rp_radio_params.rx.timeout_in_ms  = RAL_RX_TIMEOUT_CONTINUOUS_MODE;

    // Radio config for FSK
    ralf_params_gfsk_t gfsk_param;
    memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

    gfsk_param.rf_freq_in_hz  = frequency_hz;
    gfsk_param.sync_word      = ( uint8_t* ) sync_word_gfsk;
    gfsk_param.whitening_seed = GFSK_WHITENING_SEED;
    gfsk_param.crc_seed       = GFSK_CRC_SEED;
    gfsk_param.crc_polynomial = GFSK_CRC_POLYNOMIAL;

    gfsk_param.pkt_params.header_type           = RAL_GFSK_PKT_VAR_LEN;
    gfsk_param.pkt_params.pld_len_in_bytes      = 255;
    gfsk_param.pkt_params.preamble_len_in_bits  = 40;
    gfsk_param.pkt_params.preamble_detector     = RAL_GFSK_PREAMBLE_DETECTOR_MIN_16BITS;
    gfsk_param.pkt_params.sync_word_len_in_bits = 24;
    gfsk_param.pkt_params.crc_type              = RAL_GFSK_CRC_2_BYTES_INV;
    gfsk_param.pkt_params.dc_free               = RAL_GFSK_DC_FREE_WHITENING;

    gfsk_param.mod_params.br_in_bps    = 50000;
    gfsk_param.mod_params.fdev_in_hz   = 25000;
    gfsk_param.mod_params.bw_dsb_in_hz = 100000;
    gfsk_param.mod_params.pulse_shape  = RAL_GFSK_PULSE_SHAPE_BT_1;

    rp_radio_params.pkt_type = RAL_PKT_TYPE_GFSK;
    rp_radio_params.rx.gfsk  = gfsk_param;

    SMTC_MODEM_HAL_TRACE_PRINTF( "GFSK Rx - Freq:%d\n", frequency_hz );

    // Radio planner task config
    rp_task.hook_id               = modem_test_context.hook_id;
    rp_task.state                 = RP_TASK_STATE_ASAP;
    rp_task.type                  = RP_TASK_TYPE_RX_FSK;
    rp_task.launch_task_callbacks = lr1_stack_mac_rx_gfsk_launch_callback_for_rp;
    rp_task.start_time_ms         = smtc_modem_hal_get_time_in_ms( ) + 20;
    rp_task.duration_time_ms      = 20;  // toa;
    rp_release_hook( modem_test_context.rp, modem_test_context.hook_id );
    rp_disable_failsafe( modem_test_context.rp, true );
    rp_hook_init( modem_test_context.rp, modem_test_context.hook_id, ( void ( * )( void* ) )( modem_test_rx_callback ),
                  &modem_test_context );

    if( rp_task_enqueue( modem_test_context.rp, &rp_task, modem_test_context.tx_rx_payload, 255, &rp_radio_params ) !=
        RP_HOOK_STATUS_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_get_nb_rx_packets( uint32_t* nb_rx_packets )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    *nb_rx_packets = modem_test_context.total_rx_packets;
    return SMTC_MODEM_RC_OK;
}
smtc_modem_return_code_t smtc_modem_test_get_last_rx_packets( int16_t* rssi, int16_t* snr, uint8_t* rx_payload,
                                                              uint8_t* rx_payload_length )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    *rssi              = modem_test_context.last_rx_payload_rssi;
    *snr               = modem_test_context.last_rx_payload_snr;
    *rx_payload_length = modem_test_context.last_rx_payload_length;
    memcpy( rx_payload, modem_test_context.tx_rx_payload, modem_test_context.last_rx_payload_length );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_rssi_lbt( uint32_t frequency_hz, uint32_t bw_hz, uint16_t time_ms )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    if( ( bw_hz < 125000 ) || ( bw_hz > 467000 ) )  // 467000 Maximum supported GFSK BW
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid bw %d\n", bw_hz );
        return SMTC_MODEM_RC_INVALID;
    }

    modem_test_context.rssi_ready = false;

    smtc_lbt_init( &modem_test_context.lbt_obj, modem_test_context.rp, RP_HOOK_ID_LBT,
                   ( void ( * )( void* ) ) modem_test_compute_rssi_callback, &modem_test_context,
                   ( void ( * )( void* ) ) modem_test_compute_rssi_callback, &modem_test_context,
                   ( void ( * )( void* ) ) modem_test_compute_rssi_callback, &modem_test_context );
    smtc_lbt_set_parameters( &modem_test_context.lbt_obj, time_ms, 50, bw_hz );
    smtc_lbt_set_state( &modem_test_context.lbt_obj, true );
    smtc_lbt_listen_channel( &modem_test_context.lbt_obj, frequency_hz, 0, smtc_modem_hal_get_time_in_ms( ), 0 );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_get_rssi( int8_t* rssi )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }

    smtc_modem_return_code_t return_code;

    if( modem_test_context.rssi_ready == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "RSSI TEST RESULT NOT READY\n" )
        return_code = SMTC_MODEM_RC_BUSY;
    }
    else
    {
        *rssi       = ( ( int8_t ) ( modem_test_context.rssi + 64 ) );
        return_code = SMTC_MODEM_RC_OK;
    }
    return return_code;
}

void modem_test_set_rssi( int16_t rssi )
{
    modem_test_context.rssi = rssi;
}

smtc_modem_return_code_t smtc_modem_test_radio_reset( void )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    if( ral_reset( &( modem_test_context.rp->radio->ral ) ) != RAL_STATUS_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    if( ral_init( &( modem_test_context.rp->radio->ral ) ) != RAL_STATUS_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    if( ral_set_sleep( &( modem_test_context.rp->radio->ral ), true ) != RAL_STATUS_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    smtc_modem_hal_set_ant_switch( false );
    smtc_modem_hal_stop_radio_tcxo( );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_direct_radio_write( uint8_t* command, uint16_t command_length, uint8_t* data,
                                                             uint16_t data_length )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }
#if defined( SX128X )
    if( sx128x_hal_read( modem_test_context.rp->radio->ral.context, command, command_length, data, data_length ) !=
        SX128X_HAL_STATUS_OK )
#elif defined( SX126X )
    if( sx126x_hal_write( modem_test_context.rp->radio->ral.context, command, command_length, data, data_length ) !=
        SX126X_HAL_STATUS_OK )
#elif defined( LR11XX_TRANSCEIVER )
    if( lr11xx_hal_write( modem_test_context.rp->radio->ral.context, command, command_length, data, data_length ) !=
        LR11XX_HAL_STATUS_OK )
#elif defined( SX127X )
    // if( sx127x_hal_write( ( sx127x_t* ) ( modem_test_context.rp->radio->ral.context ), command, data,
    //                       data_length ) != SX127X_HAL_STATUS_OK )
#else
    return SMTC_MODEM_RC_FAIL;
#endif
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_direct_radio_read( uint8_t* command, uint16_t command_length, uint8_t* data,
                                                            uint16_t data_length )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }
#if defined( SX128X )
    if( sx128x_hal_read( modem_test_context.rp->radio->ral.context, command, command_length, data, data_length ) !=
        SX128X_HAL_STATUS_OK )
#elif defined( SX126X )
    if( sx126x_hal_read( modem_test_context.rp->radio->ral.context, command, command_length, data, data_length ) !=
        SX126X_HAL_STATUS_OK )
#elif defined( LR11XX_TRANSCEIVER )
    if( lr11xx_hal_read( modem_test_context.rp->radio->ral.context, command, command_length, data, data_length ) !=
        LR11XX_HAL_STATUS_OK )
#elif defined( SX127X )
    // if( sx127x_hal_read( ( sx127x_t* ) ( modem_test_context.rp->radio->ral.context ), command, data,
    //                      data_length ) != SX127X_HAL_STATUS_OK )
#else
    return SMTC_MODEM_RC_FAIL;
#endif
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
}
status_lorawan_t test_mode_cb_tpm( uint8_t* payload, uint8_t payload_length, bool abort )
{
    if( abort == true )
    {
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TEST_MODE, SMTC_MODEM_EVENT_TEST_MODE_ENDED, 0xFF );
        return ERRORLORAWAN;
    }
    // Enqueue task in radio planner
    if( rp_task_enqueue( modem_test_context.rp, &rp_task, modem_test_context.tx_rx_payload, payload_length,
                         &rp_radio_params ) != RP_HOOK_STATUS_OK )
    {
        return ERRORLORAWAN;
    }
    return OKLORAWAN;
}

modem_test_context_t* smtc_modem_test_get_context( void )
{
    return ( &modem_test_context );
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void modem_test_empty_callback( modem_test_context_t* context )
{
    increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TEST_MODE, SMTC_MODEM_EVENT_TEST_MODE_ENDED, 0xFF );
    SMTC_MODEM_HAL_TRACE_PRINTF( "TEST mode Empty callback\n" );
}

void modem_test_compute_rssi_callback( modem_test_context_t* context )
{
    float rssi_mean           = ( ( float ) context->lbt_obj.rssi_accu ) / context->lbt_obj.rssi_nb_of_meas;
    context->rssi             = ( int16_t ) ( rssi_mean );
    context->total_rx_packets = context->lbt_obj.rssi_nb_of_meas;
    context->rssi_ready       = true;

    SMTC_MODEM_HAL_TRACE_PRINTF( "rssi_accu: %d, cnt:%d --> rssi: %d dBm\n", context->lbt_obj.rssi_accu,
                                 context->lbt_obj.rssi_nb_of_meas, context->rssi );
}

void modem_test_tx_callback( modem_test_context_t* context )
{
    // SMTC_MODEM_HAL_TRACE_PRINTF( " modem_test_tx_callback %u\n", context->nb_of_repetition );
    smtc_modem_hal_reload_wdog( );
    context->nb_of_repetition--;
    if( context->nb_of_repetition == 0 )
    {
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TEST_MODE, SMTC_MODEM_EVENT_TEST_MODE_TX_COMPLETED, 0xFF );
        SMTC_MODEM_HAL_TRACE_PRINTF( " modem_test_tx_callback finished\n" );
        return;
    }
    else
    {
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TEST_MODE, SMTC_MODEM_EVENT_TEST_MODE_TX_DONE, 0xFF );
    }

    rp_task.start_time_ms = smtc_modem_hal_get_time_in_ms( ) + context->delay_ms;

    rp_radio_params = context->rp->radio_params[context->hook_id];

    if( rp_task.type == RP_TASK_TYPE_TX_LORA )
    {
        rp_task.launch_task_callbacks = lr1_stack_mac_tx_lora_launch_callback_for_rp;
        if( context->random_payload == true )
        {
            for( uint8_t i = 0; i < rp_radio_params.tx.lora.pkt_params.pld_len_in_bytes; i++ )
            {
                context->tx_rx_payload[i] = smtc_modem_hal_get_random_nb_in_range( 0, 0xFF );
            }
        }

        rp_task.duration_time_ms =
            ral_get_lora_time_on_air_in_ms( &( context->rp->radio->ral ), &( rp_radio_params.tx.lora.pkt_params ),
                                            &( rp_radio_params.tx.lora.mod_params ) );

        if( tx_protocol_manager_request( TX_PROTOCOL_TRANSMIT_TEST_MODE, 0, false, context->tx_rx_payload,
                                         rp_radio_params.tx.lora.pkt_params.pld_len_in_bytes, 0, rp_task.start_time_ms,
                                         TEST_STACK_ID_0 ) == ERRORLORAWAN )
        {
            modem_test_empty_callback( NULL );
        }
    }
    else if( rp_task.type == RP_TASK_TYPE_TX_FSK )
    {
        rp_task.launch_task_callbacks = lr1_stack_mac_tx_gfsk_launch_callback_for_rp;

        if( context->random_payload == true )
        {
            for( uint8_t i = 0; i < rp_radio_params.tx.gfsk.pkt_params.pld_len_in_bytes; i++ )
            {
                context->tx_rx_payload[i] = smtc_modem_hal_get_random_nb_in_range( 0, 0xFF );
            }
        }

        rp_task.duration_time_ms =
            ral_get_gfsk_time_on_air_in_ms( &( context->rp->radio->ral ), &( rp_radio_params.tx.gfsk.pkt_params ),
                                            &( rp_radio_params.tx.gfsk.mod_params ) );
        if( tx_protocol_manager_request( TX_PROTOCOL_TRANSMIT_TEST_MODE, 0, false, context->tx_rx_payload,
                                         rp_radio_params.tx.gfsk.pkt_params.pld_len_in_bytes, 0, rp_task.start_time_ms,
                                         TEST_STACK_ID_0 ) == ERRORLORAWAN )
        {
            modem_test_empty_callback( NULL );
        }
    }
    else if( rp_task.type == RP_TASK_TYPE_TX_LR_FHSS )
    {
        uint8_t payload_length        = context->rp->payload_buffer_size[context->hook_id];
        rp_task.launch_task_callbacks = lr1_stack_mac_tx_lr_fhss_launch_callback_for_rp;

        ralf_params_lr_fhss_t* lr_fhss_param = &rp_radio_params.tx.lr_fhss;

        // Hopping sequence must be different for each uplink
        unsigned int nb_max_hop_sequence = 0;
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_lr_fhss_get_hop_sequence_count( &( context->rp->radio->ral ),
                                                                             &( lr_fhss_param->ral_lr_fhss_params ),
                                                                             &nb_max_hop_sequence ) == RAL_STATUS_OK );
        lr_fhss_param->hop_sequence_id =
            smtc_modem_hal_get_random_nb_in_range( 0, ( uint32_t ) nb_max_hop_sequence - 1 );

        if( context->random_payload == true )
        {
            for( uint8_t i = 0; i < payload_length; i++ )
            {
                context->tx_rx_payload[i] = smtc_modem_hal_get_random_nb_in_range( 0, 0xFF );
            }
        }
        rp_task.duration_time_ms = ral_lr_fhss_get_time_on_air_in_ms(
            &( context->rp->radio->ral ), &( rp_radio_params.tx.lr_fhss.ral_lr_fhss_params ), payload_length,
            &rp_task.duration_time_ms );

        if( tx_protocol_manager_request( TX_PROTOCOL_TRANSMIT_TEST_MODE, 0, false, context->tx_rx_payload,
                                         payload_length, 0, rp_task.start_time_ms, TEST_STACK_ID_0 ) == ERRORLORAWAN )
        {
            modem_test_empty_callback( NULL );
        }
    }
}

void modem_test_rx_callback( modem_test_context_t* context )
{
    smtc_modem_hal_reload_wdog( );
    rp_status_t rp_status    = context->rp->status[context->hook_id];
    rp_task.start_time_ms    = smtc_modem_hal_get_time_in_ms( ) + 20;
    rp_task.duration_time_ms = 20;  // will be extended by the radio planner

    rp_radio_params_t radio_params = context->rp->radio_params[context->hook_id];

    if( rp_status == RP_STATUS_RX_PACKET )
    {
        uint16_t payload_length;
        if( radio_params.pkt_type == RAL_PKT_TYPE_LORA )
        {
            rp_task.type                  = RP_TASK_TYPE_RX_LORA;
            rp_task.launch_task_callbacks = lr1_stack_mac_rx_lora_launch_callback_for_rp;
            payload_length                = radio_params.rx.lora.pkt_params.pld_len_in_bytes;
            context->last_rx_payload_snr = context->rp->radio_params[context->hook_id].rx.lora_pkt_status.snr_pkt_in_db;
            context->last_rx_payload_rssi =
                context->rp->radio_params[context->hook_id].rx.lora_pkt_status.rssi_pkt_in_dbm;
        }
        else
        {
            rp_task.type                  = RP_TASK_TYPE_RX_FSK;
            rp_task.launch_task_callbacks = lr1_stack_mac_rx_gfsk_launch_callback_for_rp;
            payload_length                = radio_params.rx.gfsk.pkt_params.pld_len_in_bytes;
            context->last_rx_payload_snr  = 0;  // not available for GFSK
            context->last_rx_payload_rssi =
                context->rp->radio_params[context->hook_id].rx.gfsk_pkt_status.rssi_avg_in_dbm;
        }

        if( rp_task_enqueue( context->rp, &rp_task, context->tx_rx_payload, payload_length, &radio_params ) !=
            RP_HOOK_STATUS_OK )
        {
            SMTC_MODEM_HAL_PANIC( );
        }

        context->total_rx_packets++;
        context->last_rx_payload_length = context->rp->rx_payload_size[context->hook_id];
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TEST_MODE, SMTC_MODEM_EVENT_TEST_MODE_RX_DONE, 0xFF );
    }
    else
    {  // RP_STATUS_RX_TIMEOUT or RP_STATUS_RX_ABORTED

        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TEST_MODE, SMTC_MODEM_EVENT_TEST_MODE_RX_ABORTED, 0xFF );
    }
}

void test_mode_cw_callback_for_rp( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_init( &( rp->radio->ral ) ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ralf_setup_lora( rp->radio, &rp->radio_params[id].tx.lora ) == RAL_STATUS_OK );
    smtc_modem_hal_set_ant_switch( true );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_tx_cw( &( rp->radio->ral ) ) == RAL_STATUS_OK );
}

static void modem_test_enqueue_task( uint8_t* payload, uint8_t payload_length )
{
    rp_task.hook_id = modem_test_context.hook_id;
    rp_task.state   = RP_TASK_STATE_ASAP;
    rp_release_hook( modem_test_context.rp, modem_test_context.hook_id );
    rp_hook_init( modem_test_context.rp, modem_test_context.hook_id, ( void ( * )( void* ) )( modem_test_tx_callback ),
                  &modem_test_context );

    if( payload == NULL )
    {
        // user payload is NULL=> generate a random before at next step
        modem_test_context.random_payload = true;
        for( uint8_t i = 0; i < payload_length; i++ )
        {
            modem_test_context.tx_rx_payload[i] = smtc_modem_hal_get_random_nb_in_range( 0, 0xFF );
        }
    }
    else
    {
        modem_test_context.random_payload = false;
        // save tx payload in context
        memcpy( modem_test_context.tx_rx_payload, payload, payload_length );
    }
    rp_task.start_time_ms = smtc_modem_hal_get_time_in_ms( ) + modem_test_context.delay_ms;

    if( tx_protocol_manager_request( TX_PROTOCOL_TRANSMIT_TEST_MODE, 0, false, modem_test_context.tx_rx_payload,
                                     payload_length, 0, rp_task.start_time_ms, TEST_STACK_ID_0 ) == ERRORLORAWAN )
    {
        modem_test_empty_callback( NULL );
    }
}

/* --- EOF ------------------------------------------------------------------ */
