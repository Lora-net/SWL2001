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
#include "modem_context.h"
#include "lr1mac_core.h"
#include "smtc_real.h"
#include "smtc_modem_hal.h"

#include "ralf.h"
#include "radio_planner.h"

#if defined( SX128X )
#include "sx128x_hal.h"
#elif defined( SX126X )
#include "sx126x_hal.h"
#elif defined( LR11XX )
#include "lr11xx_hal.h"
#else
#error "Please select radio board.."
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
/* clang-format off */
/*!
 * \brief   Conversion table for Spreading Factor
 */
static const uint8_t modem_test_sf_convert[SMTC_MODEM_TEST_LORA_SF_COUNT] = {
    [SMTC_MODEM_TEST_LORA_SF5]  = RAL_LORA_SF5,
    [SMTC_MODEM_TEST_LORA_SF6]  = RAL_LORA_SF6,
    [SMTC_MODEM_TEST_LORA_SF7]  = RAL_LORA_SF7,
    [SMTC_MODEM_TEST_LORA_SF8]  = RAL_LORA_SF8,
    [SMTC_MODEM_TEST_LORA_SF9]  = RAL_LORA_SF9,
    [SMTC_MODEM_TEST_LORA_SF10] = RAL_LORA_SF10,
    [SMTC_MODEM_TEST_LORA_SF11] = RAL_LORA_SF11,
    [SMTC_MODEM_TEST_LORA_SF12] = RAL_LORA_SF12,
};

/*!
 * \brief   Conversion table for BandWidth
 */
static const uint8_t modem_test_bw_convert[SMTC_MODEM_TEST_BW_COUNT] = {
    [SMTC_MODEM_TEST_BW_125_KHZ]  = RAL_LORA_BW_125_KHZ,
    [SMTC_MODEM_TEST_BW_250_KHZ]  = RAL_LORA_BW_250_KHZ,
    [SMTC_MODEM_TEST_BW_500_KHZ]  = RAL_LORA_BW_500_KHZ,
    [SMTC_MODEM_TEST_BW_200_KHZ]  = RAL_LORA_BW_200_KHZ,
    [SMTC_MODEM_TEST_BW_400_KHZ]  = RAL_LORA_BW_400_KHZ,
    [SMTC_MODEM_TEST_BW_800_KHZ]  = RAL_LORA_BW_800_KHZ,
    [SMTC_MODEM_TEST_BW_1600_KHZ] = RAL_LORA_BW_1600_KHZ,
};

/*!
 * \brief   Conversion table for Coding Rate
 */
static const uint8_t modem_test_cr_convert[SMTC_MODEM_TEST_CR_COUNT] = {
    [SMTC_MODEM_TEST_CR_4_5] = RAL_LORA_CR_4_5,
    [SMTC_MODEM_TEST_CR_4_6] = RAL_LORA_CR_4_6,
    [SMTC_MODEM_TEST_CR_4_7] = RAL_LORA_CR_4_7,
    [SMTC_MODEM_TEST_CR_4_8] = RAL_LORA_CR_4_8,
    [SMTC_MODEM_TEST_CR_LI_4_5] = RAL_LORA_CR_LI_4_5,
    [SMTC_MODEM_TEST_CR_LI_4_6] = RAL_LORA_CR_LI_4_6,
    [SMTC_MODEM_TEST_CR_LI_4_8] = RAL_LORA_CR_LI_4_8,
};
/*!
 * \brief   Helper table for gfsk Bandwidth
 */
static const uint32_t modem_test_bw_helper[SMTC_MODEM_TEST_BW_COUNT] = {
    [SMTC_MODEM_TEST_BW_125_KHZ]  = 125000,
    [SMTC_MODEM_TEST_BW_250_KHZ]  = 250000,
    [SMTC_MODEM_TEST_BW_500_KHZ]  = 100000,
    [SMTC_MODEM_TEST_BW_200_KHZ]  = 200000,
    [SMTC_MODEM_TEST_BW_400_KHZ]  = 400000,
    [SMTC_MODEM_TEST_BW_800_KHZ]  = 800000,
    [SMTC_MODEM_TEST_BW_1600_KHZ] = 1600000,
};
/* clang-format on */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * \typedef modem_test_context_t
 * \brief   Test context
 */
typedef struct modem_test_context
{
    radio_planner_t* rp;                  //!< Radio planner instance
    lr1_stack_mac_t* lr1_mac_obj;         //!< Lorawan lr1mac instance
    uint8_t          hook_id;             //!< Lorawan lr1mac hook id used for test
    uint8_t          tx_rx_payload[255];  //!< Transmit/Received buffer
    int16_t          rssi;                //!< Placeholder for mean rssi
    bool             rssi_ready;          //!< True when rssi mean test is finished
    uint32_t         total_rx_packets;    //!< Number of received packet
    bool             random_payload;      //!< True in case of random payload
} modem_test_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

#if defined( LR1110_MODEM_E )
modem_test_context_t modem_test_context;
#else
static modem_test_context_t modem_test_context;
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
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

    if( get_join_state( ) != MODEM_NOT_JOINED )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "TST MODE: not available if joined\n" );
        return SMTC_MODEM_RC_FAIL;
    }

    modem_set_test_mode_status( true );
    SMTC_MODEM_HAL_TRACE_INFO( "TST MODE: START\n" );
    memset( &modem_test_context, 0, sizeof( modem_test_context_t ) );

    modem_test_context.rp          = modem_context_get_modem_rp( );
    modem_test_context.lr1_mac_obj = lorawan_api_stack_mac_get( );
    modem_test_context.hook_id     = lorawan_api_rp_hook_id_get( );

    rp_release_hook( modem_test_context.rp, modem_test_context.hook_id );
    lorawan_api_init( modem_test_context.rp );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_stop( void )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }

    if( smtc_modem_test_nop( ) != SMTC_MODEM_RC_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }
    rp_release_hook( modem_test_context.rp, modem_test_context.hook_id );
    lorawan_api_init( modem_test_context.rp );
    modem_set_test_mode_status( false );

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

smtc_modem_return_code_t smtc_modem_test_nop( void )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    rp_task_abort( modem_test_context.rp, modem_test_context.hook_id );
    smtc_modem_test_radio_reset( );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_tx( uint8_t* payload, uint8_t payload_length, uint32_t frequency_hz,
                                             int8_t tx_power_dbm, smtc_modem_test_sf_t sf, smtc_modem_test_bw_t bw,
                                             smtc_modem_test_cr_t cr, uint32_t preamble_size, bool continuous_tx )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    if( smtc_real_is_frequency_valid( modem_test_context.lr1_mac_obj, frequency_hz ) != OKLORAWAN )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid Frequency %d\n", frequency_hz );
        return SMTC_MODEM_RC_INVALID;
    }
    if( sf >= SMTC_MODEM_TEST_LORA_SF_COUNT )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid sf %d\n", sf );
        return SMTC_MODEM_RC_INVALID;
    }
    if( bw >= SMTC_MODEM_TEST_BW_COUNT )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid bw %d\n", bw );
        return SMTC_MODEM_RC_INVALID;
    }
    if( cr >= SMTC_MODEM_TEST_CR_COUNT )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid cr %d\n", cr );
        return SMTC_MODEM_RC_INVALID;
    }

    rp_task_t         rp_task         = { 0 };
    rp_radio_params_t rp_radio_params = { 0 };

    rp_task.hook_id = modem_test_context.hook_id;
    rp_task.state   = RP_TASK_STATE_ASAP;

    if( sf == SMTC_MODEM_TEST_FSK )  // FSK
    {
        ralf_params_gfsk_t gfsk_param;
        memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

        gfsk_param.rf_freq_in_hz                    = frequency_hz;
        gfsk_param.output_pwr_in_dbm                = tx_power_dbm;
        gfsk_param.sync_word                        = smtc_real_get_gfsk_sync_word( modem_test_context.lr1_mac_obj );
        gfsk_param.dc_free_is_on                    = true;
        gfsk_param.whitening_seed                   = GFSK_WHITENING_SEED;
        gfsk_param.crc_seed                         = GFSK_CRC_SEED;
        gfsk_param.crc_polynomial                   = GFSK_CRC_POLYNOMIAL;
        gfsk_param.pkt_params.header_type           = RAL_GFSK_PKT_VAR_LEN;
        gfsk_param.pkt_params.pld_len_in_bytes      = payload_length;
        gfsk_param.pkt_params.preamble_len_in_bits  = 40;
        gfsk_param.pkt_params.sync_word_len_in_bits = 24;
        gfsk_param.pkt_params.dc_free               = RAL_GFSK_DC_FREE_WHITENING;
        gfsk_param.pkt_params.crc_type              = RAL_GFSK_CRC_2_BYTES_INV;
        gfsk_param.mod_params.fdev_in_hz            = 25000;
        gfsk_param.mod_params.br_in_bps             = 50000;
        gfsk_param.mod_params.bw_dsb_in_hz          = 100000;
        gfsk_param.mod_params.pulse_shape           = RAL_GFSK_PULSE_SHAPE_BT_1;

        rp_radio_params.pkt_type = RAL_PKT_TYPE_GFSK;
        rp_radio_params.tx.gfsk  = gfsk_param;

        SMTC_MODEM_HAL_TRACE_PRINTF( "GFSK Tx - Freq:%d, Power:%d, length:%u\n", frequency_hz, tx_power_dbm,
                                     payload_length );

        rp_task.type                  = RP_TASK_TYPE_TX_FSK;
        rp_task.launch_task_callbacks = lr1_stack_mac_tx_gfsk_launch_callback_for_rp;
        rp_task.duration_time_ms      = ral_get_gfsk_time_on_air_in_ms( &( modem_test_context.rp->radio->ral ),
                                                                   &( rp_radio_params.tx.gfsk.pkt_params ),
                                                                   &( rp_radio_params.tx.gfsk.mod_params ) );
    }
    else  // LoRa
    {
        ralf_params_lora_t lora_param;
        memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );

        lora_param.rf_freq_in_hz     = frequency_hz;
        lora_param.output_pwr_in_dbm = tx_power_dbm;
        lora_param.sync_word         = smtc_real_get_sync_word( modem_test_context.lr1_mac_obj );

        lora_param.pkt_params.preamble_len_in_symb = preamble_size;
        lora_param.pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT;
        lora_param.pkt_params.pld_len_in_bytes     = payload_length;
        lora_param.pkt_params.crc_is_on            = true;
        lora_param.pkt_params.invert_iq_is_on      = false;

        lora_param.mod_params.sf   = ( ral_lora_sf_t ) modem_test_sf_convert[sf];
        lora_param.mod_params.bw   = ( ral_lora_bw_t ) modem_test_bw_convert[bw];
        lora_param.mod_params.cr   = ( ral_lora_cr_t ) modem_test_cr_convert[cr];
        lora_param.mod_params.ldro = ral_compute_lora_ldro( lora_param.mod_params.sf, lora_param.mod_params.bw );

        rp_radio_params.pkt_type = RAL_PKT_TYPE_LORA;
        rp_radio_params.tx.lora  = lora_param;

        SMTC_MODEM_HAL_TRACE_PRINTF( "LoRa Tx - Freq:%u, Power:%d, sf:%u, bw:%u, cr:%u, length:%u\n", frequency_hz,
                                     tx_power_dbm, rp_radio_params.tx.lora.mod_params.sf, lora_param.mod_params.bw,
                                     lora_param.mod_params.cr, payload_length );

        rp_task.type                  = RP_TASK_TYPE_TX_LORA;
        rp_task.launch_task_callbacks = lr1_stack_mac_tx_lora_launch_callback_for_rp;
        rp_task.duration_time_ms      = ral_get_lora_time_on_air_in_ms( &( modem_test_context.rp->radio->ral ),
                                                                   &( rp_radio_params.tx.lora.pkt_params ),
                                                                   &( rp_radio_params.tx.lora.mod_params ) );
    }

    if( smtc_modem_test_nop( ) != SMTC_MODEM_RC_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    if( continuous_tx == false )  // single tx
    {
        rp_release_hook( modem_test_context.rp, modem_test_context.hook_id );
        rp_hook_init( modem_test_context.rp, modem_test_context.hook_id,
                      ( void ( * )( void* ) )( modem_test_empty_callback ), &modem_test_context );
    }
    else
    {
        rp_release_hook( modem_test_context.rp, modem_test_context.hook_id );
        rp_hook_init( modem_test_context.rp, modem_test_context.hook_id,
                      ( void ( * )( void* ) )( modem_test_tx_callback ), &modem_test_context );
    }

    if( payload == NULL )
    {
        // user payload is NULL=> generate a random before at next step
        modem_test_context.random_payload = true;
        for( uint8_t i = 0; i < payload_length; i++ )
        {
            modem_test_context.tx_rx_payload[i] = ( smtc_modem_hal_get_random_nb( ) % 256 );
        }
    }
    else
    {
        modem_test_context.random_payload = false;
        // save tx payload in context
        memcpy( modem_test_context.tx_rx_payload, payload, payload_length );
    }

    rp_task.start_time_ms = smtc_modem_hal_get_time_in_ms( ) + 20;

    // Enqueue task in radio planner
    rp_task_enqueue( modem_test_context.rp, &rp_task, modem_test_context.tx_rx_payload, payload_length,
                     &rp_radio_params );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_tx_cw( uint32_t frequency_hz, int8_t tx_power_dbm )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    if( smtc_real_is_frequency_valid( modem_test_context.lr1_mac_obj, frequency_hz ) != OKLORAWAN )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid Frequency %d\n", frequency_hz );
        return SMTC_MODEM_RC_INVALID;
    }

    ralf_params_lora_t lora_param;
    memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );

    lora_param.rf_freq_in_hz     = frequency_hz;
    lora_param.output_pwr_in_dbm = tx_power_dbm;
    lora_param.mod_params.sf     = RAL_LORA_SF12;
#if defined( SX128X )
    lora_param.mod_params.bw = RAL_LORA_BW_800_KHZ;
#elif defined( SX126X )
    lora_param.mod_params.bw = RAL_LORA_BW_125_KHZ;
#elif defined( LR11XX )
    lora_param.mod_params.bw = RAL_LORA_BW_125_KHZ;
#endif
    lora_param.mod_params.cr = smtc_real_get_coding_rate( modem_test_context.lr1_mac_obj );
    lora_param.sync_word     = smtc_real_get_sync_word( modem_test_context.lr1_mac_obj );

    rp_radio_params_t radio_params = { 0 };
    radio_params.pkt_type          = RAL_PKT_TYPE_LORA;
    radio_params.tx.lora           = lora_param;

    rp_task_t rp_task             = { 0 };
    rp_task.hook_id               = modem_test_context.hook_id;
    rp_task.state                 = RP_TASK_STATE_ASAP;
    rp_task.start_time_ms         = smtc_modem_hal_get_time_in_ms( ) + 2;
    rp_task.duration_time_ms      = 2000;  // toa;
    rp_task.type                  = RP_TASK_TYPE_RX_LORA;
    rp_task.launch_task_callbacks = test_mode_cw_callback_for_rp;

    if( rp_task_enqueue( modem_test_context.rp, &rp_task, NULL, 0, &radio_params ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Radio planner hook %d is busy \n", rp_task.hook_id );
    }

    SMTC_MODEM_HAL_TRACE_PRINTF( "Tx CW - Freq:%u, Power:%d\n", frequency_hz, lora_param.output_pwr_in_dbm );
    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_rx_continuous( uint32_t frequency_hz, smtc_modem_test_sf_t sf,
                                                        smtc_modem_test_bw_t bw, smtc_modem_test_cr_t cr )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    if( smtc_real_is_frequency_valid( modem_test_context.lr1_mac_obj, frequency_hz ) != OKLORAWAN )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid Frequency %u\n", frequency_hz );
        return SMTC_MODEM_RC_INVALID;
    }
    if( sf >= SMTC_MODEM_TEST_LORA_SF_COUNT )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid sf %d\n", sf );
        return SMTC_MODEM_RC_INVALID;
    }
    if( bw >= SMTC_MODEM_TEST_BW_COUNT )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid bw %d\n", bw );
        return SMTC_MODEM_RC_INVALID;
    }
    if( cr >= SMTC_MODEM_TEST_CR_COUNT )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid cr %d\n", cr );
        return SMTC_MODEM_RC_INVALID;
    }

    // reset number of received packets
    modem_test_context.total_rx_packets = 0;

    rp_radio_params_t rp_radio_params = { 0 };
    rp_radio_params.rx.timeout_in_ms  = RAL_RX_TIMEOUT_CONTINUOUS_MODE;

    rp_task_t rp_task = { 0 };
    rp_task.hook_id   = modem_test_context.hook_id;
    rp_task.state     = RP_TASK_STATE_ASAP;

    if( sf == 0 )  // FSK
    {
        // Radio config for FSK
        ralf_params_gfsk_t gfsk_param;
        memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

        gfsk_param.rf_freq_in_hz                    = frequency_hz;
        gfsk_param.sync_word                        = smtc_real_get_gfsk_sync_word( modem_test_context.lr1_mac_obj );
        gfsk_param.dc_free_is_on                    = true;
        gfsk_param.whitening_seed                   = GFSK_WHITENING_SEED;
        gfsk_param.crc_seed                         = GFSK_CRC_SEED;
        gfsk_param.crc_polynomial                   = GFSK_CRC_POLYNOMIAL;
        gfsk_param.pkt_params.header_type           = RAL_GFSK_PKT_VAR_LEN;
        gfsk_param.pkt_params.pld_len_in_bytes      = 255;
        gfsk_param.pkt_params.preamble_len_in_bits  = 40;
        gfsk_param.pkt_params.sync_word_len_in_bits = 24;
        gfsk_param.pkt_params.dc_free               = RAL_GFSK_DC_FREE_WHITENING;
        gfsk_param.pkt_params.crc_type              = RAL_GFSK_CRC_2_BYTES_INV;
        gfsk_param.mod_params.fdev_in_hz            = 25000;
        gfsk_param.mod_params.br_in_bps             = 50000;
        gfsk_param.mod_params.bw_dsb_in_hz          = 100000;
        gfsk_param.mod_params.pulse_shape           = RAL_GFSK_PULSE_SHAPE_BT_1;

        rp_radio_params.pkt_type = RAL_PKT_TYPE_GFSK;
        rp_radio_params.rx.gfsk  = gfsk_param;

        SMTC_MODEM_HAL_TRACE_PRINTF( "GFSK Rx - Freq:%d\n", frequency_hz );

        // Radio planner task config
        rp_task.type                  = RP_TASK_TYPE_RX_FSK;
        rp_task.launch_task_callbacks = lr1_stack_mac_rx_gfsk_launch_callback_for_rp;
    }
    else  // LoRa
    {
        // Radio config for LoRa
        ralf_params_lora_t lora_param;
        memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );

        lora_param.rf_freq_in_hz   = frequency_hz;
        lora_param.sync_word       = smtc_real_get_sync_word( modem_test_context.lr1_mac_obj );
        lora_param.symb_nb_timeout = 0;

        lora_param.pkt_params.preamble_len_in_symb =
            smtc_real_get_preamble_len( modem_test_context.lr1_mac_obj, modem_test_sf_convert[sf] );
        lora_param.pkt_params.header_type      = RAL_LORA_PKT_EXPLICIT;
        lora_param.pkt_params.pld_len_in_bytes = 255;
        lora_param.pkt_params.crc_is_on        = false;
        lora_param.pkt_params.invert_iq_is_on  = true;

        lora_param.mod_params.sf   = ( ral_lora_sf_t ) modem_test_sf_convert[sf];
        lora_param.mod_params.bw   = ( ral_lora_bw_t ) modem_test_bw_convert[bw];
        lora_param.mod_params.cr   = ( ral_lora_cr_t ) modem_test_cr_convert[cr];
        lora_param.mod_params.ldro = ral_compute_lora_ldro( lora_param.mod_params.sf, lora_param.mod_params.bw );

        rp_radio_params.pkt_type = RAL_PKT_TYPE_LORA;
        rp_radio_params.rx.lora  = lora_param;

        SMTC_MODEM_HAL_TRACE_PRINTF( "LoRa Rx - Freq:%u, sf:%u, bw:%u, cr:%u\n", frequency_hz,
                                     rp_radio_params.rx.lora.mod_params.sf, bw, cr );

        // Radio planner task config
        rp_task.type                  = RP_TASK_TYPE_RX_LORA;
        rp_task.launch_task_callbacks = lr1_stack_mac_rx_lora_launch_callback_for_rp;
    }

    if( smtc_modem_test_nop( ) != SMTC_MODEM_RC_OK )
    {
        return SMTC_MODEM_RC_FAIL;
    }

    rp_release_hook( modem_test_context.rp, modem_test_context.hook_id );
    rp_hook_init( modem_test_context.rp, modem_test_context.hook_id, ( void ( * )( void* ) )( modem_test_rx_callback ),
                  &modem_test_context );

    rp_task.start_time_ms    = smtc_modem_hal_get_time_in_ms( ) + 20;
    rp_task.duration_time_ms = 2000;  // toa;

    rp_task_enqueue( modem_test_context.rp, &rp_task, modem_test_context.tx_rx_payload, 255, &rp_radio_params );

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

smtc_modem_return_code_t smtc_modem_test_rssi( uint32_t frequency_hz, smtc_modem_test_bw_t bw, uint16_t time_ms )
{
    if( modem_get_test_mode_status( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TEST FUNCTION CANNOT BE CALLED: NOT IN TEST MODE\n" );
        return SMTC_MODEM_RC_INVALID;
    }
    if( smtc_real_is_frequency_valid( modem_test_context.lr1_mac_obj, frequency_hz ) != OKLORAWAN )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( " Invalid Frequency %d\n", frequency_hz );
        return SMTC_MODEM_RC_INVALID;
    }
    if( bw >= SMTC_MODEM_TEST_BW_COUNT )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid bw %d\n", bw );
        return SMTC_MODEM_RC_INVALID;
    }

    uint32_t bw_tmp = 0;
    if( modem_test_bw_helper[bw] > 467000 )
    {
        bw_tmp = 467000;  // Maximum supported GFSK BW
    }
    else
    {
        bw_tmp = modem_test_bw_helper[bw];
    }

    modem_test_context.rssi_ready = false;

    smtc_lbt_init( modem_test_context.lr1_mac_obj->lbt_obj, modem_test_context.lr1_mac_obj->rp, RP_HOOK_ID_LBT,
                   ( void ( * )( void* ) ) modem_test_compute_rssi_callback, &modem_test_context,
                   ( void ( * )( void* ) ) modem_test_compute_rssi_callback, &modem_test_context,
                   ( void ( * )( void* ) ) modem_test_compute_rssi_callback, &modem_test_context );
    smtc_lbt_set_parameters( modem_test_context.lr1_mac_obj->lbt_obj, time_ms, 50, bw_tmp );
    smtc_lbt_set_state( modem_test_context.lr1_mac_obj->lbt_obj, true );
    smtc_lbt_listen_channel( modem_test_context.lr1_mac_obj->lbt_obj, frequency_hz, 0, smtc_modem_hal_get_time_in_ms( ),
                             0 );

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
        *rssi       = ( ( int8_t )( modem_test_context.rssi + 64 ) );
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

    smtc_modem_hal_stop_radio_tcxo( );

    return SMTC_MODEM_RC_OK;
}

smtc_modem_return_code_t smtc_modem_test_duty_cycle_app_activate( bool status )
{
    // First check if modem is in test mode
    if( modem_get_test_mode_status( ) == true )
    {
        return SMTC_MODEM_RC_BUSY;
    }

    if( lorawan_api_duty_cycle_enable_set( ( status == false ) ? SMTC_DTC_FULL_DISABLED : SMTC_DTC_ENABLED ) ==
        OKLORAWAN )
    {
        // When status = 0, Duty cycle disabled by host
        modem_set_duty_cycle_disabled_by_host( ( status == false ) ? true : false );

        return SMTC_MODEM_RC_OK;
    }
    return SMTC_MODEM_RC_INVALID;
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
#elif defined( LR1110_MODEM_E )
    return SMTC_MODEM_RC_FAIL;
#else
#error "Please select radio board.."
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
#elif defined( LR1110_MODEM_E )
    return SMTC_MODEM_RC_FAIL;
#else
#error "Please select radio board.."
#endif
    {
        return SMTC_MODEM_RC_FAIL;
    }
    return SMTC_MODEM_RC_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void modem_test_empty_callback( modem_test_context_t* context )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "TEST mode Empty callback\n" );
}

void modem_test_compute_rssi_callback( modem_test_context_t* context )
{
    float rssi_mean = ( ( float ) context->lr1_mac_obj->lbt_obj->rssi_accu ) /
                      modem_test_context.lr1_mac_obj->lbt_obj->rssi_nb_of_meas;
    context->rssi             = ( int16_t )( rssi_mean );
    context->total_rx_packets = context->lr1_mac_obj->lbt_obj->rssi_nb_of_meas;
    context->rssi_ready       = true;

    SMTC_MODEM_HAL_TRACE_PRINTF( "rssi_accu: %d, cnt:%d --> rssi: %d dBm\n", context->lr1_mac_obj->lbt_obj->rssi_accu,
                                 context->lr1_mac_obj->lbt_obj->rssi_nb_of_meas, context->rssi );
}

void modem_test_tx_callback( modem_test_context_t* context )
{
    smtc_modem_hal_reload_wdog( );
    rp_status_t rp_status = context->rp->status[context->hook_id];
    if( rp_status == RP_STATUS_TASK_ABORTED )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " modem_test_tx_callback ABORTED\n" );
        return;
    }
    rp_task_t rp_task = { 0 };

    rp_task.hook_id       = context->hook_id;
    rp_task.state         = RP_TASK_STATE_ASAP;
    rp_task.start_time_ms = smtc_modem_hal_get_time_in_ms( ) + 20;

    rp_radio_params_t radio_params = context->rp->radio_params[context->hook_id];

    if( radio_params.pkt_type == RAL_PKT_TYPE_LORA )
    {
        rp_task.type                  = RP_TASK_TYPE_TX_LORA;
        rp_task.launch_task_callbacks = lr1_stack_mac_tx_lora_launch_callback_for_rp;
        if( context->random_payload == true )
        {
            for( uint8_t i = 0; i < radio_params.tx.lora.pkt_params.pld_len_in_bytes; i++ )
            {
                context->tx_rx_payload[i] = ( smtc_modem_hal_get_random_nb( ) % 256 );
            }
        }

        rp_task.duration_time_ms = ral_get_lora_time_on_air_in_ms(
            &( context->rp->radio->ral ), &( radio_params.tx.lora.pkt_params ), &( radio_params.tx.lora.mod_params ) );

        rp_task_enqueue( context->rp, &rp_task, context->tx_rx_payload,
                         radio_params.tx.lora.pkt_params.pld_len_in_bytes, &radio_params );
    }
    else
    {
        rp_task.type                  = RP_TASK_TYPE_TX_FSK;
        rp_task.launch_task_callbacks = lr1_stack_mac_tx_gfsk_launch_callback_for_rp;

        if( context->random_payload == true )
        {
            for( uint8_t i = 0; i < radio_params.tx.gfsk.pkt_params.pld_len_in_bytes; i++ )
            {
                context->tx_rx_payload[i] = ( smtc_modem_hal_get_random_nb( ) % 256 );
            }
        }

        rp_task.duration_time_ms = ral_get_gfsk_time_on_air_in_ms(
            &( context->rp->radio->ral ), &( radio_params.tx.gfsk.pkt_params ), &( radio_params.tx.gfsk.mod_params ) );

        rp_task_enqueue( context->rp, &rp_task, context->tx_rx_payload,
                         radio_params.tx.gfsk.pkt_params.pld_len_in_bytes, &radio_params );
    }
}

void modem_test_rx_callback( modem_test_context_t* context )
{
    smtc_modem_hal_reload_wdog( );
    rp_status_t rp_status = context->rp->status[context->hook_id];

    if( rp_status == RP_STATUS_RX_PACKET )
    {
        context->total_rx_packets++;
#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
        int16_t  snr              = context->rp->radio_params[context->hook_id].rx.lora_pkt_status.snr_pkt_in_db;
        int16_t  rssi             = context->rp->radio_params[context->hook_id].rx.lora_pkt_status.rssi_pkt_in_dbm;
        uint32_t irq_timestamp_ms = context->rp->irq_timestamp_ms[context->hook_id];
        SMTC_MODEM_HAL_TRACE_PRINTF( "t: %d, rp_status %u, snr: %d, rssi: %d\n", irq_timestamp_ms, rp_status, snr,
                                     rssi );
        SMTC_MODEM_HAL_TRACE_ARRAY( "rx_payload", context->tx_rx_payload, context->rp->payload_size[context->hook_id] );
#endif
    }
    else if( rp_status == RP_STATUS_TASK_ABORTED )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " modem_test_rx_callback ABORTED\n" );
        return;
    }

    rp_task_t rp_task = { 0 };

    rp_task.hook_id          = context->hook_id;
    rp_task.state            = RP_TASK_STATE_ASAP;
    rp_task.start_time_ms    = smtc_modem_hal_get_time_in_ms( ) + 20;
    rp_task.duration_time_ms = 2000;  // toa;

    rp_radio_params_t radio_params = context->rp->radio_params[context->hook_id];

    if( radio_params.pkt_type == RAL_PKT_TYPE_LORA )
    {
        rp_task.type                  = RP_TASK_TYPE_RX_LORA;
        rp_task.launch_task_callbacks = lr1_stack_mac_rx_lora_launch_callback_for_rp;
        rp_task_enqueue( context->rp, &rp_task, context->tx_rx_payload,
                         radio_params.rx.lora.pkt_params.pld_len_in_bytes, &radio_params );
    }
    else
    {
        rp_task.type                  = RP_TASK_TYPE_RX_FSK;
        rp_task.launch_task_callbacks = lr1_stack_mac_rx_gfsk_launch_callback_for_rp;
        rp_task_enqueue( context->rp, &rp_task, context->tx_rx_payload,
                         radio_params.rx.gfsk.pkt_params.pld_len_in_bytes, &radio_params );
    }
}

void test_mode_cw_callback_for_rp( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;
    smtc_modem_hal_assert( ral_init( &( rp->radio->ral ) ) == RAL_STATUS_OK );
    smtc_modem_hal_assert( ralf_setup_lora( rp->radio, &rp->radio_params[id].tx.lora ) == RAL_STATUS_OK );
    smtc_modem_hal_assert( ral_set_tx_cw( &( rp->radio->ral ) ) == RAL_STATUS_OK );
}

/* --- EOF ------------------------------------------------------------------ */
