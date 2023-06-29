/*!
 * \file      smtc_d2d.c
 *
 * \brief     Device to device class b implementation , manage the device transmission in class b multicast
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

#include <stdlib.h>
#include <stdio.h>
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_beacon_sniff.h"
#include "smtc_ping_slot.h"
#include "radio_planner.h"
#include "stddef.h"
#include "smtc_modem_hal.h"
#include "smtc_secure_element.h"
#include "smtc_modem_crypto.h"
#include "lr1mac_core.h"
#include "lr1mac_utilities.h"
#include "smtc_real.h"
#include "smtc_d2d.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define CURRENT_PING_SLOT \
    class_b_d2d_obj->ping_slot_obj->rx_session_param[class_b_d2d_obj->ping_slot_obj->rx_session_index]
#define MULTICAST_OBJ class_b_d2d_obj->ping_slot_obj->rx_session_param[class_b_d2d_obj->multi_cast_group_id]
#define LR1MAC class_b_d2d_obj->ping_slot_obj->lr1_mac
#define RP class_b_d2d_obj->ping_slot_obj->rp

#define MULTICAST_SYMB_DURATION_US smtc_real_get_symbol_duration_us( LR1MAC->real, MULTICAST_OBJ->rx_data_rate )
#define MULTICAST_SYMB_DURATION_MS \
    ( ( ( MULTICAST_SYMB_DURATION_US / 1000UL ) == 0 ) ? 1 : ( MULTICAST_SYMB_DURATION_US / 1000UL ) )
#define MAX_TX_PREAMBLE_DURATION_MS 1000UL
#define CAD_TO_TX_SWITCH_DURATION_US 6000UL

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static void class_b_d2d_rp_request( smtc_class_b_d2d_t* class_b_d2d_obj );

static void class_b_d2d_rp_callback( smtc_class_b_d2d_t* class_b_d2d_obj );

// call by the radioplanner when the radio task is finished.
static void class_b_d2d_launch_callback_for_rp( void* rp_void );

// call by the radioplanner when the radio task is finished.
// call by ping slot obj as soon as ping slot obj enqueue a task inside the radioplanner => give us an opportunitie
// of tx
static void class_b_d2d_call_by_ping_slot( void* class_b_d2d_obj_void );

// return the fcnt_down for the d2d tx transaction
static uint32_t class_b_d2d_get_fcnt_down( smtc_class_b_d2d_t* class_b_d2d_obj );

// Start the Tx
static void class_b_d2d_cad_to_tx( smtc_class_b_d2d_t* class_b_d2d_obj );

// Get the CAD DetPeak value
static uint8_t class_b_d2d_get_cad_det_peak_4_symb( uint8_t sf, lr1mac_bandwidth_t bw );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS  -------------------------------------------
 */

void smtc_class_b_d2d_init( smtc_class_b_d2d_t* class_b_d2d_obj, smtc_ping_slot_t* ping_slot_obj,
                            uint8_t classb_d2d_id_rp, void ( *tx_event_callback )( void* tx_event_context ),
                            void*   tx_event_context )
{
    memset( class_b_d2d_obj, 0, sizeof( smtc_class_b_d2d_t ) );
    class_b_d2d_obj->ping_slot_obj    = ping_slot_obj;
    class_b_d2d_obj->classb_d2d_id_rp = classb_d2d_id_rp;
    class_b_d2d_obj->tx_on_going      = false;
    rp_release_hook( class_b_d2d_obj->ping_slot_obj->rp,
                     classb_d2d_id_rp );  // no need to check return code because in case of
                                          // error panic inside the function
    rp_hook_init( class_b_d2d_obj->ping_slot_obj->rp, classb_d2d_id_rp,
                  ( void ( * )( void* ) )( class_b_d2d_rp_callback ),
                  class_b_d2d_obj );  // no need to check return code because in case of error panic inside the function
    ping_slot_obj->d2d_callback = class_b_d2d_call_by_ping_slot;  // have a set function in ping slot to be cleaner
    ping_slot_obj->d2d_context  = class_b_d2d_obj;

    class_b_d2d_obj->tx_event_callback = tx_event_callback;
    class_b_d2d_obj->tx_event_context  = tx_event_context;

    class_b_d2d_obj->ping_slot_obj->d2d_check_fcnt_down_callback = smtc_class_b_d2d_fcnt_down;
}

smtc_class_b_d2d_status_t smtc_class_b_d2d_request_tx( smtc_class_b_d2d_t* class_b_d2d_obj,
                                                       rx_session_type_t multi_cast_group_id, uint8_t fport,
                                                       uint8_t priority, const uint8_t* payload, uint8_t payload_size,
                                                       uint8_t nb_rep, uint8_t nb_trans_max_retry,
                                                       uint8_t* ping_slots_mask, uint8_t ping_slots_mask_size )
{
    if( ( multi_cast_group_id < RX_SESSION_MULTICAST_G0 ) || ( multi_cast_group_id >= RX_SESSION_COUNT ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid multi_cast_group_id\n" );
        return SMTC_CLASS_B_D2D_ERROR;
    }

    if( class_b_d2d_obj->ping_slot_obj->rx_session_param[multi_cast_group_id]->enabled == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "CLASS B ping slot not running\n" );
        return SMTC_CLASS_B_D2D_ERROR;
    }

    status_lorawan_t status = smtc_real_is_payload_size_valid(
        LR1MAC->real, class_b_d2d_obj->ping_slot_obj->rx_session_param[multi_cast_group_id]->rx_data_rate, payload_size,
        UP_LINK, LR1MAC->tx_fopts_current_length );
    if( status == ERRORLORAWAN )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "PAYLOAD SIZE TOO HIGH\n" );
        return SMTC_CLASS_B_D2D_ERROR;
    }

    if( ping_slots_mask_size > 16 )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "MASK SIZE TOO HIGH\n" );
        return SMTC_CLASS_B_D2D_ERROR;
    }

    // Check if all bits are not cleared from 0 to NbPingSlot
    if( SMTC_ARE_CLR_BYTE8( ping_slots_mask, MAX( ( 1 << ( 7 - MULTICAST_OBJ->ping_slot_periodicity ) / 8 ), 1 ) ) ==
        true )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "ping_slots_mask set to 0\n" );
        return SMTC_CLASS_B_D2D_ERROR;
    }
    memcpy( class_b_d2d_obj->ping_slots_mask, ping_slots_mask, ping_slots_mask_size );

    // Decrement duty cycle before check the available DTC
    smtc_duty_cycle_update( LR1MAC->dtc_obj );
    if( smtc_duty_cycle_is_channel_free( LR1MAC->dtc_obj, MULTICAST_OBJ->rx_frequency ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Duty Cycle is full\n" );
        return SMTC_CLASS_B_D2D_ERROR;
    }

    // store the payload and start to encrypt but have to be updated on each transmission due to the fact that the
    // fcnt will be updated
    class_b_d2d_obj->multi_cast_group_id = ( rx_session_type_t ) multi_cast_group_id;  // add a check
    uint32_t multicast_dev_addr          = MULTICAST_OBJ->dev_addr;
    class_b_d2d_obj->tx_payload_size     = payload_size;
    class_b_d2d_obj->tx_payload[0]       = ( ( UNCONF_DATA_DOWN & 0x7 ) << 5 ) + ( LORAWANR1 & 0x3 );
    class_b_d2d_obj->tx_payload[1]       = ( uint8_t )( ( multicast_dev_addr & 0x000000FF ) );
    class_b_d2d_obj->tx_payload[2]       = ( uint8_t )( ( multicast_dev_addr & 0x0000FF00 ) >> 8 );
    class_b_d2d_obj->tx_payload[3]       = ( uint8_t )( ( multicast_dev_addr & 0x00FF0000 ) >> 16 );
    class_b_d2d_obj->tx_payload[4]       = ( uint8_t )( ( multicast_dev_addr & 0xFF000000 ) >> 24 );
    class_b_d2d_obj->tx_payload[5]       = 0;
    class_b_d2d_obj->tx_payload[6]       = 0;  // let the counter to zero and fill it when transmission is setup
    class_b_d2d_obj->tx_payload[7]       = 0;
    class_b_d2d_obj->tx_payload[8]       = fport;
    memcpy( &class_b_d2d_obj->tx_payload[9], payload, payload_size );
    class_b_d2d_obj->tx_payload_size    = payload_size + D2D_HEADER_LORAWAN_SIZE + D2D_MIC_SIZE;
    class_b_d2d_obj->nb_trans_cnt       = ( nb_rep > 15 ) ? 16 : nb_rep + 1;
    class_b_d2d_obj->nb_trans_max_retry = nb_trans_max_retry;
    class_b_d2d_obj->nb_trans_trial_cnt = 0;
    class_b_d2d_obj->tx_priority        = priority;

    return SMTC_CLASS_B_D2D_OK;
}

uint8_t smtc_class_b_d2d_next_max_payload_length_get( smtc_class_b_d2d_t* class_b_d2d_obj,
                                                      rx_session_type_t   multi_cast_group_id )
{
    if( MULTICAST_OBJ->enabled == false )
    {
        return 0;
    }
    return ( smtc_real_get_max_payload_size( LR1MAC->real, MULTICAST_OBJ->rx_data_rate, UP_LINK ) - 8 );
}

status_lorawan_t smtc_class_b_d2d_fcnt_down( void* ping_slot_obj_void, uint32_t* fcnt_dwn_stack_tmp, uint32_t mic_in )
{
    smtc_ping_slot_t* ping_slot_obj = ( smtc_ping_slot_t* ) ping_slot_obj_void;
    uint32_t          seconds_since_epoch;
    uint32_t          fractional_second;
    uint32_t          fcnt_tmp;
    status_lorawan_t  status = lr1mac_core_convert_rtc_to_gps_epoch_time(
         ping_slot_obj->lr1_mac, smtc_modem_hal_get_time_in_ms( ), &seconds_since_epoch, &fractional_second );

    // number of virtually beacon since 5 jan 1980
    uint32_t number_of_beacon_period_since_gps_epoch = seconds_since_epoch >> 7;

    // number of ping slot per beacon period for this multicast group
    uint8_t number_ping_per_beacon =
        1 << ( 7 - ping_slot_obj->rx_session_param[ping_slot_obj->rx_session_index]->ping_slot_periodicity );

    // number of ping slot already consummed in this current beacon
    uint8_t number_ping_since_begin_of_current_beacon =
        number_ping_per_beacon -
        ping_slot_obj->rx_session_param[ping_slot_obj->rx_session_index]->ping_slot_parameters.ping_number;

    // fcnt down for d2d transaction
    fcnt_tmp = ( uint32_t )( ( number_of_beacon_period_since_gps_epoch * number_ping_per_beacon ) +
                             number_ping_since_begin_of_current_beacon );
    SMTC_MODEM_HAL_TRACE_WARNING( " fcnt_tmp = %d\n ", fcnt_tmp );
    if( smtc_modem_crypto_verify_mic( &ping_slot_obj->rx_payload[0], ping_slot_obj->rx_payload_size,
                                      ping_slot_obj->rx_session_param[ping_slot_obj->rx_session_index]->nwk_skey,
                                      ping_slot_obj->rx_session_param[ping_slot_obj->rx_session_index]->dev_addr, 1,
                                      fcnt_tmp, mic_in ) == SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        status              = OKLORAWAN;
        *fcnt_dwn_stack_tmp = fcnt_tmp;
        SMTC_MODEM_HAL_TRACE_WARNING( " mic 1 ok \n" );
    }
    else
    {
        status = ERRORLORAWAN;
    }
    return status;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS  -------------------------------------------
 */
// this function is call by ping slot object when this one enqueue a task inside the radioplanner to prepare the
// next ping slot opportunitie
static void class_b_d2d_call_by_ping_slot( void* class_b_d2d_obj_void )
{
    smtc_class_b_d2d_t* class_b_d2d_obj = ( smtc_class_b_d2d_t* ) class_b_d2d_obj_void;
    if( class_b_d2d_obj->nb_trans_cnt == 0 )
    {
        return;  // no more transmission increment event tx done is performed on the radio interrupt to be call only
                 // one time
    }
    if( class_b_d2d_obj->nb_trans_trial_cnt >= class_b_d2d_obj->nb_trans_max_retry )
    {
        return;  // no more transmission increment event tx done is performed on the radio interrupt to be call only
                 // one time
    }
    if( class_b_d2d_obj->tx_on_going == true )  // transmission already launch
    {
    }
    if( class_b_d2d_obj->ping_slot_obj->rx_session_index != class_b_d2d_obj->multi_cast_group_id )
    {
        return;  // next ping slot isn't on the same multicast_group
    }

    uint8_t bit_slot =
        ( 1 << ( 7 - MULTICAST_OBJ->ping_slot_periodicity ) ) - MULTICAST_OBJ->ping_slot_parameters.ping_number;
    if( SMTC_GET_BIT8( class_b_d2d_obj->ping_slots_mask, bit_slot ) == false )
    {
        return;  // Ping slot disabled by the user for the Tx
    }

    // Decrement duty cycle before check the available DTC
    smtc_duty_cycle_update( LR1MAC->dtc_obj );
    if( smtc_duty_cycle_is_channel_free( LR1MAC->dtc_obj, MULTICAST_OBJ->rx_frequency ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Duty Cycle is full\n" );
        return;
    }
    // by reaching this part of code meaning have to set a tx on this next ping slot opportunity
    SMTC_D2D_HAL_TRACE_PRINTF( "launch a tx phase in d2d\n" );
    class_b_d2d_rp_request( class_b_d2d_obj );
}

static void class_b_d2d_rp_request( smtc_class_b_d2d_t* class_b_d2d_obj )
{
    // fcnt down for d2d transaction
    uint32_t fcnt_down = class_b_d2d_get_fcnt_down( class_b_d2d_obj );

    // add fcnt in the payload
    class_b_d2d_obj->tx_payload[6] = ( uint8_t )( ( fcnt_down & 0x000000FF ) );
    class_b_d2d_obj->tx_payload[7] = ( uint8_t )( ( fcnt_down & 0x0000FF00 ) >> 8 );
    // encrypt payload
    memcpy( class_b_d2d_obj->tx_payload_encrypt, class_b_d2d_obj->tx_payload, D2D_HEADER_LORAWAN_SIZE );
    if( smtc_modem_crypto_payload_encrypt( &class_b_d2d_obj->tx_payload[D2D_HEADER_LORAWAN_SIZE],
                                           class_b_d2d_obj->tx_payload_size - D2D_HEADER_LORAWAN_SIZE - D2D_MIC_SIZE,
                                           MULTICAST_OBJ->app_skey, MULTICAST_OBJ->dev_addr, 1, fcnt_down,
                                           &class_b_d2d_obj->tx_payload_encrypt[D2D_HEADER_LORAWAN_SIZE] ) !=
        SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        smtc_modem_hal_lr1mac_panic( "Crypto error during payload encryption\n" );
    }

    // add mic

    if( smtc_modem_crypto_compute_and_add_mic( &class_b_d2d_obj->tx_payload_encrypt[0],
                                               class_b_d2d_obj->tx_payload_size - D2D_MIC_SIZE, MULTICAST_OBJ->nwk_skey,
                                               MULTICAST_OBJ->dev_addr, 1, fcnt_down ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        smtc_modem_hal_lr1mac_panic( "Crypto error during mic computation\n" );
    }

    // add this step the app payload is encapsuled and encrypted as a lorawan packet

    // compute preamble length in symbols , double the preamble length for each higher priority
    uint8_t nb_symb_max = MAX( MAX_TX_PREAMBLE_DURATION_MS / MULTICAST_SYMB_DURATION_MS, 255 );
    uint8_t preamble_length_symb =
        MIN( MAX( ( CURRENT_PING_SLOT->rx_window_symb << 1 ) * ( 1 << class_b_d2d_obj->tx_priority ),
                  8 * ( 1 << class_b_d2d_obj->tx_priority ) ),
             nb_symb_max );

    uint32_t preamble_duration_us = preamble_length_symb * MULTICAST_SYMB_DURATION_US;

    // if( tx_modulation_type == LORA ) have to manage lora  or gfsk
    // prepare the task inside the rp will be probably this this mandatory to compute toa  but in reality the task
    // will be first a cad
    uint8_t            tx_sf;
    lr1mac_bandwidth_t tx_bw;
    uint32_t           toa;
    rp_radio_params_t  radio_params = { 0 };
    smtc_real_lora_dr_to_sf_bw( LR1MAC->real, MULTICAST_OBJ->rx_data_rate, &tx_sf, &tx_bw );
    ralf_params_lora_t lora_param;
    memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );
    lora_param.rf_freq_in_hz     = MULTICAST_OBJ->rx_frequency;
    lora_param.sync_word         = smtc_real_get_sync_word( LR1MAC->real );
    lora_param.output_pwr_in_dbm = smtc_real_clamp_output_power_eirp_vs_freq_and_dr(
        LR1MAC->real, LR1MAC->tx_power, MULTICAST_OBJ->rx_frequency, MULTICAST_OBJ->rx_data_rate );
    lora_param.mod_params.sf   = ( ral_lora_sf_t ) tx_sf;
    lora_param.mod_params.bw   = ( ral_lora_bw_t ) tx_bw;
    lora_param.mod_params.cr   = smtc_real_get_coding_rate( LR1MAC->real );
    lora_param.mod_params.ldro = ral_compute_lora_ldro( lora_param.mod_params.sf, lora_param.mod_params.bw );

    lora_param.pkt_params.preamble_len_in_symb = preamble_length_symb;
    lora_param.pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT;
    lora_param.pkt_params.pld_len_in_bytes     = class_b_d2d_obj->tx_payload_size;
    lora_param.pkt_params.crc_is_on            = false;
    lora_param.pkt_params.invert_iq_is_on      = true;
    radio_params.pkt_type                      = RAL_PKT_TYPE_LORA;
    radio_params.tx.lora                       = lora_param;
    toa = ral_get_lora_time_on_air_in_ms( ( &LR1MAC->rp->radio->ral ), ( &lora_param.pkt_params ),
                                          ( &lora_param.mod_params ) );

    // Enqueue this tx inside , start time is given by the ping slot object itself and
    // corrected using preambule length
    rp_task_t rp_task = { 0 };
    rp_task.type = RP_TASK_TYPE_CAD_TO_TX;  // in the radioplanner task is a special case in rp, radio config in case of
                                            // negative cad can be launched without enqueued a new rp task
    rp_task.hook_id = class_b_d2d_obj->classb_d2d_id_rp;
    rp_task.state   = RP_TASK_STATE_SCHEDULE;
    // get the rp param set by the ping slot object itself
    uint32_t ping_slot_start_time =
        class_b_d2d_obj->ping_slot_obj->rp->tasks[class_b_d2d_obj->ping_slot_obj->ping_slot_id4rp].start_time_ms;
    uint32_t ping_slot_rx_duration_ms =
        class_b_d2d_obj->ping_slot_obj->rp->tasks[class_b_d2d_obj->ping_slot_obj->ping_slot_id4rp].duration_time_ms;

    // configure the d2d start time and duration even in case of cad add the cad duration to the tx duration to book
    // the right time inside the rp

    uint32_t cad_duration_us =
        ( 1 << RAL_LORA_CAD_04_SYMB ) * MULTICAST_SYMB_DURATION_US + CAD_TO_TX_SWITCH_DURATION_US;
    // configure the d2d start time and duration even in case of cad add the cad duration to
    // the tx duration to book the right time inside the rp
    rp_task.start_time_ms = ping_slot_start_time - ( ( cad_duration_us + preamble_duration_us ) / 1000 ) +
                            MAX( ( 6 * MULTICAST_SYMB_DURATION_US ) / 1000, ( ping_slot_rx_duration_ms >> 1 ) );
    rp_task.duration_time_ms      = toa;
    rp_task.launch_task_callbacks = class_b_d2d_launch_callback_for_rp;

    if( rp_task_enqueue( class_b_d2d_obj->ping_slot_obj->rp, &rp_task, NULL, 0, &radio_params ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "d2d task not enqueued\n" );
        return;
    }

    class_b_d2d_obj->tx_on_going = true;
}

static void class_b_d2d_launch_callback_for_rp( void* rp_void )
{
    radio_planner_t*    rp = ( radio_planner_t* ) rp_void;
    smtc_class_b_d2d_t* class_b_d2d_obj =
        ( smtc_class_b_d2d_t* ) rp->hooks[rp->radio_task_id];  // verify if it is true !!!!

    uint8_t            sf;
    lr1mac_bandwidth_t bw;
    modulation_type_t  modulation_type =
        smtc_real_get_modulation_type_from_datarate( LR1MAC->real, MULTICAST_OBJ->rx_data_rate );

    if( modulation_type == LORA )
    {
        smtc_real_lora_dr_to_sf_bw( LR1MAC->real, MULTICAST_OBJ->rx_data_rate, &sf, &bw );

        ral_lora_cad_params_t cad_params = { .cad_symb_nb          = RAL_LORA_CAD_04_SYMB,
                                             .cad_det_peak_in_symb = class_b_d2d_get_cad_det_peak_4_symb( sf, bw ),
                                             .cad_det_min_in_symb  = 10,
                                             .cad_exit_mode        = RAL_LORA_CAD_ONLY,
                                             .cad_timeout_in_ms    = 0 };

        smtc_modem_hal_start_radio_tcxo( );
        smtc_modem_hal_assert( ralf_setup_lora( rp->radio, &RP->radio_params[rp->radio_task_id].tx.lora ) ==
                               RAL_STATUS_OK );
        smtc_modem_hal_assert( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_CAD_DONE | RAL_IRQ_CAD_OK ) ==
                               RAL_STATUS_OK );
        // smtc_modem_hal_assert( ral_set_pkt_type( &( rp->radio->ral ), RAL_PKT_TYPE_LORA ) == RAL_STATUS_OK );
        smtc_modem_hal_assert( ral_set_lora_cad_params( &( rp->radio->ral ), &cad_params ) == RAL_STATUS_OK );
        smtc_modem_hal_assert( ral_set_lora_cad( &( rp->radio->ral ) ) == RAL_STATUS_OK );
    }
    else
    {
        smtc_modem_hal_mcu_panic( );
    }
    SMTC_D2D_HAL_TRACE_PRINTF( "launch cad  \n" );
}

static void class_b_d2d_rp_callback( smtc_class_b_d2d_t* class_b_d2d_obj )
{
    radio_planner_t* rp        = RP;
    rp_status_t      rp_status = rp->status[class_b_d2d_obj->classb_d2d_id_rp];
    if( rp_status == RP_STATUS_CAD_NEGATIVE )
    {
        class_b_d2d_cad_to_tx( class_b_d2d_obj );
        return;
    }
    if( rp_status == RP_STATUS_CAD_POSITIVE )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "d2d RP_STATUS_CAD_POSITIVE\n" );

        class_b_d2d_obj->nb_trans_trial_cnt++;
        // radio task is aborted by the rp itself
    }
    else if( rp_status == RP_STATUS_TX_DONE )
    {
        SMTC_D2D_HAL_TRACE_PRINTF( "d2d RP_STATUS_TX_DONE\n" );
        if( class_b_d2d_obj->nb_trans_cnt > 0 )
        {
            class_b_d2d_obj->nb_trans_cnt--;
        }
        // increment event
        //  relaunch taskonly if nbtrans != 0

        smtc_duty_cycle_sum( LR1MAC->dtc_obj, MULTICAST_OBJ->rx_frequency,
                             rp->stats.tx_last_toa_ms[class_b_d2d_obj->classb_d2d_id_rp] );
    }
    else if( rp_status == RP_STATUS_TASK_ABORTED )
    {
        // relaunch taskonly if max nb retry not reached
        class_b_d2d_obj->nb_trans_trial_cnt++;
        SMTC_MODEM_HAL_TRACE_PRINTF( "d2d aborted in the radioplanner\n" );
        // class_b_d2d_cad_to_tx( class_b_d2d_obj );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "d2d receive an unknown status from the radioplanner\n" );
        smtc_modem_hal_mcu_panic( )
    }
    class_b_d2d_obj->tx_on_going = false;

    // Call the Tx Done callback
    if( ( class_b_d2d_obj->nb_trans_cnt == 0 ) ||
        ( class_b_d2d_obj->nb_trans_trial_cnt >= class_b_d2d_obj->nb_trans_max_retry ) )
    {
        class_b_d2d_obj->tx_event_callback( class_b_d2d_obj->tx_event_context );
    }
}

static uint32_t class_b_d2d_get_fcnt_down( smtc_class_b_d2d_t* class_b_d2d_obj )
{
    uint32_t seconds_since_epoch;
    uint32_t fractional_second;
    lr1mac_core_convert_rtc_to_gps_epoch_time( LR1MAC, smtc_modem_hal_get_time_in_ms( ), &seconds_since_epoch,
                                               &fractional_second );

    // number of virtually beacon since 5 jan 1980
    uint32_t number_of_beacon_period_since_gps_epoch = seconds_since_epoch >> 7;

    // number of ping slot per beacon period for this multicast group
    uint8_t number_ping_per_beacon = 1 << ( 7 - MULTICAST_OBJ->ping_slot_periodicity );

    // number of ping slot already consummed in this current beacon
    uint8_t number_ping_since_begin_of_current_beacon =
        number_ping_per_beacon - MULTICAST_OBJ->ping_slot_parameters.ping_number;

    // fcnt down for d2d transaction
    return ( uint32_t )( ( number_of_beacon_period_since_gps_epoch * number_ping_per_beacon ) +
                         number_ping_since_begin_of_current_beacon );
}

static void class_b_d2d_cad_to_tx( smtc_class_b_d2d_t* class_b_d2d_obj )
{
    modulation_type_t modulation_type =
        smtc_real_get_modulation_type_from_datarate( LR1MAC->real, MULTICAST_OBJ->rx_data_rate );
    uint8_t id = class_b_d2d_obj->ping_slot_obj->rp->radio_task_id;

    if( modulation_type == LORA )
    {
        RP->tasks[class_b_d2d_obj->classb_d2d_id_rp].type = RP_TASK_TYPE_TX_LORA;
        smtc_modem_hal_assert( ralf_setup_lora( RP->radio, &RP->radio_params[id].tx.lora ) == RAL_STATUS_OK );
        smtc_modem_hal_assert( ral_set_dio_irq_params( &( RP->radio->ral ), RAL_IRQ_TX_DONE ) == RAL_STATUS_OK );
        smtc_modem_hal_assert( ral_set_pkt_payload( &( RP->radio->ral ), class_b_d2d_obj->tx_payload_encrypt,
                                                    class_b_d2d_obj->tx_payload_size ) == RAL_STATUS_OK );
        smtc_modem_hal_assert( ral_set_tx( &( RP->radio->ral ) ) == RAL_STATUS_OK );
        rp_stats_set_tx_timestamp( &RP->stats, smtc_modem_hal_get_time_in_ms( ) );
    }
    else
    {
        smtc_modem_hal_mcu_panic( );
    }
    SMTC_MODEM_HAL_TRACE_PRINTF( "launch TX , Freq = %d Hz, SF = %d BW = %d preamble length =  %d \n",
                                 RP->radio_params[id].tx.lora.rf_freq_in_hz, RP->radio_params[id].tx.lora.mod_params.sf,
                                 RP->radio_params[id].tx.lora.mod_params.bw,
                                 RP->radio_params[id].tx.lora.pkt_params.preamble_len_in_symb );
}

/**
 * @brief Get the CAD DetPeak value
 * @remark ONLY VALID FOR LR11XX
 *
 * @param sf
 * @param bw
 * @return uint8_t
 */
static uint8_t class_b_d2d_get_cad_det_peak_4_symb( uint8_t sf, lr1mac_bandwidth_t bw )
{
    if( ( bw == BW125 ) || ( bw == BW250 ) )
    {
        if( sf == 7 )
        {
            return 57;
        }
        else if( sf == 8 )
        {
            return 63;
        }
        else if( sf == 9 )
        {
            return 63;
        }
        else if( sf == 10 )
        {
            return 67;
        }
        else if( sf == 11 )
        {
            return 71;
        }
        else if( sf == 12 )
        {
            return 73;
        }
        else
        {
            smtc_modem_hal_mcu_panic( );
        }
    }
    else if( bw == BW500 )
    {
        if( sf == 7 )
        {
            return 82;
        }
        else if( sf == 8 )
        {
            return 90;
        }
        else if( sf == 9 )
        {
            return 83;
        }
        else if( sf == 10 )
        {
            return 85;
        }
        else if( sf == 11 )
        {
            return 84;
        }
        else if( sf == 12 )
        {
            return 87;
        }
        else
        {
            smtc_modem_hal_mcu_panic( );
        }
    }
    else
    {
        smtc_modem_hal_mcu_panic( );
    }
    // Never reached, avoid warning
    return 127;
}