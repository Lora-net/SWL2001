/*!
 * \file      smtc_lbt.c
 *
 * \brief     Listen Before Talk Object
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

#include "smtc_lbt.h"

#include "smtc_modem_hal_dbg_trace.h"
#include "radio_planner.h"
#include "stddef.h"
#include "smtc_modem_hal.h"
#include "lr1_stack_mac_layer.h"

void smtc_lbt_init( smtc_lbt_t* lbt_obj, radio_planner_t* rp, uint8_t lbt_id_rp,
                    void ( *free_callback )( void* free_context ), void*   free_context,
                    void ( *busy_callback )( void* busy_context ), void*   busy_context,
                    void ( *abort_callback )( void* abort_context ), void* abort_context )
{
    if( ( free_callback == NULL ) || ( busy_callback == NULL ) || ( abort_callback == NULL ) )
    {
        smtc_modem_hal_mcu_panic( "lbt bad init\n" );
    }
    lbt_obj->rp                 = rp;
    lbt_obj->lbt_id4rp          = lbt_id_rp;  //@none protection if this id already used by un other task
    lbt_obj->free_callback      = free_callback;
    lbt_obj->free_context       = free_context;
    lbt_obj->busy_callback      = busy_callback;
    lbt_obj->busy_context       = busy_context;
    lbt_obj->abort_callback     = abort_callback;
    lbt_obj->abort_context      = abort_context;
    lbt_obj->rssi_inst          = 0;
    lbt_obj->rssi_accu          = 0;
    lbt_obj->rssi_nb_of_meas    = 0;
    lbt_obj->is_at_time         = false;
    lbt_obj->enabled            = false;
    lbt_obj->listen_duration_ms = 0;
    lbt_obj->threshold          = 0;
    lbt_obj->bw_hz              = 0;
    rp_release_hook( rp, lbt_id_rp );
    rp_hook_init( rp, lbt_id_rp, ( void ( * )( void* ) )( smtc_lbt_rp_callback ), lbt_obj );
}

void smtc_lbt_set_parameters( smtc_lbt_t* lbt_obj, uint32_t listen_duration_ms, int16_t threshold_dbm, uint32_t bw_hz )
{
    lbt_obj->rssi_inst          = 0;
    lbt_obj->rssi_accu          = 0;
    lbt_obj->rssi_nb_of_meas    = 0;
    lbt_obj->listen_duration_ms = listen_duration_ms + LAP_OF_TIME_TO_GET_A_RSSI_VALID;
    lbt_obj->threshold          = threshold_dbm;
    lbt_obj->bw_hz              = bw_hz;
}

void smtc_lbt_get_parameters( smtc_lbt_t* lbt_obj, uint32_t* listen_duration_ms, int16_t* threshold_dbm,
                              uint32_t* bw_hz )
{
    *listen_duration_ms = lbt_obj->listen_duration_ms - LAP_OF_TIME_TO_GET_A_RSSI_VALID;
    *threshold_dbm      = lbt_obj->threshold;
    *bw_hz              = lbt_obj->bw_hz;
}

void smtc_lbt_set_state( smtc_lbt_t* lbt_obj, bool enable )
{
    lbt_obj->enabled = enable;
}

bool smtc_lbt_get_state( smtc_lbt_t* lbt_obj )
{
    return lbt_obj->enabled;
}

void smtc_lbt_launch_callback_for_rp( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;
    int16_t          rssi_tmp;
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_assert( ral_set_pkt_type( &( rp->radio->ral ), rp->radio_params[id].pkt_type ) == RAL_STATUS_OK );
    smtc_modem_hal_assert( ral_set_rf_freq( &( rp->radio->ral ), rp->radio_params[id].rx.gfsk.rf_freq_in_hz ) ==
                           RAL_STATUS_OK );
    smtc_modem_hal_assert( ral_set_gfsk_mod_params( &( rp->radio->ral ), &rp->radio_params[id].rx.gfsk.mod_params ) ==
                           RAL_STATUS_OK );
    smtc_modem_hal_assert( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_NONE ) == RAL_STATUS_OK );
    smtc_modem_hal_assert( ral_set_rx( &( rp->radio->ral ), RAL_RX_TIMEOUT_CONTINUOUS_MODE ) == RAL_STATUS_OK );

    uint32_t carrier_sense_time = smtc_modem_hal_get_time_in_ms( );
    while( ( int32_t )( carrier_sense_time + LAP_OF_TIME_TO_GET_A_RSSI_VALID - smtc_modem_hal_get_time_in_ms( ) ) > 0 )
    {  // delay LAP_OF_TIME_TO_GET_A_RSSI_VALID ms
    }
    do
    {
        smtc_modem_hal_assert( ral_get_rssi_inst( &( rp->radio->ral ), &rssi_tmp ) == RAL_STATUS_OK );
        ( ( smtc_lbt_t* ) rp->hooks[id] )->rssi_inst = rssi_tmp;
        ( ( smtc_lbt_t* ) rp->hooks[id] )->rssi_accu += rssi_tmp;
        ( ( smtc_lbt_t* ) rp->hooks[id] )->rssi_nb_of_meas++;
        if( rssi_tmp >= rp->radio_params[id].lbt_threshold )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "lbt rssi: %d dBm\n", rssi_tmp );
            rp->status[id] = RP_STATUS_LBT_BUSY_CHANNEL;
            rp_radio_irq_callback( rp_void );
            return;
        }
    } while( ( int32_t )( carrier_sense_time + rp->radio_params[id].rx.timeout_in_ms -
                          smtc_modem_hal_get_time_in_ms( ) ) > 0 );

    rp->status[id] = RP_STATUS_LBT_FREE_CHANNEL;
    rp_radio_irq_callback( rp_void );
}

void smtc_lbt_listen_channel( smtc_lbt_t* lbt_obj, uint32_t freq, bool is_at_time, uint32_t target_time_ms,
                              uint32_t tx_duration_ms )
{
    lbt_obj->is_at_time = is_at_time;
    if( ( lbt_obj->free_callback == NULL ) || ( lbt_obj->busy_callback == NULL ) ||
        ( lbt_obj->abort_callback == NULL ) )
    {
        smtc_modem_hal_mcu_panic( "lbt_obj bad initialization \n" );
    }

    ralf_params_gfsk_t gfsk_param;
    rp_radio_params_t  radio_params;
    rp_task_t          rp_task = { 0 };
    memset( &radio_params, 0, sizeof( rp_radio_params_t ) );
    memset( &rp_task, 0, sizeof( rp_task_t ) );
    memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

    gfsk_param.dc_free_is_on = true;
    gfsk_param.rf_freq_in_hz = freq;

    gfsk_param.mod_params.br_in_bps    = lbt_obj->bw_hz >> 1;
    gfsk_param.mod_params.bw_dsb_in_hz = lbt_obj->bw_hz;
    gfsk_param.mod_params.pulse_shape  = RAL_GFSK_PULSE_SHAPE_BT_1;
    gfsk_param.mod_params.fdev_in_hz   = lbt_obj->bw_hz >> 2;

    radio_params.pkt_type         = RAL_PKT_TYPE_GFSK;
    radio_params.rx.gfsk          = gfsk_param;
    radio_params.rx.timeout_in_ms = lbt_obj->listen_duration_ms;
    radio_params.lbt_threshold    = lbt_obj->threshold;

    uint8_t my_hook_id;
    if( rp_hook_get_id( lbt_obj->rp, lbt_obj, &my_hook_id ) != RP_HOOK_STATUS_OK )
    {
        smtc_modem_hal_mcu_panic( "radioplanner isn't initialized for lbt obj \n" );
    }
    rp_task.hook_id               = my_hook_id;
    rp_task.duration_time_ms      = lbt_obj->listen_duration_ms + tx_duration_ms;
    rp_task.type                  = RP_TASK_TYPE_LBT;
    rp_task.launch_task_callbacks = smtc_lbt_launch_callback_for_rp;
    if( is_at_time == true )
    {
        rp_task.start_time_ms =
            target_time_ms - lbt_obj->listen_duration_ms - smtc_modem_hal_get_radio_tcxo_startup_delay_ms( );
        rp_task.state = RP_TASK_STATE_SCHEDULE;
    }
    else
    {
        rp_task.start_time_ms = target_time_ms;
        rp_task.state         = RP_TASK_STATE_ASAP;
        rp_task.state         = RP_TASK_STATE_ASAP;
    }

    if( rp_task_enqueue( lbt_obj->rp, &rp_task, NULL, 0, &radio_params ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Radio planner hook %d is busy \n", my_hook_id );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "  Listen Frequency = %u during %d ms \n", freq,
                                     lbt_obj->listen_duration_ms - LAP_OF_TIME_TO_GET_A_RSSI_VALID );
    }
}

void smtc_lbt_rp_callback( smtc_lbt_t* lbt_obj )
{
    uint32_t    tcurrent_ms;
    rp_status_t rp_status;
    uint8_t     my_hook_id;
    rp_hook_get_id( lbt_obj->rp, lbt_obj, &my_hook_id );
    rp_get_status( lbt_obj->rp, my_hook_id, &tcurrent_ms, &( rp_status ) );
    if( rp_status == RP_STATUS_LBT_FREE_CHANNEL )
    {
        lbt_obj->free_callback( lbt_obj->free_context );
    }
    else if( rp_status == RP_STATUS_LBT_BUSY_CHANNEL )
    {
        if( lbt_obj->is_at_time == true )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "lbt abort max retry reach \n" );
            lbt_obj->abort_callback( lbt_obj->abort_context );
        }
        else
        {
            lbt_obj->busy_callback( lbt_obj->busy_context );
        }
    }
    else
    {
        lbt_obj->abort_callback( lbt_obj->abort_context );
    }
}