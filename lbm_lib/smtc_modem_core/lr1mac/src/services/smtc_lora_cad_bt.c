/*!
 * \file      smtc_lora_cad_bt.c
 *
 * \brief     LoRa Channel Activities Detection Before Talk Object (CSMA)
 *
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

#include "smtc_modem_hal_dbg_trace.h"
#include "radio_planner.h"
#include "smtc_modem_hal.h"
#include "lr1_stack_mac_layer.h"
#include "smtc_lora_cad_bt.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*!
 * \brief Returns the minimum value between a and b
 *
 * \param [in] a 1st value
 * \param [in] b 2nd value
 * \retval minValue Minimum value
 */
#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * @brief NBO_MAX Number of BackOff MAX
 * @remark The application can utilize different BO_MAX values for prioritization (lower BO_MAX means higher priority
 * and vice versa to send an uplink frame).
 */
#define NBO_MAX ( 6 )

/**
 * @brief MAX_CH_CHANGES The number max of channel change
 */
#define MAX_CH_CHANGES ( 4 )

/**
 * @brief NB_CAD_SYMBOLS_IN_DIFS The number of symbols listen in DIFS
 */
#define NB_CAD_SYMBOLS_IN_DIFS ( RAL_LORA_CAD_04_SYMB )

/**
 * @brief NB_CAD_SYMBOLS_IN_BO The number of symbols listen in BackOff if enabled
 */
#define NB_CAD_SYMBOLS_IN_BO ( RAL_LORA_CAD_02_SYMB )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static smtc_lora_cad_bt_t cad_obj_declare[NUMBER_OF_STACKS];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static void     smtc_cad_bt_launch_radio_callback_for_rp( void* rp_void );
static void     smtc_lora_cad_bt_rp_callback( smtc_lora_cad_bt_t* cad_obj );
static void     smtc_lora_cad_bt_reset_to_difs_phase( smtc_lora_cad_bt_t* cad_obj );
static void     smtc_lora_cad_bt_channel_is_free( smtc_lora_cad_bt_t* cad_obj );
static uint32_t smtc_lora_cad_bt_get_symbol_duration_us( ral_lora_bw_t bw, ral_lora_sf_t sf );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void smtc_lora_cad_bt_init( smtc_lora_cad_bt_t* cad_obj, radio_planner_t* rp, uint8_t cad_id_rp,
                            void ( *ch_free_callback )( void* ch_free_context ), void* ch_free_context,
                            void ( *ch_busy_callback_update_channel )( void* ch_busy_context_update_channel ),
                            void* ch_busy_context_update_channel,
                            void ( *ch_free_callback_on_back_off )( void* ch_free_context_on_back_off ),
                            void* ch_free_context_on_back_off, void ( *abort_callback )( void* abort_context ),
                            void* abort_context )
{
    if( ( ch_free_callback == NULL ) || ( ch_busy_callback_update_channel == NULL ) ||
        ( ch_free_callback_on_back_off == NULL ) || ( abort_callback == NULL ) )
    {
        SMTC_MODEM_HAL_PANIC( "cad bt bad init\n" );
    }

    memset( cad_obj, 0, sizeof( smtc_lora_cad_bt_t ) );

    cad_obj->rp        = rp;
    cad_obj->cad_id4rp = cad_id_rp;  //@none protection if this id already used by un other task

    cad_obj->ch_free_callback                = ch_free_callback;
    cad_obj->ch_free_context                 = ch_free_context;
    cad_obj->ch_busy_callback_update_channel = ch_busy_callback_update_channel;
    cad_obj->ch_busy_context_update_channel  = ch_busy_context_update_channel;
    cad_obj->ch_free_callback_on_back_off    = ch_free_callback_on_back_off;
    cad_obj->ch_free_context_on_back_off     = ch_free_context_on_back_off;
    cad_obj->abort_callback                  = abort_callback;
    cad_obj->abort_context                   = abort_context;

    // cad_obj->bo_enabled             = true;  // TODO remove it, for debug purpose
    cad_obj->nb_bo_max_conf         = NBO_MAX;
    cad_obj->max_ch_change_cnt_conf = MAX_CH_CHANGES;

    cad_obj->cad_state_prev = SMTC_LORA_CAD_DIFS;

    smtc_lora_cad_bt_reset_to_difs_phase( cad_obj );

    rp_release_hook( rp, cad_id_rp );
    rp_hook_init( rp, cad_id_rp, ( void ( * )( void* ) )( smtc_lora_cad_bt_rp_callback ), cad_obj );
}

smtc_lora_cad_status_t smtc_lora_cad_bt_set_parameters( smtc_lora_cad_bt_t* cad_obj, uint8_t max_ch_change,
                                                        bool bo_enabled, uint8_t nb_bo_max )
{
    if( ( max_ch_change > 0 ) && ( max_ch_change <= MAX_CH_CHANGES ) )
    {
        cad_obj->max_ch_change_cnt_conf = max_ch_change;
    }
    else
    {
        return SMTC_LORA_CAD_ERROR;
    }

    if( ( nb_bo_max > 0 ) && ( nb_bo_max <= NBO_MAX ) )
    {
        cad_obj->nb_bo_max_conf = nb_bo_max;
    }
    else
    {
        return SMTC_LORA_CAD_ERROR;
    }

    cad_obj->bo_enabled = bo_enabled;

    return SMTC_LORA_CAD_OK;
}

void smtc_lora_cad_bt_get_parameters( smtc_lora_cad_bt_t* cad_obj, uint8_t* max_ch_change, bool* bo_enabled,
                                      uint8_t* nb_bo_max )
{
    *bo_enabled    = cad_obj->bo_enabled;
    *max_ch_change = cad_obj->max_ch_change_cnt_conf;
    *nb_bo_max     = cad_obj->nb_bo_max_conf;
}

void smtc_lora_cad_bt_set_state( smtc_lora_cad_bt_t* cad_obj, bool enable )
{
    cad_obj->enabled = enable;
}

bool smtc_lora_cad_bt_get_state( smtc_lora_cad_bt_t* cad_obj )
{
    return cad_obj->enabled;
}

void smtc_lora_cad_bt_listen_channel( smtc_lora_cad_bt_t* cad_obj, uint32_t freq_hz, ral_lora_sf_t sf,
                                      ral_lora_bw_t bandwidth, bool is_at_time, uint32_t target_time_ms,
                                      uint32_t tx_duration_ms, uint8_t nb_available_channel, bool invert_iq_is_on )
{
    // SMTC_MODEM_HAL_TRACE_PRINTF( "smtc_lora_cad_bt_listen_channel\n" );
    if( cad_obj->is_cad_running == true )
    {
        return;
    }

    cad_obj->is_at_time = is_at_time;
    if( ( cad_obj->ch_free_callback == NULL ) || ( cad_obj->ch_busy_callback_update_channel == NULL ) ||
        ( cad_obj->ch_free_callback_on_back_off == NULL ) || ( cad_obj->abort_callback == NULL ) )
    {
        SMTC_MODEM_HAL_PANIC( "cad_obj bad initialization \n" );
    }

    ralf_params_lora_cad_t lora_cad_param     = { 0 };
    rp_radio_params_t      radio_params       = { 0 };
    rp_task_t              rp_task            = { 0 };
    uint32_t               listen_duration_ms = 0;

    uint32_t symbol_duration_us = smtc_lora_cad_bt_get_symbol_duration_us( bandwidth, sf );

    cad_obj->max_ch_change_cnt = MIN( cad_obj->max_ch_change_cnt, nb_available_channel );

    // If the BO is enabled and we are in the first step of the diagram, set the random nb_bo
    if( cad_obj->cad_state == SMTC_LORA_CAD_DIFS )
    {
        if( cad_obj->cad_state_prev == SMTC_LORA_CAD_DIFS )
        {
            if( cad_obj->bo_enabled == true )
            {
                // the nb_bo is not recomputed when different not 0
                if( cad_obj->nb_bo == 0 )
                {
                    cad_obj->nb_bo = smtc_modem_hal_get_random_nb_in_range( 1, cad_obj->nb_bo_max_conf );
                    SMTC_MODEM_HAL_TRACE_PRINTF( "set nb_bo:%d, target_time %u\n", cad_obj->nb_bo, target_time_ms );
                }
            }
        }

        // Compute listen duration of the DIFS phase
        listen_duration_ms += ( ( ( 1 << NB_CAD_SYMBOLS_IN_DIFS ) * symbol_duration_us ) / 1000 );

        lora_cad_param.ral_lora_cad_params.cad_symb_nb = NB_CAD_SYMBOLS_IN_DIFS;
    }
    else
    {
        lora_cad_param.ral_lora_cad_params.cad_symb_nb = NB_CAD_SYMBOLS_IN_BO;
    }

    // Compute listen duration of the BO phase if BO enabled
    if( cad_obj->nb_bo > 0 )
    {
        listen_duration_ms += ( cad_obj->nb_bo * ( ( 1 << NB_CAD_SYMBOLS_IN_BO ) * symbol_duration_us ) ) / 1000;
        listen_duration_ms += cad_obj->nb_bo * smtc_modem_hal_get_radio_tcxo_startup_delay_ms( );
    }

    lora_cad_param.ral_lora_cad_params.cad_det_min_in_symb = 10;
    lora_cad_param.ral_lora_cad_params.cad_exit_mode       = RAL_LORA_CAD_ONLY;
    lora_cad_param.ral_lora_cad_params.cad_timeout_in_ms   = 1000;

    uint8_t      cad_det_peak        = 0;
    ral_status_t status_ral_det_peak = ral_get_lora_cad_det_peak(
        &( cad_obj->rp->radio->ral ), sf, bandwidth, lora_cad_param.ral_lora_cad_params.cad_symb_nb, &cad_det_peak );

    switch( status_ral_det_peak )
    {
    case RAL_STATUS_OK:
        // CAD is supported by the radio
        break;
    case RAL_STATUS_UNSUPPORTED_FEATURE:
        // CAD is not supported by the radio, consider the channel as free
        smtc_lora_cad_bt_channel_is_free( cad_obj );
        return;
        break;
    default:
        SMTC_MODEM_HAL_PANIC( );
        break;
    }

    lora_cad_param.ral_lora_cad_params.cad_det_peak_in_symb = cad_obj->detect_peak_offset + cad_det_peak;

    lora_cad_param.rf_freq_in_hz   = freq_hz;
    lora_cad_param.invert_iq_is_on = invert_iq_is_on;
    lora_cad_param.sf              = sf;
    lora_cad_param.bw              = bandwidth;

    radio_params.pkt_type    = RAL_PKT_TYPE_LORA;
    radio_params.rx.lora_cad = lora_cad_param;

    uint8_t my_hook_id;
    if( rp_hook_get_id( cad_obj->rp, cad_obj, &my_hook_id ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_PANIC( "radio planner isn't initialized for cad obj \n" );
    }

    rp_task.hook_id                    = my_hook_id;
    rp_task.type                       = RP_TASK_TYPE_CAD;
    rp_task.duration_time_ms           = listen_duration_ms + tx_duration_ms;
    rp_task.launch_task_callbacks      = smtc_cad_bt_launch_radio_callback_for_rp;
    rp_task.schedule_task_low_priority = false;
    rp_task.start_time_ms = target_time_ms - listen_duration_ms - smtc_modem_hal_get_radio_tcxo_startup_delay_ms( );
    if( is_at_time == true )
    {
        rp_task.state = RP_TASK_STATE_SCHEDULE;
    }
    else
    {
        rp_task.state = RP_TASK_STATE_ASAP;
    }

    if( rp_task_enqueue( cad_obj->rp, &rp_task, NULL, 0, &radio_params ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Radio planner cad hook %d is busy \n", my_hook_id );
        cad_obj->abort_callback( cad_obj->abort_context );
        smtc_lora_cad_bt_reset_to_difs_phase( cad_obj );
    }
    else
    {
        cad_obj->is_cad_running = true;

        if( cad_obj->cad_state == SMTC_LORA_CAD_DIFS )
        {
            // SMTC_MODEM_HAL_TRACE_PRINTF(
            //    "symb_duration_us %u, TOA:%u, listen_duration_ms %u, task_duration:%u, is_at_time:%d\n",
            //    symbol_duration_us, tx_duration_ms, listen_duration_ms, rp_task.duration_time_ms, is_at_time );
        }
    }
}

smtc_lora_cad_bt_t* smtc_cad_get_obj( uint8_t stack_id )
{
    if( stack_id < NUMBER_OF_STACKS )
    {
        return ( &cad_obj_declare[stack_id] );
    }
    else
    {
        return NULL;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void smtc_cad_bt_launch_radio_callback_for_rp( void* rp_void )
{
    radio_planner_t*        rp              = ( radio_planner_t* ) rp_void;
    uint8_t                 id              = rp->radio_task_id;
    ralf_params_lora_cad_t* params_lora_cad = &rp->radio_params[id].rx.lora_cad;

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ralf_setup_lora_cad( rp->radio, params_lora_cad ) == RAL_STATUS_OK );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_CAD_DONE | RAL_IRQ_CAD_OK ) ==
                                     RAL_STATUS_OK );

    // Wait the exact expected time (ie target - tcxo startup delay)
    while( ( int32_t ) ( rp->tasks[id].start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) > 0 )
    {
        // Do nothing
    }
    smtc_modem_hal_start_radio_tcxo( );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( ral_set_lora_cad( &( rp->radio->ral ) ) == RAL_STATUS_OK );

    rp_stats_set_rx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
}

static void smtc_lora_cad_bt_rp_callback( smtc_lora_cad_bt_t* cad_obj )
{
    // SMTC_MODEM_HAL_TRACE_PRINTF( "smtc_lora_cad_bt_rp_callback\n" );
    uint32_t    tcurrent_ms;
    rp_status_t rp_status;
    uint8_t     my_hook_id;
    rp_hook_get_id( cad_obj->rp, cad_obj, &my_hook_id );
    rp_get_status( cad_obj->rp, my_hook_id, &tcurrent_ms, &( rp_status ) );

    cad_obj->is_cad_running = false;

    if( rp_status == RP_STATUS_CAD_NEGATIVE )
    {
        // SMTC_MODEM_HAL_TRACE_PRINTF( "CAD_NEGATIF\n" );

        if( cad_obj->is_at_time == true )
        {
            smtc_lora_cad_bt_channel_is_free( cad_obj );
        }
        else
        {
            // Decrement nb_bo only in Back Off step when the result of the previous BO CAD is free
            if( cad_obj->cad_state_prev == SMTC_LORA_CAD_BO )
            {
                cad_obj->nb_bo--;
            }

            if( cad_obj->nb_bo == 0 )
            {
                smtc_lora_cad_bt_channel_is_free( cad_obj );
            }
            else
            {
                cad_obj->cad_state = SMTC_LORA_CAD_BO;
                cad_obj->ch_free_callback_on_back_off( cad_obj->ch_free_context_on_back_off );
            }
        }
    }
    else if( rp_status == RP_STATUS_CAD_POSITIVE )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "RP_STATUS_CAD_POSITIVE\n" );

        if( cad_obj->is_at_time == true )
        {
            cad_obj->abort_callback( cad_obj->abort_context );
            smtc_lora_cad_bt_reset_to_difs_phase( cad_obj );
        }
        else
        {
            // what ever the cad state SMTC_LORA_CAD_DIFS or SMTC_LORA_CAD_BO
            if( cad_obj->max_ch_change_cnt > 0 )
            {
                cad_obj->max_ch_change_cnt--;
                cad_obj->cad_state = SMTC_LORA_CAD_DIFS;
                cad_obj->ch_busy_callback_update_channel( cad_obj->ch_busy_context_update_channel );
            }
            else
            {
                smtc_lora_cad_bt_channel_is_free( cad_obj );
            }
        }
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "receive an unknown status (%d) from the radio planner hook %d\n", rp_status,
                                    my_hook_id );
        cad_obj->abort_callback( cad_obj->abort_context );
        smtc_lora_cad_bt_reset_to_difs_phase( cad_obj );
    }

    cad_obj->cad_state_prev = cad_obj->cad_state;
}

static void smtc_lora_cad_bt_reset_to_difs_phase( smtc_lora_cad_bt_t* cad_obj )
{
    cad_obj->cad_state         = SMTC_LORA_CAD_DIFS;
    cad_obj->max_ch_change_cnt = cad_obj->max_ch_change_cnt_conf;
    cad_obj->nb_bo             = 0;
}

static void smtc_lora_cad_bt_channel_is_free( smtc_lora_cad_bt_t* cad_obj )
{
    cad_obj->ch_free_callback( cad_obj->ch_free_context );
    smtc_lora_cad_bt_reset_to_difs_phase( cad_obj );
}

static uint32_t smtc_lora_cad_bt_get_symbol_duration_us( ral_lora_bw_t bw, ral_lora_sf_t sf )
{
    uint32_t bw_temp = 125;  // temporary variable to store the bandwidth
    switch( bw )
    {
    case RAL_LORA_BW_007_KHZ:
        bw_temp = 7;
        break;
    case RAL_LORA_BW_010_KHZ:
        bw_temp = 10;
        break;
    case RAL_LORA_BW_015_KHZ:
        bw_temp = 15;
        break;
    case RAL_LORA_BW_020_KHZ:
        bw_temp = 20;
        break;
    case RAL_LORA_BW_031_KHZ:
        bw_temp = 31;
        break;
    case RAL_LORA_BW_041_KHZ:
        bw_temp = 41;
        break;
    case RAL_LORA_BW_062_KHZ:
        bw_temp = 62;
        break;
    case RAL_LORA_BW_125_KHZ:
        bw_temp = 125;
        break;
    case RAL_LORA_BW_200_KHZ:
        bw_temp = 200;
        break;
    case RAL_LORA_BW_250_KHZ:
        bw_temp = 250;
        break;
    case RAL_LORA_BW_400_KHZ:
        bw_temp = 400;
        break;
    case RAL_LORA_BW_500_KHZ:
        bw_temp = 500;
        break;
    case RAL_LORA_BW_800_KHZ:
        bw_temp = 800;
        break;
    case RAL_LORA_BW_1600_KHZ:
        bw_temp = 1600;
        break;
    default:
        SMTC_MODEM_HAL_PANIC( " invalid BW " );
        break;
    }
    return ( ( ( uint32_t ) ( ( 1 << sf ) * 1000 ) / bw_temp ) );
}

/* --- EOF ------------------------------------------------------------------ */
