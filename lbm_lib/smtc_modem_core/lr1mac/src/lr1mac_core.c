/*!
 * \file      lr1_mac_core.c
 *
 * \brief     LoRaWan core definition
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
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */

#include "lr1mac_core.h"

#include "smtc_modem_hal_dbg_trace.h"
#include "lr1mac_utilities.h"
#include "smtc_modem_hal.h"
#include "smtc_real.h"
#include "smtc_real_defs.h"
#include "smtc_real_defs_str.h"

#include "lr1mac_config.h"

#if defined( ADD_RELAY_TX )
#include "relay_tx_api.h"
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define DBG_PRINT_WITH_LINE( ... )                                                        \
    do                                                                                    \
    {                                                                                     \
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n  *************************************\n  * " ); \
        SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ );                                       \
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n  *************************************\n" );     \
    } while( 0 );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
#define FAILSAFE_DURATION 300U

#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
static const char* smtc_name_bw[]         = { "BW007", "BW010", "BW015", "BW020", "BW031", "BW041", "BW062",
                                              "BW125", "BW200", "BW250", "BW400", "BW500", "BW800", "BW1600" };
static const char* smtc_name_lr_fhss_bw[] = { "BW 39063",  "BW 85938",  "BW 136719", "BW 183594",  "BW 335938",
                                              "BW 386719", "BW 722656", "BW 773438", "BW 1523438", "BW 1574219" };
static const char* smtc_name_lr_fhss_cr[] = { "CR 5/6", "CR 2/3", "CR 1/2", "CR 1/3" };
#endif

#ifndef DISABLE_LORAWAN_RX_WINDOWS
#define DISABLE_LORAWAN_RX_WINDOWS 0
#endif

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE VARIABLES -------------------------------------------------------------
 */
/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DECLARATION -------------------------------------------------
 */
static void copy_user_payload( lr1_stack_mac_t* lr1_mac_obj, const uint8_t* data_in, const uint8_t size_in );
static void lr1mac_mac_update( lr1_stack_mac_t* lr1_mac_obj );
/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS DEFINITIONS --------------------------------------------------
 */

void lr1mac_core_init( lr1_stack_mac_t* lr1_mac_obj, smtc_real_t* real, radio_planner_t* rp,
                       lr1mac_activation_mode_t activation_mode,
                       void ( *push_callback )( lr1_stack_mac_down_data_t* push_context ), void* stack_id )
{
    memset( lr1_mac_obj, 0, sizeof( lr1_stack_mac_t ) );
    lr1_mac_obj->stack_id                    = ( *( uint8_t* ) stack_id );
    lr1_mac_obj->lr1mac_state                = LWPSTATE_IDLE;
    lr1_mac_obj->valid_rx_packet             = NO_MORE_VALID_RX_PACKET;
    lr1_mac_obj->stack_id4rp                 = RP_HOOK_ID_LR1MAC_STACK + ( *( uint8_t* ) stack_id );
    lr1_mac_obj->real                        = real;
    lr1_mac_obj->send_at_time                = false;
    lr1_mac_obj->push_callback               = push_callback;
    lr1_mac_obj->push_context                = &( lr1_mac_obj->rx_down_data );
    lr1_mac_obj->crystal_error               = BSP_CRYSTAL_ERROR;
    lr1_mac_obj->device_time_invalid_delay_s = LR1MAC_DEVICE_TIME_DELAY_TO_BE_NO_SYNC;
    lr1_mac_obj->no_rx_windows               = DISABLE_LORAWAN_RX_WINDOWS;

    lr1_mac_obj->rx_down_data.stack_id = lr1_mac_obj->stack_id;

    lr1_stack_mac_init( lr1_mac_obj, activation_mode );

    status_lorawan_t status = lr1mac_core_context_load( lr1_mac_obj );

    if( status == OKLORAWAN )
    {
        // Check if the region stored in flash is still valid
        if( smtc_real_is_supported_region( lr1_mac_obj->real->region_type ) == SMTC_REAL_STATUS_OK )
        {
            status = OKLORAWAN;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "Region (0x%x) in nvm context not supported\n",
                                        lr1_mac_obj->real->region_type );
            status = ERRORLORAWAN;
        }
    }

    if( status != OKLORAWAN )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "No valid lr1mac context --> Factory reset\n" );
        lr1mac_core_context_factory_reset( lr1_mac_obj );
        lr1mac_core_context_load( lr1_mac_obj );
        lr1_mac_obj->real->region_type = ( smtc_real_region_types_t ) smtc_real_region_list[0];
        lr1mac_core_context_save( lr1_mac_obj );
    }

    lr1_mac_obj->rp                           = rp;
    lr1_mac_obj->no_rx_packet_reset_threshold = LR1MAC_NO_RX_PACKET_RESET_THRESHOLD;

    rp_hook_init( lr1_mac_obj->rp, lr1_mac_obj->stack_id4rp, ( void ( * )( void* ) )( lr1_stack_mac_rp_callback ),
                  ( lr1_mac_obj ) );
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "rp_hook_init done for hook : %d \n", lr1_mac_obj->stack_id4rp );

    lr1_stack_mac_region_init( lr1_mac_obj, lr1_mac_obj->real->region_type );
    lr1_stack_mac_region_config( lr1_mac_obj );

    // Initialize here adr_ack_limit_init and adr_ack_delay_init which are real dependant and must be updated after the
    // real is initialized and not reinit after the join accept
    lr1_mac_obj->adr_ack_limit_init = smtc_real_get_adr_ack_limit( lr1_mac_obj->real );
    lr1_mac_obj->adr_ack_delay_init = smtc_real_get_adr_ack_delay( lr1_mac_obj->real );

    if( lr1_mac_obj->is_lorawan_modem_certification_enabled == true )
    {
        lr1_mac_obj->is_join_duty_cycle_backoff_bypass_enabled = true;
    }

    SMTC_MODEM_HAL_TRACE_PRINTF( "stack_id %u\n", lr1_mac_obj->stack_id );
    SMTC_MODEM_HAL_TRACE_PRINTF( " DevNonce = %d\n", lr1_mac_obj->dev_nonce );
    SMTC_MODEM_HAL_TRACE_PRINTF( " JoinNonce = 0x%02x %02x %02x, NetID = 0x%02x %02x %02x\n",
                                 lr1_mac_obj->join_nonce[0], lr1_mac_obj->join_nonce[1], lr1_mac_obj->join_nonce[2],
                                 lr1_mac_obj->join_nonce[3], lr1_mac_obj->join_nonce[4], lr1_mac_obj->join_nonce[5] );
    SMTC_MODEM_HAL_TRACE_PRINTF( " Region = %s\n", smtc_real_region_list_str[lr1_mac_obj->real->region_type] );
}

/***********************************************************************************************/
/*    LoraWanProcess Method                                                                    */
/***********************************************************************************************/

lr1mac_states_t lr1mac_core_process( lr1_stack_mac_t* lr1_mac_obj )
{
    uint8_t myhook_id;
    bool    timer_in_past = false;
    rp_hook_get_id( lr1_mac_obj->rp, ( void* ) ( ( lr1_mac_obj ) ), &myhook_id );



    if( ( lr1_mac_obj->lr1mac_state != LWPSTATE_IDLE ) &&
        ( ( int32_t ) ( smtc_modem_hal_get_time_in_s( ) - lr1_mac_obj->timestamp_failsafe - FAILSAFE_DURATION ) > 0 ) )
    {
        SMTC_MODEM_HAL_PANIC( "FAILSAFE EVENT OCCUR (lr1mac_state:0x%x)\n", lr1_mac_obj->lr1mac_state );
        lr1_mac_obj->lr1mac_state = LWPSTATE_ERROR;
    }
    if( lr1_mac_obj->radio_process_state == RADIOSTATE_ABORTED_BY_RP )
    {
        lr1mac_mac_update( lr1_mac_obj );
    }

    switch( lr1_mac_obj->lr1mac_state )
    {
    //**********************************************************************************
    //                                    STATE IDLE
    //**********************************************************************************
    case LWPSTATE_IDLE:
        break;
        //**********************************************************************************
    //                              STATE TXwait MAC
    //**********************************************************************************
    case LWPSTATE_TX_WAIT:
        SMTC_MODEM_HAL_TRACE_PRINTF( " ." );
        lr1_mac_obj->lr1mac_state = LWPSTATE_SEND;  //@note the frame have already been prepare in Update Mac Layer

        // Break is missing du to the fact that the send is immediatly enqueued
        // Intentional fallthrough

    //**********************************************************************************
    //                                    STATE TX
    //**********************************************************************************
    case LWPSTATE_SEND:

        switch( lr1_mac_obj->radio_process_state )
        {
        case RADIOSTATE_IDLE:
        {
            lr1_mac_obj->radio_process_state = RADIOSTATE_PENDING;
            DBG_PRINT_WITH_LINE( "Send Payload  for stack_id = %d", lr1_mac_obj->stack_id );

            uint8_t            tx_sf;
            lr1mac_bandwidth_t tx_bw;
            modulation_type_t  tx_modulation_type =
                smtc_real_get_modulation_type_from_datarate( lr1_mac_obj->real, lr1_mac_obj->tx_data_rate );

            if( tx_modulation_type == LORA )
            {
                smtc_real_lora_dr_to_sf_bw( lr1_mac_obj->real, lr1_mac_obj->tx_data_rate, &tx_sf, &tx_bw );

                SMTC_MODEM_HAL_TRACE_PRINTF(
                    "  Tx  LoRa at %u ms: freq:%u, SF%u, %s, len %u bytes %d dBm, fcnt_up %d, toa = %d\n",
                    lr1_mac_obj->rtc_target_timer_ms, lr1_mac_obj->tx_frequency, tx_sf, smtc_name_bw[tx_bw],
                    lr1_mac_obj->tx_payload_size, lr1_mac_obj->tx_power, lr1_mac_obj->fcnt_up,
                    lr1_stack_toa_get( lr1_mac_obj ) );
            }
            else if( tx_modulation_type == FSK )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF(
                    "  Tx  FSK  at %u ms: freq:%u, len %u bytes %d dBm, fcnt_up %d, toa = %d\n",
                    lr1_mac_obj->rtc_target_timer_ms, lr1_mac_obj->tx_frequency, lr1_mac_obj->tx_payload_size,
                    lr1_mac_obj->tx_power, lr1_mac_obj->fcnt_up, lr1_stack_toa_get( lr1_mac_obj ) );
            }
            else if( tx_modulation_type == LR_FHSS )
            {
                lr_fhss_v1_cr_t tx_cr;
                lr_fhss_v1_bw_t tx_bw;
                smtc_real_lr_fhss_dr_to_cr_bw( lr1_mac_obj->real, lr1_mac_obj->tx_data_rate, &tx_cr, &tx_bw );
                SMTC_MODEM_HAL_TRACE_PRINTF(
                    "  Tx  LR FHSS at %u ms: freq:%u, DR%u (%s, %s Hz), len %u bytes, %d dBm, fcnt_up %d, toa = %d\n",
                    lr1_mac_obj->rtc_target_timer_ms, lr1_mac_obj->tx_frequency, lr1_mac_obj->tx_data_rate,
                    smtc_name_lr_fhss_cr[tx_cr], smtc_name_lr_fhss_bw[tx_bw], lr1_mac_obj->tx_payload_size,
                    lr1_mac_obj->tx_power, lr1_mac_obj->fcnt_up, lr1_stack_toa_get( lr1_mac_obj ) );
            }
            lr1_stack_mac_tx_radio_start( lr1_mac_obj );

            break;
        }
        case RADIOSTATE_TX_FINISHED:
        {
            DBG_PRINT_WITH_LINE( " TX DONE" );
            lr1_mac_obj->lr1mac_state               = LWPSTATE_RX1;
            lr1_mac_obj->tx_duty_cycle_timestamp_ms = lr1_mac_obj->isr_tx_done_radio_timestamp;
            lr1_mac_obj->tx_duty_cycle_time_off_ms =
                ( lr1_mac_obj->rp->stats.tx_last_toa_ms[myhook_id] << lr1_mac_obj->max_duty_cycle_index ) -
                lr1_mac_obj->rp->stats.tx_last_toa_ms[myhook_id];

            if( lr1_mac_obj->join_status == JOINING )
            {
                // save devnonce after the end of TX
                lr1mac_core_context_save( lr1_mac_obj );
            }

            lr1_stack_mac_update_tx_done( lr1_mac_obj );

            if( ( lr1_mac_obj->no_rx_windows == 0 ) || ( lr1_mac_obj->join_status != JOINED ) )
            {
                timer_in_past = lr1_stack_mac_rx_timer_configure( lr1_mac_obj, RX1 );
            }
            else
            {
                lr1mac_mac_update( lr1_mac_obj );
            }

            break;
        }
        default:
            // Do nothing
            break;
        }
        if( timer_in_past == false )
        {
            break;
        }
        else
        {
            timer_in_past = false;
        }
        // Intentional fallthrough

    //**********************************************************************************
    //                                   STATE RX1
    //**********************************************************************************
    case LWPSTATE_RX1:
        if( lr1_mac_obj->radio_process_state == RADIOSTATE_RX_FINISHED )
        {
            if( lr1_mac_obj->rp_planner_status == RP_STATUS_RX_PACKET )
            {
                lr1_mac_obj->rx_down_data.rx_metadata.rx_window = RECEIVE_ON_RX1;
                lr1_mac_obj->valid_rx_packet                    = lr1_stack_mac_rx_frame_decode( lr1_mac_obj );
                if( lr1_mac_obj->valid_rx_packet == NO_MORE_VALID_RX_PACKET )
                {
                    lr1_mac_obj->lr1mac_state = LWPSTATE_RX2;
                    DBG_PRINT_WITH_LINE( "Receive a bad packet on RX1 for stack_id = %d continue with RX2 ",
                                         lr1_mac_obj->stack_id );
                    timer_in_past = lr1_stack_mac_rx_timer_configure( lr1_mac_obj, RX2 );
                }
                else
                {
                    DBG_PRINT_WITH_LINE( "Receive a Valid downlink RX1 for stack_id = %d", lr1_mac_obj->stack_id );
                    lr1mac_mac_update( lr1_mac_obj );
                }
            }
            else
            {
                lr1_mac_obj->lr1mac_state = LWPSTATE_RX2;
                DBG_PRINT_WITH_LINE( "RX1 Timeout for stack_id = %d", lr1_mac_obj->stack_id );
                timer_in_past = lr1_stack_mac_rx_timer_configure( lr1_mac_obj, RX2 );
            }
        }
        if( timer_in_past == false )
        {
            break;
        }
        else
        {
            timer_in_past = false;
        }
        // Intentional fallthrough

    //**********************************************************************************
    //                                   STATE RX2
    //**********************************************************************************
    case LWPSTATE_RX2:
#if !defined( ADD_RELAY_TX )
        if( lr1_mac_obj->radio_process_state == RADIOSTATE_RX_FINISHED )
        {
            if( lr1_mac_obj->rp_planner_status == RP_STATUS_RX_PACKET )
            {
                lr1_mac_obj->rx_down_data.rx_metadata.rx_window = RECEIVE_ON_RX2;
                lr1_mac_obj->valid_rx_packet                    = lr1_stack_mac_rx_frame_decode( lr1_mac_obj );
                if( lr1_mac_obj->valid_rx_packet == NO_MORE_VALID_RX_PACKET )
                {
                    DBG_PRINT_WITH_LINE( "Receive a bad packet on RX2 for stack_id = %d", lr1_mac_obj->stack_id );
                }
                else
                {
                    DBG_PRINT_WITH_LINE( "Receive a Valid downlink RX2 for stack_id = %d", lr1_mac_obj->stack_id );
                }
            }
            else
            {
                DBG_PRINT_WITH_LINE( "RX2 Timeout for stack_id = %d", lr1_mac_obj->stack_id );
            }
            lr1mac_mac_update( lr1_mac_obj );
        }
        break;
#else
        // case LWPSTATE_RX2:
        if( lr1_mac_obj->radio_process_state == RADIOSTATE_RX_FINISHED )
        {
            bool has_receive_valid_packet = false;
            if( lr1_mac_obj->rp_planner_status == RP_STATUS_RX_PACKET )
            {
                lr1_mac_obj->rx_down_data.rx_metadata.rx_window = RECEIVE_ON_RX2;
                lr1_mac_obj->valid_rx_packet                    = lr1_stack_mac_rx_frame_decode( lr1_mac_obj );
                if( lr1_mac_obj->valid_rx_packet == NO_MORE_VALID_RX_PACKET )
                {
                    DBG_PRINT_WITH_LINE( "Receive a bad packet on RX2 for stack_id = %d", lr1_mac_obj->stack_id );
                }
                else
                {
                    has_receive_valid_packet = true;
                    DBG_PRINT_WITH_LINE( "Receive a Valid downlink RX2 for stack_id = %d", lr1_mac_obj->stack_id );
                }
            }
            else
            {
                DBG_PRINT_WITH_LINE( "RX2 Timeout for stack_id = %d", lr1_mac_obj->stack_id );
            }

            if( ( has_receive_valid_packet == false ) && ( smtc_relay_tx_is_enable( lr1_mac_obj->stack_id ) == true ) )
            {
                lr1_mac_obj->lr1mac_state = LWPSTATE_RXR;
                timer_in_past             = lr1_stack_mac_rx_timer_configure( lr1_mac_obj, RXR );
            }
            else
            {
                lr1mac_mac_update( lr1_mac_obj );
            }
        }

        if( timer_in_past == false )
        {
            break;
        }
        else
        {
            timer_in_past = false;
        }
        // Intentional fallthrough

        //**********************************************************************************
        //                                   STATE RXR
        //**********************************************************************************
    case LWPSTATE_RXR:
        if( lr1_mac_obj->radio_process_state == RADIOSTATE_RX_FINISHED )
        {
            if( lr1_mac_obj->rp_planner_status == RP_STATUS_RX_PACKET )
            {
                lr1_mac_obj->rx_down_data.rx_metadata.rx_window = RECEIVE_ON_RXR;
                lr1_mac_obj->valid_rx_packet                    = lr1_stack_mac_rx_frame_decode( lr1_mac_obj );
                if( lr1_mac_obj->valid_rx_packet == NO_MORE_VALID_RX_PACKET )
                {
                    DBG_PRINT_WITH_LINE( "Receive a bad packet on RXR for Hook Id = %d", myhook_id );
                }
                else
                {
                    DBG_PRINT_WITH_LINE( "Receive a Valid downlink RXR for Hook Id = %d", myhook_id );
                }
            }
            else
            {
                DBG_PRINT_WITH_LINE( "RXR Timeout for Hook Id = %d", myhook_id );
            }
            lr1mac_mac_update( lr1_mac_obj );
        }
        break;
#endif

    default:
        SMTC_MODEM_HAL_PANIC( "Illegal state in lorawan process\n" );
        break;
    }

    return ( lr1_mac_obj->lr1mac_state );
}

/***********************************************************************************************/
/*    End Of LoraWanProcess Method                                                             */
/***********************************************************************************************/

/**************************************************/
/*            LoraWan  Join  Method               */
/**************************************************/
status_lorawan_t lr1mac_core_join( lr1_stack_mac_t* lr1_mac_obj, uint32_t target_time_ms )
{
    if( lr1_mac_obj->lr1mac_state != LWPSTATE_IDLE )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "LP STATE NOT EQUAL TO IDLE\n" );
        return ERRORLORAWAN;
    }
#if !defined( TEST_BYPASS_JOIN_DUTY_CYCLE )
    if( lr1_mac_obj->is_join_duty_cycle_backoff_bypass_enabled == false )
    {
        if( ( int32_t ) ( lr1_mac_obj->next_time_to_join_seconds - smtc_modem_hal_get_time_in_s( ) ) > 0 )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "TOO SOON TO JOIN time is  %d time target is : %d\n",
                                         smtc_modem_hal_get_time_in_s( ), lr1_mac_obj->next_time_to_join_seconds );
            return ERRORLORAWAN;
        }
    }
#endif
    if( lr1_mac_obj->activation_mode == ACTIVATION_MODE_ABP )
    {
        lr1_mac_obj->join_status = JOINED;
        return OKLORAWAN;
    }
    uint32_t current_timestamp       = smtc_modem_hal_get_time_in_s( );
    lr1_mac_obj->timestamp_failsafe  = current_timestamp;
    lr1_mac_obj->rtc_target_timer_ms = target_time_ms;
    lr1_mac_obj->join_status         = JOINING;

    //    lr1_stack_mac_region_config( lr1_mac_obj );

    // check adr mode to see if it is already set in join DR and it is the first join try or if adr was set to other
    // DR
    //   if( ( lr1_mac_obj->adr_mode_select != JOIN_DR_DISTRIBUTION ) ||
    //       ( ( lr1_mac_obj->retry_join_cpt == 0 ) && ( lr1_mac_obj->adr_mode_select == JOIN_DR_DISTRIBUTION ) ) )
    //   {
    // if it is the case force dr distribution to join
    //       lr1_mac_obj->adr_mode_select_tmp = lr1_mac_obj->adr_mode_select;
    //       smtc_real_set_dr_distribution( lr1_mac_obj->real, JOIN_DR_DISTRIBUTION, &lr1_mac_obj->nb_trans );
    //       lr1_mac_obj->adr_mode_select = JOIN_DR_DISTRIBUTION;
    //   }
    //   smtc_real_get_next_tx_dr( lr1_mac_obj->real, lr1_mac_obj->join_status, &lr1_mac_obj->adr_mode_select,
    //                            &lr1_mac_obj->tx_data_rate, lr1_mac_obj->tx_data_rate_adr, &lr1_mac_obj->adr_enable );
    // if( smtc_real_get_join_next_channel( lr1_mac_obj->real, &lr1_mac_obj->tx_data_rate, &lr1_mac_obj->tx_frequency,
    //                                      &lr1_mac_obj->rx1_frequency, &lr1_mac_obj->rx2_frequency,
    //                                      &lr1_mac_obj->nb_available_tx_channel ) != OKLORAWAN )
    // {
    //     return ERRORLORAWAN;
    // }

    lr1_stack_mac_join_request_build( lr1_mac_obj );
    lr1_mac_obj->rx1_delay_s   = smtc_real_get_rx1_join_delay( lr1_mac_obj->real );
    lr1_mac_obj->rx2_data_rate = smtc_real_get_rx2_join_dr( lr1_mac_obj->real );

    // check if it first join try
    if( lr1_mac_obj->retry_join_cpt == 0 )
    {
        // take the timestamp reference for join duty cycle management
        lr1_mac_obj->first_join_timestamp = current_timestamp;
    }

    lr1_mac_obj->lr1mac_state = LWPSTATE_SEND;
    return OKLORAWAN;
}

/**************************************************/
/*          LoraWan  IsJoined  Method             */
/**************************************************/
join_status_t lr1_mac_joined_status_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->join_status;
}

/**************************************************/
/*          LoraWan  ClearJoinStatus  Method      */
/**************************************************/
void lr1mac_core_join_status_clear( lr1_stack_mac_t* lr1_mac_obj )
{
    lr1_mac_obj->join_status = NOT_JOINED;
    lr1mac_core_abort( lr1_mac_obj );
    smtc_real_init_join_snapshot_channel_mask( lr1_mac_obj->real );
    lr1_mac_obj->retry_join_cpt = 0;

    // Revert ADR modem in case the leave network command is called before join success
    lr1_mac_obj->adr_mode_select = lr1_mac_obj->adr_mode_select_tmp;
}

/**************************************************/
/*         LoraWan  SendPayload  Method           */
/**************************************************/
status_lorawan_t lr1mac_core_send_stack_cid_req( lr1_stack_mac_t* lr1_mac_obj, uint8_t* cid_req_list,
                                                 uint8_t cid_req_list_size, uint32_t target_time_ms )
{
    uint8_t data_in[242];
    uint8_t size_in = 0;

    for( uint8_t i = 0; i < cid_req_list_size; i++ )
    {
        // If sticky are present add it to the payload only once
        if( ( lr1_mac_obj->link_check_user_req != USER_MAC_REQ_REQUESTED ) &&
            ( lr1_mac_obj->device_time_user_req != USER_MAC_REQ_REQUESTED ) &&
            ( lr1_mac_obj->ping_slot_info_user_req != USER_MAC_REQ_REQUESTED ) )

        {
            if( lr1_mac_obj->tx_fopts_current_length > 0 )
            {
                size_in += lr1_mac_obj->tx_fopts_current_length;
                memcpy( data_in, lr1_mac_obj->tx_fopts_current_data, lr1_mac_obj->tx_fopts_current_length );
            }
        }

        switch( cid_req_list[i] )
        {
        case LINK_CHECK_REQ:
            if( lr1_mac_obj->link_check_user_req != USER_MAC_REQ_REQUESTED )
            {
                lr1_mac_obj->link_check_user_req = USER_MAC_REQ_REQUESTED;
                lr1_mac_obj->link_check_margin   = 0;
                lr1_mac_obj->link_check_gw_cnt   = 0;
                data_in[size_in++]               = cid_req_list[i];
            }
            break;

        case DEVICE_TIME_REQ:
            if( lr1_mac_obj->device_time_user_req != USER_MAC_REQ_REQUESTED )
            {
                lr1_mac_obj->device_time_user_req = USER_MAC_REQ_REQUESTED;
                data_in[size_in++]                = cid_req_list[i];
            }
            break;

        case PING_SLOT_INFO_REQ:
            if( lr1_mac_obj->ping_slot_info_user_req != USER_MAC_REQ_REQUESTED )
            {
                lr1_mac_obj->ping_slot_info_user_req = USER_MAC_REQ_REQUESTED;
                data_in[size_in++]                   = cid_req_list[i];
                data_in[size_in++]                   = lr1_mac_obj->ping_slot_periodicity_req & 0x7;
                lr1_mac_obj->tx_class_b_bit          = 0;
            }
            break;

        default:
            return ERRORLORAWAN;
            break;
        }
    }

    return lr1mac_core_payload_send( lr1_mac_obj, PORTNWK, true, data_in, size_in, UNCONF_DATA_UP, target_time_ms );
}

status_lorawan_t lr1mac_core_payload_send_at_time( lr1_stack_mac_t* lr1_mac_obj, uint8_t fport, bool fport_enabled,
                                                   const uint8_t* data_in, uint8_t size_in,
                                                   lr1mac_layer_param_t packet_type, uint32_t target_time_ms )
{
    status_lorawan_t status =
        lr1mac_core_payload_send( lr1_mac_obj, fport, fport_enabled, data_in, size_in, packet_type, target_time_ms );

    if( status == OKLORAWAN )
    {
        lr1_mac_obj->send_at_time = true;
        lr1_mac_obj->nb_trans_cpt = 1;  // Overwrite nb_trans_cpt, when downlink is At Time, repetitions are out dated
    }
    return status;
}

status_lorawan_t lr1mac_core_payload_send( lr1_stack_mac_t* lr1_mac_obj, uint8_t fport, bool fport_enabled,
                                           const uint8_t* data_in, uint8_t size_in, lr1mac_layer_param_t packet_type,
                                           uint32_t target_time_ms )
{
    if( fport_enabled == false )
    {
        size_in = 0;
    }

    status_lorawan_t status = smtc_real_is_payload_size_valid( lr1_mac_obj->real, lr1_mac_obj->tx_data_rate, size_in,
                                                               UP_LINK, lr1_mac_obj->tx_fopts_current_length );
    if( status == ERRORLORAWAN )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "PAYLOAD SIZE TOO HIGH\n" );
        return ERRORLORAWAN;
    }
    if( ( packet_type != CONF_DATA_UP ) && ( packet_type != UNCONF_DATA_UP ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "PAYLOAD PACKET TYPE INVALID\n" );
        return ERRORLORAWAN;
    }
    if( lr1_mac_obj->activation_mode == ACTIVATION_MODE_OTAA )
    {
        if( lr1_mac_obj->join_status != JOINED )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "OTAA DEVICE NOT JOINED YET\n" );
            return ERRORLORAWAN;
        }
    }
    if( lr1_mac_obj->lr1mac_state != LWPSTATE_IDLE )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "LP STATE NOT EQUAL TO IDLE\n" );
        return ERRORLORAWAN;
    }
    // // Decrement duty cycle before check the available DTC
    // if( smtc_real_get_next_channel( lr1_mac_obj->real, lr1_mac_obj->tx_data_rate, &lr1_mac_obj->tx_frequency,
    //                                 &lr1_mac_obj->rx1_frequency, &lr1_mac_obj->nb_available_tx_channel ) != OKLORAWAN
    //                                 )
    // {
    //     return ERRORLORAWAN;
    // }

    if( lr1_stack_network_next_free_duty_cycle_ms_get( lr1_mac_obj ) > 0 )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Network Duty Cycle toff\n" );
        return ERRORLORAWAN;
    }

    lr1_mac_obj->timestamp_failsafe  = smtc_modem_hal_get_time_in_s( );
    lr1_mac_obj->rtc_target_timer_ms = target_time_ms;
    lr1_mac_obj->tx_fport            = fport;
    lr1_mac_obj->tx_fport_present    = fport_enabled;
    lr1_mac_obj->tx_mtype            = packet_type;
    // Because Network payload are sent unconfirmed, we have to keep the Rx ACK bit for the App layer
    lr1_mac_obj->rx_down_data.rx_metadata.rx_ack_bit = false;

    // copy payload after the update of the lr1mac object
    if( data_in != NULL )
    {
        lr1_mac_obj->app_payload_size = size_in;
        copy_user_payload( lr1_mac_obj, data_in, size_in );
    }
    else
    {
        lr1_mac_obj->app_payload_size = 0;
    }

    lr1_stack_mac_tx_frame_build( lr1_mac_obj );
    lr1_stack_mac_tx_frame_encrypt( lr1_mac_obj );

    lr1_mac_obj->rx_down_data.rx_metadata.rx_window        = RECEIVE_NONE;
    lr1_mac_obj->rx_down_data.rx_metadata.rx_fport         = 0;
    lr1_mac_obj->rx_down_data.rx_metadata.rx_fport_present = false;
    lr1_mac_obj->nb_trans_cpt                              = lr1_mac_obj->nb_trans;
    lr1_mac_obj->lr1mac_state                              = LWPSTATE_SEND;

    return OKLORAWAN;
}

/**************************************************/
/*       LoraWan  AdrModeSelect  Method           */
/**************************************************/

status_lorawan_t lr1mac_core_dr_strategy_set( lr1_stack_mac_t* lr1_mac_obj, dr_strategy_t adr_mode_select )
{
    status_lorawan_t status;
    dr_strategy_t    adr_mode_select_cpy = lr1_mac_obj->adr_mode_select;
    lr1_mac_obj->adr_mode_select         = adr_mode_select;

    if( adr_mode_select == STATIC_ADR_MODE )
    {
        lr1_mac_obj->tx_data_rate_adr = smtc_real_get_min_tx_channel_dr( lr1_mac_obj->real );
    }
    else
    {
        lr1_mac_obj->tx_power = smtc_real_get_default_max_eirp( lr1_mac_obj->real );
    }

    smtc_real_set_dr_distribution( lr1_mac_obj->real, adr_mode_select, &lr1_mac_obj->nb_trans );
    status =
        smtc_real_get_next_tx_dr( lr1_mac_obj->real, lr1_mac_obj->join_status, &lr1_mac_obj->adr_mode_select,
                                  &lr1_mac_obj->tx_data_rate, lr1_mac_obj->tx_data_rate_adr, &lr1_mac_obj->adr_enable );

    if( status == ERRORLORAWAN )  // new adr profile no compatible with channel mask, retrieve old adr profile
    {
        lr1_mac_obj->adr_mode_select = adr_mode_select_cpy;
        smtc_real_set_dr_distribution( lr1_mac_obj->real, adr_mode_select_cpy, &lr1_mac_obj->nb_trans );
        smtc_real_get_next_tx_dr( lr1_mac_obj->real, lr1_mac_obj->join_status, &lr1_mac_obj->adr_mode_select,
                                  &lr1_mac_obj->tx_data_rate, lr1_mac_obj->tx_data_rate_adr, &lr1_mac_obj->adr_enable );
    }
    return ( status );
}
/**************************************************/
/*       LoraWan  AdrModeSelect  Get Method       */
/**************************************************/
dr_strategy_t lr1mac_core_dr_strategy_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( lr1_mac_obj->adr_mode_select );
}

/**************************************************/
/*         LoraWan  GetDevAddr  Method            */
/**************************************************/
uint32_t lr1mac_core_devaddr_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( lr1_mac_obj->dev_addr );
}

status_lorawan_t lr1mac_core_devaddr_set( lr1_stack_mac_t* lr1_mac_obj, uint32_t dev_addr )
{
    if( lr1_mac_obj->activation_mode == ACTIVATION_MODE_ABP )
    {
        lr1_mac_obj->dev_addr = dev_addr;
        return OKLORAWAN;
    }
    return ERRORLORAWAN;
}

/**************************************************/
/*         LoraWan  GetNextPower  Method          */
/**************************************************/
uint8_t lr1mac_core_next_power_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( lr1_mac_obj->tx_power );
}

/**************************************************/
/*    LoraWan  GetLorawanProcessState  Method     */
/**************************************************/
lr1mac_states_t lr1mac_core_state_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( lr1_mac_obj->lr1mac_state );
}

/**************************************************/
/*    LoraWan  NVM                                */
/**************************************************/
void lr1mac_core_context_save( lr1_stack_mac_t* lr1_mac_obj )
{
    lr1_mac_nvm_context_t ctx = { 0 };

    smtc_modem_hal_context_restore( CONTEXT_LORAWAN_STACK, lr1_mac_obj->stack_id * sizeof( ctx ), ( uint8_t* ) &ctx,
                                    sizeof( ctx ) );

    if( ( ctx.devnonce != lr1_mac_obj->dev_nonce ) ||
        ( memcmp( ctx.join_nonce, lr1_mac_obj->join_nonce, sizeof( ctx.join_nonce ) ) != 0 ) ||
        ( ctx.certification_enabled != lr1_mac_obj->is_lorawan_modem_certification_enabled ) ||
        ( ctx.region != lr1_mac_obj->real->region_type ) )
    {
        ctx.ctx_version = LORAWAN_NVM_CTX_VERSION;
        ctx.devnonce    = lr1_mac_obj->dev_nonce;
        memcpy( ctx.join_nonce, lr1_mac_obj->join_nonce, sizeof( ctx.join_nonce ) );
        ctx.certification_enabled = lr1_mac_obj->is_lorawan_modem_certification_enabled;
        ctx.region                = lr1_mac_obj->real->region_type;
        ctx.crc                   = lr1mac_utilities_crc( ( uint8_t* ) &ctx, sizeof( ctx ) - sizeof( ctx.crc ) );

        smtc_modem_hal_context_store( CONTEXT_LORAWAN_STACK, lr1_mac_obj->stack_id * sizeof( ctx ), ( uint8_t* ) &ctx,
                                      sizeof( ctx ) );

        // dummy context reading to ensure context store is done before exiting the function
        lr1_mac_nvm_context_t dummy_context = { 0 };
        smtc_modem_hal_context_restore( CONTEXT_LORAWAN_STACK, lr1_mac_obj->stack_id * sizeof( ctx ),
                                        ( uint8_t* ) &dummy_context, sizeof( dummy_context ) );
    }
}

status_lorawan_t lr1mac_core_context_load( lr1_stack_mac_t* lr1_mac_obj )
{
    lr1_mac_nvm_context_t ctx = { 0 };
    smtc_modem_hal_context_restore( CONTEXT_LORAWAN_STACK, lr1_mac_obj->stack_id * sizeof( ctx ), ( uint8_t* ) &ctx,
                                    sizeof( ctx ) );

    if( lr1mac_utilities_crc( ( uint8_t* ) &ctx, sizeof( ctx ) - sizeof( ctx.crc ) ) == ctx.crc )
    {
        lr1_mac_obj->dev_nonce = ctx.devnonce;
        memcpy( lr1_mac_obj->join_nonce, ctx.join_nonce, sizeof( lr1_mac_obj->join_nonce ) );
        lr1_mac_obj->is_lorawan_modem_certification_enabled = ctx.certification_enabled;
        lr1_mac_obj->real->region_type                      = ctx.region;

        return OKLORAWAN;
    }
    else
    {  // == factory reset
        SMTC_MODEM_HAL_TRACE_WARNING( "No valid DevNonce in NVM, use default (0)\n" );
        return ERRORLORAWAN;
    }
}

void lr1mac_core_context_factory_reset( lr1_stack_mac_t* lr1_mac_obj )
{
    lr1_mac_nvm_context_t ctx = { 0 };
    ctx.ctx_version           = LORAWAN_NVM_CTX_VERSION;
    memset( ctx.join_nonce, 0xFF, sizeof( ctx.join_nonce ) );
    ctx.crc = lr1mac_utilities_crc( ( uint8_t* ) &ctx, sizeof( ctx ) - sizeof( ctx.crc ) );

    smtc_modem_hal_context_store( CONTEXT_LORAWAN_STACK, lr1_mac_obj->stack_id * sizeof( ctx ), ( uint8_t* ) &ctx,
                                  sizeof( ctx ) );

    // dummy context reading to ensure context store is done before exiting the function
    lr1_mac_nvm_context_t dummy_context = { 0 };
    smtc_modem_hal_context_restore( CONTEXT_LORAWAN_STACK, lr1_mac_obj->stack_id * sizeof( ctx ),
                                    ( uint8_t* ) &dummy_context, sizeof( dummy_context ) );
}

/**************************************************/
/*   LoraWan  lr1mac_core_next_max_payload_length_get  Method     */
/**************************************************/
uint32_t lr1mac_core_next_max_payload_length_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( smtc_real_get_max_payload_size( lr1_mac_obj->real, lr1_mac_obj->tx_data_rate, UP_LINK ) -
             lr1_mac_obj->tx_fopts_current_length - FHDROFFSET );
}

/**************************************************/
/*        LoraWan  lr1mac_core_next_dr_get  Method        */
/**************************************************/
uint8_t lr1mac_core_next_dr_get( lr1_stack_mac_t* lr1_mac_obj )
{  // note return datareate in case of adr
    return ( lr1_mac_obj->tx_data_rate );
}

uint32_t lr1mac_core_next_frequency_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( lr1_mac_obj->tx_frequency );
}

lr1mac_activation_mode_t lr1mac_core_get_activation_mode( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->activation_mode;
}

void lr1mac_core_set_activation_mode( lr1_stack_mac_t* lr1_mac_obj, lr1mac_activation_mode_t activation_mode )
{
    lr1_mac_obj->activation_mode = activation_mode;
}

uint16_t lr1mac_core_devnonce_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->dev_nonce;
}

uint32_t lr1mac_core_fcnt_up_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( lr1_mac_obj->fcnt_up );
}

uint32_t lr1mac_core_next_join_time_second_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( lr1_mac_obj->next_time_to_join_seconds );
}

smtc_real_region_types_t lr1mac_core_get_region( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->real->region_type;
}

status_lorawan_t lr1mac_core_set_region( lr1_stack_mac_t* lr1_mac_obj, smtc_real_region_types_t region_type )
{
    if( smtc_real_is_supported_region( region_type ) == SMTC_REAL_STATUS_OK )
    {
        lr1_stack_mac_region_init( lr1_mac_obj, region_type );
        lr1_stack_mac_region_config( lr1_mac_obj );
        lr1mac_core_context_save( lr1_mac_obj );

        return OKLORAWAN;
    }
    return ERRORLORAWAN;
}

void lr1mac_core_set_no_rx_packet_threshold( lr1_stack_mac_t* lr1_mac_obj, uint16_t no_rx_packet_reset_threshold )
{
    lr1_mac_obj->no_rx_packet_reset_threshold = no_rx_packet_reset_threshold;
}

uint16_t lr1mac_core_get_no_rx_packet_threshold( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->no_rx_packet_reset_threshold;
}

uint16_t lr1mac_core_get_current_adr_ack_cnt( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->adr_ack_cnt;
}

uint16_t lr1mac_core_get_current_no_rx_packet_in_mobile_mode( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->no_rx_packet_count_in_mobile_mode;
}

void lr1mac_core_reset_no_rx_packet_in_mobile_mode_cnt( lr1_stack_mac_t* lr1_mac_obj )
{
    lr1_mac_obj->no_rx_packet_count_in_mobile_mode = 0;
}

uint16_t lr1mac_core_get_current_no_rx_packet_cnt( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->no_rx_packet_count;
}

uint32_t lr1mac_core_get_current_no_rx_packet_cnt_since_s( lr1_stack_mac_t* lr1_mac_obj )
{
    return smtc_modem_hal_get_time_in_s( ) - lr1_mac_obj->no_rx_packet_since_s;
}

bool lr1mac_core_available_link_adr_get( lr1_stack_mac_t* lr1_mac_obj )
{
    if( lr1_mac_obj->available_link_adr == true )
    {
        lr1_mac_obj->available_link_adr = false;  // reset the internal flag when the function is called by upper layer
        return ( true );
    }

    return false;
}

bool lr1mac_core_join_duty_cycle_backoff_bypass_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->is_join_duty_cycle_backoff_bypass_enabled;
}

void lr1mac_core_join_duty_cycle_backoff_bypass_set( lr1_stack_mac_t* lr1_mac_obj, bool enable )
{
    lr1_mac_obj->is_join_duty_cycle_backoff_bypass_enabled = enable;
}

void lr1mac_core_certification_set( lr1_stack_mac_t* lr1_mac_obj, uint8_t enable )
{
    lr1_mac_obj->is_lorawan_modem_certification_enabled    = enable;
    lr1_mac_obj->is_join_duty_cycle_backoff_bypass_enabled = ( bool ) enable;
    lr1mac_core_context_save( lr1_mac_obj );
}

uint8_t lr1mac_core_certification_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->is_lorawan_modem_certification_enabled;
}

lr1mac_version_t lr1mac_core_get_lorawan_version( lr1_stack_mac_t* lr1_mac_obj )
{
    lr1mac_version_t version = { .major    = LORAWAN_VERSION_MAJOR,
                                 .minor    = LORAWAN_VERSION_MINOR,
                                 .patch    = LORAWAN_VERSION_PATCH,
                                 .revision = LORAWAN_VERSION_REVISION };
    return version;
}

bool lr1mac_core_convert_rtc_to_gps_epoch_time( lr1_stack_mac_t* lr1_mac_obj, uint32_t rtc_ms,
                                                uint32_t* seconds_since_epoch, uint32_t* fractional_second )
{
    uint32_t tmp_seconds_since_epoch;
    uint32_t tmp_fractional_second;

    uint32_t delta_tx_rx_ms = rtc_ms - lr1_mac_obj->timestamp_tx_done_device_time_req_ms;

    tmp_fractional_second = lr1_mac_obj->fractional_second + delta_tx_rx_ms;

    uint32_t tmp_s        = tmp_fractional_second / 1000;  // number of seconds without ms
    tmp_fractional_second = tmp_fractional_second - ( tmp_s * 1000 );

    tmp_seconds_since_epoch = lr1_mac_obj->seconds_since_epoch + tmp_s;

    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "tx: %u, rx:%u, diff:%u\n", lr1_mac_obj->timestamp_tx_done_device_time_req_ms,
                                       lr1_mac_obj->timestamp_last_device_time_ans_s, delta_tx_rx_ms );
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "DeviceTime GPS : %u.%u s\n", tmp_seconds_since_epoch, tmp_fractional_second );

    *seconds_since_epoch = tmp_seconds_since_epoch;
    *fractional_second   = tmp_fractional_second;

    return lr1mac_core_is_time_valid( lr1_mac_obj );
}

bool lr1mac_core_is_time_valid( lr1_stack_mac_t* lr1_mac_obj )
{
    uint32_t rtc_s = smtc_modem_hal_get_time_in_s( );

    if( ( lr1_mac_obj->timestamp_last_device_time_ans_s != 0 ) &&
        ( ( int32_t ) ( rtc_s - lr1_mac_obj->timestamp_last_device_time_ans_s -
                        lr1_mac_obj->device_time_invalid_delay_s ) < 0 ) )
    {
        return true;
    }
    return false;
}

uint32_t lr1mac_core_get_timestamp_last_device_time_ans_s( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->timestamp_last_device_time_ans_s;
}

uint32_t lr1mac_core_get_time_left_connection_lost( lr1_stack_mac_t* lr1_mac_obj )
{
    uint32_t rtc_s                        = smtc_modem_hal_get_time_in_s( );
    uint32_t time_since_last_correction_s = 0;
    uint32_t time_left_connection_lost    = lr1_mac_obj->device_time_invalid_delay_s;

    if( lr1_mac_obj->timestamp_last_device_time_ans_s > 0 )
    {
        time_since_last_correction_s = rtc_s - lr1_mac_obj->timestamp_last_device_time_ans_s;
        if( time_since_last_correction_s <= lr1_mac_obj->device_time_invalid_delay_s )
        {
            time_left_connection_lost = lr1_mac_obj->device_time_invalid_delay_s - time_since_last_correction_s;
            // manage the wrapping case shouldn't occur if it occur launch a sync in 1h
            if( time_left_connection_lost > lr1_mac_obj->device_time_invalid_delay_s )
            {
                time_left_connection_lost = 3600;  // todo
            }
        }
    }

    return ( time_left_connection_lost );
}

status_lorawan_t lr1_mac_core_set_device_time_invalid_delay_s( lr1_stack_mac_t* lr1_mac_obj, uint32_t delay_s )
{
    if( delay_s > LR1MAC_DEVICE_TIME_DELAY_TO_BE_NO_SYNC )
    {
        return ERRORLORAWAN;
    }
    lr1_mac_obj->device_time_invalid_delay_s = delay_s;
    return OKLORAWAN;
}

uint32_t lr1_mac_core_get_device_time_invalid_delay_s( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->device_time_invalid_delay_s;
}

status_lorawan_t lr1mac_core_set_ping_slot_periodicity( lr1_stack_mac_t* lr1_mac_obj, uint8_t ping_slot_periodicity )
{
    if( ping_slot_periodicity > SMTC_REAL_PING_SLOT_PERIODICITY_DEFAULT )
    {
        return ERRORLORAWAN;
    }

    // The periodicity must not be modified when the MAC command is currently requested
    if( ( lr1_mac_obj->ping_slot_info_user_req == USER_MAC_REQ_REQUESTED ) ||
        ( lr1_mac_obj->ping_slot_info_user_req == USER_MAC_REQ_SENT ) )
    {
        return ERRORLORAWAN;
    }

    lr1_mac_obj->ping_slot_info_user_req   = USER_MAC_REQ_NOT_REQUESTED;
    lr1_mac_obj->ping_slot_periodicity_req = ping_slot_periodicity;
    return OKLORAWAN;
}

uint8_t lr1mac_core_get_ping_slot_periodicity( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->ping_slot_periodicity_req;
}

bool lr1mac_core_get_class_b_status( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->tx_class_b_bit;
}

status_lorawan_t lr1_mac_core_get_link_check_ans( lr1_stack_mac_t* lr1_mac_obj, uint8_t* margin, uint8_t* gw_cnt )
{
    status_lorawan_t ret = OKLORAWAN;

    if( lr1_mac_obj->link_check_user_req == USER_MAC_REQ_ACKED )
    {
        *margin = lr1_mac_obj->link_check_margin;
        *gw_cnt = lr1_mac_obj->link_check_gw_cnt;
    }
    else
    {
        *margin = 0;
        *gw_cnt = 0;
        ret     = ERRORLORAWAN;
    }

    return ret;
}

status_lorawan_t lr1_mac_core_get_device_time_req_status( lr1_stack_mac_t* lr1_mac_obj )
{
    if( lr1_mac_obj->device_time_user_req == USER_MAC_REQ_ACKED )
    {
        return OKLORAWAN;
    }
    return ERRORLORAWAN;
}

status_lorawan_t lr1_mac_core_get_ping_slot_info_req_status( lr1_stack_mac_t* lr1_mac_obj )
{
    if( lr1_mac_obj->ping_slot_info_user_req == USER_MAC_REQ_ACKED )
    {
        return OKLORAWAN;
    }
    return ERRORLORAWAN;
}

status_lorawan_t lr1mac_core_set_adr_ack_limit_delay( lr1_stack_mac_t* lr1_mac_obj, uint8_t adr_ack_limit,
                                                      uint8_t adr_ack_delay )
{
    if( ( adr_ack_limit > 1 ) && ( adr_ack_limit < 128 ) && ( adr_ack_delay > 1 ) && ( adr_ack_delay < 128 ) )
    {
        lr1_mac_obj->adr_ack_limit_init = adr_ack_limit;
        lr1_mac_obj->adr_ack_delay_init = adr_ack_delay;

        lr1_mac_obj->adr_ack_cnt = 0;
        return OKLORAWAN;
    }

    return ERRORLORAWAN;
}

void lr1mac_core_get_adr_ack_limit_delay( lr1_stack_mac_t* lr1_mac_obj, uint8_t* adr_ack_limit, uint8_t* adr_ack_delay )
{
    *adr_ack_limit = lr1_mac_obj->adr_ack_limit_init;
    *adr_ack_delay = lr1_mac_obj->adr_ack_delay_init;
}

void lr1mac_core_set_no_rx_windows( lr1_stack_mac_t* lr1_mac_obj, uint8_t disable_rx_windows )
{
    lr1_mac_obj->no_rx_windows = disable_rx_windows;
}

uint8_t lr1mac_core_get_no_rx_windows( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->no_rx_windows;
}
status_lorawan_t lr1mac_core_update_join_channel( lr1_stack_mac_t* lr1_mac_obj )
{
    lr1_stack_mac_region_config( lr1_mac_obj );

    if( ( lr1_mac_obj->adr_mode_select != JOIN_DR_DISTRIBUTION ) &&
        ( lr1_mac_obj->adr_mode_select != JOIN_DR_DISTRIBUTION_LONG_TERM ) )
    {
        // if it is the case force dr distribution to join
        lr1_mac_obj->adr_mode_select_tmp = lr1_mac_obj->adr_mode_select;
        smtc_real_set_dr_distribution( lr1_mac_obj->real, JOIN_DR_DISTRIBUTION, &lr1_mac_obj->nb_trans );
        lr1_mac_obj->adr_mode_select = JOIN_DR_DISTRIBUTION;
    }

    if( lr1_mac_obj->adr_mode_select == JOIN_DR_DISTRIBUTION_LONG_TERM )
    {
        smtc_real_set_dr_distribution( lr1_mac_obj->real, JOIN_DR_DISTRIBUTION_LONG_TERM, &lr1_mac_obj->nb_trans );
    }

    smtc_real_get_next_tx_dr( lr1_mac_obj->real, lr1_mac_obj->join_status, &lr1_mac_obj->adr_mode_select,
                              &lr1_mac_obj->tx_data_rate, lr1_mac_obj->tx_data_rate_adr, &lr1_mac_obj->adr_enable );
    return ( smtc_real_get_join_next_channel(
        lr1_mac_obj->real, &( lr1_mac_obj->tx_data_rate ), &( lr1_mac_obj->tx_frequency ),
        &( lr1_mac_obj->rx1_frequency ), &( lr1_mac_obj->rx2_frequency ), &( lr1_mac_obj->nb_available_tx_channel ) ) );
}
status_lorawan_t lr1mac_core_update_next_tx_channel( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( smtc_real_get_next_channel( lr1_mac_obj->real, lr1_mac_obj->tx_data_rate, &( lr1_mac_obj->tx_frequency ),
                                         &( lr1_mac_obj->rx1_frequency ), &( lr1_mac_obj->nb_available_tx_channel ) ) );
}
uint32_t lr1mac_core_get_time_of_nwk_ans( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( lr1_mac_obj->rtc_target_timer_ms );
}
void lr1mac_core_set_time_of_nwk_ans( lr1_stack_mac_t* lr1_mac_obj, uint32_t target_time )
{
    lr1_mac_obj->rtc_target_timer_ms = target_time;
}

void lr1mac_core_set_next_tx_at_time( lr1_stack_mac_t* lr1_mac_obj, bool is_send_at_time )
{
    lr1_mac_obj->send_at_time = is_send_at_time;
}
void lr1mac_core_set_join_status( lr1_stack_mac_t* lr1_mac_obj, join_status_t join_status )
{
    lr1_mac_obj->join_status = join_status;
}
/*
 *-----------------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITIONS ------------------------------------------------
 */

static void copy_user_payload( lr1_stack_mac_t* lr1_mac_obj, const uint8_t* data_in, const uint8_t size_in )
{
    uint8_t tx_fopts_length = 0;
    if( lr1_mac_obj->tx_fport != PORTNWK )
    {
        tx_fopts_length = lr1_mac_obj->tx_fopts_current_length;
    }

    memcpy( &( lr1_mac_obj->tx_payload[FHDROFFSET + lr1_mac_obj->tx_fport_present + tx_fopts_length] ), data_in,
            size_in );
}

void lr1mac_core_abort( lr1_stack_mac_t* lr1_mac_obj )
{
    lr1_mac_obj->valid_rx_packet     = NO_MORE_VALID_RX_PACKET;
    lr1_mac_obj->type_of_ans_to_send = NOFRAME_TOSEND;
    memset( &lr1_mac_obj->rx_down_data, 0, sizeof( lr1_mac_obj->rx_down_data ) );
    lr1_mac_obj->rx_down_data.stack_id = lr1_mac_obj->stack_id;
    lr1_mac_obj->nb_trans_cpt          = 1;
    lr1_mac_obj->lr1mac_state          = LWPSTATE_IDLE;
    rp_task_abort( lr1_mac_obj->rp, lr1_mac_obj->stack_id4rp );
}

static void lr1mac_mac_update( lr1_stack_mac_t* lr1_mac_obj )
{
    lr1_mac_obj->radio_process_state = RADIOSTATE_IDLE;

    lr1_mac_obj->rx_down_data.rx_metadata.is_a_join_accept = false;

    if( lr1_mac_obj->valid_rx_packet == JOIN_ACCEPT_PACKET )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " update join procedure\n" );
        if( lr1_stack_mac_join_accept( lr1_mac_obj ) == OKLORAWAN )
        {
            lr1_mac_obj->rx_down_data.rx_payload_size              = 0;
            lr1_mac_obj->rx_down_data.rx_metadata.is_a_join_accept = true;
            lr1_mac_obj->rx_down_data.rx_metadata.rx_fport_present = false;

            //@note because datarate Distribution has been changed during join
            lr1_mac_obj->adr_mode_select = lr1_mac_obj->adr_mode_select_tmp;
            // copy the join data rate to the default data_rate_adr => the first uplink will be transmit with the join
            // dr (ease GW one chanel)
            lr1_mac_obj->tx_data_rate_adr = lr1_mac_obj->tx_data_rate;
            smtc_real_set_dr_distribution( lr1_mac_obj->real, lr1_mac_obj->adr_mode_select_tmp,
                                           &lr1_mac_obj->nb_trans );
            lr1mac_core_context_save( lr1_mac_obj );
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "Not Joined\n" );
        }
    }

    if( ( lr1_mac_obj->valid_rx_packet == NWKRXPACKET ) || ( lr1_mac_obj->valid_rx_packet == USERRX_FOPTSPACKET ) )
    {
        lr1_stack_mac_cmd_parse( lr1_mac_obj );
    }

    lr1_stack_mac_update( lr1_mac_obj );

    /// If those MAC commands are not acked, set as not requested ///
    if( lr1_mac_obj->link_check_user_req == USER_MAC_REQ_SENT )
    {
        lr1_mac_obj->link_check_user_req = USER_MAC_REQ_NOT_REQUESTED;
    }

    if( lr1_mac_obj->device_time_user_req == USER_MAC_REQ_SENT )
    {
        lr1_mac_obj->device_time_user_req = USER_MAC_REQ_NOT_REQUESTED;
    }

    if( lr1_mac_obj->ping_slot_info_user_req == USER_MAC_REQ_SENT )
    {
        lr1_mac_obj->ping_slot_info_user_req = USER_MAC_REQ_NOT_REQUESTED;
    }
    /////////////////////////////////////////////////////////////////

    // Reset all data, not a valid packet or no downlink received, the upper layer is notified with an empty
    // stucture
    if( lr1_mac_obj->valid_rx_packet == NO_MORE_VALID_RX_PACKET )
    {
        memset( &lr1_mac_obj->rx_down_data, 0, sizeof( lr1_mac_obj->rx_down_data ) );
        lr1_mac_obj->rx_down_data.stack_id = lr1_mac_obj->stack_id;
    }

    lr1_mac_obj->push_callback( lr1_mac_obj->push_context );

    if( ( lr1_mac_obj->type_of_ans_to_send == NWKFRAME_TOSEND ) ||
        ( lr1_mac_obj->type_of_ans_to_send == USRFRAME_TORETRANSMIT ) )
    {  // @note ack send during the next tx|| ( packet.IsFrameToSend == USERACK_TOSEND ) ) {
        if( smtc_real_get_next_channel( lr1_mac_obj->real, lr1_mac_obj->tx_data_rate, &lr1_mac_obj->tx_frequency,
                                        &lr1_mac_obj->rx1_frequency,
                                        &lr1_mac_obj->nb_available_tx_channel ) != OKLORAWAN )
        {
            lr1_mac_obj->lr1mac_state = LWPSTATE_IDLE;
            if( lr1_mac_obj->type_of_ans_to_send == USRFRAME_TORETRANSMIT )
            {
                lr1_mac_obj->fcnt_up++;
                lr1_mac_obj->adr_ack_cnt++;  // increment adr counter each new uplink frame
            }
        }
        else
        {
            lr1_mac_obj->type_of_ans_to_send = NOFRAME_TOSEND;
            lr1_mac_obj->rtc_target_timer_ms =
                smtc_modem_hal_get_time_in_ms( ) + smtc_modem_hal_get_random_nb_in_range( 1000, 3000 );
            lr1_mac_obj->lr1mac_state = LWPSTATE_TX_WAIT;
            // protected by tpm failsafe
            lr1_mac_obj->timestamp_failsafe = smtc_modem_hal_get_time_in_s( );
        }
    }
    else
    {
        lr1_mac_obj->lr1mac_state = LWPSTATE_IDLE;
    }
    lr1_mac_obj->valid_rx_packet = NO_MORE_VALID_RX_PACKET;
}

/* --- EOF ------------------------------------------------------------------ */
