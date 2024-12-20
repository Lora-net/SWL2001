/*!
 * \file      modem_tx_protocol_manager.c
 *
 * \brief     soft modem task scheduler
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

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include "modem_supervisor_light.h"
#include "lorawan_send_management.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "lorawan_certification.h"
#include "smtc_modem_hal.h"
#include "modem_core.h"
#include "lorawan_api.h"
#include "smtc_modem_api.h"
#include "smtc_modem_test_api.h"
#include "smtc_duty_cycle.h"
#include "modem_event_utilities.h"
#include "modem_tx_protocol_manager.h"
#include "lr1mac_defs.h"
#include "smtc_real.h"
#include "smtc_lbt.h"
#include "smtc_modem_test.h"
#include "smtc_lora_cad_bt.h"
#if defined( ADD_RELAY_TX )
#include "relay_tx_api.h"
#include "relay_tx_mac_parser.h"
#include "relay_def.h"
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
typedef enum tx_protocol_manager_state
{
#if defined( ADD_RELAY_TX )
    TPM_STATE_PREPARE_WOR,     //!<
    TPM_STATE_LBT_BEFORE_WOR,  //!<
#if defined( ADD_CSMA )
    TPM_STATE_CSMA_BEFORE_WOR,  //!<
#endif
    TPM_STATE_RELAY_TX,  //!<
#endif
    TPM_STATE_LBT,  //!<
#if defined( ADD_CSMA )
    TPM_STATE_CSMA,  //!<
#endif
    TPM_STATE_TX_LORA,      //!<
    TPM_STATE_NWK_TX_LORA,  //!<
    TPM_STATE_TEST_MODE,
    TPM_STATE_IDLE,  //!<
    TPM_NUMBER_OF_STATE,
} tx_protocol_manager_state_t;
#define ENABLE_DEBUG_TPM 0
#define MAX_LIST_LENGTH 10
#if( MODEM_HAL_DBG_TRACE )
#define LOG_MARGIN_DELAY 50  // Log duration could impact  (it is advise to disable LOG if LBT /CSMA  are enabled)
#else
#define LOG_MARGIN_DELAY 0  // Log duration could impact  (it is advise to disable LOG if LBT /CSMA  are enabled)
#endif
#define SLEEP_UNTIL_RADIO_INTERRUPT_MS 10000
#define READY_FOR_LR1MAC_TX 0
#define MAX_TRIAL_RELAY 5
/*
 *-----------------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------------
 */
#define STACK_ID_CURRENT_TASK task_manager.modem_task[task_manager.next_task_id].stack_id
#define VIRTUAL_TASK_ID task_manager.next_task_id - ( NUMBER_OF_TASKS * STACK_ID_CURRENT_TASK )
#ifndef MODEM_MIN_RANDOM_DELAY_MS

#if defined( ADD_RELAY_TX )
#define MODEM_MIN_RANDOM_DELAY_MS ( smtc_relay_tx_is_enable( current_tpm_stack_id ) == false ) ? 500 : 3000
#else
#define MODEM_MIN_RANDOM_DELAY_MS 500
#endif
#endif
#ifndef MODEM_MAX_RANDOM_DELAY_MS
#define MODEM_MAX_RANDOM_DELAY_MS 6000
#endif

#if defined( ADD_RELAY_TX )
#define MODEM_TASK_DELAY_MS                                                \
    ( ( ( request_type == TX_PROTOCOL_TRANSMIT_TEST_MODE ) ||              \
        ( ( request_type == TX_PROTOCOL_TRANSMIT_LORA_CERTIFICATION ) &&   \
          ( smtc_relay_tx_is_enable( current_tpm_stack_id ) == false ) ) ) \
          ? 0                                                              \
      : ( ( request_type == TX_PROTOCOL_TRANSMIT_LORA_CERTIFICATION ) &&   \
          ( smtc_relay_tx_is_enable( current_tpm_stack_id ) == true ) )    \
          ? 2000                                                           \
          : ( smtc_modem_hal_get_random_nb_in_range( MODEM_MIN_RANDOM_DELAY_MS, MODEM_MAX_RANDOM_DELAY_MS ) ) )
#else

#define MODEM_TASK_DELAY_MS                                           \
    ( ( ( request_type == TX_PROTOCOL_TRANSMIT_TEST_MODE ) ||         \
        ( request_type == TX_PROTOCOL_TRANSMIT_LORA_CERTIFICATION ) ) \
          ? 0                                                         \
          : ( smtc_modem_hal_get_random_nb_in_range( MODEM_MIN_RANDOM_DELAY_MS, MODEM_MAX_RANDOM_DELAY_MS ) ) )
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
#define NB_REQUEST_ACCEPTED 5
static struct
{
    tx_protocol_manager_tx_type_t current_tpm_request_type;
    tx_protocol_manager_state_t   current_tpm_state;

    tx_protocol_manager_state_t tpm_list_of_state_to_execute[MAX_LIST_LENGTH];
    bool                        current_tpm_transaction_is_a_retransmit;
    radio_planner_t*            current_tpm_rp_target;
    uint8_t                     current_tpm_fport;
    bool                        current_tpm_fport_enabled;
    uint8_t                     current_tpm_data[242];
    uint8_t                     current_tpm_data_len;
    lr1mac_layer_param_t        current_tpm_packet_type;
    uint32_t                    current_tpm_target_time_ms;
    uint32_t                    current_tpm_add_delay_ms;
    bool                        current_tpm_transmit_at_time;
    uint32_t                    current_tpm_target_transmit_at_time;
    uint8_t                     current_tpm_stack_id;
    uint32_t                    current_tpm_cpt_lbt_max_trial;
    uint8_t                     current_tpm_cpt_relay_max_trial;
    uint32_t                    current_tpm_target_time_csma_before_wor_ms;
    uint32_t                    current_tpm_target_time_lbt_before_wor_ms;
    bool                        current_tpm_transmit_is_aborted;
    uint32_t                    current_tpm_failsafe_time_init;

    tx_protocol_manager_tx_type_t next_tpm_request_type[NB_REQUEST_ACCEPTED];
    tx_protocol_manager_state_t   next_tpm_state[NB_REQUEST_ACCEPTED];
    uint8_t                       next_tpm_fport[NB_REQUEST_ACCEPTED];
    bool                          next_tpm_fport_enabled[NB_REQUEST_ACCEPTED];
    uint8_t*                      next_tpm_data[NB_REQUEST_ACCEPTED];
    uint8_t                       next_tpm_data_len[NB_REQUEST_ACCEPTED];
    lr1mac_layer_param_t          next_tpm_packet_type[NB_REQUEST_ACCEPTED];
    uint32_t                      next_tpm_target_time_ms[NB_REQUEST_ACCEPTED];
    uint8_t                       next_tpm_stack_id[NB_REQUEST_ACCEPTED];
    bool                          next_tpm_stand_alone_stack_request[NB_REQUEST_ACCEPTED];
    uint8_t                       next_tpm_pending_request;

} modem_tpm_context;

#define current_tpm_request_type modem_tpm_context.current_tpm_request_type
#define current_tpm_state modem_tpm_context.current_tpm_state
#define tpm_list_of_state_to_execute modem_tpm_context.tpm_list_of_state_to_execute
#define current_tpm_rp_target modem_tpm_context.current_tpm_rp_target
#define current_tpm_fport modem_tpm_context.current_tpm_fport
#define current_tpm_fport_enabled modem_tpm_context.current_tpm_fport_enabled
#define current_tpm_data modem_tpm_context.current_tpm_data
#define current_tpm_data_len modem_tpm_context.current_tpm_data_len
#define current_tpm_packet_type modem_tpm_context.current_tpm_packet_type
#define current_tpm_target_time_ms modem_tpm_context.current_tpm_target_time_ms
#define current_tpm_transmit_at_time modem_tpm_context.current_tpm_transmit_at_time
#define current_tpm_stack_id modem_tpm_context.current_tpm_stack_id
#define current_tpm_transaction_is_a_retransmit modem_tpm_context.current_tpm_transaction_is_a_retransmit
#define current_tpm_cpt_lbt_max_trial modem_tpm_context.current_tpm_cpt_lbt_max_trial
#define current_tpm_cpt_relay_max_trial modem_tpm_context.current_tpm_cpt_relay_max_trial
#define current_tpm_target_time_csma_before_wor_ms modem_tpm_context.current_tpm_target_time_csma_before_wor_ms
#define current_tpm_target_time_lbt_before_wor_ms modem_tpm_context.current_tpm_target_time_lbt_before_wor_ms
#define current_tpm_transmit_is_aborted modem_tpm_context.current_tpm_transmit_is_aborted
#define current_tpm_failsafe_time_init modem_tpm_context.current_tpm_failsafe_time_init
#define current_tpm_add_delay_ms modem_tpm_context.current_tpm_add_delay_ms
#define current_tpm_target_transmit_at_time modem_tpm_context.current_tpm_target_transmit_at_time
#define next_tpm_request_type modem_tpm_context.next_tpm_request_type
#define next_tpm_state modem_tpm_context.next_tpm_state
#define next_tpm_fport modem_tpm_context.next_tpm_fport
#define next_tpm_fport_enabled modem_tpm_context.next_tpm_fport_enabled
#define next_tpm_data modem_tpm_context.next_tpm_data
#define next_tpm_data_len modem_tpm_context.next_tpm_data_len
#define next_tpm_packet_type modem_tpm_context.next_tpm_packet_type
#define next_tpm_target_time_ms modem_tpm_context.next_tpm_target_time_ms
#define next_tpm_stack_id modem_tpm_context.next_tpm_stack_id
#define next_tpm_stand_alone_stack_request modem_tpm_context.next_tpm_stand_alone_stack_request
#define next_tpm_pending_request modem_tpm_context.next_tpm_pending_request

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void shift_left_tpm_list( void );
static void reset_tpm_list( void );
static void compute_tpm_list( void );

static status_lorawan_t manage_tx_lora_state( void );
static status_lorawan_t manage_tx_nwk_lora_state( void );
static status_lorawan_t manage_idle_state( void );
static status_lorawan_t manage_test_mode( void );
static status_lorawan_t manage_lbt_state( void );
static void             modem_tpm_radio_busy_lbt( void* context );
static void             modem_tpm_radio_free_lbt( void* context );
static void             modem_tpm_radio_abort_lbt( void* context );
#if defined( ADD_CSMA )
static status_lorawan_t manage_csma_state( void );
static void             modem_tpm_radio_busy_csma( void* context );
static void             modem_tpm_radio_free_csma( void* context );
static void             modem_tpm_radio_abort_csma( void* context );
static void             modem_tpm_radio_free_cad_keep_channel( void* context );
#endif
#if defined( ADD_RELAY_TX )
static status_lorawan_t manage_relay_tx_state( void );
static status_lorawan_t manage_lbt_before_wor_state( void );
#if defined( ADD_CSMA )
static status_lorawan_t manage_csma_before_wor_state( void );
#endif
static status_lorawan_t manage_prepare_wor_state( void );
static void             modem_tpm_radio_free_relay_tx( void* context );
static void             modem_tpm_radio_abort_relay_tx( void* context );
#endif
static status_lorawan_t modem_tx_protocol_manager_engine( void );
static void             tpm_debug_print( void );
static status_lorawan_t tpm_get_next_channel( void );
static void             tpm_abort( void );
static void             update_tpm_target_time( void );
static uint32_t         update_add_delay_ms( void );
static status_lorawan_t ( *launch_tpm_func[TPM_NUMBER_OF_STATE] )( void ) = {
    [TPM_STATE_TX_LORA]     = &manage_tx_lora_state,
    [TPM_STATE_NWK_TX_LORA] = &manage_tx_nwk_lora_state,
    [TPM_STATE_LBT]         = &manage_lbt_state,
    [TPM_STATE_IDLE]        = &manage_idle_state,
    [TPM_STATE_TEST_MODE]   = &manage_test_mode,
#if defined( ADD_CSMA )
    [TPM_STATE_CSMA] = &manage_csma_state,
#endif
#if defined( ADD_RELAY_TX )
    [TPM_STATE_RELAY_TX]       = &manage_relay_tx_state,
    [TPM_STATE_LBT_BEFORE_WOR] = &manage_lbt_before_wor_state,
#if defined( ADD_CSMA )
    [TPM_STATE_CSMA_BEFORE_WOR] = &manage_csma_before_wor_state,
#endif
    [TPM_STATE_PREPARE_WOR] = &manage_prepare_wor_state,
#endif

};
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
/**
 * @brief call by smtc_modem_init to init the tx protocol manager
 *
 * @param rp pointer to the radioplanner object
 */
void modem_tx_protocol_manager_init( radio_planner_t* rp )
{
    memset( &modem_tpm_context, 0, sizeof( modem_tpm_context ) );
    current_tpm_rp_target                   = rp;
    current_tpm_transaction_is_a_retransmit = false;
    current_tpm_transmit_at_time            = false;
    for( int i = 0; i < NUMBER_OF_STACKS; i++ )
    {
        smtc_lbt_init( smtc_lbt_get_obj( i ), current_tpm_rp_target, RP_HOOK_ID_LBT + i,
                       ( void ( * )( void* ) ) modem_tpm_radio_free_lbt, NULL,
                       ( void ( * )( void* ) ) modem_tpm_radio_busy_lbt, NULL,
                       ( void ( * )( void* ) ) modem_tpm_radio_abort_lbt, NULL );
        if( lorawan_api_stack_mac_get( i )->real->real_const.const_lbt_supported == true )
        {
            smtc_lbt_set_parameters( smtc_lbt_get_obj( lorawan_api_stack_mac_get( i )->stack_id ),
                                     smtc_real_get_lbt_duration_ms( lorawan_api_stack_mac_get( i )->real ),
                                     smtc_real_get_lbt_threshold_dbm( lorawan_api_stack_mac_get( i )->real ),
                                     smtc_real_get_lbt_bw_hz( lorawan_api_stack_mac_get( i )->real ) );
            smtc_lbt_set_state( smtc_lbt_get_obj( lorawan_api_stack_mac_get( i )->stack_id ), true );
        }

#if defined( ADD_CSMA )
        smtc_lora_cad_bt_init( smtc_cad_get_obj( i ), current_tpm_rp_target, RP_HOOK_ID_CAD + i,
                               ( void ( * )( void* ) ) modem_tpm_radio_free_csma, NULL,
                               ( void ( * )( void* ) ) modem_tpm_radio_busy_csma, NULL,
                               ( void ( * )( void* ) ) modem_tpm_radio_free_cad_keep_channel, NULL,
                               ( void ( * )( void* ) ) modem_tpm_radio_abort_csma, NULL );
#if defined( ENABLE_CSMA_BY_DEFAULT )
        smtc_lora_cad_bt_set_state( smtc_cad_get_obj( i ), true );
#endif
#endif
#if defined( ADD_RELAY_TX )
        smtc_relay_tx_init( i, current_tpm_rp_target, lorawan_api_stack_mac_get( i )->real,
                            ( void ( * )( void* ) ) modem_tpm_radio_free_relay_tx, NULL, NULL, NULL,
                            ( void ( * )( void* ) ) modem_tpm_radio_abort_relay_tx, NULL );

#endif
    }
    reset_tpm_list( );
}

/**
 * @brief call by modem_supervisor_engine to check if there is a current task manages by  tpm
 *        TPM can perform multiple tasks (LBT, CSMA, etc.) before actually launching the LoRaWAN transaction.
 *        This function returns SLEEP_UNTIL_RADIO_INTERRUPT_MS if there is still a task to execute before launching the
 * LR1MAC transaction.
 *
 * @param return a delay in ms. If the return value is equal to 0, it means that the TPM is free and ready to launch the
 * LoRaWAN Tx transaction.
 */
uint32_t tx_protocol_manager_is_busy( void )
{
    uint32_t time_to_sleep = READY_FOR_LR1MAC_TX;
    if( ( tpm_list_of_state_to_execute[0] == TPM_STATE_LBT ) ||
        ( tpm_list_of_state_to_execute[0] == TPM_STATE_TEST_MODE )
#if defined( ADD_CSMA )
        || ( tpm_list_of_state_to_execute[0] == TPM_STATE_CSMA )
#endif
#if defined( ADD_RELAY_TX )
        || ( tpm_list_of_state_to_execute[0] == TPM_STATE_LBT_BEFORE_WOR ) ||
#if defined( ADD_CSMA )
        ( tpm_list_of_state_to_execute[0] == TPM_STATE_CSMA_BEFORE_WOR ) ||
#endif
        ( tpm_list_of_state_to_execute[0] == TPM_STATE_RELAY_TX ) ||
        ( tpm_list_of_state_to_execute[0] == TPM_STATE_PREPARE_WOR )
#endif
    )
    {
        time_to_sleep = SLEEP_UNTIL_RADIO_INTERRUPT_MS;
        if( ( ( int32_t ) ( smtc_modem_hal_get_time_in_s( ) - current_tpm_failsafe_time_init - FAILSAFE_TPM_S ) > 0 ) )
        {
            tpm_abort( );
            time_to_sleep = 0;
        }
    }
    if( ( tpm_list_of_state_to_execute[0] == TPM_STATE_IDLE ) && ( next_tpm_pending_request > 0 ) &&
        ( next_tpm_pending_request <= NB_REQUEST_ACCEPTED ) )
    {
        next_tpm_pending_request--;
        if( next_tpm_stand_alone_stack_request[next_tpm_pending_request] == false )
        {
            tx_protocol_manager_request(
                next_tpm_request_type[next_tpm_pending_request], next_tpm_fport[next_tpm_pending_request],
                next_tpm_fport_enabled[next_tpm_pending_request], next_tpm_data[next_tpm_pending_request],
                next_tpm_data_len[next_tpm_pending_request], next_tpm_packet_type[next_tpm_pending_request],
                next_tpm_target_time_ms[next_tpm_pending_request], next_tpm_stack_id[next_tpm_pending_request] );
        }
        else
        {
            tx_protocol_manager_lr1mac_stand_alone_tx( );
        }
        time_to_sleep = 0;
    }

    return time_to_sleep;
}

/**
 * @brief Before starting any LoRaWAN transmission, any requester calls this function to perform post-actions
 * (such as LBT, CSMA, RELAY, etc.). "Requesters" can be any services (CID, Store & Forward, Join Manager, etc.)
 * or application layer components (Tx requests made by the application) that request a LoRaWAN transmission.
 * @remark After each request, this function launches compute_tpm_list to establish the type of preprocessing to
 * execute before actually launching the LoRaWAN transaction. It also calls tpm_get_next_channel to retrieve the
 * channel (frequency) on which the LoRaWAN transaction will be executed, and finally calls the
 * modem_tx_protocol_manager_engine to start the first preprocessing step if there is one, or directly launches
 * the LoRaWAN transaction.
 *
 * @param return
 */
status_lorawan_t tx_protocol_manager_request( tx_protocol_manager_tx_type_t request_type, uint8_t fport,
                                              bool fport_enabled, const uint8_t* data, uint8_t data_len,
                                              lr1mac_layer_param_t packet_type, uint32_t target_time_ms,
                                              uint8_t stack_id )
{
    status_lorawan_t status = ERRORLORAWAN;
    if( next_tpm_pending_request >= NB_REQUEST_ACCEPTED )
    {
        return ERRORLORAWAN;
    }
    if( tpm_list_of_state_to_execute[0] != TPM_STATE_IDLE )
    {
        next_tpm_request_type[next_tpm_pending_request]              = request_type;
        next_tpm_fport[next_tpm_pending_request]                     = fport;
        next_tpm_target_time_ms[next_tpm_pending_request]            = target_time_ms;
        next_tpm_fport_enabled[next_tpm_pending_request]             = fport_enabled;
        next_tpm_data[next_tpm_pending_request]                      = ( uint8_t* ) data;
        next_tpm_data_len[next_tpm_pending_request]                  = data_len;
        next_tpm_packet_type[next_tpm_pending_request]               = packet_type;
        next_tpm_stack_id[next_tpm_pending_request]                  = stack_id;
        next_tpm_stand_alone_stack_request[next_tpm_pending_request] = false;
        next_tpm_pending_request++;
        return OKLORAWAN;
    }

    current_tpm_failsafe_time_init          = smtc_modem_hal_get_time_in_s( );
    current_tpm_transaction_is_a_retransmit = false;
    current_tpm_transmit_is_aborted         = false;
    current_tpm_cpt_relay_max_trial         = 0;
    current_tpm_cpt_lbt_max_trial           = 0;
    current_tpm_add_delay_ms                = 0;
    current_tpm_target_transmit_at_time     = 0;
    if( request_type != TX_PROTOCOL_NONE )
    {
        current_tpm_request_type     = request_type;
        current_tpm_fport            = fport;
        current_tpm_transmit_at_time = false;
        current_tpm_fport_enabled    = fport_enabled;
        memcpy( &current_tpm_data[0], data, data_len );
        current_tpm_data_len    = data_len;
        current_tpm_packet_type = packet_type;
        current_tpm_stack_id    = stack_id;
        compute_tpm_list( );

        if( request_type == TX_PROTOCOL_TRANSMIT_LORA_AT_TIME )
        {
            current_tpm_target_transmit_at_time = target_time_ms;
            current_tpm_target_time_ms          = target_time_ms - update_add_delay_ms( );
            if( ( ( int32_t ) ( smtc_modem_hal_get_time_in_ms( ) - current_tpm_target_time_ms ) > 0 ) )
            {
                return ERRORLORAWAN;
            }
        }
        else
        {
            current_tpm_target_time_ms = target_time_ms + MODEM_TASK_DELAY_MS;
        }
        if( tpm_get_next_channel( ) != OKLORAWAN )
        {
            return ERRORLORAWAN;
        }
        status = modem_tx_protocol_manager_engine( );
    }
    return status;
}

/**
 * @brief this function is called by supervisor_run_lorawan_engine when a retransmission or a nwk frame is on going
 * in the LoRaWAN stack
 * @remark At this stage, the LR1MAC stack is running and has already scheduled a new transmission by itself,
 * typically to manage network frame answers. In this case, the TPM must once again manage each preprocessing stage
 * (LBT, CSMA, or relay work). It checks with the function tpm_get_next_channel if there is still an available
 * channel and restarts a full TPM transaction as if it had been initiated by the upper layer itself.
 * @param return
 */
void tx_protocol_manager_lr1mac_stand_alone_tx( void )
{
    if( next_tpm_pending_request >= NB_REQUEST_ACCEPTED )
    {
        return;
    }
    if( tpm_list_of_state_to_execute[0] != TPM_STATE_IDLE )
    {
        next_tpm_stand_alone_stack_request[next_tpm_pending_request] = true;
        return;
    }
    current_tpm_transaction_is_a_retransmit = true;
    current_tpm_cpt_relay_max_trial         = 0;
    current_tpm_cpt_lbt_max_trial           = 0;
    current_tpm_failsafe_time_init          = smtc_modem_hal_get_time_in_s( );
    current_tpm_target_time_ms              = lorawan_api_get_time_of_nwk_ans( current_tpm_stack_id );
    current_tpm_add_delay_ms                = 0;
    current_tpm_request_type                = TX_PROTOCOL_TRANSMIT_LORA;
    current_tpm_transmit_at_time            = false;
    if( tpm_get_next_channel( ) != OKLORAWAN )
    {
        reset_tpm_list( );
        current_tpm_transmit_is_aborted = true;
        lorawan_api_core_abort( current_tpm_stack_id );
    }
    else
    {
        compute_tpm_list( );
        modem_tx_protocol_manager_engine( );
    }
}

bool tx_protocol_manager_tx_is_aborted( void )
{
    if( current_tpm_transaction_is_a_retransmit == false )
    {
        return current_tpm_transmit_is_aborted;
    }
    else
    {
        return false;
    }
}
void tx_protocol_manager_abort( void )
{
    tpm_abort( );
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/**
 * @brief this function is called by supervisor_run_lorawan_engine when a retransmission or a nwk frame is on going
 * in the LoRaWAN stack
 * @remark
 * This function is called each time the TPM algorithm intends to progress in its state machine. In the array
 * tpm_list_of_state_to_execute, the different tasks to execute (such as LBT, CSMA, etc.) are stored in order. The first
 * task in this array is always the one proceeded by the TPM. After a task is finished, there are typically three
 * possibilities:
 *
 * 1) The task is fully completed successfully (for example, LBT sniff has found that the channel is
 * available). In this case, the state machine advances one step ahead by shifting this list of one element.
 *
 * 2) The task needs to be restarted (for example, LBT sniff detected a busy channel). In this case, the state machine
 * is put on hold, and the same task is relaunched, often with a new configuration (for example, testing a new channel).
 *
 * 3) The task is fully completed but unsuccessfully (for example, after many retries, the LBT sniff task has found that
 * all channels are unavailable). In this case, the state machine is fully reset, and the current transaction is
 * aborted. The upper layer will be informed by the event "Tx done" but with the status "Tx failed".
 *
 * @param return status_lorawan_t
 */

static status_lorawan_t modem_tx_protocol_manager_engine( void )
{
    status_lorawan_t status = ERRORLORAWAN;
    if( ENABLE_DEBUG_TPM == 1 )
    {
        tpm_debug_print( );
    }
    if( ( ( int32_t ) ( smtc_modem_hal_get_time_in_s( ) - current_tpm_failsafe_time_init - FAILSAFE_TPM_S ) > 0 ) )
    {
        tpm_abort( );
    }
    else
    {
        status = launch_tpm_func[tpm_list_of_state_to_execute[0]]( );
    }
    return status;
}
/*******************************************************************************************************************
 *In this section, every function that manages the LR1MAC Tx transaction itself is implemented. These functions are
 *called when the state in the list is TPM_STATE_TX_LORA or TPM_STATE_NWK_TX_LORA
 *******************************************************************************************************************/

/**
 * @brief this function is called when the state in the list is TPM_STATE_NWK_TX_LORA
 * @remark In this particular case, the LR1MAC stack is already running and waiting for an internal transmission (such
 * as a network answer, for example). The trick in this case is that we modify the starting time of this frame within
 * the LR1MAC stack itself. The stack has already set a target time to launch this answer, but because this time will
 * likely be delayed by the TPM, due to the fact that it could have launched some preprocessing tasks (LBT, CSMA), this
 * starting time is no longer valid. When we reach this state, every preprocessing task is now finished successfully,
 * and we are ready to launch the LR1MAC transmission. So, this function updates this time by calling the
 * lorawan_api_stack_mac_get function with the appropriate parameters.
 *
 * @param return status_lorawan_t
 */

static status_lorawan_t manage_tx_nwk_lora_state( void )
{
    shift_left_tpm_list( );
    if( current_tpm_transmit_at_time == true )
    {
        lorawan_api_set_next_tx_at_time( current_tpm_stack_id, true );
    }

    lorawan_api_set_time_of_nwk_ans( current_tpm_stack_id, current_tpm_target_time_ms );
    return OKLORAWAN;
}

/**
 * @brief this function is called when the state in the list is TPM_STATE_TX_LORA
 * @remark When we reach this state, every preprocessing task is now finished successfully, and we are ready to launch
 * the LR1MAC transmission. Depending on the type of task, it launches either a join transaction, a "normal" LoRaWAN Tx
 * transaction, or a specific "CID transmission" in the case of link check/ping slot or time request.
 *
 * If the relay mode is enabled, the transmission must be executed "at time" to respect the relay timing defined in the
 * protocol (50ms after the WoR ack). In this case, the send_at_time parameter available in the LR1MAC stack is directly
 * updated.
 *
 * If the transaction can be executed without any timing constraint, some randomization of the start time is added using
 * the macro MODEM_TASK_DELAY_MS.
 *
 * @param return status_lorawan_t
 */
static status_lorawan_t manage_tx_lora_state( void )
{
    shift_left_tpm_list( );
    status_lorawan_t status = ERRORLORAWAN;
    switch( current_tpm_request_type )
    {
    case TX_PROTOCOL_JOIN_LORA:
        status = lorawan_api_join( current_tpm_target_time_ms, current_tpm_stack_id );

        break;
    case TX_PROTOCOL_TRANSMIT_LORA:
    case TX_PROTOCOL_TRANSMIT_LORA_CERTIFICATION:

        status = lorawan_api_payload_send( current_tpm_fport, current_tpm_fport_enabled, current_tpm_data,
                                           current_tpm_data_len, current_tpm_packet_type, current_tpm_target_time_ms,
                                           current_tpm_stack_id );

        break;
    case TX_PROTOCOL_TRANSMIT_LORA_AT_TIME:

        if( ( ( int32_t ) ( current_tpm_target_transmit_at_time + MODEM_MIN_RANDOM_DELAY_MS -
                            ( current_tpm_target_time_ms + update_add_delay_ms( ) ) ) < 0 ) )
        {
            tpm_abort( );
            return ( ERRORLORAWAN );
        }
        else
        {
            status = lorawan_api_payload_send(
                current_tpm_fport, current_tpm_fport_enabled, current_tpm_data, current_tpm_data_len,
                current_tpm_packet_type, current_tpm_target_time_ms + update_add_delay_ms( ), current_tpm_stack_id );
            current_tpm_transmit_at_time = true;
        }
        break;

    case TX_PROTOCOL_TRANSMIT_CID:

        SMTC_MODEM_HAL_TRACE_PRINTF( "TPM Launch TX_PROTOCOL_TRANSMIT_CID current_tpm_data_len = %d data = %d , %d \n",
                                     current_tpm_data_len, current_tpm_data[0], current_tpm_data[1] );
        status = lorawan_api_send_stack_cid_req( &current_tpm_data[0], current_tpm_data_len, current_tpm_target_time_ms,
                                                 current_tpm_stack_id );
        break;
    default:
        break;
    }
    if( current_tpm_transmit_at_time == true )
    {
        lorawan_api_set_next_tx_at_time( current_tpm_stack_id, true );
    }
    if( status == ERRORLORAWAN )
    {
        tpm_abort( );
    }
    return status;
}

/*******************************************************************************************************************
 *In this next section, there is every function that manages the LBT transaction. These functions are
 *called when the state in the list is TPM_STATE_LBT_BEFORE_WOR or TPM_STATE_LBT
 *The LBT service has been defined with three different callbacks. When the LBT service is executed, it finishes by
 *calling one of these three callbacks:
 *
 * 1) It calls modem_tpm_radio_busy_lbt if an interferer has been detected (channel not free).
 * 2) It calls modem_tpm_radio_free_lbt if the channel is free of any other signal.
 * 3) It calls modem_tpm_radio_abort_lbt if it can't execute the LBT sniff task. This can occur, for example, if another
 *higher priority task has been launched by the radio planner. Depending on the callback launched by the LBT service,
 *TPM updates its state machine to launch the next task.
 ********************************************************************************************************************/

// In the case where the channel is always busy, TPM tries MAX_TRIAL_LBT times to relaunch the LBT service before
// aborting the current Tx transaction.
#define MAX_TRIAL_LBT ( ( current_tpm_request_type == TX_PROTOCOL_TRANSMIT_TEST_MODE ) ? 10000UL : 10UL )

/**
 * @brief this function is called by the service LBT when the channel is "busy" meaning interferer block the next
 *transmission
 * @remark This function can be called in two cases: first, when LBT (Listen Before Talk) is performed before a normal
 *LoRaWAN transmission, or before a WOR (Wake on Radio) transmission when the relay is activated.
 *
 *1) Before LoRaWAN transmission ( LBT (optional) + CSMA (optional) + Lr1mac TX):
 *In the case of a busy channel detected during LBT, TPM requests a new channel and relaunches a new LBT sniff. If the
 *maximum number of trials is reached or if there are no more available channels, the current transmission is aborted.
 *
 *2)Before WOR transmission ( Prepare WOR + CSMA (optional) + LBT (optional) + Tx Wor + Rx WorAck (except for join) +
 *LBT (optional) + Lr1mac Tx):
 *In this case, TPM restarts the relay transmission sequence from the beginning (=> call a
 *new prepare wor as a consequence to delayed the transmission of at least 2 CAD periods )). If TPM has already retried
 *MAX_TRIAL_RELAY times without success, the current transmission is aborted
 *
 * @param return
 */

static void modem_tpm_radio_busy_lbt( void* context )
{
#if defined( ADD_RELAY_TX )
    // manage WOR LBT
    if( smtc_relay_tx_is_enable( current_tpm_stack_id ) == true )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "modem_tpm_radio_busy_lbt\n" );
        current_tpm_cpt_relay_max_trial++;
        if( current_tpm_cpt_relay_max_trial < MAX_TRIAL_RELAY )
        {
            current_tpm_target_time_ms = smtc_modem_hal_get_time_in_ms( ) + MODEM_MIN_RANDOM_DELAY_MS;
            // compute_tpm_list to restart the full relay sequence from the beginning (prepare new wor)
            compute_tpm_list( );
            modem_tx_protocol_manager_engine( );
        }
        else
        {
            modem_tpm_radio_abort_relay_tx( NULL );
        }
    }
    else
    {
#endif
        SMTC_MODEM_HAL_TRACE_PRINTF( "modem_tpm_radio_busy_lbt\n" );
        status_lorawan_t status = tpm_get_next_channel( );
        current_tpm_cpt_lbt_max_trial++;
        if( ( current_tpm_cpt_lbt_max_trial < MAX_TRIAL_LBT ) && ( status == OKLORAWAN ) )
        {
            current_tpm_target_time_ms = smtc_modem_hal_get_time_in_ms( ) + MODEM_MIN_RANDOM_DELAY_MS;
            modem_tx_protocol_manager_engine( );
        }
        else
        {
            modem_tpm_radio_abort_lbt( context );
        }
#if defined( ADD_RELAY_TX )
    }

#endif
}
/**
 * @brief this function is called by the service LBT when the channel is "free" meaning no interferer block the next
 *transmission
 * @remark This function can be called in two cases: first, when LBT (Listen Before Talk) is performed before a normal
 *LoRaWAN transmission, or before a WOR (Wake on Radio) transmission when the relay is activated.
 *In the case of a free channel, shift_left_tpm_list is called to execute the next radio preprocessing or the
 *transmission itself.
 *The target time of the transmission is updated to now + 50ms, and the next transmission becomes a
 *scheduled transmission. This can be explained by the fact that if, for any reason, the actual transmission is delayed
 *(due to a higher-priority task in the radio planner), this transmission should not be "protected" by the previous LBT
 *sniff. Therefore, it is preferable to abort the future transmission if the current one is not completed at the correct
 *target time. If the next task in TPM isn't the transmission itself but, for example, a CSMA sniff, this target time
 *will be updated a second time.
 *
 * @param return
 */

static void modem_tpm_radio_free_lbt( void* context )
{
    shift_left_tpm_list( );
    current_tpm_cpt_lbt_max_trial = 0;
    update_tpm_target_time( );
    modem_tx_protocol_manager_engine( );
}
/**
 * @brief this function is to abort the current tpm transmission
 * @remark If, for any reason as explained previously, the LBT phase cannot complete successfully, the current
 * transaction is aborted by resetting the TPM list of tasks. At this stage, if the stack is already running (a specific
 * case of TPM_STATE_NWK_TX_LORA scheduled in the TPM list), it is mandatory to abort the stack itself.
 * @param return
 */

static void modem_tpm_radio_abort_lbt( void* context )
{
    tpm_abort( );
    SMTC_MODEM_HAL_TRACE_PRINTF( "modem_tpm_radio_abort_lbt\n" );
}
/**
 * @brief This function is responsible for initiating the LBT process. It is called by modem_tx_protocol_manager_engine
 * when the current state value is TPM_STATE_LBT.
 * @remark The listen frequency is provided by the LR1MAC stack itself. It is parameterized to be an ASAP task now, and
 * we do not consider reserving the radio planer for the next transmission. This task was designated as an ASAP task in
 * the past, so fairness delay will be applied by the radio planner.
 * @param return OKLORAWAN
 */
static status_lorawan_t manage_lbt_state( void )
{
    uint32_t target_time_lbt = current_tpm_target_time_ms;

    if( current_tpm_request_type == TX_PROTOCOL_TRANSMIT_TEST_MODE )
    {
        smtc_lbt_listen_channel( smtc_lbt_get_obj( current_tpm_stack_id ),
                                 ( smtc_modem_test_get_context ) ( )->tx_frequency, false, target_time_lbt, 0 );
        return OKLORAWAN;
    }
    lr1_stack_mac_t* lr1mac_obj = lorawan_api_stack_mac_get( current_tpm_stack_id );

    if( current_tpm_transmit_at_time == true )
    {
        // this case covers the case where relay is on and to perform the lbt between wor_ack and  lr1mac transmission
        // ASAP
        current_tpm_cpt_lbt_max_trial = MAX_TRIAL_LBT;
        smtc_lbt_listen_channel( smtc_lbt_get_obj( current_tpm_stack_id ), lr1mac_obj->tx_frequency, false,
                                 smtc_modem_hal_get_time_in_ms( ), 0 );
    }
    else
    {
        smtc_lbt_listen_channel( smtc_lbt_get_obj( current_tpm_stack_id ), lr1mac_obj->tx_frequency, false,
                                 target_time_lbt, 0 );
    }
    return OKLORAWAN;
}
/*******************************************************************************************************************
 *In this next section,there is every function that manages the CSMA transaction. These functions are
 *called when the state in the list is TPM_STATE_CSMA_BEFORE_WOR or TPM_STATE_CSMA
 *The CSMA service has been defined with four different callbacks. When the CSMA service is executed, it finishes by
 *calling one of these three callbacks:
 *
 * 1) It calls modem_tpm_radio_busy_csma if an interferer has been detected (channel not free).
 * 2) It calls modem_tpm_radio_free_csma if the channel is free of any other LoRa signal.
 * 3) It calls modem_tpm_radio_free_cad_keep_channel if the csma algorithm have activated the backoff mode and is
 *still in it "confirmed" state 4) It calls modem_tpm_radio_abort_csma if it can't execute the csma sniff task. This can
 *occur, for example, if another higher priority task has been launched by the radio planner. Depending on the callback
 *launched by the CSMA service, TPM updates its state machine to launch the next task.
 ********************************************************************************************************************/

/**
 * @brief this function is called by the service CSMA when the channel is "busy" meaning LoRa interferer block the next
 *transmission
 * @remark This function can be called in two cases: first, when CSMA  is performed before a normal
 *LoRaWAN transmission, or before a WOR (Wake on Radio) transmission when the relay is activated.
 *
 *1) Before LoRaWAN transmission ( LBT (optional) + CSMA (optional) + Lr1mac TX):
 *In the case of a busy channel detected during CSMA, TPM requests a new channel and restart the tpm process from the
 *beginning (calling compute_tpm_list). If the maximum number of trials (in case of dual mode LBT + CSMA) is reached or
 *if there are no more available channels, the current transmission is aborted.
 *
 *2)Before WOR transmission ( Prepare WOR + CSMA (optional) + LBT (optional) + Tx Wor + Rx WorAck (except for join) +
 *LBT (optional) + Lr1mac Tx):
 *In this case, TPM restarts the relay transmission sequence from the beginning (=> call a
 *new prepare wor as a consequence to delayed the transmission of at least 2 CAD periods ) If TPM has already retried
 *MAX_TRIAL_RELAY times without success, the current transmission is aborted
 *
 * @param return
 */
#if defined( ADD_CSMA )
static void modem_tpm_radio_busy_csma( void* context )
{
#if defined( ADD_RELAY_TX )
    // manage WOR CSMA
    if( smtc_relay_tx_is_enable( current_tpm_stack_id ) == true )
    {
        current_tpm_cpt_relay_max_trial++;
        if( current_tpm_cpt_relay_max_trial < MAX_TRIAL_RELAY )
        {
            current_tpm_target_time_ms = smtc_modem_hal_get_time_in_ms( ) + MODEM_MIN_RANDOM_DELAY_MS;
            // compute_tpm_list to restart the full relay sequence from the beginning (prepare new wor)
            compute_tpm_list( );
            modem_tx_protocol_manager_engine( );
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "relay aborted due to busy csma\n" );
            modem_tpm_radio_abort_relay_tx( NULL );
        }
    }
    else
    {
#endif
        SMTC_MODEM_HAL_TRACE_PRINTF( "modem_tpm_radio_busy_csma\n" );
        status_lorawan_t status = tpm_get_next_channel( );
        compute_tpm_list( );
        if( status == OKLORAWAN )
        {
            modem_tx_protocol_manager_engine( );
        }
        else
        {
            modem_tpm_radio_abort_csma( context );
        }
#if defined( ADD_RELAY_TX )
    }

#endif
}
/**
 * @brief this function is called by the service CSMA when the channel is "free" meaning no LoRa interferer block the
 *next transmission
 * @remark This function can be called in two cases: first, when CSMA  is performed before a normal
 *LoRaWAN transmission, or before a WOR (Wake on Radio) transmission when the relay is activated.
 *In the case of a free channel, shift_left_tpm_list is called to execute the next radio preprocessing or the
 *transmission itself.
 *The target time (except in a specific case explain below) of the transmission is updated to now + 50ms, and the next
 *transmission becomes a scheduled transmission. This can be explained by the fact that if, for any reason, the actual
 *transmission is delayed (due to a higher-priority task in the radio planner), this transmission should not be
 *"protected" by the previous CSMA sniff. Therefore, it is preferable to abort the future transmission if the current
 *one is not completed at the correct target time. If the next task in TPM isn't the transmission itself , this target
 *time will be updated a second time. In a specific case of basic Lr1mac transmission just protected by CSMA (but no LBT
 *, no relay mode enabled), it is preferable to optimized the delay between the CSMA sniff and the starting time of the
 *transmission , so we have choosen to let the tx asap taking the risk to interleave other RP task between csma and tx
 * @param return
 */
static void modem_tpm_radio_free_csma( void* context )
{
    shift_left_tpm_list( );
    update_tpm_target_time( );
    modem_tx_protocol_manager_engine( );
}
/**
 * @brief this function is to abort the current tpm transmission
 * @remark If, for any reason as explained previously, the CSMA phase cannot complete successfully, the current
 * transaction is aborted by resetting the TPM list of tasks. At this stage, if the stack is already running (a specific
 * case of TPM_STATE_NWK_TX_LORA scheduled in the TPM list), it is mandatory to abort the stack itself.
 * @param return
 */
static void modem_tpm_radio_abort_csma( void* context )
{
    tpm_abort( );
    SMTC_MODEM_HAL_TRACE_PRINTF( "modem_tpm_radio_abort_csma\n" );
}

/**
 * @brief this function is called by the service CSMA when the channel is "busy" meaning LoRa interferer block the next
 * but in special case of csma backoff procedure.
 * @remark This function can be called in two cases: first, when CSMA  is performed before a normal
 *LoRaWAN transmission, or before a WOR (Wake on Radio) transmission when the relay is activated.
 *
 *1) Before LoRaWAN transmission ( LBT (optional) + CSMA (optional) + Lr1mac TX):
 *In the case of a busy channel detected during CSMA, In this special case TPM doesn't requests a new channel and
 *shouldn't restart the tpm process from the beginning (calling compute_tpm_list).
 *
 *2)Before WOR transmission ( Prepare WOR + CSMA (optional) + LBT (optional) + Tx Wor + Rx WorAck (except for join) +
 *LBT (optional) + Lr1mac Tx):
 *In this case, this function has the same behaviour that the function modem_tpm_radio_busy_csma
 *
 */
static void modem_tpm_radio_free_cad_keep_channel( void* context )
{
#if defined( ADD_RELAY_TX )
    // manage WOR CSMA
    if( smtc_relay_tx_is_enable( current_tpm_stack_id ) == true )
    {
        modem_tpm_radio_free_csma( context );
    }
    else
    {
#endif
        // Do not restart LBT between each CSMA check , on this mode we have to keep the same frequency
        // => so do not call compute_tpm_list( );
        update_tpm_target_time( );
        modem_tx_protocol_manager_engine( );
#if defined( ADD_RELAY_TX )
    }
#endif
}
/**
 * @brief This function is responsible for initiating the CSMA process. It is called by modem_tx_protocol_manager_engine
 * when the current state value is TPM_STATE_CSMA.
 * @remark The parameters are provided by the LR1MAC stack itself. It is parameterized to be an ASAP task now, and
 * we do not consider reserving the radio planer for the next transmission. This task was designated as an ASAP task in
 * the past, so fairness delay will be applied by the radio planner.
 * Note : if the current modulation isn't a LoRa modulation the chanel is considered as a free chanel
 * @param return OKLORAWAN
 */
static status_lorawan_t manage_csma_state( void )
{
    uint32_t target_time_csma = current_tpm_target_time_ms;

    if( current_tpm_request_type == TX_PROTOCOL_TRANSMIT_TEST_MODE )
    {
        modem_test_context_t* test_mode_context = smtc_modem_test_get_context( );
        smtc_lora_cad_bt_listen_channel(
            smtc_cad_get_obj( current_tpm_stack_id ), test_mode_context->tx_frequency, test_mode_context->sf,
            test_mode_context->bw, false, target_time_csma, 0,
            16,  // 16 because the max trial of csma have to be manage only by cad user setting
            test_mode_context->invert_iq );
        return OKLORAWAN;
    }
    lr1_stack_mac_t* lr1mac_obj = lorawan_api_stack_mac_get( current_tpm_stack_id );
    if( smtc_real_get_modulation_type_from_datarate( lr1mac_obj->real, lr1mac_obj->tx_data_rate ) == LORA )
    {
        uint8_t            tx_sf;
        lr1mac_bandwidth_t tx_bw;
        smtc_real_lora_dr_to_sf_bw( lr1mac_obj->real, lr1mac_obj->tx_data_rate, &tx_sf, &tx_bw );

        smtc_lora_cad_bt_listen_channel( smtc_cad_get_obj( current_tpm_stack_id ), lr1mac_obj->tx_frequency, tx_sf,
                                         ( ral_lora_bw_t ) tx_bw, false, target_time_csma, 0,
                                         lr1mac_obj->nb_available_tx_channel, false );
    }
    else
    {
        modem_tpm_radio_free_csma( NULL );
    }
    return OKLORAWAN;
}
#endif  // end of CSMA section

/*******************************************************************************************************************
 *In this next section,there is every function that manages the Relay transaction. These functions are
 *called when the state in the list is one of the following state :   TPM_STATE_PREPARE_WOR,
 *TPM_STATE_LBT_BEFORE_WOR,TPM_STATE_CSMA_BEFORE_WOR, TPM_STATE_RELAY_TX,
 *The Relay service has been defined with two different callbacks. When the Relay service is executed, it finishes by
 *calling one of these two callbacks:
 *
 * 1) It calls modem_tpm_radio_free_relay_tx if tpm can launch the Lr1mac transmission
 * 2) It calls modem_tpm_radio_abort_relay_tx if tpm have to abort the current transmission .
 *Depending on the callback, TPM updates its state machine to launch the next task.
 ********************************************************************************************************************/

#if defined( ADD_RELAY_TX )
// the wor_tx variable is updated by smtc_relay_tx_prepare_wor to provide every parameters required to start the wor
// transmission at the right time this variable will be reuse by smtc_relay_tx_send_wor, manage_lbt_before_wor_state or
// manage_csma_before_wor_state functions
static wor_tx_prepare_t wor_tx = { 0 };

/**
 * @brief This function is responsible for initiating the computation of the wor parameter when relay tx mode is
 *enabled. It is called by modem_tx_protocol_manager_engine when the current state value is TPM_STATE_CSMA.
 * @remark When relay mode is activated, TPM should first execute this function in order to pre-compute the new
 *WOR parameter. Unlike the other functions called by
 *modem_tx_protocol_manager_engine, this one has no callback executed after a radio action.
 * Note : at the end of this function, the modem_tx_protocol_manager_engine should be  called a second time, which means
 *that this function is reentrant.
 * @param return status_lorawan_t
 */
static status_lorawan_t manage_prepare_wor_state( void )
{
    shift_left_tpm_list( );
    lr1_stack_mac_t* lr1mac_obj = lorawan_api_stack_mac_get( current_tpm_stack_id );
    wor_lr1_infos_t  lr1_infos  = {

        .dev_addr          = lr1mac_obj->dev_addr,
        .freq_hz           = lr1mac_obj->tx_frequency,
        .dr                = lr1mac_obj->tx_data_rate,
        .is_join           = ( ( lr1mac_obj->join_status == JOINED ) ? true : false ),
        .crystal_error_ppm = lr1mac_obj->crystal_error,
    };

    if( smtc_relay_tx_prepare_wor( current_tpm_stack_id, current_tpm_target_time_ms, &lr1_infos, &wor_tx ) == false )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "relay aborted due to prepare wor\n" );
        modem_tpm_radio_abort_relay_tx( NULL );
        return ( ERRORLORAWAN );
    }
    else
    {
        current_tpm_target_time_ms = wor_tx.target_time_ms - update_add_delay_ms( );
        modem_tx_protocol_manager_engine( );
        return ( OKLORAWAN );
    }
}

/**
 * @brief This function is responsible for initiating the LBT process. It is called by modem_tx_protocol_manager_engine
 * when the current state value is TPM_STATE_LBT_BEFORE_WOR.
 * @remark The listen frequency is provided by the wor_tx variable itself. It is parameterized to be an ASAP task, and
 * we do not consider reserving the radio planner for the next transmission. This task could be performed after CSMA and
 * before WOR transmission due to the fact that WOR transmission occurs at a certain time (to cover relay sync mode);
 * the LBT should be finished before the starting time of the WOR. Note: this timing has been provided during the state
 * prepare WOR. In any case, if this LBT task finishes too late, the next transmission of the WOR will be aborted
 * by the radio planner itself. Therefore, it is not mandatory at this step to cover all potential failure cases.
 * * @param return OKLORAWAN
 */
static status_lorawan_t manage_lbt_before_wor_state( void )
{
    current_tpm_cpt_lbt_max_trial = MAX_TRIAL_LBT;
    smtc_lbt_listen_channel( smtc_lbt_get_obj( current_tpm_stack_id ), wor_tx.freq_hz, false,
                             current_tpm_target_time_ms, 0 );

    return OKLORAWAN;
}
/**
 * @brief This function is responsible for initiating the CSMA process. It is called by modem_tx_protocol_manager_engine
 * when the current state value is TPM_STATE_CSMA_BEFORE_WOR.
 * @remark The different parameters are provided by the wor_tx variable itself. It is parameterized to be csma performed
 * ASAP."
 * @param return status_lorawan_t
 */
#if defined( ADD_CSMA )
static status_lorawan_t manage_csma_before_wor_state( void )
{
    smtc_lora_cad_bt_listen_channel( smtc_cad_get_obj( current_tpm_stack_id ), wor_tx.freq_hz, wor_tx.sf, wor_tx.bw,
                                     false, current_tpm_target_time_ms, 0, MAX_TRIAL_RELAY, true );

    return OKLORAWAN;
}
#endif
/**
 * @brief This function is responsible for initiating the WOR transmission process. It is called by
 * modem_tx_protocol_manager_engine when the current state value is TPM_STATE_RELAY_TX.
 * @remark The different parameters are provided by the wor_tx variable itself. smtc_relay_tx_send_wor takes in charge
 * the WOR transmission but also the WOR Ack reception it finishes by calling one of these two callbacks:
 *
 * 1) It calls modem_tpm_radio_free_relay_tx if tpm can continue the Lr1mac transmission sequence.
 * 2) It calls modem_tpm_radio_abort_relay_tx if tpm have to abort the current transmission .
 * @param return status_lorawan_t
 */
static status_lorawan_t manage_relay_tx_state( void )
{
    if( smtc_relay_tx_send_wor( current_tpm_stack_id, &wor_tx ) == false )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "relay aborted due to smtc_relay_tx_send_wor\n" );
        modem_tpm_radio_abort_relay_tx( NULL );
        return ( ERRORLORAWAN );
    }
    else
    {
        return ( OKLORAWAN );
    }
}
/**
 * @brief This function is called by relay service when the current WOR process has been completed successfully .
 * @remark This callback provides also a context to update the starting time of the current lr1mac transmission, the
 * standard relay keep a fix delay of 50ms after the wor ack reception before to really launch the lr1mac transmission.
 * it allows to perform a one shot lbt sniffing if this one is enabled.
 * @param return NONE
 */
static void modem_tpm_radio_free_relay_tx( void* context )
{
    shift_left_tpm_list( );
    current_tpm_target_time_ms = ( ( cb_return_param_t* ) context )->lr1_timing;
    if( current_tpm_request_type == TX_PROTOCOL_TRANSMIT_LORA_AT_TIME )
    {
        current_tpm_target_time_ms -= update_add_delay_ms( );
    }
    current_tpm_transmit_at_time = true;
    modem_tx_protocol_manager_engine( );
}
/**
 * @brief This function is called by relay service when the current WOR process has been completed unsuccessfully .
 * @remark In this case the current transmission is aborted
 * @param return NONE
 */
static void modem_tpm_radio_abort_relay_tx( void* context )
{
    tpm_abort( );
    SMTC_MODEM_HAL_TRACE_PRINTF( "modem_tpm_radio_abort_relay_tx\n" );
}
#endif  // END of relay section

/****************************************************************/
/******************Utilities functions***************************/
/****************************************************************/
static status_lorawan_t tpm_get_next_channel( void )
{
#if defined( ADD_RELAY_TX )
    if( ( tpm_list_of_state_to_execute[0] == TPM_STATE_LBT_BEFORE_WOR )
#if defined( ADD_CSMA )
        || ( tpm_list_of_state_to_execute[0] == TPM_STATE_CSMA_BEFORE_WOR )
#endif
    )
    {
        return ( OKLORAWAN );
    }
#endif
    if( current_tpm_request_type == TX_PROTOCOL_TRANSMIT_TEST_MODE )
    {
        return ( OKLORAWAN );
    }

    if( current_tpm_request_type == TX_PROTOCOL_JOIN_LORA )
    {
        return ( lorawan_api_update_join_channel( current_tpm_stack_id ) );
    }
    else
    {
        return ( lorawan_api_update_next_tx_channel( current_tpm_stack_id ) );
    }
}

/**
 * @brief This function compute and fix the order of the different state that will be executed by tpm before to really
 * transmit the lr1mac frame
 *  .
 * @remark The order of the tasks is fix and depend only of the activation of the different services (LBT,CSmA,RELAY)
 * case 1) the relay is disabled, the list is :  LBT (optional) + CSMA (optional) + Lr1mac TX
 * case 2) the relay is enabled, the list is  : Prepare WOR + CSMA (optional) + LBT (optional) + Tx Wor + Rx WorAck
 * (except for join) + LBT (optional) + Lr1mac Tx
 * @param return NONE
 */
static uint32_t update_add_delay_ms( void )
{
    current_tpm_add_delay_ms = 0;
#if defined( ADD_CSMA )
    if( smtc_lora_cad_bt_get_state( smtc_cad_get_obj( current_tpm_stack_id ) ) == true )
    {
        current_tpm_add_delay_ms += 10;  // 10 ms margin;
    }
#endif
    if( smtc_lbt_get_state( smtc_lbt_get_obj( current_tpm_stack_id ) ) == true )
    {
        uint32_t listen_duration_ms;
        int16_t  threshold_dbm;
        uint32_t bw_hz;
        smtc_lbt_get_parameters( smtc_lbt_get_obj( current_tpm_stack_id ), &listen_duration_ms, &threshold_dbm,
                                 &bw_hz );
        current_tpm_add_delay_ms += ( listen_duration_ms ) + smtc_modem_hal_get_radio_tcxo_startup_delay_ms( );
    }

    current_tpm_add_delay_ms += LOG_MARGIN_DELAY;

    return ( current_tpm_add_delay_ms );
}
/**
 * @brief This function compute and fix the order of the different state
 **/
/**
 * @brief Computes the list of states to execute for the transmission protocol manager (TPM).
 *
 * This function determines the sequence of states that the TPM should execute based on the current
 * request type and various conditions such as LBT (Listen Before Talk) and CSMA (Carrier Sense Multiple Access).
 * The sequence of states is stored in the global array `tpm_list_of_state_to_execute`.
 *
 * The function handles different scenarios:
 * - Transmit Test Mode: Adds TPM_STATE_LBT and TPM_STATE_TEST_MODE to the list.
 * - Relay TX Mode: Adds a sequence of states including TPM_STATE_PREPARE_WOR, TPM_STATE_CSMA_BEFORE_WOR,
 *   TPM_STATE_LBT_BEFORE_WOR, TPM_STATE_RELAY_TX, TPM_STATE_LBT, TPM_STATE_TX_LORA, or TPM_STATE_NWK_TX_LORA.
 * - Default Mode: Adds a sequence of states including TPM_STATE_CSMA, TPM_STATE_LBT, TPM_STATE_TX_LORA, or
 * TPM_STATE_NWK_TX_LORA.
 *
 * The function ensures that the index does not exceed the maximum list length and terminates the list with
 * TPM_STATE_IDLE.
 *
 * @note This function uses conditional compilation to include or exclude CSMA and Relay TX functionality.
 */
static void compute_tpm_list( void )
{
#if defined( ADD_CSMA )
    lr1_stack_mac_t* lr1mac_obj = lorawan_api_stack_mac_get( current_tpm_stack_id );
#endif
    uint8_t index = 0;
    reset_tpm_list( );
    if( current_tpm_request_type == TX_PROTOCOL_TRANSMIT_TEST_MODE )
    {
#if defined( ADD_CSMA )
        modem_test_context_t* test_mode_context = smtc_modem_test_get_context( );
        if( ( smtc_lora_cad_bt_get_state( smtc_cad_get_obj( current_tpm_stack_id ) ) == true ) &&
            ( test_mode_context->modulation_type == LORA ) )
        {
            tpm_list_of_state_to_execute[index++] = TPM_STATE_CSMA;
        }
#endif
        if( smtc_lbt_get_state( smtc_lbt_get_obj( current_tpm_stack_id ) ) == true )
        {
            tpm_list_of_state_to_execute[index++] = TPM_STATE_LBT;
        }
        tpm_list_of_state_to_execute[index++] = TPM_STATE_TEST_MODE;
        tpm_list_of_state_to_execute[index]   = TPM_STATE_IDLE;
        return;
    }
#if defined( ADD_RELAY_TX )
    ///////////////////////////////////////////////////////////////
    // SEQUENCE IS PREPARE_WOR,CSMA_WOR,LBT_WOR,WOR,LBT,TX_LORA  //
    ///////////////////////////////////////////////////////////////
    if( smtc_relay_tx_is_enable( current_tpm_stack_id ) == true )
    {
        tpm_list_of_state_to_execute[index++] = TPM_STATE_PREPARE_WOR;

#if defined( ADD_CSMA )

        if( ( smtc_lora_cad_bt_get_state( smtc_cad_get_obj( current_tpm_stack_id ) ) == true ) &&
            ( smtc_real_get_modulation_type_from_datarate( lr1mac_obj->real, lr1mac_obj->tx_data_rate ) == LORA ) )
        {
            tpm_list_of_state_to_execute[index++] = TPM_STATE_CSMA_BEFORE_WOR;
        }
#endif
        if( smtc_lbt_get_state( smtc_lbt_get_obj( current_tpm_stack_id ) ) == true )
        {
            tpm_list_of_state_to_execute[index++] = TPM_STATE_LBT_BEFORE_WOR;
        }
        tpm_list_of_state_to_execute[index++] = TPM_STATE_RELAY_TX;
        if( smtc_lbt_get_state( smtc_lbt_get_obj( current_tpm_stack_id ) ) == true )
        {
            tpm_list_of_state_to_execute[index++] = TPM_STATE_LBT;
        }
        if( current_tpm_transaction_is_a_retransmit == false )
        {
            tpm_list_of_state_to_execute[index++] = TPM_STATE_TX_LORA;
        }
        else
        {
            tpm_list_of_state_to_execute[index++] = TPM_STATE_NWK_TX_LORA;
        }
    }
    else
    {
        ///////////////////////////////////
        // SEQUENCE IS CSMA,LBT,TX_LORA  //
        ///////////////////////////////////
#endif

#if defined( ADD_CSMA )
        if( ( smtc_lora_cad_bt_get_state( smtc_cad_get_obj( current_tpm_stack_id ) ) == true ) &&
            ( smtc_real_get_modulation_type_from_datarate( lr1mac_obj->real, lr1mac_obj->tx_data_rate ) == LORA ) )
        {
            tpm_list_of_state_to_execute[index++] = TPM_STATE_CSMA;
        }
#endif
        if( smtc_lbt_get_state( smtc_lbt_get_obj( current_tpm_stack_id ) ) == true )
        {
            tpm_list_of_state_to_execute[index++] = TPM_STATE_LBT;
        }
        if( current_tpm_transaction_is_a_retransmit == false )
        {
            tpm_list_of_state_to_execute[index++] = TPM_STATE_TX_LORA;
        }
        else
        {
            tpm_list_of_state_to_execute[index++] = TPM_STATE_NWK_TX_LORA;
        }
#if defined( ADD_RELAY_TX )
    }
#endif
    if( index >= MAX_LIST_LENGTH )
    {
        SMTC_MODEM_HAL_PANIC( "not valid index %d", index );
    }
    tpm_list_of_state_to_execute[index] = TPM_STATE_IDLE;
}
/**
 * @brief This function update the current list by shifting left the elements , as a consequence the first element is
 * lost and the last one is replaced by TPM_STATE_IDLE
 * @param return NONE
 */
static void shift_left_tpm_list( void )
{
    for( int i = 0; i < ( MAX_LIST_LENGTH - 1 ); i++ )
    {
        tpm_list_of_state_to_execute[i] = tpm_list_of_state_to_execute[i + 1];
    }
    tpm_list_of_state_to_execute[MAX_LIST_LENGTH - 1] = TPM_STATE_IDLE;
}
/**
 * @brief This function reset the current list  as a consequence all the element are replaced by TPM_STATE_IDLE
 * @param return NONE
 */
static void reset_tpm_list( void )
{
    for( int i = 0; i < MAX_LIST_LENGTH; i++ )
    {
        tpm_list_of_state_to_execute[i] = TPM_STATE_IDLE;
    }
}
#if 0  // TODO for future use
static void remove_state_in_tpm_list( tx_protocol_manager_state_t state )
{
    for( int i = 0; i < MAX_LIST_LENGTH - 2; i++ )
    {
        if( tpm_list_of_state_to_execute[i] == state )
        {
            memcpy( &tpm_list_of_state_to_execute[i], &tpm_list_of_state_to_execute[i + 1],
                    MAX_LIST_LENGTH - ( i + 1 ) );
        }
        if( tpm_list_of_state_to_execute[MAX_LIST_LENGTH - 1] == state )
        {
            tpm_list_of_state_to_execute[MAX_LIST_LENGTH - 1] = TPM_STATE_IDLE;
        }
    }
}
#endif

/**
 * @brief This function is responsible for aborting current tpm process. .
 * @param return NONE
 */
static void tpm_abort( void )
{
    reset_tpm_list( );
    current_tpm_transmit_is_aborted = true;
    if( current_tpm_request_type == TX_PROTOCOL_TRANSMIT_TEST_MODE )
    {
        test_mode_cb_tpm( current_tpm_data, current_tpm_data_len, true );
    }

    lorawan_api_core_abort( current_tpm_stack_id );
}

/**
 * @brief This function is responsible for initiating the CSMA process. It is called by modem_tx_protocol_manager_engine
 * when the current state value is TPM_STATE_IDLE. it doesn't proceed anything.
 * @param return NONE
 */
static status_lorawan_t manage_idle_state( void )
{
    return OKLORAWAN;
}
static status_lorawan_t manage_test_mode( void )
{
    shift_left_tpm_list( );
    return ( test_mode_cb_tpm( current_tpm_data, current_tpm_data_len, false ) );
}
static void update_tpm_target_time( void )
{
    if( current_tpm_transmit_at_time == false )
    {
        current_tpm_target_time_ms = smtc_modem_hal_get_time_in_ms( );
    }
}
/**
 * @brief This function is called each time the current state is executed. It is written for purpose debug and could be
 * activated by setting ENABLE_DEBUG_TPM = 1
 * @param return NONE
 */
static void tpm_debug_print( void )
{
    switch( tpm_list_of_state_to_execute[0] )
    {
#if defined( ADD_RELAY_TX )
    case TPM_STATE_PREPARE_WOR:
        SMTC_MODEM_HAL_TRACE_PRINTF( "TPM Launch PREPARE_WOR Service at %d \n", smtc_modem_hal_get_time_in_ms( ) );
        break;
#if defined( ADD_CSMA )
    case TPM_STATE_CSMA_BEFORE_WOR:
        SMTC_MODEM_HAL_TRACE_PRINTF( "TPM Launch CSMA_BEFORE_WOR Service at %d \n", smtc_modem_hal_get_time_in_ms( ) );
        break;
#endif
    case TPM_STATE_LBT_BEFORE_WOR:
        SMTC_MODEM_HAL_TRACE_PRINTF( "TPM Launch LBT_BEFORE_WOR Service at %d \n", smtc_modem_hal_get_time_in_ms( ) );
        break;
    case TPM_STATE_RELAY_TX:
        SMTC_MODEM_HAL_TRACE_PRINTF( "TPM Launch RELAY_TX Service at %d \n", smtc_modem_hal_get_time_in_ms( ) );
        break;
#endif
    case TPM_STATE_LBT:
        SMTC_MODEM_HAL_TRACE_PRINTF( "TPM Launch LBT Service at %d \n", smtc_modem_hal_get_time_in_ms( ) );
        break;
#if defined( ADD_CSMA )
    case TPM_STATE_CSMA:
        SMTC_MODEM_HAL_TRACE_PRINTF( "TPM Launch CSMA Service at %d \n", smtc_modem_hal_get_time_in_ms( ) );
        break;
#endif

    case TPM_STATE_TX_LORA:
        SMTC_MODEM_HAL_TRACE_PRINTF( "TPM Launch TX Lr1mac  at %d \n", smtc_modem_hal_get_time_in_ms( ) );
        break;
    case TPM_STATE_NWK_TX_LORA:
        SMTC_MODEM_HAL_TRACE_PRINTF( "TPM Launch NWK TX Lr1mac  at %d \n", smtc_modem_hal_get_time_in_ms( ) );
        break;
    case TPM_STATE_TEST_MODE:
        SMTC_MODEM_HAL_TRACE_PRINTF( "TPM Launch TEST MODE at %d \n", smtc_modem_hal_get_time_in_ms( ) );
        break;
    case TPM_STATE_IDLE:
        SMTC_MODEM_HAL_TRACE_PRINTF( "TPM Launch idle task at %d \n", smtc_modem_hal_get_time_in_ms( ) );
        break;
    default:
        SMTC_MODEM_HAL_TRACE_ERROR( "TPM state error 0x%x\n", tpm_list_of_state_to_execute[0] );
        break;
    }
}
