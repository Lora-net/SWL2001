/**
 * @file      lorawan_remote_multicast_setup_package.c
 *
 * @brief      Implements the LoRa-Alliance remote multicast setup package
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
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "modem_event_utilities.h"
#include "lorawan_send_management.h"
#include "modem_supervisor_light.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "lorawan_api.h"
#include "lr1mac_defs.h"
#include "lorawan_remote_multicast_setup_package.h"
#include "fragmentation_helper.h"
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
/*!
 * \brief Returns the minimum value between a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval minValue Minimum value
 */
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )

/**
 * @brief Number of MULTICAST_SETUP_PACKAGE object
 *
 */

#define NUMBER_OF_REMOTE_MULTICAST_SETUP_PACKAGE_OBJ 1
/**
 * @brief Compute current LoRaWAN Stack from the supervisor task_id
 *
 */
#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )

/**
 * @brief Check is the index is valid before accessing object
 *
 */
#define IS_VALID_OBJECT_ID( x )                                   \
    do                                                            \
    {                                                             \
        if( x >= NUMBER_OF_REMOTE_MULTICAST_SETUP_PACKAGE_OBJ )   \
        {                                                         \
            SMTC_MODEM_HAL_PANIC( "not valid service_id %d", x ); \
        }                                                         \
    } while( 0 )

#define IS_VALID_PKG_CMD( x )                                       \
    do                                                              \
    {                                                               \
        if( ( rx_buffer_index + x ) > rx_buffer_length )            \
        {                                                           \
            SMTC_MODEM_HAL_TRACE_ERROR( "%u\n", rx_buffer_length ); \
            return REMOTE_MULTICAST_SETUP_STATUS_ERROR;             \
        }                                                           \
    } while( 0 )
#define CURRENT_TASK_CONTEXT \
    ( modem_supervisor_get_task( ) )->modem_task[( modem_supervisor_get_task( ) )->next_task_id].task_context

/*!
 * \brief Get a particular bit value from a byte
 *
 * \param [IN] b Any byte from which we want a bit value
 * \param [IN] p Position of the bit in the byte [0..7]
 * \param [IN] n Number of bits we want to get
 * \retval The value corresponding the requested bits
 */
#define TAKE_N_BITS_FROM( b, p, n ) ( ( ( b ) >> ( p ) ) & ( ( 1 << ( n ) ) - 1 ) )

// Return byte b from value v
#define BYTE( v, b ) ( ( ( v ) >> ( 8 * ( b ) ) ) & 0xFF )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * LoRaWAN Application Layer Fragmented Data Block Transport Specification
 */

/* -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

#define REMOTE_MULTICAST_SETUP_PORT ( 200 )
#define REMOTE_MULTICAST_SETUP_ID ( 2 )
#define REMOTE_MULTICAST_SETUP_VERSION ( 2 )
#define REMOTE_MULTICAST_SETUP_SIZE_ANS_MAX ( 15 )

#define REMOTE_MULTICAST_SETUP_PKG_VERSION_ANS 0x00
#define REMOTE_MULTICAST_SETUP_MC_GROUP_STATUS_ANS 0x01
#define REMOTE_MULTICAST_SETUP_MC_GROUP_SETUP_ANS 0x02
#define REMOTE_MULTICAST_SETUP_MC_GROUP_DELETE_ANS 0x03
#define REMOTE_MULTICAST_SETUP_MC_GROUP_CLASS_C_SESSION_ANS 0x04
#define REMOTE_MULTICAST_SETUP_MC_GROUP_CLASS_B_SESSION_ANS 0x05

#define REMOTE_MULTICAST_SETUP_PKG_VERSION_REQ 0x00
#define REMOTE_MULTICAST_SETUP_MC_GROUP_STATUS_REQ 0x01
#define REMOTE_MULTICAST_SETUP_MC_GROUP_SETUP_REQ 0x02
#define REMOTE_MULTICAST_SETUP_MC_GROUP_DELETE_REQ 0x03
#define REMOTE_MULTICAST_SETUP_MC_GROUP_CLASS_C_SESSION_REQ 0x04
#define REMOTE_MULTICAST_SETUP_MC_GROUP_CLASS_B_SESSION_REQ 0x05

#define REMOTE_MULTICAST_SETUP_PKG_VERSION_REQ_SIZE ( 1 )
#define REMOTE_MULTICAST_SETUP_MC_GROUP_STATUS_REQ_SIZE ( 2 )
#define REMOTE_MULTICAST_SETUP_MC_GROUP_SETUP_REQ_SIZE ( 30 )
#define REMOTE_MULTICAST_SETUP_MC_GROUP_DELETE_REQ_SIZE ( 2 )
#define REMOTE_MULTICAST_SETUP_MC_GROUP_CLASS_C_SESSION_REQ_SIZE ( 11 )
#define REMOTE_MULTICAST_SETUP_MC_GROUP_CLASS_B_SESSION_REQ_SIZE ( 11 )

static uint8_t multicast_setup_req_cmd_size[6] = { REMOTE_MULTICAST_SETUP_PKG_VERSION_REQ_SIZE,
                                                   REMOTE_MULTICAST_SETUP_MC_GROUP_STATUS_REQ_SIZE,
                                                   REMOTE_MULTICAST_SETUP_MC_GROUP_SETUP_REQ_SIZE,
                                                   REMOTE_MULTICAST_SETUP_MC_GROUP_DELETE_REQ_SIZE,
                                                   REMOTE_MULTICAST_SETUP_MC_GROUP_CLASS_C_SESSION_REQ_SIZE,
                                                   REMOTE_MULTICAST_SETUP_MC_GROUP_CLASS_B_SESSION_REQ_SIZE };

#define NB_MULTICAST_GROUPS ( 4 )

typedef enum
{
    ANS_CMD_TASK,
    REQUEST_TIME_SYNC_TASK,
    LAUNCH_CLASS_C_TASK_GROUP_ID_0,
    LAUNCH_CLASS_C_TASK_GROUP_ID_1,
    LAUNCH_CLASS_C_TASK_GROUP_ID_2,
    LAUNCH_CLASS_C_TASK_GROUP_ID_3,
    STOP_CLASS_C_TASK_GROUP_ID_0,
    STOP_CLASS_C_TASK_GROUP_ID_1,
    STOP_CLASS_C_TASK_GROUP_ID_2,
    STOP_CLASS_C_TASK_GROUP_ID_3,
    LAUNCH_CLASS_B_TASK_GROUP_ID_0,
    LAUNCH_CLASS_B_TASK_GROUP_ID_1,
    LAUNCH_CLASS_B_TASK_GROUP_ID_2,
    LAUNCH_CLASS_B_TASK_GROUP_ID_3,
    STOP_CLASS_B_TASK_GROUP_ID_0,
    STOP_CLASS_B_TASK_GROUP_ID_1,
    STOP_CLASS_B_TASK_GROUP_ID_2,
    STOP_CLASS_B_TASK_GROUP_ID_3,
} remote_multicast_supervisor_task_types_t;

typedef struct lorawan_remote_multicast_setup_package_s
{
    uint8_t stack_id;
    uint8_t task_id;
    // Uplink buffer ans
    uint8_t  remote_multicast_tx_payload_ans[REMOTE_MULTICAST_SETUP_SIZE_ANS_MAX];
    uint8_t  remote_multicast_tx_payload_ans_size;
    bool     request_time_sync;
    bool     launch_class_c[NB_MULTICAST_GROUPS];
    bool     launch_class_b[NB_MULTICAST_GROUPS];
    bool     stop_class_c[NB_MULTICAST_GROUPS];
    bool     stop_class_b[NB_MULTICAST_GROUPS];
    uint16_t fragmentation_ans_delay;
    bool     enabled;
} lorawan_remote_multicast_setup_package_ctx_t;

typedef struct
{
    /**
     * session_time is the start of the Class C window, and is expressed as the time in
     * seconds since 00:00:00, Sunday 6th of January 1980 (start of the GPS epoch) modulo 2^32.
     * Note that this is the same format as the Time field in the beacon frame.
     */
    uint32_t session_time;

    /**
     * time_out encodes the maximum length in seconds of the multicast session
     * (max time the end-device stays in class C before reverting to class A to save battery)
     * This is a maximum duration because the end-device’s application might decide to revert
     * to class A before the end of the session, this decision is application specific.
     */
    uint32_t time_out;

    /**
     * Frequency used for the multicast in Hz.
     * Values representing frequencies below 100 MHz are reserved for future use.
     * This allows setting the frequency of a channel
     * anywhere between 100 MHz to 1.67 GHz in 100 Hz steps.
     */
    uint32_t frequency;

    /**
     * index of the data rate used for the multicast.
     * Uses the same look-up table than the one used by the LinkAdrReq MAC command of the LoRaWAN protocol.
     */
    uint8_t dr;

    /**
     * Periodicity parameter used for ClassB configuration
     */
    uint8_t periodicity;

} multicast_session_params_t;

typedef struct
{
    /**
     * Whether the group is active
     */
    bool active;

    /**
     * mc_addr is the multicast group network address.
     * mc_addr is negotiated off-band by the application server with the network server.
     */
    uint32_t mc_addr;

    /**
     * The min_fc_fcount field is the next frame counter value of the multicast downlink to be sent by the server
     * for this group. This information is required in case an end-device is added to a group that already exists.
     * The end-device MUST reject any downlink multicast frame using this group multicast address if the frame
     * counter is < min_fc_fcount.
     */
    uint32_t min_fc_fcount;

    /**
     * max_fc_fcount specifies the life time of this multicast group expressed as a maximum number of frames.
     * The end-device will only accept a multicast downlink frame if the 32bits frame counter value
     * min_fc_fcount ≤ fc_fcount < max_fc_fcount.
     */
    uint32_t max_fc_fcount;

    /**
     * Session parameters
     */
    multicast_session_params_t params;

} multicast_group_params_t;

/* -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lorawan_remote_multicast_setup_package_ctx_t
    lorawan_remote_multicast_setup_package_ctx[NUMBER_OF_REMOTE_MULTICAST_SETUP_PACKAGE_OBJ];
static multicast_group_params_t
    multicast_group_params[NB_MULTICAST_GROUPS * NUMBER_OF_REMOTE_MULTICAST_SETUP_PACKAGE_OBJ];
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static void lorawan_remote_multicast_setup_add_task( lorawan_remote_multicast_setup_package_ctx_t* ctx,
                                                     remote_multicast_supervisor_task_types_t      task_type,
                                                     uint32_t                                      delay_s );

/**
 * @brief parse the command received from the application server
 *
 * @param [in] ctx
 * @param [in] rx_buffer        buffer that will be decoded
 * @param [in] rx_buffer_length buffer length
 * @param [in] rx_window        window type to filter unicast/multicast
 * @param [in] stack_id         stack_id
 * @return remote_multicast_setup_status_t
 */

static remote_multicast_setup_status_t remote_multicast_setup_package_parser(
    lorawan_remote_multicast_setup_package_ctx_t* ctx, uint8_t* rx_buffer, uint8_t rx_buffer_length,
    receive_win_t rx_window, uint8_t stack_id );
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lorawan_remote_multicast_setup_package_services_init(
    uint8_t* service_id, uint8_t task_id, uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
    void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ), void** context_callback )
{
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG(
        " lorawan_remote_multicast_setup_package_services_init task_id %d, service_id %d, CURRENT_STACK:%d \n", task_id,
        *service_id, CURRENT_STACK );

    IS_VALID_OBJECT_ID( *service_id );

    memset( &lorawan_remote_multicast_setup_package_ctx[*service_id], 0,
            sizeof( lorawan_remote_multicast_setup_package_ctx_t ) );

    *downlink_callback  = lorawan_remote_multicast_setup_package_service_downlink_handler;
    *on_launch_callback = lorawan_remote_multicast_setup_package_service_on_launch;
    *on_update_callback = lorawan_remote_multicast_setup_package_service_on_update;
    *context_callback   = ( void* ) &( lorawan_remote_multicast_setup_package_ctx[*service_id] );

    lorawan_remote_multicast_setup_package_ctx[*service_id].task_id           = task_id;
    lorawan_remote_multicast_setup_package_ctx[*service_id].stack_id          = CURRENT_STACK;
    lorawan_remote_multicast_setup_package_ctx[*service_id].enabled           = true;
    lorawan_remote_multicast_setup_package_ctx[*service_id].request_time_sync = false;
    for( int i = 0; i < NB_MULTICAST_GROUPS; i++ )
    {
        lorawan_remote_multicast_setup_package_ctx[*service_id].launch_class_c[i] = false;
        lorawan_remote_multicast_setup_package_ctx[*service_id].stop_class_c[i]   = false;
        lorawan_remote_multicast_setup_package_ctx[*service_id].launch_class_b[i] = false;
        lorawan_remote_multicast_setup_package_ctx[*service_id].stop_class_b[i]   = false;
    }
    // fragmentation_package_init( &lorawan_remote_multicast_setup_package_ctx[*service_id] );
}

void lorawan_remote_multicast_setup_package_service_on_launch( void* ctx_service )
{
    lorawan_remote_multicast_setup_package_ctx_t* ctx = ( lorawan_remote_multicast_setup_package_ctx_t* ) ctx_service;
    uint8_t                                       stack_id = ctx->stack_id;
    if( CURRENT_TASK_CONTEXT == ANS_CMD_TASK )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " lorawan_remote_multicast_setup_package launch ANS_CMD_TASK \n" );
        SMTC_MODEM_HAL_TRACE_ARRAY( "ans = ", ctx->remote_multicast_tx_payload_ans,
                                    ctx->remote_multicast_tx_payload_ans_size )
        lorawan_api_payload_send(
            REMOTE_MULTICAST_SETUP_PORT, true, ctx->remote_multicast_tx_payload_ans,
            ctx->remote_multicast_tx_payload_ans_size, UNCONF_DATA_UP,
            smtc_modem_hal_get_time_in_ms( ) + smtc_modem_hal_get_random_nb_in_range( 0, ctx->fragmentation_ans_delay ),
            ctx->stack_id );
        ctx->remote_multicast_tx_payload_ans_size = 0;
    }

    if( CURRENT_TASK_CONTEXT == REQUEST_TIME_SYNC_TASK )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " lorawan_remote_multicast_setup_package launch REQUEST_TIME_SYNC_TASK n" );
        ctx->request_time_sync             = false;
        cid_from_device_t cid_buffer[]     = { DEVICE_TIME_REQ };
        uint8_t           cid_request_size = 1;
        lorawan_api_send_stack_cid_req( cid_buffer, cid_request_size, stack_id );
    }

    if( ( CURRENT_TASK_CONTEXT >= LAUNCH_CLASS_C_TASK_GROUP_ID_0 ) &&
        ( CURRENT_TASK_CONTEXT <= LAUNCH_CLASS_C_TASK_GROUP_ID_3 ) )
    {
        uint8_t mc_grp_id = CURRENT_TASK_CONTEXT - LAUNCH_CLASS_C_TASK_GROUP_ID_0;

        if( smtc_modem_set_class( stack_id, SMTC_MODEM_CLASS_C ) == SMTC_MODEM_RC_OK )
        {
            lorawan_api_multicast_c_start_session( mc_grp_id, multicast_group_params[mc_grp_id].params.frequency,
                                                   multicast_group_params[mc_grp_id].params.dr, stack_id );
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_C, mc_grp_id, stack_id );
            SMTC_MODEM_HAL_TRACE_PRINTF(
                " lorawan_remote_multicast_setup_package launch class c session  with group _id = %d , data_rate = %d, "
                "frequency = %d \n",
                mc_grp_id, multicast_group_params[mc_grp_id].params.dr,
                multicast_group_params[mc_grp_id].params.frequency );
            ctx->launch_class_c[mc_grp_id] = false;
            ctx->stop_class_c[mc_grp_id]   = true;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( " lorawan_remote_multicast_setup_package launch LAUNCH_CLASS_C_TASK \n" );
        }
    }
    if( ( CURRENT_TASK_CONTEXT >= STOP_CLASS_C_TASK_GROUP_ID_0 ) &&
        ( CURRENT_TASK_CONTEXT <= STOP_CLASS_C_TASK_GROUP_ID_3 ) )
    {
        uint8_t mc_grp_id = CURRENT_TASK_CONTEXT - STOP_CLASS_C_TASK_GROUP_ID_0;
        lorawan_api_multicast_c_stop_session( mc_grp_id, stack_id );
        ctx->stop_class_c[mc_grp_id] = false;
        uint8_t tmp                  = 0;
        for( uint8_t i = 0; i < mc_grp_id; i++ )
        {
            // check if there is still a multicast class c session running
            tmp += ( ctx->stop_class_c[i] == false ) ? 0 : 1;
            tmp += ( ctx->launch_class_c[i] == false ) ? 0 : 1;
        }
        if( tmp > 0 )
        {
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_C, 0, stack_id );
        }
    }

    if( ( CURRENT_TASK_CONTEXT >= LAUNCH_CLASS_B_TASK_GROUP_ID_0 ) &&
        ( CURRENT_TASK_CONTEXT <= LAUNCH_CLASS_B_TASK_GROUP_ID_3 ) )
    {
        uint8_t mc_grp_id = CURRENT_TASK_CONTEXT - LAUNCH_CLASS_B_TASK_GROUP_ID_0;

        if( lorawan_api_get_class_b_status( stack_id ) == true )
        {
            lorawan_api_multicast_b_start_session( mc_grp_id, multicast_group_params[mc_grp_id].params.frequency,
                                                   multicast_group_params[mc_grp_id].params.dr,
                                                   multicast_group_params[mc_grp_id].params.periodicity, stack_id );
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_B, mc_grp_id, stack_id );
            SMTC_MODEM_HAL_TRACE_PRINTF(
                " lorawan_remote_multicast_setup_package launch class b session  with group _id = %d , data_rate = %d, "
                "frequency = %d \n",
                mc_grp_id, multicast_group_params[mc_grp_id].params.dr,
                multicast_group_params[mc_grp_id].params.frequency );
            ctx->launch_class_b[mc_grp_id] = false;
            ctx->stop_class_b[mc_grp_id]   = true;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( " lorawan_remote_multicast_setup_package launch LAUNCH_CLASS_C_TASK \n" );
        }
    }
    if( ( CURRENT_TASK_CONTEXT >= STOP_CLASS_B_TASK_GROUP_ID_0 ) &&
        ( CURRENT_TASK_CONTEXT <= STOP_CLASS_B_TASK_GROUP_ID_3 ) )
    {
        uint8_t mc_grp_id = CURRENT_TASK_CONTEXT - STOP_CLASS_B_TASK_GROUP_ID_0;
        lorawan_api_multicast_b_stop_session( mc_grp_id, stack_id );
        ctx->stop_class_b[mc_grp_id] = false;
        uint8_t tmp                  = 0;
        for( uint8_t i = 0; i < mc_grp_id; i++ )
        {
            // check if there is still a multicast class c session running
            tmp += ( ctx->stop_class_b[i] == false ) ? 0 : 1;
            tmp += ( ctx->launch_class_b[i] == false ) ? 0 : 1;
        }
        if( tmp > 0 )
        {
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_B, 0, stack_id );
        }
    }
}

void lorawan_remote_multicast_setup_package_service_on_update( void* ctx_service )
{
    lorawan_remote_multicast_setup_package_ctx_t* ctx = ( lorawan_remote_multicast_setup_package_ctx_t* ) ctx_service;
    uint8_t                                       stack_id = ctx->stack_id;
    // check if time is available
    ctx->request_time_sync = true;
    uint32_t gps_time_s;
    uint32_t gps_fractional_s;

    if( lorawan_api_is_time_valid( stack_id ) == true )
    {
        lorawan_api_convert_rtc_to_gps_epoch_time( smtc_modem_hal_get_time_in_ms( ), &gps_time_s, &gps_fractional_s,
                                                   stack_id );
        ctx->request_time_sync = false;
    }
    else
    {
        // try clk sync package
        ctx->request_time_sync =
            ( lorawan_alcsync_get_gps_time_second( stack_id, &gps_time_s ) == ALC_SYNC_OK ) ? false : true;
    }
    // if time sync is missing laucnh a task to ask time sync (network sync)
    if( ctx->request_time_sync == true )
    {
        lorawan_remote_multicast_setup_add_task( ( lorawan_remote_multicast_setup_package_ctx_t* ) ctx_service,
                                                 REQUEST_TIME_SYNC_TASK, 0 );
        return;
    }

    // search if have to launch class C
    int32_t time_to_start_s  = 0x7FFFFFFF;
    uint8_t find_mc_group_id = 0;
    for( int i = 0; i < NB_MULTICAST_GROUPS; i++ )
    {
        if( ( ctx->launch_class_c[i] == true ) &&
            ( ( int32_t ) ( multicast_group_params[i].params.session_time - gps_time_s ) < time_to_start_s ) )
        {
            time_to_start_s  = ( int32_t ) ( multicast_group_params[i].params.session_time - gps_time_s );
            find_mc_group_id = i;
        }
    }
    if( time_to_start_s != 0x7FFFFFFF )
    {
        uint32_t delay_s = ( time_to_start_s < 0 ) ? 0 : time_to_start_s;
        lorawan_remote_multicast_setup_add_task( ( lorawan_remote_multicast_setup_package_ctx_t* ) ctx_service,
                                                 LAUNCH_CLASS_C_TASK_GROUP_ID_0 + find_mc_group_id, delay_s );
        return;
    }

    // search if have to stop class C
    int32_t time_to_stop_s = 0x7FFFFFFF;
    find_mc_group_id       = 0;
    for( int i = 0; i < NB_MULTICAST_GROUPS; i++ )
    {
        if( ( ctx->stop_class_c[i] == true ) &&
            ( ( int32_t ) ( multicast_group_params[i].params.session_time + multicast_group_params[i].params.time_out -
                            gps_time_s ) < time_to_stop_s ) )
        {
            time_to_stop_s   = ( int32_t ) ( multicast_group_params[i].params.session_time +
                                           multicast_group_params[i].params.time_out - gps_time_s );
            find_mc_group_id = i;
        }
    }
    if( time_to_stop_s != 0x7FFFFFFF )
    {
        uint32_t delay_s = ( time_to_stop_s < 0 ) ? 0 : time_to_stop_s;
        lorawan_remote_multicast_setup_add_task( ( lorawan_remote_multicast_setup_package_ctx_t* ) ctx_service,
                                                 STOP_CLASS_C_TASK_GROUP_ID_0 + find_mc_group_id, delay_s );
    }
    // search if have to launch class B
    time_to_start_s  = 0x7FFFFFFF;
    find_mc_group_id = 0;
    for( int i = 0; i < NB_MULTICAST_GROUPS; i++ )
    {
        if( ( ctx->launch_class_b[i] == true ) &&
            ( ( int32_t ) ( multicast_group_params[i].params.session_time - gps_time_s ) < time_to_start_s ) )
        {
            time_to_start_s  = ( int32_t ) ( multicast_group_params[i].params.session_time - gps_time_s );
            find_mc_group_id = i;
        }
    }
    if( time_to_start_s != 0x7FFFFFFF )
    {
        uint32_t delay_s = ( time_to_start_s < 0 ) ? 0 : time_to_start_s;
        lorawan_remote_multicast_setup_add_task( ( lorawan_remote_multicast_setup_package_ctx_t* ) ctx_service,
                                                 LAUNCH_CLASS_B_TASK_GROUP_ID_0 + find_mc_group_id, delay_s );
        return;
    }
    // search if have to stop class B
    time_to_stop_s   = 0x7FFFFFFF;
    find_mc_group_id = 0;
    for( int i = 0; i < NB_MULTICAST_GROUPS; i++ )
    {
        if( ( ctx->stop_class_b[i] == true ) &&
            ( ( int32_t ) ( multicast_group_params[i].params.session_time + multicast_group_params[i].params.time_out -
                            gps_time_s ) < time_to_stop_s ) )
        {
            time_to_stop_s   = ( int32_t ) ( multicast_group_params[i].params.session_time +
                                           multicast_group_params[i].params.time_out - gps_time_s );
            find_mc_group_id = i;
        }
    }
    if( time_to_stop_s != 0x7FFFFFFF )
    {
        uint32_t delay_s = ( time_to_stop_s < 0 ) ? 0 : time_to_stop_s;
        lorawan_remote_multicast_setup_add_task( ( lorawan_remote_multicast_setup_package_ctx_t* ) ctx_service,
                                                 STOP_CLASS_B_TASK_GROUP_ID_0 + find_mc_group_id, delay_s );
    }
}

uint8_t lorawan_remote_multicast_setup_package_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    uint8_t                                       stack_id = rx_down_data->stack_id;
    lorawan_remote_multicast_setup_package_ctx_t* ctx      = NULL;
    for( uint8_t i = 0; i < NUMBER_OF_REMOTE_MULTICAST_SETUP_PACKAGE_OBJ; i++ )
    {
        if( lorawan_remote_multicast_setup_package_ctx[i].stack_id == stack_id )
        {
            ctx = &lorawan_remote_multicast_setup_package_ctx[i];
            break;
        }
    }

    if( ctx == NULL )
    {
        return false;
    }

    if( stack_id >= NUMBER_OF_REMOTE_MULTICAST_SETUP_PACKAGE_OBJ )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "stack id not valid %u \n", stack_id );
        return false;
    }

    if( ctx->enabled != true )
    {
        return false;
    }

    if( ( rx_down_data->rx_metadata.rx_fport_present == true ) &&
        ( rx_down_data->rx_metadata.rx_fport == REMOTE_MULTICAST_SETUP_PORT ) )
    {
        SMTC_MODEM_HAL_TRACE_ARRAY( "lorawan_remote_multicast_setup_package_service_downlink_handler receive data ",
                                    rx_down_data->rx_payload, rx_down_data->rx_payload_size );
        remote_multicast_setup_status_t rms_status =
            remote_multicast_setup_package_parser( ctx, rx_down_data->rx_payload, rx_down_data->rx_payload_size,
                                                   rx_down_data->rx_metadata.rx_window, stack_id );
        // check if answer have to been transmit
        if( ( rms_status == REMOTE_MULTICAST_SETUP_STATUS_OK ) && ( ctx->remote_multicast_tx_payload_ans_size > 0 ) )
        {
            lorawan_remote_multicast_setup_add_task( ctx, ANS_CMD_TASK, 0 );
        }
        return true;
    }

    return false;
}

void lorawan_remote_multicast_setup_package_service_get_id( uint8_t* pkt_id, uint8_t* pkt_version, uint8_t* pkt_port )
{
    *pkt_id      = REMOTE_MULTICAST_SETUP_ID;
    *pkt_version = REMOTE_MULTICAST_SETUP_VERSION;
    *pkt_port    = REMOTE_MULTICAST_SETUP_PORT;
}
bool lorawan_remote_multicast_setup_mpa_injector( uint8_t stack_id, uint8_t* payload_in, receive_win_t rx_window,
                                                  uint8_t* payload_out, uint8_t* payload_out_length )
{
    lorawan_remote_multicast_setup_package_ctx_t* ctx = NULL;
    *payload_out_length                               = 0;
    for( uint8_t i = 0; i < NUMBER_OF_REMOTE_MULTICAST_SETUP_PACKAGE_OBJ; i++ )
    {
        if( lorawan_remote_multicast_setup_package_ctx[i].stack_id == stack_id )
        {
            ctx = &lorawan_remote_multicast_setup_package_ctx[i];
            break;
        }
    }

    if( ctx == NULL )
    {
        return false;
    }

    if( stack_id >= NUMBER_OF_REMOTE_MULTICAST_SETUP_PACKAGE_OBJ )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "stack id not valid %u \n", stack_id );
        return false;
    }

    if( ctx->enabled != true )
    {
        return false;
    }
    // check cmd id to find cmd_id length special case for data_fragment
    uint8_t                         cmd_id     = payload_in[0];
    remote_multicast_setup_status_t rms_status = remote_multicast_setup_package_parser(
        ctx, payload_in, multicast_setup_req_cmd_size[cmd_id], rx_window, stack_id );

    if( ( rms_status == REMOTE_MULTICAST_SETUP_STATUS_OK ) && ( ctx->remote_multicast_tx_payload_ans_size > 0 ) )

    {
        *payload_out_length = ctx->remote_multicast_tx_payload_ans_size;
        memcpy( payload_out, ctx->remote_multicast_tx_payload_ans, ctx->remote_multicast_tx_payload_ans_size );
        return true;
    }
    else
    {
        return false;
    }
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lorawan_remote_multicast_setup_add_task( lorawan_remote_multicast_setup_package_ctx_t* ctx,
                                                     remote_multicast_supervisor_task_types_t      task_type,
                                                     uint32_t                                      delay_s )
{
    smodem_task task       = { 0 };
    task.id                = ctx->task_id;
    task.stack_id          = ctx->stack_id;
    task.priority          = TASK_HIGH_PRIORITY;
    task.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + delay_s;
    task.task_context      = ( uint8_t ) task_type;
    if( modem_supervisor_add_task( &task ) != TASK_VALID )
    {
        SMTC_MODEM_HAL_PANIC( "Task not valid\n" );
    }
}

// 0200faabbffa23bda959f79e2d5eb22c414025d4f90f010000007d7e0000
static remote_multicast_setup_status_t remote_multicast_setup_package_parser(
    lorawan_remote_multicast_setup_package_ctx_t* ctx, uint8_t* rx_buffer, uint8_t rx_buffer_length,
    receive_win_t rx_window, uint8_t stack_id )
{
    uint8_t rx_buffer_index                   = 0;
    uint8_t ans_index                         = 0;
    ctx->remote_multicast_tx_payload_ans_size = 0;

    SMTC_MODEM_HAL_TRACE_PRINTF( "lorawan1 data size%d  %d %d\n", rx_buffer[0], rx_buffer[1], rx_buffer_length );
    while( rx_buffer_length > rx_buffer_index )
    {
        switch( rx_buffer[rx_buffer_index] )
        {
        case REMOTE_MULTICAST_SETUP_PKG_VERSION_REQ: {
            IS_VALID_PKG_CMD( REMOTE_MULTICAST_SETUP_PKG_VERSION_REQ_SIZE );
            rx_buffer_index += REMOTE_MULTICAST_SETUP_PKG_VERSION_REQ_SIZE;
            ctx->remote_multicast_tx_payload_ans[ans_index++] = REMOTE_MULTICAST_SETUP_PKG_VERSION_ANS;
            ctx->remote_multicast_tx_payload_ans[ans_index++] = REMOTE_MULTICAST_SETUP_ID;
            ctx->remote_multicast_tx_payload_ans[ans_index++] = REMOTE_MULTICAST_SETUP_VERSION;
            break;
        }

        case REMOTE_MULTICAST_SETUP_MC_GROUP_STATUS_REQ: {
            IS_VALID_PKG_CMD( REMOTE_MULTICAST_SETUP_MC_GROUP_STATUS_REQ_SIZE );
            uint8_t mc_grp_mask                               = rx_buffer[rx_buffer_index + 1] & 0x0F;
            uint8_t ans_group_mask                            = 0;
            uint8_t total_groups                              = 0;
            ctx->remote_multicast_tx_payload_ans[ans_index++] = REMOTE_MULTICAST_SETUP_MC_GROUP_STATUS_ANS;
            ctx->remote_multicast_tx_payload_ans[ans_index]   = ans_group_mask;
            uint8_t index_tmp                                 = 2;
            for( uint8_t i = 0; i < NB_MULTICAST_GROUPS; i++ )
            {
                bool requested = ( mc_grp_mask >> i ) & 0x01;
                if( requested && multicast_group_params[i].active )
                {
                    total_groups++;
                    ans_group_mask |= ( 1 << i );
                    ctx->remote_multicast_tx_payload_ans[index_tmp++] = i;
                    ctx->remote_multicast_tx_payload_ans[index_tmp++] = multicast_group_params[i].mc_addr & 0xFF;
                    ctx->remote_multicast_tx_payload_ans[index_tmp++] =
                        ( multicast_group_params[i].mc_addr >> 8 ) & 0xFF;
                    ctx->remote_multicast_tx_payload_ans[index_tmp++] =
                        ( multicast_group_params[i].mc_addr >> 16 ) & 0xFF;
                    ctx->remote_multicast_tx_payload_ans[index_tmp++] =
                        ( multicast_group_params[i].mc_addr >> 24 ) & 0xFF;
                }
            }
            ans_group_mask |= ( total_groups << 4 );
            ctx->remote_multicast_tx_payload_ans[ans_index] = ans_group_mask;
            ans_index                                       = index_tmp;
            rx_buffer_index += REMOTE_MULTICAST_SETUP_MC_GROUP_STATUS_REQ_SIZE;
            break;
        }
        case REMOTE_MULTICAST_SETUP_MC_GROUP_SETUP_REQ: {
            IS_VALID_PKG_CMD( REMOTE_MULTICAST_SETUP_MC_GROUP_SETUP_REQ_SIZE );
            uint8_t mc_grp_id = rx_buffer[rx_buffer_index + 1] & 0x03;
            uint8_t mc_key_encrypted[16];
            multicast_group_params[mc_grp_id].mc_addr =
                rx_buffer[rx_buffer_index + 2] + ( rx_buffer[rx_buffer_index + 3] << 8 ) +
                ( rx_buffer[rx_buffer_index + 4] << 16 ) + ( rx_buffer[rx_buffer_index + 5] << 24 );
            memcpy( mc_key_encrypted, &rx_buffer[rx_buffer_index + 6], 16 );
            multicast_group_params[mc_grp_id].min_fc_fcount =
                rx_buffer[rx_buffer_index + 22] + ( rx_buffer[rx_buffer_index + 23] << 8 ) +
                ( rx_buffer[rx_buffer_index + 24] << 16 ) + ( rx_buffer[rx_buffer_index + 25] << 24 );
            multicast_group_params[mc_grp_id].max_fc_fcount =
                rx_buffer[rx_buffer_index + 26] + ( rx_buffer[rx_buffer_index + 27] << 8 ) +
                ( rx_buffer[rx_buffer_index + 28] << 16 ) + ( rx_buffer[rx_buffer_index + 29] << 24 );

            lorawan_api_multicast_set_fcnt_down_range( mc_grp_id, multicast_group_params[mc_grp_id].min_fc_fcount,
                                                       multicast_group_params[mc_grp_id].max_fc_fcount, stack_id );
            lorawan_api_multicast_set_group_address( mc_grp_id, multicast_group_params[mc_grp_id].mc_addr, stack_id );
            lorawan_api_multicast_derive_group_keys( mc_grp_id, mc_key_encrypted, stack_id );
            multicast_group_params[mc_grp_id].active = true;
            rx_buffer_index += REMOTE_MULTICAST_SETUP_MC_GROUP_SETUP_REQ_SIZE;
            ctx->remote_multicast_tx_payload_ans[ans_index++] = REMOTE_MULTICAST_SETUP_MC_GROUP_SETUP_ANS;
            ctx->remote_multicast_tx_payload_ans[ans_index++] = mc_grp_id;
            break;
        }
        case REMOTE_MULTICAST_SETUP_MC_GROUP_DELETE_REQ: {
            IS_VALID_PKG_CMD( REMOTE_MULTICAST_SETUP_MC_GROUP_DELETE_REQ_SIZE );
            uint8_t mc_grp_id                                 = rx_buffer[rx_buffer_index + 1] & 0x03;
            multicast_group_params[mc_grp_id].active          = false;
            ctx->remote_multicast_tx_payload_ans[ans_index++] = REMOTE_MULTICAST_SETUP_MC_GROUP_DELETE_ANS;
            ctx->remote_multicast_tx_payload_ans[ans_index++] = mc_grp_id;
            // reset time out to force stop class c in case of class c is already started
            multicast_group_params[mc_grp_id].params.time_out = 0;
            ctx->launch_class_c[mc_grp_id] =
                false;  // don't reset ctx->stop_class_c, let do it by the task enqueu by the supervisor
            if( ctx->stop_class_c[mc_grp_id] == true )
            {
                lorawan_api_multicast_c_stop_session( mc_grp_id, stack_id );
                ctx->stop_class_c[mc_grp_id] = false;
            }
            rx_buffer_index += REMOTE_MULTICAST_SETUP_MC_GROUP_DELETE_REQ_SIZE;
            break;
        }

        case REMOTE_MULTICAST_SETUP_MC_GROUP_CLASS_C_SESSION_REQ: {
            IS_VALID_PKG_CMD( REMOTE_MULTICAST_SETUP_MC_GROUP_CLASS_C_SESSION_REQ_SIZE );
            uint8_t mc_grp_id = rx_buffer[rx_buffer_index + 1] & 0x03;

            multicast_group_params[mc_grp_id].params.session_time =
                rx_buffer[rx_buffer_index + 2] + ( rx_buffer[rx_buffer_index + 3] << 8 ) +
                ( rx_buffer[rx_buffer_index + 4] << 16 ) + ( rx_buffer[rx_buffer_index + 5] << 24 );
            multicast_group_params[mc_grp_id].params.time_out  = ( 1 << ( rx_buffer[rx_buffer_index + 6] & 0x0F ) );
            multicast_group_params[mc_grp_id].params.frequency = rx_buffer[rx_buffer_index + 7] +
                                                                 ( rx_buffer[rx_buffer_index + 8] << 8 ) +
                                                                 ( rx_buffer[rx_buffer_index + 9] << 16 );
            multicast_group_params[mc_grp_id].params.frequency *= 100;
            multicast_group_params[mc_grp_id].params.dr = rx_buffer[rx_buffer_index + 10] & 0x0F;
            SMTC_MODEM_HAL_TRACE_PRINTF(
                "multicast_group_params %d ;%d ;%d ;%d \n", multicast_group_params[mc_grp_id].params.session_time,
                multicast_group_params[mc_grp_id].params.time_out, multicast_group_params[mc_grp_id].params.frequency,
                multicast_group_params[mc_grp_id].params.dr );
            // find gps epoch time source
            // try network clk sync package
            uint32_t seconds_since_epoch = 0;
            uint32_t fractional_second;
            // try network clk sync
            if( lorawan_api_is_time_valid( stack_id ) == true )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "multicast_group_params 2\n" );

                lorawan_api_convert_rtc_to_gps_epoch_time( smtc_modem_hal_get_time_in_ms( ), &seconds_since_epoch,
                                                           &fractional_second, stack_id );
            }
            else
            {
                // try clk sync package
                uint32_t gps_time_s_tmp = 0;
                seconds_since_epoch =
                    ( lorawan_alcsync_get_gps_time_second( stack_id, &gps_time_s_tmp ) == ALC_SYNC_OK ) ? gps_time_s_tmp
                                                                                                        : 0;
                SMTC_MODEM_HAL_TRACE_PRINTF( "multicast_group_params 3\n" );
            }
            // checks paramaters
            uint8_t mc_grp_id_status = mc_grp_id;
            mc_grp_id_status |=
                ( seconds_since_epoch >= multicast_group_params[mc_grp_id].params.session_time ) ? ( 1 << 5 ) : 0;
            mc_grp_id_status |= ( multicast_group_params[mc_grp_id].active == false ) ? ( 1 << 4 ) : 0;
            mc_grp_id_status |= ( lorawan_api_is_frequency_valid(
                                      stack_id, multicast_group_params[mc_grp_id].params.frequency ) == ERRORLORAWAN )
                                    ? ( 1 << 3 )
                                    : 0;
            mc_grp_id_status |= ( lorawan_api_is_datarate_valid(
                                      stack_id, multicast_group_params[mc_grp_id].params.dr ) == ERRORLORAWAN )
                                    ? ( 1 << 2 )
                                    : 0;
            ctx->remote_multicast_tx_payload_ans[ans_index++] = REMOTE_MULTICAST_SETUP_MC_GROUP_CLASS_C_SESSION_ANS;
            ctx->remote_multicast_tx_payload_ans[ans_index++] = mc_grp_id_status;

            if( mc_grp_id_status == mc_grp_id )  // no error
            {
                uint32_t time_to_start = multicast_group_params[mc_grp_id].params.session_time - seconds_since_epoch;
                if( time_to_start >= ( 1 << 24 ) )
                {
                    uint32_t time_to_start =
                        multicast_group_params[mc_grp_id].params.session_time - seconds_since_epoch;
                    if( time_to_start >= ( 1 << 24 ) )
                    {
                        time_to_start = 0xFFFFFF;
                    }
                    ctx->remote_multicast_tx_payload_ans[ans_index++] = time_to_start & 0xFF;
                    ctx->remote_multicast_tx_payload_ans[ans_index++] = ( time_to_start >> 8 ) & 0xFF;
                    ctx->remote_multicast_tx_payload_ans[ans_index++] = ( time_to_start >> 16 ) & 0xFF;

                    ctx->launch_class_c[mc_grp_id] = true;
                    lorawan_remote_multicast_setup_add_task( ctx, LAUNCH_CLASS_C_TASK_GROUP_ID_0 + mc_grp_id,
                                                             time_to_start );
                }
                ctx->remote_multicast_tx_payload_ans[ans_index++] = time_to_start & 0xFF;
                ctx->remote_multicast_tx_payload_ans[ans_index++] = ( time_to_start >> 8 ) & 0xFF;
                ctx->remote_multicast_tx_payload_ans[ans_index++] = ( time_to_start >> 16 ) & 0xFF;
                SMTC_MODEM_HAL_TRACE_PRINTF( "multicast_group_params 10\n" );
                ctx->launch_class_c[mc_grp_id] = true;
                lorawan_remote_multicast_setup_add_task( ctx, LAUNCH_CLASS_C_TASK_GROUP_ID_0 + mc_grp_id,
                                                         time_to_start );
            }
            rx_buffer_index += REMOTE_MULTICAST_SETUP_MC_GROUP_CLASS_C_SESSION_REQ_SIZE;

            break;
        }
        case REMOTE_MULTICAST_SETUP_MC_GROUP_CLASS_B_SESSION_REQ: {
            IS_VALID_PKG_CMD( REMOTE_MULTICAST_SETUP_MC_GROUP_CLASS_B_SESSION_REQ_SIZE );
            uint8_t mc_grp_id = rx_buffer[rx_buffer_index + 1] & 0x03;
            multicast_group_params[mc_grp_id].params.session_time =
                rx_buffer[rx_buffer_index + 2] + ( rx_buffer[rx_buffer_index + 3] << 8 ) +
                ( rx_buffer[rx_buffer_index + 4] << 16 ) + ( rx_buffer[rx_buffer_index + 5] << 24 );
            multicast_group_params[mc_grp_id].params.time_out =
                ( 1 << ( rx_buffer[rx_buffer_index + 6] & 0x0F ) ) * 128;
            multicast_group_params[mc_grp_id].params.periodicity = ( ( rx_buffer[rx_buffer_index + 6] & 0x70 ) >> 4 );
            multicast_group_params[mc_grp_id].params.frequency   = rx_buffer[rx_buffer_index + 7] +
                                                                 ( rx_buffer[rx_buffer_index + 8] << 8 ) +
                                                                 ( rx_buffer[rx_buffer_index + 9] << 16 );
            multicast_group_params[mc_grp_id].params.frequency *= 100;
            multicast_group_params[mc_grp_id].params.dr = rx_buffer[rx_buffer_index + 10] & 0x0F;
            // find gps epoch time source
            uint32_t seconds_since_epoch = 0;
            uint32_t fractional_second;
            // try network clk sync
            if( lorawan_api_is_time_valid( stack_id ) == true )
            {
                lorawan_api_convert_rtc_to_gps_epoch_time( smtc_modem_hal_get_time_in_ms( ), &seconds_since_epoch,
                                                           &fractional_second, stack_id );
            }
            else
            {
                uint32_t gps_time_s_tmp = 0;
                seconds_since_epoch =
                    ( lorawan_alcsync_get_gps_time_second( stack_id, &gps_time_s_tmp ) == ALC_SYNC_OK ) ? gps_time_s_tmp
                                                                                                        : 0;
            }
            // checks paramaters

            uint8_t mc_grp_id_status = mc_grp_id;
            mc_grp_id_status |=
                ( seconds_since_epoch >= multicast_group_params[mc_grp_id].params.session_time ) ? ( 1 << 5 ) : 0;
            mc_grp_id_status |= ( multicast_group_params[mc_grp_id].active == false ) ? ( 1 << 4 ) : 0;
            mc_grp_id_status |= ( lorawan_api_is_frequency_valid(
                                      stack_id, multicast_group_params[mc_grp_id].params.frequency ) == false )
                                    ? ( 1 << 3 )
                                    : 0;
            mc_grp_id_status |=
                ( lorawan_api_is_datarate_valid( stack_id, multicast_group_params[mc_grp_id].params.dr ) == false )
                    ? ( 1 << 2 )
                    : 0;

            ctx->remote_multicast_tx_payload_ans[ans_index++] = REMOTE_MULTICAST_SETUP_MC_GROUP_CLASS_B_SESSION_ANS;
            ctx->remote_multicast_tx_payload_ans[ans_index++] = mc_grp_id_status;
            if( mc_grp_id_status == mc_grp_id )  // no error
            {
                uint32_t time_to_start = multicast_group_params[mc_grp_id].params.session_time - seconds_since_epoch;
                if( time_to_start >= ( 1 << 24 ) )
                {
                    time_to_start = 0xFFFFFF;
                }
                ctx->remote_multicast_tx_payload_ans[ans_index++] = time_to_start & 0xFF;
                ctx->remote_multicast_tx_payload_ans[ans_index++] = ( time_to_start >> 8 ) & 0xFF;
                ctx->remote_multicast_tx_payload_ans[ans_index++] = ( time_to_start >> 16 ) & 0xFF;
                ctx->launch_class_b[mc_grp_id]                    = true;
                lorawan_remote_multicast_setup_add_task( ctx, LAUNCH_CLASS_B_TASK_GROUP_ID_0 + mc_grp_id,
                                                         time_to_start );
            }
            rx_buffer_index += REMOTE_MULTICAST_SETUP_MC_GROUP_CLASS_B_SESSION_REQ_SIZE;
            break;
        }
        }
    }
    // If this message was received on a multicast address, the end-device MUST check that the
    // multicast address used was enabled at the creation of the fragmentation session through the
    // McGroupBitMask field of the FragSessionSetup command. If not, the frame SHALL be
    // silently dropped.

    ctx->remote_multicast_tx_payload_ans_size = ans_index;
    SMTC_MODEM_HAL_TRACE_PRINTF( "parser remote_multicast setup %d  \n", ans_index );
    return REMOTE_MULTICAST_SETUP_STATUS_OK;
}

/* --- EOF ------------------------------------------------------------------ */
