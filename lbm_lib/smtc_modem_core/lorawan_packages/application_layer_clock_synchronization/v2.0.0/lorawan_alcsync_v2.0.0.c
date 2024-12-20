/**
 * @file      lorawan_alcsync_v2.0.0.c
 *
 * @brief     LoRaWAN Application Layer Clock Synchronization V2.0.0 Implementation
 *            Specification:
 * https://resources.lora-alliance.org/document/ts003-2-0-0-application-layer-clock-synchronization
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

#include "modem_supervisor_light.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "modem_core.h"
#include "lorawan_api.h"
#include "lorawan_alcsync.h"
#include "modem_event_utilities.h"

#if defined( ADD_SMTC_CLOUD_DEVICE_MANAGEMENT )
#include "device_management_defs.h"
#include "cloud_dm_package.h"
#endif

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

/**
 * @brief Number of object
 *
 */
#define NUMBER_OF_ALCSYNC_OBJ 1

/**
 * @brief Compute current LoRaWAN Stack from the supervisor task_id
 *
 */
#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )

/**
 * @brief Check is the index is valid before accessing the object
 *
 */
#define IS_VALID_OBJECT_ID( x )                                       \
    do                                                                \
    {                                                                 \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_OF_ALCSYNC_OBJ ); \
    } while( 0 )

/**
 * @brief Check is the index is valid before accessing the object
 *
 */
#define IS_VALID_STACK_ID( x )                                   \
    do                                                           \
    {                                                            \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_OF_STACKS ); \
    } while( 0 )

#if ( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
static const char alc_sync_bad_size_str[] = "ALC Sync payload bad size";
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

// ALCSYNC Package defines
#define ALC_SYNC_DEFAULT_PORT ( 202 )
#define ALC_PACKAGE_IDENTIFIER ( 1 )
#define ALC_PACKAGE_VERSION ( 2 )

#define ALC_SYNC_PACKAGE_VERSION_REQ_SIZE ( 1 )
#define ALC_SYNC_PACKAGE_VERSION_ANS_SIZE ( 3 )
#define ALC_SYNC_APP_TIME_REQ_SIZE ( 6 )
#define ALC_SYNC_APP_TIME_ANS_SIZE ( 6 )
#define ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ_SIZE ( 2 )
#define ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_ANS_SIZE ( 6 )
#define ALC_SYNC_FORCE_DEVICE_RESYNC_REQ_SIZE ( 2 )

#define ALC_SYNC_TX_PAYLOAD_SIZE_MAX \
    ( ALC_SYNC_PACKAGE_VERSION_ANS_SIZE + ALC_SYNC_APP_TIME_REQ_SIZE + ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_ANS_SIZE )

#define ALC_SYNC_ANSWER_DELAY_S ( 1 )
#define ALC_SYNC_DEFAULT_REQUEST_PERIOD_S ( 129600UL )  // 36 hours

#define APP_TIME_ANS_TIME_CORRECTION_BYTE ( 0 )
#define APP_TIME_ANS_TOKEN_BYTE ( 4 )

#define ALC_SYNC_DELAY_BEFORE_SEND_S ( 5 )
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */
/**
 * @brief ALC Sync Command ID Request
 *
 * @enum alc_sync_cid_req_t
 */
typedef enum alc_sync_cid_req_e
{
    ALC_SYNC_PACKAGE_VERSION_REQ             = 0,
    ALC_SYNC_APP_TIME_REQ                    = 1,
    ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ = 2,
    ALC_SYNC_FORCE_DEVICE_RESYNC_REQ         = 3,
    ALC_SYNC_NB_CMD_REQ
} alc_sync_cid_req_t;

/**
 * @brief ALC Sync Command ID Answer
 *
 * @enum alc_sync_cid_ans_t
 */
typedef enum alc_sync_cid_ans_e
{
    ALC_SYNC_PACKAGE_VERSION_ANS             = 0,
    ALC_SYNC_APP_TIME_ANS                    = 1,
    ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_ANS = 2,
    ALC_SYNC_NB_CMD_ANS
} alc_sync_cid_ans_t;

static const uint8_t alcsync_req_cmd_size[ALC_SYNC_NB_CMD_REQ] = { ALC_SYNC_PACKAGE_VERSION_REQ_SIZE,
                                                                   ALC_SYNC_APP_TIME_REQ_SIZE,
                                                                   ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ_SIZE,
                                                                   ALC_SYNC_FORCE_DEVICE_RESYNC_REQ_SIZE };

typedef struct lorawan_alcsync_s
{
    uint8_t stack_id;
    uint8_t task_id;

    bool     enabled;
    uint8_t  req_status;
    uint8_t  tx_payload_index;
    uint8_t  tx_payload[ALC_SYNC_TX_PAYLOAD_SIZE_MAX];
    uint8_t  nb_transmission;
    uint8_t  token_req;
    uint8_t  max_length_up_payload;
    uint32_t periodicity_s;
    int32_t  time_correction_s;
    uint32_t timestamp_last_correction_s;
    bool     ans_required;
    bool     event_time_sync;
    bool     use_cloud_dm;
    uint8_t  fport;
} lorawan_alcsync_ctx_t;

typedef struct lorawan_alcsync_service_ctx_s
{
    uint32_t              timestamp_launch[NUMBER_OF_ALCSYNC_OBJ];
    lorawan_alcsync_ctx_t lorawan_alcsync_ctx[NUMBER_OF_ALCSYNC_OBJ];
} lorawan_alcsync_service_ctx_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lorawan_alcsync_service_ctx_t lorawan_alcsync_service_ctx;

#define timestamp_launch lorawan_alcsync_service_ctx.timestamp_launch
#define lorawan_alcsync_ctx lorawan_alcsync_service_ctx.lorawan_alcsync_ctx

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Initialize ALC sync context to default values
 *
 * @param [in] ctx ALCSYNC context
 */
static void alc_sync_init( lorawan_alcsync_ctx_t* ctx );

/**
 * @brief Get the service context linked to a stack_id
 *
 * @param [in]  stack_id                Stack identifier
 * @param [out] service_id              associated occurrence of this service
 * @return lorawan_alcsync_ctx_t*                  ALCSync object context
 */
static lorawan_alcsync_ctx_t* alc_sync_get_ctx_from_stack_id( uint8_t stack_id, uint8_t* service_id );

/**
 * @brief parse the command received from the application server
 *
 * @param [in] ctx                       ALCSYNC context
 * @param [in] alc_sync_rx_buffer        buffer that will be decoded
 * @param [in] alc_sync_rx_buffer_length buffer length
 * @param [in] timestamp                 downlink received timestamp
 * @return alc_sync_ret_t
 */
static alc_sync_ret_t alc_sync_parser( lorawan_alcsync_ctx_t* ctx, uint8_t* alc_sync_rx_buffer,
                                       uint8_t alc_sync_rx_buffer_length, uint32_t timestamp_ms );

/**
 * @brief Construct the package version answer
 *
 * @param [in] lorawan_alcsync_ctx_t *ctx          clock sync pointer context
 * @return None
 */
static void alc_sync_construct_package_version_answer( lorawan_alcsync_ctx_t* ctx );

/**
 * @brief Construct the applicative time request
 *
 * @param [in] lorawan_alcsync_ctx_t *ctx  Clock sync pointer context
 * @param [in] device_time          DeviceTime is the current end-device clock and is expressed as the time in seconds
 *                                  since 00:00:00, Sunday 6 th of January 1980 (start of the GPS epoch) modulo 2^32
 * @param [in] ans_required         If the AnsRequired bit is set to 1 the end-device expects an answer whether its
 *                                  clock is well synchronized or not. If this bit is set to 0, this signals to the AS
 *                                  that it only needs to answer if the end-device clock is de-synchronized.
 * @return None
 */
static void alc_sync_construct_app_time_request( lorawan_alcsync_ctx_t* ctx, uint32_t device_time,
                                                 uint8_t ans_required );

/**
 * @brief Construct the applicative time periodicity answer
 *
 * @param [in] lorawan_alcsync_ctx_t *ctx  Clock sync pointer context
 * @param status                    NotSupported bit is set to 1 if the end-device’s application does not accept a
 *                                  periodicity set by the application server and manages the clock synchronization
 *                                  process and periodicity itself
 * @param time                      Is the current end-device’s clock time captured immediately before the transmission
 *                                  of the radio message
 * @return None
 */
static void alc_sync_construct_app_time_periodicity_answer( lorawan_alcsync_ctx_t* ctx, uint8_t status, uint32_t time );

/**
 * @brief Check if the Tx ALC sync buffer is full
 * @param cmd_size Check is the command size can be added to the current Tx
 * buffer
 * @return bool
 */
static inline bool is_alc_sync_tx_buffer_not_full( lorawan_alcsync_ctx_t* ctx, uint8_t cmd_size );

/**
 * @brief Decode the applicative time answer
 * @param buffer* Contains the app time answer
 * @return alc_sync_ret_t
 */

static alc_sync_ret_t alc_sync_decode_app_time_ans( lorawan_alcsync_ctx_t* ctx, uint8_t* buffer );

/**
 * @brief Decode the app time periodicity
 * @param buffer* Contains the app time periodicity
 * @return None
 */
static void alc_sync_decode_device_app_time_periodicity_req( lorawan_alcsync_ctx_t* ctx, uint8_t* buffer );

/**
 * @brief Decode the force device resync request
 * @param buffer* Contains the resync parameters
 * @return None
 */
static void alc_sync_decode_force_device_resync_req( lorawan_alcsync_ctx_t* ctx, uint8_t* buffer );

/**
 * @brief Returns a signed random number between min and max
 *
 * @param [in] val_1 first range signed value
 * @param [in] val_2 second range signed value
 *
 * @return int32_t Generated random signed number between smallest value and biggest value (between val_1 and val_2)
 */
static int32_t alcsync_get_signed_random_nb_in_range( const int32_t val_1, const int32_t val_2 );

static uint32_t alcsync_build_ans_payload( uint8_t idx );

/**
 * @brief Return the true is the rx_window is multicast
 *
 * @param rx_window
 * @return true
 * @return false
 */
static bool is_received_on_multicast_window( receive_win_t rx_window );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lorawan_alcsync_services_init( uint8_t* service_id, uint8_t task_id,
                                    uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                    void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                                    void** context_callback )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( " lorawan_alcsync_services_init task_id %d, service_id %d, CURRENT_STACK:%d \n",
                                 task_id, *service_id, CURRENT_STACK );

    IS_VALID_OBJECT_ID( *service_id );

    lorawan_alcsync_ctx_t* ctx = &lorawan_alcsync_ctx[*service_id];
    memset( ctx, 0, sizeof( lorawan_alcsync_ctx_t ) );

    *downlink_callback  = lorawan_alcsync_service_downlink_handler;
    *on_launch_callback = lorawan_alcsync_service_on_launch;
    *on_update_callback = lorawan_alcsync_service_on_update;
    *context_callback   = ( void* ) service_id;

    ctx->task_id  = task_id;
    ctx->stack_id = CURRENT_STACK;

    alc_sync_init( ctx );
}

void lorawan_alcsync_service_on_launch( void* service_id )
{
    uint8_t idx = *( ( uint8_t* ) service_id );
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " lorawan_alcsync_service_on_launch service_id %d \n", idx );

    IS_VALID_OBJECT_ID( idx );
    uint32_t time_tmp = alcsync_build_ans_payload( idx );

    // Send the ALC sync payload
    status_lorawan_t send_status = ERRORLORAWAN;
    if( lorawan_alcsync_ctx[idx].tx_payload_index > 0 )
    {
        uint8_t tx_payload_size = 0;
        uint8_t tx_payload[ALC_SYNC_TX_PAYLOAD_SIZE_MAX + 1];  // +1 Device Management opcode if builded

#if defined( ADD_SMTC_CLOUD_DEVICE_MANAGEMENT )
        if( lorawan_alcsync_ctx[idx].use_cloud_dm == true )
        {
            lorawan_alcsync_ctx[idx].fport = cloud_dm_get_dm_port( lorawan_alcsync_ctx[idx].stack_id );
            tx_payload[0]                  = DM_INFO_ALCSYNC;
            memcpy( &tx_payload[1], lorawan_alcsync_ctx[idx].tx_payload, lorawan_alcsync_ctx[idx].tx_payload_index );
            tx_payload_size = lorawan_alcsync_ctx[idx].tx_payload_index + 1;
        }
        else
#endif
        {
            lorawan_alcsync_ctx[idx].fport = ALC_SYNC_DEFAULT_PORT;
            memcpy( &tx_payload[0], lorawan_alcsync_ctx[idx].tx_payload, lorawan_alcsync_ctx[idx].tx_payload_index );
            tx_payload_size = lorawan_alcsync_ctx[idx].tx_payload_index;
        }

        send_status = tx_protocol_manager_request( TX_PROTOCOL_TRANSMIT_LORA_AT_TIME, lorawan_alcsync_ctx[idx].fport,
                                                   true, tx_payload, tx_payload_size, UNCONF_DATA_UP, time_tmp * 1000,
                                                   lorawan_alcsync_ctx[idx].stack_id );

        if( send_status == OKLORAWAN )
        {
            if( lorawan_alcsync_ctx[idx].nb_transmission > 0 )
            {
                lorawan_alcsync_ctx[idx].nb_transmission--;
            }
            lorawan_alcsync_ctx[idx].ans_required = false;
        }
    }
}

void lorawan_alcsync_service_on_update( void* service_id )
{
    uint8_t idx = *( ( uint8_t* ) service_id );
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " lorawan_alcsync_service_on_update service_id %d \n", idx );
    IS_VALID_OBJECT_ID( idx );

    smodem_task task_alc_sync = { 0 };
    task_alc_sync.id          = lorawan_alcsync_ctx[idx].task_id;
    task_alc_sync.priority    = TASK_MEDIUM_HIGH_PRIORITY;
    task_alc_sync.stack_id    = lorawan_alcsync_ctx[idx].stack_id;

    if( lorawan_alcsync_ctx[idx].enabled == true )
    {
        if( ( lorawan_alcsync_ctx[idx].nb_transmission > 0 ) ||
            ( ( int32_t ) ( smtc_modem_hal_get_time_in_s( ) - timestamp_launch[idx] -
                            lorawan_alcsync_ctx[idx].periodicity_s + 30 ) >= 0 ) )  // +30 because period +/-30s
        {
            lorawan_alcsync_ctx[idx].req_status |= ( 1 << ALC_SYNC_APP_TIME_REQ );
        }

        if( lorawan_alcsync_ctx[idx].req_status != 0 )
        {
            // Relaunch with rapid delay
            task_alc_sync.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + ALC_SYNC_ANSWER_DELAY_S;
        }
        else
        {
            // Follow the normal periodicity to relaunch the service
            task_alc_sync.time_to_execute_s =
                timestamp_launch[idx] + lorawan_alcsync_ctx[idx].periodicity_s +
                alcsync_get_signed_random_nb_in_range( ( -25 - ALC_SYNC_DELAY_BEFORE_SEND_S ),
                                                       ( 25 - ALC_SYNC_DELAY_BEFORE_SEND_S ) );
        }

        modem_supervisor_add_task( &task_alc_sync );

        if( lorawan_alcsync_ctx[idx].event_time_sync == true )
        {
            lorawan_alcsync_ctx[idx].event_time_sync = false;
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_ALCSYNC_TIME, 0, lorawan_alcsync_ctx[idx].stack_id );
        }
    }
}

uint8_t lorawan_alcsync_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    // Message must be received only on unicast windows
    if( is_received_on_multicast_window( rx_down_data->rx_metadata.rx_window ) == true )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    uint8_t stack_id = rx_down_data->stack_id;

    uint8_t                service_id;
    lorawan_alcsync_ctx_t* ctx = alc_sync_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( stack_id >= NUMBER_OF_ALCSYNC_OBJ )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "%s: stack id not valid %u \n", __func__, stack_id );
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( ctx->enabled != true )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( rx_down_data->rx_metadata.is_a_join_accept == true )
    {
        lorawan_alcsync_request_sync( stack_id, false );
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( rx_down_data->rx_metadata.rx_fport_present == true )
    {
        uint8_t* rx_payload_ptr      = NULL;
        uint8_t  rx_payload_ptr_size = 0;

#if defined( ADD_SMTC_CLOUD_DEVICE_MANAGEMENT )
        if( ( ctx->use_cloud_dm == true ) &&
            ( rx_down_data->rx_metadata.rx_fport == cloud_dm_get_dm_port( stack_id ) ) &&
            ( rx_down_data->rx_payload[2] == DM_ALC_SYNC ) )  // remove up_count/up_delay
        {
            rx_payload_ptr      = &rx_down_data->rx_payload[3];  // remove up_count/up_delay and DM_ALC_SYNC opcode
            rx_payload_ptr_size = rx_down_data->rx_payload_size - 3;
        }
        else
#endif
            if( ( ctx->use_cloud_dm == false ) && ( rx_down_data->rx_metadata.rx_fport == ctx->fport ) )
        {
            rx_payload_ptr      = &rx_down_data->rx_payload[0];
            rx_payload_ptr_size = rx_down_data->rx_payload_size;
        }
        else
        {
            return MODEM_DOWNLINK_UNCONSUMED;
        }

        SMTC_MODEM_HAL_TRACE_PRINTF( "lorawan_alcsync_service_downlink_handler\n" );
        alc_sync_parser( ctx, rx_payload_ptr, rx_payload_ptr_size, rx_down_data->rx_metadata.timestamp_ms );

        task_id_t current_task_id = modem_supervisor_get_task( )->next_task_id;

        if( current_task_id != ctx->task_id )
        {
            // To generate event in case receive downlink when another service is running
            lorawan_alcsync_service_on_update( &service_id );
        }
        return MODEM_DOWNLINK_CONSUMED;
    }

    return MODEM_DOWNLINK_UNCONSUMED;
}
bool lorawan_alcsync_mpa_injector( uint8_t stack_id, uint8_t* payload_in, uint8_t* nb_bytes_read_payload_in,
                                   receive_win_t rx_window, uint8_t* payload_out, uint8_t* payload_out_length,
                                   const uint8_t max_payload_out_length, uint32_t rx_timestamp_ms )

{
    if( is_received_on_multicast_window( rx_window ) == true )
    {
        *payload_out_length = 0;
        return false;
    }
    uint8_t                service_id;
    lorawan_alcsync_ctx_t* ctx = alc_sync_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
    {
        return false;
    }

    if( stack_id >= NUMBER_OF_ALCSYNC_OBJ )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "%s: stack id not valid %u \n", __func__, stack_id );
        return false;
    }

    if( ctx->enabled != true )
    {
        return false;
    }

    uint8_t cmd_id = payload_in[0];
    if( cmd_id >= ALC_SYNC_NB_CMD_REQ )
    {
        return false;
    }
    *nb_bytes_read_payload_in = alcsync_req_cmd_size[cmd_id];

    SMTC_MODEM_HAL_TRACE_WARNING( "ALCSYNC CiD 0x%02x (byte read %u)\n", cmd_id, *nb_bytes_read_payload_in );
    alc_sync_parser( ctx, payload_in, alcsync_req_cmd_size[cmd_id], rx_timestamp_ms );
    if( ctx->event_time_sync == true )
    {
        ctx->event_time_sync = false;
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_ALCSYNC_TIME, 0, stack_id );
    }

    if( ctx->req_status > 0 )
    {
        alcsync_build_ans_payload( service_id );
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( ctx->tx_payload_index <= max_payload_out_length );

        *payload_out_length = ctx->tx_payload_index;
        memcpy( payload_out, ctx->tx_payload, ctx->tx_payload_index );
        return true;
    }
    else
    {
        return false;
    }
}

void lorawan_alcsync_set_enabled( uint8_t stack_id, bool enabled )
{
    IS_VALID_STACK_ID( stack_id );

    uint8_t                service_id;
    lorawan_alcsync_ctx_t* ctx = alc_sync_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
    {
        return;
    }

    if( ctx->enabled != enabled )
    {
        ctx->enabled = enabled;

        if( enabled == true )
        {
            lorawan_alcsync_request_sync( stack_id, false );
        }
        else
        {
            modem_supervisor_remove_task( ctx->task_id );
        }
    }
}

alc_sync_ret_t lorawan_alcsync_config_service( uint8_t stack_id, bool use_cloud_dm_alcsync )
{
    IS_VALID_STACK_ID( stack_id );

    uint8_t                service_id;
    lorawan_alcsync_ctx_t* ctx = alc_sync_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
    {
        return ALC_SYNC_FAIL;
    }

    if( ctx->enabled == true )
    {
        return ALC_SYNC_FAIL;
    }

    ctx->use_cloud_dm = use_cloud_dm_alcsync;

    return ALC_SYNC_OK;
}

alc_sync_ret_t lorawan_alcsync_request_sync( uint8_t stack_id, bool ans_required )
{
    IS_VALID_STACK_ID( stack_id );
    if( lorawan_alcsync_ctx[stack_id].enabled != true )
    {
        return ALC_SYNC_FAIL;
    }

    uint8_t                service_id;
    lorawan_alcsync_ctx_t* ctx = alc_sync_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
    {
        return ALC_SYNC_FAIL;
    }

    ctx->ans_required = ans_required;
    ctx->req_status |= ( 1 << ALC_SYNC_APP_TIME_REQ );

    smodem_task task_alcsync       = { 0 };
    task_alcsync.id                = ctx->task_id;
    task_alcsync.stack_id          = ctx->stack_id;
    task_alcsync.priority          = TASK_MEDIUM_HIGH_PRIORITY;
    task_alcsync.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + smtc_modem_hal_get_random_nb_in_range( 1, 5 );

    modem_supervisor_add_task( &task_alcsync );

    return ALC_SYNC_OK;
}

alc_sync_ret_t lorawan_alcsync_get_gps_time_second( uint8_t stack_id, uint32_t* gps_time_s )
{
    IS_VALID_STACK_ID( stack_id );

    uint8_t                service_id;
    lorawan_alcsync_ctx_t* ctx = alc_sync_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
    {
        return ALC_SYNC_FAIL;
    }

    // Never sync
    if( ctx->time_correction_s == 0 )
    {
        *gps_time_s = 0;
        return ALC_SYNC_FAIL;
    }

    *gps_time_s = smtc_modem_hal_get_time_in_s( ) + ctx->time_correction_s;
    return ALC_SYNC_OK;
}

void lorawan_alcsync_service_get_id( uint8_t* pkt_id, uint8_t* pkt_version, uint8_t* pkt_port )

{
    *pkt_id      = ALC_PACKAGE_IDENTIFIER;
    *pkt_version = ALC_PACKAGE_VERSION;
    *pkt_port    = ALC_SYNC_DEFAULT_PORT;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static lorawan_alcsync_ctx_t* alc_sync_get_ctx_from_stack_id( uint8_t stack_id, uint8_t* service_id )
{
    lorawan_alcsync_ctx_t* ctx = NULL;
    for( uint8_t i = 0; i < NUMBER_OF_ALCSYNC_OBJ; i++ )
    {
        if( lorawan_alcsync_ctx[i].stack_id == stack_id )
        {
            ctx         = &lorawan_alcsync_ctx[i];
            *service_id = i;
            break;
        }
    }

    return ctx;
}

static void alc_sync_init( lorawan_alcsync_ctx_t* ctx )
{
    ctx->fport                 = ALC_SYNC_DEFAULT_PORT;
    ctx->periodicity_s         = ALC_SYNC_DEFAULT_REQUEST_PERIOD_S;
    ctx->max_length_up_payload = ALC_SYNC_TX_PAYLOAD_SIZE_MAX;
}

static alc_sync_ret_t alc_sync_parser( lorawan_alcsync_ctx_t* ctx, uint8_t* alc_sync_rx_buffer,
                                       uint8_t alc_sync_rx_buffer_length, uint32_t timestamp_ms )
{
    alc_sync_ret_t ret                      = ALC_SYNC_OK;
    uint8_t        alc_sync_rx_buffer_index = 0;
    ctx->req_status                         = 0;

    while( alc_sync_rx_buffer_length > alc_sync_rx_buffer_index )
    {
        switch( alc_sync_rx_buffer[alc_sync_rx_buffer_index] )
        {
        case ALC_SYNC_PACKAGE_VERSION_REQ:
            if( ( alc_sync_rx_buffer_index + ALC_SYNC_PACKAGE_VERSION_REQ_SIZE ) <= alc_sync_rx_buffer_length )
            {
                ctx->req_status |= ( 1 << ALC_SYNC_PACKAGE_VERSION_REQ );
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "%s\n", alc_sync_bad_size_str );
                ret = ALC_SYNC_FAIL;
            }
            alc_sync_rx_buffer_index += ALC_SYNC_PACKAGE_VERSION_REQ_SIZE;
            break;

        case ALC_SYNC_APP_TIME_ANS:
            if( ( alc_sync_rx_buffer_index + ALC_SYNC_APP_TIME_ANS_SIZE ) <= alc_sync_rx_buffer_length )
            {
                if( alc_sync_decode_app_time_ans( ctx, &alc_sync_rx_buffer[alc_sync_rx_buffer_index + 1] ) ==
                    ALC_SYNC_OK )
                {
                    // The end - device stops re-transmissions of the AppTimeReq if a valid
                    // AppTimeAns is received
                    ctx->nb_transmission = 0;

                    // Timestamp the last received data
                    ctx->timestamp_last_correction_s = timestamp_ms / 1000;

                    ctx->event_time_sync = true;

                    SMTC_MODEM_HAL_TRACE_INFO( "ALC Sync time correction %d s -> new GPS Time: %d s\n",
                                               ctx->time_correction_s,
                                               ctx->timestamp_last_correction_s + ctx->time_correction_s );
                }
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "%s\n", alc_sync_bad_size_str );
                ret = ALC_SYNC_FAIL;
            }
            alc_sync_rx_buffer_index += ALC_SYNC_APP_TIME_ANS_SIZE;
            break;

        case ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ:
            if( ( alc_sync_rx_buffer_index + ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ_SIZE ) <=
                alc_sync_rx_buffer_length )
            {
                alc_sync_decode_device_app_time_periodicity_req( ctx,
                                                                 &alc_sync_rx_buffer[alc_sync_rx_buffer_index + 1] );
                ctx->req_status |= ( 1 << ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ );
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "%s\n", alc_sync_bad_size_str );
                ret = ALC_SYNC_FAIL;
            }
            alc_sync_rx_buffer_index += ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ_SIZE;
            break;

        case ALC_SYNC_FORCE_DEVICE_RESYNC_REQ:
            if( ( alc_sync_rx_buffer_index + ALC_SYNC_FORCE_DEVICE_RESYNC_REQ_SIZE ) <= alc_sync_rx_buffer_length )
            {
                alc_sync_decode_force_device_resync_req( ctx, &alc_sync_rx_buffer[alc_sync_rx_buffer_index + 1] );
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "%s\n", alc_sync_bad_size_str );
                ret = ALC_SYNC_FAIL;
            }
            alc_sync_rx_buffer_index += ALC_SYNC_FORCE_DEVICE_RESYNC_REQ_SIZE;
            break;

        default:
            SMTC_MODEM_HAL_TRACE_ERROR( "%s Illegal state\n ", __func__ );
            alc_sync_rx_buffer_length = 0;
            ret                       = ALC_SYNC_FAIL;
            break;
        }
    }
    return ret;
}

static void alc_sync_construct_package_version_answer( lorawan_alcsync_ctx_t* ctx )
{
    if( is_alc_sync_tx_buffer_not_full( ctx, ALC_SYNC_PACKAGE_VERSION_ANS_SIZE ) == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "ctx->tx_payload buffer is full\n" );
        return;
    }

    ctx->req_status &= ~( 1 << ALC_SYNC_PACKAGE_VERSION_REQ );

    ctx->tx_payload[ctx->tx_payload_index++] = ALC_SYNC_PACKAGE_VERSION_ANS;
    ctx->tx_payload[ctx->tx_payload_index++] = ALC_PACKAGE_IDENTIFIER;
    ctx->tx_payload[ctx->tx_payload_index++] = ALC_PACKAGE_VERSION;
}

static void alc_sync_construct_app_time_request( lorawan_alcsync_ctx_t* ctx, uint32_t device_time,
                                                 uint8_t ans_required )
{
    if( is_alc_sync_tx_buffer_not_full( ctx, ALC_SYNC_APP_TIME_REQ_SIZE ) == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "ctx->tx_payload buffer is full\n" );
        return;
    }
    ctx->req_status &= ~( 1 << ALC_SYNC_APP_TIME_REQ );

    ctx->tx_payload[ctx->tx_payload_index++] = ALC_SYNC_APP_TIME_REQ;
    ctx->tx_payload[ctx->tx_payload_index++] = device_time & 0xFF;
    ctx->tx_payload[ctx->tx_payload_index++] = ( device_time >> 8 ) & 0xFF;
    ctx->tx_payload[ctx->tx_payload_index++] = ( device_time >> 16 ) & 0xFF;
    ctx->tx_payload[ctx->tx_payload_index++] = ( device_time >> 24 ) & 0xFF;
    ctx->tx_payload[ctx->tx_payload_index++] = ( ( ans_required & 0x01 ) << 4 ) + ( ctx->token_req & 0x0F );
}

static void alc_sync_construct_app_time_periodicity_answer( lorawan_alcsync_ctx_t* ctx, uint8_t status, uint32_t time )
{
    if( is_alc_sync_tx_buffer_not_full( ctx, ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_ANS_SIZE ) == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "ctx->tx_payload buffer is full\n" );
        return;
    }
    ctx->req_status &= ~( 1 << ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ );

    ctx->tx_payload[ctx->tx_payload_index++] = ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_ANS;
    ctx->tx_payload[ctx->tx_payload_index++] = status & 0x01;
    ctx->tx_payload[ctx->tx_payload_index++] = time & 0xFF;
    ctx->tx_payload[ctx->tx_payload_index++] = ( time >> 8 ) & 0xFF;
    ctx->tx_payload[ctx->tx_payload_index++] = ( time >> 16 ) & 0xFF;
    ctx->tx_payload[ctx->tx_payload_index++] = ( time >> 24 ) & 0xFF;
}

static inline bool is_alc_sync_tx_buffer_not_full( lorawan_alcsync_ctx_t* ctx, uint8_t cmd_size )
{
    return ( ( ctx->tx_payload_index + cmd_size ) <= MIN( ALC_SYNC_TX_PAYLOAD_SIZE_MAX, ctx->max_length_up_payload )
                 ? true
                 : false );
}

static alc_sync_ret_t alc_sync_decode_app_time_ans( lorawan_alcsync_ctx_t* ctx, uint8_t* buffer )
{
    if( ctx->token_req == ( buffer[APP_TIME_ANS_TOKEN_BYTE] & 0x0F ) )
    {
        // Check Token Request
        ctx->token_req                = ( ctx->token_req + 1 ) & 0x0F;
        int32_t tmp_time_correction_s = ( buffer[APP_TIME_ANS_TIME_CORRECTION_BYTE] ) +
                                        ( buffer[APP_TIME_ANS_TIME_CORRECTION_BYTE + 1] << 8 ) +
                                        ( buffer[APP_TIME_ANS_TIME_CORRECTION_BYTE + 2] << 16 ) +
                                        ( buffer[APP_TIME_ANS_TIME_CORRECTION_BYTE + 3] << 24 );

        if( ( tmp_time_correction_s == ( int32_t ) 0x7FFFFFFF ) || ( tmp_time_correction_s == ( int32_t ) 0x80000000 ) )
        {
            // A new time correction is required
            ctx->req_status |= ( 1 << ALC_SYNC_APP_TIME_REQ );
        }

        ctx->time_correction_s += tmp_time_correction_s;

        return ALC_SYNC_OK;
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "ALC Sync token mismatch: %d - %d\n", ctx->token_req, ( buffer[4] & 0x0F ) );
        return ALC_SYNC_FAIL;
    }
}

static void alc_sync_decode_device_app_time_periodicity_req( lorawan_alcsync_ctx_t* ctx, uint8_t* buffer )
{
    // The actual periodicity in seconds is 128*2^Period
    ctx->periodicity_s = 128 << ( buffer[0] & 0x0F );
}

static void alc_sync_decode_force_device_resync_req( lorawan_alcsync_ctx_t* ctx, uint8_t* buffer )
{
    ctx->nb_transmission = ( buffer[0] & 0x07 );
}

static int32_t alcsync_get_signed_random_nb_in_range( const int32_t val_1, const int32_t val_2 )
{
    uint32_t tmp_range = 0;  // ( val_1 <= val_2 ) ? ( val_2 - val_1 ) : ( val_1 - val_2 );

    if( val_1 <= val_2 )
    {
        tmp_range = ( val_2 - val_1 );
        return ( int32_t ) ( ( val_1 + smtc_modem_hal_get_random_nb_in_range( 0, tmp_range ) ) );
    }
    else
    {
        tmp_range = ( val_1 - val_2 );
        return ( int32_t ) ( ( val_2 + smtc_modem_hal_get_random_nb_in_range( 0, tmp_range ) ) );
    }
}
static uint32_t alcsync_build_ans_payload( uint8_t idx )
{
    uint32_t time_tmp = smtc_modem_hal_get_time_in_s( ) + ALC_SYNC_DELAY_BEFORE_SEND_S;

    // Reset uplink buffer
    lorawan_alcsync_ctx[idx].tx_payload_index = 0;
    lorawan_alcsync_ctx[idx].max_length_up_payload =
        lorawan_api_next_max_payload_length_get( lorawan_alcsync_ctx[idx].stack_id );

    if( ( ( lorawan_alcsync_ctx[idx].req_status >> ALC_SYNC_PACKAGE_VERSION_REQ ) & 0x1 ) == 1 )
    {
        alc_sync_construct_package_version_answer( &lorawan_alcsync_ctx[idx] );
    }

    if( ( ( lorawan_alcsync_ctx[idx].req_status >> ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ ) & 0x1 ) == 1 )
    {
        timestamp_launch[idx] = time_tmp;
        alc_sync_construct_app_time_periodicity_answer(
            &lorawan_alcsync_ctx[idx], false, timestamp_launch[idx] + lorawan_alcsync_ctx[idx].time_correction_s );
    }

    if( ( ( lorawan_alcsync_ctx[idx].req_status >> ALC_SYNC_APP_TIME_REQ ) & 0x1 ) == 1 )
    {
        timestamp_launch[idx] = time_tmp;
        if( lorawan_alcsync_ctx[idx].nb_transmission > 0 )
        {
            lorawan_alcsync_ctx[idx].ans_required = false;
        }
        alc_sync_construct_app_time_request( &lorawan_alcsync_ctx[idx],
                                             timestamp_launch[idx] + lorawan_alcsync_ctx[idx].time_correction_s,
                                             lorawan_alcsync_ctx[idx].ans_required );
    }
    return time_tmp;
}

static bool is_received_on_multicast_window( receive_win_t rx_window )
{
#if defined( SMTC_MULTICAST )
    if( ( ( rx_window >= RECEIVE_ON_RXC_MC_GRP0 ) && ( rx_window <= RECEIVE_ON_RXC_MC_GRP3 ) ) ||
        ( ( rx_window >= RECEIVE_ON_RXB_MC_GRP0 ) && ( rx_window <= RECEIVE_ON_RXB_MC_GRP3 ) ) )
    {
        return true;
    }
#endif
    return false;
}

/* --- EOF ------------------------------------------------------------------ */
