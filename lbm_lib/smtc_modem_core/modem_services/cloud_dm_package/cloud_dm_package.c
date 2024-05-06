/**
 * @file      cloud_dm_package.c
 *
 * @brief     Device management for Semtech cloud service
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
#include "modem_core.h"
#include "smtc_modem_api.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "modem_supervisor_light.h"
#include "smtc_real.h"
#include "lorawan_api.h"
#include "modem_event_utilities.h"
#include "device_management_defs.h"
#include "cloud_dm_package.h"

#if defined( USE_LR11XX_CE )
#include "lr11xx_system.h"
#endif  // USE_LR11XX_CE

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define CURRENT_STACK ( task_id / NUMBER_OF_TASKS )
#define NUMBER_MAX_OF_CLOUD_DM_OBJ 1  // modify in case of multiple obj

#define MODEM_TASK_DELAY_MS ( smtc_modem_hal_get_random_nb_in_range( 200, 3000 ) )

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
 * @brief Check is the index is valid before accessing ALCSync object
 *
 */
#define IS_VALID_OBJECT_ID( x )                                            \
    do                                                                     \
    {                                                                      \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_MAX_OF_CLOUD_DM_OBJ ); \
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

/**
 * @brief Check is the service is initialized before accessing the object
 *
 */
#define IS_SERVICE_INITIALIZED( x )                                                   \
    do                                                                                \
    {                                                                                 \
        if( cloud_dm_obj[x].initialized == false )                                    \
        {                                                                             \
            SMTC_MODEM_HAL_TRACE_WARNING( "cloud_dm_obj service not initialized\n" ); \
            return;                                                                   \
        }                                                                             \
    } while( 0 )
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#if MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON
static const char* dm_cmd_str[DM_CMD_MAX] = {
    [DM_RESET]       = "RESET",          //
    [DM_FUOTA]       = "FUOTA",          //
    [DM_FILE_DONE]   = "FILE_DONE",      //
    [DM_GET_INFO]    = "GET_INFO",       //
    [DM_SET_CONF]    = "SET_CONF",       //
    [DM_REJOIN]      = "REJOIN",         //
    [DM_MUTE]        = "MUTE",           //
    [DM_SET_DM_INFO] = "SET_DM_INFO",    //
    [DM_STREAM]      = "STREAM",         //
    [DM_ALC_SYNC]    = "ALC_SYNC",       //
    [DM_ALM_UPDATE]  = "ALM_UPDATE",     //
    [DM_ALM_DBG]     = "ALM_DEBUG",      //
    [DM_SOLV_UPDATE] = "SOLVER_UPDATE",  //
    [DM_ALM_FUPDATE] = "ALM_FUPDATE",    //
};
#endif

static const uint8_t dm_cmd_len[DM_CMD_MAX][2] = {
    // CMD              = {min,       max}
    [DM_RESET]       = { 3, 3 },            //
    [DM_FUOTA]       = { 1, 255 },          //
    [DM_FILE_DONE]   = { 1, 1 },            //
    [DM_GET_INFO]    = { 1, 255 },          //
    [DM_SET_CONF]    = { 2, 255 },          //
    [DM_REJOIN]      = { 2, 2 },            //
    [DM_MUTE]        = { 1, 1 },            //
    [DM_SET_DM_INFO] = { 1, DM_INFO_MAX },  //
    [DM_STREAM]      = { 1, 255 },          //
    [DM_ALC_SYNC]    = { 1, 255 },          //
    [DM_ALM_UPDATE]  = { 1, 255 },          //
    [DM_ALM_DBG]     = { 1, 255 },          //
    [DM_SOLV_UPDATE] = { 1, 255 },          //
    [DM_ALM_FUPDATE] = { 1, 255 }           //
};

// DM info field sizes
static const uint8_t dm_info_field_sz[DM_INFO_MAX] = {
    [DM_INFO_STATUS]    = 1,  //
    [DM_INFO_CHARGE]    = 2,  //
    [DM_INFO_VOLTAGE]   = 1,  //
    [DM_INFO_TEMP]      = 1,  //
    [DM_INFO_SIGNAL]    = 2,  //
    [DM_INFO_UPTIME]    = 2,  //
    [DM_INFO_RXTIME]    = 2,  //
    [DM_INFO_FIRMWARE]  = 8,  //
    [DM_INFO_ADRMODE]   = 1,  //
    [DM_INFO_JOINEUI]   = 8,  //
    [DM_INFO_INTERVAL]  = 1,  //
    [DM_INFO_REGION]    = 1,  //
    [DM_INFO_RFU_0]     = 4,  //
    [DM_INFO_CRASHLOG]  = 0,  // (variable-length, send as last field or in separate frame)
    [DM_INFO_UPLOAD]    = 0,  // (variable-length, not sent periodically)
    [DM_INFO_RSTCOUNT]  = 2,  //
    [DM_INFO_DEVEUI]    = 8,  //
    [DM_INFO_RFU_1]     = 2,  //
    [DM_INFO_SESSION]   = 2,  //
    [DM_INFO_CHIPEUI]   = 8,  //
    [DM_INFO_STREAM]    = 0,  // (variable-length, not sent periodically)
    [DM_INFO_STREAMPAR] = 2,  //
    [DM_INFO_APPSTATUS] = 8,  //
    [DM_INFO_ALCSYNC]   = 0,  // (variable-length, not sent periodically)
    [DM_INFO_ALMSTATUS] = 7
};

#define DEFAULT_DM_REPORTING_INTERVAL 0x81  // 1h
#define DEFAULT_DM_REPORTING_FIELDS 0x7B    // status, charge, temp, signal, uptime, rxtime

#define REQ_EVENT_DM_SET_CONF_BIT 0x01
#define REQ_EVENT_MUTE_BIT 0x02
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Defined the type of reset that can be requested by downlink
 *
 * @enum dm_reset_code_t
 */
typedef enum dm_reset_code_e
{
    DM_RESET_MODEM   = 0x01,  //!< Reset the modem
    DM_RESET_APP_MCU = 0x02,  //!< Reset the MCU
    DM_RESET_BOTH    = 0x03,  //!< Reset modem and app MCU
    DM_RESET_MAX              //!< number of elements
} dm_reset_code_t;

/**
 * @brief DM interval unit definitions
 *
 * @enum dm_interval_unit_t
 */
typedef enum dm_interval_unit_e
{
    DM_INTERVAL_UNIT_SEC  = 0,  //!< Interval in second(s)
    DM_INTERVAL_UNIT_DAY  = 1,  //!< Interval in day(s)
    DM_INTERVAL_UNIT_HOUR = 2,  //!< Interval in hour(s)
    DM_INTERVAL_UNIT_MIN  = 3   //!< Interval in minute(s)
} dm_interval_unit_t;

/**
 * @brief Downlink Message Format.
 *        The cloud service might return one or more request messages which have to be delivered over
 *        the network back to the modem on the device management port.
 *        All downlink messages have the following format.
 *
 * @remark  Next to the request code and data each message contains an upcount field which indicates the
 *          number of uplinks to generate.
 *          These uplinks can be used to create additional downlink opportunities and should be generated
 *          at the rate specified by the updelay field.
 *          The reception of a new request message resets the uplink generation.
 *
 * @struct dm_cmd_msg_t
 */
typedef struct dm_cmd_msg_s
{
    dm_opcode_t request_code;  //!< request code
    uint8_t*    buffer;        //!< request data
    uint8_t     buffer_len;    //!< request data length in byte(s)
} dm_cmd_msg_t;

/**
 * @brief LoRaWAN template Object
 *
 * @struct cloud_dm_s
 *
 */
typedef struct cloud_dm_s
{
    uint8_t stack_id;
    uint8_t task_id;
    bool    enabled;
    bool    initialized;

    uint8_t  dm_port;
    uint8_t  dm_interval;
    uint32_t dm_info_bitfield_periodic;
    uint32_t dm_info_bitfield_now;
    uint8_t  next_dm_opcode_periodic;
    uint8_t  next_dm_opcode_now;
    bool     is_first_dm_after_join;
    bool     is_pending_dm_status_payload_periodic;
    bool     is_pending_dm_status_payload_now;
    uint8_t  number_of_muted_day;
    uint32_t mute_timestamp_s;
    uint32_t dm_periodic_timestamp_s;
    uint8_t  up_count;  //!< uplink count
    uint8_t  up_delay;  //!< uplink delay [s]
    uint8_t  modem_user_app_status[8];
    int16_t  lorawan_last_snr_get;
    int16_t  lorawan_last_rssi_get;
    uint32_t last_dl_timestamp_s;

    uint8_t         dm_event_requested_bitfield;
    dm_info_field_t last_dm_set_conf_opcode_requested;

} cloud_dm_t;

static cloud_dm_t cloud_dm_obj[NUMBER_MAX_OF_CLOUD_DM_OBJ];
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief   Enqueue task in supervisor to report a periodic DM status message
 *
 * @param [in] ctx                  stream object context
 * @param [in] delay_to_execute_s   duration before the next execution of this task
 */
static void cloud_dm_status_add_task( cloud_dm_t* ctx, uint32_t delay_to_execute_s );

/**
 * @brief Callback called at task launch
 *
 * @param context_callback
 */
static void cloud_dm_service_on_launch( void* context );

/**
 * @brief Callback called at the end of the task
 *
 * @param context_callback
 */
static void cloud_dm_service_on_update( void* context );

/**
 * @brief Callback to handle the downlink received by the LoRaWAN layer
 *
 * @param rx_down_data
 */
static uint8_t cloud_dm_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

static void memcpy1_r( uint8_t* dst, const uint8_t* src, uint16_t size );

/**
 * @brief Get the service context linked to a stack_id
 *
 * @param [in]  stack_id                Stack identifier
 * @param [out] service_id              associated occurrence of this service
 * @return cloud_dm_t*                  Cloud dm object context
 */
static cloud_dm_t* cloud_dm_get_ctx_from_stack_id( uint8_t stack_id, uint8_t* service_id );

/*!
 * @brief get modem dm interval in second
 * @remark
 *
 * @param [in] ctx *                    Cloud DM context
 * @retval   uint32_t                   Return the DM interval in seconds
 */
static uint32_t cloud_dm_get_dm_interval_second( cloud_dm_t* ctx );

/**
 * @brief Get the number of pending muted days
 *
 * @param [in] ctx *                    Cloud DM context
 * @return int32_t                      Number of second before to be un-muted modem
 */
static int32_t cloud_dm_get_number_pending_muted_s( cloud_dm_t* ctx );

/*!
 * @brief   check DM cmd Size
 *
 * @param [in]  cmd                     Current dm code that must be checked
 * @param [in]  length                  Length of the tested requested code
 * @param [out] dm_cmd_length_valid_t   Return valid length or not
 */
static dm_cmd_length_valid_t dm_check_cmd_size( dm_opcode_t cmd, uint8_t length );

/*!
 * @brief   Handle DM command received
 *
 * @param [in]  ctx *                   Cloud DM context
 * @param [in]  cmd_input *             Command received
 */
static void dm_parse_cmd( cloud_dm_t* ctx, dm_cmd_msg_t* cmd_input );

/*!
 * @brief   DM Reset
 *
 * @param [in]  ctx *                   Cloud DM context
 * @param [in]  reset_code              Type of requested reset
 * @param [in]  reset_session           reset Session is compare to the current number of modem reset
 * @param [out] dm_rc_t                 Return valid or not
 */
static dm_rc_t dm_reset( cloud_dm_t* ctx, dm_reset_code_t reset_code, uint16_t reset_session );

/*!
 * @brief   DM SetConf
 *
 * @param [in]  ctx *                   Cloud DM context
 * @param [in]  tag                     dm_info_field_t that will be handle
 * @param [in]  data *                  Current dm info code that must be checked
 * @param [in]  length                  Length of the tested requested dm info code+data
 * @retval dm_rc_t                      Return valid or not
 */
static dm_rc_t dm_set_conf( cloud_dm_t* ctx, dm_info_field_t tag, uint8_t* data, uint8_t length );

/*!
 * @brief   check DM Info cmd Size
 *
 * @param   [in]  cmd                   Current dm info code that must be checked
 * @param   [in]  length                Length of the tested requested dm info code
 * @retval dm_cmd_length_valid_t        Return valid length or not
 */
static dm_cmd_length_valid_t dm_check_dminfo_size( dm_info_field_t cmd, uint8_t length );

/*!
 * @brief   convert requested DM bytes fields to bitfield
 *
 * @param [in]  requested_info_list     Array of bytes with requested DM code in each bytes
 * @param [in]  len                     Number of byte that composed requested_info_list
 * @param [in]  bitfields *             Returned bitfield
 * @return     void
 */
static void convert_requested_dm_info_bytes_to_bitfield( const uint8_t* requested_info_list, uint8_t len,
                                                         uint32_t* bitfields );

/**
 * @brief convert a bitfield DM to an array of opcode
 *
 * @param [in] dm_info_bitfield
 * @param [out] dm_array
 * @return uint8_t size of the array in byte
 */
static uint8_t convert_dm_bitfield_to_bytes( uint32_t dm_info_bitfield, uint8_t* dm_array );

/*!
 * @brief   Uplink DM status messages
 *
 * @param [in]    ctx *                   Cloud DM context
 * @param [in]    stack_id                    - Stack identifier
 * @param [out]   dm_uplink_message *         - Returned array that contains one or more concatenated device
 *                                              information fields.
 * @param [out]   dm_uplink_message_len *     - Returned array length
 * @param [in]    max_size                    - max payload size that must be returned
 * @param [in]    dm_info_bitfield            - bitfield with requested reporting data
 * @param [inout] dm_opcode *                 - report data in requested bitfield from this bit
 * @retval bool                               - Return true if there are pending message(s) else false
 */
static bool dm_status_payload( cloud_dm_t* ctx, uint8_t stack_id, uint8_t* dm_uplink_message,
                               uint8_t* dm_uplink_message_len, uint8_t max_size, uint32_t dm_info_bitfield,
                               uint8_t* dm_opcode );

/*!
 * @brief   Check if the biggest requested DM status field can be inserted
 *          in the payload in regard of the max payload size requested
 *
 * @param [in]  info_requested              Requested bitfield
 * @param [in]  max_size                    Max size of the payload
 * @param [out] dm_cmd_length_valid_t       Return valid or not
 */
static dm_cmd_length_valid_t check_dm_status_max_size( uint32_t info_requested, uint8_t max_size );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void cloud_dm_services_init( uint8_t* service_id, uint8_t task_id,
                             uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                             void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                             void** context_callback )
{
    IS_VALID_OBJECT_ID( *service_id );

    cloud_dm_t* ctx = &cloud_dm_obj[*service_id];
    memset( ctx, 0, sizeof( cloud_dm_t ) );

    *downlink_callback  = cloud_dm_service_downlink_handler;
    *on_launch_callback = cloud_dm_service_on_launch;
    *on_update_callback = cloud_dm_service_on_update;
    *context_callback   = ( void* ) service_id;

    ctx->task_id     = task_id;
    ctx->stack_id    = CURRENT_STACK;
    ctx->enabled     = false;
    ctx->initialized = true;

    ctx->dm_port                               = DM_PORT;
    ctx->dm_interval                           = DEFAULT_DM_REPORTING_INTERVAL;
    ctx->dm_info_bitfield_periodic             = DEFAULT_DM_REPORTING_FIELDS;  // context for periodic GetInfo
    ctx->dm_info_bitfield_now                  = 0x00;                         // User GetInfo
    ctx->next_dm_opcode_periodic               = 0;
    ctx->next_dm_opcode_now                    = 0;
    ctx->number_of_muted_day                   = 0;
    ctx->is_first_dm_after_join                = true;
    ctx->is_pending_dm_status_payload_periodic = false;
    ctx->is_pending_dm_status_payload_now      = false;
}

void cloud_dm_services_enable( uint8_t stack_id, bool enabled )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t     service_id;
    cloud_dm_t* ctx = cloud_dm_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
    {
        return;
    }

    if( ctx->enabled != enabled )
    {
        ctx->enabled = enabled;

        if( enabled == true )
        {
            ctx->is_first_dm_after_join = true;
            if( lorawan_api_isjoined( stack_id ) == JOINED )
            {
                cloud_dm_status_add_task( ctx, smtc_modem_hal_get_random_nb_in_range( 10, 15 ) );
            }
        }
        else
        {
            modem_supervisor_remove_task( ctx->task_id );
        }
    }
}

dm_rc_t cloud_dm_set_dm_interval( uint8_t stack_id, uint8_t interval )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t     service_id;
    cloud_dm_t* ctx = cloud_dm_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
    {
        return DM_ERROR;
    }

    ctx->dm_interval = interval;
    cloud_dm_status_add_task( ctx, MODEM_TASK_DELAY_MS / 1000 );

    return DM_OK;
}

uint8_t cloud_dm_get_dm_interval( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t     service_id;
    cloud_dm_t* ctx = cloud_dm_get_ctx_from_stack_id( stack_id, &service_id );
    if( ctx == NULL )
    {
        SMTC_MODEM_HAL_PANIC( );
    }
    return ctx->dm_interval;
}

void cloud_dm_set_modem_user_app_status( uint8_t stack_id, const uint8_t* app_status )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t     service_id;
    cloud_dm_t* ctx = cloud_dm_get_ctx_from_stack_id( stack_id, &service_id );
    if( ctx == NULL )
    {
        SMTC_MODEM_HAL_PANIC( );
    }
    memcpy( ctx->modem_user_app_status, app_status, dm_info_field_sz[DM_INFO_APPSTATUS] );
}

void cloud_dm_get_modem_user_app_status( uint8_t stack_id, uint8_t* app_status )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t     service_id;
    cloud_dm_t* ctx = cloud_dm_get_ctx_from_stack_id( stack_id, &service_id );
    if( ctx == NULL )
    {
        SMTC_MODEM_HAL_PANIC( );
    }
    memcpy( app_status, ctx->modem_user_app_status, dm_info_field_sz[DM_INFO_APPSTATUS] );
}

dm_rc_t cloud_dm_set_dm_port( uint8_t stack_id, uint8_t port )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t     service_id;
    cloud_dm_t* ctx = cloud_dm_get_ctx_from_stack_id( stack_id, &service_id );
    if( ctx == NULL )
    {
        return DM_ERROR;
    }
    if( ( port == 0 ) || ( port >= 224 ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "modem port invalid\n" );
        return DM_ERROR;
    }
    else
    {
        if( ctx->dm_port != port )
        {
            ctx->dm_port = port;
            // modem_store_context( );  // TODO do we still store context ?
        }
        return DM_OK;
    }
}

uint8_t cloud_dm_get_dm_port( uint8_t stack_id )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t     service_id;
    cloud_dm_t* ctx = cloud_dm_get_ctx_from_stack_id( stack_id, &service_id );
    if( ctx == NULL )
    {
        return DM_ERROR;
    }
    return ctx->dm_port;
}

dm_rc_t cloud_dm_get_info_field( uint8_t stack_id, uint8_t* dm_fields_payload, uint8_t* dm_field_length,
                                 dm_info_rate_t rate )
{
    IS_VALID_STACK_ID( stack_id );
    uint8_t     service_id;
    cloud_dm_t* ctx = cloud_dm_get_ctx_from_stack_id( stack_id, &service_id );
    if( ctx == NULL )
    {
        return DM_ERROR;
    }
    uint32_t info_req_bitfield;

    if( rate == DM_INFO_NOW )
    {
        info_req_bitfield = ctx->dm_info_bitfield_now;
    }
    else
    {
        info_req_bitfield = ctx->dm_info_bitfield_periodic;
    }

    *dm_field_length = convert_dm_bitfield_to_bytes( info_req_bitfield, dm_fields_payload );
    return DM_OK;
}

dm_rc_t cloud_dm_set_info_field( uint8_t stack_id, const uint8_t* requested_info_list, uint8_t len,
                                 dm_info_rate_t flag )
{
    IS_VALID_STACK_ID( stack_id );
    dm_rc_t     ret      = DM_OK;
    uint32_t    info_req = 0;
    uint8_t     service_id;
    cloud_dm_t* ctx = cloud_dm_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
    {
        return DM_ERROR;
    }

    for( uint8_t i = 0; i < len; i++ )
    {
        // Ignore DM status with variable length and forbidden fields
        if( ( requested_info_list[i] == DM_INFO_UPLOAD ) || ( requested_info_list[i] == DM_INFO_STREAM ) ||
            ( requested_info_list[i] == DM_INFO_ALCSYNC ) || ( requested_info_list[i] == DM_INFO_DBGRSP ) ||
            ( requested_info_list[i] == DM_INFO_GNSSLOC ) || ( requested_info_list[i] == DM_INFO_WIFILOC ) ||
            ( requested_info_list[i] == DM_INFO_RFU_0 ) || ( requested_info_list[i] == DM_INFO_RFU_1 ) ||
            ( requested_info_list[i] >= DM_INFO_MAX ) )
        {
            ret = DM_ERROR;
            SMTC_MODEM_HAL_TRACE_ERROR( "invalid DM info code (0x%02x)\n", requested_info_list[i] );
        }
        // the almanac update is now handled in a dedicated services, reject any attempt to add the opcode
        if( requested_info_list[i] == DM_INFO_ALMSTATUS )
        {
            ret = DM_ERROR;
        }
    }

    if( ret == DM_OK )
    {
        convert_requested_dm_info_bytes_to_bitfield( requested_info_list, len, &info_req );
        if( flag == DM_INFO_NOW )
        {
            ctx->dm_info_bitfield_now = info_req;
            ctx->next_dm_opcode_now = 0;  // Reset tag_number used by dm_status_payload to start a report from beginning
            ctx->is_pending_dm_status_payload_now = true;
            cloud_dm_status_add_task( ctx, MODEM_TASK_DELAY_MS / 1000 );
        }
        else
        {
            for( uint8_t i = 0; i < len; i++ )
            {
                if( requested_info_list[i] == DM_INFO_CRASHLOG )
                {
                    ret = DM_ERROR;
                }
            }
            if( ret == DM_OK )
            {
                if( ctx->dm_info_bitfield_periodic != info_req )
                {
                    ctx->dm_info_bitfield_periodic = info_req;
                    ctx->next_dm_opcode_periodic   = 0;  // Reset tag_number used by dm_status_payload to start
                                                         // a report from beginning
                }
            }
        }
    }

    return ret;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void cloud_dm_service_on_launch( void* service_id )
{
    uint8_t idx = *( ( uint8_t* ) service_id );
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " %s service_id %d \n", __func__, idx );

    IS_SERVICE_INITIALIZED( idx );
    uint8_t stack_id = cloud_dm_obj[idx].stack_id;

    // If not join -> exit
    if( lorawan_api_isjoined( stack_id ) != JOINED )
    {
        return;
    }

    // If service not enabled -> exit
    if( cloud_dm_obj[idx].enabled == false )
    {
        return;
    }

    // If no pending DM frame or periodicity is 0 -> exit
    if( ( cloud_dm_obj[idx].dm_interval == 0 ) && ( cloud_dm_obj[idx].is_pending_dm_status_payload_now == false ) &&
        ( cloud_dm_obj[idx].up_count == 0 ) )
    {
        return;
    }
    uint32_t rtc_s = smtc_modem_hal_get_time_in_s( );

    uint8_t max_payload = lorawan_api_next_max_payload_length_get( stack_id );
    uint8_t payload[242];
    uint8_t payload_length = 0;

    if( ( cloud_dm_obj[idx].dm_interval != 0 ) && ( cloud_dm_obj[idx].is_first_dm_after_join == true ) )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " info bit field = %x\n", cloud_dm_obj[idx].dm_info_bitfield_periodic );

        uint32_t info_bitfield_periodic_join = cloud_dm_obj[idx].dm_info_bitfield_periodic;
        info_bitfield_periodic_join |= ( 1 << DM_INFO_RSTCOUNT );
        info_bitfield_periodic_join |= ( 1 << DM_INFO_SESSION );
        info_bitfield_periodic_join |= ( 1 << DM_INFO_FIRMWARE );

        // If there is an available crash log
        if( smtc_modem_hal_crashlog_get_status( ) == true )
        {
            cloud_dm_obj[idx].dm_info_bitfield_now |= ( 1 << DM_INFO_CRASHLOG );
            cloud_dm_obj[idx].is_pending_dm_status_payload_now = true;
        }

        SMTC_MODEM_HAL_TRACE_PRINTF( " info bit field join = %x\n", info_bitfield_periodic_join );

        bool dm_periodic_pending =
            dm_status_payload( &cloud_dm_obj[idx], stack_id, payload, &payload_length, max_payload,
                               info_bitfield_periodic_join, &cloud_dm_obj[idx].next_dm_opcode_periodic );

        status_lorawan_t send_status =
            tx_protocol_manager_request (TX_PROTOCOL_TRANSMIT_LORA, cloud_dm_obj[idx].dm_port, true, payload, payload_length, UNCONF_DATA_UP,
                                      smtc_modem_hal_get_time_in_ms( )  , stack_id );

        if( send_status == OKLORAWAN )
        {
            cloud_dm_obj[idx].is_pending_dm_status_payload_periodic = dm_periodic_pending;
            if( dm_periodic_pending == false )
            {
                cloud_dm_obj[idx].is_first_dm_after_join  = false;
                cloud_dm_obj[idx].dm_periodic_timestamp_s = rtc_s;
            }
            SMTC_MODEM_HAL_TRACE_ARRAY( "join payload DM ", payload, payload_length );
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "Join Periodic DM can't be send! internal code: %x\n", send_status );
        }
    }
    else
    {
        // First check if a crash log must be reported, then the status is cleared
        if( ( ( cloud_dm_obj[idx].dm_info_bitfield_now >> DM_INFO_CRASHLOG ) & 0x01 ) == 0x01 )
        {
            smtc_modem_hal_crashlog_restore( payload, &payload_length );

            // First byte is replaced by the crash log opcode, it shouldn't be a problem to understand the log
            payload[0]     = DM_INFO_CRASHLOG;
            payload_length = MIN( max_payload, payload_length );

            smtc_modem_hal_crashlog_set_status( false );

            // clear crashlog request
            cloud_dm_obj[idx].dm_info_bitfield_now &= ~( 1 << DM_INFO_CRASHLOG );

            // If the immediate DM bitfield is 0, reset the pending state
            if( cloud_dm_obj[idx].dm_info_bitfield_now == 0x00 )
            {
                cloud_dm_obj[idx].is_pending_dm_status_payload_now = false;
            }
            SMTC_MODEM_HAL_TRACE_PRINTF( "DM send crashlog\n" );
        }
        // DM request instant
        else if( cloud_dm_obj[idx].is_pending_dm_status_payload_now == true )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "DM send info immediate\n" );
            cloud_dm_obj[idx].is_pending_dm_status_payload_now =
                dm_status_payload( &cloud_dm_obj[idx], stack_id, payload, &payload_length, max_payload,
                                   cloud_dm_obj[idx].dm_info_bitfield_now, &cloud_dm_obj[idx].next_dm_opcode_now );
        }
        // DM periodic
        else if( ( cloud_dm_get_dm_interval_second( &cloud_dm_obj[idx] ) > 0 ) &&
                 ( cloud_dm_obj[idx].dm_info_bitfield_periodic != 0 ) &&
                 ( ( int32_t ) ( rtc_s - cloud_dm_obj[idx].dm_periodic_timestamp_s -
                                 cloud_dm_get_dm_interval_second( &cloud_dm_obj[idx] ) ) >= 0 ) )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "DM send info periodic\n" );
            cloud_dm_obj[idx].dm_periodic_timestamp_s = rtc_s;

            cloud_dm_obj[idx].is_pending_dm_status_payload_periodic = dm_status_payload(
                &cloud_dm_obj[idx], stack_id, payload, &payload_length, max_payload,
                cloud_dm_obj[idx].dm_info_bitfield_periodic, &cloud_dm_obj[idx].next_dm_opcode_periodic );
        }
        // DL opportunities requested
        else if( cloud_dm_obj[idx].up_count != 0 )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "UL for DL opportunities\n" );
            tx_protocol_manager_request (TX_PROTOCOL_TRANSMIT_LORA, cloud_dm_obj[idx].dm_port, false, NULL, 0, UNCONF_DATA_UP,
                                      smtc_modem_hal_get_time_in_ms( )  , stack_id );
        }
        // else nothing to do

        if( payload_length > 0 )
        {
            status_lorawan_t send_status =
                tx_protocol_manager_request (TX_PROTOCOL_TRANSMIT_LORA, cloud_dm_obj[idx].dm_port, true, payload, payload_length, UNCONF_DATA_UP,
                                          smtc_modem_hal_get_time_in_ms( )  , stack_id );
            if( send_status == OKLORAWAN )
            {
                SMTC_MODEM_HAL_TRACE_ARRAY( "payload DM ", payload, payload_length );
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_WARNING( "DM can't be send! internal code: %x\n", send_status );
            }
        }
    }
}

static void cloud_dm_service_on_update( void* service_id )
{
    uint8_t idx = *( ( uint8_t* ) service_id );
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( " %s service_id %d \n", __func__, idx );

    IS_SERVICE_INITIALIZED( idx );
    uint8_t stack_id = cloud_dm_obj[idx].stack_id;

    if( cloud_dm_obj[idx].enabled == false )
    {
        return;
    }
    if( lorawan_api_isjoined( stack_id ) == JOINED )
    {
        int32_t delay_tmp = 0x7FFFFFFF;
        int32_t tmp;

        // DL opportunities is requested, wake up device at up_delay
        if( cloud_dm_obj[idx].up_count > 0 )
        {
            tmp = cloud_dm_obj[idx].up_delay;
            if( delay_tmp > tmp )
            {
                delay_tmp = tmp;
            }
        }

        // The user/cloud DM status is requested, wake up device at MODEM_TASK_DELAY_MS
        if( cloud_dm_obj[idx].is_pending_dm_status_payload_now == true )
        {
            tmp = MODEM_TASK_DELAY_MS / 1000;
            if( delay_tmp > tmp )
            {
                delay_tmp = tmp;
            }
        }

        // Periodic DM is requested, wake up device at dm_interval_second or faster if just after join network
        if( cloud_dm_get_dm_interval_second( &cloud_dm_obj[idx] ) > 0 )
        {
            if( ( cloud_dm_obj[idx].is_first_dm_after_join == true ) ||
                ( cloud_dm_obj[idx].is_pending_dm_status_payload_periodic == true ) )
            {
                tmp = smtc_modem_hal_get_random_nb_in_range( 10, 15 );
                if( delay_tmp > tmp )
                {
                    delay_tmp = tmp;
                }
            }
            else
            {
                tmp = ( int32_t ) ( cloud_dm_obj[idx].dm_periodic_timestamp_s +
                                    cloud_dm_get_dm_interval_second( &cloud_dm_obj[idx] ) -
                                    smtc_modem_hal_get_time_in_s( ) );
                if( tmp <= 0 )
                {
                    // Next periodic DM is already in past, enqueue the next in MODEM_TASK_DELAY_MS
                    tmp = MODEM_TASK_DELAY_MS / 1000;
                }

                if( delay_tmp > tmp )
                {
                    delay_tmp = tmp;
                }
            }
        }

        if( ( cloud_dm_obj[idx].number_of_muted_day > 0 ) && ( cloud_dm_obj[idx].number_of_muted_day < 255 ) )
        {
            tmp = cloud_dm_get_number_pending_muted_s( &cloud_dm_obj[idx] );
            if( tmp > 0 )
            {
                if( delay_tmp > tmp )
                {
                    delay_tmp = tmp;
                }
            }
            else
            {
                cloud_dm_obj[idx].number_of_muted_day = 0;
                cloud_dm_obj[idx].dm_event_requested_bitfield |= REQ_EVENT_MUTE_BIT;
            }
        }
        else if( cloud_dm_obj[idx].number_of_muted_day == 255 )
        {
            delay_tmp = 0x7FFFFFFF;
        }

        if( delay_tmp != 0x7FFFFFFF )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "Next DM task in %d \n", delay_tmp );
            cloud_dm_status_add_task( &cloud_dm_obj[idx], delay_tmp );
        }
    }

    if( ( cloud_dm_obj[idx].dm_event_requested_bitfield & REQ_EVENT_DM_SET_CONF_BIT ) == REQ_EVENT_DM_SET_CONF_BIT )
    {
        cloud_dm_obj[idx].dm_event_requested_bitfield &= ~REQ_EVENT_DM_SET_CONF_BIT;
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_DM_SET_CONF,
                                          cloud_dm_obj[idx].last_dm_set_conf_opcode_requested,
                                          cloud_dm_obj[idx].stack_id );
    }

    if( ( cloud_dm_obj[idx].dm_event_requested_bitfield & REQ_EVENT_MUTE_BIT ) == REQ_EVENT_MUTE_BIT )
    {
        cloud_dm_obj[idx].dm_event_requested_bitfield &= ~REQ_EVENT_MUTE_BIT;
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_MUTE,
                                          ( cloud_dm_obj[idx].number_of_muted_day == 0 ) ? false : true,
                                          cloud_dm_obj[idx].stack_id );
    }
}

static uint8_t cloud_dm_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data )
{
    uint8_t ret = MODEM_DOWNLINK_UNCONSUMED;

    dm_cmd_msg_t dm_input = { 0 };
    uint8_t      stack_id = rx_down_data->stack_id;
    uint8_t      service_id;
    cloud_dm_t*  ctx = cloud_dm_get_ctx_from_stack_id( stack_id, &service_id );

    if( ctx == NULL )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( ctx->enabled == false )
    {
        return MODEM_DOWNLINK_UNCONSUMED;
    }

    if( rx_down_data->rx_metadata.rx_window != RECEIVE_NONE )
    {
        ctx->lorawan_last_rssi_get = rx_down_data->rx_metadata.rx_rssi;
        ctx->lorawan_last_snr_get  = rx_down_data->rx_metadata.rx_snr;
        ctx->last_dl_timestamp_s   = rx_down_data->rx_metadata.timestamp_ms / 1000;

        if( rx_down_data->rx_metadata.is_a_join_accept == true )
        {
            // Update flag to know it is the first dm after join that shall be sent next
            ctx->is_first_dm_after_join = true;
            cloud_dm_service_on_update( &service_id );
            return MODEM_DOWNLINK_UNCONSUMED;
        }

        if( ( rx_down_data->rx_metadata.rx_fport_present == true ) &&
            ( rx_down_data->rx_metadata.rx_fport == ctx->dm_port ) &&
            ( rx_down_data->rx_payload_size > DM_DOWNLINK_HEADER_LENGTH ) )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "%s\n", __func__ );

            dm_input.request_code = ( dm_opcode_t ) rx_down_data->rx_payload[2];
            dm_input.buffer       = &rx_down_data->rx_payload[3];
            dm_input.buffer_len   = rx_down_data->rx_payload_size - DM_DOWNLINK_HEADER_LENGTH;

            // Almanac is managed by its own service
            // Stream and file upload are also handled by their own services, but we need to take care the
            // up_count/up_delay for them
            if( dm_input.request_code != DM_ALM_UPDATE )
            {
                dm_parse_cmd( ctx, &dm_input );

                // init up_count/up_delay only if opcode is not Almanac, it's managed by the almanac service itself
                ctx->up_count = rx_down_data->rx_payload[0];
                ctx->up_delay = rx_down_data->rx_payload[1];
            }
            ret = MODEM_DOWNLINK_CONSUMED;
        }
    }

    // if the downlink handler is called, an uplink was sent by a service (stream, large file upload, user ..), we can
    // decrement the downlink opportunities counter
    if( ( ctx->up_count > 0 ) || ( ctx->dm_event_requested_bitfield != 0 ) )
    {
        task_id_t current_task_id = modem_supervisor_get_task( )->next_task_id;

        if( current_task_id != ctx->task_id )
        {
            // To generate DL opportunities in case receive downlink when another service is running
            cloud_dm_service_on_update( &service_id );
        }

        // If the downlink is consumed, the up_cout has just been updated and do not need to be decremented
        if( ( ctx->up_count > 0 ) && ( ret == MODEM_DOWNLINK_UNCONSUMED ) )
        {
            ctx->up_count--;
        }
    }

    return ret;
}

static void memcpy1_r( uint8_t* dst, const uint8_t* src, uint16_t size )
{
    const uint8_t* p = src + ( size - 1 );
    while( size-- )
    {
        *dst++ = *p--;
    }
}

static cloud_dm_t* cloud_dm_get_ctx_from_stack_id( uint8_t stack_id, uint8_t* service_id )
{
    cloud_dm_t* ctx = NULL;
    for( uint8_t i = 0; i < NUMBER_MAX_OF_CLOUD_DM_OBJ; i++ )
    {
        if( cloud_dm_obj[i].stack_id == stack_id )
        {
            ctx         = &cloud_dm_obj[i];
            *service_id = i;
            break;
        }
    }

    return ctx;
}

static uint32_t cloud_dm_get_dm_interval_second( cloud_dm_t* ctx )
{
    uint8_t  dm_interval = ctx->dm_interval;
    uint32_t temp        = 0;
    switch( ( dm_interval >> 6 ) & 0x03 )
    {
    case DM_INTERVAL_UNIT_SEC:
        temp = ( dm_interval & 0x3F );
        break;
    case DM_INTERVAL_UNIT_DAY:
        temp = ( dm_interval & 0x3F ) * 3600 * 24;
        break;
    case DM_INTERVAL_UNIT_HOUR:
        temp = ( dm_interval & 0x3F ) * 3600;
        break;
    case DM_INTERVAL_UNIT_MIN:
        temp = ( dm_interval & 0x3F ) * 60;
        break;
    default:  // never reach
        SMTC_MODEM_HAL_PANIC( );
        break;
    }
    return temp;
}

static int32_t cloud_dm_get_number_pending_muted_s( cloud_dm_t* ctx )
{
    if( ctx == NULL )
    {
        return DM_ERROR;
    }

    uint32_t tmp_timestamp_s = ctx->mute_timestamp_s;
    uint32_t rtc_s           = smtc_modem_hal_get_time_in_s( );
    uint32_t delta_t_s;
    uint32_t muted_s = ctx->number_of_muted_day * 86400;

    if( rtc_s >= tmp_timestamp_s )
    {
        delta_t_s = ( rtc_s - tmp_timestamp_s );
    }
    else
    {
        delta_t_s = ( ( uint32_t ) ( ~0 ) - tmp_timestamp_s + rtc_s );
    }

    if( delta_t_s >= muted_s )
    {
        // Not muted
        return 0;
    }
    else
    {
        // pending muted n seconds
        return ( muted_s - delta_t_s );
    }
}

static dm_cmd_length_valid_t dm_check_cmd_size( dm_opcode_t cmd, uint8_t length )
{
    if( cmd >= DM_CMD_MAX )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid DM command %x\n", cmd );
        return DM_CMD_NOT_VALID;
    }
    if( ( length >= dm_cmd_len[cmd][0] ) && ( length <= dm_cmd_len[cmd][1] ) )
    {
        return DM_CMD_LENGTH_VALID;
    }
    SMTC_MODEM_HAL_TRACE_ERROR( "Invalid DM command size:%u (min:%u,max%u)\n", length, dm_cmd_len[cmd][0],
                                dm_cmd_len[cmd][1] );
    return DM_CMD_LENGTH_NOT_VALID;
}

static void dm_parse_cmd( cloud_dm_t* ctx, dm_cmd_msg_t* cmd_input )
{
    if( dm_check_cmd_size( cmd_input->request_code, cmd_input->buffer_len ) != DM_CMD_LENGTH_VALID )
    {
        return;
    }
#if MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON
    SMTC_MODEM_HAL_TRACE_PRINTF( "DM_%s (0x%02x)\n", dm_cmd_str[cmd_input->request_code], cmd_input->request_code );
    SMTC_MODEM_HAL_TRACE_ARRAY( "DM frame", cmd_input->buffer, cmd_input->buffer_len );
#endif

    switch( cmd_input->request_code )
    {
    case DM_RESET:
        if( dm_reset( ctx, ( dm_reset_code_t ) cmd_input->buffer[0],
                      cmd_input->buffer[1] | ( cmd_input->buffer[2] << 8 ) ) != DM_OK )
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "DM invalid reset session\n" );
        }
        break;
    case DM_GET_INFO:
        if( cloud_dm_set_info_field( ctx->stack_id, cmd_input->buffer, cmd_input->buffer_len, DM_INFO_NOW ) != DM_OK )
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "DM invalid get info field\n" );
        }
        break;
    case DM_SET_CONF:
        if( dm_set_conf( ctx, ( dm_info_field_t ) cmd_input->buffer[0], cmd_input->buffer + 1,
                         cmd_input->buffer_len - 1 ) != DM_OK )
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "DM invalid set conf\n" );
        }
        else
        {
            ctx->dm_event_requested_bitfield |= REQ_EVENT_DM_SET_CONF_BIT;
        }
        break;
    case DM_REJOIN: {
        uint16_t dev_nonce       = ( cmd_input->buffer[1] << 8 ) | cmd_input->buffer[0];
        uint16_t lr1mac_devnonce = lorawan_api_devnonce_get( ctx->stack_id );
        if( lr1mac_devnonce != dev_nonce )
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "DM invalid devnonce to rejoin\n" );
            uint8_t dm_fields_payload[1] = { DM_INFO_SESSION };
            cloud_dm_set_info_field( ctx->stack_id, dm_fields_payload, 1, DM_INFO_NOW );
            break;
        }
        // Leave network
        smtc_modem_leave_network( ctx->stack_id );
        // Add a new join task
        lorawan_join_add_task( ctx->stack_id );
        break;
    }
    case DM_MUTE:
        ctx->mute_timestamp_s    = smtc_modem_hal_get_time_in_s( );
        ctx->number_of_muted_day = cmd_input->buffer[0];

        if( ctx->number_of_muted_day > 0 )
        {
            // Mute N days with DM periodic allowed
            modem_supervisor_set_modem_mute_with_priority_parameter( TASK_VERY_HIGH_PRIORITY, ctx->stack_id );
        }
        else
        {
            // Un-mute
            modem_supervisor_set_modem_mute_with_priority_parameter( TASK_LOW_PRIORITY, ctx->stack_id );
        }
        ctx->dm_event_requested_bitfield |= REQ_EVENT_MUTE_BIT;
        break;
    case DM_SET_DM_INFO:
        if( cloud_dm_set_info_field( ctx->stack_id, cmd_input->buffer, cmd_input->buffer_len, DM_INFO_PERIODIC ) !=
            DM_OK )
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "DM invalid set info field\n" );
        }
        break;
    case DM_ALC_SYNC:
    case DM_FILE_DONE:
    case DM_STREAM:
        // ALC sync, File upload and Stream are handled in another service
        break;
    default:
        SMTC_MODEM_HAL_TRACE_WARNING( "DM unknown opcode 0x%x\n", cmd_input->request_code );
        break;
    }
}

static dm_rc_t dm_reset( cloud_dm_t* ctx, dm_reset_code_t reset_code, uint16_t reset_session )
{
    dm_rc_t ret = DM_OK;

    uint32_t nb_reset = modem_get_reset_counter( );

    if( reset_session == nb_reset )
    {
        switch( reset_code )
        {
        case DM_RESET_MODEM:
        case DM_RESET_APP_MCU:
        case DM_RESET_BOTH:
            // lorawan_api_context_save( );  // TODO save context
            smtc_modem_hal_reset_mcu( );
            break;
        default:
            ret = DM_ERROR;
            SMTC_MODEM_HAL_TRACE_ERROR( "invalid DM reset code (0x%02x)\n", reset_code );
            break;
        }
    }
    else
    {
        uint8_t dm_fields_payload[1] = { DM_INFO_RSTCOUNT };
        cloud_dm_set_info_field( ctx->stack_id, dm_fields_payload, 1, DM_INFO_NOW );
        ret = DM_ERROR;
        SMTC_MODEM_HAL_TRACE_ERROR( "invalid DM reset session code\n" );
    }

    return ret;
}

static dm_rc_t dm_set_conf( cloud_dm_t* ctx, dm_info_field_t tag, uint8_t* data, uint8_t length )
{
    dm_rc_t ret = DM_OK;

    if( dm_check_dminfo_size( tag, length ) != DM_CMD_LENGTH_VALID )
    {
        tag = DM_INFO_MAX;
        ret = DM_ERROR;
    }
    else
    {
        switch( tag )
        {
        case DM_INFO_ADRMODE: {
            if( lorawan_api_dr_strategy_set( ( dr_strategy_t ) data[0], ctx->stack_id ) == ERRORLORAWAN )
            {
                ret = DM_ERROR;
            }
            break;
        }
        case DM_INFO_JOINEUI: {
            uint8_t p_tmp[8];
            memcpy1_r( p_tmp, data, 8 );
            lorawan_api_set_joineui( &p_tmp[0], ctx->stack_id );
            break;
        }
        case DM_INFO_INTERVAL: {
            ctx->dm_interval = data[0];
            break;
        }
        case DM_INFO_REGION: {
            smtc_real_region_types_t region_type = data[0];
            if( smtc_real_is_supported_region( region_type ) == SMTC_REAL_STATUS_OK )
            {
                smtc_modem_leave_network( ctx->stack_id );
                lorawan_api_set_region( region_type, ctx->stack_id );
            }
            else
            {
                ret = DM_ERROR;
            }
            break;
        }
        default:
            tag = DM_INFO_MAX;
            ret = DM_ERROR;
            break;
        }
    }

    return ret;
}

static dm_cmd_length_valid_t dm_check_dminfo_size( dm_info_field_t cmd, uint8_t length )
{
    if( cmd >= DM_INFO_MAX )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid DM command\n" );
        return DM_CMD_LENGTH_NOT_VALID;
    }

    if( length != dm_info_field_sz[cmd] )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid DM command size\n" );
        return DM_CMD_LENGTH_NOT_VALID;
    }

    return DM_CMD_LENGTH_VALID;
}

static void convert_requested_dm_info_bytes_to_bitfield( const uint8_t* requested_info_list, uint8_t len,
                                                         uint32_t* bitfields )
{
    // Reset bitfield
    *bitfields = 0;
    for( uint8_t i = 0; i < len; i++ )
    {
#if !defined( ADD_SMTC_STREAM )
        // do not add DM_INFO_STREAMPAR bit if stream not builded
        if( requested_info_list[i] != DM_INFO_STREAMPAR )
#endif  // !ADD_SMTC_STREAM
        {
            *bitfields |= ( 1 << requested_info_list[i] );
        }
    }
    return;
}

static uint8_t convert_dm_bitfield_to_bytes( uint32_t dm_info_bitfield, uint8_t* dm_array )
{
    uint8_t* p = dm_array;

    for( uint8_t i = 0; i < DM_INFO_MAX; i++ )
    {
        if( ( dm_info_bitfield & ( 1 << i ) ) )
        {
            *p++ = i;  // If bit is set, added id Code in payload
        }
    }
    return p - dm_array;
}

/* --------------------- DM Uplink -------------------------------------------*/

static void cloud_dm_status_add_task( cloud_dm_t* ctx, uint32_t delay_to_execute_s )
{
    if( ctx->enabled == false )
    {
        return;
    }
    smodem_task task_dm = { 0 };
    task_dm.id          = ctx->task_id;
    task_dm.stack_id    = ctx->stack_id;

    if( ctx->number_of_muted_day > 0 )
    {
        task_dm.priority = TASK_VERY_HIGH_PRIORITY;  // very hight to be not filtered when the device is muted
    }
    else
    {
        task_dm.priority = TASK_LOW_PRIORITY;
    }

    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + delay_to_execute_s;

    if( ctx->number_of_muted_day < 255 )
    {
        modem_supervisor_add_task( &task_dm );
    }
}

static bool dm_status_payload( cloud_dm_t* ctx, uint8_t stack_id, uint8_t* dm_uplink_message,
                               uint8_t* dm_uplink_message_len, uint8_t max_size, uint32_t dm_info_bitfield,
                               uint8_t* dm_opcode )
{
    uint8_t* p_tmp   = dm_uplink_message;
    uint8_t* p       = dm_uplink_message;
    bool     pending = false;

    if( check_dm_status_max_size( dm_info_bitfield, max_size ) != DM_CMD_LENGTH_VALID )
    {
        *dm_uplink_message_len = 0;
        SMTC_MODEM_HAL_TRACE_ERROR( "check_dm_status_max_size\n" );
        return false;
    }

    if( *dm_opcode >= DM_INFO_MAX )
    {
        *dm_opcode = 0;
    }
    SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( "dm_info_bitfield = %d \n", dm_info_bitfield );
    while( ( *dm_opcode ) < DM_INFO_MAX )
    {
        // SMTC_MODEM_HAL_TRACE_WARNING("dm_opcode %d - %d\n",*dm_opcode, (dm_info_bitfield >> *dm_opcode) & 0x01
        // );
        if( ( dm_info_bitfield & ( 1 << *dm_opcode ) ) )
        {
            *p_tmp++ = *dm_opcode;  // Add id Code in payload then the value(s)
            switch( *dm_opcode )
            {
            case DM_INFO_STATUS:
                *p_tmp = modem_get_status( stack_id );
                break;
            case DM_INFO_CHARGE: {
                uint32_t charge;
                smtc_modem_get_charge( &charge );
                *p_tmp         = charge & 0xFF;
                *( p_tmp + 1 ) = ( charge >> 8 ) & 0xFF;
                break;
            }
            case DM_INFO_VOLTAGE:
                *p_tmp = smtc_modem_hal_get_voltage_mv( ) / 20;
                break;
            case DM_INFO_TEMP:
                *p_tmp = smtc_modem_hal_get_temperature( );
                break;
            case DM_INFO_SIGNAL: {
                int16_t rssi = ctx->lorawan_last_rssi_get;
                if( rssi >= -128 && rssi <= 63 )
                {
                    // strength of last downlink (RSSI [dBm]+64)
                    *p_tmp = ( int8_t ) ( rssi + 64 );
                }
                else if( rssi > 63 )
                {
                    *p_tmp = 127;
                }
                else if( rssi < -128 )
                {
                    *p_tmp = -128;
                }
                // strength of last downlink (SNR [0.25 dB])
                *( p_tmp + 1 ) = ctx->lorawan_last_snr_get << 2;
                break;
            }
            case DM_INFO_UPTIME: {
                // The uptime is the RTC start
                uint32_t time  = smtc_modem_hal_get_time_in_s( ) / 3600;
                *p_tmp         = time & 0xFF;
                *( p_tmp + 1 ) = time >> 8;
            }
            break;
            case DM_INFO_RXTIME: {
                uint32_t time_h = 0;
                uint32_t rtc_s  = smtc_modem_hal_get_time_in_s( );

                if( rtc_s >= ctx->last_dl_timestamp_s )
                {
                    time_h = ( rtc_s - ctx->last_dl_timestamp_s ) / 3600;
                }
                else
                {
                    time_h = ( uint32_t ) ( ~0UL ) - ctx->last_dl_timestamp_s + rtc_s;
                }

                *p_tmp         = time_h & 0xFF;
                *( p_tmp + 1 ) = time_h >> 8;
            }
            break;
            case DM_INFO_FIRMWARE: {
                memset( p_tmp, 0, 8 );  // fill with 0 as firmware info is only useful in embedded modem
            }
            break;
            case DM_INFO_ADRMODE:
                *p_tmp = lorawan_api_dr_strategy_get( stack_id );
                break;
            case DM_INFO_JOINEUI: {
                uint8_t p_tmp_app_eui[8];
                lorawan_api_get_joineui( p_tmp_app_eui, stack_id );
                memcpy1_r( p_tmp, p_tmp_app_eui, 8 );
                break;
            }
            case DM_INFO_INTERVAL:
                *p_tmp = cloud_dm_get_dm_interval( stack_id );
                break;
            case DM_INFO_REGION:
                *p_tmp = lorawan_api_get_region( stack_id );
                break;
            case DM_INFO_RFU_0:
                // Nothing to do
                break;
            case DM_INFO_CRASHLOG:

                break;
            case DM_INFO_RSTCOUNT: {
                uint32_t nb_reset = modem_get_reset_counter( );
                *p_tmp            = nb_reset & 0xFF;
                *( p_tmp + 1 )    = nb_reset >> 8;
                break;
            }
            case DM_INFO_DEVEUI: {
                uint8_t p_tmp_dev_eui[8];
                lorawan_api_get_deveui( p_tmp_dev_eui, stack_id );
                memcpy1_r( p_tmp, p_tmp_dev_eui, 8 );
                break;
            }
            case DM_INFO_RFU_1:
                // Nothing to do
                break;
            case DM_INFO_SESSION: {
                uint16_t dev_nonce = lorawan_api_devnonce_get( stack_id );
                *p_tmp             = dev_nonce & 0xFF;
                *( p_tmp + 1 )     = dev_nonce >> 8;
                break;
            }
            case DM_INFO_CHIPEUI: {
                uint8_t p_tmp_chip_eui[8] = { 0 };
#if defined( USE_LR11XX_CE )
                lr11xx_system_read_uid( modem_get_radio_ctx( ), ( uint8_t* ) &p_tmp_chip_eui );
#endif  // USE_LR11XX_CE
                memcpy1_r( p_tmp, p_tmp_chip_eui, 8 );
                break;
            }
#if defined( ADD_SMTC_STREAM )
            case DM_INFO_STREAMPAR:
                *p_tmp         = stream_get_port( stack_id );
                *( p_tmp + 1 ) = stream_encrypted_mode( stack_id );
                break;
#endif  // ADD_SMTC_STREAM
            case DM_INFO_APPSTATUS:
                cloud_dm_get_modem_user_app_status( stack_id, p_tmp );
                break;
            case DM_INFO_ALMSTATUS:
                // handled in dedicated almanac update service
                break;
            default:
                SMTC_MODEM_HAL_TRACE_ERROR( "Construct DM payload report, unknown code 0x%02x\n", *dm_opcode );
                break;
            }

            p_tmp += dm_info_field_sz[*dm_opcode];
            // Check if last message can be enqueued
            if( ( p_tmp - dm_uplink_message ) <= max_size )
            {
                p = p_tmp;
            }
            else
            {
                // last message can't be enqueued
                pending = true;
                break;  // break for loop
            }
        }
        ( *dm_opcode )++;
    }

    *dm_uplink_message_len = p - dm_uplink_message;
    return pending;
}

static dm_cmd_length_valid_t check_dm_status_max_size( uint32_t info_requested, uint8_t max_size )
{
    for( uint8_t i = 0; i < DM_INFO_MAX; i++ )
    {
        if( ( info_requested & ( 1 << i ) ) )
        {
            if( max_size < dm_info_field_sz[i] )
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "max_size must be greater than the smallest requested information\n" );
                return DM_CMD_LENGTH_NOT_VALID;
            }
        }
    }
    return DM_CMD_LENGTH_VALID;
}

/* --- EOF ------------------------------------------------------------------ */
