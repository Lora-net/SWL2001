/*!
 * \file      dm_downlink.c
 *
 * \brief     handle downlink from device management
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

#include "dm_downlink.h"

#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_modem_hal.h"
#include "alc_sync.h"
#include "stream.h"
#include "device_management_defs.h"
#include "modem_context.h"
#include "modem_supervisor.h"
#include "lorawan_api.h"

#if defined( LR1110_MODEM_E ) && defined( _MODEM_E_GNSS_ENABLE )
#include "gnss_ctrl_api.h"
#endif  // LR1110_MODEM_E && _MODEM_E_GNSS_ENABLE

#if defined( LR11XX_TRANSCEIVER ) || defined( LR1110_MODEM_E )
#include "smtc_basic_modem_lr11xx_api_extension.h"
#endif  // LR11XX_TRANSCEIVER || LR1110_MODEM_E

#if defined( ENABLE_MODEM_GNSS_FEATURE ) && defined( LR11XX_TRANSCEIVER )
#include "almanac_update.h"
#include "lr11xx_gnss.h"
#endif  // ENABLE_MODEM_GNSS_FEATURE && LR11XX_TRANSCEIVER

#if defined( LR1110_MODEM_E )
#include "smtc_modem_e_api_extension.h"
#endif  // LR1110_MODEM_E

extern smtc_modem_services_t smtc_modem_services_ctx;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACRO -----------------------------------------------------------
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
 * @brief Math Abs macro
 */
#define ABS( N ) ( ( N < 0 ) ? ( -N ) : ( N ) )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

#if MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON
static const char* dm_cmd_str[DM_CMD_MAX] = {
    [DM_RESET] = "RESET",  //
    [DM_FUOTA] = "FUOTA",  //
#if defined( ADD_SMTC_FILE_UPLOAD )
    [DM_FILE_DONE] = "FILE_DONE",      //
#endif                                 // ADD_SMTC_FILE_UPLOAD
    [DM_GET_INFO]    = "GET_INFO",     //
    [DM_SET_CONF]    = "SET_CONF",     //
    [DM_REJOIN]      = "REJOIN",       //
    [DM_MUTE]        = "MUTE",         //
    [DM_SET_DM_INFO] = "SET_DM_INFO",  //
#if defined( ADD_SMTC_STREAM )
    [DM_STREAM] = "STREAM",          //
#endif                               // ADD_SMTC_STREAM
    [DM_ALC_SYNC]   = "ALC_SYNC",    //
    [DM_ALM_UPDATE] = "ALM_UPDATE",  //
#if !defined( DISABLE_ALMANAC_DBG_OPCODE )
    [DM_ALM_DBG] = "ALM_DEBUG",          //
#endif                                   //! DISABLE_ALMANAC_DBG_OPCODE
    [DM_SOLV_UPDATE] = "SOLVER_UPDATE",  //
    [DM_ALM_FUPDATE] = "ALM_FUPDATE",    //
};
#endif

static const uint8_t dm_cmd_len[DM_CMD_MAX][2] = {  // CMD              = {min,       max}
    [DM_RESET] = { 3, 3 },
    [DM_FUOTA] = { 1, 255 },
#if defined( ADD_SMTC_FILE_UPLOAD )
    [DM_FILE_DONE] = { 1, 1 },
#endif  // ADD_SMTC_FILE_UPLOAD
    [DM_GET_INFO]    = { 1, 255 },
    [DM_SET_CONF]    = { 2, 255 },
    [DM_REJOIN]      = { 2, 2 },
    [DM_MUTE]        = { 1, 1 },
    [DM_SET_DM_INFO] = { 1, DM_INFO_MAX },
#if defined( ADD_SMTC_STREAM )
    [DM_STREAM] = { 1, 255 },
#endif  // ADD_SMTC_STREAM
    [DM_ALC_SYNC]   = { 1, 255 },
    [DM_ALM_UPDATE] = { 1, 255 },
#if !defined( DISABLE_ALMANAC_DBG_OPCODE )
    [DM_ALM_DBG] = { 1, 255 },
#endif  // !DISABLE_ALMANAC_DBG_OPCODE
    [DM_SOLV_UPDATE] = { 1, 255 },
    [DM_ALM_FUPDATE] = { 1, 255 }
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * \brief   check DM cmd Size
 *
 * \param [in]  cmd                         Current dm code that must be checked
 * \param [in]  length                      Length of the tested requested code
 * \param [out] dm_cmd_length_valid_t       Return valid length or not
 */
static dm_cmd_length_valid_t dm_check_cmd_size( dm_opcode_t cmd, uint8_t length );

/*!
 * \brief   DM Reset
 *
 * \param [in]  reset_code                  Type of requested reset
 * \param [in]  reset_session               reset Session is compare to the current number of modem reset
 * \param [out] dm_rc_t                     Return valid or not
 */
static dm_rc_t dm_reset( dm_reset_code_t reset_code, uint16_t reset_session );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

dm_rc_t dm_downlink( uint8_t* data, uint8_t length )
{
    dm_cmd_msg_t dm_input;
    if( length <= DM_DOWNLINK_HEADER_LENGTH )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "DM Downlink must contain at least 4 bytes\n" );
        return DM_ERROR;
    }

    dm_input.up_count     = data[0];
    dm_input.up_delay     = data[1];
    dm_input.request_code = ( dm_opcode_t ) data[2];
    dm_input.buffer       = &data[3];
    dm_input.buffer_len   = length - DM_DOWNLINK_HEADER_LENGTH;

    // parse dm downlink
    dm_rc_t dm_parse_status = dm_parse_cmd( &dm_input );

    // save upcount and updelay
    set_dm_retrieve_pending_dl( dm_input.up_count, dm_input.up_delay );

    // if needed add a task in supervisor to send an empty downlink
    if( dm_input.up_count > 0 )
    {
        modem_supervisor_add_task_retrieve_dl( dm_input.up_delay );
    }

    return dm_parse_status;
}

dm_rc_t dm_parse_cmd( dm_cmd_msg_t* cmd_input )
{
    dm_rc_t ret = DM_OK;
    if( dm_check_cmd_size( cmd_input->request_code, cmd_input->buffer_len ) != DM_CMD_LENGTH_VALID )
    {
        return DM_ERROR;
    }
#if MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON
    SMTC_MODEM_HAL_TRACE_WARNING( "DM_%s (0x%02x)\n", dm_cmd_str[cmd_input->request_code], cmd_input->request_code );
#endif

    switch( cmd_input->request_code )
    {
    case DM_RESET:

        if( dm_reset( ( dm_reset_code_t ) cmd_input->buffer[0],
                      cmd_input->buffer[1] | ( cmd_input->buffer[2] << 8 ) ) != DM_OK )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "NOT RESET !!!\n" );
            ret = DM_ERROR;
            break;
        }
        break;
    case DM_FUOTA:
        // Not supported yet
        break;
#if defined( ADD_SMTC_FILE_UPLOAD )
    case DM_FILE_DONE:
        SMTC_MODEM_HAL_TRACE_WARNING( "DM_FILE_DONE donwlink\n" );

        if( modem_get_upload_state( ) != MODEM_UPLOAD_ON_GOING )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "No FileUpload ongoing\n" );
            break;
        }

        // process frame
        if( file_upload_process_file_done_frame( &( smtc_modem_services_ctx.file_upload_ctx ), cmd_input->buffer,
                                                 cmd_input->buffer_len ) != FILE_UPLOAD_OK )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "DM_FILE_DONE bad session_counter or bad message\n" );
        }
        else
        {
            // file upload is done with server confirmation
            SMTC_MODEM_HAL_TRACE_INFO( "File upload DONE with server confirmation \n" );
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_UPLOADDONE, 0x01 );
            set_modem_status_file_upload( false );
            modem_set_upload_state( MODEM_UPLOAD_FINISHED );
            modem_supervisor_remove_task( FILE_UPLOAD_TASK );
        }
        break;
#endif  // ADD_SMTC_FILE_UPLOAD
    case DM_GET_INFO:
        if( set_dm_info( cmd_input->buffer, cmd_input->buffer_len, DM_INFO_NOW ) != DM_OK )
        {
            ret = DM_ERROR;
            break;
        }
        break;
    case DM_SET_CONF:
        if( dm_set_conf( ( dm_info_field_t ) cmd_input->buffer[0], cmd_input->buffer + 1, cmd_input->buffer_len - 1 ) !=
            DM_OK )
        {
            ret = DM_ERROR;
            break;
        }

        break;
    case DM_REJOIN: {
        uint16_t dev_nonce = ( cmd_input->buffer[1] << 8 ) | cmd_input->buffer[0];
        if( lorawan_api_devnonce_get( ) != dev_nonce )
        {
            uint8_t dm_fields_payload[1] = { DM_INFO_SESSION };
            set_dm_info( dm_fields_payload, 1, DM_INFO_NOW );
            ret = DM_ERROR;
            break;
        }
        // Leave network
        modem_leave( );
        // Add a new join task
        modem_supervisor_add_task_join( );
        break;
    }
    case DM_MUTE:
        dm_set_number_of_days_mute( cmd_input->buffer[0] );
        if( get_modem_muted( ) == MODEM_TEMPORARY_MUTE )
        {
            modem_supervisor_add_task_modem_mute( );
        }
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_MUTE, 0 );
        break;
    case DM_SET_DM_INFO:
        if( set_dm_info( cmd_input->buffer, cmd_input->buffer_len, DM_INFO_PERIODIC ) != DM_OK )
        {
            ret = DM_ERROR;
            break;
        }

        break;
#if defined( ADD_SMTC_STREAM )
    case DM_STREAM:
        if( stream_process_dn_frame( &( smtc_modem_services_ctx.stream_ROSE_ctx ), cmd_input->buffer,
                                     cmd_input->buffer_len ) != STREAM_OK )
        {
            ret = DM_ERROR;
        }
        break;
#endif  // ADD_SMTC_STREAM
#if defined( ADD_SMTC_ALC_SYNC )
    case DM_ALC_SYNC: {
        uint8_t alc_sync_status =
            alc_sync_parser( &( smtc_modem_services_ctx.alc_sync_ctx ), cmd_input->buffer, cmd_input->buffer_len );

        if( ( ( alc_sync_status >> ALC_SYNC_APP_TIME_ANS ) & 0x1 ) == 1 )
        {
            // an alcsync dl with time was received => update flag
            smtc_modem_services_ctx.alc_sync_ctx.is_sync_dl_received = true;

            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TIME, SMTC_MODEM_EVENT_TIME_VALID );

            // Remove all alc sync task.
            modem_supervisor_remove_task_clock_sync( );

            if( clock_sync_is_enabled( &( smtc_modem_services_ctx.clock_sync_ctx ) ) == true )
            {
                int32_t  tmp_rand = 0;
                uint32_t tmp_delay =
                    MIN( clock_sync_get_interval_second( &( smtc_modem_services_ctx.clock_sync_ctx ) ),
                         clock_sync_get_time_left_connection_lost( &( smtc_modem_services_ctx.clock_sync_ctx ) ) );
                do
                {
                    tmp_rand = smtc_modem_hal_get_signed_random_nb_in_range( -30, 30 );
                } while( ( tmp_rand < 0 ) && ( ABS( tmp_rand ) > tmp_delay ) );

                modem_supervisor_add_task_clock_sync_time_req( tmp_delay + tmp_rand );
            }
        }

        if( alc_sync_status & ( ~( 1 << ALC_SYNC_APP_TIME_ANS ) ) )
        {
            if( ( ( alc_sync_status >> ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ ) & 0x1 ) == 1 )
            {
                // If periodic time request is configured, add task to handle it

                if( clock_sync_is_enabled( &( smtc_modem_services_ctx.clock_sync_ctx ) ) == true )
                {
                    int32_t  tmp_rand = 0;
                    uint32_t tmp_delay =
                        MIN( clock_sync_get_interval_second( &( smtc_modem_services_ctx.clock_sync_ctx ) ),
                             clock_sync_get_time_left_connection_lost( &( smtc_modem_services_ctx.clock_sync_ctx ) ) );
                    do
                    {
                        tmp_rand = smtc_modem_hal_get_signed_random_nb_in_range( -30, 30 );
                    } while( ( tmp_rand < 0 ) && ( ABS( tmp_rand ) > tmp_delay ) );

                    modem_supervisor_add_task_clock_sync_time_req( tmp_delay + tmp_rand );
                }
            }
            // When a request requiring an answer is requested, add task to handle it
            modem_supervisor_add_task_alc_sync_ans( 1 );
        }
    }
    break;
#endif  // ADD_SMTC_ALC_SYNC

    case DM_ALM_UPDATE: {
#if defined( LR1110_MODEM_E ) && defined( _MODEM_E_GNSS_ENABLE )
        if( Gnss_push_almanac_update( cmd_input->buffer, cmd_input->buffer_len, 0 ) != GNSS_CMD_OK )
#elif defined( LR11XX_TRANSCEIVER ) && defined( ENABLE_MODEM_GNSS_FEATURE )
        // format received buffer to fit das raw input (with 2 more bytes upcount updelay and the DM_ALM_UPDATE
        // opcode)
        uint8_t                      buff[cmd_input->buffer_len + 3];
        almanac_update_return_code_t rc = ALMANAC_ERROR;

        buff[2] = DM_ALM_UPDATE;
        memcpy( &buff[3], cmd_input->buffer, cmd_input->buffer_len );
        rc = almanac_update_process_downlink_payload( modem_context_get_modem_radio_ctx( ), buff,
                                                      cmd_input->buffer_len + 3 );
        if( rc == ALMANAC_OK )
        {
            event_almanac_update_status_t event_almanac_update_status = ALMANAC_EVENT_ALL_UPDATES_RECEIVED;
            // check upcount value to manage the event status
            if( cmd_input->up_count > 0 )
            {
                // DAS asked for a new uplink containing the almanac crc
                event_almanac_update_status = ALMANAC_EVENT_DAS_NEED_NEW_CRC;

                // reset up_count and up_delay value to only let user uplink needed dm.
                cmd_input->up_count = 0;
                cmd_input->up_delay = 0;
            }
            // update event almanac update
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_ALMANAC_UPDATE, event_almanac_update_status );
        }
        else
#endif
        {
            ret = DM_ERROR;
        }
        break;
    }
    case DM_ALM_FUPDATE: {
#if defined( LR1110_MODEM_E ) && defined( _MODEM_E_GNSS_ENABLE )
        if( Gnss_push_almanac_update( cmd_input->buffer, cmd_input->buffer_len, 1 ) != GNSS_CMD_OK )
#elif defined( LR11XX_TRANSCEIVER ) && defined( ENABLE_MODEM_GNSS_FEATURE )
        // format received buffer to fit das raw input (with 2 more bytes upcount updelay and the DM_ALM_UPDATE
        // opcode)
        uint8_t                      buff[cmd_input->buffer_len + 3];
        almanac_update_return_code_t rc = ALMANAC_ERROR;

        buff[2] = DM_ALM_FUPDATE;
        memcpy( &buff[3], cmd_input->buffer, cmd_input->buffer_len );
        rc = almanac_update_process_downlink_payload( modem_context_get_modem_radio_ctx( ), buff,
                                                      cmd_input->buffer_len + 3 );
        if( rc == ALMANAC_OK )
        {
            event_almanac_update_status_t event_almanac_update_status = ALMANAC_EVENT_ALL_UPDATES_RECEIVED;
            // check upcount value to manage the event status
            if( cmd_input->up_count > 0 )
            {
                // DAS asked for a new uplink containing the almanac crc
                event_almanac_update_status = ALMANAC_EVENT_DAS_NEED_NEW_CRC;

                // reset up_count and up_delay value to only let user uplink needed dm.
                cmd_input->up_count = 0;
                cmd_input->up_delay = 0;
            }
            // update event almanac update
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_ALMANAC_UPDATE, event_almanac_update_status );
        }
        else
#endif
        {
            ret = DM_ERROR;
        }
        break;
    }
#if !defined( DISABLE_ALMANAC_DBG_OPCODE )
    case DM_ALM_DBG: {
#if defined( LR1110_MODEM_E ) && defined( _MODEM_E_GNSS_ENABLE )
        // Is DM set time request (not used in soft modem)
        uint32_t gps_time_s = is_set_time_request( cmd_input->buffer, cmd_input->buffer_len );
        if( gps_time_s )
        {
            smtc_modem_set_time( gps_time_s );
        }
        if( Gnss_push_dbg_request( cmd_input->buffer, cmd_input->buffer_len ) != GNSS_CMD_OK )
        {
            ret = DM_ERROR;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "modem_supervisor_add_task_alm_dbg_ans\n" );

            // Prepare the task to send answers
            modem_supervisor_add_task_alm_dbg_ans( 1 );
        }
#elif defined( LR11XX_TRANSCEIVER ) && defined( ENABLE_MODEM_GNSS_FEATURE )
        // Not supported on Basic Modem with LR11XX tranceiver
        ret = DM_ERROR;
#endif
        ret = DM_ERROR;
    }
#endif  // !DISABLE_ALMANAC_DBG_OPCODE
    break;
    case DM_SOLV_UPDATE: {
#if defined( LR1110_MODEM_E ) && defined( _MODEM_E_GNSS_ENABLE )
        if( Gnss_push_msg_from_solver( cmd_input->buffer, cmd_input->buffer_len ) != GNSS_CMD_OK )
#elif defined( LR11XX_TRANSCEIVER ) && defined( ENABLE_MODEM_GNSS_FEATURE )
        lr11xx_status_t rc = LR11XX_STATUS_ERROR;
        modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
        rc = lr11xx_gnss_push_solver_msg( modem_context_get_modem_radio_ctx( ), cmd_input->buffer,
                                          cmd_input->buffer_len );
        modem_context_resume_radio_access( );
        if( rc != LR11XX_STATUS_OK )
#endif
        {
            ret = DM_ERROR;
        }
        break;
    }
    default:
        ret = DM_ERROR;
        break;
    }

    return ret;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static dm_cmd_length_valid_t dm_check_cmd_size( dm_opcode_t cmd, uint8_t length )
{
    if( cmd >= DM_CMD_MAX )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid DM command %x\n", cmd );
        return DM_CMD_NOT_VALID;
    }
    // cmd len too small
    if( length < dm_cmd_len[cmd][0] )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid DM command size (too small)\n" );
        return DM_CMD_LENGTH_NOT_VALID;
    }
    // cmd len too long
    if( length > dm_cmd_len[cmd][1] )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid DM command size (too long)\n" );
        return DM_CMD_LENGTH_NOT_VALID;
    }
    return DM_CMD_LENGTH_VALID;
}

static dm_rc_t dm_reset( dm_reset_code_t reset_code, uint16_t reset_session )
{
    dm_rc_t ret = DM_OK;

    if( reset_session == lorawan_api_nb_reset_get( ) )
    {
        switch( reset_code )
        {
        case DM_RESET_MODEM:
        case DM_RESET_APP_MCU:
        case DM_RESET_BOTH:
            lorawan_api_context_save( );
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
        set_dm_info( dm_fields_payload, 1, DM_INFO_NOW );
        ret = DM_ERROR;
        SMTC_MODEM_HAL_TRACE_ERROR( "invalid DM reset session code\n" );
    }

    return ret;
}