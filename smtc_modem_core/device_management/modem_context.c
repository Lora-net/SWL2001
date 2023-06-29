/*!
 * \file      modem_context.c
 *
 * \brief     share functions + context of the soft modem .
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

#include "modem_context.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_modem_hal.h"
#include "smtc_real.h"
#include "device_management_defs.h"
#include "lorawan_api.h"
#include "modem_utilities.h"  // for crc
#include "smtc_modem_api.h"
#include "smtc_modem_middleware_advanced_api.h"
#include "smtc_modem_utilities.h"
#include "alc_sync.h"
#include "lr1mac_utilities.h"
#include "modem_supervisor.h"

#if defined( LR1110_MODEM_E )
#include "pool_mem.h"
#include "smtc_hal_mcu.h"
#include "fragmented_data_block.h"
#if defined( _MODEM_E_GNSS_ENABLE )
#include "gnss_ctrl_api.h"
#endif  //_MODEM_E_GNSS_ENABLE
#endif  // LR1110_MODEM_E

#if defined( LR11XX_TRANSCEIVER ) && defined( ENABLE_MODEM_GNSS_FEATURE )
#include "almanac_update.h"
#endif  // LR11XX_TRANSCEIVER && ENABLE_MODEM_GNSS_FEATURE

#if defined( USE_LR11XX_CE )
#include "lr11xx_system.h"
#endif  // USE_LR11XX_CE
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
#define MODEM_APPKEY_CRC_STATUS_VALID ( 0 )
#define MODEM_APPKEY_CRC_STATUS_INVALID ( 1 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
typedef struct modem_context_nvm_idx_s
{
    uint16_t dm_port;
    uint16_t dm_upload_sctr;
    uint8_t  appkey_crc_status;
    uint32_t appkey_crc;
    uint32_t rfu[3];
    uint32_t crc;  // !! crc MUST be the last field of the structure !!
} modem_context_nvm_t;

#if !defined( LR1110_MODEM_E )
static int16_t  modem_appkey_status = MODEM_APPKEY_CRC_STATUS_INVALID;
static uint32_t modem_appkey_crc    = 0;
static uint8_t  modem_status        = 0;
static uint8_t  modem_dm_interval   = DEFAULT_DM_REPORTING_INTERVAL;
static uint8_t  modem_dm_port       = DEFAULT_DM_PORT;
#if defined( ADD_SMTC_PATCH_UPDATE )
static uint8_t modem_frag_port = DEFAULT_FRAG_PORT;
#endif  // ADD_SMTC_PATCH_UPDATE
static uint8_t                modem_appstatus[8] = { 0 };
static smtc_modem_class_t     modem_dm_class     = SMTC_MODEM_CLASS_A;
static modem_suspend_status_t is_modem_suspend   = MODEM_NOT_SUSPEND;
static uint32_t               modem_start_time   = 0;
#if defined( ADD_SMTC_FILE_UPLOAD )
static uint8_t              modem_dm_upload_sctr = 0;
static modem_upload_state_t modem_upload_state   = MODEM_UPLOAD_NOT_INIT;
#endif  // ADD_SMTC_FILE_UPLOAD
#if defined( ADD_SMTC_STREAM )
static modem_stream_t modem_stream_state = {  //
    .port       = DEFAULT_DM_PORT,            //
    .state      = MODEM_STREAM_NOT_INIT,      //
    .encryption = false
};
#endif                                                                            // ADD_SMTC_STREAM
static uint32_t dm_info_bitfield_periodic         = DEFAULT_DM_REPORTING_FIELDS;  // context for periodic GetInfo
static uint32_t dm_info_bitfield_now              = 0;                            // User GetInfo
static uint8_t  tag_number                        = 0;
static uint8_t  tag_number_now                    = 0;
static uint8_t  number_of_muted_day               = 0;
static dm_dl_opportunities_config_t dm_pending_dl = { .up_count = 0, .up_delay = 0 };
static uint32_t                     user_alarm    = 0x7FFFFFFF;
static uint8_t                      asynchronous_msgnumber = 0;
static uint8_t                      modem_event_count[MODEM_NUMBER_OF_EVENTS];
static uint8_t                      modem_event_status[MODEM_NUMBER_OF_EVENTS];
static uint8_t                      asynch_msg[MODEM_NUMBER_OF_EVENTS];
static modem_downlink_msg_t         modem_dwn_pkt;
static bool                         is_modem_reset_requested    = false;
static bool                         is_modem_charge_loaded      = false;
static uint32_t                     modem_charge_offset         = 0;
static bool                         start_time_was_set          = false;
static uint16_t                     user_define_charge_counter  = 0;
static charge_counter_value_t       charge_counter_to_send      = CHARGE_COUNTER_MODEM;
static rf_output_t                  modem_rf_output             = MODEM_RFO_LP_LF;
static uint8_t                      duty_cycle_disabled_by_host = false;
static uint32_t                     crc_fw;
static smtc_modem_adr_profile_t     modem_adr_profile;
#if defined( ADD_SMTC_FILE_UPLOAD )
static uint32_t modem_upload_avgdelay;
#endif  // ADD_SMTC_FILE_UPLOAD
static uint16_t             nb_adr_mobile_timeout;
static bool                 is_modem_in_test_mode = false;
static int8_t               rx_pathloss_db        = 0;
static int8_t               tx_power_offset_db    = 0;
static radio_planner_t*     modem_rp              = NULL;
static modem_power_config_t power_config_lut[POWER_CONFIG_LUT_SIZE];
#if defined( SMTC_D2D )
static modem_context_class_b_d2d_t class_b_d2d_ctx;
#endif  // SMTC_D2D
static void ( *modem_lbm_notification_extended_1_callback )( void );
static void ( *modem_lbm_notification_extended_2_callback )( void );
static const void* modem_radio_ctx;  // use to save lr11xx user radio context needed to perform direct access to radio
                                     // withing modem code (almanac update, crypto)

#else
struct
{
    int16_t                      modem_appkey_status;
    uint32_t                     modem_appkey_crc;
    uint8_t                      modem_status;
    uint8_t                      modem_dm_interval;
    uint8_t                      modem_dm_port;
#if defined( ADD_SMTC_PATCH_UPDATE )
    uint8_t                      modem_frag_port;
#endif  // ADD_SMTC_PATCH_UPDATE
    uint8_t                      modem_appstatus[8];
    smtc_modem_class_t           modem_dm_class;
    modem_suspend_status_t       is_modem_suspend;
    uint32_t                     modem_start_time;
#if defined( ADD_SMTC_FILE_UPLOAD )
    uint8_t                      modem_dm_upload_sctr;
    modem_upload_state_t         modem_upload_state;
    uint32_t                     modem_upload_avgdelay;
#endif  // ADD_SMTC_FILE_UPLOAD
#if defined( ADD_SMTC_STREAM )
    modem_stream_t               modem_stream_state;
#endif  // ADD_SMTC_STREAM
    uint32_t                     dm_info_bitfield_periodic;  // context for periodic GetInfo
    uint32_t                     dm_info_bitfield_now;       // User GetInfo
    uint8_t                      tag_number;
    uint8_t                      tag_number_now;
    uint8_t                      number_of_muted_day;
    dm_dl_opportunities_config_t dm_pending_dl;
    uint32_t                     user_alarm;
    uint8_t                      asynchronous_msgnumber;
    uint8_t                      modem_event_count[MODEM_NUMBER_OF_EVENTS];
    uint8_t                      modem_event_status[MODEM_NUMBER_OF_EVENTS];
    uint8_t                      asynch_msg[MODEM_NUMBER_OF_EVENTS];
    modem_downlink_msg_t         modem_dwn_pkt;
    bool                         is_modem_reset_requested;
    bool                         is_modem_charge_loaded;
    uint32_t                     modem_charge_offset;
    uint8_t                      start_time_was_set;
    uint16_t                     user_define_charge_counter;
    charge_counter_value_t       charge_counter_to_send;
    rf_output_t                  modem_rf_output;
    uint8_t                      duty_cycle_disabled_by_host;
    uint32_t                     crc_fw;
    smtc_modem_adr_profile_t     modem_adr_profile;
    uint16_t                     nb_adr_mobile_timeout;
    bool                         is_modem_in_test_mode;
    int8_t                       rx_pathloss_db;
    int8_t                       tx_power_offset_db;
    radio_planner_t*             modem_rp;
    modem_power_config_t         power_config_lut[POWER_CONFIG_LUT_SIZE];
#if defined( SMTC_D2D )
    modem_context_class_b_d2d_t  class_b_d2d_ctx;
#endif  // SMTC_D2D
    void ( *modem_lbm_notification_extended_1_callback )( void );
    void ( *modem_lbm_notification_extended_2_callback )( void );
    const void* modem_radio_ctx;
} modem_ctx_context;

// clang-format off
#define  modem_appkey_status                        modem_ctx_context.modem_appkey_status
#define  modem_appkey_crc                           modem_ctx_context.modem_appkey_crc
#define  modem_status                               modem_ctx_context.modem_status
#define  modem_dm_interval                          modem_ctx_context.modem_dm_interval
#define  modem_dm_port                              modem_ctx_context.modem_dm_port
#if defined( ADD_SMTC_PATCH_UPDATE )
#define  modem_frag_port                            modem_ctx_context.modem_frag_port
#endif  // ADD_SMTC_PATCH_UPDATE
#define  modem_appstatus                            modem_ctx_context.modem_appstatus
#define  modem_dm_class                             modem_ctx_context.modem_dm_class
#define  is_modem_suspend                           modem_ctx_context.is_modem_suspend
#define  modem_start_time                           modem_ctx_context.modem_start_time
#if defined( ADD_SMTC_FILE_UPLOAD )
#define  modem_dm_upload_sctr                       modem_ctx_context.modem_dm_upload_sctr
#define  modem_upload_state                         modem_ctx_context.modem_upload_state
#define  modem_upload_avgdelay                      modem_ctx_context.modem_upload_avgdelay
#endif // ADD_SMTC_FILE_UPLOAD
#if defined( ADD_SMTC_STREAM )
#define  modem_stream_state                         modem_ctx_context.modem_stream_state
#endif  // ADD_SMTC_STREAM
#define  dm_info_bitfield_periodic                  modem_ctx_context.dm_info_bitfield_periodic
#define  dm_info_bitfield_now                       modem_ctx_context.dm_info_bitfield_now
#define  tag_number                                 modem_ctx_context.tag_number
#define  tag_number_now                             modem_ctx_context.tag_number_now
#define  number_of_muted_day                        modem_ctx_context.number_of_muted_day
#define  dm_pending_dl                              modem_ctx_context.dm_pending_dl
#define  user_alarm                                 modem_ctx_context.user_alarm
#define  asynchronous_msgnumber                     modem_ctx_context.asynchronous_msgnumber
#define  modem_event_count                          modem_ctx_context.modem_event_count
#define  modem_event_status                         modem_ctx_context.modem_event_status
#define  asynch_msg                                 modem_ctx_context.asynch_msg
#define  modem_dwn_pkt                              modem_ctx_context.modem_dwn_pkt
#define  is_modem_reset_requested                   modem_ctx_context.is_modem_reset_requested
#define  is_modem_charge_loaded                     modem_ctx_context.is_modem_charge_loaded
#define  modem_charge_offset                        modem_ctx_context.modem_charge_offset
#define  start_time_was_set                         modem_ctx_context.start_time_was_set
#define  user_define_charge_counter                 modem_ctx_context.user_define_charge_counter
#define  charge_counter_to_send                     modem_ctx_context.charge_counter_to_send
#define  modem_rf_output                            modem_ctx_context.modem_rf_output
#define  duty_cycle_disabled_by_host                modem_ctx_context.duty_cycle_disabled_by_host
#define  crc_fw                                     modem_ctx_context.crc_fw
#define  modem_adr_profile                          modem_ctx_context.modem_adr_profile
#define  nb_adr_mobile_timeout                      modem_ctx_context.nb_adr_mobile_timeout
#define  is_modem_in_test_mode                      modem_ctx_context.is_modem_in_test_mode
#define  rx_pathloss_db                             modem_ctx_context.rx_pathloss_db
#define  tx_power_offset_db                         modem_ctx_context.tx_power_offset_db
#define  modem_rp                                   modem_ctx_context.modem_rp
#define  power_config_lut                           modem_ctx_context.power_config_lut
#define  class_b_d2d_ctx                            modem_ctx_context.class_b_d2d_ctx
#define modem_lbm_notification_extended_1_callback  modem_ctx_context.modem_lbm_notification_extended_1_callback
#define modem_lbm_notification_extended_2_callback  modem_ctx_context.modem_lbm_notification_extended_2_callback
#define modem_radio_ctx                             modem_ctx_context.modem_radio_ctx
// clang-format on

#endif

// DM info field sizes
static const uint8_t dm_info_field_sz[DM_INFO_MAX] = {
    [DM_INFO_STATUS] = 1,    [DM_INFO_CHARGE] = 2,    [DM_INFO_VOLTAGE] = 1,  [DM_INFO_TEMP] = 1,
    [DM_INFO_SIGNAL] = 2,    [DM_INFO_UPTIME] = 2,    [DM_INFO_RXTIME] = 2,   [DM_INFO_FIRMWARE] = 8,
    [DM_INFO_ADRMODE] = 1,   [DM_INFO_JOINEUI] = 8,   [DM_INFO_INTERVAL] = 1, [DM_INFO_REGION] = 1,
    [DM_INFO_RFU_0]    = 4,
    [DM_INFO_CRASHLOG] = 0,  // (variable-length, send as last field or in separate frame)
    [DM_INFO_UPLOAD]   = 0,  // (variable-length, not sent periodically)
    [DM_INFO_RSTCOUNT] = 2,  [DM_INFO_DEVEUI] = 8,    [DM_INFO_RFU_1] = 2,    [DM_INFO_SESSION] = 2,
    [DM_INFO_CHIPEUI] = 8,
#if defined( ADD_SMTC_STREAM )
    [DM_INFO_STREAM]    = 0,  // (variable-length, not sent periodically)
    [DM_INFO_STREAMPAR] = 2, [DM_INFO_APPSTATUS] = 8,
#endif                        // ADD_SMTC_STREAM
    [DM_INFO_ALCSYNC]   = 0,  // (variable-length, not sent periodically)
    [DM_INFO_ALMSTATUS] = 7
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/*!
 * \brief   convert requested DM bytes fields to bitfield
 *
 * \param [in]  requested_info_list     Array of bytes with requested DM code in each bytes
 * \param [in]  len                     Number of byte that composed requested_info_list
 * \param [in]  bitfields *             Returned bitfield
 * \return     void
 */
static void convert_requested_dm_info_bytes_to_bitfield( const uint8_t* requested_info_list, uint8_t len,
                                                         uint32_t* bitfields )
{
    // Reset bitfield
    *bitfields = 0;
    for( uint8_t i = 0; i < len; i++ )
    {
        if( requested_info_list[i] != DM_INFO_CRASHLOG )
        {
            *bitfields |= ( 1 << requested_info_list[i] );
        }
    }
    return;
}

/*!
 * \brief   Check if the biggest requested DM status field can be inserted
 *          in the payload in regard of the max payload size requested
 *
 * \param [in]  info_requested              Requested bitfield
 * \param [in]  max_size                    Max size of the payload
 * \param [out] dm_cmd_length_valid_t       Return valid or not
 */
static dm_cmd_length_valid_t check_dm_status_max_size( uint32_t info_requested, uint8_t max_size )
{
    for( uint8_t i = 0; i < DM_INFO_MAX; i++ )
    {
        if( ( info_requested & ( 1 << i ) ) )
        {
            if( max_size < dm_info_field_sz[i] )
            {
                SMTC_MODEM_HAL_TRACE_ERROR(
                    "max_size must be greater than the smallest requested "
                    "information\n" );
                return DM_CMD_LENGTH_NOT_VALID;
            }
        }
    }
    return DM_CMD_LENGTH_VALID;
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void modem_context_init( )
{
    modem_status      = 0;
    modem_dm_interval = DEFAULT_DM_REPORTING_INTERVAL;
    modem_dm_port     = DEFAULT_DM_PORT;
#if defined( ADD_SMTC_PATCH_UPDATE )
    modem_frag_port = DEFAULT_FRAG_PORT;
#endif  // ADD_SMTC_PATCH_UPDATE
    modem_dm_class   = SMTC_MODEM_CLASS_A;
    is_modem_suspend = MODEM_NOT_SUSPEND;
    modem_start_time = 0;
#if defined( ADD_SMTC_FILE_UPLOAD )
    modem_dm_upload_sctr  = 0;
    modem_upload_state    = MODEM_UPLOAD_NOT_INIT;
    modem_upload_avgdelay = 0;
#endif  // ADD_SMTC_FILE_UPLOAD
#if defined( ADD_SMTC_STREAM )
    modem_stream_state.port       = DEFAULT_DM_PORT;
    modem_stream_state.state      = MODEM_STREAM_NOT_INIT;
    modem_stream_state.encryption = false;
#endif                                                          // ADD_SMTC_STREAM
    dm_info_bitfield_periodic   = DEFAULT_DM_REPORTING_FIELDS;  // context for periodic GetInfo
    dm_info_bitfield_now        = 0;                            // User GetInfo
    tag_number                  = 0;
    tag_number_now              = 0;
    number_of_muted_day         = 0;
    dm_pending_dl.up_count      = 0;
    dm_pending_dl.up_delay      = 0;
    user_alarm                  = 0;
    asynchronous_msgnumber      = 0;
    is_modem_reset_requested    = false;
    is_modem_charge_loaded      = false;
    modem_charge_offset         = 0;
    start_time_was_set          = false;
    user_define_charge_counter  = 0;
    charge_counter_to_send      = CHARGE_COUNTER_MODEM;
    modem_rf_output             = MODEM_RFO_LP_LF;
    duty_cycle_disabled_by_host = false;
    crc_fw                      = compute_crc_fw( );
    modem_adr_profile           = SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED;
    nb_adr_mobile_timeout       = DEFAULT_ADR_MOBILE_MODE_TIMEOUT;
    is_modem_in_test_mode       = false;
    rx_pathloss_db              = 0;
    tx_power_offset_db          = 0;
    modem_rp                    = NULL;
    modem_appkey_status         = MODEM_APPKEY_CRC_STATUS_INVALID;
    modem_appkey_crc            = 0;
    memset( modem_appstatus, 0, 8 );
    memset( modem_event_count, 0, MODEM_NUMBER_OF_EVENTS );
    memset( modem_event_status, 0, MODEM_NUMBER_OF_EVENTS );
    memset( asynch_msg, 0, MODEM_NUMBER_OF_EVENTS );
    memset( &modem_dwn_pkt, 0, sizeof( modem_downlink_msg_t ) );
    // init power config tab to 0x80 as it corresponds to an expected power of 128dbm, value that is never reached
    memset( power_config_lut, 0x80, POWER_CONFIG_LUT_SIZE * sizeof( modem_power_config_t ) );
#if defined( SMTC_D2D )
    memset( &class_b_d2d_ctx, 0, sizeof( modem_context_class_b_d2d_t ) );
#endif  // SMTC_D2D
}

void modem_event_init( void )
{
    for( int i = 0; i < MODEM_NUMBER_OF_EVENTS; i++ )
    {
        set_modem_event_count_and_status( i, 0, 0 );
    }
}

uint8_t get_modem_event_count( uint8_t event_type )
{
    if( event_type >= MODEM_NUMBER_OF_EVENTS )
    {
        smtc_modem_hal_mcu_panic( );
    }

    return ( modem_event_count[event_type] );
}

uint8_t get_modem_event_status( uint8_t event_type )
{
    if( event_type >= MODEM_NUMBER_OF_EVENTS )
    {
        smtc_modem_hal_mcu_panic( );
    }
    return ( modem_event_status[event_type] );
}

void set_modem_event_count_and_status( uint8_t event_type, uint8_t value, uint8_t status )
{
    if( event_type < MODEM_NUMBER_OF_EVENTS )
    {
        modem_event_count[event_type]  = value;
        modem_event_status[event_type] = status;
    }
}

void increment_modem_event_count_and_status( uint8_t event_type, uint8_t status )
{
    if( event_type < MODEM_NUMBER_OF_EVENTS )
    {
        if( modem_event_count[event_type] < 255 )
        {
            modem_event_count[event_type]++;
        }
        // Set last status even if the number of event max is reached
        modem_event_status[event_type] = status;
    }
}

void decrement_asynchronous_msgnumber( void )
{
    if( asynchronous_msgnumber > 0 )
    {
        asynchronous_msgnumber--;
    }
    else
    {
        asynchronous_msgnumber = 0;
    }
}

uint8_t get_asynchronous_msgnumber( void )
{
    return ( asynchronous_msgnumber );
}

void increment_asynchronous_msgnumber( uint8_t event_type, uint8_t status )
{
    // Next condition should never append because only one asynch msg by type of message
    if( asynchronous_msgnumber >= MODEM_NUMBER_OF_EVENTS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( " Modem reach the max number of asynch message\n" );
        return;
    }
    uint8_t tmp;
    tmp = get_modem_event_count( event_type );
    if( tmp == 0 )
    {
        asynch_msg[asynchronous_msgnumber] = event_type;
        asynchronous_msgnumber++;
    }

    increment_modem_event_count_and_status( event_type, status );
}

uint8_t get_last_msg_event( void )
{
    if( asynchronous_msgnumber > 0 )
    {
        return asynch_msg[asynchronous_msgnumber - 1];
    }
    return 0xFF;
}

uint32_t get_modem_uptime_s( void )
{
    return ( smtc_modem_hal_get_time_in_s( ) - modem_start_time );
}

void set_modem_start_time_s( uint32_t time )
{
    if( !start_time_was_set )
    {
        start_time_was_set = true;
        modem_start_time   = time;
    }
}

dm_rc_t set_modem_dm_interval( uint8_t interval )
{
    if( modem_dm_interval != interval )
    {
        modem_dm_interval = interval;
    }

    return ( DM_OK );
}
uint8_t get_modem_dm_interval( void )
{
    return ( modem_dm_interval );
}
uint32_t get_modem_dm_interval_second( void )
{
    uint8_t  dm_interval = get_modem_dm_interval( );
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
        smtc_modem_hal_mcu_panic( );
        break;
    }
    return temp;
}

void set_modem_class( smtc_modem_class_t lorawan_class )
{
    modem_dm_class = lorawan_class;
}

smtc_modem_class_t get_modem_class( void )
{
    return ( modem_dm_class );
}

dm_rc_t set_modem_dm_port( uint8_t port )
{
    if( ( port == 0 ) || ( port >= 224 ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "modem port invalid\n" );
        return ( DM_ERROR );
    }
    else
    {
        if( modem_dm_port != port )
        {
            modem_dm_port = port;
            modem_store_context( );
        }
        return ( DM_OK );
    }
}

uint8_t get_modem_dm_port( void )
{
    return ( modem_dm_port );
}

#if defined( ADD_SMTC_PATCH_UPDATE )
dm_rc_t set_modem_frag_port( uint8_t port )
{
    SMTC_MODEM_HAL_TRACE_ERROR( "set_modem_frag_port not implemented\n" );
    return ( DM_ERROR );
}

uint8_t get_modem_frag_port( void )
{
    return ( modem_frag_port );
}
#endif  // ADD_SMTC_PATCH_UPDATE

smtc_modem_adr_profile_t get_modem_adr_profile( void )
{
    return modem_adr_profile;
}

uint8_t get_modem_region( void )
{
    return lorawan_api_get_region( );
}

dm_rc_t set_modem_region( uint8_t region )
{
    if( lorawan_api_set_region( ( smtc_real_region_types_t ) region ) != OKLORAWAN )
    {
        return DM_ERROR;
    }
    return DM_OK;
}

modem_join_state_t get_join_state( void )
{
    modem_join_state_t joinstate;
    if( get_modem_status_joining( ) == true )
    {
        joinstate = MODEM_JOIN_ONGOING;
    }
    else if( lorawan_api_isjoined( ) != JOINED )
    {
        joinstate = MODEM_NOT_JOINED;
    }
    else
    {
        joinstate = MODEM_JOINED;
    }
    return ( joinstate );
}

void set_modem_appstatus( const uint8_t* app_status )
{
    memcpy( modem_appstatus, app_status, dm_info_field_sz[DM_INFO_APPSTATUS] );
}

void get_modem_appstatus( uint8_t* app_status )
{
    memcpy( app_status, modem_appstatus, dm_info_field_sz[DM_INFO_APPSTATUS] );
}

void modem_supervisor_add_task_join( void )
{
    smodem_task task_join = { 0 };
    task_join.id          = JOIN_TASK;
    task_join.priority    = TASK_HIGH_PRIORITY;

    uint32_t current_time_s = smtc_modem_hal_get_time_in_s( );

    task_join.time_to_execute_s = smtc_modem_hal_get_random_nb_in_range( 0, 5 );

#if defined( TEST_BYPASS_JOIN_DUTY_CYCLE )
    SMTC_MODEM_HAL_TRACE_WARNING( "BYPASS JOIN DUTY CYCLE activated\n" );
    task_join.time_to_execute_s += current_time_s;
#else
    if( lorawan_api_modem_certification_is_enabled( ) == false )
    {
        // current time is already taken in count in lr1mac time computation
        task_join.time_to_execute_s += lorawan_api_next_join_time_second_get( );
    }
#endif

    if( ( int32_t )( task_join.time_to_execute_s - current_time_s ) <= 0 )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " Start a new join sequence now \n" );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( " Start a new join sequence in %d seconds \n",
                                     task_join.time_to_execute_s - current_time_s );
    }

    set_modem_status_joining( true );
    modem_supervisor_add_task( &task_join );
}

void modem_supervisor_add_task_dm_status( uint32_t next_execute )
{
    smodem_task task_dm       = { 0 };
    task_dm.id                = DM_TASK;
    task_dm.priority          = TASK_LOW_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + next_execute;
    if( get_join_state( ) == MODEM_JOINED )
    {
        modem_supervisor_add_task( &task_dm );
    }
}

void modem_supervisor_add_task_dm_status_now( void )
{
    smodem_task task_dm       = { 0 };
    task_dm.id                = DM_TASK_NOW;
    task_dm.priority          = TASK_LOW_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) +
                                smtc_modem_hal_get_random_nb_in_range( DM_STATUS_NOW_MIN_TIME, DM_STATUS_NOW_MAX_TIME );
    modem_supervisor_add_task( &task_dm );
}
void modem_supervisor_add_task_crash_log( uint32_t next_execute )
{
    smodem_task task_dm       = { 0 };
    task_dm.id                = CRASH_LOG_TASK;
    task_dm.priority          = TASK_LOW_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + next_execute +
                                smtc_modem_hal_get_random_nb_in_range( DM_STATUS_NOW_MIN_TIME, DM_STATUS_NOW_MAX_TIME );
    modem_supervisor_add_task( &task_dm );
}

#if defined( ADD_SMTC_ALC_SYNC )
void modem_supervisor_add_task_clock_sync_time_req( uint32_t next_execute )
{
    smodem_task task_dm       = { 0 };
    task_dm.id                = CLOCK_SYNC_TIME_REQ_TASK;
    task_dm.priority          = TASK_HIGH_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + next_execute;

    modem_supervisor_add_task( &task_dm );
}

void modem_supervisor_remove_task_clock_sync( void )
{
    if( modem_supervisor_get_task_priority( CLOCK_SYNC_TIME_REQ_TASK ) != TASK_FINISH )
    {
        modem_supervisor_remove_task( CLOCK_SYNC_TIME_REQ_TASK );
    }
    if( modem_supervisor_get_task_priority( ALC_SYNC_ANS_TASK ) != TASK_FINISH )
    {
        modem_supervisor_remove_task( ALC_SYNC_ANS_TASK );
    }
}

bool modem_supervisor_is_clock_sync_running( void )
{
    bool ret = false;
    if( modem_supervisor_get_task_priority( CLOCK_SYNC_TIME_REQ_TASK ) != TASK_FINISH )
    {
        ret = true;
    }
    return ( ret );
}

void modem_supervisor_add_task_alc_sync_ans( uint32_t next_execute )
{
    smodem_task task_dm       = { 0 };
    task_dm.id                = ALC_SYNC_ANS_TASK;
    task_dm.priority          = TASK_HIGH_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + next_execute;
    if( get_join_state( ) == MODEM_JOINED )
    {
        modem_supervisor_add_task( &task_dm );
    }
}
#endif  // ADD_SMTC_ALC_SYNC

void modem_supervisor_add_task_alm_dbg_ans( uint32_t next_execute )
{
    smodem_task task_dm       = { 0 };
    task_dm.id                = DM_ALM_DBG_ANS;
    task_dm.priority          = TASK_HIGH_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + next_execute;
    if( get_join_state( ) == MODEM_JOINED )
    {
        modem_supervisor_add_task( &task_dm );
    }
}

void modem_supervisor_add_task_modem_mute( void )
{
    smodem_task task_dm       = { 0 };
    task_dm.id                = MUTE_TASK;
    task_dm.priority          = TASK_MEDIUM_HIGH_PRIORITY;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + 86400;  // Every 24h
    modem_supervisor_add_task( &task_dm );
}

void modem_supervisor_add_task_retrieve_dl( uint32_t next_execute )
{
    smodem_task task_dm       = { 0 };
    task_dm.id                = RETRIEVE_DL_TASK;
    task_dm.priority          = TASK_LOW_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.sizeIn            = 0;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + next_execute;
    if( get_join_state( ) == MODEM_JOINED )
    {
        modem_supervisor_add_task( &task_dm );
    }
}

void modem_supervisor_add_task_frag( uint32_t next_execute )
{
    smodem_task task_dm       = { 0 };
    task_dm.id                = FRAG_TASK;
    task_dm.priority          = TASK_HIGH_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + next_execute;
    if( get_join_state( ) == MODEM_JOINED )
    {
        modem_supervisor_add_task( &task_dm );
    }
}

#if defined( ADD_SMTC_STREAM )
void modem_supervisor_add_task_stream( void )
{
    // Modem supervisor copy everything,
    // so this is safe even when it is going to be invalidated.
    smodem_task stream_task = { 0 };

    stream_task.id                = STREAM_TASK;
    stream_task.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + smtc_modem_hal_get_random_nb_in_range( 1, 3 );
    stream_task.priority          = TASK_HIGH_PRIORITY;
    stream_task.fPort             = modem_get_stream_port( );
    stream_task.fPort_present     = true;
    // stream_task.dataIn        not used in task
    // stream_task.sizeIn        not used in task
    // stream_task.PacketType    not used in task
    modem_supervisor_add_task( &stream_task );
    // stream task is now on going: set modem status accordingly
    set_modem_status_streaming( true );
}
#endif  // ADD_SMTC_STREAM

#if defined( ADD_SMTC_FILE_UPLOAD )
void modem_supervisor_add_task_file_upload( uint32_t delay_in_s )
{
    smodem_task upload_task = { 0 };

    upload_task.id                = FILE_UPLOAD_TASK;
    upload_task.priority          = TASK_HIGH_PRIORITY;
    upload_task.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + delay_in_s;

    modem_supervisor_add_task( &upload_task );
}
#endif  // ADD_SMTC_FILE_UPLOAD

void modem_supervisor_add_task_link_check_req( uint32_t delay_in_s )
{
    smodem_task task_dm       = { 0 };
    task_dm.id                = LINK_CHECK_REQ_TASK;
    task_dm.priority          = TASK_HIGH_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + delay_in_s;
    if( get_join_state( ) == MODEM_JOINED )
    {
        modem_supervisor_add_task( &task_dm );
    }
}

void modem_supervisor_add_task_device_time_req( uint32_t delay_in_s )
{
    smodem_task task_dm       = { 0 };
    task_dm.id                = DEVICE_TIME_REQ_TASK;
    task_dm.priority          = TASK_HIGH_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + delay_in_s;
    if( get_join_state( ) == MODEM_JOINED )
    {
        modem_supervisor_add_task( &task_dm );
    }
}

void modem_supervisor_add_task_ping_slot_info_req( uint32_t delay_in_s )
{
    smodem_task task_dm       = { 0 };
    task_dm.id                = PING_SLOT_INFO_REQ_TASK;
    task_dm.priority          = TASK_HIGH_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + delay_in_s;
    if( get_join_state( ) == MODEM_JOINED )
    {
        modem_supervisor_add_task( &task_dm );
    }
}

uint8_t get_modem_status( void )
{
    // If the stack is no more join, the modem status was not aware of the disconnection
    if( get_join_state( ) != MODEM_JOINED )
    {
        set_modem_status_modem_joined( false );
    }
    return ( modem_status );
}

void get_modem_gnss_status( uint8_t* gnss_status )
{
#if defined( LR1110_MODEM_E ) && defined( _MODEM_E_GNSS_ENABLE )
    Gnss_context_status( gnss_status );
#elif defined( LR11XX_TRANSCEIVER ) && defined( ENABLE_MODEM_GNSS_FEATURE )
    uint8_t buffer_response[ALM_UPDATE_UPLINK_PAYLOAD_LENGTH];
    almanac_update_create_uplink_payload( modem_radio_ctx, buffer_response );
    // Discard first byte as it is already handle by the dm uplink process in modem_context
    memcpy( gnss_status, &buffer_response[1], ALM_UPDATE_UPLINK_PAYLOAD_LENGTH - 1 );
#endif
}

void set_modem_status_reset_after_brownout( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << MODEM_STATUS_OFFSET_BROWNOUT ) )
                                     : ( modem_status & ~( 1 << MODEM_STATUS_OFFSET_BROWNOUT ) );
}

void set_modem_status_reset_after_crash( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << MODEM_STATUS_OFFSET_CRASH ) )
                                     : ( modem_status & ~( 1 << MODEM_STATUS_OFFSET_CRASH ) );
}

void set_modem_status_modem_mute( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << MODEM_STATUS_OFFSET_MUTE ) )
                                     : ( modem_status & ~( 1 << MODEM_STATUS_OFFSET_MUTE ) );
}

void set_modem_status_modem_joined( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << MODEM_STATUS_OFFSET_JOINED ) )
                                     : ( modem_status & ~( 1 << MODEM_STATUS_OFFSET_JOINED ) );
}

void set_modem_status_radio_suspend( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << MODEM_STATUS_OFFSET_SUSPEND ) )
                                     : ( modem_status & ~( 1 << MODEM_STATUS_OFFSET_SUSPEND ) );
}

void set_modem_status_file_upload( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << MODEM_STATUS_OFFSET_UPLOAD ) )
                                     : ( modem_status & ~( 1 << MODEM_STATUS_OFFSET_UPLOAD ) );
}

void set_modem_status_joining( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << MODEM_STATUS_OFFSET_JOINING ) )
                                     : ( modem_status & ~( 1 << MODEM_STATUS_OFFSET_JOINING ) );
}

void set_modem_status_streaming( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << MODEM_STATUS_OFFSET_STREAMING ) )
                                     : ( modem_status & ~( 1 << MODEM_STATUS_OFFSET_STREAMING ) );
}

bool get_modem_status_reset_after_crash( void )
{
    return ( ( modem_status >> MODEM_STATUS_OFFSET_CRASH ) & 0x01 );
}

bool get_modem_status_file_upload( void )
{
    return ( ( modem_status >> MODEM_STATUS_OFFSET_UPLOAD ) & 0x01 );
}

bool get_modem_status_joining( void )
{
    return ( ( modem_status >> MODEM_STATUS_OFFSET_JOINING ) & 0x01 );
}

bool get_modem_status_streaming( void )
{
    return ( ( modem_status >> MODEM_STATUS_OFFSET_STREAMING ) & 0x01 );
}

void reset_modem_charge( void )
{
    radio_planner_t* rp = modem_context_get_modem_rp( );
    rp_stats_init( &rp->stats );
}

uint32_t get_modem_charge_ma_s( void )
{
    radio_planner_t* rp = modem_context_get_modem_rp( );
    uint32_t         total_consumption =
        rp->stats.tx_total_consumption_ma + rp->stats.rx_total_consumption_ma + rp->stats.none_total_consumption_ma;
    return ( ( total_consumption / 1000 ) + modem_charge_offset );
}

uint32_t get_modem_charge_ma_h( void )
{
    return get_modem_charge_ma_s( ) / 3600;
}

uint16_t get_modem_user_define_charge_ma_h( void )
{
    return user_define_charge_counter;
}

void set_modem_user_define_charge_ma_h( const uint16_t value )
{
    user_define_charge_counter = value;
}

void choose_modem_charge_counter( void )
{
    charge_counter_to_send = CHARGE_COUNTER_MODEM;
}

void choose_user_define_charge_counter( void )
{
    charge_counter_to_send = CHARGE_COUNTER_USER_DEFINE;
}

charge_counter_value_t get_charge_counter_to_send( void )
{
    return charge_counter_to_send;
}

uint8_t get_modem_voltage( void )
{
    return smtc_modem_hal_get_voltage( );
}
int8_t get_modem_temp( void )
{
    return smtc_modem_hal_get_temperature( );
}

dm_cmd_length_valid_t dm_check_dminfo_size( dm_info_field_t cmd, uint8_t length )
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

dm_rc_t dm_set_conf( dm_info_field_t tag, uint8_t* data, uint8_t length )
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
            // update modem context adr
            modem_adr_profile = ( smtc_modem_adr_profile_t ) data[0];

            status_lorawan_t status = lorawan_api_dr_strategy_set( ( dr_strategy_t ) data[0] );
            if( status == ERRORLORAWAN )
            {
                ret = DM_ERROR;
            }
            break;
        }
        case DM_INFO_JOINEUI: {
            uint8_t p_tmp[8];
            memcpy1_r( p_tmp, data, 8 );
            lorawan_api_set_joineui( &p_tmp[0] );
            break;
        }
        case DM_INFO_INTERVAL:
            set_modem_dm_interval( data[0] );
            modem_supervisor_add_task_dm_status( get_modem_dm_interval_second( ) );
            break;
        case DM_INFO_REGION:
            // TODO: see how to handle stack id here
            smtc_modem_leave_network( 0 );
            set_modem_region( data[0] );
            break;
        default:
            tag = DM_INFO_MAX;
            ret = DM_ERROR;
            break;
        }
    }
    if( ret == DM_OK )
    {
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_SETCONF, tag );
    }

    return ret;
}

modem_mute_status_t get_modem_muted( void )
{
    modem_mute_status_t mute;

    if( number_of_muted_day == MODEM_INFINITE_MUTE )
    {
        mute = MODEM_INFINITE_MUTE;
    }
    else if( number_of_muted_day > 0 )
    {
        mute = MODEM_TEMPORARY_MUTE;
    }
    else
    {
        mute = MODEM_NOT_MUTE;
    }
    return mute;
}

uint8_t dm_get_number_of_days_mute( void )
{
    return number_of_muted_day;
}

void dm_set_number_of_days_mute( uint8_t days )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "MUTE for %d days\n", days );
    if( number_of_muted_day != days )
    {
        number_of_muted_day = days;

        if( number_of_muted_day > 0 )
        {
            set_modem_status_modem_mute( true );
        }
        else
        {
            set_modem_status_modem_mute( false );
        }
    }
}

uint8_t get_dm_info_tag_list( uint8_t* dm, dm_info_rate_t flag )
{
    uint8_t* p = dm;
    uint32_t info_req;

    if( flag == DM_INFO_NOW )
    {
        info_req = dm_info_bitfield_now;
    }
    else
    {
        info_req = dm_info_bitfield_periodic;
    }

    for( uint8_t i = 0; i < DM_INFO_MAX; i++ )
    {
        if( ( info_req & ( 1 << i ) ) )
        {
            *p++ = i;  // If bit is set, added id Code in payload
        }
    }
    return p - dm;
}

dm_rc_t set_dm_info( const uint8_t* requested_info_list, uint8_t len, dm_info_rate_t flag )
{
    dm_rc_t  ret      = DM_OK;
    uint32_t info_req = 0;

    for( uint8_t i = 0; i < len; i++ )
    {
        // Ignore DM status with variable length and forbiden fields
        if( ( requested_info_list[i] == DM_INFO_UPLOAD ) || ( requested_info_list[i] == DM_INFO_STREAM ) ||
            ( requested_info_list[i] == DM_INFO_ALCSYNC ) || ( requested_info_list[i] == DM_INFO_DBGRSP ) ||
            ( requested_info_list[i] == DM_INFO_GNSSLOC ) || ( requested_info_list[i] == DM_INFO_WIFILOC ) ||
            ( requested_info_list[i] == DM_INFO_RFU_0 ) || ( requested_info_list[i] == DM_INFO_RFU_1 ) ||
            ( requested_info_list[i] >= DM_INFO_MAX ) )
        {
            ret = DM_ERROR;
            SMTC_MODEM_HAL_TRACE_ERROR( "invalid DM info code (0x%02x)\n", requested_info_list[i] );
        }
// In case geoloc is not available, check if DM_INFO_ALMSTATUS is requested and return an error if yes
#if !defined( ENABLE_MODEM_GNSS_FEATURE )
        if( requested_info_list[i] == DM_INFO_ALMSTATUS )
        {
            ret = DM_ERROR;
            SMTC_MODEM_HAL_TRACE_ERROR(
                "invalid DM info code: SMTC_MODEM_DM_FIELD_ALMANAC_STATUS is not allowed on chip without gnss "
                "features\n" );
        }
#endif
    }
    if( ret == DM_OK )
    {
        for( uint8_t i = 0; i < len; i++ )
        {
            if( requested_info_list[i] == DM_INFO_CRASHLOG )
            {
                if( flag == DM_INFO_NOW )
                {
                    modem_supervisor_add_task_crash_log( 0 );
                    if( len == 1 )
                    {
                        // only crash log is requested return directly here to avoid sending another empty message
                        return DM_OK;
                    }
                }
                else
                {
                    ret = DM_ERROR;
                }
            }
        }
    }

    if( ret == DM_OK )
    {
        convert_requested_dm_info_bytes_to_bitfield( requested_info_list, len, &info_req );
        if( flag == DM_INFO_NOW )
        {
            dm_info_bitfield_now = info_req;
            tag_number_now       = 0;  // Reset tag_number used by dm_status_payload to
            // start a report from beginning
            modem_supervisor_add_task_dm_status_now( );
        }
        else
        {
            if( dm_info_bitfield_periodic != info_req )
            {
                dm_info_bitfield_periodic = info_req;
                tag_number                = 0;  // Reset tag_number used by dm_status_payload to start
                                                // a report from beginning
            }
        }
    }

    return ret;
}

bool dm_status_payload( uint8_t* dm_uplink_message, uint8_t* dm_uplink_message_len, uint8_t max_size,
                        dm_info_rate_t flag )
{
    uint8_t* p_tmp = dm_uplink_message;
    uint8_t* p     = dm_uplink_message;
    uint32_t info_requested;
    uint8_t* tag     = NULL;
    bool     pending = false;

    // Used DM code given in parameter
    if( flag == DM_INFO_NOW )
    {
        info_requested = dm_info_bitfield_now;
        tag            = &tag_number_now;
        // Used DM code in context
    }
    else
    {
        info_requested = dm_info_bitfield_periodic;
        tag            = &tag_number;
    }

    if( check_dm_status_max_size( info_requested, max_size ) != DM_CMD_LENGTH_VALID )
    {
        *dm_uplink_message_len = 0;
        SMTC_MODEM_HAL_TRACE_ERROR( "check_dm_status_max_size\n" );
        return false;
    }

    if( *tag >= DM_INFO_MAX )
    {
        *tag = 0;
    }
    // SMTC_MODEM_HAL_TRACE_PRINTF("info_requested = %d \n",info_requested);
    while( ( *tag ) < DM_INFO_MAX )
    {
        // SMTC_MODEM_HAL_TRACE_WARNING("tag %d - %d\n",*tag, (info_requested >> *tag) & 0x01
        // );
        if( ( info_requested & ( 1 << *tag ) ) )
        {
            *p_tmp++ = *tag;  // Add id Code in payload then the value(s)
            switch( *tag )
            {
            case DM_INFO_STATUS:
                *p_tmp = get_modem_status( );
                break;
            case DM_INFO_CHARGE: {
                uint32_t charge;
                if( get_charge_counter_to_send( ) == CHARGE_COUNTER_MODEM )
                {
                    charge = get_modem_charge_ma_h( );
                }
                else
                {
                    charge = get_modem_user_define_charge_ma_h( );
                }

                *p_tmp         = charge & 0xFF;
                *( p_tmp + 1 ) = ( charge >> 8 ) & 0xFF;
                break;
            }
            case DM_INFO_VOLTAGE:
                *p_tmp = get_modem_voltage( );
                break;
            case DM_INFO_TEMP:
                *p_tmp = get_modem_temp( );
                break;
            case DM_INFO_SIGNAL: {
                int16_t rssi = lorawan_api_last_rssi_get( );
                if( rssi >= -128 && rssi <= 63 )
                {
                    *p_tmp = ( int8_t )( rssi + 64 );  // strength of last downlink (RSSI [dBm]+64)
                }
                else if( rssi > 63 )
                {
                    *p_tmp = 127;
                }
                else if( rssi < -128 )
                {
                    *p_tmp = -128;
                }
                *( p_tmp + 1 ) = lorawan_api_last_snr_get( ) << 2;  // strength of last downlink (SNR [0.25 dB])
                break;
            }
            case DM_INFO_UPTIME: {
                uint32_t time  = get_modem_uptime_s( ) / 3600;
                *p_tmp         = time & 0xFF;
                *( p_tmp + 1 ) = time >> 8;
            }
            break;
            case DM_INFO_RXTIME: {
                modem_downlink_msg_t dwnframe = { 0 };
                get_modem_downlink_frame( &dwnframe );
                uint32_t time  = ( smtc_modem_hal_get_time_in_s( ) - ( dwnframe.timestamp / 1000 ) ) / 3600;
                *p_tmp         = time & 0xFF;
                *( p_tmp + 1 ) = time >> 8;
            }
            break;
            case DM_INFO_FIRMWARE: {
#if defined( LR1110_MODEM_E ) && defined( ADD_SMTC_PATCH_UPDATE )
                // return the crc value of fuota dedicated for test have to be
                // re implement when fuota availble
                *( p_tmp + 0 ) = crc_fw & 0xFF;
                *( p_tmp + 1 ) = ( crc_fw >> 8 ) & 0xFF;
                *( p_tmp + 2 ) = ( crc_fw >> 16 ) & 0xFF;
                *( p_tmp + 3 ) = ( crc_fw >> 24 ) & 0xFF;
                *( p_tmp + 4 ) = frag_get_session_counter( ) & 0xFF;
                *( p_tmp + 5 ) = ( frag_get_session_counter( ) >> 8 ) & 0xFF;
                *( p_tmp + 6 ) = frag_get_nb_frag_received( ) & 0xFF;
                *( p_tmp + 7 ) = ( frag_get_nb_frag_received( ) >> 8 ) & 0xFF;
#else
                memset( p_tmp, 0, 8 );  // TODO remove this
#endif  // LR1110_MODEM_E && ADD_SMTC_PATCH_UPDATE
            }
            break;
            case DM_INFO_ADRMODE:
                *p_tmp = get_modem_adr_profile( );
                break;
            case DM_INFO_JOINEUI: {
                uint8_t p_tmp_app_eui[8];
                lorawan_api_get_joineui( p_tmp_app_eui );
                memcpy1_r( p_tmp, p_tmp_app_eui, 8 );
                break;
            }
            case DM_INFO_INTERVAL:
                *p_tmp = get_modem_dm_interval( );
                break;
            case DM_INFO_REGION:
                *p_tmp = get_modem_region( );
                break;
            case DM_INFO_RFU_0:
                // Nothing to do
                break;
            case DM_INFO_CRASHLOG:

                break;
            case DM_INFO_RSTCOUNT:
                *p_tmp         = lorawan_api_nb_reset_get( ) & 0xFF;
                *( p_tmp + 1 ) = lorawan_api_nb_reset_get( ) >> 8;
                break;
            case DM_INFO_DEVEUI: {
                uint8_t p_tmp_dev_eui[8];
                lorawan_api_get_deveui( p_tmp_dev_eui );
                memcpy1_r( p_tmp, p_tmp_dev_eui, 8 );
                break;
            }
            case DM_INFO_RFU_1:
                // Nothing to do
                break;
            case DM_INFO_SESSION:
                *p_tmp         = lorawan_api_devnonce_get( ) & 0xFF;
                *( p_tmp + 1 ) = lorawan_api_devnonce_get( ) >> 8;
                break;
            case DM_INFO_CHIPEUI: {
                uint8_t p_tmp_chip_eui[8] = { 0 };
#if defined( LR1110_MODEM_E )
                hal_mcu_read_chip_eui( p_tmp_chip_eui );
#endif  // LR1110_MODEM_E
#if defined( USE_LR11XX_CE )
                // lr11xx operation needed: suspend modem radio access to secure this direct access
                modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
                lr11xx_system_read_uid( modem_radio_ctx, ( uint8_t* ) &p_tmp_chip_eui );
                // lr11xx operation done: resume modem radio access
                modem_context_resume_radio_access( );
#endif  // USE_LR11XX_CE
                memcpy1_r( p_tmp, p_tmp_chip_eui, 8 );
                break;
            }
#if defined( ADD_SMTC_STREAM )
            case DM_INFO_STREAMPAR:
                *p_tmp         = modem_get_stream_port( );
                *( p_tmp + 1 ) = modem_get_stream_encryption( );
                break;
#endif  // ADD_SMTC_STREAM
            case DM_INFO_APPSTATUS:
                get_modem_appstatus( p_tmp );
                break;
            case DM_INFO_ALMSTATUS:
                get_modem_gnss_status( p_tmp );
                break;
            default:
                SMTC_MODEM_HAL_TRACE_ERROR( "Construct DM payload report, unknown code 0x%02x\n", *tag );
                break;
            }

            p_tmp += dm_info_field_sz[*tag];
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
        ( *tag )++;
    }

    *dm_uplink_message_len = p - dm_uplink_message;
    return pending;
}
#if defined( LR1110_MODEM_E ) && defined( ADD_SMTC_PATCH_UPDATE )
void dm_frag_uplink_payload( uint8_t max_payload_length, uint8_t* dm_uplink_message, uint8_t* dm_uplink_message_len )
{
    frag_set_max_length_up_payload( max_payload_length );

    frag_construct_uplink_payload( );

    frag_get_tx_buffer( &dm_uplink_message[0], dm_uplink_message_len );
}
#endif  // LR1110_MODEM_E && ADD_SMTC_PATCH_UPDATE

#if defined( LR1110_MODEM_E ) && defined( _MODEM_E_GNSS_ENABLE )
void dm_alm_dbg_uplink_payload( uint8_t max_payload_length, uint8_t* dm_uplink_message, uint8_t* dm_uplink_message_len )
{
    uint8_t* gnss_payload      = ( uint8_t* ) &POOL_MEM.GNSS_MEM.Buf_data[0];
    uint16_t gnss_payload_size = GnssGetSize( );
    SMTC_MODEM_HAL_TRACE_PRINTF( "GnssGetSize %d\n", gnss_payload_size );
    SMTC_MODEM_HAL_TRACE_PRINTF( "max_payload_length %d\n", max_payload_length );
    SMTC_MODEM_HAL_TRACE_ARRAY( "dm_alm_dbg_uplink_payload", gnss_payload, gnss_payload_size );
    if( ( gnss_payload[0] == GNSS_DM_MSG ) && ( gnss_payload_size <= max_payload_length ) && ( gnss_payload_size > 0 ) )
    {
        *dm_uplink_message_len = gnss_payload_size - 1;
        memcpy( dm_uplink_message, &gnss_payload[1], *dm_uplink_message_len );
    }
    else
    {
        *dm_uplink_message_len = 0;
    }
}
#endif  // LR1110_MODEM_E && _MODEM_E_GNSS_ENABLE

dm_rc_t set_modem_suspend( bool suspend )
{
    set_modem_status_radio_suspend( suspend );
    is_modem_suspend = ( ( suspend == true ) ? MODEM_SUSPEND : MODEM_NOT_SUSPEND );
    return DM_OK;
}

modem_suspend_status_t get_modem_suspend( void )
{
    return ( ( is_modem_suspend == MODEM_SUSPEND ) ? true : false );
}

dm_rc_t set_modem_adr_profile( smtc_modem_adr_profile_t adr_profile, const uint8_t* adr_custom_data,
                               uint8_t adr_custom_length )
{
    /* error case : 1) user_dr invalid
                    2) user_dr = custom but length not equal to 16
                    3) user_dr not custom but length not equal to 0*/
    if( ( adr_profile > SMTC_MODEM_ADR_PROFILE_CUSTOM ) ||
        ( ( adr_profile == SMTC_MODEM_ADR_PROFILE_CUSTOM ) && ( adr_custom_length != 16 ) ) ||
        ( ( adr_profile < SMTC_MODEM_ADR_PROFILE_CUSTOM ) && ( adr_custom_length != 0 ) ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "user_dr = %d not compatible with adr data length = %d \n ", adr_profile,
                                    adr_custom_length );
        return DM_ERROR;
    }

    // save profile in context:
    modem_adr_profile = adr_profile;

    status_lorawan_t status = ERRORLORAWAN;

    switch( adr_profile )
    {
    case SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED:
        // update profile in lorawan stack
        status = lorawan_api_dr_strategy_set( STATIC_ADR_MODE );
        break;
    case SMTC_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE:
        // update profile in lorawan stack
        status = lorawan_api_dr_strategy_set( MOBILE_LONGRANGE_DR_DISTRIBUTION );
        break;
    case SMTC_MODEM_ADR_PROFILE_MOBILE_LOW_POWER:
        // update profile in lorawan stack
        status = lorawan_api_dr_strategy_set( MOBILE_LOWPER_DR_DISTRIBUTION );
        break;
    case SMTC_MODEM_ADR_PROFILE_CUSTOM: {
        uint16_t MaskDrTmp          = lorawan_api_mask_tx_dr_channel_up_dwell_time_check( );
        uint32_t adrDistribution[2] = { 0 };
        uint8_t  cpt_tmp            = 0;
        for( uint8_t i = 0; i < 16; i++ )
        {
            if( adr_custom_data[i] > 15 )  // DR are defined from 0 to 15 by definition in LoRaWAN spec
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "ADR with DataRate out of range\n" );
                return DM_ERROR;
            }
            if( ( ( MaskDrTmp >> adr_custom_data[i] ) & 0x01 ) == 1 )
            {
                cpt_tmp++;

                if( adr_custom_data[i] == 0x00 )
                {
                    adrDistribution[0] += ( ( ( adrDistribution[0] & 0xF0000000 ) >> 28 ) != 0xF ) ? ( 1 << 28 ) : 0;
                }
                else if( adr_custom_data[i] == 0x01 )
                {
                    adrDistribution[0] += ( ( ( adrDistribution[0] & 0x0F000000 ) >> 24 ) != 0xF ) ? ( 1 << 24 ) : 0;
                }
                else if( adr_custom_data[i] == 0x02 )
                {
                    adrDistribution[0] += ( ( ( adrDistribution[0] & 0x00F00000 ) >> 20 ) != 0xF ) ? ( 1 << 20 ) : 0;
                }
                else if( adr_custom_data[i] == 0x03 )
                {
                    adrDistribution[0] += ( ( ( adrDistribution[0] & 0x000F0000 ) >> 16 ) != 0xF ) ? ( 1 << 16 ) : 0;
                }
                else if( adr_custom_data[i] == 0x04 )
                {
                    adrDistribution[0] += ( ( ( adrDistribution[0] & 0x0000F000 ) >> 12 ) != 0xF ) ? ( 1 << 12 ) : 0;
                }
                else if( adr_custom_data[i] == 0x05 )
                {
                    adrDistribution[0] += ( ( ( adrDistribution[0] & 0x00000F00 ) >> 8 ) != 0xF ) ? ( 1 << 8 ) : 0;
                }
                else if( adr_custom_data[i] == 0x06 )
                {
                    adrDistribution[0] += ( ( ( adrDistribution[0] & 0x000000F0 ) >> 4 ) != 0xF ) ? ( 1 << 4 ) : 0;
                }
                else if( adr_custom_data[i] == 0x07 )
                {
                    adrDistribution[0] += ( ( ( adrDistribution[0] & 0x0000000F ) ) != 0xF ) ? 1 : 0;
                }
                else if( adr_custom_data[i] == 0x08 )
                {
                    adrDistribution[1] += ( ( ( adrDistribution[1] & 0xF0000000 ) >> 28 ) != 0xF ) ? ( 1 << 28 ) : 0;
                }
                else if( adr_custom_data[i] == 0x09 )
                {
                    adrDistribution[1] += ( ( ( adrDistribution[1] & 0x0F000000 ) >> 24 ) != 0xF ) ? ( 1 << 24 ) : 0;
                }
                else if( adr_custom_data[i] == 0x0A )
                {
                    adrDistribution[1] += ( ( ( adrDistribution[1] & 0x00F00000 ) >> 20 ) != 0xF ) ? ( 1 << 20 ) : 0;
                }
                else if( adr_custom_data[i] == 0x0B )
                {
                    adrDistribution[1] += ( ( ( adrDistribution[1] & 0x000F0000 ) >> 16 ) != 0xF ) ? ( 1 << 16 ) : 0;
                }
            }
        }
        if( cpt_tmp == 0 )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "ADR with a bad DataRate value\n" );
            return DM_ERROR;
        }
        lorawan_api_dr_custom_set( adrDistribution );
        // update profile in lorawan stack
        status = lorawan_api_dr_strategy_set( USER_DR_DISTRIBUTION );
        break;
    }
    default: {
        SMTC_MODEM_HAL_TRACE_ERROR( "Unknown adr profile %d\n ", adr_profile );
        return DM_ERROR;
    }
    break;
    }

    if( status == ERRORLORAWAN )
    {
        return DM_ERROR;
    }
    else
    {
        return DM_OK;
    }
}

void set_modem_downlink_frame( uint8_t* data, uint8_t data_length, lr1mac_down_metadata_t* metadata )
{
    memcpy( modem_dwn_pkt.data, data, data_length );
    modem_dwn_pkt.length       = data_length;
    modem_dwn_pkt.timestamp    = metadata->timestamp;
    modem_dwn_pkt.snr          = metadata->rx_snr << 2;
    modem_dwn_pkt.rssi         = metadata->rx_rssi + 64;
    modem_dwn_pkt.port         = metadata->rx_fport;
    modem_dwn_pkt.fpending_bit = metadata->rx_fpending_bit;
    modem_dwn_pkt.frequency_hz = metadata->rx_frequency_hz;
    modem_dwn_pkt.datarate     = metadata->rx_datarate;
    SMTC_MODEM_HAL_TRACE_ARRAY( "Downlink frame ", modem_dwn_pkt.data, modem_dwn_pkt.length );
    SMTC_MODEM_HAL_TRACE_PRINTF( "DL Port = %d , ", modem_dwn_pkt.port );
    SMTC_MODEM_HAL_TRACE_PRINTF( "DL SNR = %d , DL RSSI = %d , ", modem_dwn_pkt.snr, modem_dwn_pkt.rssi );
    SMTC_MODEM_HAL_TRACE_PRINTF( "DL Freq = %lu , DL DR = %d , ", modem_dwn_pkt.frequency_hz, modem_dwn_pkt.datarate );
    SMTC_MODEM_HAL_TRACE_PRINTF( "DL Fpending Bit = %d \n", modem_dwn_pkt.fpending_bit );
}
void get_modem_downlink_frame( modem_downlink_msg_t* modem_dwn_in )
{
    modem_dwn_in->timestamp    = modem_dwn_pkt.timestamp;
    modem_dwn_in->snr          = modem_dwn_pkt.snr;
    modem_dwn_in->rssi         = modem_dwn_pkt.rssi;
    modem_dwn_in->port         = modem_dwn_pkt.port;
    modem_dwn_in->fpending_bit = modem_dwn_pkt.fpending_bit;
    modem_dwn_in->frequency_hz = modem_dwn_pkt.frequency_hz;
    modem_dwn_in->datarate     = modem_dwn_pkt.datarate;
    modem_dwn_in->length       = modem_dwn_pkt.length;
    memcpy( modem_dwn_in->data, modem_dwn_pkt.data, modem_dwn_pkt.length );
}

void set_dm_retrieve_pending_dl( uint8_t up_count, uint8_t up_delay )
{
    dm_pending_dl.up_count = up_count;
    dm_pending_dl.up_delay = ( up_delay < 20 ) ? 20 : up_delay;
}

void get_dm_retrieve_pending_dl( dm_dl_opportunities_config_t* pending_dl )
{
    pending_dl->up_count = dm_pending_dl.up_count;
    pending_dl->up_delay = dm_pending_dl.up_delay;
}

void decrement_dm_retrieve_pending_dl( void )
{
    if( dm_pending_dl.up_count > 0 )
    {
        dm_pending_dl.up_count--;
    }
}

void modem_store_context( void )
{
    modem_context_nvm_t ctx = {
        .dm_port = modem_dm_port,
        //.dm_upload_sctr    = modem_dm_upload_sctr,
        .appkey_crc_status = modem_appkey_status,
        .appkey_crc        = modem_appkey_crc,
        .rfu               = { 0 },
    };

    ctx.crc = crc( ( uint8_t* ) &ctx, sizeof( ctx ) - 4 );
    smtc_modem_hal_context_store( CONTEXT_MODEM, ( uint8_t* ) &ctx, sizeof( ctx ) );
    modem_load_context( );
}

/*!
 * \brief    load modem context in non volatile memory
 * \remark
 * \retval void
 */
void modem_load_context( void )
{
    modem_context_nvm_t ctx;

    smtc_modem_hal_context_restore( CONTEXT_MODEM, ( uint8_t* ) &ctx, sizeof( ctx ) );

    if( crc( ( uint8_t* ) &ctx, sizeof( ctx ) - 4 ) == ctx.crc )
    {
        modem_dm_port = ctx.dm_port;
        // modem_dm_upload_sctr = ctx.dm_upload_sctr;
        modem_appkey_status = ctx.appkey_crc_status;
        modem_appkey_crc    = ctx.appkey_crc;

        SMTC_MODEM_HAL_TRACE_PRINTF( "Modem Load Config :\n Port = %d\n", modem_dm_port );

#if defined( ADD_SMTC_FILE_UPLOAD )
        SMTC_MODEM_HAL_TRACE_PRINTF( "Upload_sctr = %d\n", modem_dm_upload_sctr );
#endif  // ADD_SMTC_FILE_UPLOAD
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Restore Modem context fail => Factory Reset Modem\n" );
        modem_context_factory_reset( );
    }
}

void modem_context_factory_reset( void )
{
    modem_context_nvm_t ctx = {
        .dm_port = DEFAULT_DM_PORT,
        // .dm_upload_sctr    = 0,
        .appkey_crc_status = MODEM_APPKEY_CRC_STATUS_INVALID,
        .appkey_crc        = 0,
    };

    ctx.crc = crc( ( uint8_t* ) &ctx, sizeof( ctx ) - 4 );
    smtc_modem_hal_context_store( CONTEXT_MODEM, ( uint8_t* ) &ctx, sizeof( ctx ) );

    // dummy context reading to ensure context store is done before exiting the function
    smtc_modem_hal_context_restore( CONTEXT_MODEM, ( uint8_t* ) &ctx, sizeof( ctx ) );

    is_modem_reset_requested = true;
    SMTC_MODEM_HAL_TRACE_INFO( "modem_context_factory_reset done\n" );
}

#if defined( ADD_SMTC_FILE_UPLOAD )
uint8_t modem_context_compute_and_get_next_dm_upload_sctr( void )
{
    // take current and add 1 then mask to fit 4bits length (overflow is implicitely managed)
    modem_dm_upload_sctr = ( modem_dm_upload_sctr + 1 ) & 0xf;
    return modem_dm_upload_sctr;
}

modem_upload_state_t modem_get_upload_state( void )
{
    return ( modem_upload_state );
}

void modem_set_upload_state( modem_upload_state_t upload_state )
{
    modem_upload_state = upload_state;
}
#endif  // ADD_SMTC_FILE_UPLOAD

#if defined( ADD_SMTC_STREAM )
modem_stream_status_t modem_get_stream_state( void )
{
    return ( modem_stream_state.state );
}

uint8_t modem_get_stream_port( void )
{
    return ( modem_stream_state.port );
}

bool modem_get_stream_encryption( void )
{
    return ( modem_stream_state.encryption );
}

void modem_set_stream_state( modem_stream_status_t stream_state )
{
    modem_stream_state.state = stream_state;
}

void modem_set_stream_port( uint8_t port )
{
    modem_stream_state.port = port;
}

void modem_set_stream_encryption( bool enc )
{
    modem_stream_state.encryption = enc;
}
#endif  // ADD_SMTC_STREAM

void modem_set_dm_info_bitfield_periodic( uint32_t value )
{
    if( dm_info_bitfield_periodic != value )
    {
        dm_info_bitfield_periodic = value;
    }
}

void modem_context_reset_dm_tag_number( void )
{
    tag_number = 0;
}

uint32_t modem_get_dm_info_bitfield_periodic( void )
{
    return ( dm_info_bitfield_periodic );
}
uint32_t modem_get_user_alarm( void )
{
    return ( user_alarm );
}

void modem_set_user_alarm( uint32_t alarm )
{
    user_alarm = alarm;
}

bool get_modem_reset_requested( void )
{
    return is_modem_reset_requested;
}
void set_modem_reset_requested( bool reset_req )
{
    is_modem_reset_requested = reset_req;
}

rf_output_t modem_get_rfo_pa( void )
{
    return modem_rf_output;
}

uint8_t modem_set_rfo_pa( rf_output_t rf_output )
{
    if( rf_output >= MODEM_RFO_MAX )
    {
        return DM_ERROR;
    }
    modem_rf_output = rf_output;
    return DM_OK;
}

void modem_set_duty_cycle_disabled_by_host( uint8_t disabled_by_host )
{
    duty_cycle_disabled_by_host = disabled_by_host;
}

uint8_t modem_get_duty_cycle_disabled_by_host( void )
{
    return duty_cycle_disabled_by_host;
}

void modem_set_adr_mobile_timeout_config( uint16_t nb_tx )
{
    nb_adr_mobile_timeout = nb_tx;
}

uint16_t modem_get_adr_mobile_timeout_config( void )
{
    return nb_adr_mobile_timeout;
}

bool modem_available_new_link_adr_request( void )
{
    return lorawan_api_available_link_adr_get( );
}
void modem_set_test_mode_status( bool enable )
{
    is_modem_in_test_mode = enable;
}

bool modem_get_test_mode_status( void )
{
    return is_modem_in_test_mode;
}

void modem_context_set_rx_pathloss_db( int8_t rx_pathloss )
{
    rx_pathloss_db = rx_pathloss;
}

int8_t modem_context_get_rx_pathloss_db( void )
{
    return rx_pathloss_db;
}

void modem_context_set_tx_power_offset_db( int8_t tx_power_offset )
{
    tx_power_offset_db = tx_power_offset;
}

int8_t modem_context_get_tx_power_offset_db( void )
{
    return tx_power_offset_db;
}

radio_planner_t* modem_context_get_modem_rp( void )
{
    return modem_rp;
}

void modem_context_set_modem_rp( radio_planner_t* rp )
{
    modem_rp = rp;
}

void modem_context_empty_callback( void* ctx )
{
    // SMTC_MODEM_HAL_TRACE_ERROR( " empty call back \n" );
}

#if !defined( LR1110_MODEM_E )
bool modem_context_suspend_user_radio_access( rp_task_types_t type )
{
    rp_radio_params_t fake_radio_params = { 0 };

    rp_task_t rp_task = {
        .hook_id                    = RP_HOOK_ID_USER_SUSPEND,
        .launch_task_callbacks      = modem_context_empty_callback, /* called when the task starts */
        .duration_time_ms           = 20000,
        .state                      = RP_TASK_STATE_SCHEDULE,
        .type                       = type,
        .schedule_task_low_priority = false,
        .start_time_ms              = smtc_modem_hal_get_time_in_ms( ) + 4,
    };
    rp_hook_status_t status = rp_task_enqueue( modem_rp, &rp_task, NULL, 0, &fake_radio_params );

    return ( status == RP_HOOK_STATUS_OK ) ? true : false;
}

bool modem_context_resume_user_radio_access( void )
{
    bool status = true;

    if( rp_task_abort( modem_rp, RP_HOOK_ID_USER_SUSPEND ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to abort hook\n" );
        status = false;
    }

    return status;
}
#endif  // !LR1110_MODEM_E

bool modem_context_suspend_radio_access( rp_task_types_t type )
{
    rp_radio_params_t fake_radio_params = { 0 };

    rp_task_t rp_task = {
        .hook_id                    = RP_HOOK_ID_SUSPEND,
        .launch_task_callbacks      = modem_context_empty_callback,
        .duration_time_ms           = 20000,
        .state                      = RP_TASK_STATE_SCHEDULE,
        .type                       = type,
        .schedule_task_low_priority = false,
        .start_time_ms              = smtc_modem_hal_get_time_in_ms( ) + 4,
    };

    // First disable modem irq to secure radio access
    smtc_modem_hal_disable_modem_irq( );

    rp_hook_status_t status = rp_task_enqueue( modem_rp, &rp_task, NULL, 0, &fake_radio_params );

    return ( status == RP_HOOK_STATUS_OK ) ? true : false;
}

bool modem_context_resume_radio_access( void )
{
    bool status = true;

    if( rp_task_abort( modem_rp, RP_HOOK_ID_SUSPEND ) != RP_HOOK_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to abort hook\n" );
        status = false;
    }

    // After the suspension re-enable modem irq
    smtc_modem_hal_enable_modem_irq( );

    return status;
}

void modem_context_set_power_config_lut( uint8_t config[30] )
{
    // save raw buff into power lut
    for( uint8_t i = 0; i < POWER_CONFIG_LUT_SIZE; i++ )
    {
        power_config_lut[i].expected_power   = config[5 * i];
        power_config_lut[i].configured_power = config[( 5 * i ) + 1];
        power_config_lut[i].pa_param1        = config[( 5 * i ) + 2];
        power_config_lut[i].pa_param2        = config[( 5 * i ) + 3];
        power_config_lut[i].pa_ramp_time     = config[( 5 * i ) + 4];
    }
}

modem_power_config_t* modem_context_get_power_config_lut( void )
{
    return power_config_lut;
}

modem_ctx_rc_t modem_context_set_appkey( const uint8_t app_key[16] )
{
    modem_ctx_rc_t rc = MODEM_CTX_RC_SUCCESS;
// To prevent too much flash access first check crc on key in case of Hardware Secure element
#if( defined( LR1110_MODEM_E ) && defined( USE_LR11XX_CE ) ) || defined( USE_LR11XX_CE )
    uint32_t new_crc = crc( app_key, 16 );

    if( ( modem_appkey_status == MODEM_APPKEY_CRC_STATUS_INVALID ) || ( modem_appkey_crc != new_crc ) )
    {
        modem_appkey_crc    = new_crc;
        modem_appkey_status = MODEM_APPKEY_CRC_STATUS_VALID;

        if( lorawan_api_set_appkey( app_key ) != OKLORAWAN )
        {
            rc = MODEM_CTX_RC_ERROR;
        }
        else
        {
            // Store appkey crc and status
            modem_store_context( );
        }
    }
#else
    if( lorawan_api_set_appkey( app_key ) != OKLORAWAN )
    {
        rc = MODEM_CTX_RC_ERROR;
    }
#endif
    return rc;
}

void modem_context_appkey_is_derived( void )
{
    modem_appkey_status = MODEM_APPKEY_CRC_STATUS_INVALID;
    modem_store_context( );
}

bool modem_context_get_network_type( void )
{
    return lorawan_api_get_network_type( );
}

void modem_context_set_network_type( bool network_type )
{
    lorawan_api_set_network_type( network_type );
}

const void* modem_context_get_modem_radio_ctx( void )
{
    return modem_radio_ctx;
}

void modem_context_set_modem_radio_ctx( const void* radio_ctx )
{
    modem_radio_ctx = radio_ctx;
}

#if defined( SMTC_D2D )
void modem_context_set_class_b_d2d_last_metadata( uint8_t mc_grp_id, bool tx_done, uint8_t nb_trans_not_send )
{
    class_b_d2d_ctx.tx_done           = tx_done;
    class_b_d2d_ctx.nb_trans_not_send = nb_trans_not_send;
    class_b_d2d_ctx.mc_grp_id         = mc_grp_id;

    if( tx_done == true )
    {
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_D2D_CLASS_B_TX_DONE,
                                          SMTC_MODEM_EVENT_D2D_CLASS_B_TX_DONE_SENT );
    }
    else
    {
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_D2D_CLASS_B_TX_DONE,
                                          SMTC_MODEM_EVENT_D2D_CLASS_B_TX_DONE_NOT_SENT );
    }
}

void modem_context_get_class_b_d2d_last_metadata( modem_context_class_b_d2d_t* class_b_d2d )
{
    memcpy( class_b_d2d, &class_b_d2d_ctx, sizeof( modem_context_class_b_d2d_t ) );
}
#endif  // SMTC_D2D

void modem_set_extended_callback( func_callback callback, uint8_t extended_uplink_id )
{
    if( extended_uplink_id == 1 )
    {
        modem_lbm_notification_extended_1_callback = callback;
    }
    else if( extended_uplink_id == 2 )
    {
        modem_lbm_notification_extended_2_callback = callback;
    }
    else
    {
        // already manage by upper layer
    }
}
func_callback modem_get_extended_callback( uint8_t extended_uplink_id )
{
    if( extended_uplink_id == 1 )
    {
        return modem_lbm_notification_extended_1_callback;
    }
    else if( extended_uplink_id == 2 )
    {
        return modem_lbm_notification_extended_2_callback;
    }
    else
    {
        return NULL;
    }
}

void modem_leave( void )
{
    // reset all tasks to retrieve a clean env (and clear all ongoing tasks)
    modem_supervisor_init_task( );

    // Set joined/joining status to false
    set_modem_status_modem_joined( false );
    lorawan_api_join_status_clear( );
    set_modem_status_joining( false );

    // Disable properly class B and class C
    lorawan_api_class_b_enabled( false );
    lorawan_api_class_c_enabled( false );

#if defined( ADD_SMTC_FILE_UPLOAD )
    // Stop and reset file upload service
    set_modem_status_file_upload( false );
    modem_set_upload_state( MODEM_UPLOAD_NOT_INIT );
#endif  // ADD_SMTC_FILE_UPLOAD

#if defined( ADD_SMTC_STREAM )
    // Stop and reset stream service
    set_modem_status_streaming( false );
    modem_set_stream_state( MODEM_STREAM_NOT_INIT );
#endif  // ADD_SMTC_STREAM
}

/* --- EOF ------------------------------------------------------------------ */
