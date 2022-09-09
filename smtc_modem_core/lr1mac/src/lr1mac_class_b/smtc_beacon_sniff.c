/*!
 * \file      smtc_beacon_sniff.c
 *
 * \brief     Beacon management for LoRaWAN class B devices
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

#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_beacon_sniff.h"
#include "smtc_ping_slot.h"
#include "radio_planner.h"
#include "smtc_modem_hal.h"
#include "lr1_stack_mac_layer.h"
#include "lr1mac_core.h"
#include "smtc_real.h"
#include "smtc_secure_element.h"
#include "lr1mac_utilities.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
/**
 * @brief is called by the radio planner when the task enqueued inside beacon_rp_request is granted
 *
 * @param [out] rp_void radio planner object is given back by the rp as a context
 */
static void smtc_beacon_sniff_launch_callback_for_rp( void* rp_void );

/**
 * @brief define and enqueue a beacon task inside the rp
 *
 * @param [in] lr1_beacon_obj Beacon object
 */
static void beacon_rp_request( smtc_lr1_beacon_t* lr1_beacon_obj );

/**
 * @brief compute beacon crc
 *
 * @param [in] data* data buffer on which crc in computed
 * @param [in] size  data size
 * @return return the crc
 */
static uint16_t crc16_beacon( uint8_t* data, uint16_t size );

/**
 * @brief get beacon sf
 *
 * @param [in] lr1_mac* pointer to lr1mac stack object
 * @param [in] beacon_datarate  beacon data rate
 * @return return the beacon spreading factor
 */
static uint8_t get_beacon_sf( lr1_stack_mac_t* lr1_mac, uint8_t beacon_datarate );

/**
 * @brief get beacon bw
 *
 * @param [in] lr1_mac* pointer to lr1mac stack object
 * @param [in] beacon_datarate  beacon data rate
 * @return return the beacon bandwith
 */
static lr1mac_bandwidth_t get_beacon_bw( lr1_stack_mac_t* lr1_mac, uint8_t beacon_datarate );

/**
 * @brief get beacon length
 *
 * @param [in] beacon_sf  beacon sf
 * @return return the beacon length
 */
static uint8_t get_beacon_length( uint8_t beacon_sf );

/**
 * @brief get beacon task type for rp should be rx for normal case but also empty task to save powerand not listen this
 * beacon
 *
 * @param [in] lr1_beacon_obj Beacon object
 * @return return RP_TASK_TYPE_RX_LORA to listen beacon or RP_TASK_TYPE_NONE to not listen the next beacon
 */
static rp_task_types_t get_beacon_rp_task_type( smtc_lr1_beacon_t* lr1_beacon_obj );

/**
 * @brief update beacon meta data
 *
 * @param [in] lr1_beacon_obj Beacon object
 * @param [in] timestamp  Beacon timestamp
 * @param [in] beacon_epoch_time  Beacon epoch time
 */
static void compute_beacon_metadata( smtc_lr1_beacon_t* lr1_beacon_obj, uint32_t timestamp,
                                     uint32_t beacon_epoch_time );

/**
 * @brief update beacon pll
 *
 * @param [in,out] lr1_beacon_obj Beacon object
 * @param [in] timestamp  Beacon timestamp
 */
static void update_beacon_pll( smtc_lr1_beacon_t* lr1_beacon_obj, uint32_t timestamp );

/**
 * @brief update beacon state
 *
 * @param [in,out] lr1_beacon_obj Beacon object
 */
static void update_beacon_state( smtc_lr1_beacon_t* lr1_beacon_obj );

/**
 * @brief beacon print for debug
 *
 * @param [in] lr1_beacon_obj Beacon object
 */
static void beacon_debug_print( smtc_lr1_beacon_t* lr1_beacon_obj );

/**
 * @brief update the length in symboles of the duration of the rx beacon window
 *
 * @param [in,out] lr1_beacon_obj Beacon object
 * @param [in] target_time target time of the next beacon
 */
static void update_beacon_rx_nb_symb( smtc_lr1_beacon_t* lr1_beacon_obj, uint32_t target_time );

/**
 * @brief compute the time of the next beacon
 *
 * @param [in] lr1_beacon_obj Beacon object
 * @return  return the target time of the next beacon
 */
static uint32_t compute_start_time( smtc_lr1_beacon_t* lr1_beacon_obj );

/**
 * @brief check if the received beacon is valid
 *
 * @param [in] lr1_beacon_obj Beacon object
 * @param [in] timestamp timestamp of the received beacon
 * @return  true if it is a valid beacon , false if not
 */
static bool is_valid_beacon( smtc_lr1_beacon_t* lr1_beacon_obj, uint32_t timestamp );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACRO---- --------------------------------------------------------
 */

/**
 * @brief Compute the beacon datarate
 */
#define BEACON_DATA_RATE( ) smtc_real_get_beacon_dr( lr1_beacon_obj->lr1_mac )

/**
 * @brief Compute the beacon duration in us
 */
#define BEACON_SYMB_DURATION_US( ) smtc_real_get_symbol_duration_us( lr1_beacon_obj->lr1_mac, BEACON_DATA_RATE( ) )
/**
 * @brief Compute the beacon duration in ms
 */
#define BEACON_SYMB_DURATION_MS( ) \
    ( ( ( BEACON_SYMB_DURATION_US( ) / 1000UL ) == 0 ) ? 1 : ( BEACON_SYMB_DURATION_US( ) / 1000UL ) )
/**
 * @brief Compute the beacon spreading factor
 */
#define GET_BEACON_SF( ) \
    ( ral_lora_sf_t ) get_beacon_sf( lr1_beacon_obj->lr1_mac, smtc_real_get_beacon_dr( lr1_beacon_obj->lr1_mac ) )
/**
 * @brief Compute the beacon bandwith
 */
#define GET_BEACON_BW( ) \
    ( ral_lora_bw_t ) get_beacon_bw( lr1_beacon_obj->lr1_mac, smtc_real_get_beacon_dr( lr1_beacon_obj->lr1_mac ) )
/**
 * @brief Compute the beacon payload length in bytes
 */
#define GET_BEACON_LENGTH_BYTES( ) \
    get_beacon_length( get_beacon_sf( lr1_beacon_obj->lr1_mac, smtc_real_get_beacon_dr( lr1_beacon_obj->lr1_mac ) ) )
/**
 * @brief Compute the beacon frequency in hertz
 */
#define GET_BEACON_FREQUENCY( ) \
    smtc_real_get_beacon_frequency( lr1_beacon_obj->lr1_mac, lr1_beacon_obj->beacon_epoch_time )
/**
 * @brief use to compute the rx windows size of a beacon defined in ms , this value is clamp at 255 symbols which is the
 * maximum allowed value in the semtech radio
 */
#define MAX_BEACON_WINDOW_SYMB( ) MIN( MAX_BEACON_WINDOW_MS / BEACON_SYMB_DURATION_MS( ), 255 )
/**
 * @brief return the duration in ms of a beacon duration initally defined in number of symbols
 */
#define TIME_MS_TO_BEACON_SYMB( N ) \
    ( MIN( MAX( ( N / BEACON_SYMB_DURATION_MS( ) ), MIN_BEACON_WINDOW_SYMB ), MAX_BEACON_WINDOW_SYMB( ) ) )
/**
 * @brief compute the dpll phase with a resultion of 1ms
 */
#define DPLL_PHASE_MS( ) lr1_beacon_obj->dpll_phase_100us / 10

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void smtc_beacon_sniff_init( smtc_lr1_beacon_t* lr1_beacon_obj, smtc_ping_slot_t* ping_slot_obj,
                             lr1_stack_mac_t* lr1_mac, radio_planner_t* rp, uint8_t beacon_sniff_id_rp,
                             void ( *push_callback )( void* push_context ), void* push_context )

{
    memset( lr1_beacon_obj, 0, sizeof( smtc_lr1_beacon_t ) );
    lr1_beacon_obj->rp                   = rp;
    lr1_beacon_obj->beacon_sniff_id_rp   = beacon_sniff_id_rp;
    lr1_beacon_obj->ping_slot_obj        = ping_slot_obj;
    lr1_beacon_obj->lr1_mac              = lr1_mac;
    lr1_beacon_obj->enabled              = false;
    lr1_beacon_obj->started              = false;
    lr1_beacon_obj->is_valid_beacon      = false;
    lr1_beacon_obj->beacon_state         = BEACON_UNLOCK;
    lr1_beacon_obj->push_callback        = push_callback;
    lr1_beacon_obj->push_context         = push_context;
    lr1_beacon_obj->dpll_frequency_100us = BEACON_PERIOD_MS * 10;
    lr1_beacon_obj->listen_beacon_rate   = DEFAULT_LISTEN_BEACON_RATE;

    rp_release_hook( lr1_beacon_obj->rp, lr1_beacon_obj->beacon_sniff_id_rp );
    rp_hook_init( lr1_beacon_obj->rp, lr1_beacon_obj->beacon_sniff_id_rp,
                  ( void ( * )( void* ) )( smtc_beacon_sniff_rp_callback ),
                  lr1_beacon_obj );  // no need to check return code because in case of error panic inside the function
}

void smtc_beacon_class_b_enable_service( smtc_lr1_beacon_t* lr1_beacon_obj, bool enable )
{
    if( enable == false )
    {
        smtc_beacon_sniff_stop( lr1_beacon_obj );
    }

    lr1_beacon_obj->enabled = enable;
}

void smtc_beacon_sniff_stop( smtc_lr1_beacon_t* lr1_beacon_obj )
{
    if( lr1_beacon_obj->enabled == false )
    {
        return;
    }
    lr1_beacon_obj->started = false;
    rp_task_abort( lr1_beacon_obj->rp, lr1_beacon_obj->beacon_sniff_id_rp );  // no need to check return code because in
                                                                              // case of error panic inside the function

    smtc_ping_slot_stop( lr1_beacon_obj->ping_slot_obj );
}

smtc_class_b_beacon_t smtc_beacon_sniff_start( smtc_lr1_beacon_t* lr1_beacon_obj )
{
    uint32_t rtc =
        smtc_modem_hal_get_time_in_ms( ) + 1000;  // +1000 to let time for rp enqueue task in case of abort task
    uint32_t seconds_since_epoch = 0;
    uint32_t fractional_second   = 0;
    uint8_t  beacon_id;
    rp_hook_get_id( lr1_beacon_obj->rp, lr1_beacon_obj, &beacon_id );  // no need to check return code because in case
                                                                       // of error panic inside the function

    lr1_beacon_obj->beacon_state = BEACON_UNLOCK;
    smtc_modem_hal_assert( beacon_id == lr1_beacon_obj->beacon_sniff_id_rp );
    if( lr1_beacon_obj->enabled == false )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "class_b_obj disabled\n" );
        return SMTC_CLASS_B_BEACON_NOT_ENABLED;
    }

    // if the gps epoch time isn't available in the lr1mac stack, the class b couldn't started
    if( lr1mac_core_convert_rtc_to_gps_epoch_time( lr1_beacon_obj->lr1_mac, rtc, &seconds_since_epoch,
                                                   &fractional_second ) == false )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "beacon sniff not started, time is not sync\n" );
        return SMTC_CLASS_B_BEACON_NOT_TIME_SYNC;
    }
    // init the digital pll phase to the closest gps epoch time multiple of BEACON_PERIOD_S. time is provided by the
    // stack with the device time request cmd. The format of the dpll_phase_100us isn't expressed in gps_epoch_time unit
    // but convert into the internal rtc millisecond counter. As a consequence dpll_phase_100us will contain the time of
    // the next target beacon time (in the internal rtc millisecond format)
    uint32_t pll_phase_temp;
    pll_phase_temp = 1000 * ( BEACON_PERIOD_S - ( seconds_since_epoch % BEACON_PERIOD_S ) );
    pll_phase_temp += rtc;
    pll_phase_temp -= fractional_second;
    lr1_beacon_obj->dpll_phase_100us       = pll_phase_temp * 10;  // dpll phase in 100us resolution
    lr1_beacon_obj->beacon_open_rx_nb_symb = MAX_BEACON_WINDOW_SYMB( );
    lr1_beacon_obj->started                = true;
    lr1mac_core_convert_rtc_to_gps_epoch_time( lr1_beacon_obj->lr1_mac, pll_phase_temp, &seconds_since_epoch,
                                               &fractional_second );
    // store the target gps epoch time (format gps epoch time) to lr1_beacon_obj->beacon_epoch_time
    lr1_beacon_obj->beacon_epoch_time = seconds_since_epoch;
    SMTC_MODEM_HAL_TRACE_PRINTF( "!!!!! seconds_since_epoch %u, fractional_second %u ms\n", seconds_since_epoch,
                                 fractional_second );
    // launch beacon radio sniff
    beacon_rp_request( lr1_beacon_obj );
    SMTC_MODEM_HAL_TRACE_PRINTF( "Next beacon in %u ms at %u\n", DPLL_PHASE_MS( ) - rtc,
                                 lr1_beacon_obj->dpll_phase_100us );

    return SMTC_CLASS_B_BEACON_OK;
}

void beacon_rp_request( smtc_lr1_beacon_t* lr1_beacon_obj )
{
    rp_task_t          rp_task = { 0 };
    ralf_params_lora_t lora_param;
    memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );
    rp_task.type                  = get_beacon_rp_task_type( lr1_beacon_obj );
    rp_task.hook_id               = lr1_beacon_obj->beacon_sniff_id_rp;
    rp_task.state                 = RP_TASK_STATE_SCHEDULE;
    rp_task.start_time_ms         = compute_start_time( lr1_beacon_obj );
    rp_task.duration_time_ms      = BEACON_SYMB_DURATION_MS( ) * lr1_beacon_obj->beacon_open_rx_nb_symb;
    rp_task.launch_task_callbacks = smtc_beacon_sniff_launch_callback_for_rp;

    rp_radio_params_t rp_radio_params      = { 0 };
    rp_radio_params.pkt_type               = RAL_PKT_TYPE_LORA;
    lora_param.symb_nb_timeout             = lr1_beacon_obj->beacon_open_rx_nb_symb;
    lora_param.sync_word                   = smtc_real_get_sync_word( lr1_beacon_obj->lr1_mac );
    lora_param.mod_params.cr               = smtc_real_get_coding_rate( lr1_beacon_obj->lr1_mac );
    lora_param.pkt_params.header_type      = RAL_LORA_PKT_IMPLICIT;
    lora_param.pkt_params.pld_len_in_bytes = GET_BEACON_LENGTH_BYTES( );
    lora_param.pkt_params.crc_is_on        = false;
    lora_param.pkt_params.invert_iq_is_on  = false;
    lora_param.mod_params.sf               = GET_BEACON_SF( );
    lora_param.mod_params.bw               = GET_BEACON_BW( );
    lora_param.rf_freq_in_hz               = GET_BEACON_FREQUENCY( );
    lora_param.mod_params.ldro = ral_compute_lora_ldro( lora_param.mod_params.sf, lora_param.mod_params.bw );
    lora_param.pkt_params.preamble_len_in_symb = BEACON_PREAMBLE_LENGTH_SYMB;
    rp_radio_params.rx.lora                    = lora_param;
    rp_radio_params.rx.timeout_in_ms           = 3000;
    lr1_beacon_obj->beacon_toa                 = ral_get_lora_time_on_air_in_ms(
        &( lr1_beacon_obj->rp->radio->ral ), ( &lora_param.pkt_params ), ( &lora_param.mod_params ) );
    smtc_modem_hal_assert( rp_task_enqueue( lr1_beacon_obj->rp, &rp_task, lr1_beacon_obj->beacon_buffer, BEACON_SIZE,
                                            &rp_radio_params ) == RP_HOOK_STATUS_OK );
}

// the function "smtc_beacon_sniff_launch_callback_for_rp" is called by the radio planner when the task enqueued inside
// beacon_rp_request is granted
void smtc_beacon_sniff_launch_callback_for_rp( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;
    if( rp->tasks[id].type == RP_TASK_TYPE_NONE )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "doesn't listen this beacon , jump it to save power \n" );
        rp_task_abort( rp, id );
        return;
    }
    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_assert( ralf_setup_lora( rp->radio, &rp->radio_params[id].rx.lora ) == RAL_STATUS_OK );
    smtc_modem_hal_assert( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT |
                                                                            RAL_IRQ_RX_HDR_ERROR |
                                                                            RAL_IRQ_RX_CRC_ERROR ) == RAL_STATUS_OK );
    // Wait the exact time
    while( ( int32_t )( rp->tasks[id].start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) > 0 )
    {
    }
    smtc_modem_hal_assert( ral_set_rx( &( rp->radio->ral ), rp->radio_params[id].rx.timeout_in_ms ) == RAL_STATUS_OK );
    rp_stats_set_rx_timestamp( &rp->stats, smtc_modem_hal_get_time_in_ms( ) );
}

void smtc_beacon_sniff_rp_callback( smtc_lr1_beacon_t* lr1_beacon_obj )
{
    if( lr1_beacon_obj->started == false )
    {
        return;
    }

    rp_status_t rp_status         = lr1_beacon_obj->rp->status[lr1_beacon_obj->beacon_sniff_id_rp];
    uint32_t    beacon_epoch_time = 0;
    // assuming that the radio hw latency between the end of the packet and the timestamp of the rx packet is about
    // 1/2 symbol
    uint32_t timestamp = lr1_beacon_obj->rp->irq_timestamp_100us[lr1_beacon_obj->beacon_sniff_id_rp] -
                         ( ( BEACON_SYMB_DURATION_US( ) >> 1 ) / 100 );

    SMTC_MODEM_HAL_TRACE_PRINTF( " beacon_timestamp_us = %u us\n", timestamp * 100 );
    lr1_beacon_obj->is_valid_beacon = false;
    if( rp_status == RP_STATUS_RX_PACKET )
    {
        beacon_epoch_time = smtc_decode_beacon_epoch_time( lr1_beacon_obj->beacon_buffer, GET_BEACON_SF( ) );
        lr1_beacon_obj->is_valid_beacon = is_valid_beacon( lr1_beacon_obj, timestamp );
        lr1_beacon_obj->beacon_buffer_length =
            ( uint8_t ) lr1_beacon_obj->rp->payload_size[lr1_beacon_obj->beacon_sniff_id_rp];
    }

    update_beacon_pll( lr1_beacon_obj, timestamp );
    update_beacon_state( lr1_beacon_obj );

    compute_beacon_metadata( lr1_beacon_obj, timestamp / 10, beacon_epoch_time );
    update_beacon_rx_nb_symb( lr1_beacon_obj, DPLL_PHASE_MS( ) );
    beacon_debug_print( lr1_beacon_obj );

    // The Network must have answered the pingslot request and the beacon must be locked to be considered ready
    // for class B
    if( ( lr1_beacon_obj->beacon_state == BEACON_LOCK ) &&
        ( lr1_beacon_obj->lr1_mac->ping_slot_info_user_req == USER_MAC_REQ_ACKED ) )
    {
        uint32_t current_beacon_phase_100us =
            ( lr1_beacon_obj->dpll_phase_100us - lr1_beacon_obj->dpll_frequency_100us );
        smtc_ping_slot_init_after_beacon( lr1_beacon_obj->ping_slot_obj, current_beacon_phase_100us, DPLL_PHASE_MS( ),
                                          BEACON_RESERVED_MS, BEACON_GUARD_MS, lr1_beacon_obj->beacon_epoch_time );

        smtc_ping_slot_start( lr1_beacon_obj->ping_slot_obj );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "beacon stop ping slot\n" );
        smtc_ping_slot_stop( lr1_beacon_obj->ping_slot_obj );
    }
    lr1_beacon_obj->beacon_epoch_time += BEACON_PERIOD_S;

    if( lr1_beacon_obj->beacon_state == BEACON_UNLOCK )
    {
        smtc_beacon_sniff_start( lr1_beacon_obj );
    }
    else
    {
        beacon_rp_request( lr1_beacon_obj );
        if( lr1mac_core_get_status_push_network_downlink_to_user( lr1_beacon_obj->lr1_mac ) == true )
        {
            lr1_beacon_obj->beacon_metadata.rx_metadata.rx_window = RECEIVE_ON_RXBEACON;
            lr1_beacon_obj->push_callback( lr1_beacon_obj->push_context );
        }
    }
}

void smtc_beacon_sniff_get_metadata( smtc_lr1_beacon_t* lr1_beacon_obj, smtc_beacon_metadata_t* beacon_metadata )
{
    memcpy( beacon_metadata, &lr1_beacon_obj->beacon_metadata, sizeof( smtc_beacon_metadata_t ) );
}

uint32_t smtc_decode_beacon_epoch_time( uint8_t* beacon_payload, uint8_t beacon_sf )
{
    // the format of the beacon payload is different according to the beacon sf. The following formula is a way to
    // abstact the RFU bytes position with a very simple way.
    uint16_t frame_crc    = beacon_payload[beacon_sf - 3] + ( beacon_payload[beacon_sf - 2] << 8 );
    uint16_t computed_crc = crc16_beacon( beacon_payload, beacon_sf - 3 );
    if( computed_crc != frame_crc )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "INVALID CRC  \n" );
        return 0;
    }
    else
    {
        return ( beacon_payload[beacon_sf - 7] + ( beacon_payload[beacon_sf - 6] << 8 ) +
                 ( beacon_payload[beacon_sf - 5] << 16 ) + ( beacon_payload[beacon_sf - 4] << 24 ) );
    }
}

uint8_t smtc_decode_beacon_param( uint8_t* beacon_payload, uint8_t beacon_sf )
{
    return beacon_payload[beacon_sf - 8];
}

bool smtc_decode_beacon_gw_specific( uint8_t* beacon_payload, uint8_t beacon_sf, uint8_t* gw_specific )
{
    uint16_t frame_crc = beacon_payload[get_beacon_length( beacon_sf ) - 2] +
                         ( beacon_payload[get_beacon_length( beacon_sf ) - 1] << 8 );
    uint16_t computed_crc =
        crc16_beacon( beacon_payload + ( beacon_sf - 1 ), get_beacon_length( beacon_sf ) - ( beacon_sf - 1 ) );

    if( computed_crc != frame_crc )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "INVALID CRC GwSpecific\n" );
        memset( gw_specific, 0, 7 );
        return false;
    }
    else
    {
        memcpy( gw_specific, beacon_payload + ( beacon_sf - 1 ), 7 );
        return true;
    }
}

/**********************************************/
/*           Private function                 */
/**********************************************/
static uint16_t crc16_beacon( uint8_t* data, uint16_t size )
{
    uint16_t crc = 0;
    if( size == 0 )
    {
        return 0;
    }
    for( unsigned int i = 0; i < size; ++i )
    {
        uint16_t dbyte = data[i];
        crc ^= dbyte << 8;

        for( unsigned char j = 0; j < 8; ++j )
        {
            uint16_t mix = crc & 0x8000;
            crc          = ( crc << 1 );
            if( mix )
                crc = crc ^ 0x1021;
        }
    }
    return crc;
}
static uint8_t get_beacon_sf( lr1_stack_mac_t* lr1_mac, uint8_t beacon_datarate )
{
    uint8_t            sf;
    lr1mac_bandwidth_t bw;
    smtc_real_lora_dr_to_sf_bw( lr1_mac, beacon_datarate, &sf, &bw );
    return sf;
}
static lr1mac_bandwidth_t get_beacon_bw( lr1_stack_mac_t* lr1_mac, uint8_t beacon_datarate )
{
    uint8_t            sf;
    lr1mac_bandwidth_t bw;
    smtc_real_lora_dr_to_sf_bw( lr1_mac, beacon_datarate, &sf, &bw );
    return bw;
}

static uint8_t get_beacon_length( uint8_t beacon_sf )
{
    uint8_t length = 0;
    switch( beacon_sf )
    {
    case 8:
        length = 19;
        break;
    case 9:
        length = 17;
        break;
    case 10:
        length = 19;
        break;
    case 11:
        length = 21;
        break;
    case 12:
        length = 23;
        break;
    default:
        smtc_modem_hal_mcu_panic( " invalid beacon sf " );
        break;
    }
    return length;
}
static rp_task_types_t get_beacon_rp_task_type( smtc_lr1_beacon_t* lr1_beacon_obj )
{
    if( ( ( ( lr1_beacon_obj->beacon_metadata.nb_beacon_missed + lr1_beacon_obj->beacon_metadata.nb_beacon_received ) %
            lr1_beacon_obj->listen_beacon_rate ) == 0 ) ||
        ( lr1_beacon_obj->beacon_state == BEACON_UNLOCK ) )
    {
        // to save power consumption , user can decide to not listen all the beacon setting the variable
        // listen_beacon_rate
        return RP_TASK_TYPE_RX_LORA;
    }
    else
    {
        return RP_TASK_TYPE_NONE;
    }
}

static void compute_beacon_metadata( smtc_lr1_beacon_t* lr1_beacon_obj, uint32_t timestamp, uint32_t beacon_epoch_time )
{
    lr1_beacon_obj->beacon_metadata.rx_metadata.rx_fpending_bit = 0;
    lr1_beacon_obj->beacon_metadata.rx_metadata.rx_fport        = 0;

    if( lr1_beacon_obj->is_valid_beacon == true )
    {
        lr1_beacon_obj->beacon_metadata.rx_metadata.timestamp = timestamp;
        lr1_beacon_obj->beacon_metadata.rx_metadata.rx_snr =
            lr1_beacon_obj->rp->radio_params[lr1_beacon_obj->beacon_sniff_id_rp].rx.lora_pkt_status.snr_pkt_in_db;
        lr1_beacon_obj->beacon_metadata.rx_metadata.rx_rssi =
            lr1_beacon_obj->rp->radio_params[lr1_beacon_obj->beacon_sniff_id_rp].rx.lora_pkt_status.rssi_pkt_in_dbm;
        lr1_beacon_obj->beacon_metadata.rx_metadata.rx_datarate     = BEACON_DATA_RATE( );
        lr1_beacon_obj->beacon_metadata.rx_metadata.rx_frequency_hz = GET_BEACON_FREQUENCY( );

        lr1_beacon_obj->beacon_metadata.last_beacon_received_timestamp = timestamp;
        lr1_beacon_obj->ping_slot_obj->last_valid_rx_beacon_ms         = timestamp;
        lr1_beacon_obj->beacon_metadata.nb_beacon_received++;
        lr1_beacon_obj->beacon_metadata.last_beacon_received_consecutively++;
        lr1_beacon_obj->beacon_metadata.last_beacon_lost_consecutively = 0;
        lr1_beacon_obj->beacon_metadata.four_last_beacon_rx_statistic =
            MIN( lr1_beacon_obj->beacon_metadata.four_last_beacon_rx_statistic + 1, 4 );
    }
    else
    {
        lr1_beacon_obj->beacon_metadata.nb_beacon_missed++;
        lr1_beacon_obj->beacon_metadata.last_beacon_received_consecutively = 0;
        lr1_beacon_obj->beacon_metadata.last_beacon_lost_consecutively++;
        if( lr1_beacon_obj->rp->tasks[lr1_beacon_obj->beacon_sniff_id_rp].type != RP_TASK_TYPE_NONE )
        {
            if( lr1_beacon_obj->beacon_metadata.four_last_beacon_rx_statistic > 0 )
            {
                lr1_beacon_obj->beacon_metadata.four_last_beacon_rx_statistic--;
            }
        }
    }
}

static void update_beacon_pll( smtc_lr1_beacon_t* lr1_beacon_obj, uint32_t timestamp )
{
    if( lr1_beacon_obj->is_valid_beacon == true )
    {
        if( lr1_beacon_obj->beacon_state == BEACON_UNLOCK )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF(
                "time error on first beacon = %d (100us resolution)  \n",
                timestamp - lr1_beacon_obj->dpll_phase_100us - 10 * lr1_beacon_obj->beacon_toa );
            lr1_beacon_obj->dpll_frequency_100us    = BEACON_PERIOD_MS * 10;
            lr1_beacon_obj->dpll_error              = 0;
            lr1_beacon_obj->dpll_error_wo_filtering = 0;
            lr1_beacon_obj->dpll_error_sum          = 0;
            lr1_beacon_obj->dpll_phase_100us        = timestamp - 10 * lr1_beacon_obj->beacon_toa;
        }
        if( lr1_beacon_obj->beacon_state == BEACON_LOCK )
        {
            lr1_beacon_obj->dpll_error_wo_filtering =
                timestamp - lr1_beacon_obj->dpll_phase_100us - ( 10 * lr1_beacon_obj->beacon_toa );
            lr1_beacon_obj->dpll_error =
                ( BEACON_PLL_PHASE_GAIN_MUL * lr1_beacon_obj->dpll_error + lr1_beacon_obj->dpll_error_wo_filtering ) /
                BEACON_PLL_PHASE_GAIN_DIV;
            lr1_beacon_obj->dpll_error_sum += lr1_beacon_obj->dpll_error;
            if( lr1_beacon_obj->dpll_error_sum > BEACON_PLL_FREQUENCY_GAIN )
            {
                lr1_beacon_obj->dpll_frequency_100us++;
                lr1_beacon_obj->dpll_error_sum = 0;
            }
            if( lr1_beacon_obj->dpll_error_sum < ( BEACON_PLL_FREQUENCY_GAIN * ( -1 ) ) )
            {
                lr1_beacon_obj->dpll_frequency_100us--;
                lr1_beacon_obj->dpll_error_sum = 0;
            }
        }
    }
    lr1_beacon_obj->dpll_phase_100us +=
        ( lr1_beacon_obj->dpll_frequency_100us ) + ( ( ABS( lr1_beacon_obj->dpll_error ) > 100 )
                                                         ? ( 100 * SIGN( lr1_beacon_obj->dpll_error ) )
                                                         : lr1_beacon_obj->dpll_error );
}
static void update_beacon_state( smtc_lr1_beacon_t* lr1_beacon_obj )
{
    if( ( lr1_beacon_obj->is_valid_beacon == false ) &&
        ( ( lr1_beacon_obj->beacon_open_rx_nb_symb >= MAX_BEACON_WINDOW_SYMB( ) ) ||
          ( lr1_beacon_obj->beacon_metadata.last_beacon_lost_consecutively > NB_OF_BEACON_BEFORE_DELOCK ) ) )
    {
        // Reach this point if no received beacon for a long period
        //=> step 1: The unlock state of the beacon state machine will stop the ping slot reception
        //=> step 2: when the ping slot are stopping the class_b_bit_stack is cleared
        //=> step 3: Supervisor will enqueue a downlink opportunitie frame to advertise the NS
        //=> step 4: Supervisor will notify the user that the ping slot is no more active
        //  Remark the beacon acquisition is still activated until the user itself decide/or not to deactivate the
        //  beacon acquisition
        lr1_beacon_obj->beacon_state = BEACON_UNLOCK;
    }
    else
    {
        // Don't update epoch with beacon because not authenticated authentificat
        // lr1_beacon_obj->beacon_epoch_time = beacon_epoch_time;

        lr1_beacon_obj->beacon_state = BEACON_LOCK;
    }
}
static void beacon_debug_print( smtc_lr1_beacon_t* lr1_beacon_obj )
{
    if( lr1_beacon_obj->is_valid_beacon == true )
    {
        SMTC_MODEM_HAL_TRACE_ARRAY( "Beacon Payload ", lr1_beacon_obj->beacon_buffer,
                                    lr1_beacon_obj->beacon_buffer_length );
    }

    SMTC_MODEM_HAL_TRACE_PRINTF( "--> PLL INFO ppl_phase =%d, pll_error_100us= %d  pll_frequency_100us = %d \n",
                                 lr1_beacon_obj->dpll_phase_100us, lr1_beacon_obj->dpll_error,
                                 lr1_beacon_obj->dpll_frequency_100us );
    SMTC_MODEM_HAL_TRACE_PRINTF( "\n********************************************\n" );
    SMTC_MODEM_HAL_TRACE_PRINTF(
        "-->BEACON STATUS \n received = %d\n missed = %d\n received_consecutively = %d\n lost_consecutively "
        "=%d\n four_last_beacon_rx_statistic = %d\n"
        " next beacon rx_nb_symb = %d\n",
        lr1_beacon_obj->beacon_metadata.nb_beacon_received, lr1_beacon_obj->beacon_metadata.nb_beacon_missed,
        lr1_beacon_obj->beacon_metadata.last_beacon_received_consecutively,
        lr1_beacon_obj->beacon_metadata.last_beacon_lost_consecutively,
        lr1_beacon_obj->beacon_metadata.four_last_beacon_rx_statistic, lr1_beacon_obj->beacon_open_rx_nb_symb );
    SMTC_MODEM_HAL_TRACE_PRINTF( "*********************************************\n " );
}
static void update_beacon_rx_nb_symb( smtc_lr1_beacon_t* lr1_beacon_obj, uint32_t target_time )
{
    if( lr1_beacon_obj->beacon_state == BEACON_UNLOCK )
    {
        lr1_beacon_obj->beacon_open_rx_nb_symb = MAX_BEACON_WINDOW_SYMB( );
    }
    else
    {
        uint32_t rx_timeout_symb_in_ms_tmp;  // unused for beacon
        uint32_t rx_timeout_symb_locked_in_ms_tmp;
        SMTC_MODEM_HAL_TRACE_PRINTF( "rx delay = %d ms\n",
                                     target_time - lr1_beacon_obj->beacon_metadata.last_beacon_received_timestamp );
        smtc_real_get_rx_window_parameters(
            lr1_beacon_obj->lr1_mac, BEACON_DATA_RATE( ),
            ( target_time - lr1_beacon_obj->beacon_metadata.last_beacon_received_timestamp ),
            &lr1_beacon_obj->beacon_open_rx_nb_symb, &rx_timeout_symb_in_ms_tmp, &rx_timeout_symb_locked_in_ms_tmp, 0 );
        // in case of beacon has not been YET received 4 times consecutively it enlarge the rx windows.
        if( lr1_beacon_obj->beacon_metadata.last_beacon_lost_consecutively == 0 )
        {
            lr1_beacon_obj->beacon_open_rx_nb_symb =
                MIN( lr1_beacon_obj->beacon_open_rx_nb_symb +
                         ( ( uint32_t )( 4 - lr1_beacon_obj->beacon_metadata.four_last_beacon_rx_statistic ) *
                           lr1_beacon_obj->beacon_open_rx_nb_symb ),
                     TIME_MS_TO_BEACON_SYMB( MAX_BEACON_WINDOW_MS ) );
        }
    }
}
static uint32_t compute_start_time( smtc_lr1_beacon_t* lr1_beacon_obj )
{
    int8_t  board_delay_ms = smtc_modem_hal_get_radio_tcxo_startup_delay_ms( ) + smtc_modem_hal_get_board_delay_ms( );
    int32_t rx_offset_ms;
    smtc_real_get_rx_start_time_offset_ms( lr1_beacon_obj->lr1_mac, BEACON_DATA_RATE( ), board_delay_ms,
                                           lr1_beacon_obj->beacon_open_rx_nb_symb, &rx_offset_ms );
    return ( DPLL_PHASE_MS( ) + rx_offset_ms );
}

static bool is_valid_beacon( smtc_lr1_beacon_t* lr1_beacon_obj, uint32_t timestamp )
{
    uint32_t seconds_since_epoch;
    uint32_t fractional_second;
    uint32_t beacon_epoch_time = smtc_decode_beacon_epoch_time( lr1_beacon_obj->beacon_buffer, GET_BEACON_SF( ) );
    bool     status =
        lr1mac_core_convert_rtc_to_gps_epoch_time( lr1_beacon_obj->lr1_mac, timestamp / 10 - lr1_beacon_obj->beacon_toa,
                                                   &seconds_since_epoch, &fractional_second );
    if( status == true )
    {
        int32_t check_time = ( beacon_epoch_time - seconds_since_epoch ) * 1000 - fractional_second;
        SMTC_MODEM_HAL_TRACE_PRINTF( " beacon_time - network time = %d ms\n", check_time );
        if( ( uint32_t ) ABS( check_time ) < MAX_BEACON_WINDOW_MS )
        {
            return true;
        }
        else
        {
            return true;  // will be manage in the future by a specific event
        }
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "device no more synchronized\n" )
    }
    return true;  // case receive a beacon but device no more synchronized
}
