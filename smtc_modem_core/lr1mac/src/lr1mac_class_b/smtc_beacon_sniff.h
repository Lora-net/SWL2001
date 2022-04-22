/*!
 * \file      smtc_beacon_sniff.h
 *
 * \brief     Beacon management for LoRaWAN class B devices
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

#ifndef __SMTC_BCN_SNIFF_H__
#define __SMTC_BCN_SNIFF_H__

#include <stdint.h>
#include <stdbool.h>

#include "lr1_stack_mac_layer.h"
#include "lr1mac_defs.h"
#include "radio_planner.h"
#include "smtc_ping_slot.h"
#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * API definitions
 * ============================================================================
 */
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
/**
 * @brief Beacon reserved interval in ms
 */
#define BEACON_RESERVED_MS ( 2120UL )
/**
 * @brief Beacon guard interval in ms
 */
#define BEACON_GUARD_MS ( 3000UL )
/**
 * @brief Max beacon size in bytes
 */
#define BEACON_SIZE ( 25UL )
/**
 * @brief Period in seconds between 2 beacons
 */
#define BEACON_PERIOD_S ( 128UL )
/**
 * @brief Period in milliseconds between 2 beacons
 */
#define BEACON_PERIOD_MS ( BEACON_PERIOD_S * 1000UL )
/**
 * @brief Preamble length in symbols of a beacon frame
 */
#define BEACON_PREAMBLE_LENGTH_SYMB ( 10UL )
/**
 * @brief Number of beacon missed before to decide that the session class b is lost
 */
#define NB_OF_BEACON_BEFORE_DELOCK ( 2 * 3600000UL / BEACON_PERIOD_MS )  //  2hours with beacon period
/**
 * @brief the smallest rx beacon windows size expressed in number of symbols
 */
#define MIN_BEACON_WINDOW_SYMB ( 6 )
/**
 * @brief the smallest rx beacon windows timeout expressed in ms.
 */
#define MIN_BEACON_WINDOW_MS ( 6 )
/**
 * @brief the largest rx beacon windows timeout expressed in ms
 */
#define MAX_BEACON_WINDOW_MS ( 2000UL )
/**
 * @brief the current implementation allow to not listen all the beacon to save power, the default_listen_beacon_rate
 * parameter allow to down sampled the beacon listening. For example setting the value 4 meaning that the device will
 * open 1 time a reception to receive the beacon and to stay in mute 3 times to save power
 */
#ifndef DEFAULT_LISTEN_BEACON_RATE
#define DEFAULT_LISTEN_BEACON_RATE ( 1 )
#endif
/**
 * @brief to track the Beacon it is implemented a digital (Phase Locked Loop), the BEACON_PLL_FREQUENCY_GAIN allow to
 * adjust the pll gain of the frequency loop. By reducing this value it increases the gain of frequency loop
 */
#define BEACON_PLL_FREQUENCY_GAIN ( 32 )
/**
 * @brief to track the Beacon it is implemented a digital pll, the phase loop gain of the pll is defined as :
 * BEACON_PLL_PHASE_GAIN_MUL/BEACON_PLL_PHASE_GAIN_DIV
 */
#define BEACON_PLL_PHASE_GAIN_MUL ( 7 )
#define BEACON_PLL_PHASE_GAIN_DIV ( 8 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACRO---- --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief beacon state of the beacon state machine
 * @remark the beacon state machine switch to locked as soon as a beacon is received and validated, the beacon state
 * machine switch to unlock if device hasn't received NB_OF_BEACON_BEFORE_DELOCK consecutively or after 2 hours without
 * receiving any valid beacons
 */
typedef enum beacon_state_e
{
    BEACON_UNLOCK,
    BEACON_LOCK,
} beacon_state_t;

/**
 * @brief return code when user launch the beacon acquisition process
 */
typedef enum smtc_class_b_beacon_e
{
    SMTC_CLASS_B_BEACON_OK = 0,           //!< launch the beacon acquisition process without error
    SMTC_CLASS_B_BEACON_NOT_ENABLED,      //!< can't launch the beacon acquisition process because the class b object is
                                          //!< not defined
    SMTC_CLASS_B_BEACON_ALREADY_STARTED,  //!< can't launch the beacon acquisition process because the class b object
                                          //!< is already started
    SMTC_CLASS_B_BEACON_NOT_TIME_SYNC,    //!< can't launch the beacon acquisition process because the epoch time isn't
                                          //!< available in the device
} smtc_class_b_beacon_t;

/**
 * @brief beacon statistics collection
 */
typedef struct smtc_beacon_metadata_s
{
    uint32_t nb_beacon_received;  //!< total number of valid received beacon since the process is BEACON_LOCK
    uint32_t nb_beacon_missed;    //!< total number of missed beacon since the process is BEACON_LOCK
    uint32_t last_beacon_received_consecutively;  //!< total number of beacon received consecutively
    uint32_t last_beacon_lost_consecutively;      //!< total number of beacon lost consecutively
    uint32_t last_beacon_received_timestamp;  //!< timestamp of the last valid received beacon defined in the local rtc
                                              //!< time based (in ms)
    uint8_t
        four_last_beacon_rx_statistic;  //!< return the numbers of valid received beacon during the 4 last period beacon
    lr1mac_down_metadata_t rx_metadata;  //!< usual reception metadata such as snr, rssi,..
} smtc_beacon_metadata_t;

/**
 * @brief Define the beacon  object used to collect and track the beacon during a  class b session
 */
typedef struct smtc_lr1_beacon_s
{
    smtc_ping_slot_t* ping_slot_obj;  //!< the beacon object embeds the ping slot object
    radio_planner_t*  rp;             //!< the beacon object embeds the radio planer object
    lr1_stack_mac_t*  lr1_mac;        //!< the beacon object embeds the lr1mac stack class a object
    uint8_t beacon_sniff_id_rp;      //!< the beacon acquisition requires a dedicated hook inside the radio planer, this
                                     //!< value defined this specific hook id, by changing this value we change the
                                     //!< prioritization of the beacon acquisition
    bool           is_valid_beacon;  //!< define if the last received beacon is valid
    bool           enabled;          //!< to enable the beacon acquisition
    bool           started;          //!< to launch the beacon acquisition
    uint8_t        beacon_buffer[BEACON_SIZE];  //!< the beacon payload
    uint8_t        beacon_buffer_length;        //!< the beacon payload length in bytes
    beacon_state_t beacon_state;                //!< the state of the beacon state machine
    uint32_t       beacon_epoch_time;           //!< the epoch time inside the last valid beacon
    uint16_t       beacon_open_rx_nb_symb;   //!< the duration in symbol of the rx time out of the next beacon reception
    uint32_t       beacon_toa;               //!< the beacon toa
    int32_t        dpll_error_wo_filtering;  //!< the internal digital pll phase error without filtering
    int32_t        dpll_error;               //!< the internal digital pll phase error after low pass filter
    int32_t        dpll_error_sum;           //!< the cumulative digital pll phase error after low pass filter
    uint32_t       dpll_frequency_100us;     //!< the digital pll frequency with a 0.1ms resolution
    uint32_t       dpll_phase_100us;         //!< the digital pll phase with a 0.1ms resolution
    uint8_t listen_beacon_rate;  //!< default value : DEFAULT_LISTEN_BEACON_RATE, referred to the explanation of this
                                 //!< default value to understood this parameter

    void ( *push_callback )( void* );  //!< this call back is used to push a valid beacon payload to the upper layer,
    void* push_context;  //!< the context given by the upper layer to transmit with the previous push_callback function

    smtc_beacon_metadata_t beacon_metadata;  // the beacon metadata
} smtc_lr1_beacon_t;

/**
 * @brief Init the beacon sniff process
 * @remark this function init the class b feature, this function is called only one time during the initialization of
 * the device after each reset
 * @param [in] lr1_beacon_obj     Beacon object
 * @param [in] ping_slot_obj      Ping slot object
 * @param [in] lr1_mac            Lr1mac object
 * @param [in] rp                 Radio planner object
 * @param [in] beacon_sniff_id_rp  Hook ID of the beacon acquisition thread for radio planner
 * @param [in] push_callback      Callback to push a valid received beacon to the upper layer (in LBM a downlink fifo)
 * @param [in] push_context       User context transmit with the push callback function
 */

void smtc_beacon_sniff_init( smtc_lr1_beacon_t* lr1_beacon_obj, smtc_ping_slot_t* ping_slot_obj,
                             lr1_stack_mac_t* lr1_mac, radio_planner_t* rp, uint8_t beacon_snif_id_rp,
                             void ( *push_callback )( void* push_context ), void* push_context );

/**
 * @brief Beacon service enablement
 *
 * @param [in,out] lr1_beacon_obj  Beacon object
 * @param [in] enable          to enable the beacon acquisition
 */
void smtc_beacon_class_b_enable_service( smtc_lr1_beacon_t* lr1_beacon_obj, bool enable );

/**
 * @brief Stop beacon windows
 *
 * @param [in,out] lr1_beacon_obj Beacon object
 */
void smtc_beacon_sniff_stop( smtc_lr1_beacon_t* lr1_beacon_obj );

/**
 * @brief Start beacon acquisition
 * @remark this function is launch when you enable the class b session, it managed the first beacon acquisition,
 * @param [in,out] lr1_beacon_obj  Beacon object
 * @return return code as defined in @ref smtc_class_b_beacon_t
 * @retval SMTC_CLASS_B_BEACON_OK                beacon acquisition process without error
 * @retval SMTC_CLASS_B_BEACON_NOT_ENABLED       beacon acquisition process failed because beacon object is not enabled
 * @retval SMTC_CLASS_B_BEACON_ALREADY_STARTED   beacon acquisition process failed because acquisition is already
 * started
 * @retval  SMTC_CLASS_B_BEACON_NOT_TIME_SYNC    beacon acquisition process failed beacause the epoch time isn't
 * available in the device
 */
smtc_class_b_beacon_t smtc_beacon_sniff_start( smtc_lr1_beacon_t* lr1_beacon_obj );

/**
 * @brief Callback called by radio planner after the radio interrupt. this callback perform the validation of a received
 * beacon before to execute the push_callback
 *
 * @param [in,out] lr1_beacon_obj Beacon object
 */
void smtc_beacon_sniff_rp_callback( smtc_lr1_beacon_t* lr1_beacon_obj );

/**
 * @brief Get beacon statistics
 *
 * @param [in] lr1_beacon_obj Beacon object
 * @param [out] beacon_metadata return beacon metadata as defined in @ref smtc_beacon_metadata_t
 */
void smtc_beacon_sniff_get_metadata( smtc_lr1_beacon_t* lr1_beacon_obj, smtc_beacon_metadata_t* beacon_metadata );

/**
 * @brief Decode the epoch time field in beacon payload
 * @remark the beacon payload format is dependant of the spreading factor
 * @param [in] beacon_payload payload inside the beacon
 * @param [in] beacon_sf      the spreading factor of the beacon
 * @return uint32_t      the epoch time defined in seconds
 */
uint32_t smtc_decode_beacon_epoch_time( uint8_t* beacon_payload, uint8_t beacon_sf );

/**
 * @brief Decode the param field in beacon payload
 * @remark the beacon payload format is dependant of the spreading factor
 * @param beacon_payload   payload inside the beacon
 * @param beacon_sf the spreading factor of the beacon
 * @return uint8_t return the param field inside the beacon payload
 */
uint8_t smtc_decode_beacon_param( uint8_t* beacon_payload, uint8_t beacon_sf );

/**
 * @brief Decode the gateway specific field in beacon payload
 * @remark implemented only for certification purpose,, the gw specific fields aren't transmit to the upper layer
 * @param [in,out] beacon_payload payload inside the beacon
 * @param [in] beacon_sf the spreading factor of the beacon
 * @param [out] gw_specific return the gw_specific field inside the beacon
 * @return bool return false if the crc check failed
 */
bool smtc_decode_beacon_gw_specific( uint8_t* beacon_payload, uint8_t beacon_sf, uint8_t* gw_specific );

#ifdef __cplusplus
}
#endif

#endif  //  __SMTC_BCN_SNIFF_H__
