/*!
 * \file      modem_core.h
 *
 * \brief     utilities for modem
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

#ifndef __MODEM_UTILITIES_H__
#define __MODEM_UTILITIES_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "fifo_ctrl.h"
#include "smtc_modem_api.h"
#include "smtc_modem_hal.h"
#include "lr1mac_defs.h"
#include "radio_planner.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define MODEM_MAX_TIME 0x1FFFFF

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Modem context return code
 */
typedef enum modem_ctx_rc_s
{
    MODEM_CTX_RC_SUCCESS,
    MODEM_CTX_RC_ERROR,
} modem_ctx_rc_t;

/**
 * @brief The parameter of the TxDone event indicates the status of the requested Tx
 *
 * @enum event_tx_done_state_t
 */
typedef enum event_tx_done_state_e
{
    MODEM_TX_FAILED           = 0,  //!< The frame was not sent
    MODEM_TX_SUCCESS          = 1,  //!< The frame was but not acknowledge
    MODEM_TX_SUCCESS_WITH_ACK = 2   //!< The frame was and acknowledge
} event_tx_done_state_t;

enum
{
    MODEM_DOWNLINK_UNCONSUMED = 0,
    MODEM_DOWNLINK_CONSUMED   = 1,
};
/**
 * @brief Downlink message structure
 *
 * @struct modem_downlink_msg_t
 */
typedef struct modem_downlink_msg_s
{
    uint8_t  port;          //!< LoRaWAN FPort
    uint8_t  data[242];     //!< data received
    uint8_t  length;        //!< data length in byte(s)
    int16_t  rssi;          //!< RSSI is a signed value in dBm + 64
    int16_t  snr;           //!< SNR is a signed value in 0.25 dB steps
    uint32_t timestamp;     //!< timestamp of the received message
    bool     fpending_bit;  //!< status of the frame pending bit
    uint32_t frequency_hz;  //!< Frequency of the received message
    uint8_t  datarate;      //!< Datarate of the received message
} modem_downlink_msg_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */
/*!
 * \brief  Init modem context
 * \retval void
 */
void modem_context_init_light( void ( *callback )( void ), radio_planner_t* rp );

/*!
 * \brief  get modem fifo
 * \retval void
 */
fifo_ctrl_t* modem_context_get_fifo_obj( void );

uint32_t modem_get_user_alarm( void );

void modem_set_user_alarm( uint32_t alarm );

/**
 * @brief Set the AppKey of the device
 * @remark In case lr11xx crypto element is used add a check on the key to avoid multiple un-needed key saving
 *
 * @param [in] app_key The LoRaWan 1.0.x application Key
 * @param [in] stack_id
 * @return modem_ctx_rc_t
 */
modem_ctx_rc_t modem_set_appkey( const uint8_t app_key[16], uint8_t stack_id );

/**
 * @brief Set test mode status
 *
 * @param [in] enable
 */
void modem_set_test_mode_status( bool enable );

/**
 * @brief Get test mode status
 *
 * @return true
 * @return false
 */
bool modem_get_test_mode_status( void );

/**
 * @brief set modem radio context
 *
 * @param [in] radio_ctx Radio context
 */
void modem_set_radio_ctx( const void* radio_ctx );

/**
 * @brief get modem radio context
 *
 * @return the pointer on radio context
 */
const void* modem_get_radio_ctx( void );

/**
 * @brief get modem rp
 *
 * @return the pointer on radio planner
 */
radio_planner_t* modem_get_rp( void );

/**
 * @brief Suspend radio access
 *
 * @return true if operation was ok, false otherwise
 */
bool modem_suspend_radio_access( void );

/**
 * @brief Resume radio access
 *
 * @return true if operation was ok, false otherwise
 */
bool modem_resume_radio_access( void );

/*!
 * \brief   Compute crc
 * \remark
 *
 * \param  [in]     buf*            - input buffer
 * \param  [in]     len*            - input buffer length
 * \retval [out]    crc             - computed crc
 */
uint32_t crc( const uint8_t* buf, int len );
uint8_t  crc8( const uint8_t* data, int length );

int32_t modem_duty_cycle_get_status( uint8_t stack_id );

/*!
 * @brief   return the modem status
 *
 * @param [in]    stack_id            - Stack identifier
 * @retval  uint8_t      bit 0 : reset after brownout
 *                       bit 1 : reset after panic
 *                       bit 2 : modem is muted
 *                       bit 3 : modem is joined
 *                       bit 4 : modem radio communication is suspended
 *                       bit 5 : file upload in progress
 *                       bit 6 : modem is trying to join the network
 *                       bit 7 : streaming in progress
 */
uint8_t modem_get_status( uint8_t stack_id );

/**
 * @brief Load current modem context
 */
void modem_load_modem_context( void );

/**
 * @brief Store modem context
 */
void modem_store_modem_context( void );

/**
 * @brief Get current reset counter value
 */
uint32_t modem_get_reset_counter( void );

#ifdef __cplusplus
}
#endif

#endif  // __MODEM_UTILITIES_H__

/* --- EOF ------------------------------------------------------------------ */
