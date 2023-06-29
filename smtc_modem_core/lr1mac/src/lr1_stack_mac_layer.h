/*!
 * \file      lr1_stack_mac_layer.h
 *
 * \brief     LoRaWan stack mac layer definition
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

#ifndef __LR1_STACK_MAC_LAYER_H__
#define __LR1_STACK_MAC_LAYER_H__
#ifdef __cplusplus
extern "C" {
#endif
/*
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */
#include "lr1mac_defs.h"
#include "smtc_real_defs.h"
#include "radio_planner.h"
#include "smtc_duty_cycle.h"
#include "smtc_lbt.h"

#ifndef MIN_RX_WINDOW_SYMB
#define MIN_RX_WINDOW_SYMB 6  // open rx window at least 6 symbols
#endif

#define MAX_RX_WINDOW_SYMB 248  // open rx window at max 248 symbol hardware limitation

#ifndef MIN_RX_WINDOW_DURATION_MS
#define MIN_RX_WINDOW_DURATION_MS 16  // open rx window at least 6ms
#endif
/*
 *-----------------------------------------------------------------------------------
 * --- PUBLIC TYPES -----------------------------------------------------------------
 */
typedef struct lr1_stack_mac_s
{
    mac_context_t mac_context;
    smtc_real_t*  real;  // Region Abstraction Layer
    smtc_dtc_t*   dtc_obj;
    smtc_lbt_t*   lbt_obj;

    void ( *push_callback )( void* );
    void* push_context;

    uint16_t nb_of_reset;
    /* LoraWan Context */

    /*******************************************/
    /*      Update by Link ADR command         */
    /*******************************************/
    uint8_t tx_data_rate;
    uint8_t tx_data_rate_adr;
    int8_t  tx_power;
    uint8_t nb_trans;
    uint8_t nb_trans_cpt;
    /********************************************/
    /*     Update by RxParamaSetupRequest       */
    /********************************************/
    uint8_t  rx2_data_rate;
    uint32_t rx2_frequency;
    uint8_t  rx1_dr_offset;
    /********************************************/
    /*     Update by NewChannelReq command      */
    /********************************************/
    // NbOfActiveChannel value are valid

    /********************************************/
    /*   Update by RxTimingSetupReq command     */
    /********************************************/
    uint8_t rx1_delay_s;
    /********************************************/
    /*   Update by TxParamSetupReq command      */
    /********************************************/
    uint8_t max_erp_dbm;
    /********************************************/
    /*   Update by DutyCycleReq command         */
    /********************************************/
    uint32_t max_duty_cycle_index;
    /********************************************/
    /*   Other Data To store                    */
    /********************************************/
    uint32_t                 fcnt_up;
    uint32_t                 fcnt_dwn;
    uint32_t                 dev_addr;
    lr1mac_activation_mode_t activation_mode;

    // LoRaWan Mac Data for uplink
    uint8_t              tx_fport;
    bool                 tx_fport_present;
    lr1mac_layer_param_t tx_mtype;
    uint8_t              tx_major_bits;
    uint8_t              tx_fctrl;
    uint8_t              tx_ack_bit;
    uint8_t              tx_class_b_bit;
    uint8_t              app_payload_size;
    uint8_t              tx_payload_size;
    uint8_t              tx_payload[255];
    uint8_t              tx_fopts_length;
    uint8_t              tx_fopts_data[DEVICE_MAC_PAYLOAD_MAX_SIZE];
    uint8_t              tx_fopts_lengthsticky;
    uint8_t              tx_fopts_datasticky[15];
    uint8_t              tx_fopts_current_length;
    uint8_t              tx_fopts_current_data[15];
    // LoRaWan Mac Data for downlink
    lr1mac_layer_param_t  rx_ftype;
    uint8_t               rx_major;
    uint8_t               rx_fctrl;
    uint8_t               rx_ack_bit;
    uint8_t               rx_fpending_bit_current;
    uint8_t               rx_fopts_length;
    uint8_t               rx_fopts[15];
    uint8_t               rx_payload_size;  //@note Have to by replace by a fifo objet to manage class c
    uint8_t               rx_payload[255];  //@note Have to by replace by a fifo objet to manage class c
    uint8_t               rx_payload_empty;
    user_rx_packet_type_t available_app_packet;
    rx_packet_type_t      valid_rx_packet;

    // LoRaWan Mac Data for duty-cycle
    uint32_t tx_duty_cycle_time_off_ms;
    uint32_t tx_duty_cycle_timestamp_ms;

    // LoRaWan Mac Data for join
    uint16_t dev_nonce;
    uint8_t  join_nonce[6];  // Join_nonce + NetId
    uint8_t  cf_list[16];

    // LoRaWan Mac Data for nwk Ans
    uint8_t nwk_payload[NWK_MAC_PAYLOAD_MAX_SIZE];  //@note resize this buffer
    uint8_t nwk_payload_size;

    uint8_t nwk_ans[DEVICE_MAC_PAYLOAD_MAX_SIZE];  //@note reuse user payload data or at least
                                                   // reduce size or use opt byte
    uint8_t nwk_ans_size;

    // LoraWan Config
    int           adr_ack_cnt;
    uint8_t       adr_ack_delay_init;
    uint8_t       adr_ack_limit_init;
    uint8_t       adr_ack_limit;
    uint8_t       adr_ack_delay;
    uint8_t       adr_ack_req;
    bool          adr_enable;
    dr_strategy_t adr_mode_select;
    dr_strategy_t adr_mode_select_tmp;
    uint32_t      adr_custom[2];
    uint16_t      no_rx_packet_reset_threshold;
    uint16_t      no_rx_packet_count;
    uint16_t      no_rx_packet_count_in_mobile_mode;

    // Join Duty cycle management
    uint32_t next_time_to_join_seconds;
    uint32_t retry_join_cpt;
    uint32_t first_join_timestamp;

    uint32_t      tx_frequency;
    uint32_t      rx1_frequency;
    uint8_t       rx_data_rate;
    uint8_t       sync_word;  // TODO does it useful , it's on the real
    rx_win_type_t current_win;

    // initially implemented in phy layer
    lr1mac_radio_state_t   radio_process_state;
    radio_planner_t*       rp;
    uint8_t                stack_id4rp;
    uint32_t               rx_timeout_ms;
    uint32_t               rx_timeout_symb_in_ms;
    uint16_t               rx_window_symb;
    join_status_t          join_status;
    rp_status_t            planner_status;
    lr1mac_down_metadata_t rx_metadata;
    uint32_t               isr_tx_done_radio_timestamp;
    int16_t                fine_tune_board_setting_delay_ms[16];
    int32_t                rx_offset_ms;
    uint32_t               timestamp_failsafe;
    uint8_t                type_of_ans_to_send;
    uint8_t                nwk_payload_index;
    lr1mac_states_t        lr1mac_state;
    uint32_t               rtc_target_timer_ms;
    uint8_t                send_at_time;
    bool                   available_link_adr;
    uint8_t                is_lorawan_modem_certification_enabled;
    uint32_t               crystal_error;

    // LinkCheck
    uint8_t link_check_margin;
    uint8_t link_check_gw_cnt;

    // Network Device Time
    uint32_t seconds_since_epoch;
    uint32_t fractional_second;
    uint32_t timestamp_tx_done_device_time_req_ms;
    uint32_t timestamp_tx_done_device_time_req_ms_tmp;
    uint32_t timestamp_last_device_time_ans_s;
    uint32_t device_time_invalid_delay_s;

    // MAC command requested by user
    user_mac_req_status_t link_check_user_req;
    user_mac_req_status_t device_time_user_req;
    user_mac_req_status_t ping_slot_info_user_req;

    // Class B
    uint32_t beacon_freq_hz;
    uint32_t ping_slot_freq_hz;
    uint8_t  ping_slot_dr;
    uint8_t  ping_slot_periodicity_req;  // Value Requested by the user
    uint8_t  ping_slot_periodicity_ans;  // Value Acknowledged by the Network

    // Downlink Network
    bool push_network_downlink_to_user;
} lr1_stack_mac_t;

/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------------
 */

/**
 * @brief Initialize mac layer
 *
 * @param [in] lr1_mac         The Lr1mac object
 * @param [in] activation_mode Type of activation used (ABP or OTAA)
 * @param [in] region          Chosen region
 */
void lr1_stack_mac_init( lr1_stack_mac_t* lr1_mac, lr1mac_activation_mode_t activation_mode );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */

/**
 * @brief
 *
 * @param lr1_mac
 */
void lr1_stack_mac_session_init( lr1_stack_mac_t* lr1_mac );

/**
 * @brief
 *
 * @param lr1_mac
 */
void lr1_stack_mac_region_init( lr1_stack_mac_t* lr1_mac, smtc_real_region_types_t region_type );

/**
 * @brief
 *
 * @param lr1_mac
 */
void lr1_stack_mac_region_config( lr1_stack_mac_t* lr1_mac );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_tx_frame_build( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_tx_frame_encrypt( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_tx_radio_start( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_tx_radio_free_lbt( lr1_stack_mac_t* lr1_mac );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_radio_busy_lbt( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_radio_abort_lbt( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_rx_radio_start( lr1_stack_mac_t* lr1_mac, const rx_win_type_t type, const uint32_t time_to_start );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_rp_callback( lr1_stack_mac_t* lr1_mac );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_rx_timer_configure( lr1_stack_mac_t* lr1_mac, const rx_win_type_t type );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
rx_packet_type_t lr1_stack_mac_rx_frame_decode( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_update_tx_done( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_update( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t lr1_stack_mac_cmd_parse( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_join_request_build( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t lr1_stack_mac_join_accept( lr1_stack_mac_t* lr1_mac );

/*!
 * \brief lr1_stack_network_next_free_duty_cycle_ms_get
 * \remark duty cycle time off left about the network duty-cycle request
 * \param [IN]  lr1_stack_mac_t
 * \return duty-cycle time-off left, 0 if no pending limitation
 */
int32_t lr1_stack_network_next_free_duty_cycle_ms_get( lr1_stack_mac_t* lr1_mac );

/*!
 * \brief lr1_stack_toa_get
 * \remark
 * \param [IN]  lr1_stack_mac_t
 * \return toa of the urrent tx frame
 */
uint32_t lr1_stack_toa_get( lr1_stack_mac_t* lr1_mac );

/**
 * @brief
 *
 * @param lr1_mac
 * @return uint8_t return the Nb Trans configured
 */
uint8_t lr1_stack_nb_trans_get( lr1_stack_mac_t* lr1_mac );

/**
 * @brief
 *
 * @param lr1_mac
 * @param [in] nb_trans set the Number of NbTrans
 * @return status_lorawan_t
 */
status_lorawan_t lr1_stack_nb_trans_set( lr1_stack_mac_t* lr1_mac, uint8_t nb_trans );

/**
 * @brief
 *
 * @param lr1_mac
 * @return uint32_t
 */
uint32_t lr1_stack_get_crystal_error( lr1_stack_mac_t* lr1_mac );

/**
 * @brief
 *
 * @param lr1_mac
 * @param [in] crystal_error
 */
void lr1_stack_set_crystal_error( lr1_stack_mac_t* lr1_mac, uint32_t crystal_error );

/**
 * @brief
 *
 * @param lr1_mac
 * @param size
 * @return status_lorawan_t
 */
status_lorawan_t lr1mac_rx_payload_max_size_check( lr1_stack_mac_t* lr1_mac, uint8_t size, uint8_t rx_datarate );

/*!
 * \brief tx launch call back for rp
 * \remark
 * \param
 * \return
 */
void lr1_stack_mac_tx_lora_launch_callback_for_rp( void* rp_void );
void lr1_stack_mac_tx_gfsk_launch_callback_for_rp( void* rp_void );
void lr1_stack_mac_tx_lr_fhss_launch_callback_for_rp( void* rp_void );
void lr1_stack_mac_rx_lora_launch_callback_for_rp( void* rp_void );
void lr1_stack_mac_rx_gfsk_launch_callback_for_rp( void* rp_void );

#ifdef __cplusplus
}
#endif

#endif  // __LR1_STACK_MAC_LAYER_H__
