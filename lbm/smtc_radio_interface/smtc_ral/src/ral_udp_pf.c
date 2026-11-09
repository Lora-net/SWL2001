/**
 * @file      ral_udp_pf.c
 *
 * @brief     Radio abstraction layer implementation for UDP Packet Forwarder protocol
 *
 * @details   This RAL implementation replaces physical radio hardware with
 *            UDP communication using the Semtech Packet Forwarder JSON protocol.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the disclaimer.
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

// Enable POSIX features for getaddrinfo, struct timeval, etc.
#define _POSIX_C_SOURCE 200112L

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>       // snprintf
#include <stdlib.h>      // rand
#include <sys/socket.h>  // socket, connect, send
#include <sys/time.h>    // struct timeval
#include <netinet/in.h>  // sockaddr_in
#include <arpa/inet.h>   // inet_pton
#include <netdb.h>       // getaddrinfo
#include <unistd.h>      // close
#include <pthread.h>     // pthread_create, pthread_mutex, pthread_cond
#include <poll.h>        // poll
#include <errno.h>       // errno
#include <fcntl.h>       // fcntl, O_NONBLOCK
#include "ral_udp_pf.h"
#include "base64.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

// Forward declarations of HAL functions (to avoid including smtc_modem_hal.h)
extern void     smtc_modem_hal_start_timer( const uint32_t milliseconds, void ( *callback )( void* context ),
                                            void*          context );
extern uint32_t smtc_modem_hal_get_time_in_ms( void );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

// UDP Packet Forwarder protocol constants
#define PROTOCOL_VERSION 2
#define PKT_PUSH_DATA 0
#define PKT_PUSH_ACK 1
#define PKT_PULL_DATA 2
#define PKT_PULL_RESP 3
#define PKT_PULL_ACK 4
#define PKT_TX_ACK 5

// Environment variable names for runtime configuration
#define UDP_PF_SERVER_ADDR_ENV_VAR_NAME "UDP_PF_SERVER_ADDR"
#define UDP_PF_SERVER_PORT_ENV_VAR_NAME "UDP_PF_SERVER_PORT"
#define UDP_PF_GATEWAY_EUI_ENV_VAR_NAME "UDP_PF_GATEWAY_EUI"

// Default gateway configuration (used if environment variables are not set)
#define UDP_PF_DEFAULT_SERVER_ADDR "127.0.0.1"
#define UDP_PF_DEFAULT_SERVER_PORT "1700"
#define UDP_PF_DEFAULT_GATEWAY_EUI "000000FFFE000000"

// Default uplink packet metadata (hardcoded values for rxpk JSON)
#define UDP_PF_DEFAULT_CHAN 0     // Channel index
#define UDP_PF_DEFAULT_RFCH 0     // RF chain index
#define UDP_PF_DEFAULT_RSSI -100  // RSSI in dBm
#define UDP_PF_DEFAULT_LSNR 10.0  // LoRa SNR in dB

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Context structure for UDP Packet Forwarder virtual radio
 */
typedef struct
{
    // UDP sockets
    int     sock_up;         // Upstream socket for PUSH_DATA (-1 if not initialized)
    int     sock_down;       // Downstream socket for PULL_DATA/PULL_RESP (-1 if not initialized)
    bool    connected;       // Sockets connection status
    uint8_t gateway_eui[8];  // Gateway unique identifier

    // TX payload buffer (from set_pkt_payload)
    uint8_t  tx_payload[255];
    uint16_t tx_payload_size;

    // RX payload buffer (from UDP downlink)
    uint8_t  rx_payload[255];
    uint16_t rx_payload_size;

    // Radio configuration (stored from set_xxx_params functions)
    uint32_t rf_freq_in_hz;
    int8_t   output_pwr_in_dbm;

    // Packet type and parameters for TX/RX
    ral_pkt_type_t        pkt_type;
    ral_lora_pkt_params_t lora_pkt_params;
    ral_lora_mod_params_t lora_mod_params;
    ral_gfsk_pkt_params_t gfsk_pkt_params;
    ral_gfsk_mod_params_t gfsk_mod_params;

    // LR-FHSS configuration (lr_fhss_mode replaces pkt_type for LR-FHSS since no RAL_PKT_TYPE_LR_FHSS exists)
    bool                 lr_fhss_mode;
    ral_lr_fhss_params_t lr_fhss_params;
    uint16_t             lr_fhss_hop_sequence_id;

    // Shared work buffer for TX/RX packet assembly (TX and RX are mutually exclusive)
    uint8_t work_buffer[1024];

} ral_udp_pf_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

// Static context for UDP virtual radio IRQ simulation
static ral_irq_t pending_irq                  = RAL_IRQ_NONE;
static void ( *stored_irq_callback )( void* ) = NULL;
static void* stored_irq_context               = NULL;

// Runtime configuration (initialized from env vars or defaults in load_config_from_env)
static char    udp_pf_server_addr[256];
static char    udp_pf_server_port[16];
static uint8_t udp_pf_gateway_eui[8];

// UDP Packet Forwarder context
static ral_udp_pf_context_t udp_ctx = {
    .sock_up   = -1,
    .sock_down = -1,
    .connected = false,
};

// RX worker thread state (makes ral_udp_pf_set_rx non-blocking, like real radio hardware).
// A persistent thread + cancel pipe is the minimal approach that satisfies:
//   - non-blocking set_rx() return
//   - responsive cancellation from set_standby()/set_sleep()
//   - re-entrancy (callback may call set_rx() again from within the thread)
//   - soft reset safety (longjmp preserves the thread)
static pthread_t       rx_thread;
static pthread_mutex_t rx_mutex                = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  rx_start_cond           = PTHREAD_COND_INITIALIZER;
static pthread_cond_t  rx_done_cond            = PTHREAD_COND_INITIALIZER;
static bool            rx_thread_created       = false;
static volatile bool   rx_start_requested      = false;
static volatile bool   rx_busy                 = false;
static volatile bool   rx_cancel               = false;
static uint32_t        rx_requested_timeout_ms = 0;
static int             rx_cancel_pipe[2]       = { -1, -1 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Parse hex string to bytes (e.g., "000000FFFE000000" -> {0x00,0x00,0x00,0xFF,0xFE,0x00,0x00,0x00})
 *
 * @param hex_str Input hex string (must have even number of characters)
 * @param bytes   Output byte array
 * @param len     Expected number of bytes to parse
 * @return true on success, false on invalid input
 */
static bool parse_hex_string( const char* hex_str, uint8_t* bytes, size_t len );

/**
 * @brief Load configuration from environment variables (called once at init)
 */
static void load_config_from_env( void );

/**
 * @brief Timer callback for TX_DONE IRQ simulation
 */
static void ral_udp_pf_tx_done_timer_callback( void* context );

/**
 * @brief Timer callback for RX_TIMEOUT IRQ simulation
 */
static void ral_udp_pf_rx_timeout_timer_callback( void* context );

/**
 * @brief Build PUSH_DATA packet with rxpk JSON payload
 */
static int build_push_data_packet( uint8_t* buff, size_t max_len );

/**
 * @brief Build LoRa datarate string (e.g. "SF7BW125")
 */
static int build_datr_string( char* buff, size_t max_len, ral_lora_sf_t sf, ral_lora_bw_t bw );

/**
 * @brief Build LoRa coding rate string (e.g. "4/5")
 */
static int build_codr_string( char* buff, size_t max_len, ral_lora_cr_t cr );

/**
 * @brief Build LR-FHSS coding rate string (e.g. "1/3")
 */
static int build_lr_fhss_codr_string( char* buff, size_t max_len, lr_fhss_v1_cr_t cr );

/**
 * @brief Build LR-FHSS datarate string (e.g. "M0BW137")
 */
static int build_lr_fhss_datr_string( char* buff, size_t max_len, lr_fhss_v1_bw_t bw );

/**
 * @brief Send PULL_DATA packet to server
 */
static int send_pull_data( void );

/**
 * @brief Parse PULL_RESP packet and extract downlink payload
 */
static int parse_pull_resp( const uint8_t* packet, size_t packet_len, uint8_t* payload_out, uint16_t* size_out );

/**
 * @brief Send TX_ACK packet to server after transmitting a downlink
 */
static void send_tx_ack( uint8_t token_h, uint8_t token_l );

/**
 * @brief RX worker thread function (runs blocking recv loop in background)
 */
static void* rx_worker_thread_func( void* arg );

/**
 * @brief Perform the blocking UDP receive, cancelable via rx_cancel_pipe
 * @return true if a downlink PULL_RESP was received, false on timeout/cancel
 */
static bool rx_do_recv( uint32_t timeout_ms );

/**
 * @brief Cancel any in-progress RX operation and wait for the worker to become idle
 */
static void rx_cancel_in_progress( void );

/**
 * @brief Create the RX worker thread and cancel pipe (idempotent, called on first RX)
 */
static void rx_ensure_thread_started( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

bool ral_udp_pf_handles_part( const char* part_number )
{
    return ( strcmp( "udp_pf", part_number ) == 0 );
}

ral_status_t ral_udp_pf_reset( const void* context )
{
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_init( const void* context )
{
    ( void ) context;
    struct addrinfo hints, *result, *q;
    int             i;

    pending_irq = RAL_IRQ_NONE;

    // Cancel any in-progress RX (safety net for soft reset via longjmp)
    rx_cancel_in_progress( );

    // Load configuration from environment variables (or use defaults)
    load_config_from_env( );

    // Copy gateway EUI to context
    memcpy( udp_ctx.gateway_eui, udp_pf_gateway_eui, 8 );

    // If sockets already initialized, skip
    if( udp_ctx.sock_up >= 0 && udp_ctx.sock_down >= 0 )
    {
        return RAL_STATUS_OK;
    }

    // Resolve server address
    memset( &hints, 0, sizeof hints );
    hints.ai_family   = AF_UNSPEC;   // IPv4 or IPv6
    hints.ai_socktype = SOCK_DGRAM;  // UDP

    i = getaddrinfo( udp_pf_server_addr, udp_pf_server_port, &hints, &result );
    if( i != 0 )
    {
        printf( "getaddrinfo failed: %s\n", gai_strerror( i ) );
        return RAL_STATUS_ERROR;
    }

    // Create upstream socket (for PUSH_DATA)
    for( q = result; q != NULL; q = q->ai_next )
    {
        udp_ctx.sock_up = socket( q->ai_family, q->ai_socktype, q->ai_protocol );
        if( udp_ctx.sock_up == -1 )
            continue;
        else
            break;
    }

    if( q == NULL )
    {
        freeaddrinfo( result );
        printf( "socket failed\n" );
        return RAL_STATUS_ERROR;
    }

    // Connect upstream socket
    i = connect( udp_ctx.sock_up, q->ai_addr, q->ai_addrlen );
    if( i != 0 )
    {
        close( udp_ctx.sock_up );
        udp_ctx.sock_up = -1;
        freeaddrinfo( result );
        printf( "connect failed\n" );
        return RAL_STATUS_ERROR;
    }

    // Create downstream socket (for PULL_DATA/PULL_RESP)
    // Reuse the same address info
    udp_ctx.sock_down = socket( q->ai_family, q->ai_socktype, q->ai_protocol );
    if( udp_ctx.sock_down == -1 )
    {
        close( udp_ctx.sock_up );
        udp_ctx.sock_up = -1;
        freeaddrinfo( result );
        printf( "socket failed\n" );
        return RAL_STATUS_ERROR;
    }

    // Connect downstream socket
    i = connect( udp_ctx.sock_down, q->ai_addr, q->ai_addrlen );
    freeaddrinfo( result );

    if( i != 0 )
    {
        close( udp_ctx.sock_up );
        close( udp_ctx.sock_down );
        udp_ctx.sock_up   = -1;
        udp_ctx.sock_down = -1;
        printf( "connect failed\n" );
        return RAL_STATUS_ERROR;
    }

    // Set socket timeout for upstream (short timeout for ACKs)
    struct timeval tv_up;
    tv_up.tv_sec  = 0;
    tv_up.tv_usec = 100000;  // 100ms timeout
    setsockopt( udp_ctx.sock_up, SOL_SOCKET, SO_RCVTIMEO, &tv_up, sizeof( tv_up ) );

    udp_ctx.connected = true;

    // Send initial PULL_DATA to open return path
    send_pull_data( );

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_wakeup( const void* context )
{
    // No sleep mode for UDP virtual radio
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_sleep( const void* context, const bool retain_config )
{
    ( void ) context;
    ( void ) retain_config;
    rx_cancel_in_progress( );
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_standby( const void* context, ral_standby_cfg_t standby_cfg )
{
    ( void ) context;
    ( void ) standby_cfg;
    rx_cancel_in_progress( );
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_fs( const void* context )
{
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_check_lora_parameters( const void* context, const uint32_t rf_freq_in_hz,
                                               const ral_lora_pkt_params_t* pkt_params,
                                               const ral_lora_mod_params_t* mod_params )
{
    ( void ) context;
    ( void ) rf_freq_in_hz;
    ( void ) pkt_params;
    ( void ) mod_params;
    // No real radio hardware constraints to validate against for this virtual radio.
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_tx( const void* context )
{
    ( void ) context;
    int     packet_len;
    ssize_t sent;

    // Check socket is connected
    if( !udp_ctx.connected || udp_ctx.sock_up < 0 )
    {
        printf( "not connected\n" );
        return RAL_STATUS_ERROR;
    }

    // Refresh the downlink route so the LNS can schedule RX windows after this TX
    send_pull_data( );

    // Clear work buffer before use
    memset( udp_ctx.work_buffer, 0, sizeof( udp_ctx.work_buffer ) );

    // Build PUSH_DATA packet
    packet_len = build_push_data_packet( udp_ctx.work_buffer, sizeof( udp_ctx.work_buffer ) );
    if( packet_len < 0 )
    {
        printf( "build_push_data_packet failed\n" );
        return RAL_STATUS_ERROR;
    }

    // Print the JSON rxpk frame being sent (skip 12-byte binary header)
    printf( "[UDP] Uplink JSON: %s\n", ( const char* ) ( udp_ctx.work_buffer + 12 ) );

    // Send UDP packet to server
    sent = send( udp_ctx.sock_up, udp_ctx.work_buffer, packet_len, 0 );
    if( sent < 0 )
    {
        printf( "send failed\n" );
    }

    // Calculate actual time-on-air based on packet type and modulation
    uint32_t toa_ms;
    if( udp_ctx.lr_fhss_mode )
    {
        // LR-FHSS mode (lr_fhss_mode replaces pkt_type since no RAL_PKT_TYPE_LR_FHSS exists)
        ral_udp_pf_lr_fhss_get_time_on_air_in_ms( context, &udp_ctx.lr_fhss_params, udp_ctx.tx_payload_size, &toa_ms );
    }
    else if( udp_ctx.pkt_type == RAL_PKT_TYPE_LORA )
    {
        toa_ms = ral_udp_pf_get_lora_time_on_air_in_ms( &udp_ctx.lora_pkt_params, &udp_ctx.lora_mod_params );
    }
    else if( udp_ctx.pkt_type == RAL_PKT_TYPE_GFSK )
    {
        toa_ms = ral_udp_pf_get_gfsk_time_on_air_in_ms( &udp_ctx.gfsk_pkt_params, &udp_ctx.gfsk_mod_params );
    }
    else
    {
        // Unknown packet type, no ToA
        toa_ms = 10;
        printf( "WARNING: Unknown packet type %d, using ToA of %u ms\n", udp_ctx.pkt_type, toa_ms );
    }

    printf( "  (TX_DONE triggered in %u ms)\n", toa_ms );

    // Start timer - will trigger TX_DONE IRQ after calculated ToA
    smtc_modem_hal_start_timer( toa_ms, ral_udp_pf_tx_done_timer_callback, NULL );

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_rx( const void* context, const uint32_t timeout_in_ms )
{
    ( void ) context;

    if( !udp_ctx.connected || udp_ctx.sock_down < 0 )
    {
        smtc_modem_hal_start_timer( timeout_in_ms, ral_udp_pf_rx_timeout_timer_callback, NULL );
        return RAL_STATUS_OK;
    }

    rx_ensure_thread_started( );

    if( !rx_thread_created )
    {
        smtc_modem_hal_start_timer( timeout_in_ms, ral_udp_pf_rx_timeout_timer_callback, NULL );
        return RAL_STATUS_OK;
    }

    rx_cancel_in_progress( );

    pthread_mutex_lock( &rx_mutex );
    rx_requested_timeout_ms = timeout_in_ms;
    rx_start_requested      = true;
    pthread_cond_signal( &rx_start_cond );
    pthread_mutex_unlock( &rx_mutex );

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_cfg_rx_boosted( const void* context, const bool enable_boost_mode )
{
    // Not applicable for UDP virtual radio
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_rx_tx_fallback_mode( const void* context, const ral_fallback_modes_t ral_fallback_mode )
{
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_stop_timer_on_preamble( const void* context, const bool enable )
{
    // Not applicable for UDP virtual radio
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_rx_duty_cycle( const void* context, const uint32_t rx_time_in_ms,
                                           const uint32_t sleep_time_in_ms )
{
    // Not applicable for UDP virtual radio
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_set_lora_cad( const void* context )
{
    // CAD not supported for UDP virtual radio
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_set_tx_cw( const void* context )
{
    // CW mode not supported for UDP virtual radio
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_set_tx_infinite_preamble( const void* context )
{
    // Infinite preamble not supported for UDP virtual radio
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_cal_img( const void* context, const uint16_t freq1_in_mhz, const uint16_t freq2_in_mhz )
{
    // No calibration needed for UDP virtual radio
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_tx_cfg( const void* context, const int8_t output_pwr_in_dbm, const uint32_t rf_freq_in_hz )
{
    ( void ) context;

    // Store TX configuration for UDP packet metadata
    udp_ctx.output_pwr_in_dbm = output_pwr_in_dbm;

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_pkt_payload( const void* context, const uint8_t* buffer, const uint16_t size )
{
    ( void ) context;

    // Store payload to be sent via UDP
    if( size > sizeof( udp_ctx.tx_payload ) )
    {
        printf( "ERROR: TX payload size (%u) exceeds buffer size (%zu)\n", size, sizeof( udp_ctx.tx_payload ) );
        return RAL_STATUS_ERROR;
    }

    memcpy( udp_ctx.tx_payload, buffer, size );
    udp_ctx.tx_payload_size = size;

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_get_pkt_payload( const void* context, uint16_t max_size_in_bytes, uint8_t* buffer,
                                         uint16_t* size_in_bytes )
{
    // Return the received payload stored in udp_ctx.rx_payload
    if( udp_ctx.rx_payload_size > max_size_in_bytes )
    {
        printf( "ERROR: RX payload size (%u) exceeds buffer size (%u)\n", udp_ctx.rx_payload_size, max_size_in_bytes );
        return RAL_STATUS_ERROR;
    }

    memcpy( buffer, udp_ctx.rx_payload, udp_ctx.rx_payload_size );
    *size_in_bytes = udp_ctx.rx_payload_size;

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_get_tx_fifo_level( const void* context, uint16_t* fifo_level )
{
    // Not applicable for UDP virtual radio
    *fifo_level = 0;
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_get_rx_fifo_level( const void* context, uint16_t* fifo_level )
{
    // Not applicable for UDP virtual radio
    *fifo_level = 0;
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_cfg_fifo_irq( const void* context, ral_radio_fifo_flag_t rx_fifo_irq_enable,
                                      ral_radio_fifo_flag_t tx_fifo_irq_enable, uint16_t rx_fifo_high_threshold,
                                      uint16_t tx_fifo_low_threshold, uint16_t rx_fifo_low_threshold,
                                      uint16_t tx_fifo_high_threshold )
{
    // Not applicable for UDP virtual radio
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_clear_fifo_irq( const void* context, ral_radio_fifo_flag_t rx_fifo_flags_to_clear,
                                        ral_radio_fifo_flag_t tx_fifo_flags_to_clear )
{
    // Not applicable for UDP virtual radio
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_get_irq_status( const void* context, ral_irq_t* irq )
{
    // Return pending IRQ flags
    *irq = pending_irq;

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_clear_irq_status( const void* context, const ral_irq_t irq )
{
    pending_irq &= ~irq;

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_get_and_clear_irq_status( const void* context, ral_irq_t* irq )
{
    ( void ) context;

    // Return pending IRQ flags
    *irq = pending_irq;

    // Clear pending IRQs
    pending_irq = RAL_IRQ_NONE;

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_dio_irq_params( const void* context, const ral_irq_t irq )
{
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_rf_freq( const void* context, const uint32_t freq_in_hz )
{
    udp_ctx.rf_freq_in_hz = freq_in_hz;
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_pkt_type( const void* context, const ral_pkt_type_t pkt_type )
{
    udp_ctx.pkt_type = pkt_type;
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_get_pkt_type( const void* context, ral_pkt_type_t* pkt_type )
{
    *pkt_type = udp_ctx.pkt_type;
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_gfsk_mod_params( const void* context, const ral_gfsk_mod_params_t* params )
{
    ( void ) context;

    // Store GFSK modulation parameters for ToA calculation
    udp_ctx.gfsk_mod_params = *params;

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_gfsk_pkt_params( const void* context, const ral_gfsk_pkt_params_t* params )
{
    ( void ) context;

    // Store GFSK packet parameters for ToA calculation
    udp_ctx.gfsk_pkt_params = *params;

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_gfsk_pkt_address( const void* context, const uint8_t node_address,
                                              const uint8_t broadcast_address )
{
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_lora_mod_params( const void* context, const ral_lora_mod_params_t* params )
{
    ( void ) context;

    // Store LoRa modulation parameters for UDP packet metadata and ToA calculation
    udp_ctx.lora_mod_params = *params;

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_lora_pkt_params( const void* context, const ral_lora_pkt_params_t* params )
{
    ( void ) context;

    // Store LoRa packet parameters for ToA calculation
    udp_ctx.lora_pkt_params = *params;

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_lora_cad_params( const void* context, const ral_lora_cad_params_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_set_lora_symb_nb_timeout( const void* context, const uint16_t nb_of_symbs )
{
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_flrc_mod_params( const void* context, const ral_flrc_mod_params_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_set_flrc_pkt_params( const void* context, const ral_flrc_pkt_params_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_get_gfsk_rx_pkt_status( const void* context, ral_gfsk_rx_pkt_status_t* rx_pkt_status )
{
    rx_pkt_status->rx_status        = 0;
    rx_pkt_status->rssi_sync_in_dbm = ( int16_t ) UDP_PF_DEFAULT_RSSI;
    rx_pkt_status->rssi_avg_in_dbm  = ( int16_t ) UDP_PF_DEFAULT_RSSI;
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_get_lora_rx_pkt_status( const void* context, ral_lora_rx_pkt_status_t* rx_pkt_status )
{
    rx_pkt_status->rssi_pkt_in_dbm        = ( int16_t ) UDP_PF_DEFAULT_RSSI;
    rx_pkt_status->snr_pkt_in_db          = ( int16_t ) UDP_PF_DEFAULT_LSNR;
    rx_pkt_status->signal_rssi_pkt_in_dbm = ( int16_t ) UDP_PF_DEFAULT_RSSI;
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_get_flrc_rx_pkt_status( const void* context, ral_flrc_rx_pkt_status_t* rx_pkt_status )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_get_rssi_inst( const void* context, int16_t* rssi_in_dbm )
{
    *rssi_in_dbm = ( int16_t ) UDP_PF_DEFAULT_RSSI;
    return RAL_STATUS_OK;
}

uint32_t ral_udp_pf_get_lora_time_on_air_in_ms( const ral_lora_pkt_params_t* pkt_p, const ral_lora_mod_params_t* mod_p )
{
    // Calculate LoRa time-on-air based on standard formula
    // Reference: Semtech SX1272/SX1276 datasheet, lora_pkt_fwd toa.c

    uint8_t  sf         = mod_p->sf;  // SF value is directly the enum value (5-12)
    uint8_t  cr         = mod_p->cr;  // CR: 1=4/5, 2=4/6, 3=4/7, 4=4/8
    uint16_t preamble   = pkt_p->preamble_len_in_symb;
    uint8_t  payload_sz = pkt_p->pld_len_in_bytes;
    bool     no_header  = ( pkt_p->header_type == RAL_LORA_PKT_IMPLICIT );
    bool     no_crc     = !pkt_p->crc_is_on;

    // Get bandwidth in Hz for symbol time calculation
    uint32_t bw_hz;
    switch( mod_p->bw )
    {
    case RAL_LORA_BW_125_KHZ:
        bw_hz = 125000;
        break;
    case RAL_LORA_BW_250_KHZ:
        bw_hz = 250000;
        break;
    case RAL_LORA_BW_500_KHZ:
        bw_hz = 500000;
        break;
    default:
        printf( "WARNING: Unsupported LoRa BW %d, assuming 125 kHz\n", mod_p->bw );
        bw_hz = 125000;
        break;
    }

    // Duration of 1 symbol in microseconds: T_symbol = (2^SF / BW)
    uint32_t t_symbol_us = ( 1 << sf ) * 1000000 / bw_hz;

    // Packet parameters
    uint8_t H         = no_header ? 0 : 1;     // Header enabled except for explicit mode
    uint8_t DE        = ( sf >= 11 ) ? 1 : 0;  // Low datarate optimization for SF11/SF12
    uint8_t n_bit_crc = no_crc ? 0 : 16;

    // Number of symbols in the payload
    // Formula: ceil(max(8*PL + CRC - 4*SF + 28 + 16*CRC - 20*H, 0) / (4*(SF-2*DE))) * (CR+4)
    int payload_bits = 8 * payload_sz + n_bit_crc - 4 * sf + ( ( sf >= 7 ) ? 8 : 0 ) + 20 * H;
    if( payload_bits < 0 )
    {
        payload_bits = 0;
    }

    // Use floating point for accurate division, then ceiling
    double   n_symbol_payload_d = ( double ) payload_bits / ( double ) ( 4 * ( sf - 2 * DE ) );
    uint32_t n_symbol_payload   = ( uint32_t ) ( n_symbol_payload_d + 0.9999 );  // Ceiling
    n_symbol_payload *= ( cr + 4 );

    // Total number of symbols in packet
    double n_symbol = ( double ) preamble + ( ( sf >= 7 ) ? 4.25 : 6.25 ) + 8.0 + ( double ) n_symbol_payload;

    // Duration of packet in microseconds
    uint32_t toa_us = ( uint32_t ) ( n_symbol * ( double ) t_symbol_us );

    // Convert to milliseconds (round up)
    uint32_t toa_ms = ( toa_us + 999 ) / 1000;

    return toa_ms;
}

uint32_t ral_udp_pf_get_gfsk_time_on_air_in_ms( const ral_gfsk_pkt_params_t* pkt_p, const ral_gfsk_mod_params_t* mod_p )
{
    // Calculate GFSK/FSK time-on-air
    // Reference: lora_pkt_fwd toa.c
    // Formula: ToA = (preamble + sync_word + packet_length_byte + payload + CRC) * 8 / bitrate

    uint32_t br_bps     = mod_p->br_in_bps;  // Bitrate in bits per second
    uint16_t preamble   = pkt_p->preamble_len_in_bits;
    uint8_t  sync_word  = pkt_p->sync_word_len_in_bits;
    uint16_t payload_sz = pkt_p->pld_len_in_bytes;

    // CRC size in bytes: 0, 1, or 2 bytes depending on crc_type
    uint8_t crc_bytes = 0;
    switch( pkt_p->crc_type )
    {
    case RAL_GFSK_CRC_OFF:
        crc_bytes = 0;
        break;
    case RAL_GFSK_CRC_1_BYTE:
    case RAL_GFSK_CRC_1_BYTE_INV:
        crc_bytes = 1;
        break;
    case RAL_GFSK_CRC_2_BYTES:
    case RAL_GFSK_CRC_2_BYTES_INV:
        crc_bytes = 2;
        break;
    case RAL_GFSK_CRC_3_BYTES:
    case RAL_GFSK_CRC_3_BYTES_INV:
        crc_bytes = 3;
        break;
    case RAL_GFSK_CRC_4_BYTES:
    case RAL_GFSK_CRC_4_BYTES_INV:
        crc_bytes = 4;
        break;
    default:
        crc_bytes = 0;
        break;
    }

    // Variable length mode has 1 byte for packet length
    uint8_t pkt_len_byte = ( pkt_p->header_type == RAL_GFSK_PKT_VAR_LEN ) ? 1 : 0;

    // Total bits = preamble + sync_word + (packet_length_byte + payload + CRC) * 8
    uint32_t total_bits = preamble + sync_word + ( pkt_len_byte + payload_sz + crc_bytes ) * 8;

    // Time-on-air in milliseconds
    // ToA_ms = (total_bits / bitrate) * 1000
    double   toa_s  = ( double ) total_bits / ( double ) br_bps;
    uint32_t toa_ms = ( uint32_t ) ( toa_s * 1000.0 + 1.0 );  // Add 1ms margin for rounding

    return toa_ms;
}

uint32_t ral_udp_pf_get_flrc_time_on_air_in_us( const ral_flrc_pkt_params_t* pkt_p, const ral_flrc_mod_params_t* mod_p )
{
    return 0;
}

ral_status_t ral_udp_pf_set_gfsk_sync_word( const void* context, const uint8_t* sync_word, const uint8_t sync_word_len )
{
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_lora_sync_word( const void* context, const uint8_t sync_word )
{
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_flrc_sync_word( const void* context, const uint8_t sync_word_id, const uint8_t* sync_word,
                                            const uint8_t sync_word_len )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_set_gfsk_crc_params( const void* context, const uint32_t seed, const uint32_t polynomial )
{
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_set_flrc_crc_params( const void* context, const uint32_t seed, const uint32_t polynomial )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_set_gfsk_whitening_seed( const void* context, const uint16_t seed )
{
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_lr_fhss_init( const void* context, const ral_lr_fhss_params_t* lr_fhss_params )
{
    ( void ) context;

    // Set LR-FHSS mode flag (replaces pkt_type since no RAL_PKT_TYPE_LR_FHSS exists)
    udp_ctx.lr_fhss_mode = true;

    // Store LR-FHSS parameters
    udp_ctx.lr_fhss_params = *lr_fhss_params;

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_lr_fhss_build_frame( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                             ral_lr_fhss_memory_state_t memory_state_holder, uint16_t hop_sequence_id,
                                             const uint8_t* payload, uint16_t payload_length )
{
    ( void ) context;
    ( void ) memory_state_holder;

    // Store LR-FHSS parameters
    udp_ctx.lr_fhss_params          = *lr_fhss_params;
    udp_ctx.lr_fhss_hop_sequence_id = hop_sequence_id;

    // Store payload (LR-FHSS provides payload directly here, not via set_pkt_payload)
    if( payload_length > sizeof( udp_ctx.tx_payload ) )
    {
        printf( "ERROR: LR-FHSS TX payload size (%u) exceeds buffer size (%zu)\n", payload_length,
                sizeof( udp_ctx.tx_payload ) );
        return RAL_STATUS_ERROR;
    }

    memcpy( udp_ctx.tx_payload, payload, payload_length );
    udp_ctx.tx_payload_size = payload_length;

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_lr_fhss_handle_hop( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                            ral_lr_fhss_memory_state_t state )
{
    ( void ) context;         // Unused arguments
    ( void ) lr_fhss_params;  // Unused arguments
    ( void ) state;           // Unused arguments
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_lr_fhss_handle_tx_done( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                                ral_lr_fhss_memory_state_t state )
{
    ( void ) context;
    ( void ) lr_fhss_params;
    ( void ) state;

    // Reset LR-FHSS mode flag after TX completion
    udp_ctx.lr_fhss_mode = false;

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_lr_fhss_get_time_on_air_in_ms( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                                       uint16_t payload_length, uint32_t* time_on_air )
{
    ( void ) context;

    // LR-FHSS constants (same for LR11xx and LR20xx)
    const uint16_t LR_FHSS_HEADER_BITS = 114;
    const uint16_t LR_FHSS_FRAG_BITS   = 48;
    const uint16_t LR_FHSS_BLOCK_BITS  = 50;  // FRAG_BITS + 2 preamble bits

    // Calculate length in bits: (payload + 2 CRC bytes) * 8 + 6 tail bits
    uint16_t length_bits = ( payload_length + 2 ) * 8 + 6;

    // Apply coding rate multiplier
    switch( lr_fhss_params->lr_fhss_params.cr )
    {
    case LR_FHSS_V1_CR_5_6:
        length_bits = ( ( length_bits * 6 ) + 4 ) / 5;
        break;
    case LR_FHSS_V1_CR_2_3:
        length_bits = length_bits * 3 / 2;
        break;
    case LR_FHSS_V1_CR_1_2:
        length_bits = length_bits * 2;
        break;
    case LR_FHSS_V1_CR_1_3:
        length_bits = length_bits * 3;
        break;
    default:
        break;
    }

    // Calculate payload bits with block structure
    uint16_t payload_bits    = ( length_bits / LR_FHSS_FRAG_BITS ) * LR_FHSS_BLOCK_BITS;
    uint16_t last_block_bits = length_bits % LR_FHSS_FRAG_BITS;
    if( last_block_bits > 0 )
    {
        payload_bits += last_block_bits + 2;
    }

    // Total bits = header bits + payload bits
    uint16_t total_bits = LR_FHSS_HEADER_BITS * lr_fhss_params->lr_fhss_params.header_count + payload_bits;

    // Convert bits to milliseconds: bits * 1000 / 488.28125 = (bits * 256 + 124) / 125
    *time_on_air = ( ( ( uint32_t ) total_bits << 8 ) + 124 ) / 125;

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_lr_fhss_get_hop_sequence_count( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                                        unsigned int* hop_sequence_count )
{
    if( ( lr_fhss_params->lr_fhss_params.grid == LR_FHSS_V1_GRID_25391_HZ ) ||
        ( ( lr_fhss_params->lr_fhss_params.grid == LR_FHSS_V1_GRID_3906_HZ ) &&
          ( lr_fhss_params->lr_fhss_params.bw < LR_FHSS_V1_BW_335938_HZ ) ) )
    {
        *hop_sequence_count = 384;
    }
    else
    {
        *hop_sequence_count = 512;
    }

    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_lr_fhss_get_bit_delay_in_us( const void* context, const ral_lr_fhss_params_t* params,
                                                     uint16_t payload_length, uint16_t* delay )
{
    *delay = 0;
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_get_lora_rx_pkt_cr_crc( const void* context, ral_lora_cr_t* cr, bool* is_crc_present )
{
    *cr             = RAL_LORA_CR_4_5;
    *is_crc_present = true;
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_get_tx_consumption_in_ua( const void* context, const int8_t output_pwr_in_dbm,
                                                  const uint32_t rf_freq_in_hz, uint32_t* pwr_consumption_in_ua )
{
    *pwr_consumption_in_ua = 0;
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_get_gfsk_rx_consumption_in_ua( const void* context, const uint32_t br_in_bps,
                                                       const uint32_t bw_dsb_in_hz, const bool rx_boosted,
                                                       uint32_t* pwr_consumption_in_ua )
{
    *pwr_consumption_in_ua = 0;
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_get_lora_rx_consumption_in_ua( const void* context, const ral_lora_bw_t bw,
                                                       const bool rx_boosted, uint32_t* pwr_consumption_in_ua )
{
    *pwr_consumption_in_ua = 0;
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_get_random_numbers( const void* context, uint32_t* numbers, unsigned int n )
{
    ( void ) context;

    for( unsigned int i = 0; i < n; i++ )
    {
        numbers[i] = ( uint32_t ) rand( );
    }
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_handle_rx_done( const void* context )
{
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_handle_tx_done( const void* context )
{
    return RAL_STATUS_OK;
}

ral_status_t ral_udp_pf_get_lora_cad_det_peak( const void* context, ral_lora_sf_t sf, ral_lora_bw_t bw,
                                               ral_lora_cad_symbs_t nb_symbol, uint8_t* cad_det_peak )
{
    // CAD not supported for UDP virtual radio
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_rttof_set_parameters( const void* context, const uint8_t nb_symbols )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_rttof_set_request_address( const void* context, const uint32_t request_address )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_rttof_set_rx_tx_delay_indicator( const void* context, const uint32_t delay_indicator )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_rttof_get_raw_result( const void* context, ral_lora_bw_t rttof_bw, int32_t* raw_results,
                                              int32_t* results_meter, int8_t* rssi_result )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_rttof_set_address( const void* context, const uint32_t address, const uint8_t check_length )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_udp_pf_set_fake_irq_callback( const void* context, void ( *callback )( void* ),
                                               void*       callback_context )
{
    ( void ) context;  // Unused for UDP_PF

    stored_irq_callback = callback;
    stored_irq_context  = callback_context;

    return RAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static bool parse_hex_string( const char* hex_str, uint8_t* bytes, size_t len )
{
    if( hex_str == NULL || bytes == NULL )
    {
        return false;
    }

    size_t hex_len = strlen( hex_str );
    if( hex_len != len * 2 )
    {
        return false;  // String length must be exactly 2x the number of bytes
    }

    for( size_t i = 0; i < len; i++ )
    {
        char  byte_str[3] = { hex_str[i * 2], hex_str[i * 2 + 1], '\0' };
        char* endptr;
        long  val = strtol( byte_str, &endptr, 16 );
        if( *endptr != '\0' || val < 0 || val > 255 )
        {
            return false;  // Invalid hex character
        }
        bytes[i] = ( uint8_t ) val;
    }
    return true;
}

static void load_config_from_env( void )
{
    const char* env_val;

    // Initialize with defaults
    strncpy( udp_pf_server_addr, UDP_PF_DEFAULT_SERVER_ADDR, sizeof( udp_pf_server_addr ) - 1 );
    udp_pf_server_addr[sizeof( udp_pf_server_addr ) - 1] = '\0';

    strncpy( udp_pf_server_port, UDP_PF_DEFAULT_SERVER_PORT, sizeof( udp_pf_server_port ) - 1 );
    udp_pf_server_port[sizeof( udp_pf_server_port ) - 1] = '\0';

    parse_hex_string( UDP_PF_DEFAULT_GATEWAY_EUI, udp_pf_gateway_eui, 8 );

    // Override with environment variables if set
    env_val = getenv( UDP_PF_SERVER_ADDR_ENV_VAR_NAME );
    if( env_val != NULL && strlen( env_val ) > 0 )
    {
        if( strlen( env_val ) >= sizeof( udp_pf_server_addr ) )
        {
            printf( "UDP_PF: Warning - Server address too long (max %zu), using default\n",
                    sizeof( udp_pf_server_addr ) - 1 );
        }
        else
        {
            strncpy( udp_pf_server_addr, env_val, sizeof( udp_pf_server_addr ) - 1 );
            udp_pf_server_addr[sizeof( udp_pf_server_addr ) - 1] = '\0';
        }
    }

    env_val = getenv( UDP_PF_SERVER_PORT_ENV_VAR_NAME );
    if( env_val != NULL && strlen( env_val ) > 0 )
    {
        char* endptr;
        long  port_val = strtol( env_val, &endptr, 10 );
        if( *endptr != '\0' || port_val < 1 || port_val > 65535 )
        {
            printf( "UDP_PF: Warning - Invalid server port (must be 1-65535), using default\n" );
        }
        else
        {
            strncpy( udp_pf_server_port, env_val, sizeof( udp_pf_server_port ) - 1 );
            udp_pf_server_port[sizeof( udp_pf_server_port ) - 1] = '\0';
        }
    }

    env_val = getenv( UDP_PF_GATEWAY_EUI_ENV_VAR_NAME );
    if( env_val != NULL && strlen( env_val ) > 0 )
    {
        if( strlen( env_val ) != 16 )
        {
            printf( "UDP_PF: Warning - Gateway EUI must be 16 hex characters, got %zu, using default\n",
                    strlen( env_val ) );
        }
        else if( !parse_hex_string( env_val, udp_pf_gateway_eui, 8 ) )
        {
            printf( "UDP_PF: Warning - Invalid gateway EUI format (not valid hex), using default\n" );
        }
    }

    // Print configuration
    printf( "UDP_PF: Configuration:\n" );
    printf( "  Server address: %s\n", udp_pf_server_addr );
    printf( "  Server port:    %s\n", udp_pf_server_port );
    printf( "  Gateway EUI:    %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n", udp_pf_gateway_eui[0], udp_pf_gateway_eui[1],
            udp_pf_gateway_eui[2], udp_pf_gateway_eui[3], udp_pf_gateway_eui[4], udp_pf_gateway_eui[5],
            udp_pf_gateway_eui[6], udp_pf_gateway_eui[7] );
    printf( "  To override, set environment variables, e.g.:\n" );
    printf( "    export %s=lns.example.com\n", UDP_PF_SERVER_ADDR_ENV_VAR_NAME );
    printf( "    export %s=1700\n", UDP_PF_SERVER_PORT_ENV_VAR_NAME );
    printf( "    export %s=AA0000FFFE000001\n", UDP_PF_GATEWAY_EUI_ENV_VAR_NAME );
}

static void rx_cancel_in_progress( void )
{
    pthread_mutex_lock( &rx_mutex );
    if( rx_busy )
    {
        rx_cancel = true;
        char c    = 'c';
        if( write( rx_cancel_pipe[1], &c, 1 ) < 0 )
        {
            /* Non-blocking pipe, EAGAIN is expected if pipe is full (harmless) */
        }

        while( rx_busy )
        {
            pthread_cond_wait( &rx_done_cond, &rx_mutex );
        }
    }
    pthread_mutex_unlock( &rx_mutex );
}

static void rx_ensure_thread_started( void )
{
    if( rx_thread_created )
    {
        return;
    }

    if( pipe( rx_cancel_pipe ) < 0 )
    {
        printf( "ERROR: Failed to create RX cancel pipe\n" );
        return;
    }

    // Make pipe non-blocking so stale cancel bytes never cause hangs
    fcntl( rx_cancel_pipe[0], F_SETFL, fcntl( rx_cancel_pipe[0], F_GETFL ) | O_NONBLOCK );
    fcntl( rx_cancel_pipe[1], F_SETFL, fcntl( rx_cancel_pipe[1], F_GETFL ) | O_NONBLOCK );

    if( pthread_create( &rx_thread, NULL, rx_worker_thread_func, NULL ) != 0 )
    {
        printf( "ERROR: Failed to create RX worker thread\n" );
        close( rx_cancel_pipe[0] );
        close( rx_cancel_pipe[1] );
        rx_cancel_pipe[0] = -1;
        rx_cancel_pipe[1] = -1;
        return;
    }

    pthread_detach( rx_thread );
    rx_thread_created = true;
}

static bool rx_do_recv( uint32_t timeout_ms )
{
    // Drain any stale cancel bytes from previous cancellations
    {
        char drain;
        while( read( rx_cancel_pipe[0], &drain, 1 ) > 0 )
        {
        }
    }

    struct pollfd pfds[2];
    pfds[0].fd     = udp_ctx.sock_down;
    pfds[0].events = POLLIN;
    pfds[1].fd     = rx_cancel_pipe[0];
    pfds[1].events = POLLIN;

    send_pull_data( );

    printf( "[UDP] RX window started, timeout %u ms\n", timeout_ms );

    uint32_t start_ms          = smtc_modem_hal_get_time_in_ms( );
    bool     downlink_received = false;

    while( !downlink_received )
    {
        uint32_t elapsed_ms = smtc_modem_hal_get_time_in_ms( ) - start_ms;
        if( elapsed_ms >= timeout_ms )
        {
            printf( "[UDP] nothing received on downlink socket\n" );
            break;
        }

        int remaining_ms = ( int ) ( timeout_ms - elapsed_ms );
        int ret          = poll( pfds, 2, remaining_ms );

        if( ret < 0 )
        {
            if( errno == EINTR )
            {
                continue;
            }
            printf( "[UDP] poll error: %s\n", strerror( errno ) );
            break;
        }
        if( ret == 0 )
        {
            printf( "[UDP] nothing received on downlink socket\n" );
            break;
        }

        if( pfds[1].revents & POLLIN )
        {
            printf( "[UDP] RX cancelled\n" );
            break;
        }

        if( pfds[0].revents & POLLIN )
        {
            memset( udp_ctx.work_buffer, 0, sizeof( udp_ctx.work_buffer ) );
            ssize_t received = recv( udp_ctx.sock_down, udp_ctx.work_buffer, sizeof( udp_ctx.work_buffer ), 0 );

            if( received > 0 )
            {
                printf( "[UDP] Received %zd bytes on sock_down, type=%u\n", received,
                        ( received >= 4 ) ? udp_ctx.work_buffer[3] : 0xFF );

                if( received >= 4 && udp_ctx.work_buffer[0] == PROTOCOL_VERSION &&
                    udp_ctx.work_buffer[3] == PKT_PULL_RESP )
                {
                    int parse_result =
                        parse_pull_resp( udp_ctx.work_buffer, received, udp_ctx.rx_payload, &udp_ctx.rx_payload_size );

                    if( parse_result >= 0 )
                    {
                        send_tx_ack( udp_ctx.work_buffer[1], udp_ctx.work_buffer[2] );
                        downlink_received = true;
                    }
                    else
                    {
                        printf( "WARNING: Failed to parse PULL_RESP\n" );
                    }
                }
                else if( received == 4 && udp_ctx.work_buffer[3] == PKT_PULL_ACK )
                {
                    printf( "[UDP] PULL_ACK received, waiting for PULL_RESP...\n" );
                }
            }
            else
            {
                printf( "[UDP] nothing received on downlink socket\n" );
                break;
            }
        }
    }

    return downlink_received;
}

static void* rx_worker_thread_func( void* arg )
{
    ( void ) arg;

    pthread_mutex_lock( &rx_mutex );
    for( ;; )
    {
        while( !rx_start_requested )
        {
            pthread_cond_wait( &rx_start_cond, &rx_mutex );
        }

        rx_start_requested = false;
        rx_busy            = true;
        rx_cancel          = false;
        uint32_t timeout   = rx_requested_timeout_ms;
        pthread_mutex_unlock( &rx_mutex );

        bool downlink = rx_do_recv( timeout );

        pthread_mutex_lock( &rx_mutex );
        rx_busy            = false;
        bool was_cancelled = rx_cancel;
        pthread_cond_signal( &rx_done_cond );
        pthread_mutex_unlock( &rx_mutex );

        /* Fire IRQ callback outside the mutex so the callback can safely call
         * set_rx(), set_standby(), etc. without deadlocking.
         */
        if( !was_cancelled )
        {
            pending_irq |= downlink ? RAL_IRQ_RX_DONE : RAL_IRQ_RX_TIMEOUT;
            if( stored_irq_callback != NULL )
            {
                stored_irq_callback( stored_irq_context );
            }
        }

        pthread_mutex_lock( &rx_mutex );
    }

    return NULL;
}

static void ral_udp_pf_tx_done_timer_callback( void* context )
{
    ( void ) context;  // Unused

    // TX completed - set IRQ flag
    pending_irq |= RAL_IRQ_TX_DONE;

    // Trigger the radio planner IRQ callback
    if( stored_irq_callback != NULL )
    {
        stored_irq_callback( stored_irq_context );
    }
}

static void ral_udp_pf_rx_timeout_timer_callback( void* context )
{
    ( void ) context;  // Unused

    // RX timeout - set IRQ flag
    pending_irq |= RAL_IRQ_RX_TIMEOUT;

    // Trigger the radio planner IRQ callback
    if( stored_irq_callback != NULL )
    {
        stored_irq_callback( stored_irq_context );
    }
}

static int send_pull_data( void )
{
    uint8_t packet[12];
    ssize_t sent;

    // Build PULL_DATA packet (12 bytes total, no JSON payload)
    packet[0] = PROTOCOL_VERSION;                  // Protocol version = 2
    packet[1] = ( uint8_t ) rand( );               // Random token
    packet[2] = ( uint8_t ) rand( );               // Random token
    packet[3] = PKT_PULL_DATA;                     // PULL_DATA identifier = 0x02
    memcpy( packet + 4, udp_ctx.gateway_eui, 8 );  // Gateway EUI

    // Send PULL_DATA packet
    sent = send( udp_ctx.sock_down, packet, 12, 0 );

    if( sent < 0 )
    {
        return -1;
    }

    return 0;
}

static void send_tx_ack( uint8_t token_h, uint8_t token_l )
{
    uint8_t packet[12];

    packet[0] = PROTOCOL_VERSION;
    packet[1] = token_h;
    packet[2] = token_l;
    packet[3] = PKT_TX_ACK;
    memcpy( packet + 4, udp_ctx.gateway_eui, 8 );

    ssize_t sent = send( udp_ctx.sock_down, packet, 12, 0 );
    if( sent < 0 )
    {
        printf( "[UDP] WARNING: Failed to send TX_ACK\n" );
    }
    else
    {
        printf( "[UDP] TX_ACK sent\n" );
    }
}

static int parse_pull_resp( const uint8_t* packet, size_t packet_len, uint8_t* payload_out, uint16_t* size_out )
{
    // Minimum PULL_RESP: 4 bytes header + minimal JSON {"txpk":{}}
    if( packet_len < 20 )
    {
        printf( "DEBUG: packet too short (%zu bytes)\n", packet_len );
        return -1;
    }

    // Verify packet type
    if( packet[0] != PROTOCOL_VERSION || packet[3] != PKT_PULL_RESP )
    {
        printf( "DEBUG: wrong packet type (ver=%u, type=%u)\n", packet[0], packet[3] );
        return -1;
    }

    // JSON payload starts at byte 4
    const char* json_str = ( const char* ) ( packet + 4 );
    int         json_len = packet_len - 4;
    printf( "[UDP] JSON received: %.*s\n", json_len, json_str );

    // find "data" field and decode base64
    const char* data_field = strstr( json_str, "\"data\":" );
    if( data_field == NULL )
    {
        printf( "DEBUG: data field not found in JSON\n" );
        return -1;
    }

    // Move past "data": and find the opening quote
    const char* data_start = strchr( data_field + 7, '"' );
    if( data_start == NULL )
    {
        printf( "DEBUG: data field opening quote not found\n" );
        return -1;
    }
    data_start++;  // Move past the opening quote

    // Find closing quote
    const char* data_end = strchr( data_start, '"' );
    if( data_end == NULL )
    {
        printf( "DEBUG: data field closing quote not found\n" );
        return -1;
    }

    // Calculate base64 string length
    size_t b64_len = data_end - data_start;

    // Parse "size" field from JSON and cross-check with decoded length
    int         json_size  = -1;
    const char* size_field = strstr( json_str, "\"size\":" );
    if( size_field != NULL )
    {
        json_size = ( int ) strtol( size_field + 7, NULL, 10 );
    }

    // Decode base64
    int decoded_len = b64_to_bin( data_start, b64_len, payload_out, sizeof( udp_ctx.rx_payload ) );

    if( decoded_len < 0 )
    {
        printf( "DEBUG: base64 decode failed\n" );
        return -1;
    }

    if( json_size >= 0 && decoded_len != json_size )
    {
        printf( "DEBUG: size mismatch: JSON \"size\"=%d but decoded %d bytes\n", json_size, decoded_len );
        return -1;
    }

    *size_out = ( uint16_t ) decoded_len;

    return 0;
}

static int build_datr_string( char* buff, size_t max_len, ral_lora_sf_t sf, ral_lora_bw_t bw )
{
    const char* sf_str = "SF?";
    const char* bw_str = "BW?";

    // Convert spreading factor
    switch( sf )
    {
    case RAL_LORA_SF5:
        sf_str = "SF5";
        break;
    case RAL_LORA_SF6:
        sf_str = "SF6";
        break;
    case RAL_LORA_SF7:
        sf_str = "SF7";
        break;
    case RAL_LORA_SF8:
        sf_str = "SF8";
        break;
    case RAL_LORA_SF9:
        sf_str = "SF9";
        break;
    case RAL_LORA_SF10:
        sf_str = "SF10";
        break;
    case RAL_LORA_SF11:
        sf_str = "SF11";
        break;
    case RAL_LORA_SF12:
        sf_str = "SF12";
        break;
    default:
        break;
    }

    // Convert bandwidth
    switch( bw )
    {
    case RAL_LORA_BW_125_KHZ:
        bw_str = "BW125";
        break;
    case RAL_LORA_BW_250_KHZ:
        bw_str = "BW250";
        break;
    case RAL_LORA_BW_500_KHZ:
        bw_str = "BW500";
        break;
    default:
        break;
    }

    return snprintf( buff, max_len, ",\"datr\":\"%s%s\"", sf_str, bw_str );
}

static int build_codr_string( char* buff, size_t max_len, ral_lora_cr_t cr )
{
    const char* cr_str = "?";

    switch( cr )
    {
    case RAL_LORA_CR_4_5:
        cr_str = "4/5";
        break;
    case RAL_LORA_CR_4_6:
        cr_str = "4/6";
        break;
    case RAL_LORA_CR_4_7:
        cr_str = "4/7";
        break;
    case RAL_LORA_CR_4_8:
        cr_str = "4/8";
        break;
    default:
        break;
    }

    return snprintf( buff, max_len, ",\"codr\":\"%s\"", cr_str );
}

static int build_lr_fhss_codr_string( char* buff, size_t max_len, lr_fhss_v1_cr_t cr )
{
    const char* cr_str = "?";

    switch( cr )
    {
    case LR_FHSS_V1_CR_5_6:
        cr_str = "5/6";
        break;
    case LR_FHSS_V1_CR_2_3:
        cr_str = "4/6";
        break;
    case LR_FHSS_V1_CR_1_2:
        cr_str = "1/2";
        break;
    case LR_FHSS_V1_CR_1_3:
        cr_str = "1/3";
        break;
    default:
        break;
    }

    return snprintf( buff, max_len, ",\"codr\":\"%s\"", cr_str );
}

static int build_lr_fhss_datr_string( char* buff, size_t max_len, lr_fhss_v1_bw_t bw )
{
    // LR-FHSS datarate format: "M<modulation>BW<bandwidth_in_khz>"
    // Modulation is always 0 (GMSK 488)
    const char* bw_str = "???";

    switch( bw )
    {
    case LR_FHSS_V1_BW_39063_HZ:
        bw_str = "39";
        break;
    case LR_FHSS_V1_BW_85938_HZ:
        bw_str = "86";
        break;
    case LR_FHSS_V1_BW_136719_HZ:
        bw_str = "137";
        break;
    case LR_FHSS_V1_BW_183594_HZ:
        bw_str = "184";
        break;
    case LR_FHSS_V1_BW_335938_HZ:
        bw_str = "336";
        break;
    case LR_FHSS_V1_BW_386719_HZ:
        bw_str = "387";
        break;
    case LR_FHSS_V1_BW_722656_HZ:
        bw_str = "723";
        break;
    case LR_FHSS_V1_BW_773438_HZ:
        bw_str = "773";
        break;
    case LR_FHSS_V1_BW_1523438_HZ:
        bw_str = "1523";
        break;
    case LR_FHSS_V1_BW_1574219_HZ:
        bw_str = "1574";
        break;
    default:
        break;
    }

    return snprintf( buff, max_len, ",\"datr\":\"M0CW%s\"", bw_str );
}

static int build_push_data_packet( uint8_t* buff, size_t max_len )
{
    int buff_index = 0;
    int j;

    // Binary header (12 bytes)
    buff[0] = PROTOCOL_VERSION;                  // Protocol version = 2
    buff[1] = ( uint8_t ) rand( );               // Random token
    buff[2] = ( uint8_t ) rand( );               // Random token
    buff[3] = PKT_PUSH_DATA;                     // PUSH_DATA identifier = 0x00
    memcpy( buff + 4, udp_ctx.gateway_eui, 8 );  // Gateway EUI
    buff_index = 12;

    // Start JSON object
    buff[buff_index++] = '{';

    // Start rxpk array
    j = snprintf( ( char* ) ( buff + buff_index ), max_len - buff_index, "\"rxpk\":[{" );
    if( j <= 0 )
        return -1;
    buff_index += j;

    // Timestamp (convert ms to µs)
    j = snprintf( ( char* ) ( buff + buff_index ), max_len - buff_index, "\"tmst\":%u",
                  smtc_modem_hal_get_time_in_ms( ) * 1000 );
    if( j <= 0 )
        return -1;
    buff_index += j;

    // Frequency (convert Hz to MHz)
    j = snprintf( ( char* ) ( buff + buff_index ), max_len - buff_index, ",\"freq\":%.6f",
                  ( double ) udp_ctx.rf_freq_in_hz / 1e6 );
    if( j <= 0 )
        return -1;
    buff_index += j;

    // Channel and RF chain
    j = snprintf( ( char* ) ( buff + buff_index ), max_len - buff_index, ",\"chan\":%d,\"rfch\":%d",
                  UDP_PF_DEFAULT_CHAN, UDP_PF_DEFAULT_RFCH );
    if( j <= 0 )
        return -1;
    buff_index += j;

    // Status: CRC OK
    j = snprintf( ( char* ) ( buff + buff_index ), max_len - buff_index, ",\"stat\":1" );
    if( j <= 0 )
        return -1;
    buff_index += j;

    // Modulation-specific fields
    if( udp_ctx.lr_fhss_mode )
    {
        // LR-FHSS modulation
        j = snprintf( ( char* ) ( buff + buff_index ), max_len - buff_index, ",\"modu\":\"LR-FHSS\"" );
        if( j <= 0 )
            return -1;
        buff_index += j;

        // LR-FHSS datarate (e.g. "M0BW137" for 137 kHz bandwidth)
        j = build_lr_fhss_datr_string( ( char* ) ( buff + buff_index ), max_len - buff_index,
                                       udp_ctx.lr_fhss_params.lr_fhss_params.bw );
        if( j <= 0 )
            return -1;
        buff_index += j;

        // LR-FHSS coding rate
        j = build_lr_fhss_codr_string( ( char* ) ( buff + buff_index ), max_len - buff_index,
                                       udp_ctx.lr_fhss_params.lr_fhss_params.cr );
        if( j <= 0 )
            return -1;
        buff_index += j;

        // RSSI only (no lsnr for LR-FHSS)
        j = snprintf( ( char* ) ( buff + buff_index ), max_len - buff_index, ",\"rssi\":%d", UDP_PF_DEFAULT_RSSI );
        if( j <= 0 )
            return -1;
        buff_index += j;
    }
    else if( udp_ctx.pkt_type == RAL_PKT_TYPE_GFSK )
    {
        // FSK modulation
        j = snprintf( ( char* ) ( buff + buff_index ), max_len - buff_index, ",\"modu\":\"FSK\"" );
        if( j <= 0 )
            return -1;
        buff_index += j;

        // FSK datarate is numeric (bitrate in bps)
        j = snprintf( ( char* ) ( buff + buff_index ), max_len - buff_index, ",\"datr\":%u",
                      udp_ctx.gfsk_mod_params.br_in_bps );
        if( j <= 0 )
            return -1;
        buff_index += j;

        // FSK has no coding rate field

        // RSSI only (no lsnr for FSK)
        j = snprintf( ( char* ) ( buff + buff_index ), max_len - buff_index, ",\"rssi\":%d", UDP_PF_DEFAULT_RSSI );
        if( j <= 0 )
            return -1;
        buff_index += j;
    }
    else
    {
        // LoRa modulation (default)
        j = snprintf( ( char* ) ( buff + buff_index ), max_len - buff_index, ",\"modu\":\"LORA\"" );
        if( j <= 0 )
            return -1;
        buff_index += j;

        // Datarate (SF + BW)
        j = build_datr_string( ( char* ) ( buff + buff_index ), max_len - buff_index, udp_ctx.lora_mod_params.sf,
                               udp_ctx.lora_mod_params.bw );
        if( j <= 0 )
            return -1;
        buff_index += j;

        // Coding rate
        j = build_codr_string( ( char* ) ( buff + buff_index ), max_len - buff_index, udp_ctx.lora_mod_params.cr );
        if( j <= 0 )
            return -1;
        buff_index += j;

        // RSSI
        j = snprintf( ( char* ) ( buff + buff_index ), max_len - buff_index, ",\"rssi\":%d", UDP_PF_DEFAULT_RSSI );
        if( j <= 0 )
            return -1;
        buff_index += j;

        // LoRa SNR (only for LoRa)
        j = snprintf( ( char* ) ( buff + buff_index ), max_len - buff_index, ",\"lsnr\":%.1f", UDP_PF_DEFAULT_LSNR );
        if( j <= 0 )
            return -1;
        buff_index += j;
    }

    // Size
    j = snprintf( ( char* ) ( buff + buff_index ), max_len - buff_index, ",\"size\":%u", udp_ctx.tx_payload_size );
    if( j <= 0 )
        return -1;
    buff_index += j;

    // Base64 encoded payload
    j = snprintf( ( char* ) ( buff + buff_index ), max_len - buff_index, ",\"data\":\"" );
    if( j <= 0 )
        return -1;
    buff_index += j;

    j = bin_to_b64( udp_ctx.tx_payload, udp_ctx.tx_payload_size, ( char* ) ( buff + buff_index ),
                    max_len - buff_index );
    if( j < 0 )
        return -1;
    buff_index += j;

    buff[buff_index++] = '"';

    // End rxpk object
    buff[buff_index++] = '}';

    // End rxpk array
    buff[buff_index++] = ']';

    // End JSON object
    buff[buff_index++] = '}';

    // Null terminator for safety
    buff[buff_index] = 0;

    return buff_index;
}

/* --- EOF ------------------------------------------------------------------ */
