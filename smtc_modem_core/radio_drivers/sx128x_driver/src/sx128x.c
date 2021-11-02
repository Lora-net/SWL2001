/**
 * @file      sx128x.c
 *
 * @brief     SX128X radio driver implementation
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

#include <string.h>  // memcpy
#include <limits.h>
#include "sx128x_hal.h"
#include "sx128x_regs.h"
#include "sx128x.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Bit shift to read chip mode
 */
#define SX128X_CHIP_MODES_POS 5

/**
 * @brief Bit mask to read chip mode
 */
#define SX128X_CHIP_MODES_MASK 0x07

/**
 * @brief Bit shift to read command status
 */
#define SX128X_CMD_STATUS_POS 2

/**
 * @brief Bit mask to read command status
 */
#define SX128X_CMD_STATUS_MASK 0x07

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * @brief Internal frequency of the radio
 */
#define SX128X_XTAL_FREQ 52000000UL

/**
 * @brief Scaling factor used to perform fixed-point operations
 */
#define SX128X_PLL_STEP_SHIFT_AMOUNT ( 10 )

/**
 * @brief PLL step - scaled with SX128X_PLL_STEP_SHIFT_AMOUNT
 */
#define SX128X_PLL_STEP_SCALED ( SX128X_XTAL_FREQ >> ( 18 - SX128X_PLL_STEP_SHIFT_AMOUNT ) )

/**
 * @brief Length in byte of the packet status sent by the chip
 */
#define SX128X_PKT_STATUS_LEN 5

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * Commands Interface
 */
typedef enum sx128x_commands_e
{
    // Operational Modes Functions
    SX128X_SET_SLEEP                = 0x84,
    SX128X_SET_STANDBY              = 0x80,
    SX128X_SET_FS                   = 0xC1,
    SX128X_SET_TX                   = 0x83,
    SX128X_SET_RX                   = 0x82,
    SX128X_SET_RX_DUTY_CYCLE        = 0x94,
    SX128X_SET_CAD                  = 0xC5,
    SX128X_SET_AUTOTX               = 0x98,
    SX128X_SET_AUTO_FS              = 0x9E,
    SX128X_SET_TX_CONTINUOUS_WAVE   = 0xD1,
    SX128X_SET_TX_INFINITE_PREAMBLE = 0xD2,
    // Registers and buffer Access
    SX128X_WRITE_REGISTER = 0x18,
    SX128X_READ_REGISTER  = 0x19,
    SX128X_WRITE_BUFFER   = 0x1A,
    SX128X_READ_BUFFER    = 0x1B,
    // DIO and IRQ Control Functions
    SX128X_SET_DIO_IRQ_PARAMS = 0x8D,
    SX128X_GET_IRQ_STATUS     = 0x15,
    SX128X_CLR_IRQ_STATUS     = 0x97,
    // RF Modulation and Packet-Related Functions
    SX128X_SET_RF_FREQUENCY        = 0x86,
    SX128X_SET_PKT_TYPE            = 0x8A,
    SX128X_GET_PKT_TYPE            = 0x03,
    SX128X_SET_TX_PARAMS           = 0x8E,
    SX128X_SET_MODULATION_PARAMS   = 0x8B,
    SX128X_SET_PKT_PARAMS          = 0x8C,
    SX128X_SET_CAD_PARAMS          = 0x88,
    SX128X_SET_BUFFER_BASE_ADDRESS = 0x8F,
    // Communication Status Information
    SX128X_GET_STATUS           = 0xC0,
    SX128X_GET_RX_BUFFER_STATUS = 0x17,
    SX128X_GET_PKT_STATUS       = 0x1D,
    SX128X_GET_RSSI_INST        = 0x1F,
    // Miscellaneous
    SX128X_SET_LONG_PREAMBLE  = 0x9B,
    SX128X_SET_REGULATOR_MODE = 0x96,
    SX128X_SET_SAVE_CONTEXT   = 0xD5,
    SX128X_SET_RANGING_ROLE   = 0xA3,
    SX128X_SET_ADV_RANGING    = 0x9A,
} sx128x_commands_t;

/**
 * Commands Interface buffer sizes
 */
typedef enum sx128x_commands_size_e
{
    // Operational Modes Functions
    SX128X_SIZE_SET_SLEEP                = 2,
    SX128X_SIZE_SET_STANDBY              = 2,
    SX128X_SIZE_SET_FS                   = 1,
    SX128X_SIZE_SET_TX                   = 4,
    SX128X_SIZE_SET_RX                   = 4,
    SX128X_SIZE_SET_RX_DUTY_CYCLE        = 6,
    SX128X_SIZE_SET_CAD                  = 1,
    SX128X_SIZE_SET_AUTO_TX              = 3,
    SX128X_SIZE_SET_AUTO_FS              = 2,
    SX128X_SIZE_SET_TX_CONTINUOUS_WAVE   = 1,
    SX128X_SIZE_SET_TX_INFINITE_PREAMBLE = 1,
    // Registers and buffer Access
    // Full size: this value plus buffer size
    SX128X_SIZE_WRITE_REGISTER = 3,
    // Full size: this value plus buffer size
    SX128X_SIZE_READ_REGISTER = 4,
    // Full size: this value plus buffer size
    SX128X_SIZE_WRITE_BUFFER = 2,
    // Full size: this value plus buffer size
    SX128X_SIZE_READ_BUFFER = 3,
    // DIO and IRQ Control Functions
    SX128X_SIZE_SET_DIO_IRQ_PARAMS = 9,
    SX128X_SIZE_GET_IRQ_STATUS     = 2,
    SX128X_SIZE_CLR_IRQ_STATUS     = 3,
    // RF Modulation and Packet-Related Functions
    SX128X_SIZE_SET_RF_FREQUENCY        = 4,
    SX128X_SIZE_SET_PKT_TYPE            = 2,
    SX128X_SIZE_GET_PKT_TYPE            = 2,
    SX128X_SIZE_SET_TX_PARAMS           = 3,
    SX128X_SIZE_SET_MODULATION_PARAMS   = 4,
    SX128X_SIZE_SET_PKT_PARAMS          = 8,
    SX128X_SIZE_SET_CAD_PARAMS          = 2,
    SX128X_SIZE_SET_BUFFER_BASE_ADDRESS = 3,
    // Communication Status Information
    SX128X_SIZE_GET_STATUS           = 1,
    SX128X_SIZE_GET_RX_BUFFER_STATUS = 2,
    SX128X_SIZE_GET_PKT_STATUS       = 2,
    SX128X_SIZE_GET_RSSI_INST        = 2,
    // Miscellaneous
    SX128X_SIZE_SET_LONG_PREAMBLE  = 2,
    SX128X_SIZE_SET_REGULATOR_MODE = 2,
    SX128X_SIZE_SET_SAVE_CONTEXT   = 1,
    SX128X_SIZE_SET_RANGING_ROLE   = 2,
    SX128X_SIZE_SET_ADV_RANGING    = 2,
    SX128X_SIZE_MAX_BUFFER         = 255,
} sx128x_commands_size_t;

/**
 * @brief Associative structure that link bitrate and bandwidth to sx128x_gfsk_ble_br_bw_t
 */
typedef struct
{
    uint32_t                br;     //!< Bitrate (bps)
    uint32_t                bw;     //!< Bandwidth (Hz)
    sx128x_gfsk_ble_br_bw_t param;  //!< Corresponding GFSK/BLE configuration parameter
} sx128x_gfsk_br_bw_convert_t;

static const sx128x_gfsk_br_bw_convert_t sx128x_gfsk_br_bw[] = {
    { 125000, 300000, SX128X_GFSK_BLE_BR_0_125_BW_0_3 },   { 250000, 300000, SX128X_GFSK_BLE_BR_0_250_BW_0_3 },
    { 250000, 600000, SX128X_GFSK_BLE_BR_0_250_BW_0_6 },   { 400000, 600000, SX128X_GFSK_BLE_BR_0_400_BW_0_6 },
    { 500000, 600000, SX128X_GFSK_BLE_BR_0_500_BW_0_6 },   { 400000, 1200000, SX128X_GFSK_BLE_BR_0_400_BW_1_2 },
    { 500000, 1200000, SX128X_GFSK_BLE_BR_0_500_BW_1_2 },  { 800000, 1200000, SX128X_GFSK_BLE_BR_0_800_BW_1_2 },
    { 1000000, 1200000, SX128X_GFSK_BLE_BR_1_000_BW_1_2 }, { 800000, 2400000, SX128X_GFSK_BLE_BR_0_800_BW_2_4 },
    { 1000000, 2400000, SX128X_GFSK_BLE_BR_1_000_BW_2_4 }, { 1600000, 2400000, SX128X_GFSK_BLE_BR_1_600_BW_2_4 },
    { 2000000, 2400000, SX128X_GFSK_BLE_BR_2_000_BW_2_4 },
};

/**
 * @brief Associative structure that link bitrate and bandwidth to sx128x_flrc_br_bw_t
 */
typedef struct
{
    uint32_t            br;     //!< Bitrate (bps)
    uint32_t            bw;     //!< Bandwidth (Hz)
    sx128x_flrc_br_bw_t param;  //!< Corresponding FLRC configuration parameter
} sx128x_flrc_br_bw_convert_t;

static const sx128x_flrc_br_bw_convert_t sx128x_flrc_br_bw[] = {
    { 260000, 300000, SX128X_FLRC_BR_0_260_BW_0_3 },   { 325000, 300000, SX128X_FLRC_BR_0_325_BW_0_3 },
    { 520000, 600000, SX128X_FLRC_BR_0_520_BW_0_6 },   { 650000, 600000, SX128X_FLRC_BR_0_650_BW_0_6 },
    { 1040000, 1200000, SX128X_FLRC_BR_1_040_BW_1_2 }, { 1300000, 1200000, SX128X_FLRC_BR_1_300_BW_1_2 },
};

/**
 * @brief Denominator used in the modulation index search
 *
 * @see sx128x_get_gfsk_mod_ind_param
 */
#define SX128X_GFSK_MOD_IND_DENOMINATOR 20

/**
 * @brief Associative structure that link a numerator value to sx128x_gfsk_ble_mod_ind_t
 *
 * @see sx128x_get_gfsk_mod_ind_param
 */
typedef struct
{
    uint8_t                   numerator;  //!< Numerator value
    sx128x_gfsk_ble_mod_ind_t mod_ind;    //!< Corresponding modulation index
} sx128x_gfsk_mod_ind_frac_t;

static const sx128x_gfsk_mod_ind_frac_t sx128x_gfsk_mod_ind[] = {
    { 7, SX128X_GFSK_BLE_MOD_IND_0_35 },  { 10, SX128X_GFSK_BLE_MOD_IND_0_50 }, { 15, SX128X_GFSK_BLE_MOD_IND_0_75 },
    { 20, SX128X_GFSK_BLE_MOD_IND_1_00 }, { 25, SX128X_GFSK_BLE_MOD_IND_1_25 }, { 30, SX128X_GFSK_BLE_MOD_IND_1_50 },
    { 35, SX128X_GFSK_BLE_MOD_IND_1_75 }, { 40, SX128X_GFSK_BLE_MOD_IND_2_00 }, { 45, SX128X_GFSK_BLE_MOD_IND_2_25 },
    { 50, SX128X_GFSK_BLE_MOD_IND_2_50 }, { 55, SX128X_GFSK_BLE_MOD_IND_2_75 }, { 60, SX128X_GFSK_BLE_MOD_IND_3_00 },
    { 65, SX128X_GFSK_BLE_MOD_IND_3_25 }, { 70, SX128X_GFSK_BLE_MOD_IND_3_50 }, { 75, SX128X_GFSK_BLE_MOD_IND_3_75 },
    { 80, SX128X_GFSK_BLE_MOD_IND_4_00 },

};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Determine number of bits used for a given preamble
 *
 * @param [in] preamble_len Enumeration indicating preamble length
 *
 * @returns Bit count
 */
static inline uint32_t sx128x_get_gfsk_flrc_preamble_len_in_bits( sx128x_gfsk_preamble_len_t preamble_len );

/**
 * @brief Determine number of bytes used for a given sync word
 *
 * @param [in] sync_word_len Enumeration indicating sync word length
 *
 * @returns Byte count
 */
static inline uint32_t sx128x_get_gfsk_sync_word_len_in_bytes( sx128x_gfsk_sync_word_len_t sync_word_len );

/**
 * @brief Determine number of bytes used for a given CRC
 *
 * @param [in] crc_type Enumeration indicating CRC type
 *
 * @returns Byte count
 */
static inline uint32_t sx128x_get_gfsk_crc_len_in_bytes( sx128x_gfsk_crc_types_t crc_type );

/**
 * @brief Determine number of bytes used for a given CRC
 *
 * @param [in] crc_type Enumeration indicating CRC type
 *
 * @returns Byte count
 */
static inline uint32_t sx128x_get_flrc_crc_len_in_bytes( sx128x_flrc_crc_types_t crc_type );

/**
 * @brief Determine number of bits used for a given header
 *
 * @param [in] header_type Enumeration indicating header type
 *
 * @returns Bit count
 */
static inline uint32_t sx128x_get_gfsk_header_len_in_bits( sx128x_gfsk_flrc_pkt_len_modes_t header_type );

/**
 * @brief Determine number of bytes used for a given sync word
 *
 * @param [in] sync_word_len Enumeration indicating sync word length
 *
 * @returns Byte count
 */
static inline uint32_t sx128x_get_flrc_sync_word_len_in_bytes( sx128x_flrc_sync_word_len_t sync_word_len );

/**
 * @brief Determine number of bits used for a given header
 *
 * @param [in] header_type Enumeration indicating header type
 *
 * @returns Bit count
 */
static inline uint32_t sx128x_get_flrc_header_len_in_bits( sx128x_gfsk_flrc_pkt_len_modes_t header_type );

/**
 * @brief Determine number of bits used for tail of a FLRC packet
 *
 * @param [in] cr Enumeration indicating the coding rate
 *
 * @returns Bit count
 */
static inline uint32_t sx128x_get_flrc_tail_len_in_bits( sx128x_flrc_cr_t cr );

/**
 * @brief Determine the denominator of the coding-rate time-on-air multiplier
 *
 * @param [in] cr Enumeration indicating the coding rate
 *
 * @returns Denominator of the coding-rate time-on-air multiplier
 */
static inline uint32_t sx128x_get_flrc_cr_den( sx128x_flrc_cr_t cr );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

sx128x_status_t sx128x_set_sleep( const void* context, const bool retain_data_buffer,
                                  const bool retain_radio_configuration )
{
    const uint8_t buf[SX128X_SIZE_SET_SLEEP] = {
        SX128X_SET_SLEEP,
        ( ( retain_data_buffer == true ) ? SX128X_SLEEP_CFG_DATA_BUFFER_RETENTION : 0 ) +
            ( ( retain_radio_configuration == true ) ? SX128X_SLEEP_CFG_DATA_RETENTION : 0 ),
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_SLEEP, 0, 0 );
}

sx128x_status_t sx128x_set_standby( const void* context, const sx128x_standby_cfg_t cfg )
{
    const uint8_t buf[SX128X_SIZE_SET_STANDBY] = {
        SX128X_SET_STANDBY,
        ( uint8_t ) cfg,
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_STANDBY, 0, 0 );
}

sx128x_status_t sx128x_wakeup( const void* context )
{
    sx128x_status_t status = ( sx128x_status_t ) sx128x_hal_wakeup( context );
    if( status != SX128X_STATUS_OK )
    {
        return status;
    }

    // WORKAROUND: rssi is 0 after a sleep
    sx128x_pkt_type_t pkt_type;
    status = sx128x_get_pkt_type( context, &pkt_type );
    if( status != SX128X_STATUS_OK )
    {
        return status;
    }

    if( pkt_type == SX128X_PKT_TYPE_LORA )
    {
        uint8_t fix[] = SX128X_REG_RSSI_SNR_BUGFIX_BLOB;
        return sx128x_write_register( context, SX128X_REG_RSSI_SNR_BUGFIX_ADDRESS, fix, sizeof( fix ) );
    }

    return status;
}

sx128x_status_t sx128x_set_fs( const void* context )
{
    const uint8_t buf[SX128X_SIZE_SET_FS] = {
        SX128X_SET_FS,
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_FS, 0, 0 );
}

sx128x_status_t sx128x_set_tx( const void* context, sx128x_tick_size_t period_base, const uint16_t period_base_count )
{
    const uint8_t buf[SX128X_SIZE_SET_TX] = {
        SX128X_SET_TX,
        ( uint8_t ) period_base,
        ( uint8_t )( period_base_count >> 8 ),
        ( uint8_t )( period_base_count >> 0 ),
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_TX, 0, 0 );
}

sx128x_status_t sx128x_set_rx( const void* context, sx128x_tick_size_t period_base, const uint16_t period_base_count )
{
    const uint8_t buf[SX128X_SIZE_SET_RX] = {
        SX128X_SET_RX,
        ( uint8_t ) period_base,
        ( uint8_t )( period_base_count >> 8 ),
        ( uint8_t )( period_base_count >> 0 ),
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_RX, 0, 0 );
}

sx128x_status_t sx128x_set_rx_duty_cycle( const void* context, sx128x_tick_size_t period_base,
                                          const uint16_t rx_period_base_count, const uint16_t sleep_period_base_count )
{
    const uint8_t buf[SX128X_SIZE_SET_RX_DUTY_CYCLE] = {
        SX128X_SET_RX_DUTY_CYCLE,
        ( uint8_t ) period_base,
        ( uint8_t )( rx_period_base_count >> 8 ),
        ( uint8_t )( rx_period_base_count >> 0 ),
        ( uint8_t )( sleep_period_base_count >> 8 ),
        ( uint8_t )( sleep_period_base_count >> 0 ),
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_RX_DUTY_CYCLE, 0, 0 );
}

sx128x_status_t sx128x_set_cad( const void* context )
{
    const uint8_t buf[SX128X_SIZE_SET_CAD] = {
        SX128X_SET_CAD,
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_CAD, 0, 0 );
}

sx128x_status_t sx128x_set_auto_fs( const void* context, bool is_enabled )
{
    const uint8_t buf[SX128X_SIZE_SET_AUTO_FS] = {
        SX128X_SET_AUTO_FS,
        ( uint8_t )( is_enabled ? 0x01 : 0x00 ),
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_AUTO_FS, 0, 0 );
}

sx128x_status_t sx128x_set_tx_cw( const void* context )
{
    const uint8_t buf[SX128X_SIZE_SET_TX_CONTINUOUS_WAVE] = {
        SX128X_SET_TX_CONTINUOUS_WAVE,
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_TX_CONTINUOUS_WAVE, 0, 0 );
}

sx128x_status_t sx128x_set_tx_infinite_preamble( const void* context )
{
    const uint8_t buf[SX128X_SIZE_SET_TX_INFINITE_PREAMBLE] = {
        SX128X_SET_TX_INFINITE_PREAMBLE,
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_TX_INFINITE_PREAMBLE, 0, 0 );
}

//
// Registers and buffer Access
//

sx128x_status_t sx128x_write_register( const void* context, const uint16_t address, const uint8_t* buffer,
                                       const uint16_t size )
{
    const uint8_t buf[SX128X_SIZE_WRITE_REGISTER] = {
        SX128X_WRITE_REGISTER,
        ( uint8_t )( address >> 8 ),
        ( uint8_t )( address >> 0 ),
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_WRITE_REGISTER, buffer, size );
}

sx128x_status_t sx128x_read_register( const void* context, const uint16_t address, uint8_t* buffer,
                                      const uint16_t size )
{
    const uint8_t buf[SX128X_SIZE_READ_REGISTER] = {
        SX128X_READ_REGISTER,
        ( uint8_t )( address >> 8 ),
        ( uint8_t )( address >> 0 ),
        SX128X_NOP,
    };

    return ( sx128x_status_t ) sx128x_hal_read( context, buf, SX128X_SIZE_READ_REGISTER, buffer, size );
}

sx128x_status_t sx128x_write_buffer( const void* context, const uint8_t offset, const uint8_t* buffer,
                                     const uint16_t size )
{
    const uint8_t buf[SX128X_SIZE_WRITE_BUFFER] = {
        SX128X_WRITE_BUFFER,
        offset,
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_WRITE_BUFFER, buffer, size );
}

sx128x_status_t sx128x_read_buffer( const void* context, const uint8_t offset, uint8_t* buffer, const uint16_t size )
{
    const uint8_t buf[SX128X_SIZE_READ_BUFFER] = {
        SX128X_READ_BUFFER,
        offset,
        SX128X_NOP,
    };

    return ( sx128x_status_t ) sx128x_hal_read( context, buf, SX128X_SIZE_READ_BUFFER, buffer, size );
}

//
// DIO and IRQ Control Functions
//
sx128x_status_t sx128x_set_dio_irq_params( const void* context, const uint16_t irq_mask, const uint16_t dio1_mask,
                                           const uint16_t dio2_mask, const uint16_t dio3_mask )
{
    const uint8_t buf[SX128X_SIZE_SET_DIO_IRQ_PARAMS] = {
        SX128X_SET_DIO_IRQ_PARAMS,     ( uint8_t )( irq_mask >> 8 ),  ( uint8_t )( irq_mask >> 0 ),
        ( uint8_t )( dio1_mask >> 8 ), ( uint8_t )( dio1_mask >> 0 ), ( uint8_t )( dio2_mask >> 8 ),
        ( uint8_t )( dio2_mask >> 0 ), ( uint8_t )( dio3_mask >> 8 ), ( uint8_t )( dio3_mask >> 0 ),
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_DIO_IRQ_PARAMS, 0, 0 );
}

sx128x_status_t sx128x_get_irq_status( const void* context, sx128x_irq_mask_t* irq )
{
    uint8_t irq_local[sizeof( sx128x_irq_mask_t )] = { 0x00 };

    const uint8_t buf[SX128X_SIZE_GET_IRQ_STATUS] = {
        SX128X_GET_IRQ_STATUS,
        SX128X_NOP,
    };

    sx128x_status_t status = ( sx128x_status_t ) sx128x_hal_read( context, buf, SX128X_SIZE_GET_IRQ_STATUS, irq_local,
                                                                  sizeof( sx128x_irq_mask_t ) );

    if( status == SX128X_STATUS_OK )
    {
        *irq = ( ( sx128x_irq_mask_t ) irq_local[0] << 8 ) + ( ( sx128x_irq_mask_t ) irq_local[1] << 0 );
    }

    return status;
}

sx128x_status_t sx128x_clear_irq_status( const void* context, const sx128x_irq_mask_t irq_mask )
{
    const uint8_t buf[SX128X_SIZE_CLR_IRQ_STATUS] = {
        SX128X_CLR_IRQ_STATUS,
        ( uint8_t )( irq_mask >> 8 ),
        ( uint8_t )( irq_mask >> 0 ),
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_CLR_IRQ_STATUS, 0, 0 );
}

sx128x_status_t sx128x_get_and_clear_irq_status( const void* context, sx128x_irq_mask_t* irq )
{
    sx128x_irq_mask_t sx128x_irq_mask = SX128X_IRQ_NONE;

    sx128x_status_t status = sx128x_get_irq_status( context, &sx128x_irq_mask );

    if( ( status == SX128X_STATUS_OK ) && ( sx128x_irq_mask != 0 ) )
    {
        status = sx128x_clear_irq_status( context, sx128x_irq_mask );
    }
    if( ( status == SX128X_STATUS_OK ) && ( irq != NULL ) )
    {
        *irq = sx128x_irq_mask;
    }
    return status;
}

//
// RF Modulation and Packet-Related Functions
//

sx128x_status_t sx128x_set_rf_freq( const void* context, const uint32_t freq_in_hz )
{
    const uint32_t freq = sx128x_convert_freq_in_hz_to_pll_step( freq_in_hz );

    return sx128x_set_rf_freq_in_pll_steps( context, freq );
}

sx128x_status_t sx128x_set_rf_freq_in_pll_steps( const void* context, const uint32_t freq )
{
    const uint8_t buf[SX128X_SIZE_SET_RF_FREQUENCY] = {
        SX128X_SET_RF_FREQUENCY,
        ( uint8_t )( freq >> 16 ),
        ( uint8_t )( freq >> 8 ),
        ( uint8_t )( freq >> 0 ),
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_RF_FREQUENCY, 0, 0 );
}

sx128x_status_t sx128x_set_pkt_type( const void* context, const sx128x_pkt_type_t pkt_type )
{
    const uint8_t buf[SX128X_SIZE_SET_PKT_TYPE] = {
        SX128X_SET_PKT_TYPE,
        ( uint8_t ) pkt_type,
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_PKT_TYPE, 0, 0 );
}

sx128x_status_t sx128x_get_pkt_type( const void* context, sx128x_pkt_type_t* pkt_type )
{
    uint8_t pkt_type_raw;

    const uint8_t buf[SX128X_SIZE_GET_PKT_TYPE] = {
        SX128X_GET_PKT_TYPE,
        SX128X_NOP,
    };

    sx128x_status_t status =
        ( sx128x_status_t ) sx128x_hal_read( context, buf, SX128X_SIZE_GET_PKT_TYPE, &pkt_type_raw, 1 );
    *pkt_type = ( sx128x_pkt_type_t ) pkt_type_raw;

    return status;
}

sx128x_status_t sx128x_set_tx_params( const void* context, const int8_t pwr_in_dbm, const sx128x_ramp_time_t ramp_time )
{
    if( ( pwr_in_dbm < SX128X_PWR_MIN ) || ( pwr_in_dbm > SX128X_PWR_MAX ) )
    {
        return SX128X_STATUS_ERROR;
    }

    const uint8_t buf[SX128X_SIZE_SET_TX_PARAMS] = {
        SX128X_SET_TX_PARAMS,
        ( uint8_t )( pwr_in_dbm + 18 ),
        ( uint8_t ) ramp_time,
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_TX_PARAMS, 0, 0 );
}

sx128x_status_t sx128x_set_gfsk_mod_params( const void* context, const sx128x_mod_params_gfsk_t* params )
{
    const uint8_t buf[SX128X_SIZE_SET_MODULATION_PARAMS] = {
        SX128X_SET_MODULATION_PARAMS,
        params->br_bw,
        params->mod_ind,
        params->pulse_shape,
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_MODULATION_PARAMS, 0, 0 );
}

sx128x_status_t sx128x_set_lora_mod_params( const void* context, const sx128x_mod_params_lora_t* params )
{
    const uint8_t buf[SX128X_SIZE_SET_MODULATION_PARAMS] = {
        SX128X_SET_MODULATION_PARAMS,
        ( uint8_t )( params->sf ),
        ( uint8_t )( params->bw ),
        ( uint8_t )( params->cr ),
    };

    sx128x_status_t status =
        ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_MODULATION_PARAMS, 0, 0 );

    if( status == SX128X_STATUS_OK )
    {
        uint8_t reg_buffer;

        switch( params->sf )
        {
        case SX128X_LORA_RANGING_SF5:
        case SX128X_LORA_RANGING_SF6:
            reg_buffer = 0x1E;
            break;
        case SX128X_LORA_RANGING_SF7:
        case SX128X_LORA_RANGING_SF8:
            reg_buffer = 0x37;
            break;
        case SX128X_LORA_RANGING_SF9:
        case SX128X_LORA_RANGING_SF10:
        case SX128X_LORA_RANGING_SF11:
        case SX128X_LORA_RANGING_SF12:
            reg_buffer = 0x32;
            break;
        }

        status = sx128x_write_register( context, SX128X_REG_LORA_SF_CFG0, &reg_buffer, 1 );

        if( status == SX128X_STATUS_OK )
        {
            reg_buffer = 0;
            status     = sx128x_read_register( context, SX128X_REG_LORA_FREQ_ERR_CORR, &reg_buffer, 1 );

            if( status == SX128X_STATUS_OK )
            {
                reg_buffer &= ~SX128X_REG_LORA_FREQ_ERR_CORR_MASK;
                reg_buffer |= SX128X_REG_LORA_FREQ_ERR_CORR_MODE_1;

                status = sx128x_write_register( context, SX128X_REG_LORA_FREQ_ERR_CORR, &reg_buffer, 1 );
            }
        }
    }

    return status;
}

sx128x_status_t sx128x_set_flrc_mod_params( const void* context, const sx128x_mod_params_flrc_t* params )
{
    const uint8_t buf[SX128X_SIZE_SET_MODULATION_PARAMS] = {
        SX128X_SET_MODULATION_PARAMS,
        ( uint8_t )( params->br_bw ),
        ( uint8_t )( params->cr ),
        ( uint8_t )( params->pulse_shape ),
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_MODULATION_PARAMS, 0, 0 );
}

sx128x_status_t sx128x_set_ble_mod_params( const void* context, const sx128x_mod_params_ble_t* params )
{
    const uint8_t buf[SX128X_SIZE_SET_MODULATION_PARAMS] = {
        SX128X_SET_MODULATION_PARAMS,
        ( uint8_t )( params->br_bw ),
        ( uint8_t )( params->mod_ind ),
        ( uint8_t )( params->pulse_shape ),
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_MODULATION_PARAMS, 0, 0 );
}

sx128x_status_t sx128x_set_gfsk_pkt_params( const void* context, const sx128x_pkt_params_gfsk_t* params )
{
    const uint8_t buf[SX128X_SIZE_SET_PKT_PARAMS] = {
        SX128X_SET_PKT_PARAMS,
        ( uint8_t )( params->preamble_len ),
        ( uint8_t )( params->sync_word_len ),
        ( uint8_t )( params->match_sync_word ),
        ( uint8_t )( params->header_type ),
        params->pld_len_in_bytes,
        ( uint8_t )( params->crc_type ),
        ( uint8_t )( params->dc_free ),
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_PKT_PARAMS, 0, 0 );
}

sx128x_status_t sx128x_set_lora_pkt_params( const void* context, const sx128x_pkt_params_lora_t* params )
{
    const uint8_t buf[SX128X_SIZE_SET_PKT_PARAMS] = {
        SX128X_SET_PKT_PARAMS,
        params->preamble_len.mant | ( params->preamble_len.exp << 4 ),
        ( uint8_t )( params->header_type ),
        params->pld_len_in_bytes,
        ( uint8_t )( ( params->crc_is_on == true ) ? 0x20 : 0x00 ),
        ( uint8_t )( ( params->invert_iq_is_on == true ) ? 0x00 : 0x40 ),
    };

    sx128x_status_t status = ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_PKT_PARAMS, 0, 0 );

    if( status == SX128X_STATUS_OK )
    {
        uint8_t data = ( buf[5] == 0x00 ) ? 0x0D : 0x09;
        status       = sx128x_write_register( context, SX128X_REG_LR_IQ_CONF, &data, 1 );
    }

    return status;
}

sx128x_status_t sx128x_set_flrc_pkt_params( const void* context, const sx128x_pkt_params_flrc_t* params )
{
    const uint8_t buf[SX128X_SIZE_SET_PKT_PARAMS] = {
        SX128X_SET_PKT_PARAMS,
        ( uint8_t )( params->preamble_len ),
        ( uint8_t )( params->sync_word_len ),
        ( uint8_t )( params->match_sync_word ),
        ( uint8_t )( params->header_type ),
        params->pld_len_in_bytes,
        ( uint8_t )( params->crc_type ),
        SX128X_GFSK_FLRC_BLE_DC_FREE_OFF,
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_PKT_PARAMS, 0, 0 );
}

sx128x_status_t sx128x_set_ble_pkt_params( const void* context, const sx128x_pkt_params_ble_t* params )
{
    const uint8_t buf[SX128X_SIZE_SET_PKT_PARAMS] = {
        SX128X_SET_PKT_PARAMS,           ( uint8_t )( params->con_state ), ( uint8_t )( params->crc_type ),
        ( uint8_t )( params->pkt_type ), ( uint8_t )( params->dc_free ),
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_PKT_PARAMS, 0, 0 );
}

sx128x_status_t sx128x_set_cad_params( const void* context, const sx128x_lora_cad_params_t* params )
{
    const uint8_t buf[SX128X_SIZE_SET_CAD_PARAMS] = {
        SX128X_SET_CAD_PARAMS,
        ( uint8_t ) params->cad_symb_nb,
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_CAD_PARAMS, 0, 0 );
}

sx128x_status_t sx128x_set_buffer_base_address( const void* context, const uint8_t tx_base_address,
                                                const uint8_t rx_base_address )
{
    const uint8_t buf[SX128X_SIZE_SET_BUFFER_BASE_ADDRESS] = {
        SX128X_SET_BUFFER_BASE_ADDRESS,
        tx_base_address,
        rx_base_address,
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_BUFFER_BASE_ADDRESS, 0, 0 );
}

//
// Communication Status Information
//

sx128x_status_t sx128x_get_status( const void* context, sx128x_chip_status_t* radio_status )
{
    uint8_t radio_status_raw = 0x00;

    const uint8_t buf[SX128X_SIZE_GET_STATUS] = {
        SX128X_GET_STATUS,
    };

    sx128x_status_t status =
        ( sx128x_status_t ) sx128x_hal_read( context, buf, SX128X_SIZE_GET_STATUS, &radio_status_raw, 1 );

    if( status == SX128X_STATUS_OK )
    {
        radio_status->cmd_status =
            ( sx128x_cmd_status_t )( ( radio_status_raw >> SX128X_CMD_STATUS_POS ) & SX128X_CMD_STATUS_MASK );
        radio_status->chip_mode =
            ( sx128x_chip_modes_t )( ( radio_status_raw >> SX128X_CHIP_MODES_POS ) & SX128X_CHIP_MODES_MASK );
    }

    return status;
}

sx128x_status_t sx128x_get_rx_buffer_status( const void* context, sx128x_rx_buffer_status_t* rx_buffer_status )
{
    uint8_t rx_buffer_status_raw[sizeof( sx128x_rx_buffer_status_t )] = { 0x00 };

    const uint8_t buf[SX128X_SIZE_GET_RX_BUFFER_STATUS] = {
        SX128X_GET_RX_BUFFER_STATUS,
        SX128X_NOP,
    };

    sx128x_status_t status = ( sx128x_status_t ) sx128x_hal_read(
        context, buf, SX128X_SIZE_GET_RX_BUFFER_STATUS, rx_buffer_status_raw, sizeof( sx128x_rx_buffer_status_t ) );

    if( status == SX128X_STATUS_OK )
    {
        rx_buffer_status->pld_len_in_bytes     = rx_buffer_status_raw[0];
        rx_buffer_status->buffer_start_pointer = rx_buffer_status_raw[1];
    }

    return status;
}

sx128x_status_t sx128x_get_lora_implicit_payload_len( const void* context, uint8_t* payload_len )
{
    return sx128x_read_register( context, SX128X_REG_LR_PAYLOAD_LENGTH, payload_len, 1 );
}

sx128x_status_t sx128x_get_gfsk_pkt_status( const void* context, sx128x_pkt_status_gfsk_t* pkt_status )
{
    uint8_t pkt_status_raw[SX128X_PKT_STATUS_LEN] = { 0x00 };

    const uint8_t buf[SX128X_SIZE_GET_PKT_STATUS] = {
        SX128X_GET_PKT_STATUS,
        SX128X_NOP,
    };

    sx128x_status_t status =
        ( sx128x_status_t ) sx128x_hal_read( context, buf, SX128X_SIZE_GET_PKT_STATUS, pkt_status_raw, 5 );

    if( status == SX128X_STATUS_OK )
    {
        // pkt_status_raw[0] is RFU
        pkt_status->rssi   = ( int8_t )( -pkt_status_raw[1] >> 1 );
        pkt_status->errors = pkt_status_raw[2];
        pkt_status->status = pkt_status_raw[3];
        pkt_status->sync   = pkt_status_raw[4];
    }

    return status;
}

sx128x_status_t sx128x_get_lora_pkt_status( const void* context, sx128x_pkt_status_lora_t* pkt_status )
{
    uint8_t pkt_status_raw[SX128X_PKT_STATUS_LEN] = { 0x00 };

    const uint8_t buf[SX128X_SIZE_GET_PKT_STATUS] = {
        SX128X_GET_PKT_STATUS,
        SX128X_NOP,
    };

    sx128x_status_t status =
        ( sx128x_status_t ) sx128x_hal_read( context, buf, SX128X_SIZE_GET_PKT_STATUS, pkt_status_raw, 5 );

    if( status == SX128X_STATUS_OK )
    {
        pkt_status->rssi = ( int8_t )( -pkt_status_raw[0] >> 1 );
        pkt_status->snr  = ( ( ( int8_t ) pkt_status_raw[1] ) + 2 ) >> 2;
    }

    return status;
}

sx128x_status_t sx128x_get_flrc_pkt_status( const void* context, sx128x_pkt_status_flrc_t* pkt_status )
{
    uint8_t pkt_status_raw[SX128X_PKT_STATUS_LEN] = { 0x00 };

    const uint8_t buf[SX128X_SIZE_GET_PKT_STATUS] = {
        SX128X_GET_PKT_STATUS,
        SX128X_NOP,
    };

    sx128x_status_t status =
        ( sx128x_status_t ) sx128x_hal_read( context, buf, SX128X_SIZE_GET_PKT_STATUS, pkt_status_raw, 5 );

    if( status == SX128X_STATUS_OK )
    {
        // pkt_status_raw[0] is RFU
        pkt_status->rssi   = ( int8_t )( -pkt_status_raw[1] >> 1 );
        pkt_status->errors = pkt_status_raw[2];
        pkt_status->status = pkt_status_raw[3];
        pkt_status->sync   = pkt_status_raw[4];
    }

    return status;
}

sx128x_status_t sx128x_get_ble_pkt_status( const void* context, sx128x_pkt_status_ble_t* pkt_status )
{
    uint8_t pkt_status_raw[SX128X_PKT_STATUS_LEN] = { 0x00 };

    const uint8_t buf[SX128X_SIZE_GET_PKT_STATUS] = {
        SX128X_GET_PKT_STATUS,
        SX128X_NOP,
    };

    sx128x_status_t status =
        ( sx128x_status_t ) sx128x_hal_read( context, buf, SX128X_SIZE_GET_PKT_STATUS, pkt_status_raw, 5 );

    if( status == SX128X_STATUS_OK )
    {
        // pkt_status_raw[0] is RFU
        pkt_status->rssi   = ( int8_t )( -pkt_status_raw[1] >> 1 );
        pkt_status->errors = pkt_status_raw[2];
        pkt_status->status = pkt_status_raw[3];
        pkt_status->sync   = pkt_status_raw[4];
    }

    return status;
}

sx128x_status_t sx128x_get_rssi_inst( const void* context, int16_t* rssi )
{
    uint8_t rssi_raw = 0x00;

    const uint8_t buf[SX128X_SIZE_GET_RSSI_INST] = {
        SX128X_GET_RSSI_INST,
        SX128X_NOP,
    };

    sx128x_status_t status =
        ( sx128x_status_t ) sx128x_hal_read( context, buf, SX128X_SIZE_GET_RSSI_INST, &rssi_raw, 1 );

    if( status == SX128X_STATUS_OK )
    {
        *rssi = ( int16_t )( -rssi_raw >> 1 );
    }

    return status;
}

//
// Miscellaneous
//

sx128x_status_t sx128x_reset( const void* context )
{
    return ( sx128x_status_t ) sx128x_hal_reset( context );
}

sx128x_status_t sx128x_set_long_preamble( const void* context, const bool state )
{
    const uint8_t buf[SX128X_SIZE_SET_LONG_PREAMBLE] = {
        SX128X_SET_LONG_PREAMBLE,
        ( uint8_t )( ( state == true ) ? 0x01 : 0x00 ),
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_LONG_PREAMBLE, 0, 0 );
}

sx128x_status_t sx128x_set_reg_mode( const void* context, const sx128x_reg_mod_t mode )
{
    const uint8_t buf[SX128X_SIZE_SET_REGULATOR_MODE] = {
        SX128X_SET_REGULATOR_MODE,
        ( uint8_t ) mode,
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_REGULATOR_MODE, 0, 0 );
}

sx128x_status_t sx128x_set_lna_settings( const void* context, sx128x_lna_settings_t settings )
{
    sx128x_status_t status;
    uint8_t         reg_value;

    status = sx128x_read_register( context, SX128X_REG_LNA_REGIME, &reg_value, 1 );

    if( status == SX128X_STATUS_OK )
    {
        if( settings == SX128X_LNA_HIGH_SENSITIVITY_MODE )
        {
            reg_value |= 0xC0;
        }
        if( settings == SX128X_LNA_LOW_POWER_MODE )
        {
            reg_value &= ~0xC0;
        }

        status = sx128x_write_register( context, SX128X_REG_LNA_REGIME, &reg_value, 1 );
    }

    return status;
}

sx128x_status_t sx128x_save_context( const void* context )
{
    const uint8_t buf[SX128X_SIZE_SET_SAVE_CONTEXT] = {
        SX128X_SET_SAVE_CONTEXT,
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_SAVE_CONTEXT, 0, 0 );
}

sx128x_status_t sx128x_set_ranging_role( const void* context, const sx128x_ranging_role_t role )
{
    uint8_t buf[SX128X_SIZE_SET_RANGING_ROLE] = {
        SX128X_SET_RANGING_ROLE,
        ( uint8_t ) role,
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_RANGING_ROLE, 0, 0 );
}

sx128x_status_t sx128x_set_adv_ranging( const void* context, const bool state )
{
    uint8_t buf[SX128X_SIZE_SET_ADV_RANGING] = {
        SX128X_SET_ADV_RANGING,
        ( uint8_t )( ( state == true ) ? 0x01 : 0x00 ),
    };

    return ( sx128x_status_t ) sx128x_hal_write( context, buf, SX128X_SIZE_SET_ADV_RANGING, 0, 0 );
}

sx128x_status_t sx128x_set_gfsk_flrc_sync_word_tolerance( const void* context, uint8_t tolerance )
{
    sx128x_status_t status;
    uint8_t         reg_value;

    status = sx128x_read_register( context, SX128X_REG_GFSK_FLRC_SYNC_WORD_TOLERANCE, &reg_value, 1 );

    if( status == SX128X_STATUS_OK )
    {
        reg_value = ( reg_value & 0xF0 ) | ( tolerance & 0x0F );
        status    = sx128x_write_register( context, SX128X_REG_GFSK_FLRC_SYNC_WORD_TOLERANCE, &reg_value, 1 );
    }

    return status;
}

sx128x_status_t sx128x_get_gfsk_br_bw_param( const uint32_t br, const uint32_t bw, sx128x_gfsk_ble_br_bw_t* param )
{
    sx128x_status_t status = SX128X_STATUS_ERROR;

    if( ( br != 0 ) && ( bw != 0 ) )
    {
        status = SX128X_STATUS_UNKNOWN_VALUE;
        for( uint8_t i = 0; i < ( sizeof( sx128x_gfsk_br_bw ) / sizeof( sx128x_gfsk_br_bw_convert_t ) ); i++ )
        {
            if( ( br <= sx128x_gfsk_br_bw[i].br ) && ( bw <= sx128x_gfsk_br_bw[i].bw ) )
            {
                *param = sx128x_gfsk_br_bw[i].param;
                status = SX128X_STATUS_OK;
                break;
            }
        }
    }

    return status;
}

sx128x_status_t sx128x_get_gfsk_mod_ind_param( const uint32_t br, const uint32_t fdev,
                                               sx128x_gfsk_ble_mod_ind_t* param )
{
    sx128x_status_t status = SX128X_STATUS_ERROR;

    if( br != 0 )
    {
        status = SX128X_STATUS_UNKNOWN_VALUE;

        for( int8_t i = ( sizeof( sx128x_gfsk_mod_ind ) / sizeof( sx128x_gfsk_mod_ind_frac_t ) ) - 1; i >= 0; i-- )
        {
            if( ( br * sx128x_gfsk_mod_ind[i].numerator ) <= ( ( 2 * SX128X_GFSK_MOD_IND_DENOMINATOR ) * fdev ) )
            {
                *param = sx128x_gfsk_mod_ind[i].mod_ind;
                status = SX128X_STATUS_OK;
                break;
            }
        }
    }

    return status;
}

sx128x_status_t sx128x_get_flrc_br_bw_param( const uint32_t br, const uint32_t bw, sx128x_flrc_br_bw_t* param )
{
    sx128x_status_t status = SX128X_STATUS_ERROR;

    if( ( br != 0 ) && ( bw != 0 ) )
    {
        status = SX128X_STATUS_UNKNOWN_VALUE;
        for( uint8_t i = 0; i < ( sizeof( sx128x_flrc_br_bw ) / sizeof( sx128x_flrc_br_bw_convert_t ) ); i++ )
        {
            if( ( br <= sx128x_flrc_br_bw[i].br ) && ( bw <= sx128x_flrc_br_bw[i].bw ) )
            {
                *param = sx128x_flrc_br_bw[i].param;
                status = SX128X_STATUS_OK;
                break;
            }
        }
    }

    return status;
}

uint32_t sx128x_get_lora_bw_in_hz( sx128x_lora_bw_t bw )
{
    switch( bw )
    {
    case SX128X_LORA_RANGING_BW_200:
        return 203125UL;
    case SX128X_LORA_RANGING_BW_400:
        return 406250UL;
    case SX128X_LORA_RANGING_BW_800:
        return 812500UL;
    case SX128X_LORA_RANGING_BW_1600:
        return 1625000UL;
    }
    return 0;
}

uint32_t sx128x_get_lora_time_on_air_numerator( const sx128x_pkt_params_lora_t* pkt_params,
                                                const sx128x_mod_params_lora_t* mod_params )
{
    const bool    pld_is_fix = pkt_params->header_type == SX128X_LORA_RANGING_PKT_IMPLICIT;
    const uint8_t cr_plus_4  = ( mod_params->cr == 7 ) ? 8 : ( ( mod_params->cr - 1 ) & 0x3 ) + 5;

    const int32_t pld_len_in_bits   = pkt_params->pld_len_in_bytes << 3;
    const int32_t sf                = mod_params->sf >> 4;
    const int32_t n_sym_header      = pld_is_fix == true ? 0 : 20;
    const int32_t n_bit_crc         = ( pkt_params->crc_is_on == true ) ? 16 : 0;
    const bool    long_interleaving = mod_params->cr > SX128X_LORA_RANGING_CR_4_8;

    int32_t preamble_len_in_symb = ( ( int32_t ) pkt_params->preamble_len.mant ) << pkt_params->preamble_len.exp;

    int32_t ceil_num;
    int32_t ceil_den;
    int32_t max_val = pld_len_in_bits + n_bit_crc;
    int32_t ceiling;

    if( pld_is_fix == true && long_interleaving == true )
    {
        int32_t n_sym_begin_num = max_val * cr_plus_4;
        int32_t n_sym_begin_den;

        if( sf < 7 )
        {
            n_sym_begin_den = 4 * sf;
        }
        else if( sf <= 10 )
        {
            n_sym_begin_den = 4 * ( sf - 2 );
        }
        else
        {
            n_sym_begin_den = 4 * ( sf - 2 );
        }

        const int32_t n_sym_begin = ( n_sym_begin_num + n_sym_begin_den - 1 ) / n_sym_begin_den;

        if( n_sym_begin <= 8 )
        {
            ceiling = n_sym_begin;
        }
        else
        {
            if( sf < 7 )
            {
                ceil_num = n_sym_begin_num - 32 * sf;
                ceil_den = 4 * sf;
            }
            else if( sf <= 10 )
            {
                ceil_num = n_sym_begin_num + 64 - 32 * sf;
                ceil_den = 4 * sf;
            }
            else
            {
                ceil_num = n_sym_begin_num - 32 * ( sf - 2 );
                ceil_den = 4 * ( sf - 2 );
            }

            ceiling = ( ( ceil_num + ceil_den - 1 ) / ceil_den ) + 8;
        }
    }
    else
    {
        if( long_interleaving == true )
        {
            int32_t n_bit_header_space;
            if( sf < 7 )
            {
                n_bit_header_space = ( ( sf - 5 ) >> 1 ) * 8;
            }
            else
            {
                n_bit_header_space = ( ( sf - 7 ) >> 1 ) * 8;
            }

            if( max_val > n_bit_header_space )
            {
                if( pld_len_in_bits < n_bit_header_space )
                {
                    n_bit_header_space = pld_len_in_bits;
                }
            }

            max_val -= n_bit_header_space;

            if( max_val < 0 )
            {
                max_val = 0;
            }

            ceil_num = max_val * cr_plus_4;
        }
        else
        {
            max_val = max_val - ( 4 * sf ) + n_sym_header;

            if( sf >= 7 )
            {
                max_val += 8;
            }

            if( max_val < 0 )
            {
                max_val = 0;
            }

            ceil_num = max_val;
        }

        if( sf <= 10 )
        {
            ceil_den = 4 * sf;
        }
        else
        {
            ceil_den = 4 * ( sf - 2 );
        }

        ceiling = ( ( ceil_num + ceil_den - 1 ) / ceil_den );

        if( long_interleaving == false )
        {
            ceiling *= cr_plus_4;
        }

        ceiling += 8;
    }

    int32_t intermed = ceiling + preamble_len_in_symb + 4;

    if( sf < 7 )
    {
        intermed += 2;
    }

    // Add the quarter symbol and calculate the TOA numerator
    return ( uint32_t )( ( 4 * intermed + 1 ) * ( 1 << ( sf - 2 ) ) );
}

uint32_t sx128x_get_lora_time_on_air_in_ms( const sx128x_pkt_params_lora_t* pkt_params,
                                            const sx128x_mod_params_lora_t* mod_params )
{
    uint32_t numerator   = 1000U * sx128x_get_lora_time_on_air_numerator( pkt_params, mod_params );
    uint32_t denominator = sx128x_get_lora_bw_in_hz( mod_params->bw );

    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

uint32_t sx128x_get_gfsk_br_in_bps( sx128x_gfsk_br_bw_t br_bw )
{
    switch( br_bw )
    {
    case SX128X_GFSK_BLE_BR_2_000_BW_2_4:
        return 2000000UL;
    case SX128X_GFSK_BLE_BR_1_600_BW_2_4:
        return 1600000UL;
    case SX128X_GFSK_BLE_BR_1_000_BW_2_4:
        return 1000000UL;
    case SX128X_GFSK_BLE_BR_1_000_BW_1_2:
        return 1000000UL;
    case SX128X_GFSK_BLE_BR_0_800_BW_2_4:
        return 800000UL;
    case SX128X_GFSK_BLE_BR_0_800_BW_1_2:
        return 800000UL;
    case SX128X_GFSK_BLE_BR_0_500_BW_1_2:
        return 500000UL;
    case SX128X_GFSK_BLE_BR_0_500_BW_0_6:
        return 500000UL;
    case SX128X_GFSK_BLE_BR_0_400_BW_1_2:
        return 400000UL;
    case SX128X_GFSK_BLE_BR_0_400_BW_0_6:
        return 400000UL;
    case SX128X_GFSK_BLE_BR_0_250_BW_0_6:
        return 250000UL;
    case SX128X_GFSK_BLE_BR_0_250_BW_0_3:
        return 250000UL;
    case SX128X_GFSK_BLE_BR_0_125_BW_0_3:
        return 125000UL;
    }
    return 0;
}

uint32_t sx128x_get_gfsk_time_on_air_numerator( const sx128x_pkt_params_gfsk_t* pkt_params )
{
    return sx128x_get_gfsk_flrc_preamble_len_in_bits( pkt_params->preamble_len ) +
           sx128x_get_gfsk_header_len_in_bits( pkt_params->header_type ) +
           ( ( sx128x_get_gfsk_sync_word_len_in_bytes( pkt_params->sync_word_len ) + pkt_params->pld_len_in_bytes +
               sx128x_get_gfsk_crc_len_in_bytes( pkt_params->crc_type ) )
             << 3 );
}

uint32_t sx128x_get_gfsk_time_on_air_in_ms( const sx128x_pkt_params_gfsk_t* pkt_params,
                                            const sx128x_mod_params_gfsk_t* mod_params )
{
    uint32_t numerator   = 1000U * sx128x_get_gfsk_time_on_air_numerator( pkt_params );
    uint32_t denominator = sx128x_get_gfsk_br_in_bps( mod_params->br_bw );

    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

uint32_t sx128x_get_flrc_br_in_bps( sx128x_flrc_br_bw_t br_bw )
{
    switch( br_bw )
    {
    case SX128X_FLRC_BR_1_300_BW_1_2:
        return 1300000UL;
    case SX128X_FLRC_BR_1_040_BW_1_2:
        return 1040000UL;
    case SX128X_FLRC_BR_0_650_BW_0_6:
        return 650000UL;
    case SX128X_FLRC_BR_0_520_BW_0_6:
        return 520000UL;
    case SX128X_FLRC_BR_0_325_BW_0_3:
        return 325000UL;
    case SX128X_FLRC_BR_0_260_BW_0_3:
        return 260000UL;
    }
    return 0;
}

uint32_t sx128x_get_flrc_time_on_air_numerator( const sx128x_pkt_params_flrc_t* pkt_params,
                                                const sx128x_mod_params_flrc_t* mod_params )
{
    uint32_t       numerator = 0;
    const uint32_t ceil_den  = sx128x_get_flrc_cr_den( mod_params->cr );

    if( ceil_den != 0 )
    {
        uint32_t ceil_num = 0;
        ceil_num += sx128x_get_flrc_header_len_in_bits( pkt_params->header_type );
        ceil_num += sx128x_get_flrc_tail_len_in_bits( mod_params->cr );
        ceil_num += ( pkt_params->pld_len_in_bytes + sx128x_get_flrc_crc_len_in_bytes( pkt_params->crc_type ) ) << 3;

        // Divide by coding rate, rounding up
        ceil_num <<= 2;

        numerator += ( ( ceil_num + ceil_den - 1 ) / ceil_den );
        numerator += sx128x_get_gfsk_flrc_preamble_len_in_bits( pkt_params->preamble_len ) + 21;
        numerator += sx128x_get_flrc_sync_word_len_in_bytes( pkt_params->sync_word_len ) << 3;
    }

    return numerator;
}

uint32_t sx128x_get_flrc_time_on_air_in_ms( const sx128x_pkt_params_flrc_t* pkt_params,
                                            const sx128x_mod_params_flrc_t* mod_params )
{
    uint32_t numerator   = 1000U * sx128x_get_flrc_time_on_air_numerator( pkt_params, mod_params );
    uint32_t denominator = sx128x_get_flrc_br_in_bps( mod_params->br_bw );

    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

uint32_t sx128x_convert_freq_in_hz_to_pll_step( uint32_t freq_in_hz )
{
    uint32_t steps_int;
    uint32_t steps_frac;

    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    steps_int  = freq_in_hz / SX128X_PLL_STEP_SCALED;
    steps_frac = freq_in_hz - ( steps_int * SX128X_PLL_STEP_SCALED );

    // Apply the scaling factor to retrieve a frequency in Hz, with rounding
    return ( steps_int << SX128X_PLL_STEP_SHIFT_AMOUNT ) +
           ( ( ( steps_frac << SX128X_PLL_STEP_SHIFT_AMOUNT ) + ( SX128X_PLL_STEP_SCALED >> 1 ) ) /
             SX128X_PLL_STEP_SCALED );
}

//
// Registers access
//

sx128x_status_t sx128x_set_gfsk_sync_word( const void* context, const uint8_t sync_word_id, const uint8_t* sync_word,
                                           const uint8_t sync_word_len )
{
    sx128x_status_t status      = SX128X_STATUS_ERROR;
    uint8_t         buf[5]      = { 0 };
    uint16_t        reg_address = 0;

    switch( sync_word_id )
    {
    case 1:
        reg_address = SX128X_REG_GFSK_SYNC_WORD_1;
        break;
    case 2:
        reg_address = SX128X_REG_GFSK_SYNC_WORD_2;
        break;
    case 3:
        reg_address = SX128X_REG_GFSK_SYNC_WORD_3;
        break;
    default:
        return SX128X_STATUS_UNKNOWN_VALUE;
    }
    if( sync_word_len <= 5 )
    {
        memcpy( buf, sync_word, sync_word_len );
        status = sx128x_write_register( context, reg_address, buf, 5 );
    }

    return status;
}

sx128x_status_t sx128x_set_lora_sync_word( const void* context, const uint8_t sync_word )
{
    sx128x_status_t status    = SX128X_STATUS_ERROR;
    uint8_t         buffer[2] = { 0x00 };

    status = sx128x_read_register( context, SX128X_REG_LR_SYNC_WORD, buffer, 2 );

    if( status == SX128X_STATUS_OK )
    {
        buffer[0] = ( buffer[0] & ~0xF0 ) + ( sync_word & 0xF0 );
        buffer[1] = ( buffer[1] & ~0xF0 ) + ( ( sync_word & 0x0F ) << 4 );

        status = sx128x_write_register( context, SX128X_REG_LR_SYNC_WORD, buffer, 2 );
    }

    return status;
}

sx128x_status_t sx128x_set_flrc_sync_word( const void* context, const uint8_t sync_word_id, const uint8_t sync_word[4] )
{
    uint16_t reg_address = 0;

    switch( sync_word_id )
    {
    case 1:
        reg_address = SX128X_REG_FLRC_SYNC_WORD_1;
        break;
    case 2:
        reg_address = SX128X_REG_FLRC_SYNC_WORD_2;
        break;
    case 3:
        reg_address = SX128X_REG_FLRC_SYNC_WORD_3;
        break;
    default:
        return SX128X_STATUS_UNKNOWN_VALUE;
    }

    return sx128x_write_register( context, reg_address, sync_word, 4 );
}

sx128x_status_t sx128x_set_ble_sync_word( const void* context, const uint8_t sync_word[4] )
{
    return sx128x_write_register( context, SX128X_REG_BLE_SYNC_WORD, sync_word, 4 );
}

sx128x_status_t sx128x_set_gfsk_crc_seed( const void* context, uint16_t seed )
{
    const uint8_t buf[] = { ( uint8_t )( seed >> 8 ), ( uint8_t ) seed };

    return sx128x_write_register( context, SX128X_REG_GFSK_CRC_SEED, buf, sizeof( buf ) );
}

sx128x_status_t sx128x_set_flrc_crc_seed( const void* context, uint32_t seed )
{
    const uint8_t buf[] = {
        ( uint8_t )( seed >> 24 ),
        ( uint8_t )( seed >> 16 ),
        ( uint8_t )( seed >> 8 ),
        ( uint8_t ) seed,
    };

    return sx128x_write_register( context, SX128X_REG_FLRC_CRC_SEED, buf, sizeof( buf ) );
}

sx128x_status_t sx128x_set_ble_crc_seed( const void* context, uint32_t seed )
{
    const uint8_t buf[] = {
        ( uint8_t )( seed >> 16 ),
        ( uint8_t )( seed >> 8 ),
        ( uint8_t ) seed,
    };

    return sx128x_write_register( context, SX128X_REG_BLE_CRC_SEED, buf, sizeof( buf ) );
}

sx128x_status_t sx128x_set_gfsk_crc_polynomial( const void* context, uint16_t polynomial )
{
    uint8_t poly[] = { ( uint8_t )( polynomial >> 8 ), ( uint8_t ) polynomial };

    return sx128x_write_register( context, SX128X_REG_GFSK_CRC_POLYNOM, poly, sizeof( poly ) );
}

sx128x_status_t sx128x_set_gfsk_ble_whitening_seed( const void* context, uint8_t seed )
{
    return sx128x_write_register( context, SX128X_REG_GFSK_BLE_WHITENING_SEED, &seed, 1 );
}

sx128x_status_t sx128x_get_lora_pkt_len_mode( const void* context, sx128x_lora_pkt_len_modes_t* header_type )
{
    sx128x_status_t status    = SX128X_STATUS_ERROR;
    uint8_t         reg_value = 0;

    status = sx128x_read_register( context, SX128X_REG_LORA_HEADER_MODE, &reg_value, 1 );
    if( status == SX128X_STATUS_OK )
    {
        // We need only to read the MSBit
        *header_type = ( sx128x_lora_pkt_len_modes_t )( reg_value & SX128X_LORA_RANGING_PKT_IMPLICIT );
    }

    return status;
}

sx128x_status_t sx128x_get_lora_pkt_len( const void* context, uint8_t* pkt_len )
{
    return sx128x_read_register( context, SX128X_REG_LR_PAYLOAD_LENGTH, pkt_len, 1 );
}

sx128x_status_t sx128x_get_lora_rx_pkt_cr( const void* context, sx128x_lora_ranging_cr_t* lora_incoming_cr )
{
    sx128x_status_t status = SX128X_STATUS_ERROR;
    uint8_t         buffer = 0;

    status = sx128x_read_register( context, SX128X_REG_LR_INCOMING_CR, &buffer, 1 );
    if( status == SX128X_STATUS_OK )
    {
        *lora_incoming_cr = ( sx128x_lora_ranging_cr_t )( ( buffer & SX128X_REG_LR_INCOMING_CR_MASK ) >>
                                                          SX128X_REG_LR_INCOMING_CR_POS );
    }

    return status;
}

sx128x_status_t sx128x_get_lora_rx_pkt_crc_present( const void* context, bool* is_lora_incoming_crc_present )
{
    sx128x_status_t status = SX128X_STATUS_ERROR;
    uint8_t         buffer = 0;

    status = sx128x_read_register( context, SX128X_REG_LR_INCOMING_CRC, &buffer, 1 );
    if( status == SX128X_STATUS_OK )
    {
        *is_lora_incoming_crc_present =
            ( ( buffer & SX128X_REG_LR_INCOMING_CRC_MASK ) >> SX128X_REG_LR_INCOMING_CRC_POS ) != 0 ? true : false;
    }

    return status;
}

sx128x_status_t sx128x_set_ranging_master_address( const void* context, uint32_t address )
{
    const uint8_t buf[] = {
        ( uint8_t )( address >> 24 ),
        ( uint8_t )( address >> 16 ),
        ( uint8_t )( address >> 8 ),
        ( uint8_t )( address >> 0 ),
    };

    return sx128x_write_register( context, SX128X_REG_RANGING_MST_REQ_ADDRESS, buf, sizeof( buf ) );
}

sx128x_status_t sx128x_set_ranging_slave_address( const void* context, uint32_t address )
{
    const uint8_t buf[] = {
        ( uint8_t )( address >> 24 ),
        ( uint8_t )( address >> 16 ),
        ( uint8_t )( address >> 8 ),
        ( uint8_t )( address >> 0 ),
    };

    return sx128x_write_register( context, SX128X_REG_RANGING_SLV_REQ_ADDRESS, buf, sizeof( buf ) );
}

sx128x_status_t sx128x_set_ranging_address_len( const void* context, sx128x_ranging_address_len_t len )
{
    const uint8_t buf[] = {
        ( uint8_t )( len << 6 ),
    };

    return sx128x_write_register( context, SX128X_REG_RANGING_ID_LEN_CHECK, buf, sizeof( buf ) );
}

sx128x_status_t sx128x_set_ranging_calibration_value( const void* context, uint16_t calibration )
{
    const uint8_t buf[] = {
        ( uint8_t )( calibration >> 8 ),
        ( uint8_t )( calibration >> 0 ),
    };

    return sx128x_write_register( context, SX128X_REG_RANGING_CALIBRATION, buf, sizeof( buf ) );
}

sx128x_status_t sx128x_ranging_results_freeze( const void* context )
{
    uint8_t tmp;

    sx128x_status_t status = sx128x_read_register( context, SX128X_REG_RANGING_RESULTS_FREEZE, &tmp, 1 );
    if( status != SX128X_STATUS_OK )
    {
        return status;
    }

    tmp |= ( 1 << 1 );
    return sx128x_write_register( context, SX128X_REG_RANGING_RESULTS_FREEZE, &tmp, 1 );
}

sx128x_status_t sx128x_set_ranging_result_type( const void* context, sx128x_ranging_result_type_t type )
{
    uint8_t tmp;

    sx128x_status_t status = sx128x_read_register( context, SX128X_REG_RANGING_RESULTS_CFG, &tmp, 1 );
    if( status != SX128X_STATUS_OK )
    {
        return status;
    }

    tmp = tmp & ~SX128X_REG_RANGING_RESULTS_CFG_MASK;
    tmp = tmp | ( ( uint8_t ) type ) << SX128X_REG_RANGING_RESULTS_CFG_POS;
    return sx128x_write_register( context, SX128X_REG_RANGING_RESULTS_CFG, &tmp, 1 );
}

sx128x_status_t sx128x_get_ranging_result( const void* context, sx128x_ranging_result_type_t type, int32_t* result )
{
    uint8_t tmp[3];

    sx128x_status_t status = sx128x_ranging_results_freeze( context );
    if( status != SX128X_STATUS_OK )
    {
        return status;
    }

    status = sx128x_set_ranging_result_type( context, type );
    if( status != SX128X_STATUS_OK )
    {
        return status;
    }

    status = sx128x_read_register( context, SX128X_REG_RANGING_RESULTS, tmp, sizeof( tmp ) );
    if( status == SX128X_STATUS_OK )
    {
        *result = ( ( int32_t )( ( ( int32_t ) tmp[0] ) << 24 ) | ( ( ( int32_t ) tmp[1] ) << 16 ) |
                    ( ( ( int32_t ) tmp[2] ) << 8 ) ) >>
                  8;
    }

    return status;
}

sx128x_status_t sx128x_get_ranging_result_in_cm( const void* context, sx128x_ranging_result_type_t type,
                                                 sx128x_lora_bw_t bw, int32_t* result )
{
    const int32_t scale = ( 150 * 1000000 / 4096 );

    int32_t         raw_result;
    sx128x_status_t status = sx128x_get_ranging_result( context, type, &raw_result );
    if( status != SX128X_STATUS_OK )
    {
        return status;
    }
    if( raw_result >= ( INT_MAX / scale ) )
    {
        raw_result = ( INT_MAX / scale ) - 1;
    }
    if( raw_result <= ( INT_MIN / scale ) )
    {
        raw_result = ( INT_MIN / scale ) + 1;
    }
    *result = raw_result * scale / ( int32_t )( sx128x_get_lora_bw_in_hz( bw ) / 100 );

    return SX128X_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static inline uint32_t sx128x_get_gfsk_flrc_preamble_len_in_bits( sx128x_gfsk_preamble_len_t preamble_len )
{
    return ( preamble_len >> 2 ) + 4;
}

static inline uint32_t sx128x_get_gfsk_sync_word_len_in_bytes( sx128x_gfsk_sync_word_len_t sync_word_len )
{
    return ( sync_word_len >> 1 ) + 1;
}

static inline uint32_t sx128x_get_gfsk_crc_len_in_bytes( sx128x_gfsk_crc_types_t crc_type )
{
    return crc_type >> 4;
}

static inline uint32_t sx128x_get_flrc_crc_len_in_bytes( sx128x_flrc_crc_types_t crc_len )
{
    if( crc_len == SX128X_FLRC_CRC_OFF )
    {
        return 0;
    }
    return ( crc_len >> 4 ) + 1;
}

static inline uint32_t sx128x_get_gfsk_header_len_in_bits( sx128x_gfsk_flrc_pkt_len_modes_t header_type )
{
    if( header_type == SX128X_GFSK_FLRC_PKT_VAR_LEN )
    {
        return 9;
    }
    else
    {
        return 0;
    }
}

static inline uint32_t sx128x_get_flrc_sync_word_len_in_bytes( sx128x_flrc_sync_word_len_t sync_word_len )
{
    return ( uint32_t ) sync_word_len;
}

static inline uint32_t sx128x_get_flrc_header_len_in_bits( sx128x_gfsk_flrc_pkt_len_modes_t header_type )
{
    if( header_type == SX128X_GFSK_FLRC_PKT_VAR_LEN )
    {
        return 12;
    }
    else
    {
        return 0;
    }
}

static inline uint32_t sx128x_get_flrc_tail_len_in_bits( sx128x_flrc_cr_t cr )
{
    if( ( cr == SX128X_FLRC_CR_1_2 ) || ( cr == SX128X_FLRC_CR_3_4 ) )
    {
        return 6;
    }
    else
    {
        return 0;
    }
}

static inline uint32_t sx128x_get_flrc_cr_den( sx128x_flrc_cr_t cr )
{
    uint32_t ceil_den = 0;

    if( cr == SX128X_FLRC_CR_1_2 )
    {
        ceil_den = 2;
    }
    else if( cr == SX128X_FLRC_CR_3_4 )
    {
        ceil_den = 3;
    }
    else if( cr == SX128X_FLRC_CR_1_1 )
    {
        ceil_den = 4;
    }

    return ceil_den;
}

/* --- EOF ------------------------------------------------------------------ */
