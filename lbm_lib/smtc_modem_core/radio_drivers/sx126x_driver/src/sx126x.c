/**
 * @file      sx126x.c
 *
 * @brief     SX126x radio driver implementation
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
#include <stddef.h>
#include "sx126x.h"
#include "sx126x_hal.h"
#include "sx126x_regs.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * @brief Internal frequency of the radio
 */
#define SX126X_XTAL_FREQ 32000000UL

/**
 * @brief Internal frequency of the radio
 */
#define SX126X_RTC_FREQ_IN_HZ 64000UL

/**
 * @brief Scaling factor used to perform fixed-point operations
 */
#define SX126X_PLL_STEP_SHIFT_AMOUNT ( 14 )

/**
 * @brief PLL step - scaled with SX126X_PLL_STEP_SHIFT_AMOUNT
 */
#define SX126X_PLL_STEP_SCALED ( SX126X_XTAL_FREQ >> ( 25 - SX126X_PLL_STEP_SHIFT_AMOUNT ) )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * Commands Interface
 */
typedef enum sx126x_commands_e
{
    // Operational Modes Functions
    SX126X_SET_SLEEP                  = 0x84,
    SX126X_SET_STANDBY                = 0x80,
    SX126X_SET_FS                     = 0xC1,
    SX126X_SET_TX                     = 0x83,
    SX126X_SET_RX                     = 0x82,
    SX126X_SET_STOP_TIMER_ON_PREAMBLE = 0x9F,
    SX126X_SET_RX_DUTY_CYCLE          = 0x94,
    SX126X_SET_CAD                    = 0xC5,
    SX126X_SET_TX_CONTINUOUS_WAVE     = 0xD1,
    SX126X_SET_TX_INFINITE_PREAMBLE   = 0xD2,
    SX126X_SET_REGULATOR_MODE         = 0x96,
    SX126X_CALIBRATE                  = 0x89,
    SX126X_CALIBRATE_IMAGE            = 0x98,
    SX126X_SET_PA_CFG                 = 0x95,
    SX126X_SET_RX_TX_FALLBACK_MODE    = 0x93,
    // Registers and buffer Access
    SX126X_WRITE_REGISTER = 0x0D,
    SX126X_READ_REGISTER  = 0x1D,
    SX126X_WRITE_BUFFER   = 0x0E,
    SX126X_READ_BUFFER    = 0x1E,
    // DIO and IRQ Control Functions
    SX126X_SET_DIO_IRQ_PARAMS         = 0x08,
    SX126X_GET_IRQ_STATUS             = 0x12,
    SX126X_CLR_IRQ_STATUS             = 0x02,
    SX126X_SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D,
    SX126X_SET_DIO3_AS_TCXO_CTRL      = 0x97,
    // RF Modulation and Packet-Related Functions
    SX126X_SET_RF_FREQUENCY          = 0x86,
    SX126X_SET_PKT_TYPE              = 0x8A,
    SX126X_GET_PKT_TYPE              = 0x11,
    SX126X_SET_TX_PARAMS             = 0x8E,
    SX126X_SET_MODULATION_PARAMS     = 0x8B,
    SX126X_SET_PKT_PARAMS            = 0x8C,
    SX126X_SET_CAD_PARAMS            = 0x88,
    SX126X_SET_BUFFER_BASE_ADDRESS   = 0x8F,
    SX126X_SET_LORA_SYMB_NUM_TIMEOUT = 0xA0,
    // Communication Status Information
    SX126X_GET_STATUS           = 0xC0,
    SX126X_GET_RX_BUFFER_STATUS = 0x13,
    SX126X_GET_PKT_STATUS       = 0x14,
    SX126X_GET_RSSI_INST        = 0x15,
    SX126X_GET_STATS            = 0x10,
    SX126X_RESET_STATS          = 0x00,
    // Miscellaneous
    SX126X_GET_DEVICE_ERRORS = 0x17,
    SX126X_CLR_DEVICE_ERRORS = 0x07,
} sx126x_commands_t;

/**
 * Commands Interface buffer sizes
 */
typedef enum sx126x_commands_size_e
{
    // Operational Modes Functions
    SX126X_SIZE_SET_SLEEP                  = 2,
    SX126X_SIZE_SET_STANDBY                = 2,
    SX126X_SIZE_SET_FS                     = 1,
    SX126X_SIZE_SET_TX                     = 4,
    SX126X_SIZE_SET_RX                     = 4,
    SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE = 2,
    SX126X_SIZE_SET_RX_DUTY_CYCLE          = 7,
    SX126X_SIZE_SET_CAD                    = 1,
    SX126X_SIZE_SET_TX_CONTINUOUS_WAVE     = 1,
    SX126X_SIZE_SET_TX_INFINITE_PREAMBLE   = 1,
    SX126X_SIZE_SET_REGULATOR_MODE         = 2,
    SX126X_SIZE_CALIBRATE                  = 2,
    SX126X_SIZE_CALIBRATE_IMAGE            = 3,
    SX126X_SIZE_SET_PA_CFG                 = 5,
    SX126X_SIZE_SET_RX_TX_FALLBACK_MODE    = 2,
    // Registers and buffer Access
    // Full size: this value plus buffer size
    SX126X_SIZE_WRITE_REGISTER = 3,
    // Full size: this value plus buffer size
    SX126X_SIZE_READ_REGISTER = 4,
    // Full size: this value plus buffer size
    SX126X_SIZE_WRITE_BUFFER = 2,
    // Full size: this value plus buffer size
    SX126X_SIZE_READ_BUFFER = 3,
    // DIO and IRQ Control Functions
    SX126X_SIZE_SET_DIO_IRQ_PARAMS         = 9,
    SX126X_SIZE_GET_IRQ_STATUS             = 2,
    SX126X_SIZE_CLR_IRQ_STATUS             = 3,
    SX126X_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL = 2,
    SX126X_SIZE_SET_DIO3_AS_TCXO_CTRL      = 5,
    // RF Modulation and Packet-Related Functions
    SX126X_SIZE_SET_RF_FREQUENCY           = 5,
    SX126X_SIZE_SET_PKT_TYPE               = 2,
    SX126X_SIZE_GET_PKT_TYPE               = 2,
    SX126X_SIZE_SET_TX_PARAMS              = 3,
    SX126X_SIZE_SET_MODULATION_PARAMS_GFSK = 9,
    SX126X_SIZE_SET_MODULATION_PARAMS_BPSK = 5,
    SX126X_SIZE_SET_MODULATION_PARAMS_LORA = 5,
    SX126X_SIZE_SET_PKT_PARAMS_GFSK        = 10,
    SX126X_SIZE_SET_PKT_PARAMS_BPSK        = 2,
    SX126X_SIZE_SET_PKT_PARAMS_LORA        = 7,
    SX126X_SIZE_SET_CAD_PARAMS             = 8,
    SX126X_SIZE_SET_BUFFER_BASE_ADDRESS    = 3,
    SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT  = 2,
    // Communication Status Information
    SX126X_SIZE_GET_STATUS           = 1,
    SX126X_SIZE_GET_RX_BUFFER_STATUS = 2,
    SX126X_SIZE_GET_PKT_STATUS       = 2,
    SX126X_SIZE_GET_RSSI_INST        = 2,
    SX126X_SIZE_GET_STATS            = 2,
    SX126X_SIZE_RESET_STATS          = 7,
    // Miscellaneous
    SX126X_SIZE_GET_DEVICE_ERRORS = 2,
    SX126X_SIZE_CLR_DEVICE_ERRORS = 3,
    SX126X_SIZE_MAX_BUFFER        = 255,
    SX126X_SIZE_DUMMY_BYTE        = 1,
} sx126x_commands_size_t;

typedef struct
{
    uint32_t bw;
    uint8_t  param;
} gfsk_bw_t;

gfsk_bw_t gfsk_bw[] = {
    { 4800, SX126X_GFSK_BW_4800 },     { 5800, SX126X_GFSK_BW_5800 },     { 7300, SX126X_GFSK_BW_7300 },
    { 9700, SX126X_GFSK_BW_9700 },     { 11700, SX126X_GFSK_BW_11700 },   { 14600, SX126X_GFSK_BW_14600 },
    { 19500, SX126X_GFSK_BW_19500 },   { 23400, SX126X_GFSK_BW_23400 },   { 29300, SX126X_GFSK_BW_29300 },
    { 39000, SX126X_GFSK_BW_39000 },   { 46900, SX126X_GFSK_BW_46900 },   { 58600, SX126X_GFSK_BW_58600 },
    { 78200, SX126X_GFSK_BW_78200 },   { 93800, SX126X_GFSK_BW_93800 },   { 117300, SX126X_GFSK_BW_117300 },
    { 156200, SX126X_GFSK_BW_156200 }, { 187200, SX126X_GFSK_BW_187200 }, { 234300, SX126X_GFSK_BW_234300 },
    { 312000, SX126X_GFSK_BW_312000 }, { 373600, SX126X_GFSK_BW_373600 }, { 467000, SX126X_GFSK_BW_467000 },
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
 * @brief 15.1.2 Workaround
 *
 * @remark Before any packet transmission, bit #2 of SX126X_REG_TX_MODULATION shall be set to:
 * 0 if the LoRa BW = 500 kHz
 * 1 for any other LoRa BW
 * 1 for any (G)FSK configuration
 *
 * @param [in] context Chip implementation context.
 * @param [in] pkt_type The modulation type (G)FSK/LoRa
 * @param [in] bw In case of LoRa modulation the bandwith must be specified
 *
 * @returns Operation status
 */
static sx126x_status_t sx126x_tx_modulation_workaround( const void* context, sx126x_pkt_type_t pkt_type,
                                                        sx126x_lora_bw_t bw );

static inline uint32_t sx126x_get_gfsk_crc_len_in_bytes( sx126x_gfsk_crc_types_t crc_type );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

sx126x_status_t sx126x_set_sleep( const void* context, const sx126x_sleep_cfgs_t cfg )
{
    const uint8_t buf[SX126X_SIZE_SET_SLEEP] = {
        SX126X_SET_SLEEP,
        ( uint8_t ) cfg,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_SLEEP, 0, 0 );
}

sx126x_status_t sx126x_set_standby( const void* context, const sx126x_standby_cfg_t cfg )
{
    const uint8_t buf[SX126X_SIZE_SET_STANDBY] = {
        SX126X_SET_STANDBY,
        ( uint8_t ) cfg,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_STANDBY, 0, 0 );
}

sx126x_status_t sx126x_set_fs( const void* context )
{
    const uint8_t buf[SX126X_SIZE_SET_FS] = {
        SX126X_SET_FS,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_FS, 0, 0 );
}

sx126x_status_t sx126x_set_tx( const void* context, const uint32_t timeout_in_ms )
{
    if( timeout_in_ms > SX126X_MAX_TIMEOUT_IN_MS )
    {
        return SX126X_STATUS_UNKNOWN_VALUE;
    }

    const uint32_t timeout_in_rtc_step = sx126x_convert_timeout_in_ms_to_rtc_step( timeout_in_ms );

    return sx126x_set_tx_with_timeout_in_rtc_step( context, timeout_in_rtc_step );
}

sx126x_status_t sx126x_set_tx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step )
{
    const uint8_t buf[SX126X_SIZE_SET_TX] = {
        SX126X_SET_TX,
        ( uint8_t )( timeout_in_rtc_step >> 16 ),
        ( uint8_t )( timeout_in_rtc_step >> 8 ),
        ( uint8_t )( timeout_in_rtc_step >> 0 ),
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_TX, 0, 0 );
}

sx126x_status_t sx126x_set_rx( const void* context, const uint32_t timeout_in_ms )
{
    if( timeout_in_ms > SX126X_MAX_TIMEOUT_IN_MS )
    {
        return SX126X_STATUS_UNKNOWN_VALUE;
    }

    const uint32_t timeout_in_rtc_step = sx126x_convert_timeout_in_ms_to_rtc_step( timeout_in_ms );

    return sx126x_set_rx_with_timeout_in_rtc_step( context, timeout_in_rtc_step );
}

sx126x_status_t sx126x_set_rx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step )
{
    const uint8_t buf[SX126X_SIZE_SET_RX] = {
        SX126X_SET_RX,
        ( uint8_t )( timeout_in_rtc_step >> 16 ),
        ( uint8_t )( timeout_in_rtc_step >> 8 ),
        ( uint8_t )( timeout_in_rtc_step >> 0 ),
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_RX, 0, 0 );
}

sx126x_status_t sx126x_stop_timer_on_preamble( const void* context, const bool enable )
{
    const uint8_t buf[SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE] = {
        SX126X_SET_STOP_TIMER_ON_PREAMBLE,
        ( enable == true ) ? 1 : 0,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE, 0, 0 );
}

sx126x_status_t sx126x_set_rx_duty_cycle( const void* context, const uint32_t rx_time_in_ms,
                                          const uint32_t sleep_time_in_ms )
{
    const uint32_t rx_time_in_rtc_step    = sx126x_convert_timeout_in_ms_to_rtc_step( rx_time_in_ms );
    const uint32_t sleep_time_in_rtc_step = sx126x_convert_timeout_in_ms_to_rtc_step( sleep_time_in_ms );

    return sx126x_set_rx_duty_cycle_with_timings_in_rtc_step( context, rx_time_in_rtc_step, sleep_time_in_rtc_step );
}

sx126x_status_t sx126x_set_rx_duty_cycle_with_timings_in_rtc_step( const void*    context,
                                                                   const uint32_t rx_time_in_rtc_step,
                                                                   const uint32_t sleep_time_in_rtc_step )
{
    const uint8_t buf[SX126X_SIZE_SET_RX_DUTY_CYCLE] = {
        SX126X_SET_RX_DUTY_CYCLE,
        ( uint8_t )( rx_time_in_rtc_step >> 16 ),
        ( uint8_t )( rx_time_in_rtc_step >> 8 ),
        ( uint8_t )( rx_time_in_rtc_step >> 0 ),
        ( uint8_t )( sleep_time_in_rtc_step >> 16 ),
        ( uint8_t )( sleep_time_in_rtc_step >> 8 ),
        ( uint8_t )( sleep_time_in_rtc_step >> 0 ),
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_RX_DUTY_CYCLE, 0, 0 );
}

sx126x_status_t sx126x_set_cad( const void* context )
{
    const uint8_t buf[SX126X_SIZE_SET_CAD] = {
        SX126X_SET_CAD,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_CAD, 0, 0 );
}

sx126x_status_t sx126x_set_tx_cw( const void* context )
{
    const uint8_t buf[SX126X_SIZE_SET_TX_CONTINUOUS_WAVE] = {
        SX126X_SET_TX_CONTINUOUS_WAVE,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_TX_CONTINUOUS_WAVE, 0, 0 );
}

sx126x_status_t sx126x_set_tx_infinite_preamble( const void* context )
{
    const uint8_t buf[SX126X_SIZE_SET_TX_INFINITE_PREAMBLE] = {
        SX126X_SET_TX_INFINITE_PREAMBLE,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_TX_INFINITE_PREAMBLE, 0, 0 );
}

sx126x_status_t sx126x_set_reg_mode( const void* context, const sx126x_reg_mod_t mode )
{
    const uint8_t buf[SX126X_SIZE_SET_REGULATOR_MODE] = {
        SX126X_SET_REGULATOR_MODE,
        ( uint8_t ) mode,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_REGULATOR_MODE, 0, 0 );
}

sx126x_status_t sx126x_cal( const void* context, const sx126x_cal_mask_t param )
{
    const uint8_t buf[SX126X_SIZE_CALIBRATE] = {
        SX126X_CALIBRATE,
        ( uint8_t ) param,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_CALIBRATE, 0, 0 );
}

sx126x_status_t sx126x_cal_img( const void* context, const uint8_t freq1, const uint8_t freq2 )
{
    const uint8_t buf[SX126X_SIZE_CALIBRATE_IMAGE] = {
        SX126X_CALIBRATE_IMAGE,
        freq1,
        freq2,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_CALIBRATE_IMAGE, 0, 0 );
}

sx126x_status_t sx126x_cal_img_in_mhz( const void* context, const uint16_t freq1_in_mhz, const uint16_t freq2_in_mhz )
{
    // Perform a floor() to get a value for freq1 corresponding to a frequency lower than or equal to freq1_in_mhz
    const uint8_t freq1 = freq1_in_mhz / SX126X_IMAGE_CALIBRATION_STEP_IN_MHZ;

    // Perform a ceil() to get a value for freq2 corresponding to a frequency higher than or equal to freq2_in_mhz
    const uint8_t freq2 =
        ( freq2_in_mhz + SX126X_IMAGE_CALIBRATION_STEP_IN_MHZ - 1 ) / SX126X_IMAGE_CALIBRATION_STEP_IN_MHZ;

    return sx126x_cal_img( context, freq1, freq2 );
}

sx126x_status_t sx126x_set_pa_cfg( const void* context, const sx126x_pa_cfg_params_t* params )
{
    const uint8_t buf[SX126X_SIZE_SET_PA_CFG] = {
        SX126X_SET_PA_CFG, params->pa_duty_cycle, params->hp_max, params->device_sel, params->pa_lut,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PA_CFG, 0, 0 );
}

sx126x_status_t sx126x_set_rx_tx_fallback_mode( const void* context, const sx126x_fallback_modes_t fallback_mode )
{
    const uint8_t buf[SX126X_SIZE_SET_RX_TX_FALLBACK_MODE] = {
        SX126X_SET_RX_TX_FALLBACK_MODE,
        ( uint8_t ) fallback_mode,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_RX_TX_FALLBACK_MODE, 0, 0 );
}

//
// Registers and buffer Access
//

sx126x_status_t sx126x_write_register( const void* context, const uint16_t address, const uint8_t* buffer,
                                       const uint8_t size )
{
    const uint8_t buf[SX126X_SIZE_WRITE_REGISTER] = {
        SX126X_WRITE_REGISTER,
        ( uint8_t )( address >> 8 ),
        ( uint8_t )( address >> 0 ),
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_WRITE_REGISTER, buffer, size );
}

sx126x_status_t sx126x_read_register( const void* context, const uint16_t address, uint8_t* buffer, const uint8_t size )
{
    const uint8_t buf[SX126X_SIZE_READ_REGISTER] = {
        SX126X_READ_REGISTER,
        ( uint8_t )( address >> 8 ),
        ( uint8_t )( address >> 0 ),
        SX126X_NOP,
    };

    return ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_READ_REGISTER, buffer, size );
}

sx126x_status_t sx126x_write_buffer( const void* context, const uint8_t offset, const uint8_t* buffer,
                                     const uint8_t size )
{
    const uint8_t buf[SX126X_SIZE_WRITE_BUFFER] = {
        SX126X_WRITE_BUFFER,
        offset,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_WRITE_BUFFER, buffer, size );
}

sx126x_status_t sx126x_read_buffer( const void* context, const uint8_t offset, uint8_t* buffer, const uint8_t size )
{
    const uint8_t buf[SX126X_SIZE_READ_BUFFER] = {
        SX126X_READ_BUFFER,
        offset,
        SX126X_NOP,
    };

    return ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_READ_BUFFER, buffer, size );
}

//
// DIO and IRQ Control Functions
//
sx126x_status_t sx126x_set_dio_irq_params( const void* context, const uint16_t irq_mask, const uint16_t dio1_mask,
                                           const uint16_t dio2_mask, const uint16_t dio3_mask )
{
    const uint8_t buf[SX126X_SIZE_SET_DIO_IRQ_PARAMS] = {
        SX126X_SET_DIO_IRQ_PARAMS,     ( uint8_t )( irq_mask >> 8 ),  ( uint8_t )( irq_mask >> 0 ),
        ( uint8_t )( dio1_mask >> 8 ), ( uint8_t )( dio1_mask >> 0 ), ( uint8_t )( dio2_mask >> 8 ),
        ( uint8_t )( dio2_mask >> 0 ), ( uint8_t )( dio3_mask >> 8 ), ( uint8_t )( dio3_mask >> 0 ),
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_DIO_IRQ_PARAMS, 0, 0 );
}

sx126x_status_t sx126x_get_irq_status( const void* context, sx126x_irq_mask_t* irq )
{
    const uint8_t buf[SX126X_SIZE_GET_IRQ_STATUS] = {
        SX126X_GET_IRQ_STATUS,
        SX126X_NOP,
    };
    uint8_t irq_local[sizeof( sx126x_irq_mask_t )] = { 0x00 };

    const sx126x_status_t status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_IRQ_STATUS,
                                                                        irq_local, sizeof( sx126x_irq_mask_t ) );

    if( status == SX126X_STATUS_OK )
    {
        *irq = ( ( sx126x_irq_mask_t ) irq_local[0] << 8 ) + ( ( sx126x_irq_mask_t ) irq_local[1] << 0 );
    }

    return status;
}

sx126x_status_t sx126x_clear_irq_status( const void* context, const sx126x_irq_mask_t irq_mask )
{
    const uint8_t buf[SX126X_SIZE_CLR_IRQ_STATUS] = {
        SX126X_CLR_IRQ_STATUS,
        ( uint8_t )( irq_mask >> 8 ),
        ( uint8_t )( irq_mask >> 0 ),
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_CLR_IRQ_STATUS, 0, 0 );
}

sx126x_status_t sx126x_get_and_clear_irq_status( const void* context, sx126x_irq_mask_t* irq )
{
    sx126x_irq_mask_t sx126x_irq_mask = SX126X_IRQ_NONE;

    sx126x_status_t status = sx126x_get_irq_status( context, &sx126x_irq_mask );

    if( ( status == SX126X_STATUS_OK ) && ( sx126x_irq_mask != 0 ) )
    {
        status = sx126x_clear_irq_status( context, sx126x_irq_mask );
    }
    if( ( status == SX126X_STATUS_OK ) && ( irq != NULL ) )
    {
        *irq = sx126x_irq_mask;
    }
    return status;
}

sx126x_status_t sx126x_set_dio2_as_rf_sw_ctrl( const void* context, const bool enable )
{
    const uint8_t buf[SX126X_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL] = {
        SX126X_SET_DIO2_AS_RF_SWITCH_CTRL,
        ( enable == true ) ? 1 : 0,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL, 0, 0 );
}

sx126x_status_t sx126x_set_dio3_as_tcxo_ctrl( const void* context, const sx126x_tcxo_ctrl_voltages_t tcxo_voltage,
                                              const uint32_t timeout )
{
    const uint8_t buf[SX126X_SIZE_SET_DIO3_AS_TCXO_CTRL] = {
        SX126X_SET_DIO3_AS_TCXO_CTRL, ( uint8_t ) tcxo_voltage,    ( uint8_t )( timeout >> 16 ),
        ( uint8_t )( timeout >> 8 ),  ( uint8_t )( timeout >> 0 ),
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_DIO3_AS_TCXO_CTRL, 0, 0 );
}

//
// RF Modulation and Packet-Related Functions
//

sx126x_status_t sx126x_set_rf_freq( const void* context, const uint32_t freq_in_hz )
{
    const uint32_t freq = sx126x_convert_freq_in_hz_to_pll_step( freq_in_hz );

    return sx126x_set_rf_freq_in_pll_steps( context, freq );
}

sx126x_status_t sx126x_set_rf_freq_in_pll_steps( const void* context, const uint32_t freq )
{
    const uint8_t buf[SX126X_SIZE_SET_RF_FREQUENCY] = {
        SX126X_SET_RF_FREQUENCY,  ( uint8_t )( freq >> 24 ), ( uint8_t )( freq >> 16 ),
        ( uint8_t )( freq >> 8 ), ( uint8_t )( freq >> 0 ),
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_RF_FREQUENCY, 0, 0 );
}

sx126x_status_t sx126x_set_pkt_type( const void* context, const sx126x_pkt_type_t pkt_type )
{
    const uint8_t buf[SX126X_SIZE_SET_PKT_TYPE] = {
        SX126X_SET_PKT_TYPE,
        ( uint8_t ) pkt_type,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PKT_TYPE, 0, 0 );
}

sx126x_status_t sx126x_get_pkt_type( const void* context, sx126x_pkt_type_t* pkt_type )
{
    const uint8_t buf[SX126X_SIZE_GET_PKT_TYPE] = {
        SX126X_GET_PKT_TYPE,
        SX126X_NOP,
    };

    return ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_PKT_TYPE, ( uint8_t* ) pkt_type, 1 );
}

sx126x_status_t sx126x_set_tx_params( const void* context, const int8_t pwr_in_dbm, const sx126x_ramp_time_t ramp_time )
{
    const uint8_t buf[SX126X_SIZE_SET_TX_PARAMS] = {
        SX126X_SET_TX_PARAMS,
        ( uint8_t ) pwr_in_dbm,
        ( uint8_t ) ramp_time,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_TX_PARAMS, 0, 0 );
}

sx126x_status_t sx126x_set_gfsk_mod_params( const void* context, const sx126x_mod_params_gfsk_t* params )
{
    const uint32_t bitrate = ( uint32_t )( 32 * SX126X_XTAL_FREQ / params->br_in_bps );
    const uint32_t fdev    = sx126x_convert_freq_in_hz_to_pll_step( params->fdev_in_hz );
    const uint8_t  buf[SX126X_SIZE_SET_MODULATION_PARAMS_GFSK] = {
        SX126X_SET_MODULATION_PARAMS, ( uint8_t )( bitrate >> 16 ),       ( uint8_t )( bitrate >> 8 ),
        ( uint8_t )( bitrate >> 0 ),  ( uint8_t )( params->pulse_shape ), params->bw_dsb_param,
        ( uint8_t )( fdev >> 16 ),    ( uint8_t )( fdev >> 8 ),           ( uint8_t )( fdev >> 0 ),
    };

    sx126x_status_t status =
        ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_MODULATION_PARAMS_GFSK, 0, 0 );

    if( status == SX126X_STATUS_OK )
    {
        // WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_SX1261-2_V1.2 datasheet chapter 15.1
        status = sx126x_tx_modulation_workaround( context, SX126X_PKT_TYPE_GFSK, ( sx126x_lora_bw_t ) 0 );
        // WORKAROUND END
    }
    return status;
}

sx126x_status_t sx126x_set_bpsk_mod_params( const void* context, const sx126x_mod_params_bpsk_t* params )
{
    const uint32_t bitrate = ( uint32_t )( 32 * SX126X_XTAL_FREQ / params->br_in_bps );

    const uint8_t buf[SX126X_SIZE_SET_MODULATION_PARAMS_BPSK] = {
        SX126X_SET_MODULATION_PARAMS, ( uint8_t )( bitrate >> 16 ),       ( uint8_t )( bitrate >> 8 ),
        ( uint8_t )( bitrate >> 0 ),  ( uint8_t )( params->pulse_shape ),
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_MODULATION_PARAMS_BPSK, 0, 0 );
}

sx126x_status_t sx126x_set_lora_mod_params( const void* context, const sx126x_mod_params_lora_t* params )
{
    const uint8_t buf[SX126X_SIZE_SET_MODULATION_PARAMS_LORA] = {
        SX126X_SET_MODULATION_PARAMS, ( uint8_t )( params->sf ), ( uint8_t )( params->bw ),
        ( uint8_t )( params->cr ),    params->ldro & 0x01,
    };

    sx126x_status_t status =
        ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_MODULATION_PARAMS_LORA, 0, 0 );

    if( status == SX126X_STATUS_OK )
    {
        // WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see datasheet DS_SX1261-2_V1.2 §15.1
        status = sx126x_tx_modulation_workaround( context, SX126X_PKT_TYPE_LORA, params->bw );
        // WORKAROUND END
    }

    return status;
}

sx126x_status_t sx126x_set_gfsk_pkt_params( const void* context, const sx126x_pkt_params_gfsk_t* params )
{
    const uint8_t buf[SX126X_SIZE_SET_PKT_PARAMS_GFSK] = {
        SX126X_SET_PKT_PARAMS,
        ( uint8_t )( params->preamble_len_in_bits >> 8 ),
        ( uint8_t )( params->preamble_len_in_bits >> 0 ),
        ( uint8_t )( params->preamble_detector ),
        params->sync_word_len_in_bits,
        ( uint8_t )( params->address_filtering ),
        ( uint8_t )( params->header_type ),
        params->pld_len_in_bytes,
        ( uint8_t )( params->crc_type ),
        ( uint8_t )( params->dc_free ),
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PKT_PARAMS_GFSK, 0, 0 );
}

sx126x_status_t sx126x_set_bpsk_pkt_params( const void* context, const sx126x_pkt_params_bpsk_t* params )
{
    const uint8_t buf[SX126X_SIZE_SET_PKT_PARAMS_BPSK] = {
        SX126X_SET_PKT_PARAMS,
        params->pld_len_in_bytes,
    };

    sx126x_status_t status =
        ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PKT_PARAMS_BPSK, 0, 0 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    const uint8_t buf2[] = {
        ( uint8_t )( params->ramp_up_delay >> 8 ),   ( uint8_t )( params->ramp_up_delay >> 0 ),
        ( uint8_t )( params->ramp_down_delay >> 8 ), ( uint8_t )( params->ramp_down_delay >> 0 ),
        ( uint8_t )( params->pld_len_in_bits >> 8 ), ( uint8_t )( params->pld_len_in_bits >> 0 ),
    };

    return sx126x_write_register( context, 0x00F0, buf2, sizeof( buf2 ) );
}

sx126x_status_t sx126x_set_lora_pkt_params( const void* context, const sx126x_pkt_params_lora_t* params )
{
    const uint8_t buf[SX126X_SIZE_SET_PKT_PARAMS_LORA] = {
        SX126X_SET_PKT_PARAMS,
        ( uint8_t )( params->preamble_len_in_symb >> 8 ),
        ( uint8_t )( params->preamble_len_in_symb >> 0 ),
        ( uint8_t )( params->header_type ),
        params->pld_len_in_bytes,
        ( uint8_t )( params->crc_is_on ? 1 : 0 ),
        ( uint8_t )( params->invert_iq_is_on ? 1 : 0 ),
    };

    sx126x_status_t status =
        ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PKT_PARAMS_LORA, 0, 0 );

    // WORKAROUND - Optimizing the Inverted IQ Operation, see datasheet DS_SX1261-2_V1.2 §15.4
    if( status == SX126X_STATUS_OK )
    {
        uint8_t reg_value = 0;

        status = sx126x_read_register( context, SX126X_REG_IQ_POLARITY, &reg_value, 1 );
        if( status == SX126X_STATUS_OK )
        {
            if( params->invert_iq_is_on == true )
            {
                reg_value &= ~( 1 << 2 );  // Bit 2 set to 0 when using inverted IQ polarity
            }
            else
            {
                reg_value |= ( 1 << 2 );  // Bit 2 set to 1 when using standard IQ polarity
            }
            status = sx126x_write_register( context, SX126X_REG_IQ_POLARITY, &reg_value, 1 );
        }
    }
    // WORKAROUND END

    return status;
}

sx126x_status_t sx126x_set_gfsk_pkt_address( const void* context, const uint8_t node_address,
                                             const uint8_t broadcast_address )
{
    const uint8_t addresses[2] = { node_address, broadcast_address };

    return sx126x_write_register( context, SX126X_REG_GFSK_NODE_ADDRESS, addresses, 2 );
}

sx126x_status_t sx126x_set_cad_params( const void* context, const sx126x_cad_params_t* params )
{
    const uint8_t buf[SX126X_SIZE_SET_CAD_PARAMS] = {
        SX126X_SET_CAD_PARAMS,
        ( uint8_t ) params->cad_symb_nb,
        params->cad_detect_peak,
        params->cad_detect_min,
        ( uint8_t ) params->cad_exit_mode,
        ( uint8_t )( params->cad_timeout >> 16 ),
        ( uint8_t )( params->cad_timeout >> 8 ),
        ( uint8_t )( params->cad_timeout >> 0 ),
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_CAD_PARAMS, 0, 0 );
}

sx126x_status_t sx126x_set_buffer_base_address( const void* context, const uint8_t tx_base_address,
                                                const uint8_t rx_base_address )
{
    const uint8_t buf[SX126X_SIZE_SET_BUFFER_BASE_ADDRESS] = {
        SX126X_SET_BUFFER_BASE_ADDRESS,
        tx_base_address,
        rx_base_address,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_BUFFER_BASE_ADDRESS, 0, 0 );
}

sx126x_status_t sx126x_set_lora_symb_nb_timeout( const void* context, const uint8_t nb_of_symbs )
{
    uint8_t exp = 0;
    uint8_t mant =
        ( ( ( nb_of_symbs > SX126X_MAX_LORA_SYMB_NUM_TIMEOUT ) ? SX126X_MAX_LORA_SYMB_NUM_TIMEOUT : nb_of_symbs ) +
          1 ) >>
        1;

    while( mant > 31 )
    {
        mant = ( mant + 3 ) >> 2;
        exp++;
    }

    const uint8_t buf[SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT] = {
        SX126X_SET_LORA_SYMB_NUM_TIMEOUT,
        mant << ( 2 * exp + 1 ),
    };

    sx126x_status_t status =
        ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT, 0, 0 );

    if( ( status == SX126X_STATUS_OK ) && ( nb_of_symbs > 0 ) )
    {
        uint8_t reg = exp + ( mant << 3 );
        status      = sx126x_write_register( context, SX126X_REG_LR_SYNCH_TIMEOUT, &reg, 1 );
    }

    return status;
}

//
// Communication Status Information
//

sx126x_status_t sx126x_get_status( const void* context, sx126x_chip_status_t* radio_status )
{
    const uint8_t buf[SX126X_SIZE_GET_STATUS] = {
        SX126X_GET_STATUS,
    };
    uint8_t status_local = 0;

    const sx126x_status_t status =
        ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_STATUS, &status_local, 1 );

    if( status == SX126X_STATUS_OK )
    {
        radio_status->cmd_status =
            ( sx126x_cmd_status_t )( ( status_local & SX126X_CMD_STATUS_MASK ) >> SX126X_CMD_STATUS_POS );
        radio_status->chip_mode =
            ( sx126x_chip_modes_t )( ( status_local & SX126X_CHIP_MODES_MASK ) >> SX126X_CHIP_MODES_POS );
    }

    return status;
}

sx126x_status_t sx126x_get_rx_buffer_status( const void* context, sx126x_rx_buffer_status_t* rx_buffer_status )
{
    const uint8_t buf[SX126X_SIZE_GET_RX_BUFFER_STATUS] = {
        SX126X_GET_RX_BUFFER_STATUS,
        SX126X_NOP,
    };
    uint8_t status_local[sizeof( sx126x_rx_buffer_status_t )] = { 0x00 };

    const sx126x_status_t status = ( sx126x_status_t ) sx126x_hal_read(
        context, buf, SX126X_SIZE_GET_RX_BUFFER_STATUS, status_local, sizeof( sx126x_rx_buffer_status_t ) );

    if( status == SX126X_STATUS_OK )
    {
        rx_buffer_status->pld_len_in_bytes     = status_local[0];
        rx_buffer_status->buffer_start_pointer = status_local[1];
    }

    return status;
}

sx126x_status_t sx126x_get_gfsk_pkt_status( const void* context, sx126x_pkt_status_gfsk_t* pkt_status )
{
    const uint8_t buf[SX126X_SIZE_GET_PKT_STATUS] = {
        SX126X_GET_PKT_STATUS,
        SX126X_NOP,
    };
    uint8_t pkt_status_local[3] = { 0x00 };

    const sx126x_status_t status =
        ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_PKT_STATUS, pkt_status_local, 3 );

    if( status == SX126X_STATUS_OK )
    {
        pkt_status->rx_status.pkt_sent =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_PKT_SENT_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.pkt_received =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_PKT_RECEIVED_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.abort_error =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_ABORT_ERROR_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.length_error =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_LENGTH_ERROR_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.crc_error =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_CRC_ERROR_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.adrs_error =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_ADRS_ERROR_MASK ) != 0 ) ? true : false;

        pkt_status->rssi_sync = ( int8_t )( -pkt_status_local[1] >> 1 );
        pkt_status->rssi_avg  = ( int8_t )( -pkt_status_local[2] >> 1 );
    }

    return status;
}

sx126x_status_t sx126x_get_lora_pkt_status( const void* context, sx126x_pkt_status_lora_t* pkt_status )
{
    const uint8_t buf[SX126X_SIZE_GET_PKT_STATUS] = {
        SX126X_GET_PKT_STATUS,
        SX126X_NOP,
    };
    uint8_t pkt_status_local[sizeof( sx126x_pkt_status_lora_t )] = { 0x00 };

    const sx126x_status_t status = ( sx126x_status_t ) sx126x_hal_read(
        context, buf, SX126X_SIZE_GET_PKT_STATUS, pkt_status_local, sizeof( sx126x_pkt_status_lora_t ) );

    if( status == SX126X_STATUS_OK )
    {
        pkt_status->rssi_pkt_in_dbm        = ( int8_t )( -pkt_status_local[0] >> 1 );
        pkt_status->snr_pkt_in_db          = ( ( ( int8_t ) pkt_status_local[1] ) + 2 ) >> 2;
        pkt_status->signal_rssi_pkt_in_dbm = ( int8_t )( -pkt_status_local[2] >> 1 );
    }

    return status;
}

sx126x_status_t sx126x_get_rssi_inst( const void* context, int16_t* rssi_in_dbm )
{
    const uint8_t buf[SX126X_SIZE_GET_RSSI_INST] = {
        SX126X_GET_RSSI_INST,
        SX126X_NOP,
    };
    uint8_t rssi_local = 0x00;

    const sx126x_status_t status =
        ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_RSSI_INST, &rssi_local, 1 );

    if( status == SX126X_STATUS_OK )
    {
        *rssi_in_dbm = ( int8_t )( -rssi_local >> 1 );
    }

    return status;
}

sx126x_status_t sx126x_get_gfsk_stats( const void* context, sx126x_stats_gfsk_t* stats )
{
    const uint8_t buf[SX126X_SIZE_GET_STATS] = {
        SX126X_GET_STATS,
        SX126X_NOP,
    };
    uint8_t stats_local[sizeof( sx126x_stats_gfsk_t )] = { 0 };

    const sx126x_status_t status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_STATS,
                                                                        stats_local, sizeof( sx126x_stats_gfsk_t ) );

    if( status == SX126X_STATUS_OK )
    {
        stats->nb_pkt_received  = ( ( uint16_t ) stats_local[0] << 8 ) + ( uint16_t ) stats_local[1];
        stats->nb_pkt_crc_error = ( ( uint16_t ) stats_local[2] << 8 ) + ( uint16_t ) stats_local[3];
        stats->nb_pkt_len_error = ( ( uint16_t ) stats_local[4] << 8 ) + ( uint16_t ) stats_local[5];
    }

    return status;
}

sx126x_status_t sx126x_get_lora_stats( const void* context, sx126x_stats_lora_t* stats )
{
    const uint8_t buf[SX126X_SIZE_GET_STATS] = {
        SX126X_GET_STATS,
        SX126X_NOP,
    };
    uint8_t stats_local[sizeof( sx126x_stats_lora_t )] = { 0 };

    const sx126x_status_t status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_STATS,
                                                                        stats_local, sizeof( sx126x_stats_lora_t ) );

    if( status == SX126X_STATUS_OK )
    {
        stats->nb_pkt_received     = ( ( uint16_t ) stats_local[0] << 8 ) + ( uint16_t ) stats_local[1];
        stats->nb_pkt_crc_error    = ( ( uint16_t ) stats_local[2] << 8 ) + ( uint16_t ) stats_local[3];
        stats->nb_pkt_header_error = ( ( uint16_t ) stats_local[4] << 8 ) + ( uint16_t ) stats_local[5];
    }
    return status;
}

sx126x_status_t sx126x_reset_stats( const void* context )
{
    const uint8_t buf[SX126X_SIZE_RESET_STATS] = {
        SX126X_RESET_STATS, SX126X_NOP, SX126X_NOP, SX126X_NOP, SX126X_NOP, SX126X_NOP, SX126X_NOP,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_RESET_STATS, 0, 0 );
}

//
// Miscellaneous
//

sx126x_status_t sx126x_reset( const void* context )
{
    return ( sx126x_status_t ) sx126x_hal_reset( context );
}

sx126x_status_t sx126x_wakeup( const void* context )
{
    return ( sx126x_status_t ) sx126x_hal_wakeup( context );
}

sx126x_status_t sx126x_get_device_errors( const void* context, sx126x_errors_mask_t* errors )
{
    const uint8_t buf[SX126X_SIZE_GET_DEVICE_ERRORS] = {
        SX126X_GET_DEVICE_ERRORS,
        SX126X_NOP,
    };
    uint8_t errors_local[sizeof( sx126x_errors_mask_t )] = { 0x00 };

    const sx126x_status_t status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_DEVICE_ERRORS,
                                                                        errors_local, sizeof( sx126x_errors_mask_t ) );

    if( status == SX126X_STATUS_OK )
    {
        *errors = ( ( sx126x_errors_mask_t ) errors_local[0] << 8 ) + ( ( sx126x_errors_mask_t ) errors_local[1] << 0 );
    }

    return status;
}

sx126x_status_t sx126x_clear_device_errors( const void* context )
{
    const uint8_t buf[SX126X_SIZE_CLR_DEVICE_ERRORS] = {
        SX126X_CLR_DEVICE_ERRORS,
        SX126X_NOP,
        SX126X_NOP,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_CLR_DEVICE_ERRORS, 0, 0 );
}

sx126x_status_t sx126x_get_gfsk_bw_param( const uint32_t bw, uint8_t* param )
{
    sx126x_status_t status = SX126X_STATUS_ERROR;

    if( bw != 0 )
    {
        status = SX126X_STATUS_UNKNOWN_VALUE;
        for( uint8_t i = 0; i < ( sizeof( gfsk_bw ) / sizeof( gfsk_bw_t ) ); i++ )
        {
            if( bw <= gfsk_bw[i].bw )
            {
                *param = gfsk_bw[i].param;
                status = SX126X_STATUS_OK;
                break;
            }
        }
    }

    return status;
}

uint32_t sx126x_get_lora_bw_in_hz( sx126x_lora_bw_t bw )
{
    uint32_t bw_in_hz = 0;

    switch( bw )
    {
    case SX126X_LORA_BW_007:
        bw_in_hz = 7812UL;
        break;
    case SX126X_LORA_BW_010:
        bw_in_hz = 10417UL;
        break;
    case SX126X_LORA_BW_015:
        bw_in_hz = 15625UL;
        break;
    case SX126X_LORA_BW_020:
        bw_in_hz = 20833UL;
        break;
    case SX126X_LORA_BW_031:
        bw_in_hz = 31250UL;
        break;
    case SX126X_LORA_BW_041:
        bw_in_hz = 41667UL;
        break;
    case SX126X_LORA_BW_062:
        bw_in_hz = 62500UL;
        break;
    case SX126X_LORA_BW_125:
        bw_in_hz = 125000UL;
        break;
    case SX126X_LORA_BW_250:
        bw_in_hz = 250000UL;
        break;
    case SX126X_LORA_BW_500:
        bw_in_hz = 500000UL;
        break;
    }

    return bw_in_hz;
}

uint32_t sx126x_get_lora_time_on_air_numerator( const sx126x_pkt_params_lora_t* pkt_p,
                                                const sx126x_mod_params_lora_t* mod_p )
{
    const int32_t pld_len_in_bytes = pkt_p->pld_len_in_bytes;
    const int32_t sf               = mod_p->sf;
    const bool    pld_is_fix       = pkt_p->header_type == SX126X_LORA_PKT_IMPLICIT;
    const int32_t cr_denom         = mod_p->cr + 4;

    int32_t ceil_denominator;
    int32_t ceil_numerator =
        ( pld_len_in_bytes << 3 ) + ( pkt_p->crc_is_on ? 16 : 0 ) - ( 4 * sf ) + ( pld_is_fix ? 0 : 20 );

    if( sf <= 6 )
    {
        ceil_denominator = 4 * sf;
    }
    else
    {
        ceil_numerator += 8;

        if( mod_p->ldro )
        {
            ceil_denominator = 4 * ( sf - 2 );
        }
        else
        {
            ceil_denominator = 4 * sf;
        }
    }

    if( ceil_numerator < 0 )
    {
        ceil_numerator = 0;
    }

    // Perform integral ceil()
    int32_t intermed =
        ( ( ceil_numerator + ceil_denominator - 1 ) / ceil_denominator ) * cr_denom + pkt_p->preamble_len_in_symb + 12;

    if( sf <= 6 )
    {
        intermed += 2;
    }

    return ( uint32_t )( ( 4 * intermed + 1 ) * ( 1 << ( sf - 2 ) ) );
}

uint32_t sx126x_get_lora_time_on_air_in_ms( const sx126x_pkt_params_lora_t* pkt_p,
                                            const sx126x_mod_params_lora_t* mod_p )
{
    uint32_t numerator   = 1000U * sx126x_get_lora_time_on_air_numerator( pkt_p, mod_p );
    uint32_t denominator = sx126x_get_lora_bw_in_hz( mod_p->bw );
    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

uint32_t sx126x_get_gfsk_time_on_air_numerator( const sx126x_pkt_params_gfsk_t* pkt_p )
{
    return pkt_p->preamble_len_in_bits + ( pkt_p->header_type == SX126X_GFSK_PKT_VAR_LEN ? 8 : 0 ) +
           pkt_p->sync_word_len_in_bits +
           ( ( pkt_p->pld_len_in_bytes + ( pkt_p->address_filtering == SX126X_GFSK_ADDRESS_FILTERING_DISABLE ? 0 : 1 ) +
               sx126x_get_gfsk_crc_len_in_bytes( pkt_p->crc_type ) )
             << 3 );
}

uint32_t sx126x_get_gfsk_time_on_air_in_ms( const sx126x_pkt_params_gfsk_t* pkt_p,
                                            const sx126x_mod_params_gfsk_t* mod_p )
{
    uint32_t numerator   = 1000U * sx126x_get_gfsk_time_on_air_numerator( pkt_p );
    uint32_t denominator = mod_p->br_in_bps;

    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

sx126x_status_t sx126x_get_random_numbers( const void* context, uint32_t* numbers, unsigned int n )
{
    sx126x_status_t status;

    uint8_t tmp_ana_lna   = 0x00;
    uint8_t tmp_ana_mixer = 0x00;
    uint8_t tmp           = 0x00;

    // Configure for random number generation
    status = sx126x_read_register( context, SX126X_REG_ANA_LNA, &tmp_ana_lna, 1 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }
    tmp    = tmp_ana_lna & ~( 1 << 0 );
    status = sx126x_write_register( context, SX126X_REG_ANA_LNA, &tmp, 1 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    status = sx126x_read_register( context, SX126X_REG_ANA_MIXER, &tmp_ana_mixer, 1 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }
    tmp    = tmp_ana_mixer & ~( 1 << 7 );
    status = sx126x_write_register( context, SX126X_REG_ANA_MIXER, &tmp, 1 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    // Start RX continuous
    status = sx126x_set_rx_with_timeout_in_rtc_step( context, SX126X_RX_CONTINUOUS );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    // Store values
    for( unsigned int i = 0; i < n; i++ )
    {
        status = sx126x_read_register( context, SX126X_REG_RNGBASEADDRESS, ( uint8_t* ) &numbers[i], 4 );
        if( status != SX126X_STATUS_OK )
        {
            return status;
        }
    }

    status = sx126x_set_standby( context, SX126X_STANDBY_CFG_RC );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    // Restore registers
    status = sx126x_write_register( context, SX126X_REG_ANA_LNA, &tmp_ana_lna, 1 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }
    status = sx126x_write_register( context, SX126X_REG_ANA_MIXER, &tmp_ana_mixer, 1 );

    return status;
}

uint32_t sx126x_convert_freq_in_hz_to_pll_step( uint32_t freq_in_hz )
{
    uint32_t steps_int;
    uint32_t steps_frac;

    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    steps_int  = freq_in_hz / SX126X_PLL_STEP_SCALED;
    steps_frac = freq_in_hz - ( steps_int * SX126X_PLL_STEP_SCALED );

    // Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
    return ( steps_int << SX126X_PLL_STEP_SHIFT_AMOUNT ) +
           ( ( ( steps_frac << SX126X_PLL_STEP_SHIFT_AMOUNT ) + ( SX126X_PLL_STEP_SCALED >> 1 ) ) /
             SX126X_PLL_STEP_SCALED );
}

uint32_t sx126x_convert_timeout_in_ms_to_rtc_step( uint32_t timeout_in_ms )
{
    return ( uint32_t )( timeout_in_ms * ( SX126X_RTC_FREQ_IN_HZ / 1000 ) );
}

sx126x_status_t sx126x_handle_rx_done( const void* context )
{
    return sx126x_stop_rtc( context );
}

//
// Registers access
//

sx126x_status_t sx126x_cfg_rx_boosted( const void* context, const bool state )
{
    if( state == true )
    {
        return sx126x_write_register( context, SX126X_REG_RXGAIN, ( const uint8_t[] ){ 0x96 }, 1 );
    }
    else
    {
        return sx126x_write_register( context, SX126X_REG_RXGAIN, ( const uint8_t[] ){ 0x94 }, 1 );
    }
}

sx126x_status_t sx126x_set_gfsk_sync_word( const void* context, const uint8_t* sync_word, const uint8_t sync_word_len )
{
    sx126x_status_t status = SX126X_STATUS_ERROR;

    if( sync_word_len <= 8 )
    {
        uint8_t buf[8] = { 0 };

        for( int i = 0; i < sync_word_len; i++ )
        {
            buf[i] = sync_word[i];
        }
        status = sx126x_write_register( context, SX126X_REG_SYNCWORDBASEADDRESS, buf, 8 );
    }

    return status;
}

sx126x_status_t sx126x_set_lora_sync_word( const void* context, const uint8_t sync_word )
{
    uint8_t buffer[2] = { 0x00 };

    sx126x_status_t status = sx126x_read_register( context, SX126X_REG_LR_SYNCWORD, buffer, 2 );

    if( status == SX126X_STATUS_OK )
    {
        buffer[0] = ( buffer[0] & ~0xF0 ) + ( sync_word & 0xF0 );
        buffer[1] = ( buffer[1] & ~0xF0 ) + ( ( sync_word & 0x0F ) << 4 );

        status = sx126x_write_register( context, SX126X_REG_LR_SYNCWORD, buffer, 2 );
    }

    return status;
}

sx126x_status_t sx126x_set_gfsk_crc_seed( const void* context, uint16_t seed )
{
    uint8_t s[] = { ( uint8_t )( seed >> 8 ), ( uint8_t ) seed };

    return sx126x_write_register( context, SX126X_REG_CRCSEEDBASEADDRESS, s, sizeof( s ) );
}

sx126x_status_t sx126x_set_gfsk_crc_polynomial( const void* context, const uint16_t polynomial )
{
    uint8_t poly[] = { ( uint8_t )( polynomial >> 8 ), ( uint8_t ) polynomial };

    return sx126x_write_register( context, SX126X_REG_CRCPOLYBASEADDRESS, poly, sizeof( poly ) );
}

sx126x_status_t sx126x_set_gfsk_whitening_seed( const void* context, const uint16_t seed )
{
    uint8_t reg_value = 0;

    // The SX126X_REG_WHITSEEDBASEADDRESS @ref LSBit is used for the seed value. The 7 MSBits must not be modified.
    // Thus, we first need to read the current value and then change the LSB according to the provided seed @ref value.
    sx126x_status_t status = sx126x_read_register( context, SX126X_REG_WHITSEEDBASEADDRESS, &reg_value, 1 );
    if( status == SX126X_STATUS_OK )
    {
        reg_value = ( reg_value & 0xFE ) | ( ( uint8_t )( seed >> 8 ) & 0x01 );
        status    = sx126x_write_register( context, SX126X_REG_WHITSEEDBASEADDRESS, &reg_value, 1 );
        if( status == SX126X_STATUS_OK )
        {
            reg_value = ( uint8_t ) seed;
            status    = sx126x_write_register( context, SX126X_REG_WHITSEEDBASEADDRESS + 1, &reg_value, 1 );
        }
    }

    return status;
}

sx126x_status_t sx126x_cfg_tx_clamp( const void* context )
{
    uint8_t reg_value = 0x00;

    sx126x_status_t status = sx126x_read_register( context, SX126X_REG_TX_CLAMP_CFG, &reg_value, 1 );

    if( status == SX126X_STATUS_OK )
    {
        reg_value |= SX126X_REG_TX_CLAMP_CFG_MASK;
        status = sx126x_write_register( context, SX126X_REG_TX_CLAMP_CFG, &reg_value, 1 );
    }

    return status;
}

sx126x_status_t sx126x_stop_rtc( const void* context )
{
    uint8_t reg_value = 0;

    sx126x_status_t status = sx126x_write_register( context, SX126X_REG_RTC_CTRL, &reg_value, 1 );

    if( status == SX126X_STATUS_OK )
    {
        status = sx126x_read_register( context, SX126X_REG_EVT_CLR, &reg_value, 1 );

        if( status == SX126X_STATUS_OK )
        {
            reg_value |= SX126X_REG_EVT_CLR_TIMEOUT_MASK;
            status = sx126x_write_register( context, SX126X_REG_EVT_CLR, &reg_value, 1 );
        }
    }

    return status;
}

sx126x_status_t sx126x_set_ocp_value( const void* context, const uint8_t ocp_in_step_of_2_5_ma )
{
    return ( sx126x_status_t ) sx126x_write_register( context, SX126X_REG_OCP, &ocp_in_step_of_2_5_ma, 1 );
}

sx126x_status_t sx126x_set_trimming_capacitor_values( const void* context, const uint8_t trimming_cap_xta,
                                                      const uint8_t trimming_cap_xtb )
{
    uint8_t trimming_capacitor_values[2] = { trimming_cap_xta, trimming_cap_xtb };

    return ( sx126x_status_t ) sx126x_write_register( context, SX126X_REG_XTATRIM, trimming_capacitor_values, 2 );
}

sx126x_status_t sx126x_add_registers_to_retention_list( const void* context, const uint16_t* register_addr,
                                                        uint8_t register_nb )
{
    uint8_t buffer[9] = {0};

    sx126x_status_t status = sx126x_read_register( context, SX126X_REG_RETENTION_LIST_BASE_ADDRESS, buffer, 9 );

    if( status == SX126X_STATUS_OK )
    {
        const uint8_t initial_nb_of_registers = buffer[0];
        uint8_t*      register_list           = &buffer[1];

        for( uint8_t index = 0; index < register_nb; index++ )
        {
            bool register_has_to_be_added = true;

            // Check if the current register is already added to the list
            for( uint8_t i = 0; i < buffer[0]; i++ )
            {
                if( register_addr[index] == ( ( uint16_t ) register_list[2 * i] << 8 ) + register_list[2 * i + 1] )
                {
                    register_has_to_be_added = false;
                    break;
                }
            }

            if( register_has_to_be_added == true )
            {
                if( buffer[0] < SX126X_MAX_NB_REG_IN_RETENTION )
                {
                    register_list[2 * buffer[0]]     = ( uint8_t )( register_addr[index] >> 8 );
                    register_list[2 * buffer[0] + 1] = ( uint8_t )( register_addr[index] >> 0 );
                    buffer[0] += 1;
                }
                else
                {
                    return SX126X_STATUS_ERROR;
                }
            }
        }

        if( buffer[0] != initial_nb_of_registers )
        {
            status = sx126x_write_register( context, SX126X_REG_RETENTION_LIST_BASE_ADDRESS, buffer, 9 );
        }
    }

    return status;
}

sx126x_status_t sx126x_init_retention_list( const void* context )
{
    const uint16_t list_of_registers[3] = { SX126X_REG_RXGAIN, SX126X_REG_TX_MODULATION, SX126X_REG_IQ_POLARITY };

    return sx126x_add_registers_to_retention_list( context, list_of_registers,
                                                   sizeof( list_of_registers ) / sizeof( list_of_registers[0] ) );
}

sx126x_status_t sx126x_get_lora_params_from_header( const void* context, sx126x_lora_cr_t* cr, bool* crc_is_on )
{
    uint8_t buffer_cr = 0;
    uint8_t buffer_crc = 0;

    sx126x_status_t status = sx126x_read_register( context, SX126X_REG_LR_HEADER_CR, &buffer_cr, 1 );

    if( status == SX126X_STATUS_OK )
    {
        status = sx126x_read_register( context, SX126X_REG_LR_HEADER_CRC, &buffer_crc, 1 );

        if( status == SX126X_STATUS_OK )
        {
            *cr = ( sx126x_lora_cr_t )( ( buffer_cr & SX126X_REG_LR_HEADER_CR_MASK ) >> SX126X_REG_LR_HEADER_CR_POS );
            *crc_is_on = ( ( buffer_crc & SX126X_REG_LR_HEADER_CRC_MASK ) != 0 ) ? true : false;
        }
    }

    return status;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static sx126x_status_t sx126x_tx_modulation_workaround( const void* context, sx126x_pkt_type_t pkt_type,
                                                        sx126x_lora_bw_t bw )
{
    uint8_t reg_value = 0;

    sx126x_status_t status = sx126x_read_register( context, SX126X_REG_TX_MODULATION, &reg_value, 1 );

    if( status == SX126X_STATUS_OK )
    {
        if( pkt_type == SX126X_PKT_TYPE_LORA )
        {
            if( bw == SX126X_LORA_BW_500 )
            {
                reg_value &= ~( 1 << 2 );  // Bit 2 set to 0 if the LoRa BW = 500 kHz
            }
            else
            {
                reg_value |= ( 1 << 2 );  // Bit 2 set to 1 for any other LoRa BW
            }
        }
        else
        {
            reg_value |= ( 1 << 2 );  // Bit 2 set to 1 for any (G)FSK configuration
        }

        status = sx126x_write_register( context, SX126X_REG_TX_MODULATION, &reg_value, 1 );
    }
    return status;
}

static inline uint32_t sx126x_get_gfsk_crc_len_in_bytes( sx126x_gfsk_crc_types_t crc_type )
{
    switch( crc_type )
    {
    case SX126X_GFSK_CRC_OFF:
        return 0;
    case SX126X_GFSK_CRC_1_BYTE:
        return 1;
    case SX126X_GFSK_CRC_2_BYTES:
        return 2;
    case SX126X_GFSK_CRC_1_BYTE_INV:
        return 1;
    case SX126X_GFSK_CRC_2_BYTES_INV:
        return 2;
    }

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
