/*!
 * @file      lr20xx_system.c
 *
 * @brief     System driver implementation for LR20XX
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

#include <stdlib.h>
#include "lr20xx_system.h"
#include "lr20xx_hal.h"
#include "lr20xx_workarounds.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR20XX_SYSTEM_GET_STATUS_CMD_LENGTH ( 2 )
#define LR20XX_SYSTEM_GET_VERSION_CMD_LENGTH ( 2 )
#define LR20XX_SYSTEM_GET_ERRORS_CMD_LENGTH ( 2 )
#define LR20XX_SYSTEM_CLEAR_ERRORS_CMD_LENGTH ( 2 )
#define LR20XX_SYSTEM_SET_DIO_FUNC_CMD_LENGTH ( 2 + 2 )
#define LR20XX_SYSTEM_SET_DIO_RF_SWITCH_CFG_CMD_LENGTH ( 2 + 2 )
#define LR20XX_SYSTEM_SET_DIO_IRQ_CFG_CMD_LENGTH ( 2 + 5 )
#define LR20XX_SYSTEM_CLEAR_IRQ_STATUS_CMD_LENGTH ( 2 + 4 )
#define LR20XX_SYSTEM_GET_AND_CLEAR_IRQ_STATUS_CMD_LENGTH ( 2 )
#define LR20XX_SYSTEM_CFG_LF_CLK_CMD_LENGTH ( 2 + 1 )
#define LR20XX_SYSTEM_CFG_CLK_OUTPUT_CMD_LENGTH ( 2 + 1 )
#define LR20XX_SYSTEM_SET_TCXO_MODE_CMD_LENGTH ( 2 + 5 )
#define LR20XX_SYSTEM_SET_REG_MODE_CMD_LENGTH ( 2 + 1 )
#define LR20XX_SYSTEM_CALIBRATE_CMD_LENGTH ( 2 + 1 )
#define LR20XX_SYSTEM_GET_VBAT_CMD_LENGTH ( 3 )
#define LR20XX_SYSTEM_GET_TEMP_CMD_LENGTH ( 3 )
#define LR20XX_SYSTEM_GET_RANDOM_NUMBER_CMD_LENGTH ( 2 + 1 )
#define LR20XX_SYSTEM_SET_SLEEP_MODE_CMD_LENGTH ( 2 + 5 )
#define LR20XX_SYSTEM_SET_STANDBY_MODE_CMD_LENGTH ( 2 + 1 )
#define LR20XX_SYSTEM_SET_FS_MODE_CMD_LENGTH ( 2 )
#define LR20XX_SYSTEM_ADD_REGISTER_TO_RETENTION_MEM_CMD_LENGTH ( 2 + 4 )
#define LR20XX_SYSTEM_CONFIGURE_XOSC_CMD_LENGTH ( 2 + 3 )
#define LR20XX_SYSTEM_SET_EOL_CFG_CMD_LENGTH ( 2 + 1 )

/*!
 * @brief Length in byte of the status returned by the transceiver
 */
#define LR20XX_SYSTEM_GET_STATUS_DIRECT_READ_LENGTH ( 6 )

/*!
 * @brief Length in byte of the version returned by the transceiver
 */
#define LR20XX_SYSTEM_VERSION_LENGTH ( 2 )

/*!
 * @brief Length in byte of the error list returned by the transceiver
 */
#define LR20XX_SYSTEM_ERRORS_LENGTH ( 2 )

/*!
 * @brief Length in byte of the random number returned by the transceiver
 */
#define LR20XX_SYSTEM_RANDOM_NUMBER_LENGTH ( 4 )

/*!
 * @brief Length in byte of the measure (temperature or voltage) returned by the transceiver
 */
#define LR20XX_SYSTEM_MEASURE_LENGTH ( 2 )

/*!
 * @brief Length in byte of the interrupt flags returned by the transceiver
 */
#define LR20XX_SYSTEM_INTERRUPTS_LENGTH ( 4 )

static const lr20xx_system_dio_t dio_list[] = {
    LR20XX_SYSTEM_DIO_5, LR20XX_SYSTEM_DIO_6,  LR20XX_SYSTEM_DIO_7,  LR20XX_SYSTEM_DIO_8,
    LR20XX_SYSTEM_DIO_9, LR20XX_SYSTEM_DIO_10, LR20XX_SYSTEM_DIO_11,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for system related operations
 */
enum
{
    LR20XX_SYSTEM_GET_STATUS_OC                    = 0x0100,
    LR20XX_SYSTEM_GET_VERSION_OC                   = 0x0101,
    LR20XX_SYSTEM_GET_ERRORS_OC                    = 0x0110,
    LR20XX_SYSTEM_CLEAR_ERRORS_OC                  = 0x0111,
    LR20XX_SYSTEM_SET_DIO_FUNC_OC                  = 0x0112,
    LR20XX_SYSTEM_SET_DIO_RF_SWITCH_CFG_OC         = 0x0113,
    LR20XX_SYSTEM_SET_DIO_IRQ_CFG_OC               = 0x0115,
    LR20XX_SYSTEM_CLEAR_IRQ_STATUS_OC              = 0x0116,
    LR20XX_SYSTEM_GET_AND_CLEAR_IRQ_STATUS_OC      = 0x0117,
    LR20XX_SYSTEM_CFG_LF_CLK_OC                    = 0x0118,
    LR20XX_SYSTEM_CFG_CLK_OUTPUT_OC                = 0x0119,
    LR20XX_SYSTEM_SET_TCXO_MODE_OC                 = 0x0120,
    LR20XX_SYSTEM_SET_REG_MODE_OC                  = 0x0121,
    LR20XX_SYSTEM_CALIBRATE_OC                     = 0x0122,
    LR20XX_SYSTEM_GET_VBAT_OC                      = 0x0124,
    LR20XX_SYSTEM_GET_TEMP_OC                      = 0x0125,
    LR20XX_SYSTEM_GET_RANDOM_NUMBER_OC             = 0x0126,
    LR20XX_SYSTEM_SET_SLEEP_MODE_OC                = 0x0127,
    LR20XX_SYSTEM_SET_STANDBY_MODE_OC              = 0x0128,
    LR20XX_SYSTEM_SET_FS_MODE_OC                   = 0x0129,
    LR20XX_SYSTEM_ADD_REGISTER_TO_RETENTION_MEM_OC = 0x012A,
    LR20XX_SYSTEM_SET_EOL_CFG_OC                   = 0x0130,
    LR20XX_SYSTEM_CONFIGURE_XOSC_OC                = 0x0131,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Fill stat1 structure with data from stat1_byte
 *
 * @remark If \p stat1 is NULL, the function does not perform any operation
 *
 * @param [in]  stat1_byte stat1 byte
 * @param [out] stat1      stat1 structure
 */
static void lr20xx_system_convert_stat1_byte_to_enum( uint8_t stat1_byte, lr20xx_system_stat1_t* stat1 );

/*!
 * @brief Fill stat2 structure with data from stat2_byte
 *
 * @remark If \p stat2 is NULL, the function does not perform any operation
 *
 * @param [in]  stat2_byte stat2 byte
 * @param [out] stat2      stat2 structure
 */
static void lr20xx_system_convert_stat2_byte_to_enum( uint8_t stat2_byte, lr20xx_system_stat2_t* stat2 );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr20xx_status_t lr20xx_system_reset( const void* context )
{
    return ( lr20xx_status_t ) lr20xx_hal_reset( context );
}

lr20xx_status_t lr20xx_system_wakeup( const void* context )
{
    return ( lr20xx_status_t ) lr20xx_hal_wakeup( context );
}

lr20xx_status_t lr20xx_system_get_status( const void* context, lr20xx_system_stat1_t* stat1,
                                          lr20xx_system_stat2_t* stat2, lr20xx_system_irq_mask_t* irq_status )
{
    uint8_t               data[LR20XX_SYSTEM_GET_STATUS_DIRECT_READ_LENGTH] = { 0 };
    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_direct_read( context, data, LR20XX_SYSTEM_GET_STATUS_DIRECT_READ_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        lr20xx_system_convert_stat1_byte_to_enum( data[0], stat1 );
        lr20xx_system_convert_stat2_byte_to_enum( data[1], stat2 );
        if( irq_status != NULL )
        {
            *irq_status = ( ( lr20xx_system_irq_mask_t ) data[2] << 24 ) +
                          ( ( lr20xx_system_irq_mask_t ) data[3] << 16 ) +
                          ( ( lr20xx_system_irq_mask_t ) data[4] << 8 ) + ( ( lr20xx_system_irq_mask_t ) data[5] << 0 );
        }
    }

    return status;
}

lr20xx_status_t lr20xx_system_clear_reset_status_info( const void* context )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_GET_STATUS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_GET_STATUS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_GET_STATUS_OC >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_SYSTEM_GET_STATUS_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_system_get_version( const void* context, lr20xx_system_version_t* version )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_GET_VERSION_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_GET_VERSION_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_GET_VERSION_OC >> 0 ),
    };
    uint8_t rbuffer[LR20XX_SYSTEM_VERSION_LENGTH] = { 0x00 };

    const lr20xx_status_t status = ( lr20xx_status_t ) lr20xx_hal_read(
        context, cbuffer, LR20XX_SYSTEM_GET_VERSION_CMD_LENGTH, rbuffer, LR20XX_SYSTEM_VERSION_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        version->major = rbuffer[0];
        version->minor = rbuffer[1];
    }

    return status;
}

lr20xx_status_t lr20xx_system_get_errors( const void* context, lr20xx_system_errors_t* errors )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_GET_ERRORS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_GET_ERRORS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_GET_ERRORS_OC >> 0 ),
    };
    uint8_t rbuffer[LR20XX_SYSTEM_ERRORS_LENGTH] = { 0x00 };

    const lr20xx_status_t status = ( lr20xx_status_t ) lr20xx_hal_read(
        context, cbuffer, LR20XX_SYSTEM_GET_ERRORS_CMD_LENGTH, rbuffer, LR20XX_SYSTEM_ERRORS_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        *errors = ( uint16_t ) ( ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1] );
    }

    return status;
}

lr20xx_status_t lr20xx_system_clear_errors( const void* context )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_CLEAR_ERRORS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_CLEAR_ERRORS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_CLEAR_ERRORS_OC >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_SYSTEM_CLEAR_ERRORS_CMD_LENGTH, 0, 0 );
}

uint8_t lr20xx_system_dio_get_count( void )
{
    return sizeof( dio_list ) / sizeof( dio_list[0] );
}

bool lr20xx_system_dio_get_nth( uint8_t nth, lr20xx_system_dio_t* dio )
{
    if( nth < lr20xx_system_dio_get_count( ) )
    {
        *dio = dio_list[nth];
        return true;
    }
    else
    {
        return false;
    }
}

lr20xx_status_t lr20xx_system_set_dio_function( const void* context, lr20xx_system_dio_t dio,
                                                lr20xx_system_dio_func_t func, lr20xx_system_dio_drive_t drive )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_SET_DIO_FUNC_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_SET_DIO_FUNC_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_SET_DIO_FUNC_OC >> 0 ),
        ( uint8_t ) ( dio ),
        ( uint8_t ) ( ( ( uint8_t ) func << 4 ) + ( ( uint8_t ) drive << 0 ) ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_SYSTEM_SET_DIO_FUNC_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_system_set_dio_rf_switch_cfg( const void* context, lr20xx_system_dio_t dio,
                                                     const lr20xx_system_dio_rf_switch_cfg_t rf_switch_cfg )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_SET_DIO_RF_SWITCH_CFG_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_SET_DIO_RF_SWITCH_CFG_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_SET_DIO_RF_SWITCH_CFG_OC >> 0 ),
        ( uint8_t ) ( dio ),
        ( uint8_t ) ( rf_switch_cfg ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_SYSTEM_SET_DIO_RF_SWITCH_CFG_CMD_LENGTH, 0,
                                                 0 );
}

lr20xx_status_t lr20xx_system_set_dio_irq_cfg( const void* context, lr20xx_system_dio_t dio,
                                               const lr20xx_system_irq_mask_t irq_cfg )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_SET_DIO_IRQ_CFG_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_SET_DIO_IRQ_CFG_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_SET_DIO_IRQ_CFG_OC >> 0 ),
        ( uint8_t ) ( dio ),
        ( uint8_t ) ( irq_cfg >> 24 ),
        ( uint8_t ) ( irq_cfg >> 16 ),
        ( uint8_t ) ( irq_cfg >> 8 ),
        ( uint8_t ) ( irq_cfg >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_SYSTEM_SET_DIO_IRQ_CFG_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_system_clear_irq_status( const void* context, const lr20xx_system_irq_mask_t irqs_to_clear )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_CLEAR_IRQ_STATUS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_CLEAR_IRQ_STATUS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_CLEAR_IRQ_STATUS_OC >> 0 ),
        ( uint8_t ) ( irqs_to_clear >> 24 ),
        ( uint8_t ) ( irqs_to_clear >> 16 ),
        ( uint8_t ) ( irqs_to_clear >> 8 ),
        ( uint8_t ) ( irqs_to_clear >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_SYSTEM_CLEAR_IRQ_STATUS_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_system_get_and_clear_irq_status( const void* context, lr20xx_system_irq_mask_t* irqs )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_GET_AND_CLEAR_IRQ_STATUS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_GET_AND_CLEAR_IRQ_STATUS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_GET_AND_CLEAR_IRQ_STATUS_OC >> 0 ),
    };
    uint8_t rbuffer[LR20XX_SYSTEM_INTERRUPTS_LENGTH] = { 0x00 };

    const lr20xx_status_t status = ( lr20xx_status_t ) lr20xx_hal_read(
        context, cbuffer, LR20XX_SYSTEM_GET_AND_CLEAR_IRQ_STATUS_CMD_LENGTH, rbuffer, LR20XX_SYSTEM_INTERRUPTS_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        *irqs = ( ( uint32_t ) rbuffer[0] << 24 ) + ( ( uint32_t ) rbuffer[1] << 16 ) +
                ( ( uint32_t ) rbuffer[2] << 8 ) + ( ( uint32_t ) rbuffer[3] << 0 );
    }

    return status;
}

lr20xx_status_t lr20xx_system_cfg_lfclk( const void* context, const lr20xx_system_lfclk_cfg_t lfclock_cfg )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_CFG_LF_CLK_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_CFG_LF_CLK_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_CFG_LF_CLK_OC >> 0 ),
        ( uint8_t ) lfclock_cfg,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_SYSTEM_CFG_LF_CLK_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_system_cfg_clk_output( const void* context, lr20xx_system_hf_clk_scaling_t hf_clk_scaling )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_CFG_CLK_OUTPUT_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_CFG_CLK_OUTPUT_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_CFG_CLK_OUTPUT_OC >> 0 ),
        ( uint8_t ) hf_clk_scaling,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_SYSTEM_CFG_CLK_OUTPUT_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_system_set_tcxo_mode( const void* context, const lr20xx_system_tcxo_supply_voltage_t tune,
                                             const uint32_t start_delay_in_rtc_step )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_SET_TCXO_MODE_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_SET_TCXO_MODE_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_SET_TCXO_MODE_OC >> 0 ),
        ( uint8_t ) tune,
        ( uint8_t ) ( start_delay_in_rtc_step >> 24 ),
        ( uint8_t ) ( start_delay_in_rtc_step >> 16 ),
        ( uint8_t ) ( start_delay_in_rtc_step >> 8 ),
        ( uint8_t ) ( start_delay_in_rtc_step >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_SYSTEM_SET_TCXO_MODE_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_system_set_reg_mode( const void* context, const lr20xx_system_reg_mode_t reg_mode )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_SET_REG_MODE_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_SET_REG_MODE_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_SET_REG_MODE_OC >> 0 ),
        ( uint8_t ) reg_mode,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_SYSTEM_SET_REG_MODE_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_system_calibrate( const void*                            context,
                                         const lr20xx_system_calibration_mask_t blocks_to_calibrate )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_CALIBRATE_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_CALIBRATE_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_CALIBRATE_OC >> 0 ),
        ( uint8_t ) blocks_to_calibrate,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_SYSTEM_CALIBRATE_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_system_get_vbat( const void* context, lr20xx_system_value_format_t format,
                                        lr20xx_system_meas_res_t res, uint16_t* vbat )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_GET_VBAT_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_GET_VBAT_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_GET_VBAT_OC >> 0 ),
        ( uint8_t ) ( ( ( uint8_t ) format << 3 ) + ( uint8_t ) res ),
    };
    uint8_t rbuffer[LR20XX_SYSTEM_MEASURE_LENGTH] = { 0 };

    const lr20xx_status_t status = ( lr20xx_status_t ) lr20xx_hal_read(
        context, cbuffer, LR20XX_SYSTEM_GET_VBAT_CMD_LENGTH, rbuffer, LR20XX_SYSTEM_MEASURE_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        *vbat = ( uint16_t ) ( ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1] );
    }

    return status;
}

lr20xx_status_t lr20xx_system_get_temp( const void* context, lr20xx_system_value_format_t format,
                                        lr20xx_system_meas_res_t res, lr20xx_system_temp_src_t src, uint16_t* temp )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_GET_TEMP_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_GET_TEMP_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_GET_TEMP_OC >> 0 ),
        ( uint8_t ) ( ( ( uint8_t ) src << 4 ) + ( ( uint8_t ) format << 3 ) + ( uint8_t ) res ),
    };
    uint8_t rbuffer[LR20XX_SYSTEM_MEASURE_LENGTH] = { 0 };

    const lr20xx_status_t status = ( lr20xx_status_t ) lr20xx_hal_read(
        context, cbuffer, LR20XX_SYSTEM_GET_TEMP_CMD_LENGTH, rbuffer, LR20XX_SYSTEM_MEASURE_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        *temp = ( uint16_t ) ( ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1] );
    }

    return status;
}

lr20xx_status_t lr20xx_system_get_random_number( const void*                                   context,
                                                 lr20xx_system_random_entropy_source_bitmask_t source,
                                                 uint32_t*                                     random_number )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_GET_RANDOM_NUMBER_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_GET_RANDOM_NUMBER_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_GET_RANDOM_NUMBER_OC >> 0 ),
        ( uint8_t ) source,
    };

    uint8_t buffer[LR20XX_SYSTEM_RANDOM_NUMBER_LENGTH] = { 0 };

    const lr20xx_status_t status = ( lr20xx_status_t ) lr20xx_hal_read(
        context, cbuffer, LR20XX_SYSTEM_GET_RANDOM_NUMBER_CMD_LENGTH, buffer, LR20XX_SYSTEM_RANDOM_NUMBER_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        *random_number = ( ( uint32_t ) buffer[0] << 24 ) + ( ( uint32_t ) buffer[1] << 16 ) +
                         ( ( uint32_t ) buffer[2] << 8 ) + ( ( uint32_t ) buffer[3] << 0 );
    }

    return status;
}

lr20xx_status_t lr20xx_system_set_sleep_mode( const void* context, const lr20xx_system_sleep_cfg_t* sleep_cfg,
                                              const uint32_t sleep_time )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_SET_SLEEP_MODE_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_SET_SLEEP_MODE_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_SET_SLEEP_MODE_OC >> 0 ),
        ( uint8_t ) ( ( ( sleep_cfg->is_ram_retention_enabled == true ) ? 0x02 : 0x00 ) +
                      ( ( sleep_cfg->is_clk_32k_enabled == true ) ? 0x01 : 0x00 ) ),
        ( uint8_t ) ( sleep_time >> 24 ),
        ( uint8_t ) ( sleep_time >> 16 ),
        ( uint8_t ) ( sleep_time >> 8 ),
        ( uint8_t ) ( sleep_time >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_SYSTEM_SET_SLEEP_MODE_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_system_set_standby_mode( const void* context, const lr20xx_system_standby_mode_t standby_mode )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_SET_STANDBY_MODE_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_SET_STANDBY_MODE_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_SET_STANDBY_MODE_OC >> 0 ),
        ( uint8_t ) standby_mode,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_SYSTEM_SET_STANDBY_MODE_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_system_set_fs_mode( const void* context )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_SET_FS_MODE_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_SET_FS_MODE_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_SET_FS_MODE_OC >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_SYSTEM_SET_FS_MODE_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_system_add_register_to_retention_mem( const void* context, uint8_t slot, uint32_t address )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_ADD_REGISTER_TO_RETENTION_MEM_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_ADD_REGISTER_TO_RETENTION_MEM_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_ADD_REGISTER_TO_RETENTION_MEM_OC >> 0 ),
        slot,
        ( uint8_t ) ( address >> 16 ),
        ( uint8_t ) ( address >> 8 ),
        ( uint8_t ) ( address >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer,
                                                 LR20XX_SYSTEM_ADD_REGISTER_TO_RETENTION_MEM_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_system_set_lbd_cfg( const void* context, bool is_enabled, lr20xx_system_lbd_trim_t trim )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_SET_EOL_CFG_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_SET_EOL_CFG_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_SET_EOL_CFG_OC >> 0 ),
        ( uint8_t ) ( ( ( uint8_t ) trim << 1 ) | ( ( is_enabled == true ) ? 1 : 0 ) ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_SYSTEM_SET_EOL_CFG_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_system_configure_xosc( const void* context, uint8_t xta, uint8_t xtb, uint8_t wait_time_us )
{
    const uint8_t cbuffer[LR20XX_SYSTEM_CONFIGURE_XOSC_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_SYSTEM_CONFIGURE_XOSC_OC >> 8 ),
        ( uint8_t ) ( LR20XX_SYSTEM_CONFIGURE_XOSC_OC >> 0 ),
        xta,
        xtb,
        wait_time_us,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_SYSTEM_CONFIGURE_XOSC_CMD_LENGTH, 0, 0 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lr20xx_system_convert_stat1_byte_to_enum( uint8_t stat1_byte, lr20xx_system_stat1_t* stat1 )
{
    if( stat1 != NULL )
    {
        stat1->is_interrupt_active = ( ( stat1_byte & 0x01 ) == 1 ) ? true : false;
        stat1->command_status      = ( lr20xx_system_command_status_t ) ( stat1_byte >> 1 );
    }
}

static void lr20xx_system_convert_stat2_byte_to_enum( uint8_t stat2_byte, lr20xx_system_stat2_t* stat2 )
{
    if( stat2 != NULL )
    {
        stat2->chip_mode    = ( lr20xx_system_chip_modes_t ) ( stat2_byte & 0x07 );
        stat2->reset_status = ( lr20xx_system_reset_status_t ) ( ( stat2_byte & 0xF0 ) >> 4 );
    }
}

/* --- EOF ------------------------------------------------------------------ */
