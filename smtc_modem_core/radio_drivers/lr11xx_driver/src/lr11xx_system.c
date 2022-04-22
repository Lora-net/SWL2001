/*!
 * @file      lr11xx_system.c
 *
 * @brief     System driver implementation for LR11XX
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

#include <stdlib.h>

#include "lr11xx_system.h"
#include "lr11xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR11XX_SYSTEM_GET_VERSION_CMD_LENGTH ( 2 )
#define LR11XX_SYSTEM_GET_ERRORS_CMD_LENGTH ( 2 )
#define LR11XX_SYSTEM_CLEAR_ERRORS_CMD_LENGTH ( 2 )
#define LR11XX_SYSTEM_CALIBRATE_CMD_LENGTH ( 2 + 1 )
#define LR11XX_SYSTEM_SET_REGMODE_CMD_LENGTH ( 2 + 1 )
#define LR11XX_SYSTEM_CALIBRATE_IMAGE_CMD_LENGTH ( 2 + 2 )
#define LR11XX_SYSTEM_SET_DIO_AS_RF_SWITCH_CMD_LENGTH ( 2 + 8 )
#define LR11XX_SYSTEM_SET_DIO_IRQ_PARAMS_CMD_LENGTH ( 2 + 8 )
#define LR11XX_SYSTEM_CLEAR_IRQ_CMD_LENGTH ( 2 + 4 )
#define LR11XX_SYSTEM_CFG_LFCLK_CMD_LENGTH ( 2 + 1 )
#define LR11XX_SYSTEM_SET_TCXO_MODE_CMD_LENGTH ( 2 + 4 )
#define LR11XX_SYSTEM_REBOOT_CMD_LENGTH ( 2 + 1 )
#define LR11XX_SYSTEM_GET_VBAT_CMD_LENGTH ( 2 )
#define LR11XX_SYSTEM_GET_TEMP_CMD_LENGTH ( 2 )
#define LR11XX_SYSTEM_SET_SLEEP_CMD_LENGTH ( 2 + 5 )
#define LR11XX_SYSTEM_SET_STANDBY_CMD_LENGTH ( 2 + 1 )
#define LR11XX_SYSTEM_SET_FS_CMD_LENGTH ( 2 )
#define LR11XX_SYSTEM_ERASE_INFOPAGE_CMD_LENGTH ( 2 + 1 )
#define LR11XX_SYSTEM_WRITE_INFOPAGE_CMD_LENGTH ( 2 + 3 )
#define LR11XX_SYSTEM_READ_INFOPAGE_CMD_LENGTH ( 2 + 4 )
#define LR11XX_SYSTEM_READ_UID_CMD_LENGTH ( 2 )
#define LR11XX_SYSTEM_READ_JOIN_EUI_CMD_LENGTH ( 2 )
#define LR11XX_SYSTEM_READ_PIN_CMD_LENGTH ( 2 )
#define LR11XX_SYSTEM_READ_PIN_CUSTOM_EUI_CMD_LENGTH ( LR11XX_SYSTEM_READ_PIN_CMD_LENGTH + 17 )
#define LR11XX_SYSTEM_GET_RANDOM_CMD_LENGTH ( 2 )
#define LR11XX_SYSTEM_ENABLE_SPI_CRC_CMD_LENGTH ( 3 )
#define LR11XX_SYSTEM_DRIVE_DIO_IN_SLEEP_MODE_CMD_LENGTH ( 3 )

#define LR11XX_SYSTEM_GET_STATUS_DIRECT_READ_LENGTH ( 6 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for system-related operations
 */
enum
{
    LR11XX_SYSTEM_GET_STATUS_OC              = 0x0100,
    LR11XX_SYSTEM_GET_VERSION_OC             = 0x0101,
    LR11XX_SYSTEM_GET_ERRORS_OC              = 0x010D,
    LR11XX_SYSTEM_CLEAR_ERRORS_OC            = 0x010E,
    LR11XX_SYSTEM_CALIBRATE_OC               = 0x010F,
    LR11XX_SYSTEM_SET_REGMODE_OC             = 0x0110,
    LR11XX_SYSTEM_CALIBRATE_IMAGE_OC         = 0x0111,
    LR11XX_SYSTEM_SET_DIO_AS_RF_SWITCH_OC    = 0x0112,
    LR11XX_SYSTEM_SET_DIOIRQPARAMS_OC        = 0x0113,
    LR11XX_SYSTEM_CLEAR_IRQ_OC               = 0x0114,
    LR11XX_SYSTEM_CFG_LFCLK_OC               = 0x0116,
    LR11XX_SYSTEM_SET_TCXO_MODE_OC           = 0x0117,
    LR11XX_SYSTEM_REBOOT_OC                  = 0x0118,
    LR11XX_SYSTEM_GET_VBAT_OC                = 0x0119,
    LR11XX_SYSTEM_GET_TEMP_OC                = 0x011A,
    LR11XX_SYSTEM_SET_SLEEP_OC               = 0x011B,
    LR11XX_SYSTEM_SET_STANDBY_OC             = 0x011C,
    LR11XX_SYSTEM_SET_FS_OC                  = 0x011D,
    LR11XX_SYSTEM_GET_RANDOM_OC              = 0x0120,
    LR11XX_SYSTEM_ERASE_INFOPAGE_OC          = 0x0121,
    LR11XX_SYSTEM_WRITE_INFOPAGE_OC          = 0x0122,
    LR11XX_SYSTEM_READ_INFOPAGE_OC           = 0x0123,
    LR11XX_SYSTEM_READ_UID_OC                = 0x0125,
    LR11XX_SYSTEM_READ_JOIN_EUI_OC           = 0x0126,
    LR11XX_SYSTEM_READ_PIN_OC                = 0x0127,
    LR11XX_SYSTEM_ENABLE_SPI_CRC_OC          = 0x0128,
    LR11XX_SYSTEM_DRIVE_DIO_IN_SLEEP_MODE_OC = 0x012A,
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
 * @param [in]  stat1_byte stat1 byte
 * @param [out] stat1      stat1 structure
 */
static void lr11xx_system_convert_stat1_byte_to_enum( uint8_t stat1_byte, lr11xx_system_stat1_t* stat1 );

/*!
 * @brief Fill stat2 structure with data from stat2_byte
 *
 * @param [in]  stat2_byte stat2 byte
 * @param [out] stat2      stat2 structure
 */
static void lr11xx_system_convert_stat2_byte_to_enum( uint8_t stat2_byte, lr11xx_system_stat2_t* stat2 );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_status_t lr11xx_system_reset( const void* context )
{
    return ( lr11xx_status_t ) lr11xx_hal_reset( context );
}

lr11xx_status_t lr11xx_system_get_status( const void* context, lr11xx_system_stat1_t* stat1,
                                          lr11xx_system_stat2_t* stat2, lr11xx_system_irq_mask_t* irq_status )
{
    uint8_t         data[LR11XX_SYSTEM_GET_STATUS_DIRECT_READ_LENGTH];
    lr11xx_status_t status;

    status = ( lr11xx_status_t ) lr11xx_hal_direct_read( context, data, LR11XX_SYSTEM_GET_STATUS_DIRECT_READ_LENGTH );

    if( status == LR11XX_STATUS_OK )
    {
        lr11xx_system_convert_stat1_byte_to_enum( data[0], stat1 );
        lr11xx_system_convert_stat2_byte_to_enum( data[1], stat2 );
        if( irq_status != NULL )
        {
            *irq_status = ( ( lr11xx_system_irq_mask_t ) data[2] << 24 ) +
                          ( ( lr11xx_system_irq_mask_t ) data[3] << 16 ) +
                          ( ( lr11xx_system_irq_mask_t ) data[4] << 8 ) + ( ( lr11xx_system_irq_mask_t ) data[5] << 0 );
        }
    }

    return status;
}

lr11xx_status_t lr11xx_system_clear_reset_status_info( const void* context )
{
    uint8_t cbuffer[2] = {
        ( uint8_t ) ( LR11XX_SYSTEM_GET_STATUS_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_GET_STATUS_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, sizeof( cbuffer ), 0, 0 );
}

lr11xx_status_t lr11xx_system_get_version( const void* context, lr11xx_system_version_t* version )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_GET_VERSION_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_GET_VERSION_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_GET_VERSION_OC >> 0 ),
    };
    uint8_t rbuffer[LR11XX_SYSTEM_VERSION_LENGTH] = { 0x00 };

    const lr11xx_status_t status = ( lr11xx_status_t ) lr11xx_hal_read(
        context, cbuffer, LR11XX_SYSTEM_GET_VERSION_CMD_LENGTH, rbuffer, LR11XX_SYSTEM_VERSION_LENGTH );

    if( status == LR11XX_STATUS_OK )
    {
        version->hw   = rbuffer[0];
        version->type = rbuffer[1];
        version->fw   = ( ( uint16_t ) rbuffer[2] << 8 ) + ( uint16_t ) rbuffer[3];
    }

    return status;
}

lr11xx_status_t lr11xx_system_get_errors( const void* context, lr11xx_system_errors_t* errors )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_GET_ERRORS_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_GET_ERRORS_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_GET_ERRORS_OC >> 0 ),
    };
    uint8_t rbuffer[sizeof( errors )] = { 0x00 };

    const lr11xx_status_t status = ( lr11xx_status_t ) lr11xx_hal_read(
        context, cbuffer, LR11XX_SYSTEM_GET_ERRORS_CMD_LENGTH, rbuffer, sizeof( *errors ) );

    if( status == LR11XX_STATUS_OK )
    {
        *errors = ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1];
    }

    return status;
}

lr11xx_status_t lr11xx_system_clear_errors( const void* context )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_CLEAR_ERRORS_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_CLEAR_ERRORS_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_CLEAR_ERRORS_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_SYSTEM_CLEAR_ERRORS_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_system_calibrate( const void* context, const uint8_t calib_param )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_CALIBRATE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_CALIBRATE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_CALIBRATE_OC >> 0 ),
        calib_param,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_SYSTEM_CALIBRATE_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_system_set_reg_mode( const void* context, const lr11xx_system_reg_mode_t reg_mode )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_SET_REGMODE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_SET_REGMODE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_SET_REGMODE_OC >> 0 ),
        ( uint8_t ) reg_mode,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_SYSTEM_SET_REGMODE_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_system_calibrate_image( const void* context, const uint8_t freq1, const uint8_t freq2 )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_CALIBRATE_IMAGE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_CALIBRATE_IMAGE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_CALIBRATE_IMAGE_OC >> 0 ),
        freq1,
        freq2,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_SYSTEM_CALIBRATE_IMAGE_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_system_calibrate_image_in_mhz( const void* context, const uint16_t freq1_in_mhz,
                                                      const uint16_t freq2_in_mhz )
{
    // Perform a floor() to get a value for freq1 corresponding to a frequency lower than or equal to freq1_in_mhz
    const uint8_t freq1 = freq1_in_mhz / LR11XX_SYSTEM_IMAGE_CALIBRATION_STEP_IN_MHZ;

    // Perform a ceil() to get a value for freq2 corresponding to a frequency higher than or equal to freq2_in_mhz
    const uint8_t freq2 = ( freq2_in_mhz + LR11XX_SYSTEM_IMAGE_CALIBRATION_STEP_IN_MHZ - 1 ) /
                          LR11XX_SYSTEM_IMAGE_CALIBRATION_STEP_IN_MHZ;

    return lr11xx_system_calibrate_image( context, freq1, freq2 );
}

lr11xx_status_t lr11xx_system_set_dio_as_rf_switch( const void*                         context,
                                                    const lr11xx_system_rfswitch_cfg_t* rf_switch_cfg )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_SET_DIO_AS_RF_SWITCH_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_SET_DIO_AS_RF_SWITCH_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_SET_DIO_AS_RF_SWITCH_OC >> 0 ),
        rf_switch_cfg->enable,
        rf_switch_cfg->standby,
        rf_switch_cfg->rx,
        rf_switch_cfg->tx,
        rf_switch_cfg->tx_hp,
        rf_switch_cfg->tx_hf,
        rf_switch_cfg->gnss,
        rf_switch_cfg->wifi,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_SYSTEM_SET_DIO_AS_RF_SWITCH_CMD_LENGTH, 0,
                                                 0 );
}

lr11xx_status_t lr11xx_system_set_dio_irq_params( const void*                    context,
                                                  const lr11xx_system_irq_mask_t irqs_to_enable_dio1,
                                                  const lr11xx_system_irq_mask_t irqs_to_enable_dio2 )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_SET_DIO_IRQ_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_SET_DIOIRQPARAMS_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_SET_DIOIRQPARAMS_OC >> 0 ),
        ( uint8_t ) ( irqs_to_enable_dio1 >> 24 ),
        ( uint8_t ) ( irqs_to_enable_dio1 >> 16 ),
        ( uint8_t ) ( irqs_to_enable_dio1 >> 8 ),
        ( uint8_t ) ( irqs_to_enable_dio1 >> 0 ),
        ( uint8_t ) ( irqs_to_enable_dio2 >> 24 ),
        ( uint8_t ) ( irqs_to_enable_dio2 >> 16 ),
        ( uint8_t ) ( irqs_to_enable_dio2 >> 8 ),
        ( uint8_t ) ( irqs_to_enable_dio2 >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_SYSTEM_SET_DIO_IRQ_PARAMS_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_system_clear_irq_status( const void* context, const lr11xx_system_irq_mask_t irqs_to_clear )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_CLEAR_IRQ_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_CLEAR_IRQ_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_CLEAR_IRQ_OC >> 0 ),
        ( uint8_t ) ( irqs_to_clear >> 24 ),
        ( uint8_t ) ( irqs_to_clear >> 16 ),
        ( uint8_t ) ( irqs_to_clear >> 8 ),
        ( uint8_t ) ( irqs_to_clear >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_SYSTEM_CLEAR_IRQ_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_system_get_and_clear_irq_status( const void* context, lr11xx_system_irq_mask_t* irq )
{
    lr11xx_system_irq_mask_t lr11xx_irq_mask = LR11XX_SYSTEM_IRQ_NONE;

    lr11xx_status_t status = lr11xx_system_get_irq_status( context, &lr11xx_irq_mask );

    if( ( status == LR11XX_STATUS_OK ) && ( lr11xx_irq_mask != 0 ) )
    {
        status = lr11xx_system_clear_irq_status( context, lr11xx_irq_mask );
    }
    if( ( status == LR11XX_STATUS_OK ) && ( irq != NULL ) )
    {
        *irq = lr11xx_irq_mask;
    }

    return status;
}

lr11xx_status_t lr11xx_system_cfg_lfclk( const void* context, const lr11xx_system_lfclk_cfg_t lfclock_cfg,
                                         const bool wait_for_32k_ready )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_CFG_LFCLK_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_CFG_LFCLK_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_CFG_LFCLK_OC >> 0 ),
        ( uint8_t ) ( lfclock_cfg | ( wait_for_32k_ready << 2 ) ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_SYSTEM_CFG_LFCLK_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_system_set_tcxo_mode( const void* context, const lr11xx_system_tcxo_supply_voltage_t tune,
                                             const uint32_t timeout )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_SET_TCXO_MODE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_SET_TCXO_MODE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_SET_TCXO_MODE_OC >> 0 ),
        ( uint8_t ) tune,
        ( uint8_t ) ( timeout >> 16 ),
        ( uint8_t ) ( timeout >> 8 ),
        ( uint8_t ) ( timeout >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_SYSTEM_SET_TCXO_MODE_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_system_reboot( const void* context, const bool stay_in_bootloader )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_REBOOT_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_REBOOT_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_REBOOT_OC >> 0 ),
        ( stay_in_bootloader == true ) ? 0x03 : 0x00,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_SYSTEM_REBOOT_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_system_get_vbat( const void* context, uint8_t* vbat )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_GET_VBAT_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_GET_VBAT_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_GET_VBAT_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_SYSTEM_GET_VBAT_CMD_LENGTH, vbat,
                                                sizeof( *vbat ) );
}

lr11xx_status_t lr11xx_system_get_temp( const void* context, uint16_t* temp )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_GET_TEMP_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_GET_TEMP_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_GET_TEMP_OC >> 0 ),
    };
    uint8_t rbuffer[sizeof( uint16_t )] = { 0x00 };

    const lr11xx_status_t status = ( lr11xx_status_t ) lr11xx_hal_read(
        context, cbuffer, LR11XX_SYSTEM_GET_TEMP_CMD_LENGTH, rbuffer, sizeof( uint16_t ) );

    if( status == LR11XX_STATUS_OK )
    {
        *temp = ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1];
    }

    return status;
}

lr11xx_status_t lr11xx_system_set_sleep( const void* context, const lr11xx_system_sleep_cfg_t sleep_cfg,
                                         const uint32_t sleep_time )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_SET_SLEEP_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_SET_SLEEP_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_SET_SLEEP_OC >> 0 ),
        ( sleep_cfg.is_rtc_timeout << 1 ) + sleep_cfg.is_warm_start,
        ( uint8_t ) ( sleep_time >> 24 ),
        ( uint8_t ) ( sleep_time >> 16 ),
        ( uint8_t ) ( sleep_time >> 8 ),
        ( uint8_t ) ( sleep_time >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_SYSTEM_SET_SLEEP_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_system_set_standby( const void* context, const lr11xx_system_standby_cfg_t standby_cfg )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_SET_STANDBY_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_SET_STANDBY_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_SET_STANDBY_OC >> 0 ),
        ( uint8_t ) standby_cfg,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_SYSTEM_SET_STANDBY_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_system_wakeup( const void* context )
{
    return ( lr11xx_status_t ) lr11xx_hal_wakeup( context );
}

lr11xx_status_t lr11xx_system_set_fs( const void* context )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_SET_FS_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_SET_FS_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_SET_FS_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_SYSTEM_SET_FS_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_system_erase_infopage( const void* context, const lr11xx_system_infopage_id_t infopage_id )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_ERASE_INFOPAGE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_ERASE_INFOPAGE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_ERASE_INFOPAGE_OC >> 0 ),
        ( uint8_t ) infopage_id,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_SYSTEM_ERASE_INFOPAGE_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_system_write_infopage( const void* context, const lr11xx_system_infopage_id_t infopage_id,
                                              const uint16_t address, const uint32_t* data, const uint8_t length )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_WRITE_INFOPAGE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_WRITE_INFOPAGE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_WRITE_INFOPAGE_OC >> 0 ),
        ( uint8_t ) infopage_id,
        ( uint8_t ) ( address >> 8 ),
        ( uint8_t ) ( address >> 0 ),
    };
    uint8_t cdata[256];

    for( uint16_t index = 0; index < length; index++ )
    {
        uint8_t* cdata_local = &cdata[index * sizeof( uint32_t )];

        cdata_local[0] = ( uint8_t ) ( data[index] >> 24 );
        cdata_local[1] = ( uint8_t ) ( data[index] >> 16 );
        cdata_local[2] = ( uint8_t ) ( data[index] >> 8 );
        cdata_local[3] = ( uint8_t ) ( data[index] >> 0 );
    }

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_SYSTEM_WRITE_INFOPAGE_CMD_LENGTH, cdata,
                                                 length * sizeof( uint32_t ) );
}

lr11xx_status_t lr11xx_system_read_infopage( const void* context, const lr11xx_system_infopage_id_t infopage_id,
                                             const uint16_t address, uint32_t* data, const uint8_t length )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_READ_INFOPAGE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_READ_INFOPAGE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_READ_INFOPAGE_OC >> 0 ),
        ( uint8_t ) infopage_id,
        ( uint8_t ) ( address >> 8 ),
        ( uint8_t ) ( address >> 0 ),
        length,
    };

    const lr11xx_status_t status = ( lr11xx_status_t ) lr11xx_hal_read(
        context, cbuffer, LR11XX_SYSTEM_READ_INFOPAGE_CMD_LENGTH, ( uint8_t* ) data, length * sizeof( *data ) );

    if( status == LR11XX_STATUS_OK )
    {
        for( uint8_t index = 0; index < length; index++ )
        {
            uint8_t* buffer_local = ( uint8_t* ) &data[index];

            data[index] = ( ( uint32_t ) buffer_local[0] << 24 ) + ( ( uint32_t ) buffer_local[1] << 16 ) +
                          ( ( uint32_t ) buffer_local[2] << 8 ) + ( ( uint32_t ) buffer_local[3] << 0 );
        }
    }

    return status;
}

lr11xx_status_t lr11xx_system_read_uid( const void* context, lr11xx_system_uid_t unique_identifier )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_READ_UID_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_READ_UID_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_READ_UID_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_SYSTEM_READ_UID_CMD_LENGTH, unique_identifier,
                                                LR11XX_SYSTEM_UID_LENGTH );
}

lr11xx_status_t lr11xx_system_read_join_eui( const void* context, lr11xx_system_join_eui_t join_eui )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_READ_JOIN_EUI_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_READ_JOIN_EUI_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_READ_JOIN_EUI_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_SYSTEM_READ_JOIN_EUI_CMD_LENGTH, join_eui,
                                                LR11XX_SYSTEM_JOIN_EUI_LENGTH );
}

lr11xx_status_t lr11xx_system_read_pin( const void* context, lr11xx_system_pin_t pin )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_READ_PIN_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_READ_PIN_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_READ_PIN_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_SYSTEM_READ_PIN_CMD_LENGTH, pin,
                                                LR11XX_SYSTEM_PIN_LENGTH );
}

lr11xx_status_t lr11xx_system_read_pin_custom_eui( const void* context, lr11xx_system_uid_t device_eui,
                                                   lr11xx_system_join_eui_t join_eui, uint8_t rfu,
                                                   lr11xx_system_pin_t pin )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_READ_PIN_CUSTOM_EUI_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_READ_PIN_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_READ_PIN_OC >> 0 ),
        device_eui[0],
        device_eui[1],
        device_eui[2],
        device_eui[3],
        device_eui[4],
        device_eui[5],
        device_eui[6],
        device_eui[7],
        join_eui[0],
        join_eui[1],
        join_eui[2],
        join_eui[3],
        join_eui[4],
        join_eui[5],
        join_eui[6],
        join_eui[7],
        rfu,
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_SYSTEM_READ_PIN_CUSTOM_EUI_CMD_LENGTH, pin,
                                                LR11XX_SYSTEM_PIN_LENGTH );
}

lr11xx_status_t lr11xx_system_get_random_number( const void* context, uint32_t* random_number )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_GET_RANDOM_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_GET_RANDOM_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_GET_RANDOM_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_SYSTEM_GET_RANDOM_CMD_LENGTH,
                                                ( uint8_t* ) random_number, sizeof( uint32_t ) );
}

lr11xx_status_t lr11xx_system_enable_spi_crc( const void* context, bool enable_crc )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_ENABLE_SPI_CRC_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_ENABLE_SPI_CRC_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_ENABLE_SPI_CRC_OC >> 0 ),
        ( enable_crc == true ) ? 0x01 : 0x00,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_SYSTEM_ENABLE_SPI_CRC_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_system_drive_dio_in_sleep_mode( const void* context, bool enable_drive )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_DRIVE_DIO_IN_SLEEP_MODE_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_SYSTEM_DRIVE_DIO_IN_SLEEP_MODE_OC >> 8 ),
        ( uint8_t ) ( LR11XX_SYSTEM_DRIVE_DIO_IN_SLEEP_MODE_OC >> 0 ),
        ( enable_drive == true ) ? 0x01 : 0x00,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_SYSTEM_DRIVE_DIO_IN_SLEEP_MODE_CMD_LENGTH, 0,
                                                 0 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lr11xx_system_convert_stat1_byte_to_enum( uint8_t stat1_byte, lr11xx_system_stat1_t* stat1 )
{
    if( stat1 != NULL )
    {
        stat1->is_interrupt_active = ( ( stat1_byte & 0x01 ) != 0 ) ? true : false;
        stat1->command_status      = ( lr11xx_system_command_status_t ) ( stat1_byte >> 1 );
    }
}

static void lr11xx_system_convert_stat2_byte_to_enum( uint8_t stat2_byte, lr11xx_system_stat2_t* stat2 )
{
    if( stat2 != NULL )
    {
        stat2->is_running_from_flash = ( ( stat2_byte & 0x01 ) != 0 ) ? true : false;
        stat2->chip_mode             = ( lr11xx_system_chip_modes_t ) ( ( stat2_byte & 0x0F ) >> 1 );
        stat2->reset_status          = ( lr11xx_system_reset_status_t ) ( ( stat2_byte & 0xF0 ) >> 4 );
    }
}

/* --- EOF ------------------------------------------------------------------ */
