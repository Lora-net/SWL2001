/**
 * @file      geoloc_bsp.c
 *
 * @brief     Board Support Package for the lbm geolocation.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2023. All rights reserved.
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

#include <stdint.h>
#include "geolocation_bsp.h"
#include "modem_pinout.h"
#include "smtc_hal_gpio.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void geolocation_bsp_gnss_prescan_actions( void )
{
    hal_gpio_set_value( SMTC_LED_SCAN, 1 );
    hal_gpio_set_value( RADIO_LNA_CTRL, 1 );  // LNA on
}

void geolocation_bsp_gnss_postscan_actions( void )
{
    hal_gpio_set_value( SMTC_LED_SCAN, 0 );
    hal_gpio_set_value( RADIO_LNA_CTRL, 0 );  // LNA off
}

void geolocation_bsp_wifi_prescan_actions( void )
{
    hal_gpio_set_value( SMTC_LED_SCAN, 1 );
}

void geolocation_bsp_wifi_postscan_actions( void )
{
    hal_gpio_set_value( SMTC_LED_SCAN, 0 );
}

lr11xx_system_lfclk_cfg_t geolocation_bsp_get_lr11xx_lf_clock_cfg( void )
{
    return LR11XX_SYSTEM_LFCLK_XTAL;
}

void geolocation_bsp_get_lr11xx_reg_mode( const void* context, lr11xx_system_reg_mode_t* reg_mode )
{
    *reg_mode = LR11XX_SYSTEM_REG_MODE_DCDC;
}

void geolocation_bsp_gnss_get_consumption( lr11xx_gnss_instantaneous_power_consumption_ua_t* instantaneous_power_consumption_ua )
{
    /* These value are for EVK board in DC DC mode with Xtal 32KHz and a TCXO 32MHz*/
    instantaneous_power_consumption_ua->board_voltage_mv              = 3300;
    instantaneous_power_consumption_ua->init_ua                       = 3150;
    instantaneous_power_consumption_ua->phase1_gps_capture_ua         = 11900;
    instantaneous_power_consumption_ua->phase1_gps_process_ua         = 3340;
    instantaneous_power_consumption_ua->multiscan_gps_capture_ua      = 10700;
    instantaneous_power_consumption_ua->multiscan_gps_process_ua      = 4180;
    instantaneous_power_consumption_ua->phase1_beidou_capture_ua      = 13500;
    instantaneous_power_consumption_ua->phase1_beidou_process_ua      = 3190;
    instantaneous_power_consumption_ua->multiscan_beidou_capture_ua   = 12600;
    instantaneous_power_consumption_ua->multiscan_beidou_process_ua   = 3430;
    instantaneous_power_consumption_ua->sleep_32k_ua                  = 1210;
    instantaneous_power_consumption_ua->demod_sleep_32m_ua            = 2530;
}

/* --- EOF ------------------------------------------------------------------ */
