/*!
 * @file      smtc_modem_api_lr1110_system.c
 *
 * @brief     System api implementation for modem on LR1110
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

#include "smtc_modem_api_lr1110_system.h"

#include "modem_context.h"
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr1110_status_t smtc_modem_lr1110_system_read_uid( const void* context, lr1110_system_uid_t unique_identifier )
{
    lr1110_status_t status;
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    status = lr1110_system_read_uid( context, unique_identifier );
    modem_context_resume_radio_access( );
    return status;
}

lr1110_status_t smtc_modem_lr1110_system_read_join_eui( const void* context, lr1110_system_join_eui_t join_eui )
{
    lr1110_status_t status;
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    status = lr1110_system_read_join_eui( context, join_eui );
    modem_context_resume_radio_access( );
    return status;
}

lr1110_status_t smtc_modem_lr1110_system_read_pin( const void* context, lr1110_system_pin_t pin )
{
    lr1110_status_t status;
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    status = lr1110_system_read_pin( context, pin );
    modem_context_resume_radio_access( );
    return status;
}

lr1110_status_t smtc_modem_lr1110_system_read_pin_custom_eui( const void* context, lr1110_system_uid_t device_eui,
                                                              lr1110_system_join_eui_t join_eui, uint8_t rfu,
                                                              lr1110_system_pin_t pin )
{
    lr1110_status_t status;
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    status = lr1110_system_read_pin_custom_eui( context, device_eui, join_eui, rfu, pin );
    modem_context_resume_radio_access( );
    return status;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
