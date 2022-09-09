/*!
 * \file      radio_planner_hook_id_defs.h
 *
 * \brief     Radio planner Hardware Abstraction Layer functions definition
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
#ifndef __RADIO_PLANNER_HOOK_ID_DEFS_H__
#define __RADIO_PLANNER_HOOK_ID_DEFS_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
#ifndef RP_HOOK_ID_REDEFINE
enum RP_HOOK_ID_DEF
{
#if !defined( LR1110_MODEM_E )
    RP_HOOK_ID_USER_SUSPEND = 0,
    RP_HOOK_ID_USER_SUSPEND_0,
#endif  // !LR1110_MODEM_E
    RP_HOOK_ID_SUSPEND,
    RP_HOOK_ID_LR1MAC_STACK,
    RP_HOOK_ID_LBT,
    RP_HOOK_ID_RTC_COMPENSATION,
    RP_HOOK_ID_CLASS_B_BEACON,
#if defined( SMTC_D2D )
    RP_HOOK_ID_CLASS_B_D2D,
#endif  // SMTC_D2D
    RP_HOOK_ID_CLASS_B_PING_SLOT,
    RP_HOOK_ID_USER_SUSPEND_1,
#if !defined( LR1110_MODEM_E )
    RP_HOOK_ID_USER_SUSPEND_2,
#endif  // !LR1110_MODEM_E
    RP_HOOK_ID_CLASS_C,
    RP_HOOK_ID_MAX
};
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // __RADIO_PLANNER_HOOK_ID_DEFS_H__

/* --- EOF ------------------------------------------------------------------ */
