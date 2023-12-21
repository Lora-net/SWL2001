/**
 * @file      app_threadx.h
 *
 * @brief     User options to be used in example applications
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
#ifndef __APP_THREADX_H
#define __APP_THREADX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tx_api.h"
#include "main.h"
#include "tx_user.h"
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#define USE_STATIC_ALLOCATION 1
#define TX_APP_MEM_POOL_SIZE ( 8 * 1024 )

#define MS_TO_TICK( x ) x / 10 + 1
#define TICK_TO_MS( x ) x * 10



#define TX_LBM_STACK_SIZE 2048
#define TX_LORAWAN_TX_PERIODIC_STACK_SIZE 2048
#define TX_APP_STACK_SIZE 2048

#define TX_LORAWAN_TX_PERIODIC_THREAD_PRIO 15
#define TX_LBM_THREAD_PRIO 10
#define TX_APP_THREAD_PRIO 15

#ifndef TX_APP_THREAD_PREEMPTION_THRESHOLD
#define TX_APP_THREAD_PREEMPTION_THRESHOLD TX_APP_THREAD_PRIO
#endif

#ifndef TX_APP_THREAD_TIME_SLICE
#define TX_APP_THREAD_TIME_SLICE TX_NO_TIME_SLICE
#endif

#ifndef TX_APP_THREAD_AUTO_START
#define TX_APP_THREAD_AUTO_START TX_AUTO_START
#endif

#define VERYHIGH 2
#ifndef TX_LBM_THREAD_PREEMPTION_THRESHOLD
#define TX_LBM_THREAD_PREEMPTION_THRESHOLD VERYHIGH
#endif

#ifndef TX_LORAWAN_TX_PERIODIC_THREAD_PREEMPTION_THRESHOLD
#define TX_LORAWAN_TX_PERIODIC_THREAD_PREEMPTION_THRESHOLD TX_LORAWAN_TX_PERIODIC_THREAD_PRIO
#endif
#ifndef TX_LORAWAN_TX_PERIODIC_THREAD_TIME_SLICE
#define TX_LORAWAN_TX_PERIODIC_THREAD_TIME_SLICE TX_NO_TIME_SLICE
#endif
#ifndef TX_LBM_THREAD_TIME_SLICE
#define TX_LBM_THREAD_TIME_SLICE TX_NO_TIME_SLICE
#endif
#ifndef TX_LBM_THREAD_AUTO_START
#define TX_LBM_THREAD_AUTO_START TX_AUTO_START
#endif
#ifndef TX_LORAWAN_TX_PERIODIC_THREAD_AUTO_START
#define TX_LORAWAN_TX_PERIODIC_THREAD_AUTO_START TX_AUTO_START
#endif

uint32_t                    app_threadx_init( VOID* memory_ptr );
void                        mx_threadx_init( void );
void                        thread_lbm( ULONG thread_input );
void                        thread_lorawan_tx_periodic( ULONG thread_input );
void                        thread_app( ULONG thread_input );
void                        threadx_user_lbm_irq( void );
void                        threadx_lorawan_tx_periodic_irq( void );
void                        threadx_callback_to_protect_smtc_tx( void );
void                        threadx_callback_to_release_smtc_tx( void );
void                        modem_event_callback( void );
extern TX_EVENT_FLAGS_GROUP tx_lbm_isr_flag;
extern TX_THREAD            tx_lbm_thread;
extern TX_THREAD            tx_app_thread;


#ifdef __cplusplus
}
#endif
#endif /* __APP_THREADX_H__ */
