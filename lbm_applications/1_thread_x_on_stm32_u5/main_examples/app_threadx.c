/**
 * @file      app_threadx.c
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
#include "app_threadx.h"
#include "smtc_modem_hal.h"
#include "smtc_hal_dbg_trace.h"
#include "smtc_hal_mcu.h"
#include "stm32u5xx_hal.h"
#include "stm32u5xx_ll_utils.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_api.h"
#include "example_options.h"
#include "smtc_hal_gpio.h"
#include "modem_pinout.h"
#include "smtc_modem_utilities.h"
#include "hw_modem.h"
#include <string.h>

/*!
 * @brief Helper macro that returned a human-friendly message if a command does not return SMTC_MODEM_RC_OK
 *
 * @remark The macro is implemented to be used with functions returning a @ref smtc_modem_return_code_t
 *
 * @param[in] rc  Return code
 */

/**
 * @brief
 *
 * @param rc
 */

/*!
 * @brief Stringify constants
 */
#define xstr( a ) str( a )
#define str( a ) #a
static void  assert_smtc_modem_rc_dbg( smtc_modem_return_code_t rc );
#define ASSERT_SMTC_MODEM_RC( rc )                                                           \
    do                                                                                       \
    {                                                                                        \
        if( tx_semaphore_get( &smtc_api_protect_semaphore, TX_WAIT_FOREVER ) == TX_SUCCESS ) \
        {                                                                                    \
            if( rc != SMTC_MODEM_RC_OK )                                                     \
            {                                                                                \
                assert_smtc_modem_rc_dbg( rc );                                              \
            }                                                                                \
        }                                                                                    \
        tx_semaphore_put( &smtc_api_protect_semaphore );                                     \
        tx_thread_wait_abort( &tx_lbm_thread ); \
    } while( 0 )

/* Private variables ---------------------------------------------------------*/

static uint32_t sleep_tick_adjust;
// LBM thread to perform LoRA TX/RX
TX_THREAD    tx_lbm_thread;
// First Application thread example , send LoRA uplink on port 101 periodically
TX_THREAD    tx_lorawan_tx_periodic_thread;
// Second Application thread example, send LoRA uplink on port 106 when user pushed the nucleo blue button or every 2 minutes 
TX_THREAD    tx_app_thread;
// A semaphore on clicking user button
TX_SEMAPHORE tx_app_semaphore;
// A semaphore to protect lbm api
TX_SEMAPHORE smtc_api_protect_semaphore;
// a semaphore to protect LBM transmission
TX_SEMAPHORE smtc_protect_smtc_tx_semaphore;
#define STACK_ID 0

/**
 * @brief User button callback for modem event
 *
 *  This callback is called every time user pushed the blue button on the nucleo board
 *  on this event, it is initiated a LoRaWan transmission.
 */
static void user_button_callback( void* context );

static hal_gpio_irq_t nucleo_blue_button = {
    .pin      = EXTI_BUTTON,
    .context  = NULL,                  // context pass to the callback - not used in this example
    .callback = user_button_callback,  // callback called when EXTI is triggered
};

/**
 * @brief static function to transmit an incremented counter en a specific LoRaWan port
 */
static void     send_uplink_counter_on_port( uint8_t port );
static uint32_t uplink_counter = 0;  // uplink raising counter


/**
 * @brief  Application ThreadX Initialization.
 * @param memory_ptr: memory pointer
 * @retval int
 */
uint32_t app_threadx_init( VOID* memory_ptr )
{
    uint32_t      ret       = TX_SUCCESS;
    TX_BYTE_POOL* byte_pool = ( TX_BYTE_POOL* ) memory_ptr;

    /* USER CODE BEGIN App_ThreadX_MEM_POOL */

    /* USER CODE END App_ThreadX_MEM_POOL */
    CHAR* pointer;
    SMTC_HAL_TRACE_INFO( "app_threadx_init is running \n" );
    /* Allocate the stack for LBM Thread  */
    if( tx_byte_allocate( byte_pool, ( VOID** ) &pointer, TX_LBM_STACK_SIZE, TX_NO_WAIT ) != TX_SUCCESS )
    {
        return TX_POOL_ERROR;
    }
    /* Create LBM Thread.  */
    if( tx_thread_create( &tx_lbm_thread, "LBM Thread", thread_lbm, 0, pointer, TX_LBM_STACK_SIZE, TX_LBM_THREAD_PRIO,
                          TX_LBM_THREAD_PREEMPTION_THRESHOLD, TX_LBM_THREAD_TIME_SLICE,
                          TX_LBM_THREAD_AUTO_START ) != TX_SUCCESS )
    {
        return TX_THREAD_ERROR;
    }
    /* Allocate the stack for tx periodic thread Thread  */
    if( tx_byte_allocate( byte_pool, ( VOID** ) &pointer, TX_LORAWAN_TX_PERIODIC_STACK_SIZE, TX_NO_WAIT ) !=
        TX_SUCCESS )
    {
        return TX_POOL_ERROR;
    }
    /* Create tx periodic thread.  */
    if( tx_thread_create( &tx_lorawan_tx_periodic_thread, "LBM TX periodic Thread", thread_lorawan_tx_periodic, 0, pointer,
                          TX_LORAWAN_TX_PERIODIC_STACK_SIZE, TX_LORAWAN_TX_PERIODIC_THREAD_PRIO,
                          TX_LORAWAN_TX_PERIODIC_THREAD_PREEMPTION_THRESHOLD, TX_LORAWAN_TX_PERIODIC_THREAD_TIME_SLICE,
                          TX_LORAWAN_TX_PERIODIC_THREAD_AUTO_START ) != TX_SUCCESS )
    {
        return TX_THREAD_ERROR;
    }
    if( tx_byte_allocate( byte_pool, ( VOID** ) &pointer, TX_APP_STACK_SIZE, TX_NO_WAIT ) != TX_SUCCESS )
    {
        return TX_POOL_ERROR;
    }
    /* Create application Thread.  */
    if( tx_thread_create( &tx_app_thread, "app Thread", thread_app, 0, pointer, TX_APP_STACK_SIZE, TX_APP_THREAD_PRIO,
                          TX_APP_THREAD_PREEMPTION_THRESHOLD, TX_APP_THREAD_TIME_SLICE,
                          TX_APP_THREAD_AUTO_START ) != TX_SUCCESS )
    {
        return TX_THREAD_ERROR;
    }
    /* Create Semaphore.  */
    if( tx_semaphore_create( &tx_app_semaphore, "Semaphore", 0 ) != TX_SUCCESS )
    {
        return TX_SEMAPHORE_ERROR;
    }
    if( tx_semaphore_create( &smtc_api_protect_semaphore, "Api_Smtc_Semaphore", 1 ) != TX_SUCCESS )
    {
        return TX_SEMAPHORE_ERROR;
    }

    if( tx_semaphore_create( &smtc_protect_smtc_tx_semaphore, "Tx_Smtc_Semaphore", 0 ) != TX_SUCCESS )
    {
        return TX_SEMAPHORE_ERROR;
    }

    // initialize the blue button as interrupt GPIO attached nucleo_blue_button as Interruption callback

    hal_gpio_init_in( EXTI_BUTTON, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_FALLING, &nucleo_blue_button );


    SMTC_HAL_TRACE_INFO( "app_threadx_init NO ERROR_V1.0 \n" );
    return ret;
}
/**
 * @brief  Function implementing the thread_lbm thread.
 * @param  thread_input: Not used.
 * @retval None
 */

// LBM thread to manage LBM stack
void thread_lbm( ULONG thread_input )
{
    SMTC_HAL_TRACE_INFO( "launch thread_lbm \n" );
    // Init LBM , modem_event_callback is called by LBM for each asynchronous events
    smtc_modem_init( &modem_event_callback );
    // main loop for Lbm thread
    while( 1 )
    {
        // call LBM run engine, this function return a max delay in ms.
        // This function can be called as many times as desired before this maximum delay,
        // but it must be called at least once before the expiration of this maximum delay
        uint32_t delay_to_sleep_ms = smtc_modem_run_engine( );
        UINT     old_priority;
        tx_thread_priority_change( &tx_lbm_thread, TX_LBM_THREAD_PRIO, &old_priority );
        tx_thread_sleep( MS_TO_TICK( delay_to_sleep_ms ) );
    }
}


// First Application thread example , send LoRa uplink on port 101 periodically (every 30 seconds)
void thread_lorawan_tx_periodic( ULONG thread_input )
{
   
#ifndef HW_MODEM_ENABLED
    SMTC_HAL_TRACE_INFO( "launch thread_lorawan_tx_periodic \n" );
    while( 1 )
    {
        // send incremental counter payload on port 101
        send_uplink_counter_on_port( 101 );
        tx_thread_sleep( MS_TO_TICK( 30000 ) );
    }
#else
    SMTC_HAL_TRACE_INFO( "launch thread hw modem emulation\n" );
    while( 1 )
    {
        // Command may generate work for the stack, so drop down to smtc_modem_run_engine().
        if( hw_modem_is_a_cmd_available( ) == true )
        {
            if( tx_semaphore_get( &smtc_api_protect_semaphore, TX_WAIT_FOREVER ) == TX_SUCCESS )
            {
                hw_modem_process_cmd( );
            }
            tx_semaphore_put( &smtc_api_protect_semaphore );
            tx_thread_wait_abort( &tx_lbm_thread );
        }
        if ( hw_modem_is_low_power_ok( ) == true )
        {
            tx_thread_sleep( 100000 );
        }
    }
#endif
}

static bool is_lbmthread_is_created = true;
/*Second Application thread example, send LoRa uplink on port 106 every 2 minutes 
 or suspend / resume radio communications when user pushed the nucleo blue button*/
void thread_app( ULONG thread_input )
{
    SMTC_HAL_TRACE_INFO( "launch thread app \n" );
    while( 1 )
    {
        // tx_app_semaphore is released when nucleo blue button is pressed or after 2 minutes
        if( tx_semaphore_get( &tx_app_semaphore, MS_TO_TICK( 120000 ) ) == TX_SUCCESS )
        {
            SMTC_HAL_TRACE_INFO( "blue button pushed\n" );
            if (is_lbmthread_is_created == true)
            {
                SMTC_HAL_TRACE_INFO( "LBM stack is suspended\n" );
                ASSERT_SMTC_MODEM_RC(smtc_modem_suspend_radio_communications (true));
                is_lbmthread_is_created =false;
            } 
            else
            {
                SMTC_HAL_TRACE_INFO( "LBM stack is resumed\n" );
                ASSERT_SMTC_MODEM_RC(smtc_modem_suspend_radio_communications (false));
                is_lbmthread_is_created =true;
            }

        }
        #ifndef HW_MODEM_ENABLED
        SMTC_HAL_TRACE_INFO( "send_uplink_counter_on_port\n" );
        send_uplink_counter_on_port( 106 );
        #endif
    }
}
/**
 * @brief  mx_threadx_init called in the main
 * @param  None
 * @retval None
 */
void mx_threadx_init( void )
{
    tx_kernel_enter( );
}

/**
 * @brief  app_threadx_low_power_enter
 * @param  None
 * @retval None
 */
static uint32_t timestamp_before_to_sleep;
void            app_threadx_low_power_enter( void )
{
    if( sleep_tick_adjust == 0 )
    {
        return;
    }
    hal_mcu_disable_irq( );
    // timestamp before to go to sleep
    timestamp_before_to_sleep = smtc_modem_hal_get_time_in_ms( );
    // this function de init the peripherals, clear and stop the systick,
    // configure the rtc wake up timer and enter in sleep mode 2
    // when an interrupt expire (rtc,radio,..) init clk,peripherals, stop wake up timer and restart
    hal_mcu_set_sleep_for_ms( TICK_TO_MS( sleep_tick_adjust ) );
}

/**
 * @brief  app_threadx_low_power_exit
 * @param  None
 * @retval None
 */
void app_threadx_low_power_exit( void )
{
    hal_mcu_enable_irq( );
}

/**
 * @brief  App_ThreadX_LowPower_Timer_Adjust
 * @param  None
 * @retval Amount of time (in ticks)
 */
ULONG App_ThreadX_LowPower_Timer_Adjust( void )
{
    if( sleep_tick_adjust == 0 )
    {
        return 0;
    }
    uint32_t adjust_time;
   // SMTC_HAL_TRACE_PRINTF( "sleep during  :%d\n", smtc_modem_hal_get_time_in_ms( ) - timestamp_before_to_sleep);
    adjust_time = smtc_modem_hal_get_time_in_ms( ) - timestamp_before_to_sleep;
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    return ( MS_TO_TICK( adjust_time ) );
}
void App_ThreadX_LowPower_Timer_Setup( unsigned long count )
{
    // this function is called by kernel to populate the time sleeping duration
    sleep_tick_adjust = count;
}

// this function is called each time LBM stack manages an interupt (low power timer or radio interupts) 
void threadx_user_lbm_irq( void )
{
    UINT old_priority;
    tx_thread_priority_change( &tx_lbm_thread, VERYHIGH, &old_priority );
    tx_thread_wait_abort( &tx_lbm_thread );
}

#ifdef HW_MODEM_ENABLED
// 
void threadx_lorawan_tx_periodic_irq( void )
{
    tx_thread_wait_abort( &tx_lorawan_tx_periodic_thread );
}
#endif

void threadx_callback_to_protect_smtc_tx( void )
{
    tx_semaphore_get( &smtc_protect_smtc_tx_semaphore, TX_WAIT_FOREVER );
}

void threadx_callback_to_release_smtc_tx( void )
{
    ULONG currentValue = 0;
    /* Add additional checks to avoid multiple semaphore  */
    tx_semaphore_info_get( &smtc_protect_smtc_tx_semaphore, NULL, &currentValue, NULL, NULL, NULL );
    if( currentValue == 0 )
    {
        /* Put the semaphore to release the LoRaWan transmission */
        tx_semaphore_put( &smtc_protect_smtc_tx_semaphore );
    }
}

/**
 * Stack id value (multistacks modem is not yet available)
 */

/**
 * @brief Stack credentials
 */
#ifndef HW_MODEM_ENABLED
#if !defined( USE_LR11XX_CREDENTIALS )
static const uint8_t user_dev_eui[8]  = USER_LORAWAN_DEVICE_EUI;
static const uint8_t user_join_eui[8] = USER_LORAWAN_JOIN_EUI;
static const uint8_t user_app_key[16] = USER_LORAWAN_APP_KEY;
#endif
/**
 * @brief Periodical uplink alarm delay in seconds
 */
#define PERIODICAL_UPLINK_DELAY_S 60
static uint8_t                  rx_payload[SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH] = { 0 };  // Buffer for rx payload
static uint8_t                  rx_payload_size = 0;      // Size of the payload in the rx_payload buffer
static smtc_modem_dl_metadata_t rx_metadata     = { 0 };  // Metadata of downlink
static uint8_t                  rx_remaining    = 0;      // Remaining downlink payload in modem
static bool                     is_joined       = false;  // Flag for join status

// this function is called by LBM stack itself to manage unsynchronous events created by LBM
void modem_event_callback( void )
{
    SMTC_HAL_TRACE_MSG_COLOR( "Modem event callback\n", HAL_DBG_TRACE_COLOR_BLUE );

    smtc_modem_event_t current_event;
    uint8_t            event_pending_count;

    // Continue to read modem event until all event has been processed
    do
    {
        // Read modem event
        ASSERT_SMTC_MODEM_RC( smtc_modem_get_event( &current_event, &event_pending_count ) );

        switch( current_event.event_type )
        {
        case SMTC_MODEM_EVENT_RESET:
            SMTC_HAL_TRACE_INFO( "Event received: RESET\n" );

#if !defined( USE_LR11XX_CREDENTIALS )
            // Set user credentials
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_deveui( STACK_ID, user_dev_eui ) );
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_joineui( STACK_ID, user_join_eui ) );
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_nwkkey( STACK_ID, user_app_key ) );
#else
            // Get internal credentials
            ASSERT_SMTC_MODEM_RC( smtc_modem_get_chip_eui( STACK_ID, chip_eui ) );
            SMTC_HAL_TRACE_ARRAY( "CHIP_EUI", chip_eui, SMTC_MODEM_EUI_LENGTH );
            ASSERT_SMTC_MODEM_RC( smtc_modem_get_pin( STACK_ID, chip_pin ) );
            SMTC_HAL_TRACE_ARRAY( "CHIP_PIN", chip_pin, SMTC_MODEM_PIN_LENGTH );
#endif
            // Set user region
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_region( STACK_ID, MODEM_EXAMPLE_REGION ) );
            // Schedule a Join LoRaWAN network
            is_joined = false;
            ASSERT_SMTC_MODEM_RC( smtc_modem_join_network( STACK_ID ) );
            break;

        case SMTC_MODEM_EVENT_ALARM:
            SMTC_HAL_TRACE_INFO( "Event received: ALARM\n" );
            ASSERT_SMTC_MODEM_RC( smtc_modem_alarm_start_timer( PERIODICAL_UPLINK_DELAY_S ) );
            break;

        case SMTC_MODEM_EVENT_JOINED:
            SMTC_HAL_TRACE_INFO( "Event received: JOINED\n" );
            SMTC_HAL_TRACE_INFO( "Modem is now joined \n" );
            is_joined = true;
            threadx_callback_to_release_smtc_tx( );
            ASSERT_SMTC_MODEM_RC( smtc_modem_alarm_start_timer( PERIODICAL_UPLINK_DELAY_S ) );
            break;

        case SMTC_MODEM_EVENT_TXDONE:
            SMTC_HAL_TRACE_INFO( "Event received: TXDONE\n" );
            threadx_callback_to_release_smtc_tx( );
            break;

        case SMTC_MODEM_EVENT_DOWNDATA:
            SMTC_HAL_TRACE_INFO( "Event received: DOWNDATA\n" );
            // Get downlink data
            ASSERT_SMTC_MODEM_RC(
                smtc_modem_get_downlink_data( rx_payload, &rx_payload_size, &rx_metadata, &rx_remaining ) );
            SMTC_HAL_TRACE_PRINTF( "Data received on port %u\n", rx_metadata.fport );
            SMTC_HAL_TRACE_ARRAY( "Received payload", rx_payload, rx_payload_size );
            break;

        case SMTC_MODEM_EVENT_JOINFAIL:
            SMTC_HAL_TRACE_INFO( "Event received: JOINFAIL\n" );
            threadx_callback_to_release_smtc_tx( );
            break;

        case SMTC_MODEM_EVENT_ALCSYNC_TIME:
            SMTC_HAL_TRACE_INFO( "Event received: ALCSync service TIME\n" );
            break;

        case SMTC_MODEM_EVENT_LINK_CHECK:
            SMTC_HAL_TRACE_INFO( "Event received: LINK_CHECK\n" );
            break;

        case SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO:
            SMTC_HAL_TRACE_INFO( "Event received: PING SLOT INFO REQ\n" );
            break;

        case SMTC_MODEM_EVENT_CLASS_B_STATUS:
            SMTC_HAL_TRACE_INFO( "Event received: CLASS_B_STATUS\n" );
            break;

        case SMTC_MODEM_EVENT_LORAWAN_MAC_TIME:
            SMTC_HAL_TRACE_WARNING( "Event received: LORAWAN MAC TIME\n" );
            break;

        case SMTC_MODEM_EVENT_LORAWAN_FUOTA_DONE:
        {
            bool status = current_event.event_data.fuota_status.successful;
            if( status == true )
            {
                SMTC_HAL_TRACE_INFO( "Event received: FUOTA SUCCESSFUL \n" );
            }
            else
            {
                SMTC_HAL_TRACE_WARNING( "Event received: FUOTA FAIL \n" );
            }
            break;
        }
        case SMTC_MODEM_EVENT_STREAM_DONE:
        {
            SMTC_HAL_TRACE_INFO( "Event received: STREAM DONE \n" );
            break;
        }
        case SMTC_MODEM_EVENT_DM_SET_CONF:
        {
            SMTC_HAL_TRACE_INFO( "Event received: DM SETCONF\n" );
            break;
        }
        break;
        case SMTC_MODEM_EVENT_MUTE:
        {
            SMTC_HAL_TRACE_INFO( "Event received: MUTE \n" );
            break;
        }
        break;

        default:
            SMTC_HAL_TRACE_ERROR( "Unknown event %u\n", current_event.event_type );
            break;
        }
    } while( event_pending_count > 0 );
}
#else
void modem_event_callback( void )
{
    return;
}
#endif
// an example of function that perform LoRA uplink  on a dedicated port 
// the 4 bytes payload is an incremented counter. 
static void send_uplink_counter_on_port( uint8_t port )
{
    threadx_callback_to_protect_smtc_tx( );
    uint8_t buff[4] = { 0 };
    buff[0]         = ( uplink_counter >> 24 ) & 0xFF;
    buff[1]         = ( uplink_counter >> 16 ) & 0xFF;
    buff[2]         = ( uplink_counter >> 8 ) & 0xFF;
    buff[3]         = ( uplink_counter & 0xFF );
    smtc_modem_return_code_t rc ;
    rc = smtc_modem_request_uplink( STACK_ID, port, false, buff, 4 );
    ASSERT_SMTC_MODEM_RC(rc);
    if (rc == SMTC_MODEM_RC_OK )
    {
    // Increment uplink counter
        if( uplink_counter < 0xFFFFFFFF )
        {
            uplink_counter++;
        }
        else
        {
            uplink_counter = 0;
        }
    } else{
        threadx_callback_to_release_smtc_tx ();
    }
   
}

// callback when user pressed the button
static void user_button_callback( void* context )
{
    ULONG currentValue = 0;
    /* Add additional checks to avoid multiple semaphore puts by successively
    clicking on the user button */
    tx_semaphore_info_get( &tx_app_semaphore, NULL, &currentValue, NULL, NULL, NULL );
    if( currentValue == 0 )
    {
        /* Put the semaphore to release the MainThread */
        tx_semaphore_put( &tx_app_semaphore );
    }
}

// Print assert for debug purpose

void assert_smtc_modem_rc_dbg( smtc_modem_return_code_t rc )
{
    if( rc == SMTC_MODEM_RC_NOT_INIT )
    {
        SMTC_HAL_TRACE_WARNING( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,
                              xstr( SMTC_MODEM_RC_NOT_INIT ) );
    }
    else if( rc == SMTC_MODEM_RC_INVALID )
    {
        SMTC_HAL_TRACE_WARNING( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,
                              xstr( SMTC_MODEM_RC_INVALID ) );
    }
    else if( rc == SMTC_MODEM_RC_BUSY )
    {
        SMTC_HAL_TRACE_WARNING( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__, xstr( SMTC_MODEM_RC_BUSY ) );
    }
    else if( rc == SMTC_MODEM_RC_FAIL )
    {
        SMTC_HAL_TRACE_WARNING( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__, xstr( SMTC_MODEM_RC_FAIL ) );
    }
    else if( rc == SMTC_MODEM_RC_NO_TIME )
    {
        SMTC_HAL_TRACE_WARNING( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,
                                xstr( SMTC_MODEM_RC_NO_TIME ) );
    }
    else if( rc == SMTC_MODEM_RC_INVALID_STACK_ID )
    {
        SMTC_HAL_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,
                              xstr( SMTC_MODEM_RC_INVALID_STACK_ID ) );
    }
    else if( rc == SMTC_MODEM_RC_NO_EVENT )
    {
        SMTC_HAL_TRACE_INFO( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,
                             xstr( SMTC_MODEM_RC_NO_EVENT ) );
    }
}