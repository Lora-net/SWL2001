# SPDX-License-Identifier: BSD-3-Clause-Clear

set(CMSIS_DIR ${MCU_DRIVERS_DIR}/cmsis)
set(ST_CORE_DIR ${MCU_DRIVERS_DIR}/core/STM32U5xx)
set(ST_HAL_DIR ${MCU_DRIVERS_DIR}/STM32U5xx_HAL_Driver)
set(THREADX_DIR ${CMAKE_CURRENT_LIST_DIR}/threadx)

add_library(st_hal OBJECT
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_rng.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_lptim.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_rtc.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_rtc_ex.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_spi.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_spi_ex.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_uart.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_dma_ex.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_iwdg.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_rcc.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_rcc_ex.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_flash.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_flash_ex.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_gpio.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_dma.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_pwr.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_pwr_ex.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_cortex.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_icache.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_timebase_tim.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_tim_ex.c
    ${ST_HAL_DIR}/Src/stm32u5xx_hal_tim.c
    ${ST_CORE_DIR}/system_stm32u5xx.c
    ${ST_CORE_DIR}/startup_stm32u575xx.s

    ${THREADX_DIR}/common/src/tx_initialize_high_level.c
    ${THREADX_DIR}/common/src/tx_initialize_kernel_enter.c
    ${THREADX_DIR}/common/src/tx_initialize_kernel_setup.c
    ${THREADX_DIR}/common/src/tx_thread_stack_error_handler.c
    ${THREADX_DIR}/common/src/tx_thread_stack_error_notify.c
    ${THREADX_DIR}/common/src/tx_thread_system_resume.c
    ${THREADX_DIR}/common/src/tx_block_allocate.c
    ${THREADX_DIR}/common/src/tx_block_pool_cleanup.c
    ${THREADX_DIR}/common/src/tx_block_pool_create.c
    ${THREADX_DIR}/common/src/tx_block_pool_delete.c
    ${THREADX_DIR}/common/src/tx_block_pool_info_get.c
    ${THREADX_DIR}/common/src/tx_block_pool_initialize.c
    ${THREADX_DIR}/common/src/tx_block_pool_prioritize.c
    ${THREADX_DIR}/common/src/tx_block_release.c
    ${THREADX_DIR}/common/src/tx_byte_allocate.c
    ${THREADX_DIR}/common/src/tx_byte_pool_cleanup.c
    ${THREADX_DIR}/common/src/tx_byte_pool_create.c
    ${THREADX_DIR}/common/src/tx_byte_pool_delete.c
    ${THREADX_DIR}/common/src/tx_byte_pool_info_get.c
    ${THREADX_DIR}/common/src/tx_byte_pool_initialize.c
    ${THREADX_DIR}/common/src/tx_byte_pool_prioritize.c
    ${THREADX_DIR}/common/src/tx_byte_pool_search.c
    ${THREADX_DIR}/common/src/tx_byte_release.c
    ${THREADX_DIR}/common/src/tx_event_flags_cleanup.c
    ${THREADX_DIR}/common/src/tx_event_flags_create.c
    ${THREADX_DIR}/common/src/tx_event_flags_delete.c
    ${THREADX_DIR}/common/src/tx_event_flags_get.c
    ${THREADX_DIR}/common/src/tx_event_flags_info_get.c
    ${THREADX_DIR}/common/src/tx_event_flags_initialize.c
    ${THREADX_DIR}/common/src/tx_event_flags_set.c
    ${THREADX_DIR}/common/src/tx_event_flags_set_notify.c
    ${THREADX_DIR}/common/src/tx_mutex_cleanup.c
    ${THREADX_DIR}/common/src/tx_mutex_create.c
    ${THREADX_DIR}/common/src/tx_mutex_delete.c
    ${THREADX_DIR}/common/src/tx_mutex_get.c
    ${THREADX_DIR}/common/src/tx_mutex_info_get.c
    ${THREADX_DIR}/common/src/tx_mutex_initialize.c
    ${THREADX_DIR}/common/src/tx_mutex_prioritize.c
    ${THREADX_DIR}/common/src/tx_mutex_priority_change.c
    ${THREADX_DIR}/common/src/tx_mutex_put.c
    ${THREADX_DIR}/common/src/tx_queue_cleanup.c
    ${THREADX_DIR}/common/src/tx_queue_create.c
    ${THREADX_DIR}/common/src/tx_queue_delete.c
    ${THREADX_DIR}/common/src/tx_queue_flush.c
    ${THREADX_DIR}/common/src/tx_queue_front_send.c
    ${THREADX_DIR}/common/src/tx_queue_info_get.c
    ${THREADX_DIR}/common/src/tx_queue_initialize.c
    ${THREADX_DIR}/common/src/tx_queue_prioritize.c
    ${THREADX_DIR}/common/src/tx_queue_receive.c
    ${THREADX_DIR}/common/src/tx_queue_send.c
    ${THREADX_DIR}/common/src/tx_queue_send_notify.c
    ${THREADX_DIR}/common/src/tx_semaphore_ceiling_put.c
    ${THREADX_DIR}/common/src/tx_semaphore_cleanup.c
    ${THREADX_DIR}/common/src/tx_semaphore_create.c
    ${THREADX_DIR}/common/src/tx_semaphore_delete.c
    ${THREADX_DIR}/common/src/tx_semaphore_get.c
    ${THREADX_DIR}/common/src/tx_semaphore_info_get.c
    ${THREADX_DIR}/common/src/tx_semaphore_initialize.c
    ${THREADX_DIR}/common/src/tx_semaphore_prioritize.c
    ${THREADX_DIR}/common/src/tx_semaphore_put.c
    ${THREADX_DIR}/common/src/tx_semaphore_put_notify.c
    ${THREADX_DIR}/common/src/tx_thread_create.c
    ${THREADX_DIR}/common/src/tx_thread_delete.c
    ${THREADX_DIR}/common/src/tx_thread_entry_exit_notify.c
    ${THREADX_DIR}/common/src/tx_thread_identify.c
    ${THREADX_DIR}/common/src/tx_thread_info_get.c
    ${THREADX_DIR}/common/src/tx_thread_initialize.c
    ${THREADX_DIR}/common/src/tx_thread_preemption_change.c
    ${THREADX_DIR}/common/src/tx_thread_priority_change.c
    ${THREADX_DIR}/common/src/tx_thread_relinquish.c
    ${THREADX_DIR}/common/src/tx_thread_reset.c
    ${THREADX_DIR}/common/src/tx_thread_resume.c
    ${THREADX_DIR}/common/src/tx_thread_shell_entry.c
    ${THREADX_DIR}/common/src/tx_thread_sleep.c
    ${THREADX_DIR}/common/src/tx_thread_stack_analyze.c
    ${THREADX_DIR}/common/src/tx_thread_suspend.c
    ${THREADX_DIR}/common/src/tx_thread_system_preempt_check.c
    ${THREADX_DIR}/common/src/tx_thread_system_suspend.c
    ${THREADX_DIR}/common/src/tx_thread_terminate.c
    ${THREADX_DIR}/common/src/tx_thread_time_slice.c
    ${THREADX_DIR}/common/src/tx_thread_time_slice_change.c
    ${THREADX_DIR}/common/src/tx_thread_timeout.c
    ${THREADX_DIR}/common/src/tx_thread_wait_abort.c
    ${THREADX_DIR}/common/src/tx_time_get.c
    ${THREADX_DIR}/common/src/tx_time_set.c
    ${THREADX_DIR}/common/src/txe_block_allocate.c
    ${THREADX_DIR}/common/src/txe_block_pool_create.c
    ${THREADX_DIR}/common/src/txe_block_pool_delete.c
    ${THREADX_DIR}/common/src/txe_block_pool_info_get.c
    ${THREADX_DIR}/common/src/txe_block_pool_prioritize.c
    ${THREADX_DIR}/common/src/txe_block_release.c
    ${THREADX_DIR}/common/src/txe_byte_allocate.c
    ${THREADX_DIR}/common/src/txe_byte_pool_create.c
    ${THREADX_DIR}/common/src/txe_byte_pool_delete.c
    ${THREADX_DIR}/common/src/txe_byte_pool_info_get.c
    ${THREADX_DIR}/common/src/txe_byte_pool_prioritize.c
    ${THREADX_DIR}/common/src/txe_byte_release.c
    ${THREADX_DIR}/common/src/txe_event_flags_create.c
    ${THREADX_DIR}/common/src/txe_event_flags_delete.c
    ${THREADX_DIR}/common/src/txe_event_flags_get.c
    ${THREADX_DIR}/common/src/txe_event_flags_info_get.c
    ${THREADX_DIR}/common/src/txe_event_flags_set.c
    ${THREADX_DIR}/common/src/txe_event_flags_set_notify.c
    ${THREADX_DIR}/common/src/txe_mutex_create.c
    ${THREADX_DIR}/common/src/txe_mutex_delete.c
    ${THREADX_DIR}/common/src/txe_mutex_get.c
    ${THREADX_DIR}/common/src/txe_mutex_info_get.c
    ${THREADX_DIR}/common/src/txe_mutex_prioritize.c
    ${THREADX_DIR}/common/src/txe_mutex_put.c
    ${THREADX_DIR}/common/src/txe_queue_create.c
    ${THREADX_DIR}/common/src/txe_queue_delete.c
    ${THREADX_DIR}/common/src/txe_queue_flush.c
    ${THREADX_DIR}/common/src/txe_queue_front_send.c
    ${THREADX_DIR}/common/src/txe_queue_info_get.c
    ${THREADX_DIR}/common/src/txe_queue_prioritize.c
    ${THREADX_DIR}/common/src/txe_queue_receive.c
    ${THREADX_DIR}/common/src/txe_queue_send.c
    ${THREADX_DIR}/common/src/txe_queue_send_notify.c
    ${THREADX_DIR}/common/src/txe_semaphore_ceiling_put.c
    ${THREADX_DIR}/common/src/txe_semaphore_create.c
    ${THREADX_DIR}/common/src/txe_semaphore_delete.c
    ${THREADX_DIR}/common/src/txe_semaphore_get.c
    ${THREADX_DIR}/common/src/txe_semaphore_info_get.c
    ${THREADX_DIR}/common/src/txe_semaphore_prioritize.c
    ${THREADX_DIR}/common/src/txe_semaphore_put.c
    ${THREADX_DIR}/common/src/txe_semaphore_put_notify.c
    ${THREADX_DIR}/common/src/txe_thread_create.c
    ${THREADX_DIR}/common/src/txe_thread_delete.c
    ${THREADX_DIR}/common/src/txe_thread_entry_exit_notify.c
    ${THREADX_DIR}/common/src/txe_thread_info_get.c
    ${THREADX_DIR}/common/src/txe_thread_preemption_change.c
    ${THREADX_DIR}/common/src/txe_thread_priority_change.c
    ${THREADX_DIR}/common/src/txe_thread_relinquish.c
    ${THREADX_DIR}/common/src/txe_thread_reset.c
    ${THREADX_DIR}/common/src/txe_thread_resume.c
    ${THREADX_DIR}/common/src/txe_thread_suspend.c
    ${THREADX_DIR}/common/src/txe_thread_terminate.c
    ${THREADX_DIR}/common/src/txe_thread_time_slice_change.c
    ${THREADX_DIR}/common/src/txe_thread_wait_abort.c
    ${THREADX_DIR}/common/src/tx_timer_activate.c
    ${THREADX_DIR}/common/src/tx_timer_change.c
    ${THREADX_DIR}/common/src/tx_timer_create.c
    ${THREADX_DIR}/common/src/tx_timer_deactivate.c
    ${THREADX_DIR}/common/src/tx_timer_delete.c
    ${THREADX_DIR}/common/src/tx_timer_expiration_process.c
    ${THREADX_DIR}/common/src/tx_timer_info_get.c
    ${THREADX_DIR}/common/src/tx_timer_initialize.c
    ${THREADX_DIR}/common/src/tx_timer_system_activate.c
    ${THREADX_DIR}/common/src/tx_timer_system_deactivate.c
    ${THREADX_DIR}/common/src/tx_timer_thread_entry.c
    ${THREADX_DIR}/common/src/txe_timer_activate.c
    ${THREADX_DIR}/common/src/txe_timer_change.c
    ${THREADX_DIR}/common/src/txe_timer_create.c
    ${THREADX_DIR}/common/src/txe_timer_deactivate.c
    ${THREADX_DIR}/common/src/txe_timer_delete.c
    ${THREADX_DIR}/common/src/txe_timer_info_get.c
    ${THREADX_DIR}/utility/low_power/tx_low_power.c

    ${THREADX_DIR}/tx_initialize_low_level.s
    ${THREADX_DIR}/tx_thread_schedule.s
    ${THREADX_DIR}/tx_thread_stack_build.s
    ${THREADX_DIR}/tx_thread_system_return.s
    ${THREADX_DIR}/tx_timer_interrupt.s
)

# This is because stm32l4xx_hal_rcc.c:946 triggers a well-known preprocessor warning
target_compile_options(st_hal PRIVATE -Wno-pedantic)

target_compile_definitions(st_hal PRIVATE
    USE_HAL_DRIVER
    TX_SINGLE_MODE_NON_SECURE=1
    TX_LOW_POWER
)
target_include_directories(st_hal PUBLIC
    ${ST_CORE_DIR}
    ${CMSIS_DIR}/Core/Include
    ${CMSIS_DIR}/Device/ST/STM32U5xx/Include
    ${ST_HAL_DIR}/Inc
    ${ST_HAL_DIR}/Inc/Legacy

    ${THREADX_DIR}/utility/low_power
    ${THREADX_DIR}/common/inc
    ${THREADX_DIR}/ports/cortex_m33/gnu/inc
)

# For tx_user.h
target_include_directories(st_hal PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../main_examples
)

add_library(smtc_hal STATIC
    smtc_hal_flash.c
    smtc_hal_gpio.c
    smtc_hal_lp_timer.c
    smtc_hal_libc_stub.c
    smtc_hal_mcu.c
    smtc_hal_rng.c
    smtc_hal_rtc.c
    smtc_hal_spi.c
    smtc_hal_trace.c
    smtc_hal_uart.c
    smtc_hal_watchdog.c
)

target_include_directories(smtc_hal PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
)

target_link_libraries(smtc_hal PUBLIC st_hal)
