##############################################################################
# Definitions for the STM32 U575 board
##############################################################################

#-----------------------------------------------------------------------------
# Compilation flags
#-----------------------------------------------------------------------------

#MCU compilation flags
MCU_FLAGS ?= -mcpu=cortex-m33 -mthumb -mabi=aapcs -mfpu=fpv4-sp-d16 -mfloat-abi=hard
BOARD_AS_DEFS = \
	-DTX_SINGLE_MODE_NON_SECURE=1\
    -DTX_LOW_POWER 
BOARD_C_DEFS =  \
	-DUSE_HAL_DRIVER \
	-DTX_SINGLE_MODE_NON_SECURE=1 \
	-DSTM32U575xx

BOARD_LDSCRIPT = mcu_drivers/core/STM32U5xx/STM32U575ZITXQ_FLASH.ld

#-----------------------------------------------------------------------------
# Hardware-specific sources
#-----------------------------------------------------------------------------
BOARD_C_SOURCES = \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rng.c\
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_lptim.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc_ex.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi_ex.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma_ex.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_iwdg.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc_ex.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash_ex.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpio.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr_ex.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cortex.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_icache.c \
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_timebase_tim.c\
	mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim_ex.c\
    mcu_drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim.c\
	smtc_modem_hal/smtc_modem_hal.c\
	mcu_drivers/core/STM32U5xx/system_stm32u5xx.c\
	smtc_hal_u5/smtc_hal_flash.c\
	smtc_hal_u5/smtc_hal_gpio.c\
	smtc_hal_u5/smtc_hal_mcu.c\
	smtc_hal_u5/smtc_hal_rtc.c\
	smtc_hal_u5/smtc_hal_rng.c\
	smtc_hal_u5/smtc_hal_spi.c\
	smtc_hal_u5/smtc_hal_lp_timer.c\
	smtc_hal_u5/smtc_hal_trace.c\
	smtc_hal_u5/smtc_hal_uart.c\
	smtc_hal_u5/smtc_hal_watchdog.c\
    smtc_hal_u5/threadx/common/src/tx_initialize_high_level.c \
smtc_hal_u5/threadx/common/src/tx_initialize_kernel_enter.c \
smtc_hal_u5/threadx/common/src/tx_initialize_kernel_setup.c \
smtc_hal_u5/threadx/common/src/tx_thread_stack_error_handler.c \
smtc_hal_u5/threadx/common/src/tx_thread_stack_error_notify.c \
smtc_hal_u5/threadx/common/src/tx_thread_system_resume.c \
smtc_hal_u5/threadx/common/src/tx_block_allocate.c \
smtc_hal_u5/threadx/common/src/tx_block_pool_cleanup.c \
smtc_hal_u5/threadx/common/src/tx_block_pool_create.c \
smtc_hal_u5/threadx/common/src/tx_block_pool_delete.c \
smtc_hal_u5/threadx/common/src/tx_block_pool_info_get.c \
smtc_hal_u5/threadx/common/src/tx_block_pool_initialize.c \
smtc_hal_u5/threadx/common/src/tx_block_pool_prioritize.c \
smtc_hal_u5/threadx/common/src/tx_block_release.c \
smtc_hal_u5/threadx/common/src/tx_byte_allocate.c \
smtc_hal_u5/threadx/common/src/tx_byte_pool_cleanup.c \
smtc_hal_u5/threadx/common/src/tx_byte_pool_create.c \
smtc_hal_u5/threadx/common/src/tx_byte_pool_delete.c \
smtc_hal_u5/threadx/common/src/tx_byte_pool_info_get.c \
smtc_hal_u5/threadx/common/src/tx_byte_pool_initialize.c \
smtc_hal_u5/threadx/common/src/tx_byte_pool_prioritize.c \
smtc_hal_u5/threadx/common/src/tx_byte_pool_search.c \
smtc_hal_u5/threadx/common/src/tx_byte_release.c \
smtc_hal_u5/threadx/common/src/tx_event_flags_cleanup.c \
smtc_hal_u5/threadx/common/src/tx_event_flags_create.c \
smtc_hal_u5/threadx/common/src/tx_event_flags_delete.c \
smtc_hal_u5/threadx/common/src/tx_event_flags_get.c \
smtc_hal_u5/threadx/common/src/tx_event_flags_info_get.c \
smtc_hal_u5/threadx/common/src/tx_event_flags_initialize.c \
smtc_hal_u5/threadx/common/src/tx_event_flags_set.c \
smtc_hal_u5/threadx/common/src/tx_event_flags_set_notify.c \
smtc_hal_u5/threadx/common/src/tx_mutex_cleanup.c \
smtc_hal_u5/threadx/common/src/tx_mutex_create.c \
smtc_hal_u5/threadx/common/src/tx_mutex_delete.c \
smtc_hal_u5/threadx/common/src/tx_mutex_get.c \
smtc_hal_u5/threadx/common/src/tx_mutex_info_get.c \
smtc_hal_u5/threadx/common/src/tx_mutex_initialize.c \
smtc_hal_u5/threadx/common/src/tx_mutex_prioritize.c \
smtc_hal_u5/threadx/common/src/tx_mutex_priority_change.c \
smtc_hal_u5/threadx/common/src/tx_mutex_put.c \
smtc_hal_u5/threadx/common/src/tx_queue_cleanup.c \
smtc_hal_u5/threadx/common/src/tx_queue_create.c \
smtc_hal_u5/threadx/common/src/tx_queue_delete.c \
smtc_hal_u5/threadx/common/src/tx_queue_flush.c \
smtc_hal_u5/threadx/common/src/tx_queue_front_send.c \
smtc_hal_u5/threadx/common/src/tx_queue_info_get.c \
smtc_hal_u5/threadx/common/src/tx_queue_initialize.c \
smtc_hal_u5/threadx/common/src/tx_queue_prioritize.c \
smtc_hal_u5/threadx/common/src/tx_queue_receive.c \
smtc_hal_u5/threadx/common/src/tx_queue_send.c \
smtc_hal_u5/threadx/common/src/tx_queue_send_notify.c \
smtc_hal_u5/threadx/common/src/tx_semaphore_ceiling_put.c \
smtc_hal_u5/threadx/common/src/tx_semaphore_cleanup.c \
smtc_hal_u5/threadx/common/src/tx_semaphore_create.c \
smtc_hal_u5/threadx/common/src/tx_semaphore_delete.c \
smtc_hal_u5/threadx/common/src/tx_semaphore_get.c \
smtc_hal_u5/threadx/common/src/tx_semaphore_info_get.c \
smtc_hal_u5/threadx/common/src/tx_semaphore_initialize.c \
smtc_hal_u5/threadx/common/src/tx_semaphore_prioritize.c \
smtc_hal_u5/threadx/common/src/tx_semaphore_put.c \
smtc_hal_u5/threadx/common/src/tx_semaphore_put_notify.c \
smtc_hal_u5/threadx/common/src/tx_thread_create.c \
smtc_hal_u5/threadx/common/src/tx_thread_delete.c \
smtc_hal_u5/threadx/common/src/tx_thread_entry_exit_notify.c \
smtc_hal_u5/threadx/common/src/tx_thread_identify.c \
smtc_hal_u5/threadx/common/src/tx_thread_info_get.c \
smtc_hal_u5/threadx/common/src/tx_thread_initialize.c \
smtc_hal_u5/threadx/common/src/tx_thread_preemption_change.c \
smtc_hal_u5/threadx/common/src/tx_thread_priority_change.c \
smtc_hal_u5/threadx/common/src/tx_thread_relinquish.c \
smtc_hal_u5/threadx/common/src/tx_thread_reset.c \
smtc_hal_u5/threadx/common/src/tx_thread_resume.c \
smtc_hal_u5/threadx/common/src/tx_thread_shell_entry.c \
smtc_hal_u5/threadx/common/src/tx_thread_sleep.c \
smtc_hal_u5/threadx/common/src/tx_thread_stack_analyze.c \
smtc_hal_u5/threadx/common/src/tx_thread_suspend.c \
smtc_hal_u5/threadx/common/src/tx_thread_system_preempt_check.c \
smtc_hal_u5/threadx/common/src/tx_thread_system_suspend.c \
smtc_hal_u5/threadx/common/src/tx_thread_terminate.c \
smtc_hal_u5/threadx/common/src/tx_thread_time_slice.c \
smtc_hal_u5/threadx/common/src/tx_thread_time_slice_change.c \
smtc_hal_u5/threadx/common/src/tx_thread_timeout.c \
smtc_hal_u5/threadx/common/src/tx_thread_wait_abort.c \
smtc_hal_u5/threadx/common/src/tx_time_get.c \
smtc_hal_u5/threadx/common/src/tx_time_set.c \
smtc_hal_u5/threadx/common/src/txe_block_allocate.c \
smtc_hal_u5/threadx/common/src/txe_block_pool_create.c \
smtc_hal_u5/threadx/common/src/txe_block_pool_delete.c \
smtc_hal_u5/threadx/common/src/txe_block_pool_info_get.c \
smtc_hal_u5/threadx/common/src/txe_block_pool_prioritize.c \
smtc_hal_u5/threadx/common/src/txe_block_release.c \
smtc_hal_u5/threadx/common/src/txe_byte_allocate.c \
smtc_hal_u5/threadx/common/src/txe_byte_pool_create.c \
smtc_hal_u5/threadx/common/src/txe_byte_pool_delete.c \
smtc_hal_u5/threadx/common/src/txe_byte_pool_info_get.c \
smtc_hal_u5/threadx/common/src/txe_byte_pool_prioritize.c \
smtc_hal_u5/threadx/common/src/txe_byte_release.c \
smtc_hal_u5/threadx/common/src/txe_event_flags_create.c \
smtc_hal_u5/threadx/common/src/txe_event_flags_delete.c \
smtc_hal_u5/threadx/common/src/txe_event_flags_get.c \
smtc_hal_u5/threadx/common/src/txe_event_flags_info_get.c \
smtc_hal_u5/threadx/common/src/txe_event_flags_set.c \
smtc_hal_u5/threadx/common/src/txe_event_flags_set_notify.c \
smtc_hal_u5/threadx/common/src/txe_mutex_create.c \
smtc_hal_u5/threadx/common/src/txe_mutex_delete.c \
smtc_hal_u5/threadx/common/src/txe_mutex_get.c \
smtc_hal_u5/threadx/common/src/txe_mutex_info_get.c \
smtc_hal_u5/threadx/common/src/txe_mutex_prioritize.c \
smtc_hal_u5/threadx/common/src/txe_mutex_put.c \
smtc_hal_u5/threadx/common/src/txe_queue_create.c \
smtc_hal_u5/threadx/common/src/txe_queue_delete.c \
smtc_hal_u5/threadx/common/src/txe_queue_flush.c \
smtc_hal_u5/threadx/common/src/txe_queue_front_send.c \
smtc_hal_u5/threadx/common/src/txe_queue_info_get.c \
smtc_hal_u5/threadx/common/src/txe_queue_prioritize.c \
smtc_hal_u5/threadx/common/src/txe_queue_receive.c \
smtc_hal_u5/threadx/common/src/txe_queue_send.c \
smtc_hal_u5/threadx/common/src/txe_queue_send_notify.c \
smtc_hal_u5/threadx/common/src/txe_semaphore_ceiling_put.c \
smtc_hal_u5/threadx/common/src/txe_semaphore_create.c \
smtc_hal_u5/threadx/common/src/txe_semaphore_delete.c \
smtc_hal_u5/threadx/common/src/txe_semaphore_get.c \
smtc_hal_u5/threadx/common/src/txe_semaphore_info_get.c \
smtc_hal_u5/threadx/common/src/txe_semaphore_prioritize.c \
smtc_hal_u5/threadx/common/src/txe_semaphore_put.c \
smtc_hal_u5/threadx/common/src/txe_semaphore_put_notify.c \
smtc_hal_u5/threadx/common/src/txe_thread_create.c \
smtc_hal_u5/threadx/common/src/txe_thread_delete.c \
smtc_hal_u5/threadx/common/src/txe_thread_entry_exit_notify.c \
smtc_hal_u5/threadx/common/src/txe_thread_info_get.c \
smtc_hal_u5/threadx/common/src/txe_thread_preemption_change.c \
smtc_hal_u5/threadx/common/src/txe_thread_priority_change.c \
smtc_hal_u5/threadx/common/src/txe_thread_relinquish.c \
smtc_hal_u5/threadx/common/src/txe_thread_reset.c \
smtc_hal_u5/threadx/common/src/txe_thread_resume.c \
smtc_hal_u5/threadx/common/src/txe_thread_suspend.c \
smtc_hal_u5/threadx/common/src/txe_thread_terminate.c \
smtc_hal_u5/threadx/common/src/txe_thread_time_slice_change.c \
smtc_hal_u5/threadx/common/src/txe_thread_wait_abort.c \
smtc_hal_u5/threadx/common/src/tx_timer_activate.c \
smtc_hal_u5/threadx/common/src/tx_timer_change.c \
smtc_hal_u5/threadx/common/src/tx_timer_create.c \
smtc_hal_u5/threadx/common/src/tx_timer_deactivate.c \
smtc_hal_u5/threadx/common/src/tx_timer_delete.c \
smtc_hal_u5/threadx/common/src/tx_timer_expiration_process.c \
smtc_hal_u5/threadx/common/src/tx_timer_info_get.c \
smtc_hal_u5/threadx/common/src/tx_timer_initialize.c \
smtc_hal_u5/threadx/common/src/tx_timer_system_activate.c \
smtc_hal_u5/threadx/common/src/tx_timer_system_deactivate.c \
smtc_hal_u5/threadx/common/src/tx_timer_thread_entry.c \
smtc_hal_u5/threadx/common/src/txe_timer_activate.c \
smtc_hal_u5/threadx/common/src/txe_timer_change.c \
smtc_hal_u5/threadx/common/src/txe_timer_create.c \
smtc_hal_u5/threadx/common/src/txe_timer_deactivate.c \
smtc_hal_u5/threadx/common/src/txe_timer_delete.c \
smtc_hal_u5/threadx/common/src/txe_timer_info_get.c \
smtc_hal_u5/threadx/utility/low_power/tx_low_power.c  


BOARD_ASM_SOURCES =  \
	mcu_drivers/core/STM32U5xx/startup_stm32u575xx.s\
	smtc_hal_u5/threadx/tx_initialize_low_level.s\
	smtc_hal_u5/threadx/tx_thread_schedule.s\
	smtc_hal_u5/threadx/tx_thread_stack_build.s\
	smtc_hal_u5/threadx/tx_thread_system_return.s\
	smtc_hal_u5/threadx/tx_timer_interrupt.s

BOARD_C_INCLUDES =  \
	-Imcu_drivers/cmsis/Core/Include\
	-Imcu_drivers/cmsis/Device/ST/STM32U5xx/Include\
	-Imcu_drivers/STM32U5xx_HAL_Driver/Inc\
	-Imcu_drivers/STM32U5xx_HAL_Driver/Inc/Legacy\
	-Ismtc_modem_hal\
	-Iuser_app\
	-Imcu_drivers/core/STM32U5xx\
	-Ismtc_modem_hal\
    -Ismtc_hal_u5/threadx/utility/low_power\
	-Ismtc_hal_u5/threadx/common/inc\
    -Ismtc_hal_u5/threadx/ports/cortex_m33/gnu/inc\
    -Ismtc_hal_u5