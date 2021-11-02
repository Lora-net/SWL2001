##############################################################################
# Definitions for the STM32 L476 board
##############################################################################


#-----------------------------------------------------------------------------
# Compilation flags
#-----------------------------------------------------------------------------
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard

MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

BOARD_C_DEFS =  \
	-DUSE_HAL_DRIVER \
	-DSTM32L476xx

BOARD_LDSCRIPT = user_app/mcu_drivers/core/stm32l476rgtx_flash.ld

#-----------------------------------------------------------------------------
# Hardware-specific sources
#-----------------------------------------------------------------------------
BOARD_C_SOURCES = \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rng.c\
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lptim.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_wwdg.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_iwdg.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc.c \
	user_app/mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc_ex.c\
	user_app/smtc_modem_hal/smtc_modem_hal.c\
	user_app/mcu_drivers/core/system_stm32l4xx.c\
	user_app/smtc_hal_l4/smtc_hal_adc.c\
	user_app/smtc_hal_l4/smtc_hal_flash.c\
	user_app/smtc_hal_l4/smtc_hal_gpio.c\
	user_app/smtc_hal_l4/smtc_hal_mcu.c\
	user_app/smtc_hal_l4/smtc_hal_rtc.c\
	user_app/smtc_hal_l4/smtc_hal_rng.c\
	user_app/smtc_hal_l4/smtc_hal_spi.c\
	user_app/smtc_hal_l4/smtc_hal_lp_timer.c\
	user_app/smtc_hal_l4/smtc_hal_trace.c\
	user_app/smtc_hal_l4/smtc_hal_uart.c\
	user_app/smtc_hal_l4/smtc_hal_watchdog.c

BOARD_ASM_SOURCES =  \
	user_app/mcu_drivers/core/startup_stm32l476xx.s

BOARD_C_INCLUDES =  \
	-Iuser_app/mcu_drivers/cmsis/Core/Include\
	-Iuser_app/mcu_drivers/cmsis/Device/ST/STM32L4xx/Include\
	-Iuser_app/mcu_drivers/STM32L4xx_HAL_Driver/Inc\
	-Iuser_app/mcu_drivers/STM32L4xx_HAL_Driver/Inc/Legacy\
	-Ismtc_modem_hal\
	-Iuser_app\
	-Iuser_app/littlefs\
	-Iuser_app/mcu_drivers/core\
	-Iuser_app/smtc_modem_hal\
	-Iuser_app/modem_config\
	-Iuser_app/smtc_hal_l4

