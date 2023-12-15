##############################################################################
# Definitions for the STM32 L476 board
##############################################################################

#-----------------------------------------------------------------------------
# Compilation flags
#-----------------------------------------------------------------------------

#MCU compilation flags
MCU_FLAGS ?= -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfpu=fpv4-sp-d16 -mfloat-abi=hard

BOARD_C_DEFS =  \
	-DUSE_HAL_DRIVER \
	-DSTM32L476xx

BOARD_LDSCRIPT = mcu_drivers/core/STM32L4xx/stm32l476rgtx_flash.ld

#-----------------------------------------------------------------------------
# Hardware-specific sources
#-----------------------------------------------------------------------------
BOARD_C_SOURCES = \
	mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rng.c\
	mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lptim.c \
	mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.c \
	mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.c \
	mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.c \
	mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c \
	mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_iwdg.c \
	mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c \
	mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c \
	mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c \
	mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c \
	mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c \
	mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c \
	mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c \
	mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c \
	mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c \
	mcu_drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c \
	smtc_modem_hal/smtc_modem_hal.c\
	mcu_drivers/core/STM32L4xx/system_stm32l4xx.c\
	smtc_hal_l4/smtc_hal_flash.c\
	smtc_hal_l4/smtc_hal_gpio.c\
	smtc_hal_l4/smtc_hal_mcu.c\
	smtc_hal_l4/smtc_hal_rtc.c\
	smtc_hal_l4/smtc_hal_rng.c\
	smtc_hal_l4/smtc_hal_spi.c\
	smtc_hal_l4/smtc_hal_lp_timer.c\
	smtc_hal_l4/smtc_hal_trace.c\
	smtc_hal_l4/smtc_hal_uart.c\
	smtc_hal_l4/smtc_hal_watchdog.c

BOARD_ASM_SOURCES =  \
	mcu_drivers/core/STM32L4xx/startup_stm32l476xx.s

BOARD_C_INCLUDES =  \
	-Imcu_drivers/core/STM32L4xx\
	-Imcu_drivers/cmsis/Core/Include\
	-Imcu_drivers/cmsis/Device/ST/STM32L4xx/Include\
	-Imcu_drivers/STM32L4xx_HAL_Driver/Inc\
	-Imcu_drivers/STM32L4xx_HAL_Driver/Inc/Legacy\
	-Ismtc_modem_hal\
	-I.\
	-Imcu_drivers/core\
	-Ismtc_modem_hal\
	-Ismtc_hal_l4
