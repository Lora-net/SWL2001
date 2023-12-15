##############################################################################
# Definitions for the STM32 L073 board
##############################################################################


#-----------------------------------------------------------------------------
# Compilation flags
#-----------------------------------------------------------------------------
#MCU compilation flags
MCU_FLAGS ?= -mcpu=cortex-m0plus -mabi=aapcs -mthumb

BOARD_C_DEFS =  \
	-DSTM32L073xx

BOARD_C_DEFS +=  \
	-DUSE_FULL_LL_DRIVER

BOARD_LDSCRIPT = mcu_drivers/core/STM32L0xx/stm32l073xx_flash.ld

#-----------------------------------------------------------------------------
# Hardware-specific sources
#-----------------------------------------------------------------------------
BOARD_C_SOURCES = \
	mcu_drivers/core/STM32L0xx/system_stm32l0xx.c

BOARD_C_SOURCES += \
	smtc_hal_l0_LL/smtc_hal_watchdog.c\
	smtc_hal_l0_LL/smtc_hal_spi.c\
	smtc_hal_l0/smtc_hal_rng.c\
	smtc_hal_l0/smtc_hal_eeprom.c\
	smtc_hal_l0/smtc_hal_gpio.c\
	smtc_hal_l0/smtc_hal_uart.c\
	smtc_hal_l0/smtc_hal_lp_timer.c\
	smtc_hal_l0/smtc_hal_rtc.c\
	smtc_hal_l0/smtc_hal_trace.c\
	smtc_hal_l0/smtc_hal_mcu.c\
	mcu_drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_usart.c \
	mcu_drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_rcc.c \
	mcu_drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_lptim.c \
	mcu_drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_rtc.c \
	mcu_drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_dma.c \
	mcu_drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_utils.c \
	smtc_modem_hal/smtc_modem_hal.c

BOARD_ASM_SOURCES =  \
	mcu_drivers/core/STM32L0xx/startup_stm32l073xx.s

BOARD_C_INCLUDES =  \
	-Imcu_drivers/core/STM32L0xx\
	-Imcu_drivers/cmsis/Core/Include\
	-Imcu_drivers/cmsis/Device/ST/STM32L0xx/Include\
	-Imcu_drivers/STM32L0xx_HAL_Driver/Inc\
	-Imcu_drivers/STM32L0xx_HAL_Driver/Inc/Legacy

BOARD_C_INCLUDES +=  \
	-Ismtc_hal_l0_LL

#STM32l0 is slower than STM32L4 and wake up delay is longer so radio planner shall have a bigger delay (wake up time approc 4ms, stm lp timer delay = 4ms, and a security margin of 2 ms)
LBM_FLAGS="-DRP_MARGIN_DELAY=10"
