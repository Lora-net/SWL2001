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

BOARD_LDSCRIPT = user_app/mcu_drivers/core/STM32L0xx/stm32l073xx_flash.ld

#-----------------------------------------------------------------------------
# Hardware-specific sources
#-----------------------------------------------------------------------------
BOARD_C_SOURCES = \
	user_app/mcu_drivers/core/STM32L0xx/system_stm32l0xx.c

BOARD_C_SOURCES += \
	user_app/smtc_hal_l0_LL/smtc_hal_watchdog.c\
	user_app/smtc_hal_l0_LL/smtc_hal_spi.c\
	user_app/smtc_hal_l0/smtc_hal_rng.c\
	user_app/smtc_hal_l0/smtc_hal_eeprom.c\
	user_app/smtc_hal_l0/smtc_hal_gpio.c\
	user_app/smtc_hal_l0/smtc_hal_uart.c\
	user_app/smtc_hal_l0/smtc_hal_lp_timer.c\
	user_app/smtc_hal_l0/smtc_hal_rtc.c\
	user_app/smtc_hal_l0/smtc_hal_trace.c\
	user_app/smtc_hal_l0/smtc_hal_mcu.c\
	user_app/mcu_drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_usart.c \
	user_app/mcu_drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_rcc.c \
	user_app/mcu_drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_lptim.c \
	user_app/mcu_drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_rtc.c \
	user_app/mcu_drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_dma.c \
	user_app/mcu_drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_utils.c \
	user_app/smtc_modem_hal/smtc_modem_hal.c

BOARD_ASM_SOURCES =  \
	user_app/mcu_drivers/core/STM32L0xx/startup_stm32l073xx.s

BOARD_C_INCLUDES =  \
	-Iuser_app/mcu_drivers/core/STM32L0xx\
	-Iuser_app/mcu_drivers/cmsis/Core/Include\
	-Iuser_app/mcu_drivers/cmsis/Device/ST/STM32L0xx/Include\
	-Iuser_app/mcu_drivers/STM32L0xx_HAL_Driver/Inc\
	-Iuser_app/mcu_drivers/STM32L0xx_HAL_Driver/Inc/Legacy

BOARD_C_INCLUDES +=  \
	-Iuser_app/smtc_hal_l0_LL

#STM32l0 is slower than STM32L4 and wake up delay is longer so radio planner shall have a bigger delay (wake up time approc 4ms, stm lp timer delay = 4ms, and a security margin of 2 ms)
LBM_FLAGS="-DRP_MARGIN_DELAY=10"
