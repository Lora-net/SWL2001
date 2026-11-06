# SPDX-License-Identifier: BSD-3-Clause-Clear

set(CMSIS_DIR ${MCU_DRIVERS_DIR}/cmsis)
set(ST_CORE_DIR ${MCU_DRIVERS_DIR}/core/STM32L0xx)
set(ST_HAL_DIR ${MCU_DRIVERS_DIR}/STM32L0xx_HAL_Driver)

add_library(st_hal OBJECT
    ${ST_HAL_DIR}/Src/stm32l0xx_ll_usart.c
    ${ST_HAL_DIR}/Src/stm32l0xx_ll_rcc.c
    ${ST_HAL_DIR}/Src/stm32l0xx_ll_lptim.c
    ${ST_HAL_DIR}/Src/stm32l0xx_ll_rtc.c
    ${ST_HAL_DIR}/Src/stm32l0xx_ll_dma.c
    ${ST_HAL_DIR}/Src/stm32l0xx_ll_utils.c

    ${ST_CORE_DIR}/system_stm32l0xx.c
    ${ST_CORE_DIR}/startup_stm32l073xx.s
)

target_compile_definitions(st_hal PRIVATE USE_HAL_DRIVER)
target_include_directories(st_hal PUBLIC
    ${ST_CORE_DIR}
    ${CMSIS_DIR}/Core/Include
    ${CMSIS_DIR}/Device/ST/STM32L0xx/Include
    ${ST_HAL_DIR}/Inc
    ${ST_HAL_DIR}/Inc/Legacy
)

add_library(smtc_hal STATIC
    smtc_hal_eeprom.c
    smtc_hal_gpio.c
    smtc_hal_libc_stub.c
    smtc_hal_lp_timer.c
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

target_link_libraries(smtc_hal PRIVATE st_hal)

set(HAL_FLASH_UNAVAILABLE ON PARENT_SCOPE)
