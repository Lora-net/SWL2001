# SPDX-License-Identifier: BSD-3-Clause-Clear

set(CMSIS_DIR ${MCU_DRIVERS_DIR}/cmsis)
set(ST_CORE_DIR ${MCU_DRIVERS_DIR}/core/STM32L4xx)
set(ST_HAL_DIR ${MCU_DRIVERS_DIR}/STM32L4xx_HAL_Driver)

add_library(st_hal OBJECT
    ${ST_HAL_DIR}/Src/stm32l4xx_hal_cortex.c
    ${ST_HAL_DIR}/Src/stm32l4xx_hal_dma.c
    ${ST_HAL_DIR}/Src/stm32l4xx_hal_flash_ex.c
    ${ST_HAL_DIR}/Src/stm32l4xx_hal_flash.c
    ${ST_HAL_DIR}/Src/stm32l4xx_hal_gpio.c
    ${ST_HAL_DIR}/Src/stm32l4xx_hal_iwdg.c
    ${ST_HAL_DIR}/Src/stm32l4xx_hal_lptim.c
    ${ST_HAL_DIR}/Src/stm32l4xx_hal_pwr_ex.c
    ${ST_HAL_DIR}/Src/stm32l4xx_hal_pwr.c
    ${ST_HAL_DIR}/Src/stm32l4xx_hal_rcc_ex.c
    ${ST_HAL_DIR}/Src/stm32l4xx_hal_rcc.c
    ${ST_HAL_DIR}/Src/stm32l4xx_hal_rng.c
    ${ST_HAL_DIR}/Src/stm32l4xx_hal_rtc_ex.c
    ${ST_HAL_DIR}/Src/stm32l4xx_hal_rtc.c
    ${ST_HAL_DIR}/Src/stm32l4xx_hal_spi.c
    ${ST_HAL_DIR}/Src/stm32l4xx_hal_uart.c
    ${ST_HAL_DIR}/Src/stm32l4xx_hal_uart_ex.c
    ${ST_HAL_DIR}/Src/stm32l4xx_hal.c
    ${ST_CORE_DIR}/system_stm32l4xx.c
    ${ST_CORE_DIR}/startup_stm32l476xx.s
)

# This is because stm32l4xx_hal_rcc.c:946 triggers a well-known preprocessor warning
target_compile_options(st_hal PRIVATE -Wno-pedantic)

target_compile_definitions(st_hal PRIVATE USE_HAL_DRIVER)
target_include_directories(st_hal PUBLIC
    ${ST_CORE_DIR}
    ${CMSIS_DIR}/Core/Include
    ${CMSIS_DIR}/Device/ST/STM32L4xx/Include
    ${ST_HAL_DIR}/Inc
    ${ST_HAL_DIR}/Inc/Legacy
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

target_link_libraries(smtc_hal PRIVATE st_hal)
