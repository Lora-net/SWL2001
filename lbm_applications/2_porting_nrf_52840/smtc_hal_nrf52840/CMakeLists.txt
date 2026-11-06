# SPDX-License-Identifier: BSD-3-Clause-Clear

set(NRF_SDK_DIR ${MCU_DRIVERS_DIR}/nRF5_SDK_17.1.0_ddde560_Driver)

add_library(nrf_sdk OBJECT
    ${NRF_SDK_DIR}/components/boards/boards.c
    ${NRF_SDK_DIR}/components/libraries/atomic/nrf_atomic.c
    ${NRF_SDK_DIR}/components/libraries/queue/nrf_queue.c
    ${NRF_SDK_DIR}/components/libraries/ringbuf/nrf_ringbuf.c
    ${NRF_SDK_DIR}/components/libraries/util/app_error_handler_gcc.c
    ${NRF_SDK_DIR}/components/libraries/util/app_error_weak.c
    ${NRF_SDK_DIR}/components/libraries/util/app_error.c
    ${NRF_SDK_DIR}/components/libraries/util/app_util_platform.c
    ${NRF_SDK_DIR}/components/libraries/util/nrf_assert.c
    ${NRF_SDK_DIR}/integration/nrfx/legacy/nrf_drv_clock.c
    ${NRF_SDK_DIR}/integration/nrfx/legacy/nrf_drv_rng.c
    ${NRF_SDK_DIR}/integration/nrfx/legacy/nrf_drv_spi.c
    ${NRF_SDK_DIR}/integration/nrfx/legacy/nrf_drv_uart.c
    ${NRF_SDK_DIR}/modules/nrfx/drivers/src/nrfx_clock.c
    ${NRF_SDK_DIR}/modules/nrfx/drivers/src/nrfx_gpiote.c
    ${NRF_SDK_DIR}/modules/nrfx/drivers/src/nrfx_nvmc.c
    ${NRF_SDK_DIR}/modules/nrfx/drivers/src/nrfx_rng.c
    ${NRF_SDK_DIR}/modules/nrfx/drivers/src/nrfx_rtc.c
    ${NRF_SDK_DIR}/modules/nrfx/drivers/src/nrfx_spi.c
    ${NRF_SDK_DIR}/modules/nrfx/drivers/src/nrfx_uart.c
    ${NRF_SDK_DIR}/modules/nrfx/drivers/src/nrfx_uarte.c
    ${NRF_SDK_DIR}/modules/nrfx/drivers/src/nrfx_wdt.c
    ${NRF_SDK_DIR}/modules/nrfx/drivers/src/prs/nrfx_prs.c
    ${NRF_SDK_DIR}/modules/nrfx/hal/nrf_nvmc.c
    ${NRF_SDK_DIR}/modules/nrfx/mdk/system_nrf52840.c
    ${NRF_SDK_DIR}/modules/nrfx/soc/nrfx_atomic.c
    ${NRF_SDK_DIR}/modules/nrfx/mdk/gcc_startup_nrf52840.s
)

target_link_libraries(nrf_sdk PRIVATE m)


target_compile_definitions(nrf_sdk PUBLIC
    NRF52840_XXAA
    BOARD_PCA10056
    BSP_DEFINES_ONLY
    CONFIG_GPIO_AS_PINRESET
    FLOAT_ABI_HARD
    DEBUG
    __HEAP_SIZE=8192
    __STACK_SIZE=8192
)
target_compile_options(nrf_sdk PUBLIC "-Wno-expansion-to-defined")
target_compile_options(nrf_sdk PRIVATE "-Wno-pedantic")

target_include_directories(nrf_sdk PUBLIC
    ${NRF_SDK_DIR}/components
    ${NRF_SDK_DIR}/components/boards
    ${NRF_SDK_DIR}/components/drivers_nrf/nrf_soc_nosd
    ${NRF_SDK_DIR}/components/libraries/atomic
    ${NRF_SDK_DIR}/components/libraries/bsp
    ${NRF_SDK_DIR}/components/libraries/delay
    ${NRF_SDK_DIR}/components/libraries/experimental_section_vars
    ${NRF_SDK_DIR}/components/libraries/log
    ${NRF_SDK_DIR}/components/libraries/log/src
    ${NRF_SDK_DIR}/components/libraries/memobj
    ${NRF_SDK_DIR}/components/libraries/queue
    ${NRF_SDK_DIR}/components/libraries/ringbuf
    ${NRF_SDK_DIR}/components/libraries/strerror
    ${NRF_SDK_DIR}/components/libraries/uart
    ${NRF_SDK_DIR}/components/libraries/util
    ${NRF_SDK_DIR}/components/toolchain/cmsis/include
    ${NRF_SDK_DIR}/integration/nrfx
    ${NRF_SDK_DIR}/integration/nrfx/legacy
    ${NRF_SDK_DIR}/modules/nrfx
    ${NRF_SDK_DIR}/modules/nrfx/drivers/include
    ${NRF_SDK_DIR}/modules/nrfx/hal
    ${NRF_SDK_DIR}/modules/nrfx/mdk
    ${CMAKE_CURRENT_LIST_DIR}
)

# For tx_user.h
# target_include_directories(st_hal PRIVATE
#     ${CMAKE_CURRENT_LIST_DIR}/../main_examples
# )

add_library(smtc_hal STATIC
    smtc_hal_flash.c
    smtc_hal_gpio.c
    smtc_hal_mcu.c
    smtc_hal_rng.c
    smtc_hal_rtc.c
    smtc_hal_spi.c
    smtc_hal_trace.c
    smtc_hal_uart.c
    smtc_hal_watchdog.c
    smtc_hal_libc_stub.c
)

target_include_directories(smtc_hal PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
)

target_link_libraries(smtc_hal PUBLIC nrf_sdk smtc_modem_hal_interface)
