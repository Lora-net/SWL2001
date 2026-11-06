# SPDX-License-Identifier: BSD-3-Clause-Clear

target_sources(lbm_example.elf PRIVATE
    app_azure_rtos.c
    app_threadx.c
    cmd_parser.c
    hw_modem.c
    main_thread_x.c
)


if(APP STREQUAL porting_tests)
    option(TEST_FLASH "Enable Flash tests (but disable other porting tests)")
    if(TEST_FLASH)
        target_compile_definitions(lbm_example.elf ENABLE_TEST_FLASH)
    endif()
endif()
