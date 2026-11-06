# SPDX-License-Identifier: BSD-3-Clause-Clear

target_sources(lbm_example.elf PRIVATE
    main_${APP}.c
)

if(APP STREQUAL porting_tests)
    option(TEST_FLASH "Enable Flash tests (but disable other porting tests)")
    if(TEST_FLASH)
        target_compile_definitions(lbm_example.elf ENABLE_TEST_FLASH)
    endif()
endif()
