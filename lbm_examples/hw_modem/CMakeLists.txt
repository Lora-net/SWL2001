# SPDX-License-Identifier: BSD-3-Clause-Clear

target_sources(lbm_example.elf PRIVATE
    main_${APP}.c
    cmd_parser.c
    git_version.c
    hw_modem.c
)

if(LBM_GEOLOCATION)
    target_sources(lbm_example.elf PRIVATE geoloc_bsp.c)
endif()
