# @file
#
# @brief Sets CMake target_sources and target_include_directories
#
# --- The Clear BSD License ---
# Copyright Semtech Corporation 2021. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Semtech corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
# THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
# NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

add_library(lr20xx_driver STATIC
    lr20xx_driver_version.c
    lr20xx_radio_common.c
    lr20xx_radio_fifo.c
    lr20xx_regmem.c
    lr20xx_system.c
    lr20xx_workarounds.c
)

add_library(lr20xx_driver_bluetooth_le STATIC lr20xx_radio_bluetooth_le.c)
add_library(lr20xx_driver_bpsk STATIC lr20xx_radio_bpsk.c)
add_library(lr20xx_driver_flrc STATIC lr20xx_radio_flrc.c)
add_library(lr20xx_driver_fsk STATIC lr20xx_radio_fsk.c)
add_library(lr20xx_driver_lora STATIC lr20xx_radio_lora.c)
add_library(lr20xx_driver_lr_fhss STATIC lr20xx_radio_lr_fhss.c)
add_library(lr20xx_driver_ook STATIC lr20xx_radio_ook.c)
add_library(lr20xx_driver_oqpsk_15_4 STATIC lr20xx_radio_oqpsk_15_4.c)
add_library(lr20xx_driver_wi_sun STATIC lr20xx_radio_wi_sun.c)
add_library(lr20xx_driver_wm_bus STATIC lr20xx_radio_wm_bus.c)
add_library(lr20xx_driver_z_wave STATIC lr20xx_radio_z_wave.c)
add_library(lr20xx_driver_rttof STATIC lr20xx_rttof.c)

add_library(lr20xx_driver::lr20xx_driver ALIAS lr20xx_driver)
add_library(lr20xx_driver::lr20xx_driver_lora ALIAS lr20xx_driver_lora)
add_library(lr20xx_driver::lr20xx_driver_fsk ALIAS lr20xx_driver_fsk)
add_library(lr20xx_driver::lr20xx_driver_bluetooth_le ALIAS lr20xx_driver_bluetooth_le)
add_library(lr20xx_driver::lr20xx_driver_bpsk ALIAS lr20xx_driver_bpsk)
add_library(lr20xx_driver::lr20xx_driver_flrc ALIAS lr20xx_driver_flrc)
add_library(lr20xx_driver::lr20xx_driver_lr_fhss ALIAS lr20xx_driver_lr_fhss)
add_library(lr20xx_driver::lr20xx_driver_ook ALIAS lr20xx_driver_ook)
add_library(lr20xx_driver::lr20xx_driver_oqpsk_15_4 ALIAS lr20xx_driver_oqpsk_15_4)
add_library(lr20xx_driver::lr20xx_driver_wi_sun ALIAS lr20xx_driver_wi_sun)
add_library(lr20xx_driver::lr20xx_driver_wm_bus ALIAS lr20xx_driver_wm_bus)
add_library(lr20xx_driver::lr20xx_driver_z_wave ALIAS lr20xx_driver_z_wave)
add_library(lr20xx_driver::lr20xx_driver_rttof ALIAS lr20xx_driver_rttof)

install(TARGETS
        lr20xx_driver
        lr20xx_driver_lora
        lr20xx_driver_fsk
        lr20xx_driver_bluetooth_le
        lr20xx_driver_bpsk
        lr20xx_driver_flrc
        lr20xx_driver_lr_fhss
        lr20xx_driver_ook
        lr20xx_driver_oqpsk_15_4
        lr20xx_driver_wi_sun
        lr20xx_driver_wm_bus
        lr20xx_driver_z_wave
        lr20xx_driver_rttof
    EXPORT Lr20xxDriverTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
