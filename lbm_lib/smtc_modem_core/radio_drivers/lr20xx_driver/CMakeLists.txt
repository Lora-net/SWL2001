# @file
#
# @brief CMake library project
#
# --- The Clear BSD License ---
# Copyright Semtech Corporation 2025. All rights reserved.
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

cmake_minimum_required(VERSION 3.25)

project(
    lr20xx_driver
    DESCRIPTION "LR20xx driver library"
    LANGUAGES C
)

add_subdirectory(src)

foreach(target lr20xx_driver lr20xx_driver_lora lr20xx_driver_fsk lr20xx_driver_bluetooth_le lr20xx_driver_bpsk lr20xx_driver_flrc lr20xx_driver_lr_fhss lr20xx_driver_ook lr20xx_driver_oqpsk_15_4 lr20xx_driver_wi_sun lr20xx_driver_wm_bus lr20xx_driver_z_wave lr20xx_driver_rttof)
    target_include_directories(${target} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
        $<INSTALL_INTERFACE:>
    )
endforeach()

install(EXPORT Lr20xxDriverTargets
    FILE Lr20xxDriverConfig.cmake
    NAMESPACE lr20xx_driver::
    DESTINATION lib/cmake/Lr20xx
)

export(TARGETS
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
    NAMESPACE lr20xx_driver:: FILE Lr20xxDriverTargets.cmake)
