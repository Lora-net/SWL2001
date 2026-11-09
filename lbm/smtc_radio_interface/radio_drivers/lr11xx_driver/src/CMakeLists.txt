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

add_library(lr11xx_driver STATIC
    lr11xx_bootloader.c
    lr11xx_crypto_engine.c
    lr11xx_driver_version.c
    lr11xx_lr_fhss.c
    lr11xx_radio_timings.c
    lr11xx_radio.c
    lr11xx_regmem.c
    lr11xx_rttof.c
    lr11xx_system.c
)

if(LR11XX_DRIVER_GEOLOCATION)
    target_sources(lr11xx_driver PRIVATE
        lr11xx_gnss.c
        lr11xx_wifi.c
    )
endif()

add_library(lr11xx_driver::lr11xx_driver ALIAS lr11xx_driver)
    
target_include_directories(lr11xx_driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:>
)

install(TARGETS lr11xx_driver
    EXPORT Lr11xxDriverTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
