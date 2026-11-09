# @file
#
# @brief Sets CMake target_sources and target_include_directories
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

option(SX126X_ENABLE_BPSK "Enable BPSK in build" OFF)
option(SX126X_ENABLE_LR_FHSS "Enable LR-FHSS in build" OFF)

set(LR_FHSS_SRC_PATH ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "Path to folder containing LR-FHSS driver")

set(LIBRARY_SOURCES)
list(APPEND LIBRARY_SOURCES
    sx126x_driver_version.c
    sx126x.c
    $<$<BOOL:${SX126X_ENABLE_BPSK}>:sx126x_bpsk.c>
    $<$<BOOL:${SX126X_ENABLE_LR_FHSS}>:sx126x_lr_fhss.c>
    $<$<BOOL:${SX126X_ENABLE_LR_FHSS}>:${LR_FHSS_SRC_PATH}/lr_fhss_mac.c>
)

add_library(sx126x_driver STATIC ${LIBRARY_SOURCES})

add_library(sx126x_driver::sx126x_driver ALIAS sx126x_driver)

target_include_directories(sx126x_driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${LR_FHSS_SRC_PATH}>
    $<INSTALL_INTERFACE:>
)

install(TARGETS sx126x_driver
    EXPORT Sx126xDriverTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
