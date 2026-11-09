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

set(LR_FHSS_SRC_PATH ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "Path to folder containing LR-FHSS driver")

add_library(sx127x_driver STATIC
    sx127x.c
)

option(ENABLE_SX1272 "Enable SX1272 driver features" OFF)
option(ENABLE_SX1276 "Enable SX1276 driver features" OFF)

if(ENABLE_SX1272)
    target_compile_definitions(sx127x_driver PRIVATE SX1272)
endif(ENABLE_SX1272)

if(ENABLE_SX1276)
    target_compile_definitions(sx127x_driver PRIVATE SX1276)
endif(ENABLE_SX1276)

add_library(sx127x_driver::sx127x_driver ALIAS sx127x_driver)

target_include_directories(sx127x_driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:>
)

install(TARGETS sx127x_driver
    EXPORT Sx127xDriverTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
