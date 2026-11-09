# @file      CMakeLists.txt
#
# @brief     CMake file
#
# --- The Clear BSD License ---
# Copyright Semtech Corporation 2021. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of the Semtech corporation nor the
# names of its contributors may be used to endorse or promote products
# derived from this software without specific prior written permission.
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

foreach(radio llcc68 lr11xx lr20xx sx126x sx127x sx128x udp_pf)
    set(library_name smtc_ral_${radio})

    # For udp_pf, add base64.c for packet encoding
    if(${radio} STREQUAL "udp_pf")
        add_library(${library_name} STATIC EXCLUDE_FROM_ALL
            ${CMAKE_CURRENT_SOURCE_DIR}/ral_${radio}.c
            ${CMAKE_CURRENT_SOURCE_DIR}/base64.c
        )
    else()
        add_library(${library_name} STATIC EXCLUDE_FROM_ALL
            ${CMAKE_CURRENT_SOURCE_DIR}/ral_${radio}.c
        )
    endif()

    target_include_directories(${library_name} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:>
    )
    
    # Add radio-specific defines
    if(radio STREQUAL "lr11xx")
        if(DEFINED RAC_RADIO)
            if(RAC_RADIO STREQUAL "lr1110")
                target_compile_definitions(${library_name} PUBLIC LR1110)
            elseif(RAC_RADIO STREQUAL "lr1120")
                target_compile_definitions(${library_name} PUBLIC LR1120)
            elseif(RAC_RADIO STREQUAL "lr1121")
                target_compile_definitions(${library_name} PUBLIC LR1121)
            endif()
        endif()
    elseif(radio STREQUAL "lr20xx")
        if(DEFINED RAC_RADIO AND RAC_RADIO STREQUAL "lr2021")
            target_compile_definitions(${library_name} PUBLIC LR2021)
        elseif(DEFINED RAC_RADIO AND RAC_RADIO STREQUAL "lr2012")
            target_compile_definitions(${library_name} PUBLIC LR2012)
        elseif(DEFINED RAC_RADIO AND RAC_RADIO STREQUAL "lr2022")
            target_compile_definitions(${library_name} PUBLIC LR2022)
        endif()
    elseif(radio STREQUAL "sx127x")
        # ral_sx127x.c selects its supported bandwidths from the chip define
        if(DEFINED RAC_RADIO AND RAC_RADIO STREQUAL "sx1272")
            target_compile_definitions(${library_name} PUBLIC SX1272)
        elseif(DEFINED RAC_RADIO AND RAC_RADIO STREQUAL "sx1276")
            target_compile_definitions(${library_name} PUBLIC SX1276)
        endif()
    elseif(radio STREQUAL "sx126x")
        if(DEFINED RAC_RADIO AND RAC_RADIO STREQUAL "sx1261")
            target_compile_definitions(${library_name} PUBLIC SX1261)
        elseif(DEFINED RAC_RADIO AND RAC_RADIO STREQUAL "sx1262")
            target_compile_definitions(${library_name} PUBLIC SX1262)
        elseif(DEFINED RAC_RADIO AND RAC_RADIO STREQUAL "sx1268")
            target_compile_definitions(${library_name} PUBLIC SX1268)
        endif()
    endif()
    
    install(TARGETS ${library_name} OPTIONAL
        EXPORT SmtcRalTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
    )
    add_library(smtc_ral::${library_name} ALIAS ${library_name})
endforeach(radio)
