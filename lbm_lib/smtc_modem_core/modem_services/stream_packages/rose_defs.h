/*!
 * \file      rose_defs.h
 *
 * \brief     RELIABLE OCTET STREAM ENCODING (ROSE) definitions
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

enum
{
    ROSE_CRYPT_DIR = 0x42,
};

#define MAX_SYSC 0x7E  // 7F - reserved, identifies SINFO message
#define SDATA_HDR_OFF 0
#define SDATA_SOFFL_OFF 1
#define SDATA_PCTX_FLAG 0x80
#define SDATA_HDR_LEN 3
#define SDATA_PCTX_LEN 3

#define SCMD_FLAGS_SCMD 0x01
#define SCMD_FLAGS_SINFO 0x02
#define SCMD_FLAGS_ACKWL 0x04
#define SCMD_FLAGS_UPDWL 0x08
#define SCMD_FLAGS_UPDRR 0x10
#define SCMD_FLAGS_UPDPCI 0x20
#define SCMD_FLAGS_OFF 0
#define SCMD_WL_OFF 1
#define SCMD_RR_OFF 2
#define SCMD_PCTXINTV_OFF 3
#define SCMD_LEN 4

#define SINFO_HDR_VALUE 0xFF
#define SINFO_FLAGS_RQAWL 0x01  // ACK
#define SINFO_FLAGS_USZ_SHIFT 1
#define SINFO_HDR_OFF 0
#define SINFO_FLAGS_OFF 1
#define SINFO_WL_OFF 2
#define SINFO_RR_OFF 3
#define SINFO_SOFFL_OFF 4
#define SINFO_SOFFH_OFF 6
#define SINFO_PCTXINTV_OFF 8
#define SINFO_LEN 9

#define REC_TAG 0xA0
