/**
 * @file      sx128x_regs.h
 *
 * @brief     SX128X register definitions
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

#ifndef SX128X_REGS_H
#define SX128X_REGS_H

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Firmware version value register. (16 bits)
 */
#define SX128X_REG_FW_VERSION 0x01F0

/**
 * @brief GFSK CRC seed value register. (16 bits)
 */
#define SX128X_REG_GFSK_CRC_SEED 0x09C8

/**
 * @brief GFSK polynomial value register. (16 bits)
 */
#define SX128X_REG_GFSK_CRC_POLYNOM 0x09C6

/**
 * @brief GFSK or BLE whitening seed value register. (8 bits)
 */
#define SX128X_REG_GFSK_BLE_WHITENING_SEED 0x09C5

/**
 * @brief FLRC CRC seed value register. (32 bits)
 */
#define SX128X_REG_FLRC_CRC_SEED 0x09C6

/**
 * @brief BLE CRC seed value register. (24 bits)
 */
#define SX128X_REG_BLE_CRC_SEED 0x9C7

/**
 * @brief Ranging ID length check control register. (8 bits)
 */
#define SX128X_REG_RANGING_ID_LEN_CHECK 0x0931

/**
 * @brief Ranging slave req address register. (32 bits)
 */
#define SX128X_REG_RANGING_SLV_REQ_ADDRESS 0x0916

/**
 * @brief Ranging master req address register. (32 bits)
 */
#define SX128X_REG_RANGING_MST_REQ_ADDRESS 0x0912

/**
 * @brief Ranging calibration register. (16 bits)
 */
#define SX128X_REG_RANGING_CALIBRATION 0x092C

/**
 * @brief Ranging results configuration register. (8 bits)
 */
#define SX128X_REG_RANGING_RESULTS_CFG 0x0924

/**
 * @brief Bit shift for ranging results configuration register.
 */
#define SX128X_REG_RANGING_RESULTS_CFG_POS ( 4U )

/**
 * @brief Bit mask for ranging results configuration register.
 */
#define SX128X_REG_RANGING_RESULTS_CFG_MASK ( 0x03UL << SX128X_REG_RANGING_RESULTS_CFG_POS )

/**
 * @brief LoRa SF additional configuration register. (8 bits)
 */
#define SX128X_REG_LORA_SF_CFG0 0x0925

/**
 * @brief LoRa frequency error correction register (3 bits)
 */
#define SX128X_REG_LORA_FREQ_ERR_CORR 0x093C

/**
 * @brief Bit shift for LoRa frequency error correction register.
 */
#define SX128X_REG_LORA_FREQ_ERR_CORR_POS ( 0U )

/**
 * @brief Bit mask for LoRa frequency error correction register
 */
#define SX128X_REG_LORA_FREQ_ERR_CORR_MASK ( 0x07UL << SX128X_REG_LORA_FREQ_ERR_CORR_POS )

/**
 * @brief Bit mask for LoRa frequency error correction register mode
 */
#define SX128X_REG_LORA_FREQ_ERR_CORR_MODE_1 ( 0x01UL << SX128X_REG_LORA_FREQ_ERR_CORR_POS )

/**
 * @brief Raging results register. (24 bits)
 */
#define SX128X_REG_RANGING_RESULTS 0x0961

/**
 * @brief Raging results freeze control register. (8 bits)
 */
#define SX128X_REG_RANGING_RESULTS_FREEZE 0x097F

/**
 * @brief Ranging Rx/Tx delay calibration value register. (24 bits)
 */
#define SX128X_REG_RANGING_RX_TX_DELAY_CAL 0x092B

/**
 * @brief Ranging sliding filter window size ragister. (8 bits)
 */
#define SX128X_REG_RANGING_SLD_FILTER_WINDOW_SIZE 0x091E

/**
 * @brief Ranging sliding filter reset register. (8 bits)
 */
#define SX128X_REG_RANGING_SLD_FILTER_RST 0x0923

/**
 * @brief The RSSI value of the last ranging exchange
 */
#define SX128X_REG_RANGING_RSSI 0x0964

/**
 * @brief Indicates the LoRa modem header mode. 0=Header, 1=No header
 */
#define SX128X_REG_LORA_HEADER_MODE 0x903

/**
 * @brief LoRa payload length value register. May be used when using implicit header
 * mode to retrieve the previously-configured payload length.
 */
#define SX128X_REG_LR_PAYLOAD_LENGTH 0x901

/**
 * @brief GFSK sync word 1 value register. (40 bits)
 */
#define SX128X_REG_GFSK_SYNC_WORD_1 0x09CE

/**
 * @brief GFSK sync word 2 value register. (40 bits)
 */
#define SX128X_REG_GFSK_SYNC_WORD_2 0x09D3

/**
 * @brief GFSK sync word 3 value register. (40 bits)
 */
#define SX128X_REG_GFSK_SYNC_WORD_3 0x09D8

/**
 * @brief FLRC sync word 1 value register. (32 bits)
 */
#define SX128X_REG_FLRC_SYNC_WORD_1 0x09CF

/**
 * @brief FLRC sync word 2 value register. (32 bits)
 */
#define SX128X_REG_FLRC_SYNC_WORD_2 0x09D4

/**
 * @brief FLRC sync word 3 value register. (32 bits)
 */
#define SX128X_REG_FLRC_SYNC_WORD_3 0x09D9

/**
 * @brief BLE sync word value register. (32 bits)
 */
#define SX128X_REG_BLE_SYNC_WORD 0x09CF

/**
 * @brief Defines how many bit errors are tolerated in sync word detection
 */
#define SX128X_REG_GFSK_FLRC_SYNC_WORD_TOLERANCE 0x09CD

/**
 * @brief The addresses of the register holding LoRa Modem SyncWord value
 *     0x1424: LoRaWAN private network,
 *     0x3444: LoRaWAN public network
 */
#define SX128X_REG_LR_SYNC_WORD 0x0944

/**
 * @brief LoRa Frequency Error Indicator (FEI) register. (20 bits signed - 3 bytes)
 */
#define SX128X_REG_LR_FEI 0x0954

/**
 * @brief Register for MSB Access Address (BLE)
 */
#define SX128X_REG_BLE_ACCESS_ADDRESS 0x09CF

/**
 * @brief Register address and mask for LNA regime selection
 */
#define SX128X_REG_LNA_REGIME 0x0891

/**
 * @brief Manual Gain Control control register
 */
#define SX128X_REG_MGC_CTRL_1 0x089F

/**
 * @brief Register and mask controlling demodulation detection
 */
#define SX128X_REG_MGC_CTRL_2 0x0895

/**
 * @brief Manual Gain Control value register
 */
#define SX128X_REG_MGC_VALUE 0x089E

/**
 * @brief LoRa incoming packet coding rate register
 */
#define SX128X_REG_LR_INCOMING_CR 0x0950

/**
 * @brief Bit shift for LoRa incoming packet coding rate register
 */
#define SX128X_REG_LR_INCOMING_CR_POS ( 4U )

/**
 * @brief Bit mask for LoRa incoming packet coding rate register
 */
#define SX128X_REG_LR_INCOMING_CR_MASK ( 0x07UL << SX128X_REG_LR_INCOMING_CR_POS )

/**
 * @brief LoRa incoming packet coding rate register
 */
#define SX128X_REG_LR_INCOMING_CRC 0x0954

/**
 * @brief Bit shift for LoRa incoming packet coding rate register
 */
#define SX128X_REG_LR_INCOMING_CRC_POS ( 4U )

/**
 * @brief Bit mask for LoRa incoming packet coding rate register
 */
#define SX128X_REG_LR_INCOMING_CRC_MASK ( 0x01 << SX128X_REG_LR_INCOMING_CRC_POS )

/**
 * @brief LoRa incoming packet coding rate register
 */
#define SX128X_REG_LR_IQ_CONF 0x093B

/**
 * @brief The address where the blob SX128X_REG_RSSI_SNR_BUGFIX_BLOB must be written
 * after wakeup to fix the "rssi is 0 after a sleep" bug
 */
#define SX128X_REG_RSSI_SNR_BUGFIX_ADDRESS 0x00EF

/**
 * @brief The blob that must be written to SX128X_REG_RSSI_SNR_BUGFIX_ADDRESS after
 * wakeup to fix the "rssi is 0 after a sleep" bug
 */
#define SX128X_REG_RSSI_SNR_BUGFIX_BLOB \
    {                                   \
        0x00, 0xad, 0x08, 0x9a          \
    }

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#endif  // SX128X_REGS_H

/* --- EOF ------------------------------------------------------------------ */
