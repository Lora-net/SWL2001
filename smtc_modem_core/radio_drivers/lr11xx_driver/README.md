# LR11xx driver

This package proposes an implementation in C of the driver for **LR11XX** radio component.

## Components

The driver is split in several components:

- Bootloader
- Register / memory access
- System configuration
- Radio
- Wi-Fi Passive Scanning
- GNSS Scan Scanning
- Crypto engine

### Bootloader

This component is used to update the firmware.

### Register / memory access

This component is used to read / write data from registers or internal memory.

### System configuration

This component is used to interact with system-wide parameters like clock sources, integrated RF switches, etc.

### Radio

This component is used to send / receive data through the different modems (LoRa and GFSK) or perform a LoRa CAD (Channel Activity Detection). Parameters like power amplifier selection, output power and fallback modes are also accessible through this component.

### Wi-Fi Passive Scanning

This component is used to configure and initiate the passive scanning of the Wi-Fi signals that can be shared to request a geolocation.

### GNSS Scanning

This component is used to configure and initiate the acquisition of GNSS signals that can be shared to request a geolocation.

### Crypto engine

This component is used to set and derive keys in the internal keychain and perform cryptographic operations with the integrated hardware accelerator.

## Structure

Each component is based on different files:

- lr11xx_component.c: implementation of the functions related to component
- lr11xx_component.h: declarations of the functions related to component
- lr11xx_component_types.h: type definitions related to components

## HAL

The HAL (Hardware Abstraction Layer) is a collection of functions that the user shall implement to write platform-dependent calls to the host. The list of functions is the following:

- lr11xx_hal_reset()
- lr11xx_hal_wakeup()
- lr11xx_hal_write()
- lr11xx_hal_read()
- lr11xx_hal_direct_read()

## LR11xx firmware known limitations

### High ACP (Adjacent Channel Power)

#### Description

When the chip wakes up from sleep mode with retention, a parameter is not reconfigured properly. This misconfiguration can lead to an unexpectedly high adjacent channel power in all subsequent transmissions.

The issue appears only in LoRa modulation, for all bandwidths except for 500kHz and 800kHz.

The following firmware versions are affected:

- LR1110 firmware from 0x0303 to 0x0307
- LR1120 firmware 0x0101

### Workaround

The workaround is to reset the bit 30 in the register `0x00F30054` when the chip wakes up from sleep mode with retention.

This workaround does not solve the case where `LR11XX_RADIO_MODE_SLEEP` is configured with `lr11xx_radio_auto_tx_rx` and the chip is set to Rx mode. This is dues to the fact that the workaround cannot be applied before the subsequent transmission, automatically launched by the chip after waking up from sleep mode with retention.

#### Implementation 1

The first implementation - enabled by default in the driver - adds an implicit call updating the parameter to each function that could set the chip in transmission - directly or not -:

- `lr11xx_radio_set_tx_with_timeout_in_rtc_step`
- `lr11xx_radio_set_tx_infinite_preamble`
- `lr11xx_radio_set_rx_with_timeout_in_rtc_step` - in case `lr11xx_radio_auto_tx_rx` has been enabled
- `lr11xx_radio_set_cad` - in case `LR11XX_RADIO_CAD_EXIT_MODE_TX` has been set with `lr11xx_radio_set_cad_params`

This implementation can be disabled by defining the macro `LR11XX_DISABLE_HIGH_ACP_WORKAROUND`. This disabling will be useful when, in the future, a new firmware integrating a fix is released and does not require the workaround anymore.

The main advantage of this implementation is that it is transparent to the user who only needs to update the driver without changing its application. The main drawback is that the implicit call is done systematically even when not required.

#### Implementation 2

The second method requires the user to explicitly call the function `lr11xx_radio_apply_high_acp_workaround` when the chip wakes up from sleep mode with retention (note: to ease the implementation, it can be called when the chip wakes up from any sleep mode).

This method requires the macro `LR11XX_DISABLE_HIGH_ACP_WORKAROUND` to be defined so the implementation 1 of the workaround (enabled by default) is disabled.
