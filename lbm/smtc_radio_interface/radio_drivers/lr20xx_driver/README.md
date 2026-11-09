# LR20XX driver

This package proposes an implementation in C of the driver for **LR20XX** radio component.

## Components

The driver is split in multiple components.
Each component is based on different files:

- lr20xx_component_name.c: implementation of the functions related to *component_name*, located in `src/` folder
- lr20xx_component_name.h: declarations of the functions related to *component_name*, located in `inc/` folder
- lr20xx_component_name_types.h: type definitions related to *component_name*, located in `inc/` folder

## HAL

The HAL (Hardware Abstraction Layer) is a collection of functions that the user shall implement to write platform-dependent calls to the host. The HAL functions are declared in [lr20xx_hal.h](inc/lr20xx_hal.h):

- lr20xx_hal_reset()
- lr20xx_hal_wakeup()
- lr20xx_hal_write()
- lr20xx_hal_read()
- lr20xx_hal_direct_read()
- lr20xx_hal_direct_read_fifo()

## Limitations

The LR20xx limitations and specific configurations are addressed either through patch RAM (PRAM) usage, or through workaround driver functions.

### Limitation covered by PRAM usage

This sections provides details concerning the limitations that are fixed by using the provided PRAM.

Driver functions to load and enable PRAMs are provided in file [lr20xx_pram.h](inc/lr20xx_pram.h):

- `lr20xx_pram_load_pram_lr2021` to load Patch RAM specific to LR2021 chip; and
- `lr20xx_pram_load_pram_lr20x2` to load Patch RAM specific to LR20x2 chips.

The PRAM must be loaded:

- after LR20xx reset; and
- after leaving a sleep mode without retention (refer to `lr20xx_system_set_sleep_mode` function).

The following sub-sections list the limitations that are fixed by using the PRAMs.

#### Bluetooth LE Coded PHY Access Address

When configuring Bluetooth LE modulation or packet with `phy` value being `LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_500KB` or `LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_125KB`, some access address may fail to be transmitted and received correctly, resulting in degraded packet error rate.

#### Bluetooth LE Coded PHY Frequency drift

When configuring Bluetooth LE modulation or packet with `phy` value being `LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_500KB` or `LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_125KB`, the default frequency drift may negatively impact sensitivity when used with high frequency drift transmitters.

#### Bluetooth LE 2Mbps preamble length

The default preamble length for Bluetooth LE mode phy `LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_2M` is incorrect.

#### Bluetooth LE Coded PHY blocking RF performance

Degraded RF blocking performance may be observed with Bluetooth LE Coded PHY `LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_500KB` and `LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_125KB`.

#### RTToF PLL frequency step

Biased RTToF results may be observed if the RF frequency configured is not a multiple of 122Hz.

#### RTToF RSSI computation

The RSSI value returned by the chip can be incorrect.

#### RTToF Extended mode stuck

When executing RTToF operations configured with `LR20XX_RTTOF_MODE_EXTENDED`, it may be observed that the manager stays stuck for few seconds (duration depends on the configured bandwidth).

#### DCDC (SIMO) impact on sensitivity

Usage of DCDC regulator (a.k.a. SIMO converter) may negatively impact RF sensitivity for sub-GHz operations for the following modulations:
- FSK
- FLRC
- OOK
- LoRa
- Z-Wave

#### LoRa Rx single stuck

Reception in LoRa Rx single mode can be stuck if the payload length indicated in the received header is larger than the one configured in *Set Packet Parameters* command.

### Limitation covered by workaround functions

Workarounds are defined in files [lr20xx_workarounds.h](inc/lr20xx_workarounds.h) and [lr20xx_workarounds.c](src/lr20xx_workarounds.c) when appropriate.

#### SX1276 LoRa compatibility mode

When SX1276 LoRa compatibility is required, the workaround `lr20xx_workarounds_lora_enable_sx1276_compatibility_mode` must be called. It can be disabled afterwards by calling `lr20xx_workarounds_lora_disable_sx1276_compatibility_mode`.

By default this configuration is not retain in memory when entering sleep mode. The function `lr20xx_workarounds_lora_sx1276_compatibility_mode_store_retention_mem` allows to store the SX1276 LoRa compatible state in a retention memory slot.

#### SX1276 LoRa intra-packet frequency hopping mode

When LoRa intra-packet frequency hopping is required to be compatible with SX1276, the workaround `lr20xx_workarounds_lora_freq_hop_enable_sx1276_compatibility_mode` must be called after `lr20xx_radio_lora_set_freq_hop`. It can be disabled afterwards by calling `lr20xx_workarounds_lora_freq_hop_disable_sx1276_compatibility_mode`.

By default this configuration is not retain in memory when entering sleep mode. The function `lr20xx_workarounds_lora_freq_hop_sx1276_compatibility_mode_store_retention_mem` allows to store the SX1276 LoRa intra-packet frequency hopping compatible state in a retention memory slot.

#### OOK detection threshold

The LR20xx automatically computes a detection threshold when OOK modulation is being configured. The computed value is known to be too conservative so that the Packet Error Rate (PER) of received OOK packets is higher than expected.

Therefore if the noise level is higher than the instantaneous RSSI measured, the value can changed by calling `lr20xx_workarounds_ook_set_detection_threshold_level`.
The instantaneous RSSI is obtained by calling `lr20xx_radio_common_get_rssi_inst`.
The default OOK detection threshold depends on the modulation bandwidth configured, and can be obtained by calling `lr20xx_workarounds_ook_get_default_detection_threshold_level`.

#### RTToF results standard deviation on fractional bandwidths

Executing RTToF operations on bandwidths 812kHz, 406kHz, 203kHz and 101kHz (a.k.a. *fractional bandwidths*) exposes unexpectedly high standard deviation of the RTToF results.

The workaround `lr20xx_workarounds_rttof_results_deviation` reduces the result standard deviation on fractional bandwidths.
It must be applied after `lr20xx_radio_lora_set_modulation_params`, and the changes executed by this workaround are reverted by next call of `lr20xx_radio_lora_set_modulation_params`.

The workaround `lr20xx_workarounds_rttof_results_deviation` must be called only when attempting RTToF operations on fractional bandwidths.

The workaround addresses two registers that are, by default, not maintained during sleep mode with retention.
To avoid reverting the workaround by going in sleep mode with retention, the helper function `lr20xx_workarounds_rttof_results_deviation_store_retention_mem` must be called to keep the workaround registers in retention memory during sleep with retention.

#### Disabling of LoRa length check on packet reception

The LoRa packet parameter payload length is used for both length of transmitted packet, and upper limit for received payload length.
However using Auto Tx/Rx feature, it is possible to configure only one single value which serves in both Tx and Rx sequences.
This can be an issue if the Tx payload length is inferior to the expected payload length of the automatic Rx sequence.

The workaround `lr20xx_workarounds_lora_disable_rx_length_check` disables the automatic payload legnth check in Rx.
It must be called after `lr20xx_radio_lora_set_packet_params`, and a call to `lr20xx_radio_lora_set_packet_params` revert the workaround.

The workaround is not maintained in sleep mode with retention by default.
To maintain it during sleep mode with retention, the function `lr20xx_workarounds_lora_disable_rx_length_check_store_retention_mem` must be called.

#### FLRC sampling rate

Low power modulation spurs are present in FLRC transmit mode, that can pose an issue for regulatory compliance in some regions (for example Indian compliance with the EN 340 220 regulations).

The sampling rate can be reconfigure to avoid the spurs by calling `lr20xx_workarounds_flrc_configure_double_rate_sampling` with argument `enable` set to `true`.
It must be called after `lr20xx_radio_flrc_set_modulation_params`.

By default this configuration is not retained in memory when entering sleep mode. The function `lr20xx_workarounds_flrc_configure_double_rate_sampling_store_retention_mem` allows to store the FLRC double sample rate configuration state in a retention memory slot.

It can be reverted by calling `lr20xx_workarounds_flrc_configure_double_rate_sampling` with argument `enable` set to `false`.
Note that a call to `lr20xx_radio_common_set_pkt_type` revert the sampling rate configuration.

#### FLRC IF correlators disabling

FLRC reception power consumption and false detection probability can be reduced by disabling the intermediate frequency correlators.
This is achieved by calling function `lr20xx_workarounds_flrc_disable_if_correlators`.

Note that the frequency error between receiver and transmitter must have been previously compensated, typically by receiving a LoRa frame, reading the field `freq_offset_hz` from command `lr20xx_radio_lora_get_packet_status`, and adding the offset to expected RF frequency when calling `lr20xx_radio_common_set_rf_freq`.

To re-enable the FLRC IF correlators, the function `lr20xx_radio_flrc_set_modulation_params` has to be called.
Note that as calling `lr20xx_radio_flrc_set_modulation_params` re-enables the FLRC IF correlators, therefore `lr20xx_workarounds_flrc_disable_if_correlators` must be called after `lr20xx_radio_flrc_set_modulation_params` to properly disable the FLRC IF correlators.

By default this configuration is not retained in memory when entering sleep mode. The function `lr20xx_workarounds_flrc_disable_if_correlators_store_retention_mem` allows to store the FLRC IF correlators configuration state in a retention memory slot.

#### Incorrect VBAT read after temperature reading

When `lr20xx_system_get_temp` is called with `src` being `LR20XX_SYSTEM_TEMP_SRC_VBE` or `LR20XX_SYSTEM_TEMP_SRC_XOSC` an internal state is not correctly reset, so that next call to `lr20xx_system_get_vbat` returns an incorrect value.

The internal state can be reset with workaround function `lr20xx_workaround_clear_post_temperature_read`.
`lr20xx_system_get_temp` automatically calls `lr20xx_workaround_clear_post_temperature_read`, unless the macro `LR20XX_WORKAROUND_DISABLE_CLEAR_POST_TEMPERATURE_READ` is defined.
