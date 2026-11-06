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

## Workarounds

The workarounds defined here are expected to be used for LR20xx engineering samples (date code: `2513`, version `0x0110`).

Workarounds are defined in files [lr20xx_workarounds.h](inc/lr20xx_workarounds.h) and [lr20xx_workarounds.c](src/lr20xx_workarounds.c) when appropriate.

### Bluetooth LE Coded PHY Access Address

When configuring Bluetooth LE modulation or packet with `phy` value being `LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_500KB` or `LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_125KB`, some access address may fail to be transmitted and received correctly, resulting in degraded packet error rate.

To workaround this issue, the function `lr20xx_workaround_bluetooth_le_phy_coded_syncwords` must be called after issuing `lr20xx_radio_bluetooth_le_set_modulation_params` or `lr20xx_radio_bluetooth_le_set_pkt_params` if the configured `phy` is `LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_500KB` or `LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_125KB` after the latest call to `lr20xx_radio_bluetooth_le_set_modulation_params` or `lr20xx_radio_bluetooth_le_set_pkt_params`.

None of the functions `lr20xx_radio_bluetooth_le_set_modulation_params` and `lr20xx_radio_bluetooth_le_set_pkt_params` automatically calls `lr20xx_workaround_bluetooth_le_phy_coded_syncwords`.

However the helper function `lr20xx_radio_bluetooth_le_set_modulation_pkt_params` calls `lr20xx_radio_bluetooth_le_set_modulation_params`, `lr20xx_radio_bluetooth_le_set_pkt_params` and then `lr20xx_workaround_bluetooth_le_phy_coded_syncwords`.
Therefore it should be used to ensure both Bluetooth LE modulation and packet parameters are configured, and the workaround is applied appropriately.

### SX1276 LoRa compatibility mode

When SX1276 LoRa compatibility is required, the workaround `lr20xx_workaround_lora_enable_sx1276_compatibility_mode` must be called. It can be disabled afterwards by calling `lr20xx_workaround_lora_disable_sx1276_compatibility_mode`.

By default this configuration is not retain in memory when entering sleep mode. The function `lr20xx_workaround_lora_sx1276_compatibility_mode_store_retention_mem` allows to store the SX1276 LoRa compatible state in a retention memory slot.

### SX1276 LoRa intra-packet frequency hopping mode

When LoRa intra-packet frequency hopping is required to be compatible with SX1276, the workaround `lr20xx_workaround_lora_freq_hop_enable_sx1276_compatiblity_mode` must be called after `lr20xx_radio_lora_set_freq_hop`. It can be disabled afterwards by calling `lr20xx_workaround_lora_freq_hop_disable_sx1276_compatiblity_mode`.

By default this configuration is not retain in memory when entering sleep mode. The function `lr20xx_workaround_lora_freq_hop_sx1276_compatiblity_mode_store_retention_mem` allows to store the SX1276 LoRa intra-packet frequency hopping compatible state in a retention memory slot.

### OOK detection threshold

The LR20xx automatically computes a detection threshold when OOK modulation is being configured. The computed value is known to be too conservative so that the Packet Error Rate (PER) of received OOK packets is higher than expected.

Therefore if the noise level is higher than the instantaneous RSSI measured, the value can changed by calling `lr20xx_workarounds_ook_set_detection_threshold_level`.
The instantaneous RSSI is obtained by calling `lr20xx_radio_common_get_rssi_inst`.
The default OOK detection threshold depends on the modulation bandwidth configured, and can be obtained by calling `lr20xx_workarounds_ook_get_default_detection_threshold_level`.

### RTToF PLL frequency step

Biased RTToF results may be observed if the RF frequency configured is not a multiple of 122Hz.

To avoid this situation, the workaround `lr20xx_workarounds_rttof_truncate_pll_freq_step` must be called after `lr20xx_radio_common_set_rf_freq` if RTToF operations are intended.
After executing the workaround, the RF frequency will effectively be changed by a quantity inferior or equal to 122Hz.
