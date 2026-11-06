# Reference Implementation 3: Geolocation example application

## Hardware

### MCU Board
- Nucleo L476RG board ( with STM32L476RGT6 MCU)

### Radio Boards
- lr1110 (EVK board)
- lr1120 (EVK board)

## Description

This example illustrates how a typical application running LoRa Basics Modem (LBM) can benefit from the geolocation services to perform GNSS and Wi-Fi scans and send it either with direct uplinks, or using the Store & Forward service.

This example demonstrates:
- how to configure LoRa Basics Modem library
- how to start the needed services (almanac demodulation update for GNSS scan, store & forward, ...)
- how to program GNSS/Wi-Fi scans
- how to get events from the services to sequence the geolocation scans

## Geolocation services

This example application relies on various services provided by LoRa Basics Modem to simplify the usage of a LR11xx radio to perform geolocation based on GNSS and Wi-Fi scans.

The services used are:
* GNSS scan & send
* GNSS almanac demodulation
* Wi-Fi scan & send
* Store & Forward

For more details about the GNSS and Wi-Fi services, please refer to the documentation here:

[Geolocation services documentation](<../../lbm_lib/smtc_modem_core/geolocation_services/README.md>)

## LoRa Basics Modem configuration

Once the modem has reset, the application configures the various services used and initiate both GNSS and Wi-Fi scans. Then, it completely relies on LBM arbitration to handle LR11xx radio access.

In this example, the device does not need to successfully join a LoRaWAN network to start scanning.
If "Store & Forward" is enabled the scan results are stored in the device's flash memory, and forwarded later once the device has joined a network. Otherwise, the scan results are lost.

Once the modem has joined the LoRaWAN network, the application configures the ADR custom profile.
It is necessary to use a custom profile due to payload size constraints.

## Geolocation services events

After the initial scan has been initiated, the application has to rely on the events sent by the GNSS/Wi-Fi services to get results and program the next scan.

The events to be handled are the following:
* SMTC_MODEM_EVENT_GNSS_SCAN_DONE: the GNSS scan service has completed the multiframe scan sequence. The application can retrieve the associated data.
* SMTC_MODEM_EVENT_GNSS_TERMINATED: the GNSS send service has completed the multiframe send sequence according to the selected send mode (direct uplink, store & forward, bypass...). The application must wait for this event before programming the next GNSS scan.
* SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE: the GNSS almanac demodulation service has progressed. This event is for information, there is no action to take from the application.
* SMTC_MODEM_EVENT_WIFI_SCAN_DONE: the Wi-Fi scan service has completed the scan. The application can retrieve the associated data.
* SMTC_MODEM_EVENT_WIFI_TERMINATED: the Wi-Fi send service has completed the send sequence according to the selected send mode (direct uplink, store & forward, bypass...). The application must wait for this event before programming the next Wi-Fi scan.

## Configuration

### LoRaWAN related configuration

The `example_options.h` header file defines several constants to configure the LoRaWAN parameters (region, keys).

* MODEM_EXAMPLE_REGION: Selects the regulatory region. This example has been tested for SMTC_MODEM_REGION_EU_868.
* USER_LORAWAN_DEVICE_EUI: LoRaWAN device EUI for user defined credentials.
* USER_LORAWAN_JOIN_EUI: LoRaWAN Join EUI for user defined credentials.
* USER_LORAWAN_APP_KEY: LoRaWAN App Key for user defined credentials.

The custom ADR profile parameters are defined in the `main_geolocation.c` file.

* CUSTOM_NB_TRANS: The number of retransmission to be done for each LoRaWAN uplink
* ADR_CUSTOM_LIST: The custom datarate distribution to be used for LoRaWAN uplink

Those parameters have to be carefully defined depending of the region of operation, the scan period, the payload size etc...

### Geolocation demonstration related configuration

* GEOLOCATION_GNSS_SCAN_PERIOD_S: Defines the duration between the end of a GNSS scan & send sequence and the start of next sequence, in seconds.
* GEOLOCATION_WIFI_SCAN_PERIOD_S: Defines the duration between the end of a Wi-Fi scan & send sequence and the start of next sequence, in seconds.
* By default, the selected GNSS scan mode is SMTC_MODEM_GNSS_MODE_MOBILE, and the GNSS send mode is SMTC_MODEM_SEND_MODE_STORE_AND_FORWARD.
* By default, the selected WiFi send mode is SMTC_MODEM_SEND_MODE_UPLINK.

## Build

By default, the demonstration is compiled to use the EUIs and Application key defined in the file `example_options.h` file.

### With CMake

Ensure that there is no existing build directory from a previous build.
Use "rm -r build" if needed to clean it up.

``` bash
cmake -B build -G Ninja -DLBM_RADIO=lr1110 -DCMAKE_BUILD_TYPE=MinSizeRel -DLBM_CMAKE_CONFIG_AUTO=ON -DAPP=geolocation
```

Optionnally add `-DLBM_MODEM_TRACE=no` to the cmake command to disable LBM traces.

``` bash
ninja -C build
```

It is possible to flash the target with the following commands:

``` bash
ninja -C build flash
```

Note: `stlink-tools` needs to be installed.

For "copy" flashing, the following command can be used:

``` bash
ninja -C build flash_copy
```

Note: it expects the target to be mounted to `/media/$(USER)/NODE_L476RG`

### With make

The example can be built through GNU make command by doing the following:

```shell
$ make full_lr1110
```

All the default compilation options used are available in the `app_makefiles/app_options.mk` file.

Optionnally add `LBM_TRACE=no` to the make command to disable LBM traces.

## Usage

### Serial console

The application requires no user intervention after the static configuration
option have been set.

### LoRaWAN Network Server / Application Server

This application needs an Application Server to run in order to perform the GNSS and Wi-Fi solving.

The Traxmate Cloud for LoRaWAN provides APIs for almanac update, WiFi and GNSS solving:
https://traxmate.io/solutions/tracking-integrations/traxmate-cloud-for-lorawan

## Tools

### Almanac full update

A tool is provided to flash a full almanac image to the LR11xx chip. Please refer to the following documentation for more details.

[Full almanac update tool documentation](<./main_full_almanac_update/README.md>)

### LR11xx flasher

A tool is provided to flash the LR11xx chip transceiver firmware. Please refer to the following documentation for more details.

[LR11xx flasher tool documentation](<./main_lr11xx_flasher/README.md>)

### WiFi region detector

An example provided to demonstrate how to use WiFi scanning to infer the current regulatory region around the device.
Please refer to the following documentation for more details.

[WiFi Region Detection documentation](<./main_wifi_region_detection/README.md>)