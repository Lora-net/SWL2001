# LoRa Basics Modem

## LoRaWAN parameters

### LoRaWAN version

The LoRaWAN version that is currently implemented in LoRa Basics Modem is v1.0.4.

### LoRaWAN region

LoRa Basics Modem supports the following LoRaWAN regions:

* AS_923 (AS923-1, AS923-2, AS923-3, AS923-4)
* AU_915
* CN_470
* CN_470_RP_1_0
* EU_868
* IN_865
* KR_920
* RU_864
* US_915

LoRa Basics Modem supports an emulation of LoRaWAN protocol for the 2.4GHz global ISM band (WW2G4)

### LoRaWAN regional parameters

Default regional parameters version supported by LoRa Basics Modem is rp2-1.0.3. It is possible to switch back to rp2-1.0.1 at compile time.

### LoRaWAN class

LoRa Basics Modem supports the following LoRaWAN classes:

* Class A
* Class B (with up to 4 multicast sessions)
* Class C (with up to 4 multicast sessions)

## LoRa Basics Modem services

LoRa Basics Modem supports the following services:

* Large files upload
* ROSE Streaming
* Application-Layer Clock synchronization
* Almanac Update for LoRa Edge

## LoRa Basics Modem API

The Application Programming Interface of LoRa Basics Modem is defined in `smtc_modem_api/smtc_modem_api.h` header file.

## LoRa Basics Modem engine

LoRa Basics Modem has to be initialized first by calling `smtc_modem_init()`. Then, calling periodically `smtc_modem_run_engine()` is required to make the state machine move forward.

These functions can be found in `smtc_modem_api/smtc_modem_utilities.h`

## LoRa Basics Modem HAL

The Hardware Abstraction Layer of LoRa Basics Modem is defined in the `smtc_modem_hal/smtc_modem_hal.h` header file. Porting LoRa Basics Modem to a new architecture requires one to implement the functions described by the prototypes in it.

## Transceiver

LoRa Basics Modem supports the following transceivers:

* LR1110 with firmware 0x0308.
* LR1120 with firmware 0x0102
* LR1121 with firmware 0x0102
* SX1261
* SX1262
* SX1280
* SX1281

## Known Limitations

* [LFU] In case LoRa Basics Modem is acting in US915 region with datarate DR0, files smaller than 13 bytes are not properly sent and cannot be reconstructed on LoRa Cloud side
* [charge] Values returned by `smtc_modem_get_charge()` for regions CN470 and CN470_RP1 are not accurate
* [charge] Values returned by `smtc_modem_get_charge()` for the LR-FHSS based datarate are not accurate
* [time] In case ALC_SYNC time service is used, when a valid time is received, the generated `SMTC_MODEM_EVENT_TIME` event will show a ghost missed event.

## Disclaimer

This software has been extensively tested when targeting LR1110 / LR1120 / LR1121 / SX1261 / SX1262 / SX1280 / SX1281 for LoRaWAN regions mentioned in [this paragraph](#lorawan-region). For all other combinations of features this software shall be considered an Engineering Sample.

Modem trace prints can only be used for debug purpose and shall be deactivated for production release.

### Disclaimer for Engineering Samples

Information relating to this product and the application or design described herein is believed to be reliable, however such information is provided as a guide only and Semtech assumes no liability for any errors related to the product, documentation, or for the application or design described herein. Semtech reserves the right to make changes to the product or this document at any time without notice.
