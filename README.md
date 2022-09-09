# LoRa Basics Modem

## LoRaWAN parameters

### LoRaWAN version

The LoRaWAN version that is currently implemented in LoRa Basics Modem is v1.0.4.

### LoRaWAN region

LoRa Basics Modem supports the following LoRaWAN regions:

* AS_923 (AS923-1, AS923-2, AS923-3)
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

Default regional parameters version supported by LoRa Basics Modem is rp2-1.0.1. It is possible to switch to rp2-1.0.3 at compile time.

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
* Almanac Update

## LoRa Basics Modem API

The Application Programming Interface of LoRa Basics Modem is defined in `smtc_modem_api/smtc_modem_api.h` header file.

## LoRa Basics Modem engine

LoRa Basics Modem has to be initialized first by calling `smtc_modem_init()`. Then, calling periodically `smtc_modem_run_engine()` is required to make the state machine move forward.

These functions can be found in `smtc_modem_api/smtc_modem_utilities.h`

## LoRa Basics Modem HAL

The Hardware Abstraction Layer of LoRa Basics Modem is defined in the `smtc_modem_hal/smtc_modem_hal.h` header file. Porting LoRa Basics Modem to a new architecture requires one to implement the functions described by the prototypes in it.

## Transceiver

LoRa Basics Modem supports the following transceivers:

* LR1110 with firmware 0x0307.
* LR1120 with firmware 0x0101
* SX1261
* SX1262
* SX1280
* SX1281

## Known Limitations

* [LFU] In case LoRa Basics Modem is acting in US915 region with datarate DR0, files smaller than 13 bytes are not properly sent and cannot be reconstructed on LoRa Cloud side
* [charge] Values returned by `smtc_modem_get_charge()` for regions CN470 and CN470_RP1 are not accurate
* [charge] Values returned by `smtc_modem_get_charge()` for the LR-FHSS based datarate are not accurate
* [multicast_class_b] Starting a class B multicast session with frequency equal to 0 will always return SMTC_MODEM_RC_INVALID even in the case lbm is acting in regions with frequency hopping beacon
* [LBT] In case LBT is used (by user's choice or imposed by regional parameters) and if TCXO start time is greater than default RP_MARGIN_DELAY value (8ms), uplinks will never be sent.
        Workaround: At makefile level define RP_MARGIN_DELAY value to `TCXO start time + 3ms`. The consequence is that before rx1 and rx2 windows opening the mcu will be running and waiting during extra time.

## Disclaimer

This software has been extensively tested when targeting LR1110 / LR1120 / SX1261 / SX1262 / SX1280 / SX1281 for LoRaWAN regions mentioned in [this paragraph](#lorawan-region). For all other combinations of features this software shall be considered an Engineering Sample.

Modem trace prints can only be used for debug purpose and shall be deactivated for production release.

### Disclaimer for Engineering Samples

Information relating to this product and the application or design described herein is believed to be reliable, however such information is provided as a guide only and Semtech assumes no liability for any errors related to the product, documentation, or for the application or design described herein. Semtech reserves the right to make changes to the product or this document at any time without notice.
