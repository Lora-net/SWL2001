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
* SX1261
* SX1262

## Known Limitations

* [LFU] In case LoRa Basics Modem is acting in US915 region with datarate DR0, files smaller than 13 bytes are not properly sent and cannot be econstructed on LoRa Cloud side
* [LFU] LoRa Basics Modem does not reject files with a size between 8181 and 8192 bytes while they cannot be sent properly
* [charge] Values returned by `smtc_modem_get_charge()` for regions CN470 and CN470_RP1 are not accurate
* [charge] Values returned by `smtc_modem_get_charge()` for the LR-FHSS based datarate are not accurate
* [LBT] On LR1110 target, sometimes the LBT pre-hook can be outdated and aborted which leads to no uplink issued (this is due to a radio reset called before starting LBT operation which adds the LR1110 boot delay before any LBT actions) - as workaround, the call to `ral_init()` can be removed from `smtc_modem_core/lr1mac/src/services/smtc_lbt.c`
* [ADR] When a MAC command `link_adr_req` with a new channel mask is received, it is rejected if the custom datarate profile is enabled and configured with the highest datarate of the corresponding region - as a workaround, make sure there is at least one datarate different from the highest possible one in the custom ADR list

## Disclaimer

This software has been extensively tested when targeting LR1110 / SX1261 / SX1262 for LoRaWAN regions mentioned in [this paragraph](#lorawan-region). For all other combinations of features this software shall be considered an Engineering Sample.

All customers wanting to leverage LoRa Basics Modem for 2.4GHz running with SX1280 transceiver must still refer to the [release v1.0.1](https://github.com/lorabasics/lorabasicsmodem/releases/tag/v1.0.1) for which Semtech provides technical customer support.

### Disclaimer for Engineering Samples

Information relating to this product and the application or design described herein is believed to be reliable, however such information is provided as a guide only and Semtech assumes no liability for any errors related to the product, documentation, or for the application or design described herein. Semtech reserves the right to make changes to the product or this document at any time without notice.
