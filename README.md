# LoRa Basic Modem

**LoRa Basic Modem** proposes an full implementation of the [TS001-LoRaWAN L2 1.0.4](https://resources.lora-alliance.org/technical-specifications/ts001-1-0-4-lorawan-l2-1-0-4-specification) and [Regional Parameters RP2-1.0.3](https://resources.lora-alliance.org/technical-specifications/rp2-1-0-3-lorawan-regional-parameters) specifications.

**LoRa Basic Modem** embedds also an implementation of all LoRaWAN packages dedicated to Firmware Update Over The Air (FUOTA):  
- Application Layer Clock Synchronization (ALCSync)
- Remote Multicast Setup
- Fragmented Data Block Transport
- Firmware Management Protocol (FMP)
- Multi-Package Access (MPA)
- Relay RX

**LoRa Basic Modem** offers extended services:  
- LoRaWAN certification process
- Geolocation with LoRa Edge chips
- LoRaCloud features such as Stream, Large File Upload, Device Management or Almanac Update

## LoRa Basics Modem library

LBM library code can be found in folder [lbm_lib](lbm_lib/).  
Please refer to [README.md](lbm_lib/README.md) to get all information related to LoRa Basics Modem library

## Examples

Under `lbm_examples` folder, one will find a few examples on how to use the LoRa Basics Modem stack.

- Hardware Modem (Implements a hardware modem controlled by a serial interface)
- Periodical uplink (joins the network and then sends periodic uplinks and each time the button is pushed)
- Porting tests (Allows to verify if the project porting process is correct)
- LCTT certification (to run LoRaWAN certification) 

The examples are targeted for the NucleoL476 kit featuring an STM32L476 micro-controller.
For further details please refer to `lbm_examples` directory [README](lbm_examples/README.md) file.

To build the periodical uplink example targeting the LR1110 Semtech radio the following should be executed on the command line:

```bash
make -C lbm_examples full_lr1110 MODEM_APP=PERIODICAL_UPLINK
```

## Applications
Under `lbm_applications` folder, one will find 3 specific applications that are using LoRa Basics Modem stack.  

- A ThreadX Operationg System running on STM32U5 ([lbm_applications/1_thread_x_on_stm32_u5/README.md](lbm_applications/1_thread_x_on_stm32_u5/README.md)) 
- A LBM porting on Nordic NRF52840 ([lbm_applications/2_porting_nrf_52840/README.md](lbm_applications/2_porting_nrf_52840/README.md))  
- A Geolocation application running on Lora Edge ([lbm_applications/3_geolocation_on_lora_edge/README.md](lbm_applications/3_geolocation_on_lora_edge/README.md)) 

