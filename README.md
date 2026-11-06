# LoRa Basic Modem

**LoRa Basic Modem** proposes a full implementation of the [TS001-LoRaWAN L2 1.0.4](https://resources.lora-alliance.org/technical-specifications/ts001-1-0-4-lorawan-l2-1-0-4-specification) and [Regional Parameters RP2-1.0.3](https://resources.lora-alliance.org/technical-specifications/rp2-1-0-3-lorawan-regional-parameters) specifications.

**LoRa Basic Modem** embeds an implementation of all LoRaWAN packages dedicated to Firmware Update Over The Air (FUOTA):

- Application Layer Clock Synchronization (ALCSync) [TS003-1.0.0](https://resources.lora-alliance.org/technical-specifications/lorawan-application-layer-clock-synchronization-specification-v1-0-0) / [TS003-2.0.0](https://resources.lora-alliance.org/technical-specifications/ts003-2-0-0-application-layer-clock-synchronization)
- Fragmented Data Block Transport [TS004-1.0.0](https://resources.lora-alliance.org/technical-specifications/lorawan-fragmented-data-block-transport-specification-v1-0-0) / [TS004-2.0.0](https://resources.lora-alliance.org/technical-specifications/ts004-2-0-0-fragmented-data-block-transport)
- Remote Multicast Setup [TS005-1.0.0](https://resources.lora-alliance.org/technical-specifications/lorawan-remote-multicast-setup-specification-v1-0-0) / [TS005-2.0.0](https://resources.lora-alliance.org/technical-specifications/ts005-2-0-0-remote-multicast-setup)
- Firmware Management Protocol (FMP) [TS006-1.0.0](https://resources.lora-alliance.org/technical-specifications/ts006-1-0-0-firmware-management-protocol)
- Multi-Package Access (MPA) [TS007-1.0.0](https://resources.lora-alliance.org/technical-specifications/ts007-1-0-0-multi-package-access)

**LoRa Basic Modem** embeds an implementation of the Relay LoRaWAN® Specification [TS011-1.0.1](https://resources.lora-alliance.org/technical-specifications/ts011-1-0-1-relay)

- Relay Tx (relayed end-device)
- Relay Rx
```
          +--------------------+       +------------------+
          | Wake On Radio      |       |     LoRaWAN      |
          | protocol + LoRaWAN |       | Class A, B, or C |
          +--------------------+       +------------------+
                    \                        /
                     \                      /
                      \                    /
         Relay Tx      v                  v
      ( End-Device ) <----> (Relay Rx) <----> (Gateway) <----> (Network Server)
              ^                                 ^
               \                               /
                -------------------------------
```

**LoRa Basic Modem** embeds an implementation of the LoRaWAN certification process
- LoRaWAN certification process [TS009-1.2.1](https://resources.lora-alliance.org/technical-specifications/ts009-1-2-1-certification-protocol)

**LoRa Basic Modem** offers:
- Geolocation services in combination with LoRa Edge chips

## Prerequisites

- **GNU Arm Embedded Toolchain**  
  The LoRa Basics Modem library is developed using:  
  **GNU Arm Embedded Toolchain 13.2.rel1-2 (13.2.1 20231009)**  
  Ensure this version (or a compatible one) is installed and available in your `PATH`.  

- **Make** (if building with Make)  
  Make is typically available by default on Linux systems.

- **CMake** (if building with CMake)  
  Install CMake and ensure it is available in your `PATH`.
  
- **Ninja** (optional, if using CMake with Ninja generator)  
  Ninja must be installed if you choose to build with CMake + Ninja.

## LoRa Basics Modem library

LBM library code can be found in folder [lbm_lib](lbm_lib/).  
Please refer to [README.md](lbm_lib/README.md) to get all information related to LoRa Basics Modem library

## Examples

Under `lbm_examples` folder, there are few examples on how to use the LoRa Basics Modem stack.

- Hardware Modem (Implements a hardware modem controlled by a serial interface)
- Periodical uplink (joins the network and then sends periodic uplinks and each time the button is pushed)
- Porting tests (Allows to verify if the project porting process is correct)
- LCTT certification (to run LoRaWAN certification)

The examples are targeted for the Nucleo L476 kit featuring an STM32L476 micro-controller.
For further details please refer to `lbm_examples` directory [README](lbm_examples/README.md) file.

To build the periodical uplink example targeting the LR1110 Semtech radio the following should be executed on the command line:

```bash
make -C lbm_examples full_lr1110 MODEM_APP=PERIODICAL_UPLINK
```

Or, with CMake:

```bash
cd lbm_examples
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=MinSizeRel -DLBM_CMAKE_CONFIG_AUTO=ON -DBOARD=NUCLEO_L476 -DLBM_RADIO=lr1110 -DAPP=periodical_uplink
cd build
ninja
# This will flash a connected stm32 nucleo board
ninja flash
```

## Applications

Under `lbm_applications` folder, there are 3 specific applications that are using the LoRa Basics Modem stack.

- A ThreadX Operating System running on STM32U5 ([lbm_applications/1_thread_x_on_stm32_u5/README.md](lbm_applications/1_thread_x_on_stm32_u5/README.md))
- A LBM porting on Nordic NRF52840 ([lbm_applications/2_porting_nrf_52840/README.md](lbm_applications/2_porting_nrf_52840/README.md))  
- A Geolocation application running on Lora Edge ([lbm_applications/3_geolocation_on_lora_edge/README.md](lbm_applications/3_geolocation_on_lora_edge/README.md))

An integration in Zephyr OS is available in another repository, instructions to download this integration and LoRa Basics Modem
are available at [LBM_Zephyr](https://github.com/Lora-net/LBM_Zephyr/blob/master/README.md).
