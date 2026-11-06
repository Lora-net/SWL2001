# LR11xx full almanac update example

## Description

This application executes a full almanac update from a given almanac binary image, by using dedicated LoRa Basics Modem
API.

This example also provides a simple python script *get_full_almanac.py* that fetches almanac content from LoRa Cloud and generate a C header file that is compiled with the embedded binary.

**NOTE**: This example is only applicable to LR1110 / LR1120 chips.

## Usage

The full almanac update with this example is executed in two steps:

1. Generate an almanac C header file with the python script *get_full_almanac.py*;
2. Build the example code and flash the binary to the Nucleo board.

### Generation of almanac C header file

The python script usage to generate the almanac C header file can be obtained with:

```bash
$ python ./get_full_almanac.py --help
```

For example, in order to get the latest almanac image, one can execute the following:

```bash
$ python get_full_almanac.py -f almanac.h PUT_YOUR_TRAXMATE_TOKEN_HERE
```

In order to get an almanac image for a specific date (for testing purpose), one can execute the following:

```bash
$ python get_full_almanac.py -f almanac.h -g 1419724818 PUT_YOUR_TRAXMATE_TOKEN_HERE
```

> Note: update the GPS time provided with the -g option with the desired GPS time.

### Compile and flash the binary code

The example code expects the almanac C header file produced by *get_full_almanac.py* python script to be named `almanac.h`.

The full almanac flasher tool can be compiled with CMake or make.

#### With CMake

```bash
cmake -B build -G Ninja -DLBM_RADIO=lr1110 -DCMAKE_BUILD_TYPE=MinSizeRel -DLBM_CMAKE_CONFIG_AUTO=ON -DAPP=full_almanac_update
```

Optionnally add `-DLBM_MODEM_TRACE=no` to the cmake command to disable LBM traces.

```bash
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

#### With make

```bash
make full_lr1110 MODEM_APP=EXAMPLE_FULL_ALMANAC_UPDATE
```

Optionnally add `LBM_TRACE=no` to the make command to disable LBM traces.

### Output example

```shell
INFO: FULL ALMANAC UPDATE example is starting
...
INFO: Event received: RESET
INFO: LR11XX FW: 0x0401, type: 0x01
INFO: mw_gnss_almanac_full_update: SUCCESS (123 ms)
INFO: SUCESS: almanac source date Sun 2025-03-30 00:00:00 GMT
```