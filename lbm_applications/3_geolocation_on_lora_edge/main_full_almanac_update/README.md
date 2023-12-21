# LR11xx full almanac update example

## Description

This application executes a full almanac update by using the LR11xx API.
It does not involve LoRaWAN communication to update the almanac.
For the almanac update through LoRaWAN, refer to the example *almanac_update*.

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
$ python get_full_almanac.py -f almanac.h put_you_LoRaCloud_MGS_token_here
```

### Compile and flash the binary code

The example code expects the almanac C header file produced by *get_full_almanac.py* python script to be named `almanac.h`.

In order to build the binary file, do the following:
```bash
$ make full_lr1110 MODEM_APP=EXAMPLE_FULL_ALMANAC_UPDATE
```

## Expected Behavior

Here follow the steps that have to be seen in the logs to indicate the expected behavior of the application.

### Device starts

```
INFO: Modem Initialization

INFO: FULL ALMANAC UPDATE example is starting
```

### Full update of the almanac

The successful completion of the full almanac update is indicated by:

```
Source almanac date: Sun 2023-11-12 00:00:00 GMT

INFO: Local almanac doesn't match LR11XX almanac -> start update
INFO: Almanac update succeeded
```
