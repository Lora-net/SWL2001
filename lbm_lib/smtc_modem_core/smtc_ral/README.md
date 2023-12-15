# Semtech Radio Abstraction Layer

This package proposes an implementation in C of the RAL (Radio Abstraction Layer).

## Components

The RAL is split in several components:

- RAL interface
- Implementation for SX1261/2/8 transceivers
- Implementation for LR1110/20/21 transceivers
- Implementation for SX1280/1 transceivers
- Implementation for LLCC68 transceivers

## Compatibility

The current version of the RAL is compatible with the following version the drivers:

- LR11xx driver unreleased (head of develop branch)
- SX126x driver v2.0.1 and later
- SX128x driver unreleased (head of develop branch)
- LLCC68 driver v2.0.1 and later
