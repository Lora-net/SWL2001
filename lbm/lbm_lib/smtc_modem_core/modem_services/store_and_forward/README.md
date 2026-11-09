# LoRa Basics Modem - Store and Forward flash

## 1. Introduction

The Store and Forward feature allows a user to store multiple data such as measurements without waiting for the stack to be available or the modem to be under network coverage. This data will be sent when the modem is under LoRaWAN coverage.

The Store and Forward used CircularFS library. It is a persistent, Flash-based ring buffer designed for embedded software. CircularFS has been designed to run on NOR Flash memory, which exhibits the following semantics:

- Bits are programmed by flipping them from 1 to 0 with byte granularity.
- Bits are erased by flipping them from 0 to 1 with sector granularity.

## 2. Operating mode

To be sure that the modem is always under coverage, data are acknowledged every 10 uplink frames (but the user can request an acknowledgment themselves when new data is stored to be acknowledged more often).

If the uplink frame with acknowledgment request is acknowledged by the network, previous sent data are deleted from the FIFO.

If the uplink frame with acknowledgment request is not acknowledged by the network, the FIFO goes back to the oldest unacknowledged data and will attempt to send this data indefinitely until the modem has received the acknowledgment.

To avoid retrying too often, a time limit is calculated to schedule the next attempt. This delay is generally longer than the previous one for each retry and the maximum is one hour.

When the device is under coverage (ACK or downlink data received by the modem), data in FIFO are fetch and sent as fast as possible.
