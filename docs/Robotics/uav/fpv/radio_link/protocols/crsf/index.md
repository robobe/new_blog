---
title: CRSF Protocol
tags:
    - drone
    - fpv
    - crsf
    - crossfire
    - expresslrs
---

## What CRSF Is

`CRSF` means `Crossfire Serial Protocol`. It started as the serial protocol used by `TBS Crossfire`, but it is now also used by other FPV radio systems, especially `ExpressLRS`.

The important distinction:

| Name | Meaning |
| ---- | ------- |
| `TBS Crossfire` | A proprietary long-range radio control system |
| `CRSF` | The serial message protocol used between receiver, transmitter module, radio handset, and flight controller |
| `ExpressLRS` / `ELRS` | An open-source radio control system that commonly uses CRSF-compatible serial messages |

So `ELRS` is not Crossfire RF. It uses its own radio link and firmware, but it speaks `CRSF` on the serial side so flight controllers can treat it like a CRSF receiver.

## Frame Structure

A normal CRSF frame has this byte layout:

```text
+---------+--------+-----------+-------------+--------+
| address | length | type      | payload     | crc8   |
+---------+--------+-----------+-------------+--------+
| 1 byte  | 1 byte | 1 byte    | 0..N bytes  | 1 byte |
+---------+--------+-----------+-------------+--------+
```

Fields:

| Field | Description |
| ----- | ----------- |
| `address` | Destination or source device address, depending on frame direction |
| `length` | Number of bytes after the length byte: `type + payload + crc8` |
| `type` | Message type ID |
| `payload` | Type-specific data |
| `crc8` | CRC over `type + payload`, using polynomial `0xD5` |

Example layout for an RC channel frame:

```text
C8 18 16 ...payload... CRC
```

Meaning:

| Byte | Meaning |
| ---- | ------- |
| `0xC8` | Flight controller address |
| `0x18` | 24 bytes after length: 1 type + 22 payload + 1 CRC |
| `0x16` | `RC_CHANNELS_PACKED` |

## Common Addresses

The address is the first byte in every CRSF frame. It identifies the intended
recipient of the frame, not necessarily the device that sent it. For example,
RC channel data sent from a receiver to a flight controller usually starts with
`0xC8`, because the frame is addressed to the flight controller.

Some devices also send telemetry or control frames back in the other direction,
using the address of the radio-side device as the destination. `0x00` is the
broadcast address and may be used for frames that are not targeted at one
specific device. The address byte is not included in the CRC calculation; the
CRC covers only the message type and payload bytes.

| Value | Name |
| ----- | ---- |
| `0x00` | `BROADCAST` |
| `0xC0` | `USB` |
| `0xC2` | `TBS_CORE_PNP_PRO` |
| `0xC4` | `RESERVED1` |
| `0xC6` | `CURRENT_SENSOR` |
| `0xC8` | `FLIGHT_CONTROLLER` |
| `0xCA` | `GPS` |
| `0xCC` | `TBS_BLACKBOX` |
| `0xCE` | `RACE_TAG` |
| `0xEA` | `RADIO_TRANSMITTER` |
| `0xEC` | `CRSF_RECEIVER` |
| `0xEE` | `CRSF_TRANSMITTER` |

## Common Message Types

| Value | Name | Typical use |
| ----- | ---- | ----------- |
| `0x02` | `GPS` | GPS telemetry |
| `0x07` | `VARIO` | Vertical speed telemetry |
| `0x08` | `BATTERY` | Battery telemetry |
| `0x09` | `BARO_ALTITUDE` | Barometer altitude telemetry |
| `0x0B` | `HEARTBEAT` | Device heartbeat |
| `0x0C` | `VIDEO_TRANSMITTER` | VTX control |
| `0x14` | `LINK_STATISTICS` | Link quality, RSSI, SNR, RF mode |
| `0x16` | `RC_CHANNELS_PACKED` | 16 packed RC control channels |
| `0x17` | `SUBSET_RC_CHANNELS_PACKED` | Packed subset of RC channels |
| `0x1C` | `LINK_RX_ID` | Receiver identity |
| `0x1D` | `LINK_TX_ID` | Transmitter identity |
| `0x21` | `ATTITUDE` | Attitude telemetry |
| `0x29` | `DEVICE_PING` | Device discovery |
| `0x2A` | `DEVICE_INFO` | Device information |
| `0x2B` | `PARAMETER_SETTINGS_ENTRY` | Device parameter entry |
| `0x2C` | `PARAMETER_READ` | Read device parameter |
| `0x2D` | `PARAMETER_WRITE` | Write device parameter |
| `0x32` | `COMMAND` | Command message |
| `0x3A` | `RADIO_ID` | Radio identity / sync data |

## RC Channels Packed

`RC_CHANNELS_PACKED` carries 16 RC channels in 22 payload bytes.

The channel values are packed as little-endian 11-bit integers:

```text
16 channels * 11 bits = 176 bits = 22 bytes
```

The raw channel range is commonly interpreted around:

| Raw CRSF value | Approx PWM value |
| -------------- | ---------------- |
| `172` | `988 us` |
| `992` | `1500 us` |
| `1811` | `2012 us` |

Approximate conversion:

```text
pwm_us = round((raw - 992) * 5 / 8 + 1500)
```

This is why a CRSF sniffer can read one `0x16` payload and print a normal channel array like:

```text
channels_us=[1500, 1500, 1000, 1000, ...]
```

## Relation To ExpressLRS

ExpressLRS uses CRSF because Betaflight, EdgeTX/OpenTX, and many FPV tools already understand it.

Typical paths:

```text
Radio handset <-> ELRS transmitter module: CRSF-style serial communication
ELRS receiver <-> Flight controller: CRSF receiver protocol
```

What is shared:

- Message framing
- Device addresses
- Common message types
- RC channel packing
- Telemetry transport pattern

What is not shared:

- RF modulation
- Binding process
- Firmware ecosystem
- Over-the-air packet format

In practice, this means a flight controller can be configured for `CRSF` whether the receiver is `TBS Crossfire`, `TBS Tracer`, or `ExpressLRS`.

## Related

- [Receiver Protocols](../index.md)
- [ExpressLRS](../../expresslrs/index.md)
- [TBS Crossfire](../../crossfire/index.md)
