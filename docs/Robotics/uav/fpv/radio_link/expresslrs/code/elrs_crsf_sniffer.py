#!/usr/bin/env python3
import argparse
import sys
import time

import serial


FrameOutput = tuple[tuple[int, int], str]

CRSF_TYPE_RC_CHANNELS_PACKED = 0x16
CRSF_RC_CHANNEL_COUNT = 16
CRSF_RC_CHANNEL_BITS = 11
CRSF_RC_CHANNEL_PAYLOAD_LEN = 22

CRSF_TYPE_NAMES = {
    0x02: "GPS",
    0x07: "VARIO",
    0x08: "BATTERY",
    0x09: "BARO_ALTITUDE",
    0x0B: "HEARTBEAT",
    0x0C: "VIDEO_TRANSMITTER",
    0x14: "LINK_STATISTICS",
    0x16: "RC_CHANNELS_PACKED",
    0x17: "SUBSET_RC_CHANNELS_PACKED",
    0x1C: "LINK_RX_ID",
    0x1D: "LINK_TX_ID",
    0x21: "ATTITUDE",
    0x29: "DEVICE_PING",
    0x2A: "DEVICE_INFO",
    0x2B: "PARAMETER_SETTINGS_ENTRY",
    0x2C: "PARAMETER_READ",
    0x2D: "PARAMETER_WRITE",
    0x32: "COMMAND",
    0x3A: "RADIO_ID",
}

CRSF_ADDR_NAMES = {
    0x00: "BROADCAST",
    0xC0: "USB",
    0xC2: "TBS_CORE_PNP_PRO",
    0xC4: "RESERVED1",
    0xC6: "CURRENT_SENSOR",
    0xC8: "FLIGHT_CONTROLLER",
    0xCA: "GPS",
    0xCC: "TBS_BLACKBOX",
    0xCE: "RACE_TAG",
    0xEA: "RADIO_TRANSMITTER",
    0xEC: "CRSF_RECEIVER",
    0xEE: "CRSF_TRANSMITTER",
}


def raw_rc_to_us(raw: int) -> int:
    return round((raw - 992) * 5 / 8 + 1500)


def parse_rc_channels(payload: bytes) -> list[int]:
    if len(payload) != CRSF_RC_CHANNEL_PAYLOAD_LEN:
        raise ValueError(
            f"expected {CRSF_RC_CHANNEL_PAYLOAD_LEN} bytes, got {len(payload)}"
        )

    packed = int.from_bytes(payload, "little")
    channels = []
    mask = (1 << CRSF_RC_CHANNEL_BITS) - 1
    for channel_index in range(CRSF_RC_CHANNEL_COUNT):
        raw = (packed >> (channel_index * CRSF_RC_CHANNEL_BITS)) & mask
        channels.append(raw_rc_to_us(raw))
    return channels


def crc8_d5(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def printable_frame(addr: int, frame_type: int, payload: bytes, crc_ok: bool) -> str:
    addr_name = CRSF_ADDR_NAMES.get(addr, "UNKNOWN")
    type_name = CRSF_TYPE_NAMES.get(frame_type, "UNKNOWN")
    payload_hex = payload.hex(" ")
    status = "ok" if crc_ok else "bad-crc"
    line = (
        f"addr=0x{addr:02X}({addr_name}) "
        f"type=0x{frame_type:02X}({type_name}) "
        f"payload_len={len(payload)} crc={status} payload={payload_hex}"
    )
    if frame_type == CRSF_TYPE_RC_CHANNELS_PACKED:
        try:
            channels_us = parse_rc_channels(payload)
            line += f" channels_us={channels_us}"
        except ValueError as exc:
            line += f" channels_decode_error={exc}"
    return line


def parse_buffer(buf: bytearray, show_bad_crc: bool) -> list[FrameOutput]:
    lines = []
    while len(buf) >= 4:
        length = buf[1]
        if length < 2 or length > 64:
            del buf[0]
            continue

        frame_size = length + 2
        if len(buf) < frame_size:
            break

        frame = bytes(buf[:frame_size])
        addr = frame[0]
        frame_type = frame[2]
        payload = frame[3:-1]
        received_crc = frame[-1]
        calculated_crc = crc8_d5(frame[2:-1])
        crc_ok = received_crc == calculated_crc

        if crc_ok or show_bad_crc:
            lines.append(
                ((addr, frame_type), printable_frame(addr, frame_type, payload, crc_ok))
            )

        del buf[:frame_size]
    return lines


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Sniff ExpressLRS / CRSF serial traffic from a USB serial adapter."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=420000)
    parser.add_argument("--raw", action="store_true", help="also print raw bytes when no valid frame is decoded")
    parser.add_argument("--bad-crc", action="store_true", help="print frames that fail CRSF CRC validation")
    parser.add_argument(
        "--no-change-filter",
        action="store_true",
        help="print every decoded frame instead of only frames that changed",
    )
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as exc:
        print(f"Could not open {args.port}: {exc}", file=sys.stderr)
        return 1

    print(f"Listening on {args.port} at {args.baud} baud. Press Ctrl-C to stop.")
    buf = bytearray()
    last_output_by_frame_type = {}
    try:
        while True:
            chunk = ser.read(256)
            if not chunk:
                continue

            buf.extend(chunk)
            lines = parse_buffer(buf, args.bad_crc)
            timestamp = time.strftime("%H:%M:%S")
            if lines:
                for frame_key, line in lines:
                    if not args.no_change_filter:
                        if last_output_by_frame_type.get(frame_key) == line:
                            continue
                        last_output_by_frame_type[frame_key] = line
                    print(f"{timestamp} {line}", flush=True)
            elif args.raw:
                print(f"{timestamp} raw {chunk.hex(' ')}", flush=True)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
