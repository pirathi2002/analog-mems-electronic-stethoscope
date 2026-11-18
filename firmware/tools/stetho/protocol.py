"""Decoder for the stethoscope's binary serial protocol.

Frame layout (see firmware/stethoscope/src/link/protocol.h):

    A5 5A | type u8 | len u16le | payload[len] | crc16 u16le (CCITT-FALSE over type..payload)
"""
from __future__ import annotations

import json
import struct
from dataclasses import dataclass

import numpy as np

SYNC = b"\xA5\x5A"
FRAME_AUDIO, FRAME_STATUS, FRAME_TEXT = 0x01, 0x02, 0x03
FULL_SCALE = 32767.0  # int16 value of ADC full scale


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass
class AudioBlock:
    first_index: int
    raw: np.ndarray       # int16
    filtered: np.ndarray  # int16
    mode: int


@dataclass
class Message:
    kind: str             # "audio" | "status" | "text"
    audio: AudioBlock | None = None
    status: dict | None = None
    text: str | None = None


class FrameParser:
    """Incremental parser: feed() arbitrary byte chunks, get decoded messages.

    Bytes outside valid frames (e.g. plain-text boot messages) are collected and
    returned as "text" messages line by line.
    """

    MAX_LEN = 4096

    def __init__(self) -> None:
        self.buf = bytearray()
        self.text_buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes) -> list[Message]:
        self.buf += data
        out: list[Message] = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                # keep a possible partial sync byte
                keep = 1 if self.buf.endswith(b"\xA5") else 0
                self._text(bytes(self.buf[: len(self.buf) - keep]), out)
                del self.buf[: len(self.buf) - keep]
                break
            if i > 0:
                self._text(bytes(self.buf[:i]), out)
                del self.buf[:i]
            if len(self.buf) < 5:
                break
            ftype = self.buf[2]
            (length,) = struct.unpack_from("<H", self.buf, 3)
            if length > self.MAX_LEN or ftype not in (FRAME_AUDIO, FRAME_STATUS, FRAME_TEXT):
                del self.buf[:1]
                continue
            total = 7 + length
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:5 + length])
            (crc,) = struct.unpack_from("<H", self.buf, 5 + length)
            if crc16(body) != crc:
                self.crc_errors += 1
                del self.buf[:1]
                continue
            payload = body[3:]
            del self.buf[:total]
            out.append(self._decode(ftype, payload))
        return out

    def _text(self, data: bytes, out: list[Message]) -> None:
        if not data:
            return
        self.text_buf += data
        while b"\n" in self.text_buf:
            line, _, rest = self.text_buf.partition(b"\n")
            self.text_buf = bytearray(rest)
            s = line.decode("utf-8", "replace").strip()
            if s:
                out.append(Message("text", text=s))
        if len(self.text_buf) > 4096:
            self.text_buf.clear()

    @staticmethod
    def _decode(ftype: int, payload: bytes) -> Message:
        if ftype == FRAME_AUDIO:
            first, n, _chans, mode = struct.unpack_from("<IHBB", payload, 0)
            data = np.frombuffer(payload, dtype="<i2", count=2 * n, offset=8)
            return Message("audio", audio=AudioBlock(first, data[:n].copy(), data[n:].copy(), mode))
        text = payload.decode("utf-8", "replace")
        if ftype == FRAME_STATUS:
            try:
                return Message("status", status=json.loads(text))
            except json.JSONDecodeError:
                return Message("text", text=text)
        return Message("text", text=text.strip())


def decode_ble_audio(packet: bytes) -> AudioBlock:
    """BLE AUDIO notification (legacy): uint32 first_index + int16 filtered samples."""
    (first,) = struct.unpack_from("<I", packet, 0)
    filt = np.frombuffer(packet, dtype="<i2", offset=4).copy()
    return AudioBlock(first, np.zeros(0, dtype=np.int16), filt, -1)


def decode_ble_audio_rf(packet: bytes) -> AudioBlock:
    """BLE AUDIO_RF notification: uint32 first_index + int16 raw[n] + int16 filtered[n]."""
    (first,) = struct.unpack_from("<I", packet, 0)
    n = (len(packet) - 4) // 4
    both = np.frombuffer(packet, dtype="<i2", offset=4, count=2 * n)
    return AudioBlock(first, both[:n].copy(), both[n:].copy(), -1)
