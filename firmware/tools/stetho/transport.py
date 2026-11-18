"""Data sources for the PC tools: USB serial, BLE, or a synthetic demo.

All transports run in a background thread and deliver ``protocol.Message``
objects through a thread-safe queue (``transport.messages``).
"""
from __future__ import annotations

import asyncio
import json
import queue
import threading
import time

import numpy as np

from . import dsp
from .protocol import AudioBlock, FrameParser, Message, decode_ble_audio, decode_ble_audio_rf

ESPRESSIF_VID = 0x303A
BLE_NAME = "MEMS-Stetho"
BLE_AUDIO_UUID = "6e5a0002-5354-4554-484f-53434f504531"
BLE_AUDIO_RF_UUID = "6e5a0005-5354-4554-484f-53434f504531"
BLE_STATUS_UUID = "6e5a0003-5354-4554-484f-53434f504531"
BLE_CONTROL_UUID = "6e5a0004-5354-4554-484f-53434f504531"


def find_serial_port() -> str | None:
    """Pick the ESP32-S3 native USB port (VID 0x303A), else the first ACM/USB port."""
    from serial.tools import list_ports

    ports = list(list_ports.comports())
    for p in ports:
        if p.vid == ESPRESSIF_VID:
            return p.device
    for p in ports:
        if "ACM" in p.device or "USB" in p.device or p.device.upper().startswith("COM"):
            return p.device
    return None


class Transport:
    has_raw = True  # does the stream contain the raw channel?

    def __init__(self) -> None:
        self.messages: queue.Queue[Message] = queue.Queue(maxsize=10000)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)

    def send(self, command: str) -> None:
        raise NotImplementedError

    def _put(self, m: Message) -> None:
        try:
            self.messages.put_nowait(m)
        except queue.Full:
            pass

    def _run(self) -> None:
        raise NotImplementedError


class SerialTransport(Transport):
    def __init__(self, port: str | None = None, baud: int = 921600) -> None:
        super().__init__()
        self.port = port or find_serial_port()
        if not self.port:
            raise RuntimeError("no serial port found - is the stethoscope plugged in? (use --port)")
        self.baud = baud
        self._ser = None
        self.parser = FrameParser()

    def start(self) -> None:
        import serial

        self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
        self._ser.reset_input_buffer()
        super().start()
        self.send("stream bin")

    def stop(self) -> None:
        try:
            if self._ser and self._ser.is_open:
                self.send("stream log")
                time.sleep(0.1)
        except Exception:
            pass
        super().stop()
        if self._ser:
            self._ser.close()

    def send(self, command: str) -> None:
        if self._ser:
            self._ser.write((command.strip() + "\n").encode())

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._ser.read(4096)
            except Exception as e:  # unplugged
                self._put(Message("text", text=f"serial error: {e}"))
                break
            if data:
                for m in self.parser.feed(data):
                    self._put(m)


class BleTransport(Transport):
    """BLE client (bleak). Streams raw + filtered; firmware before the AUDIO_RF
    characteristic only sends the filtered channel (has_raw then becomes False)."""

    has_raw = True

    def __init__(self, address: str | None = None, name: str = BLE_NAME) -> None:
        super().__init__()
        self.address = address
        self.name = name
        self._loop: asyncio.AbstractEventLoop | None = None
        self._client = None
        self._pending: list[str] = []  # commands issued before the connection is up
        self._lock = threading.Lock()

    def send(self, command: str) -> None:
        with self._lock:
            if not (self._loop and self._client):
                self._pending.append(command)
                return
        self._write(command)

    def _write(self, command: str) -> None:
        asyncio.run_coroutine_threadsafe(
            self._client.write_gatt_char(BLE_CONTROL_UUID, command.strip().encode(), response=True),
            self._loop)

    def _run(self) -> None:
        self._loop = asyncio.new_event_loop()
        self._loop.run_until_complete(self._main())

    async def _main(self) -> None:
        from bleak import BleakClient, BleakScanner

        addr = self.address
        if not addr:
            self._put(Message("text", text=f"scanning for '{self.name}' ..."))
            dev = await BleakScanner.find_device_by_name(self.name, timeout=10.0)
            if not dev:
                self._put(Message("text", text="BLE device not found"))
                return
            addr = dev.address
        async with BleakClient(addr) as client:
            try:  # BlueZ only reports the negotiated MTU after this call
                await client._backend._acquire_mtu()  # noqa: SLF001
            except Exception:
                pass
            self._put(Message("text", text=f"BLE connected to {addr} (MTU {client.mtu_size})"))

            def on_audio(_, data: bytearray) -> None:
                self._put(Message("audio", audio=decode_ble_audio(bytes(data))))

            def on_audio_rf(_, data: bytearray) -> None:
                self._put(Message("audio", audio=decode_ble_audio_rf(bytes(data))))

            json_buf = {"s": ""}

            def on_status(_, data: bytearray) -> None:
                # JSON status is longer than one BLE packet: re-assemble chunks.
                s = bytes(data).decode("utf-8", "replace")
                if json_buf["s"] or s.startswith("{"):
                    json_buf["s"] += s
                    if json_buf["s"].rstrip().endswith("}"):
                        txt, json_buf["s"] = json_buf["s"], ""
                        try:
                            self._put(Message("status", status=json.loads(txt)))
                        except json.JSONDecodeError:
                            self._put(Message("text", text=txt))
                    elif len(json_buf["s"]) > 4096:
                        json_buf["s"] = ""
                    return
                self._put(Message("text", text=s.strip()))

            if client.services.get_characteristic(BLE_AUDIO_RF_UUID) is not None:
                await client.start_notify(BLE_AUDIO_RF_UUID, on_audio_rf)
            else:
                self.has_raw = False
                self._put(Message("text", text="old firmware: BLE streams the filtered channel only"))
                await client.start_notify(BLE_AUDIO_UUID, on_audio)
            await client.start_notify(BLE_STATUS_UUID, on_status)
            with self._lock:
                self._client = client
                pending, self._pending = self._pending, []
            for c in pending:
                await client.write_gatt_char(BLE_CONTROL_UUID, c.strip().encode(), response=True)
            while not self._stop.is_set() and client.is_connected:
                await asyncio.sleep(0.1)
        self._client = None


class DemoTransport(Transport):
    """Synthetic heart sounds processed by the reference pipeline, in real time."""

    def __init__(self, bpm: float = 72.0, snr_db: float = 15.0) -> None:
        super().__init__()
        self.bpm = bpm
        self.snr_db = snr_db
        self.mode = "heart"
        self.notch = 50.0
        self.gain = 1.0
        self._changed = threading.Event()

    def send(self, command: str) -> None:
        p = command.lower().split()
        if not p:
            return
        if p[0] == "mode" and len(p) > 1 and p[1] in dsp.MODES:
            self.mode = p[1]
        elif p[0] == "notch" and len(p) > 1:
            self.notch = 0.0 if p[1] == "off" else float(p[1])
        elif p[0] == "gain" and len(p) > 1:
            self.gain = float(p[1])
        self._changed.set()
        self._put(Message("text", text=f"demo: mode={self.mode} notch={self.notch} gain={self.gain}"))

    def _run(self) -> None:
        from scipy import signal

        fs, n = dsp.FS, 64
        x, _ = dsp_synth(self.bpm, self.snr_db)
        x = x * 0.3
        sos = dsp.user_chain_sos(self.mode, self.notch)
        zi = np.zeros((sos.shape[0], 2))
        idx, t_next, last_status = 0, time.monotonic(), 0.0
        hist: list[np.ndarray] = []
        while not self._stop.is_set():
            if self._changed.is_set():
                self._changed.clear()
                sos = dsp.user_chain_sos(self.mode, self.notch)
                zi = np.zeros((sos.shape[0], 2))
            seg = x[(idx + np.arange(n)) % len(x)]
            filt, zi = signal.sosfilt(sos, seg, zi=zi)
            filt *= self.gain
            to16 = lambda v: np.clip(np.round(v * 32767), -32767, 32767).astype(np.int16)  # noqa: E731
            self._put(Message("audio", audio=AudioBlock(idx, to16(seg), to16(filt),
                                                        dsp.MODE_NAMES.index(self.mode))))
            hist.append(seg)
            idx += n
            now = time.monotonic()
            if now - last_status >= 1.0:
                last_status = now
                tail = np.concatenate(hist)[-dsp.FS * dsp.HR_WINDOW_S:]
                hist = [tail]
                hr = dsp.HrEstimate(0, 0, False)
                if len(tail) >= dsp.FS * dsp.HR_WINDOW_S:
                    y = signal.sosfilt(dsp.hr_chain_sos(self.notch), tail)
                    hr = dsp.hr_from_envelope(dsp.shannon_envelope(y, causal_norm=False))
                lo, hi = dsp.MODES[self.mode]
                self._put(Message("status", status={
                    "t": "status", "fw": "demo", "mode": self.mode, "hp": lo, "lp": hi,
                    "notch": self.notch, "gain": self.gain, "hr": hr.bpm, "hr_conf": hr.confidence,
                    "hr_ok": int(hr.valid), "fs": fs, "fs_adc": dsp.FS_ADC,
                    "rms": 20 * np.log10(np.std(tail) + 1e-9), "dc_mv": 1650, "clip": 0}))
            t_next += n / fs
            time.sleep(max(0.0, t_next - time.monotonic()))


def dsp_synth(bpm: float, snr_db: float):
    from .synth import synth_pcg

    return synth_pcg(duration_s=60, fs=dsp.FS, bpm=bpm, snr_db=snr_db, hum_hz=50, hum_rel=0.5, seed=42)


class StreamAssembler:
    """Re-assembles audio blocks into continuous arrays, zero-filling lost blocks."""

    def __init__(self) -> None:
        self.expected: int | None = None
        self.lost = 0
        self.received = 0

    def add(self, blk: AudioBlock) -> tuple[np.ndarray, np.ndarray]:
        raw = blk.raw.astype(np.float32) / 32767.0
        filt = blk.filtered.astype(np.float32) / 32767.0
        if raw.size == 0:
            raw = np.full_like(filt, np.nan)
        gap = 0
        if self.expected is not None:
            gap = (blk.first_index - self.expected) & 0xFFFFFFFF
            if gap > 10 * dsp.FS:  # device restarted / counter reset
                gap = 0
        if gap:
            self.lost += gap
            raw = np.concatenate([np.zeros(gap, np.float32), raw])
            filt = np.concatenate([np.zeros(gap, np.float32), filt])
        self.expected = (blk.first_index + len(blk.filtered)) & 0xFFFFFFFF
        self.received += len(blk.filtered)
        return raw, filt


def make_transport(kind: str, port: str | None = None, ble_address: str | None = None) -> Transport:
    if kind == "serial":
        return SerialTransport(port)
    if kind == "ble":
        return BleTransport(ble_address)
    if kind == "demo":
        return DemoTransport()
    raise ValueError(kind)
