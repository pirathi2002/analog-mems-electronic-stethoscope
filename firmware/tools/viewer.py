#!/usr/bin/env python3
"""Real-time heart-sound viewer for the MEMS electronic stethoscope.

* scrolling raw and filtered phonocardiogram (PCG)
* live spectrum (raw vs filtered)
* heart rate + signal-quality indicators from the device
* filter mode / notch / gain controls (sent to the device)
* record to WAV, listen through the PC speakers / headphones

Usage:
    python viewer.py                  # USB (auto-detects the ESP32-S3 port)
    python viewer.py --port /dev/ttyACM0
    python viewer.py --ble            # Bluetooth LE (raw + filtered)
    python viewer.py --demo           # synthetic heart sounds, no hardware
"""
from __future__ import annotations

import argparse
import queue
import sys
import threading
from datetime import datetime
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
from scipy import signal

from stetho import dsp
from stetho.recio import save_recording
from stetho.transport import StreamAssembler, make_transport

FS = dsp.FS


class AudioOut:
    """Plays the filtered stream through the default sound device (optional)."""

    OUT_FS = 48000
    UP = OUT_FS // FS  # 12

    def __init__(self) -> None:
        import sounddevice as sd  # optional dependency

        self.h = signal.firwin(16 * self.UP + 1, 1800, fs=self.OUT_FS) * self.UP
        self.zi = np.zeros(len(self.h) - 1)
        self.buf = np.zeros(0, np.float32)
        self.lock = threading.Lock()
        self.volume = 10.0
        self.stream = sd.OutputStream(samplerate=self.OUT_FS, channels=1, dtype="float32",
                                      callback=self._cb, blocksize=1024, latency="low")
        self.stream.start()

    def feed(self, x: np.ndarray) -> None:
        up = np.zeros(len(x) * self.UP)
        up[:: self.UP] = x
        y, self.zi = signal.lfilter(self.h, 1.0, up, zi=self.zi)
        y = np.tanh(y * self.volume).astype(np.float32)  # soft limiter protects ears/speakers
        with self.lock:
            self.buf = np.concatenate([self.buf, y])[-self.OUT_FS:]  # max 1 s latency

    def _cb(self, out, frames, _time, _status) -> None:
        with self.lock:
            n = min(frames, len(self.buf))
            out[:n, 0] = self.buf[:n]
            out[n:, 0] = 0
            self.buf = self.buf[n:]

    def close(self) -> None:
        self.stream.stop()
        self.stream.close()


class Viewer(QtWidgets.QMainWindow):
    def __init__(self, transport, window_s: float = 5.0) -> None:
        super().__init__()
        self.tr = transport
        self.n = int(window_s * FS)
        self.raw = np.zeros(self.n, np.float32)
        self.filt = np.zeros(self.n, np.float32)
        self.t = (np.arange(self.n) - self.n) / FS
        self.asm = StreamAssembler()
        self.recording: list[tuple[np.ndarray, np.ndarray]] | None = None
        self.rec_status: list[dict] = []
        self.last_status: dict = {}
        self.audio: AudioOut | None = None

        self.setWindowTitle("MEMS Electronic Stethoscope")
        self.resize(1400, 850)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        lay = QtWidgets.QHBoxLayout(central)

        pg.setConfigOptions(antialias=False, background="w", foreground="k")
        gw = pg.GraphicsLayoutWidget()
        lay.addWidget(gw, stretch=4)
        self.p_raw = gw.addPlot(row=0, col=0, title="Raw PCG (DC removed, unfiltered)")
        self.p_filt = gw.addPlot(row=1, col=0, title="Filtered PCG")
        self.p_psd = gw.addPlot(row=2, col=0, title="Spectrum (last 2 s)")
        for p in (self.p_raw, self.p_filt):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel("bottom", "time", "s")
            p.setLabel("left", "amplitude", "FS")
        self.p_filt.setXLink(self.p_raw)
        self.c_raw = self.p_raw.plot(self.t, self.raw, pen=pg.mkPen("#1f77b4", width=1))
        self.c_filt = self.p_filt.plot(self.t, self.filt, pen=pg.mkPen("#d62728", width=1))
        self.p_psd.setLogMode(x=True, y=False)
        self.p_psd.showGrid(x=True, y=True, alpha=0.3)
        self.p_psd.setLabel("bottom", "frequency", "Hz")
        self.p_psd.setLabel("left", "PSD", "dB")
        self.p_psd.addLegend()
        self.c_psd_raw = self.p_psd.plot(pen=pg.mkPen("#1f77b4"), name="raw")
        self.c_psd_filt = self.p_psd.plot(pen=pg.mkPen("#d62728"), name="filtered")
        self.p_psd.setXRange(np.log10(10), np.log10(FS / 2))

        panel = QtWidgets.QVBoxLayout()
        lay.addLayout(panel, stretch=1)
        self.lbl_hr = QtWidgets.QLabel("--- bpm")
        f = QtGui.QFont()
        f.setPointSize(36)
        f.setBold(True)
        self.lbl_hr.setFont(f)
        self.lbl_hr.setAlignment(QtCore.Qt.AlignCenter)
        panel.addWidget(self.lbl_hr)
        self.lbl_conf = QtWidgets.QLabel("confidence -")
        self.lbl_conf.setAlignment(QtCore.Qt.AlignCenter)
        panel.addWidget(self.lbl_conf)

        form = QtWidgets.QFormLayout()
        self.cb_mode = QtWidgets.QComboBox()
        self.cb_mode.addItems(["heart (25-400 Hz)", "bell (20-200 Hz)", "diaphragm (100-500 Hz)",
                               "wide (20-1000 Hz)", "raw (no filter)"])
        self.cb_mode.currentTextChanged.connect(lambda t: self.tr.send(f"mode {t.split()[0]}"))
        form.addRow("Filter mode", self.cb_mode)
        self.cb_notch = QtWidgets.QComboBox()
        self.cb_notch.addItems(["50", "60", "off"])
        self.cb_notch.currentTextChanged.connect(lambda t: self.tr.send(f"notch {t}"))
        form.addRow("Mains notch (Hz)", self.cb_notch)
        self.sp_gain = QtWidgets.QDoubleSpinBox()
        self.sp_gain.setRange(0.1, 100)
        self.sp_gain.setValue(1.0)
        self.sp_gain.setSingleStep(0.5)
        self.sp_gain.editingFinished.connect(lambda: self.tr.send(f"gain {self.sp_gain.value():.2f}"))
        form.addRow("Digital gain", self.sp_gain)
        self.chk_auto = QtWidgets.QCheckBox("auto-scale Y")
        self.chk_auto.setChecked(True)
        form.addRow(self.chk_auto)
        panel.addLayout(form)

        self.btn_rec = QtWidgets.QPushButton("● Record")
        self.btn_rec.setCheckable(True)
        self.btn_rec.toggled.connect(self.toggle_record)
        panel.addWidget(self.btn_rec)

        hl = QtWidgets.QHBoxLayout()
        self.chk_listen = QtWidgets.QCheckBox("Listen")
        self.chk_listen.toggled.connect(self.toggle_listen)
        hl.addWidget(self.chk_listen)
        self.sl_vol = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sl_vol.setRange(0, 40)
        self.sl_vol.setValue(20)
        self.sl_vol.setToolTip("playback volume (0-40 dB)")
        hl.addWidget(self.sl_vol)
        panel.addLayout(hl)

        self.lbl_status = QtWidgets.QLabel()
        self.lbl_status.setFont(QtGui.QFont("monospace", 9))
        self.lbl_status.setTextFormat(QtCore.Qt.RichText)
        panel.addWidget(self.lbl_status)

        cl = QtWidgets.QHBoxLayout()
        self.ed_cmd = QtWidgets.QLineEdit()
        self.ed_cmd.setPlaceholderText("command (help, status, band 30 300 ...)")
        self.ed_cmd.returnPressed.connect(self.send_cmd)
        cl.addWidget(self.ed_cmd)
        panel.addLayout(cl)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.log.setFont(QtGui.QFont("monospace", 8))
        panel.addWidget(self.log, stretch=1)

        if not self.tr.has_raw:
            self.p_raw.setTitle("Raw PCG (not available over BLE)")

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)
        self.psd_counter = 0

    # ------------------------------------------------------------------
    def send_cmd(self) -> None:
        c = self.ed_cmd.text().strip()
        if c:
            self.log.appendPlainText(f"> {c}")
            self.tr.send(c)
        self.ed_cmd.clear()

    def toggle_listen(self, on: bool) -> None:
        if on:
            try:
                self.audio = AudioOut()
            except Exception as e:  # sounddevice / PortAudio missing
                self.log.appendPlainText(f"audio unavailable: {e}")
                self.chk_listen.setChecked(False)
        elif self.audio:
            self.audio.close()
            self.audio = None

    def toggle_record(self, on: bool) -> None:
        if on:
            self.recording = []
            self.rec_status = []
            self.btn_rec.setText("■ Stop (recording…)")
            self.btn_rec.setStyleSheet("background:#d62728;color:white")
        else:
            chunks, self.recording = self.recording or [], None
            self.btn_rec.setText("● Record")
            self.btn_rec.setStyleSheet("")
            if not chunks:
                return
            raw = np.concatenate([c[0] for c in chunks])
            filt = np.concatenate([c[1] for c in chunks])
            path = Path("recordings") / datetime.now().strftime("pcg_%Y%m%d_%H%M%S.wav")
            save_recording(path, FS, filt, raw if self.tr.has_raw else None,
                           {"source": "viewer", "settings": self.last_status,
                            "status_log": self.rec_status, "lost_samples": self.asm.lost})
            self.log.appendPlainText(f"saved {path} ({len(filt) / FS:.1f} s)")

    # ------------------------------------------------------------------
    def tick(self) -> None:
        new_r, new_f = [], []
        for _ in range(2000):
            try:
                m = self.tr.messages.get_nowait()
            except queue.Empty:
                break
            if m.kind == "audio":
                r, f = self.asm.add(m.audio)
                new_r.append(r)
                new_f.append(f)
            elif m.kind == "status":
                self.on_status(m.status)
            elif m.kind == "text":
                self.log.appendPlainText(m.text)
        if new_f:
            r = np.concatenate(new_r)
            f = np.concatenate(new_f)
            k = min(len(f), self.n)
            self.raw = np.concatenate([self.raw[k:], np.nan_to_num(r[-k:])])
            self.filt = np.concatenate([self.filt[k:], f[-k:]])
            if self.recording is not None:
                self.recording.append((r, f))
            if self.audio:
                self.audio.volume = 10 ** (self.sl_vol.value() / 20)
                self.audio.feed(f)
            self.c_raw.setData(self.t, self.raw)
            self.c_filt.setData(self.t, self.filt)
            if self.chk_auto.isChecked():
                for p, y in ((self.p_raw, self.raw), (self.p_filt, self.filt)):
                    a = max(1e-4, float(np.percentile(np.abs(y), 99.9)) * 1.2)
                    p.setYRange(-a, a, padding=0)
            self.psd_counter += 1
            if self.psd_counter % 10 == 0:
                self.update_psd()

    def update_psd(self) -> None:
        seg = 2 * FS
        f, p = signal.welch(self.filt[-seg:], FS, nperseg=1024)
        ok = f > 0
        self.c_psd_filt.setData(f[ok], 10 * np.log10(p[ok] + 1e-16))
        if self.tr.has_raw:
            _, pr = signal.welch(self.raw[-seg:], FS, nperseg=1024)
            self.c_psd_raw.setData(f[ok], 10 * np.log10(pr[ok] + 1e-16))

    def on_status(self, s: dict) -> None:
        self.last_status = s
        if self.recording is not None:
            self.rec_status.append(s)
        if s.get("hr_ok"):
            self.lbl_hr.setText(f"{s.get('hr', 0):.0f} bpm")
            self.lbl_hr.setStyleSheet("color:#2ca02c")
        else:
            self.lbl_hr.setText("--- bpm")
            self.lbl_hr.setStyleSheet("color:#888")
        self.lbl_conf.setText(f"confidence {s.get('hr_conf', 0):.2f}")
        self.p_filt.setTitle(f"Filtered PCG - {s.get('mode', '?')} "
                             f"({s.get('hp', 0):.0f}-{s.get('lp', 0):.0f} Hz, notch {s.get('notch', 0):.0f} Hz, "
                             f"gain {s.get('gain', 1):.2f})")
        clip = s.get("clip", 0)
        dc = s.get("dc_mv", 0)
        warn = []
        if clip:
            warn.append("<span style='color:red'>ADC clipping: turn gain trimmer R2 down</span>")
        if dc and not 1300 <= dc <= 2000:
            warn.append(f"<span style='color:red'>bias {dc} mV (expected ~1650): check AFE / Vref</span>")
        rows = [
            ("level raw", f"{s.get('rms', -99):.1f} dBFS"),
            ("level filtered", f"{s.get('rms_f', -99):.1f} dBFS"),
            ("peak", f"{100 * s.get('peak', 0):.0f} % FS"),
            ("AFE bias", f"{dc} mV"),
            ("clipped samples", f"{clip}/s"),
            ("ADC rate", f"{s.get('fs_adc', 0):.0f} Hz"),
            ("lost samples", f"{self.asm.lost}"),
            ("BLE", "connected" if s.get("ble_conn") else ("on" if s.get("ble") else "off")),
            ("firmware", s.get("fw", "?")),
        ]
        html = "<table>" + "".join(f"<tr><td>{k}&nbsp;</td><td><b>{v}</b></td></tr>" for k, v in rows) + "</table>"
        self.lbl_status.setText(html + "<br>".join(warn))

    def closeEvent(self, e) -> None:  # noqa: N802
        if self.recording is not None:
            self.btn_rec.setChecked(False)
        if self.audio:
            self.audio.close()
        self.tr.stop()
        super().closeEvent(e)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (auto-detected by default)")
    ap.add_argument("--ble", action="store_true", help="connect over Bluetooth LE")
    ap.add_argument("--ble-address", help="BLE address (default: scan for MEMS-Stetho)")
    ap.add_argument("--demo", action="store_true", help="synthetic heart sounds, no hardware")
    ap.add_argument("--window", type=float, default=5.0, help="seconds shown (default 5)")
    args = ap.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    kind = "demo" if args.demo else "ble" if args.ble else "serial"
    tr = make_transport(kind, args.port, args.ble_address)
    tr.start()
    w = Viewer(tr, args.window)
    w.show()
    return app.exec_() if hasattr(app, "exec_") else app.exec()


if __name__ == "__main__":
    sys.exit(main())
