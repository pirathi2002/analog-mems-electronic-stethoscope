#!/usr/bin/env python3
"""Real-time heart-sound viewer for the MEMS electronic stethoscope.

* scrolling raw and filtered phonocardiogram (PCG)
* live spectrum (raw vs filtered)
* heart rate + signal-quality indicators from the device
* filter mode / notch / gain controls (sent to the device)
* record to WAV, listen through the PC speakers / headphones
* noise reduction on the PC (right-hand panel): "Calibrate noise" records the
  noise with the chest-piece off the body; afterwards the Bluetooth
  interference pulses and the steady background noise are subtracted from
  every new sample, shown as "Denoised PCG", with a heart rate computed on the
  PC from the denoised signal (see stetho/denoise.py)

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
from stetho.denoise import Denoiser
from stetho.recio import save_recording
from stetho.transport import StreamAssembler, make_transport

FS = dsp.FS
LP_CUTOFF_HZ = 400.0  # extra "Denoised, low-pass" panel: cut-off frequency
LP_ORDER = 8          # Butterworth order of that low-pass (48 dB/octave above the cut-off)


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
        self.dn = Denoiser()
        self.den = np.zeros(self.n, np.float32)
        self.rec_den: list[np.ndarray] = []
        self._den_samples = 0
        self.lp = np.zeros(self.n, np.float32)
        self._lp_sos = dsp.butter_sos("lp", FS, LP_CUTOFF_HZ, LP_ORDER)
        self._lp_zi = np.zeros((self._lp_sos.shape[0], 2))

        self.setWindowTitle("MEMS Electronic Stethoscope")
        self.resize(1680, 1000)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        lay = QtWidgets.QHBoxLayout(central)

        pg.setConfigOptions(antialias=False, background="w", foreground="k")
        gw = pg.GraphicsLayoutWidget()
        lay.addWidget(gw, stretch=4)
        self.p_raw = gw.addPlot(row=0, col=0, title="Raw PCG (DC removed, unfiltered)")
        self.p_filt = gw.addPlot(row=1, col=0, title="Filtered PCG")
        self.p_den = gw.addPlot(row=2, col=0, title="Denoised PCG (PC) - press 'Calibrate noise' on the right")
        self.p_lp = gw.addPlot(row=3, col=0,
                               title=f"Denoised, low-pass {LP_CUTOFF_HZ:.0f} Hz (everything above "
                                     f"{LP_CUTOFF_HZ:.0f} Hz cut) - after calibration")
        spectra = gw.addLayout(row=4, col=0)  # the two spectra side by side, equal width
        self.p_psd = spectra.addPlot(row=0, col=0, title="Spectrum (last 2 s)")
        self.p_cal = spectra.addPlot(row=0, col=1, title="Calibration noise spectrum")
        for i, p in enumerate((self.p_psd, self.p_cal)):
            p.setPreferredWidth(100)  # same size hint, so the stretch splits the width evenly
            spectra.layout.setColumnStretchFactor(i, 1)
        for p in (self.p_raw, self.p_filt, self.p_den, self.p_lp):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel("bottom", "time", "s")
            p.setLabel("left", "amplitude", "FS")
        self.p_filt.setXLink(self.p_raw)
        self.p_den.setXLink(self.p_raw)
        self.p_lp.setXLink(self.p_raw)
        self.c_raw = self.p_raw.plot(self.t, self.raw, pen=pg.mkPen("#1f77b4", width=1))
        self.c_filt = self.p_filt.plot(self.t, self.filt, pen=pg.mkPen("#d62728", width=1))
        self.c_den = self.p_den.plot(self.t, self.den, pen=pg.mkPen("#2ca02c", width=1))
        self.c_lp = self.p_lp.plot(self.t, self.lp, pen=pg.mkPen("#9467bd", width=1))
        self.p_cal.setLogMode(x=True, y=False)
        self.p_cal.showGrid(x=True, y=True, alpha=0.3)
        self.p_cal.setLabel("bottom", "frequency", "Hz")
        self.p_cal.setLabel("left", "PSD", "dB")
        self.p_cal.addLegend()
        self.c_cal_raw = self.p_cal.plot(pen=pg.mkPen("#888888"), name="calibration, raw")
        self.c_cal_clean = self.p_cal.plot(pen=pg.mkPen("#2ca02c"), name="calibration, pulses removed")
        self.p_cal.setXRange(np.log10(10), np.log10(FS / 2))
        self.p_psd.setLogMode(x=True, y=False)
        self.p_psd.showGrid(x=True, y=True, alpha=0.3)
        self.p_psd.setLabel("bottom", "frequency", "Hz")
        self.p_psd.setLabel("left", "PSD", "dB")
        self.p_psd.addLegend()
        self.c_psd_raw = self.p_psd.plot(pen=pg.mkPen("#1f77b4"), name="raw")
        self.c_psd_filt = self.p_psd.plot(pen=pg.mkPen("#d62728"), name="filtered")
        self.c_psd_den = self.p_psd.plot(pen=pg.mkPen("#2ca02c"), name="denoised")
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

        lay.addLayout(self._build_noise_panel(), stretch=1)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)
        self.psd_counter = 0

    # ------------------------------------------------------------------
    def _build_noise_panel(self) -> QtWidgets.QVBoxLayout:
        """Right-hand column: noise calibration, noise reduction, PC heart rate."""
        col = QtWidgets.QVBoxLayout()

        g_cal = QtWidgets.QGroupBox("1. Noise calibration")
        v = QtWidgets.QVBoxLayout(g_cal)
        how = QtWidgets.QLabel(
            "Take the chest-piece <b>off the body</b> and keep it still. "
            "Keep the link as it will be during the measurement (Bluetooth on/off), "
            "then press <b>Calibrate noise</b>. Calibrate again after changing the "
            "link or the position of the board.")
        how.setWordWrap(True)
        v.addWidget(how)
        hl = QtWidgets.QHBoxLayout()
        hl.addWidget(QtWidgets.QLabel("Duration"))
        self.sp_cal = QtWidgets.QDoubleSpinBox()
        self.sp_cal.setRange(3, 30)
        self.sp_cal.setValue(5)
        self.sp_cal.setSuffix(" s")
        hl.addWidget(self.sp_cal)
        v.addLayout(hl)
        self.btn_cal = QtWidgets.QPushButton("Calibrate noise")
        self.btn_cal.clicked.connect(self.start_calibration)
        v.addWidget(self.btn_cal)
        self.pb_cal = QtWidgets.QProgressBar()
        self.pb_cal.setRange(0, 100)
        self.pb_cal.setValue(0)
        v.addWidget(self.pb_cal)
        self.lbl_cal = QtWidgets.QLabel("not calibrated")
        self.lbl_cal.setWordWrap(True)
        self.lbl_cal.setTextFormat(QtCore.Qt.RichText)
        v.addWidget(self.lbl_cal)
        col.addWidget(g_cal)

        g_nr = QtWidgets.QGroupBox("2. Noise reduction (PC)")
        f = QtWidgets.QFormLayout(g_nr)
        self.chk_pulse = QtWidgets.QCheckBox("Remove Bluetooth pulses")
        self.chk_pulse.setChecked(True)
        self.chk_pulse.toggled.connect(lambda on: setattr(self.dn, "pulse_on", on))
        f.addRow(self.chk_pulse)
        self.chk_spec = QtWidgets.QCheckBox("Subtract background noise")
        self.chk_spec.setChecked(True)
        self.chk_spec.toggled.connect(lambda on: setattr(self.dn, "spectral_on", on))
        f.addRow(self.chk_spec)
        self.sl_strength = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sl_strength.setRange(5, 30)  # x0.1
        self.sl_strength.setValue(15)
        self.sl_strength.setToolTip("over-subtraction factor of the background noise (0.5-3.0)")
        self.lbl_strength = QtWidgets.QLabel("1.5")
        self.sl_strength.valueChanged.connect(self._set_strength)
        sh = QtWidgets.QHBoxLayout()
        sh.addWidget(self.sl_strength)
        sh.addWidget(self.lbl_strength)
        f.addRow("Strength", sh)
        self.chk_listen_den = QtWidgets.QCheckBox("Listen to the denoised signal")
        self.chk_listen_den.setChecked(True)
        f.addRow(self.chk_listen_den)
        self.chk_save_den = QtWidgets.QCheckBox("Save denoised signal with recordings")
        self.chk_save_den.setChecked(True)
        f.addRow(self.chk_save_den)
        self.lbl_nr = QtWidgets.QLabel("-")
        self.lbl_nr.setFont(QtGui.QFont("monospace", 9))
        self.lbl_nr.setWordWrap(True)
        f.addRow(self.lbl_nr)
        col.addWidget(g_nr)

        g_hr = QtWidgets.QGroupBox("3. Heart rate from the denoised signal (PC)")
        v = QtWidgets.QVBoxLayout(g_hr)
        self.lbl_hr_pc = QtWidgets.QLabel("--- bpm")
        fnt = QtGui.QFont()
        fnt.setPointSize(28)
        fnt.setBold(True)
        self.lbl_hr_pc.setFont(fnt)
        self.lbl_hr_pc.setAlignment(QtCore.Qt.AlignCenter)
        v.addWidget(self.lbl_hr_pc)
        self.lbl_hr_pc_info = QtWidgets.QLabel("needs 8 s of signal after calibration")
        self.lbl_hr_pc_info.setWordWrap(True)
        self.lbl_hr_pc_info.setTextFormat(QtCore.Qt.RichText)
        self.lbl_hr_pc_info.setAlignment(QtCore.Qt.AlignCenter)
        v.addWidget(self.lbl_hr_pc_info)
        f2 = QtWidgets.QFormLayout()
        self.sp_maxbpm = QtWidgets.QSpinBox()
        self.sp_maxbpm.setRange(60, 200)
        self.sp_maxbpm.setValue(int(self.dn.max_bpm))
        self.sp_maxbpm.setSuffix(" bpm")
        self.sp_maxbpm.setToolTip("Upper limit of the search. At rest keep 120: a higher limit lets the "
                                  "S1-S2 gap be counted as a beat (double the true rate). Raise it after exercise.")
        self.sp_maxbpm.valueChanged.connect(lambda v: setattr(self.dn, "max_bpm", float(v)))
        f2.addRow("Max expected HR", self.sp_maxbpm)
        self.chk_status = QtWidgets.QCheckBox("Ignore the 1-per-second status bursts")
        self.chk_status.setChecked(True)
        self.chk_status.setToolTip("The device's status message (once per second over Bluetooth) disturbs "
                                   "the signal; left in, it looks like a 60 bpm heartbeat.")
        self.chk_status.toggled.connect(lambda on: setattr(self.dn, "status_on", on))
        f2.addRow(self.chk_status)
        v.addLayout(f2)
        col.addWidget(g_hr)
        col.addStretch(1)
        return col

    def _set_strength(self, v: int) -> None:
        self.dn.strength = v / 10
        self.lbl_strength.setText(f"{v / 10:.1f}")

    def start_calibration(self) -> None:
        if self.dn.calibrating:
            self.dn.cancel_calibration()
            self.btn_cal.setText("Calibrate noise")
            self.lbl_cal.setText("calibration cancelled")
            return
        self.dn.start_calibration(self.sp_cal.value())
        self.den[:] = 0
        self.lp[:] = 0
        self._lp_zi = np.zeros((self._lp_sos.shape[0], 2))
        self.btn_cal.setText("Cancel calibration")
        self.lbl_cal.setText("recording noise... keep the chest-piece off the body and still")
        self.log.appendPlainText(f"noise calibration started ({self.sp_cal.value():.0f} s)")

    def _calibration_done(self) -> None:
        self.btn_cal.setText("Calibrate noise")
        self.pb_cal.setValue(100)
        i = self.dn.info
        parts = []
        if self.dn.pulses is not None:
            parts.append(f"Bluetooth pulses: <b>{i.get('pulses_per_s', 0):.0f}/s</b> learned")
        else:
            parts.append("no Bluetooth pulses found (pulse removal off)")
        parts.append(f"noise before: {i.get('rms_before_dbfs', 0):.1f} dBFS")
        parts.append(f"after pulse removal: {i.get('noise_rms_dbfs', 0):.1f} dBFS")
        self.lbl_cal.setText("<span style='color:#2ca02c'><b>calibrated</b></span> "
                             f"({i.get('seconds', 0):.0f} s)<br>" + "<br>".join(parts))
        self.log.appendPlainText("noise calibration done: " + ", ".join(
            f"{k}={v:.3g}" for k, v in i.items() if isinstance(v, (int, float))))
        self._show_calibration_spectrum()
        self.p_den.setTitle("Denoised PCG (PC: Bluetooth pulses + background noise removed)")

    def _show_calibration_spectrum(self) -> None:
        """Noise spectrum of the calibration recording, before and after pulse removal."""
        x, c = self.dn.calib_raw, self.dn.calib_clean
        if x is None or len(x) < 1024:
            return
        f, p = signal.welch(x, FS, nperseg=1024)
        ok = f > 0
        self.c_cal_raw.setData(f[ok], 10 * np.log10(p[ok] + 1e-16))
        if self.dn.pulses is not None and c is not None:
            _, pc = signal.welch(c, FS, nperseg=1024)
            self.c_cal_clean.setData(f[ok], 10 * np.log10(pc[ok] + 1e-16))
        else:
            self.c_cal_clean.setData([], [])
        self.p_cal.setTitle(f"Calibration noise spectrum ({len(x) / FS:.0f} s)")

    def _update_noise_status(self) -> None:
        if self.dn.calibrating:
            self.pb_cal.setValue(int(100 * self.dn.progress))
            return
        if not self.dn.calibrated:
            return
        pr = self.dn.pulses
        dur = max(1e-9, self._den_samples / FS)
        txt = []
        if pr is not None:
            txt.append(f"pulses removed  {pr.removed / dur:5.1f}/s")
            txt.append(f"bursts cut      {pr.bursts / dur:5.1f}/s")
            txt.append(f"analog gain     x{pr.gain_ratio:.2f} vs calib.")
            txt.append(f"1-s status lock {'yes' if self.dn.status.locked else 'searching...'}")
        txt.append(f"latency         {self.dn.delay / FS * 1000:.0f} ms")
        self.lbl_nr.setText("\n".join(txt))
        r = self.dn.hr.result
        if len(self.dn.hr.buf) < dsp.HR_WINDOW_S * FS:
            self.lbl_hr_pc_info.setText(f"collecting signal... {len(self.dn.hr.buf) / FS:.0f}/{dsp.HR_WINDOW_S} s")
            return
        hr = self.dn.hr
        if hr.history and hr.rejected == 0:
            self.lbl_hr_pc.setText(f"{hr.bpm:.0f} bpm")
            self.lbl_hr_pc.setStyleSheet("color:#2ca02c")
        elif hr.history and hr.rejected <= 5:  # keep the last value for a few seconds, greyed
            self.lbl_hr_pc.setText(f"{hr.bpm:.0f} bpm")
            self.lbl_hr_pc.setStyleSheet("color:#8fbf8f")
        else:
            self.lbl_hr_pc.setText("--- bpm")
            self.lbl_hr_pc.setStyleSheet("color:#888")
        info = f"confidence {r.confidence:.2f} &middot; pulsatility {r.pulsatility:.2f} (needs &ge; {dsp.HR_MIN_PULSATILITY})"
        if hr.history and hr.rejected:
            info += f"<br>last {hr.rejected} window(s) rejected; showing the last valid value"
        status_handled = self.dn.status_on and self.dn.status.locked
        if r.valid and abs(r.bpm - 60.0) < 2.0 and not status_handled:
            info += ("<br><span style='color:#d62728'>about 60 bpm: the device's 1-per-second status "
                     "message can also cause this. Check with a pulse count.</span>")
        self.lbl_hr_pc_info.setText(info)

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
            self.rec_den = []
            self.btn_rec.setText("■ Stop (recording…)")
            self.btn_rec.setStyleSheet("background:#d62728;color:white")
        else:
            chunks, self.recording = self.recording or [], None
            den_chunks, self.rec_den = self.rec_den, []
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
            if den_chunks and self.chk_save_den.isChecked():
                den = np.concatenate(den_chunks)
                p2 = path.with_name(path.stem + "_denoised.wav")
                save_recording(p2, FS, den, None, {
                    "source": "viewer (PC noise reduction)", "denoised_from": path.name,
                    "delay_samples": self.dn.delay,
                    "note": "starts delay_samples later than the main recording",
                    "pulse_removal": self.dn.pulse_on and self.dn.pulses is not None,
                    "background_subtraction": self.dn.spectral_on and self.dn.spectral is not None,
                    "strength": self.dn.strength, "calibration": self.dn.info,
                    "settings": self.last_status})
                self.log.appendPlainText(f"saved {p2}")

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
            d = self._denoise(r)
            if self.audio:
                self.audio.volume = 10 ** (self.sl_vol.value() / 20)
                use_den = self.chk_listen_den.isChecked() and self.dn.calibrated
                if use_den and d is not None:
                    self.audio.feed(d)
                elif not use_den:
                    self.audio.feed(f)
            self.c_raw.setData(self.t, self.raw)
            self.c_filt.setData(self.t, self.filt)
            self.c_den.setData(self.t - self.dn.delay / FS, self.den)
            self.c_lp.setData(self.t - self.dn.delay / FS, self.lp)
            if self.chk_auto.isChecked():
                for p, y in ((self.p_raw, self.raw), (self.p_filt, self.filt), (self.p_den, self.den),
                             (self.p_lp, self.lp)):
                    a = max(1e-4, float(np.percentile(np.abs(y), 99.9)) * 1.2)
                    p.setYRange(-a, a, padding=0)
            self.psd_counter += 1
            if self.psd_counter % 10 == 0:
                self.update_psd()
                self._update_noise_status()

    def _denoise(self, r: np.ndarray) -> np.ndarray | None:
        """Feeds the RAW samples to the PC noise reduction; returns new denoised samples."""
        if np.isnan(r).all():  # firmware without the raw channel over BLE
            if self.dn.calibrating:
                self.dn.cancel_calibration()
                self.btn_cal.setText("Calibrate noise")
                self.lbl_cal.setText("<span style='color:red'>needs the raw channel: update the firmware</span>")
            return None
        was_calibrating = self.dn.calibrating
        d = self.dn.feed(r)
        if was_calibrating and not self.dn.calibrating and self.dn.calibrated:
            self._den_samples = 0
            self._calibration_done()
        if d is None or len(d) == 0:
            return None
        d = d.astype(np.float32)
        self._den_samples = getattr(self, "_den_samples", 0) + len(d)
        k = min(len(d), self.n)
        self.den = np.concatenate([self.den[k:], d[-k:]])
        # extra panel: the cleaned signal (pulses + background removed) with only a sharp low-pass
        lp, self._lp_zi = signal.sosfilt(self._lp_sos, self.dn.last_clean, zi=self._lp_zi)
        k = min(len(lp), self.n)
        self.lp = np.concatenate([self.lp[k:], lp[-k:].astype(np.float32)])
        if self.recording is not None:
            self.rec_den.append(d)
        return d

    def update_psd(self) -> None:
        seg = 2 * FS
        f, p = signal.welch(self.filt[-seg:], FS, nperseg=1024)
        ok = f > 0
        self.c_psd_filt.setData(f[ok], 10 * np.log10(p[ok] + 1e-16))
        if self.tr.has_raw:
            _, pr = signal.welch(self.raw[-seg:], FS, nperseg=1024)
            self.c_psd_raw.setData(f[ok], 10 * np.log10(pr[ok] + 1e-16))
        if self.dn.calibrated:
            _, pd = signal.welch(self.den[-seg:], FS, nperseg=1024)
            self.c_psd_den.setData(f[ok], 10 * np.log10(pd[ok] + 1e-16))

    def on_status(self, s: dict) -> None:
        self.last_status = s
        self.dn.update_settings(s)
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
