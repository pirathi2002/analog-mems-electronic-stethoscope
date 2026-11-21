"""PC-side noise reduction with a calibration step (used by viewer.py).

The stethoscope picks up two kinds of noise that can be learned while the
chest-piece is *not* on the body ("calibration"):

1. **Interference pulses.** Each Bluetooth radio burst of the ESP32 pulls the
   supply and adds a short square pulse (~2.5 ms, 10-12.5 ms apart) to the
   analog signal. The pulses all have the same shape but their timing jitters,
   so a notch or comb filter cannot remove them. ``PulseRemover`` learns the
   pulse shape (median of all pulses in the calibration), then finds every
   pulse in the live RAW signal by its steep rising edge (heart sounds are far
   too slow to produce such an edge) and subtracts the template, scaled to the
   pulse's own height.

2. **Steady background noise** (room noise, hiss, hum that is left). The
   ``SpectralSubtractor`` learns the average noise power spectrum and reduces
   every frequency bin of the live signal by that amount (spectral subtraction,
   Boll 1979), with an over-subtraction factor and a floor that limits the
   reduction.

``Denoiser`` chains both stages on the RAW channel, then applies the same
listening filter as the device (mode / band / notch / order / gain, read from
the device's status) and estimates the heart rate from the cleaned signal with
the firmware's algorithm (25-150 Hz band, Shannon envelope, 8 s
autocorrelation, median of 5).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy import signal

from . import dsp

FS = dsp.FS


# --------------------------------------------------------------------------
# Stage 1: interference pulses
# --------------------------------------------------------------------------
class PulseRemover:
    """Removes the radio interference from the RAW signal.

    * Normal pulses (one radio event, ~2.5 ms) match the learned template and
      are subtracted, which keeps the heart sound underneath.
    * Longer bursts (several packets back to back, e.g. the 1-per-second status
      message, ~8 ms) do not match the template. They are cut from their
      rising edge to their falling edge and bridged with a straight line
      between the baselines before and after.
    """

    PRE = 4         # template starts 1 ms before the rising edge
    LEN = 28        # template length: 7 ms
    BASE = 8        # samples before / after a pulse used as local baseline
    MIN_GAP = 24    # pulses are at least 6 ms apart
    MAX_BURST = 80  # longest burst that is cut out: 20 ms
    PULSE_MAX = 16  # rise-to-fall time of a normal pulse is ~10 samples; longer = burst
    LOOK = 96       # look-ahead needed to find the end of a burst
    MIN_RATE = 5.0  # pulses/s in the calibration needed to enable this stage

    def __init__(self, template: np.ndarray, threshold: float) -> None:
        self.template = np.asarray(template, dtype=np.float64)
        self.threshold = float(threshold)
        # The pulses enter before the amplifier, so their height follows the gain
        # trimmer R2. The threshold tracks the height of the recent pulses.
        self._edge_ratio = 0.35
        self._heights: list[float] = [threshold / self._edge_ratio] * 8
        self.calib_level = float(np.median(self._heights))
        self._quiet = 0               # samples since the last pulse
        self._t2 = float(np.dot(self.template, self.template))
        self._buf = np.zeros(self.BASE + self.PRE + self.LOOK + 4)
        self.delay = len(self._buf)   # output latency in samples (34 ms)
        self._scan = len(self._buf)   # next buffer index to check for an edge
        self._last = -10 ** 9         # buffer index of the last pulse found
        self.removed = 0              # pulses subtracted since start
        self.bursts = 0               # long bursts cut out since start
        self.burst_events: list[int] = []  # input sample index of each burst's start (new ones appended)
        self._n_in = 0                # input samples consumed so far
        self.enabled = True           # False: pass through with the same delay

    @property
    def gain_ratio(self) -> float:
        """Analog gain now relative to the calibration (from the pulse heights)."""
        return float(np.median(self._heights)) / self.calib_level

    @staticmethod
    def _rise(x: np.ndarray) -> np.ndarray:
        d = np.zeros_like(x)
        d[2:] = x[2:] - x[:-2]
        return d

    @classmethod
    def learn(cls, x: np.ndarray, fs: float = FS) -> tuple[PulseRemover | None, dict]:
        """Learn the pulse template from a calibration recording (RAW channel)."""
        x = np.asarray(x, dtype=np.float64)
        d = cls._rise(x)
        sigma = np.median(np.abs(d)) / 0.6745 + 1e-12
        pk, props = signal.find_peaks(d, height=8 * sigma, distance=cls.MIN_GAP)
        info = {"pulses_per_s": len(pk) * fs / max(len(x), 1)}
        if info["pulses_per_s"] < cls.MIN_RATE:
            return None, info
        thr = max(8 * sigma, 0.35 * float(np.median(props["peak_heights"])))
        pk, props = signal.find_peaks(d, height=thr, distance=cls.MIN_GAP)
        thr = 0.35 * float(np.median(props["peak_heights"]))  # = edge ratio x typical edge height
        segs = []
        for p in pk:
            a = p - 1 - cls.PRE
            if a < cls.BASE or a + cls.LEN > len(x):
                continue
            segs.append(x[a:a + cls.LEN] - np.median(x[a - cls.BASE:a]))
        if len(segs) < 10:
            return None, info
        template = np.median(np.array(segs), axis=0)
        info.update({"pulses_per_s": len(pk) * fs / len(x), "threshold": thr,
                     "pulse_height": float(template.max())})
        return cls(template, thr), info

    def _burst_end(self, d: np.ndarray, i: int) -> int | None:
        """Index of the last falling edge of a burst that rose at i, or None."""
        thr = self.threshold
        last_fall, quiet = None, 0
        for k in range(i + 1, i + self.MAX_BURST):
            if d[k] < -thr:
                last_fall, quiet = k, 0
            elif d[k] > thr:
                quiet = 0
            else:
                quiet += 1
            if last_fall is not None and quiet >= 12:  # 3 ms without edges: burst is over
                return last_fall
        return None

    def process(self, x: np.ndarray) -> np.ndarray:
        """Streaming: returns as many samples as it gets, delayed by ``self.delay``."""
        if len(x) == 0:
            return x
        buf = np.concatenate([self._buf, np.asarray(x, dtype=np.float64)])
        d = self._rise(buf)
        L, P, B = self.LEN, self.PRE, self.BASE
        n = len(buf)
        i = max(self._scan, B + P + 3)
        if not self.enabled:
            i = max(i, n - self.LOOK)
        sigma = float(np.median(np.abs(d[B:]))) / 0.6745
        self.threshold = max(8 * sigma, self._edge_ratio * float(np.median(self._heights)))
        start = i
        while i < n - self.LOOK:  # template and burst search must fit after the edge
            if d[i] > self.threshold and d[i] >= d[i - 1] and d[i] >= d[i + 1] and i - self._last >= self.MIN_GAP:
                best = None
                for a in (i - 2 - P, i - 1 - P, i - P):
                    seg = buf[a:a + L] - np.median(buf[a - B:a])
                    amp = float(np.dot(seg, self.template)) / self._t2
                    err = float(np.sum((seg - amp * self.template) ** 2))
                    if best is None or err < best[0]:
                        best = (err, a, amp)
                err, a, amp = best
                seg_energy = float(np.sum((buf[a:a + L] - np.median(buf[a - B:a])) ** 2))
                end = self._burst_end(d, i)
                long_burst = end is not None and end - i > self.PULSE_MAX
                # accept when the template explains most of the segment (it is a pulse)
                if not long_burst and amp > 0.1 and err < 0.5 * seg_energy:
                    self._heights = (self._heights + [float(d[i])])[-64:]
                    buf[a:a + L] -= amp * self.template
                    self.removed += 1
                    self._quiet = 0
                    d = self._rise(buf)
                    self._last = i
                    i += self.MIN_GAP
                    continue
                if end is not None:
                    a, e = i - 3, end + 1
                    pre = float(np.median(buf[a - B:a]))
                    post = float(np.median(buf[e:e + B]))
                    buf[a:e] = np.linspace(pre, post, e - a)
                    self.bursts += 1
                    self.burst_events.append(self._n_in - len(self._buf) + a)
                    self._quiet = 0
                    d = self._rise(buf)
                    self._last = e
                    i = e + 1
                    continue
                self._last = i
                i += self.MIN_GAP
                continue
            i += 1
        self._quiet += max(0, i - start)
        if self._quiet > FS // 2:  # no pulse for 0.5 s: gain was lowered, relax the threshold
            self._heights = [h * 0.7 for h in self._heights]
            self._quiet = 0
        out = buf[: len(x)]
        keep = len(self._buf)
        self._n_in += len(x)
        self._buf = buf[len(x):]
        self._scan = i - len(x)
        self._last -= len(x)
        assert len(self._buf) == keep
        return out


# --------------------------------------------------------------------------
# Stage 2: steady background noise
# --------------------------------------------------------------------------
class SpectralSubtractor:
    N = 512   # 128 ms frames, 7.8 Hz resolution
    H = 256   # 50 % overlap, sqrt-Hann analysis + synthesis = perfect reconstruction

    def __init__(self, noise_power: np.ndarray, strength: float = 1.5, floor_db: float = -20.0) -> None:
        self.noise_power = np.asarray(noise_power, dtype=np.float64)
        self.scale = 1.0  # noise power multiplier (follows analog gain changes)
        self.strength = strength
        self.floor_db = floor_db
        self.win = np.sqrt(signal.windows.hann(self.N, sym=False))
        self._in = np.zeros(self.N - self.H)
        self._ola = np.zeros(self.N)
        self.delay = self.N - self.H

    @classmethod
    def learn(cls, x: np.ndarray) -> SpectralSubtractor | None:
        x = np.asarray(x, dtype=np.float64)
        if len(x) < 4 * cls.N:
            return None
        w = np.sqrt(signal.windows.hann(cls.N, sym=False))
        n = 1 + (len(x) - cls.N) // cls.H
        idx = np.arange(cls.N)[None, :] + cls.H * np.arange(n)[:, None]
        frames = x[idx]
        frames = frames - frames.mean(axis=1, keepdims=True)
        return cls(np.mean(np.abs(np.fft.rfft(frames * w, axis=1)) ** 2, axis=0))

    def process(self, x: np.ndarray) -> np.ndarray:
        buf = np.concatenate([self._in, np.asarray(x, dtype=np.float64)])
        out = []
        floor2 = 10 ** (self.floor_db / 10)
        pos = 0
        while pos + self.N <= len(buf):
            frame = buf[pos:pos + self.N] * self.win
            X = np.fft.rfft(frame)
            P = np.abs(X) ** 2
            gain = np.sqrt(np.maximum(1.0 - self.strength * self.scale * self.noise_power / np.maximum(P, 1e-20),
                                      floor2))
            y = np.fft.irfft(X * gain, n=self.N) * self.win
            self._ola += y
            out.append(self._ola[: self.H].copy())
            self._ola = np.concatenate([self._ola[self.H:], np.zeros(self.H)])
            pos += self.H
        self._in = buf[pos:]
        return np.concatenate(out) if out else np.zeros(0)

    def noise_psd(self) -> tuple[np.ndarray, np.ndarray]:
        """Calibration noise spectrum as (frequency Hz, dB re FS^2/Hz) for plotting."""
        f = np.fft.rfftfreq(self.N, 1 / FS)
        psd = self.noise_power / (FS * np.sum(self.win ** 2)) * 2
        return f, 10 * np.log10(psd + 1e-20)


# --------------------------------------------------------------------------
# 1-per-second status burst
# --------------------------------------------------------------------------
class StatusLock:
    """Locks onto the device's once-per-second status message.

    The firmware sends its status over Bluetooth every 1000 ms (millis()), as
    several packets back to back. That burst disturbs the analog signal like
    the pulses, but with a slow tail that cannot be cut out cleanly, and it
    repeats exactly once per second: the heart-rate autocorrelation would see
    it as 60 bpm. Its timing is stable (period = 1 s of the ADC clock, jitter
    about +-4 ms), so the times of all bursts can be predicted from the long
    bursts the PulseRemover finds. The heart-rate estimate ignores a short
    window around each predicted burst.
    """

    PRE, POST = 60, 320   # ignore 15 ms before .. 80 ms after the burst start
    MATCH = 40            # a burst within +-10 ms of the prediction belongs to the lock
    HISTORY_S = 20

    def __init__(self, period: float = FS * 1.0016) -> None:
        self.period = period  # ADC runs 0.16 % fast: 1 s = 4006.4 output samples
        self.phase: float | None = None
        self.events: list[int] = []
        self._new = 0

    @property
    def locked(self) -> bool:
        return self.phase is not None

    def set_fs_adc(self, fs_adc: float) -> None:
        if 15000 < fs_adc < 17000:
            self.period = fs_adc / dsp.DECIMATION

    def add(self, events: list[int]) -> None:
        if not events:
            return
        self.events = [e for e in self.events + list(events) if e > events[-1] - self.HISTORY_S * FS]
        self._new += len(events)
        if self._new >= 5:
            self._new = 0
            self._relock()

    def _relock(self) -> None:
        ev = np.array(self.events, dtype=np.float64)
        span_s = max(1.0, (ev[-1] - ev[0]) / self.period)
        ph = np.mod(ev, self.period)
        nb = int(self.period // self.MATCH)
        hist, edges = np.histogram(ph, bins=nb, range=(0, self.period))
        k = int(np.argmax(hist + np.roll(hist, 1) + np.roll(hist, -1)))
        centre = edges[k] + self.MATCH / 2
        r = (ph - centre + self.period / 2) % self.period - self.period / 2
        near = np.abs(r) < self.MATCH
        if near.sum() >= max(5, 0.4 * span_s):
            self.phase = float((centre + np.median(r[near])) % self.period)
        else:
            self.phase = None

    def mask(self, start: int, n: int) -> np.ndarray:
        """True for the samples [start, start + n) that lie in a status-burst window."""
        m = np.zeros(n, dtype=bool)
        if self.phase is None or n <= 0:
            return m
        k0 = int(np.floor((start - self.PRE - self.phase) / self.period))
        k = k0
        while True:
            c = self.phase + k * self.period
            a, b = int(round(c - self.PRE)) - start, int(round(c + self.POST)) - start
            if a >= n:
                break
            if b > 0:
                m[max(a, 0):min(b, n)] = True
            k += 1
        return m


# --------------------------------------------------------------------------
# Heart rate from the cleaned signal (firmware algorithm, run on the PC)
# --------------------------------------------------------------------------
class HrTracker:
    def __init__(self, notch_hz: float = 50.0, max_bpm: float = 120.0) -> None:
        self.set_notch(notch_hz)
        self.max_bpm = max_bpm
        self.buf = np.zeros(0)
        self.ignore = np.zeros(0, dtype=bool)
        self.since = 0
        self.history: list[float] = []
        self.result = dsp.HrEstimate(0.0, 0.0, False)
        self.bpm = 0.0
        self.rejected = 0  # windows rejected since the last valid estimate

    def set_notch(self, notch_hz: float) -> None:
        self.sos = dsp.hr_chain_sos(notch_hz)
        self.zi = np.zeros((self.sos.shape[0], 2))

    def feed(self, x: np.ndarray, ignore: np.ndarray | None = None) -> bool:
        """Push cleaned samples (``ignore``: samples to leave out of the estimate).
        Returns True when a new estimate was made (once per second)."""
        y, self.zi = signal.sosfilt(self.sos, x, zi=self.zi)
        if ignore is None:
            ignore = np.zeros(len(x), dtype=bool)
        w = dsp.HR_WINDOW_S * FS
        self.buf = np.concatenate([self.buf, y])[-w:]
        self.ignore = np.concatenate([self.ignore, ignore])[-w:]
        self.since += len(x)
        if self.since < FS or len(self.buf) < w:
            return False
        self.since = 0
        env = dsp.shannon_envelope(np.where(self.ignore, 0.0, self.buf), FS, causal_norm=False)
        seg = FS // dsp.ENV_RATE
        bad = self.ignore[: len(env) * seg].reshape(len(env), seg).any(axis=1)
        if bad.any() and not bad.all():
            env[bad] = np.median(env[~bad])  # status bursts: replace by the typical envelope level
        self.result = dsp.hr_from_envelope(env, max_bpm=self.max_bpm)
        if self.result.valid:
            self.history = (self.history + [self.result.bpm])[-5:]
            self.bpm = float(np.median(self.history))
            self.rejected = 0
        else:
            self.rejected += 1
        return True


# --------------------------------------------------------------------------
# Complete chain
# --------------------------------------------------------------------------
@dataclass
class ChainSettings:
    mode: str = "heart"
    hp: float = 25.0
    lp: float = 400.0
    notch: float = 50.0
    order: int = 4
    gain: float = 1.0

    @classmethod
    def from_status(cls, s: dict) -> ChainSettings:
        return cls(str(s.get("mode", "heart")), float(s.get("hp", 25)), float(s.get("lp", 400)),
                   float(s.get("notch", 50)), int(s.get("order", 4)), float(s.get("gain", 1)))


@dataclass
class Denoiser:
    pulse_on: bool = True
    spectral_on: bool = True
    status_on: bool = True      # leave the 1-s status bursts out of the heart rate
    strength: float = 1.5
    max_bpm: float = 120.0      # upper limit of the heart-rate search (avoids counting S1 + S2 as 2 beats)
    calib_seconds: float = 5.0
    pulses: PulseRemover | None = None
    spectral: SpectralSubtractor | None = None
    info: dict = field(default_factory=dict)
    settings: ChainSettings = field(default_factory=ChainSettings)

    def __post_init__(self) -> None:
        self._calib: list[np.ndarray] | None = None
        self._calib_n = 0
        self._build_chain()
        self.hr = HrTracker(self.settings.notch, self.max_bpm)
        self.status = StatusLock()
        self._hr_n = 0
        self.calib_raw: np.ndarray | None = None    # calibration recording (RAW, mean removed)
        self.calib_clean: np.ndarray | None = None  # the same after pulse removal
        self.last_clean = np.zeros(0)               # newest cleaned samples, before the listening chain

    # -- calibration ---------------------------------------------------
    @property
    def calibrated(self) -> bool:
        return self.pulses is not None or self.spectral is not None

    @property
    def calibrating(self) -> bool:
        return self._calib is not None

    @property
    def progress(self) -> float:
        return min(1.0, self._calib_n / (self.calib_seconds * FS)) if self._calib is not None else 0.0

    def start_calibration(self, seconds: float) -> None:
        self.calib_seconds = seconds
        self._calib, self._calib_n = [], 0

    def cancel_calibration(self) -> None:
        self._calib = None

    def _finish_calibration(self) -> None:
        x = np.concatenate(self._calib)[: int(self.calib_seconds * FS)]
        self._calib = None
        x = x - np.mean(x)
        pulses, info = PulseRemover.learn(x)
        cleaned = x
        period = self.status.period
        self.status = StatusLock(period)
        if pulses is not None:
            probe = PulseRemover(pulses.template, pulses.threshold)
            y = probe.process(np.concatenate([x, np.zeros(probe.delay)]))
            cleaned = y[probe.delay:]
            # burst times relative to the first live sample (calibration = negative times)
            self.status.add([e - len(x) for e in probe.burst_events if e < len(x)])
        spectral = SpectralSubtractor.learn(cleaned)
        if spectral is not None:
            spectral.strength = self.strength
        info.update({"seconds": len(x) / FS,
                     "noise_rms_dbfs": float(20 * np.log10(np.std(cleaned) + 1e-12)),
                     "rms_before_dbfs": float(20 * np.log10(np.std(x) + 1e-12))})
        self.pulses, self.spectral, self.info = pulses, spectral, info
        self.calib_raw, self.calib_clean = x, cleaned
        self._build_chain()
        self.hr = HrTracker(self.settings.notch, self.max_bpm)
        self._hr_n = 0

    # -- device listening chain ----------------------------------------
    def _build_chain(self) -> None:
        s = self.settings
        self._sos = dsp.user_chain_sos("raw" if s.mode == "raw" else "custom", s.notch, s.order,
                                       custom=(s.hp, s.lp))
        self._zi = np.zeros((self._sos.shape[0], 2))

    def update_settings(self, status: dict) -> None:
        if status.get("fs_adc"):
            self.status.set_fs_adc(float(status["fs_adc"]))
        s = ChainSettings.from_status(status)
        if s != self.settings:
            old, self.settings = self.settings, s
            if (s.mode, s.hp, s.lp, s.notch, s.order) != (old.mode, old.hp, old.lp, old.notch, old.order):
                self._build_chain()
            if s.notch != old.notch:
                self.hr.set_notch(s.notch)

    @property
    def delay(self) -> int:
        """Latency of the denoised output relative to the input, in samples.
        Disabled stages still pass the signal through with their delay."""
        d = 0
        if self.pulses is not None:
            d += self.pulses.delay
        if self.spectral is not None:
            d += self.spectral.delay
        return d

    # -- streaming -----------------------------------------------------
    def feed(self, raw: np.ndarray) -> np.ndarray | None:
        """Push RAW samples. Returns the denoised + filtered signal, or None
        while calibrating / before calibration."""
        raw = np.nan_to_num(np.asarray(raw, dtype=np.float64))
        if self._calib is not None:
            need = int(self.calib_seconds * FS) - self._calib_n
            self._calib.append(raw[:need])
            self._calib_n += min(need, len(raw))
            if self._calib_n < self.calib_seconds * FS:
                return None
            self._finish_calibration()
            raw = raw[need:]  # the rest of this chunk is already live signal
        if not self.calibrated:
            return None
        y = raw
        # Stages always run, so their buffers and the output timing stay continuous;
        # switched off they pass the signal through unchanged (same delay).
        if self.pulses is not None:
            self.pulses.enabled = self.pulse_on
            y = self.pulses.process(y)
            new, self.pulses.burst_events = self.pulses.burst_events, []
            self.status.add(new)
        if self.spectral is not None:
            self.spectral.strength = self.strength if self.spectral_on else 0.0
            if self.pulses is not None:
                self.spectral.scale = self.pulses.gain_ratio ** 2
            y = self.spectral.process(y)
        self.last_clean = y
        if len(y) == 0:
            return y
        self.hr.max_bpm = self.max_bpm
        ignore = self.status.mask(self._hr_n - self.delay, len(y)) if self.status_on else None
        self.hr.feed(y, ignore)
        self._hr_n += len(y)
        out, self._zi = signal.sosfilt(self._sos, y, zi=self._zi)
        return out * self.settings.gain
