"""Synthetic PCG (heart-sound) generator.

Used by the host tests, by the viewer's ``--demo`` mode (so the PC software
can be developed and presented without hardware), and to sanity-check the
analysis scripts. It is a simple model, not a physiological simulator:

* S1: ~120 ms burst of damped 40-70 Hz oscillations (mitral/tricuspid closure)
* S2: ~90 ms burst of damped 60-110 Hz oscillations (aortic/pulmonary closure)
* systolic interval S1->S2 scales with the RR interval
* beat-to-beat RR jitter, white + low-frequency noise, optional mains hum
"""
from __future__ import annotations

import numpy as np


def _burst(fs: float, dur: float, f_lo: float, f_hi: float, rng: np.random.Generator) -> np.ndarray:
    t = np.arange(int(dur * fs)) / fs
    env = np.sin(np.pi * t / dur) ** 2
    out = np.zeros_like(t)
    for _ in range(3):
        f = rng.uniform(f_lo, f_hi)
        out += rng.uniform(0.5, 1.0) * np.sin(2 * np.pi * f * t + rng.uniform(0, 2 * np.pi))
    return env * out / 3


def synth_pcg(duration_s: float = 30.0, fs: float = 4000.0, bpm: float = 72.0,
              hrv: float = 0.03, snr_db: float = 20.0, hum_hz: float = 0.0,
              hum_rel: float = 0.2, seed: int = 0):
    """Return (signal, s1_times_s). Signal RMS of the heart sounds ~0.1."""
    rng = np.random.default_rng(seed)
    n = int(duration_s * fs)
    x = np.zeros(n)
    s1_times = []
    t = rng.uniform(0.1, 0.5)
    rr0 = 60.0 / bpm
    while t < duration_s - 1.0:
        rr = rr0 * (1 + hrv * rng.standard_normal())
        systole = 0.10 + 0.25 * rr  # ~0.3 s at 72 bpm
        for start, dur, lo, hi, amp in ((t, 0.12, 40, 70, 1.0),
                                        (t + systole, 0.09, 60, 110, 0.7)):
            b = amp * _burst(fs, dur, lo, hi, rng)
            i = int(start * fs)
            j = min(n, i + len(b))
            x[i:j] += b[: j - i]
        s1_times.append(t)
        t += rr
    sig_rms = np.sqrt(np.mean(x ** 2)) + 1e-12
    x *= 0.1 / sig_rms
    noise = rng.standard_normal(n)
    # add some low-frequency (body / handling) noise
    lf = np.cumsum(rng.standard_normal(n))
    lf -= np.convolve(lf, np.ones(int(fs)) / fs, mode="same")
    noise = noise / np.std(noise) + 0.5 * lf / (np.std(lf) + 1e-12)
    noise *= 0.1 / (10 ** (snr_db / 20)) / np.std(noise)
    x = x + noise
    if hum_hz:
        x += hum_rel * 0.1 * np.sqrt(2) * np.sin(2 * np.pi * hum_hz * np.arange(n) / fs)
    return x, np.array(s1_times)


def to_adc_codes(x: np.ndarray, fs_in: float, fs_adc: float = 16000.0, dc_v: float = 1.65,
                 full_scale_v: float = 3.3, seed: int = 1) -> np.ndarray:
    """Upsample a signal (in 'normalised' units, |x| <= ~1) to ADC codes at fs_adc.

    The signal is placed on the AFE bias voltage, passed through a model of the
    analog anti-alias filter (~1.6 kHz) and quantised to 12 bits.
    """
    from scipy import signal

    up = int(round(fs_adc / fs_in))
    y = signal.resample_poly(x, up, 1)
    sos = signal.butter(2, 1600, fs=fs_adc, output="sos")
    y = signal.sosfilt(sos, y)
    v = dc_v + y * (full_scale_v / 2)
    rng = np.random.default_rng(seed)
    codes = np.round(v / full_scale_v * 4095 + rng.uniform(-0.5, 0.5, len(v)))
    return np.clip(codes, 0, 4095).astype(np.uint16)
