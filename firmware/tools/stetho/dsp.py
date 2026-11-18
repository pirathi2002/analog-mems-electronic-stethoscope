"""Reference DSP for the MEMS electronic stethoscope.

Everything here mirrors the firmware in ``firmware/stethoscope/src/dsp`` so that
(a) the C++ implementation can be verified against it (firmware/test/) and
(b) recordings can be re-processed offline with exactly the same filters.

It also contains the offline analysis algorithms required by the project
brief: SNR, signal correlation against the BIOPAC reference, and heart-rate
estimation from long recordings.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import signal

# --------------------------------------------------------------------------
# System constants (keep in sync with firmware/stethoscope/src/config.h)
# --------------------------------------------------------------------------
FS_ADC = 16000          # ADC sample rate (Hz)
DECIMATION = 4
FS = FS_ADC // DECIMATION   # output sample rate of raw/filtered streams (Hz)

DECIM_NUMTAPS = 63
DECIM_CUTOFF_HZ = 1500.0
DECIM_KAISER_BETA = 6.0

NOTCH_Q = 30.0
DC_BLOCK_HZ = 0.5
HR_BAND = (25.0, 150.0)

MODES = {
    # name: (high-pass Hz, low-pass Hz); 0 disables that section
    "raw": (0.0, 0.0),
    "heart": (25.0, 400.0),
    "bell": (20.0, 200.0),
    "diaphragm": (100.0, 500.0),
    "wide": (20.0, 1000.0),
}
MODE_NAMES = ["raw", "heart", "bell", "diaphragm", "wide", "custom"]


# --------------------------------------------------------------------------
# Filter design (RBJ cookbook, identical formulas to biquad.cpp)
# --------------------------------------------------------------------------
def _rbj(kind: str, fs: float, f0: float, q: float) -> np.ndarray:
    w0 = 2 * np.pi * f0 / fs
    cs, sn = np.cos(w0), np.sin(w0)
    alpha = sn / (2 * q)
    if kind == "lp":
        b = [(1 - cs) / 2, 1 - cs, (1 - cs) / 2]
    elif kind == "hp":
        b = [(1 + cs) / 2, -(1 + cs), (1 + cs) / 2]
    elif kind == "notch":
        b = [1.0, -2 * cs, 1.0]
    else:
        raise ValueError(kind)
    a = [1 + alpha, -2 * cs, 1 - alpha]
    return np.array([*(np.array(b) / a[0]), 1.0, a[1] / a[0], a[2] / a[0]])


def butterworth_q(order: int, k: int) -> float:
    theta = (2 * k + 1) * np.pi / (2 * order)
    return 1.0 / (2 * np.cos(theta))


def butter_sos(kind: str, fs: float, fc: float, order: int) -> np.ndarray:
    return np.array([_rbj(kind, fs, fc, butterworth_q(order, k)) for k in range(order // 2)])


def notch_sos(fs: float, f0: float, q: float = NOTCH_Q) -> np.ndarray:
    return _rbj("notch", fs, f0, q)[None, :]


def mode_band(mode: str, custom: tuple[float, float] = (25.0, 400.0)) -> tuple[float, float]:
    return custom if mode == "custom" else MODES[mode]


def user_chain_sos(mode: str = "heart", notch_hz: float = 50.0, order: int = 4,
                   custom: tuple[float, float] = (25.0, 400.0), fs: float = FS) -> np.ndarray:
    """SOS matrix of the user listening chain (notch -> HP -> LP)."""
    if mode == "raw":
        return np.array([[1.0, 0, 0, 1.0, 0, 0]])
    hp, lp = mode_band(mode, custom)
    if not (0 < hp < 0.45 * fs):
        hp = 0.0
    if not (0 < lp < 0.45 * fs):
        lp = 0.0
    if hp and lp and hp >= lp:
        lp = 0.0
    sos = []
    if 0 < notch_hz < fs / 2:
        sos.append(notch_sos(fs, notch_hz))
    if hp:
        sos.append(butter_sos("hp", fs, hp, order))
    if lp:
        sos.append(butter_sos("lp", fs, lp, order))
    if not sos:
        return np.array([[1.0, 0, 0, 1.0, 0, 0]])
    return np.vstack(sos)


def hr_chain_sos(notch_hz: float = 50.0, fs: float = FS) -> np.ndarray:
    sos = []
    if 0 < notch_hz < fs / 2:
        sos.append(notch_sos(fs, notch_hz))
    sos.append(butter_sos("hp", fs, HR_BAND[0], 4))
    sos.append(butter_sos("lp", fs, HR_BAND[1], 4))
    return np.vstack(sos)


def decimation_taps() -> np.ndarray:
    """Anti-aliasing FIR for the 16 kHz -> 4 kHz decimation."""
    return signal.firwin(DECIM_NUMTAPS, DECIM_CUTOFF_HZ, window=("kaiser", DECIM_KAISER_BETA),
                         fs=FS_ADC)


def dc_block(x: np.ndarray, fs: float = FS) -> np.ndarray:
    r = 1.0 - 2 * np.pi * DC_BLOCK_HZ / fs
    return signal.lfilter([1.0, -1.0], [1.0, -r], x)


# --------------------------------------------------------------------------
# Full pipeline (bit-for-bit structure of PcgPipeline::push)
# --------------------------------------------------------------------------
@dataclass
class PipelineOutput:
    raw: np.ndarray
    filtered: np.ndarray
    hr_band: np.ndarray


def adc_to_float(codes: np.ndarray) -> np.ndarray:
    return (np.asarray(codes, dtype=np.float64) - 2048.0) / 2048.0


def run_pipeline(adc_norm: np.ndarray, mode: str = "heart", notch_hz: float = 50.0,
                 gain: float = 1.0, order: int = 4,
                 custom: tuple[float, float] = (25.0, 400.0)) -> PipelineOutput:
    """Process normalised ADC samples at FS_ADC exactly like the firmware."""
    h = decimation_taps()
    d = signal.lfilter(h, 1.0, adc_norm)[DECIMATION - 1::DECIMATION]
    raw = dc_block(d)
    filt = signal.sosfilt(user_chain_sos(mode, notch_hz, order, custom), raw) * gain
    hr_band = signal.sosfilt(hr_chain_sos(notch_hz), raw)
    return PipelineOutput(raw=raw, filtered=filt, hr_band=hr_band)


def apply_mode(x: np.ndarray, mode: str = "heart", notch_hz: float = 50.0, order: int = 4,
               fs: float = FS, zero_phase: bool = False) -> np.ndarray:
    """Apply a listening-mode filter to an already-decimated signal."""
    sos = user_chain_sos(mode, notch_hz, order, fs=fs)
    return signal.sosfiltfilt(sos, x) if zero_phase else signal.sosfilt(sos, x)


# --------------------------------------------------------------------------
# Envelope + heart-rate estimation
# --------------------------------------------------------------------------
ENV_RATE = 50
HR_WINDOW_S = 8
HR_MIN_BPM, HR_MAX_BPM = 40.0, 200.0
HR_MIN_CONF = 0.15
HR_MIN_PULSATILITY = 0.8  # envelope std/mean; heart sounds ~1.3-3, steady noise < 0.4


def shannon_envelope(y: np.ndarray, fs: float = FS, causal_norm: bool = True) -> np.ndarray:
    """Average Shannon energy per 20 ms segment (envelope at ENV_RATE Hz).

    With ``causal_norm`` the normalisation is the running 3 s peak (as in the
    firmware); otherwise the global peak is used (classic offline method).
    """
    seg = int(round(fs / ENV_RATE))
    n = len(y) // seg
    a = np.abs(y[: n * seg]).reshape(n, seg)
    seg_max = a.max(axis=1)
    if causal_norm:
        k = 3 * ENV_RATE
        norm = np.zeros(n)
        run = np.zeros(k)
        for i in range(n):           # norm used for segment i = max of previous k segments
            norm[i] = run.max()
            run[i % k] = seg_max[i]
    else:
        norm = np.full(n, a.max())
    with np.errstate(divide="ignore", invalid="ignore"):
        xn = np.clip(a / (1.2 * norm[:, None]), 0, 1)
        e = xn ** 2
        se = np.where(e > 1e-12, -e * np.log(np.maximum(e, 1e-30)), 0.0)
    se[norm <= 1e-9] = 0.0
    return se.mean(axis=1)


@dataclass
class HrEstimate:
    bpm: float
    confidence: float
    valid: bool
    pulsatility: float = 0.0


def hr_from_envelope(env: np.ndarray, env_rate: float = ENV_RATE,
                     min_bpm: float = HR_MIN_BPM, max_bpm: float = HR_MAX_BPM) -> HrEstimate:
    """Autocorrelation HR estimate from one envelope window (firmware algorithm)."""
    m = float(np.mean(env))
    e = np.asarray(env, dtype=np.float64) - m
    n = len(e)
    r0 = float(np.dot(e, e))
    puls = float(np.sqrt(r0 / n) / m) if m > 1e-12 else 0.0
    if r0 <= 1e-12 or puls < HR_MIN_PULSATILITY:
        return HrEstimate(0.0, 0.0, False, puls)
    lag_min = int(60 * env_rate / max_bpm)
    lag_max = int(60 * env_rate / min_bpm + 0.5)
    lags = np.arange(lag_min - 1, min(lag_max + 2, n - 1))
    r = np.array([np.dot(e[: n - l], e[l:]) for l in lags]) / r0
    peaks = [j for j in range(1, len(r) - 1) if r[j] >= r[j - 1] and r[j] >= r[j + 1]]
    if not peaks:
        return HrEstimate(0.0, 0.0, False)
    j = max(peaks, key=lambda k: r[k])
    denom = r[j - 1] - 2 * r[j] + r[j + 1]
    delta = 0.5 * (r[j - 1] - r[j + 1]) / denom if abs(denom) > 1e-12 else 0.0
    delta = float(np.clip(delta, -0.5, 0.5))
    bpm = 60 * env_rate / (lags[j] + delta)
    conf = float(r[j])
    return HrEstimate(float(bpm), conf, conf >= HR_MIN_CONF and min_bpm <= bpm <= max_bpm, puls)


def hr_track(x: np.ndarray, fs: float = FS, window_s: float = HR_WINDOW_S, step_s: float = 1.0,
             notch_hz: float = 50.0, already_filtered: bool = False):
    """Sliding-window HR track. Returns (times_s, bpm, confidence, valid)."""
    y = x if already_filtered else signal.sosfilt(hr_chain_sos(notch_hz, fs), x)
    env = shannon_envelope(y, fs)
    w = int(window_s * ENV_RATE)
    step = int(step_s * ENV_RATE)
    t, b, c, v = [], [], [], []
    for end in range(w, len(env) + 1, step):
        est = hr_from_envelope(env[end - w:end])
        t.append(end / ENV_RATE)
        b.append(est.bpm)
        c.append(est.confidence)
        v.append(est.valid)
    return np.array(t), np.array(b), np.array(c), np.array(v, dtype=bool)


def detect_beats(x: np.ndarray, fs: float = FS, notch_hz: float = 50.0):
    """S1/S2 peak detection on the Shannon envelope (offline, zero-phase).

    Returns (peak_times_s, envelope, envelope_rate). Peaks include both S1 and
    S2; use :func:`hr_from_beats` to turn them into a heart rate.
    """
    y = signal.sosfiltfilt(hr_chain_sos(notch_hz, fs), x)
    env = shannon_envelope(y, fs, causal_norm=False)
    env = signal.filtfilt(np.ones(3) / 3, [1.0], env)
    thr = np.mean(env) + 0.5 * np.std(env)
    peaks, _ = signal.find_peaks(env, height=thr, distance=int(0.2 * ENV_RATE))
    return peaks / ENV_RATE, env, ENV_RATE


def hr_from_beats(peak_times: np.ndarray, bpm_hint: float | None = None) -> float:
    """HR from S1 peak times: HR = 60 / mean(T_S1(n+1) - T_S1(n)).

    Because S1 and S2 are both detected, the cardiac cycle is the sum of two
    consecutive intervals; ``bpm_hint`` (e.g. from autocorrelation) resolves
    the S1/S2 ambiguity.
    """
    if len(peak_times) < 3:
        return float("nan")
    d = np.diff(peak_times)
    if bpm_hint:
        period = 60.0 / bpm_hint
        single = np.median(d)
        if abs(single - period) < abs(2 * single - period):
            return 60.0 / np.median(d)
    cycle = d[:-1] + d[1:]
    return float(60.0 / np.median(cycle))


# --------------------------------------------------------------------------
# Performance metrics (project brief: SNR, correlation vs BIOPAC)
# --------------------------------------------------------------------------
def snr_tone(x: np.ndarray, fs: float, f0: float, bw_hz: float = 5.0,
             band: tuple[float, float] = (20.0, 1000.0)) -> float:
    """SNR (dB) of a known-frequency test tone: tone power / noise power in band."""
    f, p = signal.welch(x, fs, nperseg=min(len(x), int(fs * 2)))
    inband = (f >= band[0]) & (f <= band[1])
    tone = inband & (np.abs(f - f0) <= bw_hz)
    noise = inband & ~tone
    df = f[1] - f[0]
    ps = np.sum(p[tone]) * df
    pn = np.sum(p[noise]) * df
    return float(10 * np.log10(ps / pn)) if pn > 0 else float("inf")


def snr_heart(x: np.ndarray, fs: float = FS, notch_hz: float = 50.0) -> float:
    """SNR (dB) of a heart-sound recording.

    Heart-sound segments (S1/S2, where the envelope is high) are treated as
    signal, the quiet systolic/diastolic intervals as noise:
        SNR = 10 log10(P_sound / P_quiet)
    """
    y = signal.sosfiltfilt(hr_chain_sos(notch_hz, fs), x)
    seg = int(round(fs / ENV_RATE))
    n = len(y) // seg
    p = (y[: n * seg].reshape(n, seg) ** 2).mean(axis=1)
    lo, hi = np.percentile(p, [30, 90])
    quiet = p[p <= lo]
    loud = p[p >= hi]
    if len(quiet) == 0 or np.mean(quiet) <= 0:
        return float("inf")
    return float(10 * np.log10(np.mean(loud) / np.mean(quiet)))


def snr_signal_vs_noise(sig: np.ndarray, noise: np.ndarray) -> float:
    """SNR (dB) from two separate recordings: with signal vs. noise only."""
    ps = np.mean(np.asarray(sig, float) ** 2)
    pn = np.mean(np.asarray(noise, float) ** 2)
    return float(10 * np.log10((ps - pn) / pn)) if ps > pn > 0 else float("nan")


@dataclass
class Alignment:
    lag_s: float
    pearson_r: float
    envelope_r: float
    n_samples: int


def align_and_correlate(a: np.ndarray, fs_a: float, b: np.ndarray, fs_b: float,
                        fs_common: float = 2000.0, band: tuple[float, float] = (25.0, 400.0),
                        max_lag_s: float = 5.0) -> Alignment:
    """Correlation between two simultaneous recordings (e.g. ours vs BIOPAC).

    Both signals are resampled to ``fs_common``, band-passed identically, aligned
    by the cross-correlation maximum (within +/- max_lag_s), and then the
    Pearson correlation of the overlapping waveforms and of their Shannon
    envelopes is computed. A positive lag means ``b`` is delayed relative to ``a``.
    """
    def prep(x, fs):
        from fractions import Fraction

        x = np.asarray(x, dtype=np.float64)
        # Rational resampling also handles non-integer *measured* rates
        # (the ESP32-S3 ADC runs ~0.16 % fast: 4006.5 Hz instead of 4000 Hz).
        r = Fraction(float(fs_common) / float(fs)).limit_denominator(2000)
        x = signal.resample_poly(x, r.numerator, r.denominator)
        sos = signal.butter(4, band, btype="bandpass", fs=fs_common, output="sos")
        x = signal.sosfiltfilt(sos, x - np.mean(x))
        return x / (np.std(x) + 1e-12)

    xa, xb = prep(a, fs_a), prep(b, fs_b)
    # Align on envelopes first (robust to the different phase responses of the
    # two acquisition chains), then refine on the waveform.
    ea = shannon_envelope(xa, fs_common, causal_norm=False)
    eb = shannon_envelope(xb, fs_common, causal_norm=False)
    ea, eb = ea - ea.mean(), eb - eb.mean()
    max_lag_env = int(max_lag_s * ENV_RATE)
    cc = signal.correlate(eb, ea, mode="full")
    lags = signal.correlation_lags(len(eb), len(ea), mode="full")
    m = np.abs(lags) <= max_lag_env
    coarse = int(lags[m][np.argmax(cc[m])]) * int(fs_common / ENV_RATE)

    seg = int(fs_common / ENV_RATE)
    cc2 = signal.correlate(xb, xa, mode="full")
    lags2 = signal.correlation_lags(len(xb), len(xa), mode="full")
    m2 = np.abs(lags2 - coarse) <= 2 * seg
    lag = int(lags2[m2][np.argmax(np.abs(cc2[m2]))])

    if lag >= 0:
        sa, sb = xa, xb[lag:]
    else:
        sa, sb = xa[-lag:], xb
    n = min(len(sa), len(sb))
    sa, sb = sa[:n], sb[:n]
    r = float(np.corrcoef(sa, sb)[0, 1])
    ea2 = shannon_envelope(sa, fs_common, causal_norm=False)
    eb2 = shannon_envelope(sb, fs_common, causal_norm=False)
    re = float(np.corrcoef(ea2, eb2)[0, 1])
    return Alignment(lag_s=lag / fs_common, pearson_r=r, envelope_r=re, n_samples=n)
