#!/usr/bin/env python3
"""Host tests for the firmware DSP.

1. Compiles firmware/stethoscope/src/dsp/*.cpp with g++ (same sources as the
   ESP32 build) into a small harness (host_pipeline.cpp).
2. Feeds it synthetic heart sounds converted to 16 kHz ADC codes.
3. Checks:
   - firmware output == SciPy reference (firmware/tools/stetho/dsp.py)
   - RBJ biquad Butterworth design == scipy.signal.butter
   - 50 Hz mains hum is removed by the notch
   - heart-rate estimate is within +/-3 bpm of the truth at several rates

Run:  python3 firmware/test/test_dsp.py
"""
from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from scipy import signal

HERE = Path(__file__).resolve().parent
FW = HERE.parent
sys.path.insert(0, str(FW / "tools"))

from stetho import dsp  # noqa: E402
from stetho.synth import synth_pcg, to_adc_codes  # noqa: E402

FAILS: list[str] = []


def check(cond: bool, msg: str) -> None:
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond:
        FAILS.append(msg)


def build(tmp: Path) -> Path:
    exe = tmp / "host_pipeline"
    src = FW / "stethoscope" / "src" / "dsp"
    cmd = ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-o", str(exe),
           str(HERE / "host_pipeline.cpp"), str(src / "biquad.cpp"), str(src / "hr_estimator.cpp"),
           str(src / "pcg_pipeline.cpp")]
    subprocess.run(cmd, check=True)
    return exe


def run_fw(exe: Path, tmp: Path, codes: np.ndarray, mode: str, notch: float = 50.0, gain: float = 1.0):
    cin, cout = tmp / "in.u16", tmp / "out.f32"
    codes.astype("<u2").tofile(cin)
    p = subprocess.run([str(exe), str(cin), mode, str(notch), str(gain), str(cout)],
                       check=True, capture_output=True, text=True)
    y = np.fromfile(cout, dtype="<f4").reshape(-1, 2)
    hr = np.array([[float(v) for v in line.split(",")] for line in p.stdout.split()]) if p.stdout else np.zeros((0, 6))
    return y[:, 0].astype(float), y[:, 1].astype(float), hr


def test_design_matches_scipy() -> None:
    print("[design] RBJ Butterworth biquads vs scipy.signal.butter")
    for kind, fc, order in [("hp", 25, 4), ("lp", 400, 4), ("hp", 100, 6), ("lp", 1000, 2)]:
        ours = dsp.butter_sos(kind, dsp.FS, fc, order)
        ref = signal.butter(order, fc, btype="highpass" if kind == "hp" else "lowpass", fs=dsp.FS, output="sos")
        w = np.linspace(1, dsp.FS / 2 - 1, 2000)
        _, h1 = signal.sosfreqz(ours, worN=w, fs=dsp.FS)
        _, h2 = signal.sosfreqz(ref, worN=w, fs=dsp.FS)
        err = np.max(np.abs(np.abs(h1) - np.abs(h2)))
        check(err < 1e-9, f"{kind} {fc} Hz order {order}: max |H| difference {err:.2e}")
    for mode, (lo, hi) in dsp.MODES.items():
        if mode == "raw":
            continue
        _, h = signal.sosfreqz(dsp.user_chain_sos(mode, notch_hz=0), worN=[lo, hi, np.sqrt(lo * hi)], fs=dsp.FS)
        db = 20 * np.log10(np.abs(h))
        check(abs(db[0] + 3.01) < 0.05 and abs(db[1] + 3.01) < 0.05 and db[2] > -0.5,
              f"mode {mode}: -3 dB at {lo:.0f}/{hi:.0f} Hz ({db[0]:.2f}/{db[1]:.2f} dB), centre {db[2]:.2f} dB")


def test_firmware_matches_reference(exe: Path, tmp: Path) -> None:
    print("[pipeline] firmware C++ output vs SciPy reference")
    x, _ = synth_pcg(duration_s=12, fs=4000, bpm=72, snr_db=15, hum_hz=50, seed=1)
    codes = to_adc_codes(0.5 * x, 4000)
    norm = dsp.adc_to_float(codes)
    for mode in ["raw", "heart", "bell", "diaphragm", "wide"]:
        raw, filt, _ = run_fw(exe, tmp, codes, mode, 50.0, 2.0)
        ref = dsp.run_pipeline(norm, mode=mode, notch_hz=50.0, gain=2.0)
        n = min(len(raw), len(ref.raw))
        e_raw = np.max(np.abs(raw[:n] - ref.raw[:n])) / np.max(np.abs(ref.raw[:n]))
        e_f = np.max(np.abs(filt[:n] - ref.filtered[:n])) / np.max(np.abs(ref.filtered[:n]))
        check(n == len(codes) // 4 and e_raw < 1e-3 and e_f < 1e-3,
              f"mode {mode:9s}: {n} samples, rel. error raw {e_raw:.1e}, filtered {e_f:.1e}")


def test_notch(exe: Path, tmp: Path) -> None:
    print("[notch] 50 Hz hum rejection")
    fs = 4000
    t = np.arange(10 * fs) / fs
    hum = 0.3 * np.sin(2 * np.pi * 50 * t)
    codes = to_adc_codes(hum, fs)
    raw, filt, _ = run_fw(exe, tmp, codes, "heart", 50.0, 1.0)
    s = slice(len(raw) // 2, None)  # after settling
    att = 20 * np.log10(np.std(filt[s]) / np.std(raw[s]))
    check(att < -30, f"50 Hz attenuation {att:.1f} dB (need < -30 dB)")
    raw, filt, _ = run_fw(exe, tmp, codes, "heart", 0.0, 1.0)
    att_off = 20 * np.log10(np.std(filt[s]) / np.std(raw[s]))
    check(att_off > -3, f"notch off: 50 Hz passes ({att_off:.1f} dB)")


def test_hr(exe: Path, tmp: Path) -> None:
    print("[hr] heart-rate estimation on synthetic PCG (+/-3 bpm)")
    for bpm, snr in [(50, 20), (72, 20), (72, 6), (95, 15), (120, 15), (150, 20)]:
        x, s1 = synth_pcg(duration_s=40, fs=4000, bpm=bpm, snr_db=snr, hum_hz=50, seed=bpm)
        codes = to_adc_codes(0.4 * x, 4000)
        _, _, hr = run_fw(exe, tmp, codes, "heart")
        true = 60 / np.mean(np.diff(s1))
        late = hr[hr[:, 0] > 20 * 4000]          # after 20 s
        valid = late[late[:, 4] == 1]
        ok_frac = len(valid) / max(1, len(late))
        est = float(np.median(valid[:, 1])) if len(valid) else float("nan")
        check(abs(est - true) <= 3 and ok_frac > 0.8,
              f"true {true:6.1f} bpm, SNR {snr:2d} dB -> firmware {est:6.1f} bpm, valid {ok_frac:.0%}")

    print("[hr] no false heart rate on steady noise (beating mains harmonics, white noise)")
    fs = 4000
    t = np.arange(30 * fs) / fs
    hum = sum(0.05 * np.sin(2 * np.pi * f * t) for f in (99.6, 100.6, 149.4, 150.4))
    for name, x in (("beating hum", hum), ("white noise", 0.05 * np.random.default_rng(1).standard_normal(len(t)))):
        _, _, hr = run_fw(exe, tmp, to_adc_codes(x, fs), "heart")
        n_valid = int(hr[:, 4].sum()) if len(hr) else 0
        puls = float(np.max(hr[:, 5])) if len(hr) else 0.0
        check(n_valid == 0, f"{name}: {n_valid} valid HR outputs (max pulsatility {puls:.2f} < 0.8)")

    print("[hr] python reference HR track agrees with firmware")
    x, s1 = synth_pcg(duration_s=30, fs=4000, bpm=80, snr_db=15, seed=7)
    codes = to_adc_codes(0.4 * x, 4000)
    raw, _, hr = run_fw(exe, tmp, codes, "heart")
    t, b, c, v = dsp.hr_track(dsp.run_pipeline(dsp.adc_to_float(codes)).raw)
    n = min(len(b), len(hr))
    d = np.max(np.abs(b[:n] - hr[:n, 2]))
    check(d < 0.5, f"max |python - firmware| instantaneous bpm = {d:.3f}")

    beats, _, _ = dsp.detect_beats(raw)
    hb = dsp.hr_from_beats(beats, bpm_hint=float(np.median(b[v])))
    true = 60 / np.mean(np.diff(s1))
    check(abs(hb - true) < 3, f"offline beat-interval HR {hb:.1f} bpm (true {true:.1f})")


def test_metrics() -> None:
    print("[metrics] SNR and correlation helpers")
    fs = 4000
    t = np.arange(20 * fs) / fs
    rng = np.random.default_rng(0)
    tone = np.sin(2 * np.pi * 100 * t) + 0.1 * rng.standard_normal(len(t))
    snr = dsp.snr_tone(tone, fs, 100)
    check(20 < snr < 40, f"tone SNR {snr:.1f} dB")
    x, _ = synth_pcg(duration_s=30, fs=fs, bpm=70, snr_db=20, seed=2)
    delayed = np.concatenate([np.zeros(int(0.25 * fs)), x])[: len(x)] + 0.02 * rng.standard_normal(len(x))
    b = signal.resample_poly(delayed, 2, 1)  # "BIOPAC" at 8 kHz
    al = dsp.align_and_correlate(x, fs, b, 2 * fs)
    check(abs(al.lag_s - 0.25) < 0.002 and al.pearson_r > 0.9,
          f"alignment lag {al.lag_s * 1e3:.1f} ms (true 250), r = {al.pearson_r:.3f}, envelope r = {al.envelope_r:.3f}")


def _interference(n: int, fs: int, amp: float, rng: np.random.Generator) -> np.ndarray:
    """Bluetooth-like interference as measured on the main board: a 2.5 ms double-hump
    pulse every 10-12.5 ms, plus an 8 ms multi-packet burst once per second."""
    z = np.zeros(n)
    pulse = np.array([0.5, 1, 1, 1, 0.4, 1, 1, 1, 0.5, 0.1])
    t = 20
    while t < n - 20:
        z[t:t + len(pulse)] += amp * pulse
        t += int(rng.choice([40, 45, 50]))
    burst = np.concatenate([np.ones(10), [0.3], np.ones(9), [0.3], np.ones(10)])
    period = fs * 1.0016
    k = 0
    while True:
        s = int(0.3 * fs + k * period + rng.integers(-12, 13))
        if s + len(burst) >= n:
            break
        z[s:s + len(burst)] = amp * 1.1 * burst
        k += 1
    return z


def test_denoise() -> None:
    from stetho.denoise import Denoiser

    print("\n[denoise] PC noise reduction (calibration + pulse removal + background subtraction)")
    fs = dsp.FS
    rng = np.random.default_rng(5)
    bpm = 72.0
    heart, _ = synth_pcg(duration_s=60, fs=fs, bpm=bpm, snr_db=40, seed=7)
    heart = 0.01 * heart / np.max(np.abs(heart))
    cal = _interference(5 * fs, fs, 0.01, rng) + 2e-4 * rng.standard_normal(5 * fs)
    live = heart + _interference(len(heart), fs, 0.01, rng) + 2e-4 * rng.standard_normal(len(heart))
    status = {"mode": "heart", "hp": 25, "lp": 400, "notch": 50, "order": 4, "gain": 1.0, "fs_adc": fs * 4 * 1.0016}

    def run(dn: Denoiser, x: np.ndarray) -> np.ndarray:
        out, i = [], 0
        while i < len(x):
            k = int(rng.integers(60, 400))
            y = dn.feed(x[i:i + k])
            if y is not None and len(y):
                out.append(y)
            i += k
        return np.concatenate(out)

    chain = dsp.user_chain_sos("custom", 50.0, 4, custom=(25.0, 400.0))
    dn = Denoiser(pulse_on=False, spectral_on=False)
    dn.update_settings(status)
    dn.start_calibration(5)
    y = run(dn, np.concatenate([cal, live]))
    ref = signal.sosfilt(chain, live)
    err = np.max(np.abs(y[dn.delay:] - ref[: len(y) - dn.delay]))
    check(err < 1e-9, f"both stages off = device listening chain (max error {err:.1e}, latency {dn.delay} samples)")

    dn = Denoiser()
    dn.update_settings(status)
    dn.start_calibration(5)
    y = run(dn, np.concatenate([cal, live]))
    clean = signal.sosfilt(chain, heart)
    noisy = signal.sosfilt(chain, live)
    a, b = y[dn.delay + 10 * fs:], clean[10 * fs: len(y) - dn.delay]
    r_after = np.corrcoef(a, b)[0, 1]
    r_before = np.corrcoef(noisy[10 * fs:], clean[10 * fs:])[0, 1]
    check(dn.pulses is not None and 70 < dn.info["pulses_per_s"] < 100,
          f"calibration learned the pulses ({dn.info.get('pulses_per_s', 0):.0f}/s)")
    check(r_after > 0.85 and r_after > r_before + 0.3,
          f"correlation with the clean heart sound {r_before:.3f} -> {r_after:.3f}")
    check(dn.status.locked, "locked onto the 1-per-second status bursts")
    check(dn.hr.result.valid and abs(dn.hr.bpm - bpm) < 3, f"PC heart rate {dn.hr.bpm:.1f} bpm (true {bpm:.0f})")

    # the analog gain (trimmer R2) is raised 5x after calibration: pulses and heart scale together
    dn = Denoiser()
    dn.update_settings(status)
    dn.start_calibration(5)
    run(dn, np.concatenate([cal, 5 * live]))
    check(4 < dn.pulses.gain_ratio < 6 and abs(dn.hr.bpm - bpm) < 3,
          f"gain raised 5x: tracked x{dn.pulses.gain_ratio:.1f}, heart rate {dn.hr.bpm:.1f} bpm")


def main() -> int:
    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d)
        exe = build(tmp)
        test_design_matches_scipy()
        test_firmware_matches_reference(exe, tmp)
        test_notch(exe, tmp)
        test_hr(exe, tmp)
        test_metrics()
        test_denoise()
    print()
    if FAILS:
        print(f"{len(FAILS)} check(s) FAILED")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
