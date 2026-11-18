#!/usr/bin/env python3
"""Offline analysis for the project's performance evaluation.

Sub-commands
------------
report  <rec.wav> [--ref-bpm 72]
    Raw vs. filtered comparison (waveforms + spectra), heart-sound SNR,
    heart rate (autocorrelation track and S1-S1 beat intervals), HR accuracy
    against a reference value (e.g. pulse oximeter / BIOPAC / manual count).

tone    <rec.wav> [<rec2.wav> ...] --freq 100 [200 ...]
    Characterisation with known-frequency acoustic signals (speaker test):
    level and SNR of each test tone -> table (+ frequency-response plot when
    several frequencies are given).

noise   <signal.wav> <noise_only.wav>
    SNR from a controlled-noise test: recording with signal vs. recording of
    the noise alone.

compare <ours.wav> <biopac.(txt|csv|mat|wav|acq)> [--ref-fs 2000] [--ref-col 1]
    Same controlled acoustic signal recorded simultaneously with our device and
    the BIOPAC stethoscope: time alignment, Pearson correlation of the
    waveforms and of the envelopes, SNR and HR of both.

repeat  <rec1.wav> <rec2.wav> ...
    Repeatability: mean +/- SD and coefficient of variation of level, SNR and
    HR over repeated recordings.

All plots are written next to the input file (or to --out).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy import signal

from stetho import dsp
from stetho.recio import load_recording, load_reference


def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def _outdir(args, path: Path) -> Path:
    d = Path(args.out) if args.out else path.with_suffix("")
    d.mkdir(parents=True, exist_ok=True)
    return d


def _db(x):
    return 10 * np.log10(np.maximum(x, 1e-20))


def analyse_hr(x: np.ndarray, fs: float, notch: float) -> dict:
    t, bpm, conf, valid = dsp.hr_track(x, fs, notch_hz=notch)
    res = {"hr_track_t": t.tolist(), "hr_track_bpm": bpm.tolist(), "hr_track_valid": valid.tolist()}
    if valid.any():
        res["hr_autocorr_bpm"] = float(np.median(bpm[valid]))
        res["hr_valid_fraction"] = float(valid.mean())
    # whole-recording autocorrelation (one window = entire recording)
    y = signal.sosfiltfilt(dsp.hr_chain_sos(notch, fs), x)
    whole = dsp.hr_from_envelope(dsp.shannon_envelope(y, fs, causal_norm=False))
    res["hr_whole_bpm"] = whole.bpm
    res["hr_whole_conf"] = whole.confidence
    beats, env, er = dsp.detect_beats(x, fs, notch)
    res["hr_beats_bpm"] = dsp.hr_from_beats(beats, bpm_hint=whole.bpm if whole.valid else None)
    res["n_peaks"] = int(len(beats))
    res["_beats"] = beats
    res["_env"] = env
    return res


def cmd_report(args) -> int:
    path = Path(args.file)
    fs, raw, filt, meta = load_recording(path)
    out = _outdir(args, path)
    notch = args.notch
    base = raw if raw is not None else filt
    dur = len(base) / fs
    hr = analyse_hr(base, fs, notch)

    summary = {
        "file": str(path), "duration_s": round(dur, 2), "fs": fs,
        "device_mode": meta.get("settings", {}).get("mode"),
        "rms_raw_dbfs": float(_db(np.mean(base ** 2))) if raw is not None else None,
        "rms_filtered_dbfs": float(_db(np.mean(filt ** 2))),
        "snr_heart_raw_db": dsp.snr_heart(raw, fs, notch) if raw is not None else None,
        "snr_heart_filtered_db": dsp.snr_heart(filt, fs, notch),
        "hr_autocorr_bpm": hr.get("hr_autocorr_bpm"),
        "hr_whole_recording_bpm": hr["hr_whole_bpm"],
        "hr_beat_interval_bpm": hr["hr_beats_bpm"],
        "hr_valid_fraction": hr.get("hr_valid_fraction"),
    }
    dev = [s.get("hr") for s in meta.get("status_log", []) if s.get("hr_ok")]
    if dev:
        summary["hr_device_median_bpm"] = float(np.median(dev))
    if args.ref_bpm:
        for k in ("hr_autocorr_bpm", "hr_whole_recording_bpm", "hr_beat_interval_bpm", "hr_device_median_bpm"):
            if summary.get(k):
                summary[k.replace("_bpm", "_error_bpm")] = round(summary[k] - args.ref_bpm, 2)
        summary["ref_bpm"] = args.ref_bpm
        if "hr_track_bpm" in hr:
            b = np.array(hr["hr_track_bpm"])[np.array(hr["hr_track_valid"], bool)]
            if len(b):
                summary["hr_track_mae_bpm"] = float(np.mean(np.abs(b - args.ref_bpm)))
                summary["hr_track_rmse_bpm"] = float(np.sqrt(np.mean((b - args.ref_bpm) ** 2)))

    plt = _plt()
    fig, ax = plt.subplots(4, 1, figsize=(11, 11))
    t0 = min(args.start, max(0, dur - args.span))
    sl = slice(int(t0 * fs), int((t0 + args.span) * fs))
    t = np.arange(len(base))[sl] / fs
    if raw is not None:
        ax[0].plot(t, raw[sl], lw=0.5)
    ax[0].set_title("Raw (unfiltered)" if raw is not None else "Raw not recorded (BLE)")
    ax[1].plot(t, filt[sl], lw=0.5, color="C3")
    ax[1].set_title(f"Filtered (device mode: {summary['device_mode']})")
    te = np.arange(len(hr["_env"])) / dsp.ENV_RATE
    me = (te >= t0) & (te < t0 + args.span)
    ax[2].plot(te[me], hr["_env"][me], color="C2", label="Shannon envelope")
    bt = hr["_beats"][(hr["_beats"] >= t0) & (hr["_beats"] < t0 + args.span)]
    ax[2].plot(bt, np.interp(bt, te, hr["_env"]), "kv", label="S1/S2 peaks")
    ax[2].legend(loc="upper right")
    ax[2].set_title("Envelope and detected heart sounds")
    tt, bb, vv = np.array(hr["hr_track_t"]), np.array(hr["hr_track_bpm"]), np.array(hr["hr_track_valid"], bool)
    if len(tt):
        ax[3].plot(tt[vv], bb[vv], ".", label="HR (8 s autocorrelation windows)")
    if args.ref_bpm:
        ax[3].axhline(args.ref_bpm, color="k", ls="--", label=f"reference {args.ref_bpm} bpm")
    ax[3].set_ylim(40, 200)
    ax[3].set_xlabel("time (s)")
    ax[3].set_ylabel("bpm")
    ax[3].legend(loc="upper right")
    ax[3].set_title("Heart rate")
    for a in ax:
        a.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "waveforms_hr.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 5))
    for x, lab in ((raw, "raw"), (filt, "filtered (device)")):
        if x is None:
            continue
        f, p = signal.welch(x, fs, nperseg=4096)
        ax.semilogx(f[1:], _db(p[1:]), label=lab)
    ax.set_xlim(5, fs / 2)
    ax.set_xlabel("frequency (Hz)")
    ax.set_ylabel("PSD (dB re FS²/Hz)")
    ax.set_title("Raw vs filtered spectrum")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "spectrum.png", dpi=150)
    plt.close(fig)

    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    _print(summary)
    print(f"\nplots + summary.json in {out}/")
    return 0


def cmd_tone(args) -> int:
    files = [Path(f) for f in args.files]
    freqs = args.freq * len(files) if len(args.freq) == 1 else args.freq
    if len(freqs) != len(files):
        print("give one --freq per file (or one for all)", file=sys.stderr)
        return 2
    rows = []
    for p, f0 in zip(files, freqs):
        fs, raw, filt, _ = load_recording(p)
        for name, x in (("raw", raw), ("filtered", filt)):
            if x is None:
                continue
            fw, pw = signal.welch(x, fs, nperseg=min(len(x), 4 * fs))
            k = np.abs(fw - f0) <= 5
            level = float(_db(np.sum(pw[k]) * (fw[1] - fw[0])))
            rows.append({"file": p.name, "f_hz": f0, "channel": name, "tone_level_dbfs": round(level, 2),
                         "snr_db": round(dsp.snr_tone(x, fs, f0, band=(10, 0.45 * fs)), 2)})
    for r in rows:
        print(f"{r['file']:30s} {r['f_hz']:7.1f} Hz  {r['channel']:9s} level {r['tone_level_dbfs']:7.2f} dBFS"
              f"   SNR {r['snr_db']:6.2f} dB")
    if len(set(freqs)) > 1:
        plt = _plt()
        fig, ax = plt.subplots(figsize=(8, 4))
        for ch in ("raw", "filtered"):
            rr = sorted([r for r in rows if r["channel"] == ch], key=lambda r: r["f_hz"])
            if rr:
                ax.semilogx([r["f_hz"] for r in rr], [r["tone_level_dbfs"] for r in rr], "o-", label=ch)
        ax.set_xlabel("test-tone frequency (Hz)")
        ax.set_ylabel("level (dBFS)")
        ax.set_title("Measured frequency response (same speaker level)")
        ax.grid(True, which="both", alpha=0.3)
        ax.legend()
        out = Path(args.out or files[0].parent)
        fig.tight_layout()
        fig.savefig(out / "tone_response.png", dpi=150)
        print(f"plot: {out / 'tone_response.png'}")
    if args.out:
        Path(args.out).mkdir(parents=True, exist_ok=True)
        (Path(args.out) / "tones.json").write_text(json.dumps(rows, indent=2))
    return 0


def cmd_noise(args) -> int:
    fs, rs, fsig, _ = load_recording(Path(args.signal))
    _, rn, fnoise, _ = load_recording(Path(args.noise))
    res = {"snr_filtered_db": dsp.snr_signal_vs_noise(fsig, fnoise)}
    if rs is not None and rn is not None:
        res["snr_raw_db"] = dsp.snr_signal_vs_noise(rs, rn)
    _print(res)
    return 0


def cmd_compare(args) -> int:
    ours_p, ref_p = Path(args.ours), Path(args.reference)
    fs, raw, filt, meta = load_recording(ours_p)
    ours = raw if (raw is not None and not args.use_filtered) else filt
    # Use the device's measured ADC rate: over minutes a 0.16 % rate error
    # shifts the waveforms by many cycles and destroys the correlation.
    fs_adc = [s_.get("fs_adc") for s_ in meta.get("status_log", []) if s_.get("fs_adc")]
    fs_true = float(np.median(fs_adc)) / dsp.DECIMATION if fs_adc and not args.nominal_fs else float(fs)
    print(f"  our sample rate used          {fs_true:.2f} Hz" + (" (measured)" if fs_true != fs else ""))
    rfs, ref = load_reference(ref_p, args.ref_fs, args.ref_col)
    band = (args.band_lo, args.band_hi)
    al = dsp.align_and_correlate(ours, fs_true, ref, rfs, fs_common=args.fs_common, band=band,
                                 max_lag_s=args.max_lag)
    ref_bp = signal.sosfiltfilt(signal.butter(4, band, btype="bandpass", fs=rfs, output="sos"), ref - np.mean(ref))
    res = {
        "ours": str(ours_p), "reference": str(ref_p), "ref_fs": rfs,
        "band_hz": band, "lag_s": round(al.lag_s, 4), "overlap_s": round(al.n_samples / args.fs_common, 1),
        "pearson_r_waveform": round(al.pearson_r, 4), "pearson_r_envelope": round(al.envelope_r, 4),
        "snr_heart_ours_db": round(dsp.snr_heart(ours, fs), 2),
        "snr_heart_reference_db": round(dsp.snr_heart(signal.resample_poly(ref_bp, int(fs), int(rfs))
                                                      if rfs != fs else ref_bp, fs), 2),
        "hr_ours_bpm": round(analyse_hr(ours, fs, args.notch)["hr_whole_bpm"], 1),
    }
    try:
        ref_fs_int = int(round(rfs))
        ref_4k = signal.resample_poly(ref_bp, int(fs), ref_fs_int) if ref_fs_int != fs else ref_bp
        res["hr_reference_bpm"] = round(analyse_hr(ref_4k, fs, 0.0)["hr_whole_bpm"], 1)
    except Exception:
        pass
    _print(res)

    plt = _plt()
    fc = args.fs_common
    g = np.gcd(int(fc), int(fs))
    a = signal.resample_poly(ours - ours.mean(), int(fc) // g, int(fs) // g)
    g2 = np.gcd(int(fc), int(round(rfs)))
    b = signal.resample_poly(ref - ref.mean(), int(fc) // g2, int(round(rfs)) // g2)
    sos = signal.butter(4, band, btype="bandpass", fs=fc, output="sos")
    a, b = signal.sosfiltfilt(sos, a), signal.sosfiltfilt(sos, b)
    lag = int(round(al.lag_s * fc))
    a, b = (a, b[lag:]) if lag >= 0 else (a[-lag:], b)
    n = min(len(a), len(b), int(args.span * fc))
    t = np.arange(n) / fc
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t, a[:n] / np.std(a[:n]), lw=0.6, label="our stethoscope")
    ax.plot(t, b[:n] / np.std(b[:n]) + 6, lw=0.6, label="BIOPAC (offset)")
    ax.set_title(f"Aligned recordings (lag {al.lag_s * 1e3:.1f} ms) - r = {al.pearson_r:.3f}, "
                 f"envelope r = {al.envelope_r:.3f}")
    ax.set_xlabel("time (s)")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    out = _outdir(args, ours_p)
    fig.tight_layout()
    fig.savefig(out / "compare_biopac.png", dpi=150)
    (out / "compare.json").write_text(json.dumps(res, indent=2, default=float))
    print(f"\nplot + compare.json in {out}/")
    return 0


def cmd_repeat(args) -> int:
    rows = []
    for f in args.files:
        fs, raw, filt, _ = load_recording(Path(f))
        x = raw if raw is not None else filt
        rows.append({"file": Path(f).name, "rms_dbfs": float(_db(np.mean(filt ** 2))),
                     "snr_db": dsp.snr_heart(filt, fs, args.notch),
                     "hr_bpm": analyse_hr(x, fs, args.notch)["hr_whole_bpm"]})
    for r in rows:
        print(f"{r['file']:30s} level {r['rms_dbfs']:7.2f} dBFS  SNR {r['snr_db']:6.2f} dB  HR {r['hr_bpm']:6.1f}")
    print()
    for k in ("rms_dbfs", "snr_db", "hr_bpm"):
        v = np.array([r[k] for r in rows])
        cv = 100 * np.std(v, ddof=1) / abs(np.mean(v)) if len(v) > 1 and np.mean(v) else float("nan")
        print(f"{k:10s} mean {np.mean(v):8.2f}   SD {np.std(v, ddof=1) if len(v) > 1 else 0:6.2f}   CV {cv:5.1f} %")
    return 0


def _print(d: dict) -> None:
    for k, v in d.items():
        if k.startswith("_"):
            continue
        if isinstance(v, float):
            v = round(v, 3)
        print(f"  {k:28s} {v}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("report", help="raw vs filtered, SNR, HR")
    p.add_argument("file")
    p.add_argument("--ref-bpm", type=float, help="reference HR for accuracy")
    p.add_argument("--notch", type=float, default=50.0)
    p.add_argument("--start", type=float, default=5.0, help="start of the plotted excerpt (s)")
    p.add_argument("--span", type=float, default=5.0, help="length of the plotted excerpt (s)")
    p.add_argument("--out")
    p.set_defaults(fn=cmd_report)

    p = sub.add_parser("tone", help="known-frequency test tones")
    p.add_argument("files", nargs="+")
    p.add_argument("--freq", type=float, nargs="+", required=True)
    p.add_argument("--out")
    p.set_defaults(fn=cmd_tone)

    p = sub.add_parser("noise", help="SNR from signal and noise-only recordings")
    p.add_argument("signal")
    p.add_argument("noise")
    p.set_defaults(fn=cmd_noise)

    p = sub.add_parser("compare", help="correlation with the BIOPAC stethoscope")
    p.add_argument("ours")
    p.add_argument("reference")
    p.add_argument("--ref-fs", type=float, help="reference sampling rate (if not in the file)")
    p.add_argument("--ref-col", type=int, help="reference data column / channel")
    p.add_argument("--use-filtered", action="store_true", help="use our filtered channel instead of raw")
    p.add_argument("--nominal-fs", action="store_true", help="ignore the measured ADC rate (use 4000 Hz)")
    p.add_argument("--band-lo", type=float, default=25.0)
    p.add_argument("--band-hi", type=float, default=400.0)
    p.add_argument("--fs-common", type=float, default=2000.0)
    p.add_argument("--max-lag", type=float, default=5.0, help="max start offset between files (s)")
    p.add_argument("--notch", type=float, default=50.0)
    p.add_argument("--span", type=float, default=6.0)
    p.add_argument("--out")
    p.set_defaults(fn=cmd_compare)

    p = sub.add_parser("repeat", help="repeatability over several recordings")
    p.add_argument("files", nargs="+")
    p.add_argument("--notch", type=float, default=50.0)
    p.set_defaults(fn=cmd_repeat)

    args = ap.parse_args()
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
